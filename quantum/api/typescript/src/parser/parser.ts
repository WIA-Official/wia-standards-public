/**
 * WIA-QL Parser
 *
 * Parses tokens into an Abstract Syntax Tree (AST).
 */

import { Lexer, Token, TokenType } from './lexer';
import {
    ProgramNode,
    StatementNode,
    RegisterDeclaration,
    GateApplication,
    Measurement,
    ConditionalStatement,
    ForLoop,
    Return,
    QubitRef,
    Parameter,
    GateType,
    ParseError,
    SourceLocation,
} from '../types';

// ============================================================
// Gate Categories for Parsing
// ============================================================

const SINGLE_QUBIT_GATES = new Set([
    'I', 'X', 'Y', 'Z', 'H', 'S', 'T', 'SDG', 'TDG'
]);

const PARAMETERIZED_SINGLE_QUBIT_GATES = new Set([
    'RX', 'RY', 'RZ', 'U', 'P'
]);

const TWO_QUBIT_GATES = new Set([
    'CNOT', 'CX', 'CY', 'CZ', 'SWAP', 'ISWAP'
]);

const PARAMETERIZED_TWO_QUBIT_GATES = new Set([
    'CRX', 'CRY', 'CRZ', 'CU'
]);

const THREE_QUBIT_GATES = new Set([
    'CCNOT', 'CCX', 'TOFFOLI', 'CSWAP', 'FREDKIN'
]);

// ============================================================
// Parser Class
// ============================================================

export class Parser {
    private tokens: Token[] = [];
    private current: number = 0;

    /**
     * Parse WIA-QL source code into AST
     */
    parse(source: string): ProgramNode {
        const lexer = new Lexer(source);
        this.tokens = lexer.tokenize();
        this.current = 0;

        return this.parseProgram();
    }

    /**
     * Parse program: quantum program Name { ... }
     */
    private parseProgram(): ProgramNode {
        this.expect(TokenType.QUANTUM, 'Expected "quantum" keyword');
        this.expect(TokenType.PROGRAM, 'Expected "program" keyword');

        const nameToken = this.expect(TokenType.IDENTIFIER, 'Expected program name');
        const name = nameToken.value;

        this.expect(TokenType.LBRACE, 'Expected "{"');

        const body: StatementNode[] = [];
        while (!this.check(TokenType.RBRACE) && !this.isAtEnd()) {
            body.push(this.parseStatement());
        }

        this.expect(TokenType.RBRACE, 'Expected "}"');

        return {
            type: 'Program',
            name,
            body,
            location: { line: 1, column: 1 }
        };
    }

    /**
     * Parse a single statement
     */
    private parseStatement(): StatementNode {
        // Register declaration
        if (this.check(TokenType.QREG)) {
            return this.parseRegisterDeclaration('quantum');
        }
        if (this.check(TokenType.CREG)) {
            return this.parseRegisterDeclaration('classical');
        }

        // Conditional
        if (this.check(TokenType.IF)) {
            return this.parseConditional();
        }

        // For loop
        if (this.check(TokenType.FOR)) {
            return this.parseForLoop();
        }

        // Return
        if (this.check(TokenType.RETURN)) {
            return this.parseReturn();
        }

        // Measurement
        if (this.check(TokenType.MEASURE)) {
            return this.parseMeasurement();
        }

        // Gate application or function call
        return this.parseGateOrCall();
    }

    /**
     * Parse register declaration: qreg name[size]; or creg name[size];
     */
    private parseRegisterDeclaration(regType: 'quantum' | 'classical'): RegisterDeclaration {
        const location = this.peek().location;
        this.advance(); // consume qreg/creg

        const nameToken = this.expect(TokenType.IDENTIFIER, 'Expected register name');
        this.expect(TokenType.LBRACKET, 'Expected "["');
        const sizeToken = this.expect(TokenType.NUMBER, 'Expected register size');
        this.expect(TokenType.RBRACKET, 'Expected "]"');
        this.expect(TokenType.SEMICOLON, 'Expected ";"');

        return {
            type: 'RegisterDeclaration',
            registerType: regType,
            name: nameToken.value,
            size: parseInt(sizeToken.value, 10),
            location
        };
    }

    /**
     * Parse gate application: H(q[0]); CNOT(q[0], q[1]); Rx(0.5, q[0]);
     */
    private parseGateOrCall(): GateApplication {
        const location = this.peek().location;
        const gateToken = this.advance();
        const gateName = gateToken.value.toUpperCase();

        this.expect(TokenType.LPAREN, 'Expected "(" after gate name');

        // Check if this is a parameterized gate
        const params: Parameter[] = [];
        const qubits: QubitRef[] = [];

        // Parse parameters and qubits
        if (!this.check(TokenType.RPAREN)) {
            do {
                // Check if it's a number (parameter) or identifier (qubit)
                if (this.check(TokenType.NUMBER) || this.check(TokenType.FLOAT)) {
                    const paramToken = this.advance();
                    params.push(parseFloat(paramToken.value));
                } else if (this.check(TokenType.IDENTIFIER)) {
                    qubits.push(this.parseQubitRef());
                } else {
                    throw new ParseError(
                        'Expected parameter or qubit reference',
                        this.peek().location
                    );
                }
            } while (this.match(TokenType.COMMA));
        }

        this.expect(TokenType.RPAREN, 'Expected ")"');
        this.expect(TokenType.SEMICOLON, 'Expected ";"');

        return {
            type: 'GateApplication',
            gate: this.normalizeGateName(gateName) as GateType,
            qubits,
            parameters: params.length > 0 ? params : undefined,
            location
        };
    }

    /**
     * Parse qubit reference: q[0] or q
     */
    private parseQubitRef(): QubitRef {
        const nameToken = this.expect(TokenType.IDENTIFIER, 'Expected register name');
        const ref: QubitRef = { register: nameToken.value };

        if (this.match(TokenType.LBRACKET)) {
            const indexToken = this.expect(TokenType.NUMBER, 'Expected qubit index');
            ref.index = parseInt(indexToken.value, 10);
            this.expect(TokenType.RBRACKET, 'Expected "]"');
        }

        return ref;
    }

    /**
     * Parse measurement: measure q -> c; or measure q[0] -> c[0];
     */
    private parseMeasurement(): Measurement {
        const location = this.peek().location;
        this.advance(); // consume 'measure'

        // Check for measure_all
        if (this.previous().value === 'measure_all') {
            this.expect(TokenType.LPAREN, 'Expected "("');
            const qubits = [this.parseQubitRef()];
            this.expect(TokenType.RPAREN, 'Expected ")"');
            this.expect(TokenType.SEMICOLON, 'Expected ";"');

            return {
                type: 'Measurement',
                qubits,
                measureAll: true,
                location
            };
        }

        const qubits: QubitRef[] = [this.parseQubitRef()];

        // Optional classical target
        let classical: QubitRef[] | undefined;
        if (this.match(TokenType.ARROW)) {
            classical = [this.parseQubitRef()];
        }

        this.expect(TokenType.SEMICOLON, 'Expected ";"');

        return {
            type: 'Measurement',
            qubits,
            classical,
            location
        };
    }

    /**
     * Parse conditional: if (c[0] == 1) { ... } else { ... }
     */
    private parseConditional(): ConditionalStatement {
        const location = this.peek().location;
        this.advance(); // consume 'if'

        this.expect(TokenType.LPAREN, 'Expected "("');

        // Parse condition
        const leftToken = this.expect(TokenType.IDENTIFIER, 'Expected variable');
        let left = leftToken.value;

        // Handle array index
        if (this.match(TokenType.LBRACKET)) {
            const indexToken = this.expect(TokenType.NUMBER, 'Expected index');
            left += `[${indexToken.value}]`;
            this.expect(TokenType.RBRACKET, 'Expected "]"');
        }

        const opToken = this.advance();
        let operator: '==' | '!=' | '<' | '>' | '<=' | '>=';
        switch (opToken.type) {
            case TokenType.EQ: operator = '=='; break;
            case TokenType.NEQ: operator = '!='; break;
            case TokenType.LT: operator = '<'; break;
            case TokenType.GT: operator = '>'; break;
            case TokenType.LTE: operator = '<='; break;
            case TokenType.GTE: operator = '>='; break;
            default:
                throw new ParseError(
                    'Expected comparison operator',
                    opToken.location
                );
        }

        const rightToken = this.expect(TokenType.NUMBER, 'Expected number');
        const right = parseInt(rightToken.value, 10);

        this.expect(TokenType.RPAREN, 'Expected ")"');
        this.expect(TokenType.LBRACE, 'Expected "{"');

        const thenBody: StatementNode[] = [];
        while (!this.check(TokenType.RBRACE) && !this.isAtEnd()) {
            thenBody.push(this.parseStatement());
        }
        this.expect(TokenType.RBRACE, 'Expected "}"');

        let elseBody: StatementNode[] | undefined;
        if (this.match(TokenType.ELSE)) {
            this.expect(TokenType.LBRACE, 'Expected "{"');
            elseBody = [];
            while (!this.check(TokenType.RBRACE) && !this.isAtEnd()) {
                elseBody.push(this.parseStatement());
            }
            this.expect(TokenType.RBRACE, 'Expected "}"');
        }

        return {
            type: 'ConditionalStatement',
            condition: { left, operator, right },
            thenBody,
            elseBody,
            location
        };
    }

    /**
     * Parse for loop: for i in 0..4 { ... }
     */
    private parseForLoop(): ForLoop {
        const location = this.peek().location;
        this.advance(); // consume 'for'

        const varToken = this.expect(TokenType.IDENTIFIER, 'Expected loop variable');
        this.expect(TokenType.IN, 'Expected "in"');

        const startToken = this.expect(TokenType.NUMBER, 'Expected start value');
        this.expect(TokenType.DOTDOT, 'Expected ".."');
        const endToken = this.expect(TokenType.NUMBER, 'Expected end value');

        this.expect(TokenType.LBRACE, 'Expected "{"');

        const body: StatementNode[] = [];
        while (!this.check(TokenType.RBRACE) && !this.isAtEnd()) {
            body.push(this.parseStatement());
        }
        this.expect(TokenType.RBRACE, 'Expected "}"');

        return {
            type: 'ForLoop',
            variable: varToken.value,
            start: parseInt(startToken.value, 10),
            end: parseInt(endToken.value, 10),
            body,
            location
        };
    }

    /**
     * Parse return statement: return measure_all(q);
     */
    private parseReturn(): Return {
        const location = this.peek().location;
        this.advance(); // consume 'return'

        let value: Measurement | string | undefined;

        if (this.check(TokenType.MEASURE)) {
            value = this.parseMeasurement();
            // Remove the semicolon expectation since measurement already consumed it
            return { type: 'Return', value, location };
        } else if (this.check(TokenType.IDENTIFIER)) {
            value = this.advance().value;
        }

        this.expect(TokenType.SEMICOLON, 'Expected ";"');

        return {
            type: 'Return',
            value,
            location
        };
    }

    // ============================================================
    // Helper Methods
    // ============================================================

    private normalizeGateName(name: string): string {
        const mapping: Record<string, string> = {
            'CX': 'CNOT',
            'CCX': 'CCNOT',
        };
        return mapping[name] || name;
    }

    private isAtEnd(): boolean {
        return this.peek().type === TokenType.EOF;
    }

    private peek(): Token {
        return this.tokens[this.current];
    }

    private previous(): Token {
        return this.tokens[this.current - 1];
    }

    private advance(): Token {
        if (!this.isAtEnd()) this.current++;
        return this.previous();
    }

    private check(type: TokenType): boolean {
        if (this.isAtEnd()) return false;
        return this.peek().type === type;
    }

    private match(...types: TokenType[]): boolean {
        for (const type of types) {
            if (this.check(type)) {
                this.advance();
                return true;
            }
        }
        return false;
    }

    private expect(type: TokenType, message: string): Token {
        if (this.check(type)) return this.advance();
        throw new ParseError(message, this.peek().location);
    }
}

/**
 * Parse WIA-QL source code into AST
 */
export function parse(source: string): ProgramNode {
    const parser = new Parser();
    return parser.parse(source);
}
