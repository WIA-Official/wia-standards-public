/**
 * WIA-QL Lexer
 *
 * Tokenizes WIA-QL source code into tokens for parsing.
 */

import { ParseError, SourceLocation } from '../types';

// ============================================================
// Token Types
// ============================================================

export enum TokenType {
    // Keywords
    QUANTUM = 'QUANTUM',
    PROGRAM = 'PROGRAM',
    QREG = 'QREG',
    CREG = 'CREG',
    IF = 'IF',
    ELSE = 'ELSE',
    FOR = 'FOR',
    IN = 'IN',
    RETURN = 'RETURN',
    MEASURE = 'MEASURE',

    // Gates (single qubit)
    I = 'I', X = 'X', Y = 'Y', Z = 'Z',
    H = 'H', S = 'S', T = 'T',
    SDG = 'SDG', TDG = 'TDG',
    RX = 'RX', RY = 'RY', RZ = 'RZ',
    U = 'U', P = 'P',

    // Gates (two qubit)
    CNOT = 'CNOT', CX = 'CX', CY = 'CY', CZ = 'CZ',
    SWAP = 'SWAP', ISWAP = 'ISWAP',
    CRX = 'CRX', CRY = 'CRY', CRZ = 'CRZ', CU = 'CU',

    // Gates (three qubit)
    CCNOT = 'CCNOT', CCX = 'CCX', TOFFOLI = 'TOFFOLI',
    CSWAP = 'CSWAP', FREDKIN = 'FREDKIN',

    // Identifiers and literals
    IDENTIFIER = 'IDENTIFIER',
    NUMBER = 'NUMBER',
    FLOAT = 'FLOAT',
    STRING = 'STRING',

    // Operators
    ARROW = 'ARROW',         // ->
    DOTDOT = 'DOTDOT',       // ..
    EQ = 'EQ',               // ==
    NEQ = 'NEQ',             // !=
    LT = 'LT',               // <
    GT = 'GT',               // >
    LTE = 'LTE',             // <=
    GTE = 'GTE',             // >=
    ASSIGN = 'ASSIGN',       // =

    // Punctuation
    LBRACE = 'LBRACE',       // {
    RBRACE = 'RBRACE',       // }
    LPAREN = 'LPAREN',       // (
    RPAREN = 'RPAREN',       // )
    LBRACKET = 'LBRACKET',   // [
    RBRACKET = 'RBRACKET',   // ]
    SEMICOLON = 'SEMICOLON', // ;
    COMMA = 'COMMA',         // ,
    COLON = 'COLON',         // :

    // Special
    EOF = 'EOF',
    NEWLINE = 'NEWLINE',
    COMMENT = 'COMMENT',
}

// ============================================================
// Token
// ============================================================

export interface Token {
    type: TokenType;
    value: string;
    location: SourceLocation;
}

// ============================================================
// Keywords Map
// ============================================================

const KEYWORDS: Record<string, TokenType> = {
    // Language keywords
    'quantum': TokenType.QUANTUM,
    'program': TokenType.PROGRAM,
    'qreg': TokenType.QREG,
    'creg': TokenType.CREG,
    'if': TokenType.IF,
    'else': TokenType.ELSE,
    'for': TokenType.FOR,
    'in': TokenType.IN,
    'return': TokenType.RETURN,
    'measure': TokenType.MEASURE,
    'measure_all': TokenType.MEASURE,

    // Single qubit gates
    'I': TokenType.I, 'X': TokenType.X, 'Y': TokenType.Y, 'Z': TokenType.Z,
    'H': TokenType.H, 'S': TokenType.S, 'T': TokenType.T,
    'Sdg': TokenType.SDG, 'Tdg': TokenType.TDG,
    'Rx': TokenType.RX, 'Ry': TokenType.RY, 'Rz': TokenType.RZ,
    'U': TokenType.U, 'P': TokenType.P,

    // Two qubit gates
    'CNOT': TokenType.CNOT, 'CX': TokenType.CX,
    'CY': TokenType.CY, 'CZ': TokenType.CZ,
    'SWAP': TokenType.SWAP, 'iSWAP': TokenType.ISWAP,
    'CRx': TokenType.CRX, 'CRy': TokenType.CRY, 'CRz': TokenType.CRZ,
    'CU': TokenType.CU,

    // Three qubit gates
    'CCNOT': TokenType.CCNOT, 'CCX': TokenType.CCX, 'Toffoli': TokenType.TOFFOLI,
    'CSWAP': TokenType.CSWAP, 'Fredkin': TokenType.FREDKIN,
};

// ============================================================
// Lexer Class
// ============================================================

export class Lexer {
    private source: string;
    private pos: number = 0;
    private line: number = 1;
    private column: number = 1;
    private tokens: Token[] = [];

    constructor(source: string) {
        this.source = source;
    }

    /**
     * Tokenize the entire source code
     */
    tokenize(): Token[] {
        while (!this.isAtEnd()) {
            this.scanToken();
        }

        this.tokens.push({
            type: TokenType.EOF,
            value: '',
            location: { line: this.line, column: this.column }
        });

        return this.tokens;
    }

    /**
     * Scan a single token
     */
    private scanToken(): void {
        this.skipWhitespace();

        if (this.isAtEnd()) return;

        const char = this.peek();

        // Comments
        if (char === '/' && this.peekNext() === '/') {
            this.skipLineComment();
            return;
        }

        // Strings
        if (char === '"') {
            this.scanString();
            return;
        }

        // Numbers
        if (this.isDigit(char) || (char === '-' && this.isDigit(this.peekNext()))) {
            this.scanNumber();
            return;
        }

        // Identifiers and keywords
        if (this.isAlpha(char)) {
            this.scanIdentifier();
            return;
        }

        // Operators and punctuation
        this.scanOperator();
    }

    /**
     * Scan an identifier or keyword
     */
    private scanIdentifier(): void {
        const start = this.pos;
        const startColumn = this.column;

        while (!this.isAtEnd() && this.isAlphaNumeric(this.peek())) {
            this.advance();
        }

        const value = this.source.slice(start, this.pos);
        const type = KEYWORDS[value] || TokenType.IDENTIFIER;

        this.tokens.push({
            type,
            value,
            location: { line: this.line, column: startColumn }
        });
    }

    /**
     * Scan a number (integer or float)
     */
    private scanNumber(): void {
        const startColumn = this.column;
        let value = '';
        let isFloat = false;

        // Handle negative numbers
        if (this.peek() === '-') {
            value += this.advance();
        }

        // Integer part
        while (!this.isAtEnd() && this.isDigit(this.peek())) {
            value += this.advance();
        }

        // Decimal part
        if (this.peek() === '.' && this.isDigit(this.peekNext())) {
            isFloat = true;
            value += this.advance(); // consume '.'
            while (!this.isAtEnd() && this.isDigit(this.peek())) {
                value += this.advance();
            }
        }

        // Scientific notation
        if (this.peek() === 'e' || this.peek() === 'E') {
            isFloat = true;
            value += this.advance();
            if (this.peek() === '+' || this.peek() === '-') {
                value += this.advance();
            }
            while (!this.isAtEnd() && this.isDigit(this.peek())) {
                value += this.advance();
            }
        }

        this.tokens.push({
            type: isFloat ? TokenType.FLOAT : TokenType.NUMBER,
            value,
            location: { line: this.line, column: startColumn }
        });
    }

    /**
     * Scan a string literal
     */
    private scanString(): void {
        const startColumn = this.column;
        this.advance(); // consume opening quote

        let value = '';
        while (!this.isAtEnd() && this.peek() !== '"') {
            if (this.peek() === '\n') {
                throw new ParseError(
                    'Unterminated string literal',
                    { line: this.line, column: this.column }
                );
            }
            if (this.peek() === '\\') {
                this.advance();
                value += this.scanEscapeSequence();
            } else {
                value += this.advance();
            }
        }

        if (this.isAtEnd()) {
            throw new ParseError(
                'Unterminated string literal',
                { line: this.line, column: startColumn }
            );
        }

        this.advance(); // consume closing quote

        this.tokens.push({
            type: TokenType.STRING,
            value,
            location: { line: this.line, column: startColumn }
        });
    }

    /**
     * Scan escape sequence in string
     */
    private scanEscapeSequence(): string {
        const char = this.advance();
        switch (char) {
            case 'n': return '\n';
            case 't': return '\t';
            case 'r': return '\r';
            case '\\': return '\\';
            case '"': return '"';
            default: return char;
        }
    }

    /**
     * Scan operator or punctuation
     */
    private scanOperator(): void {
        const startColumn = this.column;
        const char = this.advance();

        let type: TokenType;
        let value = char;

        switch (char) {
            case '{': type = TokenType.LBRACE; break;
            case '}': type = TokenType.RBRACE; break;
            case '(': type = TokenType.LPAREN; break;
            case ')': type = TokenType.RPAREN; break;
            case '[': type = TokenType.LBRACKET; break;
            case ']': type = TokenType.RBRACKET; break;
            case ';': type = TokenType.SEMICOLON; break;
            case ',': type = TokenType.COMMA; break;
            case ':': type = TokenType.COLON; break;
            case '-':
                if (this.peek() === '>') {
                    this.advance();
                    type = TokenType.ARROW;
                    value = '->';
                } else {
                    throw new ParseError(
                        `Unexpected character: ${char}`,
                        { line: this.line, column: startColumn }
                    );
                }
                break;
            case '.':
                if (this.peek() === '.') {
                    this.advance();
                    type = TokenType.DOTDOT;
                    value = '..';
                } else {
                    throw new ParseError(
                        `Unexpected character: ${char}`,
                        { line: this.line, column: startColumn }
                    );
                }
                break;
            case '=':
                if (this.peek() === '=') {
                    this.advance();
                    type = TokenType.EQ;
                    value = '==';
                } else {
                    type = TokenType.ASSIGN;
                }
                break;
            case '!':
                if (this.peek() === '=') {
                    this.advance();
                    type = TokenType.NEQ;
                    value = '!=';
                } else {
                    throw new ParseError(
                        `Unexpected character: ${char}`,
                        { line: this.line, column: startColumn }
                    );
                }
                break;
            case '<':
                if (this.peek() === '=') {
                    this.advance();
                    type = TokenType.LTE;
                    value = '<=';
                } else {
                    type = TokenType.LT;
                }
                break;
            case '>':
                if (this.peek() === '=') {
                    this.advance();
                    type = TokenType.GTE;
                    value = '>=';
                } else {
                    type = TokenType.GT;
                }
                break;
            default:
                throw new ParseError(
                    `Unexpected character: ${char}`,
                    { line: this.line, column: startColumn }
                );
        }

        this.tokens.push({
            type,
            value,
            location: { line: this.line, column: startColumn }
        });
    }

    /**
     * Skip whitespace characters
     */
    private skipWhitespace(): void {
        while (!this.isAtEnd()) {
            const char = this.peek();
            if (char === ' ' || char === '\t' || char === '\r') {
                this.advance();
            } else if (char === '\n') {
                this.line++;
                this.column = 1;
                this.pos++;
            } else {
                break;
            }
        }
    }

    /**
     * Skip line comment
     */
    private skipLineComment(): void {
        while (!this.isAtEnd() && this.peek() !== '\n') {
            this.advance();
        }
    }

    // ============================================================
    // Helper Methods
    // ============================================================

    private isAtEnd(): boolean {
        return this.pos >= this.source.length;
    }

    private peek(): string {
        return this.source[this.pos] || '\0';
    }

    private peekNext(): string {
        return this.source[this.pos + 1] || '\0';
    }

    private advance(): string {
        this.column++;
        return this.source[this.pos++];
    }

    private isDigit(char: string): boolean {
        return char >= '0' && char <= '9';
    }

    private isAlpha(char: string): boolean {
        return (char >= 'a' && char <= 'z') ||
               (char >= 'A' && char <= 'Z') ||
               char === '_';
    }

    private isAlphaNumeric(char: string): boolean {
        return this.isAlpha(char) || this.isDigit(char);
    }
}
