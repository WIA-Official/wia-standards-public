/**
 * WIA-QIR Generator
 *
 * Converts AST to Quantum Intermediate Representation.
 */

import {
    ProgramNode,
    StatementNode,
    RegisterDeclaration,
    GateApplication,
    Measurement,
    ConditionalStatement,
    ForLoop,
    Return,
    QIRProgram,
    QIRInstruction,
    QIRMetadata,
    QIRRegisters,
    QubitRef,
    SemanticError,
} from '../types';

// ============================================================
// QIR Generator Class
// ============================================================

export class QIRGenerator {
    private quantumRegs: Map<string, number> = new Map();
    private classicalRegs: Map<string, number> = new Map();
    private instructions: QIRInstruction[] = [];

    /**
     * Generate QIR from AST
     */
    generate(ast: ProgramNode): QIRProgram {
        this.quantumRegs.clear();
        this.classicalRegs.clear();
        this.instructions = [];

        // First pass: collect register declarations
        for (const stmt of ast.body) {
            if (stmt.type === 'RegisterDeclaration') {
                this.processRegisterDeclaration(stmt);
            }
        }

        // Second pass: generate instructions
        for (const stmt of ast.body) {
            if (stmt.type !== 'RegisterDeclaration') {
                this.processStatement(stmt);
            }
        }

        // Calculate total qubits and bits
        let totalQubits = 0;
        let totalBits = 0;
        for (const size of this.quantumRegs.values()) {
            totalQubits += size;
        }
        for (const size of this.classicalRegs.values()) {
            totalBits += size;
        }

        return {
            version: '1.0',
            program: ast.name,
            metadata: {
                created: new Date().toISOString(),
                targetQubits: totalQubits,
                targetClassicalBits: totalBits,
            },
            registers: {
                quantum: Array.from(this.quantumRegs.entries()).map(([name, size]) => ({
                    name,
                    size,
                })),
                classical: Array.from(this.classicalRegs.entries()).map(([name, size]) => ({
                    name,
                    size,
                })),
            },
            instructions: this.instructions,
        };
    }

    /**
     * Process register declaration
     */
    private processRegisterDeclaration(decl: RegisterDeclaration): void {
        if (decl.registerType === 'quantum') {
            if (this.quantumRegs.has(decl.name)) {
                throw new SemanticError(
                    `Quantum register '${decl.name}' already declared`,
                    decl.location
                );
            }
            this.quantumRegs.set(decl.name, decl.size);
        } else {
            if (this.classicalRegs.has(decl.name)) {
                throw new SemanticError(
                    `Classical register '${decl.name}' already declared`,
                    decl.location
                );
            }
            this.classicalRegs.set(decl.name, decl.size);
        }
    }

    /**
     * Process a statement
     */
    private processStatement(stmt: StatementNode): void {
        switch (stmt.type) {
            case 'GateApplication':
                this.processGateApplication(stmt);
                break;
            case 'Measurement':
                this.processMeasurement(stmt);
                break;
            case 'ConditionalStatement':
                this.processConditional(stmt);
                break;
            case 'ForLoop':
                this.processForLoop(stmt);
                break;
            case 'Return':
                this.processReturn(stmt);
                break;
        }
    }

    /**
     * Process gate application
     */
    private processGateApplication(gate: GateApplication): void {
        const qubits = gate.qubits.map(q => this.formatQubitRef(q));
        const params = gate.parameters?.map(p =>
            typeof p === 'number' ? p : 0
        ) || [];

        this.instructions.push({
            op: gate.gate,
            qubits,
            params,
        });
    }

    /**
     * Process measurement
     */
    private processMeasurement(meas: Measurement): void {
        const qubits = meas.qubits.map(q => this.formatQubitRef(q));
        const classical = meas.classical?.map(c => this.formatQubitRef(c));

        if (meas.measureAll) {
            // Measure all qubits in the register
            const regName = meas.qubits[0].register;
            const regSize = this.quantumRegs.get(regName);
            if (regSize === undefined) {
                throw new SemanticError(`Unknown quantum register: ${regName}`);
            }

            for (let i = 0; i < regSize; i++) {
                this.instructions.push({
                    op: 'MEASURE',
                    qubits: [`${regName}[${i}]`],
                    params: [],
                    classical: classical ? [`${classical[0]}[${i}]`] : undefined,
                });
            }
        } else {
            this.instructions.push({
                op: 'MEASURE',
                qubits,
                params: [],
                classical,
            });
        }
    }

    /**
     * Process conditional statement
     */
    private processConditional(cond: ConditionalStatement): void {
        // For now, we inline conditional gates with condition metadata
        // A more sophisticated approach would use branching in QIR

        const conditionValue = cond.condition.right;
        const conditionReg = cond.condition.left.replace(/\[(\d+)\]/, '');

        for (const stmt of cond.thenBody) {
            const prevLength = this.instructions.length;
            this.processStatement(stmt);

            // Add condition to all generated instructions
            for (let i = prevLength; i < this.instructions.length; i++) {
                this.instructions[i].condition = {
                    register: conditionReg,
                    value: conditionValue,
                };
            }
        }

        // Note: else body would need inverse condition
        // Simplified here for demonstration
    }

    /**
     * Process for loop (unroll)
     */
    private processForLoop(loop: ForLoop): void {
        // Unroll the loop at compile time
        for (let i = loop.start; i < loop.end; i++) {
            for (const stmt of loop.body) {
                // Replace loop variable with current value
                const expandedStmt = this.expandLoopVariable(stmt, loop.variable, i);
                this.processStatement(expandedStmt);
            }
        }
    }

    /**
     * Expand loop variable in statement
     */
    private expandLoopVariable(
        stmt: StatementNode,
        variable: string,
        value: number
    ): StatementNode {
        if (stmt.type === 'GateApplication') {
            return {
                ...stmt,
                qubits: stmt.qubits.map(q => ({
                    register: q.register,
                    index: q.index === variable ? value :
                           typeof q.index === 'string' ? parseInt(q.index, 10) : q.index,
                })),
            };
        }
        return stmt;
    }

    /**
     * Process return statement
     */
    private processReturn(ret: Return): void {
        if (ret.value && typeof ret.value !== 'string') {
            this.processMeasurement(ret.value);
        }
    }

    /**
     * Format qubit reference as string
     */
    private formatQubitRef(ref: QubitRef): string {
        if (ref.index !== undefined) {
            return `${ref.register}[${ref.index}]`;
        }
        return ref.register;
    }
}

/**
 * Generate QIR from AST
 */
export function generateQIR(ast: ProgramNode): QIRProgram {
    const generator = new QIRGenerator();
    return generator.generate(ast);
}
