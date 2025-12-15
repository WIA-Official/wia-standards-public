/**
 * WIA-QUANTUM TypeScript SDK
 *
 * A unified quantum computing standard that abstracts across
 * IBM Qiskit, Google Cirq, Microsoft Q#, Amazon Braket, and Rigetti PyQuil.
 *
 * 홍익인간 (弘益人間) - Benefit All Humanity
 *
 * @version 1.0.0
 * @author World Certification Industry Association (WIA)
 * @license MIT
 */

// ============================================================
// Type Exports
// ============================================================

export * from './types';

// ============================================================
// Parser Exports
// ============================================================

export { Lexer } from './parser/lexer';
export { Parser } from './parser/parser';

// ============================================================
// IR Exports
// ============================================================

export { QIRGenerator, generateQIR } from './ir/generator';

// ============================================================
// Backend Exports
// ============================================================

export {
    QiskitBackend,
    generateQiskit,
    CirqBackend,
    generateCirq,
    QSharpBackend,
    generateQSharp,
    BraketBackend,
    generateBraket,
    PyQuilBackend,
    generatePyQuil,
    generateCode,
    generateAll,
} from './backends';

// ============================================================
// High-Level API
// ============================================================

import { Lexer } from './parser/lexer';
import { Parser } from './parser/parser';
import { generateQIR } from './ir/generator';
import { generateCode, generateAll } from './backends';
import {
    WiaQLProgram,
    QIRProgram,
    CompilationResult,
    BackendTarget,
    CompilerOptions,
    CompilerError,
} from './types';

/**
 * WIA-QUANTUM Compiler
 *
 * Main entry point for compiling WIA-QL source code to various
 * quantum computing platform backends.
 */
export class WiaQuantumCompiler {
    private options: CompilerOptions;

    constructor(options: Partial<CompilerOptions> = {}) {
        this.options = {
            target: options.target || 'qiskit',
            optimize: options.optimize ?? true,
            includeComments: options.includeComments ?? true,
            validateCircuit: options.validateCircuit ?? true,
        };
    }

    /**
     * Parse WIA-QL source code to AST
     */
    parse(source: string): WiaQLProgram {
        const lexer = new Lexer(source);
        const tokens = lexer.tokenize();
        const parser = new Parser(tokens);
        return parser.parse();
    }

    /**
     * Generate QIR from AST
     */
    toQIR(ast: WiaQLProgram): QIRProgram {
        return generateQIR(ast);
    }

    /**
     * Compile WIA-QL source to target backend code
     */
    compile(source: string, target?: BackendTarget): CompilationResult {
        const targetPlatform = target || this.options.target;

        // Parse source
        const ast = this.parse(source);

        // Generate QIR
        const qir = this.toQIR(ast);

        // Generate target code
        return generateCode(qir, targetPlatform, {
            includeComments: this.options.includeComments,
        });
    }

    /**
     * Compile WIA-QL source to all supported backends
     */
    compileAll(source: string): Map<BackendTarget, CompilationResult> {
        const ast = this.parse(source);
        const qir = this.toQIR(ast);
        return generateAll(qir, {
            includeComments: this.options.includeComments,
        });
    }

    /**
     * Quick compile - one line convenience function
     */
    static compile(
        source: string,
        target: BackendTarget = 'qiskit'
    ): CompilationResult {
        const compiler = new WiaQuantumCompiler({ target });
        return compiler.compile(source);
    }
}

/**
 * Quick compile function
 *
 * @example
 * const result = compile(`
 *   program BellState
 *   qreg q[2]
 *   creg c[2]
 *   H q[0]
 *   CNOT q[0], q[1]
 *   measure q -> c
 * `, 'qiskit');
 * console.log(result.code);
 */
export function compile(
    source: string,
    target: BackendTarget = 'qiskit'
): CompilationResult {
    return WiaQuantumCompiler.compile(source, target);
}

/**
 * Version information
 */
export const VERSION = '1.0.0';
export const WIA_QUANTUM_VERSION = '1.0';

// ============================================================
// Default Export
// ============================================================

export default WiaQuantumCompiler;
