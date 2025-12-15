/**
 * WIA-QUANTUM Type Definitions
 *
 * Core types for the WIA Quantum Language (WIA-QL) and
 * Quantum Intermediate Representation (WIA-QIR)
 *
 * @packageDocumentation
 */

// ============================================================
// Basic Types
// ============================================================

/** Qubit index type */
export type QubitIndex = number;

/** Classical bit index type */
export type BitIndex = number;

/** Parameter value (for rotation gates) */
export type Parameter = number | string; // number for literal, string for variable

// ============================================================
// Quantum Gates
// ============================================================

/** Standard single-qubit gates */
export type SingleQubitGate =
    | 'I' | 'X' | 'Y' | 'Z'
    | 'H' | 'S' | 'T' | 'Sdg' | 'Tdg';

/** Parameterized single-qubit gates */
export type ParameterizedSingleQubitGate =
    | 'Rx' | 'Ry' | 'Rz' | 'U' | 'P';

/** Two-qubit gates */
export type TwoQubitGate =
    | 'CNOT' | 'CX' | 'CY' | 'CZ'
    | 'SWAP' | 'iSWAP';

/** Parameterized two-qubit gates */
export type ParameterizedTwoQubitGate =
    | 'CRx' | 'CRy' | 'CRz' | 'CU';

/** Three-qubit gates */
export type ThreeQubitGate =
    | 'CCNOT' | 'CCX' | 'Toffoli'
    | 'CSWAP' | 'Fredkin';

/** All gate types */
export type GateType =
    | SingleQubitGate
    | ParameterizedSingleQubitGate
    | TwoQubitGate
    | ParameterizedTwoQubitGate
    | ThreeQubitGate;

// ============================================================
// AST (Abstract Syntax Tree) Nodes
// ============================================================

/** Base AST node */
export interface ASTNode {
    type: string;
    location?: SourceLocation;
}

/** Source location for error reporting */
export interface SourceLocation {
    line: number;
    column: number;
    file?: string;
}

/** Program node (root) */
export interface ProgramNode extends ASTNode {
    type: 'Program';
    name: string;
    body: StatementNode[];
}

/** Statement types */
export type StatementNode =
    | RegisterDeclaration
    | GateApplication
    | Measurement
    | ConditionalStatement
    | ForLoop
    | FunctionCall
    | Return;

/** Quantum register declaration */
export interface RegisterDeclaration extends ASTNode {
    type: 'RegisterDeclaration';
    registerType: 'quantum' | 'classical';
    name: string;
    size: number;
}

/** Gate application */
export interface GateApplication extends ASTNode {
    type: 'GateApplication';
    gate: GateType;
    qubits: QubitRef[];
    parameters?: Parameter[];
}

/** Qubit reference (register[index] or just register) */
export interface QubitRef {
    register: string;
    index?: number | string; // number for literal, string for variable
}

/** Measurement operation */
export interface Measurement extends ASTNode {
    type: 'Measurement';
    qubits: QubitRef[];
    classical?: QubitRef[];
    measureAll?: boolean;
}

/** Conditional statement (if) */
export interface ConditionalStatement extends ASTNode {
    type: 'ConditionalStatement';
    condition: Condition;
    thenBody: StatementNode[];
    elseBody?: StatementNode[];
}

/** Condition for if statements */
export interface Condition {
    left: string;
    operator: '==' | '!=' | '<' | '>' | '<=' | '>=';
    right: number | string;
}

/** For loop */
export interface ForLoop extends ASTNode {
    type: 'ForLoop';
    variable: string;
    start: number;
    end: number;
    body: StatementNode[];
}

/** Function call */
export interface FunctionCall extends ASTNode {
    type: 'FunctionCall';
    name: string;
    arguments: (QubitRef | Parameter)[];
}

/** Return statement */
export interface Return extends ASTNode {
    type: 'Return';
    value?: Measurement | string;
}

// ============================================================
// WIA-QIR (Intermediate Representation)
// ============================================================

/** QIR Program */
export interface QIRProgram {
    version: string;
    program: string;
    metadata: QIRMetadata;
    registers: QIRRegisters;
    instructions: QIRInstruction[];
}

/** QIR Metadata */
export interface QIRMetadata {
    author?: string;
    created: string;
    targetQubits: number;
    targetClassicalBits: number;
    description?: string;
}

/** QIR Registers */
export interface QIRRegisters {
    quantum: QIRRegister[];
    classical: QIRRegister[];
}

/** QIR Register */
export interface QIRRegister {
    name: string;
    size: number;
}

/** QIR Instruction */
export interface QIRInstruction {
    op: string;
    qubits: string[];
    params: number[];
    classical?: string[];
    condition?: QIRCondition;
}

/** QIR Condition */
export interface QIRCondition {
    register: string;
    value: number;
}

// ============================================================
// Backend Output Types
// ============================================================

/** Supported backend targets */
export type BackendTarget =
    | 'qiskit'
    | 'cirq'
    | 'qsharp'
    | 'braket'
    | 'pyquil'
    | 'openqasm';

/** Backend compilation result */
export interface CompilationResult {
    target: BackendTarget;
    code: string;
    sourceMap?: SourceMap;
    warnings: CompilerWarning[];
    stats: CompilationStats;
}

/** Source map for debugging */
export interface SourceMap {
    mappings: SourceMapping[];
}

/** Source mapping entry */
export interface SourceMapping {
    originalLine: number;
    generatedLine: number;
    originalColumn?: number;
    generatedColumn?: number;
}

/** Compiler warning */
export interface CompilerWarning {
    message: string;
    location?: SourceLocation;
    severity: 'info' | 'warning' | 'error';
}

/** Compilation statistics */
export interface CompilationStats {
    totalGates: number;
    singleQubitGates: number;
    twoQubitGates: number;
    threeQubitGates: number;
    measurements: number;
    circuitDepth: number;
    qubitCount: number;
}

// ============================================================
// Compiler Options
// ============================================================

/** Compiler configuration options */
export interface CompilerOptions {
    target: BackendTarget;
    optimize: boolean;
    optimizationLevel?: 0 | 1 | 2 | 3;
    includeComments?: boolean;
    prettyPrint?: boolean;
    validateCircuit?: boolean;
}

/** Default compiler options */
export const DEFAULT_COMPILER_OPTIONS: CompilerOptions = {
    target: 'qiskit',
    optimize: true,
    optimizationLevel: 1,
    includeComments: true,
    prettyPrint: true,
    validateCircuit: true,
};

// ============================================================
// Error Types
// ============================================================

/** Base compiler error */
export class WiaQuantumError extends Error {
    constructor(
        message: string,
        public code: string,
        public location?: SourceLocation
    ) {
        super(message);
        this.name = 'WiaQuantumError';
    }
}

/** Parse error */
export class ParseError extends WiaQuantumError {
    constructor(message: string, location?: SourceLocation) {
        super(message, 'PARSE_ERROR', location);
        this.name = 'ParseError';
    }
}

/** Semantic error */
export class SemanticError extends WiaQuantumError {
    constructor(message: string, location?: SourceLocation) {
        super(message, 'SEMANTIC_ERROR', location);
        this.name = 'SemanticError';
    }
}

/** Code generation error */
export class CodeGenError extends WiaQuantumError {
    constructor(message: string, location?: SourceLocation) {
        super(message, 'CODEGEN_ERROR', location);
        this.name = 'CodeGenError';
    }
}
