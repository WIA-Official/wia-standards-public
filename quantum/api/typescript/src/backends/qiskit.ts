/**
 * Qiskit Backend
 *
 * Generates IBM Qiskit Python code from WIA-QIR.
 */

import {
    QIRProgram,
    QIRInstruction,
    CompilationResult,
    CompilationStats,
    CompilerWarning,
    CodeGenError,
} from '../types';

// ============================================================
// Gate Mapping to Qiskit
// ============================================================

const QISKIT_GATE_MAP: Record<string, string> = {
    // Single qubit gates
    'I': 'id',
    'X': 'x',
    'Y': 'y',
    'Z': 'z',
    'H': 'h',
    'S': 's',
    'T': 't',
    'SDG': 'sdg',
    'TDG': 'tdg',

    // Parameterized single qubit
    'RX': 'rx',
    'RY': 'ry',
    'RZ': 'rz',
    'U': 'u',
    'P': 'p',

    // Two qubit gates
    'CNOT': 'cx',
    'CX': 'cx',
    'CY': 'cy',
    'CZ': 'cz',
    'SWAP': 'swap',
    'ISWAP': 'iswap',

    // Parameterized two qubit
    'CRX': 'crx',
    'CRY': 'cry',
    'CRZ': 'crz',
    'CU': 'cu',

    // Three qubit gates
    'CCNOT': 'ccx',
    'CCX': 'ccx',
    'TOFFOLI': 'ccx',
    'CSWAP': 'cswap',
    'FREDKIN': 'cswap',

    // Special
    'MEASURE': 'measure',
};

// ============================================================
// Qiskit Backend Class
// ============================================================

export class QiskitBackend {
    private warnings: CompilerWarning[] = [];
    private stats: CompilationStats = {
        totalGates: 0,
        singleQubitGates: 0,
        twoQubitGates: 0,
        threeQubitGates: 0,
        measurements: 0,
        circuitDepth: 0,
        qubitCount: 0,
    };

    /**
     * Generate Qiskit Python code from QIR
     */
    generate(qir: QIRProgram, options?: { includeComments?: boolean }): CompilationResult {
        this.warnings = [];
        this.resetStats();

        const includeComments = options?.includeComments ?? true;
        const lines: string[] = [];

        // Header
        if (includeComments) {
            lines.push('"""');
            lines.push(`WIA-QUANTUM Generated Code`);
            lines.push(`Program: ${qir.program}`);
            lines.push(`Generated: ${new Date().toISOString()}`);
            lines.push('"""');
            lines.push('');
        }

        // Imports
        lines.push('from qiskit import QuantumCircuit, QuantumRegister, ClassicalRegister');
        lines.push('from qiskit import transpile');
        lines.push('from qiskit_aer import AerSimulator');
        lines.push('import numpy as np');
        lines.push('');

        // Function definition
        const funcName = this.toSnakeCase(qir.program);
        lines.push(`def ${funcName}():`);

        // Register declarations
        const indent = '    ';
        for (const reg of qir.registers.quantum) {
            lines.push(`${indent}${reg.name} = QuantumRegister(${reg.size}, '${reg.name}')`);
            this.stats.qubitCount += reg.size;
        }
        for (const reg of qir.registers.classical) {
            lines.push(`${indent}${reg.name} = ClassicalRegister(${reg.size}, '${reg.name}')`);
        }

        // Create circuit
        const qregs = qir.registers.quantum.map(r => r.name).join(', ');
        const cregs = qir.registers.classical.map(r => r.name).join(', ');
        if (cregs) {
            lines.push(`${indent}qc = QuantumCircuit(${qregs}, ${cregs})`);
        } else {
            lines.push(`${indent}qc = QuantumCircuit(${qregs})`);
        }
        lines.push('');

        // Instructions
        for (const inst of qir.instructions) {
            const code = this.generateInstruction(inst);
            if (code) {
                // Handle conditional instructions
                if (inst.condition) {
                    lines.push(`${indent}# Conditional: ${inst.condition.register} == ${inst.condition.value}`);
                    lines.push(`${indent}${code}.c_if(${inst.condition.register}, ${inst.condition.value})`);
                } else {
                    lines.push(`${indent}${code}`);
                }
            }
        }

        lines.push('');
        lines.push(`${indent}return qc`);
        lines.push('');

        // Main execution block
        lines.push('');
        lines.push('if __name__ == "__main__":');
        lines.push(`${indent}# Create circuit`);
        lines.push(`${indent}circuit = ${funcName}()`);
        lines.push('');
        lines.push(`${indent}# Print circuit`);
        lines.push(`${indent}print(circuit.draw())`);
        lines.push('');
        lines.push(`${indent}# Simulate`);
        lines.push(`${indent}simulator = AerSimulator()`);
        lines.push(`${indent}compiled = transpile(circuit, simulator)`);
        lines.push(`${indent}result = simulator.run(compiled, shots=1000).result()`);
        lines.push(`${indent}counts = result.get_counts()`);
        lines.push(`${indent}print("\\nResults:", counts)`);

        // Calculate depth (simplified)
        this.stats.circuitDepth = this.calculateDepth(qir);

        return {
            target: 'qiskit',
            code: lines.join('\n'),
            warnings: this.warnings,
            stats: this.stats,
        };
    }

    /**
     * Generate code for a single instruction
     */
    private generateInstruction(inst: QIRInstruction): string | null {
        const qiskitGate = QISKIT_GATE_MAP[inst.op];
        if (!qiskitGate) {
            this.warnings.push({
                message: `Unknown gate: ${inst.op}`,
                severity: 'warning',
            });
            return null;
        }

        // Format qubit references
        const qubits = inst.qubits.map(q => this.formatQubit(q)).join(', ');

        // Update statistics
        this.stats.totalGates++;
        if (inst.qubits.length === 1) {
            this.stats.singleQubitGates++;
        } else if (inst.qubits.length === 2) {
            this.stats.twoQubitGates++;
        } else if (inst.qubits.length === 3) {
            this.stats.threeQubitGates++;
        }

        // Handle measurement specially
        if (inst.op === 'MEASURE') {
            this.stats.measurements++;
            if (inst.classical && inst.classical.length > 0) {
                const classical = inst.classical.map(c => this.formatQubit(c)).join(', ');
                return `qc.measure(${qubits}, ${classical})`;
            }
            return `qc.measure_all()`;
        }

        // Handle parameterized gates
        if (inst.params && inst.params.length > 0) {
            const params = inst.params.join(', ');
            return `qc.${qiskitGate}(${params}, ${qubits})`;
        }

        return `qc.${qiskitGate}(${qubits})`;
    }

    /**
     * Format qubit reference for Qiskit
     */
    private formatQubit(ref: string): string {
        // Convert "q[0]" to "q[0]" (already correct format)
        return ref;
    }

    /**
     * Convert name to snake_case
     */
    private toSnakeCase(name: string): string {
        return name
            .replace(/([A-Z])/g, '_$1')
            .toLowerCase()
            .replace(/^_/, '');
    }

    /**
     * Calculate circuit depth (simplified)
     */
    private calculateDepth(qir: QIRProgram): number {
        // Simplified: count max sequential gates on any qubit
        const qubitDepth: Map<string, number> = new Map();

        for (const inst of qir.instructions) {
            if (inst.op === 'MEASURE') continue;

            let maxDepth = 0;
            for (const q of inst.qubits) {
                const current = qubitDepth.get(q) || 0;
                maxDepth = Math.max(maxDepth, current);
            }

            for (const q of inst.qubits) {
                qubitDepth.set(q, maxDepth + 1);
            }
        }

        return Math.max(...Array.from(qubitDepth.values()), 0);
    }

    /**
     * Reset statistics
     */
    private resetStats(): void {
        this.stats = {
            totalGates: 0,
            singleQubitGates: 0,
            twoQubitGates: 0,
            threeQubitGates: 0,
            measurements: 0,
            circuitDepth: 0,
            qubitCount: 0,
        };
    }
}

/**
 * Generate Qiskit code from QIR
 */
export function generateQiskit(
    qir: QIRProgram,
    options?: { includeComments?: boolean }
): CompilationResult {
    const backend = new QiskitBackend();
    return backend.generate(qir, options);
}
