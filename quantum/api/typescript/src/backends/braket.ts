/**
 * Amazon Braket Backend
 *
 * Generates AWS Braket Python code from WIA-QIR.
 */

import {
    QIRProgram,
    QIRInstruction,
    CompilationResult,
    CompilationStats,
    CompilerWarning,
} from '../types';

// ============================================================
// Gate Mapping to Braket
// ============================================================

const BRAKET_GATE_MAP: Record<string, string> = {
    // Single qubit gates
    'I': 'I',
    'X': 'X',
    'Y': 'Y',
    'Z': 'Z',
    'H': 'H',
    'S': 'S',
    'T': 'T',
    'SDG': 'Si',
    'TDG': 'Ti',

    // Parameterized single qubit
    'RX': 'Rx',
    'RY': 'Ry',
    'RZ': 'Rz',
    'U': 'U',
    'P': 'PhaseShift',

    // Two qubit gates
    'CNOT': 'CNot',
    'CX': 'CNot',
    'CY': 'CY',
    'CZ': 'CZ',
    'SWAP': 'Swap',
    'ISWAP': 'ISwap',

    // Parameterized two qubit
    'CRX': 'CPhaseShift',  // Braket uses different controlled gates
    'CRY': 'CPhaseShift',
    'CRZ': 'CPhaseShift',

    // Three qubit gates
    'CCNOT': 'CCNot',
    'CCX': 'CCNot',
    'TOFFOLI': 'CCNot',
    'CSWAP': 'CSwap',
    'FREDKIN': 'CSwap',

    // Special
    'MEASURE': 'measure',
};

// ============================================================
// Braket Backend Class
// ============================================================

export class BraketBackend {
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
     * Generate Braket Python code from QIR
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
        lines.push('from braket.circuits import Circuit');
        lines.push('from braket.devices import LocalSimulator');
        lines.push('import numpy as np');
        lines.push('');

        // Function definition
        const funcName = this.toSnakeCase(qir.program);
        lines.push(`def ${funcName}():`);

        const indent = '    ';

        // Calculate total qubits
        let totalQubits = 0;
        for (const reg of qir.registers.quantum) {
            totalQubits += reg.size;
            this.stats.qubitCount += reg.size;
        }

        // Create circuit
        lines.push(`${indent}# Create circuit`);
        lines.push(`${indent}circuit = Circuit()`);
        lines.push('');

        // Create qubit index mapping
        lines.push(`${indent}# Qubit mapping`);
        let qubitOffset = 0;
        const qubitMap: Record<string, number> = {};
        for (const reg of qir.registers.quantum) {
            for (let i = 0; i < reg.size; i++) {
                qubitMap[`${reg.name}[${i}]`] = qubitOffset + i;
            }
            lines.push(`${indent}# ${reg.name}: qubits ${qubitOffset} to ${qubitOffset + reg.size - 1}`);
            qubitOffset += reg.size;
        }
        lines.push('');

        // Instructions
        lines.push(`${indent}# Quantum operations`);
        for (const inst of qir.instructions) {
            const code = this.generateInstruction(inst, qubitMap);
            if (code) {
                lines.push(`${indent}${code}`);
            }
        }

        lines.push('');
        lines.push(`${indent}return circuit`);
        lines.push('');

        // Main execution block
        lines.push('');
        lines.push('if __name__ == "__main__":');
        lines.push(`${indent}# Create circuit`);
        lines.push(`${indent}circuit = ${funcName}()`);
        lines.push('');
        lines.push(`${indent}# Print circuit`);
        lines.push(`${indent}print(circuit)`);
        lines.push('');
        lines.push(`${indent}# Run on local simulator`);
        lines.push(`${indent}device = LocalSimulator()`);
        lines.push(`${indent}result = device.run(circuit, shots=1000).result()`);
        lines.push(`${indent}counts = result.measurement_counts`);
        lines.push(`${indent}print("\\nResults:", counts)`);

        // Calculate depth
        this.stats.circuitDepth = this.calculateDepth(qir);

        return {
            target: 'braket',
            code: lines.join('\n'),
            warnings: this.warnings,
            stats: this.stats,
        };
    }

    /**
     * Generate code for a single instruction
     */
    private generateInstruction(
        inst: QIRInstruction,
        qubitMap: Record<string, number>
    ): string | null {
        const braketGate = BRAKET_GATE_MAP[inst.op];
        if (!braketGate) {
            this.warnings.push({
                message: `Unknown gate: ${inst.op}`,
                severity: 'warning',
            });
            return null;
        }

        // Get qubit indices
        const qubitIndices = inst.qubits.map(q => qubitMap[q] ?? 0);

        // Update statistics
        this.stats.totalGates++;
        if (inst.qubits.length === 1) {
            this.stats.singleQubitGates++;
        } else if (inst.qubits.length === 2) {
            this.stats.twoQubitGates++;
        } else if (inst.qubits.length === 3) {
            this.stats.threeQubitGates++;
        }

        // Handle measurement
        if (inst.op === 'MEASURE') {
            this.stats.measurements++;
            // Measure all specified qubits
            const qubits = qubitIndices.join(', ');
            return `circuit.measure([${qubits}])`;
        }

        // Handle parameterized gates
        if (inst.params && inst.params.length > 0) {
            const params = inst.params.join(', ');
            if (inst.qubits.length === 1) {
                return `circuit.${braketGate.toLowerCase()}(${qubitIndices[0]}, ${params})`;
            } else {
                const qubits = qubitIndices.join(', ');
                return `circuit.${braketGate.toLowerCase()}(${qubits}, ${params})`;
            }
        }

        // Handle regular gates
        if (inst.qubits.length === 1) {
            return `circuit.${braketGate.toLowerCase()}(${qubitIndices[0]})`;
        } else if (inst.qubits.length === 2) {
            return `circuit.${braketGate.toLowerCase()}(${qubitIndices[0]}, ${qubitIndices[1]})`;
        } else if (inst.qubits.length === 3) {
            return `circuit.${braketGate.toLowerCase()}(${qubitIndices[0]}, ${qubitIndices[1]}, ${qubitIndices[2]})`;
        }

        return `circuit.${braketGate.toLowerCase()}(${qubitIndices.join(', ')})`;
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
     * Calculate circuit depth
     */
    private calculateDepth(qir: QIRProgram): number {
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
 * Generate Braket code from QIR
 */
export function generateBraket(
    qir: QIRProgram,
    options?: { includeComments?: boolean }
): CompilationResult {
    const backend = new BraketBackend();
    return backend.generate(qir, options);
}
