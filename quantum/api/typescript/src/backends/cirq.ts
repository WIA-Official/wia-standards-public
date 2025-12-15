/**
 * Cirq Backend
 *
 * Generates Google Cirq Python code from WIA-QIR.
 */

import {
    QIRProgram,
    QIRInstruction,
    CompilationResult,
    CompilationStats,
    CompilerWarning,
} from '../types';

// ============================================================
// Gate Mapping to Cirq
// ============================================================

const CIRQ_GATE_MAP: Record<string, string> = {
    // Single qubit gates
    'I': 'cirq.I',
    'X': 'cirq.X',
    'Y': 'cirq.Y',
    'Z': 'cirq.Z',
    'H': 'cirq.H',
    'S': 'cirq.S',
    'T': 'cirq.T',
    'SDG': 'cirq.S**-1',
    'TDG': 'cirq.T**-1',

    // Parameterized single qubit
    'RX': 'cirq.rx',
    'RY': 'cirq.ry',
    'RZ': 'cirq.rz',

    // Two qubit gates
    'CNOT': 'cirq.CNOT',
    'CX': 'cirq.CNOT',
    'CY': 'cirq.ControlledGate(cirq.Y)',
    'CZ': 'cirq.CZ',
    'SWAP': 'cirq.SWAP',
    'ISWAP': 'cirq.ISWAP',

    // Parameterized two qubit
    'CRX': 'cirq.ControlledGate(cirq.rx',
    'CRY': 'cirq.ControlledGate(cirq.ry',
    'CRZ': 'cirq.ControlledGate(cirq.rz',

    // Three qubit gates
    'CCNOT': 'cirq.TOFFOLI',
    'CCX': 'cirq.TOFFOLI',
    'TOFFOLI': 'cirq.TOFFOLI',
    'CSWAP': 'cirq.FREDKIN',
    'FREDKIN': 'cirq.FREDKIN',

    // Special
    'MEASURE': 'cirq.measure',
};

// ============================================================
// Cirq Backend Class
// ============================================================

export class CirqBackend {
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
     * Generate Cirq Python code from QIR
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
        lines.push('import cirq');
        lines.push('import numpy as np');
        lines.push('');

        // Function definition
        const funcName = this.toSnakeCase(qir.program);
        lines.push(`def ${funcName}():`);

        const indent = '    ';

        // Create qubits
        let totalQubits = 0;
        for (const reg of qir.registers.quantum) {
            totalQubits += reg.size;
            this.stats.qubitCount += reg.size;
        }

        lines.push(`${indent}# Create qubits`);
        for (const reg of qir.registers.quantum) {
            lines.push(`${indent}${reg.name} = [cirq.NamedQubit(f'${reg.name}_{i}') for i in range(${reg.size})]`);
        }
        lines.push('');

        // Build circuit
        lines.push(`${indent}# Build circuit`);
        lines.push(`${indent}circuit = cirq.Circuit()`);
        lines.push('');

        // Group operations into moments where possible
        lines.push(`${indent}# Add operations`);
        for (const inst of qir.instructions) {
            const code = this.generateInstruction(inst);
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
        lines.push(`${indent}# Simulate`);
        lines.push(`${indent}simulator = cirq.Simulator()`);
        lines.push(`${indent}result = simulator.run(circuit, repetitions=1000)`);
        lines.push(`${indent}print("\\nResults:")`);
        lines.push(`${indent}print(result.histogram(key='result'))`);

        // Calculate depth
        this.stats.circuitDepth = this.calculateDepth(qir);

        return {
            target: 'cirq',
            code: lines.join('\n'),
            warnings: this.warnings,
            stats: this.stats,
        };
    }

    /**
     * Generate code for a single instruction
     */
    private generateInstruction(inst: QIRInstruction): string | null {
        const cirqGate = CIRQ_GATE_MAP[inst.op];
        if (!cirqGate) {
            this.warnings.push({
                message: `Unknown gate: ${inst.op}`,
                severity: 'warning',
            });
            return null;
        }

        // Format qubit references for Cirq
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

        // Handle measurement
        if (inst.op === 'MEASURE') {
            this.stats.measurements++;
            return `circuit.append(cirq.measure(${qubits}, key='result'))`;
        }

        // Handle parameterized gates
        if (inst.params && inst.params.length > 0) {
            const params = inst.params.join(', ');
            // Check if it's a controlled parameterized gate
            if (cirqGate.startsWith('cirq.ControlledGate(')) {
                return `circuit.append(${cirqGate}(${params}))(${qubits}))`;
            }
            return `circuit.append(${cirqGate}(${params})(${qubits}))`;
        }

        // Handle power gates (SDG, TDG)
        if (cirqGate.includes('**')) {
            return `circuit.append((${cirqGate})(${qubits}))`;
        }

        return `circuit.append(${cirqGate}(${qubits}))`;
    }

    /**
     * Format qubit reference for Cirq
     */
    private formatQubit(ref: string): string {
        // Convert "q[0]" to "q[0]"
        const match = ref.match(/(\w+)\[(\d+)\]/);
        if (match) {
            return `${match[1]}[${match[2]}]`;
        }
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
 * Generate Cirq code from QIR
 */
export function generateCirq(
    qir: QIRProgram,
    options?: { includeComments?: boolean }
): CompilationResult {
    const backend = new CirqBackend();
    return backend.generate(qir, options);
}
