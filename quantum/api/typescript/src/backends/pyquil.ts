/**
 * PyQuil Backend
 *
 * Generates Rigetti PyQuil Python code from WIA-QIR.
 */

import {
    QIRProgram,
    QIRInstruction,
    CompilationResult,
    CompilationStats,
    CompilerWarning,
} from '../types';

// ============================================================
// Gate Mapping to PyQuil
// ============================================================

const PYQUIL_GATE_MAP: Record<string, string> = {
    // Single qubit gates
    'I': 'I',
    'X': 'X',
    'Y': 'Y',
    'Z': 'Z',
    'H': 'H',
    'S': 'S',
    'T': 'T',
    'SDG': 'PHASE(-pi/2)',  // S† = Phase(-π/2)
    'TDG': 'PHASE(-pi/4)',  // T† = Phase(-π/4)

    // Parameterized single qubit
    'RX': 'RX',
    'RY': 'RY',
    'RZ': 'RZ',
    'P': 'PHASE',

    // Two qubit gates
    'CNOT': 'CNOT',
    'CX': 'CNOT',
    'CY': 'CY',  // Custom gate needed
    'CZ': 'CZ',
    'SWAP': 'SWAP',
    'ISWAP': 'ISWAP',

    // Parameterized two qubit
    'CRX': 'CONTROLLED RX',
    'CRY': 'CONTROLLED RY',
    'CRZ': 'CPHASE',  // PyQuil uses CPHASE

    // Three qubit gates
    'CCNOT': 'CCNOT',
    'CCX': 'CCNOT',
    'TOFFOLI': 'CCNOT',
    'CSWAP': 'CSWAP',
    'FREDKIN': 'CSWAP',

    // Special
    'MEASURE': 'MEASURE',
};

// ============================================================
// PyQuil Backend Class
// ============================================================

export class PyQuilBackend {
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
     * Generate PyQuil Python code from QIR
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
        lines.push('from pyquil import Program, get_qc');
        lines.push('from pyquil.gates import *');
        lines.push('from pyquil.quilbase import Declare');
        lines.push('import numpy as np');
        lines.push('');

        // Function definition
        const funcName = this.toSnakeCase(qir.program);
        lines.push(`def ${funcName}():`);

        const indent = '    ';

        // Create program
        lines.push(`${indent}# Create program`);
        lines.push(`${indent}p = Program()`);
        lines.push('');

        // Qubit mapping
        let totalQubits = 0;
        const qubitMap: Record<string, number> = {};
        for (const reg of qir.registers.quantum) {
            for (let i = 0; i < reg.size; i++) {
                qubitMap[`${reg.name}[${i}]`] = totalQubits + i;
            }
            this.stats.qubitCount += reg.size;
            totalQubits += reg.size;
        }

        // Classical memory declaration
        if (qir.registers.classical.length > 0) {
            lines.push(`${indent}# Declare classical memory`);
            for (const reg of qir.registers.classical) {
                lines.push(`${indent}p += Declare('${reg.name}', 'BIT', ${reg.size})`);
            }
            lines.push('');
        }

        // Instructions
        lines.push(`${indent}# Quantum operations`);
        for (const inst of qir.instructions) {
            const code = this.generateInstruction(inst, qubitMap);
            if (code) {
                lines.push(`${indent}${code}`);
            }
        }

        lines.push('');
        lines.push(`${indent}return p`);
        lines.push('');

        // Main execution block
        lines.push('');
        lines.push('if __name__ == "__main__":');
        lines.push(`${indent}# Create program`);
        lines.push(`${indent}program = ${funcName}()`);
        lines.push('');
        lines.push(`${indent}# Print program`);
        lines.push(`${indent}print(program)`);
        lines.push('');
        lines.push(`${indent}# Run on QVM simulator`);
        lines.push(`${indent}qc = get_qc('${totalQubits}q-qvm')`);
        lines.push(`${indent}executable = qc.compile(program)`);
        lines.push(`${indent}result = qc.run(executable)`);
        lines.push(`${indent}print("\\nResults:", result.readout_data)`);

        // Calculate depth
        this.stats.circuitDepth = this.calculateDepth(qir);

        return {
            target: 'pyquil',
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
        const pyquilGate = PYQUIL_GATE_MAP[inst.op];
        if (!pyquilGate) {
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
            if (inst.classical && inst.classical.length > 0) {
                // Parse classical reference
                const classicalRef = inst.classical[0];
                const match = classicalRef.match(/(\w+)\[(\d+)\]/);
                if (match) {
                    const regName = match[1];
                    const regIndex = match[2];
                    return `p += MEASURE(${qubitIndices[0]}, ('${regName}', ${regIndex}))`;
                }
            }
            return `p += MEASURE(${qubitIndices[0]})`;
        }

        // Handle special gates (SDG, TDG using PHASE)
        if (inst.op === 'SDG') {
            return `p += PHASE(-np.pi/2, ${qubitIndices[0]})`;
        }
        if (inst.op === 'TDG') {
            return `p += PHASE(-np.pi/4, ${qubitIndices[0]})`;
        }

        // Handle parameterized gates
        if (inst.params && inst.params.length > 0) {
            const params = inst.params.join(', ');
            if (inst.qubits.length === 1) {
                return `p += ${pyquilGate}(${params}, ${qubitIndices[0]})`;
            } else {
                const qubits = qubitIndices.join(', ');
                return `p += ${pyquilGate}(${params}, ${qubits})`;
            }
        }

        // Handle regular gates
        if (inst.qubits.length === 1) {
            return `p += ${pyquilGate}(${qubitIndices[0]})`;
        } else if (inst.qubits.length === 2) {
            return `p += ${pyquilGate}(${qubitIndices[0]}, ${qubitIndices[1]})`;
        } else if (inst.qubits.length === 3) {
            return `p += ${pyquilGate}(${qubitIndices[0]}, ${qubitIndices[1]}, ${qubitIndices[2]})`;
        }

        return `p += ${pyquilGate}(${qubitIndices.join(', ')})`;
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
 * Generate PyQuil code from QIR
 */
export function generatePyQuil(
    qir: QIRProgram,
    options?: { includeComments?: boolean }
): CompilationResult {
    const backend = new PyQuilBackend();
    return backend.generate(qir, options);
}
