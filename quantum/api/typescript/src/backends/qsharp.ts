/**
 * Q# Backend
 *
 * Generates Microsoft Q# code from WIA-QIR.
 */

import {
    QIRProgram,
    QIRInstruction,
    CompilationResult,
    CompilationStats,
    CompilerWarning,
} from '../types';

// ============================================================
// Gate Mapping to Q#
// ============================================================

const QSHARP_GATE_MAP: Record<string, string> = {
    // Single qubit gates
    'I': 'I',
    'X': 'X',
    'Y': 'Y',
    'Z': 'Z',
    'H': 'H',
    'S': 'S',
    'T': 'T',
    'SDG': 'Adjoint S',
    'TDG': 'Adjoint T',

    // Parameterized single qubit
    'RX': 'Rx',
    'RY': 'Ry',
    'RZ': 'Rz',
    'U': 'U3',
    'P': 'R1',

    // Two qubit gates
    'CNOT': 'CNOT',
    'CX': 'CNOT',
    'CY': 'CY',
    'CZ': 'CZ',
    'SWAP': 'SWAP',

    // Three qubit gates
    'CCNOT': 'CCNOT',
    'CCX': 'CCNOT',
    'TOFFOLI': 'CCNOT',
    'CSWAP': 'Controlled SWAP',
    'FREDKIN': 'Controlled SWAP',

    // Special
    'MEASURE': 'M',
};

// ============================================================
// Q# Backend Class
// ============================================================

export class QSharpBackend {
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
     * Generate Q# code from QIR
     */
    generate(qir: QIRProgram, options?: { includeComments?: boolean }): CompilationResult {
        this.warnings = [];
        this.resetStats();

        const includeComments = options?.includeComments ?? true;
        const lines: string[] = [];

        // Header
        if (includeComments) {
            lines.push('/// WIA-QUANTUM Generated Code');
            lines.push(`/// Program: ${qir.program}`);
            lines.push(`/// Generated: ${new Date().toISOString()}`);
            lines.push('');
        }

        // Namespace and imports
        lines.push('namespace WiaQuantum {');
        lines.push('');
        lines.push('    open Microsoft.Quantum.Canon;');
        lines.push('    open Microsoft.Quantum.Intrinsic;');
        lines.push('    open Microsoft.Quantum.Measurement;');
        lines.push('');

        // Operation definition
        const opName = this.toPascalCase(qir.program);

        // Calculate total qubits needed
        let totalQubits = 0;
        for (const reg of qir.registers.quantum) {
            totalQubits += reg.size;
            this.stats.qubitCount += reg.size;
        }

        // Check if we have classical registers (need to return results)
        const hasClassical = qir.registers.classical.length > 0;
        const returnType = hasClassical ? 'Result[]' : 'Unit';

        lines.push(`    @EntryPoint()`);
        lines.push(`    operation ${opName}() : ${returnType} {`);

        const indent = '        ';

        // Allocate qubits
        lines.push(`${indent}// Allocate qubits`);
        for (const reg of qir.registers.quantum) {
            lines.push(`${indent}use ${reg.name} = Qubit[${reg.size}];`);
        }
        lines.push('');

        // Mutable results array if needed
        if (hasClassical) {
            let totalClassical = 0;
            for (const reg of qir.registers.classical) {
                totalClassical += reg.size;
            }
            lines.push(`${indent}mutable results = [Zero, size = ${totalClassical}];`);
            lines.push('');
        }

        // Instructions
        lines.push(`${indent}// Quantum operations`);
        let measureIndex = 0;
        for (const inst of qir.instructions) {
            const code = this.generateInstruction(inst, measureIndex);
            if (code) {
                if (inst.op === 'MEASURE') {
                    lines.push(`${indent}${code}`);
                    measureIndex++;
                } else {
                    lines.push(`${indent}${code}`);
                }
            }
        }

        lines.push('');

        // Reset qubits before release
        lines.push(`${indent}// Reset qubits`);
        for (const reg of qir.registers.quantum) {
            lines.push(`${indent}ResetAll(${reg.name});`);
        }

        // Return results if needed
        if (hasClassical) {
            lines.push('');
            lines.push(`${indent}return results;`);
        }

        lines.push('    }');
        lines.push('}');

        // Calculate depth
        this.stats.circuitDepth = this.calculateDepth(qir);

        return {
            target: 'qsharp',
            code: lines.join('\n'),
            warnings: this.warnings,
            stats: this.stats,
        };
    }

    /**
     * Generate code for a single instruction
     */
    private generateInstruction(inst: QIRInstruction, measureIndex: number): string | null {
        const qsharpGate = QSHARP_GATE_MAP[inst.op];
        if (!qsharpGate) {
            this.warnings.push({
                message: `Unknown gate: ${inst.op}`,
                severity: 'warning',
            });
            return null;
        }

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
            const qubit = this.formatQubit(inst.qubits[0]);
            return `set results w/= ${measureIndex} <- M(${qubit});`;
        }

        // Format qubit references
        const qubits = inst.qubits.map(q => this.formatQubit(q));

        // Handle parameterized gates
        if (inst.params && inst.params.length > 0) {
            const params = inst.params.join(', ');
            return `${qsharpGate}(${params}, ${qubits.join(', ')});`;
        }

        // Handle multi-qubit gates
        if (inst.qubits.length === 1) {
            return `${qsharpGate}(${qubits[0]});`;
        } else if (inst.qubits.length === 2) {
            // Control, target order for CNOT
            if (qsharpGate === 'CNOT') {
                return `CNOT(${qubits[0]}, ${qubits[1]});`;
            }
            return `${qsharpGate}(${qubits.join(', ')});`;
        } else if (inst.qubits.length === 3) {
            // CCNOT takes (control1, control2, target)
            return `${qsharpGate}(${qubits.join(', ')});`;
        }

        return `${qsharpGate}(${qubits.join(', ')});`;
    }

    /**
     * Format qubit reference for Q#
     */
    private formatQubit(ref: string): string {
        // Convert "q[0]" to "q[0]"
        return ref;
    }

    /**
     * Convert name to PascalCase
     */
    private toPascalCase(name: string): string {
        return name
            .replace(/[-_](.)/g, (_, c) => c.toUpperCase())
            .replace(/^(.)/, (_, c) => c.toUpperCase());
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
 * Generate Q# code from QIR
 */
export function generateQSharp(
    qir: QIRProgram,
    options?: { includeComments?: boolean }
): CompilationResult {
    const backend = new QSharpBackend();
    return backend.generate(qir, options);
}
