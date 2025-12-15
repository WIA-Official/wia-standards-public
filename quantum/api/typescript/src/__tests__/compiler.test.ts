/**
 * Compiler Integration Tests
 */

import { describe, it, expect } from 'vitest';
import { WiaQuantumCompiler, compile } from '../index';
import { BackendTarget } from '../types';

describe('WiaQuantumCompiler', () => {
    const bellStateSource = `
        program BellState
        qreg q[2]
        creg c[2]
        H q[0]
        CNOT q[0], q[1]
        measure q -> c
    `;

    describe('Basic Compilation', () => {
        it('should compile to Qiskit', () => {
            const result = compile(bellStateSource, 'qiskit');

            expect(result.target).toBe('qiskit');
            expect(result.code).toContain('from qiskit import QuantumCircuit');
            expect(result.code).toContain('qc.h');
            expect(result.code).toContain('qc.cx');
            expect(result.code).toContain('qc.measure');
        });

        it('should compile to Cirq', () => {
            const result = compile(bellStateSource, 'cirq');

            expect(result.target).toBe('cirq');
            expect(result.code).toContain('import cirq');
            expect(result.code).toContain('cirq.H');
            expect(result.code).toContain('cirq.CNOT');
        });

        it('should compile to Q#', () => {
            const result = compile(bellStateSource, 'qsharp');

            expect(result.target).toBe('qsharp');
            expect(result.code).toContain('namespace WiaQuantum');
            expect(result.code).toContain('H(');
            expect(result.code).toContain('CNOT(');
        });

        it('should compile to Braket', () => {
            const result = compile(bellStateSource, 'braket');

            expect(result.target).toBe('braket');
            expect(result.code).toContain('from braket.circuits import Circuit');
            expect(result.code).toContain('circuit.h');
            expect(result.code).toContain('circuit.cnot');
        });

        it('should compile to PyQuil', () => {
            const result = compile(bellStateSource, 'pyquil');

            expect(result.target).toBe('pyquil');
            expect(result.code).toContain('from pyquil import Program');
            expect(result.code).toContain('p += H(');
            expect(result.code).toContain('p += CNOT(');
        });
    });

    describe('Statistics', () => {
        it('should track gate statistics', () => {
            const result = compile(bellStateSource, 'qiskit');

            expect(result.stats.totalGates).toBe(3); // H, CNOT, MEASURE
            expect(result.stats.singleQubitGates).toBe(1); // H
            expect(result.stats.twoQubitGates).toBe(1); // CNOT
            expect(result.stats.measurements).toBe(1);
            expect(result.stats.qubitCount).toBe(2);
        });

        it('should calculate circuit depth', () => {
            const result = compile(bellStateSource, 'qiskit');

            // H on q[0], then CNOT on q[0],q[1] -> depth is 2
            expect(result.stats.circuitDepth).toBe(2);
        });
    });

    describe('Compile All Backends', () => {
        it('should compile to all backends at once', () => {
            const compiler = new WiaQuantumCompiler();
            const results = compiler.compileAll(bellStateSource);

            expect(results.size).toBe(5);
            expect(results.has('qiskit')).toBe(true);
            expect(results.has('cirq')).toBe(true);
            expect(results.has('qsharp')).toBe(true);
            expect(results.has('braket')).toBe(true);
            expect(results.has('pyquil')).toBe(true);
        });
    });

    describe('Parameterized Gates', () => {
        it('should handle parameterized gates', () => {
            const source = `
                program RotationTest
                qreg q[1]
                RX(1.5708) q[0]
                RY(3.1416) q[0]
                RZ(0.7854) q[0]
            `;

            const result = compile(source, 'qiskit');

            expect(result.code).toContain('qc.rx(1.5708');
            expect(result.code).toContain('qc.ry(3.1416');
            expect(result.code).toContain('qc.rz(0.7854');
        });
    });

    describe('Three Qubit Gates', () => {
        it('should handle Toffoli gate', () => {
            const source = `
                program ToffoliTest
                qreg q[3]
                CCX q[0], q[1], q[2]
            `;

            const qiskitResult = compile(source, 'qiskit');
            expect(qiskitResult.code).toContain('qc.ccx');

            const cirqResult = compile(source, 'cirq');
            expect(cirqResult.code).toContain('cirq.TOFFOLI');

            const qsharpResult = compile(source, 'qsharp');
            expect(qsharpResult.code).toContain('CCNOT(');
        });
    });

    describe('Error Handling', () => {
        it('should warn on unknown gates', () => {
            const source = `
                program Test
                qreg q[1]
                UNKNOWNGATE q[0]
            `;

            const result = compile(source, 'qiskit');
            expect(result.warnings.length).toBeGreaterThan(0);
            expect(result.warnings[0].message).toContain('Unknown gate');
        });
    });

    describe('Comments', () => {
        it('should include comments when enabled', () => {
            const compiler = new WiaQuantumCompiler({ includeComments: true });
            const result = compiler.compile(bellStateSource, 'qiskit');

            expect(result.code).toContain('WIA-QUANTUM Generated Code');
        });

        it('should exclude comments when disabled', () => {
            const compiler = new WiaQuantumCompiler({ includeComments: false });
            const result = compiler.compile(bellStateSource, 'qiskit');

            expect(result.code).not.toContain('WIA-QUANTUM Generated Code');
        });
    });
});
