/**
 * Parser Tests
 */

import { describe, it, expect } from 'vitest';
import { Lexer } from '../parser/lexer';
import { Parser } from '../parser/parser';

describe('Parser', () => {
    const parse = (source: string) => {
        const lexer = new Lexer(source);
        const tokens = lexer.tokenize();
        const parser = new Parser(tokens);
        return parser.parse();
    };

    describe('Program Structure', () => {
        it('should parse program name', () => {
            const ast = parse('program TestProgram');
            expect(ast.name).toBe('TestProgram');
        });

        it('should parse quantum registers', () => {
            const ast = parse(`
                program Test
                qreg q[4]
                qreg aux[2]
            `);

            expect(ast.registers).toHaveLength(2);
            expect(ast.registers[0].name).toBe('q');
            expect(ast.registers[0].size).toBe(4);
            expect(ast.registers[0].type).toBe('quantum');
            expect(ast.registers[1].name).toBe('aux');
            expect(ast.registers[1].size).toBe(2);
        });

        it('should parse classical registers', () => {
            const ast = parse(`
                program Test
                creg c[2]
            `);

            expect(ast.registers).toHaveLength(1);
            expect(ast.registers[0].name).toBe('c');
            expect(ast.registers[0].type).toBe('classical');
        });
    });

    describe('Gate Operations', () => {
        it('should parse single qubit gates', () => {
            const ast = parse(`
                program Test
                qreg q[1]
                H q[0]
            `);

            expect(ast.statements).toHaveLength(1);
            expect(ast.statements[0].type).toBe('gate');
            if (ast.statements[0].type === 'gate') {
                expect(ast.statements[0].gate).toBe('H');
                expect(ast.statements[0].qubits).toHaveLength(1);
                expect(ast.statements[0].qubits[0]).toEqual({ register: 'q', index: 0 });
            }
        });

        it('should parse two qubit gates', () => {
            const ast = parse(`
                program Test
                qreg q[2]
                CNOT q[0], q[1]
            `);

            expect(ast.statements).toHaveLength(1);
            if (ast.statements[0].type === 'gate') {
                expect(ast.statements[0].gate).toBe('CNOT');
                expect(ast.statements[0].qubits).toHaveLength(2);
            }
        });

        it('should parse three qubit gates', () => {
            const ast = parse(`
                program Test
                qreg q[3]
                CCX q[0], q[1], q[2]
            `);

            expect(ast.statements).toHaveLength(1);
            if (ast.statements[0].type === 'gate') {
                expect(ast.statements[0].gate).toBe('CCX');
                expect(ast.statements[0].qubits).toHaveLength(3);
            }
        });

        it('should parse parameterized gates', () => {
            const ast = parse(`
                program Test
                qreg q[1]
                RX(1.5708) q[0]
            `);

            expect(ast.statements).toHaveLength(1);
            if (ast.statements[0].type === 'gate') {
                expect(ast.statements[0].gate).toBe('RX');
                expect(ast.statements[0].params).toEqual([1.5708]);
            }
        });

        it('should parse gates with multiple parameters', () => {
            const ast = parse(`
                program Test
                qreg q[1]
                U(1.57, 0, 3.14) q[0]
            `);

            if (ast.statements[0].type === 'gate') {
                expect(ast.statements[0].params).toEqual([1.57, 0, 3.14]);
            }
        });
    });

    describe('Measurement', () => {
        it('should parse measure statement', () => {
            const ast = parse(`
                program Test
                qreg q[2]
                creg c[2]
                measure q -> c
            `);

            const measureStmt = ast.statements.find(s => s.type === 'measure');
            expect(measureStmt).toBeDefined();
            if (measureStmt && measureStmt.type === 'measure') {
                expect(measureStmt.quantum).toBe('q');
                expect(measureStmt.classical).toBe('c');
            }
        });

        it('should parse indexed measure', () => {
            const ast = parse(`
                program Test
                qreg q[2]
                creg c[2]
                measure q[0] -> c[0]
            `);

            const measureStmt = ast.statements.find(s => s.type === 'measure');
            expect(measureStmt).toBeDefined();
        });
    });

    describe('Complete Programs', () => {
        it('should parse Bell state program', () => {
            const ast = parse(`
                program BellState
                qreg q[2]
                creg c[2]
                H q[0]
                CNOT q[0], q[1]
                measure q -> c
            `);

            expect(ast.name).toBe('BellState');
            expect(ast.registers).toHaveLength(2);
            expect(ast.statements).toHaveLength(3); // H, CNOT, measure
        });

        it('should parse GHZ state program', () => {
            const ast = parse(`
                program GHZState
                qreg q[3]
                creg c[3]
                H q[0]
                CNOT q[0], q[1]
                CNOT q[1], q[2]
                measure q -> c
            `);

            expect(ast.name).toBe('GHZState');
            expect(ast.statements).toHaveLength(4);
        });
    });
});
