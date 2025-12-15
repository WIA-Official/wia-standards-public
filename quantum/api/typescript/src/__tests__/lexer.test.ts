/**
 * Lexer Tests
 */

import { describe, it, expect } from 'vitest';
import { Lexer } from '../parser/lexer';
import { TokenType } from '../types';

describe('Lexer', () => {
    describe('Basic Tokenization', () => {
        it('should tokenize program declaration', () => {
            const lexer = new Lexer('program Test');
            const tokens = lexer.tokenize();

            expect(tokens[0].type).toBe(TokenType.KEYWORD);
            expect(tokens[0].value).toBe('program');
            expect(tokens[1].type).toBe(TokenType.IDENTIFIER);
            expect(tokens[1].value).toBe('Test');
        });

        it('should tokenize quantum register', () => {
            const lexer = new Lexer('qreg q[4]');
            const tokens = lexer.tokenize();

            expect(tokens[0].type).toBe(TokenType.KEYWORD);
            expect(tokens[0].value).toBe('qreg');
            expect(tokens[1].type).toBe(TokenType.IDENTIFIER);
            expect(tokens[1].value).toBe('q');
            expect(tokens[2].type).toBe(TokenType.LBRACKET);
            expect(tokens[3].type).toBe(TokenType.INTEGER);
            expect(tokens[3].value).toBe('4');
            expect(tokens[4].type).toBe(TokenType.RBRACKET);
        });

        it('should tokenize classical register', () => {
            const lexer = new Lexer('creg c[2]');
            const tokens = lexer.tokenize();

            expect(tokens[0].type).toBe(TokenType.KEYWORD);
            expect(tokens[0].value).toBe('creg');
        });
    });

    describe('Gate Tokenization', () => {
        it('should tokenize single qubit gates', () => {
            const gates = ['H', 'X', 'Y', 'Z', 'S', 'T'];
            for (const gate of gates) {
                const lexer = new Lexer(`${gate} q[0]`);
                const tokens = lexer.tokenize();
                expect(tokens[0].type).toBe(TokenType.GATE);
                expect(tokens[0].value).toBe(gate);
            }
        });

        it('should tokenize two qubit gates', () => {
            const lexer = new Lexer('CNOT q[0], q[1]');
            const tokens = lexer.tokenize();

            expect(tokens[0].type).toBe(TokenType.GATE);
            expect(tokens[0].value).toBe('CNOT');
            expect(tokens[3].type).toBe(TokenType.COMMA);
        });

        it('should tokenize parameterized gates', () => {
            const lexer = new Lexer('RX(1.5708) q[0]');
            const tokens = lexer.tokenize();

            expect(tokens[0].type).toBe(TokenType.GATE);
            expect(tokens[0].value).toBe('RX');
            expect(tokens[1].type).toBe(TokenType.LPAREN);
            expect(tokens[2].type).toBe(TokenType.FLOAT);
            expect(tokens[2].value).toBe('1.5708');
            expect(tokens[3].type).toBe(TokenType.RPAREN);
        });
    });

    describe('Comments', () => {
        it('should skip single line comments', () => {
            const lexer = new Lexer('H q[0] // This is a comment');
            const tokens = lexer.tokenize();

            // Should only have H, q, [, 0, ], EOF
            expect(tokens.length).toBe(6);
            expect(tokens[tokens.length - 1].type).toBe(TokenType.EOF);
        });
    });

    describe('Measurement', () => {
        it('should tokenize measure statement', () => {
            const lexer = new Lexer('measure q -> c');
            const tokens = lexer.tokenize();

            expect(tokens[0].type).toBe(TokenType.KEYWORD);
            expect(tokens[0].value).toBe('measure');
            expect(tokens[2].type).toBe(TokenType.ARROW);
        });
    });

    describe('Full Program', () => {
        it('should tokenize a complete Bell state program', () => {
            const source = `
program BellState
qreg q[2]
creg c[2]
H q[0]
CNOT q[0], q[1]
measure q -> c
            `;
            const lexer = new Lexer(source);
            const tokens = lexer.tokenize();

            // Should have many tokens
            expect(tokens.length).toBeGreaterThan(10);

            // Last token should be EOF
            expect(tokens[tokens.length - 1].type).toBe(TokenType.EOF);
        });
    });
});
