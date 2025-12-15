/**
 * Example: Compile WIA-QL to all backends
 *
 * Run with: npx ts-node examples/compile.ts
 */

import { WiaQuantumCompiler, BackendTarget } from '../src';
import * as fs from 'fs';
import * as path from 'path';

// Read Bell State example
const bellStateSource = `
program BellState

qreg q[2]
creg c[2]

H q[0]
CNOT q[0], q[1]
measure q -> c
`;

console.log('='.repeat(60));
console.log('WIA-QUANTUM Compiler Example');
console.log('='.repeat(60));
console.log('');
console.log('Source (WIA-QL):');
console.log('-'.repeat(40));
console.log(bellStateSource);

// Create compiler
const compiler = new WiaQuantumCompiler({
    includeComments: true,
    optimize: true,
});

// Compile to all backends
const targets: BackendTarget[] = ['qiskit', 'cirq', 'qsharp', 'braket', 'pyquil'];

for (const target of targets) {
    console.log('');
    console.log('='.repeat(60));
    console.log(`Target: ${target.toUpperCase()}`);
    console.log('='.repeat(60));

    try {
        const result = compiler.compile(bellStateSource, target);

        console.log('');
        console.log('Generated Code:');
        console.log('-'.repeat(40));
        console.log(result.code);

        console.log('');
        console.log('Statistics:');
        console.log(`  Total Gates: ${result.stats.totalGates}`);
        console.log(`  Single Qubit: ${result.stats.singleQubitGates}`);
        console.log(`  Two Qubit: ${result.stats.twoQubitGates}`);
        console.log(`  Measurements: ${result.stats.measurements}`);
        console.log(`  Circuit Depth: ${result.stats.circuitDepth}`);
        console.log(`  Qubit Count: ${result.stats.qubitCount}`);

        if (result.warnings.length > 0) {
            console.log('');
            console.log('Warnings:');
            for (const warning of result.warnings) {
                console.log(`  - ${warning.message}`);
            }
        }
    } catch (error) {
        console.error(`Error compiling to ${target}:`, error);
    }
}

console.log('');
console.log('='.repeat(60));
console.log('Compilation complete!');
console.log('='.repeat(60));
