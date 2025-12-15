/**
 * Backend Exports
 *
 * Unified export for all quantum computing backend code generators.
 */

export { QiskitBackend, generateQiskit } from './qiskit';
export { CirqBackend, generateCirq } from './cirq';
export { QSharpBackend, generateQSharp } from './qsharp';
export { BraketBackend, generateBraket } from './braket';
export { PyQuilBackend, generatePyQuil } from './pyquil';

import { QIRProgram, CompilationResult, BackendTarget } from '../types';
import { generateQiskit } from './qiskit';
import { generateCirq } from './cirq';
import { generateQSharp } from './qsharp';
import { generateBraket } from './braket';
import { generatePyQuil } from './pyquil';

/**
 * Generate code for a specific backend target
 */
export function generateCode(
    qir: QIRProgram,
    target: BackendTarget,
    options?: { includeComments?: boolean }
): CompilationResult {
    switch (target) {
        case 'qiskit':
            return generateQiskit(qir, options);
        case 'cirq':
            return generateCirq(qir, options);
        case 'qsharp':
            return generateQSharp(qir, options);
        case 'braket':
            return generateBraket(qir, options);
        case 'pyquil':
            return generatePyQuil(qir, options);
        default:
            throw new Error(`Unknown backend target: ${target}`);
    }
}

/**
 * Generate code for all supported backends
 */
export function generateAll(
    qir: QIRProgram,
    options?: { includeComments?: boolean }
): Map<BackendTarget, CompilationResult> {
    const results = new Map<BackendTarget, CompilationResult>();
    const targets: BackendTarget[] = ['qiskit', 'cirq', 'qsharp', 'braket', 'pyquil'];

    for (const target of targets) {
        results.set(target, generateCode(qir, target, options));
    }

    return results;
}
