# WIA-QUANTUM TypeScript SDK

> Unified Quantum Computing Standard
>
> 홍익인간 (弘益人間) - Benefit All Humanity

[![npm version](https://badge.fury.io/js/%40wia%2Fquantum.svg)](https://www.npmjs.com/package/@wia/quantum)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## Overview

WIA-QUANTUM provides a unified programming interface for quantum computing that compiles to multiple quantum computing platforms:

- **IBM Qiskit** (Python)
- **Google Cirq** (Python)
- **Microsoft Q#**
- **Amazon Braket** (Python)
- **Rigetti PyQuil** (Python)

Write once, run on any quantum computer.

## Installation

```bash
npm install @wia/quantum
```

## Quick Start

```typescript
import { compile } from '@wia/quantum';

const source = `
program BellState
qreg q[2]
creg c[2]
H q[0]
CNOT q[0], q[1]
measure q -> c
`;

// Compile to IBM Qiskit
const qiskit = compile(source, 'qiskit');
console.log(qiskit.code);

// Compile to Google Cirq
const cirq = compile(source, 'cirq');
console.log(cirq.code);

// Compile to all backends
import { WiaQuantumCompiler } from '@wia/quantum';
const compiler = new WiaQuantumCompiler();
const all = compiler.compileAll(source);
```

## WIA-QL Language

WIA-QL (WIA Quantum Language) is a hardware-agnostic quantum assembly language.

### Syntax

```
program ProgramName

// Registers
qreg q[4]      // Quantum register with 4 qubits
creg c[4]      // Classical register with 4 bits

// Single qubit gates
H q[0]         // Hadamard
X q[1]         // Pauli-X
Y q[2]         // Pauli-Y
Z q[3]         // Pauli-Z
S q[0]         // Phase gate
T q[0]         // T gate

// Parameterized gates
RX(1.5708) q[0]      // Rotation X
RY(3.1416) q[0]      // Rotation Y
RZ(0.7854) q[0]      // Rotation Z
U(θ, φ, λ) q[0]      // Universal gate

// Two qubit gates
CNOT q[0], q[1]      // Controlled-NOT
CZ q[0], q[1]        // Controlled-Z
SWAP q[0], q[1]      // Swap gate

// Three qubit gates
CCX q[0], q[1], q[2] // Toffoli
CSWAP q[0], q[1], q[2] // Fredkin

// Measurement
measure q -> c       // Measure all
measure q[0] -> c[0] // Measure single
```

### Supported Gates

| Gate | Description | Qubits |
|------|-------------|--------|
| I | Identity | 1 |
| X | Pauli-X (NOT) | 1 |
| Y | Pauli-Y | 1 |
| Z | Pauli-Z | 1 |
| H | Hadamard | 1 |
| S | Phase (√Z) | 1 |
| T | π/8 gate | 1 |
| SDG | S† (S-dagger) | 1 |
| TDG | T† (T-dagger) | 1 |
| RX | X-rotation | 1 |
| RY | Y-rotation | 1 |
| RZ | Z-rotation | 1 |
| U | Universal | 1 |
| P | Phase | 1 |
| CNOT/CX | Controlled-NOT | 2 |
| CY | Controlled-Y | 2 |
| CZ | Controlled-Z | 2 |
| SWAP | Swap | 2 |
| ISWAP | iSWAP | 2 |
| CCX/CCNOT | Toffoli | 3 |
| CSWAP | Fredkin | 3 |

## API Reference

### `compile(source, target)`

Quick compile function.

```typescript
const result = compile(source, 'qiskit');
```

**Parameters:**
- `source`: WIA-QL source code
- `target`: Backend target (`'qiskit'` | `'cirq'` | `'qsharp'` | `'braket'` | `'pyquil'`)

**Returns:** `CompilationResult`

### `WiaQuantumCompiler`

Main compiler class with full options.

```typescript
const compiler = new WiaQuantumCompiler({
    target: 'qiskit',
    includeComments: true,
    optimize: true,
    validateCircuit: true,
});

// Parse to AST
const ast = compiler.parse(source);

// Convert to QIR
const qir = compiler.toQIR(ast);

// Compile to target
const result = compiler.compile(source, 'cirq');

// Compile to all targets
const allResults = compiler.compileAll(source);
```

### `CompilationResult`

```typescript
interface CompilationResult {
    target: string;
    code: string;
    warnings: CompilerWarning[];
    stats: CompilationStats;
}

interface CompilationStats {
    totalGates: number;
    singleQubitGates: number;
    twoQubitGates: number;
    threeQubitGates: number;
    measurements: number;
    circuitDepth: number;
    qubitCount: number;
}
```

## Examples

### Bell State

```typescript
const bellState = `
program BellState
qreg q[2]
creg c[2]
H q[0]
CNOT q[0], q[1]
measure q -> c
`;

const result = compile(bellState, 'qiskit');
```

### Grover's Algorithm

```typescript
const grover = `
program GroverSearch
qreg q[3]
creg c[3]

// Initialize
H q[0]
H q[1]
H q[2]

// Oracle
CCX q[0], q[1], q[2]

// Diffusion
H q[0]
H q[1]
H q[2]
X q[0]
X q[1]
X q[2]
CCX q[0], q[1], q[2]
X q[0]
X q[1]
X q[2]
H q[0]
H q[1]
H q[2]

measure q -> c
`;
```

### Quantum Teleportation

```typescript
const teleportation = `
program QuantumTeleportation
qreg q[3]
creg c[2]

// Prepare state to teleport
H q[0]

// Create Bell pair
H q[1]
CNOT q[1], q[2]

// Bell measurement
CNOT q[0], q[1]
H q[0]
measure q[0] -> c[0]
measure q[1] -> c[1]
`;
```

## Architecture

```
WIA-QL Source
     │
     ▼
┌─────────┐
│  Lexer  │  Tokenization
└────┬────┘
     │
     ▼
┌─────────┐
│ Parser  │  AST Generation
└────┬────┘
     │
     ▼
┌─────────┐
│   QIR   │  Intermediate Representation
└────┬────┘
     │
     ▼
┌─────────────────────────────────────┐
│           Code Generators           │
├─────────┬────────┬────────┬────────┤
│ Qiskit  │  Cirq  │   Q#   │ Braket │
└─────────┴────────┴────────┴────────┘
```

## Development

```bash
# Install dependencies
npm install

# Build
npm run build

# Test
npm test

# Lint
npm run lint

# Type check
npm run typecheck
```

## Contributing

Contributions are welcome! Please read our contributing guidelines.

## License

MIT License - see [LICENSE](LICENSE) for details.

## About WIA

World Certification Industry Association (WIA) creates open standards that benefit all humanity.

- Website: https://wia.family
- GitHub: https://github.com/WIA-Official

---

> 홍익인간 (弘益人間)
>
> "Benefit All Humanity" - Ancient Korean philosophy
