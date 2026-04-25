# WIA-QUA-002: Quantum Algorithm Specification v1.0

> **Standard ID:** WIA-QUA-002
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Quantum Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Quantum Gates](#2-quantum-gates)
3. [Quantum Circuits](#3-quantum-circuits)
4. [Shor's Algorithm](#4-shors-algorithm)
5. [Grover's Algorithm](#5-grovers-algorithm)
6. [Variational Quantum Eigensolver (VQE)](#6-variational-quantum-eigensolver-vqe)
7. [Quantum Approximate Optimization Algorithm (QAOA)](#7-quantum-approximate-optimization-algorithm-qaoa)
8. [Quantum Error Correction](#8-quantum-error-correction)
9. [Quantum Supremacy Benchmarks](#9-quantum-supremacy-benchmarks)
10. [Implementation Guidelines](#10-implementation-guidelines)
11. [References](#11-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines the mathematical framework and computational methods for quantum algorithms, enabling standardized implementation across quantum computing platforms.

### 1.2 Scope

The standard covers:
- Quantum gate operations and matrices
- Quantum circuit construction and simulation
- Key quantum algorithms (Shor, Grover, VQE, QAOA)
- Quantum error correction codes
- Benchmarking and performance metrics

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard democratizes access to quantum computing by providing clear, implementable specifications for quantum algorithms that can solve real-world problems.

### 1.4 Terminology

- **Qubit**: Quantum bit, basis states |0⟩ and |1⟩
- **Superposition**: Linear combination of basis states
- **Entanglement**: Non-classical correlation between qubits
- **Gate**: Unitary operation on qubits
- **Measurement**: Projection onto computational basis
- **Decoherence**: Loss of quantum information

---

## 2. Quantum Gates

### 2.1 Single-Qubit Gates

#### 2.1.1 Pauli Gates

**Pauli-X (NOT gate)**:
```
X = |0⟩⟨1| + |1⟩⟨0| = [0 1]
                       [1 0]
```

**Pauli-Y**:
```
Y = -i|0⟩⟨1| + i|1⟩⟨0| = [0 -i]
                          [i  0]
```

**Pauli-Z (Phase flip)**:
```
Z = |0⟩⟨0| - |1⟩⟨1| = [1  0]
                        [0 -1]
```

#### 2.1.2 Hadamard Gate

Creates superposition:
```
H = (|0⟩⟨0| + |0⟩⟨1| + |1⟩⟨0| - |1⟩⟨1|)/√2

H = 1/√2 [1  1]
         [1 -1]

H|0⟩ = (|0⟩ + |1⟩)/√2
H|1⟩ = (|0⟩ - |1⟩)/√2
```

#### 2.1.3 Phase Gates

**S-gate (Phase gate)**:
```
S = [1 0]
    [0 i]

S|1⟩ = i|1⟩
```

**T-gate (π/8 gate)**:
```
T = [1    0        ]
    [0  e^(iπ/4)   ]

T|1⟩ = e^(iπ/4)|1⟩
```

#### 2.1.4 Rotation Gates

**Rotation around X-axis**:
```
Rx(θ) = [cos(θ/2)   -i·sin(θ/2)]
        [-i·sin(θ/2)  cos(θ/2) ]
```

**Rotation around Y-axis**:
```
Ry(θ) = [cos(θ/2)  -sin(θ/2)]
        [sin(θ/2)   cos(θ/2)]
```

**Rotation around Z-axis**:
```
Rz(θ) = [e^(-iθ/2)     0     ]
        [0          e^(iθ/2) ]
```

### 2.2 Multi-Qubit Gates

#### 2.2.1 CNOT Gate

Controlled-NOT (2-qubit):
```
CNOT = |0⟩⟨0| ⊗ I + |1⟩⟨1| ⊗ X

CNOT = [1 0 0 0]
       [0 1 0 0]
       [0 0 0 1]
       [0 0 1 0]

CNOT|00⟩ = |00⟩
CNOT|01⟩ = |01⟩
CNOT|10⟩ = |11⟩
CNOT|11⟩ = |10⟩
```

#### 2.2.2 SWAP Gate

Exchange qubit states:
```
SWAP = [1 0 0 0]
       [0 0 1 0]
       [0 1 0 0]
       [0 0 0 1]

SWAP|01⟩ = |10⟩
SWAP|10⟩ = |01⟩
```

#### 2.2.3 Toffoli Gate

Controlled-Controlled-NOT (3-qubit):
```
Toffoli = |00⟩⟨00| ⊗ I + |01⟩⟨01| ⊗ I + |10⟩⟨10| ⊗ I + |11⟩⟨11| ⊗ X

Toffoli|110⟩ = |111⟩
Toffoli|111⟩ = |110⟩
```

#### 2.2.4 Controlled-Z Gate

```
CZ = |0⟩⟨0| ⊗ I + |1⟩⟨1| ⊗ Z

CZ = [1  0  0  0]
     [0  1  0  0]
     [0  0  1  0]
     [0  0  0 -1]
```

### 2.3 Universal Gate Sets

Any quantum computation can be approximated to arbitrary precision using:

**Clifford + T**:
- Gates: {H, S, CNOT, T}
- Universal for quantum computation
- Suitable for fault-tolerant quantum computing

**Rotation-based**:
- Gates: {Rx, Ry, Rz, CNOT}
- Continuous parameter space
- Used in variational algorithms

---

## 3. Quantum Circuits

### 3.1 Circuit Representation

A quantum circuit is a sequence of gates applied to qubits:

```
|ψ₀⟩ → [Gate₁] → [Gate₂] → ... → [Gateₙ] → Measurement
```

### 3.2 Quantum State Evolution

**Initial state** (n qubits):
```
|ψ₀⟩ = |0⟩^⊗n = |00...0⟩
```

**After gate sequence**:
```
|ψ_final⟩ = Uₙ·Uₙ₋₁·...·U₂·U₁|ψ₀⟩
```

Where Uᵢ are unitary gate operations.

### 3.3 Measurement

**Computational basis measurement**:
```
P(outcome i) = |⟨i|ψ⟩|²
```

**Expectation value**:
```
⟨O⟩ = ⟨ψ|O|ψ⟩
```

For observable O.

### 3.4 Bell State Creation

Creating maximally entangled state:
```
Circuit: H(0) → CNOT(0,1)

|00⟩ → H(0) → (|00⟩ + |10⟩)/√2 → CNOT → (|00⟩ + |11⟩)/√2

Result: Bell state |Φ⁺⟩ = (|00⟩ + |11⟩)/√2
```

---

## 4. Shor's Algorithm

### 4.1 Problem Statement

Given integer N, find prime factors p and q such that:
```
N = p × q
```

### 4.2 Algorithm Overview

1. **Classical preprocessing**: Choose random a < N
2. **Quantum period finding**: Find period r of f(x) = aˣ mod N
3. **Classical postprocessing**: Extract factors from r

### 4.3 Quantum Fourier Transform (QFT)

Core subroutine for period finding:

```
QFT|j⟩ = 1/√N ∑ₖ e^(2πijk/N)|k⟩
```

**QFT Circuit**:
```
For n qubits:
1. Apply Hadamard to qubit 0
2. Apply controlled phase rotations
3. Repeat for remaining qubits
4. Apply SWAP gates for bit reversal
```

**QFT Matrix** (4-dimensional example):
```
QFT₄ = 1/2 [1   1   1   1 ]
           [1   i  -1  -i ]
           [1  -1   1  -1 ]
           [1  -i  -1   i ]
```

### 4.4 Period Finding Circuit

```
1. Initialize: |0⟩^⊗n|1⟩^⊗m
2. Apply Hadamard to first register: (∑ₓ|x⟩)|1⟩/√2ⁿ
3. Apply Uₐ: (∑ₓ|x⟩|aˣ mod N⟩)/√2ⁿ
4. Measure second register → collapses to period
5. Apply QFT to first register
6. Measure → get period r (with high probability)
```

### 4.5 Factor Extraction

Given period r:
```
If r is even and aʳ/² ≠ -1 (mod N):
  gcd(aʳ/² - 1, N) is a non-trivial factor
  gcd(aʳ/² + 1, N) is a non-trivial factor
```

### 4.6 Complexity

**Quantum**: O((log N)²(log log N)(log log log N))
**Classical (best known)**: O(exp(∛((log N)(log log N)²)))

**Speedup**: Exponential

### 4.7 Example: Factoring 15

```
N = 15, choose a = 7
Period of 7ˣ mod 15: r = 4

7¹ mod 15 = 7
7² mod 15 = 4
7³ mod 15 = 13
7⁴ mod 15 = 1  ← period is 4

gcd(7² - 1, 15) = gcd(48, 15) = 3
gcd(7² + 1, 15) = gcd(50, 15) = 5

Factors: 15 = 3 × 5 ✓
```

---

## 5. Grover's Algorithm

### 5.1 Problem Statement

Search unstructured database of N items for marked item(s):
```
Given: f(x) where f(x) = 1 for marked items, 0 otherwise
Find: x such that f(x) = 1
```

### 5.2 Algorithm Overview

1. **Initialize**: Equal superposition over all states
2. **Oracle**: Mark target states with phase flip
3. **Diffusion**: Amplify amplitude of marked states
4. **Iterate**: Repeat ~√N times
5. **Measure**: Find marked state with high probability

### 5.3 Grover Operator

```
G = (2|ψ⟩⟨ψ| - I) · Oᶠ

Where:
- Oᶠ is oracle: Oᶠ|x⟩ = (-1)^f(x)|x⟩
- |ψ⟩ = H^⊗n|0⟩^⊗n (uniform superposition)
- 2|ψ⟩⟨ψ| - I is inversion about average
```

### 5.4 Number of Iterations

For M marked items out of N total:
```
θ = arcsin(√(M/N))
k ≈ π/(4θ) = π/4 × √(N/M)

Optimal iterations: k
```

For single marked item (M=1):
```
k ≈ π/4 × √N
```

### 5.5 Success Probability

After k iterations:
```
P(success) = sin²((2k+1)θ)

For optimal k: P(success) ≥ 1 - 1/N
```

### 5.6 Oracle Construction

For simple database search:
```
Oracle Oᶠ: |x⟩|0⟩ → |x⟩|f(x)⟩

Phase kickback with ancilla in |−⟩:
Oᶠ|x⟩|−⟩ = (-1)^f(x)|x⟩|−⟩
```

### 5.7 Diffusion Operator

```
D = 2|ψ⟩⟨ψ| - I
  = H^⊗n(2|0⟩⟨0| - I)H^⊗n

Implementation:
1. Apply H^⊗n
2. Apply phase flip on |0⟩: X-gates, multi-controlled Z, X-gates
3. Apply H^⊗n
```

### 5.8 Complexity

**Quantum**: O(√N)
**Classical**: O(N)

**Speedup**: Quadratic

### 5.9 Example: 8-item Search

```
N = 8, target index = 6 (110 in binary)

Iterations: π/4 × √8 ≈ 2.2 → use 2 iterations

Initial: |ψ⟩ = (|000⟩ + |001⟩ + ... + |111⟩)/√8

After 2 Grover iterations:
Amplitude of |110⟩ ≈ 0.95
Amplitude of others ≈ 0.05 each

Measurement: 95% chance to find |110⟩ = 6
```

---

## 6. Variational Quantum Eigensolver (VQE)

### 6.1 Problem Statement

Find ground state energy of Hamiltonian H:
```
E₀ = min_|ψ⟩ ⟨ψ|H|ψ⟩
```

### 6.2 Algorithm Overview

1. **Prepare ansatz**: |ψ(θ)⟩ with parameters θ
2. **Measure energy**: ⟨ψ(θ)|H|ψ(θ)⟩
3. **Optimize classically**: Update θ to minimize energy
4. **Iterate**: Repeat until convergence

### 6.3 Ansatz Design

**Hardware-efficient ansatz**:
```
|ψ(θ)⟩ = U_ent(θ₂) U_rot(θ₁) |0⟩^⊗n

U_rot(θ): Single-qubit rotations
U_ent(θ): Entangling gates (CNOT ladder)
```

**UCCSD ansatz** (chemistry):
```
|ψ(θ)⟩ = exp(∑ᵢ θᵢ(Tᵢ - Tᵢ†))|HF⟩

Where:
- |HF⟩ is Hartree-Fock reference
- Tᵢ are excitation operators
```

### 6.4 Energy Measurement

Decompose Hamiltonian into Pauli terms:
```
H = ∑ᵢ αᵢ Pᵢ

Where Pᵢ ∈ {I, X, Y, Z}^⊗n

⟨H⟩ = ∑ᵢ αᵢ ⟨Pᵢ⟩
```

Measure each ⟨Pᵢ⟩ separately.

### 6.5 Classical Optimization

Use gradient-based or gradient-free optimizers:
- **Gradient descent**: θ ← θ - η∇E(θ)
- **COBYLA**: Constrained optimization
- **BFGS**: Quasi-Newton method
- **SPSA**: Simultaneous perturbation

### 6.6 Example: H₂ Molecule

```
Hamiltonian (2 qubits):
H = -1.05 I + 0.39 Z₀ - 0.39 Z₁ - 0.01 Z₀Z₁ + 0.18 X₀X₁

Ansatz:
|ψ(θ₁,θ₂)⟩ = Ry(θ₂,1) CNOT(0,1) Ry(θ₁,0) |00⟩

Optimal: θ₁ = 0.54, θ₂ = -1.14
Ground state energy: E₀ = -1.857 Ha
```

---

## 7. Quantum Approximate Optimization Algorithm (QAOA)

### 7.1 Problem Statement

Solve combinatorial optimization:
```
max_x f(x)  where x ∈ {0,1}ⁿ

Encode as: min_x ⟨x|C|x⟩
```

### 7.2 Algorithm Overview

1. **Problem Hamiltonian**: HC encoding cost function
2. **Mixer Hamiltonian**: HB = ∑ᵢ Xᵢ
3. **Parameterized circuit**: U(β,γ) = e^(-iβHB)e^(-iγHC)
4. **Iterate**: Apply p layers
5. **Optimize**: Classical optimization of (β,γ)

### 7.3 QAOA Circuit

```
|ψ(β,γ)⟩ = U(βₚ,γₚ)···U(β₁,γ₁)|+⟩^⊗n

Where:
- |+⟩^⊗n is uniform superposition
- p is number of layers (depth)
```

### 7.4 Cost Hamiltonian

For MaxCut problem on graph G=(V,E):
```
HC = ∑_{(i,j)∈E} (1 - ZᵢZⱼ)/2

Measures number of cut edges
```

### 7.5 Mixing Hamiltonian

```
HB = ∑ᵢ Xᵢ

Induces transitions between computational basis states
```

### 7.6 Parameter Optimization

```
Maximize: F(β,γ) = ⟨ψ(β,γ)|HC|ψ(β,γ)⟩

Use classical optimizer:
- Gradient descent
- COBYLA
- Nelder-Mead
```

### 7.7 Approximation Ratio

For MaxCut with p layers:
```
Guarantee: F ≥ 0.6924 × OPT (p=1)
Improves with larger p
```

### 7.8 Example: 4-node MaxCut

```
Graph: 0—1—2—3 (ring)
       └─────┘

HC = (1-Z₀Z₁)/2 + (1-Z₁Z₂)/2 + (1-Z₂Z₃)/2 + (1-Z₃Z₀)/2

p = 1, optimal (β,γ):
β ≈ 0.39
γ ≈ 0.31

Expected cut: ≈ 3.97 (classical optimum: 4)
```

---

## 8. Quantum Error Correction

### 8.1 Shor Code

**Encoding**: 9 physical qubits → 1 logical qubit

```
|0_L⟩ = (|000⟩ + |111⟩)(|000⟩ + |111⟩)(|000⟩ + |111⟩) / 2√2
|1_L⟩ = (|000⟩ - |111⟩)(|000⟩ - |111⟩)(|000⟩ - |111⟩) / 2√2
```

**Corrects**: Any single-qubit error (bit flip OR phase flip)

**Syndrome measurement**:
```
Bit flip: Measure Z₀Z₁, Z₁Z₂ on each triple
Phase flip: Measure X₀X₁X₂X₃X₄X₅X₆X₇X₈
```

### 8.2 Steane Code

**Encoding**: 7 physical qubits → 1 logical qubit

```
Stabilizer generators:
S₁ = I ⊗ I ⊗ I ⊗ X ⊗ X ⊗ X ⊗ X
S₂ = I ⊗ X ⊗ X ⊗ I ⊗ I ⊗ X ⊗ X
S₃ = X ⊗ I ⊗ X ⊗ I ⊗ X ⊗ I ⊗ X
(+ 3 more for Z)
```

**Properties**:
- CSS code (Calderbank-Shor-Steane)
- Distance 3
- Transversal Clifford gates

### 8.3 Surface Code

**Encoding**: d² physical qubits → 1 logical qubit (distance d)

```
Lattice layout:
Data qubits on vertices
Syndrome qubits on faces/edges

Stabilizers:
- Plaquette: X₁X₂X₃X₄ (face operators)
- Star: Z₁Z₂Z₃Z₄ (vertex operators)
```

**Properties**:
- High threshold: ~1%
- Local interactions (2D lattice)
- Scalable architecture

### 8.4 Error Detection

**Syndrome extraction**:
```
1. Measure stabilizers without collapsing data
2. Compute syndrome s = (s₁, s₂, ..., sₖ)
3. Identify error from syndrome table
4. Apply correction
```

### 8.5 Fault-Tolerant Gates

**Transversal gates**:
```
Logical gate = ⊗ Physical gates

Example: Logical CNOT = CNOT⊗7 (Steane code)
```

**Magic state distillation**:
```
For non-Clifford gates (T-gate):
1. Prepare noisy magic state
2. Distill to high fidelity
3. Consume for T-gate via teleportation
```

---

## 9. Quantum Supremacy Benchmarks

### 9.1 Random Circuit Sampling

**Task**: Sample from output distribution of random quantum circuit

**Metrics**:
- Circuit depth
- Number of qubits
- Gate fidelity
- Cross-entropy benchmarking

**Threshold**: Classical simulation becomes infeasible

### 9.2 Quantum Volume

```
VQ = 2ⁿ

Where n = min(number of qubits, circuit depth)

Requirements:
- Heavy output generation >2/3
- Gate fidelity sufficiently high
```

### 9.3 CLOPS (Circuit Layer Operations Per Second)

```
CLOPS = K × L / T

Where:
- K: Number of quantum circuits
- L: Number of layers per circuit
- T: Time to execute
```

### 9.4 Fidelity Metrics

**Process fidelity**:
```
F = ⟨ψ_ideal|ρ_actual|ψ_ideal⟩
```

**Average gate fidelity**:
```
F_avg = ∫ dψ ⟨ψ|U†E(|ψ⟩⟨ψ|)U|ψ⟩
```

---

## 10. Implementation Guidelines

### 10.1 API Interface

#### 10.1.1 Quantum Circuit

```typescript
interface QuantumCircuit {
  numQubits: number;
  gates: Gate[];

  hadamard(qubit: number): void;
  pauliX(qubit: number): void;
  pauliY(qubit: number): void;
  pauliZ(qubit: number): void;

  cnot(control: number, target: number): void;
  toffoli(control1: number, control2: number, target: number): void;

  measure(): number[];
  getState(): ComplexVector;
}
```

#### 10.1.2 Quantum Algorithms

```typescript
interface QuantumAlgorithms {
  shorFactorize(N: number): number[];
  groverSearch(oracle: (x: number) => boolean, N: number): number;
  runVQE(hamiltonian: Operator, ansatz: Circuit): number;
  runQAOA(costHamiltonian: Operator, layers: number): BitString;
}
```

### 10.2 Gate Implementation

All gates must satisfy unitarity:
```
U†U = UU† = I
```

Verify with:
```typescript
function isUnitary(U: Matrix): boolean {
  const UDagger = conjugateTranspose(U);
  const product = matrixMultiply(UDagger, U);
  return isIdentity(product, tolerance=1e-10);
}
```

### 10.3 State Vector Simulation

```typescript
class StateVectorSimulator {
  state: ComplexVector;  // 2^n dimensional

  applyGate(gate: Matrix, qubits: number[]): void {
    const fullGate = expandGate(gate, qubits, this.numQubits);
    this.state = matrixVectorMultiply(fullGate, this.state);
  }

  measure(): number[] {
    const probs = this.state.map(amp => abs(amp) ** 2);
    return sample(probs);
  }
}
```

### 10.4 Error Handling

Standard error codes:

| Code | Meaning | Action |
|------|---------|--------|
| Q001 | Invalid qubit index | Check circuit size |
| Q002 | Non-unitary gate | Verify gate matrix |
| Q003 | Measurement before preparation | Initialize state |
| Q004 | Insufficient qubits | Increase circuit size |
| Q005 | Factorization failed | Retry with different a |
| Q006 | Oracle not provided | Define oracle function |

---

## 11. References

### 11.1 Foundational Papers

1. Shor, P.W. (1997). "Polynomial-Time Algorithms for Prime Factorization and Discrete Logarithms on a Quantum Computer"
2. Grover, L.K. (1996). "A Fast Quantum Mechanical Algorithm for Database Search"
3. 선행 연구. "A Variational Eigenvalue Solver on a Photonic Quantum Processor"
4. 선행 연구. "A Quantum Approximate Optimization Algorithm"
5. Shor, P.W. (1995). "Scheme for Reducing Decoherence in Quantum Computer Memory"

### 11.2 Quantum Constants

| Constant | Symbol | Value |
|----------|--------|-------|
| Planck constant | h | 6.626 × 10⁻³⁴ J·s |
| Reduced Planck | ℏ | 1.055 × 10⁻³⁴ J·s |
| Rydberg constant | R∞ | 1.097 × 10⁷ m⁻¹ |
| Bohr radius | a₀ | 5.292 × 10⁻¹¹ m |
| Fine structure constant | α | 1/137.036 |

### 11.3 WIA Standards

- WIA-QUA-001: Quantum Computing Fundamentals
- WIA-QUA-003: Quantum Machine Learning
- WIA-SECURITY: Post-Quantum Cryptography
- WIA-CLOUD: Quantum Cloud Computing

---

## Appendix A: Example Calculations

### A.1 Shor's Algorithm: Factoring 21

```
N = 21, choose a = 2

Period finding:
2¹ mod 21 = 2
2² mod 21 = 4
2³ mod 21 = 8
2⁴ mod 21 = 16
2⁵ mod 21 = 11
2⁶ mod 21 = 1  ← period r = 6

Factor extraction:
gcd(2³ - 1, 21) = gcd(7, 21) = 7
gcd(2³ + 1, 21) = gcd(9, 21) = 3

Result: 21 = 3 × 7 ✓
```

### A.2 Grover's Algorithm: 16-item Database

```
N = 16, M = 1 (one marked item)

Iterations:
k = π/4 × √16 = π ≈ 3.14

Use 3 iterations

Success probability:
P = sin²((2×3+1)arcsin(1/4))
P ≈ 0.945 (94.5%)
```

---

**弘익人間 (홍익인간) · Benefit All Humanity**

*WIA-QUA-002 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
