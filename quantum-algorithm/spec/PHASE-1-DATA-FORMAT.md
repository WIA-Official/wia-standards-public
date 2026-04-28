# WIA-QUA-002 — Phase 1: Data Format

> Quantum-algorithm canonical envelopes: gate, circuit,
> measurement record, and the runtime conventions that fix the
> wire format of every algorithm in subsequent phases. The
> mathematics of single-qubit and multi-qubit gates is the
> bedrock of this phase because every API and protocol below
> assumes a shared gate algebra.

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


## A.1 Canonical envelope conventions

Every Phase 1 envelope conforms to the following baseline:

- **Encoding**: UTF-8 JSON with the canonical form defined by RFC 8785 (JSON Canonicalization Scheme); signatures cover the canonical bytes.
- **Signature suite**: Ed25519 (IETF RFC 8032) over the canonical JSON form. Verifiers apply the standard `crypto_sign_open` primitive; constant-time verification is mandatory.
- **Identifiers**: ULIDs (Crockford base-32, 26 characters) for every envelope identity; the lexicographic order of ULIDs gives a usable secondary index by issuance time without disclosing wall-clock time precisely.
- **Numeric precision**: amplitudes and angles are encoded as IEEE 754 double-precision; producers MUST emit the shortest round-trip representation (`%.17g`) and consumers MUST parse with the same standard.
- **Error model**: `noise_model` references reference noise channels parameterised by gate-time and decoherence rates; depolarising, amplitude-damping, and dephasing channels are reserved tokens.

## A.2 Gate envelope worked example

A single Hadamard on logical qubit 0 of register `q` in a 5-qubit circuit:

```json
{
  "wia_qua_version": "1.0.0",
  "type": "gate",
  "gate_id": "01HXY...",
  "gate_kind": "H",
  "register": "q",
  "qubit_indices": [0],
  "params": {},
  "applied_at": "circuit_step:0",
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

A two-qubit CNOT with control 0, target 1:

```json
{
  "wia_qua_version": "1.0.0",
  "type": "gate",
  "gate_kind": "CX",
  "register": "q",
  "qubit_indices": [0, 1],
  "applied_at": "circuit_step:1",
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

A parameterised single-qubit rotation about the X axis:

```json
{
  "wia_qua_version": "1.0.0",
  "type": "gate",
  "gate_kind": "RX",
  "register": "q",
  "qubit_indices": [3],
  "params": { "theta": 1.5707963267948966 },
  "applied_at": "circuit_step:7",
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

## A.3 Circuit-envelope completeness invariant

Every circuit envelope MUST satisfy the following invariant before
publication: the union of `qubit_indices` across all gate steps is
a subset of the declared register size, and the set of measurement
steps does not write to a qubit after it has been reset within the
same step. Violations are refused at verifier time with a problem
document of type `.../circuit-malformed`.

## A.4 Measurement record envelope

A measurement record carries the bit string outcome, the shot
count, and the per-shot wall-clock issuance band. Shot strings
are encoded as compact bitfields (LSB = qubit 0) for transport
efficiency; verifiers expand them on demand.

## A.5 Compatibility with OpenQASM 3.0

The Phase 1 envelopes round-trip cleanly to OpenQASM 3.0 source.
A reference translator at
`https://github.com/WIA-Official/wia-qua-conformance` maps every
gate kind in this Phase to its OpenQASM 3.0 keyword and re-parses
the source back into Phase 1 envelopes. Round-trip equality (gate
kind, register, qubit indices, parameters) is part of the
conformance suite.

弘益人間 — Benefit All Humanity.

## B.1 Single-qubit gate algebra reference

The standard pins the matrix conventions for every single-qubit
gate to remove ambiguity across vendor SDKs:

```
I  = [[1, 0], [0, 1]]
X  = [[0, 1], [1, 0]]                 # NOT
Y  = [[0, -i], [i, 0]]                # bit + phase flip
Z  = [[1, 0], [0, -1]]                # phase flip
H  = (1/√2)[[1, 1], [1, -1]]          # Hadamard
S  = [[1, 0], [0, i]]                 # √Z
T  = [[1, 0], [0, e^{iπ/4}]]          # √S

RX(θ) = [[cos(θ/2), -i sin(θ/2)],
         [-i sin(θ/2), cos(θ/2)]]
RY(θ) = [[cos(θ/2), -sin(θ/2)],
         [sin(θ/2),  cos(θ/2)]]
RZ(θ) = [[e^{-iθ/2}, 0],
         [0, e^{iθ/2}]]
```

Producers of envelopes encoding parameterised rotations MUST
emit angles in radians on the [-2π, 2π] interval; consumers
reduce to the canonical interval before applying.

## B.2 Two-qubit gate algebra reference

```
CNOT (CX) with control c, target t:
  acts as |c,t⟩ → |c, t ⊕ c⟩

CZ with control c, target t:
  acts as |c,t⟩ → (-1)^{c·t} |c,t⟩

SWAP between qubits a,b:
  acts as |a,b⟩ → |b,a⟩

iSWAP between qubits a,b:
  acts as |01⟩ → i|10⟩ , |10⟩ → i|01⟩

CRX(θ), CRY(θ), CRZ(θ) controlled rotations:
  apply RX(θ) / RY(θ) / RZ(θ) to target conditioned on control = 1
```

## B.3 Multi-qubit gate decomposition reference

The standard does not require hardware-native support for
arbitrary multi-qubit gates; instead, every multi-qubit gate
decomposes into a sequence of one- and two-qubit primitives
that is part of the conformance test suite. The decomposition
is published at `https://github.com/WIA-Official/wia-qua-gate-decomp`
and is the authoritative reference for compiler implementers.

## B.4 Measurement basis declarations

Every measurement step declares a basis explicitly; the default
is the computational (Z) basis. Other declared bases:

- `X` — Hadamard then Z-basis measurement
- `Y` — S† then H then Z-basis measurement
- `Bell` — CNOT then H on control then Z-basis measurement
  on both qubits
- `parity` — sum of Z-basis outcomes mod 2

## C.1 JSON Schema validation

The Phase 1 envelopes ship with JSON Schema 2020-12 documents
at `https://wiastandards.com/quantum-algorithm/schemas/`. Every
conformant producer MUST validate envelopes against the schema
before signing; signed envelopes that fail validation are
considered malformed and MUST be refused at verifier time.

## C.2 Envelope versioning

The `wia_qua_version` field follows semantic versioning with
the discipline that:

- **Patch** versions add no required fields, may add optional
  fields with documented defaults; consumers ignore unknown
  optional fields silently.
- **Minor** versions add new envelope types; consumers ignore
  unknown types silently and SHOULD log them for operator
  review.
- **Major** versions remove or rename fields; major version
  bumps ride a 12-month deprecation window per IETF RFC 8594
  and 9745.

## C.3 Forbidden envelope shapes

The following envelope shapes are explicitly forbidden and MUST
be refused at verifier time with a problem document of type
`.../forbidden-envelope`:

- A circuit declaring a register of size 0 or with a duplicate
  register name
- A gate referencing a qubit index outside the register size
- A measurement step that targets a qubit reset in the same
  step (the standard requires reset-then-measure to be in
  separate steps)
- A signed envelope whose `alg` field is anything other than
  `Ed25519` (Phase 4 introduces post-quantum hybrid signatures
  via an additive minor-version bump; consumers refusing
  unknown `alg` is forward-compatible)

## C.4 Discovery document expectations

Each host publishes a discovery document at
`/.well-known/wia-quantum-algorithm` with:

- Supported gate kinds (allows compilers to refuse unsupported
  kinds early)
- Supported algorithm endpoints
- Maximum N for Shors factoring on each backend
- Supported ansatz families for VQE
- Supported decoder families for the QEC protocol layer
- Trust-anchor reference for envelope signatures
- Federation peers (if the host participates in a federation)

Discovery documents are cacheable for 300 seconds via
`Cache-Control: public, max-age=300`. Clients refresh on the
documented cadence; hosts emit warning envelopes when their
trust-anchor or federation peer set changes.

## Closing implementer note for PHASE-1-DATA-FORMAT

Phase 1 envelopes are the contract every consumer relies on; future minor versions will add envelope types but never remove or rename existing fields, in line with the backwards-compatibility promise documented in Phase 4 §A.5.

A reference implementation of every Phase mentioned in this
specification ships in the conformance container at
`wia/quantum-algorithm-host:1.0.0` and is the recommended
starting point for new integrators. Customising the container
to a host operators specific cloud-quantum provider is
typically a one-week project for an experienced quantum-software
engineer.
