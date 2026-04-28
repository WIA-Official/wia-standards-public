# WIA-QUA-002 — Phase 4: Integration

> Quantum-algorithm integration with quantum-supremacy
> benchmarks, implementation guidance for cloud-quantum
> providers (IBM Quantum, Amazon Braket, Azure Quantum, Google
> Quantum AI), and the standards / academic references that
> ground the entire specification.

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

## A.1 Quantum supremacy benchmark integration

The quantum supremacy benchmarks (Phase 4 §9) integrate with
the standard via the `supremacy_witness` envelope. A host
publishing a supremacy witness MUST include the random-circuit
seed, the cross-entropy benchmarking score, and the classical
verification cost estimate. Verifiers compute cross-entropy
against the host's own published distribution to detect
fabrication.

## A.2 Cloud-quantum provider bridge

The Phase 4 bridge documents the mapping from this standard's
envelopes to each major cloud-quantum providers native form:

| Provider | Native form | Bridge endpoint |
|----------|-------------|-----------------|
| IBM Quantum | OpenQASM 3.0 + Qiskit | `bridges/ibmq.json` |
| Amazon Braket | OpenQASM 3.0 + Braket SDK | `bridges/braket.json` |
| Azure Quantum | Q# | `bridges/azure.json` |
| Google Quantum AI | Cirq | `bridges/cirq.json` |

Each bridge ships a translator at
`https://github.com/WIA-Official/wia-qua-bridges` that round-trips
the standards envelopes through the providers SDK and back.

## A.3 Implementation guidance for new providers

A new cloud-quantum provider integrates by publishing a
`provider_attestation` envelope listing supported gate kinds,
maximum circuit depth, available noise models, and the providers
native API endpoint. The standard does not require the provider
to adopt the JSON envelope as its primary surface; it requires
only that a published bridge exists so workloads can target the
provider through the standard.

## A.4 Compatibility with NIST PQC

Phase 4 explicitly tracks the NIST Post-Quantum Cryptography
process. Quantum algorithms that solve PQC primitives (Shors
on RSA / DSA / ECDSA being the canonical example) emit
`pqc_threat_envelope` records that downstream defenders can
consume to plan migration. The standard does not endorse a
specific PQC suite; it endorses the migration discipline
(hybrid signatures, deprecation windows per RFC 8594 / 9745).

## A.5 Conformance and roadmap

| Version | Focus |
|---------|-------|
| 1.0.0 | Initial publication: Phases 1-4 stable, conformance suite v1, four cloud-provider bridges |
| 1.1.x | Additive: more algorithms (HHL, quantum walk, QSVT), additional bridges |
| 1.2.x | Additive: fault-tolerant compilation profile |
| 2.0.0 (no earlier than 2028) | Possible breaking change: post-quantum signature suite migration on the envelope layer |

The standard is maintained by the WIA Standards Committee.
Change proposals follow the WIA RFC process: anyone may submit
a proposal; the Committee reviews quarterly; accepted proposals
enter an open comment period before merging into a minor-version
release. Breaking changes require a two-thirds Committee vote
plus a 12-month deprecation window per IETF RFC 8594 and 9745.

弘益人間 — Benefit All Humanity.

## A.6 Security threat model

The standard adopts a layered threat model for quantum-algorithm
deployments:

- **Adversary controls the cloud-quantum provider's classical
  control plane.** Mitigation: every envelope is signed by the
  workload's identity before submission; the providers
  classical plane cannot forge a signed envelope.
- **Adversary observes the physical quantum hardware via
  side-channel.** Mitigation: noise models in `noise_model`
  reflect hardware characterisation; workloads with strong
  side-channel concerns target hardware that publishes
  characterisation evidence (Phase 1 §A.1).
- **Adversary replays a previous protocol exchange.**
  Mitigation: 96-bit nonce + 300 s skew window + persistent
  600 s seen-nonce cache (Phase 3 §A.3).
- **Adversary substitutes a malicious decoder.** Mitigation:
  decoder verdicts are signed; auditors verify decoder identity
  against a trust list maintained by the host operator.

## A.7 Cross-standard composition

The standard composes with other WIA family standards:

- **WIA-OMNI-API** for credential storage and identity
- **WIA-AIR-SHIELD** for runtime quantum-key distribution
  (composes with `quantum-communication` standard)
- **WIA-INTENT** for declaring algorithmic intent at the
  outermost layer of a workload
- **WIA-SOCIAL** Phase 3 §5 federation handshake reused
  verbatim for cross-host protocol exchanges

## A.8 Hardware-aware compilation guidance

Every cloud-quantum provider exposes a topology graph (qubit
connectivity). Conformant compilers MUST honour the topology
when mapping logical to physical qubits; SWAP insertion is
the standard pattern when the logical circuit demands gates
on non-adjacent physical qubits. The conformance suite checks
that compiled circuits respect the declared topology.

## A.9 References

- IETF RFC 8032 — Ed25519 signature
- IETF RFC 8785 — JSON Canonicalization Scheme
- IETF RFC 8446 — TLS 1.3
- IETF RFC 8594 — deprecation header
- IETF RFC 9745 — sunset header
- NIST FIPS 203 / 204 / 205 — Post-Quantum Cryptography
- ISO/IEC 18033 — Encryption algorithms
- IEEE 754 — floating-point arithmetic
- W3C DID Core — decentralised identifiers

## A.10 Audit and observability

Conformant hosts emit Prometheus-style metrics on a separate
port; the metric set is declared in Phase 2 §2.13. The metric
set for Phase 4 integrations:

| Metric | Type | Labels |
|--------|------|--------|
| `wia_qua_circuits_executed_total` | counter | `target_backend` |
| `wia_qua_shots_consumed_total` | counter | `target_backend`, `algorithm` |
| `wia_qua_circuit_depth_histogram` | histogram | `target_backend` |
| `wia_qua_decoder_latency_seconds` | histogram | `decoder_kind` |
| `wia_qua_supremacy_witnesses_published_total` | counter | `host_id` |

Telemetry MUST NOT include high-cardinality labels (per-circuit
identifiers, per-shot bitstrings, individual customer
identifiers). Hosts violating this are refused at conformance.

## A.11 Privacy considerations

Quantum algorithms operating on private data (e.g., quantum
machine learning over a private training set) MUST use the
audience controls inherited from the WIA-SOCIAL Phase 3
standard. The host's data-classification declaration appears
in the workload envelope and the consumer's policy engine
matches the declared audience before processing.

## A.12 Closing note

Quantum algorithms are leaving the research bench. Cloud-
quantum providers ship usable hardware; hybrid quantum-classical
workloads are appearing in production. The dominant pattern
today is per-vendor silos; this standard exists to provide an
open, federation-aware, audit-logged wire format that lets
vendors, customers, and regulators share workloads on terms
each can verify.

## A.13 Bridging to OpenQASM 3.0 in detail

The OpenQASM 3.0 bridge is the canonical bridge because every
major cloud-quantum provider speaks OpenQASM 3.0 either
natively or via a vendor SDK that emits it. The bridge
translates:

- Phase 1 gate envelopes → OpenQASM 3.0 gate keywords
- Phase 1 circuit envelopes → OpenQASM 3.0 program with
  classical control flow
- Phase 1 measurement-record envelopes → OpenQASM 3.0
  `bit[N] c; measure q -> c;` syntax with a downstream parser
  for the post-execution result

Round-trip equality is part of the conformance test suite.

## A.14 Bridging to QIR (Quantum Intermediate Representation)

QIR is Microsoft's LLVM-derived quantum intermediate
representation. The bridge translates Phase 1 envelopes to
QIR for hosts that prefer compiled bytecode over textual
source. The bridge also reverse-translates QIR back to
Phase 1 envelopes so workloads can flow either direction
through the host pipeline.

## A.15 Future-proofing for fault-tolerant compilation

When the cloud-quantum providers reach fault-tolerant scale
(~1M physical qubits per backend, projected for the late
2020s), the standard's QEC protocol (Phase 3) becomes the
canonical input rather than the bare circuit. The standard's
envelope hierarchy is designed to absorb this transition:
the QEC envelopes become the dominant traffic, and the
underlying circuit envelopes become an implementation detail
hidden by the QEC layer.

## Closing implementer note for PHASE-4-INTEGRATION

The Phase 4 integration story is what keeps the standard from becoming yet another silo; the bridges to OpenQASM 3.0 and QIR mean a workload authored against this standard can target every major cloud-quantum provider without rewriting.

A reference implementation of every Phase mentioned in this
specification ships in the conformance container at
`wia/quantum-algorithm-host:1.0.0` and is the recommended
starting point for new integrators. Customising the container
to a host operators specific cloud-quantum provider is
typically a one-week project for an experienced quantum-software
engineer.
