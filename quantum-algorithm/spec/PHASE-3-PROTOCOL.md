# WIA-QUA-002 — Phase 3: Protocol

> Quantum optimisation protocol (QAOA) plus the error-correction
> protocol that every NISQ-era algorithm needs to ride to
> fault-tolerant scale. Both protocols are wire-level — the
> sender encodes one form, the receiver decodes another, with
> well-defined failure modes when the channel exceeds noise
> budgets.

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


## A.1 QAOA-protocol operational guidance

QAOA is a two-protocol layer cake: an outer classical
optimisation loop (typically Nelder-Mead or COBYLA on the
parameter vector) and an inner quantum sampling step that
returns expectation values to feed the outer loop. The
protocol is sensitive to barren-plateau effects when the
parameter dimension grows; conformant implementations MUST
document the parameter-initialisation strategy used (random,
warm-start from classical relaxation, fixed-angle).

The protocol's wire format:

```
Iteration N:
  classical → quantum: parameters [β_1..β_p, γ_1..γ_p]
  quantum   → classical: ⟨H_C⟩ estimate, shot count, noise budget
Repeat until ‖ΔE‖ < tolerance or iteration budget exhausted.
```

## A.2 Quantum error-correction protocol

The QEC protocol layer below QAOA wraps every logical operation
in a code-distance encoding (the standard recommends surface code
for two-dimensional architectures). Syndrome measurements are
emitted as `syndrome_record` envelopes that the decoder consumes
in real time; the decoder's choice (minimum-weight perfect
matching, neural decoder, or union-find) is declared in the
host's discovery document.

Logical error rates below the surface-code threshold (~1%)
qualify the host for the `qec.fault_tolerant` capability flag;
hosts above the threshold MUST NOT advertise the flag and MUST
log every decoder failure to the audit envelope.

## A.3 Replay defence and signed protocol exchanges

Every protocol envelope (parameter update, syndrome record,
expectation estimate) carries a 96-bit nonce and an RFC 3339
timestamp. Receivers reject envelopes with skew > ±300 s and
maintain a 600-second seen-nonce cache. The cache is persistent
across host restarts so a power cycle does not re-open the
window for a previously-blocked replay.

## A.4 Cross-vendor protocol interop

When a workload spans multiple cloud-quantum providers (e.g.,
syndrome decoding on one and parameter optimisation on another),
the protocol envelopes survive the boundary because they are
vendor-neutral. The conformance suite includes a multi-vendor
test that runs QAOA with parameter optimisation on the local
host and circuit execution against a mock cloud-quantum
provider, exercising the full envelope round-trip.

弘益人間 — Benefit All Humanity.

## A.5 QAOA worked example: Max-Cut on a 4-vertex graph

A 4-vertex weighted graph with edges (0,1), (1,2), (2,3), (3,0)
weighted [1, 1, 1, 1] gives the cost Hamiltonian:

```
H_C = (1 - Z_0 Z_1)/2 + (1 - Z_1 Z_2)/2
    + (1 - Z_2 Z_3)/2 + (1 - Z_3 Z_0)/2
```

A depth-1 QAOA ansatz with parameters (β, γ) prepares:

```
|ψ(β, γ)⟩ = e^{-iβ Σ X_j} e^{-iγ H_C} |+⟩^4
```

The classical optimiser searches the (β, γ) plane for the
minimum ⟨H_C⟩; the optimal angles for this graph are
known analytically and serve as the conformance test
ground truth.

## A.6 Surface-code distance-3 worked example

A logical qubit encoded in distance-3 surface code uses 9
data qubits plus 8 syndrome qubits (4 X-type, 4 Z-type)
arranged on a 3×3 lattice. The protocol publishes:

```
syndrome_record:
  cycle: 0
  X-syndromes: [0, 1, 0, 0]
  Z-syndromes: [0, 0, 1, 0]
  decoder_verdict: "single-qubit Z error on data qubit 4"
  applied_correction: "Z[4]"
```

Each decoder verdict is signed by the decoder service so
auditors can reconstruct the correction history later.

## A.7 Audit logging for protocol exchanges

Every protocol exchange (parameter update, syndrome record,
QAOA expectation estimate) is logged to an append-only audit
store with the signing identity, the canonical envelope hash,
and the wall-clock timestamp. Audit entries are replicated
across at least two storage backends so a verifier can
reconstruct the full exchange history independently.

## A.8 Failure modes catalogue

| Failure | Detected by | Operator response |
|---------|-------------|-------------------|
| Decoder timeout | host watchdog | mark logical qubit unhealthy; rerun from last checkpoint |
| Optimiser stagnation | iteration counter exhaustion | warm-start from best-so-far, restart with new initialisation |
| Excessive shot budget | rate-limit accounting | abort with problem document; suggest reducing depth or shots |
| Cross-vendor protocol drift | conformance suite | refuse the bridge until both sides reconcile envelope versions |

## A.9 Decoder selection guide

The protocol allows three decoder families:

- **Minimum-weight perfect matching (MWPM)** — the canonical
  decoder for surface code; has predictable latency profile,
  scales O(N^3) in worst case but typically O(N) on real
  syndrome graphs. Reference implementation: PyMatching at
  `https://github.com/oscarhiggott/PyMatching`.
- **Union-find decoder** — near-linear-time decoder with
  slightly worse logical accuracy than MWPM, suitable for
  high-throughput real-time decoding. Reference: Delfosse-Nickerson
  union-find as documented in arxiv:1709.06218 (cite as
  arxiv preprint reference, not the standards canon).
- **Neural decoder** — trained on syndrome / correction pairs;
  faster inference than MWPM for some topologies but requires
  per-hardware retraining. Hosts using neural decoders MUST
  publish the training-data envelope and the validation
  threshold for transparency.

The protocol does not endorse a specific decoder; it requires
that the host's choice is declared in the discovery document
and that audit envelopes carry the decoder identifier so a
reviewer can verify the decision chain after the fact.

## A.10 Cross-protocol interaction with QKD

When a quantum-algorithm workload coexists with a
quantum-key-distribution channel (the `quantum-communication`
companion standard), the protocol exchanges share a common
trust list. A QKD-derived key may be rotated into the
algorithm's signing identity per the WIA-AIR-SHIELD rotation
discipline; the rotation is itself a signed protocol envelope
so auditors can reconstruct the key history.

## A.11 QAOA ansatz family reference

The standard recognises four ansatz families with documented
parameter counts and depth profiles:

- **Standard QAOA** — alternating cost and mixer Hamiltonians
  with depth-`p` parameters (`β_1..β_p`, `γ_1..γ_p`);
  parameter count `2p`; depth scales linearly with `p`.
- **Multi-angle QAOA (ma-QAOA)** — separate mixer angles per
  qubit; parameter count `(n+m)·p` where `m` is edge count;
  better expressivity at higher parameter cost.
- **Warm-start QAOA** — initial state seeded from a classical
  relaxation (typically Goemans-Williamson for Max-Cut);
  parameter count identical to standard QAOA but converges
  faster on average.
- **Adaptive QAOA (ADAPT-QAOA)** — operator pool grows during
  optimisation; reports the operator sequence chosen so the
  workload is reproducible.

Conformant hosts declare the ansatz family in the discovery
document; consumers can require a specific family when their
workload depends on it.

## A.12 Surface-code resource estimation reference

Logical error rate for surface code at code distance `d` with
physical error rate `p` (below threshold):

```
P_L ≈ A · (p / p_th)^((d+1)/2)
```

where `A` is a fitting constant (≈ 0.03 for typical
implementations) and `p_th` is the surface-code threshold
(≈ 0.01). At `p = 1e-3` and `d = 11`, `P_L ≈ 1e-9` per logical
operation, which suffices for early fault-tolerant workloads
of moderate depth.

The standard does not require a specific code distance; it
requires that the chosen distance is declared and that the
estimated logical error rate is published in the host's
attestation envelope so workload owners can refuse hosts whose
estimates do not meet their tolerance.

## A.13 Decoder performance benchmark suite

The conformance test suite includes a decoder benchmark that
measures:

- Logical error rate at a fixed code distance and physical
  error rate
- Decoder latency at a given syndrome volume per second
- Decoder throughput sustained over a 1-minute window
- Memory footprint at 5-minute steady state

The benchmark publishes results at
`https://wiastandards.com/quantum-algorithm/decoder-bench/`
so customers can compare decoders across hosts before
committing to a specific deployment.

## Closing implementer note for PHASE-3-PROTOCOL

The Phase 3 protocol layer is the highest-stakes part of the standard because it defines the wire-level exchange that crosses trust boundaries; the replay defence and audit-log discipline are mandatory rather than recommended.

A reference implementation of every Phase mentioned in this
specification ships in the conformance container at
`wia/quantum-algorithm-host:1.0.0` and is the recommended
starting point for new integrators. Customising the container
to a host operators specific cloud-quantum provider is
typically a one-week project for an experienced quantum-software
engineer.
