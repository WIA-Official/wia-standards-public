# WIA-QUA-005 — Phase 1: Data Format

> Quantum-simulation canonical envelopes: state-vector, density-matrix, and tensor-network records, plus the runtime conventions that fix the wire format for every simulation method below.

## 1. Introduction

### 1.1 Purpose

This specification defines the theoretical framework and computational methods for simulating quantum systems on classical computers, enabling research, development, and validation of quantum algorithms.

### 1.2 Scope

The standard covers:
- State vector and density matrix representations
- Quantum gate operations and circuits
- Tensor network decompositions
- Quantum chemistry calculations
- Noise and error modeling
- Variational algorithms
- Performance benchmarks

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to make quantum simulation accessible to all researchers, educators, and developers, accelerating quantum technology development for the benefit of humanity.

### 1.4 Terminology

- **Qubit**: Quantum bit, the fundamental unit of quantum information
- **State Vector**: Complex vector representing a pure quantum state
- **Density Matrix**: Matrix representing a mixed quantum state
- **Entanglement**: Quantum correlation between subsystems
- **Fidelity**: Measure of similarity between quantum states
- **Gate Fidelity**: Accuracy of quantum gate implementation

---


## 2. State Vector Simulation

### 2.1 Quantum State Representation

A quantum state of n qubits is represented by a complex vector in 2^n dimensional Hilbert space:

```
|ψ⟩ = Σᵢ₌₀^(2ⁿ⁻¹) αᵢ |i⟩
```

Where:
- `αᵢ ∈ ℂ` (complex amplitudes)
- `Σᵢ |αᵢ|² = 1` (normalization)
- `|i⟩` = computational basis state

### 2.2 State Vector Storage

**Memory requirement**: `M = 2^n × 16 bytes`

For n qubits:
- Each amplitude: 16 bytes (8 bytes real + 8 bytes imaginary)
- Total entries: 2^n
- Example: 30 qubits ≈ 16 GB RAM

**Storage format**:
```
amplitudes = [α₀, α₁, α₂, ..., α₂ⁿ₋₁]
```

Stored as interleaved real/imaginary components:
```
[Re(α₀), Im(α₀), Re(α₁), Im(α₁), ...]
```

### 2.3 Single-Qubit Gates

General single-qubit gate:
```
U = [a  b]
    [c  d]
```

Where `ad - bc = 1` (unitarity for determinant 1).

**Common gates**:

#### 2.3.1 Pauli Gates
```
X = [0  1]    Y = [0  -i]    Z = [1   0]
    [1  0]        [i   0]        [0  -1]
```

#### 2.3.2 Hadamard Gate
```
H = 1/√2 [1   1]
         [1  -1]
```

#### 2.3.3 Phase Gates
```
S = [1  0]    T = [1  0      ]
    [0  i]        [0  e^(iπ/4)]
```

#### 2.3.4 Rotation Gates
```
Rx(θ) = [cos(θ/2)    -i·sin(θ/2)]
        [-i·sin(θ/2)  cos(θ/2)   ]

Ry(θ) = [cos(θ/2)   -sin(θ/2)]
        [sin(θ/2)    cos(θ/2)]

Rz(θ) = [e^(-iθ/2)  0        ]
        [0           e^(iθ/2) ]
```

### 2.4 Two-Qubit Gates

**CNOT (Controlled-NOT)**:
```
CNOT = [1  0  0  0]
       [0  1  0  0]
       [0  0  0  1]
       [0  0  1  0]
```

**CZ (Controlled-Z)**:
```
CZ = [1  0  0   0]
     [0  1  0   0]
     [0  0  1   0]
     [0  0  0  -1]
```

**SWAP**:
```
SWAP = [1  0  0  0]
       [0  0  1  0]
       [0  1  0  0]
       [0  0  0  1]
```

### 2.5 Gate Application Algorithm

For a single-qubit gate U on qubit k:

```python
def apply_single_qubit_gate(state, gate, qubit):
    n = log2(len(state))
    stride = 2^qubit

    for i in range(2^(n-1)):
        # Calculate indices
        idx0 = insert_zero_bit(i, qubit)
        idx1 = idx0 | stride

        # Apply gate
        tmp0 = state[idx0]
        tmp1 = state[idx1]

        state[idx0] = gate[0,0] * tmp0 + gate[0,1] * tmp1
        state[idx1] = gate[1,0] * tmp0 + gate[1,1] * tmp1
```

**Complexity**: O(2^n) time, O(1) additional memory

### 2.6 Measurement Simulation

**Computational basis measurement** on qubit k:

```python
def measure_qubit(state, qubit):
    # Calculate probability of |0⟩
    prob_0 = 0
    for i in range(len(state)):
        if (i >> qubit) & 1 == 0:
            prob_0 += abs(state[i])**2

    # Sample outcome
    outcome = 0 if random() < prob_0 else 1

    # Collapse state
    norm = sqrt(prob_0 if outcome == 0 else (1 - prob_0))
    for i in range(len(state)):
        if ((i >> qubit) & 1) != outcome:
            state[i] = 0
        else:
            state[i] /= norm

    return outcome
```

---


## 3. Density Matrix Methods

### 3.1 Density Matrix Representation

The density matrix represents mixed quantum states:

```
ρ = Σᵢ pᵢ |ψᵢ⟩⟨ψᵢ|
```

**Properties**:
- Hermitian: `ρ† = ρ`
- Positive semi-definite: `⟨ψ|ρ|ψ⟩ ≥ 0` for all `|ψ⟩`
- Trace one: `Tr(ρ) = 1`
- Purity: `Tr(ρ²) ≤ 1`

### 3.2 Storage Requirements

**Memory**: `M = 2^(2n) × 16 bytes`

For n qubits:
- Matrix dimensions: 2^n × 2^n
- Complex entries: 2^(2n)
- Example: 15 qubits ≈ 16 GB RAM

### 3.3 Quantum Operations

**Unitary evolution**:
```
ρ' = U ρ U†
```

**Measurement in basis {|i⟩}**:
```
p(i) = ⟨i|ρ|i⟩ = ρᵢᵢ
```

**Post-measurement state**:
```
ρ' = (|i⟩⟨i|) ρ (|i⟩⟨i|) / p(i)
```

### 3.4 Partial Trace

To trace out subsystem B from system AB:

```
ρA = TrB(ρAB) = Σₖ (IᴬA ⊗ ⟨k|B) ρAB (IA ⊗ |k⟩B)
```

**Algorithm**:
```python
def partial_trace(rho, dims, trace_over):
    # rho: density matrix
    # dims: list of subsystem dimensions
    # trace_over: indices of systems to trace out

    remaining = [i for i in range(len(dims)) if i not in trace_over]
    dim_keep = prod([dims[i] for i in remaining])

    rho_reduced = zeros((dim_keep, dim_keep), dtype=complex)

    for i in range(dim_keep):
        for j in range(dim_keep):
            # Sum over traced subsystems
            for k in traced_indices(dims, trace_over):
                idx_i = combine_indices(i, k, remaining, trace_over, dims)
                idx_j = combine_indices(j, k, remaining, trace_over, dims)
                rho_reduced[i, j] += rho[idx_i, idx_j]

    return rho_reduced
```

### 3.5 Quantum Channels

A quantum channel is represented by Kraus operators {Kᵢ}:

```
ε(ρ) = Σᵢ Kᵢ ρ Kᵢ†
```

Where `Σᵢ Kᵢ†Kᵢ = I` (completeness).

**Common channels**:

#### 3.5.1 Depolarizing Channel
```
ε(ρ) = (1-p)ρ + p·I/2
```

Kraus operators:
```
K₀ = √(1-3p/4) I
K₁ = √(p/4) X
K₂ = √(p/4) Y
K₃ = √(p/4) Z
```

#### 3.5.2 Amplitude Damping
```
K₀ = [1      0    ]    K₁ = [0  √γ]
     [0  √(1-γ)]           [0   0]
```

#### 3.5.3 Phase Damping
```
K₀ = [1  0         ]    K₁ = [0      0    ]
     [0  √(1-γ)]           [0  √γ]
```

---


## 4. Tensor Network Techniques

### 4.1 Matrix Product States (MPS)

An MPS representation of an n-qubit state:

```
|ψ⟩ = Σ A[1]ⁱ¹ A[2]ⁱ² ... A[n]ⁱⁿ |i₁i₂...iₙ⟩
```

Where:
- `A[k]ⁱᵏ` are χₖ₋₁ × χₖ matrices
- `χₖ` is the bond dimension
- `χ₀ = χₙ = 1`

### 4.2 Bond Dimension

The bond dimension χ controls:
- **Accuracy**: Higher χ → better approximation
- **Memory**: O(nχ²)
- **Time**: O(nχ³) per gate

**Typical values**:
- 1D systems: χ = 10-100
- Slightly entangled: χ = 100-1000
- Highly entangled: χ > 1000

### 4.3 MPS Operations

**Single-site gate**:
```
A[k]ⁱ → Σⱼ U[i,j] A[k]ʲ
```
Complexity: O(χ²)

**Two-site gate** on sites k and k+1:
```
1. Contract A[k] and A[k+1] into θ
2. Apply gate: θ' = (U ⊗ I) θ
3. SVD: θ' = A[k] Σ V†
4. Split: A[k]_new = A[k], A[k+1]_new = Σ V†
5. Truncate to bond dimension χ
```
Complexity: O(χ³)

### 4.4 Canonical Forms

**Left-canonical** (A):
```
Σᵢ A[k]ⁱ† A[k]ⁱ = I
```

**Right-canonical** (B):
```
Σᵢ B[k]ⁱ B[k]ⁱ† = I
```

**Mixed-canonical** form with center at site k:
```
|ψ⟩ = A[1] ... A[k-1] C[k] B[k+1] ... B[n]
```

### 4.5 PEPS (Projected Entangled Pair States)

For 2D systems, PEPS generalizes MPS:

```
Each tensor: A[x,y]ⁱ has 5 indices (i, up, down, left, right)
Bond dimensions: χ
Memory: O(N χ⁴) for N sites
Computation: O(χ⁶) per update
```

---



## A.1 Canonical envelope conventions

Every Phase 1 quantum-simulation envelope follows the WIA family
baseline: UTF-8 JSON with RFC 8785 canonicalisation, Ed25519
signatures (IETF RFC 8032), ULIDs as identifiers. Quantum amplitudes
are encoded as IEEE 754 double-precision pairs (real, imaginary)
with the shortest round-trip representation.

## A.2 State-vector envelope

```json
{
  "wia_qsim_version": "1.0.0",
  "type": "state_vector",
  "vector_id": "sv_01HX...",
  "n_qubits": 8,
  "amplitudes_layout": "lexicographic_msb",
  "amplitudes_b64": "<base64 packed pairs>",
  "norm_check": 0.9999999998,
  "computed_at": "RFC 3339",
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

State vectors above n=20 qubits are typically too large to ship
inline; the host returns a chunked stream for vectors at scale.

## A.3 Density-matrix envelope

```json
{
  "wia_qsim_version": "1.0.0",
  "type": "density_matrix",
  "matrix_id": "dm_01HX...",
  "n_qubits": 6,
  "trace_check": 1.0,
  "purity": 0.94,
  "elements_b64": "<base64 N×N complex matrix>"
}
```

Purity = Tr(ρ²) is included as a sanity check; pure states have
purity 1.0 within numerical precision.

## A.4 Tensor-network envelope

Tensor-network representations (MPS, PEPS, MERA) carry the network
topology, the bond dimensions, and the per-tensor data with a hash
commitment so re-computation produces byte-identical artefacts.

## A.5 Chemistry-input envelope

Quantum-chemistry simulations consume molecular Hamiltonians; the
input envelope carries atom positions, basis-set choice, and
active-space selection so the trained simulator output is reproducible.


## Z.1 Glossary

The companion glossary at `https://wiastandards.com/quantum-simulation/glossary/`
expands every term used throughout this Phase. Implementers
unfamiliar with the domain should treat it as load-bearing reading.

## Z.2 Cross-standard composition

This Phase composes with: **WIA-OMNI-API** (credential storage),
**WIA-AIR-SHIELD** (runtime trust list), **WIA-SOCIAL Phase 3 §5**
(federation handshake), and **WIA-INTENT** (workload intent
declaration).

## Z.3 Conformance test suite + reference container

A black-box conformance test suite at
`https://github.com/WIA-Official/wia-quantum-simulation-conformance` walks
every public endpoint and protocol exchange. The reference
container at `wia/quantum-simulation-host:1.0.0` implements every Phase 2
endpoint with mock data so integrators exercise their bridge
before production. The companion CLI at `cli/quantum-simulation.sh` ships
sample envelope generators (validate, info, plus phase-specific
subcommands) so an implementer can produce conformant payloads
without hand-rolling JSON.

## Z.4 Implementation runbook

A first implementation typically follows: (1) stand up reference
container, (2) run conformance suite against it, (3) replace mock
backend with real backend one endpoint at a time, (4) wire up audit
log replication, (5) onboard a single trusted peer for federation,
(6) expand to multiple peers, (7) promote to production with
warning-envelope subscription.

## Z.5 Backwards-compatibility promise + governance

Within the 1.x line every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable.
Hosts MAY add optional fields and new envelopes; hosts MUST NOT
remove existing ones. Breaking changes ride a major version bump
with a 12-month deprecation window per IETF RFC 8594 / 9745, and
require a two-thirds Committee vote.

弘益人間 — Benefit All Humanity.


## A.8 Reproducibility envelope

```json
{
  "type": "reproducibility_envelope",
  "run_id": "run_01HX...",
  "random_seed": 12345,
  "noise_model_hash": "sha256:...",
  "input_hash": "sha256:...",
  "framework_version": "wia-qsim/1.0.0",
  "backend": "local-statevector",
  "started_at": "RFC 3339"
}
```

Re-running with the same envelope on the same backend MUST produce
byte-identical results; deviation indicates a non-determinism bug.

## A.9 Computed-result envelope

The computed-result envelope wraps state-vector, density-matrix,
or tensor-network outputs with a hash commitment over the result
bytes plus a runtime-statistics block. Consumers verify the hash
before further processing.


## A.10 Glossary

State vector: complex amplitude vector representing pure quantum
states; size 2^n for n qubits. Density matrix: positive semi-definite
operator describing both pure and mixed states; size 2^n × 2^n.
Tensor network: factorised representation reducing memory cost from
exponential to polynomial in many cases. MPS (Matrix Product State):
1D tensor network with bounded bond dimension. PEPS (Projected
Entangled Pair State): 2D tensor network. MERA (Multi-scale
Entanglement Renormalisation Ansatz): hierarchical tensor network
for critical systems. UCCSD (Unitary Coupled Cluster with Singles
and Doubles): chemistry ansatz with documented expressibility.
Trotter-Suzuki splitting: time-evolution decomposition trading
circuit depth against approximation error. Active space:
chemistry orbital subset included in the quantum part of a
hybrid calculation.

## A.11 Cross-standard composition

This Phase composes with: WIA-OMNI-API for credentials,
WIA-AIR-SHIELD for trust list, WIA-SOCIAL Phase 3 §5 for federation,
WIA-INTENT for workload intent declaration, and the WIA quantum-algorithm
companion standard for circuits whose simulation feeds into this
Phase. The composition is intentional: a host running multiple
WIA family standards reuses one identity, signature, and audit
machinery rather than maintaining N parallel implementations.


## Z.1 Worked deployment trace

A typical first-90-day deployment of this standard at a host
operator follows the trace below:

```
Day 0    : Reference container stood up in dev environment.
Day 1-2  : Conformance suite passes against reference container.
Day 3-7  : Backend bridge implemented for the host's primary tool.
Day 8-10 : Conformance suite passes against bridged backend.
Day 11-15: Audit log replication wired up; first envelope chain audited.
Day 16-20: First federation peer onboarded; trust list cadence verified.
Day 21-30: Federation expanded to 3-5 peers; cross-peer audit verified.
Day 31-60: Production traffic shadow-routed through new stack.
Day 61-90: Cutover from legacy to new stack; legacy retained as
           fallback for the deprecation window.
```

The 90-day timeline accommodates conformance-suite passes,
operations-team training, and the regulator-notification cadence
typical for high-stakes deployments. Lighter deployments (small
operators, prototypes) compress this to 30 days.

## Z.2 Operations runbook excerpt

Day-to-day operations focus on three signals: (a) audit-log
replication lag — alarm if either replica falls more than 60s
behind the primary; (b) trust-list freshness — alarm 7 days
before any peer's signed list expires; (c) replay-cache footprint
— alarm if cache memory exceeds 80% of the documented budget.

The runbook also covers incident response: rotating signing keys
on suspected compromise, replaying the seen-nonce cache from
persistent storage on standby failover, and re-issuing federation
handshakes when the primary controller has been offline for
longer than the seen-nonce window.
