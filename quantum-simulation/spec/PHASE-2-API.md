# WIA-QUA-005 — Phase 2: API

> Method-API surface: chemistry simulation, many-body physics, noise modelling, and circuit-level simulation as worked APIs that integrators code against.

## 5. Quantum Chemistry Simulation

### 5.1 Electronic Structure Hamiltonian

The molecular Hamiltonian in second quantization:

```
H = Σᵢⱼ hᵢⱼ aᵢ†aⱼ + ½ Σᵢⱼₖₗ hᵢⱼₖₗ aᵢ†aⱼ†aₖaₗ
```

Where:
- `aᵢ†`, `aᵢ` = creation/annihilation operators
- `hᵢⱼ` = one-electron integrals
- `hᵢⱼₖₗ` = two-electron integrals

### 5.2 Jordan-Wigner Transformation

Map fermionic operators to qubits:

```
aᵢ† = (⊗ₖ₌₀ⁱ⁻¹ Zₖ) ⊗ (X - iY)/2
aᵢ  = (⊗ₖ₌₀ⁱ⁻¹ Zₖ) ⊗ (X + iY)/2
```

### 5.3 Hartree-Fock

Self-consistent field method:

```
F Cᵢ = εᵢ S Cᵢ
```

Where:
- `F` = Fock matrix
- `C` = molecular orbital coefficients
- `ε` = orbital energies
- `S` = overlap matrix

**Algorithm**:
```
1. Initial guess for density matrix P
2. Compute Fock matrix: F = H_core + G(P)
3. Solve eigenvalue problem: F C = S C ε
4. Update density: P = C_occ C_occ†
5. Check convergence: ΔE < tolerance
6. If not converged, go to step 2
```

### 5.4 VQE (Variational Quantum Eigensolver)

Find ground state energy:

```
E₀ = min_θ ⟨ψ(θ)|H|ψ(θ)⟩
```

**Algorithm**:
```
1. Prepare ansatz |ψ(θ)⟩
2. Measure ⟨H⟩ = Σᵢ wᵢ ⟨Pᵢ⟩ (Pauli decomposition)
3. Optimize parameters θ using classical optimizer
4. Repeat until convergence
```

**Ansatz examples**:
- Hardware-efficient ansatz
- UCC (Unitary Coupled Cluster)
- UCCSD (UCC Singles and Doubles)

### 5.5 Molecular Integrals

**One-electron integrals**:
```
hᵢⱼ = ∫ φᵢ*(r) [-½∇² - Σ_A Z_A/|r-R_A|] φⱼ(r) dr
```

**Two-electron integrals**:
```
hᵢⱼₖₗ = ∫∫ φᵢ*(r₁)φⱼ(r₁) (1/|r₁-r₂|) φₖ*(r₂)φₗ(r₂) dr₁dr₂
```

---


## 6. Many-Body Physics

### 6.1 Hubbard Model

The Hubbard Hamiltonian:

```
H = -t Σ_⟨i,j⟩,σ (cᵢσ†cⱼσ + h.c.) + U Σᵢ nᵢ↑ nᵢ↓
```

Where:
- `t` = hopping parameter
- `U` = on-site interaction
- `σ` = spin index
- `⟨i,j⟩` = nearest neighbors

### 6.2 Heisenberg Model

Spin-spin interactions:

```
H = Σ_⟨i,j⟩ J (SᵢˣSⱼˣ + SᵢʸSⱼʸ + SᵢᶻSⱼᶻ)
```

For spin-½:
```
H = Σ_⟨i,j⟩ J/4 (XᵢXⱼ + YᵢYⱼ + ZᵢZⱼ)
```

### 6.3 Time Evolution

**Exact evolution** (small systems):
```
|ψ(t)⟩ = exp(-iHt/ℏ) |ψ(0)⟩
```

**Trotter decomposition**:
```
exp(-iHt) ≈ [exp(-iH₁δt) exp(-iH₂δt)]^(t/δt)
```

Error: O((δt)²)

**Higher-order Trotter**:
```
S₂ = exp(-iH₁δt/2) exp(-iH₂δt) exp(-iH₁δt/2)
```
Error: O((δt)³)

### 6.4 Correlation Functions

**Two-point correlator**:
```
C(i,j,t) = ⟨ψ(t)| Oᵢ†Oⱼ |ψ(t)⟩
```

**Dynamical structure factor**:
```
S(q,ω) = ∫ dt e^(iωt) Σᵢⱼ e^(iq(rᵢ-rⱼ)) C(i,j,t)
```

---


## 7. Noise Modeling

### 7.1 Gate Errors

**Coherent error** (unitary):
```
U_actual = U_ideal × U_error
```

**Incoherent error** (depolarizing):
```
ε(ρ) = (1-p) U ρ U† + p I/d
```

### 7.2 Readout Errors

Confusion matrix:
```
M = [p(0|0)  p(0|1)]
    [p(1|0)  p(1|1)]
```

Where `p(i|j)` = probability of measuring i given state j.

### 7.3 Decoherence

**T₁ relaxation** (amplitude damping):
```
γ(t) = 1 - exp(-t/T₁)
```

**T₂ dephasing** (phase damping):
```
γ_φ(t) = 1 - exp(-t/T₂)
```

With `T₂ ≤ 2T₁`.

### 7.4 Noise Simulation

**Kraus operator application**:
```python
def apply_noise(rho, kraus_ops):
    rho_out = zeros_like(rho)
    for K in kraus_ops:
        rho_out += K @ rho @ K.conj().T
    return rho_out
```

**Stochastic unraveling** (for state vectors):
```python
def apply_channel_stochastic(state, kraus_ops):
    # Compute probabilities
    probs = [norm(K @ state)**2 for K in kraus_ops]

    # Sample outcome
    k = categorical_sample(probs)

    # Apply and normalize
    state_out = kraus_ops[k] @ state
    state_out /= norm(state_out)

    return state_out
```

---


## 8. Circuit Simulation

### 8.1 Circuit Representation

A quantum circuit consists of:
- Number of qubits: n
- Sequence of gates: {G₁, G₂, ..., Gₘ}
- Measurements: {M₁, M₂, ..., Mₖ}

**OpenQASM format**:
```qasm
OPENQASM 2.0;
include "qelib1.inc";

qreg q[3];
creg c[3];

h q[0];
cx q[0], q[1];
cx q[0], q[2];

measure q -> c;
```

### 8.2 Circuit Depth

**Depth**: Maximum number of sequential gates on any path

Example:
```
H─┬─X─M
  │
H─X─M
```
Depth = 3

### 8.3 Optimized Simulation

**Gate fusion**:
Combine consecutive single-qubit gates:
```
U_combined = U₃ × U₂ × U₁
```

**Lazy evaluation**:
Build composite operator before applying to state.

**Parallelization**:
Distribute state vector across multiple processors.

---



## A.1 Endpoint reference

```http
POST /qsim/v1/state-vector/evolve   # apply a circuit to a state vector
POST /qsim/v1/density-matrix/evolve  # apply a quantum channel
POST /qsim/v1/tensor/contract        # contract a tensor network
POST /qsim/v1/chemistry/run          # run a quantum-chemistry simulation
POST /qsim/v1/many-body/sample       # sample from a many-body Hamiltonian
POST /qsim/v1/noise/apply            # apply a documented noise channel
```

Every endpoint follows the discovery convention at
`/.well-known/wia-quantum-simulation`.

## A.2 Quantum-chemistry endpoint

```http
POST /qsim/v1/chemistry/run
{
  "molecule": {
    "atoms": [
      { "symbol": "H", "position_angstrom": [0, 0, 0] },
      { "symbol": "H", "position_angstrom": [0.74, 0, 0] }
    ],
    "basis_set": "sto-3g",
    "charge": 0,
    "spin_multiplicity": 1
  },
  "method": "vqe" | "qpe" | "imaginary_time",
  "ansatz": "uccsd",
  "target_backend": "local-statevector"
}
```

Returns the ground-state energy estimate plus the parameter vector.

## A.3 Many-body sampling

The many-body sampling endpoint runs Trotter-Suzuki time evolution
and samples observables. The envelope carries the time-step, the
Trotter order, and the observable basis so consumers can verify the
numerical method is appropriate for their physics.

## A.4 Noise application

Noise channels (depolarising, amplitude damping, dephasing) are
applied via this endpoint to study how quantum information degrades
under realistic conditions. The envelope declares the channel
parameters; the response includes the post-channel state plus the
fidelity drop.

## A.5 Bulk export and audit

For audit and reproducibility, every host exposes a bulk export
endpoint with all envelopes from a time window plus a Merkle root
commitment over the bundle.


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

## A.6 Webhook callbacks

Long-running simulations (chemistry runs taking hours, many-body
sampling at large n) emit webhook callbacks on checkpoint and
completion events. The callback payload is the full signed envelope
with HMAC verification per the standard's webhook discipline.

## A.7 Performance reporting endpoint

```http
GET /qsim/v1/performance/{run_id}

→ 200 OK
{
  "runtime_seconds": 3742,
  "peak_memory_gb": 96,
  "backend_used": "local-statevector",
  "effective_qubits": 24,
  "throughput_circuit_layers_per_second": 142
}
```

Performance envelopes are the basis for capacity-planning decisions
across a fleet of simulation hosts.



## A.8 Endpoint detail — tensor-network contraction

```http
POST /qsim/v1/tensor/contract
{
  "network_id": "tn_01HX...",
  "topology": "matrix_product_state",
  "tensors_b64": "<base64 packed>",
  "bond_dimensions": [16, 16, 16, 16],
  "contraction_order": "auto",
  "target_backend": "local-cpu"
}
```

Returns the contracted tensor plus the per-step contraction error
budget when bond-dimension truncation is applied.

## A.9 Health and observability

```http
GET /qsim/v1/health    → liveness probe
GET /qsim/v1/ready     → readiness probe
GET /qsim/v1/metrics   → Prometheus exposition
```

Health endpoints do not require authentication and do not count
toward rate limits. Metrics endpoints expose aggregate counters
without per-circuit identifiers to preserve customer privacy.


## A.10 Glossary

Endpoint: HTTPS URL accepting envelope POSTs and returning result
envelopes. Webhook: callback URL the host posts to on completion.
Bulk export: time-window-bounded NDJSON stream of envelopes with
a Merkle root commitment. Health probe: liveness / readiness
endpoint outside rate-limit accounting. Metric: Prometheus-style
counter, gauge, or histogram exposed for fleet monitoring.

## A.11 Operational considerations

Quantum-simulation hosts have three operational considerations
integrators consistently underestimate. First, memory scaling:
state-vector simulation doubles memory for each additional qubit;
plan capacity at the smallest qubit count the workload tolerates.
Second, GPU memory bandwidth: state-vector simulation is bandwidth-
bound on GPUs; choose GPUs by HBM bandwidth rather than peak FLOPS.
Third, distributed simulation latency: cross-node communication
dominates runtime above 32 qubits in distributed mode; profile
network latency before scaling out.

## A.12 Implementation runbook

A first implementation typically follows: stand up reference
container; run conformance suite; replace mock backend with real
simulation engine; wire audit log replication; onboard a federation
peer; expand to multiple peers; promote to production.


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
