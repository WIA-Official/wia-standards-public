# WIA-QUA-005 — Phase 3: Protocol

> Hybrid classical-quantum protocol layer plus implementation-level guidance that defines the wire-level exchange between the simulation engine and downstream consumers.

## 9. Hybrid Classical-Quantum

### 9.1 QAOA (Quantum Approximate Optimization Algorithm)

For optimization problem with cost function C(z):

```
|ψ(β,γ)⟩ = U(B,β_p) U(C,γ_p) ... U(B,β₁) U(C,γ₁) |s⟩
```

Where:
- `U(C,γ) = exp(-iγC)`
- `U(B,β) = exp(-iβB)` with `B = Σᵢ Xᵢ`
- `|s⟩ = |+⟩^⊗n` (equal superposition)

### 9.2 Variational Circuits

Parameterized circuit:
```
|ψ(θ)⟩ = U(θₚ) ... U(θ₂) U(θ₁) |0⟩^⊗n
```

**Parameter optimization**:
- Gradient descent
- BFGS, L-BFGS
- COBYLA, Nelder-Mead
- SPSA (Simultaneous Perturbation Stochastic Approximation)

### 9.3 Gradient Calculation

**Parameter shift rule**:
```
∂⟨H⟩/∂θ = r [⟨H⟩_+ - ⟨H⟩_-]
```

Where:
- `⟨H⟩_± = ⟨ψ(θ ± π/4r)|H|ψ(θ ± π/4r)⟩`
- `r` depends on gate type

---


## 10. Implementation Guidelines

### 10.1 Required Components

Any WIA-QUA-005 compliant simulator must include:

1. **State representation**: State vector or density matrix
2. **Gate library**: Common single and two-qubit gates
3. **Measurement**: Computational basis measurements
4. **Circuit builder**: Construct and execute circuits
5. **Expectation values**: Calculate observable expectations

### 10.2 API Interface

#### 10.2.1 State Initialization
```typescript
interface QuantumState {
  numQubits: number;
  method: 'statevector' | 'densitymatrix' | 'mps';
  amplitudes?: Complex[];
  densityMatrix?: Complex[][];
  bondDimension?: number;
}
```

#### 10.2.2 Gate Application
```typescript
interface Gate {
  type: string;
  qubits: number[];
  parameters?: number[];
  matrix?: Complex[][];
}

interface Circuit {
  gates: Gate[];
  numQubits: number;
  apply(state: QuantumState): QuantumState;
}
```

#### 10.2.3 Measurement
```typescript
interface MeasurementResult {
  outcomes: number[];
  probabilities: number[];
  state: QuantumState; // Post-measurement state
}
```

### 10.3 Performance Benchmarks

| Operation | Time Complexity | Memory |
|-----------|-----------------|--------|
| Single-qubit gate | O(2^n) | O(2^n) |
| Two-qubit gate | O(2^n) | O(2^n) |
| Full circuit (m gates) | O(m × 2^n) | O(2^n) |
| Measurement | O(2^n) | O(2^n) |
| Expectation value | O(2^n) | O(2^n) |

### 10.4 Error Handling

Standard error codes:

| Code | Meaning | Action |
|------|---------|--------|
| Q001 | Invalid qubit index | Check qubit range |
| Q002 | Non-unitary gate | Verify gate matrix |
| Q003 | State not normalized | Renormalize state |
| Q004 | Memory exceeded | Reduce qubit count |
| Q005 | Invalid measurement basis | Use valid basis |
| Q006 | Numerical instability | Increase precision |

---



## A.1 Hybrid classical-quantum protocol

The hybrid protocol layer coordinates a classical optimiser (running
on a host CPU) with a quantum simulator (potentially on a different
node, even on real hardware). The protocol exchanges:

```
classical → quantum: parameter_update_envelope
quantum → classical: expectation_estimate_envelope
                     gradient_estimate_envelope (if requested)
                     noise_metrics_envelope (always, for transparency)
```

Each envelope is signed by the originating identity so a third party
can audit the optimisation trajectory after the fact.

## A.2 Implementation guidance

A first deployment typically runs:

- Classical optimiser on the host (Python with SciPy or JAX)
- Quantum simulator in a separate process (C++ or Rust for speed)
- IPC over Unix domain sockets for low-latency exchange
- Persistent audit log replicated to a remote storage backend

The reference container at `wia/quantum-simulation-host:1.0.0`
ships with all four components configured.

## A.3 Convergence and stopping criteria

Conformant hosts publish three stopping criteria in the discovery
document: parameter-vector L² change tolerance, expectation-value
change tolerance, and absolute iteration budget. Sessions hitting
the iteration budget without convergence MUST emit a
`convergence_failure` envelope; consumers that require convergence
refuse the result.

## A.4 Replay defence

Standard 96-bit nonce + 300-second skew window + 600-second seen-nonce
cache applies to every protocol envelope.

## A.5 Cross-host federation

When optimisation runs on one host and quantum simulation runs on
another, the protocol envelopes flow via the WIA-SOCIAL Phase 3 §5
federation handshake. The two hosts establish a signed peer
relationship before exchanging parameter updates and expectation
estimates.


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

## A.6 Trotter-Suzuki time-evolution protocol

Time evolution under Hamiltonians is the canonical use case for
quantum simulation. The protocol decomposes `exp(-iHt)` into a
product of operator exponentials via Trotter-Suzuki splitting:

```
exp(-iHt) ≈ [exp(-iH_1 t/n) exp(-iH_2 t/n) ... exp(-iH_k t/n)]^n
```

The protocol envelope carries the splitting order (1st, 2nd, 4th,
6th), the time step `t/n`, and the operator decomposition. Higher
Trotter orders reduce error at the cost of deeper circuits;
conformant hosts publish the error bound for each declared order.

## A.7 Imaginary-time evolution

Imaginary-time evolution `exp(-Hτ)|ψ⟩` projects toward the ground
state. The protocol envelope carries the imaginary-time step and the
expected ground-state-energy convergence trajectory. Variational
imaginary-time evolution is the variational analogue used on
NISQ-era hardware.

## A.8 Audit and reproducibility

Every protocol envelope is signed by the originating identity and
appended to the audit log. Reproducibility envelopes carry the random
seed, the noise model, and a hash of the input Hamiltonian so a
verifier can re-run the simulation byte-identically given the same
envelope inputs and the same backend.



## A.9 Density-matrix and noise-channel composition

Density-matrix simulation studies open-quantum systems where
decoherence cannot be ignored. The protocol composes unitary
evolution with documented noise channels:

- Depolarising channel: random Pauli error with documented probability
- Amplitude-damping channel: models energy loss to environment
- Dephasing channel: phase coherence loss without energy exchange
- Thermal-relaxation channel: composite parametrised by T1, T2 times

Each channel application is a signed protocol envelope so the full
simulation history is auditable.

## A.10 Many-body localisation studies

The many-body sampling endpoint supports MBL studies by sampling
from disorder-averaged Hamiltonians. The protocol envelope carries
the disorder realisation, the sampling temperature, and the sampled
observables. MBL studies typically run thousands of disorder
realisations; the bulk-export endpoint streams the entire sample
set with a Merkle root commitment.


## A.11 Glossary expansion for protocol terms

Replay defence: 96-bit nonce + 300-second skew window + 600-second
seen-nonce cache enforced uniformly across the WIA family.
Federation handshake: signed exchange between two hosts
establishing a peer relationship; reuses WIA-SOCIAL Phase 3 §5
receipt shape. Audit log: append-only replicated log of every
signed protocol envelope with retention sized to regulatory
requirements. Trust list: signed list of permitted peers
republished on a documented cadence (≤ 30 days for non-quantum
standards, faster for quantum-network and quantum-key-distribution).

## A.12 Cross-standard composition for the protocol layer

The protocol layer of this Phase shares its replay-defence cache,
trust-list maintenance, and audit-log replication with adjacent
WIA family standards running on the same host. A host running
WIA Quantum Simulation alongside WIA Quantum Algorithm and WIA
Quantum Network reuses one cache, one trust list, and one audit
log across all three standards rather than maintaining three
parallel implementations.

## A.13 Implementation tip — cross-host federation

When optimisation runs on host A and quantum simulation runs on
host B, the WIA-SOCIAL Phase 3 §5 federation handshake establishes
the peer relationship before any optimisation envelope flows.
Without the handshake, host B refuses optimisation envelopes from
host A; with the handshake, the audit log on both sides records
the relationship establishment so a third party can verify the
chain after the fact.


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
