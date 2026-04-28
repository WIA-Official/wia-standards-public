# WIA-QUA-006 — Phase 3: Protocol

> Training-loop and optimization protocol layer. The barren-plateau problem dominates this phase because the protocol's wire-level exchange must surface plateau detection signals to the classical optimiser without silent failure.

## 8. Training and Optimization

### 8.1 Parameter Shift Rule

For gradient computation:

```
∂/∂θ ⟨ψ(θ)|M|ψ(θ)⟩ = r[⟨ψ(θ+s)|M|ψ(θ+s)⟩ - ⟨ψ(θ-s)|M|ψ(θ-s)⟩]
```

Where:
- r = scaling factor (typically 1/2)
- s = shift value (typically π/2)

### 8.2 Quantum Natural Gradient

Update rule:
```
θₜ₊₁ = θₜ - ηF⁻¹∇C(θₜ)
```

Where F is the Fubini-Study metric tensor:
```
F_ij = Re[⟨∂ᵢψ|∂ⱼψ⟩ - ⟨∂ᵢψ|ψ⟩⟨ψ|∂ⱼψ⟩]
```

### 8.3 Classical Optimizers for QML

Recommended optimizers:
1. **COBYLA**: Constrained optimization, derivative-free
2. **SPSA**: Simultaneous perturbation stochastic approximation
3. **Adam**: Adaptive moment estimation
4. **L-BFGS-B**: Limited-memory BFGS

### 8.4 Learning Rate Scheduling

```
η(t) = η₀ / (1 + decay × t)
```

Or cosine annealing:
```
η(t) = η_min + (η_max - η_min) × (1 + cos(πt/T)) / 2
```

### 8.5 Batch Training

Mini-batch gradient:
```
∇C(θ) ≈ (1/B) Σᵢ₌₁ᴮ ∇C_i(θ)
```

Where B = batch size

---


## 9. Barren Plateau Problem

### 9.1 Definition

A barren plateau occurs when gradients vanish exponentially with system size:

```
Var[∂C/∂θ] ∈ O(1/2ⁿ)
```

Where n = number of qubits.

### 9.2 Causes

1. **Random Circuits**: Deep random unitaries
2. **Global Cost Functions**: Measuring all qubits
3. **Hardware Noise**: Decoherence amplifies the problem

### 9.3 Detection

Check gradient variance:
```
σ² = 𝔼[(∂C/∂θ - 𝔼[∂C/∂θ])²]
```

If σ² ∝ exp(-cn), barren plateau detected.

### 9.4 Mitigation Strategies

#### 9.4.1 Layer-wise Training
Train shallow circuits first, gradually add layers:
```
θ₁ → optimize → θ₁,θ₂ → optimize → θ₁,θ₂,θ₃ → ...
```

#### 9.4.2 Local Cost Functions
Use local observables:
```
C = Σᵢ ⟨Mᵢ⟩  (i indexes local regions)
```

#### 9.4.3 Correlated Initialization
Initialize parameters with problem structure:
```
θ₀ ∼ problem-specific distribution
```

#### 9.4.4 Identity Block Initialization
Start with identity-like circuits:
```
U(θ=0) ≈ I
```

#### 9.4.5 Parameter Sharing
Reduce parameter space:
```
θᵢ = θⱼ for certain i,j
```

### 9.5 Avoiding Barren Plateaus

Design principles:
1. Use shallow circuits when possible (depth < 10)
2. Employ hardware-efficient ansätze
3. Use problem-inspired ansätze
4. Implement local cost functions
5. Careful parameter initialization

---



## A.1 Training-loop protocol

The QML training loop is a two-protocol layer cake: an outer classical optimiser that updates parameters, and an inner quantum sampler that returns expectation values to feed the optimiser. The protocol exchanges parameter updates as signed envelopes so a third party can audit the training trajectory.

## A.2 Barren plateau detection

Barren plateaus (regions of vanishing gradient in parameter space) are the dominant failure mode for QML training. The protocol requires every training loop to publish a `gradient_variance` telemetry stream; consumers detecting plateau onset can switch to a layerwise-trainable alternative ansatz mid-training.

## A.3 Mitigation strategies

The standard recognises four plateau-mitigation strategies: parameter initialisation from a classical relaxation (warm start), layerwise pretraining, identity-block ansatze, and adaptive operator pools. The chosen strategy appears in the model attestation envelope.

## A.4 Replay defence and audit

Training-loop envelopes carry the standard 96-bit nonce + 300-second skew window + 600-second seen-nonce cache.


## A.5 Quantum-classical training-loop trace

```
Iteration N:
  classical_optimiser → quantum_sampler:
    parameter_vector θ_N (signed by classical_optimiser identity)
  quantum_sampler → classical_optimiser:
    expectation ⟨H⟩_θ_N (signed by quantum_sampler identity)
    gradient_estimate ∇⟨H⟩_θ_N (if requested)
    gradient_variance Var[∇⟨H⟩_θ_N] (always)
  classical_optimiser:
    update θ_{N+1} = θ_N - η ∇⟨H⟩_θ_N
    if Var[∇⟨H⟩_θ_N] < plateau_threshold:
      emit plateau_warning envelope; switch to layerwise training.
Repeat until convergence or iteration budget exhausted.
```

## A.6 Plateau-detection thresholds

The standard recommends `plateau_threshold = 1e-4 / parameter_count`
as a rule of thumb; production hosts tune this per-ansatz based on
empirical observation. The chosen threshold appears in the model
attestation envelope so consumers can refuse models trained without
plateau detection enabled.

## A.7 Federated training considerations

Federated QML (multiple parties holding private datasets contributing
to a single trained model) adds privacy constraints. The standard's
federated extension (Phase 4) describes the secure aggregation
envelope for parameter updates so individual party data is not
revealed to the aggregator.

## A.8 Audit log discipline

Training-loop envelopes are written to an append-only audit log
replicated across at least two storage backends. The log MUST be
inspectable by the model owner without granting the auditor access
to the trained parameters; a parameter-redacted view is part of the
conformance suite.


## B.1 Conformance test suite

A black-box conformance test suite is published at
`https://github.com/WIA-Official/wia-quantum-machine-learning-conformance` and walks
through every public endpoint plus the cross-Phase integration
scenarios. Hosts publishing `bridge_profile=Full` SHOULD additionally
pass the suite's bridge-extension tests.

The suite checks: discovery document round-trip, every Phase 2 endpoint
with mock data, problem-detail emission for malformed inputs, rate-limit
header presence and exhaustion behaviour, replay-defence bounds (300-second
skew, 600-second nonce cache), and audit-log envelope shape.

## B.2 Reference container

The `wia/quantum-machine-learning-host:1.0.0` container image implements every Phase 2
endpoint with mock data; integrators exercise their bridge against it
before going to production. The container ships with a small library of
mock scenarios so the conformance suite has fixtures to run against.

## B.3 Companion CLI

The `cli/quantum-machine-learning.sh` script ships sample envelope generators (validate,
info, plus phase-specific subcommands) so an implementer can produce
conformant payloads without hand-rolling JSON. The CLI has no dependency
beyond `jq` and POSIX shell, so it runs in any CI environment without
additional tooling installation.

## B.4 Operational considerations

Quantum machine-learning infrastructure has three operational considerations
that integrators consistently underestimate. First, the wire-format
discipline: every envelope is signed and verified at the boundary;
unsigned envelopes are refused at conformance, full stop. Second, the
trust-list refresh cadence: stale trust anchors are the single largest
source of avoidable production incidents in this standard family. Third,
the audit-log replication discipline: audit logs replicated across only
one storage backend cannot survive a site failure, and a site failure
that takes the audit log with it leaves the operator unable to
reconstruct what happened during the failure window.

## B.5 Backwards-compatibility promise

Within the 1.x line every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable
and MUST continue to honour the documented status codes and content
shapes. Hosts MAY add optional fields, optional query parameters,
new envelope types, new endpoints, or new protocol exchanges; hosts
MUST NOT remove or repurpose existing ones. Breaking changes ride a
major version bump and MUST be preceded by a 12-month deprecation
window per IETF RFC 8594 and 9745.

## B.6 Governance

The standard is maintained by the WIA Standards Committee. Change
proposals follow the WIA RFC process: anyone may submit a proposal;
the Committee reviews quarterly; accepted proposals enter an open
comment period before merging into a minor-version release. Breaking
changes require a two-thirds Committee vote.

弘益人間 — Benefit All Humanity.

## C.1 Glossary

The following terms appear repeatedly throughout this Phase and
the wider quantum-machine-learning standard: VQA (Variational Quantum Algorithm); QNN (Quantum Neural Network); QCBM (Quantum Circuit Born Machine); QGAN (Quantum Generative Adversarial Network); ansatz (parameterised circuit family); barren plateau; expressibility; trainability; warm-start initialisation; layerwise training; ADAPT-QAOA.

Implementers unfamiliar with the domain should treat these terms as
load-bearing — every endpoint, every protocol exchange, and every
integration document below assumes the reader understands what each
term means in context. Expanded definitions appear in the standard's
companion glossary at `https://wiastandards.com/quantum-machine-learning/glossary/`.

## C.2 Cross-standard composition

This Phase composes with adjacent WIA family standards as follows:

- **WIA-OMNI-API** owns credential storage and identity for every
  signed envelope in this Phase.
- **WIA-AIR-SHIELD** owns runtime trust list maintenance and key
  rotation; this Phase consumes WIA-AIR-SHIELD events.
- **WIA-SOCIAL Phase 3 §5** receipt shape is reused verbatim for
  every cross-host federation handshake referenced in this Phase.
- **WIA-INTENT** owns the outermost-layer declaration of workload
  intent; consumers parse the intent envelope before drilling into
  this Phase's specifics.

The composition is intentional: a single host running multiple WIA
family standards reuses the same identity, signature, and federation
machinery across all of them rather than maintaining N parallel
implementations. The conformance suite verifies the composition by
running a multi-standard scenario where this Phase's envelopes flow
through the adjacent standards' machinery and back.

## C.3 Implementation runbook

A first implementation of this Phase typically follows the runbook:

1. Stand up the reference container ('wia/quantum-machine-learning-host:1.0.0') in a
   development environment.
2. Run the conformance suite against the container to verify all
   tests pass on the reference implementation.
3. Replace the mock backend with the host's real backend
   one endpoint at a time; re-run conformance after each
   replacement.
4. Wire up the audit log replication; verify a full session round
   trip lands in both replicas.
5. Onboard a single trusted peer for federation; exercise the
   handshake and audit envelope flow.
6. Expand to multiple peers; rotate trust anchors per the
   30-day cadence.
7. Promote to production; subscribe operations to the warning
   envelope cadence (collateral expiry, drift detection,
   barren-plateau onset, etc., as relevant per phase).

The full runbook is roughly two engineer-weeks of work for a team
already familiar with the underlying domain (QKD, QML, quantum-
network, or quantum-sensor as applicable).
