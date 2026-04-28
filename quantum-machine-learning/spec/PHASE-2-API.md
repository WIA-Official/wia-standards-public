# WIA-QUA-006 — Phase 2: API

> QML API surface: variational quantum algorithms, quantum neural network endpoints, quantum kernel methods, and quantum generative models — each presented as an endpoint specification integrators can target directly.

## 3. Quantum Neural Networks

### 3.1 QNN Architecture

A QNN consists of:

1. **Input Encoding Layer**: U_in(x)
2. **Parameterized Layers**: U(θ) = ∏ₗ Uₗ(θₗ)
3. **Measurement Layer**: M

Mathematical form:
```
f(x,θ) = ⟨0|U†(θ)U†_in(x) M U_in(x)U(θ)|0⟩
```

### 3.2 Single Layer Structure

```
Uₗ(θₗ) = Entanglement × Rotation(θₗ)
```

**Rotation Gates**:
```
R_X(θ) = exp(-iθX/2)
R_Y(θ) = exp(-iθY/2)
R_Z(θ) = exp(-iθZ/2)
```

**Entanglement Patterns**:
- Linear: CNOT gates between adjacent qubits
- Full: CNOT gates between all qubit pairs
- Circular: Ring connectivity

### 3.3 Universal Quantum Approximation

A QNN can approximate any quantum operation with sufficient depth:

```
U_target ≈ ∏ₗ₌₁ᴸ U_layer(θₗ)
```

For L → ∞, the approximation becomes exact.

### 3.4 QNN Forward Pass

```python
def qnn_forward(x, theta, num_qubits):
    # Initialize quantum circuit
    qc = QuantumCircuit(num_qubits)

    # Encode input data
    qc.compose(encode_data(x))

    # Apply parameterized layers
    for layer in range(num_layers):
        qc.compose(variational_layer(theta[layer]))

    # Measure
    qc.measure_all()

    return execute(qc).result()
```

### 3.5 Parameter Count

For a QNN with:
- n qubits
- L layers
- 3 rotation gates per qubit per layer

Total parameters: `P = 3 × n × L`

---


## 4. Variational Quantum Algorithms

### 4.1 Variational Quantum Classifier (VQC)

The VQC optimizes parameters to minimize classification loss:

```
θ* = argmin_θ L(θ)
```

Where:
```
L(θ) = (1/N) Σᵢ loss(f(xᵢ,θ), yᵢ)
```

### 4.2 VQC Components

#### 4.2.1 Feature Map
```
ϕ(x): ℝⁿ → ℋ (Hilbert space)
```

Example: Angle encoding feature map
```
U_ϕ(x) = ⊗ⁱ R_Y(xᵢ)
```

#### 4.2.2 Variational Ansatz

Hardware-efficient ansatz:
```
U(θ) = [R_Z(θ) ⊗ R_Y(θ) ⊗ R_Z(θ)] × CNOT_pattern
```

#### 4.2.3 Measurement

Expectation value of observable M:
```
⟨M⟩_θ,x = ⟨ψ(x,θ)|M|ψ(x,θ)⟩
```

### 4.3 Variational Quantum Eigensolver (VQE)

VQE finds ground state energy:

```
E_ground ≈ min_θ ⟨ψ(θ)|H|ψ(θ)⟩
```

Where H is the Hamiltonian.

### 4.4 Quantum Approximate Optimization Algorithm (QAOA)

QAOA for combinatorial optimization:

```
|ψ(β,γ)⟩ = ∏ₚ₌₁ᴾ U_B(βₚ)U_C(γₚ)|+⟩⊗ⁿ
```

Where:
- `U_C(γ)` = exp(-iγC) (problem Hamiltonian)
- `U_B(β)` = exp(-iβB) (mixer Hamiltonian)

---


## 5. Quantum Kernel Methods

### 5.1 Quantum Kernel Definition

The quantum kernel measures similarity in feature space:

```
K(x,x') = |⟨ϕ(x)|ϕ(x')⟩|²
```

Where `ϕ(x) = U_ϕ(x)|0⟩` is the quantum feature map.

### 5.2 Kernel Circuit

```
K(x,x') = |⟨0|U†_ϕ(x')U_ϕ(x)|0⟩|²
```

Implemented as:
1. Prepare |0⟩
2. Apply U_ϕ(x)
3. Apply U†_ϕ(x')
4. Measure in computational basis
5. Probability of |0⟩ = kernel value

### 5.3 Quantum Support Vector Machine (QSVM)

Classification function:
```
f(x) = sign(Σᵢ αᵢyᵢK(xᵢ,x) + b)
```

Where:
- `αᵢ` = Lagrange multipliers (from classical SVM training)
- `yᵢ` = Training labels
- `K(xᵢ,x)` = Quantum kernel
- `b` = Bias term

### 5.4 Quantum Kernel Estimation

Estimating kernel from measurements:

```
K̂(x,x') = (# times measured |0⟩) / (total shots)
```

Variance of estimate:
```
Var[K̂] = K(1-K) / N_shots
```

### 5.5 Quantum Kernel Advantage

Quantum kernels can be hard to compute classically when:
1. Feature map uses deep quantum circuits
2. Entanglement creates complex correlations
3. No efficient classical simulation exists

---


## 6. Quantum Generative Models

### 6.1 Quantum Generative Adversarial Networks (QGANs)

QGAN components:
1. **Quantum Generator**: G(z,θ_G) produces |ψ(z,θ_G)⟩
2. **Classical/Quantum Discriminator**: D(x,θ_D) ∈ [0,1]

Objective:
```
min_G max_D 𝔼_real[log D(x)] + 𝔼_z[log(1 - D(G(z)))]
```

### 6.2 Quantum Generator

```
G(z,θ): z → |ψ(z,θ)⟩
```

Sampling: Measure |ψ(z,θ)⟩ to get synthetic data

### 6.3 Quantum Boltzmann Machine (QBM)

Energy function:
```
E(v,h) = -Σᵢⱼ Wᵢⱼvᵢhⱼ - Σᵢ bᵢvᵢ - Σⱼ cⱼhⱼ
```

Quantum state:
```
|ψ⟩ = (1/Z) Σᵥ,ₕ exp(-E(v,h))|v,h⟩
```

Where:
- v = Visible units
- h = Hidden units
- W = Weights
- b, c = Biases

### 6.4 Quantum Circuit Born Machine (QCBM)

Probability distribution:
```
p(x|θ) = |⟨x|ψ(θ)⟩|²
```

Where `|ψ(θ)⟩ = U(θ)|0⟩`

Training objective (KL divergence):
```
D_KL(p_data||p_θ) = Σₓ p_data(x) log(p_data(x)/p_θ(x))
```

### 6.5 Quantum Autoencoder

Encoding:
```
|ψ_in⟩ → U_encode → Trace out ancillas → ρ_latent
```

Decoding:
```
ρ_latent → U_decode → |ψ_out⟩
```

Loss:
```
L = 1 - |⟨ψ_in|ψ_out⟩|²
```

---



## A.1 Endpoint reference

```http
POST /qml/v1/qnn/train       # quantum neural network training
POST /qml/v1/qnn/predict     # inference
POST /qml/v1/vqa/optimize    # variational quantum algorithm
POST /qml/v1/kernel/compute  # quantum kernel matrix computation
POST /qml/v1/generative/sample  # quantum generative model sampling
```

Every endpoint accepts a `target_backend` parameter and returns either a result envelope or a problem document.

## A.2 Quantum neural network endpoint

QNN training accepts an architecture descriptor (parameterised quantum circuit), a dataset reference, and an optimiser specification. The endpoint returns a model attestation envelope with the trained parameters and the training-loss history.

## A.3 Quantum kernel methods

Quantum kernel methods compute a kernel matrix `K[i][j] = |⟨φ(x_i)|φ(x_j)⟩|²` over a classical dataset. The kernel matrix is the input to a downstream classical SVM or other kernel-based learner. The endpoint returns the matrix as a base64-encoded array with a hash commitment.

## A.4 Quantum generative models

Quantum generative models (QCBM, QGAN) sample from a learned distribution. The endpoint accepts a sample count and returns the samples plus a Wasserstein distance estimate to the training distribution as a quality witness.


## A.5 Endpoint detail — VQA optimisation

```http
POST /qml/v1/vqa/optimize
{
  "ansatz_kind": "hardware-efficient",
  "ansatz_depth": 4,
  "parameter_initialisation": "warm_start",
  "warm_start_run_id": "run_01HX..." | null,
  "objective": { "kind": "ground_state_energy", "hamiltonian": {...} },
  "convergence_threshold": 1e-6,
  "max_iterations": 200,
  "target_backend": "local-noisy"
}
```

Returns a model attestation envelope with the optimised parameters,
the convergence trajectory, and a calibrated estimate of the
out-of-sample performance.

## A.6 Webhook subscriptions

QML training is long-running; the host emits webhook callbacks on
checkpoint events (every 50 iterations) and on completion. The
callback payload is the full signed envelope with the
`X-WIA-Signature` HMAC header for verification.

## A.7 Bulk export

For audit and reproducibility, every host exposes a bulk export
endpoint returning a signed manifest plus per-envelope content as
NDJSON. The manifest carries a Merkle root over the included
envelopes for completeness verification.


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
