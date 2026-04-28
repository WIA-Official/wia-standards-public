# WIA-QUA-002 — Phase 2: API

> Algorithmic API surface: Shors factoring, Grover search, and
> the variational quantum eigensolver (VQE) as worked APIs that
> integrators code against. Each algorithm is presented as a
> circuit blueprint plus the classical control surface required
> to drive it on todays NISQ devices.

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


## A.1 Algorithm-API conventions

Every API surface in this Phase is a wrapper around a circuit
blueprint plus the classical control surface needed to drive it.
The wrapper is HTTP-based with TLS 1.3 (IETF RFC 8446) only and
follows the discovery convention at
`/.well-known/wia-quantum-algorithm`.

Endpoints accept a `target_backend` parameter pointing at one of:

- `local-statevector` — exact simulation on the host
- `local-noisy` — noisy simulation with declared noise model
- `cloud-quantum` — passthrough to IBM Quantum, AWS Braket,
  Azure Quantum, or Google Quantum AI based on host configuration

The host translates the circuit into the target backend's native
form and returns a measurement-record envelope (Phase 1 §A.4)
on the wire.

## A.2 Shors factoring API — operational notes

The Shors factoring API at `/qua/v1/shors/factor` accepts a
classical integer `N` and returns either a factorisation or a
problem document explaining why the algorithm did not converge.
Convergence depends on the circuit depth available on the
target backend: NISQ-era backends rarely factor numbers above
21 reliably; the API documents the maximum N for each backend
in its discovery document.

```http
POST /qua/v1/shors/factor
{
  "N": 21,
  "shots": 8192,
  "target_backend": "local-noisy"
}
```

## A.3 Grover search API — operational notes

The Grover search API at `/qua/v1/grover/search` accepts an oracle
specification and the search-space size and returns the index
with the highest measured probability. The API documents the
optimal number of Grover iterations for each search-space size
so callers can avoid the over-rotation pitfall typical of naive
Grover implementations.

## A.4 VQE API — convergence and warm-start

The VQE API at `/qua/v1/vqe/optimize` runs the variational loop
on the target backend until either the convergence threshold is
met or the iteration budget is exhausted. Warm-start from a
prior VQE run is supported via `warm_start_run_id` so practitioners
can resume optimisation across sessions.

## A.5 Conformance test coverage

The black-box conformance suite at
`https://github.com/WIA-Official/wia-qua-conformance` exercises
each Phase 2 API path with mock backends. Conformant hosts MUST
pass: discovery round-trip, all three algorithm endpoints,
problem-detail emission for malformed inputs, and rate-limit
header presence on every response.

弘益人間 — Benefit All Humanity.

## A.6 Worked Shors invocation trace

```http
POST /qua/v1/shors/factor
Content-Type: application/json
Authorization: WIA-Sig keyid="...",signature="..."

{
  "N": 21,
  "shots": 8192,
  "target_backend": "local-noisy",
  "noise_model": {
    "kind": "depolarising",
    "single_qubit_p": 0.001,
    "two_qubit_p": 0.01
  },
  "compile_strategy": "default",
  "client_request_id": "req_01HX..."
}

→ 200 OK
{
  "wia_qua_version": "1.0.0",
  "type": "shors_result",
  "run_id": "run_01HX...",
  "input_N": 21,
  "factors": [3, 7],
  "iteration_count": 4,
  "circuit_depth": 248,
  "shots_consumed": 8192,
  "elapsed_seconds": 12.4,
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

## A.7 Worked Grover invocation trace

```http
POST /qua/v1/grover/search
{
  "oracle": {
    "kind": "boolean_circuit",
    "qasm_3_0": "..."
  },
  "n_qubits": 4,
  "marked_count_estimate": 1,
  "shots": 4096,
  "target_backend": "local-statevector"
}

→ 200 OK
{
  "wia_qua_version": "1.0.0",
  "type": "grover_result",
  "run_id": "run_01HX...",
  "iterations_applied": 3,
  "best_index": 11,
  "best_index_probability": 0.97,
  "all_index_distribution": [...],
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

## A.8 Worked VQE invocation trace

```http
POST /qua/v1/vqe/optimize
{
  "hamiltonian": {
    "kind": "pauli_sum",
    "terms": [
      { "coefficient": 0.5, "pauli_string": "ZZII" },
      { "coefficient": 0.5, "pauli_string": "IZZI" },
      { "coefficient": 0.5, "pauli_string": "IIZZ" }
    ]
  },
  "ansatz": "uccsd",
  "max_iterations": 200,
  "convergence_threshold": 1.0e-6,
  "target_backend": "local-noisy"
}

→ 200 OK
{
  "wia_qua_version": "1.0.0",
  "type": "vqe_result",
  "run_id": "run_01HX...",
  "ground_state_energy_estimate": -1.85,
  "convergence_reached": true,
  "iterations_used": 87,
  "best_parameter_vector": [...],
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

## A.9 Rate limits and backpressure

Public read endpoints return RFC 6585 §4 (`429 Too Many Requests`)
with `Retry-After` when rate limits are exceeded. Authenticated
write endpoints additionally publish RFC 9180 (`RateLimit`,
`RateLimit-Limit`, `RateLimit-Remaining`, `RateLimit-Reset`)
on every response; the values are advisory and clients SHOULD
track them to avoid backpressure cascades.

## A.10 Bulk export endpoint

For audit and reproducibility, every host exposes a bulk
export of past runs:

```http
POST /qua/v1/exports
{
  "from": "2026-04-01T00:00:00Z",
  "to":   "2026-04-30T23:59:59Z",
  "filter": { "algorithm": "shors_factor" }
}

→ 202 Accepted
   Location: /qua/v1/exports/{export_id}

GET /qua/v1/exports/{export_id}/manifest
GET /qua/v1/exports/{export_id}/content   (NDJSON stream of envelopes)
```

The manifest carries a Merkle root over the included
envelopes so a downstream auditor can verify completeness
without re-signing every envelope individually. Bulk
exports are subject to data-residency rules of the host
operators jurisdiction.

## A.11 Webhook subscriptions

Long-running algorithms (especially VQE with hours-long
convergence) emit webhook callbacks on completion:

```http
POST /qua/v1/webhooks
{
  "url": "https://...",
  "events": ["shors_result", "grover_result", "vqe_result"],
  "shared_secret_hint": "k_2026q2",
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

Callback delivery uses exponential backoff (1, 2, 4, 8, 16,
32, 64 seconds, then dead-letter at the 8th attempt). The
HMAC of the signed envelope appears in the
`X-WIA-Signature` header so the receiver can verify before
acting on the content.

## Closing implementer note for PHASE-2-API

The Phase 2 API surface deliberately keeps a small endpoint count so integrators can implement the bridge in a long afternoon rather than a multi-week project; the conformance suite is the authoritative reference for what each endpoint must accept and return.

A reference implementation of every Phase mentioned in this
specification ships in the conformance container at
`wia/quantum-algorithm-host:1.0.0` and is the recommended
starting point for new integrators. Customising the container
to a host operators specific cloud-quantum provider is
typically a one-week project for an experienced quantum-software
engineer.
