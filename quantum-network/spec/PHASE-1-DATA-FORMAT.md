# WIA-QUA-003 — Phase 1: Data Format

> Quantum-network canonical envelopes: link descriptor, entanglement record, quantum-memory state, and the runtime conventions that fix the wire format of every protocol below.

## 1. Introduction

### 1.1 Purpose

This specification defines the framework for quantum network infrastructure, enabling secure quantum communication, entanglement distribution, and the foundation for a global quantum internet.

### 1.2 Scope

The standard covers:
- Quantum key distribution protocols (BB84, E91, B92)
- Entanglement distribution mechanisms
- Quantum repeater architecture
- Quantum memory requirements
- Network topology and routing
- Security and authentication protocols

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to establish quantum communication networks that provide unconditional security and enable revolutionary applications in computing, sensing, and communication for the benefit of all humanity.

### 1.4 Terminology

- **Qubit**: Quantum bit, the basic unit of quantum information
- **Entanglement**: Quantum correlation between particles
- **QKD**: Quantum Key Distribution
- **QBER**: Quantum Bit Error Rate
- **Fidelity**: Measure of quantum state quality (0-1)
- **Bell State**: Maximally entangled two-qubit state
- **BSM**: Bell State Measurement

---


## 2. Quantum Key Distribution (QKD)

### 2.1 BB84 Protocol

The BB84 protocol, developed by Bennett and Brassard in 1984, uses four quantum states for secure key distribution.

#### 2.1.1 Quantum States

**Rectilinear Basis (+ basis):**
```
|0⟩ = |→⟩  (horizontal polarization, bit 0)
|1⟩ = |↑⟩  (vertical polarization, bit 1)
```

**Diagonal Basis (× basis):**
```
|+⟩ = (|→⟩ + |↑⟩)/√2  (45° polarization, bit 0)
|−⟩ = (|→⟩ - |↑⟩)/√2  (135° polarization, bit 1)
```

#### 2.1.2 Protocol Steps

1. **Preparation Phase**:
   - Alice generates random bit string: `b = [0,1,0,1,1,0,...]`
   - Alice generates random basis string: `a = [+,×,+,+,×,...]`
   - Alice encodes each bit in chosen basis and sends to Bob

2. **Measurement Phase**:
   - Bob generates random basis string: `b' = [+,+,×,+,×,...]`
   - Bob measures each qubit in chosen basis

3. **Sifting Phase**:
   - Alice and Bob publicly compare bases (not results)
   - Keep only bits where bases matched
   - Discard approximately 50% of bits

4. **Error Estimation**:
   - Sample subset of sifted key to estimate QBER
   - QBER = (errors / total sampled bits)
   - If QBER > 11%, abort (possible eavesdropper)

5. **Error Correction**:
   - Apply classical error correction (e.g., Cascade, LDPC)
   - Reconcile remaining differences in keys

6. **Privacy Amplification**:
   - Apply universal hash function
   - Reduce key length to remove eavesdropper's information
   - Final key length: `r = n(1 - h(Q))` where Q is QBER

#### 2.1.3 Security Analysis

**Mutual Information**:
```
I(Alice:Eve) ≤ h(Q)
I(Alice:Bob) = 1 - h(Q)
```

Where `h(x) = -x log₂(x) - (1-x) log₂(1-x)` is the binary entropy function.

**Secret Key Rate**:
```
r = q[1 - 2h(Q)]
```

Where:
- `q` = sifting efficiency (≈ 0.5 for BB84)
- `Q` = QBER
- `h(Q)` = binary entropy

**Security Threshold**: QBER must be < 11% for security against individual attacks, < 20% for collective attacks with privacy amplification.

### 2.2 E91 Protocol

The E91 protocol uses entangled photon pairs and Bell inequality tests.

#### 2.2.1 Entangled States

Uses Bell state |Φ⁺⟩:
```
|Φ⁺⟩ = (|00⟩ + |11⟩)/√2
```

#### 2.2.2 Protocol Steps

1. **Entanglement Distribution**:
   - Source creates EPR pairs in |Φ⁺⟩ state
   - One photon sent to Alice, one to Bob

2. **Measurement**:
   - Alice randomly chooses from 3 bases: `{0°, 45°, 90°}`
   - Bob randomly chooses from 3 bases: `{45°, 90°, 135°}`
   - Both measure received photons

3. **Classical Communication**:
   - Alice and Bob announce measurement bases
   - Keep results where bases enable key generation
   - Use other results for Bell inequality test

4. **Bell Test**:
   - Calculate CHSH inequality parameter S
   - For quantum correlations: S = 2√2 ≈ 2.828
   - For local hidden variables: S ≤ 2
   - If S > 2, verify quantum correlations (security)

5. **Key Extraction**:
   - Use correlated measurement results as key bits
   - Apply error correction and privacy amplification

#### 2.2.3 CHSH Inequality

```
S = |E(a₁,b₁) + E(a₁,b₂) + E(a₂,b₁) - E(a₂,b₂)|
```

Where `E(aᵢ,bⱼ)` is the correlation between Alice's measurement in basis aᵢ and Bob's measurement in basis bⱼ.

**Quantum Prediction**: S = 2√2 ≈ 2.828
**Classical Limit**: S ≤ 2

### 2.3 B92 Protocol

Simplified two-state protocol using non-orthogonal states.

#### 2.3.1 States

Alice uses two non-orthogonal states:
```
|0⟩ = |→⟩  (horizontal, bit 0)
|1⟩ = |↗⟩  (45°, bit 1)
```

Inner product: `⟨0|1⟩ = 1/√2` (non-orthogonal)

#### 2.3.2 Protocol

1. Alice randomly sends |0⟩ or |1⟩
2. Bob randomly measures in basis {|→⟩, |↑⟩} or {|↗⟩, |↖⟩}
3. Bob announces which measurements gave results
4. Alice and Bob keep only conclusive results
5. Error correction and privacy amplification

**Efficiency**: Lower than BB84 (~25% vs 50%) but simpler implementation.

---


## 5. Quantum Memory

### 5.1 Requirements

**Storage Time**:
- Metropolitan networks: > 1 ms
- Long-distance: > 1 second
- Network switches: > 100 μs

**Efficiency**:
- Write efficiency: > 50%
- Read efficiency: > 50%
- Total efficiency: > 25%

**Fidelity**:
- Stored state fidelity: > 0.99
- Retrieved state fidelity: > 0.95

**Multimode Capacity**:
- Temporal modes: > 100
- Spatial modes: > 10
- Total capacity: > 1000 qubits

### 5.2 Implementations

#### 5.2.1 Atomic Frequency Combs (AFC)

**Principle**: Use rare-earth ion ensemble with periodic absorption spectrum.

**Storage Mechanism**:
- Photon absorbed by ensemble
- Periodic structure creates revivals
- Controlled retrieval via echo

**Parameters**:
- Material: Pr³⁺:Y₂SiO₅, Nd³⁺:Y₂SiO₅
- Temperature: < 4 K (cryogenic)
- Storage time: > 1 second
- Efficiency: 30-50%

#### 5.2.2 Electromagnetically Induced Transparency (EIT)

**Principle**: Control medium transparency with coupling laser.

**Storage Mechanism**:
- Signal photon enters atomic medium
- Coupling laser creates transparency window
- Turn off coupling → photon stored as atomic excitation
- Turn on coupling → retrieve photon

**Parameters**:
- Medium: Warm atomic vapor or cold atoms
- Temperature: 50-100°C (warm) or μK (cold)
- Storage time: 10-100 ms
- Efficiency: 50-80%

---



## A.1 Canonical envelope conventions

Every Phase 1 quantum-network envelope follows the WIA family baseline: UTF-8 JSON, RFC 8785 canonical form, Ed25519 signatures, ULID identifiers. Network addresses use a registered URN scheme `urn:wia:qnet:<node-id>`.

## A.2 Link descriptor envelope

```json
{
  "wia_qnet_version": "1.0.0",
  "type": "link_descriptor",
  "link_id": "link_01HX...",
  "endpoints": ["urn:wia:qnet:node-A", "urn:wia:qnet:node-B"],
  "fidelity_baseline": 0.96,
  "entanglement_rate_hz": 1200,
  "decoherence_time_us": 250,
  "supported_protocols": ["BB84", "entanglement-distribution", "teleportation"]
}
```

## A.3 Entanglement record envelope

Each entangled pair generated across the network is recorded with a unique identifier, the participating nodes, and a fidelity estimate so consumers can choose pairs above a quality threshold for downstream protocols.

## A.4 Quantum memory state

Quantum memory holds entangled qubits between generation and use. The state envelope carries the memory identity, the qubits stored, the storage timestamp, and the decoherence projection so consumers can decide whether to use a qubit before its fidelity drops below threshold.


## A.5 Network address scheme

URN scheme `urn:wia:qnet:<node-id>` where `<node-id>` is a stable
ULID assigned at node enrolment. Address resolution uses DNS with
`_wia-qnet._tcp.<domain>` SRV records pointing at the network
controller; the controller returns the URN-to-network-address map
on demand.

## A.6 Fidelity-quality envelope

```json
{
  "type": "fidelity_estimate",
  "link_id": "link_01HX...",
  "estimated_fidelity": 0.94,
  "estimation_method": "process_tomography" | "randomized_benchmarking" | "direct_fidelity",
  "estimation_shots": 10000,
  "confidence_interval_95": [0.92, 0.96],
  "estimated_at": "RFC 3339"
}
```

Fidelity estimates carry the estimation method and confidence
interval so consumers can refuse links whose estimates are below
their tolerance.

## A.7 Quantum memory state

Quantum memory holds entangled qubits. The state envelope carries
the memory identity, the qubits stored, the storage timestamp, the
projected fidelity at expiry, and the storage technology
(atomic ensemble, NV centre, optical cavity).


## B.1 Conformance test suite

A black-box conformance test suite is published at
`https://github.com/WIA-Official/wia-quantum-network-conformance` and walks
through every public endpoint plus the cross-Phase integration
scenarios. Hosts publishing `bridge_profile=Full` SHOULD additionally
pass the suite's bridge-extension tests.

The suite checks: discovery document round-trip, every Phase 2 endpoint
with mock data, problem-detail emission for malformed inputs, rate-limit
header presence and exhaustion behaviour, replay-defence bounds (300-second
skew, 600-second nonce cache), and audit-log envelope shape.

## B.2 Reference container

The `wia/quantum-network-host:1.0.0` container image implements every Phase 2
endpoint with mock data; integrators exercise their bridge against it
before going to production. The container ships with a small library of
mock scenarios so the conformance suite has fixtures to run against.

## B.3 Companion CLI

The `cli/quantum-network.sh` script ships sample envelope generators (validate,
info, plus phase-specific subcommands) so an implementer can produce
conformant payloads without hand-rolling JSON. The CLI has no dependency
beyond `jq` and POSIX shell, so it runs in any CI environment without
additional tooling installation.

## B.4 Operational considerations

Quantum-network infrastructure has three operational considerations
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
the wider quantum-network standard: entanglement swap; quantum repeater; quantum memory; quantum teleportation; trusted-node network; curve repeater network; quantum-internet; link-state announcement; fidelity-weighted routing; URN scheme; trust list.

Implementers unfamiliar with the domain should treat these terms as
load-bearing — every endpoint, every protocol exchange, and every
integration document below assumes the reader understands what each
term means in context. Expanded definitions appear in the standard's
companion glossary at `https://wiastandards.com/quantum-network/glossary/`.

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

1. Stand up the reference container ('wia/quantum-network-host:1.0.0') in a
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
