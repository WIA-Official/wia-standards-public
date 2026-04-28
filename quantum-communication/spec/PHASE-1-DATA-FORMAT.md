# WIA-COMM-006 — Phase 1: Data Format

> QKD canonical envelopes: protocol descriptors (BB84, E91, B92, MDI-QKD, CV-QKD), polarisation-encoding records, and the runtime conventions that fix the wire format of every protocol in subsequent phases.

## 1. Introduction

### 1.1 Purpose

This specification defines a comprehensive framework for quantum communication systems, enabling unconditionally secure key distribution and quantum information transmission using fundamental principles of quantum mechanics.

### 1.2 Scope

The standard covers:
- Quantum Key Distribution (QKD) protocols: BB84, E91, B92
- Photon polarization encoding schemes
- Fiber-based and free-space quantum channels
- Satellite quantum communication
- Quantum repeater architecture
- Entanglement distribution mechanisms
- QBER (Quantum Bit Error Rate) monitoring
- Post-quantum cryptography transition

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to provide information-theoretically secure communication infrastructure that protects individual privacy and organizational security for the benefit of all humanity.

### 1.4 Terminology

- **Qubit**: Quantum bit, basic unit of quantum information
- **QKD**: Quantum Key Distribution
- **QBER**: Quantum Bit Error Rate
- **Fidelity**: Measure of quantum state quality (0-1)
- **Photon**: Elementary particle of light carrying quantum information
- **Polarization**: Direction of electric field oscillation in photon
- **Entanglement**: Quantum correlation between particles
- **BB84**: Bennett-Brassard 1984 QKD protocol
- **E91**: Ekert 1991 entanglement-based QKD protocol
- **B92**: Bennett 1992 two-state QKD protocol

---


## 2. Quantum Key Distribution Protocols

### 2.1 BB84 Protocol

The BB84 protocol is the most widely deployed QKD protocol, using four quantum states encoded in two conjugate bases.

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

**Phase 1: Quantum Transmission**
1. Alice generates random bit string: `b = [0,1,0,1,1,0,...]`
2. Alice generates random basis string: `a = [+,×,+,+,×,...]`
3. Alice encodes each bit in chosen basis and transmits photons to Bob
4. Bob generates random basis string: `b' = [+,+,×,+,×,...]`
5. Bob measures each photon in his chosen basis

**Phase 2: Classical Post-Processing**
1. **Sifting**: Alice and Bob publicly compare basis choices (not measurement results)
2. Keep only bits where bases matched (sifted key)
3. Discard approximately 50% of bits where bases didn't match

**Phase 3: Error Estimation**
1. Sample random subset of sifted key (typically 10-20%)
2. Publicly compare sampled bits to estimate QBER
3. QBER = (number of errors) / (total sampled bits)
4. If QBER > 11%, abort protocol (possible eavesdropper)

**Phase 4: Error Correction**
1. Use classical error correction codes (Cascade, LDPC, etc.)
2. Correct remaining errors in unsaved portion of sifted key
3. Verify error correction success through parity checks

**Phase 5: Privacy Amplification**
1. Apply universal hash function to reduce key length
2. Remove any information potentially gained by eavesdropper
3. Output final secure key

#### 2.1.3 Security Proof

**Individual Attacks**: Proven secure against individual photon attacks with QBER < 11%

**Collective Attacks**: Secure with appropriate error correction and privacy amplification

**Coherent Attacks**: Secure in asymptotic limit with proper parameter selection

#### 2.1.4 Key Rate Formula

For lossy channel with transmittance η and QBER e:

```
R = η [1 - h(e) - h(e)]
```

Where h(x) is binary entropy function:
```
h(x) = -x log₂(x) - (1-x) log₂(1-x)
```

For practical systems:
```
R ≈ (η/2) [1 - 2h(e)]  bits per sifted bit
```

### 2.2 E91 Protocol

Entanglement-based QKD protocol using EPR pairs and Bell inequality violations.

#### 2.2.1 Entanglement Source

Generate Bell state:
```
|Φ⁺⟩ = (|HH⟩ + |VV⟩)/√2
```

Distribute one photon to Alice, one to Bob.

#### 2.2.2 Measurement Bases

**Alice's bases**: a₁, a₂, a₃ at angles 0°, 45°, 90°
**Bob's bases**: b₁, b₂, b₃ at angles 45°, 90°, 135°

#### 2.2.3 Protocol Steps

1. **Entanglement Distribution**: EPR source sends entangled photon pairs
2. **Random Measurement**: Alice and Bob randomly choose measurement bases
3. **Classical Communication**: Announce basis choices (not results)
4. **Key Generation**: Use results from correlated bases (a₁,b₁), (a₃,b₃)
5. **Bell Test**: Use results from (a₁,b₂), (a₂,b₁), (a₂,b₃), (a₃,b₂) to test CHSH inequality
6. **Security Verification**: If S > 2√2 ≈ 2.828, quantum correlations confirmed
7. **Post-Processing**: Error correction and privacy amplification

#### 2.2.4 CHSH Inequality

```
S = |E(a₁,b₁) - E(a₁,b₂) + E(a₂,b₁) + E(a₂,b₂)|
```

Classical bound: S ≤ 2
Quantum maximum: S = 2√2 ≈ 2.828

If S > 2, eavesdropping detected.

#### 2.2.5 Device-Independent Security

E91 can provide device-independent security:
- No trust in quantum devices required
- Security based solely on Bell violation
- Resistant to side-channel attacks on detectors

### 2.3 B92 Protocol

Simplified two-state QKD protocol for resource-constrained systems.

#### 2.3.1 Quantum States

Uses only two non-orthogonal states:
```
|ψ₀⟩ = |0⟩  (horizontal, encodes bit 0)
|ψ₁⟩ = |+⟩  (diagonal, encodes bit 1)
```

#### 2.3.2 Protocol Steps

1. **Preparation**: Alice randomly chooses bits and sends corresponding states
2. **Measurement**: Bob randomly measures in {|0⟩,|1⟩} or {|+⟩,|−⟩} basis
3. **Announcement**: Bob announces which measurements were inconclusive
4. **Sifting**: Keep only conclusive measurements
5. **Post-Processing**: Error correction and privacy amplification

#### 2.3.3 Efficiency

- Lower efficiency than BB84 (~25% vs ~50% sifting efficiency)
- Simpler implementation (only two states)
- Suitable for IoT and embedded systems

---


## 3. Photon Polarization Encoding

### 3.1 Linear Polarization

**Horizontal (H)**: 0°
**Vertical (V)**: 90°
**Diagonal (D)**: 45°
**Anti-Diagonal (A)**: 135°

Mathematical representation:
```
|H⟩ = |0⟩ = [1, 0]ᵀ
|V⟩ = |1⟩ = [0, 1]ᵀ
|D⟩ = |+⟩ = (|H⟩ + |V⟩)/√2
|A⟩ = |−⟩ = (|H⟩ - |V⟩)/√2
```

### 3.2 Circular Polarization

**Left Circular (L)**:
```
|L⟩ = (|H⟩ + i|V⟩)/√2
```

**Right Circular (R)**:
```
|R⟩ = (|H⟩ - i|V⟩)/√2
```

### 3.3 Phase Encoding

Alternative to polarization encoding for fiber systems:

**Time-bin encoding**: Photon in early or late time bin
**Phase encoding**: Relative phase in Mach-Zehnder interferometer

Advantages:
- More stable in fiber
- Lower polarization mode dispersion
- Better long-distance performance

### 3.4 Polarization Measurement

**Polarizing Beam Splitter (PBS)**: Separates H and V polarization

**Half-Wave Plate (HWP)**: Rotates polarization
- 22.5° rotation converts between rectilinear and diagonal bases

**Single-Photon Detectors**:
- **APD** (Avalanche Photodiode): Efficiency ~20%, simple
- **SNSPD** (Superconducting Nanowire): Efficiency ~85%, cryogenic
- **PMT** (Photomultiplier Tube): Efficiency ~40%, high dark counts

---



## A.1 Canonical envelope conventions

Every Phase 1 envelope follows the WIA family baseline: UTF-8 JSON with RFC 8785 (JSON Canonicalization Scheme) for signed payloads, Ed25519 (IETF RFC 8032) signatures, and ULIDs as identifiers. Photon-counting timestamps use TAI per BIPM conventions to avoid leap-second ambiguity in security analysis.

## A.2 Worked BB84 protocol envelope

```json
{
  "wia_qkd_version": "1.0.0",
  "type": "qkd_session",
  "session_id": "01HX...",
  "protocol": "BB84",
  "alice_id": "did:wia:qkd:alice-node",
  "bob_id":   "did:wia:qkd:bob-node",
  "channel_kind": "fibre_optic",
  "channel_length_km": 25.4,
  "key_length_bits": 4096,
  "qber_estimate": 0.024,
  "secret_key_rate_bps": 1240.5,
  "started_at": "RFC 3339",
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

QBER (Quantum Bit Error Rate) is the dominant security parameter. The standard requires QBER threshold publication in the discovery document so a downstream consumer can refuse sessions whose QBER exceeds the security argument.

## A.3 Polarisation-encoding record

```json
{
  "type": "polarization_record",
  "session_id": "01HX...",
  "basis_choice_history": "<base64-bitstring>",
  "measurement_outcomes": "<base64-bitstring>",
  "sifted_key_length_bits": 2048,
  "post_processing_state": "raw" | "sifted" | "reconciled" | "amplified"
}
```

The polarisation record carries the raw measurement output and the post-processing state so an auditor can reconstruct the key-distillation chain. Keys at `post_processing_state=raw` MUST NOT be exposed outside the QKD nodes.

## A.4 MDI-QKD measurement-device-independent extension

MDI-QKD removes the trust assumption on the measurement device by routing both Alice and Bobs photons through an untrusted Bell-state measurement device. The Phase 1 envelope adds a `measurement_device_id` field referencing the untrusted node so audit trails capture which device participated in each session.


## B.1 Conformance test suite

A black-box conformance test suite is published at
`https://github.com/WIA-Official/wia-quantum-communication-conformance` and walks
through every public endpoint plus the cross-Phase integration
scenarios. Hosts publishing `bridge_profile=Full` SHOULD additionally
pass the suite's bridge-extension tests.

The suite checks: discovery document round-trip, every Phase 2 endpoint
with mock data, problem-detail emission for malformed inputs, rate-limit
header presence and exhaustion behaviour, replay-defence bounds (300-second
skew, 600-second nonce cache), and audit-log envelope shape.

## B.2 Reference container

The `wia/quantum-communication-host:1.0.0` container image implements every Phase 2
endpoint with mock data; integrators exercise their bridge against it
before going to production. The container ships with a small library of
mock scenarios so the conformance suite has fixtures to run against.

## B.3 Companion CLI

The `cli/quantum-communication.sh` script ships sample envelope generators (validate,
info, plus phase-specific subcommands) so an implementer can produce
conformant payloads without hand-rolling JSON. The CLI has no dependency
beyond `jq` and POSIX shell, so it runs in any CI environment without
additional tooling installation.

## B.4 Operational considerations

Quantum-key-distribution infrastructure has three operational considerations
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
the wider quantum-communication standard: QBER (Quantum Bit Error Rate); BB84 (Bennett-Brassard 1984); E91 (Ekert 1991); B92 (Bennett 1992); MDI-QKD (Measurement-Device-Independent QKD); CV-QKD (Continuous-Variable QKD); twin-field QKD; decoy state; privacy amplification; sifted key; reconciled key.

Implementers unfamiliar with the domain should treat these terms as
load-bearing — every endpoint, every protocol exchange, and every
integration document below assumes the reader understands what each
term means in context. Expanded definitions appear in the standard's
companion glossary at `https://wiastandards.com/quantum-communication/glossary/`.

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

1. Stand up the reference container ('wia/quantum-communication-host:1.0.0') in a
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
