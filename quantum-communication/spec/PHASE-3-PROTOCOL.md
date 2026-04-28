# WIA-COMM-006 — Phase 3: Protocol

> QKD security analysis and performance-metric protocol layer. Every analytical claim is wire-level with a documented evidence chain so auditors can reconstruct security guarantees later.

## 7. Security Analysis

### 7.1 Quantum Attacks

#### 7.1.1 Photon Number Splitting (PNS)

**Vulnerability**: Weak coherent pulses contain multi-photon states

**Attack**:
1. Eve splits off one photon from multi-photon pulses
2. Blocks single-photon pulses
3. Measures intercepted photons after basis announcement

**Countermeasures**:
- Decoy states (varying intensities)
- Monitor photon statistics
- Single-photon sources

#### 7.1.2 Trojan Horse Attacks

**Attack**: Eve sends bright light into Alice/Bob's system
**Goal**: Extract information from internal components

**Countermeasures**:
- Optical isolators
- Monitor for back-reflected light
- Wavelength filtering

#### 7.1.3 Detector Blinding

**Attack**: Blind detectors with bright light, control in classical regime

**Countermeasures**:
- Monitor detector operation
- Randomize detection efficiency
- Use detector self-testing

#### 7.1.4 Phase Remapping

**Attack**: Manipulate phase in phase-encoded QKD

**Countermeasures**:
- Active phase randomization
- Monitor phase stability
- Use polarization encoding

### 7.2 Side-Channel Attacks

**Spatial Mode Vulnerabilities**:
- Information in spatial modes
- Monitor all optical modes

**Temporal Correlations**:
- Timing information leakage
- Randomize timing

**Device Imperfections**:
- State preparation flaws
- Measurement device attacks
- Use MDI-QKD (Measurement-Device-Independent)

### 7.3 Authentication

**Classical Channel Authentication**:
- Required for classical post-processing
- Use pre-shared key or post-quantum signatures
- Wegman-Carter authentication
- Renew authentication keys regularly

### 7.4 Information-Theoretic Security

**Composable Security**:
- ε-security definition
- Security against arbitrary attacks
- Finite-key analysis

**Security Parameters**:
- Correctness: P(error) < ε_cor
- Secrecy: I(Eve : Key) < ε_sec
- Total: ε = ε_cor + ε_sec

**Typical Values**: ε = 10⁻⁶ to 10⁻¹⁰

---


## 8. Performance Metrics

### 8.1 Quantum Bit Error Rate (QBER)

**Definition**:
```
QBER = (Number of errors) / (Total sifted bits)
```

**Thresholds**:
- BB84: QBER < 11% (individual attacks)
- BB84: QBER < 20% (collective attacks with good error correction)
- Practical: QBER < 5% preferred

**Sources of QBER**:
- Detector dark counts
- Background light
- Optical losses
- Channel noise
- Eavesdropping

### 8.2 Secure Key Rate

**Asymptotic Key Rate**:
```
r = q [I(A:B) - I(A:E)]
```
Where:
- q = sifting efficiency
- I(A:B) = mutual information Alice-Bob
- I(A:E) = Eve's information (upper bound)

**Finite-Key Rate**:
Includes statistical fluctuations and finite-size effects

**Practical Rates**:
- 10 km fiber: 10-100 kbps
- 50 km fiber: 1-10 kbps
- 100 km fiber: 100-1000 bps
- 150 km fiber: 10-100 bps

### 8.3 Channel Transmittance

**Fiber**:
```
η = 10^(-0.2 × L/10)  (for 1550nm, L in km)
```

**Free-Space**:
```
η = η_atm × η_pointing × η_diffraction
```

**Satellite**:
```
η = η_atmosphere × η_space × η_pointing × η_telescope
```

### 8.4 Detector Performance

**Detection Efficiency (η_det)**:
- APD: 20-60%
- SNSPD: 70-95%
- Required: > 20% for practical systems

**Dark Count Rate**:
- APD: 100-1000 Hz
- SNSPD: 1-100 Hz
- Impact: Limits maximum range

**Timing Jitter**:
- APD: 300-500 ps
- SNSPD: 30-100 ps
- Impact: Time-bin encoding resolution

---



## A.1 QBER threshold model

The standard pins QBER thresholds per protocol:

| Protocol | QBER threshold | Rationale |
|----------|----------------|-----------|
| BB84 | 11% | Information-theoretic security against intercept-resend |
| E91 | 7.07% | CHSH inequality violation requires QBER ≤ S/4 |
| B92 | 5% | Conservative threshold for non-orthogonal state distinguishing |
| MDI-QKD | 1% per channel | Measurement device untrusted; both channels degrade additively |
| CV-QKD | per protocol-specific Holevo bound | Continuous-variable Gaussian modulation |

Sessions exceeding threshold MUST abort with a problem document of type `.../qber-threshold-breached`.

## A.2 Privacy amplification protocol

The privacy-amplification step compresses a sifted key (with partial information leakage to an eavesdropper) into a shorter key with negligible leakage. The standard recommends Toeplitz-matrix hashing with seed transmitted over an authenticated classical channel; the seed appears in the post-processing audit envelope.

## A.3 Authentication of the classical channel

The classical channel between Alice and Bob carries basis-choice announcements, error correction syndromes, and privacy-amplification seed. It MUST be authenticated; the standard requires Ed25519 signatures on every classical message keyed to a pre-shared authentication tag (initial bootstrap) or a previous-session derived tag (subsequent sessions).

## A.4 Replay defence

Every QKD protocol envelope carries a 96-bit nonce + RFC 3339 timestamp. Receivers reject envelopes with skew > ±300s and maintain a 600-second seen-nonce cache.


## A.5 Quantum cryptanalysis impact

Shors algorithm threatens RSA, DSA, ECDSA, and ECDH. The standard's
PQC-integration discipline (Phase 4) is the migration plan; the
classical channel between Alice and Bob MUST migrate to PQC
signatures (NIST FIPS 204 ML-DSA) by 2030 to remain secure under
the assumption of cryptographically relevant quantum computers
appearing within the protected lifetime of derived keys.

## A.6 Decoy-state extension

Decoy-state QKD defends against photon-number-splitting attacks by
randomly emitting decoy pulses at varying intensities. The standard's
decoy-state extension envelope carries the decoy schedule and the
detection statistics so the security argument is reproducible by
auditors after the fact.

## A.7 Continuous-variable QKD considerations

CV-QKD encodes information in quadratures of the optical field
rather than discrete qubit states. The Phase 1 envelopes for CV-QKD
carry quadrature samples plus the channel transmittance and excess
noise estimates; the Phase 3 protocol layer adapts the privacy
amplification to the continuous-variable Holevo bound rather than
the discrete-variable QBER threshold.


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
