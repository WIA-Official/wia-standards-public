# WIA-COMM-006 — Phase 2: API

> Channel + repeater API surface: free-space and fibre channel descriptors, quantum repeater orchestration, and the entanglement-distribution endpoint set integrators code against.

## 4. Quantum Communication Channels

### 4.1 Fiber-Optic QKD

#### 4.1.1 Optical Fiber Properties

**Telecom Wavelengths**:
- 1310 nm (O-band): Loss ~0.3 dB/km
- 1550 nm (C-band): Loss ~0.2 dB/km (optimal)

**Dispersion**: Chromatic and polarization mode dispersion affect timing

**Attenuation Formula**:
```
η = 10^(-αL/10)
```
Where α = loss coefficient (dB/km), L = distance (km)

#### 4.1.2 Maximum Distance

Without repeaters:
```
L_max ≈ -10/α × log₁₀(R_min/R₀)
```

Typical: 100-150 km for practical key rates

#### 4.1.3 Fiber QKD Systems

**Standard Single-Mode Fiber (SMF-28)**:
- Core: 9 μm diameter
- Cladding: 125 μm diameter
- Compatible with existing telecom infrastructure

**Wavelength Division Multiplexing (WDM)**:
- QKD on quantum channel (1550 nm)
- Classical data on separate wavelengths
- Filters required to prevent noise

### 4.2 Free-Space Optical QKD

#### 4.2.1 Atmospheric Transmission

**Transmission Windows**:
- 780-850 nm: Good detector efficiency
- 1550 nm: Eye-safe, lower atmospheric absorption

**Atmospheric Loss**:
```
η = exp(-αL)
```
Where α depends on visibility, weather conditions

**Turbulence Effects**:
- Beam wandering
- Scintillation
- Wavefront distortion

**Mitigation**:
- Adaptive optics
- Multiple apertures
- Beacon tracking

#### 4.2.2 Daylight Operation

**Challenges**:
- Solar background noise
- Atmospheric scattering

**Solutions**:
- Narrow spectral filtering (< 1 nm)
- Temporal gating
- Spatial filtering
- Optimal wavelength selection

#### 4.2.3 Range

**Ground-to-Ground**: 10-100 km
- Clear weather required
- Night operation preferred
- Line-of-sight necessary

**Ground-to-Aircraft**: 20-50 km
**Ground-to-Satellite**: 500-2000 km (LEO)

### 4.3 Satellite Quantum Communication

#### 4.3.1 Satellite QKD Architecture

**Low Earth Orbit (LEO)**:
- Altitude: 500-1200 km
- Orbital period: 90-120 minutes
- Overhead pass: 3-10 minutes
- Global coverage with constellation

**Link Budget**:
```
P_received = P_transmit × G_transmit × G_receive × L_atmosphere × L_space
```

#### 4.3.2 Atmospheric Effects

**Uplink (Ground to Satellite)**:
- Atmospheric turbulence
- Beam expansion
- Absorption and scattering

**Downlink (Satellite to Ground)**:
- Lower turbulence effect
- Simpler beam shaping
- Better link budget

#### 4.3.3 Satellite Systems

**Components**:
- Compact photon source
- Pointing, acquisition, tracking (PAT)
- Attitude control
- Quantum receiver/transmitter

**Key Rates**:
- 1-100 kbps during overhead pass
- Megabit keys per pass possible

**Examples**:
- Micius (China): First QKD satellite, 2016
- QEYSSat (Canada): Planned
- CubeSat QKD missions: Multiple planned

---


## 5. Quantum Repeaters

### 5.1 Need for Quantum Repeaters

**Fiber Loss Problem**:
- Exponential decay: η = 10^(-αL/10)
- Key rate: R ∝ η²
- Beyond 100-150 km, rates become impractical

**Classical Repeaters Won't Work**:
- Cannot amplify quantum states (no-cloning theorem)
- Measurement destroys quantum information

### 5.2 Quantum Repeater Architecture

#### 5.2.1 Basic Components

**Quantum Memory**:
- Store qubits for extended time
- Technologies: rare-earth ions, quantum dots, atomic ensembles
- Requirements: Long coherence time, high efficiency

**Entanglement Swapping**:
- Bell state measurement on intermediate qubits
- Extends entanglement range
- Success probability: ~25% (standard BSM)

**Purification**:
- Improve fidelity of noisy entanglement
- Multiple pairs → fewer high-fidelity pairs
- Required for long-distance links

#### 5.2.2 Repeater Chain

**Segmented Architecture**:
```
A ←→ R1 ←→ R2 ←→ R3 ←→ B
```

**Process**:
1. Establish entanglement in each segment
2. Store qubits in quantum memory
3. Perform entanglement swapping at repeaters
4. Result: Entanglement between A and B

**Key Rate Scaling**:
- Without repeaters: R ∝ η²
- With N repeaters: R ∝ (η₀)^(2N) × p_success
- Polynomial vs exponential scaling

### 5.3 Quantum Memory Technologies

#### 5.3.1 Rare-Earth Ion Doped Crystals

**Materials**: Pr³⁺:Y₂SiO₅, Eu³⁺:Y₂SiO₅, Er³⁺:LiNbO₃

**Properties**:
- Coherence time: up to 6 hours
- Wavelength: 600-1550 nm
- Temperature: 3-10 K
- Efficiency: 20-70%

#### 5.3.2 Atomic Ensembles

**Technologies**: Cold atoms, warm vapor

**Properties**:
- Coherence time: milliseconds to seconds
- Wavelength: typically 780-850 nm
- Temperature: Room temp or μK
- Scalability: Good

#### 5.3.3 Nitrogen-Vacancy (NV) Centers

**Material**: Diamond with nitrogen-vacancy defects

**Properties**:
- Coherence time: milliseconds
- Temperature: Room temperature possible
- Wavelength: 637 nm
- Advantage: Room temperature operation

---


## 6. Entanglement Distribution

### 6.1 Entangled Photon Sources

#### 6.1.1 Spontaneous Parametric Down-Conversion (SPDC)

**Process**:
- Pump photon → Two entangled photons
- Nonlinear crystal (BBO, KTP, PPKTP)
- Wavelength: pump (UV/blue) → signal + idler (red/IR)

**Type-I SPDC**: Same polarization
**Type-II SPDC**: Orthogonal polarization (preferred for entanglement)

**Bell State Generation**:
```
|Φ⁺⟩ = (|HH⟩ + |VV⟩)/√2
```

**Properties**:
- Brightness: 10⁶-10⁹ pairs/s/mW
- Heralding efficiency: 30-70%
- Spectral purity: Good with filtering

#### 6.1.2 Quantum Dot Sources

**Technology**: InAs/GaAs semiconductor quantum dots

**Advantages**:
- On-demand generation
- High brightness
- Electrical pumping
- Integrated photonics compatible

**Challenges**:
- Cryogenic operation
- Fabrication complexity
- Wavelength stability

### 6.2 Entanglement Swapping

#### 6.2.1 Protocol

**Initial State**: Two pairs A-B and B-C
```
|ψ⟩_AB ⊗ |ψ⟩_BC = |Φ⁺⟩_AB ⊗ |Φ⁺⟩_BC
```

**Bell Measurement on B**:
- Measure qubits at node B
- Result: One of four Bell states

**Outcome**: Entanglement between A and C
```
|Φ⁺⟩_AC, |Φ⁻⟩_AC, |Ψ⁺⟩_AC, or |Ψ⁻⟩_AC
```

**Success Probability**: 25% (standard BSM)
- Can be improved with auxiliary photons

#### 6.2.2 Fidelity Decay

Fidelity after N swaps with initial fidelity F₀:
```
F_N ≈ F₀^N
```

**Purification Required**: For F₀ < 1, purification needed after multiple swaps

### 6.3 Entanglement Purification

#### 6.3.1 BBPSSW Protocol

**Input**: Two low-fidelity pairs (F < 1)
**Output**: One higher-fidelity pair
**Cost**: Lose one pair

**Fidelity Improvement**:
```
F' = [F² + ((1-F)/3)²] / [F² + 2F(1-F) + 5((1-F)/3)²]
```

**Iteration**: Can be repeated for further improvement

#### 6.3.2 Pumping Purification

**Multiple Rounds**: Iteratively improve fidelity
**Trade-off**: Pairs vs. Fidelity
**Threshold**: F > 0.5 required for convergence

---



## A.1 Endpoint reference

```http
POST /qkd/v1/session/start    # initiate BB84/E91/B92/MDI-QKD/CV-QKD
GET  /qkd/v1/session/{id}     # session state
POST /qkd/v1/key/derive       # request a fresh key from an open session
POST /qkd/v1/repeater/swap    # entanglement swapping at a repeater
GET  /qkd/v1/channel/health   # channel telemetry: dark count, loss, drift
```

Every endpoint is HTTPS over TLS 1.3 (IETF RFC 8446) only and follows the discovery convention at `/.well-known/wia-quantum-communication`.

## A.2 Channel descriptor envelope

```json
{
  "type": "channel_descriptor",
  "channel_id": "ch_01HX...",
  "kind": "fibre_optic" | "free_space" | "satellite",
  "wavelength_nm": 1550,
  "loss_db_per_km": 0.18,
  "dark_count_rate_hz": 0.001,
  "supported_protocols": ["BB84", "E91", "MDI-QKD"]
}
```

Channel descriptors are republished at least daily; consumers refuse stale descriptors older than 24 hours.

## A.3 Repeater orchestration

A multi-hop QKD path uses quantum repeaters with entanglement swapping. The orchestration envelope chains repeater identities and the swap operations so the end-to-end key is reconstructible by Alice and Bob without trusting any individual repeater operator.

## A.4 Conformance test coverage

Conformant hosts MUST pass: discovery round-trip, BB84/E91/MDI-QKD session start, channel-descriptor freshness check, repeater chain swap, key derivation round-trip, problem-detail emission for QBER threshold breaches, and rate-limit headers on every response.


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
