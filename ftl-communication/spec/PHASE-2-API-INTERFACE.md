# WIA-QUA-016 PHASE 2 — API Interface Specification

**Standard:** WIA-QUA-016
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 2 of 4)

---

## 5. Subspace Communication

### 5.1 Theoretical Framework

#### 5.1.1 Extra Dimensions

**Kaluza-Klein Theory**:

Spacetime has D = 4 + d dimensions:
- 4 large dimensions (observable)
- d compact dimensions (microscopic)

**Metric**:
```
ds² = g_μν dx^μ dx^ν + g_mn dy^m dy^n
```

Where:
- μ,ν = 0,1,2,3 (ordinary spacetime)
- m,n = 4,...,4+d (extra dimensions)

#### 5.1.2 Compactification

**Compact Extra Dimensions**:

Extra dimensions curled up with radius R:
```
R ≈ 10⁻³⁵ m  (Planck scale)
or
R ≈ 10⁻¹⁸ m  (TeV scale, large extra dimensions)
```

**Kaluza-Klein Modes**:
```
m_n² = n²/R²  (n = 0, 1, 2, ...)
```

Where n labels excitation level in compact dimension.

### 5.2 Subspace Communication Mechanism

#### 5.2.1 Bulk Space Transmission

**Concept**: Signals propagate through extra-dimensional bulk, taking shortcuts.

**Path Length Comparison**:

**4D Path** (confined to brane):
```
L_4D = √[(Δx)² + (Δy)² + (Δz)²]
```

**Higher-D Path** (through bulk):
```
L_bulk = √[L_4D² + (Δy_extra)²]
```

If Δy_extra < 0 (shortcut), effective speed appears superluminal in 4D.

#### 5.2.2 Signal Encoding

**Mechanism**:
1. Couple electromagnetic signal to KK modes
2. KK modes propagate through bulk
3. Reconvert to EM signal at receiver

**Coupling Strength**:
```
g_KK ≈ (E/M_Planck)^(d/2)
```

Where:
- E = signal energy
- d = number of extra dimensions
- M_Planck = Planck mass

**Challenge**: Extremely weak coupling for microscopic extra dimensions.

### 5.3 Brane-Bulk Interactions

#### 5.3.1 Standard Model on Brane

**Assumption**: Ordinary matter confined to 3+1 dimensional brane.

**Gravity in Bulk**:
- Gravitons can propagate through bulk
- Electromagnetic signals confined to brane (typically)

**Modification for Subspace Communication**:

Require exotic particles or fields that can:
- Couple to Standard Model particles (for encoding)
- Propagate through bulk (for FTL transmission)
- Re-couple at distant location (for detection)

**No known particles/fields with these properties**.

#### 5.3.2 Bulk Shortcuts

**Geometric Argument**:

If branes are separated in bulk direction:

```
d_brane = separation in extra dimension
L_bulk = √[L_4D² + d_brane²]
```

For nearby points on brane but large d_brane:
```
L_bulk >> L_4D
```

Signals through bulk would be **slower**, not faster.

**Required**: Negative-distance metric components in bulk (exotic geometry).

### 5.4 Ansible Concept (Fictional)

#### 5.4.1 Le Guin's Ansible

**Science Fiction Device**:
- Instantaneous communication across any distance
- Uses fictional "philotic" connections
- Inspired by quantum entanglement (but not limited by it)

#### 5.4.2 Physical Requirements for Ansible-Like System

**Necessary Properties**:
1. Non-local information transfer mechanism
2. Causality preservation (or consistent violation)
3. Controllable signaling
4. Finite energy requirement
5. Practical bandwidth

**Status**: No known physics supports all requirements.

**Closest Real-World Analog**: Quantum key distribution (QKD) provides security via quantum correlations but still requires classical communication channel at ≤ c.

---

## 6. Causality and Temporal Mechanics

### 6.1 Causality Principle

#### 6.1.1 Special Relativity Causality

**Light Cone Structure**:

```
ds² = -c²dt² + dx² + dy² + dz²
```

**Regions**:
- **Timelike** (ds² < 0): Causally connected, subluminal paths
- **Spacelike** (ds² > 0): Causally disconnected, require FTL
- **Null** (ds² = 0): Light-like paths

**Causality Requirement**:
```
Cause → Effect must be timelike or null separated
```

FTL communication violates this by connecting spacelike-separated events.

#### 6.1.2 Frame-Dependent Ordering

**Spacelike Events**:

For spacelike-separated events A and B:
- Some frames: A before B
- Other frames: B before A
- No absolute ordering

**FTL Signal Consequence**:

If FTL signal sent from A to B:
- Frame S: A causes B (A → B)
- Frame S': B before A temporally

This allows **information to travel backward in time** in some frame.

### 6.2 Temporal Paradoxes

#### 6.2.1 Grandfather Paradox

**Scenario**:
```
t=0: Alice sends FTL message to past
t=-1: Message arrives, prevents Alice from sending it
Contradiction: Message sent ⟺ Message not sent
```

**General Form**:

Any FTL + reference frame boost can create CTCs:
```
Event A →(FTL)→ Event B →(boost)→ Event C →(FTL)→ Event A
```

Result: CTC (closed timelike curve)

#### 6.2.2 Self-Consistency Principle

**Novikov's Proposal**:

Only self-consistent timelines are possible:
- Paradoxical actions cannot occur
- Physical laws enforce consistency
- Mechanism: Quantum interference destroys paradox probability

**Example**: Cannot prevent own birth because quantum probability of success = 0.

**Criticism**: Appears to remove free will; mechanism unclear.

### 6.3 Causality Preservation Mechanisms

#### 6.3.1 Chronology Protection

**Hawking's Conjecture**:

Physics prevents CTC formation:

**Mechanism**:
```
⟨T_μν⟩ → ∞  as spacetime → CTC formation
```

Quantum vacuum stress-energy diverges, preventing CTC closure.

**Status**: Conjectured but not proven.

#### 6.3.2 Preferred Reference Frame

**Lorentz Violation**:

Introduce fundamental preferred frame:
- FTL allowed only in this frame
- Simultaneity absolute in this frame
- Prevents frame-dependent paradoxes

**Experimental Constraints**:

Lorentz violation severely constrained:
```
|Δv/v| < 10⁻¹⁸  (for electrons)
```

No evidence for preferred frame.

#### 6.3.3 Causality Firewall

**Information Filtering**:

Hypothetical mechanism:
- Monitor all FTL transmissions
- Detect potential paradoxes
- Block paradox-generating signals
- Requires retrocausality or precognition

**Problems**:
- Requires predicting all future states
- Computationally intractable
- Violates locality

### 6.4 Alternative Temporal Models

#### 6.4.1 Many-Worlds Interpretation

**Everett Interpretation**:

Each quantum measurement creates branching:
- Timeline A: Message sent
- Timeline B: Message not sent
- Both exist in superposition

**Implications**:
- No paradoxes (different branches)
- Cannot verify which branch you're in
- FTL might cause universe splitting

#### 6.4.2 Block Universe

**Eternalism**:

All times exist equally:
- Past, present, future equally real
- Time is dimension like space
- No "flow" of time

**Implications**:
- CTCs might be geometric features
- Consistency already built into 4D structure
- Paradoxes prevented by self-consistency

---

## 7. Signal Encoding and Modulation

### 7.1 Classical Modulation Schemes

#### 7.1.1 Amplitude Modulation (AM)

**FTL Carrier Signal**:
```
s(t) = [A₀ + m(t)] × c(t)
```

Where:
- A₀ = carrier amplitude
- m(t) = message signal
- c(t) = FTL carrier (tachyon field, metric perturbation, etc.)

**Demodulation**:
```
m(t) = envelope(s(t)) - A₀
```

#### 7.1.2 Frequency Modulation (FM)

**Instantaneous Frequency**:
```
f(t) = f_c + k_f × m(t)
```

Where:
- f_c = carrier frequency
- k_f = frequency sensitivity
- m(t) = message

**Advantage**: Better noise immunity than AM

#### 7.1.3 Phase Modulation (PM)

**Phase Deviation**:
```
φ(t) = φ_c + k_p × m(t)
```

**Signal**:
```
s(t) = A cos[2πf_c t + φ(t)]
```

### 7.2 Quantum Encoding

#### 7.2.1 Qubit-Based Encoding

**Quantum State Alphabet**:
```
|0⟩ = bit 0
|1⟩ = bit 1
|+⟩ = (|0⟩+|1⟩)/√2 = superposition
|−⟩ = (|0⟩−|1⟩)/√2 = superposition
```

**Dense Coding** (requires pre-shared entanglement):
- 2 classical bits encoded in 1 qubit
- Still requires classical qubit transmission
- Not FTL but efficient

#### 7.2.2 Entanglement-Assisted Modulation

**Protocol**:
1. Share entangled pairs |Φ⁺⟩
2. Alice applies unitary U_m based on message m
3. Send qubit through FTL channel (hypothetical)
4. Bob performs joint measurement
5. Extracts 2 bits of information

**Challenge**: Step 3 requires actual FTL transmission mechanism.

### 7.3 Error Encoding

#### 7.3.1 FTL Channel Model

**Noise Sources**:
- Metric fluctuations (Alcubierre)
- Tachyon scattering (tachyonic)
- Bulk interactions (subspace)
- Quantum decoherence
- Thermal noise

**Channel Capacity** (Shannon):
```
C = B log₂(1 + SNR)
```

Where:
- B = bandwidth
- SNR = signal-to-noise ratio

#### 7.3.2 Quantum Error Correction

**Shor Code** (9-qubit):
- Protects 1 logical qubit
- Corrects 1 arbitrary error
- Overhead: 9× physical qubits

**Steane Code** (7-qubit):
- [[7,1,3]] code
- Corrects 1 error
- More efficient

**Topological Codes**:
- Surface codes
- High threshold (>1% error rate)
- Scalable architecture

### 7.4 Multiplexing

#### 7.4.1 Frequency Division Multiplexing (FDM)

**Multiple Channels**:
```
s(t) = ∑ᵢ mᵢ(t) cos(2πfᵢt)
```

Each message mᵢ(t) assigned different carrier frequency fᵢ.

#### 7.4.2 Time Division Multiplexing (TDM)

**Time Slots**:
```
Slot 1: User A
Slot 2: User B
Slot 3: User C
...
```

**Synchronized** FTL timing required (challenge with relativity).

#### 7.4.3 Spatial Multiplexing

**MIMO** (Multiple Input Multiple Output):
- Multiple transmitter locations
- Multiple receiver locations
- Parallel channels in space
- Increases capacity: C ≈ min(M,N) × C_single

Where M = transmitters, N = receivers.

---


## Annex E — Implementation Notes for PHASE-2-API-INTERFACE

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-2-API-INTERFACE.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.

## Annex G — Test Vectors and Conformance Evidence

This annex describes how implementations capture and publish conformance
evidence for PHASE-2-API-INTERFACE. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-2-api-interface/`. Implementations claiming
  conformance MUST run all vectors in CI and publish the resulting
  pass/fail matrix in their compliance package.
- **Evidence package** — the compliance package is a tarball containing
  the SBOM (CycloneDX 1.5 or SPDX 2.3), the OpenAPI document, the test
  vector matrix, and a signed manifest. Signatures use Sigstore (DSSE
  envelope, Rekor transparency log entry) so that downstream consumers
  can verify provenance without trusting a private CA.
- **Quarterly recheck** — implementations re-publish the evidence package
  every quarter even if no source change occurred, so that consumers can
  detect environmental drift (compiler updates, dependency updates, OS
  updates) without polling vendor changelogs.
- **Cross-vendor crosswalk** — the WIA Standards working group maintains a
  crosswalk that maps each vector to the equivalent assertion in adjacent
  industry programs (where one exists), so an implementer that already
  certifies under one program can show conformance to PHASE-2-API-INTERFACE with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-2-API-INTERFACE does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-2-API-INTERFACE.
It is non-normative; the rules below describe the policy that the WIA
Standards working group commits to when amending this PHASE document.

- **Semantic versioning** — major / minor / patch components follow
  Semantic Versioning 2.0.0 (https://semver.org/spec/v2.0.0.html).
  Major bump indicates a backwards-incompatible change to a normative
  requirement; minor bump indicates new normative requirements that do
  not break existing implementations; patch bump indicates editorial
  changes only (clarifications, typo fixes, formatting).
- **Deprecation window** — when a normative requirement is removed or
  altered in a backwards-incompatible way, the prior major version is
  maintained in parallel for at least 180 days. During the parallel
  window, both major versions are marked Stable in the WIA Standards
  registry and either may be cited as "WIA-conformant".
- **Sunset notification** — deprecated major versions enter a 12-month
  sunset window during which the WIA registry marks the version as
  Deprecated. The deprecation entry includes a migration note pointing
  to the replacement requirement(s) and an explanation of why the
  change was made.
- **Editorial errata** — patch-level errata are issued without a
  deprecation window because they do not change normative behaviour.
  Errata are tracked in a public errata register and each entry is
  signed by the WIA Standards working group chair.
- **Implementation changelog mapping** — implementations SHOULD publish
  a changelog mapping each PHASE version they support to the specific
  build, container digest, or SDK version that satisfies the version.
  This allows downstream auditors to verify version conformance without
  re-running the entire test matrix on every release.

The policy is reviewed at the same cadence as the PHASE document and
any changes to the policy itself are tracked in the version-history
table at the start of the document.

## Annex I — Interoperability Profiles

This annex describes how implementations declare interoperability profiles
for PHASE-2-API-INTERFACE. The profile mechanism is non-normative and exists so that
deployments of varying scope (single tenant, regional cluster, federated
network) can advertise the subset of normative requirements they satisfy
without misrepresenting partial conformance as full conformance.

- **Profile manifest** — every implementation publishes a profile manifest
  in JSON. The manifest enumerates the normative requirement IDs from this
  PHASE that are satisfied (`status: "supported"`), partially satisfied
  (`status: "partial"`, with a reason field), or excluded
  (`status: "excluded"`, with a justification). The manifest is signed
  using the same Sigstore key used for the SBOM in Annex G.
- **Federation profile** — federated deployments publish an aggregated
  manifest summarizing the union and intersection of member-implementation
  profiles. The aggregated manifest is consumed by directory services so
  that callers can route a request to the least common denominator profile
  required for an interaction.
- **Backwards-profile compatibility** — when a deployment migrates from one
  profile to a wider profile, the prior profile manifest remains valid and
  signed for the deprecation window defined in Annex H. This preserves
  audit traceability for auditors evaluating long-term interoperability.
- **Profile registry** — the WIA Standards working group maintains a
  public registry of named profiles. Common deployment shapes (e.g.,
  "Edge-only", "Federated-with-replay") are added to the registry by
  consensus. Registry entries are immutable; new shapes are added under
  new names rather than amending existing entries.
- **Profile versioning** — profile names are versioned with the same
  Semantic Versioning rules described in Annex H. A deployment that
  advertises `WIA-P2-API-INTERFACE-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.

## Annex J — Reference Implementation Topology

The reference implementation topology described in this annex is
non-normative; it documents the deployment shape that the WIA
Standards working group used to validate the test vectors in Annex G
and is intended as a starting point, not a recommendation against
alternative topologies.

- **Single-tenant edge** — one runtime per organization, no shared
  state. Used for early-pilot deployments where conformance evidence
  is published manually. Sufficient for PHASE-2-API-INTERFACE validation when the
  organization signs the manifest itself.
- **Multi-tenant gateway** — one shared runtime serves multiple
  tenants via header-based isolation. Typically backed by a
  rate-limited gateway (Envoy or NGINX) and a shared OAuth 2.1
  identity provider. The manifest is per-tenant; the runtime
  publishes a federation manifest that aggregates tenant manifests.
- **Federated mesh** — multiple runtimes peer to one another and
  publish their manifests to a directory service. Each peer signs
  its own manifest; the directory service signs the aggregated
  index. This is the topology used by cross-organization deployments
  that need to compose conformance.
- **Air-gapped batch** — no network connection between the runtime
  and the directory service. The runtime emits a signed evidence
  package on each batch and the operator transports the package via
  out-of-band channels. This is the topology used by regulators that
  prohibit live connectivity from sensitive environments.

Implementations declare their topology in the manifest (see Annex I).
A topology change MUST be reflected in a new manifest signature; the
prior topology's manifest remains valid for the deprecation window
described in Annex H to preserve audit traceability.
