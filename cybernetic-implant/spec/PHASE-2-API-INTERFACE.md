# WIA-AUG-002 PHASE 2 — API Interface Specification

**Standard:** WIA-AUG-002
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 2 of 4)

---

## 5. Communication Protocols

### 5.1 Internal Communication

#### 5.1.1 Neural Interface Protocol

```
Signal Type: Bioelectrical potentials
Frequency Range: 0.1 Hz - 10 kHz
Sampling Rate: 1-50 kHz
Channels: 1-256 (array-dependent)
Impedance: < 1 MΩ @ 1 kHz
Encoding: Spike detection, frequency modulation
Processing: Real-time digital signal processing
```

#### 5.1.2 Implant-to-Implant Communication

```
Protocol: Medical Body Area Network (MBAN)
Frequency: 2.4 GHz ISM band
Range: 0-2 meters (in-body)
Data Rate: 1 Mbps
Topology: Mesh or star
Security: AES-256 encryption
Power: Ultra-low (< 10 mW)
```

### 5.2 External Communication

#### 5.2.1 Bluetooth Low Energy (BLE)

```
Version: BLE 5.0+
Frequency: 2.4 GHz
Range: 0-10 meters
Data Rate: 125 kbps - 2 Mbps
Power: < 15 mW
Use Cases: Mobile apps, configuration, data sync
Security: Pairing, bonding, encryption
```

#### 5.2.2 Medical Implant Communication Service (MICS)

```
Frequency: 402-405 MHz
Range: 0-2 meters
Data Rate: 200-800 kbps
Power: < 25 μW
Use Cases: Clinical monitoring, diagnostics
Regulation: FCC Part 95 (USA), ETSI EN 301 839 (EU)
```

#### 5.2.3 Near-Field Communication (NFC)

```
Frequency: 13.56 MHz
Range: 0-10 cm
Data Rate: 106-424 kbps
Power: Passive (reader-powered)
Use Cases: ID, configuration, emergency access
Standard: ISO/IEC 14443, 15693
```

### 5.3 Communication Security

```
Requirements:
  - End-to-end encryption (AES-256)
  - Mutual authentication
  - Secure key exchange
  - Anti-replay protection
  - Access control (role-based)
  - Audit logging

Privacy:
  - Data anonymization
  - Patient consent management
  - HIPAA/GDPR compliance
  - Secure data storage
  - Right to erasure
```

---

## 6. Surgical Integration

### 6.1 Pre-Operative Assessment

#### 6.1.1 Patient Screening

```
Medical History:
  - Previous surgeries
  - Allergies and sensitivities
  - Medications
  - Immune status
  - Comorbidities

Physical Examination:
  - Implant site assessment
  - Tissue quality
  - Vascular mapping
  - Neural mapping (if applicable)

Diagnostic Tests:
  - Blood work (CBC, metabolic panel)
  - Imaging (CT, MRI, ultrasound)
  - Biocompatibility patch test
  - Psychological evaluation (Level 4-5)
```

#### 6.1.2 Implant Site Selection

```
Criteria:
  - Anatomical suitability
  - Tissue thickness and quality
  - Proximity to nerves/vessels
  - Mobility considerations
  - Patient lifestyle
  - Aesthetic considerations

Planning:
  - 3D modeling and simulation
  - Surgical approach planning
  - Backup site identification
  - Risk assessment
```

### 6.2 Intra-Operative Procedure

#### 6.2.1 Standard Surgical Protocol

```
Phase 1: Preparation
  - Anesthesia (local/regional/general)
  - Sterile field preparation
  - Patient positioning
  - Equipment verification

Phase 2: Implantation
  - Incision and tissue exposure
  - Pocket creation or bone preparation
  - Implant positioning and fixation
  - Neural connection (if applicable)
  - Vascular considerations

Phase 3: Activation
  - Power system activation
  - Communication link establishment
  - Functional testing
  - Calibration and tuning

Phase 4: Closure
  - Hemostasis verification
  - Layered closure
  - Dressing application
  - Post-op imaging
```

#### 6.2.2 Neural Interface Integration

```
Procedure:
  1. Nerve identification and exposure
  2. Epineurial window creation
  3. Electrode array insertion
  4. Fixation and anchoring
  5. Electrical testing
  6. Signal quality verification
  7. Threshold mapping
  8. Protective wrapping

Monitoring:
  - Impedance measurement
  - Signal-to-noise ratio
  - Stimulation thresholds
  - Evoked potentials
  - No adverse neural response
```

### 6.3 Post-Operative Care

#### 6.3.1 Acute Phase (0-2 weeks)

```
Monitoring:
  - Wound healing
  - Pain management
  - Infection surveillance
  - Device function
  - Early rejection signs

Actions:
  - Dressing changes
  - Antibiotic prophylaxis
  - Pain medication
  - Limited activity
  - Daily device checks
```

#### 6.3.2 Integration Phase (2 weeks - 6 months)

```
Monitoring:
  - Tissue integration progress
  - Impedance changes
  - Functional improvement
  - Rejection markers
  - Patient adaptation

Actions:
  - Progressive rehabilitation
  - Calibration adjustments
  - Training sessions
  - Psychological support
  - Bi-weekly clinical visits
```

#### 6.3.3 Maintenance Phase (6+ months)

```
Monitoring:
  - Long-term stability
  - Performance metrics
  - Wear and aging
  - Patient satisfaction
  - Lifestyle integration

Actions:
  - Monthly/quarterly checkups
  - Software updates
  - Optimization
  - Preventive maintenance
  - Continuous education
```

---

## 7. Rejection Monitoring

### 7.1 Rejection Mechanisms

```
Types of Rejection:
  1. Acute Rejection (0-3 months)
     - Rapid immune response
     - Inflammation, pain, swelling
     - High severity

  2. Chronic Rejection (3+ months)
     - Slow tissue degradation
     - Encapsulation, fibrosis
     - Progressive dysfunction

  3. Infection-Related
     - Bacterial biofilm
     - Systemic infection
     - Secondary rejection
```

### 7.2 Biomarker Monitoring

#### 7.2.1 Inflammatory Markers

```
C-Reactive Protein (CRP):
  - Normal: < 3 mg/L
  - Warning: 3-10 mg/L
  - Critical: > 10 mg/L
  - Measurement: Blood test

Interleukin-6 (IL-6):
  - Normal: < 5 pg/mL
  - Warning: 5-20 pg/mL
  - Critical: > 20 pg/mL
  - Measurement: Blood test

Local Temperature:
  - Normal: 36-37°C
  - Warning: 37-38°C
  - Critical: > 38°C
  - Measurement: Implant sensor
```

#### 7.2.2 Immune Response Markers

```
Antibody Levels:
  - IgG, IgM against implant materials
  - Normal: < 100 AU/mL
  - Warning: 100-500 AU/mL
  - Critical: > 500 AU/mL

T-Cell Activation:
  - CD4+, CD8+ counts
  - Normal: Baseline ± 20%
  - Warning: Baseline ± 20-50%
  - Critical: > 50% deviation

Complement Activation:


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
