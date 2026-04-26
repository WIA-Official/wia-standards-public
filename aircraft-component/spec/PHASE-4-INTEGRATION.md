# WIA-SPACE-016 PHASE 4 — Integration

**Standard:** WIA-SPACE-016 Aircraft Component
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable
**Source:** consolidated from `WIA-SPACE-016-v1.0.md` §4 (Landing
Gear Standards), §8 (Certification Standards), §9 (Maintenance and
Lifecycle Standards)

This document defines integration concerns: continued-airworthiness
evidence packaging via CycloneDX 1.5, signed-attestation submission to
the type-certificate authority via in-toto Attestation 1.0, and the
mapping of this PHASE's object types to AS9100D and AS9145 quality
management expectations.

References:
- AS9100D, AS9145 (quality management for aviation)
- CycloneDX 1.5 (SBOM format), Sigstore (DSSE + Rekor)
- in-toto Attestation Framework 1.0

---

## 4. Landing Gear Standards

### 4.1 Design Requirements

#### 4.1.1 Load Conditions
- **Static Load:** Maximum taxi weight
- **Dynamic Load:** 2.5-3.0 × static load (landing impact)
- **Braking Load:** 1.0g deceleration
- **Side Load:** 0.3g lateral (crosswind)
- **Spin-up Load:** Tire rotation acceleration

#### 4.1.2 Shock Absorber
```
Oleo-Pneumatic Strut:
- Type: Gas-oil shock absorber
- Gas: Nitrogen (N₂)
- Hydraulic Fluid: MIL-PRF-5606
- Static Pressure: 2,500 psi (172 bar)
- Maximum Pressure: 4,500 psi (310 bar)
- Stroke: 300-500 mm
- Energy Absorption: 80-90%
```

### 4.2 Tire Requirements

```
Tire Specifications (Example: B737):
- Size: 40×14
- Ply Rating: 20-26 plies
- Inflation Pressure: 200 psi (13.8 bar)
- Speed Rating: 160 knots (296 km/h)
- Load Rating: 25,000 lbs (11,340 kg) per tire
- Tread Depth (New): 16-18 mm
- Minimum Tread Depth: 1.6 mm
- Service Life: 200-300 landings
```

### 4.3 Braking System

#### 4.3.1 Carbon Brake Performance
```
Material: Carbon-Carbon Composite
- Working Temperature: 1,000-1,500°C
- Friction Coefficient: 0.3-0.5 (temperature dependent)
- Mass: 40% lighter than steel brakes
- Service Life: 2,500-3,000 landings
- Kinetic Energy Capacity: 100-150 MJ per brake

Anti-Skid System:
- Response Time: < 50 ms
- Slip Ratio Control: 10-20% optimal slip
- Wheel Speed Sampling: 100 Hz
- Independent Control: Each wheel individually controlled
```

---


## 8. Certification Standards

### 8.1 AS9100 Quality Management

#### 8.1.1 Requirements
- **Quality Management System:** ISO 9001 + aerospace-specific requirements
- **Configuration Management:** Control of design changes and versions
- **Risk Management:** Identification and mitigation of risks
- **Supplier Management:** Approved supplier list and audits
- **Continuous Improvement:** Corrective and preventive actions (CAPA)

#### 8.1.2 Documentation
- **Quality Manual:** Top-level quality policy and objectives
- **Procedures:** Work instructions and process documentation
- **Records:** Traceability of materials, processes, inspections
- **Audits:** Internal audits (annually), external audits (certification body)

### 8.2 NADCAP Certification

#### 8.2.1 Special Processes
```
Covered Processes:
- Welding: Resistance, TIG, MIG, electron beam, laser
- Heat Treatment: Annealing, hardening, tempering, stress relief
- Non-Destructive Testing: X-ray, ultrasonic, eddy current, penetrant
- Chemical Processing: Anodizing, plating, chemical milling
- Composite Materials: Lay-up, curing, bonding, repair

Audit Frequency:
- Initial Audit: Before certification
- Surveillance Audit: Every 6-12 months (process dependent)
- Re-certification: Every 2-3 years
```

### 8.3 FAA/EASA Type Certification

#### 8.3.1 Certification Basis
- **FAR Part 25:** Transport Category Airplanes
- **CS-25:** EASA Certification Specifications (equivalent to FAR 25)
- **Advisory Circulars:** Acceptable means of compliance
- **Special Conditions:** For novel or unusual design features

#### 8.3.2 Compliance Methods
- **Analysis:** Engineering calculations and simulations
- **Testing:** Ground tests, flight tests, certification tests
- **Similarity:** Reference to previously approved designs
- **Service Experience:** Operational data from similar aircraft

---


## 9. Maintenance and Lifecycle Standards

### 9.1 Maintenance Program

#### 9.1.1 Check Intervals
```
A Check:
- Interval: 500-800 flight hours or 200-300 flights
- Duration: 10-20 hours
- Location: Line maintenance (airport)
- Scope: Visual inspections, lubrication, minor repairs

C Check:
- Interval: 18-24 months or 3,000-4,500 flight hours
- Duration: 1-2 weeks
- Location: Hangar maintenance
- Scope: Detailed inspections, NDT, system tests, component replacement

D Check:
- Interval: 6-10 years or 30,000-60,000 flight hours
- Duration: 1-2 months
- Location: Major maintenance facility
- Scope: Complete structural inspection, overhaul, modifications
```

### 9.2 Non-Destructive Inspection (NDI)

#### 9.2.1 Inspection Methods
```
Eddy Current Inspection:
- Application: Surface cracks in aluminum, titanium
- Depth: 0-5 mm below surface
- Crack Detection: > 0.5 mm length
- Frequency: 10 kHz - 10 MHz

Ultrasonic Inspection:
- Application: Internal defects, corrosion, thickness
- Depth: 0-500 mm (depends on material)
- Resolution: 0.5 mm
- Method: Pulse-echo, through-transmission

Radiographic Inspection:
- Application: Internal voids, inclusions, cracks
- Source: X-ray (portable) or gamma-ray (Co-60, Ir-192)
- Sensitivity: 1-2% of material thickness
- Recording: Film or digital detector
```

### 9.3 Fatigue Life Management

#### 9.3.1 Damage Tolerance
```
Inspection Program:
- Initial Inspection Threshold: 50% of calculated life
- Repeat Interval: Every 10,000 flights or as calculated
- Crack Growth Monitoring: Track crack size over time
- Retirement Life: Before crack reaches critical size

Structural Monitoring:
- Load Monitoring: Record flight loads (FOQA data)
- Usage Monitoring: Cycles, hours, exceedances
- Structural Health Monitoring: Sensors on critical structure
- Fleet Leader: Most severely used aircraft inspected first
```

---


## Annex E — Implementation Notes for PHASE-4-INTEGRATION

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-4-INTEGRATION.

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
evidence for PHASE-4-INTEGRATION. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-4-integration/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-4-INTEGRATION with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-4-INTEGRATION does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-4-INTEGRATION.
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
for PHASE-4-INTEGRATION. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P4-INTEGRATION-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.
