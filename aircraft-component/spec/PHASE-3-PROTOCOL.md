# WIA-SPACE-016 PHASE 3 — Protocol

**Standard:** WIA-SPACE-016 Aircraft Component
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable
**Source:** consolidated from `WIA-SPACE-016-v1.0.md` §5 (Flight
Control Standards) and §6 (Hydraulic and Fuel System Standards)

This document defines the protocols between component supplier, MRO,
operator, and certification authority. References ARP-4754A and
DO-178C for the development assurance process; protocol schemas
reflect the message types defined in those documents but do not
redefine them.

References:
- ARP-4754A (development of civil aircraft and systems)
- ARP-4761A (safety assessment process)
- DO-178C (software considerations)
- DO-254 (hardware considerations)

---

## 5. Flight Control Standards

### 5.1 Primary Flight Controls

#### 5.1.1 Control Surface Sizing
```
Ailerons:
- Span: 15-25% of wing semi-span
- Chord: 20-30% of wing chord
- Deflection: ±25° typical
- Hinge Moment: 5,000-15,000 N⋅m

Elevators:
- Area: 15-20% of horizontal stabilizer area
- Deflection: +25°/-15° typical
- Hinge Moment: 10,000-30,000 N⋅m

Rudder:
- Area: 25-30% of vertical stabilizer area
- Deflection: ±25° typical
- Hinge Moment: 8,000-25,000 N⋅m
```

### 5.2 Fly-by-Wire System

#### 5.2.1 Architecture
- **Flight Control Computers:** Triple or quadruple redundancy
- **Data Bus:** ARINC 429 or ARINC 664 (AFDX)
- **Actuators:** Electro-hydraulic or electro-mechanical
- **Sensors:** Position, rate, acceleration (triple redundant)
- **Power:** Independent power sources for each channel

#### 5.2.2 Performance Requirements
```
System Response:
- Command to Surface Movement: < 100 ms
- Position Accuracy: ±0.5°
- Rate Capability: 50-100°/s
- Failure Detection: < 20 ms
- Fault Recovery: < 200 ms

Control Laws:
- Normal Law: Full envelope protection
- Alternate Law: Reduced protections
- Direct Law: Mechanical backup feel
```

---


## 6. Hydraulic and Fuel System Standards

### 6.1 Hydraulic System

#### 6.1.1 System Specifications
```
Operating Pressure: 3,000 psi (207 bar)
- Proof Pressure: 4,500 psi (310 bar)
- Burst Pressure: 9,000 psi (620 bar)

Hydraulic Fluid:
- Type: MIL-PRF-5606 or MIL-PRF-83282
- Operating Temp: -54°C to +135°C
- Viscosity: 13.5-15.5 cSt @ 40°C

System Redundancy:
- Primary: System A + System B
- Backup: System C (electric pump)
- Emergency: Ram Air Turbine (RAT)
```

#### 6.1.2 Pump Requirements
```
Engine-Driven Pump (EDP):
- Type: Variable displacement piston pump
- Flow Rate: 20-40 gpm (75-150 L/min)
- Efficiency: > 85%
- MTBF: > 10,000 flight hours

Electric Motor Pump (EMP):
- Type: Fixed displacement or variable
- Power: 7.5-15 kW
- Flow Rate: 10-20 gpm (38-75 L/min)
- Operating Time: Continuous or intermittent
```

### 6.2 Fuel System

#### 6.2.1 Fuel Tank Design
```
Tank Types:
- Integral Tanks: Sealed wing structure
- Bladder Tanks: Flexible fuel cells
- Rigid Tanks: Aluminum alloy containers

Fuel Capacity (Example: B737):
- Wing Tanks: 46,534 lbs (21,100 kg)
- Center Tank: 19,160 lbs (8,690 kg)
- Total Usable Fuel: 65,694 lbs (29,790 kg)

Ullage: 2-3% of tank volume (fuel expansion)
Vent System: Prevent overpressure and vacuum
```

#### 6.2.2 Fuel Management
```
Fuel Feed System:
- Boost Pumps: 2 per tank (redundancy)
- Suction Feed: Backup (gravity/suction)
- Flow Rate: 5,000-10,000 lbs/hr per engine

Center of Gravity Control:
- Forward CG Limit: 15% MAC
- Aft CG Limit: 30% MAC
- Fuel Sequencing: Maintain CG within limits

Fuel Jettison (if equipped):
- Jettison Rate: 2,000-4,000 lbs/min
- Jettison to: Maximum landing weight
- Safety Height: > 5,000 ft AGL
```

---


## Annex E — Implementation Notes for PHASE-3-PROTOCOL

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-3-PROTOCOL.

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
evidence for PHASE-3-PROTOCOL. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-3-protocol/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-3-PROTOCOL with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-3-PROTOCOL does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-3-PROTOCOL.
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
for PHASE-3-PROTOCOL. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P3-PROTOCOL-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.
