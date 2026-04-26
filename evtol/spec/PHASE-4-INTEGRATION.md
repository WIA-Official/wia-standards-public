# WIA-SPACE-019 PHASE 4 — Integration

**Standard:** WIA-SPACE-019 eVTOL
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable
**Source:** consolidated from `evtol-v1.0.md` §8 (Certification
Framework) and `WIA-SPACE-019-v1.0.md` §7 (Safety Systems) + §8
(Airworthiness Certification)

This document defines the integration concerns: regulatory crosswalks
(EASA SC-VTOL, FAA part 21, KCAA), evidence packaging via CycloneDX
1.5, and signed-attestation submission to the type-certificate
authority. Includes the mapping from this PHASE's object types to the
evidence categories required by certification authorities.

References:
- ARP-4754A (development of civil aircraft and systems)
- ARP-4761A (safety assessment process)
- DO-178C (software considerations)
- DO-254 (hardware considerations)
- CycloneDX 1.5 (SBOM format), Sigstore (DSSE + Rekor)

---

## 8. Certification Framework

### 8.1 Type Certification

Follow FAA Part 23 with Special Conditions for:
- Powered-lift category airworthiness
- Electric propulsion systems
- Distributed propulsion
- Fly-by-wire flight controls
- Transition flight regime

### 8.2 Production Certification

- AS9100 or equivalent quality management system
- Production flight testing: 100% of aircraft
- Supplier qualification and auditing
- Configuration management
- Non-conformance tracking and resolution

### 8.3 Operational Certification

- Part 135 Air Carrier certificate (US)
- Equivalent EASA AOC (Europe)
- Pilot training programs and type ratings
- Maintenance programs and intervals
- Safety Management System (SMS)

---


## 7. Safety Systems

### 7.1 Redundancy

#### 7.1.1 Propulsion Redundancy
- **Minimum:** N+1 motor failure tolerance
- **Recommended:** N+2 simultaneous motor failures
- **Best Practice:** N/2 configuration (50% motors can fail)

#### 7.1.2 Battery Redundancy
- **Multiple Modules:** 4-12 independent battery modules
- **Isolation:** Electrical and thermal isolation between modules
- **Graceful Degradation:** Continue flight with reduced modules

#### 7.1.3 Flight Control Redundancy
- **Dual/Triple Computers:** 2-3 independent FCC
- **Sensor Redundancy:** All critical sensors duplicated
- **Dissimilar Redundancy:** Different hardware/software architecture

### 7.2 Emergency Systems

#### 7.2.1 Emergency Landing
- **Automatic Site Selection:** AI-based safe landing location identification
- **Controlled Descent:** Optimized thrust distribution with degraded systems
- **Priority Locations:** Vertiports > Airports > Open spaces > Roads > Water

#### 7.2.2 Ballistic Recovery System (BRS)
- **Deployment Altitude:** Minimum 100-150m AGL
- **Descent Rate:** 6-8 m/s with full parachute
- **Parachute Size:** 2-3x aircraft weight capacity
- **Activation:** Manual (pilot) or automatic (system)

### 7.3 Fire Safety

#### 7.3.1 Prevention
- **Thermal Barriers:** Ceramic/aerogel cell isolation
- **Temperature Monitoring:** Real-time cell-level monitoring
- **Current Limiting:** Immediate overcurrent shutoff
- **Pressure Relief:** Safe gas venting when overheated

#### 7.3.2 Suppression
- **Inert Gas:** N₂ or CO₂ injection system
- **Coolant Spray:** Liquid cooling for thermal runaway cells
- **Module Isolation:** Disconnect and isolate affected module
- **Exhaust System:** Toxic gas venting outside aircraft


## 8. Airworthiness Certification

### 8.1 Certification Authorities

#### 8.1.1 FAA (United States)
- **Category:** Powered Lift
- **Basis:** Part 23 + Special Conditions
- **Process:** 5-stage certification (Basis → Compliance → Verification → Approval → TC)
- **Timeline:** 5-7 years average

#### 8.1.2 EASA (Europe)
- **Standard:** SC-VTOL (Special Condition for VTOL)
- **Categories:** Basic (<2 pax) and Enhanced (commercial)
- **Process:** Similar to FAA 5-stage
- **Timeline:** 5-7 years average

#### 8.1.3 CAAC (China)
- **Fast-Track:** Streamlined for domestic industry
- **First Certified:** EHang EH216 (2021)
- **Timeline:** 3-5 years

### 8.2 Testing Requirements

#### 8.2.1 Ground Testing
- **Static Structural:** Ultimate load testing (1.5x limit load)
- **Fatigue:** Lifecycle structural testing (10,000+ cycles)
- **Battery Safety:** Overcharge, nail penetration, crush, fire tests
- **EMC:** Lightning strike, radio interference testing
- **Noise:** Sound level measurement at 100m

#### 8.2.2 Flight Testing
- **Performance:** Speed, climb, range, ceiling verification
- **Handling:** Control response, stability characteristics
- **Transition:** Vertical-to-horizontal mode change (tilt-rotor)
- **Failure Modes:** Motor/battery failure simulation
- **Environmental:** Temperature, wind, precipitation limits

#### 8.2.3 Safety Analysis
- **FHA:** Functional Hazard Assessment
- **FTA:** Fault Tree Analysis
- **FMEA:** Failure Modes and Effects Analysis
- **SSA:** System Safety Assessment
- **Target:** 10⁻⁷ fatal accidents per flight hour

### 8.3 Noise Standards

#### 8.3.1 Requirements
- **Takeoff/Landing:** Maximum 65 dB @ 100m
- **Cruise:** Maximum 60 dB @ 100m
- **Ground Operations:** Maximum 70 dB @ vertiport boundary
- **Night Operations:** Additional 5-10 dB reduction


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
