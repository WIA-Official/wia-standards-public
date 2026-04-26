# WIA-SPACE-019 PHASE 1 — Data Format Specification

**Standard:** WIA-SPACE-019 eVTOL
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable
**Source:** consolidated from `evtol-v1.0.md` §3 (Aircraft Configurations)
and `WIA-SPACE-019-v1.0.md` §4 (eVTOL Design Configurations)

This document defines the canonical data structures exchanged across
WIA-SPACE-019 implementations: aircraft type designations, design
envelopes, mission profiles, and energy-budget descriptors. Schemas are
expressed in JSON Schema 2020-12 and are stable for the lifetime of
this PHASE.

References:
- ICAO Annex 8 (airworthiness)
- EASA SC-VTOL (special condition for VTOL aircraft)
- ASTM F3338-21 (electric propulsion units)
- OpenAPI 3.1, JSON Schema 2020-12

---

## 3. Aircraft Configurations

### 3.1 Configuration Types

#### 3.1.1 Multicopter
- **Definition:** Multiple fixed rotors providing lift and control
- **Example:** Volocopter 2X, EHang 216
- **Characteristics:**
  - Simple mechanical design
  - High redundancy through distributed propulsion
  - Limited cruise efficiency
  - Typical range: 15-30 miles

#### 3.1.2 Lift+Cruise
- **Definition:** Separate propulsion for vertical lift and horizontal cruise
- **Example:** Joby S4, Archer Midnight
- **Characteristics:**
  - Moderate mechanical complexity
  - Good cruise efficiency
  - Transition phase requires sophisticated control
  - Typical range: 60-150 miles

#### 3.1.3 Tilt-Wing
- **Definition:** Entire wing structure tilts from vertical to horizontal
- **Example:** Lilium Jet
- **Characteristics:**
  - Complex mechanical and control systems
  - Excellent cruise efficiency
  - Challenging transition dynamics
  - Typical range: 100-200 miles

#### 3.1.4 Tilt-Rotor
- **Definition:** Rotors tilt from vertical to horizontal configuration
- **Example:** Wisk Cora
- **Characteristics:**
  - Proven concept from V-22 Osprey
  - High cruise efficiency
  - Mechanical complexity
  - Typical range: 150-300 miles

### 3.2 Configuration Selection Criteria

Selection based on mission requirements:
- **Urban Air Mobility (short trips):** Multicopter or Lift+Cruise
- **Regional Connectivity:** Lift+Cruise or Tilt-Wing
- **Long-Range Operations:** Tilt-Wing or Tilt-Rotor
- **Autonomous Operations:** Any configuration with appropriate sensors and controls

---


## 4. eVTOL Design Configurations

### 4.1 Multicopter

#### 4.1.1 Configuration
- **Rotor Count:** 4-36 fixed rotors
- **Control:** Individual rotor RPM adjustment
- **Advantages:** Simple design, excellent hover, high redundancy
- **Limitations:** Lower cruise speed (80-130 km/h), shorter range (30-50 km)
- **Applications:** Short urban trips, tourism, emergency medical

#### 4.1.2 Performance Requirements
- Minimum hover efficiency: 5-7 kg/kW
- Disk loading: 50-150 N/m²
- Noise: <65 dB @ 100m during takeoff/landing

### 4.2 Tilt-Rotor

#### 4.2.1 Configuration
- **Rotor Count:** 2-8 tilting rotors with wings
- **Transition:** Gradual tilt from vertical to horizontal (15-45 seconds)
- **Advantages:** High cruise speed (200-350 km/h), long range (150-300 km)
- **Limitations:** Complex mechanics, difficult transition
- **Applications:** Airport shuttles, intercity routes, premium air taxi

#### 4.2.2 Performance Requirements
- Cruise speed: Minimum 200 km/h
- Transition altitude: 50-150m AGL
- Wing loading: 150-300 kg/m²

### 4.3 Lift+Cruise

#### 4.3.1 Configuration
- **Lift System:** 8-16 vertical rotors
- **Cruise System:** 2-6 forward propellers
- **Advantages:** Optimized for each flight phase
- **Limitations:** Additional weight, complexity
- **Applications:** Medium-range urban aviation, cargo

#### 4.3.2 Performance Requirements
- Combined efficiency: >80% in cruise
- Power distribution flexibility: <3 seconds mode transition
- Cruise speed: 180-280 km/h


## Annex E — Implementation Notes for PHASE-1-DATA-FORMAT

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-1-DATA-FORMAT.

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
evidence for PHASE-1-DATA-FORMAT. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-1-data-format/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-1-DATA-FORMAT with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-1-DATA-FORMAT does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-1-DATA-FORMAT.
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
for PHASE-1-DATA-FORMAT. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P1-DATA-FORMAT-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.
