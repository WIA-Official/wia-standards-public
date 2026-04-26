# WIA-SPACE-015 PHASE 3 — Protocol

**Standard:** WIA-SPACE-015 Aerospace Engine
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable
**Source:** consolidated from `WIA-SPACE-015-v1.0.md` §6 (Testing and
Certification) and §7 (Performance and Environmental Requirements)

This document defines the protocols for test-cell data exchange,
certification-evidence aggregation, and emissions reporting between
engine OEM, MRO, and certification authority. It standardizes the
hand-off events and capability negotiation so that multi-vendor
maintenance workflows are reproducible and auditable.

References:
- ARP-4754A (development of civil aircraft and systems)
- ARP-4761A (safety assessment process)
- DO-178C (software considerations)
- DO-254 (hardware considerations)
- ICAO Annex 16 Vol. II (engine emissions reporting)

---

## 6. Testing and Certification

### 6.1 Ground Testing

#### 6.1.1 Calibration Test
Establish baseline engine performance:
- Thrust measurement (sea level & altitude simulation)
- SFC measurement
- Temperature and pressure profiles

#### 6.1.2 Endurance Test (FAR 33.87)
- Duration: 150 hours minimum
- Cycles: Take-off, climb, cruise, descent, idle
- Requirements: No unacceptable deterioration

#### 6.1.3 Bird Ingestion Test (FAR 33.77)
- Medium birds: 1.85~2.5 lbs (0.85~1.13 kg), 1+ birds
- Small birds: 0.05~0.25 lbs, multiple birds
- Requirements: Safe shutdown, no fire, no case penetration

#### 6.1.4 Fan Blade-Off Test (FAR 33.94)
- Condition: Maximum RPM, 1 blade failure
- Requirements: Complete containment, safe shutdown within 15s

### 6.2 Flight Testing

- Functional tests: Start, shutdown, relight
- Performance verification: Cruise, climb, descent
- Extreme maneuvers: Rapid climbs, descents, turns
- Environmental tests: Extreme temperature, humidity, altitude
- ETOPS validation (if applicable)

### 6.3 Certification

**Required certifications:**
- FAA Type Certificate (FAR Part 33)
- EASA Type Certificate (CS-E)
- Additional: CAAC, Transport Canada, ANAC (Brazil)

**ETOPS (Extended Operations):**
- ETOPS-120/180/207/240/330
- IFSD rate: < 0.02 per 1,000 flight hours
- Minimum fleet experience: 250,000 engine hours

---


## 7. Performance and Environmental Requirements

### 7.1 Performance Standards

**Modern turbofan engines SHALL meet:**
- SFC (cruise, ISA+10°C): ≤ 0.55 lb/lbf/hr
- Thrust-to-weight ratio: ≥ 5:1
- Reliability: IFSD rate < 0.01 per 1,000 hours

### 7.2 Emissions (ICAO CAEP)

**NOx emissions:**
- New engines (2020+): CAEP/10 or better
- Target: 75~85% reduction vs. CAEP/2 (1999 baseline)

**CO and UHC:**
- Minimize through complete combustion
- Lean-burn combustor technology recommended

**CO₂:**
- Reduce through SFC improvement
- SAF (Sustainable Aviation Fuel) compatibility required

### 7.3 Noise (ICAO Annex 16)

**Noise certification points:**
- Approach
- Lateral (sideline)
- Flyover

**Requirements:**
- Comply with ICAO Chapter 14 (2017+)
- Target: 20+ dB cumulative reduction vs. 1960s baseline

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
