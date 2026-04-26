# WIA-SPACE-027 PHASE 4 — Integration

**Standard:** WIA-SPACE-027 Asteroid Mining
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable
**Source:** consolidated from `WIA-SPACE-027-v1.0.md` §8 (Legal and
Economic Framework), §9 (Quality and Certification), §10 (Safety and
Risk Management)

This document defines integration concerns: regulatory crosswalks
(Outer Space Treaty, Moon Agreement, national space-mining acts),
evidence packaging via CycloneDX 1.5, and signed-attestation
submission to a competent authority. The mapping from WIA object
types to commercial-space regulatory categories is defined in
Annex G's evidence-package convention.

References:
- COSPAR Planetary Protection Policy
- ISO/IEC 27001:2022 (information security)
- CycloneDX 1.5 (SBOM format), Sigstore (DSSE + Rekor)

---

## 8. Legal and Economic Framework

### 8.1 Ownership and Rights

Mining operators SHALL:

1. **Register:** Submit mining plan to International Asteroid Mining Bureau (IAMB) 90 days prior
2. **Report:** Real-time orbital data transmission (hourly updates)
3. **Transparency:** Publicly disclose asteroid target, timeline, resource estimates
4. **Revenue Sharing:** Contribute 5-10% of gross revenue to International Space Fund

### 8.2 Protected Asteroids

The following asteroids are PROHIBITED from mining:

- **Category A:** Asteroids with potential biosignatures (complete protection)
- **Category B:** Scientifically unique asteroids (limited extraction for research only)
- **Category C:** Sample return mission targets (10-year moratorium post-mission)

### 8.3 Environmental Protection

Mining operations SHALL:

- **Debris Management:** Track and mitigate debris > 1 cm
- **Orbit Stability:** Monitor asteroid rotation and structural integrity
- **Collision Avoidance:** Maintain > 10 km clearance from other spacecraft
- **Reporting:** Submit environmental impact assessment within 3 months post-mining

### 8.4 Insurance Requirements

Operators SHALL obtain:

- **Launch Insurance:** 5-10% of spacecraft value
- **Liability Insurance:** Minimum $1B third-party coverage
- **Market Risk Insurance:** Optional price protection (3-7% premium)


## 9. Quality and Certification

### 9.1 "Space-Refined" Certification

Metals meeting WIA-SPACE-027 standards SHALL be certified:

| Grade | Purity | Price Premium | Applications |
|-------|--------|---------------|--------------|
| Space-Grade Platinum | 99.99% | +20% | Electronics, catalysis |
| Space-Grade Iron | 99.5% | +10% | Structural steel |
| Space-Grade Nickel | 99.8% | +15% | Alloys, batteries |

### 9.2 Testing and Validation

All refined materials SHALL undergo:

- **XRF Analysis:** Elemental composition (accuracy ±0.1%)
- **Mass Spectrometry:** Isotope ratios and purity
- **Mechanical Testing:** Tensile strength, hardness (ASTM standards)
- **Traceability:** Blockchain-based provenance tracking


## 10. Safety and Risk Management

### 10.1 Mission Phases and Risk

| Phase | Duration | Primary Risk | Mitigation |
|-------|----------|--------------|------------|
| Launch | 1 day | Vehicle failure | Redundant systems, insurance |
| Transit | 6-24 months | Navigation error | AI autonomous navigation |
| Rendezvous | 1-3 months | Collision | LIDAR, optical nav, abort capability |
| Mining | 3-12 months | Equipment failure | Robotic swarms (redundancy) |
| Return | 6-18 months | Reentry failure | Heat shield testing, landing zone |

### 10.2 Emergency Protocols

Operators SHALL maintain:

- **Abort Capability:** Return to safe orbit within 24 hours
- **Communication:** Redundant Earth contact (X-band + Ka-band)
- **Autonomous Operation:** 30-day self-sufficiency without ground control
- **Data Backup:** Continuous telemetry to Earth stations


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
