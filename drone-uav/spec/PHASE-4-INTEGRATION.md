# WIA-SPACE-017 PHASE 4 — Integration

**Standard:** WIA-SPACE-017 Drone / UAV
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable
**Source:** consolidated from `drone-uav-v1.0.md` §5 (Regulatory
Compliance) + §11 (Compliance and Certification) and
`WIA-SPACE-017-v1.0.md` §9 (Safety Requirements)

This document defines the integration concerns: regulatory crosswalks
(EASA, FAA, KCAA), safety case assembly per ARP-4754A, evidence
packaging via CycloneDX 1.5, and signed-attestation submission to a
type-certificate authority.

References:
- ARP-4754A (development of civil aircraft and systems)
- ARP-4761A (safety assessment process)
- DO-178C (software considerations)
- DO-254 (hardware considerations)
- ISO/IEC 27001:2022 (information security)

---

## 5. Regulatory Compliance

### 5.1 United States (FAA)

**Part 107 Requirements:**
- Remote pilot certificate required for commercial operations
- Aircraft registration for drones > 250g
- VLOS, 400ft AGL, 100mph speed limits
- Daylight operations with civil twilight extensions
- LAANC authorization for controlled airspace
- Remote ID compliance mandatory (with limited exceptions)

### 5.2 European Union (EASA)

**Open Category:**
- A1: <250g or C0 class, over uninvolved people permitted
- A2: <2kg or C2 class, 30m horizontal distance from people
- A3: <25kg or C3 class, 150m from residential/commercial/industrial areas

**Specific Category:**
- Operational authorization required
- SORA (Specific Operations Risk Assessment) methodology
- Standard Scenarios (STS) for common operations

**Certified Category:**
- High-risk operations (passenger transport, dangerous goods, large crowds)
- Aircraft certification required
- Licensed remote pilots

### 5.3 International Standards

**ICAO Framework:**
- Remotely Piloted Aircraft Systems (RPAS) guidance
- Detect and Avoid requirements
- Communication, Navigation, Surveillance standards
- Licensing and certification framework

---


## 9. Safety Requirements

### 9.1 Pre-Flight Safety

**Mandatory Checks**:
1. Visual inspection of frame, propellers, motors
2. Battery voltage and capacity verification
3. Control surface and motor response test
4. GPS satellite lock (for GPS-dependent modes)
5. Compass calibration verification
6. Failsafe function test
7. Geofence activation (if applicable)

### 9.2 In-Flight Safety

**Failsafe Behaviors** (configurable priority):
1. **RC Signal Loss**:
   - Option A: Return to Home (RTH)
   - Option B: Land immediately
   - Option C: Hover (GPS required)

2. **Low Battery**:
   - Warning threshold: 30% remaining (configurable)
   - Critical threshold: 15% remaining
   - Action: Automatic RTH or land

3. **GPS Loss** (in GPS mode):
   - Transition to ATTI (attitude) mode
   - Alert operator
   - Hover or descend

### 9.3 Geofencing

**Requirements**:
- Maximum altitude limit (default: 120m AGL, configurable)
- Maximum distance limit (default: 500m, configurable)
- No-fly zone database integration (optional)
- Automatic action on breach: Stop, RTH, or hover

### 9.4 Collision Avoidance

**For drones > 2kg** (recommended):
- Forward obstacle detection (minimum range: 15m)
- Automatic braking or avoidance maneuver
- Visual and/or audible warning to operator

---


## 11. Compliance and Certification

### 11.1 Manufacturer Requirements

**Documentation:**
- Technical specifications and performance data
- Maintenance procedures and schedules
- Safety analysis and risk assessment
- User manuals and training materials

**Testing:**
- Flight performance validation
- Failsafe mechanism verification
- Endurance and reliability testing
- Environmental testing (temperature, humidity, vibration)

### 11.2 Operator Requirements

**Training:**
- Aeronautical knowledge
- Platform-specific operational procedures
- Emergency procedures
- Regulatory compliance

**Certification:**
- Appropriate license for operations (Part 107, EASA, etc.)
- Platform-specific training completion
- Recurrent training requirements

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
