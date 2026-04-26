# WIA-AUG-013 PHASE 4 — Integration Specification

**Standard:** WIA-AUG-013
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 4 of 4)

---

## 9. Implementation Guidelines

### 9.1 Certification Requirements

To achieve WIA-AUG-013 certification:

```
1. Safety Classification (Section 2)
   - Complete classification assessment
   - Submit documentation

2. Risk Assessment (Section 3)
   - Complete FMEA
   - RPN below threshold

3. Biocompatibility (Section 4)
   - Pass all required ISO 10993 tests
   - Material traceability

4. Pre-Clinical Testing (Section 6)
   - Complete bench testing
   - Animal studies (if required)

5. Clinical Testing (Section 6)
   - Phase I/II/III trials
   - Statistical significance

6. Safety Systems (Section 7)
   - Safe mode implementation
   - Emergency protocols

7. Monitoring System (Section 8)
   - Real-time monitoring capability
   - Reporting infrastructure
```

### 9.2 Documentation Requirements

```
Required Documents:
□ Device Master File
□ Risk Assessment Report
□ Biocompatibility Test Reports
□ Clinical Trial Data
□ Manufacturing Process Validation
□ Sterilization Validation
□ Software Validation (if applicable)
□ Labeling and Instructions for Use
□ Training Materials
□ Post-Market Surveillance Plan
```

### 9.3 API Interface

```typescript
interface SafetyAssessment {
  deviceId: string;
  classification: SafetyLevel;
  rpn: number;
  biocompatibility: BiocompatibilityResult;
  testResults: TestResult[];
  certificationStatus: 'pending' | 'approved' | 'rejected';
  validUntil: Date;
}

interface AssessmentRequest {
  device: DeviceInfo;
  manufacturer: ManufacturerInfo;
  testData: TestData[];
  documentation: Document[];
}

function assessSafety(request: AssessmentRequest): SafetyAssessment;
function validateCompliance(deviceId: string): ComplianceResult;
function generateReport(deviceId: string, format: 'pdf' | 'json'): Report;
```

---

## 10. References

### 10.1 International Standards

1. ISO 10993 - Biological evaluation of medical devices
2. ISO 14971 - Medical devices — Application of risk management
3. IEC 60601-1 - Medical electrical equipment — General requirements
4. IEC 62304 - Medical device software — Software life cycle processes
5. ISO 13485 - Medical devices — Quality management systems

### 10.2 Regulatory Guidelines

- FDA Guidance for Industry: Cybersecurity in Medical Devices
- EU MDR 2017/745 - Medical Device Regulation
- IMDRF Guidelines for Software as a Medical Device

### 10.3 WIA Standards

- WIA-AUG-001: Human Augmentation General
- WIA-AUG-014: Human-Machine Interface
- WIA-BCI: Brain-Computer Interface
- WIA-SEC: Security Standards
- WIA-MED: Medical Device Standards

---

## Appendix A: Risk Assessment Worksheet

```
Device: _______________
Date: _______________
Assessor: _______________

| ID | Component | Failure Mode | Cause | Effect | S | O | D | RPN | Mitigation |
|----|-----------|--------------|-------|--------|---|---|---|-----|------------|
| 1  |           |              |       |        |   |   |   |     |            |
| 2  |           |              |       |        |   |   |   |     |            |
...

Total RPN: ___
Maximum Single RPN: ___
Assessment Result: □ Pass □ Fail □ Conditional
```

## Appendix B: Biocompatibility Test Checklist

```
Device Classification: Level ___
Contact Duration: ___

Required Tests:
□ Cytotoxicity (ISO 10993-5)
□ Sensitization (ISO 10993-10)
□ Irritation (ISO 10993-10)
□ Systemic Toxicity (ISO 10993-11)
□ Genotoxicity (ISO 10993-3)
□ Implantation (ISO 10993-6)
□ Carcinogenicity (ISO 10993-3)
□ Reproductive Toxicity (ISO 10993-3)

Results Summary:
Test                  | Result    | Date       | Lab
---------------------|-----------|------------|-----
                     |           |            |
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-AUG-013 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
*MIT License*


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

## Annex J — Reference Implementation Topology

The reference implementation topology described in this annex is
non-normative; it documents the deployment shape that the WIA
Standards working group used to validate the test vectors in Annex G
and is intended as a starting point, not a recommendation against
alternative topologies.

- **Single-tenant edge** — one runtime per organization, no shared
  state. Used for early-pilot deployments where conformance evidence
  is published manually. Sufficient for PHASE-4-INTEGRATION validation when the
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
