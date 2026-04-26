# WIA-IND-030 PHASE 4 — Integration Specification

**Standard:** WIA-IND-030
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 4 of 4)

---

## 15. Certification and Compliance

### 15.1 Circular Economy Certifications

#### 15.1.1 Cradle to Cradle Certified

Levels: Basic, Bronze, Silver, Gold, Platinum

Categories:
- Material Health
- Material Reutilization
- Renewable Energy & Carbon Management
- Water Stewardship
- Social Fairness

#### 15.1.2 Ellen MacArthur Foundation CE100

Recognition for circular economy commitment and implementation.

#### 15.1.3 EU Ecolabel

Environmental excellence across product lifecycle.

#### 15.1.4 Zero Waste Certification

Verified > 90% waste diversion from landfill.

#### 15.1.5 B Corporation

Comprehensive environmental and social performance.

### 15.2 Compliance Requirements

#### 15.2.1 Data Collection

Organizations SHALL collect and maintain:
- Material passport records
- Lifecycle event logs
- Waste and recycling data
- EPR compliance data
- Certification documents
- Audit reports

#### 15.2.2 Documentation Retention

Records SHALL be retained for:
- Minimum 7 years for compliance
- 10 years for certification
- Permanently for blockchain records

#### 15.2.3 Third-Party Verification

Independent verification SHALL include:
- Annual audits
- Material testing
- Facility inspections
- Stakeholder interviews
- Data validation

### 15.3 Continuous Improvement

Organizations SHALL:
- Set annual improvement targets
- Implement corrective actions
- Track performance metrics
- Benchmark against peers
- Report progress publicly

---

## 16. Data Models

### 16.1 Material Passport Schema

See Section 6.3 for detailed JSON schema.

### 16.2 Lifecycle Event Schema

See Section 7.3 for detailed JSON schema.

### 16.3 Circularity Assessment Schema

```json
{
  "assessmentId": "ASSESS-2025-001234",
  "productId": "PROD-2025-001234",
  "assessmentDate": "2025-12-27",
  "score": 87,
  "rating": "B",
  "mci": 0.87,
  "breakdown": {
    "recycledInput": 85,
    "longevity": 80,
    "design": 92,
    "eolRecovery": 88,
    "reparability": 90,
    "materialEfficiency": 87
  },
  "carbonSaved": 45.0,
  "carbonReduction": 45,
  "wasteReduction": 78,
  "resourceProductivity": 1250,
  "recommendations": [
    {
      "category": "Materials",
      "priority": "high",
      "action": "Increase recycled content to 95%",
      "potentialImpact": 5
    }
  ],
  "validUntil": "2026-12-27"
}
```

### 16.4 EPR Report Schema

```json
{
  "reportId": "EPR-REPORT-2025-Q4",
  "programId": "EPR-PROG-001",
  "period": {
    "start": "2025-10-01",
    "end": "2025-12-31"
  },
  "productsSold": {
    "count": 125000,
    "totalMass": 187500,
    "byCategory": {
      "electronics": 75000,
      "appliances": 50000
    }
  },
  "productsCollected": {
    "count": 98500,
    "totalMass": 147750,
    "collectionRate": 78.8
  },
  "productsRecycled": {
    "count": 89200,
    "totalMass": 133800,
    "recyclingRate": 90.6
  },
  "complianceStatus": "compliant"
}
```

---

## 17. API Specifications

### 17.1 RESTful API Endpoints

#### 17.1.1 Material Passport

```
POST   /api/v1/material-passports
GET    /api/v1/material-passports/{id}
PUT    /api/v1/material-passports/{id}
DELETE /api/v1/material-passports/{id}
GET    /api/v1/material-passports?productId={id}
```

#### 17.1.2 Lifecycle Tracking

```
POST   /api/v1/lifecycle/events
GET    /api/v1/lifecycle/products/{id}
GET    /api/v1/lifecycle/products/{id}/events
POST   /api/v1/lifecycle/products/{id}/refurbishments
```

#### 17.1.3 Circularity Assessment

```
POST   /api/v1/circularity/assess
GET    /api/v1/circularity/assessments/{id}
GET    /api/v1/circularity/products/{id}/score
```

#### 17.1.4 Recycling Routes

```
POST   /api/v1/recycling/routes/find
GET    /api/v1/recycling/facilities
GET    /api/v1/recycling/facilities/{id}
```

#### 17.1.5 EPR Programs

```
POST   /api/v1/epr/programs
GET    /api/v1/epr/programs/{id}
POST   /api/v1/epr/reports
GET    /api/v1/epr/reports/{id}
```

### 17.2 Authentication

API requests SHALL use:
- Bearer token authentication
- API keys for service accounts
- OAuth 2.0 for user authentication

### 17.3 Rate Limiting

- 1000 requests per hour per API key
- 10,000 requests per day per organization
- Exponential backoff for retries

### 17.4 Response Format

All API responses SHALL follow:

```json
{
  "success": true,
  "data": { /* response data */ },
  "metadata": {
    "timestamp": "2025-12-27T10:00:00Z",
    "requestId": "req-123456",
    "version": "1.0"
  }
}
```

Error responses:

```json
{
  "success": false,
  "error": {
    "code": "INVALID_INPUT",
    "message": "Product ID is required",
    "details": { /* additional context */ }
  },
  "metadata": {
    "timestamp": "2025-12-27T10:00:00Z",
    "requestId": "req-123456"
  }
}
```

---

## 18. Security and Privacy

### 18.1 Data Protection

Personal data SHALL be:
- Collected only when necessary
- Stored securely with encryption
- Retained only as long as required
- Deleted upon request (GDPR right to erasure)
- Protected from unauthorized access

### 18.2 Blockchain Privacy

When using blockchain:
- Personal data SHALL NOT be stored on-chain
- Use hashes to reference off-chain data
- Implement access controls
- Support data deletion where possible

### 18.3 Access Control

Implement role-based access control (RBAC):
- Administrator
- Manufacturer
- Recycler
- Auditor
- Consumer
- Public (read-only)

### 18.4 Audit Trails

Maintain comprehensive audit logs:
- User actions
- Data modifications
- API calls
- Authentication events
- Retention: 7 years minimum

---

## 19. Implementation Guidelines

### 19.1 Pilot Program

Organizations SHOULD start with:
1. Select representative product line
2. Create material passports
3. Implement lifecycle tracking
4. Measure circularity metrics
5. Iterate and improve
6. Scale to full portfolio

### 19.2 Stakeholder Engagement

Engage with:
- Internal teams (design, manufacturing, sales)
- Supply chain partners
- Customers
- Recyclers
- Regulators
- NGOs and advocacy groups

### 19.3 Technology Stack

Recommended technologies:
- Cloud infrastructure (AWS, Azure, GCP)
- Database: PostgreSQL, MongoDB
- Blockchain: Ethereum, Hyperledger
- API: RESTful, GraphQL
- Frontend: React, Vue.js
- Mobile: React Native, Flutter

### 19.4 Training and Education

Provide training on:
- Circular economy principles
- Material passport creation
- Lifecycle tracking procedures
- Circularity metrics
- EPR compliance
- Sustainability reporting

---

## 20. Appendices

### Appendix A: Material Recyclability Reference

| Material | Recyclability | Notes |
|----------|---------------|-------|
| Aluminum | 95-100% | Highly recyclable, infinite cycles |
| Steel | 90-95% | Magnetic separation, high value |
| Glass | 100% | Infinite recycling, quality maintained |
| PET Plastic | 70-80% | Quality degrades with cycles |
| HDPE Plastic | 70-80% | Good recycling potential |
| PP Plastic | 60-70% | Growing recycling infrastructure |
| Paper | 75-85% | 5-7 recycling cycles |
| Copper | 95-100% | High value, well-established |
| Lithium Batteries | 85-95% | Complex process, valuable materials |
| Electronics | 65-85% | Labor-intensive disassembly |

### Appendix B: Carbon Intensity Factors

| Material | Virgin (kg CO2/kg) | Recycled (kg CO2/kg) | Savings |
|----------|-------------------|---------------------|---------|
| Aluminum | 12.0 | 0.6 | 95% |
| Steel | 2.5 | 0.4 | 84% |
| Plastic (PET) | 3.5 | 1.5 | 57% |
| Glass | 1.2 | 0.6 | 50% |
| Paper | 2.0 | 1.0 | 50% |
| Copper | 4.0 | 1.0 | 75% |

### Appendix C: Certifications Quick Reference

- **Cradle to Cradle**: Material health, circularity, renewable energy
- **EU Ecolabel**: Environmental performance across lifecycle
- **Zero Waste**: > 90% landfill diversion
- **B Corp**: Social and environmental performance
- **ISO 14001**: Environmental management system
- **ISO 59004**: Circular economy framework

### Appendix D: Glossary

See Section 4 for comprehensive terms and definitions.

### Appendix E: References

1. Ellen MacArthur Foundation. (2015). Towards a Circular Economy
2. ISO 59004:2024. Circular economy - Terminology, principles and guidance
3. Cradle to Cradle Products Innovation Institute. Product Standard v4.0
4. European Commission. Ecodesign Directive 2009/125/EC
5. WIA Standards. Integration guidelines for WIA family standards

---

## Document Control

**Document ID:** WIA-IND-030-v1.0
**Version:** 1.0.0
**Status:** Active
**Effective Date:** 2025-12-27
**Review Date:** 2026-12-27
**Author:** WIA Industry Standards Group
**Approved By:** WIA Technical Committee

---

## Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0.0 | 2025-12-27 | WIA Standards Group | Initial release |

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA - World Certification Industry Association*
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
