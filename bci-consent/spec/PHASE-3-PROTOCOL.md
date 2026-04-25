# WIA BCI Consent Protocol
## Phase 3: Protocol Workflows Specification

**Version:** 1.0.0  
**Status:** Release  
**Last Updated:** 2025-12-25

---

## 1. Introduction

This specification defines end-to-end workflows for consent management in BCI systems.

## 2. Initial Consent Workflow

### 2.1 Workflow Steps

1. **Capacity Assessment** - Evaluate ability to consent
2. **Information Provision** - Present risks, benefits, alternatives
3. **Understanding Verification** - Confirm comprehension
4. **Voluntary Agreement** - Ensure free choice
5. **Digital Signature** - Capture cryptographic signature
6. **Record Creation** - Generate and store consent record
7. **Confirmation** - Provide copy to subject

### 2.2 State Machine

```
[Initial] → [Capacity Assessment] → [Information Provided] 
→ [Understanding Verified] → [Agreement Obtained] 
→ [Signed] → [Active]
```

## 3. Ongoing Consent Workflow

### 3.1 Pre-Operation Verification

Before each BCI operation:
1. Verify consent exists and is active
2. Check required permissions granted
3. Confirm not expired
4. Log verification attempt

### 3.2 Periodic Renewal

```
30 days before expiration:
  - Send renewal reminder to subject
  - Provide updated information
  - Request renewed consent
  
Upon renewal:
  - Create new consent version
  - Link to previous version
  - Update expiration date
```

## 4. Modification Workflow

1. **Change Request** - Subject requests modification
2. **Impact Analysis** - Assess implications
3. **Additional Disclosure** - Explain changes
4. **Understanding Confirmation** - Verify comprehension
5. **Signature** - Capture signature on modifications
6. **Version Creation** - Create new version with tracking

## 5. Revocation Workflow

1. **Revocation Request** - Subject initiates
2. **Confirmation** - Verify intentional
3. **Data Processing Cessation** - Stop all processing immediately
4. **Data Deletion Timeline** - Schedule data deletion
5. **Notification** - Inform all parties
6. **Documentation** - Record revocation

### 5.1 Data Deletion Schedule

| Data Type | Timeline |
|-----------|----------|
| Real-time data | Immediate |
| Cached data | 24 hours |
| Analytical data | 30 days |
| Backup data | 90 days |
| Anonymized aggregates | Retained |
| Regulatory submissions | Per requirements |

## 6. Emergency Override Protocol

### 6.1 Conditions

- Immediate threat to life or serious harm
- Consent impossible to obtain
- Delay would increase risk
- Within standard of care

### 6.2 Process

1. **Emergency Determination** - Physician declares emergency
2. **Authorization** - Licensed physician authorizes
3. **Documentation** - Record justification
4. **Time Limit** - 24-48 hours maximum
5. **Ethics Review** - Submit within 48 hours
6. **Transition** - Obtain proper consent when able

---

**弘益人間 (Hongik Ingan)** - *Benefit All Humanity*

© 2025 SmileStory Inc. / WIA

---

## Annex A — Conformance Tier Matrix

WIA conformance for bci-consent is evaluated across three tiers:

| Tier | Scope | Mandatory artifacts | Audit cadence |
|------|-------|--------------------|----------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | OpenAPI 3.0 contract, JSON Schema validation report, security threat model | Annual self-review |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Tier 1 artifacts + signed third-party assessor report against this PHASE | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Tier 2 artifacts + WIA accreditation, ISO/IEC 17065:2012 conformity assessment, evidence retention ≥ 7 years | Every 12 months |

Implementations MUST disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page. Tier downgrade events MUST be reported to the WIA registry within 30 days.

---

## Annex B — Cross-Walk to International Standards

The PHASE specification reuses or normatively references published standards alongside this document. Implementers SHOULD review the relevant standards alongside this PHASE document; where a conflict exists, the more specific WIA requirement governs unless explicitly superseded by a binding national regulation.

- ISO/IEC 17065:2012 — Conformity assessment — Requirements for bodies certifying products, processes and services
- ISO/IEC 27001:2022 — Information security management systems
- IETF RFC 9457 — Problem Details for HTTP APIs
- IETF RFC 7519 — JSON Web Token (JWT)
- IETF RFC 6749 — The OAuth 2.0 Authorization Framework
- W3C PROV-DM — provenance data model

This cross-walk is informative only. WIA does not republish the referenced documents; readers MUST obtain authoritative copies from the issuing body. Cross-walk entries are reviewed at every minor version of this PHASE.

---

## Annex C — Reference Implementations and Test Vectors

### C.1 Reference Implementations

WIA does not require implementers to use a particular library, but maintains pointers to the canonical reference implementation directories under the WIA-Official GitHub organization:

- `wia-standards/standards/bci-consent/api/` — TypeScript SDK skeleton
- `wia-standards/standards/bci-consent/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/bci-consent/simulator/` — interactive browser-based simulator for the PHASE protocol

Each reference artifact ships with an MIT license and is intended as a starting point, not as a production-grade implementation.

### C.2 Test Vectors

A normative set of request/response pairs covering the schemas defined in this PHASE is published alongside this document. Implementations claiming Tier 2 or Tier 3 conformance MUST pass every published test vector in both serialization (request) and deserialization (response) directions, and MUST publish a signed report identifying which vectors were exercised.

Test vectors are versioned independently of the PHASE document; refer to the `test-vectors/` directory for the active version.

---

## Annex D — Open Questions and Future Work

This PHASE document captures the consensus position at v1.0. The following items are tracked for future minor or major revisions:

1. **Schema versioning policy** — formal MUST/SHOULD rules for backward-compatible vs. breaking changes between minor releases.
2. **Privacy-preserving aggregation** — guidance on differential privacy and secure aggregation patterns for telemetry channels, without prescribing a specific algorithm.
3. **Multilingual error catalogs** — localization strategy for the standard error codes defined in this PHASE.
4. **Long-term retention** — alignment with sectoral retention regulations across jurisdictions.
5. **Sustainability disclosure** — optional fields for energy and emissions reporting tied to operations covered by this PHASE.

Items in this annex are non-normative. Comments and proposals are accepted via the GitHub issues tracker on the WIA-Official organization.

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
