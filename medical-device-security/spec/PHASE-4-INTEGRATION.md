# WIA Medical Device Security — PHASE Specification

**Standard slug:** `medical-device-security`
**Editor's draft:** v1.0
**Status:** Normative
**License:** MIT
**弘益人間 — Benefit All Humanity**

> The PHASE documents under `spec/` are the single normative source for the WIA Medical Device Security standard. Code under `api/`, `cli/`, and `simulator/` is informative reference material that demonstrates the contract; production implementations may diverge as long as they preserve the PHASE contract.

This document follows the conventions of the WIA Standards governance procedure published at <https://wiastandards.com/governance>. Conformance terms `MUST`, `MUST NOT`, `SHOULD`, `SHOULD NOT`, `MAY` are used in the sense of IETF RFC 2119 and RFC 8174 ("BCP 14") and apply to the implementation behaviour, not to the specification document itself.

The Medical Device Security standard targets regulated medical device lifecycle and SBOM deployments and is reviewed at every minor revision against the WIA crosswalk in Annex B.

## PHASE 4 — Integration Patterns

### 4.1 Scope

PHASE 4 defines the **integration patterns** that conformant deployments use when composing the standard with adjacent WIA standards, with national or international regulation, and with operator-specific business processes.

### 4.2 Adjacent WIA standards

Implementations integrating with adjacent WIA standards MUST follow the linkages defined in Annex B. Where two WIA standards specify overlapping behaviour, the more specific standard governs unless explicitly superseded by a binding national regulation. Implementations MUST disclose every adjacent WIA standard they integrate with in the `/v1/health` payload's `wia_integrations` member.

### 4.3 Identity federation

Federated identity deployments MUST follow OpenID Connect Discovery and the OpenID Connect Federation drafts as published by the OpenID Foundation. WIA does not republish OpenID Connect; implementers MUST obtain authoritative copies from the issuing body. Implementations SHOULD support SCIM 2.0 per IETF RFC 7644 for user-provisioning flows.

### 4.4 Audit and evidence

Every integration MUST emit audit events conforming to the PHASE-3 envelope, addressed to the operator's audit sink. The events MUST contain:

- The `core-record` `id` of the affected resource.
- The actor identity from the access token's `sub` claim.
- The integration type (a reverse-DNS string).
- An RFC 3339 timestamp.
- An optional W3C PROV-DM provenance pointer per PHASE-1 §1.3.2.

Audit events MUST be retained for the duration declared by the deployment's conformance tier (seven years for Tier 3 by default).

### 4.5 Privacy and data protection

Implementations processing personal data MUST be configurable to support the data-subject rights enumerated in ISO/IEC 29100:2011 (Privacy framework). Where applicable national regulation imposes stricter requirements (for example, the EU General Data Protection Regulation), the stricter requirement governs. WIA does not republish national regulation; implementers MUST consult authoritative sources directly.

### 4.6 Cryptographic primitives

Implementations MUST default to a cryptographic suite from the NIST cryptographic-algorithm validation programme (CMVP) or the IETF JOSE working group's recommended algorithm set at the time of deployment. WIA does not republish CMVP; implementations MUST consult <https://csrc.nist.gov/projects/cryptographic-module-validation-program> for the active list.

Hash functions MUST be from the SHA-2 or SHA-3 family per FIPS 180-4 and FIPS 202. Implementations MUST NOT use MD5 or SHA-1 for new evidence after the publication date of this PHASE.

### 4.7 Hardware security

Tier 3 deployments handling key material MUST store private keys in a hardware security module conformant to FIPS 140-3 Level 2 or higher, or to ISO/IEC 19790:2012 at the equivalent level. Tier 2 deployments SHOULD do the same; Tier 1 deployments MAY operate with software key stores so long as their threat model documents the choice.

### 4.8 Disaster recovery

Tier 3 deployments MUST publish a Recovery Time Objective and Recovery Point Objective for each PHASE-2 endpoint. Implementations SHOULD validate the published RTO/RPO at least annually through a tabletop or live exercise and MUST retain evidence of the exercise for at least the conformance retention period.

### 4.9 Conformance — PHASE 4

A PHASE-4-conformant implementation MUST:

1. Disclose its adjacent WIA-standard integrations per §4.2.
2. Emit audit events on every integration boundary per §4.4.
3. Default to a NIST CMVP or IETF JOSE recommended cryptographic suite per §4.6.
4. Pass every PHASE-4 test vector in `test-vectors/phase-4/`.

Tier 2 implementations MUST additionally publish a third-party assessor report covering §4.4–§4.6. Tier 3 implementations MUST further publish a disaster-recovery exercise report per §4.8.

---

## Annex A — Conformance Tier Matrix

WIA conformance for medical-device-security is evaluated across three tiers:

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

- `wia-standards/standards/medical-device-security/api/` — TypeScript SDK skeleton
- `wia-standards/standards/medical-device-security/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/medical-device-security/simulator/` — interactive browser-based simulator for the PHASE protocol

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



---

## Annex F — Operations and lifecycle notes

This informative annex captures operational guidance that has emerged from reference implementations and is expected to migrate into the normative body in a future minor revision.

### F.1 Deprecation policy

When a member of a canonical schema is deprecated, the schema MUST continue to accept and emit the member for at least 12 months from the publication of the deprecation notice. During the deprecation window, the implementation SHOULD emit a `Deprecation` HTTP header per the IETF deprecation-header draft for any response that contains the deprecated member, and SHOULD provide a `Sunset` header indicating the planned removal date per IETF RFC 8594.

### F.2 Backwards-compatible extensions

Vendors who extend a canonical schema with their own members MUST namespace those members with a reverse-DNS prefix, MUST treat the extension as opt-in, and MUST NOT shadow any normative member name reserved for future minor revisions of the standard. The reserved-name list is published in the WIA-Official catalogue and is updated at every minor revision; vendors SHOULD subscribe to the catalogue feed to be notified of additions.

### F.3 Migration patterns

Implementations migrating between PHASE versions SHOULD adopt the strangler-fig pattern: route new traffic to the new PHASE while continuing to serve old traffic from the previous version, and migrate clients incrementally. Tier 3 deployments MUST publish a written migration plan covering the data-format change, the API surface change, and the protocol change for every major-version bump, and MUST retain that plan for the conformance evidence retention period.

### F.4 Operational telemetry

Every conformant deployment SHOULD expose a small set of operational metrics aligned with the OpenTelemetry semantic conventions. The recommended metric names are:

- `wia.<slug>.requests.duration` — histogram of request durations in seconds.
- `wia.<slug>.requests.errors` — counter of error responses by RFC 9457 `type`.
- `wia.<slug>.records.in_flight` — gauge of records currently being processed.
- `wia.<slug>.conformance.tier` — gauge of declared conformance tier (1, 2, or 3).

Implementations MAY expose additional metrics; the recommended set is intended as a minimum for cross-deployment dashboards.

### F.5 Logging

Logs MUST NOT contain unredacted authentication tokens, raw evidence pointers that the deployment does not own, or any plaintext personal data. Implementations SHOULD adopt structured JSON logging and SHOULD include the W3C Trace Context `trace_id` in every log line so that logs can be joined to the distributed-tracing graph.
