# WIA Medical AI Ethics — PHASE Specification

**Standard slug:** `medical-ai-ethics`
**Editor's draft:** v1.0
**Status:** Normative
**License:** MIT
**弘益人間 — Benefit All Humanity**

> The PHASE documents under `spec/` are the single normative source for the WIA Medical AI Ethics standard. Code under `api/`, `cli/`, and `simulator/` is informative reference material that demonstrates the contract; production implementations may diverge as long as they preserve the PHASE contract.

This document follows the conventions of the WIA Standards governance procedure published at <https://wiastandards.com/governance>. Conformance terms `MUST`, `MUST NOT`, `SHOULD`, `SHOULD NOT`, `MAY` are used in the sense of IETF RFC 2119 and RFC 8174 ("BCP 14") and apply to the implementation behaviour, not to the specification document itself.

The Medical AI Ethics standard targets medical AI lifecycle and bias-audit deployments and is reviewed at every minor revision against the WIA crosswalk in Annex B.

## PHASE 1 — Canonical Data Formats

### 1.1 Scope

PHASE 1 defines the **canonical data formats** for the standard. It is the foundation referenced by every later PHASE: the API surface (PHASE 2), the wire-level protocol (PHASE 3), and the integration patterns (PHASE 4) all serialise the structures defined here.

Implementations claiming any conformance tier MUST be able to consume the canonical structures defined in §1.3, and MUST emit them when interoperating with other WIA-conformant systems. Implementations MAY internally use richer or alternative representations as long as the canonical structures are exposed at the contract boundary.

### 1.2 Notation

Schemas are expressed in **JSON Schema 2020-12** unless explicitly stated otherwise. Where helpful, an inline TypeScript type is provided for clarity; the JSON Schema document is normative.

```jsonc
// PHASE-1 base envelope (informative TypeScript notation)
type PhaseEnvelope<T> = {
  envelope_version: "wia/1";
  generated_at: string;        // RFC 3339 timestamp
  generator: { name: string; version: string };
  payload: T;
};
```

### 1.3 Canonical structures

Each canonical structure is identified by an `$id` URI of the form
`https://wiastandards.com/<slug>/schemas/<name>.json`. Implementations MUST treat the `$id` as the stable identity of the schema across versions; backwards-incompatible changes MUST result in a new `$id` and a new spec major version.

#### 1.3.1 `<core-record>`

```json
{
  "$id": "https://wiastandards.com/<slug>/schemas/core-record.json",
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "type": "object",
  "required": ["id", "type", "version", "issued_at"],
  "properties": {
    "id": { "type": "string", "minLength": 1, "maxLength": 256 },
    "type": { "type": "string", "minLength": 1, "maxLength": 64 },
    "version": { "type": "string", "pattern": "^[0-9]+\\.[0-9]+\\.[0-9]+$" },
    "issued_at": { "type": "string", "format": "date-time" },
    "issuer": { "type": "string", "format": "uri" },
    "subject": { "type": "string", "format": "uri" },
    "evidence": { "type": "array", "items": { "type": "string", "format": "uri" } },
    "extensions": { "type": "object", "additionalProperties": true }
  }
}
```

The `core-record` is the unit of exchange between conformant systems. The `extensions` member is reserved for **non-normative** extensions that MUST NOT shadow normative members defined in this PHASE or in any future minor revision. Extensions SHOULD be namespaced with a reverse-DNS prefix.

#### 1.3.2 `<provenance-pointer>`

```json
{
  "$id": "https://wiastandards.com/<slug>/schemas/provenance-pointer.json",
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "type": "object",
  "required": ["pointer_type", "value"],
  "properties": {
    "pointer_type": { "enum": ["w3c-prov", "did", "uri", "internal"] },
    "value": { "type": "string", "minLength": 1, "maxLength": 4096 },
    "as_of": { "type": "string", "format": "date-time" }
  }
}
```

Provenance pointers SHOULD use the W3C PROV-DM data model whenever the consuming system supports it. When the consuming system does not, an opaque URI MAY be used; in that case the issuing system MUST commit to maintaining the URI for the retention period declared by its conformance tier.

#### 1.3.3 `<conformance-claim>`

```json
{
  "$id": "https://wiastandards.com/<slug>/schemas/conformance-claim.json",
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "type": "object",
  "required": ["tier", "issued_by", "valid_until", "evidence"],
  "properties": {
    "tier": { "enum": ["tier-1", "tier-2", "tier-3"] },
    "issued_by": { "type": "string", "format": "uri" },
    "valid_until": { "type": "string", "format": "date-time" },
    "evidence": {
      "type": "array",
      "minItems": 1,
      "items": { "type": "string", "format": "uri" }
    }
  }
}
```

A conformance claim is the standard's primary trust signal. Tier 2 and Tier 3 claims MUST reference assessor reports retained for at least seven years.

### 1.4 Serialisation

Implementations MUST support **UTF-8 JSON** as the canonical wire serialisation. Implementations MAY additionally support CBOR (RFC 8949) where binary efficiency is required (for example, in resource-constrained deployments). When CBOR is used, the canonical mapping is the deterministic encoding defined in RFC 8949 §4.2.

### 1.5 Versioning

The PHASE-1 schemas are versioned together with the spec document. A breaking change requires:

1. A new `$id` URI for every changed schema.
2. A new major version on the spec document.
3. A migration note in `Annex D — Open Questions and Future Work` of every PHASE in this standard.

Non-breaking additions are signalled by the spec minor version. Implementers SHOULD ignore unknown members during deserialisation and MUST NOT reject documents solely because they contain additive members from a later minor version.

### 1.6 Conformance — PHASE 1

A PHASE-1-conformant implementation MUST:

1. Validate every produced canonical record against the appropriate schema before emission.
2. Validate every consumed canonical record against the appropriate schema before acting on it.
3. Reject documents that fail validation with an RFC 9457 problem-details response when an HTTP context applies.
4. Preserve unknown fields when round-tripping documents through stages that are not the schema authority.
5. Disclose its declared conformance tier in the `conformance-claim` accompanying each emission.

Failure to comply with the above MUST result in the implementation being downgraded to "non-conformant" until the failure is remedied. Tier 2 and Tier 3 implementations additionally MUST publish a conformance attestation against this PHASE.

### 1.7 Test vectors

The `test-vectors/phase-1/` directory contains a normative set of accept and reject documents covering every canonical structure. Implementations claiming Tier 2 or Tier 3 MUST pass every published vector in both serialisation directions and MUST publish a signed report identifying which vectors were exercised.

The vectors are versioned independently of the PHASE document. Implementations MUST report the vector-set version in their attestations.

---

## Annex A — Conformance Tier Matrix

WIA conformance for medical-ai-ethics is evaluated across three tiers:

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

- `wia-standards/standards/medical-ai-ethics/api/` — TypeScript SDK skeleton
- `wia-standards/standards/medical-ai-ethics/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/medical-ai-ethics/simulator/` — interactive browser-based simulator for the PHASE protocol

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
