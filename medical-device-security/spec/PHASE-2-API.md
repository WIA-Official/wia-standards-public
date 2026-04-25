# WIA Medical Device Security — PHASE Specification

**Standard slug:** `medical-device-security`
**Editor's draft:** v1.0
**Status:** Normative
**License:** MIT
**弘益人間 — Benefit All Humanity**

> The PHASE documents under `spec/` are the single normative source for the WIA Medical Device Security standard. Code under `api/`, `cli/`, and `simulator/` is informative reference material that demonstrates the contract; production implementations may diverge as long as they preserve the PHASE contract.

This document follows the conventions of the WIA Standards governance procedure published at <https://wiastandards.com/governance>. Conformance terms `MUST`, `MUST NOT`, `SHOULD`, `SHOULD NOT`, `MAY` are used in the sense of IETF RFC 2119 and RFC 8174 ("BCP 14") and apply to the implementation behaviour, not to the specification document itself.

The Medical Device Security standard targets regulated medical device lifecycle and SBOM deployments and is reviewed at every minor revision against the WIA crosswalk in Annex B.

## PHASE 2 — Client-Facing API Surface

### 2.1 Scope

PHASE 2 defines the **client-facing API surface** of the standard. The data formats from PHASE 1 are the message bodies; this PHASE specifies the request/response choreography, error model, identity, and pagination conventions used by every conformant implementation.

The API surface is defined declaratively as an **OpenAPI 3.0** document under `spec/openapi/`. The OpenAPI document is normative. This PHASE document explains the design rationale, the mandatory cross-references, and the conformance-tier requirements that the OpenAPI document itself does not capture.

### 2.2 Endpoints

Every conformant deployment MUST expose the following endpoints under a versioned prefix `/v1/`:

| Endpoint | Method | Body | Returns |
|---|---|---|---|
| `/v1/records` | `GET` | — | `core-record[]` (paginated) |
| `/v1/records/{id}` | `GET` | — | `core-record` |
| `/v1/records` | `POST` | `core-record` | `{ id, receipt }` |
| `/v1/conformance-claims` | `GET` | — | `conformance-claim[]` |
| `/v1/conformance-claims` | `POST` | `conformance-claim` | `{ id, valid_until }` |
| `/v1/health` | `GET` | — | `{ status, build, started_at }` |

Endpoint paths are stable across minor versions. New endpoints MAY be added at any minor version. Removing or changing an endpoint MUST result in a new major version.

### 2.3 Identity

Authenticated endpoints MUST accept **OAuth 2.0 bearer tokens** per IETF RFC 6749 carried in the `Authorization` header. The token format MUST be a JSON Web Token per IETF RFC 7519, signed with one of the algorithms recommended by the IETF JOSE working group at the time of issue. Implementations SHOULD support PKCE per IETF RFC 7636 for clients that cannot maintain a confidential client secret.

The token MUST include the following claims:

```json
{
  "iss": "https://wiastandards.com/<slug>/issuer",
  "aud": "https://wiastandards.com/<slug>",
  "sub": "<unique opaque identifier>",
  "iat": 0, "exp": 0,
  "scope": "<space-separated scopes>",
  "wia_tier": "tier-1 | tier-2 | tier-3"
}
```

`wia_tier` is a private claim that mirrors the deployment's declared conformance tier and SHOULD be enforced by the resource server when authorising sensitive endpoints.

### 2.4 Pagination

List endpoints MUST implement cursor-based pagination using `?cursor=…&limit=…` query parameters. The response carries a `Link` header with `rel="next"` and (when applicable) `rel="prev"` URIs per IETF RFC 8288. Implementations MUST use opaque cursors; clients MUST NOT decode cursor values.

### 2.5 Error model

Errors MUST be returned as **RFC 9457 problem-details** objects with `Content-Type: application/problem+json`. The `type` member MUST be a stable URI under `https://wiastandards.com/<slug>/errors/`. WIA reserves the following error types under this namespace:

```json
{
  "type": "https://wiastandards.com/<slug>/errors/validation-failed",
  "title": "Validation failed",
  "status": 422,
  "detail": "<human-readable explanation>",
  "instance": "/v1/records/abc",
  "errors": [ { "path": "/issuer", "message": "format must be uri" } ]
}
```

Application-specific extension members MUST be namespaced with a `wia_` prefix to avoid collisions with future WIA-reserved members.

### 2.6 Caching

Idempotent `GET` endpoints SHOULD emit `ETag` headers. Clients SHOULD use `If-None-Match` for conditional GETs to reduce bandwidth. Mutating endpoints MUST NOT be cached and MUST set `Cache-Control: no-store`.

### 2.7 Observability

Every response MUST carry an `X-WIA-Trace-Id` header containing a W3C Trace Context-compatible trace identifier. Implementations SHOULD propagate inbound `traceparent` and `tracestate` headers per the W3C Trace Context recommendation. Tier 3 deployments MUST surface the trace identifier in any user-facing error message so that operators can correlate complaints with backend evidence.

### 2.8 Conformance — PHASE 2

A PHASE-2-conformant implementation MUST:

1. Expose the endpoint table in §2.2 at the documented paths.
2. Authenticate every non-health request per §2.3.
3. Return RFC 9457 problem-details on every error path.
4. Implement cursor pagination per §2.4.
5. Pass every PHASE-2 test vector in `test-vectors/phase-2/`.

Tier 2 implementations MUST additionally publish an OpenAPI document that lints clean under Spectral with the WIA ruleset (see Annex C). Tier 3 implementations MUST further publish a signed third-party assessor report confirming that the live deployment matches the published OpenAPI document.

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


## Annex E — Implementation Notes for PHASE-2-API

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-2-API.

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
