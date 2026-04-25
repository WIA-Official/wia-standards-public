# WIA Medical AI Ethics — PHASE Specification

**Standard slug:** `medical-ai-ethics`
**Editor's draft:** v1.0
**Status:** Normative
**License:** MIT
**弘益人間 — Benefit All Humanity**

> The PHASE documents under `spec/` are the single normative source for the WIA Medical AI Ethics standard. Code under `api/`, `cli/`, and `simulator/` is informative reference material that demonstrates the contract; production implementations may diverge as long as they preserve the PHASE contract.

This document follows the conventions of the WIA Standards governance procedure published at <https://wiastandards.com/governance>. Conformance terms `MUST`, `MUST NOT`, `SHOULD`, `SHOULD NOT`, `MAY` are used in the sense of IETF RFC 2119 and RFC 8174 ("BCP 14") and apply to the implementation behaviour, not to the specification document itself.

The Medical AI Ethics standard targets medical AI lifecycle and bias-audit deployments and is reviewed at every minor revision against the WIA crosswalk in Annex B.

## PHASE 3 — Wire-Level Protocol

### 3.1 Scope

PHASE 3 defines the **wire-level protocol** for streaming and event-driven exchanges between conformant deployments. Whereas PHASE 2 specifies the request/response surface, this PHASE specifies the choreography of long-running, multi-party, or asynchronous workflows.

### 3.2 Transports

Conformant deployments MUST support **HTTPS** as the baseline transport for the PHASE-2 endpoints. Conformant deployments SHOULD additionally support one of the following streaming transports:

- **Server-Sent Events** (HTML Living Standard) for unidirectional notifications from server to client.
- **WebSockets** per IETF RFC 6455 for bidirectional message exchange.
- **WebTransport over HTTP/3** per IETF QUIC and HTTP/3 specifications when sub-RTT delivery is required.

Implementations MUST disclose the supported streaming transports in the `/v1/health` payload's `transports` member so that clients can negotiate without trial-and-error.

### 3.3 Message envelope

All streaming messages MUST be wrapped in a **Cloud Events 1.0** envelope. The envelope is a JSON object with the following members:

```json
{
  "specversion": "1.0",
  "id": "01JF7AX9N2RG3QHC6CFFGTMK1Z",
  "source": "https://wiastandards.com/<slug>/source",
  "type": "com.wia.<slug>.record.created",
  "time": "2026-04-26T11:30:00Z",
  "datacontenttype": "application/json",
  "subject": "<core-record id>",
  "data": { /* PHASE-1 payload */ }
}
```

The `type` member MUST follow the `com.wia.<slug>.<entity>.<lifecycle>` reverse-DNS scheme. WIA reserves the `com.wia.*` namespace; vendors extending the standard MUST use their own namespace and SHOULD register it in the WIA-Official catalogue.

### 3.4 Identifiers

Message identifiers MUST be unique within the issuer. Implementations SHOULD use ULIDs (lexicographic order) or UUIDv7 per IETF draft-ietf-uuidrev. Implementations MUST NOT reuse identifiers within the retention window declared by their conformance tier.

### 3.5 Backpressure and flow control

Streaming consumers MUST signal backpressure via the transport's native mechanism (TCP for SSE, RFC 6455 frame mechanics for WebSockets, QUIC stream credit for WebTransport). Implementations MUST NOT silently drop messages on the producer side; when an outbound message cannot be delivered within a deployment-configured timeout, the producer MUST emit an `com.wia.<slug>.delivery.failed` event with the offending message identifier and a stable error code.

### 3.6 Replay and idempotency

Consumers MUST tolerate at-least-once delivery. Producers SHOULD include a deterministic `id` per §3.4 so that consumers can deduplicate using a small in-memory cache keyed by `id`. Idempotency keys for `POST` endpoints in PHASE 2 MUST be honoured for at least 24 hours; implementations MAY honour them for longer.

### 3.7 Error propagation

Streaming error events MUST carry the same RFC 9457 problem-details body as the synchronous PHASE-2 error path. The Cloud Events envelope's `type` member MUST be `com.wia.<slug>.error` and the `data` member MUST be the problem-details object.

### 3.8 Disconnect handling

When a streaming connection disconnects:

1. The producer MUST persist any unacknowledged events to durable storage.
2. The consumer MUST reconnect with exponential backoff starting at 1 s, doubling up to 64 s, with full jitter.
3. The consumer MUST resume from the last acknowledged identifier supplied via the `Last-Event-ID` header (SSE) or an equivalent transport-level mechanism.

### 3.9 Conformance — PHASE 3

A PHASE-3-conformant implementation MUST:

1. Expose at least one streaming transport from §3.2.
2. Wrap every event in a Cloud Events 1.0 envelope per §3.3.
3. Honour idempotency keys for at least 24 hours per §3.6.
4. Reconnect per §3.8 on transient disconnects.
5. Pass every PHASE-3 test vector in `test-vectors/phase-3/`.

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
