# WIA Cryo-Monitoring SDK and Client API Specification

**Phase 2: SDK / Client API**

**Version**: 1.0.0
**Status**: Draft
**Date**: 2026-04-26
**Standard ID**: WIA-CRYO-008

---

## 1. Overview

### 1.1 Purpose

WIA Cryo-Monitoring Phase 2는 Phase 1 데이터 포맷을 사용하는 클라이언트 SDK와 RESTful Client API의 표준 계약을 정의합니다. 액체질소 레벨 조회, 센서 판독값 수집, 알람 이벤트 등록, 보정 기록 회수까지 통일된 인터페이스로 제공합니다.

This document defines the client-facing surface area: the language-agnostic REST API endpoints and the reference TypeScript SDK shape that cryo-monitoring backends MUST implement and clients MAY rely on.

### 1.2 Design Goals

1. **Format-Faithful** — payloads carry the WIA Cryo-Monitoring objects defined in PHASE 1 verbatim, with no lossy normalization.
2. **Sensor-Class Aware** — temperature (IEC 60751 RTD or IEC 60584-1 thermocouple) and level sensors expose their class as metadata.
3. **Audit First** — every mutating call records a calibration- or operator-attributed audit entry.
4. **Stable** — the REST contract is versioned independently of the underlying schema.
5. **Backend Portable** — the same SDK MUST be able to talk to any compliant backend without code change.

### 1.3 Scope

| In scope | Out of scope |
|----------|--------------|
| REST endpoint contract | Vendor-specific PLC drivers |
| TypeScript reference SDK shape | Refilling/dispensing actuators |
| OpenAPI 3.0 surface | Real-time MQTT/OPC UA transport (PHASE 3) |
| Authentication and quotas | LIMS/biobank workflow orchestration (PHASE 4) |
| Error catalog | Clinical or diagnostic claims |

This specification is process-monitoring infrastructure. It does not claim, prescribe, or evaluate medical or clinical efficacy of stored materials.

---

## 2. Transport and Encoding

### 2.1 Base URL

```
https://{backend-host}/api/v1
```

### 2.2 Content Type

All request and response bodies use `application/json; charset=utf-8`.

### 2.3 Versioning Header

Clients MUST send the WIA Cryo-Monitoring version they were built against:

```
X-WIA-Cryo-Version: 1.0.0
```

If the backend cannot honor the requested version, it MUST return HTTP 400 with `error.code = "UNSUPPORTED_VERSION"`.

### 2.4 Idempotency

Mutating endpoints accept an `Idempotency-Key` header (UUIDv4). When a duplicate key is observed within 24 hours, the backend MUST return the original response without re-executing the operation.

---

## 3. Authentication

OAuth 2.0 (IETF RFC 6749) with PKCE (IETF RFC 7636) is the default for interactive operators; service accounts MAY use the client-credentials grant. JWTs follow IETF RFC 7519.

### 3.1 Scopes

| Scope | Grants |
|-------|--------|
| `sensors.read` / `.write` | Read or register sensor inventory |
| `readings.read` / `.write` | Read or post sensor readings |
| `alerts.read` / `.write`  | Read alert events or post operator-acknowledged events |
| `levels.read` / `.write`  | Read or post LN2 level events |
| `calibration.manage`      | Submit and revoke calibration records |
| `provenance.read`         | Read audit graphs |

---

## 4. Resource Endpoints

### 4.1 Sensors

| Method | Path | Description |
|--------|------|-------------|
| `GET`    | `/sensors`            | List sensor inventory |
| `POST`   | `/sensors`            | Register a sensor |
| `GET`    | `/sensors/{id}`       | Sensor descriptor (class, location, calibration epoch) |
| `PUT`    | `/sensors/{id}`       | Replace a sensor descriptor |
| `DELETE` | `/sensors/{id}`       | Decommission a sensor |

### 4.2 Readings

| Method | Path | Description |
|--------|------|-------------|
| `POST` | `/readings`             | Bulk-post one or more sensor readings |
| `GET`  | `/readings`             | Query readings (range filter on `sensor_id`, `from`, `to`) |
| `GET`  | `/readings/{id}`        | Read a single reading record |

### 4.3 Alerts

| Method | Path | Description |
|--------|------|-------------|
| `GET`    | `/alerts`            | List alert events |
| `POST`   | `/alerts`            | Post a new alert event |
| `GET`    | `/alerts/{id}`       | Read an alert event |
| `POST`   | `/alerts/{id}/ack`   | Acknowledge an alert (operator-attributed) |

### 4.4 LN2 Levels

| Method | Path | Description |
|--------|------|-------------|
| `GET`  | `/levels`             | Query LN2 level events |
| `POST` | `/levels`             | Post an LN2 level reading |
| `GET`  | `/levels/{id}`        | Read a level event |

### 4.5 Calibration Records

| Method | Path | Description |
|--------|------|-------------|
| `GET`    | `/calibration`        | List calibration records |
| `POST`   | `/calibration`        | Submit a calibration record |
| `GET`    | `/calibration/{id}`   | Read a calibration record |
| `DELETE` | `/calibration/{id}`   | Revoke a calibration record |

---

## 5. Reference TypeScript SDK Shape

```ts
import { WiaCryoClient } from "@wia-official/cryo-monitoring";

const client = new WiaCryoClient({
  baseUrl: "https://cryo.example.com/api/v1",
  auth: { kind: "bearer", token: process.env.WIA_CRYO_TOKEN },
  version: "1.0.0",
});

await client.readings.post([{
  sensor_id: "sens-temp-001",
  observed_at: "2026-04-26T03:00:00Z",
  value_celsius: -195.8,
  quality: "good",
}]);

const open = await client.alerts.list({ status: "open", limit: 50 });
```

The SDK MUST: encode retries with exponential backoff (250 ms initial, factor 2, jitter ±25 %, max 6); surface every backend error as a `WiaCryoError` with `code`, `message`, `requestId`; honor `Retry-After` on HTTP 429.

---

## 6. Pagination, Filtering, Sorting

| Parameter | Default | Notes |
|-----------|---------|-------|
| `page_size`  | 50 | 1–500 (readings endpoints allow up to 500) |
| `page_token` | (none) | opaque cursor |
| `filter`     | (none) | RFC 7644 SCIM-like filter |
| `order_by`   | `observed_at desc` | sortable per resource |

---

## 7. Error Model

All error responses follow IETF RFC 9457 (Problem Details for HTTP APIs):

```json
{
  "type": "https://wiastandards.com/errors/cryo/reading-out-of-range",
  "title": "Reading rejected",
  "status": 400,
  "code": "READING_OUT_OF_RANGE",
  "detail": "value_celsius=-300 below sensor floor (-200)",
  "instance": "/api/v1/readings",
  "request_id": "req_<32+chars>"
}
```

Standard codes: `INVALID_REQUEST` (400), `UNAUTHENTICATED` (401), `FORBIDDEN` (403), `NOT_FOUND` (404), `CONFLICT` (409), `UNSUPPORTED_VERSION` (400), `READING_OUT_OF_RANGE` (400), `QUOTA_EXCEEDED` (429), `INTERNAL_ERROR` (500).

---

## 8. Quotas and Rate Limits

| Tier | Default rate limit |
|------|--------------------|
| Public | 60 requests / minute (sliding window) |
| Authenticated | 600 requests / minute per token |
| Premium | negotiated |

Bulk `/readings` posts are independently rate-limited. Backends emit the standard `RateLimit-Limit`, `RateLimit-Remaining`, `RateLimit-Reset` headers.

---

## 9. Observability and Audit Contract

Compliant backends MUST emit `request_id` (echoed in `X-Request-Id`), `tenant_id`, `endpoint`, `http_status`, `latency_ms`, `error_code`. Audit entries written for every mutating call MUST follow W3C PROV-DM and reference the `prov:wasAttributedTo` agent (the authenticated principal) and the `prov:generatedAtTime` timestamp.

`/healthz` and `/readyz` SHOULD be exposed for orchestration probes.

---

## 10. Compliance and Certification

| Conformance level | Required surface |
|-------------------|------------------|
| Tier 1 — Self-declared | All §4 endpoints + RFC 9457 errors + OpenAPI document + audit writes per §9 |
| Tier 2 — Assessed | Tier 1 + signed third-party assessor report against this PHASE |
| Tier 3 — Accredited | Tier 2 + WIA accreditation + ISO/IEC 17065:2012 + ISO 20387:2018 alignment for biobanking deployments |

A backend that omits §4.5 (calibration) MAY still claim Tier 1 conformance for the implemented subset, but MUST advertise the omission in its OpenAPI document via `info.x-wia-cryo-omits`.

---

## 11. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2026-04-26 | Initial PHASE 2 specification (this document) |

---

## 12. References

- WIA Cryo-Monitoring PHASE 1 Data Format Specification (this repository)
- WIA Cryo-Monitoring PHASE 3 Communication Protocol (this repository)
- WIA Cryo-Monitoring PHASE 4 Ecosystem Integration (this repository)
- IEC 60751 — Industrial platinum resistance thermometers
- IEC 60584-1 — Thermocouples — EMF specifications and tolerances
- IEC 61508 — Functional safety of electrical/electronic/programmable electronic safety-related systems
- IEC 62541 (OPC UA) — Unified architecture
- ISO/IEC 20922 (MQTT 5.0) — Message Queuing Telemetry Transport
- ISO 20387:2018 — Biotechnology — Biobanking — General requirements for biobanking
- ISO/IEC 17065:2012 — Conformity assessment
- IETF RFC 6749 — The OAuth 2.0 Authorization Framework
- IETF RFC 7519 — JSON Web Token (JWT)
- IETF RFC 7636 — Proof Key for Code Exchange by OAuth Public Clients
- IETF RFC 7644 — System for Cross-domain Identity Management: Protocol
- IETF RFC 9457 — Problem Details for HTTP APIs

---

## Annex A — Conformance Tier Matrix

WIA conformance for the WIA Cryo-Monitoring SDK and Client API surface is evaluated across three tiers, applied to sensor inventory, reading ingest, alert lifecycle, level events, and calibration records:

| Tier | Scope | Mandatory artifacts | Audit cadence |
|------|-------|--------------------|----------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | OpenAPI 3.0 contract, JSON Schema validation report, security threat model, audit record samples | Annual self-review |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Tier 1 artifacts + signed third-party assessor report against this PHASE | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Tier 2 artifacts + WIA accreditation, ISO/IEC 17065:2012 conformity assessment, ISO 20387:2018 biobanking alignment where applicable, evidence retention ≥ 7 years | Every 12 months |

---

## Annex B — Cross-Walk to International Standards

The PHASE specification reuses or normatively references the standards listed in §12. Implementers SHOULD review the listed standards alongside this PHASE document; where a conflict exists, the more specific WIA requirement governs unless explicitly superseded by a binding national regulation.

This cross-walk is informative only.

---

## Annex C — Reference Implementations and Test Vectors

WIA does not require implementers to use a particular library, but maintains pointers to the canonical reference implementation directories under the WIA-Official GitHub organization:

- `wia-standards/standards/cryo-monitoring/api/` — TypeScript SDK skeleton
- `wia-standards/standards/cryo-monitoring/cli/` — POSIX shell client
- `wia-standards/standards/cryo-monitoring/simulator/` — interactive browser-based simulator

A normative set of request/response pairs covering the schemas defined in this PHASE is published alongside this document. Implementations claiming Tier 2 or Tier 3 conformance MUST pass every published test vector in both serialization and deserialization directions.

---

## Annex D — Open Questions and Future Work

1. **Streaming reading ingest** — chunked submission for high-frequency telemetry without bulk-post quotas.
2. **Sensor self-test reporting** — first-class field for IEC 61508 functional-safety self-test outcomes.
3. **Federated calibration registries** — inter-organization recognition of calibration records.
4. **Time-series compression negotiation** — declarative content-encoding hints for long-window queries.
5. **Sustainability disclosure** — optional fields for energy and refrigerant reporting tied to facility operations.

Items in this annex are non-normative.

---

**弘益人間 · Benefit All Humanity**

© 2026 WIA — World Certification Industry Association
Licensed under MIT License


## Annex E — Implementation Notes for PHASE-2-API

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-2-API.

- **Sensor populations** — Mixed RTD (IEC 60751 Class A) and Type-T
  thermocouple (IEC 60584-1) installations are common in walk-in cryo
  rooms; implementers SHOULD record the sensor class on every sensor
  descriptor so that downstream calibration tooling can compute the
  correct uncertainty envelope per the relevant tolerance class.
- **LN2 dewar telemetry** — Capacitive level probes typically sample at
  0.1 Hz; ultrasonic probes at 1 Hz. The wire formats accommodate both
  without changing the schema.
- **Operator workstations** — Operators typically supervise 4–16
  rooms simultaneously; UI implementations SHOULD subscribe to the
  facility-level alert topic class only and lazy-fetch reading streams
  on demand. Eager-fetch designs raise broker load disproportionately
  to the operator's attention budget.
- **Audit retention** — A 7-year retention window is sufficient to
  satisfy ISO 20387:2018 audit expectations in most jurisdictions; some
  regulators require longer retention for human-derived materials, in
  which case the deployment policy MUST extend the retention window
  rather than the standard's defaults.
- **Time synchronization** — Sub-second alert deadlines depend on
  synchronized clocks. NTPv4 with stratum-2 servers is sufficient for
  the deadlines defined in PHASE 3 §4; PTP is recommended for sites
  that require deterministic interlocks.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA Cryo-Monitoring conformance.


