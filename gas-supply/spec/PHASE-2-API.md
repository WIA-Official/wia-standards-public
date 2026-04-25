# WIA-SOC-011: Gas Supply Standard
## PHASE 2: API Interface Specification

**Version:** 1.0  
**Status:** Complete  
**Last Updated:** 2025-12-26

---

## 1. Overview

Phase 2 defines RESTful API interfaces for accessing gas supply system data and controlling equipment. APIs follow OpenAPI 3.1 specification and REST architectural principles.

---

## 2. Base API Structure

### 2.1 Endpoint Base URL

```
https://api.gas-operator.com/wia-soc-011/v1
```

### 2.2 Authentication

All requests require OAuth 2.0 Bearer token:

```http
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
```

---

## 3. Pipeline Endpoints

### 3.1 List Pipelines

```http
GET /pipelines
Query Parameters:
  - type: string (transmission|distribution|gathering|service)
  - region: string
  - status: string (active|inactive|planned)
  - limit: integer (default: 50, max: 500)
  - offset: integer

Response: 200 OK
{
  "data": [
    {
      "pipelineId": "PL-KR-001-2025",
      "pipelineType": "transmission",
      "length_km": 145.3,
      "diameter_mm": 914,
      "material": {"type": "steel", "grade": "API-5L-X70"},
      "status": "active"
    }
  ],
  "pagination": {
    "total": 156,
    "limit": 50,
    "offset": 0,
    "hasMore": true
  }
}
```

### 3.2 Get Pipeline Details

```http
GET /pipelines/{pipelineId}

Response: 200 OK
{
  "pipelineId": "PL-KR-001-2025",
  "pipelineType": "transmission",
  "geometry": {...},
  "material": {...},
  "pressureRating": {...},
  "installation": {...},
  "measurements": {
    "latest": "/pipelines/PL-KR-001-2025/measurements/latest",
    "history": "/pipelines/PL-KR-001-2025/measurements"
  }
}
```

---

## 4. Operational Data Endpoints

### 4.1 Real-time Measurements

```http
GET /measurements/realtime
Query Parameters:
  - assetIds: string[] (comma-separated)
  - types: string[] (pressure,flow,temperature)
  - since: ISO8601 timestamp

Response: 200 OK
{
  "measurements": [
    {
      "measurementId": "uuid-here",
      "timestamp": "2025-12-26T14:30:00Z",
      "assetId": "PL-KR-001-2025",
      "type": "pressure",
      "value": 75.5,
      "unit": "bar",
      "quality": "good"
    }
  ]
}
```

### 4.2 Historical Time Series

```http
GET /measurements/history
Query Parameters:
  - assetId: string (required)
  - type: string (required)
  - start: ISO8601 timestamp
  - end: ISO8601 timestamp
  - interval: string (1min|5min|1hour|1day)
  - aggregation: string (avg|min|max|sum)

Response: 200 OK
{
  "assetId": "PL-KR-001-2025",
  "type": "pressure",
  "unit": "bar",
  "interval": "5min",
  "data": [
    {"timestamp": "2025-12-26T00:00:00Z", "value": 75.2},
    {"timestamp": "2025-12-26T00:05:00Z", "value": 75.4}
  ]
}
```

---

## 5. Equipment Control Endpoints

### 5.1 Compressor Control

```http
POST /equipment/compressors/{compressorId}/commands
Request Body:
{
  "command": "start|stop|adjust_speed",
  "parameters": {
    "targetSpeed_rpm": 3600,
    "rampRate_rpm_min": 100
  },
  "authorization": {
    "approvedBy": "operator-id",
    "reason": "Increase pipeline pressure"
  }
}

Response: 202 Accepted
{
  "commandId": "CMD-2025-001",
  "status": "pending",
  "estimatedCompletion": "2025-12-26T14:35:00Z",
  "statusUrl": "/commands/CMD-2025-001/status"
}
```

### 5.2 Valve Control

```http
POST /equipment/valves/{valveId}/commands
Request Body:
{
  "command": "open|close|modulate",
  "parameters": {
    "position_percent": 75,
    "rateOfChange_percent_sec": 5
  }
}
```

---

## 6. Gas Quality Endpoints

```http
GET /gas-quality/composition
Query Parameters:
  - location: string (pipelineId or GPS coordinates)
  - timestamp: ISO8601

Response: 200 OK
{
  "compositionId": "COMP-2025-001",
  "timestamp": "2025-12-26T14:30:00Z",
  "location": {...},
  "components": [...],
  "calculatedProperties": {...}
}
```

---

## 7. Event and Alarm Endpoints

### 7.1 Subscribe to Alarms

```http
POST /events/subscriptions
Request Body:
{
  "filters": {
    "severity": ["high", "critical"],
    "categories": ["safety", "environmental"],
    "assets": ["PL-KR-001-2025"]
  },
  "delivery": {
    "method": "webhook",
    "url": "https://client.com/webhooks/alarms",
    "headers": {"X-API-Key": "secret"}
  }
}

Response: 201 Created
{
  "subscriptionId": "SUB-001",
  "status": "active"
}
```

### 7.2 Acknowledge Alarm

```http
POST /events/{eventId}/acknowledge
Request Body:
{
  "acknowledgedBy": "operator-id",
  "notes": "Investigating high pressure reading"
}
```

---

## 8. Batch Operations

```http
POST /batch
Request Body:
{
  "operations": [
    {
      "method": "GET",
      "path": "/pipelines/PL-KR-001-2025/measurements/latest"
    },
    {
      "method": "GET",
      "path": "/pipelines/PL-KR-002-2025/measurements/latest"
    }
  ]
}

Response: 200 OK
{
  "results": [
    {"status": 200, "body": {...}},
    {"status": 200, "body": {...}}
  ]
}
```

---

## 9. WebSocket Real-time Streaming

```javascript
const ws = new WebSocket('wss://api.gas-operator.com/wia-soc-011/v1/stream');

ws.send(JSON.stringify({
  "action": "subscribe",
  "channels": [
    {"type": "measurements", "assetIds": ["PL-KR-001-2025"]},
    {"type": "alarms", "severity": ["high", "critical"]}
  ]
}));

ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  console.log('Received:', data);
};
```

---

## 10. Error Handling

### 10.1 Error Response Format

```json
{
  "error": {
    "code": "INVALID_PARAMETER",
    "message": "Pressure value exceeds pipeline MAOP",
    "details": {
      "field": "pressure_bar",
      "value": 95.0,
      "constraint": "maop_bar",
      "limit": 80.0
    },
    "correlationId": "req-123-456-789"
  }
}
```

### 10.2 HTTP Status Codes

- 200 OK: Success
- 201 Created: Resource created
- 202 Accepted: Command accepted, processing async
- 400 Bad Request: Invalid parameters
- 401 Unauthorized: Missing/invalid authentication
- 403 Forbidden: Insufficient permissions
- 404 Not Found: Resource not found
- 429 Too Many Requests: Rate limit exceeded
- 500 Internal Server Error: Server error
- 503 Service Unavailable: Temporary unavailability

---

## 11. Rate Limiting

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 987
X-RateLimit-Reset: 1640529600
```

Limits:
- Standard: 1000 requests/hour
- Premium: 10000 requests/hour
- Real-time streaming: No limit on WebSocket connections

---

## 12. API Versioning

- URL versioning: `/v1/`, `/v2/`
- Deprecation warnings in headers:
  ```http
  Sunset: Sat, 01 Jan 2027 00:00:00 GMT
  Deprecation: true
  Link: <https://api.gas-operator.com/wia-soc-011/v2/>; rel="successor-version"
  ```

---

**Document Control**
- Version: 1.0
- OpenAPI Spec: https://schemas.wiastandards.com/soc-011/openapi.yaml
- Contact: standards@wiastandards.com

© 2025 World Certification Industry Association | MIT License

---

## Annex A — Conformance Tier Matrix

WIA conformance for gas-supply is evaluated across three tiers:

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

- `wia-standards/standards/gas-supply/api/` — TypeScript SDK skeleton
- `wia-standards/standards/gas-supply/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/gas-supply/simulator/` — interactive browser-based simulator for the PHASE protocol

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

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.
