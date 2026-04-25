# WIA-ROB-011 Phase 2: API Interface Specification

**Version:** 1.0.0  
**Status:** Approved  
**Last Updated:** 2025-12-26

弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. API Overview

Phase 2 defines the RESTful API and WebSocket interfaces for controlling and monitoring cleaning robots. All endpoints MUST implement proper authentication and rate limiting.

## 2. Base URL

```
http://{robot_ip}:{port}/api/v1
```

Default port: 8080

## 3. Authentication

### 3.1 Token-Based Auth

```http
POST /auth/token
Content-Type: application/json

{
  "username": "string",
  "password": "string"
}

Response:
{
  "access_token": "JWT token",
  "refresh_token": "string",
  "expires_in": 3600
}
```

### 3.2 Using Tokens

```http
Authorization: Bearer {access_token}
```

## 4. Core Endpoints

### 4.1 Robot Status

```http
GET /robot/status

Response:
{
  "robotId": "string",
  "state": "RobotState object",
  "uptime": "integer (seconds)",
  "lastUpdate": "ISO8601 datetime"
}
```

### 4.2 Start Cleaning

```http
POST /robot/clean
Content-Type: application/json

{
  "mode": "auto|spot|edge|custom",
  "rooms": ["room_id1", "room_id2"],
  "powerLevel": 0-100,
  "waterLevel": 0-100
}

Response:
{
  "sessionId": "UUID",
  "status": "started",
  "estimatedDuration": "integer (seconds)"
}
```

### 4.3 Stop Cleaning

```http
POST /robot/stop

Response:
{
  "status": "stopped",
  "sessionId": "UUID",
  "areaCleaned": "float (m²)"
}
```

### 4.4 Return to Dock

```http
POST /robot/dock

Response:
{
  "status": "returning",
  "distance": "float (meters)",
  "estimatedTime": "integer (seconds)"
}
```

### 4.5 Get Map

```http
GET /robot/map?floor={floor_number}

Response:
{
  "map": "OccupancyGrid object",
  "lastUpdated": "ISO8601 datetime"
}
```

### 4.6 Update Map

```http
PUT /robot/map/rooms/{room_id}
Content-Type: application/json

{
  "name": "Living Room",
  "surfaceType": "hardwood",
  "cleaningPriority": 1-10
}

Response:
{
  "status": "updated",
  "room": "Room object"
}
```

### 4.7 Get Cleaning History

```http
GET /robot/history?limit=10&offset=0

Response:
{
  "sessions": ["CleaningSession", "array"],
  "total": "integer",
  "hasMore": "boolean"
}
```

### 4.8 Schedule Cleaning

```http
POST /robot/schedule
Content-Type: application/json

{
  "enabled": true,
  "schedule": [
    {
      "day": "monday|tuesday|...",
      "time": "HH:MM",
      "mode": "auto",
      "rooms": ["array (optional)"]
    }
  ]
}

Response:
{
  "scheduleId": "UUID",
  "status": "created"
}
```

## 5. WebSocket API

### 5.1 Connection

```
ws://{robot_ip}:{port}/ws
```

### 5.2 Message Format

```json
{
  "type": "subscribe|unsubscribe|command|event",
  "channel": "status|sensors|map|logs",
  "data": {}
}
```

### 5.3 Subscribe to Updates

```json
{
  "type": "subscribe",
  "channel": "status",
  "interval": 1000
}
```

### 5.4 Real-time Events

```json
{
  "type": "event",
  "channel": "status",
  "timestamp": "ISO8601",
  "data": {
    "battery": 75,
    "position": {"x": 2.5, "y": 3.1},
    "mode": "cleaning"
  }
}
```

## 6. Error Handling

### 6.1 Error Response Format

```json
{
  "error": {
    "code": "ERROR_CODE",
    "message": "Human readable message",
    "details": {},
    "timestamp": "ISO8601 datetime"
  }
}
```

### 6.2 Standard Error Codes

- `ROBOT_BUSY`: Robot is currently cleaning
- `LOW_BATTERY`: Battery too low for operation
- `INVALID_COMMAND`: Command parameters invalid
- `NOT_CONNECTED`: Robot not connected to network
- `SENSOR_ERROR`: Sensor malfunction detected
- `NAVIGATION_ERROR`: Unable to navigate
- `DOCK_NOT_FOUND`: Cannot locate charging dock

## 7. Rate Limiting

- Default: 100 requests per minute per IP
- WebSocket: 10 messages per second
- Exceeding limits returns HTTP 429

## 8. Versioning

API version specified in URL: `/api/v1/`  
Breaking changes require new version number

---

© 2025 WIA · MIT License

## 9. Pagination and Filtering

All list endpoints support standard pagination:

```http
GET /robot/history?limit=20&offset=40&sort=startTime&order=desc

Query Parameters:
- limit: Items per page (default: 10, max: 100)
- offset: Number of items to skip
- sort: Field to sort by
- order: asc or desc
```

### Filter Examples

```http
GET /robot/history?mode=auto&minDuration=300
GET /robot/map/rooms?surfaceType=carpet
GET /robot/events?level=error&category=navigation
```

## 10. Batch Operations

```http
POST /robot/batch
Content-Type: application/json

{
  "operations": [
    {
      "method": "POST",
      "path": "/robot/clean",
      "body": {"mode": "spot"}
    },
    {
      "method": "PUT",
      "path": "/robot/config",
      "body": {"language": "ko"}
    }
  ]
}

Response:
{
  "results": [
    {"status": 200, "data": {}},
    {"status": 200, "data": {}}
  ]
}
```

## 11. Monitoring and Metrics

```http
GET /robot/metrics

Response:
{
  "cpu": 45.2,
  "memory": 62.8,
  "temperature": 38.5,
  "networkRxBytes": 1048576,
  "networkTxBytes": 524288,
  "uptime": 86400
}
```

## 12. Advanced Features

### 12.1 Remote Control

```http
POST /robot/control/joystick
Content-Type: application/json

{
  "linear": 0.5,    // m/s
  "angular": 0.3    // rad/s
}
```

### 12.2 Camera Stream

```http
GET /robot/camera/stream

Returns: MJPEG stream
Content-Type: multipart/x-mixed-replace
```

### 12.3 Map Export

```http
GET /robot/map/export?format=png|svg|json

Returns: Map in requested format
```

---

For complete examples and integration guides, see Phase 4 documentation.

© 2025 WIA · MIT License

---

## Annex A — Conformance Tier Matrix

WIA conformance for cleaning-robot is evaluated across three tiers:

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

- `wia-standards/standards/cleaning-robot/api/` — TypeScript SDK skeleton
- `wia-standards/standards/cleaning-robot/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/cleaning-robot/simulator/` — interactive browser-based simulator for the PHASE protocol

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
