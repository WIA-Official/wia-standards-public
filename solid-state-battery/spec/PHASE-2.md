# WIA-AUTO-028: Solid-State Battery Standard
## Phase 2: API Interface

**Version:** 1.0.0
**Status:** Active
**Last Updated:** 2025-12-27
**Category:** AUTO / Mobility

---

## 1. Overview

Phase 2 specifies RESTful HTTP APIs for programmatic interaction with solid-state battery systems. These APIs build upon Phase 1 data formats to provide real-time monitoring, control, diagnostics, and integration capabilities.

### 1.1 Scope

Phase 2 covers:
- REST API endpoint specifications
- Authentication and authorization mechanisms
- Request/response formats
- Error handling
- Rate limiting
- Security requirements
- WebSocket streaming protocols

### 1.2 Design Principles

- **RESTful:** Standard HTTP methods (GET, POST, PUT, DELETE)
- **Stateless:** Each request contains all necessary information
- **Versioned:** API version in URL path for backward compatibility
- **Secured:** All endpoints require authentication
- **Documented:** OpenAPI 3.0 specifications provided
- **Standard Codes:** HTTP status codes follow RFC 7231

---

## 2. Base URL and Versioning

### 2.1 URL Structure

```
https://{host}/api/v1/{resource}
```

Components:
- **host:** Battery system hostname or IP
- **v1:** API version (major version only)
- **resource:** Specific resource endpoint

### 2.2 Version Policy

- **Minor updates:** Backward compatible, no version change
- **Major updates:** Breaking changes, new version number
- **Deprecation:** 12-month notice before removal
- **Support:** Minimum 2 versions supported concurrently

---

## 3. Core Endpoints

### 3.1 Battery Information

#### GET /api/v1/battery/specification

Retrieve complete battery specification (Phase 1 format).

**Request:**
```http
GET /api/v1/battery/specification HTTP/1.1
Host: battery.example.com
Authorization: Bearer {token}
Accept: application/json
```

**Response (200 OK):**
```json
{
  "@context": "https://wiastandards.com/contexts/auto-028/v1",
  "type": "SolidStateBatterySpecification",
  "id": "did:wia:battery:ssb-2025-12345",
  ...
}
```

**Caching:** Public, max-age=3600 (1 hour)

#### GET /api/v1/battery/metadata

Retrieve battery metadata (manufacturer, model, serial).

**Response:** Subset of specification focusing on identification.

#### GET /api/v1/battery/certifications

List all certifications and compliance records.

**Response:**
```json
{
  "certifications": [
    {
      "standard": "WIA-AUTO-028",
      "level": 3,
      "date": "2025-01-10",
      "expirationDate": "2028-01-10",
      "certificationBody": "TÜV SÜD",
      "certificateNumber": "WIA-AUTO-028-2025-001"
    }
  ]
}
```

### 3.2 Status and Monitoring

#### GET /api/v1/battery/status

Current battery state (SOC, SOH, voltage, current, temperature).

**Request:**
```http
GET /api/v1/battery/status HTTP/1.1
Host: battery.example.com
Authorization: Bearer {token}
Accept: application/json
```

**Response (200 OK):**
```json
{
  "@context": "https://wiastandards.com/contexts/auto-028/v1",
  "type": "BatteryStatus",
  "batteryId": "did:wia:battery:ssb-2025-12345",
  "timestamp": "2025-12-27T14:30:45.123Z",
  "stateOfCharge": {"value": 67.5, "unit": "%", "confidence": 0.98},
  "stateOfHealth": {"value": 94.2, "unit": "%"},
  ...
}
```

**Caching:** no-cache (always fetch fresh data)
**Update Frequency:** 1-10 Hz

#### GET /api/v1/battery/telemetry

Detailed multi-point sensor measurements.

**Query Parameters:**
- `points`: Comma-separated list of measurement points
- `detail`: Level of detail (summary|full)

**Response:** Extended status with cell-level measurements.

#### GET /api/v1/battery/health

Health metrics and degradation indicators.

**Response:**
```json
{
  "timestamp": "2025-12-27T14:30:00Z",
  "soh": {"value": 94.2, "unit": "%"},
  "capacityFade": {"value": 5.8, "unit": "%"},
  "impedanceIncrease": {"value": 12.3, "unit": "%"},
  "predictedRemainingLife": {"value": 2450, "unit": "cycles"},
  "degradationRate": {"value": 0.58, "unit": "%/1000cycles"}
}
```

### 3.3 Historical Data

#### GET /api/v1/battery/history

Time-series historical data.

**Query Parameters:**
- `start`: ISO 8601 timestamp
- `end`: ISO 8601 timestamp
- `interval`: Sampling interval in seconds
- `metrics`: Comma-separated metrics (soc,soh,voltage,current,temp)
- `limit`: Maximum data points (default: 1000, max: 10000)

**Response:**
```json
{
  "timeRange": {
    "start": "2025-12-20T00:00:00Z",
    "end": "2025-12-27T00:00:00Z"
  },
  "samplingInterval": {"value": 60, "unit": "seconds"},
  "dataPoints": [
    {
      "timestamp": "2025-12-20T00:01:00Z",
      "soc": 85.3,
      "soh": 94.2,
      "voltage": 392.5,
      "current": -12.3,
      "tempAvg": 25.2
    },
    ...
  ]
}
```

#### GET /api/v1/battery/statistics

Statistical summaries over time period.

**Query Parameters:**
- `start`, `end`: Time range
- `aggregation`: Aggregation method (avg|min|max|sum)

### 3.4 Diagnostics

#### GET /api/v1/battery/diagnostics

Current fault codes and warnings.

**Response:**
```json
{
  "timestamp": "2025-12-27T14:30:00Z",
  "faults": [
    {
      "code": "SSB-T001",
      "severity": "warning",
      "timestamp": "2025-12-27T14:25:00Z",
      "description": "Cell temperature exceeds warning threshold",
      "active": true
    }
  ],
  "warnings": [...],
  "summary": {
    "totalFaults": 1,
    "criticalFaults": 0,
    "warnings": 2
  }
}
```

#### GET /api/v1/battery/diagnostics/history

Historical fault log with optional filtering.

#### POST /api/v1/battery/diagnostics/{code}/clear

Clear a fault code (requires Technician authorization).

**Request:**
```json
{
  "reason": "Condition resolved, temperature returned to normal",
  "technicianId": "tech-12345"
}
```

### 3.5 Charging Control

#### GET /api/v1/battery/charging/status

Current charging session status.

**Response:**
```json
{
  "sessionId": "session-20251227-143045",
  "status": "charging",
  "startTime": "2025-12-27T14:30:45Z",
  "currentSOC": 67.5,
  "targetSOC": 80,
  "chargingPower": {"value": 142.3, "unit": "kW"},
  "estimatedTimeRemaining": {"value": 8.5, "unit": "min"},
  "energyDelivered": {"value": 5.2, "unit": "kWh"}
}
```

#### POST /api/v1/battery/charging/start

Initiate charging session.

**Request:**
```json
{
  "targetSOC": 80,
  "maxPower": 150,
  "maxCurrent": 375,
  "priority": "speed",
  "schedule": null
}
```

**Response (201 Created):**
```json
{
  "sessionId": "session-20251227-143045",
  "status": "initiated",
  "estimatedDuration": {"value": 15, "unit": "min"}
}
```

#### POST /api/v1/battery/charging/stop

Terminate charging session.

---

## 4. Authentication and Authorization

### 4.1 Supported Methods

1. **OAuth 2.0:** For cloud services, mobile apps
2. **API Keys:** For system-to-system communication
3. **Mutual TLS (mTLS):** For high-security applications
4. **DID-based:** For WIA ecosystem integration

### 4.2 OAuth 2.0 Flow

```
1. Client requests authorization
2. User authenticates and grants permission
3. Authorization server issues access token
4. Client uses token for API requests
5. Token expires (configurable: 1-24 hours)
6. Client uses refresh token for new access token
```

**Token Request:**
```http
POST /oauth/token HTTP/1.1
Host: battery.example.com
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials
&client_id={client_id}
&client_secret={client_secret}
&scope=battery:read battery:control
```

**Response:**
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "battery:read battery:control"
}
```

### 4.3 API Key Authentication

**Request Header:**
```http
X-API-Key: wia_ssb_a1b2c3d4e5f6g7h8i9j0k1l2m3n4o5p6
```

### 4.4 Permission Scopes

| Scope | Description | Endpoints |
|-------|-------------|-----------|
| battery:read | Read battery data | GET /specification, /status, /telemetry |
| battery:monitor | Access diagnostics | GET /diagnostics, /health |
| battery:control | Control operations | POST /charging/start, /charging/stop |
| battery:admin | Administrative access | POST /diagnostics/clear, configuration |

---

## 5. Rate Limiting

### 5.1 Limits by Endpoint Category

| Category | Rate Limit | Quota (per day) | Burst |
|----------|------------|-----------------|-------|
| Real-time Status | 10 req/s | Unlimited | 20 req/s for 10s |
| Historical Data | 5 req/min | 1000 requests | No |
| Charging Control | 10 req/min | 500 requests | No |
| Configuration | 1 req/min | 100 requests | No |

### 5.2 Rate Limit Headers

**Response Headers:**
```http
X-RateLimit-Limit: 600
X-RateLimit-Remaining: 573
X-RateLimit-Reset: 1703692845
```

**429 Too Many Requests:**
```json
{
  "error": {
    "code": "RATE_LIMIT_EXCEEDED",
    "message": "Rate limit exceeded. Please retry after 45 seconds.",
    "retryAfter": 45
  }
}
```

---

## 6. Error Handling

### 6.1 Standard Error Response

```json
{
  "error": {
    "code": "ERROR_CODE",
    "message": "Human-readable error message",
    "field": "fieldName",
    "value": "invalidValue",
    "timestamp": "2025-12-27T14:30:45Z"
  }
}
```

### 6.2 HTTP Status Codes

| Code | Meaning | Usage |
|------|---------|-------|
| 200 | OK | Successful GET |
| 201 | Created | Successful POST creating resource |
| 204 | No Content | Successful DELETE |
| 400 | Bad Request | Invalid request parameters |
| 401 | Unauthorized | Authentication required/failed |
| 403 | Forbidden | Insufficient permissions |
| 404 | Not Found | Resource does not exist |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Server Error | Server-side error |
| 503 | Service Unavailable | Temporary unavailability |

---

## 7. WebSocket Streaming

### 7.1 Connection

```javascript
const ws = new WebSocket('wss://battery.example.com/api/v1/battery/stream?metrics=soc,voltage,current');

ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  console.log('Battery update:', data);
};
```

### 7.2 Message Format

**Server → Client:**
```json
{
  "type": "telemetry-update",
  "timestamp": "2025-12-27T14:30:45.123Z",
  "data": {
    "soc": 67.5,
    "voltage": 386.4,
    "current": -45.2,
    "tempAvg": 26.1
  }
}
```

---

## 8. Compliance Requirements

### 8.1 Level 1

- Implement Battery Information endpoints
- Implement basic Status endpoint
- Support one authentication method
- Implement error handling

### 8.2 Level 2

- All Level 1 requirements
- Implement Historical Data endpoints
- Implement Diagnostics endpoints
- Support rate limiting
- Provide OpenAPI specification

### 8.3 Level 3

- All Level 2 requirements
- Implement Charging Control endpoints
- Support WebSocket streaming
- Implement OAuth 2.0
- Support DID-based authentication

---

**弘益人間 (Hongik Ingan) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 MIT License*

---

## Annex A — Conformance Tier Matrix

WIA conformance for WIA-MFG-SSB (Solid-State Battery) is evaluated across three tiers, applied to cell chemistry metadata · state-of-health telemetry · cycle test reporting · safety thresholds:

| Tier | Scope | Mandatory artifacts | Audit cadence |
|------|-------|--------------------|----------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | OpenAPI 3.0 contract, JSON Schema validation report, security threat model | None (annual self-review recommended) |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Tier 1 artifacts + signed third-party assessor report against this PHASE | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Tier 2 artifacts + WIA accreditation, ISO/IEC 17065:2012 conformity assessment, evidence retention ≥ 7 years | Every 12 months |

Implementations MUST clearly disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page. Tier downgrade events MUST be reported to the WIA registry within 30 days.

---

## Annex B — Cross-Walk to International Standards

The PHASE specification reuses or normatively references the following published standards. Implementers SHOULD review the listed standards alongside this PHASE document; where a conflict exists, the more specific WIA requirement governs unless explicitly superseded by a binding national regulation.

- IEC 62660-1:2018 — Secondary lithium-ion cells for propulsion of electric road vehicles
- IEC 62660-2:2018 — Reliability and abuse testing
- IEC 62619:2022 — Safety requirements for secondary lithium batteries for industrial applications
- ISO 12405-4:2018 — Test specification for lithium-ion traction battery packs and systems
- UN 38.3 — Manual of Tests and Criteria, Section 38.3 (transport of lithium batteries)

This cross-walk is informative only. WIA does not republish the referenced documents; readers MUST obtain authoritative copies from the issuing body. Cross-walk entries are reviewed at every minor version of this PHASE.

---

## Annex C — Reference Implementations and Test Vectors

### C.1 Reference Implementations

WIA does not require implementers to use a particular library, but maintains pointers to the canonical reference implementation directories under the WIA-Official GitHub organization:

- `wia-standards/standards/solid-state-battery/api/` — TypeScript SDK skeleton
- `wia-standards/standards/solid-state-battery/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/solid-state-battery/simulator/` — interactive browser-based simulator for the PHASE protocol

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
