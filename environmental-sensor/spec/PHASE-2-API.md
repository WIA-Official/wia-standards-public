# WIA Environmental Sensor Standard - Phase 2: API Interface
## Version 1.0.0 | WIA-ENE-027-PHASE-2

**Status:** Final
**Published:** 2025-01-01
**Organization:** World Certification Industry Association (WIA)
**License:** MIT License

---

## 1. Overview

Phase 2 of the WIA Environmental Sensor Standard defines standardized RESTful API interfaces for accessing environmental sensor data, managing devices, and enabling real-time data streaming. This specification ensures consistent programmatic access to sensor data regardless of platform or vendor.

### 1.1 Scope

Phase 2 covers:
- RESTful API design principles and URL structures
- Request/response formats and content negotiation
- Authentication and authorization mechanisms
- Query parameters and filtering capabilities
- Real-time streaming interfaces (WebSocket, SSE)
- Error handling and status codes
- Rate limiting and pagination

### 1.2 Prerequisites

- Implementations MUST comply with Phase 1 (Data Format)
- HTTP/1.1 or HTTP/2 protocol support
- TLS 1.3 for secure communications
- JSON as primary content type

---

## 2. API Design Principles

### 2.1 RESTful Architecture

The API follows REST (Representational State Transfer) principles:
- Resource-oriented URLs
- Standard HTTP methods (GET, POST, PUT, DELETE)
- Stateless operations
- Cacheable responses where appropriate
- Hypermedia links for navigation (HATEOAS)

### 2.2 URL Structure

Base URL pattern:
```
https://{host}/api/v{version}/{resource}
```

Example:
```
https://api.example.com/api/v1/sensors
```

### 2.3 Versioning

API version is included in the URL path (`/api/v1/`). Major version changes indicate breaking changes. Minor and patch versions maintain backward compatibility.

---

## 3. Core Endpoints

### 3.1 Sensor Discovery

#### List All Sensors

```http
GET /api/v1/sensors
```

**Query Parameters:**
- `type`: Filter by sensor type (air_quality, water_quality, soil, etc.)
- `location`: Geographic bounding box `lat1,lon1,lat2,lon2`
- `status`: Filter by status (active, inactive, maintenance)
- `limit`: Maximum results (default 100, max 1000)
- `offset`: Pagination offset

**Response (200 OK):**
```json
{
  "total": 1547,
  "count": 100,
  "offset": 0,
  "sensors": [
    {
      "deviceId": "ENV-AIR-001",
      "type": "air_quality",
      "location": {"latitude": 37.5665, "longitude": 126.9780},
      "status": "active",
      "lastUpdate": "2024-12-26T10:30:00Z",
      "capabilities": ["pm2_5", "pm10", "temperature"]
    }
  ],
  "links": {
    "next": "/api/v1/sensors?offset=100",
    "self": "/api/v1/sensors"
  }
}
```

#### Get Sensor Details

```http
GET /api/v1/sensors/{deviceId}
```

**Response (200 OK):**
```json
{
  "deviceId": "ENV-AIR-001",
  "type": "air_quality",
  "manufacturer": "AirSense Corp",
  "model": "AS-3000",
  "firmware": "v2.4.1",
  "location": {
    "latitude": 37.5665,
    "longitude": 126.9780,
    "altitude": 38.5
  },
  "status": "active",
  "lastCalibration": "2024-11-15T09:00:00Z",
  "capabilities": {
    "pm2_5": {"range": [0, 500], "unit": "μg/m³", "accuracy": 2.0},
    "temperature": {"range": [-40, 85], "unit": "°C", "accuracy": 0.3}
  }
}
```

### 3.2 Data Retrieval

#### Latest Reading

```http
GET /api/v1/sensors/{deviceId}/data/latest
```

**Response (200 OK):**
WIA Phase 1 compliant JSON data structure

#### Historical Data

```http
GET /api/v1/sensors/{deviceId}/data
```

**Query Parameters:**
- `start`: ISO 8601 start timestamp (required)
- `end`: ISO 8601 end timestamp (required)
- `parameters`: Comma-separated parameter list (e.g., `pm2_5,temperature`)
- `aggregation`: none | hourly | daily | weekly | monthly
- `format`: json | csv | xml

**Response (200 OK):**
```json
{
  "deviceId": "ENV-AIR-001",
  "start": "2024-12-25T00:00:00Z",
  "end": "2024-12-26T00:00:00Z",
  "aggregation": "hourly",
  "data": [
    {
      "timestamp": "2024-12-25T00:00:00Z",
      "pm2_5": {"mean": 12.5, "min": 8.2, "max": 18.3, "count": 60}
    }
  ]
}
```

#### Bulk Data Retrieval

```http
POST /api/v1/data/bulk
Content-Type: application/json

{
  "sensorIds": ["ENV-AIR-001", "ENV-AIR-002"],
  "start": "2024-12-26T00:00:00Z",
  "end": "2024-12-26T12:00:00Z",
  "parameters": ["pm2_5", "temperature"]
}
```

### 3.3 Device Management

#### Register Sensor

```http
POST /api/v1/sensors
Content-Type: application/json

{
  "deviceId": "ENV-AIR-NEW",
  "type": "air_quality",
  "manufacturer": "AirSense Corp",
  "model": "AS-3000",
  "location": {"latitude": 37.5665, "longitude": 126.9780}
}
```

**Response (201 Created):**
```json
{
  "deviceId": "ENV-AIR-NEW",
  "status": "registered",
  "registrationDate": "2024-12-26T10:30:00Z"
}
```

#### Update Sensor Configuration

```http
PUT /api/v1/sensors/{deviceId}/config
Content-Type: application/json

{
  "samplingInterval": 60,
  "reportingInterval": 300,
  "parameters": ["pm2_5", "pm10", "temperature"]
}
```

---

## 4. Real-Time Streaming

### 4.1 WebSocket Interface

**Connection:**
```
ws://api.example.com/v1/stream
```

**Subscribe Message:**
```json
{
  "action": "subscribe",
  "sensors": ["ENV-AIR-001", "ENV-AIR-002"],
  "parameters": ["pm2_5", "temperature"]
}
```

**Data Messages:**
```json
{
  "type": "measurement",
  "deviceId": "ENV-AIR-001",
  "timestamp": "2024-12-26T10:30:00Z",
  "readings": {
    "pm2_5": {"value": 15.3, "unit": "μg/m³"}
  }
}
```

### 4.2 Server-Sent Events (SSE)

```http
GET /api/v1/sensors/{deviceId}/stream
Accept: text/event-stream
```

**Response:**
```
event: measurement
data: {"timestamp":"2024-12-26T10:30:00Z","pm2_5":15.3}

event: heartbeat
data: {"timestamp":"2024-12-26T10:35:00Z"}
```

---

## 5. Authentication and Authorization

### 5.1 API Keys

```http
GET /api/v1/sensors
Authorization: Bearer YOUR_API_KEY
```

### 5.2 OAuth 2.0

**Authorization Code Flow:**

1. Redirect to authorization endpoint
2. Exchange code for access token
3. Use token in requests:

```http
GET /api/v1/sensors
Authorization: Bearer {access_token}
```

### 5.3 JWT Tokens

```http
GET /api/v1/sensors
Authorization: Bearer {jwt_token}
```

### 5.4 Role-Based Access Control

| Role | Permissions |
|------|-------------|
| Public Viewer | Read public sensor data |
| Data Consumer | Read all sensor data |
| Sensor Operator | Read data, update config |
| Administrator | Full access |

---

## 6. Error Handling

### 6.1 HTTP Status Codes

- `200 OK`: Successful request
- `201 Created`: Resource created
- `400 Bad Request`: Invalid request
- `401 Unauthorized`: Authentication required
- `403 Forbidden`: Insufficient permissions
- `404 Not Found`: Resource not found
- `429 Too Many Requests`: Rate limit exceeded
- `500 Internal Server Error`: Server error

### 6.2 Error Response Format

```json
{
  "error": {
    "code": "SENSOR_NOT_FOUND",
    "message": "Sensor with ID 'ENV-AIR-999' not found",
    "details": {
      "requestedId": "ENV-AIR-999"
    },
    "timestamp": "2024-12-26T10:30:00Z",
    "requestId": "req_abc123"
  }
}
```

---

## 7. Rate Limiting

### 7.1 Rate Limit Headers

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 987
X-RateLimit-Reset: 1704067200
```

### 7.2 Rate Limit Tiers

| Tier | Requests/Hour | Burst Limit |
|------|---------------|-------------|
| Free | 1,000 | 100/minute |
| Standard | 10,000 | 500/minute |
| Professional | 100,000 | 2,000/minute |
| Enterprise | Unlimited | Custom |

---

## 8. Pagination

### 8.1 Offset-Based

```http
GET /api/v1/sensors?limit=100&offset=200
```

Response includes links:
```json
{
  "links": {
    "first": "/api/v1/sensors?limit=100&offset=0",
    "prev": "/api/v1/sensors?limit=100&offset=100",
    "self": "/api/v1/sensors?limit=100&offset=200",
    "next": "/api/v1/sensors?limit=100&offset=300"
  }
}
```

### 8.2 Cursor-Based

```http
GET /api/v1/sensors/{deviceId}/data?limit=100&cursor={cursor}
```

---

## 9. Content Negotiation

Supported content types:
- `application/json` (default)
- `application/xml`
- `text/csv`
- `application/cbor`

Request format:
```http
GET /api/v1/sensors
Accept: application/json
```

---

## 10. Compliance Requirements

Phase 2 compliance requires:
- Implementation of all core endpoints (Sections 3.1, 3.2)
- At least one authentication method (Section 5)
- Proper error handling (Section 6)
- Rate limiting (Section 7)
- Phase 1 data format compliance

Optional features:
- Real-time streaming (Section 4)
- Device management endpoints (Section 3.3)
- Multiple authentication methods

---

**弘益人間 (Hongik Ingan) - Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 MIT License*

---

## Annex A — Conformance Tier Matrix

WIA conformance for environmental-sensor is evaluated across three tiers:

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

- `wia-standards/standards/environmental-sensor/api/` — TypeScript SDK skeleton
- `wia-standards/standards/environmental-sensor/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/environmental-sensor/simulator/` — interactive browser-based simulator for the PHASE protocol

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
