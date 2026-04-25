# WIA-UNI-015: Peace Monitoring - Phase 2: API Interface

> **Version:** 1.0.0
> **Status:** Complete
> **Last Updated:** 2025-12-25

## 1. Overview

This specification defines the RESTful API interface for peace monitoring operations on the Korean Peninsula. The API enables real-time monitoring, verification requests, incident reporting, and coordination between North Korea, South Korea, and international observers.

## 2. Base URL & Authentication

### 2.1 API Endpoints

```
Production:  https://api.wia.org/peace-monitoring/v1
Staging:     https://staging-api.wia.org/peace-monitoring/v1
Development: https://dev-api.wia.org/peace-monitoring/v1
```

### 2.2 Authentication

All API requests require authentication using Bearer tokens:

```http
Authorization: Bearer {access_token}
```

**Token Types:**
- **Observer Token**: Read-only access to monitoring data
- **Reporter Token**: Submit monitoring events and sensor data
- **Verifier Token**: Create verification requests, access restricted data
- **Admin Token**: Full access including system configuration

**Token Acquisition:**

```http
POST /auth/token
Content-Type: application/json

{
  "organizationId": "UN_COMMAND",
  "credentials": {
    "clientId": "unc-observer-001",
    "clientSecret": "***",
    "scope": ["read:monitoring", "write:reports"]
  }
}

Response:
{
  "accessToken": "eyJhbGc...",
  "tokenType": "Bearer",
  "expiresIn": 3600,
  "scope": ["read:monitoring", "write:reports"]
}
```

## 3. Core API Endpoints

### 3.1 Monitoring Events

#### GET /monitoring/events

Retrieve monitoring events with filtering and pagination.

**Query Parameters:**
- `type`: MonitoringType (optional)
- `zone`: GeographicZone (optional)
- `startDate`: ISO 8601 timestamp (optional)
- `endDate`: ISO 8601 timestamp (optional)
- `status`: EventStatus (optional)
- `page`: Page number (default: 1)
- `limit`: Results per page (default: 50, max: 200)

**Request:**
```http
GET /monitoring/events?zone=DMZ&type=DMZ_SENSOR&startDate=2025-12-20T00:00:00Z&limit=20
Authorization: Bearer {token}
```

**Response:**
```json
{
  "data": [
    {
      "monitoringId": "MON-1735140600",
      "timestamp": "2025-12-25T14:30:00Z",
      "type": "DMZ_SENSOR",
      "zone": "DMZ",
      "coordinates": {
        "latitude": 38.123456,
        "longitude": 127.234567
      },
      "description": "Seismic activity detected",
      "status": "ACTIVE",
      "verificationRequired": true,
      "reportedBy": "DMZ-SENSOR-1234",
      "confidenceLevel": 0.95
    }
  ],
  "pagination": {
    "page": 1,
    "limit": 20,
    "total": 157,
    "totalPages": 8
  }
}
```

#### POST /monitoring/events

Submit a new monitoring event.

**Request:**
```http
POST /monitoring/events
Authorization: Bearer {token}
Content-Type: application/json

{
  "type": "TROOP_MOVEMENT",
  "zone": "SOUTH_KOREA",
  "coordinates": {
    "latitude": 37.456789,
    "longitude": 127.123456
  },
  "description": "Routine training exercise notification",
  "verificationRequired": false,
  "metadata": {
    "unitSize": 500,
    "duration": 48,
    "purpose": "TRAINING"
  }
}
```

**Response:**
```json
{
  "monitoringId": "MON-1735141200",
  "status": "CREATED",
  "timestamp": "2025-12-25T15:00:00Z",
  "message": "Monitoring event created successfully"
}
```

#### GET /monitoring/events/{monitoringId}

Retrieve specific monitoring event details.

**Response:**
```json
{
  "monitoringId": "MON-1735140600",
  "timestamp": "2025-12-25T14:30:00Z",
  "type": "DMZ_SENSOR",
  "zone": "DMZ",
  "coordinates": {
    "latitude": 38.123456,
    "longitude": 127.234567,
    "altitude": 45
  },
  "description": "Seismic activity detected - possible underground test",
  "status": "UNDER_INVESTIGATION",
  "verificationRequired": true,
  "reportedBy": "DMZ-SENSOR-1234",
  "confidenceLevel": 0.95,
  "attachments": [
    {
      "attachmentId": "ATT-001",
      "type": "SENSOR_DATA",
      "filename": "seismic-reading-2025-12-25.json",
      "mimeType": "application/json"
    }
  ],
  "timeline": [
    {
      "timestamp": "2025-12-25T14:30:00Z",
      "action": "DETECTED",
      "actor": "DMZ-SENSOR-1234"
    },
    {
      "timestamp": "2025-12-25T14:32:00Z",
      "action": "NOTIFIED_OBSERVERS",
      "actor": "SYSTEM"
    }
  ]
}
```

### 3.2 Arms Inventory

#### GET /arms/inventory

Retrieve arms inventory declarations.

**Query Parameters:**
- `party`: "NORTH_KOREA" | "SOUTH_KOREA"
- `category`: Weapon category
- `verificationStatus`: VerificationStatus
- `asOfDate`: ISO 8601 date

**Response:**
```json
{
  "data": [
    {
      "inventoryId": "INV-2025-Q4-NK",
      "declaringParty": "NORTH_KOREA",
      "timestamp": "2025-10-01T00:00:00Z",
      "location": {
        "name": "Northern Region Storage Facility",
        "zone": "NORTH_KOREA"
      },
      "categories": [
        {
          "category": "ARTILLERY",
          "subcategory": "HEAVY_ARTILLERY",
          "quantity": 450,
          "specifications": {
            "model": "M-1978 KOKSAN",
            "caliber": "170mm",
            "range": 40
          }
        }
      ],
      "totalItems": 1247,
      "verificationStatus": "VERIFIED",
      "verifiedBy": ["UNC", "NNSC"],
      "nextInspectionDue": "2026-01-15"
    }
  ]
}
```

#### POST /arms/inventory

Submit arms inventory declaration.

**Request:**
```json
{
  "declaringParty": "SOUTH_KOREA",
  "location": {
    "name": "Central Arms Depot",
    "coordinates": {
      "latitude": 37.123456,
      "longitude": 127.456789
    }
  },
  "categories": [
    {
      "category": "TANKS",
      "subcategory": "MAIN_BATTLE_TANK",
      "quantity": 120,
      "specifications": {
        "model": "K2 Black Panther",
        "operational": true
      }
    }
  ]
}
```

### 3.3 Verification Requests

#### POST /verification/requests

Create a verification request.

**Request:**
```json
{
  "requestType": "ARMS_INSPECTION",
  "targetLocation": {
    "name": "Yongbyon Nuclear Complex",
    "coordinates": {
      "latitude": 39.802222,
      "longitude": 125.754722
    },
    "zone": "NORTH_KOREA"
  },
  "proposedDate": "2026-01-15T10:00:00Z",
  "requestedBy": "IAEA",
  "respondent": "NORTH_KOREA",
  "observersRequired": ["UNC", "NNSC"],
  "justification": "Routine IAEA safeguards inspection per NPT obligations",
  "priority": "HIGH"
}
```

**Response:**
```json
{
  "requestId": "VRQ-1735141800",
  "status": "SUBMITTED",
  "expectedResponseBy": "2025-12-28T15:00:00Z",
  "trackingUrl": "/verification/requests/VRQ-1735141800"
}
```

#### GET /verification/requests/{requestId}

Get verification request status.

**Response:**
```json
{
  "requestId": "VRQ-1735141800",
  "status": "ACCEPTED",
  "requestType": "ARMS_INSPECTION",
  "requestedBy": "IAEA",
  "respondent": "NORTH_KOREA",
  "proposedDate": "2026-01-15T10:00:00Z",
  "confirmedDate": "2026-01-15T10:00:00Z",
  "observers": ["UNC", "NNSC", "IAEA"],
  "accessConditions": [
    "Valid credentials required",
    "72-hour advance notice confirmed",
    "Escort protocol agreed"
  ],
  "statusHistory": [
    {
      "timestamp": "2025-12-25T15:30:00Z",
      "status": "SUBMITTED",
      "note": "Request submitted by IAEA"
    },
    {
      "timestamp": "2025-12-26T10:00:00Z",
      "status": "UNDER_REVIEW",
      "note": "Under review by DPRK authorities"
    },
    {
      "timestamp": "2025-12-27T14:00:00Z",
      "status": "ACCEPTED",
      "note": "Inspection approved, conditions set"
    }
  ]
}
```

### 3.4 DMZ Sensors

#### GET /sensors/readings

Retrieve sensor data.

**Query Parameters:**
- `sensorId`: Specific sensor ID
- `sensorType`: SensorType enum
- `zone`: Geographic zone
- `alertLevel`: Filter by alert level
- `startTime`: ISO 8601 timestamp
- `endTime`: ISO 8601 timestamp

**Response:**
```json
{
  "data": [
    {
      "sensorId": "DMZ-SEISMIC-045",
      "sensorType": "SEISMIC",
      "location": {
        "latitude": 38.234567,
        "longitude": 127.345678
      },
      "timestamp": "2025-12-25T14:30:00Z",
      "readings": [
        {
          "timestamp": "2025-12-25T14:30:00Z",
          "value": 2.4,
          "unit": "RICHTER",
          "threshold": 2.0,
          "thresholdExceeded": true,
          "confidence": 0.95
        }
      ],
      "alertLevel": "MEDIUM"
    }
  ]
}
```

#### POST /sensors/readings

Submit sensor data (for authorized sensor systems).

### 3.5 Confidence Building Measures

#### GET /cbm/activities

List confidence-building measure activities.

**Response:**
```json
{
  "data": [
    {
      "cbmId": "CBM-2025-12-001",
      "type": "MILITARY_HOTLINE_TEST",
      "initiatingParty": "JOINT",
      "timestamp": "2025-12-25T09:00:00Z",
      "status": "COMPLETED",
      "participants": [
        {
          "party": "NORTH_KOREA",
          "role": "PARTICIPANT"
        },
        {
          "party": "SOUTH_KOREA",
          "role": "PARTICIPANT"
        }
      ],
      "outcomes": [
        {
          "timestamp": "2025-12-25T09:30:00Z",
          "description": "Hotline test successful, all channels operational",
          "followUpRequired": false
        }
      ]
    }
  ]
}
```

## 4. WebSocket API (Real-time)

### 4.1 Connection

```javascript
const ws = new WebSocket('wss://api.wia.org/peace-monitoring/v1/stream');

ws.onopen = () => {
  ws.send(JSON.stringify({
    type: 'authenticate',
    token: 'Bearer {token}'
  }));

  ws.send(JSON.stringify({
    type: 'subscribe',
    channels: ['dmz-sensors', 'critical-alerts']
  }));
};
```

### 4.2 Event Streaming

**Server Messages:**
```json
{
  "channel": "critical-alerts",
  "eventType": "SENSOR_ALERT",
  "timestamp": "2025-12-25T14:30:00Z",
  "data": {
    "sensorId": "DMZ-SEISMIC-045",
    "alertLevel": "CRITICAL",
    "location": {
      "latitude": 38.234567,
      "longitude": 127.345678
    },
    "description": "Seismic event detected - magnitude 4.2"
  }
}
```

## 5. Error Handling

### 5.1 Error Response Format

```json
{
  "error": {
    "code": "UNAUTHORIZED",
    "message": "Invalid or expired authentication token",
    "timestamp": "2025-12-25T15:00:00Z",
    "requestId": "req-abc123",
    "details": {
      "reason": "TOKEN_EXPIRED",
      "expiryTime": "2025-12-25T14:00:00Z"
    }
  }
}
```

### 5.2 Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| UNAUTHORIZED | 401 | Invalid authentication |
| FORBIDDEN | 403 | Insufficient permissions |
| NOT_FOUND | 404 | Resource not found |
| VALIDATION_ERROR | 400 | Invalid request data |
| RATE_LIMIT_EXCEEDED | 429 | Too many requests |
| INTERNAL_ERROR | 500 | Server error |
| SERVICE_UNAVAILABLE | 503 | Temporary service outage |

## 6. Rate Limiting

- **Observer Tier**: 1000 requests/hour
- **Reporter Tier**: 5000 requests/hour
- **Verifier Tier**: 10000 requests/hour
- **International Organizations**: Unlimited

Headers:
```
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 847
X-RateLimit-Reset: 2025-12-25T16:00:00Z
```

## 7. Versioning

API versions are specified in the URL path: `/v1`, `/v2`, etc.

## 8. SDK Support

Official SDKs available:
- TypeScript/JavaScript: `@wia/peace-monitoring-sdk`
- Python: `wia-peace-monitoring`
- Java: `org.wia.peace-monitoring`
- Go: `github.com/wia/peace-monitoring-go`

---

**弘益人間 (Benefit All Humanity)**
© 2025 WIA - World Certification Industry Association
Licensed under MIT License

---

## Annex A — Conformance Tier Matrix

WIA conformance for WIA-SOC-PEACE-001 (Peace Monitoring) is evaluated across three tiers, applied to incident telemetry · ceasefire verification · civilian-impact reporting · audit chain:

| Tier | Scope | Mandatory artifacts | Audit cadence |
|------|-------|--------------------|----------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | OpenAPI 3.0 contract, JSON Schema validation report, security threat model | None (annual self-review recommended) |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Tier 1 artifacts + signed third-party assessor report against this PHASE | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Tier 2 artifacts + WIA accreditation, ISO/IEC 17065:2012 conformity assessment, evidence retention ≥ 7 years | Every 12 months |

Implementations MUST clearly disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page. Tier downgrade events MUST be reported to the WIA registry within 30 days.

---

## Annex B — Cross-Walk to International Standards

The PHASE specification reuses or normatively references the following published standards. Implementers SHOULD review the listed standards alongside this PHASE document; where a conflict exists, the more specific WIA requirement governs unless explicitly superseded by a binding national regulation.

- ISO 18091:2019 — Quality management for local government (governance baseline)
- ISO 37120:2018 — Sustainable cities and communities indicators
- ISO/IEC 27001:2022 — Information security management
- IETF RFC 9457 — Problem details for HTTP APIs (incident reporting)
- W3C PROV-DM — provenance data model for monitoring records

This cross-walk is informative only. WIA does not republish the referenced documents; readers MUST obtain authoritative copies from the issuing body. Cross-walk entries are reviewed at every minor version of this PHASE.

---

## Annex C — Reference Implementations and Test Vectors

### C.1 Reference Implementations

WIA does not require implementers to use a particular library, but maintains pointers to the canonical reference implementation directories under the WIA-Official GitHub organization:

- `wia-standards/standards/peace-monitoring/api/` — TypeScript SDK skeleton
- `wia-standards/standards/peace-monitoring/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/peace-monitoring/simulator/` — interactive browser-based simulator for the PHASE protocol

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
