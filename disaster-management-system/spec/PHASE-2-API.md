# WIA-SOC-006 Phase 2: API Specification

**Version:** 1.0.0  
**Status:** Approved  
**Last Updated:** 2025-12-26

弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. Overview

Phase 2 defines the RESTful API and real-time communication interfaces for disaster management systems. This specification enables standardized interaction between emergency management software, mobile apps, and inter-agency systems.

## 2. Base API Structure

### 2.1 Base URL

```
https://api.disaster-management.org/v1
```

### 2.2 Authentication

All API requests MUST include authentication:

**Methods Supported:**
- OAuth 2.0 (RECOMMENDED)
- JWT Bearer Tokens
- API Keys (for system-to-system)

**Header Format:**
```
Authorization: Bearer <token>
X-API-Key: <api-key>  (alternative)
```

### 2.3 Common Headers

```
Content-Type: application/json
Accept: application/json
X-Request-ID: <UUID>  (for tracing)
X-Agency-ID: <agency-identifier>
```

## 3. Event Management API

### 3.1 Create Disaster Event

```
POST /events
```

**Request Body:**
```json
{
  "disasterType": "tornado",
  "severity": 8,
  "status": "warning",
  "affectedArea": {
    "type": "Polygon",
    "coordinates": [[[...], ...]]
  },
  "estimatedAffectedPopulation": 50000,
  "description": "Category F4 tornado approaching urban area"
}
```

**Response: 201 Created**
```json
{
  "eventId": "EVT-2025-001",
  "status": "created",
  "createdAt": "2025-12-26T14:30:00Z",
  "event": { /* full event object */ }
}
```

### 3.2 Get Event Details

```
GET /events/{eventId}
```

**Response: 200 OK**
```json
{
  "eventId": "EVT-2025-001",
  "disasterType": "tornado",
  "severity": 8,
  "status": "warning",
  "affectedArea": {...},
  "startTime": "2025-12-26T14:00:00Z",
  "estimatedAffectedPopulation": 50000,
  "resources": [...],
  "alerts": [...],
  "metadata": {...}
}
```

### 3.3 Update Event

```
PATCH /events/{eventId}
```

**Request Body:**
```json
{
  "status": "critical",
  "confirmedCasualties": 12,
  "estimatedAffectedPopulation": 75000
}
```

**Response: 200 OK**

### 3.4 List Active Events

```
GET /events?status=active&type=tornado&severity_min=5
```

**Query Parameters:**
- `status`: filter by status
- `type`: disaster type
- `severity_min/max`: severity range
- `bbox`: bounding box (west,south,east,north)
- `limit`: max results (default 50)
- `offset`: pagination offset

**Response: 200 OK**
```json
{
  "total": 15,
  "limit": 50,
  "offset": 0,
  "events": [...]
}
```

## 4. Alert Management API

### 4.1 Issue Alert

```
POST /alerts
```

**Request Body:**
```json
{
  "eventId": "EVT-2025-001",
  "priority": "critical",
  "category": "safety",
  "urgency": "immediate",
  "severity": "extreme",
  "headline": "Tornado Warning - Take Shelter Immediately",
  "description": "A confirmed tornado is on the ground...",
  "instruction": "Move to lowest floor interior room...",
  "area": {
    "polygon": [[...], ...]
  },
  "expiresAt": "2025-12-26T18:00:00Z"
}
```

**Response: 201 Created**
```json
{
  "alertId": "ALT-2025-5678",
  "status": "issued",
  "issuedAt": "2025-12-26T14:32:00Z",
  "broadcastChannels": ["sms", "sirens", "radio", "tv", "mobile_app"]
}
```

### 4.2 Get Alert Status

```
GET /alerts/{alertId}
```

### 4.3 Cancel Alert

```
DELETE /alerts/{alertId}
```

**Response: 200 OK**
```json
{
  "alertId": "ALT-2025-5678",
  "status": "cancelled",
  "cancelledAt": "2025-12-26T16:00:00Z"
}
```

## 5. Resource Management API

### 5.1 Register Resource

```
POST /resources
```

**Request Body:**
```json
{
  "type": "medical_team",
  "name": "County General Mobile Medical Unit 3",
  "agency": "County Health Services",
  "location": {"lat": 40.7128, "lon": -74.0060},
  "capacity": {
    "personnel": 12,
    "equipment": {...}
  },
  "capabilities": ["trauma", "triage", "surgery"],
  "contact": {...}
}
```

**Response: 201 Created**

### 5.2 Deploy Resource

```
POST /deployments
```

**Request Body:**
```json
{
  "eventId": "EVT-2025-001",
  "resourceId": "RES-M-001",
  "assignedArea": {...},
  "mission": "medical_aid"
}
```

**Response: 201 Created**
```json
{
  "deploymentId": "DEP-2025-123",
  "status": "dispatched",
  "estimatedArrival": "2025-12-26T14:45:00Z"
}
```

### 5.3 Update Deployment Status

```
PATCH /deployments/{deploymentId}
```

**Request Body:**
```json
{
  "status": "on_site",
  "progress": {
    "people_rescued": 5,
    "area_covered": 2.5
  }
}
```

### 5.4 Recall Resource

```
POST /deployments/{deploymentId}/recall
```

**Response: 200 OK**

## 6. Evacuation Management API

### 6.1 Issue Evacuation Order

```
POST /evacuations
```

**Request Body:**
```json
{
  "eventId": "EVT-2025-001",
  "type": "mandatory",
  "evacuationZone": {...},
  "estimatedPopulation": 25000,
  "evacuationRoutes": [...],
  "shelters": ["SHL-001", "SHL-002"],
  "deadline": "2025-12-26T16:00:00Z",
  "instructions": "..."
}
```

**Response: 201 Created**

### 6.2 Get Evacuation Status

```
GET /evacuations/{orderId}
```

**Response: 200 OK**
```json
{
  "orderId": "EVAC-2025-45",
  "evacuatedCount": 18500,
  "percentComplete": 74,
  "routeStatus": [
    {"routeId": "R1", "status": "open", "traffic": "moderate"},
    {"routeId": "R2", "status": "congested", "traffic": "heavy"}
  ],
  "shelterCapacity": {
    "total": 30000,
    "occupied": 15000,
    "available": 15000
  }
}
```

## 7. Damage Assessment API

### 7.1 Submit Assessment

```
POST /assessments
```

**Request Body:**
```json
{
  "eventId": "EVT-2025-001",
  "location": {"lat": 40.7128, "lon": -74.0060},
  "structureType": "residential",
  "damageLevel": "severe",
  "estimatedCost": 250000,
  "hazards": ["structural", "fire"],
  "photosUrls": ["https://..."]
}
```

### 7.2 Get Assessment Summary

```
GET /events/{eventId}/assessments/summary
```

**Response: 200 OK**
```json
{
  "totalAssessments": 1250,
  "damageDistribution": {
    "destroyed": 150,
    "severe": 400,
    "moderate": 500,
    "minor": 200
  },
  "totalEstimatedCost": 125000000,
  "structuresInspected": 1250,
  "structuresRemaining": 500
}
```

## 8. Communication API

### 8.1 Send Inter-Agency Message

```
POST /messages
```

**Request Body:**
```json
{
  "fromAgency": "County Emergency Management",
  "toAgency": ["State Emergency Services", "National Guard"],
  "priority": "immediate",
  "subject": "Request for additional medical resources",
  "body": "...",
  "requiresResponse": true
}
```

### 8.2 Get Messages

```
GET /messages?unread=true&priority=immediate
```

## 9. Real-Time WebSocket API

### 9.1 Connection

```
wss://api.disaster-management.org/v1/ws
```

### 9.2 Subscribe to Events

**Client Message:**
```json
{
  "action": "subscribe",
  "channels": ["events", "alerts", "deployments"],
  "filters": {
    "eventId": "EVT-2025-001",
    "severity_min": 5
  }
}
```

**Server Updates:**
```json
{
  "channel": "alerts",
  "type": "new_alert",
  "timestamp": "2025-12-26T14:35:00Z",
  "data": { /* alert object */ }
}
```

### 9.3 Real-Time Location Updates

**Client Message:**
```json
{
  "action": "update_location",
  "resourceId": "RES-M-001",
  "location": {"lat": 40.7128, "lon": -74.0060},
  "heading": 45,
  "speed": 65
}
```

## 10. Search & Query API

### 10.1 Geospatial Search

```
POST /search/geo
```

**Request Body:**
```json
{
  "point": {"lat": 40.7128, "lon": -74.0060},
  "radius": 10000,
  "types": ["events", "resources", "shelters"]
}
```

**Response: 200 OK**
```json
{
  "results": [
    {
      "type": "event",
      "id": "EVT-2025-001",
      "distance": 2500,
      "data": {...}
    },
    ...
  ]
}
```

## 11. Analytics & Reporting API

### 11.1 Get Statistics

```
GET /stats?period=24h&eventId=EVT-2025-001
```

**Response: 200 OK**
```json
{
  "period": "2025-12-25T14:00:00Z to 2025-12-26T14:00:00Z",
  "alerts_issued": 47,
  "people_evacuated": 28500,
  "resources_deployed": 125,
  "avg_response_time": 4.2,
  "casualties": 15,
  "structures_damaged": 1250
}
```

## 12. Error Handling

### Standard Error Response

```json
{
  "error": {
    "code": "RESOURCE_NOT_FOUND",
    "message": "The requested resource does not exist",
    "details": {...},
    "timestamp": "2025-12-26T14:00:00Z",
    "requestId": "req-uuid-1234"
  }
}
```

### HTTP Status Codes

- `200 OK`: Success
- `201 Created`: Resource created
- `400 Bad Request`: Invalid input
- `401 Unauthorized`: Authentication required
- `403 Forbidden`: Insufficient permissions
- `404 Not Found`: Resource not found
- `409 Conflict`: Resource conflict
- `429 Too Many Requests`: Rate limit exceeded
- `500 Internal Server Error`: Server error
- `503 Service Unavailable`: Temporary unavailability

## 13. Rate Limiting

- **Standard Tier**: 1000 requests/hour
- **Premium Tier**: 10000 requests/hour
- **Emergency Tier**: Unlimited (for active disasters)

**Headers:**
```
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 750
X-RateLimit-Reset: 1640534400
```

## 14. Pagination

For list endpoints:

```
GET /events?limit=50&offset=100
```

**Response:**
```json
{
  "total": 1500,
  "limit": 50,
  "offset": 100,
  "hasMore": true,
  "links": {
    "next": "/events?limit=50&offset=150",
    "prev": "/events?limit=50&offset=50"
  },
  "data": [...]
}
```

---

© 2025 WIA · MIT License

---

## Annex A — Conformance Tier Matrix

WIA conformance for disaster-management-system is evaluated across three tiers:

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

- `wia-standards/standards/disaster-management-system/api/` — TypeScript SDK skeleton
- `wia-standards/standards/disaster-management-system/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/disaster-management-system/simulator/` — interactive browser-based simulator for the PHASE protocol

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
