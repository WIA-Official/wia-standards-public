# WIA-SOC-007 Phase 2: API Interface Specification

**Version:** 1.0.0  
**Status:** Approved  
**Last Updated:** 2025-12-26

弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. Overview

Phase 2 defines the RESTful API and WebSocket specifications for accessing public transportation data. The API provides real-time information, trip planning, fare calculation, and system status.

## 2. Base URL Structure

```
Production: https://api.transit.example.com/v1
Staging: https://api-staging.transit.example.com/v1
```

## 3. Authentication

### 3.1 API Key Authentication

```http
GET /routes HTTP/1.1
Host: api.transit.example.com
X-API-Key: your-api-key-here
```

### 3.2 OAuth 2.0 (for user-specific operations)

```http
GET /user/favorites HTTP/1.1
Host: api.transit.example.com
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
```

## 4. Core Endpoints

### 4.1 Agency Information

#### GET /agencies
Returns list of all transit agencies.

**Response:**
```json
{
  "agencies": [
    {
      "agencyId": "mta",
      "agencyName": "Metropolitan Transit Authority",
      "agencyUrl": "https://mta.example.com",
      "agencyTimezone": "America/New_York",
      "agencyLang": "en"
    }
  ],
  "count": 1
}
```

#### GET /agencies/:agencyId
Returns details for specific agency.

### 4.2 Routes

#### GET /routes
Returns all routes across all agencies.

**Query Parameters:**
- `agency_id` - Filter by agency
- `route_type` - Filter by route type (0-12)
- `bbox` - Geographic bounding box (min_lat,min_lon,max_lat,max_lon)

**Response:**
```json
{
  "routes": [
    {
      "routeId": "route-101",
      "routeShortName": "101",
      "routeLongName": "Downtown Express",
      "routeType": 3,
      "routeColor": "FF6319"
    }
  ],
  "count": 1
}
```

#### GET /routes/:routeId/trips
Returns all trips for a specific route.

**Query Parameters:**
- `date` - Service date (YYYY-MM-DD)
- `direction_id` - Direction (0 or 1)

### 4.3 Stops

#### GET /stops
Returns all stops/stations.

**Query Parameters:**
- `lat` - Latitude for proximity search
- `lon` - Longitude for proximity search
- `radius` - Search radius in meters (default: 500)
- `wheelchair_accessible` - Filter by accessibility (0,1,2)

**Response:**
```json
{
  "stops": [
    {
      "stopId": "stop-12345",
      "stopName": "Main Street Station",
      "stopLat": 40.748817,
      "stopLon": -73.985428,
      "wheelchairBoarding": 1,
      "distance": 125.4
    }
  ],
  "count": 1
}
```

#### GET /stops/:stopId/arrivals
Returns next arrivals at specified stop.

**Query Parameters:**
- `route_id` - Filter by route
- `limit` - Maximum number of arrivals (default: 10)
- `timeframe` - Minutes to look ahead (default: 60)

**Response:**
```json
{
  "stopId": "stop-12345",
  "stopName": "Main Street Station",
  "arrivals": [
    {
      "tripId": "trip-101-001",
      "routeId": "route-101",
      "routeShortName": "101",
      "headsign": "Airport Terminal",
      "scheduledArrival": "2025-12-26T14:35:00Z",
      "predictedArrival": "2025-12-26T14:37:00Z",
      "delay": 120,
      "occupancyStatus": "MANY_SEATS_AVAILABLE",
      "wheelchairAccessible": true,
      "realtime": true
    }
  ],
  "timestamp": "2025-12-26T14:32:00Z"
}
```

### 4.4 Trip Planning

#### POST /trip-planner
Calculates optimal journey from origin to destination.

**Request Body:**
```json
{
  "origin": {
    "lat": 40.748817,
    "lon": -73.985428
  },
  "destination": {
    "lat": 40.712776,
    "lon": -74.005974
  },
  "time": "2025-12-26T14:30:00Z",
  "arriveBy": false,
  "modes": ["BUS", "METRO", "TRAM"],
  "preferences": {
    "optimize": "fastest",
    "maxWalkDistance": 800,
    "wheelchairAccessible": false,
    "maxTransfers": 3
  }
}
```

**Response:**
```json
{
  "itineraries": [
    {
      "duration": 2520,
      "walkDistance": 450,
      "walkTime": 360,
      "transitTime": 2160,
      "waitingTime": 180,
      "transfers": 2,
      "fare": {
        "currency": "USD",
        "amount": 3.50
      },
      "carbonFootprint": {
        "emissions": 0.42,
        "savingsVsCar": 2.8
      },
      "legs": [
        {
          "mode": "WALK",
          "from": "Origin",
          "to": "Main Street Station",
          "startTime": "2025-12-26T14:30:00Z",
          "endTime": "2025-12-26T14:36:00Z",
          "distance": 450,
          "path": [...]
        },
        {
          "mode": "BUS",
          "routeId": "route-101",
          "routeShortName": "101",
          "tripId": "trip-101-001",
          "headsign": "Downtown",
          "from": "Main Street Station",
          "to": "Transfer Hub",
          "startTime": "2025-12-26T14:39:00Z",
          "endTime": "2025-12-26T14:52:00Z",
          "stops": 8
        }
      ]
    }
  ],
  "requestId": "req-20251226-143000-abc123"
}
```

### 4.5 Vehicle Tracking

#### GET /vehicles/:vehicleId/position
Returns current position of specific vehicle.

**Response:**
```json
{
  "vehicleId": "bus-1234",
  "tripId": "trip-101-001",
  "routeId": "route-101",
  "position": {
    "lat": 40.748817,
    "lon": -73.985428,
    "bearing": 135.5,
    "speed": 12.5
  },
  "timestamp": "2025-12-26T14:32:15Z",
  "occupancyStatus": "MANY_SEATS_AVAILABLE",
  "nextStop": {
    "stopId": "stop-12346",
    "stopName": "2nd Avenue",
    "eta": "2025-12-26T14:35:00Z"
  }
}
```

#### GET /routes/:routeId/vehicles
Returns all vehicles currently serving a route.

### 4.6 Service Alerts

#### GET /alerts
Returns all active service alerts.

**Query Parameters:**
- `route_id` - Filter by route
- `stop_id` - Filter by stop
- `agency_id` - Filter by agency

**Response:**
```json
{
  "alerts": [
    {
      "alertId": "alert-2025-001",
      "cause": "CONSTRUCTION",
      "effect": "DETOUR",
      "severity": "MODERATE",
      "header": "Route 101 Detour",
      "description": "Due to construction...",
      "url": "https://example.com/alerts/2025-001",
      "activePeriod": {
        "start": "2025-12-26T06:00:00Z",
        "end": "2025-12-26T18:00:00Z"
      },
      "affectedRoutes": ["route-101"],
      "affectedStops": []
    }
  ]
}
```

### 4.7 Fare Information

#### POST /fare/calculate
Calculates fare for a journey.

**Request Body:**
```json
{
  "itinerary": {
    "legs": [...],
    "date": "2025-12-26"
  },
  "passengerType": "adult",
  "paymentMethod": "contactless"
}
```

**Response:**
```json
{
  "totalFare": 3.50,
  "currency": "USD",
  "breakdown": [
    {
      "leg": 0,
      "fare": 2.75,
      "description": "Bus single ride"
    },
    {
      "leg": 1,
      "fare": 0.75,
      "description": "Transfer discount"
    }
  ],
  "validUntil": "2025-12-26T16:30:00Z"
}
```

## 5. WebSocket Real-time Updates

### 5.1 Connection

```javascript
const ws = new WebSocket('wss://api.transit.example.com/v1/realtime');

ws.onopen = () => {
  ws.send(JSON.stringify({
    type: 'subscribe',
    channels: ['route:101', 'stop:12345']
  }));
};
```

### 5.2 Message Types

**Vehicle Position Update:**
```json
{
  "type": "vehicle_position",
  "vehicleId": "bus-1234",
  "position": {...},
  "timestamp": "2025-12-26T14:32:15Z"
}
```

**Arrival Update:**
```json
{
  "type": "arrival_update",
  "stopId": "stop-12345",
  "tripId": "trip-101-001",
  "predictedArrival": "2025-12-26T14:37:00Z",
  "delay": 120
}
```

**Service Alert:**
```json
{
  "type": "service_alert",
  "alert": {...}
}
```

## 6. Rate Limiting

### 6.1 Limits
- **Free Tier**: 100 requests/minute, 10,000 requests/day
- **Standard Tier**: 1,000 requests/minute, 100,000 requests/day
- **Premium Tier**: 10,000 requests/minute, unlimited daily

### 6.2 Headers
```
X-RateLimit-Limit: 100
X-RateLimit-Remaining: 87
X-RateLimit-Reset: 1640000000
```

## 7. Error Responses

### 7.1 Error Format
```json
{
  "error": {
    "code": "INVALID_PARAMETER",
    "message": "Invalid stop ID provided",
    "details": {
      "parameter": "stop_id",
      "value": "invalid-123"
    },
    "requestId": "req-20251226-143000-abc123"
  }
}
```

### 7.2 Status Codes
- `400` - Bad Request (invalid parameters)
- `401` - Unauthorized (invalid API key)
- `403` - Forbidden (insufficient permissions)
- `404` - Not Found (resource doesn't exist)
- `429` - Too Many Requests (rate limit exceeded)
- `500` - Internal Server Error
- `503` - Service Unavailable

## 8. Pagination

For list endpoints returning many results:

```http
GET /stops?limit=50&offset=0
```

**Response Headers:**
```
X-Total-Count: 1247
Link: </stops?limit=50&offset=50>; rel="next"
```

## 9. Caching

### 9.1 Cache Headers
```
Cache-Control: public, max-age=60
ETag: "abc123"
Last-Modified: Wed, 26 Dec 2025 14:30:00 GMT
```

### 9.2 Conditional Requests
```http
GET /routes HTTP/1.1
If-None-Modified-Since: Wed, 26 Dec 2025 14:30:00 GMT
If-None-Match: "abc123"
```

---

**Previous**: [Phase 1 - Data Format](PHASE-1-DATA-FORMAT.md)  
**Next**: [Phase 3 - Communication Protocol](PHASE-3-PROTOCOL.md)

弘益人間 - Benefit All Humanity

© 2025 WIA / SmileStory Inc.

---

## Annex A — Conformance Tier Matrix

WIA conformance for WIA-SOC-007 (Public Transportation) is evaluated across three tiers, applied to schedules · stop inventory · real-time arrivals · accessibility metadata:

| Tier | Scope | Mandatory artifacts | Audit cadence |
|------|-------|--------------------|----------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | OpenAPI 3.0 contract, JSON Schema validation report, security threat model | None (annual self-review recommended) |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Tier 1 artifacts + signed third-party assessor report against this PHASE | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Tier 2 artifacts + WIA accreditation, ISO/IEC 17065:2012 conformity assessment, evidence retention ≥ 7 years | Every 12 months |

Implementations MUST clearly disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page. Tier downgrade events MUST be reported to the WIA registry within 30 days.

---

## Annex B — Cross-Walk to International Standards

The PHASE specification reuses or normatively references the following published standards. Implementers SHOULD review the listed standards alongside this PHASE document; where a conflict exists, the more specific WIA requirement governs unless explicitly superseded by a binding national regulation.

- GTFS Reference — General Transit Feed Specification (open spec, MobilityData)
- GTFS-Realtime — protobuf schema for live updates (open spec, MobilityData)
- ISO 17572-3:2018 — Intelligent transport systems — Location referencing
- ISO/IEC 18004:2015 — QR Code (ticket encoding reference)
- IETF RFC 7946 — GeoJSON (stop and shape geometry)

This cross-walk is informative only. WIA does not republish the referenced documents; readers MUST obtain authoritative copies from the issuing body. Cross-walk entries are reviewed at every minor version of this PHASE.

---

## Annex C — Reference Implementations and Test Vectors

### C.1 Reference Implementations

WIA does not require implementers to use a particular library, but maintains pointers to the canonical reference implementation directories under the WIA-Official GitHub organization:

- `wia-standards/standards/public-transportation/api/` — TypeScript SDK skeleton
- `wia-standards/standards/public-transportation/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/public-transportation/simulator/` — interactive browser-based simulator for the PHASE protocol

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
