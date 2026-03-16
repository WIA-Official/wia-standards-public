# WIA-AUTO-024: Fleet Management Standard
## PHASE 3 - Protocol Definitions

**Version:** 1.0.0  
**Status:** Active  
**Category:** Automotive / Fleet Management  
**Last Updated:** January 2025

---

## Overview

Phase 3 defines communication protocols, APIs, authentication mechanisms, and data exchange standards for fleet management systems. These specifications ensure secure, reliable, and efficient communication between vehicles, backend systems, mobile applications, and third-party integrations.

**弘益人間 (Benefit All Humanity)** - Open, standardized protocols enable innovation and interoperability across the fleet management ecosystem.

---

## 1. REST API Specifications

### 1.1 Base URL Structure

```
https://api.{provider}.com/fleet/v1/{resource}
```

### 1.2 Core Endpoints

#### Vehicle Management

**GET /vehicles**
- List all vehicles
- Query parameters: `status`, `type`, `fleet_id`, `page`, `limit`
- Response: Paginated list of vehicles

**GET /vehicles/{vehicleId}**
- Get vehicle details
- Response: Complete vehicle profile

**POST /vehicles**
- Register new vehicle
- Request body: Vehicle registration data
- Response: Created vehicle object

**PUT /vehicles/{vehicleId}**
- Update vehicle information
- Request body: Updated fields
- Response: Updated vehicle object

**DELETE /vehicles/{vehicleId}**
- Decommission vehicle
- Response: Confirmation

#### Real-Time Tracking

**GET /vehicles/{vehicleId}/location**
- Get current vehicle location
- Response: Latest position update

**GET /vehicles/{vehicleId}/telemetry**
- Get current telemetry data
- Response: Latest diagnostic information

**POST /vehicles/{vehicleId}/telemetry**
- Submit telemetry data (device endpoint)
- Request body: Telemetry data batch
- Response: Acknowledgment

#### Route Management

**POST /routes/optimize**
- Request route optimization
- Request body: Waypoints, vehicles, constraints
- Response: Optimized routes

**GET /routes/{routeId}**
- Get route details
- Response: Route definition

**PUT /routes/{routeId}/status**
- Update route status
- Request body: New status
- Response: Updated route

#### Driver Management

**GET /drivers**
- List all drivers
- Query parameters: `status`, `fleet_id`, `page`, `limit`

**GET /drivers/{driverId}/safety-score**
- Get driver safety score
- Query parameters: `period` (day, week, month, year)
- Response: Safety score components and history

**GET /drivers/{driverId}/hours**
- Get hours of service data
- Query parameters: `date`
- Response: HOS summary

---

## 2. GraphQL API

### 2.1 Schema Definition

```graphql
type Query {
  vehicle(id: ID!): Vehicle
  vehicles(filter: VehicleFilter, page: Int, limit: Int): VehiclePage
  driver(id: ID!): Driver
  route(id: ID!): Route
  fleet(id: ID!): Fleet
  analytics(fleetId: ID!, period: DateRange!): FleetAnalytics
}

type Mutation {
  createVehicle(input: VehicleInput!): Vehicle
  updateVehicle(id: ID!, input: VehicleUpdateInput!): Vehicle
  optimizeRoutes(input: RouteOptimizationInput!): [Route]
  updateDriverStatus(driverId: ID!, status: DriverStatus!): Driver
}

type Subscription {
  vehicleLocationUpdated(vehicleId: ID!): LocationUpdate
  telemetryUpdated(vehicleId: ID!): TelemetryUpdate
  routeProgressUpdated(routeId: ID!): RouteProgress
  alertTriggered(fleetId: ID!): Alert
}

type Vehicle {
  id: ID!
  vin: String!
  make: String!
  model: String!
  year: Int!
  status: VehicleStatus!
  currentLocation: Location
  telemetry: Telemetry
  driver: Driver
  route: Route
}

type Location {
  latitude: Float!
  longitude: Float!
  accuracy: Float
  heading: Float
  speed: Float
  timestamp: DateTime!
}

type Telemetry {
  engineRPM: Int
  fuelLevel: Float
  batteryVoltage: Float
  odometer: Float
  diagnostics: DiagnosticData
}
```

### 2.2 Example Queries

**Get Fleet Overview:**
```graphql
query FleetOverview($fleetId: ID!) {
  fleet(id: $fleetId) {
    id
    name
    vehicles {
      id
      status
      currentLocation {
        latitude
        longitude
      }
      driver {
        id
        name
        safetyScore
      }
    }
  }
}
```

**Subscribe to Vehicle Updates:**
```graphql
subscription VehicleTracking($vehicleId: ID!) {
  vehicleLocationUpdated(vehicleId: $vehicleId) {
    vehicleId
    location {
      latitude
      longitude
      speed
      timestamp
    }
  }
}
```

---

## 3. WebSocket Protocol

### 3.1 Connection

```
wss://realtime.{provider}.com/fleet/v1
```

### 3.2 Authentication

```json
{
  "type": "auth",
  "token": "Bearer {jwt_token}"
}
```

### 3.3 Subscribe to Updates

```json
{
  "type": "subscribe",
  "channel": "vehicle.location",
  "vehicleId": "VEH-001",
  "updateInterval": 5000
}
```

### 3.4 Message Format

**Location Update:**
```json
{
  "type": "update",
  "channel": "vehicle.location",
  "vehicleId": "VEH-001",
  "timestamp": "2025-01-15T14:32:45.123Z",
  "data": {
    "latitude": 37.7749,
    "longitude": -122.4194,
    "speed": 65.5,
    "heading": 185.7
  }
}
```

**Event Notification:**
```json
{
  "type": "event",
  "channel": "vehicle.alerts",
  "vehicleId": "VEH-001",
  "timestamp": "2025-01-15T14:32:45.123Z",
  "data": {
    "eventType": "harsh_braking",
    "severity": "medium",
    "location": {
      "latitude": 37.7749,
      "longitude": -122.4194
    }
  }
}
```

---

## 4. MQTT Protocol for IoT Devices

### 4.1 Topic Structure

```
{organizationId}/fleet/{vehicleId}/{dataType}
```

**Examples:**
- `org123/fleet/VEH-001/location`
- `org123/fleet/VEH-001/telemetry`
- `org123/fleet/VEH-001/events`

### 4.2 QoS Levels

- **QoS 0:** Telemetry data (best effort)
- **QoS 1:** Location updates (at least once)
- **QoS 2:** Critical events (exactly once)

### 4.3 Message Payload

All messages use JSON format as defined in Phase 1 data schemas.

### 4.4 Last Will and Testament

```json
{
  "type": "device_offline",
  "vehicleId": "VEH-001",
  "timestamp": "2025-01-15T14:32:45.123Z",
  "reason": "connection_lost"
}
```

---

## 5. Authentication & Authorization

### 5.1 OAuth 2.0 Flow

**Authorization Code Grant (for user applications):**

1. Request authorization:
```
GET /oauth/authorize?
  response_type=code&
  client_id={client_id}&
  redirect_uri={redirect_uri}&
  scope=fleet.read fleet.write&
  state={random_string}
```

2. Exchange code for token:
```
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=authorization_code&
code={authorization_code}&
redirect_uri={redirect_uri}&
client_id={client_id}&
client_secret={client_secret}
```

3. Response:
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "refresh_token": "tGzv3JOkF0XG5Qx2TlKWIA",
  "scope": "fleet.read fleet.write"
}
```

**Client Credentials Grant (for M2M):**

```
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials&
client_id={client_id}&
client_secret={client_secret}&
scope=telemetry.write
```

### 5.2 JWT Token Structure

**Header:**
```json
{
  "alg": "RS256",
  "typ": "JWT",
  "kid": "key-id-1"
}
```

**Payload:**
```json
{
  "iss": "https://auth.{provider}.com",
  "sub": "user_id_123",
  "aud": "fleet-api",
  "exp": 1642262400,
  "iat": 1642258800,
  "scope": ["fleet.read", "fleet.write"],
  "org_id": "org_456",
  "role": "fleet_manager"
}
```

### 5.3 API Key Authentication

For simple device authentication:

```
Authorization: ApiKey {api_key}
```

API keys should:
- Be rotated every 90 days
- Have specific scope limitations
- Include rate limiting
- Log all usage

---

## 6. Rate Limiting

### 6.1 Rate Limit Headers

```
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 847
X-RateLimit-Reset: 1642262400
```

### 6.2 Rate Limit Tiers

- **Free Tier:** 100 requests/hour
- **Basic Tier:** 1,000 requests/hour
- **Pro Tier:** 10,000 requests/hour
- **Enterprise:** Custom limits

### 6.3 Rate Limit Response

```json
{
  "status": "error",
  "statusCode": 429,
  "error": {
    "code": "RATE_LIMIT_EXCEEDED",
    "message": "API rate limit exceeded",
    "retryAfter": 3600
  }
}
```

---

## 7. Webhooks

### 7.1 Configuration

**POST /webhooks**
```json
{
  "url": "https://yourserver.com/webhook",
  "events": ["vehicle.location_update", "driver.hos_violation", "maintenance.due"],
  "secret": "webhook_signing_secret"
}
```

### 7.2 Webhook Payload

```json
{
  "id": "evt_1234567890",
  "type": "vehicle.location_update",
  "timestamp": "2025-01-15T14:32:45.123Z",
  "data": {
    "vehicleId": "VEH-001",
    "location": {
      "latitude": 37.7749,
      "longitude": -122.4194
    }
  }
}
```

### 7.3 Signature Verification

```
X-Webhook-Signature: sha256={hmac_hex_digest}
```

Verify using:
```python
import hmac
import hashlib

def verify_signature(payload, signature, secret):
    expected = hmac.new(
        secret.encode(),
        payload.encode(),
        hashlib.sha256
    ).hexdigest()
    return hmac.compare_digest(f"sha256={expected}", signature)
```

---

## 8. Error Handling

### 8.1 Standard Error Codes

- **400 Bad Request:** Invalid request parameters
- **401 Unauthorized:** Missing or invalid authentication
- **403 Forbidden:** Insufficient permissions
- **404 Not Found:** Resource does not exist
- **409 Conflict:** Request conflicts with current state
- **422 Unprocessable Entity:** Validation errors
- **429 Too Many Requests:** Rate limit exceeded
- **500 Internal Server Error:** Server error
- **503 Service Unavailable:** Temporary outage

### 8.2 Error Response Format

```json
{
  "status": "error",
  "statusCode": 422,
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "Request validation failed",
    "details": [
      {
        "field": "latitude",
        "message": "Must be between -90 and 90",
        "value": 95.5
      }
    ]
  },
  "metadata": {
    "requestId": "req_abc123",
    "documentation": "https://docs.{provider}.com/errors/VALIDATION_ERROR"
  }
}
```

---

## 9. Pagination

### 9.1 Cursor-Based Pagination

**Request:**
```
GET /vehicles?limit=50&cursor=eyJpZCI6MTIzNDU2fQ==
```

**Response:**
```json
{
  "data": [...],
  "pagination": {
    "hasMore": true,
    "nextCursor": "eyJpZCI6MTIzNTA2fQ==",
    "limit": 50
  }
}
```

---

## 10. API Versioning

### 10.1 Version in URL

```
/api/v1/vehicles
/api/v2/vehicles
```

### 10.2 Version Header

```
Accept: application/vnd.wia-fleet.v1+json
```

### 10.3 Deprecation Notice

```
Deprecation: true
Sunset: Sat, 31 Dec 2025 23:59:59 GMT
Link: <https://docs.{provider}.com/migrations/v2>; rel="sunset"
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA - World Certification Industry Association*  
*© 2025 SmileStory Inc. / WIA*  
*MIT License*
