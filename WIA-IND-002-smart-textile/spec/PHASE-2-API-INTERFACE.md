# WIA-IND-002: Smart Textile Standard
## PHASE 2: API INTERFACE SPECIFICATION
### 弘益人間 - Benefit All Humanity

---

## 1. Overview

This document defines the API interface specifications for WIA-IND-002 Smart Textile Standard. These APIs enable smart textile devices to communicate with applications, cloud platforms, and other IoT systems.

**Version:** 1.0  
**Status:** Final  
**Last Updated:** 2025-12-27

## 2. RESTful API Endpoints

### 2.1 Base URL Structure

All API endpoints MUST follow this structure:

```
https://api.example.com/wia/ind-002/v1/{resource}
```

### 2.2 Device Management APIs

#### 2.2.1 Register Device

```
POST /devices/register
Content-Type: application/json
Authorization: Bearer {token}

Request:
{
  "deviceId": "UUID",
  "deviceType": "smart_shirt|smart_sock|smart_band",
  "manufacturer": "Company Name",
  "model": "Model-2025",
  "firmwareVersion": "1.0.3",
  "sensors": ["temperature", "ecg", "motion"],
  "philosophy": "弘益人間"
}

Response: 201 Created
{
  "deviceId": "UUID",
  "apiKey": "generated_api_key",
  "registrationDate": "ISO 8601",
  "status": "active"
}
```

#### 2.2.2 Get Device Status

```
GET /devices/{deviceId}/status
Authorization: Bearer {token}

Response: 200 OK
{
  "deviceId": "UUID",
  "status": "online|offline|error",
  "lastSeen": "ISO 8601",
  "batteryLevel": 85,
  "firmwareVersion": "1.0.3",
  "activeSensors": ["temperature", "ecg"]
}
```

#### 2.2.3 Update Device Configuration

```
PUT /devices/{deviceId}/config
Content-Type: application/json
Authorization: Bearer {token}

Request:
{
  "samplingRates": {
    "temperature": 10,
    "ecg": 250
  },
  "dataTransmission": "realtime|batch",
  "powerMode": "high_performance|balanced|low_power"
}

Response: 200 OK
```

### 2.3 Sensor Data APIs

#### 2.3.1 Stream Sensor Data

```
POST /data/stream
Content-Type: application/json
Authorization: Bearer {token}

Request:
{
  "deviceId": "UUID",
  "timestamp": "ISO 8601",
  "sensors": {
    "temperature": {...},
    "ecg": {...}
  }
}

Response: 202 Accepted
{
  "received": true,
  "sequenceNumber": 12345
}
```

#### 2.3.2 Query Historical Data

```
GET /data/history?deviceId={id}&start={ISO8601}&end={ISO8601}&sensors=temp,hr
Authorization: Bearer {token}

Response: 200 OK
{
  "deviceId": "UUID",
  "timeRange": {
    "start": "ISO 8601",
    "end": "ISO 8601"
  },
  "records": [
    {
      "timestamp": "ISO 8601",
      "sensors": {...}
    }
  ],
  "totalRecords": 1000,
  "page": 1,
  "pageSize": 100
}
```

#### 2.3.3 Get Real-Time Data

```
GET /data/realtime/{deviceId}
Authorization: Bearer {token}

Response: 200 OK
{
  "deviceId": "UUID",
  "timestamp": "ISO 8601",
  "sensors": {
    "temperature": {"value": 36.5, "unit": "celsius"},
    "heartRate": {"value": 72, "unit": "bpm"}
  }
}
```

### 2.4 Analytics APIs

#### 2.4.1 Get Health Insights

```
GET /analytics/health/{userId}?period=7d
Authorization: Bearer {token}

Response: 200 OK
{
  "userId": "hashed UUID",
  "period": "7 days",
  "insights": {
    "averageHeartRate": 68,
    "restingHeartRate": 58,
    "hrvTrend": "improving",
    "sleepQuality": "good",
    "activityLevel": "moderate",
    "stressLevel": "low"
  },
  "recommendations": [
    "Maintain current activity levels",
    "Consider increasing cardio training"
  ]
}
```

#### 2.4.2 Detect Anomalies

```
POST /analytics/anomalies/detect
Content-Type: application/json
Authorization: Bearer {token}

Request:
{
  "deviceId": "UUID",
  "sensorType": "ecg|temperature|pressure",
  "timeWindow": "1h"
}

Response: 200 OK
{
  "anomaliesDetected": true,
  "count": 3,
  "anomalies": [
    {
      "timestamp": "ISO 8601",
      "type": "irregular_heartbeat",
      "severity": "medium",
      "value": 145,
      "expectedRange": "60-100"
    }
  ]
}
```

## 3. WebSocket Streaming API

For real-time bidirectional communication:

```
wss://stream.example.com/wia/ind-002/v1/stream

Connection Request:
{
  "action": "subscribe",
  "deviceId": "UUID",
  "sensors": ["temperature", "ecg"],
  "apiKey": "key"
}

Server Messages:
{
  "type": "data",
  "deviceId": "UUID",
  "timestamp": "ISO 8601",
  "sensors": {...}
}

Client Commands:
{
  "action": "configure",
  "samplingRate": 100
}
```

## 4. MQTT Pub/Sub API

### 4.1 Topic Structure

```
wia/ind-002/{deviceId}/{sensorType}/{action}
```

### 4.2 Publish Sensor Data

```
Topic: wia/ind-002/device-123/temperature/data
QoS: 1
Payload (JSON):
{
  "timestamp": "ISO 8601",
  "value": 36.5,
  "unit": "celsius"
}
```

### 4.3 Subscribe to Commands

```
Topic: wia/ind-002/device-123/+/command
QoS: 2
```

### 4.4 Status Updates

```
Topic: wia/ind-002/device-123/status
Payload:
{
  "status": "online",
  "batteryLevel": 85
}
```

## 5. GraphQL API

```graphql
type Device {
  id: ID!
  type: DeviceType!
  status: DeviceStatus!
  sensors: [Sensor!]!
  lastData: SensorData
  batteryLevel: Int
}

type SensorData {
  timestamp: DateTime!
  temperature: Temperature
  heartRate: HeartRate
  ecg: ECG
  motion: Motion
}

type Query {
  device(id: ID!): Device
  devices(filter: DeviceFilter): [Device!]!
  sensorData(
    deviceId: ID!
    start: DateTime!
    end: DateTime!
    sensors: [SensorType!]
  ): [SensorData!]!
}

type Mutation {
  registerDevice(input: DeviceInput!): Device!
  updateConfig(deviceId: ID!, config: ConfigInput!): Device!
  calibrateSensor(deviceId: ID!, sensor: SensorType!): Boolean!
}

type Subscription {
  deviceData(deviceId: ID!): SensorData!
  deviceStatus(deviceId: ID!): DeviceStatus!
}
```

## 6. Authentication and Authorization

### 6.1 OAuth 2.0 Flow

```
1. Authorization Request
GET /oauth/authorize?
  response_type=code&
  client_id={client_id}&
  redirect_uri={uri}&
  scope=read:devices write:data&
  state={state}

2. Token Exchange
POST /oauth/token
{
  "grant_type": "authorization_code",
  "code": "{auth_code}",
  "client_id": "{client_id}",
  "client_secret": "{secret}",
  "redirect_uri": "{uri}"
}

Response:
{
  "access_token": "token",
  "token_type": "Bearer",
  "expires_in": 3600,
  "refresh_token": "refresh",
  "scope": "read:devices write:data"
}
```

### 6.2 API Key Authentication

```
GET /devices/{id}
X-API-Key: your_api_key_here
```

### 6.3 JWT Bearer Token

```
GET /data/stream
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
```

## 7. Rate Limiting

All APIs implement rate limiting to ensure fair usage:

```
Headers:
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 999
X-RateLimit-Reset: 1640995200

Response when exceeded (429 Too Many Requests):
{
  "error": "rate_limit_exceeded",
  "message": "Too many requests",
  "retryAfter": 60
}
```

Rate limits:
- Free tier: 100 requests/hour
- Basic tier: 1,000 requests/hour
- Professional tier: 10,000 requests/hour
- Enterprise: Custom limits

## 8. Webhooks

### 8.1 Register Webhook

```
POST /webhooks
{
  "url": "https://your-app.com/webhook",
  "events": ["data_received", "anomaly_detected", "battery_low"],
  "secret": "your_webhook_secret"
}
```

### 8.2 Webhook Payload

```
POST https://your-app.com/webhook
X-WIA-Signature: sha256=hash
X-WIA-Event: anomaly_detected

{
  "event": "anomaly_detected",
  "timestamp": "ISO 8601",
  "deviceId": "UUID",
  "data": {
    "type": "irregular_heartbeat",
    "severity": "high"
  }
}
```

## 9. Error Handling

Standard HTTP status codes and error formats:

```json
{
  "error": {
    "code": "INVALID_SENSOR_DATA",
    "message": "Temperature value out of valid range",
    "details": {
      "field": "temperature.value",
      "provided": 150,
      "validRange": "-40 to 85"
    },
    "timestamp": "ISO 8601",
    "requestId": "UUID"
  }
}
```

Common error codes:
- 400: Bad Request - Invalid input data
- 401: Unauthorized - Missing or invalid authentication
- 403: Forbidden - Insufficient permissions
- 404: Not Found - Resource doesn't exist
- 429: Too Many Requests - Rate limit exceeded
- 500: Internal Server Error - Server-side issue
- 503: Service Unavailable - Temporary outage

## 10. API Versioning

APIs MUST support versioning:

- URL versioning: `/v1/devices`, `/v2/devices`
- Header versioning: `Accept: application/vnd.wia.ind-002.v1+json`
- Query parameter: `/devices?api_version=1`

Deprecation notice (6 months minimum):

```
Deprecation: version="v1"
Sunset: Sat, 31 Dec 2025 23:59:59 GMT
Link: <https://api.example.com/v2/devices>; rel="successor-version"
```

---

**Philosophy:** 弘益人間 - These API specifications ensure that smart textile data and functionality are accessible to all developers, enabling innovation that benefits humanity through open, well-documented, and interoperable interfaces.

**License:** Creative Commons BY-SA 4.0  
**Contact:** standards@wia.org

---

## Annex A — Conformance Tier Matrix

WIA conformance for WIA-IND-002-smart-textile is evaluated across three tiers:

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

- `wia-standards/standards/WIA-IND-002-smart-textile/api/` — TypeScript SDK skeleton
- `wia-standards/standards/WIA-IND-002-smart-textile/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/WIA-IND-002-smart-textile/simulator/` — interactive browser-based simulator for the PHASE protocol

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
