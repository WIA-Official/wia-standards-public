# WIA-PET-007 PHASE 3: API STANDARDS

**Version:** 1.0.0  
**Date:** 2025-12-25  
**Status:** Active Standard

---

## 1. API Overview

PHASE 3 defines RESTful API standards, BLE communication protocols, and data exchange mechanisms for WIA-PET-007 pet wearable ecosystem.

---

## 2. RESTful API Endpoints

### 2.1 Base URL Structure

```
https://api.{provider}.com/v1/
```

### 2.2 Authentication

**OAuth 2.0 with PKCE (Proof Key for Code Exchange)**

```
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=authorization_code&
code={authorization_code}&
redirect_uri={redirect_uri}&
client_id={client_id}&
code_verifier={code_verifier}
```

**Response:**
```json
{
  "access_token": "eyJhbGc...",
  "token_type": "Bearer",
  "expires_in": 86400,
  "refresh_token": "refresh_token_here",
  "scope": "read_health write_settings"
}
```

### 2.3 Core API Endpoints

+------------+--------------------------------+---------------------------+
| Method     | Endpoint                       | Description               |
+------------+--------------------------------+---------------------------+
| GET        | /devices                       | List user's devices       |
| GET        | /devices/{id}                  | Get device info           |
| GET        | /devices/{id}/health           | Get health data           |
| GET        | /devices/{id}/activity         | Get activity data         |
| GET        | /devices/{id}/location         | Get location data         |
| POST       | /devices/{id}/geofence         | Create geofence           |
| PUT        | /devices/{id}/settings         | Update settings           |
| GET        | /pets/{petId}/history          | Get historical data       |
| POST       | /alerts/acknowledge            | Acknowledge alert         |
| GET        | /alerts                        | Get active alerts         |
+------------+--------------------------------+---------------------------+

---

## 3. Request/Response Format

### 3.1 Request Headers

```
GET /devices/PW-DOG-12345/health
Authorization: Bearer {access_token}
Accept: application/json
Content-Type: application/json
User-Agent: PetWearApp/1.0 (iOS 14.0)
```

### 3.2 Success Response

```json
{
  "status": "success",
  "timestamp": "2025-12-25T15:30:00.000Z",
  "data": {
    "deviceId": "PW-DOG-12345",
    "healthMetrics": [ ... ]
  },
  "pagination": {
    "page": 1,
    "pageSize": 100,
    "totalPages": 3,
    "nextPage": "/devices/PW-DOG-12345/health?page=2"
  }
}
```

### 3.3 Error Response

```json
{
  "status": "error",
  "timestamp": "2025-12-25T15:30:00.000Z",
  "error": {
    "code": "INVALID_GEOFENCE_RADIUS",
    "message": "Geofence radius must be between 50 and 5000 meters",
    "details": {
      "field": "radius",
      "providedValue": 25,
      "allowedRange": "50-5000"
    },
    "documentation": "https://wia.org/docs/pet-007/geofences#radius",
    "requestId": "req-20251225-abc123"
  }
}
```

---

## 4. HTTP Status Codes

+-------------+-------------------------+--------------------------------+
| Status Code | Meaning                 | Use Case                       |
+-------------+-------------------------+--------------------------------+
| 200         | OK                      | Successful request             |
| 201         | Created                 | Resource created               |
| 204         | No Content              | Successful delete              |
| 400         | Bad Request             | Invalid JSON, missing fields   |
| 401         | Unauthorized            | Invalid/missing auth token     |
| 403         | Forbidden               | Insufficient permissions       |
| 404         | Not Found               | Device/resource doesn't exist  |
| 429         | Too Many Requests       | Rate limit exceeded            |
| 500         | Internal Server Error   | Server-side error              |
| 503         | Service Unavailable     | Maintenance or overload        |
+-------------+-------------------------+--------------------------------+

---

## 5. Rate Limiting

### 5.1 Rate Limit Tiers

+-------------------+------------------+------------------+
| Endpoint Type     | Free Tier        | Premium Tier     |
+-------------------+------------------+------------------+
| Device Data       | 100 req/hour     | 1000 req/hour    |
| Historical Query  | 20 req/hour      | 200 req/hour     |
| Location Updates  | 500 req/day      | 5000 req/day     |
| Geofence Mgmt     | 50 req/day       | 500 req/day      |
+-------------------+------------------+------------------+

### 5.2 Rate Limit Headers

```
HTTP/1.1 200 OK
X-RateLimit-Limit: 100
X-RateLimit-Remaining: 87
X-RateLimit-Reset: 1735134000
Retry-After: 3600
```

---

## 6. BLE Communication Protocol

### 6.1 GATT Service UUIDs

+----------------------+-------------------------+
| Service              | UUID                    |
+----------------------+-------------------------+
| Device Information   | 0x180A                  |
| Battery Service      | 0x180F                  |
| Health Data Service  | WIA-PET-001             |
| Activity Service     | WIA-PET-002             |
| Location Service     | WIA-PET-003             |
+----------------------+-------------------------+

**WIA Custom UUIDs:**
- WIA-PET-001: `a0b1c2d3-e4f5-6789-abcd-ef0123456789`
- WIA-PET-002: `a0b1c2d3-e4f5-6789-abcd-ef0123456790`
- WIA-PET-003: `a0b1c2d3-e4f5-6789-abcd-ef0123456791`

### 6.2 Health Data Service (WIA-PET-001)

**Characteristics:**

| Characteristic | UUID | Properties | Description |
|----------------|------|------------|-------------|
| Heart Rate | 0x2A37 | Notify | Heart rate in bpm |
| Temperature | 0x2A6E | Read, Notify | Body temperature |
| Respiratory Rate | WIA-custom | Read, Notify | Breaths per minute |
| HRV | WIA-custom | Read | Heart rate variability |

### 6.3 BLE Data Transfer

**Notification Format:**
```
Timestamp (4 bytes) + Metric ID (1 byte) + Value (2 bytes) + Quality (1 byte)
```

Example: Heart rate notification
```
[0x61, 0x7E, 0xA2, 0x1C, 0x01, 0x00, 0x5F, 0x5C]
 ^timestamp (1703592476)   ^HR  ^95bpm  ^qual 92
```

---

## 7. WebSocket Realtime API

### 7.1 Connection

```javascript
const ws = new WebSocket('wss://realtime.api.provider.com/v1/stream');
ws.send(JSON.stringify({
  action: 'subscribe',
  deviceId: 'PW-DOG-12345',
  channels: ['health', 'location', 'alerts'],
  token: 'Bearer eyJhbGc...'
}));
```

### 7.2 Realtime Message Format

```json
{
  "channel": "health",
  "deviceId": "PW-DOG-12345",
  "timestamp": "2025-12-25T15:30:00.000Z",
  "data": {
    "heartRate": {
      "value": 95,
      "unit": "bpm",
      "quality": 92
    }
  }
}
```

---

## 8. Webhook Notifications

### 8.1 Webhook Registration

```
POST /webhooks
{
  "url": "https://your-server.com/wia-webhooks",
  "events": ["health_alert", "geofence_exit", "low_battery"],
  "secret": "your_signing_secret"
}
```

### 8.2 Webhook Payload

```
POST https://your-server.com/wia-webhooks
X-WIA-Signature: sha256=abc123...
X-WIA-Event: geofence_exit
Content-Type: application/json

{
  "event": "geofence_exit",
  "deviceId": "PW-DOG-12345",
  "timestamp": "2025-12-25T15:30:00.000Z",
  "data": {
    "fenceId": "FENCE-HOME-001",
    "location": {...}
  }
}
```

---

## 9. Data Export API

### 9.1 Export Request

```
POST /exports
{
  "deviceId": "PW-DOG-12345",
  "dataTypes": ["health", "activity", "location"],
  "startDate": "2025-01-01",
  "endDate": "2025-12-31",
  "format": "json|csv|pdf"
}
```

### 9.2 Export Response

```json
{
  "exportId": "EXPORT-abc123",
  "status": "processing",
  "estimatedTime": 120,
  "downloadUrl": null
}
```

**Poll for completion:**
```
GET /exports/EXPORT-abc123
{
  "exportId": "EXPORT-abc123",
  "status": "completed",
  "downloadUrl": "https://exports.provider.com/abc123.zip",
  "expiresAt": "2025-12-26T15:30:00.000Z"
}
```

---

## 10. FHIR Veterinary Integration

### 10.1 FHIR Patient Resource (Adapted for Pets)

```json
{
  "resourceType": "Patient",
  "id": "PET-ABC-789",
  "extension": [
    {
      "url": "http://wia.org/fhir/StructureDefinition/pet-species",
      "valueString": "dog"
    },
    {
      "url": "http://wia.org/fhir/StructureDefinition/pet-breed",
      "valueString": "Golden Retriever"
    }
  ],
  "identifier": [
    {
      "system": "http://wia.org/pet-id",
      "value": "PET-ABC-789"
    },
    {
      "system": "http://petmicrochip.org",
      "value": "985112345678901"
    }
  ],
  "name": [{"text": "Max"}],
  "gender": "male",
  "birthDate": "2020-03-15"
}
```

### 10.2 FHIR Observation Resource (Vital Signs)

```json
{
  "resourceType": "Observation",
  "status": "final",
  "code": {
    "coding": [{
      "system": "http://loinc.org",
      "code": "8867-4",
      "display": "Heart rate"
    }]
  },
  "subject": {
    "reference": "Patient/PET-ABC-789"
  },
  "effectiveDateTime": "2025-12-25T10:30:00Z",
  "valueQuantity": {
    "value": 95,
    "unit": "beats/minute",
    "system": "http://unitsofmeasure.org",
    "code": "/min"
  },
  "device": {
    "reference": "Device/PW-DOG-12345"
  }
}
```

---

## 11. API Versioning

### 11.1 URL Versioning

```
https://api.provider.com/v1/devices
https://api.provider.com/v2/devices
```

### 11.2 Version Support Policy

- Current version (v1): Full support
- Previous version (v0): Deprecated, 18-month sunset
- Deprecated versions: Read-only access, no new features

---

## Appendix A: Complete API Request Examples

### A.1 Get Health Data with Pagination

```bash
curl -X GET \
  'https://api.provider.com/v1/devices/PW-DOG-12345/health?start=2025-12-25T00:00:00Z&end=2025-12-25T23:59:59Z&page=1&pageSize=100' \
  -H 'Authorization: Bearer eyJhbGc...' \
  -H 'Accept: application/json'
```

### A.2 Create Geofence

```bash
curl -X POST \
  'https://api.provider.com/v1/devices/PW-DOG-12345/geofence' \
  -H 'Authorization: Bearer eyJhbGc...' \
  -H 'Content-Type: application/json' \
  -d '{
    "name": "Home Safe Zone",
    "type": "circular",
    "center": {"latitude": 37.5665, "longitude": 126.9780},
    "radius": 500,
    "notifications": {"entry": true, "exit": true}
  }'
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

© 2025 SmileStory Inc. / WIA  
WIA-PET-007 PHASE 3 Specification
