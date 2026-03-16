# WIA-IND-003: Phase 2 - API Interface Specification
## Wearable Fashion API Standards

**Version:** 1.0
**Status:** Final
**Date:** 2025-01-15
**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

Phase 2 defines RESTful API interfaces for wearable fashion devices, enabling communication between devices, mobile applications, cloud services, and fashion platforms.

## 2. API Architecture

### 2.1 Base URL Structure

```
https://api.{domain}/wia-ind-003/v1/{resource}
```

Example: `https://api.fashiontech.com/wia-ind-003/v1/devices`

### 2.2 Authentication

All API requests MUST include authentication:

**OAuth 2.0 (Recommended)**
```http
Authorization: Bearer {access_token}
```

**API Key (Alternative)**
```http
X-API-Key: {api_key}
```

## 3. Device Management APIs

### 3.1 Register Device

**Endpoint:** `POST /devices`

**Request:**
```json
{
  "deviceId": "WF-SW-12345",
  "deviceType": "smartwatch",
  "brand": "FashionTech",
  "model": "Elegance Pro X",
  "firmware": "2.1.0",
  "userId": "user-uuid"
}
```

**Response:** `201 Created`
```json
{
  "deviceId": "WF-SW-12345",
  "registeredAt": "2025-01-15T14:30:00Z",
  "status": "active",
  "syncToken": "sync-token-abc123"
}
```

### 3.2 Get Device Information

**Endpoint:** `GET /devices/{deviceId}`

**Response:** `200 OK`
```json
{
  "device": { /* Device data schema from Phase 1 */ },
  "lastSync": "2025-01-15T14:25:00Z",
  "batteryLevel": 85,
  "connectionStatus": "connected"
}
```

### 3.3 Update Device Settings

**Endpoint:** `PUT /devices/{deviceId}/settings`

**Request:**
```json
{
  "notifications": {
    "enabled": true,
    "vibration": true,
    "display": false
  },
  "sensors": {
    "heartRate": {
      "enabled": true,
      "frequency": 5
    }
  },
  "display": {
    "brightness": 70,
    "alwaysOn": true
  }
}
```

**Response:** `200 OK`

## 4. Sensor Data APIs

### 4.1 Upload Sensor Data

**Endpoint:** `POST /devices/{deviceId}/data`

**Request:**
```json
{
  "timestamp": "2025-01-15T14:30:00Z",
  "batch": [
    { /* Sensor data from Phase 1 */ },
    { /* More sensor readings */ }
  ]
}
```

**Response:** `201 Created`
```json
{
  "recordsProcessed": 42,
  "nextSyncToken": "sync-token-xyz789"
}
```

### 4.2 Query Sensor Data

**Endpoint:** `GET /devices/{deviceId}/data`

**Query Parameters:**
- `type`: Sensor type (heartRate, steps, etc.)
- `start`: Start timestamp (ISO 8601)
- `end`: End timestamp (ISO 8601)
- `limit`: Maximum records (default: 100, max: 1000)
- `offset`: Pagination offset

**Response:** `200 OK`
```json
{
  "data": [ /* Array of sensor readings */ ],
  "pagination": {
    "total": 500,
    "limit": 100,
    "offset": 0,
    "hasMore": true
  }
}
```

## 5. Fashion Platform Integration APIs

### 5.1 Product Catalog Sync

**Endpoint:** `POST /fashion/products`

**Request:**
```json
{
  "sku": "FT-SW-001",
  "name": "Elegance Pro X Smartwatch",
  "category": "smartwatch",
  "price": {
    "amount": 399.99,
    "currency": "USD"
  },
  "availability": {
    "inStock": true,
    "quantity": 150,
    "stores": ["STORE-001", "STORE-002"]
  },
  "fashionMetadata": { /* From Phase 1 */ }
}
```

**Response:** `201 Created`

### 5.2 Virtual Try-On Data

**Endpoint:** `GET /fashion/products/{sku}/tryon`

**Response:** `200 OK`
```json
{
  "sku": "FT-SW-001",
  "ar3dModel": "https://cdn.example.com/models/ft-sw-001.gltf",
  "dimensions": {
    "caseWidth": 44,
    "caseHeight": 44,
    "thickness": 11.5
  },
  "anchorPoints": [
    { "type": "wrist", "size": "medium", "offset": [0, 0, 0] }
  ]
}
```

### 5.3 Style Recommendations

**Endpoint:** `POST /fashion/recommendations`

**Request:**
```json
{
  "userId": "user-uuid",
  "context": {
    "occasion": "business",
    "season": "spring",
    "preferences": ["modern", "minimalist"]
  }
}
```

**Response:** `200 OK`
```json
{
  "recommendations": [
    {
      "sku": "FT-SW-001",
      "matchScore": 0.92,
      "reasons": ["matches style preference", "appropriate for business"]
    }
  ]
}
```

## 6. User Management APIs

### 6.1 Create User Profile

**Endpoint:** `POST /users`

**Request:**
```json
{
  "email": "user@example.com",
  "preferences": {
    "units": "metric",
    "language": "en",
    "timezone": "America/New_York"
  },
  "styleProfile": {
    "preferredStyles": ["modern", "classic"],
    "sizes": {
      "wrist": "medium",
      "ring": 8
    }
  }
}
```

**Response:** `201 Created`

### 6.2 Update User Preferences

**Endpoint:** `PUT /users/{userId}/preferences`

### 6.3 Data Export (GDPR Compliance)

**Endpoint:** `GET /users/{userId}/export`

**Response:** `200 OK`
- Returns complete user data archive in JSON format
- Includes all sensor data, settings, and metadata

### 6.4 Delete User Data

**Endpoint:** `DELETE /users/{userId}`

**Response:** `204 No Content`

## 7. Health and Fitness APIs

### 7.1 Daily Summary

**Endpoint:** `GET /health/summary/daily/{date}`

**Response:** `200 OK`
```json
{
  "date": "2025-01-15",
  "steps": 8420,
  "distance": 6.2,
  "activeMinutes": 45,
  "caloriesBurned": 420,
  "heartRate": {
    "resting": 65,
    "average": 78,
    "max": 142
  },
  "sleep": {
    "duration": 7.5,
    "quality": "good",
    "deepSleep": 2.1
  }
}
```

### 7.2 Workout Recording

**Endpoint:** `POST /health/workouts`

**Request:**
```json
{
  "type": "running",
  "startTime": "2025-01-15T07:00:00Z",
  "endTime": "2025-01-15T07:45:00Z",
  "distance": 7.5,
  "averageHeartRate": 145,
  "calories": 520,
  "route": { /* GeoJSON optional */ }
}
```

## 8. Notification APIs

### 8.1 Send Notification to Device

**Endpoint:** `POST /devices/{deviceId}/notifications`

**Request:**
```json
{
  "type": "message",
  "title": "New Message",
  "body": "You have a new message from...",
  "priority": "normal",
  "actions": [
    { "id": "reply", "label": "Reply" },
    { "id": "dismiss", "label": "Dismiss" }
  ],
  "vibration": true,
  "sound": false
}
```

**Response:** `202 Accepted`

## 9. Retail Integration APIs

### 9.1 Inventory Update

**Endpoint:** `PUT /retail/inventory/{sku}`

**Request:**
```json
{
  "sku": "FT-SW-001",
  "stores": [
    {
      "storeId": "STORE-001",
      "quantity": 25,
      "displayLocation": "Main Floor - Section A"
    }
  ]
}
```

### 9.2 In-Store Availability Check

**Endpoint:** `GET /retail/availability/{sku}?location={lat,lon}&radius={km}`

## 10. Firmware and Updates

### 10.1 Check for Updates

**Endpoint:** `GET /devices/{deviceId}/updates`

**Response:** `200 OK`
```json
{
  "updateAvailable": true,
  "version": "2.2.0",
  "releaseDate": "2025-01-10",
  "size": 15728640,
  "changelog": "Bug fixes and performance improvements",
  "downloadUrl": "https://cdn.example.com/firmware/2.2.0.bin",
  "mandatory": false
}
```

### 10.2 Report Update Status

**Endpoint:** `POST /devices/{deviceId}/updates/status`

## 11. Error Responses

### 11.1 Standard Error Format

```json
{
  "error": {
    "code": "INVALID_REQUEST",
    "message": "Human-readable error message",
    "details": {
      "field": "heartRate",
      "issue": "Value out of range"
    },
    "timestamp": "2025-01-15T14:30:00Z",
    "requestId": "req-uuid-123"
  }
}
```

### 11.2 HTTP Status Codes

- `200 OK`: Successful GET request
- `201 Created`: Successful POST creating new resource
- `204 No Content`: Successful DELETE
- `400 Bad Request`: Invalid request data
- `401 Unauthorized`: Missing or invalid authentication
- `403 Forbidden`: Authenticated but not authorized
- `404 Not Found`: Resource does not exist
- `429 Too Many Requests`: Rate limit exceeded
- `500 Internal Server Error`: Server error
- `503 Service Unavailable`: Temporary service outage

## 12. Rate Limiting

**Headers:**
```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 999
X-RateLimit-Reset: 1642262400
```

**Limits:**
- Device data upload: 1000 requests/hour
- Query APIs: 5000 requests/hour
- User management: 100 requests/hour

## 13. Webhooks

### 13.1 Register Webhook

**Endpoint:** `POST /webhooks`

**Request:**
```json
{
  "url": "https://your-app.com/webhook",
  "events": ["device.connected", "data.uploaded", "battery.low"],
  "secret": "webhook-secret-key"
}
```

### 13.2 Webhook Payload

```json
{
  "event": "battery.low",
  "timestamp": "2025-01-15T14:30:00Z",
  "data": {
    "deviceId": "WF-SW-12345",
    "batteryLevel": 10
  },
  "signature": "hmac-sha256-signature"
}
```

---

**Philosophy Note:** 弘益人間 (Benefit All Humanity)

These APIs benefit humanity by:
- Enabling seamless device integration across platforms
- Respecting user privacy and data ownership (GDPR compliance)
- Supporting health and wellness through accessible data
- Creating open, interoperable ecosystems

---

**© 2025 SmileStory Inc. / WIA**
**WIA-IND-003 Phase 2 Specification v1.0**
