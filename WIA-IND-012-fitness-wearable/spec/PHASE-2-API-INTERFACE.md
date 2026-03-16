# WIA-IND-012 PHASE 2: API INTERFACE SPECIFICATION
## Fitness Wearable Standard - 弘益人間 (Benefit All Humanity)

**Version:** 1.0
**Status:** Active
**Category:** IND (Industrial)
**Last Updated:** 2025-01-15

---

## 1. Introduction

Phase 2 of WIA-IND-012 defines comprehensive API interfaces for fitness wearable devices and platforms. The **弘益人間** (Benefit All Humanity) principle ensures that APIs are open, accessible, and enable innovation benefiting global populations.

### 1.1 Scope

- RESTful API specifications
- WebSocket real-time protocols
- Device-to-app communication
- Cloud platform APIs
- Third-party developer access
- Authentication and authorization

### 1.2 Design Goals

- **Developer-Friendly:** Intuitive, well-documented APIs
- **Secure:** OAuth 2.0 and modern security practices
- **Scalable:** Support millions of concurrent users
- **Extensible:** Easy addition of new endpoints
- **Standards-Based:** Follow REST and HTTP best practices

---

## 2. API Architecture

### 2.1 Base URL Structure

```
https://api.wia-fitness.org/v1/{resource}
```

- All APIs use HTTPS (TLS 1.3+)
- Version prefix enables evolution
- RESTful resource naming

### 2.2 Authentication

All API requests require authentication using OAuth 2.0:

```http
Authorization: Bearer {access_token}
```

### 2.3 Standard Headers

```http
Accept: application/json
Content-Type: application/json
X-WIA-Version: 1.0
X-WIA-Philosophy: 弘益人間
X-Request-ID: {uuid}
```

---

## 3. User Profile API

### 3.1 Get User Profile

**Endpoint:** `GET /v1/users/{userId}/profile`

**Request:**
```http
GET /v1/users/me/profile HTTP/1.1
Host: api.wia-fitness.org
Authorization: Bearer {token}
```

**Response:**
```json
{
  "userId": "user-123456",
  "profile": {
    "age": 35,
    "sex": "male",
    "height_cm": 175,
    "weight_kg": 70,
    "timezone": "America/Los_Angeles",
    "language": "en-US"
  },
  "preferences": {
    "units": "metric",
    "privacy_level": "standard",
    "data_sharing_consent": true
  },
  "philosophy": "弘益人間"
}
```

### 3.2 Update User Profile

**Endpoint:** `PATCH /v1/users/{userId}/profile`

**Request:**
```json
{
  "weight_kg": 68.5,
  "preferences": {
    "units": "imperial"
  }
}
```

**Response:**
```json
{
  "status": "success",
  "updated_fields": ["weight_kg", "preferences.units"],
  "timestamp": "2025-01-15T10:30:00.000Z"
}
```

---

## 4. Activity Data API

### 4.1 Get Daily Steps

**Endpoint:** `GET /v1/users/{userId}/activities/steps`

**Query Parameters:**
- `date`: YYYY-MM-DD format
- `start_date`: Range start
- `end_date`: Range end

**Request:**
```http
GET /v1/users/me/activities/steps?start_date=2025-01-01&end_date=2025-01-15
```

**Response:**
```json
{
  "standard": "WIA-IND-012",
  "philosophy": "弘益人間",
  "data": [
    {
      "date": "2025-01-15",
      "steps": 10247,
      "distance_km": 7.82,
      "calories": 520,
      "active_minutes": 145
    },
    {
      "date": "2025-01-14",
      "steps": 8532,
      "distance_km": 6.51,
      "calories": 435,
      "active_minutes": 118
    }
  ],
  "pagination": {
    "total_records": 15,
    "page": 1,
    "per_page": 30
  }
}
```

### 4.2 Create Activity Session

**Endpoint:** `POST /v1/users/{userId}/activities/sessions`

**Request:**
```json
{
  "activity_type": "running",
  "start_time": "2025-01-15T18:30:00.000Z",
  "duration_minutes": 45,
  "distance_km": 8.5,
  "avg_heart_rate": 152,
  "calories": 580,
  "gps_track": {
    "encoded_polyline": "encoded_string_here"
  }
}
```

**Response:**
```json
{
  "session_id": "session-789012",
  "status": "created",
  "timestamp": "2025-01-15T19:15:30.000Z",
  "philosophy": "弘益人間"
}
```

---

## 5. Heart Rate API

### 5.1 Get Heart Rate Data

**Endpoint:** `GET /v1/users/{userId}/heart-rate`

**Query Parameters:**
- `start_time`: ISO 8601 timestamp
- `end_time`: ISO 8601 timestamp
- `granularity`: "1min", "5min", "1hour", "1day"

**Request:**
```http
GET /v1/users/me/heart-rate?start_time=2025-01-15T00:00:00Z&end_time=2025-01-15T23:59:59Z&granularity=1min
```

**Response:**
```json
{
  "standard": "WIA-IND-012",
  "philosophy": "弘益人間",
  "data": [
    {
      "timestamp": "2025-01-15T10:30:00.000Z",
      "bpm": 75,
      "confidence": 0.95
    },
    {
      "timestamp": "2025-01-15T10:31:00.000Z",
      "bpm": 76,
      "confidence": 0.94
    }
  ],
  "summary": {
    "resting_hr": 58,
    "avg_hr": 72,
    "max_hr": 165,
    "min_hr": 54
  }
}
```

### 5.2 Get HRV Data

**Endpoint:** `GET /v1/users/{userId}/hrv`

**Response:**
```json
{
  "standard": "WIA-IND-012",
  "philosophy": "弘益人間",
  "data": [
    {
      "date": "2025-01-15",
      "sdnn_ms": 65.4,
      "rmssd_ms": 42.3,
      "lf_hf_ratio": 1.40,
      "quality": "good"
    }
  ]
}
```

---

## 6. Sleep API

### 6.1 Get Sleep Data

**Endpoint:** `GET /v1/users/{userId}/sleep`

**Query Parameters:**
- `date`: Specific date
- `start_date`, `end_date`: Range query

**Response:**
```json
{
  "standard": "WIA-IND-012",
  "philosophy": "弘益人間",
  "data": [
    {
      "date": "2025-01-15",
      "sleep_start": "2025-01-14T23:15:00.000Z",
      "sleep_end": "2025-01-15T07:30:00.000Z",
      "total_sleep_minutes": 465,
      "sleep_efficiency": 0.88,
      "light_sleep_minutes": 230,
      "deep_sleep_minutes": 115,
      "rem_sleep_minutes": 120,
      "sleep_score": 85
    }
  ]
}
```

---

## 7. Device Management API

### 7.1 List Devices

**Endpoint:** `GET /v1/users/{userId}/devices`

**Response:**
```json
{
  "devices": [
    {
      "device_id": "WIA-DEVICE-001-ABC123",
      "model": "WIA Fitness Pro",
      "manufacturer": "WIA Official",
      "firmware_version": "2.1.0",
      "battery_level": 0.75,
      "last_sync": "2025-01-15T10:30:00.000Z",
      "status": "active"
    }
  ],
  "philosophy": "弘益人間"
}
```

### 7.2 Sync Device Data

**Endpoint:** `POST /v1/devices/{deviceId}/sync`

**Request:**
```json
{
  "data_types": ["heart_rate", "steps", "sleep"],
  "sync_from": "2025-01-15T00:00:00.000Z",
  "sync_to": "2025-01-15T23:59:59.000Z"
}
```

**Response:**
```json
{
  "sync_id": "sync-456789",
  "status": "processing",
  "records_synced": 1440,
  "estimated_completion": "2025-01-15T10:35:00.000Z"
}
```

---

## 8. Real-Time Streaming API (WebSocket)

### 8.1 Connection

**Endpoint:** `wss://stream.wia-fitness.org/v1/live`

**Authentication:**
```
wss://stream.wia-fitness.org/v1/live?token={access_token}
```

### 8.2 Subscribe to Heart Rate Stream

**Subscribe Message:**
```json
{
  "action": "subscribe",
  "channel": "heart_rate",
  "user_id": "user-123456"
}
```

**Stream Messages:**
```json
{
  "channel": "heart_rate",
  "data": {
    "timestamp": "2025-01-15T10:30:45.123Z",
    "bpm": 75,
    "confidence": 0.95
  },
  "philosophy": "弘益人間"
}
```

---

## 9. Goals and Achievements API

### 9.1 Get Goals

**Endpoint:** `GET /v1/users/{userId}/goals`

**Response:**
```json
{
  "goals": [
    {
      "goal_id": "goal-123",
      "type": "daily_steps",
      "target": 10000,
      "current": 7820,
      "progress": 0.78,
      "status": "active"
    },
    {
      "goal_id": "goal-124",
      "type": "weekly_exercise_minutes",
      "target": 150,
      "current": 105,
      "progress": 0.70,
      "status": "active"
    }
  ]
}
```

### 9.2 Create Goal

**Endpoint:** `POST /v1/users/{userId}/goals`

**Request:**
```json
{
  "type": "daily_steps",
  "target": 12000,
  "start_date": "2025-01-16"
}
```

---

## 10. Data Export API

### 10.1 Request Data Export

**Endpoint:** `POST /v1/users/{userId}/export`

**Request:**
```json
{
  "format": "json",
  "data_types": ["all"],
  "start_date": "2024-01-01",
  "end_date": "2024-12-31",
  "include_raw_data": false
}
```

**Response:**
```json
{
  "export_id": "export-987654",
  "status": "processing",
  "estimated_completion": "2025-01-15T11:00:00.000Z",
  "download_url": null
}
```

### 10.2 Check Export Status

**Endpoint:** `GET /v1/users/{userId}/export/{exportId}`

**Response:**
```json
{
  "export_id": "export-987654",
  "status": "completed",
  "download_url": "https://downloads.wia-fitness.org/exports/user-123456/export-987654.zip",
  "expires_at": "2025-01-22T11:00:00.000Z",
  "file_size_mb": 45.2
}
```

---

## 11. OAuth 2.0 Authentication

### 11.1 Authorization Request

**Endpoint:** `GET /oauth/authorize`

**Parameters:**
```
https://auth.wia-fitness.org/oauth/authorize?
  response_type=code&
  client_id={client_id}&
  redirect_uri={redirect_uri}&
  scope=read:profile+read:activities+write:activities&
  state={random_string}
```

### 11.2 Token Exchange

**Endpoint:** `POST /oauth/token`

**Request:**
```http
POST /oauth/token HTTP/1.1
Content-Type: application/x-www-form-urlencoded

grant_type=authorization_code&
code={authorization_code}&
redirect_uri={redirect_uri}&
client_id={client_id}&
client_secret={client_secret}
```

**Response:**
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "refresh_token": "def50200...",
  "scope": "read:profile read:activities write:activities"
}
```

---

## 12. Error Handling

### 12.1 Standard Error Format

```json
{
  "error": {
    "code": "INVALID_REQUEST",
    "message": "Missing required parameter: start_date",
    "details": {
      "parameter": "start_date",
      "requirement": "ISO 8601 date format"
    },
    "request_id": "req-123456",
    "philosophy": "弘益人間"
  }
}
```

### 12.2 HTTP Status Codes

- `200 OK`: Success
- `201 Created`: Resource created
- `400 Bad Request`: Invalid request
- `401 Unauthorized`: Authentication required
- `403 Forbidden`: Insufficient permissions
- `404 Not Found`: Resource not found
- `429 Too Many Requests`: Rate limit exceeded
- `500 Internal Server Error`: Server error

---

## 13. Rate Limiting

### 13.1 Rate Limits

- **Standard tier:** 1000 requests/hour per user
- **Developer tier:** 10,000 requests/hour
- **Enterprise tier:** Custom limits

### 13.2 Rate Limit Headers

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 742
X-RateLimit-Reset: 1642262400
```

---

## 14. Webhooks

### 14.1 Register Webhook

**Endpoint:** `POST /v1/webhooks`

**Request:**
```json
{
  "url": "https://your-app.com/webhooks/wia",
  "events": ["activity.created", "sleep.updated"],
  "secret": "webhook_secret_key"
}
```

### 14.2 Webhook Payload

```json
{
  "event": "activity.created",
  "timestamp": "2025-01-15T19:15:30.000Z",
  "user_id": "user-123456",
  "data": {
    "session_id": "session-789012",
    "activity_type": "running"
  },
  "philosophy": "弘益人間"
}
```

---

## 15. SDK and Libraries

WIA provides official SDKs for:
- **JavaScript/TypeScript:** `npm install @wia/fitness-sdk`
- **Python:** `pip install wia-fitness`
- **Swift:** CocoaPods, SPM
- **Kotlin:** Maven Central
- **Go:** `go get github.com/wia/fitness-sdk-go`

### 15.1 JavaScript Example

```javascript
import { WIAFitnessClient } from '@wia/fitness-sdk';

const client = new WIAFitnessClient({
  accessToken: 'your_token_here',
  philosophy: '弘益人間'
});

const steps = await client.activities.getSteps({
  startDate: '2025-01-01',
  endDate: '2025-01-15'
});
```

---

## 16. Conclusion

Phase 2 of WIA-IND-012 provides comprehensive, secure, developer-friendly APIs enabling rich third-party applications and integrations. By adhering to these specifications, developers contribute to the **弘益人間** vision of technology benefiting all humanity through innovation and interoperability.

---

**Document Control**

- Version: 1.0
- Effective Date: 2025-01-15
- Next Review: 2026-01-15
- Contact: standards@wia.org

**Copyright © 2025 SmileStory Inc. / WIA**
**弘益人間 (Benefit All Humanity)**
