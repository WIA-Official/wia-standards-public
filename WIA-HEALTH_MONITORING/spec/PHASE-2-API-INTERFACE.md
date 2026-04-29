# WIA-HEALTH_MONITORING: Phase 2 - API Interface Specification

**Version**: 1.0
**Status**: Draft
**Last Updated**: 2026-01-12
**Authors**: WIA Technical Committee

---

## 1. Overview

This document defines the RESTful API interface for health monitoring systems, enabling standardized communication between devices, applications, and healthcare systems.

### 1.1 API Design Principles

1. **RESTful Architecture**: Resource-oriented design
2. **Stateless**: No server-side session state
3. **HATEOAS**: Hypermedia-driven navigation
4. **Idempotent Operations**: Safe retry mechanisms
5. **Rate Limiting**: Prevent abuse and ensure fair usage

### 1.2 Base URL Structure

```
https://api.wia-health.org/v1
```

### 1.3 Authentication

All API requests require authentication using OAuth 2.0 or API keys:

```http
Authorization: Bearer <access_token>
X-API-Key: <api_key>
```

---

## 2. Core API Endpoints

### 2.1 Health Metrics

#### POST /metrics
Submit new health metric data.

**Request:**
```http
POST /v1/metrics HTTP/1.1
Host: api.wia-health.org
Authorization: Bearer eyJhbGciOiJIUzI1NiIs...
Content-Type: application/json

{
  "metric_type": "HEART_RATE",
  "timestamp": "2026-01-12T14:35:22.123+00:00",
  "device_id": "apple-watch-series-9-abc123",
  "value": {
    "bpm": 72,
    "variability_ms": 45.3
  },
  "quality_score": 0.98
}
```

**Response (201 Created):**
```json
{
  "metric_id": "550e8400-e29b-41d4-a716-446655440000",
  "status": "accepted",
  "timestamp": "2026-01-12T14:35:23.456+00:00",
  "links": {
    "self": "/v1/metrics/550e8400-e29b-41d4-a716-446655440000"
  }
}
```

**Error Response (400 Bad Request):**
```json
{
  "error": "VALIDATION_ERROR",
  "message": "Invalid metric type",
  "details": {
    "field": "metric_type",
    "allowed_values": ["HEART_RATE", "BLOOD_GLUCOSE", "SPO2", ...]
  }
}
```

#### POST /metrics/batch
Submit multiple metrics in a single request.

**Request:**
```http
POST /v1/metrics/batch HTTP/1.1
Content-Type: application/json

{
  "device_id": "fitbit-sense-2-xyz789",
  "metrics": [
    {
      "metric_type": "HEART_RATE",
      "timestamp": "2026-01-12T14:30:00+00:00",
      "value": {"bpm": 70}
    },
    {
      "metric_type": "STEPS",
      "timestamp": "2026-01-12T14:30:00+00:00",
      "value": {"step_count": 8542}
    }
  ]
}
```

**Response (201 Created):**
```json
{
  "batch_id": "batch-uuid-123",
  "accepted_count": 2,
  "rejected_count": 0,
  "results": [
    {
      "index": 0,
      "status": "accepted",
      "metric_id": "metric-uuid-1"
    },
    {
      "index": 1,
      "status": "accepted",
      "metric_id": "metric-uuid-2"
    }
  ]
}
```

#### GET /metrics/:metric_id
Retrieve a specific metric by ID.

**Request:**
```http
GET /v1/metrics/550e8400-e29b-41d4-a716-446655440000 HTTP/1.1
Authorization: Bearer <token>
```

**Response (200 OK):**
```json
{
  "metric_id": "550e8400-e29b-41d4-a716-446655440000",
  "metric_type": "HEART_RATE",
  "timestamp": "2026-01-12T14:35:22.123+00:00",
  "value": {
    "bpm": 72,
    "variability_ms": 45.3,
    "resting_hr": 65
  },
  "quality_score": 0.98,
  "metadata": {
    "device_model": "Apple Watch Series 9",
    "activity_context": "RESTING"
  }
}
```

#### GET /metrics
Query metrics with filters.

**Request:**
```http
GET /v1/metrics?metric_type=HEART_RATE&start=2026-01-12T00:00:00Z&end=2026-01-12T23:59:59Z&limit=100&offset=0 HTTP/1.1
Authorization: Bearer <token>
```

**Query Parameters:**
| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| metric_type | string | No | Filter by metric type |
| start | ISO8601 | No | Start timestamp |
| end | ISO8601 | No | End timestamp |
| device_id | string | No | Filter by device |
| limit | integer | No | Results per page (default: 50, max: 500) |
| offset | integer | No | Pagination offset |
| quality_min | float | No | Minimum quality score |

**Response (200 OK):**
```json
{
  "metrics": [
    {
      "metric_id": "uuid-1",
      "metric_type": "HEART_RATE",
      "timestamp": "2026-01-12T14:35:22Z",
      "value": {"bpm": 72}
    },
    {
      "metric_id": "uuid-2",
      "metric_type": "HEART_RATE",
      "timestamp": "2026-01-12T15:35:22Z",
      "value": {"bpm": 75}
    }
  ],
  "pagination": {
    "total_count": 1247,
    "limit": 100,
    "offset": 0,
    "next": "/v1/metrics?offset=100&limit=100",
    "prev": null
  }
}
```

#### DELETE /metrics/:metric_id
Delete a specific metric (soft delete).

**Request:**
```http
DELETE /v1/metrics/550e8400-e29b-41d4-a716-446655440000 HTTP/1.1
Authorization: Bearer <token>
```

**Response (204 No Content)**

---

### 2.2 User Profile and Health Data

#### GET /users/me
Get current user's profile and health summary.

**Response (200 OK):**
```json
{
  "user_id": "anon-user-7f8d9e0a",
  "created_at": "2025-06-15T10:00:00Z",
  "profile": {
    "age_range": "30-40",
    "gender": "PREFER_NOT_TO_SAY",
    "height_cm": 175,
    "weight_kg": 70,
    "timezone": "America/New_York"
  },
  "health_summary": {
    "last_updated": "2026-01-12T14:35:00Z",
    "active_devices": 2,
    "total_data_points": 125847
  },
  "consent": {
    "data_sharing": true,
    "research_participation": false,
    "third_party_access": false
  }
}
```

#### PATCH /users/me
Update user profile or consent settings.

**Request:**
```http
PATCH /v1/users/me HTTP/1.1
Content-Type: application/json

{
  "profile": {
    "weight_kg": 72
  },
  "consent": {
    "research_participation": true
  }
}
```

**Response (200 OK):**
```json
{
  "user_id": "anon-user-7f8d9e0a",
  "updated_fields": ["profile.weight_kg", "consent.research_participation"],
  "updated_at": "2026-01-12T14:40:00Z"
}
```

---

### 2.3 Device Management

#### GET /devices
List all registered devices for the user.

**Response (200 OK):**
```json
{
  "devices": [
    {
      "device_id": "apple-watch-series-9-abc123",
      "device_type": "SMARTWATCH",
      "manufacturer": "Apple",
      "model": "Watch Series 9",
      "firmware_version": "10.2.1",
      "registered_at": "2025-08-20T12:00:00Z",
      "last_sync": "2026-01-12T14:30:00Z",
      "capabilities": ["HEART_RATE", "ECG", "SPO2", "STEPS", "SLEEP"],
      "battery_level": 0.82,
      "status": "ACTIVE"
    },
    {
      "device_id": "freestyle-libre-3-def456",
      "device_type": "CGM",
      "manufacturer": "Abbott",
      "model": "FreeStyle Libre 3",
      "sensor_insertion_date": "2026-01-05T09:00:00Z",
      "sensor_expiry_date": "2026-01-19T09:00:00Z",
      "capabilities": ["BLOOD_GLUCOSE"],
      "status": "ACTIVE"
    }
  ],
  "total_count": 2
}
```

#### POST /devices
Register a new device.

**Request:**
```json
{
  "device_id": "garmin-fenix-7-ghi789",
  "device_type": "SMARTWATCH",
  "manufacturer": "Garmin",
  "model": "Fenix 7",
  "capabilities": ["HEART_RATE", "SPO2", "STEPS", "SLEEP"]
}
```

**Response (201 Created):**
```json
{
  "device_id": "garmin-fenix-7-ghi789",
  "status": "registered",
  "registered_at": "2026-01-12T14:45:00Z",
  "api_key": "device-specific-api-key-for-submission"
}
```

#### DELETE /devices/:device_id
Unregister a device.

**Response (204 No Content)**

---

### 2.4 Health Summaries

#### GET /summaries/daily/:date
Get daily health summary.

**Request:**
```http
GET /v1/summaries/daily/2026-01-12 HTTP/1.1
```

**Response (200 OK):**
```json
{
  "date": "2026-01-12",
  "summary_type": "DAILY",
  "metrics": {
    "heart_rate": {
      "avg": 72,
      "min": 58,
      "max": 145,
      "resting": 65,
      "data_points": 1440
    },
    "sleep": {
      "total_minutes": 456,
      "efficiency": 92.5,
      "deep_sleep_minutes": 95,
      "rem_sleep_minutes": 110,
      "light_sleep_minutes": 220,
      "awake_minutes": 31,
      "sleep_score": 85
    },
    "activity": {
      "steps": 8542,
      "distance_km": 6.834,
      "calories": 342,
      "active_minutes": 67,
      "floors": 12
    },
    "glucose": {
      "avg_mg_dl": 105,
      "time_in_range_percent": 78.5,
      "hypoglycemic_events": 0,
      "hyperglycemic_events": 2
    }
  },
  "alerts": [
    {
      "alert_type": "HIGH_HEART_RATE",
      "timestamp": "2026-01-12T16:30:00Z",
      "severity": "WARNING",
      "message": "Elevated heart rate detected: 145 bpm during resting"
    }
  ],
  "completeness_score": 0.95
}
```

#### GET /summaries/weekly/:year/:week
Get weekly health summary.

**Request:**
```http
GET /v1/summaries/weekly/2026/02 HTTP/1.1
```

**Response:** Similar to daily, aggregated over 7 days.

#### GET /summaries/monthly/:year/:month
Get monthly health summary.

**Request:**
```http
GET /v1/summaries/monthly/2026/01 HTTP/1.1
```

**Response:** Similar to daily, aggregated over 30/31 days.

---

### 2.5 Alerts and Notifications

#### GET /alerts
Get user's health alerts.

**Request:**
```http
GET /v1/alerts?start=2026-01-01&severity=WARNING,CRITICAL&status=UNREAD&limit=50 HTTP/1.1
```

**Query Parameters:**
| Parameter | Type | Options |
|-----------|------|---------|
| start | ISO8601 | Start date |
| end | ISO8601 | End date |
| severity | string | INFO, WARNING, CRITICAL |
| status | string | UNREAD, READ, DISMISSED |
| alert_type | string | Specific alert type |

**Response (200 OK):**
```json
{
  "alerts": [
    {
      "alert_id": "alert-uuid-123",
      "alert_type": "LOW_GLUCOSE",
      "timestamp": "2026-01-12T03:15:00Z",
      "severity": "CRITICAL",
      "status": "UNREAD",
      "message": "Low blood glucose detected: 65 mg/dL",
      "metric_id": "metric-uuid-456",
      "recommended_action": "Check glucose immediately and consume fast-acting carbs if below 70 mg/dL"
    }
  ],
  "unread_count": 3,
  "total_count": 15
}
```

#### PATCH /alerts/:alert_id
Update alert status (mark as read/dismissed).

**Request:**
```json
{
  "status": "READ"
}
```

**Response (200 OK):**
```json
{
  "alert_id": "alert-uuid-123",
  "status": "READ",
  "updated_at": "2026-01-12T14:50:00Z"
}
```

#### POST /alerts/config
Configure alert thresholds and preferences.

**Request:**
```json
{
  "alert_rules": [
    {
      "metric_type": "HEART_RATE",
      "condition": "GREATER_THAN",
      "threshold": 140,
      "duration_seconds": 300,
      "severity": "WARNING",
      "notification_channels": ["PUSH", "SMS"]
    },
    {
      "metric_type": "BLOOD_GLUCOSE",
      "condition": "LESS_THAN",
      "threshold": 70,
      "severity": "CRITICAL",
      "notification_channels": ["PUSH", "SMS", "EMAIL"]
    }
  ]
}
```

**Response (200 OK):**
```json
{
  "config_id": "config-uuid-789",
  "active_rules": 2,
  "updated_at": "2026-01-12T15:00:00Z"
}
```

---

### 2.6 Data Export

#### GET /export/metrics
Export health data in various formats.

**Request:**
```http
GET /v1/export/metrics?format=CSV&start=2026-01-01&end=2026-01-31&metric_types=HEART_RATE,STEPS HTTP/1.1
```

**Query Parameters:**
| Parameter | Options |
|-----------|---------|
| format | CSV, JSON, FHIR, PDF |
| start | ISO8601 date |
| end | ISO8601 date |
| metric_types | Comma-separated list |
| include_metadata | true/false |

**Response (200 OK):**
```json
{
  "export_id": "export-uuid-abc",
  "status": "PROCESSING",
  "estimated_completion": "2026-01-12T15:10:00Z",
  "download_url": null,
  "expires_at": null
}
```

#### GET /export/:export_id
Check export status and download.

**Response (200 OK - Ready):**
```json
{
  "export_id": "export-uuid-abc",
  "status": "COMPLETED",
  "download_url": "https://cdn.wia-health.org/exports/export-uuid-abc.csv",
  "file_size_bytes": 524288,
  "expires_at": "2026-01-19T15:10:00Z"
}
```

---

### 2.7 Integration with EMR/EHR

#### POST /integrations/emr
Connect to an EMR/EHR system.

**Request:**
```json
{
  "system_type": "EPIC | CERNER | ALLSCRIPTS | FHIR_GENERIC",
  "credentials": {
    "client_id": "emr-client-id",
    "client_secret": "emr-client-secret"
  },
  "sync_preferences": {
    "auto_sync": true,
    "sync_interval_hours": 24,
    "metric_types": ["BLOOD_GLUCOSE", "BLOOD_PRESSURE", "WEIGHT"]
  }
}
```

**Response (201 Created):**
```json
{
  "integration_id": "integration-uuid-def",
  "status": "CONNECTED",
  "last_sync": null,
  "next_sync": "2026-01-13T15:00:00Z"
}
```

#### GET /integrations
List all active integrations.

**Response (200 OK):**
```json
{
  "integrations": [
    {
      "integration_id": "integration-uuid-def",
      "system_type": "EPIC",
      "status": "CONNECTED",
      "created_at": "2026-01-12T15:00:00Z",
      "last_sync": "2026-01-12T15:30:00Z",
      "sync_count": 1,
      "error_count": 0
    }
  ]
}
```

#### POST /integrations/:integration_id/sync
Trigger manual sync to EMR.

**Response (202 Accepted):**
```json
{
  "sync_id": "sync-uuid-ghi",
  "status": "IN_PROGRESS",
  "started_at": "2026-01-12T15:35:00Z"
}
```

---

## 3. Real-Time Streaming API

### 3.1 WebSocket Connection

For real-time health monitoring:

**Connection:**
```javascript
const ws = new WebSocket('wss://stream.wia-health.org/v1/stream');

ws.on('open', () => {
  ws.send(JSON.stringify({
    action: 'authenticate',
    token: 'bearer-token'
  }));

  ws.send(JSON.stringify({
    action: 'subscribe',
    metric_types: ['HEART_RATE', 'BLOOD_GLUCOSE'],
    device_ids: ['apple-watch-series-9-abc123']
  }));
});

ws.on('message', (data) => {
  const metric = JSON.parse(data);
  console.log('Real-time metric:', metric);
});
```

**Message Format:**
```json
{
  "event_type": "METRIC_UPDATE",
  "metric_id": "uuid",
  "metric_type": "HEART_RATE",
  "timestamp": "2026-01-12T15:40:00Z",
  "value": {"bpm": 72},
  "quality_score": 0.98
}
```

### 3.2 Server-Sent Events (SSE)

Alternative to WebSocket for unidirectional streaming:

```http
GET /v1/stream/sse?metric_types=HEART_RATE HTTP/1.1
Authorization: Bearer <token>
Accept: text/event-stream
```

**Response:**
```
data: {"metric_type":"HEART_RATE","value":{"bpm":72},"timestamp":"2026-01-12T15:40:00Z"}

data: {"metric_type":"HEART_RATE","value":{"bpm":73},"timestamp":"2026-01-12T15:40:05Z"}
```

---

## 4. Rate Limiting

### 4.1 Rate Limit Headers

All API responses include rate limit information:

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 987
X-RateLimit-Reset: 1705069200
```

### 4.2 Rate Limit Tiers

| Tier | Requests/Hour | Burst | Cost |
|------|---------------|-------|------|
| Free | 1,000 | 50 | $0 |
| Basic | 10,000 | 200 | $29/mo |
| Pro | 100,000 | 1,000 | $99/mo |
| Enterprise | Unlimited | Custom | Custom |

### 4.3 Rate Limit Error

**Response (429 Too Many Requests):**
```json
{
  "error": "RATE_LIMIT_EXCEEDED",
  "message": "API rate limit exceeded",
  "retry_after_seconds": 3600,
  "limit": 1000,
  "window": "1 hour"
}
```

---

## 5. Error Handling

### 5.1 Error Response Format

```json
{
  "error": "ERROR_CODE",
  "message": "Human-readable error message",
  "details": {
    "field": "specific_field",
    "reason": "detailed reason"
  },
  "request_id": "req-uuid-123",
  "timestamp": "2026-01-12T15:45:00Z"
}
```

### 5.2 Standard Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| AUTHENTICATION_REQUIRED | 401 | Missing or invalid auth token |
| AUTHORIZATION_FAILED | 403 | Insufficient permissions |
| RESOURCE_NOT_FOUND | 404 | Requested resource doesn't exist |
| VALIDATION_ERROR | 400 | Invalid request data |
| RATE_LIMIT_EXCEEDED | 429 | Too many requests |
| INTERNAL_ERROR | 500 | Server error |
| SERVICE_UNAVAILABLE | 503 | Temporary outage |
| DEVICE_OFFLINE | 422 | Device not connected |
| DATA_QUALITY_LOW | 422 | Metric quality below threshold |

---

## 6. API Versioning

### 6.1 Version Strategy

- URL-based versioning: `/v1`, `/v2`
- Minimum support: 2 years per version
- Deprecation notice: 6 months before EOL
- Breaking changes require new version

### 6.2 Version Headers

```http
X-API-Version: 1.0
X-API-Deprecated: false
X-API-Sunset: 2028-01-12T00:00:00Z
```

---

## 7. Security

### 7.1 HTTPS Only

All API endpoints MUST use HTTPS. HTTP requests are automatically redirected.

### 7.2 CORS Policy

```http
Access-Control-Allow-Origin: https://app.wia-health.org
Access-Control-Allow-Methods: GET, POST, PATCH, DELETE
Access-Control-Allow-Headers: Authorization, Content-Type
Access-Control-Max-Age: 3600
```

### 7.3 Content Security

- Input sanitization on all endpoints
- SQL injection prevention
- XSS protection
- CSRF tokens for state-changing operations

---

## 8. SDK Examples

### 8.1 JavaScript/TypeScript

```typescript
import { WIAHealthClient } from '@wia/health-monitoring-sdk';

const client = new WIAHealthClient({
  apiKey: 'your-api-key',
  environment: 'production'
});

// Submit metric
const metric = await client.metrics.create({
  metric_type: 'HEART_RATE',
  timestamp: new Date().toISOString(),
  value: { bpm: 72 }
});

// Query metrics
const heartRateData = await client.metrics.query({
  metric_type: 'HEART_RATE',
  start: '2026-01-01T00:00:00Z',
  end: '2026-01-31T23:59:59Z'
});

// Real-time streaming
client.stream.subscribe(['HEART_RATE'], (metric) => {
  console.log('New heart rate:', metric.value.bpm);
});
```

### 8.2 Python

```python
from wia_health import WIAHealthClient

client = WIAHealthClient(api_key='your-api-key')

# Submit metric
metric = client.metrics.create({
    'metric_type': 'BLOOD_GLUCOSE',
    'timestamp': '2026-01-12T15:50:00Z',
    'value': {'glucose_mg_dl': 105}
})

# Get daily summary
summary = client.summaries.get_daily('2026-01-12')
print(f"Average heart rate: {summary['metrics']['heart_rate']['avg']}")
```

---

## 9. Webhook Integration

### 9.1 Webhook Registration

```http
POST /v1/webhooks HTTP/1.1
Content-Type: application/json

{
  "url": "https://your-app.com/webhooks/wia-health",
  "events": ["METRIC_CREATED", "ALERT_TRIGGERED", "DAILY_SUMMARY_READY"],
  "secret": "webhook-secret-for-signature-verification"
}
```

### 9.2 Webhook Payload

```json
{
  "event_id": "event-uuid-123",
  "event_type": "ALERT_TRIGGERED",
  "timestamp": "2026-01-12T16:00:00Z",
  "data": {
    "alert_id": "alert-uuid-456",
    "alert_type": "HIGH_HEART_RATE",
    "severity": "WARNING",
    "user_id": "anon-user-7f8d9e0a"
  },
  "signature": "sha256-hmac-signature"
}
```

---

## 10. Testing and Sandbox

### 10.1 Sandbox Environment

```
https://sandbox-api.wia-health.org/v1
```

- Pre-populated test data
- No rate limits
- Simulated devices
- Reset daily at 00:00 UTC

### 10.2 Test Credentials

```
API Key: test_sk_123456789
User ID: test-user-sandbox
Device ID: test-device-apple-watch
```

---

**弘益人間 (홍익인간)** - Benefit All Humanity

© 2026 WIA (World Certification Industry Association)
