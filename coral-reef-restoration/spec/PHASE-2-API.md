# WIA Coral Reef Restoration - Phase 2: API Interface

**Version:** 1.0.0
**Status:** Complete
**Last Updated:** 2025-12-25

---

## 1. Overview

This specification defines the REST API interface for coral reef restoration monitoring, health assessment, and data exchange. The API enables real-time data submission, querying, and analysis for marine conservation stakeholders.

### 1.1 Base URL

```
Production: https://api.coral-reef.wiastandards.org/v1
Staging: https://staging-api.coral-reef.wiastandards.org/v1
```

### 1.2 Protocol

- **Transport:** HTTPS only (TLS 1.3+)
- **Format:** JSON (application/json)
- **Encoding:** UTF-8
- **Authentication:** OAuth 2.0 + Bearer tokens

---

## 2. Authentication

### 2.1 OAuth 2.0 Flow

```http
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials
&client_id={your_client_id}
&client_secret={your_client_secret}
&scope=reef:read reef:write
```

**Response:**
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "reef:read reef:write"
}
```

### 2.2 Using Access Token

```http
GET /reefs/{reefId}
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
```

### 2.3 API Scopes

| Scope | Description |
|-------|-------------|
| `reef:read` | Read reef health data |
| `reef:write` | Submit monitoring data |
| `reef:admin` | Administrative operations |
| `restoration:read` | View restoration activities |
| `restoration:write` | Log restoration activities |
| `alerts:read` | Receive bleaching alerts |
| `reports:generate` | Generate reports |

---

## 3. Reef Monitoring Endpoints

### 3.1 Submit Reef Health Data

**Endpoint:** `POST /reefs/monitor`

**Request:**
```json
{
  "reefId": "REEF-GBR-A0123",
  "location": {
    "name": "Great Barrier Reef, Zone A",
    "coordinates": {
      "latitude": -16.2456,
      "longitude": 145.7823
    },
    "depth": 12.5
  },
  "timestamp": "2025-12-25T08:30:00Z",
  "observer": {
    "id": "OBS-001",
    "name": "Dr. Sarah Ocean",
    "organization": "Marine Research Institute"
  },
  "metrics": {
    "coralCoverage": 65.5,
    "speciesDiversity": 42,
    "waterTemperature": 28.5,
    "bleachingStatus": "none",
    "phLevel": 8.1,
    "salinity": 35.2
  }
}
```

**Response (201 Created):**
```json
{
  "status": "success",
  "recordId": "MON-2025-12-25-00123",
  "healthScore": 92.5,
  "alerts": [],
  "recommendations": [
    "Continue routine monitoring",
    "Excellent coral health maintained"
  ],
  "nextMonitoring": "2025-12-26T08:30:00Z"
}
```

### 3.2 Get Reef Health Status

**Endpoint:** `GET /reefs/{reefId}/health`

**Query Parameters:**
- `period` (optional): `day`, `week`, `month`, `year`, `all`
- `include_history` (optional): `true` | `false`

**Response (200 OK):**
```json
{
  "reefId": "REEF-GBR-A0123",
  "currentHealth": {
    "healthScore": 92.5,
    "coralCoverage": 65.5,
    "speciesDiversity": 42,
    "waterTemperature": 28.5,
    "bleachingStatus": "none",
    "lastUpdated": "2025-12-25T08:30:00Z"
  },
  "trend": {
    "direction": "stable",
    "changeRate": 0.2,
    "period": "30d"
  },
  "history": [
    {
      "date": "2025-12-24",
      "healthScore": 92.3
    },
    {
      "date": "2025-12-23",
      "healthScore": 92.1
    }
  ]
}
```

### 3.3 List All Reefs

**Endpoint:** `GET /reefs`

**Query Parameters:**
- `zone` (optional): Filter by management zone
- `min_health` (optional): Minimum health score (0-100)
- `bleaching_status` (optional): Filter by bleaching status
- `limit` (optional): Results per page (default: 50, max: 100)
- `offset` (optional): Pagination offset

**Response (200 OK):**
```json
{
  "total": 1247,
  "offset": 0,
  "limit": 50,
  "reefs": [
    {
      "reefId": "REEF-GBR-A0123",
      "name": "Great Barrier Reef, Zone A",
      "healthScore": 92.5,
      "bleachingStatus": "none",
      "lastMonitored": "2025-12-25T08:30:00Z"
    }
  ]
}
```

---

## 4. Bleaching Prediction Endpoints

### 4.1 Predict Bleaching Risk

**Endpoint:** `POST /reefs/predict-bleaching`

**Request:**
```json
{
  "reefId": "REEF-GBR-A0123",
  "currentTemperature": 29.5,
  "historicalAverage": 27.0,
  "daysElevated": 14,
  "forecast": {
    "days": 7,
    "expectedTemperature": 30.2
  }
}
```

**Response (200 OK):**
```json
{
  "reefId": "REEF-GBR-A0123",
  "prediction": {
    "bleachingRisk": "HIGH",
    "riskScore": 78.5,
    "confidence": 0.89,
    "temperatureAnomaly": 2.5,
    "degreeHeatingWeeks": 4.2
  },
  "recommendation": {
    "action": "IMMEDIATE_MONITORING",
    "frequency": "daily",
    "mitigation": [
      "Reduce visitor access",
      "Monitor water quality",
      "Prepare emergency response team"
    ]
  },
  "forecast": {
    "day7risk": 85.2,
    "day14risk": 91.5,
    "expectedPeakRisk": "2026-01-03"
  }
}
```

### 4.2 Get Active Bleaching Alerts

**Endpoint:** `GET /reefs/alerts`

**Query Parameters:**
- `severity` (optional): `mild`, `moderate`, `severe`
- `region` (optional): Geographic region filter
- `active_only` (optional): `true` | `false`

**Response (200 OK):**
```json
{
  "total": 23,
  "alerts": [
    {
      "alertId": "ALERT-2025-12-25-001",
      "reefId": "REEF-CAR-B0456",
      "severity": "MODERATE",
      "issuedAt": "2025-12-25T10:00:00Z",
      "message": "Temperature elevated 3.2°C above average for 14 days",
      "affectedArea": 2.5,
      "recommendedAction": "Increase monitoring frequency"
    }
  ]
}
```

---

## 5. Restoration Activity Endpoints

### 5.1 Log Restoration Activity

**Endpoint:** `POST /reefs/restoration`

**Request:**
```json
{
  "reefId": "REEF-INDO-C0789",
  "organization": {
    "id": "ORG-CORAL-456",
    "name": "Coral Triangle Restoration Project",
    "type": "conservation_ngo"
  },
  "restoration": {
    "type": "coral_gardening",
    "areaRestored": 2.5,
    "coralsPlanted": 1500,
    "speciesPlanted": [
      "Acropora millepora",
      "Pocillopora damicornis"
    ],
    "method": "Mid-water rope nursery",
    "startDate": "2024-06-15",
    "completionDate": "2024-07-20",
    "cost": 15000
  }
}
```

**Response (201 Created):**
```json
{
  "status": "success",
  "activityId": "ACT-REST-2025-1234",
  "trackingUrl": "https://api.coral-reef.wiastandards.org/v1/restoration/ACT-REST-2025-1234",
  "certificate": {
    "id": "CERT-2025-1234",
    "downloadUrl": "https://cert.wiastandards.org/coral/CERT-2025-1234"
  }
}
```

### 5.2 Get Restoration Progress

**Endpoint:** `GET /reefs/{reefId}/restoration`

**Query Parameters:**
- `since` (optional): ISO 8601 date
- `include_metrics` (optional): `true` | `false`

**Response (200 OK):**
```json
{
  "reefId": "REEF-INDO-C0789",
  "totalActivities": 5,
  "summary": {
    "totalAreaRestored": 12.5,
    "totalCoralsPlanted": 7500,
    "averageSurvivalRate": 87.2,
    "averageGrowthRate": 2.3,
    "investmentTotal": 75000
  },
  "recentActivities": [
    {
      "activityId": "ACT-REST-2025-1234",
      "date": "2024-07-20",
      "type": "coral_gardening",
      "coralsPlanted": 1500,
      "currentSurvivalRate": 87.2
    }
  ],
  "successMetrics": {
    "healthScoreChange": "+15.3",
    "coverageIncrease": "+8.2%",
    "speciesAdded": 3,
    "projectedRecovery": "8-12 years"
  }
}
```

### 5.3 Calculate Restoration Success Index

**Endpoint:** `GET /reefs/{reefId}/restoration/success`

**Response (200 OK):**
```json
{
  "reefId": "REEF-INDO-C0789",
  "successIndex": 87.5,
  "metrics": {
    "survivalRate": 87.2,
    "growthRate": 2.3,
    "coverageIncrease": 8.2,
    "speciesDiversityIncrease": 12,
    "monthsTracked": 18
  },
  "comparison": {
    "regionalAverage": 72.5,
    "globalAverage": 68.3,
    "percentile": 92
  },
  "factors": {
    "waterQuality": "excellent",
    "temperature": "stable",
    "humanImpact": "minimal",
    "funding": "adequate"
  }
}
```

---

## 6. Biodiversity Endpoints

### 6.1 Submit Biodiversity Assessment

**Endpoint:** `POST /reefs/biodiversity`

**Request:**
```json
{
  "reefId": "REEF-GBR-A0123",
  "timestamp": "2025-12-25T10:00:00Z",
  "survey": {
    "method": "transect",
    "duration": 120,
    "areaSurveyed": 100
  },
  "species": {
    "coralSpecies": [
      {"name": "Acropora millepora", "count": 245},
      {"name": "Pocillopora damicornis", "count": 189}
    ],
    "fishSpecies": [
      {"name": "Chaetodon lunulatus", "count": 42},
      {"name": "Centropyge bicolor", "count": 28}
    ],
    "invertebrateSpecies": [
      {"name": "Diadema setosum", "count": 67}
    ]
  }
}
```

**Response (201 Created):**
```json
{
  "status": "success",
  "assessmentId": "BIO-2025-12-25-001",
  "indices": {
    "shannon": 3.42,
    "simpson": 0.94,
    "evenness": 0.87,
    "richness": 42
  },
  "status": "excellent",
  "trend": "increasing"
}
```

### 6.2 Get Biodiversity Metrics

**Endpoint:** `GET /reefs/{reefId}/biodiversity`

**Response (200 OK):**
```json
{
  "reefId": "REEF-GBR-A0123",
  "current": {
    "totalSpecies": 42,
    "coralSpecies": 18,
    "fishSpecies": 19,
    "invertebrateSpecies": 5,
    "shannonIndex": 3.42,
    "simpsonIndex": 0.94
  },
  "keyIndicators": [
    {
      "species": "Acropora cervicornis",
      "status": "present",
      "significance": "Critical habitat former"
    },
    {
      "species": "Scaridae (Parrotfish)",
      "status": "abundant",
      "significance": "Algae control"
    }
  ],
  "trend": {
    "direction": "increasing",
    "speciesChange": "+5 species (6 months)",
    "diversityChange": "+0.12 (Shannon)"
  }
}
```

---

## 7. Reporting Endpoints

### 7.1 Generate Reef Report

**Endpoint:** `POST /reefs/report`

**Request:**
```json
{
  "reefId": "REEF-GBR-A0123",
  "period": {
    "start": "2025-01-01",
    "end": "2025-12-25"
  },
  "sections": [
    "health_summary",
    "restoration_activities",
    "biodiversity",
    "bleaching_events",
    "recommendations"
  ],
  "format": "pdf"
}
```

**Response (202 Accepted):**
```json
{
  "status": "processing",
  "reportId": "RPT-2025-12-25-001",
  "estimatedCompletion": "2025-12-25T15:30:00Z",
  "statusUrl": "https://api.coral-reef.wiastandards.org/v1/reports/RPT-2025-12-25-001"
}
```

### 7.2 Download Report

**Endpoint:** `GET /reports/{reportId}`

**Response (200 OK):**
```json
{
  "reportId": "RPT-2025-12-25-001",
  "status": "completed",
  "generatedAt": "2025-12-25T15:28:00Z",
  "downloadUrl": "https://reports.coral-reef.wiastandards.org/RPT-2025-12-25-001.pdf",
  "expiresAt": "2025-12-26T15:28:00Z"
}
```

---

## 8. Error Handling

### 8.1 Error Response Format

```json
{
  "error": {
    "code": "INVALID_REEF_ID",
    "message": "Reef ID format is invalid",
    "details": "Expected format: REEF-[A-Z]{3}-[A-Z0-9]{5}",
    "timestamp": "2025-12-25T10:00:00Z",
    "requestId": "req-abc123"
  }
}
```

### 8.2 HTTP Status Codes

| Code | Meaning | Description |
|------|---------|-------------|
| 200 | OK | Request successful |
| 201 | Created | Resource created successfully |
| 202 | Accepted | Request accepted for processing |
| 400 | Bad Request | Invalid request format |
| 401 | Unauthorized | Missing or invalid authentication |
| 403 | Forbidden | Insufficient permissions |
| 404 | Not Found | Resource not found |
| 409 | Conflict | Resource conflict (duplicate) |
| 422 | Unprocessable Entity | Validation error |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Server Error | Server error |
| 503 | Service Unavailable | Temporary service disruption |

### 8.3 Common Error Codes

| Error Code | Description | Resolution |
|------------|-------------|------------|
| `INVALID_REEF_ID` | Reef ID format invalid | Check ID format |
| `REEF_NOT_FOUND` | Reef doesn't exist | Verify reef ID |
| `INVALID_COORDINATES` | GPS coordinates invalid | Check lat/long values |
| `METRIC_OUT_OF_RANGE` | Metric value outside valid range | Check data validation rules |
| `DUPLICATE_RECORD` | Record already exists | Check for duplicates |
| `INSUFFICIENT_PERMISSIONS` | Missing required scope | Request appropriate permissions |
| `RATE_LIMIT_EXCEEDED` | Too many requests | Wait and retry with backoff |

---

## 9. Rate Limiting

### 9.1 Rate Limits

| Tier | Requests/Minute | Requests/Day |
|------|-----------------|--------------|
| Free | 10 | 1,000 |
| Basic | 100 | 10,000 |
| Pro | 500 | 50,000 |
| Enterprise | Custom | Custom |

### 9.2 Rate Limit Headers

```http
X-RateLimit-Limit: 100
X-RateLimit-Remaining: 87
X-RateLimit-Reset: 1640434800
```

---

## 10. Webhooks

### 10.1 Register Webhook

**Endpoint:** `POST /webhooks`

**Request:**
```json
{
  "url": "https://your-server.com/webhook",
  "events": [
    "bleaching.alert.created",
    "reef.health.critical",
    "restoration.completed"
  ],
  "secret": "your-webhook-secret"
}
```

### 10.2 Webhook Events

| Event | Triggered When |
|-------|----------------|
| `bleaching.alert.created` | New bleaching alert issued |
| `bleaching.alert.updated` | Bleaching status changes |
| `reef.health.critical` | Health score drops below 50 |
| `reef.health.improved` | Health score increases >10 points |
| `restoration.started` | New restoration activity |
| `restoration.completed` | Restoration milestone reached |
| `biodiversity.assessed` | New biodiversity survey |

### 10.3 Webhook Payload

```json
{
  "event": "bleaching.alert.created",
  "timestamp": "2025-12-25T14:00:00Z",
  "data": {
    "alertId": "ALERT-2025-12-25-001",
    "reefId": "REEF-CAR-B0456",
    "severity": "MODERATE",
    "message": "Temperature anomaly detected"
  }
}
```

---

## 11. SDK Support

### 11.1 Official SDKs

- **JavaScript/TypeScript:** `npm install @wia/coral-reef-sdk`
- **Python:** `pip install wia-coral-reef`
- **Java:** Maven/Gradle available
- **Go:** `go get github.com/wia/coral-reef-go`

### 11.2 Example (JavaScript)

```javascript
import { CoralReefClient } from '@wia/coral-reef-sdk';

const client = new CoralReefClient({
  apiKey: 'your-api-key'
});

// Submit monitoring data
const result = await client.reefs.monitor({
  reefId: 'REEF-GBR-A0123',
  metrics: {
    coralCoverage: 65.5,
    waterTemperature: 28.5
  }
});

// Get health status
const health = await client.reefs.getHealth('REEF-GBR-A0123');
```

---

**弘益人間 (홍익인간) - Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 MIT License*
