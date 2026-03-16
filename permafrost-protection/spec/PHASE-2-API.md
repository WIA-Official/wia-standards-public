# WIA Permafrost Protection API Standard
## Phase 2 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #EF4444 (Red)

---

## Table of Contents

1. [Overview](#overview)
2. [Authentication](#authentication)
3. [REST API Endpoints](#rest-api-endpoints)
4. [WebSocket Real-time Monitoring](#websocket-real-time-monitoring)
5. [Error Handling](#error-handling)
6. [Rate Limiting](#rate-limiting)
7. [SDK Examples](#sdk-examples)
8. [Version History](#version-history)

---

## Overview

### 1.1 Purpose

The WIA Permafrost Protection API provides programmatic access to permafrost monitoring data, enabling researchers, climate scientists, and governments to access real-time measurements, historical trends, and risk assessments.

**Core Features**:
- Real-time permafrost temperature and emissions data
- Historical trend analysis and prediction
- Alert and threshold management
- Multi-station data aggregation
- Integration with climate models

### 1.2 Base URL

```
Production:  https://api.wia.live/permafrost/v1
Staging:     https://api-staging.wia.live/permafrost/v1
```

### 1.3 Content Types

```
Request:  application/json
Response: application/json
```

---

## Authentication

### 2.1 API Key Authentication

```http
Authorization: Bearer <API_KEY>
```

**Example Request:**

```bash
curl -X GET "https://api.wia.live/permafrost/v1/stations" \
  -H "Authorization: Bearer wia_live_1234567890abcdef"
```

### 2.2 OAuth 2.0

For third-party applications and research institutions:

```
Authorization URL: https://auth.wia.live/oauth/authorize
Token URL:         https://auth.wia.live/oauth/token
Scopes:
  - permafrost:read         : Read station data
  - permafrost:write        : Create/update station records
  - permafrost:admin        : Manage stations and alerts
  - permafrost:realtime     : Access WebSocket streams
```

---

## REST API Endpoints

### 3.1 Station Management

#### GET /stations

List all monitoring stations.

**Request:**

```http
GET /api/v1/permafrost/stations
Authorization: Bearer <API_KEY>
```

**Query Parameters:**

| Parameter | Type | Description | Example |
|-----------|------|-------------|---------|
| `zone` | string | Filter by permafrost zone | `continuous` |
| `status` | string | Filter by station status | `monitoring` |
| `region` | string | Geographic region | `Alaska` |
| `limit` | integer | Results per page (max 100) | `50` |
| `offset` | integer | Pagination offset | `0` |

**Response (200 OK):**

```json
{
  "status": "success",
  "data": {
    "stations": [
      {
        "stationId": "FROST-2025-000001",
        "location": {
          "region": "Alaska Interior",
          "gps": {
            "latitude": 64.8378,
            "longitude": -147.7164,
            "elevation": 215.5
          },
          "permafrostZone": "continuous"
        },
        "status": "monitoring",
        "lastUpdate": "2025-01-15T10:30:00Z",
        "alertLevel": "normal"
      }
    ],
    "total": 1,
    "limit": 50,
    "offset": 0
  }
}
```

#### GET /stations/:stationId

Get detailed data for a specific station.

**Request:**

```http
GET /api/v1/permafrost/stations/FROST-2025-000001
Authorization: Bearer <API_KEY>
```

**Response (200 OK):**

```json
{
  "status": "success",
  "data": {
    "stationId": "FROST-2025-000001",
    "status": "monitoring",
    "location": {
      "region": "Alaska Interior",
      "gps": {"latitude": 64.8378, "longitude": -147.7164, "elevation": 215.5},
      "permafrostZone": "continuous",
      "ecosystemType": "boreal_forest"
    },
    "measurements": {
      "timestamp": "2025-01-15T10:30:00Z",
      "groundTemperature": {
        "surface": {"value": -12.5, "unit": "celsius"},
        "at1m": {"value": -5.2, "unit": "celsius"}
      },
      "permafrostDepth": {"value": 15.5, "unit": "m"},
      "thawRate": {"value": 2.5, "unit": "cm_per_year"}
    },
    "emissions": {
      "methane": {
        "flux": {"value": 15.2, "unit": "mg_ch4_m2_day"}
      }
    },
    "risk": {
      "stabilityScore": 65,
      "alertLevel": "moderate"
    }
  }
}
```

#### POST /stations/:stationId/measurements

Submit new measurement data for a station.

**Request:**

```http
POST /api/v1/permafrost/stations/FROST-2025-000001/measurements
Authorization: Bearer <API_KEY>
Content-Type: application/json
```

**Request Body:**

```json
{
  "timestamp": "2025-01-15T10:30:00Z",
  "groundTemperature": {
    "surface": {"value": -12.5, "unit": "celsius"},
    "at1m": {"value": -5.2, "unit": "celsius"}
  },
  "permafrostDepth": {"value": 15.5, "unit": "m"},
  "thawRate": {"value": 2.5, "unit": "cm_per_year"},
  "emissions": {
    "methane": {
      "flux": {"value": 15.2, "unit": "mg_ch4_m2_day"}
    }
  }
}
```

**Response (201 Created):**

```json
{
  "status": "success",
  "message": "Measurement recorded successfully",
  "data": {
    "measurementId": "MEAS-2025-ABC123",
    "stationId": "FROST-2025-000001",
    "timestamp": "2025-01-15T10:30:00Z",
    "alertsTriggered": []
  }
}
```

### 3.2 Historical Data & Trends

#### GET /stations/:stationId/history

Get historical measurement data.

**Request:**

```http
GET /api/v1/permafrost/stations/FROST-2025-000001/history?start=2024-01-01&end=2025-01-15&interval=daily
Authorization: Bearer <API_KEY>
```

**Query Parameters:**

| Parameter | Type | Description | Example |
|-----------|------|-------------|---------|
| `start` | string | Start date (ISO 8601) | `2024-01-01` |
| `end` | string | End date (ISO 8601) | `2025-01-15` |
| `interval` | string | Data aggregation | `hourly`, `daily`, `weekly` |
| `metrics` | string | Comma-separated metrics | `temperature,thawRate,methane` |

**Response (200 OK):**

```json
{
  "status": "success",
  "data": {
    "stationId": "FROST-2025-000001",
    "period": {
      "start": "2024-01-01T00:00:00Z",
      "end": "2025-01-15T23:59:59Z"
    },
    "interval": "daily",
    "measurements": [
      {
        "date": "2024-01-01",
        "avgSurfaceTemp": {"value": -15.2, "unit": "celsius"},
        "avgThawRate": {"value": 2.1, "unit": "cm_per_year"},
        "avgMethaneFlux": {"value": 12.5, "unit": "mg_ch4_m2_day"}
      },
      {
        "date": "2024-01-02",
        "avgSurfaceTemp": {"value": -14.8, "unit": "celsius"},
        "avgThawRate": {"value": 2.3, "unit": "cm_per_year"},
        "avgMethaneFlux": {"value": 13.1, "unit": "mg_ch4_m2_day"}
      }
    ]
  }
}
```

#### GET /stations/:stationId/trends

Get trend analysis and predictions.

**Response (200 OK):**

```json
{
  "status": "success",
  "data": {
    "stationId": "FROST-2025-000001",
    "trends": {
      "temperature": {
        "trend": "warming",
        "rate": {"value": 0.5, "unit": "celsius_per_decade"},
        "confidence": 0.92
      },
      "thawRate": {
        "trend": "increasing",
        "rate": {"value": 0.3, "unit": "cm_per_year_per_decade"},
        "confidence": 0.88
      },
      "methaneEmissions": {
        "trend": "increasing",
        "rate": {"value": 1.2, "unit": "percent_per_year"},
        "confidence": 0.85
      }
    },
    "predictions": {
      "2030": {
        "avgSurfaceTemp": {"value": -11.5, "unit": "celsius"},
        "thawRate": {"value": 3.8, "unit": "cm_per_year"}
      },
      "2050": {
        "avgSurfaceTemp": {"value": -9.0, "unit": "celsius"},
        "thawRate": {"value": 5.5, "unit": "cm_per_year"}
      }
    }
  }
}
```

### 3.3 Alerts & Thresholds

#### GET /alerts

Get active alerts.

**Request:**

```http
GET /api/v1/permafrost/alerts?severity=high&status=active
Authorization: Bearer <API_KEY>
```

**Response (200 OK):**

```json
{
  "status": "success",
  "data": {
    "alerts": [
      {
        "alertId": "ALERT-2025-001",
        "stationId": "FROST-2025-000042",
        "type": "rapid_thaw",
        "severity": "high",
        "message": "Thaw rate exceeds 4 cm/year - infrastructure at risk",
        "triggeredAt": "2025-01-15T14:30:00Z",
        "status": "active",
        "measurements": {
          "thawRate": {"value": 4.2, "unit": "cm_per_year"},
          "threshold": {"value": 3.0, "unit": "cm_per_year"}
        }
      }
    ],
    "total": 1
  }
}
```

#### POST /stations/:stationId/thresholds

Set alert thresholds for a station.

**Request Body:**

```json
{
  "thresholds": [
    {
      "metric": "thawRate",
      "condition": "greater_than",
      "value": {"value": 3.0, "unit": "cm_per_year"},
      "severity": "high",
      "notifyEmails": ["researcher@example.com"]
    },
    {
      "metric": "methaneFlux",
      "condition": "greater_than",
      "value": {"value": 20.0, "unit": "mg_ch4_m2_day"},
      "severity": "moderate"
    }
  ]
}
```

**Response (200 OK):**

```json
{
  "status": "success",
  "message": "Thresholds configured successfully",
  "data": {
    "stationId": "FROST-2025-000001",
    "thresholds": 2
  }
}
```

### 3.4 Aggregated Data

#### GET /aggregate/regional

Get aggregated data by region.

**Request:**

```http
GET /api/v1/permafrost/aggregate/regional?zone=continuous&metric=methane
Authorization: Bearer <API_KEY>
```

**Response (200 OK):**

```json
{
  "status": "success",
  "data": {
    "zone": "continuous",
    "metric": "methane",
    "aggregation": {
      "totalStations": 45,
      "avgFlux": {"value": 14.8, "unit": "mg_ch4_m2_day"},
      "totalEmissions": {"value": 2450.5, "unit": "kg_ch4_year"},
      "co2Equivalent": {"value": 68614, "unit": "kg_co2eq_year"}
    },
    "topEmitters": [
      {
        "stationId": "FROST-2025-000012",
        "region": "Siberia",
        "flux": {"value": 45.2, "unit": "mg_ch4_m2_day"}
      }
    ]
  }
}
```

---

## WebSocket Real-time Monitoring

### 4.1 Connection

```javascript
const ws = new WebSocket('wss://api.wia.live/permafrost/v1/stream');

ws.onopen = () => {
  // Authenticate
  ws.send(JSON.stringify({
    type: 'auth',
    token: 'wia_live_1234567890abcdef'
  }));

  // Subscribe to station
  ws.send(JSON.stringify({
    type: 'subscribe',
    stationId: 'FROST-2025-000001'
  }));
};

ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  console.log('Real-time measurement:', data);
};
```

### 4.2 Message Types

**Measurement Update:**

```json
{
  "type": "measurement",
  "stationId": "FROST-2025-000001",
  "timestamp": "2025-01-15T10:30:00Z",
  "data": {
    "groundTemperature": {
      "surface": {"value": -12.5, "unit": "celsius"}
    },
    "methaneFlux": {"value": 15.2, "unit": "mg_ch4_m2_day"}
  }
}
```

**Alert Notification:**

```json
{
  "type": "alert",
  "alertId": "ALERT-2025-001",
  "stationId": "FROST-2025-000001",
  "severity": "high",
  "message": "Rapid thaw detected",
  "timestamp": "2025-01-15T10:30:00Z"
}
```

---

## Error Handling

### 5.1 Error Response Format

```json
{
  "status": "error",
  "error": {
    "code": "ERR_STATION_NOT_FOUND",
    "message": "Station FROST-2025-999999 not found",
    "details": {
      "stationId": "FROST-2025-999999"
    }
  }
}
```

### 5.2 HTTP Status Codes

| Code | Description | Usage |
|------|-------------|-------|
| 200 | OK | Successful request |
| 201 | Created | Resource created |
| 400 | Bad Request | Invalid parameters |
| 401 | Unauthorized | Missing/invalid authentication |
| 403 | Forbidden | Insufficient permissions |
| 404 | Not Found | Resource not found |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Server Error | Server error |

### 5.3 Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| `ERR_STATION_NOT_FOUND` | 404 | Station ID does not exist |
| `ERR_INVALID_MEASUREMENT` | 400 | Measurement data validation failed |
| `ERR_INVALID_TIMERANGE` | 400 | Invalid date range |
| `ERR_UNAUTHORIZED` | 401 | Authentication required |
| `ERR_RATE_LIMIT` | 429 | Too many requests |

---

## Rate Limiting

### 6.1 Limits

| Tier | Requests/Minute | WebSocket Connections |
|------|----------------|----------------------|
| Free | 60 | 1 |
| Research | 600 | 5 |
| Enterprise | 6000 | 50 |

### 6.2 Headers

```http
X-RateLimit-Limit: 60
X-RateLimit-Remaining: 45
X-RateLimit-Reset: 1642262400
```

---

## SDK Examples

### 7.1 JavaScript/TypeScript

```typescript
import { PermafrostAPI } from '@wia/permafrost-sdk';

const client = new PermafrostAPI({
  apiKey: 'wia_live_1234567890abcdef'
});

// Get station data
const station = await client.stations.get('FROST-2025-000001');
console.log(station.measurements.groundTemperature);

// Submit measurement
await client.stations.submitMeasurement('FROST-2025-000001', {
  timestamp: new Date().toISOString(),
  groundTemperature: {
    surface: { value: -12.5, unit: 'celsius' }
  }
});

// Real-time monitoring
client.stream.subscribe('FROST-2025-000001', (data) => {
  console.log('Real-time update:', data);
});
```

### 7.2 Python

```python
from wia_permafrost import PermafrostClient

client = PermafrostClient(api_key='wia_live_1234567890abcdef')

# Get station
station = client.stations.get('FROST-2025-000001')
print(station.measurements.ground_temperature)

# Get historical data
history = client.stations.history(
    'FROST-2025-000001',
    start='2024-01-01',
    end='2025-01-15',
    interval='daily'
)

# Set threshold
client.stations.set_threshold(
    'FROST-2025-000001',
    metric='thawRate',
    condition='greater_than',
    value={'value': 3.0, 'unit': 'cm_per_year'},
    severity='high'
)
```

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01 | Initial API release |

---

<div align="center">

**WIA Permafrost Protection API v1.0.0**

**弘益人間 (홍익인간)** - Benefit All Humanity

---

**© 2025 WIA**

**MIT License**

</div>
