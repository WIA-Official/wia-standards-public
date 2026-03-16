# WIA Water Scarcity Response API Standard
## Phase 2 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #EF4444 (Red - ENE Category)

---

## Table of Contents

1. [API Overview](#api-overview)
2. [Authentication](#authentication)
3. [Core Endpoints](#core-endpoints)
4. [Water Monitoring API](#water-monitoring-api)
5. [Drought Prediction API](#drought-prediction-api)
6. [Consumption Management API](#consumption-management-api)
7. [Alert System API](#alert-system-api)
8. [Error Handling](#error-handling)
9. [Rate Limiting](#rate-limiting)
10. [Code Examples](#code-examples)

---

## API Overview

### 1.1 Base URL

```
Production: https://api.wia-water.org/v1
Staging: https://staging-api.wia-water.org/v1
Development: https://dev-api.wia-water.org/v1
```

### 1.2 API Principles

- **RESTful Design**: Standard HTTP methods (GET, POST, PUT, DELETE)
- **JSON Format**: All requests and responses use JSON
- **Versioning**: API version in URL path (/v1/)
- **Rate Limiting**: 1000 requests/hour per API key
- **Authentication**: OAuth 2.0 and API keys
- **Real-time**: WebSocket support for live monitoring

### 1.3 Request Headers

```http
Content-Type: application/json
Authorization: Bearer {access_token}
X-API-Key: {api_key}
X-WIA-Standard: WIA-ENE-053
```

---

## Authentication

### 2.1 API Key Authentication

```bash
curl -H "X-API-Key: your-api-key" \
  https://api.wia-water.org/v1/water-levels
```

### 2.2 OAuth 2.0 Flow

```bash
# Step 1: Get access token
POST /oauth/token
{
  "grant_type": "client_credentials",
  "client_id": "your-client-id",
  "client_secret": "your-client-secret"
}

# Response
{
  "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600
}

# Step 2: Use token
curl -H "Authorization: Bearer {access_token}" \
  https://api.wia-water.org/v1/water-levels
```

---

## Core Endpoints

### 3.1 Endpoint Summary

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/water-levels` | GET | Get current water levels |
| `/water-levels/{id}` | GET | Get specific water source |
| `/consumption` | GET | Get consumption data |
| `/consumption/optimize` | POST | Optimize water allocation |
| `/aquifers` | GET | Get aquifer data |
| `/aquifers/{id}/health` | GET | Get aquifer health metrics |
| `/drought-forecast` | POST | Predict drought risk |
| `/drought-alerts` | GET | Get active drought alerts |
| `/alerts` | GET | Get all alerts |
| `/alerts/subscribe` | POST | Subscribe to alerts |
| `/desalination` | GET | Get desalination plant data |
| `/conservation` | GET | Get conservation metrics |

---

## Water Monitoring API

### 4.1 Get Water Levels

```http
GET /api/v1/water-levels
```

**Query Parameters**:
- `region` (string): Filter by region
- `type` (string): Filter by source type (reservoir, aquifer, etc.)
- `status` (string): Filter by status (critical, warning, normal)
- `limit` (number): Max results (default: 50)
- `offset` (number): Pagination offset

**Response**:

```json
{
  "status": "success",
  "data": {
    "count": 2,
    "results": [
      {
        "id": "source-lakemead",
        "name": "Lake Mead",
        "type": "reservoir",
        "location": {
          "latitude": 36.0155,
          "longitude": -114.7443,
          "region": "Nevada/Arizona"
        },
        "currentLevel": {
          "value": 327.5,
          "unit": "meters",
          "timestamp": "2025-01-15T10:00:00Z"
        },
        "capacity": {
          "percentFull": 34,
          "status": "critical"
        },
        "trend": "declining"
      },
      {
        "id": "source-lakepowell",
        "name": "Lake Powell",
        "type": "reservoir",
        "currentLevel": {
          "value": 1075.8,
          "unit": "meters"
        },
        "capacity": {
          "percentFull": 28,
          "status": "critical"
        },
        "trend": "stable"
      }
    ]
  },
  "meta": {
    "timestamp": "2025-01-15T10:00:00Z",
    "requestId": "req-12345"
  }
}
```

### 4.2 Get Specific Water Source

```http
GET /api/v1/water-levels/{id}
```

**Response**:

```json
{
  "status": "success",
  "data": {
    "id": "source-lakemead",
    "name": "Lake Mead",
    "type": "reservoir",
    "location": {
      "latitude": 36.0155,
      "longitude": -114.7443,
      "region": "Nevada/Arizona",
      "country": "USA"
    },
    "currentLevel": {
      "value": 327.5,
      "unit": "meters",
      "timestamp": "2025-01-15T10:00:00Z"
    },
    "capacity": {
      "current": 34,
      "maximum": 100,
      "total": 32236000000,
      "unit": "cubic_meters",
      "percentFull": 34,
      "status": "critical"
    },
    "historicalData": {
      "7days": { "average": 327.8, "min": 327.2, "max": 328.5 },
      "30days": { "average": 329.5, "trend": "declining" }
    },
    "alerts": [
      {
        "level": "critical",
        "message": "Water level below critical threshold",
        "timestamp": "2025-01-14T08:00:00Z"
      }
    ]
  }
}
```

### 4.3 Get Consumption Data

```http
GET /api/v1/consumption
```

**Query Parameters**:
- `region` (string)
- `sector` (string): residential, agricultural, industrial
- `timeframe` (string): 1d, 7d, 30d, 1y

**Response**:

```json
{
  "status": "success",
  "data": {
    "region": "southwest",
    "timeframe": "30d",
    "consumption": {
      "daily": 2500000,
      "weekly": 17500000,
      "monthly": 75000000,
      "unit": "cubic_meters"
    },
    "perCapita": 250,
    "sectors": {
      "residential": {
        "percentage": 45,
        "volume": 1125000,
        "trend": "-2.1%"
      },
      "agricultural": {
        "percentage": 35,
        "volume": 875000,
        "trend": "-1.5%"
      },
      "industrial": {
        "percentage": 20,
        "volume": 500000,
        "trend": "+0.5%"
      }
    },
    "comparison": {
      "previousPeriod": "+3.2%",
      "yearOverYear": "-5.8%"
    }
  }
}
```

---

## Drought Prediction API

### 5.1 Forecast Drought Risk

```http
POST /api/v1/drought-forecast
```

**Request Body**:

```json
{
  "region": "southwest",
  "climate": {
    "rainfall": 450,
    "temperature": 28,
    "evaporation": 6.5,
    "humidity": 35
  },
  "waterSources": ["source-lakemead", "source-lakepowell"],
  "timeframe": "30d"
}
```

**Response**:

```json
{
  "status": "success",
  "data": {
    "forecastId": "forecast-2025-01-15",
    "region": "southwest",
    "timeframe": "30d",
    "droughtRisk": {
      "score": 78,
      "level": "high",
      "severity": "severe",
      "confidence": 92
    },
    "forecast": {
      "30days": {
        "probability": 85,
        "expectedRainfall": 15,
        "waterDeficit": 450000
      },
      "60days": {
        "probability": 80,
        "expectedRainfall": 30
      },
      "90days": {
        "probability": 73,
        "expectedRainfall": 60
      }
    },
    "recommendations": [
      "Implement emergency conservation measures",
      "Activate water restriction policies",
      "Increase desalination production",
      "Alert agricultural sector for irrigation planning"
    ],
    "modelInfo": {
      "algorithm": "WIA-Drought-Predictor-v1.0",
      "lastTrained": "2025-01-01",
      "accuracy": 94.5
    }
  }
}
```

### 5.2 Get Drought Alerts

```http
GET /api/v1/drought-alerts
```

**Response**:

```json
{
  "status": "success",
  "data": {
    "activeAlerts": [
      {
        "id": "alert-drought-001",
        "level": "critical",
        "region": "southwest",
        "message": "Severe drought conditions detected",
        "severity": "severe",
        "issuedAt": "2025-01-14T08:00:00Z",
        "expiresAt": "2025-01-21T08:00:00Z",
        "affectedSources": ["source-lakemead", "aquifer-sw-001"],
        "actions": [
          "Mandatory 25% water reduction",
          "Outdoor watering prohibited",
          "Industrial use restricted"
        ]
      }
    ],
    "count": 1
  }
}
```

---

## Consumption Management API

### 6.1 Optimize Water Allocation

```http
POST /api/v1/consumption/optimize
```

**Request Body**:

```json
{
  "region": "southwest",
  "population": 2000000,
  "currentUsage": {
    "perCapita": 250,
    "sectors": {
      "residential": 45,
      "agricultural": 35,
      "industrial": 20
    }
  },
  "constraints": {
    "targetReduction": 15,
    "minimumResidential": 150,
    "criticalIndustries": ["healthcare", "food-processing"]
  },
  "objectives": ["minimize-waste", "ensure-equity", "protect-critical"]
}
```

**Response**:

```json
{
  "status": "success",
  "data": {
    "optimizationId": "opt-2025-01-15",
    "targetUsage": {
      "perCapita": 212.5,
      "reduction": 15
    },
    "allocation": {
      "residential": {
        "percentage": 48,
        "volume": 1020000,
        "perCapita": 204
      },
      "agricultural": {
        "percentage": 32,
        "volume": 680000,
        "reduction": 22.3
      },
      "industrial": {
        "percentage": 20,
        "volume": 425000,
        "reduction": 15
      }
    },
    "impact": {
      "dailySavings": 375000,
      "annualSavings": 136875000,
      "equivalentOlympicPools": 54.75,
      "costSavings": "$4,106,250"
    },
    "implementation": {
      "phase1": "Immediate: Public awareness campaign",
      "phase2": "Week 1: Implement tiered pricing",
      "phase3": "Week 2: Smart meter installation",
      "phase4": "Week 3: Enforce restrictions"
    }
  }
}
```

---

## Alert System API

### 7.1 Get All Alerts

```http
GET /api/v1/alerts
```

**Query Parameters**:
- `level` (string): info, warning, critical
- `active` (boolean): true, false
- `region` (string)

**Response**:

```json
{
  "status": "success",
  "data": {
    "alerts": [
      {
        "id": "alert-001",
        "type": "water-level",
        "level": "critical",
        "message": "Lake Mead at historic low",
        "severity": "high",
        "region": "Nevada/Arizona",
        "timestamp": "2025-01-15T08:00:00Z",
        "active": true
      },
      {
        "id": "alert-002",
        "type": "conservation",
        "level": "info",
        "message": "Conservation target 60% achieved",
        "severity": "low",
        "timestamp": "2025-01-15T07:30:00Z",
        "active": true
      }
    ],
    "count": 2
  }
}
```

### 7.2 Subscribe to Alerts

```http
POST /api/v1/alerts/subscribe
```

**Request Body**:

```json
{
  "channels": ["email", "sms", "webhook"],
  "email": "alerts@wateragency.gov",
  "phone": "+1-555-0100",
  "webhookUrl": "https://myapp.com/webhooks/water-alerts",
  "filters": {
    "regions": ["southwest", "california"],
    "levels": ["warning", "critical"],
    "types": ["drought", "water-level", "quality"]
  }
}
```

**Response**:

```json
{
  "status": "success",
  "data": {
    "subscriptionId": "sub-12345",
    "status": "active",
    "channels": ["email", "sms", "webhook"],
    "createdAt": "2025-01-15T10:00:00Z"
  }
}
```

### 7.3 WebSocket Real-Time Alerts

```javascript
const ws = new WebSocket('wss://api.wia-water.org/v1/alerts/stream');

ws.onopen = () => {
  ws.send(JSON.stringify({
    type: 'subscribe',
    filters: {
      regions: ['southwest'],
      levels: ['critical', 'warning']
    }
  }));
};

ws.onmessage = (event) => {
  const alert = JSON.parse(event.data);
  console.log('New alert:', alert);
};
```

---

## Error Handling

### 8.1 Error Response Format

```json
{
  "status": "error",
  "error": {
    "code": "RESOURCE_NOT_FOUND",
    "message": "Water source not found",
    "details": "No water source with id 'source-invalid'",
    "timestamp": "2025-01-15T10:00:00Z",
    "requestId": "req-12345"
  }
}
```

### 8.2 Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| `INVALID_REQUEST` | 400 | Malformed request |
| `UNAUTHORIZED` | 401 | Invalid credentials |
| `FORBIDDEN` | 403 | Insufficient permissions |
| `RESOURCE_NOT_FOUND` | 404 | Resource doesn't exist |
| `RATE_LIMIT_EXCEEDED` | 429 | Too many requests |
| `INTERNAL_ERROR` | 500 | Server error |
| `SERVICE_UNAVAILABLE` | 503 | Service temporarily down |

---

## Rate Limiting

### 9.1 Rate Limit Headers

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 995
X-RateLimit-Reset: 1642251600
```

### 9.2 Rate Limit Tiers

| Tier | Requests/Hour | WebSocket Connections |
|------|---------------|----------------------|
| Free | 100 | 1 |
| Standard | 1,000 | 5 |
| Professional | 10,000 | 25 |
| Enterprise | Unlimited | Unlimited |

---

## Code Examples

### 10.1 Python SDK

```python
from wia_water import WaterClient

client = WaterClient(api_key='your-api-key')

# Get water levels
levels = client.water_levels.list(region='southwest')
print(f"Found {len(levels)} water sources")

# Predict drought
forecast = client.drought.forecast(
    region='southwest',
    climate={'rainfall': 450, 'temperature': 28},
    timeframe='30d'
)
print(f"Drought risk: {forecast.risk_score}")

# Optimize consumption
optimization = client.consumption.optimize(
    region='southwest',
    population=2000000,
    target_reduction=15
)
print(f"Daily savings: {optimization.daily_savings} m³")
```

### 10.2 JavaScript SDK

```javascript
import { WaterClient } from '@wia/water-sdk';

const client = new WaterClient({ apiKey: 'your-api-key' });

// Get water levels
const levels = await client.waterLevels.list({ region: 'southwest' });
console.log(`Found ${levels.count} water sources`);

// Subscribe to real-time alerts
client.alerts.subscribe({
  regions: ['southwest'],
  levels: ['critical', 'warning'],
  onAlert: (alert) => {
    console.log('New alert:', alert);
  }
});
```

---

**Document Control**
© 2025 WIA Standards Committee
License: MIT
Contact: api@wia.org
弘益人間 · Benefit All Humanity
