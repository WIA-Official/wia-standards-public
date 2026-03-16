# WIA Urban Heat Island Response API Standard
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

1. [Overview](#overview)
2. [API Design Principles](#api-design-principles)
3. [Authentication](#authentication)
4. [Core Endpoints](#core-endpoints)
5. [Data Models](#data-models)
6. [Error Handling](#error-handling)
7. [Rate Limiting](#rate-limiting)
8. [Examples](#examples)

---

## Overview

### 1.1 Purpose

The WIA Urban Heat Island Response API provides standardized endpoints for monitoring, analyzing, and mitigating urban heat islands through RESTful interfaces, real-time data streaming, and integration capabilities.

### 1.2 Base URL

```
Production:  https://api.wia.live/heat/v1
Sandbox:     https://sandbox-api.wia.live/heat/v1
```

### 1.3 Supported Formats

- **Request**: JSON, XML (optional)
- **Response**: JSON (default), XML (via Accept header)
- **Streaming**: Server-Sent Events (SSE), WebSocket

---

## API Design Principles

1. **RESTful**: Resource-oriented URLs
2. **Versioned**: URL-based versioning (/v1)
3. **Stateless**: No server-side session state
4. **Cacheable**: Appropriate cache headers
5. **Paginated**: Large result sets paginated
6. **HATEOAS**: Hypermedia links for navigation

---

## Authentication

### 3.1 API Key Authentication

```http
GET /api/v1/heat/sensors HTTP/1.1
Host: api.wia.live
Authorization: Bearer {API_KEY}
```

### 3.2 OAuth 2.0

```http
POST /oauth/token HTTP/1.1
Host: api.wia.live
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials
&client_id={CLIENT_ID}
&client_secret={CLIENT_SECRET}
```

---

## Core Endpoints

### 4.1 Heat Monitoring

#### GET /heat/areas

List all monitored heat island areas.

**Request:**
```http
GET /api/v1/heat/areas?city=Seoul&limit=10&offset=0 HTTP/1.1
Authorization: Bearer {API_KEY}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "areas": [
      {
        "areaId": "AREA-SEOUL-GANGNAM",
        "name": "Gangnam District",
        "classification": "urban_high_density",
        "currentIntensity": {
          "value": 7.0,
          "unit": "celsius",
          "severity": "critical"
        },
        "lastUpdated": "2025-07-15T14:00:00Z",
        "_links": {
          "self": "/api/v1/heat/areas/AREA-SEOUL-GANGNAM",
          "sensors": "/api/v1/heat/areas/AREA-SEOUL-GANGNAM/sensors",
          "heatmap": "/api/v1/heat/areas/AREA-SEOUL-GANGNAM/heatmap"
        }
      }
    ],
    "pagination": {
      "total": 45,
      "limit": 10,
      "offset": 0,
      "hasMore": true
    }
  }
}
```

#### GET /heat/areas/{areaId}

Get detailed information for a specific area.

**Parameters:**
- `areaId` (path): Area identifier

**Response:**
```json
{
  "status": "success",
  "data": {
    "areaId": "AREA-SEOUL-GANGNAM",
    "name": "Gangnam District",
    "location": {
      "center": {
        "latitude": 37.5172,
        "longitude": 127.0473
      },
      "coverage": {
        "value": 39.5,
        "unit": "km2"
      }
    },
    "currentMeasurements": {
      "timestamp": "2025-07-15T14:00:00Z",
      "airTemperature": {
        "urban": {"value": 35.5, "unit": "celsius"},
        "rural": {"value": 28.5, "unit": "celsius"}
      },
      "heatIslandIntensity": {
        "value": 7.0,
        "unit": "celsius",
        "severity": "critical"
      }
    },
    "sensors": {
      "count": 25,
      "active": 23,
      "inactive": 2
    },
    "mitigation": {
      "strategiesCount": 5,
      "totalCoolingEffect": {
        "value": 2.5,
        "unit": "celsius"
      }
    }
  }
}
```

#### GET /heat/areas/{areaId}/heatmap

Get thermal heatmap data for visualization.

**Parameters:**
- `resolution` (query): Grid resolution in meters (default: 100)
- `timestamp` (query): Specific timestamp or 'latest'

**Response:**
```json
{
  "status": "success",
  "data": {
    "areaId": "AREA-SEOUL-GANGNAM",
    "timestamp": "2025-07-15T14:00:00Z",
    "resolution": 100,
    "bounds": {
      "north": 37.5300,
      "south": 37.5000,
      "east": 127.0600,
      "west": 127.0300
    },
    "grid": [
      {
        "lat": 37.5000,
        "lon": 127.0300,
        "temperature": 38.5,
        "material": "asphalt"
      },
      {
        "lat": 37.5000,
        "lon": 127.0400,
        "temperature": 35.2,
        "material": "concrete"
      }
    ]
  }
}
```

### 4.2 Sensor Management

#### GET /heat/sensors

List all sensors.

**Query Parameters:**
- `areaId` (optional): Filter by area
- `status` (optional): active, inactive, calibration_needed
- `type` (optional): temperature, humidity, infrared

**Response:**
```json
{
  "status": "success",
  "data": {
    "sensors": [
      {
        "sensorId": "SENSOR-SEOUL-001",
        "type": "temperature",
        "location": {
          "latitude": 37.5172,
          "longitude": 127.0473,
          "installationType": "rooftop"
        },
        "status": "active",
        "lastMeasurement": {
          "airTemperature": {"value": 35.5, "unit": "celsius"},
          "timestamp": "2025-07-15T14:00:00Z"
        },
        "dataQuality": "high"
      }
    ]
  }
}
```

#### POST /heat/sensors

Register a new sensor.

**Request:**
```json
{
  "sensorId": "SENSOR-SEOUL-NEW",
  "type": "temperature",
  "location": {
    "latitude": 37.5172,
    "longitude": 127.0473,
    "altitude": 50.0,
    "installationType": "rooftop"
  },
  "calibrationDate": "2025-01-15T00:00:00Z"
}
```

**Response:**
```json
{
  "status": "success",
  "message": "Sensor registered successfully",
  "data": {
    "sensorId": "SENSOR-SEOUL-NEW",
    "registrationDate": "2025-01-15T10:30:00Z",
    "apiKey": "sk_live_abc123..."
  }
}
```

#### POST /heat/sensors/{sensorId}/data

Submit sensor measurement data.

**Request:**
```json
{
  "timestamp": "2025-07-15T14:00:00Z",
  "measurements": {
    "airTemperature": {"value": 35.5, "unit": "celsius"},
    "surfaceTemperature": {"value": 45.2, "unit": "celsius"},
    "humidity": {"value": 60, "unit": "percent"},
    "windSpeed": {"value": 2.5, "unit": "m/s"}
  }
}
```

**Response:**
```json
{
  "status": "success",
  "message": "Data recorded successfully",
  "data": {
    "recordId": "REC-2025-000001",
    "timestamp": "2025-07-15T14:00:00Z",
    "validated": true
  }
}
```

### 4.3 Mitigation Strategies

#### GET /heat/mitigation/strategies

List available mitigation strategies.

**Response:**
```json
{
  "status": "success",
  "data": {
    "strategies": [
      {
        "strategyType": "green_roof",
        "description": "Vegetated roof system",
        "effectiveness": {
          "temperatureReduction": {"min": 2, "max": 5, "unit": "celsius"},
          "energySavings": {"min": 10, "max": 20, "unit": "percent"}
        },
        "cost": "medium-high",
        "lifespan": {"value": 20, "unit": "years"}
      }
    ]
  }
}
```

#### POST /heat/mitigation/recommendations

Get mitigation recommendations for an area.

**Request:**
```json
{
  "areaId": "AREA-SEOUL-GANGNAM",
  "budget": {"value": 1000000, "unit": "USD"},
  "priorities": ["temperature_reduction", "energy_savings"],
  "constraints": {
    "implementation_time": "fast"
  }
}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "recommendations": [
      {
        "priority": 1,
        "strategyType": "cool_pavement",
        "targetLocations": [
          {
            "latitude": 37.5150,
            "longitude": 127.0450,
            "currentTemp": 58.3,
            "coverage": {"value": 5000, "unit": "m2"}
          }
        ],
        "estimatedImpact": {
          "temperatureReduction": {"value": 1.8, "unit": "celsius"},
          "cost": {"value": 500000, "unit": "USD"},
          "roi": {"value": 3.5, "unit": "years"}
        }
      }
    ]
  }
}
```

### 4.4 Alerts and Notifications

#### POST /heat/alerts

Create a heat alert.

**Request:**
```json
{
  "areaId": "AREA-SEOUL-GANGNAM",
  "severity": "critical",
  "threshold": {
    "heatIslandIntensity": {"value": 6.0, "unit": "celsius"}
  },
  "notificationChannels": ["email", "sms", "webhook"],
  "recipients": ["admin@city.gov"]
}
```

#### GET /heat/alerts

Get active alerts.

**Response:**
```json
{
  "status": "success",
  "data": {
    "alerts": [
      {
        "alertId": "ALERT-2025-001",
        "areaId": "AREA-SEOUL-GANGNAM",
        "severity": "critical",
        "message": "Heat island intensity exceeds 6°C threshold",
        "triggeredAt": "2025-07-15T14:00:00Z",
        "status": "active"
      }
    ]
  }
}
```

---

## Data Models

### 5.1 Temperature Object

```typescript
interface Temperature {
  value: number;
  unit: 'celsius' | 'fahrenheit';
}
```

### 5.2 Location Object

```typescript
interface Location {
  latitude: number;
  longitude: number;
  altitude?: number;
}
```

### 5.3 Pagination Object

```typescript
interface Pagination {
  total: number;
  limit: number;
  offset: number;
  hasMore: boolean;
}
```

---

## Error Handling

### 6.1 Error Response Format

```json
{
  "status": "error",
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "Invalid GPS coordinates",
    "details": {
      "field": "latitude",
      "reason": "Value must be between -90 and 90"
    },
    "timestamp": "2025-07-15T14:00:00Z",
    "requestId": "req_abc123"
  }
}
```

### 6.2 HTTP Status Codes

| Code | Meaning | Description |
|------|---------|-------------|
| 200 | OK | Request successful |
| 201 | Created | Resource created |
| 400 | Bad Request | Invalid request format |
| 401 | Unauthorized | Invalid API key |
| 403 | Forbidden | Insufficient permissions |
| 404 | Not Found | Resource not found |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Server Error | Server error |

---

## Rate Limiting

### 7.1 Rate Limit Headers

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 999
X-RateLimit-Reset: 1642262400
```

### 7.2 Rate Limits

| Tier | Requests/Hour | Burst |
|------|---------------|-------|
| Free | 100 | 10/min |
| Basic | 1,000 | 50/min |
| Pro | 10,000 | 200/min |
| Enterprise | Unlimited | Custom |

---

## Examples

### 8.1 Complete Workflow

```bash
# 1. Get areas
curl -X GET "https://api.wia.live/heat/v1/heat/areas?city=Seoul" \
  -H "Authorization: Bearer {API_KEY}"

# 2. Get area details
curl -X GET "https://api.wia.live/heat/v1/heat/areas/AREA-SEOUL-GANGNAM" \
  -H "Authorization: Bearer {API_KEY}"

# 3. Get heatmap
curl -X GET "https://api.wia.live/heat/v1/heat/areas/AREA-SEOUL-GANGNAM/heatmap?resolution=100" \
  -H "Authorization: Bearer {API_KEY}"

# 4. Submit sensor data
curl -X POST "https://api.wia.live/heat/v1/heat/sensors/SENSOR-001/data" \
  -H "Authorization: Bearer {API_KEY}" \
  -H "Content-Type: application/json" \
  -d '{
    "timestamp": "2025-07-15T14:00:00Z",
    "measurements": {
      "airTemperature": {"value": 35.5, "unit": "celsius"}
    }
  }'
```

---

<div align="center">

**WIA Urban Heat Island Response API v1.0.0**

**弘益人間 (홍익인간)** - Benefit All Humanity

---

**© 2025 WIA**

**MIT License**

</div>
