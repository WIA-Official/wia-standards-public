# WIA Polar Region Protection API Standard
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
2. [API Architecture](#api-architecture)
3. [Authentication](#authentication)
4. [Core Endpoints](#core-endpoints)
5. [Request/Response Formats](#requestresponse-formats)
6. [Error Handling](#error-handling)
7. [Rate Limiting](#rate-limiting)
8. [Examples](#examples)

---

## Overview

### 1.1 Purpose

The WIA Polar Region Protection API provides programmatic access to polar monitoring data, enabling researchers, governments, and organizations to:

- Submit and retrieve monitoring data
- Access real-time ice coverage and temperature information
- Track wildlife populations and ecosystem health
- Generate environmental impact reports
- Integrate with satellite and ground-based sensor networks

### 1.2 API Principles

1. **RESTful Design**: Standard HTTP methods and status codes
2. **JSON Payload**: All data exchanged in JSON format
3. **Authentication**: API key-based authentication
4. **Versioning**: URL-based versioning (/v1/, /v2/)
5. **Real-time**: WebSocket support for live data streams

---

## API Architecture

### 2.1 Base URL

**Production**: `https://api.wia-polar.org/v1`
**Staging**: `https://staging-api.wia-polar.org/v1`
**Development**: `http://localhost:3000/v1`

### 2.2 Supported Protocols

- **HTTP/HTTPS**: RESTful API
- **WebSocket**: Real-time monitoring streams
- **GraphQL**: Advanced querying (optional)

### 2.3 Data Formats

- **Request**: JSON (application/json)
- **Response**: JSON (application/json)
- **Authentication**: Bearer token (Authorization header)

---

## Authentication

### 3.1 API Key Authentication

All API requests require authentication via API key.

**Header Format**:
```
Authorization: Bearer YOUR_API_KEY
```

**Example Request**:
```bash
curl -X GET https://api.wia-polar.org/v1/regions/arctic \
  -H "Authorization: Bearer wia_polar_abc123xyz"
```

### 3.2 Obtaining API Keys

1. Register at `https://portal.wia-polar.org`
2. Create a new API key in dashboard
3. Select appropriate access scope (read, write, admin)
4. Include key in all requests

### 3.3 Access Scopes

| Scope | Permissions | Use Case |
|-------|-------------|----------|
| `read` | Read monitoring data | Public access, research |
| `write` | Submit monitoring data | Sensor networks, stations |
| `admin` | Full access + management | System administrators |

---

## Core Endpoints

### 4.1 Monitoring Data Endpoints

#### POST /v1/polar/monitor

Submit new polar monitoring data.

**Request Body**:
```json
{
  "region": "arctic",
  "monitoring": {
    "iceCoverage": {
      "value": 14000000,
      "unit": "km2",
      "measurementDate": "2025-01-15T10:00:00Z"
    },
    "temperature": {
      "air": -25.5,
      "water": -1.8,
      "anomaly": 2.1,
      "unit": "celsius"
    },
    "wildlife": [
      {
        "species": "Ursus maritimus",
        "population": 26000,
        "trend": "declining"
      }
    ]
  },
  "metadata": {
    "source": "satellite",
    "dataQuality": "high",
    "sensorId": "SAT-ARCTIC-001"
  }
}
```

**Response** (201 Created):
```json
{
  "status": "success",
  "message": "Monitoring data recorded successfully",
  "data": {
    "recordId": "POLAR-ARCTIC-2025-001",
    "timestamp": "2025-01-15T10:30:00Z",
    "region": "arctic"
  }
}
```

---

#### GET /v1/polar/{region}

Retrieve latest monitoring data for a specific region.

**Path Parameters**:
- `region`: arctic | antarctic | greenland | alaska | siberia

**Query Parameters**:
- `limit`: Number of records (default: 100, max: 1000)
- `startDate`: ISO8601 timestamp
- `endDate`: ISO8601 timestamp
- `dataQuality`: high | medium | low

**Example Request**:
```bash
GET /v1/polar/arctic?limit=10&startDate=2025-01-01T00:00:00Z
```

**Response** (200 OK):
```json
{
  "status": "success",
  "data": [
    {
      "recordId": "POLAR-ARCTIC-2025-001",
      "region": "arctic",
      "monitoring": {
        "iceCoverage": {
          "value": 14000000,
          "unit": "km2"
        },
        "temperature": {
          "air": -25.5,
          "anomaly": 2.1
        }
      },
      "metadata": {
        "timestamp": "2025-01-15T10:30:00Z",
        "source": "satellite"
      }
    }
  ],
  "pagination": {
    "total": 1500,
    "page": 1,
    "pageSize": 10
  }
}
```

---

#### GET /v1/polar/temperature/{region}

Retrieve temperature data and trends.

**Response** (200 OK):
```json
{
  "status": "success",
  "data": {
    "region": "arctic",
    "current": {
      "air": -25.5,
      "water": -1.8,
      "anomaly": 2.1
    },
    "trend": {
      "direction": "warming",
      "rate": "+0.042°C/year",
      "confidence": 0.95
    },
    "historical": {
      "average": -27.6,
      "min": -68.5,
      "max": 12.3
    }
  }
}
```

---

#### GET /v1/polar/wildlife/{region}

Retrieve wildlife population data.

**Response** (200 OK):
```json
{
  "status": "success",
  "data": {
    "region": "arctic",
    "species": [
      {
        "name": "Ursus maritimus",
        "commonName": "Polar Bear",
        "population": 26000,
        "trend": "declining",
        "threatLevel": "vulnerable",
        "habitat": "sea ice"
      },
      {
        "name": "Vulpes lagopus",
        "commonName": "Arctic Fox",
        "population": 200000,
        "trend": "stable",
        "threatLevel": "least concern"
      }
    ]
  }
}
```

---

### 4.2 Analysis Endpoints

#### POST /v1/polar/analysis/impact

Calculate environmental impact based on monitoring data.

**Request Body**:
```json
{
  "iceLoss": 50000,
  "temperatureRise": 2.5,
  "protectedArea": 100000
}
```

**Response** (200 OK):
```json
{
  "status": "success",
  "data": {
    "seaLevelRise": 5.0,
    "habitatLoss": 40000,
    "wildlifeThreat": 2500,
    "protectionCoverage": 200.0
  }
}
```

---

#### GET /v1/polar/trends/{region}

Retrieve historical trends and predictions.

**Response** (200 OK):
```json
{
  "status": "success",
  "data": {
    "region": "arctic",
    "iceCoverage": {
      "trend": "declining",
      "rate": -50000,
      "unit": "km2/year",
      "prediction2050": 10000000
    },
    "temperature": {
      "trend": "warming",
      "rate": 0.042,
      "unit": "celsius/year",
      "prediction2050": -20.0
    }
  }
}
```

---

### 4.3 Integration Endpoints

#### POST /v1/polar/satellite/integrate

Integrate satellite monitoring data.

**Request Body**:
```json
{
  "satelliteId": "SAT-ARCTIC-001",
  "imagery": {
    "url": "https://satellite-data.wia-polar.org/image123.tif",
    "format": "GeoTIFF",
    "resolution": "30m",
    "timestamp": "2025-01-15T10:00:00Z"
  },
  "metadata": {
    "coverage": "arctic",
    "cloudCover": 15,
    "quality": "high"
  }
}
```

**Response** (201 Created):
```json
{
  "status": "success",
  "message": "Satellite data integrated",
  "data": {
    "integrationId": "INT-SAT-2025-001",
    "processed": true,
    "iceCoverageDetected": 13950000
  }
}
```

---

## Request/Response Formats

### 5.1 Standard Response Format

**Success Response**:
```json
{
  "status": "success",
  "message": "Optional success message",
  "data": { /* response data */ }
}
```

**Error Response**:
```json
{
  "status": "error",
  "error": {
    "code": "ERROR_CODE",
    "message": "Human-readable error message",
    "details": { /* additional error context */ }
  }
}
```

---

## Error Handling

### 6.1 HTTP Status Codes

| Code | Meaning | Usage |
|------|---------|-------|
| 200 | OK | Successful GET request |
| 201 | Created | Successful POST request |
| 400 | Bad Request | Invalid request format |
| 401 | Unauthorized | Missing or invalid API key |
| 403 | Forbidden | Insufficient permissions |
| 404 | Not Found | Resource not found |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Server Error | Server-side error |

### 6.2 Error Response Examples

**Invalid Region** (400):
```json
{
  "status": "error",
  "error": {
    "code": "INVALID_REGION",
    "message": "Region 'pacific' is not a valid polar region",
    "details": {
      "validRegions": ["arctic", "antarctic", "greenland", "alaska", "siberia"]
    }
  }
}
```

**Rate Limit Exceeded** (429):
```json
{
  "status": "error",
  "error": {
    "code": "RATE_LIMIT_EXCEEDED",
    "message": "API rate limit exceeded",
    "details": {
      "limit": 1000,
      "remaining": 0,
      "resetAt": "2025-01-15T11:00:00Z"
    }
  }
}
```

---

## Rate Limiting

### 7.1 Rate Limit Tiers

| Tier | Requests/Hour | Use Case |
|------|---------------|----------|
| Free | 100 | Individual researchers |
| Research | 1,000 | Academic institutions |
| Professional | 10,000 | Organizations |
| Enterprise | 100,000 | Satellite networks |

### 7.2 Rate Limit Headers

All responses include rate limit information:

```
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 850
X-RateLimit-Reset: 1642248000
```

---

## Examples

### 8.1 Complete Monitoring Submission Workflow

```javascript
const WIA_API_KEY = 'wia_polar_abc123xyz';
const BASE_URL = 'https://api.wia-polar.org/v1';

async function submitMonitoringData() {
  const data = {
    region: 'arctic',
    monitoring: {
      iceCoverage: {
        value: 14000000,
        unit: 'km2',
        measurementDate: new Date().toISOString()
      },
      temperature: {
        air: -25.5,
        water: -1.8,
        anomaly: 2.1,
        unit: 'celsius'
      }
    },
    metadata: {
      source: 'satellite',
      dataQuality: 'high',
      sensorId: 'SAT-ARCTIC-001'
    }
  };

  const response = await fetch(`${BASE_URL}/polar/monitor`, {
    method: 'POST',
    headers: {
      'Authorization': `Bearer ${WIA_API_KEY}`,
      'Content-Type': 'application/json'
    },
    body: JSON.stringify(data)
  });

  const result = await response.json();
  console.log('Record ID:', result.data.recordId);
}
```

### 8.2 Retrieve and Analyze Data

```javascript
async function analyzeArcticTrends() {
  const response = await fetch(
    `${BASE_URL}/polar/trends/arctic`,
    {
      headers: {
        'Authorization': `Bearer ${WIA_API_KEY}`
      }
    }
  );

  const result = await response.json();

  console.log('Ice coverage trend:', result.data.iceCoverage.trend);
  console.log('Annual ice loss:', result.data.iceCoverage.rate, 'km²/year');
  console.log('2050 prediction:', result.data.iceCoverage.prediction2050, 'km²');
}
```

---

**License**: MIT
**Copyright**: © 2025 WIA - World Certification Industry Association
**Philosophy**: 弘益人間 (Hongik Ingan) - Benefit All Humanity
