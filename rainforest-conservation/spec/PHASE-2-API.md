# WIA Rainforest Conservation API Standard
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
4. [Endpoints](#endpoints)
5. [Request/Response Format](#requestresponse-format)
6. [Error Handling](#error-handling)
7. [Rate Limiting](#rate-limiting)
8. [Examples](#examples)

---

## Overview

### 1.1 Purpose

Define REST API endpoints for rainforest monitoring, biodiversity tracking, deforestation alerts, and conservation management.

### 1.2 Base URL

```
Production: https://api.wia.rainforest.org/v1
Staging: https://api-staging.wia.rainforest.org/v1
```

### 1.3 API Principles

- RESTful design
- JSON format
- HTTPS only
- API versioning
- Rate limiting
- OAuth 2.0 authentication

---

## API Architecture

### 2.1 Resource Structure

```
/forest           - Forest area management
/biodiversity     - Species and ecosystem data
/deforestation    - Deforestation monitoring
/carbon           - Carbon storage and credits
/indigenous       - Indigenous rights and territories
/satellite        - Satellite data integration
/alerts           - Real-time alerts
```

---

## Authentication

### 3.1 OAuth 2.0

**Token Endpoint**: `POST /auth/token`

**Request**:
```bash
curl -X POST https://api.wia.rainforest.org/v1/auth/token \
  -H "Content-Type: application/json" \
  -d '{
    "client_id": "your-client-id",
    "client_secret": "your-client-secret",
    "grant_type": "client_credentials"
  }'
```

**Response**:
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600
}
```

### 3.2 Using Bearer Token

```bash
curl -H "Authorization: Bearer {access_token}" \
  https://api.wia.rainforest.org/v1/forest
```

---

## Endpoints

### 4.1 Forest Monitoring

#### POST /forest/monitor
Record new forest monitoring data.

**Request**:
```json
{
  "coverage": 15000,
  "canopyCover": 85,
  "location": {
    "type": "Point",
    "coordinates": [-60.123, -3.456]
  },
  "forestType": "amazon"
}
```

**Response**: `201 Created`
```json
{
  "id": "FOREST-2025-AMZ-001",
  "status": "recorded",
  "timestamp": "2025-01-15T12:00:00Z",
  "nextMeasurement": "2025-01-22T12:00:00Z"
}
```

#### GET /forest/:id
Retrieve forest area details.

**Response**: `200 OK`
```json
{
  "id": "FOREST-2025-AMZ-001",
  "coverage": 15000,
  "canopyCover": 85,
  "deforestationRate": 120,
  "biodiversityIndex": 88.5,
  "carbonStorage": 450000,
  "alertLevel": "warning",
  "lastUpdated": "2025-01-15T12:00:00Z"
}
```

#### GET /forest
List all monitored forest areas.

**Query Parameters**:
- `forestType` (optional): Filter by forest type
- `alertLevel` (optional): Filter by alert level
- `page` (default: 1)
- `limit` (default: 20, max: 100)

**Response**: `200 OK`
```json
{
  "total": 142,
  "page": 1,
  "limit": 20,
  "data": [
    {
      "id": "FOREST-2025-AMZ-001",
      "coverage": 15000,
      "alertLevel": "warning"
    }
  ]
}
```

---

### 4.2 Biodiversity Tracking

#### POST /biodiversity/survey
Record biodiversity survey data.

**Request**:
```json
{
  "forestId": "FOREST-2025-AMZ-001",
  "speciesCount": 523,
  "endemicSpecies": 68,
  "threatenedSpecies": 12,
  "keystone": ["jaguar", "harpy eagle"]
}
```

**Response**: `201 Created`
```json
{
  "surveyId": "BIO-2025-001",
  "biodiversityIndex": 88.5,
  "status": "recorded"
}
```

#### GET /biodiversity/:forestId
Get biodiversity data for a forest area.

**Response**: `200 OK`
```json
{
  "forestId": "FOREST-2025-AMZ-001",
  "speciesCount": 523,
  "endemicSpecies": 68,
  "threatenedSpecies": 12,
  "biodiversityIndex": 88.5,
  "trend": "+2.3%",
  "lastSurvey": "2025-01-15T12:00:00Z"
}
```

---

### 4.3 Deforestation Alerts

#### POST /alerts/deforestation
Create deforestation alert.

**Request**:
```json
{
  "forestId": "FOREST-2025-AMZ-001",
  "severity": "high",
  "area": 250,
  "causes": ["logging", "agriculture"],
  "location": {
    "type": "Point",
    "coordinates": [-60.234, -3.567]
  }
}
```

**Response**: `201 Created`
```json
{
  "alertId": "ALERT-2025-001",
  "status": "active",
  "severity": "high",
  "notified": ["forest-authority", "conservation-ngo"],
  "timestamp": "2025-01-15T12:00:00Z"
}
```

#### GET /alerts/active
Get all active deforestation alerts.

**Response**: `200 OK`
```json
{
  "count": 5,
  "alerts": [
    {
      "alertId": "ALERT-2025-001",
      "forestId": "FOREST-2025-AMZ-001",
      "severity": "high",
      "area": 250,
      "timestamp": "2025-01-15T12:00:00Z"
    }
  ]
}
```

---

### 4.4 Carbon Storage

#### GET /carbon/:forestId
Get carbon storage data.

**Response**: `200 OK`
```json
{
  "forestId": "FOREST-2025-AMZ-001",
  "totalCarbon": 450000,
  "sequestrationRate": 3200,
  "carbonCredits": 15000,
  "biomass": 380000,
  "soilCarbon": 70000,
  "lastCalculated": "2025-01-15T12:00:00Z"
}
```

#### POST /carbon/credits/issue
Issue carbon credits for verified conservation.

**Request**:
```json
{
  "forestId": "FOREST-2025-AMZ-001",
  "carbonSequestered": 5000,
  "verificationMethod": "satellite-mRV",
  "period": "2024-2025"
}
```

**Response**: `201 Created`
```json
{
  "creditId": "CARBON-2025-001",
  "credits": 5000,
  "verified": true,
  "registry": "verra-vcs",
  "issued": "2025-01-15T12:00:00Z"
}
```

---

### 4.5 Indigenous Rights

#### POST /indigenous/territory
Register indigenous territory.

**Request**:
```json
{
  "territoryName": "Yanomami Territory",
  "community": "Yanomami People",
  "location": {
    "type": "Polygon",
    "coordinates": [[...]]
  },
  "landRights": "recognized",
  "managementAgreement": true
}
```

**Response**: `201 Created`
```json
{
  "territoryId": "INDIG-2025-001",
  "status": "registered",
  "protectedArea": 96650,
  "timestamp": "2025-01-15T12:00:00Z"
}
```

#### GET /indigenous/:territoryId
Get indigenous territory details.

**Response**: `200 OK`
```json
{
  "territoryId": "INDIG-2025-001",
  "territoryName": "Yanomami Territory",
  "community": "Yanomami People",
  "landRights": "recognized",
  "area": 96650,
  "forestCoverage": 98,
  "managementAgreement": true
}
```

---

### 4.6 Satellite Integration

#### POST /satellite/analyze
Request satellite imagery analysis.

**Request**:
```json
{
  "forestId": "FOREST-2025-AMZ-001",
  "satellite": "sentinel-2",
  "startDate": "2024-01-01",
  "endDate": "2025-01-01",
  "metrics": ["ndvi", "deforestation", "fire"]
}
```

**Response**: `202 Accepted`
```json
{
  "analysisId": "SAT-2025-001",
  "status": "processing",
  "estimatedCompletion": "2025-01-15T12:30:00Z"
}
```

#### GET /satellite/analysis/:analysisId
Get satellite analysis results.

**Response**: `200 OK`
```json
{
  "analysisId": "SAT-2025-001",
  "status": "completed",
  "results": {
    "ndvi": {
      "average": 0.78,
      "trend": "-0.02"
    },
    "deforestation": {
      "detected": true,
      "area": 250,
      "confidence": 0.95
    },
    "fire": {
      "detected": false
    }
  }
}
```

---

## Request/Response Format

### 5.1 Standard Response Structure

**Success Response**:
```json
{
  "status": "success",
  "data": { ... },
  "timestamp": "2025-01-15T12:00:00Z"
}
```

**Error Response**:
```json
{
  "status": "error",
  "error": {
    "code": "INVALID_FOREST_TYPE",
    "message": "Forest type must be one of: tropical, amazon, congo, southeast, atlantic",
    "field": "forestType"
  },
  "timestamp": "2025-01-15T12:00:00Z"
}
```

---

## Error Handling

### 6.1 HTTP Status Codes

| Code | Meaning | Usage |
|------|---------|-------|
| 200 | OK | Successful GET request |
| 201 | Created | Successful POST request |
| 202 | Accepted | Async processing started |
| 400 | Bad Request | Invalid input data |
| 401 | Unauthorized | Missing or invalid token |
| 403 | Forbidden | Insufficient permissions |
| 404 | Not Found | Resource doesn't exist |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Server Error | Server error |

### 6.2 Error Codes

| Code | Description |
|------|-------------|
| `INVALID_FOREST_TYPE` | Invalid forest type value |
| `COVERAGE_OUT_OF_RANGE` | Coverage value exceeds limits |
| `INVALID_GEOJSON` | Malformed GeoJSON |
| `FOREST_NOT_FOUND` | Forest ID doesn't exist |
| `DEFORESTATION_ALERT_FAILED` | Failed to create alert |
| `CARBON_CREDIT_ERROR` | Carbon credit issuance failed |

---

## Rate Limiting

### 7.1 Limits

| Tier | Requests/Hour | Requests/Day |
|------|---------------|--------------|
| Free | 100 | 1,000 |
| Standard | 1,000 | 10,000 |
| Premium | 10,000 | 100,000 |

### 7.2 Rate Limit Headers

```
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 995
X-RateLimit-Reset: 1642262400
```

---

## Examples

### 8.1 Complete Monitoring Workflow

```bash
# 1. Authenticate
TOKEN=$(curl -X POST https://api.wia.rainforest.org/v1/auth/token \
  -H "Content-Type: application/json" \
  -d '{"client_id":"xxx","client_secret":"yyy","grant_type":"client_credentials"}' \
  | jq -r .access_token)

# 2. Record forest data
curl -X POST https://api.wia.rainforest.org/v1/forest/monitor \
  -H "Authorization: Bearer $TOKEN" \
  -H "Content-Type: application/json" \
  -d '{
    "coverage": 15000,
    "canopyCover": 85,
    "location": {"type":"Point","coordinates":[-60.123,-3.456]},
    "forestType": "amazon"
  }'

# 3. Create deforestation alert
curl -X POST https://api.wia.rainforest.org/v1/alerts/deforestation \
  -H "Authorization: Bearer $TOKEN" \
  -H "Content-Type: application/json" \
  -d '{
    "forestId": "FOREST-2025-AMZ-001",
    "severity": "high",
    "area": 250,
    "causes": ["logging"]
  }'

# 4. Request satellite analysis
curl -X POST https://api.wia.rainforest.org/v1/satellite/analyze \
  -H "Authorization: Bearer $TOKEN" \
  -H "Content-Type: application/json" \
  -d '{
    "forestId": "FOREST-2025-AMZ-001",
    "satellite": "sentinel-2",
    "metrics": ["ndvi", "deforestation"]
  }'
```

---

**© 2025 WIA (World Certification Industry Association)**
**License**: MIT
**Philosophy**: 弘益人間 (Hongik Ingan) - Benefit All Humanity
