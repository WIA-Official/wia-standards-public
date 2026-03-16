# WIA Mangrove Restoration API Standard
## Phase 2 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT

---

## Table of Contents

1. [Overview](#overview)
2. [API Design](#api-design)
3. [Endpoints](#endpoints)
4. [Authentication](#authentication)
5. [Request/Response](#requestresponse)
6. [Error Handling](#error-handling)
7. [Rate Limiting](#rate-limiting)
8. [Examples](#examples)

---

## Overview

### 1.1 Purpose

The WIA Mangrove Restoration API enables programmatic access to mangrove ecosystem data, carbon accounting, coastal protection metrics, and biodiversity monitoring through RESTful endpoints.

### 1.2 Base URL

```
Production: https://api.wia.live/mangrove/v1
Sandbox: https://sandbox.api.wia.live/mangrove/v1
```

### 1.3 API Principles

- **RESTful**: Standard HTTP methods (GET, POST, PUT, DELETE)
- **JSON**: All requests and responses use JSON format
- **Stateless**: Each request contains all necessary information
- **Versioned**: API version in URL path
- **Secure**: HTTPS required, OAuth 2.0 authentication

---

## API Design

### 2.1 Resource Structure

```
/mangrove/
  /sites                    # Restoration sites
  /species                  # Mangrove species
  /carbon                   # Carbon accounting
  /coastal                  # Coastal protection
  /monitoring               # Monitoring data
  /reports                  # Analytics reports
```

### 2.2 HTTP Methods

| Method | Usage | Idempotent |
|--------|-------|------------|
| GET | Retrieve resources | ✅ Yes |
| POST | Create new resources | ❌ No |
| PUT | Update entire resources | ✅ Yes |
| PATCH | Partial resource update | ❌ No |
| DELETE | Remove resources | ✅ Yes |

---

## Endpoints

### 3.1 Sites Management

#### Create Restoration Site

```http
POST /api/v1/mangrove/sites
Content-Type: application/json
Authorization: Bearer {token}

{
  "location": {
    "gps": {"latitude": 10.4806, "longitude": 123.3050},
    "region": "Central Visayas",
    "ecosystemType": "fringe"
  },
  "coverage": {"value": 25.5, "unit": "hectares"},
  "status": "planning"
}
```

**Response: 201 Created**

```json
{
  "siteId": "MANG-2025-000001",
  "status": "planning",
  "created": "2025-01-15T10:30:00Z",
  "location": {
    "gps": {"latitude": 10.4806, "longitude": 123.3050},
    "region": "Central Visayas",
    "ecosystemType": "fringe"
  }
}
```

#### Get Site Details

```http
GET /api/v1/mangrove/sites/{siteId}
Authorization: Bearer {token}
```

**Response: 200 OK**

```json
{
  "siteId": "MANG-2025-000001",
  "status": "active",
  "coverage": {"value": 25.5, "unit": "hectares"},
  "species": [
    {
      "scientificName": "Rhizophora apiculata",
      "coverage": {"value": 12.5, "unit": "hectares"}
    }
  ],
  "carbon": {
    "totalCarbon": {"value": 660.7, "unit": "tons_c_ha"}
  }
}
```

#### Update Site

```http
PUT /api/v1/mangrove/sites/{siteId}
Content-Type: application/json
Authorization: Bearer {token}

{
  "status": "monitoring",
  "coverage": {"value": 28.0, "unit": "hectares"}
}
```

#### List Sites

```http
GET /api/v1/mangrove/sites?status=active&limit=50&offset=0
Authorization: Bearer {token}
```

**Query Parameters:**

| Parameter | Type | Description | Default |
|-----------|------|-------------|---------|
| `status` | string | Filter by status | all |
| `region` | string | Filter by region | all |
| `limit` | integer | Results per page | 20 |
| `offset` | integer | Pagination offset | 0 |

---

### 3.2 Species Management

#### Add Species to Site

```http
POST /api/v1/mangrove/sites/{siteId}/species
Content-Type: application/json
Authorization: Bearer {token}

{
  "scientificName": "Rhizophora apiculata",
  "commonName": "Red Mangrove",
  "coverage": {"value": 12.5, "unit": "hectares"},
  "density": {"value": 1200, "unit": "trees_per_hectare"}
}
```

#### Get Species List

```http
GET /api/v1/mangrove/species
Authorization: Bearer {token}
```

**Response: 200 OK**

```json
{
  "species": [
    {
      "scientificName": "Rhizophora apiculata",
      "commonName": "Red Mangrove",
      "region": "Indo-Pacific",
      "carbonSequestration": "high"
    }
  ]
}
```

---

### 3.3 Carbon Accounting

#### Calculate Carbon Stock

```http
POST /api/v1/mangrove/carbon/calculate
Content-Type: application/json
Authorization: Bearer {token}

{
  "siteId": "MANG-2025-000001",
  "measurements": {
    "abovegroundBiomass": {"value": 125.5, "unit": "tons_c_ha"},
    "belowgroundBiomass": {"value": 85.2, "unit": "tons_c_ha"},
    "soilCarbon": {"value": 450.0, "unit": "tons_c_ha"}
  }
}
```

**Response: 200 OK**

```json
{
  "siteId": "MANG-2025-000001",
  "carbon": {
    "totalCarbon": {"value": 660.7, "unit": "tons_c_ha"},
    "totalCO2Equivalent": {"value": 2422.6, "unit": "tons_co2_ha"},
    "sequestrationRate": {"value": 12.5, "unit": "tons_co2_ha_year"}
  },
  "carbonCredits": {
    "potential": 2500,
    "estimatedValue": {"amount": 50000, "currency": "USD"}
  }
}
```

#### Get Carbon Report

```http
GET /api/v1/mangrove/sites/{siteId}/carbon
Authorization: Bearer {token}
```

---

### 3.4 Coastal Protection

#### Assess Protection Metrics

```http
POST /api/v1/mangrove/coastal/assess
Content-Type: application/json
Authorization: Bearer {token}

{
  "siteId": "MANG-2025-000001",
  "shorelineLength": {"value": 2.5, "unit": "km"},
  "mangroveWidth": {"value": 100, "unit": "m"},
  "treeDensity": {"value": 1200, "unit": "trees_per_hectare"}
}
```

**Response: 200 OK**

```json
{
  "siteId": "MANG-2025-000001",
  "protection": {
    "waveAttenuation": {"value": 70, "unit": "percent"},
    "erosionReduction": {"value": 85, "unit": "percent"},
    "stormSurgeProtection": "high",
    "floodRiskReduction": {"value": 60, "unit": "percent"}
  },
  "economicValue": {
    "annualBenefit": {"amount": 250000, "currency": "USD"}
  }
}
```

---

### 3.5 Monitoring Data

#### Submit Monitoring Data

```http
POST /api/v1/mangrove/monitoring
Content-Type: application/json
Authorization: Bearer {token}

{
  "siteId": "MANG-2025-000001",
  "timestamp": "2025-01-15T10:30:00Z",
  "waterQuality": {
    "salinity": {"value": 15.5, "unit": "ppt"},
    "temperature": {"value": 28.5, "unit": "celsius"},
    "pH": 8.1
  },
  "biodiversity": {
    "marineSpeciesCount": 45,
    "birdSpeciesCount": 12
  }
}
```

#### Get Monitoring History

```http
GET /api/v1/mangrove/sites/{siteId}/monitoring?startDate=2024-01-01&endDate=2025-01-15
Authorization: Bearer {token}
```

---

### 3.6 Reports & Analytics

#### Generate Site Report

```http
POST /api/v1/mangrove/reports/site
Content-Type: application/json
Authorization: Bearer {token}

{
  "siteId": "MANG-2025-000001",
  "period": {
    "start": "2024-01-01T00:00:00Z",
    "end": "2025-01-15T23:59:59Z"
  },
  "sections": ["coverage", "carbon", "coastal", "biodiversity"]
}
```

**Response: 200 OK**

```json
{
  "reportId": "RPT-2025-000123",
  "generated": "2025-01-15T10:30:00Z",
  "summary": {
    "coverageChange": "+15%",
    "carbonSequestered": {"value": 125.5, "unit": "tons_co2"},
    "shorelineProtected": {"value": 2.5, "unit": "km"}
  },
  "downloadUrl": "https://reports.wia.live/MANG-2025-000001/2025-01.pdf"
}
```

---

## Authentication

### 4.1 OAuth 2.0

```http
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials
&client_id={your_client_id}
&client_secret={your_client_secret}
&scope=mangrove:read mangrove:write
```

**Response:**

```json
{
  "access_token": "eyJhbGc...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "mangrove:read mangrove:write"
}
```

### 4.2 API Key (Sandbox Only)

```http
GET /api/v1/mangrove/sites
X-API-Key: your_api_key_here
```

---

## Request/Response

### 5.1 Common Headers

**Request Headers:**

```http
Authorization: Bearer {access_token}
Content-Type: application/json
Accept: application/json
X-Request-ID: {uuid}
```

**Response Headers:**

```http
Content-Type: application/json
X-Request-ID: {uuid}
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 999
X-RateLimit-Reset: 1642262400
```

---

## Error Handling

### 6.1 Error Response Format

```json
{
  "error": {
    "code": "INVALID_SITE_ID",
    "message": "Site ID format is invalid",
    "details": {
      "field": "siteId",
      "expected": "MANG-YYYY-NNNNNN",
      "received": "INVALID-123"
    },
    "timestamp": "2025-01-15T10:30:00Z",
    "requestId": "req-123456"
  }
}
```

### 6.2 Error Codes

| HTTP Status | Error Code | Description |
|-------------|------------|-------------|
| 400 | `INVALID_REQUEST` | Malformed request |
| 401 | `UNAUTHORIZED` | Missing or invalid auth |
| 403 | `FORBIDDEN` | Insufficient permissions |
| 404 | `NOT_FOUND` | Resource not found |
| 409 | `CONFLICT` | Resource already exists |
| 422 | `VALIDATION_ERROR` | Invalid data format |
| 429 | `RATE_LIMIT_EXCEEDED` | Too many requests |
| 500 | `INTERNAL_ERROR` | Server error |

---

## Rate Limiting

### 7.1 Limits

| Tier | Requests/Hour | Burst |
|------|---------------|-------|
| Free | 100 | 10 |
| Basic | 1,000 | 50 |
| Pro | 10,000 | 200 |
| Enterprise | Unlimited | 1000 |

### 7.2 Headers

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 999
X-RateLimit-Reset: 1642262400
```

---

## Examples

### 8.1 Complete Workflow

```javascript
// 1. Authenticate
const token = await getAccessToken();

// 2. Create site
const site = await fetch('https://api.wia.live/mangrove/v1/sites', {
  method: 'POST',
  headers: {
    'Authorization': `Bearer ${token}`,
    'Content-Type': 'application/json'
  },
  body: JSON.stringify({
    location: {
      gps: {latitude: 10.4806, longitude: 123.3050},
      ecosystemType: 'fringe'
    },
    coverage: {value: 25.5, unit: 'hectares'}
  })
});

// 3. Add species
await fetch(`https://api.wia.live/mangrove/v1/sites/${site.siteId}/species`, {
  method: 'POST',
  headers: {
    'Authorization': `Bearer ${token}`,
    'Content-Type': 'application/json'
  },
  body: JSON.stringify({
    scientificName: 'Rhizophora apiculata',
    coverage: {value: 12.5, unit: 'hectares'}
  })
});

// 4. Calculate carbon
const carbon = await fetch('https://api.wia.live/mangrove/v1/carbon/calculate', {
  method: 'POST',
  headers: {
    'Authorization': `Bearer ${token}`,
    'Content-Type': 'application/json'
  },
  body: JSON.stringify({
    siteId: site.siteId,
    measurements: {
      abovegroundBiomass: {value: 125.5, unit: 'tons_c_ha'}
    }
  })
});
```

---

<div align="center">

**WIA Mangrove Restoration API v1.0.0**

**弘益人間 (홍익인간)** - Benefit All Humanity

---

**© 2025 WIA**

**MIT License**

</div>
