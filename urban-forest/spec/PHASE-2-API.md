# WIA Urban Forest Creation API Standard
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
2. [API Architecture](#api-architecture)
3. [Authentication](#authentication)
4. [Core Endpoints](#core-endpoints)
5. [Forest Management](#forest-management)
6. [Tree Operations](#tree-operations)
7. [Carbon Tracking](#carbon-tracking)
8. [Monitoring & Analytics](#monitoring--analytics)
9. [Error Handling](#error-handling)
10. [Rate Limiting](#rate-limiting)

---

## Overview

### 1.1 Purpose

The WIA Urban Forest API provides RESTful endpoints for managing urban forests, tracking tree health, monitoring carbon sequestration, and analyzing biodiversity data.

### 1.2 Base URL

```
Production:  https://api.wia.live/urban-forest/v1
Staging:     https://api-staging.wia.live/urban-forest/v1
```

### 1.3 API Principles

- RESTful design
- JSON request/response
- OAuth 2.0 authentication
- Versioned endpoints
- Comprehensive error messages

---

## API Architecture

### 2.1 Request Format

```http
POST /api/v1/forest HTTP/1.1
Host: api.wia.live
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "city": "Seoul",
  "zoneType": "public_park",
  "trees": [...]
}
```

### 2.2 Response Format

```json
{
  "status": "success",
  "data": {
    "forestId": "FOREST-2025-SEOUL-001",
    "created": "2025-01-15T10:30:00Z"
  },
  "meta": {
    "version": "1.0.0",
    "timestamp": "2025-01-15T10:30:00Z"
  }
}
```

---

## Authentication

### 3.1 OAuth 2.0 Flow

```http
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials
&client_id={your_client_id}
&client_secret={your_client_secret}
&scope=forest:read forest:write
```

**Response:**
```json
{
  "access_token": "eyJhbGc...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "forest:read forest:write"
}
```

### 3.2 API Key (Alternative)

```http
GET /api/v1/forest/{id}
X-API-Key: your_api_key_here
```

---

## Core Endpoints

### 4.1 Health Check

```http
GET /api/v1/health
```

**Response:**
```json
{
  "status": "healthy",
  "version": "1.0.0",
  "uptime": 99.99,
  "services": {
    "database": "operational",
    "carbon_calculator": "operational",
    "monitoring": "operational"
  }
}
```

### 4.2 Get API Version

```http
GET /api/v1/version
```

**Response:**
```json
{
  "version": "1.0.0",
  "releaseDate": "2025-01-15",
  "features": ["forest-management", "carbon-tracking", "biodiversity"]
}
```

---

## Forest Management

### 5.1 Create Forest

```http
POST /api/v1/forest
Authorization: Bearer {token}
Content-Type: application/json
```

**Request Body:**
```json
{
  "city": "Seoul",
  "district": "Gangnam-gu",
  "location": {
    "latitude": 37.5665,
    "longitude": 126.9780,
    "address": "123 Urban Park St"
  },
  "zoneType": "public_park",
  "trees": [
    {
      "species": {
        "scientificName": "Zelkova serrata",
        "commonName": "Japanese Zelkova"
      },
      "plantedDate": "2025-01-15",
      "quantity": 50
    }
  ]
}
```

**Response (201 Created):**
```json
{
  "status": "success",
  "data": {
    "forestId": "FOREST-2025-SEOUL-001",
    "created": "2025-01-15T10:30:00Z",
    "treesPlanted": 50,
    "estimatedCarbonOffset": "1088.5 kg CO2/year"
  }
}
```

### 5.2 Get Forest Details

```http
GET /api/v1/forest/{forestId}
Authorization: Bearer {token}
```

**Response (200 OK):**
```json
{
  "status": "success",
  "data": {
    "forestId": "FOREST-2025-SEOUL-001",
    "city": "Seoul",
    "district": "Gangnam-gu",
    "status": "active",
    "totalTrees": 50,
    "area": "2.5 hectares",
    "carbon": {
      "annualSequestration": "1088.5 kg CO2",
      "totalSequestered": "5442.5 kg CO2"
    },
    "biodiversity": {
      "speciesCount": 15,
      "biodiversityIndex": 68
    }
  }
}
```

### 5.3 Update Forest

```http
PATCH /api/v1/forest/{forestId}
Authorization: Bearer {token}
Content-Type: application/json
```

**Request Body:**
```json
{
  "status": "maintenance",
  "notes": "Undergoing seasonal maintenance"
}
```

**Response (200 OK):**
```json
{
  "status": "success",
  "data": {
    "forestId": "FOREST-2025-SEOUL-001",
    "updated": "2025-01-15T12:00:00Z"
  }
}
```

### 5.4 List Forests

```http
GET /api/v1/forest?city=Seoul&status=active&limit=20&offset=0
Authorization: Bearer {token}
```

**Response (200 OK):**
```json
{
  "status": "success",
  "data": [
    {
      "forestId": "FOREST-2025-SEOUL-001",
      "city": "Seoul",
      "totalTrees": 50,
      "carbonOffset": "1088.5 kg CO2/year"
    }
  ],
  "pagination": {
    "total": 150,
    "limit": 20,
    "offset": 0,
    "hasMore": true
  }
}
```

---

## Tree Operations

### 6.1 Add Tree

```http
POST /api/v1/forest/{forestId}/tree
Authorization: Bearer {token}
Content-Type: application/json
```

**Request Body:**
```json
{
  "species": {
    "scientificName": "Quercus robur",
    "commonName": "English Oak"
  },
  "location": {
    "latitude": 37.5665,
    "longitude": 126.9780
  },
  "plantedDate": "2025-01-15",
  "height": 1.5,
  "dbh": 3.2
}
```

**Response (201 Created):**
```json
{
  "status": "success",
  "data": {
    "treeId": "TREE-051",
    "forestId": "FOREST-2025-SEOUL-001",
    "qrCode": "https://wia.live/tree/TREE-051/qr"
  }
}
```

### 6.2 Update Tree Health

```http
PATCH /api/v1/forest/{forestId}/tree/{treeId}
Authorization: Bearer {token}
Content-Type: application/json
```

**Request Body:**
```json
{
  "health": "healthy",
  "height": 2.1,
  "dbh": 4.5,
  "notes": "Strong growth observed"
}
```

**Response (200 OK):**
```json
{
  "status": "success",
  "data": {
    "treeId": "TREE-051",
    "updated": "2025-01-15T14:00:00Z",
    "healthTrend": "improving"
  }
}
```

### 6.3 Get Tree History

```http
GET /api/v1/forest/{forestId}/tree/{treeId}/history
Authorization: Bearer {token}
```

**Response (200 OK):**
```json
{
  "status": "success",
  "data": {
    "treeId": "TREE-051",
    "inspections": [
      {
        "date": "2025-01-15",
        "height": 2.1,
        "health": "healthy",
        "inspector": "park-ranger-001"
      }
    ],
    "growthRate": "+0.6m/year"
  }
}
```

---

## Carbon Tracking

### 7.1 Calculate Carbon Offset

```http
POST /api/v1/carbon/calculate
Authorization: Bearer {token}
Content-Type: application/json
```

**Request Body:**
```json
{
  "forestId": "FOREST-2025-SEOUL-001",
  "timeframe": "annual"
}
```

**Response (200 OK):**
```json
{
  "status": "success",
  "data": {
    "forestId": "FOREST-2025-SEOUL-001",
    "carbonSequestration": {
      "annual": "1088.5 kg CO2",
      "lifetime": "5442.5 kg CO2"
    },
    "oxygenProduction": {
      "annual": "791.5 kg O2"
    },
    "equivalents": {
      "carsDriven": "2.4 cars/year",
      "treesPlanted": "equivalent to 50 trees"
    }
  }
}
```

### 7.2 Get Carbon Credits

```http
GET /api/v1/carbon/credits/{forestId}
Authorization: Bearer {token}
```

**Response (200 OK):**
```json
{
  "status": "success",
  "data": {
    "forestId": "FOREST-2025-SEOUL-001",
    "credits": {
      "available": 5.44,
      "issued": 0,
      "pending": 5.44,
      "value": "$272.10 USD"
    },
    "verificationStatus": "pending"
  }
}
```

---

## Monitoring & Analytics

### 8.1 Forest Health Dashboard

```http
GET /api/v1/analytics/health/{forestId}
Authorization: Bearer {token}
```

**Response (200 OK):**
```json
{
  "status": "success",
  "data": {
    "forestId": "FOREST-2025-SEOUL-001",
    "healthMetrics": {
      "overall": 85,
      "treeHealth": {
        "excellent": 15,
        "healthy": 30,
        "fair": 5,
        "poor": 0
      },
      "soilQuality": 78,
      "biodiversity": 68
    },
    "alerts": []
  }
}
```

### 8.2 Biodiversity Report

```http
GET /api/v1/analytics/biodiversity/{forestId}
Authorization: Bearer {token}
```

**Response (200 OK):**
```json
{
  "status": "success",
  "data": {
    "forestId": "FOREST-2025-SEOUL-001",
    "species": {
      "birds": 15,
      "insects": 45,
      "plants": 8
    },
    "biodiversityIndex": 68,
    "trend": "improving",
    "keySpecies": ["Korean magpie", "Great tit", "Honeybee"]
  }
}
```

---

## Error Handling

### 9.1 Error Response Format

```json
{
  "status": "error",
  "error": {
    "code": "FOREST_NOT_FOUND",
    "message": "Forest with ID FOREST-2025-SEOUL-999 not found",
    "details": {
      "forestId": "FOREST-2025-SEOUL-999"
    }
  },
  "meta": {
    "timestamp": "2025-01-15T10:30:00Z",
    "requestId": "req-12345"
  }
}
```

### 9.2 Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| `INVALID_REQUEST` | 400 | Malformed request |
| `UNAUTHORIZED` | 401 | Missing or invalid auth |
| `FORBIDDEN` | 403 | Insufficient permissions |
| `FOREST_NOT_FOUND` | 404 | Forest doesn't exist |
| `TREE_NOT_FOUND` | 404 | Tree doesn't exist |
| `VALIDATION_ERROR` | 422 | Invalid data format |
| `RATE_LIMIT_EXCEEDED` | 429 | Too many requests |
| `INTERNAL_ERROR` | 500 | Server error |

---

## Rate Limiting

### 10.1 Rate Limits

| Tier | Requests/hour | Requests/day |
|------|---------------|--------------|
| Free | 100 | 1,000 |
| Basic | 1,000 | 10,000 |
| Pro | 10,000 | 100,000 |
| Enterprise | Unlimited | Unlimited |

### 10.2 Rate Limit Headers

```http
HTTP/1.1 200 OK
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 999
X-RateLimit-Reset: 1642248000
```

---

<div align="center">

**WIA Urban Forest Creation API v1.0.0**

**弘益人間 (홍익인간)** - Benefit All Humanity

---

**© 2025 WIA**

**MIT License**

</div>
