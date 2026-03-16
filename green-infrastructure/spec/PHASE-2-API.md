# WIA Green Infrastructure API Standard
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
2. [API Endpoints](#api-endpoints)
3. [Authentication](#authentication)
4. [Request/Response Format](#requestresponse-format)
5. [Error Handling](#error-handling)
6. [Rate Limiting](#rate-limiting)
7. [Examples](#examples)

---

## Overview

### 1.1 Purpose

The WIA Green Infrastructure API Standard defines REST API endpoints for registering, monitoring, and managing green infrastructure installations across urban environments.

### 1.2 Base URL

```
Production: https://api.wia.live/green-infrastructure/v1
Staging: https://staging-api.wia.live/green-infrastructure/v1
```

### 1.3 Protocol

- **Protocol**: HTTPS only
- **Format**: JSON
- **Encoding**: UTF-8
- **Authentication**: Bearer token (JWT)

---

## API Endpoints

### 2.1 Infrastructure Management

#### Register New Infrastructure

```http
POST /api/v1/infrastructure/register
```

**Request Body:**
```json
{
  "type": "green_roof",
  "subtype": "extensive",
  "location": {
    "gps": {"latitude": 37.5665, "longitude": 126.9780},
    "address": "123 Green Street, Seoul"
  },
  "dimensions": {
    "area": {"value": 500, "unit": "m2"}
  },
  "vegetation": {
    "coverage": 85,
    "types": ["sedum", "grasses"]
  }
}
```

**Response (201 Created):**
```json
{
  "status": "success",
  "data": {
    "infrastructureId": "GI-2025-000001",
    "certification": "WIA-GREEN-ABC12345",
    "created": "2025-01-15T10:30:00Z"
  }
}
```

#### Get Infrastructure Details

```http
GET /api/v1/infrastructure/{id}
```

**Response (200 OK):**
```json
{
  "status": "success",
  "data": {
    "infrastructureId": "GI-2025-000001",
    "type": "green_roof",
    "status": "active",
    "location": {
      "gps": {"latitude": 37.5665, "longitude": 126.9780}
    },
    "performance": {
      "stormwaterRetention": 450,
      "carbonSequestration": 510,
      "coolingEffect": 3.5
    }
  }
}
```

#### Update Infrastructure

```http
PUT /api/v1/infrastructure/{id}
```

#### Delete Infrastructure

```http
DELETE /api/v1/infrastructure/{id}
```

### 2.2 Monitoring Endpoints

#### Submit Sensor Data

```http
POST /api/v1/infrastructure/{id}/monitor
```

**Request Body:**
```json
{
  "sensorId": "SENSOR-GI-001",
  "type": "soil_moisture",
  "data": {
    "moisture": 65,
    "temperature": 22.5
  },
  "timestamp": "2025-01-15T10:30:00Z"
}
```

**Response (200 OK):**
```json
{
  "status": "success",
  "monitorId": "MON-2025-001",
  "received": "2025-01-15T10:30:05Z"
}
```

#### Get Latest Readings

```http
GET /api/v1/infrastructure/{id}/readings
```

**Query Parameters:**
- `sensor_type` (optional): Filter by sensor type
- `start_date` (optional): Start date for range
- `end_date` (optional): End date for range
- `limit` (optional): Max results (default: 100)

#### Get Alerts

```http
GET /api/v1/infrastructure/{id}/alerts
```

### 2.3 Performance Analytics

#### Calculate Impact

```http
GET /api/v1/infrastructure/{id}/impact
```

**Query Parameters:**
- `period`: "daily", "monthly", "annual"
- `metrics`: Comma-separated list (e.g., "stormwater,carbon,temperature")

**Response (200 OK):**
```json
{
  "status": "success",
  "period": "annual",
  "impact": {
    "stormwaterRetained": {
      "value": 450.5,
      "unit": "m3_per_year"
    },
    "carbonSequestered": {
      "value": 612.3,
      "unit": "kg_co2_per_year"
    },
    "coolingEffect": {
      "value": 3.2,
      "unit": "celsius"
    },
    "costSavings": {
      "value": 1250.75,
      "unit": "USD"
    }
  }
}
```

#### Generate Report

```http
POST /api/v1/infrastructure/{id}/report
```

**Request Body:**
```json
{
  "reportType": "performance",
  "period": {
    "start": "2024-01-01T00:00:00Z",
    "end": "2024-12-31T23:59:59Z"
  },
  "format": "pdf"
}
```

### 2.4 Maintenance Endpoints

#### Schedule Maintenance

```http
POST /api/v1/infrastructure/{id}/maintenance
```

#### Get Maintenance History

```http
GET /api/v1/infrastructure/{id}/maintenance/history
```

---

## Authentication

### 3.1 JWT Token

All API requests require authentication using JWT Bearer token:

```http
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
```

### 3.2 Obtaining Token

```http
POST /api/v1/auth/token
```

**Request Body:**
```json
{
  "apiKey": "your-api-key",
  "apiSecret": "your-api-secret"
}
```

**Response:**
```json
{
  "token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "expiresIn": 3600
}
```

---

## Request/Response Format

### 4.1 Standard Response Structure

**Success Response:**
```json
{
  "status": "success",
  "data": {},
  "metadata": {
    "timestamp": "2025-01-15T10:30:00Z",
    "version": "1.0.0"
  }
}
```

**Error Response:**
```json
{
  "status": "error",
  "error": {
    "code": "ERR_INVALID_DATA",
    "message": "Invalid infrastructure data",
    "details": {}
  },
  "metadata": {
    "timestamp": "2025-01-15T10:30:00Z"
  }
}
```

### 4.2 Pagination

For list endpoints:

```http
GET /api/v1/infrastructure?page=1&limit=20
```

**Response:**
```json
{
  "status": "success",
  "data": [],
  "pagination": {
    "page": 1,
    "limit": 20,
    "total": 150,
    "totalPages": 8
  }
}
```

---

## Error Handling

### 5.1 HTTP Status Codes

| Code | Status | Description |
|------|--------|-------------|
| 200 | OK | Request successful |
| 201 | Created | Resource created |
| 400 | Bad Request | Invalid request data |
| 401 | Unauthorized | Missing or invalid token |
| 403 | Forbidden | Insufficient permissions |
| 404 | Not Found | Resource not found |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Server Error | Server error |

### 5.2 Error Codes

| Code | Description |
|------|-------------|
| `ERR_INVALID_DATA` | Invalid request data |
| `ERR_INVALID_LOCATION` | Invalid GPS coordinates |
| `ERR_INFRASTRUCTURE_NOT_FOUND` | Infrastructure not found |
| `ERR_SENSOR_OFFLINE` | Sensor not responding |
| `ERR_RATE_LIMIT` | Rate limit exceeded |

---

## Rate Limiting

### 6.1 Limits

| Tier | Requests/minute | Requests/hour |
|------|-----------------|---------------|
| Free | 60 | 1,000 |
| Standard | 300 | 10,000 |
| Premium | 1,000 | 50,000 |

### 6.2 Headers

```http
X-RateLimit-Limit: 60
X-RateLimit-Remaining: 45
X-RateLimit-Reset: 1642342800
```

---

## Examples

### 7.1 Register Green Roof

```bash
curl -X POST https://api.wia.live/green-infrastructure/v1/infrastructure/register \
  -H "Authorization: Bearer YOUR_TOKEN" \
  -H "Content-Type: application/json" \
  -d '{
    "type": "green_roof",
    "location": {
      "gps": {"latitude": 37.5665, "longitude": 126.9780}
    },
    "dimensions": {
      "area": {"value": 500, "unit": "m2"}
    },
    "vegetation": {
      "coverage": 85
    }
  }'
```

### 7.2 Submit Sensor Reading

```bash
curl -X POST https://api.wia.live/green-infrastructure/v1/infrastructure/GI-2025-001/monitor \
  -H "Authorization: Bearer YOUR_TOKEN" \
  -H "Content-Type: application/json" \
  -d '{
    "sensorId": "SENSOR-001",
    "type": "soil_moisture",
    "data": {
      "moisture": 65,
      "temperature": 22.5
    }
  }'
```

### 7.3 Get Performance Impact

```bash
curl -X GET "https://api.wia.live/green-infrastructure/v1/infrastructure/GI-2025-001/impact?period=annual" \
  -H "Authorization: Bearer YOUR_TOKEN"
```

---

<div align="center">

**WIA Green Infrastructure API v1.0.0**

**弘益人間 (홍익인간)** - Benefit All Humanity

---

**© 2025 WIA**

**MIT License**

</div>
