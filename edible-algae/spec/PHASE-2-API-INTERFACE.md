# WIA Edible Algae API Interface Standard
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
5. [Cultivation Management](#cultivation-management)
6. [Real-Time Monitoring](#real-time-monitoring)
7. [Quality Control](#quality-control)
8. [Harvest Management](#harvest-management)
9. [Analytics & Reporting](#analytics--reporting)
10. [Error Handling](#error-handling)
11. [Rate Limiting](#rate-limiting)
12. [Webhooks](#webhooks)

---

## Overview

### 1.1 Purpose

The WIA Edible Algae API provides RESTful endpoints for managing algae cultivation operations, monitoring photobioreactors, tracking nutritional data, and ensuring quality control throughout the production lifecycle.

### 1.2 Base URL

```
Production: https://api.wia-edible-algae.org/v1
Staging: https://api-staging.wia-edible-algae.org/v1
```

### 1.3 API Design Principles

- **RESTful**: Resource-oriented URLs, HTTP verbs
- **JSON**: Request/response in JSON format
- **Versioned**: API version in URL path
- **Stateless**: No server-side session storage
- **Idempotent**: Safe retry of failed requests
- **Real-time**: WebSocket support for live monitoring

---

## API Architecture

### 2.1 Request Format

```http
GET /v1/cultivations/CULT-2025-ALG-001 HTTP/1.1
Host: api.wia-edible-algae.org
Authorization: Bearer <access_token>
Content-Type: application/json
Accept: application/json
X-Request-ID: req-123e4567-e89b-12d3-a456-426614174000
```

### 2.2 Response Format

```json
{
  "status": "success",
  "data": {
    "cultivation_id": "CULT-2025-ALG-001",
    "species": "Spirulina platensis",
    "current_density": 3.8
  },
  "meta": {
    "request_id": "req-123e4567-e89b-12d3-a456-426614174000",
    "timestamp": "2025-01-15T10:30:00Z",
    "version": "1.0.0"
  }
}
```

---

## Authentication

### 3.1 OAuth 2.0

```http
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials
&client_id=YOUR_CLIENT_ID
&client_secret=YOUR_CLIENT_SECRET
&scope=cultivation:read cultivation:write
```

**Response**:
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "cultivation:read cultivation:write"
}
```

### 3.2 API Key (Alternative)

```http
GET /v1/cultivations
X-API-Key: wia_live_sk_1234567890abcdef
```

---

## Core Endpoints

### 4.1 Cultivation CRUD

#### Create Cultivation Batch

```http
POST /v1/cultivations
Content-Type: application/json

{
  "facility_id": "FAC-OCEAN-FARMS-01",
  "species": {
    "scientific_name": "Arthrospira platensis",
    "strain_id": "SPR-HI-2023"
  },
  "cultivation_system": {
    "type": "PHOTOBIOREACTOR",
    "volume_liters": 5000
  },
  "environmental_parameters": {
    "temperature": {"setpoint": 25.0},
    "ph": {"setpoint": 7.8},
    "light_intensity": {"setpoint": 400}
  }
}
```

**Response**:
```json
{
  "status": "success",
  "data": {
    "cultivation_id": "CULT-2025-ALG-042",
    "created_at": "2025-01-15T10:00:00Z",
    "status": "INOCULATION",
    "estimated_harvest": "2025-02-05"
  }
}
```

#### Get Cultivation Details

```http
GET /v1/cultivations/{cultivation_id}
```

**Response**: Full cultivation object (see Phase 1 Data Format)

#### Update Cultivation Parameters

```http
PATCH /v1/cultivations/{cultivation_id}
Content-Type: application/json

{
  "environmental_parameters": {
    "temperature": {"setpoint": 26.0},
    "light_intensity": {"setpoint": 450}
  }
}
```

#### List Cultivations

```http
GET /v1/cultivations?status=ACTIVE_GROWTH&species=SPIRULINA&limit=50
```

**Query Parameters**:
- `status`: Filter by lifecycle state
- `species`: Filter by algae species
- `facility_id`: Filter by facility
- `start_date`: Filter by creation date (ISO 8601)
- `limit`: Results per page (default 20, max 100)
- `offset`: Pagination offset

---

## Cultivation Management

### 5.1 Growth Monitoring

#### Record Measurement

```http
POST /v1/cultivations/{cultivation_id}/measurements
Content-Type: application/json

{
  "timestamp": "2025-01-15T14:00:00Z",
  "optical_density_OD680": 1.35,
  "biomass_concentration_g_per_L": 4.2,
  "ph": 7.85,
  "temperature_celsius": 25.2,
  "dissolved_oxygen_mg_per_L": 7.8
}
```

#### Get Growth Curve

```http
GET /v1/cultivations/{cultivation_id}/growth-curve?start_date=2025-01-01&end_date=2025-01-15
```

**Response**:
```json
{
  "status": "success",
  "data": {
    "measurements": [
      {"timestamp": "2025-01-01T08:00:00Z", "density": 0.5},
      {"timestamp": "2025-01-02T08:00:00Z", "density": 0.8},
      {"timestamp": "2025-01-15T08:00:00Z", "density": 4.2}
    ],
    "growth_curve_fit": {
      "model": "LOGISTIC",
      "r_squared": 0.98,
      "max_specific_growth_rate": 0.35
    },
    "predicted_harvest_date": "2025-01-18"
  }
}
```

### 5.2 Nutrient Management

#### Update Nutrient Levels

```http
POST /v1/cultivations/{cultivation_id}/nutrients
Content-Type: application/json

{
  "nitrogen_concentration": 160.0,
  "phosphorus_concentration": 28.0,
  "co2_supply_rate": 2.8
}
```

---

## Real-Time Monitoring

### 6.1 WebSocket Connection

```javascript
const ws = new WebSocket('wss://api.wia-edible-algae.org/v1/ws');

ws.onopen = () => {
  ws.send(JSON.stringify({
    action: 'subscribe',
    channel: 'cultivation:CULT-2025-ALG-001',
    auth: { token: 'Bearer ...' }
  }));
};

ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  console.log('Real-time update:', data);
  // { type: 'measurement', biomass_density: 4.25, timestamp: '...' }
};
```

### 6.2 Alerts & Notifications

```http
GET /v1/cultivations/{cultivation_id}/alerts
```

**Response**:
```json
{
  "status": "success",
  "data": {
    "alerts": [
      {
        "alert_id": "ALT-2025-001",
        "severity": "WARNING",
        "type": "TEMPERATURE_HIGH",
        "message": "Temperature 32°C exceeds optimal range",
        "timestamp": "2025-01-15T12:30:00Z",
        "recommended_action": "Activate cooling system"
      }
    ]
  }
}
```

---

## Quality Control

### 7.1 Nutritional Analysis

```http
POST /v1/cultivations/{cultivation_id}/analysis
Content-Type: application/json

{
  "analysis_type": "NUTRITIONAL_PROFILE",
  "sample_id": "SAMP-2025-001",
  "protein_percent": 65.2,
  "carbohydrates_percent": 18.5,
  "lipids_percent": 8.3,
  "phycocyanin_percent": 15.0
}
```

### 7.2 Contamination Screening

```http
POST /v1/cultivations/{cultivation_id}/contamination-test
Content-Type: application/json

{
  "test_date": "2025-01-15",
  "bacteria_cfu_per_g": 85,
  "e_coli": "ABSENT",
  "salmonella": "ABSENT",
  "heavy_metals_ppm": {
    "lead": 0.04,
    "cadmium": 0.01,
    "mercury": 0.005
  }
}
```

---

## Harvest Management

### 8.1 Create Harvest Record

```http
POST /v1/harvests
Content-Type: application/json

{
  "cultivation_id": "CULT-2025-ALG-001",
  "harvest_date": "2025-01-18T14:00:00Z",
  "harvest_method": "CONTINUOUS_FLOW",
  "wet_weight_kg": 450,
  "dry_weight_kg": 22.5
}
```

**Response**:
```json
{
  "status": "success",
  "data": {
    "harvest_id": "HRV-2025-ALG-001",
    "cultivation_id": "CULT-2025-ALG-001",
    "harvest_efficiency": {
      "recovery_rate_percent": 85,
      "yield_per_liter": 4.5
    }
  }
}
```

### 8.2 Get Harvest History

```http
GET /v1/harvests?facility_id=FAC-OCEAN-FARMS-01&start_date=2025-01-01
```

---

## Analytics & Reporting

### 9.1 Production Analytics

```http
GET /v1/analytics/production?facility_id=FAC-OCEAN-FARMS-01&period=MONTHLY
```

**Response**:
```json
{
  "status": "success",
  "data": {
    "period": "2025-01",
    "total_batches": 15,
    "total_biomass_kg": 450,
    "average_productivity_g_per_L_per_day": 0.38,
    "average_protein_content": 64.5,
    "sustainability_metrics": {
      "co2_sequestered_kg": 810,
      "water_recycling_percent": 98
    }
  }
}
```

### 9.2 Financial Reporting

```http
GET /v1/analytics/financial?facility_id=FAC-OCEAN-FARMS-01&year=2025
```

---

## Error Handling

### 10.1 Error Response Format

```json
{
  "status": "error",
  "error": {
    "code": "INVALID_PARAMETER",
    "message": "Temperature value must be between 15-40°C",
    "field": "environmental_parameters.temperature",
    "details": "Received value: 50.0"
  },
  "meta": {
    "request_id": "req-123e4567",
    "timestamp": "2025-01-15T10:30:00Z"
  }
}
```

### 10.2 HTTP Status Codes

| Code | Meaning | Example |
|------|---------|---------|
| 200 | Success | GET request successful |
| 201 | Created | POST created new resource |
| 400 | Bad Request | Invalid JSON or parameters |
| 401 | Unauthorized | Missing or invalid token |
| 403 | Forbidden | Insufficient permissions |
| 404 | Not Found | Resource doesn't exist |
| 422 | Unprocessable Entity | Validation error |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Server Error | Server error |
| 503 | Service Unavailable | Maintenance mode |

---

## Rate Limiting

### 11.1 Limits

| Tier | Requests/minute | Requests/day |
|------|-----------------|--------------|
| Free | 60 | 10,000 |
| Basic | 600 | 100,000 |
| Pro | 6,000 | 1,000,000 |
| Enterprise | Custom | Custom |

### 11.2 Rate Limit Headers

```http
X-RateLimit-Limit: 600
X-RateLimit-Remaining: 587
X-RateLimit-Reset: 1642249200
```

---

## Webhooks

### 12.1 Configure Webhook

```http
POST /v1/webhooks
Content-Type: application/json

{
  "url": "https://your-server.com/webhooks/algae",
  "events": [
    "cultivation.harvest_ready",
    "cultivation.contamination_detected",
    "measurement.threshold_exceeded"
  ],
  "secret": "whsec_your_webhook_secret"
}
```

### 12.2 Webhook Payload

```json
{
  "event": "cultivation.harvest_ready",
  "timestamp": "2025-01-18T10:00:00Z",
  "data": {
    "cultivation_id": "CULT-2025-ALG-001",
    "current_density": 4.6,
    "target_density": 4.5,
    "days_in_cultivation": 18
  },
  "signature": "sha256=..."
}
```

---

**弘益人間 (Hongik Ingan)** · Benefit All Humanity

© 2025 WIA Standards - MIT License
