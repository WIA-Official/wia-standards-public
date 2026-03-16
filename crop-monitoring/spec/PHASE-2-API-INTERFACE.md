# WIA Crop Monitoring API Interface Standard
## Phase 2 Specification

---

**Version**: 1.0.0
**Status**: Complete
**Date**: 2025-01
**Standard ID**: WIA-AGRI-006
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #84CC16 (Lime - AGRI)

---

## Table of Contents

1. [Overview](#overview)
2. [API Architecture](#api-architecture)
3. [Authentication](#authentication)
4. [Core Endpoints](#core-endpoints)
5. [Request/Response Formats](#requestresponse-formats)
6. [Error Handling](#error-handling)
7. [Rate Limiting](#rate-limiting)
8. [SDK Examples](#sdk-examples)
9. [Version History](#version-history)

---

## Overview

### 1.1 Purpose

The WIA Crop Monitoring API Interface Standard defines RESTful API endpoints, WebSocket streaming protocols, and SDK interfaces for crop monitoring systems. This enables seamless integration of IoT sensors, AI analysis engines, and agricultural management platforms.

**Core Capabilities**:
- Real-time crop data submission and retrieval
- AI-powered disease detection API
- Yield prediction and forecasting
- Weather integration and alerts
- Marketplace and insurance integrations

### 1.2 API Architecture

```
┌─────────────────┐
│  Client Apps   │
│ (Farm Mgmt UI) │
└────────┬────────┘
         │
         ▼
┌─────────────────────────────┐
│   WIA Crop API Gateway      │
│   - Authentication          │
│   - Rate Limiting           │
│   - Load Balancing          │
└────────┬────────────────────┘
         │
    ┌────┴────┬─────────┬──────────┐
    ▼         ▼         ▼          ▼
┌────────┐ ┌──────┐ ┌────────┐ ┌────────┐
│ Crop   │ │ AI   │ │ Weather│ │ Market │
│ Data   │ │ Model│ │ API    │ │ API    │
└────────┘ └──────┘ └────────┘ └────────┘
```

### 1.3 Base URL

```
Production:  https://api.crop-monitoring.wiastandards.com/v1
Staging:     https://staging-api.crop-monitoring.wiastandards.com/v1
Development: https://dev-api.crop-monitoring.wiastandards.com/v1
```

---

## Authentication

### 2.1 API Key Authentication

All API requests require an API key in the request header:

```http
GET /crops/CROP-2025-001
Host: api.crop-monitoring.wiastandards.com
Authorization: Bearer wia_api_key_1234567890abcdef
```

### 2.2 OAuth 2.0 (Enterprise)

For enterprise integrations, OAuth 2.0 is supported:

```http
POST /oauth/token
Content-Type: application/json

{
  "grant_type": "client_credentials",
  "client_id": "your_client_id",
  "client_secret": "your_client_secret",
  "scope": "crop:read crop:write"
}
```

Response:
```json
{
  "access_token": "eyJhbGciOiJIUzI1NiIs...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "crop:read crop:write"
}
```

### 2.3 Scopes

| Scope | Description |
|-------|-------------|
| `crop:read` | Read crop data |
| `crop:write` | Submit crop data |
| `ai:detect` | Use AI detection endpoints |
| `forecast:read` | Access yield predictions |
| `admin` | Full administrative access |

---

## Core Endpoints

### 3.1 Crop Data Management

#### POST /crops
Submit new crop monitoring data

**Request:**
```http
POST /crops
Authorization: Bearer {api_key}
Content-Type: application/json

{
  "cropId": "CROP-2025-001",
  "farmId": "FARM-KR-12345",
  "timestamp": "2025-06-15T10:30:00Z",
  "location": {
    "gps": {"latitude": 37.5665, "longitude": 126.9780}
  },
  "cropType": "rice",
  "growthStage": {
    "code": "BBCH-30",
    "description": "Vegetative stage"
  },
  "measurements": {
    "plantHeight": {"value": 45.5, "unit": "cm"},
    "leafAreaIndex": {"value": 3.8, "unit": "m²/m²"},
    "chlorophyllSPAD": {"value": 42.0, "unit": "SPAD"}
  }
}
```

**Response (201 Created):**
```json
{
  "status": "success",
  "data": {
    "cropId": "CROP-2025-001",
    "recordId": "REC-20250615-001",
    "timestamp": "2025-06-15T10:30:00Z",
    "validationStatus": "passed"
  }
}
```

#### GET /crops/{cropId}
Retrieve crop data by ID

**Request:**
```http
GET /crops/CROP-2025-001
Authorization: Bearer {api_key}
```

**Response (200 OK):**
```json
{
  "status": "success",
  "data": {
    "cropId": "CROP-2025-001",
    "farmId": "FARM-KR-12345",
    "latestMeasurement": "2025-06-15T10:30:00Z",
    "currentStage": "BBCH-30",
    "healthStatus": "healthy",
    "records": [
      {
        "timestamp": "2025-06-15T10:30:00Z",
        "measurements": {...}
      }
    ]
  }
}
```

#### GET /crops
List all crops for a farm

**Request:**
```http
GET /crops?farmId=FARM-KR-12345&limit=50&offset=0
Authorization: Bearer {api_key}
```

**Response (200 OK):**
```json
{
  "status": "success",
  "data": {
    "total": 150,
    "limit": 50,
    "offset": 0,
    "crops": [
      {
        "cropId": "CROP-2025-001",
        "cropType": "rice",
        "currentStage": "BBCH-30",
        "healthStatus": "healthy"
      }
    ]
  }
}
```

#### PATCH /crops/{cropId}
Update crop information

**Request:**
```http
PATCH /crops/CROP-2025-001
Authorization: Bearer {api_key}
Content-Type: application/json

{
  "growthStage": {
    "code": "BBCH-51",
    "description": "Flowering"
  }
}
```

#### DELETE /crops/{cropId}
Delete crop data (admin only)

---

### 3.2 AI Disease Detection

#### POST /ai/detect-disease
Analyze crop image for diseases

**Request:**
```http
POST /ai/detect-disease
Authorization: Bearer {api_key}
Content-Type: multipart/form-data

{
  "cropId": "CROP-2025-001",
  "image": [binary image data],
  "cropType": "tomato"
}
```

**Response (200 OK):**
```json
{
  "status": "success",
  "data": {
    "detections": [
      {
        "disease": "Late Blight (Phytophthora infestans)",
        "confidence": 0.87,
        "severity": "medium",
        "affectedArea": 15.5,
        "recommendations": [
          "Apply copper-based fungicide within 24 hours",
          "Remove severely affected leaves",
          "Improve air circulation"
        ]
      }
    ],
    "modelVersion": "WIA-CropVision-v2.1",
    "processingTime": 1.23
  }
}
```

#### POST /ai/detect-pest
Identify pest infestations

**Response (200 OK):**
```json
{
  "status": "success",
  "data": {
    "detections": [
      {
        "pest": "Fall Armyworm (Spodoptera frugiperda)",
        "confidence": 0.92,
        "severity": "high",
        "estimatedCount": 25,
        "recommendations": [
          "Apply Bacillus thuringiensis (Bt) spray",
          "Scout fields every 2-3 days",
          "Consider pheromone traps"
        ]
      }
    ]
  }
}
```

---

### 3.3 Yield Prediction

#### GET /forecast/yield/{cropId}
Get yield prediction for a crop

**Request:**
```http
GET /forecast/yield/CROP-2025-001
Authorization: Bearer {api_key}
```

**Response (200 OK):**
```json
{
  "status": "success",
  "data": {
    "cropId": "CROP-2025-001",
    "prediction": {
      "yieldEstimate": {
        "value": 5500,
        "unit": "kg/ha",
        "confidence": 0.82
      },
      "harvestDate": "2025-09-01",
      "qualityScore": 85,
      "factors": {
        "weather": "favorable",
        "soilHealth": "good",
        "diseaseRisk": "low"
      }
    },
    "modelVersion": "YieldPredictor-v3.0"
  }
}
```

#### POST /forecast/scenario
Run "what-if" scenario analysis

**Request:**
```http
POST /forecast/scenario
Authorization: Bearer {api_key}
Content-Type: application/json

{
  "cropId": "CROP-2025-001",
  "scenario": {
    "weatherOverride": {
      "rainfall": "+20%",
      "temperature": "+2°C"
    },
    "interventions": [
      "extra_fertilizer",
      "pest_control"
    ]
  }
}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "baselineYield": 5500,
    "scenarioYield": 6200,
    "difference": "+12.7%",
    "confidence": 0.75
  }
}
```

---

### 3.4 Weather Integration

#### GET /weather/forecast/{location}
Get weather forecast for field location

**Request:**
```http
GET /weather/forecast/37.5665,126.9780?days=7
Authorization: Bearer {api_key}
```

**Response (200 OK):**
```json
{
  "status": "success",
  "data": {
    "location": {"latitude": 37.5665, "longitude": 126.9780},
    "forecast": [
      {
        "date": "2025-06-16",
        "temperature": {"min": 18, "max": 28, "unit": "°C"},
        "humidity": 65,
        "precipitation": 5.2,
        "windSpeed": 12,
        "cropImpact": {
          "diseaseRisk": "medium",
          "wateringNeeded": false,
          "alerts": ["High humidity - monitor for fungal diseases"]
        }
      }
    ]
  }
}
```

#### GET /weather/alerts/{cropId}
Get weather alerts for specific crop

---

### 3.5 Marketplace Integration

#### GET /marketplace/prices
Get current market prices

**Request:**
```http
GET /marketplace/prices?cropType=rice&region=KR
Authorization: Bearer {api_key}
```

**Response (200 OK):**
```json
{
  "status": "success",
  "data": {
    "cropType": "rice",
    "region": "KR",
    "prices": [
      {
        "grade": "premium",
        "pricePerKg": 2.50,
        "currency": "USD",
        "buyers": 8,
        "trend": "stable"
      }
    ]
  }
}
```

---

## Request/Response Formats

### 4.1 Common Headers

**Request Headers:**
```http
Authorization: Bearer {api_key}
Content-Type: application/json
Accept: application/json
X-Request-ID: uuid-v4
```

**Response Headers:**
```http
Content-Type: application/json
X-Request-ID: uuid-v4
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 999
X-RateLimit-Reset: 1625097600
```

### 4.2 Standard Response Format

```json
{
  "status": "success" | "error",
  "data": {...},
  "error": {
    "code": "string",
    "message": "string",
    "details": {}
  },
  "metadata": {
    "requestId": "uuid",
    "timestamp": "ISO8601",
    "version": "v1"
  }
}
```

---

## Error Handling

### 5.1 HTTP Status Codes

| Code | Meaning | Description |
|------|---------|-------------|
| 200 | OK | Request successful |
| 201 | Created | Resource created |
| 400 | Bad Request | Invalid input data |
| 401 | Unauthorized | Missing or invalid API key |
| 403 | Forbidden | Insufficient permissions |
| 404 | Not Found | Resource not found |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Server Error | Server error |

### 5.2 Error Response Format

```json
{
  "status": "error",
  "error": {
    "code": "INVALID_CROP_DATA",
    "message": "Invalid chlorophyll SPAD value",
    "details": {
      "field": "measurements.chlorophyllSPAD.value",
      "value": 75,
      "constraint": "Must be between 0 and 60"
    }
  },
  "metadata": {
    "requestId": "req-123456",
    "timestamp": "2025-06-15T10:30:00Z"
  }
}
```

### 5.3 Error Codes

| Code | Description |
|------|-------------|
| `INVALID_API_KEY` | API key is missing or invalid |
| `INVALID_CROP_DATA` | Crop data validation failed |
| `CROP_NOT_FOUND` | Crop ID not found |
| `AI_MODEL_UNAVAILABLE` | AI detection service unavailable |
| `RATE_LIMIT_EXCEEDED` | Too many requests |

---

## Rate Limiting

### 6.1 Rate Limits

| Tier | Requests/Hour | Requests/Day |
|------|---------------|--------------|
| Free | 100 | 1,000 |
| Basic | 1,000 | 10,000 |
| Pro | 10,000 | 100,000 |
| Enterprise | Unlimited | Unlimited |

### 6.2 Rate Limit Headers

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 950
X-RateLimit-Reset: 1625097600
```

---

## SDK Examples

### 7.1 JavaScript/TypeScript SDK

```typescript
import { WIACropMonitoring } from '@wia/crop-monitoring';

const client = new WIACropMonitoring({
  apiKey: 'wia_api_key_1234567890abcdef',
  environment: 'production'
});

// Submit crop data
const result = await client.crops.create({
  cropId: 'CROP-2025-001',
  farmId: 'FARM-KR-12345',
  cropType: 'rice',
  measurements: {
    plantHeight: { value: 45.5, unit: 'cm' },
    chlorophyllSPAD: { value: 42.0, unit: 'SPAD' }
  }
});

// AI disease detection
const detection = await client.ai.detectDisease({
  cropId: 'CROP-2025-001',
  image: imageBuffer,
  cropType: 'tomato'
});

console.log('Detected:', detection.detections);
```

### 7.2 Python SDK

```python
from wia_crop_monitoring import CropMonitoringClient

client = CropMonitoringClient(api_key='wia_api_key_1234567890abcdef')

# Submit crop data
result = client.crops.create(
    crop_id='CROP-2025-001',
    farm_id='FARM-KR-12345',
    crop_type='rice',
    measurements={
        'plant_height': {'value': 45.5, 'unit': 'cm'},
        'chlorophyll_spad': {'value': 42.0, 'unit': 'SPAD'}
    }
)

# Yield prediction
forecast = client.forecast.yield_estimate('CROP-2025-001')
print(f"Estimated yield: {forecast['yieldEstimate']['value']} kg/ha")
```

### 7.3 cURL Example

```bash
# Submit crop data
curl -X POST https://api.crop-monitoring.wiastandards.com/v1/crops \
  -H "Authorization: Bearer wia_api_key_1234567890abcdef" \
  -H "Content-Type: application/json" \
  -d '{
    "cropId": "CROP-2025-001",
    "farmId": "FARM-KR-12345",
    "cropType": "rice",
    "measurements": {
      "plantHeight": {"value": 45.5, "unit": "cm"}
    }
  }'
```

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01 | Initial API specification |

---

**Philosophy**: 弘益人間 (Benefit All Humanity)
**License**: MIT
**Contact**: api-support@wiastandards.com
**Documentation**: https://docs.crop-monitoring.wiastandards.com
