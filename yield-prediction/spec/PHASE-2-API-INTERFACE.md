# WIA-AGRI-008: Yield Prediction Standard
## PHASE 2: API Interface Specification

**Version:** 1.0
**Status:** Active
**Category:** AGRI
**Last Updated:** 2025-12-26

---

## 1. Overview

This specification defines the RESTful API interfaces for yield prediction systems, enabling seamless integration between agricultural data providers, prediction models, and consumer applications.

## 2. Base API Structure

### 2.1 Endpoint Base URL

```
Production: https://api.wia.org/agri/v1
Staging: https://api-staging.wia.org/agri/v1
Development: https://api-dev.wia.org/agri/v1
```

### 2.2 Authentication

**OAuth 2.0 Bearer Token:**

```http
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
```

**API Key (Alternative):**

```http
X-API-Key: wia_live_sk_1234567890abcdef
```

## 3. Core Endpoints

### 3.1 Submit Yield Data

**POST** `/yield/submit`

Submit historical yield data for model training.

**Request Body:**

```json
{
  "farmId": "KR-FARM-2025-001",
  "crop": "rice",
  "harvestYear": 2025,
  "location": {
    "province": "chungnam",
    "latitude": 36.5184,
    "longitude": 127.2158
  },
  "yield": {
    "amount": 5200,
    "unit": "kg/ha",
    "area": 2.5
  },
  "inputs": {
    "fertilizer": {
      "nitrogen": 90,
      "phosphorus": 45,
      "potassium": 57
    },
    "irrigation": 800,
    "pesticides": 15
  },
  "weather": {
    "avgTemp": 22.5,
    "rainfall": 850,
    "sunshine": 1800
  }
}
```

**Response (201 Created):**

```json
{
  "status": "success",
  "data": {
    "recordId": "rec_2025_abc123",
    "farmId": "KR-FARM-2025-001",
    "validationStatus": "passed",
    "qualityScore": 95,
    "createdAt": "2025-12-26T10:30:00Z",
    "verificationUrl": "https://verify.wia.org/yield/rec_2025_abc123"
  }
}
```

**Error Response (400 Bad Request):**

```json
{
  "status": "error",
  "error": {
    "code": "VALIDATION_FAILED",
    "message": "Yield amount exceeds expected range for crop type",
    "field": "yield.amount",
    "expected": "2000-12000 kg/ha",
    "received": 15000
  }
}
```

### 3.2 Get Yield Prediction

**GET** `/yield/predict`

Get yield prediction for specified parameters.

**Query Parameters:**

```
crop=rice
province=chungnam
year=2026
farmId=KR-FARM-2025-001 (optional)
algorithm=ensemble (optional: ensemble|randomforest|lstm)
```

**Request Example:**

```http
GET /api/v1/yield/predict?crop=rice&province=chungnam&year=2026
Authorization: Bearer {token}
```

**Response (200 OK):**

```json
{
  "status": "success",
  "data": {
    "prediction": {
      "amount": 5350,
      "unit": "kg/ha",
      "confidence": 0.92,
      "range": {
        "min": 5100,
        "max": 5600
      }
    },
    "factors": {
      "primary": ["favorable_rainfall", "optimal_temperature"],
      "risks": ["mid_season_drought_potential"],
      "weights": {
        "weather": 0.45,
        "soil": 0.25,
        "inputs": 0.20,
        "historical": 0.10
      }
    },
    "model": {
      "algorithm": "ensemble",
      "accuracy": 0.89,
      "trainingSize": 1520,
      "lastUpdated": "2025-12-20T00:00:00Z"
    },
    "timestamp": "2025-12-26T10:30:00Z",
    "validUntil": "2026-10-31T00:00:00Z"
  }
}
```

### 3.3 Batch Predictions

**POST** `/yield/predict/batch`

Get predictions for multiple farms or scenarios.

**Request Body:**

```json
{
  "predictions": [
    {
      "farmId": "KR-FARM-001",
      "crop": "rice",
      "year": 2026,
      "scenario": "baseline"
    },
    {
      "farmId": "KR-FARM-001",
      "crop": "rice",
      "year": 2026,
      "scenario": "high_fertilizer"
    },
    {
      "farmId": "KR-FARM-002",
      "crop": "wheat",
      "year": 2026,
      "scenario": "baseline"
    }
  ]
}
```

**Response (200 OK):**

```json
{
  "status": "success",
  "data": {
    "results": [
      {
        "farmId": "KR-FARM-001",
        "scenario": "baseline",
        "prediction": 5200,
        "confidence": 0.91
      },
      {
        "farmId": "KR-FARM-001",
        "scenario": "high_fertilizer",
        "prediction": 5450,
        "confidence": 0.88
      },
      {
        "farmId": "KR-FARM-002",
        "scenario": "baseline",
        "prediction": 4300,
        "confidence": 0.85
      }
    ],
    "batchId": "batch_2025_xyz789",
    "processedAt": "2025-12-26T10:30:00Z"
  }
}
```

### 3.4 Regional Aggregate Prediction

**GET** `/yield/predict/regional`

Get aggregated yield predictions for a region.

**Query Parameters:**

```
province=chungnam
crop=rice
year=2026
level=province|city|district
```

**Response (200 OK):**

```json
{
  "status": "success",
  "data": {
    "region": {
      "province": "chungnam",
      "name": "충청남도"
    },
    "crop": "rice",
    "year": 2026,
    "prediction": {
      "totalProduction": 387500,
      "unit": "tons",
      "avgYield": 5500,
      "yieldUnit": "kg/ha",
      "cultivatedArea": 70455,
      "areaUnit": "ha"
    },
    "confidence": 0.87,
    "historicalComparison": {
      "5yearAvg": 382000,
      "variance": "+1.4%"
    },
    "breakdown": [
      {
        "city": "당진시",
        "production": 45200,
        "yield": 5600
      },
      {
        "city": "서산시",
        "production": 38900,
        "yield": 5450
      }
    ]
  }
}
```

## 4. Data Management Endpoints

### 4.1 List Historical Data

**GET** `/yield/history`

Retrieve historical yield records.

**Query Parameters:**

```
farmId=KR-FARM-001 (optional)
crop=rice (optional)
startYear=2020 (optional)
endYear=2025 (optional)
limit=50 (default: 100, max: 1000)
offset=0 (default: 0)
```

**Response (200 OK):**

```json
{
  "status": "success",
  "data": {
    "records": [
      {
        "recordId": "rec_2025_001",
        "farmId": "KR-FARM-001",
        "crop": "rice",
        "year": 2025,
        "yield": 5200,
        "createdAt": "2025-11-15T10:00:00Z"
      }
    ],
    "pagination": {
      "total": 243,
      "limit": 50,
      "offset": 0,
      "hasMore": true
    }
  }
}
```

### 4.2 Update Yield Record

**PATCH** `/yield/{recordId}`

Update existing yield record.

**Request Body:**

```json
{
  "yield": {
    "amount": 5250
  },
  "metadata": {
    "verificationStatus": "verified"
  }
}
```

**Response (200 OK):**

```json
{
  "status": "success",
  "data": {
    "recordId": "rec_2025_001",
    "updated": true,
    "updatedFields": ["yield.amount", "metadata.verificationStatus"],
    "updatedAt": "2025-12-26T11:00:00Z"
  }
}
```

### 4.3 Delete Yield Record

**DELETE** `/yield/{recordId}`

Delete a yield record (with proper authorization).

**Response (200 OK):**

```json
{
  "status": "success",
  "data": {
    "recordId": "rec_2025_001",
    "deleted": true,
    "deletedAt": "2025-12-26T11:30:00Z"
  }
}
```

## 5. Model Management Endpoints

### 5.1 Get Model Status

**GET** `/models/{cropType}/status`

Get current status of prediction model.

**Response (200 OK):**

```json
{
  "status": "success",
  "data": {
    "crop": "rice",
    "model": {
      "version": "v2.3.1",
      "algorithm": "ensemble",
      "accuracy": {
        "rmse": 245.3,
        "mae": 189.7,
        "r2": 0.89
      },
      "trainingData": {
        "recordCount": 1520,
        "dateRange": "2015-2025",
        "lastUpdate": "2025-12-20T00:00:00Z"
      },
      "status": "active",
      "nextUpdate": "2026-01-15T00:00:00Z"
    }
  }
}
```

### 5.2 Request Model Retraining

**POST** `/models/{cropType}/retrain`

Request model retraining (admin only).

**Request Body:**

```json
{
  "reason": "new_season_data_available",
  "priority": "normal",
  "includeData": {
    "startYear": 2015,
    "endYear": 2025
  }
}
```

**Response (202 Accepted):**

```json
{
  "status": "accepted",
  "data": {
    "jobId": "retrain_job_001",
    "status": "queued",
    "estimatedCompletion": "2025-12-27T10:00:00Z",
    "statusUrl": "/models/rice/retrain/status/retrain_job_001"
  }
}
```

## 6. Integration Endpoints

### 6.1 Market Integration

**POST** `/integrations/market/forecast`

Get market price forecast based on yield predictions.

**Request Body:**

```json
{
  "crop": "rice",
  "region": "chungnam",
  "year": 2026,
  "currentPrice": 180000,
  "currency": "KRW",
  "unit": "80kg"
}
```

**Response (200 OK):**

```json
{
  "status": "success",
  "data": {
    "priceF forecast": {
      "3months": 175000,
      "6months": 172000,
      "12months": 178000
    },
    "factors": {
      "yieldImpact": "-2.8%",
      "demandTrend": "stable",
      "imports": "low"
    },
    "recommendation": "Prices expected to soften due to higher yield. Consider early selling or forward contracts."
  }
}
```

### 6.2 Insurance Integration

**POST** `/integrations/insurance/risk-assessment`

Assess crop insurance risk based on predictions.

**Request Body:**

```json
{
  "farmId": "KR-FARM-001",
  "crop": "rice",
  "year": 2026,
  "insuredYield": 5000,
  "coverageLevel": 0.85
}
```

**Response (200 OK):**

```json
{
  "status": "success",
  "data": {
    "riskScore": 23,
    "riskLevel": "low",
    "predictedYield": 5350,
    "claimProbability": 0.08,
    "recommendedPremium": {
      "amount": 45000,
      "currency": "KRW",
      "unit": "per_ha"
    },
    "factors": {
      "weather": "favorable",
      "soilQuality": "good",
      "historicalPerformance": "above_average"
    }
  }
}
```

## 7. Webhook Notifications

### 7.1 Register Webhook

**POST** `/webhooks/register`

Register a webhook for event notifications.

**Request Body:**

```json
{
  "url": "https://your-server.com/webhooks/wia-yield",
  "events": [
    "yield.submitted",
    "prediction.ready",
    "model.updated"
  ],
  "secret": "whsec_abc123xyz789"
}
```

**Response (201 Created):**

```json
{
  "status": "success",
  "data": {
    "webhookId": "wh_001",
    "url": "https://your-server.com/webhooks/wia-yield",
    "events": ["yield.submitted", "prediction.ready", "model.updated"],
    "active": true,
    "createdAt": "2025-12-26T10:30:00Z"
  }
}
```

### 7.2 Webhook Payload Example

```json
{
  "event": "prediction.ready",
  "timestamp": "2025-12-26T10:30:00Z",
  "data": {
    "farmId": "KR-FARM-001",
    "crop": "rice",
    "year": 2026,
    "prediction": 5350,
    "confidence": 0.92,
    "predictionUrl": "https://api.wia.org/agri/v1/yield/predict?farmId=KR-FARM-001"
  },
  "signature": "sha256=abcd1234..."
}
```

## 8. Rate Limiting

### 8.1 Rate Limits

| Tier | Requests/Minute | Requests/Day |
|------|-----------------|--------------|
| Free | 60 | 1,000 |
| Standard | 300 | 10,000 |
| Professional | 1,000 | 100,000 |
| Enterprise | Custom | Custom |

### 8.2 Rate Limit Headers

```http
X-RateLimit-Limit: 300
X-RateLimit-Remaining: 245
X-RateLimit-Reset: 1704182400
```

## 9. Error Codes

| Code | Status | Description |
|------|--------|-------------|
| `VALIDATION_FAILED` | 400 | Request data validation failed |
| `UNAUTHORIZED` | 401 | Invalid or missing authentication |
| `FORBIDDEN` | 403 | Insufficient permissions |
| `NOT_FOUND` | 404 | Resource not found |
| `RATE_LIMIT_EXCEEDED` | 429 | Too many requests |
| `SERVER_ERROR` | 500 | Internal server error |
| `MODEL_UNAVAILABLE` | 503 | Prediction model temporarily unavailable |

## 10. SDK Examples

### 10.1 Python SDK

```python
from wia_agri import YieldPredictionClient

client = YieldPredictionClient(api_key="wia_live_sk_...")

# Submit yield data
result = client.yield_data.submit({
    "farmId": "KR-FARM-001",
    "crop": "rice",
    "harvestYear": 2025,
    "yield": {"amount": 5200, "unit": "kg/ha"}
})

# Get prediction
prediction = client.predictions.get(
    crop="rice",
    province="chungnam",
    year=2026
)
print(f"Predicted yield: {prediction.amount} kg/ha")
```

### 10.2 JavaScript SDK

```javascript
import { WIAAgriClient } from '@wia/agri-sdk';

const client = new WIAAgriClient({
  apiKey: 'wia_live_sk_...'
});

// Get prediction
const prediction = await client.predictions.get({
  crop: 'rice',
  province: 'chungnam',
  year: 2026
});

console.log(`Predicted yield: ${prediction.amount} kg/ha`);
console.log(`Confidence: ${prediction.confidence * 100}%`);
```

---

**License:** CC BY 4.0
**Maintained by:** WIA Technical Committee
**Contact:** api@wia.org
