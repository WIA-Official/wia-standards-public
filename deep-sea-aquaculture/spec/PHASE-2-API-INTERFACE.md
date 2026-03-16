# WIA-AGRI-023: Deep Sea Aquaculture Standard
## Phase 2: API Interface Specification

**Version:** 1.0
**Status:** Complete
**Last Updated:** 2025-12-26
**Standard ID:** WIA-AGRI-023

---

## 1. Overview

This specification defines the RESTful API interfaces for deep-sea aquaculture systems, enabling standardized access to farm data, control systems, analytics, and integrations.

### 1.1 Base URL Structure

```
https://api.wia-aquaculture.org/v1
```

### 1.2 Authentication

All API requests require authentication using OAuth 2.0 or API keys.

```http
Authorization: Bearer {access_token}
```

Or:

```http
X-API-Key: {api_key}
```

---

## 2. Farm Management Endpoints

### 2.1 List All Farms

```http
GET /api/v1/farms
```

**Query Parameters:**
- `country` (optional): Filter by country code
- `operator` (optional): Filter by operator ID
- `status` (optional): active, inactive, maintenance
- `page` (optional): Page number (default: 1)
- `limit` (optional): Items per page (default: 20, max: 100)

**Response:**

```json
{
  "status": "success",
  "data": {
    "farms": [
      {
        "farmId": "DSA-FARM-USA-001",
        "name": "Pacific Deep Aquaculture Farm",
        "operator": "Oceanic Farms Inc.",
        "location": {
          "latitude": 36.5,
          "longitude": -127.2,
          "country": "USA"
        },
        "status": "active",
        "species": ["Atlantic Salmon"],
        "totalBiomass": 125000,
        "lastUpdate": "2025-12-26T10:30:00Z"
      }
    ],
    "pagination": {
      "page": 1,
      "limit": 20,
      "total": 45,
      "pages": 3
    }
  }
}
```

### 2.2 Get Farm Details

```http
GET /api/v1/farms/{farmId}
```

**Response:**

```json
{
  "status": "success",
  "data": {
    "farmId": "DSA-FARM-USA-001",
    "farmName": "Pacific Deep Aquaculture Farm",
    "operator": {
      "organizationId": "ORG-123456",
      "name": "Oceanic Farms Inc.",
      "license": "AQUA-LIC-2025-001"
    },
    "location": {
      "gps": {
        "latitude": 36.5,
        "longitude": -127.2
      },
      "region": "Pacific Northwest"
    },
    "infrastructure": {
      "totalCages": 10,
      "depth": {"min": 30, "max": 60, "unit": "meters"},
      "totalVolume": 50000
    },
    "status": "active",
    "certifications": ["ASC", "BAP"],
    "created": "2025-01-15T00:00:00Z"
  }
}
```

### 2.3 Register New Farm

```http
POST /api/v1/farms
```

**Request Body:**

```json
{
  "farmName": "New Deep Sea Farm",
  "operatorId": "ORG-123456",
  "location": {
    "latitude": 35.5,
    "longitude": -125.3
  },
  "infrastructure": {
    "totalCages": 5,
    "depth": {"min": 40, "max": 70}
  },
  "species": ["Atlantic Salmon", "Yellowtail"]
}
```

**Response:**

```json
{
  "status": "success",
  "data": {
    "farmId": "DSA-FARM-USA-046",
    "message": "Farm registered successfully",
    "apiKey": "wia_live_sk_abcdef123456"
  }
}
```

---

## 3. Sensor Data Endpoints

### 3.1 Get Real-time Sensor Data

```http
GET /api/v1/farms/{farmId}/sensors
```

**Query Parameters:**
- `cageId` (optional): Specific cage
- `sensorType` (optional): temperature, salinity, oxygen, pH, current
- `startTime` (optional): ISO 8601 timestamp
- `endTime` (optional): ISO 8601 timestamp

**Response:**

```json
{
  "status": "success",
  "data": {
    "farmId": "DSA-FARM-USA-001",
    "timestamp": "2025-12-26T10:30:00Z",
    "sensors": [
      {
        "sensorId": "ENV-SENSOR-001",
        "cageId": "CAGE-01",
        "depth": 45,
        "measurements": {
          "waterTemperature": {"value": 18.5, "unit": "celsius", "status": "normal"},
          "salinity": {"value": 35.2, "unit": "ppt", "status": "normal"},
          "dissolvedOxygen": {"value": 7.8, "unit": "mg/L", "status": "normal"},
          "pH": {"value": 8.1, "status": "normal"}
        }
      }
    ],
    "count": 1
  }
}
```

### 3.2 Submit Sensor Data

```http
POST /api/v1/farms/{farmId}/sensors
```

**Request Body:**

```json
{
  "sensorId": "ENV-SENSOR-002",
  "cageId": "CAGE-02",
  "timestamp": "2025-12-26T10:35:00Z",
  "measurements": {
    "waterTemperature": {"value": 18.7, "unit": "celsius"},
    "salinity": {"value": 35.1, "unit": "ppt"},
    "dissolvedOxygen": {"value": 7.9, "unit": "mg/L"},
    "pH": {"value": 8.0}
  }
}
```

**Response:**

```json
{
  "status": "success",
  "data": {
    "recordId": "REC-20251226-001234",
    "timestamp": "2025-12-26T10:35:00Z"
  }
}
```

---

## 4. Fish Health Endpoints

### 4.1 Get Fish Health Report

```http
GET /api/v1/farms/{farmId}/fish-health
```

**Query Parameters:**
- `cageId` (optional): Specific cage
- `reportType` (optional): summary, detailed
- `period` (optional): daily, weekly, monthly

**Response:**

```json
{
  "status": "success",
  "data": {
    "farmId": "DSA-FARM-USA-001",
    "timestamp": "2025-12-26T10:30:00Z",
    "summary": {
      "totalFish": 50000,
      "totalBiomass": 125000,
      "averageWeight": 2.5,
      "healthScore": 94
    },
    "cages": [
      {
        "cageId": "CAGE-01",
        "species": "Atlantic Salmon",
        "population": 5000,
        "biomass": 12500,
        "mortalityRate": 0.1,
        "growthRate": 3.2,
        "feedConversionRatio": 1.15,
        "diseases": [],
        "healthScore": 95
      }
    ]
  }
}
```

### 4.2 Report Disease Outbreak

```http
POST /api/v1/farms/{farmId}/fish-health/disease
```

**Request Body:**

```json
{
  "cageId": "CAGE-03",
  "disease": {
    "type": "sea-lice",
    "severity": "moderate",
    "affectedCount": 150,
    "detectedDate": "2025-12-26",
    "symptoms": ["skin lesions", "reduced feeding"]
  },
  "treatment": {
    "plan": "freshwater bath",
    "scheduledDate": "2025-12-27",
    "expectedDuration": 3
  }
}
```

**Response:**

```json
{
  "status": "success",
  "data": {
    "incidentId": "DISEASE-2025-001",
    "alerts": [
      "Veterinary inspection recommended within 24 hours",
      "Regulatory reporting required"
    ]
  }
}
```

---

## 5. Feeding System Endpoints

### 5.1 Get Feeding Schedule

```http
GET /api/v1/farms/{farmId}/feeding/schedule
```

**Response:**

```json
{
  "status": "success",
  "data": {
    "farmId": "DSA-FARM-USA-001",
    "schedules": [
      {
        "cageId": "CAGE-01",
        "mode": "automatic",
        "frequency": 4,
        "feedingTimes": ["06:00", "10:00", "14:00", "18:00"],
        "dailyAmount": 312.5,
        "unit": "kg",
        "feedType": "pellets-9mm"
      }
    ]
  }
}
```

### 5.2 Trigger Feeding

```http
POST /api/v1/farms/{farmId}/feeding
```

**Request Body:**

```json
{
  "cageId": "CAGE-01",
  "feedAmount": 50,
  "feedType": "pellets-9mm",
  "duration": 15,
  "override": false
}
```

**Response:**

```json
{
  "status": "success",
  "data": {
    "eventId": "FEED-20251226-001",
    "cageId": "CAGE-01",
    "startTime": "2025-12-26T10:00:00Z",
    "estimatedEndTime": "2025-12-26T10:15:00Z",
    "feedAmount": 50,
    "status": "in-progress"
  }
}
```

### 5.3 Get Feeding History

```http
GET /api/v1/farms/{farmId}/feeding/history
```

**Query Parameters:**
- `cageId` (optional)
- `startDate` (optional)
- `endDate` (optional)
- `limit` (optional)

**Response:**

```json
{
  "status": "success",
  "data": {
    "feedingEvents": [
      {
        "eventId": "FEED-20251226-001",
        "cageId": "CAGE-01",
        "timestamp": "2025-12-26T10:00:00Z",
        "feedAmount": 50,
        "duration": 15,
        "uneatenFeed": 0.6,
        "efficiency": 98.8
      }
    ],
    "summary": {
      "totalFeedDispensed": 625,
      "averageEfficiency": 98.5,
      "period": "2025-12-20 to 2025-12-26"
    }
  }
}
```

---

## 6. Environmental Impact Endpoints

### 6.1 Get Environmental Metrics

```http
GET /api/v1/farms/{farmId}/environmental-impact
```

**Query Parameters:**
- `period` (optional): daily, weekly, monthly, yearly

**Response:**

```json
{
  "status": "success",
  "data": {
    "farmId": "DSA-FARM-USA-001",
    "period": "2025-12-01 to 2025-12-26",
    "wasteOutput": {
      "solidWaste": {"feces": 11700, "uneatenFeed": 390, "unit": "kg/month"},
      "dissolvedWaste": {"ammonia": 312, "nitrate": 208, "unit": "kg/month"}
    },
    "carbonFootprint": {
      "total": 3130,
      "unit": "kg CO2e/month",
      "breakdown": {
        "feedProduction": 2500,
        "transportation": 450,
        "operations": 180
      }
    },
    "ecosystemImpact": {
      "benthicImpact": "minimal",
      "waterQualityImpact": "acceptable"
    },
    "sustainabilityScore": 87
  }
}
```

---

## 7. Harvest and Traceability Endpoints

### 7.1 Record Harvest Event

```http
POST /api/v1/farms/{farmId}/harvest
```

**Request Body:**

```json
{
  "cageId": "CAGE-01",
  "harvestDate": "2025-12-26",
  "quantity": {
    "count": 500,
    "totalWeight": 1250
  },
  "quality": {
    "grade": "Premium",
    "fatContent": 12.5
  },
  "destination": {
    "processor": "Pacific Seafood Processing",
    "location": "Portland, OR"
  }
}
```

**Response:**

```json
{
  "status": "success",
  "data": {
    "harvestId": "HARVEST-2025-001",
    "batchId": "BATCH-2025-001",
    "traceabilityUrl": "https://trace.wia-aquaculture.org/batch/BATCH-2025-001",
    "qrCode": "data:image/png;base64,iVBORw0KG...",
    "blockchain": {
      "transactionHash": "0x7f9fade1c0d57a7af66ab4ead79fade1c0d",
      "network": "ethereum-mainnet"
    }
  }
}
```

### 7.2 Get Traceability Information

```http
GET /api/v1/traceability/{batchId}
```

**Response:**

```json
{
  "status": "success",
  "data": {
    "batchId": "BATCH-2025-001",
    "species": "Atlantic Salmon",
    "hatchery": {
      "id": "HATCH-NO-001",
      "location": "Norway",
      "hatchDate": "2024-03-15"
    },
    "farms": [
      {
        "farmId": "DSA-FARM-USA-001",
        "stockingDate": "2025-01-15",
        "harvestDate": "2025-12-26",
        "growthDuration": 345
      }
    ],
    "certifications": ["ASC", "BAP"],
    "blockchain": {
      "verified": true,
      "transactionHash": "0x7f9fade1c0d57a7af66ab4ead79fade1c0d"
    }
  }
}
```

---

## 8. Analytics and Reporting Endpoints

### 8.1 Get Performance Analytics

```http
GET /api/v1/farms/{farmId}/analytics
```

**Query Parameters:**
- `metric` (optional): growth, feed-efficiency, mortality, sustainability
- `period` (optional): daily, weekly, monthly, yearly
- `compare` (optional): industry-average, own-historical

**Response:**

```json
{
  "status": "success",
  "data": {
    "farmId": "DSA-FARM-USA-001",
    "period": "2025-12",
    "metrics": {
      "growthRate": {
        "value": 3.2,
        "unit": "percent/week",
        "comparison": {
          "industryAverage": 2.8,
          "performance": "above-average"
        }
      },
      "feedConversionRatio": {
        "value": 1.15,
        "comparison": {
          "industryAverage": 1.3,
          "performance": "excellent"
        }
      },
      "mortality": {
        "value": 0.1,
        "unit": "percent/week",
        "comparison": {
          "industryAverage": 0.15,
          "performance": "better"
        }
      },
      "sustainabilityScore": 87
    },
    "trends": {
      "growthRate": "improving",
      "feedEfficiency": "stable",
      "mortality": "improving"
    }
  }
}
```

---

## 9. Alerts and Notifications Endpoints

### 9.1 Get Active Alerts

```http
GET /api/v1/farms/{farmId}/alerts
```

**Query Parameters:**
- `severity` (optional): info, warning, critical
- `type` (optional): environmental, health, equipment, weather
- `status` (optional): active, acknowledged, resolved

**Response:**

```json
{
  "status": "success",
  "data": {
    "alerts": [
      {
        "alertId": "ALERT-2025-001",
        "farmId": "DSA-FARM-USA-001",
        "cageId": "CAGE-02",
        "type": "environmental",
        "severity": "warning",
        "message": "Dissolved oxygen below optimal range",
        "value": 5.5,
        "threshold": 6.0,
        "timestamp": "2025-12-26T09:45:00Z",
        "status": "active"
      }
    ]
  }
}
```

### 9.2 Subscribe to Webhooks

```http
POST /api/v1/webhooks
```

**Request Body:**

```json
{
  "url": "https://your-server.com/webhook",
  "events": [
    "sensor.critical",
    "health.disease_detected",
    "equipment.failure",
    "harvest.completed"
  ],
  "farmIds": ["DSA-FARM-USA-001"],
  "secret": "whsec_abcdef123456"
}
```

**Response:**

```json
{
  "status": "success",
  "data": {
    "webhookId": "wh_123456789",
    "status": "active",
    "created": "2025-12-26T10:30:00Z"
  }
}
```

---

## 10. Error Handling

All errors follow this format:

```json
{
  "status": "error",
  "error": {
    "code": "INVALID_FARM_ID",
    "message": "Farm ID not found",
    "details": {
      "farmId": "DSA-FARM-XXX-999"
    }
  },
  "timestamp": "2025-12-26T10:30:00Z"
}
```

### Common Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| INVALID_API_KEY | 401 | Invalid or missing API key |
| UNAUTHORIZED | 403 | Insufficient permissions |
| INVALID_FARM_ID | 404 | Farm not found |
| VALIDATION_ERROR | 400 | Invalid request data |
| RATE_LIMIT_EXCEEDED | 429 | Too many requests |
| SERVER_ERROR | 500 | Internal server error |

---

## 11. Rate Limiting

- **Standard Tier**: 1000 requests/hour
- **Professional Tier**: 10,000 requests/hour
- **Enterprise Tier**: Unlimited

Rate limit headers:

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 847
X-RateLimit-Reset: 1640520000
```

---

**弘益人間 (Hongik Ingan) - Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 MIT License*
