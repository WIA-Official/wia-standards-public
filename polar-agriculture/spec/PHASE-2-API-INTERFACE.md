# WIA-AGRI-022 Polar Agriculture - Phase 2: API Interface

> **Standard ID:** WIA-AGRI-022
> **Phase:** 2 - API Interface Specification
> **Version:** 1.0.0
> **Status:** ✅ Complete
> **Last Updated:** 2025-12-26

---

## 1. Overview

The WIA-AGRI-022 API Interface defines RESTful endpoints for managing polar agricultural facilities, enabling remote monitoring and control of farming operations in extreme cold environments.

### 1.1 Base URL

```
Production:  https://api.wiastandards.com/v1/polar-agriculture
Staging:     https://api-staging.wiastandards.com/v1/polar-agriculture
Development: http://localhost:8080/v1/polar-agriculture
```

### 1.2 API Principles

- **RESTful Design:** Standard HTTP methods (GET, POST, PUT, DELETE)
- **JSON First:** All requests and responses use JSON
- **Idempotent:** Safe retry of PUT/DELETE operations
- **Versioned:** API version in URL path
- **Secure:** HTTPS required, OAuth 2.0 authentication

---

## 2. Authentication

### 2.1 OAuth 2.0 Flow

```http
POST /oauth/token
Content-Type: application/json

{
  "grant_type": "client_credentials",
  "client_id": "your_client_id",
  "client_secret": "your_client_secret",
  "scope": "polar-farm:read polar-farm:write"
}
```

**Response:**
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "polar-farm:read polar-farm:write"
}
```

### 2.2 Using Access Token

```http
GET /v1/polar-agriculture/farms/POLAR-FARM-001
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
```

### 2.3 Scopes

| Scope | Permissions | Description |
|-------|-------------|-------------|
| `polar-farm:read` | GET | Read farm data and status |
| `polar-farm:write` | POST, PUT | Create and update farm data |
| `polar-farm:delete` | DELETE | Delete farm records |
| `polar-farm:admin` | ALL + config | Full administrative access |
| `polar-farm:emergency` | Override | Emergency control access |

---

## 3. Core Endpoints

### 3.1 Get All Farms

**Endpoint:** `GET /farms`

**Description:** Retrieve all polar farms associated with the authenticated user.

**Query Parameters:**
- `region` (optional): Filter by region (arctic, antarctic, subpolar)
- `status` (optional): Filter by operational status
- `page` (optional, default: 1): Page number
- `limit` (optional, default: 20, max: 100): Results per page

**Example Request:**
```http
GET /v1/polar-agriculture/farms?region=arctic&limit=10
Authorization: Bearer {token}
```

**Example Response (200 OK):**
```json
{
  "data": [
    {
      "farmId": "POLAR-FARM-SVB-001",
      "site": "Longyearbyen Research Station",
      "region": "arctic",
      "status": "operational",
      "crops": 2,
      "lastUpdate": "2025-12-26T14:22:00Z"
    },
    {
      "farmId": "POLAR-FARM-MCM-002",
      "site": "McMurdo Station",
      "region": "antarctic",
      "status": "maintenance",
      "crops": 1,
      "lastUpdate": "2025-12-25T09:15:00Z"
    }
  ],
  "pagination": {
    "page": 1,
    "limit": 10,
    "total": 2,
    "totalPages": 1
  }
}
```

---

### 3.2 Get Farm by ID

**Endpoint:** `GET /farms/{farmId}`

**Description:** Retrieve detailed information for a specific polar farm.

**Path Parameters:**
- `farmId` (required): Unique farm identifier

**Example Request:**
```http
GET /v1/polar-agriculture/farms/POLAR-FARM-SVB-001
Authorization: Bearer {token}
```

**Example Response (200 OK):**
```json
{
  "farmId": "POLAR-FARM-SVB-001",
  "standardVersion": "WIA-AGRI-022-v1.0",
  "location": {
    "latitude": 78.2232,
    "longitude": 15.6267,
    "site": "Longyearbyen Research Station",
    "region": "arctic"
  },
  "climate": {
    "externalTemp": -42.3,
    "internalTemp": 22.1,
    "targetTemp": 22.0,
    "humidity": 65,
    "co2Level": 1200
  },
  "crops": [
    {
      "cropId": "crop-lettuce-001",
      "species": "Lactuca sativa",
      "variety": "Arctic Green",
      "growthStage": "vegetative",
      "healthStatus": "excellent"
    }
  ],
  "energy": {
    "totalConsumption": 87.5,
    "batteryCharge": 78,
    "source": "hybrid"
  },
  "metadata": {
    "updatedAt": "2025-12-26T14:22:00Z"
  }
}
```

**Error Response (404 Not Found):**
```json
{
  "error": {
    "code": "FARM_NOT_FOUND",
    "message": "Farm with ID 'POLAR-FARM-XXX' not found",
    "timestamp": "2025-12-26T14:30:00Z"
  }
}
```

---

### 3.3 Create New Farm

**Endpoint:** `POST /farms`

**Description:** Register a new polar agriculture facility.

**Request Body:**
```json
{
  "farmId": "POLAR-FARM-NEW-001",
  "location": {
    "latitude": 64.8378,
    "longitude": -147.7164,
    "site": "Fairbanks Research Center",
    "region": "subpolar"
  },
  "facility": {
    "type": "modular",
    "totalArea": 100.0,
    "growingArea": 75.0
  },
  "climate": {
    "targetTemp": 20.0
  }
}
```

**Example Response (201 Created):**
```json
{
  "farmId": "POLAR-FARM-NEW-001",
  "status": "created",
  "message": "Polar farm successfully registered",
  "metadata": {
    "createdAt": "2025-12-26T14:35:00Z"
  }
}
```

**Validation Error (400 Bad Request):**
```json
{
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "Invalid farm data",
    "details": [
      {
        "field": "location.latitude",
        "issue": "Must be between -90 and 90"
      }
    ]
  }
}
```

---

### 3.4 Update Farm Data

**Endpoint:** `PUT /farms/{farmId}`

**Description:** Update farm configuration or real-time data.

**Request Body:**
```json
{
  "climate": {
    "externalTemp": -38.5,
    "internalTemp": 22.3,
    "humidity": 68,
    "co2Level": 1150
  },
  "crops": [
    {
      "cropId": "crop-lettuce-001",
      "growthStage": "flowering",
      "healthStatus": "excellent"
    }
  ]
}
```

**Example Response (200 OK):**
```json
{
  "farmId": "POLAR-FARM-SVB-001",
  "status": "updated",
  "fieldsUpdated": ["climate", "crops"],
  "metadata": {
    "updatedAt": "2025-12-26T14:40:00Z"
  }
}
```

---

### 3.5 Delete Farm

**Endpoint:** `DELETE /farms/{farmId}`

**Description:** Remove a farm from the system (soft delete).

**Example Response (200 OK):**
```json
{
  "farmId": "POLAR-FARM-OLD-001",
  "status": "deleted",
  "message": "Farm archived successfully",
  "metadata": {
    "deletedAt": "2025-12-26T14:45:00Z"
  }
}
```

---

## 4. Climate Control Endpoints

### 4.1 Update Climate Settings

**Endpoint:** `PUT /farms/{farmId}/climate`

**Request Body:**
```json
{
  "targetTemp": 23.0,
  "humidity": 70,
  "co2Level": 1300,
  "airFlow": 1600.0
}
```

**Response (200 OK):**
```json
{
  "status": "updated",
  "climate": {
    "targetTemp": 23.0,
    "currentTemp": 22.1,
    "adjustmentInProgress": true,
    "estimatedTime": "15 minutes"
  }
}
```

### 4.2 Get Climate History

**Endpoint:** `GET /farms/{farmId}/climate/history`

**Query Parameters:**
- `from` (required): Start datetime (ISO8601)
- `to` (required): End datetime (ISO8601)
- `interval` (optional, default: 1h): Data granularity (1m, 5m, 1h, 1d)

**Example Response:**
```json
{
  "farmId": "POLAR-FARM-SVB-001",
  "period": {
    "from": "2025-12-25T00:00:00Z",
    "to": "2025-12-26T00:00:00Z"
  },
  "data": [
    {
      "timestamp": "2025-12-25T00:00:00Z",
      "externalTemp": -40.2,
      "internalTemp": 22.0,
      "humidity": 65,
      "co2Level": 1200
    },
    {
      "timestamp": "2025-12-25T01:00:00Z",
      "externalTemp": -41.5,
      "internalTemp": 22.1,
      "humidity": 66,
      "co2Level": 1180
    }
  ]
}
```

---

## 5. Crop Management Endpoints

### 5.1 Add New Crop

**Endpoint:** `POST /farms/{farmId}/crops`

**Request Body:**
```json
{
  "species": "Spinacia oleracea",
  "variety": "Polar Spinach",
  "quantity": 200,
  "plantingDate": "2025-12-26T10:00:00Z",
  "expectedHarvest": "2026-02-10T10:00:00Z"
}
```

**Response (201 Created):**
```json
{
  "cropId": "crop-spinach-002",
  "status": "planted",
  "message": "Crop successfully added to farm"
}
```

### 5.2 Update Crop Status

**Endpoint:** `PUT /farms/{farmId}/crops/{cropId}`

**Request Body:**
```json
{
  "growthStage": "ripening",
  "healthStatus": "good"
}
```

### 5.3 Harvest Crop

**Endpoint:** `POST /farms/{farmId}/crops/{cropId}/harvest`

**Request Body:**
```json
{
  "harvestDate": "2025-12-26T15:00:00Z",
  "yieldAmount": 42.5,
  "yieldUnit": "kg",
  "quality": "grade-a"
}
```

**Response (200 OK):**
```json
{
  "cropId": "crop-lettuce-001",
  "status": "harvested",
  "yield": {
    "amount": 42.5,
    "unit": "kg",
    "quality": "grade-a"
  },
  "efficiency": {
    "yieldPerM2": 5.67,
    "growthDays": 44
  }
}
```

---

## 6. Alert Management

### 6.1 Get Active Alerts

**Endpoint:** `GET /farms/{farmId}/alerts`

**Query Parameters:**
- `severity` (optional): Filter by severity level
- `type` (optional): Filter by alert type

**Response:**
```json
{
  "farmId": "POLAR-FARM-SVB-001",
  "alerts": [
    {
      "alertId": "alert-001",
      "type": "temperature",
      "severity": "warning",
      "message": "Internal temperature 1.5°C below target",
      "timestamp": "2025-12-26T14:50:00Z",
      "acknowledged": false
    }
  ]
}
```

### 6.2 Acknowledge Alert

**Endpoint:** `PUT /farms/{farmId}/alerts/{alertId}/acknowledge`

**Response:**
```json
{
  "alertId": "alert-001",
  "status": "acknowledged",
  "acknowledgedBy": "operator-001",
  "acknowledgedAt": "2025-12-26T14:55:00Z"
}
```

---

## 7. Energy Management

### 7.1 Get Energy Statistics

**Endpoint:** `GET /farms/{farmId}/energy/stats`

**Query Parameters:**
- `period` (optional, default: 24h): Statistics period

**Response:**
```json
{
  "farmId": "POLAR-FARM-SVB-001",
  "period": "24h",
  "totalConsumption": 2100.0,
  "breakdown": {
    "heating": 1008.0,
    "lighting": 684.0,
    "ventilation": 192.0,
    "pumps": 216.0
  },
  "efficiency": {
    "kWhPerKgYield": 49.4,
    "comparedToAverage": "+12%"
  }
}
```

---

## 8. Error Codes

| Code | HTTP Status | Description | Solution |
|------|-------------|-------------|----------|
| `FARM_NOT_FOUND` | 404 | Farm ID does not exist | Verify farmId |
| `VALIDATION_ERROR` | 400 | Invalid request data | Check request body |
| `UNAUTHORIZED` | 401 | Invalid or missing token | Authenticate first |
| `FORBIDDEN` | 403 | Insufficient permissions | Request higher scope |
| `RATE_LIMIT_EXCEEDED` | 429 | Too many requests | Wait and retry |
| `INTERNAL_ERROR` | 500 | Server error | Contact support |
| `CLIMATE_OUT_OF_RANGE` | 422 | Climate parameters unsafe | Adjust values |
| `ENERGY_INSUFFICIENT` | 422 | Not enough power | Check energy system |

---

## 9. Rate Limiting

### 9.1 Rate Limits

| Scope | Requests per Minute | Requests per Hour |
|-------|---------------------|-------------------|
| `polar-farm:read` | 60 | 1000 |
| `polar-farm:write` | 30 | 500 |
| `polar-farm:admin` | 120 | 2000 |

### 9.2 Rate Limit Headers

```http
X-RateLimit-Limit: 60
X-RateLimit-Remaining: 45
X-RateLimit-Reset: 1735223400
```

---

## 10. Webhooks

### 10.1 Webhook Configuration

**Endpoint:** `POST /webhooks`

**Request Body:**
```json
{
  "url": "https://your-server.com/webhook/polar-farm",
  "events": ["climate.alert", "crop.harvest", "system.emergency"],
  "secret": "your_webhook_secret"
}
```

### 10.2 Webhook Payload Example

```json
{
  "event": "climate.alert",
  "farmId": "POLAR-FARM-SVB-001",
  "timestamp": "2025-12-26T15:00:00Z",
  "data": {
    "alertType": "temperature",
    "severity": "critical",
    "externalTemp": -55.0,
    "internalTemp": 18.5,
    "message": "Heating system failure detected"
  },
  "signature": "sha256=..."
}
```

---

## 11. SDK Examples

### 11.1 JavaScript/Node.js

```javascript
const WIA_PolarAgriculture = require('@wia/polar-agriculture');

const client = new WIA_PolarAgriculture({
  clientId: 'your_client_id',
  clientSecret: 'your_client_secret'
});

// Get farm data
const farm = await client.farms.get('POLAR-FARM-SVB-001');
console.log(`External Temp: ${farm.climate.externalTemp}°C`);

// Update climate
await client.farms.updateClimate('POLAR-FARM-SVB-001', {
  targetTemp: 23.0,
  co2Level: 1300
});
```

### 11.2 Python

```python
from wia_polar_agriculture import WIAClient

client = WIAClient(
    client_id='your_client_id',
    client_secret='your_client_secret'
)

# Get all farms
farms = client.farms.list(region='arctic')
for farm in farms:
    print(f"{farm.site}: {farm.climate.internalTemp}°C")

# Harvest crop
harvest = client.crops.harvest(
    farm_id='POLAR-FARM-SVB-001',
    crop_id='crop-lettuce-001',
    yield_amount=42.5,
    yield_unit='kg'
)
```

---

**弘益人間 (Hongik Ingan) - Benefit All Humanity**

*WIA-AGRI-022 Polar Agriculture Standard - API Interface*
*© 2025 WIA - World Certification Industry Association*
*MIT License*
