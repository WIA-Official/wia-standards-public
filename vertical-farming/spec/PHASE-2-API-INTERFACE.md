# WIA-AGRI-018: Vertical Farming Standard
## Phase 2: API Interface Specification

**Version:** 1.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-01-01

---

## 1. Overview

This specification defines RESTful API interfaces for vertical farming systems, enabling integration with IoT devices, farm management software, and supply chain platforms.

### 1.1 Design Principles

- **RESTful Architecture**: Standard HTTP methods and status codes
- **Stateless**: Each request contains all necessary information
- **Versioned**: Backward compatibility through API versioning
- **Secure**: TLS encryption and token-based authentication
- **Real-time**: WebSocket support for live sensor streams

---

## 2. API Endpoints

### 2.1 Base URL

```
Production: https://api.verticalfarm.io/v1
Staging: https://api-staging.verticalfarm.io/v1
Development: http://localhost:3000/v1
```

### 2.2 Authentication

All API requests require Bearer token authentication.

**Request Header:**
```http
Authorization: Bearer <access_token>
X-Farm-ID: VF-SEOUL-001
Content-Type: application/json
```

**Authentication Endpoint:**
```http
POST /auth/login
Content-Type: application/json

{
  "email": "operator@verticalfarm.com",
  "password": "secure_password",
  "farmId": "VF-SEOUL-001"
}
```

**Response:**
```json
{
  "success": true,
  "token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "expiresIn": 3600,
  "refreshToken": "refresh_token_here",
  "user": {
    "id": "USER-123",
    "email": "operator@verticalfarm.com",
    "role": "FARM_OPERATOR",
    "permissions": ["READ_SENSORS", "CONTROL_ENVIRONMENT", "MANAGE_CROPS"]
  }
}
```

---

## 3. Farm Management APIs

### 3.1 Get Farm Configuration

```http
GET /farms/{farmId}
```

**Response:**
```json
{
  "success": true,
  "data": {
    "farmId": "VF-SEOUL-001",
    "farmName": "Seoul Vertical Farm",
    "location": {
      "address": "123 Gangnam-daero, Seoul",
      "latitude": 37.5665,
      "longitude": 126.9780
    },
    "infrastructure": {
      "totalArea": 1000,
      "numberOfTiers": 8,
      "totalCapacity": 8000
    },
    "status": "OPERATIONAL"
  }
}
```

### 3.2 Update Farm Configuration

```http
PUT /farms/{farmId}
Content-Type: application/json

{
  "farmName": "Seoul Vertical Farm - Expanded",
  "numberOfTiers": 10,
  "capacity": 10000
}
```

**Response:**
```json
{
  "success": true,
  "message": "Farm configuration updated successfully",
  "data": {
    "farmId": "VF-SEOUL-001",
    "updatedAt": "2025-01-01T10:30:00Z"
  }
}
```

---

## 4. Environmental Control APIs

### 4.1 Get Environmental Data

```http
GET /farms/{farmId}/environment?tier=3&limit=100
```

**Query Parameters:**
- `tier` (optional): Filter by specific tier
- `zone` (optional): Filter by zone (A, B, C, etc.)
- `from` (optional): Start timestamp (ISO 8601)
- `to` (optional): End timestamp (ISO 8601)
- `limit` (optional): Number of records (default: 100, max: 1000)

**Response:**
```json
{
  "success": true,
  "data": [
    {
      "timestamp": "2025-01-01T10:30:00Z",
      "tier": 3,
      "zone": "A",
      "temperature": 22.5,
      "humidity": 65,
      "co2": 1200,
      "lightIntensity": 450,
      "ph": 6.2,
      "ec": 1.8
    }
  ],
  "pagination": {
    "total": 500,
    "limit": 100,
    "offset": 0,
    "hasMore": true
  }
}
```

### 4.2 Set Environmental Setpoints

```http
POST /farms/{farmId}/environment/setpoints
Content-Type: application/json

{
  "tier": 3,
  "zone": "A",
  "setpoints": {
    "temperature": {
      "value": 23.0,
      "tolerance": 1.5
    },
    "humidity": {
      "value": 68,
      "tolerance": 5
    },
    "co2": {
      "value": 1300,
      "tolerance": 100
    },
    "lightIntensity": {
      "value": 500,
      "photoperiod": {
        "hoursOn": 16,
        "hoursOff": 8
      }
    }
  }
}
```

**Response:**
```json
{
  "success": true,
  "message": "Environmental setpoints updated",
  "data": {
    "tier": 3,
    "zone": "A",
    "appliedAt": "2025-01-01T10:35:00Z",
    "estimatedStabilization": "2025-01-01T10:50:00Z"
  }
}
```

### 4.3 Real-time Sensor Stream (WebSocket)

```javascript
// WebSocket connection
const ws = new WebSocket('wss://api.verticalfarm.io/v1/stream');

ws.onopen = () => {
  ws.send(JSON.stringify({
    type: 'subscribe',
    farmId: 'VF-SEOUL-001',
    tier: 3,
    parameters: ['temperature', 'humidity', 'co2']
  }));
};

ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  console.log('Sensor update:', data);
  // {
  //   "type": "sensor_update",
  //   "timestamp": "2025-01-01T10:30:45.123Z",
  //   "tier": 3,
  //   "temperature": 22.5,
  //   "humidity": 65,
  //   "co2": 1200
  // }
};
```

---

## 5. Crop Management APIs

### 5.1 Create Crop Batch

```http
POST /farms/{farmId}/crops
Content-Type: application/json

{
  "tier": 3,
  "cropType": {
    "species": "Lactuca sativa",
    "variety": "Green Leaf Lettuce",
    "cultivar": "Rex"
  },
  "seedingDate": "2025-01-01T00:00:00Z",
  "plantCount": 500,
  "expectedHarvestDate": "2025-01-30T00:00:00Z",
  "nutrientSchedule": "LETTUCE-VEG-01"
}
```

**Response:**
```json
{
  "success": true,
  "message": "Crop batch created successfully",
  "data": {
    "batchId": "BATCH-2025-001",
    "qrCode": "https://api.verticalfarm.io/qr/BATCH-2025-001.png",
    "trackingUrl": "https://trace.verticalfarm.io/BATCH-2025-001"
  }
}
```

### 5.2 Get Crop Batch Status

```http
GET /farms/{farmId}/crops/{batchId}
```

**Response:**
```json
{
  "success": true,
  "data": {
    "batchId": "BATCH-2025-001",
    "cropType": "Green Leaf Lettuce",
    "currentAge": 17,
    "growthStage": "VEGETATIVE",
    "healthScore": 95,
    "plantCount": 480,
    "estimatedYield": 216,
    "daysToHarvest": 13,
    "lastUpdated": "2025-01-01T10:00:00Z"
  }
}
```

### 5.3 Update Crop Observations

```http
POST /farms/{farmId}/crops/{batchId}/observations
Content-Type: application/json

{
  "timestamp": "2025-01-01T09:00:00Z",
  "observer": "EMP-123",
  "observations": {
    "averageHeight": 12.5,
    "leafCount": 8,
    "healthScore": 95,
    "notes": "Excellent growth, uniform development",
    "images": [
      "https://storage.verticalfarm.io/images/batch-001-day17-01.jpg"
    ]
  }
}
```

**Response:**
```json
{
  "success": true,
  "message": "Observation recorded",
  "data": {
    "observationId": "OBS-2025-001",
    "timestamp": "2025-01-01T09:00:00Z"
  }
}
```

### 5.4 List Active Crops

```http
GET /farms/{farmId}/crops?status=ACTIVE&tier=3
```

**Query Parameters:**
- `status`: SEEDING, VEGETATIVE, READY_TO_HARVEST, HARVESTED
- `tier`: Filter by tier number
- `cropType`: Filter by crop species

**Response:**
```json
{
  "success": true,
  "data": [
    {
      "batchId": "BATCH-2025-001",
      "tier": 3,
      "cropType": "Green Leaf Lettuce",
      "growthStage": "VEGETATIVE",
      "plantCount": 480,
      "daysToHarvest": 13
    },
    {
      "batchId": "BATCH-2025-002",
      "tier": 3,
      "cropType": "Basil",
      "growthStage": "VEGETATIVE",
      "plantCount": 320,
      "daysToHarvest": 20
    }
  ],
  "total": 2
}
```

---

## 6. Harvest Management APIs

### 6.1 Record Harvest

```http
POST /farms/{farmId}/harvests
Content-Type: application/json

{
  "batchId": "BATCH-2025-001",
  "harvestDate": "2025-01-15T08:30:00Z",
  "operator": "EMP-123",
  "yield": {
    "totalWeight": 216.5,
    "plantCount": 480,
    "wasteWeight": 8.2
  },
  "quality": {
    "grade": "PREMIUM",
    "appearance": 9.5,
    "freshness": 9.8
  },
  "packaging": {
    "packageType": "CLAMSHELL_200G",
    "packagesProduced": 1080
  }
}
```

**Response:**
```json
{
  "success": true,
  "message": "Harvest recorded successfully",
  "data": {
    "harvestId": "HRV-2025-001",
    "blockchainTx": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb",
    "certificateUrl": "https://cert.verticalfarm.io/HRV-2025-001.pdf",
    "qrCodes": [
      "https://api.verticalfarm.io/qr/HRV-2025-001-PKG-001.png"
    ]
  }
}
```

### 6.2 Get Harvest History

```http
GET /farms/{farmId}/harvests?from=2025-01-01&to=2025-01-31
```

**Response:**
```json
{
  "success": true,
  "data": [
    {
      "harvestId": "HRV-2025-001",
      "batchId": "BATCH-2025-001",
      "harvestDate": "2025-01-15T08:30:00Z",
      "cropType": "Green Leaf Lettuce",
      "totalWeight": 216.5,
      "grade": "PREMIUM",
      "revenue": 3248.50
    }
  ],
  "summary": {
    "totalHarvests": 15,
    "totalWeight": 3248,
    "averageQuality": 9.3,
    "totalRevenue": 48727.50
  }
}
```

---

## 7. Alert Management APIs

### 7.1 Get Active Alerts

```http
GET /farms/{farmId}/alerts?severity=WARNING,CRITICAL
```

**Response:**
```json
{
  "success": true,
  "data": [
    {
      "alertId": "ALERT-2025-001",
      "tier": 3,
      "severity": "WARNING",
      "type": "TEMPERATURE_HIGH",
      "message": "Temperature exceeding optimal range",
      "timestamp": "2025-01-01T14:22:15Z",
      "resolved": false
    }
  ],
  "total": 1
}
```

### 7.2 Acknowledge Alert

```http
POST /farms/{farmId}/alerts/{alertId}/acknowledge
Content-Type: application/json

{
  "acknowledgedBy": "EMP-123",
  "notes": "Increased cooling, monitoring situation"
}
```

**Response:**
```json
{
  "success": true,
  "message": "Alert acknowledged",
  "data": {
    "alertId": "ALERT-2025-001",
    "acknowledgedAt": "2025-01-01T14:25:00Z"
  }
}
```

### 7.3 Resolve Alert

```http
POST /farms/{farmId}/alerts/{alertId}/resolve
Content-Type: application/json

{
  "resolvedBy": "EMP-123",
  "resolution": "Cooling system restored, temperature normalized",
  "preventiveMeasures": "Scheduled HVAC maintenance"
}
```

---

## 8. Analytics & Reporting APIs

### 8.1 Get Production Analytics

```http
GET /farms/{farmId}/analytics/production?period=monthly&year=2025&month=1
```

**Response:**
```json
{
  "success": true,
  "data": {
    "period": "2025-01",
    "harvests": 15,
    "totalYield": 3248,
    "averageYieldPerBatch": 216.5,
    "productionByTier": [
      { "tier": 1, "yield": 450 },
      { "tier": 2, "yield": 430 },
      { "tier": 3, "yield": 420 }
    ],
    "revenueByGrade": {
      "PREMIUM": 35000,
      "GRADE_A": 12000,
      "GRADE_B": 1728
    }
  }
}
```

### 8.2 Get Resource Efficiency Metrics

```http
GET /farms/{farmId}/analytics/resources?period=monthly
```

**Response:**
```json
{
  "success": true,
  "data": {
    "period": "2025-01",
    "production": 3248,
    "waterUsage": {
      "total": 15000,
      "perKg": 4.62,
      "savings": 93.4
    },
    "energyUsage": {
      "total": 12000,
      "perKg": 3.69,
      "renewable": 80
    },
    "carbonFootprint": {
      "total": 405,
      "perKg": 0.125,
      "offset": 100
    }
  }
}
```

---

## 9. Integration APIs

### 9.1 Export Data (CSV/JSON)

```http
GET /farms/{farmId}/export?type=environmental&from=2025-01-01&to=2025-01-31&format=csv
```

**Response Headers:**
```http
Content-Type: text/csv
Content-Disposition: attachment; filename="VF-SEOUL-001-environmental-2025-01.csv"
```

### 9.2 Blockchain Traceability

```http
GET /farms/{farmId}/blockchain/{harvestId}
```

**Response:**
```json
{
  "success": true,
  "data": {
    "harvestId": "HRV-2025-001",
    "blockchainNetwork": "Ethereum",
    "contractAddress": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb",
    "transactionHash": "0x9f2d4e...",
    "timestamp": "2025-01-15T08:35:00Z",
    "verified": true,
    "explorerUrl": "https://etherscan.io/tx/0x9f2d4e..."
  }
}
```

---

## 10. Error Handling

### 10.1 Error Response Format

```json
{
  "success": false,
  "error": {
    "code": "INVALID_CREDENTIALS",
    "message": "Invalid email or password",
    "details": "Authentication failed",
    "timestamp": "2025-01-01T10:30:00Z",
    "requestId": "req_123456"
  }
}
```

### 10.2 HTTP Status Codes

| Code | Meaning | Usage |
|------|---------|-------|
| 200 | OK | Successful GET/PUT request |
| 201 | Created | Successful POST creating resource |
| 204 | No Content | Successful DELETE |
| 400 | Bad Request | Invalid parameters |
| 401 | Unauthorized | Missing/invalid token |
| 403 | Forbidden | Insufficient permissions |
| 404 | Not Found | Resource doesn't exist |
| 409 | Conflict | Resource already exists |
| 422 | Unprocessable Entity | Validation error |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Server Error | Server error |
| 503 | Service Unavailable | Maintenance mode |

### 10.3 Common Error Codes

- `INVALID_CREDENTIALS`: Authentication failed
- `TOKEN_EXPIRED`: Access token expired
- `INSUFFICIENT_PERMISSIONS`: User lacks required permission
- `RESOURCE_NOT_FOUND`: Requested resource doesn't exist
- `VALIDATION_ERROR`: Input validation failed
- `DUPLICATE_RESOURCE`: Resource already exists
- `RATE_LIMIT_EXCEEDED`: Too many requests
- `SYSTEM_MAINTENANCE`: System under maintenance

---

## 11. Rate Limiting

**Rate Limits:**
- Standard tier: 1000 requests/hour
- Premium tier: 10000 requests/hour
- Enterprise tier: Unlimited

**Response Headers:**
```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 950
X-RateLimit-Reset: 1640995200
```

---

## 12. Webhooks

### 12.1 Register Webhook

```http
POST /farms/{farmId}/webhooks
Content-Type: application/json

{
  "url": "https://your-server.com/webhook",
  "events": ["harvest.completed", "alert.critical", "crop.ready"],
  "secret": "your_webhook_secret"
}
```

### 12.2 Webhook Payload Example

```json
{
  "event": "harvest.completed",
  "timestamp": "2025-01-15T08:35:00Z",
  "farmId": "VF-SEOUL-001",
  "data": {
    "harvestId": "HRV-2025-001",
    "batchId": "BATCH-2025-001",
    "totalWeight": 216.5,
    "grade": "PREMIUM"
  },
  "signature": "sha256=abc123..."
}
```

---

## 13. SDK Examples

### 13.1 JavaScript/TypeScript

```typescript
import { VerticalFarmAPI } from '@wia/vertical-farming';

const api = new VerticalFarmAPI({
  apiKey: 'your_api_key',
  farmId: 'VF-SEOUL-001'
});

// Get environmental data
const env = await api.environment.getCurrent(3); // tier 3

// Set temperature setpoint
await api.environment.setpoint({
  tier: 3,
  temperature: 23.0,
  humidity: 68
});

// Create crop batch
const batch = await api.crops.create({
  tier: 3,
  cropType: 'lettuce',
  plantCount: 500
});
```

### 13.2 Python

```python
from wia_vertical_farming import VerticalFarmAPI

api = VerticalFarmAPI(
    api_key='your_api_key',
    farm_id='VF-SEOUL-001'
)

# Get environmental data
env_data = api.environment.get_current(tier=3)

# Record harvest
harvest = api.harvests.create(
    batch_id='BATCH-2025-001',
    total_weight=216.5,
    grade='PREMIUM'
)
```

---

## 14. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01-01 | Initial API specification |

---

**© 2025 WIA Standards · MIT License**
**弘益人間 (Benefit All Humanity)**
