# WIA-AGRI-024: Seaweed Farming Standard
## Phase 2 - API Interface Specification

**Version:** 1.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-01-01

---

## 1. Overview

This specification defines RESTful API interfaces for seaweed farming systems, enabling integration between farm management software, sensor networks, analytics platforms, and carbon credit marketplaces.

### 1.1 Design Principles

- **RESTful Architecture**: Resource-oriented design with standard HTTP methods
- **Hypermedia-Driven**: HATEOAS links for discoverability
- **Real-time Capable**: WebSocket and Server-Sent Events support
- **Marine-Grade**: Designed for intermittent connectivity in ocean environments
- **Carbon-Focused**: First-class support for carbon sequestration data

---

## 2. Base Configuration

### 2.1 Base URL

```
Production:  https://api.wia-seaweed.org/v1
Staging:     https://api-staging.wia-seaweed.org/v1
Development: http://localhost:3000/v1
```

### 2.2 Authentication

```http
Authorization: Bearer <JWT_TOKEN>
X-API-Key: <API_KEY>
```

**OAuth 2.0 Scopes:**
- `farm:read` - Read farm data
- `farm:write` - Modify farm data
- `sensor:read` - Access sensor data
- `harvest:write` - Record harvest operations
- `carbon:read` - View carbon sequestration data
- `analytics:read` - Access analytics and reports

### 2.3 Rate Limiting

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 995
X-RateLimit-Reset: 1609459200
```

**Limits:**
- Standard: 1,000 requests/hour
- Premium: 10,000 requests/hour
- Sensor Data: 100,000 requests/hour

---

## 3. Farm Management API

### 3.1 List All Farms

```http
GET /farms
```

**Query Parameters:**
- `country` (string): Filter by country code
- `species` (string): Filter by seaweed species
- `status` (enum): active, inactive, maintenance
- `page` (number): Page number (default: 1)
- `limit` (number): Results per page (default: 20, max: 100)

**Response:**

```json
{
  "data": [
    {
      "farmId": "550e8400-e29b-41d4-a716-446655440000",
      "farmName": "Ocean Harvest Kelp Farm",
      "farmType": "longline",
      "location": {
        "country": "US",
        "region": "Oregon Coast",
        "coordinates": {
          "latitude": 45.234567,
          "longitude": -124.056789
        }
      },
      "species": ["kelp", "nori"],
      "totalArea": 5.5,
      "status": "active",
      "links": {
        "self": "/farms/550e8400-e29b-41d4-a716-446655440000",
        "growth": "/farms/550e8400-e29b-41d4-a716-446655440000/growth",
        "harvest": "/farms/550e8400-e29b-41d4-a716-446655440000/harvests"
      }
    }
  ],
  "pagination": {
    "page": 1,
    "limit": 20,
    "total": 145,
    "pages": 8
  }
}
```

### 3.2 Get Farm Details

```http
GET /farms/{farmId}
```

**Response:**

```json
{
  "farmId": "550e8400-e29b-41d4-a716-446655440000",
  "farmName": "Ocean Harvest Kelp Farm",
  "farmType": "longline",
  "location": {
    "latitude": 45.234567,
    "longitude": -124.056789,
    "waterBody": "Pacific Ocean",
    "depth": 15,
    "coastalRegion": "Oregon Coast",
    "country": "US",
    "timezone": "America/Los_Angeles"
  },
  "capacity": {
    "totalArea": 5.5,
    "cultivationLines": 120,
    "ropeLength": 24000,
    "maxBiomass": 165
  },
  "currentStatus": {
    "biomass": 89.5,
    "occupancyRate": 0.543,
    "healthScore": 94,
    "lastUpdated": "2025-01-15T14:30:00Z"
  },
  "certifications": ["Organic", "WIA-AGRI-024", "Blue Carbon Verified"],
  "establishedDate": "2023-06-15T00:00:00Z",
  "operator": {
    "name": "Ocean Harvest LLC",
    "contact": "contact@oceanharvest.com",
    "did": "did:wia:ocean-harvest-001"
  }
}
```

### 3.3 Create New Farm

```http
POST /farms
Content-Type: application/json
```

**Request Body:**

```json
{
  "farmName": "New Kelp Farm",
  "farmType": "longline",
  "location": {
    "latitude": 45.234567,
    "longitude": -124.056789,
    "waterBody": "Pacific Ocean",
    "depth": 15,
    "country": "US"
  },
  "capacity": {
    "totalArea": 3.0,
    "cultivationLines": 60,
    "ropeLength": 12000
  },
  "operator": {
    "name": "Farm Operator Name",
    "contact": "operator@example.com",
    "license": "AQ-OR-2025-0123"
  }
}
```

**Response:**

```json
{
  "farmId": "660e9500-f39c-52e5-b827-557766551111",
  "status": "created",
  "message": "Farm successfully registered",
  "links": {
    "self": "/farms/660e9500-f39c-52e5-b827-557766551111"
  }
}
```

### 3.4 Update Farm

```http
PATCH /farms/{farmId}
Content-Type: application/json
```

---

## 4. Water Quality API

### 4.1 Get Current Water Quality

```http
GET /farms/{farmId}/water-quality
```

**Query Parameters:**
- `zoneId` (string): Specific cultivation zone
- `latest` (boolean): Return only most recent reading

**Response:**

```json
{
  "farmId": "550e8400-e29b-41d4-a716-446655440000",
  "timestamp": "2025-01-15T14:30:00Z",
  "measurements": {
    "temperature": {
      "value": 15.5,
      "unit": "°C",
      "status": "optimal"
    },
    "salinity": {
      "value": 32.5,
      "unit": "PSU",
      "status": "optimal"
    },
    "pH": {
      "value": 8.1,
      "status": "optimal"
    },
    "nitrate": {
      "value": 15.2,
      "unit": "mg/L",
      "status": "high"
    },
    "phosphate": {
      "value": 1.8,
      "unit": "mg/L",
      "status": "optimal"
    },
    "dissolvedOxygen": {
      "value": 8.5,
      "unit": "mg/L",
      "saturation": 95,
      "status": "optimal"
    },
    "currentSpeed": {
      "value": 0.25,
      "unit": "m/s",
      "direction": 180
    }
  },
  "alerts": [],
  "nextMeasurement": "2025-01-15T14:45:00Z"
}
```

### 4.2 Get Water Quality History

```http
GET /farms/{farmId}/water-quality/history
```

**Query Parameters:**
- `startDate` (ISO 8601): Start of time range
- `endDate` (ISO 8601): End of time range
- `parameter` (string): Specific parameter (temperature, salinity, etc.)
- `interval` (enum): raw, hourly, daily, weekly
- `format` (enum): json, csv, netcdf

**Response:**

```json
{
  "farmId": "550e8400-e29b-41d4-a716-446655440000",
  "parameter": "temperature",
  "interval": "hourly",
  "timeRange": {
    "start": "2025-01-01T00:00:00Z",
    "end": "2025-01-15T23:59:59Z"
  },
  "data": [
    {
      "timestamp": "2025-01-01T00:00:00Z",
      "value": 14.2,
      "unit": "°C"
    },
    {
      "timestamp": "2025-01-01T01:00:00Z",
      "value": 14.1,
      "unit": "°C"
    }
  ],
  "statistics": {
    "min": 12.5,
    "max": 17.8,
    "mean": 15.3,
    "stdDev": 1.2
  }
}
```

### 4.3 Submit Water Quality Data

```http
POST /farms/{farmId}/water-quality
Content-Type: application/json
```

**Request Body:**

```json
{
  "sensorId": "SENSOR-WQ-001",
  "zoneId": "ZONE-A",
  "timestamp": "2025-01-15T14:30:00Z",
  "depth": 5,
  "measurements": {
    "temperature": 15.5,
    "salinity": 32.5,
    "pH": 8.1,
    "nitrate": 15.2,
    "phosphate": 1.8
  }
}
```

---

## 5. Growth Monitoring API

### 5.1 Get Growth Data

```http
GET /farms/{farmId}/growth
```

**Query Parameters:**
- `lineId` (string): Specific cultivation line
- `speciesId` (string): Filter by species
- `startDate`, `endDate` (ISO 8601): Date range

**Response:**

```json
{
  "farmId": "550e8400-e29b-41d4-a716-446655440000",
  "timestamp": "2025-01-15T14:30:00Z",
  "overallMetrics": {
    "totalBiomass": 89500,
    "averageDensity": 12.5,
    "healthScore": 94,
    "maturityIndex": 78
  },
  "bySpecies": [
    {
      "speciesId": "kelp-saccharina-latissima",
      "scientificName": "Saccharina latissima",
      "commonName": "Sugar Kelp",
      "biomass": 75000,
      "averageLength": 2.3,
      "growthRate": 0.15,
      "harvestReadiness": false,
      "daysToHarvest": 45
    }
  ],
  "zoneMetrics": [
    {
      "zoneId": "ZONE-A",
      "biomass": 45000,
      "healthScore": 96,
      "maturityIndex": 82
    }
  ]
}
```

### 5.2 Submit Growth Measurement

```http
POST /farms/{farmId}/growth
Content-Type: application/json
```

**Request Body:**

```json
{
  "lineId": "LINE-025",
  "speciesId": "kelp-saccharina-latissima",
  "timestamp": "2025-01-15T10:00:00Z",
  "measurementMethod": "drone",
  "growthMetrics": {
    "biomass": {
      "density": 12.5,
      "totalWeight": 500
    },
    "dimensions": {
      "averageLength": 230,
      "averageWidth": 15
    },
    "healthScore": {
      "overall": 95,
      "color": "vibrant",
      "diseasePresence": false
    }
  }
}
```

---

## 6. Harvest Management API

### 6.1 Get Harvest History

```http
GET /farms/{farmId}/harvests
```

**Response:**

```json
{
  "farmId": "550e8400-e29b-41d4-a716-446655440000",
  "totalHarvests": 12,
  "data": [
    {
      "harvestId": "HARVEST-2025-001",
      "harvestDate": "2025-01-10T08:00:00Z",
      "species": "Sugar Kelp",
      "yield": {
        "totalWetWeight": 5000,
        "totalDryWeight": 500,
        "yieldPerArea": 909
      },
      "qualityGrade": "premium",
      "destination": "food",
      "carbonCredits": 12.5
    }
  ]
}
```

### 6.2 Schedule Harvest

```http
POST /farms/{farmId}/harvests
Content-Type: application/json
```

**Request Body:**

```json
{
  "scheduledDate": "2025-03-15T07:00:00Z",
  "speciesId": "kelp-saccharina-latissima",
  "zoneIds": ["ZONE-A", "ZONE-B"],
  "harvestMethod": "mechanical",
  "expectedYield": 5000,
  "destination": {
    "purpose": "food",
    "buyer": "Kelp Foods Inc",
    "processingFacility": "PROC-001"
  }
}
```

---

## 7. Carbon Sequestration API

### 7.1 Get Carbon Data

```http
GET /farms/{farmId}/carbon
```

**Query Parameters:**
- `period` (enum): current, monthly, quarterly, annual
- `includeCredits` (boolean): Include carbon credit information

**Response:**

```json
{
  "farmId": "550e8400-e29b-41d4-a716-446655440000",
  "reportingPeriod": {
    "start": "2024-01-01T00:00:00Z",
    "end": "2024-12-31T23:59:59Z",
    "duration": 365
  },
  "carbonMetrics": {
    "biomassCarbonContent": {
      "totalCarbon": 18500,
      "carbonPercentage": 37
    },
    "co2Sequestered": {
      "totalCO2": 67850,
      "co2PerHectare": 12336,
      "co2PerDay": 185.9
    },
    "carbonCredits": {
      "totalCredits": 67.85,
      "creditPrice": 25.50,
      "totalValue": 1730.18,
      "verificationStatus": "verified"
    },
    "ecosystemBenefits": {
      "nitrogenRemoval": 850,
      "phosphorusRemoval": 125,
      "oxygenProduction": 50000,
      "habitatProvided": 55000
    }
  },
  "methodology": {
    "calculationStandard": "Verra VCS",
    "carbonFactor": 1.0,
    "verifiedBy": "Ocean Carbon Verification",
    "verificationDate": "2025-01-05T00:00:00Z"
  }
}
```

### 7.2 Submit Carbon Report

```http
POST /farms/{farmId}/carbon/reports
Content-Type: application/json
```

---

## 8. Real-time Streaming API

### 8.1 WebSocket Connection

```javascript
const ws = new WebSocket('wss://api.wia-seaweed.org/v1/stream');

ws.on('open', () => {
  ws.send(JSON.stringify({
    action: 'subscribe',
    farmId: '550e8400-e29b-41d4-a716-446655440000',
    channels: ['water-quality', 'growth', 'alerts']
  }));
});

ws.on('message', (data) => {
  const event = JSON.parse(data);
  console.log('Event:', event);
});
```

### 8.2 Server-Sent Events

```http
GET /farms/{farmId}/stream
Accept: text/event-stream
```

**Event Stream:**

```
event: water-quality
data: {"temperature": 15.5, "salinity": 32.5}

event: growth-update
data: {"biomass": 89500, "healthScore": 94}

event: alert
data: {"type": "temperature_warning", "value": 18.5}
```

---

## 9. Analytics API

### 9.1 Get Farm Analytics

```http
GET /farms/{farmId}/analytics
```

**Query Parameters:**
- `metric` (enum): growth_rate, yield_prediction, health_trends, carbon_forecast
- `period` (enum): week, month, quarter, year

**Response:**

```json
{
  "farmId": "550e8400-e29b-41d4-a716-446655440000",
  "analytics": {
    "growthForecast": {
      "predictedBiomass": 165000,
      "confidence": 0.92,
      "harvestDate": "2025-04-15T00:00:00Z"
    },
    "healthTrends": {
      "currentScore": 94,
      "trend": "improving",
      "factors": ["optimal_nutrients", "favorable_temperature"]
    },
    "yieldPrediction": {
      "estimatedYield": 6500,
      "qualityGrade": "premium",
      "confidence": 0.88
    },
    "carbonForecast": {
      "projectedAnnualCO2": 82000,
      "projectedCredits": 82,
      "projectedValue": 2091
    }
  }
}
```

---

## 10. Error Handling

### 10.1 Error Response Format

```json
{
  "error": {
    "code": "RESOURCE_NOT_FOUND",
    "message": "Farm with ID 550e8400-e29b-41d4-a716-446655440000 not found",
    "details": {
      "resource": "farm",
      "resourceId": "550e8400-e29b-41d4-a716-446655440000"
    },
    "timestamp": "2025-01-15T14:30:00Z",
    "requestId": "req_abc123xyz"
  }
}
```

### 10.2 HTTP Status Codes

| Code | Meaning | Usage |
|------|---------|-------|
| 200 | OK | Successful GET request |
| 201 | Created | Resource successfully created |
| 204 | No Content | Successful DELETE |
| 400 | Bad Request | Invalid request format |
| 401 | Unauthorized | Missing or invalid authentication |
| 403 | Forbidden | Insufficient permissions |
| 404 | Not Found | Resource doesn't exist |
| 409 | Conflict | Resource already exists |
| 422 | Unprocessable Entity | Validation error |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Server Error | Server error |
| 503 | Service Unavailable | Temporary unavailability |

---

## 11. SDK Examples

### 11.1 JavaScript/TypeScript

```typescript
import { SeaweedFarmAPI } from '@wia/seaweed-farming-sdk';

const api = new SeaweedFarmAPI({
  apiKey: process.env.WIA_API_KEY,
  baseUrl: 'https://api.wia-seaweed.org/v1'
});

// Get farm details
const farm = await api.farms.get('550e8400-e29b-41d4-a716-446655440000');

// Subscribe to real-time updates
api.stream.subscribe({
  farmId: farm.farmId,
  channels: ['water-quality', 'growth'],
  onData: (event) => console.log(event)
});

// Get carbon data
const carbon = await api.carbon.get(farm.farmId, { period: 'annual' });
```

### 11.2 Python

```python
from wia_seaweed import SeaweedFarmAPI

api = SeaweedFarmAPI(
    api_key=os.getenv('WIA_API_KEY'),
    base_url='https://api.wia-seaweed.org/v1'
)

# Get farm details
farm = api.farms.get('550e8400-e29b-41d4-a716-446655440000')

# Get growth data
growth = api.growth.get(farm.farm_id)
print(f"Total biomass: {growth.total_biomass} kg")

# Schedule harvest
harvest = api.harvests.schedule(
    farm_id=farm.farm_id,
    scheduled_date='2025-03-15',
    expected_yield=5000
)
```

---

**© 2025 WIA Standards | 弘益人間 · Benefit All Humanity**
