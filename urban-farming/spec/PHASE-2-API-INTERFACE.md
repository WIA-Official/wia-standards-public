# WIA-AGRI-031: Urban Farming Standard
## Phase 2: API Interface Specification

**Version:** 1.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-01-01

---

## 1. Overview

This specification defines RESTful API interfaces for urban farming systems, enabling farm management, community coordination, data sharing, and integration with city services.

### 1.1 Design Principles

- **RESTful**: Standard HTTP methods and status codes
- **Community-Friendly**: Simple enough for community members to use
- **Comprehensive**: Support all urban farming operations
- **Secure**: Authentication and authorization for sensitive data
- **Scalable**: Handle from single gardens to city-wide networks

### 1.2 Base URL Structure

```
https://api.urbanfarming.example.com/v1
```

---

## 2. Authentication

### 2.1 API Key Authentication

```http
GET /api/v1/farms
Authorization: Bearer YOUR_API_KEY
```

### 2.2 OAuth 2.0 (Recommended)

```http
POST /api/v1/oauth/token
Content-Type: application/json

{
  "grant_type": "password",
  "username": "gardener@example.com",
  "password": "secure_password",
  "client_id": "urban_farm_app"
}
```

**Response:**
```json
{
  "access_token": "eyJhbGciOiJIUzI1NiIs...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "refresh_token": "eyJhbGciOiJIUzI1NiIs..."
}
```

---

## 3. Farm Management APIs

### 3.1 List All Farms

**Endpoint:** `GET /api/v1/farms`

**Query Parameters:**
- `city` (optional): Filter by city
- `type` (optional): Filter by farm type
- `page` (optional): Page number (default: 1)
- `limit` (optional): Results per page (default: 20)

**Request:**
```http
GET /api/v1/farms?city=Brooklyn&type=COMMUNITY&page=1&limit=10
Authorization: Bearer YOUR_API_KEY
```

**Response (200 OK):**
```json
{
  "data": [
    {
      "farmId": "UF-NYC-001",
      "farmName": "Brooklyn Community Garden",
      "farmType": "COMMUNITY",
      "location": {
        "address": "123 Brooklyn Ave, Brooklyn, NY",
        "coordinates": {
          "lat": 40.6782,
          "lng": -73.9442
        }
      },
      "area": 150,
      "plots": 12,
      "status": "ACTIVE",
      "memberCount": 24
    }
  ],
  "pagination": {
    "page": 1,
    "limit": 10,
    "total": 45,
    "totalPages": 5
  }
}
```

### 3.2 Get Farm Details

**Endpoint:** `GET /api/v1/farms/{farmId}`

**Request:**
```http
GET /api/v1/farms/UF-NYC-001
Authorization: Bearer YOUR_API_KEY
```

**Response (200 OK):**
```json
{
  "farmId": "UF-NYC-001",
  "farmName": "Brooklyn Community Garden",
  "farmType": "COMMUNITY",
  "location": {
    "address": "123 Brooklyn Ave, Brooklyn, NY 11201",
    "coordinates": {
      "lat": 40.6782,
      "lng": -73.9442
    },
    "timezone": "America/New_York"
  },
  "infrastructure": {
    "totalArea": 150,
    "numberOfPlots": 12,
    "waterSource": "MUNICIPAL",
    "irrigationType": "DRIP"
  },
  "features": {
    "composting": true,
    "rainwaterHarvesting": true,
    "solarPower": false
  },
  "community": {
    "totalMembers": 24,
    "activeGardeners": 18,
    "waitlist": 15
  },
  "operationalSince": "2020-04-15",
  "status": "ACTIVE"
}
```

### 3.3 Create New Farm

**Endpoint:** `POST /api/v1/farms`

**Request:**
```http
POST /api/v1/farms
Authorization: Bearer YOUR_API_KEY
Content-Type: application/json

{
  "farmName": "Manhattan Rooftop Garden",
  "farmType": "ROOFTOP",
  "location": {
    "address": "456 5th Ave, Manhattan, NY 10018",
    "coordinates": {
      "lat": 40.7589,
      "lng": -73.9851
    }
  },
  "infrastructure": {
    "totalArea": 80,
    "numberOfPlots": 8,
    "waterSource": "RAINWATER",
    "irrigationType": "DRIP"
  },
  "features": {
    "composting": true,
    "rainwaterHarvesting": true,
    "solarPower": true
  }
}
```

**Response (201 Created):**
```json
{
  "farmId": "UF-NYC-045",
  "message": "Farm created successfully",
  "createdAt": "2025-06-15T14:30:00Z"
}
```

### 3.4 Update Farm

**Endpoint:** `PATCH /api/v1/farms/{farmId}`

**Request:**
```http
PATCH /api/v1/farms/UF-NYC-001
Authorization: Bearer YOUR_API_KEY
Content-Type: application/json

{
  "features": {
    "solarPower": true
  },
  "infrastructure": {
    "numberOfPlots": 14
  }
}
```

**Response (200 OK):**
```json
{
  "farmId": "UF-NYC-001",
  "message": "Farm updated successfully",
  "updatedAt": "2025-06-15T14:35:00Z"
}
```

### 3.5 Delete Farm

**Endpoint:** `DELETE /api/v1/farms/{farmId}`

**Request:**
```http
DELETE /api/v1/farms/UF-NYC-001
Authorization: Bearer YOUR_API_KEY
```

**Response (200 OK):**
```json
{
  "message": "Farm deleted successfully",
  "farmId": "UF-NYC-001"
}
```

---

## 4. Plot Management APIs

### 4.1 Get Plots for Farm

**Endpoint:** `GET /api/v1/plots/{farmId}`

**Request:**
```http
GET /api/v1/plots/UF-NYC-001
Authorization: Bearer YOUR_API_KEY
```

**Response (200 OK):**
```json
{
  "farmId": "UF-NYC-001",
  "plots": [
    {
      "plotId": "UF-NYC-001-P01",
      "plotNumber": 1,
      "area": 12.0,
      "status": "OCCUPIED",
      "gardener": {
        "gardenerId": "G-2345",
        "name": "Jane Smith",
        "email": "jane@example.com"
      },
      "currentCrops": ["Tomatoes", "Basil"]
    },
    {
      "plotId": "UF-NYC-001-P02",
      "plotNumber": 2,
      "area": 12.0,
      "status": "AVAILABLE"
    }
  ],
  "total": 12,
  "occupied": 8,
  "available": 4
}
```

### 4.2 Assign Plot

**Endpoint:** `POST /api/v1/plots/{farmId}/assign`

**Request:**
```http
POST /api/v1/plots/UF-NYC-001/assign
Authorization: Bearer YOUR_API_KEY
Content-Type: application/json

{
  "plotNumber": 5,
  "gardenerId": "G-3456",
  "startDate": "2025-06-20",
  "terms": {
    "duration": "1_YEAR",
    "fee": 50.00,
    "currency": "USD"
  }
}
```

**Response (200 OK):**
```json
{
  "plotId": "UF-NYC-001-P05",
  "message": "Plot assigned successfully",
  "assignedTo": "G-3456",
  "assignedAt": "2025-06-15T15:00:00Z"
}
```

---

## 5. Harvest Recording APIs

### 5.1 Record Harvest

**Endpoint:** `POST /api/v1/harvest`

**Request:**
```http
POST /api/v1/harvest
Authorization: Bearer YOUR_API_KEY
Content-Type: application/json

{
  "farmId": "UF-NYC-001",
  "plotId": "UF-NYC-001-P01",
  "gardenerId": "G-2345",
  "crop": {
    "cropName": "Tomatoes",
    "variety": "Heirloom Beefsteak"
  },
  "yield": {
    "quantity": 12.5,
    "unit": "kg",
    "quality": "GRADE_A"
  },
  "distribution": {
    "personalUse": 8.0,
    "donated": 2.5,
    "sold": 2.0
  },
  "harvestDate": "2025-07-20"
}
```

**Response (201 Created):**
```json
{
  "harvestId": "HRV-2025-001234",
  "message": "Harvest recorded successfully",
  "recordedAt": "2025-07-20T16:30:00Z"
}
```

### 5.2 Get Harvest History

**Endpoint:** `GET /api/v1/harvest/{farmId}`

**Query Parameters:**
- `startDate` (optional): Filter from date
- `endDate` (optional): Filter to date
- `plotId` (optional): Filter by plot
- `crop` (optional): Filter by crop name

**Request:**
```http
GET /api/v1/harvest/UF-NYC-001?startDate=2025-06-01&endDate=2025-07-31
Authorization: Bearer YOUR_API_KEY
```

**Response (200 OK):**
```json
{
  "farmId": "UF-NYC-001",
  "period": {
    "startDate": "2025-06-01",
    "endDate": "2025-07-31"
  },
  "harvests": [
    {
      "harvestId": "HRV-2025-001234",
      "plotId": "UF-NYC-001-P01",
      "crop": "Tomatoes",
      "quantity": 12.5,
      "unit": "kg",
      "harvestDate": "2025-07-20"
    }
  ],
  "summary": {
    "totalHarvests": 45,
    "totalYield": 234.5,
    "unit": "kg",
    "crops": 12
  }
}
```

---

## 6. Environmental Monitoring APIs

### 6.1 Get Current Conditions

**Endpoint:** `GET /api/v1/environment/{farmId}`

**Request:**
```http
GET /api/v1/environment/UF-NYC-001
Authorization: Bearer YOUR_API_KEY
```

**Response (200 OK):**
```json
{
  "farmId": "UF-NYC-001",
  "timestamp": "2025-06-15T12:00:00Z",
  "weather": {
    "temperature": 24.5,
    "humidity": 62,
    "windSpeed": 12,
    "precipitation": 2.5
  },
  "soil": {
    "moisture": 68,
    "temperature": 22.0,
    "status": "OPTIMAL"
  },
  "airQuality": {
    "pm25": 12,
    "quality": "GOOD"
  }
}
```

### 6.2 Post Sensor Data

**Endpoint:** `POST /api/v1/environment/{farmId}/sensor`

**Request:**
```http
POST /api/v1/environment/UF-NYC-001/sensor
Authorization: Bearer YOUR_API_KEY
Content-Type: application/json

{
  "sensorId": "SENSOR-001",
  "location": "PLOT_AREA_A",
  "readings": {
    "soilMoisture": 68,
    "soilTemperature": 22.0,
    "airTemperature": 24.5,
    "humidity": 62
  },
  "timestamp": "2025-06-15T12:00:00Z"
}
```

**Response (201 Created):**
```json
{
  "message": "Sensor data recorded successfully",
  "dataId": "ENV-001234"
}
```

---

## 7. Community Management APIs

### 7.1 Get Member List

**Endpoint:** `GET /api/v1/members/{farmId}`

**Request:**
```http
GET /api/v1/members/UF-NYC-001
Authorization: Bearer YOUR_API_KEY
```

**Response (200 OK):**
```json
{
  "farmId": "UF-NYC-001",
  "members": [
    {
      "gardenerId": "G-2345",
      "name": "Jane Smith",
      "email": "jane@example.com",
      "memberSince": "2022-03-15",
      "membershipType": "FULL_MEMBER",
      "plotsAssigned": ["UF-NYC-001-P01"],
      "volunteerHours": 45,
      "status": "ACTIVE"
    }
  ],
  "total": 24,
  "active": 18
}
```

### 7.2 Add Member

**Endpoint:** `POST /api/v1/members/{farmId}`

**Request:**
```http
POST /api/v1/members/UF-NYC-001
Authorization: Bearer YOUR_API_KEY
Content-Type: application/json

{
  "name": "John Doe",
  "email": "john@example.com",
  "phone": "+1-555-0123",
  "membershipType": "FULL_MEMBER",
  "startDate": "2025-06-20"
}
```

**Response (201 Created):**
```json
{
  "gardenerId": "G-3456",
  "message": "Member added successfully",
  "createdAt": "2025-06-15T15:30:00Z"
}
```

---

## 8. Analytics APIs

### 8.1 Get Farm Analytics

**Endpoint:** `GET /api/v1/analytics/{farmId}`

**Query Parameters:**
- `period` (optional): MONTH, QUARTER, YEAR (default: MONTH)

**Request:**
```http
GET /api/v1/analytics/UF-NYC-001?period=QUARTER
Authorization: Bearer YOUR_API_KEY
```

**Response (200 OK):**
```json
{
  "farmId": "UF-NYC-001",
  "period": "2025-Q2",
  "productivity": {
    "totalYield": 234.5,
    "unit": "kg",
    "yieldPerSqMeter": 1.56,
    "crops": 12
  },
  "waterEfficiency": {
    "totalUsed": 4500,
    "perKg": 19.2,
    "savedVsConventional": 2100
  },
  "communityImpact": {
    "mealsProvided": 780,
    "foodDonated": 78.5,
    "volunteerHours": 320
  },
  "sustainability": {
    "co2Saved": 145.5,
    "wasteComposted": 185,
    "biodiversityScore": 82
  }
}
```

### 8.2 Get City-Wide Statistics

**Endpoint:** `GET /api/v1/analytics/city/{cityName}`

**Request:**
```http
GET /api/v1/analytics/city/Brooklyn
Authorization: Bearer YOUR_API_KEY
```

**Response (200 OK):**
```json
{
  "city": "Brooklyn",
  "totalFarms": 45,
  "totalArea": 6750,
  "activeGardeners": 810,
  "quarterlyProduction": {
    "totalYield": 10575,
    "mealsProvided": 35250,
    "foodDonated": 3530
  },
  "environmentalImpact": {
    "co2Saved": 6547,
    "waterSaved": 189000,
    "greenSpaceCreated": 6750
  }
}
```

---

## 9. Error Handling

### 9.1 Standard Error Response

```json
{
  "error": {
    "code": "FARM_NOT_FOUND",
    "message": "Farm with ID UF-NYC-999 does not exist",
    "statusCode": 404,
    "timestamp": "2025-06-15T12:00:00Z"
  }
}
```

### 9.2 Common Error Codes

| Status Code | Error Code | Description |
|-------------|------------|-------------|
| 400 | INVALID_REQUEST | Malformed request body |
| 401 | UNAUTHORIZED | Missing or invalid authentication |
| 403 | FORBIDDEN | Insufficient permissions |
| 404 | RESOURCE_NOT_FOUND | Requested resource doesn't exist |
| 409 | CONFLICT | Resource already exists |
| 422 | VALIDATION_ERROR | Request validation failed |
| 429 | RATE_LIMIT_EXCEEDED | Too many requests |
| 500 | INTERNAL_ERROR | Server error |

---

## 10. Rate Limiting

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 987
X-RateLimit-Reset: 1640995200
```

- **Free Tier**: 1,000 requests/hour
- **Community Tier**: 5,000 requests/hour
- **City Tier**: 50,000 requests/hour

---

## 11. Webhooks

### 11.1 Register Webhook

**Endpoint:** `POST /api/v1/webhooks`

**Request:**
```http
POST /api/v1/webhooks
Authorization: Bearer YOUR_API_KEY
Content-Type: application/json

{
  "url": "https://your-app.com/webhook",
  "events": ["harvest.recorded", "plot.assigned", "member.added"],
  "farmId": "UF-NYC-001"
}
```

**Response (201 Created):**
```json
{
  "webhookId": "WH-001",
  "message": "Webhook registered successfully"
}
```

### 11.2 Webhook Payload Example

```json
{
  "event": "harvest.recorded",
  "farmId": "UF-NYC-001",
  "timestamp": "2025-07-20T16:30:00Z",
  "data": {
    "harvestId": "HRV-2025-001234",
    "crop": "Tomatoes",
    "quantity": 12.5,
    "unit": "kg"
  }
}
```

---

## 12. SDK Examples

### 12.1 JavaScript/TypeScript

```typescript
import { UrbanFarmingAPI } from '@wia/urban-farming';

const client = new UrbanFarmingAPI({
  apiKey: 'YOUR_API_KEY',
  baseUrl: 'https://api.urbanfarming.example.com/v1'
});

// Get farm details
const farm = await client.farms.get('UF-NYC-001');

// Record harvest
const harvest = await client.harvest.record({
  farmId: 'UF-NYC-001',
  plotId: 'UF-NYC-001-P01',
  crop: 'Tomatoes',
  quantity: 12.5
});
```

### 12.2 Python

```python
from wia_urban_farming import UrbanFarmingClient

client = UrbanFarmingClient(
    api_key='YOUR_API_KEY',
    base_url='https://api.urbanfarming.example.com/v1'
)

# Get farm details
farm = client.farms.get('UF-NYC-001')

# Record harvest
harvest = client.harvest.record(
    farm_id='UF-NYC-001',
    plot_id='UF-NYC-001-P01',
    crop='Tomatoes',
    quantity=12.5
)
```

---

**Previous Phase:** [PHASE-1-DATA-FORMAT.md](PHASE-1-DATA-FORMAT.md)
**Next Phase:** [PHASE-3-PROTOCOL.md](PHASE-3-PROTOCOL.md)

© 2025 WIA - World Certification Industry Association
弘益人間 · Benefit All Humanity
