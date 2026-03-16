# WIA-AGRI-001: Smart Farm Standard
## Phase 2 - API Interface Specification

**Version:** 1.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-01-01

---

## 1. Overview

This specification defines REST and GraphQL APIs for smart farm data management, sensor control, automation, and analytics.

### 1.1 Base URL

```
Production:  https://api.wiastandards.com/v1/smart-farm
Staging:     https://staging-api.wiastandards.com/v1/smart-farm
Development: http://localhost:3000/v1/smart-farm
```

### 1.2 Authentication

All API requests require authentication using one of the following methods:

1. **API Key** (Header: `X-API-Key: your_api_key`)
2. **JWT Token** (Header: `Authorization: Bearer token`)
3. **OAuth 2.0** (For third-party integrations)

---

## 2. REST API Endpoints

### 2.1 Farm Management

#### GET /farms

Get list of farms owned by authenticated user.

**Query Parameters:**
- `page` (integer, default: 1)
- `limit` (integer, default: 20, max: 100)
- `sortBy` (string, options: name, area, createdAt)
- `order` (string, options: asc, desc)

**Response 200 OK:**
```json
{
  "data": [
    {
      "farmId": "123e4567-e89b-12d3-a456-426614174000",
      "farmName": "Green Valley Farm",
      "location": {
        "latitude": 37.5665,
        "longitude": 126.9780,
        "address": "Seoul, South Korea"
      },
      "area": {
        "total": 50000,
        "cultivated": 45000
      },
      "status": "active"
    }
  ],
  "pagination": {
    "currentPage": 1,
    "totalPages": 5,
    "totalItems": 100,
    "itemsPerPage": 20
  }
}
```

#### POST /farms

Create a new farm.

**Request Body:**
```json
{
  "farmName": "Green Valley Farm",
  "location": {
    "latitude": 37.5665,
    "longitude": 126.9780,
    "address": "Seoul, South Korea",
    "timezone": "Asia/Seoul"
  },
  "area": {
    "total": 50000,
    "cultivated": 45000,
    "greenhouse": 10000,
    "openField": 35000
  }
}
```

**Response 201 Created:**
```json
{
  "farmId": "123e4567-e89b-12d3-a456-426614174000",
  "farmName": "Green Valley Farm",
  "status": "active",
  "createdAt": "2025-01-01T10:00:00Z"
}
```

#### GET /farms/{farmId}

Get detailed information about a specific farm.

**Response 200 OK:**
```json
{
  "farmId": "123e4567-e89b-12d3-a456-426614174000",
  "farmName": "Green Valley Farm",
  "location": { ... },
  "area": { ... },
  "zones": [
    {
      "zoneId": "greenhouse-A",
      "zoneName": "Greenhouse A",
      "area": 5000,
      "cropType": "tomato",
      "sensorCount": 12
    }
  ],
  "statistics": {
    "totalSensors": 48,
    "activeSensors": 46,
    "totalCrops": 15000,
    "healthyCrops": 14500,
    "estimatedYield": 75000
  }
}
```

#### PUT /farms/{farmId}

Update farm information.

#### DELETE /farms/{farmId}

Delete a farm (soft delete).

---

### 2.2 Sensor Data

#### GET /farms/{farmId}/sensors

Get all sensors for a farm.

**Query Parameters:**
- `zoneId` (string, optional)
- `sensorType` (string, options: soil, air, water, light)
- `status` (string, options: active, inactive, maintenance)

**Response 200 OK:**
```json
{
  "data": [
    {
      "sensorId": "550e8400-e29b-41d4-a716-446655440001",
      "sensorType": "soil",
      "zoneId": "greenhouse-A",
      "status": "active",
      "lastReading": "2025-01-01T10:30:00Z",
      "batteryLevel": 85,
      "metadata": {
        "manufacturer": "FarmSense",
        "model": "FS-SOIL-PRO"
      }
    }
  ]
}
```

#### POST /farms/{farmId}/sensors

Register a new sensor.

**Request Body:**
```json
{
  "sensorType": "soil",
  "zoneId": "greenhouse-A",
  "position": {
    "latitude": 37.5665,
    "longitude": 126.9780,
    "depth": 0.15
  },
  "metadata": {
    "manufacturer": "FarmSense",
    "model": "FS-SOIL-PRO",
    "serialNumber": "FS-12345"
  }
}
```

#### GET /farms/{farmId}/sensors/{sensorId}/data

Get sensor readings (time series).

**Query Parameters:**
- `from` (ISO 8601 datetime, required)
- `to` (ISO 8601 datetime, required)
- `interval` (string, options: raw, 1min, 5min, 1hour, 1day)
- `metrics` (array of strings, e.g., ["soil.moisture", "soil.temperature"])

**Response 200 OK:**
```json
{
  "sensorId": "550e8400-e29b-41d4-a716-446655440001",
  "metric": "soil.moisture",
  "interval": "5min",
  "dataPoints": [
    {
      "timestamp": 1704096600000,
      "value": 68.5,
      "quality": "good"
    },
    {
      "timestamp": 1704096900000,
      "value": 68.2,
      "quality": "good"
    }
  ]
}
```

#### POST /farms/{farmId}/sensors/{sensorId}/data

Submit new sensor reading (for manual entry or IoT device).

**Request Body:**
```json
{
  "timestamp": "2025-01-01T10:30:00+09:00",
  "measurements": {
    "soil": {
      "moisture": 68.5,
      "temperature": 22.3,
      "pH": 6.4
    }
  }
}
```

---

### 2.3 Crop Management

#### GET /farms/{farmId}/crops

Get all crops in a farm.

**Query Parameters:**
- `zoneId` (string, optional)
- `cropType` (string, optional)
- `growthStage` (string, optional)
- `healthStatus` (string, optional)

**Response 200 OK:**
```json
{
  "data": [
    {
      "cropId": "crop-001",
      "cropType": "tomato",
      "variety": "Beefsteak",
      "zoneId": "greenhouse-A",
      "plantingDate": "2024-11-01",
      "expectedHarvestDate": "2025-02-15",
      "growthStage": "fruiting",
      "plantCount": 5000,
      "health": {
        "status": "healthy",
        "ndvi": 0.82
      }
    }
  ]
}
```

#### POST /farms/{farmId}/crops

Add new crop planting.

#### PUT /farms/{farmId}/crops/{cropId}

Update crop information.

#### GET /farms/{farmId}/crops/{cropId}/health

Get detailed health analysis.

**Response 200 OK:**
```json
{
  "cropId": "crop-001",
  "healthStatus": "healthy",
  "ndvi": 0.82,
  "chlorophyllContent": 42.5,
  "diseaseDetected": [],
  "pestDetected": [],
  "recommendations": [
    "Maintain current irrigation schedule",
    "Monitor for early blight in next 7 days"
  ],
  "aiConfidence": 0.94
}
```

---

### 2.4 Automation & Control

#### GET /farms/{farmId}/automation/events

Get automation event history.

**Query Parameters:**
- `from` (ISO 8601 datetime)
- `to` (ISO 8601 datetime)
- `eventType` (string, options: irrigation, fertilization, pesticide, hvac, lighting)
- `status` (string, options: success, partial, failed)

**Response 200 OK:**
```json
{
  "data": [
    {
      "eventId": "event-001",
      "eventType": "irrigation",
      "timestamp": "2025-01-01T06:00:00Z",
      "zoneId": "greenhouse-A",
      "duration": 15,
      "volume": 450,
      "status": "success",
      "energyConsumed": 0.75
    }
  ]
}
```

#### POST /farms/{farmId}/automation/irrigation

Trigger manual or scheduled irrigation.

**Request Body:**
```json
{
  "zoneId": "greenhouse-A",
  "duration": 15,
  "schedule": {
    "type": "immediate",
    "cronExpression": null
  },
  "waterSource": "rainwater"
}
```

**Response 202 Accepted:**
```json
{
  "eventId": "event-001",
  "status": "scheduled",
  "estimatedStartTime": "2025-01-01T06:00:00Z",
  "estimatedDuration": 15,
  "estimatedCost": 0.05
}
```

#### POST /farms/{farmId}/automation/fertilization

Apply fertilizer.

#### POST /farms/{farmId}/automation/hvac

Control HVAC system.

**Request Body:**
```json
{
  "zoneId": "greenhouse-A",
  "targetTemperature": 24,
  "targetHumidity": 65,
  "mode": "auto"
}
```

#### POST /farms/{farmId}/automation/lighting

Control lighting system.

---

### 2.5 Analytics & AI

#### GET /farms/{farmId}/analytics/yield-prediction

Get AI-powered yield prediction.

**Query Parameters:**
- `cropId` (string, required)
- `horizon` (integer, days ahead, default: 30)

**Response 200 OK:**
```json
{
  "cropId": "crop-001",
  "currentYield": 12000,
  "predictedYield": 15000,
  "confidence": 0.87,
  "factors": [
    {
      "factor": "Temperature",
      "impact": 0.15,
      "recommendation": "Maintain 22-26°C range"
    },
    {
      "factor": "Soil Moisture",
      "impact": 0.25,
      "recommendation": "Increase irrigation by 10%"
    }
  ],
  "harvestDate": "2025-02-15",
  "estimatedRevenue": 45000
}
```

#### GET /farms/{farmId}/analytics/resource-optimization

Get resource optimization recommendations.

**Response 200 OK:**
```json
{
  "water": {
    "currentUsage": 5000,
    "optimizedUsage": 4200,
    "savingsPotential": 16,
    "recommendations": [
      "Install drip irrigation in Zone B",
      "Adjust irrigation schedule based on weather forecast"
    ]
  },
  "energy": {
    "currentUsage": 1200,
    "optimizedUsage": 980,
    "savingsPotential": 18.3,
    "recommendations": [
      "Use LED grow lights instead of HPS",
      "Enable night-time temperature setback"
    ]
  },
  "fertilizer": {
    "currentUsage": 150,
    "optimizedUsage": 135,
    "savingsPotential": 10,
    "recommendations": [
      "Apply nitrogen based on soil test results",
      "Use slow-release fertilizers"
    ]
  }
}
```

#### POST /farms/{farmId}/analytics/disease-detection

Submit crop image for AI disease detection.

**Request Body (multipart/form-data):**
- `image` (file, required, max 10MB, formats: jpg, png)
- `cropType` (string, required)
- `zoneId` (string, optional)

**Response 200 OK:**
```json
{
  "detections": [
    {
      "disease": "early_blight",
      "confidence": 0.92,
      "severity": "moderate",
      "affectedArea": 15,
      "treatment": [
        "Apply copper-based fungicide",
        "Remove infected leaves",
        "Improve air circulation"
      ],
      "urgency": "medium"
    }
  ],
  "imageId": "img-001",
  "analyzedAt": "2025-01-01T10:30:00Z"
}
```

---

### 2.6 Alerts & Notifications

#### GET /farms/{farmId}/alerts

Get active and historical alerts.

**Query Parameters:**
- `status` (string, options: active, resolved, all)
- `severity` (string, options: critical, warning, info)
- `category` (string, options: weather, pest, disease, equipment, resource)

**Response 200 OK:**
```json
{
  "data": [
    {
      "alertId": "alert-001",
      "severity": "critical",
      "category": "weather",
      "title": "Frost Warning",
      "description": "Temperature expected to drop below 0°C in 6 hours",
      "affectedZones": ["greenhouse-A", "greenhouse-B"],
      "recommendations": [
        "Activate heating system",
        "Cover sensitive plants"
      ],
      "requiresAction": true,
      "actionDeadline": "2025-01-01T18:00:00Z",
      "createdAt": "2025-01-01T12:00:00Z"
    }
  ]
}
```

#### POST /farms/{farmId}/alerts/{alertId}/resolve

Mark alert as resolved.

---

### 2.7 Weather Integration

#### GET /farms/{farmId}/weather/current

Get current weather conditions.

**Response 200 OK:**
```json
{
  "temperature": 18.5,
  "humidity": 65,
  "windSpeed": 3.2,
  "windDirection": 180,
  "precipitation": 0,
  "cloudCover": 40,
  "uvIndex": 5,
  "timestamp": "2025-01-01T10:30:00Z"
}
```

#### GET /farms/{farmId}/weather/forecast

Get weather forecast (7 days).

**Response 200 OK:**
```json
{
  "forecast": [
    {
      "date": "2025-01-02",
      "temperatureMin": 12,
      "temperatureMax": 22,
      "precipitation": 0,
      "humidity": 60,
      "windSpeed": 2.5,
      "summary": "Partly cloudy"
    }
  ]
}
```

---

## 3. GraphQL API

### 3.1 Schema

```graphql
type Farm {
  farmId: ID!
  farmName: String!
  location: Location!
  area: Area!
  zones: [Zone!]!
  sensors: [Sensor!]!
  crops: [Crop!]!
  statistics: FarmStatistics!
}

type Sensor {
  sensorId: ID!
  sensorType: SensorType!
  zoneId: String!
  status: SensorStatus!
  lastReading: DateTime
  batteryLevel: Int
  data(from: DateTime!, to: DateTime!, interval: Interval): [DataPoint!]!
}

type Crop {
  cropId: ID!
  cropType: String!
  variety: String
  plantingDate: Date!
  expectedHarvestDate: Date
  growthStage: GrowthStage!
  health: CropHealth!
  yieldPrediction: YieldPrediction
}

type Query {
  farm(farmId: ID!): Farm
  farms(page: Int, limit: Int): FarmConnection!
  sensor(sensorId: ID!): Sensor
  crop(cropId: ID!): Crop
  alerts(farmId: ID!, status: AlertStatus): [Alert!]!
}

type Mutation {
  createFarm(input: CreateFarmInput!): Farm!
  updateFarm(farmId: ID!, input: UpdateFarmInput!): Farm!
  deleteFarm(farmId: ID!): Boolean!

  registerSensor(farmId: ID!, input: RegisterSensorInput!): Sensor!
  submitSensorData(sensorId: ID!, input: SensorDataInput!): Boolean!

  addCrop(farmId: ID!, input: AddCropInput!): Crop!
  updateCrop(cropId: ID!, input: UpdateCropInput!): Crop!

  triggerIrrigation(farmId: ID!, input: IrrigationInput!): AutomationEvent!
  triggerFertilization(farmId: ID!, input: FertilizationInput!): AutomationEvent!

  resolveAlert(alertId: ID!): Alert!
}

type Subscription {
  sensorDataUpdated(farmId: ID!, sensorId: ID): SensorData!
  alertCreated(farmId: ID!): Alert!
  automationEventCompleted(farmId: ID!): AutomationEvent!
}
```

### 3.2 Example Queries

```graphql
# Get farm with sensors and crops
query GetFarmDetails($farmId: ID!) {
  farm(farmId: $farmId) {
    farmId
    farmName
    location {
      latitude
      longitude
      address
    }
    zones {
      zoneId
      zoneName
      area
      cropType
    }
    sensors {
      sensorId
      sensorType
      status
      lastReading
      batteryLevel
    }
    crops {
      cropId
      cropType
      growthStage
      health {
        status
        ndvi
      }
    }
  }
}

# Get sensor data with time range
query GetSensorData($sensorId: ID!, $from: DateTime!, $to: DateTime!) {
  sensor(sensorId: $sensorId) {
    sensorId
    sensorType
    data(from: $from, to: $to, interval: FIVE_MIN) {
      timestamp
      value
      quality
    }
  }
}

# Subscribe to real-time sensor updates
subscription SensorUpdates($farmId: ID!) {
  sensorDataUpdated(farmId: $farmId) {
    sensorId
    timestamp
    measurements
  }
}
```

---

## 4. Webhooks

### 4.1 Configuration

```json
POST /webhooks
{
  "url": "https://your-app.com/webhooks/smart-farm",
  "events": ["sensor.data.new", "alert.created", "automation.completed"],
  "secret": "your_webhook_secret"
}
```

### 4.2 Event Payloads

**sensor.data.new**
```json
{
  "event": "sensor.data.new",
  "farmId": "123e4567-e89b-12d3-a456-426614174000",
  "sensorId": "550e8400-e29b-41d4-a716-446655440001",
  "timestamp": "2025-01-01T10:30:00Z",
  "data": { ... }
}
```

**alert.created**
```json
{
  "event": "alert.created",
  "farmId": "123e4567-e89b-12d3-a456-426614174000",
  "alertId": "alert-001",
  "severity": "critical",
  "category": "weather",
  "title": "Frost Warning",
  "timestamp": "2025-01-01T12:00:00Z"
}
```

---

## 5. Rate Limiting

| Tier | Requests/Hour | Burst | Price |
|------|---------------|-------|-------|
| Free | 1,000 | 100 | $0 |
| Basic | 10,000 | 500 | $29/mo |
| Pro | 100,000 | 2,000 | $99/mo |
| Enterprise | Unlimited | Custom | Custom |

**Headers:**
- `X-RateLimit-Limit`: Maximum requests per hour
- `X-RateLimit-Remaining`: Remaining requests
- `X-RateLimit-Reset`: Unix timestamp when limit resets

---

## 6. Error Handling

### 6.1 Error Response Format

```json
{
  "error": {
    "code": "SENSOR_NOT_FOUND",
    "message": "Sensor with ID 550e8400-e29b-41d4-a716-446655440001 not found",
    "details": {
      "sensorId": "550e8400-e29b-41d4-a716-446655440001"
    },
    "timestamp": "2025-01-01T10:30:00Z",
    "requestId": "req-12345"
  }
}
```

### 6.2 Error Codes

| HTTP Status | Error Code | Description |
|-------------|------------|-------------|
| 400 | INVALID_REQUEST | Malformed request body |
| 401 | UNAUTHORIZED | Missing or invalid authentication |
| 403 | FORBIDDEN | Insufficient permissions |
| 404 | RESOURCE_NOT_FOUND | Farm, sensor, or crop not found |
| 409 | CONFLICT | Duplicate resource |
| 429 | RATE_LIMIT_EXCEEDED | Too many requests |
| 500 | INTERNAL_ERROR | Server error |
| 503 | SERVICE_UNAVAILABLE | Temporary unavailability |

---

## 7. SDK Support

### 7.1 Official SDKs

- **JavaScript/TypeScript**: `npm install @wia/smart-farm-sdk`
- **Python**: `pip install wia-smart-farm`
- **Java**: `maven dependency: com.wia:smart-farm-sdk`
- **Go**: `go get github.com/wia-official/smart-farm-go`

### 7.2 Example Usage (JavaScript)

```javascript
import { SmartFarmClient } from '@wia/smart-farm-sdk';

const client = new SmartFarmClient({
  apiKey: 'your_api_key',
  environment: 'production'
});

// Get farm details
const farm = await client.farms.get('farm-id');

// Get sensor data
const sensorData = await client.sensors.getData({
  sensorId: 'sensor-id',
  from: new Date('2025-01-01'),
  to: new Date('2025-01-02'),
  interval: '5min'
});

// Trigger irrigation
const event = await client.automation.irrigate({
  farmId: 'farm-id',
  zoneId: 'greenhouse-A',
  duration: 15
});
```

---

## 8. Implementation Checklist

- [ ] Set up REST API server (Express, FastAPI, Spring Boot)
- [ ] Implement authentication (JWT, OAuth 2.0)
- [ ] Create database schema (PostgreSQL, MongoDB)
- [ ] Implement rate limiting (Redis)
- [ ] Set up GraphQL server (Apollo Server, GraphQL Yoga)
- [ ] Configure webhooks
- [ ] Write API documentation (OpenAPI/Swagger)
- [ ] Develop SDKs
- [ ] Set up monitoring (Datadog, New Relic)
- [ ] Implement caching (Redis, Memcached)

---

**Previous Phase:** [PHASE-1-DATA-FORMAT.md](./PHASE-1-DATA-FORMAT.md)
**Next Phase:** [PHASE-3-PROTOCOL.md](./PHASE-3-PROTOCOL.md)

---

© 2025 WIA (World Certification Industry Association)
弘益人間 · Benefit All Humanity
