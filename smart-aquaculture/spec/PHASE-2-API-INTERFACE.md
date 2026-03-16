# WIA-AGRI-012: Smart Aquaculture Standard
## Phase 2 - API Interface Specification

**Version:** 1.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-01-01

---

## 1. Overview

This specification defines RESTful API endpoints, GraphQL schemas, and real-time streaming interfaces for smart aquaculture systems.

### 1.1 Base URL

```
Production: https://api.aqua.wiastandards.com/v1
Sandbox: https://sandbox-api.aqua.wiastandards.com/v1
```

### 1.2 Authentication

```http
Authorization: Bearer {JWT_TOKEN}
X-API-Key: {API_KEY}
X-Farm-ID: {FARM_UUID}
```

---

## 2. RESTful API Endpoints

### 2.1 Farm Management

#### GET /farms

List all farms for authenticated user.

**Response:**
```json
{
  "farms": [
    {
      "farmId": "farm-001",
      "farmName": "Blue Ocean Aquafarm",
      "location": {"latitude": 34.8547, "longitude": 128.4333},
      "species": ["flatfish", "seabass"],
      "status": "active"
    }
  ],
  "total": 1,
  "page": 1,
  "perPage": 20
}
```

#### GET /farms/{farmId}

Get detailed farm information.

#### POST /farms

Create new farm.

**Request:**
```json
{
  "farmName": "Green Waves Fishery",
  "farmType": "marine",
  "location": {
    "latitude": 34.8547,
    "longitude": 128.4333,
    "waterBody": "South Sea"
  },
  "capacity": {
    "totalVolume": 50000,
    "cageCount": 10
  }
}
```

#### PUT /farms/{farmId}

Update farm details.

#### DELETE /farms/{farmId}

Deactivate farm.

---

### 2.2 Water Quality Monitoring

#### GET /farms/{farmId}/water-quality

Get latest water quality readings.

**Query Parameters:**
- `tankId` (optional): Filter by specific tank
- `from` (ISO 8601): Start time
- `to` (ISO 8601): End time
- `parameters`: Comma-separated (temperature,do,ph)

**Response:**
```json
{
  "farmId": "farm-001",
  "readings": [
    {
      "timestamp": "2025-01-15T14:30:00Z",
      "tankId": "tank-A-03",
      "temperature": 18.5,
      "dissolvedOxygen": 7.8,
      "pH": 8.1,
      "salinity": 32.5,
      "ammonia": 0.047,
      "status": "normal"
    }
  ],
  "alerts": [
    {
      "severity": "warning",
      "parameter": "dissolvedOxygen",
      "tankId": "tank-B-01",
      "value": 5.2,
      "threshold": 6.0,
      "timestamp": "2025-01-15T14:28:00Z"
    }
  ]
}
```

#### POST /farms/{farmId}/water-quality

Submit water quality data (from sensors).

#### GET /farms/{farmId}/water-quality/trends

Get historical trends and statistics.

---

### 2.3 Fish Health & Biomass

#### GET /farms/{farmId}/batches

List all fish batches in farm.

**Response:**
```json
{
  "batches": [
    {
      "batchId": "batch-2024-flatfish-001",
      "species": "Paralichthys olivaceus",
      "commonName": "Flatfish (넙치)",
      "stockingDate": "2024-03-15",
      "daysInCulture": 306,
      "currentBiomass": 12500,
      "population": 50000,
      "averageWeight": 250,
      "mortalityRate": 2.3,
      "projectedHarvestDate": "2025-03-01"
    }
  ]
}
```

#### GET /farms/{farmId}/batches/{batchId}

Get detailed batch information.

#### POST /farms/{farmId}/batches

Create new batch (stocking event).

#### PUT /farms/{farmId}/batches/{batchId}/biomass

Update biomass estimation.

**Request:**
```json
{
  "timestamp": "2025-01-15T10:00:00Z",
  "samplingMethod": "seine_net",
  "sampleSize": 100,
  "averageWeight": 265,
  "totalBiomass": 13250,
  "weightDistribution": {
    "min": 180,
    "max": 350,
    "median": 260,
    "stdDev": 42
  }
}
```

#### GET /farms/{farmId}/health-alerts

Get disease and health alerts.

---

### 2.4 Feeding Management

#### GET /farms/{farmId}/feeding/schedule

Get feeding schedules for all tanks.

**Response:**
```json
{
  "schedules": [
    {
      "tankId": "tank-A-03",
      "frequency": 3,
      "times": ["08:00", "13:00", "18:00"],
      "dailyFeedingRate": 2.5,
      "feedType": "EP-3mm-45P",
      "autoFeeder": true
    }
  ]
}
```

#### POST /farms/{farmId}/feeding/schedule

Create/update feeding schedule.

#### GET /farms/{farmId}/feeding/events

Get feeding history.

**Query Parameters:**
- `from`, `to`: Date range
- `tankId`: Filter by tank

**Response:**
```json
{
  "events": [
    {
      "feedingEventId": "feed-evt-12345",
      "timestamp": "2025-01-15T08:00:00Z",
      "tankId": "tank-A-03",
      "feedAmount": 18.5,
      "feedType": "EP-3mm-45P",
      "feedingDuration": 15,
      "appetite": "excellent",
      "waste": 0.2
    }
  ],
  "performance": {
    "averageFCR": 1.18,
    "feedCostPerKg": 2.35,
    "totalFeedCost": 1250.50
  }
}
```

#### POST /farms/{farmId}/feeding/events

Log feeding event (manual or automated).

---

### 2.5 Harvest & Traceability

#### GET /farms/{farmId}/harvests

List harvest records.

**Response:**
```json
{
  "harvests": [
    {
      "harvestId": "harvest-2025-001",
      "harvestDate": "2025-01-10",
      "batchId": "batch-2024-flatfish-001",
      "totalWeight": 15000,
      "fishCount": 60000,
      "averageWeight": 250,
      "cumulativeFCR": 1.15,
      "survivalRate": 97.2,
      "revenue": 120000,
      "traceabilityQR": "https://trace.wia.com/harvest-2025-001"
    }
  ]
}
```

#### POST /farms/{farmId}/harvests

Create harvest record.

**Request:**
```json
{
  "batchId": "batch-2024-flatfish-001",
  "harvestDate": "2025-01-15",
  "tankId": "tank-A-03",
  "totalWeight": 15000,
  "fishCount": 60000,
  "sizeGrades": {
    "small": 2000,
    "medium": 8000,
    "large": 4000,
    "extraLarge": 1000
  },
  "quality": {
    "freshness": "A",
    "parasiteCheck": "pass"
  },
  "destination": "Seoul Fish Market"
}
```

#### GET /harvests/{harvestId}/traceability

Get full traceability data with W3C Verifiable Credential.

**Response:**
```json
{
  "harvestId": "harvest-2025-001",
  "qrCode": "base64_encoded_qr_image",
  "verifiableCredential": {
    "@context": ["https://www.w3.org/2018/credentials/v1"],
    "type": ["VerifiableCredential", "AquacultureTraceability"],
    "issuer": "did:wia:issuer:aquaculture",
    "credentialSubject": {
      "farmName": "Blue Ocean Aquafarm",
      "species": "Flatfish (넙치)",
      "harvestDate": "2025-01-10",
      "antibioticFree": true,
      "organicCertified": false,
      "waterQualityCompliant": true
    }
  },
  "blockchainTxId": "0x7f8d3c2a1b9e4f6d..."
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
  tanks: [Tank!]!
  batches: [FishBatch!]!
  waterQuality: [WaterQualityReading!]!
}

type Tank {
  tankId: ID!
  volume: Float!
  depth: Float!
  currentBatch: FishBatch
  waterQuality: WaterQualityReading!
  feedingSchedule: FeedingSchedule
}

type FishBatch {
  batchId: ID!
  species: Species!
  stockingDate: DateTime!
  daysInCulture: Int!
  biomass: Biomass!
  health: HealthMetrics!
  feeding: FeedingPerformance!
}

type WaterQualityReading {
  timestamp: DateTime!
  temperature: Float!
  dissolvedOxygen: Float!
  pH: Float!
  salinity: Float!
  ammonia: Float!
  status: WaterQualityStatus!
}

type Query {
  farm(farmId: ID!): Farm
  farms(limit: Int, offset: Int): [Farm!]!

  waterQuality(
    farmId: ID!
    tankId: ID
    from: DateTime!
    to: DateTime!
  ): [WaterQualityReading!]!

  batches(farmId: ID!): [FishBatch!]!

  harvestProjection(batchId: ID!): HarvestProjection!
}

type Mutation {
  createBatch(input: CreateBatchInput!): FishBatch!
  updateBiomass(batchId: ID!, biomass: BiomassInput!): FishBatch!
  logFeeding(input: FeedingEventInput!): FeedingEvent!
  recordHarvest(input: HarvestInput!): Harvest!
}

type Subscription {
  waterQualityUpdates(farmId: ID!, tankId: ID): WaterQualityReading!
  alerts(farmId: ID!): Alert!
}
```

### 3.2 Example Queries

#### Get Farm with Real-time Water Quality

```graphql
query GetFarmDashboard($farmId: ID!) {
  farm(farmId: $farmId) {
    farmName
    location {
      latitude
      longitude
    }
    tanks {
      tankId
      waterQuality {
        timestamp
        temperature
        dissolvedOxygen
        pH
        status
      }
      currentBatch {
        species {
          commonName
        }
        biomass {
          total
          averageWeight
        }
        health {
          mortalityRate
          feedingResponse
        }
      }
    }
  }
}
```

#### Subscribe to Real-time Alerts

```graphql
subscription FarmAlerts($farmId: ID!) {
  alerts(farmId: $farmId) {
    severity
    parameter
    tankId
    value
    threshold
    message
    timestamp
  }
}
```

---

## 4. WebSocket API (Real-time Streaming)

### 4.1 Connection

```javascript
const ws = new WebSocket('wss://stream.aqua.wiastandards.com/v1');

ws.onopen = () => {
  ws.send(JSON.stringify({
    action: 'authenticate',
    token: 'JWT_TOKEN',
    farmId: 'farm-001'
  }));
};
```

### 4.2 Subscribe to Water Quality Stream

```javascript
ws.send(JSON.stringify({
  action: 'subscribe',
  channel: 'water-quality',
  tankId: 'tank-A-03',
  parameters: ['temperature', 'do', 'ph']
}));

ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  console.log('Water quality update:', data);
  // {
  //   channel: 'water-quality',
  //   tankId: 'tank-A-03',
  //   timestamp: '2025-01-15T14:30:00Z',
  //   temperature: 18.5,
  //   dissolvedOxygen: 7.8,
  //   pH: 8.1
  // }
};
```

### 4.3 Subscribe to Alerts

```javascript
ws.send(JSON.stringify({
  action: 'subscribe',
  channel: 'alerts',
  severity: ['warning', 'critical']
}));
```

---

## 5. MQTT API (IoT Sensors)

### 5.1 Topic Structure

```
wia/aqua/{farmId}/{tankId}/water-quality
wia/aqua/{farmId}/{tankId}/feeding
wia/aqua/{farmId}/{tankId}/alerts
wia/aqua/{farmId}/commands/{deviceId}
```

### 5.2 Publish Water Quality Data

```javascript
const mqtt = require('mqtt');
const client = mqtt.connect('mqtts://mqtt.aqua.wiastandards.com', {
  username: 'farm-001',
  password: 'API_KEY'
});

client.publish('wia/aqua/farm-001/tank-A-03/water-quality', JSON.stringify({
  timestamp: new Date().toISOString(),
  temperature: 18.5,
  dissolvedOxygen: 7.8,
  pH: 8.1,
  salinity: 32.5
}));
```

### 5.3 Subscribe to Commands

```javascript
client.subscribe('wia/aqua/farm-001/commands/feeder-01');

client.on('message', (topic, message) => {
  const command = JSON.parse(message.toString());
  // {
  //   commandId: 'cmd-12345',
  //   action: 'feed',
  //   amount: 15.5,
  //   duration: 10
  // }
});
```

---

## 6. Korean Integration APIs

### 6.1 NIFS (국립수산과학원) Reporting

#### POST /kr/nifs/report

Submit monthly production report to National Institute of Fisheries Science.

**Request:**
```json
{
  "farmId": "farm-001",
  "reportMonth": "2025-01",
  "species": "넙치 (Flatfish)",
  "productionVolume": 15000,
  "mortalityRate": 2.3,
  "feedConversionRatio": 1.15,
  "antibioticUsage": {
    "used": false,
    "details": null
  },
  "waterQualitySummary": {
    "averageTemperature": 18.2,
    "averageDO": 7.5,
    "averagePH": 8.0
  }
}
```

### 6.2 Seafood Traceability (수산물이력제)

#### POST /kr/traceability/register

Register seafood batch in Korean traceability system.

**Request:**
```json
{
  "harvestId": "harvest-2025-001",
  "farmLicense": "KR-AQ-12345",
  "species": "넙치",
  "harvestDate": "2025-01-10",
  "quantity": 15000,
  "destination": "노량진 수산시장"
}
```

**Response:**
```json
{
  "traceabilityCode": "TR-2025-001-넙치-12345",
  "qrCode": "base64_encoded_image",
  "registrationDate": "2025-01-10T15:30:00+09:00",
  "status": "registered"
}
```

---

## 7. Third-Party Integrations

### 7.1 Weather API Integration

```javascript
GET /integrations/weather?lat=34.8547&lon=128.4333

Response:
{
  "current": {
    "temperature": 8.5,
    "windSpeed": 4.2,
    "waves": {"height": 1.2, "period": 5}
  },
  "forecast": [
    {"date": "2025-01-16", "tempMax": 10, "tempMin": 4, "rainfall": 0}
  ]
}
```

### 7.2 Market Price Integration

```javascript
GET /integrations/market-price?species=flatfish&market=seoul

Response:
{
  "species": "Flatfish (넙치)",
  "market": "Seoul Noryangjin Market",
  "prices": [
    {"size": "large", "pricePerKg": 8.5, "unit": "USD"},
    {"size": "medium", "pricePerKg": 7.2, "unit": "USD"}
  ],
  "trend": "stable",
  "lastUpdated": "2025-01-15T06:00:00Z"
}
```

---

## 8. Error Handling

### 8.1 HTTP Status Codes

- `200 OK`: Success
- `201 Created`: Resource created
- `400 Bad Request`: Invalid input
- `401 Unauthorized`: Authentication failed
- `403 Forbidden`: Insufficient permissions
- `404 Not Found`: Resource not found
- `429 Too Many Requests`: Rate limit exceeded
- `500 Internal Server Error`: Server error

### 8.2 Error Response Format

```json
{
  "error": {
    "code": "INVALID_WATER_QUALITY_DATA",
    "message": "Temperature value out of acceptable range",
    "details": {
      "parameter": "temperature",
      "value": -10,
      "minValue": 5,
      "maxValue": 35
    },
    "timestamp": "2025-01-15T14:30:00Z"
  }
}
```

---

## 9. Rate Limiting

- **Free Tier**: 1,000 requests/day
- **Basic**: 10,000 requests/day
- **Pro**: 100,000 requests/day
- **Enterprise**: Unlimited

Headers:
```
X-RateLimit-Limit: 10000
X-RateLimit-Remaining: 9547
X-RateLimit-Reset: 1642291200
```

---

## 10. SDK Examples

### 10.1 JavaScript/Node.js

```javascript
const WiaAquaculture = require('@wia/aquaculture-sdk');

const client = new WiaAquaculture({
  apiKey: 'YOUR_API_KEY',
  farmId: 'farm-001'
});

// Get water quality
const waterQuality = await client.getWaterQuality({
  tankId: 'tank-A-03',
  from: '2025-01-15T00:00:00Z',
  to: '2025-01-15T23:59:59Z'
});

// Log feeding event
const feeding = await client.logFeeding({
  tankId: 'tank-A-03',
  feedAmount: 18.5,
  feedType: 'EP-3mm-45P'
});

// Subscribe to real-time alerts
client.onAlert((alert) => {
  console.log('Alert:', alert);
});
```

### 10.2 Python

```python
from wia_aquaculture import AquacultureClient

client = AquacultureClient(
    api_key='YOUR_API_KEY',
    farm_id='farm-001'
)

# Get latest water quality
water_quality = client.get_water_quality(
    tank_id='tank-A-03'
)

# Calculate FCR
fcr = client.calculate_fcr(
    batch_id='batch-2024-flatfish-001'
)

print(f"Current FCR: {fcr.fcr:.2f}")
```

---

**Document Status**: ✅ Phase 2 Complete
**Next Phase**: [PHASE-3-PROTOCOL.md](./PHASE-3-PROTOCOL.md)

---

© 2025 WIA (World Certification Industry Association)
**License**: MIT
**Philosophy**: 弘益人間 (Benefit All Humanity)
