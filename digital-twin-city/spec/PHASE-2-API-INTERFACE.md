# WIA Digital Twin City API Interface Standard
## Phase 2 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #0EA5E9 (Sky)

---

## Table of Contents

1. [Overview](#overview)
2. [Terminology](#terminology)
3. [Core Interfaces](#core-interfaces)
4. [API Endpoints](#api-endpoints)
5. [City Object Management](#city-object-management)
6. [Sensor Data Services](#sensor-data-services)
7. [Simulation Services](#simulation-services)
8. [Authentication](#authentication)
9. [Error Handling](#error-handling)
10. [Usage Examples](#usage-examples)
11. [References](#references)

---

## Overview

### 1.1 Purpose

The WIA Digital Twin City API Interface Standard defines a comprehensive programmatic interface for managing digital twin city platforms, including 3D city models, real-time IoT sensor data, simulations, and citizen services. This Phase 2 specification builds upon the Phase 1 Data Format, providing developers with standardized APIs for smart city operations.

**Core Objectives**:
- Provide unified API for all digital twin city operations
- Enable real-time sensor data streaming and queries
- Support simulation execution and result retrieval
- Facilitate integration with GIS systems and IoT platforms
- Enable citizen service applications and dashboards

### 1.2 Scope

This standard defines:

| Component | Description |
|-----------|-------------|
| **Core API** | Main DigitalTwinCity class interface |
| **REST Endpoints** | HTTP API for city operations |
| **WebSocket API** | Real-time sensor data streaming |
| **Event System** | City event notifications |
| **SDKs** | TypeScript and Python client libraries |

### 1.3 Phase 1 Compatibility

Phase 2 API is fully compatible with Phase 1 Data Format:

```
Phase 1: Data Format (JSON structure)
    ↓
Phase 2: API Interface (programming interface)
    ↓
Phase 3: Protocol (communication protocol)
    ↓
Phase 4: Integration (ecosystem integration)
```

---

## Terminology

### 2.1 Core Terms

| Term | Definition |
|------|------------|
| **DigitalTwinCity** | Main API class for city operations |
| **CityObject** | Physical entity in the city (building, road, sensor) |
| **SensorNode** | IoT device collecting real-time data |
| **Simulation** | Computational model for predictions |
| **CityService** | Public service integration (transport, waste, emergency) |
| **TemporalQuery** | Time-based data query |

### 2.2 Event Types

| Event | Description | Data Payload |
|-------|-------------|--------------|
| `city:updated` | City model updated | CityUpdateEvent |
| `sensor:data` | New sensor reading | SensorData |
| `simulation:completed` | Simulation finished | SimulationResult |
| `alert:created` | Alert triggered | AlertEvent |
| `service:status` | Service status changed | ServiceStatus |
| `error` | Error occurred | ErrorDetails |

---

## Core Interfaces

### 3.1 DigitalTwinCity Class

Main API entry point for digital twin city operations.

#### TypeScript

```typescript
class DigitalTwinCity {
  // Constructor
  constructor(options?: DigitalTwinCityOptions);

  // City Model Management
  getCityInfo(): Promise<CityInfo>;
  updateCityInfo(updates: Partial<CityInfo>): Promise<CityInfo>;
  getBounds(): Promise<GeoBounds>;
  getStatistics(): Promise<CityStatistics>;

  // City Object Operations
  createCityObject(data: CityObjectData): Promise<CityObject>;
  getCityObject(id: string): Promise<CityObject | null>;
  updateCityObject(id: string, updates: Partial<CityObject>): Promise<CityObject>;
  deleteCityObject(id: string): Promise<void>;
  queryCityObjects(query: CityObjectQuery): Promise<CityObject[]>;
  getCityObjectsInBounds(bounds: GeoBounds): Promise<CityObject[]>;

  // Sensor Operations
  registerSensor(sensor: SensorData): Promise<SensorNode>;
  getSensor(id: string): Promise<SensorNode | null>;
  updateSensor(id: string, updates: Partial<SensorNode>): Promise<SensorNode>;
  getSensorData(id: string, timeRange?: TimeRange): Promise<SensorMeasurement[]>;
  streamSensorData(ids: string[], handler: SensorDataHandler): StreamSubscription;
  querySensors(query: SensorQuery): Promise<SensorNode[]>;

  // Simulation Operations
  createSimulation(config: SimulationConfig): Promise<Simulation>;
  runSimulation(id: string): Promise<SimulationResult>;
  getSimulation(id: string): Promise<Simulation | null>;
  getSimulationResult(id: string): Promise<SimulationResult | null>;
  cancelSimulation(id: string): Promise<void>;

  // Service Integration
  getServices(): Promise<CityService[]>;
  getService(id: string): Promise<CityService | null>;
  updateServiceStatus(id: string, status: ServiceStatus): Promise<void>;
  queryServices(query: ServiceQuery): Promise<CityService[]>;

  // Spatial Queries
  findNearby(location: GeoPoint, radius: number, type?: string): Promise<CityObject[]>;
  getRoute(start: GeoPoint, end: GeoPoint, mode?: TransportMode): Promise<Route>;
  analyzeAccessibility(location: GeoPoint, criteria: AccessibilityCriteria): Promise<AccessibilityResult>;

  // Temporal Operations
  getHistoricalData(objectId: string, timeRange: TimeRange): Promise<HistoricalData>;
  getSnapshot(timestamp: Date): Promise<CitySnapshot>;
  compareSnapshots(time1: Date, time2: Date): Promise<SnapshotDiff>;

  // Event Handling
  on<T extends EventType>(event: T, handler: EventHandler<T>): void;
  off<T extends EventType>(event: T, handler: EventHandler<T>): void;
  once<T extends EventType>(event: T, handler: EventHandler<T>): void;

  // Export/Import
  exportCity(format: ExportFormat): Promise<Buffer>;
  importCityData(data: Buffer, format: ImportFormat): Promise<ImportResult>;
}
```

#### Python

```python
class DigitalTwinCity:
    def __init__(self, options: Optional[DigitalTwinCityOptions] = None):
        ...

    # City Model Management
    async def get_city_info(self) -> CityInfo: ...
    async def update_city_info(self, updates: dict) -> CityInfo: ...
    async def get_bounds(self) -> GeoBounds: ...
    async def get_statistics(self) -> CityStatistics: ...

    # City Object Operations
    async def create_city_object(self, data: CityObjectData) -> CityObject: ...
    async def get_city_object(self, id: str) -> Optional[CityObject]: ...
    async def update_city_object(self, id: str, updates: dict) -> CityObject: ...
    async def delete_city_object(self, id: str) -> None: ...
    async def query_city_objects(self, query: CityObjectQuery) -> List[CityObject]: ...
    async def get_city_objects_in_bounds(self, bounds: GeoBounds) -> List[CityObject]: ...

    # Sensor Operations
    async def register_sensor(self, sensor: SensorData) -> SensorNode: ...
    async def get_sensor(self, id: str) -> Optional[SensorNode]: ...
    async def update_sensor(self, id: str, updates: dict) -> SensorNode: ...
    async def get_sensor_data(self, id: str, time_range: Optional[TimeRange] = None) -> List[SensorMeasurement]: ...
    def stream_sensor_data(self, ids: List[str], handler: SensorDataHandler) -> StreamSubscription: ...
    async def query_sensors(self, query: SensorQuery) -> List[SensorNode]: ...

    # Simulation Operations
    async def create_simulation(self, config: SimulationConfig) -> Simulation: ...
    async def run_simulation(self, id: str) -> SimulationResult: ...
    async def get_simulation(self, id: str) -> Optional[Simulation]: ...
    async def get_simulation_result(self, id: str) -> Optional[SimulationResult]: ...
    async def cancel_simulation(self, id: str) -> None: ...

    # Service Integration
    async def get_services(self) -> List[CityService]: ...
    async def get_service(self, id: str) -> Optional[CityService]: ...
    async def update_service_status(self, id: str, status: ServiceStatus) -> None: ...

    # Spatial Queries
    async def find_nearby(self, location: GeoPoint, radius: float, type: Optional[str] = None) -> List[CityObject]: ...
    async def get_route(self, start: GeoPoint, end: GeoPoint, mode: Optional[TransportMode] = None) -> Route: ...

    # Event Handling
    def on(self, event: EventType, handler: EventHandler) -> None: ...
    def off(self, event: EventType, handler: EventHandler) -> None: ...
```

### 3.2 DigitalTwinCityOptions

```typescript
interface DigitalTwinCityOptions {
  // API Configuration
  apiEndpoint?: string;
  apiKey?: string;
  oauth?: {
    clientId: string;
    clientSecret: string;
    tokenUrl: string;
  };

  // City Configuration
  cityId: string;
  defaultBounds?: GeoBounds;

  // Real-time Configuration
  enableWebSocket?: boolean;
  wsEndpoint?: string;
  reconnectInterval?: number;

  // Cache Configuration
  cacheEnabled?: boolean;
  cacheTTL?: number;

  // Logging
  logLevel?: 'debug' | 'info' | 'warn' | 'error' | 'none';
}
```

---

## API Endpoints

### 4.1 REST API Overview

All endpoints follow RESTful conventions with JSON payloads.

**Base URL**: `https://api.wia.live/digital-twin-city/v1`

**Authentication**:
- API Key: `X-API-Key: <key>`
- OAuth 2.0: `Authorization: Bearer <token>`

**Content-Type**: `application/json`

### 4.2 City Information Endpoints

#### GET /cities/:cityId

Retrieve city information.

```http
GET /cities/KR-SEOUL-2025
X-API-Key: your-api-key
```

**Response** (200 OK):
```json
{
  "cityId": "KR-SEOUL-2025",
  "name": "Seoul Digital Twin",
  "bounds": {
    "type": "Polygon",
    "coordinates": [[[126.764, 37.413], [127.184, 37.413], [127.184, 37.701], [126.764, 37.701], [126.764, 37.413]]]
  },
  "coordinateSystem": {
    "type": "WGS84",
    "epsg": 4326
  },
  "statistics": {
    "totalObjects": 125000,
    "buildings": 85000,
    "roads": 15000,
    "sensors": 5000
  },
  "lastUpdated": "2025-01-15T10:30:00+09:00"
}
```

#### PATCH /cities/:cityId

Update city information.

```http
PATCH /cities/KR-SEOUL-2025
X-API-Key: your-api-key
Content-Type: application/json

{
  "metadata": {
    "description": "Seoul Metropolitan Digital Twin Platform",
    "updateFrequency": "1min"
  }
}
```

#### GET /cities/:cityId/statistics

Get detailed city statistics.

```http
GET /cities/KR-SEOUL-2025/statistics
X-API-Key: your-api-key
```

**Response**:
```json
{
  "cityObjects": {
    "total": 125000,
    "byType": {
      "Building": 85000,
      "Road": 15000,
      "Bridge": 500,
      "Park": 2000,
      "Utility": 10000,
      "Landmark": 500
    }
  },
  "sensors": {
    "total": 5000,
    "operational": 4850,
    "offline": 150,
    "byType": {
      "EnvironmentalSensor": 2000,
      "TrafficSensor": 1500,
      "EnergySensor": 1000,
      "OccupancySensor": 500
    }
  },
  "dataQuality": {
    "overall": 0.96,
    "geometryAccuracy": 0.98,
    "sensorReliability": 0.94
  }
}
```

### 4.3 City Object Endpoints

#### POST /cities/:cityId/objects

Create a new city object.

```http
POST /cities/KR-SEOUL-2025/objects
X-API-Key: your-api-key
Content-Type: application/json

{
  "type": "Building",
  "geometry": {
    "type": "Polygon",
    "coordinates": [[[127.0276, 37.4979], [127.0277, 37.4979], [127.0277, 37.4980], [127.0276, 37.4980], [127.0276, 37.4979]]]
  },
  "properties": {
    "name": "New Office Tower",
    "address": "123 Gangnam-daero, Gangnam-gu, Seoul",
    "buildingType": "commercial",
    "floors": {
      "above": 20,
      "below": 3
    },
    "height": 80.0
  }
}
```

**Response** (201 Created):
```json
{
  "id": "building-gangnam-1523",
  "type": "Building",
  "created": "2025-01-15T10:30:00+09:00"
}
```

#### GET /cities/:cityId/objects/:objectId

Retrieve a city object.

```http
GET /cities/KR-SEOUL-2025/objects/building-gangnam-001
X-API-Key: your-api-key
```

**Response** (200 OK):
```json
{
  "id": "building-gangnam-001",
  "type": "Building",
  "geometry": {...},
  "properties": {...},
  "sensors": ["sensor-env-gangnam-001"],
  "metadata": {
    "created": "2024-01-01T00:00:00+09:00",
    "lastUpdated": "2025-01-15T10:30:00+09:00",
    "dataSource": "3d-scan",
    "accuracy": 0.99
  }
}
```

#### PATCH /cities/:cityId/objects/:objectId

Update a city object.

```http
PATCH /cities/KR-SEOUL-2025/objects/building-gangnam-001
X-API-Key: your-api-key
Content-Type: application/json

{
  "properties": {
    "occupancy": {
      "current": 1150,
      "capacity": 1500
    },
    "energyClass": "A+"
  }
}
```

#### DELETE /cities/:cityId/objects/:objectId

Delete a city object.

```http
DELETE /cities/KR-SEOUL-2025/objects/building-gangnam-001
X-API-Key: your-api-key
```

#### GET /cities/:cityId/objects

Query city objects.

```http
GET /cities/KR-SEOUL-2025/objects?type=Building&energyClass=A+&limit=10
X-API-Key: your-api-key
```

**Query Parameters**:
- `type`: Object type filter
- `bounds`: Geographic bounds (minLon,minLat,maxLon,maxLat)
- `properties.*`: Filter by property values
- `limit`: Result limit (default: 100, max: 1000)
- `offset`: Pagination offset

### 4.4 Sensor Endpoints

#### POST /cities/:cityId/sensors

Register a new sensor.

```http
POST /cities/KR-SEOUL-2025/sensors
X-API-Key: your-api-key
Content-Type: application/json

{
  "type": "EnvironmentalSensor",
  "location": {
    "type": "Point",
    "coordinates": [127.0276, 37.4979, 25.0]
  },
  "attachedTo": "building-gangnam-001",
  "specifications": {
    "manufacturer": "SmartCity Sensors Inc.",
    "model": "ENV-2000",
    "updateInterval": 300
  }
}
```

#### GET /cities/:cityId/sensors/:sensorId/data

Get sensor data.

```http
GET /cities/KR-SEOUL-2025/sensors/sensor-env-gangnam-001/data?start=2025-01-15T00:00:00+09:00&end=2025-01-15T23:59:59+09:00
X-API-Key: your-api-key
```

**Response**:
```json
{
  "sensorId": "sensor-env-gangnam-001",
  "timeRange": {
    "start": "2025-01-15T00:00:00+09:00",
    "end": "2025-01-15T23:59:59+09:00"
  },
  "measurements": [
    {
      "parameter": "temperature",
      "value": 22.5,
      "unit": "celsius",
      "timestamp": "2025-01-15T10:30:00+09:00",
      "quality": "excellent"
    },
    {
      "parameter": "humidity",
      "value": 42.0,
      "unit": "percent",
      "timestamp": "2025-01-15T10:30:00+09:00",
      "quality": "excellent"
    }
  ],
  "count": 288
}
```

#### POST /cities/:cityId/sensors/:sensorId/data

Submit sensor data.

```http
POST /cities/KR-SEOUL-2025/sensors/sensor-env-gangnam-001/data
X-API-Key: your-api-key
Content-Type: application/json

{
  "measurements": [
    {
      "parameter": "temperature",
      "value": 23.5,
      "unit": "celsius",
      "timestamp": "2025-01-15T10:35:00+09:00",
      "quality": "excellent"
    }
  ]
}
```

#### GET /cities/:cityId/sensors

Query sensors.

```http
GET /cities/KR-SEOUL-2025/sensors?type=EnvironmentalSensor&status=operational
X-API-Key: your-api-key
```

### 4.5 Simulation Endpoints

#### POST /cities/:cityId/simulations

Create and run a simulation.

```http
POST /cities/KR-SEOUL-2025/simulations
X-API-Key: your-api-key
Content-Type: application/json

{
  "type": "TrafficFlow",
  "parameters": {
    "timeRange": {
      "start": "2025-01-16T07:00:00+09:00",
      "end": "2025-01-16T09:00:00+09:00"
    },
    "weatherConditions": "clear",
    "resolution": "5min",
    "focusAreas": ["gangnam-district", "downtown"]
  }
}
```

**Response** (202 Accepted):
```json
{
  "id": "sim-traffic-20250115-001",
  "type": "TrafficFlow",
  "status": "running",
  "created": "2025-01-15T10:30:00+09:00",
  "estimatedCompletion": "2025-01-15T10:35:00+09:00"
}
```

#### GET /cities/:cityId/simulations/:simulationId

Get simulation status.

```http
GET /cities/KR-SEOUL-2025/simulations/sim-traffic-20250115-001
X-API-Key: your-api-key
```

**Response**:
```json
{
  "id": "sim-traffic-20250115-001",
  "type": "TrafficFlow",
  "status": "completed",
  "created": "2025-01-15T10:30:00+09:00",
  "completed": "2025-01-15T10:34:00+09:00",
  "progress": 100
}
```

#### GET /cities/:cityId/simulations/:simulationId/results

Get simulation results.

```http
GET /cities/KR-SEOUL-2025/simulations/sim-traffic-20250115-001/results
X-API-Key: your-api-key
```

**Response**:
```json
{
  "simulationId": "sim-traffic-20250115-001",
  "type": "TrafficFlow",
  "results": {
    "averageSpeed": {
      "value": 35.5,
      "unit": "km/h"
    },
    "congestionLevel": 0.65,
    "predictedDelay": {
      "value": 12.5,
      "unit": "minutes"
    },
    "affectedRoads": ["road-001", "road-015", "road-023"],
    "recommendations": [
      {
        "type": "route-optimization",
        "message": "Recommend alternative route via Highway 3",
        "priority": "medium"
      }
    ]
  },
  "confidence": 0.87,
  "validUntil": "2025-01-16T12:00:00+09:00"
}
```

#### DELETE /cities/:cityId/simulations/:simulationId

Cancel a running simulation.

```http
DELETE /cities/KR-SEOUL-2025/simulations/sim-traffic-20250115-001
X-API-Key: your-api-key
```

### 4.6 Service Endpoints

#### GET /cities/:cityId/services

Get city services.

```http
GET /cities/KR-SEOUL-2025/services?category=transportation
X-API-Key: your-api-key
```

**Response**:
```json
{
  "services": [
    {
      "id": "service-subway",
      "name": "Seoul Metro",
      "category": "transportation",
      "type": "public-transit",
      "status": "operational",
      "coverage": {
        "type": "MultiLineString",
        "coordinates": [...]
      },
      "realTimeData": {
        "activeVehicles": 450,
        "avgDelay": 2.5,
        "passengerCount": 1500000
      }
    }
  ]
}
```

#### GET /cities/:cityId/services/:serviceId/status

Get service status.

```http
GET /cities/KR-SEOUL-2025/services/service-subway/status
X-API-Key: your-api-key
```

### 4.7 Spatial Query Endpoints

#### POST /cities/:cityId/spatial/nearby

Find nearby objects.

```http
POST /cities/KR-SEOUL-2025/spatial/nearby
X-API-Key: your-api-key
Content-Type: application/json

{
  "location": {
    "type": "Point",
    "coordinates": [127.0276, 37.4979]
  },
  "radius": 500,
  "type": "Park"
}
```

**Response**:
```json
{
  "query": {
    "location": [127.0276, 37.4979],
    "radius": 500,
    "type": "Park"
  },
  "results": [
    {
      "id": "park-bongeunsa",
      "name": "Bongeunsa Park",
      "distance": 320.5,
      "location": [127.0291, 37.4985]
    }
  ],
  "count": 1
}
```

#### POST /cities/:cityId/spatial/route

Calculate route between points.

```http
POST /cities/KR-SEOUL-2025/spatial/route
X-API-Key: your-api-key
Content-Type: application/json

{
  "start": {
    "type": "Point",
    "coordinates": [127.0276, 37.4979]
  },
  "end": {
    "type": "Point",
    "coordinates": [126.9780, 37.5665]
  },
  "mode": "driving"
}
```

**Response**:
```json
{
  "route": {
    "type": "LineString",
    "coordinates": [...]
  },
  "distance": 12500,
  "duration": 1200,
  "mode": "driving",
  "instructions": [...]
}
```

### 4.8 Temporal Query Endpoints

#### GET /cities/:cityId/objects/:objectId/history

Get object history.

```http
GET /cities/KR-SEOUL-2025/objects/building-gangnam-001/history?start=2025-01-01T00:00:00+09:00&end=2025-01-15T23:59:59+09:00
X-API-Key: your-api-key
```

#### GET /cities/:cityId/snapshots/:timestamp

Get city snapshot at specific time.

```http
GET /cities/KR-SEOUL-2025/snapshots/2025-01-15T10:30:00+09:00
X-API-Key: your-api-key
```

---

## City Object Management

### 5.1 Creating City Objects

```typescript
import { DigitalTwinCity } from 'wia-digital-twin-city';

const city = new DigitalTwinCity({
  cityId: 'KR-SEOUL-2025',
  apiKey: 'your-api-key'
});

const building = await city.createCityObject({
  type: 'Building',
  geometry: {
    type: 'Polygon',
    coordinates: [[[127.0276, 37.4979], [127.0277, 37.4979], [127.0277, 37.4980], [127.0276, 37.4980], [127.0276, 37.4979]]]
  },
  properties: {
    name: 'New Office Tower',
    buildingType: 'commercial',
    floors: { above: 20, below: 3 },
    height: 80.0
  }
});

console.log('Created building:', building.id);
```

### 5.2 Querying City Objects

```typescript
interface CityObjectQuery {
  // Type filter
  type?: string | string[];

  // Spatial filters
  bounds?: GeoBounds;
  near?: {
    location: GeoPoint;
    radius: number;
  };

  // Property filters
  properties?: {
    [key: string]: any;
  };

  // Temporal filters
  updatedAfter?: Date;
  updatedBefore?: Date;

  // Pagination
  limit?: number;
  offset?: number;

  // Sorting
  sortBy?: string;
  sortOrder?: 'asc' | 'desc';
}
```

```typescript
const buildings = await city.queryCityObjects({
  type: 'Building',
  properties: {
    buildingType: 'commercial',
    'floors.above': { $gte: 20 }
  },
  bounds: {
    minLon: 127.0,
    minLat: 37.4,
    maxLon: 127.1,
    maxLat: 37.5
  },
  limit: 50
});
```

---

## Sensor Data Services

### 6.1 Real-time Sensor Streaming

```typescript
// Subscribe to sensor data stream
const subscription = city.streamSensorData(
  ['sensor-env-gangnam-001', 'sensor-env-gangnam-002'],
  (data) => {
    console.log(`Sensor ${data.sensorId}: ${data.parameter} = ${data.value} ${data.unit}`);
  }
);

// Unsubscribe
subscription.unsubscribe();
```

### 6.2 Historical Sensor Data

```typescript
const sensorData = await city.getSensorData('sensor-env-gangnam-001', {
  start: new Date('2025-01-15T00:00:00+09:00'),
  end: new Date('2025-01-15T23:59:59+09:00')
});

console.log(`Retrieved ${sensorData.length} measurements`);
```

### 6.3 Sensor Aggregations

```typescript
const avgTemperature = await city.aggregateSensorData({
  sensors: ['sensor-env-gangnam-001', 'sensor-env-gangnam-002'],
  parameter: 'temperature',
  aggregation: 'average',
  timeRange: {
    start: new Date('2025-01-15T00:00:00+09:00'),
    end: new Date('2025-01-15T23:59:59+09:00')
  },
  interval: '1hour'
});
```

---

## Simulation Services

### 7.1 Supported Simulation Types

| Type | Description | Parameters |
|------|-------------|------------|
| `TrafficFlow` | Traffic pattern prediction | timeRange, weatherConditions, focusAreas |
| `EnergyConsumption` | Energy usage forecast | timeRange, weatherData, buildings |
| `AirQuality` | Air quality prediction | timeRange, emissions, meteorology |
| `FloodRisk` | Flood risk assessment | rainfall, waterLevels, terrain |
| `EmergencyResponse` | Emergency scenario | incidentType, location, resources |
| `CrowdDynamics` | Crowd movement | event, location, expectedAttendance |

### 7.2 Running Simulations

```typescript
const simulation = await city.createSimulation({
  type: 'TrafficFlow',
  parameters: {
    timeRange: {
      start: new Date('2025-01-16T07:00:00+09:00'),
      end: new Date('2025-01-16T09:00:00+09:00')
    },
    weatherConditions: 'clear',
    resolution: '5min'
  }
});

// Wait for completion
const result = await city.runSimulation(simulation.id);

console.log('Average speed:', result.results.averageSpeed);
console.log('Congestion level:', result.results.congestionLevel);
```

---

## Authentication

### 8.1 API Key Authentication

```http
GET /cities/KR-SEOUL-2025
X-API-Key: EXAMPLE_API_KEY_REPLACE_ME
```

```typescript
const city = new DigitalTwinCity({
  cityId: 'KR-SEOUL-2025',
  apiKey: 'EXAMPLE_API_KEY_REPLACE_ME'
});
```

### 8.2 OAuth 2.0 Authentication

```typescript
const city = new DigitalTwinCity({
  cityId: 'KR-SEOUL-2025',
  oauth: {
    clientId: 'your-client-id',
    clientSecret: 'your-client-secret',
    tokenUrl: 'https://auth.wia.live/oauth/token'
  }
});
```

**OAuth Flow**:

1. **Get Access Token**:
```http
POST https://auth.wia.live/oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials&client_id=your-client-id&client_secret=your-client-secret
```

2. **Use Access Token**:
```http
GET /cities/KR-SEOUL-2025
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
```

---

## Error Handling

### 9.1 Error Response Format

```json
{
  "error": {
    "code": "OBJECT_NOT_FOUND",
    "message": "City object with ID 'building-invalid' not found",
    "details": {
      "cityId": "KR-SEOUL-2025",
      "objectId": "building-invalid"
    },
    "timestamp": "2025-01-15T10:30:00+09:00"
  }
}
```

### 9.2 Error Codes

```typescript
enum DigitalTwinCityErrorCode {
  // City errors (1xxx)
  CITY_NOT_FOUND = 1001,
  INVALID_CITY_DATA = 1002,

  // Object errors (2xxx)
  OBJECT_NOT_FOUND = 2001,
  INVALID_GEOMETRY = 2002,
  OUT_OF_BOUNDS = 2003,
  DUPLICATE_OBJECT = 2004,

  // Sensor errors (3xxx)
  SENSOR_NOT_FOUND = 3001,
  SENSOR_OFFLINE = 3002,
  INVALID_MEASUREMENT = 3003,
  DATA_QUALITY_LOW = 3004,

  // Simulation errors (4xxx)
  SIMULATION_NOT_FOUND = 4001,
  SIMULATION_FAILED = 4002,
  INVALID_PARAMETERS = 4003,

  // Auth errors (5xxx)
  UNAUTHORIZED = 5001,
  FORBIDDEN = 5002,
  INVALID_API_KEY = 5003,

  // Rate limiting (6xxx)
  RATE_LIMIT_EXCEEDED = 6001
}
```

### 9.3 Error Handling Example

```typescript
try {
  await city.getCityObject('building-invalid');
} catch (error) {
  if (error instanceof DigitalTwinCityError) {
    switch (error.code) {
      case DigitalTwinCityErrorCode.OBJECT_NOT_FOUND:
        console.log('Object not found');
        break;
      case DigitalTwinCityErrorCode.UNAUTHORIZED:
        console.log('Authentication required');
        break;
      default:
        console.error('Unknown error:', error.message);
    }
  }
}
```

---

## Usage Examples

### 10.1 Complete City Setup

```typescript
import { DigitalTwinCity } from 'wia-digital-twin-city';

async function setupCity() {
  const city = new DigitalTwinCity({
    cityId: 'KR-SEOUL-2025',
    apiKey: process.env.WIA_API_KEY,
    enableWebSocket: true
  });

  // Get city info
  const info = await city.getCityInfo();
  console.log('City:', info.name);
  console.log('Objects:', info.statistics.totalObjects);

  // Query buildings
  const buildings = await city.queryCityObjects({
    type: 'Building',
    properties: {
      buildingType: 'commercial'
    },
    limit: 100
  });

  console.log(`Found ${buildings.length} commercial buildings`);

  return city;
}
```

### 10.2 Real-time Monitoring Dashboard

```typescript
async function monitorCity() {
  const city = new DigitalTwinCity({
    cityId: 'KR-SEOUL-2025',
    apiKey: process.env.WIA_API_KEY,
    enableWebSocket: true
  });

  // Stream environmental sensors
  city.streamSensorData(
    await city.querySensors({ type: 'EnvironmentalSensor' }).then(s => s.map(x => x.id)),
    (data) => {
      if (data.parameter === 'air_quality' && data.value > 100) {
        console.warn(`High AQI detected: ${data.value} at sensor ${data.sensorId}`);
      }
    }
  );

  // Listen for alerts
  city.on('alert:created', (alert) => {
    console.log('ALERT:', alert.message);
    sendNotification(alert);
  });
}
```

### 10.3 Traffic Simulation

```python
import asyncio
from wia_digital_twin_city import DigitalTwinCity

async def predict_traffic():
    city = DigitalTwinCity(
        city_id='KR-SEOUL-2025',
        api_key='your-api-key'
    )

    # Create traffic simulation
    simulation = await city.create_simulation({
        'type': 'TrafficFlow',
        'parameters': {
            'timeRange': {
                'start': '2025-01-16T07:00:00+09:00',
                'end': '2025-01-16T09:00:00+09:00'
            },
            'weatherConditions': 'clear',
            'focusAreas': ['gangnam-district']
        }
    })

    print(f'Simulation created: {simulation.id}')

    # Run simulation
    result = await city.run_simulation(simulation.id)

    print(f'Average speed: {result.results.averageSpeed.value} {result.results.averageSpeed.unit}')
    print(f'Congestion level: {result.results.congestionLevel}')

    for rec in result.results.recommendations:
        print(f'- {rec.type}: {rec.message}')

asyncio.run(predict_traffic())
```

### 10.4 Spatial Analysis

```typescript
async function findParkingNear(location: GeoPoint) {
  const city = new DigitalTwinCity({
    cityId: 'KR-SEOUL-2025',
    apiKey: process.env.WIA_API_KEY
  });

  const parkingLots = await city.findNearby(location, 1000, 'ParkingLot');

  for (const lot of parkingLots) {
    const availability = await city.getCityObject(lot.id);
    console.log(`${availability.properties.name}: ${availability.properties.availableSpaces} spaces`);
  }
}
```

### 10.5 Energy Optimization

```typescript
async function optimizeEnergy() {
  const city = new DigitalTwinCity({
    cityId: 'KR-SEOUL-2025',
    apiKey: process.env.WIA_API_KEY
  });

  // Get all buildings with energy sensors
  const buildings = await city.queryCityObjects({
    type: 'Building',
    properties: {
      energyClass: { $in: ['A', 'A+', 'A++'] }
    }
  });

  // Run energy simulation
  const simulation = await city.createSimulation({
    type: 'EnergyConsumption',
    parameters: {
      timeRange: {
        start: new Date(),
        end: new Date(Date.now() + 24 * 60 * 60 * 1000)
      },
      buildings: buildings.map(b => b.id)
    }
  });

  const result = await city.runSimulation(simulation.id);

  console.log('Total consumption:', result.results.totalConsumption);
  console.log('Peak demand:', result.results.peakDemand);

  for (const rec of result.results.recommendations) {
    console.log(`Recommendation: ${rec.message}`);
    console.log(`Potential savings: ${rec.potentialSavings}`);
  }
}
```

---

## References

### Related Standards

- [WIA Digital Twin City Data Format (Phase 1)](/digital-twin-city/spec/PHASE-1-DATA-FORMAT.md)
- [WIA Digital Twin City Protocol (Phase 3)](/digital-twin-city/spec/PHASE-3-PROTOCOL.md)
- [GeoJSON RFC 7946](https://tools.ietf.org/html/rfc7946)
- [CityGML 3.0](https://www.ogc.org/standards/citygml)

### API Standards

- [OpenAPI 3.0 Specification](https://swagger.io/specification/)
- [REST API Best Practices](https://restfulapi.net/)
- [WebSocket Protocol RFC 6455](https://tools.ietf.org/html/rfc6455)

### Authentication Standards

- [OAuth 2.0 RFC 6749](https://tools.ietf.org/html/rfc6749)
- [JWT RFC 7519](https://tools.ietf.org/html/rfc7519)

---

<div align="center">

**WIA Digital Twin City Standard v1.0.0**

**弘益人間 (홍익인간)** - Benefit All Humanity

---

**© 2025 WIA**

**MIT License**

</div>


## Annex E — Implementation Notes for PHASE-2-API-INTERFACE

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-2-API-INTERFACE.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.

## Annex G — Test Vectors and Conformance Evidence

This annex describes how implementations capture and publish conformance
evidence for PHASE-2-API-INTERFACE. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-2-api-interface/`. Implementations claiming
  conformance MUST run all vectors in CI and publish the resulting
  pass/fail matrix in their compliance package.
- **Evidence package** — the compliance package is a tarball containing
  the SBOM (CycloneDX 1.5 or SPDX 2.3), the OpenAPI document, the test
  vector matrix, and a signed manifest. Signatures use Sigstore (DSSE
  envelope, Rekor transparency log entry) so that downstream consumers
  can verify provenance without trusting a private CA.
- **Quarterly recheck** — implementations re-publish the evidence package
  every quarter even if no source change occurred, so that consumers can
  detect environmental drift (compiler updates, dependency updates, OS
  updates) without polling vendor changelogs.
- **Cross-vendor crosswalk** — the WIA Standards working group maintains a
  crosswalk that maps each vector to the equivalent assertion in adjacent
  industry programs (where one exists), so an implementer that already
  certifies under one program can show conformance to PHASE-2-API-INTERFACE with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-2-API-INTERFACE does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-2-API-INTERFACE.
It is non-normative; the rules below describe the policy that the WIA
Standards working group commits to when amending this PHASE document.

- **Semantic versioning** — major / minor / patch components follow
  Semantic Versioning 2.0.0 (https://semver.org/spec/v2.0.0.html).
  Major bump indicates a backwards-incompatible change to a normative
  requirement; minor bump indicates new normative requirements that do
  not break existing implementations; patch bump indicates editorial
  changes only (clarifications, typo fixes, formatting).
- **Deprecation window** — when a normative requirement is removed or
  altered in a backwards-incompatible way, the prior major version is
  maintained in parallel for at least 180 days. During the parallel
  window, both major versions are marked Stable in the WIA Standards
  registry and either may be cited as "WIA-conformant".
- **Sunset notification** — deprecated major versions enter a 12-month
  sunset window during which the WIA registry marks the version as
  Deprecated. The deprecation entry includes a migration note pointing
  to the replacement requirement(s) and an explanation of why the
  change was made.
- **Editorial errata** — patch-level errata are issued without a
  deprecation window because they do not change normative behaviour.
  Errata are tracked in a public errata register and each entry is
  signed by the WIA Standards working group chair.
- **Implementation changelog mapping** — implementations SHOULD publish
  a changelog mapping each PHASE version they support to the specific
  build, container digest, or SDK version that satisfies the version.
  This allows downstream auditors to verify version conformance without
  re-running the entire test matrix on every release.

The policy is reviewed at the same cadence as the PHASE document and
any changes to the policy itself are tracked in the version-history
table at the start of the document.

## Annex I — Interoperability Profiles

This annex describes how implementations declare interoperability profiles
for PHASE-2-API-INTERFACE. The profile mechanism is non-normative and exists so that
deployments of varying scope (single tenant, regional cluster, federated
network) can advertise the subset of normative requirements they satisfy
without misrepresenting partial conformance as full conformance.

- **Profile manifest** — every implementation publishes a profile manifest
  in JSON. The manifest enumerates the normative requirement IDs from this
  PHASE that are satisfied (`status: "supported"`), partially satisfied
  (`status: "partial"`, with a reason field), or excluded
  (`status: "excluded"`, with a justification). The manifest is signed
  using the same Sigstore key used for the SBOM in Annex G.
- **Federation profile** — federated deployments publish an aggregated
  manifest summarizing the union and intersection of member-implementation
  profiles. The aggregated manifest is consumed by directory services so
  that callers can route a request to the least common denominator profile
  required for an interaction.
- **Backwards-profile compatibility** — when a deployment migrates from one
  profile to a wider profile, the prior profile manifest remains valid and
  signed for the deprecation window defined in Annex H. This preserves
  audit traceability for auditors evaluating long-term interoperability.
- **Profile registry** — the WIA Standards working group maintains a
  public registry of named profiles. Common deployment shapes (e.g.,
  "Edge-only", "Federated-with-replay") are added to the registry by
  consensus. Registry entries are immutable; new shapes are added under
  new names rather than amending existing entries.
- **Profile versioning** — profile names are versioned with the same
  Semantic Versioning rules described in Annex H. A deployment that
  advertises `WIA-P2-API-INTERFACE-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.

## Annex J — Reference Implementation Topology

The reference implementation topology described in this annex is
non-normative; it documents the deployment shape that the WIA
Standards working group used to validate the test vectors in Annex G
and is intended as a starting point, not a recommendation against
alternative topologies.

- **Single-tenant edge** — one runtime per organization, no shared
  state. Used for early-pilot deployments where conformance evidence
  is published manually. Sufficient for PHASE-2-API-INTERFACE validation when the
  organization signs the manifest itself.
- **Multi-tenant gateway** — one shared runtime serves multiple
  tenants via header-based isolation. Typically backed by a
  rate-limited gateway (Envoy or NGINX) and a shared OAuth 2.1
  identity provider. The manifest is per-tenant; the runtime
  publishes a federation manifest that aggregates tenant manifests.
- **Federated mesh** — multiple runtimes peer to one another and
  publish their manifests to a directory service. Each peer signs
  its own manifest; the directory service signs the aggregated
  index. This is the topology used by cross-organization deployments
  that need to compose conformance.
- **Air-gapped batch** — no network connection between the runtime
  and the directory service. The runtime emits a signed evidence
  package on each batch and the operator transports the package via
  out-of-band channels. This is the topology used by regulators that
  prohibit live connectivity from sensitive environments.

Implementations declare their topology in the manifest (see Annex I).
A topology change MUST be reflected in a new manifest signature; the
prior topology's manifest remains valid for the deprecation window
described in Annex H to preserve audit traceability.
