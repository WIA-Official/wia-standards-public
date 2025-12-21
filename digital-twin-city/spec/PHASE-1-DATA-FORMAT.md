# WIA Digital Twin City Data Format Standard
## Phase 1 Specification

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
3. [Base Structure](#base-structure)
4. [Data Schema](#data-schema)
5. [Field Specifications](#field-specifications)
6. [Data Types](#data-types)
7. [Validation Rules](#validation-rules)
8. [Examples](#examples)
9. [Version History](#version-history)

---

## Overview

### 1.1 Purpose

The WIA Digital Twin City Data Format Standard defines a unified format for representing urban infrastructure, real-time sensor data, and simulation results in digital twin city platforms. This standard enables seamless data exchange between smart city systems, IoT platforms, and citizen services.

**Core Objectives**:
- Establish standardized format for 3D city models and urban infrastructure
- Enable real-time sensor data integration from IoT devices
- Support simulation and predictive analytics for city planning
- Facilitate citizen service integration and data transparency
- Ensure interoperability across different smart city platforms

### 1.2 Scope

This standard covers:

| Domain | Description |
|--------|-------------|
| 3D City Models | Buildings, roads, utilities, terrain geometry |
| IoT Sensor Data | Real-time data from environmental, traffic, energy sensors |
| City Objects | Landmarks, public facilities, infrastructure assets |
| Simulation Data | Traffic flow, energy consumption, environmental predictions |
| Citizen Services | Public transportation, waste management, emergency services |
| Temporal Data | Historical records and real-time updates |

### 1.3 Design Principles

1. **Interoperability**: Compatible with CityGML, GeoJSON, and common GIS formats
2. **Real-time**: Support streaming sensor data and live updates
3. **Scalability**: Handle city-scale data with millions of objects
4. **Accuracy**: Maintain spatial precision and temporal consistency
5. **Extensibility**: Allow custom attributes and domain-specific extensions

---

## Terminology

### 2.1 Core Terms

| Term | Definition |
|------|------------|
| **Digital Twin** | Virtual representation of physical city infrastructure |
| **City Object** | Any physical entity in the city (building, road, sensor) |
| **Sensor Node** | IoT device collecting real-time data |
| **Simulation Layer** | Computational model for predictions |
| **Temporal State** | Time-stamped snapshot of city conditions |
| **Geospatial Reference** | Coordinate system for spatial positioning |

### 2.2 Data Types

| Type | Description | Example |
|------|-------------|---------|
| `geometry` | GeoJSON geometry object | `{"type": "Polygon", ...}` |
| `coordinate` | [longitude, latitude, altitude] | `[126.9780, 37.5665, 50.0]` |
| `timestamp` | ISO 8601 datetime with timezone | `"2025-01-15T10:30:00+09:00"` |
| `sensor_value` | Numeric measurement with unit | `{"value": 23.5, "unit": "celsius"}` |
| `uuid` | Unique identifier | `"550e8400-e29b-41d4-a716-446655440000"` |

### 2.3 Field Requirements

| Marker | Meaning |
|--------|---------|
| **REQUIRED** | Must be present in all records |
| **OPTIONAL** | May be omitted |
| **CONDITIONAL** | Required under specific conditions |

---

## Base Structure

### 3.1 Digital Twin Record Format

```json
{
  "$schema": "https://wia.live/digital-twin-city/v1/schema.json",
  "version": "1.0.0",
  "cityId": "KR-SEOUL-2025",
  "timestamp": "2025-01-15T10:30:00+09:00",
  "bounds": {
    "type": "Polygon",
    "coordinates": [[[126.764, 37.413], [127.184, 37.413], [127.184, 37.701], [126.764, 37.701], [126.764, 37.413]]]
  },
  "coordinateSystem": {
    "type": "WGS84",
    "epsg": 4326
  },
  "cityObjects": [],
  "sensorNetwork": {
    "nodes": [],
    "aggregations": []
  },
  "simulations": [],
  "services": [],
  "metadata": {
    "created": "2025-01-01T00:00:00+09:00",
    "lastUpdated": "2025-01-15T10:30:00+09:00",
    "updateFrequency": "5min",
    "dataQuality": 0.95
  }
}
```

### 3.2 Field Details

#### 3.2.1 `cityId` (REQUIRED)

```
Type: string
Format: CC-CITY-YYYY (Country Code-City Name-Year)
Description: Unique identifier for the digital twin city instance
Example: "KR-SEOUL-2025", "US-NYC-2025"
```

#### 3.2.2 `coordinateSystem` (REQUIRED)

```
Type: object
Description: Spatial reference system for all geometric data
Fields:
  - type: "WGS84", "UTM", "local"
  - epsg: EPSG code (e.g., 4326 for WGS84)
  - zone: UTM zone (if applicable)
```

---

## Data Schema

### 4.1 Complete JSON Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wia.live/digital-twin-city/v1/schema.json",
  "title": "WIA Digital Twin City Record",
  "type": "object",
  "required": ["version", "cityId", "timestamp", "bounds", "coordinateSystem"],
  "properties": {
    "version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+\\.\\d+$"
    },
    "cityId": {
      "type": "string",
      "pattern": "^[A-Z]{2}-[A-Z0-9]+-\\d{4}$"
    },
    "timestamp": {
      "type": "string",
      "format": "date-time"
    },
    "bounds": {
      "type": "object",
      "properties": {
        "type": { "type": "string", "enum": ["Polygon"] },
        "coordinates": { "type": "array" }
      },
      "required": ["type", "coordinates"]
    },
    "coordinateSystem": {
      "type": "object",
      "properties": {
        "type": { "type": "string", "enum": ["WGS84", "UTM", "local"] },
        "epsg": { "type": "number" }
      },
      "required": ["type", "epsg"]
    },
    "cityObjects": {
      "type": "array",
      "items": { "$ref": "#/definitions/CityObject" }
    },
    "sensorNetwork": {
      "type": "object",
      "properties": {
        "nodes": { "type": "array" },
        "aggregations": { "type": "array" }
      }
    },
    "simulations": {
      "type": "array",
      "items": { "$ref": "#/definitions/Simulation" }
    }
  },
  "definitions": {
    "CityObject": {
      "type": "object",
      "required": ["id", "type", "geometry"],
      "properties": {
        "id": { "type": "string" },
        "type": { "type": "string" },
        "geometry": { "type": "object" },
        "properties": { "type": "object" }
      }
    },
    "Simulation": {
      "type": "object",
      "required": ["id", "type", "parameters", "results"],
      "properties": {
        "id": { "type": "string" },
        "type": { "type": "string" },
        "parameters": { "type": "object" },
        "results": { "type": "object" }
      }
    }
  }
}
```

### 4.2 City Object Schema

```json
{
  "cityObjects": [
    {
      "id": "building-001",
      "type": "Building",
      "geometry": {
        "type": "MultiPolygon",
        "coordinates": [
          [[[126.9780, 37.5665, 0], [126.9781, 37.5665, 0], [126.9781, 37.5666, 0], [126.9780, 37.5666, 0], [126.9780, 37.5665, 0]]],
          [[[126.9780, 37.5665, 50], [126.9781, 37.5665, 50], [126.9781, 37.5666, 50], [126.9780, 37.5666, 50], [126.9780, 37.5665, 50]]]
        ],
        "lod": 2
      },
      "properties": {
        "name": "City Hall",
        "address": "110 Sejong-daero, Jung-gu, Seoul",
        "buildingType": "government",
        "constructionYear": 2012,
        "floors": {
          "above": 12,
          "below": 3
        },
        "height": 50.0,
        "area": 5000.0,
        "occupancy": {
          "current": 450,
          "capacity": 600
        },
        "energyClass": "A",
        "materials": {
          "facade": "glass",
          "structure": "reinforced-concrete"
        }
      },
      "sensors": ["sensor-temp-001", "sensor-energy-001"],
      "metadata": {
        "created": "2025-01-01T00:00:00+09:00",
        "lastUpdated": "2025-01-15T10:30:00+09:00",
        "dataSource": "cadastral-survey",
        "accuracy": 0.98
      }
    }
  ]
}
```

### 4.3 Sensor Network Schema

```json
{
  "sensorNetwork": {
    "nodes": [
      {
        "id": "sensor-temp-001",
        "type": "EnvironmentalSensor",
        "location": {
          "type": "Point",
          "coordinates": [126.9780, 37.5665, 25.0]
        },
        "attachedTo": "building-001",
        "measurements": [
          {
            "parameter": "temperature",
            "value": 23.5,
            "unit": "celsius",
            "timestamp": "2025-01-15T10:30:00+09:00",
            "quality": "good"
          },
          {
            "parameter": "humidity",
            "value": 45.2,
            "unit": "percent",
            "timestamp": "2025-01-15T10:30:00+09:00",
            "quality": "good"
          }
        ],
        "specifications": {
          "manufacturer": "SmartCity Sensors Inc.",
          "model": "ENV-2000",
          "installDate": "2024-06-15",
          "calibrationDate": "2024-12-01",
          "accuracy": 0.99,
          "updateInterval": 300
        },
        "status": {
          "operational": true,
          "battery": 85,
          "lastMaintenance": "2024-12-01T09:00:00+09:00"
        }
      }
    ],
    "aggregations": [
      {
        "id": "agg-district-temp",
        "type": "spatial-average",
        "sensors": ["sensor-temp-001", "sensor-temp-002", "sensor-temp-003"],
        "parameter": "temperature",
        "value": 24.1,
        "unit": "celsius",
        "timestamp": "2025-01-15T10:30:00+09:00",
        "coverage": {
          "type": "Polygon",
          "coordinates": [[[126.97, 37.56], [126.98, 37.56], [126.98, 37.57], [126.97, 37.57], [126.97, 37.56]]]
        }
      }
    ]
  }
}
```

### 4.4 Simulation Schema

```json
{
  "simulations": [
    {
      "id": "sim-traffic-morning",
      "type": "TrafficFlow",
      "status": "completed",
      "timestamp": "2025-01-15T10:30:00+09:00",
      "parameters": {
        "timeRange": {
          "start": "2025-01-15T07:00:00+09:00",
          "end": "2025-01-15T09:00:00+09:00"
        },
        "weatherConditions": "clear",
        "specialEvents": [],
        "resolution": "5min"
      },
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
      "validUntil": "2025-01-15T12:00:00+09:00"
    }
  ]
}
```

---

## Field Specifications

### 5.1 City Object Fields

| Field | Type | Required | Description | Example |
|-------|------|----------|-------------|---------|
| `id` | string | REQUIRED | Unique object identifier | `"building-001"` |
| `type` | string | REQUIRED | Object type category | `"Building"`, `"Road"`, `"Bridge"` |
| `geometry` | object | REQUIRED | GeoJSON geometry | `{...}` |
| `geometry.lod` | number | OPTIONAL | Level of Detail (0-4) | `2` |
| `properties` | object | REQUIRED | Object attributes | `{...}` |
| `sensors` | array | OPTIONAL | Associated sensor IDs | `["sensor-001"]` |

**Valid Object Types:**

| Type | Description | Typical Attributes |
|------|-------------|-------------------|
| `Building` | Structures with roofs | height, floors, buildingType |
| `Road` | Transportation routes | width, lanes, surfaceType |
| `Bridge` | Elevated connections | span, loadCapacity, material |
| `Park` | Green spaces | area, facilities, vegetation |
| `Utility` | Infrastructure systems | type, capacity, networkId |
| `Landmark` | Notable locations | name, category, significance |

### 5.2 Sensor Measurement Fields

| Field | Type | Required | Description | Example |
|-------|------|----------|-------------|---------|
| `parameter` | string | REQUIRED | Measurement type | `"temperature"` |
| `value` | number | REQUIRED | Measured value | `23.5` |
| `unit` | string | REQUIRED | Measurement unit | `"celsius"` |
| `timestamp` | string | REQUIRED | Measurement time | `"2025-01-15T10:30:00+09:00"` |
| `quality` | string | REQUIRED | Data quality | `"good"`, `"fair"`, `"poor"` |

**Standard Measurement Parameters:**

| Parameter | Unit | Range | Typical Sensors |
|-----------|------|-------|----------------|
| `temperature` | celsius, fahrenheit | -40 to 60 | Environmental |
| `humidity` | percent | 0 to 100 | Environmental |
| `air_quality` | aqi | 0 to 500 | Environmental |
| `noise_level` | decibel | 0 to 120 | Environmental |
| `traffic_volume` | vehicles/hour | 0 to 10000 | Traffic |
| `occupancy` | count | 0 to capacity | Building |
| `energy_consumption` | kWh | 0 to max | Energy |
| `water_flow` | liters/min | 0 to max | Utility |

### 5.3 Geometry Level of Detail (LOD)

| LOD | Description | Use Case | Example |
|-----|-------------|----------|---------|
| 0 | Footprint only | City-wide overview | 2D polygon |
| 1 | Block model | District planning | Simple 3D box |
| 2 | Detailed exterior | Visualization | Roof structure, facade |
| 3 | Architectural model | Building management | Windows, doors, balconies |
| 4 | Interior model | Facility management | Rooms, furniture, equipment |

---

## Data Types

### 6.1 Custom Types

```typescript
type CityObjectType =
  | 'Building'
  | 'Road'
  | 'Bridge'
  | 'Park'
  | 'Utility'
  | 'Landmark'
  | 'TransportHub'
  | 'WaterBody';

type SensorType =
  | 'EnvironmentalSensor'
  | 'TrafficSensor'
  | 'EnergySensor'
  | 'OccupancySensor'
  | 'UtilitySensor'
  | 'WeatherStation';

type SimulationType =
  | 'TrafficFlow'
  | 'EnergyConsumption'
  | 'AirQuality'
  | 'FloodRisk'
  | 'EmergencyResponse'
  | 'CrowdDynamics';

type DataQuality =
  | 'excellent'  // > 95% accuracy
  | 'good'       // 80-95% accuracy
  | 'fair'       // 60-80% accuracy
  | 'poor';      // < 60% accuracy

interface GeoCoordinate {
  longitude: number;  // -180 to 180
  latitude: number;   // -90 to 90
  altitude?: number;  // meters above sea level
}

interface SensorMeasurement {
  parameter: string;
  value: number;
  unit: string;
  timestamp: string;
  quality: DataQuality;
  confidence?: number;
}
```

### 6.2 GeoJSON Integration

All geometric data follows GeoJSON RFC 7946 specification:

```json
{
  "type": "Feature",
  "geometry": {
    "type": "Polygon",
    "coordinates": [
      [[126.9780, 37.5665], [126.9781, 37.5665], [126.9781, 37.5666], [126.9780, 37.5666], [126.9780, 37.5665]]
    ]
  },
  "properties": {
    "cityObjectId": "building-001",
    "name": "City Hall"
  }
}
```

### 6.3 CityGML References

Compatible with CityGML 3.0 core module types:

| WIA Type | CityGML Equivalent | Notes |
|----------|-------------------|-------|
| `Building` | `Building` | Mapped 1:1 |
| `Road` | `Road` | Part of Transportation module |
| `Bridge` | `Bridge` | Part of Bridge module |
| `Utility` | `Utility Network` | Extended attributes |
| `Park` | `Vegetation` | Part of Vegetation module |

---

## Validation Rules

### 7.1 Required Field Validation

| Rule ID | Field | Validation |
|---------|-------|------------|
| VAL-001 | `cityId` | Must match `^[A-Z]{2}-[A-Z0-9]+-\d{4}$` |
| VAL-002 | `geometry.coordinates` | Must be valid WGS84 coordinates |
| VAL-003 | `sensor.value` | Must be within parameter range |
| VAL-004 | `timestamp` | Must be valid ISO 8601 with timezone |
| VAL-005 | `geometry.lod` | Must be 0, 1, 2, 3, or 4 |

### 7.2 Business Logic Validation

| Rule ID | Description | Error Code |
|---------|-------------|------------|
| BUS-001 | Object IDs must be unique within city | `ERR_DUPLICATE_ID` |
| BUS-002 | Sensor timestamps must not be future dates | `ERR_FUTURE_TIMESTAMP` |
| BUS-003 | Geometry coordinates must be within city bounds | `ERR_OUT_OF_BOUNDS` |
| BUS-004 | Sensor values must match parameter type | `ERR_INVALID_VALUE` |
| BUS-005 | Referenced objects must exist | `ERR_MISSING_REFERENCE` |

### 7.3 Error Codes

| Code | Message | Description |
|------|---------|-------------|
| `ERR_INVALID_CITY_ID` | Invalid city identifier format | cityId format violation |
| `ERR_DUPLICATE_ID` | Object ID already exists | Duplicate object detection |
| `ERR_OUT_OF_BOUNDS` | Coordinates outside city bounds | Spatial validation failed |
| `ERR_INVALID_GEOMETRY` | Invalid GeoJSON geometry | Geometry format error |
| `ERR_SENSOR_OFFLINE` | Sensor not responding | Connectivity issue |
| `ERR_QUALITY_LOW` | Data quality below threshold | Quality check failed |

---

## Examples

### 8.1 Valid Digital Twin Record

```json
{
  "$schema": "https://wia.live/digital-twin-city/v1/schema.json",
  "version": "1.0.0",
  "cityId": "KR-SEOUL-2025",
  "timestamp": "2025-01-15T10:30:00+09:00",
  "bounds": {
    "type": "Polygon",
    "coordinates": [[[126.764, 37.413], [127.184, 37.413], [127.184, 37.701], [126.764, 37.701], [126.764, 37.413]]]
  },
  "coordinateSystem": {
    "type": "WGS84",
    "epsg": 4326
  },
  "cityObjects": [
    {
      "id": "building-gangnam-001",
      "type": "Building",
      "geometry": {
        "type": "MultiPolygon",
        "coordinates": [
          [[[127.0276, 37.4979, 0], [127.0277, 37.4979, 0], [127.0277, 37.4980, 0], [127.0276, 37.4980, 0], [127.0276, 37.4979, 0]]],
          [[[127.0276, 37.4979, 120], [127.0277, 37.4979, 120], [127.0277, 37.4980, 120], [127.0276, 37.4980, 120], [127.0276, 37.4979, 120]]]
        ],
        "lod": 2
      },
      "properties": {
        "name": "Gangnam Finance Center",
        "address": "152 Teheran-ro, Gangnam-gu, Seoul",
        "buildingType": "commercial",
        "constructionYear": 2020,
        "floors": {
          "above": 30,
          "below": 5
        },
        "height": 120.0,
        "area": 15000.0,
        "occupancy": {
          "current": 1200,
          "capacity": 1500
        },
        "energyClass": "A+",
        "smartFeatures": ["automated-hvac", "smart-lighting", "ev-charging"]
      },
      "sensors": ["sensor-env-gangnam-001", "sensor-energy-gangnam-001"],
      "metadata": {
        "created": "2024-01-01T00:00:00+09:00",
        "lastUpdated": "2025-01-15T10:30:00+09:00",
        "dataSource": "3d-scan",
        "accuracy": 0.99
      }
    }
  ],
  "sensorNetwork": {
    "nodes": [
      {
        "id": "sensor-env-gangnam-001",
        "type": "EnvironmentalSensor",
        "location": {
          "type": "Point",
          "coordinates": [127.0276, 37.4979, 60.0]
        },
        "attachedTo": "building-gangnam-001",
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
          },
          {
            "parameter": "air_quality",
            "value": 45,
            "unit": "aqi",
            "timestamp": "2025-01-15T10:30:00+09:00",
            "quality": "good"
          }
        ],
        "specifications": {
          "manufacturer": "Samsung SmartCity",
          "model": "ENV-3000",
          "installDate": "2024-03-15",
          "calibrationDate": "2024-12-15",
          "accuracy": 0.99,
          "updateInterval": 300
        },
        "status": {
          "operational": true,
          "battery": 92,
          "lastMaintenance": "2024-12-15T14:00:00+09:00"
        }
      }
    ]
  },
  "simulations": [
    {
      "id": "sim-energy-daily",
      "type": "EnergyConsumption",
      "status": "completed",
      "timestamp": "2025-01-15T10:30:00+09:00",
      "parameters": {
        "timeRange": {
          "start": "2025-01-15T00:00:00+09:00",
          "end": "2025-01-15T23:59:59+09:00"
        },
        "weatherData": {
          "avgTemperature": 5.0,
          "maxTemperature": 8.0,
          "minTemperature": 2.0
        }
      },
      "results": {
        "totalConsumption": {
          "value": 45000,
          "unit": "kWh"
        },
        "peakDemand": {
          "value": 2500,
          "unit": "kW",
          "time": "2025-01-15T19:00:00+09:00"
        },
        "efficiency": 0.88,
        "recommendations": [
          {
            "type": "load-shifting",
            "message": "Shift 15% of non-critical loads to off-peak hours",
            "potentialSavings": "2000 kWh",
            "priority": "high"
          }
        ]
      },
      "confidence": 0.92,
      "validUntil": "2025-01-16T00:00:00+09:00"
    }
  ],
  "metadata": {
    "created": "2025-01-01T00:00:00+09:00",
    "lastUpdated": "2025-01-15T10:30:00+09:00",
    "updateFrequency": "5min",
    "dataQuality": 0.96
  }
}
```

### 8.2 Invalid Example - Missing Required Fields

```json
{
  "version": "1.0.0",
  "cityId": "INVALID-ID",
  "cityObjects": [
    {
      "id": "building-001"
    }
  ]
}
```

**Errors**:
- `ERR_INVALID_CITY_ID` - cityId format must be CC-CITY-YYYY
- `ERR_MISSING_FIELD` - Missing required field: timestamp
- `ERR_MISSING_FIELD` - Missing required field: bounds
- `ERR_MISSING_FIELD` - Missing required field: coordinateSystem
- `ERR_MISSING_FIELD` - cityObjects[0] missing required field: type
- `ERR_MISSING_FIELD` - cityObjects[0] missing required field: geometry

### 8.3 Invalid Example - Out of Bounds Coordinates

```json
{
  "$schema": "https://wia.live/digital-twin-city/v1/schema.json",
  "version": "1.0.0",
  "cityId": "KR-SEOUL-2025",
  "timestamp": "2025-01-15T10:30:00+09:00",
  "bounds": {
    "type": "Polygon",
    "coordinates": [[[126.764, 37.413], [127.184, 37.413], [127.184, 37.701], [126.764, 37.701], [126.764, 37.413]]]
  },
  "coordinateSystem": {
    "type": "WGS84",
    "epsg": 4326
  },
  "cityObjects": [
    {
      "id": "building-001",
      "type": "Building",
      "geometry": {
        "type": "Point",
        "coordinates": [140.0, 45.0]
      },
      "properties": {
        "name": "Test Building"
      }
    }
  ]
}
```

**Error**: `ERR_OUT_OF_BOUNDS` - Building coordinates [140.0, 45.0] are outside city bounds

### 8.4 Invalid Example - Future Timestamp

```json
{
  "sensorNetwork": {
    "nodes": [
      {
        "id": "sensor-001",
        "type": "EnvironmentalSensor",
        "measurements": [
          {
            "parameter": "temperature",
            "value": 25.0,
            "unit": "celsius",
            "timestamp": "2030-01-15T10:30:00+09:00",
            "quality": "good"
          }
        ]
      }
    ]
  }
}
```

**Error**: `ERR_FUTURE_TIMESTAMP` - Sensor timestamp cannot be in the future

### 8.5 Invalid Example - Invalid Sensor Value

```json
{
  "sensorNetwork": {
    "nodes": [
      {
        "id": "sensor-001",
        "type": "EnvironmentalSensor",
        "measurements": [
          {
            "parameter": "humidity",
            "value": 150,
            "unit": "percent",
            "timestamp": "2025-01-15T10:30:00+09:00",
            "quality": "good"
          }
        ]
      }
    ]
  }
}
```

**Error**: `ERR_INVALID_VALUE` - Humidity value 150 exceeds valid range (0-100)

### 8.6 Valid Traffic Sensor Example

```json
{
  "sensorNetwork": {
    "nodes": [
      {
        "id": "sensor-traffic-gangnam-01",
        "type": "TrafficSensor",
        "location": {
          "type": "Point",
          "coordinates": [127.0276, 37.4979, 0]
        },
        "attachedTo": "road-teheran-main",
        "measurements": [
          {
            "parameter": "traffic_volume",
            "value": 850,
            "unit": "vehicles/hour",
            "timestamp": "2025-01-15T10:30:00+09:00",
            "quality": "excellent"
          },
          {
            "parameter": "average_speed",
            "value": 42.5,
            "unit": "km/h",
            "timestamp": "2025-01-15T10:30:00+09:00",
            "quality": "excellent"
          }
        ],
        "specifications": {
          "manufacturer": "TrafficTech Korea",
          "model": "TRF-5000",
          "installDate": "2024-01-10",
          "calibrationDate": "2024-11-01",
          "accuracy": 0.97,
          "updateInterval": 60
        },
        "status": {
          "operational": true,
          "lastMaintenance": "2024-11-01T10:00:00+09:00"
        }
      }
    ]
  }
}
```

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01 | Initial release |

---

<div align="center">

**WIA Digital Twin City Standard v1.0.0**

**弘益人間 (홍익인간)** - Benefit All Humanity

---

**© 2025 WIA**

**MIT License**

</div>
