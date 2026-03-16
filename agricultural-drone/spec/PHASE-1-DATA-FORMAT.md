# WIA-AGRI-004: Agricultural Drone Standard
## Phase 1: Data Format Specification

**Version:** 1.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-01-01

---

## 1. Overview

This specification defines the data formats for agricultural drone operations, including flight telemetry, sensor data, imagery metadata, and mission planning.

### 1.1 Design Principles

- **Interoperability**: Standard JSON/XML formats compatible with major agricultural platforms
- **Extensibility**: Support for custom fields and future sensor types
- **Efficiency**: Optimized for bandwidth-constrained rural networks
- **Traceability**: Complete audit trail for regulatory compliance

---

## 2. Core Data Structures

### 2.1 Flight Telemetry Data

Real-time drone flight information transmitted during operations.

```json
{
  "telemetry": {
    "droneId": "AGRI-DRONE-001",
    "timestamp": "2025-01-01T10:30:45.123Z",
    "location": {
      "latitude": 37.5665,
      "longitude": 126.9780,
      "altitude": 5.2,
      "altitudeType": "AGL",
      "accuracy": 0.5
    },
    "orientation": {
      "roll": 2.3,
      "pitch": -1.5,
      "yaw": 87.4
    },
    "velocity": {
      "groundSpeed": 3.5,
      "verticalSpeed": 0.1,
      "heading": 87.4
    },
    "battery": {
      "voltage": 22.4,
      "current": 15.2,
      "percentage": 78,
      "remainingFlightTime": 18
    },
    "status": "SPRAYING",
    "satellites": 14,
    "hdop": 0.8
  }
}
```

**Field Descriptions:**
- `droneId`: Unique identifier for the drone
- `timestamp`: ISO 8601 format timestamp
- `location.altitudeType`: "AGL" (Above Ground Level) or "AMSL" (Above Mean Sea Level)
- `status`: One of: IDLE, TAKEOFF, FLYING, SPRAYING, IMAGING, LANDING, EMERGENCY

### 2.2 Mission Data Format

Pre-flight mission planning and waypoint configuration.

```json
{
  "mission": {
    "missionId": "MISSION-2025-001",
    "farmId": "FARM-KR-001",
    "operator": {
      "name": "Kim Nongbu",
      "licenseNumber": "KR-AGRI-2025-001",
      "contactPhone": "+82-10-1234-5678"
    },
    "drone": {
      "model": "WIA-AGRI-X1",
      "serialNumber": "WAX1-2024-0123",
      "maxPayload": 10,
      "tankCapacity": 10
    },
    "missionType": "CROP_SPRAYING",
    "createdAt": "2025-01-01T09:00:00Z",
    "scheduledStart": "2025-01-01T10:00:00Z",
    "estimatedDuration": 45,
    "waypoints": [
      {
        "seq": 0,
        "latitude": 37.5665,
        "longitude": 126.9780,
        "altitude": 5.0,
        "action": "TAKEOFF",
        "speed": 2.0
      },
      {
        "seq": 1,
        "latitude": 37.5675,
        "longitude": 126.9790,
        "altitude": 5.0,
        "action": "SPRAY",
        "speed": 3.5,
        "sprayRate": 1.5
      }
    ],
    "fieldBoundary": {
      "type": "Polygon",
      "coordinates": [
        [[126.9780, 37.5665], [126.9790, 37.5665],
         [126.9790, 37.5675], [126.9780, 37.5675],
         [126.9780, 37.5665]]
      ]
    },
    "fieldArea": 10.5,
    "cropType": "RICE",
    "targetPest": "BROWN_PLANTHOPPER"
  }
}
```

### 2.3 Spray Operation Data

Detailed records of pesticide/fertilizer application.

```json
{
  "sprayOperation": {
    "operationId": "SPRAY-2025-001",
    "missionId": "MISSION-2025-001",
    "timestamp": "2025-01-01T10:30:45Z",
    "chemical": {
      "name": "Organic Pesticide A",
      "registrationNumber": "KR-PEST-2024-123",
      "activeIngredient": "Neem Oil",
      "concentration": 2.5,
      "unit": "percent"
    },
    "application": {
      "tankVolume": 10.0,
      "sprayRate": 15.0,
      "flowRate": 1.5,
      "nozzleType": "FLAT_FAN_110_03",
      "nozzleCount": 4,
      "swathWidth": 4.5,
      "targetCoverage": 10.5
    },
    "environmental": {
      "temperature": 22.5,
      "humidity": 65,
      "windSpeed": 2.3,
      "windDirection": 180,
      "precipitation": 0
    },
    "coverage": [
      {
        "timestamp": "2025-01-01T10:30:45Z",
        "location": [126.9780, 37.5665],
        "sprayActive": true,
        "flowRate": 1.5,
        "altitude": 5.2
      }
    ],
    "totalAreaCovered": 10.3,
    "totalVolumeSprayed": 155.0,
    "efficiency": 98.1
  }
}
```

### 2.4 Imagery Metadata

Metadata for aerial images and multispectral data.

```json
{
  "imagery": {
    "imageId": "IMG-2025-001-0123",
    "missionId": "MISSION-2025-001",
    "timestamp": "2025-01-01T10:35:22.456Z",
    "camera": {
      "model": "AGRI-CAM-MS5",
      "type": "MULTISPECTRAL",
      "resolution": "4000x3000",
      "fov": 73.7,
      "sensorSize": "1/2.3",
      "focalLength": 5.4
    },
    "location": {
      "latitude": 37.5670,
      "longitude": 126.9785,
      "altitude": 50.0,
      "altitudeType": "AGL"
    },
    "gimbal": {
      "roll": 0.0,
      "pitch": -90.0,
      "yaw": 0.0
    },
    "bands": [
      {
        "name": "RED",
        "wavelength": 650,
        "bandwidth": 40,
        "fileUrl": "s3://agri-data/img-001-red.tif"
      },
      {
        "name": "NIR",
        "wavelength": 850,
        "bandwidth": 40,
        "fileUrl": "s3://agri-data/img-001-nir.tif"
      },
      {
        "name": "GREEN",
        "wavelength": 560,
        "bandwidth": 40,
        "fileUrl": "s3://agri-data/img-001-green.tif"
      }
    ],
    "quality": {
      "brightness": 128,
      "contrast": 1.2,
      "sharpness": 0.85,
      "cloudCover": 5
    },
    "processing": {
      "rawFormat": "RAW12",
      "outputFormat": "GEOTIFF",
      "calibrated": true,
      "orthorectified": false
    }
  }
}
```

### 2.5 NDVI Analysis Data

Normalized Difference Vegetation Index calculation results.

```json
{
  "ndviAnalysis": {
    "analysisId": "NDVI-2025-001",
    "imageId": "IMG-2025-001-0123",
    "fieldId": "FIELD-KR-001",
    "timestamp": "2025-01-01T11:00:00Z",
    "algorithm": "STANDARD_NDVI",
    "version": "1.0",
    "statistics": {
      "mean": 0.65,
      "median": 0.67,
      "stdDev": 0.12,
      "min": 0.25,
      "max": 0.85,
      "percentiles": {
        "p25": 0.58,
        "p50": 0.67,
        "p75": 0.74
      }
    },
    "healthZones": [
      {
        "category": "EXCELLENT",
        "ndviRange": [0.7, 0.85],
        "area": 4.5,
        "percentage": 42.8,
        "color": "#00FF00"
      },
      {
        "category": "GOOD",
        "ndviRange": [0.5, 0.7],
        "area": 5.2,
        "percentage": 49.5,
        "color": "#90EE90"
      },
      {
        "category": "MODERATE",
        "ndviRange": [0.3, 0.5],
        "area": 0.6,
        "percentage": 5.7,
        "color": "#FFFF00"
      },
      {
        "category": "POOR",
        "ndviRange": [0.0, 0.3],
        "area": 0.2,
        "percentage": 1.9,
        "color": "#FF0000"
      }
    ],
    "recommendations": [
      {
        "zone": "POOR",
        "action": "INVESTIGATE_SOIL",
        "priority": "HIGH",
        "details": "Check for drainage issues or pest damage"
      },
      {
        "zone": "MODERATE",
        "action": "APPLY_FERTILIZER",
        "priority": "MEDIUM",
        "details": "Consider nitrogen supplementation"
      }
    ],
    "outputFiles": {
      "ndviMap": "s3://agri-data/ndvi-001-map.tif",
      "falseColor": "s3://agri-data/ndvi-001-false.png",
      "report": "s3://agri-data/ndvi-001-report.pdf"
    }
  }
}
```

---

## 3. Binary Data Formats

### 3.1 Flight Log Binary Format

For efficient storage of high-frequency telemetry data.

```
Header (64 bytes):
  - Magic Number: 0x57494144 ("WIAD")
  - Version: uint16
  - Record Count: uint32
  - Start Timestamp: uint64
  - Drone ID: char[32]
  - Reserved: 16 bytes

Record (48 bytes):
  - Timestamp Offset: uint32 (milliseconds from start)
  - Latitude: int32 (degrees * 1e7)
  - Longitude: int32 (degrees * 1e7)
  - Altitude: int16 (meters * 10)
  - Speed: uint16 (m/s * 100)
  - Heading: uint16 (degrees * 100)
  - Battery: uint8 (percentage)
  - Status: uint8
  - Spray Active: bool
  - Reserved: 31 bytes
```

### 3.2 Image Tile Format

For efficient transmission of large aerial images.

```
Tile Header (32 bytes):
  - Magic: 0x57494154 ("WIAT")
  - Tile X: uint16
  - Tile Y: uint16
  - Tile Width: uint16
  - Tile Height: uint16
  - Compression: uint8 (0=None, 1=JPEG, 2=PNG, 3=WebP)
  - Quality: uint8
  - Data Length: uint32
  - CRC32: uint32
  - Reserved: 12 bytes

Tile Data:
  - Compressed image data
```

---

## 4. Field Mapping Data

### 4.1 Field Boundary Format (GeoJSON)

```json
{
  "type": "FeatureCollection",
  "crs": {
    "type": "name",
    "properties": {
      "name": "urn:ogc:def:crs:EPSG::4326"
    }
  },
  "features": [
    {
      "type": "Feature",
      "properties": {
        "fieldId": "FIELD-KR-001",
        "farmId": "FARM-KR-001",
        "cropType": "RICE",
        "plantingDate": "2025-04-15",
        "area": 10.5,
        "perimeter": 450.2,
        "elevation": 25.5,
        "soilType": "CLAY_LOAM"
      },
      "geometry": {
        "type": "Polygon",
        "coordinates": [
          [
            [126.9780, 37.5665],
            [126.9790, 37.5665],
            [126.9790, 37.5675],
            [126.9780, 37.5675],
            [126.9780, 37.5665]
          ]
        ]
      }
    }
  ]
}
```

---

## 5. Weather Data Integration

### 5.1 Weather Conditions Format

```json
{
  "weather": {
    "timestamp": "2025-01-01T10:00:00Z",
    "location": {
      "latitude": 37.5665,
      "longitude": 126.9780
    },
    "current": {
      "temperature": 22.5,
      "humidity": 65,
      "pressure": 1013.25,
      "windSpeed": 2.3,
      "windDirection": 180,
      "windGust": 3.5,
      "precipitation": 0,
      "cloudCover": 15,
      "visibility": 10000,
      "uvIndex": 5
    },
    "forecast": [
      {
        "time": "2025-01-01T11:00:00Z",
        "temperature": 23.0,
        "windSpeed": 2.5,
        "precipitation": 0,
        "condition": "CLEAR"
      }
    ],
    "sprayConditions": {
      "suitable": true,
      "score": 85,
      "warnings": [],
      "recommendations": [
        "Optimal spraying conditions",
        "Low wind - minimal drift risk"
      ]
    }
  }
}
```

---

## 6. Compliance & Regulatory Data

### 6.1 Flight Authorization Record

```json
{
  "authorization": {
    "authId": "AUTH-2025-001",
    "missionId": "MISSION-2025-001",
    "issuedBy": "KR-AVIATION-AUTHORITY",
    "issuedAt": "2025-01-01T08:00:00Z",
    "validFrom": "2025-01-01T09:00:00Z",
    "validUntil": "2025-01-01T18:00:00Z",
    "flightArea": {
      "type": "Polygon",
      "coordinates": [[[126.9780, 37.5665], [126.9790, 37.5675]]]
    },
    "maxAltitude": 30,
    "restrictions": [
      "No flights within 500m of residential areas",
      "Maintain visual line of sight",
      "Weather conditions must be favorable"
    ],
    "operator": {
      "name": "Kim Nongbu",
      "license": "KR-AGRI-2025-001",
      "organization": "Green Farm Co."
    },
    "signature": "BASE64_ENCODED_SIGNATURE"
  }
}
```

---

## 7. Data Validation Rules

### 7.1 Required Fields

**Telemetry Data:**
- droneId, timestamp, location (lat, lon, alt), status

**Mission Data:**
- missionId, farmId, operator, missionType, waypoints

**Spray Operation:**
- operationId, chemical, application, environmental

### 7.2 Value Constraints

```yaml
constraints:
  location:
    latitude: [-90, 90]
    longitude: [-180, 180]
    altitude: [0, 150]  # meters AGL

  spray:
    flowRate: [0.1, 10.0]  # liters per minute
    sprayRate: [5, 50]  # liters per hectare

  environmental:
    temperature: [-10, 45]  # Celsius
    humidity: [0, 100]  # percentage
    windSpeed: [0, 15]  # m/s (max safe for spraying: 10)

  battery:
    voltage: [18.0, 26.0]  # for 6S LiPo
    percentage: [0, 100]
```

---

## 8. Data Storage Recommendations

### 8.1 Time-Series Database (InfluxDB)

```sql
-- Measurement: flight_telemetry
timestamp, droneId, latitude, longitude, altitude, speed, battery_pct

-- Measurement: spray_operations
timestamp, operationId, flowRate, tankLevel, sprayActive

-- Measurement: environmental
timestamp, temperature, humidity, windSpeed, windDirection
```

### 8.2 Document Database (MongoDB)

```javascript
// Collections
db.missions
db.images
db.ndvi_analyses
db.spray_operations
db.authorizations
```

---

## 9. API Response Envelope

All API responses use this standard envelope:

```json
{
  "success": true,
  "timestamp": "2025-01-01T10:00:00Z",
  "version": "1.0.0",
  "data": {
    // Actual response data
  },
  "meta": {
    "page": 1,
    "pageSize": 50,
    "total": 234
  },
  "errors": []
}
```

---

## 10. Changelog

**v1.0.0 (2025-01-01)**
- Initial release
- Core telemetry, mission, spray, imagery formats
- NDVI analysis data structure
- Weather and regulatory data formats

---

**© 2025 WIA Standards | MIT License**
**弘益人間 · Benefit All Humanity**
