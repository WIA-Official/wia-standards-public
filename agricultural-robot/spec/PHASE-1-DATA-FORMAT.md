# WIA-AGRI-003: Agricultural Robot Standard
## Phase 1: Data Format Specification

**Version:** 1.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-12-26
**Category:** Agriculture (AGRI)

---

## 1. Overview

### 1.1 Purpose
The WIA-AGRI-003 Agricultural Robot Standard defines a unified data format for agricultural robots including autonomous tractors, harvesting robots, weeding bots, crop monitors, and precision agriculture systems. This standard enables interoperability between different robot platforms, farm management systems, and agricultural IoT ecosystems.

### 1.2 Scope
- Robot profile and capability declaration
- Telemetry and sensor data
- Field mapping and navigation data
- Crop monitoring and health metrics
- Work mode and task management

### 1.3 Philosophy
**弘益人間 (Benefit All Humanity)** - This standard aims to revolutionize agriculture through automation, making precision farming accessible to all farmers regardless of farm size or technological expertise.

---

## 2. Core Data Schema

### 2.1 Agricultural Robot Profile

```json
{
  "@context": "https://wiastandards.com/contexts/agricultural-robot/v1",
  "@type": "AgriculturalRobotProfile",
  "robotId": "string (required)",
  "robotType": "enum (required)",
  "manufacturer": {
    "name": "string",
    "did": "string",
    "certification": "string"
  },
  "specifications": {
    "dimensions": {
      "width": "number (meters)",
      "length": "number (meters)",
      "height": "number (meters)",
      "weight": "number (kg)"
    },
    "powerSystem": {
      "type": "enum",
      "capacity": "number (kWh or liters)",
      "range": "number (km or hours)"
    },
    "capabilities": ["string"],
    "cropTypes": ["string"],
    "maxSpeed": "number (km/h)",
    "workingWidth": "number (meters)"
  },
  "autonomy": {
    "level": "enum",
    "gpsAccuracy": "enum",
    "navigationSystem": "string",
    "obstacleDetection": "boolean"
  },
  "version": "string",
  "timestamp": "ISO8601"
}
```

### 2.2 Field Definitions

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `robotId` | string | Yes | Unique identifier (e.g., `AGRI-ROBOT-2025-001`) |
| `robotType` | enum | Yes | `autonomous-tractor`, `harvesting-robot`, `weeding-bot`, `crop-monitor`, `sprayer-drone`, `seeding-robot` |
| `powerSystem.type` | enum | Yes | `diesel`, `electric`, `hybrid`, `solar` |
| `autonomy.level` | enum | Yes | `0` (manual), `1` (assisted), `2` (partial), `3` (conditional), `4` (high), `5` (full) |
| `gpsAccuracy` | enum | Yes | `standard`, `DGPS`, `RTK`, `PPP` |
| `capabilities` | array | Yes | `plowing`, `harvesting`, `weeding`, `spraying`, `seeding`, `monitoring` |

---

## 3. Telemetry Data Format

### 3.1 Real-time Telemetry

```json
{
  "@context": "https://wiastandards.com/contexts/agricultural-robot/v1",
  "@type": "AgriRobotTelemetry",
  "robotId": "string",
  "timestamp": "ISO8601",
  "location": {
    "latitude": "number",
    "longitude": "number",
    "altitude": "number",
    "accuracy": "string",
    "heading": "number (degrees)"
  },
  "status": {
    "workMode": "enum",
    "battery": "number (0-100)",
    "speed": "number (km/h)",
    "fuelLevel": "number (0-100)",
    "temperature": "number (celsius)"
  },
  "sensors": {
    "soilMoisture": "number (0-100)",
    "soilTemperature": "number (celsius)",
    "airTemperature": "number (celsius)",
    "humidity": "number (0-100)",
    "cropHealth": "number (0-1)",
    "weedDensity": "number (per m²)"
  },
  "cropData": {
    "cropType": "string",
    "growthStage": "string",
    "estimatedYield": "number (kg/hectare)",
    "healthIndex": "number (0-1)"
  },
  "wia": {
    "standard": "WIA-AGRI-003",
    "version": "1.0.0"
  }
}
```

### 3.2 Work Mode Enumeration

| Value | Description | Typical Speed |
|-------|-------------|---------------|
| `idle` | Stationary, engines running | 0 km/h |
| `transit` | Moving between fields | 10-25 km/h |
| `plowing` | Soil preparation | 3-8 km/h |
| `harvesting` | Crop collection | 2-6 km/h |
| `weeding` | Selective weed removal | 1-4 km/h |
| `spraying` | Pesticide/fertilizer application | 5-12 km/h |
| `seeding` | Planting seeds | 4-10 km/h |
| `monitoring` | Crop inspection | 2-8 km/h |

---

## 4. Field Mapping Data

### 4.1 Field Boundary Definition

```json
{
  "@context": "https://wiastandards.com/contexts/agricultural-robot/v1",
  "@type": "FieldMap",
  "fieldId": "string",
  "farmId": "string",
  "boundary": {
    "type": "Polygon",
    "coordinates": [
      [
        ["longitude", "latitude"],
        ["longitude", "latitude"]
      ]
    ]
  },
  "area": "number (hectares)",
  "cropType": "string",
  "obstacles": [
    {
      "type": "enum",
      "location": {"latitude": "number", "longitude": "number"},
      "radius": "number (meters)"
    }
  ],
  "zones": [
    {
      "zoneId": "string",
      "type": "enum",
      "polygon": "GeoJSON Polygon"
    }
  ]
}
```

### 4.2 Obstacle Types

- `tree` - Individual trees
- `rock` - Large rocks or boulders
- `building` - Structures
- `water` - Ponds, streams
- `slope` - Steep terrain
- `fence` - Boundary fences
- `power-line` - Overhead cables

---

## 5. Task Management Data

### 5.1 Task Assignment

```json
{
  "@context": "https://wiastandards.com/contexts/agricultural-robot/v1",
  "@type": "AgriTask",
  "taskId": "string",
  "robotId": "string",
  "fieldId": "string",
  "taskType": "enum",
  "priority": "enum",
  "scheduledStart": "ISO8601",
  "estimatedDuration": "number (minutes)",
  "parameters": {
    "workingDepth": "number (cm)",
    "sprayRate": "number (liters/hectare)",
    "seedingDensity": "number (seeds/m²)"
  },
  "status": "enum",
  "progress": "number (0-100)"
}
```

### 5.2 Task Status Lifecycle

```
pending → assigned → in-progress → paused → in-progress → completed
                                  ↘ cancelled
                                  ↘ failed
```

---

## 6. Crop Health Monitoring

### 6.1 Crop Health Data

```json
{
  "@context": "https://wiastandards.com/contexts/agricultural-robot/v1",
  "@type": "CropHealthReport",
  "reportId": "string",
  "fieldId": "string",
  "timestamp": "ISO8601",
  "cropType": "string",
  "growthStage": "string",
  "measurements": {
    "ndvi": "number (0-1)",
    "leafAreaIndex": "number",
    "chlorophyllContent": "number (μg/cm²)",
    "plantHeight": "number (cm)",
    "canopyCover": "number (0-100)"
  },
  "issues": [
    {
      "type": "enum",
      "severity": "enum",
      "location": "GeoJSON Point",
      "area": "number (m²)"
    }
  ]
}
```

### 6.2 Issue Types

- `pest-infestation` - Insect damage
- `disease` - Plant diseases
- `weed-pressure` - Weed competition
- `nutrient-deficiency` - Lack of nutrients
- `water-stress` - Drought or overwatering
- `physical-damage` - Mechanical damage

---

## 7. Fleet Coordination Data

### 7.1 Fleet Status

```json
{
  "@context": "https://wiastandards.com/contexts/agricultural-robot/v1",
  "@type": "FleetStatus",
  "farmId": "string",
  "timestamp": "ISO8601",
  "robots": [
    {
      "robotId": "string",
      "status": "enum",
      "currentTask": "string",
      "location": "GeoJSON Point",
      "battery": "number (0-100)"
    }
  ],
  "coordination": {
    "collisionAvoidanceActive": "boolean",
    "workloadBalancing": "boolean",
    "energyOptimization": "boolean"
  }
}
```

---

## 8. Integration with WIA Ecosystem

### 8.1 Verifiable Credentials

Agricultural robots can issue and verify credentials for:
- Crop certification (organic, sustainable)
- Treatment history (pesticides, fertilizers)
- Yield verification
- Environmental compliance

### 8.2 Blockchain Integration

- Immutable record of field operations
- Supply chain traceability
- Carbon credit calculation
- Regulatory compliance tracking

---

## 9. Data Privacy and Security

### 9.1 Privacy Requirements

- Farm location data must be anonymized for public sharing
- Yield data is proprietary to farm owner
- Robot telemetry can be shared with manufacturer for maintenance
- Crop health data can be aggregated for research (with consent)

### 9.2 Security Standards

- End-to-end encryption for command and control
- Digital signatures for task assignments
- Role-based access control
- Audit logging for all data access

---

## 10. Compliance and Interoperability

### 10.1 Standards Compliance

- ISO 11783 (ISOBUS) compatibility
- ROS (Robot Operating System) message compatibility
- OGC GeoJSON for spatial data
- ISO 8601 for timestamps

### 10.2 Related WIA Standards

- **WIA-INTENT** - Natural language task commands
- **WIA-OMNI-API** - Unified API gateway
- **WIA-BLOCKCHAIN** - Immutable data logging
- **WIA-IOT** - Sensor network integration

---

## 11. Example Use Cases

### 11.1 Autonomous Harvesting

```json
{
  "@type": "AgriRobotTelemetry",
  "robotId": "HARVESTER-2025-042",
  "robotType": "harvesting-robot",
  "workMode": "harvesting",
  "cropData": {
    "cropType": "wheat",
    "growthStage": "ripe",
    "estimatedYield": 4500
  },
  "sensors": {
    "cropHealth": 0.92,
    "grainMoisture": 14.5
  }
}
```

### 11.2 Precision Weeding

```json
{
  "@type": "AgriTask",
  "taskType": "weeding",
  "parameters": {
    "method": "mechanical",
    "selectivity": "high",
    "weedSpecies": ["pigweed", "lambsquarters"]
  }
}
```

---

## 12. Future Extensions

### Phase 2: API Interface
- RESTful API endpoints
- Real-time WebSocket feeds
- GraphQL queries

### Phase 3: Protocol
- ROS2 topic definitions
- CAN bus protocols
- MQTT broker configuration

### Phase 4: Integration
- Farm management system connectors
- Weather API integration
- Market price feeds

---

**© 2025 WIA (World Certification Industry Association)**
**弘益人間 · Benefit All Humanity**
**License:** MIT
