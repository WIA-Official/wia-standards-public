# WIA-AGRI-001: Smart Farm Standard
## Phase 1 - Data Format Specification

**Version:** 1.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-01-01

---

## 1. Overview

This specification defines standardized data formats for smart farm sensor networks, environmental monitoring, crop management, and agricultural automation systems.

### 1.1 Design Principles

- **Interoperability**: Compatible with major IoT platforms (AWS IoT, Azure IoT Hub, Google Cloud IoT)
- **Scalability**: Support from small farms (100㎡) to large agricultural operations (100+ hectares)
- **Real-time**: Sub-second latency for critical alerts (frost, drought, pests)
- **Extensibility**: Easy addition of new sensor types and crop-specific parameters
- **Sustainability**: Carbon footprint tracking and resource optimization metrics

---

## 2. Core Data Structures

### 2.1 Farm Entity

```json
{
  "farmId": "string (UUID)",
  "farmName": "string",
  "location": {
    "latitude": "number (decimal degrees)",
    "longitude": "number (decimal degrees)",
    "elevation": "number (meters)",
    "address": "string",
    "country": "string (ISO 3166-1 alpha-2)",
    "timezone": "string (IANA timezone)"
  },
  "area": {
    "total": "number (square meters)",
    "cultivated": "number (square meters)",
    "greenhouse": "number (square meters)",
    "openField": "number (square meters)"
  },
  "certifications": ["string (certification IDs)"],
  "establishedDate": "string (ISO 8601)",
  "owner": {
    "name": "string",
    "contact": "string",
    "did": "string (W3C DID)"
  }
}
```

### 2.2 Sensor Data Packet

```json
{
  "sensorId": "string (UUID)",
  "farmId": "string (UUID)",
  "zoneId": "string (zone identifier)",
  "timestamp": "string (ISO 8601 with timezone)",
  "sensorType": "string (enum: soil, air, water, light, camera)",
  "measurements": {
    "soil": {
      "moisture": "number (percentage 0-100)",
      "temperature": "number (Celsius)",
      "pH": "number (0-14)",
      "ec": "number (dS/m - electrical conductivity)",
      "nitrogen": "number (ppm)",
      "phosphorus": "number (ppm)",
      "potassium": "number (ppm)",
      "organicMatter": "number (percentage)"
    },
    "air": {
      "temperature": "number (Celsius)",
      "humidity": "number (percentage 0-100)",
      "co2": "number (ppm)",
      "pressure": "number (hPa)",
      "windSpeed": "number (m/s)",
      "windDirection": "number (degrees 0-360)"
    },
    "light": {
      "intensity": "number (lux)",
      "par": "number (μmol/m²/s - Photosynthetically Active Radiation)",
      "uvIndex": "number (0-11+)",
      "spectrum": {
        "red": "number (nm)",
        "blue": "number (nm)",
        "green": "number (nm)",
        "farRed": "number (nm)"
      }
    },
    "water": {
      "flowRate": "number (L/min)",
      "totalVolume": "number (L)",
      "pH": "number (0-14)",
      "ec": "number (dS/m)",
      "temperature": "number (Celsius)",
      "dissolved02": "number (mg/L)"
    }
  },
  "quality": {
    "accuracy": "number (percentage)",
    "calibrationDate": "string (ISO 8601)",
    "batteryLevel": "number (percentage 0-100)"
  },
  "metadata": {
    "deviceManufacturer": "string",
    "deviceModel": "string",
    "firmwareVersion": "string"
  }
}
```

### 2.3 Crop Data

```json
{
  "cropId": "string (UUID)",
  "farmId": "string (UUID)",
  "zoneId": "string",
  "cropType": "string (enum: tomato, lettuce, strawberry, cucumber, etc.)",
  "variety": "string",
  "plantingDate": "string (ISO 8601)",
  "expectedHarvestDate": "string (ISO 8601)",
  "growthStage": "string (enum: seedling, vegetative, flowering, fruiting, harvest)",
  "plantCount": "number",
  "density": "number (plants per square meter)",
  "health": {
    "status": "string (enum: healthy, stressed, diseased, pest-infested)",
    "ndvi": "number (0-1 - Normalized Difference Vegetation Index)",
    "chlorophyllContent": "number (μg/cm²)",
    "leafAreaIndex": "number",
    "diseaseDetected": ["string (disease names)"],
    "pestDetected": ["string (pest names)"]
  },
  "yield": {
    "estimated": "number (kg)",
    "actual": "number (kg)",
    "quality": "string (enum: premium, standard, substandard)"
  }
}
```

### 2.4 Automation Event

```json
{
  "eventId": "string (UUID)",
  "farmId": "string (UUID)",
  "timestamp": "string (ISO 8601)",
  "eventType": "string (enum: irrigation, fertilization, pesticide, hvac, lighting)",
  "trigger": {
    "type": "string (enum: manual, scheduled, sensor-triggered, ai-optimized)",
    "condition": "string (description of trigger condition)",
    "sensorData": "object (reference to sensor readings)"
  },
  "action": {
    "irrigationEvent": {
      "zoneId": "string",
      "duration": "number (minutes)",
      "volume": "number (liters)",
      "waterSource": "string (enum: municipal, well, rainwater, recycled)"
    },
    "fertilizationEvent": {
      "zoneId": "string",
      "fertilizerType": "string",
      "npkRatio": "string (e.g., '10-10-10')",
      "quantity": "number (kg or L)",
      "applicationMethod": "string (enum: drip, spray, granular)"
    },
    "pesticideEvent": {
      "zoneId": "string",
      "activeIngredient": "string",
      "concentration": "number (percentage)",
      "quantity": "number (L)",
      "targetPest": "string"
    },
    "hvacEvent": {
      "targetTemperature": "number (Celsius)",
      "targetHumidity": "number (percentage)",
      "fanSpeed": "number (percentage 0-100)",
      "heatingPower": "number (kW)",
      "coolingPower": "number (kW)"
    },
    "lightingEvent": {
      "intensity": "number (percentage 0-100)",
      "spectrum": "string (enum: full-spectrum, red-blue, custom)",
      "duration": "number (hours)",
      "schedule": "string (cron expression)"
    }
  },
  "result": {
    "status": "string (enum: success, partial, failed)",
    "message": "string",
    "energyConsumed": "number (kWh)",
    "costEstimate": "number (currency)"
  }
}
```

### 2.5 Alert/Notification

```json
{
  "alertId": "string (UUID)",
  "farmId": "string (UUID)",
  "timestamp": "string (ISO 8601)",
  "severity": "string (enum: critical, warning, info)",
  "category": "string (enum: weather, pest, disease, equipment, resource)",
  "title": "string",
  "description": "string",
  "affectedZones": ["string (zone IDs)"],
  "recommendations": ["string"],
  "requiresAction": "boolean",
  "actionDeadline": "string (ISO 8601)",
  "resolvedAt": "string (ISO 8601 or null)"
}
```

---

## 3. Data Exchange Formats

### 3.1 JSON (Primary)

- **Use Case**: REST APIs, cloud storage, data lakes
- **Encoding**: UTF-8
- **Compression**: gzip recommended for large payloads
- **File Extension**: `.json`

### 3.2 CBOR (Binary JSON)

- **Use Case**: IoT device communication, bandwidth-limited networks
- **Advantage**: 30-50% smaller than JSON
- **Libraries**: `cbor` (Python), `cbor-js` (JavaScript)

### 3.3 Protocol Buffers

- **Use Case**: High-frequency sensor data streams
- **Schema Definition**: `.proto` files (see Phase 3)
- **Advantage**: Strongly typed, backward compatible

### 3.4 CSV (Legacy/Export)

- **Use Case**: Data export for spreadsheet analysis
- **Format**: RFC 4180 compliant
- **Headers**: Required in first row

---

## 4. Time Series Data

### 4.1 Format

```json
{
  "farmId": "string (UUID)",
  "sensorId": "string (UUID)",
  "metric": "string (e.g., 'soil.moisture')",
  "aggregation": "string (enum: raw, 1min, 5min, 1hour, 1day)",
  "dataPoints": [
    {
      "timestamp": "number (Unix timestamp in milliseconds)",
      "value": "number",
      "quality": "string (enum: good, fair, poor, missing)"
    }
  ]
}
```

### 4.2 Recommended Storage

- **InfluxDB**: Time-series optimized, supports retention policies
- **TimescaleDB**: PostgreSQL extension for time-series
- **AWS Timestream**: Managed time-series database
- **Prometheus**: Monitoring and alerting

---

## 5. Data Quality Standards

### 5.1 Sensor Calibration

| Sensor Type | Calibration Frequency | Acceptable Drift |
|-------------|----------------------|------------------|
| Soil Moisture | Every 3 months | ±3% |
| pH Sensors | Every 1 month | ±0.2 pH |
| Temperature | Every 6 months | ±0.5°C |
| Humidity | Every 6 months | ±5% |
| Light (PAR) | Every 6 months | ±10% |

### 5.2 Data Validation Rules

```javascript
// Example validation
{
  "soil.moisture": { "min": 0, "max": 100 },
  "soil.pH": { "min": 0, "max": 14 },
  "air.temperature": { "min": -40, "max": 60 },
  "air.humidity": { "min": 0, "max": 100 },
  "light.intensity": { "min": 0, "max": 200000 }
}
```

### 5.3 Missing Data Handling

- **Strategy 1**: Linear interpolation (for short gaps < 15 minutes)
- **Strategy 2**: Forward fill (for categorical data)
- **Strategy 3**: Mark as missing, do not impute (for critical decisions)

---

## 6. Privacy & Security

### 6.1 Data Classification

| Data Type | Classification | Encryption Required |
|-----------|---------------|---------------------|
| Sensor readings | Public | No |
| Farm location (precise) | Sensitive | TLS in transit |
| Owner information | Personal | At rest + in transit |
| Financial data | Confidential | At rest + in transit |

### 6.2 Anonymization

```json
{
  "farmId": "hashed_uuid",
  "location": {
    "latitude": "rounded_to_2_decimals",
    "longitude": "rounded_to_2_decimals"
  }
}
```

---

## 7. Examples

### 7.1 Complete Sensor Reading

```json
{
  "sensorId": "550e8400-e29b-41d4-a716-446655440001",
  "farmId": "123e4567-e89b-12d3-a456-426614174000",
  "zoneId": "greenhouse-A",
  "timestamp": "2025-01-01T10:30:00+09:00",
  "sensorType": "soil",
  "measurements": {
    "soil": {
      "moisture": 68.5,
      "temperature": 22.3,
      "pH": 6.4,
      "ec": 1.8,
      "nitrogen": 120,
      "phosphorus": 45,
      "potassium": 200,
      "organicMatter": 4.2
    }
  },
  "quality": {
    "accuracy": 98.5,
    "calibrationDate": "2024-12-01",
    "batteryLevel": 85
  },
  "metadata": {
    "deviceManufacturer": "FarmSense",
    "deviceModel": "FS-SOIL-PRO",
    "firmwareVersion": "2.3.1"
  }
}
```

### 7.2 Irrigation Event

```json
{
  "eventId": "e7b8c9d0-1234-5678-9abc-def012345678",
  "farmId": "123e4567-e89b-12d3-a456-426614174000",
  "timestamp": "2025-01-01T06:00:00+09:00",
  "eventType": "irrigation",
  "trigger": {
    "type": "ai-optimized",
    "condition": "Soil moisture below 60%, no rain forecast for 48h",
    "sensorData": {
      "sensorId": "550e8400-e29b-41d4-a716-446655440001",
      "soilMoisture": 58.2
    }
  },
  "action": {
    "irrigationEvent": {
      "zoneId": "greenhouse-A",
      "duration": 15,
      "volume": 450,
      "waterSource": "rainwater"
    }
  },
  "result": {
    "status": "success",
    "message": "Irrigation completed successfully",
    "energyConsumed": 0.75,
    "costEstimate": 0.05
  }
}
```

---

## 8. Implementation Checklist

- [ ] Define farm entity schema
- [ ] Implement sensor data packet structure
- [ ] Set up time-series database
- [ ] Configure data validation rules
- [ ] Implement encryption for sensitive data
- [ ] Create data export utilities (JSON, CSV)
- [ ] Set up data retention policies
- [ ] Implement backup and disaster recovery

---

## 9. Compliance

### 9.1 Standards Compatibility

- **ISO 11783** (ISOBUS) - Tractor and machinery communication
- **OGC SensorThings API** - Sensor data interoperability
- **PROV-O** (W3C) - Data provenance tracking

### 9.2 Agricultural Regulations

- **GAP** (Good Agricultural Practices)
- **Organic certification** data requirements
- **Pesticide application** record keeping

---

## 10. Revision History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01-01 | Initial release |

---

**Next Phase:** [PHASE-2-API-INTERFACE.md](./PHASE-2-API-INTERFACE.md)

---

© 2025 WIA (World Certification Industry Association)
弘益人間 · Benefit All Humanity
