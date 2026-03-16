# WIA-AGRI-020: Agricultural Data Exchange Standard
## Phase 1: Data Format Specification

**Version:** 1.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-12-26

---

## 1. Overview

This specification defines standardized data formats for agricultural data exchange across IoT sensors, farm management systems, weather APIs, and analytics platforms.

### 1.1 Design Principles

- **Interoperability**: JSON/XML formats compatible with all major agricultural platforms
- **Extensibility**: Custom field support for future sensor types and data sources
- **Efficiency**: Optimized for rural bandwidth constraints and edge computing
- **Validation**: Built-in data quality checks and schema validation
- **Privacy**: GDPR-compliant data handling with anonymization support

---

## 2. Core Data Structures

### 2.1 Sensor Reading Format

Standard format for all agricultural sensor data submissions.

```json
{
  "dataExchange": {
    "standardVersion": "WIA-AGRI-020-v1.0",
    "dataId": "DATA-20251226-001",
    "timestamp": "2025-12-26T10:30:00.000Z",
    "farmId": "FARM-2025-001",
    "location": {
      "latitude": 37.5665,
      "longitude": 126.9780,
      "elevation": 85.3,
      "fieldId": "FIELD-NORTH-01",
      "zone": "A1"
    },
    "dataType": "SOIL_SENSOR",
    "source": {
      "deviceId": "SENSOR-SOIL-001",
      "manufacturer": "AgriTech Inc.",
      "model": "SoilPro-X200",
      "firmwareVersion": "2.1.4"
    },
    "readings": {
      "soilMoisture": {
        "value": 45.5,
        "unit": "percentage",
        "depth": 15,
        "depthUnit": "cm"
      },
      "soilTemperature": {
        "value": 18.3,
        "unit": "celsius"
      },
      "soilPH": {
        "value": 6.8,
        "unit": "ph"
      },
      "soilEC": {
        "value": 1.2,
        "unit": "dS/m"
      },
      "nutrients": {
        "nitrogen": 45,
        "phosphorus": 23,
        "potassium": 156,
        "unit": "ppm"
      }
    },
    "metadata": {
      "quality": "HIGH",
      "validated": true,
      "calibrationDate": "2025-12-01",
      "sampleSize": 1,
      "confidence": 95.5
    }
  }
}
```

**Field Descriptions:**
- `dataId`: Unique identifier for this data submission
- `timestamp`: ISO 8601 UTC timestamp
- `dataType`: SOIL_SENSOR, WEATHER, CROP_HEALTH, EQUIPMENT, YIELD, IRRIGATION
- `metadata.quality`: HIGH, MEDIUM, LOW based on sensor accuracy
- `readings`: Flexible object supporting multiple measurement types

### 2.2 Weather Data Format

Standardized weather information for agricultural decision-making.

```json
{
  "dataExchange": {
    "standardVersion": "WIA-AGRI-020-v1.0",
    "dataId": "WEATHER-20251226-001",
    "timestamp": "2025-12-26T10:00:00.000Z",
    "farmId": "FARM-2025-001",
    "location": {
      "latitude": 37.5665,
      "longitude": 126.9780
    },
    "dataType": "WEATHER",
    "source": {
      "provider": "OpenWeather",
      "stationId": "KR-SEOUL-001",
      "stationType": "AUTOMATED"
    },
    "current": {
      "temperature": 24.5,
      "feelsLike": 23.8,
      "humidity": 65,
      "pressure": 1013.2,
      "windSpeed": 3.5,
      "windDirection": 225,
      "windGust": 5.2,
      "precipitation": 0.0,
      "uvIndex": 6,
      "visibility": 10000,
      "cloudCover": 45,
      "dewPoint": 17.2,
      "weatherCondition": "PARTLY_CLOUDY"
    },
    "forecast": {
      "nextHour": {
        "precipitation": 0.0,
        "probability": 10
      },
      "next24Hours": {
        "minTemp": 18.5,
        "maxTemp": 27.3,
        "precipitationTotal": 2.3,
        "avgHumidity": 68
      },
      "next7Days": {
        "growingDegreeDays": 145,
        "totalPrecipitation": 45.2,
        "avgSolarRadiation": 18.5
      }
    },
    "agronomic": {
      "evapotranspiration": 4.2,
      "soilMoistureIndex": 0.72,
      "frostRisk": "LOW",
      "heatStress": "MODERATE",
      "sprayingConditions": "FAVORABLE"
    }
  }
}
```

### 2.3 Crop Health Data Format

Plant health monitoring data from visual inspection, sensors, and imaging.

```json
{
  "dataExchange": {
    "standardVersion": "WIA-AGRI-020-v1.0",
    "dataId": "CROP-20251226-001",
    "timestamp": "2025-12-26T14:30:00.000Z",
    "farmId": "FARM-2025-001",
    "location": {
      "fieldId": "FIELD-NORTH-01",
      "zone": "A1",
      "area": 2.5,
      "areaUnit": "hectares"
    },
    "dataType": "CROP_HEALTH",
    "cropInfo": {
      "species": "Oryza sativa",
      "variety": "Koshihikari",
      "commonName": "Rice",
      "plantingDate": "2025-05-15",
      "growthStage": "FLOWERING",
      "daysAfterPlanting": 223
    },
    "healthMetrics": {
      "ndvi": {
        "value": 0.78,
        "status": "HEALTHY"
      },
      "lai": {
        "value": 4.2,
        "description": "Leaf Area Index"
      },
      "chlorophyll": {
        "value": 45.3,
        "unit": "SPAD"
      },
      "biomass": {
        "value": 3.2,
        "unit": "kg/m2"
      }
    },
    "stress": {
      "waterStress": "LOW",
      "nutrientStress": "NONE",
      "diseasePresence": false,
      "pestPresence": false,
      "weedPressure": "LOW"
    },
    "predictions": {
      "yieldEstimate": 8.5,
      "yieldUnit": "tons/hectare",
      "harvestDate": "2025-10-15",
      "quality": "GRADE_A"
    }
  }
}
```

### 2.4 Equipment Telemetry Format

Farm equipment operation data for precision agriculture.

```json
{
  "dataExchange": {
    "standardVersion": "WIA-AGRI-020-v1.0",
    "dataId": "EQUIP-20251226-001",
    "timestamp": "2025-12-26T08:45:00.000Z",
    "farmId": "FARM-2025-001",
    "dataType": "EQUIPMENT",
    "equipment": {
      "equipmentId": "TRACTOR-001",
      "type": "TRACTOR",
      "manufacturer": "John Deere",
      "model": "8R 410",
      "serialNumber": "JD8R410-2024-001"
    },
    "location": {
      "latitude": 37.5670,
      "longitude": 126.9785,
      "heading": 87.5,
      "speed": 5.2,
      "speedUnit": "km/h"
    },
    "operation": {
      "operationType": "PLOWING",
      "fieldId": "FIELD-SOUTH-02",
      "startTime": "2025-12-26T07:00:00.000Z",
      "areaCovered": 3.2,
      "areaUnit": "hectares",
      "fuelConsumption": 18.5,
      "fuelUnit": "liters"
    },
    "telemetry": {
      "engineRPM": 1850,
      "engineTemp": 87,
      "hydraulicPressure": 2800,
      "workingWidth": 4.5,
      "workingDepth": 25,
      "depthUnit": "cm"
    },
    "diagnostics": {
      "engineHours": 1245,
      "maintenanceDue": false,
      "alerts": [],
      "fuelLevel": 68
    }
  }
}
```

---

## 3. Data Validation Rules

### 3.1 Required Fields
All data submissions MUST include:
- `standardVersion`
- `dataId` (unique)
- `timestamp` (ISO 8601 UTC)
- `farmId`
- `dataType`
- At least one measurement in `readings` or equivalent

### 3.2 Data Type Constraints

**Numeric Values:**
- Temperature: -50 to 60 °C
- Humidity: 0 to 100 %
- Soil Moisture: 0 to 100 %
- Soil pH: 0 to 14
- NDVI: -1 to 1

**Timestamp Rules:**
- Must be valid ISO 8601 format
- Must not be in the future (tolerance: 5 minutes)
- Historical data accepted up to 1 year old

**Location Validation:**
- Latitude: -90 to 90
- Longitude: -180 to 180
- Must be within declared farm boundaries (if available)

### 3.3 Quality Scoring

Data quality is automatically scored:
- **HIGH**: Calibrated sensors, < 1 hour old, validated
- **MEDIUM**: Uncalibrated sensors, < 24 hours old
- **LOW**: > 24 hours old, manual entry, unvalidated

---

## 4. Data Compression & Encoding

### 4.1 Supported Formats
- **JSON**: Default, human-readable
- **JSON-LD**: With semantic context
- **Protocol Buffers**: For high-volume streaming
- **MessagePack**: Binary JSON alternative

### 4.2 Compression
- GZIP compression recommended for HTTP transport
- Brotli compression for modern browsers
- No compression required for MQTT (protocol-level compression)

---

## 5. Schema Versioning

### 5.1 Version Format
`WIA-AGRI-020-vX.Y.Z`
- X: Major (breaking changes)
- Y: Minor (new fields, backward compatible)
- Z: Patch (bug fixes, clarifications)

### 5.2 Backward Compatibility
- Servers MUST accept data from previous minor versions
- Unknown fields MUST be ignored, not rejected
- Deprecated fields supported for 12 months after deprecation notice

---

## 6. Example: Complete Multi-Sensor Submission

```json
{
  "batch": {
    "batchId": "BATCH-20251226-001",
    "farmId": "FARM-2025-001",
    "submittedAt": "2025-12-26T10:30:00.000Z",
    "count": 3,
    "data": [
      {
        "dataId": "SOIL-001",
        "dataType": "SOIL_SENSOR",
        "readings": { "soilMoisture": { "value": 45.5 } }
      },
      {
        "dataId": "WEATHER-001",
        "dataType": "WEATHER",
        "current": { "temperature": 24.5, "humidity": 65 }
      },
      {
        "dataId": "CROP-001",
        "dataType": "CROP_HEALTH",
        "healthMetrics": { "ndvi": { "value": 0.78 } }
      }
    ]
  }
}
```

---

## 7. Error Handling

### 7.1 Validation Error Response

```json
{
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "Data validation failed",
    "details": [
      {
        "field": "readings.soilMoisture.value",
        "error": "Value 150 exceeds maximum allowed (100)",
        "constraint": "range"
      }
    ]
  }
}
```

---

## 8. Implementation Checklist

- [ ] Support all core data types (Soil, Weather, Crop, Equipment)
- [ ] Implement data validation rules
- [ ] Handle batch submissions
- [ ] Support JSON and at least one binary format
- [ ] Implement quality scoring
- [ ] Version compatibility checks
- [ ] Error handling and reporting
- [ ] Timestamp validation and timezone handling

---

**Next Phase:** [PHASE-2-API-INTERFACE.md](./PHASE-2-API-INTERFACE.md)

---

© 2025 WIA Standards · MIT License
弘益人間 · Benefit All Humanity
