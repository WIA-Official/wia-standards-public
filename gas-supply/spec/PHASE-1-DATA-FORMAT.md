# WIA-SOC-011: Gas Supply Standard
## PHASE 1: Data Format Specification

**Version:** 1.0  
**Status:** Complete  
**Last Updated:** 2025-12-26

---

## 1. Overview

Phase 1 of the WIA-SOC-011 standard defines comprehensive data formats for representing all aspects of gas supply infrastructure and operations. These formats use JSON Schema to ensure human readability and machine parseability, enabling automated validation and consistent data exchange.

### 1.1 Design Principles

- **Extensibility**: Schemas support extension without breaking compatibility
- **Validation**: Built-in validation rules ensure data quality
- **Interoperability**: Standard formats enable cross-system data exchange
- **Versioning**: Schema versions allow controlled evolution

---

## 2. Core Data Schemas

### 2.1 Pipeline Schema

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "type": "object",
  "properties": {
    "pipelineId": {
      "type": "string",
      "pattern": "^PL-[A-Z]{2}-[0-9]{3}-[0-9]{4}$",
      "description": "Unique pipeline identifier"
    },
    "pipelineType": {
      "type": "string",
      "enum": ["transmission", "distribution", "gathering", "service"],
      "description": "Pipeline classification"
    },
    "geometry": {
      "$ref": "https://geojson.org/schema/LineString.json",
      "description": "Pipeline route in GeoJSON format"
    },
    "material": {
      "type": "object",
      "properties": {
        "type": {"type": "string", "enum": ["steel", "plastic", "cast_iron", "composite"]},
        "grade": {"type": "string"},
        "diameter_mm": {"type": "number", "minimum": 25, "maximum": 2000},
        "wallThickness_mm": {"type": "number", "minimum": 1, "maximum": 100}
      },
      "required": ["type", "diameter_mm"]
    },
    "pressureRating": {
      "type": "object",
      "properties": {
        "maop_bar": {"type": "number", "description": "Maximum Allowable Operating Pressure"},
        "designPressure_bar": {"type": "number"}
      },
      "required": ["maop_bar"]
    },
    "installation": {
      "type": "object",
      "properties": {
        "date": {"type": "string", "format": "date"},
        "installer": {"type": "string"},
        "manufacturer": {"type": "string"}
      }
    },
    "operationalStatus": {
      "type": "string",
      "enum": ["active", "inactive", "abandoned", "planned", "under_construction"]
    }
  },
  "required": ["pipelineId", "pipelineType", "geometry", "material", "pressureRating"]
}
```

### 2.2 Operational Measurement Schema

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "type": "object",
  "properties": {
    "measurementId": {"type": "string", "format": "uuid"},
    "timestamp": {"type": "string", "format": "date-time"},
    "location": {
      "oneOf": [
        {"$ref": "#/definitions/pipelineLocation"},
        {"$ref": "#/definitions/gpsLocation"}
      ]
    },
    "measurementType": {
      "type": "string",
      "enum": ["pressure", "flow", "temperature", "composition", "density"]
    },
    "value": {"type": "number"},
    "unit": {"type": "string"},
    "quality": {
      "type": "string",
      "enum": ["good", "uncertain", "bad"],
      "default": "good"
    },
    "sensorId": {"type": "string"}
  },
  "required": ["timestamp", "measurementType", "value", "unit"],
  "definitions": {
    "pipelineLocation": {
      "type": "object",
      "properties": {
        "pipelineId": {"type": "string"},
        "kilometer": {"type": "number", "minimum": 0}
      },
      "required": ["pipelineId", "kilometer"]
    },
    "gpsLocation": {
      "type": "object",
      "properties": {
        "latitude": {"type": "number", "minimum": -90, "maximum": 90},
        "longitude": {"type": "number", "minimum": -180, "maximum": 180}
      },
      "required": ["latitude", "longitude"]
    }
  }
}
```

---

## 3. Gas Quality Data

### 3.1 Composition Schema

Gas composition data follows molecular-level specification:

```json
{
  "compositionId": "COMP-2025-001",
  "timestamp": "2025-12-26T14:30:00Z",
  "location": {"pipelineId": "PL-KR-001-2025", "kilometer": 45.3},
  "components": [
    {"molecule": "CH4", "moleFraction": 0.958},
    {"molecule": "C2H6", "moleFraction": 0.025},
    {"molecule": "C3H8", "moleFraction": 0.008},
    {"molecule": "CO2", "moleFraction": 0.007},
    {"molecule": "N2", "moleFraction": 0.002}
  ],
  "calculatedProperties": {
    "heatingValue_MJ_m3": 38.5,
    "wobbeIndex": 51.2,
    "density_kg_m3": 0.72,
    "compressibility": 0.998
  },
  "contaminants": {
    "H2S_ppm": 0.5,
    "moisture_mg_m3": 32,
    "totalSulfur_mg_m3": 5.2
  }
}
```

---

## 4. Equipment Data Structures

### 4.1 Compressor Station

```json
{
  "stationId": "CS-KR-SEOUL-01",
  "location": {
    "latitude": 37.5665,
    "longitude": 126.9780,
    "address": "123 Pipeline Road, Seoul"
  },
  "compressors": [
    {
      "compressorId": "COMP-001",
      "type": "centrifugal",
      "driver": "gas_turbine",
      "capacity": {
        "flow_m3_h": 50000,
        "pressureRatio": 1.4,
        "powerRating_MW": 12.5
      },
      "efficiency": {
        "designPoint": 0.85,
        "currentOperating": 0.82
      },
      "status": "running",
      "runningHours": 45230
    }
  ],
  "controlSystem": {
    "type": "SCADA",
    "manufacturer": "Siemens",
    "softwareVersion": "PCS7-v9.1"
  }
}
```

### 4.2 Meter Station

```json
{
  "stationId": "MS-KR-BUSAN-05",
  "meteringTechnology": "ultrasonic",
  "flowMeters": [
    {
      "meterId": "FM-001",
      "manufacturer": "Daniel",
      "model": "SeniorSonic 3414",
      "diameter_mm": 300,
      "paths": 4,
      "accuracyClass": 0.5,
      "calibration": {
        "lastDate": "2024-11-15",
        "nextDue": "2025-11-15",
        "certificateNumber": "CAL-2024-1234"
      },
      "measurement": {
        "flow_m3_h": 8540,
        "totalVolume_m3": 125634789,
        "pressure_bar": 55.3,
        "temperature_C": 18.5
      }
    }
  ]
}
```

---

## 5. Event and Alarm Data

### 5.1 Event Schema

```json
{
  "eventId": "EVT-2025-12-26-00123",
  "timestamp": "2025-12-26T14:35:22Z",
  "eventType": "alarm",
  "severity": "high",
  "category": "safety",
  "description": "High pressure detected on pipeline PL-KR-001-2025",
  "affectedAssets": [
    {"type": "pipeline", "id": "PL-KR-001-2025"},
    {"type": "compressor", "id": "CS-KR-SEOUL-01"}
  ],
  "measurements": {
    "pressure_bar": 85.2,
    "threshold_bar": 80.0,
    "deviation_percent": 6.5
  },
  "status": "active",
  "acknowledgedBy": null,
  "resolvedTimestamp": null,
  "actionsTaken": []
}
```

---

## 6. Hydrogen Blending Data

### 6.1 Blended Gas Schema

```json
{
  "blendId": "BLEND-2025-001",
  "timestamp": "2025-12-26T15:00:00Z",
  "location": {"pipelineId": "PL-KR-001-2025", "kilometer": 12.5},
  "composition": {
    "H2_moleFraction": 0.20,
    "CH4_moleFraction": 0.75,
    "other_moleFraction": 0.05
  },
  "blendingSource": {
    "H2_source": "green_hydrogen_electrolysis",
    "H2_injectionRate_m3_h": 2000,
    "NG_flowRate_m3_h": 8000
  },
  "calculatedProperties": {
    "heatingValue_MJ_m3": 33.2,
    "wobbeIndex": 46.8,
    "H2_volumePercent": 20.0
  },
  "compatibility": {
    "pipelineMaterial": "compatible",
    "compressorSeals": "requires_monitoring",
    "endUseEquipment": "within_tolerance"
  }
}
```

---

## 7. Data Validation Rules

### 7.1 Validation Layers

1. **Schema Validation**: JSON Schema validation ensures structural correctness
2. **Range Validation**: Values must fall within physical limits
3. **Relationship Validation**: Foreign key references must exist
4. **Temporal Validation**: Timestamps must be logical and sequential
5. **Business Rule Validation**: Domain-specific constraints

### 7.2 Example Validation Rules

- Pressure cannot exceed pipeline MAOP
- Flow rate must be non-negative
- Temperature must be within -50°C to +50°C for standard operations
- Gas composition components must sum to 1.0 (±0.001 tolerance)
- Meter calibration must be current (within certification period)

---

## 8. Spatial Data Formats

### 8.1 GeoJSON Integration

All geographic data uses GeoJSON format with WGS84 coordinate system (EPSG:4326):

```json
{
  "type": "Feature",
  "geometry": {
    "type": "LineString",
    "coordinates": [
      [126.9780, 37.5665],
      [126.9850, 37.5720],
      [127.0000, 37.5800]
    ]
  },
  "properties": {
    "pipelineId": "PL-KR-001-2025",
    "length_km": 5.8,
    "material": "steel",
    "diameter_mm": 914
  }
}
```

---

## 9. Time Series Data Optimization

### 9.1 Efficient Encoding

For high-frequency measurements, use compact encoding:

```json
{
  "seriesId": "TS-PRESSURE-001",
  "startTime": "2025-12-26T00:00:00Z",
  "interval_seconds": 10,
  "unit": "bar",
  "values": [50.1, 50.2, 50.15, 50.18, 50.22, 50.19],
  "quality": ["good", "good", "good", "good", "good", "good"]
}
```

---

## 10. Implementation Guidelines

### 10.1 Adoption Strategy

1. **New Projects**: Implement full schema compliance from start
2. **Existing Systems**: Phased migration with translation layers
3. **Data Exchange**: Use schemas for all inter-system communication
4. **Storage**: Native storage can use optimized formats with schema transformation

### 10.2 Tooling Support

- **Schema Validators**: Use standard JSON Schema validators
- **Code Generation**: Generate types/classes from schemas
- **Documentation**: Auto-generate API docs from schemas
- **Testing**: Schema-based test data generation

---

## 11. Appendix: Complete Schema Repository

All schemas available at:
- **GitHub**: https://github.com/WIA-Official/wia-standards/tree/main/gas-supply/spec/schemas
- **JSON Schema Store**: https://schemas.wiastandards.com/soc-011/v1/

---

**Document Control**
- Version: 1.0
- Status: Complete
- Next Review: 2026-12-26
- Contact: standards@wiastandards.com

© 2025 World Certification Industry Association | MIT License
