# Chapter 4: Data Formats and Schemas

## Phase 1: Standardized Data Structures for Environmental Sensors

---

## 4.1 Core Data Model Structure

This chapter details the Phase 1 data format specification, providing complete JSON schemas, examples, and validation rules for all sensor types.

### Complete Core Message Structure

```json
{
  "version": "1.0.0",
  "standard": "WIA-ENE-027",
  "deviceId": "ENV-AIR-001",
  "timestamp": "2025-01-09T10:30:00.000Z",
  "sensorType": "air_quality",
  "readings": {
    // Sensor-specific data
  },
  "location": {
    "latitude": 37.5665,
    "longitude": 126.9780,
    "altitude": 38.5,
    "accuracy": 10.0,
    "datum": "WGS84"
  },
  "metadata": {
    "manufacturer": "AirSense Corp",
    "model": "AS-3000",
    "firmware": "v2.4.1",
    "battery": 85,
    "signalStrength": -65,
    "uptime": 86400
  },
  "quality": {
    "overall": "good",
    "flags": [],
    "confidence": 0.95
  },
  "calibration": {
    "lastCalibration": "2024-11-15T09:00:00Z",
    "nextCalibration": "2025-05-15T09:00:00Z",
    "method": "reference_colocated",
    "parameters": {"slope": 1.02, "intercept": -0.5},
    "certificateId": "CAL-2024-1234"
  }
}
```

---

## 4.2 Air Quality Sensor Data Format

### Particulate Matter

Complete PM sensor reading structure:

```json
{
  "version": "1.0.0",
  "standard": "WIA-ENE-027",
  "deviceId": "ENV-AIR-001",
  "timestamp": "2025-01-09T10:30:00.000Z",
  "sensorType": "air_quality",
  "readings": {
    "pm1_0": {
      "value": 8.5,
      "unit": "μg/m³",
      "method": "laser_scattering",
      "uncertainty": 2.0,
      "averaged": 300
    },
    "pm2_5": {
      "value": 15.3,
      "unit": "μg/m³",
      "method": "laser_scattering",
      "uncertainty": 3.0,
      "averaged": 300
    },
    "pm10": {
      "value": 22.8,
      "unit": "μg/m³",
      "method": "laser_scattering",
      "uncertainty": 4.5,
      "averaged": 300
    }
  },
  "location": {
    "latitude": 37.5665,
    "longitude": 126.9780,
    "altitude": 38.5
  },
  "quality": {
    "overall": "good",
    "flags": []
  }
}
```

**Valid Ranges:**
- PM1.0: 0-500 μg/m³
- PM2.5: 0-500 μg/m³
- PM10: 0-1000 μg/m³

### Gaseous Pollutants

```json
{
  "version": "1.0.0",
  "standard": "WIA-ENE-027",
  "deviceId": "ENV-AIR-002",
  "timestamp": "2025-01-09T10:30:00.000Z",
  "sensorType": "air_quality",
  "readings": {
    "co2": {
      "value": 420,
      "unit": "ppm",
      "method": "NDIR",
      "uncertainty": 30
    },
    "co": {
      "value": 0.5,
      "unit": "ppm",
      "method": "electrochemical",
      "uncertainty": 0.1
    },
    "no2": {
      "value": 25,
      "unit": "ppb",
      "method": "electrochemical",
      "uncertainty": 5
    },
    "o3": {
      "value": 45,
      "unit": "ppb",
      "method": "electrochemical",
      "uncertainty": 10
    },
    "voc": {
      "value": 150,
      "unit": "ppb",
      "method": "metal_oxide",
      "uncertainty": 30
    }
  }
}
```

### Air Quality Index

```json
{
  "version": "1.0.0",
  "standard": "WIA-ENE-027",
  "deviceId": "ENV-AIR-003",
  "timestamp": "2025-01-09T10:30:00.000Z",
  "sensorType": "air_quality",
  "readings": {
    "pm2_5": {
      "value": 35.5,
      "unit": "μg/m³"
    },
    "aqi": {
      "value": 102,
      "category": "unhealthy_sensitive",
      "pollutant": "pm2_5",
      "standard": "US_EPA"
    }
  }
}
```

**AQI Categories:**
- `"good"`: 0-50
- `"moderate"`: 51-100
- `"unhealthy_sensitive"`: 101-150
- `"unhealthy"`: 151-200
- `"very_unhealthy"`: 201-300
- `"hazardous"`: 301-500

---

## 4.3 Water Quality Sensor Data Format

### Physical and Chemical Parameters

```json
{
  "version": "1.0.0",
  "standard": "WIA-ENE-027",
  "deviceId": "WATER-SITE-001",
  "timestamp": "2025-01-09T10:30:00.000Z",
  "sensorType": "water_quality",
  "readings": {
    "ph": {
      "value": 7.2,
      "unit": "pH",
      "method": "glass_electrode",
      "temperature_compensated": true,
      "uncertainty": 0.1
    },
    "dissolved_oxygen": {
      "value": 8.5,
      "unit": "mg/L",
      "method": "optical",
      "saturation": 95,
      "uncertainty": 0.2
    },
    "turbidity": {
      "value": 2.5,
      "unit": "NTU",
      "method": "nephelometric",
      "uncertainty": 0.3
    },
    "conductivity": {
      "value": 450,
      "unit": "μS/cm",
      "temperature": 20.5,
      "specific_conductance": 460,
      "uncertainty": 10
    },
    "temperature": {
      "value": 20.5,
      "unit": "°C",
      "uncertainty": 0.2
    },
    "orp": {
      "value": 250,
      "unit": "mV",
      "uncertainty": 10
    }
  },
  "location": {
    "latitude": 37.5700,
    "longitude": 126.9800,
    "altitude": 25.0
  }
}
```

**Valid Ranges:**
- pH: 0-14
- Dissolved Oxygen: 0-20 mg/L
- Turbidity: 0-1000 NTU
- Conductivity: 0-10,000 μS/cm
- Temperature: -2 to 40°C
- ORP: -500 to +500 mV

---

## 4.4 Soil Sensor Data Format

### Moisture, Temperature, and Nutrients

```json
{
  "version": "1.0.0",
  "standard": "WIA-ENE-027",
  "deviceId": "SOIL-FIELD-A-001",
  "timestamp": "2025-01-09T10:30:00.000Z",
  "sensorType": "soil",
  "readings": {
    "moisture": {
      "value": 28.5,
      "unit": "%VWC",
      "method": "capacitance",
      "depth": 0.15,
      "uncertainty": 3.0
    },
    "temperature": {
      "value": 18.5,
      "unit": "°C",
      "depth": 0.10,
      "uncertainty": 0.5
    },
    "electrical_conductivity": {
      "value": 520,
      "unit": "μS/cm",
      "depth": 0.15,
      "uncertainty": 30
    },
    "nutrients": {
      "nitrogen": {
        "value": 45,
        "unit": "ppm",
        "form": "nitrate_NO3",
        "method": "ion_selective",
        "uncertainty": 10
      },
      "phosphorus": {
        "value": 12,
        "unit": "ppm",
        "form": "available_P",
        "uncertainty": 3
      },
      "potassium": {
        "value": 85,
        "unit": "ppm",
        "form": "exchangeable_K",
        "uncertainty": 15
      }
    }
  },
  "location": {
    "latitude": 37.5800,
    "longitude": 126.9900,
    "altitude": 45.0
  }
}
```

**Valid Ranges:**
- Moisture: 0-60 %VWC
- Temperature: -10 to 60°C
- EC: 0-4000 μS/cm
- Nitrogen: 0-100 ppm
- Phosphorus: 0-50 ppm
- Potassium: 0-200 ppm

---

## 4.5 Meteorological Sensor Data Format

### Weather Parameters

```json
{
  "version": "1.0.0",
  "standard": "WIA-ENE-027",
  "deviceId": "WEATHER-STN-001",
  "timestamp": "2025-01-09T10:30:00.000Z",
  "sensorType": "meteorological",
  "readings": {
    "temperature": {
      "value": 15.5,
      "unit": "°C",
      "uncertainty": 0.3
    },
    "humidity": {
      "value": 65,
      "unit": "%RH",
      "uncertainty": 3
    },
    "pressure": {
      "value": 1013.2,
      "unit": "hPa",
      "type": "station",
      "uncertainty": 0.5
    },
    "wind_speed": {
      "value": 3.5,
      "unit": "m/s",
      "gust": 5.2,
      "uncertainty": 0.3
    },
    "wind_direction": {
      "value": 225,
      "unit": "degrees",
      "uncertainty": 10
    },
    "precipitation": {
      "value": 0.2,
      "unit": "mm",
      "type": "cumulative",
      "period": 3600
    },
    "solar_radiation": {
      "value": 450,
      "unit": "W/m²",
      "type": "global"
    },
    "uv_index": {
      "value": 3,
      "unit": "index"
    }
  },
  "location": {
    "latitude": 37.5665,
    "longitude": 126.9780,
    "altitude": 38.5
  }
}
```

---

## 4.6 Metadata and Quality Flags

### Quality Flag Definitions

```json
{
  "quality": {
    "overall": "suspect",
    "flags": [
      "calibration_due",
      "low_battery"
    ],
    "confidence": 0.75
  }
}
```

**Standard Quality Flags:**

| Flag | Description | Severity |
|------|-------------|----------|
| `calibration_due` | Calibration approaching due date | Warning |
| `calibration_overdue` | Calibration past due date | Critical |
| `out_of_range` | Value outside plausible range | Critical |
| `rapid_change` | Unrealistic rate of change | Warning |
| `low_battery` | Battery < 20% | Warning |
| `critical_battery` | Battery < 10% | Critical |
| `poor_signal` | Weak network signal | Warning |
| `sensor_fault` | Self-diagnostic failure | Critical |
| `maintenance_required` | Service interval exceeded | Warning |
| `estimated` | Interpolated/estimated value | Info |

---

## 4.7 Calibration Data Structures

### Calibration Metadata

```json
{
  "calibration": {
    "lastCalibration": "2024-11-15T09:00:00Z",
    "nextCalibration": "2025-05-15T09:00:00Z",
    "method": "reference_colocated",
    "parameters": {
      "slope": 1.02,
      "intercept": -0.5,
      "r_squared": 0.95
    },
    "certificateId": "CAL-2024-1234",
    "calibratedBy": "TechCal Services",
    "referenceDevice": "MetOne BAM-1020"
  }
}
```

**Calibration Methods:**
- `"factory"`: Factory calibration only
- `"zero_span"`: Zero and span gas calibration
- `"reference_colocated"`: Co-located with reference instrument
- `"laboratory"`: Laboratory calibration
- `"field"`: Field calibration procedure

---

## 4.8 JSON Schema Validation

### Complete JSON Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "WIA-ENE-027 Environmental Sensor Data",
  "type": "object",
  "required": ["version", "standard", "deviceId", "timestamp", "sensorType", "readings"],
  "properties": {
    "version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+\\.\\d+$"
    },
    "standard": {
      "type": "string",
      "const": "WIA-ENE-027"
    },
    "deviceId": {
      "type": "string",
      "minLength": 1
    },
    "timestamp": {
      "type": "string",
      "format": "date-time"
    },
    "sensorType": {
      "type": "string",
      "enum": ["air_quality", "water_quality", "soil", "meteorological", "radiation", "noise"]
    },
    "readings": {
      "type": "object"
    },
    "location": {
      "type": "object",
      "properties": {
        "latitude": {"type": "number", "minimum": -90, "maximum": 90},
        "longitude": {"type": "number", "minimum": -180, "maximum": 180},
        "altitude": {"type": "number"},
        "accuracy": {"type": "number", "minimum": 0},
        "datum": {"type": "string", "default": "WGS84"}
      },
      "required": ["latitude", "longitude"]
    },
    "quality": {
      "type": "object",
      "properties": {
        "overall": {
          "type": "string",
          "enum": ["good", "suspect", "bad", "missing"]
        },
        "flags": {
          "type": "array",
          "items": {"type": "string"}
        },
        "confidence": {
          "type": "number",
          "minimum": 0.0,
          "maximum": 1.0
        }
      }
    }
  }
}
```

---

## 4.9 Review Questions and Key Takeaways

### Review Questions

1. Create a complete WIA-ENE-027 message for an air quality sensor measuring PM2.5=25 μg/m³, PM10=40 μg/m³, temperature=22°C, with calibration due in 30 days.

2. What quality flags should be set for a water quality sensor with pH=3.5, last calibrated 400 days ago, battery at 15%?

3. Design a soil moisture sensor message with readings at three depths (10cm, 20cm, 30cm). How would you structure the data?

4. Validate this message: `{"deviceId": "TEST", "timestamp": "2025-01-09", "sensorType": "air"}`. What's missing?

5. Calculate the AQI for PM2.5=55 μg/m³ using US EPA method. Create the complete readings object.

### Key Takeaways

1. **Core Structure**: All messages require version, standard, deviceId, timestamp, sensorType, and readings fields for basic compliance.

2. **Sensor-Specific**: Each sensor type (air/water/soil/meteorological) has defined structures for the readings object with appropriate parameters.

3. **Quality Metadata**: Quality flags, calibration data, and overall quality indicators enable automated data assessment.

4. **JSON Schema**: Validation schemas enable automated checking of message compliance and data quality.

5. **Measurement Structure**: Individual measurements include value, unit, method, and uncertainty for complete documentation.

6. **Location Data**: Latitude, longitude, and altitude enable spatial analysis and mapping of environmental data.

7. **Calibration**: Calibration metadata includes dates, methods, parameters, and certificate IDs for traceability.

8. **Extensibility**: The format supports new sensor types and parameters through extension without breaking existing implementations.

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - Benefit All Humanity

**Next Chapter: [Chapter 5: API Interfaces](05-api-interface.md)**
