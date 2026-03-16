# WIA-AGRI-021: Desert Agriculture
## Phase 1 - Data Format Specification

**Version:** 1.0
**Status:** Official Standard
**Last Updated:** 2025-12-26
**Standard ID:** WIA-AGRI-021

---

## 1. Overview

The WIA-AGRI-021 Desert Agriculture standard defines comprehensive data formats for managing agricultural operations in arid and semi-arid environments. This specification establishes standardized data structures for sensor readings, environmental monitoring, crop management, and irrigation control optimized for desert conditions.

### 1.1 Design Principles

- **Precision**: Data formats support high-precision measurements critical for water-scarce environments
- **Efficiency**: Optimized data structures minimize bandwidth and storage requirements
- **Extensibility**: Flexible schema allows for future sensor types and parameters
- **Interoperability**: Standard JSON format ensures compatibility with existing systems
- **Real-time**: Designed for low-latency data transmission and processing

### 1.2 Scope

This specification covers:
- Environmental sensor data formats
- Crop health and growth metrics
- Irrigation and water management data
- Weather and climate information
- Soil analysis parameters
- Energy consumption tracking
- Alert and notification formats

---

## 2. Core Data Types

### 2.1 Base Data Structure

All WIA-AGRI-021 data objects must conform to the following base structure:

```json
{
  "standard": "WIA-AGRI-021",
  "version": "1.0",
  "timestamp": "ISO-8601 datetime",
  "farmId": "unique farm identifier",
  "data": {
    /* Specific data payload */
  },
  "metadata": {
    /* Optional metadata */
  }
}
```

### 2.2 Timestamp Format

All timestamps MUST use ISO 8601 format with UTC timezone:

```
2025-12-26T10:30:45.123Z
```

### 2.3 Identifier Format

Farm, sensor, and device identifiers follow the pattern:

```
{type}_{location}_{sequence}

Examples:
- farm_sahara_001
- sensor_temp_a1_042
- zone_greenhouse_b3
```

---

## 3. Environmental Data Format

### 3.1 Temperature Readings

```json
{
  "standard": "WIA-AGRI-021",
  "version": "1.0",
  "timestamp": "2025-12-26T10:30:00Z",
  "farmId": "farm_sahara_001",
  "data": {
    "type": "temperature",
    "measurements": {
      "air": {
        "value": 38.5,
        "unit": "celsius",
        "location": "ambient",
        "height": 2.0
      },
      "soil": {
        "value": 42.3,
        "unit": "celsius",
        "depth": 0.15
      },
      "canopy": {
        "value": 35.2,
        "unit": "celsius"
      }
    },
    "sensorId": "sensor_temp_001",
    "accuracy": 0.1,
    "calibrationDate": "2025-11-15"
  }
}
```

### 3.2 Humidity Readings

```json
{
  "standard": "WIA-AGRI-021",
  "version": "1.0",
  "timestamp": "2025-12-26T10:30:00Z",
  "farmId": "farm_sahara_001",
  "data": {
    "type": "humidity",
    "measurements": {
      "relative": {
        "value": 15.2,
        "unit": "percent"
      },
      "absolute": {
        "value": 12.8,
        "unit": "g/m3"
      },
      "dewPoint": {
        "value": 8.5,
        "unit": "celsius"
      }
    },
    "sensorId": "sensor_hum_001",
    "location": "greenhouse_a1"
  }
}
```

### 3.3 Soil Moisture

```json
{
  "standard": "WIA-AGRI-021",
  "version": "1.0",
  "timestamp": "2025-12-26T10:30:00Z",
  "farmId": "farm_sahara_001",
  "data": {
    "type": "soil_moisture",
    "measurements": [
      {
        "depth": 0.10,
        "value": 12.8,
        "unit": "percent",
        "volumetric": true
      },
      {
        "depth": 0.20,
        "value": 15.3,
        "unit": "percent",
        "volumetric": true
      },
      {
        "depth": 0.30,
        "value": 18.1,
        "unit": "percent",
        "volumetric": true
      }
    ],
    "soilType": "sandy_loam",
    "sensorId": "sensor_sm_zone_a1_001",
    "zoneId": "zone_a1"
  }
}
```

### 3.4 Solar Radiation

```json
{
  "standard": "WIA-AGRI-021",
  "version": "1.0",
  "timestamp": "2025-12-26T10:30:00Z",
  "farmId": "farm_sahara_001",
  "data": {
    "type": "solar_radiation",
    "measurements": {
      "global": {
        "value": 950,
        "unit": "W/m2"
      },
      "direct": {
        "value": 820,
        "unit": "W/m2"
      },
      "diffuse": {
        "value": 130,
        "unit": "W/m2"
      },
      "par": {
        "value": 1850,
        "unit": "µmol/m2/s",
        "description": "Photosynthetically Active Radiation"
      }
    },
    "sensorId": "sensor_solar_001",
    "solarAngle": 67.5
  }
}
```

### 3.5 Wind Measurements

```json
{
  "standard": "WIA-AGRI-021",
  "version": "1.0",
  "timestamp": "2025-12-26T10:30:00Z",
  "farmId": "farm_sahara_001",
  "data": {
    "type": "wind",
    "measurements": {
      "speed": {
        "value": 18.5,
        "unit": "km/h",
        "avgPeriod": 600
      },
      "gust": {
        "value": 32.1,
        "unit": "km/h"
      },
      "direction": {
        "value": 285,
        "unit": "degrees",
        "cardinal": "WNW"
      }
    },
    "sensorId": "sensor_wind_001",
    "height": 10.0
  }
}
```

---

## 4. Crop Data Format

### 4.1 Crop Health Monitoring

```json
{
  "standard": "WIA-AGRI-021",
  "version": "1.0",
  "timestamp": "2025-12-26T10:30:00Z",
  "farmId": "farm_sahara_001",
  "data": {
    "type": "crop_health",
    "cropId": "crop_tomato_a1_2025",
    "cropType": "tomato",
    "variety": "desert_king",
    "plantingDate": "2024-11-01",
    "currentStage": "fruiting",
    "health": {
      "overall": 87,
      "scale": "0-100",
      "indicators": {
        "leafColor": "healthy_green",
        "growth_rate": "optimal",
        "pest_presence": false,
        "disease_signs": false,
        "stress_level": "low"
      }
    },
    "metrics": {
      "height": {
        "average": 1.45,
        "unit": "meters"
      },
      "leafArea": {
        "index": 3.2,
        "description": "LAI"
      },
      "chlorophyll": {
        "content": 45.8,
        "unit": "SPAD"
      }
    },
    "zoneId": "zone_a1"
  }
}
```

### 4.2 Growth Stage Tracking

```json
{
  "standard": "WIA-AGRI-021",
  "version": "1.0",
  "timestamp": "2025-12-26T10:30:00Z",
  "farmId": "farm_sahara_001",
  "data": {
    "type": "growth_stage",
    "cropId": "crop_tomato_a1_2025",
    "stages": {
      "current": "fruiting",
      "progress": 65,
      "expectedCompletion": "2025-01-15",
      "nextStage": "ripening"
    },
    "timeline": [
      {
        "stage": "germination",
        "startDate": "2024-11-01",
        "endDate": "2024-11-08",
        "duration": 7
      },
      {
        "stage": "seedling",
        "startDate": "2024-11-08",
        "endDate": "2024-11-22",
        "duration": 14
      },
      {
        "stage": "vegetative",
        "startDate": "2024-11-22",
        "endDate": "2024-12-13",
        "duration": 21
      },
      {
        "stage": "flowering",
        "startDate": "2024-12-13",
        "endDate": "2024-12-27",
        "duration": 14
      },
      {
        "stage": "fruiting",
        "startDate": "2024-12-27",
        "endDate": "2025-01-15",
        "duration": 19,
        "status": "in_progress"
      }
    ]
  }
}
```

---

## 5. Irrigation Data Format

### 5.1 Irrigation Event

```json
{
  "standard": "WIA-AGRI-021",
  "version": "1.0",
  "timestamp": "2025-12-26T10:30:00Z",
  "farmId": "farm_sahara_001",
  "data": {
    "type": "irrigation_event",
    "eventId": "irr_event_12345",
    "zoneId": "zone_a1",
    "control": {
      "mode": "automatic",
      "trigger": "soil_moisture_threshold",
      "startTime": "2025-12-26T10:00:00Z",
      "endTime": "2025-12-26T10:30:00Z",
      "duration": 1800
    },
    "water": {
      "volume": {
        "value": 450,
        "unit": "liters"
      },
      "flowRate": {
        "value": 15,
        "unit": "L/min"
      },
      "pressure": {
        "value": 2.5,
        "unit": "bar"
      }
    },
    "efficiency": {
      "application": 92,
      "distribution": 88,
      "overall": 81
    },
    "energy": {
      "consumed": 2.5,
      "unit": "kWh",
      "source": "solar"
    }
  }
}
```

### 5.2 Water Quality

```json
{
  "standard": "WIA-AGRI-021",
  "version": "1.0",
  "timestamp": "2025-12-26T10:30:00Z",
  "farmId": "farm_sahara_001",
  "data": {
    "type": "water_quality",
    "sourceId": "well_001",
    "parameters": {
      "ph": {
        "value": 7.2,
        "acceptable_range": [6.5, 8.5]
      },
      "ec": {
        "value": 1.8,
        "unit": "dS/m",
        "description": "Electrical Conductivity"
      },
      "tds": {
        "value": 1150,
        "unit": "ppm",
        "description": "Total Dissolved Solids"
      },
      "salinity": {
        "value": 0.9,
        "unit": "g/L"
      },
      "temperature": {
        "value": 24.5,
        "unit": "celsius"
      }
    },
    "quality_grade": "good",
    "suitable_for": ["drip_irrigation", "greenhouse"],
    "treatment_required": false
  }
}
```

---

## 6. Weather Data Format

### 6.1 Current Weather

```json
{
  "standard": "WIA-AGRI-021",
  "version": "1.0",
  "timestamp": "2025-12-26T10:30:00Z",
  "farmId": "farm_sahara_001",
  "data": {
    "type": "current_weather",
    "conditions": {
      "description": "clear_sky",
      "icon": "01d"
    },
    "temperature": {
      "current": 38.5,
      "feels_like": 42.3,
      "unit": "celsius"
    },
    "humidity": 15.2,
    "pressure": {
      "value": 1013,
      "unit": "hPa"
    },
    "wind": {
      "speed": 18.5,
      "direction": 285,
      "unit": "km/h"
    },
    "clouds": 5,
    "visibility": 10000,
    "uv_index": 11,
    "sunrise": "2025-12-26T07:30:00Z",
    "sunset": "2025-12-26T18:45:00Z"
  }
}
```

### 6.2 Weather Forecast

```json
{
  "standard": "WIA-AGRI-021",
  "version": "1.0",
  "timestamp": "2025-12-26T10:30:00Z",
  "farmId": "farm_sahara_001",
  "data": {
    "type": "weather_forecast",
    "forecast_period": "7_day",
    "forecasts": [
      {
        "date": "2025-12-27",
        "temperature": {
          "min": 18.0,
          "max": 39.5,
          "unit": "celsius"
        },
        "humidity": {
          "min": 10,
          "max": 25
        },
        "precipitation": {
          "probability": 5,
          "amount": 0,
          "unit": "mm"
        },
        "wind": {
          "speed": 22.0,
          "direction": 290,
          "unit": "km/h"
        },
        "conditions": "clear",
        "uv_index": 11
      }
    ]
  }
}
```

---

## 7. Alert and Notification Format

### 7.1 System Alert

```json
{
  "standard": "WIA-AGRI-021",
  "version": "1.0",
  "timestamp": "2025-12-26T10:30:00Z",
  "farmId": "farm_sahara_001",
  "data": {
    "type": "alert",
    "alertId": "alert_12345",
    "severity": "high",
    "category": "irrigation",
    "title": "Low Soil Moisture Detected",
    "message": "Soil moisture in Zone A1 has dropped below critical threshold",
    "details": {
      "zoneId": "zone_a1",
      "current_value": 8.5,
      "threshold": 12.0,
      "unit": "percent",
      "duration": 3600
    },
    "recommended_action": "Start irrigation immediately",
    "auto_action": {
      "enabled": true,
      "action": "start_irrigation",
      "parameters": {
        "duration": 1800,
        "flowRate": 15
      }
    },
    "acknowledged": false,
    "resolved": false
  }
}
```

---

## 8. Energy Data Format

### 8.1 Energy Consumption

```json
{
  "standard": "WIA-AGRI-021",
  "version": "1.0",
  "timestamp": "2025-12-26T10:30:00Z",
  "farmId": "farm_sahara_001",
  "data": {
    "type": "energy_consumption",
    "period": {
      "start": "2025-12-26T00:00:00Z",
      "end": "2025-12-26T23:59:59Z",
      "interval": "1_day"
    },
    "consumption": {
      "total": 145.8,
      "unit": "kWh",
      "breakdown": {
        "irrigation": 65.2,
        "climate_control": 48.5,
        "lighting": 22.1,
        "monitoring": 10.0
      }
    },
    "generation": {
      "solar": 180.5,
      "unit": "kWh"
    },
    "grid": {
      "imported": 0,
      "exported": 34.7,
      "unit": "kWh"
    },
    "efficiency": 87.5
  }
}
```

---

## 9. Data Validation Rules

### 9.1 Required Fields

All data objects MUST include:
- `standard`: "WIA-AGRI-021"
- `version`: Semantic version string
- `timestamp`: ISO-8601 formatted datetime
- `farmId`: Valid farm identifier
- `data`: Object containing measurement data

### 9.2 Value Ranges

| Parameter | Min | Max | Unit |
|-----------|-----|-----|------|
| Air Temperature | -20 | 60 | °C |
| Soil Temperature | 0 | 70 | °C |
| Humidity | 0 | 100 | % |
| Soil Moisture | 0 | 100 | % |
| Wind Speed | 0 | 200 | km/h |
| Solar Radiation | 0 | 1400 | W/m² |
| pH | 0 | 14 | - |
| EC | 0 | 10 | dS/m |

### 9.3 Precision Requirements

- Temperature: ±0.1°C
- Humidity: ±0.1%
- Soil Moisture: ±0.1%
- Flow Rate: ±1%
- Energy: ±0.01 kWh

---

## 10. Data Retention Policy

### 10.1 Storage Duration

| Data Type | Retention Period | Aggregation |
|-----------|-----------------|-------------|
| Raw Sensor Data | 30 days | None |
| Hourly Averages | 1 year | Mean, Min, Max |
| Daily Summaries | 5 years | Mean, Min, Max, Sum |
| Monthly Reports | 10 years | Aggregated |
| Alerts & Events | Permanent | None |

---

## 11. Compliance and Standards

This specification complies with:
- ISO 8601 (Date and time format)
- RFC 8259 (JSON data interchange format)
- IEEE 754 (Floating-point arithmetic)
- WGS 84 (Geographic coordinate system)

---

**弘益人間 (홍익인간) · Benefit All Humanity**

© 2025 SmileStory Inc. / WIA · All Rights Reserved
