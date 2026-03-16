# WIA-ENE-017: Air Quality Monitoring Standard
## PHASE 1: DATA FORMAT

**Document Version:** 1.0  
**Status:** Active  
**Last Updated:** December 25, 2025  
**Philosophy:** 弘益人間 (Hongik Ingan) · Benefit All Humanity

---

## 1. Introduction

Phase 1 of the WIA-ENE-017 standard defines the foundational data structures, JSON schemas, measurement units, and metadata standards for air quality monitoring. This specification ensures that data from diverse sensors and monitoring stations can be exchanged, processed, and analyzed consistently across global networks.

### 1.1 Scope

This document covers:
- Standard data models for pollutant measurements
- JSON schema definitions and validation rules
- Unit standardization and conversion factors
- Metadata requirements for stations and sensors
- Quality control flags and uncertainty reporting
- Timestamp and timezone handling

### 1.2 Normative References

- ISO 8601:2019 Date and time format
- JSON Schema Draft 07
- US EPA Air Quality System (AQS) Data Dictionary
- WHO Air Quality Guidelines (2021)

---

## 2. Core Data Model

### 2.1 Pollutant Measurements

All WIA-ENE-017 compliant systems must support these six primary pollutants:

| Pollutant | Field Name | Units | Range | Resolution | Averaging Period |
|-----------|------------|-------|-------|------------|------------------|
| PM2.5 | `pm25` | μg/m³ | 0-1000 | 0.1 | 1-hour, 24-hour |
| PM10 | `pm10` | μg/m³ | 0-2000 | 0.1 | 1-hour, 24-hour |
| Ozone | `o3` | ppb | 0-500 | 1 | 1-hour, 8-hour |
| Nitrogen Dioxide | `no2` | ppb | 0-500 | 1 | 1-hour |
| Sulfur Dioxide | `so2` | ppb | 0-500 | 1 | 1-hour |
| Carbon Monoxide | `co` | ppm | 0-50 | 0.1 | 1-hour, 8-hour |

### 2.2 Complete JSON Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "WIA-ENE-017 Air Quality Reading v1.0",
  "type": "object",
  "required": ["station_id", "timestamp", "measurements"],
  "properties": {
    "station_id": {
      "type": "string",
      "pattern": "^WIA-AQ-[A-Z]{2}-[A-Za-z0-9-]+$",
      "description": "Format: WIA-AQ-{Country}-{City}-{ID}",
      "examples": ["WIA-AQ-US-LA-001", "WIA-AQ-KR-Seoul-042"]
    },
    "timestamp": {
      "type": "string",
      "format": "date-time",
      "description": "ISO 8601 timestamp in UTC (YYYY-MM-DDTHH:MM:SSZ)"
    },
    "location": {
      "type": "object",
      "required": ["latitude", "longitude"],
      "properties": {
        "latitude": {
          "type": "number",
          "minimum": -90,
          "maximum": 90,
          "description": "WGS84 decimal degrees"
        },
        "longitude": {
          "type": "number",
          "minimum": -180,
          "maximum": 180,
          "description": "WGS84 decimal degrees"
        },
        "elevation": {
          "type": "number",
          "description": "Meters above sea level"
        },
        "address": {
          "type": "string",
          "description": "Human-readable location description"
        }
      }
    },
    "measurements": {
      "type": "object",
      "minProperties": 1,
      "properties": {
        "pm25": {"$ref": "#/definitions/pollutant_reading"},
        "pm10": {"$ref": "#/definitions/pollutant_reading"},
        "o3": {"$ref": "#/definitions/pollutant_reading"},
        "no2": {"$ref": "#/definitions/pollutant_reading"},
        "so2": {"$ref": "#/definitions/pollutant_reading"},
        "co": {"$ref": "#/definitions/pollutant_reading"}
      }
    },
    "meteorology": {
      "type": "object",
      "description": "Optional but recommended",
      "properties": {
        "temperature": {"type": "number", "description": "Celsius"},
        "humidity": {"type": "number", "minimum": 0, "maximum": 100},
        "pressure": {"type": "number", "description": "hectoPascals (hPa)"},
        "wind_speed": {"type": "number", "minimum": 0, "description": "m/s"},
        "wind_direction": {"type": "number", "minimum": 0, "maximum": 360},
        "precipitation": {"type": "number", "minimum": 0, "description": "mm/hour"}
      }
    },
    "device_status": {
      "type": "object",
      "properties": {
        "battery_level": {"type": "integer", "minimum": 0, "maximum": 100},
        "last_calibration": {"type": "string", "format": "date-time"},
        "firmware_version": {"type": "string"},
        "uptime_seconds": {"type": "integer", "minimum": 0}
      }
    },
    "aqi": {
      "type": "object",
      "description": "Calculated AQI values",
      "properties": {
        "value": {"type": "integer", "minimum": 0},
        "category": {"type": "string"},
        "dominant_pollutant": {"type": "string"},
        "color": {"type": "string", "pattern": "^#[0-9A-F]{6}$"}
      }
    }
  },
  "definitions": {
    "pollutant_reading": {
      "type": "object",
      "required": ["value", "unit", "qc_flag"],
      "properties": {
        "value": {
          "type": "number",
          "description": "Measured concentration"
        },
        "unit": {
          "type": "string",
          "enum": ["ug/m3", "ppb", "ppm"],
          "description": "Measurement unit"
        },
        "qc_flag": {
          "type": "string",
          "enum": ["valid", "suspect", "invalid", "missing", "calibration"],
          "description": "Quality control status"
        },
        "uncertainty": {
          "type": "number",
          "minimum": 0,
          "description": "±1σ measurement uncertainty in same units as value"
        },
        "averaging_period": {
          "type": "string",
          "enum": ["1min", "5min", "15min", "1hour", "8hour", "24hour"],
          "description": "Temporal averaging period"
        },
        "method": {
          "type": "string",
          "description": "Measurement method (e.g., 'beta_attenuation', 'chemiluminescence')"
        }
      }
    }
  }
}
```

### 2.3 Example Valid Data Packet

```json
{
  "station_id": "WIA-AQ-KR-Seoul-001",
  "timestamp": "2025-12-25T14:30:00Z",
  "location": {
    "latitude": 37.5665,
    "longitude": 126.9780,
    "elevation": 38,
    "address": "Seoul City Hall, Jung-gu, Seoul, South Korea"
  },
  "measurements": {
    "pm25": {
      "value": 35.4,
      "unit": "ug/m3",
      "qc_flag": "valid",
      "uncertainty": 3.5,
      "averaging_period": "1hour",
      "method": "beta_attenuation"
    },
    "pm10": {
      "value": 68.2,
      "unit": "ug/m3",
      "qc_flag": "valid",
      "uncertainty": 6.8,
      "averaging_period": "1hour"
    },
    "o3": {
      "value": 42,
      "unit": "ppb",
      "qc_flag": "valid",
      "uncertainty": 4,
      "averaging_period": "1hour",
      "method": "uv_photometry"
    },
    "no2": {
      "value": 28,
      "unit": "ppb",
      "qc_flag": "valid",
      "uncertainty": 3,
      "averaging_period": "1hour"
    },
    "so2": {
      "value": 12,
      "unit": "ppb",
      "qc_flag": "valid",
      "uncertainty": 2,
      "averaging_period": "1hour"
    },
    "co": {
      "value": 2.4,
      "unit": "ppm",
      "qc_flag": "valid",
      "uncertainty": 0.2,
      "averaging_period": "8hour"
    }
  },
  "meteorology": {
    "temperature": 18.5,
    "humidity": 62,
    "pressure": 1013.2,
    "wind_speed": 3.2,
    "wind_direction": 45,
    "precipitation": 0
  },
  "device_status": {
    "battery_level": 87,
    "last_calibration": "2025-12-01T00:00:00Z",
    "firmware_version": "2.1.3",
    "uptime_seconds": 2592000
  },
  "aqi": {
    "value": 98,
    "category": "Moderate",
    "dominant_pollutant": "pm10",
    "color": "#FFFF00"
  }
}
```

---

## 3. Unit Standardization

### 3.1 Standard Units

| Measurement Type | Standard Unit | Alternatives | Conversion Factor |
|------------------|---------------|--------------|-------------------|
| PM2.5/PM10 Mass | μg/m³ | mg/m³ | × 1000 |
| Gases (ppb range) | ppb | μg/m³ | See molecular weight table |
| Gases (ppm range) | ppm | ppb | × 1000 |
| Temperature | °C | °F, K | (°F-32)×5/9, K-273.15 |
| Pressure | hPa | mbar, mmHg, inHg | 1 hPa = 1 mbar = 0.750 mmHg |
| Wind Speed | m/s | km/h, mph | × 3.6, × 2.237 |

### 3.2 PPB to μg/m³ Conversion

At standard temperature (25°C) and pressure (1 atm):

```
C(μg/m³) = C(ppb) × (Molecular Weight / 24.45)
```

| Pollutant | Molecular Weight | Conversion Factor |
|-----------|------------------|-------------------|
| O₃ | 48.00 g/mol | 1 ppb = 1.96 μg/m³ |
| NO₂ | 46.01 g/mol | 1 ppb = 1.88 μg/m³ |
| SO₂ | 64.07 g/mol | 1 ppb = 2.62 μg/m³ |
| CO | 28.01 g/mol | 1 ppm = 1.145 mg/m³ |

---

## 4. Quality Control Flags

### 4.1 QC Flag Definitions

| Flag | Meaning | Use Cases |
|------|---------|-----------|
| `valid` | Data passed all QC checks | Normal operations |
| `suspect` | Data questionable but not conclusively invalid | Anomalies, outliers pending review |
| `invalid` | Data failed QC checks, should not be used | Out of range, instrument malfunction |
| `missing` | No data available for this measurement | Sensor offline, transmission failure |
| `calibration` | Sensor undergoing calibration | Maintenance periods, zero/span checks |

### 4.2 Automated QC Rules

Systems must implement these minimum validation checks:

```python
def validate_reading(value, pollutant, metadata):
    """WIA-ENE-017 required validation checks"""
    
    # Rule 1: Range Check
    ranges = {
        'pm25': (0, 1000), 'pm10': (0, 2000),
        'o3': (0, 500), 'no2': (0, 500),
        'so2': (0, 500), 'co': (0, 50)
    }
    min_val, max_val = ranges[pollutant]
    if not (min_val <= value <= max_val):
        return 'invalid'
    
    # Rule 2: Step Change Detection
    if metadata.get('previous_value'):
        delta = abs(value - metadata['previous_value'])
        threshold = 3 * metadata.get('historical_std', float('inf'))
        if delta > threshold:
            return 'suspect'
    
    # Rule 3: Persistence Check
    if metadata.get('unchanged_duration_hours', 0) > 6:
        return 'suspect'
    
    # Rule 4: Inter-Pollutant Consistency
    if pollutant == 'pm25' and metadata.get('pm10'):
        if value > metadata['pm10']:
            return 'invalid'  # PM2.5 cannot exceed PM10
    
    # Rule 5: Meteorological Correlation
    if metadata.get('precipitation', 0) > 5:  # heavy rain
        if value > metadata.get('dry_condition_avg', 0) * 0.7:
            return 'suspect'  # Expect washout effect
    
    return 'valid'
```

---

## 5. Metadata Requirements

### 5.1 Station Metadata

Permanent information about monitoring stations:

```json
{
  "station_metadata": {
    "station_id": "WIA-AQ-KR-Seoul-001",
    "name": "Seoul City Hall Monitoring Station",
    "type": "urban_background",
    "location": {
      "latitude": 37.5665,
      "longitude": 126.9780,
      "elevation": 38,
      "address": "Seoul City Hall, Jung-gu, Seoul, South Korea",
      "nearest_major_road_distance_m": 150,
      "land_use": "commercial"
    },
    "operator": {
      "organization": "Seoul Metropolitan Government",
      "contact_email": "airquality@seoul.go.kr",
      "website": "https://cleanair.seoul.go.kr"
    },
    "sensors": [
      {
        "pollutant": "pm25",
        "manufacturer": "Met One Instruments",
        "model": "BAM-1020",
        "serial_number": "BAM1020-2024-001",
        "installation_date": "2024-01-15",
        "last_calibration": "2025-12-01",
        "calibration_frequency_days": 90,
        "measurement_method": "beta_attenuation",
        "accuracy": "±5%",
        "data_quality_tier": 1
      }
    ],
    "operational_since": "2024-01-15T00:00:00Z",
    "timezone": "Asia/Seoul",
    "reporting_frequency_minutes": 5,
    "data_availability_url": "https://api.seoul.gov.kr/airquality/v1/stations/001"
  }
}
```

### 5.2 Required vs. Optional Metadata

**Required Fields:**
- `station_id`, `name`, `location.latitude`, `location.longitude`
- `operator.organization`, `operator.contact_email`
- At least one sensor with `pollutant`, `manufacturer`, `model`

**Recommended Fields:**
- `location.elevation`, `location.address`, `type`
- `sensor.accuracy`, `sensor.last_calibration`, `data_quality_tier`

---

## 6. Timestamp Handling

### 6.1 ISO 8601 Format

All timestamps MUST use ISO 8601 format in UTC timezone:

```
YYYY-MM-DDTHH:MM:SSZ

Examples:
2025-12-25T14:30:00Z  ✓ Correct
2025-12-25T14:30:00+00:00  ✓ Also acceptable
2025-12-25 14:30:00  ✗ Incorrect (missing T and Z)
12/25/2025 14:30  ✗ Incorrect (non-ISO format)
```

### 6.2 Averaging Period Alignment

Temporal averages must align with standard boundaries:

- **1-hour averages:** :00 to :59 (e.g., 14:00:00 to 14:59:59, reported at 15:00:00)
- **24-hour averages:** Midnight to midnight UTC
- **8-hour averages:** Rolling (use timestamp of period end)

---

## 7. Implementation Checklist

**Phase 1 Compliance requires:**

- [ ] JSON output conforms to WIA-ENE-017 schema
- [ ] All six primary pollutants measurable (or null if not equipped)
- [ ] Standard units used (μg/m³, ppb, ppm)
- [ ] QC flags implemented for all measurements
- [ ] Timestamps in ISO 8601 UTC format
- [ ] Station metadata complete with required fields
- [ ] Automated validation rules deployed
- [ ] Data retention policy documented (minimum 5 years for Tier 1)

---

**© 2025 SmileStory Inc. / WIA**  
弘益人間 (홍익인간) · Benefit All Humanity
