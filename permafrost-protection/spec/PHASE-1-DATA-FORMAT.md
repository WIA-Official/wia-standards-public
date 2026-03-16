# WIA Permafrost Protection Data Format Standard
## Phase 1 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #EF4444 (Red)

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

The WIA Permafrost Protection Data Format Standard defines a unified format for monitoring permafrost conditions, tracking thaw rates, and preventing methane release from degrading frozen ground, enabling global coordination of climate change mitigation efforts.

**Core Objectives**:
- Enable precise monitoring of permafrost temperature and stability
- Track methane emissions from thawing permafrost
- Integrate ground sensors and satellite data for comprehensive monitoring
- Support climate model integration and early warning systems

### 1.2 Scope

This standard covers:

| Domain | Description |
|--------|-------------|
| Ground Monitoring | Temperature sensors, permafrost depth, thaw rate measurements |
| Methane Detection | CH₄ emissions, carbon release, greenhouse gas tracking |
| Sensor Integration | IoT devices, satellite data, research station networks |
| Risk Assessment | Stability analysis, infrastructure impact, ecosystem threats |
| Climate Data | Historical trends, prediction models, tipping point detection |

### 1.3 Design Principles

1. **Precision**: Accurate measurement of ground conditions and emissions
2. **Real-time**: Continuous monitoring with automated alerts
3. **Interoperability**: Compatible with climate models and research networks
4. **Scalability**: Support for regional to global-scale monitoring
5. **Prevention**: Early detection to enable intervention strategies

---

## Terminology

### 2.1 Core Terms

| Term | Definition |
|------|------------|
| **Active Layer** | Top layer of soil that thaws and refreezes seasonally |
| **Permafrost** | Ground that remains at or below 0°C for at least two consecutive years |
| **Thaw Rate** | Speed of permafrost degradation measured in cm/year |
| **Methane Flux** | Rate of CH₄ emission from thawing permafrost |
| **Monitoring Station** | Ground-based sensor installation for continuous data collection |
| **Talik** | Layer of unfrozen ground within or beneath permafrost |

### 2.2 Data Types

| Type | Description | Example |
|------|-------------|---------|
| `string` | UTF-8 encoded text | `"FROST-2025-001"` |
| `temperature` | Celsius with precision | `{"value": -5.2, "unit": "celsius"}` |
| `depth` | Meters below surface | `{"value": 15.5, "unit": "m"}` |
| `flux_rate` | Gas emission rate | `{"value": 2.5, "unit": "mg_ch4_m2_day"}` |
| `timestamp` | ISO 8601 datetime | `"2025-01-15T10:30:00Z"` |
| `gps_coordinate` | Decimal degrees | `{"lat": 64.8378, "lon": -147.7164}` |

### 2.3 Field Requirements

| Marker | Meaning |
|--------|---------|
| **REQUIRED** | Must be present |
| **OPTIONAL** | May be omitted |
| **CONDITIONAL** | Required under specific conditions |

---

## Base Structure

### 3.1 Permafrost Monitoring Record Format

```json
{
  "$schema": "https://wia.live/permafrost-protection/v1/schema.json",
  "version": "1.0.0",
  "stationId": "FROST-2025-000001",
  "status": "monitoring",
  "created": "2025-01-15T10:30:00Z",
  "lastUpdated": "2025-01-15T10:30:00Z",
  "location": {
    "gps": {
      "latitude": 64.8378,
      "longitude": -147.7164,
      "elevation": 215.5,
      "accuracy": 2.0
    },
    "region": "Alaska Interior",
    "permafrostZone": "continuous",
    "ecosystemType": "boreal_forest"
  },
  "measurements": {
    "timestamp": "2025-01-15T10:30:00Z",
    "groundTemperature": {
      "surface": {"value": -12.5, "unit": "celsius"},
      "at1m": {"value": -5.2, "unit": "celsius"},
      "at2m": {"value": -3.8, "unit": "celsius"},
      "at5m": {"value": -2.1, "unit": "celsius"}
    },
    "activeLayerThickness": {
      "value": 1.5,
      "unit": "m"
    },
    "permafrostDepth": {
      "value": 15.5,
      "unit": "m"
    },
    "thawRate": {
      "value": 2.5,
      "unit": "cm_per_year"
    },
    "soilMoisture": {
      "value": 45.0,
      "unit": "percent"
    }
  },
  "emissions": {
    "methane": {
      "flux": {"value": 15.2, "unit": "mg_ch4_m2_day"},
      "cumulativeAnnual": {"value": 5.55, "unit": "kg_ch4_year"}
    },
    "carbonDioxide": {
      "flux": {"value": 120.5, "unit": "mg_co2_m2_day"}
    }
  },
  "sensors": [],
  "risk": {
    "stabilityScore": 65,
    "alertLevel": "moderate",
    "threats": []
  },
  "meta": {
    "hash": "sha256:...",
    "signature": "...",
    "previousHash": "..."
  }
}
```

### 3.2 Field Details

#### 3.2.1 `stationId` (REQUIRED)

```
Type: string
Format: FROST-YYYY-NNNNNN
Description: Unique identifier for monitoring station
Example: "FROST-2025-000001"
```

#### 3.2.2 `status` (REQUIRED)

```
Type: string
Valid values:
  - "monitoring"   : Active data collection
  - "alert"        : Threshold exceeded, attention needed
  - "critical"     : Rapid thaw or high emissions detected
  - "maintenance"  : Sensors offline for service
  - "inactive"     : Station decommissioned
```

---

## Data Schema

### 4.1 Complete JSON Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wia.live/permafrost-protection/v1/schema.json",
  "title": "WIA Permafrost Protection Record",
  "type": "object",
  "required": ["version", "stationId", "status", "created", "location", "measurements"],
  "properties": {
    "version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+\\.\\d+$"
    },
    "stationId": {
      "type": "string",
      "pattern": "^FROST-\\d{4}-\\d{6}$"
    },
    "status": {
      "type": "string",
      "enum": ["monitoring", "alert", "critical", "maintenance", "inactive"]
    },
    "created": {
      "type": "string",
      "format": "date-time"
    },
    "location": {
      "type": "object",
      "required": ["gps", "permafrostZone"],
      "properties": {
        "gps": {
          "type": "object",
          "required": ["latitude", "longitude"],
          "properties": {
            "latitude": {
              "type": "number",
              "minimum": -90,
              "maximum": 90
            },
            "longitude": {
              "type": "number",
              "minimum": -180,
              "maximum": 180
            },
            "elevation": {"type": "number"},
            "accuracy": {"type": "number"}
          }
        },
        "region": {"type": "string"},
        "permafrostZone": {
          "type": "string",
          "enum": ["continuous", "discontinuous", "sporadic", "isolated", "alpine"]
        },
        "ecosystemType": {"type": "string"}
      }
    },
    "measurements": {
      "type": "object",
      "required": ["timestamp", "groundTemperature", "permafrostDepth"],
      "properties": {
        "timestamp": {
          "type": "string",
          "format": "date-time"
        },
        "groundTemperature": {
          "type": "object",
          "required": ["surface"],
          "properties": {
            "surface": {
              "type": "object",
              "required": ["value", "unit"],
              "properties": {
                "value": {"type": "number"},
                "unit": {"type": "string", "enum": ["celsius", "fahrenheit", "kelvin"]}
              }
            }
          }
        },
        "activeLayerThickness": {
          "type": "object",
          "required": ["value", "unit"],
          "properties": {
            "value": {"type": "number", "minimum": 0},
            "unit": {"type": "string", "enum": ["m", "cm", "ft"]}
          }
        },
        "permafrostDepth": {
          "type": "object",
          "required": ["value", "unit"],
          "properties": {
            "value": {"type": "number", "minimum": 0},
            "unit": {"type": "string", "enum": ["m", "cm", "ft"]}
          }
        },
        "thawRate": {
          "type": "object",
          "properties": {
            "value": {"type": "number"},
            "unit": {"type": "string"}
          }
        }
      }
    },
    "emissions": {
      "type": "object",
      "properties": {
        "methane": {
          "type": "object",
          "properties": {
            "flux": {
              "type": "object",
              "required": ["value", "unit"],
              "properties": {
                "value": {"type": "number"},
                "unit": {"type": "string"}
              }
            }
          }
        }
      }
    }
  }
}
```

### 4.2 Sensor Array Schema

```json
{
  "sensors": [
    {
      "sensorId": "TEMP-001",
      "type": "temperature",
      "depth": {"value": 1.0, "unit": "m"},
      "reading": {
        "value": -5.2,
        "unit": "celsius",
        "timestamp": "2025-01-15T10:30:00Z"
      },
      "status": "active",
      "calibrationDate": "2025-01-01T00:00:00Z"
    },
    {
      "sensorId": "METH-001",
      "type": "methane",
      "location": "surface",
      "reading": {
        "value": 15.2,
        "unit": "mg_ch4_m2_day",
        "timestamp": "2025-01-15T10:30:00Z"
      },
      "status": "active"
    },
    {
      "sensorId": "GPS-001",
      "type": "gps",
      "reading": {
        "latitude": 64.8378,
        "longitude": -147.7164,
        "elevation": 215.5,
        "timestamp": "2025-01-15T10:30:00Z"
      },
      "status": "active"
    }
  ]
}
```

### 4.3 Risk Assessment Schema

```json
{
  "risk": {
    "stabilityScore": 65,
    "alertLevel": "moderate",
    "threats": [
      {
        "type": "rapid_thaw",
        "severity": "medium",
        "description": "Thaw rate exceeds 2 cm/year",
        "detectedAt": "2025-01-15T10:30:00Z"
      },
      {
        "type": "methane_spike",
        "severity": "low",
        "description": "CH₄ emissions 10% above baseline",
        "detectedAt": "2025-01-15T10:30:00Z"
      }
    ],
    "infrastructureRisk": {
      "nearbyStructures": ["research_station", "pipeline"],
      "riskLevel": "moderate",
      "recommendations": ["increase_monitoring_frequency", "structural_inspection"]
    }
  }
}
```

---

## Field Specifications

### 5.1 Location Fields

| Field | Type | Required | Description | Example |
|-------|------|----------|-------------|---------|
| `location.gps.latitude` | number | REQUIRED | GPS latitude (-90 to 90) | `64.8378` |
| `location.gps.longitude` | number | REQUIRED | GPS longitude (-180 to 180) | `-147.7164` |
| `location.gps.elevation` | number | OPTIONAL | Elevation above sea level (m) | `215.5` |
| `location.permafrostZone` | string | REQUIRED | Permafrost zone classification | `"continuous"` |

**Valid permafrost zones:**

| Value | Description | Permafrost Extent |
|-------|-------------|-------------------|
| `continuous` | Continuous permafrost | 90-100% coverage |
| `discontinuous` | Discontinuous permafrost | 50-90% coverage |
| `sporadic` | Sporadic permafrost | 10-50% coverage |
| `isolated` | Isolated patches | <10% coverage |
| `alpine` | High-altitude permafrost | Varies by elevation |

### 5.2 Temperature Measurements

| Depth | Purpose | Typical Range (°C) |
|-------|---------|-------------------|
| Surface | Air-ground interface | -40 to +30 |
| 1m | Active layer monitoring | -15 to +5 |
| 2m | Transition zone | -10 to +2 |
| 5m | Deep permafrost | -8 to 0 |

### 5.3 Emission Types

| Gas | Unit | Detection Method | Climate Impact |
|-----|------|------------------|----------------|
| Methane (CH₄) | mg/m²/day | Chamber sampling, eddy covariance | 28x CO₂ (100-year GWP) |
| Carbon Dioxide (CO₂) | mg/m²/day | Infrared sensors | Baseline GHG |
| Nitrous Oxide (N₂O) | µg/m²/day | Gas chromatography | 265x CO₂ (100-year GWP) |

---

## Data Types

### 6.1 Custom Types

```typescript
type PermafrostStatus =
  | 'monitoring'
  | 'alert'
  | 'critical'
  | 'maintenance'
  | 'inactive';

type PermafrostZone =
  | 'continuous'
  | 'discontinuous'
  | 'sporadic'
  | 'isolated'
  | 'alpine';

type AlertLevel =
  | 'normal'
  | 'moderate'
  | 'high'
  | 'critical';

type ThreatType =
  | 'rapid_thaw'
  | 'methane_spike'
  | 'subsidence'
  | 'talik_formation'
  | 'infrastructure_damage';

interface Temperature {
  value: number;
  unit: 'celsius' | 'fahrenheit' | 'kelvin';
}

interface Depth {
  value: number;
  unit: 'm' | 'cm' | 'ft';
}

interface EmissionFlux {
  value: number;
  unit: 'mg_ch4_m2_day' | 'mg_co2_m2_day' | 'ug_n2o_m2_day';
}
```

### 6.2 Ecosystem Types

| Code | Description | Region |
|------|-------------|--------|
| `tundra` | Arctic tundra | High Arctic |
| `boreal_forest` | Taiga/boreal forest | Subarctic |
| `wetland` | Permafrost wetlands | Lowlands |
| `alpine_meadow` | High-altitude grassland | Mountains |
| `coastal` | Coastal permafrost | Arctic Ocean |

---

## Validation Rules

### 7.1 Required Field Validation

| Rule ID | Field | Validation |
|---------|-------|------------|
| VAL-001 | `stationId` | Must match `^FROST-\d{4}-\d{6}$` |
| VAL-002 | `location.gps` | Latitude: -90 to 90, Longitude: -180 to 180 |
| VAL-003 | `measurements.groundTemperature` | Surface temp required |
| VAL-004 | `measurements.timestamp` | Cannot be in future |
| VAL-005 | `status` | Must be valid enum value |

### 7.2 Business Logic Validation

| Rule ID | Description | Error Code |
|---------|-------------|------------|
| BUS-001 | GPS coordinates must be in permafrost region | `ERR_INVALID_LOCATION` |
| BUS-002 | Temperature at depth must be ≤ surface temp | `ERR_INVALID_TEMP_PROFILE` |
| BUS-003 | Thaw rate cannot be negative | `ERR_INVALID_THAW_RATE` |
| BUS-004 | Methane flux must be non-negative | `ERR_INVALID_EMISSION` |
| BUS-005 | Active layer thickness ≤ permafrost depth | `ERR_INVALID_DEPTH` |

### 7.3 Error Codes

| Code | Message | Description |
|------|---------|-------------|
| `ERR_INVALID_STATION` | Invalid station format | Station ID format violation |
| `ERR_INVALID_LOCATION` | Invalid GPS coordinates | Not in permafrost region |
| `ERR_TEMP_ANOMALY` | Temperature anomaly detected | Unrealistic temperature reading |
| `ERR_SENSOR_OFFLINE` | Sensor offline | Missing sensor data |
| `ERR_THRESHOLD_EXCEEDED` | Alert threshold exceeded | Automatic alert triggered |

---

## Examples

### 8.1 Valid Monitoring Record - Arctic Station

```json
{
  "$schema": "https://wia.live/permafrost-protection/v1/schema.json",
  "version": "1.0.0",
  "stationId": "FROST-2025-000001",
  "status": "monitoring",
  "created": "2025-01-15T10:30:00Z",
  "lastUpdated": "2025-01-15T10:30:00Z",
  "location": {
    "gps": {
      "latitude": 64.8378,
      "longitude": -147.7164,
      "elevation": 215.5,
      "accuracy": 2.0
    },
    "region": "Alaska Interior",
    "permafrostZone": "continuous",
    "ecosystemType": "boreal_forest"
  },
  "measurements": {
    "timestamp": "2025-01-15T10:30:00Z",
    "groundTemperature": {
      "surface": {"value": -12.5, "unit": "celsius"},
      "at1m": {"value": -5.2, "unit": "celsius"},
      "at2m": {"value": -3.8, "unit": "celsius"},
      "at5m": {"value": -2.1, "unit": "celsius"}
    },
    "activeLayerThickness": {"value": 1.5, "unit": "m"},
    "permafrostDepth": {"value": 15.5, "unit": "m"},
    "thawRate": {"value": 2.5, "unit": "cm_per_year"},
    "soilMoisture": {"value": 45.0, "unit": "percent"}
  },
  "emissions": {
    "methane": {
      "flux": {"value": 15.2, "unit": "mg_ch4_m2_day"},
      "cumulativeAnnual": {"value": 5.55, "unit": "kg_ch4_year"}
    },
    "carbonDioxide": {
      "flux": {"value": 120.5, "unit": "mg_co2_m2_day"}
    }
  },
  "risk": {
    "stabilityScore": 65,
    "alertLevel": "moderate",
    "threats": [
      {
        "type": "rapid_thaw",
        "severity": "medium",
        "description": "Thaw rate exceeds 2 cm/year",
        "detectedAt": "2025-01-15T10:30:00Z"
      }
    ]
  }
}
```

### 8.2 Valid Record - Alpine Permafrost

```json
{
  "$schema": "https://wia.live/permafrost-protection/v1/schema.json",
  "version": "1.0.0",
  "stationId": "FROST-2025-000042",
  "status": "alert",
  "created": "2025-01-01T00:00:00Z",
  "lastUpdated": "2025-01-15T14:30:00Z",
  "location": {
    "gps": {
      "latitude": 46.5547,
      "longitude": 7.9763,
      "elevation": 3450.0,
      "accuracy": 3.0
    },
    "region": "Swiss Alps",
    "permafrostZone": "alpine",
    "ecosystemType": "alpine_meadow"
  },
  "measurements": {
    "timestamp": "2025-01-15T14:30:00Z",
    "groundTemperature": {
      "surface": {"value": -2.1, "unit": "celsius"},
      "at1m": {"value": -0.8, "unit": "celsius"},
      "at2m": {"value": -0.5, "unit": "celsius"}
    },
    "activeLayerThickness": {"value": 2.5, "unit": "m"},
    "permafrostDepth": {"value": 8.0, "unit": "m"},
    "thawRate": {"value": 4.2, "unit": "cm_per_year"}
  },
  "emissions": {
    "methane": {
      "flux": {"value": 8.5, "unit": "mg_ch4_m2_day"}
    }
  },
  "risk": {
    "stabilityScore": 45,
    "alertLevel": "high",
    "threats": [
      {
        "type": "rapid_thaw",
        "severity": "high",
        "description": "Thaw rate exceeds 4 cm/year - infrastructure at risk",
        "detectedAt": "2025-01-15T14:30:00Z"
      }
    ],
    "infrastructureRisk": {
      "nearbyStructures": ["ski_resort", "cable_car"],
      "riskLevel": "high",
      "recommendations": ["immediate_structural_assessment", "evacuation_planning"]
    }
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

**WIA Permafrost Protection Standard v1.0.0**

**弘益人間 (홍익인간)** - Benefit All Humanity

---

**© 2025 WIA**

**MIT License**

</div>
