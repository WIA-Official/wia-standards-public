# WIA Urban Heat Island Response Data Format Standard
## Phase 1 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #EF4444 (Red - ENE Category)

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

The WIA Urban Heat Island Response Data Format Standard defines a unified format for monitoring, analyzing, and mitigating urban heat islands through comprehensive temperature data collection, surface material tracking, and cooling strategy implementation.

**Core Objectives**:
- Enable precise temperature monitoring across urban environments
- Track heat island intensity and spatial distribution
- Support data-driven cooling and mitigation strategies
- Facilitate integration with weather stations and building systems

### 1.2 Scope

This standard covers:

| Domain | Description |
|--------|-------------|
| Temperature Data | Air temperature, surface temperature, thermal mapping |
| Sensor Networks | IoT sensors, weather stations, satellite imagery |
| Surface Materials | Building materials, pavement types, vegetation coverage |
| Mitigation Strategies | Green roofs, cool pavements, tree canopy, water features |
| Impact Metrics | Energy consumption, heat-related health, cooling effectiveness |

### 1.3 Design Principles

1. **Precision**: High-resolution temperature monitoring
2. **Real-time**: Continuous data collection and analysis
3. **Scalability**: City-wide to neighborhood-level monitoring
4. **Actionable**: Data-driven mitigation recommendations
5. **Integration**: Compatible with smart city platforms

---

## Terminology

### 2.1 Core Terms

| Term | Definition |
|------|------------|
| **Heat Island** | Urban area significantly warmer than surrounding rural areas |
| **Heat Island Intensity** | Temperature difference between urban and rural areas |
| **Surface Temperature** | Temperature of ground/building surfaces (thermal infrared) |
| **Air Temperature** | Ambient air temperature at specific height |
| **Albedo** | Surface reflectivity (0=black, 1=white) |
| **Cooling Strategy** | Intervention to reduce urban heat |

### 2.2 Data Types

| Type | Description | Example |
|------|-------------|---------|
| `string` | UTF-8 encoded text | `"SENSOR-SEOUL-001"` |
| `temperature` | Celsius with precision | `{"value": 35.5, "unit": "celsius"}` |
| `gps_coordinate` | Decimal degrees | `{"lat": 37.5665, "lon": 126.9780}` |
| `timestamp` | ISO 8601 datetime | `"2025-01-15T14:30:00Z"` |
| `percentage` | 0-100 value | `{"value": 25, "unit": "percent"}` |
| `material_type` | Surface material enum | `"asphalt"` or `"grass"` |

### 2.3 Field Requirements

| Marker | Meaning |
|--------|---------|
| **REQUIRED** | Must be present |
| **OPTIONAL** | May be omitted |
| **CONDITIONAL** | Required under specific conditions |

---

## Base Structure

### 3.1 Heat Monitoring Record Format

```json
{
  "$schema": "https://wia.live/urban-heat-island/v1/schema.json",
  "version": "1.0.0",
  "recordId": "HEAT-2025-000001",
  "area": {
    "areaId": "AREA-SEOUL-GANGNAM",
    "name": "Gangnam District",
    "location": {
      "center": {
        "latitude": 37.5172,
        "longitude": 127.0473
      },
      "bounds": {
        "north": 37.5300,
        "south": 37.5000,
        "east": 127.0600,
        "west": 127.0300
      },
      "coverage": {
        "value": 39.5,
        "unit": "km2"
      }
    },
    "classification": "urban_high_density"
  },
  "measurements": {
    "timestamp": "2025-07-15T14:00:00Z",
    "airTemperature": {
      "urban": {
        "value": 35.5,
        "unit": "celsius",
        "height": 2.0
      },
      "rural": {
        "value": 28.5,
        "unit": "celsius",
        "height": 2.0
      },
      "difference": {
        "value": 7.0,
        "unit": "celsius"
      }
    },
    "surfaceTemperature": {
      "average": {
        "value": 42.5,
        "unit": "celsius"
      },
      "maximum": {
        "value": 58.3,
        "unit": "celsius",
        "location": {
          "latitude": 37.5150,
          "longitude": 127.0450,
          "material": "asphalt"
        }
      },
      "minimum": {
        "value": 26.8,
        "unit": "celsius",
        "location": {
          "latitude": 37.5180,
          "longitude": 127.0480,
          "material": "grass"
        }
      }
    },
    "heatIslandIntensity": {
      "value": 7.0,
      "unit": "celsius",
      "severity": "high"
    }
  },
  "sensors": [],
  "surfaceAnalysis": {},
  "mitigation": {},
  "environmental": {},
  "meta": {
    "hash": "sha256:...",
    "signature": "...",
    "previousHash": "..."
  }
}
```

### 3.2 Field Details

#### 3.2.1 `recordId` (REQUIRED)

```
Type: string
Format: HEAT-YYYY-NNNNNN
Description: Unique identifier for this heat monitoring record
Example: "HEAT-2025-000001"
```

#### 3.2.2 `area.classification` (REQUIRED)

```
Type: string
Valid values:
  - "urban_high_density"   : Dense urban core
  - "urban_medium_density" : Moderate urban area
  - "suburban"             : Suburban area
  - "rural"                : Rural reference area
  - "industrial"           : Industrial zone
  - "commercial"           : Commercial district
```

---

## Data Schema

### 4.1 Sensor Network Schema

```json
{
  "sensors": [
    {
      "sensorId": "SENSOR-SEOUL-001",
      "type": "temperature",
      "location": {
        "latitude": 37.5172,
        "longitude": 127.0473,
        "altitude": 2.0,
        "installationType": "rooftop",
        "height": 2.0
      },
      "measurements": {
        "airTemperature": {
          "value": 35.5,
          "unit": "celsius",
          "timestamp": "2025-07-15T14:00:00Z"
        },
        "surfaceTemperature": {
          "value": 45.2,
          "unit": "celsius",
          "timestamp": "2025-07-15T14:00:00Z"
        },
        "humidity": {
          "value": 60,
          "unit": "percent"
        },
        "windSpeed": {
          "value": 2.5,
          "unit": "m/s"
        }
      },
      "status": "active",
      "calibrationDate": "2025-01-01T00:00:00Z",
      "dataQuality": "high"
    }
  ]
}
```

### 4.2 Surface Analysis Schema

```json
{
  "surfaceAnalysis": {
    "materials": [
      {
        "materialType": "asphalt",
        "coverage": {
          "value": 35,
          "unit": "percent"
        },
        "averageTemperature": {
          "value": 52.5,
          "unit": "celsius"
        },
        "albedo": 0.12
      },
      {
        "materialType": "concrete",
        "coverage": {
          "value": 25,
          "unit": "percent"
        },
        "averageTemperature": {
          "value": 48.3,
          "unit": "celsius"
        },
        "albedo": 0.30
      },
      {
        "materialType": "grass",
        "coverage": {
          "value": 15,
          "unit": "percent"
        },
        "averageTemperature": {
          "value": 28.5,
          "unit": "celsius"
        },
        "albedo": 0.25
      },
      {
        "materialType": "tree_canopy",
        "coverage": {
          "value": 12,
          "unit": "percent"
        },
        "averageTemperature": {
          "value": 25.8,
          "unit": "celsius"
        },
        "albedo": 0.20
      },
      {
        "materialType": "building_roof",
        "coverage": {
          "value": 10,
          "unit": "percent"
        },
        "averageTemperature": {
          "value": 55.2,
          "unit": "celsius"
        },
        "albedo": 0.15
      },
      {
        "materialType": "water",
        "coverage": {
          "value": 3,
          "unit": "percent"
        },
        "averageTemperature": {
          "value": 24.5,
          "unit": "celsius"
        },
        "albedo": 0.06
      }
    ],
    "greenSpaceCoverage": {
      "value": 27,
      "unit": "percent"
    },
    "imperviousSurface": {
      "value": 70,
      "unit": "percent"
    }
  }
}
```

### 4.3 Mitigation Strategy Schema

```json
{
  "mitigation": {
    "currentStrategies": [
      {
        "strategyId": "MIT-001",
        "type": "green_roof",
        "implementationDate": "2024-05-01T00:00:00Z",
        "coverage": {
          "value": 2500,
          "unit": "m2"
        },
        "location": {
          "latitude": 37.5165,
          "longitude": 127.0455
        },
        "effectiveness": {
          "temperatureReduction": {
            "value": 2.5,
            "unit": "celsius"
          },
          "energySavings": {
            "value": 15,
            "unit": "percent"
          },
          "status": "verified"
        }
      },
      {
        "strategyId": "MIT-002",
        "type": "tree_planting",
        "implementationDate": "2023-03-15T00:00:00Z",
        "coverage": {
          "value": 150,
          "unit": "trees"
        },
        "location": {
          "latitude": 37.5180,
          "longitude": 127.0480
        },
        "effectiveness": {
          "temperatureReduction": {
            "value": 3.0,
            "unit": "celsius"
          },
          "shadeCoverage": {
            "value": 1200,
            "unit": "m2"
          },
          "status": "verified"
        }
      }
    ],
    "recommendations": [
      {
        "priority": "high",
        "type": "cool_pavement",
        "estimatedImpact": {
          "temperatureReduction": {
            "value": 1.8,
            "unit": "celsius"
          }
        },
        "targetAreas": [
          {
            "latitude": 37.5150,
            "longitude": 127.0450,
            "reason": "Maximum surface temperature hotspot"
          }
        ],
        "estimatedCost": {
          "value": 500000,
          "unit": "USD"
        }
      }
    ]
  }
}
```

### 4.4 Environmental Impact Schema

```json
{
  "environmental": {
    "energyImpact": {
      "coolingDemand": {
        "baseline": {
          "value": 150,
          "unit": "MWh/day"
        },
        "current": {
          "value": 165,
          "unit": "MWh/day"
        },
        "increase": {
          "value": 10,
          "unit": "percent"
        }
      },
      "peakDemand": {
        "time": "14:00-16:00",
        "value": 25,
        "unit": "MW"
      }
    },
    "healthImpact": {
      "heatStressDays": {
        "count": 45,
        "threshold": {
          "value": 35,
          "unit": "celsius"
        }
      },
      "vulnerablePopulation": {
        "elderly": 15000,
        "children": 8000,
        "outdoor_workers": 12000
      }
    },
    "waterUsage": {
      "irrigationDemand": {
        "value": 2500,
        "unit": "m3/day"
      },
      "coolingTowers": {
        "value": 1500,
        "unit": "m3/day"
      }
    }
  }
}
```

---

## Field Specifications

### 5.1 Material Types

| Material | Description | Typical Albedo | Heat Absorption |
|----------|-------------|----------------|-----------------|
| `asphalt` | Road pavement | 0.05-0.20 | Very High |
| `concrete` | Sidewalks, buildings | 0.25-0.40 | High |
| `grass` | Lawns, parks | 0.20-0.30 | Low |
| `tree_canopy` | Tree cover | 0.15-0.25 | Very Low |
| `building_roof` | Rooftops | 0.10-0.90 | Variable |
| `water` | Water features | 0.03-0.10 | Very Low |
| `metal` | Metal roofs | 0.10-0.70 | Variable |
| `cool_pavement` | Reflective pavement | 0.40-0.60 | Low |

### 5.2 Mitigation Strategy Types

| Type | Description | Temp Reduction | Implementation Cost |
|------|-------------|----------------|---------------------|
| `green_roof` | Vegetated roof system | 2-5°C | Medium-High |
| `cool_roof` | Reflective roof coating | 1-3°C | Low-Medium |
| `tree_planting` | Urban tree canopy | 2-4°C | Low-Medium |
| `cool_pavement` | High-albedo pavement | 1-2°C | High |
| `water_feature` | Fountains, ponds | 1-3°C | Medium-High |
| `green_wall` | Vertical vegetation | 1-2°C | Medium |
| `permeable_surface` | Permeable paving | 1-2°C | Medium |

### 5.3 Severity Levels

| Intensity (°C) | Severity | Impact | Action Required |
|----------------|----------|--------|-----------------|
| < 2°C | Low | Minimal | Monitor |
| 2-4°C | Medium | Moderate energy increase | Plan mitigation |
| 4-6°C | High | Significant health/energy impact | Implement strategies |
| > 6°C | Critical | Severe health/energy impact | Urgent intervention |

---

## Data Types

### 6.1 TypeScript Types

```typescript
type MaterialType =
  | 'asphalt'
  | 'concrete'
  | 'grass'
  | 'tree_canopy'
  | 'building_roof'
  | 'water'
  | 'metal'
  | 'cool_pavement';

type MitigationStrategy =
  | 'green_roof'
  | 'cool_roof'
  | 'tree_planting'
  | 'cool_pavement'
  | 'water_feature'
  | 'green_wall'
  | 'permeable_surface';

type AreaClassification =
  | 'urban_high_density'
  | 'urban_medium_density'
  | 'suburban'
  | 'rural'
  | 'industrial'
  | 'commercial';

type Severity = 'low' | 'medium' | 'high' | 'critical';

interface Temperature {
  value: number;
  unit: 'celsius' | 'fahrenheit';
}

interface Coverage {
  value: number;
  unit: 'percent' | 'm2' | 'km2';
}

interface GPSLocation {
  latitude: number;  // -90 to 90
  longitude: number; // -180 to 180
  altitude?: number; // meters
}
```

---

## Validation Rules

### 7.1 Required Field Validation

| Rule ID | Field | Validation |
|---------|-------|------------|
| VAL-001 | `recordId` | Must match `^HEAT-\d{4}-\d{6}$` |
| VAL-002 | `area.location.center` | Valid GPS coordinates |
| VAL-003 | `measurements.heatIslandIntensity` | Must be >= 0 |
| VAL-004 | `surfaceAnalysis.materials` | Coverage sum = 100% |
| VAL-005 | `sensors[].status` | Must be valid enum value |

### 7.2 Business Logic Validation

| Rule ID | Description | Error Code |
|---------|-------------|------------|
| BUS-001 | Urban temp must be >= rural temp | `ERR_INVALID_TEMPERATURE` |
| BUS-002 | Surface temp must be >= air temp | `ERR_INVALID_SURFACE_TEMP` |
| BUS-003 | Material coverage sum must equal 100% | `ERR_COVERAGE_MISMATCH` |
| BUS-004 | Sensor must be calibrated within 1 year | `ERR_CALIBRATION_EXPIRED` |
| BUS-005 | GPS coordinates must be within area bounds | `ERR_LOCATION_OUT_OF_BOUNDS` |

---

## Examples

### 8.1 Complete Heat Monitoring Record

```json
{
  "$schema": "https://wia.live/urban-heat-island/v1/schema.json",
  "version": "1.0.0",
  "recordId": "HEAT-2025-000001",
  "area": {
    "areaId": "AREA-SEOUL-GANGNAM",
    "name": "Gangnam District",
    "location": {
      "center": {
        "latitude": 37.5172,
        "longitude": 127.0473
      },
      "bounds": {
        "north": 37.5300,
        "south": 37.5000,
        "east": 127.0600,
        "west": 127.0300
      },
      "coverage": {
        "value": 39.5,
        "unit": "km2"
      }
    },
    "classification": "urban_high_density",
    "population": 570500
  },
  "measurements": {
    "timestamp": "2025-07-15T14:00:00Z",
    "airTemperature": {
      "urban": {
        "value": 35.5,
        "unit": "celsius",
        "height": 2.0
      },
      "rural": {
        "value": 28.5,
        "unit": "celsius",
        "height": 2.0
      },
      "difference": {
        "value": 7.0,
        "unit": "celsius"
      }
    },
    "surfaceTemperature": {
      "average": {
        "value": 42.5,
        "unit": "celsius"
      },
      "maximum": {
        "value": 58.3,
        "unit": "celsius",
        "location": {
          "latitude": 37.5150,
          "longitude": 127.0450,
          "material": "asphalt"
        }
      },
      "minimum": {
        "value": 26.8,
        "unit": "celsius",
        "location": {
          "latitude": 37.5180,
          "longitude": 127.0480,
          "material": "grass"
        }
      }
    },
    "heatIslandIntensity": {
      "value": 7.0,
      "unit": "celsius",
      "severity": "critical"
    },
    "meteorological": {
      "humidity": 60,
      "windSpeed": {
        "value": 2.5,
        "unit": "m/s"
      },
      "cloudCover": 20,
      "solarRadiation": {
        "value": 850,
        "unit": "W/m2"
      }
    }
  },
  "sensors": [
    {
      "sensorId": "SENSOR-SEOUL-001",
      "type": "temperature",
      "location": {
        "latitude": 37.5172,
        "longitude": 127.0473,
        "altitude": 50.0,
        "installationType": "rooftop",
        "height": 2.0
      },
      "measurements": {
        "airTemperature": {
          "value": 35.5,
          "unit": "celsius",
          "timestamp": "2025-07-15T14:00:00Z"
        },
        "surfaceTemperature": {
          "value": 45.2,
          "unit": "celsius",
          "timestamp": "2025-07-15T14:00:00Z"
        },
        "humidity": {
          "value": 60,
          "unit": "percent"
        }
      },
      "status": "active",
      "calibrationDate": "2025-01-01T00:00:00Z",
      "dataQuality": "high"
    }
  ],
  "surfaceAnalysis": {
    "materials": [
      {
        "materialType": "asphalt",
        "coverage": {"value": 35, "unit": "percent"},
        "averageTemperature": {"value": 52.5, "unit": "celsius"},
        "albedo": 0.12
      },
      {
        "materialType": "concrete",
        "coverage": {"value": 25, "unit": "percent"},
        "averageTemperature": {"value": 48.3, "unit": "celsius"},
        "albedo": 0.30
      },
      {
        "materialType": "grass",
        "coverage": {"value": 15, "unit": "percent"},
        "averageTemperature": {"value": 28.5, "unit": "celsius"},
        "albedo": 0.25
      },
      {
        "materialType": "tree_canopy",
        "coverage": {"value": 12, "unit": "percent"},
        "averageTemperature": {"value": 25.8, "unit": "celsius"},
        "albedo": 0.20
      },
      {
        "materialType": "building_roof",
        "coverage": {"value": 10, "unit": "percent"},
        "averageTemperature": {"value": 55.2, "unit": "celsius"},
        "albedo": 0.15
      },
      {
        "materialType": "water",
        "coverage": {"value": 3, "unit": "percent"},
        "averageTemperature": {"value": 24.5, "unit": "celsius"},
        "albedo": 0.06
      }
    ],
    "greenSpaceCoverage": {"value": 27, "unit": "percent"},
    "imperviousSurface": {"value": 70, "unit": "percent"}
  },
  "mitigation": {
    "currentStrategies": [
      {
        "strategyId": "MIT-001",
        "type": "green_roof",
        "implementationDate": "2024-05-01T00:00:00Z",
        "coverage": {"value": 2500, "unit": "m2"},
        "location": {"latitude": 37.5165, "longitude": 127.0455},
        "effectiveness": {
          "temperatureReduction": {"value": 2.5, "unit": "celsius"},
          "energySavings": {"value": 15, "unit": "percent"},
          "status": "verified"
        }
      }
    ],
    "recommendations": [
      {
        "priority": "high",
        "type": "cool_pavement",
        "estimatedImpact": {
          "temperatureReduction": {"value": 1.8, "unit": "celsius"}
        },
        "targetAreas": [
          {
            "latitude": 37.5150,
            "longitude": 127.0450,
            "reason": "Maximum surface temperature hotspot"
          }
        ],
        "estimatedCost": {"value": 500000, "unit": "USD"}
      }
    ]
  },
  "environmental": {
    "energyImpact": {
      "coolingDemand": {
        "baseline": {"value": 150, "unit": "MWh/day"},
        "current": {"value": 165, "unit": "MWh/day"},
        "increase": {"value": 10, "unit": "percent"}
      }
    },
    "healthImpact": {
      "heatStressDays": {
        "count": 45,
        "threshold": {"value": 35, "unit": "celsius"}
      }
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

**WIA Urban Heat Island Response Standard v1.0.0**

**弘益人間 (홍익인간)** - Benefit All Humanity

---

**© 2025 WIA**

**MIT License**

</div>
