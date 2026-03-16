# WIA Crop Monitoring Data Format Standard
## Phase 1 Specification

---

**Version**: 1.0.0
**Status**: Complete
**Date**: 2025-01
**Standard ID**: WIA-AGRI-006
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #84CC16 (Lime - AGRI)

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

The WIA Crop Monitoring Data Format Standard defines a unified format for tracking crop growth, health monitoring, disease detection, pest identification, and yield forecasting. This standard enables real-time observation of agricultural fields through IoT sensors, cameras, and AI-powered analysis.

**Core Objectives**:
- Enable precise tracking of crop growth stages and health metrics
- Standardize phenology data for global agricultural interoperability
- Support AI-based disease and pest detection
- Facilitate yield prediction and harvest planning
- Enable integration with weather, marketplace, and insurance systems

### 1.2 Scope

This standard covers:

| Domain | Description |
|--------|-------------|
| Growth Monitoring | Height, leaf area, chlorophyll, biomass tracking |
| Phenology Stages | Seedling, vegetative, flowering, fruiting, maturity |
| Health Assessment | Disease detection, pest identification, nutrient status |
| Environmental Data | Soil moisture, temperature, humidity, light intensity |
| Yield Forecasting | Predictive models based on growth patterns |

### 1.3 Design Principles

1. **Real-Time**: Sub-hourly data collection and streaming
2. **AI-Ready**: Structured for machine learning and computer vision
3. **Scalability**: Support from small farms to large-scale agriculture
4. **Interoperability**: Compatible with major agricultural standards (FAO, USDA)
5. **Privacy**: Farm data encrypted, aggregated insights public

---

## Terminology

### 2.1 Core Terms

| Term | Definition |
|------|------------|
| **Crop Growth Stage** | Phenological development phase (BBCH scale) |
| **Leaf Area Index (LAI)** | Total leaf surface area per unit ground area |
| **Chlorophyll Content** | SPAD value indicating photosynthetic capacity |
| **NDVI** | Normalized Difference Vegetation Index (remote sensing) |
| **Disease Pressure** | Risk level for crop diseases based on conditions |
| **Pest Infestation** | Severity and type of pest damage |

### 2.2 Data Types

| Type | Description | Example |
|------|-------------|---------|
| `crop_id` | Unique crop/field identifier | `"CROP-2025-001"` |
| `stage_code` | BBCH phenology code | `"BBCH-51"` (flowering) |
| `measurement` | Numeric value with unit | `{"value": 45.5, "unit": "cm"}` |
| `gps_location` | Geographic coordinates | `{"lat": 37.5665, "lon": 126.9780}` |
| `image_data` | Base64 or URL to crop image | `"https://..."` |

### 2.3 Field Requirements

| Marker | Meaning |
|--------|---------|
| **REQUIRED** | Must be present |
| **OPTIONAL** | May be omitted |
| **CONDITIONAL** | Required under specific conditions |

---

## Base Structure

### 3.1 Crop Monitoring Record Format

All crop monitoring records follow this base structure:

```json
{
  "cropId": "string (REQUIRED)",
  "farmId": "string (REQUIRED)",
  "timestamp": "ISO8601 (REQUIRED)",
  "location": {
    "gps": "object (REQUIRED)",
    "fieldId": "string (OPTIONAL)"
  },
  "cropType": "string (REQUIRED)",
  "variety": "string (OPTIONAL)",
  "growthStage": "object (REQUIRED)",
  "measurements": "object (REQUIRED)",
  "healthStatus": "object (OPTIONAL)",
  "predictions": "object (OPTIONAL)",
  "metadata": "object (OPTIONAL)"
}
```

### 3.2 Growth Stage Object

```json
{
  "growthStage": {
    "code": "BBCH-XX (REQUIRED)",
    "description": "string (REQUIRED)",
    "daysSinceSeeding": "integer (OPTIONAL)",
    "estimatedDaysToHarvest": "integer (OPTIONAL)"
  }
}
```

### 3.3 Measurements Object

```json
{
  "measurements": {
    "plantHeight": {"value": "number", "unit": "cm"},
    "leafAreaIndex": {"value": "number", "unit": "m²/m²"},
    "chlorophyllSPAD": {"value": "number", "unit": "SPAD"},
    "ndvi": {"value": "number", "range": "-1 to 1"},
    "biomass": {"value": "number", "unit": "kg/ha"},
    "soilMoisture": {"value": "number", "unit": "%"},
    "temperature": {"value": "number", "unit": "°C"},
    "humidity": {"value": "number", "unit": "%"}
  }
}
```

---

## Data Schema

### 4.1 Complete Crop Monitoring Schema

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "title": "WIA Crop Monitoring Data v1.0",
  "type": "object",
  "required": ["cropId", "farmId", "timestamp", "location", "cropType", "growthStage", "measurements"],
  "properties": {
    "cropId": {
      "type": "string",
      "pattern": "^CROP-[0-9]{4}-[A-Z0-9]{3,10}$",
      "description": "Unique crop batch identifier"
    },
    "farmId": {
      "type": "string",
      "description": "Farm or field owner identifier"
    },
    "timestamp": {
      "type": "string",
      "format": "date-time",
      "description": "Measurement timestamp (ISO 8601)"
    },
    "location": {
      "type": "object",
      "required": ["gps"],
      "properties": {
        "gps": {
          "type": "object",
          "required": ["latitude", "longitude"],
          "properties": {
            "latitude": {"type": "number", "minimum": -90, "maximum": 90},
            "longitude": {"type": "number", "minimum": -180, "maximum": 180}
          }
        },
        "fieldId": {"type": "string"},
        "areaHectares": {"type": "number", "minimum": 0}
      }
    },
    "cropType": {
      "type": "string",
      "enum": ["rice", "corn", "wheat", "soybean", "tomato", "pepper", "cabbage", "potato", "other"]
    },
    "variety": {
      "type": "string",
      "description": "Specific crop variety or cultivar"
    },
    "growthStage": {
      "type": "object",
      "required": ["code", "description"],
      "properties": {
        "code": {
          "type": "string",
          "pattern": "^BBCH-[0-9]{2}$",
          "description": "BBCH phenology code (00-99)"
        },
        "description": {"type": "string"},
        "daysSinceSeeding": {"type": "integer", "minimum": 0},
        "estimatedDaysToHarvest": {"type": "integer", "minimum": 0}
      }
    },
    "measurements": {
      "type": "object",
      "properties": {
        "plantHeight": {
          "type": "object",
          "properties": {
            "value": {"type": "number", "minimum": 0},
            "unit": {"type": "string", "const": "cm"}
          }
        },
        "leafAreaIndex": {
          "type": "object",
          "properties": {
            "value": {"type": "number", "minimum": 0, "maximum": 10},
            "unit": {"type": "string", "const": "m²/m²"}
          }
        },
        "chlorophyllSPAD": {
          "type": "object",
          "properties": {
            "value": {"type": "number", "minimum": 0, "maximum": 60},
            "unit": {"type": "string", "const": "SPAD"}
          }
        },
        "ndvi": {
          "type": "object",
          "properties": {
            "value": {"type": "number", "minimum": -1, "maximum": 1}
          }
        }
      }
    },
    "healthStatus": {
      "type": "object",
      "properties": {
        "overall": {"type": "string", "enum": ["healthy", "stressed", "diseased", "critical"]},
        "diseases": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "name": {"type": "string"},
              "severity": {"type": "string", "enum": ["low", "medium", "high"]},
              "confidence": {"type": "number", "minimum": 0, "maximum": 1}
            }
          }
        },
        "pests": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "species": {"type": "string"},
              "severity": {"type": "string", "enum": ["low", "medium", "high"]},
              "confidence": {"type": "number", "minimum": 0, "maximum": 1}
            }
          }
        }
      }
    },
    "predictions": {
      "type": "object",
      "properties": {
        "yieldEstimate": {
          "type": "object",
          "properties": {
            "value": {"type": "number"},
            "unit": {"type": "string", "const": "kg/ha"},
            "confidence": {"type": "number", "minimum": 0, "maximum": 1}
          }
        },
        "harvestDate": {"type": "string", "format": "date"},
        "diseaseRisk": {"type": "number", "minimum": 0, "maximum": 100}
      }
    }
  }
}
```

---

## Field Specifications

### 5.1 Crop Identification Fields

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `cropId` | string | REQUIRED | Unique identifier (e.g., "CROP-2025-001") |
| `farmId` | string | REQUIRED | Farm owner or organization ID |
| `cropType` | string | REQUIRED | Crop species (rice, corn, etc.) |
| `variety` | string | OPTIONAL | Specific cultivar or variety |

### 5.2 Growth Stage Fields (BBCH Scale)

| Code | Stage | Description |
|------|-------|-------------|
| BBCH-00 | Dry seed | Before germination |
| BBCH-10 | Seedling | First leaves emerging |
| BBCH-30 | Stem elongation | Vegetative growth |
| BBCH-50 | Flowering | Inflorescence emergence |
| BBCH-70 | Fruit development | Fruit set and growth |
| BBCH-90 | Maturity | Ready for harvest |

### 5.3 Measurement Fields

| Field | Unit | Range | Description |
|-------|------|-------|-------------|
| `plantHeight` | cm | 0-500 | Average plant height |
| `leafAreaIndex` | m²/m² | 0-10 | Leaf surface area ratio |
| `chlorophyllSPAD` | SPAD | 0-60 | Chlorophyll content |
| `ndvi` | - | -1 to 1 | Vegetation index |
| `soilMoisture` | % | 0-100 | Soil water content |
| `temperature` | °C | -20 to 50 | Air temperature |
| `humidity` | % | 0-100 | Relative humidity |

---

## Validation Rules

### 6.1 Data Integrity Rules

1. **Timestamp Validation**
   - Must be valid ISO 8601 format
   - Cannot be in the future (> current time + 5 minutes)
   - Must be within planting season for crop type

2. **GPS Validation**
   - Latitude: -90 to 90
   - Longitude: -180 to 180
   - Must be on land (not ocean)

3. **Growth Stage Consistency**
   - BBCH code must match crop type
   - Days since seeding must align with stage
   - Cannot regress to earlier stage

4. **Measurement Ranges**
   - All numeric values must be within specified ranges
   - Units must match schema
   - Negative values not allowed (except NDVI)

### 6.2 Cross-Field Validation

```python
def validate_crop_data(data):
    # LAI and NDVI correlation
    if data['measurements']['lai']['value'] > 5:
        assert data['measurements']['ndvi']['value'] > 0.6

    # Chlorophyll and health status
    if data['measurements']['chlorophyllSPAD']['value'] < 30:
        assert data['healthStatus']['overall'] in ['stressed', 'diseased']

    # Harvest readiness
    if data['growthStage']['code'] == 'BBCH-90':
        assert data['predictions']['harvestDate'] is not None
```

---

## Examples

### 7.1 Healthy Rice Crop (Vegetative Stage)

```json
{
  "cropId": "CROP-2025-RICE-001",
  "farmId": "FARM-KR-12345",
  "timestamp": "2025-06-15T10:30:00Z",
  "location": {
    "gps": {
      "latitude": 37.5665,
      "longitude": 126.9780
    },
    "fieldId": "FIELD-A-01",
    "areaHectares": 2.5
  },
  "cropType": "rice",
  "variety": "Koshihikari",
  "growthStage": {
    "code": "BBCH-30",
    "description": "Vegetative stage - stem elongation",
    "daysSinceSeeding": 45,
    "estimatedDaysToHarvest": 75
  },
  "measurements": {
    "plantHeight": {"value": 45.5, "unit": "cm"},
    "leafAreaIndex": {"value": 3.8, "unit": "m²/m²"},
    "chlorophyllSPAD": {"value": 42.0, "unit": "SPAD"},
    "ndvi": {"value": 0.75},
    "soilMoisture": {"value": 85, "unit": "%"},
    "temperature": {"value": 25.5, "unit": "°C"},
    "humidity": {"value": 70, "unit": "%"}
  },
  "healthStatus": {
    "overall": "healthy",
    "diseases": [],
    "pests": []
  },
  "predictions": {
    "yieldEstimate": {
      "value": 5500,
      "unit": "kg/ha",
      "confidence": 0.82
    },
    "harvestDate": "2025-09-01",
    "diseaseRisk": 15
  }
}
```

### 7.2 Tomato with Disease Detection

```json
{
  "cropId": "CROP-2025-TOMA-102",
  "farmId": "FARM-US-98765",
  "timestamp": "2025-07-20T14:45:00Z",
  "location": {
    "gps": {"latitude": 40.7128, "longitude": -74.0060},
    "fieldId": "GREENHOUSE-3"
  },
  "cropType": "tomato",
  "variety": "Beefsteak",
  "growthStage": {
    "code": "BBCH-70",
    "description": "Fruit development",
    "daysSinceSeeding": 80,
    "estimatedDaysToHarvest": 25
  },
  "measurements": {
    "plantHeight": {"value": 120, "unit": "cm"},
    "leafAreaIndex": {"value": 4.2, "unit": "m²/m²"},
    "chlorophyllSPAD": {"value": 28.5, "unit": "SPAD"},
    "temperature": {"value": 28.0, "unit": "°C"},
    "humidity": {"value": 85, "unit": "%"}
  },
  "healthStatus": {
    "overall": "diseased",
    "diseases": [
      {
        "name": "Late Blight (Phytophthora infestans)",
        "severity": "medium",
        "confidence": 0.87
      }
    ],
    "pests": []
  },
  "predictions": {
    "yieldEstimate": {
      "value": 45000,
      "unit": "kg/ha",
      "confidence": 0.65
    },
    "diseaseRisk": 75
  }
}
```

### 7.3 Corn with Pest Infestation

```json
{
  "cropId": "CROP-2025-CORN-505",
  "farmId": "FARM-BR-55443",
  "timestamp": "2025-08-10T09:15:00Z",
  "location": {
    "gps": {"latitude": -15.7801, "longitude": -47.9292},
    "areaHectares": 100
  },
  "cropType": "corn",
  "variety": "Pioneer 30F35",
  "growthStage": {
    "code": "BBCH-51",
    "description": "Flowering - tassel emergence",
    "daysSinceSeeding": 65,
    "estimatedDaysToHarvest": 45
  },
  "measurements": {
    "plantHeight": {"value": 180, "unit": "cm"},
    "leafAreaIndex": {"value": 5.5, "unit": "m²/m²"},
    "chlorophyllSPAD": {"value": 50.0, "unit": "SPAD"},
    "ndvi": {"value": 0.82}
  },
  "healthStatus": {
    "overall": "stressed",
    "diseases": [],
    "pests": [
      {
        "species": "Fall Armyworm (Spodoptera frugiperda)",
        "severity": "high",
        "confidence": 0.92
      }
    ]
  },
  "predictions": {
    "yieldEstimate": {
      "value": 8500,
      "unit": "kg/ha",
      "confidence": 0.70
    },
    "harvestDate": "2025-09-25",
    "diseaseRisk": 25
  }
}
```

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01 | Initial release with full schema |

---

**Philosophy**: 弘益人間 (Benefit All Humanity)
**License**: MIT
**Contact**: standards@wiastandards.com
**Repository**: https://github.com/WIA-Official/wia-standards/crop-monitoring
