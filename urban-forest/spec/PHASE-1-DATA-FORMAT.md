# WIA Urban Forest Creation Data Format Standard
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

The WIA Urban Forest Creation Data Format Standard defines a unified format for tracking urban tree planting, forest corridor development, biodiversity monitoring, and carbon offset verification in urban environments.

**Core Objectives**:
- Enable precise tracking of tree planting from seedling to maturity
- Monitor forest health, growth, and biodiversity
- Calculate and verify carbon sequestration
- Facilitate collaboration between city planners, environmental agencies, and citizens

### 1.2 Scope

This standard covers:

| Domain | Description |
|--------|-------------|
| Tree Inventory | Species, location, age, health status |
| Forest Corridors | Connectivity mapping, wildlife pathways |
| Soil Data | pH, moisture, nutrient composition |
| Carbon Tracking | CO2 absorption, oxygen production, air quality |
| Biodiversity | Flora and fauna species counts, ecosystem health |

### 1.3 Design Principles

1. **Sustainability**: Long-term forest health monitoring
2. **Transparency**: Open data for public engagement
3. **Scalability**: Support from single tree to city-wide forests
4. **Interoperability**: Compatible with GIS and environmental systems
5. **Community**: Citizen participation in urban greening

---

## Terminology

### 2.1 Core Terms

| Term | Definition |
|------|------------|
| **Urban Forest** | Trees and green spaces within city boundaries |
| **Forest Corridor** | Connected tree canopy providing wildlife pathways |
| **Canopy Coverage** | Area covered by tree crown/foliage (m²) |
| **DBH** | Diameter at Breast Height - tree trunk measurement |
| **Carbon Sequestration** | CO2 absorbed and stored by trees |
| **Biodiversity Index** | Measure of species variety in forest area |

### 2.2 Data Types

| Type | Description | Example |
|------|-------------|---------|
| `string` | UTF-8 encoded text | `"FOREST-2025-001"` |
| `gps_coordinate` | Decimal degrees | `{"lat": 37.5665, "lon": 126.9780}` |
| `species_code` | Tree species identifier | `"QUERCUS_ROBUR"` |
| `area` | Square meters | `{"value": 25.5, "unit": "m2"}` |
| `timestamp` | ISO 8601 datetime | `"2025-01-15T10:30:00Z"` |
| `health_status` | Tree condition | `"healthy"` |

### 2.3 Field Requirements

| Marker | Meaning |
|--------|---------|
| **REQUIRED** | Must be present |
| **OPTIONAL** | May be omitted |
| **CONDITIONAL** | Required under specific conditions |

---

## Base Structure

### 3.1 Urban Forest Record Format

```json
{
  "$schema": "https://wia.live/urban-forest/v1/schema.json",
  "version": "1.0.0",
  "forestId": "FOREST-2025-SEOUL-001",
  "status": "active",
  "created": "2025-01-15T10:30:00Z",
  "lastUpdated": "2025-01-15T10:30:00Z",
  "location": {
    "city": "Seoul",
    "district": "Gangnam-gu",
    "gps": {
      "latitude": 37.5665,
      "longitude": 126.9780,
      "altitude": 50,
      "accuracy": 2.0
    },
    "address": "123 Urban Forest Park, Seoul",
    "zoneType": "public_park"
  },
  "trees": [
    {
      "treeId": "TREE-001",
      "species": {
        "scientificName": "Quercus acutissima",
        "commonName": "Sawtooth Oak",
        "localName": "상수리나무"
      },
      "plantedDate": "2020-03-15T00:00:00Z",
      "age": 5,
      "height": {
        "value": 4.5,
        "unit": "m"
      },
      "dbh": {
        "value": 15.2,
        "unit": "cm"
      },
      "canopyCoverage": {
        "value": 12.5,
        "unit": "m2"
      },
      "health": "healthy",
      "lastInspection": "2025-01-10T00:00:00Z"
    }
  ],
  "soil": {
    "type": "loam",
    "pH": 6.5,
    "moisture": 35,
    "nutrients": {
      "nitrogen": "medium",
      "phosphorus": "high",
      "potassium": "medium"
    }
  },
  "carbon": {
    "annualSequestration": {
      "value": 2177,
      "unit": "kg_co2"
    },
    "totalSequestered": {
      "value": 10885,
      "unit": "kg_co2"
    },
    "oxygenProduction": {
      "value": 1582,
      "unit": "kg_o2"
    }
  },
  "biodiversity": {
    "birdSpecies": 15,
    "insectSpecies": 45,
    "plantSpecies": 8,
    "biodiversityIndex": 68
  },
  "monitoring": {
    "sensors": [],
    "inspections": []
  },
  "meta": {
    "hash": "sha256:...",
    "signature": "...",
    "previousHash": "..."
  }
}
```

### 3.2 Field Details

#### 3.2.1 `forestId` (REQUIRED)

```
Type: string
Format: FOREST-YYYY-CITY-NNN
Description: Unique identifier for this urban forest
Example: "FOREST-2025-SEOUL-001"
```

#### 3.2.2 `status` (REQUIRED)

```
Type: string
Valid values:
  - "planned"    : Forest planning stage
  - "planting"   : Trees being planted
  - "active"     : Established and growing
  - "mature"     : Fully mature forest
  - "maintenance": Under maintenance/restoration
  - "protected"  : Protected conservation area
```

---

## Data Schema

### 4.1 Complete JSON Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wia.live/urban-forest/v1/schema.json",
  "title": "WIA Urban Forest Record",
  "type": "object",
  "required": ["version", "forestId", "status", "created", "location", "trees"],
  "properties": {
    "version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+\\.\\d+$"
    },
    "forestId": {
      "type": "string",
      "pattern": "^FOREST-\\d{4}-[A-Z]+-\\d{3}$"
    },
    "status": {
      "type": "string",
      "enum": ["planned", "planting", "active", "mature", "maintenance", "protected"]
    },
    "created": {
      "type": "string",
      "format": "date-time"
    },
    "location": {
      "type": "object",
      "required": ["city", "gps"],
      "properties": {
        "city": {"type": "string"},
        "district": {"type": "string"},
        "gps": {
          "type": "object",
          "required": ["latitude", "longitude"],
          "properties": {
            "latitude": {"type": "number", "minimum": -90, "maximum": 90},
            "longitude": {"type": "number", "minimum": -180, "maximum": 180},
            "altitude": {"type": "number"},
            "accuracy": {"type": "number"}
          }
        },
        "address": {"type": "string"},
        "zoneType": {
          "type": "string",
          "enum": ["public_park", "street_trees", "private_garden", "green_roof", "forest_corridor"]
        }
      }
    },
    "trees": {
      "type": "array",
      "minItems": 1,
      "items": {
        "type": "object",
        "required": ["treeId", "species", "plantedDate", "health"],
        "properties": {
          "treeId": {"type": "string"},
          "species": {
            "type": "object",
            "required": ["scientificName", "commonName"],
            "properties": {
              "scientificName": {"type": "string"},
              "commonName": {"type": "string"},
              "localName": {"type": "string"}
            }
          },
          "plantedDate": {"type": "string", "format": "date-time"},
          "age": {"type": "number", "minimum": 0},
          "height": {
            "type": "object",
            "properties": {
              "value": {"type": "number"},
              "unit": {"type": "string", "enum": ["m", "cm", "ft"]}
            }
          },
          "dbh": {
            "type": "object",
            "properties": {
              "value": {"type": "number"},
              "unit": {"type": "string", "enum": ["cm", "in"]}
            }
          },
          "canopyCoverage": {
            "type": "object",
            "properties": {
              "value": {"type": "number"},
              "unit": {"type": "string", "enum": ["m2", "ft2"]}
            }
          },
          "health": {
            "type": "string",
            "enum": ["excellent", "healthy", "fair", "poor", "critical", "dead"]
          },
          "lastInspection": {"type": "string", "format": "date-time"}
        }
      }
    },
    "soil": {
      "type": "object",
      "properties": {
        "type": {
          "type": "string",
          "enum": ["sandy", "clay", "loam", "silt", "peat"]
        },
        "pH": {"type": "number", "minimum": 0, "maximum": 14},
        "moisture": {"type": "number", "minimum": 0, "maximum": 100},
        "nutrients": {
          "type": "object",
          "properties": {
            "nitrogen": {"type": "string", "enum": ["low", "medium", "high"]},
            "phosphorus": {"type": "string", "enum": ["low", "medium", "high"]},
            "potassium": {"type": "string", "enum": ["low", "medium", "high"]}
          }
        }
      }
    },
    "carbon": {
      "type": "object",
      "properties": {
        "annualSequestration": {
          "type": "object",
          "properties": {
            "value": {"type": "number"},
            "unit": {"type": "string", "enum": ["kg_co2", "ton_co2"]}
          }
        },
        "totalSequestered": {
          "type": "object",
          "properties": {
            "value": {"type": "number"},
            "unit": {"type": "string", "enum": ["kg_co2", "ton_co2"]}
          }
        },
        "oxygenProduction": {
          "type": "object",
          "properties": {
            "value": {"type": "number"},
            "unit": {"type": "string", "enum": ["kg_o2", "ton_o2"]}
          }
        }
      }
    },
    "biodiversity": {
      "type": "object",
      "properties": {
        "birdSpecies": {"type": "number"},
        "insectSpecies": {"type": "number"},
        "plantSpecies": {"type": "number"},
        "biodiversityIndex": {"type": "number", "minimum": 0, "maximum": 100}
      }
    }
  }
}
```

---

## Field Specifications

### 5.1 Tree Species Classification

Common urban tree species with carbon sequestration rates:

| Scientific Name | Common Name | CO2/year (mature) | Growth Rate |
|----------------|-------------|-------------------|-------------|
| Quercus robur | English Oak | 28 kg | Slow |
| Acer platanoides | Norway Maple | 24 kg | Medium |
| Platanus acerifolia | London Plane | 35 kg | Fast |
| Ginkgo biloba | Ginkgo | 18 kg | Slow |
| Zelkova serrata | Japanese Zelkova | 22 kg | Medium |

### 5.2 Health Status Criteria

| Status | Description | Action Required |
|--------|-------------|----------------|
| `excellent` | Perfect condition, vigorous growth | Routine monitoring |
| `healthy` | Good condition, normal growth | Standard maintenance |
| `fair` | Minor issues, some stress | Increased monitoring |
| `poor` | Significant decline | Immediate intervention |
| `critical` | Severe decline or disease | Emergency treatment |
| `dead` | Tree has died | Removal and replacement |

### 5.3 Zone Types

| Type | Description | Characteristics |
|------|-------------|----------------|
| `public_park` | City parks | High public access |
| `street_trees` | Roadside trees | Linear corridors |
| `private_garden` | Private property | Limited access |
| `green_roof` | Rooftop gardens | Elevated, contained |
| `forest_corridor` | Wildlife pathways | Ecological connectivity |

---

## Validation Rules

### 7.1 Required Field Validation

| Rule ID | Field | Validation |
|---------|-------|------------|
| VAL-001 | `forestId` | Must match `^FOREST-\\d{4}-[A-Z]+-\\d{3}$` |
| VAL-002 | `location.gps` | Latitude: -90 to 90, Longitude: -180 to 180 |
| VAL-003 | `trees` | Array must not be empty |
| VAL-004 | `trees[].health` | Must be valid enum value |
| VAL-005 | `soil.pH` | Must be between 0 and 14 |

### 7.2 Business Logic Validation

| Rule ID | Description | Error Code |
|---------|-------------|------------|
| BUS-001 | Tree age must match planted date | `ERR_AGE_MISMATCH` |
| BUS-002 | GPS coordinates must be within city boundaries | `ERR_INVALID_LOCATION` |
| BUS-003 | Planted date cannot be in future | `ERR_FUTURE_DATE` |
| BUS-004 | DBH must increase with tree age | `ERR_INVALID_GROWTH` |
| BUS-005 | Carbon sequestration must correlate with canopy coverage | `ERR_CARBON_CALC` |

---

## Examples

### 8.1 Valid Urban Forest Record - Public Park

```json
{
  "$schema": "https://wia.live/urban-forest/v1/schema.json",
  "version": "1.0.0",
  "forestId": "FOREST-2025-SEOUL-001",
  "status": "active",
  "created": "2020-03-15T00:00:00Z",
  "lastUpdated": "2025-01-15T10:30:00Z",
  "location": {
    "city": "Seoul",
    "district": "Gangnam-gu",
    "gps": {
      "latitude": 37.5172,
      "longitude": 127.0473,
      "altitude": 45,
      "accuracy": 1.5
    },
    "address": "Bongeunsa-ro, Gangnam-gu, Seoul",
    "zoneType": "public_park"
  },
  "trees": [
    {
      "treeId": "TREE-001",
      "species": {
        "scientificName": "Zelkova serrata",
        "commonName": "Japanese Zelkova",
        "localName": "느티나무"
      },
      "plantedDate": "2020-03-15T00:00:00Z",
      "age": 5,
      "height": {
        "value": 5.2,
        "unit": "m"
      },
      "dbh": {
        "value": 18.5,
        "unit": "cm"
      },
      "canopyCoverage": {
        "value": 15.8,
        "unit": "m2"
      },
      "health": "healthy",
      "lastInspection": "2025-01-10T00:00:00Z"
    }
  ],
  "soil": {
    "type": "loam",
    "pH": 6.8,
    "moisture": 42,
    "nutrients": {
      "nitrogen": "medium",
      "phosphorus": "high",
      "potassium": "medium"
    }
  },
  "carbon": {
    "annualSequestration": {
      "value": 110,
      "unit": "kg_co2"
    },
    "totalSequestered": {
      "value": 550,
      "unit": "kg_co2"
    },
    "oxygenProduction": {
      "value": 80,
      "unit": "kg_o2"
    }
  },
  "biodiversity": {
    "birdSpecies": 8,
    "insectSpecies": 25,
    "plantSpecies": 12,
    "biodiversityIndex": 55
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

**WIA Urban Forest Creation Standard v1.0.0**

**弘益人間 (홍익인간)** - Benefit All Humanity

---

**© 2025 WIA**

**MIT License**

</div>
