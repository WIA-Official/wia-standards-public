# WIA Ocean Plastic Track Data Format Standard
## Phase 1 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #84CC16 (Lime)

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

The WIA Ocean Plastic Track Data Format Standard defines a unified format for tracking plastic waste throughout its lifecycle from ocean collection to recycling, enabling global coordination of ocean cleanup efforts and transparent recycling chain management.

**Core Objectives**:
- Enable precise tracking of plastic waste from collection to recycling
- Integrate GPS and IoT sensor data for real-time monitoring
- Support environmental impact measurement and reporting
- Facilitate collaboration between cleanup organizations, recyclers, and governments

### 1.2 Scope

This standard covers:

| Domain | Description |
|--------|-------------|
| Collection Data | GPS coordinates, collection timestamps, plastic types |
| Material Classification | Plastic resin types, contamination levels, weight |
| IoT Sensor Integration | Real-time tracking, environmental sensors, device status |
| Recycling Chain | Processing stages, facility tracking, end-product data |
| Environmental Impact | Carbon footprint, marine life impact, cleanup metrics |

### 1.3 Design Principles

1. **Traceability**: Every plastic item tracked from ocean to recycling
2. **Real-time**: GPS and IoT integration for live tracking
3. **Transparency**: Open data for environmental impact verification
4. **Interoperability**: Compatible with global cleanup and recycling systems
5. **Sustainability**: Metrics-driven approach to ocean conservation

---

## Terminology

### 2.1 Core Terms

| Term | Definition |
|------|------------|
| **Collection Event** | Single instance of plastic waste collection from ocean/beach |
| **Plastic Batch** | Group of plastic items collected together |
| **Resin Code** | Standard plastic type identifier (ASTM D7611) |
| **IoT Collector** | GPS/sensor-enabled collection device or vessel |
| **Processing Stage** | Step in recycling chain (sorting, cleaning, recycling) |
| **Impact Metric** | Environmental measurement (CO2, marine life saved) |

### 2.2 Data Types

| Type | Description | Example |
|------|-------------|---------|
| `string` | UTF-8 encoded text | `"BATCH-2025-001"` |
| `gps_coordinate` | Decimal degrees | `{"lat": 37.5665, "lon": 126.9780}` |
| `resin_code` | ASTM D7611 code | `"PET"` or `1` |
| `weight` | Kilograms with precision | `{"value": 15.5, "unit": "kg"}` |
| `timestamp` | ISO 8601 datetime | `"2025-01-15T10:30:00Z"` |
| `device_id` | IoT device identifier | `"IOT-DEV-001"` |

### 2.3 Field Requirements

| Marker | Meaning |
|--------|---------|
| **REQUIRED** | Must be present |
| **OPTIONAL** | May be omitted |
| **CONDITIONAL** | Required under specific conditions |

---

## Base Structure

### 3.1 Plastic Tracking Record Format

```json
{
  "$schema": "https://wia.live/ocean-plastic-track/v1/schema.json",
  "version": "1.0.0",
  "batchId": "BATCH-2025-000001",
  "status": "collected",
  "created": "2025-01-15T10:30:00Z",
  "lastUpdated": "2025-01-15T10:30:00Z",
  "collection": {
    "eventId": "COLLECT-2025-001",
    "location": {
      "gps": {
        "latitude": 37.5665,
        "longitude": 126.9780,
        "altitude": 0,
        "accuracy": 5.0
      },
      "description": "East Sea, Korea",
      "oceanZone": "EEZ-KR-EAST"
    },
    "timestamp": "2025-01-15T10:30:00Z",
    "collector": {
      "deviceId": "IOT-DEV-001",
      "organizationId": "ORG-CLEANUP-001",
      "vesselId": "VESSEL-001",
      "operator": "cleanup-team-alpha"
    },
    "method": "vessel_net"
  },
  "material": {
    "totalWeight": {
      "value": 45.5,
      "unit": "kg"
    },
    "composition": [
      {
        "resinCode": "PET",
        "percentage": 60,
        "weight": {"value": 27.3, "unit": "kg"},
        "condition": "good"
      },
      {
        "resinCode": "HDPE",
        "percentage": 30,
        "weight": {"value": 13.65, "unit": "kg"},
        "condition": "contaminated"
      }
    ],
    "contaminationLevel": "low",
    "estimatedAge": "1-5_years"
  },
  "iot": {
    "sensors": [],
    "tracking": {},
    "battery": {}
  },
  "recycling": {
    "currentStage": "collected",
    "facilities": [],
    "history": []
  },
  "environmental": {
    "carbonFootprint": {},
    "impactMetrics": {}
  },
  "meta": {
    "hash": "sha256:...",
    "signature": "...",
    "previousHash": "..."
  }
}
```

### 3.2 Field Details

#### 3.2.1 `batchId` (REQUIRED)

```
Type: string
Format: BATCH-YYYY-NNNNNN
Description: Unique identifier for this plastic batch
Example: "BATCH-2025-000001"
```

#### 3.2.2 `status` (REQUIRED)

```
Type: string
Valid values:
  - "collected"    : Collected from ocean/beach
  - "transported"  : In transit to facility
  - "sorting"      : Being sorted by material type
  - "processing"   : Being cleaned/processed
  - "recycled"     : Converted to recycled material
  - "disposed"     : Non-recyclable, properly disposed
```

---

## Data Schema

### 4.1 Complete JSON Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wia.live/ocean-plastic-track/v1/schema.json",
  "title": "WIA Ocean Plastic Track Record",
  "type": "object",
  "required": ["version", "batchId", "status", "created", "collection", "material"],
  "properties": {
    "version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+\\.\\d+$"
    },
    "batchId": {
      "type": "string",
      "pattern": "^BATCH-\\d{4}-\\d{6}$"
    },
    "status": {
      "type": "string",
      "enum": ["collected", "transported", "sorting", "processing", "recycled", "disposed"]
    },
    "created": {
      "type": "string",
      "format": "date-time"
    },
    "collection": {
      "type": "object",
      "required": ["eventId", "location", "timestamp", "collector"],
      "properties": {
        "eventId": {
          "type": "string",
          "pattern": "^COLLECT-\\d{4}-\\d{3}$"
        },
        "location": {
          "type": "object",
          "required": ["gps"],
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
                "altitude": {"type": "number"},
                "accuracy": {"type": "number"}
              }
            },
            "description": {"type": "string"},
            "oceanZone": {"type": "string"}
          }
        },
        "timestamp": {
          "type": "string",
          "format": "date-time"
        },
        "collector": {
          "type": "object",
          "properties": {
            "deviceId": {"type": "string"},
            "organizationId": {"type": "string"},
            "vesselId": {"type": "string"},
            "operator": {"type": "string"}
          }
        },
        "method": {
          "type": "string",
          "enum": ["vessel_net", "beach_cleanup", "underwater_collection", "river_trap"]
        }
      }
    },
    "material": {
      "type": "object",
      "required": ["totalWeight", "composition"],
      "properties": {
        "totalWeight": {
          "type": "object",
          "required": ["value", "unit"],
          "properties": {
            "value": {"type": "number", "minimum": 0},
            "unit": {"type": "string", "enum": ["kg", "g", "ton"]}
          }
        },
        "composition": {
          "type": "array",
          "minItems": 1,
          "items": {
            "type": "object",
            "required": ["resinCode", "percentage", "weight"],
            "properties": {
              "resinCode": {
                "type": "string",
                "enum": ["PET", "HDPE", "PVC", "LDPE", "PP", "PS", "OTHER", "1", "2", "3", "4", "5", "6", "7"]
              },
              "percentage": {
                "type": "number",
                "minimum": 0,
                "maximum": 100
              },
              "weight": {
                "type": "object",
                "required": ["value", "unit"],
                "properties": {
                  "value": {"type": "number"},
                  "unit": {"type": "string"}
                }
              },
              "condition": {
                "type": "string",
                "enum": ["good", "contaminated", "degraded", "microplastic"]
              }
            }
          }
        },
        "contaminationLevel": {
          "type": "string",
          "enum": ["none", "low", "medium", "high"]
        },
        "estimatedAge": {
          "type": "string",
          "enum": ["0-1_years", "1-5_years", "5-10_years", "10+_years", "unknown"]
        }
      }
    },
    "iot": {
      "type": "object",
      "properties": {
        "sensors": {"type": "array"},
        "tracking": {"type": "object"},
        "battery": {"type": "object"}
      }
    },
    "recycling": {
      "type": "object",
      "properties": {
        "currentStage": {"type": "string"},
        "facilities": {"type": "array"},
        "history": {"type": "array"}
      }
    },
    "environmental": {
      "type": "object",
      "properties": {
        "carbonFootprint": {"type": "object"},
        "impactMetrics": {"type": "object"}
      }
    }
  }
}
```

### 4.2 IoT Sensor Schema

```json
{
  "iot": {
    "sensors": [
      {
        "sensorId": "SENSOR-001",
        "type": "gps",
        "data": {
          "latitude": 37.5665,
          "longitude": 126.9780,
          "speed": 0,
          "heading": 0
        },
        "timestamp": "2025-01-15T10:30:00Z",
        "accuracy": 5.0,
        "status": "active"
      },
      {
        "sensorId": "SENSOR-002",
        "type": "environmental",
        "data": {
          "waterTemperature": 15.5,
          "salinity": 35.0,
          "pH": 8.1,
          "turbidity": 2.5
        },
        "timestamp": "2025-01-15T10:30:00Z",
        "unit": "celsius",
        "status": "active"
      },
      {
        "sensorId": "SENSOR-003",
        "type": "weight",
        "data": {
          "currentWeight": 45.5,
          "maxCapacity": 100.0,
          "percentFull": 45.5
        },
        "timestamp": "2025-01-15T10:30:00Z",
        "unit": "kg",
        "status": "active"
      }
    ],
    "tracking": {
      "deviceId": "IOT-DEV-001",
      "lastPing": "2025-01-15T10:30:00Z",
      "signalStrength": -65,
      "networkType": "4G",
      "firmware": "v2.1.0"
    },
    "battery": {
      "level": 85,
      "voltage": 12.5,
      "estimatedHours": 48,
      "chargingStatus": "not_charging"
    }
  }
}
```

### 4.3 Recycling Chain Schema

```json
{
  "recycling": {
    "currentStage": "sorting",
    "facilities": [
      {
        "facilityId": "FAC-SORT-001",
        "name": "Seoul Plastic Sorting Center",
        "type": "sorting",
        "location": {
          "latitude": 37.5665,
          "longitude": 126.9780,
          "address": "123 Recycling St, Seoul, Korea"
        },
        "arrivalTime": "2025-01-16T08:00:00Z",
        "departureTime": null,
        "certifications": ["ISO-14001", "GREEN-CERT-KR"]
      }
    ],
    "history": [
      {
        "stage": "collected",
        "timestamp": "2025-01-15T10:30:00Z",
        "location": {
          "latitude": 37.5665,
          "longitude": 126.9780
        },
        "notes": "Collected from East Sea",
        "verifiedBy": "ORG-CLEANUP-001"
      },
      {
        "stage": "transported",
        "timestamp": "2025-01-16T06:00:00Z",
        "location": {
          "latitude": 37.5665,
          "longitude": 126.9780
        },
        "transportMethod": "truck",
        "distance": {"value": 50, "unit": "km"},
        "verifiedBy": "TRANS-001"
      }
    ],
    "expectedCompletion": "2025-01-20T00:00:00Z",
    "recyclabilityScore": 0.85
  }
}
```

### 4.4 Environmental Impact Schema

```json
{
  "environmental": {
    "carbonFootprint": {
      "collection": {
        "value": 5.2,
        "unit": "kg_co2"
      },
      "transportation": {
        "value": 3.8,
        "unit": "kg_co2"
      },
      "processing": {
        "value": 2.1,
        "unit": "kg_co2"
      },
      "total": {
        "value": 11.1,
        "unit": "kg_co2"
      },
      "offsetBy": "recycling",
      "netImpact": {
        "value": -35.5,
        "unit": "kg_co2"
      }
    },
    "impactMetrics": {
      "marineLifeSaved": {
        "estimatedAnimals": 15,
        "speciesTypes": ["fish", "turtle", "seabird"],
        "calculationMethod": "statistical_model_v1"
      },
      "oceanCleanupArea": {
        "value": 1000,
        "unit": "m2"
      },
      "recycledMaterial": {
        "value": 38.7,
        "unit": "kg",
        "percentage": 85
      },
      "virginPlasticAvoided": {
        "value": 38.7,
        "unit": "kg"
      }
    }
  }
}
```

---

## Field Specifications

### 5.1 Collection Fields

| Field | Type | Required | Description | Example |
|-------|------|----------|-------------|---------|
| `collection.eventId` | string | REQUIRED | Unique collection event ID | `"COLLECT-2025-001"` |
| `collection.location.gps.latitude` | number | REQUIRED | GPS latitude (-90 to 90) | `37.5665` |
| `collection.location.gps.longitude` | number | REQUIRED | GPS longitude (-180 to 180) | `126.9780` |
| `collection.timestamp` | string | REQUIRED | Collection timestamp | `"2025-01-15T10:30:00Z"` |
| `collection.method` | string | REQUIRED | Collection method | `"vessel_net"` |

**Valid collection methods:**

| Value | Description |
|-------|-------------|
| `vessel_net` | Collection by boat using nets |
| `beach_cleanup` | Manual beach collection |
| `underwater_collection` | Diving/ROV collection |
| `river_trap` | River barrier/trap system |

### 5.2 Resin Codes (ASTM D7611)

| Code | Name | Common Uses | Recyclability |
|------|------|-------------|---------------|
| `PET` (1) | Polyethylene Terephthalate | Bottles, containers | High |
| `HDPE` (2) | High-Density Polyethylene | Milk jugs, detergent | High |
| `PVC` (3) | Polyvinyl Chloride | Pipes, packaging | Low |
| `LDPE` (4) | Low-Density Polyethylene | Bags, wraps | Medium |
| `PP` (5) | Polypropylene | Containers, caps | High |
| `PS` (6) | Polystyrene | Foam, cups | Low |
| `OTHER` (7) | Other plastics | Mixed materials | Variable |

### 5.3 IoT Sensor Types

| Type | Measurements | Update Frequency | Purpose |
|------|--------------|------------------|---------|
| `gps` | Lat, lon, speed, heading | 1-60 seconds | Real-time tracking |
| `environmental` | Temperature, pH, salinity | 5-300 seconds | Water quality |
| `weight` | Current weight, capacity | On-change | Load monitoring |
| `camera` | Images, video | On-demand | Documentation |

---

## Data Types

### 6.1 Custom Types

```typescript
type PlasticStatus =
  | 'collected'
  | 'transported'
  | 'sorting'
  | 'processing'
  | 'recycled'
  | 'disposed';

type ResinCode =
  | 'PET' | '1'
  | 'HDPE' | '2'
  | 'PVC' | '3'
  | 'LDPE' | '4'
  | 'PP' | '5'
  | 'PS' | '6'
  | 'OTHER' | '7';

type CollectionMethod =
  | 'vessel_net'
  | 'beach_cleanup'
  | 'underwater_collection'
  | 'river_trap';

type ContaminationLevel =
  | 'none'
  | 'low'
  | 'medium'
  | 'high';

interface GPSCoordinate {
  latitude: number;    // -90 to 90
  longitude: number;   // -180 to 180
  altitude?: number;   // meters
  accuracy?: number;   // meters
}

interface Weight {
  value: number;
  unit: 'kg' | 'g' | 'ton';
}
```

### 6.2 Enum Values

#### Ocean Zones

| Code | Region | Description |
|------|--------|-------------|
| `EEZ-KR-EAST` | Korea East Sea | Exclusive Economic Zone |
| `EEZ-KR-WEST` | Korea West Sea | Yellow Sea region |
| `EEZ-KR-SOUTH` | Korea South Sea | Korean Strait |
| `INTERNATIONAL` | International Waters | High seas |
| `COASTAL` | Coastal Waters | Within 12 nautical miles |

---

## Validation Rules

### 7.1 Required Field Validation

| Rule ID | Field | Validation |
|---------|-------|------------|
| VAL-001 | `batchId` | Must match `^BATCH-\d{4}-\d{6}$` |
| VAL-002 | `collection.location.gps` | Latitude: -90 to 90, Longitude: -180 to 180 |
| VAL-003 | `material.composition` | Array must not be empty |
| VAL-004 | `material.composition[].percentage` | Sum must equal 100% |
| VAL-005 | `status` | Must be valid enum value |

### 7.2 Business Logic Validation

| Rule ID | Description | Error Code |
|---------|-------------|------------|
| BUS-001 | GPS coordinates must be in ocean/coastal area | `ERR_INVALID_LOCATION` |
| BUS-002 | Total weight must match sum of composition weights | `ERR_WEIGHT_MISMATCH` |
| BUS-003 | Collection timestamp cannot be in future | `ERR_FUTURE_TIMESTAMP` |
| BUS-004 | IoT device must be registered and active | `ERR_INVALID_DEVICE` |
| BUS-005 | Recycling facilities must be certified | `ERR_UNCERTIFIED_FACILITY` |

### 7.3 Error Codes

| Code | Message | Description |
|------|---------|-------------|
| `ERR_INVALID_BATCH` | Invalid batch format | Batch ID format violation |
| `ERR_INVALID_LOCATION` | Invalid GPS coordinates | Location validation failed |
| `ERR_WEIGHT_MISMATCH` | Weight calculation mismatch | Total weight doesn't match sum |
| `ERR_INVALID_RESIN` | Invalid resin code | Unknown plastic type |
| `ERR_TRACKING_FAILED` | IoT tracking failed | Sensor data unavailable |

---

## Examples

### 8.1 Valid Plastic Tracking Record - Beach Cleanup

```json
{
  "$schema": "https://wia.live/ocean-plastic-track/v1/schema.json",
  "version": "1.0.0",
  "batchId": "BATCH-2025-000001",
  "status": "collected",
  "created": "2025-01-15T10:30:00Z",
  "lastUpdated": "2025-01-15T10:30:00Z",
  "collection": {
    "eventId": "COLLECT-2025-001",
    "location": {
      "gps": {
        "latitude": 37.5665,
        "longitude": 126.9780,
        "altitude": 2,
        "accuracy": 3.5
      },
      "description": "Haeundae Beach, Busan",
      "oceanZone": "COASTAL"
    },
    "timestamp": "2025-01-15T10:30:00Z",
    "collector": {
      "deviceId": "IOT-MOBILE-001",
      "organizationId": "ORG-CLEANUP-BUSAN",
      "operator": "volunteer-group-5"
    },
    "method": "beach_cleanup"
  },
  "material": {
    "totalWeight": {
      "value": 25.8,
      "unit": "kg"
    },
    "composition": [
      {
        "resinCode": "PET",
        "percentage": 50,
        "weight": {"value": 12.9, "unit": "kg"},
        "condition": "good"
      },
      {
        "resinCode": "PP",
        "percentage": 30,
        "weight": {"value": 7.74, "unit": "kg"},
        "condition": "good"
      },
      {
        "resinCode": "OTHER",
        "percentage": 20,
        "weight": {"value": 5.16, "unit": "kg"},
        "condition": "contaminated"
      }
    ],
    "contaminationLevel": "low",
    "estimatedAge": "1-5_years"
  },
  "environmental": {
    "carbonFootprint": {
      "collection": {"value": 2.1, "unit": "kg_co2"},
      "transportation": {"value": 1.5, "unit": "kg_co2"},
      "total": {"value": 3.6, "unit": "kg_co2"},
      "netImpact": {"value": -20.5, "unit": "kg_co2"}
    },
    "impactMetrics": {
      "marineLifeSaved": {
        "estimatedAnimals": 8,
        "speciesTypes": ["seabird", "crab"],
        "calculationMethod": "statistical_model_v1"
      },
      "oceanCleanupArea": {"value": 500, "unit": "m2"},
      "recycledMaterial": {"value": 20.64, "unit": "kg", "percentage": 80}
    }
  }
}
```

### 8.2 Valid Record - Vessel Ocean Collection with IoT

```json
{
  "$schema": "https://wia.live/ocean-plastic-track/v1/schema.json",
  "version": "1.0.0",
  "batchId": "BATCH-2025-000042",
  "status": "transported",
  "created": "2025-01-15T08:00:00Z",
  "lastUpdated": "2025-01-15T14:30:00Z",
  "collection": {
    "eventId": "COLLECT-2025-042",
    "location": {
      "gps": {
        "latitude": 36.5,
        "longitude": 130.2,
        "altitude": 0,
        "accuracy": 5.0
      },
      "description": "East Sea, 50km from Ulleungdo",
      "oceanZone": "EEZ-KR-EAST"
    },
    "timestamp": "2025-01-15T08:00:00Z",
    "collector": {
      "deviceId": "IOT-VESSEL-007",
      "organizationId": "ORG-OCEAN-CLEANUP-KR",
      "vesselId": "VESSEL-CLEANUP-3",
      "operator": "captain-kim"
    },
    "method": "vessel_net"
  },
  "material": {
    "totalWeight": {
      "value": 150.5,
      "unit": "kg"
    },
    "composition": [
      {
        "resinCode": "HDPE",
        "percentage": 45,
        "weight": {"value": 67.725, "unit": "kg"},
        "condition": "degraded"
      },
      {
        "resinCode": "PP",
        "percentage": 35,
        "weight": {"value": 52.675, "unit": "kg"},
        "condition": "contaminated"
      },
      {
        "resinCode": "PET",
        "percentage": 15,
        "weight": {"value": 22.575, "unit": "kg"},
        "condition": "good"
      },
      {
        "resinCode": "OTHER",
        "percentage": 5,
        "weight": {"value": 7.525, "unit": "kg"},
        "condition": "microplastic"
      }
    ],
    "contaminationLevel": "medium",
    "estimatedAge": "5-10_years"
  },
  "iot": {
    "sensors": [
      {
        "sensorId": "GPS-007-A",
        "type": "gps",
        "data": {
          "latitude": 37.2,
          "longitude": 129.5,
          "speed": 15,
          "heading": 180
        },
        "timestamp": "2025-01-15T14:30:00Z",
        "accuracy": 3.0,
        "status": "active"
      },
      {
        "sensorId": "ENV-007-B",
        "type": "environmental",
        "data": {
          "waterTemperature": 12.5,
          "salinity": 34.5,
          "pH": 8.0
        },
        "timestamp": "2025-01-15T14:30:00Z",
        "status": "active"
      },
      {
        "sensorId": "WEIGHT-007-C",
        "type": "weight",
        "data": {
          "currentWeight": 150.5,
          "maxCapacity": 500.0,
          "percentFull": 30.1
        },
        "timestamp": "2025-01-15T14:30:00Z",
        "unit": "kg",
        "status": "active"
      }
    ],
    "tracking": {
      "deviceId": "IOT-VESSEL-007",
      "lastPing": "2025-01-15T14:30:00Z",
      "signalStrength": -70,
      "networkType": "satellite",
      "firmware": "v3.2.1"
    },
    "battery": {
      "level": 65,
      "voltage": 24.5,
      "estimatedHours": 72,
      "chargingStatus": "solar_charging"
    }
  },
  "recycling": {
    "currentStage": "transported",
    "facilities": [
      {
        "facilityId": "FAC-PORT-POHANG",
        "name": "Pohang Receiving Center",
        "type": "receiving",
        "location": {
          "latitude": 36.0190,
          "longitude": 129.3435,
          "address": "Pohang Port, Korea"
        },
        "arrivalTime": null,
        "expectedArrival": "2025-01-15T18:00:00Z"
      }
    ],
    "history": [
      {
        "stage": "collected",
        "timestamp": "2025-01-15T08:00:00Z",
        "location": {"latitude": 36.5, "longitude": 130.2},
        "notes": "Fishing net debris collected",
        "verifiedBy": "ORG-OCEAN-CLEANUP-KR"
      }
    ]
  },
  "environmental": {
    "carbonFootprint": {
      "collection": {"value": 45.0, "unit": "kg_co2"},
      "transportation": {"value": 12.0, "unit": "kg_co2"},
      "total": {"value": 57.0, "unit": "kg_co2"},
      "netImpact": {"value": -120.0, "unit": "kg_co2"}
    },
    "impactMetrics": {
      "marineLifeSaved": {
        "estimatedAnimals": 45,
        "speciesTypes": ["fish", "turtle", "whale"],
        "calculationMethod": "statistical_model_v1"
      },
      "oceanCleanupArea": {"value": 5000, "unit": "m2"}
    }
  }
}
```

### 8.3 Invalid Example - Missing Required GPS Data

```json
{
  "version": "1.0.0",
  "batchId": "BATCH-2025-000002",
  "status": "collected",
  "collection": {
    "eventId": "COLLECT-2025-002",
    "location": {
      "description": "Somewhere in ocean"
    },
    "timestamp": "2025-01-15T10:30:00Z"
  }
}
```

**Error**: `ERR_VALIDATION_FAILED` - GPS coordinates are required

### 8.4 Invalid Example - Weight Mismatch

```json
{
  "version": "1.0.0",
  "batchId": "BATCH-2025-000003",
  "status": "collected",
  "material": {
    "totalWeight": {"value": 100, "unit": "kg"},
    "composition": [
      {
        "resinCode": "PET",
        "percentage": 50,
        "weight": {"value": 30, "unit": "kg"}
      },
      {
        "resinCode": "HDPE",
        "percentage": 50,
        "weight": {"value": 30, "unit": "kg"}
      }
    ]
  }
}
```

**Error**: `ERR_WEIGHT_MISMATCH` - Total weight (100kg) doesn't match sum (60kg)

### 8.5 Invalid Example - Percentage Sum Error

```json
{
  "material": {
    "composition": [
      {"resinCode": "PET", "percentage": 50},
      {"resinCode": "HDPE", "percentage": 30}
    ]
  }
}
```

**Error**: `ERR_VALIDATION_FAILED` - Composition percentages must sum to 100%

### 8.6 Valid Example - Microplastic Collection

```json
{
  "$schema": "https://wia.live/ocean-plastic-track/v1/schema.json",
  "version": "1.0.0",
  "batchId": "BATCH-2025-000088",
  "status": "collected",
  "created": "2025-01-15T12:00:00Z",
  "collection": {
    "eventId": "COLLECT-2025-088",
    "location": {
      "gps": {
        "latitude": 35.1796,
        "longitude": 129.0756,
        "altitude": 0,
        "accuracy": 2.0
      },
      "description": "Gwangalli Beach, Busan - Microplastic sampling",
      "oceanZone": "COASTAL"
    },
    "timestamp": "2025-01-15T12:00:00Z",
    "collector": {
      "deviceId": "IOT-SAMPLER-003",
      "organizationId": "ORG-RESEARCH-PUSAN",
      "operator": "research-team-marine"
    },
    "method": "beach_cleanup"
  },
  "material": {
    "totalWeight": {
      "value": 0.350,
      "unit": "kg"
    },
    "composition": [
      {
        "resinCode": "OTHER",
        "percentage": 100,
        "weight": {"value": 0.350, "unit": "kg"},
        "condition": "microplastic"
      }
    ],
    "contaminationLevel": "high",
    "estimatedAge": "10+_years"
  },
  "environmental": {
    "impactMetrics": {
      "marineLifeSaved": {
        "estimatedAnimals": 100,
        "speciesTypes": ["plankton", "small_fish"],
        "calculationMethod": "microplastic_impact_model_v1"
      },
      "oceanCleanupArea": {"value": 10, "unit": "m2"}
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

**WIA Ocean Plastic Track Standard v1.0.0**

**弘益人間 (홍익인간)** - Benefit All Humanity

---

**© 2025 WIA**

**MIT License**

</div>
