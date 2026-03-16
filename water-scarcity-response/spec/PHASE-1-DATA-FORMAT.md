# WIA Water Scarcity Response Data Format Standard
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
6. [Water Source Types](#water-source-types)
7. [Monitoring Metrics](#monitoring-metrics)
8. [Validation Rules](#validation-rules)
9. [Examples](#examples)
10. [Version History](#version-history)

---

## Overview

### 1.1 Purpose

The WIA Water Scarcity Response Data Format Standard defines a unified digital framework for monitoring water resources, predicting droughts, and optimizing water allocation. This standard enables real-time tracking of water levels, consumption patterns, aquifer health, and forecasting water scarcity events to ensure sustainable water management.

**Core Objectives**:
- Enable comprehensive water resource monitoring across all sources
- Standardize drought prediction and early warning systems
- Support AI-driven consumption optimization and allocation
- Facilitate integration with municipal, agricultural, and industrial systems
- Provide real-time data for desalination plant operations
- Ensure transparent water management and conservation tracking

### 1.2 Scope

This standard covers:

| Domain | Description |
|--------|-------------|
| Water Sources | Reservoirs, lakes, rivers, aquifers, groundwater |
| Consumption Data | Municipal, agricultural, industrial usage patterns |
| Climate Metrics | Rainfall, temperature, evaporation, humidity |
| Drought Indicators | Risk scores, forecasts, severity levels |
| Conservation Metrics | Efficiency scores, savings, targets |
| Aquifer Health | Depth, recharge rates, depletion trends |
| Desalination Data | Production capacity, energy usage, costs |
| Quality Metrics | TDS, pH, contamination levels |

### 1.3 Design Principles

1. **Real-Time Monitoring**: Continuous data collection from all water sources
2. **Predictive Analytics**: AI-driven drought forecasting and risk assessment
3. **Optimization**: Smart allocation and consumption management
4. **Transparency**: Open access to water resource data
5. **Interoperability**: Compatible with existing water management systems
6. **Sustainability**: Track conservation efforts and efficiency improvements

---

## Terminology

### 2.1 Core Terms

| Term | Definition |
|------|------------|
| **Water Level** | Current height of water in reservoir or aquifer (meters) |
| **Capacity** | Percentage of total storage capacity currently filled |
| **Consumption Rate** | Volume of water used per unit time (m³/day) |
| **Aquifer** | Underground layer of water-bearing rock |
| **Recharge Rate** | Speed at which aquifer is naturally replenished |
| **Drought Index** | Numerical measure of drought severity (0-100) |
| **TDS** | Total Dissolved Solids - measure of water purity (ppm) |
| **Desalination** | Process of removing salt from seawater |
| **Water Stress** | Demand exceeds available supply |
| **Conservation Target** | Goal for water usage reduction (%) |

### 2.2 Measurement Units

| Metric | Unit | Symbol |
|--------|------|--------|
| Volume | Cubic meters | m³ |
| Flow Rate | Cubic meters per day | m³/day |
| Depth | Meters | m |
| Rainfall | Millimeters | mm |
| Temperature | Celsius | °C |
| Evaporation | Millimeters per day | mm/day |
| Energy | Kilowatt-hours | kWh |
| TDS | Parts per million | ppm |

---

## Base Structure

### 3.1 Water Resource Document

```json
{
  "standard": "WIA-ENE-053",
  "version": "1.0.0",
  "type": "water-scarcity-response",
  "timestamp": "ISO 8601 timestamp",
  "id": "unique resource identifier",
  "waterSource": {},
  "consumption": {},
  "climate": {},
  "drought": {},
  "aquifer": {},
  "quality": {},
  "conservation": {}
}
```

### 3.2 Required Fields

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `standard` | string | ✅ | Always "WIA-ENE-053" |
| `version` | string | ✅ | Semantic version (e.g., "1.0.0") |
| `type` | string | ✅ | Document type identifier |
| `timestamp` | string | ✅ | ISO 8601 format |
| `id` | string | ✅ | Unique resource ID |
| `waterSource` | object | ✅ | Water source details |
| `consumption` | object | ✅ | Consumption metrics |

---

## Data Schema

### 4.1 Water Source Schema

```json
{
  "waterSource": {
    "id": "source-unique-id",
    "name": "Lake Mead",
    "type": "reservoir",
    "location": {
      "latitude": 36.0155,
      "longitude": -114.7443,
      "region": "Nevada/Arizona",
      "country": "USA"
    },
    "currentLevel": {
      "value": 327.5,
      "unit": "meters",
      "timestamp": "2025-01-15T10:00:00Z"
    },
    "capacity": {
      "current": 34,
      "maximum": 100,
      "total": 32236000000,
      "unit": "cubic_meters",
      "percentFull": 34
    },
    "trend": "declining",
    "historicalLow": 320.5,
    "criticalLevel": 328.0
  }
}
```

### 4.2 Consumption Schema

```json
{
  "consumption": {
    "daily": 2500000,
    "weekly": 17500000,
    "monthly": 75000000,
    "unit": "cubic_meters",
    "perCapita": 250,
    "sectors": {
      "residential": {
        "percentage": 45,
        "volume": 1125000,
        "trend": "-2.1%"
      },
      "agricultural": {
        "percentage": 35,
        "volume": 875000,
        "trend": "-1.5%"
      },
      "industrial": {
        "percentage": 20,
        "volume": 500000,
        "trend": "+0.5%"
      }
    }
  }
}
```

### 4.3 Climate Schema

```json
{
  "climate": {
    "rainfall": {
      "current": 450,
      "historical": 680,
      "unit": "millimeters",
      "deficit": -230
    },
    "temperature": {
      "current": 28,
      "average": 22,
      "unit": "celsius"
    },
    "evaporation": {
      "rate": 6.5,
      "unit": "millimeters_per_day"
    },
    "humidity": 35,
    "windSpeed": 12
  }
}
```

### 4.4 Drought Prediction Schema

```json
{
  "drought": {
    "currentStatus": "high_risk",
    "riskScore": 78,
    "severity": "severe",
    "forecast": {
      "30days": {
        "probability": 85,
        "confidence": 92
      },
      "90days": {
        "probability": 73,
        "confidence": 87
      }
    },
    "alerts": [
      {
        "level": "warning",
        "message": "Water levels critically low",
        "timestamp": "2025-01-15T10:00:00Z"
      }
    ],
    "recommendations": [
      "Implement emergency conservation measures",
      "Activate water restriction policies",
      "Increase desalination production"
    ]
  }
}
```

### 4.5 Aquifer Schema

```json
{
  "aquifer": {
    "id": "aquifer-sw-001",
    "name": "Southwest Basin",
    "depth": {
      "current": 85.2,
      "historical": 45.0,
      "unit": "meters"
    },
    "rechargeRate": {
      "current": 1800000,
      "required": 2500000,
      "unit": "cubic_meters_per_day",
      "deficit": -700000
    },
    "quality": {
      "TDS": 485,
      "pH": 7.2,
      "contamination": "low"
    },
    "status": "declining",
    "sustainabilityIndex": 42
  }
}
```

### 4.6 Conservation Schema

```json
{
  "conservation": {
    "targetReduction": 15,
    "currentReduction": 8.5,
    "unit": "percentage",
    "dailySavings": 212500,
    "annualSavings": 77562500,
    "initiatives": [
      {
        "name": "Smart Metering Program",
        "status": "active",
        "savings": 50000
      },
      {
        "name": "Rainwater Harvesting",
        "status": "active",
        "savings": 30000
      }
    ],
    "publicEngagement": {
      "participants": 125000,
      "complianceRate": 87
    }
  }
}
```

---

## Field Specifications

### 5.1 Water Source Types

```typescript
enum WaterSourceType {
  RESERVOIR = "reservoir",
  LAKE = "lake",
  RIVER = "river",
  AQUIFER = "aquifer",
  GROUNDWATER = "groundwater",
  DESALINATION = "desalination",
  RECLAIMED = "reclaimed",
  IMPORTED = "imported"
}
```

### 5.2 Drought Severity Levels

```typescript
enum DroughtSeverity {
  NORMAL = "normal",           // 0-20
  MILD = "mild",               // 21-40
  MODERATE = "moderate",       // 41-60
  SEVERE = "severe",           // 61-80
  EXTREME = "extreme",         // 81-100
  EXCEPTIONAL = "exceptional"  // > 100
}
```

### 5.3 Trend Indicators

```typescript
enum Trend {
  DECLINING = "declining",
  STABLE = "stable",
  IMPROVING = "improving",
  CRITICAL = "critical"
}
```

---

## Monitoring Metrics

### 6.1 Key Performance Indicators

```json
{
  "kpi": {
    "waterSecurityIndex": 65,
    "sustainabilityScore": 72,
    "efficiencyRating": 88,
    "droughtPreparedness": 54,
    "conservationProgress": 57,
    "infrastructureHealth": 81
  }
}
```

### 6.2 Alert Thresholds

```json
{
  "thresholds": {
    "waterLevel": {
      "critical": 328.0,
      "warning": 335.0,
      "normal": 350.0
    },
    "droughtRisk": {
      "critical": 80,
      "warning": 60,
      "normal": 40
    },
    "consumption": {
      "excessive": 3000000,
      "normal": 2500000,
      "efficient": 2000000
    }
  }
}
```

---

## Validation Rules

### 7.1 Data Validation

- Water level must be positive number
- Capacity percentage: 0-100
- Consumption values must be non-negative
- Temperature in valid range: -50 to 60°C
- Rainfall: 0-2000mm
- TDS: 0-5000 ppm
- pH: 0-14
- Timestamps must be ISO 8601 format
- Coordinates must be valid lat/lon

### 7.2 Business Rules

- Consumption cannot exceed capacity
- Recharge deficit triggers alerts
- Drought risk auto-calculated from climate data
- Conservation targets must be realistic (0-50%)
- Alert levels cascade appropriately

---

## Examples

### 8.1 Complete Water Resource Document

```json
{
  "standard": "WIA-ENE-053",
  "version": "1.0.0",
  "type": "water-scarcity-response",
  "timestamp": "2025-01-15T10:00:00Z",
  "id": "resource-lakemead-2025",
  "waterSource": {
    "id": "source-lakemead",
    "name": "Lake Mead",
    "type": "reservoir",
    "location": {
      "latitude": 36.0155,
      "longitude": -114.7443,
      "region": "Nevada/Arizona",
      "country": "USA"
    },
    "currentLevel": {
      "value": 327.5,
      "unit": "meters",
      "timestamp": "2025-01-15T10:00:00Z"
    },
    "capacity": {
      "current": 34,
      "maximum": 100,
      "total": 32236000000,
      "unit": "cubic_meters",
      "percentFull": 34
    },
    "trend": "declining"
  },
  "consumption": {
    "daily": 2500000,
    "unit": "cubic_meters",
    "perCapita": 250,
    "sectors": {
      "residential": { "percentage": 45, "volume": 1125000 },
      "agricultural": { "percentage": 35, "volume": 875000 },
      "industrial": { "percentage": 20, "volume": 500000 }
    }
  },
  "climate": {
    "rainfall": { "current": 450, "historical": 680, "unit": "millimeters" },
    "temperature": { "current": 28, "average": 22, "unit": "celsius" },
    "evaporation": { "rate": 6.5, "unit": "millimeters_per_day" }
  },
  "drought": {
    "currentStatus": "high_risk",
    "riskScore": 78,
    "severity": "severe",
    "forecast": {
      "30days": { "probability": 85, "confidence": 92 }
    }
  },
  "aquifer": {
    "depth": { "current": 85.2, "unit": "meters" },
    "rechargeRate": { "current": 1800000, "unit": "cubic_meters_per_day" }
  },
  "conservation": {
    "targetReduction": 15,
    "currentReduction": 8.5,
    "dailySavings": 212500
  }
}
```

---

## Version History

### Version 1.0.0 (2025-01)
- Initial release
- Core data schemas for water monitoring
- Drought prediction framework
- Conservation tracking
- Aquifer health metrics
- Desalination integration

---

**Document Control**
© 2025 WIA Standards Committee
License: MIT
Contact: standards@wia.org
弘益人間 · Benefit All Humanity
