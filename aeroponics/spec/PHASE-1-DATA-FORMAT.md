# WIA-AGRI-028: Aeroponics Standard
## Phase 1: Data Format Specification

**Version:** 1.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-01-01

---

## 1. Overview

This specification defines the data formats for aeroponic farming systems, including mist-based nutrient delivery, root zone monitoring, environmental control, and ultra-efficient water management.

### 1.1 Design Principles

- **Precision**: Sub-micron control of nutrient droplet size (5-50 microns)
- **Efficiency**: 98% water efficiency through mist-based delivery
- **Real-time**: Millisecond-level control for misting cycles
- **Scalability**: Support from home gardens to commercial facilities
- **Interoperability**: Compatible with IoT platforms and agricultural systems
- **Traceability**: Complete farm-to-table data lineage with NASA heritage

---

## 2. Core Data Structures

### 2.1 System Configuration Data

Basic information about the aeroponic system.

```json
{
  "systemConfig": {
    "systemId": "AERO-SEOUL-001",
    "systemName": "Seoul AeroFarm",
    "location": {
      "address": "456 Gangnam-daero, Seoul, South Korea",
      "latitude": 37.5665,
      "longitude": 126.9780,
      "timezone": "Asia/Seoul"
    },
    "infrastructure": {
      "totalArea": 500,
      "growingChambers": 8,
      "chamberVolume": 12,
      "totalCapacity": 3000,
      "powerCapacity": 150,
      "waterReservoir": 1000
    },
    "mistingSystem": {
      "systemType": "HIGH_PRESSURE_AEROPONIC",
      "pumpType": "HIGH_PRESSURE",
      "maxPressure": 150,
      "nozzleCount": 192,
      "nozzleType": "MICRO_MISTER",
      "dropletSize": "5-50_MICRONS",
      "mistInterval": 180,
      "mistDuration": 5
    },
    "certifications": ["ORGANIC", "PESTICIDE_FREE", "WATER_EFFICIENT"],
    "operationalSince": "2024-06-01",
    "capacity": {
      "annualProduction": 15000,
      "unit": "kg",
      "simultaneousPlants": 3000
    }
  }
}
```

**Field Descriptions:**
- `systemType`: HIGH_PRESSURE_AEROPONIC (80-150 PSI) or LOW_PRESSURE_AEROPONIC (20-40 PSI)
- `dropletSize`: Optimal range 5-50 microns for maximum nutrient absorption
- `mistInterval`: Seconds between misting cycles (typical: 120-300 seconds)
- `mistDuration`: Spray duration in seconds (typical: 3-10 seconds)

### 2.2 Misting Cycle Data

Real-time data from the misting system.

```json
{
  "mistingData": {
    "systemId": "AERO-SEOUL-001",
    "chamber": 3,
    "timestamp": "2025-01-01T14:30:00Z",
    "cycleStatus": "completed",
    "cycleMetrics": {
      "interval": 180,
      "duration": 5,
      "pressure": 85,
      "volume": 125,
      "dropletSize": 25,
      "nozzlesActive": 24,
      "nozzlesFailed": 0,
      "pumpRPM": 3400,
      "pumpTemp": 42,
      "energyUsed": 0.15
    },
    "timing": {
      "lastCycle": "2025-01-01T14:27:00Z",
      "currentCycle": "2025-01-01T14:30:00Z",
      "nextCycle": "2025-01-01T14:33:00Z",
      "cyclesCompleted": 248,
      "missedCycles": 0
    },
    "performance": {
      "coverageUniformity": 98.5,
      "dropletDistribution": "optimal",
      "rootZoneCoverage": 100,
      "efficiency": 97.8
    }
  }
}
```

**Field Descriptions:**
- `pressure`: PSI reading during mist cycle
- `volume`: Milliliters of nutrient solution used
- `dropletSize`: Measured in microns (optimal: 5-50)
- `coverageUniformity`: Percentage of even distribution

### 2.3 Root Zone Monitoring Data

Specialized sensors for suspended root systems.

```json
{
  "rootZoneData": {
    "systemId": "AERO-SEOUL-001",
    "chamber": 3,
    "timestamp": "2025-01-01T14:30:00Z",
    "environment": {
      "oxygenLevel": 21.5,
      "humidity": 95,
      "temperature": 20.5,
      "airflow": "optimal",
      "co2Level": 450
    },
    "rootMetrics": {
      "rootDensity": "high",
      "rootColor": "white",
      "rootLength": 25,
      "lateralRoots": "abundant",
      "rootHairs": "present",
      "healthScore": 95
    },
    "moisture": {
      "postMist": 100,
      "preMist": 45,
      "optimalRange": "40-100",
      "status": "optimal"
    },
    "issues": []
  }
}
```

**Field Descriptions:**
- `oxygenLevel`: Percentage in root zone (optimal: >20%)
- `rootDensity`: low, medium, high, very_high
- `healthScore`: 0-100 composite score
- `moisture`: Percentage immediately post-mist and pre-mist

### 2.4 Nutrient Solution Data

Precise nutrient delivery system.

```json
{
  "nutrientData": {
    "systemId": "AERO-SEOUL-001",
    "chamber": 3,
    "timestamp": "2025-01-01T14:30:00Z",
    "solution": {
      "ec": 1.8,
      "ph": 6.1,
      "temperature": 20,
      "dissolvedOxygen": 8.5,
      "orp": 350
    },
    "nutrients": {
      "nitrogen": 180,
      "phosphorus": 60,
      "potassium": 220,
      "calcium": 200,
      "magnesium": 60,
      "sulfur": 64,
      "iron": 3.0,
      "manganese": 0.5,
      "zinc": 0.3,
      "copper": 0.1,
      "boron": 0.5,
      "molybdenum": 0.05
    },
    "reservoir": {
      "volume": 850,
      "capacity": 1000,
      "level": 85,
      "temperature": 19.5,
      "agitation": "active",
      "filterStatus": "clean",
      "lastRefill": "2024-12-28T10:00:00Z"
    },
    "delivery": {
      "pumpFlow": 3.2,
      "pumpPressure": 85,
      "filterPressure": 82,
      "pressureDrop": 3,
      "nozzleStatus": "all_operational"
    }
  }
}
```

**Field Descriptions:**
- `ec`: Electrical Conductivity in mS/cm (typical: 1.2-2.4)
- `ph`: Acidity level (optimal: 5.5-6.5)
- `dissolvedOxygen`: mg/L in solution
- `orp`: Oxidation-Reduction Potential in mV
- All nutrient values in ppm (parts per million)

### 2.5 Environmental Control Data

Climate control for optimal growth.

```json
{
  "environmentData": {
    "systemId": "AERO-SEOUL-001",
    "chamber": 3,
    "timestamp": "2025-01-01T14:30:00Z",
    "climate": {
      "airTemperature": 22.5,
      "rootZoneTemp": 20.0,
      "humidity": 70,
      "vpd": 0.85,
      "co2Level": 1100,
      "airflow": "medium",
      "airChanges": 12
    },
    "lighting": {
      "intensity": 450,
      "spectrum": "full_spectrum",
      "red": 660,
      "blue": 450,
      "farRed": 730,
      "white": 5000,
      "dli": 18,
      "photoperiod": "16h_on_8h_off",
      "power": 120
    },
    "hvac": {
      "cooling": "active",
      "heating": "standby",
      "dehumidification": "auto",
      "fanSpeed": 60,
      "filterStatus": "clean"
    }
  }
}
```

**Field Descriptions:**
- `vpd`: Vapor Pressure Deficit in kPa (optimal: 0.8-1.2)
- `airChanges`: Fresh air exchanges per hour
- `dli`: Daily Light Integral in mol/m²/day
- Light wavelengths in nanometers

### 2.6 Crop Growth Data

Plant development tracking.

```json
{
  "cropData": {
    "systemId": "AERO-SEOUL-001",
    "chamber": 3,
    "batchId": "BATCH-2025-001",
    "timestamp": "2025-01-01T14:30:00Z",
    "crop": {
      "type": "basil",
      "variety": "Genovese",
      "seedSource": "Organic Seeds Co.",
      "plantCount": 375
    },
    "timeline": {
      "germinationDate": "2024-12-05T00:00:00Z",
      "transplantDate": "2024-12-12T00:00:00Z",
      "vegetativeStart": "2024-12-15T00:00:00Z",
      "expectedHarvest": "2025-01-05T00:00:00Z",
      "daysToHarvest": 4,
      "totalGrowthDays": 31
    },
    "growth": {
      "stage": "late_vegetative",
      "averageHeight": 18.5,
      "leafCount": 12,
      "stemDiameter": 4.2,
      "leafColor": "deep_green",
      "growthRate": "+45%",
      "healthScore": 96
    },
    "rootDevelopment": {
      "rootMass": "excellent",
      "rootColor": "white",
      "rootLength": 28,
      "rootHairs": "abundant",
      "rootHealth": "pristine"
    }
  }
}
```

**Field Descriptions:**
- `growthRate`: Comparison to traditional soil growing
- `healthScore`: 0-100 composite health rating
- All measurements in centimeters unless specified

### 2.7 Harvest Data

Harvest tracking and quality metrics.

```json
{
  "harvestData": {
    "harvestId": "HRV-AERO-20250101-001",
    "systemId": "AERO-SEOUL-001",
    "chamber": 3,
    "batchId": "BATCH-2025-001",
    "timestamp": "2025-01-01T10:00:00Z",
    "crop": {
      "type": "basil",
      "variety": "Genovese",
      "plantsHarvested": 375,
      "totalWeight": 18.75,
      "unit": "kg",
      "yieldPerPlant": 50
    },
    "quality": {
      "grade": "Premium",
      "leafColor": "deep_green",
      "aroma": "strong",
      "texture": "tender",
      "shelfLife": 14,
      "brixLevel": 8.5
    },
    "rootQuality": {
      "rootHealth": "pristine",
      "rootColor": "pure_white",
      "noDisease": true,
      "noBrownSpots": true
    },
    "metrics": {
      "growthCycle": 31,
      "waterUsed": 75,
      "waterEfficiency": 98,
      "energyUsed": 85,
      "nutrientEfficiency": 95
    },
    "destination": {
      "type": "restaurant",
      "name": "Seoul Organic Bistro",
      "location": "Seoul",
      "deliveryDate": "2025-01-01T14:00:00Z"
    }
  }
}
```

**Field Descriptions:**
- `yieldPerPlant`: Grams per plant
- `shelfLife`: Expected days until degradation
- `brixLevel`: Sugar content (higher = better taste)
- `waterEfficiency`: Percentage efficiency vs traditional farming

---

## 3. Data Exchange Formats

### 3.1 Standard Formats

**Supported Formats:**
- **JSON**: Primary format for APIs and data exchange
- **MessagePack**: Compact binary format for IoT devices
- **Protocol Buffers**: High-performance serialization
- **CSV**: Batch data export for analytics

### 3.2 Timestamp Standard

All timestamps MUST use ISO 8601 format with UTC timezone:
```
2025-01-01T14:30:00Z
```

### 3.3 Unit Standards

- **Temperature**: Celsius (°C)
- **Pressure**: PSI (pounds per square inch)
- **Volume**: Liters (L) or Milliliters (mL)
- **Weight**: Kilograms (kg) or Grams (g)
- **Distance**: Centimeters (cm) or Millimeters (mm)
- **EC**: mS/cm (milliSiemens per centimeter)
- **Droplet Size**: Microns (μm)
- **Light**: PPFD (μmol/m²/s), DLI (mol/m²/day)

---

## 4. Validation Rules

### 4.1 Misting Cycle Validation

```javascript
function validateMistingCycle(data) {
  const rules = {
    interval: { min: 30, max: 600 },      // seconds
    duration: { min: 1, max: 30 },        // seconds
    pressure: { min: 40, max: 150 },      // PSI
    dropletSize: { min: 5, max: 50 }      // microns
  };

  return Object.entries(rules).every(([key, range]) =>
    data[key] >= range.min && data[key] <= range.max
  );
}
```

### 4.2 Nutrient Solution Validation

```javascript
function validateNutrientSolution(data) {
  const rules = {
    ec: { min: 0.5, max: 3.0 },
    ph: { min: 5.0, max: 7.0 },
    temperature: { min: 15, max: 25 },
    dissolvedOxygen: { min: 6, max: 10 }
  };

  return Object.entries(rules).every(([key, range]) =>
    data[key] >= range.min && data[key] <= range.max
  );
}
```

---

## 5. Error Codes

| Code | Description | Severity |
|------|-------------|----------|
| AERO-1001 | Misting pressure too low | Warning |
| AERO-1002 | Droplet size out of range | Warning |
| AERO-1003 | Misting cycle missed | Error |
| AERO-2001 | Root zone humidity critical | Critical |
| AERO-2002 | Root discoloration detected | Warning |
| AERO-3001 | Nutrient EC out of range | Warning |
| AERO-3002 | pH level critical | Critical |
| AERO-3003 | Reservoir level low | Warning |
| AERO-4001 | Nozzle clogged | Error |
| AERO-4002 | Pump failure | Critical |
| AERO-5001 | Temperature extreme | Critical |

---

## 6. Best Practices

1. **Misting Frequency**: Monitor root moisture and adjust cycles dynamically
2. **Droplet Size**: Maintain 5-50 micron range for optimal absorption
3. **Data Logging**: Store all misting cycles for ML optimization
4. **Root Inspection**: Daily visual checks for health monitoring
5. **Nutrient Testing**: Verify EC/pH before every reservoir refill
6. **Backup Systems**: Redundant pumps for critical operations
7. **Clean Nozzles**: Weekly cleaning to prevent clogs
8. **Water Quality**: Use filtered or RO water for best results

---

## 7. Compliance

This specification complies with:
- ISO 8601 (Date and time format)
- IEEE 754 (Floating-point arithmetic)
- RFC 8259 (JSON data interchange)
- WIA-AGRI-001 through WIA-AGRI-027 (Agricultural standards)

---

**Document History:**
- v1.0.0 (2025-01-01): Initial release

**Contributors:**
- WIA Agricultural Standards Committee
- NASA Agricultural Research Division
- International Aeroponic Growers Association

---

© 2025 WIA Standards · MIT License
弘益人間 (Benefit All Humanity)
