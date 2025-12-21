# WIA Battery Passport Data Format Standard
## Phase 1 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #22C55E (Green)

---

## Table of Contents

1. [Overview](#overview)
2. [Terminology](#terminology)
3. [Base Structure](#base-structure)
4. [Data Schema](#data-schema)
5. [Field Specifications](#field-specifications)
6. [Battery Chemistry](#battery-chemistry)
7. [Lifecycle Tracking](#lifecycle-tracking)
8. [Validation Rules](#validation-rules)
9. [Examples](#examples)
10. [Version History](#version-history)

---

## Overview

### 1.1 Purpose

The WIA Battery Passport Data Format Standard defines a unified digital framework for tracking battery lifecycle data, from manufacturing through end-of-life. This standard enables complete transparency in battery supply chains, sustainability metrics, and circular economy integration, ensuring compliance with EU Battery Regulation and global environmental standards.

**Core Objectives**:
- Enable complete battery traceability from raw materials to recycling
- Standardize battery identity, chemistry, and performance data
- Support carbon footprint tracking and sustainability reporting
- Facilitate second-life applications and recycling processes
- Ensure EU Battery Regulation compliance
- Enable digital passport access via QR codes and NFC

### 1.2 Scope

This standard covers:

| Domain | Description |
|--------|-------------|
| Battery Identity | Unique identification, manufacturer data, model specifications |
| Chemistry Composition | Materials, cell chemistry, sourcing transparency |
| Lifecycle Data | Manufacturing, usage, state of health (SoH), aging |
| Carbon Footprint | Cradle-to-gate emissions, supply chain impact |
| Supply Chain | Material sourcing, ethical mining, traceability |
| Performance Metrics | Capacity, voltage, energy density, cycle life |
| Second-Life Applications | Repurposing, energy storage systems |
| Recycling Data | Material recovery, circular economy integration |

### 1.3 Design Principles

1. **Transparency**: Complete supply chain visibility from mining to recycling
2. **Sustainability**: Carbon footprint tracking and environmental impact assessment
3. **Traceability**: Immutable record of battery lifecycle events
4. **Accessibility**: QR code and NFC-enabled digital passport access
5. **Interoperability**: Compatible with EU Battery Regulation, GBA standards
6. **Privacy**: Protect sensitive manufacturing data while ensuring compliance

---

## Terminology

### 2.1 Core Terms

| Term | Definition |
|------|------------|
| **Battery Passport** | Digital record of battery identity, composition, and lifecycle |
| **State of Health (SoH)** | Battery's current capacity as percentage of original capacity |
| **State of Charge (SoC)** | Current charge level as percentage of total capacity |
| **Cycle Life** | Number of charge-discharge cycles before degradation |
| **Carbon Footprint** | Total CO2e emissions from production and use |
| **Second-Life** | Battery repurposing after primary application |
| **Material Traceability** | Tracking of raw materials from source to product |
| **Circular Economy** | Closed-loop system for material recovery and reuse |

### 2.2 Data Types

| Type | Description | Example |
|------|-------------|---------|
| `battery_id` | Unique battery identifier | `"BAT-2025-LI-001234"` |
| `chemistry_code` | Battery chemistry type | `"NMC811"` |
| `soh_percentage` | State of health (0-100%) | `95.5` |
| `carbon_co2e` | CO2 equivalent in kg | `125.8` |
| `gps_coordinate` | Geographic coordinates | `{"lat": 37.5665, "lng": 126.9780}` |

### 2.3 Field Requirements

| Marker | Meaning |
|--------|---------|
| **REQUIRED** | Must be present |
| **OPTIONAL** | May be omitted |
| **CONDITIONAL** | Required under specific conditions |

---

## Base Structure

### 3.1 Battery Passport Record Format

```json
{
  "$schema": "https://wia.live/battery-passport/v1/schema.json",
  "version": "1.0.0",
  "passportId": "BP-2025-000001",
  "batteryId": "BAT-2025-LI-001234",
  "created": "2025-01-15T10:00:00Z",
  "updated": "2025-01-15T10:00:00Z",
  "identity": {
    "manufacturer": "GreenCell Technologies",
    "manufacturerId": "MFG-2025-001",
    "model": "GC-NMC-100kWh",
    "serialNumber": "SN-2025-001234",
    "manufacturingDate": "2025-01-10",
    "manufacturingLocation": {
      "country": "KR",
      "city": "Seoul",
      "facility": "Facility-A",
      "coordinates": {
        "lat": 37.5665,
        "lng": 126.9780
      }
    }
  },
  "chemistry": {
    "type": "lithium-ion",
    "cathode": "NMC811",
    "anode": "graphite",
    "electrolyte": "liquid",
    "materials": [],
    "weight": {}
  },
  "specifications": {
    "nominalVoltage": 400,
    "nominalCapacity": 250,
    "energyDensity": 260,
    "powerDensity": 1800,
    "expectedCycleLife": 3000
  },
  "lifecycle": {
    "status": "in_use",
    "currentOwner": "USER-2025-001",
    "installationDate": "2025-01-12",
    "application": "electric_vehicle",
    "stateOfHealth": 100.0,
    "cycleCount": 0,
    "events": []
  },
  "carbonFootprint": {
    "totalCO2e": 12580,
    "cradleToGate": 12580,
    "productionPhase": {},
    "transportationEmissions": 850,
    "recyclability": 95
  },
  "supplyChain": {
    "materials": [],
    "certifications": [],
    "ethicalSourcing": true
  },
  "digitalAccess": {
    "qrCode": "https://wia.live/passport/BP-2025-000001",
    "nfcTag": "NFC-2025-001234",
    "publicUrl": "https://wia.live/passport/BP-2025-000001"
  },
  "compliance": {
    "euBatteryRegulation": true,
    "regulationVersion": "2023/1542",
    "certifications": [],
    "safetyStandards": []
  },
  "verification": {
    "status": "verified",
    "verifiedBy": "WIA-Verifier-001",
    "verifiedAt": "2025-01-15T10:00:00Z",
    "blockchainHash": "0xabc123...",
    "dataIntegrity": "sha256:..."
  }
}
```

### 3.2 Field Details

#### 3.2.1 `passportId` (REQUIRED)

```
Type: string
Format: BP-YYYY-NNNNNN
Description: Unique identifier for this battery passport
Example: "BP-2025-000001"
```

#### 3.2.2 `batteryId` (REQUIRED)

```
Type: string
Format: BAT-YYYY-CC-NNNNNN (CC = Chemistry Code)
Description: Unique battery identifier
Example: "BAT-2025-LI-001234"
```

---

## Data Schema

### 4.1 Complete JSON Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wia.live/battery-passport/v1/schema.json",
  "title": "WIA Battery Passport Record",
  "type": "object",
  "required": ["version", "passportId", "batteryId", "identity", "chemistry", "specifications"],
  "properties": {
    "version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+\\.\\d+$",
      "description": "Schema version"
    },
    "passportId": {
      "type": "string",
      "pattern": "^BP-\\d{4}-\\d{6}$",
      "description": "Unique passport identifier"
    },
    "batteryId": {
      "type": "string",
      "pattern": "^BAT-\\d{4}-[A-Z]{2}-\\d{6}$",
      "description": "Unique battery identifier"
    },
    "identity": {
      "type": "object",
      "required": ["manufacturer", "model", "serialNumber", "manufacturingDate"],
      "properties": {
        "manufacturer": { "type": "string" },
        "manufacturerId": { "type": "string" },
        "model": { "type": "string" },
        "serialNumber": { "type": "string" },
        "manufacturingDate": { "type": "string", "format": "date" },
        "manufacturingLocation": {
          "type": "object",
          "properties": {
            "country": { "type": "string", "minLength": 2, "maxLength": 2 },
            "city": { "type": "string" },
            "facility": { "type": "string" },
            "coordinates": {
              "type": "object",
              "properties": {
                "lat": { "type": "number", "minimum": -90, "maximum": 90 },
                "lng": { "type": "number", "minimum": -180, "maximum": 180 }
              }
            }
          }
        }
      }
    },
    "chemistry": {
      "type": "object",
      "required": ["type", "cathode", "anode"],
      "properties": {
        "type": {
          "type": "string",
          "enum": ["lithium-ion", "lithium-polymer", "solid-state", "sodium-ion", "lead-acid"]
        },
        "cathode": { "type": "string" },
        "anode": { "type": "string" },
        "electrolyte": { "type": "string" },
        "materials": { "type": "array" },
        "weight": { "type": "object" }
      }
    },
    "specifications": {
      "type": "object",
      "required": ["nominalVoltage", "nominalCapacity"],
      "properties": {
        "nominalVoltage": { "type": "number", "minimum": 0 },
        "nominalCapacity": { "type": "number", "minimum": 0 },
        "energyDensity": { "type": "number", "minimum": 0 },
        "powerDensity": { "type": "number", "minimum": 0 },
        "expectedCycleLife": { "type": "integer", "minimum": 0 }
      }
    }
  }
}
```

### 4.2 Battery Chemistry Schema

```json
{
  "chemistry": {
    "type": "lithium-ion",
    "subtype": "NMC811",
    "cathode": "NMC811",
    "anode": "graphite",
    "electrolyte": "liquid",
    "separator": "polyethylene",
    "materials": [
      {
        "materialId": "MAT-2025-0001",
        "name": "Nickel",
        "symbol": "Ni",
        "percentage": 80,
        "weight": 48.0,
        "unit": "kg",
        "source": {
          "country": "AU",
          "mine": "Australian Nickel Mine",
          "supplier": "Nickel Supply Co",
          "certifications": ["RMI", "IRMA"],
          "ethicalSourcing": true
        }
      },
      {
        "materialId": "MAT-2025-0002",
        "name": "Manganese",
        "symbol": "Mn",
        "percentage": 10,
        "weight": 6.0,
        "unit": "kg",
        "source": {
          "country": "ZA",
          "supplier": "SA Minerals Ltd",
          "certifications": ["RMI"],
          "ethicalSourcing": true
        }
      },
      {
        "materialId": "MAT-2025-0003",
        "name": "Cobalt",
        "symbol": "Co",
        "percentage": 10,
        "weight": 6.0,
        "unit": "kg",
        "source": {
          "country": "CD",
          "supplier": "Ethical Cobalt Supply",
          "certifications": ["RMI", "Fair Cobalt Alliance"],
          "ethicalSourcing": true,
          "conflictFree": true
        }
      },
      {
        "materialId": "MAT-2025-0004",
        "name": "Lithium",
        "symbol": "Li",
        "percentage": 7,
        "weight": 4.2,
        "unit": "kg",
        "source": {
          "country": "CL",
          "supplier": "Chilean Lithium Corp",
          "certifications": ["RMI"],
          "ethicalSourcing": true
        }
      }
    ],
    "weight": {
      "total": 250,
      "activeMaterials": 180,
      "packaging": 50,
      "electronics": 20,
      "unit": "kg"
    },
    "recycledContent": {
      "nickel": 15,
      "cobalt": 20,
      "lithium": 10,
      "unit": "percentage"
    }
  }
}
```

### 4.3 Lifecycle Tracking Schema

```json
{
  "lifecycle": {
    "status": "in_use",
    "currentOwner": "USER-2025-001",
    "ownershipHistory": [
      {
        "owner": "MFG-2025-001",
        "ownerType": "manufacturer",
        "startDate": "2025-01-10",
        "endDate": "2025-01-11"
      },
      {
        "owner": "USER-2025-001",
        "ownerType": "end_user",
        "startDate": "2025-01-12",
        "endDate": null
      }
    ],
    "installationDate": "2025-01-12T09:00:00Z",
    "application": "electric_vehicle",
    "vehicleVIN": "VIN123456789ABCDEF",
    "stateOfHealth": 100.0,
    "stateOfCharge": 85.0,
    "cycleCount": 45,
    "totalEnergyThroughput": 11250,
    "totalEnergyThroughputUnit": "kWh",
    "operatingHours": 450,
    "temperatureExposure": {
      "min": -10,
      "max": 45,
      "average": 25,
      "unit": "celsius"
    },
    "events": [
      {
        "eventId": "EVT-2025-0001",
        "type": "manufacturing_complete",
        "timestamp": "2025-01-10T18:00:00Z",
        "description": "Battery manufacturing completed",
        "location": "Seoul, KR"
      },
      {
        "eventId": "EVT-2025-0002",
        "type": "quality_inspection",
        "timestamp": "2025-01-11T10:00:00Z",
        "description": "Quality inspection passed",
        "inspector": "QA-001",
        "result": "passed"
      },
      {
        "eventId": "EVT-2025-0003",
        "type": "installation",
        "timestamp": "2025-01-12T09:00:00Z",
        "description": "Installed in electric vehicle",
        "application": "electric_vehicle",
        "installer": "INST-001"
      },
      {
        "eventId": "EVT-2025-0004",
        "type": "soh_measurement",
        "timestamp": "2025-01-15T10:00:00Z",
        "stateOfHealth": 99.8,
        "cycleCount": 45,
        "capacity": 249.5,
        "capacityUnit": "Ah"
      }
    ],
    "maintenanceHistory": [
      {
        "maintenanceId": "MNT-2025-0001",
        "date": "2025-01-14",
        "type": "diagnostic",
        "description": "Routine diagnostic check",
        "technician": "TECH-001",
        "findings": "All systems normal"
      }
    ],
    "warrantyInfo": {
      "startDate": "2025-01-12",
      "endDate": "2033-01-12",
      "durationYears": 8,
      "coverageType": "full",
      "minSoHGuarantee": 70
    }
  }
}
```

### 4.4 Carbon Footprint Schema

```json
{
  "carbonFootprint": {
    "totalCO2e": 12580,
    "unit": "kg CO2e",
    "calculationMethod": "ISO 14067",
    "calculationDate": "2025-01-15",
    "cradleToGate": 12580,
    "productionPhase": {
      "rawMaterialExtraction": 3800,
      "materialProcessing": 2500,
      "cellManufacturing": 3200,
      "packAssembly": 1500,
      "qualityControl": 180,
      "packaging": 250
    },
    "transportationEmissions": 850,
    "transportationDetails": [
      {
        "from": "Australian Nickel Mine, AU",
        "to": "Seoul, KR",
        "material": "Nickel",
        "distance": 8500,
        "distanceUnit": "km",
        "transportMode": "ship",
        "co2e": 320
      },
      {
        "from": "Chilean Lithium Mine, CL",
        "to": "Seoul, KR",
        "material": "Lithium",
        "distance": 18000,
        "distanceUnit": "km",
        "transportMode": "ship",
        "co2e": 280
      }
    ],
    "usePhaseEstimate": {
      "lifetimeKm": 300000,
      "estimatedUseCO2e": 15000,
      "gridEmissionFactor": 0.459,
      "gridRegion": "KR"
    },
    "endOfLifeRecycling": {
      "recyclingCO2e": -2500,
      "materialRecovery": 95,
      "energyRecovery": 85
    },
    "recyclability": 95,
    "recyclabilityBreakdown": {
      "metals": 98,
      "plastics": 85,
      "electrolyte": 60
    },
    "certifications": [
      "ISO 14067",
      "Carbon Trust Certified"
    ]
  }
}
```

---

## Field Specifications

### 5.1 Battery Identity Fields

| Field | Type | Required | Description | Example |
|-------|------|----------|-------------|---------|
| `manufacturer` | string | REQUIRED | Manufacturer name | `"GreenCell Technologies"` |
| `model` | string | REQUIRED | Battery model | `"GC-NMC-100kWh"` |
| `serialNumber` | string | REQUIRED | Unique serial number | `"SN-2025-001234"` |
| `manufacturingDate` | string | REQUIRED | Date of manufacture | `"2025-01-10"` |
| `manufacturingLocation` | object | OPTIONAL | Manufacturing facility | `{...}` |

### 5.2 Battery Chemistry Fields

| Field | Type | Required | Description | Example |
|-------|------|----------|-------------|---------|
| `type` | string | REQUIRED | Battery chemistry type | `"lithium-ion"` |
| `cathode` | string | REQUIRED | Cathode material | `"NMC811"` |
| `anode` | string | REQUIRED | Anode material | `"graphite"` |
| `materials[]` | array | REQUIRED | Material composition | `[{...}]` |
| `recycledContent` | object | OPTIONAL | Recycled material % | `{...}` |

**Valid Chemistry Types:**

| Type | Description | Common Applications |
|------|-------------|-------------------|
| `lithium-ion` | Li-ion batteries (NMC, NCA, LFP) | EVs, energy storage |
| `lithium-polymer` | Li-Po batteries | Mobile devices, drones |
| `solid-state` | Solid electrolyte batteries | Next-gen EVs |
| `sodium-ion` | Na-ion batteries | Grid storage |
| `lead-acid` | Traditional lead-acid | Automotive starter |

### 5.3 Lifecycle Status Values

| Status | Description | Next States |
|--------|-------------|-------------|
| `manufactured` | Freshly produced | `in_transit`, `in_storage` |
| `in_storage` | Warehouse storage | `in_transit`, `in_use` |
| `in_transit` | Being transported | `in_storage`, `in_use` |
| `in_use` | Active operation | `maintenance`, `second_life`, `end_of_life` |
| `maintenance` | Under maintenance | `in_use`, `second_life` |
| `second_life` | Repurposed application | `maintenance`, `end_of_life` |
| `end_of_life` | Ready for recycling | `recycled` |
| `recycled` | Materials recovered | (terminal state) |

### 5.4 State of Health (SoH) Calculation

```
SoH (%) = (Current Capacity / Original Capacity) × 100

Where:
- Current Capacity: Measured capacity at present
- Original Capacity: Nominal capacity when new
- Range: 0-100%
- Warning threshold: < 80%
- End-of-life threshold: < 70%
```

### 5.5 Carbon Footprint Calculation

```
Total CO2e = Production + Transportation + Use + End-of-Life

Production CO2e = Raw Materials + Processing + Manufacturing
Transportation CO2e = Σ(Distance × Emission Factor × Weight)
Use CO2e = Energy Consumption × Grid Emission Factor
End-of-Life CO2e = Recycling Emissions - Material Recovery Credit
```

---

## Battery Chemistry

### 6.1 Common Cathode Materials

| Code | Name | Formula | Voltage | Energy Density | Applications |
|------|------|---------|---------|---------------|--------------|
| **NMC111** | Nickel Manganese Cobalt | LiNi₀.₃₃Mn₀.₃₃Co₀.₃₃O₂ | 3.7V | 150-200 Wh/kg | Early EVs |
| **NMC532** | NMC 5-3-2 | LiNi₀.₅Mn₀.₃Co₀.₂O₂ | 3.7V | 180-220 Wh/kg | EVs |
| **NMC622** | NMC 6-2-2 | LiNi₀.₆Mn₀.₂Co₀.₂O₂ | 3.7V | 200-230 Wh/kg | EVs |
| **NMC811** | NMC 8-1-1 | LiNi₀.₈Mn₀.₁Co₀.₁O₂ | 3.7V | 220-260 Wh/kg | Modern EVs |
| **NCA** | Nickel Cobalt Aluminum | LiNi₀.₈Co₀.₁₅Al₀.₀₅O₂ | 3.65V | 200-260 Wh/kg | Tesla EVs |
| **LFP** | Lithium Iron Phosphate | LiFePO₄ | 3.2V | 90-160 Wh/kg | Buses, storage |
| **LCO** | Lithium Cobalt Oxide | LiCoO₂ | 3.7V | 150-200 Wh/kg | Smartphones |

### 6.2 Material Sourcing Requirements

| Material | Certification | Traceability Level | Ethical Requirements |
|----------|--------------|-------------------|---------------------|
| **Cobalt** | RMI, Fair Cobalt Alliance | Full chain | Conflict-free, child labor-free |
| **Lithium** | RMI | Mine to manufacturer | Environmental impact assessment |
| **Nickel** | RMI, IRMA | Mine to manufacturer | Responsible mining practices |
| **Graphite** | RMI | Supplier verified | Sustainable sourcing |

---

## Lifecycle Tracking

### 7.1 Lifecycle Event Types

| Event Type | Description | Required Data |
|------------|-------------|---------------|
| `manufacturing_complete` | Battery production finished | Timestamp, location, QA status |
| `quality_inspection` | Quality control check | Inspector, result, metrics |
| `installation` | Installed in application | Application type, installer |
| `soh_measurement` | State of health measured | SoH %, cycle count, capacity |
| `maintenance` | Maintenance performed | Type, technician, findings |
| `relocation` | Moved to new location | From, to, transport mode |
| `second_life_conversion` | Repurposed for second-life | New application, modifications |
| `end_of_life_declaration` | Marked for recycling | Reason, final SoH |
| `recycling_complete` | Materials recovered | Recovery %, materials |

---

## Validation Rules

### 8.1 Required Field Validation

| Rule ID | Field | Validation |
|---------|-------|------------|
| VAL-001 | `passportId` | Must match `^BP-\d{4}-\d{6}$` |
| VAL-002 | `batteryId` | Must match `^BAT-\d{4}-[A-Z]{2}-\d{6}$` |
| VAL-003 | `stateOfHealth` | Must be >= 0 and <= 100 |
| VAL-004 | `nominalVoltage` | Must be > 0 |
| VAL-005 | `manufacturingDate` | Must be valid date, not in future |

### 8.2 Business Logic Validation

| Rule ID | Description | Error Code |
|---------|-------------|------------|
| BUS-001 | Material percentages must sum to 100% | `ERR_MATERIAL_SUM` |
| BUS-002 | SoH must decrease or stay same over time | `ERR_SOH_INCREASE` |
| BUS-003 | Cycle count must only increase | `ERR_CYCLE_DECREASE` |
| BUS-004 | End-of-life requires SoH measurement | `ERR_MISSING_SOH` |
| BUS-005 | Cobalt must have conflict-free certification | `ERR_COBALT_CERT` |

---

## Examples

### 8.1 Valid Battery Passport - Electric Vehicle

```json
{
  "$schema": "https://wia.live/battery-passport/v1/schema.json",
  "version": "1.0.0",
  "passportId": "BP-2025-000001",
  "batteryId": "BAT-2025-LI-001234",
  "created": "2025-01-15T10:00:00Z",
  "updated": "2025-01-15T10:00:00Z",
  "identity": {
    "manufacturer": "GreenCell Technologies",
    "manufacturerId": "MFG-2025-001",
    "model": "GC-NMC811-100kWh",
    "serialNumber": "SN-2025-001234",
    "manufacturingDate": "2025-01-10",
    "manufacturingLocation": {
      "country": "KR",
      "city": "Seoul",
      "facility": "GreenCell Facility A",
      "coordinates": { "lat": 37.5665, "lng": 126.9780 }
    }
  },
  "chemistry": {
    "type": "lithium-ion",
    "subtype": "NMC811",
    "cathode": "NMC811",
    "anode": "graphite",
    "electrolyte": "liquid",
    "separator": "polyethylene",
    "weight": {
      "total": 250,
      "activeMaterials": 180,
      "packaging": 50,
      "electronics": 20,
      "unit": "kg"
    }
  },
  "specifications": {
    "nominalVoltage": 400,
    "nominalVoltageUnit": "V",
    "nominalCapacity": 250,
    "nominalCapacityUnit": "Ah",
    "energyCapacity": 100,
    "energyCapacityUnit": "kWh",
    "energyDensity": 260,
    "energyDensityUnit": "Wh/kg",
    "powerDensity": 1800,
    "powerDensityUnit": "W/kg",
    "expectedCycleLife": 3000,
    "operatingTemperatureMin": -20,
    "operatingTemperatureMax": 55,
    "temperatureUnit": "celsius"
  },
  "lifecycle": {
    "status": "in_use",
    "currentOwner": "USER-2025-001",
    "installationDate": "2025-01-12T09:00:00Z",
    "application": "electric_vehicle",
    "vehicleVIN": "VIN123456789ABCDEF",
    "stateOfHealth": 99.8,
    "stateOfCharge": 85.0,
    "cycleCount": 45,
    "totalEnergyThroughput": 4500,
    "totalEnergyThroughputUnit": "kWh"
  },
  "carbonFootprint": {
    "totalCO2e": 12580,
    "unit": "kg CO2e",
    "cradleToGate": 12580,
    "recyclability": 95
  },
  "digitalAccess": {
    "qrCode": "https://wia.live/passport/BP-2025-000001",
    "nfcTag": "NFC-2025-001234",
    "publicUrl": "https://wia.live/passport/BP-2025-000001"
  },
  "compliance": {
    "euBatteryRegulation": true,
    "regulationVersion": "2023/1542",
    "certifications": ["CE", "UN38.3", "ISO 14001"]
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

**WIA Battery Passport Standard v1.0.0**

**弘益人間 (홍익인간)** - Benefit All Humanity

---

**© 2025 WIA**

**MIT License**

</div>
