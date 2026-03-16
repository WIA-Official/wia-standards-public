# WIA-ENE-050: Direct Air Capture Standard
## PHASE 2: DATA FORMATS

**Version:** 1.0
**Status:** Draft
**Date:** 2025-12-25
**Category:** Energy (ENE)

---

## 1. Overview

This document specifies the data formats for the WIA-ENE-050 Direct Air Capture standard. It defines schemas, validation rules, and examples for all data structures used in DAC systems.

### 1.1 Design Principles

- **Structured:** JSON/JSON-LD for machine-readable data
- **Extensible:** Support for custom fields and future enhancements
- **Validated:** JSON Schema validation for all formats
- **Versioned:** Semantic versioning for schema evolution
- **Linked Data:** Support for semantic web (RDF, JSON-LD)

### 1.2 Data Categories

1. **Facility Data:** Static and semi-static facility information
2. **Operational Data:** Real-time capture and operational metrics
3. **Sorbent Data:** Material properties and lifecycle tracking
4. **Energy Data:** Energy consumption and source information
5. **Storage Data:** CO2 storage and sequestration records
6. **Verification Data:** MRV and certification information
7. **Carbon Credit Data:** Registry and trading information

---

## 2. Facility Data Format

### 2.1 Facility Information Schema

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "title": "DAC Facility Information",
  "type": "object",
  "required": ["facilityId", "name", "location", "technology", "capacity"],
  "properties": {
    "facilityId": {
      "type": "string",
      "pattern": "^dac-[a-z0-9-]+$",
      "description": "Unique facility identifier"
    },
    "name": {
      "type": "string",
      "minLength": 1,
      "maxLength": 200
    },
    "location": {
      "type": "object",
      "required": ["latitude", "longitude", "country"],
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
        "altitude": {
          "type": "number",
          "description": "Meters above sea level"
        },
        "country": {
          "type": "string",
          "pattern": "^[A-Z]{2}$",
          "description": "ISO 3166-1 alpha-2 country code"
        },
        "region": {
          "type": "string"
        },
        "address": {
          "type": "string"
        }
      }
    },
    "technology": {
      "type": "object",
      "required": ["type"],
      "properties": {
        "type": {
          "type": "string",
          "enum": [
            "solid-sorbent-dac",
            "liquid-solvent-dac",
            "electrochemical-dac",
            "hybrid-dac"
          ]
        },
        "sorbent": {
          "type": "string",
          "description": "Sorbent material type"
        },
        "manufacturer": {
          "type": "string"
        },
        "model": {
          "type": "string"
        },
        "generation": {
          "type": "string"
        }
      }
    },
    "capacity": {
      "type": "object",
      "required": ["annualTonsCO2"],
      "properties": {
        "annualTonsCO2": {
          "type": "number",
          "minimum": 0,
          "description": "Annual CO2 capture capacity in metric tons"
        },
        "modules": {
          "type": "integer",
          "minimum": 1,
          "description": "Number of capture modules"
        },
        "captureRatePerModule": {
          "type": "number",
          "minimum": 0,
          "description": "Tons CO2 per module per day"
        }
      }
    },
    "energySource": {
      "type": "object",
      "properties": {
        "type": {
          "type": "string",
          "enum": [
            "geothermal",
            "solar",
            "wind",
            "hydro",
            "nuclear",
            "natural-gas",
            "grid-mix"
          ]
        },
        "provider": {
          "type": "string"
        },
        "renewable": {
          "type": "boolean"
        },
        "energyPerTonCO2": {
          "type": "number",
          "description": "MWh per ton CO2 captured"
        }
      }
    },
    "storage": {
      "type": "object",
      "properties": {
        "method": {
          "type": "string",
          "enum": [
            "mineralization",
            "geological-storage",
            "ocean-storage",
            "utilization",
            "hybrid"
          ]
        },
        "site": {
          "type": "string",
          "description": "Storage site identifier"
        },
        "depth": {
          "type": "string",
          "description": "Storage depth range"
        },
        "permanence": {
          "type": "string",
          "enum": ["permanent", ">10000-years", ">1000-years", "varies"]
        }
      }
    },
    "operational": {
      "type": "object",
      "properties": {
        "startDate": {
          "type": "string",
          "format": "date"
        },
        "endDate": {
          "type": "string",
          "format": "date"
        },
        "status": {
          "type": "string",
          "enum": ["operational", "testing", "maintenance", "decommissioned"]
        },
        "uptime": {
          "type": "number",
          "minimum": 0,
          "maximum": 1,
          "description": "Operational uptime percentage"
        },
        "maintenanceCycle": {
          "type": "string",
          "enum": ["weekly", "monthly", "quarterly", "annually"]
        }
      }
    },
    "certifications": {
      "type": "array",
      "items": {
        "type": "string"
      },
      "description": "ISO certifications, etc."
    },
    "contacts": {
      "type": "object",
      "properties": {
        "operator": {
          "type": "string"
        },
        "email": {
          "type": "string",
          "format": "email"
        },
        "website": {
          "type": "string",
          "format": "uri"
        }
      }
    },
    "metadata": {
      "type": "object",
      "properties": {
        "createdAt": {
          "type": "string",
          "format": "date-time"
        },
        "updatedAt": {
          "type": "string",
          "format": "date-time"
        },
        "version": {
          "type": "string"
        }
      }
    }
  }
}
```

### 2.2 Example: Facility Information

```json
{
  "facilityId": "dac-climeworks-orca-001",
  "name": "Orca DAC Facility",
  "location": {
    "latitude": 64.0685,
    "longitude": -21.9479,
    "altitude": 268,
    "country": "IS",
    "region": "Hellisheidi",
    "address": "Hellisheidi Geothermal Power Plant, Iceland"
  },
  "technology": {
    "type": "solid-sorbent-dac",
    "sorbent": "amine-functionalized",
    "manufacturer": "Climeworks",
    "model": "Orca",
    "generation": "Generation-3"
  },
  "capacity": {
    "annualTonsCO2": 4000,
    "modules": 8,
    "captureRatePerModule": 0.5
  },
  "energySource": {
    "type": "geothermal",
    "provider": "ON Power",
    "renewable": true,
    "energyPerTonCO2": 2.5
  },
  "storage": {
    "method": "mineralization",
    "site": "carbfix-hellisheidi",
    "depth": "700-2000 meters",
    "permanence": "permanent"
  },
  "operational": {
    "startDate": "2021-09-08",
    "status": "operational",
    "uptime": 0.92,
    "maintenanceCycle": "quarterly"
  },
  "certifications": [
    "ISO 14064-2",
    "ISO 27916"
  ],
  "contacts": {
    "operator": "Climeworks AG",
    "email": "info@climeworks.com",
    "website": "https://climeworks.com"
  },
  "metadata": {
    "createdAt": "2021-09-01T00:00:00Z",
    "updatedAt": "2025-12-25T00:00:00Z",
    "version": "1.0.0"
  }
}
```

---

## 3. Operational Data Format

### 3.1 Real-Time Capture Data Schema

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "title": "DAC Operational Data",
  "type": "object",
  "required": ["timestamp", "facilityId", "capture", "energy"],
  "properties": {
    "timestamp": {
      "type": "string",
      "format": "date-time",
      "description": "ISO 8601 timestamp"
    },
    "facilityId": {
      "type": "string",
      "pattern": "^dac-[a-z0-9-]+$"
    },
    "capture": {
      "type": "object",
      "required": ["currentRate"],
      "properties": {
        "currentRate": {
          "type": "number",
          "minimum": 0,
          "description": "Current capture rate in tons/day"
        },
        "dailyTotal": {
          "type": "number",
          "minimum": 0,
          "description": "Cumulative daily capture in tons"
        },
        "monthlyTotal": {
          "type": "number",
          "minimum": 0
        },
        "yearlyTotal": {
          "type": "number",
          "minimum": 0
        },
        "efficiency": {
          "type": "number",
          "minimum": 0,
          "maximum": 1,
          "description": "Capture efficiency as fraction"
        },
        "co2Purity": {
          "type": "number",
          "minimum": 0,
          "maximum": 100,
          "description": "CO2 purity percentage"
        }
      }
    },
    "energy": {
      "type": "object",
      "properties": {
        "currentConsumption": {
          "type": "number",
          "minimum": 0,
          "description": "Current power consumption in kW"
        },
        "dailyTotal": {
          "type": "number",
          "minimum": 0,
          "description": "Daily energy consumption in MWh"
        },
        "specificEnergy": {
          "type": "number",
          "minimum": 0,
          "description": "MWh per ton CO2 captured"
        },
        "renewablePercentage": {
          "type": "number",
          "minimum": 0,
          "maximum": 100
        }
      }
    },
    "environmental": {
      "type": "object",
      "properties": {
        "ambientCO2": {
          "type": "number",
          "minimum": 0,
          "description": "Ambient CO2 concentration in ppm"
        },
        "temperature": {
          "type": "number",
          "description": "Ambient temperature in Celsius"
        },
        "humidity": {
          "type": "number",
          "minimum": 0,
          "maximum": 100,
          "description": "Relative humidity percentage"
        },
        "pressure": {
          "type": "number",
          "minimum": 0,
          "description": "Atmospheric pressure in hPa"
        },
        "windSpeed": {
          "type": "number",
          "minimum": 0,
          "description": "Wind speed in m/s"
        }
      }
    },
    "sorbent": {
      "type": "object",
      "properties": {
        "cycleNumber": {
          "type": "integer",
          "minimum": 0
        },
        "saturation": {
          "type": "number",
          "minimum": 0,
          "maximum": 1,
          "description": "Sorbent saturation fraction"
        },
        "expectedLifetime": {
          "type": "integer",
          "minimum": 0,
          "description": "Expected total cycles"
        },
        "regenerationTemp": {
          "type": "number",
          "description": "Current regeneration temperature in Celsius"
        }
      }
    },
    "airflow": {
      "type": "object",
      "properties": {
        "rate": {
          "type": "number",
          "minimum": 0,
          "description": "Air flow rate in m³/min"
        },
        "pressure": {
          "type": "number",
          "description": "System pressure in Pa"
        },
        "pressureDrop": {
          "type": "number",
          "description": "Pressure drop across contactor in Pa"
        }
      }
    },
    "status": {
      "type": "object",
      "properties": {
        "operationalState": {
          "type": "string",
          "enum": ["adsorption", "regeneration", "idle", "maintenance", "error"]
        },
        "alerts": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "severity": {
                "type": "string",
                "enum": ["info", "warning", "error", "critical"]
              },
              "message": {
                "type": "string"
              },
              "timestamp": {
                "type": "string",
                "format": "date-time"
              }
            }
          }
        }
      }
    }
  }
}
```

### 3.2 Example: Real-Time Operational Data

```json
{
  "timestamp": "2025-12-25T10:30:00Z",
  "facilityId": "dac-climeworks-orca-001",
  "capture": {
    "currentRate": 0.48,
    "dailyTotal": 3.84,
    "monthlyTotal": 115.2,
    "yearlyTotal": 3840,
    "efficiency": 0.96,
    "co2Purity": 98.5
  },
  "energy": {
    "currentConsumption": 120,
    "dailyTotal": 2.88,
    "specificEnergy": 2.4,
    "renewablePercentage": 100
  },
  "environmental": {
    "ambientCO2": 415,
    "temperature": 12,
    "humidity": 65,
    "pressure": 1013,
    "windSpeed": 3.5
  },
  "sorbent": {
    "cycleNumber": 1247,
    "saturation": 0.85,
    "expectedLifetime": 10000,
    "regenerationTemp": 100
  },
  "airflow": {
    "rate": 5000,
    "pressure": 101300,
    "pressureDrop": 250
  },
  "status": {
    "operationalState": "adsorption",
    "alerts": []
  }
}
```

---

## 4. Sorbent Data Format

### 4.1 Sorbent Material Schema

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "title": "Sorbent Material Data",
  "type": "object",
  "required": ["sorbentId", "type", "properties"],
  "properties": {
    "sorbentId": {
      "type": "string",
      "pattern": "^sorbent-[a-z0-9-]+$"
    },
    "type": {
      "type": "string",
      "enum": [
        "solid-amine",
        "liquid-solvent",
        "metal-organic-framework",
        "zeolite",
        "activated-carbon",
        "other"
      ]
    },
    "properties": {
      "type": "object",
      "properties": {
        "maxLoading": {
          "type": "number",
          "minimum": 0,
          "description": "Maximum CO2 loading in mmol/g"
        },
        "workingCapacity": {
          "type": "number",
          "minimum": 0,
          "description": "Practical working capacity in mmol/g"
        },
        "regenerationTemp": {
          "type": "number",
          "description": "Optimal regeneration temperature in Celsius"
        },
        "regenerationPressure": {
          "type": "number",
          "description": "Regeneration pressure in bar"
        },
        "cycleLifetime": {
          "type": "integer",
          "minimum": 0,
          "description": "Expected number of cycles"
        },
        "degradationRate": {
          "type": "number",
          "description": "Capacity loss per cycle (fraction)"
        },
        "selectivity": {
          "type": "number",
          "minimum": 0,
          "description": "CO2/N2 selectivity ratio"
        }
      }
    },
    "manufacturer": {
      "type": "string"
    },
    "batchNumber": {
      "type": "string"
    },
    "installDate": {
      "type": "string",
      "format": "date"
    },
    "replacementDate": {
      "type": "string",
      "format": "date"
    },
    "lifecycle": {
      "type": "object",
      "properties": {
        "currentCycles": {
          "type": "integer",
          "minimum": 0
        },
        "currentCapacity": {
          "type": "number",
          "description": "Current capacity in mmol/g"
        },
        "degradation": {
          "type": "number",
          "minimum": 0,
          "maximum": 1,
          "description": "Degradation fraction"
        },
        "remainingLifetime": {
          "type": "integer",
          "description": "Estimated remaining cycles"
        }
      }
    }
  }
}
```

### 4.2 Example: Sorbent Data

```json
{
  "sorbentId": "sorbent-climeworks-gen3-batch42",
  "type": "solid-amine",
  "properties": {
    "maxLoading": 3.5,
    "workingCapacity": 2.8,
    "regenerationTemp": 100,
    "regenerationPressure": 1.0,
    "cycleLifetime": 10000,
    "degradationRate": 0.00001,
    "selectivity": 150
  },
  "manufacturer": "Climeworks",
  "batchNumber": "GEN3-2021-042",
  "installDate": "2021-09-01",
  "replacementDate": null,
  "lifecycle": {
    "currentCycles": 1247,
    "currentCapacity": 3.46,
    "degradation": 0.011,
    "remainingLifetime": 8753
  }
}
```

---

## 5. Storage Data Format

### 5.1 Storage Event Schema

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "title": "CO2 Storage Event",
  "type": "object",
  "required": ["eventId", "facilityId", "timestamp", "amount"],
  "properties": {
    "eventId": {
      "type": "string",
      "pattern": "^storage-[a-z0-9-]+$"
    },
    "facilityId": {
      "type": "string"
    },
    "timestamp": {
      "type": "string",
      "format": "date-time"
    },
    "amount": {
      "type": "number",
      "minimum": 0,
      "description": "Tons of CO2 stored"
    },
    "storageMethod": {
      "type": "string",
      "enum": [
        "mineralization",
        "geological-storage",
        "ocean-storage",
        "utilization"
      ]
    },
    "storageSite": {
      "type": "object",
      "properties": {
        "siteId": {
          "type": "string"
        },
        "name": {
          "type": "string"
        },
        "location": {
          "type": "object",
          "properties": {
            "latitude": {
              "type": "number"
            },
            "longitude": {
              "type": "number"
            },
            "depth": {
              "type": "number",
              "description": "Storage depth in meters"
            }
          }
        }
      }
    },
    "injectionData": {
      "type": "object",
      "properties": {
        "temperature": {
          "type": "number",
          "description": "Injection temperature in Celsius"
        },
        "pressure": {
          "type": "number",
          "description": "Injection pressure in bar"
        },
        "rate": {
          "type": "number",
          "description": "Injection rate in tons/hour"
        },
        "duration": {
          "type": "number",
          "description": "Injection duration in hours"
        }
      }
    },
    "co2Quality": {
      "type": "object",
      "properties": {
        "purity": {
          "type": "number",
          "minimum": 0,
          "maximum": 100,
          "description": "CO2 purity percentage"
        },
        "contaminants": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "compound": {
                "type": "string"
              },
              "concentration": {
                "type": "number",
                "description": "ppm"
              }
            }
          }
        }
      }
    },
    "permanence": {
      "type": "object",
      "properties": {
        "guaranteedYears": {
          "type": "integer",
          "minimum": 1000
        },
        "monitoringPlan": {
          "type": "string"
        },
        "insurancePolicy": {
          "type": "string"
        }
      }
    },
    "verification": {
      "type": "object",
      "properties": {
        "verifierId": {
          "type": "string"
        },
        "verificationDate": {
          "type": "string",
          "format": "date"
        },
        "standard": {
          "type": "string"
        },
        "certificateUrl": {
          "type": "string",
          "format": "uri"
        }
      }
    }
  }
}
```

### 5.2 Example: Storage Event

```json
{
  "eventId": "storage-orca-20251225-001",
  "facilityId": "dac-climeworks-orca-001",
  "timestamp": "2025-12-25T14:00:00Z",
  "amount": 10.5,
  "storageMethod": "mineralization",
  "storageSite": {
    "siteId": "carbfix-hellisheidi-001",
    "name": "CarbFix Hellisheidi",
    "location": {
      "latitude": 64.0211,
      "longitude": -21.3122,
      "depth": 1000
    }
  },
  "injectionData": {
    "temperature": 35,
    "pressure": 90,
    "rate": 0.5,
    "duration": 21
  },
  "co2Quality": {
    "purity": 98.5,
    "contaminants": [
      {
        "compound": "H2O",
        "concentration": 1000
      },
      {
        "compound": "N2",
        "concentration": 500
      }
    ]
  },
  "permanence": {
    "guaranteedYears": 10000,
    "monitoringPlan": "Seismic + water sampling annually for 30 years",
    "insurancePolicy": "POL-CARBFIX-2025-001"
  },
  "verification": {
    "verifierId": "dnv-gl",
    "verificationDate": "2025-12-26",
    "standard": "ISO 27916",
    "certificateUrl": "https://verify.wia.org/storage/orca-20251225-001"
  }
}
```

---

## 6. Verification Data Format

### 6.1 Verification Report Schema

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "title": "MRV Verification Report",
  "type": "object",
  "required": ["reportId", "facilityId", "period", "verifiedAmount"],
  "properties": {
    "reportId": {
      "type": "string",
      "pattern": "^verify-[a-z0-9-]+$"
    },
    "facilityId": {
      "type": "string"
    },
    "period": {
      "type": "object",
      "required": ["startDate", "endDate"],
      "properties": {
        "startDate": {
          "type": "string",
          "format": "date"
        },
        "endDate": {
          "type": "string",
          "format": "date"
        }
      }
    },
    "verifiedAmount": {
      "type": "number",
      "minimum": 0,
      "description": "Verified tons CO2 captured"
    },
    "uncertainty": {
      "type": "number",
      "description": "Measurement uncertainty percentage"
    },
    "verifier": {
      "type": "object",
      "properties": {
        "organization": {
          "type": "string"
        },
        "accreditation": {
          "type": "string"
        },
        "auditor": {
          "type": "string"
        }
      }
    },
    "standards": {
      "type": "array",
      "items": {
        "type": "string"
      },
      "description": "Standards used for verification (e.g., ISO 27916)"
    },
    "methodology": {
      "type": "string",
      "description": "Verification methodology description"
    },
    "findings": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "category": {
            "type": "string",
            "enum": ["conformance", "non-conformance", "observation", "recommendation"]
          },
          "description": {
            "type": "string"
          },
          "severity": {
            "type": "string",
            "enum": ["minor", "major", "critical"]
          }
        }
      }
    },
    "conclusion": {
      "type": "string",
      "enum": ["verified", "verified-with-conditions", "not-verified"]
    },
    "verificationDate": {
      "type": "string",
      "format": "date"
    },
    "nextVerificationDue": {
      "type": "string",
      "format": "date"
    },
    "reportUrl": {
      "type": "string",
      "format": "uri"
    }
  }
}
```

---

## 7. Carbon Credit Data Format

### 7.1 Carbon Credit Schema

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "title": "Carbon Removal Credit",
  "type": "object",
  "required": ["creditId", "facilityId", "amount", "vintage"],
  "properties": {
    "creditId": {
      "type": "string",
      "description": "Unique credit identifier"
    },
    "serialNumber": {
      "type": "string",
      "description": "Registry serial number"
    },
    "facilityId": {
      "type": "string"
    },
    "amount": {
      "type": "number",
      "minimum": 0,
      "description": "Tons CO2 removed"
    },
    "vintage": {
      "type": "integer",
      "minimum": 2020,
      "description": "Year of CO2 removal"
    },
    "registry": {
      "type": "string",
      "enum": ["Puro.earth", "Verra", "Gold Standard", "ACR", "Other"]
    },
    "methodology": {
      "type": "string",
      "description": "Carbon crediting methodology"
    },
    "issuanceDate": {
      "type": "string",
      "format": "date"
    },
    "status": {
      "type": "string",
      "enum": ["issued", "transferred", "retired", "cancelled"]
    },
    "owner": {
      "type": "string",
      "description": "Current credit owner"
    },
    "permanence": {
      "type": "object",
      "properties": {
        "years": {
          "type": "integer",
          "minimum": 100
        },
        "guarantee": {
          "type": "string"
        }
      }
    },
    "cobenefits": {
      "type": "array",
      "items": {
        "type": "string"
      },
      "description": "Additional environmental or social benefits"
    },
    "price": {
      "type": "object",
      "properties": {
        "amount": {
          "type": "number"
        },
        "currency": {
          "type": "string",
          "pattern": "^[A-Z]{3}$"
        }
      }
    },
    "blockchain": {
      "type": "object",
      "properties": {
        "network": {
          "type": "string"
        },
        "contractAddress": {
          "type": "string"
        },
        "tokenId": {
          "type": "string"
        },
        "transactionHash": {
          "type": "string"
        }
      }
    }
  }
}
```

---

## 8. API Response Formats

### 8.1 Standard API Response

All API endpoints should return responses in this format:

```json
{
  "success": true,
  "data": { },
  "metadata": {
    "timestamp": "2025-12-25T10:00:00Z",
    "version": "1.0.0",
    "requestId": "req-12345"
  },
  "pagination": {
    "page": 1,
    "limit": 50,
    "total": 150
  }
}
```

### 8.2 Error Response

```json
{
  "success": false,
  "error": {
    "code": "INVALID_FACILITY_ID",
    "message": "Facility ID must match pattern ^dac-[a-z0-9-]+$",
    "details": {
      "field": "facilityId",
      "provided": "invalid-id"
    }
  },
  "metadata": {
    "timestamp": "2025-12-25T10:00:00Z",
    "version": "1.0.0",
    "requestId": "req-12346"
  }
}
```

---

## 9. Data Validation

### 9.1 Validation Rules

1. **Required Fields:** All fields marked as "required" must be present
2. **Type Checking:** Values must match specified types (string, number, etc.)
3. **Format Validation:** Dates, URIs, emails must conform to formats
4. **Range Validation:** Numeric values must be within specified ranges
5. **Enum Validation:** String values must match allowed enum values
6. **Pattern Matching:** Strings must match regex patterns where specified

### 9.2 Validation Tools

- **JSON Schema Validators:** AJV, tv4, jsonschema
- **API Validation:** Express Validator, Joi, Yup
- **TypeScript:** Generate types from JSON Schema

---

## 10. Versioning

### 10.1 Schema Versioning

Schemas follow semantic versioning: MAJOR.MINOR.PATCH

- **MAJOR:** Breaking changes to schema structure
- **MINOR:** Backward-compatible additions
- **PATCH:** Bug fixes, clarifications

### 10.2 Deprecation Policy

- Deprecated fields marked with `"deprecated": true`
- 6-month deprecation period before removal
- Migration guides provided for major versions

---

**弘益人間 (홍익인간) - Benefit All Humanity**

© 2025 SmileStory Inc. / World Internet Alliance (WIA)
