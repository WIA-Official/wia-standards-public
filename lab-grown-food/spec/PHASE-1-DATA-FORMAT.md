# WIA-AGRI-019: Lab-Grown Food
## Phase 1: Data Format Specification

**Version:** 1.0
**Status:** Draft
**Last Updated:** 2025-01-15

---

## 1. Introduction

This specification defines the standardized data formats for cellular agriculture and lab-grown food production systems. It provides a common language for describing cell lines, culture parameters, bioreactor conditions, quality metrics, and production workflows.

### 1.1 Scope

This standard covers:
- Cell line metadata and genealogy
- Bioreactor telemetry and control parameters
- Culture media composition
- Quality assurance and safety testing results
- Production batch tracking
- Regulatory compliance documentation

### 1.2 Conformance

Systems claiming WIA-AGRI-019 Phase 1 conformance MUST:
- Implement all REQUIRED data structures
- Use specified units and measurement standards
- Validate data against provided JSON schemas
- Support international unit systems (SI units)

---

## 2. Core Data Structures

### 2.1 Cell Line Definition

```json
{
  "cellLine": {
    "id": "string (required)",
    "name": "string (required)",
    "species": "string (required)",
    "tissueType": "string (required)",
    "cellType": "enum (required)",
    "source": {
      "type": "enum (required)",
      "origin": "string (optional)",
      "donorId": "string (optional)",
      "biopsyDate": "ISO8601 (optional)"
    },
    "characteristics": {
      "morphology": "string (optional)",
      "growthRate": "number (optional)",
      "doublingTime": "number (optional)",
      "maxPassage": "integer (optional)",
      "immortalized": "boolean (required)"
    },
    "storage": {
      "location": "string (required)",
      "cryopreservationDate": "ISO8601 (optional)",
      "vialCount": "integer (optional)",
      "passage": "integer (required)"
    },
    "validation": {
      "karyotype": "string (optional)",
      "mycoplasma": "enum (required)",
      "crossContamination": "boolean (required)",
      "lastTested": "ISO8601 (required)"
    }
  }
}
```

**Cell Type Enum:**
- `satellite-cell` - Muscle satellite cells
- `fibroblast` - Connective tissue cells
- `myoblast` - Muscle precursor cells
- `adipocyte` - Fat cells
- `ipsc` - Induced pluripotent stem cells
- `esc` - Embryonic stem cells

**Source Type Enum:**
- `biopsy` - Fresh tissue sample
- `cell-bank` - Commercial or research cell bank
- `reprogrammed` - iPSC derived from somatic cells

**Mycoplasma Status Enum:**
- `negative` - No contamination detected
- `positive` - Contamination detected
- `not-tested` - Testing not performed

### 2.2 Culture Batch

```json
{
  "batch": {
    "id": "string (required)",
    "cellLineId": "string (required)",
    "bioreactorId": "string (required)",
    "startDate": "ISO8601 (required)",
    "endDate": "ISO8601 (optional)",
    "status": "enum (required)",
    "phase": "enum (required)",
    "operator": "string (required)",
    "protocol": {
      "id": "string (required)",
      "version": "string (required)",
      "modifications": "array (optional)"
    },
    "targets": {
      "cellDensity": "number (required)",
      "duration": "integer (required)",
      "yieldKg": "number (optional)"
    },
    "metadata": {
      "purpose": "string (optional)",
      "funding": "string (optional)",
      "notes": "string (optional)"
    }
  }
}
```

**Status Enum:**
- `planned` - Scheduled but not started
- `inoculation` - Initial seeding
- `proliferation` - Active cell growth
- `differentiation` - Cell specialization
- `maturation` - Tissue formation
- `harvested` - Completed and removed
- `failed` - Terminated due to issues
- `on-hold` - Temporarily suspended

**Phase Enum:**
- `phase-0` - Pre-culture preparation
- `phase-1` - Seed culture expansion
- `phase-2` - Bioreactor proliferation
- `phase-3` - Differentiation
- `phase-4` - Maturation and harvest

### 2.3 Bioreactor Telemetry

```json
{
  "telemetry": {
    "timestamp": "ISO8601 (required)",
    "bioreactorId": "string (required)",
    "batchId": "string (required)",
    "parameters": {
      "pH": {
        "value": "number (required)",
        "unit": "pH",
        "range": [6.8, 7.4]
      },
      "temperature": {
        "value": "number (required)",
        "unit": "celsius",
        "range": [36.5, 37.5]
      },
      "dissolvedOxygen": {
        "value": "number (required)",
        "unit": "percent",
        "range": [20, 40]
      },
      "agitation": {
        "value": "number (required)",
        "unit": "rpm",
        "range": [40, 120]
      },
      "workingVolume": {
        "value": "number (required)",
        "unit": "liters"
      }
    },
    "nutrients": {
      "glucose": {
        "value": "number (required)",
        "unit": "g/L",
        "threshold": 2.0
      },
      "glutamine": {
        "value": "number (required)",
        "unit": "mM",
        "threshold": 1.0
      },
      "lactate": {
        "value": "number (required)",
        "unit": "mM",
        "maxLevel": 20.0
      },
      "ammonia": {
        "value": "number (required)",
        "unit": "mM",
        "maxLevel": 3.0
      }
    },
    "cellMetrics": {
      "viableCellDensity": {
        "value": "number (required)",
        "unit": "cells/mL"
      },
      "totalCellDensity": {
        "value": "number (required)",
        "unit": "cells/mL"
      },
      "viability": {
        "value": "number (required)",
        "unit": "percent"
      },
      "averageDiameter": {
        "value": "number (optional)",
        "unit": "micrometers"
      }
    },
    "alarms": [
      {
        "code": "string (required)",
        "severity": "enum (required)",
        "message": "string (required)",
        "timestamp": "ISO8601 (required)"
      }
    ]
  }
}
```

**Alarm Severity Enum:**
- `info` - Informational message
- `warning` - Non-critical issue
- `critical` - Immediate attention required
- `emergency` - System failure

### 2.4 Culture Medium Formulation

```json
{
  "medium": {
    "id": "string (required)",
    "name": "string (required)",
    "type": "enum (required)",
    "baseMedia": "string (required)",
    "supplements": [
      {
        "component": "string (required)",
        "concentration": "number (required)",
        "unit": "string (required)",
        "supplier": "string (optional)",
        "catalogNumber": "string (optional)"
      }
    ],
    "serum": {
      "type": "enum (required)",
      "concentration": "number (required)",
      "unit": "percent"
    },
    "growthFactors": [
      {
        "name": "string (required)",
        "concentration": "number (required)",
        "unit": "string (required)"
      }
    ],
    "pH": {
      "target": "number (required)",
      "buffer": "string (required)"
    },
    "osmolality": {
      "value": "number (optional)",
      "unit": "mOsm/kg"
    },
    "sterility": {
      "filtered": "boolean (required)",
      "autoclaved": "boolean (required)",
      "tested": "boolean (required)"
    }
  }
}
```

**Medium Type Enum:**
- `proliferation` - For cell expansion
- `differentiation` - For cell specialization
- `maintenance` - For cell preservation
- `serum-free` - No animal serum

**Serum Type Enum:**
- `fbs` - Fetal bovine serum
- `horse-serum` - Horse serum
- `human-serum` - Human serum
- `none` - Serum-free

### 2.5 Quality Control Results

```json
{
  "qualityControl": {
    "batchId": "string (required)",
    "testDate": "ISO8601 (required)",
    "laboratory": "string (required)",
    "tests": {
      "microbiological": {
        "totalViableCount": {
          "value": "number (required)",
          "unit": "CFU/g",
          "limit": 10000,
          "status": "enum (required)"
        },
        "pathogens": [
          {
            "organism": "string (required)",
            "detected": "boolean (required)",
            "method": "string (required)"
          }
        ],
        "mycotoxins": {
          "tested": "boolean (required)",
          "detected": "boolean (required)"
        }
      },
      "nutritional": {
        "protein": {
          "value": "number (required)",
          "unit": "g/100g"
        },
        "fat": {
          "value": "number (required)",
          "unit": "g/100g"
        },
        "carbohydrates": {
          "value": "number (required)",
          "unit": "g/100g"
        },
        "minerals": [
          {
            "name": "string (required)",
            "value": "number (required)",
            "unit": "string (required)"
          }
        ],
        "vitamins": [
          {
            "name": "string (required)",
            "value": "number (required)",
            "unit": "string (required)"
          }
        ]
      },
      "safety": {
        "heavyMetals": [
          {
            "element": "string (required)",
            "value": "number (required)",
            "unit": "mg/kg",
            "limit": "number (required)",
            "status": "enum (required)"
          }
        ],
        "allergens": {
          "tested": "array (required)",
          "detected": "array (required)"
        },
        "antibiotics": {
          "tested": "boolean (required)",
          "detected": "boolean (required)"
        }
      },
      "sensory": {
        "color": {
          "score": "number (required)",
          "range": [1, 10]
        },
        "texture": {
          "score": "number (required)",
          "range": [1, 10]
        },
        "odor": {
          "score": "number (required)",
          "range": [1, 10]
        },
        "overallAcceptability": {
          "score": "number (required)",
          "range": [1, 10]
        }
      }
    },
    "overallStatus": "enum (required)",
    "certifiedBy": "string (required)"
  }
}
```

**Test Status Enum:**
- `pass` - Meets specifications
- `fail` - Does not meet specifications
- `pending` - Results not available
- `not-applicable` - Test not required

---

## 3. Unit Standards

### 3.1 Measurement Units

| Parameter | Standard Unit | Alternative Units |
|-----------|---------------|-------------------|
| Cell Density | cells/mL | cells/L, 10^6 cells/mL |
| Temperature | Celsius (°C) | Kelvin (K) |
| pH | pH units | - |
| Dissolved Oxygen | Percent saturation (%) | mg/L, ppm |
| Pressure | kPa | bar, psi, atm |
| Volume | Liters (L) | mL, m³ |
| Mass | Kilograms (kg) | g, mg |
| Time | Hours (h) | days, minutes, seconds |
| Agitation | RPM | rad/s |
| Flow Rate | L/min | mL/min, L/h |

### 3.2 Concentration Units

- Glucose: g/L or mM
- Amino acids: mM or mg/L
- Proteins: g/L or mg/mL
- Growth factors: ng/mL or pg/mL
- Serum: % v/v
- Oxygen: % saturation or mg/L

---

## 4. Validation & Compliance

### 4.1 Data Validation

All data structures MUST:
1. Pass JSON schema validation
2. Include required fields
3. Use correct units
4. Fall within acceptable ranges
5. Include timestamp information
6. Be digitally signed (optional)

### 4.2 Schema Validation

JSON schemas are provided for each data structure:
- `cell-line-schema.json`
- `batch-schema.json`
- `telemetry-schema.json`
- `medium-schema.json`
- `qc-schema.json`

Example validation (JavaScript):
```javascript
const Ajv = require('ajv');
const ajv = new Ajv();

const schema = require('./schemas/batch-schema.json');
const validate = ajv.compile(schema);

const valid = validate(batchData);
if (!valid) {
  console.error(validate.errors);
}
```

### 4.3 Interoperability

Systems MUST support:
- JSON format for data exchange
- CSV format for tabular data export
- ISO 8601 for timestamps
- UTF-8 encoding
- RESTful API interfaces (Phase 2)

---

## 5. Security & Privacy

### 5.1 Data Classification

- **Public**: Standard specifications, units
- **Internal**: Production metrics, protocols
- **Confidential**: Proprietary cell lines, formulations
- **Restricted**: Regulatory submissions, safety data

### 5.2 Data Protection

Implementations SHOULD:
- Encrypt data at rest (AES-256)
- Encrypt data in transit (TLS 1.3+)
- Implement access controls (RBAC)
- Maintain audit logs
- Support data anonymization

---

## 6. Examples

### 6.1 Complete Batch Record

```json
{
  "batch": {
    "id": "BATCH-2025-001",
    "cellLineId": "BOVINE-SAT-V2",
    "bioreactorId": "BR-A1",
    "startDate": "2025-01-15T08:00:00Z",
    "status": "proliferation",
    "phase": "phase-2",
    "operator": "Dr. Jane Smith",
    "protocol": {
      "id": "PROTO-BOVINE-STD",
      "version": "2.1",
      "modifications": ["Increased FGF-2 to 10ng/mL"]
    },
    "targets": {
      "cellDensity": 1e7,
      "duration": 14,
      "yieldKg": 5.0
    }
  },
  "currentTelemetry": {
    "timestamp": "2025-01-20T14:30:00Z",
    "parameters": {
      "pH": {"value": 7.22, "unit": "pH"},
      "temperature": {"value": 37.0, "unit": "celsius"},
      "dissolvedOxygen": {"value": 28.5, "unit": "percent"}
    },
    "cellMetrics": {
      "viableCellDensity": {"value": 8.5e6, "unit": "cells/mL"},
      "viability": {"value": 95.2, "unit": "percent"}
    }
  }
}
```

---

## 7. References

- ISO 20387:2018 - Biotechnology — Biobanking
- FDA Guidance: Cell-Based Products for Human Consumption
- EFSA Novel Food Regulations
- GMP Guidelines for Cell Culture Manufacturing
- ISO 8601:2019 - Date and time format

---

## 8. Changelog

**v1.0 (2025-01-15)**
- Initial specification release
- Core data structures defined
- Unit standards established
- Validation requirements specified

---

**© 2025 SmileStory Inc. / WIA**
弘益人間 (홍익인간) · Benefit All Humanity
