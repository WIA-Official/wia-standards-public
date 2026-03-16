# WIA-PLASTIC-ENZYME Phase 1: Data Format Specification

> **Version:** 1.0.0
> **Status:** Official
> **Last Updated:** 2025-01-01
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

The WIA-PLASTIC-ENZYME Data Format specification defines standardized data structures for representing enzyme profiles, plastic substrates, degradation processes, and quality metrics. This specification enables interoperability between research laboratories, production facilities, regulatory bodies, and supply chain partners.

### 1.1 Purpose

The primary goals of this data format specification are:

- **Interoperability:** Enable seamless data exchange between diverse systems
- **Extensibility:** Support future enzymes, plastics, and assessment methods
- **Traceability:** Maintain complete chain of custody for recycled materials
- **Quality Assurance:** Ensure consistent product quality through standardized metrics

### 1.2 Scope

This specification covers:

- Enzyme profile data structures
- Plastic substrate characterization
- Degradation process recording
- Monomer quality metrics
- Metadata and provenance information

---

## 2. Data Model Architecture

### 2.1 Entity Relationship

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│     Enzyme      │───▶│     Process      │───▶│    Products     │
└─────────────────┘    └──────────────────┘    └─────────────────┘
        │                      │                        │
        │                      │                        │
        ▼                      ▼                        ▼
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│    Kinetics     │    │   Conditions     │    │  QualityMetrics │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

### 2.2 Core Entities

| Entity | Description | Primary Key |
|--------|-------------|-------------|
| Enzyme | Plastic-degrading enzyme profile | enzyme_id |
| Plastic | Substrate material to be degraded | plastic_id |
| Process | Complete degradation operation | process_id |
| Products | Recovered monomers (TPA, EG) | product_id |
| Kinetics | Enzyme kinetic parameters | enzyme_id |
| QualityMetrics | Product quality measurements | batch_id |

---

## 3. JSON Schema Definitions

### 3.1 Enzyme Profile Schema

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "$id": "https://wia.live/schemas/plastic-enzyme/v1.0.0/enzyme-profile.json",
  "title": "WIA Enzyme Profile",
  "description": "Complete enzyme profile following WIA-PLASTIC-ENZYME v1.0",
  "type": "object",
  "required": ["enzyme_id", "name", "classification", "source", "optimal_conditions"],
  "properties": {
    "enzyme_id": {
      "type": "string",
      "format": "uri",
      "description": "Globally unique identifier (URN format)",
      "pattern": "^urn:wia:enzyme:[a-z0-9-]+$"
    },
    "name": {
      "type": "string",
      "description": "Common name of the enzyme",
      "minLength": 1,
      "maxLength": 100
    },
    "classification": {
      "type": "string",
      "enum": ["PETase", "MHETase", "BHETase", "Cutinase", "Lipase", "Esterase"],
      "description": "Enzyme classification category"
    },
    "source": {
      "$ref": "#/$defs/EnzymeSource"
    },
    "target_plastics": {
      "type": "array",
      "items": {
        "type": "string",
        "enum": ["PET", "PBAT", "PLA", "PCL", "PHA", "PBS"]
      },
      "minItems": 1
    },
    "kinetics": {
      "$ref": "#/$defs/EnzymeKinetics"
    },
    "optimal_conditions": {
      "$ref": "#/$defs/OptimalConditions"
    },
    "stability": {
      "$ref": "#/$defs/EnzymeStability"
    },
    "products": {
      "$ref": "#/$defs/EnzymeProducts"
    },
    "sequence": {
      "type": "string",
      "description": "Amino acid sequence (single-letter code)"
    },
    "structure_pdb": {
      "type": "string",
      "description": "PDB ID for structural data"
    },
    "reference": {
      "type": "string",
      "format": "uri",
      "description": "DOI or URL to primary publication"
    },
    "metadata": {
      "$ref": "#/$defs/Metadata"
    }
  },
  "$defs": {
    "EnzymeSource": {
      "type": "object",
      "required": ["organism"],
      "properties": {
        "organism": { "type": "string" },
        "domain": {
          "type": "string",
          "enum": ["bacteria", "archaea", "fungi", "engineered"]
        },
        "environment": {
          "type": "string",
          "enum": ["marine", "soil", "geothermal", "wastewater", "laboratory"]
        },
        "engineered": { "type": "boolean", "default": false },
        "parent_enzyme": { "type": "string" },
        "mutations": {
          "type": "array",
          "items": { "type": "string" }
        }
      }
    },
    "EnzymeKinetics": {
      "type": "object",
      "properties": {
        "km": {
          "type": "object",
          "properties": {
            "value": { "type": "number", "minimum": 0 },
            "unit": { "type": "string", "default": "mM" }
          }
        },
        "kcat": {
          "type": "object",
          "properties": {
            "value": { "type": "number", "minimum": 0 },
            "unit": { "type": "string", "default": "s-1" }
          }
        },
        "kcat_km": {
          "type": "object",
          "properties": {
            "value": { "type": "number", "minimum": 0 },
            "unit": { "type": "string", "default": "M-1s-1" }
          }
        }
      }
    },
    "OptimalConditions": {
      "type": "object",
      "properties": {
        "temperature_c": {
          "type": "object",
          "properties": {
            "optimal": { "type": "number" },
            "range": {
              "type": "array",
              "items": { "type": "number" },
              "minItems": 2,
              "maxItems": 2
            }
          }
        },
        "ph": {
          "type": "object",
          "properties": {
            "optimal": { "type": "number" },
            "range": {
              "type": "array",
              "items": { "type": "number" },
              "minItems": 2,
              "maxItems": 2
            }
          }
        },
        "substrate_loading_percent": {
          "type": "number",
          "minimum": 0,
          "maximum": 100
        }
      }
    },
    "EnzymeStability": {
      "type": "object",
      "properties": {
        "half_life_hours": { "type": "number", "minimum": 0 },
        "thermostability_tm_c": { "type": "number" },
        "storage_conditions": { "type": "string" }
      }
    },
    "EnzymeProducts": {
      "type": "object",
      "properties": {
        "primary": {
          "type": "array",
          "items": {
            "type": "string",
            "enum": ["TPA", "MHET", "BHET", "EG", "oligomers"]
          }
        },
        "toxicity_class": {
          "type": "string",
          "enum": ["non-toxic", "low", "moderate", "high"]
        }
      }
    },
    "Metadata": {
      "type": "object",
      "properties": {
        "created_at": { "type": "string", "format": "date-time" },
        "updated_at": { "type": "string", "format": "date-time" },
        "wia_version": { "type": "string" },
        "created_by": { "type": "string" }
      }
    }
  }
}
```

### 3.2 Degradation Process Schema

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "$id": "https://wia.live/schemas/plastic-enzyme/v1.0.0/degradation-process.json",
  "title": "WIA Degradation Process",
  "type": "object",
  "required": ["process_id", "plastic_input", "enzyme_cocktail", "conditions"],
  "properties": {
    "process_id": {
      "type": "string",
      "format": "uri",
      "pattern": "^urn:wia:process:[a-z0-9-]+$"
    },
    "facility_id": {
      "type": "string",
      "format": "uri"
    },
    "timestamp": {
      "type": "string",
      "format": "date-time"
    },
    "plastic_input": {
      "type": "object",
      "required": ["type", "weight_kg"],
      "properties": {
        "type": {
          "type": "string",
          "enum": ["PET", "PBAT", "PLA", "PCL", "PHA"]
        },
        "form": {
          "type": "string",
          "enum": ["flakes", "powder", "film", "fiber", "mixed"]
        },
        "weight_kg": { "type": "number", "minimum": 0 },
        "crystallinity_percent": { "type": "number", "minimum": 0, "maximum": 100 },
        "contamination_level": {
          "type": "string",
          "enum": ["none", "low", "medium", "high"]
        },
        "source": { "type": "string" }
      }
    },
    "enzyme_cocktail": {
      "type": "array",
      "items": {
        "type": "object",
        "required": ["enzyme_id", "concentration_mg_g"],
        "properties": {
          "enzyme_id": { "type": "string" },
          "concentration_mg_g": { "type": "number", "minimum": 0 }
        }
      },
      "minItems": 1
    },
    "conditions": {
      "type": "object",
      "properties": {
        "temperature_c": { "type": "number" },
        "ph": { "type": "number" },
        "duration_hours": { "type": "number", "minimum": 0 },
        "agitation_rpm": { "type": "number", "minimum": 0 },
        "volume_liters": { "type": "number", "minimum": 0 }
      }
    },
    "output": {
      "type": "object",
      "properties": {
        "degradation_percent": { "type": "number", "minimum": 0, "maximum": 100 },
        "tpa_yield_kg": { "type": "number", "minimum": 0 },
        "tpa_purity_percent": { "type": "number", "minimum": 0, "maximum": 100 },
        "eg_yield_kg": { "type": "number", "minimum": 0 },
        "eg_purity_percent": { "type": "number", "minimum": 0, "maximum": 100 },
        "residue_kg": { "type": "number", "minimum": 0 }
      }
    },
    "quality_grade": {
      "type": "string",
      "enum": ["food-contact", "bottle-grade", "textile-grade", "industrial"]
    },
    "certification_status": {
      "type": "string",
      "enum": ["pending", "WIA-certified", "rejected"]
    }
  }
}
```

---

## 4. Plastic Type Codes

| Code | Full Name | Description | Primary Enzymes |
|------|-----------|-------------|-----------------|
| PET | Polyethylene Terephthalate | Bottles, containers, textiles | PETase, MHETase |
| PBAT | Polybutylene Adipate Terephthalate | Biodegradable packaging | PETase, Cutinase |
| PLA | Polylactic Acid | Bioplastic from renewable sources | Proteinase K |
| PCL | Polycaprolactone | Medical and packaging | Lipase |
| PHA | Polyhydroxyalkanoates | Bacterial bioplastics | PHA depolymerase |
| PBS | Polybutylene Succinate | Biodegradable films | Cutinase |

---

## 5. Data Exchange Formats

### 5.1 Supported Formats

| Format | Use Case | Content-Type |
|--------|----------|--------------|
| JSON | API exchanges, storage | application/json |
| JSON-LD | Linked data, semantic web | application/ld+json |
| CSV | Bulk import/export | text/csv |
| Protocol Buffers | High-performance binary | application/protobuf |

### 5.2 JSON-LD Context

```json
{
  "@context": {
    "@version": 1.1,
    "wia": "https://wia.live/schemas/plastic-enzyme/v1.0.0/",
    "enzyme_id": "@id",
    "name": "wia:name",
    "classification": "wia:classification",
    "kinetics": "wia:kinetics",
    "km": "wia:km",
    "kcat": "wia:kcat"
  }
}
```

---

## 6. Validation Rules

### 6.1 Enzyme Profile Validation

- `enzyme_id` must be unique and in URN format
- `kinetics.km` and `kinetics.kcat` must be positive numbers
- `optimal_conditions.temperature_c.range[0]` must be less than `range[1]`
- `optimal_conditions.ph.range` must be within 0-14

### 6.2 Process Validation

- `output.degradation_percent` must be 0-100
- Total output yields cannot exceed input mass (within 5% tolerance)
- `conditions.temperature_c` should be within enzyme's optimal range
- `conditions.ph` should be within enzyme's optimal range

---

## 7. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01-01 | Initial release |

---

**弘益人間 (Benefit All Humanity)**

© 2025 WIA - World Certification Industry Association
