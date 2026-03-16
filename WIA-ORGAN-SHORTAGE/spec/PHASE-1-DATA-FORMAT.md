# WIA-ORGAN-SHORTAGE - Phase 1: Data Format

> **Version:** 1.0.0
> **Status:** Complete
> **弘益人間 (Hongik Ingan)** - Benefit All Humanity

## 1. Overview

This document defines the standardized data formats for the WIA-ORGAN-SHORTAGE standard, enabling interoperability between transplant centers, research institutions, and organ procurement organizations worldwide.

## 2. Core Data Schema

### 2.1 Organ Profile Schema

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "$id": "https://wia.live/schemas/organ-shortage/v1.0.0/organ-profile",
  "title": "WIA Organ Profile",
  "type": "object",
  "required": ["patient_id", "organ_needed", "urgency", "immunological"],
  "properties": {
    "patient_id": {
      "type": "string",
      "format": "uuid",
      "description": "Unique patient identifier (UUID v4)"
    },
    "organ_needed": {
      "type": "string",
      "enum": ["kidney", "liver", "heart", "lung", "pancreas", "intestine"],
      "description": "Type of organ required"
    },
    "urgency": {
      "type": "string",
      "enum": ["urgent", "high", "medium", "stable"],
      "description": "Medical urgency level"
    },
    "waitlist_status": {
      "$ref": "#/$defs/WaitlistStatus"
    },
    "immunological": {
      "$ref": "#/$defs/ImmunologicalProfile"
    },
    "alternative_options": {
      "$ref": "#/$defs/AlternativeOptions"
    },
    "xenotransplant": {
      "$ref": "#/$defs/XenotransplantProfile"
    },
    "bioprinted_organ": {
      "$ref": "#/$defs/BioprintedOrganProfile"
    }
  }
}
```

### 2.2 Waitlist Status

```json
{
  "$defs": {
    "WaitlistStatus": {
      "type": "object",
      "properties": {
        "registered_date": {
          "type": "string",
          "format": "date-time"
        },
        "unos_status": {
          "type": "string",
          "description": "UNOS/OPTN status code"
        },
        "waiting_time_days": {
          "type": "integer",
          "minimum": 0
        },
        "geographic_region": {
          "type": "string"
        },
        "listing_center": {
          "type": "string"
        },
        "multi_listing": {
          "type": "boolean",
          "default": false
        }
      }
    }
  }
}
```

### 2.3 Immunological Profile

```json
{
  "$defs": {
    "ImmunologicalProfile": {
      "type": "object",
      "required": ["blood_type"],
      "properties": {
        "blood_type": {
          "type": "string",
          "enum": ["A", "B", "AB", "O"]
        },
        "rh_factor": {
          "type": "string",
          "enum": ["positive", "negative"]
        },
        "hla_typing": {
          "$ref": "#/$defs/HLATyping"
        },
        "pra_percent": {
          "type": "number",
          "minimum": 0,
          "maximum": 100,
          "description": "Panel Reactive Antibody percentage"
        },
        "crossmatch_history": {
          "type": "array",
          "items": {
            "$ref": "#/$defs/CrossmatchResult"
          }
        },
        "unacceptable_antigens": {
          "type": "array",
          "items": { "type": "string" }
        }
      }
    }
  }
}
```

### 2.4 HLA Typing

```json
{
  "$defs": {
    "HLATyping": {
      "type": "object",
      "properties": {
        "classI": {
          "type": "object",
          "properties": {
            "A": { "type": "array", "items": { "type": "string" }, "maxItems": 2 },
            "B": { "type": "array", "items": { "type": "string" }, "maxItems": 2 },
            "C": { "type": "array", "items": { "type": "string" }, "maxItems": 2 }
          }
        },
        "classII": {
          "type": "object",
          "properties": {
            "DR": { "type": "array", "items": { "type": "string" }, "maxItems": 2 },
            "DQ": { "type": "array", "items": { "type": "string" }, "maxItems": 2 },
            "DP": { "type": "array", "items": { "type": "string" }, "maxItems": 2 }
          }
        },
        "typing_method": {
          "type": "string",
          "enum": ["serology", "molecular-sso", "molecular-ssp", "ngs"]
        },
        "resolution": {
          "type": "string",
          "enum": ["low", "intermediate", "high"]
        }
      }
    }
  }
}
```

### 2.5 Alternative Options

```json
{
  "$defs": {
    "AlternativeOptions": {
      "type": "object",
      "properties": {
        "xenotransplant_eligible": {
          "type": "boolean",
          "description": "Eligible for xenotransplantation clinical trial"
        },
        "bioprinted_organ_eligible": {
          "type": "boolean",
          "description": "Eligible for bioprinted organ (research)"
        },
        "living_donor_available": {
          "type": "boolean"
        },
        "domino_transplant_option": {
          "type": "boolean"
        },
        "paired_exchange_enrolled": {
          "type": "boolean"
        }
      }
    }
  }
}
```

### 2.6 Xenotransplant Profile

```json
{
  "$defs": {
    "XenotransplantProfile": {
      "type": "object",
      "properties": {
        "genetic_modifications": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "gene": { "type": "string" },
              "modification_type": {
                "type": "string",
                "enum": ["knockout", "insertion", "inactivation"]
              },
              "purpose": { "type": "string" }
            }
          }
        },
        "trial_enrollment": {
          "type": "string",
          "description": "Clinical trial identifier"
        },
        "immunosuppression_protocol": {
          "type": "string"
        },
        "source_facility": {
          "type": "string"
        },
        "pig_source": {
          "type": "object",
          "properties": {
            "breed": { "type": "string" },
            "facility": { "type": "string" },
            "certification": { "type": "string" }
          }
        }
      }
    }
  }
}
```

### 2.7 Bioprinted Organ Profile

```json
{
  "$defs": {
    "BioprintedOrganProfile": {
      "type": "object",
      "properties": {
        "organ_type": { "type": "string" },
        "vascularization_status": {
          "type": "string",
          "enum": ["none", "partial", "complete"]
        },
        "maturation_days": {
          "type": "integer",
          "minimum": 0
        },
        "functionality_score": {
          "type": "number",
          "minimum": 0,
          "maximum": 1
        },
        "cell_source": {
          "type": "string",
          "enum": ["autologous", "allogeneic", "xenogeneic", "ipsc"]
        },
        "bioink_composition": {
          "type": "array",
          "items": { "type": "string" }
        },
        "printing_facility": { "type": "string" },
        "quality_metrics": {
          "type": "object",
          "properties": {
            "cell_viability": { "type": "number" },
            "structural_integrity": { "type": "number" },
            "vascular_perfusion": { "type": "number" }
          }
        }
      }
    }
  }
}
```

## 3. Organ Codes

| Code | Organ | Description |
|------|-------|-------------|
| WIA-ORG-001 | Kidney | Renal organ |
| WIA-ORG-002 | Liver | Hepatic organ |
| WIA-ORG-003 | Heart | Cardiac organ |
| WIA-ORG-004 | Lung | Pulmonary organ |
| WIA-ORG-005 | Pancreas | Pancreatic organ |
| WIA-ORG-006 | Intestine | Small/Large intestine |

## 4. Urgency Classifications

| Level | Code | Description | Response Time |
|-------|------|-------------|---------------|
| Urgent | 1A | Life-threatening, immediate need | < 24 hours |
| High | 1B | Severe deterioration | < 1 week |
| Medium | 2 | Significant impairment | < 1 month |
| Stable | 3 | Stable condition | Standard queue |

## 5. Example: Complete Organ Profile

```json
{
  "$schema": "https://wia.live/schemas/organ-shortage/v1.0.0",
  "organ_profile": {
    "patient_id": "550e8400-e29b-41d4-a716-446655440000",
    "organ_needed": "kidney",
    "urgency": "high",
    "waitlist_status": {
      "registered_date": "2022-03-15T00:00:00Z",
      "unos_status": "1B",
      "waiting_time_days": 847,
      "geographic_region": "Region 5",
      "listing_center": "Mayo Clinic",
      "multi_listing": false
    },
    "immunological": {
      "blood_type": "O",
      "rh_factor": "positive",
      "hla_typing": {
        "classI": {
          "A": ["02:01", "03:01"],
          "B": ["07:02", "44:02"],
          "C": ["07:02", "05:01"]
        },
        "classII": {
          "DR": ["15:01", "04:01"],
          "DQ": ["06:02", "03:01"]
        },
        "typing_method": "ngs",
        "resolution": "high"
      },
      "pra_percent": 65,
      "crossmatch_history": [],
      "unacceptable_antigens": ["A1", "B8", "DR17"]
    },
    "alternative_options": {
      "xenotransplant_eligible": true,
      "bioprinted_organ_eligible": true,
      "living_donor_available": false,
      "domino_transplant_option": false,
      "paired_exchange_enrolled": true
    },
    "xenotransplant": {
      "genetic_modifications": [
        { "gene": "GGTA1", "modification_type": "knockout", "purpose": "Remove alpha-gal antigen" },
        { "gene": "CMAH", "modification_type": "knockout", "purpose": "Remove Neu5Gc" },
        { "gene": "B4GALNT2", "modification_type": "knockout", "purpose": "Remove Sda antigen" }
      ],
      "trial_enrollment": "NCT06488183",
      "immunosuppression_protocol": "Tacrolimus + MMF + Rituximab"
    }
  },
  "metadata": {
    "standard": "WIA-ORGAN-SHORTAGE",
    "version": "1.0.0",
    "philosophy": "弘益人間",
    "created_at": "2024-12-15T10:30:00Z"
  }
}
```

## 6. Validation Rules

1. **Patient ID**: Must be valid UUID v4
2. **Blood Type**: Must match ABO system
3. **HLA Typing**: Must follow WHO nomenclature
4. **PRA Percent**: Must be 0-100
5. **Dates**: Must be ISO 8601 format

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 MIT License*
