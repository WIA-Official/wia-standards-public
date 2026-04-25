# WIA-IND-005: Phase 1 - Data Format Specification

**Version:** 1.0
**Status:** Active
**Philosophy:** 弘益人間 - Benefit All Humanity

## Overview

Phase 1 defines standardized data formats for cosmetic ingredient information, product formulations, safety ratings, and regulatory compliance data. This specification ensures interoperability across systems and enables global data exchange.

## Core Data Formats

### 1. Ingredient Data Format

#### INCI Ingredient Schema

```json
{
  "@context": "https://wia-standards.org/contexts/ind-005/v1",
  "type": "CosmeticIngredient",
  "inci_name": "string (required)",
  "identifiers": {
    "cas_number": "string (required)",
    "ec_number": "string",
    "einecs_number": "string",
    "alternative_names": ["string"]
  },
  "chemistry": {
    "molecular_formula": "string",
    "molecular_weight": "number",
    "smiles": "string",
    "inchi": "string",
    "inchi_key": "string"
  },
  "function": {
    "primary": "string (enum)",
    "secondary": ["string"],
    "mechanism": "string"
  },
  "safety": {
    "ewg_score": "number (1-10)",
    "ewg_rating": "string (enum: Low Hazard, Moderate Hazard, High Hazard)",
    "eu_cosing_status": "string (enum: Approved, Restricted, Prohibited)",
    "fda_status": "string",
    "sccs_opinion": "string",
    "allergen_potential": "string (enum: Low, Moderate, High)",
    "phototoxicity": "string (enum: None, Low, Moderate, High)",
    "comedogenic_rating": "number (0-5)"
  },
  "regulatory": {
    "eu_approved": "boolean",
    "us_approved": "boolean",
    "china_approved": "boolean",
    "japan_approved": "boolean",
    "max_concentration": {
      "leave_on": "string",
      "rinse_off": "string"
    },
    "restrictions": ["string"],
    "required_warnings": ["string"]
  },
  "allergen": {
    "is_eu_26_allergen": "boolean",
    "allergen_type": "string",
    "cross_reactivity": ["string"],
    "threshold_leave_on": "number",
    "threshold_rinse_off": "number"
  },
  "sustainability": {
    "biodegradable": "boolean",
    "biodegradation_rate": "string",
    "natural_origin": "boolean",
    "synthetic": "boolean",
    "vegan": "boolean",
    "halal_certified": "boolean",
    "kosher_certified": "boolean",
    "certifications": ["string"]
  },
  "sourcing": {
    "manufacturing_methods": ["string"],
    "major_suppliers": ["string"],
    "origin_regions": ["string"],
    "sustainability_certifications": ["string"]
  },
  "metadata": {
    "created_at": "datetime (ISO 8601)",
    "updated_at": "datetime (ISO 8601)",
    "data_sources": ["string"],
    "confidence_score": "number (0-1)",
    "philosophy": "弘益人間"
  }
}
```

#### Ingredient Function Enumerations

- **Surfactant:** Cleansing, foaming, emulsifying
- **Emollient:** Skin softening and moisturizing
- **Humectant:** Moisture attraction and retention
- **Preservative:** Antimicrobial protection
- **Emulsifier:** Oil-water phase blending
- **Active Ingredient:** Therapeutic or beneficial effects
- **Fragrance:** Scent provision
- **Colorant:** Color addition
- **UV Filter:** Sun protection
- **Antioxidant:** Oxidation prevention
- **Thickener:** Viscosity modification
- **pH Adjuster:** pH modification
- **Chelating Agent:** Metal ion sequestration

### 2. Product Formulation Format

#### Product Schema

```json
{
  "@context": "https://wia-standards.org/contexts/ind-005/v1",
  "type": "CosmeticProduct",
  "product_id": "string (required, unique)",
  "gtin": "string (GTIN-13 or GTIN-14)",
  "name": "string (required)",
  "brand": "string",
  "category": "string (enum)",
  "formulation_type": "string (enum)",
  "intended_use": "string",
  "target_demographics": {
    "skin_types": ["string"],
    "age_groups": ["string"],
    "gender": "string (enum: All, Male, Female, Non-Binary)"
  },
  "ingredients": [
    {
      "inci": "string (required)",
      "percentage": "number (0-100)",
      "function": "string",
      "phase": "string (enum: Water, Oil, Emulsifier, Active, Additive)",
      "addition_order": "integer"
    }
  ],
  "phases": [
    {
      "phase_name": "string",
      "phase_type": "string (enum)",
      "mixing_temperature": "string",
      "mixing_speed": "string",
      "mixing_duration": "string",
      "ingredients": ["object"]
    }
  ],
  "processing": {
    "method": "string",
    "equipment_type": "string",
    "homogenization_speed": "string",
    "cooling_rate": "string",
    "final_ph": "string (range)",
    "final_viscosity": "string"
  },
  "safety_summary": {
    "overall_rating": "number (0-10)",
    "allergen_count": "integer",
    "eu_26_allergens": ["string"],
    "regulatory_compliant": "boolean",
    "compliance_markets": ["string"]
  },
  "claims": {
    "marketing_claims": ["string"],
    "substantiation_documents": ["string"],
    "clinical_studies": ["string"]
  },
  "certifications": ["string"],
  "manufacturing": {
    "batch_number": "string",
    "manufacture_date": "date (ISO 8601)",
    "expiry_date": "date (ISO 8601)",
    "pao": "integer (months)",
    "country_of_origin": "string (ISO 3166-1 alpha-2)",
    "facility_id": "string",
    "gmp_certified": "boolean"
  },
  "packaging": {
    "primary_material": "string",
    "secondary_material": "string",
    "recyclable": "boolean",
    "recycled_content_percentage": "number",
    "refillable": "boolean",
    "weight": "number (grams)",
    "volume": "number (ml)"
  },
  "metadata": {
    "created_at": "datetime (ISO 8601)",
    "updated_at": "datetime (ISO 8601)",
    "version": "string (semantic versioning)",
    "philosophy": "弘益人間"
  }
}
```

#### Product Category Enumerations

- Skin Care (Cleanser, Moisturizer, Serum, Toner, Mask, Exfoliant, Eye Care, Sun Care)
- Hair Care (Shampoo, Conditioner, Treatment, Styling, Colorant)
- Makeup (Foundation, Concealer, Powder, Blush, Eye Shadow, Mascara, Lipstick, Nail Polish)
- Body Care (Body Wash, Lotion, Scrub, Deodorant)
- Fragrance (Eau de Parfum, Eau de Toilette, Cologne, Body Spray)
- Oral Care (Toothpaste, Mouthwash)
- Baby Care (Baby Wash, Baby Lotion, Diaper Cream)

### 3. Safety Assessment Format

#### Safety Rating Schema

```json
{
  "@context": "https://wia-standards.org/contexts/ind-005/v1",
  "type": "SafetyAssessment",
  "assessment_id": "string (required, unique)",
  "product_id": "string (required)",
  "assessment_date": "date (ISO 8601)",
  "assessor": {
    "name": "string",
    "qualifications": ["string"],
    "license_number": "string",
    "country": "string"
  },
  "methodology": "string (enum: SCCS, FDA, Health Canada, etc.)",
  "ingredient_safety": [
    {
      "inci": "string",
      "concentration": "number",
      "noael": "number (mg/kg bw/day)",
      "sed": "number (mg/kg bw/day)",
      "mos": "number",
      "acceptable": "boolean",
      "notes": "string"
    }
  ],
  "overall_assessment": {
    "safe_for_intended_use": "boolean",
    "restrictions": ["string"],
    "warnings": ["string"],
    "contraindications": ["string"]
  },
  "allergen_assessment": {
    "hript_conducted": "boolean",
    "sensitization_rate": "number",
    "allergen_alerts": ["string"]
  },
  "microbiological_safety": {
    "challenge_test_passed": "boolean",
    "preservative_efficacy": "string"
  },
  "stability_data": {
    "accelerated_stability": "boolean",
    "shelf_life_months": "integer",
    "storage_conditions": "string"
  },
  "philosophy": "弘益人間 - Safety for all users"
}
```

### 4. Regulatory Compliance Format

#### Compliance Record Schema

```json
{
  "@context": "https://wia-standards.org/contexts/ind-005/v1",
  "type": "RegulatoryCompliance",
  "compliance_id": "string (required, unique)",
  "product_id": "string (required)",
  "target_market": "string (ISO 3166-1 alpha-2)",
  "regulatory_framework": "string (enum)",
  "compliance_status": "string (enum: Compliant, Non-Compliant, Pending)",
  "registration_number": "string",
  "registration_date": "date (ISO 8601)",
  "checks": [
    {
      "check_type": "string",
      "status": "string (enum: Pass, Fail, Warning)",
      "details": "string",
      "regulation_reference": "string"
    }
  ],
  "prohibited_ingredients": [
    {
      "inci": "string",
      "reason": "string",
      "regulation_reference": "string"
    }
  ],
  "restricted_ingredients": [
    {
      "inci": "string",
      "max_concentration": "string",
      "conditions": "string",
      "regulation_reference": "string"
    }
  ],
  "required_testing": ["string"],
  "required_documentation": ["string"],
  "labeling_requirements": {
    "ingredient_list_format": "string",
    "allergen_declaration": ["string"],
    "warnings": ["string"],
    "usage_instructions": "string"
  },
  "responsible_person": {
    "name": "string",
    "address": "string",
    "country": "string",
    "contact": "string"
  },
  "pif_location": "string (URL or file reference)",
  "metadata": {
    "last_checked": "datetime (ISO 8601)",
    "next_review_date": "date (ISO 8601)",
    "philosophy": "弘益人間"
  }
}
```

## Data Format Requirements

### JSON-LD Context

All data formats must include the WIA-IND-005 JSON-LD context for semantic interoperability:

```json
{
  "@context": {
    "@version": 1.1,
    "@base": "https://wia-standards.org/ind-005/",
    "wia": "https://wia-standards.org/vocab/",
    "schema": "https://schema.org/",
    "inci": "wia:inci_name",
    "cas": "wia:cas_number",
    "safety": "wia:safety_rating"
  }
}
```

### Character Encoding

- **UTF-8 encoding required** for all text fields
- Support for multilingual content (99+ languages)
- Unicode normalization (NFC) for consistent representation

### Data Validation

#### Required Fields

All data must include:
- `@context`: JSON-LD context URL
- `type`: Data type identifier
- Unique identifier field (varies by type)
- `metadata.philosophy`: "弘益人間" or equivalent

#### Field Constraints

- **Percentages:** 0-100 with maximum 2 decimal places
- **Dates:** ISO 8601 format (YYYY-MM-DD or full datetime)
- **URLs:** Valid HTTP/HTTPS URLs
- **Email:** RFC 5322 compliant
- **Phone:** E.164 format recommended

### File Formats

#### Primary Format: JSON

```json
{
  "file_extension": ".json",
  "mime_type": "application/json",
  "encoding": "UTF-8",
  "pretty_print": true,
  "indent": 2
}
```

#### Alternative Format: XML

```xml
<?xml version="1.0" encoding="UTF-8"?>
<cosmeticIngredient xmlns="https://wia-standards.org/schemas/ind-005/v1">
  <inciName>Aqua</inciName>
  <philosophy>弘益人間</philosophy>
</cosmeticIngredient>
```

## Data Storage and Transmission

### Database Schema

Recommended database structure:
- **Relational:** PostgreSQL with JSONB support
- **Document:** MongoDB with schema validation
- **Graph:** Neo4j for relationship mapping

### API Transmission

- **Format:** JSON over HTTPS
- **Compression:** gzip or brotli
- **Pagination:** Cursor-based for large datasets
- **Rate Limiting:** Documented in Phase 2

### File Exchange

- **Batch Format:** JSON Lines (.jsonl) for bulk transfer
- **Compression:** .zip or .tar.gz
- **Encryption:** AES-256-GCM for sensitive data

## Versioning and Compatibility

### Semantic Versioning

Format versions follow semantic versioning (MAJOR.MINOR.PATCH):
- **MAJOR:** Breaking changes
- **MINOR:** Backward-compatible additions
- **PATCH:** Backward-compatible fixes

### Backward Compatibility

- Version 1.x must support reading all 1.0 data
- Deprecated fields marked with `@deprecated` annotation
- Minimum 2-year support for deprecated fields

## Philosophy Statement

**弘益人間 (Benefit All Humanity):** These data formats are designed to be accessible, transparent, and usable by all stakeholders—from large manufacturers to small indie brands, from regulatory authorities to individual consumers. Open standards enable innovation while protecting safety and promoting trust.

---

**Document Version:** 1.0
**Last Updated:** 2025-01-15
**Status:** Active
**Philosophy:** 弘益人間 - Benefit All Humanity

---

## Annex A — Conformance Tier Matrix

WIA conformance for cosmetics-data is evaluated across three tiers:

| Tier | Scope | Mandatory artifacts | Audit cadence |
|------|-------|--------------------|----------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | OpenAPI 3.0 contract, JSON Schema validation report, security threat model | Annual self-review |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Tier 1 artifacts + signed third-party assessor report against this PHASE | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Tier 2 artifacts + WIA accreditation, ISO/IEC 17065:2012 conformity assessment, evidence retention ≥ 7 years | Every 12 months |

Implementations MUST disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page. Tier downgrade events MUST be reported to the WIA registry within 30 days.

---

## Annex B — Cross-Walk to International Standards

The PHASE specification reuses or normatively references published standards alongside this document. Implementers SHOULD review the relevant standards alongside this PHASE document; where a conflict exists, the more specific WIA requirement governs unless explicitly superseded by a binding national regulation.

- ISO/IEC 17065:2012 — Conformity assessment — Requirements for bodies certifying products, processes and services
- ISO/IEC 27001:2022 — Information security management systems
- IETF RFC 9457 — Problem Details for HTTP APIs
- IETF RFC 7519 — JSON Web Token (JWT)
- IETF RFC 6749 — The OAuth 2.0 Authorization Framework
- W3C PROV-DM — provenance data model

This cross-walk is informative only. WIA does not republish the referenced documents; readers MUST obtain authoritative copies from the issuing body. Cross-walk entries are reviewed at every minor version of this PHASE.

---

## Annex C — Reference Implementations and Test Vectors

### C.1 Reference Implementations

WIA does not require implementers to use a particular library, but maintains pointers to the canonical reference implementation directories under the WIA-Official GitHub organization:

- `wia-standards/standards/cosmetics-data/api/` — TypeScript SDK skeleton
- `wia-standards/standards/cosmetics-data/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/cosmetics-data/simulator/` — interactive browser-based simulator for the PHASE protocol

Each reference artifact ships with an MIT license and is intended as a starting point, not as a production-grade implementation.

### C.2 Test Vectors

A normative set of request/response pairs covering the schemas defined in this PHASE is published alongside this document. Implementations claiming Tier 2 or Tier 3 conformance MUST pass every published test vector in both serialization (request) and deserialization (response) directions, and MUST publish a signed report identifying which vectors were exercised.

Test vectors are versioned independently of the PHASE document; refer to the `test-vectors/` directory for the active version.

---

## Annex D — Open Questions and Future Work

This PHASE document captures the consensus position at v1.0. The following items are tracked for future minor or major revisions:

1. **Schema versioning policy** — formal MUST/SHOULD rules for backward-compatible vs. breaking changes between minor releases.
2. **Privacy-preserving aggregation** — guidance on differential privacy and secure aggregation patterns for telemetry channels, without prescribing a specific algorithm.
3. **Multilingual error catalogs** — localization strategy for the standard error codes defined in this PHASE.
4. **Long-term retention** — alignment with sectoral retention regulations across jurisdictions.
5. **Sustainability disclosure** — optional fields for energy and emissions reporting tied to operations covered by this PHASE.

Items in this annex are non-normative. Comments and proposals are accepted via the GitHub issues tracker on the WIA-Official organization.

---
