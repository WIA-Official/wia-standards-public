# WIA-IND-004 Phase 1: Data Format Standard
## Beauty Technology Data Schemas

**Version:** 1.0.0
**Status:** Active
**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## Overview

Phase 1 establishes standardized JSON schemas for all beauty technology data types. These schemas ensure interoperability between different systems, apps, and devices while maintaining privacy and security.

## Core Principles

1. **Human-Readable**: JSON format for easy debugging and inspection
2. **Versioned**: All schemas include version numbers for evolution
3. **Extensible**: Custom fields allowed via `extensions` object
4. **Typed**: Strict type definitions for validation
5. **Localized**: Multi-language support via `language` field

---

## 1. Skin Analysis Result Schema

### Purpose
Standardizes output from AI-powered skin analysis systems.

### Schema Definition

```json
{
  "standard": "WIA-IND-004",
  "version": "1.0.0",
  "type": "SkinAnalysisResult",
  "timestamp": "2025-12-27T10:30:00Z",
  "language": "en",
  "data": {
    "analysisId": "string (uuid)",
    "userId": "string (uuid)",
    "imageMetadata": {
      "resolution": "1920x1080",
      "deviceType": "smartphone|camera|specialized",
      "lightingCondition": "natural|indoor|mixed|low",
      "captureAngle": "front|slight-left|slight-right|profile"
    },
    "metrics": {
      "wrinkles": {
        "score": "integer (0-100)",
        "severity": "low|moderate|high",
        "zones": ["forehead", "eyes", "mouth", "neck"],
        "details": {
          "fineLines": "integer (count)",
          "deepWrinkles": "integer (count)",
          "averageDepth": "float (mm)"
        }
      },
      "pores": {
        "score": "integer (0-100)",
        "visibility": "low|moderate|high",
        "density": "float (pores per cm²)",
        "zones": ["nose", "cheeks", "chin", "forehead"],
        "averageSize": "float (mm)"
      },
      "darkSpots": {
        "score": "integer (0-100)",
        "count": "integer",
        "totalArea": "float (cm²)",
        "zones": ["cheeks", "forehead", "nose", "chin"],
        "types": ["hyperpigmentation", "sun-damage", "age-spots"]
      },
      "texture": {
        "score": "integer (0-100)",
        "smoothness": "integer (0-100, inverse of score)",
        "roughness": "float (texture index)"
      },
      "redness": {
        "score": "integer (0-100)",
        "zones": ["cheeks", "nose", "chin"],
        "type": "rosacea|sensitivity|inflammation|normal"
      },
      "moisture": {
        "score": "integer (0-100)",
        "level": "dry|normal|oily",
        "zones": ["tzone", "cheeks", "overall"]
      }
    },
    "skinType": "normal|dry|oily|combination|sensitive",
    "skinTone": {
      "fitzpatrick": "I|II|III|IV|V|VI",
      "undertone": "cool|warm|neutral"
    },
    "overallScore": "integer (0-100)",
    "confidence": "float (0-1)",
    "recommendations": [
      {
        "type": "product|routine|lifestyle|consultation",
        "priority": "high|medium|low",
        "description": "string",
        "expectedImpact": "string"
      }
    ]
  },
  "metadata": {
    "deviceType": "smartphone|tablet|specialized-camera",
    "appVersion": "string",
    "algorithmVersion": "string",
    "processingTime": "integer (ms)",
    "certifiedBy": "WIA-IND-004"
  },
  "philosophy": "弘益人間"
}
```

### Validation Rules

- `score` fields: 0-100 inclusive
- `confidence`: 0.0-1.0 inclusive
- `timestamp`: ISO 8601 format
- `analysisId` and `userId`: UUID v4 format
- All arrays: non-empty if present

---

## 2. Beauty Product Schema

### Purpose
Standardizes product catalog information for interoperability across platforms.

### Schema Definition

```json
{
  "standard": "WIA-IND-004",
  "version": "1.0.0",
  "type": "BeautyProduct",
  "language": "en",
  "data": {
    "productId": "string (SKU or UUID)",
    "name": "string",
    "brand": "string",
    "category": "skincare|makeup|haircare|bodycare",
    "subcategory": "cleanser|serum|moisturizer|sunscreen|lipstick|foundation|...",
    "description": {
      "short": "string (max 200 chars)",
      "long": "string (max 2000 chars)",
      "benefits": ["string"],
      "usage": "string"
    },
    "ingredients": [
      {
        "inciName": "string (INCI nomenclature)",
        "commonName": "string",
        "concentration": "float (%)",
        "purpose": "active|emollient|humectant|preservative|fragrance|...",
        "allergen": "boolean",
        "vegan": "boolean"
      }
    ],
    "suitableFor": {
      "skinTypes": ["normal", "dry", "oily", "combination", "sensitive"],
      "concerns": ["acne", "wrinkles", "hyperpigmentation", "dehydration", "..."],
      "ageGroups": ["teen", "20s", "30s", "40s", "50plus"],
      "skinTones": ["all", "I-III", "IV-VI"]
    },
    "contraindications": {
      "pregnancy": "safe|caution|avoid",
      "nursing": "safe|caution|avoid",
      "conditions": ["rosacea", "eczema", "..."]
    },
    "usage": {
      "frequency": "once-daily|twice-daily|2-3x-weekly|as-needed",
      "application": "string (e.g., 'after-toner-before-moisturizer')",
      "amount": "string (e.g., '2-3 drops', 'pea-sized')",
      "timing": "AM|PM|both"
    },
    "formulation": {
      "texture": "gel|cream|lotion|oil|serum|foam|...",
      "finish": "matte|dewy|satin|natural",
      "scent": "fragrance-free|light|strong",
      "pH": "float (3.0-8.0)"
    },
    "certifications": ["cruelty-free", "vegan", "organic", "dermatologist-tested", "..."],
    "packaging": {
      "size": "float (ml or g)",
      "type": "bottle|jar|tube|pump|dropper|...",
      "material": "glass|plastic|aluminum|...",
      "recyclable": "boolean",
      "refillable": "boolean"
    },
    "price": {
      "amount": "float",
      "currency": "USD|EUR|KRW|...",
      "pricePerUnit": "float (per ml or g)"
    },
    "availability": {
      "inStock": "boolean",
      "quantity": "integer",
      "releaseDate": "ISO 8601 date",
      "discontinuedDate": "ISO 8601 date | null"
    },
    "ratings": {
      "average": "float (1.0-5.0)",
      "count": "integer",
      "distribution": {
        "5star": "integer",
        "4star": "integer",
        "3star": "integer",
        "2star": "integer",
        "1star": "integer"
      }
    },
    "images": [
      {
        "url": "string (HTTPS)",
        "type": "product|packaging|texture|swatch|lifestyle",
        "alt": "string",
        "width": "integer",
        "height": "integer"
      }
    ]
  },
  "philosophy": "弘益人間"
}
```

---

## 3. User Profile Schema

### Purpose
Standardizes user beauty profiles for cross-platform personalization.

### Schema Definition

```json
{
  "standard": "WIA-IND-004",
  "version": "1.0.0",
  "type": "UserProfile",
  "language": "en",
  "data": {
    "userId": "string (UUID)",
    "created": "ISO 8601 timestamp",
    "lastUpdated": "ISO 8601 timestamp",
    "demographics": {
      "age": "integer",
      "gender": "female|male|non-binary|prefer-not-to-say",
      "location": {
        "country": "string (ISO 3166-1)",
        "region": "string",
        "climate": "tropical|dry|temperate|continental|polar"
      }
    },
    "skinProfile": {
      "type": "normal|dry|oily|combination|sensitive",
      "tone": {
        "fitzpatrick": "I|II|III|IV|V|VI",
        "undertone": "cool|warm|neutral"
      },
      "concerns": [
        {
          "type": "acne|wrinkles|hyperpigmentation|dehydration|redness|...",
          "severity": "low|moderate|high",
          "priority": "integer (1=highest)"
        }
      ],
      "sensitivity": "none|low|moderate|high",
      "allergies": ["ingredient1", "ingredient2"]
    },
    "preferences": {
      "preferredIngredients": ["hyaluronic-acid", "niacinamide", "..."],
      "avoidIngredients": ["fragrance", "alcohol", "..."],
      "budgetRange": "budget|mid|premium|luxury",
      "values": ["vegan", "cruelty-free", "sustainable", "clean-beauty", "..."],
      "productTypes": ["serum", "moisturizer", "sunscreen", "..."],
      "textures": ["lightweight", "rich", "gel", "cream", "..."],
      "fragrancePreference": "fragrance-free|light|strong|no-preference"
    },
    "routines": {
      "morning": [
        {
          "step": "integer",
          "productId": "string",
          "productName": "string",
          "frequency": "daily|2-3x-weekly|as-needed"
        }
      ],
      "evening": [
        {
          "step": "integer",
          "productId": "string",
          "productName": "string",
          "frequency": "daily|2-3x-weekly|as-needed"
        }
      ]
    },
    "goals": [
      {
        "type": "clear-skin|anti-aging|hydration|even-tone|...",
        "targetDate": "ISO 8601 date",
        "progress": "integer (0-100)"
      }
    ],
    "history": {
      "skinAnalysisCount": "integer",
      "lastAnalysis": "ISO 8601 timestamp",
      "productsTriedCount": "integer",
      "purchaseCount": "integer",
      "averageOrderValue": "float"
    },
    "privacy": {
      "consentToDataSharing": "boolean",
      "consentToMarketing": "boolean",
      "consentToResearch": "boolean",
      "privacyLevel": "public|friends|private|encrypted",
      "dataRetention": "1year|5years|indefinite"
    }
  },
  "philosophy": "弘益人間"
}
```

---

## 4. Treatment Record Schema

### Purpose
Tracks beauty treatment progress over time for outcome measurement.

### Schema Definition

```json
{
  "standard": "WIA-IND-004",
  "version": "1.0.0",
  "type": "TreatmentRecord",
  "language": "en",
  "data": {
    "treatmentId": "string (UUID)",
    "userId": "string (UUID)",
    "name": "string",
    "startDate": "ISO 8601 date",
    "endDate": "ISO 8601 date | null (if ongoing)",
    "duration": "string (e.g., '8 weeks')",
    "type": "product-routine|professional-treatment|lifestyle-change|combination",
    "products": [
      {
        "productId": "string",
        "name": "string",
        "usage": "string",
        "startDate": "ISO 8601 date",
        "endDate": "ISO 8601 date | null"
      }
    ],
    "targetConcerns": ["acne", "wrinkles", "hyperpigmentation", "..."],
    "progress": {
      "baseline": {
        "date": "ISO 8601 date",
        "analysisId": "string (UUID)",
        "metrics": "SkinAnalysisResult.metrics"
      },
      "checkpoints": [
        {
          "date": "ISO 8601 date",
          "analysisId": "string (UUID)",
          "metrics": "SkinAnalysisResult.metrics",
          "notes": "string"
        }
      ],
      "final": {
        "date": "ISO 8601 date",
        "analysisId": "string (UUID)",
        "metrics": "SkinAnalysisResult.metrics"
      }
    },
    "results": {
      "improvement": "float (% improvement)",
      "concernsResolved": ["string"],
      "newConcerns": ["string"],
      "satisfaction": "very-satisfied|satisfied|neutral|dissatisfied|very-dissatisfied",
      "willContinue": "boolean",
      "wouldRecommend": "boolean"
    },
    "sideEffects": [
      {
        "type": "irritation|redness|dryness|breakout|...",
        "severity": "mild|moderate|severe",
        "date": "ISO 8601 date",
        "resolved": "boolean"
      }
    ],
    "notes": "string (freeform notes)",
    "clinician": {
      "name": "string | null",
      "credentials": "string | null",
      "facilityId": "string | null"
    }
  },
  "philosophy": "弘益人間"
}
```

---

## 5. Recommendation Schema

### Purpose
Standardizes AI-generated product recommendations.

### Schema Definition

```json
{
  "standard": "WIA-IND-004",
  "version": "1.0.0",
  "type": "ProductRecommendation",
  "language": "en",
  "data": {
    "recommendationId": "string (UUID)",
    "userId": "string (UUID)",
    "generatedAt": "ISO 8601 timestamp",
    "expiresAt": "ISO 8601 timestamp",
    "algorithm": {
      "name": "string",
      "version": "string",
      "confidence": "float (0-1)",
      "method": "collaborative|content-based|hybrid|ai-powered"
    },
    "context": {
      "triggeredBy": "skin-analysis|user-request|cart-add|browse|...",
      "season": "spring|summer|fall|winter",
      "budget": "float | null",
      "urgency": "immediate|within-week|flexible"
    },
    "recommendations": [
      {
        "rank": "integer",
        "productId": "string",
        "productName": "string",
        "matchScore": "integer (0-100)",
        "reasons": [
          "matches-skin-type",
          "addresses-concerns",
          "within-budget",
          "highly-rated",
          "popular-with-similar-users",
          "compatible-with-routine",
          "..."
        ],
        "expectedResults": "string",
        "confidence": "float (0-1)",
        "alternatives": ["productId1", "productId2"]
      }
    ],
    "routineSuggestion": {
      "morning": ["productId1", "productId2", "..."],
      "evening": ["productId1", "productId2", "..."],
      "reasoning": "string"
    },
    "educationalContent": [
      {
        "type": "article|video|tutorial",
        "title": "string",
        "url": "string",
        "relevance": "string"
      }
    ]
  },
  "philosophy": "弘益人間"
}
```

---

## Implementation Guidelines

### Validation
All implementations must validate data against these schemas before accepting or transmitting.

### Required Libraries
- JSON Schema validator (e.g., Ajv for JavaScript, jsonschema for Python)
- UUID generator (UUID v4)
- ISO 8601 timestamp library

### Storage
- Encrypt all personal data (user profiles, skin analysis) at rest (AES-256)
- Use HTTPS/TLS 1.3 for all data transmission
- Implement proper access controls (RBAC)

### Versioning
- Include schema version in all documents
- Support backward compatibility for minor version changes
- Provide migration scripts for major version changes

### Extensions
Custom fields can be added via `extensions` object:
```json
{
  ...
  "extensions": {
    "vendorName": {
      "customField": "value"
    }
  }
}
```

---

## Compliance

All implementations claiming WIA-IND-004 Phase 1 compliance must:
1. Support all five core schemas
2. Validate all inputs and outputs
3. Encrypt sensitive data
4. Include `philosophy: "弘益人間"` in all documents
5. Pass WIA certification testing

---

**Maintained by:** WIA (World Certification Industry Association)
**Contact:** standards@wia.org
**Last Updated:** 2025-12-27

弘益人間 · Benefit All Humanity

---

## Annex A — Conformance Tier Matrix

WIA conformance for WIA-IND-004-beauty-tech is evaluated across three tiers:

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

- `wia-standards/standards/WIA-IND-004-beauty-tech/api/` — TypeScript SDK skeleton
- `wia-standards/standards/WIA-IND-004-beauty-tech/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/WIA-IND-004-beauty-tech/simulator/` — interactive browser-based simulator for the PHASE protocol

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
