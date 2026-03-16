# WIA-IND-006 Phase 1: Data Format Specification
## Personalized Cosmetics Standard - Data Schemas and Formats

**Version:** 1.0
**Last Updated:** 2025-01-15
**Status:** Active
**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## Overview

Phase 1 of the WIA-IND-006 standard defines standardized data formats and schemas for personalized cosmetics systems. These specifications ensure interoperability between skin analysis tools, formulation algorithms, manufacturing systems, and consumer applications.

## 1. User Profile Schema

### 1.1 Core Profile Structure

```json
{
  "profileId": "string (UUID)",
  "userId": "string (unique identifier)",
  "createdAt": "ISO 8601 timestamp",
  "updatedAt": "ISO 8601 timestamp",
  "version": "string (semver)",
  "demographicData": { },
  "skinAnalysis": { },
  "preferences": { },
  "allergens": [],
  "medicalHistory": { }
}
```

### 1.2 Demographic Data

```json
"demographicData": {
  "age": "integer (13-120)",
  "gender": "string (optional)",
  "ethnicity": "string (optional)",
  "location": {
    "country": "ISO 3166-1 alpha-2 code",
    "region": "string",
    "climate": "enum [tropical, dry, temperate, cold, humid]"
  }
}
```

### 1.3 Skin Analysis Data

```json
"skinAnalysis": {
  "timestamp": "ISO 8601",
  "analysisType": "enum [visual, sensor, imaging, genetic]",
  "skinType": "enum [oily, dry, combination, normal, sensitive]",
  "skinTone": {
    "fitzpatrickScale": "integer (1-6)",
    "hexColor": "string (#RRGGBB)",
    "undertone": "enum [cool, warm, neutral]"
  },
  "measurements": {
    "moistureLevel": "integer (0-100)",
    "oilLevel": "integer (0-100)",
    "pH": "float (4.0-7.0)",
    "TEWL": "float (g/m²/h)",
    "elasticity": "integer (0-100)",
    "poreSize": "enum [small, medium, large]",
    "sensitivity": "integer (1-10)"
  },
  "concerns": [
    {
      "type": "enum [acne, aging, pigmentation, dryness, redness, texture]",
      "severity": "integer (1-10)",
      "location": "enum [full-face, t-zone, cheeks, forehead, chin, eyes]"
    }
  ],
  "imagingData": {
    "uvImage": "base64 or URL",
    "polarizedImage": "base64 or URL",
    "nirImage": "base64 or URL",
    "aiAnalysis": {
      "wrinkleCount": "integer",
      "wrinkleDepth": "float (mm)",
      "spotCount": "integer",
      "poreCount": "integer",
      "confidenceScore": "float (0.0-1.0)"
    }
  }
}
```

### 1.4 Preferences

```json
"preferences": {
  "texture": "enum [gel, cream, lotion, serum, oil, balm]",
  "fragrance": "enum [none, light, moderate, strong]",
  "fragranceType": "enum [unscented, lavender, rose, citrus, vanilla, custom]",
  "ingredientPreference": "enum [natural, synthetic, balanced]",
  "sustainabilityPriority": "integer (1-10)",
  "budget": "enum [economy, mid-range, premium, luxury]",
  "packaging": "enum [minimal, standard, luxury, refillable]"
}
```

### 1.5 Allergens and Contraindications

```json
"allergens": [
  {
    "ingredient": "string (INCI name)",
    "severity": "enum [mild, moderate, severe]",
    "verified": "boolean",
    "notes": "string"
  }
]
```

## 2. Formulation Specification Format

### 2.1 Core Formulation Structure

```json
{
  "formulationId": "string (UUID)",
  "version": "string (semver)",
  "createdAt": "ISO 8601",
  "profileId": "string (reference to user profile)",
  "productType": "string",
  "baseFormulation": "string (template ID or custom)",
  "ingredients": [],
  "properties": {},
  "manufacturing": {},
  "qualityControl": {},
  "labeling": {}
}
```

### 2.2 Ingredient Specification

```json
"ingredients": [
  {
    "id": "string (unique ID)",
    "inciName": "string (INCI nomenclature)",
    "commonName": "string",
    "cas": "string (CAS registry number)",
    "concentration": "float (0.0-100.0)",
    "unit": "enum [%, ppm, mg/ml]",
    "function": "enum [active, emulsifier, preservative, fragrance, colorant, carrier]",
    "source": "enum [natural, synthetic, biotechnology]",
    "supplier": "string",
    "lotNumber": "string",
    "expiryDate": "ISO 8601",
    "certifications": ["organic", "vegan", "cruelty-free"]
  }
]
```

### 2.3 Formulation Properties

```json
"properties": {
  "pH": "float (4.0-7.0)",
  "viscosity": "float (cP)",
  "density": "float (g/ml)",
  "color": "string (#RRGGBB or LAB)",
  "texture": "enum [gel, cream, lotion, serum]",
  "stability": {
    "shelfLife": "integer (months)",
    "storageConditions": "string",
    "stabilityTested": "boolean",
    "testDate": "ISO 8601"
  },
  "efficacy": {
    "primaryBenefits": ["hydration", "anti-aging", "brightening"],
    "secondaryBenefits": ["soothing", "pore-minimizing"],
    "expectedResults": "string",
    "timeToResults": "integer (days)"
  }
}
```

### 2.4 Manufacturing Instructions

```json
"manufacturing": {
  "batchSize": "integer (units)",
  "productionDate": "ISO 8601",
  "facilityId": "string",
  "productionLine": "string",
  "procedure": [
    {
      "step": "integer",
      "action": "string",
      "temperature": "float (°C)",
      "duration": "integer (seconds)",
      "mixingSpeed": "integer (RPM)",
      "notes": "string"
    }
  ],
  "equipmentUsed": ["mixer-A", "dispenser-B", "filler-C"]
}
```

## 3. Skin Analysis Result Format

### 3.1 Analysis Session

```json
{
  "sessionId": "string (UUID)",
  "profileId": "string (reference)",
  "timestamp": "ISO 8601",
  "location": "string (in-store, at-home, clinic)",
  "deviceId": "string",
  "operator": "string (optional)",
  "environmentalConditions": {
    "temperature": "float (°C)",
    "humidity": "float (%)",
    "lighting": "string"
  },
  "measurements": {},
  "recommendations": {}
}
```

### 3.2 Multi-Modal Measurements

```json
"measurements": {
  "visual": {
    "images": [
      {
        "type": "enum [visible, uv, polarized, nir]",
        "url": "string",
        "timestamp": "ISO 8601",
        "resolution": "string (WxH)",
        "metadata": {}
      }
    ]
  },
  "sensor": {
    "moistureReadings": [
      {
        "location": "enum [forehead, cheeks, nose, chin]",
        "value": "integer (0-100)",
        "unit": "arbitrary units"
      }
    ],
    "oilReadings": [],
    "phReadings": [],
    "tewlReadings": []
  },
  "ai": {
    "modelId": "string",
    "modelVersion": "string",
    "features": {
      "wrinkles": {"count": 0, "severity": 0},
      "spots": {"count": 0, "type": ""},
      "pores": {"count": 0, "size": ""},
      "redness": {"severity": 0, "distribution": ""}
    },
    "confidenceScores": {}
  }
}
```

## 4. Ingredient Database Schema

### 4.1 Ingredient Record

```json
{
  "ingredientId": "string (UUID)",
  "inciName": "string (official INCI)",
  "commonNames": ["string"],
  "cas": "string",
  "chemicalFormula": "string",
  "molecularWeight": "float",
  "category": "enum [active, emulsifier, preservative, etc.]",
  "functions": ["moisturizing", "anti-aging"],
  "source": "enum [plant, animal, mineral, synthetic, biotechnology]",
  "safetyData": {
    "maxConcentration": "float (%)",
    "minConcentration": "float (%)",
    "restrictedUse": "boolean",
    "restrictions": "string",
    "pregnancySafe": "boolean",
    "commonAllergen": "boolean",
    "comedogenic": "integer (0-5)",
    "irritationPotential": "enum [low, moderate, high]"
  },
  "efficacyData": {
    "benefits": ["hydration", "brightening"],
    "evidenceLevel": "enum [clinical, in-vitro, anecdotal]",
    "studies": [
      {
        "title": "string",
        "url": "string",
        "year": "integer",
        "findings": "string"
      }
    ]
  },
  "compatibility": {
    "incompatibleWith": ["ingredient IDs"],
    "synergyWith": ["ingredient IDs"],
    "pHStable": "float range",
    "temperatureStable": "float range"
  },
  "regulatory": {
    "euApproved": "boolean",
    "fdaApproved": "boolean",
    "restrictions": [
      {
        "jurisdiction": "string",
        "status": "enum [approved, restricted, banned]",
        "maxConcentration": "float (%)",
        "notes": "string"
      }
    ]
  }
}
```

## 5. QR Code and Verifiable Credential Formats

### 5.1 Product QR Code Data

```json
{
  "qrVersion": "1.0",
  "productId": "string (UUID)",
  "formulationId": "string (UUID)",
  "batchNumber": "string",
  "manufactureDate": "ISO 8601",
  "expiryDate": "ISO 8601",
  "url": "https://verify.wia-ind-006.org/product/{productId}",
  "ingredients": ["INCI names"],
  "certifications": ["organic", "vegan", "cruelty-free"],
  "blockchain": {
    "network": "string",
    "txHash": "string",
    "contractAddress": "string"
  }
}
```

### 5.2 Verifiable Credential

```json
{
  "@context": [
    "https://www.w3.org/2018/credentials/v1",
    "https://wia-official.org/credentials/ind-006/v1"
  ],
  "id": "urn:uuid:{uuid}",
  "type": ["VerifiableCredential", "ProductAuthenticityCredential"],
  "issuer": "did:wia:ind006:manufacturer:{id}",
  "issuanceDate": "ISO 8601",
  "expirationDate": "ISO 8601",
  "credentialSubject": {
    "id": "did:wia:ind006:product:{id}",
    "productType": "personalized-cosmetic",
    "formulationId": "string",
    "qualityChecks": {
      "passed": "boolean",
      "certifications": [],
      "inspector": "string",
      "date": "ISO 8601"
    }
  },
  "proof": {
    "type": "Ed25519Signature2020",
    "created": "ISO 8601",
    "verificationMethod": "did:key#{keyId}",
    "proofPurpose": "assertionMethod",
    "proofValue": "base64"
  }
}
```

## 6. Data Exchange Formats

### 6.1 Profile Export Format

For GDPR compliance and data portability, user profiles must be exportable in machine-readable JSON format as defined above, optionally with CSV representation for non-technical users.

### 6.2 Batch Data Format

For bulk operations (multiple profiles, formulations, or analyses):

```json
{
  "batchId": "string (UUID)",
  "version": "1.0",
  "timestamp": "ISO 8601",
  "count": "integer",
  "type": "enum [profiles, formulations, analyses]",
  "data": [
    {  }
  ]
}
```

## 7. Data Validation Rules

### 7.1 Required Fields

All timestamps must be in ISO 8601 format with timezone.
All UUIDs must be RFC 4122 compliant.
All percentages must be 0.0-100.0 inclusive.
All enum values must match defined options exactly.

### 7.2 Constraints

- Profile age: 13-120
- Ingredient concentration: 0.0-100.0%
- pH: 4.0-7.0 for cosmetic products
- Skin measurements: 0-100 arbitrary units
- Confidence scores: 0.0-1.0

### 7.3 Data Integrity

All formulations must reference valid profileId.
All ingredients must reference valid ingredientId from approved database.
Total ingredient concentrations should sum to 100% ± 2% tolerance.

## 8. Versioning and Updates

Data format version follows semantic versioning (MAJOR.MINOR.PATCH).
Breaking changes increment MAJOR version.
Backward-compatible additions increment MINOR version.
Bug fixes increment PATCH version.

Systems must support at least two MAJOR versions simultaneously for transition periods.

---

**Copyright © 2025 SmileStory Inc. / WIA**
**弘益人間 - Benefit All Humanity**
