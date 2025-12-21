# WIA Food Allergy Passport Data Format Standard
## Phase 1 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #84CC16 (Lime)

---

## Table of Contents

1. [Overview](#overview)
2. [Terminology](#terminology)
3. [Base Structure](#base-structure)
4. [Data Schema](#data-schema)
5. [Field Specifications](#field-specifications)
6. [Data Types](#data-types)
7. [Validation Rules](#validation-rules)
8. [Examples](#examples)
9. [Version History](#version-history)

---

## Overview

### 1.1 Purpose

The WIA Food Allergy Passport Data Format Standard defines a unified format for managing personal food allergy information across global food service ecosystems. This standard enables individuals to safely share their allergy profiles with restaurants, airlines, hospitals, and other food service providers through a secure, multilingual, QR-code-based digital passport.

**Core Objectives**:
- Ensure accurate communication of allergy information across language barriers
- Support emergency medical response with critical allergy data
- Enable seamless integration with restaurant POS and airline catering systems
- Maintain privacy while providing necessary medical information
- Reduce allergic reaction incidents through standardized data sharing

### 1.2 Scope

This standard covers:

| Domain | Description |
|--------|-------------|
| Allergy Profile | Individual allergen sensitivities and severity levels |
| Medical Information | Emergency medications, medical IDs, emergency contacts |
| Multilingual Support | Allergy information in multiple languages |
| QR Code Format | Encoded passport data for quick scanning |
| Integration Points | Restaurant, airline, and healthcare system interfaces |

### 1.3 Design Principles

1. **Safety First**: Accurate, complete allergy information to prevent reactions
2. **Multilingual**: Support for 50+ languages for global travel
3. **Privacy**: Encrypted personal data with selective disclosure
4. **Accessibility**: QR codes, NFC, and voice-based access
5. **Interoperability**: Compatible with FDA, EU, and global allergen standards

---

## Terminology

### 2.1 Core Terms

| Term | Definition |
|------|------------|
| **Passport Holder** | Individual with food allergies using the passport |
| **Allergen** | Substance that can cause allergic reaction |
| **Severity Level** | Classification of reaction intensity (mild, moderate, severe, anaphylaxis) |
| **Cross-Contamination** | Presence of allergens in food not intended to contain them |
| **Safe Threshold** | Minimum allergen amount that triggers reaction |
| **Emergency Protocol** | Steps to take during allergic reaction |

### 2.2 Data Types

| Type | Description | Example |
|------|-------------|---------|
| `string` | UTF-8 encoded text | `"Peanuts"` |
| `allergen_code` | FDA/EU allergen code | `"FDA-PEANUT"` |
| `icd_code` | ICD-10 diagnosis code | `"Z91.010"` |
| `severity` | Reaction severity level | `"anaphylaxis"` |
| `language_code` | ISO 639-1 language code | `"en"`, `"ko"`, `"ja"` |
| `timestamp` | ISO 8601 datetime | `"2025-01-15T10:30:00Z"` |

### 2.3 Field Requirements

| Marker | Meaning |
|--------|---------|
| **REQUIRED** | Must be present |
| **OPTIONAL** | May be omitted |
| **CONDITIONAL** | Required under specific conditions |

---

## Base Structure

### 3.1 Allergy Passport Format

```json
{
  "$schema": "https://wia.live/food-allergy-passport/v1/schema.json",
  "version": "1.0.0",
  "passportId": "FAP-2025-000001",
  "status": "active",
  "created": "2025-01-15T10:00:00Z",
  "lastUpdated": "2025-01-15T10:30:00Z",
  "expiresAt": "2026-01-15T10:00:00Z",
  "holder": {
    "userId": "USER-2025-001",
    "displayName": "encrypted:...",
    "dateOfBirth": "encrypted:...",
    "emergencyContacts": [],
    "medicalId": "encrypted:...",
    "photoHash": "sha256:..."
  },
  "allergies": [
    {
      "allergen": "peanuts",
      "allergenCode": "FDA-PEANUT",
      "icdCode": "Z91.010",
      "severity": "anaphylaxis",
      "symptoms": [],
      "firstDiagnosed": "2020-03-15",
      "lastReaction": "2024-12-10T14:30:00Z",
      "threshold": "trace amounts",
      "verifiedBy": "Dr. Kim, Allergist"
    }
  ],
  "medications": {
    "epipen": {
      "brand": "EpiPen",
      "dosage": "0.3mg",
      "location": "always carried",
      "expiresAt": "2025-12-31"
    },
    "antihistamines": []
  },
  "dietary": {
    "preferences": ["vegetarian"],
    "restrictions": [],
    "safeFoods": []
  },
  "translations": {
    "languages": ["en", "ko", "ja", "zh", "es"],
    "primaryLanguage": "en"
  },
  "qrCode": {
    "data": "base64-encoded-qr-data",
    "version": 10,
    "errorCorrection": "H"
  },
  "verification": {
    "medicallyVerified": true,
    "verifiedBy": "Seoul National University Hospital",
    "verifiedAt": "2025-01-10T09:00:00Z",
    "nextReview": "2026-01-10T09:00:00Z"
  },
  "meta": {
    "hash": "sha256:...",
    "signature": "...",
    "previousHash": "..."
  }
}
```

### 3.2 Field Details

#### 3.2.1 `passportId` (REQUIRED)

```
Type: string
Format: FAP-YYYY-NNNNNN
Description: Unique identifier for this allergy passport
Example: "FAP-2025-000001"
```

#### 3.2.2 `status` (REQUIRED)

```
Type: string
Valid values:
  - "active"      : Passport is current and valid
  - "expired"     : Passport needs renewal
  - "suspended"   : Temporarily inactive
  - "revoked"     : Cancelled by holder
  - "archived"    : Historical record
```

---

## Data Schema

### 4.1 Complete JSON Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wia.live/food-allergy-passport/v1/schema.json",
  "title": "WIA Food Allergy Passport",
  "type": "object",
  "required": ["version", "passportId", "status", "created", "holder", "allergies"],
  "properties": {
    "version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+\\.\\d+$",
      "description": "Schema version"
    },
    "passportId": {
      "type": "string",
      "pattern": "^FAP-\\d{4}-\\d{6}$",
      "description": "Unique passport identifier"
    },
    "status": {
      "type": "string",
      "enum": ["active", "expired", "suspended", "revoked", "archived"],
      "description": "Passport status"
    },
    "created": {
      "type": "string",
      "format": "date-time",
      "description": "Creation timestamp"
    },
    "lastUpdated": {
      "type": "string",
      "format": "date-time",
      "description": "Last modification timestamp"
    },
    "expiresAt": {
      "type": "string",
      "format": "date-time",
      "description": "Expiration date (typically 1 year from creation)"
    },
    "holder": {
      "type": "object",
      "required": ["userId"],
      "properties": {
        "userId": { "type": "string" },
        "displayName": { "type": "string" },
        "dateOfBirth": { "type": "string" },
        "emergencyContacts": { "type": "array" },
        "medicalId": { "type": "string" },
        "photoHash": { "type": "string" }
      }
    },
    "allergies": {
      "type": "array",
      "minItems": 1,
      "items": {
        "type": "object",
        "required": ["allergen", "severity"],
        "properties": {
          "allergen": { "type": "string" },
          "allergenCode": { "type": "string" },
          "icdCode": { "type": "string" },
          "severity": {
            "type": "string",
            "enum": ["mild", "moderate", "severe", "anaphylaxis"]
          },
          "symptoms": { "type": "array" },
          "firstDiagnosed": { "type": "string" },
          "lastReaction": { "type": "string" },
          "threshold": { "type": "string" },
          "verifiedBy": { "type": "string" }
        }
      }
    },
    "medications": {
      "type": "object",
      "properties": {
        "epipen": { "type": "object" },
        "antihistamines": { "type": "array" },
        "corticosteroids": { "type": "array" }
      }
    },
    "translations": {
      "type": "object",
      "properties": {
        "languages": {
          "type": "array",
          "items": { "type": "string", "pattern": "^[a-z]{2}$" }
        },
        "primaryLanguage": { "type": "string" }
      }
    }
  }
}
```

### 4.2 Allergy Entry Schema

```json
{
  "allergen": "peanuts",
  "allergenCode": "FDA-PEANUT",
  "icdCode": "Z91.010",
  "severity": "anaphylaxis",
  "symptoms": [
    {
      "type": "respiratory",
      "description": "difficulty breathing",
      "onsetTime": "5-10 minutes"
    },
    {
      "type": "skin",
      "description": "hives and swelling",
      "onsetTime": "immediate"
    },
    {
      "type": "cardiovascular",
      "description": "drop in blood pressure",
      "onsetTime": "10-15 minutes"
    }
  ],
  "firstDiagnosed": "2020-03-15",
  "lastReaction": "2024-12-10T14:30:00Z",
  "reactionHistory": [
    {
      "date": "2024-12-10T14:30:00Z",
      "location": "Restaurant, Seoul",
      "exposureAmount": "trace",
      "reaction": "anaphylaxis",
      "treatment": "EpiPen administered",
      "hospitalized": true
    }
  ],
  "threshold": "trace amounts",
  "crossReactivity": ["tree nuts", "legumes"],
  "verifiedBy": "Dr. Kim, Allergist",
  "verificationDate": "2025-01-10",
  "notes": "Severe peanut allergy, always carries EpiPen"
}
```

### 4.3 Emergency Medications Schema

```json
{
  "medications": {
    "epipen": {
      "brand": "EpiPen",
      "genericName": "epinephrine",
      "dosage": "0.3mg",
      "route": "intramuscular",
      "location": "always carried",
      "quantity": 2,
      "expiresAt": "2025-12-31",
      "instructions": "Inject into outer thigh, call 911 immediately",
      "prescribedBy": "Dr. Kim",
      "prescriptionDate": "2025-01-01"
    },
    "antihistamines": [
      {
        "brand": "Benadryl",
        "genericName": "diphenhydramine",
        "dosage": "25mg",
        "route": "oral",
        "instructions": "Take for mild reactions",
        "quantity": 10
      }
    ],
    "corticosteroids": [
      {
        "brand": "Prednisone",
        "dosage": "20mg",
        "route": "oral",
        "instructions": "As prescribed by doctor",
        "prescribedBy": "Dr. Kim"
      }
    ]
  }
}
```

---

## Field Specifications

### 5.1 FDA Major Allergens

The FDA recognizes 9 major food allergens (as of 2023):

| Allergen | FDA Code | ICD-10 Code | Common Names |
|----------|----------|-------------|--------------|
| Milk | FDA-MILK | Z91.011 | Dairy, lactose, casein, whey |
| Eggs | FDA-EGG | Z91.012 | Albumin, ovalbumin |
| Fish | FDA-FISH | Z91.013 | All finned fish species |
| Shellfish | FDA-SHELLFISH | Z91.014 | Crustaceans, mollusks |
| Tree Nuts | FDA-TREENUT | Z91.010 | Almonds, walnuts, cashews, etc. |
| Peanuts | FDA-PEANUT | Z91.010 | Groundnuts, arachis |
| Wheat | FDA-WHEAT | Z91.018 | Gluten (when wheat-based) |
| Soybeans | FDA-SOY | Z91.015 | Soy protein, lecithin |
| Sesame | FDA-SESAME | Z91.019 | Tahini, sesame oil |

### 5.2 Severity Levels

| Level | Description | Symptoms | Treatment |
|-------|-------------|----------|-----------|
| `mild` | Minor discomfort | Mild itching, tingling mouth | Antihistamines, monitoring |
| `moderate` | Noticeable reaction | Hives, stomach pain, vomiting | Antihistamines, medical evaluation |
| `severe` | Serious reaction | Difficulty breathing, swelling | Emergency medication, hospital visit |
| `anaphylaxis` | Life-threatening | Multiple systems affected, shock | EpiPen, immediate 911, hospital |

### 5.3 Symptom Categories

| Category | Examples | Onset Time |
|----------|----------|------------|
| `skin` | Hives, redness, swelling, eczema | Immediate - 2 hours |
| `respiratory` | Wheezing, coughing, throat tightness | 5-30 minutes |
| `gastrointestinal` | Nausea, vomiting, diarrhea, cramping | 30 minutes - 2 hours |
| `cardiovascular` | Low blood pressure, rapid pulse, shock | 10-30 minutes |
| `neurological` | Dizziness, confusion, anxiety | 10-30 minutes |

### 5.4 Supported Languages

| Code | Language | Native Name |
|------|----------|-------------|
| `en` | English | English |
| `ko` | Korean | 한국어 |
| `ja` | Japanese | 日本語 |
| `zh` | Chinese | 中文 |
| `es` | Spanish | Español |
| `fr` | French | Français |
| `de` | German | Deutsch |
| `it` | Italian | Italiano |
| `pt` | Portuguese | Português |
| `ar` | Arabic | العربية |
| `ru` | Russian | Русский |
| `hi` | Hindi | हिन्दी |

---

## Data Types

### 6.1 Custom Types

```typescript
type PassportStatus =
  | 'active'
  | 'expired'
  | 'suspended'
  | 'revoked'
  | 'archived';

type SeverityLevel =
  | 'mild'
  | 'moderate'
  | 'severe'
  | 'anaphylaxis';

type SymptomCategory =
  | 'skin'
  | 'respiratory'
  | 'gastrointestinal'
  | 'cardiovascular'
  | 'neurological';

interface AllergyEntry {
  allergen: string;
  allergenCode: string;
  icdCode: string;
  severity: SeverityLevel;
  symptoms: Symptom[];
  firstDiagnosed: string;
  lastReaction?: string;
  threshold: string;
  crossReactivity?: string[];
  verifiedBy?: string;
  notes?: string;
}

interface Symptom {
  type: SymptomCategory;
  description: string;
  onsetTime: string;
}

interface EmergencyMedication {
  brand: string;
  genericName: string;
  dosage: string;
  route: 'oral' | 'intramuscular' | 'intravenous' | 'sublingual';
  instructions: string;
  location?: string;
  quantity?: number;
  expiresAt?: string;
}
```

---

## Validation Rules

### 7.1 Required Field Validation

| Rule ID | Field | Validation |
|---------|-------|------------|
| VAL-001 | `passportId` | Must match `^FAP-\d{4}-\d{6}$` |
| VAL-002 | `allergies` | At least 1 allergy entry required |
| VAL-003 | `allergies[].severity` | Must be valid severity level |
| VAL-004 | `translations.languages` | Must include primaryLanguage |
| VAL-005 | `expiresAt` | Must be within 2 years of creation |

### 7.2 Business Logic Validation

| Rule ID | Description | Error Code |
|---------|-------------|------------|
| BUS-001 | Anaphylaxis severity must have EpiPen listed | `ERR_MISSING_EPIPEN` |
| BUS-002 | Emergency contacts required for severe allergies | `ERR_MISSING_CONTACTS` |
| BUS-003 | Medical verification required for anaphylaxis | `ERR_UNVERIFIED_SEVERE` |
| BUS-004 | QR code must encode valid passport data | `ERR_INVALID_QR` |
| BUS-005 | Photo hash must match holder image | `ERR_PHOTO_MISMATCH` |

### 7.3 Error Codes

| Code | Message | Description |
|------|---------|-------------|
| `ERR_INVALID_PASSPORT` | Invalid passport format | ID format violation |
| `ERR_MISSING_EPIPEN` | EpiPen required for anaphylaxis | Safety requirement |
| `ERR_EXPIRED_PASSPORT` | Passport has expired | Needs renewal |
| `ERR_INVALID_ALLERGEN` | Unknown allergen code | Code not recognized |
| `ERR_MISSING_TRANSLATION` | Required language missing | Translation incomplete |

---

## Examples

### 8.1 Valid Allergy Passport - Peanut Allergy

```json
{
  "$schema": "https://wia.live/food-allergy-passport/v1/schema.json",
  "version": "1.0.0",
  "passportId": "FAP-2025-000001",
  "status": "active",
  "created": "2025-01-15T10:00:00Z",
  "lastUpdated": "2025-01-15T10:30:00Z",
  "expiresAt": "2026-01-15T10:00:00Z",
  "holder": {
    "userId": "USER-2025-001",
    "displayName": "encrypted:aes256:QmFzZTY0...",
    "dateOfBirth": "encrypted:aes256:QmFzZTY0...",
    "emergencyContacts": [
      {
        "name": "encrypted:aes256:...",
        "relationship": "spouse",
        "phone": "encrypted:aes256:...",
        "primary": true
      }
    ],
    "medicalId": "encrypted:aes256:...",
    "photoHash": "sha256:abc123def456..."
  },
  "allergies": [
    {
      "allergen": "peanuts",
      "allergenCode": "FDA-PEANUT",
      "icdCode": "Z91.010",
      "severity": "anaphylaxis",
      "symptoms": [
        {
          "type": "respiratory",
          "description": "difficulty breathing, throat closing",
          "onsetTime": "5-10 minutes"
        },
        {
          "type": "skin",
          "description": "severe hives and facial swelling",
          "onsetTime": "immediate"
        },
        {
          "type": "cardiovascular",
          "description": "drop in blood pressure, dizziness",
          "onsetTime": "10-15 minutes"
        }
      ],
      "firstDiagnosed": "2020-03-15",
      "lastReaction": "2024-12-10T14:30:00Z",
      "threshold": "trace amounts",
      "crossReactivity": ["tree nuts", "legumes"],
      "verifiedBy": "Dr. Sarah Kim, Allergist",
      "verificationDate": "2025-01-10",
      "notes": "Severe allergy - even airborne particles can trigger reaction"
    }
  ],
  "medications": {
    "epipen": {
      "brand": "EpiPen",
      "genericName": "epinephrine",
      "dosage": "0.3mg",
      "route": "intramuscular",
      "location": "always carried in purse",
      "quantity": 2,
      "expiresAt": "2025-12-31",
      "instructions": "Inject into outer thigh, call 911 immediately, may repeat after 5-15 minutes if needed",
      "prescribedBy": "Dr. Sarah Kim",
      "prescriptionDate": "2025-01-01"
    },
    "antihistamines": [
      {
        "brand": "Benadryl",
        "genericName": "diphenhydramine",
        "dosage": "25mg",
        "route": "oral",
        "instructions": "Take for mild reactions only",
        "quantity": 10
      }
    ]
  },
  "dietary": {
    "preferences": [],
    "restrictions": ["all peanut products", "foods processed in facilities with peanuts"],
    "safeFoods": ["most fruits", "vegetables", "rice", "chicken", "beef"]
  },
  "translations": {
    "languages": ["en", "ko", "ja", "zh", "es", "fr"],
    "primaryLanguage": "en",
    "customPhrases": {
      "ko": "저는 땅콩 알레르기가 심각합니다. 땅콩이 조금이라도 들어간 음식을 먹으면 생명이 위험합니다.",
      "ja": "私は重度のピーナッツアレルギーがあります。ピーナッツが少しでも入っていると命に関わります。",
      "zh": "我对花生严重过敏。即使少量花生也会危及生命。",
      "es": "Tengo alergia severa al maní. Incluso cantidades pequeñas pueden ser mortales.",
      "fr": "J'ai une allergie sévère aux arachides. Même de petites quantités peuvent être mortelles."
    }
  },
  "qrCode": {
    "data": "base64:iVBORw0KGgoAAAANSUhEUgAA...",
    "version": 10,
    "errorCorrection": "H",
    "size": 512
  },
  "verification": {
    "medicallyVerified": true,
    "verifiedBy": "Seoul National University Hospital",
    "verifiedAt": "2025-01-10T09:00:00Z",
    "verificationMethod": "medical records + allergy test results",
    "nextReview": "2026-01-10T09:00:00Z"
  },
  "sharing": {
    "defaultVisibility": "emergency-info-only",
    "shareHistory": [
      {
        "sharedWith": "Korean Air",
        "sharedAt": "2025-01-14T08:00:00Z",
        "purpose": "flight meal preparation",
        "dataShared": ["allergies", "severity"]
      }
    ]
  },
  "meta": {
    "hash": "sha256:passport_hash_abc123...",
    "signature": "ed25519:signature_def456...",
    "previousHash": "sha256:previous_version...",
    "version": 3
  }
}
```

### 8.2 Valid Passport - Multiple Allergies

```json
{
  "$schema": "https://wia.live/food-allergy-passport/v1/schema.json",
  "version": "1.0.0",
  "passportId": "FAP-2025-000002",
  "status": "active",
  "created": "2025-01-15T11:00:00Z",
  "lastUpdated": "2025-01-15T11:00:00Z",
  "expiresAt": "2026-01-15T11:00:00Z",
  "holder": {
    "userId": "USER-2025-002",
    "displayName": "encrypted:aes256:..."
  },
  "allergies": [
    {
      "allergen": "shellfish",
      "allergenCode": "FDA-SHELLFISH",
      "icdCode": "Z91.014",
      "severity": "severe",
      "symptoms": [
        {"type": "skin", "description": "hives", "onsetTime": "immediate"},
        {"type": "gastrointestinal", "description": "vomiting", "onsetTime": "30 minutes"}
      ],
      "threshold": "any amount"
    },
    {
      "allergen": "tree nuts",
      "allergenCode": "FDA-TREENUT",
      "icdCode": "Z91.010",
      "severity": "moderate",
      "symptoms": [
        {"type": "skin", "description": "itching", "onsetTime": "immediate"}
      ],
      "threshold": "small amounts"
    }
  ],
  "medications": {
    "antihistamines": [
      {
        "brand": "Zyrtec",
        "genericName": "cetirizine",
        "dosage": "10mg",
        "route": "oral"
      }
    ]
  },
  "translations": {
    "languages": ["en", "ko"],
    "primaryLanguage": "en"
  },
  "verification": {
    "medicallyVerified": false
  },
  "meta": {
    "hash": "sha256:...",
    "version": 1
  }
}
```

### 8.3 Invalid Example - Missing EpiPen for Anaphylaxis

```json
{
  "version": "1.0.0",
  "passportId": "FAP-2025-000003",
  "status": "active",
  "holder": {
    "userId": "USER-2025-003"
  },
  "allergies": [
    {
      "allergen": "peanuts",
      "severity": "anaphylaxis"
    }
  ],
  "medications": {}
}
```

**Error**: `ERR_MISSING_EPIPEN` - EpiPen required for anaphylaxis-level allergies

### 8.4 Invalid Example - Unknown Allergen Code

```json
{
  "version": "1.0.0",
  "passportId": "FAP-2025-000004",
  "allergies": [
    {
      "allergen": "strawberries",
      "allergenCode": "FDA-STRAWBERRY",
      "severity": "mild"
    }
  ]
}
```

**Error**: `ERR_INVALID_ALLERGEN` - FDA-STRAWBERRY is not a recognized FDA major allergen code

### 8.5 Invalid Example - Expired Passport

```json
{
  "version": "1.0.0",
  "passportId": "FAP-2023-000005",
  "status": "active",
  "created": "2023-01-15T10:00:00Z",
  "expiresAt": "2024-01-15T10:00:00Z",
  "allergies": [
    {
      "allergen": "milk",
      "severity": "moderate"
    }
  ]
}
```

**Error**: `ERR_EXPIRED_PASSPORT` - Passport expired on 2024-01-15, renewal required

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01 | Initial release |

---

<div align="center">

**WIA Food Allergy Passport Standard v1.0.0**

**弘益人間 (홍익인간)** - Benefit All Humanity

---

**© 2025 WIA**

**MIT License**

</div>
