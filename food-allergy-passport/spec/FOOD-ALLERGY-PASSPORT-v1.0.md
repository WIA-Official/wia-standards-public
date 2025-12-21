# WIA-FOOD-ALLERGY-PASSPORT Specification v1.0

## Document Information

- **Standard**: WIA-FOOD-ALLERGY-PASSPORT
- **Version**: 1.0
- **Status**: Draft
- **Created**: 2025-12
- **Category**: Health & Safety

---

## 1. Introduction

### 1.1 Purpose

WIA-FOOD-ALLERGY-PASSPORT defines a universal standard for digital food allergy documentation that enables safe dining experiences worldwide, regardless of language or location.

### 1.2 Scope

This standard covers:
- Allergy passport data format
- Severity classification system
- Multi-language representation
- Restaurant and airline integration
- Emergency protocol communication
- Medical verification workflow

### 1.3 Problem Statement

Current challenges in food allergy communication:
- Language barriers when traveling
- Inconsistent allergy documentation
- No standard format for restaurants
- Difficulty communicating severity
- Emergency responders lack quick reference
- Cross-contamination risks poorly communicated

---

## 2. Philosophy

### 2.1 Hongik Ingan Principles

"Benefit all humanity" - Food safety is a universal right.

Core beliefs:
- Everyone deserves safe dining experiences
- Allergies should not limit travel or exploration
- Clear communication saves lives
- Technology can bridge language barriers
- Privacy and safety can coexist

### 2.2 Design Goals

1. **Universal Understanding**: Works across all languages and cultures
2. **Life-Saving Clarity**: Severity immediately apparent
3. **Privacy Respecting**: Share only what's necessary
4. **Easily Scannable**: QR code for instant access
5. **Medically Accurate**: Physician-verified information

---

## 3. Allergy Classification

### 3.1 Severity Levels

```
Level     Code    Description                      Symbol
--------- ------- -------------------------------- ------
SEVERE    SEV     Life-threatening, anaphylaxis    [!!!]
HIGH      HI      Serious, requires medication     [!!]
MEDIUM    MED     Significant symptoms             [!]
LOW       LO      Mild reaction/intolerance        [i]
```

### 3.2 Standard Allergen Categories

```
Category          Code    Common Allergens
----------------- ------- ---------------------------------
TREE_NUTS         TN      almonds, cashews, walnuts, pecans
PEANUTS           PN      peanuts, groundnuts
SHELLFISH         SF      shrimp, crab, lobster, oyster
FISH              FI      salmon, tuna, cod, anchovy
DAIRY             DA      milk, cheese, butter, yogurt
EGGS              EG      eggs, mayonnaise, meringue
WHEAT             WH      wheat, bread, pasta, flour
SOY               SO      soy, tofu, soy sauce, edamame
SESAME            SE      sesame seeds, tahini, sesame oil
MUSTARD           MU      mustard, mustard oil
CELERY            CE      celery, celeriac
LUPIN             LU      lupin flour, lupin seeds
MOLLUSKS          MO      squid, octopus, snails
SULFITES          SU      wine, dried fruits, preservatives
OTHER             OT      custom allergen specification
```

### 3.3 Reaction Types

```
Reaction Code     Description
----------------- ---------------------------------
ANAPHYLAXIS       Severe systemic allergic reaction
RESPIRATORY       Breathing difficulty, wheezing
CARDIOVASCULAR    Drop in blood pressure, dizziness
SKIN              Hives, rash, itching, swelling
GASTROINTESTINAL  Vomiting, diarrhea, abdominal pain
ORAL              Tingling, swelling of lips/tongue
NEUROLOGICAL      Confusion, anxiety, sense of doom
```

---

## 4. Passport Format

### 4.1 Core Data Structure

```json
{
  "wia_type": "FOOD-ALLERGY-PASSPORT",
  "wia_version": "1.0",
  "passport_id": "FAP-{COUNTRY}-{YEAR}-{SEQUENCE}",

  "holder": {
    "name": "string",
    "date_of_birth": "YYYY-MM-DD",
    "photo_hash": "sha256:...",
    "blood_type": "A+|A-|B+|B-|AB+|AB-|O+|O-",
    "emergency_contacts": [
      {
        "name": "string",
        "relationship": "string",
        "phone": "string",
        "email": "string"
      }
    ]
  },

  "allergies": [
    {
      "allergen_code": "string",
      "allergen_name": "string",
      "custom_name": "string (if OTHER)",
      "severity": "SEV|HI|MED|LO",
      "reaction_types": ["string"],
      "reaction_description": "string",
      "onset_time": "immediate|minutes|hours",
      "medications": [
        {
          "name": "string",
          "dosage": "string",
          "administration": "string"
        }
      ],
      "notes": "string"
    }
  ],

  "dietary_restrictions": {
    "vegetarian": "boolean",
    "vegan": "boolean",
    "halal": "boolean",
    "kosher": "boolean",
    "other": ["string"]
  },

  "verification": {
    "verified": "boolean",
    "verified_by": "string",
    "institution": "string",
    "date": "YYYY-MM-DD",
    "certificate_id": "string",
    "signature": "base64"
  },

  "metadata": {
    "created": "ISO8601",
    "updated": "ISO8601",
    "valid_until": "YYYY-MM-DD",
    "issuing_country": "ISO3166-1 alpha-2",
    "language": "ISO639-1"
  }
}
```

### 4.2 Compact QR Format

For QR code generation, use compact representation:

```
FAP1|KR|2025|12345|Kim Min-Su|PN:SEV:ANAPHYLAXIS:EPI|SF:HI:SKIN:ANTIHISTAMINE|+821012345678
```

Format: `FAP{version}|{country}|{year}|{id}|{name}|{allergy1}|{allergy2}...|{emergency_phone}`

Allergy encoding: `{allergen_code}:{severity}:{reaction}:{medication}`

### 4.3 Emergency Card Format

```
+--------------------------------------------------+
|  [!!!] FOOD ALLERGY ALERT                        |
+--------------------------------------------------+
|  Name: Kim Min-Su                                |
|  DOB: 1990-03-15          Blood: A+              |
+--------------------------------------------------+
|  SEVERE ALLERGIES:                               |
|  [!!!] PEANUTS - Anaphylaxis                     |
|        EpiPen required immediately               |
+--------------------------------------------------+
|  HIGH ALLERGIES:                                 |
|  [!!] SHELLFISH - Hives, swelling                |
|       Antihistamine (cetirizine 10mg)            |
+--------------------------------------------------+
|  Emergency: +82-10-1234-5678 (Spouse)            |
|  Verified: Seoul National University Hospital    |
+--------------------------------------------------+
|  [QR CODE]           FAP-KR-2025-12345           |
+--------------------------------------------------+
```

---

## 5. Multi-Language Support

### 5.1 Translation Structure

```json
{
  "translations": {
    "en": {
      "allergen_peanut": "Peanut",
      "severity_severe": "SEVERE - Life Threatening",
      "alert_contains": "Contains",
      "alert_may_contain": "May contain",
      "emergency_call": "Call emergency services"
    },
    "ko": {
      "allergen_peanut": "땅콩",
      "severity_severe": "심각 - 생명 위험",
      "alert_contains": "포함",
      "alert_may_contain": "포함 가능",
      "emergency_call": "응급 서비스 호출"
    },
    "ja": {
      "allergen_peanut": "ピーナッツ",
      "severity_severe": "重度 - 生命の危険",
      "alert_contains": "含む",
      "alert_may_contain": "含む可能性",
      "emergency_call": "救急サービスに電話"
    }
  }
}
```

### 5.2 Restaurant Display Languages

Minimum required translations:
- Local language
- English
- Passport holder's native language

### 5.3 Universal Symbols

Visual symbols for non-readers:

```
[!!!] Red exclamation - Severe allergy
[!!]  Orange exclamation - High allergy
[X]   Crossed out allergen - Must avoid
[?]   Question mark - Check with staff
[OK]  Green check - Safe to consume
```

---

## 6. Restaurant Integration

### 6.1 Workflow

```
1. Guest scans QR at entrance/table
2. System registers allergies for table
3. Kitchen receives allergy alert
4. Menu items flagged for allergens
5. Order placed with allergy check
6. Kitchen prepares with precautions
7. Server confirms safe delivery
```

### 6.2 Kitchen Display Protocol

```
+------------------------------------------+
| TABLE 5 - ALLERGY ALERT                  |
+------------------------------------------+
| Guest: Kim (FAP-KR-2025-12345)           |
|                                          |
| [!!!] PEANUTS                            |
|     - Use separate preparation area      |
|     - New utensils required              |
|     - No peanut oil                      |
|                                          |
| [!!] SHELLFISH                           |
|     - Avoid cross-contamination          |
|                                          |
| ORDER: Bibimbap (modified)               |
|        Remove: peanuts, shrimp           |
|        Confirm before serving            |
+------------------------------------------+
```

### 6.3 POS Integration API

```javascript
// Register guest allergies
POST /api/v1/guest/allergies
{
  "table_id": "5",
  "passport_data": {/* QR scan result */},
  "session_id": "uuid"
}

// Check menu item
GET /api/v1/menu/{item_id}/allergy-check?passport_id=FAP-KR-2025-12345

Response:
{
  "item_id": "bibimbap-001",
  "safe": false,
  "conflicts": [
    {
      "allergen": "PN",
      "ingredient": "ground peanuts",
      "severity": "SEV",
      "removable": true
    }
  ],
  "modifications_possible": true,
  "safe_alternatives": ["vegetable-bibimbap-002"]
}
```

---

## 7. Airline Integration

### 7.1 Booking Flow

```
1. Passenger adds passport to booking
2. System flags special meal requirement
3. Pre-departure meal confirmation
4. Crew briefed on allergies
5. In-flight announcement if needed
6. Alternative meals prepared
```

### 7.2 Airline Data Format

```json
{
  "pnr": "ABC123",
  "passenger": {
    "name": "Kim Min-Su",
    "seat": "23A"
  },
  "allergy_passport": {
    "passport_id": "FAP-KR-2025-12345",
    "severe_allergies": ["PN"],
    "high_allergies": ["SF"],
    "special_meal_code": "NLML",
    "medication_carried": true,
    "cabin_announcement": false
  }
}
```

### 7.3 Crew Alert Card

```
+------------------------------------------+
| PASSENGER ALLERGY ALERT                  |
+------------------------------------------+
| Seat: 23A    Name: Kim Min-Su            |
|                                          |
| [!!!] SEVERE: PEANUTS                    |
|     Passenger carries EpiPen             |
|     DO NOT serve any peanut products     |
|     Avoid opening peanut snacks nearby   |
|                                          |
| [!!] HIGH: SHELLFISH                     |
|     Passenger carries antihistamine      |
|                                          |
| Special Meal: NLML (confirmed)           |
| Emergency Contact: +82-10-1234-5678      |
+------------------------------------------+
```

---

## 8. Emergency Protocol

### 8.1 Reaction Response Levels

```
Level 1 - MILD (LO severity reaction)
  - Monitor symptoms
  - Offer water
  - Advise rest
  - Document incident

Level 2 - MODERATE (MED severity reaction)
  - Administer antihistamine if available
  - Contact emergency contact
  - Prepare for escalation
  - Stay with patient

Level 3 - SEVERE (HI severity reaction)
  - Call emergency services
  - Administer medication (EpiPen if available)
  - Keep patient still
  - Monitor breathing

Level 4 - CRITICAL (SEV/anaphylaxis)
  - IMMEDIATE: Call emergency services
  - Administer EpiPen (thigh)
  - Lay patient flat, elevate legs
  - Prepare for CPR if needed
  - Second EpiPen after 5 mins if no improvement
```

### 8.2 First Responder Quick Reference

```json
{
  "emergency_format": {
    "patient": "Kim Min-Su",
    "known_allergies": [
      {
        "allergen": "PEANUTS",
        "severity": "SEVERE",
        "typical_reaction": "Anaphylaxis within 5 minutes",
        "medication": "EpiPen 0.3mg IM"
      }
    ],
    "blood_type": "A+",
    "other_conditions": [],
    "current_medications": [],
    "emergency_contact": "+82-10-1234-5678"
  }
}
```

### 8.3 Hospital Integration

FHIR-compatible allergy resource:

```json
{
  "resourceType": "AllergyIntolerance",
  "clinicalStatus": {
    "coding": [{
      "system": "http://terminology.hl7.org/CodeSystem/allergyintolerance-clinical",
      "code": "active"
    }]
  },
  "verificationStatus": {
    "coding": [{
      "system": "http://terminology.hl7.org/CodeSystem/allergyintolerance-verification",
      "code": "confirmed"
    }]
  },
  "category": ["food"],
  "criticality": "high",
  "code": {
    "coding": [{
      "system": "http://snomed.info/sct",
      "code": "91935009",
      "display": "Allergy to peanut"
    }]
  },
  "patient": {
    "reference": "Patient/kim-min-su-12345"
  }
}
```

---

## 9. Privacy & Security

### 9.1 Data Sharing Levels

```
Level       Visible Information
----------- --------------------------------------------------
MINIMAL     Allergens and severity only (no personal info)
STANDARD    + Name, emergency contact
MEDICAL     + DOB, blood type, medications, medical history
FULL        Complete passport data
```

### 9.2 Consent Model

```json
{
  "consent": {
    "restaurants": "STANDARD",
    "airlines": "MEDICAL",
    "emergency_services": "FULL",
    "default": "MINIMAL"
  },
  "temporary_shares": [
    {
      "recipient": "restaurant-uuid",
      "level": "STANDARD",
      "expires": "2025-12-21T23:59:59Z"
    }
  ]
}
```

### 9.3 Security Requirements

- End-to-end encryption for full data
- QR codes contain only essential info
- Verification signatures non-forgeable
- Audit log for all data access
- Right to erasure supported

---

## 10. API Reference

### 10.1 Core Endpoints

```
Passport Management
-------------------
POST   /api/v1/passport                Create new passport
GET    /api/v1/passport/{id}           Retrieve passport
PUT    /api/v1/passport/{id}           Update passport
DELETE /api/v1/passport/{id}           Delete passport

Verification
------------
POST   /api/v1/passport/{id}/verify    Submit for verification
GET    /api/v1/passport/{id}/status    Check verification status

Sharing
-------
POST   /api/v1/passport/{id}/share     Create temporary share
GET    /api/v1/share/{token}           Access shared passport
DELETE /api/v1/share/{token}           Revoke share

Translation
-----------
GET    /api/v1/passport/{id}/translate?lang=ja
                                       Get translated view
```

### 10.2 SDK Example

```javascript
import { AllergyPassport } from '@anthropic/wia-food-allergy-passport';

// Create passport
const passport = new AllergyPassport({
  holder: {
    name: 'Kim Min-Su',
    dateOfBirth: '1990-03-15',
    bloodType: 'A+',
    emergencyContacts: [{
      name: 'Kim Young-Hee',
      relationship: 'spouse',
      phone: '+82-10-1234-5678'
    }]
  }
});

// Add allergies
passport.addAllergy({
  allergen: 'PEANUTS',
  severity: 'SEVERE',
  reactions: ['ANAPHYLAXIS'],
  medications: [{
    name: 'EpiPen',
    dosage: '0.3mg',
    administration: 'intramuscular'
  }]
});

// Generate QR code
const qr = passport.generateQR({ format: 'svg' });

// Share with restaurant
const shareToken = await passport.createShare({
  recipient: 'restaurant-uuid',
  level: 'STANDARD',
  duration: '4h'
});

// Get translated card
const japaneseCard = passport.translate('ja');
```

---

## 11. Interoperability

### 11.1 With WIA-PET-HEALTH-PASSPORT

- Similar structure for pet allergies
- Shared allergen database
- Combined family allergy profile

### 11.2 With WIA-MEDICINE-VERIFY

- Medication verification for allergy meds
- Drug interaction checks
- Prescription validation

### 11.3 With WIA-ELDER-CARE

- Dietary restriction integration
- Caregiver allergy alerts
- Meal planning integration

### 11.4 External Standards

- FHIR AllergyIntolerance resource
- HL7 ADT messages
- FDA allergen codes
- WHO ICD-11 classifications

---

## 12. Implementation Levels

### 12.1 Level 1: Basic

```
Required:
- Passport creation and storage
- QR code generation
- Basic allergen list
- Single language support

Recommended:
- Emergency card format
- Mobile app
```

### 12.2 Level 2: Standard

```
Required:
- Level 1 features
- Multi-language support (3+)
- Restaurant integration API
- Sharing mechanism
- Verification workflow

Recommended:
- Airline integration
- Emergency protocol
```

### 12.3 Level 3: Complete

```
Required:
- Level 2 features
- Full language support (50+)
- FHIR compatibility
- Real-time translation
- Emergency services integration
- Analytics and reporting

Recommended:
- AI ingredient analysis
- Cross-contamination tracking
```

---

## 13. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2025-12 | Initial release |

---

WIA-FOOD-ALLERGY-PASSPORT: Safe dining worldwide, one scan at a time.
