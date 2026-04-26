# WIA-FOOD-ALLERGY-PASSPORT

A standardized digital passport for food allergies enabling safe dining worldwide.

## Overview

WIA-FOOD-ALLERGY-PASSPORT provides a universal format for declaring food allergies that can be understood by restaurants, airlines, hotels, and emergency services across language barriers.

## Philosophy: Hongik Ingan

"Benefit all humanity" - Safe eating is a fundamental right. This standard ensures everyone can dine safely regardless of where they are in the world.

## Directory Structure

```
food-allergy-passport/
+-- README.md                    # This file
+-- spec/
|   +-- FOOD-ALLERGY-PASSPORT-v1.0.md  # Technical specification
+-- simulator/
|   +-- index.html               # Interactive passport creator
+-- ebook/
    +-- en/
    |   +-- index.html           # English ebook
    |   +-- chapter-01.html      # Introduction
    |   +-- chapter-02.html      # Allergy Classification
    |   +-- chapter-03.html      # Passport Format
    |   +-- chapter-04.html      # Restaurant Integration
    |   +-- chapter-05.html      # Travel & Airlines
    |   +-- chapter-06.html      # Emergency Protocol
    |   +-- chapter-07.html      # Multi-Language Support
    |   +-- chapter-08.html      # Implementation
    +-- ko/
        +-- index.html           # Korean ebook
        +-- chapter-01.html ~ chapter-08.html
```

## Key Features

1. **Universal Format**: Standard JSON/QR format understood globally
2. **Severity Levels**: Clear indication of reaction severity
3. **Multi-Language**: Automatic translation to 50+ languages
4. **Restaurant Integration**: Kitchen display and ordering system integration
5. **Airline Support**: Pre-flight meal requests and in-flight alerts
6. **Emergency Info**: First responder quick reference format
7. **Medical Verification**: Physician-verified allergy records
8. **Privacy Control**: User controls what information is shared

## Allergy Categories

```
[SEVERE] Life-threatening (anaphylaxis risk)
[HIGH]   Serious reaction requiring medication
[MEDIUM] Significant discomfort or symptoms
[LOW]    Mild reaction or intolerance
```

## Quick Example

```json
{
  "wia_version": "1.0",
  "passport_id": "FAP-KR-2025-12345",
  "holder": {
    "name": "Kim Min-Su",
    "emergency_contact": "+82-10-1234-5678"
  },
  "allergies": [
    {
      "allergen": "peanut",
      "severity": "SEVERE",
      "reaction": "anaphylaxis",
      "medication": "epinephrine auto-injector"
    },
    {
      "allergen": "shellfish",
      "severity": "HIGH",
      "reaction": "hives, swelling",
      "medication": "antihistamine"
    }
  ],
  "verified_by": "Seoul National University Hospital",
  "valid_until": "2026-12-31"
}
```

## Integration Points

- Restaurant POS systems
- Airline booking systems
- Hotel guest management
- Food delivery apps
- Emergency medical services
- Healthcare records (FHIR compatible)

## Simulator

The interactive simulator allows you to:
- Create your allergy passport
- Generate QR codes
- Test restaurant scenario
- Practice emergency protocol
- Export in multiple formats

## Related Standards

- WIA-PET-HEALTH-PASSPORT: Similar passport for pet health
- WIA-MEDICINE-VERIFY: Medication verification
- WIA-ELDER-CARE: Elder care including dietary needs

## License

Creative Commons Attribution 4.0 International (CC BY 4.0)

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2025-12 | Initial release |

---

**홍익인간 (弘益人間)** - Benefit All Humanity 🌍
