# WIA-ENE-048: Plastic Alternative - Phase 2 Data Format

## Overview

**Standard ID:** WIA-ENE-048
**Category:** Energy & Environment (ENE)
**Phase:** 2 - Data Format
**Version:** 1.0

This document defines standardized data formats for plastic alternative materials, enabling interoperability between manufacturers, testing facilities, certifiers, and end users.

## Material Data Schema

### Core Material Object

```json
{
  "materialId": "string (unique identifier)",
  "standard": "WIA-ENE-048",
  "version": "1.0",
  "materialName": "string",
  "materialType": "enum [pla, pha, pbs, cellulose, seaweed, mushroom, starch, other]",
  "category": "enum [A, B, C, D]",
  "certificationLevel": "enum [basic, advanced, premium, marine]",
  "manufacturer": {
    "name": "string",
    "did": "string (DID identifier)",
    "location": "string (ISO country code)",
    "contact": "string (email)"
  },
  "composition": {
    "bioBasedContent": "number (0-100, percentage)",
    "components": [
      {
        "name": "string",
        "percentage": "number (0-100)",
        "source": "string",
        "renewable": "boolean"
      }
    ]
  },
  "properties": {
    "mechanical": {},
    "thermal": {},
    "chemical": {},
    "environmental": {}
  },
  "testing": {},
  "certifications": [],
  "applications": [],
  "endOfLife": {},
  "metadata": {}
}
```

## Detailed Property Specifications

### Mechanical Properties

```json
{
  "mechanical": {
    "tensileStrength": {
      "value": "number (MPa)",
      "method": "ISO 527 or ASTM D638",
      "testDate": "ISO 8601 datetime"
    },
    "elongationAtBreak": {
      "value": "number (percentage)",
      "method": "ISO 527 or ASTM D638"
    },
    "flexuralModulus": {
      "value": "number (MPa)",
      "method": "ISO 178 or ASTM D790"
    },
    "impactStrength": {
      "value": "number (kJ/m²)",
      "method": "ISO 179 or ASTM D256"
    },
    "hardness": {
      "value": "number",
      "scale": "Shore A/D",
      "method": "ISO 868"
    }
  }
}
```

### Thermal Properties

```json
{
  "thermal": {
    "meltingPoint": {
      "value": "number (°C)",
      "method": "DSC - ISO 11357"
    },
    "glassTran sitionTemp": {
      "value": "number (°C)",
      "method": "DSC - ISO 11357"
    },
    "heatDeflectionTemp": {
      "value": "number (°C)",
      "method": "ISO 75 or ASTM D648",
      "load": "0.45 or 1.8 MPa"
    },
    "thermalConductivity": {
      "value": "number (W/m·K)",
      "method": "ISO 8302"
    },
    "coefficientOfExpansion": {
      "value": "number (10⁻⁵/K)",
      "method": "ISO 11359"
    }
  }
}
```

### Chemical Properties

```json
{
  "chemical": {
    "waterAbsorption": {
      "value": "number (percentage)",
      "duration": "24 hours",
      "method": "ISO 62 or ASTM D570"
    },
    "chemicalResistance": {
      "acids": "enum [excellent, good, fair, poor]",
      "bases": "enum [excellent, good, fair, poor]",
      "solvents": "enum [excellent, good, fair, poor]",
      "oils": "enum [excellent, good, fair, poor]"
    },
    "oxygenPermeability": {
      "value": "number (cc·mm/m²·day·atm)",
      "method": "ASTM D3985"
    },
    "waterVaporTransmission": {
      "value": "number (g/m²·day)",
      "method": "ASTM F1249"
    }
  }
}
```

### Environmental Properties

```json
{
  "environmental": {
    "biodegradation": {
      "industrial": {
        "percentage": "number (0-100)",
        "duration": "number (days)",
        "method": "ISO 14855 or ASTM D5338",
        "testDate": "ISO 8601 datetime",
        "labName": "string"
      },
      "home": {
        "percentage": "number (0-100)",
        "duration": "number (days)",
        "method": "ISO 14855-1"
      },
      "soil": {
        "percentage": "number (0-100)",
        "duration": "number (days)",
        "method": "ISO 17556"
      },
      "marine": {
        "percentage": "number (0-100)",
        "duration": "number (days)",
        "method": "ASTM D7081"
      }
    },
    "compostability": {
      "certified": "boolean",
      "standard": "ASTM D6400 or EN 13432",
      "certificateId": "string",
      "issuer": "string",
      "issueDate": "ISO 8601 date",
      "expiryDate": "ISO 8601 date"
    },
    "carbonFootprint": {
      "value": "number (kg CO2 eq per kg material)",
      "scope": "cradle-to-grave or cradle-to-gate",
      "method": "ISO 14067",
      "reportUrl": "string (URL)"
    }
  }
}
```

## Testing Data Schema

### Test Results Object

```json
{
  "testing": {
    "bioBasedContent": {
      "result": "number (percentage)",
      "method": "ASTM D6866",
      "uncertainty": "number (±%)",
      "testDate": "ISO 8601 datetime",
      "laboratory": {
        "name": "string",
        "accreditation": "ISO/IEC 17025",
        "location": "string"
      },
      "reportId": "string",
      "reportUrl": "string (URL)"
    },
    "biodegradation": {
      "testType": "enum [industrial, home, soil, marine]",
      "result": "number (% conversion to CO2)",
      "duration": "number (days)",
      "temperature": "number (°C)",
      "method": "string (ISO/ASTM standard)",
      "passedCriteria": "boolean",
      "testDate": "ISO 8601 datetime",
      "laboratory": {},
      "reportId": "string"
    },
    "ecotoxicity": {
      "plantToxicity": {
        "species": "string",
        "result": "enum [passed, failed]",
        "method": "ISO 20200",
        "details": "string"
      },
      "earthwormToxicity": {
        "species": "Eisenia fetida",
        "result": "enum [passed, failed]",
        "method": "ISO 20200"
      },
      "aquaticToxicity": {
        "species": "string",
        "result": "enum [passed, failed]",
        "method": "ISO 14669"
      }
    },
    "heavyMetals": {
      "totalConcentration": "number (ppm)",
      "passedCriteria": "boolean (<100 ppm)",
      "method": "EN 13432",
      "elements": [
        {
          "symbol": "string (Pb, Cd, Hg, etc.)",
          "concentration": "number (ppm)",
          "limit": "number (ppm)"
        }
      ]
    }
  }
}
```

## Certification Data Schema

```json
{
  "certifications": [
    {
      "certificateId": "string (CERT-ENE048-XXXXX)",
      "standard": "WIA-ENE-048",
      "level": "enum [basic, advanced, premium, marine]",
      "status": "enum [pending, active, expired, revoked]",
      "issuer": {
        "name": "WIA Certification Authority",
        "did": "string",
        "accreditation": "string"
      },
      "issueDate": "ISO 8601 date",
      "expiryDate": "ISO 8601 date",
      "scope": "string (description of certification scope)",
      "conditions": ["string (any special conditions)"],
      "verifiableCredential": {
        "id": "string (VC DID)",
        "type": ["VerifiableCredential", "PlasticAlternativeCertificate"],
        "proof": {
          "type": "Ed25519Signature2020",
          "proofValue": "string"
        }
      },
      "blockchainAnchor": {
        "network": "string (Ethereum, Polygon, etc.)",
        "txHash": "string",
        "blockNumber": "number",
        "timestamp": "ISO 8601 datetime"
      }
    }
  ]
}
```

## Application Data

```json
{
  "applications": [
    {
      "category": "enum [packaging, agriculture, medical, consumer_goods, construction, other]",
      "specificUse": "string",
      "performanceNotes": "string",
      "regulatoryStatus": {
        "fdaApproved": "boolean",
        "euCompliant": "boolean",
        "otherRegulations": ["string"]
      },
      "caseStu dies": [
        {
          "company": "string",
          "product": "string",
          "outcome": "string",
          "url": "string"
        }
      ]
    }
  ]
}
```

## End-of-Life Data

```json
{
  "endOfLife": {
    "disposalInstructions": {
      "primary": "enum [industrial_composting, home_composting, recycling, anaerobic_digestion]",
      "secondary": ["enum options"],
      "notRecommended": ["enum [landfill, incineration, ocean]"],
      "instructionsUrl": "string"
    },
    "labelingRequirements": {
      "compostableLogo": "boolean",
      "certificationMark": "WIA-ENE-048",
      "qrCode": "string (URL to verification)",
      "disposalSymbol": "string (unicode or image URL)"
    },
    "facilityCompatibility": {
      "industrialComposting": "boolean",
      "homeComposting": "boolean",
      "anaerobicDigestion": "boolean",
      "mechanicalRecycling": "boolean"
    },
    "degradationConditions": {
      "optimalTemperature": "number (°C)",
      "optimalMoisture": "number (percentage)",
      "optimalPH": "number (pH range)",
      "typicalTimeline": "number (days)"
    }
  }
}
```

## Metadata

```json
{
  "metadata": {
    "createdDate": "ISO 8601 datetime",
    "lastUpdated": "ISO 8601 datetime",
    "version": "string (semantic versioning)",
    "dataSource": "string (origin of data)",
    "verificationStatus": "enum [unverified, third_party_verified, wia_certified]",
    "publiclyAvailable": "boolean",
    "licenseType": "enum [open, restricted, proprietary]",
    "citations": [
      {
        "type": "enum [research_paper, technical_report, certification]",
        "title": "string",
        "authors": ["string"],
        "doi": "string",
        "url": "string"
      }
    ],
    "tags": ["string (searchable keywords)"],
    "language": "string (ISO 639-1 code)"
  }
}
```

## Real-World Example: PLA Material

```json
{
  "materialId": "PLA-2025-001",
  "standard": "WIA-ENE-048",
  "version": "1.0",
  "materialName": "EcoPLA-Premium",
  "materialType": "pla",
  "category": "A",
  "certificationLevel": "advanced",
  "manufacturer": {
    "name": "GreenPolymer Industries",
    "did": "did:wia:manufacturer:12345",
    "location": "US",
    "contact": "info@greenpolymer.example"
  },
  "composition": {
    "bioBasedContent": 98,
    "components": [
      {
        "name": "Polylactic Acid",
        "percentage": 98,
        "source": "Corn starch (non-GMO)",
        "renewable": true
      },
      {
        "name": "Nucleating agent",
        "percentage": 1.5,
        "source": "Mineral-based",
        "renewable": false
      },
      {
        "name": "Plasticizer",
        "percentage": 0.5,
        "source": "Bio-based glycerol",
        "renewable": true
      }
    ]
  },
  "properties": {
    "mechanical": {
      "tensileStrength": {
        "value": 65,
        "method": "ISO 527-2",
        "testDate": "2025-01-10T10:00:00Z"
      },
      "elongationAtBreak": {
        "value": 3.5,
        "method": "ISO 527-2"
      },
      "flexuralModulus": {
        "value": 3500,
        "method": "ISO 178"
      }
    },
    "thermal": {
      "meltingPoint": {
        "value": 165,
        "method": "DSC - ISO 11357"
      },
      "glassTransitionTemp": {
        "value": 58,
        "method": "DSC - ISO 11357"
      },
      "heatDeflectionTemp": {
        "value": 55,
        "method": "ISO 75",
        "load": "0.45 MPa"
      }
    },
    "chemical": {
      "waterAbsorption": {
        "value": 0.5,
        "duration": "24 hours",
        "method": "ISO 62"
      },
      "chemicalResistance": {
        "acids": "good",
        "bases": "fair",
        "solvents": "poor",
        "oils": "excellent"
      }
    },
    "environmental": {
      "biodegradation": {
        "industrial": {
          "percentage": 94,
          "duration": 180,
          "method": "ISO 14855-1",
          "testDate": "2024-12-15T00:00:00Z",
          "labName": "BioTest Laboratories Inc."
        }
      },
      "compostability": {
        "certified": true,
        "standard": "ASTM D6400",
        "certificateId": "BPI-12345",
        "issuer": "Biodegradable Products Institute",
        "issueDate": "2024-11-01",
        "expiryDate": "2025-11-01"
      },
      "carbonFootprint": {
        "value": 1.8,
        "scope": "cradle-to-gate",
        "method": "ISO 14067",
        "reportUrl": "https://greenpolymer.example/lca-report-pla-001"
      }
    }
  },
  "testing": {
    "bioBasedContent": {
      "result": 98,
      "method": "ASTM D6866",
      "uncertainty": 2,
      "testDate": "2024-12-01T00:00:00Z",
      "laboratory": {
        "name": "Beta Analytic",
        "accreditation": "ISO/IEC 17025",
        "location": "Miami, FL, USA"
      },
      "reportId": "BA-2024-12345"
    },
    "ecotoxicity": {
      "plantToxicity": {
        "species": "Lepidium sativum (cress)",
        "result": "passed",
        "method": "ISO 20200"
      },
      "earthwormToxicity": {
        "species": "Eisenia fetida",
        "result": "passed",
        "method": "ISO 20200"
      }
    },
    "heavyMetals": {
      "totalConcentration": 12,
      "passedCriteria": true,
      "method": "EN 13432",
      "elements": [
        {"symbol": "Pb", "concentration": 3, "limit": 50},
        {"symbol": "Cd", "concentration": 0.5, "limit": 0.5},
        {"symbol": "Hg", "concentration": 0.1, "limit": 0.5},
        {"symbol": "Cr", "concentration": 8, "limit": 50}
      ]
    }
  },
  "certifications": [
    {
      "certificateId": "CERT-ENE048-00123",
      "standard": "WIA-ENE-048",
      "level": "advanced",
      "status": "active",
      "issuer": {
        "name": "WIA Certification Authority",
        "did": "did:wia:certifier:001"
      },
      "issueDate": "2025-01-15",
      "expiryDate": "2026-01-15",
      "verifiableCredential": {
        "id": "did:wia:credential:ENE048-00123",
        "type": ["VerifiableCredential", "PlasticAlternativeCertificate"]
      },
      "blockchainAnchor": {
        "network": "Ethereum",
        "txHash": "0x1a2b3c4d5e6f...",
        "blockNumber": 18234567,
        "timestamp": "2025-01-15T14:30:00Z"
      }
    }
  ],
  "applications": [
    {
      "category": "packaging",
      "specificUse": "Food containers, cold beverage cups",
      "performanceNotes": "Suitable for refrigerated foods, not for hot liquids above 60°C",
      "regulatoryStatus": {
        "fdaApproved": true,
        "euCompliant": true,
        "otherRegulations": ["FDA FCN 178", "EU 10/2011"]
      }
    }
  ],
  "endOfLife": {
    "disposalInstructions": {
      "primary": "industrial_composting",
      "secondary": ["anaerobic_digestion"],
      "notRecommended": ["landfill", "ocean"]
    },
    "labelingRequirements": {
      "compostableLogo": true,
      "certificationMark": "WIA-ENE-048-ADVANCED",
      "qrCode": "https://verify.wia.org/ENE048/CERT-ENE048-00123"
    },
    "facilityCompatibility": {
      "industrialComposting": true,
      "homeComposting": false,
      "anaerobicDigestion": true,
      "mechanicalRecycling": false
    },
    "degradationConditions": {
      "optimalTemperature": 58,
      "optimalMoisture": 60,
      "optimalPH": 7.5,
      "typicalTimeline": 180
    }
  },
  "metadata": {
    "createdDate": "2025-01-10T00:00:00Z",
    "lastUpdated": "2025-01-15T12:00:00Z",
    "version": "1.2.0",
    "dataSource": "Manufacturer + third-party testing",
    "verificationStatus": "wia_certified",
    "publiclyAvailable": true,
    "licenseType": "open",
    "tags": ["pla", "compostable", "food-contact", "bioplastic"],
    "language": "en"
  }
}
```

## API Endpoints

### Material Database API

```
GET    /api/v1/materials              - List all materials
GET    /api/v1/materials/{id}         - Get specific material
POST   /api/v1/materials              - Submit new material (authenticated)
PUT    /api/v1/materials/{id}         - Update material (authenticated)
DELETE /api/v1/materials/{id}         - Remove material (authenticated)

GET    /api/v1/materials/search       - Search materials
  Query params: type, category, certLevel, bioContent, etc.

GET    /api/v1/materials/{id}/verify  - Verify certification
GET    /api/v1/materials/{id}/qr      - Generate QR code
```

### Testing Data API

```
POST   /api/v1/testing/submit         - Submit test results
GET    /api/v1/testing/{reportId}     - Get test report
GET    /api/v1/laboratories           - List approved labs
```

### Certification API

```
POST   /api/v1/certifications/apply   - Apply for certification
GET    /api/v1/certifications/{id}    - Get certificate details
POST   /api/v1/certifications/verify  - Verify certificate
GET    /api/v1/certifications/{id}/vc - Get Verifiable Credential
```

## Data Validation Rules

### Required Fields (Minimum)
- materialId, standard, version
- materialName, materialType, category
- manufacturer.name, manufacturer.location
- composition.bioBasedContent
- At least one testing.biodegradation result
- metadata.createdDate, metadata.version

### Data Constraints
- bioBasedContent: 0-100
- Biodegradation percentage: 0-100
- Dates: ISO 8601 format
- URLs: Valid HTTP/HTTPS
- DIDs: Valid W3C DID format

### Certification Requirements by Level

**Basic:**
- bioBasedContent ≥ 50%
- At least 1 biodegradation test passed
- Ecotoxicity passed
- Heavy metals passed

**Advanced:**
- bioBasedContent ≥ 80%
- Industrial composting ≥ 90% in 180 days
- Compostability certified (ASTM D6400 or EN 13432)
- Full mechanical testing

**Premium:**
- bioBasedContent ≥ 95%
- Multiple environment biodegradation
- Home compostable
- Carbon footprint < 2.0 kg CO2 eq/kg
- Supply chain transparency

**Marine:**
- ASTM D7081 passed (≥70% in 365 days)
- Marine ecotoxicity passed
- No microplastic formation

---

**Document Version:** 1.0
**Last Updated:** 2025-01-15

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
