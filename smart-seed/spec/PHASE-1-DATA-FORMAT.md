# WIA-AGRI-011: Smart Seed Standard
## Phase 1 - Data Format Specification

**Version:** 1.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-01-01

---

## 1. Overview

This specification defines standardized data formats for smart seed systems, variety registration, germination testing, seed certification, digital seed passports, and intellectual property protection.

### 1.1 Design Principles

- **Traceability**: Complete seed lifecycle tracking from breeding to farmer
- **Quality Assurance**: Standardized germination testing and purity verification
- **IP Protection**: Protect plant breeder rights and variety registration
- **Interoperability**: Compatible with ISTA, OECD Seed Schemes, and UPOV systems
- **Transparency**: Blockchain-based immutable seed history records

---

## 2. Core Data Structures

### 2.1 Seed Variety Entity

```json
{
  "varietyId": "string (UUID)",
  "varietyName": "string",
  "scientificName": "string (binomial nomenclature)",
  "commonName": "string",
  "cropType": "string (enum: grain, vegetable, fruit, oilseed, fiber)",
  "breeder": {
    "organizationName": "string",
    "breederName": "string",
    "country": "string (ISO 3166-1 alpha-2)",
    "did": "string (W3C DID)"
  },
  "parentLines": {
    "maternal": "string (variety ID)",
    "paternal": "string (variety ID)",
    "crossingDate": "string (ISO 8601)"
  },
  "characteristics": {
    "maturityDays": "number (days from planting to harvest)",
    "plantHeight": "number (cm)",
    "yieldPotential": "number (kg/ha)",
    "diseaseResistance": ["string (disease names)"],
    "climateAdaptation": ["string (climate zones)"],
    "specialTraits": ["string (drought tolerance, high protein, etc.)"]
  },
  "intellectualProperty": {
    "pvpNumber": "string (Plant Variety Protection number)",
    "patentNumber": "string (if applicable)",
    "upovRegistration": "string (UPOV registration ID)",
    "registrationDate": "string (ISO 8601)",
    "protectionExpiry": "string (ISO 8601)",
    "licenseType": "string (enum: open-source, proprietary, hybrid)"
  },
  "registrationAuthority": {
    "country": "string (ISO 3166-1 alpha-2)",
    "agency": "string",
    "registrationNumber": "string"
  }
}
```

### 2.2 Seed Lot Data

```json
{
  "lotId": "string (unique lot identifier)",
  "varietyId": "string (UUID, links to Seed Variety)",
  "productionInfo": {
    "producer": {
      "name": "string",
      "license": "string (seed production license)",
      "location": "string",
      "did": "string (W3C DID)"
    },
    "productionDate": "string (ISO 8601)",
    "harvestDate": "string (ISO 8601)",
    "processingDate": "string (ISO 8601)",
    "quantity": {
      "value": "number",
      "unit": "string (kg, ton, bags)"
    },
    "generation": "string (Foundation, Registered, Certified)",
    "fieldLocation": {
      "latitude": "number (decimal degrees)",
      "longitude": "number (decimal degrees)",
      "elevation": "number (meters)",
      "soilType": "string",
      "previousCrop": "string"
    }
  },
  "qualityMetrics": {
    "germinationRate": "number (percentage 0-100)",
    "purity": "number (percentage 0-100)",
    "moisture": "number (percentage 0-100)",
    "vigourIndex": "number (0-100)",
    "weightPer1000Seeds": "number (grams)",
    "diseaseStatus": {
      "tested": "boolean",
      "diseases": ["string (disease names)"],
      "status": "string (enum: clean, infected, unknown)"
    },
    "weedSeedContent": "number (seeds per kg)",
    "inertMatter": "number (percentage)"
  },
  "testingRecords": {
    "laboratoryName": "string",
    "istaAccredited": "boolean",
    "testDate": "string (ISO 8601)",
    "testMethod": "string (ISTA Rules reference)",
    "testerName": "string",
    "certificateNumber": "string"
  }
}
```

### 2.3 Germination Test Record

```json
{
  "testId": "string (UUID)",
  "lotId": "string (links to Seed Lot)",
  "testStandard": "string (ISTA, AOSA, National)",
  "testDate": "string (ISO 8601)",
  "laboratory": {
    "name": "string",
    "accreditation": "string (ISTA, ISO 17025)",
    "location": "string",
    "did": "string (W3C DID)"
  },
  "testConditions": {
    "temperature": "number (Celsius)",
    "substrate": "string (paper, sand, soil)",
    "duration": "number (days)",
    "lightCondition": "string (light, dark, alternating)",
    "replicates": "number",
    "seedsPerReplicate": "number"
  },
  "results": {
    "normalSeedlings": "number",
    "abnormalSeedlings": "number",
    "deadSeeds": "number",
    "dormantSeeds": "number",
    "germinationPercentage": "number (0-100)",
    "vigourClassification": "string (high, medium, low)",
    "speedOfGermination": "number (days to 50% germination)"
  },
  "images": [
    {
      "imageUrl": "string (URL to germination test photo)",
      "captureDate": "string (ISO 8601)",
      "description": "string"
    }
  ],
  "verification": {
    "verifiedBy": "string (tester name)",
    "signature": "string (digital signature)",
    "timestamp": "string (ISO 8601)"
  }
}
```

### 2.4 Seed Certification

```json
{
  "certificateId": "string (unique certificate ID)",
  "lotId": "string (links to Seed Lot)",
  "certificationType": "string (Foundation, Registered, Certified)",
  "certificationScheme": "string (OECD, AOSCA, National)",
  "issuingAuthority": {
    "name": "string (certification agency)",
    "country": "string (ISO 3166-1 alpha-2)",
    "accreditation": "string",
    "did": "string (W3C DID)"
  },
  "issueDate": "string (ISO 8601)",
  "validUntil": "string (ISO 8601)",
  "certificationCriteria": {
    "minimumGermination": "number (percentage)",
    "minimumPurity": "number (percentage)",
    "maximumMoisture": "number (percentage)",
    "maximumWeedSeeds": "number (seeds per kg)",
    "diseaseStatus": "string (required: clean)"
  },
  "complianceStatus": {
    "germination": "boolean",
    "purity": "boolean",
    "moisture": "boolean",
    "diseaseScreen": "boolean",
    "fieldInspection": "boolean",
    "overallCompliance": "boolean"
  },
  "fieldInspection": {
    "inspectionDate": "string (ISO 8601)",
    "inspectorName": "string",
    "isolationDistance": "number (meters)",
    "varietyPurity": "number (percentage)",
    "offTypes": "number (plants per hectare)",
    "diseaseIncidence": "number (percentage)"
  },
  "blockchainRecord": {
    "transactionHash": "string (blockchain tx hash)",
    "blockNumber": "number",
    "network": "string (Ethereum, Polygon, etc.)",
    "timestamp": "string (ISO 8601)"
  }
}
```

### 2.5 Digital Seed Passport

```json
{
  "passportId": "string (UUID)",
  "qrCode": "string (QR code data)",
  "lotId": "string (links to Seed Lot)",
  "varietyId": "string (links to Seed Variety)",
  "issueDate": "string (ISO 8601)",
  "expiryDate": "string (ISO 8601)",
  "seedOrigin": {
    "country": "string (ISO 3166-1 alpha-2)",
    "region": "string",
    "producer": "string",
    "productionYear": "number (year)"
  },
  "qualitySummary": {
    "germinationRate": "number (percentage)",
    "purity": "number (percentage)",
    "certificationStatus": "string (Certified, Not Certified)",
    "organicCertified": "boolean",
    "gmoCertified": "string (GMO-free, GMO, Not tested)"
  },
  "traceabilityChain": [
    {
      "stage": "string (production, processing, distribution, retail)",
      "actor": "string (organization name)",
      "location": "string",
      "timestamp": "string (ISO 8601)",
      "action": "string (harvested, processed, shipped, received)",
      "blockchainHash": "string"
    }
  ],
  "verifiableCredential": {
    "@context": ["https://www.w3.org/2018/credentials/v1"],
    "type": ["VerifiableCredential", "SeedPassportCredential"],
    "issuer": "string (DID of issuing authority)",
    "issuanceDate": "string (ISO 8601)",
    "credentialSubject": {
      "id": "string (DID of seed lot)",
      "lotId": "string",
      "varietyName": "string",
      "certificationStatus": "string"
    },
    "proof": {
      "type": "Ed25519Signature2020",
      "created": "string (ISO 8601)",
      "proofPurpose": "assertionMethod",
      "verificationMethod": "string (DID#key-id)",
      "proofValue": "string (signature)"
    }
  }
}
```

---

## 3. Data Formats

### 3.1 JSON Schema

All seed data MUST conform to JSON Schema Draft 2020-12.

**Example: Seed Lot Schema**

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "$id": "https://wiastandards.com/schemas/seed-lot/v1.0.0",
  "type": "object",
  "required": ["lotId", "varietyId", "productionInfo", "qualityMetrics"],
  "properties": {
    "lotId": {
      "type": "string",
      "pattern": "^[A-Z0-9-]+$"
    },
    "varietyId": {
      "type": "string",
      "format": "uuid"
    },
    "qualityMetrics": {
      "type": "object",
      "required": ["germinationRate", "purity"],
      "properties": {
        "germinationRate": {
          "type": "number",
          "minimum": 0,
          "maximum": 100
        },
        "purity": {
          "type": "number",
          "minimum": 0,
          "maximum": 100
        }
      }
    }
  }
}
```

### 3.2 CSV Format (Bulk Import)

```csv
LotID,VarietyName,Producer,ProductionDate,GerminationRate,Purity,Moisture,CertificationStatus
SEED-2025-001,Rice Japonica,Korea Seed Co,2025-01-15,95.5,98.2,12.5,Certified
SEED-2025-002,Wheat Winter,US Seed Bank,2025-01-20,92.0,99.0,13.0,Certified
```

### 3.3 XML Format (Legacy Systems)

```xml
<?xml version="1.0" encoding="UTF-8"?>
<SeedLot xmlns="https://wiastandards.com/schemas/seed-lot/v1">
  <LotID>SEED-2025-001</LotID>
  <VarietyID>550e8400-e29b-41d4-a716-446655440000</VarietyID>
  <QualityMetrics>
    <GerminationRate unit="percentage">95.5</GerminationRate>
    <Purity unit="percentage">98.2</Purity>
    <Moisture unit="percentage">12.5</Moisture>
  </QualityMetrics>
</SeedLot>
```

---

## 4. Enumerations

### 4.1 Crop Types

```
grain, vegetable, fruit, oilseed, fiber, forage, ornamental, medicinal, cover-crop
```

### 4.2 Seed Generations

```
Breeder, Foundation, Registered, Certified, Commercial
```

### 4.3 Certification Standards

```
OECD, AOSCA, ISTA, EU-Seed-Directive, National-Certified
```

### 4.4 Disease Status

```
clean, infected, unknown, not-tested
```

---

## 5. Validation Rules

### 5.1 Germination Rate
- MUST be between 0 and 100
- MUST be based on ISTA or AOSA standard testing
- SHOULD be tested within 12 months of sale

### 5.2 Purity
- MUST be ≥ 95% for Certified seed
- MUST be ≥ 98% for Foundation seed

### 5.3 Moisture Content
- MUST be ≤ 14% for most grain seeds (long-term storage)
- MUST be ≤ 8% for tropical seeds

### 5.4 Lot ID Format
- MUST follow pattern: `[PRODUCER-CODE]-[YEAR]-[SEQUENCE]`
- Example: `KSC-2025-001`

---

## 6. Interoperability

### 6.1 ISTA (International Seed Testing Association)

WIA Smart Seed data format is compatible with ISTA International Rules for Seed Testing.

### 6.2 OECD Seed Schemes

Supports OECD Seed Schemes for:
- Cereals
- Maize and Sorghum
- Crucifer Crops and other Oil or Fibre Species
- Grasses and Legumes
- Sugar and Fodder Beet
- Vegetables

### 6.3 UPOV (Plant Variety Protection)

Integrates with UPOV PLUTO database for variety registration.

### 6.4 Blockchain Integration

Supports Ethereum, Polygon, and Hyperledger Fabric for immutable traceability.

---

## 7. Example Implementation

```javascript
// Create a Seed Lot
const seedLot = {
  lotId: "KSC-2025-001",
  varietyId: "550e8400-e29b-41d4-a716-446655440000",
  productionInfo: {
    producer: {
      name: "Korea Seed Company",
      license: "KR-SEED-12345",
      location: "Seoul, South Korea",
      did: "did:wia:seed:producer:ksc"
    },
    productionDate: "2025-01-15",
    quantity: { value: 1000, unit: "kg" },
    generation: "Certified"
  },
  qualityMetrics: {
    germinationRate: 95.5,
    purity: 98.2,
    moisture: 12.5,
    vigourIndex: 88,
    diseaseStatus: { tested: true, status: "clean" }
  }
};

// Validate against schema
const isValid = validateSeedLot(seedLot);
console.log(isValid); // true
```

---

## 8. References

- ISTA International Rules for Seed Testing (2024 Edition)
- OECD Seed Schemes Rules and Directions (2024)
- UPOV Convention (1991 Act)
- ISO 17025: General requirements for testing laboratories
- W3C Verifiable Credentials Data Model v2.0

---

**Next Phase:** [Phase 2 - API Interface](./PHASE-2-API-INTERFACE.md)
