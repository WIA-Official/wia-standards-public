# WIA-PET-010 Pet Insurance - Phase 1: Data Format

**Version:** 1.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-12-25

---

## 1. Overview

Phase 1 defines the core data structures and formats for the WIA Pet Insurance Standard (WIA-PET-010). This specification ensures interoperability between insurance providers, veterinary clinics, pet health systems, and blockchain verification networks.

### 1.1 Design Principles

- **Standardization**: Unified schema across all providers
- **Extensibility**: Support for future enhancements
- **Interoperability**: Compatible with WIA-PET-001 (Health Passport)
- **Security**: Encryption and blockchain verification
- **Privacy**: GDPR and data protection compliance

### 1.2 Scope

This phase covers:
- Insurance Policy Data Schema
- Coverage Plan Definitions
- Claims Data Format
- Premium Calculation Schema
- Risk Assessment Data
- Fraud Detection Markers
- Blockchain Integration Fields

---

## 2. Core Data Schema

### 2.1 Insurance Policy Object

The primary data structure representing a pet insurance policy.

```json
{
  "policyId": "string (required, unique)",
  "standardVersion": "string (required, format: WIA-PET-010-vX.Y.Z)",
  "issuer": {
    "id": "string (required)",
    "name": "string (required)",
    "licenseNumber": "string (required)",
    "country": "string (ISO 3166-1 alpha-2)",
    "wiaVerified": "boolean"
  },
  "pet": {
    "id": "string (required, unique)",
    "name": "string (required)",
    "species": "string (enum: Dog, Cat, Rabbit, Bird, Other)",
    "breed": "string",
    "birthDate": "string (ISO 8601 date)",
    "gender": "string (enum: Male, Female, Unknown)",
    "microchipId": "string (optional, ISO 11784/11785)",
    "healthPassportId": "string (optional, WIA-PET-001 reference)",
    "weight": "number (kg)",
    "color": "string"
  },
  "owner": {
    "id": "string (required, unique)",
    "name": "string (required)",
    "email": "string (required, format: email)",
    "phone": "string (E.164 format)",
    "address": {
      "street": "string",
      "city": "string",
      "state": "string",
      "zipCode": "string",
      "country": "string (ISO 3166-1 alpha-2)"
    },
    "did": "string (optional, W3C DID)"
  },
  "plan": {
    "tier": "string (enum: Basic, Standard, Premium)",
    "coverageType": "string (enum: Comprehensive, AccidentOnly, IllnessOnly)",
    "annualLimit": "number (USD)",
    "deductible": "number (USD)",
    "coInsurance": "number (percentage, 0-100)",
    "monthlyPremium": "number (USD)",
    "startDate": "string (ISO 8601 date)",
    "renewalDate": "string (ISO 8601 date)",
    "status": "string (enum: Active, Suspended, Cancelled, Expired)"
  },
  "coverage": {
    "accidents": "boolean",
    "illnesses": "boolean",
    "wellness": "boolean",
    "dental": "boolean",
    "hereditary": "boolean",
    "chronic": "boolean",
    "prescriptions": "boolean",
    "emergencyCare": "boolean",
    "specialistCare": "boolean",
    "surgery": "boolean",
    "hospitalization": "boolean",
    "diagnostics": "boolean",
    "alternativeTherapy": "boolean",
    "behavioralTherapy": "boolean"
  },
  "exclusions": ["string (array of excluded conditions/procedures)"],
  "riskFactors": {
    "age": "number (years)",
    "breedRisk": "string (enum: Low, Medium, High)",
    "location": "string (enum: Urban, Suburban, Rural)",
    "priorClaims": "number (count)",
    "healthScore": "number (0-100)",
    "lifestyleRisk": "string (enum: Low, Medium, High)"
  },
  "blockchain": {
    "policyHash": "string (hex)",
    "timestamp": "string (ISO 8601 timestamp)",
    "network": "string (enum: Ethereum, Polygon, BSC)",
    "smartContract": "string (address)",
    "transactionId": "string (tx hash)"
  },
  "metadata": {
    "createdAt": "string (ISO 8601 timestamp)",
    "updatedAt": "string (ISO 8601 timestamp)",
    "version": "number (integer)",
    "tags": ["string"]
  }
}
```

### 2.2 Field Definitions

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `policyId` | String | Yes | Unique policy identifier (format: `PET-INS-YYYY-NNNNNN`) |
| `standardVersion` | String | Yes | WIA standard version (e.g., `WIA-PET-010-v1.0.0`) |
| `issuer.id` | String | Yes | Insurance provider unique ID |
| `issuer.wiaVerified` | Boolean | No | WIA certification status |
| `pet.id` | String | Yes | Unique pet identifier |
| `pet.microchipId` | String | No | ISO 11784/11785 compliant chip ID |
| `pet.healthPassportId` | String | No | Reference to WIA-PET-001 health passport |
| `owner.did` | String | No | W3C Decentralized Identifier |
| `plan.tier` | Enum | Yes | Coverage tier: Basic, Standard, Premium |
| `plan.annualLimit` | Number | Yes | Maximum annual coverage in USD |
| `plan.deductible` | Number | Yes | Annual deductible in USD |
| `plan.coInsurance` | Number | Yes | Percentage of costs after deductible (0-100) |
| `coverage.*` | Boolean | Yes | Individual coverage flags |
| `riskFactors.healthScore` | Number | No | From WIA-PET-001 (0-100) |
| `blockchain.policyHash` | String | Yes | SHA-256 hash of policy data |

---

## 3. Claims Data Format

### 3.1 Claim Object Schema

```json
{
  "claimId": "string (required, unique, format: CLM-YYYY-NNNNNN)",
  "policyId": "string (required, references policy)",
  "status": "string (enum: Pending, Processing, Approved, Rejected, Paid)",
  "type": "string (enum: Accident, Illness, Wellness, Dental, Emergency, Surgery)",
  "submissionDate": "string (ISO 8601 timestamp)",
  "treatmentDate": "string (ISO 8601 date)",
  "clinic": {
    "id": "string (required)",
    "name": "string (required)",
    "address": "string",
    "phone": "string",
    "wiaVerified": "boolean",
    "licenseNumber": "string"
  },
  "veterinarian": {
    "id": "string",
    "name": "string",
    "licenseNumber": "string"
  },
  "diagnosis": {
    "code": "string (ICD-10-CM or SNOMED CT)",
    "description": "string (required)",
    "severity": "string (enum: Minor, Moderate, Severe, Critical)"
  },
  "treatment": {
    "description": "string (required)",
    "procedures": ["string (array of procedure codes)"],
    "medications": [{
      "name": "string",
      "dosage": "string",
      "duration": "string"
    }]
  },
  "amounts": {
    "total": "number (USD, required)",
    "eligible": "number (USD)",
    "deductible": "number (USD)",
    "coInsurance": "number (USD)",
    "insurancePays": "number (USD)",
    "ownerPays": "number (USD)"
  },
  "documents": [{
    "type": "string (enum: Invoice, Diagnosis, Prescription, MedicalRecords)",
    "url": "string (HTTPS URL)",
    "hash": "string (SHA-256)",
    "uploadedAt": "string (ISO 8601 timestamp)"
  }],
  "fraudCheck": {
    "score": "number (0-100)",
    "flags": ["string (array of detected anomalies)"],
    "verified": "boolean",
    "verifiedBy": "string (system or user ID)",
    "verifiedAt": "string (ISO 8601 timestamp)"
  },
  "payment": {
    "method": "string (enum: BankTransfer, Check, PayPal, Crypto)",
    "accountInfo": "string (encrypted)",
    "processedDate": "string (ISO 8601 timestamp)",
    "transactionId": "string"
  },
  "blockchain": {
    "claimHash": "string (hex)",
    "timestamp": "string (ISO 8601 timestamp)",
    "network": "string",
    "transactionId": "string"
  }
}
```

### 3.2 Claim Status Flow

```
Pending → Processing → Approved/Rejected → Paid
         ↓
    Needs Review (fraud detected)
```

---

## 4. Premium Calculation Data

### 4.1 Premium Calculation Schema

```json
{
  "calculationId": "string (unique)",
  "policyId": "string (references policy)",
  "calculatedAt": "string (ISO 8601 timestamp)",
  "basePremium": "number (USD)",
  "factors": {
    "speciesFactor": {
      "species": "string",
      "multiplier": "number (e.g., 1.2 for dogs)"
    },
    "ageFactor": {
      "age": "number (years)",
      "multiplier": "number"
    },
    "breedFactor": {
      "breed": "string",
      "riskLevel": "string (Low/Medium/High)",
      "multiplier": "number"
    },
    "locationFactor": {
      "location": "string (Urban/Suburban/Rural)",
      "multiplier": "number"
    },
    "healthFactor": {
      "healthScore": "number (0-100)",
      "multiplier": "number (0.8-1.2)"
    },
    "claimHistoryFactor": {
      "priorClaims": "number",
      "claimAmount": "number (USD)",
      "multiplier": "number"
    },
    "tierFactor": {
      "tier": "string (Basic/Standard/Premium)",
      "multiplier": "number"
    }
  },
  "discounts": [{
    "type": "string (e.g., MultiPet, AnnualPayment, HealthScore)",
    "amount": "number (USD)",
    "percentage": "number (0-100)"
  }],
  "finalPremium": {
    "monthly": "number (USD)",
    "annual": "number (USD)"
  },
  "breakdown": {
    "base": "number (USD)",
    "adjustments": "number (USD, can be negative)",
    "taxes": "number (USD)",
    "total": "number (USD)"
  }
}
```

### 4.2 Premium Factor Ranges

| Factor | Range | Impact |
|--------|-------|--------|
| Species | 0.7 - 1.5x | Dogs: 1.2x, Cats: 1.0x, Rabbits: 0.8x |
| Age (0-2y) | 0.9x | Young, low risk |
| Age (3-7y) | 1.0x | Prime years |
| Age (8-12y) | 1.3x | Senior, moderate risk |
| Age (13+y) | 1.6x | Elderly, high risk |
| Breed Risk Low | 0.9x | Mixed breeds, low hereditary risk |
| Breed Risk Medium | 1.0x | Most purebreds |
| Breed Risk High | 1.2x | Breeds with known issues |
| Location Urban | 1.1x | Higher vet costs |
| Location Suburban | 1.0x | Standard costs |
| Location Rural | 0.9x | Lower vet costs |
| Health Score 90-100 | 0.8x | Excellent health |
| Health Score 70-89 | 1.0x | Good health |
| Health Score <70 | 1.2x | Health concerns |

---

## 5. Coverage Plan Definitions

### 5.1 Basic Plan

```json
{
  "tier": "Basic",
  "annualLimit": 5000,
  "deductible": 500,
  "coInsurance": 30,
  "monthlyPremium": "calculated",
  "coverage": {
    "accidents": true,
    "illnesses": true,
    "wellness": false,
    "dental": false,
    "hereditary": false,
    "chronic": false,
    "prescriptions": true,
    "emergencyCare": true,
    "specialistCare": false,
    "surgery": true,
    "hospitalization": true,
    "diagnostics": true,
    "alternativeTherapy": false,
    "behavioralTherapy": false
  }
}
```

### 5.2 Standard Plan

```json
{
  "tier": "Standard",
  "annualLimit": 10000,
  "deductible": 350,
  "coInsurance": 20,
  "monthlyPremium": "calculated",
  "coverage": {
    "accidents": true,
    "illnesses": true,
    "wellness": true,
    "dental": true,
    "hereditary": false,
    "chronic": false,
    "prescriptions": true,
    "emergencyCare": true,
    "specialistCare": true,
    "surgery": true,
    "hospitalization": true,
    "diagnostics": true,
    "alternativeTherapy": true,
    "behavioralTherapy": false
  }
}
```

### 5.3 Premium Plan

```json
{
  "tier": "Premium",
  "annualLimit": 15000,
  "deductible": 250,
  "coInsurance": 20,
  "monthlyPremium": "calculated",
  "coverage": {
    "accidents": true,
    "illnesses": true,
    "wellness": true,
    "dental": true,
    "hereditary": true,
    "chronic": true,
    "prescriptions": true,
    "emergencyCare": true,
    "specialistCare": true,
    "surgery": true,
    "hospitalization": true,
    "diagnostics": true,
    "alternativeTherapy": true,
    "behavioralTherapy": true
  }
}
```

---

## 6. Validation Rules

### 6.1 Policy Validation

| Rule | Description | Validation |
|------|-------------|------------|
| Policy ID Format | Must follow pattern | `^PET-INS-\d{4}-\d{6}$` |
| Standard Version | Must be valid WIA version | `^WIA-PET-010-v\d+\.\d+\.\d+$` |
| Pet Age | Must be realistic | 0 ≤ age ≤ 30 years |
| Annual Limit | Must be positive | > 0 |
| Deductible | Must not exceed limit | ≤ annualLimit |
| Co-Insurance | Valid percentage | 0 ≤ coInsurance ≤ 100 |
| Email Format | Valid email | RFC 5322 compliant |
| Phone Format | Valid phone | E.164 format |
| Microchip ID | ISO compliant | 15 digits (ISO 11784/11785) |

### 6.2 Claim Validation

| Rule | Description | Validation |
|------|-------------|------------|
| Claim ID Format | Must follow pattern | `^CLM-\d{4}-\d{6}$` |
| Policy Reference | Must exist | Valid policyId |
| Treatment Date | Cannot be future | ≤ today |
| Claim Amount | Must be positive | > 0 |
| Documents | At least one required | documents.length > 0 |
| Diagnosis Code | Valid medical code | ICD-10-CM or SNOMED CT |

### 6.3 Premium Validation

| Rule | Description | Validation |
|------|-------------|------------|
| Base Premium | Must be positive | > 0 |
| Factor Multipliers | Valid range | 0.5 ≤ multiplier ≤ 2.0 |
| Final Premium | Reasonable amount | 10 ≤ monthly ≤ 500 USD |
| Discount Percentage | Valid range | 0 ≤ discount ≤ 50 |

---

## 7. Examples

### 7.1 Complete Policy Example

```json
{
  "policyId": "PET-INS-2025-001234",
  "standardVersion": "WIA-PET-010-v1.0.0",
  "issuer": {
    "id": "INSURER-US-001",
    "name": "Premium Pet Insurance Co.",
    "licenseNumber": "INS-CA-2024-5678",
    "country": "US",
    "wiaVerified": true
  },
  "pet": {
    "id": "PET-2025-789456",
    "name": "Buddy",
    "species": "Dog",
    "breed": "Golden Retriever",
    "birthDate": "2020-05-15",
    "gender": "Male",
    "microchipId": "985112345678901",
    "healthPassportId": "WIA-PET-HEALTH-001234",
    "weight": 32.5,
    "color": "Golden"
  },
  "owner": {
    "id": "OWNER-2025-456789",
    "name": "John Smith",
    "email": "john.smith@example.com",
    "phone": "+1-555-123-4567",
    "address": {
      "street": "123 Main St",
      "city": "San Francisco",
      "state": "CA",
      "zipCode": "94102",
      "country": "US"
    },
    "did": "did:wia:owner:456789"
  },
  "plan": {
    "tier": "Premium",
    "coverageType": "Comprehensive",
    "annualLimit": 15000,
    "deductible": 250,
    "coInsurance": 20,
    "monthlyPremium": 89.99,
    "startDate": "2025-01-01",
    "renewalDate": "2026-01-01",
    "status": "Active"
  },
  "coverage": {
    "accidents": true,
    "illnesses": true,
    "wellness": true,
    "dental": true,
    "hereditary": true,
    "chronic": true,
    "prescriptions": true,
    "emergencyCare": true,
    "specialistCare": true,
    "surgery": true,
    "hospitalization": true,
    "diagnostics": true,
    "alternativeTherapy": true,
    "behavioralTherapy": true
  },
  "exclusions": [
    "Pre-existing conditions",
    "Cosmetic procedures",
    "Breeding costs",
    "Elective procedures"
  ],
  "riskFactors": {
    "age": 4,
    "breedRisk": "Medium",
    "location": "Urban",
    "priorClaims": 2,
    "healthScore": 85,
    "lifestyleRisk": "Low"
  },
  "blockchain": {
    "policyHash": "0x1234567890abcdef1234567890abcdef1234567890abcdef1234567890abcdef",
    "timestamp": "2025-01-01T00:00:00Z",
    "network": "Ethereum",
    "smartContract": "0xabcdef1234567890abcdef1234567890abcdef12",
    "transactionId": "0xfedcba0987654321fedcba0987654321fedcba0987654321fedcba0987654321"
  },
  "metadata": {
    "createdAt": "2025-01-01T00:00:00Z",
    "updatedAt": "2025-01-01T00:00:00Z",
    "version": 1,
    "tags": ["premium", "comprehensive", "verified"]
  }
}
```

### 7.2 Complete Claim Example

```json
{
  "claimId": "CLM-2025-001234",
  "policyId": "PET-INS-2025-001234",
  "status": "Approved",
  "type": "Illness",
  "submissionDate": "2025-12-21T10:30:00Z",
  "treatmentDate": "2025-12-20",
  "clinic": {
    "id": "VET-2025-456",
    "name": "City Pet Hospital",
    "address": "456 Oak Ave, San Francisco, CA 94103",
    "phone": "+1-555-987-6543",
    "wiaVerified": true,
    "licenseNumber": "VET-CA-2024-1234"
  },
  "veterinarian": {
    "id": "VET-DOC-789",
    "name": "Dr. Sarah Johnson",
    "licenseNumber": "VMD-CA-12345"
  },
  "diagnosis": {
    "code": "K29.70",
    "description": "Gastritis, unspecified",
    "severity": "Moderate"
  },
  "treatment": {
    "description": "Treatment for acute gastritis with medication and dietary changes",
    "procedures": ["99213", "82947"],
    "medications": [{
      "name": "Metronidazole",
      "dosage": "250mg",
      "duration": "10 days"
    }]
  },
  "amounts": {
    "total": 850.00,
    "eligible": 850.00,
    "deductible": 250.00,
    "coInsurance": 120.00,
    "insurancePays": 480.00,
    "ownerPays": 370.00
  },
  "documents": [{
    "type": "Invoice",
    "url": "https://secure.storage.wia/claims/CLM-2025-001234/invoice.pdf",
    "hash": "sha256:9f86d081884c7d659a2feaa0c55ad015a3bf4f1b2b0b822cd15d6c15b0f00a08",
    "uploadedAt": "2025-12-21T10:30:00Z"
  }, {
    "type": "Diagnosis",
    "url": "https://secure.storage.wia/claims/CLM-2025-001234/diagnosis.pdf",
    "hash": "sha256:e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855",
    "uploadedAt": "2025-12-21T10:31:00Z"
  }],
  "fraudCheck": {
    "score": 95,
    "flags": [],
    "verified": true,
    "verifiedBy": "FRAUD-AI-001",
    "verifiedAt": "2025-12-21T10:32:00Z"
  },
  "payment": {
    "method": "BankTransfer",
    "accountInfo": "encrypted:...",
    "processedDate": "2025-12-22T14:00:00Z",
    "transactionId": "PAY-2025-567890"
  },
  "blockchain": {
    "claimHash": "0xabcd1234abcd1234abcd1234abcd1234abcd1234abcd1234abcd1234abcd1234",
    "timestamp": "2025-12-21T10:35:00Z",
    "network": "Ethereum",
    "transactionId": "0x9876fedc9876fedc9876fedc9876fedc9876fedc9876fedc9876fedc9876fedc"
  }
}
```

---

## 8. Data Security & Privacy

### 8.1 Encryption Requirements

- All personal data MUST be encrypted at rest (AES-256)
- Data in transit MUST use TLS 1.3+
- Owner payment information MUST be encrypted with additional layer
- Blockchain hashes use SHA-256

### 8.2 Privacy Compliance

- GDPR compliant (EU)
- CCPA compliant (California)
- Right to data deletion
- Data portability support
- Consent management

### 8.3 Data Retention

| Data Type | Retention Period |
|-----------|------------------|
| Active Policies | Duration + 7 years |
| Claims | 10 years |
| Medical Records | 10 years |
| Payment Data | 7 years |
| Blockchain Records | Permanent (immutable) |

---

## 9. Blockchain Integration

### 9.1 Policy Hash Calculation

```javascript
const policyHash = SHA256(
  policyId +
  pet.microchipId +
  owner.id +
  plan.tier +
  plan.startDate +
  plan.annualLimit
);
```

### 9.2 Smart Contract Events

- `PolicyCreated(policyId, hash, timestamp)`
- `PolicyUpdated(policyId, newHash, timestamp)`
- `PolicyCancelled(policyId, timestamp)`
- `ClaimSubmitted(claimId, policyId, amount, timestamp)`
- `ClaimApproved(claimId, paymentAmount, timestamp)`

---

## 10. Future Enhancements

### 10.1 Planned Features (v1.1)

- Multi-pet family policies
- International currency support
- AI-driven premium optimization
- Telemedicine integration
- IoT device data integration (smart collars)

### 10.2 Under Consideration (v2.0)

- Genetic risk assessment integration
- Preventive care rewards program
- Community pooling options
- Parametric insurance for specific conditions

---

**弘益人間 · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 MIT License*
