# WIA BCI Consent Protocol
## Phase 1: Data Format Specification

**Version:** 1.0.0  
**Status:** Release  
**Last Updated:** 2025-12-25  
**Author:** WIA Technical Committee

---

## Table of Contents

1. [Introduction](#introduction)
2. [Consent Record Data Model](#consent-record-data-model)
3. [JSON Schema Definitions](#json-schema-definitions)
4. [Permission Types](#permission-types)
5. [Cryptographic Signatures](#cryptographic-signatures)
6. [Validation Rules](#validation-rules)
7. [Example Payloads](#example-payloads)

---

## 1. Introduction

This specification defines the standardized data format for representing BCI consent information. The format is designed to be:

- **Universal**: Support all consent types and use cases
- **Extensible**: Allow future enhancements without breaking changes
- **Secure**: Built-in cryptographic signature support
- **Compliant**: Meet requirements across all major jurisdictions
- **Machine-readable**: Enable automated processing and validation

### 1.1 Design Principles

1. **Interoperability**: JSON format compatible with all modern systems
2. **Accessibility**: Human-readable with clear structure
3. **Versioning**: Semantic versioning for evolution
4. **Privacy**: Designed for data minimization and protection

---

## 2. Consent Record Data Model

### 2.1 Core Consent Record Structure

```json
{
  "id": "CST-20251225-001",
  "version": "1.0.0",
  "subject": {...},
  "consent": {...},
  "device": {...},
  "permissions": {...},
  "capacity": {...},
  "signature": {...},
  "legal": {...},
  "audit": {...},
  "metadata": {...}
}
```

### 2.2 Subject Information

```typescript
interface Subject {
  id: string;                    // Unique subject identifier
  type: "human";                 // Subject type
  demographics?: {
    ageRange?: string;           // Age range (not exact age for privacy)
    jurisdiction?: string;        // Legal jurisdiction (ISO 3166-1)
    primaryLanguage?: string;     // Language code (ISO 639-1)
  };
  medicalCondition?: string;      // Relevant medical condition
  communicationStatus?: string;   // Communication ability status
}
```

### 2.3 Consent Information

```typescript
interface Consent {
  type: ConsentType;              // Consent type
  date: string;                   // ISO 8601 timestamp
  validUntil: string;             // ISO 8601 timestamp  
  renewalRequired?: boolean;       // Requires periodic renewal
  status: ConsentStatus;          // Current status
  language: string;               // Language of consent materials
  witness?: WitnessInfo;          // Witness if required
}

type ConsentType = 
  | "initial" 
  | "ongoing" 
  | "research" 
  | "data_sharing" 
  | "emergency" 
  | "pediatric" 
  | "surrogate";

type ConsentStatus = 
  | "active" 
  | "pending_signature" 
  | "expired" 
  | "revoked" 
  | "suspended";
```

### 2.4 Device Information

```typescript
interface Device {
  type: DeviceType;               // BCI device type
  invasiveness?: InvasivenessLevel; // For invasive devices
  manufacturer: string;            // Manufacturer name
  model: string;                  // Device model
  serialNumber?: string;          // Device serial number
  fdaApproval?: string;           // FDA approval number if applicable
  purpose: string;                // Purpose of BCI use
  implantDate?: string;           // For implanted devices (ISO 8601)
  expectedLifespan?: string;       // Expected device lifespan
}

type DeviceType = "invasive" | "semi_invasive" | "non_invasive" | "hybrid";
type InvasivenessLevel = "intracortical" | "electrocorticography" | "subdural";
```

### 2.5 Permissions

```typescript
interface Permissions {
  // Basic permissions
  dataCollection: boolean;
  dataStorage: boolean;
  realTimeProcessing?: boolean;
  offlineAnalysis?: boolean;
  
  // Advanced permissions
  research?: boolean;
  researchScope?: string;
  thirdPartySharing?: boolean;
  thirdPartyScope?: string;
  aiTraining?: boolean;
  aiTrainingScope?: string;
  commercialUse?: boolean;
  
  // Specific permissions
  dataLinking?: boolean;
  inferenceProcessing?: boolean;
  inferenceScope?: string;
  longTermRetention?: boolean;
  retentionPeriod?: string;
  cloudStorage?: boolean;
  crossBorderTransfer?: boolean;
  publicationRights?: boolean;
  publicationRequirement?: string;
  emergencyAccess?: boolean;
  geneticLinking?: boolean;
  biometricUse?: boolean;
}
```

### 2.6 Capacity Assessment

```typescript
interface Capacity {
  assessmentDate: string;          // ISO 8601 timestamp
  score: number;                   // 0-100
  assessor: AssessorInfo;          // Who performed assessment
  method: string;                  // Assessment method used
  understandingScore?: number;      // Understanding component (0-10)
  decisionMakingScore?: number;    // Decision-making component (0-10)
  communicationMethod?: string;     // How subject communicates
  notes?: string;                  // Additional notes
  reassessmentSchedule?: string;    // When to reassess
}

interface AssessorInfo {
  name: string;
  credentials?: string;
  license?: string;
}
```

### 2.7 Signature

```typescript
interface Signature {
  method: SignatureMethod;
  algorithm: SignatureAlgorithm;
  hash: HashAlgorithm;
  value: string;                   // Base64 encoded signature
  timestamp: string;               // ISO 8601 timestamp
  signingMethod?: string;          // How signature was captured
  witnessSignature?: WitnessSignature;
}

type SignatureMethod = "digital" | "digital_witnessed" | "biometric";
type SignatureAlgorithm = "RSA-2048" | "RSA-4096" | "ECDSA-P256" | "Ed25519";
type HashAlgorithm = "SHA-256" | "SHA-384" | "SHA-512";
```

### 2.8 Legal Information

```typescript
interface Legal {
  jurisdiction: string;            // ISO 3166-1 alpha-2
  primaryFramework: string;        // Primary legal framework
  additionalFrameworks?: string[]; // Additional applicable frameworks
  ethicsApproval?: EthicsApproval;
  regulatoryStatus?: string;
  trialRegistration?: string;
  insuranceCoverage?: string;
  legalRepresentation?: LegalRep;
}

interface EthicsApproval {
  irbNumber: string;
  approvalDate: string;
  expirationDate: string;
  institution: string;
}
```

### 2.9 Audit Trail

```typescript
interface Audit {
  created: string;                 // ISO 8601 timestamp
  createdBy: string;
  modified: string;                // ISO 8601 timestamp
  modifiedBy: string;
  version: number;
  previousVersions?: string[];      // References to prior versions
  accessLog: AccessLogEntry[];
  complianceChecks?: ComplianceInfo;
}

interface AccessLogEntry {
  timestamp: string;
  actor: string;
  action: string;
  ipAddress?: string;
}
```

---

## 3. JSON Schema Definitions

Complete JSON Schema available at: `https://wiastandards.com/schemas/bci-consent/v1.0.0/consent-record.json`

Key validation rules:
- All required fields must be present
- Dates must be valid ISO 8601 format
- `validUntil` must be after `consent.date`
- Capacity score must be 0-100
- Jurisdiction must be valid ISO 3166-1 code
- Signature hash must match computed hash

---

## 4. Permission Types

### 4.1 Standard Permission Set

| Permission | Risk Level | Description |
|-----------|------------|-------------|
| dataCollection | Medium | Permission to collect neural signals |
| dataStorage | Medium | Permission to store data beyond session |
| realTimeProcessing | Low | Process data in real-time for BCI control |
| offlineAnalysis | Medium | Analyze historical data |
| research | Medium | Use data in research studies |
| thirdPartySharing | High | Share data with external entities |
| aiTraining | High | Use data to train ML models |
| commercialUse | High | Use data for commercial purposes |
| dataLinking | Very High | Link neural data with other sources |
| inferenceProcessing | Very High | Derive information beyond measurements |
| longTermRetention | Medium | Keep data beyond minimum period |
| cloudStorage | Medium | Store in cloud vs. local only |
| crossBorderTransfer | High | Transfer data across borders |
| publicationRights | Medium | Publish anonymized data/results |
| emergencyAccess | Low | Allow access in emergencies |
| geneticLinking | Very High | Link neural data with genetic info |
| biometricUse | High | Use neural patterns for ID |

---

## 5. Cryptographic Signatures

### 5.1 Signature Generation Process

1. Extract all fields except `signature`
2. Canonicalize JSON (deterministic ordering)
3. Hash canonical representation (SHA-256)
4. Sign hash with private key
5. Store signature in Base64 encoding

### 5.2 Signature Verification

```javascript
function verifyConsentSignature(consentRecord, publicKey) {
    const dataToVerify = {...consentRecord};
    delete dataToVerify.signature;
    
    const canonical = JSON.stringify(dataToVerify, Object.keys(dataToVerify).sort());
    const hash = sha256(canonical);
    
    return crypto.verify(hash, consentRecord.signature.value, publicKey);
}
```

---

## 6. Validation Rules

### 6.1 Required Field Validation

All top-level fields are required except where marked optional in schema.

### 6.2 Data Type Validation

- String fields: non-empty UTF-8
- Number fields: valid JSON numbers
- Boolean fields: true or false only
- Date fields: ISO 8601 format

### 6.3 Business Logic Validation

- Capacity score 70+ required for valid consent (unless surrogate)
- `validUntil` must be future date at time of creation
- Jurisdiction must match list of supported jurisdictions
- At least one permission must be granted

---

## 7. Example Payloads

### 7.1 Minimal Valid Consent

```json
{
  "id": "CST-MIN-001",
  "version": "1.0.0",
  "subject": {
    "id": "SUBJ-001",
    "type": "human"
  },
  "consent": {
    "type": "initial",
    "date": "2025-12-25T10:00:00Z",
    "validUntil": "2026-12-25T10:00:00Z",
    "status": "active",
    "language": "en-US"
  },
  "device": {
    "type": "non_invasive",
    "manufacturer": "Example Corp",
    "model": "EEG-100",
    "purpose": "Research study"
  },
  "permissions": {
    "dataCollection": true,
    "dataStorage": false
  },
  "capacity": {
    "assessmentDate": "2025-12-25T09:00:00Z",
    "score": 95,
    "assessor": {
      "name": "Dr. Smith"
    },
    "method": "MoCA"
  },
  "signature": {
    "method": "digital",
    "algorithm": "Ed25519",
    "hash": "SHA-256",
    "value": "base64_encoded_signature",
    "timestamp": "2025-12-25T10:00:15Z"
  },
  "legal": {
    "jurisdiction": "US",
    "primaryFramework": "Common_Rule"
  },
  "audit": {
    "created": "2025-12-25T10:00:15Z",
    "createdBy": "system",
    "modified": "2025-12-25T10:00:15Z",
    "modifiedBy": "system",
    "version": 1,
    "accessLog": []
  },
  "metadata": {
    "standard": "WIA-AAC-016",
    "standardVersion": "1.0.0",
    "philosophy": "弘益人間 (Benefit All Humanity)"
  }
}
```

---

**弘益人間 (Hongik Ingan)** - *Benefit All Humanity*

© 2025 SmileStory Inc. / WIA  
Released under MIT License
