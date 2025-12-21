# WIA Cryo-Consent Data Format Standard
## Phase 1 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #06B6D4 (Cyan)

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

The WIA Cryo-Consent Data Format Standard defines a comprehensive JSON-based format for managing legal consent records related to cryopreservation procedures. This standard ensures that consent is properly documented, verifiable, and legally enforceable across multiple jurisdictions while respecting individual autonomy and legal requirements.

**Core Objectives**:
- Standardize consent documentation for cryopreservation procedures globally
- Enable multi-jurisdiction legal compliance and validation
- Support consent modification, revocation, and delegation workflows
- Ensure cryptographic verification and immutable audit trails
- Facilitate guardian and proxy consent management
- Integrate with existing legal and medical consent frameworks

### 1.2 Scope

This standard covers the following consent domains:

| Domain | Description |
|--------|-------------|
| Primary Consent | Core cryopreservation authorization and scope |
| Consent Scope | Specific permissions for preservation, research, and revival |
| Modification & Revocation | Consent changes and withdrawal procedures |
| Proxy & Guardian Consent | Delegated authority and legal representation |
| Multi-Jurisdiction Compliance | Cross-border legal framework alignment |
| Audit & Verification | Cryptographic proof and legal validation |

### 1.3 Design Principles

1. **Legal Validity**: Compliance with local and international consent laws
2. **Immutability**: Tamper-proof consent records with blockchain-ready architecture
3. **Traceability**: Complete audit trail for all consent modifications
4. **Flexibility**: Support for diverse legal frameworks and cultural requirements
5. **Privacy**: GDPR, HIPAA, and international privacy law compliance
6. **Verifiability**: Cryptographic signatures and multi-party witnessing

---

## Terminology

### 2.1 Core Terms

| Term | Definition |
|------|------------|
| **Consent Grantor** | The individual providing consent for cryopreservation |
| **Consent Scope** | Specific permissions granted (preservation, research, revival) |
| **Legal Guardian** | Court-appointed representative with consent authority |
| **Healthcare Proxy** | Designated individual authorized to make medical decisions |
| **Revocation** | Formal withdrawal of previously granted consent |
| **Modification** | Amendment to existing consent terms |
| **Witnessing** | Legal attestation by authorized parties |
| **Jurisdiction** | Legal territory governing consent validity |

### 2.2 Data Types

| Type | Description | Example |
|------|-------------|---------|
| `string` | UTF-8 encoded text | `"CONSENT-2025-001"` |
| `number` | IEEE 754 double precision | `1.0`, `0.5` |
| `integer` | Signed 64-bit integer | `1`, `100` |
| `boolean` | Boolean value | `true`, `false` |
| `timestamp` | ISO 8601 datetime | `"2025-01-15T10:30:00Z"` |
| `uuid` | UUID v4 identifier | `"550e8400-e29b-41d4-a716-446655440000"` |
| `signature` | Cryptographic signature | `"0x1234abcd..."` |

### 2.3 Field Requirements

| Marker | Meaning |
|--------|---------|
| **REQUIRED** | Field must be present |
| **OPTIONAL** | Field may be omitted |
| **CONDITIONAL** | Required under specific conditions |

---

## Base Structure

### 3.1 Message Format

All WIA Cryo-Consent messages follow this base structure:

```json
{
  "$schema": "https://wia.live/cryo-consent/v1/schema.json",
  "version": "1.0.0",
  "consentId": "uuid-v4-string",
  "messageType": "consent_record",
  "timestamp": {
    "created": "2025-01-15T10:30:00Z",
    "modified": "2025-01-15T10:30:00Z",
    "effectiveDate": "2025-01-15T00:00:00Z",
    "expirationDate": null
  },
  "grantor": {
    "id": "PERSON-001",
    "identityVerification": {
      "method": "biometric",
      "verifiedAt": "2025-01-15T10:00:00Z",
      "verifierId": "NOTARY-001"
    }
  },
  "consent": {
    "status": "active",
    "scope": {},
    "conditions": {},
    "jurisdiction": []
  },
  "legal": {
    "witnesses": [],
    "notarization": {},
    "jurisdiction": "US-CA"
  },
  "meta": {
    "hash": "sha256-hash",
    "signature": "digital-signature",
    "previousHash": "previous-record-hash",
    "version": 1
  }
}
```

### 3.2 Field Details

#### 3.2.1 `consentId` (REQUIRED)

```
Type: string
Format: UUID v4
Description: Unique identifier for this consent record
Example: "550e8400-e29b-41d4-a716-446655440000"
```

#### 3.2.2 `messageType` (REQUIRED)

```
Type: string
Description: Type of consent message
Valid values:
  - "consent_record"      : Initial consent documentation
  - "consent_modification": Amendment to existing consent
  - "consent_revocation"  : Withdrawal of consent
  - "proxy_delegation"    : Guardian/proxy authorization
  - "jurisdiction_update" : Legal jurisdiction changes
  - "verification_record" : Third-party verification
```

#### 3.2.3 `status` (REQUIRED)

```
Type: string
Description: Current consent status
Valid values:
  - "active"     : Consent is valid and in effect
  - "pending"    : Awaiting verification or approval
  - "revoked"    : Consent has been withdrawn
  - "expired"    : Consent validity period ended
  - "suspended"  : Temporarily on hold
  - "superseded" : Replaced by newer consent
```

---

## Data Schema

### 4.1 Complete JSON Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wia.live/cryo-consent/v1/schema.json",
  "title": "WIA Cryo-Consent Record",
  "type": "object",
  "required": ["version", "consentId", "messageType", "timestamp", "grantor", "consent", "legal"],
  "properties": {
    "version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+\\.\\d+$"
    },
    "consentId": {
      "type": "string",
      "format": "uuid"
    },
    "messageType": {
      "type": "string",
      "enum": ["consent_record", "consent_modification", "consent_revocation", "proxy_delegation", "jurisdiction_update", "verification_record"]
    },
    "timestamp": {
      "type": "object",
      "required": ["created", "effectiveDate"],
      "properties": {
        "created": { "type": "string", "format": "date-time" },
        "modified": { "type": "string", "format": "date-time" },
        "effectiveDate": { "type": "string", "format": "date-time" },
        "expirationDate": { "type": ["string", "null"], "format": "date-time" }
      }
    },
    "grantor": {
      "type": "object",
      "required": ["id"],
      "properties": {
        "id": { "type": "string" },
        "identityVerification": {
          "type": "object",
          "required": ["method", "verifiedAt"],
          "properties": {
            "method": {
              "type": "string",
              "enum": ["biometric", "government_id", "notary", "witness", "video", "digital_signature"]
            },
            "verifiedAt": { "type": "string", "format": "date-time" },
            "verifierId": { "type": "string" },
            "documentId": { "type": "string" }
          }
        },
        "capacity": {
          "type": "object",
          "properties": {
            "mentalCompetence": { "type": "boolean" },
            "assessedBy": { "type": "string" },
            "assessmentDate": { "type": "string", "format": "date-time" }
          }
        }
      }
    },
    "consent": {
      "type": "object",
      "required": ["status", "scope"],
      "properties": {
        "status": {
          "type": "string",
          "enum": ["active", "pending", "revoked", "expired", "suspended", "superseded"]
        },
        "scope": {
          "type": "object",
          "required": ["preservation", "research", "revival"],
          "properties": {
            "preservation": {
              "type": "object",
              "properties": {
                "authorized": { "type": "boolean" },
                "types": {
                  "type": "array",
                  "items": { "type": "string", "enum": ["whole_body", "neuro", "tissue", "dna"] }
                },
                "conditions": { "type": "array", "items": { "type": "string" } }
              }
            },
            "research": {
              "type": "object",
              "properties": {
                "authorized": { "type": "boolean" },
                "categories": {
                  "type": "array",
                  "items": { "type": "string", "enum": ["medical", "scientific", "commercial", "educational"] }
                },
                "restrictions": { "type": "array", "items": { "type": "string" } }
              }
            },
            "revival": {
              "type": "object",
              "properties": {
                "authorized": { "type": "boolean" },
                "conditions": { "type": "array", "items": { "type": "string" } },
                "minimumViabilityThreshold": { "type": "number", "minimum": 0, "maximum": 1 }
              }
            }
          }
        },
        "conditions": {
          "type": "object",
          "properties": {
            "minimumTechnology": { "type": "string" },
            "minimumSuccessRate": { "type": "number" },
            "financialConditions": { "type": "array", "items": { "type": "string" } }
          }
        },
        "jurisdiction": {
          "type": "array",
          "items": { "type": "string", "pattern": "^[A-Z]{2}(-[A-Z]{2})?$" }
        }
      }
    },
    "proxy": {
      "type": "object",
      "properties": {
        "proxies": {
          "type": "array",
          "items": {
            "type": "object",
            "required": ["id", "type", "authorizedActions"],
            "properties": {
              "id": { "type": "string" },
              "type": { "type": "string", "enum": ["guardian", "healthcare_proxy", "attorney", "family_member"] },
              "authorizedActions": { "type": "array", "items": { "type": "string" } },
              "effectiveDate": { "type": "string", "format": "date-time" },
              "expirationDate": { "type": ["string", "null"], "format": "date-time" }
            }
          }
        }
      }
    },
    "legal": {
      "type": "object",
      "required": ["jurisdiction"],
      "properties": {
        "witnesses": {
          "type": "array",
          "items": {
            "type": "object",
            "required": ["id", "name", "signedAt"],
            "properties": {
              "id": { "type": "string" },
              "name": { "type": "string" },
              "signedAt": { "type": "string", "format": "date-time" },
              "signature": { "type": "string" }
            }
          }
        },
        "notarization": {
          "type": "object",
          "properties": {
            "notaryId": { "type": "string" },
            "notaryName": { "type": "string" },
            "notarizedAt": { "type": "string", "format": "date-time" },
            "sealNumber": { "type": "string" },
            "jurisdiction": { "type": "string" }
          }
        },
        "jurisdiction": { "type": "string", "pattern": "^[A-Z]{2}(-[A-Z]{2})?$" },
        "governingLaw": { "type": "string" },
        "compliance": {
          "type": "array",
          "items": { "type": "string" }
        }
      }
    },
    "meta": {
      "type": "object",
      "required": ["hash", "signature", "version"],
      "properties": {
        "hash": { "type": "string" },
        "signature": { "type": "string" },
        "previousHash": { "type": "string" },
        "version": { "type": "integer", "minimum": 1 },
        "blockchainAnchor": { "type": "string" }
      }
    }
  }
}
```

---

## Field Specifications

### 5.1 Consent Scope Fields

| Field | Type | Required | Description | Example |
|-------|------|----------|-------------|---------|
| `preservation.authorized` | boolean | REQUIRED | Consent for cryopreservation | `true` |
| `preservation.types` | array | REQUIRED | Authorized preservation types | `["whole_body", "neuro"]` |
| `research.authorized` | boolean | REQUIRED | Consent for research use | `false` |
| `research.categories` | array | CONDITIONAL | Permitted research types | `["medical", "scientific"]` |
| `revival.authorized` | boolean | REQUIRED | Consent for revival attempts | `true` |
| `revival.minimumViabilityThreshold` | number | OPTIONAL | Minimum success probability | `0.7` |

### 5.2 Proxy Authorization Fields

| Field | Type | Required | Description | Example |
|-------|------|----------|-------------|---------|
| `proxies[].id` | string | REQUIRED | Proxy identifier | `"PROXY-001"` |
| `proxies[].type` | string | REQUIRED | Type of proxy authority | `"healthcare_proxy"` |
| `proxies[].authorizedActions` | array | REQUIRED | Permitted actions | `["modify_consent", "revoke"]` |
| `proxies[].effectiveDate` | timestamp | REQUIRED | Authority start date | `"2025-01-15T00:00:00Z"` |
| `proxies[].expirationDate` | timestamp | OPTIONAL | Authority end date | `"2030-01-15T00:00:00Z"` |

### 5.3 Legal Verification Fields

| Field | Type | Required | Description | Example |
|-------|------|----------|-------------|---------|
| `witnesses[].id` | string | REQUIRED | Witness identifier | `"WITNESS-001"` |
| `witnesses[].name` | string | REQUIRED | Witness full name | `"John Smith"` |
| `witnesses[].signedAt` | timestamp | REQUIRED | Signature timestamp | `"2025-01-15T10:30:00Z"` |
| `notarization.notaryId` | string | CONDITIONAL | Notary public identifier | `"NOTARY-CA-12345"` |
| `notarization.sealNumber` | string | CONDITIONAL | Notary seal number | `"SEAL-2025-001"` |

---

## Data Types

### 6.1 Custom Types

#### ConsentStatus

```typescript
type ConsentStatus =
  | 'active'
  | 'pending'
  | 'revoked'
  | 'expired'
  | 'suspended'
  | 'superseded';
```

#### PreservationType

```typescript
type PreservationType =
  | 'whole_body'
  | 'neuro'
  | 'tissue'
  | 'dna';
```

#### ResearchCategory

```typescript
type ResearchCategory =
  | 'medical'
  | 'scientific'
  | 'commercial'
  | 'educational';
```

#### ProxyType

```typescript
type ProxyType =
  | 'guardian'
  | 'healthcare_proxy'
  | 'attorney'
  | 'family_member';
```

#### VerificationMethod

```typescript
type VerificationMethod =
  | 'biometric'
  | 'government_id'
  | 'notary'
  | 'witness'
  | 'video'
  | 'digital_signature';
```

### 6.2 TypeScript Interfaces

```typescript
interface ConsentRecord {
  version: string;
  consentId: string;
  messageType: string;
  timestamp: {
    created: string;
    modified?: string;
    effectiveDate: string;
    expirationDate?: string | null;
  };
  grantor: {
    id: string;
    identityVerification: {
      method: VerificationMethod;
      verifiedAt: string;
      verifierId?: string;
      documentId?: string;
    };
    capacity?: {
      mentalCompetence: boolean;
      assessedBy: string;
      assessmentDate: string;
    };
  };
  consent: {
    status: ConsentStatus;
    scope: ConsentScope;
    conditions?: ConsentConditions;
    jurisdiction: string[];
  };
  proxy?: ProxyDelegation;
  legal: LegalAttestation;
  meta: MetaData;
}

interface ConsentScope {
  preservation: {
    authorized: boolean;
    types: PreservationType[];
    conditions?: string[];
  };
  research: {
    authorized: boolean;
    categories?: ResearchCategory[];
    restrictions?: string[];
  };
  revival: {
    authorized: boolean;
    conditions?: string[];
    minimumViabilityThreshold?: number;
  };
}
```

### 6.3 Python Data Classes

```python
from dataclasses import dataclass
from typing import List, Optional
from datetime import datetime
from enum import Enum

class ConsentStatus(Enum):
    ACTIVE = "active"
    PENDING = "pending"
    REVOKED = "revoked"
    EXPIRED = "expired"
    SUSPENDED = "suspended"
    SUPERSEDED = "superseded"

class PreservationType(Enum):
    WHOLE_BODY = "whole_body"
    NEURO = "neuro"
    TISSUE = "tissue"
    DNA = "dna"

@dataclass
class ConsentScope:
    preservation: dict
    research: dict
    revival: dict

@dataclass
class ConsentRecord:
    version: str
    consent_id: str
    message_type: str
    timestamp: dict
    grantor: dict
    consent: dict
    legal: dict
    meta: dict
    proxy: Optional[dict] = None
```

---

## Validation Rules

### 7.1 Required Field Validation

| Rule ID | Field | Validation |
|---------|-------|------------|
| VAL-001 | `version` | Must match `^\d+\.\d+\.\d+$` |
| VAL-002 | `consentId` | Must be valid UUID v4 |
| VAL-003 | `timestamp.created` | Must be valid ISO 8601 |
| VAL-004 | `timestamp.effectiveDate` | Must be valid ISO 8601 |
| VAL-005 | `grantor.id` | Must not be empty |
| VAL-006 | `consent.status` | Must be valid enum value |
| VAL-007 | `legal.jurisdiction` | Must match `^[A-Z]{2}(-[A-Z]{2})?$` |

### 7.2 Business Logic Validation

| Rule ID | Description | Error Code |
|---------|-------------|------------|
| BUS-001 | `effectiveDate` must not be in the future | `ERR_INVALID_EFFECTIVE_DATE` |
| BUS-002 | `expirationDate` must be after `effectiveDate` if present | `ERR_INVALID_EXPIRATION` |
| BUS-003 | At least one witness required if no notarization | `ERR_INSUFFICIENT_ATTESTATION` |
| BUS-004 | Grantor must be 18+ years old or have guardian | `ERR_INSUFFICIENT_CAPACITY` |
| BUS-005 | Revoked consent cannot be reactivated | `ERR_INVALID_STATUS_CHANGE` |
| BUS-006 | `minimumViabilityThreshold` must be 0.0-1.0 | `ERR_INVALID_THRESHOLD` |

### 7.3 Jurisdiction-Specific Validation

| Jurisdiction | Rule | Requirement |
|--------------|------|-------------|
| US-CA | Notarization | Required for whole_body preservation |
| US-NY | Witnesses | Minimum 2 witnesses required |
| EU-* | GDPR Compliance | Must include data protection notice |
| KR | Medical Review | Requires physician attestation |

---

## Examples

### 8.1 Valid Consent Record - Full Authorization

```json
{
  "$schema": "https://wia.live/cryo-consent/v1/schema.json",
  "version": "1.0.0",
  "consentId": "550e8400-e29b-41d4-a716-446655440001",
  "messageType": "consent_record",
  "timestamp": {
    "created": "2025-01-15T10:30:00Z",
    "modified": "2025-01-15T10:30:00Z",
    "effectiveDate": "2025-01-15T00:00:00Z",
    "expirationDate": null
  },
  "grantor": {
    "id": "PERSON-2025-001",
    "identityVerification": {
      "method": "government_id",
      "verifiedAt": "2025-01-15T10:00:00Z",
      "verifierId": "NOTARY-CA-12345",
      "documentId": "DL-CA-D1234567"
    },
    "capacity": {
      "mentalCompetence": true,
      "assessedBy": "DR-PSY-001",
      "assessmentDate": "2025-01-14T15:00:00Z"
    }
  },
  "consent": {
    "status": "active",
    "scope": {
      "preservation": {
        "authorized": true,
        "types": ["whole_body"],
        "conditions": [
          "Use M22 or superior cryoprotectant",
          "Begin within 15 minutes of legal death"
        ]
      },
      "research": {
        "authorized": true,
        "categories": ["medical", "scientific"],
        "restrictions": [
          "No commercial exploitation without family approval",
          "Anonymize all published data"
        ]
      },
      "revival": {
        "authorized": true,
        "conditions": [
          "Medical technology exists to restore consciousness",
          "Minimum 70% probability of full cognitive function"
        ],
        "minimumViabilityThreshold": 0.7
      }
    },
    "conditions": {
      "minimumTechnology": "Molecular repair nanotechnology",
      "minimumSuccessRate": 0.7,
      "financialConditions": [
        "Revival costs covered by trust fund",
        "No debt obligation to patient"
      ]
    },
    "jurisdiction": ["US-CA", "US"]
  },
  "proxy": {
    "proxies": [
      {
        "id": "PROXY-001",
        "type": "healthcare_proxy",
        "authorizedActions": ["modify_consent", "authorize_revival", "receive_updates"],
        "effectiveDate": "2025-01-15T00:00:00Z",
        "expirationDate": null
      }
    ]
  },
  "legal": {
    "witnesses": [
      {
        "id": "WITNESS-001",
        "name": "Jane Doe",
        "signedAt": "2025-01-15T10:30:00Z",
        "signature": "0x1234abcd..."
      },
      {
        "id": "WITNESS-002",
        "name": "Robert Johnson",
        "signedAt": "2025-01-15T10:31:00Z",
        "signature": "0x5678efgh..."
      }
    ],
    "notarization": {
      "notaryId": "NOTARY-CA-12345",
      "notaryName": "Maria Garcia",
      "notarizedAt": "2025-01-15T10:35:00Z",
      "sealNumber": "SEAL-2025-001",
      "jurisdiction": "US-CA"
    },
    "jurisdiction": "US-CA",
    "governingLaw": "California Health and Safety Code Section 7100-7117",
    "compliance": ["UAGA", "HIPAA", "California End of Life Option Act"]
  },
  "meta": {
    "hash": "sha256:a5b9c3d4e5f6...",
    "signature": "0xabcdef123456...",
    "version": 1,
    "blockchainAnchor": "ethereum:0x1234567890abcdef"
  }
}
```

### 8.2 Valid Consent Modification - Research Authorization

```json
{
  "version": "1.0.0",
  "consentId": "550e8400-e29b-41d4-a716-446655440002",
  "messageType": "consent_modification",
  "timestamp": {
    "created": "2025-02-20T14:00:00Z",
    "effectiveDate": "2025-02-20T00:00:00Z"
  },
  "grantor": {
    "id": "PERSON-2025-001",
    "identityVerification": {
      "method": "video",
      "verifiedAt": "2025-02-20T13:45:00Z",
      "verifierId": "LAWYER-001"
    }
  },
  "consent": {
    "status": "active",
    "scope": {
      "preservation": {
        "authorized": true,
        "types": ["whole_body"]
      },
      "research": {
        "authorized": true,
        "categories": ["medical", "scientific", "commercial"],
        "restrictions": [
          "50% of commercial proceeds to revival fund"
        ]
      },
      "revival": {
        "authorized": true,
        "minimumViabilityThreshold": 0.7
      }
    },
    "jurisdiction": ["US-CA"]
  },
  "legal": {
    "witnesses": [
      {
        "id": "WITNESS-003",
        "name": "Attorney Sarah Lee",
        "signedAt": "2025-02-20T14:00:00Z",
        "signature": "0x9999aaaa..."
      }
    ],
    "jurisdiction": "US-CA",
    "governingLaw": "California Health and Safety Code"
  },
  "meta": {
    "hash": "sha256:b6c8d5e7f8...",
    "signature": "0xfedcba654321...",
    "previousHash": "sha256:a5b9c3d4e5f6...",
    "version": 2
  }
}
```

### 8.3 Valid Consent Revocation

```json
{
  "version": "1.0.0",
  "consentId": "550e8400-e29b-41d4-a716-446655440003",
  "messageType": "consent_revocation",
  "timestamp": {
    "created": "2025-03-10T09:00:00Z",
    "effectiveDate": "2025-03-10T00:00:00Z"
  },
  "grantor": {
    "id": "PERSON-2025-002",
    "identityVerification": {
      "method": "notary",
      "verifiedAt": "2025-03-10T08:45:00Z",
      "verifierId": "NOTARY-NY-67890"
    }
  },
  "consent": {
    "status": "revoked",
    "scope": {
      "preservation": {
        "authorized": false,
        "types": []
      },
      "research": {
        "authorized": false,
        "categories": []
      },
      "revival": {
        "authorized": false
      }
    },
    "jurisdiction": ["US-NY"]
  },
  "legal": {
    "witnesses": [
      {
        "id": "WITNESS-004",
        "name": "Michael Chen",
        "signedAt": "2025-03-10T09:00:00Z",
        "signature": "0xbbbbcccc..."
      }
    ],
    "notarization": {
      "notaryId": "NOTARY-NY-67890",
      "notaryName": "David Brown",
      "notarizedAt": "2025-03-10T09:05:00Z",
      "sealNumber": "NY-SEAL-2025-045",
      "jurisdiction": "US-NY"
    },
    "jurisdiction": "US-NY"
  },
  "meta": {
    "hash": "sha256:c7d9e6f8g9...",
    "signature": "0x111122223333...",
    "previousHash": "sha256:prev-hash...",
    "version": 3
  }
}
```

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01 | Initial release |

---

## Appendix A: Related Standards

| Standard | Relationship |
|----------|--------------|
| WIA Cryo-Preservation | Links consent to preservation procedures |
| WIA Cryo-Identity | Subject identification and verification |
| HL7 FHIR Consent | Healthcare consent interoperability |
| GDPR Article 9 | Special category data processing |
| UAGA | Uniform Anatomical Gift Act compliance |

---

<div align="center">

**WIA Cryo-Consent Data Format Standard v1.0.0**

**弘益人間 (홍익인간)** - Benefit All Humanity

---

**© 2025 WIA Standards Committee**

**MIT License**

</div>
