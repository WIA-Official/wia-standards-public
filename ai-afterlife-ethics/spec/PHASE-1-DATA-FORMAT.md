# WIA AI Afterlife Ethics Data Format Standard
## Phase 1 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #64748B (Slate)

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

The WIA AI Afterlife Ethics Data Format Standard defines a comprehensive, ethical framework for managing AI-generated personas of deceased individuals. This standard ensures consent, data rights, and psychological safety while enabling legitimate memorial and educational applications.

**Core Objectives**:
- Establish clear consent mechanisms for posthumous AI persona creation
- Define data ownership and usage rights for afterlife AI applications
- Regulate commercial exploitation of deceased individuals' digital personas
- Provide psychological impact assessment and mitigation guidelines
- Enable ethical AI persona lifecycle management

### 1.2 Scope

This standard covers:

| Domain | Description |
|--------|-------------|
| Consent Management | Pre-mortem consent capture and verification |
| Data Rights | Ownership, access, and usage permissions |
| Persona Generation | Ethical AI persona creation parameters |
| Commercial Regulation | Licensing and monetization controls |
| Psychological Impact | User safety and mental health guidelines |

### 1.3 Design Principles

1. **Consent-First**: No posthumous AI persona without explicit pre-mortem consent
2. **Data Sovereignty**: Individual and family control over personal data
3. **Transparency**: Clear disclosure of AI-generated nature
4. **Harm Prevention**: Psychological safety mechanisms
5. **Commercial Ethics**: Fair compensation and anti-exploitation measures

---

## Terminology

### 2.1 Core Terms

| Term | Definition |
|------|------------|
| **AI Persona** | AI-generated representation of a deceased individual |
| **Consent Record** | Legal documentation of posthumous AI usage permissions |
| **Data Subject** | Deceased individual whose data is used for AI persona |
| **Authorized Agent** | Legal representative with data usage authority |
| **Interaction Session** | Time-bounded engagement with AI persona |
| **Psychological Guardian** | Mental health professional monitoring usage |

### 2.2 Data Types

| Type | Description | Example |
|------|-------------|---------|
| `string` | UTF-8 encoded text | `"CONSENT-2025-001"` |
| `consent_level` | Degree of permitted usage | `"memorial_only"` |
| `persona_fidelity` | AI accuracy level | `0.85` (0-1 scale) |
| `usage_type` | Category of AI persona use | `"family_memorial"` |
| `timestamp` | ISO 8601 datetime | `"2025-01-15T10:30:00Z"` |

### 2.3 Field Requirements

| Marker | Meaning |
|--------|---------|
| **REQUIRED** | Must be present |
| **OPTIONAL** | May be omitted |
| **CONDITIONAL** | Required under specific conditions |

---

## Base Structure

### 3.1 Ethics Record Format

```json
{
  "$schema": "https://wia.live/ai-afterlife-ethics/v1/schema.json",
  "version": "1.0.0",
  "ethicsId": "ETHICS-2025-000001",
  "dataSubjectId": "SUBJ-2025-001",
  "status": "active",
  "created": "2024-06-15T10:00:00Z",
  "lastReviewed": "2025-01-15T10:30:00Z",
  "consent": {
    "consentId": "CONSENT-2025-001",
    "granted": true,
    "level": "memorial_only",
    "grantedDate": "2024-06-15T10:00:00Z",
    "expiryDate": null,
    "witnessSignatures": [],
    "revocable": true
  },
  "dataRights": {
    "ownershipType": "family",
    "authorizedAgents": [],
    "accessControl": {},
    "usagePermissions": [],
    "commercialRights": {}
  },
  "personaParameters": {
    "allowedFidelity": 0.75,
    "trainingDataSources": [],
    "prohibitedBehaviors": [],
    "personalityConstraints": {},
    "interactionLimits": {}
  },
  "commercialRegulation": {
    "monetizationAllowed": false,
    "licensingTerms": {},
    "revenueSharing": {},
    "antiExploitation": {}
  },
  "psychologicalGuidelines": {
    "riskAssessment": {},
    "safetyMechanisms": [],
    "mandatoryDisclosures": [],
    "professionalOversight": {}
  },
  "meta": {
    "hash": "sha256:...",
    "signature": "...",
    "previousHash": "..."
  }
}
```

### 3.2 Field Details

#### 3.2.1 `ethicsId` (REQUIRED)

```
Type: string
Format: ETHICS-YYYY-NNNNNN
Description: Unique identifier for this ethics record
Example: "ETHICS-2025-000001"
```

#### 3.2.2 `status` (REQUIRED)

```
Type: string
Valid values:
  - "pending"       : Consent collection in progress
  - "active"        : Ethics record active, persona creation allowed
  - "suspended"     : Temporarily suspended by family
  - "expired"       : Consent period expired
  - "revoked"       : Consent permanently revoked
  - "archived"      : Historical record, no longer active
```

---

## Data Schema

### 4.1 Complete JSON Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wia.live/ai-afterlife-ethics/v1/schema.json",
  "title": "WIA AI Afterlife Ethics Record",
  "type": "object",
  "required": ["version", "ethicsId", "dataSubjectId", "status", "created", "consent", "dataRights", "psychologicalGuidelines"],
  "properties": {
    "version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+\\.\\d+$"
    },
    "ethicsId": {
      "type": "string",
      "pattern": "^ETHICS-\\d{4}-\\d{6}$"
    },
    "dataSubjectId": {
      "type": "string"
    },
    "status": {
      "type": "string",
      "enum": ["pending", "active", "suspended", "expired", "revoked", "archived"]
    },
    "created": {
      "type": "string",
      "format": "date-time"
    },
    "consent": {
      "type": "object",
      "required": ["consentId", "granted", "level"],
      "properties": {
        "consentId": {
          "type": "string",
          "pattern": "^CONSENT-\\d{4}-\\d{6}$"
        },
        "granted": { "type": "boolean" },
        "level": {
          "type": "string",
          "enum": ["none", "memorial_only", "family_interaction", "educational", "commercial_limited", "full_commercial"]
        },
        "grantedDate": { "type": "string", "format": "date-time" },
        "expiryDate": { "type": ["string", "null"], "format": "date-time" },
        "revocable": { "type": "boolean" }
      }
    },
    "dataRights": {
      "type": "object",
      "required": ["ownershipType", "authorizedAgents"],
      "properties": {
        "ownershipType": {
          "type": "string",
          "enum": ["individual", "family", "estate", "public_domain"]
        },
        "authorizedAgents": { "type": "array" },
        "accessControl": { "type": "object" },
        "usagePermissions": { "type": "array" }
      }
    },
    "personaParameters": {
      "type": "object",
      "properties": {
        "allowedFidelity": {
          "type": "number",
          "minimum": 0,
          "maximum": 1
        },
        "trainingDataSources": { "type": "array" },
        "prohibitedBehaviors": { "type": "array" },
        "personalityConstraints": { "type": "object" }
      }
    },
    "psychologicalGuidelines": {
      "type": "object",
      "required": ["riskAssessment", "mandatoryDisclosures"],
      "properties": {
        "riskAssessment": { "type": "object" },
        "safetyMechanisms": { "type": "array" },
        "mandatoryDisclosures": { "type": "array" }
      }
    }
  }
}
```

### 4.2 Consent Schema

```json
{
  "consent": {
    "consentId": "CONSENT-2025-001",
    "granted": true,
    "level": "family_interaction",
    "grantedDate": "2024-06-15T10:00:00Z",
    "expiryDate": "2074-06-15T10:00:00Z",
    "revocable": true,
    "witnessSignatures": [
      {
        "witnessId": "WITNESS-001",
        "name": "encrypted:...",
        "relationship": "family",
        "signature": "ed25519:...",
        "timestamp": "2024-06-15T10:00:00Z"
      },
      {
        "witnessId": "WITNESS-002",
        "name": "encrypted:...",
        "relationship": "legal",
        "signature": "ed25519:...",
        "timestamp": "2024-06-15T10:00:00Z"
      }
    ],
    "legalDocuments": [
      {
        "type": "consent_form",
        "documentId": "DOC-001",
        "hash": "sha256:...",
        "storageLocation": "ipfs://..."
      }
    ],
    "revocationProcedure": {
      "allowedBy": ["data_subject", "family_majority", "legal_guardian"],
      "notificationPeriod": 30,
      "effectiveDelay": 7
    }
  }
}
```

### 4.3 Data Rights Schema

```json
{
  "dataRights": {
    "ownershipType": "family",
    "primaryOwner": {
      "type": "estate",
      "identifier": "ESTATE-2025-001",
      "contactInfo": "encrypted:..."
    },
    "authorizedAgents": [
      {
        "agentId": "AGENT-001",
        "name": "encrypted:...",
        "role": "executor",
        "permissions": ["data_access", "consent_modification", "commercial_approval"],
        "validUntil": "2050-01-01T00:00:00Z"
      },
      {
        "agentId": "AGENT-002",
        "name": "encrypted:...",
        "role": "family_representative",
        "permissions": ["data_access", "interaction_monitoring"],
        "validUntil": null
      }
    ],
    "accessControl": {
      "publicAccess": false,
      "familyAccess": true,
      "researchAccess": false,
      "commercialAccess": false,
      "accessLog": true,
      "auditFrequency": "monthly"
    },
    "usagePermissions": [
      {
        "type": "memorial",
        "allowed": true,
        "conditions": ["family_only", "non_commercial"],
        "expiryDate": null
      },
      {
        "type": "educational",
        "allowed": true,
        "conditions": ["institutional_use", "attribution_required"],
        "expiryDate": "2035-01-01T00:00:00Z"
      }
    ]
  }
}
```

### 4.4 Persona Parameters Schema

```json
{
  "personaParameters": {
    "allowedFidelity": 0.75,
    "fidelityConstraints": {
      "voice": 0.8,
      "personality": 0.7,
      "knowledge": 0.6,
      "appearance": 0.8
    },
    "trainingDataSources": [
      {
        "sourceType": "social_media",
        "platform": "facebook",
        "dataRange": "2010-2024",
        "consentGranted": true,
        "hash": "sha256:..."
      },
      {
        "sourceType": "emails",
        "count": 15000,
        "dateRange": "2005-2024",
        "sanitized": true,
        "hash": "sha256:..."
      },
      {
        "sourceType": "voice_recordings",
        "duration": "120 hours",
        "quality": "high",
        "hash": "sha256:..."
      }
    ],
    "prohibitedBehaviors": [
      "political_endorsements",
      "commercial_advertising",
      "harmful_content",
      "impersonation_fraud",
      "romantic_relationships"
    ],
    "personalityConstraints": {
      "maintainCoreValues": true,
      "allowEvolution": false,
      "boundaryEnforcement": "strict",
      "contradictionHandling": "defer_to_original"
    },
    "interactionLimits": {
      "maxSessionDuration": 3600,
      "dailyInteractionLimit": 7200,
      "cooldownPeriod": 86400,
      "simultaneousUsers": 1
    }
  }
}
```

---

## Field Specifications

### 5.1 Consent Level Fields

| Level | Description | Permitted Uses | Restrictions |
|-------|-------------|----------------|--------------|
| `none` | No consent granted | None | All AI persona creation prohibited |
| `memorial_only` | Basic memorial | Static memorial page | No interaction, no commercial use |
| `family_interaction` | Family communication | Limited family interactions | Family only, non-commercial |
| `educational` | Academic/research | Educational institutions | Attribution required, non-profit |
| `commercial_limited` | Controlled commercial | Licensed commercial use | Revenue sharing, strict oversight |
| `full_commercial` | Full commercial rights | Unrestricted commercial | Subject to anti-exploitation rules |

### 5.2 Data Rights Ownership Types

| Type | Description | Control Authority |
|------|-------------|-------------------|
| `individual` | Pre-mortem individual control | Data subject's designated trustee |
| `family` | Family collective ownership | Majority family vote |
| `estate` | Legal estate ownership | Estate executor |
| `public_domain` | Public domain release | No individual control |

### 5.3 Psychological Risk Levels

| Level | Risk Score | Required Safeguards |
|-------|------------|---------------------|
| `minimal` | 0.0 - 0.2 | Standard disclosures |
| `low` | 0.2 - 0.4 | Usage monitoring |
| `moderate` | 0.4 - 0.6 | Professional consultation recommended |
| `high` | 0.6 - 0.8 | Mandatory professional oversight |
| `severe` | 0.8 - 1.0 | Interaction prohibited or highly restricted |

---

## Data Types

### 6.1 Custom Types

```typescript
type ConsentLevel =
  | 'none'
  | 'memorial_only'
  | 'family_interaction'
  | 'educational'
  | 'commercial_limited'
  | 'full_commercial';

type OwnershipType =
  | 'individual'
  | 'family'
  | 'estate'
  | 'public_domain';

type UsageType =
  | 'memorial'
  | 'family_interaction'
  | 'educational'
  | 'research'
  | 'commercial'
  | 'therapeutic';

type RiskLevel =
  | 'minimal'
  | 'low'
  | 'moderate'
  | 'high'
  | 'severe';

interface ConsentRecord {
  consentId: string;
  granted: boolean;
  level: ConsentLevel;
  grantedDate: string;
  expiryDate: string | null;
  revocable: boolean;
}

interface DataRights {
  ownershipType: OwnershipType;
  authorizedAgents: Agent[];
  accessControl: AccessControl;
  usagePermissions: UsagePermission[];
}

interface PsychologicalRiskAssessment {
  riskLevel: RiskLevel;
  riskScore: number;
  assessmentDate: string;
  assessedBy: string;
  factors: RiskFactor[];
  mitigationPlan: MitigationPlan;
}
```

### 6.2 Enum Values

#### Prohibited Behaviors

| Code | Behavior | Description |
|------|----------|-------------|
| `political_endorsements` | Political activity | No political campaigning or endorsements |
| `commercial_advertising` | Product promotion | No advertising without explicit consent |
| `harmful_content` | Harmful output | No violence, hate speech, or harmful content |
| `impersonation_fraud` | Identity fraud | No fraudulent impersonation |
| `romantic_relationships` | Romantic interaction | No romantic/sexual relationships |
| `medical_advice` | Medical guidance | No medical or therapeutic advice |

---

## Validation Rules

### 7.1 Required Field Validation

| Rule ID | Field | Validation |
|---------|-------|------------|
| VAL-001 | `ethicsId` | Must match `^ETHICS-\d{4}-\d{6}$` |
| VAL-002 | `consent.granted` | Must be true if status is "active" |
| VAL-003 | `consent.witnessSignatures` | At least 2 witnesses required |
| VAL-004 | `dataRights.authorizedAgents` | At least 1 authorized agent required |
| VAL-005 | `personaParameters.allowedFidelity` | Must be 0.0 to 1.0 |
| VAL-006 | `psychologicalGuidelines.riskAssessment` | Required for all active records |

### 7.2 Business Logic Validation

| Rule ID | Description | Error Code |
|---------|-------------|------------|
| BUS-001 | Commercial use requires commercial_limited or full_commercial consent | `ERR_INSUFFICIENT_CONSENT` |
| BUS-002 | High-risk assessments require professional oversight | `ERR_MISSING_OVERSIGHT` |
| BUS-003 | Expired consent automatically suspends persona | `ERR_CONSENT_EXPIRED` |
| BUS-004 | Family majority required to revoke family ownership | `ERR_INSUFFICIENT_AUTHORITY` |
| BUS-005 | Fidelity level cannot exceed consent level limits | `ERR_FIDELITY_VIOLATION` |

### 7.3 Error Codes

| Code | Message | Description |
|------|---------|-------------|
| `ERR_NO_CONSENT` | Consent not granted | Consent required but not provided |
| `ERR_INSUFFICIENT_CONSENT` | Insufficient consent level | Usage exceeds consent level |
| `ERR_CONSENT_EXPIRED` | Consent period expired | Time-limited consent expired |
| `ERR_UNAUTHORIZED_AGENT` | Unauthorized agent | Agent lacks required permissions |
| `ERR_PSYCHOLOGICAL_RISK` | Psychological risk too high | Risk assessment prevents usage |
| `ERR_COMMERCIAL_VIOLATION` | Commercial terms violated | Commercial usage rules violated |
| `ERR_FIDELITY_VIOLATION` | Fidelity exceeds limits | Persona too realistic for consent level |

---

## Examples

### 8.1 Valid Ethics Record - Family Memorial

```json
{
  "$schema": "https://wia.live/ai-afterlife-ethics/v1/schema.json",
  "version": "1.0.0",
  "ethicsId": "ETHICS-2025-000001",
  "dataSubjectId": "SUBJ-2025-001",
  "status": "active",
  "created": "2024-06-15T10:00:00Z",
  "lastReviewed": "2025-01-15T10:30:00Z",
  "consent": {
    "consentId": "CONSENT-2025-001",
    "granted": true,
    "level": "family_interaction",
    "grantedDate": "2024-06-15T10:00:00Z",
    "expiryDate": null,
    "revocable": true,
    "witnessSignatures": [
      {
        "witnessId": "WITNESS-001",
        "name": "encrypted:aes256:...",
        "relationship": "family",
        "signature": "ed25519:abc123...",
        "timestamp": "2024-06-15T10:00:00Z"
      },
      {
        "witnessId": "WITNESS-002",
        "name": "encrypted:aes256:...",
        "relationship": "legal",
        "signature": "ed25519:def456...",
        "timestamp": "2024-06-15T10:00:00Z"
      }
    ]
  },
  "dataRights": {
    "ownershipType": "family",
    "authorizedAgents": [
      {
        "agentId": "AGENT-001",
        "name": "encrypted:aes256:...",
        "role": "family_representative",
        "permissions": ["data_access", "interaction_monitoring"],
        "validUntil": null
      }
    ],
    "accessControl": {
      "publicAccess": false,
      "familyAccess": true,
      "commercialAccess": false
    },
    "usagePermissions": [
      {
        "type": "memorial",
        "allowed": true,
        "conditions": ["family_only", "non_commercial"]
      }
    ]
  },
  "personaParameters": {
    "allowedFidelity": 0.75,
    "trainingDataSources": [
      {
        "sourceType": "social_media",
        "platform": "facebook",
        "dataRange": "2010-2024"
      }
    ],
    "prohibitedBehaviors": [
      "political_endorsements",
      "commercial_advertising",
      "romantic_relationships"
    ],
    "interactionLimits": {
      "maxSessionDuration": 3600,
      "dailyInteractionLimit": 7200
    }
  },
  "psychologicalGuidelines": {
    "riskAssessment": {
      "riskLevel": "low",
      "riskScore": 0.3,
      "assessmentDate": "2024-06-15T10:00:00Z"
    },
    "mandatoryDisclosures": [
      "AI-generated persona",
      "Not the actual person",
      "Based on historical data"
    ],
    "safetyMechanisms": [
      "session_time_limits",
      "cooldown_periods",
      "grief_counseling_referral"
    ]
  },
  "meta": {
    "hash": "sha256:ethics_record_hash...",
    "signature": "ed25519:signature..."
  }
}
```

### 8.2 Valid Ethics Record - Educational Use

```json
{
  "ethicsId": "ETHICS-2025-000002",
  "dataSubjectId": "SUBJ-2025-002",
  "status": "active",
  "consent": {
    "consentId": "CONSENT-2025-002",
    "granted": true,
    "level": "educational",
    "grantedDate": "2024-08-01T10:00:00Z",
    "expiryDate": "2034-08-01T10:00:00Z"
  },
  "dataRights": {
    "ownershipType": "estate",
    "usagePermissions": [
      {
        "type": "educational",
        "allowed": true,
        "conditions": ["institutional_use", "attribution_required", "non_profit"]
      }
    ]
  },
  "personaParameters": {
    "allowedFidelity": 0.6,
    "prohibitedBehaviors": [
      "political_endorsements",
      "commercial_advertising",
      "romantic_relationships",
      "medical_advice"
    ]
  }
}
```

### 8.3 Invalid Example - Missing Consent

```json
{
  "ethicsId": "ETHICS-2025-000003",
  "dataSubjectId": "SUBJ-2025-003",
  "status": "active",
  "consent": {
    "granted": false
  }
}
```

**Error**: `ERR_NO_CONSENT` - Active status requires granted consent

### 8.4 Invalid Example - Insufficient Witnesses

```json
{
  "ethicsId": "ETHICS-2025-000004",
  "dataSubjectId": "SUBJ-2025-004",
  "status": "active",
  "consent": {
    "consentId": "CONSENT-2025-004",
    "granted": true,
    "level": "family_interaction",
    "witnessSignatures": [
      {
        "witnessId": "WITNESS-001",
        "signature": "ed25519:..."
      }
    ]
  }
}
```

**Error**: `ERR_VALIDATION_FAILED` - At least 2 witnesses required

### 8.5 Invalid Example - Commercial Without Authorization

```json
{
  "ethicsId": "ETHICS-2025-000005",
  "status": "active",
  "consent": {
    "granted": true,
    "level": "memorial_only"
  },
  "commercialRegulation": {
    "monetizationAllowed": true
  }
}
```

**Error**: `ERR_INSUFFICIENT_CONSENT` - Commercial use requires commercial_limited or full_commercial consent level

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01 | Initial release |

---

<div align="center">

**WIA AI Afterlife Ethics Standard v1.0.0**

**弘益人間 (홍익인간)** - Benefit All Humanity

---

**© 2025 WIA**

**MIT License**

</div>
