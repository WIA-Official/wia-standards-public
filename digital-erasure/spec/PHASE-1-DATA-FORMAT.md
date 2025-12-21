# WIA Digital Erasure Data Format Standard
## Phase 1 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-12-18
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

The WIA Digital Erasure Data Format Standard defines a unified JSON-based format for recording, transmitting, and managing digital presence deletion requests, erasure procedures, verification records, and compliance documentation for post-mortem digital asset removal across platforms and service providers worldwide.

**Core Objectives**:
- Standardize digital erasure request formats across all platforms
- Enable interoperability between digital executors and service providers
- Ensure complete data removal with cryptographic verification
- Support regulatory compliance (GDPR, CCPA, right to be forgotten)
- Facilitate transparent audit trails for legal compliance

### 1.2 Scope

This standard covers the following data domains:

| Domain | Description |
|--------|-------------|
| Digital Footprint Inventory | Comprehensive mapping of all digital accounts and services |
| Erasure Requests | Formal deletion requests with legal authentication |
| Deletion Verification | Cryptographic proof of complete data removal |
| Account Tracking | Real-time status of erasure across platforms |
| Compliance Documentation | GDPR Article 17 and legal compliance records |

### 1.3 Design Principles

1. **Completeness**: Comprehensive coverage of all digital presence types
2. **Verifiability**: Cryptographic proof of deletion at every step
3. **Legal Compliance**: Built-in GDPR, CCPA, and international law support
4. **Privacy-Preserving**: Minimal data exposure during erasure process
5. **Immutability**: Blockchain-ready audit trails

---

## Terminology

### 2.1 Core Terms

| Term | Definition |
|------|------------|
| **Digital Decedent** | Individual whose digital presence is being erased |
| **Digital Executor** | Authorized person managing post-mortem digital erasure |
| **Footprint Inventory** | Complete catalog of digital accounts and services |
| **Erasure Request** | Formal deletion demand with legal authentication |
| **Crypto-Shred** | Cryptographically secure data destruction method |
| **Multi-Pass Erasure** | Multiple overwrite cycles for secure deletion |
| **Verification Hash** | Cryptographic proof of successful deletion |
| **Right to be Forgotten** | GDPR Article 17 erasure right |

### 2.2 Data Types

| Type | Description | Example |
|------|-------------|---------|
| `string` | UTF-8 encoded text | `"WIA-ERASE-001"` |
| `number` | IEEE 754 double precision | `3.0`, `7.0` |
| `integer` | Signed 64-bit integer | `1`, `35` |
| `boolean` | Boolean value | `true`, `false` |
| `timestamp` | ISO 8601 datetime | `"2025-12-18T10:30:00Z"` |
| `uuid` | UUID v4 identifier | `"550e8400-e29b-41d4-a716-446655440000"` |
| `hash` | SHA-256 hash | `"b6c8d5e7f9a2b3c4d5e6f7a8b9c0d1e2..."` |

### 2.3 Field Requirements

| Marker | Meaning |
|--------|---------|
| **REQUIRED** | Field must be present |
| **OPTIONAL** | Field may be omitted |
| **CONDITIONAL** | Required under specific conditions |

---

## Base Structure

### 3.1 Message Format

All WIA Digital Erasure messages follow this base structure:

```json
{
  "$schema": "https://wia.live/digital-erasure/v1/schema.json",
  "version": "1.0.0",
  "messageId": "uuid-v4-string",
  "messageType": "erasure_request",
  "timestamp": {
    "created": "2025-12-18T10:30:00Z",
    "modified": "2025-12-18T10:30:00Z"
  },
  "decedent": {
    "id": "DEC-001",
    "anonymizedId": "ANON-550e8400",
    "deathCertificateId": "DC-2025-001",
    "dateOfDeath": "2025-12-01T00:00:00Z"
  },
  "executor": {
    "id": "EXEC-001",
    "name": "Jane Doe",
    "email": "executor@example.com",
    "authenticationMethod": "government_id",
    "verified": true,
    "verificationTimestamp": "2025-12-18T09:00:00Z"
  },
  "data": {
    // Domain-specific data
  },
  "meta": {
    "hash": "sha256-hash",
    "signature": "digital-signature",
    "previousHash": "previous-record-hash",
    "blockchainAnchor": "blockchain-tx-id"
  }
}
```

### 3.2 Field Details

#### 3.2.1 `$schema` (OPTIONAL)

```
Type: string
Format: URI
Description: JSON Schema location for validation
Example: "https://wia.live/digital-erasure/v1/schema.json"
```

#### 3.2.2 `version` (REQUIRED)

```
Type: string
Format: Semantic Versioning (MAJOR.MINOR.PATCH)
Description: Specification version
Example: "1.0.0"
```

#### 3.2.3 `messageId` (REQUIRED)

```
Type: string
Format: UUID v4
Description: Unique identifier for this message
Example: "550e8400-e29b-41d4-a716-446655440000"
```

#### 3.2.4 `messageType` (REQUIRED)

```
Type: string
Description: Type of message
Valid values:
  - "footprint_inventory"    : Digital presence catalog
  - "erasure_request"        : Deletion request
  - "deletion_status"        : Real-time deletion progress
  - "verification_proof"     : Cryptographic deletion proof
  - "compliance_report"      : GDPR/legal compliance documentation
```

---

## Data Schema

### 4.1 Complete JSON Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wia.live/digital-erasure/v1/schema.json",
  "title": "WIA Digital Erasure Record",
  "type": "object",
  "required": ["version", "messageId", "messageType", "timestamp", "decedent", "executor", "data"],
  "properties": {
    "version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+\\.\\d+$"
    },
    "messageId": {
      "type": "string",
      "format": "uuid"
    },
    "messageType": {
      "type": "string",
      "enum": ["footprint_inventory", "erasure_request", "deletion_status", "verification_proof", "compliance_report"]
    },
    "timestamp": {
      "type": "object",
      "required": ["created"],
      "properties": {
        "created": { "type": "string", "format": "date-time" },
        "modified": { "type": "string", "format": "date-time" }
      }
    },
    "decedent": {
      "type": "object",
      "required": ["id", "deathCertificateId", "dateOfDeath"],
      "properties": {
        "id": { "type": "string" },
        "anonymizedId": { "type": "string" },
        "deathCertificateId": { "type": "string" },
        "dateOfDeath": { "type": "string", "format": "date-time" },
        "fullName": { "type": "string" },
        "dateOfBirth": { "type": "string", "format": "date" },
        "nationalId": { "type": "string" }
      }
    },
    "executor": {
      "type": "object",
      "required": ["id", "name", "authenticationMethod", "verified"],
      "properties": {
        "id": { "type": "string" },
        "name": { "type": "string" },
        "email": { "type": "string", "format": "email" },
        "phone": { "type": "string" },
        "authenticationMethod": {
          "type": "string",
          "enum": ["government_id", "probate_court", "notarized_will", "digital_certificate"]
        },
        "verified": { "type": "boolean" },
        "verificationTimestamp": { "type": "string", "format": "date-time" },
        "authorizationDocument": { "type": "string" }
      }
    },
    "data": {
      "type": "object"
    },
    "meta": {
      "type": "object",
      "properties": {
        "hash": { "type": "string" },
        "signature": { "type": "string" },
        "previousHash": { "type": "string" },
        "blockchainAnchor": { "type": "string" },
        "version": { "type": "integer" }
      }
    }
  }
}
```

### 4.2 Digital Footprint Inventory Schema

```json
{
  "data": {
    "inventoryType": "comprehensive",
    "scanDate": "2025-12-18T10:00:00Z",
    "totalAccounts": 47,
    "accountCategories": {
      "social_media": 8,
      "email_messaging": 5,
      "cloud_storage": 6,
      "financial_services": 12,
      "subscriptions": 10,
      "professional_networks": 4,
      "other": 2
    },
    "accounts": [
      {
        "accountId": "ACC-001",
        "platform": "facebook",
        "platformType": "social_media",
        "accountIdentifier": "user@example.com",
        "accountUsername": "john_doe",
        "accountUrl": "https://facebook.com/john_doe",
        "creationDate": "2010-05-15T00:00:00Z",
        "lastActivity": "2025-11-30T15:22:00Z",
        "dataVolume": {
          "posts": 1234,
          "photos": 567,
          "videos": 89,
          "connections": 432,
          "messages": 8901,
          "estimatedSizeGB": 12.5
        },
        "erasureMethod": "api_deletion",
        "gdprCompliant": true,
        "priority": "high",
        "status": "pending"
      },
      {
        "accountId": "ACC-002",
        "platform": "google",
        "platformType": "email_messaging",
        "accountIdentifier": "user@gmail.com",
        "services": ["gmail", "drive", "photos", "youtube", "calendar"],
        "dataVolume": {
          "emails": 45623,
          "driveFilesGB": 145.8,
          "photos": 12345,
          "videos": 234,
          "estimatedSizeGB": 189.3
        },
        "erasureMethod": "account_deletion",
        "gdprCompliant": true,
        "priority": "critical",
        "status": "pending"
      }
    ],
    "deletionStrategy": {
      "phaseApproach": "tiered_priority",
      "estimatedDuration": "14-30 days",
      "verificationMethod": "cryptographic_hash",
      "auditTrail": true
    }
  }
}
```

### 4.3 Erasure Request Schema

```json
{
  "data": {
    "requestType": "complete_erasure",
    "requestDate": "2025-12-18T10:30:00Z",
    "legalBasis": "gdpr_article_17",
    "scope": "all_digital_presence",
    "deletionMethod": {
      "algorithm": "crypto_shred",
      "passes": 7,
      "standard": "DoD_5220_22_M",
      "verificationRequired": true
    },
    "targetAccounts": [
      {
        "accountId": "ACC-001",
        "platform": "facebook",
        "requestedAction": "permanent_deletion",
        "dataRetention": "none",
        "downloadDataFirst": false,
        "notifyConnections": false,
        "memorialization": false
      }
    ],
    "timeline": {
      "requestSubmitted": "2025-12-18T10:30:00Z",
      "expectedCompletion": "2026-01-18T00:00:00Z",
      "gracePeriodDays": 30
    },
    "compliance": {
      "gdprArticle17": true,
      "ccpaCompliant": true,
      "localLawsReviewed": true,
      "legalCounselApproved": true
    }
  }
}
```

---

## Field Specifications

### 5.1 Account Fields

| Field | Type | Required | Description | Example |
|-------|------|----------|-------------|---------|
| `accountId` | string | REQUIRED | Unique account identifier | `"ACC-001"` |
| `platform` | string | REQUIRED | Platform name | `"facebook"` |
| `platformType` | string | REQUIRED | Category of platform | `"social_media"` |
| `accountIdentifier` | string | REQUIRED | Primary identifier (email/username) | `"user@example.com"` |
| `accountUrl` | string | OPTIONAL | Profile URL | `"https://facebook.com/user"` |

**Valid platformType values:**

| Value | Description |
|-------|-------------|
| `social_media` | Facebook, Twitter, Instagram, LinkedIn, TikTok |
| `email_messaging` | Gmail, Outlook, WhatsApp, Telegram, Signal |
| `cloud_storage` | Google Drive, Dropbox, OneDrive, iCloud |
| `financial_services` | Banks, PayPal, crypto exchanges, investment platforms |
| `subscriptions` | Netflix, Spotify, SaaS services, memberships |
| `professional_networks` | LinkedIn, GitHub, Stack Overflow, ResearchGate |
| `health_fitness` | Apple Health, Fitbit, medical portals |
| `gaming` | Steam, PlayStation, Xbox, Nintendo |
| `other` | Miscellaneous services |

### 5.2 Deletion Method Fields

| Field | Type | Required | Description | Range |
|-------|------|----------|-------------|-------|
| `algorithm` | string | REQUIRED | Deletion algorithm | See enum |
| `passes` | integer | REQUIRED | Number of overwrite passes | 1-35 |
| `standard` | string | REQUIRED | Compliance standard | See enum |
| `verificationRequired` | boolean | REQUIRED | Require cryptographic proof | true/false |

**Valid algorithm values:**

| Algorithm | Description | Passes |
|-----------|-------------|--------|
| `simple_delete` | Standard deletion (file system only) | 1 |
| `secure_erase` | Single overwrite with random data | 1 |
| `DoD_5220_22_M` | US DoD standard (3 passes) | 3 |
| `Gutmann` | Peter Gutmann's method (35 passes) | 35 |
| `crypto_shred` | Cryptographic key destruction | 1 |
| `multi_pass_random` | Multiple random overwrites | 7 |

### 5.3 Compliance Fields

| Field | Type | Required | Description | Example |
|-------|------|----------|-------------|---------|
| `gdprArticle17` | boolean | REQUIRED | GDPR right to erasure | `true` |
| `ccpaCompliant` | boolean | REQUIRED | CCPA compliance | `true` |
| `localLawsReviewed` | boolean | REQUIRED | Local law review complete | `true` |
| `legalCounselApproved` | boolean | OPTIONAL | Legal approval obtained | `true` |

### 5.4 Status Values

| Status | Description |
|--------|-------------|
| `pending` | Awaiting processing |
| `authentication_required` | Executor verification needed |
| `in_progress` | Deletion underway |
| `verification_pending` | Awaiting deletion proof |
| `completed` | Successfully deleted |
| `failed` | Deletion failed |
| `partial` | Partially deleted |
| `archived` | Data archived instead of deleted |

---

## Data Types

### 6.1 Custom Types

#### PlatformType

```typescript
type PlatformType =
  | 'social_media'
  | 'email_messaging'
  | 'cloud_storage'
  | 'financial_services'
  | 'subscriptions'
  | 'professional_networks'
  | 'health_fitness'
  | 'gaming'
  | 'other';
```

#### DeletionAlgorithm

```typescript
type DeletionAlgorithm =
  | 'simple_delete'
  | 'secure_erase'
  | 'DoD_5220_22_M'
  | 'Gutmann'
  | 'crypto_shred'
  | 'multi_pass_random';
```

#### ErasureStatus

```typescript
type ErasureStatus =
  | 'pending'
  | 'authentication_required'
  | 'in_progress'
  | 'verification_pending'
  | 'completed'
  | 'failed'
  | 'partial'
  | 'archived';
```

#### AccountData

```typescript
interface AccountData {
  accountId: string;
  platform: string;
  platformType: PlatformType;
  accountIdentifier: string;
  accountUsername?: string;
  accountUrl?: string;
  creationDate?: string;
  lastActivity?: string;
  dataVolume?: {
    posts?: number;
    photos?: number;
    videos?: number;
    connections?: number;
    messages?: number;
    estimatedSizeGB?: number;
  };
  erasureMethod: string;
  gdprCompliant: boolean;
  priority: 'low' | 'medium' | 'high' | 'critical';
  status: ErasureStatus;
}
```

### 6.2 Enum Values

#### Erasure Priority

| Priority | Description | SLA |
|----------|-------------|-----|
| `critical` | Immediate deletion required | 24 hours |
| `high` | Priority deletion | 7 days |
| `medium` | Standard deletion | 30 days |
| `low` | Low priority | 90 days |

#### Legal Basis

| Code | Description |
|------|-------------|
| `gdpr_article_17` | GDPR Right to Erasure |
| `ccpa_deletion` | California Consumer Privacy Act |
| `post_mortem_right` | Post-mortem digital rights |
| `executor_authority` | Legal executor authority |
| `probate_court_order` | Court-ordered deletion |

---

## Validation Rules

### 7.1 Required Field Validation

| Rule ID | Field | Validation |
|---------|-------|------------|
| VAL-001 | `version` | Must match `^\d+\.\d+\.\d+$` |
| VAL-002 | `messageId` | Must be valid UUID v4 |
| VAL-003 | `timestamp.created` | Must be valid ISO 8601 |
| VAL-004 | `decedent.deathCertificateId` | Must not be empty |
| VAL-005 | `executor.verified` | Must be true for processing |
| VAL-006 | `executor.authenticationMethod` | Must be valid enum value |

### 7.2 Business Logic Validation

| Rule ID | Description | Error Code |
|---------|-------------|------------|
| BUS-001 | Death date must be before request date | `ERR_INVALID_DATE_SEQUENCE` |
| BUS-002 | Executor must be verified before submission | `ERR_UNVERIFIED_EXECUTOR` |
| BUS-003 | Deletion passes must be 1-35 | `ERR_INVALID_PASS_COUNT` |
| BUS-004 | Critical priority requires verification | `ERR_VERIFICATION_REQUIRED` |
| BUS-005 | GDPR requests must complete within 30 days | `ERR_SLA_VIOLATION` |
| BUS-006 | Account identifier must be valid email or username | `ERR_INVALID_IDENTIFIER` |

### 7.3 Error Codes

| Code | Message | Description |
|------|---------|-------------|
| `ERR_INVALID_FORMAT` | Invalid data format | JSON parsing failed |
| `ERR_MISSING_FIELD` | Required field missing | Required field not present |
| `ERR_INVALID_TYPE` | Invalid field type | Type mismatch |
| `ERR_INVALID_DATE_SEQUENCE` | Invalid date sequence | Dates out of logical order |
| `ERR_UNVERIFIED_EXECUTOR` | Executor not verified | Executor verification required |
| `ERR_INVALID_PASS_COUNT` | Invalid pass count | Passes outside 1-35 range |
| `ERR_VERIFICATION_REQUIRED` | Verification required | Critical priority needs verification |
| `ERR_INVALID_IDENTIFIER` | Invalid account identifier | Email/username format invalid |

---

## Examples

### 8.1 Complete Digital Footprint Inventory

```json
{
  "$schema": "https://wia.live/digital-erasure/v1/schema.json",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440001",
  "messageType": "footprint_inventory",
  "timestamp": {
    "created": "2025-12-18T10:00:00Z"
  },
  "decedent": {
    "id": "DEC-2025-001",
    "anonymizedId": "ANON-550e8400",
    "deathCertificateId": "DC-KR-2025-001",
    "dateOfDeath": "2025-12-01T00:00:00Z"
  },
  "executor": {
    "id": "EXEC-2025-001",
    "name": "Jane Doe",
    "email": "executor@example.com",
    "authenticationMethod": "government_id",
    "verified": true,
    "verificationTimestamp": "2025-12-18T09:00:00Z"
  },
  "data": {
    "inventoryType": "comprehensive",
    "scanDate": "2025-12-18T10:00:00Z",
    "totalAccounts": 47,
    "accountCategories": {
      "social_media": 8,
      "email_messaging": 5,
      "cloud_storage": 6,
      "financial_services": 12,
      "subscriptions": 10,
      "professional_networks": 4,
      "other": 2
    },
    "accounts": [
      {
        "accountId": "ACC-001",
        "platform": "facebook",
        "platformType": "social_media",
        "accountIdentifier": "user@example.com",
        "accountUsername": "john_doe",
        "accountUrl": "https://facebook.com/john_doe",
        "creationDate": "2010-05-15T00:00:00Z",
        "lastActivity": "2025-11-30T15:22:00Z",
        "dataVolume": {
          "posts": 1234,
          "photos": 567,
          "videos": 89,
          "connections": 432,
          "messages": 8901,
          "estimatedSizeGB": 12.5
        },
        "erasureMethod": "api_deletion",
        "gdprCompliant": true,
        "priority": "high",
        "status": "pending"
      },
      {
        "accountId": "ACC-002",
        "platform": "google",
        "platformType": "email_messaging",
        "accountIdentifier": "user@gmail.com",
        "services": ["gmail", "drive", "photos", "youtube", "calendar"],
        "dataVolume": {
          "emails": 45623,
          "driveFilesGB": 145.8,
          "photos": 12345,
          "videos": 234,
          "estimatedSizeGB": 189.3
        },
        "erasureMethod": "account_deletion",
        "gdprCompliant": true,
        "priority": "critical",
        "status": "pending"
      },
      {
        "accountId": "ACC-003",
        "platform": "twitter",
        "platformType": "social_media",
        "accountIdentifier": "@john_doe_twitter",
        "accountUrl": "https://twitter.com/john_doe_twitter",
        "creationDate": "2009-03-21T00:00:00Z",
        "lastActivity": "2025-11-29T08:15:00Z",
        "dataVolume": {
          "posts": 8934,
          "photos": 234,
          "videos": 45,
          "connections": 1567,
          "estimatedSizeGB": 3.2
        },
        "erasureMethod": "api_deletion",
        "gdprCompliant": true,
        "priority": "high",
        "status": "pending"
      },
      {
        "accountId": "ACC-004",
        "platform": "linkedin",
        "platformType": "professional_networks",
        "accountIdentifier": "user@example.com",
        "accountUrl": "https://linkedin.com/in/johndoe",
        "creationDate": "2008-06-10T00:00:00Z",
        "lastActivity": "2025-11-28T14:30:00Z",
        "dataVolume": {
          "posts": 456,
          "connections": 892,
          "messages": 2345,
          "estimatedSizeGB": 1.8
        },
        "erasureMethod": "account_deletion",
        "gdprCompliant": true,
        "priority": "medium",
        "status": "pending"
      },
      {
        "accountId": "ACC-005",
        "platform": "dropbox",
        "platformType": "cloud_storage",
        "accountIdentifier": "user@example.com",
        "dataVolume": {
          "files": 5678,
          "folders": 234,
          "estimatedSizeGB": 89.4
        },
        "erasureMethod": "crypto_shred",
        "gdprCompliant": true,
        "priority": "critical",
        "status": "pending"
      }
    ],
    "deletionStrategy": {
      "phaseApproach": "tiered_priority",
      "estimatedDuration": "14-30 days",
      "verificationMethod": "cryptographic_hash",
      "auditTrail": true
    }
  },
  "meta": {
    "hash": "sha256:a1b2c3d4e5f6...",
    "signature": "eyJhbGciOiJFUzI1NiIs...",
    "version": 1
  }
}
```

### 8.2 Erasure Request with Crypto-Shred

```json
{
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440002",
  "messageType": "erasure_request",
  "timestamp": {
    "created": "2025-12-18T11:00:00Z"
  },
  "decedent": {
    "id": "DEC-2025-001",
    "deathCertificateId": "DC-KR-2025-001",
    "dateOfDeath": "2025-12-01T00:00:00Z"
  },
  "executor": {
    "id": "EXEC-2025-001",
    "name": "Jane Doe",
    "email": "executor@example.com",
    "authenticationMethod": "probate_court",
    "verified": true,
    "verificationTimestamp": "2025-12-18T09:00:00Z",
    "authorizationDocument": "COURT-ORDER-2025-001"
  },
  "data": {
    "requestType": "complete_erasure",
    "requestDate": "2025-12-18T11:00:00Z",
    "legalBasis": "gdpr_article_17",
    "scope": "all_digital_presence",
    "deletionMethod": {
      "algorithm": "crypto_shred",
      "passes": 7,
      "standard": "DoD_5220_22_M",
      "verificationRequired": true
    },
    "targetAccounts": [
      {
        "accountId": "ACC-001",
        "platform": "facebook",
        "requestedAction": "permanent_deletion",
        "dataRetention": "none",
        "downloadDataFirst": false,
        "notifyConnections": false,
        "memorialization": false
      },
      {
        "accountId": "ACC-002",
        "platform": "google",
        "requestedAction": "permanent_deletion",
        "dataRetention": "none",
        "downloadDataFirst": true,
        "archiveLocation": "encrypted://backup/google-archive-001.enc",
        "notifyConnections": false
      },
      {
        "accountId": "ACC-005",
        "platform": "dropbox",
        "requestedAction": "crypto_shred_deletion",
        "dataRetention": "none",
        "downloadDataFirst": false,
        "overwritePasses": 7
      }
    ],
    "timeline": {
      "requestSubmitted": "2025-12-18T11:00:00Z",
      "expectedCompletion": "2026-01-18T00:00:00Z",
      "gracePeriodDays": 30
    },
    "compliance": {
      "gdprArticle17": true,
      "ccpaCompliant": true,
      "localLawsReviewed": true,
      "legalCounselApproved": true,
      "courtOrderNumber": "COURT-ORDER-2025-001"
    }
  }
}
```

### 8.3 Deletion Verification Proof

```json
{
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440003",
  "messageType": "verification_proof",
  "timestamp": {
    "created": "2025-12-20T15:30:00Z"
  },
  "decedent": {
    "id": "DEC-2025-001",
    "anonymizedId": "ANON-550e8400"
  },
  "executor": {
    "id": "EXEC-2025-001",
    "name": "Jane Doe",
    "verified": true
  },
  "data": {
    "accountId": "ACC-001",
    "platform": "facebook",
    "deletionMethod": "api_deletion",
    "deletionTimestamp": "2025-12-20T15:00:00Z",
    "verificationMethod": "cryptographic_hash",
    "preDeleteionHash": "sha256:a1b2c3d4e5f6a7b8c9d0e1f2...",
    "postDeletionHash": "sha256:0000000000000000000000000...",
    "hashAlgorithm": "SHA-256",
    "verificationStatus": "confirmed_deleted",
    "proofOfDeletion": {
      "platformConfirmation": true,
      "confirmationId": "FB-DEL-2025-12-20-001",
      "deletionReceipt": "https://facebook.com/deletion/receipt/ABC123",
      "apiResponse": {
        "status": "deleted",
        "timestamp": "2025-12-20T15:00:00Z",
        "permanent": true
      }
    },
    "auditTrail": [
      {
        "timestamp": "2025-12-18T11:00:00Z",
        "action": "deletion_requested",
        "actor": "EXEC-2025-001"
      },
      {
        "timestamp": "2025-12-18T11:05:00Z",
        "action": "request_submitted",
        "platform": "facebook"
      },
      {
        "timestamp": "2025-12-20T14:00:00Z",
        "action": "deletion_initiated",
        "platform": "facebook"
      },
      {
        "timestamp": "2025-12-20T15:00:00Z",
        "action": "deletion_completed",
        "platform": "facebook"
      },
      {
        "timestamp": "2025-12-20T15:30:00Z",
        "action": "verification_completed",
        "verifier": "WIA-VERIFICATION-SERVICE"
      }
    ]
  },
  "meta": {
    "hash": "sha256:c7d8e9f0a1b2...",
    "signature": "eyJhbGciOiJFUzI1NiIs...",
    "previousHash": "sha256:a1b2c3d4e5f6...",
    "blockchainAnchor": "ethereum:0x1234567890abcdef...",
    "version": 3
  }
}
```

### 8.4 Invalid Example - Unverified Executor

```json
{
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440004",
  "messageType": "erasure_request",
  "timestamp": { "created": "2025-12-18T11:00:00Z" },
  "decedent": {
    "id": "DEC-001",
    "deathCertificateId": "DC-001",
    "dateOfDeath": "2025-12-01T00:00:00Z"
  },
  "executor": {
    "id": "EXEC-001",
    "name": "Unknown Person",
    "verified": false
  },
  "data": {
    "requestType": "complete_erasure"
  }
}
```

**Error**: `ERR_UNVERIFIED_EXECUTOR` - Executor must be verified before processing erasure request

### 8.5 Complete Multi-Platform Erasure

```json
{
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440005",
  "messageType": "deletion_status",
  "timestamp": {
    "created": "2025-12-25T10:00:00Z",
    "modified": "2025-12-25T10:00:00Z"
  },
  "decedent": {
    "id": "DEC-2025-001",
    "anonymizedId": "ANON-550e8400"
  },
  "executor": {
    "id": "EXEC-2025-001",
    "verified": true
  },
  "data": {
    "overallStatus": "in_progress",
    "completionPercentage": 68.0,
    "accountsProcessed": 32,
    "accountsTotal": 47,
    "accountsCompleted": 25,
    "accountsFailed": 2,
    "accountsPending": 15,
    "statusByCategory": {
      "social_media": {
        "total": 8,
        "completed": 6,
        "in_progress": 1,
        "failed": 1,
        "pending": 0
      },
      "email_messaging": {
        "total": 5,
        "completed": 4,
        "in_progress": 1,
        "failed": 0,
        "pending": 0
      },
      "cloud_storage": {
        "total": 6,
        "completed": 5,
        "in_progress": 0,
        "failed": 0,
        "pending": 1
      },
      "financial_services": {
        "total": 12,
        "completed": 5,
        "in_progress": 2,
        "failed": 1,
        "pending": 4
      },
      "subscriptions": {
        "total": 10,
        "completed": 5,
        "in_progress": 0,
        "failed": 0,
        "pending": 5
      },
      "professional_networks": {
        "total": 4,
        "completed": 0,
        "in_progress": 1,
        "failed": 0,
        "pending": 3
      },
      "other": {
        "total": 2,
        "completed": 0,
        "in_progress": 0,
        "failed": 0,
        "pending": 2
      }
    },
    "recentActivity": [
      {
        "timestamp": "2025-12-25T09:45:00Z",
        "accountId": "ACC-017",
        "platform": "spotify",
        "action": "deletion_completed",
        "status": "completed"
      },
      {
        "timestamp": "2025-12-25T09:30:00Z",
        "accountId": "ACC-003",
        "platform": "twitter",
        "action": "deletion_completed",
        "status": "completed"
      },
      {
        "timestamp": "2025-12-25T09:15:00Z",
        "accountId": "ACC-008",
        "platform": "instagram",
        "action": "deletion_failed",
        "status": "failed",
        "errorCode": "ERR_PLATFORM_TIMEOUT",
        "errorMessage": "Platform API timeout after 3 retries"
      }
    ],
    "estimatedCompletionDate": "2026-01-10T00:00:00Z"
  }
}
```

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-12-18 | Initial release |

---

## Appendix A: Related Standards

| Standard | Relationship |
|----------|--------------|
| GDPR Article 17 | Right to erasure legal framework |
| CCPA | California privacy law compliance |
| DoD 5220.22-M | Secure deletion standard |
| NIST SP 800-88 | Media sanitization guidelines |
| ISO/IEC 27001 | Information security management |

---

**弘益人間 (홍익인간)** - Benefit All Humanity
© 2025 WIA
MIT License
