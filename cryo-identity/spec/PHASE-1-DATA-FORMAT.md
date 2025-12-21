# WIA Cryo-Identity Data Format Standard
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

The WIA Cryo-Identity Data Format Standard defines a unified format for managing and verifying the identity of cryopreserved individuals across their preservation lifecycle, including pre-mortem registration, preservation, and potential future revival.

**Core Objectives**:
- Ensure unique and persistent identification of cryopreserved subjects
- Support multiple verification methods (biometric, cryptographic, biological)
- Enable identity continuity from pre-mortem through potential revival
- Maintain privacy while enabling necessary data access

### 1.2 Scope

This standard covers:

| Domain | Description |
|--------|-------------|
| Identity Registration | Initial identity capture and verification |
| Biometric Data | Fingerprints, DNA, facial features, retinal scans |
| Cryptographic Identity | Digital signatures, key pairs, blockchain anchors |
| Identity Verification | Methods for confirming identity at various stages |
| Identity Recovery | Procedures for identity restoration post-revival |

### 1.3 Design Principles

1. **Persistence**: Identity must survive across centuries
2. **Redundancy**: Multiple backup identification methods
3. **Privacy**: Minimal disclosure, maximum security
4. **Verifiability**: Cryptographic proof of identity
5. **Interoperability**: Compatible with global identity standards

---

## Terminology

### 2.1 Core Terms

| Term | Definition |
|------|------------|
| **Subject** | Individual whose identity is being managed |
| **Identity Record** | Complete identity data package for a subject |
| **Biometric Template** | Encoded biometric feature representation |
| **Identity Anchor** | Immutable reference point for identity verification |
| **Verification Level** | Confidence level of identity confirmation |
| **Recovery Key** | Cryptographic key for identity restoration |

### 2.2 Data Types

| Type | Description | Example |
|------|-------------|---------|
| `string` | UTF-8 encoded text | `"ID-2025-001"` |
| `biometric_hash` | SHA-256 hash of biometric data | `"sha256:a1b2c3..."` |
| `public_key` | Ed25519 public key | `"ed25519:abc..."` |
| `dna_sequence` | Encoded DNA markers | `"ATCG..."` |
| `timestamp` | ISO 8601 datetime | `"2025-01-15T10:30:00Z"` |

### 2.3 Field Requirements

| Marker | Meaning |
|--------|---------|
| **REQUIRED** | Must be present |
| **OPTIONAL** | May be omitted |
| **CONDITIONAL** | Required under specific conditions |

---

## Base Structure

### 3.1 Identity Record Format

```json
{
  "$schema": "https://wia.live/cryo-identity/v1/schema.json",
  "version": "1.0.0",
  "identityId": "ID-2025-001",
  "subjectId": "SUBJ-2025-001",
  "status": "active",
  "created": "2024-06-15T10:00:00Z",
  "lastVerified": "2025-01-15T10:30:00Z",
  "personal": {
    "legalName": {
      "given": "encrypted:...",
      "family": "encrypted:...",
      "hash": "sha256:..."
    },
    "dateOfBirth": "encrypted:...",
    "placeOfBirth": "encrypted:...",
    "nationality": ["KR"],
    "governmentIds": []
  },
  "biometrics": {
    "fingerprints": [],
    "facial": {},
    "dna": {},
    "retinal": {},
    "voice": {}
  },
  "cryptographic": {
    "primaryKey": {},
    "recoveryKeys": [],
    "blockchainAnchors": []
  },
  "verification": {
    "level": "biometric",
    "methods": [],
    "history": []
  },
  "meta": {
    "hash": "sha256:...",
    "signature": "...",
    "previousHash": "..."
  }
}
```

### 3.2 Field Details

#### 3.2.1 `identityId` (REQUIRED)

```
Type: string
Format: ID-YYYY-NNNNNN
Description: Unique identifier for this identity record
Example: "ID-2025-000001"
```

#### 3.2.2 `status` (REQUIRED)

```
Type: string
Valid values:
  - "pending"     : Registration in progress
  - "active"      : Identity verified and active
  - "preserved"   : Subject is cryopreserved
  - "suspended"   : Temporarily unavailable
  - "revived"     : Subject has been revived
  - "merged"      : Combined with another identity
```

---

## Data Schema

### 4.1 Complete JSON Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wia.live/cryo-identity/v1/schema.json",
  "title": "WIA Cryo-Identity Record",
  "type": "object",
  "required": ["version", "identityId", "subjectId", "status", "created", "biometrics", "cryptographic"],
  "properties": {
    "version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+\\.\\d+$"
    },
    "identityId": {
      "type": "string",
      "pattern": "^ID-\\d{4}-\\d{6}$"
    },
    "subjectId": {
      "type": "string"
    },
    "status": {
      "type": "string",
      "enum": ["pending", "active", "preserved", "suspended", "revived", "merged"]
    },
    "created": {
      "type": "string",
      "format": "date-time"
    },
    "personal": {
      "type": "object",
      "properties": {
        "legalName": {
          "type": "object",
          "properties": {
            "given": { "type": "string" },
            "family": { "type": "string" },
            "hash": { "type": "string" }
          }
        },
        "dateOfBirth": { "type": "string" },
        "nationality": {
          "type": "array",
          "items": { "type": "string", "pattern": "^[A-Z]{2}$" }
        }
      }
    },
    "biometrics": {
      "type": "object",
      "properties": {
        "fingerprints": { "type": "array" },
        "facial": { "type": "object" },
        "dna": { "type": "object" },
        "retinal": { "type": "object" },
        "voice": { "type": "object" }
      }
    },
    "cryptographic": {
      "type": "object",
      "required": ["primaryKey"],
      "properties": {
        "primaryKey": { "type": "object" },
        "recoveryKeys": { "type": "array" },
        "blockchainAnchors": { "type": "array" }
      }
    },
    "verification": {
      "type": "object",
      "properties": {
        "level": { "type": "string" },
        "methods": { "type": "array" },
        "history": { "type": "array" }
      }
    }
  }
}
```

### 4.2 Biometrics Schema

```json
{
  "biometrics": {
    "fingerprints": [
      {
        "finger": "right_index",
        "template": "base64-encoded-template",
        "templateFormat": "ISO-19794-2",
        "quality": 0.95,
        "capturedAt": "2024-06-15T10:00:00Z",
        "hash": "sha256:..."
      }
    ],
    "facial": {
      "template": "base64-encoded-template",
      "templateFormat": "ISO-19794-5",
      "photos": [
        {
          "type": "frontal",
          "hash": "sha256:...",
          "capturedAt": "2024-06-15T10:00:00Z"
        }
      ]
    },
    "dna": {
      "markers": [
        { "locus": "D3S1358", "alleles": [15, 16] },
        { "locus": "vWA", "alleles": [17, 18] },
        { "locus": "FGA", "alleles": [22, 24] }
      ],
      "fullSequenceHash": "sha256:...",
      "sequenceStorage": "ipfs://...",
      "collectedAt": "2024-06-15T10:00:00Z"
    },
    "retinal": {
      "template": "base64-encoded-template",
      "eye": "both",
      "capturedAt": "2024-06-15T10:00:00Z"
    },
    "voice": {
      "template": "base64-encoded-template",
      "sampleHash": "sha256:...",
      "capturedAt": "2024-06-15T10:00:00Z"
    }
  }
}
```

### 4.3 Cryptographic Identity Schema

```json
{
  "cryptographic": {
    "primaryKey": {
      "algorithm": "Ed25519",
      "publicKey": "ed25519:abc123...",
      "created": "2024-06-15T10:00:00Z",
      "expires": null,
      "status": "active"
    },
    "recoveryKeys": [
      {
        "id": "recovery-1",
        "type": "shamir_share",
        "threshold": 3,
        "totalShares": 5,
        "shareHolders": [
          { "holder": "facility", "shareHash": "sha256:..." },
          { "holder": "family", "shareHash": "sha256:..." },
          { "holder": "legal", "shareHash": "sha256:..." }
        ]
      }
    ],
    "blockchainAnchors": [
      {
        "network": "ethereum",
        "transactionHash": "0x...",
        "blockNumber": 12345678,
        "timestamp": "2024-06-15T10:00:00Z",
        "identityHash": "sha256:..."
      }
    ]
  }
}
```

---

## Field Specifications

### 5.1 Biometric Fields

| Field | Type | Required | Description | Example |
|-------|------|----------|-------------|---------|
| `fingerprints[].finger` | string | REQUIRED | Finger identifier | `"right_index"` |
| `fingerprints[].template` | string | REQUIRED | Base64 template | `"QmFzZTY0..."` |
| `fingerprints[].quality` | number | REQUIRED | Quality score 0-1 | `0.95` |
| `dna.markers` | array | REQUIRED | STR markers | `[{...}]` |
| `dna.fullSequenceHash` | string | OPTIONAL | Full genome hash | `"sha256:..."` |
| `facial.template` | string | REQUIRED | Face template | `"QmFzZTY0..."` |

**Valid finger values:**

| Value | Description |
|-------|-------------|
| `right_thumb` | Right thumb |
| `right_index` | Right index finger |
| `right_middle` | Right middle finger |
| `right_ring` | Right ring finger |
| `right_little` | Right little finger |
| `left_thumb` | Left thumb |
| `left_index` | Left index finger |
| `left_middle` | Left middle finger |
| `left_ring` | Left ring finger |
| `left_little` | Left little finger |

### 5.2 Verification Levels

| Level | Requirements | Confidence |
|-------|--------------|------------|
| `basic` | Government ID only | 60% |
| `enhanced` | ID + single biometric | 80% |
| `biometric` | Multiple biometrics | 95% |
| `cryptographic` | Biometric + key signature | 99% |
| `full` | All methods + DNA | 99.9% |

### 5.3 DNA Marker Loci

| Locus | Chromosome | Use |
|-------|------------|-----|
| D3S1358 | 3 | Core identification |
| vWA | 12 | Core identification |
| FGA | 4 | Core identification |
| D8S1179 | 8 | Extended profiling |
| D21S11 | 21 | Extended profiling |
| D18S51 | 18 | Extended profiling |
| AMEL | X/Y | Sex determination |

---

## Data Types

### 6.1 Custom Types

```typescript
type IdentityStatus =
  | 'pending'
  | 'active'
  | 'preserved'
  | 'suspended'
  | 'revived'
  | 'merged';

type VerificationLevel =
  | 'basic'
  | 'enhanced'
  | 'biometric'
  | 'cryptographic'
  | 'full';

type FingerType =
  | 'right_thumb' | 'right_index' | 'right_middle' | 'right_ring' | 'right_little'
  | 'left_thumb' | 'left_index' | 'left_middle' | 'left_ring' | 'left_little';

interface BiometricTemplate {
  template: string;        // Base64 encoded
  templateFormat: string;  // ISO standard
  quality: number;         // 0.0 - 1.0
  capturedAt: string;      // ISO 8601
  hash: string;            // SHA-256
}
```

### 6.2 Enum Values

#### Template Formats

| Code | Standard | Description |
|------|----------|-------------|
| `ISO-19794-2` | ISO/IEC 19794-2 | Fingerprint minutiae |
| `ISO-19794-4` | ISO/IEC 19794-4 | Fingerprint image |
| `ISO-19794-5` | ISO/IEC 19794-5 | Face image |
| `ISO-19794-6` | ISO/IEC 19794-6 | Iris image |
| `ANSI-NIST` | ANSI/NIST-ITL | Multi-biometric |

---

## Validation Rules

### 7.1 Required Field Validation

| Rule ID | Field | Validation |
|---------|-------|------------|
| VAL-001 | `identityId` | Must match `^ID-\d{4}-\d{6}$` |
| VAL-002 | `biometrics.fingerprints` | At least 2 fingerprints required |
| VAL-003 | `biometrics.dna.markers` | At least 13 core markers |
| VAL-004 | `cryptographic.primaryKey` | Must be valid Ed25519 key |
| VAL-005 | `verification.level` | Must match biometric evidence |

### 7.2 Business Logic Validation

| Rule ID | Description | Error Code |
|---------|-------------|------------|
| BUS-001 | DNA markers must be unique within system | `ERR_DUPLICATE_DNA` |
| BUS-002 | Fingerprints must have quality > 0.7 | `ERR_LOW_QUALITY` |
| BUS-003 | Recovery keys must have threshold ≤ total | `ERR_INVALID_THRESHOLD` |
| BUS-004 | Blockchain anchor must be verified | `ERR_UNVERIFIED_ANCHOR` |

### 7.3 Error Codes

| Code | Message | Description |
|------|---------|-------------|
| `ERR_INVALID_IDENTITY` | Invalid identity format | ID format violation |
| `ERR_DUPLICATE_DNA` | DNA already registered | Duplicate detection |
| `ERR_LOW_QUALITY` | Biometric quality too low | Quality threshold failed |
| `ERR_INVALID_KEY` | Invalid cryptographic key | Key validation failed |
| `ERR_VERIFICATION_FAILED` | Identity verification failed | Match failed |

---

## Examples

### 8.1 Valid Identity Record

```json
{
  "$schema": "https://wia.live/cryo-identity/v1/schema.json",
  "version": "1.0.0",
  "identityId": "ID-2025-000001",
  "subjectId": "SUBJ-2025-001",
  "status": "active",
  "created": "2024-06-15T10:00:00Z",
  "lastVerified": "2025-01-15T10:30:00Z",
  "personal": {
    "legalName": {
      "given": "encrypted:aes256:...",
      "family": "encrypted:aes256:...",
      "hash": "sha256:abc123def456..."
    },
    "dateOfBirth": "encrypted:aes256:...",
    "nationality": ["KR"]
  },
  "biometrics": {
    "fingerprints": [
      {
        "finger": "right_index",
        "template": "QmFzZTY0RW5jb2RlZFRlbXBsYXRl...",
        "templateFormat": "ISO-19794-2",
        "quality": 0.95,
        "capturedAt": "2024-06-15T10:00:00Z",
        "hash": "sha256:fingerprint_hash_1..."
      },
      {
        "finger": "left_index",
        "template": "QmFzZTY0RW5jb2RlZFRlbXBsYXRl...",
        "templateFormat": "ISO-19794-2",
        "quality": 0.92,
        "capturedAt": "2024-06-15T10:00:00Z",
        "hash": "sha256:fingerprint_hash_2..."
      }
    ],
    "dna": {
      "markers": [
        { "locus": "D3S1358", "alleles": [15, 16] },
        { "locus": "vWA", "alleles": [17, 18] },
        { "locus": "FGA", "alleles": [22, 24] }
      ],
      "fullSequenceHash": "sha256:dna_full_sequence_hash...",
      "collectedAt": "2024-06-15T10:00:00Z"
    }
  },
  "cryptographic": {
    "primaryKey": {
      "algorithm": "Ed25519",
      "publicKey": "ed25519:abc123def456...",
      "created": "2024-06-15T10:00:00Z",
      "status": "active"
    },
    "blockchainAnchors": [
      {
        "network": "ethereum",
        "transactionHash": "0xabc123...",
        "blockNumber": 12345678,
        "timestamp": "2024-06-15T10:00:00Z"
      }
    ]
  },
  "verification": {
    "level": "biometric",
    "methods": ["fingerprint", "dna", "cryptographic"],
    "history": [
      {
        "timestamp": "2025-01-15T10:30:00Z",
        "method": "fingerprint",
        "result": "verified",
        "confidence": 0.98
      }
    ]
  },
  "meta": {
    "hash": "sha256:record_hash...",
    "signature": "ed25519:signature...",
    "version": 1
  }
}
```

### 8.2 Invalid Example - Missing Biometrics

```json
{
  "version": "1.0.0",
  "identityId": "ID-2025-000002",
  "subjectId": "SUBJ-2025-002",
  "status": "active",
  "biometrics": {
    "fingerprints": []
  }
}
```

**Error**: `ERR_VALIDATION_FAILED` - At least 2 fingerprints required

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01 | Initial release |

---

<div align="center">

**WIA Cryo-Identity Data Format Standard v1.0.0**

**弘益人間 (홍익인간)** - Benefit All Humanity

---

**© 2025 WIA Standards Committee**

**MIT License**

</div>
