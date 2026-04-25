# WIA-REFUGEE-CREDENTIAL: Phase 3 - Communication Protocol

**난민 자격증명 통신 프로토콜**
*REST API, Storage, and Verification Protocol*

> "국가가 무너져도, 사람의 가치는 무너지지 않습니다."

홍익인간 (弘益人間) - Benefit All Humanity

---

## 1. Overview

This document specifies:
- REST API for credential management
- Storage protocol (distributed, quantum-resistant)
- Peer verification protocol
- Institution verification protocol
- Multi-language support

---

## 2. REST API

### 2.1 Base URL

```
Production: https://api.credential.wia.org/v1
Staging:    https://api-staging.credential.wia.org/v1
```

### 2.2 Authentication

```http
# Holder authentication (DID-based)
Authorization: DID-Auth <did_token>

# Institution authentication (API key)
Authorization: Bearer <api_key>
X-WIA-Institution-ID: <institution_id>
```

### 2.3 Credential Endpoints

```http
# Create new credential (Level 1 - Self-Declaration)
POST /credentials
Content-Type: application/json
Accept-Language: ar

{
  "identity": {
    "display_name": "أحمد محمد",
    "birth_year": 1985,
    "nationality_claimed": "SY",
    "refugee_status": "unhcr_recognized",
    "current_country": "DE"
  },
  "education": [{
    "level": "professional",
    "field_of_study": "Medicine - Cardiology",
    "institution_name": "Damascus University",
    "institution_country": "SY",
    "year_start": 2003,
    "year_end": 2009,
    "status": "completed"
  }],
  "competencies": [{
    "domain": "medicine",
    "skill_name": "Cardiology",
    "level": "expert",
    "years_of_practice": 12
  }],
  "languages": [{
    "language_code": "ar",
    "speaking": "native",
    "reading": "native",
    "writing": "native"
  }]
}

Response: 201 Created
{
  "credential_id": "cred_01935a2b-3c4d-7e8f-9a0b-1c2d3e4f5a6b",
  "holder_did": "did:wia:z6MkpTHR8VNsBxYAAWHut2Geadd9jSwuBV8xRoAnwWsdvktH",
  "verification_level": 1,
  "confidence": 0.30,
  "qr_code": "WIAREF:1.0:eyJ...",
  "created_at": "2025-12-16T10:30:00Z"
}
```

```http
# Get credential by ID
GET /credentials/{credential_id}
Authorization: DID-Auth <token>

Response: 200 OK
{
  "id": "cred_01935a2b-...",
  "version": "1.0.0",
  "identity": { ... },
  "education": [ ... ],
  "verification_level": 2,
  "confidence": 0.65,
  ...
}
```

```http
# Update credential (add education, competency, etc.)
PATCH /credentials/{credential_id}
Content-Type: application/json

{
  "add_education": [{
    "level": "masters",
    "field_of_study": "Public Health",
    ...
  }]
}
```

```http
# Get credential verification status
GET /credentials/{credential_id}/verification

Response: 200 OK
{
  "current_level": 2,
  "confidence": 0.65,
  "verifications": [
    {
      "type": "self_declaration",
      "date": "2025-12-10T...",
      "confidence": 0.30
    },
    {
      "type": "peer_verification",
      "date": "2025-12-15T...",
      "confidence": 0.60,
      "peer_count": 3
    }
  ],
  "next_steps": [
    {
      "action": "competency_assessment",
      "potential_confidence": 0.80,
      "available_assessments": [...]
    }
  ]
}
```

### 2.4 Peer Verification Endpoints

```http
# Find potential peer verifiers
GET /credentials/{credential_id}/peers/find

Response: 200 OK
{
  "matches": [
    {
      "peer_id": "peer_abc123",
      "match_type": "education",
      "match_score": 0.85,
      "shared_context": {
        "institution": "Damascus University",
        "years": "2003-2009",
        "field": "Medicine"
      },
      "can_verify": ["education", "competency"]
    }
  ],
  "total_matches": 12
}
```

```http
# Request peer verification
POST /credentials/{credential_id}/peers/request
Content-Type: application/json

{
  "peer_ids": ["peer_abc123", "peer_def456"],
  "fields_to_verify": ["education", "competency"],
  "message": "I studied with you at Damascus University 2003-2009"
}

Response: 202 Accepted
{
  "request_ids": ["req_001", "req_002"],
  "status": "pending",
  "expires_at": "2025-12-30T..."
}
```

```http
# Peer submits verification (peer's endpoint)
POST /verifications/peer
Authorization: DID-Auth <peer_token>
Content-Type: application/json

{
  "request_id": "req_001",
  "credential_id": "cred_01935a2b-...",
  "verified_fields": {
    "education": {
      "verified": true,
      "confidence": "high",
      "notes": "I remember Ahmad, we were in the same cardiology residency"
    }
  },
  "relationship": "colleague",
  "years_known": 6
}
```

### 2.5 Assessment Endpoints

```http
# Get available assessments for credential
GET /credentials/{credential_id}/assessments/available

Response: 200 OK
{
  "assessments": [
    {
      "id": "assess_med_001",
      "name": "Medical Competency Assessment",
      "domain": "medicine",
      "type": "practical_test",
      "duration_minutes": 120,
      "languages": ["ar", "en", "de"],
      "locations": ["online", "Berlin", "Munich"],
      "cost": "free_for_refugees",
      "next_available": "2025-12-20T09:00:00Z"
    }
  ]
}
```

```http
# Schedule assessment
POST /credentials/{credential_id}/assessments/schedule
Content-Type: application/json

{
  "assessment_id": "assess_med_001",
  "preferred_date": "2025-12-20",
  "language": "ar",
  "location": "online"
}

Response: 201 Created
{
  "booking_id": "book_xyz789",
  "assessment_id": "assess_med_001",
  "scheduled_at": "2025-12-20T09:00:00Z",
  "join_url": "https://assess.wia.org/join/...",
  "preparation_materials": "https://..."
}
```

```http
# Submit assessment result (assessor endpoint)
POST /assessments/{booking_id}/result
Authorization: Bearer <assessor_token>

{
  "score": 87,
  "max_score": 100,
  "achieved_level": "expert",
  "detailed_scores": {
    "clinical_knowledge": 90,
    "practical_skills": 85,
    "communication": 86
  },
  "assessor_notes": "Strong clinical knowledge, recommended for supervised practice",
  "certificate_url": "https://..."
}
```

### 2.6 Institution Verification Endpoints

```http
# Institution verifies credential
POST /verify
Authorization: Bearer <institution_api_key>
Content-Type: application/json

{
  "credential_id": "cred_01935a2b-...",
  "purpose": "university_admission",
  "required_fields": ["education", "competencies", "languages"]
}

Response: 200 OK
{
  "valid": true,
  "verification_level": 3,
  "confidence": 0.82,
  "acceptance_score": 0.78,
  "recommendation": "conditional_accept",
  "details": {
    "education_verified": true,
    "competency_verified": true,
    "language_requirements_met": true
  },
  "suggested_conditions": [
    "Complete bridging course",
    "Pass language proficiency test"
  ]
}
```

```http
# Anonymous verification (ZK proof)
POST /verify/anonymous
Content-Type: application/json

{
  "proof": "<zk_proof_data>",
  "claim": "has_medical_degree",
  "verification_key": "<key>"
}

Response: 200 OK
{
  "claim_verified": true,
  "confidence": 0.80,
  "wia_attestation": "<signature>"
}
```

### 2.7 Evidence Upload

```http
# Upload evidence document
POST /credentials/{credential_id}/evidence
Content-Type: multipart/form-data

file: <document_file>
evidence_type: diploma
title: "Medical Degree Certificate"
description: "Original diploma from Damascus University"

Response: 201 Created
{
  "evidence_id": "evid_abc123",
  "file_hash": "sha256:...",
  "authenticity_score": 0.87,
  "ai_verification": {
    "tampering_detected": false,
    "text_extracted": true,
    "confidence": 0.85
  }
}
```

---

## 3. Storage Protocol

### 3.1 Distributed Storage Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    WIA Storage Layer                             │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌───────────────┐  ┌───────────────┐  ┌───────────────┐       │
│  │  WIA Primary  │  │  WIA Primary  │  │  WIA Primary  │       │
│  │  (Europe)     │  │  (Americas)   │  │  (Asia)       │       │
│  └───────┬───────┘  └───────┬───────┘  └───────┬───────┘       │
│          │                  │                  │                 │
│          └──────────────────┼──────────────────┘                 │
│                             │                                    │
│                    ┌────────┴────────┐                          │
│                    │   Sync Layer    │                          │
│                    └────────┬────────┘                          │
│                             │                                    │
│  ┌───────────────┐  ┌───────┴───────┐  ┌───────────────┐       │
│  │     IPFS      │  │   Arweave     │  │  User Device  │       │
│  │   (Backup)    │  │  (Permanent)  │  │   (Local)     │       │
│  └───────────────┘  └───────────────┘  └───────────────┘       │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### 3.2 Storage Operations

```typescript
interface StorageProtocol {
  // Store credential
  async store(credential: RefugeeCredential): Promise<StorageReceipt>;

  // Retrieve credential
  async retrieve(credentialId: string, holderDid: string): Promise<RefugeeCredential>;

  // Backup to distributed storage
  async backup(credentialId: string): Promise<BackupReceipt>;

  // Verify integrity
  async verifyIntegrity(credentialId: string): Promise<IntegrityResult>;

  // Export for local backup
  async exportLocal(credentialId: string): Promise<EncryptedBlob>;
}
```

### 3.3 Encryption

```yaml
encryption:
  at_rest:
    algorithm: AES-256-GCM
    key_derivation: Argon2id
    key_source: holder_master_key

  in_transit:
    protocol: TLS 1.3
    certificate: WIA CA

  long_term:
    algorithm: CRYSTALS-Kyber
    purpose: Future-proof against quantum attacks
```

---

## 4. Error Handling

### 4.1 Error Response Format

```json
{
  "error": {
    "code": "CREDENTIAL_NOT_FOUND",
    "message": "The requested credential does not exist",
    "message_localized": {
      "ar": "الشهادة المطلوبة غير موجودة",
      "uk": "Запитаний сертифікат не існує"
    },
    "details": {
      "credential_id": "cred_01935a2b-..."
    },
    "request_id": "req_xyz789"
  }
}
```

### 4.2 Error Codes

| Code | HTTP | Description |
|------|------|-------------|
| `CREDENTIAL_NOT_FOUND` | 404 | Credential does not exist |
| `ACCESS_DENIED` | 403 | Not authorized to access |
| `INVALID_DID` | 400 | DID format invalid |
| `VERIFICATION_EXPIRED` | 410 | Verification request expired |
| `PEER_NOT_FOUND` | 404 | Peer verifier not found |
| `ASSESSMENT_UNAVAILABLE` | 404 | Assessment not available |
| `DOCUMENT_TOO_LARGE` | 413 | Uploaded file too large |
| `TAMPERING_DETECTED` | 422 | Document tampering detected |
| `RATE_LIMIT_EXCEEDED` | 429 | Too many requests |

---

## 5. Webhooks

### 5.1 Events

```http
POST https://your-server.com/wia-webhook
Content-Type: application/json
X-WIA-Signature: sha256=...

{
  "event": "verification.completed",
  "timestamp": "2025-12-16T10:30:00Z",
  "data": {
    "credential_id": "cred_01935a2b-...",
    "new_level": 3,
    "new_confidence": 0.82,
    "verification_type": "assessment"
  }
}
```

### 5.2 Event Types

| Event | Description |
|-------|-------------|
| `credential.created` | New credential created |
| `verification.requested` | Peer verification requested |
| `verification.completed` | Verification level upgraded |
| `assessment.scheduled` | Assessment booked |
| `assessment.completed` | Assessment results available |
| `evidence.uploaded` | New evidence added |
| `evidence.verified` | Evidence verified |

---

## 6. Multi-Language Support

### 6.1 Request Language

```http
Accept-Language: ar, en;q=0.8
```

### 6.2 Response Localization

All user-facing messages include localized versions:

```json
{
  "status": "verification_pending",
  "message": "Your credential is pending verification",
  "message_localized": {
    "ar": "شهادتك قيد التحقق",
    "uk": "Ваш сертифікат очікує перевірки",
    "fa": "مدرک شما در انتظار تایید است"
  }
}
```

---

## 7. Rate Limiting

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 950
X-RateLimit-Reset: 1702720800
```

| Endpoint | Limit |
|----------|-------|
| Credential read | 1000/hour |
| Credential create | 10/hour |
| Evidence upload | 50/day |
| Verification request | 20/day |
| Assessment schedule | 5/day |

**Note**: Limits are increased for verified refugees with UNHCR status.

---

## 8. Offline Support

### 8.1 Offline Credential Card

```typescript
interface OfflineCredentialCard {
  // Compact representation
  credential_id: string;
  holder_name_hash: string;
  verification_level: number;
  confidence: number;

  // Key claims
  highest_education: string;
  primary_competency: string;
  languages: string[];

  // Verification
  qr_code: string;
  wia_signature: string;
  valid_until: ISO8601;
}
```

### 8.2 SMS Verification

For areas with limited internet:

```
SMS to: +1-800-WIA-VERIFY
Message: VERIFY cred_01935a2b

Response:
WIA: Credential VALID
Level: 3/4
Confidence: 82%
Education: MD Medicine
Valid until: 2026-12-16
```

---

**Document ID**: WIA-REFUGEE-CREDENTIAL-PHASE-3
**Version**: 1.0.0
**Date**: 2025-12-16
**Status**: Draft

*"시리아 의사는 여전히 의사입니다."*
