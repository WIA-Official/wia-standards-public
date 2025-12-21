# WIA Cryo-Legal Standard - Phase 2: API Interface Specification

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #06B6D4 (Cyan)

---

## 1. Overview

### 1.1 Purpose

The WIA Cryo-Legal API provides standardized interfaces for managing legal documents, consent verification, jurisdiction compliance, and audit trails in cryopreservation systems.

### 1.2 Design Principles

| Principle | Description |
|-----------|-------------|
| RESTful | Resource-oriented API design |
| Security-First | End-to-end encryption, audit logging |
| Jurisdictional | Multi-jurisdiction support |
| Versioned | API version in URL path |
| Idempotent | Safe retry operations |

### 1.3 Base URL

```
Production: https://api.wia.live/cryo-legal/v1
Staging:    https://staging-api.wia.live/cryo-legal/v1
```

---

## 2. Authentication

### 2.1 API Key Authentication

```http
Authorization: Bearer {api_key}
X-WIA-Client-ID: {client_id}
```

### 2.2 OAuth 2.0 Flow

```
┌──────────────────────────────────────────────────────────────┐
│                     OAuth 2.0 Flow                            │
├──────────────────────────────────────────────────────────────┤
│  1. Client requests authorization                             │
│     GET /oauth/authorize?client_id=...&scope=...             │
│                                                               │
│  2. User authenticates and grants access                      │
│                                                               │
│  3. Authorization server returns code                         │
│     redirect_uri?code={authorization_code}                   │
│                                                               │
│  4. Client exchanges code for token                          │
│     POST /oauth/token                                        │
│                                                               │
│  5. Use access token for API calls                           │
│     Authorization: Bearer {access_token}                      │
└──────────────────────────────────────────────────────────────┘
```

### 2.3 Token Request

```http
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=authorization_code
&code={authorization_code}
&client_id={client_id}
&client_secret={client_secret}
&redirect_uri={redirect_uri}
```

**Response:**
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIs...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "refresh_token": "dGhpcyBpcyBhIHJlZnJlc2g...",
  "scope": "documents:read documents:write signatures:create"
}
```

### 2.4 Scopes

| Scope | Description |
|-------|-------------|
| `documents:read` | Read document metadata and content |
| `documents:write` | Create and modify documents |
| `documents:delete` | Delete documents |
| `signatures:create` | Sign documents |
| `signatures:verify` | Verify signatures |
| `audit:read` | Access audit logs |
| `parties:manage` | Manage party information |
| `jurisdictions:read` | Query jurisdiction requirements |

---

## 3. Endpoints

### 3.1 Documents

#### 3.1.1 Create Document

**POST** `/documents`

Creates a new legal document.

**Request Headers:**
```http
Authorization: Bearer {token}
Content-Type: application/json
X-Jurisdiction: US-AZ
X-Idempotency-Key: {unique_key}
```

**Request Body:**
```json
{
  "documentType": "cryopreservation_contract",
  "jurisdiction": {
    "primaryCountry": "US",
    "governingLaw": "State of Arizona"
  },
  "parties": [
    {
      "role": "subject",
      "identity": {
        "type": "individual",
        "legalName": "John Smith",
        "email": "john@example.com"
      }
    }
  ],
  "content": {
    "format": "markdown",
    "body": "# Agreement\n\nTerms and conditions...",
    "templateId": "cryo-contract-v2"
  },
  "effectiveDate": "2025-02-01T00:00:00Z",
  "metadata": {
    "language": "en-US",
    "tags": ["whole-body", "arizona"]
  }
}
```

**Response (201 Created):**
```json
{
  "documentId": "550e8400-e29b-41d4-a716-446655440000",
  "version": "1.0.0",
  "documentType": "cryopreservation_contract",
  "status": "draft",
  "createdAt": "2025-01-15T10:30:00Z",
  "jurisdiction": {
    "primaryCountry": "US",
    "governingLaw": "State of Arizona"
  },
  "requiredSignatures": [
    {
      "partyId": "a1b2c3d4-e5f6-7890-abcd-ef1234567890",
      "role": "subject",
      "status": "pending"
    }
  ],
  "links": {
    "self": "/documents/550e8400-e29b-41d4-a716-446655440000",
    "sign": "/documents/550e8400-e29b-41d4-a716-446655440000/signatures",
    "pdf": "/documents/550e8400-e29b-41d4-a716-446655440000/export/pdf"
  }
}
```

#### 3.1.2 Get Document

**GET** `/documents/{documentId}`

**Request:**
```http
GET /documents/550e8400-e29b-41d4-a716-446655440000
Authorization: Bearer {token}
Accept: application/json
```

**Response (200 OK):**
```json
{
  "documentId": "550e8400-e29b-41d4-a716-446655440000",
  "version": "1.0.0",
  "documentType": "cryopreservation_contract",
  "status": "executed",
  "createdAt": "2025-01-15T10:30:00Z",
  "effectiveDate": "2025-02-01T00:00:00Z",
  "jurisdiction": {
    "primaryCountry": "US",
    "governingLaw": "State of Arizona"
  },
  "parties": [...],
  "content": {...},
  "signatures": [...],
  "notarization": {...},
  "auditLog": [...]
}
```

#### 3.1.3 List Documents

**GET** `/documents`

**Query Parameters:**
| Parameter | Type | Description |
|-----------|------|-------------|
| `type` | string | Filter by document type |
| `status` | string | Filter by status (draft, pending, executed) |
| `partyId` | string | Filter by party involvement |
| `jurisdiction` | string | Filter by jurisdiction |
| `createdAfter` | datetime | Filter by creation date |
| `createdBefore` | datetime | Filter by creation date |
| `page` | integer | Page number (default: 1) |
| `limit` | integer | Items per page (default: 20, max: 100) |

**Request:**
```http
GET /documents?type=cryopreservation_contract&status=executed&limit=10
Authorization: Bearer {token}
```

**Response (200 OK):**
```json
{
  "documents": [
    {
      "documentId": "550e8400-e29b-41d4-a716-446655440000",
      "documentType": "cryopreservation_contract",
      "status": "executed",
      "createdAt": "2025-01-15T10:30:00Z",
      "parties": [{"role": "subject", "legalName": "John Smith"}]
    }
  ],
  "pagination": {
    "page": 1,
    "limit": 10,
    "total": 45,
    "totalPages": 5
  },
  "links": {
    "self": "/documents?page=1&limit=10",
    "next": "/documents?page=2&limit=10",
    "last": "/documents?page=5&limit=10"
  }
}
```

#### 3.1.4 Update Document

**PATCH** `/documents/{documentId}`

**Request:**
```json
{
  "content": {
    "body": "Updated content..."
  },
  "metadata": {
    "tags": ["updated", "arizona"]
  }
}
```

**Response (200 OK):**
```json
{
  "documentId": "550e8400-e29b-41d4-a716-446655440000",
  "version": "1.0.1",
  "status": "draft",
  "updatedAt": "2025-01-16T09:00:00Z"
}
```

#### 3.1.5 Delete Document

**DELETE** `/documents/{documentId}`

**Response (204 No Content)**

---

### 3.2 Signatures

#### 3.2.1 Sign Document

**POST** `/documents/{documentId}/signatures`

**Request:**
```json
{
  "signerId": "a1b2c3d4-e5f6-7890-abcd-ef1234567890",
  "signatureType": "digital",
  "signatureData": "MEUCIQDx...",
  "certificate": "-----BEGIN CERTIFICATE-----...",
  "consent": {
    "acknowledged": true,
    "timestamp": "2025-01-15T14:30:00Z"
  }
}
```

**Response (201 Created):**
```json
{
  "signatureId": "sig-123456",
  "documentId": "550e8400-e29b-41d4-a716-446655440000",
  "signerId": "a1b2c3d4-e5f6-7890-abcd-ef1234567890",
  "timestamp": "2025-01-15T14:30:00Z",
  "verificationStatus": "verified",
  "documentStatus": "pending",
  "remainingSignatures": 1
}
```

#### 3.2.2 Verify Signature

**POST** `/signatures/verify`

**Request:**
```json
{
  "documentId": "550e8400-e29b-41d4-a716-446655440000",
  "signatureId": "sig-123456"
}
```

**Response:**
```json
{
  "valid": true,
  "signerId": "a1b2c3d4-e5f6-7890-abcd-ef1234567890",
  "signedAt": "2025-01-15T14:30:00Z",
  "certificateInfo": {
    "issuer": "WIA Certificate Authority",
    "validFrom": "2024-01-01T00:00:00Z",
    "validTo": "2026-01-01T00:00:00Z",
    "status": "valid"
  },
  "documentIntegrity": "intact"
}
```

#### 3.2.3 List Signatures

**GET** `/documents/{documentId}/signatures`

**Response:**
```json
{
  "signatures": [
    {
      "signatureId": "sig-123456",
      "signerId": "a1b2c3d4-e5f6-7890-abcd-ef1234567890",
      "signerName": "John Smith",
      "role": "subject",
      "timestamp": "2025-01-15T14:30:00Z",
      "verificationStatus": "verified"
    }
  ]
}
```

---

### 3.3 Parties

#### 3.3.1 Create Party

**POST** `/parties`

**Request:**
```json
{
  "role": "subject",
  "identity": {
    "type": "individual",
    "legalName": "John Smith",
    "dateOfBirth": "1980-05-15",
    "nationality": "US",
    "identificationNumber": "DL-123456",
    "identificationType": "driver_license"
  },
  "contact": {
    "email": "john@example.com",
    "phone": "+1-555-0123"
  },
  "address": {
    "street": "123 Main St",
    "city": "Phoenix",
    "state": "AZ",
    "postalCode": "85001",
    "country": "US"
  }
}
```

**Response (201 Created):**
```json
{
  "partyId": "a1b2c3d4-e5f6-7890-abcd-ef1234567890",
  "role": "subject",
  "identity": {...},
  "verificationStatus": "pending",
  "createdAt": "2025-01-15T09:00:00Z"
}
```

#### 3.3.2 Get Party

**GET** `/parties/{partyId}`

#### 3.3.3 Update Party

**PATCH** `/parties/{partyId}`

#### 3.3.4 Verify Party Identity

**POST** `/parties/{partyId}/verify`

**Request:**
```json
{
  "verificationType": "document",
  "documents": [
    {
      "type": "passport",
      "frontImage": "base64...",
      "backImage": "base64..."
    }
  ],
  "selfie": "base64..."
}
```

**Response:**
```json
{
  "verificationId": "ver-123456",
  "status": "verified",
  "confidence": 0.98,
  "checks": [
    {"check": "document_authenticity", "passed": true},
    {"check": "face_match", "passed": true, "confidence": 0.97},
    {"check": "liveness", "passed": true}
  ],
  "verifiedAt": "2025-01-15T10:00:00Z"
}
```

---

### 3.4 Jurisdictions

#### 3.4.1 Get Jurisdiction Requirements

**GET** `/jurisdictions/{countryCode}`

**Response:**
```json
{
  "countryCode": "US",
  "countryName": "United States",
  "cryopreservationLegal": true,
  "requirements": {
    "documents": [
      {
        "type": "cryopreservation_contract",
        "required": true,
        "witnessesRequired": 2,
        "notarizationRequired": true
      },
      {
        "type": "advance_directive",
        "required": true,
        "witnessesRequired": 2
      }
    ],
    "minAge": 18,
    "waitingPeriod": null,
    "restrictions": []
  },
  "subdivisions": [
    {
      "code": "US-AZ",
      "name": "Arizona",
      "additionalRequirements": {
        "facilityLicense": "required"
      }
    }
  ],
  "lastUpdated": "2025-01-01T00:00:00Z"
}
```

#### 3.4.2 List All Jurisdictions

**GET** `/jurisdictions`

**Query Parameters:**
- `cryoLegal`: Filter by cryopreservation legality
- `region`: Filter by region (NA, EU, APAC, etc.)

---

### 3.5 Audit Logs

#### 3.5.1 Get Document Audit Log

**GET** `/documents/{documentId}/audit`

**Response:**
```json
{
  "documentId": "550e8400-e29b-41d4-a716-446655440000",
  "entries": [
    {
      "entryId": "audit-001",
      "timestamp": "2025-01-15T10:30:00Z",
      "action": "created",
      "actorId": "user-123",
      "actorType": "user",
      "details": "Document created from template",
      "ipAddress": "192.168.1.1",
      "userAgent": "Mozilla/5.0..."
    },
    {
      "entryId": "audit-002",
      "timestamp": "2025-01-15T14:30:00Z",
      "action": "signed",
      "actorId": "a1b2c3d4-e5f6-7890-abcd-ef1234567890",
      "actorType": "party",
      "details": "Digital signature applied",
      "signatureId": "sig-123456"
    }
  ],
  "pagination": {...}
}
```

---

### 3.6 Templates

#### 3.6.1 List Templates

**GET** `/templates`

**Response:**
```json
{
  "templates": [
    {
      "templateId": "cryo-contract-v2",
      "name": "Cryopreservation Services Agreement",
      "documentType": "cryopreservation_contract",
      "version": "2.1.0",
      "jurisdictions": ["US", "CA", "GB"],
      "languages": ["en", "es", "ko"]
    }
  ]
}
```

#### 3.6.2 Get Template

**GET** `/templates/{templateId}`

#### 3.6.3 Render Template

**POST** `/templates/{templateId}/render`

**Request:**
```json
{
  "variables": {
    "subjectName": "John Smith",
    "facilityName": "CryoLife Inc.",
    "serviceType": "whole_body"
  },
  "language": "en",
  "outputFormat": "markdown"
}
```

**Response:**
```json
{
  "rendered": "# Cryopreservation Agreement\n\nThis agreement between John Smith and CryoLife Inc...",
  "variables": {...},
  "templateId": "cryo-contract-v2",
  "templateVersion": "2.1.0"
}
```

---

## 4. Error Handling

### 4.1 Error Response Format

```json
{
  "error": {
    "code": "DOCUMENT_NOT_FOUND",
    "message": "Document with specified ID does not exist",
    "details": {
      "documentId": "550e8400-e29b-41d4-a716-446655440000"
    },
    "requestId": "req-abc123",
    "timestamp": "2025-01-15T10:30:00Z"
  }
}
```

### 4.2 Error Codes

| Code | HTTP Status | Message | Description |
|------|-------------|---------|-------------|
| `UNAUTHORIZED` | 401 | Authentication required | Missing or invalid token |
| `FORBIDDEN` | 403 | Access denied | Insufficient permissions |
| `DOCUMENT_NOT_FOUND` | 404 | Document not found | Document ID doesn't exist |
| `PARTY_NOT_FOUND` | 404 | Party not found | Party ID doesn't exist |
| `VALIDATION_ERROR` | 400 | Validation failed | Invalid request data |
| `INVALID_SIGNATURE` | 400 | Signature invalid | Signature verification failed |
| `DOCUMENT_LOCKED` | 409 | Document is locked | Cannot modify signed document |
| `JURISDICTION_INVALID` | 400 | Invalid jurisdiction | Jurisdiction not supported |
| `SIGNATURE_EXPIRED` | 400 | Certificate expired | Signing certificate expired |
| `RATE_LIMITED` | 429 | Rate limit exceeded | Too many requests |
| `INTERNAL_ERROR` | 500 | Internal server error | Server-side error |

---

## 5. Rate Limiting

### 5.1 Limits

| Tier | Requests/Hour | Requests/Day |
|------|---------------|--------------|
| Free | 100 | 1,000 |
| Standard | 1,000 | 10,000 |
| Enterprise | 10,000 | 100,000 |

### 5.2 Rate Limit Headers

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 950
X-RateLimit-Reset: 1642288800
```

---

## 6. SDK Examples

### 6.1 Python SDK

```python
from wia_cryo_legal import CryoLegalClient

# Initialize client
client = CryoLegalClient(
    api_key="your-api-key",
    environment="production"
)

# Create document
document = client.documents.create(
    document_type="cryopreservation_contract",
    jurisdiction={
        "primary_country": "US",
        "governing_law": "State of Arizona"
    },
    parties=[
        {
            "role": "subject",
            "identity": {
                "type": "individual",
                "legal_name": "John Smith"
            }
        }
    ],
    content={
        "template_id": "cryo-contract-v2",
        "variables": {"service_type": "whole_body"}
    }
)

print(f"Created document: {document.document_id}")

# Sign document
signature = client.signatures.create(
    document_id=document.document_id,
    signer_id="party-uuid",
    signature_type="digital",
    private_key=private_key
)

# Verify signature
verification = client.signatures.verify(
    document_id=document.document_id,
    signature_id=signature.signature_id
)

print(f"Signature valid: {verification.valid}")

# List documents
documents = client.documents.list(
    type="cryopreservation_contract",
    status="executed",
    limit=10
)

for doc in documents:
    print(f"- {doc.document_id}: {doc.status}")
```

### 6.2 TypeScript SDK

```typescript
import { CryoLegalClient } from '@wia/cryo-legal';

// Initialize client
const client = new CryoLegalClient({
  apiKey: 'your-api-key',
  environment: 'production'
});

// Create document
const document = await client.documents.create({
  documentType: 'cryopreservation_contract',
  jurisdiction: {
    primaryCountry: 'US',
    governingLaw: 'State of Arizona'
  },
  parties: [
    {
      role: 'subject',
      identity: {
        type: 'individual',
        legalName: 'John Smith'
      }
    }
  ],
  content: {
    templateId: 'cryo-contract-v2',
    variables: { serviceType: 'whole_body' }
  }
});

console.log(`Created document: ${document.documentId}`);

// Sign document
const signature = await client.signatures.create({
  documentId: document.documentId,
  signerId: 'party-uuid',
  signatureType: 'digital',
  privateKey: privateKey
});

// Verify
const verification = await client.signatures.verify({
  documentId: document.documentId,
  signatureId: signature.signatureId
});

console.log(`Signature valid: ${verification.valid}`);

// List with async iteration
for await (const doc of client.documents.list({ status: 'executed' })) {
  console.log(`- ${doc.documentId}: ${doc.status}`);
}
```

### 6.3 cURL Examples

```bash
# Create document
curl -X POST https://api.wia.live/cryo-legal/v1/documents \
  -H "Authorization: Bearer $TOKEN" \
  -H "Content-Type: application/json" \
  -d '{
    "documentType": "cryopreservation_contract",
    "jurisdiction": {"primaryCountry": "US", "governingLaw": "Arizona"},
    "parties": [{"role": "subject", "identity": {"legalName": "John"}}],
    "content": {"format": "markdown", "body": "# Agreement"}
  }'

# Get document
curl https://api.wia.live/cryo-legal/v1/documents/550e8400-... \
  -H "Authorization: Bearer $TOKEN"

# List documents
curl "https://api.wia.live/cryo-legal/v1/documents?status=executed&limit=10" \
  -H "Authorization: Bearer $TOKEN"

# Sign document
curl -X POST https://api.wia.live/cryo-legal/v1/documents/550e8400-.../signatures \
  -H "Authorization: Bearer $TOKEN" \
  -H "Content-Type: application/json" \
  -d '{
    "signerId": "party-uuid",
    "signatureType": "digital",
    "signatureData": "MEUCIQDx..."
  }'
```

---

## 7. Webhooks

### 7.1 Webhook Events

| Event | Description |
|-------|-------------|
| `document.created` | New document created |
| `document.updated` | Document modified |
| `document.signed` | Signature added |
| `document.executed` | All signatures complete |
| `document.expired` | Document expired |
| `party.verified` | Party identity verified |

### 7.2 Webhook Payload

```json
{
  "event": "document.signed",
  "timestamp": "2025-01-15T14:30:00Z",
  "data": {
    "documentId": "550e8400-e29b-41d4-a716-446655440000",
    "signatureId": "sig-123456",
    "signerId": "party-uuid",
    "documentStatus": "pending",
    "remainingSignatures": 1
  }
}
```

### 7.3 Webhook Signature Verification

```python
import hmac
import hashlib

def verify_webhook(payload: bytes, signature: str, secret: str) -> bool:
    expected = hmac.new(
        secret.encode(),
        payload,
        hashlib.sha256
    ).hexdigest()
    return hmac.compare_digest(f"sha256={expected}", signature)
```

---

<div align="center">

**WIA Cryo-Legal Standard v1.0.0**

Phase 2: API Interface Specification

**弘益人間 (홍익인간)** · Benefit All Humanity

---

© 2025 WIA Standards Committee

MIT License

</div>
