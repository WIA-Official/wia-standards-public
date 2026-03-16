# WIA-AGRI-011: Smart Seed Standard
## Phase 2 - API Interface Specification

**Version:** 1.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-01-01

---

## 1. Overview

This specification defines RESTful API interfaces for smart seed systems, enabling variety registration, germination testing, seed certification, and digital seed passport management.

### 1.1 Base URL

```
Production: https://api.wiastandards.com/seed/v1
Sandbox:    https://sandbox-api.wiastandards.com/seed/v1
```

### 1.2 Authentication

All API requests MUST include authentication using one of the following methods:

**API Key (Recommended for server-to-server)**
```http
Authorization: Bearer <API_KEY>
```

**DID-based Authentication (Recommended for decentralized identity)**
```http
Authorization: DID <DID_IDENTIFIER>
DID-Signature: <SIGNATURE>
```

---

## 2. Seed Variety Management

### 2.1 Register New Variety

Register a new seed variety with intellectual property protection.

**Endpoint:** `POST /varieties`

**Request Body:**
```json
{
  "varietyName": "Rice Supreme 2025",
  "scientificName": "Oryza sativa L.",
  "cropType": "grain",
  "breeder": {
    "organizationName": "National Seed Institute",
    "breederName": "Dr. Kim Seed",
    "country": "KR",
    "did": "did:wia:breeder:nsi-kr"
  },
  "characteristics": {
    "maturityDays": 120,
    "plantHeight": 95,
    "yieldPotential": 8500,
    "diseaseResistance": ["blast", "bacterial-blight"],
    "climateAdaptation": ["temperate", "subtropical"]
  },
  "intellectualProperty": {
    "pvpNumber": "PVP-2025-001",
    "registrationAuthority": {
      "country": "KR",
      "agency": "Korea Seed & Variety Service"
    }
  }
}
```

**Response:** `201 Created`
```json
{
  "varietyId": "550e8400-e29b-41d4-a716-446655440000",
  "varietyName": "Rice Supreme 2025",
  "registrationStatus": "pending",
  "submittedAt": "2025-01-15T10:30:00Z",
  "estimatedApprovalDate": "2025-03-15"
}
```

### 2.2 Get Variety Details

**Endpoint:** `GET /varieties/{varietyId}`

**Response:** `200 OK`
```json
{
  "varietyId": "550e8400-e29b-41d4-a716-446655440000",
  "varietyName": "Rice Supreme 2025",
  "scientificName": "Oryza sativa L.",
  "registrationStatus": "approved",
  "breeder": { ... },
  "characteristics": { ... },
  "intellectualProperty": { ... }
}
```

### 2.3 Search Varieties

**Endpoint:** `GET /varieties/search?q={query}&cropType={type}&country={code}`

**Example:** `GET /varieties/search?q=rice&cropType=grain&country=KR`

**Response:** `200 OK`
```json
{
  "total": 45,
  "page": 1,
  "pageSize": 20,
  "varieties": [
    {
      "varietyId": "550e8400-e29b-41d4-a716-446655440000",
      "varietyName": "Rice Supreme 2025",
      "cropType": "grain",
      "breeder": "National Seed Institute"
    }
  ]
}
```

---

## 3. Seed Lot Management

### 3.1 Create Seed Lot

**Endpoint:** `POST /lots`

**Request Body:**
```json
{
  "lotId": "KSC-2025-001",
  "varietyId": "550e8400-e29b-41d4-a716-446655440000",
  "productionInfo": {
    "producer": {
      "name": "Korea Seed Company",
      "license": "KR-SEED-12345",
      "did": "did:wia:producer:ksc"
    },
    "productionDate": "2025-01-15",
    "quantity": { "value": 1000, "unit": "kg" },
    "generation": "Certified"
  },
  "qualityMetrics": {
    "germinationRate": 95.5,
    "purity": 98.2,
    "moisture": 12.5
  }
}
```

**Response:** `201 Created`
```json
{
  "lotId": "KSC-2025-001",
  "status": "created",
  "createdAt": "2025-01-15T14:00:00Z",
  "passportUrl": "https://api.wiastandards.com/seed/v1/lots/KSC-2025-001/passport"
}
```

### 3.2 Get Seed Lot Details

**Endpoint:** `GET /lots/{lotId}`

**Response:** `200 OK`
```json
{
  "lotId": "KSC-2025-001",
  "varietyId": "550e8400-e29b-41d4-a716-446655440000",
  "varietyName": "Rice Supreme 2025",
  "productionInfo": { ... },
  "qualityMetrics": { ... },
  "certificationStatus": "certified",
  "traceabilityUrl": "https://api.wiastandards.com/seed/v1/lots/KSC-2025-001/trace"
}
```

### 3.3 Update Lot Quality Metrics

**Endpoint:** `PATCH /lots/{lotId}/quality`

**Request Body:**
```json
{
  "germinationRate": 94.0,
  "testDate": "2025-02-01",
  "laboratory": "ISTA Accredited Lab KR-001"
}
```

**Response:** `200 OK`
```json
{
  "lotId": "KSC-2025-001",
  "qualityMetrics": {
    "germinationRate": 94.0,
    "lastTestedDate": "2025-02-01"
  },
  "updated": true
}
```

---

## 4. Germination Testing

### 4.1 Submit Germination Test

**Endpoint:** `POST /lots/{lotId}/germination-tests`

**Request Body:**
```json
{
  "testStandard": "ISTA",
  "testDate": "2025-01-20",
  "laboratory": {
    "name": "Korea Seed Testing Lab",
    "accreditation": "ISTA",
    "did": "did:wia:lab:kstl"
  },
  "testConditions": {
    "temperature": 25,
    "substrate": "paper",
    "duration": 7,
    "replicates": 4,
    "seedsPerReplicate": 100
  },
  "results": {
    "normalSeedlings": 382,
    "abnormalSeedlings": 12,
    "deadSeeds": 6,
    "germinationPercentage": 95.5
  }
}
```

**Response:** `201 Created`
```json
{
  "testId": "TEST-2025-001",
  "lotId": "KSC-2025-001",
  "status": "submitted",
  "verificationRequired": true,
  "certificateUrl": "https://api.wiastandards.com/seed/v1/tests/TEST-2025-001/certificate"
}
```

### 4.2 Get Test Results

**Endpoint:** `GET /tests/{testId}`

**Response:** `200 OK`
```json
{
  "testId": "TEST-2025-001",
  "lotId": "KSC-2025-001",
  "testStandard": "ISTA",
  "results": {
    "germinationPercentage": 95.5,
    "vigourClassification": "high"
  },
  "verificationStatus": "verified",
  "verifiedBy": "Dr. Park Test",
  "verifiedAt": "2025-01-21T09:00:00Z"
}
```

### 4.3 List Tests for Lot

**Endpoint:** `GET /lots/{lotId}/germination-tests`

**Response:** `200 OK`
```json
{
  "lotId": "KSC-2025-001",
  "totalTests": 3,
  "tests": [
    {
      "testId": "TEST-2025-001",
      "testDate": "2025-01-20",
      "germinationPercentage": 95.5,
      "status": "verified"
    }
  ]
}
```

---

## 5. Seed Certification

### 5.1 Request Certification

**Endpoint:** `POST /lots/{lotId}/certifications`

**Request Body:**
```json
{
  "certificationType": "Certified",
  "certificationScheme": "OECD",
  "requestedBy": {
    "name": "Korea Seed Company",
    "did": "did:wia:producer:ksc"
  },
  "fieldInspectionData": {
    "inspectionDate": "2024-09-15",
    "isolationDistance": 500,
    "varietyPurity": 99.5
  }
}
```

**Response:** `201 Created`
```json
{
  "certificateId": "CERT-2025-12345",
  "lotId": "KSC-2025-001",
  "status": "pending-review",
  "estimatedCompletionDate": "2025-02-15",
  "reviewerAssigned": "KR Seed Certification Agency"
}
```

### 5.2 Get Certification Status

**Endpoint:** `GET /certifications/{certificateId}`

**Response:** `200 OK`
```json
{
  "certificateId": "CERT-2025-12345",
  "lotId": "KSC-2025-001",
  "status": "approved",
  "issuingAuthority": "Korea Seed & Variety Service",
  "issueDate": "2025-02-10",
  "validUntil": "2026-02-10",
  "complianceStatus": {
    "germination": true,
    "purity": true,
    "overallCompliance": true
  },
  "blockchainRecord": {
    "transactionHash": "0x7f9fade1c0d57a7af66ab4ead79fade1c0d57a7af66ab4ead7c2c2eb7b11a91385",
    "network": "Polygon"
  },
  "certificatePdfUrl": "https://api.wiastandards.com/seed/v1/certifications/CERT-2025-12345/pdf"
}
```

### 5.3 Download Certificate PDF

**Endpoint:** `GET /certifications/{certificateId}/pdf`

**Response:** `200 OK` (application/pdf)

---

## 6. Digital Seed Passport

### 6.1 Generate Seed Passport

**Endpoint:** `POST /lots/{lotId}/passport`

**Request Body:**
```json
{
  "expiryDate": "2026-01-15",
  "includeQrCode": true,
  "includeVerifiableCredential": true
}
```

**Response:** `201 Created`
```json
{
  "passportId": "PASS-2025-001",
  "lotId": "KSC-2025-001",
  "qrCode": "https://verify.wiastandards.com/seed/KSC-2025-001",
  "verifiableCredential": {
    "@context": ["https://www.w3.org/2018/credentials/v1"],
    "type": ["VerifiableCredential", "SeedPassportCredential"],
    "issuer": "did:wia:agency:seed-certification",
    "credentialSubject": {
      "id": "did:wia:seed:ksc-2025-001",
      "lotId": "KSC-2025-001",
      "germinationRate": 95.5
    }
  },
  "passportPdfUrl": "https://api.wiastandards.com/seed/v1/lots/KSC-2025-001/passport/pdf"
}
```

### 6.2 Verify Seed Passport

**Endpoint:** `GET /passport/verify/{passportId}`

**Response:** `200 OK`
```json
{
  "valid": true,
  "passportId": "PASS-2025-001",
  "lotId": "KSC-2025-001",
  "varietyName": "Rice Supreme 2025",
  "certificationStatus": "Certified",
  "verifiedAt": "2025-01-22T10:00:00Z",
  "blockchainVerification": {
    "verified": true,
    "transactionHash": "0x..."
  }
}
```

### 6.3 Get Traceability Chain

**Endpoint:** `GET /lots/{lotId}/trace`

**Response:** `200 OK`
```json
{
  "lotId": "KSC-2025-001",
  "traceabilityChain": [
    {
      "stage": "production",
      "actor": "Korea Seed Company",
      "timestamp": "2025-01-15T08:00:00Z",
      "action": "seed-produced",
      "location": "Seoul, KR",
      "blockchainHash": "0x..."
    },
    {
      "stage": "testing",
      "actor": "Korea Seed Testing Lab",
      "timestamp": "2025-01-20T14:00:00Z",
      "action": "germination-tested",
      "blockchainHash": "0x..."
    },
    {
      "stage": "certification",
      "actor": "Korea Seed & Variety Service",
      "timestamp": "2025-02-10T10:00:00Z",
      "action": "certified",
      "blockchainHash": "0x..."
    }
  ]
}
```

---

## 7. Webhook Events

Subscribe to real-time notifications for seed lot events.

### 7.1 Register Webhook

**Endpoint:** `POST /webhooks`

**Request Body:**
```json
{
  "url": "https://your-server.com/seed-webhook",
  "events": [
    "lot.created",
    "lot.tested",
    "lot.certified",
    "certification.approved",
    "certification.rejected"
  ],
  "secret": "your-webhook-secret"
}
```

### 7.2 Webhook Payload Example

```json
{
  "eventId": "evt_2025_001",
  "eventType": "lot.certified",
  "timestamp": "2025-02-10T10:30:00Z",
  "data": {
    "lotId": "KSC-2025-001",
    "certificateId": "CERT-2025-12345",
    "status": "approved"
  },
  "signature": "sha256=..."
}
```

---

## 8. Error Handling

### 8.1 Standard Error Response

```json
{
  "error": {
    "code": "INVALID_GERMINATION_RATE",
    "message": "Germination rate must be between 0 and 100",
    "field": "qualityMetrics.germinationRate",
    "details": {
      "providedValue": 105,
      "allowedRange": "0-100"
    }
  },
  "timestamp": "2025-01-15T10:30:00Z",
  "requestId": "req_abc123"
}
```

### 8.2 Common Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| `UNAUTHORIZED` | 401 | Invalid or missing API key |
| `FORBIDDEN` | 403 | Insufficient permissions |
| `NOT_FOUND` | 404 | Resource not found |
| `VALIDATION_ERROR` | 400 | Invalid request data |
| `DUPLICATE_LOT_ID` | 409 | Lot ID already exists |
| `INVALID_GERMINATION_RATE` | 400 | Germination rate out of range |
| `CERTIFICATION_FAILED` | 422 | Seed lot does not meet certification criteria |
| `RATE_LIMIT_EXCEEDED` | 429 | Too many requests |

---

## 9. Rate Limits

- **Standard Plan:** 1,000 requests/hour
- **Professional Plan:** 10,000 requests/hour
- **Enterprise Plan:** Unlimited

Rate limit headers:
```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 850
X-RateLimit-Reset: 1642348800
```

---

## 10. SDK Support

Official SDKs available:

```javascript
// JavaScript/TypeScript
import { WIASeedAPI } from '@wia/seed-sdk';
const api = new WIASeedAPI({ apiKey: 'your-api-key' });
const lot = await api.lots.create({ ... });
```

```python
# Python
from wia_seed import SeedAPI
api = SeedAPI(api_key='your-api-key')
lot = api.lots.create(**kwargs)
```

```go
// Go
import "github.com/wia-official/seed-sdk-go"
client := seed.NewClient("your-api-key")
lot, err := client.Lots.Create(context.Background(), req)
```

---

## 11. API Versioning

API versions are specified in the URL path:
- `/v1` - Current stable version
- `/v2` - Next version (beta)

Version support policy:
- Each version supported for minimum 24 months
- Deprecation notices sent 12 months in advance

---

**Next Phase:** [Phase 3 - Protocol Specification](./PHASE-3-PROTOCOL.md)
