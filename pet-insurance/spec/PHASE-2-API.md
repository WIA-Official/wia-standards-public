# WIA-PET-010 Pet Insurance - Phase 2: API Interface

**Version:** 1.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-12-25

---

## 1. Overview

Phase 2 defines the RESTful API interface for the WIA Pet Insurance Standard. This specification enables insurance providers, veterinary clinics, and pet owners to interact with insurance systems through standardized endpoints.

### 1.1 API Principles

- **REST Architecture**: Standard HTTP methods (GET, POST, PUT, DELETE)
- **JSON Format**: All requests and responses use JSON
- **Authentication**: OAuth 2.0 and API keys
- **Rate Limiting**: Prevent abuse and ensure fair usage
- **Versioning**: URL-based versioning (/v1/, /v2/)
- **HTTPS Only**: All endpoints require TLS 1.3+

### 1.2 Base URL

```
Production: https://api.wiastandards.com/pet-insurance/v1
Staging: https://api-staging.wiastandards.com/pet-insurance/v1
```

---

## 2. Authentication

### 2.1 OAuth 2.0 Flow

```http
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials
&client_id=YOUR_CLIENT_ID
&client_secret=YOUR_CLIENT_SECRET
&scope=policies:read policies:write claims:read claims:write
```

**Response:**
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "policies:read policies:write claims:read claims:write"
}
```

### 2.2 API Key Authentication

```http
GET /policies
Authorization: Bearer YOUR_ACCESS_TOKEN
X-API-Key: your-api-key-here
```

### 2.3 Scopes

| Scope | Description |
|-------|-------------|
| `policies:read` | Read policy information |
| `policies:write` | Create and update policies |
| `claims:read` | Read claim information |
| `claims:write` | Submit and update claims |
| `premiums:calculate` | Calculate premiums |
| `fraud:check` | Access fraud detection |
| `admin:all` | Full administrative access |

---

## 3. Policy Management API

### 3.1 Create Policy

**Endpoint:** `POST /policies`

**Request:**
```json
{
  "pet": {
    "id": "PET-2025-789456",
    "name": "Buddy",
    "species": "Dog",
    "breed": "Golden Retriever",
    "birthDate": "2020-05-15",
    "microchipId": "985112345678901"
  },
  "owner": {
    "id": "OWNER-2025-456789",
    "name": "John Smith",
    "email": "john.smith@example.com",
    "phone": "+1-555-123-4567"
  },
  "plan": {
    "tier": "Premium",
    "startDate": "2025-01-01"
  }
}
```

**Response (201 Created):**
```json
{
  "policyId": "PET-INS-2025-001234",
  "status": "Active",
  "monthlyPremium": 89.99,
  "annualLimit": 15000,
  "deductible": 250,
  "createdAt": "2025-01-01T00:00:00Z",
  "blockchain": {
    "policyHash": "0x1234567890abcdef...",
    "transactionId": "0xfedcba0987654321..."
  }
}
```

### 3.2 Get Policy

**Endpoint:** `GET /policies/{policyId}`

**Response (200 OK):**
```json
{
  "policyId": "PET-INS-2025-001234",
  "standardVersion": "WIA-PET-010-v1.0.0",
  "pet": { ... },
  "owner": { ... },
  "plan": { ... },
  "coverage": { ... },
  "riskFactors": { ... },
  "status": "Active"
}
```

### 3.3 Update Policy

**Endpoint:** `PUT /policies/{policyId}`

**Request:**
```json
{
  "plan": {
    "tier": "Premium"
  },
  "owner": {
    "email": "newemail@example.com",
    "phone": "+1-555-999-8888"
  }
}
```

**Response (200 OK):**
```json
{
  "policyId": "PET-INS-2025-001234",
  "status": "Active",
  "updatedAt": "2025-12-25T10:30:00Z",
  "changes": ["plan.tier", "owner.email", "owner.phone"]
}
```

### 3.4 Cancel Policy

**Endpoint:** `DELETE /policies/{policyId}`

**Request:**
```json
{
  "reason": "Owner request",
  "effectiveDate": "2025-12-31"
}
```

**Response (200 OK):**
```json
{
  "policyId": "PET-INS-2025-001234",
  "status": "Cancelled",
  "cancelledAt": "2025-12-25T10:30:00Z",
  "effectiveDate": "2025-12-31",
  "refundAmount": 150.00
}
```

### 3.5 List Policies

**Endpoint:** `GET /policies`

**Query Parameters:**
- `owner_id` - Filter by owner ID
- `status` - Filter by status (Active, Suspended, Cancelled, Expired)
- `tier` - Filter by plan tier
- `limit` - Number of results (default: 50, max: 100)
- `offset` - Pagination offset

**Example:**
```http
GET /policies?owner_id=OWNER-2025-456789&status=Active&limit=10
```

**Response (200 OK):**
```json
{
  "total": 1,
  "limit": 10,
  "offset": 0,
  "policies": [
    {
      "policyId": "PET-INS-2025-001234",
      "pet": { "name": "Buddy", "species": "Dog" },
      "plan": { "tier": "Premium" },
      "status": "Active",
      "monthlyPremium": 89.99
    }
  ]
}
```

---

## 4. Claims Management API

### 4.1 Submit Claim

**Endpoint:** `POST /claims`

**Request:**
```json
{
  "policyId": "PET-INS-2025-001234",
  "type": "Illness",
  "treatmentDate": "2025-12-20",
  "amount": 850.00,
  "clinic": {
    "id": "VET-2025-456",
    "name": "City Pet Hospital"
  },
  "diagnosis": {
    "code": "K29.70",
    "description": "Gastritis, unspecified"
  },
  "documents": [
    {
      "type": "Invoice",
      "url": "https://storage.example.com/invoice.pdf"
    }
  ]
}
```

**Response (201 Created):**
```json
{
  "claimId": "CLM-2025-001234",
  "status": "Processing",
  "estimatedProcessingDays": 3,
  "submittedAt": "2025-12-21T10:30:00Z",
  "amounts": {
    "total": 850.00,
    "estimated": {
      "insurancePays": 480.00,
      "ownerPays": 370.00
    }
  }
}
```

### 4.2 Get Claim Status

**Endpoint:** `GET /claims/{claimId}`

**Response (200 OK):**
```json
{
  "claimId": "CLM-2025-001234",
  "policyId": "PET-INS-2025-001234",
  "status": "Approved",
  "type": "Illness",
  "treatmentDate": "2025-12-20",
  "amounts": {
    "total": 850.00,
    "eligible": 850.00,
    "deductible": 250.00,
    "insurancePays": 480.00,
    "ownerPays": 370.00
  },
  "payment": {
    "processedDate": "2025-12-22T14:00:00Z",
    "transactionId": "PAY-2025-567890"
  },
  "timeline": [
    {
      "status": "Submitted",
      "timestamp": "2025-12-21T10:30:00Z"
    },
    {
      "status": "Processing",
      "timestamp": "2025-12-21T10:35:00Z"
    },
    {
      "status": "Approved",
      "timestamp": "2025-12-22T09:00:00Z"
    }
  ]
}
```

### 4.3 Upload Claim Documents

**Endpoint:** `POST /claims/{claimId}/documents`

**Request (multipart/form-data):**
```http
POST /claims/CLM-2025-001234/documents
Content-Type: multipart/form-data

file: (binary)
type: Invoice
description: Veterinary invoice for treatment
```

**Response (201 Created):**
```json
{
  "documentId": "DOC-2025-789012",
  "claimId": "CLM-2025-001234",
  "type": "Invoice",
  "url": "https://secure.storage.wia/claims/CLM-2025-001234/invoice.pdf",
  "hash": "sha256:9f86d081884c7d659a2feaa0c55ad015a3bf4f1b2b0b822cd15d6c15b0f00a08",
  "uploadedAt": "2025-12-21T10:31:00Z"
}
```

### 4.4 List Claims

**Endpoint:** `GET /claims`

**Query Parameters:**
- `policy_id` - Filter by policy ID
- `status` - Filter by status
- `type` - Filter by claim type
- `from_date` - Claims from date (ISO 8601)
- `to_date` - Claims to date (ISO 8601)
- `limit` - Number of results
- `offset` - Pagination offset

**Example:**
```http
GET /claims?policy_id=PET-INS-2025-001234&status=Approved&limit=10
```

**Response (200 OK):**
```json
{
  "total": 3,
  "limit": 10,
  "offset": 0,
  "claims": [
    {
      "claimId": "CLM-2025-001234",
      "type": "Illness",
      "status": "Approved",
      "amount": 850.00,
      "insurancePays": 480.00,
      "treatmentDate": "2025-12-20"
    }
  ]
}
```

---

## 5. Premium Calculation API

### 5.1 Calculate Premium

**Endpoint:** `POST /premiums/calculate`

**Request:**
```json
{
  "pet": {
    "species": "Dog",
    "breed": "Golden Retriever",
    "birthDate": "2020-05-15"
  },
  "owner": {
    "zipCode": "94102",
    "location": "Urban"
  },
  "plan": {
    "tier": "Premium"
  },
  "healthPassportId": "WIA-PET-HEALTH-001234"
}
```

**Response (200 OK):**
```json
{
  "calculationId": "CALC-2025-123456",
  "basePremium": 60.00,
  "factors": {
    "speciesFactor": { "species": "Dog", "multiplier": 1.2 },
    "ageFactor": { "age": 4, "multiplier": 1.0 },
    "breedFactor": { "breed": "Golden Retriever", "riskLevel": "Medium", "multiplier": 1.0 },
    "locationFactor": { "location": "Urban", "multiplier": 1.1 },
    "healthFactor": { "healthScore": 85, "multiplier": 0.9 },
    "tierFactor": { "tier": "Premium", "multiplier": 1.5 }
  },
  "discounts": [
    { "type": "HealthScore", "amount": 5.00, "percentage": 5.5 }
  ],
  "finalPremium": {
    "monthly": 89.99,
    "annual": 1079.88
  },
  "breakdown": {
    "base": 60.00,
    "adjustments": 34.99,
    "discounts": -5.00,
    "total": 89.99
  }
}
```

### 5.2 Get Premium History

**Endpoint:** `GET /policies/{policyId}/premiums`

**Response (200 OK):**
```json
{
  "policyId": "PET-INS-2025-001234",
  "currentPremium": 89.99,
  "history": [
    {
      "effectiveDate": "2025-01-01",
      "monthlyPremium": 89.99,
      "reason": "Initial policy"
    },
    {
      "effectiveDate": "2025-06-01",
      "monthlyPremium": 84.99,
      "reason": "Health score improvement discount"
    }
  ]
}
```

---

## 6. Coverage Verification API

### 6.1 Verify Coverage

**Endpoint:** `POST /coverage/verify`

**Request:**
```json
{
  "policyId": "PET-INS-2025-001234",
  "qrCode": "optional-qr-code-data",
  "treatmentType": "Surgery",
  "estimatedCost": 2500.00
}
```

**Response (200 OK):**
```json
{
  "policyId": "PET-INS-2025-001234",
  "status": "Active",
  "coverageStatus": "Covered",
  "treatmentType": "Surgery",
  "coverage": {
    "covered": true,
    "annualLimitRemaining": 12500.00,
    "deductibleRemaining": 0.00,
    "estimatedCoverage": {
      "totalCost": 2500.00,
      "insurancePays": 2000.00,
      "ownerPays": 500.00
    }
  },
  "preApprovalRequired": false,
  "verifiedAt": "2025-12-25T10:30:00Z"
}
```

### 6.2 QR Code Verification

**Endpoint:** `GET /coverage/qr/{qrCode}`

**Response (200 OK):**
```json
{
  "policyId": "PET-INS-2025-001234",
  "pet": {
    "name": "Buddy",
    "species": "Dog",
    "microchipId": "985112345678901"
  },
  "owner": {
    "name": "John Smith",
    "phone": "+1-555-123-4567"
  },
  "plan": {
    "tier": "Premium",
    "annualLimit": 15000,
    "deductible": 250
  },
  "status": "Active",
  "verified": true
}
```

---

## 7. Fraud Detection API

### 7.1 Check Claim for Fraud

**Endpoint:** `POST /fraud/check`

**Request:**
```json
{
  "claimId": "CLM-2025-001234",
  "policyId": "PET-INS-2025-001234",
  "amount": 850.00,
  "clinic": {
    "id": "VET-2025-456"
  },
  "treatmentDate": "2025-12-20"
}
```

**Response (200 OK):**
```json
{
  "claimId": "CLM-2025-001234",
  "fraudScore": 15,
  "riskLevel": "Low",
  "flags": [],
  "recommendations": "Auto-approve",
  "checkedAt": "2025-12-21T10:32:00Z",
  "details": {
    "clinicVerified": true,
    "amountReasonable": true,
    "duplicateClaim": false,
    "frequencyNormal": true,
    "patternMatch": false
  }
}
```

### 7.2 Report Suspicious Activity

**Endpoint:** `POST /fraud/report`

**Request:**
```json
{
  "claimId": "CLM-2025-001235",
  "reason": "Duplicate claim with different dates",
  "reportedBy": "VET-2025-456",
  "evidence": [
    "Previous claim CLM-2025-000123 has identical diagnosis"
  ]
}
```

**Response (201 Created):**
```json
{
  "reportId": "FRAUD-REPORT-2025-001",
  "status": "Under Investigation",
  "claimId": "CLM-2025-001235",
  "submittedAt": "2025-12-21T14:00:00Z"
}
```

---

## 8. Health Integration API

### 8.1 Sync Health Passport

**Endpoint:** `POST /integration/health-passport/sync`

**Request:**
```json
{
  "policyId": "PET-INS-2025-001234",
  "healthPassportId": "WIA-PET-HEALTH-001234"
}
```

**Response (200 OK):**
```json
{
  "policyId": "PET-INS-2025-001234",
  "healthPassportId": "WIA-PET-HEALTH-001234",
  "syncedAt": "2025-12-25T10:30:00Z",
  "recordsSynced": 24,
  "healthScore": 85,
  "premiumAdjustment": {
    "previousPremium": 89.99,
    "newPremium": 84.99,
    "discount": 5.00,
    "reason": "Health score improvement"
  }
}
```

### 8.2 Get Health-Based Discounts

**Endpoint:** `GET /integration/health-passport/{healthPassportId}/discounts`

**Response (200 OK):**
```json
{
  "healthPassportId": "WIA-PET-HEALTH-001234",
  "healthScore": 85,
  "discounts": [
    {
      "type": "HealthScore",
      "amount": 5.00,
      "percentage": 5.5,
      "criteria": "Health score >= 80"
    },
    {
      "type": "PreventiveCare",
      "amount": 3.00,
      "percentage": 3.3,
      "criteria": "Annual checkup completed"
    }
  ],
  "totalDiscount": 8.00,
  "eligibleSince": "2025-06-01"
}
```

---

## 9. Error Handling

### 9.1 Error Response Format

All errors follow this format:

```json
{
  "error": {
    "code": "ERROR_CODE",
    "message": "Human-readable error message",
    "details": {
      "field": "Additional error context"
    },
    "timestamp": "2025-12-25T10:30:00Z",
    "requestId": "req-123456789"
  }
}
```

### 9.2 HTTP Status Codes

| Code | Description | Usage |
|------|-------------|-------|
| 200 | OK | Successful GET, PUT, DELETE |
| 201 | Created | Successful POST |
| 400 | Bad Request | Invalid request data |
| 401 | Unauthorized | Missing or invalid authentication |
| 403 | Forbidden | Insufficient permissions |
| 404 | Not Found | Resource not found |
| 409 | Conflict | Resource conflict (e.g., duplicate) |
| 422 | Unprocessable Entity | Validation errors |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Server Error | Server error |
| 503 | Service Unavailable | Temporary unavailability |

### 9.3 Common Error Codes

| Error Code | Description |
|------------|-------------|
| `INVALID_POLICY_ID` | Policy ID format invalid or not found |
| `INVALID_CLAIM_ID` | Claim ID format invalid or not found |
| `INSUFFICIENT_COVERAGE` | Treatment not covered by policy |
| `DEDUCTIBLE_NOT_MET` | Annual deductible not yet met |
| `ANNUAL_LIMIT_EXCEEDED` | Annual coverage limit exceeded |
| `POLICY_EXPIRED` | Policy is no longer active |
| `DUPLICATE_CLAIM` | Claim already exists for this treatment |
| `FRAUD_DETECTED` | Potential fraudulent activity detected |
| `INVALID_QR_CODE` | QR code invalid or expired |
| `DOCUMENT_UPLOAD_FAILED` | Document upload error |

### 9.4 Example Error Response

```json
{
  "error": {
    "code": "ANNUAL_LIMIT_EXCEEDED",
    "message": "Annual coverage limit of $15,000 has been exceeded",
    "details": {
      "annualLimit": 15000,
      "currentUsage": 14500,
      "requestedAmount": 850,
      "remainingCoverage": 500
    },
    "timestamp": "2025-12-25T10:30:00Z",
    "requestId": "req-abc123def456"
  }
}
```

---

## 10. Rate Limiting

### 10.1 Rate Limit Headers

All API responses include rate limit headers:

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 987
X-RateLimit-Reset: 1640437200
```

### 10.2 Rate Limits by Tier

| Tier | Requests/Hour | Burst |
|------|---------------|-------|
| Free | 100 | 10 |
| Basic | 1,000 | 50 |
| Professional | 10,000 | 200 |
| Enterprise | Unlimited | Unlimited |

### 10.3 Rate Limit Exceeded Response

```json
{
  "error": {
    "code": "RATE_LIMIT_EXCEEDED",
    "message": "API rate limit exceeded",
    "details": {
      "limit": 1000,
      "resetAt": "2025-12-25T11:00:00Z"
    },
    "timestamp": "2025-12-25T10:30:00Z",
    "requestId": "req-xyz789abc123"
  }
}
```

---

## 11. Webhooks

### 11.1 Configure Webhook

**Endpoint:** `POST /webhooks`

**Request:**
```json
{
  "url": "https://your-server.com/webhook",
  "events": ["claim.approved", "claim.rejected", "policy.renewed"],
  "secret": "your-webhook-secret"
}
```

**Response (201 Created):**
```json
{
  "webhookId": "WEBHOOK-2025-001",
  "url": "https://your-server.com/webhook",
  "events": ["claim.approved", "claim.rejected", "policy.renewed"],
  "status": "Active",
  "createdAt": "2025-12-25T10:30:00Z"
}
```

### 11.2 Webhook Events

| Event | Description |
|-------|-------------|
| `policy.created` | New policy created |
| `policy.updated` | Policy modified |
| `policy.cancelled` | Policy cancelled |
| `policy.renewed` | Policy renewed |
| `claim.submitted` | New claim submitted |
| `claim.approved` | Claim approved |
| `claim.rejected` | Claim rejected |
| `claim.paid` | Payment processed |
| `fraud.detected` | Fraudulent activity detected |
| `premium.changed` | Premium amount changed |

### 11.3 Webhook Payload Example

```json
{
  "eventId": "evt-123456789",
  "eventType": "claim.approved",
  "timestamp": "2025-12-22T09:00:00Z",
  "data": {
    "claimId": "CLM-2025-001234",
    "policyId": "PET-INS-2025-001234",
    "amount": 850.00,
    "insurancePays": 480.00,
    "approvedAt": "2025-12-22T09:00:00Z"
  },
  "signature": "sha256:abcdef123456..."
}
```

---

## 12. API Versioning

### 12.1 Version Format

- Current version: **v1**
- URL format: `/v1/endpoint`
- Header: `Accept: application/vnd.wia.pet-insurance.v1+json`

### 12.2 Deprecation Policy

- Minimum 6 months notice before deprecation
- Deprecation header: `X-API-Deprecated: true`
- Sunset header: `Sunset: Sat, 31 Dec 2025 23:59:59 GMT`

---

## 13. SDK Examples

### 13.1 JavaScript/Node.js

```javascript
const WIAPetInsurance = require('@wia/pet-insurance');

const client = new WIAPetInsurance({
  apiKey: 'your-api-key',
  environment: 'production'
});

// Create policy
const policy = await client.policies.create({
  pet: { ... },
  owner: { ... },
  plan: { tier: 'Premium' }
});

// Submit claim
const claim = await client.claims.submit({
  policyId: 'PET-INS-2025-001234',
  type: 'Illness',
  amount: 850
});
```

### 13.2 Python

```python
from wia_pet_insurance import WIAPetInsurance

client = WIAPetInsurance(
    api_key='your-api-key',
    environment='production'
)

# Create policy
policy = client.policies.create(
    pet={...},
    owner={...},
    plan={'tier': 'Premium'}
)

# Submit claim
claim = client.claims.submit(
    policy_id='PET-INS-2025-001234',
    type='Illness',
    amount=850
)
```

---

**弘益人間 · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 MIT License*
