# Chapter 5: API Interface Specifications

## Overview

This chapter details the RESTful API design for the WIA KYC/AML Standard, providing endpoints for identity verification, screening, risk assessment, transaction monitoring, and case management.

---

## API Design Principles

### 1. RESTful Architecture
- Resource-oriented URLs
- HTTP methods semantics (GET, POST, PUT, PATCH, DELETE)
- Stateless requests
- HATEOAS links for discoverability

### 2. Consistent Patterns
- Standard error responses
- Uniform pagination
- Common filtering and sorting
- Predictable naming conventions

### 3. Security First
- OAuth 2.0 / API Key authentication
- TLS 1.3 encryption
- Rate limiting
- Request validation

### 4. Developer-Friendly
- Clear documentation
- Example requests/responses
- SDKs in multiple languages
- Sandbox environment

---

## Base Configuration

### Base URL
```
Production: https://api.wia-kyc.org/v1
Sandbox: https://sandbox-api.wia-kyc.org/v1
```

### Authentication

**OAuth 2.0 (Recommended)**
```http
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials&
client_id=YOUR_CLIENT_ID&
client_secret=YOUR_CLIENT_SECRET&
scope=kyc:read kyc:write
```

Response:
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "kyc:read kyc:write"
}
```

**API Key (Alternative)**
```http
GET /customers/CUST-123456
Authorization: ApiKey YOUR_API_KEY
```

### Request Headers

```http
Authorization: Bearer {access_token}
Content-Type: application/json
Accept: application/json
X-Request-ID: {unique-request-id}
X-Idempotency-Key: {idempotency-key}
```

### Standard Response Format

**Success Response:**
```json
{
  "status": "success",
  "data": {
    // Resource data
  },
  "metadata": {
    "requestId": "req-123456",
    "timestamp": "2025-01-09T10:30:00Z",
    "version": "1.0"
  }
}
```

**Error Response:**
```json
{
  "status": "error",
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "Invalid date format",
    "details": [
      {
        "field": "dateOfBirth",
        "issue": "Must be in YYYY-MM-DD format"
      }
    ]
  },
  "metadata": {
    "requestId": "req-123456",
    "timestamp": "2025-01-09T10:30:00Z"
  }
}
```

---

## Customer Management APIs

### Create Customer

```http
POST /customers
```

**Request Body:**
```json
{
  "type": "individual",
  "personalInfo": {
    "firstName": "John",
    "middleName": "Michael",
    "lastName": "Smith",
    "dateOfBirth": "1985-06-15",
    "nationality": ["USA"]
  },
  "contactInfo": {
    "email": {
      "address": "john.smith@example.com"
    },
    "phone": {
      "number": "+1-555-0123"
    }
  },
  "addresses": [
    {
      "type": "residential",
      "street": "123 Main St",
      "city": "San Francisco",
      "state": "CA",
      "postalCode": "94102",
      "country": "USA",
      "primary": true
    }
  ]
}
```

**Response:** `201 Created`
```json
{
  "status": "success",
  "data": {
    "customerId": "CUST-789012",
    "type": "individual",
    "status": "pending_verification",
    "createdAt": "2025-01-09T10:30:00Z",
    "personalInfo": {
      "firstName": "John",
      "middleName": "Michael",
      "lastName": "Smith",
      "dateOfBirth": "1985-06-15",
      "nationality": ["USA"]
    },
    // ... full customer profile
  }
}
```

### Get Customer

```http
GET /customers/{customerId}
```

**Response:** `200 OK`
```json
{
  "status": "success",
  "data": {
    "customerId": "CUST-789012",
    // ... full customer profile
  }
}
```

### Update Customer

```http
PATCH /customers/{customerId}
```

**Request Body:**
```json
{
  "contactInfo": {
    "phone": {
      "number": "+1-555-9999"
    }
  }
}
```

**Response:** `200 OK`

### List Customers

```http
GET /customers?status=active&riskCategory=high&page=1&limit=20
```

**Query Parameters:**
- `status` - Filter by status
- `riskCategory` - Filter by risk category
- `createdAfter` - ISO 8601 date
- `createdBefore` - ISO 8601 date
- `page` - Page number (default: 1)
- `limit` - Items per page (default: 20, max: 100)
- `sort` - Sort field (e.g., `createdAt`, `-riskScore`)

**Response:** `200 OK`
```json
{
  "status": "success",
  "data": [
    {
      "customerId": "CUST-789012",
      // ... customer summary
    }
  ],
  "pagination": {
    "currentPage": 1,
    "pageSize": 20,
    "totalItems": 150,
    "totalPages": 8,
    "links": {
      "self": "/customers?page=1&limit=20",
      "next": "/customers?page=2&limit=20",
      "last": "/customers?page=8&limit=20"
    }
  }
}
```

---

## Identity Verification APIs

### Initiate Verification

```http
POST /identity/verifications
```

**Request Body:**
```json
{
  "customerId": "CUST-789012",
  "verificationType": "document_and_biometric",
  "returnUrl": "https://yourapp.com/verification/complete",
  "webhookUrl": "https://yourapp.com/webhooks/verification"
}
```

**Response:** `201 Created`
```json
{
  "status": "success",
  "data": {
    "verificationId": "IDV-2025-001234",
    "customerId": "CUST-789012",
    "status": "pending",
    "verificationType": "document_and_biometric",
    "createdAt": "2025-01-09T10:10:00Z",
    "sessionUrl": "https://verify.wia-kyc.org/session/abc123def456",
    "expiresAt": "2025-01-09T11:10:00Z"
  }
}
```

### Upload Document

```http
POST /identity/verifications/{verificationId}/documents
Content-Type: multipart/form-data
```

**Form Data:**
- `documentType` - passport, drivers_license, national_id
- `documentSide` - front, back
- `file` - Image file (JPEG, PNG)

**Response:** `200 OK`
```json
{
  "status": "success",
  "data": {
    "documentId": "DOC-001",
    "uploadedAt": "2025-01-09T10:11:00Z",
    "status": "processing"
  }
}
```

### Submit Biometric

```http
POST /identity/verifications/{verificationId}/biometric
```

**Request Body:**
```json
{
  "biometricType": "facial",
  "imageData": "base64_encoded_image_data",
  "livenessVideo": "base64_encoded_video_data"
}
```

**Response:** `200 OK`
```json
{
  "status": "success",
  "data": {
    "biometricId": "BIO-FACE-001",
    "status": "processing",
    "submittedAt": "2025-01-09T10:15:00Z"
  }
}
```

### Get Verification Result

```http
GET /identity/verifications/{verificationId}
```

**Response:** `200 OK`
```json
{
  "status": "success",
  "data": {
    "verificationId": "IDV-2025-001234",
    "customerId": "CUST-789012",
    "status": "verified",
    "overallResult": "pass",
    "confidenceScore": 0.94,
    "completedAt": "2025-01-09T10:15:00Z",
    "documentVerification": {
      // ... detailed results
    },
    "biometricVerification": {
      // ... detailed results
    }
  }
}
```

---

## Screening APIs

### Perform Comprehensive Screening

```http
POST /screening/comprehensive
```

**Request Body:**
```json
{
  "customerId": "CUST-789012",
  "screeningType": ["sanctions", "pep", "adverse_media"],
  "fuzzyMatching": true,
  "threshold": 85,
  "continuousMonitoring": true
}
```

**Response:** `200 OK`
```json
{
  "status": "success",
  "data": {
    "screeningId": "SCR-2025-001234",
    "customerId": "CUST-789012",
    "executedAt": "2025-01-09T10:12:00Z",
    "sanctionsScreening": {
      "status": "no_match",
      "listsChecked": ["OFAC_SDN", "UN_SC", "EU_SANCTIONS"],
      "totalRecordsChecked": 25614
    },
    "pepScreening": {
      "status": "potential_match",
      "matches": [
        {
          "matchId": "PEP-2025-5678",
          "confidence": 78,
          "requiresReview": true,
          "profile": {
            // ... PEP profile
          }
        }
      ]
    },
    "adverseMediaScreening": {
      "status": "clear",
      "articlesReviewed": 0
    },
    "overallRisk": "low_risk_pending_review",
    "recommendedAction": "manual_review_pep_match"
  }
}
```

### Sanctions Screening Only

```http
POST /screening/sanctions
```

**Request Body:**
```json
{
  "name": "John Michael Smith",
  "dateOfBirth": "1985-06-15",
  "nationality": "USA",
  "lists": ["OFAC_SDN", "UN_SC", "EU_SANCTIONS"]
}
```

**Response:** `200 OK`
```json
{
  "status": "success",
  "data": {
    "screeningId": "SCR-SANC-001234",
    "status": "no_match",
    "executedAt": "2025-01-09T10:12:00Z",
    "listsChecked": [
      {
        "listName": "OFAC_SDN",
        "listVersion": "2025-01-09",
        "recordCount": 12458,
        "matches": 0
      }
    ]
  }
}
```

### PEP Screening Only

```http
POST /screening/pep
```

**Request Body:**
```json
{
  "customerId": "CUST-789012",
  "name": "John Michael Smith",
  "dateOfBirth": "1985-06-15",
  "nationality": "USA"
}
```

**Response:** `200 OK`

### Batch Screening

```http
POST /screening/batch
```

**Request Body:**
```json
{
  "screeningType": ["sanctions", "pep"],
  "customers": [
    {
      "customerId": "CUST-001",
      "name": "John Smith",
      "dateOfBirth": "1985-06-15"
    },
    {
      "customerId": "CUST-002",
      "name": "Jane Doe",
      "dateOfBirth": "1990-03-20"
    }
  ]
}
```

**Response:** `202 Accepted`
```json
{
  "status": "success",
  "data": {
    "batchId": "BATCH-2025-001",
    "status": "processing",
    "totalRecords": 2,
    "estimatedCompletionTime": "2025-01-09T10:20:00Z",
    "statusUrl": "/screening/batch/BATCH-2025-001"
  }
}
```

---

## Risk Assessment APIs

### Assess Customer Risk

```http
POST /risk/assess
```

**Request Body:**
```json
{
  "customerId": "CUST-789012",
  "assessmentType": "comprehensive",
  "includeFactors": true
}
```

**Response:** `200 OK`
```json
{
  "status": "success",
  "data": {
    "assessmentId": "RISK-2025-001234",
    "customerId": "CUST-789012",
    "assessmentDate": "2025-01-09T10:30:00Z",
    "overallRisk": {
      "score": 35,
      "category": "low"
    },
    "riskDimensions": [
      // ... detailed risk factors
    ],
    "recommendations": {
      "approvalDecision": "approve",
      "cddLevel": "standard",
      "reviewFrequency": "biennial",
      "nextReviewDate": "2027-01-09"
    }
  }
}
```

### Get Risk Score

```http
GET /risk/score/{customerId}
```

**Response:** `200 OK`
```json
{
  "status": "success",
  "data": {
    "customerId": "CUST-789012",
    "riskScore": 35,
    "riskCategory": "low",
    "lastAssessment": "2025-01-09T10:30:00Z",
    "nextReview": "2027-01-09"
  }
}
```

### Update Risk Parameters

```http
PATCH /risk/parameters
```

**Request Body:**
```json
{
  "dimensionWeights": {
    "geographic": 0.25,
    "product": 0.20,
    "customerType": 0.20,
    "behavioral": 0.20,
    "relationship": 0.15
  },
  "riskThresholds": {
    "low": 39,
    "medium": 69,
    "high": 89
  }
}
```

**Response:** `200 OK`

---

## Transaction Monitoring APIs

### Submit Transaction

```http
POST /monitoring/transactions
```

**Request Body:**
```json
{
  "transactionId": "TXN-001-20250109",
  "customerId": "CUST-789012",
  "accountId": "ACC-CUST-789012-001",
  "timestamp": "2025-01-09T10:00:00Z",
  "type": "wire_credit",
  "amount": 45000,
  "currency": "USD",
  "counterparty": {
    "name": "ABC Trading Ltd",
    "account": "XXXX-5678",
    "bank": "Foreign Bank Corp",
    "country": "SGP"
  },
  "description": "Payment for consulting services"
}
```

**Response:** `200 OK`
```json
{
  "status": "success",
  "data": {
    "transactionId": "TXN-001-20250109",
    "processed": true,
    "alerts": [],
    "riskScore": 25
  }
}
```

### Get Alerts

```http
GET /monitoring/alerts?status=open&priority=high&page=1&limit=20
```

**Query Parameters:**
- `status` - open, under_investigation, closed
- `priority` - low, medium, high, critical
- `customerId` - Filter by customer
- `dateFrom` / `dateTo` - Date range
- `scenarioCode` - Filter by scenario

**Response:** `200 OK`
```json
{
  "status": "success",
  "data": [
    {
      "alertId": "ALERT-2025-123456",
      "customerId": "CUST-789012",
      "generatedAt": "2025-01-09T14:30:00Z",
      "status": "open",
      "priority": "high",
      "scenario": "rapid_funds_movement",
      "riskScore": 0.82
    }
  ],
  "pagination": {
    // ... pagination info
  }
}
```

### Get Alert Details

```http
GET /monitoring/alerts/{alertId}
```

**Response:** `200 OK`
```json
{
  "status": "success",
  "data": {
    "alertId": "ALERT-2025-123456",
    // ... full alert details
  }
}
```

### Dispose Alert

```http
POST /monitoring/alerts/{alertId}/dispose
```

**Request Body:**
```json
{
  "disposition": "false_positive",
  "notes": "Transaction explained by legitimate business activity. Supporting documentation verified.",
  "disposedBy": "analyst_jdoe"
}
```

**Response:** `200 OK`

### Configure Monitoring Rules

```http
POST /monitoring/rules
```

**Request Body:**
```json
{
  "ruleName": "High Value Cash Deposits",
  "ruleType": "threshold",
  "enabled": true,
  "condition": {
    "transactionType": "cash_deposit",
    "amount": {
      "greaterThan": 10000
    },
    "currency": "USD"
  },
  "action": "generate_alert",
  "severity": "medium"
}
```

**Response:** `201 Created`

---

## Case Management APIs

### Create Case

```http
POST /cases
```

**Request Body:**
```json
{
  "caseType": "suspicious_activity_investigation",
  "customerId": "CUST-789012",
  "priority": "high",
  "trigger": {
    "type": "transaction_monitoring_alert",
    "sourceId": "ALERT-2025-123456"
  },
  "assignedTo": "analyst_jdoe"
}
```

**Response:** `201 Created`
```json
{
  "status": "success",
  "data": {
    "caseId": "CASE-2025-001234",
    "status": "under_investigation",
    "createdAt": "2025-01-09T14:45:00Z",
    // ... case details
  }
}
```

### Get Case

```http
GET /cases/{caseId}
```

**Response:** `200 OK`

### Add Investigation Step

```http
POST /cases/{caseId}/investigation/steps
```

**Request Body:**
```json
{
  "action": "customer_contact",
  "notes": "Spoke with customer. Provided documentation...",
  "performedBy": "analyst_jdoe",
  "duration": 900,
  "contactMethod": "phone",
  "outcome": "successful"
}
```

**Response:** `201 Created`

### Upload Evidence

```http
POST /cases/{caseId}/evidence
Content-Type: multipart/form-data
```

**Form Data:**
- `evidenceType` - supporting_document, external_search, communication
- `description` - Text description
- `file` - Document file

**Response:** `201 Created`

### File SAR

```http
POST /cases/{caseId}/sar
```

**Request Body:**
```json
{
  "suspiciousActivity": {
    "type": ["structuring", "suspected_money_laundering"],
    "description": "Detailed narrative of suspicious activity...",
    "dateRange": {
      "from": "2024-11-15",
      "to": "2024-12-15"
    },
    "totalAmount": 142500,
    "currency": "USD"
  },
  "transactions": [
    // ... transaction details
  ],
  "narrative": "Full SAR narrative...",
  "filedBy": {
    "name": "Jane Investigator",
    "title": "Senior AML Analyst"
  }
}
```

**Response:** `201 Created`
```json
{
  "status": "success",
  "data": {
    "sarId": "SAR-2025-001234",
    "caseId": "CASE-2025-001234",
    "filedAt": "2025-01-09T18:00:00Z",
    "status": "submitted",
    "confirmationNumber": "FINCEN-20250109-123456"
  }
}
```

### Close Case

```http
POST /cases/{caseId}/close
```

**Request Body:**
```json
{
  "disposition": "closed_false_positive",
  "finalNotes": "Investigation determined activity legitimate. Case closed.",
  "sarFiled": false
}
```

**Response:** `200 OK`

---

## Webhook Events

### Webhook Configuration

```http
POST /webhooks
```

**Request Body:**
```json
{
  "url": "https://yourapp.com/webhooks/kyc",
  "events": [
    "verification.completed",
    "screening.match_found",
    "alert.generated",
    "case.status_changed"
  ],
  "secret": "your_webhook_secret"
}
```

### Event Types

| Event | Description |
|-------|-------------|
| `customer.created` | New customer created |
| `customer.updated` | Customer profile updated |
| `verification.completed` | Identity verification finished |
| `verification.failed` | Verification failed |
| `screening.completed` | Screening finished |
| `screening.match_found` | Potential match found |
| `risk.assessment_completed` | Risk assessment finished |
| `risk.category_changed` | Risk category changed |
| `alert.generated` | New monitoring alert |
| `alert.escalated` | Alert escalated |
| `case.created` | New case created |
| `case.assigned` | Case assigned to investigator |
| `case.status_changed` | Case status changed |
| `sar.filed` | SAR filed |

### Webhook Payload Example

```json
{
  "eventId": "evt_123456",
  "eventType": "alert.generated",
  "timestamp": "2025-01-09T14:30:00Z",
  "data": {
    "alertId": "ALERT-2025-123456",
    "customerId": "CUST-789012",
    "priority": "high",
    "scenario": "rapid_funds_movement"
  },
  "signature": "sha256_hmac_signature"
}
```

---

## Rate Limiting

### Limits

| Tier | Requests/Minute | Burst |
|------|-----------------|-------|
| **Free** | 60 | 100 |
| **Basic** | 300 | 500 |
| **Professional** | 1,000 | 2,000 |
| **Enterprise** | Custom | Custom |

### Headers

```http
X-RateLimit-Limit: 300
X-RateLimit-Remaining: 287
X-RateLimit-Reset: 1704797400
```

### Rate Limit Error

**Response:** `429 Too Many Requests`
```json
{
  "status": "error",
  "error": {
    "code": "RATE_LIMIT_EXCEEDED",
    "message": "Rate limit exceeded. Please retry after 30 seconds.",
    "retryAfter": 30
  }
}
```

---

## Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| `VALIDATION_ERROR` | 400 | Request validation failed |
| `AUTHENTICATION_ERROR` | 401 | Invalid or missing authentication |
| `AUTHORIZATION_ERROR` | 403 | Insufficient permissions |
| `NOT_FOUND` | 404 | Resource not found |
| `CONFLICT` | 409 | Resource already exists |
| `RATE_LIMIT_EXCEEDED` | 429 | Too many requests |
| `INTERNAL_ERROR` | 500 | Server error |
| `SERVICE_UNAVAILABLE` | 503 | Service temporarily unavailable |

---

## SDK Examples

### TypeScript/Node.js

```typescript
import { WIAKYCClient } from '@wia/kyc-aml';

const client = new WIAKYCClient({
  apiKey: process.env.WIA_API_KEY,
  environment: 'production'
});

// Create customer
const customer = await client.customers.create({
  type: 'individual',
  personalInfo: {
    firstName: 'John',
    lastName: 'Smith',
    dateOfBirth: '1985-06-15'
  }
});

// Initiate verification
const verification = await client.identity.verify({
  customerId: customer.customerId,
  verificationType: 'document_and_biometric'
});

// Perform screening
const screening = await client.screening.comprehensive({
  customerId: customer.customerId,
  screeningType: ['sanctions', 'pep', 'adverse_media']
});
```

### Python

```python
from wia_kyc import WIAKYCClient

client = WIAKYCClient(
    api_key=os.environ['WIA_API_KEY'],
    environment='production'
)

# Create customer
customer = client.customers.create(
    type='individual',
    personal_info={
        'first_name': 'John',
        'last_name': 'Smith',
        'date_of_birth': '1985-06-15'
    }
)

# Perform risk assessment
risk = client.risk.assess(
    customer_id=customer.customer_id,
    assessment_type='comprehensive'
)
```

---

## Key Takeaways

1. 🔌 **RESTful design** with consistent patterns
2. 🔐 **OAuth 2.0** and API Key authentication
3. 📋 **Comprehensive endpoints** for all KYC/AML workflows
4. 🔔 **Webhook support** for real-time notifications
5. ⚡ **Rate limiting** with clear headers
6. 📚 **SDKs available** in multiple languages
7. 🧪 **Sandbox environment** for testing

---

**Previous**: [← Chapter 4 - Data Format](04-data-format.md) | **Next**: [Chapter 6 - Protocol →](06-protocol.md)

---

© 2025 WIA Standards Committee
弘益人間 (홍익인간) - Benefit All Humanity
