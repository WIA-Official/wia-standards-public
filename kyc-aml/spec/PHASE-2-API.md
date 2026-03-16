# WIA-FIN-011 KYC/AML Standard
## Phase 2: API Specification v1.0.0

**Status:** ✅ Complete  
**Last Updated:** 2025-12-25  
**Maintainer:** WIA Standards Committee

---

## 1. Overview

This specification defines the Application Programming Interfaces (APIs) for KYC/AML systems, enabling standardized integration, data exchange, and automation of compliance processes.

### 1.1 API Design Principles

- **RESTful Architecture**: HTTP-based, resource-oriented design
- **Stateless**: Each request contains all necessary information
- **Versioned**: API version in URL path
- **Secure**: OAuth 2.0 / JWT authentication
- **Rate Limited**: Protection against abuse
- **Well-Documented**: OpenAPI 3.0 specification

---

## 2. Authentication & Authorization

### 2.1 OAuth 2.0 Flows

**Client Credentials Flow** (Machine-to-Machine):
```
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials
&client_id={client_id}
&client_secret={client_secret}
&scope=kyc.read kyc.write
```

**Authorization Code Flow** (User Authorization):
```
GET /oauth/authorize?
  response_type=code
  &client_id={client_id}
  &redirect_uri={redirect_uri}
  &scope=kyc.read
  &state={random_state}
```

### 2.2 JWT Token Structure

```json
{
  "header": {
    "alg": "RS256",
    "typ": "JWT"
  },
  "payload": {
    "iss": "https://api.kyc-aml.example.com",
    "sub": "client_id",
    "aud": "kyc-api",
    "exp": 1735776000,
    "iat": 1735689600,
    "scope": "kyc.read kyc.write transaction.monitor"
  }
}
```

### 2.3 API Scopes

- `kyc.read` - Read customer and CDD data
- `kyc.write` - Create/update customer records
- `risk.assess` - Perform risk assessments
- `transaction.monitor` - Access transaction monitoring
- `sar.submit` - Submit suspicious activity reports
- `screening.perform` - Execute screenings
- `admin.manage` - Administrative operations

---

## 3. Core API Endpoints

### 3.1 Customer Management API

**Create Customer**
```
POST /api/v1/customers
Authorization: Bearer {token}
Content-Type: application/json

{
  "customerType": "individual",
  "personalInfo": { ... },
  "contactInfo": { ... }
}

Response 201 Created:
{
  "customerId": "uuid",
  "status": "pending_verification",
  "createdAt": "2025-12-25T10:00:00Z"
}
```

**Get Customer**
```
GET /api/v1/customers/{customerId}
Authorization: Bearer {token}

Response 200 OK:
{
  "customerId": "uuid",
  "customerType": "individual",
  "personalInfo": { ... },
  "riskRating": "medium",
  "lastUpdated": "2025-12-25T10:00:00Z"
}
```

**Update Customer**
```
PATCH /api/v1/customers/{customerId}
Authorization: Bearer {token}
Content-Type: application/json

{
  "addresses": [
    {
      "addressType": "residential",
      "streetAddress1": "123 New St"
    }
  ]
}
```

**Search Customers**
```
GET /api/v1/customers/search?
  name=John+Smith
  &dateOfBirth=1980-01-15
  &country=USA

Response 200 OK:
{
  "results": [
    {
      "customerId": "uuid",
      "matchScore": 95,
      "customerInfo": { ... }
    }
  ],
  "totalResults": 1,
  "page": 1
}
```

### 3.2 Customer Due Diligence API

**Initiate CDD**
```
POST /api/v1/cdd
Authorization: Bearer {token}

{
  "customerId": "uuid",
  "cddType": "standard",
  "triggerReason": "new_customer"
}

Response 202 Accepted:
{
  "cddId": "uuid",
  "status": "in_progress",
  "estimatedCompletion": "2025-12-25T11:00:00Z"
}
```

**Get CDD Status**
```
GET /api/v1/cdd/{cddId}

Response 200 OK:
{
  "cddId": "uuid",
  "customerId": "uuid",
  "status": "completed",
  "overallRiskRating": "low",
  "verificationResults": { ... },
  "completedAt": "2025-12-25T10:30:00Z"
}
```

### 3.3 Document Verification API

**Upload Document**
```
POST /api/v1/documents
Authorization: Bearer {token}
Content-Type: multipart/form-data

file: (binary)
documentType: passport
customerId: uuid

Response 201 Created:
{
  "documentId": "uuid",
  "status": "processing",
  "uploadedAt": "2025-12-25T10:00:00Z"
}
```

**Verify Document**
```
POST /api/v1/documents/{documentId}/verify
Authorization: Bearer {token}

Response 200 OK:
{
  "documentId": "uuid",
  "verificationStatus": "verified",
  "extractedData": {
    "documentNumber": "A12345678",
    "fullName": "JOHN MICHAEL SMITH",
    "dateOfBirth": "1980-01-15",
    "expiryDate": "2030-12-31"
  },
  "validationChecks": {
    "documentAuthenticity": "pass",
    "expiryCheck": "pass",
    "biometricMatch": "pass",
    "mrzValidation": "pass"
  },
  "confidenceScore": 98
}
```

### 3.4 Screening API

**Perform Sanctions Screening**
```
POST /api/v1/screening/sanctions
Authorization: Bearer {token}

{
  "searchType": "individual",
  "firstName": "John",
  "lastName": "Smith",
  "dateOfBirth": "1980-01-15",
  "nationality": "USA"
}

Response 200 OK:
{
  "screeningId": "uuid",
  "searchDate": "2025-12-25T10:00:00Z",
  "totalMatches": 0,
  "clearanceStatus": "clear",
  "listsSearched": [
    "OFAC SDN",
    "UN Consolidated",
    "EU Sanctions"
  ]
}
```

**PEP Screening**
```
POST /api/v1/screening/pep
Authorization: Bearer {token}

{
  "fullName": "John Smith",
  "dateOfBirth": "1980-01-15",
  "country": "USA"
}

Response 200 OK:
{
  "screeningId": "uuid",
  "pepStatus": "not_pep",
  "matches": [],
  "screeningDate": "2025-12-25T10:00:00Z"
}
```

### 3.5 Risk Assessment API

**Calculate Risk Score**
```
POST /api/v1/risk/assess
Authorization: Bearer {token}

{
  "customerId": "uuid",
  "riskFactors": {
    "customerType": "individual",
    "geographicRisk": "low",
    "productRisk": "medium",
    "isPEP": false,
    "highRiskJurisdiction": false
  }
}

Response 200 OK:
{
  "assessmentId": "uuid",
  "customerId": "uuid",
  "overallRiskRating": "medium",
  "riskScore": 45,
  "riskBreakdown": {
    "customerRisk": 20,
    "geographicRisk": 10,
    "productRisk": 15
  },
  "recommendation": "standard_monitoring",
  "assessedAt": "2025-12-25T10:00:00Z"
}
```

### 3.6 Transaction Monitoring API

**Submit Transaction**
```
POST /api/v1/transactions
Authorization: Bearer {token}

{
  "customerId": "uuid",
  "transactionType": "wire",
  "amount": {
    "value": 50000,
    "currency": "USD"
  },
  "originator": { ... },
  "beneficiary": { ... }
}

Response 201 Created:
{
  "transactionId": "uuid",
  "monitoringStatus": "screening",
  "alerts": [],
  "clearanceStatus": "pending"
}
```

**Get Transaction Alerts**
```
GET /api/v1/transactions/{transactionId}/alerts

Response 200 OK:
{
  "transactionId": "uuid",
  "alerts": [
    {
      "alertId": "uuid",
      "alertType": "large_transaction",
      "severity": "medium",
      "description": "Transaction exceeds typical pattern",
      "status": "open",
      "createdAt": "2025-12-25T10:00:00Z"
    }
  ]
}
```

### 3.7 SAR Management API

**Create SAR**
```
POST /api/v1/sar
Authorization: Bearer {token}

{
  "reportType": "initial",
  "subject": {
    "customerId": "uuid",
    "role": "primary"
  },
  "suspiciousActivity": {
    "activityType": ["structuring"],
    "activityStartDate": "2025-11-01",
    "activityEndDate": "2025-12-24",
    "narrative": "Customer made multiple deposits..."
  }
}

Response 201 Created:
{
  "sarId": "uuid",
  "status": "draft",
  "createdAt": "2025-12-25T10:00:00Z"
}
```

**Submit SAR**
```
POST /api/v1/sar/{sarId}/submit
Authorization: Bearer {token}

Response 200 OK:
{
  "sarId": "uuid",
  "filingId": "SAR-2025-001234",
  "status": "filed",
  "filedAt": "2025-12-25T10:30:00Z"
}
```

---

## 4. Webhook Events

### 4.1 Webhook Registration

```
POST /api/v1/webhooks
Authorization: Bearer {token}

{
  "url": "https://your-app.com/webhooks/kyc",
  "events": [
    "customer.created",
    "cdd.completed",
    "alert.generated"
  ],
  "secret": "your_webhook_secret"
}
```

### 4.2 Event Types

**Customer Events:**
- `customer.created`
- `customer.updated`
- `customer.risk_rating_changed`

**CDD Events:**
- `cdd.initiated`
- `cdd.completed`
- `cdd.failed`

**Transaction Events:**
- `transaction.flagged`
- `alert.generated`
- `alert.resolved`

**SAR Events:**
- `sar.created`
- `sar.filed`

### 4.3 Webhook Payload

```json
{
  "eventId": "uuid",
  "eventType": "alert.generated",
  "timestamp": "2025-12-25T10:00:00Z",
  "data": {
    "alertId": "uuid",
    "customerId": "uuid",
    "alertType": "structuring",
    "severity": "high"
  },
  "signature": "sha256_hmac_signature"
}
```

---

## 5. Error Handling

### 5.1 Error Response Format

```json
{
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "Invalid date format",
    "field": "dateOfBirth",
    "details": "Date must be in ISO8601 format",
    "requestId": "uuid"
  }
}
```

### 5.2 Standard Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| UNAUTHORIZED | 401 | Invalid or missing authentication |
| FORBIDDEN | 403 | Insufficient permissions |
| NOT_FOUND | 404 | Resource not found |
| VALIDATION_ERROR | 400 | Invalid request data |
| RATE_LIMIT_EXCEEDED | 429 | Too many requests |
| INTERNAL_ERROR | 500 | Server error |
| SERVICE_UNAVAILABLE | 503 | Temporary unavailability |

---

## 6. Rate Limiting

### 6.1 Rate Limit Headers

```
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 999
X-RateLimit-Reset: 1735689600
```

### 6.2 Default Limits

- Standard tier: 1000 requests/hour
- Premium tier: 10000 requests/hour
- Enterprise tier: Custom limits

---

## 7. Pagination

```
GET /api/v1/customers?page=1&limit=50

Response:
{
  "data": [...],
  "pagination": {
    "page": 1,
    "limit": 50,
    "totalPages": 10,
    "totalResults": 500
  },
  "links": {
    "first": "/api/v1/customers?page=1&limit=50",
    "next": "/api/v1/customers?page=2&limit=50",
    "last": "/api/v1/customers?page=10&limit=50"
  }
}
```

---

## 8. Bulk Operations

**Bulk Customer Import**
```
POST /api/v1/bulk/customers
Authorization: Bearer {token}
Content-Type: application/json

{
  "customers": [
    { "personalInfo": { ... } },
    { "personalInfo": { ... } }
  ]
}

Response 202 Accepted:
{
  "jobId": "uuid",
  "status": "processing",
  "totalRecords": 100,
  "estimatedCompletion": "2025-12-25T11:00:00Z"
}
```

**Check Bulk Job Status**
```
GET /api/v1/bulk/jobs/{jobId}

Response 200 OK:
{
  "jobId": "uuid",
  "status": "completed",
  "totalRecords": 100,
  "successCount": 98,
  "errorCount": 2,
  "errors": [
    {
      "recordIndex": 15,
      "error": "Invalid date format"
    }
  ]
}
```

---

## 9. API Versioning

- Version included in URL: `/api/v1/`
- Backward compatibility for one major version
- Deprecation notice: 6 months minimum
- Migration guides provided

---

## 10. Security Best Practices

1. **Always use HTTPS** (TLS 1.2+)
2. **Rotate API credentials** regularly
3. **Implement request signing** for sensitive operations
4. **Validate webhook signatures**
5. **Use IP whitelisting** where possible
6. **Log all API access**
7. **Monitor for anomalous patterns**

---

## Appendix: OpenAPI Specification

Full OpenAPI 3.0 specification available at:
`https://api.kyc-aml.example.com/openapi.yaml`

---

**Document Control**  
Classification: Public  
Distribution: Unrestricted  
© 2025 WIA (World Certification Industry Association)
