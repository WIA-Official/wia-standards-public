# WIA-SOC-019 PHASE 2: API Specification

## Healthcare Insurance Standard - REST API and Integration

**Version:** 1.0  
**Status:** PUBLISHED  
**Last Updated:** 2025-12-26

---

## 1. API Overview

The WIA-SOC-019 API provides RESTful endpoints for healthcare insurance operations including enrollment, claims processing, eligibility verification, provider lookups, and premium calculations.

### 1.1 Base URL

Production: `https://api.healthcare-insurance.wia.org/v1`
Sandbox: `https://sandbox-api.healthcare-insurance.wia.org/v1`

### 1.2 Authentication

OAuth 2.0 with JWT tokens:
```http
POST /auth/token
Content-Type: application/json

{
  "client_id": "your_client_id",
  "client_secret": "your_client_secret",
  "grant_type": "client_credentials",
  "scope": "enrollment claims eligibility"
}
```

Response:
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIs...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "enrollment claims eligibility"
}
```

### 1.3 Request Headers

```http
Authorization: Bearer {access_token}
Content-Type: application/json
Accept: application/json
X-Request-ID: {unique-request-id}
X-API-Version: 1.0
```

---

## 2. Enrollment APIs

### 2.1 Create Member Enrollment

```http
POST /enrollments
Content-Type: application/json

{
  "personalInfo": {
    "firstName": "John",
    "lastName": "Doe",
    "dateOfBirth": "1985-06-15",
    "gender": "MALE"
  },
  "contact": {
    "email": "john.doe@example.com",
    "phone": "+1-555-0123",
    "address": {
      "street": "123 Main St",
      "city": "San Francisco",
      "state": "CA",
      "postalCode": "94102",
      "country": "USA"
    }
  },
  "coverageType": "FAMILY",
  "planId": "plan-gold-2025",
  "effectiveDate": "2025-01-01"
}
```

Response: `201 Created`
```json
{
  "memberId": "mem-2025-001234",
  "enrollmentId": "enr-2025-789012",
  "status": "ACTIVE",
  "effectiveDate": "2025-01-01",
  "policyNumber": "POL-2025-567890"
}
```

### 2.2 Get Member Details

```http
GET /members/{memberId}
```

Response: `200 OK`
```json
{
  "memberId": "mem-2025-001234",
  "personalInfo": {...},
  "coverage": {...},
  "dependents": [...],
  "status": "ACTIVE"
}
```

### 2.3 Update Member Information

```http
PATCH /members/{memberId}
Content-Type: application/json

{
  "contact": {
    "phone": "+1-555-9999"
  }
}
```

### 2.4 Add Dependent

```http
POST /members/{memberId}/dependents

{
  "firstName": "Jane",
  "lastName": "Doe",
  "dateOfBirth": "2015-03-20",
  "relationship": "CHILD"
}
```

---

## 3. Claims APIs

### 3.1 Submit Claim

```http
POST /claims
Content-Type: application/json

{
  "memberId": "mem-2025-001234",
  "providerId": "prv-2025-567890",
  "serviceDate": "2025-12-20",
  "diagnosis": [
    {
      "code": "J44.0",
      "type": "PRIMARY",
      "description": "COPD with acute lower respiratory infection"
    }
  ],
  "procedures": [
    {
      "code": "99214",
      "description": "Office visit, established patient",
      "quantity": 1,
      "chargedAmount": 250.00
    }
  ],
  "totalCharged": 250.00,
  "currency": "USD"
}
```

Response: `201 Created`
```json
{
  "claimId": "clm-2025-123456",
  "claimNumber": "CLM-2025-0001",
  "status": "SUBMITTED",
  "submittedDate": "2025-12-26T10:30:00Z",
  "estimatedProcessingDays": 3
}
```

### 3.2 Get Claim Status

```http
GET /claims/{claimId}
```

Response: `200 OK`
```json
{
  "claimId": "clm-2025-123456",
  "status": "APPROVED",
  "adjudicatedDate": "2025-12-27T14:22:00Z",
  "approvedAmount": 200.00,
  "patientResponsibility": {
    "copay": 30.00,
    "coinsurance": 20.00,
    "total": 50.00
  },
  "insurancePayment": 150.00
}
```

### 3.3 Search Claims

```http
GET /claims?memberId=mem-2025-001234&status=APPROVED&fromDate=2025-01-01&toDate=2025-12-31
```

### 3.4 Appeal Claim

```http
POST /claims/{claimId}/appeals

{
  "reason": "Medical necessity supported by attached documentation",
  "supportingDocs": ["doc-id-1", "doc-id-2"]
}
```

---

## 4. Eligibility APIs

### 4.1 Verify Eligibility

```http
POST /eligibility/verify
Content-Type: application/json

{
  "memberId": "mem-2025-001234",
  "providerId": "prv-2025-567890",
  "serviceType": "SPECIALIST",
  "serviceDate": "2025-12-28"
}
```

Response: `200 OK`
```json
{
  "eligible": true,
  "coverageStatus": "ACTIVE",
  "copay": 30.00,
  "coinsurance": 0.20,
  "deductible": {
    "annual": 1000.00,
    "met": 650.00,
    "remaining": 350.00
  },
  "outOfPocketMax": {
    "annual": 5000.00,
    "met": 1240.00,
    "remaining": 3760.00
  },
  "preAuthRequired": false
}
```

### 4.2 Check Benefits

```http
GET /members/{memberId}/benefits?serviceType=SURGERY
```

Response:
```json
{
  "serviceType": "SURGERY",
  "covered": true,
  "coveragePercentage": 0.80,
  "requiresPriorAuth": true,
  "networkStatus": "IN_NETWORK",
  "annualLimit": null,
  "lifetimeLimit": null
}
```

---

## 5. Provider APIs

### 5.1 Search Providers

```http
GET /providers/search?specialty=CARDIOLOGY&city=San Francisco&inNetwork=true
```

Response:
```json
{
  "results": [
    {
      "providerId": "prv-2025-567890",
      "name": "Dr. Sarah Johnson",
      "specialty": "CARDIOLOGY",
      "address": {...},
      "phone": "+1-555-1234",
      "networkStatus": "IN_NETWORK",
      "acceptingNewPatients": true,
      "qualityRating": 4.8
    }
  ],
  "total": 45,
  "page": 1,
  "pageSize": 20
}
```

### 5.2 Get Provider Details

```http
GET /providers/{providerId}
```

### 5.3 Check Provider Network Status

```http
GET /providers/{providerId}/network-status?planId=plan-gold-2025
```

---

## 6. Premium APIs

### 6.1 Calculate Premium

```http
POST /premiums/calculate

{
  "age": 35,
  "gender": "MALE",
  "zipCode": "94102",
  "planId": "plan-gold-2025",
  "dependents": 2,
  "smoker": false
}
```

Response:
```json
{
  "monthlyPremium": 450.00,
  "annualPremium": 5400.00,
  "breakdown": {
    "basePremium": 300.00,
    "dependentCost": 150.00,
    "geographicAdjustment": 0.00,
    "subsidyEligible": true,
    "estimatedSubsidy": 100.00,
    "netPremium": 350.00
  }
}
```

### 6.2 Get Payment History

```http
GET /members/{memberId}/payments?fromDate=2025-01-01&toDate=2025-12-31
```

---

## 7. Cross-Border APIs

### 7.1 Check International Coverage

```http
POST /cross-border/coverage-check

{
  "memberId": "mem-2025-001234",
  "destinationCountry": "TH",
  "serviceType": "SURGERY",
  "estimatedCost": 15000.00
}
```

Response:
```json
{
  "covered": true,
  "coveragePercentage": 0.70,
  "estimatedPayment": 10500.00,
  "patientResponsibility": 4500.00,
  "requiresPreAuth": true,
  "networkProviders": [
    {
      "providerId": "intl-prv-thailand-001",
      "name": "Bangkok International Hospital",
      "city": "Bangkok"
    }
  ]
}
```

---

## 8. Error Handling

### 8.1 Error Response Format

```json
{
  "error": {
    "code": "INVALID_REQUEST",
    "message": "Member ID is required",
    "details": {
      "field": "memberId",
      "reason": "missing_required_field"
    },
    "requestId": "req-abc123"
  }
}
```

### 8.2 HTTP Status Codes

- `200 OK` - Successful request
- `201 Created` - Resource created
- `400 Bad Request` - Invalid request data
- `401 Unauthorized` - Authentication required
- `403 Forbidden` - Insufficient permissions
- `404 Not Found` - Resource not found
- `422 Unprocessable Entity` - Validation error
- `429 Too Many Requests` - Rate limit exceeded
- `500 Internal Server Error` - Server error
- `503 Service Unavailable` - Temporary outage

---

## 9. Rate Limiting

- Standard tier: 1000 requests/hour
- Premium tier: 10000 requests/hour
- Headers returned:
  - `X-RateLimit-Limit`
  - `X-RateLimit-Remaining`
  - `X-RateLimit-Reset`

---

## 10. Webhooks

Subscribe to real-time events:

```http
POST /webhooks

{
  "url": "https://your-app.com/webhook",
  "events": ["claim.approved", "claim.denied", "enrollment.completed"],
  "secret": "your-webhook-secret"
}
```

Webhook payload:
```json
{
  "event": "claim.approved",
  "timestamp": "2025-12-27T14:22:00Z",
  "data": {
    "claimId": "clm-2025-123456",
    "memberId": "mem-2025-001234"
  },
  "signature": "sha256=..."
}
```

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
