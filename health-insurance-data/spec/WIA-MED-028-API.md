# WIA-MED-028 API Specification

**Version:** 1.0.0
**Base URL:** `https://api.wia-health.org/v1`
**Protocol:** REST + EDI X12
**Authentication:** OAuth 2.0 + Client Credentials

---

## Authentication

### OAuth 2.0 Token Endpoint

```http
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials
&client_id=YOUR_CLIENT_ID
&client_secret=YOUR_CLIENT_SECRET
&scope=claims eligibility prior_auth
```

**Response:**
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIs...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "claims eligibility prior_auth"
}
```

### Using Access Token

```http
Authorization: Bearer eyJhbGciOiJSUzI1NiIs...
```

---

## API Endpoints

### 1. Eligibility Verification (270/271)

#### Check Eligibility

```http
POST /eligibility/check
Content-Type: application/json
Authorization: Bearer {token}

{
  "subscriber": {
    "memberId": "ABC123456",
    "firstName": "John",
    "lastName": "Smith",
    "dateOfBirth": "1980-05-15"
  },
  "provider": {
    "npi": "1234567890",
    "taxId": "12-3456789"
  },
  "serviceDate": "2025-01-26",
  "serviceType": "30"
}
```

**Response (200 OK):**
```json
{
  "transactionId": "ELG-20250126-001",
  "eligibilityStatus": "active",
  "subscriber": {
    "memberId": "ABC123456",
    "name": "John Smith",
    "dateOfBirth": "1980-05-15"
  },
  "coverage": {
    "planName": "Blue Cross PPO",
    "effectiveDate": "2025-01-01",
    "terminationDate": "2025-12-31",
    "inNetwork": true
  },
  "benefits": {
    "copay": {
      "office": 25.00,
      "specialist": 50.00,
      "emergency": 150.00
    },
    "deductible": {
      "individual": 1500.00,
      "family": 3000.00,
      "remaining": 1440.00
    },
    "coinsurance": {
      "inNetwork": "80-20",
      "outOfNetwork": "60-40"
    },
    "outOfPocketMax": {
      "individual": 5000.00,
      "family": 10000.00,
      "remaining": 4898.00
    }
  },
  "priorAuthRequired": {
    "mri": true,
    "ct": true,
    "surgery": true
  }
}
```

---

### 2. Claim Submission (837)

#### Submit Professional Claim

```http
POST /claims/submit
Content-Type: application/json
Authorization: Bearer {token}

{
  "claimType": "professional",
  "claimId": "CLM-2025-001",
  "billingProvider": {
    "npi": "1234567890",
    "name": "City Medical Clinic",
    "taxId": "12-3456789",
    "address": {
      "street": "123 Main St",
      "city": "Anytown",
      "state": "CA",
      "zip": "90210"
    }
  },
  "subscriber": {
    "memberId": "ABC123456",
    "firstName": "John",
    "lastName": "Smith",
    "dateOfBirth": "1980-05-15",
    "relationship": "self"
  },
  "payer": {
    "payerId": "12345",
    "name": "Blue Cross Blue Shield"
  },
  "serviceLines": [
    {
      "lineNumber": 1,
      "serviceDate": "2025-01-15",
      "placeOfService": "11",
      "procedureCode": "99213",
      "diagnosisPointers": ["A"],
      "chargedAmount": 150.00,
      "units": 1
    },
    {
      "lineNumber": 2,
      "serviceDate": "2025-01-15",
      "placeOfService": "11",
      "procedureCode": "73030",
      "diagnosisPointers": ["A"],
      "chargedAmount": 80.00,
      "units": 1
    }
  ],
  "diagnoses": [
    {
      "sequence": "A",
      "code": "M25.561",
      "description": "Pain in right knee"
    }
  ],
  "totalCharged": 230.00
}
```

**Response (201 Created):**
```json
{
  "claimId": "CLM-2025-001",
  "transactionId": "837-20250126-001",
  "status": "accepted",
  "acceptedAt": "2025-01-26T14:30:00Z",
  "clearinghouse": {
    "id": "CLH-001",
    "name": "WIA Clearinghouse"
  },
  "estimatedProcessing": "7-14 days",
  "trackingUrl": "https://api.wia-health.org/v1/claims/CLM-2025-001/status"
}
```

---

### 3. Claim Status (276/277)

#### Check Claim Status

```http
GET /claims/{claimId}/status
Authorization: Bearer {token}
```

**Response (200 OK):**
```json
{
  "claimId": "CLM-2025-001",
  "status": "paid",
  "statusCategory": "finalized",
  "statusDate": "2025-01-20",
  "paidAmount": 78.00,
  "paidDate": "2025-01-20",
  "paymentMethod": "ACH",
  "checkNumber": "12345678",
  "remittanceAdvice": {
    "eraId": "ERA-20250120-001",
    "url": "https://api.wia-health.org/v1/remittance/ERA-20250120-001"
  }
}
```

---

### 4. Prior Authorization (278)

#### Request Prior Authorization

```http
POST /prior-auth/request
Content-Type: application/json
Authorization: Bearer {token}

{
  "requestType": "initial",
  "subscriber": {
    "memberId": "ABC123456",
    "firstName": "John",
    "lastName": "Smith",
    "dateOfBirth": "1980-05-15"
  },
  "provider": {
    "npi": "1234567890",
    "name": "Dr. Jane Doe"
  },
  "requestedService": {
    "procedureCode": "70553",
    "description": "MRI Brain with contrast",
    "quantity": 1,
    "dateOfService": "2025-02-15"
  },
  "diagnosis": [
    {
      "code": "G43.909",
      "description": "Migraine, unspecified"
    }
  ],
  "clinicalInformation": {
    "symptoms": "Severe headaches for 3 months",
    "previousTreatments": ["Pain medication", "Physical therapy"],
    "urgency": "routine"
  },
  "attachments": [
    {
      "type": "clinical_notes",
      "url": "https://provider.example.com/documents/note-123"
    }
  ]
}
```

**Response (200 OK):**
```json
{
  "authorizationId": "AUTH-2025-001",
  "status": "approved",
  "decision": "A1",
  "decisionDate": "2025-01-26",
  "approvedService": {
    "procedureCode": "70553",
    "quantity": 1,
    "validFrom": "2025-02-01",
    "validUntil": "2025-03-31"
  },
  "conditions": [],
  "reviewedBy": {
    "name": "Medical Director",
    "credentials": "MD"
  },
  "referenceNumber": "REF-20250126-001"
}
```

---

### 5. Payment/Remittance (835)

#### Get ERA (Electronic Remittance Advice)

```http
GET /remittance/{eraId}
Authorization: Bearer {token}
```

**Response (200 OK):**
```json
{
  "eraId": "ERA-20250120-001",
  "paymentDate": "2025-01-20",
  "paymentMethod": "ACH",
  "totalPaid": 78.00,
  "payer": {
    "name": "Blue Cross Blue Shield",
    "id": "12345"
  },
  "payee": {
    "npi": "1234567890",
    "name": "City Medical Clinic"
  },
  "claims": [
    {
      "claimId": "CLM-2025-001",
      "patientName": "John Smith",
      "totalCharged": 230.00,
      "totalAllowed": 180.00,
      "totalPaid": 78.00,
      "patientResponsibility": 102.00,
      "serviceLines": [
        {
          "lineNumber": 1,
          "procedureCode": "99213",
          "chargedAmount": 150.00,
          "allowedAmount": 120.00,
          "paidAmount": 38.00,
          "adjustments": [
            {
              "group": "CO",
              "reasonCode": "45",
              "amount": 30.00,
              "description": "Charge exceeds fee schedule"
            },
            {
              "group": "PR",
              "reasonCode": "1",
              "amount": 50.00,
              "description": "Deductible amount"
            },
            {
              "group": "PR",
              "reasonCode": "3",
              "amount": 25.00,
              "description": "Copay amount"
            }
          ]
        }
      ]
    }
  ]
}
```

---

## Data Models

### Subscriber
```typescript
interface Subscriber {
  memberId: string;
  firstName: string;
  lastName: string;
  middleName?: string;
  dateOfBirth: string; // YYYY-MM-DD
  gender?: 'M' | 'F' | 'U';
  address?: Address;
}
```

### Provider
```typescript
interface Provider {
  npi: string; // National Provider Identifier
  name: string;
  taxId?: string;
  specialty?: string;
  address?: Address;
}
```

### ServiceLine
```typescript
interface ServiceLine {
  lineNumber: number;
  serviceDate: string; // YYYY-MM-DD
  placeOfService: string; // POS Code
  procedureCode: string; // CPT/HCPCS
  modifiers?: string[];
  diagnosisPointers: string[]; // A, B, C, etc.
  chargedAmount: number;
  units: number;
}
```

### Diagnosis
```typescript
interface Diagnosis {
  sequence: string; // A-L
  code: string; // ICD-10-CM
  description?: string;
  presentOnAdmission?: 'Y' | 'N' | 'U' | 'W';
}
```

---

## Error Handling

### Error Response Format

```json
{
  "error": {
    "code": "INVALID_MEMBER_ID",
    "message": "Member ID not found in system",
    "details": "Member ID ABC123456 is not active on service date 2025-01-26",
    "transactionId": "TXN-20250126-001",
    "timestamp": "2025-01-26T14:30:00Z"
  }
}
```

### Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| `AUTHENTICATION_FAILED` | 401 | Invalid credentials |
| `AUTHORIZATION_DENIED` | 403 | Insufficient permissions |
| `INVALID_MEMBER_ID` | 400 | Member not found |
| `COVERAGE_INACTIVE` | 400 | Coverage not active |
| `INVALID_PROVIDER_NPI` | 400 | Provider NPI invalid |
| `DUPLICATE_CLAIM` | 409 | Claim already submitted |
| `VALIDATION_ERROR` | 400 | Request validation failed |
| `RATE_LIMIT_EXCEEDED` | 429 | Too many requests |
| `INTERNAL_ERROR` | 500 | Server error |

---

## Rate Limiting

- **Default**: 1000 requests/hour per client
- **Burst**: 100 requests/minute
- **Headers**:
  ```
  X-RateLimit-Limit: 1000
  X-RateLimit-Remaining: 987
  X-RateLimit-Reset: 1643298000
  ```

---

## Webhooks

### Register Webhook

```http
POST /webhooks
Content-Type: application/json
Authorization: Bearer {token}

{
  "url": "https://your-server.com/webhooks/wia",
  "events": ["claim.paid", "claim.denied", "auth.approved"],
  "secret": "your_webhook_secret"
}
```

### Webhook Payload

```json
{
  "eventId": "EVT-20250126-001",
  "eventType": "claim.paid",
  "timestamp": "2025-01-26T14:30:00Z",
  "data": {
    "claimId": "CLM-2025-001",
    "paidAmount": 78.00,
    "paidDate": "2025-01-20"
  }
}
```

---

## SDKs

Official SDKs available:
- **Node.js**: `npm install @wia-health/sdk`
- **Python**: `pip install wia-health-sdk`
- **Java**: Maven Central
- **C#**: NuGet

### Example (Node.js)

```javascript
const WIAHealth = require('@wia-health/sdk');

const client = new WIAHealth({
  clientId: 'YOUR_CLIENT_ID',
  clientSecret: 'YOUR_CLIENT_SECRET',
  environment: 'production'
});

// Check eligibility
const eligibility = await client.eligibility.check({
  memberId: 'ABC123456',
  dateOfBirth: '1980-05-15',
  serviceDate: '2025-01-26'
});

console.log(eligibility.benefits.copay.office); // 25.00
```

---

## Testing

### Sandbox Environment
- **Base URL**: `https://sandbox.wia-health.org/v1`
- **Test Credentials**: Available at https://developer.wiastandards.com

### Test Data
- **Valid Member ID**: `TEST123456`
- **Invalid Member ID**: `INVALID999`
- **Test NPI**: `9999999999`

---

**Contact**: api-support@wiastandards.com
**Documentation**: https://developer.wiastandards.com
**Status Page**: https://status.wiastandards.com

© 2025 WIA · MIT License
