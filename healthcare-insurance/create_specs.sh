#!/bin/bash

# Create PHASE-2-API.md
cat > /home/user/wia-standards/healthcare-insurance/spec/PHASE-2-API.md << 'SPEC2'
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
SPEC2

# Create PHASE-3-PROTOCOL.md
cat > /home/user/wia-standards/healthcare-insurance/spec/PHASE-3-PROTOCOL.md << 'SPEC3'
# WIA-SOC-019 PHASE 3: Protocol Specification

## Healthcare Insurance Standard - Communication Protocols

**Version:** 1.0  
**Status:** PUBLISHED  
**Last Updated:** 2025-12-26

---

## 1. Protocol Overview

This document defines communication protocols, data exchange standards, and integration patterns for healthcare insurance systems implementing WIA-SOC-019.

### 1.1 Supported Protocols

- **HTTP/HTTPS**: RESTful APIs and web services
- **EDI X12**: Electronic Data Interchange for healthcare transactions
- **HL7 FHIR**: Fast Healthcare Interoperability Resources
- **SOAP/XML**: Legacy system integration
- **WebSocket**: Real-time bidirectional communication
- **SFTP**: Secure file transfer for batch operations

---

## 2. EDI X12 Transaction Sets

### 2.1 Eligibility Verification (270/271)

**270 Request:**
```
ISA*00*          *00*          *ZZ*SENDER         *ZZ*RECEIVER       *251226*1030*U*00401*000000001*0*P*:~
GS*HS*SENDER*RECEIVER*20251226*1030*1*X*004010X092~
ST*270*0001~
BHT*0022*13*10001234*20251226*1030~
HL*1**20*1~
NM1*PR*2*INSURANCE CO*****PI*12345~
HL*2*1*21*1~
NM1*1P*1*PROVIDER*JOHN****XX*1234567890~
HL*3*2*22*0~
NM1*IL*1*DOE*JOHN****MI*MEM2025001234~
DMG*D8*19850615*M~
DTP*291*D8*20251226~
EQ*30~
SE*12*0001~
GE*1*1~
IEA*1*000000001~
```

**271 Response:**
```
ISA*00*          *00*          *ZZ*RECEIVER       *ZZ*SENDER         *251226*1035*U*00401*000000001*0*P*:~
GS*HB*RECEIVER*SENDER*20251226*1035*1*X*004010X092~
ST*271*0001~
BHT*0022*11*10001234*20251226*1035~
HL*1**20*1~
NM1*PR*2*INSURANCE CO*****PI*12345~
HL*2*1*21*1~
NM1*1P*1*PROVIDER*JOHN****XX*1234567890~
HL*3*2*22*0~
NM1*IL*1*DOE*JOHN****MI*MEM2025001234~
N3*123 MAIN ST~
N4*SAN FRANCISCO*CA*94102~
DMG*D8*19850615*M~
INS*Y*18*025*28*A***FT~
DTP*291*D8*20251226~
EB*1*FAM*30**GOLD PLAN~
EB*C*IND*30*MA*COPAY*****30~
EB*G*FAM*30*MA*COINS****20~
MSG*MEMBER ACTIVE - FULL COVERAGE~
SE*18*0001~
GE*1*1~
IEA*1*000000001~
```

### 2.2 Professional Claims (837-P)

### 2.3 Institutional Claims (837-I)

### 2.4 Claim Payment/Remittance (835)

### 2.5 Claim Status (276/277)

### 2.6 Prior Authorization (278)

---

## 3. HL7 FHIR Resources

### 3.1 Patient Resource

```json
{
  "resourceType": "Patient",
  "id": "mem-2025-001234",
  "identifier": [
    {
      "system": "https://insurance.wia.org/member-id",
      "value": "mem-2025-001234"
    }
  ],
  "name": [
    {
      "use": "official",
      "family": "Doe",
      "given": ["John"]
    }
  ],
  "gender": "male",
  "birthDate": "1985-06-15",
  "address": [
    {
      "use": "home",
      "line": ["123 Main St"],
      "city": "San Francisco",
      "state": "CA",
      "postalCode": "94102",
      "country": "USA"
    }
  ],
  "telecom": [
    {
      "system": "phone",
      "value": "+1-555-0123"
    },
    {
      "system": "email",
      "value": "john.doe@example.com"
    }
  ]
}
```

### 3.2 Coverage Resource

```json
{
  "resourceType": "Coverage",
  "id": "cov-2025-789012",
  "status": "active",
  "subscriber": {
    "reference": "Patient/mem-2025-001234"
  },
  "beneficiary": {
    "reference": "Patient/mem-2025-001234"
  },
  "relationship": {
    "coding": [
      {
        "system": "http://terminology.hl7.org/CodeSystem/subscriber-relationship",
        "code": "self"
      }
    ]
  },
  "period": {
    "start": "2025-01-01"
  },
  "payor": [
    {
      "reference": "Organization/ins-company-001"
    }
  ],
  "class": [
    {
      "type": {
        "coding": [
          {
            "system": "http://terminology.hl7.org/CodeSystem/coverage-class",
            "code": "plan"
          }
        ]
      },
      "value": "GOLD"
    }
  ]
}
```

### 3.3 Claim Resource

```json
{
  "resourceType": "Claim",
  "id": "clm-2025-123456",
  "status": "active",
  "type": {
    "coding": [
      {
        "system": "http://terminology.hl7.org/CodeSystem/claim-type",
        "code": "professional"
      }
    ]
  },
  "use": "claim",
  "patient": {
    "reference": "Patient/mem-2025-001234"
  },
  "created": "2025-12-26T10:30:00Z",
  "provider": {
    "reference": "Practitioner/prv-2025-567890"
  },
  "priority": {
    "coding": [
      {
        "code": "normal"
      }
    ]
  },
  "insurance": [
    {
      "sequence": 1,
      "focal": true,
      "coverage": {
        "reference": "Coverage/cov-2025-789012"
      }
    }
  ],
  "diagnosis": [
    {
      "sequence": 1,
      "diagnosisCodeableConcept": {
        "coding": [
          {
            "system": "http://hl7.org/fhir/sid/icd-10",
            "code": "J44.0"
          }
        ]
      }
    }
  ],
  "item": [
    {
      "sequence": 1,
      "productOrService": {
        "coding": [
          {
            "system": "http://www.ama-assn.org/go/cpt",
            "code": "99214"
          }
        ]
      },
      "servicedDate": "2025-12-20",
      "quantity": {
        "value": 1
      },
      "unitPrice": {
        "value": 250.00,
        "currency": "USD"
      },
      "net": {
        "value": 250.00,
        "currency": "USD"
      }
    }
  ],
  "total": {
    "value": 250.00,
    "currency": "USD"
  }
}
```

---

## 4. Security Protocols

### 4.1 Transport Layer Security (TLS)

- Minimum version: TLS 1.3
- Cipher suites: AES-GCM only
- Certificate validation: X.509 v3 required
- Perfect Forward Secrecy: Mandatory

### 4.2 Authentication

**OAuth 2.0 Flow:**
1. Client requests token from authorization server
2. Authorization server validates credentials
3. Token issued with limited scope and expiration
4. Client includes token in API requests
5. Resource server validates token

**JWT Token Structure:**
```json
{
  "header": {
    "alg": "RS256",
    "typ": "JWT"
  },
  "payload": {
    "iss": "https://auth.wia.org",
    "sub": "client-id-12345",
    "aud": "https://api.healthcare-insurance.wia.org",
    "exp": 1735221000,
    "iat": 1735217400,
    "scope": "enrollment claims eligibility"
  }
}
```

### 4.3 Data Encryption

- At rest: AES-256-GCM
- In transit: TLS 1.3
- Key management: HSM or AWS KMS/Azure Key Vault
- Key rotation: Every 90 days

---

## 5. Message Exchange Patterns

### 5.1 Request-Response (Synchronous)

```
Client                    Server
  |                         |
  |-------- Request ------->|
  |                         |
  |<------- Response -------|
  |                         |
```

Use cases:
- Eligibility verification
- Premium calculation
- Provider lookup
- Member information retrieval

### 5.2 Fire-and-Forget (Asynchronous)

```
Client                    Server
  |                         |
  |-------- Request ------->|
  |<------ Accepted --------|
  |                         |
  |                    [Processing]
  |                         |
  |<----- Notification -----|
  |                         |
```

Use cases:
- Claim submission
- Batch enrollment
- Document upload
- Report generation

### 5.3 Publish-Subscribe

```
Publisher              Broker              Subscriber
    |                    |                     |
    |---- Publish ------>|                     |
    |                    |---- Deliver ------->|
    |                    |                     |
```

Use cases:
- Claim status updates
- Coverage changes
- Provider network updates
- System alerts

---

## 6. Batch Processing Protocols

### 6.1 File Transfer

**SFTP Configuration:**
- Host: sftp.wia.org
- Port: 22
- Authentication: SSH key pairs
- Directory structure:
  - `/inbound` - Files from partners
  - `/outbound` - Files to partners
  - `/archive` - Processed files

**File Naming Convention:**
```
{sender}_{receiver}_{transaction-type}_{date}_{sequence}.{format}
Example: INS001_PRV001_CLAIMS_20251226_001.x12
```

### 6.2 Batch Claim Files

Format: ANSI X12 837
Envelope: ISA/IEA, GS/GE, ST/SE
Transmission: Daily at 2:00 AM UTC
Acknowledgment: Within 4 hours

---

## 7. Real-Time Communication

### 7.1 WebSocket Protocol

```javascript
// Client connection
const ws = new WebSocket('wss://api.wia.org/v1/stream');

ws.onopen = () => {
  ws.send(JSON.stringify({
    type: 'subscribe',
    channel: 'claims',
    memberId: 'mem-2025-001234',
    token: 'your-jwt-token'
  }));
};

ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  console.log('Claim update:', data);
};
```

### 7.2 Server-Sent Events (SSE)

```javascript
const evtSource = new EventSource(
  'https://api.wia.org/v1/events?memberId=mem-2025-001234',
  {
    headers: {
      'Authorization': 'Bearer your-jwt-token'
    }
  }
);

evtSource.addEventListener('claim-status', (event) => {
  const data = JSON.parse(event.data);
  console.log('Claim status changed:', data);
});
```

---

## 8. Error Handling and Retry Logic

### 8.1 HTTP Status Code Handling

- `2xx`: Success, no retry needed
- `4xx`: Client error, fix request before retry
- `500`: Internal error, retry with exponential backoff
- `502/503/504`: Temporary issue, retry immediately
- `429`: Rate limited, wait for retry-after header

### 8.2 Retry Strategy

**Exponential Backoff:**
```
Attempt 1: Immediate
Attempt 2: Wait 1 second
Attempt 3: Wait 2 seconds
Attempt 4: Wait 4 seconds
Attempt 5: Wait 8 seconds
Max attempts: 5
```

**Jitter:**
Add random delay to prevent thundering herd:
```
actual_wait = base_wait + random(0, jitter_range)
```

---

## 9. Monitoring and Logging

### 9.1 Request Tracing

Every request includes:
- `X-Request-ID`: Unique request identifier
- `X-Correlation-ID`: End-to-end transaction tracking
- Timestamps: Request/response times
- Client info: User agent, IP address

### 9.2 Audit Logging

Log all:
- Authentication attempts
- Data access (who, what, when)
- Data modifications
- Configuration changes
- Error conditions

---

## 10. Compliance and Standards

### 10.1 Regulatory Requirements

- **HIPAA**: Privacy and Security Rules
- **GDPR**: Data protection (for EU members)
- **PCI DSS**: Payment card processing
- **SOC 2 Type II**: Security controls

### 10.2 Industry Standards

- **NIST Cybersecurity Framework**
- **ISO 27001**: Information security
- **HL7 Version 2.x, 3.0, FHIR R4**
- **DICOM**: Medical imaging

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
SPEC3

# Create PHASE-4-INTEGRATION.md
cat > /home/user/wia-standards/healthcare-insurance/spec/PHASE-4-INTEGRATION.md << 'SPEC4'
# WIA-SOC-019 PHASE 4: Integration Specification

## Healthcare Insurance Standard - System Integration Patterns

**Version:** 1.0  
**Status:** PUBLISHED  
**Last Updated:** 2025-12-26

---

## 1. Integration Architecture

### 1.1 Integration Patterns

WIA-SOC-019 supports multiple integration patterns:

- **Point-to-Point**: Direct system connections
- **Hub-and-Spoke**: Central integration hub
- **Enterprise Service Bus (ESB)**: Message-oriented middleware
- **API Gateway**: Centralized API management
- **Event-Driven Architecture**: Async event streaming
- **Microservices**: Distributed service mesh

### 1.2 Reference Architecture

```
┌─────────────────┐
│   Mobile App    │
└────────┬────────┘
         │
┌────────▼────────────────────────────┐
│         API Gateway                  │
│  (Auth, Rate Limit, Transform)      │
└────────┬────────────────────────────┘
         │
    ┌────┴─────┬──────────┬──────────┐
    │          │          │          │
┌───▼───┐  ┌──▼───┐  ┌──▼───┐  ┌──▼───┐
│Enroll │  │Claims│  │Elig  │  │Prem  │
│ment   │  │      │  │ility │  │ium   │
│Service│  │Svc   │  │Svc   │  │Svc   │
└───┬───┘  └──┬───┘  └──┬───┘  └──┬───┘
    │         │         │         │
    └─────────┴─────────┴─────────┘
              │
    ┌─────────▼──────────┐
    │   Data Layer       │
    │ (Postgres, Redis)  │
    └────────────────────┘
```

---

## 2. EHR Integration

### 2.1 HL7 FHIR Integration

**Coverage Lookup Flow:**
```
EHR System → FHIR Server → Insurance API → FHIR Response → EHR System
```

**Example Request:**
```http
GET /Coverage?patient=Patient/123&status=active
Accept: application/fhir+json
Authorization: Bearer {token}
```

**Response:**
```json
{
  "resourceType": "Bundle",
  "type": "searchset",
  "entry": [
    {
      "resource": {
        "resourceType": "Coverage",
        "id": "cov-001",
        "status": "active",
        "beneficiary": {
          "reference": "Patient/123"
        }
      }
    }
  ]
}
```

### 2.2 CDA Document Exchange

Clinical Document Architecture (CDA) integration for:
- Continuity of Care Document (CCD)
- Discharge Summary
- Progress Notes
- Diagnostic Reports

---

## 3. Claims Clearinghouse Integration

### 3.1 EDI X12 Workflow

```
Provider → PM System → Clearinghouse → Payer
   │                                      │
   └─────── Acknowledgment 997 ──────────┘
   └─────── Status 277 ───────────────────┘
   └─────── Remittance 835 ───────────────┘
```

### 3.2 Batch File Processing

**Daily Batch Schedule:**
- 02:00 UTC: Receive claims batch (837)
- 04:00 UTC: Send acknowledgments (997)
- 06:00 UTC: Adjudicate claims
- 08:00 UTC: Send remittances (835)
- 10:00 UTC: Update member accounts

---

## 4. Payment Gateway Integration

### 4.1 Premium Payment Flow

```
Member → Payment Portal → Gateway → Processor → Bank
   │                                               │
   └───────── Confirmation ────────────────────────┘
```

**Supported Payment Methods:**
- Credit/Debit Cards (Stripe, Square)
- ACH (Automated Clearing House)
- Wire Transfer
- Digital Wallets (PayPal, Apple Pay, Google Pay)

### 4.2 Provider Payment Processing

**Claim Payment Workflow:**
1. Claim adjudicated and approved
2. Payment calculated
3. Payment batch created
4. Submit to ACH network
5. Provider receives funds (2-3 business days)
6. Send remittance advice (835)

---

## 5. Third-Party Services

### 5.1 Identity Verification

**Integration with ID.me, Experian:**
```http
POST /verify/identity
{
  "firstName": "John",
  "lastName": "Doe",
  "dateOfBirth": "1985-06-15",
  "ssn": "***-**-1234",
  "address": {...}
}
```

Response:
```json
{
  "verified": true,
  "confidence": 0.95,
  "verificationId": "ver-abc123"
}
```

### 5.2 Prescription Benefit Manager (PBM)

**RxClaim Integration:**
- NCPDP D.0 format
- Real-time adjudication
- Formulary checking
- Prior authorization

### 5.3 Lab Results Integration

**LOINC-coded Results:**
```json
{
  "resourceType": "Observation",
  "code": {
    "coding": [
      {
        "system": "http://loinc.org",
        "code": "2339-0",
        "display": "Glucose"
      }
    ]
  },
  "valueQuantity": {
    "value": 95,
    "unit": "mg/dL"
  }
}
```

---

## 6. Government Systems Integration

### 6.1 Medicare Integration

**Claims Submission:**
- Format: ANSI X12 837-I/837-P
- Destination: Medicare Administrative Contractor (MAC)
- Frequency: Daily batches

### 6.2 Medicaid Management Information System (MMIS)

**Eligibility Verification:**
- State-specific EDI formats
- Real-time or batch processing
- Coordination of benefits (COB)

### 6.3 Health Insurance Exchange

**Enrollment Data:**
- FFM (Federally Facilitated Marketplace)
- State-based Exchanges
- 834 Enrollment/Maintenance transactions

---

## 7. Analytics and Reporting

### 7.1 Data Warehouse Integration

**ETL Pipeline:**
```
Operational DB → ETL Process → Data Warehouse → BI Tools
      │                                            │
      └─────── Real-time CDC ────────────────────┘
```

**Technology Stack:**
- Fivetran/Stitch for ETL
- Snowflake/BigQuery for warehousing
- Tableau/Looker for visualization

### 7.2 Predictive Analytics

**ML Model Integration:**
```python
# Fraud detection model
POST /ml/fraud-detection
{
  "claimId": "clm-2025-123456",
  "features": {
    "provider_history": [...],
    "claim_patterns": [...],
    "member_history": [...]
  }
}

Response:
{
  "fraud_probability": 0.85,
  "risk_score": "HIGH",
  "factors": [
    "Unusual billing pattern",
    "Provider flagged previously"
  ]
}
```

---

## 8. Mobile Application Integration

### 8.1 Mobile SDKs

**iOS SDK:**
```swift
import WIAHealthInsurance

let client = InsuranceClient(apiKey: "your-api-key")

client.getEligibility(memberId: "mem-2025-001234") { result in
    switch result {
    case .success(let eligibility):
        print("Coverage: \\(eligibility.status)")
    case .failure(let error):
        print("Error: \\(error)")
    }
}
```

**Android SDK:**
```kotlin
val client = InsuranceClient("your-api-key")

client.getEligibility("mem-2025-001234") { result ->
    when (result) {
        is Success -> println("Coverage: ${result.data.status}")
        is Error -> println("Error: ${result.error}")
    }
}
```

### 8.2 Push Notifications

**FCM/APNS Integration:**
```json
{
  "to": "device-token-xyz",
  "notification": {
    "title": "Claim Approved",
    "body": "Your claim CLM-2025-0001 has been approved",
    "click_action": "OPEN_CLAIM_DETAIL"
  },
  "data": {
    "claimId": "clm-2025-123456",
    "status": "APPROVED"
  }
}
```

---

## 9. Security Integration

### 9.1 Single Sign-On (SSO)

**SAML 2.0 Flow:**
```
User → Service Provider → Identity Provider → Assertion → SP
```

**OIDC (OpenID Connect):**
```http
GET /authorize?
  client_id=your-client-id&
  redirect_uri=https://your-app.com/callback&
  response_type=code&
  scope=openid profile email
```

### 9.2 API Gateway Security

**Kong/AWS API Gateway Configuration:**
- Authentication (OAuth 2.0, JWT)
- Rate limiting (per client, per IP)
- Request/response transformation
- Logging and monitoring
- DDoS protection

---

## 10. Disaster Recovery and Business Continuity

### 10.1 High Availability Architecture

```
┌─────────┐     ┌─────────┐
│ Region  │────▶│ Region  │
│ Primary │     │ Standby │
└─────────┘     └─────────┘
     │               │
     └───── Replication ────┘
```

**RTO/RPO Targets:**
- Recovery Time Objective (RTO): 4 hours
- Recovery Point Objective (RPO): 15 minutes
- Data replication: Synchronous (primary region), Asynchronous (DR region)

### 10.2 Backup Strategy

**Automated Backups:**
- Database: Continuous backup with point-in-time recovery
- Files: Daily incremental, weekly full
- Retention: 90 days online, 7 years archived
- Testing: Monthly DR drills

---

## 11. Migration Strategies

### 11.1 Legacy System Migration

**Phased Approach:**
1. **Phase 1 - Dual Run**: New system alongside legacy (3 months)
2. **Phase 2 - Pilot**: Migrate 10% of members (2 months)
3. **Phase 3 - Gradual Rollout**: 25%, 50%, 75% (6 months)
4. **Phase 4 - Full Migration**: 100% cutover
5. **Phase 5 - Legacy Decommission**: Archive and shutdown

### 11.2 Data Migration

**ETL Process:**
```sql
-- Extract from legacy system
SELECT member_id, first_name, last_name, dob
FROM legacy.members
WHERE migration_flag = 0;

-- Transform to new schema
INSERT INTO wia.members (member_id, personal_info, ...)
VALUES (...);

-- Validate data quality
SELECT COUNT(*) as issues
FROM wia.members m
LEFT JOIN legacy.members l ON m.legacy_id = l.member_id
WHERE m.dob != l.dob;
```

---

## 12. Testing and Validation

### 12.1 Integration Testing

**Test Scenarios:**
- End-to-end claim submission
- Eligibility verification
- Member enrollment
- Payment processing
- Cross-border coverage

**Tools:**
- Postman/Newman for API testing
- SoapUI for SOAP/XML services
- JMeter for load testing
- Selenium for UI testing

### 12.2 Certification Testing

**Required Certifications:**
- HL7 FHIR conformance testing
- EDI X12 validation (WEDI SNIP)
- HIPAA compliance testing
- PCI DSS for payment processing

---

## 13. Monitoring and Observability

### 13.1 Application Performance Monitoring (APM)

**Tools:** Datadog, New Relic, Dynatrace

**Key Metrics:**
- API response time (p50, p95, p99)
- Error rate
- Throughput (requests/second)
- Database query performance
- External service latency

### 13.2 Logging and Tracing

**Distributed Tracing:**
```
Request ID: req-abc123
Correlation ID: cor-xyz789

┌─────────────────────────────────────┐
│ API Gateway (10ms)                  │
│  └─> Auth Service (5ms)             │
│  └─> Enrollment Service (50ms)      │
│      └─> Database Query (30ms)      │
│      └─> Cache Lookup (2ms)         │
└─────────────────────────────────────┘
Total: 97ms
```

**Log Aggregation:** ELK Stack (Elasticsearch, Logstash, Kibana)

---

## 14. Compliance and Governance

### 14.1 Regulatory Reporting

**Automated Report Generation:**
- HIPAA compliance reports
- Financial solvency reports
- Quality measure submissions (HEDIS)
- Government program reporting (CMS)

### 14.2 Data Governance

**Policies:**
- Data classification (public, internal, confidential, restricted)
- Access controls (RBAC, ABAC)
- Data retention schedules
- Data quality rules
- Privacy by design

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
SPEC4

echo "All specification files created successfully!"
