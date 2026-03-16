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
