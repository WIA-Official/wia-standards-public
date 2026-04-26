# WIA-PET-010 Pet Insurance - Phase 3: Protocol

**Version:** 1.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-12-25

---

## 1. Overview

Phase 3 defines the communication protocols and real-time processing systems for the WIA Pet Insurance Standard. This specification ensures reliable, secure, and efficient data exchange between all stakeholders in the pet insurance ecosystem.

### 1.1 Protocol Goals

- **Real-time Processing**: Instant claim verification and approval
- **Event-Driven Architecture**: Asynchronous event handling
- **Security**: End-to-end encryption and authentication
- **Reliability**: Guaranteed message delivery
- **Scalability**: Support for millions of concurrent users
- **Interoperability**: Cross-provider communication

### 1.2 Protocol Stack

```
Application Layer: WIA Pet Insurance Protocol (WIA-PET-010)
Transport Layer: WebSocket, HTTPS, MQTT
Security Layer: TLS 1.3, JWT, OAuth 2.0
Data Layer: JSON, Protocol Buffers
Blockchain Layer: Ethereum, Polygon, Smart Contracts
```

---

## 2. Real-Time Communication

### 2.1 WebSocket Protocol

**Connection Endpoint:**
```
wss://realtime.wiastandards.com/pet-insurance/v1/ws
```

**Connection Flow:**
```
Client → Server: WebSocket Handshake
Server → Client: Connection Established
Client → Server: Authenticate (JWT token)
Server → Client: Authentication Success
[Bidirectional messaging begins]
```

**Authentication:**
```json
{
  "type": "auth",
  "token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "clientId": "CLIENT-2025-001"
}
```

**Server Response:**
```json
{
  "type": "auth_success",
  "sessionId": "SESSION-2025-123456",
  "expiresIn": 3600,
  "timestamp": "2025-12-25T10:30:00Z"
}
```

### 2.2 Message Format

All WebSocket messages follow this structure:

```json
{
  "messageId": "MSG-2025-123456",
  "type": "message_type",
  "timestamp": "2025-12-25T10:30:00Z",
  "payload": { ... },
  "signature": "sha256:abcdef123456..."
}
```

### 2.3 Message Types

| Type | Direction | Description |
|------|-----------|-------------|
| `claim.status_update` | Server → Client | Claim status changed |
| `policy.update` | Server → Client | Policy modified |
| `coverage.verification` | Client → Server | Verify coverage |
| `coverage.response` | Server → Client | Coverage verification result |
| `fraud.alert` | Server → Client | Fraud detected |
| `premium.adjustment` | Server → Client | Premium changed |
| `payment.processed` | Server → Client | Payment completed |
| `heartbeat` | Bidirectional | Keep-alive ping/pong |

---

## 3. Claims Processing Protocol

### 3.1 Instant Claim Submission

**Flow:**
```
1. Pet Owner submits claim via mobile app
2. App sends claim to API endpoint
3. API validates claim data
4. Fraud detection system checks claim (< 2 seconds)
5. If low risk: Auto-approve
6. If medium/high risk: Send to review queue
7. Notification sent to owner
8. Blockchain record created
```

**Claim Submission Message:**
```json
{
  "messageId": "MSG-2025-123456",
  "type": "claim.submit",
  "timestamp": "2025-12-21T10:30:00Z",
  "payload": {
    "claimId": "CLM-2025-001234",
    "policyId": "PET-INS-2025-001234",
    "type": "Illness",
    "amount": 850.00,
    "clinic": {
      "id": "VET-2025-456",
      "name": "City Pet Hospital"
    },
    "documents": [
      {
        "type": "Invoice",
        "hash": "sha256:9f86d081..."
      }
    ]
  }
}
```

**Instant Response:**
```json
{
  "messageId": "MSG-2025-123457",
  "type": "claim.received",
  "timestamp": "2025-12-21T10:30:01Z",
  "payload": {
    "claimId": "CLM-2025-001234",
    "status": "Processing",
    "estimatedProcessingTime": 120,
    "trackingUrl": "https://portal.wia.com/claims/CLM-2025-001234"
  }
}
```

**Auto-Approval Message (if approved):**
```json
{
  "messageId": "MSG-2025-123458",
  "type": "claim.approved",
  "timestamp": "2025-12-21T10:30:03Z",
  "payload": {
    "claimId": "CLM-2025-001234",
    "status": "Approved",
    "amounts": {
      "total": 850.00,
      "insurancePays": 480.00,
      "ownerPays": 370.00
    },
    "paymentETA": "2025-12-22T00:00:00Z",
    "fraudScore": 15,
    "autoApproved": true
  }
}
```

### 3.2 Real-Time Status Updates

Clients subscribe to claim status updates:

**Subscribe:**
```json
{
  "type": "subscribe",
  "channel": "claim.CLM-2025-001234"
}
```

**Status Update:**
```json
{
  "type": "claim.status_update",
  "payload": {
    "claimId": "CLM-2025-001234",
    "previousStatus": "Processing",
    "currentStatus": "Approved",
    "changedAt": "2025-12-22T09:00:00Z",
    "changedBy": "AUTO-APPROVAL-SYSTEM"
  }
}
```

---

## 4. Policy Management Protocol

### 4.1 Policy Creation Flow

```
1. Owner fills application
2. System calculates premium
3. Health passport data synced (if available)
4. Risk assessment performed
5. Premium finalized
6. Payment processed
7. Policy activated
8. Blockchain record created
9. QR code & VC generated
10. Welcome email sent
```

**Policy Creation Event:**
```json
{
  "type": "policy.created",
  "payload": {
    "policyId": "PET-INS-2025-001234",
    "pet": { "name": "Buddy", "species": "Dog" },
    "plan": { "tier": "Premium" },
    "monthlyPremium": 89.99,
    "status": "Active",
    "startDate": "2025-01-01",
    "blockchain": {
      "transactionId": "0xfedcba0987654321...",
      "timestamp": "2025-01-01T00:00:00Z"
    }
  }
}
```

### 4.2 Policy Update Protocol

**Update Request:**
```json
{
  "type": "policy.update",
  "payload": {
    "policyId": "PET-INS-2025-001234",
    "changes": {
      "plan.tier": "Premium",
      "owner.email": "newemail@example.com"
    }
  }
}
```

**Update Confirmation:**
```json
{
  "type": "policy.updated",
  "payload": {
    "policyId": "PET-INS-2025-001234",
    "version": 2,
    "updatedFields": ["plan.tier", "owner.email"],
    "premiumChange": {
      "previous": 59.99,
      "new": 89.99,
      "effectiveDate": "2025-01-01"
    },
    "blockchain": {
      "transactionId": "0xabcd1234...",
      "timestamp": "2025-12-25T10:30:00Z"
    }
  }
}
```

---

## 5. Fraud Detection Protocol

### 5.1 Multi-Layer Fraud Detection

```
Layer 1: Pattern Recognition (ML)
   ↓
Layer 2: Cross-Provider Verification
   ↓
Layer 3: Blockchain Audit Trail
   ↓
Layer 4: Veterinary Clinic Validation
   ↓
Layer 5: Historical Analysis
```

### 5.2 Fraud Check Protocol

**Fraud Check Request:**
```json
{
  "type": "fraud.check",
  "payload": {
    "claimId": "CLM-2025-001234",
    "policyId": "PET-INS-2025-001234",
    "amount": 850.00,
    "clinic": { "id": "VET-2025-456" },
    "treatmentDate": "2025-12-20",
    "diagnosis": { "code": "K29.70" }
  }
}
```

**Fraud Check Response:**
```json
{
  "type": "fraud.result",
  "payload": {
    "claimId": "CLM-2025-001234",
    "fraudScore": 15,
    "riskLevel": "Low",
    "flags": [],
    "checks": {
      "duplicateClaim": false,
      "clinicVerified": true,
      "amountReasonable": true,
      "frequencyNormal": true,
      "patternMatch": false,
      "blockchainVerified": true
    },
    "recommendation": "Auto-approve",
    "confidence": 98
  }
}
```

### 5.3 Fraud Alert Protocol

**High-Risk Alert:**
```json
{
  "type": "fraud.alert",
  "severity": "High",
  "payload": {
    "claimId": "CLM-2025-001235",
    "fraudScore": 85,
    "flags": [
      "Duplicate claim detected",
      "Clinic not verified",
      "Amount exceeds typical range"
    ],
    "evidence": {
      "duplicateOf": "CLM-2025-000123",
      "clinicVerificationStatus": "Unverified",
      "amountStdDev": 3.2
    },
    "action": "Hold for manual review"
  }
}
```

---

## 6. Health Integration Protocol

### 6.1 Health Passport Sync

**Sync Request:**
```json
{
  "type": "health.sync",
  "payload": {
    "policyId": "PET-INS-2025-001234",
    "healthPassportId": "WIA-PET-HEALTH-001234",
    "syncMode": "full"
  }
}
```

**Sync Response:**
```json
{
  "type": "health.synced",
  "payload": {
    "policyId": "PET-INS-2025-001234",
    "healthPassportId": "WIA-PET-HEALTH-001234",
    "syncedAt": "2025-12-25T10:30:00Z",
    "records": {
      "vaccinations": 8,
      "medicalHistory": 12,
      "preventiveCare": 4
    },
    "healthScore": 85,
    "premiumAdjustment": {
      "previous": 89.99,
      "new": 84.99,
      "discount": 5.00,
      "reason": "Health score >= 80"
    }
  }
}
```

### 6.2 Auto-Claim from Health Records

When vet visit recorded in health passport, auto-submit claim:

**Health Event Notification:**
```json
{
  "type": "health.event",
  "payload": {
    "healthPassportId": "WIA-PET-HEALTH-001234",
    "eventType": "VetVisit",
    "eventDate": "2025-12-20",
    "clinic": { "id": "VET-2025-456" },
    "cost": 850.00,
    "diagnosis": "Gastritis"
  }
}
```

**Auto-Claim Creation:**
```json
{
  "type": "claim.auto_created",
  "payload": {
    "claimId": "CLM-2025-001234",
    "policyId": "PET-INS-2025-001234",
    "source": "health_passport",
    "healthEventId": "HEALTH-EVT-2025-567",
    "status": "Processing",
    "prePopulated": true
  }
}
```

---

## 7. Cross-Border Protocol

### 7.1 International Coverage

**Coverage Check for International Treatment:**
```json
{
  "type": "coverage.international_check",
  "payload": {
    "policyId": "PET-INS-2025-001234",
    "treatmentCountry": "CA",
    "estimatedCost": 1200.00,
    "currency": "CAD",
    "clinic": {
      "name": "Toronto Animal Hospital",
      "licenseNumber": "VET-ON-2024-789"
    }
  }
}
```

**Coverage Response:**
```json
{
  "type": "coverage.international_response",
  "payload": {
    "covered": true,
    "conversionRate": 0.74,
    "estimatedCostUSD": 888.00,
    "coverageDetails": {
      "insurancePaysUSD": 510.40,
      "ownerPaysUSD": 377.60,
      "insurancePaysCAD": 690.00,
      "ownerPaysCAD": 510.00
    },
    "preApprovalRequired": false,
    "additionalDocuments": ["Translation of medical records"]
  }
}
```

### 7.2 Multi-Currency Support

**Supported Currencies:**
- USD (United States Dollar)
- EUR (Euro)
- GBP (British Pound)
- CAD (Canadian Dollar)
- AUD (Australian Dollar)
- JPY (Japanese Yen)

**Currency Conversion Protocol:**
```json
{
  "type": "currency.convert",
  "payload": {
    "amount": 1200.00,
    "fromCurrency": "CAD",
    "toCurrency": "USD",
    "date": "2025-12-20"
  }
}
```

**Conversion Response:**
```json
{
  "type": "currency.converted",
  "payload": {
    "amount": 1200.00,
    "fromCurrency": "CAD",
    "toCurrency": "USD",
    "convertedAmount": 888.00,
    "exchangeRate": 0.74,
    "rateDate": "2025-12-20",
    "source": "XE.com API"
  }
}
```

---

## 8. Payment Protocol

### 8.1 Payment Processing Flow

```
1. Claim approved
2. Payment amount calculated
3. Payment method validated
4. Payment initiated
5. Bank/payment gateway processes
6. Confirmation received
7. Blockchain record updated
8. Owner notified
```

**Payment Initiation:**
```json
{
  "type": "payment.initiate",
  "payload": {
    "claimId": "CLM-2025-001234",
    "policyId": "PET-INS-2025-001234",
    "amount": 480.00,
    "currency": "USD",
    "method": "BankTransfer",
    "recipient": {
      "accountHolder": "John Smith",
      "accountNumber": "encrypted:...",
      "routingNumber": "encrypted:...",
      "bankName": "Wells Fargo"
    }
  }
}
```

**Payment Confirmation:**
```json
{
  "type": "payment.processed",
  "payload": {
    "claimId": "CLM-2025-001234",
    "paymentId": "PAY-2025-567890",
    "amount": 480.00,
    "currency": "USD",
    "processedAt": "2025-12-22T14:00:00Z",
    "estimatedArrival": "2025-12-24T00:00:00Z",
    "transactionId": "TXN-BANK-987654",
    "status": "Completed"
  }
}
```

### 8.2 Crypto Payment Support

**Crypto Payment Request:**
```json
{
  "type": "payment.crypto",
  "payload": {
    "claimId": "CLM-2025-001234",
    "amount": 480.00,
    "currency": "USD",
    "cryptoCurrency": "USDC",
    "network": "Ethereum",
    "recipientAddress": "0x1234567890abcdef..."
  }
}
```

**Crypto Payment Confirmation:**
```json
{
  "type": "payment.crypto_confirmed",
  "payload": {
    "claimId": "CLM-2025-001234",
    "paymentId": "PAY-CRYPTO-2025-123",
    "amount": 480.00,
    "cryptoAmount": 480.00,
    "cryptoCurrency": "USDC",
    "network": "Ethereum",
    "transactionHash": "0xabcdef1234567890...",
    "confirmations": 12,
    "status": "Confirmed"
  }
}
```

---

## 9. Notification Protocol

### 9.1 Multi-Channel Notifications

**Channels:**
- Email
- SMS
- Push Notification (Mobile App)
- WebSocket (Real-time)
- Webhook

**Notification Message:**
```json
{
  "type": "notification.send",
  "payload": {
    "recipientId": "OWNER-2025-456789",
    "channels": ["email", "push"],
    "template": "claim_approved",
    "data": {
      "claimId": "CLM-2025-001234",
      "amount": 480.00,
      "approvedAt": "2025-12-22T09:00:00Z"
    },
    "priority": "High"
  }
}
```

**Notification Status:**
```json
{
  "type": "notification.delivered",
  "payload": {
    "notificationId": "NOTIF-2025-123456",
    "channels": {
      "email": {
        "status": "Delivered",
        "deliveredAt": "2025-12-22T09:01:00Z"
      },
      "push": {
        "status": "Delivered",
        "deliveredAt": "2025-12-22T09:00:30Z"
      }
    }
  }
}
```

---

## 10. QR Code & VC Protocol

### 10.1 QR Code Generation

**QR Code Request:**
```json
{
  "type": "qr.generate",
  "payload": {
    "policyId": "PET-INS-2025-001234",
    "format": "png",
    "size": 400,
    "includeVC": true
  }
}
```

**QR Code Response:**
```json
{
  "type": "qr.generated",
  "payload": {
    "policyId": "PET-INS-2025-001234",
    "qrCode": "data:image/png;base64,iVBORw0KGg...",
    "url": "https://verify.wia.com/qr/PET-INS-2025-001234",
    "expiresAt": "2026-01-01T00:00:00Z",
    "vcIncluded": true
  }
}
```

### 10.2 QR Code Verification

**Scan and Verify:**
```json
{
  "type": "qr.verify",
  "payload": {
    "qrData": "encrypted_qr_payload",
    "scannedAt": "2025-12-25T10:30:00Z",
    "location": {
      "clinicId": "VET-2025-456",
      "latitude": 37.7749,
      "longitude": -122.4194
    }
  }
}
```

**Verification Response:**
```json
{
  "type": "qr.verified",
  "payload": {
    "valid": true,
    "policyId": "PET-INS-2025-001234",
    "pet": { "name": "Buddy", "species": "Dog" },
    "owner": { "name": "John Smith" },
    "plan": { "tier": "Premium" },
    "status": "Active",
    "coverageRemaining": 12500.00,
    "verifiedAt": "2025-12-25T10:30:01Z"
  }
}
```

### 10.3 Verifiable Credential Protocol

**VC Issuance:**
```json
{
  "type": "vc.issue",
  "payload": {
    "policyId": "PET-INS-2025-001234",
    "credentialType": "PetInsurancePolicy",
    "validUntil": "2026-01-01T00:00:00Z"
  }
}
```

**VC Response:**
```json
{
  "type": "vc.issued",
  "payload": {
    "@context": ["https://www.w3.org/2018/credentials/v1"],
    "type": ["VerifiableCredential", "PetInsurancePolicy"],
    "issuer": "did:wia:insurance:premium-pet",
    "issuanceDate": "2025-01-01T00:00:00Z",
    "credentialSubject": {
      "id": "did:wia:pet:001234",
      "policyId": "PET-INS-2025-001234"
    },
    "proof": {
      "type": "Ed25519Signature2020",
      "proofValue": "z3FXQzM2NjE4NTA..."
    }
  }
}
```

---

## 11. Error Handling & Retry

### 11.1 Retry Strategy

**Exponential Backoff:**
```
Attempt 1: Immediate
Attempt 2: 1 second
Attempt 3: 2 seconds
Attempt 4: 4 seconds
Attempt 5: 8 seconds
Max attempts: 5
```

**Retry Message:**
```json
{
  "type": "retry_attempt",
  "payload": {
    "originalMessageId": "MSG-2025-123456",
    "attempt": 2,
    "maxAttempts": 5,
    "nextRetryIn": 2000,
    "reason": "Timeout waiting for response"
  }
}
```

### 11.2 Dead Letter Queue

Failed messages after max retries go to DLQ:

```json
{
  "type": "dlq.message",
  "payload": {
    "originalMessage": { ... },
    "failureReason": "Max retries exceeded",
    "attempts": 5,
    "firstAttemptAt": "2025-12-25T10:30:00Z",
    "lastAttemptAt": "2025-12-25T10:30:15Z"
  }
}
```

---

## 12. Monitoring & Heartbeat

### 12.1 Heartbeat Protocol

**Ping:**
```json
{
  "type": "ping",
  "timestamp": "2025-12-25T10:30:00Z"
}
```

**Pong:**
```json
{
  "type": "pong",
  "timestamp": "2025-12-25T10:30:00Z",
  "serverTime": "2025-12-25T10:30:00.123Z"
}
```

### 12.2 Health Check

**Health Check Request:**
```json
{
  "type": "health.check"
}
```

**Health Check Response:**
```json
{
  "type": "health.status",
  "payload": {
    "status": "Healthy",
    "services": {
      "api": "Up",
      "database": "Up",
      "blockchain": "Up",
      "fraudDetection": "Up",
      "paymentGateway": "Up"
    },
    "latency": {
      "api": 15,
      "database": 5,
      "blockchain": 200
    },
    "timestamp": "2025-12-25T10:30:00Z"
  }
}
```

---

**弘益人間 · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 MIT License*
