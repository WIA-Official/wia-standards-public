# WIA-CORE-002 PHASE 3: Protocol Specification

**Version:** 1.0  
**Status:** Stable  
**Last Updated:** January 2025

## Overview

Phase 3 defines communication protocols for consent propagation, synchronization, and verification across organizational boundaries. These protocols enable consent to flow seamlessly between systems, partners, and third parties.

## Protocol Principles

1. **Real-Time:** Minimize latency in consent propagation
2. **Reliable:** Ensure message delivery with retry mechanisms
3. **Secure:** Cryptographic signatures and mutual TLS
4. **Auditable:** Complete audit trail of protocol interactions
5. **Interoperable:** Work across different platforms and organizations

## 1. Consent Synchronization Protocol

### Event-Driven Synchronization

**Publisher (Consent Management System):**

```http
POST https://subscriber.example.com/wia/consent/sync
Content-Type: application/json
X-WIA-Signature: sha256=abc123...
X-WIA-Event-ID: evt-abc123
X-WIA-Timestamp: 2025-06-20T14:22:00Z
X-WIA-Retry-Count: 0

{
  "event": "consent.updated",
  "consentId": "consent-550e8400...",
  "userId": "user-789012",
  "timestamp": "2025-06-20T14:22:00Z",
  "changes": {
    "purposes.marketing-email.granted": {
      "old": true,
      "new": false
    }
  },
  "metadata": {
    "source": "preference-center",
    "ipAddress": "192.0.2.1"
  },
  "signature": "RSA_SIGNATURE_HERE"
}
```

**Subscriber Response:**

```json
{
  "received": true,
  "processedAt": "2025-06-20T14:22:01Z",
  "affectedRecords": 3,
  "status": "success",
  "subscriberId": "subscriber-xyz789"
}
```

### Event Types

- `consent.created` - New consent record created
- `consent.updated` - Consent preferences updated
- `consent.revoked` - Consent withdrawn
- `consent.expired` - Consent expired
- `consent.verified` - Consent verified (e.g., email confirmation)

### Retry Logic

```
Attempt 1: Immediate
Attempt 2: 5 seconds
Attempt 3: 30 seconds
Attempt 4: 2 minutes
Attempt 5: 10 minutes
Dead Letter Queue: After 5 failed attempts
```

## 2. Third-Party Consent Verification

### Verification Request

```http
POST https://consent-authority.example.com/api/v1/verify
Authorization: Bearer {api_key}
Content-Type: application/json

{
  "userId": "user-789012",
  "purposeId": "marketing-email",
  "requestingParty": "partner-company-xyz",
  "context": {
    "dataSource": "shared-customer-list",
    "timestamp": "2025-06-20T14:30:00Z",
    "ipAddress": "203.0.113.1"
  }
}
```

### Verification Response

```json
{
  "isValid": true,
  "consentId": "consent-550e8400...",
  "grantedAt": "2025-01-15T10:30:00Z",
  "expiresAt": "2026-01-15T10:30:00Z",
  "scope": ["marketing-email"],
  "thirdPartyAllowed": true,
  "restrictions": {
    "geographicRestrictions": ["EU", "EEA"],
    "dataCategories": ["email-address", "name"],
    "processingRestrictions": ["no-profiling"]
  },
  "verificationToken": "verify-xyz789...",
  "validUntil": "2025-06-20T15:30:00Z",
  "signature": "RSA_SIGNATURE_HERE"
}
```

## 3. Consent Delegation Protocol

### Delegation Grant

```json
{
  "delegationId": "deleg-abc123",
  "delegator": "controller.example.com",
  "delegate": "processor.example.com",
  "purposes": ["analytics", "personalization"],
  "validFrom": "2025-01-15T00:00:00Z",
  "validUntil": "2026-01-15T00:00:00Z",
  "restrictions": {
    "jurisdictions": ["EU", "EEA"],
    "dataCategories": ["behavioral", "preferences"],
    "operations": ["read", "aggregate"],
    "subDelegation": false
  },
  "auditRequired": true,
  "signature": {
    "algorithm": "RS256",
    "value": "...",
    "certificate": "-----BEGIN CERTIFICATE-----..."
  }
}
```

### Delegation Verification

```http
GET https://consent-authority.example.com/api/v1/delegations/{delegationId}/verify
Authorization: Bearer {api_key}
```

**Response:**

```json
{
  "valid": true,
  "delegationId": "deleg-abc123",
  "delegator": "controller.example.com",
  "delegate": "processor.example.com",
  "purposes": ["analytics", "personalization"],
  "validUntil": "2026-01-15T00:00:00Z",
  "restrictions": {...}
}
```

## 4. Consent Receipt Protocol

### Receipt Generation

```json
{
  "receiptId": "receipt-xyz789",
  "receiptVersion": "1.0",
  "consentId": "consent-550e8400...",
  "userId": "user-789012",
  "timestamp": "2025-01-15T10:30:00Z",
  "purposes": [
    {
      "purposeId": "marketing-email",
      "purposeName": "Email Marketing Communications",
      "granted": true,
      "description": "Receive promotional emails about products and services",
      "dataCategories": ["email-address", "name"],
      "retentionPeriod": "P2Y"
    }
  ],
  "organization": {
    "name": "Example Corp",
    "jurisdiction": "EU",
    "registrationNumber": "12345678",
    "privacyOfficer": "privacy@example.com",
    "dataProtectionOfficer": "dpo@example.com"
  },
  "userRights": [
    "You may withdraw consent at any time by visiting your preference center",
    "You have the right to access your personal data",
    "You have the right to data portability",
    "You have the right to rectification of inaccurate data",
    "You have the right to erasure (right to be forgotten)"
  ],
  "withdrawalMechanism": {
    "method": "online",
    "url": "https://example.com/preferences",
    "email": "privacy@example.com"
  },
  "signature": {
    "algorithm": "RS256",
    "value": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
    "certificate": "-----BEGIN CERTIFICATE-----...",
    "timestamp": "2025-01-15T10:30:00Z"
  }
}
```

### Receipt Verification

Users can verify receipts independently:

```bash
wia-consent verify-receipt --receipt receipt-xyz789.json --public-key example-corp.pem
```

## 5. Real-Time Consent Status Broadcasting

### WebSocket Connection

```javascript
const ws = new WebSocket('wss://consent-stream.example.com/subscribe');

ws.send(JSON.stringify({
  action: 'subscribe',
  authorization: 'Bearer {token}',
  userIds: ['user-789012', 'user-456789'],
  purposes: ['marketing-email', 'analytics']
}));

ws.onmessage = (event) => {
  const update = JSON.parse(event.data);
  console.log('Consent update:', update);
};
```

### Broadcast Message

```json
{
  "event": "consent.updated",
  "userId": "user-789012",
  "purposeId": "marketing-email",
  "granted": false,
  "timestamp": "2025-06-20T14:22:00Z",
  "consentId": "consent-550e8400...",
  "source": "preference-center"
}
```

## 6. Cross-Organization Consent Exchange

### Transfer Request

```json
{
  "transferId": "xfer-abc123",
  "source": "company-a.example.com",
  "destination": "company-b.example.com",
  "timestamp": "2025-06-20T14:30:00Z",
  "legalBasis": "merger",
  "userNotification": {
    "required": true,
    "method": "email",
    "template": "merger-notification-v2",
    "sentAt": "2025-06-15T10:00:00Z"
  },
  "consents": [
    {
      "consentId": "consent-550e8400...",
      "userId": "user-789012",
      "transferAuthorization": "auth-xyz789...",
      "dataCategories": ["contact-info", "preferences"]
    }
  ],
  "auditTrail": true,
  "regulatoryApproval": {
    "authority": "ICO",
    "approvalRef": "approval-ref-123",
    "approvedAt": "2025-06-10T12:00:00Z"
  },
  "signature": "..."
}
```

## 7. Conflict Resolution Protocol

When multiple systems have conflicting consent states:

### Priority Rules

1. **Most Restrictive Wins:** If any system shows consent revoked, treat as revoked globally
2. **Timestamp Priority:** Most recent update takes precedence
3. **User Intent Priority:** Direct user actions override automated processes
4. **Audit and Alert:** All conflicts logged and investigated

### Conflict Notification

```json
{
  "conflictId": "conflict-abc123",
  "timestamp": "2025-06-20T14:30:00Z",
  "consentId": "consent-550e8400...",
  "userId": "user-789012",
  "conflictingSystems": [
    {
      "system": "crm.example.com",
      "status": "active",
      "lastUpdated": "2025-06-20T14:20:00Z"
    },
    {
      "system": "marketing.example.com",
      "status": "revoked",
      "lastUpdated": "2025-06-20T14:22:00Z"
    }
  ],
  "resolution": {
    "rule": "most-restrictive-wins",
    "resolvedStatus": "revoked",
    "appliedAt": "2025-06-20T14:30:01Z"
  }
}
```

## Security Requirements

### Mutual TLS

Both parties must authenticate with X.509 certificates:

```
Client Certificate: client.example.com
Server Certificate: server.example.com
CA: WIA Certificate Authority
```

### Request Signing

All protocol messages must be signed:

```
Algorithm: HMAC-SHA256 or RSA-SHA256
Signature Header: X-WIA-Signature
Timestamp: X-WIA-Timestamp (for replay prevention)
Nonce: X-WIA-Nonce (for replay prevention)
```

### Replay Prevention

```
Maximum timestamp drift: 5 minutes
Nonce cache: 10 minutes
Duplicate detection: By nonce + timestamp
```

---

**Previous:** [PHASE 2: API Interface](PHASE-2-API-INTERFACE.md)  
**Next:** [PHASE 4: Integration](PHASE-4-INTEGRATION.md)

© 2025 SmileStory Inc. / WIA · 弘益人間 (Benefit All Humanity)
