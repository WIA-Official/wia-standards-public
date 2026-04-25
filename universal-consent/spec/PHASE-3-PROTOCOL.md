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

## P.3 Protocol Cross-References

The protocol defined here carries the data formats from Phase 1 and the API
operations from Phase 2 across trust boundaries. Phase 4 describes how the
protocol composes with adjacent infrastructure.

### P.3.1 Transport Bindings

| Binding | Default Port | Use |
|---------|-------------:|-----|
| HTTP/2 + TLS 1.3 | 443 | Public, request-response |
| HTTP/3 (QUIC) | 443 | Mobile, lossy networks |
| gRPC | 443 | Service-to-service |
| MQTT 5.0 | 8883 | Constrained / IoT devices |
| AMQP 0-9-1 | 5671 | Backplane / event streams |

### P.3.2 Message Envelope

Every protocol message carries a small envelope independent of payload:

```
+----------------+------------------+--------------------+
| message_id     | UUIDv4           | RFC 4122           |
| trace_id       | 16-byte hex      | W3C Trace Context  |
| span_id        | 8-byte hex       | W3C Trace Context  |
| origin_node    | DNS name or NIN  | RFC 1035           |
| issued_at      | RFC 3339         | UTC required       |
| ttl_seconds    | uint32           | 0 = no expiry      |
| content_type   | media type       | RFC 6838           |
| body           | opaque bytes     | per content_type   |
+----------------+------------------+--------------------+
```

### P.3.3 Reliability Model

The protocol provides at-least-once delivery by default. Receivers
deduplicate by `message_id`. Exactly-once semantics are achieved when both
peers participate in the idempotency contract from Phase 2 §P.2.3.

### P.3.4 Backpressure

Senders MUST honour HTTP `Retry-After`, gRPC `RESOURCE_EXHAUSTED`, or MQTT
flow-control packets. The recommended back-off is full jitter exponential with
cap 30 s and cumulative cap 5 min.


---

## Appendix · Common WIA Standard Provisions

> The following provisions apply to every WIA standard and are kept in sync
> across the WIA-Standards corpus. Standard-specific deviations, where they
> exist, are listed in the standard's normative body.

### A. Conformance & Compliance

#### A.1 Conformance Levels

WIA-Standards defines four conformance levels:

| Level | Required | Description |
|-------|----------|-------------|
| L1 — Format | Phase 1 | Implementation produces and consumes the canonical data format losslessly |
| L2 — Interface | Phase 1 + 2 | Implementation exposes the API surface with required behaviour |
| L3 — Protocol | Phase 1 + 2 + 3 | Implementation interoperates over at least one normative transport binding |
| L4 — Integration | All Phases | Implementation passes the conformance test suite end-to-end in a production-shaped deployment |

Conformance claims MUST cite the level and the version of the standard
against which the claim is made (e.g. "L3 conformant against v1.0").

#### A.2 Compliance Verification

The conformance test suite is published alongside this standard at
`/cli/conformance/` and `/api/conformance/`. Implementations claiming
L2 or higher MUST publish their test report. Independent re-tests are
encouraged; the WIA Working Group accepts third-party verification reports
under the policy in §E.

### B. Security Considerations

#### B.1 Threat Model

Implementers SHOULD apply STRIDE analysis covering: spoofing of identity,
tampering with messages or stored state, repudiation of operations,
information disclosure, denial of service, and elevation of privilege.

| Threat | Default Control | Where Strengthened |
|--------|-----------------|--------------------|
| Spoofing | Mutual TLS or signed tokens | Phase 3 §P.3 |
| Tampering | TLS in transit, AEAD at rest | Phase 1 §P.1 |
| Repudiation | Append-only audit log with notarization | Phase 4 §P.4 |
| Disclosure | Field-level encryption for PII / secrets | Phase 1 §P.1 |
| DoS | Rate limit per principal & global circuit breaker | Phase 2 §P.2 |
| EoP | Least-privilege RBAC + scoped tokens | Phase 2 §P.2 |

#### B.2 Cryptographic Suites

Mandatory: TLS 1.3 with AEAD ciphers (AES-128-GCM, AES-256-GCM,
CHACHA20-POLY1305). Forbidden: TLS 1.0, TLS 1.1, RC4, MD5, SHA-1 for
signatures, RSA below 2048 bits, ECDSA on curves smaller than P-256.

Post-quantum migration: implementations SHOULD adopt hybrid key
exchange combining a classical primitive with ML-KEM (FIPS 203) once a
profile is published; signature migration to ML-DSA (FIPS 204) is
expected within the L4 conformance window of v2.0.

#### B.3 Audit Requirements

L3 and L4 implementations MUST log: (a) every authentication decision,
(b) every authorization decision, (c) every state-changing operation,
(d) every export of data outside its sovereignty boundary. Logs are
write-once for at least 1 year and 90 days indexed for incident search.

### C. Versioning & Lifecycle

Versions follow Semantic Versioning 2.0.0 (MAJOR.MINOR.PATCH).

| Phase | Duration | Conformance Status |
|-------|---------:|-------------------|
| Draft | until ratification | Non-binding |
| Active | indefinite | Binding for new deployments |
| Maintenance | 24 months from successor's Active date | Binding for existing deployments |
| Retired | indefinite | Non-binding; conformance claims rescinded |

Deprecation MUST be announced at least one minor version before a feature
is removed in a major version.

### D. Internationalization & Accessibility

Implementations SHOULD support locale negotiation via the
`Accept-Language` header (RFC 4647). Date, time, number, and currency
formatting follow CLDR. User-facing surfaces MUST satisfy WCAG 2.1 AA at
minimum and SHOULD progress towards WCAG 2.2 AA. Right-to-left scripts
(Arabic, Hebrew, Persian, Urdu) and East-Asian wide characters MUST be
laid out correctly without line-breaking heuristics that split graphemes.

### E. Governance & IP Policy

This standard is maintained by the WIA Working Group under the WIA
governance charter. Editorial changes are merged via pull request. Normative
changes require working-group consensus and a 30-day public review.
Contributions are accepted under the Apache License 2.0 with explicit
patent grant. Members participate under the WIA Patent Policy,
which requires royalty-free licensing of any essential claim necessary
to implement a normative requirement.

### F. Normative References

The following references are normative; implementations MUST satisfy
the cited clauses:

- ISO/IEC 27001:2022 — Information security management systems
- ISO/IEC 27017:2015 — Cloud-services security controls
- ISO/IEC 27701:2019 — Privacy information management
- ISO/IEC 19790:2012 — Security requirements for cryptographic modules
- ISO 8601-1:2019 — Date and time representation
- IETF RFC 8446 — TLS 1.3
- IETF RFC 7519 — JSON Web Token
- IETF RFC 6749 — OAuth 2.0
- IETF RFC 9110 — HTTP Semantics
- IETF RFC 9112 — HTTP/1.1 message syntax
- IETF RFC 9113 — HTTP/2
- IETF RFC 9114 — HTTP/3
- IETF RFC 9000 — QUIC transport
- IETF RFC 4122 — UUID URN namespace
- IETF RFC 3339 — Date and time on the Internet
- IETF RFC 6838 — Media-type specifications and registration
- W3C TraceContext — Distributed tracing context
- W3C WCAG 2.1 — Accessibility guidelines
- FIPS PUB 197 — AES
- FIPS PUB 180-4 — SHA-2 family
- FIPS PUB 203 — ML-KEM (post-quantum KEM)
- FIPS PUB 204 — ML-DSA (post-quantum signature)

### G. Glossary

| Term | Definition |
|------|------------|
| Conformance | The state of satisfying every normative requirement at a given level |
| Implementation | A software, hardware, or composite artefact that claims conformance |
| Principal | The authenticated entity bound to a security context |
| Subject | The resource or person to which an operation applies |
| Sovereignty Boundary | The legal / regulatory perimeter outside of which data export is restricted |

---

*This Appendix is authored by the WIA Standards Working Group and is kept
in lockstep across Phases 1–4 of universal-consent so that conformance claims at any
Phase remain unambiguous.*

