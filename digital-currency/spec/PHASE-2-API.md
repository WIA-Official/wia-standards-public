# WIA-FIN-004 Digital Currency Standard
## Phase 2: API Specification

**Version:** 1.0  
**Status:** Production Ready  
**Last Updated:** 2025-01-15

---

## 1. Overview

This specification defines the RESTful API and WebSocket interfaces for digital currency operations. All WIA-FIN-004 compliant systems MUST provide these standard endpoints.

---

## 2. API Principles

- **RESTful Design**: Resources accessed via HTTP methods
- **JSON Payloads**: All requests and responses in JSON
- **HTTPS Only**: TLS 1.3+ required for all connections
- **Stateless**: Each request contains all necessary information
- **Versioned**: API version in URL path (`/api/v1/`)
- **Idempotent**: Support for idempotency keys on mutations
- **Rate Limited**: Protect against abuse

---

## 3. Authentication & Authorization

### 3.1 API Key Authentication

```http
GET /api/v1/accounts HTTP/1.1
Host: api.example.com
Authorization: Bearer YOUR_API_KEY
```

### 3.2 OAuth 2.0

```http
POST /oauth/token HTTP/1.1
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials&
client_id=YOUR_CLIENT_ID&
client_secret=YOUR_CLIENT_SECRET
```

Response:
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "read write"
}
```

---

## 4. Core Endpoints

### 4.1 Account Management

#### Get Account
```http
GET /api/v1/accounts/{accountId}
Authorization: Bearer {token}
```

Response:
```json
{
  "id": "ACC-123456",
  "type": "PERSONAL",
  "status": "ACTIVE",
  "owner": { /* ... */ },
  "balances": [
    {
      "currency": "USDC",
      "available": "1000.00",
      "pending": "50.00",
      "total": "1050.00"
    }
  ]
}
```

#### Create Account
```http
POST /api/v1/accounts
Content-Type: application/json
Authorization: Bearer {token}

{
  "type": "PERSONAL",
  "owner": {
    "type": "INDIVIDUAL",
    "name": "John Doe",
    "email": "john@example.com"
  },
  "currency": "USDC"
}
```

#### Get Balance
```http
GET /api/v1/accounts/{accountId}/balance?currency=USDC
```

### 4.2 Payments

#### Initiate Payment
```http
POST /api/v1/payments
Content-Type: application/json
Idempotency-Key: {unique-key}

{
  "from": {
    "type": "ACCOUNT",
    "identifier": "ACC-123456"
  },
  "to": {
    "type": "ACCOUNT",
    "identifier": "ACC-789012"
  },
  "amount": {
    "value": "100.00",
    "currency": "USDC"
  },
  "purpose": "Payment for services",
  "reference": "INV-001"
}
```

Response:
```json
{
  "paymentId": "PAY-abc123",
  "status": "PENDING",
  "estimatedCompletion": "2024-01-15T10:30:05Z",
  "fee": { "value": "0.50", "currency": "USDC" }
}
```

#### Get Payment Status
```http
GET /api/v1/payments/{paymentId}
```

#### List Payments
```http
GET /api/v1/payments?accountId=ACC-123456&limit=10&offset=0
```

### 4.3 Currency Exchange

#### Get Exchange Rates
```http
GET /api/v1/exchange/rates?from=USDC&to=USDT
```

Response:
```json
{
  "from": "USDC",
  "to": "USDT",
  "rate": "0.9998",
  "timestamp": "2024-01-15T10:30:00Z",
  "validUntil": "2024-01-15T10:31:00Z"
}
```

#### Execute Exchange
```http
POST /api/v1/exchange
{
  "from": {
    "currency": "USDC",
    "amount": "1000.00"
  },
  "to": {
    "currency": "USDT"
  },
  "accountId": "ACC-123456"
}
```

### 4.4 Compliance

#### Submit KYC
```http
POST /api/v1/compliance/kyc
Content-Type: multipart/form-data

{
  "accountId": "ACC-123456",
  "level": "STANDARD",
  "documents": [/* files */]
}
```

#### Check Transaction
```http
POST /api/v1/compliance/screening
{
  "transactionId": "TXN-abc123",
  "checks": ["AML", "SANCTIONS"]
}
```

---

## 5. WebSocket API

### 5.1 Connection

```javascript
const ws = new WebSocket('wss://api.example.com/ws');

ws.onopen = () => {
  ws.send(JSON.stringify({
    type: 'auth',
    token: 'YOUR_API_KEY'
  }));
};
```

### 5.2 Subscribe to Events

```javascript
// Subscribe to balance updates
ws.send(JSON.stringify({
  type: 'subscribe',
  channel: 'balance',
  accountId: 'ACC-123456'
}));

// Receive updates
ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  // { type: 'balance_update', accountId: '...', newBalance: '...' }
};
```

### 5.3 Event Types

- `balance_update`: Account balance changed
- `payment_received`: Incoming payment
- `payment_completed`: Outgoing payment completed
- `payment_failed`: Payment failed
- `compliance_alert`: Compliance flag raised

---

## 6. Webhooks

### 6.1 Register Webhook

```http
POST /api/v1/webhooks
{
  "url": "https://merchant.com/webhook",
  "events": ["payment.completed", "payment.failed"],
  "secret": "webhook_secret"
}
```

### 6.2 Webhook Payload

```json
{
  "event": "payment.completed",
  "timestamp": "2024-01-15T10:30:00Z",
  "data": {
    "paymentId": "PAY-123",
    "amount": "99.99",
    "currency": "USDC"
  },
  "signature": "sha256_hmac_signature"
}
```

### 6.3 Verify Webhook Signature

```javascript
const crypto = require('crypto');

function verifyWebhook(payload, signature, secret) {
  const computed = crypto
    .createHmac('sha256', secret)
    .update(JSON.stringify(payload))
    .digest('hex');
  
  return crypto.timingSafeEqual(
    Buffer.from(signature),
    Buffer.from(computed)
  );
}
```

---

## 7. Error Handling

### 7.1 Error Response Format

```json
{
  "error": {
    "code": "INSUFFICIENT_BALANCE",
    "message": "Account balance insufficient for transaction",
    "details": "Available: 50.00 USDC, Required: 100.00 USDC",
    "field": "amount",
    "timestamp": "2024-01-15T10:30:00Z",
    "requestId": "req-abc123"
  }
}
```

### 7.2 HTTP Status Codes

- `200 OK`: Success
- `201 Created`: Resource created
- `400 Bad Request`: Invalid request
- `401 Unauthorized`: Authentication failed
- `403 Forbidden`: Insufficient permissions
- `404 Not Found`: Resource not found
- `409 Conflict`: Duplicate/conflict
- `429 Too Many Requests`: Rate limited
- `500 Internal Server Error`: Server error
- `503 Service Unavailable`: Temporary outage

---

## 8. Rate Limiting

### 8.1 Rate Limit Headers

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 999
X-RateLimit-Reset: 1642248000
```

### 8.2 Default Limits

- **Standard**: 1000 requests/hour
- **Premium**: 10000 requests/hour
- **Enterprise**: Custom

---

## 9. Pagination

### 9.1 Request

```http
GET /api/v1/payments?limit=10&offset=20
```

### 9.2 Response

```json
{
  "data": [/* items */],
  "pagination": {
    "total": 100,
    "limit": 10,
    "offset": 20,
    "hasMore": true
  }
}
```

---

**End of Phase 2 Specification**

© 2025 SmileStory Inc. / WIA

---

## Annex A — Conformance Tier Matrix

WIA conformance for digital-currency is evaluated across three tiers:

| Tier | Scope | Mandatory artifacts | Audit cadence |
|------|-------|--------------------|----------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | OpenAPI 3.0 contract, JSON Schema validation report, security threat model | Annual self-review |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Tier 1 artifacts + signed third-party assessor report against this PHASE | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Tier 2 artifacts + WIA accreditation, ISO/IEC 17065:2012 conformity assessment, evidence retention ≥ 7 years | Every 12 months |

Implementations MUST disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page. Tier downgrade events MUST be reported to the WIA registry within 30 days.

---

## Annex B — Cross-Walk to International Standards

The PHASE specification reuses or normatively references published standards alongside this document. Implementers SHOULD review the relevant standards alongside this PHASE document; where a conflict exists, the more specific WIA requirement governs unless explicitly superseded by a binding national regulation.

- ISO/IEC 17065:2012 — Conformity assessment — Requirements for bodies certifying products, processes and services
- ISO/IEC 27001:2022 — Information security management systems
- IETF RFC 9457 — Problem Details for HTTP APIs
- IETF RFC 7519 — JSON Web Token (JWT)
- IETF RFC 6749 — The OAuth 2.0 Authorization Framework
- W3C PROV-DM — provenance data model

This cross-walk is informative only. WIA does not republish the referenced documents; readers MUST obtain authoritative copies from the issuing body. Cross-walk entries are reviewed at every minor version of this PHASE.

---

## Annex C — Reference Implementations and Test Vectors

### C.1 Reference Implementations

WIA does not require implementers to use a particular library, but maintains pointers to the canonical reference implementation directories under the WIA-Official GitHub organization:

- `wia-standards/standards/digital-currency/api/` — TypeScript SDK skeleton
- `wia-standards/standards/digital-currency/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/digital-currency/simulator/` — interactive browser-based simulator for the PHASE protocol

Each reference artifact ships with an MIT license and is intended as a starting point, not as a production-grade implementation.

### C.2 Test Vectors

A normative set of request/response pairs covering the schemas defined in this PHASE is published alongside this document. Implementations claiming Tier 2 or Tier 3 conformance MUST pass every published test vector in both serialization (request) and deserialization (response) directions, and MUST publish a signed report identifying which vectors were exercised.

Test vectors are versioned independently of the PHASE document; refer to the `test-vectors/` directory for the active version.

---

## Annex D — Open Questions and Future Work

This PHASE document captures the consensus position at v1.0. The following items are tracked for future minor or major revisions:

1. **Schema versioning policy** — formal MUST/SHOULD rules for backward-compatible vs. breaking changes between minor releases.
2. **Privacy-preserving aggregation** — guidance on differential privacy and secure aggregation patterns for telemetry channels, without prescribing a specific algorithm.
3. **Multilingual error catalogs** — localization strategy for the standard error codes defined in this PHASE.
4. **Long-term retention** — alignment with sectoral retention regulations across jurisdictions.
5. **Sustainability disclosure** — optional fields for energy and emissions reporting tied to operations covered by this PHASE.

Items in this annex are non-normative. Comments and proposals are accepted via the GitHub issues tracker on the WIA-Official organization.

---


## Annex E — Implementation Notes for PHASE-2-API

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-2-API.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.
