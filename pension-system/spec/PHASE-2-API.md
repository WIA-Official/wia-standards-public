# WIA-SOC-018 Phase 2: API Interface Specification

**Version:** 1.0.0  
**Status:** Approved  
**Last Updated:** 2025-12-26

弘익人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. Overview

Phase 2 defines the RESTful API interfaces for pension system integration, enabling standardized communication between pension platforms, employers, members, and regulators.

## 2. API Architecture

### 2.1 Base URL Structure

```
https://api.{pension-provider}.com/wia/soc-018/v1/{resource}
```

### 2.2 Authentication

All API requests MUST include authentication via:

```http
Authorization: Bearer {JWT_TOKEN}
X-API-Key: {API_KEY}
X-Request-ID: {UNIQUE_REQUEST_ID}
```

JWT Token payload MUST include:
```json
{
  "iss": "issuer",
  "sub": "subject (user/system ID)",
  "aud": "audience",
  "exp": "expiration timestamp",
  "iat": "issued at timestamp",
  "scope": "space-separated permissions"
}
```

### 2.3 Rate Limiting

- **Standard Tier**: 100 requests/minute
- **Premium Tier**: 1,000 requests/minute
- **Enterprise Tier**: 10,000 requests/minute

Rate limit headers:
```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 987
X-RateLimit-Reset: 1640995200
```

## 3. Core API Endpoints

### 3.1 Member Management

#### GET /members/{memberId}

Retrieve member information.

**Response:**
```json
{
  "status": "success",
  "data": {
    "memberId": "UUID",
    "personalInfo": { /* PensionMember object */ },
    "enrollmentDate": "ISO8601",
    "status": "active"
  },
  "meta": {
    "requestId": "UUID",
    "timestamp": "ISO8601"
  }
}
```

#### POST /members

Create new member.

**Request:**
```json
{
  "personalInfo": {
    "firstName": "John",
    "lastName": "Doe",
    "dateOfBirth": "1985-05-15",
    "taxId": "123-45-6789"
  },
  "contactInfo": { /* ContactInfo object */ }
}
```

#### PATCH /members/{memberId}

Update member information.

### 3.2 Contribution Management

#### POST /contributions

Submit contribution record.

**Request:**
```json
{
  "memberId": "UUID",
  "contributionPeriod": {
    "startDate": "2025-01-01",
    "endDate": "2025-01-31"
  },
  "employerInfo": { /* EmployerInfo object */ },
  "earnings": { /* Earnings object */ },
  "contributions": { /* Contributions object */ }
}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "contributionId": "UUID",
    "validationStatus": "validated",
    "blockchainAnchor": {
      "txHash": "0x...",
      "blockNumber": 12345678
    }
  }
}
```

#### GET /contributions/member/{memberId}

Retrieve contribution history.

**Query Parameters:**
- `startDate`: ISO8601 date
- `endDate`: ISO8601 date
- `page`: integer (default: 1)
- `limit`: integer (default: 50, max: 500)

#### POST /contributions/batch

Submit multiple contributions in single request.

**Request:**
```json
{
  "contributions": [
    { /* Contribution object */ },
    { /* Contribution object */ }
  ]
}
```

### 3.3 Benefit Calculation

#### POST /benefits/calculate

Calculate projected benefits.

**Request:**
```json
{
  "memberId": "UUID",
  "retirementAge": 65,
  "calculationDate": "2025-12-26",
  "scenario": "standard|optimistic|pessimistic"
}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "calculationId": "UUID",
    "monthlyBenefit": 2450.00,
    "annualBenefit": 29400.00,
    "replacementRate": 0.49,
    "projections": [ /* Array of projections */ ]
  }
}
```

#### GET /benefits/member/{memberId}

Retrieve benefit estimates and history.

### 3.4 Fund Management

#### GET /funds/allocation/{memberId}

Get current fund allocation.

#### PUT /funds/allocation/{memberId}

Update fund allocation preferences.

**Request:**
```json
{
  "effectiveDate": "2025-01-01",
  "assetAllocation": [
    {
      "assetClass": "domestic_equity",
      "allocationPercentage": 40.0
    },
    {
      "assetClass": "international_equity",
      "allocationPercentage": 25.0
    },
    {
      "assetClass": "fixed_income",
      "allocationPercentage": 30.0
    },
    {
      "assetClass": "alternatives",
      "allocationPercentage": 5.0
    }
  ]
}
```

#### GET /funds/performance

Get fund performance metrics.

### 3.5 Cross-Border Transfer

#### POST /portability/initiate

Initiate cross-border pension transfer.

**Request:**
```json
{
  "memberId": "UUID",
  "fromJurisdiction": "US",
  "toJurisdiction": "GB",
  "transferAmount": 124580.00,
  "transferType": "full|partial",
  "receivingPensionScheme": "UUID"
}
```

#### GET /portability/status/{transferId}

Check transfer status.

#### GET /portability/treaties

List available social security treaties.

## 4. Webhook Notifications

### 4.1 Webhook Registration

#### POST /webhooks

Register webhook endpoint.

**Request:**
```json
{
  "url": "https://your-system.com/webhooks/pension",
  "events": [
    "contribution.created",
    "contribution.validated",
    "benefit.calculated",
    "transfer.completed"
  ],
  "secret": "webhook signing secret"
}
```

### 4.2 Webhook Payload

```json
{
  "event": "contribution.created",
  "timestamp": "ISO8601",
  "data": { /* Event-specific data */ },
  "signature": "HMAC-SHA256 signature"
}
```

### 4.3 Webhook Security

All webhook requests include:
- **X-WIA-Signature**: HMAC-SHA256 signature
- **X-WIA-Timestamp**: Request timestamp
- **X-WIA-Event**: Event type

Verify signature:
```
signature = HMAC-SHA256(secret, timestamp + '.' + payload)
```

## 5. Error Handling

### 5.1 Error Response Format

```json
{
  "status": "error",
  "error": {
    "code": "ERROR_CODE",
    "message": "Human-readable error message",
    "details": {
      "field": "contributionAmount",
      "issue": "Must be greater than 0.01"
    }
  },
  "meta": {
    "requestId": "UUID",
    "timestamp": "ISO8601"
  }
}
```

### 5.2 Standard Error Codes

- **400**: Bad Request - Invalid input data
- **401**: Unauthorized - Missing or invalid authentication
- **403**: Forbidden - Insufficient permissions
- **404**: Not Found - Resource doesn't exist
- **409**: Conflict - Resource already exists or version conflict
- **422**: Unprocessable Entity - Validation failed
- **429**: Too Many Requests - Rate limit exceeded
- **500**: Internal Server Error - Server-side error
- **503**: Service Unavailable - Temporary outage

## 6. API Versioning

### 6.1 Versioning Strategy

- **URL Path Versioning**: `/v1/`, `/v2/`
- **Backwards Compatibility**: Maintained for 24 months
- **Deprecation Notice**: 12 months advance notice
- **Version Header**: `X-API-Version: 1.0.0`

### 6.2 Breaking Changes

Breaking changes require new major version:
- Removing endpoints or fields
- Changing field types
- Modifying validation rules
- Changing authentication methods

## 7. Performance Requirements

- **Response Time**: 95th percentile < 200ms
- **Availability**: 99.95% uptime SLA
- **Data Freshness**: Real-time for contributions, T+1 for calculations
- **Throughput**: Handle 10,000 requests/second at peak

## 8. Compliance and Testing

### 8.1 Compliance Certification

API implementations MUST:
1. Pass automated test suite (500+ test cases)
2. Support all Phase 2 endpoints
3. Implement proper authentication and authorization
4. Provide OpenAPI 3.0 specification
5. Maintain 99.9% uptime over 30 days

### 8.2 Testing Tools

- **Postman Collection**: Available at wiastandards.com/soc-018/postman
- **Test Data**: Sample datasets provided
- **Mock Server**: Reference implementation available

---

© 2025 WIA · MIT License · 弘益人間 (Benefit All Humanity)

---

## Annex A — Conformance Tier Matrix

WIA conformance for pension-system is evaluated across three tiers:

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

- `wia-standards/standards/pension-system/api/` — TypeScript SDK skeleton
- `wia-standards/standards/pension-system/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/pension-system/simulator/` — interactive browser-based simulator for the PHASE protocol

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

