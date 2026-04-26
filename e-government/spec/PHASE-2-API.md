# WIA-SOC-003 Phase 2: API Specification

**Version:** 1.0.0
**Status:** Approved
**Last Updated:** 2025-12-26

弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. Overview

Phase 2 defines the RESTful API specifications for e-government services, including authentication endpoints, service request management, document handling, and real-time notifications. All APIs follow OpenAPI 3.0 specification.

## 2. Base Configuration

### 2.1 Base URL

```
Production: https://api.egov.{country-code}/v1
Sandbox: https://sandbox-api.egov.{country-code}/v1
```

### 2.2 Authentication

All API requests MUST include authentication:

```http
Authorization: Bearer {jwt-token}
X-API-Key: {api-key}
X-Request-ID: {uuid}
```

### 2.3 Headers

| Header | Required | Description |
|--------|----------|-------------|
| Authorization | Yes | JWT Bearer token |
| X-API-Key | Yes | API key for rate limiting |
| X-Request-ID | Yes | UUID for request tracking |
| Content-Type | Yes | application/json |
| Accept-Language | No | Preferred language (default: en) |
| X-Country-Code | No | ISO 3166-1 alpha-2 (default: from API domain) |

## 3. Authentication APIs

### 3.1 Citizen Authentication

**Endpoint**: `POST /auth/citizen`

**Request**:
```json
{
  "identityNumber": "encrypted-id",
  "method": "biometric",
  "biometricData": {
    "type": "face",
    "data": "base64-encoded-image",
    "metadata": {
      "captureTime": "2025-12-26T10:30:00Z",
      "device": "iPhone 15 Pro"
    }
  },
  "mfaToken": "optional-second-factor"
}
```

**Response (200 OK)**:
```json
{
  "success": true,
  "token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "refreshToken": "refresh-token-uuid",
  "expiresIn": 3600,
  "citizen": {
    "id": "urn:uuid:550e8400-e29b-41d4-a716-446655440000",
    "name": "Kim MinJun",
    "verificationLevel": "high"
  }
}
```

### 3.2 Token Refresh

**Endpoint**: `POST /auth/refresh`

**Request**:
```json
{
  "refreshToken": "refresh-token-uuid"
}
```

**Response (200 OK)**:
```json
{
  "token": "new-jwt-token",
  "expiresIn": 3600
}
```

### 3.3 Logout

**Endpoint**: `POST /auth/logout`

**Request**:
```json
{
  "token": "current-jwt-token"
}
```

**Response (200 OK)**:
```json
{
  "success": true,
  "message": "Logged out successfully"
}
```

## 4. Citizen Management APIs

### 4.1 Get Citizen Profile

**Endpoint**: `GET /citizen/{citizenId}`

**Response (200 OK)**:
```json
{
  "id": "urn:uuid:550e8400-e29b-41d4-a716-446655440000",
  "identityNumber": "****-****-4567",
  "givenName": "MinJun",
  "familyName": "Kim",
  "dateOfBirth": "1990-05-15",
  "nationality": "KR",
  "residency": {
    "country": "KR",
    "region": "Seoul",
    "district": "Gangnam-gu"
  },
  "contact": {
    "email": "kim.****@example.com",
    "phone": "+82-10-****-5678",
    "preferredLanguage": "ko"
  },
  "verificationLevel": "high",
  "createdAt": "2020-01-01T00:00:00Z",
  "updatedAt": "2025-12-26T10:00:00Z"
}
```

### 4.2 Update Citizen Profile

**Endpoint**: `PUT /citizen/{citizenId}`

**Request**:
```json
{
  "contact": {
    "email": "new.email@example.com",
    "phone": "+82-10-9876-5432"
  },
  "residency": {
    "district": "Songpa-gu",
    "postalCode": "05500"
  },
  "verification": {
    "method": "otp",
    "code": "123456"
  }
}
```

**Response (200 OK)**:
```json
{
  "success": true,
  "citizen": { /* updated citizen object */ },
  "message": "Profile updated successfully"
}
```

## 5. Service Request APIs

### 5.1 List Available Services

**Endpoint**: `GET /services/available`

**Query Parameters**:
- `category` (optional): Filter by category
- `language` (optional): Service description language
- `page` (optional): Page number (default: 1)
- `limit` (optional): Items per page (default: 20, max: 100)

**Response (200 OK)**:
```json
{
  "services": [
    {
      "id": "SRV-TAX-001",
      "name": {
        "en": "Individual Tax Filing",
        "ko": "개인 세금 신고"
      },
      "category": "taxation",
      "averageProcessingTime": "P3D",
      "fee": 0,
      "available": true
    }
  ],
  "pagination": {
    "page": 1,
    "limit": 20,
    "total": 142,
    "pages": 8
  }
}
```

### 5.2 Submit Service Request

**Endpoint**: `POST /services/request`

**Request**:
```json
{
  "serviceId": "SRV-TAX-001",
  "priority": "normal",
  "metadata": {
    "taxYear": 2024,
    "filingType": "individual"
  },
  "documents": ["DOC-2025-5678", "DOC-2025-5679"],
  "notes": "First time filing"
}
```

**Response (201 Created)**:
```json
{
  "success": true,
  "request": {
    "requestId": "REQ-2025-001234",
    "status": "submitted",
    "estimatedCompletion": "2025-12-30T17:00:00Z",
    "trackingUrl": "https://portal.egov.kr/track/REQ-2025-001234"
  }
}
```

### 5.3 Get Request Status

**Endpoint**: `GET /services/request/{requestId}`

**Response (200 OK)**:
```json
{
  "requestId": "REQ-2025-001234",
  "serviceType": "tax_filing",
  "status": "in_progress",
  "progress": 65,
  "currentStep": "verification",
  "steps": [
    {
      "name": "submission",
      "status": "completed",
      "completedAt": "2025-12-26T14:30:00Z"
    },
    {
      "name": "verification",
      "status": "in_progress",
      "estimatedCompletion": "2025-12-27T12:00:00Z"
    },
    {
      "name": "processing",
      "status": "pending"
    },
    {
      "name": "approval",
      "status": "pending"
    }
  ],
  "updates": [
    {
      "timestamp": "2025-12-26T14:30:00Z",
      "message": "Request submitted successfully",
      "type": "info"
    },
    {
      "timestamp": "2025-12-26T15:00:00Z",
      "message": "Documents verified",
      "type": "success"
    }
  ]
}
```

### 5.4 Cancel Service Request

**Endpoint**: `DELETE /services/request/{requestId}`

**Request**:
```json
{
  "reason": "Changed plans, no longer needed",
  "verification": {
    "method": "otp",
    "code": "123456"
  }
}
```

**Response (200 OK)**:
```json
{
  "success": true,
  "message": "Request cancelled successfully",
  "refundAmount": 0,
  "refundMethod": "none"
}
```

## 6. Document Management APIs

### 6.1 Upload Document

**Endpoint**: `POST /documents/upload`

**Request** (multipart/form-data):
```
Content-Type: multipart/form-data
---
file: [binary file data]
type: "tax_form"
subtype: "w2"
metadata: {
  "taxYear": 2024,
  "employer": "Example Corp"
}
encryption: true
```

**Response (201 Created)**:
```json
{
  "success": true,
  "document": {
    "documentId": "DOC-2025-5678",
    "type": "tax_form",
    "size": 2458624,
    "hash": "sha256:abc123...",
    "uploadedAt": "2025-12-26T14:00:00Z",
    "status": "uploaded",
    "virusScan": "clean"
  }
}
```

### 6.2 Get Document

**Endpoint**: `GET /documents/{documentId}`

**Response (200 OK)**:
```json
{
  "documentId": "DOC-2025-5678",
  "type": "certificate",
  "subtype": "birth_certificate",
  "name": "Birth Certificate - Kim MinJun",
  "issuedDate": "2025-12-26",
  "validUntil": "2035-12-26",
  "format": "application/pdf",
  "size": 1048576,
  "downloadUrl": "https://cdn.egov.kr/docs/secure/DOC-2025-5678?token=xyz",
  "qrCodeUrl": "https://verify.egov.kr/doc/DOC-2025-5678"
}
```

### 6.3 Download Document

**Endpoint**: `GET /documents/{documentId}/download`

**Response (200 OK)**:
```
Content-Type: application/pdf
Content-Disposition: attachment; filename="birth_certificate.pdf"
Content-Length: 1048576
X-Document-Hash: sha256:abc123...

[Binary PDF data]
```

### 6.4 Verify Document

**Endpoint**: `GET /documents/{documentId}/verify`

**Response (200 OK)**:
```json
{
  "documentId": "DOC-2025-5678",
  "valid": true,
  "issuedBy": "Seoul Metropolitan Government",
  "issuedDate": "2025-12-26",
  "verificationMethod": "digital_signature",
  "blockchainTx": "0x123abc...",
  "qrCode": "data:image/png;base64,..."
}
```

## 7. Payment APIs

### 7.1 Initiate Payment

**Endpoint**: `POST /payment`

**Request**:
```json
{
  "requestId": "REQ-2025-001234",
  "amount": 50000,
  "currency": "KRW",
  "method": "credit_card",
  "returnUrl": "https://example.com/payment/callback"
}
```

**Response (200 OK)**:
```json
{
  "paymentId": "PAY-2025-7890",
  "amount": 50000,
  "currency": "KRW",
  "status": "pending",
  "paymentUrl": "https://pay.egov.kr/PAY-2025-7890",
  "expiresAt": "2025-12-26T15:00:00Z"
}
```

### 7.2 Get Payment Status

**Endpoint**: `GET /payment/{paymentId}`

**Response (200 OK)**:
```json
{
  "paymentId": "PAY-2025-7890",
  "requestId": "REQ-2025-001234",
  "amount": 50000,
  "currency": "KRW",
  "status": "completed",
  "method": "credit_card",
  "transactionId": "TXN-BANK-123456",
  "paidAt": "2025-12-26T14:45:00Z",
  "receipt": {
    "receiptId": "RCP-2025-4567",
    "downloadUrl": "https://cdn.egov.kr/receipts/RCP-2025-4567.pdf"
  }
}
```

## 8. Notification APIs

### 8.1 WebSocket Connection

**Endpoint**: `WSS /ws`

**Connection**:
```javascript
const ws = new WebSocket('wss://api.egov.kr/v1/ws');
ws.addEventListener('open', () => {
  ws.send(JSON.stringify({
    type: 'auth',
    token: 'jwt-token'
  }));
});
```

**Message Types**:
```json
{
  "type": "service_update",
  "requestId": "REQ-2025-001234",
  "status": "in_progress",
  "message": "Your request is being processed",
  "timestamp": "2025-12-26T15:00:00Z"
}
```

### 8.2 Subscribe to Notifications

**Endpoint**: `POST /notifications/subscribe`

**Request**:
```json
{
  "channels": ["email", "sms", "push"],
  "events": ["service_update", "document_ready", "payment_received"],
  "preferences": {
    "email": "kim.minjun@example.com",
    "phone": "+82-10-1234-5678",
    "pushToken": "fcm-device-token"
  }
}
```

**Response (200 OK)**:
```json
{
  "success": true,
  "subscriptionId": "SUB-2025-1234",
  "channels": ["email", "sms", "push"]
}
```

## 9. Analytics APIs (Public)

### 9.1 Get Public Statistics

**Endpoint**: `GET /analytics/public`

**Response (200 OK)**:
```json
{
  "period": {
    "start": "2025-12-01",
    "end": "2025-12-26"
  },
  "statistics": {
    "totalRequests": 892567,
    "completedRequests": 845234,
    "averageProcessingTime": "P2D18H",
    "citizenSatisfaction": 4.8,
    "topServices": [
      {
        "serviceId": "SRV-TAX-001",
        "name": "Tax Filing",
        "requests": 142567
      },
      {
        "serviceId": "SRV-HEALTH-001",
        "name": "Healthcare",
        "requests": 98234
      }
    ]
  }
}
```

## 10. Error Responses

### 10.1 Error Format

```json
{
  "error": {
    "code": "INVALID_REQUEST",
    "message": "The request is invalid",
    "details": {
      "field": "identityNumber",
      "issue": "Invalid format"
    },
    "requestId": "550e8400-e29b-41d4-a716-446655440000",
    "timestamp": "2025-12-26T15:00:00Z"
  }
}
```

### 10.2 Error Codes

| HTTP Status | Error Code | Description |
|-------------|------------|-------------|
| 400 | INVALID_REQUEST | Malformed request body |
| 401 | UNAUTHORIZED | Missing or invalid authentication |
| 403 | FORBIDDEN | Insufficient permissions |
| 404 | NOT_FOUND | Resource not found |
| 409 | CONFLICT | Resource already exists |
| 429 | RATE_LIMIT_EXCEEDED | Too many requests |
| 500 | INTERNAL_ERROR | Server error |
| 503 | SERVICE_UNAVAILABLE | Service temporarily unavailable |

## 11. Rate Limiting

### 11.1 Limits

| Tier | Requests/Minute | Requests/Day |
|------|-----------------|--------------|
| Free | 60 | 5,000 |
| Standard | 600 | 50,000 |
| Premium | 6,000 | 500,000 |
| Enterprise | Custom | Custom |

### 11.2 Headers

```http
X-RateLimit-Limit: 600
X-RateLimit-Remaining: 543
X-RateLimit-Reset: 1640554800
```

## 12. Pagination

### 12.1 Request

```http
GET /services/available?page=2&limit=50
```

### 12.2 Response

```json
{
  "data": [...],
  "pagination": {
    "page": 2,
    "limit": 50,
    "total": 142,
    "pages": 3,
    "hasNext": true,
    "hasPrevious": true
  }
}
```

---

**弘益人間 · Benefit All Humanity**

© 2025 WIA / SmileStory Inc.


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

## Annex G — Test Vectors and Conformance Evidence

This annex describes how implementations capture and publish conformance
evidence for PHASE-2-API. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-2-api/`. Implementations claiming
  conformance MUST run all vectors in CI and publish the resulting
  pass/fail matrix in their compliance package.
- **Evidence package** — the compliance package is a tarball containing
  the SBOM (CycloneDX 1.5 or SPDX 2.3), the OpenAPI document, the test
  vector matrix, and a signed manifest. Signatures use Sigstore (DSSE
  envelope, Rekor transparency log entry) so that downstream consumers
  can verify provenance without trusting a private CA.
- **Quarterly recheck** — implementations re-publish the evidence package
  every quarter even if no source change occurred, so that consumers can
  detect environmental drift (compiler updates, dependency updates, OS
  updates) without polling vendor changelogs.
- **Cross-vendor crosswalk** — the WIA Standards working group maintains a
  crosswalk that maps each vector to the equivalent assertion in adjacent
  industry programs (where one exists), so an implementer that already
  certifies under one program can show conformance to PHASE-2-API with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-2-API does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-2-API.
It is non-normative; the rules below describe the policy that the WIA
Standards working group commits to when amending this PHASE document.

- **Semantic versioning** — major / minor / patch components follow
  Semantic Versioning 2.0.0 (https://semver.org/spec/v2.0.0.html).
  Major bump indicates a backwards-incompatible change to a normative
  requirement; minor bump indicates new normative requirements that do
  not break existing implementations; patch bump indicates editorial
  changes only (clarifications, typo fixes, formatting).
- **Deprecation window** — when a normative requirement is removed or
  altered in a backwards-incompatible way, the prior major version is
  maintained in parallel for at least 180 days. During the parallel
  window, both major versions are marked Stable in the WIA Standards
  registry and either may be cited as "WIA-conformant".
- **Sunset notification** — deprecated major versions enter a 12-month
  sunset window during which the WIA registry marks the version as
  Deprecated. The deprecation entry includes a migration note pointing
  to the replacement requirement(s) and an explanation of why the
  change was made.
- **Editorial errata** — patch-level errata are issued without a
  deprecation window because they do not change normative behaviour.
  Errata are tracked in a public errata register and each entry is
  signed by the WIA Standards working group chair.
- **Implementation changelog mapping** — implementations SHOULD publish
  a changelog mapping each PHASE version they support to the specific
  build, container digest, or SDK version that satisfies the version.
  This allows downstream auditors to verify version conformance without
  re-running the entire test matrix on every release.

The policy is reviewed at the same cadence as the PHASE document and
any changes to the policy itself are tracked in the version-history
table at the start of the document.

## Annex I — Interoperability Profiles

This annex describes how implementations declare interoperability profiles
for PHASE-2-API. The profile mechanism is non-normative and exists so that
deployments of varying scope (single tenant, regional cluster, federated
network) can advertise the subset of normative requirements they satisfy
without misrepresenting partial conformance as full conformance.

- **Profile manifest** — every implementation publishes a profile manifest
  in JSON. The manifest enumerates the normative requirement IDs from this
  PHASE that are satisfied (`status: "supported"`), partially satisfied
  (`status: "partial"`, with a reason field), or excluded
  (`status: "excluded"`, with a justification). The manifest is signed
  using the same Sigstore key used for the SBOM in Annex G.
- **Federation profile** — federated deployments publish an aggregated
  manifest summarizing the union and intersection of member-implementation
  profiles. The aggregated manifest is consumed by directory services so
  that callers can route a request to the least common denominator profile
  required for an interaction.
- **Backwards-profile compatibility** — when a deployment migrates from one
  profile to a wider profile, the prior profile manifest remains valid and
  signed for the deprecation window defined in Annex H. This preserves
  audit traceability for auditors evaluating long-term interoperability.
- **Profile registry** — the WIA Standards working group maintains a
  public registry of named profiles. Common deployment shapes (e.g.,
  "Edge-only", "Federated-with-replay") are added to the registry by
  consensus. Registry entries are immutable; new shapes are added under
  new names rather than amending existing entries.
- **Profile versioning** — profile names are versioned with the same
  Semantic Versioning rules described in Annex H. A deployment that
  advertises `WIA-P2-API-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.

## Annex J — Reference Implementation Topology

The reference implementation topology described in this annex is
non-normative; it documents the deployment shape that the WIA
Standards working group used to validate the test vectors in Annex G
and is intended as a starting point, not a recommendation against
alternative topologies.

- **Single-tenant edge** — one runtime per organization, no shared
  state. Used for early-pilot deployments where conformance evidence
  is published manually. Sufficient for PHASE-2-API validation when the
  organization signs the manifest itself.
- **Multi-tenant gateway** — one shared runtime serves multiple
  tenants via header-based isolation. Typically backed by a
  rate-limited gateway (Envoy or NGINX) and a shared OAuth 2.1
  identity provider. The manifest is per-tenant; the runtime
  publishes a federation manifest that aggregates tenant manifests.
- **Federated mesh** — multiple runtimes peer to one another and
  publish their manifests to a directory service. Each peer signs
  its own manifest; the directory service signs the aggregated
  index. This is the topology used by cross-organization deployments
  that need to compose conformance.
- **Air-gapped batch** — no network connection between the runtime
  and the directory service. The runtime emits a signed evidence
  package on each batch and the operator transports the package via
  out-of-band channels. This is the topology used by regulators that
  prohibit live connectivity from sensitive environments.

Implementations declare their topology in the manifest (see Annex I).
A topology change MUST be reflected in a new manifest signature; the
prior topology's manifest remains valid for the deprecation window
described in Annex H to preserve audit traceability.
