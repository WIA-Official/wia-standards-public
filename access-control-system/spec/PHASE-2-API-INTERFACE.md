# WIA Access Control System - Phase 2: API Interface Specification
## Version 1.0

### Document Information
- **Standard:** WIA-ACS (World Industry Association - Access Control System)
- **Phase:** 2 (API Interface)
- **Version:** 1.0
- **Status:** Approved
- **Date:** 2025-12-26
- **Philosophy:** 弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. Introduction

Phase 2 defines comprehensive RESTful and gRPC APIs for programmatic access to all WIA-ACS functions, enabling automation, third-party integrations, and custom application development.

### 1.1 Scope

- RESTful API endpoints
- gRPC service definitions
- Authentication mechanisms
- Rate limiting
- Error handling
- SDK libraries

---

## 2. API Design Principles

### 2.1 RESTful Architecture

- **Resource-oriented URLs:** `/v1/users`, `/v1/credentials`
- **HTTP verbs:** GET (read), POST (create), PUT/PATCH (update), DELETE (delete)
- **Stateless:** Each request contains all necessary context
- **Idempotent:** Safe retries for PUT, DELETE operations

### 2.2 Versioning

URL-based versioning:
```
https://api.example.com/v1/users
https://api.example.com/v2/users
```

Version lifecycle:
- Minor versions (1.0 → 1.1): Backward compatible, 24-month support
- Major versions (1.x → 2.x): Breaking changes, 12-month deprecation period

---

## 3. Authentication API

### 3.1 POST /v1/authenticate

Authenticate a user and return access token.

**Request:**
```json
{
  "credential_id": "cred-xyz789",
  "factors": [
    {"type": "pin", "value": "1234"}
  ],
  "device_info": {
    "device_id": "reader-001",
    "ip_address": "192.168.1.45"
  }
}
```

**Response (200 OK):**
```json
{
  "access_token": "eyJhbGci...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "refresh_token": "rt_abc123",
  "user": {
    "user_id": "usr-001",
    "name": "John Doe",
    "roles": ["role-employee"]
  }
}
```

**Response (401 - MFA Required):**
```json
{
  "error": {
    "code": "MFA_REQUIRED",
    "mfa_token": "mfa_tmp_xyz",
    "available_methods": ["totp", "sms"],
    "expires_in": 300
  }
}
```

### 3.2 POST /v1/authenticate/mfa

Complete multi-factor authentication.

**Request:**
```json
{
  "mfa_token": "mfa_tmp_xyz",
  "method": "totp",
  "code": "123456"
}
```

### 3.3 POST /v1/token/refresh

Exchange refresh token for new access token.

---

## 4. Authorization API

### 4.1 POST /v1/authorize

Evaluate access request against policies.

**Request:**
```json
{
  "user_id": "usr-001",
  "resource": {
    "type": "door",
    "id": "door-server-room-a",
    "attributes": {"location": "hq", "security_zone": "high"}
  },
  "action": "entry",
  "context": {
    "timestamp": "2025-12-26T14:32:17Z",
    "ip_address": "192.168.1.45",
    "mfa_verified": true
  }
}
```

**Response (200 - Permit):**
```json
{
  "decision": "permit",
  "reason": "User has required role and meets conditions",
  "matched_policies": ["policy-it-server-access"],
  "valid_until": "2025-12-26T22:00:00Z"
}
```

**Response (200 - Deny):**
```json
{
  "decision": "deny",
  "reason": "Access outside allowed time window",
  "requirements": {
    "time": "Monday-Friday 06:00-22:00"
  }
}
```

---

## 5. User Management API

| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | /v1/users | List users (paginated) |
| POST | /v1/users | Create user |
| GET | /v1/users/{id} | Get user details |
| PATCH | /v1/users/{id} | Update user |
| DELETE | /v1/users/{id} | Delete user |
| GET | /v1/users/{id}/roles | Get user's roles |
| POST | /v1/users/{id}/roles | Assign role |
| DELETE | /v1/users/{id}/roles/{roleId} | Remove role |

### 5.1 GET /v1/users (List with Pagination)

**Query Parameters:**
- `status` - Filter by status (active, suspended, terminated)
- `department` - Filter by department
- `page` - Page number (default: 1)
- `per_page` - Items per page (default: 20, max: 100)
- `sort` - Sort field (name, created_at, etc.)

**Response:**
```json
{
  "users": [...],
  "pagination": {
    "current_page": 1,
    "per_page": 20,
    "total_pages": 5,
    "total_items": 97,
    "has_next": true
  },
  "links": {
    "self": "/v1/users?page=1",
    "next": "/v1/users?page=2",
    "last": "/v1/users?page=5"
  }
}
```

---

## 6. Credential Management API

### 6.1 POST /v1/credentials

Issue new credential.

**Request:**
```json
{
  "user_id": "usr-001",
  "credential_type": "smart_card",
  "issued_at": "2025-12-26T10:00:00Z",
  "expires_at": "2026-12-26T10:00:00Z",
  "encoding": {
    "format": "piv",
    "facility_code": 123,
    "card_number": 45678
  },
  "access_profiles": ["profile-building-entry"],
  "factors": [
    {"factor_type": "something_you_have", "method": "smart_card"},
    {"factor_type": "something_you_know", "method": "pin", "pin": "1234"}
  ]
}
```

**Response (201 Created):**
```json
{
  "credential_id": "cred-xyz789",
  "user_id": "usr-001",
  "status": "active",
  "encoding": {
    "format": "piv",
    "facility_code": 123,
    "card_number": 45678,
    "chip_uid": "04:3F:2A:B5:C8:19:80"
  },
  "qr_code": "data:image/png;base64,...",
  "provisioning_url": "https://acs.example.com/provision/cred-xyz789"
}
```

### 6.2 DELETE /v1/credentials/{id}

Revoke credential.

**Query Parameters:**
- `reason` - Revocation reason (lost_stolen, user_terminated, etc.)

---

## 7. Audit API

### 7.1 GET /v1/audit/events

Query audit events.

**Query Parameters:**
- `start_date` - Start of date range (ISO 8601)
- `end_date` - End of date range
- `event_type` - Filter by event type
- `user_id` - Filter by user
- `resource_type` - Filter by resource type
- `result` - Filter by result (success, failure)
- `page`, `per_page` - Pagination

---

## 8. Webhooks

### 8.1 POST /v1/webhooks

Create webhook subscription.

**Request:**
```json
{
  "url": "https://example.com/webhook",
  "events": ["access_granted", "access_denied", "credential_revoked"],
  "secret": "whsec_xyz123",
  "active": true,
  "filters": {
    "resource_type": "door",
    "location": "headquarters"
  }
}
```

### 8.2 Webhook Delivery

WIA-ACS will POST to subscribed URL:

**Headers:**
```
Content-Type: application/json
X-WIA-ACS-Event: access_granted
X-WIA-ACS-Signature: sha256=abc123...
X-WIA-ACS-Delivery: delivery-xyz
```

**Body:**
```json
{
  "webhook_id": "webhook-001",
  "event": {
    "event_id": "evt-123",
    "event_type": "access_granted",
    "timestamp": "2025-12-26T14:32:17.234Z",
    ...
  }
}
```

---

## 9. Error Handling

### 9.1 Standard Error Response

```json
{
  "error": {
    "code": "INVALID_CREDENTIAL_FORMAT",
    "message": "Credential format is invalid",
    "details": "Field 'expires_at' must be after 'issued_at'",
    "field": "expires_at",
    "request_id": "req-xyz",
    "documentation_url": "https://docs.wia-acs.org/errors/INVALID_CREDENTIAL_FORMAT"
  }
}
```

### 9.2 HTTP Status Codes

| Code | Meaning | Usage |
|------|---------|-------|
| 200 | OK | Successful GET request |
| 201 | Created | Successful POST creating resource |
| 204 | No Content | Successful DELETE |
| 400 | Bad Request | Invalid input data |
| 401 | Unauthorized | Missing/invalid authentication |
| 403 | Forbidden | Authenticated but not authorized |
| 404 | Not Found | Resource doesn't exist |
| 409 | Conflict | Resource already exists |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Server Error | Server-side failure |

---

## 10. Rate Limiting

### 10.1 Rate Limit Tiers

| Tier | Rate Limit | Burst | Use Case |
|------|------------|-------|----------|
| Basic | 1,000 req/hour | 20 req/min | Small deployments |
| Standard | 10,000 req/hour | 100 req/min | Medium enterprises |
| Premium | 100,000 req/hour | 500 req/min | Large enterprises |
| Enterprise | Custom | Custom | Global deployments |

### 10.2 Rate Limit Headers

```
X-RateLimit-Limit: 10000
X-RateLimit-Remaining: 9847
X-RateLimit-Reset: 1735225200
```

On limit exceeded (429):
```
Retry-After: 3600
```

---

## 11. gRPC API

For high-performance scenarios, gRPC APIs are provided alongside REST.

### 11.1 Protocol Buffer Definition

```protobuf
service AuthenticationService {
  rpc Authenticate(AuthenticateRequest) returns (AuthenticateResponse);
  rpc AuthenticateMFA(MFARequest) returns (AuthenticateResponse);
  rpc RefreshToken(RefreshTokenRequest) returns (AuthenticateResponse);
}

message AuthenticateRequest {
  string credential_id = 1;
  repeated AuthFactor factors = 2;
  DeviceInfo device_info = 3;
}

message AuthenticateResponse {
  string access_token = 1;
  string token_type = 2;
  int32 expires_in = 3;
  User user = 4;
  bool mfa_required = 5;
}
```

### 11.2 Benefits

- 5-10x faster than REST for high-frequency operations
- Binary protocol reduces bandwidth
- Built-in streaming support
- Strong typing with Protocol Buffers
- Automatic client generation in 10+ languages

---

## 12. SDK Libraries

Official SDKs provided in:
- TypeScript/JavaScript (@wia/acs-sdk)
- Python (wia-acs)
- Go (github.com/wia-official/wia-acs-go)
- Java (com.wia:acs-sdk)
- C# (WIA.ACS.SDK)

### 12.1 TypeScript Example

```typescript
import { WiaAcsClient } from '@wia/acs-sdk';

const client = new WiaAcsClient({
  apiUrl: 'https://api.example.com',
  apiKey: 'your-api-key'
});

const result = await client.authenticate({
  credentialId: 'cred-xyz',
  factors: [{ type: 'pin', value: '1234' }]
});

if (result.success) {
  console.log('Access granted!', result.accessToken);
}
```

---

## 13. Compliance Checklist

Phase 2 API Interface compliance requires:
- [ ] All required endpoints implemented
- [ ] RESTful design principles followed
- [ ] Pagination for list endpoints
- [ ] Standard error responses
- [ ] Rate limiting implemented
- [ ] Webhook support
- [ ] SDK in at least one language
- [ ] API documentation published (OpenAPI 3.0)

---

**Document Control**
- Author: WIA Technical Committee
- Approved: 2025-12-26
- Next Review: 2026-12-26
- License: CC BY 4.0

© 2025 SmileStory Inc. / WIA
弘益人間 - Benefit All Humanity


## Annex E — Implementation Notes for PHASE-2-API-INTERFACE

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-2-API-INTERFACE.

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
