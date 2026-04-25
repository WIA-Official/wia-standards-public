# WIA-CORE-002 PHASE 2: API Interface Specification

**Version:** 1.0  
**Status:** Stable  
**Last Updated:** January 2025

## Overview

Phase 2 defines standardized REST and GraphQL APIs for programmatic consent management. These APIs enable systems to create, read, update, delete, and verify consent records in compliance with WIA-CORE-002 standards.

## API Design Principles

1. **RESTful:** Standard HTTP methods and predictable URLs
2. **Secure:** OAuth 2.0 authentication and TLS 1.3 encryption
3. **Versioned:** API versioning through URL path (/api/v1/)
4. **Idempotent:** Safe retry behavior for operations
5. **Documented:** OpenAPI 3.0 specification available
6. **Rate-Limited:** Protection against abuse
7. **Paginated:** Cursor-based pagination for large result sets

## Authentication

All API requests require authentication using OAuth 2.0 or API keys.

### OAuth 2.0 Scopes

- `consents.read` - Read consent records
- `consents.write` - Create and update consent records
- `consents.delete` - Delete and revoke consent records
- `consents.admin` - Administrative operations

### Request Headers

```
Authorization: Bearer {access_token}
Content-Type: application/json
X-WIA-Request-ID: {unique-request-id}
```

## Core Endpoints

### 1. Create Consent

**Endpoint:** `POST /api/v1/consents`

**Request:**
```json
{
  "userId": "user-789012",
  "purposes": [
    {
      "purposeId": "marketing-email",
      "granted": true
    },
    {
      "purposeId": "analytics",
      "granted": true
    }
  ],
  "jurisdiction": "EU",
  "legalBasis": "consent",
  "metadata": {
    "source": "web-signup",
    "ipAddress": "192.0.2.1",
    "consentFormVersion": "2.3"
  }
}
```

**Response (201 Created):**
```json
{
  "consentId": "consent-550e8400-e29b-41d4-a716-446655440000",
  "userId": "user-789012",
  "status": "active",
  "createdAt": "2025-01-15T10:30:00Z",
  "expiresAt": "2026-01-15T10:30:00Z",
  "purposes": [...],
  "_links": {
    "self": "/api/v1/consents/consent-550e8400...",
    "user": "/api/v1/users/user-789012/consents"
  }
}
```

### 2. Retrieve Consent

**Endpoint:** `GET /api/v1/consents/{consentId}`

**Response (200 OK):**
```json
{
  "consentId": "consent-550e8400...",
  "userId": "user-789012",
  "version": "1.0",
  "standard": "WIA-CORE-002",
  "timestamp": "2025-01-15T10:30:00Z",
  "status": "active",
  "purposes": [...],
  "metadata": {...},
  "auditTrail": [...]
}
```

### 3. List User Consents

**Endpoint:** `GET /api/v1/users/{userId}/consents`

**Query Parameters:**
- `status` - Filter by status (active, revoked, expired)
- `purposeId` - Filter by purpose
- `limit` - Number of results (default: 50, max: 100)
- `cursor` - Pagination cursor

**Response (200 OK):**
```json
{
  "data": [
    {
      "consentId": "consent-1...",
      "status": "active",
      "createdAt": "2025-01-15T10:30:00Z"
    },
    {
      "consentId": "consent-2...",
      "status": "active",
      "createdAt": "2025-01-10T14:20:00Z"
    }
  ],
  "pagination": {
    "cursor": "eyJpZCI6MTIzfQ==",
    "hasMore": true,
    "total": 247
  }
}
```

### 4. Update Consent

**Endpoint:** `PATCH /api/v1/consents/{consentId}`

**Request:**
```json
{
  "purposes": [
    {
      "purposeId": "marketing-email",
      "granted": false
    }
  ],
  "metadata": {
    "source": "preference-center"
  }
}
```

**Response (200 OK):**
```json
{
  "consentId": "consent-550e8400...",
  "status": "active",
  "updatedAt": "2025-06-20T14:22:00Z",
  "purposes": [...]
}
```

### 5. Revoke Consent

**Endpoint:** `POST /api/v1/consents/{consentId}/revoke`

**Request:**
```json
{
  "reason": "User requested via preference center",
  "revokeAll": false
}
```

**Response (200 OK):**
```json
{
  "consentId": "consent-550e8400...",
  "status": "revoked",
  "revokedAt": "2025-12-01T09:15:00Z",
  "revokedBy": "user-789012"
}
```

### 6. Verify Consent

**Endpoint:** `POST /api/v1/consents/verify`

**Request:**
```json
{
  "userId": "user-789012",
  "purposeId": "marketing-email",
  "context": {
    "timestamp": "2025-06-20T14:30:00Z",
    "source": "email-campaign-system"
  }
}
```

**Response (200 OK):**
```json
{
  "isValid": true,
  "consentId": "consent-550e8400...",
  "grantedAt": "2025-01-15T10:30:00Z",
  "expiresAt": "2026-01-15T10:30:00Z",
  "purposes": ["marketing-email"],
  "verificationToken": "verify-abc123...",
  "validUntil": "2025-06-20T15:30:00Z"
}
```

## Batch Operations

**Endpoint:** `POST /api/v1/consents/batch`

**Request:**
```json
{
  "operations": [
    {
      "operation": "create",
      "data": {
        "userId": "user-1",
        "purposes": [...]
      }
    },
    {
      "operation": "update",
      "consentId": "consent-abc...",
      "data": {
        "purposes": [...]
      }
    },
    {
      "operation": "revoke",
      "consentId": "consent-xyz..."
    }
  ]
}
```

**Response (200 OK):**
```json
{
  "results": [
    {
      "success": true,
      "consentId": "consent-new...",
      "operation": "create"
    },
    {
      "success": true,
      "consentId": "consent-abc...",
      "operation": "update"
    },
    {
      "success": false,
      "consentId": "consent-xyz...",
      "operation": "revoke",
      "error": {
        "code": "ALREADY_REVOKED",
        "message": "Consent already revoked"
      }
    }
  ],
  "summary": {
    "total": 3,
    "successful": 2,
    "failed": 1
  }
}
```

## Webhooks

**Configuration Endpoint:** `POST /api/v1/webhooks`

**Request:**
```json
{
  "url": "https://your-app.com/webhooks/consent",
  "events": ["consent.created", "consent.updated", "consent.revoked"],
  "secret": "whsec_...",
  "active": true
}
```

**Webhook Payload:**
```json
{
  "eventId": "evt-123...",
  "eventType": "consent.updated",
  "timestamp": "2025-06-20T14:22:00Z",
  "data": {
    "consentId": "consent-550e8400...",
    "userId": "user-789012",
    "changes": {
      "purposes.marketing-email.granted": {
        "old": true,
        "new": false
      }
    }
  }
}
```

**Webhook Verification:**
```
X-WIA-Signature: t=1642521600,v1=sha256_hash
```

## GraphQL Interface

**Endpoint:** `POST /api/v1/graphql`

**Query Example:**
```graphql
query GetUserConsents($userId: ID!) {
  user(id: $userId) {
    consents(status: ACTIVE) {
      consentId
      status
      createdAt
      purposes {
        purposeId
        purposeName
        granted
      }
    }
  }
}
```

**Variables:**
```json
{
  "userId": "user-789012"
}
```

## Error Handling

### Standard Error Response

```json
{
  "error": {
    "code": "INVALID_PURPOSE_ID",
    "message": "The purpose ID 'invalid-id' is not recognized",
    "details": {
      "field": "purposes[0].purposeId",
      "value": "invalid-id",
      "validValues": ["marketing-email", "analytics", ...]
    },
    "requestId": "req-abc123...",
    "timestamp": "2025-06-20T14:30:00Z"
  }
}
```

### Error Codes

- `INVALID_REQUEST` (400) - Malformed request
- `UNAUTHORIZED` (401) - Authentication required
- `FORBIDDEN` (403) - Insufficient permissions
- `NOT_FOUND` (404) - Resource not found
- `CONFLICT` (409) - Resource conflict
- `RATE_LIMIT_EXCEEDED` (429) - Too many requests
- `INTERNAL_ERROR` (500) - Server error

## Rate Limiting

```
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 987
X-RateLimit-Reset: 1642524000
```

## SDK Support

Official SDKs available for:
- TypeScript/JavaScript (`@wia/consent-sdk`)
- Python (`wia-consent-sdk`)
- Java (`com.wia:consent-sdk`)
- Go (`github.com/wia-official/consent-sdk-go`)
- Ruby (`wia-consent-sdk`)
- PHP (`wia/consent-sdk`)
- .NET (`WIA.Consent.SDK`)

---

**Previous:** [PHASE 1: Data Format](PHASE-1-DATA-FORMAT.md)  
**Next:** [PHASE 3: Protocol](PHASE-3-PROTOCOL.md)

© 2025 SmileStory Inc. / WIA · 弘益人間 (Benefit All Humanity)

## P.2 API Surface Cross-References

The API surface defined in this Phase consumes and emits the data formats from
Phase 1 and is transported by the protocol layer in Phase 3. Operators deploy
the surface using the integration patterns in Phase 4.

### P.2.1 Resource Naming

Resource paths follow REST conventions with snake_case segments. Identifier
segments use the canonical UUID encoding from Phase 1.

```
/v1/{collection}                        # collection
/v1/{collection}/{id}                    # member
/v1/{collection}/{id}/{sub_collection}   # nested collection
/v1/{collection}/{id}:{action}           # custom action (POST)
```

### P.2.2 Pagination

List endpoints support cursor-based pagination:

| Param | Default | Max | Description |
|-------|---------|-----|-------------|
| `page_size` | 50 | 500 | Items per page |
| `page_token` | empty | — | Opaque continuation token |

Servers MUST return `next_page_token` when the result set is truncated and an
empty string when the final page has been delivered.

### P.2.3 Idempotency

State-changing operations accept the `Idempotency-Key` header (RFC-style).
Servers MUST cache the response keyed by `(principal, key)` for at least 24 h
and replay the same response on retry.

### P.2.4 Field Masks

Partial-update operations use field masks (Google AIP-161 style) to avoid
clobbering unspecified fields. Masks are dot-paths into the canonical schema
with `*` wildcards.


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

