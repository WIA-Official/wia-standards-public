# WIA-CORE-001: Phase 2 - API Interface Specification

> **Phase:** 2 of 4  
> **Version:** 1.0.0  
> **Status:** Active  
> **Last Updated:** 2025-12-27

---

## Overview

Phase 2 defines standardized APIs for identity operations. These RESTful and GraphQL interfaces enable creating, managing, verifying, and revoking identities and credentials across platforms.

## API Design Principles

1. **RESTful:** HTTP methods match operations (GET, POST, PUT, DELETE)
2. **Stateless:** No server-side session state
3. **Versioned:** Clear API versioning (`/v1/`, `/v2/`)
4. **Authenticated:** Sensitive operations require authentication
5. **Rate Limited:** Prevent abuse through rate limiting
6. **Well-Documented:** OpenAPI 3.0 specifications

## Base URL Structure

```
https://api.wia.org/v1/{resource}
```

## Authentication

All API requests require Bearer token authentication:

```http
Authorization: Bearer eyJhbGciOiJFZERTQSJ9...
```

## Core API Endpoints

### Identity Management

#### POST /identity/create

Create a new universal identity.

**Request:**
```json
{
  "type": "email",
  "attributes": {
    "email": "user@example.com",
    "emailVerified": true,
    "name": "Alice Smith"
  },
  "didMethod": "web",
  "didHost": "example.com"
}
```

**Response (201 Created):**
```json
{
  "id": "did:web:example.com:users:alice",
  "type": "email",
  "trustLevel": 1,
  "attributes": {
    "email": "user@example.com",
    "emailVerified": true,
    "name": "Alice Smith"
  },
  "createdAt": "2025-01-15T19:23:24Z",
  "status": "active"
}
```

#### GET /identity/{did}

Retrieve identity information.

**Response (200 OK):**
```json
{
  "id": "did:web:example.com:users:alice",
  "type": "email",
  "trustLevel": 2,
  "createdAt": "2025-01-15T19:23:24Z",
  "updatedAt": "2025-01-20T10:15:30Z",
  "status": "active"
}
```

#### PUT /identity/{did}

Update identity attributes.

#### DELETE /identity/{did}

Delete identity (requires high trust level and MFA).

### Credential Operations

#### POST /credential/issue

Issue a verifiable credential.

**Request:**
```json
{
  "issuer": "did:web:example.com",
  "subject": "did:web:example.com:users:alice",
  "type": "EmailCredential",
  "claims": {
    "email": "alice@example.com",
    "emailVerified": true
  },
  "validityPeriod": "1y"
}
```

**Response (201 Created):**
```json
{
  "credential": {
    "@context": ["https://www.w3.org/2018/credentials/v1"],
    "id": "urn:uuid:3978344f-8596-4c3a-a978-8fcaba3903c5",
    "type": ["VerifiableCredential", "EmailCredential"],
    "issuer": "did:web:example.com",
    "issuanceDate": "2025-01-15T19:23:24Z",
    "expirationDate": "2026-01-15T19:23:24Z",
    "credentialSubject": {
      "id": "did:web:example.com:users:alice",
      "email": "alice@example.com",
      "emailVerified": true
    },
    "proof": { /* ... */ }
  }
}
```

#### POST /credential/verify

Verify a credential's authenticity.

**Request:**
```json
{
  "credential": { /* VC object */ }
}
```

**Response (200 OK):**
```json
{
  "valid": true,
  "checks": {
    "signatureValid": true,
    "notExpired": true,
    "notRevoked": true,
    "issuerTrusted": true
  },
  "issuer": {
    "did": "did:web:example.com",
    "name": "Example Inc",
    "trustScore": 95
  }
}
```

#### POST /credential/revoke

Revoke a credential.

### DID Operations

#### GET /did/resolve/{did}

Resolve DID to DID Document.

**Response (200 OK):**
```json
{
  "didDocument": {
    "@context": ["https://www.w3.org/ns/did/v1"],
    "id": "did:web:example.com:users:alice",
    "verificationMethod": [/* ... */]
  },
  "didDocumentMetadata": {
    "created": "2025-01-15T19:23:24Z",
    "updated": "2025-01-20T10:15:30Z"
  }
}
```

### Presentation Operations

#### POST /presentation/create

Create a verifiable presentation.

**Request:**
```json
{
  "holder": "did:web:example.com:users:alice",
  "verifier": "did:web:verifier.com",
  "credentials": [
    "urn:uuid:3978344f-8596-4c3a-a978-8fcaba3903c5"
  ],
  "challenge": "1f44d-6f6f61-71-72"
}
```

**Response (200 OK):**
```json
{
  "presentation": {
    "@context": ["https://www.w3.org/2018/credentials/v1"],
    "type": ["VerifiablePresentation"],
    "holder": "did:web:example.com:users:alice",
    "verifiableCredential": [/* ... */],
    "proof": {
      "challenge": "1f44d-6f6f61-71-72",
      /* ... */
    }
  }
}
```

## Error Responses

### Standard Error Format

```json
{
  "error": {
    "code": "INVALID_CREDENTIAL",
    "message": "Credential signature verification failed",
    "details": {
      "issuer": "did:web:example.com",
      "reason": "Public key not found in DID document"
    }
  }
}
```

### HTTP Status Codes

| Code | Meaning | Usage |
|------|---------|-------|
| 200 | OK | Successful request |
| 201 | Created | Resource created successfully |
| 400 | Bad Request | Invalid request format |
| 401 | Unauthorized | Missing or invalid authentication |
| 403 | Forbidden | Insufficient permissions |
| 404 | Not Found | Resource doesn't exist |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Server Error | Server-side error |

## Rate Limiting

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 987
X-RateLimit-Reset: 1642262400
```

## Pagination

```http
GET /credentials?limit=50&offset=100
```

**Response:**
```json
{
  "items": [/* ... */],
  "pagination": {
    "total": 523,
    "limit": 50,
    "offset": 100,
    "hasMore": true
  }
}
```

## GraphQL Alternative

```graphql
query {
  identity(did: "did:web:example.com:users:alice") {
    id
    trustLevel
    attributes {
      email
      emailVerified
    }
    credentials {
      id
      type
      issuanceDate
      expirationDate
    }
  }
}
```

---

**Phase 2 Complete.** Proceed to [Phase 3: Protocol Implementation](PHASE-3-PROTOCOL.md).

**弘益人間 (Benefit All Humanity)**

© 2025 SmileStory Inc. / WIA

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
in lockstep across Phases 1–4 of universal-identity so that conformance claims at any
Phase remain unambiguous.*

