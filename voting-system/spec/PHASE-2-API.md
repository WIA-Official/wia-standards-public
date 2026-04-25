# WIA-SOC-015: Phase 2 - API Interface Specification

**Version:** 1.0  
**Status:** FINAL  
**Last Updated:** 2025-01-15  
**Standards Body:** World Certification Industry Association (WIA)

---

## 1. Overview

Phase 2 defines RESTful API interfaces enabling communication between voting system components. This specification ensures interoperability while maintaining security, scalability, and accessibility.

### 1.1 Architecture Principles

- RESTful design following HTTP standards
- Stateless communication
- OAuth 2.0 authentication
- JSON request/response bodies
- Comprehensive error handling
- Rate limiting for system protection

---

## 2. Authentication & Authorization

### 2.1 OAuth 2.0 Implementation

All API access requires OAuth 2.0 authentication.

**Supported Flows:**
- Authorization Code (voter-facing applications)
- Client Credentials (server-to-server)
- Device Code (voting terminals)

**Token Endpoint:**
```
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=authorization_code&
code={authorization_code}&
redirect_uri={redirect_uri}&
client_id={client_id}&
client_secret={client_secret}
```

### 2.2 JWT Token Format

Access tokens use JSON Web Token (JWT) format:

```json
{
  "iss": "https://auth.voting.example.com",
  "sub": "voter-12345",
  "aud": "voting-api",
  "exp": 1735689600,
  "iat": 1735686000,
  "scope": "vote.cast ballot.view",
  "wia": {
    "standard": "WIA-SOC-015-v1.0",
    "jurisdiction": "CA-DIST-12",
    "election": "ELECTION-2025-001"
  }
}
```

---

## 3. Core API Endpoints

### 3.1 Voter Registration

#### Register New Voter

```
POST /v1/voters/register
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "personalInfo": {...},
  "address": {...},
  "contactInfo": {...},
  "credentials": {...}
}

Response: 201 Created
{
  "voterId": "VOTER-CA-2025-789456",
  "status": "pending-verification",
  "registrationDate": "2025-01-15T14:30:00Z"
}
```

#### Check Registration Status

```
GET /v1/voters/{voterId}/status
Authorization: Bearer {access_token}

Response: 200 OK
{
  "voterId": "VOTER-CA-2025-789456",
  "status": "active",
  "eligibility": {...}
}
```

### 3.2 Ballot Management

#### Get Voter's Ballot

```
GET /v1/elections/{electionId}/ballots/{voterId}
Authorization: Bearer {access_token}

Response: 200 OK
{
  "ballotId": "BALLOT-2025-001-VOTER-789456",
  "version": "WIA-SOC-015-v1.0",
  "contests": [...],
  "personalizations": {...}
}
```

#### Submit Completed Ballot

```
POST /v1/elections/{electionId}/votes
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "ballotId": "BALLOT-2025-001-VOTER-789456",
  "voterId": "VOTER-CA-2025-789456",
  "votes": [...],
  "metadata": {...},
  "signature": "cryptographic-signature"
}

Response: 201 Created
{
  "voteId": "VOTE-2025-001-987654",
  "status": "accepted",
  "receipt": {...},
  "timestamp": "2025-03-01T14:30:15Z"
}
```

### 3.3 Result Tabulation

#### Get Election Results

```
GET /v1/elections/{electionId}/results?contest={contestId}&level=district&format=json
Authorization: Bearer {access_token}

Response: 200 OK
{
  "electionId": "ELECTION-2025-001",
  "reportTimestamp": "2025-03-01T22:00:00Z",
  "status": "preliminary",
  "completeness": {...},
  "results": [...]
}
```

### 3.4 Audit Trail Access

#### Get Audit Events

```
GET /v1/elections/{electionId}/audit?startDate=2025-03-01&eventType=vote-cast&limit=100
Authorization: Bearer {access_token}

Response: 200 OK
{
  "events": [...],
  "pagination": {...}
}
```

#### Verify Ballot Inclusion

```
GET /v1/verification/{receiptId}

Response: 200 OK
{
  "receiptId": "RECEIPT-2025-XYZ789",
  "verificationCode": "A7F3E9",
  "status": "verified",
  "included": true,
  "blockchainVerification": {...}
}
```

---

## 4. Error Handling

### 4.1 Standard Error Response

```json
{
  "error": {
    "code": "AUTH_INVALID_TOKEN",
    "message": "The provided authentication token is invalid or expired",
    "details": "Token expired at 2025-01-15T10:00:00Z",
    "timestamp": "2025-01-15T10:05:30Z",
    "requestId": "req-abc123def456",
    "documentation": "https://docs.wia-soc-015.com/errors/AUTH_INVALID_TOKEN"
  }
}
```

### 4.2 HTTP Status Codes

| Code | Meaning | Use Case |
|------|---------|----------|
| 200 | OK | Successful GET request |
| 201 | Created | Successful POST creating resource |
| 400 | Bad Request | Invalid request data |
| 401 | Unauthorized | Missing/invalid authentication |
| 403 | Forbidden | Authenticated but not authorized |
| 404 | Not Found | Resource doesn't exist |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Server Error | Server-side failure |

---

## 5. Rate Limiting

### 5.1 Rate Limit Headers

```
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 987
X-RateLimit-Reset: 1735689600
```

### 5.2 Limit Tiers

- **Voter Operations:** 100 requests/hour/voter
- **Poll Worker Operations:** 1,000 requests/hour/worker
- **Election Official Operations:** 10,000 requests/hour
- **Audit Access:** 10,000 requests/hour (public data)

---

## 6. Security Requirements

### 6.1 Transport Security

- HTTPS/TLS 1.3 required for all API communication
- Certificate pinning recommended
- HSTS headers mandatory

### 6.2 Input Validation

- Validate all input parameters
- Sanitize user-provided data
- Enforce maximum request sizes
- Implement SQL injection protection

---

## 7. Conformance Requirements

Systems must implement:
- All required endpoints
- OAuth 2.0 authentication
- Standard error responses
- Rate limiting
- HTTPS/TLS 1.3

---

**© 2025 World Certification Industry Association (WIA)**  
**弘益人間 (Hongik Ingan) - Benefit All Humanity**

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
in lockstep across Phases 1–4 of voting-system so that conformance claims at any
Phase remain unambiguous.*

