# WIA-CORE-001: Phase 3 - Protocol Implementation

> **Phase:** 3 of 4  
> **Version:** 1.0.0  
> **Status:** Active  
> **Last Updated:** 2025-12-27

---

## Overview

Phase 3 specifies how authentication and federation protocols integrate with universal identity. This enables users to authenticate across platforms using their universal identity.

## Supported Protocols

1. **OAuth 2.1** - Authorization framework
2. **OpenID Connect (OIDC)** - Authentication layer
3. **SAML 2.0** - Enterprise federation
4. **WebAuthn/FIDO2** - Passwordless authentication
5. **DIDComm** - DID-based messaging

## OAuth 2.1 Integration

### Authorization Code Flow with PKCE

**Step 1: Generate Code Verifier and Challenge**

```python
import hashlib
import base64
import secrets

code_verifier = base64.urlsafe_b64encode(secrets.token_bytes(32)).decode('utf-8')
code_challenge = base64.urlsafe_b64encode(
    hashlib.sha256(code_verifier.encode('utf-8')).digest()
).decode('utf-8').rstrip('=')
```

**Step 2: Authorization Request**

```http
GET /oauth/authorize?
  response_type=code&
  client_id=abc123&
  redirect_uri=https://app.example.com/callback&
  scope=openid profile email&
  state=xyz&
  code_challenge=E9Melhoa2OwvFrEMTJguCHaoeK1t8URWbuGJSstw-cM&
  code_challenge_method=S256
```

**Step 3: Token Exchange**

```http
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=authorization_code&
code=AUTH_CODE&
redirect_uri=https://app.example.com/callback&
client_id=abc123&
code_verifier=CODE_VERIFIER
```

**Response:**
```json
{
  "access_token": "eyJhbGciOiJFZERTQSJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "refresh_token": "def456...",
  "id_token": "eyJhbGciOiJFZERTQSJ9..."
}
```

## OpenID Connect (OIDC)

### Discovery

```http
GET /.well-known/openid-configuration
```

**Response:**
```json
{
  "issuer": "https://accounts.example.com",
  "authorization_endpoint": "https://accounts.example.com/oauth/authorize",
  "token_endpoint": "https://accounts.example.com/oauth/token",
  "userinfo_endpoint": "https://accounts.example.com/oauth/userinfo",
  "jwks_uri": "https://accounts.example.com/.well-known/jwks.json",
  "response_types_supported": ["code"],
  "subject_types_supported": ["public"],
  "id_token_signing_alg_values_supported": ["ES256", "EdDSA"]
}
```

### ID Token Structure

```json
{
  "iss": "https://accounts.example.com",
  "sub": "did:web:example.com:users:alice",
  "aud": "abc123",
  "exp": 1735689600,
  "iat": 1735686000,
  "nonce": "n-0S6_WzA2Mj",
  "email": "alice@example.com",
  "email_verified": true,
  "name": "Alice Smith",
  "trust_level": 2
}
```

### UserInfo Endpoint

```http
GET /oauth/userinfo
Authorization: Bearer ACCESS_TOKEN
```

**Response:**
```json
{
  "sub": "did:web:example.com:users:alice",
  "email": "alice@example.com",
  "email_verified": true,
  "name": "Alice Smith",
  "trust_level": 2,
  "credentials": [
    {
      "type": "EmailCredential",
      "issuedBy": "did:web:example.com",
      "issuedAt": "2025-01-15T19:23:24Z"
    }
  ]
}
```

## SAML 2.0 Integration

### Service Provider Initiated Flow

**AuthnRequest:**
```xml
<samlp:AuthnRequest
    xmlns:samlp="urn:oasis:names:tc:SAML:2.0:protocol"
    ID="_8e8dc5f69a98cc4c1ff3427e5ce34606fd672f91e6"
    Version="2.0"
    IssueInstant="2025-01-15T19:23:24Z"
    Destination="https://idp.example.com/saml/sso"
    AssertionConsumerServiceURL="https://sp.example.com/saml/acs">
    
    <saml:Issuer>https://sp.example.com/saml/metadata</saml:Issuer>
    
    <samlp:NameIDPolicy
        Format="urn:oasis:names:tc:SAML:2.0:nameid-format:persistent"
        AllowCreate="true"/>
</samlp:AuthnRequest>
```

**SAML Assertion with DID:**
```xml
<saml:Assertion Version="2.0">
    <saml:Subject>
        <saml:NameID Format="urn:oasis:names:tc:SAML:2.0:nameid-format:persistent">
            did:web:example.com:users:alice
        </saml:NameID>
    </saml:Subject>
    
    <saml:AttributeStatement>
        <saml:Attribute Name="trustLevel">
            <saml:AttributeValue>2</saml:AttributeValue>
        </saml:Attribute>
        <saml:Attribute Name="email">
            <saml:AttributeValue>alice@example.com</saml:AttributeValue>
        </saml:Attribute>
    </saml:AttributeStatement>
</saml:Assertion>
```

## WebAuthn/FIDO2

### Registration Flow

**Server generates challenge:**
```json
{
  "challenge": "VGhpcyBpcyBhIGNoYWxsZW5nZQ",
  "rp": {
    "name": "Example Corp",
    "id": "example.com"
  },
  "user": {
    "id": "ZGlkOndlYjpleGFtcGxlLmNvbTp1c2VyczphbGljZQ",
    "name": "alice@example.com",
    "displayName": "Alice Smith"
  },
  "pubKeyCredParams": [
    {"alg": -7, "type": "public-key"},
    {"alg": -257, "type": "public-key"}
  ],
  "authenticatorSelection": {
    "authenticatorAttachment": "platform",
    "userVerification": "required"
  }
}
```

**Client creates credential:**
```javascript
const credential = await navigator.credentials.create({
  publicKey: options
});
```

### Authentication Flow

**Server challenge:**
```json
{
  "challenge": "VGhpcyBpcyBhIGNoYWxsZW5nZQ",
  "allowCredentials": [{
    "id": "CREDENTIAL_ID",
    "type": "public-key"
  }],
  "userVerification": "required"
}
```

**Client authenticates:**
```javascript
const assertion = await navigator.credentials.get({
  publicKey: options
});
```

## DIDComm Messaging

### Message Structure

```json
{
  "type": "https://didcomm.org/authentication/1.0/challenge",
  "id": "1234567890",
  "from": "did:web:verifier.com",
  "to": ["did:web:example.com:users:alice"],
  "created_time": 1735686000,
  "body": {
    "challenge": "1f44d-6f6f61-71-72",
    "requested_attributes": ["email", "trust_level"]
  }
}
```

### Response Message

```json
{
  "type": "https://didcomm.org/authentication/1.0/response",
  "id": "0987654321",
  "from": "did:web:example.com:users:alice",
  "to": ["did:web:verifier.com"],
  "thid": "1234567890",
  "created_time": 1735686010,
  "body": {
    "presentation": { /* Verifiable Presentation */ }
  }
}
```

## Trust Level Requirements

| Operation | Min Trust Level | Protocol |
|-----------|----------------|----------|
| Public content | 0 | Any |
| Basic login | 1 | OIDC |
| Account changes | 2 | OIDC + MFA |
| Financial ($1K-10K) | 3 | OIDC + Gov ID |
| Financial (>$10K) | 4 | WebAuthn |
| Legal contracts | 5 | Digital Signature |

---

**Phase 3 Complete.** Proceed to [Phase 4: Platform Integration](PHASE-4-INTEGRATION.md).

**弘益人間 (Benefit All Humanity)**

© 2025 SmileStory Inc. / WIA

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
in lockstep across Phases 1–4 of universal-identity so that conformance claims at any
Phase remain unambiguous.*

