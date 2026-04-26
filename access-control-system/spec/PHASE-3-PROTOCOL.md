# WIA Access Control System - Phase 3: Protocol Specification
## Version 1.0

### Document Information
- **Standard:** WIA-ACS (World Industry Association - Access Control System)
- **Phase:** 3 (Protocol)
- **Version:** 1.0
- **Status:** Approved
- **Date:** 2025-12-26
- **Philosophy:** 弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. Introduction

Phase 3 establishes secure communication protocols ensuring end-to-end encryption, authentication, and integrity for all WIA-ACS operations. This phase covers TLS 1.3, OAuth 2.0, OpenID Connect, SAML 2.0, and certificate management.

---

## 2. Transport Layer Security (TLS 1.3)

### 2.1 Requirements

All WIA-ACS communications MUST use **TLS 1.3** or later (RFC 8446).

### 2.2 Cipher Suites

Required cipher suites (in order of preference):
1. `TLS_AES_256_GCM_SHA384`
2. `TLS_CHACHA20_POLY1305_SHA256`
3. `TLS_AES_128_GCM_SHA256`

Deprecated cipher suites (MUST NOT use):
- Any TLS 1.2 or earlier cipher
- RSA key exchange
- SHA-1, MD5 hashes
- NULL, RC4, DES ciphers

### 2.3 TLS Configuration

```nginx
server {
    listen 443 ssl http2;
    ssl_protocols TLSv1.3;
    ssl_ciphers 'TLS_AES_256_GCM_SHA384:TLS_CHACHA20_POLY1305_SHA256:TLS_AES_128_GCM_SHA256';
    ssl_prefer_server_ciphers on;
    ssl_session_timeout 1d;
    ssl_session_cache shared:SSL:50m;
    ssl_stapling on;
    ssl_stapling_verify on;
}
```

### 2.4 Certificate Requirements

- **Algorithm:** RSA 4096-bit minimum, or ECDSA P-384/P-521
- **Signature:** SHA-256 or stronger (SHA-384, SHA-512)
- **Validity:** Maximum 397 days (13 months)
- **Key Usage:** Digital Signature, Key Encipherment
- **Extended Key Usage:** Server Authentication, Client Authentication
- **SAN:** All DNS names and IPs covered
- **CT:** Signed Certificate Timestamp required

---

## 3. OAuth 2.0 Authorization Framework

### 3.1 Supported Grant Types

| Grant Type | Use Case | Security Level |
|------------|----------|----------------|
| Authorization Code + PKCE | Web/mobile apps | High |
| Client Credentials | Server-to-server | High |
| Device Code | IoT devices, kiosks | Medium |
| Refresh Token | Long-lived sessions | High |

### 3.2 Authorization Code Flow with PKCE

**Step 1: Generate code verifier and challenge**
```
code_verifier = BASE64URL(RANDOM(32 bytes))
code_challenge = BASE64URL(SHA256(code_verifier))
```

**Step 2: Authorization request**
```
GET /oauth/authorize
  ?response_type=code
  &client_id=wia-acs-app-001
  &redirect_uri=https://app.example.com/callback
  &scope=read:credentials+write:credentials
  &state=random_state_xyz123
  &code_challenge=CHALLENGE
  &code_challenge_method=S256
```

**Step 3: Token exchange**
```
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=authorization_code
&code=AUTH_CODE
&redirect_uri=https://app.example.com/callback
&client_id=wia-acs-app-001
&code_verifier=VERIFIER
```

**Step 4: Response**
```json
{
  "access_token": "eyJhbGci...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "refresh_token": "rt_abc123",
  "scope": "read:credentials write:credentials"
}
```

### 3.3 JWT Access Token

**Header:**
```json
{
  "alg": "RS256",
  "typ": "JWT",
  "kid": "wia-acs-key-2025-001"
}
```

**Payload:**
```json
{
  "iss": "https://auth.example.com",
  "sub": "usr-001",
  "aud": ["https://api.example.com"],
  "exp": 1735228800,
  "iat": 1735225200,
  "scope": "read:credentials write:credentials",
  "roles": ["role-employee"]
}
```

**Signature:**
```
RS256(
  base64UrlEncode(header) + "." + base64UrlEncode(payload),
  privateKey
)
```

### 3.4 Token Validation

All implementations MUST:
1. Verify signature using public key (JWK or PEM)
2. Check expiration (`exp` claim)
3. Validate issuer (`iss` claim)
4. Verify audience (`aud` claim)
5. Check not-before time (`nbf` claim)
6. Validate scopes match request
7. Verify token not revoked (check revocation list)

---

## 4. OpenID Connect (OIDC)

### 4.1 OIDC Authentication Flow

Extends OAuth 2.0 by adding `openid` scope.

**Authorization request:**
```
GET /oauth/authorize
  ?response_type=code
  &client_id=wia-acs-app-001
  &redirect_uri=https://app.example.com/callback
  &scope=openid+profile+email
  &state=xyz123
  &nonce=abc789
```

**Token response includes ID token:**
```json
{
  "access_token": "...",
  "id_token": "eyJhbGci...",
  "token_type": "Bearer",
  "expires_in": 3600
}
```

### 4.2 ID Token

```json
{
  "iss": "https://auth.example.com",
  "sub": "usr-001",
  "aud": "wia-acs-app-001",
  "exp": 1735228800,
  "iat": 1735225200,
  "nonce": "abc789",
  "auth_time": 1735225150,
  "acr": "urn:mace:incommon:iap:silver",
  "amr": ["pwd", "otp"],
  "name": "John Doe",
  "email": "john.doe@example.com",
  "email_verified": true
}
```

### 4.3 UserInfo Endpoint

```
GET /oauth/userinfo
Authorization: Bearer ACCESS_TOKEN

Response:
{
  "sub": "usr-001",
  "name": "John Doe",
  "email": "john.doe@example.com",
  "department": "Engineering"
}
```

---

## 5. SAML 2.0

### 5.1 SAML Entities

- **Service Provider (SP):** WIA-ACS application
- **Identity Provider (IdP):** Enterprise SSO (Azure AD, Okta, etc.)

### 5.2 SP-Initiated Flow

**Step 1: Generate AuthnRequest**
```xml
<samlp:AuthnRequest
    ID="id-abc123"
    Version="2.0"
    IssueInstant="2025-12-26T14:32:17Z"
    Destination="https://idp.example.com/saml/sso"
    AssertionConsumerServiceURL="https://acs.example.com/saml/acs">
    <saml:Issuer>https://acs.example.com</saml:Issuer>
    <samlp:NameIDPolicy
        Format="urn:oasis:names:tc:SAML:2.0:nameid-format:persistent"
        AllowCreate="true"/>
</samlp:AuthnRequest>
```

**Step 2: Redirect to IdP**
```
HTTP 302 https://idp.example.com/saml/sso
?SAMLRequest=BASE64(DEFLATE(AuthnRequest))
&RelayState=original-url-state
```

**Step 3: IdP authenticates user and generates Response**

**Step 4: POST Response to SP**
```xml
<samlp:Response ID="response-xyz" IssueInstant="2025-12-26T14:33:00Z">
    <saml:Issuer>https://idp.example.com</saml:Issuer>
    <samlp:Status>
        <samlp:StatusCode Value="urn:oasis:names:tc:SAML:2.0:status:Success"/>
    </samlp:Status>
    <saml:Assertion>
        <saml:Subject>
            <saml:NameID>john.doe@example.com</saml:NameID>
        </saml:Subject>
        <saml:AttributeStatement>
            <saml:Attribute Name="email">
                <saml:AttributeValue>john.doe@example.com</saml:AttributeValue>
            </saml:Attribute>
        </saml:AttributeStatement>
    </saml:Assertion>
</samlp:Response>
```

### 5.3 SAML Security

All implementations MUST:
- Validate assertion signature (RSA-SHA256 minimum)
- Verify certificate chain and expiration
- Check `NotBefore` and `NotOnOrAfter` timestamps
- Validate audience restriction
- Prevent replay attacks (track assertion IDs)
- Use encrypted assertions for sensitive data

---

## 6. Certificate Management

### 6.1 PKI Hierarchy

```
Root CA (Offline)
├── Validity: 20 years
├── Key: RSA 4096 or ECDSA P-521
└── Purpose: Sign intermediate CAs only

    └── Intermediate CA (Online, HSM)
        ├── Validity: 10 years
        ├── Key: RSA 4096 or ECDSA P-384
        └── Purpose: Sign end-entity certificates

            ├── TLS Server Certificates (397 days)
            ├── TLS Client Certificates (2 years)
            ├── Code Signing Certificates (3 years)
            └── Smart Card Certificates (3 years)
```

### 6.2 ACME Protocol

Support Let's Encrypt-compatible ACME for automated certificate issuance and renewal.

```
POST /acme/new-order
{
  "identifiers": [
    {"type": "dns", "value": "api.example.com"}
  ]
}
```

### 6.3 Certificate Renewal

- Auto-renewal triggered at 30 days before expiry
- Gradual rollout to servers (blue-green deployment)
- Old certificate deprecated 7 days before expiry
- Monitor expiration with automated alerts

---

## 7. Security Headers

All WIA-ACS HTTP responses MUST include:

```
Strict-Transport-Security: max-age=31536000; includeSubDomains; preload
Content-Security-Policy: default-src 'self'
X-Frame-Options: DENY
X-Content-Type-Options: nosniff
X-XSS-Protection: 1; mode=block
Referrer-Policy: strict-origin-when-cross-origin
```

---

## 8. Compliance Checklist

Phase 3 Protocol compliance requires:
- [ ] TLS 1.3 mandatory for all communications
- [ ] Approved cipher suites only
- [ ] OAuth 2.0 with PKCE implemented
- [ ] OpenID Connect support
- [ ] SAML 2.0 support for enterprise SSO
- [ ] Automated certificate management
- [ ] Security headers on all responses
- [ ] JWT token validation
- [ ] Certificate pinning for mobile apps

---

**Document Control**
- Author: WIA Technical Committee
- Approved: 2025-12-26
- Next Review: 2026-12-26
- License: CC BY 4.0

© 2025 SmileStory Inc. / WIA
弘益人間 - Benefit All Humanity


## Annex E — Implementation Notes for PHASE-3-PROTOCOL

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-3-PROTOCOL.

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
