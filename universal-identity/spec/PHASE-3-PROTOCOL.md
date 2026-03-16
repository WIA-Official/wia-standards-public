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
