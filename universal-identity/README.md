# 🪪 WIA-CORE-001: Universal Identity Standard

[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)]()
[![License](https://img.shields.io/badge/license-MIT-green.svg)]()
[![Standard](https://img.shields.io/badge/WIA-Standard-purple.svg)]()


> **Standard ID:** WIA-CORE-001
> **Version:** 1.0.0
> **Status:** Active
> **Category:** CORE (범용 통합 표준)
> **Color:** Indigo (#6366F1)

---

## 🌟 Overview

The WIA-CORE-001 standard defines a universal identity framework for seamless identity federation, authentication, and authorization across heterogeneous systems and platforms.

**홍익인간 (弘益人間) - Benefit All Humanity** - This standard aims to empower individuals with portable, privacy-preserving digital identities that work across all platforms, services, and borders, ensuring universal access while protecting personal sovereignty.

## 🎯 Key Features

- **Universal Identity**: Single portable identity across all platforms and services
- **Federated Authentication**: SSO (Single Sign-On) with multiple identity providers
- **Privacy-First Design**: User-controlled data sharing with granular consent management
- **Decentralized Architecture**: No single point of failure or control
- **Verifiable Credentials**: Cryptographically-signed identity attributes and claims
- **Cross-Platform Interoperability**: Works with OAuth, SAML, OpenID Connect, DID, and more
- **Progressive Trust**: Context-aware authentication levels from anonymous to verified
- **Sovereign Identity**: Users own and control their identity data

## 📊 Core Concepts

### 1. Identity Layers

```
Identity Stack:
┌─────────────────────────────────────────┐
│ Layer 5: Verifiable Credentials        │  (Degrees, licenses, certificates)
├─────────────────────────────────────────┤
│ Layer 4: Verified Attributes           │  (KYC, age, nationality)
├─────────────────────────────────────────┤
│ Layer 3: Authenticated Profile         │  (Email, phone, social profiles)
├─────────────────────────────────────────┤
│ Layer 2: Pseudonymous Identity         │  (Username, handle, alias)
├─────────────────────────────────────────┤
│ Layer 1: Anonymous Access               │  (Session token, temporary ID)
└─────────────────────────────────────────┘
```

### 2. Trust Levels

```typescript
Trust Levels:
- Level 0: Anonymous (No verification)
- Level 1: Email Verified (Basic verification)
- Level 2: Multi-Factor (Enhanced security)
- Level 3: Identity Verified (Government ID checked)
- Level 4: Biometric Verified (Biometric authentication)
- Level 5: Legally Bound (Digital signature with legal standing)
```

### 3. Identity Federation

```
Federation Flow:
User Identity → Identity Provider → Service Provider → Resource Access

Supported Protocols:
✓ OAuth 2.0 / OAuth 2.1
✓ OpenID Connect (OIDC)
✓ SAML 2.0
✓ WebAuthn / FIDO2
✓ Decentralized Identifiers (DIDs)
✓ Verifiable Credentials (VCs)
```

## 🔧 Components

### TypeScript SDK

```typescript
import {
  UniversalIdentity,
  IdentityProvider,
  createIdentity,
  federateIdentity,
  verifyCredential
} from '@wia/core-001';

// Create a universal identity
const identity = await createIdentity({
  type: 'did',
  method: 'web',
  attributes: {
    email: 'user@example.com',
    displayName: 'Alice Smith'
  }
});

console.log(identity.did); // did:web:example.com:users:alice

// Federate with external identity provider
const federated = await federateIdentity({
  identity: identity.did,
  provider: 'https://accounts.google.com',
  protocol: 'oidc',
  scope: ['openid', 'profile', 'email']
});

// Issue verifiable credential
const credential = await identity.issueCredential({
  type: 'EmailCredential',
  claims: {
    email: 'user@example.com',
    verified: true,
    verifiedAt: new Date()
  },
  expiresIn: '1y'
});

// Verify credential
const verification = await verifyCredential(credential);
console.log(verification.valid); // true
console.log(verification.issuer); // did:web:example.com
```

### CLI Tool

```bash
# Create new universal identity
wia-core-001 create-identity --type did --name "Alice Smith"

# Link external provider
wia-core-001 link-provider --provider google --protocol oidc

# Issue credential
wia-core-001 issue-credential --type email --value "user@example.com"

# Verify credential
wia-core-001 verify-credential --credential ./email-credential.json

# Export identity
wia-core-001 export --format json --output ./my-identity.json

# Authenticate to service
wia-core-001 authenticate --service https://example.com --method oidc
```

## 📚 Documentation

| Document | Description |
|----------|-------------|
| [WIA-CORE-001-v1.0.md](./spec/WIA-CORE-001-v1.0.md) | Complete specification with technical details |
| [TypeScript SDK](./api/typescript/) | Full TypeScript implementation |
| [CLI Tool](./cli/wia-core-001.sh) | Command-line interface |

## 🚀 Quick Start

### Installation

```bash
# Clone repository
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/standards/universal-identity

# Run installation script
./install.sh

# Verify installation
wia-core-001 --version
```

### TypeScript Usage

```bash
# Install via npm
npm install @wia/core-001

# Or yarn
yarn add @wia/core-001
```

```typescript
import { UniversalIdentitySDK } from '@wia/core-001';

const sdk = new UniversalIdentitySDK();

// Create identity
const identity = await sdk.createIdentity({
  type: 'did:web',
  attributes: {
    name: 'Alice Smith',
    email: 'alice@example.com'
  }
});

// Authenticate with federated provider
const auth = await sdk.authenticate({
  identity: identity.did,
  provider: 'https://accounts.google.com',
  protocol: 'oidc',
  redirectUri: 'https://myapp.com/callback'
});

console.log(`Authentication URL: ${auth.authorizationUrl}`);

// After callback, exchange code for tokens
const tokens = await sdk.exchangeAuthCode({
  code: auth.code,
  state: auth.state
});

console.log(`Access Token: ${tokens.accessToken}`);
console.log(`ID Token: ${tokens.idToken}`);
```

## 🔐 Identity Types

| Type | Protocol | Use Case | Trust Level |
|------|----------|----------|-------------|
| Anonymous | Session Token | Public access | Level 0 |
| Email | OIDC | Basic services | Level 1 |
| Social | OAuth 2.0 | Social platforms | Level 2 |
| Enterprise | SAML 2.0 | Corporate systems | Level 3 |
| Government | eID | Government services | Level 4 |
| Decentralized | DID/VC | Web3, blockchain | Level 3-5 |

## 🌐 Federation Protocols

### OAuth 2.0 / 2.1

```typescript
const oauth = sdk.createOAuthProvider({
  clientId: 'your-client-id',
  clientSecret: 'your-client-secret',
  authorizationUrl: 'https://provider.com/oauth/authorize',
  tokenUrl: 'https://provider.com/oauth/token',
  scope: ['read', 'write']
});

const authUrl = await oauth.getAuthorizationUrl({
  redirectUri: 'https://yourapp.com/callback',
  state: 'random-state-string'
});
```

### OpenID Connect

```typescript
const oidc = sdk.createOIDCProvider({
  issuer: 'https://accounts.google.com',
  clientId: 'your-client-id',
  clientSecret: 'your-client-secret',
  scope: ['openid', 'profile', 'email']
});

const userInfo = await oidc.getUserInfo(accessToken);
console.log(userInfo.email); // alice@gmail.com
```

### SAML 2.0

```typescript
const saml = sdk.createSAMLProvider({
  entryPoint: 'https://idp.example.com/saml/sso',
  issuer: 'urn:myapp:saml',
  cert: certificateString,
  privateKey: privateKeyString
});

const loginUrl = await saml.getLoginUrl({
  assertionConsumerServiceUrl: 'https://myapp.com/saml/acs'
});
```

### Decentralized Identifiers (DIDs)

```typescript
const did = await sdk.createDID({
  method: 'web',
  identifier: 'example.com:users:alice'
});

console.log(did.id); // did:web:example.com:users:alice

// Resolve DID document
const document = await sdk.resolveDID(did.id);
console.log(document.verificationMethod);
```

## 📖 Use Cases

### 1. Single Sign-On (SSO)

Enable users to access multiple applications with one identity:

```typescript
// User logs in once
const session = await sdk.createSession({
  identity: userIdentity,
  trustLevel: 'authenticated',
  duration: '8h'
});

// Access multiple services
await sdk.accessService({ service: 'app1.com', session });
await sdk.accessService({ service: 'app2.com', session });
await sdk.accessService({ service: 'app3.com', session });
```

### 2. Privacy-Preserving Verification

Verify attributes without revealing full identity:

```typescript
// Verify age without revealing birthdate
const ageProof = await identity.createSelectiveDisclosure({
  attribute: 'age',
  predicate: 'greaterThan',
  value: 18,
  revealValue: false
});

const verified = await verifyProof(ageProof);
console.log(verified.valid); // true
console.log(verified.value); // undefined (not revealed)
```

### 3. Cross-Platform Identity

Use same identity across Web2 and Web3:

```typescript
// Link Web2 identity
await identity.linkProvider({
  type: 'oauth',
  provider: 'github',
  username: 'alice'
});

// Link Web3 identity
await identity.linkProvider({
  type: 'blockchain',
  chain: 'ethereum',
  address: '0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb'
});

// Use unified identity
const profile = await identity.getUnifiedProfile();
console.log(profile.github); // GitHub profile
console.log(profile.ethereum); // Ethereum address
```

### 4. Verifiable Credentials

Issue and verify credentials:

```typescript
// University issues degree credential
const degree = await university.issueCredential({
  type: 'BachelorDegree',
  recipient: student.did,
  claims: {
    degree: 'Bachelor of Science',
    major: 'Computer Science',
    graduationDate: '2024-05-15',
    honors: 'Summa Cum Laude'
  },
  signature: universitySignature
});

// Employer verifies credential
const verification = await employer.verifyCredential(degree);
console.log(verification.valid); // true
console.log(verification.issuer); // did:web:university.edu
console.log(verification.claims.major); // Computer Science
```

### 5. Progressive Trust

Adapt authentication based on risk:

```typescript
// Low-risk operation (Level 0)
await sdk.accessPublicResource({
  trustLevel: 'anonymous'
});

// Medium-risk operation (Level 2)
await sdk.updateProfile({
  trustLevel: 'authenticated',
  mfaRequired: true
});

// High-risk operation (Level 4)
await sdk.transferFunds({
  trustLevel: 'verified',
  biometricRequired: true,
  amount: 10000
});
```

## 🛡️ Privacy & Security

### Privacy Principles

- **Data Minimization**: Request only necessary attributes
- **User Consent**: Explicit consent for all data sharing
- **Purpose Limitation**: Use data only for stated purposes
- **Right to Erasure**: Users can delete their data
- **Portability**: Users can export their data
- **Transparency**: Clear explanation of data usage

### Security Features

```yaml
Encryption:
  - At-rest: AES-256-GCM
  - In-transit: TLS 1.3
  - Key storage: Hardware Security Module (HSM)

Authentication:
  - Multi-factor: TOTP, SMS, Email, Biometric
  - Passwordless: WebAuthn, FIDO2
  - Risk-based: Adaptive authentication

Authorization:
  - Attribute-based: ABAC
  - Role-based: RBAC
  - Policy-based: PBAC

Audit:
  - All access logged
  - Cryptographic audit trail
  - Tamper-evident logging
```

## 🌐 WIA Integration

This standard integrates with:
- **WIA-INTENT**: Intent-based identity requests
- **WIA-OMNI-API**: Universal API authentication
- **WIA-SOCIAL**: Social platform identity federation
- **WIA-BLOCKCHAIN**: Web3 identity integration
- **WIA-PRIVACY**: Privacy-preserving protocols

## ⚠️ Compliance

### Standards Compliance

- ✅ OAuth 2.0 (RFC 6749)
- ✅ OAuth 2.1 (Draft)
- ✅ OpenID Connect 1.0
- ✅ SAML 2.0
- ✅ W3C Decentralized Identifiers (DIDs)
- ✅ W3C Verifiable Credentials
- ✅ WebAuthn / FIDO2
- ✅ GDPR (EU)
- ✅ CCPA (California)
- ✅ eIDAS (EU)

### Security Standards

- ✅ ISO/IEC 27001 (Information Security)
- ✅ ISO/IEC 29115 (Entity Authentication)
- ✅ NIST SP 800-63 (Digital Identity Guidelines)
- ✅ FIDO2 / WebAuthn
- ✅ OpenID Foundation Certification

## 🤝 Contributing

Contributions welcome! Please see our [Contributing Guide](https://github.com/WIA-Official/wia-standards/CONTRIBUTING.md).

## 📄 License

MIT License - see [LICENSE](https://github.com/WIA-Official/wia-standards/LICENSE)

## 🔗 Links

- **Website**: [wiastandards.com](https://wiastandards.com)
- **GitHub**: [github.com/WIA-Official/wia-standards](https://github.com/WIA-Official/wia-standards)
- **Documentation**: [docs.wiastandards.com](https://docs.wiastandards.com)
- **Certification**: [cert.wiastandards.com](https://cert.wiastandards.com)

---

**홍익인간 (弘益人間) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 SmileStory Inc. / WIA*
*MIT License*

---

**홍익인간 (弘益人間) - Benefit All Humanity** 🌍
