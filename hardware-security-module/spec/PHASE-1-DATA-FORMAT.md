# WIA-SEC-014: Hardware Security Module - Phase 1 Core Specification

**Version:** 1.0
**Status:** Official Standard
**Last Updated:** 2025-12-25
**Category:** Security (SEC)
**Emoji:** 🔧

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Core Architecture](#2-core-architecture)
3. [Key Management](#3-key-management)
4. [Cryptographic Operations](#4-cryptographic-operations)
5. [Security Requirements](#5-security-requirements)
6. [API Specification](#6-api-specification)
7. [Compliance & Certification](#7-compliance--certification)
8. [Implementation Guidelines](#8-implementation-guidelines)

---

## 1. Introduction

### 1.1 Purpose

The WIA-SEC-014 Hardware Security Module (HSM) standard defines a comprehensive framework for hardware-based cryptographic key storage, management, and operations. This standard ensures:

- **Security**: Hardware-protected key storage with tamper resistance
- **Compliance**: FIPS 140-2/140-3 Level 2-3 alignment
- **Interoperability**: PKCS#11 compatibility with modern extensions
- **Performance**: High-throughput cryptographic operations
- **Auditability**: Comprehensive logging and monitoring

### 1.2 Scope

Phase 1 covers:

- Core HSM architecture and components
- Key lifecycle management (generation, storage, usage, destruction)
- Essential cryptographic operations (sign, verify, encrypt, decrypt)
- PKCS#11 v2.40 API implementation
- Security policies and access control
- Audit logging and monitoring

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)**

This standard provides enterprise-grade security tools accessible to organizations of all sizes, protecting sensitive data and enabling trust in digital systems.

---

## 2. Core Architecture

### 2.1 HSM Components

```
┌─────────────────────────────────────────────────────────┐
│                    HSM Hardware Module                   │
├─────────────────────────────────────────────────────────┤
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │  Crypto      │  │  Key         │  │  Random      │  │
│  │  Engine      │  │  Storage     │  │  Number      │  │
│  │              │  │              │  │  Generator   │  │
│  └──────────────┘  └──────────────┘  └──────────────┘  │
│                                                          │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │  Access      │  │  Audit       │  │  Firmware    │  │
│  │  Control     │  │  Logger      │  │  Manager     │  │
│  └──────────────┘  └──────────────┘  └──────────────┘  │
├─────────────────────────────────────────────────────────┤
│              Physical Security Boundary                  │
│         (Tamper-Evident/Tamper-Resistant)               │
└─────────────────────────────────────────────────────────┘
         ▲                                    ▲
         │                                    │
    ┌────┴────┐                          ┌───┴────┐
    │ PKCS#11 │                          │  REST  │
    │   API   │                          │  API   │
    └─────────┘                          └────────┘
```

### 2.2 Security Zones

#### Zone 1: Secure Cryptographic Boundary
- Hardware-protected key storage
- Cryptographic operations
- RNG and key derivation
- **No key export allowed**

#### Zone 2: Control Plane
- Authentication and authorization
- Policy enforcement
- Session management
- Audit logging

#### Zone 3: External Interface
- API endpoints (PKCS#11, REST)
- Request validation
- Response formatting
- Rate limiting

### 2.3 Key Storage Architecture

```
Key Storage Structure:
├── Slot 0 (Master Keys)
│   ├── Master Encryption Key (MEK)
│   ├── Master Signing Key (MSK)
│   └── Master Wrapping Key (MWK)
│
├── Slot 1-N (Application Keys)
│   ├── User Keys
│   │   ├── Signing Keys
│   │   ├── Encryption Keys
│   │   └── Authentication Keys
│   └── System Keys
│       ├── TLS Keys
│       └── Database Encryption Keys
```

---

## 3. Key Management

### 3.1 Key Lifecycle States

```
┌──────────┐     ┌──────────┐     ┌──────────┐
│          │     │          │     │          │
│  PRE-    ├────►│  ACTIVE  ├────►│SUSPENDED │
│ ACTIVE   │     │          │     │          │
│          │     │          │     │          │
└──────────┘     └────┬─────┘     └────┬─────┘
                      │                 │
                      │                 │
                      ▼                 ▼
                 ┌──────────┐     ┌──────────┐
                 │          │     │          │
                 │DESTROYED │◄────┤ REVOKED  │
                 │          │     │          │
                 └──────────┘     └──────────┘
```

### 3.2 Key Generation

#### 3.2.1 Generation Methods

**On-Device Generation (Recommended)**
```json
{
  "operation": "C_GenerateKeyPair",
  "mechanism": "CKM_RSA_PKCS_KEY_PAIR_GEN",
  "parameters": {
    "keySize": 2048,
    "publicExponent": 65537
  },
  "attributes": {
    "CKA_TOKEN": true,
    "CKA_PRIVATE": true,
    "CKA_SENSITIVE": true,
    "CKA_EXTRACTABLE": false,
    "CKA_SIGN": true,
    "CKA_UNWRAP": false
  },
  "label": "user-signing-key-001"
}
```

**Supported Key Types:**
- RSA: 1024, 2048, 3072, 4096 bits
- ECC: P-256, P-384, P-521, secp256k1
- AES: 128, 192, 256 bits
- 3DES: 168 bits (legacy)

#### 3.2.2 Random Number Generation

- **Source**: Hardware RNG (TRNG)
- **Standard**: NIST SP 800-90A/B/C
- **Entropy**: Minimum 256 bits
- **Compliance**: FIPS 140-2 Level 2

### 3.3 Key Attributes

#### 3.3.1 PKCS#11 Core Attributes

| Attribute | Type | Description |
|-----------|------|-------------|
| CKA_CLASS | CK_OBJECT_CLASS | Object class (key, certificate, data) |
| CKA_TOKEN | CK_BBOOL | Persistent token object |
| CKA_PRIVATE | CK_BBOOL | Requires authentication |
| CKA_LABEL | String | Human-readable label |
| CKA_ID | Bytes | Unique identifier |
| CKA_SENSITIVE | CK_BBOOL | Cannot be read in plaintext |
| CKA_EXTRACTABLE | CK_BBOOL | Can be wrapped/exported |
| CKA_MODIFIABLE | CK_BBOOL | Attributes can be changed |

#### 3.3.2 Usage Attributes

| Attribute | Purpose |
|-----------|---------|
| CKA_SIGN | Digital signature generation |
| CKA_VERIFY | Signature verification |
| CKA_ENCRYPT | Data encryption |
| CKA_DECRYPT | Data decryption |
| CKA_WRAP | Key wrapping/export |
| CKA_UNWRAP | Key unwrapping/import |
| CKA_DERIVE | Key derivation |

### 3.4 Key Backup and Recovery

#### 3.4.1 Key Wrapping

```
Master Wrapping Key (MWK)
         │
         ├── Wrap → Application Key 1 (encrypted)
         ├── Wrap → Application Key 2 (encrypted)
         └── Wrap → Application Key N (encrypted)
```

**Wrapping Mechanism**: AES-GCM or RSA-OAEP

#### 3.4.2 M-of-N Key Recovery

- Split master key into N shares
- Require M shares to reconstruct (M ≤ N)
- Shamir's Secret Sharing algorithm
- Secure key ceremony procedures

---

## 4. Cryptographic Operations

### 4.1 Digital Signatures

#### 4.1.1 RSA Signatures

**Supported Mechanisms:**
- CKM_RSA_PKCS
- CKM_SHA256_RSA_PKCS
- CKM_SHA384_RSA_PKCS
- CKM_SHA512_RSA_PKCS
- CKM_RSA_PKCS_PSS

**Example: PSS Signature**
```json
{
  "operation": "C_Sign",
  "sessionHandle": "0x4A3B2C1D",
  "mechanism": {
    "type": "CKM_RSA_PKCS_PSS",
    "parameters": {
      "hashAlg": "CKM_SHA256",
      "mgf": "CKG_MGF1_SHA256",
      "saltLength": 32
    }
  },
  "keyHandle": "0x8F7E6D5C",
  "data": "SGVsbG8sIEhTTSE="
}
```

#### 4.1.2 ECDSA Signatures

**Supported Curves:**
- NIST P-256 (secp256r1)
- NIST P-384 (secp384r1)
- NIST P-521 (secp521r1)
- secp256k1 (Bitcoin/Ethereum)

**Mechanism**: CKM_ECDSA, CKM_ECDSA_SHA256

### 4.2 Encryption/Decryption

#### 4.2.1 RSA Encryption

**Mechanisms:**
- CKM_RSA_PKCS (PKCS#1 v1.5)
- CKM_RSA_PKCS_OAEP (Recommended)

**OAEP Parameters:**
```json
{
  "hashAlg": "CKM_SHA256",
  "mgf": "CKG_MGF1_SHA256",
  "source": "CKZ_DATA_SPECIFIED",
  "sourceData": ""
}
```

#### 4.2.2 Symmetric Encryption

**AES Modes:**
- CKM_AES_ECB (Not recommended)
- CKM_AES_CBC
- CKM_AES_GCM (Recommended)
- CKM_AES_CTR

**AES-GCM Example:**
```json
{
  "operation": "C_Encrypt",
  "mechanism": {
    "type": "CKM_AES_GCM",
    "parameters": {
      "iv": "random-96-bit-nonce",
      "aad": "additional-authenticated-data",
      "tagBits": 128
    }
  },
  "keyHandle": "0x12345678",
  "plaintext": "SGVsbG8sIFdvcmxkIQ=="
}
```

### 4.3 Hashing and MAC

#### 4.3.1 Hash Functions

- SHA-256, SHA-384, SHA-512
- SHA3-256, SHA3-384, SHA3-512
- BLAKE2b, BLAKE2s

#### 4.3.2 Message Authentication Codes

**HMAC:**
- HMAC-SHA256
- HMAC-SHA384
- HMAC-SHA512

**CMAC:**
- AES-CMAC

---

## 5. Security Requirements

### 5.1 Physical Security

#### 5.1.1 Tamper Evidence
- Epoxy encapsulation of critical components
- Tamper-evident seals and labels
- Removal detection sensors

#### 5.1.2 Tamper Response
- Immediate key zeroization on tamper detection
- Audit log entry generation
- Optional administrative alert

### 5.2 Logical Security

#### 5.2.1 Authentication

**User Types:**
- Security Officer (SO)
- Crypto Officer (CO)
- User (U)
- Auditor (read-only)

**Authentication Methods:**
- PIN/Password (minimum 8 characters)
- Smart card + PIN
- Biometric (optional)
- Multi-factor authentication

#### 5.2.2 Authorization

**Role-Based Access Control (RBAC):**

| Role | Permissions |
|------|-------------|
| Security Officer | Initialize, configure, user management |
| Crypto Officer | Key management, policy configuration |
| User | Cryptographic operations with assigned keys |
| Auditor | Read audit logs, export reports |

### 5.3 Cryptographic Security

#### 5.3.1 Key Security Levels

**Level 1 - Public Keys**
- Can be freely distributed
- Stored with integrity protection

**Level 2 - Symmetric Keys**
- Encrypted at rest
- Never exposed in plaintext outside HSM

**Level 3 - Private Keys**
- Generated and stored in HSM
- CKA_SENSITIVE = true
- CKA_EXTRACTABLE = false

**Level 4 - Master Keys**
- Highest protection
- M-of-N recovery only
- Physical access required for initialization

---

## 6. API Specification

### 6.1 PKCS#11 Interface

#### 6.1.1 Core Functions

**Initialization:**
```c
CK_RV C_Initialize(CK_VOID_PTR pInitArgs);
CK_RV C_Finalize(CK_VOID_PTR pReserved);
CK_RV C_GetInfo(CK_INFO_PTR pInfo);
```

**Session Management:**
```c
CK_RV C_OpenSession(CK_SLOT_ID slotID, CK_FLAGS flags,
                    CK_VOID_PTR pApplication, CK_NOTIFY Notify,
                    CK_SESSION_HANDLE_PTR phSession);
CK_RV C_CloseSession(CK_SESSION_HANDLE hSession);
CK_RV C_Login(CK_SESSION_HANDLE hSession, CK_USER_TYPE userType,
              CK_UTF8CHAR_PTR pPin, CK_ULONG ulPinLen);
CK_RV C_Logout(CK_SESSION_HANDLE hSession);
```

**Key Generation:**
```c
CK_RV C_GenerateKey(CK_SESSION_HANDLE hSession,
                    CK_MECHANISM_PTR pMechanism,
                    CK_ATTRIBUTE_PTR pTemplate,
                    CK_ULONG ulCount,
                    CK_OBJECT_HANDLE_PTR phKey);

CK_RV C_GenerateKeyPair(CK_SESSION_HANDLE hSession,
                        CK_MECHANISM_PTR pMechanism,
                        CK_ATTRIBUTE_PTR pPublicKeyTemplate,
                        CK_ULONG ulPublicKeyAttributeCount,
                        CK_ATTRIBUTE_PTR pPrivateKeyTemplate,
                        CK_ULONG ulPrivateKeyAttributeCount,
                        CK_OBJECT_HANDLE_PTR phPublicKey,
                        CK_OBJECT_HANDLE_PTR phPrivateKey);
```

**Cryptographic Operations:**
```c
CK_RV C_SignInit(CK_SESSION_HANDLE hSession,
                 CK_MECHANISM_PTR pMechanism,
                 CK_OBJECT_HANDLE hKey);
CK_RV C_Sign(CK_SESSION_HANDLE hSession,
             CK_BYTE_PTR pData, CK_ULONG ulDataLen,
             CK_BYTE_PTR pSignature, CK_ULONG_PTR pulSignatureLen);

CK_RV C_EncryptInit(CK_SESSION_HANDLE hSession,
                    CK_MECHANISM_PTR pMechanism,
                    CK_OBJECT_HANDLE hKey);
CK_RV C_Encrypt(CK_SESSION_HANDLE hSession,
                CK_BYTE_PTR pData, CK_ULONG ulDataLen,
                CK_BYTE_PTR pEncryptedData,
                CK_ULONG_PTR pulEncryptedDataLen);
```

### 6.2 REST API (Modern Extension)

#### 6.2.1 Authentication Endpoint

```http
POST /api/v1/auth/login
Content-Type: application/json

{
  "username": "crypto-officer",
  "password": "secure-password",
  "mfa_token": "123456"
}

Response:
{
  "token": "eyJhbGciOiJSUzI1NiIs...",
  "expires_in": 3600,
  "token_type": "Bearer"
}
```

#### 6.2.2 Key Management Endpoints

**Generate Key:**
```http
POST /api/v1/keys/generate
Authorization: Bearer {token}
Content-Type: application/json

{
  "keyType": "RSA-2048",
  "label": "user-signing-key",
  "usage": ["sign", "verify"],
  "extractable": false
}

Response:
{
  "keyId": "key-abc123",
  "publicKey": "-----BEGIN PUBLIC KEY-----...",
  "createdAt": "2025-12-25T10:00:00Z"
}
```

**Sign Data:**
```http
POST /api/v1/keys/{keyId}/sign
Authorization: Bearer {token}
Content-Type: application/json

{
  "data": "SGVsbG8sIEhTTSE=",
  "algorithm": "RSA-PSS-SHA256"
}

Response:
{
  "signature": "dGhpcyBpcyBhIHNpZ25hdHVyZQ==",
  "algorithm": "RSA-PSS-SHA256",
  "timestamp": "2025-12-25T10:01:00Z"
}
```

---

## 7. Compliance & Certification

### 7.1 FIPS 140-2/140-3 Compliance

#### 7.1.1 Security Levels

| Component | Level | Requirement |
|-----------|-------|-------------|
| Cryptographic Module | 2-3 | Tamper-evident, role-based auth |
| Physical Security | 2-3 | Tamper-evident seals, detection |
| Key Management | 3 | Hardware-protected keys |
| RNG | 2 | NIST approved DRBG |

#### 7.1.2 Approved Algorithms

- AES (FIPS 197)
- RSA (FIPS 186-4)
- ECDSA (FIPS 186-4)
- SHA-2/SHA-3 (FIPS 180-4, FIPS 202)
- HMAC (FIPS 198-1)
- DRBG (NIST SP 800-90A)

### 7.2 Common Criteria

- Protection Profile: PP-HSM v1.0
- Evaluation Assurance Level: EAL 4+
- Augmented with: AVA_VAN.5, ALC_FLR.2

### 7.3 Industry Standards

- PKCS#11 v2.40
- X.509 v3 certificates
- KMIP (Key Management Interoperability Protocol)
- IEEE P1619 (Data-at-rest encryption)

---

## 8. Implementation Guidelines

### 8.1 Deployment Architecture

#### 8.1.1 Single HSM Deployment

```
┌─────────────┐
│ Application │
└──────┬──────┘
       │
       ▼
┌─────────────┐
│  HSM Agent  │
└──────┬──────┘
       │
       ▼
┌─────────────┐
│     HSM     │
└─────────────┘
```

#### 8.1.2 High Availability Deployment

```
┌─────────────┐
│ Application │
└──────┬──────┘
       │
       ▼
┌─────────────────────┐
│   Load Balancer     │
└──────┬──────┬───────┘
       │      │
   ┌───┴──┐ ┌─┴────┐
   │ HSM  │ │ HSM  │
   │  #1  │ │  #2  │
   └──────┘ └──────┘
```

### 8.2 Performance Considerations

#### 8.2.1 Operation Throughput

| Operation | Typical Rate | High Performance |
|-----------|--------------|------------------|
| RSA-2048 Sign | 1,000 ops/sec | 10,000 ops/sec |
| RSA-2048 Verify | 10,000 ops/sec | 50,000 ops/sec |
| ECDSA P-256 Sign | 5,000 ops/sec | 25,000 ops/sec |
| AES-GCM Encrypt | 1 GB/sec | 10 GB/sec |

#### 8.2.2 Optimization Strategies

- Session pooling and reuse
- Batch operations where possible
- Key handle caching
- Asynchronous operation support

### 8.3 Monitoring and Logging

#### 8.3.1 Audit Events

**Critical Events:**
- User authentication (success/failure)
- Key generation/destruction
- Policy changes
- Tamper detection

**Operational Events:**
- Cryptographic operations
- Session creation/termination
- Object access

#### 8.3.2 Log Format

```json
{
  "timestamp": "2025-12-25T10:00:00.000Z",
  "eventType": "KEY_GENERATION",
  "severity": "INFO",
  "user": "crypto-officer",
  "sessionId": "sess-12345",
  "details": {
    "keyType": "RSA-2048",
    "keyId": "key-abc123",
    "label": "user-signing-key"
  },
  "result": "SUCCESS"
}
```

---

## 9. Security Considerations

### 9.1 Threat Model

**Protected Against:**
- Key extraction attempts
- Side-channel attacks (timing, power analysis)
- Fault injection attacks
- Physical tampering
- Unauthorized access

**Assumptions:**
- HSM firmware is trusted and verified
- Physical access is controlled
- Administrative credentials are protected
- Network communication is secured (TLS)

### 9.2 Best Practices

1. **Key Management**
   - Generate keys on-device
   - Set CKA_EXTRACTABLE = false for sensitive keys
   - Implement key rotation policies
   - Regular key backup and testing recovery

2. **Access Control**
   - Enforce strong authentication
   - Implement principle of least privilege
   - Regular access review and audit
   - Separate duties (SO, CO, User)

3. **Monitoring**
   - Real-time audit log monitoring
   - Alerting on suspicious activities
   - Regular security assessments
   - Incident response procedures

---

## Appendix A: Error Codes

| Code | Name | Description |
|------|------|-------------|
| 0x00000000 | CKR_OK | Success |
| 0x00000001 | CKR_CANCEL | Operation cancelled |
| 0x00000002 | CKR_HOST_MEMORY | Host memory error |
| 0x00000003 | CKR_SLOT_ID_INVALID | Invalid slot ID |
| 0x000000A0 | CKR_SESSION_HANDLE_INVALID | Invalid session handle |
| 0x000000A1 | CKR_OBJECT_HANDLE_INVALID | Invalid object handle |
| 0x000000A4 | CKR_USER_NOT_LOGGED_IN | User not authenticated |

---

## Appendix B: References

1. PKCS#11 v2.40 - Cryptographic Token Interface Standard
2. FIPS 140-2 - Security Requirements for Cryptographic Modules
3. FIPS 140-3 - Security Requirements for Cryptographic Modules (2019)
4. NIST SP 800-90A - Recommendation for Random Number Generation
5. NIST SP 800-57 - Recommendation for Key Management
6. Common Criteria Protection Profile for Hardware Security Modules

---

**Document Control:**
- Version: 1.0
- Status: Official Standard
- Next Review: 2026-12-25
- Maintained by: WIA Security Working Group

**弘益人間 (Benefit All Humanity)**

© 2025 WIA (World Certification Industry Association)


## Annex E — Implementation Notes for PHASE-1-DATA-FORMAT

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-1-DATA-FORMAT.

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
evidence for PHASE-1-DATA-FORMAT. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-1-data-format/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-1-DATA-FORMAT with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-1-DATA-FORMAT does not require bespoke
auditor tooling.
