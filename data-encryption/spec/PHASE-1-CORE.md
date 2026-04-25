# WIA-SEC-011: Data Encryption Standard
## PHASE 1 - CORE SPECIFICATION

---

**Standard ID:** WIA-SEC-011
**Version:** 1.0.0
**Status:** Active
**Category:** Security (SEC)
**Emoji:** 🔒
**Color:** #8B5CF6 (SEC Purple)

---

## Philosophy

**弘益人間 (홍익인간)** - Benefit All Humanity

This standard provides enterprise-grade data encryption capabilities to protect sensitive information and ensure privacy for all users worldwide.

---

## 1. Introduction

### 1.1 Purpose

WIA-SEC-011 defines a comprehensive encryption standard for protecting data at rest and in transit. This standard provides:

- **Confidentiality**: Ensures data is accessible only to authorized parties
- **Integrity**: Protects data from unauthorized modification
- **Authenticity**: Verifies the source and legitimacy of data
- **Non-repudiation**: Prevents denial of data creation or transmission

### 1.2 Scope

This standard covers:

- Symmetric encryption (AES-256-GCM)
- Asymmetric encryption (RSA-2048/4096, ECC)
- Hybrid encryption protocols
- Key management and lifecycle
- Data format specifications
- Integration patterns

### 1.3 Target Audience

- Application developers
- Security engineers
- System architects
- DevOps engineers
- Compliance officers

---

## 2. Core Concepts

### 2.1 Encryption Types

#### 2.1.1 Symmetric Encryption

**Definition**: Uses the same key for encryption and decryption.

**Algorithm**: AES-256-GCM (Advanced Encryption Standard, 256-bit, Galois/Counter Mode)

**Characteristics**:
- Fast encryption/decryption
- Efficient for large data volumes
- Requires secure key distribution
- Provides authenticated encryption (AEAD)

**Use Cases**:
- File encryption
- Database encryption
- Session data protection
- Bulk data encryption

#### 2.1.2 Asymmetric Encryption

**Definition**: Uses a public key for encryption and a private key for decryption.

**Algorithms**:
- RSA-2048/4096 (Rivest-Shamir-Adleman)
- ECC P-256/P-384 (Elliptic Curve Cryptography)

**Characteristics**:
- Slower than symmetric encryption
- No key distribution problem
- Enables digital signatures
- Ideal for key exchange

**Use Cases**:
- Key exchange
- Digital signatures
- Certificate-based authentication
- Secure messaging

#### 2.1.3 Hybrid Encryption

**Definition**: Combines symmetric and asymmetric encryption for optimal performance and security.

**Process**:
1. Generate random symmetric key (AES-256)
2. Encrypt data with symmetric key
3. Encrypt symmetric key with recipient's public key (RSA)
4. Transmit encrypted data + encrypted key

**Benefits**:
- Performance of symmetric encryption
- Security of asymmetric key exchange
- Best of both worlds

---

## 3. Technical Specification

### 3.1 Symmetric Encryption (AES-256-GCM)

#### 3.1.1 Algorithm Parameters

```
Algorithm: AES-256-GCM
Key Size: 256 bits (32 bytes)
Block Size: 128 bits (16 bytes)
IV/Nonce Size: 96 bits (12 bytes) or 128 bits (16 bytes)
Authentication Tag Size: 128 bits (16 bytes)
Mode: GCM (Galois/Counter Mode)
```

#### 3.1.2 Encryption Process

```typescript
interface AESEncryptionInput {
  plaintext: Buffer;
  key: Buffer;        // 32 bytes
  aad?: Buffer;       // Additional Authenticated Data (optional)
}

interface AESEncryptionOutput {
  ciphertext: Buffer;
  iv: Buffer;         // 12 or 16 bytes
  authTag: Buffer;    // 16 bytes
}

function encryptAES256GCM(input: AESEncryptionInput): AESEncryptionOutput {
  // 1. Generate random IV
  const iv = crypto.randomBytes(16);

  // 2. Create cipher
  const cipher = crypto.createCipheriv('aes-256-gcm', input.key, iv);

  // 3. Set AAD if provided
  if (input.aad) {
    cipher.setAAD(input.aad);
  }

  // 4. Encrypt
  const ciphertext = Buffer.concat([
    cipher.update(input.plaintext),
    cipher.final()
  ]);

  // 5. Get authentication tag
  const authTag = cipher.getAuthTag();

  return { ciphertext, iv, authTag };
}
```

#### 3.1.3 Decryption Process

```typescript
interface AESDecryptionInput {
  ciphertext: Buffer;
  key: Buffer;
  iv: Buffer;
  authTag: Buffer;
  aad?: Buffer;
}

function decryptAES256GCM(input: AESDecryptionInput): Buffer {
  // 1. Create decipher
  const decipher = crypto.createDecipheriv('aes-256-gcm', input.key, input.iv);

  // 2. Set authentication tag
  decipher.setAuthTag(input.authTag);

  // 3. Set AAD if provided
  if (input.aad) {
    decipher.setAAD(input.aad);
  }

  // 4. Decrypt
  const plaintext = Buffer.concat([
    decipher.update(input.ciphertext),
    decipher.final()
  ]);

  return plaintext;
}
```

### 3.2 Asymmetric Encryption (RSA)

#### 3.2.1 Algorithm Parameters

```
Algorithm: RSA
Key Size: 2048 bits (minimum), 4096 bits (recommended)
Padding: OAEP with SHA-256
Hash Function: SHA-256
MGF: MGF1 with SHA-256
```

#### 3.2.2 Key Generation

```typescript
interface RSAKeyPair {
  publicKey: string;   // PEM format
  privateKey: string;  // PEM format
}

function generateRSAKeyPair(bits: 2048 | 4096 = 4096): RSAKeyPair {
  const { publicKey, privateKey } = crypto.generateKeyPairSync('rsa', {
    modulusLength: bits,
    publicKeyEncoding: {
      type: 'spki',
      format: 'pem'
    },
    privateKeyEncoding: {
      type: 'pkcs8',
      format: 'pem'
    }
  });

  return { publicKey, privateKey };
}
```

#### 3.2.3 Encryption Process

```typescript
function encryptRSA(plaintext: Buffer, publicKey: string): Buffer {
  return crypto.publicEncrypt(
    {
      key: publicKey,
      padding: crypto.constants.RSA_PKCS1_OAEP_PADDING,
      oaepHash: 'sha256'
    },
    plaintext
  );
}
```

#### 3.2.4 Decryption Process

```typescript
function decryptRSA(ciphertext: Buffer, privateKey: string): Buffer {
  return crypto.privateDecrypt(
    {
      key: privateKey,
      padding: crypto.constants.RSA_PKCS1_OAEP_PADDING,
      oaepHash: 'sha256'
    },
    ciphertext
  );
}
```

### 3.3 Hybrid Encryption Protocol

#### 3.3.1 Encryption Process

```typescript
interface HybridEncryptionOutput {
  encryptedData: Buffer;
  encryptedKey: Buffer;
  iv: Buffer;
  authTag: Buffer;
}

function hybridEncrypt(
  data: Buffer,
  recipientPublicKey: string
): HybridEncryptionOutput {
  // 1. Generate random AES key
  const aesKey = crypto.randomBytes(32);

  // 2. Encrypt data with AES
  const { ciphertext, iv, authTag } = encryptAES256GCM({
    plaintext: data,
    key: aesKey
  });

  // 3. Encrypt AES key with RSA
  const encryptedKey = encryptRSA(aesKey, recipientPublicKey);

  return {
    encryptedData: ciphertext,
    encryptedKey,
    iv,
    authTag
  };
}
```

#### 3.3.2 Decryption Process

```typescript
function hybridDecrypt(
  input: HybridEncryptionOutput,
  privateKey: string
): Buffer {
  // 1. Decrypt AES key with RSA
  const aesKey = decryptRSA(input.encryptedKey, privateKey);

  // 2. Decrypt data with AES
  const plaintext = decryptAES256GCM({
    ciphertext: input.encryptedData,
    key: aesKey,
    iv: input.iv,
    authTag: input.authTag
  });

  return plaintext;
}
```

---

## 4. Data Format Specification

### 4.1 Encrypted Data Container

```typescript
interface EncryptedContainer {
  version: string;              // Protocol version (e.g., "1.0")
  algorithm: string;            // Encryption algorithm
  keyId?: string;               // Key identifier (optional)
  timestamp: number;            // Unix timestamp
  metadata?: Record<string, any>; // Additional metadata
  payload: {
    ciphertext: string;         // Base64-encoded ciphertext
    iv: string;                 // Base64-encoded IV
    authTag?: string;           // Base64-encoded auth tag (for AEAD)
    encryptedKey?: string;      // Base64-encoded encrypted key (for hybrid)
  };
}
```

### 4.2 Example JSON Format

```json
{
  "version": "1.0",
  "algorithm": "AES-256-GCM",
  "keyId": "key-2025-001",
  "timestamp": 1735142400000,
  "metadata": {
    "purpose": "user-data",
    "classification": "confidential"
  },
  "payload": {
    "ciphertext": "8J+SiOKAnOKAjeKAjeKAjeKAnOKAjeKAnQ==",
    "iv": "r4nd0m1v12345678",
    "authTag": "auth3nt1c4t10nt4g"
  }
}
```

---

## 5. Key Management

### 5.1 Key Generation

**Requirements**:
- Use cryptographically secure random number generator (CSRNG)
- Minimum key size: AES-256 (32 bytes), RSA-2048 (256 bytes)
- Generate unique keys for each encryption operation

**Best Practices**:
- Use hardware security modules (HSM) when available
- Implement key derivation functions (KDF) for password-based keys
- Never hardcode encryption keys in source code

### 5.2 Key Storage

**Secure Storage Methods**:
1. **Hardware Security Module (HSM)**
   - FIPS 140-2 Level 2+ certified
   - Tamper-resistant hardware
   - Key never leaves HSM

2. **Key Management Service (KMS)**
   - Cloud-based key storage
   - Automated rotation
   - Access control and audit logging

3. **Encrypted Key Store**
   - Master key protection
   - Access control
   - Encrypted at rest

### 5.3 Key Rotation

**Rotation Schedule**:
- Symmetric keys: Every 90 days (recommended)
- Asymmetric keys: Every 365 days (recommended)
- Compromised keys: Immediately

**Rotation Process**:
1. Generate new key
2. Re-encrypt data with new key
3. Securely delete old key
4. Update key references

### 5.4 Key Revocation

**Revocation Triggers**:
- Key compromise detected
- Employee termination
- Security policy change
- Regular audit findings

**Revocation Process**:
1. Mark key as revoked in key registry
2. Notify all systems
3. Re-encrypt affected data
4. Archive old key for forensics (optional)

---

## 6. Security Considerations

### 6.1 Threat Model

**Protected Against**:
- Eavesdropping and interception
- Unauthorized data access
- Man-in-the-middle attacks
- Replay attacks
- Data tampering

**Not Protected Against**:
- Quantum computing attacks (requires post-quantum algorithms)
- Side-channel attacks (requires additional countermeasures)
- Social engineering attacks

### 6.2 Best Practices

1. **Always use authenticated encryption** (AES-GCM, ChaCha20-Poly1305)
2. **Never reuse IVs/nonces** with the same key
3. **Use constant-time comparison** for authentication tags
4. **Implement proper error handling** without leaking information
5. **Use secure random number generators** for all cryptographic operations
6. **Protect keys with strong access controls**
7. **Enable audit logging** for all encryption operations
8. **Implement key rotation policies**
9. **Use TLS 1.3+** for transport encryption
10. **Regularly update cryptographic libraries**

---

## 7. Compliance and Standards

### 7.1 Referenced Standards

- **NIST FIPS 140-2/140-3**: Cryptographic Module Validation
- **NIST SP 800-57**: Key Management Recommendations
- **NIST SP 800-38D**: AES-GCM Specification
- **PKCS #1 v2.2**: RSA Cryptography Standard
- **ISO/IEC 18033**: Encryption Algorithms

### 7.2 Regulatory Compliance

- **GDPR**: Article 32 - Security of Processing
- **HIPAA**: Security Rule - Technical Safeguards
- **PCI DSS**: Requirement 3 - Protect Stored Cardholder Data
- **SOC 2**: CC6.1 - Encryption Controls

---

## 8. Implementation Requirements

### 8.1 MUST Requirements

- ✅ Implement AES-256-GCM for symmetric encryption
- ✅ Implement RSA-2048 minimum (RSA-4096 recommended) for asymmetric encryption
- ✅ Use cryptographically secure random number generators
- ✅ Generate unique IV/nonce for each encryption operation
- ✅ Validate authentication tags in constant time
- ✅ Implement proper error handling
- ✅ Support key rotation
- ✅ Log all encryption/decryption operations

### 8.2 SHOULD Requirements

- ⚠️ Support hybrid encryption for large data
- ⚠️ Implement key derivation functions (HKDF, PBKDF2)
- ⚠️ Use hardware security modules when available
- ⚠️ Implement automated key rotation
- ⚠️ Support multiple encryption algorithms
- ⚠️ Enable audit logging with tamper protection

### 8.3 MAY Requirements

- 💡 Support post-quantum cryptography algorithms
- 💡 Implement homomorphic encryption
- 💡 Support secure multi-party computation
- 💡 Implement zero-knowledge proofs

---

## 9. Testing and Validation

### 9.1 Test Vectors

See **SPEC-APPENDIX.md** for comprehensive test vectors.

### 9.2 Validation Checklist

- [ ] AES-256-GCM encryption/decryption works correctly
- [ ] RSA encryption/decryption works correctly
- [ ] Hybrid encryption protocol works end-to-end
- [ ] IV/nonce uniqueness is enforced
- [ ] Authentication tag validation rejects tampered data
- [ ] Key rotation process completes successfully
- [ ] Error handling does not leak sensitive information
- [ ] All cryptographic operations are logged

---

## 10. References

1. NIST FIPS 197: Advanced Encryption Standard (AES)
2. NIST SP 800-38D: Recommendation for Block Cipher Modes of Operation: Galois/Counter Mode (GCM)
3. RFC 8017: PKCS #1: RSA Cryptography Specifications Version 2.2
4. RFC 5869: HMAC-based Extract-and-Expand Key Derivation Function (HKDF)
5. OWASP Cryptographic Storage Cheat Sheet

---

## Appendix

### A.1 Glossary

See **SPEC-GLOSSARY.md** for complete terminology.

### A.2 Additional Phases

See **PHASE-2-&-3-&-4.md** for advanced features and future roadmap.

---

**Document Control**

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0.0 | 2025-12-25 | WIA Standards Committee | Initial release |

---

**弘益人間 (홍익인간)** - Benefit All Humanity

© 2025 SmileStory Inc. / WIA

---

## Annex A — Conformance Tier Matrix

WIA conformance for data-encryption is evaluated across three tiers:

| Tier | Scope | Mandatory artifacts | Audit cadence |
|------|-------|--------------------|----------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | OpenAPI 3.0 contract, JSON Schema validation report, security threat model | Annual self-review |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Tier 1 artifacts + signed third-party assessor report against this PHASE | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Tier 2 artifacts + WIA accreditation, ISO/IEC 17065:2012 conformity assessment, evidence retention ≥ 7 years | Every 12 months |

Implementations MUST disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page. Tier downgrade events MUST be reported to the WIA registry within 30 days.

---

## Annex B — Cross-Walk to International Standards

The PHASE specification reuses or normatively references published standards alongside this document. Implementers SHOULD review the relevant standards alongside this PHASE document; where a conflict exists, the more specific WIA requirement governs unless explicitly superseded by a binding national regulation.

- ISO/IEC 17065:2012 — Conformity assessment — Requirements for bodies certifying products, processes and services
- ISO/IEC 27001:2022 — Information security management systems
- IETF RFC 9457 — Problem Details for HTTP APIs
- IETF RFC 7519 — JSON Web Token (JWT)
- IETF RFC 6749 — The OAuth 2.0 Authorization Framework
- W3C PROV-DM — provenance data model

This cross-walk is informative only. WIA does not republish the referenced documents; readers MUST obtain authoritative copies from the issuing body. Cross-walk entries are reviewed at every minor version of this PHASE.

---

## Annex C — Reference Implementations and Test Vectors

### C.1 Reference Implementations

WIA does not require implementers to use a particular library, but maintains pointers to the canonical reference implementation directories under the WIA-Official GitHub organization:

- `wia-standards/standards/data-encryption/api/` — TypeScript SDK skeleton
- `wia-standards/standards/data-encryption/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/data-encryption/simulator/` — interactive browser-based simulator for the PHASE protocol

Each reference artifact ships with an MIT license and is intended as a starting point, not as a production-grade implementation.

### C.2 Test Vectors

A normative set of request/response pairs covering the schemas defined in this PHASE is published alongside this document. Implementations claiming Tier 2 or Tier 3 conformance MUST pass every published test vector in both serialization (request) and deserialization (response) directions, and MUST publish a signed report identifying which vectors were exercised.

Test vectors are versioned independently of the PHASE document; refer to the `test-vectors/` directory for the active version.

---

## Annex D — Open Questions and Future Work

This PHASE document captures the consensus position at v1.0. The following items are tracked for future minor or major revisions:

1. **Schema versioning policy** — formal MUST/SHOULD rules for backward-compatible vs. breaking changes between minor releases.
2. **Privacy-preserving aggregation** — guidance on differential privacy and secure aggregation patterns for telemetry channels, without prescribing a specific algorithm.
3. **Multilingual error catalogs** — localization strategy for the standard error codes defined in this PHASE.
4. **Long-term retention** — alignment with sectoral retention regulations across jurisdictions.
5. **Sustainability disclosure** — optional fields for energy and emissions reporting tied to operations covered by this PHASE.

Items in this annex are non-normative. Comments and proposals are accepted via the GitHub issues tracker on the WIA-Official organization.

---


## Annex E — Implementation Notes for PHASE-1-CORE

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-1-CORE.

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

