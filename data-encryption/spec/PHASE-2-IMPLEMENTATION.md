## PHASE 2: Advanced Encryption Features

### 2.1 Elliptic Curve Cryptography (ECC)

#### 2.1.1 Overview

ECC provides equivalent security to RSA with significantly smaller key sizes, resulting in:
- Faster computation
- Lower power consumption
- Reduced storage requirements

#### 2.1.2 Supported Curves

```typescript
enum ECCurve {
  P256 = 'prime256v1',  // NIST P-256, 128-bit security
  P384 = 'secp384r1',   // NIST P-384, 192-bit security
  P521 = 'secp521r1',   // NIST P-521, 256-bit security
}
```

#### 2.1.3 Key Generation

```typescript
interface ECKeyPair {
  publicKey: string;   // PEM format
  privateKey: string;  // PEM format
  curve: ECCurve;
}

function generateECKeyPair(curve: ECCurve = ECCurve.P384): ECKeyPair {
  const { publicKey, privateKey } = crypto.generateKeyPairSync('ec', {
    namedCurve: curve,
    publicKeyEncoding: {
      type: 'spki',
      format: 'pem'
    },
    privateKeyEncoding: {
      type: 'pkcs8',
      format: 'pem'
    }
  });

  return { publicKey, privateKey, curve };
}
```

#### 2.1.4 ECDH Key Agreement

```typescript
interface ECDHResult {
  sharedSecret: Buffer;
  derivedKey: Buffer;
}

function performECDH(
  privateKey: string,
  peerPublicKey: string
): ECDHResult {
  // 1. Compute shared secret
  const ecdh = crypto.createECDH(ECCurve.P384);
  ecdh.setPrivateKey(crypto.createPrivateKey(privateKey));

  const peerKey = crypto.createPublicKey(peerPublicKey);
  const sharedSecret = ecdh.computeSecret(peerKey);

  // 2. Derive encryption key using HKDF
  const derivedKey = crypto.hkdfSync(
    'sha256',
    sharedSecret,
    Buffer.alloc(0),
    Buffer.from('WIA-SEC-011-v1'),
    32
  );

  return { sharedSecret, derivedKey };
}
```

#### 2.1.5 ECDSA Digital Signatures

```typescript
interface ECDSASignature {
  signature: Buffer;
  algorithm: string;
}

function signECDSA(data: Buffer, privateKey: string): ECDSASignature {
  const sign = crypto.createSign('SHA256');
  sign.update(data);
  sign.end();

  const signature = sign.sign(privateKey);

  return {
    signature,
    algorithm: 'ECDSA-SHA256'
  };
}

function verifyECDSA(
  data: Buffer,
  signature: Buffer,
  publicKey: string
): boolean {
  const verify = crypto.createVerify('SHA256');
  verify.update(data);
  verify.end();

  return verify.verify(publicKey, signature);
}
```

### 2.2 ChaCha20-Poly1305

#### 2.2.1 Overview

ChaCha20-Poly1305 is a modern authenticated encryption algorithm offering:
- High performance on mobile devices
- Resistance to timing attacks
- No AES hardware requirement

#### 2.2.2 Encryption

```typescript
interface ChaChaEncryptionOutput {
  ciphertext: Buffer;
  nonce: Buffer;
  authTag: Buffer;
}

function encryptChaCha20Poly1305(
  plaintext: Buffer,
  key: Buffer  // 32 bytes
): ChaChaEncryptionOutput {
  // 1. Generate random nonce
  const nonce = crypto.randomBytes(12);

  // 2. Create cipher
  const cipher = crypto.createCipheriv('chacha20-poly1305', key, nonce, {
    authTagLength: 16
  });

  // 3. Encrypt
  const ciphertext = Buffer.concat([
    cipher.update(plaintext),
    cipher.final()
  ]);

  // 4. Get authentication tag
  const authTag = cipher.getAuthTag();

  return { ciphertext, nonce, authTag };
}
```

### 2.3 Key Derivation Functions

#### 2.3.1 HKDF (HMAC-based Key Derivation)

```typescript
interface HKDFOptions {
  hash: 'sha256' | 'sha384' | 'sha512';
  salt?: Buffer;
  info?: Buffer;
  keyLength: number;
}

function deriveKeyHKDF(
  masterKey: Buffer,
  options: HKDFOptions
): Buffer {
  return crypto.hkdfSync(
    options.hash,
    masterKey,
    options.salt || Buffer.alloc(0),
    options.info || Buffer.alloc(0),
    options.keyLength
  );
}
```

#### 2.3.2 PBKDF2 (Password-Based Key Derivation)

```typescript
interface PBKDF2Options {
  password: string;
  salt: Buffer;
  iterations: number;  // Minimum 100,000
  keyLength: number;   // 32 for AES-256
  hash: 'sha256' | 'sha512';
}

function deriveKeyPBKDF2(options: PBKDF2Options): Buffer {
  return crypto.pbkdf2Sync(
    options.password,
    options.salt,
    options.iterations,
    options.keyLength,
    options.hash
  );
}
```

#### 2.3.3 Argon2 (Recommended for Password Hashing)

```typescript
interface Argon2Options {
  password: string;
  salt: Buffer;
  timeCost: number;    // Iterations
  memoryCost: number;  // Memory in KB
  parallelism: number; // Threads
  hashLength: number;  // Output size
}

async function deriveKeyArgon2(
  options: Argon2Options
): Promise<Buffer> {
  // Implementation using argon2 library
  const argon2 = require('argon2');

  return await argon2.hash(options.password, {
    type: argon2.argon2id,
    salt: options.salt,
    timeCost: options.timeCost,
    memoryCost: options.memoryCost,
    parallelism: options.parallelism,
    hashLength: options.hashLength,
    raw: true
  });
}
```

### 2.4 Envelope Encryption

#### 2.4.1 Concept

Envelope encryption adds an extra layer of security by encrypting data with a Data Encryption Key (DEK), then encrypting the DEK with a Key Encryption Key (KEK).

```
Data → Encrypt(DEK) → Encrypted Data
DEK → Encrypt(KEK) → Encrypted DEK
```

#### 2.4.2 Implementation

```typescript
interface EnvelopeEncryptionOutput {
  encryptedData: Buffer;
  encryptedDEK: Buffer;
  iv: Buffer;
  authTag: Buffer;
}

function envelopeEncrypt(
  data: Buffer,
  kek: Buffer  // Key Encryption Key from KMS
): EnvelopeEncryptionOutput {
  // 1. Generate random DEK
  const dek = crypto.randomBytes(32);

  // 2. Encrypt data with DEK
  const { ciphertext, iv, authTag } = encryptAES256GCM({
    plaintext: data,
    key: dek
  });

  // 3. Encrypt DEK with KEK
  const encryptedDEK = encryptAES256GCM({
    plaintext: dek,
    key: kek
  });

  return {
    encryptedData: ciphertext,
    encryptedDEK: encryptedDEK.ciphertext,
    iv,
    authTag
  };
}
```

---

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


## Annex E — Implementation Notes for PHASE-2-IMPLEMENTATION

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-2-IMPLEMENTATION.

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

