## PHASE 3: Enterprise Integration

### 3.1 Hardware Security Module (HSM) Integration

#### 3.1.1 PKCS#11 Interface

```typescript
interface HSMConfig {
  libraryPath: string;
  slotId: number;
  pin: string;
}

class HSMKeyManager {
  private session: any;

  constructor(config: HSMConfig) {
    const pkcs11 = require('pkcs11js');
    this.session = this.initializeSession(config);
  }

  async generateKey(keyId: string): Promise<void> {
    // Generate key in HSM
    // Key never leaves HSM
  }

  async encrypt(keyId: string, data: Buffer): Promise<Buffer> {
    // Encryption performed inside HSM
  }

  async decrypt(keyId: string, ciphertext: Buffer): Promise<Buffer> {
    // Decryption performed inside HSM
  }
}
```

#### 3.1.2 Key Management Service (KMS) Integration

```typescript
interface KMSConfig {
  provider: 'aws' | 'azure' | 'gcp';
  region: string;
  credentials: any;
}

class KMSKeyManager {
  private client: any;

  constructor(config: KMSConfig) {
    this.client = this.initializeKMS(config);
  }

  async createKey(alias: string): Promise<string> {
    // Create key in KMS
    // Returns key ID
  }

  async encrypt(keyId: string, data: Buffer): Promise<Buffer> {
    // Encrypt using KMS
  }

  async decrypt(keyId: string, ciphertext: Buffer): Promise<Buffer> {
    // Decrypt using KMS
  }

  async rotateKey(keyId: string): Promise<void> {
    // Automated key rotation
  }
}
```

### 3.2 Database Encryption

#### 3.2.1 Column-Level Encryption

```typescript
interface ColumnEncryptionConfig {
  tableName: string;
  columnName: string;
  keyId: string;
  encryptionType: 'deterministic' | 'randomized';
}

class DatabaseEncryption {
  async encryptColumn(
    config: ColumnEncryptionConfig,
    data: any
  ): Promise<Buffer> {
    if (config.encryptionType === 'deterministic') {
      // Same plaintext → same ciphertext (searchable)
      return this.deterministicEncrypt(data, config.keyId);
    } else {
      // Same plaintext → different ciphertext (more secure)
      return this.randomizedEncrypt(data, config.keyId);
    }
  }

  async decryptColumn(
    config: ColumnEncryptionConfig,
    ciphertext: Buffer
  ): Promise<any> {
    return this.decrypt(ciphertext, config.keyId);
  }
}
```

#### 3.2.2 Transparent Data Encryption (TDE)

```typescript
interface TDEConfig {
  databasePath: string;
  masterKey: Buffer;
  algorithm: 'AES-256-XTS';
}

class TransparentDataEncryption {
  // Encrypts entire database at rest
  // Transparent to applications
  // Performance impact minimal with hardware acceleration
}
```

### 3.3 File System Encryption

#### 3.3.1 File-Based Encryption

```typescript
interface FileEncryptionOptions {
  inputPath: string;
  outputPath: string;
  keyId: string;
  chunkSize?: number;  // Default: 64KB
}

async function encryptFile(
  options: FileEncryptionOptions
): Promise<void> {
  const key = await getKey(options.keyId);
  const readStream = fs.createReadStream(options.inputPath, {
    highWaterMark: options.chunkSize || 64 * 1024
  });
  const writeStream = fs.createWriteStream(options.outputPath);

  const iv = crypto.randomBytes(16);
  writeStream.write(iv);

  const cipher = crypto.createCipheriv('aes-256-gcm', key, iv);

  readStream.pipe(cipher).pipe(writeStream);

  await new Promise((resolve, reject) => {
    writeStream.on('finish', resolve);
    writeStream.on('error', reject);
  });

  // Write auth tag at the end
  const authTag = cipher.getAuthTag();
  writeStream.write(authTag);
}
```

### 3.4 Cloud Storage Encryption

#### 3.4.1 Client-Side Encryption

```typescript
interface CloudStorageEncryption {
  provider: 'S3' | 'Azure Blob' | 'GCS';
  encryptBeforeUpload: boolean;
  keyManagement: 'client' | 'server';
}

class SecureCloudStorage {
  async uploadEncrypted(
    filePath: string,
    destinationKey: string
  ): Promise<void> {
    // 1. Encrypt file locally
    const encryptedData = await this.encryptFile(filePath);

    // 2. Upload to cloud
    await this.upload(encryptedData, destinationKey);

    // 3. Store encryption metadata separately
    await this.storeMetadata(destinationKey, {
      keyId: '...',
      algorithm: 'AES-256-GCM',
      iv: '...'
    });
  }
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


## Annex E — Implementation Notes for PHASE-3-INTEGRATION

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-3-INTEGRATION.

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

