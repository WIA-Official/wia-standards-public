# WIA-SEC-012: Homomorphic Encryption Standard
## PHASE 1 - CORE SPECIFICATION

**Standard ID:** WIA-SEC-012
**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-12-25
**Category:** Security (SEC)
**Emoji:** 🔐

---

## 1. INTRODUCTION

### 1.1 Purpose

The WIA-SEC-012 Homomorphic Encryption Standard defines a comprehensive framework for privacy-preserving computation on encrypted data. This standard enables organizations to perform computations on sensitive data without ever decrypting it, maintaining complete data confidentiality throughout the computational process.

### 1.2 Philosophy

**弘益人間 (홍익인간) - Benefit All Humanity**

This standard embodies the principle of benefiting all humanity by providing cryptographic tools that protect individual privacy while enabling collaborative computation and data sharing for the common good.

### 1.3 Scope

This specification covers:

- Fully Homomorphic Encryption (FHE) schemes
- Partially Homomorphic Encryption (PHE) schemes
- Secure Multi-Party Computation protocols
- Privacy-preserving machine learning
- Key management and distribution
- Performance optimization techniques
- Integration with existing systems

### 1.4 Target Audience

- Cryptography engineers
- Security architects
- Privacy officers
- Data scientists
- Cloud service providers
- Healthcare and financial institutions

---

## 2. TERMINOLOGY

### 2.1 Core Concepts

**Homomorphic Encryption (HE)**
A form of encryption that allows computation on ciphertexts, generating an encrypted result which, when decrypted, matches the result of operations performed on the plaintext.

**Fully Homomorphic Encryption (FHE)**
Encryption scheme that supports arbitrary computation (both addition and multiplication) on encrypted data without limitation.

**Partially Homomorphic Encryption (PHE)**
Encryption scheme that supports only specific operations (e.g., only addition or only multiplication) on encrypted data.

**Ciphertext**
Data in its encrypted form, unreadable without the decryption key.

**Plaintext**
Original, unencrypted data.

**Noise Budget**
The amount of computational operations that can be performed before the ciphertext becomes too noisy to decrypt correctly.

**Bootstrapping**
The process of refreshing the noise budget in FHE to enable unlimited computation depth.

---

## 3. ENCRYPTION SCHEMES

### 3.1 Fully Homomorphic Encryption (FHE)

#### 3.1.1 BFV (Brakerski-Fan-Vercauteren)

**Purpose:** Integer arithmetic with exact results

**Parameters:**
```
Polynomial Modulus Degree (n): 4096, 8192, 16384, 32768
Coefficient Modulus (q): Multi-prime chain
Plaintext Modulus (t): 1024, 2048, 4096
Security Level: 128-bit, 192-bit, 256-bit
```

**Supported Operations:**
- Addition: Enc(a) + Enc(b) = Enc(a + b)
- Multiplication: Enc(a) × Enc(b) = Enc(a × b)
- Scalar multiplication: k × Enc(a) = Enc(k × a)
- Rotation: Rotate(Enc([a₁, a₂, ...]), k)

**Use Cases:**
- Financial calculations
- Statistical analysis
- Database queries on encrypted data
- Voting systems

#### 3.1.2 CKKS (Cheon-Kim-Kim-Song)

**Purpose:** Approximate arithmetic on real/complex numbers

**Parameters:**
```
Polynomial Modulus Degree (n): 4096, 8192, 16384, 32768
Coefficient Modulus: Multi-level chain for rescaling
Scale: 2^40, 2^50, 2^60
Precision: 30-50 bits
Security Level: 128-bit, 192-bit, 256-bit
```

**Supported Operations:**
- Approximate addition: Enc(a) + Enc(b) ≈ Enc(a + b)
- Approximate multiplication: Enc(a) × Enc(b) ≈ Enc(a × b)
- Rotation and slot operations
- Rescaling for multiplicative depth management

**Use Cases:**
- Machine learning inference
- Signal processing
- Scientific computing
- Real-time analytics

### 3.2 Partially Homomorphic Encryption (PHE)

#### 3.2.1 Paillier Cryptosystem

**Purpose:** Additive homomorphism for secure aggregation

**Key Size:** 2048, 3072, 4096 bits

**Operations:**
- Enc(a) × Enc(b) = Enc(a + b)
- Enc(a)^k = Enc(k × a)

**Use Cases:**
- Secure voting and auctions
- Privacy-preserving data aggregation
- Financial portfolio analysis

#### 3.2.2 RSA Homomorphic Properties

**Purpose:** Multiplicative homomorphism

**Key Size:** 2048, 3072, 4096 bits

**Operations:**
- Enc(a) × Enc(b) = Enc(a × b)

**Use Cases:**
- Digital signatures
- Zero-knowledge proofs
- Secure product calculations

#### 3.2.3 ElGamal Encryption

**Purpose:** Multiplicative homomorphism with semantic security

**Operations:**
- Enc(a) × Enc(b) = Enc(a × b)

**Use Cases:**
- Threshold cryptography
- Mix networks
- Anonymous credentials

---

## 4. KEY MANAGEMENT

### 4.1 Key Generation

#### 4.1.1 Key Types

**Public Key (pk)**
- Used for encryption
- Can be distributed freely
- Size: Dependent on security parameter

**Secret Key (sk)**
- Used for decryption
- Must be kept private
- Requires secure storage

**Relinearization Keys (rlk)**
- Used to reduce ciphertext size after multiplication
- Public component
- Generated from secret key

**Galois Keys (gk)**
- Enable rotation operations
- Public component
- Generated for specific rotation amounts

#### 4.1.2 Key Generation Parameters

```typescript
interface KeyGenParams {
  scheme: 'BFV' | 'CKKS' | 'Paillier' | 'RSA';
  polyModulusDegree: 4096 | 8192 | 16384 | 32768;
  coefficientModulus: number[];
  plainModulus?: number;  // For BFV
  scale?: number;         // For CKKS
  securityLevel: 128 | 192 | 256;
}
```

### 4.2 Key Distribution

#### 4.2.1 Public Key Distribution

**Methods:**
- Certificate-based PKI
- Direct distribution via secure channels
- Blockchain-based public key infrastructure
- WIA-SEC-012 compliant key servers

**Format:**
```json
{
  "standardId": "WIA-SEC-012",
  "version": "1.0.0",
  "keyType": "public",
  "scheme": "BFV",
  "parameters": {
    "polyModulusDegree": 8192,
    "coefficientModulus": [60, 40, 40, 60],
    "plainModulus": 1024
  },
  "publicKey": "base64-encoded-key-data",
  "createdAt": "2025-12-25T00:00:00Z",
  "expiresAt": "2026-12-25T00:00:00Z",
  "issuer": "did:wia:issuer-id"
}
```

#### 4.2.2 Secret Key Storage

**Requirements:**
- Hardware Security Module (HSM) storage preferred
- Encrypted at rest using AES-256-GCM
- Multi-factor authentication for access
- Audit logging for all access attempts
- Regular key rotation schedule

**Storage Format:**
```json
{
  "encryptedSecretKey": "base64-encrypted-key",
  "keyDerivationFunction": "PBKDF2",
  "iterations": 100000,
  "salt": "base64-salt",
  "version": "1.0.0"
}
```

### 4.3 Key Rotation

**Rotation Period:** Recommended every 90-180 days

**Process:**
1. Generate new key pair
2. Re-encrypt all stored ciphertexts with new public key
3. Update all distributed public keys
4. Securely destroy old secret key
5. Update audit logs

---

## 5. ENCRYPTION AND DECRYPTION

### 5.1 Encryption Process

#### 5.1.1 Data Preparation

**Input Validation:**
- Verify data type compatibility
- Check data size limits
- Validate encoding scheme

**Encoding:**
```typescript
interface EncodingParams {
  dataType: 'integer' | 'float' | 'complex' | 'vector' | 'matrix';
  encoding: 'polynomial' | 'batch' | 'simd';
  scale?: number;  // For CKKS
}

function encode(
  data: number | number[] | number[][],
  params: EncodingParams
): Plaintext {
  // Encode data into polynomial representation
  // Return plaintext object ready for encryption
}
```

#### 5.1.2 Encryption Algorithm

**BFV Encryption:**
```
Input: plaintext m, public key (pk₀, pk₁)
Output: ciphertext (c₀, c₁)

1. Sample error polynomial e from error distribution
2. Sample random polynomial r
3. c₀ ← pk₀ × r + e + m
4. c₁ ← pk₁ × r
5. Return (c₀, c₁)
```

**CKKS Encryption:**
```
Input: plaintext vector m, public key pk, scale Δ
Output: ciphertext ct

1. Encode m into polynomial using CKKS encoding
2. Scale by Δ and round to nearest integer
3. Add encryption noise
4. Return ciphertext
```

### 5.2 Decryption Process

#### 5.2.1 Decryption Algorithm

**BFV Decryption:**
```
Input: ciphertext (c₀, c₁), secret key sk
Output: plaintext m

1. Compute intermediate = c₀ + c₁ × sk
2. Reduce modulo plaintext modulus
3. Return decoded plaintext m
```

**CKKS Decryption:**
```
Input: ciphertext ct, secret key sk
Output: approximate plaintext m

1. Compute polynomial result using secret key
2. Decode using CKKS decoding
3. Return approximate result within precision
```

#### 5.2.2 Noise Management

**Noise Budget Tracking:**
```typescript
interface NoiseBudget {
  initial: number;      // bits
  current: number;      // bits
  threshold: number;    // minimum safe bits
  operations: number;   // operations performed
}

function checkNoiseBudget(ciphertext: Ciphertext): NoiseBudget {
  // Calculate remaining noise budget
  // Return current noise status
}
```

**Bootstrapping Trigger:**
- Automatic when noise budget < threshold
- Manual trigger available
- Performance overhead: ~100-1000x slower than regular operations

---

## 6. HOMOMORPHIC OPERATIONS

### 6.1 Addition

**Operation:** `Enc(a) + Enc(b) = Enc(a + b)`

**Noise Growth:** Minimal (logarithmic)

**Implementation:**
```typescript
function add(
  ciphertext1: Ciphertext,
  ciphertext2: Ciphertext
): Ciphertext {
  // Component-wise polynomial addition
  // Return result ciphertext
}
```

**Cost:** O(n) where n is polynomial degree

### 6.2 Multiplication

**Operation:** `Enc(a) × Enc(b) = Enc(a × b)`

**Noise Growth:** Significant (exponential)

**Implementation:**
```typescript
function multiply(
  ciphertext1: Ciphertext,
  ciphertext2: Ciphertext,
  relinKeys: RelinearizationKeys
): Ciphertext {
  // Polynomial multiplication
  // Relinearization to reduce size
  // Return result ciphertext
}
```

**Cost:** O(n log n) with NTT optimization

**Note:** Requires relinearization to maintain ciphertext structure

### 6.3 Scalar Operations

**Scalar Addition:** `Enc(a) + k = Enc(a + k)`

**Scalar Multiplication:** `k × Enc(a) = Enc(k × a)`

**Implementation:**
```typescript
function scalarMultiply(
  ciphertext: Ciphertext,
  scalar: number
): Ciphertext {
  // Multiply each component by scalar
  // Return result ciphertext
}
```

**Cost:** O(n)

**Note:** No noise growth increase

### 6.4 Rotation

**Purpose:** Cyclic rotation of encrypted vectors

**Operation:** `Rotate(Enc([a₁, a₂, ..., aₙ]), k) = Enc([aₖ₊₁, ..., aₙ, a₁, ..., aₖ])`

**Implementation:**
```typescript
function rotate(
  ciphertext: Ciphertext,
  steps: number,
  galoisKeys: GaloisKeys
): Ciphertext {
  // Apply Galois automorphism
  // Return rotated ciphertext
}
```

**Requirements:**
- Pre-generated Galois keys for rotation amount
- Noise growth similar to multiplication

### 6.5 Comparison Operations

**Challenge:** Direct comparison not supported in standard HE

**Solutions:**

**Approximate Comparison (CKKS):**
```typescript
function approximateCompare(
  ciphertext1: Ciphertext,
  ciphertext2: Ciphertext,
  iterations: number = 3
): Ciphertext {
  // Use polynomial approximation of sign function
  // Return encrypted comparison result (≈0 or ≈1)
}
```

**Secure Comparison Protocol:**
- Use secure multi-party computation
- Garbled circuits for exact comparison
- Zero-knowledge proofs for specific comparisons

---

## 7. PERFORMANCE OPTIMIZATION

### 7.1 Batching (SIMD)

**Concept:** Encrypt multiple values in a single ciphertext

**Capacity:**
- BFV: n/2 slots (e.g., 4096 integers in one ciphertext)
- CKKS: n/2 slots (e.g., 4096 complex numbers)

**Benefits:**
- Reduced encryption/decryption overhead
- Parallel computation on vectors
- Improved throughput

**Implementation:**
```typescript
function batchEncode(
  values: number[],
  params: EncodingParams
): Plaintext {
  // Pack values into polynomial slots
  // Return plaintext with batched data
}
```

### 7.2 Parallelization

**Thread-Level Parallelism:**
- Parallel polynomial operations
- Independent ciphertext operations
- Multi-threaded key generation

**Data Parallelism:**
- Batch processing of multiple ciphertexts
- GPU acceleration for polynomial arithmetic
- SIMD instructions for vector operations

**Recommended Configuration:**
```typescript
interface ParallelConfig {
  threads: number;           // CPU threads
  useGPU: boolean;          // GPU acceleration
  batchSize: number;        // Ciphertexts per batch
  pipelineDepth: number;    // Operation pipeline
}
```

### 7.3 Caching Strategies

**Key Material Caching:**
- Relinearization keys
- Galois keys
- NTT tables

**Intermediate Results:**
- Common subexpression results
- Frequently used encrypted constants
- Pre-computed rotation results

**Cache Configuration:**
```typescript
interface CacheConfig {
  maxSize: number;          // MB
  ttl: number;              // seconds
  evictionPolicy: 'LRU' | 'LFU' | 'FIFO';
  warmupKeys: string[];     // Pre-load keys
}
```

### 7.4 Number Theoretic Transform (NTT)

**Purpose:** Fast polynomial multiplication

**Complexity Reduction:** O(n²) → O(n log n)

**Implementation Requirements:**
- Special prime moduli
- Pre-computed roots of unity
- Butterfly network computation

**Performance Gain:** 100-1000x faster polynomial operations

---

## 8. SECURITY CONSIDERATIONS

### 8.1 Security Parameters

**Minimum Requirements:**

| Security Level | Poly Degree | Coeff Modulus (bits) | Estimated Security |
|---------------|-------------|----------------------|-------------------|
| Low           | 4096        | 109                  | ~128 bits         |
| Medium        | 8192        | 218                  | ~128 bits         |
| High          | 16384       | 438                  | ~192 bits         |
| Very High     | 32768       | 881                  | ~256 bits         |

**Security Model:** Ring-Learning with Errors (RLWE) hardness assumption

### 8.2 Side-Channel Protection

**Timing Attacks:**
- Constant-time polynomial operations
- Avoid data-dependent branches
- Uniform memory access patterns

**Power Analysis:**
- Hardware countermeasures in HSM
- Randomized computation order
- Noise injection techniques

**Cache Attacks:**
- Cache-oblivious algorithms
- Memory access randomization
- Secure memory allocation

### 8.3 Quantum Resistance

**Status:** Post-quantum secure under RLWE assumption

**NIST PQC Alignment:**
- Similar to CRYSTALS-Kyber (lattice-based)
- Parameters selected for quantum resistance
- Regular security assessment updates

### 8.4 Access Control

**Requirements:**
- Role-based access control (RBAC)
- Attribute-based access control (ABAC)
- Multi-factor authentication
- Audit logging

**Example Policy:**
```json
{
  "policy": {
    "resource": "encrypted-data",
    "actions": ["encrypt", "compute", "decrypt"],
    "conditions": {
      "roles": ["data-scientist", "admin"],
      "mfa": true,
      "ipWhitelist": ["10.0.0.0/8"],
      "timeWindow": "business-hours"
    }
  }
}
```

---

## 9. COMPLIANCE AND STANDARDS

### 9.1 Regulatory Compliance

**GDPR (General Data Protection Regulation):**
- Privacy by design through encryption
- Data minimization via homomorphic computation
- Right to be forgotten compatibility

**HIPAA (Health Insurance Portability and Accountability Act):**
- Protected health information (PHI) encryption
- Secure computation on medical data
- Audit trail requirements

**PCI DSS (Payment Card Industry Data Security Standard):**
- Cardholder data encryption
- Secure payment processing
- Key management requirements

### 9.2 Standards Alignment

**ISO/IEC 18033:** Encryption algorithms
**ISO/IEC 19790:** Security requirements for cryptographic modules
**NIST SP 800-57:** Key management recommendations
**FIPS 140-3:** Cryptographic module validation

### 9.3 WIA Standards Integration

**Related WIA Standards:**
- WIA-SEC-001: Foundation Security Framework
- WIA-SEC-005: Zero-Knowledge Proofs
- WIA-SEC-008: Blockchain Security
- WIA-IDENTITY-002: Decentralized Identity

**Integration Points:**
- Encrypted credential storage (WIA-IDENTITY)
- Zero-knowledge verification (WIA-SEC-005)
- Blockchain data privacy (WIA-SEC-008)

---

## 10. TESTING AND VALIDATION

### 10.1 Functional Testing

**Correctness Tests:**
```typescript
describe('Homomorphic Operations', () => {
  test('Addition correctness', async () => {
    const a = 15, b = 7;
    const encA = await encrypt(a, publicKey);
    const encB = await encrypt(b, publicKey);
    const encSum = await add(encA, encB);
    const result = await decrypt(encSum, secretKey);
    expect(result).toBe(a + b);
  });

  test('Multiplication correctness', async () => {
    const a = 15, b = 7;
    const encA = await encrypt(a, publicKey);
    const encB = await encrypt(b, publicKey);
    const encProduct = await multiply(encA, encB, relinKeys);
    const result = await decrypt(encProduct, secretKey);
    expect(result).toBe(a * b);
  });
});
```

### 10.2 Performance Benchmarks

**Key Metrics:**
- Encryption throughput (ops/sec)
- Decryption throughput (ops/sec)
- Homomorphic operation latency
- Memory consumption
- Noise budget consumption rate

**Benchmark Suite:**
```typescript
interface BenchmarkResults {
  operation: string;
  iterations: number;
  avgTime: number;      // milliseconds
  throughput: number;   // ops/sec
  memoryPeak: number;   // MB
}
```

### 10.3 Security Testing

**Required Tests:**
- Known-plaintext attack resistance
- Chosen-ciphertext attack resistance
- Side-channel attack resistance
- Parameter validation
- Key strength verification

**Security Audit:**
- Third-party cryptographic review
- Penetration testing
- Formal verification (optional)

---

## 11. REFERENCE IMPLEMENTATION

### 11.1 Core Interface

```typescript
interface WIAHomomorphicEncryption {
  // Key Management
  generateKeys(params: KeyGenParams): Promise<KeyPair>;
  exportPublicKey(format: 'pem' | 'json'): string;
  importPublicKey(key: string): PublicKey;

  // Encryption/Decryption
  encrypt(data: number | number[], publicKey: PublicKey): Promise<Ciphertext>;
  decrypt(ciphertext: Ciphertext, secretKey: SecretKey): Promise<number | number[]>;

  // Homomorphic Operations
  add(ct1: Ciphertext, ct2: Ciphertext): Promise<Ciphertext>;
  multiply(ct1: Ciphertext, ct2: Ciphertext): Promise<Ciphertext>;
  scalarAdd(ct: Ciphertext, scalar: number): Promise<Ciphertext>;
  scalarMultiply(ct: Ciphertext, scalar: number): Promise<Ciphertext>;
  rotate(ct: Ciphertext, steps: number): Promise<Ciphertext>;

  // Utility
  getNoiseBudget(ciphertext: Ciphertext): number;
  bootstrap(ciphertext: Ciphertext): Promise<Ciphertext>;
}
```

### 11.2 Example Usage

```typescript
import { WIAHomomorphic } from '@wia/sec-012';

// Initialize
const client = new WIAHomomorphic({
  scheme: 'BFV',
  polyModulusDegree: 8192,
  plainModulus: 1024,
  securityLevel: 128
});

// Generate keys
const { publicKey, secretKey, relinKeys, galoisKeys } =
  await client.generateKeys();

// Encrypt data
const plaintext1 = 42;
const plaintext2 = 58;
const ciphertext1 = await client.encrypt(plaintext1, publicKey);
const ciphertext2 = await client.encrypt(plaintext2, publicKey);

// Perform homomorphic operations
const sum = await client.add(ciphertext1, ciphertext2);
const product = await client.multiply(ciphertext1, ciphertext2);

// Decrypt results
const sumResult = await client.decrypt(sum, secretKey);
console.log(`Sum: ${sumResult}`); // 100

const productResult = await client.decrypt(product, secretKey);
console.log(`Product: ${productResult}`); // 2436
```

---

## 12. NEXT STEPS

Continue to **PHASE-2-&-3-&-4.md** for:
- Advanced use cases and applications
- Secure multi-party computation protocols
- Privacy-preserving machine learning
- Cloud deployment architectures
- Integration patterns and best practices

---

**Document Control**

| Version | Date       | Author          | Changes                    |
|---------|------------|-----------------|----------------------------|
| 1.0.0   | 2025-12-25 | WIA Standards   | Initial PHASE 1 release    |

---

弘益人間 (홍익인간) · Benefit All Humanity

© 2025 SmileStory Inc. / WIA


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
