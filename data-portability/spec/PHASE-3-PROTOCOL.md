# WIA-LEG-008 PHASE 3 — Protocol Specification

**Standard:** WIA-LEG-008
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 3 of 4)

---

"data_deletion": {
      "enabled": true,
      "delay_days": 90,
      "preserve_categories": ["creative_works", "financial"]
    }
  },
  
  "authorized_executors": [
    {
      "executor_id": "did:wia:executor789",
      "name": "Jane Doe",
      "relationship": "spouse",
      "email": "jane@example.com",
      "phone": "+1-555-0100",
      "permissions": ["full_access"],
      "priority": 1,
      "alternate_contact": {
        "email": "jane.backup@example.com",
        "phone": "+1-555-0101"
      }
    },
    {
      "executor_id": "did:wia:executor456",
      "name": "John Doe Jr.",
      "relationship": "child",
      "email": "johnjr@example.com",
      "permissions": ["read_only"],
      "priority": 2
    }
  ],
  
  "special_instructions": {
    "social_media": "Transfer all posts to family memorial site",
    "photos": "Create shared family album, send copies to children",
    "financial": "Provide statements to executor only",
    "email": "Delete after 30 days except emails in 'Important' folder",
    "creative_works": "Publish posthumously on personal blog"
  },
  
  "blockchain_proof": {
    "chain": "ethereum",
    "contract_address": "0x1234567890abcdef",
    "transaction_hash": "0xabcdef1234567890",
    "block_number": 12345678,
    "timestamp": "2024-01-01T00:00:00Z"
  },
  
  "signature": {
    "algorithm": "ECDSA",
    "public_key": "base64_encoded_public_key",
    "signature_value": "base64_encoded_signature"
  }
}
```

#### 7.1.2 Consent Verification API

```typescript
interface ConsentVerificationService {
  // Verify executor has valid consent
  verifyExecutorConsent(
    deceasedId: string,
    executorId: string,
    requestedPermissions: string[]
  ): Promise<VerificationResult>;
  
  // Check blockchain proof
  verifyBlockchainProof(
    consentId: string
  ): Promise<BlockchainVerification>;
  
  // Validate consent signature
  verifySignature(
    consent: ConsentRecord
  ): Promise<SignatureVerification>;
}

// Implementation
class ConsentVerifier implements ConsentVerificationService {
  async verifyExecutorConsent(
    deceasedId: string,
    executorId: string,
    requestedPermissions: string[]
  ): Promise<VerificationResult> {
    
    // 1. Retrieve consent record
    const consent = await this.getConsentRecord(deceasedId);
    
    // 2. Check if executor is authorized
    const executor = consent.authorized_executors.find(
      e => e.executor_id === executorId
    );
    
    if (!executor) {
      return {
        valid: false,
        reason: "Executor not authorized"
      };
    }
    
    // 3. Check if consent is still valid
    if (new Date() > new Date(consent.expires_at)) {
      return {
        valid: false,
        reason: "Consent expired"
      };
    }
    
    // 4. Verify requested permissions
    const hasPermissions = requestedPermissions.every(
      perm => executor.permissions.includes(perm) || 
              executor.permissions.includes('full_access')
    );
    
    if (!hasPermissions) {
      return {
        valid: false,
        reason: "Insufficient permissions"
      };
    }
    
    // 5. Verify blockchain proof
    const blockchainValid = await this.verifyBlockchainProof(consent.id);
    
    if (!blockchainValid.valid) {
      return {
        valid: false,
        reason: "Blockchain verification failed"
      };
    }
    
    // 6. Verify signature
    const signatureValid = await this.verifySignature(consent);
    
    if (!signatureValid.valid) {
      return {
        valid: false,
        reason: "Signature verification failed"
      };
    }
    
    return {
      valid: true,
      consent_id: consent.id,
      executor_name: executor.name,
      granted_permissions: executor.permissions
    };
  }
}
```

### 7.2 Multi-Factor Authentication

#### 7.2.1 MFA Requirements

Executor authentication requires:

1. **Knowledge Factor**: Password or PIN
2. **Possession Factor**: Hardware token, mobile device, or security key
3. **Inherence Factor**: Biometric (fingerprint, face recognition)
4. **Legal Documentation**: Death certificate, letters testamentary

```typescript
interface MFAChallenge {
  challenge_id: string;
  required_factors: FactorType[];
  completed_factors: FactorType[];
  status: 'pending' | 'completed' | 'failed';
  expires_at: Date;
}

enum FactorType {
  PASSWORD = 'password',
  TOTP = 'totp',
  SMS = 'sms',
  EMAIL = 'email',
  HARDWARE_TOKEN = 'hardware_token',
  BIOMETRIC = 'biometric',
  LEGAL_DOCUMENT = 'legal_document'
}

class MFAService {
  async initiateMFA(
    executorId: string,
    requiredFactors: FactorType[]
  ): Promise<MFAChallenge> {
    return {
      challenge_id: uuidv4(),
      required_factors: requiredFactors,
      completed_factors: [],
      status: 'pending',
      expires_at: new Date(Date.now() + 15 * 60 * 1000) // 15 min
    };
  }
  
  async verifyFactor(
    challengeId: string,
    factor: FactorType,
    proof: any
  ): Promise<FactorVerificationResult> {
    // Verify the provided factor
    switch (factor) {
      case FactorType.PASSWORD:
        return this.verifyPassword(proof.password);
      
      case FactorType.TOTP:
        return this.verifyTOTP(proof.code);
      
      case FactorType.BIOMETRIC:
        return this.verifyBiometric(proof.biometric_data);
      
      case FactorType.LEGAL_DOCUMENT:
        return this.verifyLegalDocument(proof.document);
      
      default:
        throw new Error(`Unsupported factor: ${factor}`);
    }
  }
}
```

---

## 8. Privacy-Preserving Mechanisms

### 8.1 Encryption Standards

#### 8.1.1 Data Encryption

```typescript
interface EncryptionConfig {
  algorithm: 'AES-256-GCM' | 'ChaCha20-Poly1305';
  key_derivation: 'PBKDF2' | 'Argon2id';
  iterations: number; // e.g., 100000 for PBKDF2
  salt_bytes: number; // e.g., 32
  iv_bytes: number; // e.g., 12 for GCM
}

class DataEncryption {
  async encryptDPP(
    dpp: DataPortabilityPackage,
    password: string,
    config: EncryptionConfig
  ): Promise<EncryptedDPP> {
    
    // 1. Generate salt
    const salt = crypto.getRandomValues(new Uint8Array(config.salt_bytes));
    
    // 2. Derive encryption key
    const key = await this.deriveKey(password, salt, config);
    
    // 3. Generate IV
    const iv = crypto.getRandomValues(new Uint8Array(config.iv_bytes));
    
    // 4. Encrypt data
    const plaintext = JSON.stringify(dpp);
    const ciphertext = await crypto.subtle.encrypt(
      {
        name: 'AES-GCM',
        iv: iv
      },
      key,
      new TextEncoder().encode(plaintext)
    );
    
    // 5. Create encrypted package
    return {
      version: '1.0',
      encryption: {
        algorithm: config.algorithm,
        key_derivation: config.key_derivation,
        iterations: config.iterations,
        salt: this.base64Encode(salt),
        iv: this.base64Encode(iv)
      },
      ciphertext: this.base64Encode(new Uint8Array(ciphertext))
    };
  }
  
  async decryptDPP(
    encrypted: EncryptedDPP,
    password: string
  ): Promise<DataPortabilityPackage> {
    
    // 1. Decode parameters
    const salt = this.base64Decode(encrypted.encryption.salt);
    const iv = this.base64Decode(encrypted.encryption.iv);
    const ciphertext = this.base64Decode(encrypted.ciphertext);
    
    // 2. Derive key
    const key = await this.deriveKey(password, salt, encrypted.encryption);
    
    // 3. Decrypt
    const plaintext = await crypto.subtle.decrypt(
      {
        name: 'AES-GCM',
        iv: iv
      },
      key,
      ciphertext
    );
    
    // 4. Parse JSON
    return JSON.parse(new TextDecoder().decode(plaintext));
  }
}
```

### 8.2 Selective Redaction

#### 8.2.1 Redaction Rules

```typescript
interface RedactionRule {
  field_path: string; // JSON path, e.g., "data.financial[*].account_number"
  redaction_type: 'mask' | 'remove' | 'hash' | 'encrypt';
  mask_pattern?: string; // e.g., "****{last4}"
  conditions?: {
    field: string;
    operator: '==' | '!=' | '>' | '<' | 'contains';
    value: any;
  }[];
}

const redactionRules: RedactionRule[] = [
  {
    field_path: "data.financial[*].account_number",
    redaction_type: "mask",
    mask_pattern: "****{last4}"
  },
  {
    field_path: "data.health[*].ssn",
    redaction_type: "remove"
  },
  {
    field_path: "data.communications[*].message_content",
    redaction_type: "encrypt",
    conditions: [
      {
        field: "privacy_level",
        operator: "==",
        value: "private"
      }
    ]
  }
];

class RedactionEngine {
  applyRedactions(
    dpp: DataPortabilityPackage,
    rules: RedactionRule[]
  ): DataPortabilityPackage {
    
    let redacted = JSON.parse(JSON.stringify(dpp)); // deep clone
    
    for (const rule of rules) {
      const matches = this.findFieldsByPath(redacted, rule.field_path);
      
      for (const match of matches) {
        if (this.evaluateConditions(match, rule.conditions)) {
          match.value = this.applyRedaction(match.value, rule);
        }
      }
    }
    
    return redacted;
  }
  
  private applyRedaction(value: any, rule: RedactionRule): any {
    switch (rule.redaction_type) {
      case 'mask':
        return this.maskValue(value, rule.mask_pattern);
      
      case 'remove':
        return undefined;
      
      case 'hash':
        return this.hashValue(value);
      
      case 'encrypt':
        return this.encryptValue(value);
      
      default:
        return value;
    }
  }
  
  private maskValue(value: string, pattern: string): string {
    // Example: "1234567890" with pattern "****{last4}" -> "****7890"
    const last4 = value.slice(-4);
    return pattern.replace('{last4}', last4);
  }
}
```

### 8.3 Zero-Knowledge Proofs

#### 8.3.1 ZKP for Consent Verification

```typescript
interface ZKProof {
  proof_type: 'executor_authorization' | 'data_ownership' | 'consent_validity';
  proof_data: string; // Base64-encoded proof
  public_inputs: any[];
  verification_key: string;
}

class ZKConsentProver {
  // Prove executor has valid authorization without revealing consent details
  async proveExecutorAuthorization(
    consentRecord: ConsentRecord,
    executorId: string,
    requestedPermissions: string[]
  ): Promise<ZKProof> {
    
    // Private inputs (not revealed)
    const privateInputs = {
      consent_record: consentRecord,
      executor_details: consentRecord.authorized_executors.find(
        e => e.executor_id === executorId
      )
    };
    
    // Public inputs (revealed)
    const publicInputs = {
      deceased_id: consentRecord.user_id,
      executor_id: executorId,
      current_timestamp: Date.now(),
      requested_permissions: requestedPermissions
    };
    
    // Generate proof
    const proof = await zkSNARK.prove(
      this.authorizationCircuit,
      privateInputs,
      publicInputs
    );
    
    return {
      proof_type: 'executor_authorization',
      proof_data: this.base64Encode(proof),
      public_inputs: [publicInputs],
      verification_key: this.authorizationVerificationKey
    };
  }
  
  async verifyProof(proof: ZKProof): Promise<boolean> {
    const proofData = this.base64Decode(proof.proof_data);
    
    return await zkSNARK.verify(
      proof.verification_key,
      proof.public_inputs,
      proofData
    );
  }
}
```

---

## 9. GDPR and Regulatory Compliance

### 9.1 GDPR Article 20 Compliance

#### 9.1.1 Right to Data Portability

The standard ensures full compliance with GDPR Article 20:

| GDPR Requirement | Implementation |
|------------------|----------------|
| Structured format | JSON-LD, XML, CSV formats |
| Commonly used format | Industry-standard formats |
| Machine-readable | All exports are machine-parseable |
| Interoperable | Schema mapping for cross-platform transfer |
| No hindrance | Automated export and transfer processes |
| Direct transmission | Service-to-service migration support |

#### 9.1.2 Post-Mortem Considerations

GDPR Recital 68 states that member states may provide rules for the exercise of data subject rights by deceased individuals:

```typescript


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

## Annex G — Test Vectors and Conformance Evidence

This annex describes how implementations capture and publish conformance
evidence for PHASE-3-PROTOCOL. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-3-protocol/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-3-PROTOCOL with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-3-PROTOCOL does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-3-PROTOCOL.
It is non-normative; the rules below describe the policy that the WIA
Standards working group commits to when amending this PHASE document.

- **Semantic versioning** — major / minor / patch components follow
  Semantic Versioning 2.0.0 (https://semver.org/spec/v2.0.0.html).
  Major bump indicates a backwards-incompatible change to a normative
  requirement; minor bump indicates new normative requirements that do
  not break existing implementations; patch bump indicates editorial
  changes only (clarifications, typo fixes, formatting).
- **Deprecation window** — when a normative requirement is removed or
  altered in a backwards-incompatible way, the prior major version is
  maintained in parallel for at least 180 days. During the parallel
  window, both major versions are marked Stable in the WIA Standards
  registry and either may be cited as "WIA-conformant".
- **Sunset notification** — deprecated major versions enter a 12-month
  sunset window during which the WIA registry marks the version as
  Deprecated. The deprecation entry includes a migration note pointing
  to the replacement requirement(s) and an explanation of why the
  change was made.
- **Editorial errata** — patch-level errata are issued without a
  deprecation window because they do not change normative behaviour.
  Errata are tracked in a public errata register and each entry is
  signed by the WIA Standards working group chair.
- **Implementation changelog mapping** — implementations SHOULD publish
  a changelog mapping each PHASE version they support to the specific
  build, container digest, or SDK version that satisfies the version.
  This allows downstream auditors to verify version conformance without
  re-running the entire test matrix on every release.

The policy is reviewed at the same cadence as the PHASE document and
any changes to the policy itself are tracked in the version-history
table at the start of the document.

## Annex I — Interoperability Profiles

This annex describes how implementations declare interoperability profiles
for PHASE-3-PROTOCOL. The profile mechanism is non-normative and exists so that
deployments of varying scope (single tenant, regional cluster, federated
network) can advertise the subset of normative requirements they satisfy
without misrepresenting partial conformance as full conformance.

- **Profile manifest** — every implementation publishes a profile manifest
  in JSON. The manifest enumerates the normative requirement IDs from this
  PHASE that are satisfied (`status: "supported"`), partially satisfied
  (`status: "partial"`, with a reason field), or excluded
  (`status: "excluded"`, with a justification). The manifest is signed
  using the same Sigstore key used for the SBOM in Annex G.
- **Federation profile** — federated deployments publish an aggregated
  manifest summarizing the union and intersection of member-implementation
  profiles. The aggregated manifest is consumed by directory services so
  that callers can route a request to the least common denominator profile
  required for an interaction.
- **Backwards-profile compatibility** — when a deployment migrates from one
  profile to a wider profile, the prior profile manifest remains valid and
  signed for the deprecation window defined in Annex H. This preserves
  audit traceability for auditors evaluating long-term interoperability.
- **Profile registry** — the WIA Standards working group maintains a
  public registry of named profiles. Common deployment shapes (e.g.,
  "Edge-only", "Federated-with-replay") are added to the registry by
  consensus. Registry entries are immutable; new shapes are added under
  new names rather than amending existing entries.
- **Profile versioning** — profile names are versioned with the same
  Semantic Versioning rules described in Annex H. A deployment that
  advertises `WIA-P3-PROTOCOL-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.

## Annex J — Reference Implementation Topology

The reference implementation topology described in this annex is
non-normative; it documents the deployment shape that the WIA
Standards working group used to validate the test vectors in Annex G
and is intended as a starting point, not a recommendation against
alternative topologies.

- **Single-tenant edge** — one runtime per organization, no shared
  state. Used for early-pilot deployments where conformance evidence
  is published manually. Sufficient for PHASE-3-PROTOCOL validation when the
  organization signs the manifest itself.
- **Multi-tenant gateway** — one shared runtime serves multiple
  tenants via header-based isolation. Typically backed by a
  rate-limited gateway (Envoy or NGINX) and a shared OAuth 2.1
  identity provider. The manifest is per-tenant; the runtime
  publishes a federation manifest that aggregates tenant manifests.
- **Federated mesh** — multiple runtimes peer to one another and
  publish their manifests to a directory service. Each peer signs
  its own manifest; the directory service signs the aggregated
  index. This is the topology used by cross-organization deployments
  that need to compose conformance.
- **Air-gapped batch** — no network connection between the runtime
  and the directory service. The runtime emits a signed evidence
  package on each batch and the operator transports the package via
  out-of-band channels. This is the topology used by regulators that
  prohibit live connectivity from sensitive environments.

Implementations declare their topology in the manifest (see Annex I).
A topology change MUST be reflected in a new manifest signature; the
prior topology's manifest remains valid for the deprecation window
described in Annex H to preserve audit traceability.
