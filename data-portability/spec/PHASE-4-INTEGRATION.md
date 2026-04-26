# WIA-LEG-008 PHASE 4 — Integration Specification

**Standard:** WIA-LEG-008
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 4 of 4)

---

interface GDPRPostMortemRules {
  jurisdiction: string; // ISO 3166-1
  post_mortem_rights_recognized: boolean;
  legal_basis: string; // e.g., "Article 85 domestic law"
  
  executor_requirements: {
    proof_of_death_required: boolean;
    legal_authorization_required: boolean;
    acceptable_documents: string[];
  };
  
  data_retention: {
    max_retention_period_days: number;
    exceptions: string[];
  };
  
  prohibited_transfers: {
    countries: string[];
    categories: string[];
  };
}

// Example: French law (Article 40-1 of the Data Protection Act)
const francePostMortemRules: GDPRPostMortemRules = {
  jurisdiction: "FR",
  post_mortem_rights_recognized: true,
  legal_basis: "Loi n° 2016-1321 Article 40-1",
  
  executor_requirements: {
    proof_of_death_required: true,
    legal_authorization_required: true,
    acceptable_documents: [
      "death_certificate",
      "will_designation",
      "family_authorization"
    ]
  },
  
  data_retention: {
    max_retention_period_days: 365,
    exceptions: ["archival_purposes", "legal_obligations"]
  },
  
  prohibited_transfers: {
    countries: [], // Subject to GDPR Chapter V adequacy decisions
    categories: []
  }
};
```

### 9.2 CCPA Compliance

#### 9.2.1 California Consumer Privacy Act

```typescript
interface CCPACompliance {
  // Consumer right to data portability (CCPA § 1798.110)
  consumer_data_request: {
    categories_of_data: string[];
    sources_of_data: string[];
    business_purposes: string[];
    third_parties_shared_with: string[];
  };
  
  // Format requirements
  format: {
    readily_useable: boolean; // Must be true
    formats_offered: string[]; // e.g., ["json", "csv", "pdf"]
    electronic_transmission: boolean; // Must be true
  };
  
  // Response timeline
  response_timeline_days: 45; // Max 45 days under CCPA
  extension_allowed_days: 45; // Can extend by 45 days if reasonably necessary
  
  // Verification requirements
  verification: {
    two_step_process: boolean;
    match_identity_with_records: boolean;
  };
}
```

### 9.3 Other Regulatory Frameworks

#### 9.3.1 Multi-Jurisdictional Compliance Matrix

| Regulation | Jurisdiction | Data Portability Right | Post-Mortem Provisions |
|------------|--------------|------------------------|------------------------|
| GDPR | EU/EEA | Article 20 | Recital 68 (member state law) |
| CCPA | California, USA | § 1798.110 | Limited (estate executor access) |
| LGPD | Brazil | Article 18, IV | Article 21 (heir rights) |
| PIPA | South Korea | Article 4 | Article 39-8 (family requests) |
| PIPEDA | Canada | Principle 4.9 | Common law (estate rights) |
| PDPA | Singapore | Section 21 | Not explicitly addressed |

#### 9.3.2 Compliance API

```typescript
interface ComplianceEngine {
  // Determine applicable regulations
  getApplicableRegulations(
    userJurisdiction: string,
    platformJurisdictions: string[]
  ): Regulation[];
  
  // Check if export meets regulatory requirements
  validateExport(
    dpp: DataPortabilityPackage,
    regulations: Regulation[]
  ): ComplianceReport;
  
  // Generate compliance documentation
  generateComplianceReport(
    exportId: string
  ): Promise<ComplianceReport>;
}

interface ComplianceReport {
  export_id: string;
  regulations_checked: string[];
  compliant: boolean;
  issues: ComplianceIssue[];
  recommendations: string[];
  generated_at: Date;
}
```

---

## 10. Implementation Guidelines

### 10.1 Platform Integration

#### 10.1.1 For Platform Providers

Platforms should implement the following APIs:

```typescript
// Export API
POST /wia/v1/export/initiate
GET  /wia/v1/export/{id}/status
GET  /wia/v1/export/{id}/download

// Import API
POST /wia/v1/import/validate
POST /wia/v1/import/initiate
GET  /wia/v1/import/{id}/status

// Consent API
GET  /wia/v1/consent/{user_id}
POST /wia/v1/consent/{user_id}/verify

// Metadata API
GET  /wia/v1/metadata/schema
GET  /wia/v1/metadata/categories
```

#### 10.1.2 SDK Usage

```typescript
import { DataPortabilitySDK } from '@wia/leg-008';

// Initialize SDK
const sdk = new DataPortabilitySDK({
  platform: 'my_platform',
  api_key: 'platform_api_key',
  environment: 'production'
});

// Register platform capabilities
await sdk.registerPlatform({
  platform_id: 'my_platform',
  supported_categories: [
    'social_media',
    'photos',
    'videos',
    'documents'
  ],
  export_formats: ['json-ld', 'xml'],
  import_formats: ['json-ld'],
  max_export_size_bytes: 10 * 1024 * 1024 * 1024, // 10GB
  oauth_config: {
    authorization_endpoint: 'https://myplatform.com/oauth/authorize',
    token_endpoint: 'https://myplatform.com/oauth/token',
    scopes: ['read_data', 'export_data']
  }
});

// Handle export request
sdk.onExportRequest(async (request) => {
  const data = await myPlatform.getUserData(request.user_id, request.categories);
  const dpp = await sdk.createDPP(data, request.format);
  return dpp;
});

// Handle import request
sdk.onImportRequest(async (request) => {
  const validation = await sdk.validateDPP(request.dpp);
  if (!validation.valid) {
    throw new Error(validation.errors.join(', '));
  }
  
  const transformed = await sdk.transformDPP(
    request.dpp,
    myPlatform.schema
  );
  
  await myPlatform.importData(request.user_id, transformed);
  
  return {
    import_id: generateId(),
    status: 'completed',
    items_imported: transformed.items.length
  };
});
```

### 10.2 Testing and Validation

#### 10.2.1 Test Scenarios

```typescript
describe('WIA-LEG-008 Compliance Tests', () => {
  
  test('Export generates valid DPP', async () => {
    const dpp = await sdk.exportData({
      user_id: 'test_user',
      categories: ['all']
    });
    
    expect(dpp['@context']).toBe('https://schema.wiastandards.com/leg-008/v1');
    expect(dpp.version).toBe('1.0');
    expect(dpp.deceased).toBeDefined();
    expect(dpp.data_inventory).toBeDefined();
  });
  
  test('Executor authorization verification', async () => {
    const result = await sdk.verifyExecutorConsent(
      'deceased_user_id',
      'executor_id',
      ['export_data']
    );
    
    expect(result.valid).toBe(true);
    expect(result.granted_permissions).toContain('export_data');
  });
  
  test('Cross-platform transfer', async () => {
    const exportResult = await sourcePlatform.export({
      user_id: 'test_user',
      categories: ['posts', 'photos']
    });
    
    const importResult = await destinationPlatform.import(exportResult.dpp);
    
    expect(importResult.status).toBe('completed');
    expect(importResult.items_imported).toBeGreaterThan(0);
  });
  
  test('GDPR compliance check', async () => {
    const compliance = await sdk.checkGDPRCompliance(dpp);
    
    expect(compliance.article_20_compliant).toBe(true);
    expect(compliance.machine_readable).toBe(true);
    expect(compliance.structured_format).toBe(true);
  });
  
  test('Encryption and decryption', async () => {
    const encrypted = await sdk.encryptDPP(dpp, 'password');
    const decrypted = await sdk.decryptDPP(encrypted, 'password');
    
    expect(decrypted).toEqual(dpp);
  });
  
  test('Selective redaction', async () => {
    const redacted = await sdk.applyRedactions(dpp, redactionRules);
    
    expect(redacted.data.financial[0].account_number).toMatch(/\*\*\*\*\d{4}/);
  });
});
```

### 10.3 Performance Optimization

#### 10.3.1 Streaming Large Exports

```typescript
async function streamLargeExport(
  exportId: string,
  chunkSizeMB: number = 10
): AsyncGenerator<Uint8Array> {
  
  const chunkSizeBytes = chunkSizeMB * 1024 * 1024;
  let offset = 0;
  
  while (true) {
    const chunk = await fetchExportChunk(exportId, offset, chunkSizeBytes);
    
    if (chunk.length === 0) break;
    
    // Compress chunk
    const compressed = await compressChunk(chunk, 'gzip');
    
    yield compressed;
    
    offset += chunk.length;
    
    // Rate limiting
    await sleep(100); // 100ms between chunks
  }
}

// Usage
for await (const chunk of streamLargeExport('exp_123', 10)) {
  await writeToFile(chunk);
}
```

#### 10.3.2 Parallel Processing

```typescript
async function parallelExport(
  userId: string,
  categories: string[]
): Promise<DataPortabilityPackage> {
  
  // Export categories in parallel
  const categoryPromises = categories.map(async (category) => {
    return await exportCategory(userId, category);
  });
  
  const categoryData = await Promise.all(categoryPromises);
  
  // Combine into single DPP
  return {
    '@context': 'https://schema.wiastandards.com/leg-008/v1',
    version: '1.0',
    data: Object.fromEntries(
      categories.map((cat, i) => [cat, categoryData[i]])
    )
  };
}
```

---

## 11. Security Considerations

### 11.1 Threat Model

| Threat | Mitigation |
|--------|------------|
| Unauthorized executor access | Multi-factor authentication, legal document verification |
| Data interception during transfer | End-to-end encryption (TLS 1.3+) |
| Man-in-the-middle attack | Certificate pinning, mutual TLS |
| Data tampering | Digital signatures, checksums |
| Replay attacks | Nonce, timestamp validation |
| Credential theft | Short-lived tokens, token rotation |
| Insider threat | Audit logging, least privilege access |
| DDoS on export API | Rate limiting, CAPTCHA |

### 11.2 Encryption Best Practices

```typescript
const securityConfig = {
  // Encryption
  encryption: {
    algorithm: 'AES-256-GCM',
    key_derivation: 'Argon2id',
    iterations: 100000,
    memory_kb: 65536,
    parallelism: 4
  },
  
  // Transport
  transport: {
    tls_version: 'TLS 1.3',
    ciphers: [
      'TLS_AES_256_GCM_SHA384',
      'TLS_CHACHA20_POLY1305_SHA256'
    ],
    certificate_pinning: true
  },
  
  // Authentication
  authentication: {
    token_lifetime_seconds: 900, // 15 minutes
    refresh_token_lifetime_days: 30,
    mfa_required: true,
    session_lifetime_hours: 24
  },
  
  // Rate limiting
  rate_limiting: {
    requests_per_hour: 100,
    export_size_limit_gb: 50,
    concurrent_exports: 3
  }
};
```

### 11.3 Audit Logging

```typescript
interface AuditLog {
  event_id: string;
  timestamp: Date;
  event_type: string;
  actor: {
    id: string;
    type: 'executor' | 'system' | 'platform';
    ip_address: string;
    user_agent: string;
  };
  resource: {
    type: 'dpp' | 'consent' | 'transfer';
    id: string;
  };
  action: string;
  result: 'success' | 'failure' | 'error';
  details: any;
  metadata: {
    session_id: string;
    request_id: string;
    duration_ms: number;
  };
}

class AuditLogger {
  async log(event: AuditLog): Promise<void> {
    // Store in tamper-proof log
    await this.appendToBlockchain(event);
    
    // Also store in database for querying
    await this.database.auditLogs.create(event);
    
    // Real-time monitoring
    await this.monitoringService.emit('audit_event', event);
  }
}
```

---

## 12. References

### 12.1 Standards and Specifications

1. **GDPR** - General Data Protection Regulation (EU) 2016/679
2. **CCPA** - California Consumer Privacy Act of 2018
3. **ISO/IEC 29100** - Privacy framework
4. **ISO/IEC 27001** - Information security management
5. **W3C ODRL** - Open Digital Rights Language
6. **JSON-LD 1.1** - JSON for Linking Data
7. **RFC 7519** - JSON Web Token (JWT)
8. **RFC 8259** - JSON Data Interchange Format

### 12.2 Related WIA Standards

- **WIA-LEG-001**: Digital Will
- **WIA-LEG-002**: Digital Executor
- **WIA-LEG-003**: Digital Asset Inventory
- **WIA-LEG-004**: Digital Memorial
- **WIA-LEG-005**: Digital Inheritance
- **WIA-LEG-006**: Digital Vault
- **WIA-LEG-007**: Digital Commemoration

### 12.3 External Resources

- GDPR Article 20: https://gdpr-info.eu/art-20-gdpr/
- Data Transfer Project: https://datatransferproject.dev/
- Schema.org: https://schema.org/
- DID Core Specification: https://www.w3.org/TR/did-core/

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 SmileStory Inc. / WIA*
*MIT License*


## Annex E — Implementation Notes for PHASE-4-INTEGRATION

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-4-INTEGRATION.

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
evidence for PHASE-4-INTEGRATION. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-4-integration/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-4-INTEGRATION with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-4-INTEGRATION does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-4-INTEGRATION.
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
for PHASE-4-INTEGRATION. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P4-INTEGRATION-Edge-only/2` is asserting conformance with
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
  is published manually. Sufficient for PHASE-4-INTEGRATION validation when the
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
