# WIA-LEG-008 PHASE 2 — API Interface Specification

**Standard:** WIA-LEG-008
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 2 of 4)

---

#### 4.2.4 Import Initiation

```http
POST /api/v1/import/initiate
Authorization: Bearer {executor_jwt_token}
Content-Type: multipart/form-data

{
  "dpp_file": (binary data),
  "destination_platform": "memorial_service",
  "import_options": {
    "privacy_level": "friends_only",
    "enable_comments": false,
    "notification_preferences": "none"
  }
}

Response:
{
  "import_id": "imp_xyz789",
  "status": "validating",
  "validation_results": {
    "format_valid": true,
    "schema_valid": true,
    "encryption_valid": true
  }
}
```

### 4.3 Transfer Protocols

#### 4.3.1 OAuth 2.0 Integration

```typescript
interface OAuth2Config {
  client_id: string;
  client_secret: string;
  authorization_endpoint: string;
  token_endpoint: string;
  scopes: string[];
  redirect_uri: string;
}

// Example: Facebook OAuth for data export
const fbOAuth: OAuth2Config = {
  client_id: "wia_leg_008_client",
  client_secret: "secret_key",
  authorization_endpoint: "https://www.facebook.com/v18.0/dialog/oauth",
  token_endpoint: "https://graph.facebook.com/v18.0/oauth/access_token",
  scopes: [
    "user_posts",
    "user_photos",
    "user_videos",
    "user_friends",
    "email"
  ],
  redirect_uri: "https://portability.wiastandards.com/callback"
};
```

#### 4.3.2 Data Streaming Protocol

For large datasets:

```typescript
interface StreamConfig {
  chunk_size_bytes: number; // e.g., 10MB
  compression: "gzip" | "brotli";
  checksum_algorithm: "sha256";
  retry_policy: {
    max_retries: number;
    backoff_ms: number;
  };
}

// Stream data in chunks
async function* streamExportData(
  exportId: string,
  config: StreamConfig
): AsyncGenerator<Uint8Array> {
  let offset = 0;
  while (true) {
    const chunk = await fetchChunk(exportId, offset, config.chunk_size_bytes);
    if (chunk.length === 0) break;
    yield chunk;
    offset += chunk.length;
  }
}
```

---

## 5. Digital Asset Portability

### 5.1 Asset Classification

```typescript
enum AssetType {
  SOCIAL_MEDIA_POST = "social_media_post",
  PHOTO = "photo",
  VIDEO = "video",
  AUDIO = "audio",
  DOCUMENT = "document",
  EMAIL = "email",
  MESSAGE = "message",
  FINANCIAL_RECORD = "financial_record",
  HEALTH_RECORD = "health_record",
  CONTACT = "contact",
  CALENDAR_EVENT = "calendar_event",
  BOOKMARK = "bookmark",
  PASSWORD = "password",
  CRYPTOCURRENCY = "cryptocurrency",
  NFT = "nft",
  GAME_SAVE = "game_save",
  VIRTUAL_ITEM = "virtual_item"
}

interface DigitalAsset {
  id: string;
  type: AssetType;
  platform: string;
  created_at: Date;
  modified_at: Date;
  size_bytes: number;
  format: string;
  url?: string;
  local_path?: string;
  metadata: Record<string, any>;
  ownership_proof?: string;
  transferable: boolean;
  transfer_restrictions?: string[];
}
```

### 5.2 Asset Inventory Generation

```typescript
interface AssetInventory {
  inventory_id: string;
  deceased_id: string;
  generated_at: Date;
  total_assets: number;
  total_value_usd?: number;
  categories: {
    [category: string]: {
      count: number;
      total_size_bytes: number;
      assets: DigitalAsset[];
    };
  };
  platforms: {
    [platform: string]: {
      account_id: string;
      last_activity: Date;
      asset_count: number;
      exportable: boolean;
      export_method?: string;
    };
  };
}
```

### 5.3 Media Asset Handling

#### 5.3.1 Photo/Video Metadata Preservation

```json
{
  "@type": "MediaAsset",
  "id": "photo_12345",
  "asset_type": "photo",
  "file_name": "vacation_2024.jpg",
  "size_bytes": 2048000,
  "mime_type": "image/jpeg",
  "dimensions": {
    "width": 1920,
    "height": 1080
  },
  "exif": {
    "camera_make": "Canon",
    "camera_model": "EOS R5",
    "date_taken": "2024-07-15T14:30:00Z",
    "location": {
      "latitude": 37.7749,
      "longitude": -122.4194,
      "altitude": 15.5
    },
    "settings": {
      "iso": 400,
      "aperture": "f/2.8",
      "shutter_speed": "1/250",
      "focal_length": "50mm"
    }
  },
  "people_tagged": [
    {"id": "person_1", "name": "Alice"},
    {"id": "person_2", "name": "Bob"}
  ],
  "albums": ["Vacation 2024", "Family Photos"],
  "caption": "Beautiful sunset at the beach",
  "privacy": "friends",
  "download_url": "https://encrypted-storage.example.com/photo_12345"
}
```

### 5.4 Financial Asset Portability

```typescript
interface FinancialAsset {
  asset_id: string;
  type: "bank_account" | "investment" | "cryptocurrency" | "stock" | "bond";
  institution: string;
  account_number: string; // masked
  balance: {
    amount: number;
    currency: string;
    as_of_date: Date;
  };
  transactions: Transaction[];
  statements: Statement[];
  beneficiaries?: Beneficiary[];
  transfer_instructions?: {
    method: string;
    destination_account?: string;
    executor_access_required: boolean;
  };
}
```

---

## 6. Service-to-Service Migration

### 6.1 Platform Connectors

#### 6.1.1 Connector Interface

```typescript
interface PlatformConnector {
  platform_id: string;
  platform_name: string;
  api_version: string;
  
  // Authentication
  authenticate(credentials: Credentials): Promise<AuthToken>;
  
  // Export
  initiateExport(config: ExportConfig): Promise<ExportJob>;
  getExportStatus(jobId: string): Promise<ExportStatus>;
  downloadExport(jobId: string): Promise<DataPortabilityPackage>;
  
  // Import
  validateImport(dpp: DataPortabilityPackage): Promise<ValidationResult>;
  initiateImport(dpp: DataPortabilityPackage): Promise<ImportJob>;
  getImportStatus(jobId: string): Promise<ImportStatus>;
  
  // Mapping
  mapToStandardSchema(platformData: any): StandardSchema;
  mapFromStandardSchema(standardData: StandardSchema): any;
}
```

#### 6.1.2 Facebook Connector Example

```typescript
class FacebookConnector implements PlatformConnector {
  platform_id = "facebook";
  platform_name = "Facebook";
  api_version = "v18.0";
  
  async authenticate(credentials: Credentials): Promise<AuthToken> {
    // OAuth 2.0 flow
    const response = await fetch(
      `https://graph.facebook.com/${this.api_version}/oauth/access_token`,
      {
        method: 'POST',
        body: JSON.stringify({
          client_id: credentials.client_id,
          client_secret: credentials.client_secret,
          grant_type: 'authorization_code',
          code: credentials.auth_code,
          redirect_uri: credentials.redirect_uri
        })
      }
    );
    return await response.json();
  }
  
  async initiateExport(config: ExportConfig): Promise<ExportJob> {
    // Use Facebook's Data Transfer API
    const response = await this.apiCall('/me/data_transfer', {
      method: 'POST',
      params: {
        destination: 'wia_portability_service',
        scope: config.categories.join(',')
      }
    });
    return {
      job_id: response.transfer_id,
      status: 'initiated',
      estimated_completion: response.estimated_completion_time
    };
  }
  
  mapToStandardSchema(fbData: any): StandardSchema {
    // Map Facebook-specific data to WIA standard schema
    return {
      '@context': 'https://schema.wiastandards.com/leg-008/v1',
      '@type': 'SocialMediaData',
      platform: 'facebook',
      posts: fbData.posts.data.map(post => ({
        id: post.id,
        created_at: post.created_time,
        content: post.message,
        media: post.attachments?.data || [],
        reactions: post.reactions?.summary || {},
        comments: post.comments?.data || []
      }))
    };
  }
}
```

### 6.2 Schema Mapping

#### 6.2.1 Mapping Configuration

```json
{
  "schema_mapping": {
    "version": "1.0",
    "source_platform": "facebook",
    "destination_platform": "memorial_service",
    "field_mappings": [
      {
        "source_field": "posts[].message",
        "destination_field": "memories[].content",
        "transform": "identity"
      },
      {
        "source_field": "posts[].created_time",
        "destination_field": "memories[].date",
        "transform": "iso8601_to_timestamp"
      },
      {
        "source_field": "photos[].images[0].source",
        "destination_field": "gallery[].image_url",
        "transform": "identity"
      },
      {
        "source_field": "friends[].name",
        "destination_field": "connections[].display_name",
        "transform": "identity"
      }
    ],
    "category_mappings": {
      "posts": "memories",
      "photos": "gallery",
      "friends": "connections",
      "videos": "videos"
    }
  }
}
```

#### 6.2.2 Transform Functions

```typescript
type TransformFunction = (input: any) => any;

const transforms: Record<string, TransformFunction> = {
  identity: (x) => x,
  
  iso8601_to_timestamp: (dateString: string) => 
    new Date(dateString).getTime(),
  
  timestamp_to_iso8601: (timestamp: number) => 
    new Date(timestamp).toISOString(),
  
  array_to_csv: (arr: any[]) => arr.join(','),
  
  csv_to_array: (csv: string) => csv.split(','),
  
  url_to_blob: async (url: string) => {
    const response = await fetch(url);
    return await response.blob();
  },
  
  html_to_markdown: (html: string) => {
    // Use library like turndown
    return convertHtmlToMarkdown(html);
  },
  
  markdown_to_html: (markdown: string) => {
    // Use library like marked
    return convertMarkdownToHtml(markdown);
  }
};
```

### 6.3 Automated Migration Pipeline

```typescript
class MigrationPipeline {
  async executeMigration(
    source: PlatformConnector,
    destination: PlatformConnector,
    config: MigrationConfig
  ): Promise<MigrationResult> {
    
    // Step 1: Export from source
    const exportJob = await source.initiateExport({
      categories: config.categories,
      format: 'json-ld'
    });
    
    // Step 2: Wait for export completion
    const dpp = await this.waitForExport(source, exportJob.job_id);
    
    // Step 3: Validate DPP
    const validation = await destination.validateImport(dpp);
    if (!validation.valid) {
      throw new Error(`Validation failed: ${validation.errors.join(', ')}`);
    }
    
    // Step 4: Transform data
    const transformed = await this.transformData(
      dpp,
      config.schema_mapping
    );
    
    // Step 5: Import to destination
    const importJob = await destination.initiateImport(transformed);
    
    // Step 6: Wait for import completion
    const result = await this.waitForImport(destination, importJob.job_id);
    
    // Step 7: Verify migration
    await this.verifyMigration(source, destination, dpp, result);
    
    return {
      migration_id: uuidv4(),
      source_platform: source.platform_id,
      destination_platform: destination.platform_id,
      items_migrated: result.items_imported,
      status: 'completed',
      audit_trail: [...]
    };
  }
}
```

---

## 7. Consent and Authorization

### 7.1 User Consent Framework

#### 7.1.1 Consent Record Structure

```json
{
  "@type": "ConsentRecord",
  "id": "urn:uuid:consent-abc-123",
  "version": "1.0",
  "user_id": "did:wia:123456",
  "granted_at": "2024-01-01T00:00:00Z",
  "expires_at": "2054-01-01T00:00:00Z",
  "consent_type": "post_mortem_data_portability",
  
  "granted_permissions": {
    "data_export": {
      "enabled": true,
      "categories": ["all"],
      "format": ["json-ld", "xml"],
      "encryption_required": true
    },
    "data_transfer": {
      "enabled": true,
      "allowed_destinations": ["memorial_services", "family_archives"],
      "prohibited_destinations": ["commercial_platforms"]
    },


## Annex E — Implementation Notes for PHASE-2-API-INTERFACE

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-2-API-INTERFACE.

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
evidence for PHASE-2-API-INTERFACE. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-2-api-interface/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-2-API-INTERFACE with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-2-API-INTERFACE does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-2-API-INTERFACE.
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
for PHASE-2-API-INTERFACE. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P2-API-INTERFACE-Edge-only/2` is asserting conformance with
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
  is published manually. Sufficient for PHASE-2-API-INTERFACE validation when the
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
