# WIA-LEG-008 PHASE 1 — Data Format Specification

**Standard:** WIA-LEG-008
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 1 of 4)

---

# WIA-LEG-008: Data Portability Specification v1.0

> **Standard ID:** WIA-LEG-008
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Digital Legacy Working Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Post-Mortem Data Rights Framework](#2-post-mortem-data-rights-framework)
3. [Data Export Formats and Standards](#3-data-export-formats-and-standards)
4. [Cross-Platform Transfer Protocols](#4-cross-platform-transfer-protocols)
5. [Digital Asset Portability](#5-digital-asset-portability)
6. [Service-to-Service Migration](#6-service-to-service-migration)
7. [Consent and Authorization](#7-consent-and-authorization)
8. [Privacy-Preserving Mechanisms](#8-privacy-preserving-mechanisms)
9. [GDPR and Regulatory Compliance](#9-gdpr-and-regulatory-compliance)
10. [Implementation Guidelines](#10-implementation-guidelines)
11. [Security Considerations](#11-security-considerations)
12. [References](#12-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines comprehensive data portability standards for digital legacy, ensuring that individuals and their authorized representatives can seamlessly transfer digital assets, personal data, and account information across platforms and services after death.

### 1.2 Scope

The standard covers:
- Rights to data portability after death
- Standardized export formats (JSON-LD, XML, CSV)
- Cross-platform data transfer protocols
- Digital asset inventory and cataloging
- Service-to-service migration mechanisms
- User consent and executor authorization frameworks
- Privacy-preserving data portability techniques
- Compliance with GDPR, CCPA, and other regulations

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard ensures that digital legacy rights extend beyond life, allowing individuals to maintain control over their data and enabling authorized parties to preserve, transfer, and manage digital assets in accordance with the deceased's wishes.

### 1.4 Terminology

- **Data Subject**: The individual whose data is being transferred (deceased)
- **Executor**: Authorized person with legal rights to access and transfer deceased's data
- **Data Portability Package (DPP)**: Standardized container for exported data
- **Source Platform**: Service from which data is being exported
- **Destination Platform**: Service to which data is being imported
- **Consent Record**: Blockchain-verified record of user authorization
- **Privacy Envelope**: Encrypted wrapper for sensitive data

---

## 2. Post-Mortem Data Rights Framework

### 2.1 Legal Foundation

#### 2.1.1 Rights Recognition

Post-mortem data rights include:

| Right | Description | Legal Basis |
|-------|-------------|-------------|
| Access | Right for executors to access deceased's data | Estate law, GDPR Article 15 |
| Portability | Right to transfer data between services | GDPR Article 20 |
| Preservation | Right to preserve data for posterity | Copyright law, cultural heritage |
| Distribution | Right to distribute data per will instructions | Estate law, testamentary freedom |
| Deletion | Right to delete data posthumously | GDPR Article 17, privacy law |

#### 2.1.2 Executor Authorization

Executors must prove authorization through:

```json
{
  "executor_authorization": {
    "executor_id": "did:wia:executor789",
    "deceased_id": "did:wia:user123",
    "authorization_type": "legal_executor",
    "proof_documents": [
      {
        "type": "death_certificate",
        "issuer": "vital_records_office",
        "document_hash": "sha256:abc123...",
        "verification_url": "https://verify.gov/cert/123"
      },
      {
        "type": "letters_testamentary",
        "issuer": "probate_court",
        "document_hash": "sha256:def456...",
        "court_case": "2025-EST-001234"
      }
    ],
    "valid_from": "2025-01-15T00:00:00Z",
    "valid_until": "2027-01-15T00:00:00Z",
    "granted_permissions": [
      "read_all_data",
      "export_data",
      "transfer_data",
      "delete_data"
    ]
  }
}
```

### 2.2 Jurisdiction Handling

#### 2.2.1 Multi-Jurisdictional Framework

```typescript
interface JurisdictionRules {
  jurisdiction: string; // ISO 3166-1 alpha-2
  postMortemDataRights: {
    recognizedByLaw: boolean;
    maxDataRetentionYears: number;
    executorRequirements: string[];
    restrictedDataCategories: string[];
  };
  complianceFramework: {
    gdpr: boolean;
    ccpa: boolean;
    localRegulations: string[];
  };
}
```

### 2.3 Time-Limited Access

Executor access may be time-limited:

```typescript
interface AccessWindow {
  grantedAt: Date;
  expiresAt: Date;
  renewalAllowed: boolean;
  autoRenewal: boolean;
  maxExtensions: number;
  reasonForExtension?: string;
}
```

---

## 3. Data Export Formats and Standards

### 3.1 JSON-LD Format (Primary)

#### 3.1.1 Data Portability Package Schema

```json
{
  "@context": "https://schema.wiastandards.com/leg-008/v1",
  "@type": "DataPortabilityPackage",
  "id": "urn:uuid:550e8400-e29b-41d4-a716-446655440000",
  "version": "1.0",
  "generated_at": "2025-01-15T10:30:00Z",
  "expires_at": "2025-07-15T10:30:00Z",
  
  "deceased": {
    "@type": "Person",
    "id": "did:wia:123456",
    "name": "John Doe",
    "dateOfBirth": "1980-05-15",
    "dateOfDeath": "2025-01-01",
    "lastKnownLocation": {
      "@type": "Place",
      "address": "123 Main St, City, Country"
    }
  },
  
  "executor": {
    "@type": "LegalExecutor",
    "id": "did:wia:executor789",
    "name": "Jane Doe",
    "relationship": "spouse",
    "contactEmail": "jane@example.com",
    "authorization": {
      "type": "court_appointed",
      "document_ref": "urn:uuid:auth-token-xyz",
      "issued_by": "Superior Court of County",
      "valid_from": "2025-01-15",
      "valid_until": "2027-01-15"
    }
  },
  
  "data_inventory": {
    "total_categories": 8,
    "total_items": 15234,
    "total_size_bytes": 5368709120,
    "categories": {
      "social_media": {
        "item_count": 3456,
        "size_bytes": 1073741824,
        "platforms": ["facebook", "instagram", "twitter"]
      },
      "financial": {
        "item_count": 890,
        "size_bytes": 52428800,
        "platforms": ["bank_a", "investment_firm_b"]
      },
      "creative_works": {
        "item_count": 234,
        "size_bytes": 2147483648,
        "platforms": ["blog", "youtube", "github"]
      },
      "communications": {
        "item_count": 8934,
        "size_bytes": 536870912,
        "platforms": ["gmail", "outlook"]
      },
      "cloud_storage": {
        "item_count": 1234,
        "size_bytes": 1073741824,
        "platforms": ["google_drive", "dropbox"]
      },
      "gaming": {
        "item_count": 56,
        "size_bytes": 104857600,
        "platforms": ["steam", "playstation"]
      },
      "health": {
        "item_count": 123,
        "size_bytes": 10485760,
        "platforms": ["fitbit", "apple_health"]
      },
      "professional": {
        "item_count": 307,
        "size_bytes": 268435456,
        "platforms": ["linkedin", "github"]
      }
    }
  },
  
  "encryption": {
    "algorithm": "AES-256-GCM",
    "key_derivation": "PBKDF2",
    "iterations": 100000,
    "salt": "base64_encoded_salt",
    "iv": "base64_encoded_iv",
    "auth_tag": "base64_encoded_tag"
  },
  
  "data": {
    "social_media": [...],
    "financial": [...],
    "creative_works": [...],
    "communications": [...],
    "cloud_storage": [...],
    "gaming": [...],
    "health": [...],
    "professional": [...]
  },
  
  "metadata": {
    "export_method": "automated",
    "export_tool": "wia-leg-008-sdk v1.0.0",
    "checksum": "sha256:full_package_hash",
    "signature": "executor_digital_signature",
    "audit_trail": [
      {
        "timestamp": "2025-01-15T10:00:00Z",
        "action": "export_initiated",
        "actor": "did:wia:executor789"
      },
      {
        "timestamp": "2025-01-15T10:30:00Z",
        "action": "export_completed",
        "actor": "system"
      }
    ]
  }
}
```

#### 3.1.2 Category-Specific Schemas

**Social Media Data:**

```json
{
  "@type": "SocialMediaData",
  "platform": "facebook",
  "account_id": "user_facebook_id",
  "profile": {
    "username": "johndoe",
    "display_name": "John Doe",
    "bio": "...",
    "profile_picture_url": "https://...",
    "cover_photo_url": "https://..."
  },
  "posts": [
    {
      "id": "post_123",
      "created_at": "2024-12-01T15:30:00Z",
      "content": "Post content...",
      "media": [...],
      "reactions": {...},
      "comments": [...]
    }
  ],
  "photos": [...],
  "videos": [...],
  "friends": [...],
  "messages": [...]
}
```

**Financial Data:**

```json
{
  "@type": "FinancialData",
  "institution": "Bank of Example",
  "account_type": "checking",
  "account_number": "****1234",
  "transactions": [
    {
      "id": "txn_456",
      "date": "2024-12-15",
      "description": "Purchase at Store",
      "amount": -45.67,
      "currency": "USD",
      "category": "groceries"
    }
  ],
  "statements": [...],
  "tax_documents": [...]
}
```

### 3.2 Alternative Formats

#### 3.2.1 XML Format

```xml
<?xml version="1.0" encoding="UTF-8"?>
<DataPortabilityPackage xmlns="https://schema.wiastandards.com/leg-008/v1">
  <id>urn:uuid:550e8400-e29b-41d4-a716-446655440000</id>
  <version>1.0</version>
  <deceased>
    <id>did:wia:123456</id>
    <name>John Doe</name>
  </deceased>
  <!-- ... -->
</DataPortabilityPackage>
```

#### 3.2.2 CSV Format (for tabular data)

Used for specific data categories like transactions, contacts:

```csv
category,platform,item_type,item_id,created_at,content,size_bytes
social_media,facebook,post,post_123,2024-12-01T15:30:00Z,"Post content...",1024
social_media,instagram,photo,photo_456,2024-12-05T10:00:00Z,"",2048000
financial,bank_a,transaction,txn_789,2024-12-15T00:00:00Z,"Purchase at Store",256
```

---

## 4. Cross-Platform Transfer Protocols

### 4.1 Transfer Workflow

```
┌──────────────────────────────────────────────────────────┐
│                  Transfer Initiation                      │
│  1. Executor authenticates                                │
│  2. Select source and destination platforms               │
│  3. Choose data categories to transfer                    │
└──────────────────────────────────────────────────────────┘
                           │
                           ▼
┌──────────────────────────────────────────────────────────┐
│                  Authorization Verification               │
│  1. Verify executor credentials                           │
│  2. Check platform ToS compliance                         │
│  3. Validate consent records                              │
└──────────────────────────────────────────────────────────┘
                           │
                           ▼
┌──────────────────────────────────────────────────────────┐
│                  Data Export from Source                  │
│  1. Generate DPP from source platform                     │
│  2. Encrypt sensitive data                                │
│  3. Create export manifest                                │
└──────────────────────────────────────────────────────────┘
                           │
                           ▼
┌──────────────────────────────────────────────────────────┐
│                  Data Transformation                      │
│  1. Map source schema to destination schema               │
│  2. Convert data formats                                  │
│  3. Validate data integrity                               │
└──────────────────────────────────────────────────────────┘
                           │
                           ▼
┌──────────────────────────────────────────────────────────┐
│                  Data Import to Destination               │
│  1. Create import request                                 │
│  2. Upload transformed data                               │
│  3. Verify import success                                 │
└──────────────────────────────────────────────────────────┘
                           │
                           ▼
┌──────────────────────────────────────────────────────────┐
│                  Verification & Audit                     │
│  1. Generate transfer completion report                   │
│  2. Update audit trail                                    │
│  3. Notify executor                                       │
└──────────────────────────────────────────────────────────┘
```

### 4.2 API Specifications

#### 4.2.1 Export Initiation

```http
POST /api/v1/export/initiate
Authorization: Bearer {executor_jwt_token}
Content-Type: application/json

{
  "deceased_id": "did:wia:123456",
  "executor_id": "did:wia:executor789",
  "platform": "facebook",
  "categories": ["posts", "photos", "messages"],
  "format": "json-ld",
  "encryption": {
    "enabled": true,
    "algorithm": "AES-256-GCM"
  }
}

Response:
{
  "export_id": "exp_abc123",
  "status": "initiated",
  "estimated_completion": "2025-01-15T12:00:00Z",
  "estimated_size_bytes": 1073741824
}
```

#### 4.2.2 Export Status Check

```http
GET /api/v1/export/{export_id}/status
Authorization: Bearer {executor_jwt_token}

Response:
{
  "export_id": "exp_abc123",
  "status": "in_progress",
  "progress_percent": 45,
  "items_processed": 1560,
  "items_total": 3456,
  "current_category": "photos"
}
```

#### 4.2.3 Export Download

```http
GET /api/v1/export/{export_id}/download
Authorization: Bearer {executor_jwt_token}

Response:
HTTP/1.1 200 OK
Content-Type: application/json+ld
Content-Disposition: attachment; filename="dpp_123456_20250115.json"
Content-Length: 1073741824

{DPP content}
```


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

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-1-DATA-FORMAT.
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
for PHASE-1-DATA-FORMAT. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P1-DATA-FORMAT-Edge-only/2` is asserting conformance with
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
  is published manually. Sufficient for PHASE-1-DATA-FORMAT validation when the
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
