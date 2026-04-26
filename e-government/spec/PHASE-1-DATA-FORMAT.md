# WIA-SOC-003 Phase 1: Data Format Specification

**Version:** 1.0.0
**Status:** Approved
**Last Updated:** 2025-12-26

弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. Overview

Phase 1 defines the standardized data formats for e-government services, including citizen identity representation, service request structures, document formats, and transaction logs. All data MUST use JSON-LD format for semantic interoperability.

## 2. Core Data Types

### 2.1 Citizen Identity

```json
{
  "@context": "https://wiastandards.com/soc-003/v1",
  "@type": "CitizenIdentity",
  "citizenId": "urn:uuid:550e8400-e29b-41d4-a716-446655440000",
  "identityNumber": "encrypted-id-hash",
  "givenName": "MinJun",
  "familyName": "Kim",
  "dateOfBirth": "1990-05-15",
  "nationality": "KR",
  "residency": {
    "country": "KR",
    "region": "Seoul",
    "district": "Gangnam-gu",
    "postalCode": "06000"
  },
  "contact": {
    "email": "kim.minjun@example.com",
    "phone": "+82-10-1234-5678",
    "preferredLanguage": "ko"
  },
  "verification": {
    "level": "high",
    "methods": ["biometric", "certificate"],
    "lastVerified": "2025-12-26T10:30:00Z"
  },
  "privacySettings": {
    "dataSharing": "minimal",
    "thirdPartyAccess": false,
    "retentionPeriod": "P7Y"
  }
}
```

### 2.2 Service Request

```json
{
  "@type": "ServiceRequest",
  "requestId": "REQ-2025-001234",
  "citizenId": "urn:uuid:550e8400-e29b-41d4-a716-446655440000",
  "serviceType": "tax_filing",
  "category": "financial",
  "priority": "normal",
  "status": "submitted",
  "createdAt": "2025-12-26T14:30:00Z",
  "updatedAt": "2025-12-26T14:30:00Z",
  "estimatedCompletion": "2025-12-30T17:00:00Z",
  "metadata": {
    "taxYear": 2024,
    "filingType": "individual",
    "returnType": "standard"
  },
  "documents": [
    {
      "documentId": "DOC-2025-5678",
      "type": "tax_form",
      "format": "application/pdf",
      "size": 2458624,
      "hash": "sha256:abc123...",
      "encrypted": true
    }
  ],
  "attachments": [],
  "notes": "First time filing, requesting assistance",
  "assignedTo": "dept-taxation-001",
  "tracking": {
    "steps": [
      {
        "step": "submission",
        "completedAt": "2025-12-26T14:30:00Z",
        "status": "completed"
      },
      {
        "step": "verification",
        "status": "in_progress"
      },
      {
        "step": "processing",
        "status": "pending"
      },
      {
        "step": "approval",
        "status": "pending"
      }
    ]
  }
}
```

### 2.3 Government Service

```json
{
  "@type": "GovernmentService",
  "serviceId": "SRV-TAX-001",
  "serviceName": {
    "en": "Individual Tax Filing",
    "ko": "개인 세금 신고",
    "zh": "个人纳税申报"
  },
  "description": {
    "en": "File your annual income tax returns online",
    "ko": "연간 소득세 신고를 온라인으로 제출하세요"
  },
  "category": "taxation",
  "department": "Ministry of Economy and Finance",
  "availability": {
    "hours": "24/7",
    "maintenanceWindow": "Sunday 02:00-04:00 KST"
  },
  "requirements": {
    "authentication": "high",
    "documents": [
      "income_statement",
      "tax_withholding_certificate",
      "expense_receipts"
    ],
    "eligibility": {
      "minAge": 19,
      "citizenshipRequired": true
    }
  },
  "processing": {
    "averageTime": "P3D",
    "maxTime": "P14D",
    "autoApproval": false
  },
  "fees": {
    "serviceFee": 0,
    "currency": "KRW",
    "paymentMethods": ["bank_transfer", "credit_card", "digital_wallet"]
  },
  "endpoints": {
    "submitRequest": "/api/v1/services/tax/submit",
    "checkStatus": "/api/v1/services/tax/status/{requestId}",
    "uploadDocument": "/api/v1/documents/upload"
  }
}
```

### 2.4 Document Format

```json
{
  "@type": "GovernmentDocument",
  "documentId": "DOC-2025-5678",
  "type": "certificate",
  "subtype": "birth_certificate",
  "issuedBy": {
    "authority": "Seoul Metropolitan Government",
    "department": "Civil Affairs Bureau",
    "officerName": "Park SuJin",
    "officerId": "OFC-2025-1234"
  },
  "issuedTo": {
    "citizenId": "urn:uuid:550e8400-e29b-41d4-a716-446655440000",
    "name": "Kim MinJun"
  },
  "issuedDate": "2025-12-26",
  "validFrom": "2025-12-26",
  "validUntil": "2035-12-26",
  "documentNumber": "CERT-2025-BC-1234567",
  "content": {
    "format": "application/pdf",
    "encoding": "base64",
    "data": "encrypted-content-blob",
    "hash": "sha256:def456..."
  },
  "verification": {
    "method": "digital_signature",
    "algorithm": "RSA-4096",
    "signature": "signature-data",
    "certificate": "X.509-certificate-chain"
  },
  "qrCode": {
    "data": "https://verify.gov.kr/doc/CERT-2025-BC-1234567",
    "format": "QR_CODE",
    "version": 10
  },
  "blockchain": {
    "network": "government-blockchain",
    "txHash": "0x123abc...",
    "blockNumber": 1234567
  }
}
```

### 2.5 Transaction Log

```json
{
  "@type": "TransactionLog",
  "transactionId": "TXN-2025-9012",
  "timestamp": "2025-12-26T14:30:00.123Z",
  "type": "service_request",
  "action": "submit",
  "actor": {
    "type": "citizen",
    "id": "urn:uuid:550e8400-e29b-41d4-a716-446655440000",
    "ip": "203.0.113.42",
    "userAgent": "WIA-Gov-App/1.2.3 (iOS 17.0)"
  },
  "target": {
    "type": "service",
    "id": "SRV-TAX-001",
    "requestId": "REQ-2025-001234"
  },
  "result": {
    "status": "success",
    "code": 200,
    "message": "Request submitted successfully"
  },
  "audit": {
    "dataAccessed": ["citizen_profile", "tax_history"],
    "dataModified": ["service_requests"],
    "permissionsUsed": ["service.tax.submit"],
    "complianceFlags": ["GDPR", "CCPA"]
  },
  "performance": {
    "responseTime": 245,
    "serverRegion": "ap-northeast-2",
    "requestSize": 15840,
    "responseSize": 1024
  }
}
```

## 3. Data Validation Rules

### 3.1 Citizen Identity

| Field | Type | Required | Validation |
|-------|------|----------|------------|
| citizenId | UUID | Yes | Valid UUID v4 |
| identityNumber | String | Yes | Encrypted, country-specific format |
| givenName | String | Yes | 1-100 chars, Unicode |
| familyName | String | Yes | 1-100 chars, Unicode |
| dateOfBirth | Date | Yes | ISO 8601, age >= 0 |
| nationality | String | Yes | ISO 3166-1 alpha-2 |
| email | Email | No | RFC 5322 compliant |
| phone | String | No | E.164 format |

### 3.2 Service Request

| Field | Type | Required | Validation |
|-------|------|----------|------------|
| requestId | String | Yes | Pattern: REQ-YYYY-NNNNNN |
| citizenId | UUID | Yes | Valid citizen in system |
| serviceType | String | Yes | Enum of valid services |
| priority | String | Yes | [normal, urgent, critical] |
| status | String | Yes | [submitted, in_progress, completed, rejected] |
| createdAt | DateTime | Yes | ISO 8601 with timezone |

### 3.3 Document

| Field | Type | Required | Validation |
|-------|------|----------|------------|
| documentId | String | Yes | Pattern: DOC-YYYY-NNNN |
| type | String | Yes | Enum of document types |
| content.format | String | Yes | MIME type |
| content.size | Integer | Yes | 0 < size <= 50MB |
| content.hash | String | Yes | SHA-256 hash |
| verification.signature | String | Yes (for official docs) | Valid digital signature |

## 4. Semantic Vocabularies

### 4.1 Service Categories

```json
{
  "taxation": {
    "label": {"en": "Taxation", "ko": "세무"},
    "subcategories": ["income_tax", "corporate_tax", "property_tax", "vat"]
  },
  "healthcare": {
    "label": {"en": "Healthcare", "ko": "보건의료"},
    "subcategories": ["medical_records", "insurance", "prescriptions"]
  },
  "civil_affairs": {
    "label": {"en": "Civil Affairs", "ko": "민원"},
    "subcategories": ["certificates", "registration", "permits"]
  },
  "legal": {
    "label": {"en": "Legal Services", "ko": "법률 서비스"},
    "subcategories": ["business_registration", "licenses", "legal_aid"]
  },
  "social_welfare": {
    "label": {"en": "Social Welfare", "ko": "사회복지"},
    "subcategories": ["benefits", "pensions", "disability_support"]
  }
}
```

### 4.2 Status Codes

```json
{
  "service_request": {
    "submitted": {"code": 100, "label": "Submitted"},
    "verified": {"code": 200, "label": "Verified"},
    "in_progress": {"code": 300, "label": "In Progress"},
    "pending_approval": {"code": 400, "label": "Pending Approval"},
    "approved": {"code": 500, "label": "Approved"},
    "completed": {"code": 600, "label": "Completed"},
    "rejected": {"code": 700, "label": "Rejected"},
    "cancelled": {"code": 800, "label": "Cancelled"}
  }
}
```

## 5. Encryption and Security

### 5.1 Data at Rest

- **Encryption**: AES-256-GCM
- **Key Management**: HSM (Hardware Security Module)
- **Key Rotation**: Every 90 days
- **PII Fields**: Always encrypted

### 5.2 Data in Transit

- **Protocol**: TLS 1.3
- **Cipher Suites**: ECDHE-RSA-AES256-GCM-SHA384
- **Certificate**: X.509 with OCSP stapling
- **Perfect Forward Secrecy**: Required

### 5.3 Sensitive Data Handling

```json
{
  "identityNumber": {
    "storage": "encrypted",
    "algorithm": "AES-256-GCM",
    "keyDerivation": "PBKDF2-SHA256",
    "display": "masked (last 4 digits only)",
    "logging": "hashed (SHA-256)"
  },
  "biometricData": {
    "storage": "encrypted + salted hash",
    "comparison": "homomorphic encryption",
    "retention": "until citizen removal request",
    "backup": "air-gapped storage"
  }
}
```

## 6. Data Retention Policies

| Data Type | Retention Period | Deletion Method |
|-----------|------------------|-----------------|
| Citizen Profile | Active + 7 years after death | Secure wipe (DoD 5220.22-M) |
| Service Requests | 7 years | Archival then secure deletion |
| Transaction Logs | 10 years | Compliance archival |
| Documents | Varies by type | According to legal requirements |
| Audit Trails | 10 years | Immutable blockchain storage |

## 7. Cross-Border Data Exchange

### 7.1 Data Transfer Format

```json
{
  "@type": "CrossBorderDataTransfer",
  "sourceCountry": "KR",
  "targetCountry": "JP",
  "legalBasis": {
    "agreement": "KR-JP-DataSharing-2024",
    "articleNumber": "Article 7.2",
    "purpose": "background_check"
  },
  "data": {
    "type": "citizen_verification",
    "minimized": true,
    "encrypted": true,
    "fields": ["name", "dateOfBirth", "citizenship"]
  },
  "consent": {
    "obtained": true,
    "timestamp": "2025-12-26T10:00:00Z",
    "expiry": "2026-12-26T10:00:00Z"
  },
  "audit": {
    "requestId": "XBORDER-2025-1234",
    "requestedBy": "immigration-dept-kr",
    "approvedBy": "data-protection-officer-kr"
  }
}
```

## 8. Accessibility Requirements

### 8.1 Multi-Language Support

All text fields MUST support:
- **Primary Language**: Based on citizen preference
- **Fallback**: English (en)
- **RTL Support**: Arabic, Hebrew, Persian, Urdu
- **Character Sets**: Unicode (UTF-8)

### 8.2 Alternative Formats

```json
{
  "document": {
    "formats": ["pdf", "html", "txt", "audio", "braille"],
    "accessibility": {
      "screenReader": "ARIA labels",
      "highContrast": true,
      "fontSize": "scalable",
      "voiceOver": "available"
    }
  }
}
```

## 9. Versioning and Compatibility

### 9.1 API Version

```json
{
  "@context": "https://wiastandards.com/soc-003/v1",
  "@version": "1.0.0",
  "compatibleWith": ["1.0.x"],
  "deprecations": [],
  "migrations": {
    "from": "0.9.x",
    "guide": "https://wiastandards.com/soc-003/migration"
  }
}
```

### 9.2 Backward Compatibility

- **Minor versions**: Must be backward compatible
- **Major versions**: Breaking changes allowed with migration path
- **Deprecation**: 12 months notice required

---

**弘益人間 · Benefit All Humanity**

© 2025 WIA / SmileStory Inc.


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
