# WIA-DATA-006: Data Governance - PHASE 1: DATA FORMAT

**Version:** 1.0
**Status:** DRAFT
**Last Updated:** 2025-01-15

---

## Overview

Phase 1 establishes the foundational data formats and structures for implementing data governance across your organization. This phase defines how governance metadata, policies, and catalogs are represented and stored.

## Goals

- Define standard data formats for governance artifacts
- Establish metadata schemas for data assets
- Create policy document structures
- Enable interoperability between governance tools

## Core Data Formats

### 1. Data Asset Metadata Format

```json
{
  "assetId": "uuid-v4",
  "assetName": "string",
  "assetType": "database|table|file|api|stream",
  "classification": "public|internal|confidential|restricted",
  "domain": "string",
  "description": "string",
  "owner": {
    "email": "string",
    "name": "string",
    "department": "string"
  },
  "stewards": [
    {
      "email": "string",
      "name": "string",
      "role": "business|technical|executive"
    }
  ],
  "technicalMetadata": {
    "location": "string",
    "format": "string",
    "schema": {},
    "size": "number",
    "rowCount": "number",
    "lastUpdated": "ISO-8601 datetime"
  },
  "businessMetadata": {
    "businessTerms": ["string"],
    "tags": ["string"],
    "certificationStatus": "certified|draft|deprecated",
    "usageGuidelines": "string"
  },
  "qualityMetadata": {
    "qualityScore": "number (0-100)",
    "completeness": "number (0-100)",
    "accuracy": "number (0-100)",
    "consistency": "number (0-100)",
    "timeliness": "number (0-100)",
    "lastAssessed": "ISO-8601 datetime"
  },
  "lineage": {
    "upstreamAssets": ["assetId"],
    "downstreamAssets": ["assetId"],
    "transformations": [
      {
        "transformationId": "string",
        "description": "string",
        "logic": "string"
      }
    ]
  },
  "compliance": {
    "regulations": ["GDPR|CCPA|HIPAA|SOX"],
    "retentionPeriod": "ISO-8601 duration",
    "privacyLevel": "pii|sensitive|non-sensitive"
  },
  "createdAt": "ISO-8601 datetime",
  "updatedAt": "ISO-8601 datetime"
}
```

### 2. Policy Document Format

```json
{
  "policyId": "uuid-v4",
  "policyName": "string",
  "policyType": "quality|security|privacy|retention|access",
  "version": "string (semver)",
  "status": "draft|active|deprecated",
  "effectiveDate": "ISO-8601 date",
  "expirationDate": "ISO-8601 date",
  "owner": {
    "email": "string",
    "name": "string"
  },
  "approvers": [
    {
      "email": "string",
      "name": "string",
      "approvedAt": "ISO-8601 datetime"
    }
  ],
  "scope": {
    "domains": ["string"],
    "assetTypes": ["string"],
    "applicability": "organization|domain|asset"
  },
  "statement": "string",
  "requirements": [
    {
      "requirementId": "string",
      "description": "string",
      "mandatory": "boolean",
      "controls": ["string"]
    }
  ],
  "exceptions": [
    {
      "exceptionId": "string",
      "description": "string",
      "approver": "string",
      "expiresAt": "ISO-8601 datetime"
    }
  ],
  "relatedPolicies": ["policyId"],
  "createdAt": "ISO-8601 datetime",
  "updatedAt": "ISO-8601 datetime"
}
```

### 3. Business Glossary Term Format

```json
{
  "termId": "uuid-v4",
  "termName": "string",
  "definition": "string",
  "synonyms": ["string"],
  "relatedTerms": ["termId"],
  "domain": "string",
  "owner": {
    "email": "string",
    "name": "string"
  },
  "status": "draft|approved|deprecated",
  "examples": ["string"],
  "businessRules": [
    {
      "ruleId": "string",
      "description": "string",
      "expression": "string"
    }
  ],
  "dataAssets": ["assetId"],
  "approvedBy": {
    "email": "string",
    "name": "string",
    "approvedAt": "ISO-8601 datetime"
  },
  "createdAt": "ISO-8601 datetime",
  "updatedAt": "ISO-8601 datetime"
}
```

### 4. Access Request Format

```json
{
  "requestId": "uuid-v4",
  "requestType": "access|modification|deletion",
  "requester": {
    "email": "string",
    "name": "string",
    "department": "string"
  },
  "assetId": "string",
  "accessLevel": "read|write|admin",
  "justification": "string",
  "duration": "ISO-8601 duration",
  "status": "pending|approved|rejected|expired",
  "workflow": [
    {
      "step": "number",
      "approver": "string",
      "status": "pending|approved|rejected",
      "comments": "string",
      "timestamp": "ISO-8601 datetime"
    }
  ],
  "grantedAt": "ISO-8601 datetime",
  "expiresAt": "ISO-8601 datetime",
  "createdAt": "ISO-8601 datetime",
  "updatedAt": "ISO-8601 datetime"
}
```

### 5. Data Quality Rule Format

```json
{
  "ruleId": "uuid-v4",
  "ruleName": "string",
  "ruleType": "completeness|accuracy|consistency|timeliness|validity",
  "description": "string",
  "assetId": "string",
  "field": "string",
  "expression": "string",
  "threshold": {
    "operator": "gt|gte|lt|lte|eq|ne",
    "value": "number"
  },
  "severity": "critical|high|medium|low",
  "active": "boolean",
  "schedule": "cron expression",
  "lastExecuted": "ISO-8601 datetime",
  "lastResult": {
    "passed": "boolean",
    "score": "number",
    "failureCount": "number",
    "executedAt": "ISO-8601 datetime"
  },
  "notifications": [
    {
      "email": "string",
      "severity": ["critical|high|medium|low"]
    }
  ],
  "createdAt": "ISO-8601 datetime",
  "updatedAt": "ISO-8601 datetime"
}
```

## File Formats

### Supported Formats

1. **JSON** - Primary format for APIs and modern systems
2. **YAML** - Human-readable format for configuration
3. **XML** - Legacy system compatibility
4. **CSV** - Bulk imports and exports
5. **Parquet** - Large-scale data processing

### Naming Conventions

- **Metadata Files:** `{asset-type}_{asset-name}_metadata.json`
- **Policy Files:** `policy_{policy-type}_{version}.json`
- **Glossary Files:** `glossary_{domain}_{timestamp}.json`
- **Quality Rules:** `quality_rules_{asset-id}.json`

## Data Storage Requirements

### Metadata Repository

- **Technology:** JSON documents in document database (MongoDB, DynamoDB) or relational database
- **Versioning:** All metadata changes must be versioned
- **Retention:** Minimum 7 years for compliance
- **Backup:** Daily incremental, weekly full backups

### Policy Store

- **Technology:** Version-controlled repository (Git)
- **Format:** JSON or YAML with schema validation
- **Approval:** Digital signatures for policy approvals
- **History:** Complete audit trail of changes

## Validation

### Schema Validation

All data formats must be validated against JSON Schema:

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "type": "object",
  "required": ["assetId", "assetName", "assetType", "owner"],
  "properties": {
    "assetId": {
      "type": "string",
      "format": "uuid"
    },
    "assetName": {
      "type": "string",
      "minLength": 1,
      "maxLength": 255
    }
  }
}
```

### Quality Checks

- **Completeness:** All required fields must be populated
- **Accuracy:** Data types and formats must match schema
- **Consistency:** Cross-references must be valid
- **Timeliness:** Timestamps must be current

## Migration Path

### From Legacy Systems

1. **Assessment:** Identify existing governance data
2. **Mapping:** Map legacy formats to WIA-DATA-006 formats
3. **Transformation:** Convert data using ETL processes
4. **Validation:** Verify data integrity
5. **Cutover:** Switch to new format

### Backward Compatibility

- Support legacy formats for 12 months
- Provide conversion utilities
- Document migration process
- Offer support during transition

## Next Steps

After implementing Phase 1 data formats:

1. **Test** data format implementations
2. **Validate** schema compliance
3. **Document** any customizations
4. **Proceed** to Phase 2: API implementation

---

**弘益人間 (홍익인간) · Benefit All Humanity**

© 2025 SmileStory Inc. / WIA

---

## Annex A — Conformance Tier Matrix

WIA conformance for data-governance is evaluated across three tiers:

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

- `wia-standards/standards/data-governance/api/` — TypeScript SDK skeleton
- `wia-standards/standards/data-governance/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/data-governance/simulator/` — interactive browser-based simulator for the PHASE protocol

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
