# WIA-DATA_QUALITY: Phase 1 - Data Format Specification
**Version:** 1.0
**Status:** Official
**Philosophy:** 弘益人間 (Benefit All Humanity)

## 1. Overview

This document specifies the data format requirements for DATA QUALITY. All implementations MUST follow these specifications to ensure interoperability across systems.

## 2. Data Structures

### 2.1 Primary Record Format

```json
{
  "type": "WIA-DATA_QUALITYRecord",
  "version": "1.0",
  "id": "string (UUID v4)",
  "timestamp": "ISO 8601",
  "data": {
    "category": "string",
    "value": "any",
    "metadata": {}
  },
  "signature": "string (Ed25519)"
}
```

### 2.2 Field Definitions

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| type | string | Yes | Record type identifier |
| version | string | Yes | Specification version |
| id | UUID | Yes | Unique record identifier |
| timestamp | ISO 8601 | Yes | Record creation time |
| data | object | Yes | Primary data payload |
| signature | string | No | Cryptographic signature |

## 3. Data Types

### 3.1 Core Types

| Type | Format | Example |
|------|--------|---------|
| String | UTF-8 | "Hello World" |
| Number | IEEE 754 | 3.14159 |
| Boolean | true/false | true |
| Date | ISO 8601 | "2025-01-01T00:00:00Z" |
| UUID | RFC 4122 | "550e8400-e29b-41d4-a716-446655440000" |

### 3.2 Extended Types

```typescript
interface ExtendedData {
  binaryData: ArrayBuffer;
  coordinates: GeoCoordinate;
  range: NumberRange;
  collection: DataCollection;
}

interface GeoCoordinate {
  latitude: number;   // -90 to 90
  longitude: number;  // -180 to 180
  altitude?: number;  // meters
}

interface NumberRange {
  min: number;
  max: number;
  step?: number;
}
```

## 4. Encoding Requirements

### 4.1 Character Encoding
- All text MUST be UTF-8 encoded
- No BOM (Byte Order Mark) allowed
- Line endings: LF (Unix style)

### 4.2 Binary Encoding
- Use Base64 for binary data in JSON
- Use Protocol Buffers for high-performance scenarios
- Big-endian byte order for network transmission

## 5. Validation Rules

### 5.1 Schema Validation
```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "type": "object",
  "required": ["type", "version", "id", "timestamp", "data"],
  "properties": {
    "type": { "type": "string", "const": "WIA-DATA_QUALITYRecord" },
    "version": { "type": "string", "pattern": "^\\d+\\.\\d+$" },
    "id": { "type": "string", "format": "uuid" },
    "timestamp": { "type": "string", "format": "date-time" }
  }
}
```

### 5.2 Validation Errors

| Code | Message | Resolution |
|------|---------|------------|
| E001 | Invalid type | Use correct record type |
| E002 | Missing required field | Add all required fields |
| E003 | Invalid format | Check field format specifications |
| E004 | Schema mismatch | Validate against JSON schema |

## 6. Versioning

### 6.1 Version Format
- Major.Minor (e.g., 1.0, 2.1)
- Major version: Breaking changes
- Minor version: Backward-compatible additions

### 6.2 Migration Path
- v1.0 → v1.1: Add optional fields
- v1.x → v2.0: Schema restructuring

## 7. Examples

### 7.1 Basic Record
```json
{
  "type": "WIA-DATA_QUALITYRecord",
  "version": "1.0",
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "timestamp": "2025-01-01T00:00:00Z",
  "data": {
    "category": "standard",
    "value": "example data",
    "metadata": {
      "source": "system",
      "priority": "normal"
    }
  }
}
```

---

**弘益人間 (Benefit All Humanity)**

*© 2025 WIA - World Certification Industry Association*
*MIT License*

---

## Annex A — Conformance Tier Matrix

WIA conformance for WIA-DATA_QUALITY is evaluated across three tiers:

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

- `wia-standards/standards/WIA-DATA_QUALITY/api/` — TypeScript SDK skeleton
- `wia-standards/standards/WIA-DATA_QUALITY/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/WIA-DATA_QUALITY/simulator/` — interactive browser-based simulator for the PHASE protocol

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


## Annex G — Document Index

This PHASE document is one of four PHASE artifacts that together describe this WIA standard. The full set of related artifacts is:

- `spec/PHASE-1-DATA-FORMAT.md` — normative data formats and schema definitions.
- `spec/PHASE-2-API.md` — normative SDK and Client API contract that consumes the PHASE 1 data formats.
- `spec/PHASE-3-PROTOCOL.md` — normative real-time communication protocol bindings (where applicable).
- `spec/PHASE-4-INTEGRATION.md` — normative ecosystem integration patterns (LIMS, paging, federation, reporting).

Readers SHOULD consult all four PHASE documents in sequence when planning a deployment. The OpenAPI document published alongside this standard reflects the §4 endpoints in machine-readable form and is updated synchronously with this PHASE.

This index is informative; the normative content is in the body of each PHASE document.
