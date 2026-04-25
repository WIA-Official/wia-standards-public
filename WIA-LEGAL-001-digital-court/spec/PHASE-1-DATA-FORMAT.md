# WIA-LEGAL-001: Digital Court - Phase 1: Data Format

**Version:** 1.0
**Status:** Complete
**Last Updated:** 2025-12-27

## Overview

Phase 1 defines the standardized data formats for digital court proceedings, including case information, filing documents, evidence metadata, and hearing records.

## 1. Case Data Schema

### 1.1 Core Case Structure

```json
{
  "standard": "WIA-LEGAL-001",
  "version": "1.0",
  "caseId": "string (UUID)",
  "caseNumber": "string (jurisdiction-specific format)",
  "caseType": "enum",
  "court": {
    "courtId": "string",
    "courtName": "string",
    "jurisdiction": "string",
    "level": "enum: district|appellate|supreme"
  },
  "parties": {
    "plaintiffs": ["Party"],
    "defendants": ["Party"],
    "attorneys": ["Attorney"],
    "judges": ["Judge"]
  },
  "filingDate": "ISO 8601 timestamp",
  "status": "enum",
  "metadata": "CaseMetadata",
  "timestamp": "ISO 8601 timestamp"
}
```

### 1.2 Party Data Structure

```json
{
  "partyId": "string (UUID)",
  "type": "enum: individual|corporation|government|ngo",
  "name": "string",
  "legalName": "string",
  "did": "string (DID)",
  "contact": {
    "email": "string",
    "phone": "string",
    "address": "Address"
  },
  "representative": "Attorney (optional)",
  "role": "enum: plaintiff|defendant|third-party"
}
```

### 1.3 Document Filing Format

```json
{
  "documentId": "string (UUID)",
  "caseId": "string (UUID)",
  "type": "enum: pleading|motion|evidence|order|transcript",
  "title": "string",
  "filedBy": "string (partyId)",
  "filedDate": "ISO 8601 timestamp",
  "confidentiality": "enum: public|sealed|confidential",
  "format": "string (MIME type)",
  "hash": "string (SHA-256)",
  "signature": "DigitalSignature",
  "metadata": {
    "pageCount": "number",
    "wordCount": "number",
    "language": "string (ISO 639-1)",
    "tags": ["string"]
  },
  "storageLocation": "string (URI)",
  "timestamp": "ISO 8601 timestamp"
}
```

## 2. Evidence Metadata Format

### 2.1 Digital Evidence

```json
{
  "evidenceId": "string (UUID)",
  "caseId": "string (UUID)",
  "type": "enum: document|photo|video|audio|digital|physical",
  "description": "string",
  "submittedBy": "string (partyId)",
  "submittedDate": "ISO 8601 timestamp",
  "chainOfCustody": ["CustodyRecord"],
  "authenticity": {
    "verified": "boolean",
    "method": "string",
    "verifiedBy": "string",
    "verifiedDate": "ISO 8601 timestamp"
  },
  "forensics": {
    "hash": "string",
    "metadata": "object",
    "analysisReport": "string (URI)"
  },
  "admissibility": {
    "status": "enum: pending|admitted|excluded",
    "rulingDate": "ISO 8601 timestamp",
    "rulingJudge": "string"
  }
}
```

## 3. Hearing Record Format

### 3.1 Virtual Hearing Data

```json
{
  "hearingId": "string (UUID)",
  "caseId": "string (UUID)",
  "type": "enum: preliminary|trial|motion|sentencing|appeal",
  "scheduledDate": "ISO 8601 timestamp",
  "duration": "number (minutes)",
  "mode": "enum: in-person|virtual|hybrid",
  "participants": {
    "judge": "Judge",
    "parties": ["Party"],
    "attorneys": ["Attorney"],
    "witnesses": ["Witness"],
    "observers": ["Observer"]
  },
  "recording": {
    "enabled": "boolean",
    "videoUrl": "string (URI)",
    "audioUrl": "string (URI)",
    "transcriptUrl": "string (URI)"
  },
  "proceedings": {
    "startTime": "ISO 8601 timestamp",
    "endTime": "ISO 8601 timestamp",
    "events": ["HearingEvent"]
  },
  "outcome": {
    "ruling": "string",
    "orders": ["CourtOrder"],
    "nextHearing": "ISO 8601 timestamp (optional)"
  }
}
```

## 4. Court Order Format

```json
{
  "orderId": "string (UUID)",
  "caseId": "string (UUID)",
  "type": "enum: procedural|substantive|temporary|final",
  "title": "string",
  "issuedBy": "string (judgeId)",
  "issuedDate": "ISO 8601 timestamp",
  "effectiveDate": "ISO 8601 timestamp",
  "expirationDate": "ISO 8601 timestamp (optional)",
  "content": "string",
  "conditions": ["string"],
  "signature": "DigitalSignature",
  "enforcement": {
    "status": "enum: pending|active|completed|appealed",
    "enforcedBy": "string",
    "complianceDeadline": "ISO 8601 timestamp"
  }
}
```

## 5. Validation Rules

### 5.1 Required Fields

All case records MUST include:
- `caseId` (UUID v4)
- `caseNumber` (jurisdiction-specific)
- `caseType`
- `filingDate`
- `court` information
- At least one plaintiff and one defendant

### 5.2 Data Integrity

- All timestamps MUST be in ISO 8601 format with timezone
- All document hashes MUST use SHA-256 or stronger
- All monetary amounts MUST include currency code (ISO 4217)
- All dates MUST be timezone-aware

### 5.3 Security Requirements

- Personally Identifiable Information (PII) MUST be encrypted
- Sealed documents MUST have restricted access controls
- All signatures MUST use approved cryptographic algorithms
- Audit logs MUST be maintained for all data access

## 6. Examples

### 6.1 Complete Case Example

```json
{
  "standard": "WIA-LEGAL-001",
  "version": "1.0",
  "caseId": "550e8400-e29b-41d4-a716-446655440000",
  "caseNumber": "2025-CV-001234",
  "caseType": "civil",
  "court": {
    "courtId": "court-nysd-001",
    "courtName": "Southern District of New York",
    "jurisdiction": "US-NY",
    "level": "district"
  },
  "parties": {
    "plaintiffs": [{
      "partyId": "party-001",
      "type": "individual",
      "name": "John Doe",
      "legalName": "John Michael Doe",
      "did": "did:wia:legal:johndoe123",
      "role": "plaintiff"
    }],
    "defendants": [{
      "partyId": "party-002",
      "type": "corporation",
      "name": "Acme Corp",
      "legalName": "Acme Corporation Inc.",
      "did": "did:wia:legal:acmecorp",
      "role": "defendant"
    }]
  },
  "filingDate": "2025-01-15T09:00:00Z",
  "status": "active",
  "timestamp": "2025-01-15T09:00:00Z"
}
```

## 7. Interoperability

### 7.1 Cross-Jurisdiction Mapping

Data format supports mapping to:
- US Federal Court CM/ECF system
- UK Courts Digital Case System
- EU e-Justice Portal
- International Court of Justice systems

### 7.2 WIA Standard Integration

- **WIA-LEGAL-004**: Digital Evidence format compatibility
- **WIA-LEGAL-008**: E-Notary signature integration
- **WIA-BLOCKCHAIN-001**: Immutable record storage

---

**弘益人間 (Hongik Ingan)** - *Benefit All Humanity*

© 2025 WIA - World Certification Industry Association | MIT License

---

## Annex A — Conformance Tier Matrix

WIA conformance for WIA-LEGAL-001-digital-court is evaluated across three tiers:

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

- `wia-standards/standards/WIA-LEGAL-001-digital-court/api/` — TypeScript SDK skeleton
- `wia-standards/standards/WIA-LEGAL-001-digital-court/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/WIA-LEGAL-001-digital-court/simulator/` — interactive browser-based simulator for the PHASE protocol

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

