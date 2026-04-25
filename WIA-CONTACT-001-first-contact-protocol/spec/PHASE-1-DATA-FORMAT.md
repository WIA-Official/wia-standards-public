# PHASE 1: DATA FORMAT SPECIFICATION
## WIA-CONTACT-001: First Contact Protocol

> 弘益人間 (홍익인간) · Benefit All Humanity

---

## 1. Introduction

This document defines the data format specifications for storing, transmitting, and processing extraterrestrial signal data within the First Contact Protocol framework.

## 2. Signal Data Format

### 2.1 Base Signal Structure

```json
{
  "version": "1.0",
  "signalId": "SIG-YYYY-NNNN",
  "detectionTimestamp": "ISO8601",
  "observatory": {
    "id": "string",
    "name": "string",
    "location": {
      "latitude": number,
      "longitude": number,
      "altitude": number
    }
  },
  "frequency": {
    "center": number,
    "bandwidth": number,
    "unit": "MHz"
  },
  "source": {
    "rightAscension": number,
    "declination": number,
    "galacticLatitude": number,
    "galacticLongitude": number,
    "epoch": "J2000"
  },
  "strength": {
    "value": number,
    "unit": "dBm"
  },
  "modulation": {
    "type": "am|fm|pm|pulse|binary|complex|unknown",
    "parameters": {}
  },
  "duration": {
    "value": number,
    "unit": "seconds"
  },
  "rawData": "base64|uri",
  "metadata": {}
}
```

### 2.2 Field Specifications

#### 2.2.1 Signal ID Format
- Pattern: `SIG-YYYY-NNNN`
- YYYY: Year of detection
- NNNN: Sequential number (0001-9999)
- Example: `SIG-2025-0042`

#### 2.2.2 Timestamp Format
- Standard: ISO 8601
- Format: `YYYY-MM-DDTHH:mm:ss.sssZ`
- Timezone: UTC
- Example: `2025-12-27T14:23:45.123Z`

#### 2.2.3 Frequency Specification
- Range: 1 MHz - 100 GHz
- Precision: 0.001 MHz (1 kHz)
- Notable frequencies:
  - 1420.4 MHz: Hydrogen line
  - 1666.7 MHz: Hydroxyl line
  - "Water Hole": 1420.4 - 1666.7 MHz

#### 2.2.4 Coordinate System
- Right Ascension: 0° - 360°
- Declination: -90° - +90°
- Epoch: J2000.0 (default)
- Precision: 0.001 degrees

## 3. Verification Data Format

### 3.1 Verification Record

```json
{
  "verificationId": "VER-YYYY-NNNN",
  "signalId": "SIG-YYYY-NNNN",
  "initiatedAt": "ISO8601",
  "completedAt": "ISO8601",
  "observatories": [
    {
      "observatoryId": "string",
      "observatoryName": "string",
      "confirmed": boolean,
      "confidence": number,
      "timestamp": "ISO8601",
      "notes": "string"
    }
  ],
  "consensus": number,
  "authentic": boolean,
  "status": "pending|in-progress|completed|failed"
}
```

### 3.2 Multi-Site Verification Requirements
- Minimum observatories: 3
- Minimum consensus: 0.95 (95%)
- Maximum time window: 48 hours
- Geographic diversity: 3+ continents

## 4. Pattern Analysis Format

### 4.1 Pattern Data Structure

```json
{
  "analysisId": "ANA-YYYY-NNNN",
  "signalId": "SIG-YYYY-NNNN",
  "timestamp": "ISO8601",
  "patternType": "prime-sequence|fibonacci|mathematical-constants|binary-encoding|pictorial|audio-signature|unknown",
  "patterns": [
    {
      "sequence": [],
      "interpretation": "string",
      "confidence": number
    }
  ],
  "mathematicalSignificance": "low|medium|high",
  "artificialProbability": number,
  "entropy": number,
  "decodedContent": "string",
  "recommendations": []
}
```

### 4.2 Pattern Recognition Algorithms
- Prime number detection
- Fibonacci sequence identification
- Mathematical constant recognition
- Binary encoding analysis
- Pictorial data reconstruction
- Audio signature extraction

## 5. Threat Assessment Format

### 5.1 Threat Data Structure

```json
{
  "assessmentId": "THR-YYYY-NNNN",
  "signalId": "SIG-YYYY-NNNN",
  "timestamp": "ISO8601",
  "threatLevel": "benign|caution|elevated|high|critical",
  "indicators": [
    {
      "type": "string",
      "description": "string",
      "severity": "low|medium|high"
    }
  ],
  "confidence": number,
  "recommendation": "string",
  "mitigationStrategies": []
}
```

### 5.2 Threat Levels
- **Benign**: No hostile indicators, peaceful intent likely
- **Caution**: Unknown intent, monitoring required
- **Elevated**: Potentially concerning patterns detected
- **High**: Hostile indicators present
- **Critical**: Immediate threat identified

## 6. Response Message Format

### 6.1 Response Structure

```json
{
  "responseId": "RES-YYYY-NNNN",
  "signalId": "SIG-YYYY-NNNN",
  "createdAt": "ISO8601",
  "type": "mathematical-acknowledgment|pictorial-message|audio-signal|binary-code|custom",
  "content": {
    "encoding": "text|binary|mathematical|pictorial|audio",
    "data": "base64|string",
    "checksum": "sha256"
  },
  "transmission": {
    "frequency": number,
    "power": number,
    "duration": number,
    "targetCoordinates": {}
  },
  "approvals": [
    {
      "authority": "string",
      "approved": boolean,
      "timestamp": "ISO8601",
      "votes": {
        "for": number,
        "against": number,
        "abstain": number
      }
    }
  ],
  "status": "draft|under-review|approved|rejected|transmitted"
}
```

## 7. Data Storage Requirements

### 7.1 Storage Specifications
- **Format**: JSON, HDF5, FITS
- **Compression**: GZIP, BZIP2
- **Encryption**: AES-256
- **Backup**: 3-2-1 strategy (3 copies, 2 media types, 1 offsite)
- **Retention**: Permanent (minimum 100 years)

### 7.2 Access Control
- Classification levels: Public, Restricted, Classified
- Role-based access control (RBAC)
- Audit logging required
- Multi-factor authentication for classified data

## 8. Data Exchange Format

### 8.1 Standard Exchange Format
- Primary: JSON-LD with schema.org ontology
- Secondary: XML with custom XSD schema
- Binary: Protocol Buffers for high-performance scenarios

### 8.2 API Data Format
- RESTful JSON API
- GraphQL support
- gRPC for real-time streaming
- WebSocket for live monitoring

## 9. Validation Rules

### 9.1 Required Fields
- signalId
- detectionTimestamp
- observatory
- frequency
- source
- strength

### 9.2 Data Quality Checks
- Frequency range validation
- Coordinate bounds checking
- Timestamp validity
- Signal strength reasonableness
- Data completeness verification

## 10. Version Control

### 10.1 Versioning Scheme
- Format: MAJOR.MINOR.PATCH
- Current version: 1.0.0
- Backward compatibility guaranteed within major versions

### 10.2 Migration Procedures
- Automated migration tools provided
- 6-month deprecation notice for breaking changes
- Legacy format support: 3 years minimum

---

**Document Version**: 1.0.0
**Last Updated**: 2025-12-27
**Status**: Active
**Maintainer**: WIA Technical Committee

© 2025 SmileStory Inc. / WIA · CC BY-SA 4.0

---

## Annex A — Conformance Tier Matrix

WIA conformance for WIA-CONTACT-001-first-contact-protocol is evaluated across three tiers:

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

- `wia-standards/standards/WIA-CONTACT-001-first-contact-protocol/api/` — TypeScript SDK skeleton
- `wia-standards/standards/WIA-CONTACT-001-first-contact-protocol/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/WIA-CONTACT-001-first-contact-protocol/simulator/` — interactive browser-based simulator for the PHASE protocol

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
