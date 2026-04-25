# WIA-SOC-016 PHASE 3: Protocol Specification

## Census Data Standard - Communication Protocols

**Version:** 1.0
**Status:** PUBLISHED
**Last Updated:** 2025-12-26

---

## 1. Overview

This document specifies communication protocols for census data collection, transmission, and exchange.

---

## 2. Data Collection Protocols

### 2.1 Internet Self-Enumeration

**Protocol:** HTTPS
**Methods:** POST, PUT
**Authentication:** Access code + TLS

**Workflow:**
1. Respondent receives unique access code
2. Accesses secure web portal
3. Completes questionnaire (save/resume supported)
4. Submits encrypted form data
5. Receives confirmation

### 2.2 CAPI (Computer-Assisted Personal Interviewing)

**Protocol:** HTTPS + Offline sync
**Devices:** Tablets, smartphones
**Synchronization:** WiFi, cellular data

**Features:**
- Offline data collection
- GPS coordinate capture
- Photo documentation
- Incremental sync when connected

### 2.3 Telephone Interviews (CATI)

**Protocol:** SIP (Session Initiation Protocol)
**Recording:** Optional, encrypted
**Quality Monitoring:** Real-time

---

## 3. Data Transmission Security

### 3.1 Encryption

**In Transit:** TLS 1.3 minimum
**At Rest:** AES-256
**Key Management:** HSM (Hardware Security Module)

### 3.2 Data Integrity

**Hashing:** SHA-256
**Digital Signatures:** RSA 4096 or ECDSA
**Checksums:** For file transfers

---

## 4. Inter-Agency Data Exchange

### 4.1 Secure File Transfer

**Protocols:**
- SFTP (SSH File Transfer Protocol)
- FTPS (FTP over SSL/TLS)
- AS2 (Applicability Statement 2)

### 4.2 API-Based Exchange

**RESTful APIs:** See PHASE-2
**Authentication:** OAuth 2.0, mutual TLS
**Rate Limiting:** Token bucket algorithm

### 4.3 Secure Data Rooms

**Virtual Data Rooms (VDR):**
- Controlled access
- Document watermarking
- Audit logging
- Time-limited access

---

## 5. Privacy-Preserving Protocols

### 5.1 Differential Privacy Application

**Noise Addition:** Laplace mechanism
**Privacy Budget Management:** ε tracking
**Composition:** Sequential, parallel

### 5.2 Secure Multiparty Computation

**Protocols:**
- Garbled circuits
- Secret sharing (Shamir)
- Homomorphic encryption

**Use Cases:**
- Cross-border data aggregation
- Multi-agency collaboration
- Privacy-preserving analytics

---

## 6. Quality Control Protocols

### 6.1 Real-Time Validation

**Client-side:** JavaScript validation
**Server-side:** Multi-layer validation
**Error Handling:** Immediate feedback

### 6.2 Data Quality Checks

**Automated:**
- Range checks
- Consistency checks
- Completeness checks
- Duplicate detection

**Manual:**
- Expert review
- Anomaly investigation
- Outlier analysis

---

## 7. Backup and Recovery

### 7.1 Backup Protocol

**Frequency:** Continuous (incremental), Daily (full)
**Storage:** Geographically distributed
**Encryption:** AES-256
**Retention:** 7 years minimum

### 7.2 Disaster Recovery

**RTO (Recovery Time Objective):** 4 hours
**RPO (Recovery Point Objective):** 15 minutes
**Testing:** Quarterly drills

---

## 8. Metadata Exchange

### 8.1 DDI (Data Documentation Initiative)

**Format:** DDI-Lifecycle 3.3, DDI-Codebook 2.5
**Transport:** HTTP, FTP
**Encoding:** UTF-8 XML

### 8.2 SDMX (Statistical Data and Metadata eXchange)

**Version:** SDMX 2.1
**Message Types:**
- Structure
- Data
- Metadata
- Registry

---

## 9. Logging and Auditing

### 9.1 Audit Log Format

```json
{
  "timestamp": "2025-12-26T10:30:00Z",
  "event_type": "DATA_ACCESS",
  "user_id": "analyst_123",
  "resource": "/api/v1/population/USA-CA",
  "action": "READ",
  "ip_address": "192.168.1.1",
  "status": "SUCCESS",
  "privacy_level": "PUBLIC"
}
```

### 9.2 Log Retention

**Operational Logs:** 90 days
**Audit Logs:** 7 years
**Security Logs:** 3 years

---

## 10. Compliance Protocols

### 10.1 GDPR Compliance

- Data minimization
- Purpose limitation
- Right to access
- Right to rectification
- Right to erasure (where applicable)

### 10.2 Statistical Confidentiality

**Principles:**
- No identification of individuals
- Statistical purpose only
- Secure storage and transmission
- Limited retention

---

**Document Version:** 1.0
© 2025 SmileStory Inc. / WIA

---

## Annex A — Conformance Tier Matrix

WIA conformance for census-data is evaluated across three tiers:

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

- `wia-standards/standards/census-data/api/` — TypeScript SDK skeleton
- `wia-standards/standards/census-data/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/census-data/simulator/` — interactive browser-based simulator for the PHASE protocol

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
