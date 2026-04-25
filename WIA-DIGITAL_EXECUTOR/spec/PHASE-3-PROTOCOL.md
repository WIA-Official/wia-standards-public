# WIA-DIGITAL_EXECUTOR: Phase 3 - Protocol Specification
**Version:** 1.0
**Status:** Official
**Philosophy:** 弘益人間 (Benefit All Humanity)

## 1. Overview

This document specifies the communication and operational protocols for DIGITAL EXECUTOR. All implementations MUST follow these protocols to ensure safety, consistency, and data integrity.

## 2. Communication Protocol

### 2.1 Message Structure
```
+------------------+------------------+------------------+
|  Header (64B)    |  Payload (var)   |  Signature (64B) |
+------------------+------------------+------------------+
```

### 2.2 Header Format
```json
{
  "version": "1.0",
  "messageType": "request|response|broadcast|alert",
  "messageId": "UUID",
  "source": "system-identifier",
  "destination": "system-identifier",
  "timestamp": "ISO 8601",
  "priority": "low|normal|high|critical"
}
```

### 2.3 Message Types

| Type | Code | Description |
|------|------|-------------|
| REQUEST | 0x01 | Request operation |
| RESPONSE | 0x02 | Response to request |
| BROADCAST | 0x03 | System-wide notification |
| ALERT | 0x04 | Priority notification |
| SYNC | 0x05 | Synchronization message |
| HEARTBEAT | 0x06 | Health check |

## 3. Operational Protocols

### 3.1 Initialization Protocol
```
1. CONNECT: Establish connection
2. AUTHENTICATE: Verify credentials
3. CONFIGURE: Exchange capabilities
4. SYNC: Synchronize state
5. READY: Begin operations
```

### 3.2 Data Exchange Protocol
```
1. REQUEST: Submit data request
2. VALIDATE: Check request validity
3. PROCESS: Execute operation
4. RESPOND: Return results
5. CONFIRM: Acknowledge receipt
```

### 3.3 Shutdown Protocol
```
1. NOTIFY: Announce shutdown
2. FLUSH: Complete pending operations
3. SYNC: Final state synchronization
4. DISCONNECT: Close connections
5. CLEANUP: Release resources
```

## 4. Safety Protocols

### 4.1 Error Handling Levels
```yaml
Level 1 - Warning:
  - Log event
  - Continue operation
  - Monitor for escalation

Level 2 - Caution:
  - Log with notification
  - Restrict operations
  - Prepare recovery

Level 3 - Alert:
  - Halt new operations
  - Notify administrators
  - Initiate recovery

Level 4 - Critical:
  - Emergency shutdown
  - Full system isolation
  - Immediate investigation
```

### 4.2 Recovery Protocol
```
TRIGGER CONDITIONS:
- Data corruption detected
- System failure
- Security breach
- Operator request

PROCEDURE:
1. Isolate affected systems
2. Assess damage scope
3. Initiate backup restoration
4. Verify data integrity
5. Resume operations
6. Document incident
```

## 5. Synchronization Protocol

### 5.1 State Sync
```json
{
  "protocol": "WIA_SYNC_V1",
  "participants": ["system-1", "system-2"],
  "method": "consensus",
  "timestamp": "ISO 8601",
  "checkpoints": [
    { "id": "checkpoint-1", "hash": "sha256", "verified": true }
  ]
}
```

### 5.2 Clock Synchronization
```
1. Exchange timestamps
2. Calculate round-trip time
3. Adjust for network latency
4. Verify with reference
5. Establish synchronized time
```

## 6. Security Protocol

### 6.1 Encryption
- Transport: TLS 1.3
- Data at rest: AES-256-GCM
- Key exchange: ECDHE
- Signatures: Ed25519

### 6.2 Authentication
```
Root CA → Intermediate CA → Service Certificate → Client Certificate
```

### 6.3 Access Control
| Level | Access |
|-------|--------|
| Observer | Read only |
| Operator | Read/Write |
| Administrator | Full access |
| Auditor | Read + Audit logs |

## 7. Logging Requirements

### 7.1 Required Logs
- All operations
- Error conditions
- Security events
- State changes
- Performance metrics

### 7.2 Log Format
```json
{
  "timestamp": "ISO 8601",
  "level": "DEBUG|INFO|WARN|ERROR|CRITICAL",
  "source": "component-id",
  "event": "event-type",
  "message": "description",
  "context": {}
}
```

### 7.3 Retention
- Active logs: 90 days
- Archive: 7 years
- Audit logs: Indefinite

---

**弘益人間 (Benefit All Humanity)**

*© 2025 WIA - World Certification Industry Association*
*MIT License*

---

## Annex A — Conformance Tier Matrix

WIA conformance for WIA-DIGITAL_EXECUTOR is evaluated across three tiers:

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

- `wia-standards/standards/WIA-DIGITAL_EXECUTOR/api/` — TypeScript SDK skeleton
- `wia-standards/standards/WIA-DIGITAL_EXECUTOR/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/WIA-DIGITAL_EXECUTOR/simulator/` — interactive browser-based simulator for the PHASE protocol

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


## Annex G — Document Index

This PHASE document is one of four PHASE artifacts that together describe this WIA standard. The full set of related artifacts is:

- `spec/PHASE-1-DATA-FORMAT.md` — normative data formats and schema definitions.
- `spec/PHASE-2-API.md` — normative SDK and Client API contract that consumes the PHASE 1 data formats.
- `spec/PHASE-3-PROTOCOL.md` — normative real-time communication protocol bindings (where applicable).
- `spec/PHASE-4-INTEGRATION.md` — normative ecosystem integration patterns (LIMS, paging, federation, reporting).

Readers SHOULD consult all four PHASE documents in sequence when planning a deployment. The OpenAPI document published alongside this standard reflects the §4 endpoints in machine-readable form and is updated synchronously with this PHASE.

This index is informative; the normative content is in the body of each PHASE document.
