# WIA Education - Phase 3: Protocol

## Version
- **Version**: 1.0.0
- **Date**: 2025-12-16
- **Status**: Final

## Overview

본 문서는 WIA Education의 통신 프로토콜과 LMS 통합을 정의합니다.

## LTI 1.3 Integration

### Launch Flow
1. LMS → Tool: Launch request
2. Tool: Validate JWT
3. Tool: Process accessibility profile
4. Tool → LMS: Return with accommodations

### Deep Linking
- Content selection with accessibility metadata
- Accessibility requirements propagation
- Custom parameters for accommodations

## xAPI (Experience API)

### Statement Format
```json
{
  "actor": { "mbox": "mailto:student@example.com" },
  "verb": { "id": "http://adlnet.gov/expapi/verbs/completed" },
  "object": {
    "id": "http://example.com/course/123",
    "definition": {
      "name": { "en-US": "Accessible Math Course" },
      "extensions": {
        "http://wiastandards.com/xapi/accessibility": {
          "accommodations": ["extra-time", "screen-reader"],
          "assistive_tech": ["voice-input"],
          "completion_method": "alternative-assessment"
        }
      }
    }
  }
}
```

## Event Synchronization

### Real-time Events
- **CaptionGenerated**: 자막 생성 완료
- **AudioDescriptionAdded**: 오디오 설명 추가
- **AccommodationApplied**: 조정 적용
- **AssessmentStarted**: 평가 시작 (조정 포함)

### WebSocket Protocol
```javascript
// Subscribe to events
ws.send({
  type: "subscribe",
  events: ["CaptionGenerated", "AccommodationApplied"]
});

// Receive events
ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  console.log(data.type, data.payload);
};
```

## LMS Integration

### Canvas
- OAuth 2.0 authentication
- Course accessibility settings
- Assignment accommodations
- Grade sync with accommodation notes

### Moodle
- External tool integration
- Accessibility module
- Custom user fields
- Activity completion tracking

### Blackboard
- REST API integration
- Accessibility features
- Content package import
- Grade center integration

## Security

### Authentication
- OAuth 2.0
- LTI 1.3 JWT validation
- API key management

### Data Privacy
- FERPA compliance
- Student data protection
- Audit logging
- Encryption at rest and in transit

---

**Author**: Yeon Sam-Heum, Ph.D.  
**License**: MIT  
**弘益人間** - Benefit All Humanity

---

## Annex A — Conformance Tier Matrix

WIA conformance for edu is evaluated across three tiers:

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

- `wia-standards/standards/edu/api/` — TypeScript SDK skeleton
- `wia-standards/standards/edu/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/edu/simulator/` — interactive browser-based simulator for the PHASE protocol

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
