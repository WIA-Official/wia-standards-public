# WIA-AI-018 Specification - Phase 1: Foundation

## Overview

Phase 1 establishes the foundational infrastructure for AI decision audit systems. This phase focuses on core logging capabilities, basic data schemas, and storage infrastructure.

**Status**: ✅ Stable
**Version**: 1.0
**Last Updated**: 2025-01-15

## Philosophy

弘益人間 (Hongik Ingan) - Benefit All Humanity

AI decision audit systems must serve all stakeholders: regulators, affected individuals, data scientists, and society at large. Phase 1 creates the foundation for transparent, accountable AI.

## Core Requirements

### 1. Decision Logging Schema

Every AI decision must be logged with the following required fields:

```typescript
interface DecisionAuditLog {
  // Unique identification
  decision_id: string;           // UUID or similar
  correlation_id?: string;       // Links related decisions

  // Temporal information
  timestamp: string;             // ISO 8601 format
  timezone: string;              // IANA timezone
  processing_duration_ms: number;

  // System information
  system: {
    name: string;
    version: string;
    environment: "production" | "staging" | "development";
    deployment_id: string;
  };

  // Model information
  model: {
    name: string;
    version: string;
    type: "neural_network" | "decision_tree" | "ensemble" | "rule_based" | "other";
    training_date: string;
    training_dataset_id?: string;
  };

  // Input data
  input: {
    raw_data: any;
    preprocessed_data?: any;
    feature_vector?: number[];
    data_sources: string[];
  };

  // Decision output
  output: {
    decision: any;
    confidence: number;           // 0.0 to 1.0
    alternatives?: Array<{
      decision: any;
      confidence: number;
    }>;
    flags: string[];              // "high_risk", "low_confidence", etc.
  };

  // Context
  context: {
    user_id?: string;             // Pseudonymized
    session_id?: string;
    request_id?: string;
    geographic_location?: string;
    decision_type: string;
    business_impact: "low" | "medium" | "high" | "critical";
  };

  // Audit metadata
  audit: {
    log_version: string;
    integrity_hash: string;
    previous_hash?: string;       // For hash chaining
    signature?: string;           // Cryptographic signature
  };
}
```

### 2. Storage Requirements

#### 2.1 Database Schema

Minimum database requirements:

```sql
CREATE TABLE audit_logs (
  decision_id VARCHAR(64) PRIMARY KEY,
  created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
  model_name VARCHAR(255) NOT NULL,
  model_version VARCHAR(32) NOT NULL,
  decision_type VARCHAR(64) NOT NULL,
  business_impact VARCHAR(16) NOT NULL,
  confidence DECIMAL(5,4),
  input_data JSONB NOT NULL,
  output_data JSONB NOT NULL,
  context JSONB,
  audit_metadata JSONB NOT NULL,
  hash VARCHAR(64) NOT NULL,
  previous_hash VARCHAR(64)
);

-- Required indexes
CREATE INDEX idx_audit_logs_created_at ON audit_logs(created_at);
CREATE INDEX idx_audit_logs_model ON audit_logs(model_name, model_version);
CREATE INDEX idx_audit_logs_decision_type ON audit_logs(decision_type);
CREATE INDEX idx_audit_logs_impact ON audit_logs(business_impact);

-- Prevent updates and deletes (append-only)
CREATE OR REPLACE RULE audit_logs_no_update AS
  ON UPDATE TO audit_logs
  DO INSTEAD NOTHING;

CREATE OR REPLACE RULE audit_logs_no_delete AS
  ON DELETE TO audit_logs
  DO INSTEAD NOTHING;
```

#### 2.2 Storage Tiers

- **Hot storage**: Last 90 days - SSD/fast access
- **Warm storage**: 90 days to 2 years - Standard storage
- **Cold storage**: 2-7 years - Archive storage (S3 Glacier, etc.)

### 3. Hash Chaining for Integrity

Implement cryptographic hash chaining to ensure tamper evidence:

```typescript
function calculateHash(log: DecisionAuditLog): string {
  const content = JSON.stringify({
    decision_id: log.decision_id,
    timestamp: log.timestamp,
    model: log.model,
    input: log.input,
    output: log.output,
    context: log.context,
    previous_hash: log.audit.previous_hash || '0'
  });

  return crypto
    .createHash('sha256')
    .update(content)
    .digest('hex');
}
```

### 4. Logging API Endpoints

Minimum required API endpoints:

```
POST   /api/v1/audit/log           - Log a decision
GET    /api/v1/audit/log/{id}      - Retrieve specific log
GET    /api/v1/audit/logs          - Query logs (with filters)
GET    /api/v1/audit/verify        - Verify hash chain integrity
GET    /api/v1/audit/health        - System health check
```

### 5. Performance Requirements

- **Asynchronous logging**: Must not block decision delivery
- **Latency**: < 100ms for async log submission
- **Throughput**: Support 1000+ decisions/second
- **Availability**: 99.9% uptime

### 6. Privacy and Security

#### 6.1 Data Protection

- Encrypt sensitive fields at rest
- Pseudonymize personal identifiers
- Implement role-based access control
- Audit all access to audit logs

#### 6.2 Retention Policy

- Minimum retention: As required by applicable regulations
- Default retention: 7 years
- Legal hold support: Indefinite retention when required

## Implementation Checklist

- [ ] Deploy logging service infrastructure
- [ ] Implement decision schema
- [ ] Set up database with required tables and indexes
- [ ] Implement hash chaining
- [ ] Configure storage tiers
- [ ] Deploy API endpoints
- [ ] Implement access controls
- [ ] Set up encryption at rest
- [ ] Configure backup and disaster recovery
- [ ] Document system architecture
- [ ] Establish monitoring and alerting
- [ ] Complete load testing

## Compliance Mapping

### GDPR
- Schema supports right to explanation (Article 22)
- Pseudonymization protects personal data (Article 25)
- Retention policy supports storage limitation (Article 5)

### CCPA
- Schema supports disclosure requirements
- Access controls enable consumer data requests

### EU AI Act
- Logging supports high-risk system requirements
- Hash chaining provides integrity verification

## Testing Requirements

1. **Unit tests**: All logging functions
2. **Integration tests**: End-to-end logging flow
3. **Performance tests**: 1000+ decisions/second
4. **Security tests**: Encryption, access control
5. **Integrity tests**: Hash chain verification

## Migration Path

For existing systems:

1. Deploy audit infrastructure in parallel
2. Add logging to new decisions first
3. Backfill historical data if required
4. Gradually expand coverage
5. Enforce 100% coverage for high-risk decisions

## Success Metrics

- **Coverage**: % of decisions logged
- **Latency**: Average time to persist log
- **Completeness**: % of logs with all required fields
- **Integrity**: Hash chain verification success rate
- **Availability**: System uptime %

## Next Steps

After completing Phase 1:

- **Phase 2**: Add compliance checking and risk assessment
- **Phase 3**: Implement bias detection and drift monitoring
- **Phase 4**: Deploy federation and ecosystem integration

---

© 2025 WIA (World Certification Industry Association)
弘益人間 · Benefit All Humanity

---

## Annex A — Conformance Tier Matrix

WIA conformance for ai-decision-audit is evaluated across three tiers:

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

- `wia-standards/standards/ai-decision-audit/api/` — TypeScript SDK skeleton
- `wia-standards/standards/ai-decision-audit/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/ai-decision-audit/simulator/` — interactive browser-based simulator for the PHASE protocol

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


## Annex E — Implementation Notes for PHASE-1

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-1.

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
