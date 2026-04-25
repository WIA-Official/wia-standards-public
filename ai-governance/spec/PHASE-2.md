# WIA-AI-020: AI Governance Standard
## PHASE-2: Advanced Governance & Operational Controls

**Version:** 1.0
**Status:** Active
**Last Updated:** 2025-12-25
**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

Phase 2 extends the foundational governance established in Phase 1 with advanced operational controls, sophisticated risk management, and enhanced stakeholder engagement mechanisms.

## 2. Advanced Risk Management

### 2.1 Continuous Monitoring Systems
**Requirements:**
- AI-025: Automated monitoring SHALL be implemented for all production AI systems
- AI-026: Performance metrics MUST be tracked in real-time
- AI-027: Drift detection MUST be automated with alerting thresholds
- AI-028: Anomaly detection SHALL identify unusual patterns or behaviors

### 2.2 Bias Testing Requirements
**Requirements:**
- AI-029: Fairness metrics MUST include demographic parity, equal opportunity, and predictive parity
- AI-030: Bias testing SHALL be conducted across all protected characteristics
- AI-031: Intersectional bias MUST be evaluated
- AI-032: Disaggregated performance metrics SHALL be maintained

### 2.3 Model Validation
**Requirements:**
- AI-033: Independent validation MUST be performed for high-risk models
- AI-034: Validation SHALL include performance, bias, robustness, and security testing
- AI-035: Validation documentation MUST be maintained
- AI-036: Revalidation SHALL occur after material changes

## 3. Ethics Integration

### 3.1 Algorithmic Impact Assessments
**Requirements:**
- AI-037: Impact assessments MUST be conducted for high-risk AI systems
- AI-038: Assessments SHALL include stakeholder analysis, benefits, harms, and mitigation
- AI-039: Affected communities MUST be consulted
- AI-040: Assessment summaries SHALL be published

### 3.2 Ethics Review Process
**Requirements:**
- AI-041: Ethics Board review REQUIRED for high-risk systems before deployment
- AI-042: Review SHALL evaluate alignment with ethical principles
- AI-043: Board MAY approve, conditionally approve, require modifications, or reject systems
- AI-044: Appeals process SHALL be available

## 4. Regulatory Compliance

### 4.1 GDPR Compliance (where applicable)
**Requirements:**
- AI-045: Lawful basis for data processing MUST be documented
- AI-046: Data minimization SHALL be practiced
- AI-047: Individual rights (access, rectification, erasure, objection) MUST be implementable
- AI-048: Data Protection Impact Assessments REQUIRED for high-risk processing

### 4.2 EU AI Act Compliance (where applicable)
**Requirements:**
- AI-049: AI systems MUST be classified by risk level
- AI-050: High-risk systems SHALL meet all eight requirements (risk management, data governance, documentation, record-keeping, transparency, human oversight, accuracy/robustness, cybersecurity)
- AI-051: Conformity assessment MUST be completed before deployment
- AI-052: Systems SHALL be registered in EU database

### 4.3 Sector-Specific Compliance
**Requirements:**
- AI-053: Healthcare AI SHALL comply with FDA/HIPAA requirements
- AI-054: Financial services AI SHALL comply with model risk management (SR 11-7)
- AI-055: Employment AI SHALL comply with anti-discrimination laws
- AI-056: Industry-specific requirements MUST be identified and met

## 5. Advanced Documentation

### 5.1 Model Cards
**Requirements:**
- AI-057: Model cards SHALL be created for all AI models
- AI-058: Cards MUST include intended use, training data, performance metrics, limitations, fairness metrics, and ethical considerations
- AI-059: Model cards SHALL be updated with material changes
- AI-060: Cards SHOULD be publicly available where appropriate

### 5.2 Data Sheets
**Requirements:**
- AI-061: Data sheets SHALL document all training datasets
- AI-062: Sheets MUST include composition, collection process, preprocessing, uses, distribution, and maintenance
- AI-063: Known biases and limitations SHALL be documented
- AI-064: Data lineage MUST be traceable

## 6. Stakeholder Engagement

### 6.1 Transparency Requirements
**Requirements:**
- AI-065: AI use SHALL be disclosed to affected individuals
- AI-066: Clear information about capabilities and limitations MUST be provided
- AI-067: Decision explanations SHALL be available for consequential decisions
- AI-068: Stakeholder feedback mechanisms MUST be established

### 6.2 Human Oversight
**Requirements:**
- AI-069: Human-in-the-loop REQUIRED for critical decisions
- AI-070: Human-on-the-loop REQUIRED for high-risk autonomous systems
- AI-071: Override mechanisms MUST be available
- AI-072: Human operators SHALL be trained on AI limitations

## 7. Security Controls

### 7.1 Adversarial Robustness
**Requirements:**
- AI-073: Systems SHALL be tested for adversarial vulnerabilities
- AI-074: Input validation MUST prevent malicious inputs
- AI-075: Model hardening techniques SHOULD be applied
- AI-076: Security monitoring SHALL detect potential attacks

### 7.2 Data Security
**Requirements:**
- AI-077: Training data SHALL be protected with encryption and access controls
- AI-078: Model parameters MUST be secured against theft
- AI-079: Inference data SHALL be protected
- AI-080: Security incident response procedures MUST be defined

## 8. Incident Management

### 8.1 Incident Response
**Requirements:**
- AI-081: Incident response procedures SHALL be documented
- AI-082: Incident classification system MUST be established
- AI-083: Response SHALL include detection, containment, investigation, remediation, and prevention
- AI-084: Stakeholders MUST be notified as appropriate

### 8.2 Post-Incident Review
**Requirements:**
- AI-085: Root cause analysis SHALL be conducted
- AI-086: Lessons learned MUST be documented
- AI-087: Systems and processes SHALL be updated to prevent recurrence
- AI-088: Incident database MUST be maintained

## 9. Performance Metrics

### 9.1 Governance Effectiveness Metrics
- Incident rate and severity
- Mean time to detect/respond/remediate
- Compliance audit findings
- Stakeholder satisfaction scores
- Ethics Board review completion rate

### 9.2 AI System Metrics
- Model performance (accuracy, precision, recall)
- Fairness metrics across demographics
- Drift detection frequency
- Availability and reliability
- Explainability scores

## 10. Continuous Improvement

### 10.1 Review Cycles
- Quarterly risk reassessments
- Annual comprehensive audits
- Post-incident reviews
- Regulatory landscape monitoring
- Best practice benchmarking

### 10.2 Optimization
- Process efficiency improvements
- Automation opportunities
- Tool and platform enhancements
- Training program updates
- Culture building initiatives

---

**Document Control:**
- Approved by: WIA Standards Committee
- Next Review Date: 2026-12-25

**© 2025 SmileStory Inc. / WIA**
**弘益人間 (Benefit All Humanity)**

---

## Annex A — Conformance Tier Matrix

WIA conformance for ai-governance is evaluated across three tiers:

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

- `wia-standards/standards/ai-governance/api/` — TypeScript SDK skeleton
- `wia-standards/standards/ai-governance/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/ai-governance/simulator/` — interactive browser-based simulator for the PHASE protocol

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


## Annex E — Implementation Notes for PHASE-2

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-2.

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
