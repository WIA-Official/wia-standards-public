# WIA-AI-018 Specification - Phase 2: Compliance and Risk

## Overview

Phase 2 builds on Phase 1 foundation by adding automated compliance checking and risk assessment capabilities. This phase transforms passive logging into active governance.

**Status**: ✅ Stable
**Version**: 1.0
**Last Updated**: 2025-01-15
**Prerequisites**: Phase 1 complete

## Philosophy

弘益人間 (Hongik Ingan) - Benefit All Humanity

Proactive compliance and risk assessment prevent harm before it occurs, embodying our commitment to serving humanity through responsible AI.

## Core Requirements

### 1. Compliance Rule Engine

Implement a rule engine to verify regulatory compliance:

```typescript
interface ComplianceRule {
  id: string;
  name: string;
  framework: "GDPR" | "CCPA" | "EU_AI_Act" | "ECOA" | "HIPAA" | "Custom";
  severity: "critical" | "high" | "medium" | "low";
  check: (log: DecisionAuditLog) => ComplianceResult;
  remediation?: (log: DecisionAuditLog) => Promise<void>;
}

interface ComplianceResult {
  compliant: boolean;
  message: string;
  evidence?: any;
  recommendation?: string;
}

interface ComplianceReport {
  decision_id: string;
  overall_compliant: boolean;
  violations: ComplianceViolation[];
  checked_at: string;
}
```

### 2. Required Compliance Rules

#### 2.1 GDPR Compliance

**GDPR-ART22-EXPLANATION**: Right to Explanation
```typescript
const gdprRightToExplanation: ComplianceRule = {
  id: "gdpr-art22-explanation",
  name: "GDPR Article 22 - Right to Explanation",
  framework: "GDPR",
  severity: "critical",
  check: (log) => {
    if (log.compliance.automated && log.context.business_impact >= "high") {
      if (!log.reasoning || !log.reasoning.explanation ||
          log.reasoning.explanation.length < 100) {
        return {
          compliant: false,
          message: "Missing or insufficient explanation for significant automated decision",
          recommendation: "Add detailed explanation with key factors"
        };
      }
    }
    return { compliant: true, message: "Explanation requirements satisfied" };
  }
};
```

**GDPR-ART5-MINIMIZATION**: Data Minimization
```typescript
const gdprDataMinimization: ComplianceRule = {
  id: "gdpr-art5-minimization",
  name: "GDPR Article 5 - Data Minimization",
  framework: "GDPR",
  severity: "high",
  check: (log) => {
    const allowed = getAllowedFieldsForDecisionType(log.context.decision_type);
    const actual = Object.keys(log.input.raw_data);
    const excess = actual.filter(f => !allowed.includes(f));

    if (excess.length > 0) {
      return {
        compliant: false,
        message: `Excessive data collected: ${excess.join(', ')}`,
        recommendation: "Remove unnecessary fields"
      };
    }
    return { compliant: true, message: "Data minimization satisfied" };
  }
};
```

#### 2.2 CCPA Compliance

**CCPA-DISCLOSURE**: Automated Decision Disclosure
```typescript
const ccpaDisclosure: ComplianceRule = {
  id: "ccpa-1798-185-disclosure",
  name: "CCPA Automated Decision Disclosure",
  framework: "CCPA",
  severity: "high",
  check: (log) => {
    if (log.compliance.automated && !log.compliance.disclosure_provided) {
      return {
        compliant: false,
        message: "Automated decision-making not disclosed to consumer",
        recommendation: "Provide clear disclosure of automated decision-making"
      };
    }
    return { compliant: true, message: "Disclosure requirements met" };
  }
};
```

#### 2.3 EU AI Act Compliance

**EU-AI-ACT-HIGH-RISK**: High-Risk System Requirements
```typescript
const euAIActHighRisk: ComplianceRule = {
  id: "eu-ai-act-high-risk",
  name: "EU AI Act High-Risk System Requirements",
  framework: "EU_AI_Act",
  severity: "critical",
  check: (log) => {
    if (log.context.risk_category === "high") {
      const violations = [];

      if (!log.compliance.human_oversight_available) {
        violations.push("Human oversight mechanism missing");
      }
      if (!log.model.documentation_url) {
        violations.push("Technical documentation not linked");
      }
      if (!log.model.bias_testing_completed) {
        violations.push("Bias testing not completed");
      }

      if (violations.length > 0) {
        return {
          compliant: false,
          message: violations.join('; '),
          recommendation: "Address all high-risk requirements before deployment"
        };
      }
    }
    return { compliant: true, message: "High-risk requirements satisfied" };
  }
};
```

### 3. Risk Assessment Engine

Implement multi-dimensional risk scoring:

```typescript
interface RiskScore {
  overall: number;  // 0-100
  dimensions: {
    confidence_risk: number;
    bias_risk: number;
    data_quality_risk: number;
    drift_risk: number;
    impact_risk: number;
    compliance_risk: number;
    fairness_risk: number;
    explainability_risk: number;
  };
  flags: string[];
  severity: "low" | "medium" | "high" | "critical";
}

class RiskAssessmentEngine {
  async assessRisk(log: DecisionAuditLog): Promise<RiskScore> {
    const dimensions = {
      confidence_risk: this.assessConfidenceRisk(log),
      bias_risk: await this.assessBiasRisk(log),
      data_quality_risk: this.assessDataQualityRisk(log),
      drift_risk: await this.assessDriftRisk(log),
      impact_risk: this.assessImpactRisk(log),
      compliance_risk: await this.assessComplianceRisk(log),
      fairness_risk: await this.assessFairnessRisk(log),
      explainability_risk: this.assessExplainabilityRisk(log)
    };

    const overall = this.calculateOverallRisk(dimensions);
    const flags = this.identifyRiskFlags(log, dimensions);
    const severity = this.determineSeverity(overall, flags);

    return { overall, dimensions, flags, severity };
  }
}
```

### 4. Real-Time Enforcement

Implement real-time compliance enforcement:

```typescript
class RealTimeEnforcer {
  async enforceCompliance(log: DecisionAuditLog): Promise<EnforcementResult> {
    const report = await this.ruleEngine.checkCompliance(log);

    // Block critical violations
    const criticalViolations = report.violations.filter(
      v => v.severity === "critical"
    );

    if (criticalViolations.length > 0) {
      await this.blockDecision(log.decision_id, criticalViolations);
      await this.alertComplianceTeam(log.decision_id, criticalViolations);

      return {
        allowed: false,
        reason: "Critical compliance violations detected",
        violations: criticalViolations
      };
    }

    return { allowed: true };
  }
}
```

### 5. Alerting System

Configure multi-channel alerting:

```yaml
alerting:
  channels:
    - type: pagerduty
      conditions:
        - severity: critical
        - risk_score: ">= 90"
      escalation_policy: ai-compliance-escalation

    - type: slack
      conditions:
        - severity: [critical, high]
      channels:
        critical: "#ai-critical-alerts"
        high: "#ai-high-priority"

    - type: email
      conditions:
        - severity: [critical, high, medium]
      recipients:
        critical: ["cto@company.com", "compliance@company.com"]
        high: ["ai-team@company.com"]
        medium: ["ai-monitoring@company.com"]

  aggregation:
    window_minutes: 5
    max_alerts_per_window: 10
```

### 6. Compliance Dashboard

Deploy compliance monitoring dashboard:

- Overall compliance rate
- Violations by severity
- Violations by framework
- Trend analysis
- Top violations
- Remediation status

### 7. API Extensions

New endpoints for Phase 2:

```
POST   /api/v1/compliance/check        - Check decision compliance
GET    /api/v1/compliance/rules        - List compliance rules
POST   /api/v1/compliance/rules        - Add custom rule
GET    /api/v1/compliance/report       - Generate compliance report
POST   /api/v1/risk/assess             - Assess decision risk
GET    /api/v1/risk/dashboard          - Risk monitoring dashboard
POST   /api/v1/enforce                 - Enforce compliance (blocking)
```

## Implementation Checklist

- [ ] Implement compliance rule engine
- [ ] Add GDPR compliance rules
- [ ] Add CCPA compliance rules
- [ ] Add EU AI Act compliance rules
- [ ] Add sector-specific rules (ECOA, HIPAA, etc.)
- [ ] Implement risk assessment engine
- [ ] Deploy real-time enforcement
- [ ] Configure alerting channels
- [ ] Deploy compliance dashboard
- [ ] Implement automated remediation
- [ ] Document compliance mapping
- [ ] Train compliance team

## Performance Requirements

- **Compliance check latency**: < 50ms
- **Risk assessment latency**: < 100ms
- **Alert delivery**: < 30 seconds
- **Dashboard refresh**: < 5 seconds

## Testing Requirements

1. **Compliance rule tests**: Verify each rule correctly identifies violations
2. **Risk assessment tests**: Validate risk scoring accuracy
3. **Enforcement tests**: Ensure critical violations are blocked
4. **Alerting tests**: Verify all channels deliver alerts
5. **Dashboard tests**: Confirm accuracy of displayed metrics

## Success Metrics

- **Compliance rate**: % of decisions compliant with all applicable rules
- **Violation detection rate**: % of violations caught
- **False positive rate**: % of false compliance violations
- **Mean time to detection**: Average time to detect violations
- **Mean time to remediation**: Average time to fix violations

## Next Steps

- **Phase 3**: Implement bias detection and model drift monitoring
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
