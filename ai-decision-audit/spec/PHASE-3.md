# WIA-AI-018 Specification - Phase 3: Intelligence and Analytics

## Overview

Phase 3 adds advanced analytics including bias detection, model drift monitoring, and predictive risk assessment. This phase transforms audit systems from reactive to proactive.

**Status**: ✅ Stable
**Version**: 1.0
**Last Updated**: 2025-01-15
**Prerequisites**: Phase 1 and Phase 2 complete

## Philosophy

弘益人間 (Hongik Ingan) - Benefit All Humanity

Advanced analytics enable early detection of bias, drift, and other issues before they cause harm, protecting individuals and society.

## Core Requirements

### 1. Bias Detection

Implement comprehensive fairness analysis:

```typescript
interface BiasAnalysis {
  bias_score: number;  // 0-100
  disparate_impact_ratio: number;
  statistical_parity_difference: number;
  equal_opportunity_difference: number;
  demographic_parity: Record<string, number>;
  approval_rates_by_group: Record<string, number>;
  sample_size: number;
  confidence_interval: [number, number];
}

class BiasDetector {
  async analyze(
    modelName: string,
    timeRange: DateRange
  ): Promise<BiasAnalysis> {
    const decisions = await this.loadDecisions(modelName, timeRange);
    const grouped = this.groupByDemographic(decisions);

    // Calculate disparate impact (80% rule)
    const approvalRates = this.calculateApprovalRates(grouped);
    const minRate = Math.min(...Object.values(approvalRates));
    const maxRate = Math.max(...Object.values(approvalRates));
    const disparateImpactRatio = minRate / maxRate;

    // Calculate statistical parity
    const statisticalParity = this.calculateStatisticalParity(grouped);

    // Calculate equal opportunity
    const equalOpportunity = this.calculateEqualOpportunity(grouped);

    let biasScore = 0;
    if (disparateImpactRatio < 0.80) biasScore += 40;
    if (statisticalParity > 0.10) biasScore += 30;
    if (equalOpportunity > 0.10) biasScore += 30;

    return {
      bias_score: biasScore,
      disparate_impact_ratio: disparateImpactRatio,
      statistical_parity_difference: statisticalParity,
      equal_opportunity_difference: equalOpportunity,
      demographic_parity: this.calculateDemographicParity(grouped),
      approval_rates_by_group: approvalRates,
      sample_size: decisions.length,
      confidence_interval: this.calculateConfidenceInterval(approvalRates)
    };
  }

  private calculateStatisticalParity(
    groups: Map<string, Decision[]>
  ): number {
    const rates = new Map<string, number>();

    for (const [group, decisions] of groups) {
      const positive = decisions.filter(d => d.output.decision === "APPROVED").length;
      rates.set(group, positive / decisions.length);
    }

    return this.maxPairwiseDifference(rates);
  }

  private calculateEqualOpportunity(
    groups: Map<string, Decision[]>
  ): number {
    const tpr = new Map<string, number>();

    for (const [group, decisions] of groups) {
      const actualPositives = decisions.filter(d => d.ground_truth === true);
      const truePositives = actualPositives.filter(
        d => d.output.decision === "APPROVED"
      );
      tpr.set(group, truePositives.length / actualPositives.length);
    }

    return this.maxPairwiseDifference(tpr);
  }
}
```

### 2. Model Drift Monitoring

Detect performance degradation over time:

```typescript
interface DriftAnalysis {
  feature_drift: {
    kl_divergence: number;
    drifted_features: string[];
  };
  prediction_drift: {
    distribution_shift: number;
    mean_prediction_change: number;
  };
  accuracy_drift: {
    current_accuracy: number;
    baseline_accuracy: number;
    degradation: number;
  };
  recommended_action: "none" | "investigate" | "retrain";
}

class DriftDetector {
  async detectDrift(
    modelId: string,
    comparisonPeriod: "training" | "last_month"
  ): Promise<DriftAnalysis> {
    const baseline = await this.getBaselineMetrics(modelId, comparisonPeriod);
    const current = await this.getCurrentMetrics(modelId, 7);  // Last 7 days

    // Feature distribution drift
    const featureDrift = await this.calculateFeatureDrift(
      baseline.feature_distributions,
      current.feature_distributions
    );

    // Prediction distribution drift
    const predictionDrift = this.calculatePredictionDrift(
      baseline.prediction_distribution,
      current.prediction_distribution
    );

    // Accuracy drift
    const accuracyDegradation = Math.abs(
      baseline.accuracy - current.accuracy
    );

    let action: DriftAnalysis['recommended_action'] = "none";
    if (accuracyDegradation > 0.10 || featureDrift.kl_divergence > 0.5) {
      action = "retrain";
    } else if (accuracyDegradation > 0.05 || featureDrift.kl_divergence > 0.2) {
      action = "investigate";
    }

    return {
      feature_drift: featureDrift,
      prediction_drift: predictionDrift,
      accuracy_drift: {
        current_accuracy: current.accuracy,
        baseline_accuracy: baseline.accuracy,
        degradation: accuracyDegradation
      },
      recommended_action: action
    };
  }

  private async calculateFeatureDrift(
    baseline: Distribution[],
    current: Distribution[]
  ): Promise<{kl_divergence: number; drifted_features: string[]}> {
    const drifted: string[] = [];
    let totalDivergence = 0;

    for (let i = 0; i < baseline.length; i++) {
      const kl = this.klDivergence(baseline[i], current[i]);
      totalDivergence += kl;

      if (kl > 0.3) {  // Threshold for significant drift
        drifted.push(baseline[i].feature_name);
      }
    }

    return {
      kl_divergence: totalDivergence / baseline.length,
      drifted_features: drifted
    };
  }
}
```

### 3. Anomaly Detection

Identify unusual patterns in real-time:

```typescript
interface AnomalyDetection {
  anomalies: Anomaly[];
  anomaly_score: number;
  requires_investigation: boolean;
}

interface Anomaly {
  type: "unusual_confidence" | "slow_processing" | "unusual_input" |
        "suspicious_repetition" | "data_quality";
  severity: "low" | "medium" | "high";
  description: string;
  details?: any;
}

class AnomalyDetector {
  async detectAnomalies(log: DecisionAuditLog): Promise<AnomalyDetection> {
    const anomalies: Anomaly[] = [];

    // Unusual confidence score
    const typicalConfidence = await this.getTypicalConfidence(
      log.context.decision_type
    );
    if (Math.abs(log.output.confidence - typicalConfidence.mean) >
        2 * typicalConfidence.stdDev) {
      anomalies.push({
        type: "unusual_confidence",
        severity: "medium",
        description: `Confidence ${log.output.confidence} is ${Math.abs(log.output.confidence - typicalConfidence.mean).toFixed(2)} standard deviations from mean`
      });
    }

    // Unusually slow processing
    const typicalDuration = await this.getTypicalDuration(log.model.name);
    if (log.processing_duration_ms > typicalDuration.p95) {
      anomalies.push({
        type: "slow_processing",
        severity: "low",
        description: `Processing took ${log.processing_duration_ms}ms (p95: ${typicalDuration.p95}ms)`
      });
    }

    // Input anomalies using isolation forest
    const inputAnomalyScore = await this.scoreInputAnomaly(log.input);
    if (inputAnomalyScore < -0.5) {
      anomalies.push({
        type: "unusual_input",
        severity: "medium",
        description: "Input pattern significantly different from training data",
        details: { anomaly_score: inputAnomalyScore }
      });
    }

    // Suspicious repetition
    const similarRecent = await this.findSimilarRecentDecisions(log);
    if (similarRecent.length > 5) {
      anomalies.push({
        type: "suspicious_repetition",
        severity: "high",
        description: `${similarRecent.length} similar decisions in short timeframe`
      });
    }

    const anomalyScore = this.calculateOverallAnomalyScore(anomalies);

    return {
      anomalies,
      anomaly_score: anomalyScore,
      requires_investigation: anomalyScore > 70 || anomalies.some(a => a.severity === "high")
    };
  }
}
```

### 4. Predictive Analytics

Use historical data to predict future issues:

```typescript
interface PredictiveInsights {
  predicted_compliance_violations: number;
  predicted_bias_issues: Array<{
    model: string;
    probability: number;
    timeframe: string;
  }>;
  predicted_drift: Array<{
    model: string;
    feature: string;
    days_until_drift: number;
  }>;
  recommended_actions: Action[];
}

class PredictiveAnalytics {
  async generateInsights(): Promise<PredictiveInsights> {
    const models = await this.getActiveModels();
    const insights: PredictiveInsights = {
      predicted_compliance_violations: 0,
      predicted_bias_issues: [],
      predicted_drift: [],
      recommended_actions: []
    };

    for (const model of models) {
      // Predict bias issues using trend analysis
      const biasTrajectory = await this.analyzeBiasTrend(model);
      if (biasTrajectory.slope > 0.05) {
        insights.predicted_bias_issues.push({
          model: model.name,
          probability: biasTrajectory.probability,
          timeframe: biasTrajectory.estimated_days + " days"
        });
      }

      // Predict drift using feature distribution monitoring
      const driftPrediction = await this.predictDrift(model);
      for (const feature of driftPrediction.at_risk_features) {
        insights.predicted_drift.push({
          model: model.name,
          feature: feature.name,
          days_until_drift: feature.estimated_days
        });
      }
    }

    // Generate recommended actions
    insights.recommended_actions = this.generateRecommendations(insights);

    return insights;
  }
}
```

### 5. Automated Model Retraining Triggers

Define conditions for automated retraining:

```yaml
retraining_policy:
  triggers:
    - name: accuracy_degradation
      condition: accuracy_drift > 0.10
      action: trigger_retraining
      requires_approval: true

    - name: feature_drift
      condition: kl_divergence > 0.5
      action: alert_data_science
      requires_approval: false

    - name: bias_detected
      condition: disparate_impact_ratio < 0.80
      action: pause_model
      requires_approval: false

    - name: data_volume_threshold
      condition: new_data_points > 100000
      action: trigger_retraining
      requires_approval: true

  approval_workflow:
    - approver: data_science_lead
      timeout_hours: 24
    - approver: compliance_officer
      timeout_hours: 48
```

### 6. API Extensions

New endpoints for Phase 3:

```
POST   /api/v1/analytics/bias/detect        - Run bias analysis
GET    /api/v1/analytics/bias/report        - Get bias report
POST   /api/v1/analytics/drift/detect       - Detect model drift
GET    /api/v1/analytics/drift/report       - Get drift analysis
POST   /api/v1/analytics/anomalies/detect   - Detect anomalies
GET    /api/v1/analytics/predictions        - Get predictive insights
POST   /api/v1/models/retrain/trigger       - Trigger model retraining
```

## Implementation Checklist

- [ ] Implement bias detection engine
- [ ] Add statistical parity metrics
- [ ] Add equal opportunity metrics
- [ ] Implement drift detection
- [ ] Add KL divergence calculation
- [ ] Implement anomaly detection
- [ ] Deploy isolation forest model
- [ ] Implement predictive analytics
- [ ] Configure automated retraining triggers
- [ ] Deploy analytics dashboard
- [ ] Set up scheduled analysis jobs
- [ ] Document analysis methodologies

## Performance Requirements

- **Bias analysis**: Complete within 5 minutes for 100K decisions
- **Drift detection**: Run every 6 hours
- **Anomaly detection**: Real-time (< 100ms)
- **Predictive insights**: Daily generation

## Success Metrics

- **Bias detection accuracy**: % of actual bias issues detected
- **Drift detection lead time**: Days before accuracy drops
- **Anomaly false positive rate**: < 5%
- **Prediction accuracy**: % of predicted issues that materialize

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


## Annex E — Implementation Notes for PHASE-3

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-3.

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
