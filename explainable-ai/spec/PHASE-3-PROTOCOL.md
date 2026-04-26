# WIA-AI-009 Phase 3: Explanation Protocol & Trust Metrics

## Overview

Phase 3 establishes protocols for requesting, generating, and validating explanations, along with trust metrics for measuring explanation quality.

**Philosophy**: 弘益人間 - Standardized protocols ensure reliable, trustworthy explanations that serve humanity.

## Explanation Request Protocol

### Request Format

```json
{
  "protocol": "XAI-REQUEST-v1",
  "request_id": "uuid-v4",
  "timestamp": "ISO-8601",
  "model_identifier": {
    "model_id": "loan_classifier_v2.3",
    "version": "2.3.1",
    "deployment": "production"
  },
  "input_data": { /* Instance to explain */ },
  "explanation_config": {
    "methods": ["shap", "lime"],  // Can request multiple
    "granularity": "high | medium | low",
    "num_features": 10,
    "include_counterfactuals": true,
    "target_audience": "technical | business | enduser | regulatory"
  },
  "quality_requirements": {
    "min_fidelity": 0.85,
    "min_stability": 0.80,
    "max_computation_time_ms": 5000
  },
  "output_format": {
    "format": "json | html | pdf",
    "include_visualizations": true,
    "language": "en | ko | es | fr | de"
  }
}
```

### Response Format

```json
{
  "protocol": "XAI-RESPONSE-v1",
  "request_id": "uuid-v4",
  "response_id": "uuid-v4",
  "timestamp": "ISO-8601",
  "status": "success | partial | failed",
  "prediction": {
    "value": 0.73,
    "class": "high_risk",
    "confidence": 0.87
  },
  "explanations": [
    {
      "method": "shap",
      "explanation": { /* SHAP-specific data */ },
      "quality_metrics": {
        "fidelity": 0.94,
        "stability": 0.89,
        "completeness": 0.98
      }
    },
    {
      "method": "lime",
      "explanation": { /* LIME-specific data */ },
      "quality_metrics": {
        "fidelity": 0.91,
        "r_squared": 0.88
      }
    }
  ],
  "metadata": {
    "computation_time_ms": 1250,
    "model_version_used": "2.3.1",
    "explainer_versions": {
      "shap": "0.41.0",
      "lime": "0.2.0.1"
    }
  },
  "warnings": [
    "Computation time exceeded recommendation but within max limit"
  ]
}
```

## Trust Metrics Specification

### Core Trust Metrics

#### 1. Fidelity

Measures how accurately the explanation represents the model's behavior.

```typescript
interface FidelityMetric {
  value: number;  // 0-1, higher is better
  computation_method: "r_squared | correlation | mse";
  test_set_size: number;
  confidence_interval: [number, number];
}

function computeFidelity(
  model: Model,
  explanation: Explanation,
  testSet: Instance[]
): number {
  let totalError = 0;

  for (const instance of testSet) {
    const modelPred = model.predict(instance);
    const explPred = explanation.predictFromExplanation(instance);
    totalError += Math.abs(modelPred - explPred);
  }

  return 1 - (totalError / testSet.length);
}
```

**WIA-AI-009 Threshold**: Fidelity ≥ 0.85 for production use

#### 2. Consistency

Similar inputs should produce similar explanations.

```typescript
interface ConsistencyMetric {
  lipschitz_constant: number;
  semantic_consistency: number;
  neighborhood_radius: number;
}

function computeConsistency(
  explainer: Explainer,
  instance: Instance,
  neighbors: Instance[]
): number {
  const baseExplanation = explainer.explain(instance);
  let consistencySum = 0;

  for (const neighbor of neighbors) {
    const neighborExplanation = explainer.explain(neighbor);
    const inputDistance = computeDistance(instance, neighbor);
    const explanationDistance = computeDistance(
      baseExplanation,
      neighborExplanation
    );

    // Smaller ratio = better consistency
    consistencySum += explanationDistance / (inputDistance + 1e-10);
  }

  return 1 / (1 + consistencySum / neighbors.length);
}
```

**WIA-AI-009 Threshold**: Consistency ≥ 0.75

#### 3. Stability

For stochastic methods, how much do explanations vary across runs?

```typescript
function computeStability(
  explainer: Explainer,
  instance: Instance,
  numRuns: number = 50
): number {
  const explanations = [];

  for (let i = 0; i < numRuns; i++) {
    explanations.push(explainer.explain(instance));
  }

  // Compute coefficient of variation
  const means = computeFeatureMeans(explanations);
  const stdDevs = computeFeatureStdDevs(explanations);

  let cvSum = 0;
  for (const feature in means) {
    const cv = stdDevs[feature] / (Math.abs(means[feature]) + 1e-10);
    cvSum += cv;
  }

  const avgCV = cvSum / Object.keys(means).length;
  return 1 / (1 + avgCV);  // Convert CV to 0-1 score
}
```

**WIA-AI-009 Threshold**: Stability ≥ 0.80

#### 4. Completeness

For additive methods, do attributions sum to the prediction?

```typescript
function computeCompleteness(explanation: SHAPExplanation): number {
  const attributionSum = Object.values(explanation.shap_values)
    .reduce((a, b) => a + b, 0);

  const expected = explanation.prediction - explanation.base_value;
  const error = Math.abs(attributionSum - expected);

  return Math.max(0, 1 - error);
}
```

**WIA-AI-009 Threshold**: Completeness ≥ 0.95 for additive methods

#### 5. Comprehensibility

Proxy metrics for human understandability.

```typescript
interface ComprehensibilityMetric {
  num_features_shown: number;
  avg_feature_name_length: number;
  explanation_complexity_score: number;
  estimated_cognitive_load: "low | medium | high";
}

function estimateComprehensibility(explanation: Explanation): number {
  const numFeatures = explanation.feature_count;
  const avgNameLength = explanation.avg_feature_name_length;

  // Fewer features = more comprehensible
  const featureScore = Math.exp(-numFeatures / 10);

  // Shorter names = more comprehensible
  const nameScore = Math.exp(-avgNameLength / 15);

  return (featureScore + nameScore) / 2;
}
```

### Fairness Metrics

#### Fidelity Parity

```typescript
function computeFidelityParity(
  model: Model,
  explainer: Explainer,
  groupA: Instance[],
  groupB: Instance[]
): number {
  const fidelityA = computeFidelityForGroup(model, explainer, groupA);
  const fidelityB = computeFidelityForGroup(model, explainer, groupB);

  return Math.abs(fidelityA - fidelityB);
}
```

**WIA-AI-009 Threshold**: Parity < 0.05

#### Protected Attribute Weight

```typescript
function computeProtectedAttributeWeight(
  explanation: Explanation,
  protectedAttributes: string[]
): number {
  let totalWeight = 0;
  let protectedWeight = 0;

  for (const [feature, weight] of Object.entries(explanation.feature_weights)) {
    totalWeight += Math.abs(weight);
    if (protectedAttributes.includes(feature)) {
      protectedWeight += Math.abs(weight);
    }
  }

  return protectedWeight / totalWeight;
}
```

**WIA-AI-009 Threshold**: Protected weight < 0.05 (ideally 0)

## Validation Protocol

### Ablation Testing

```typescript
async function runAblationTest(
  model: Model,
  explanation: Explanation,
  instance: Instance
): Promise<AblationResult> {
  const topFeatures = explanation.getTopFeatures(5);
  const baselinePrediction = model.predict(instance);
  const results = [];

  for (const feature of topFeatures) {
    const ablatedInstance = {...instance};
    delete ablatedInstance[feature.name];

    const ablatedPrediction = model.predict(ablatedInstance);
    const predictionChange = Math.abs(baselinePrediction - ablatedPrediction);
    const expectedChange = Math.abs(feature.importance);

    results.push({
      feature: feature.name,
      expected_change: expectedChange,
      actual_change: predictionChange,
      ratio: predictionChange / expectedChange
    });
  }

  const avgRatio = results.reduce((sum, r) => sum + r.ratio, 0) / results.length;

  return {
    passed: avgRatio > 0.70,
    correlation: avgRatio,
    details: results
  };
}
```

### Cross-Method Validation

```typescript
function validateCrossMethod(
  instance: Instance,
  shapExplanation: Explanation,
  limeExplanation: Explanation
): number {
  const shapTop5 = shapExplanation.getTopFeatures(5);
  const limeTop5 = limeExplanation.getTopFeatures(5);

  const shapFeatures = new Set(shapTop5.map(f => f.name));
  const limeFeatures = new Set(limeTop5.map(f => f.name));

  const intersection = new Set(
    [...shapFeatures].filter(f => limeFeatures.has(f))
  );

  return intersection.size / 5;  // Jaccard similarity
}
```

**WIA-AI-009 Threshold**: Agreement ≥ 0.60

## Error Handling Protocol

### Error Codes

```typescript
enum ExplanationErrorCode {
  INSUFFICIENT_SAMPLES = "E001",
  TIMEOUT = "E002",
  MODEL_INCOMPATIBLE = "E003",
  CONVERGENCE_FAILED = "E004",
  MEMORY_EXCEEDED = "E005",
  INVALID_INPUT = "E006",
  QUALITY_THRESHOLD_NOT_MET = "E007"
}

interface ExplanationError {
  code: ExplanationErrorCode;
  message: string;
  details: object;
  fallback_available: boolean;
  retry_recommended: boolean;
}
```

### Fallback Strategy

```json
{
  "status": "partial",
  "primary_method": "shap",
  "primary_result": null,
  "error": {
    "code": "E004",
    "message": "SHAP computation did not converge within time limit"
  },
  "fallback_method": "lime",
  "fallback_result": { /* LIME explanation */ },
  "warning": "Using fallback method due to primary method failure"
}
```

## Quality Assurance Checklist

```typescript
interface QualityAssuranceReport {
  fidelity: { value: number; passed: boolean };
  consistency: { value: number; passed: boolean };
  stability: { value: number; passed: boolean };
  completeness: { value: number; passed: boolean };
  ablation_test: { correlation: number; passed: boolean };
  cross_method_agreement: { value: number; passed: boolean };
  fairness_parity: { value: number; passed: boolean };
  overall_passed: boolean;
  recommendations: string[];
}
```

## Monitoring and Alerting

```typescript
interface ExplanationMonitoring {
  metrics: {
    avg_fidelity_last_24h: number;
    avg_stability_last_24h: number;
    avg_computation_time_ms: number;
    error_rate: number;
    timeout_rate: number;
  };
  alerts: [
    {
      severity: "warning | error | critical";
      message: "Fidelity dropped below 0.85 for age > 65 subgroup";
      timestamp: "ISO-8601";
      recommended_action: "Investigate data distribution changes";
    }
  ];
}
```

---

**弘益人間** - Rigorous protocols and trust metrics ensure AI explanations are reliable and beneficial.

© 2025 SmileStory Inc. / WIA | WIA-AI-009 Phase 3


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

## Annex G — Test Vectors and Conformance Evidence

This annex describes how implementations capture and publish conformance
evidence for PHASE-3-PROTOCOL. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-3-protocol/`. Implementations claiming
  conformance MUST run all vectors in CI and publish the resulting
  pass/fail matrix in their compliance package.
- **Evidence package** — the compliance package is a tarball containing
  the SBOM (CycloneDX 1.5 or SPDX 2.3), the OpenAPI document, the test
  vector matrix, and a signed manifest. Signatures use Sigstore (DSSE
  envelope, Rekor transparency log entry) so that downstream consumers
  can verify provenance without trusting a private CA.
- **Quarterly recheck** — implementations re-publish the evidence package
  every quarter even if no source change occurred, so that consumers can
  detect environmental drift (compiler updates, dependency updates, OS
  updates) without polling vendor changelogs.
- **Cross-vendor crosswalk** — the WIA Standards working group maintains a
  crosswalk that maps each vector to the equivalent assertion in adjacent
  industry programs (where one exists), so an implementer that already
  certifies under one program can show conformance to PHASE-3-PROTOCOL with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-3-PROTOCOL does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-3-PROTOCOL.
It is non-normative; the rules below describe the policy that the WIA
Standards working group commits to when amending this PHASE document.

- **Semantic versioning** — major / minor / patch components follow
  Semantic Versioning 2.0.0 (https://semver.org/spec/v2.0.0.html).
  Major bump indicates a backwards-incompatible change to a normative
  requirement; minor bump indicates new normative requirements that do
  not break existing implementations; patch bump indicates editorial
  changes only (clarifications, typo fixes, formatting).
- **Deprecation window** — when a normative requirement is removed or
  altered in a backwards-incompatible way, the prior major version is
  maintained in parallel for at least 180 days. During the parallel
  window, both major versions are marked Stable in the WIA Standards
  registry and either may be cited as "WIA-conformant".
- **Sunset notification** — deprecated major versions enter a 12-month
  sunset window during which the WIA registry marks the version as
  Deprecated. The deprecation entry includes a migration note pointing
  to the replacement requirement(s) and an explanation of why the
  change was made.
- **Editorial errata** — patch-level errata are issued without a
  deprecation window because they do not change normative behaviour.
  Errata are tracked in a public errata register and each entry is
  signed by the WIA Standards working group chair.
- **Implementation changelog mapping** — implementations SHOULD publish
  a changelog mapping each PHASE version they support to the specific
  build, container digest, or SDK version that satisfies the version.
  This allows downstream auditors to verify version conformance without
  re-running the entire test matrix on every release.

The policy is reviewed at the same cadence as the PHASE document and
any changes to the policy itself are tracked in the version-history
table at the start of the document.

## Annex I — Interoperability Profiles

This annex describes how implementations declare interoperability profiles
for PHASE-3-PROTOCOL. The profile mechanism is non-normative and exists so that
deployments of varying scope (single tenant, regional cluster, federated
network) can advertise the subset of normative requirements they satisfy
without misrepresenting partial conformance as full conformance.

- **Profile manifest** — every implementation publishes a profile manifest
  in JSON. The manifest enumerates the normative requirement IDs from this
  PHASE that are satisfied (`status: "supported"`), partially satisfied
  (`status: "partial"`, with a reason field), or excluded
  (`status: "excluded"`, with a justification). The manifest is signed
  using the same Sigstore key used for the SBOM in Annex G.
- **Federation profile** — federated deployments publish an aggregated
  manifest summarizing the union and intersection of member-implementation
  profiles. The aggregated manifest is consumed by directory services so
  that callers can route a request to the least common denominator profile
  required for an interaction.
- **Backwards-profile compatibility** — when a deployment migrates from one
  profile to a wider profile, the prior profile manifest remains valid and
  signed for the deprecation window defined in Annex H. This preserves
  audit traceability for auditors evaluating long-term interoperability.
- **Profile registry** — the WIA Standards working group maintains a
  public registry of named profiles. Common deployment shapes (e.g.,
  "Edge-only", "Federated-with-replay") are added to the registry by
  consensus. Registry entries are immutable; new shapes are added under
  new names rather than amending existing entries.
- **Profile versioning** — profile names are versioned with the same
  Semantic Versioning rules described in Annex H. A deployment that
  advertises `WIA-P3-PROTOCOL-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.

## Annex J — Reference Implementation Topology

The reference implementation topology described in this annex is
non-normative; it documents the deployment shape that the WIA
Standards working group used to validate the test vectors in Annex G
and is intended as a starting point, not a recommendation against
alternative topologies.

- **Single-tenant edge** — one runtime per organization, no shared
  state. Used for early-pilot deployments where conformance evidence
  is published manually. Sufficient for PHASE-3-PROTOCOL validation when the
  organization signs the manifest itself.
- **Multi-tenant gateway** — one shared runtime serves multiple
  tenants via header-based isolation. Typically backed by a
  rate-limited gateway (Envoy or NGINX) and a shared OAuth 2.1
  identity provider. The manifest is per-tenant; the runtime
  publishes a federation manifest that aggregates tenant manifests.
- **Federated mesh** — multiple runtimes peer to one another and
  publish their manifests to a directory service. Each peer signs
  its own manifest; the directory service signs the aggregated
  index. This is the topology used by cross-organization deployments
  that need to compose conformance.
- **Air-gapped batch** — no network connection between the runtime
  and the directory service. The runtime emits a signed evidence
  package on each batch and the operator transports the package via
  out-of-band channels. This is the topology used by regulators that
  prohibit live connectivity from sensitive environments.

Implementations declare their topology in the manifest (see Annex I).
A topology change MUST be reflected in a new manifest signature; the
prior topology's manifest remains valid for the deprecation window
described in Annex H to preserve audit traceability.
