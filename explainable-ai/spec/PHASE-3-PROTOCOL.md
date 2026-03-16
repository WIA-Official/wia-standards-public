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
