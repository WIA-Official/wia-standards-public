# WIA-AI-009 Phase 2: XAI Algorithms & Methods

## Overview

Phase 2 implements core explainability algorithms including SHAP, LIME, attention mechanisms, integrated gradients, and decision tree extraction.

**Philosophy**: 弘益人間 - Diverse methods ensure explanations are accessible to all model types and use cases.

## Supported XAI Methods

### 1. SHAP (Shapley Additive exPlanations)

#### TreeSHAP
Fast, exact SHAP for tree-based models (XGBoost, Random Forest, LightGBM).

```typescript
import { TreeSHAPExplainer } from 'wia-ai-009';

const explainer = new TreeSHAPExplainer(xgboostModel, {
  backgroundData: trainingData.sample(1000),
  checkAdditivity: true
});

const explanation = explainer.explain(instance);
// Returns: { base_value, shap_values, feature_values }
```

**Complexity**: O(TLD²) where T=trees, L=leaves, D=depth

#### KernelSHAP
Model-agnostic SHAP using sampling and weighted regression.

```typescript
const explainer = new KernelSHAPExplainer(model, {
  backgroundData: trainingData.sample(500),
  nSamples: 5000,
  l1Regularization: 0.01
});
```

**Complexity**: O(2^M × N) approximated to O(N × M²)

#### LinearSHAP
Closed-form SHAP for linear models.

```typescript
const explainer = new LinearSHAPExplainer(linearModel);
// Instant computation: φᵢ = βᵢ × (xᵢ - E[Xᵢ])
```

### 2. LIME (Local Interpretable Model-agnostic Explanations)

#### Tabular LIME

```typescript
import { TabularLIMEExplainer } from 'wia-ai-009';

const explainer = new TabularLIMEExplainer({
  featureNames: ['age', 'income', 'credit_score'],
  categoricalFeatures: ['employment_status'],
  kernelWidth: Math.sqrt(numFeatures) * 0.75,
  numSamples: 5000,
  numFeatures: 10
});

const explanation = explainer.explain(instance, predictFn);
```

#### Image LIME

```typescript
const explainer = new ImageLIMEExplainer({
  segmentationAlgorithm: 'quickshift',
  numSamples: 1000,
  numFeatures: 5,
  hideColor: 'gray'
});

const explanation = explainer.explainImage(imageArray, classifierFn);
// Returns superpixel weights
```

#### Text LIME

```typescript
const explainer = new TextLIMEExplainer({
  bowModel: true,
  numSamples: 5000,
  numFeatures: 10
});

const explanation = explainer.explainText(text, classifierFn);
// Returns word importance scores
```

### 3. Attention Mechanisms

#### Self-Attention Extraction

```typescript
import { AttentionExplainer } from 'wia-ai-009';

const explainer = new AttentionExplainer(transformerModel, {
  layer: 8,  // Which layer to extract from
  head: 3,   // Which attention head (or 'all')
  aggregation: 'mean'  // How to combine heads
});

const attention = explainer.getAttentionWeights(tokens);
```

#### Grad-CAM (Gradient-weighted Class Activation Mapping)

```typescript
const explainer = new GradCAMExplainer(cnnModel, {
  targetLayer: 'conv5_3',
  targetClass: 'predicted_class'
});

const heatmap = explainer.generateHeatmap(image);
```

### 4. Integrated Gradients

```typescript
import { IntegratedGradientsExplainer } from 'wia-ai-009';

const explainer = new IntegratedGradientsExplainer(model, {
  baseline: 'zeros',  // or 'mean', 'black', custom array
  numSteps: 50,
  method: 'riemann_trapezoidal'
});

const attributions = explainer.explain(input, targetClass);
```

### 5. Counterfactual Explanations

```typescript
import { CounterfactualExplainer } from 'wia-ai-009';

const explainer = new CounterfactualExplainer(model, {
  targetOutcome: 'approved',
  maxChanges: 3,
  featureRanges: featureConstraints,
  preferredFeatures: ['debt_ratio', 'credit_score']
});

const counterfactual = explainer.generateCounterfactual(instance);
// Returns: { changes, new_prediction, distance, actionability }
```

## Method Selection Guidelines

| Use Case | Recommended Method | Rationale |
|----------|-------------------|-----------|
| Tree models (XGBoost, RF) | TreeSHAP | Exact, fast, theoretically sound |
| Deep neural networks | Integrated Gradients, Attention | Leverages gradients, built-in attention |
| Black-box models | KernelSHAP, LIME | Model-agnostic |
| Image classification | Grad-CAM, Image LIME | Spatial visualization |
| Text classification | Text LIME, Attention | Token-level explanations |
| Linear models | LinearSHAP, coefficients | Closed-form, instant |
| Actionable insights | Counterfactuals | Shows how to change outcome |

## Implementation Details

### SHAP Implementation

```typescript
class TreeSHAPExplainer {
  constructor(model, config) {
    this.model = model;
    this.backgroundData = config.backgroundData;
    this.checkAdditivity = config.checkAdditivity || false;
  }

  explain(instance) {
    const trees = this.model.getTrees();
    const shapValues = {};

    // For each feature
    for (const feature of this.model.features) {
      shapValues[feature] = this.computeShapValue(
        instance,
        feature,
        trees
      );
    }

    const baseValue = this.computeBaseValue();

    if (this.checkAdditivity) {
      this.verifyAdditivity(instance, shapValues, baseValue);
    }

    return {
      base_value: baseValue,
      shap_values: shapValues,
      feature_values: instance,
      prediction: this.model.predict(instance)
    };
  }

  computeShapValue(instance, feature, trees) {
    // TreeSHAP polynomial-time algorithm
    // Implementation details omitted for brevity
    return computeTreeShapRecursive(trees, feature, instance);
  }

  verifyAdditivity(instance, shapValues, baseValue) {
    const sum = Object.values(shapValues).reduce((a, b) => a + b, 0);
    const prediction = this.model.predict(instance);
    const expected = prediction - baseValue;

    if (Math.abs(sum - expected) > 1e-5) {
      console.warn('Additivity property violated', { sum, expected });
    }
  }
}
```

### LIME Implementation

```typescript
class TabularLIMEExplainer {
  explain(instance, predictFn) {
    // 1. Generate perturbations
    const perturbations = this.generatePerturbations(instance);

    // 2. Get predictions for perturbations
    const predictions = perturbations.map(p => predictFn(p));

    // 3. Compute weights (kernel function)
    const weights = this.computeWeights(instance, perturbations);

    // 4. Fit weighted linear model
    const linearModel = this.fitWeightedLinear(
      perturbations,
      predictions,
      weights
    );

    // 5. Extract coefficients as explanation
    return {
      intercept: linearModel.intercept,
      feature_weights: linearModel.coefficients,
      local_prediction: linearModel.predict(instance),
      actual_prediction: predictFn(instance),
      r_squared: linearModel.r2
    };
  }

  generatePerturbations(instance) {
    const perturbations = [];

    for (let i = 0; i < this.nSamples; i++) {
      const perturbed = {};

      for (const [feature, value] of Object.entries(instance)) {
        if (this.categoricalFeatures.includes(feature)) {
          perturbed[feature] = this.sampleCategorical(feature);
        } else {
          perturbed[feature] = this.perturbNumerical(value);
        }
      }

      perturbations.push(perturbed);
    }

    return perturbations;
  }

  computeWeights(instance, perturbations) {
    return perturbations.map(p => {
      const distance = this.distance(instance, p);
      return this.kernelFn(distance);
    });
  }

  kernelFn(distance) {
    return Math.exp(-(distance ** 2) / (this.kernelWidth ** 2));
  }
}
```

## Performance Benchmarks

| Method | Model Type | Avg Time (ms) | Memory (MB) | Accuracy |
|--------|-----------|---------------|-------------|----------|
| TreeSHAP | XGBoost (100 trees) | 45 | 120 | Exact |
| KernelSHAP | Any | 2500 | 80 | ~95% |
| LIME | Any | 1800 | 60 | ~90% |
| LinearSHAP | Linear | <1 | 10 | Exact |
| Integrated Gradients | Neural Net | 1200 | 200 | ~92% |
| Attention | Transformer | 15 | 150 | Inherent |

## API Endpoints

### Generate Explanation

```http
POST /api/v1/explain
Content-Type: application/json

{
  "model_id": "loan_classifier_v2",
  "input": {...},
  "method": "shap",
  "config": {
    "variant": "TreeSHAP",
    "num_features": 10
  }
}
```

### Batch Explanation

```http
POST /api/v1/explain/batch
{
  "model_id": "loan_classifier_v2",
  "instances": [{...}, {...}, ...],
  "method": "shap",
  "async": true
}

Response:
{
  "batch_id": "uuid",
  "status": "processing",
  "estimated_completion": "2025-01-20T15:30:00Z"
}
```

### Get Explanation Status

```http
GET /api/v1/explain/batch/{batch_id}

Response:
{
  "batch_id": "uuid",
  "status": "completed",
  "progress": 1000 / 1000,
  "results_url": "/api/v1/explain/batch/uuid/results"
}
```

## Testing and Validation

```typescript
describe('SHAP Explainer', () => {
  test('additivity property', () => {
    const explanation = explainer.explain(instance);
    const sum = sumValues(explanation.shap_values);
    const expected = explanation.prediction - explanation.base_value;
    expect(sum).toBeCloseTo(expected, 5);
  });

  test('missingness property', () => {
    const instanceWithMissing = {...instance, unused_feature: null};
    const explanation = explainer.explain(instanceWithMissing);
    expect(explanation.shap_values.unused_feature).toBeCloseTo(0, 5);
  });

  test('symmetry property', () => {
    // Features with equal impact should have equal SHAP values
    const symmetricInstance = createSymmetricInstance();
    const explanation = explainer.explain(symmetricInstance);
    expect(explanation.shap_values.feature_a)
      .toBeCloseTo(explanation.shap_values.feature_b, 2);
  });
});
```

---

**弘益人間** - Multiple explanation methods ensure accessibility for all model types and user needs.

© 2025 SmileStory Inc. / WIA | WIA-AI-009 Phase 2
