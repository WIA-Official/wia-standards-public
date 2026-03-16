# WIA-AI-009 Phase 4: Integration & Visualization

## Overview

Phase 4 integrates XAI capabilities into existing ML pipelines and provides visualization tools for different audiences.

**Philosophy**: 弘益人間 - Practical integration ensures XAI benefits reach all stakeholders.

## ML Pipeline Integration

### Training Phase

```python
# WIA-AI-009 Training Integration
from wia_ai_009 import ExplanationValidator

def train_with_explainability(model, X_train, y_train):
    # Standard training
    model.fit(X_train, y_train)

    # Validate explainability
    validator = ExplanationValidator(model)
    validation_report = validator.validate_on_test_set(
        X_train.sample(1000),
        methods=['shap', 'lime']
    )

    if validation_report.average_fidelity < 0.85:
        raise ValueError("Model fails explainability requirements")

    return model, validation_report
```

### Inference Phase

```typescript
// Real-time explanation generation
class ExplainableInferenceService {
  async predictWithExplanation(input: any) {
    const [prediction, explanation] = await Promise.all([
      this.model.predict(input),
      this.explainer.explain(input)
    ]);

    return {
      prediction,
      explanation,
      timestamp: new Date().toISOString()
    };
  }
}
```

### Deployment Architecture

```
┌─────────────────────────────────────────────────┐
│             Load Balancer                       │
└─────────────────┬───────────────────────────────┘
                  │
        ┌─────────┴─────────┐
        │                   │
┌───────▼────────┐  ┌───────▼────────┐
│  Prediction    │  │  Explanation   │
│  Service       │  │  Service       │
│  (Fast)        │  │  (Scalable)    │
└───────┬────────┘  └───────┬────────┘
        │                   │
        └─────────┬─────────┘
                  │
        ┌─────────▼─────────┐
        │  Cache Service    │
        │  (Redis)          │
        └───────────────────┘
```

## Visualization Components

### Force Plot

Shows how features push prediction from baseline.

```typescript
import { ForcePlot } from 'wia-ai-009/viz';

const plot = new ForcePlot({
  baseValue: 0.15,
  prediction: 0.73,
  features: shapValues,
  colorScheme: 'redblue'
});

plot.render('#force-plot-container');
```

### Summary Plot

Global feature importance with distribution.

```typescript
import { SummaryPlot } from 'wia-ai-009/viz';

const plot = new SummaryPlot({
  shapValues: allExplanations,
  featureNames: features,
  maxDisplay: 10
});
```

### Dependence Plot

Shows feature value vs. SHAP value relationship.

```typescript
const plot = new DependencePlot({
  feature: 'credit_score',
  shapValues: explanations,
  featureValues: instances,
  interactionFeature: 'debt_ratio'  // Optional coloring
});
```

### Attention Heatmap

Visualize attention weights for text/images.

```typescript
const heatmap = new AttentionHeatmap({
  tokens: ['The', 'quick', 'brown', 'fox'],
  attentionWeights: weights,
  layer: 8,
  head: 3
});
```

## Dashboard Templates

### Executive Dashboard

```html
<div class="wia-dashboard" data-audience="executive">
  <div class="metric-card">
    <h3>Model Trustworthiness</h3>
    <div class="score">94%</div>
    <div class="trend">+3% from last month</div>
  </div>

  <div class="top-features">
    <h3>Key Decision Factors</h3>
    <ul>
      <li>Credit Score (35%)</li>
      <li>Debt Ratio (28%)</li>
      <li>Employment (18%)</li>
    </ul>
  </div>

  <div class="fairness-summary">
    <h3>Fairness Assessment</h3>
    <p>No significant disparities detected across demographic groups.</p>
  </div>
</div>
```

### Technical Dashboard

```html
<div class="wia-dashboard" data-audience="technical">
  <div class="metrics-grid">
    <div>Fidelity: 0.94</div>
    <div>Stability: 0.89</div>
    <div>Consistency: 0.87</div>
    <div>Completeness: 0.98</div>
  </div>

  <div class="shap-summary-plot" id="shap-plot"></div>

  <div class="feature-interactions">
    <h3>Top Interactions</h3>
    <ul>
      <li>credit_score ↔ debt_ratio: 0.42</li>
      <li>income ↔ loan_amount: 0.38</li>
    </ul>
  </div>
</div>
```

### End-User Interface

```html
<div class="wia-explanation" data-audience="enduser">
  <div class="decision">
    <h2>Your Application Status: Denied</h2>
  </div>

  <div class="reasons">
    <h3>Main Factors:</h3>
    <div class="factor negative">
      <span class="icon">⚠️</span>
      <span>High debt-to-income ratio (52%)</span>
    </div>
    <div class="factor positive">
      <span class="icon">✓</span>
      <span>Good credit score (720)</span>
    </div>
  </div>

  <div class="counterfactual">
    <h3>How to Improve:</h3>
    <p>Reducing your debt-to-income ratio to below 35% would likely result in approval.</p>
  </div>
</div>
```

## API Integration Examples

### REST API

```javascript
// Node.js/Express integration
const wia = require('wia-ai-009');

app.post('/api/predict', async (req, res) => {
  const { input, includeExplanation } = req.body;

  const prediction = await model.predict(input);

  if (includeExplanation) {
    const explanation = await explainer.explain(input);

    res.json({
      prediction,
      explanation,
      standard: 'WIA-AI-009'
    });
  } else {
    res.json({ prediction });
  }
});
```

### GraphQL

```graphql
type Query {
  predict(input: InputData!, includeExplanation: Boolean): PredictionResult!
  explainPrediction(predictionId: ID!, method: ExplanationMethod): Explanation!
}

type PredictionResult {
  value: Float!
  class: String
  confidence: Float
  explanation: Explanation
}

type Explanation {
  method: String!
  featureAttributions: [FeatureAttribution!]!
  qualityMetrics: QualityMetrics!
}
```

### gRPC

```protobuf
service ExplainableAI {
  rpc Predict(PredictionRequest) returns (PredictionResponse);
  rpc Explain(ExplanationRequest) returns (ExplanationResponse);
  rpc BatchExplain(BatchExplanationRequest) returns (stream ExplanationResponse);
}

message ExplanationRequest {
  string model_id = 1;
  map<string, Value> input = 2;
  ExplanationMethod method = 3;
  ExplanationConfig config = 4;
}
```

## Framework Integrations

### TensorFlow

```python
from wia_ai_009.tensorflow import TFExplainer

model = tf.keras.models.load_model('model.h5')
explainer = TFExplainer(model, method='integrated_gradients')

explanation = explainer.explain(input_data)
```

### PyTorch

```python
from wia_ai_009.pytorch import TorchExplainer

model = torch.load('model.pt')
model.eval()

explainer = TorchExplainer(model, method='gradcam')
heatmap = explainer.explain_image(image_tensor)
```

### Scikit-learn

```python
from wia_ai_009.sklearn import SklearnExplainer

model = joblib.load('model.pkl')
explainer = SklearnExplainer(model, method='shap')

explanation = explainer.explain(instance)
```

### XGBoost

```python
from wia_ai_009.xgboost import XGBoostExplainer

model = xgb.Booster()
model.load_model('model.json')

explainer = XGBoostExplainer(model)  # Automatically uses TreeSHAP
explanation = explainer.explain(dmatrix)
```

## Cloud Platform Integration

### AWS SageMaker

```python
import sagemaker
from wia_ai_009.aws import SageMakerExplainer

predictor = sagemaker.predictor.Predictor(endpoint_name='my-endpoint')
explainer = SageMakerExplainer(predictor)

explanation = explainer.explain(input_data)
```

### Google Vertex AI

```python
from wia_ai_009.gcp import VertexAIExplainer

endpoint = aiplatform.Endpoint(endpoint_name='projects/.../endpoints/...')
explainer = VertexAIExplainer(endpoint)
```

### Azure ML

```python
from wia_ai_009.azure import AzureMLExplainer

workspace = Workspace.from_config()
webservice = Webservice(workspace, 'my-service')
explainer = AzureMLExplainer(webservice)
```

## Monitoring and Logging

### Prometheus Metrics

```python
from prometheus_client import Histogram, Counter

explanation_duration = Histogram(
    'wia_explanation_duration_seconds',
    'Time spent generating explanations'
)

explanation_errors = Counter(
    'wia_explanation_errors_total',
    'Total explanation errors'
)

@explanation_duration.time()
def generate_explanation(input):
    try:
        return explainer.explain(input)
    except Exception as e:
        explanation_errors.inc()
        raise
```

### Structured Logging

```typescript
import { Logger } from 'wia-ai-009/logging';

const logger = new Logger({
  service: 'explanation-service',
  standard: 'WIA-AI-009'
});

logger.info('Explanation generated', {
  model_id: 'loan_v2',
  method: 'shap',
  fidelity: 0.94,
  duration_ms: 245
});
```

## Audit Trail

```json
{
  "audit_id": "uuid",
  "timestamp": "2025-01-20T15:30:00Z",
  "user_id": "user@example.com",
  "action": "explanation_requested",
  "model_id": "loan_classifier_v2.3",
  "input_hash": "sha256:...",
  "explanation_id": "uuid",
  "method": "shap",
  "quality_metrics": {
    "fidelity": 0.94,
    "stability": 0.89
  },
  "compliance_flags": ["GDPR", "EU_AI_ACT"]
}
```

## Testing Integration

```typescript
import { ExplanationTestSuite } from 'wia-ai-009/testing';

describe('Loan Model Explanations', () => {
  const suite = new ExplanationTestSuite(model, explainer);

  test('fidelity meets threshold', async () => {
    const result = await suite.testFidelity(testSet);
    expect(result.fidelity).toBeGreaterThan(0.85);
  });

  test('fairness across demographics', async () => {
    const result = await suite.testFairness(testSet, {
      protectedAttributes: ['age', 'gender']
    });
    expect(result.fidelityParity).toBeLessThan(0.05);
  });
});
```

---

**弘益人間** - Seamless integration makes XAI accessible to all stakeholders in the AI lifecycle.

© 2025 SmileStory Inc. / WIA | WIA-AI-009 Phase 4
