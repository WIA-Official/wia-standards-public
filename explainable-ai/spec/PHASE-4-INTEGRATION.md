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


## Annex E — Implementation Notes for PHASE-4-INTEGRATION

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-4-INTEGRATION.

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
evidence for PHASE-4-INTEGRATION. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-4-integration/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-4-INTEGRATION with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-4-INTEGRATION does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-4-INTEGRATION.
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
for PHASE-4-INTEGRATION. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P4-INTEGRATION-Edge-only/2` is asserting conformance with
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
  is published manually. Sufficient for PHASE-4-INTEGRATION validation when the
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
