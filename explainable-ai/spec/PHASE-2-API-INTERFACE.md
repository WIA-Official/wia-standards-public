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


## Annex E — Implementation Notes for PHASE-2-API-INTERFACE

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-2-API-INTERFACE.

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
evidence for PHASE-2-API-INTERFACE. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-2-api-interface/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-2-API-INTERFACE with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-2-API-INTERFACE does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-2-API-INTERFACE.
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
for PHASE-2-API-INTERFACE. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P2-API-INTERFACE-Edge-only/2` is asserting conformance with
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
  is published manually. Sufficient for PHASE-2-API-INTERFACE validation when the
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
