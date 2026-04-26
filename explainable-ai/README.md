# WIA-AI-009: Explainable AI Standard 💡

**홍익인간 (弘益人間)** - Benefit All Humanity Through Transparent AI

## Overview

WIA-AI-009 is a comprehensive standard for implementing Explainable AI (XAI) across all machine learning systems. It provides standardized data formats, algorithms, protocols, and integration patterns to make AI transparent, trustworthy, and accountable.

### Philosophy

This standard embodies the Korean principle of **홍익인간 (弘益人間)** (Hongik Ingan) - "widely benefit humanity." We believe that for AI to truly serve humanity, it must be explainable, interpretable, and transparent.

## Quick Start

### Installation

```bash
npm install wia-ai-009
```

### Basic Usage

```typescript
import { SHAPExplainer } from 'wia-ai-009';

// Create explainer
const explainer = new SHAPExplainer(model, {
  variant: 'TreeSHAP',
  n_samples: 5000
});

// Generate explanation
const explanation = await explainer.explain({
  credit_score: 720,
  debt_ratio: 0.35,
  employment_years: 8
});

console.log(explanation.explanation.feature_attributions);
// {
//   credit_score: -0.22,
//   debt_ratio: 0.45,
//   employment_years: -0.18
// }
```

## Features

### 🎯 Core XAI Methods
- **SHAP** (Shapley Additive exPlanations)
  - TreeSHAP (exact, fast for tree models)
  - KernelSHAP (model-agnostic)
  - LinearSHAP (closed-form for linear models)
  - DeepSHAP (for neural networks)

- **LIME** (Local Interpretable Model-agnostic Explanations)
  - Tabular LIME
  - Image LIME
  - Text LIME

- **Attention Mechanisms**
  - Self-attention extraction
  - Multi-head attention aggregation
  - Grad-CAM for CNNs

- **Integrated Gradients**
  - Riemann trapezoidal method
  - Configurable baselines

- **Counterfactual Explanations**
  - Minimal change sets
  - Actionability scoring

### 📊 Trust Metrics
- **Fidelity** - How accurately explanations represent model behavior
- **Consistency** - Similar inputs produce similar explanations
- **Stability** - Reproducibility across multiple runs
- **Completeness** - Attributions account for full prediction
- **Comprehensibility** - Human understandability metrics

### 🛡️ Fairness Validation
- Fidelity parity across demographic groups
- Protected attribute weight detection
- Disparate impact analysis

### 🔌 Framework Integration
- TensorFlow / Keras
- PyTorch
- Scikit-learn
- XGBoost / LightGBM

## Standard Structure

The WIA-AI-009 standard is organized into four phases:

### Phase 1: Data Format & Explanation Types
Defines standardized JSON schemas for explanations, ensuring interoperability across tools and platforms.

**See:** [`spec/PHASE-1-DATA-FORMAT.md`](spec/PHASE-1-DATA-FORMAT.md)

### Phase 2: XAI Algorithms & Methods
Implements core explainability algorithms with consistent APIs.

**See:** [`spec/PHASE-2-API.md`](spec/PHASE-2-API.md)

### Phase 3: Explanation Protocol & Trust Metrics
Establishes protocols for requesting, generating, and validating explanations.

**See:** [`spec/PHASE-3-PROTOCOL.md`](spec/PHASE-3-PROTOCOL.md)

### Phase 4: Integration & Visualization
Integrates XAI into ML pipelines and provides visualization tools.

**See:** [`spec/PHASE-4-INTEGRATION.md`](spec/PHASE-4-INTEGRATION.md)

## Documentation

### 📖 Complete eBooks
- [English eBook](ebook/en/index.html) - 8 comprehensive chapters
- [Korean eBook](ebook/ko/index.html) - 8 comprehensive chapters (한글)

### 🎮 Interactive Simulator
Try the [Interactive XAI Simulator](simulator/index.html) to experiment with different explanation methods and see how they work in real-time.

## Examples

### SHAP for Tree Models

```typescript
import { TreeSHAPExplainer } from 'wia-ai-009';

const explainer = new TreeSHAPExplainer(xgboostModel, {
  background_data: trainingData.sample(1000)
});

const explanation = await explainer.explain(instance);

// Verify additivity
const sum = Object.values(explanation.explanation.feature_attributions)
  .reduce((a, b) => a + b, 0);

console.log(sum === explanation.prediction.value - explanation.explanation.base_value);
// true (within floating point precision)
```

### LIME for Any Model

```typescript
import { LIMEExplainer } from 'wia-ai-009';

const explainer = new LIMEExplainer(model, {
  n_samples: 5000,
  kernel_width: 0.75,
  n_features: 10
});

const explanation = await explainer.explain(instance);

console.log('Local R² score:', explanation.quality_metrics.fidelity);
```

### Validation & Quality Assurance

```typescript
import { validateExplanationQuality } from 'wia-ai-009';

const explanation = await explainer.explain(instance);
const validation = validateExplanationQuality(explanation);

if (!validation.passed) {
  console.warn('Quality issues:', validation.issues);
  // ["Fidelity 0.82 below threshold 0.85"]
}
```

### Batch Explanations

```typescript
const explanations = await explainer.explainBatch([
  instance1,
  instance2,
  instance3
]);

// Aggregate for global insights
const globalImportance = aggregateAttributions(explanations);
```

## API Reference

### Core Classes

#### `SHAPExplainer`
```typescript
class SHAPExplainer extends BaseExplainer {
  constructor(model: IModel, config: SHAPConfig);
  explain(input: Record<string, any>, config?: SHAPConfig): Promise<Explanation>;
  validate(testSet: any[], config?: SHAPConfig): Promise<ValidationReport>;
}
```

#### `LIMEExplainer`
```typescript
class LIMEExplainer extends BaseExplainer {
  constructor(model: IModel, config: LIMEConfig);
  explain(input: Record<string, any>, config?: LIMEConfig): Promise<Explanation>;
}
```

### Utility Functions

#### `createExplanationRequest()`
Creates a standardized WIA-AI-009 explanation request.

#### `validateExplanationQuality()`
Validates explanation against quality thresholds.

#### `formatAttributions()`
Formats feature attributions for display.

## Quality Thresholds

The standard defines minimum quality thresholds:

| Metric | Threshold | Description |
|--------|-----------|-------------|
| Fidelity | ≥ 0.85 | Explanation accuracy |
| Consistency | ≥ 0.75 | Similar inputs → similar explanations |
| Stability | ≥ 0.80 | Reproducibility |
| Completeness | ≥ 0.95 | Full accounting (additive methods) |
| Fairness Parity | < 0.05 | Cross-group consistency |
| Protected Attribute Weight | < 0.05 | Avoid discrimination |

## Regulatory Compliance

WIA-AI-009 helps meet regulatory requirements:

- ✅ **GDPR Article 22** - Right to explanation for automated decisions
- ✅ **EU AI Act** - Transparency requirements for high-risk AI
- ✅ **Fair Lending Laws** - Adverse action notices
- ✅ **EEOC Guidance** - Algorithmic hiring transparency

## Testing

```bash
npm test
```

Run the full test suite to validate:
- SHAP additivity property
- LIME fidelity on test sets
- Cross-method agreement
- Fairness metrics across groups

## Contributing

We welcome contributions! Please see our [Contributing Guidelines](CONTRIBUTING.md).

### Areas for Contribution
- Additional explanation methods
- Framework integrations
- Visualization components
- Documentation improvements
- Bug fixes and performance optimizations

## Roadmap

### v1.1 (Q2 2025)
- [ ] Causal explanation methods
- [ ] Interactive explanation APIs
- [ ] Multi-modal explanation support
- [ ] Performance optimizations for large models

### v2.0 (Q4 2025)
- [ ] Distributed explanation computation
- [ ] Real-time explanation streaming
- [ ] Advanced fairness metrics
- [ ] Conversational XAI interfaces

## License

MIT License - see [LICENSE](LICENSE) for details

## Citation

If you use WIA-AI-009 in your research or production systems, please cite:

```bibtex
@standard{wia-ai-009,
  title={WIA-AI-009: Explainable AI Standard},
  author={SmileStory Inc. / WIA},
  year={2025},
  url={https://github.com/WIA-Official/wia-standards/tree/main/explainable-ai},
  philosophy={홍익인간 (弘益人間) - Benefit All Humanity}
}
```

## Support

- **Documentation**: [ebook/en/index.html](ebook/en/index.html)
- **Issues**: [GitHub Issues](https://github.com/WIA-Official/wia-standards/issues)
- **Discussions**: [GitHub Discussions](https://github.com/WIA-Official/wia-standards/discussions)
- **Email**: standards@wia.org

## Related Standards

- **WIA-INTENT** - Intent representation standard
- **WIA-OMNI-API** - Universal API framework
- **WIA-SOCIAL** - Social collaboration standard

## Acknowledgments

Special thanks to:
- The XAI research community for foundational methods (SHAP, LIME, etc.)
- Open source contributors
- Early adopters providing feedback

---

## 홍익인간 (弘益人間)

**Widely Benefit Humanity**

This standard exists to ensure that as AI becomes more powerful and pervasive, it remains transparent, understandable, and accountable to the humans it serves. Explainable AI is not just a technical challenge—it's a moral imperative.

© 2025 SmileStory Inc. / WIA | WIA-AI-009 Standard

---

**Quick Links:**
- [📖 English eBook](ebook/en/index.html)
- [📚 Korean eBook](ebook/ko/index.html)
- [🎮 Interactive Simulator](simulator/index.html)
- [📋 Phase 1 Spec](spec/PHASE-1-DATA-FORMAT.md)
- [🔍 Phase 2 Spec](spec/PHASE-2-API.md)
- [🔗 Phase 3 Spec](spec/PHASE-3-PROTOCOL.md)
- [🔌 Phase 4 Spec](spec/PHASE-4-INTEGRATION.md)
- [💻 TypeScript API](api/typescript/)
