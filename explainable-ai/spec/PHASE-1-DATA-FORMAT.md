# WIA-AI-009 Phase 1: Data Format & Explanation Types

## Overview

Phase 1 defines standardized data formats for explanations, feature attributions, and interpretation metadata. This ensures interoperability across different XAI tools and platforms.

**Philosophy**: 弘益人間 (Benefit All Humanity) - Standard formats enable universal access to AI explanations.

## Explanation Format Schema

### Core Structure

```json
{
  "standard": "WIA-AI-009",
  "version": "1.0",
  "explanation_id": "uuid-v4",
  "timestamp": "ISO-8601",
  "explanation_type": "local | global | counterfactual",
  "method": "shap | lime | attention | integrated_gradients | custom",
  "model_info": {
    "model_id": "string",
    "model_type": "string",
    "model_version": "string",
    "framework": "tensorflow | pytorch | sklearn | xgboost | custom"
  },
  "input": { /* original input data */ },
  "prediction": {
    "value": "number | string",
    "class": "string (for classification)",
    "probability": "number (0-1)",
    "confidence": "low | medium | high"
  },
  "explanation": { /* method-specific explanation data */ },
  "metadata": { /* computation details */ }
}
```

## Explanation Types

### 1. Local Explanations

Explain individual predictions.

```json
{
  "explanation_type": "local",
  "explanation": {
    "base_value": 0.15,
    "feature_attributions": {
      "credit_score": -0.22,
      "debt_ratio": 0.45,
      "employment_years": -0.18
    },
    "feature_values": {
      "credit_score": 720,
      "debt_ratio": 0.52,
      "employment_years": 8
    }
  }
}
```

### 2. Global Explanations

Characterize overall model behavior.

```json
{
  "explanation_type": "global",
  "explanation": {
    "feature_importance": {
      "credit_score": 0.35,
      "debt_ratio": 0.28,
      "employment_years": 0.18,
      "loan_amount": 0.12,
      "other": 0.07
    },
    "total_instances_analyzed": 10000,
    "aggregation_method": "mean_absolute_shap"
  }
}
```

### 3. Counterfactual Explanations

Show what changes would alter the prediction.

```json
{
  "explanation_type": "counterfactual",
  "explanation": {
    "original_prediction": "denied",
    "counterfactual_prediction": "approved",
    "required_changes": {
      "debt_ratio": {
        "current": 0.52,
        "required": 0.32,
        "change": -0.20
      },
      "credit_score": {
        "current": 680,
        "required": 720,
        "change": +40
      }
    },
    "minimal_change_set": ["debt_ratio"],
    "actionability_score": 0.82
  }
}
```

## Attribution Vector Format

```typescript
interface AttributionVector {
  feature_name: string;
  attribution_value: number;  // Positive = increases prediction, negative = decreases
  confidence_interval?: [number, number];
  rank?: number;  // Importance ranking
  direction: "positive" | "negative" | "neutral";
}

interface ExplanationData {
  attributions: AttributionVector[];
  base_value?: number;
  sum_check?: number;  // For additive methods, should equal prediction - base_value
}
```

## Feature Types

```json
{
  "features": [
    {
      "name": "credit_score",
      "type": "numeric",
      "value": 720,
      "range": [300, 850],
      "unit": "points",
      "description": "FICO credit score"
    },
    {
      "name": "employment_status",
      "type": "categorical",
      "value": "full_time",
      "categories": ["unemployed", "part_time", "full_time", "self_employed"]
    },
    {
      "name": "application_text",
      "type": "text",
      "value": "I need this loan for...",
      "tokenized": ["I", "need", "this", "loan", "for"]
    },
    {
      "name": "id_photo",
      "type": "image",
      "dimensions": [224, 224, 3],
      "format": "jpeg",
      "url": "https://..."
    }
  ]
}
```

## Trust Metrics Format

```json
{
  "quality_metrics": {
    "fidelity": 0.94,
    "consistency": 0.89,
    "stability": 0.91,
    "completeness": 0.98,
    "comprehensibility_score": 0.76
  },
  "fairness_metrics": {
    "fidelity_parity": 0.03,
    "stability_parity": 0.04,
    "protected_attribute_weight": 0.02
  },
  "validation": {
    "ablation_correlation": 0.79,
    "perturbation_alignment": 0.81,
    "cross_method_agreement": 0.76
  }
}
```

## Visualization Metadata

```json
{
  "visualization": {
    "recommended_type": "force_plot | bar_chart | heatmap | scatter | attention_map",
    "color_scheme": "diverging | sequential | categorical",
    "highlight_features": ["debt_ratio", "credit_score"],
    "chart_data": {
      /* Format specific to visualization type */
    }
  }
}
```

## Batch Explanation Format

For explaining multiple instances at once:

```json
{
  "standard": "WIA-AI-009",
  "batch_id": "uuid-v4",
  "explanations": [
    { /* Individual explanation 1 */ },
    { /* Individual explanation 2 */ },
    ...
  ],
  "batch_metadata": {
    "total_instances": 1000,
    "successful_explanations": 987,
    "failed_explanations": 13,
    "average_computation_time_ms": 245,
    "total_computation_time_s": 242
  }
}
```

## Error Format

```json
{
  "standard": "WIA-AI-009",
  "error": true,
  "error_code": "EXPLANATION_FAILED",
  "error_message": "Insufficient samples for LIME convergence",
  "error_details": {
    "method": "lime",
    "samples_requested": 5000,
    "samples_completed": 1243,
    "convergence_threshold": 0.85,
    "actual_r2": 0.42
  },
  "fallback_explanation": { /* Simpler method if available */ }
}
```

## Versioning and Compatibility

```json
{
  "standard": "WIA-AI-009",
  "version": "1.0",
  "backward_compatible_with": ["0.9"],
  "schema_url": "https://wia-standards.org/ai-009/v1.0/schema.json"
}
```

## Implementation Example

```typescript
import { WIAExplanation, ExplanationType } from 'wia-ai-009';

// Create explanation
const explanation: WIAExplanation = {
  standard: "WIA-AI-009",
  version: "1.0",
  explanation_id: generateUUID(),
  timestamp: new Date().toISOString(),
  explanation_type: ExplanationType.LOCAL,
  method: "shap",
  model_info: {
    model_id: "loan_classifier_v2.3",
    model_type: "xgboost",
    model_version: "2.3.1",
    framework: "xgboost"
  },
  input: inputData,
  prediction: {
    value: 0.73,
    class: "high_risk",
    probability: 0.73,
    confidence: "high"
  },
  explanation: {
    base_value: 0.15,
    feature_attributions: shapValues,
    feature_values: inputData
  },
  metadata: {
    computation_time_ms: 245,
    shap_variant: "TreeSHAP"
  }
};

// Validate format
const isValid = validateWIAFormat(explanation);
```

## Interoperability Guidelines

1. **Required Fields**: All implementations MUST include: `standard`, `version`, `explanation_type`, `method`
2. **Optional Fields**: `metadata`, `visualization`, `quality_metrics` are optional but recommended
3. **Extension Mechanism**: Use `custom_fields` object for vendor-specific extensions
4. **Encoding**: UTF-8 for text, ISO-8601 for timestamps, IEEE 754 for numbers
5. **Size Limits**: Individual explanations should not exceed 10MB; use batch format for larger datasets

## Compliance Checklist

- [ ] Uses WIA-AI-009 standard identifier
- [ ] Includes version number
- [ ] Specifies explanation type clearly
- [ ] Provides model identification information
- [ ] Includes original input and prediction
- [ ] Contains method-specific explanation data
- [ ] Validates against JSON schema
- [ ] Handles errors gracefully
- [ ] Includes timestamp and traceability information

---

**弘益人間** - Standard data formats ensure AI explanations are accessible, interoperable, and beneficial to all.

© 2025 SmileStory Inc. / WIA | WIA-AI-009 Phase 1
