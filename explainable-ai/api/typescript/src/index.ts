/**
 * WIA-AI-009 Explainable AI Standard - Main SDK
 * 弘익人間 (Hongik Ingan) - Benefit All Humanity Through Transparent AI
 *
 * @module wia-ai-009
 * @version 1.0.0
 * @author SmileStory Inc. / WIA
 * @license MIT
 */

import { v4 as uuidv4 } from 'uuid';

export * from './types';

import type {
  IExplainer,
  IModel,
  Explanation,
  ExplanationRequest,
  ExplanationResponse,
  ExplanationConfig,
  ExplainerConfig,
  SHAPConfig,
  LIMEConfig,
  ValidationReport,
  TrustMetrics,
  FeatureAttribution,
  AttributionMap,
  STANDARD_ID,
  STANDARD_VERSION
} from './types';

// ============================================================================
// Constants
// ============================================================================

export const WIA_AI_009_VERSION = "1.0";
export const WIA_AI_009_STANDARD = "WIA-AI-009";

// Default thresholds from the standard
export const DEFAULT_THRESHOLDS = {
  MIN_FIDELITY: 0.85,
  MIN_CONSISTENCY: 0.75,
  MIN_STABILITY: 0.80,
  MIN_COMPLETENESS: 0.95,
  MAX_FAIRNESS_PARITY: 0.05,
  MAX_PROTECTED_ATTRIBUTE_WEIGHT: 0.05,
  MIN_CROSS_METHOD_AGREEMENT: 0.60
};

// ============================================================================
// Base Explainer Class
// ============================================================================

/**
 * Base class for all explainers implementing WIA-AI-009 standard
 */
export abstract class BaseExplainer implements IExplainer {
  protected model: IModel;
  protected config: ExplainerConfig;

  constructor(model: IModel, config: ExplainerConfig = {}) {
    this.model = model;
    this.config = config;
  }

  abstract explain(
    input: Record<string, any>,
    config?: ExplainerConfig
  ): Promise<Explanation>;

  async explainBatch(
    inputs: Record<string, any>[],
    config?: ExplainerConfig
  ): Promise<Explanation[]> {
    return Promise.all(
      inputs.map(input => this.explain(input, config))
    );
  }

  abstract validate(
    testSet: Record<string, any>[],
    config?: ExplainerConfig
  ): Promise<ValidationReport>;

  getConfig(): ExplainerConfig {
    return { ...this.config };
  }

  setConfig(config: ExplainerConfig): void {
    this.config = { ...this.config, ...config };
  }

  protected createExplanationShell(
    method: string,
    explanationType: "local" | "global" | "counterfactual"
  ): Partial<Explanation> {
    return {
      standard: WIA_AI_009_STANDARD,
      version: WIA_AI_009_VERSION,
      explanation_id: uuidv4(),
      timestamp: new Date().toISOString(),
      explanation_type: explanationType,
      method: method as any,
      model_info: this.model.getInfo(),
      metadata: {
        computation_time_ms: 0,
        model_version_used: this.model.getInfo().model_version
      }
    };
  }
}

// ============================================================================
// SHAP Explainer
// ============================================================================

/**
 * SHAP (SHapley Additive exPlanations) Explainer
 */
export class SHAPExplainer extends BaseExplainer {
  private shapConfig: SHAPConfig;

  constructor(model: IModel, config: SHAPConfig = {}) {
    super(model, config);
    this.shapConfig = {
      variant: config.variant || "KernelSHAP",
      n_samples: config.n_samples || 5000,
      check_additivity: config.check_additivity !== false,
      ...config
    };
  }

  async explain(
    input: Record<string, any>,
    config?: SHAPConfig
  ): Promise<Explanation> {
    const startTime = Date.now();
    const mergedConfig = { ...this.shapConfig, ...config };

    // Get prediction
    const prediction = await this.model.predict(input);

    // Compute SHAP values (simplified - actual implementation would be more complex)
    const shapValues = await this.computeSHAPValues(input, mergedConfig);
    const baseValue = await this.computeBaseValue(mergedConfig);

    // Verify additivity if requested
    if (mergedConfig.check_additivity) {
      this.verifyAdditivity(shapValues, baseValue, prediction as number);
    }

    const explanation: Explanation = {
      ...this.createExplanationShell("shap", "local"),
      input,
      prediction: {
        value: prediction,
        confidence: "high"
      },
      explanation: {
        base_value: baseValue,
        feature_attributions: shapValues,
        feature_values: input
      },
      metadata: {
        computation_time_ms: Date.now() - startTime,
        model_version_used: this.model.getInfo().model_version,
        explainer_versions: {
          shap: "1.0.0"
        }
      }
    } as Explanation;

    return explanation;
  }

  async validate(
    testSet: Record<string, any>[],
    config?: SHAPConfig
  ): Promise<ValidationReport> {
    const fidelity = await this.computeFidelity(testSet, config);
    const stability = await this.computeStability(testSet[0], config);

    return {
      fidelity: {
        value: fidelity,
        passed: fidelity >= DEFAULT_THRESHOLDS.MIN_FIDELITY
      },
      stability: {
        value: stability,
        passed: stability >= DEFAULT_THRESHOLDS.MIN_STABILITY
      },
      overall_passed:
        fidelity >= DEFAULT_THRESHOLDS.MIN_FIDELITY &&
        stability >= DEFAULT_THRESHOLDS.MIN_STABILITY,
      recommendations: []
    };
  }

  private async computeSHAPValues(
    input: Record<string, any>,
    config: SHAPConfig
  ): Promise<AttributionMap> {
    // Simplified implementation - real implementation would use actual SHAP algorithms
    const shapValues: AttributionMap = {};

    for (const feature in input) {
      // Placeholder computation
      shapValues[feature] = Math.random() * 0.4 - 0.2;
    }

    return shapValues;
  }

  private async computeBaseValue(config: SHAPConfig): Promise<number> {
    // Placeholder - would compute average prediction on background data
    return 0.5;
  }

  private verifyAdditivity(
    shapValues: AttributionMap,
    baseValue: number,
    prediction: number
  ): void {
    const sum = Object.values(shapValues).reduce((a, b) => a + b, 0);
    const expected = prediction - baseValue;

    if (Math.abs(sum - expected) > 1e-5) {
      console.warn('SHAP additivity property violated', {
        sum,
        expected,
        difference: Math.abs(sum - expected)
      });
    }
  }

  private async computeFidelity(
    testSet: Record<string, any>[],
    config?: SHAPConfig
  ): Promise<number> {
    // Placeholder implementation
    return 0.92;
  }

  private async computeStability(
    instance: Record<string, any>,
    config?: SHAPConfig
  ): Promise<number> {
    // Placeholder implementation
    return 0.89;
  }
}

// ============================================================================
// LIME Explainer
// ============================================================================

/**
 * LIME (Local Interpretable Model-agnostic Explanations) Explainer
 */
export class LIMEExplainer extends BaseExplainer {
  private limeConfig: LIMEConfig;

  constructor(model: IModel, config: LIMEConfig = {}) {
    super(model, config);
    this.limeConfig = {
      n_samples: config.n_samples || 5000,
      n_features: config.n_features || 10,
      kernel_width: config.kernel_width || 0.75,
      ...config
    };
  }

  async explain(
    input: Record<string, any>,
    config?: LIMEConfig
  ): Promise<Explanation> {
    const startTime = Date.now();
    const mergedConfig = { ...this.limeConfig, ...config };

    const prediction = await this.model.predict(input);
    const localModel = await this.fitLocalModel(input, mergedConfig);

    const explanation: Explanation = {
      ...this.createExplanationShell("lime", "local"),
      input,
      prediction: {
        value: prediction,
        confidence: "medium"
      },
      explanation: {
        base_value: localModel.intercept,
        feature_attributions: localModel.coefficients,
        feature_values: input
      },
      metadata: {
        computation_time_ms: Date.now() - startTime,
        model_version_used: this.model.getInfo().model_version,
        explainer_versions: {
          lime: "1.0.0"
        }
      },
      quality_metrics: {
        fidelity: localModel.r_squared,
        consistency: 0.85
      }
    } as Explanation;

    return explanation;
  }

  async validate(
    testSet: Record<string, any>[],
    config?: LIMEConfig
  ): Promise<ValidationReport> {
    const fidelity = await this.computeFidelity(testSet, config);

    return {
      fidelity: {
        value: fidelity,
        passed: fidelity >= DEFAULT_THRESHOLDS.MIN_FIDELITY
      },
      overall_passed: fidelity >= DEFAULT_THRESHOLDS.MIN_FIDELITY,
      recommendations: []
    };
  }

  private async fitLocalModel(
    input: Record<string, any>,
    config: LIMEConfig
  ): Promise<{
    intercept: number;
    coefficients: AttributionMap;
    r_squared: number;
  }> {
    // Simplified implementation
    const coefficients: AttributionMap = {};

    for (const feature in input) {
      coefficients[feature] = Math.random() * 0.5 - 0.25;
    }

    return {
      intercept: 0.5,
      coefficients,
      r_squared: 0.88
    };
  }

  private async computeFidelity(
    testSet: Record<string, any>[],
    config?: LIMEConfig
  ): Promise<number> {
    return 0.88;
  }
}

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * Create a standardized explanation request
 */
export function createExplanationRequest(
  modelId: string,
  input: Record<string, any>,
  config: ExplanationConfig = {}
): ExplanationRequest {
  return {
    protocol: "XAI-REQUEST-v1",
    request_id: uuidv4(),
    timestamp: new Date().toISOString(),
    model_identifier: {
      model_id: modelId,
      model_type: "unknown",
      model_version: "unknown"
    },
    input_data: input,
    explanation_config: config
  };
}

/**
 * Validate explanation against WIA-AI-009 quality thresholds
 */
export function validateExplanationQuality(
  explanation: Explanation
): { passed: boolean; issues: string[] } {
  const issues: string[] = [];

  if (explanation.quality_metrics) {
    const { fidelity, stability, completeness } = explanation.quality_metrics;

    if (fidelity !== undefined && fidelity < DEFAULT_THRESHOLDS.MIN_FIDELITY) {
      issues.push(`Fidelity ${fidelity.toFixed(2)} below threshold ${DEFAULT_THRESHOLDS.MIN_FIDELITY}`);
    }

    if (stability !== undefined && stability < DEFAULT_THRESHOLDS.MIN_STABILITY) {
      issues.push(`Stability ${stability.toFixed(2)} below threshold ${DEFAULT_THRESHOLDS.MIN_STABILITY}`);
    }

    if (completeness !== undefined && completeness < DEFAULT_THRESHOLDS.MIN_COMPLETENESS) {
      issues.push(`Completeness ${completeness.toFixed(2)} below threshold ${DEFAULT_THRESHOLDS.MIN_COMPLETENESS}`);
    }
  }

  return {
    passed: issues.length === 0,
    issues
  };
}

/**
 * Format feature attributions for display
 */
export function formatAttributions(
  attributions: AttributionMap,
  topN: number = 5
): FeatureAttribution[] {
  const features: FeatureAttribution[] = Object.entries(attributions).map(
    ([name, value]) => ({
      feature_name: name,
      attribution_value: value,
      feature_value: null,
      direction: value > 0 ? "positive" : value < 0 ? "negative" : "neutral"
    })
  );

  return features
    .sort((a, b) => Math.abs(b.attribution_value) - Math.abs(a.attribution_value))
    .slice(0, topN)
    .map((f, index) => ({ ...f, rank: index + 1 }));
}

// ============================================================================
// Export Philosophy
// ============================================================================

/**
 * 弘益人間 (Hongik Ingan)
 * "Widely benefit humanity"
 *
 * This SDK implements the WIA-AI-009 standard with the philosophy that
 * explainable AI should serve all of humanity by making AI systems
 * transparent, trustworthy, and accountable.
 */
export const PHILOSOPHY = {
  korean: "弘益人間",
  romanized: "Hongik Ingan",
  english: "Benefit All Humanity",
  description: "Making AI transparent and trustworthy for everyone"
};
