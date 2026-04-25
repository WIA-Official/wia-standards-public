/**
 * WIA-BIO-004: Biomarker Data SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Biomarker Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides computational tools for biomarker data analysis including:
 * - ROC curve calculation and AUC
 * - Sensitivity and specificity analysis
 * - Biomarker validation
 * - Clinical correlation analysis
 * - Outcome prediction
 */

import {
  Biomarker,
  BiomarkerMeasurement,
  BiomarkerValidation,
  ValidationResult,
  ROCRequest,
  ROCResult,
  ROCPoint,
  DiagnosticPerformance,
  ClinicalCorrelation,
  ClinicalOutcome,
  PrognosticAnalysis,
  TreatmentResponse,
  BiomarkerReport,
  SensitivityAnalysisParams,
  SensitivityAnalysisResult,
  PredictionModel,
  OutcomePrediction,
  PredictionResult,
  StatisticalTest,
  ClinicalUtility,
  BIOMARKER_CONSTANTS,
  BiomarkerErrorCode,
  BiomarkerError,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-BIO-004 Biomarker Data SDK
 */
export class BiomarkerDataSDK {
  private version = '1.0.0';
  private initialized = false;

  constructor() {
    this.initialized = true;
  }

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  /**
   * Calculate ROC curve and AUC
   *
   * @param request - ROC analysis request
   * @returns ROC analysis result with AUC and curve points
   */
  calculateROC(request: ROCRequest): ROCResult {
    const {
      trueLabels,
      predictions,
      thresholds,
      calculateCI = true,
      confidenceLevel = 0.95,
    } = request;

    // Validate inputs
    if (trueLabels.length !== predictions.length) {
      throw new BiomarkerError(
        BiomarkerErrorCode.INVALID_MEASUREMENTS,
        'True labels and predictions must have same length'
      );
    }

    if (trueLabels.length < BIOMARKER_CONSTANTS.MIN_VALIDATION_SAMPLES) {
      throw new BiomarkerError(
        BiomarkerErrorCode.INSUFFICIENT_SAMPLES,
        `Insufficient samples: need at least ${BIOMARKER_CONSTANTS.MIN_VALIDATION_SAMPLES}`
      );
    }

    const nDiseased = trueLabels.filter((l) => l === 1).length;
    const nHealthy = trueLabels.filter((l) => l === 0).length;

    if (nDiseased === 0 || nHealthy === 0) {
      throw new BiomarkerError(
        BiomarkerErrorCode.INVALID_MEASUREMENTS,
        'Must have both diseased and healthy samples'
      );
    }

    // Generate thresholds if not provided
    let evalThresholds: number[];
    if (thresholds && thresholds.length > 0) {
      evalThresholds = [...thresholds].sort((a, b) => a - b);
    } else {
      // Use unique prediction values as thresholds
      const uniqueValues = [...new Set(predictions)].sort((a, b) => a - b);
      const step = Math.max(1, Math.floor(uniqueValues.length / 100));
      evalThresholds = uniqueValues.filter((_, i) => i % step === 0);

      // Always include min and max
      if (evalThresholds[0] !== uniqueValues[0]) {
        evalThresholds.unshift(uniqueValues[0] - 0.001);
      }
      if (evalThresholds[evalThresholds.length - 1] !== uniqueValues[uniqueValues.length - 1]) {
        evalThresholds.push(uniqueValues[uniqueValues.length - 1] + 0.001);
      }
    }

    // Calculate ROC points for each threshold
    const rocCurve: ROCPoint[] = evalThresholds.map((threshold) => {
      const { tp, tn, fp, fn } = this.calculateConfusionMatrix(
        trueLabels,
        predictions,
        threshold
      );

      const sensitivity = tp / (tp + fn);
      const specificity = tn / (tn + fp);
      const fpr = 1 - specificity;
      const tpr = sensitivity;
      const youdenIndex = sensitivity + specificity - 1;

      return {
        threshold,
        sensitivity,
        specificity,
        fpr,
        tpr,
        youdenIndex,
      };
    });

    // Calculate AUC using trapezoidal rule
    const auc = this.calculateAUCTrapezoidal(rocCurve);

    // Find optimal threshold (max Youden Index)
    const optimalPoint = rocCurve.reduce((max, point) =>
      point.youdenIndex > max.youdenIndex ? point : max
    );

    const optimalThreshold = {
      value: optimalPoint.threshold,
      sensitivity: optimalPoint.sensitivity,
      specificity: optimalPoint.specificity,
      youdenIndex: optimalPoint.youdenIndex,
    };

    // Calculate confidence interval if requested
    let aucCI95: [number, number] | undefined;
    if (calculateCI) {
      aucCI95 = this.calculateAUCConfidenceInterval(auc, nDiseased, nHealthy, confidenceLevel);
    }

    return {
      auc,
      aucCI95,
      rocCurve,
      optimalThreshold,
      nDiseased,
      nHealthy,
    };
  }

  /**
   * Validate biomarker performance
   *
   * @param validation - Biomarker validation parameters
   * @returns Validation result with performance metrics
   */
  validateBiomarker(validation: BiomarkerValidation): ValidationResult {
    const { biomarkerType, measurements, referenceRange, cutoffThreshold, prevalence = 0.5 } =
      validation;

    const errors: string[] = [];
    const warnings: string[] = [];

    // Check sample size
    if (measurements.length < BIOMARKER_CONSTANTS.MIN_VALIDATION_SAMPLES) {
      errors.push(
        `Insufficient samples: ${measurements.length} (need ≥${BIOMARKER_CONSTANTS.MIN_VALIDATION_SAMPLES})`
      );
    }

    // Separate diseased and healthy
    const diseased = measurements.filter((m) => m.diseaseStatus === true);
    const healthy = measurements.filter((m) => m.diseaseStatus === false);

    if (diseased.length === 0 || healthy.length === 0) {
      errors.push('Must have both diseased and healthy samples');
    }

    const sampleSizeAdequate = measurements.length >= BIOMARKER_CONSTANTS.MIN_VALIDATION_SAMPLES;

    // Determine cutoff
    let threshold: number;
    if (cutoffThreshold !== undefined) {
      threshold = cutoffThreshold;
    } else {
      // Use ROC analysis to find optimal threshold
      const trueLabels = measurements.map((m) => (m.diseaseStatus ? 1 : 0));
      const predictions = measurements.map((m) => m.value);

      const rocResult = this.calculateROC({ trueLabels, predictions });
      threshold = rocResult.optimalThreshold.value;

      warnings.push(`Using optimal threshold from ROC: ${threshold.toFixed(2)} ${measurements[0]?.unit || ''}`);
    }

    // Calculate performance metrics
    const trueLabels = measurements.map((m) => (m.diseaseStatus ? 1 : 0));
    const predictions = measurements.map((m) => m.value);

    const performance = this.calculateDiagnosticPerformance(
      trueLabels,
      predictions,
      threshold,
      prevalence
    );

    // ROC analysis
    const rocAnalysis = this.calculateROC({ trueLabels, predictions });

    // Assess clinical utility
    let clinicalUtility: ClinicalUtility;
    if (rocAnalysis.auc >= BIOMARKER_CONSTANTS.CLINICAL_UTILITY_THRESHOLDS.excellent) {
      clinicalUtility = 'excellent';
    } else if (rocAnalysis.auc >= BIOMARKER_CONSTANTS.CLINICAL_UTILITY_THRESHOLDS.good) {
      clinicalUtility = 'good';
    } else if (rocAnalysis.auc >= BIOMARKER_CONSTANTS.CLINICAL_UTILITY_THRESHOLDS.fair) {
      clinicalUtility = 'fair';
    } else if (rocAnalysis.auc >= BIOMARKER_CONSTANTS.CLINICAL_UTILITY_THRESHOLDS.poor) {
      clinicalUtility = 'poor';
    } else {
      clinicalUtility = 'insufficient';
    }

    // Add warnings for poor performance
    if (performance.sensitivity < 80) {
      warnings.push(`Low sensitivity: ${performance.sensitivity.toFixed(1)}% (target: ≥80%)`);
    }

    if (performance.specificity < 80) {
      warnings.push(`Low specificity: ${performance.specificity.toFixed(1)}% (target: ≥80%)`);
    }

    if (rocAnalysis.auc < BIOMARKER_CONSTANTS.MIN_AUC_CLINICAL) {
      warnings.push(
        `AUC below clinical utility threshold: ${rocAnalysis.auc.toFixed(3)} (need ≥${BIOMARKER_CONSTANTS.MIN_AUC_CLINICAL})`
      );
    }

    const isValid = errors.length === 0 && clinicalUtility !== 'insufficient';

    return {
      isValid,
      performance,
      rocAnalysis,
      recommendedCutoff: threshold,
      clinicalUtility,
      warnings,
      errors,
      sampleSizeAdequate,
    };
  }

  /**
   * Analyze sensitivity across different thresholds
   *
   * @param params - Sensitivity analysis parameters
   * @returns Sensitivity analysis results
   */
  analyzeSensitivity(params: SensitivityAnalysisParams): SensitivityAnalysisResult {
    const { measurements, thresholdRange, prevalenceScenarios = [0.1, 0.2, 0.5, 0.7, 0.9] } =
      params;

    const trueLabels = measurements.map((m) => (m.diseaseStatus ? 1 : 0));
    const predictions = measurements.map((m) => m.value);

    // Determine threshold range
    let thresholds: number[];
    if (thresholdRange) {
      const { min, max, step } = thresholdRange;
      thresholds = [];
      for (let t = min; t <= max; t += step) {
        thresholds.push(t);
      }
    } else {
      // Use ROC curve thresholds
      const rocResult = this.calculateROC({ trueLabels, predictions });
      thresholds = rocResult.rocCurve.map((p) => p.threshold);
    }

    // Analyze performance at each threshold
    const thresholdAnalysis = thresholds.map((threshold) => {
      const perf = this.calculateDiagnosticPerformance(trueLabels, predictions, threshold, 0.5);
      return {
        threshold,
        sensitivity: perf.sensitivity,
        specificity: perf.specificity,
        ppv: perf.ppv,
        npv: perf.npv,
        accuracy: perf.accuracy,
      };
    });

    // Analyze prevalence impact using ROC optimal threshold
    const rocResult = this.calculateROC({ trueLabels, predictions });
    const optimalThreshold = rocResult.optimalThreshold.value;

    const prevalenceImpact = prevalenceScenarios.map((prevalence) => {
      const perf = this.calculateDiagnosticPerformance(
        trueLabels,
        predictions,
        optimalThreshold,
        prevalence
      );
      return {
        prevalence,
        ppv: perf.ppv,
        npv: perf.npv,
      };
    });

    // Find optimal thresholds by different criteria
    const maxYoudenPoint = rocResult.optimalThreshold.value;

    const maxAccuracyPoint = thresholdAnalysis.reduce((max, point) =>
      point.accuracy > max.accuracy ? point : max
    );

    // Find threshold for 90% sensitivity
    const sens90Point = thresholdAnalysis
      .filter((p) => p.sensitivity >= 90)
      .sort((a, b) => b.specificity - a.specificity)[0];

    // Find threshold for 90% specificity
    const spec90Point = thresholdAnalysis
      .filter((p) => p.specificity >= 90)
      .sort((a, b) => b.sensitivity - a.sensitivity)[0];

    return {
      thresholdAnalysis,
      prevalenceImpact,
      optimalThresholds: {
        maxYouden: maxYoudenPoint,
        maxAccuracy: maxAccuracyPoint.threshold,
        sensitivity90: sens90Point?.threshold || maxYoudenPoint,
        specificity90: spec90Point?.threshold || maxYoudenPoint,
      },
    };
  }

  /**
   * Predict clinical outcome using biomarker model
   *
   * @param prediction - Outcome prediction request
   * @returns Prediction result
   */
  predictOutcome(prediction: OutcomePrediction): PredictionResult {
    const { model, biomarkerValues, clinicalVariables = {} } = prediction;

    // Calculate prediction score based on model type
    let score: number;

    if (model.type === 'logistic_regression') {
      // Logistic regression: linear combination
      let linearSum = model.parameters['intercept'] || 0;

      for (const biomarker of model.biomarkers) {
        const value = biomarkerValues[biomarker] || 0;
        const coef = model.parameters[biomarker] || 0;
        linearSum += coef * value;
      }

      // Logistic function
      score = 1 / (1 + Math.exp(-linearSum));
    } else {
      // For other models, use simple weighted average
      let sum = 0;
      let count = 0;

      for (const biomarker of model.biomarkers) {
        const value = biomarkerValues[biomarker];
        if (value !== undefined) {
          const weight = model.parameters[biomarker] || 1;
          sum += value * weight;
          count++;
        }
      }

      score = count > 0 ? sum / count : 0.5;

      // Normalize to [0, 1]
      score = Math.max(0, Math.min(1, score));
    }

    // Determine prediction and risk category
    const predictionResult: 'positive' | 'negative' = score >= 0.5 ? 'positive' : 'negative';

    let riskCategory: 'low' | 'medium' | 'high';
    if (score < 0.3) {
      riskCategory = 'low';
    } else if (score < 0.7) {
      riskCategory = 'medium';
    } else {
      riskCategory = 'high';
    }

    // Calculate contributing factors
    const contributingFactors = model.biomarkers
      .map((biomarker) => ({
        factor: biomarker,
        contribution: (model.parameters[biomarker] || 0) * (biomarkerValues[biomarker] || 0),
      }))
      .sort((a, b) => Math.abs(b.contribution) - Math.abs(a.contribution));

    return {
      prediction: predictionResult,
      probability: score,
      riskCategory,
      contributingFactors,
    };
  }

  /**
   * Generate comprehensive biomarker report
   *
   * @param biomarker - Biomarker information
   * @param validation - Validation result
   * @param clinicalData - Optional clinical correlation data
   * @returns Biomarker report
   */
  generateReport(
    biomarker: Biomarker,
    validation: ValidationResult,
    clinicalData?: {
      correlations?: ClinicalCorrelation[];
      prognostic?: PrognosticAnalysis;
    }
  ): BiomarkerReport {
    const recommendations: string[] = [];

    // Generate recommendations based on performance
    if (validation.performance.sensitivity < 80) {
      recommendations.push('Consider lowering threshold to improve sensitivity');
    }

    if (validation.performance.specificity < 80) {
      recommendations.push('Consider raising threshold to improve specificity');
    }

    if (validation.clinicalUtility === 'excellent') {
      recommendations.push('Biomarker shows excellent clinical utility - ready for clinical implementation');
    } else if (validation.clinicalUtility === 'good') {
      recommendations.push('Biomarker shows good performance - consider multi-center validation');
    } else if (validation.clinicalUtility === 'fair') {
      recommendations.push('Biomarker shows fair performance - may benefit from combination with other markers');
    } else {
      recommendations.push('Biomarker shows insufficient performance - not recommended for clinical use');
    }

    if (!validation.sampleSizeAdequate) {
      recommendations.push('Increase sample size for more robust validation');
    }

    return {
      id: `REPORT-${Date.now()}`,
      type: 'validation',
      biomarker,
      population: {
        totalParticipants: validation.performance.confusionMatrix.tp +
          validation.performance.confusionMatrix.tn +
          validation.performance.confusionMatrix.fp +
          validation.performance.confusionMatrix.fn,
        cases: validation.performance.confusionMatrix.tp +
          validation.performance.confusionMatrix.fn,
        controls: validation.performance.confusionMatrix.tn +
          validation.performance.confusionMatrix.fp,
      },
      diagnosticPerformance: validation.performance,
      rocAnalysis: validation.rocAnalysis,
      clinicalCorrelations: clinicalData?.correlations,
      prognosticAnalysis: clinicalData?.prognostic,
      recommendations,
      metadata: {
        dateGenerated: new Date(),
        version: this.version,
      },
    };
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  /**
   * Calculate confusion matrix
   */
  private calculateConfusionMatrix(
    trueLabels: number[],
    predictions: number[],
    threshold: number
  ): { tp: number; tn: number; fp: number; fn: number } {
    let tp = 0,
      tn = 0,
      fp = 0,
      fn = 0;

    for (let i = 0; i < trueLabels.length; i++) {
      const trueLabel = trueLabels[i];
      const prediction = predictions[i] >= threshold ? 1 : 0;

      if (trueLabel === 1 && prediction === 1) tp++;
      else if (trueLabel === 0 && prediction === 0) tn++;
      else if (trueLabel === 0 && prediction === 1) fp++;
      else if (trueLabel === 1 && prediction === 0) fn++;
    }

    return { tp, tn, fp, fn };
  }

  /**
   * Calculate diagnostic performance metrics
   */
  private calculateDiagnosticPerformance(
    trueLabels: number[],
    predictions: number[],
    threshold: number,
    prevalence: number
  ): DiagnosticPerformance {
    const { tp, tn, fp, fn } = this.calculateConfusionMatrix(trueLabels, predictions, threshold);

    const sensitivity = tp / (tp + fn) * 100;
    const specificity = tn / (tn + fp) * 100;
    const accuracy = (tp + tn) / (tp + tn + fp + fn) * 100;

    // Calculate PPV and NPV using prevalence
    const sens = sensitivity / 100;
    const spec = specificity / 100;

    const ppv = (sens * prevalence) / (sens * prevalence + (1 - spec) * (1 - prevalence)) * 100;
    const npv = (spec * (1 - prevalence)) / (spec * (1 - prevalence) + (1 - sens) * prevalence) * 100;

    // Likelihood ratios
    const lrPositive = sens / (1 - spec);
    const lrNegative = (1 - sens) / spec;

    return {
      sensitivity,
      specificity,
      ppv,
      npv,
      accuracy,
      lrPositive,
      lrNegative,
      confusionMatrix: { tp, tn, fp, fn },
      prevalence,
    };
  }

  /**
   * Calculate AUC using trapezoidal rule
   */
  private calculateAUCTrapezoidal(rocCurve: ROCPoint[]): number {
    // Sort by FPR
    const sorted = [...rocCurve].sort((a, b) => a.fpr - b.fpr);

    let auc = 0;
    for (let i = 1; i < sorted.length; i++) {
      const width = sorted[i].fpr - sorted[i - 1].fpr;
      const height = (sorted[i].tpr + sorted[i - 1].tpr) / 2;
      auc += width * height;
    }

    return Math.max(0, Math.min(1, auc));
  }

  /**
   * Calculate AUC confidence interval (DeLong method approximation)
   */
  private calculateAUCConfidenceInterval(
    auc: number,
    nDiseased: number,
    nHealthy: number,
    confidenceLevel: number
  ): [number, number] {
    // Simplified Hanley & McNeil method
    const q1 = auc / (2 - auc);
    const q2 = (2 * auc * auc) / (1 + auc);

    const se = Math.sqrt(
      (auc * (1 - auc) + (nDiseased - 1) * (q1 - auc * auc) + (nHealthy - 1) * (q2 - auc * auc)) /
        (nDiseased * nHealthy)
    );

    // Z-score for confidence level
    const z = confidenceLevel === 0.95 ? 1.96 : 2.576; // 95% or 99%

    const lower = Math.max(0, auc - z * se);
    const upper = Math.min(1, auc + z * se);

    return [lower, upper];
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Calculate ROC curve (standalone function)
 */
export function calculateROC(request: ROCRequest): ROCResult {
  const sdk = new BiomarkerDataSDK();
  return sdk.calculateROC(request);
}

/**
 * Validate biomarker (standalone function)
 */
export function validateBiomarker(validation: BiomarkerValidation): ValidationResult {
  const sdk = new BiomarkerDataSDK();
  return sdk.validateBiomarker(validation);
}

/**
 * Analyze sensitivity (standalone function)
 */
export function analyzeSensitivity(params: SensitivityAnalysisParams): SensitivityAnalysisResult {
  const sdk = new BiomarkerDataSDK();
  return sdk.analyzeSensitivity(params);
}

/**
 * Predict outcome (standalone function)
 */
export function predictOutcome(prediction: OutcomePrediction): PredictionResult {
  const sdk = new BiomarkerDataSDK();
  return sdk.predictOutcome(prediction);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { BiomarkerDataSDK };
export default BiomarkerDataSDK;
