/**
 * WIA-AUG-001: Human Augmentation SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Human Augmentation Working Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides tools for human augmentation classification, enhancement
 * measurement, compatibility assessment, and performance evaluation.
 */

import {
  AugmentationType,
  IntegrationMode,
  EnhancementLevel,
  CompatibilityLevel,
  ClassificationInput,
  ClassificationResult,
  SubjectInfo,
  BaselineMeasurement,
  BaselineRecord,
  BaselineUpdate,
  EnhancementRatioInput,
  EnhancementResult,
  MultiMetricEnhancement,
  AugmentationInfo,
  CompatibilityResult,
  TechnicalInterface,
  SafetyInteraction,
  PerformanceSynergy,
  PerformanceEvaluation,
  PerformanceMetrics,
  PerformanceCharacteristics,
  AugmentationStatus,
  SyncProtocol,
  SyncResult,
  ENHANCEMENT_THRESHOLDS,
  INTEGRATION_THRESHOLDS,
  COMPATIBILITY_THRESHOLDS,
  COMPATIBILITY_WEIGHTS,
  MINIMUM_PERFORMANCE,
  AugmentationErrorCode,
  AugmentationError,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-AUG-001 Human Augmentation SDK
 */
export class HumanAugmentationSDK {
  private version = '1.0.0';
  private baselineRegistry: Map<string, BaselineRecord> = new Map();

  constructor() {}

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  // ==========================================================================
  // Classification Methods
  // ==========================================================================

  /**
   * Classify an augmentation device or system
   */
  classifyAugmentation(input: ClassificationInput): ClassificationResult {
    // Validate input
    this.validateClassificationInput(input);

    // Determine type
    let type: AugmentationType = input.primaryDomain;
    if (input.secondaryDomains && input.secondaryDomains.length >= 2) {
      type = 'HYBRID';
    }

    // Determine enhancement level
    const level = this.determineEnhancementLevel(input.enhancementFactor);

    // Calculate safety level recommendation
    const safetyLevel = this.calculateSafetyLevel(
      input.integrationMode,
      input.integrationDepth,
      input.enhancementFactor
    );

    return {
      type,
      level,
      integrationMode: input.integrationMode,
      integrationDepth: input.integrationDepth,
      capabilities: input.targetCapabilities,
      enhancementRatio: input.enhancementFactor,
      safetyLevel,
    };
  }

  /**
   * Determine enhancement level from ratio
   */
  private determineEnhancementLevel(ratio: number): EnhancementLevel {
    if (ratio < ENHANCEMENT_THRESHOLDS.MINIMAL_MAX) {
      return 'MINIMAL';
    } else if (ratio < ENHANCEMENT_THRESHOLDS.MODERATE_MAX) {
      return 'MODERATE';
    } else if (ratio < ENHANCEMENT_THRESHOLDS.SIGNIFICANT_MAX) {
      return 'SIGNIFICANT';
    } else {
      return 'TRANSFORMATIVE';
    }
  }

  /**
   * Calculate recommended safety level
   */
  private calculateSafetyLevel(
    mode: IntegrationMode,
    depth: number,
    enhancement: number
  ): string {
    // Based on WIA-AUG-013 safety classification
    let score = 0;

    // Integration mode contribution
    if (mode === 'EXTERNAL') score += 1;
    else if (mode === 'SEMI_INVASIVE') score += 3;
    else if (mode === 'FULLY_INVASIVE') score += 5;

    // Depth contribution
    score += depth * 0.3;

    // Enhancement contribution
    if (enhancement >= 10) score += 2;
    else if (enhancement >= 5) score += 1;

    // Map to safety levels
    if (score <= 2) return 'Level1';
    if (score <= 4) return 'Level2';
    if (score <= 6) return 'Level3';
    if (score <= 8) return 'Level4';
    return 'Level5';
  }

  // ==========================================================================
  // Baseline Registry Methods
  // ==========================================================================

  /**
   * Register baseline measurements for a subject
   */
  registerBaseline(subject: SubjectInfo, measurements: BaselineMeasurement): BaselineRecord {
    const registryId = `BR-${Date.now()}-${subject.subjectId}`;

    const record: BaselineRecord = {
      registryId,
      subject,
      registrationDate: new Date(),
      baseline: measurements,
      updates: [],
    };

    // Calculate population percentiles (simplified - would use actual population data)
    record.populationPercentile = this.calculatePercentiles(measurements);

    this.baselineRegistry.set(registryId, record);

    return record;
  }

  /**
   * Get baseline record by ID
   */
  getBaseline(registryId: string): BaselineRecord | undefined {
    return this.baselineRegistry.get(registryId);
  }

  /**
   * Update baseline measurements
   */
  updateBaseline(
    registryId: string,
    measurements: BaselineMeasurement,
    reason: BaselineUpdate['reason']
  ): BaselineRecord {
    const record = this.baselineRegistry.get(registryId);
    if (!record) {
      throw new AugmentationError(
        AugmentationErrorCode.BASELINE_NOT_FOUND,
        `Baseline record ${registryId} not found`
      );
    }

    const updateId = `BU-${Date.now()}`;

    // Calculate changes
    const changes = this.calculateBaselineChanges(record.baseline, measurements);

    const update: BaselineUpdate = {
      updateId,
      updateDate: new Date(),
      reason,
      measurements,
      changes,
    };

    record.updates = record.updates || [];
    record.updates.push(update);
    record.baseline = measurements; // Update current baseline

    return record;
  }

  /**
   * Calculate population percentiles (simplified)
   */
  private calculatePercentiles(measurements: BaselineMeasurement): Record<string, number> {
    const percentiles: Record<string, number> = {};

    // Simplified - would use actual population distribution data
    if (measurements.physical?.strength) {
      percentiles.strength = 50; // Default to median
    }
    if (measurements.physical?.speed) {
      percentiles.speed = 50;
    }
    if (measurements.sensory?.visualAcuity) {
      percentiles.visualAcuity = 50;
    }

    return percentiles;
  }

  /**
   * Calculate changes between baselines
   */
  private calculateBaselineChanges(
    previous: BaselineMeasurement,
    current: BaselineMeasurement
  ): Record<string, { previous: number; current: number; delta: number }> {
    const changes: Record<string, { previous: number; current: number; delta: number }> = {};

    // Compare physical metrics
    if (previous.physical?.strength && current.physical?.strength) {
      const prev = previous.physical.strength.value;
      const curr = current.physical.strength.value;
      changes.strength = { previous: prev, current: curr, delta: curr - prev };
    }

    // Add more comparisons as needed...

    return changes;
  }

  // ==========================================================================
  // Enhancement Ratio Methods
  // ==========================================================================

  /**
   * Calculate enhancement ratio
   */
  calculateEnhancementRatio(input: EnhancementRatioInput): EnhancementResult {
    const { baselineValue, augmentedValue, metric, unit, lowerIsBetter } = input;

    if (baselineValue <= 0) {
      throw new AugmentationError(
        AugmentationErrorCode.ENHANCEMENT_CALCULATION_FAILED,
        'Baseline value must be greater than 0'
      );
    }

    let ratio: number;
    let percentageImprovement: number;

    if (lowerIsBetter) {
      // For metrics where lower is better (e.g., reaction time)
      ratio = baselineValue / augmentedValue;
      percentageImprovement = ((baselineValue - augmentedValue) / baselineValue) * 100;
    } else {
      // For metrics where higher is better (e.g., strength)
      ratio = augmentedValue / baselineValue;
      percentageImprovement = ((augmentedValue - baselineValue) / baselineValue) * 100;
    }

    const level = this.determineEnhancementLevel(ratio);

    return {
      metric,
      baseline: baselineValue,
      augmented: augmentedValue,
      ratio,
      level,
      percentageImprovement,
    };
  }

  /**
   * Calculate multi-metric enhancement score
   */
  calculateMultiMetricEnhancement(
    enhancements: EnhancementRatioInput[],
    weights?: Record<string, number>
  ): MultiMetricEnhancement {
    const results: EnhancementResult[] = enhancements.map((input) =>
      this.calculateEnhancementRatio(input)
    );

    // Use provided weights or equal weights
    const metricWeights = weights || {};
    const defaultWeight = 1.0 / enhancements.length;

    let weightedSum = 0;
    let totalWeight = 0;

    for (const result of results) {
      const weight = metricWeights[result.metric] || defaultWeight;
      weightedSum += result.ratio * weight;
      totalWeight += weight;
    }

    const weightedRatio = weightedSum / totalWeight;
    const overallLevel = this.determineEnhancementLevel(weightedRatio);

    return {
      metrics: results,
      overallScore: weightedRatio,
      weightedRatio,
      overallLevel,
      weights: metricWeights,
    };
  }

  // ==========================================================================
  // Compatibility Assessment Methods
  // ==========================================================================

  /**
   * Assess compatibility between augmentations
   */
  assessCompatibility(augmentations: AugmentationInfo[]): CompatibilityResult {
    if (augmentations.length < 2) {
      throw new AugmentationError(
        AugmentationErrorCode.COMPATIBILITY_ASSESSMENT_FAILED,
        'At least 2 augmentations required for compatibility assessment'
      );
    }

    // For simplicity, assess pairwise compatibility
    const aug1 = augmentations[0];
    const aug2 = augmentations[1];

    const technicalInterface = this.assessTechnicalInterface(aug1, aug2);
    const safetyInteraction = this.assessSafetyInteraction(aug1, aug2);
    const performanceSynergy = this.assessPerformanceSynergy(aug1, aug2);

    // Calculate overall compatibility score
    const compatibilityScore =
      technicalInterface.score * COMPATIBILITY_WEIGHTS.TECHNICAL_INTERFACE +
      safetyInteraction.score * COMPATIBILITY_WEIGHTS.SAFETY_INTERACTION +
      performanceSynergy.score * COMPATIBILITY_WEIGHTS.PERFORMANCE_SYNERGY;

    // Determine compatibility level
    let level: CompatibilityLevel;
    if (compatibilityScore >= COMPATIBILITY_THRESHOLDS.HIGHLY_COMPATIBLE) {
      level = 'HIGHLY_COMPATIBLE';
    } else if (compatibilityScore >= COMPATIBILITY_THRESHOLDS.COMPATIBLE) {
      level = 'COMPATIBLE';
    } else if (compatibilityScore >= COMPATIBILITY_THRESHOLDS.CONDITIONAL) {
      level = 'CONDITIONAL';
    } else {
      level = 'INCOMPATIBLE';
    }

    const compatible = level !== 'INCOMPATIBLE';

    // Generate warnings and recommendations
    const warnings: string[] = [];
    const recommendations: string[] = [];

    if (technicalInterface.score < 0.7) {
      warnings.push('Technical interface compatibility is below optimal');
      recommendations.push('Review power and communication protocols');
    }

    if (safetyInteraction.score < 0.8) {
      warnings.push('Safety interaction requires attention');
      recommendations.push('Conduct thorough safety assessment per WIA-AUG-013');
    }

    if (performanceSynergy.score < 0.5) {
      warnings.push('Limited performance synergy detected');
      recommendations.push('Consider optimization for cooperative operation');
    }

    return {
      augmentations: augmentations.map((a) => a.id),
      technicalInterface,
      safetyInteraction,
      performanceSynergy,
      compatibilityScore,
      level,
      compatible,
      warnings,
      recommendations,
    };
  }

  /**
   * Assess technical interface compatibility
   */
  private assessTechnicalInterface(aug1: AugmentationInfo, aug2: AugmentationInfo): TechnicalInterface {
    // Power compatibility
    const powerCompatibility = this.checkPowerCompatibility(aug1.power, aug2.power);

    // Communication protocol compatibility
    const communicationProtocol = this.checkCommunicationProtocol(aug1.protocol, aug2.protocol);

    // Physical interference
    const physicalInterference = this.checkPhysicalInterference(aug1.location, aug2.location);

    // Data format alignment
    const dataFormatAlignment = this.checkDataFormat(aug1.dataFormat, aug2.dataFormat);

    const score = (powerCompatibility + communicationProtocol + physicalInterference + dataFormatAlignment) / 4;

    return {
      powerCompatibility,
      communicationProtocol,
      physicalInterference,
      dataFormatAlignment,
      score,
    };
  }

  /**
   * Assess safety interaction
   */
  private assessSafetyInteraction(aug1: AugmentationInfo, aug2: AugmentationInfo): SafetyInteraction {
    // Biological conflict (simplified)
    const biologicalConflict = this.checkBiologicalConflict(aug1.integrationMode, aug2.integrationMode);

    // Electrical interference
    const electricalInterference = this.checkElectricalInterference(aug1.signals, aug2.signals);

    // Thermal interaction
    const thermalInteraction = this.checkThermalInteraction(aug1.heat, aug2.heat);

    // Mechanical stress
    const mechanicalStress = this.checkMechanicalStress(aug1.forces, aug2.forces);

    const score = (biologicalConflict + electricalInterference + thermalInteraction + mechanicalStress) / 4;

    return {
      biologicalConflict,
      electricalInterference,
      thermalInteraction,
      mechanicalStress,
      score,
    };
  }

  /**
   * Assess performance synergy
   */
  private assessPerformanceSynergy(aug1: AugmentationInfo, aug2: AugmentationInfo): PerformanceSynergy {
    // Cooperative effect
    const cooperativeEffect = this.calculateCooperativeEffect(aug1.type, aug2.type);

    // Resource sharing
    const resourceSharing = this.assessResourceSharing(aug1.resources, aug2.resources);

    // Functional complementarity
    const functionalComplementarity = this.assessComplementarity(aug1.capabilities, aug2.capabilities);

    // Normalize cooperative effect from [-1,1] to [0,1]
    const normalizedCoop = (cooperativeEffect + 1) / 2;

    const score = (normalizedCoop + resourceSharing + functionalComplementarity) / 3;

    return {
      cooperativeEffect,
      resourceSharing,
      functionalComplementarity,
      score,
    };
  }

  // Helper methods for compatibility checks (simplified implementations)

  private checkPowerCompatibility(power1?: any, power2?: any): number {
    if (!power1 || !power2) return 0.8; // Assume compatible if not specified
    // Simplified - would check voltage/current compatibility
    return 0.9;
  }

  private checkCommunicationProtocol(proto1?: string, proto2?: string): number {
    if (!proto1 || !proto2) return 0.7;
    return proto1 === proto2 ? 1.0 : 0.5;
  }

  private checkPhysicalInterference(loc1?: string, loc2?: string): number {
    if (!loc1 || !loc2) return 0.9;
    return loc1 === loc2 ? 0.3 : 1.0; // Low score if same location
  }

  private checkDataFormat(fmt1?: string, fmt2?: string): number {
    if (!fmt1 || !fmt2) return 0.8;
    return fmt1 === fmt2 ? 1.0 : 0.6;
  }

  private checkBiologicalConflict(mode1: IntegrationMode, mode2: IntegrationMode): number {
    // More invasive = higher potential for conflict
    if (mode1 === 'FULLY_INVASIVE' && mode2 === 'FULLY_INVASIVE') return 0.6;
    if (mode1 === 'FULLY_INVASIVE' || mode2 === 'FULLY_INVASIVE') return 0.8;
    return 0.95;
  }

  private checkElectricalInterference(sig1?: any, sig2?: any): number {
    if (!sig1 || !sig2) return 0.9;
    // Simplified - would check frequency overlap
    return 0.85;
  }

  private checkThermalInteraction(heat1?: any, heat2?: any): number {
    if (!heat1 || !heat2) return 0.95;
    // Check combined thermal load
    const totalDissipation = (heat1.dissipation || 0) + (heat2.dissipation || 0);
    return totalDissipation < 20 ? 0.95 : 0.7; // Simplified threshold
  }

  private checkMechanicalStress(force1?: any, force2?: any): number {
    if (!force1 || !force2) return 0.9;
    // Simplified stress check
    return 0.85;
  }

  private calculateCooperativeEffect(type1: AugmentationType, type2: AugmentationType): number {
    // Cooperation matrix (simplified)
    const cooperationMatrix: Record<string, Record<string, number>> = {
      PHYSICAL: { PHYSICAL: 0.5, SENSORY: 0.8, COGNITIVE: 0.3, NEURAL: 0.2, HYBRID: 0.6 },
      SENSORY: { PHYSICAL: 0.8, SENSORY: 0.2, COGNITIVE: 0.7, NEURAL: 0.6, HYBRID: 0.7 },
      COGNITIVE: { PHYSICAL: 0.3, SENSORY: 0.7, COGNITIVE: -0.2, NEURAL: 0.8, HYBRID: 0.6 },
      NEURAL: { PHYSICAL: 0.2, SENSORY: 0.6, COGNITIVE: 0.8, NEURAL: -0.5, HYBRID: 0.5 },
      HYBRID: { PHYSICAL: 0.6, SENSORY: 0.7, COGNITIVE: 0.6, NEURAL: 0.5, HYBRID: 0.4 },
    };

    return cooperationMatrix[type1]?.[type2] ?? 0;
  }

  private assessResourceSharing(res1?: string[], res2?: string[]): number {
    if (!res1 || !res2) return 0.5;
    const shared = res1.filter((r) => res2.includes(r)).length;
    const total = new Set([...res1, ...res2]).size;
    return shared > 0 ? 0.3 : 0.9; // Sharing resources may cause conflict
  }

  private assessComplementarity(cap1: string[], cap2: string[]): number {
    const overlap = cap1.filter((c) => cap2.includes(c)).length;
    const total = new Set([...cap1, ...cap2]).size;
    return 1 - overlap / total; // Less overlap = more complementary
  }

  // ==========================================================================
  // Performance Evaluation Methods
  // ==========================================================================

  /**
   * Evaluate augmentation performance
   */
  evaluatePerformance(
    augmentationId: string,
    testData: {
      effectiveness: number;
      reliability: number;
      efficiency: number;
      usability: number;
      enhancementRatio: number;
      testProtocol: string;
      performanceData?: any;
    }
  ): PerformanceEvaluation {
    const evaluationId = `PE-${Date.now()}`;

    const metrics: PerformanceMetrics = {
      effectiveness: testData.effectiveness,
      reliability: testData.reliability,
      efficiency: testData.efficiency,
      usability: testData.usability,
    };

    const level = this.determineEnhancementLevel(testData.enhancementRatio);

    // Simplified performance characteristics
    const performance: PerformanceCharacteristics = {
      peakPerformance: testData.enhancementRatio,
      sustainedPerformance: testData.enhancementRatio * 0.85,
      recoveryTime: 60, // seconds
      adaptationPeriod: 7, // days
    };

    // Check if performance meets minimum requirements
    const passed =
      metrics.effectiveness >= MINIMUM_PERFORMANCE.EFFECTIVENESS &&
      metrics.reliability >= MINIMUM_PERFORMANCE.RELIABILITY &&
      metrics.efficiency >= MINIMUM_PERFORMANCE.EFFICIENCY &&
      metrics.usability >= MINIMUM_PERFORMANCE.USABILITY;

    const recommendations: string[] = [];
    if (metrics.effectiveness < MINIMUM_PERFORMANCE.EFFECTIVENESS) {
      recommendations.push('Improve effectiveness to meet minimum threshold');
    }
    if (metrics.reliability < MINIMUM_PERFORMANCE.RELIABILITY) {
      recommendations.push('Enhance reliability through improved design');
    }
    if (metrics.efficiency < MINIMUM_PERFORMANCE.EFFICIENCY) {
      recommendations.push('Optimize efficiency to reduce resource consumption');
    }
    if (metrics.usability < MINIMUM_PERFORMANCE.USABILITY) {
      recommendations.push('Improve user experience and control mechanisms');
    }

    return {
      evaluationId,
      augmentationId,
      testDate: new Date(),
      testProtocol: testData.testProtocol,
      metrics,
      enhancementRatio: testData.enhancementRatio,
      level,
      performance,
      testData: testData.performanceData,
      passed,
      recommendations,
    };
  }

  // ==========================================================================
  // Validation Methods
  // ==========================================================================

  private validateClassificationInput(input: ClassificationInput): void {
    if (!input.primaryDomain) {
      throw new AugmentationError(
        AugmentationErrorCode.CLASSIFICATION_INVALID_INPUT,
        'Primary domain is required'
      );
    }

    if (!input.targetCapabilities || input.targetCapabilities.length === 0) {
      throw new AugmentationError(
        AugmentationErrorCode.CLASSIFICATION_INVALID_INPUT,
        'At least one target capability is required'
      );
    }

    if (input.integrationDepth < 1 || input.integrationDepth > 10) {
      throw new AugmentationError(
        AugmentationErrorCode.CLASSIFICATION_INVALID_INPUT,
        'Integration depth must be between 1 and 10'
      );
    }

    if (input.enhancementFactor < 1) {
      throw new AugmentationError(
        AugmentationErrorCode.CLASSIFICATION_INVALID_INPUT,
        'Enhancement factor must be at least 1.0'
      );
    }
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Classify augmentation (standalone)
 */
export function classifyAugmentation(input: ClassificationInput): ClassificationResult {
  const sdk = new HumanAugmentationSDK();
  return sdk.classifyAugmentation(input);
}

/**
 * Calculate enhancement ratio (standalone)
 */
export function calculateEnhancementRatio(input: EnhancementRatioInput): EnhancementResult {
  const sdk = new HumanAugmentationSDK();
  return sdk.calculateEnhancementRatio(input);
}

/**
 * Register baseline (standalone)
 */
export function registerBaseline(subject: SubjectInfo, measurements: BaselineMeasurement): BaselineRecord {
  const sdk = new HumanAugmentationSDK();
  return sdk.registerBaseline(subject, measurements);
}

/**
 * Assess compatibility (standalone)
 */
export function assessCompatibility(augmentations: AugmentationInfo[]): CompatibilityResult {
  const sdk = new HumanAugmentationSDK();
  return sdk.assessCompatibility(augmentations);
}

/**
 * Evaluate performance (standalone)
 */
export function evaluatePerformance(
  augmentationId: string,
  testData: any
): PerformanceEvaluation {
  const sdk = new HumanAugmentationSDK();
  return sdk.evaluatePerformance(augmentationId, testData);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { HumanAugmentationSDK };
export default HumanAugmentationSDK;
