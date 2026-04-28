/**
 * WIA-AUG-004: Sensory Enhancement SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Human Augmentation Sensory Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides tools for sensory enhancement including:
 * - Sensory modality classification
 * - Enhancement spectrum analysis
 * - Multi-sensory integration
 * - Sensory substitution
 * - Perception calibration
 * - Overload protection
 * - Cross-modal mapping
 */

import {
  SensoryModality,
  SensoryRange,
  EnhancementType,
  EnhancementLevel,
  EnhancementInput,
  EnhancementClassification,
  EnhancementParams,
  EnhancementResult,
  IntegrationMode,
  SensoryInput,
  IntegratedPercept,
  CalibrationParams,
  CalibrationResult,
  OverloadMonitor,
  ProtectionStatus,
  CrossModalMap,
  MappedData,
  SensoryData,
  SENSORY_CONSTANTS,
  SensoryErrorCode,
  SensoryError,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-AUG-004 Sensory Enhancement SDK
 */
export class SensoryEnhancementSDK {
  private version = '1.0.0';

  constructor() {}

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  /**
   * Classify sensory enhancement
   */
  classifySensory(input: EnhancementInput): EnhancementClassification {
    // Validate input
    this.validateEnhancementInput(input);

    // Calculate enhancement factor
    const factor = this.calculateEnhancementFactor(
      input.baselineRange,
      input.targetRange
    );

    // Determine enhancement type
    const type = this.determineEnhancementType(input.baselineRange, factor);

    // Determine enhancement level
    const level = this.determineEnhancementLevel(factor);

    // Calculate safety metrics
    const safetyScore = this.calculateSafetyScore(
      factor,
      input.modality,
      input.safetyMargin || 0.9
    );

    const neuralCompatibility = this.calculateNeuralCompatibility(
      input.modality,
      factor
    );

    const reversibility = this.calculateReversibility(type, factor);

    return {
      type,
      level,
      factor,
      safetyScore,
      neuralCompatibility,
      reversibility,
    };
  }

  /**
   * Enhance a sensory modality
   */
  enhanceModality(params: EnhancementParams): EnhancementResult {
    // Validate parameters
    this.validateEnhancementParams(params);

    // Calculate new range
    const newRange = this.calculateEnhancedRange(
      params.baseRange,
      params.enhancementFactor,
      params.safetyMargin
    );

    // Determine enhancement type
    const enhancementType = this.determineEnhancementType(
      params.baseRange,
      params.enhancementFactor
    );

    // Calculate safety score
    const safetyScore = this.calculateSafetyScore(
      params.enhancementFactor,
      params.modality,
      params.safetyMargin
    );

    // Estimate adaptation time
    const adaptationTime = this.estimateAdaptationTime(
      enhancementType,
      params.enhancementFactor,
      params.adaptationPeriod
    );

    return {
      modality: params.modality,
      enhancementType,
      originalRange: params.baseRange,
      newRange,
      factor: params.enhancementFactor,
      safetyScore,
      adaptationTime,
    };
  }

  /**
   * Integrate multiple sensory inputs
   */
  integrateMultiSensory(
    inputs: SensoryInput[],
    mode: IntegrationMode = IntegrationMode.SYNERGISTIC
  ): IntegratedPercept {
    // Validate inputs
    if (inputs.length < 2) {
      throw new SensoryError(
        SensoryErrorCode.INVALID_MODALITY,
        'At least 2 sensory inputs required for integration'
      );
    }

    // Synchronize timestamps
    const synced = this.synchronizeInputs(inputs);

    // Check synchronization quality
    const synchronization = this.calculateSynchronization(synced);

    // Weight inputs by priority and reliability
    const weighted = synced.map((input) => ({
      ...input,
      weight: input.priority * input.reliability,
    }));

    // Integrate based on mode
    const integrated = this.performIntegration(weighted, mode);

    // Calculate quality metrics
    const quality = this.calculateIntegrationQuality(synced, integrated);
    const fidelity = this.calculateFidelity(synced, integrated);
    const latency = this.calculateLatency(synced);

    return {
      modalities: inputs.map((i) => i.modality),
      mode,
      data: integrated,
      quality,
      synchronization,
      fidelity,
      latency,
    };
  }

  /**
   * Calibrate sensory perception
   */
  calibratePerception(params: CalibrationParams): CalibrationResult {
    // Validate parameters
    this.validateCalibrationParams(params);

    // Perform calibration
    const accuracy = this.performCalibration(params);

    // Calculate drift
    const drift = this.calculateDrift(params);

    // Determine if calibration is successful
    const isCalibrated = accuracy >= 0.95 && drift <= 5.0;

    // Calculate next calibration date
    const nextCalibration = this.calculateNextCalibration(
      params.modality,
      params.sensitivity
    );

    return {
      modality: params.modality,
      isCalibrated,
      accuracy,
      drift,
      timestamp: new Date(),
      nextCalibration,
      notes: isCalibrated
        ? 'Calibration successful'
        : 'Calibration requires adjustment',
    };
  }

  /**
   * Prevent sensory overload
   */
  preventOverload(monitor: OverloadMonitor): ProtectionStatus {
    // Calculate risk
    const riskPercentage = this.calculateOverloadRisk(monitor);

    // Determine risk level
    const riskLevel = this.determineRiskLevel(riskPercentage);

    // Determine actions
    const actionsTaken: string[] = [];
    const recommendations: string[] = [];

    if (riskLevel === 'danger') {
      actionsTaken.push('Emergency shutoff activated');
      recommendations.push('Immediate rest required');
    } else if (riskLevel === 'critical') {
      actionsTaken.push('Auto-limiting engaged', 'Gradual reduction started');
      recommendations.push('Reduce intensity', 'Take break within 5 minutes');
    } else if (riskLevel === 'warning') {
      actionsTaken.push('Warning threshold reached');
      recommendations.push('Monitor closely', 'Consider reducing intensity');
    } else {
      recommendations.push('Continue normal operation');
    }

    // Calculate recovery progress
    const recoveryProgress = this.calculateRecoveryProgress(monitor);

    return {
      active: riskLevel !== 'safe',
      riskLevel,
      riskPercentage,
      actionsTaken,
      recoveryProgress,
      recommendations,
    };
  }

  /**
   * Map data across sensory modalities
   */
  mapCrossModal(
    data: SensoryData,
    sourceModality: SensoryModality,
    targetModality: SensoryModality
  ): MappedData {
    // Validate modalities
    this.validateCrossModalMapping(sourceModality, targetModality);

    // Get mapping function
    const mappingFunction = this.getCrossModalMapping(
      sourceModality,
      targetModality
    );

    // Perform mapping
    const mappedData = this.performCrossModalMapping(data, mappingFunction);

    // Calculate quality
    const quality = this.calculateMappingQuality(data, mappedData);
    const informationPreserved = this.calculateInformationPreserved(
      data,
      mappedData
    );

    return {
      originalModality: sourceModality,
      targetModality,
      originalData: data,
      mappedData,
      quality,
      informationPreserved,
    };
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  private validateEnhancementInput(input: EnhancementInput): void {
    if (!input.baselineRange || !input.targetRange) {
      throw new SensoryError(
        SensoryErrorCode.INVALID_RANGE,
        'Baseline and target ranges are required'
      );
    }

    if (input.safetyMargin && (input.safetyMargin < 0.8 || input.safetyMargin > 0.95)) {
      throw new SensoryError(
        SensoryErrorCode.INVALID_RANGE,
        'Safety margin must be between 0.8 and 0.95'
      );
    }
  }

  private validateEnhancementParams(params: EnhancementParams): void {
    if (params.enhancementFactor < 1.0 || params.enhancementFactor > 10.0) {
      throw new SensoryError(
        SensoryErrorCode.INVALID_RANGE,
        'Enhancement factor must be between 1.0 and 10.0'
      );
    }

    if (params.safetyMargin < 0.8 || params.safetyMargin > 0.95) {
      throw new SensoryError(
        SensoryErrorCode.INVALID_RANGE,
        'Safety margin must be between 0.8 and 0.95'
      );
    }

    // Check max safe enhancement for modality
    const maxSafe = SENSORY_CONSTANTS.MAX_ENHANCEMENT[params.modality] || 5.0;
    if (params.enhancementFactor > maxSafe) {
      throw new SensoryError(
        SensoryErrorCode.ENHANCEMENT_UNSAFE,
        `Enhancement factor ${params.enhancementFactor} exceeds safe limit ${maxSafe} for ${params.modality}`
      );
    }
  }

  private validateCalibrationParams(params: CalibrationParams): void {
    if (params.sensitivity < 0 || params.sensitivity > 1) {
      throw new SensoryError(
        SensoryErrorCode.INVALID_RANGE,
        'Sensitivity must be between 0 and 1'
      );
    }

    if (params.adaptationRate < 0 || params.adaptationRate > 1) {
      throw new SensoryError(
        SensoryErrorCode.INVALID_RANGE,
        'Adaptation rate must be between 0 and 1'
      );
    }
  }

  private calculateEnhancementFactor(
    baseline: SensoryRange,
    target: SensoryRange
  ): number {
    const baselineSpan = baseline.max - baseline.min;
    const targetSpan = target.max - target.min;

    if (baselineSpan === 0) return 0;

    return targetSpan / baselineSpan;
  }

  private determineEnhancementType(
    baseline: SensoryRange,
    factor: number
  ): EnhancementType {
    if (baseline.min === 0 && baseline.max === 0) {
      return EnhancementType.NEW_SENSE;
    } else if (factor < 1.0) {
      return EnhancementType.RESTORATION;
    } else {
      return EnhancementType.AUGMENTATION;
    }
  }

  private determineEnhancementLevel(factor: number): EnhancementLevel {
    if (factor <= 1.25) return 'Level1';
    if (factor <= 2.0) return 'Level2';
    if (factor <= 5.0) return 'Level3';
    if (factor <= 10.0) return 'Level4';
    return 'Level5';
  }

  private calculateSafetyScore(
    factor: number,
    modality: SensoryModality,
    safetyMargin: number
  ): number {
    const maxSafe = SENSORY_CONSTANTS.MAX_ENHANCEMENT[modality] || 5.0;
    const safeFactor = Math.min(factor / maxSafe, 1.0);
    const score = (1 - safeFactor) * safetyMargin;
    return Math.max(0, Math.min(1, score));
  }

  private calculateNeuralCompatibility(
    modality: SensoryModality,
    factor: number
  ): number {
    // Higher enhancement = lower neural compatibility
    const base = 1.0 - (factor - 1.0) / 10.0;

    // Some modalities adapt better than others
    const modalityModifier: Record<string, number> = {
      [SensoryModality.AUDITORY]: 1.1,
      [SensoryModality.TACTILE]: 1.0,
      [SensoryModality.VISUAL]: 0.9,
      [SensoryModality.PROPRIOCEPTIVE]: 1.2,
    };

    const modifier = modalityModifier[modality] || 1.0;
    return Math.max(0, Math.min(1, base * modifier));
  }

  private calculateReversibility(type: EnhancementType, factor: number): number {
    // Restoration is highly reversible
    if (type === EnhancementType.RESTORATION) return 0.95;

    // New senses are less reversible (neural plasticity)
    if (type === EnhancementType.NEW_SENSE) return 0.5;

    // Augmentation reversibility decreases with factor
    return Math.max(0.6, 1.0 - (factor - 1.0) / 20.0);
  }

  private calculateEnhancedRange(
    baseRange: SensoryRange,
    factor: number,
    safetyMargin: number
  ): SensoryRange {
    const span = baseRange.max - baseRange.min;
    const enhancement = span * (factor - 1.0) * safetyMargin;

    return {
      min: baseRange.min - enhancement / 2,
      max: baseRange.max + enhancement / 2,
      resolution: baseRange.resolution / factor,
      unit: baseRange.unit,
      frequency: baseRange.frequency,
    };
  }

  private estimateAdaptationTime(
    type: EnhancementType,
    factor: number,
    customPeriod?: number
  ): number {
    if (customPeriod) return customPeriod * 24; // days to hours

    const baseTime = SENSORY_CONSTANTS.ADAPTATION_TIME[type] || 50;
    const factorMultiplier = 1 + (factor - 1.0) * 0.5;

    return baseTime * factorMultiplier;
  }

  private synchronizeInputs(inputs: SensoryInput[]): SensoryInput[] {
    // Find earliest timestamp
    const minTimestamp = Math.min(...inputs.map((i) => i.timestamp));

    // Normalize all timestamps relative to earliest
    return inputs.map((input) => ({
      ...input,
      timestamp: input.timestamp - minTimestamp,
    }));
  }

  private calculateSynchronization(inputs: SensoryInput[]): number {
    if (inputs.length < 2) return 1.0;

    const timestamps = inputs.map((i) => i.timestamp);
    const maxDiff = Math.max(...timestamps) - Math.min(...timestamps);

    // Convert to milliseconds and calculate sync score
    const maxDiffMs = maxDiff / 1000;
    const maxAllowed = 100; // ms

    return Math.max(0, 1.0 - maxDiffMs / maxAllowed);
  }

  private performIntegration(
    inputs: Array<SensoryInput & { weight: number }>,
    mode: IntegrationMode
  ): SensoryData {
    switch (mode) {
      case IntegrationMode.ADDITIVE:
        return this.integrateAdditive(inputs);

      case IntegrationMode.DOMINANT:
        return this.integrateDominant(inputs);

      case IntegrationMode.SYNERGISTIC:
        return this.integrateSynergistic(inputs);

      case IntegrationMode.COMPLEMENTARY:
        return this.integrateComplementary(inputs);

      default:
        return this.integrateAdditive(inputs);
    }
  }

  private integrateAdditive(
    inputs: Array<SensoryInput & { weight: number }>
  ): SensoryData {
    // Sum weighted values
    const maxLength = Math.max(...inputs.map((i) => i.data.values.length));
    const values: number[] = new Array(maxLength).fill(0);

    inputs.forEach((input) => {
      input.data.values.forEach((val, idx) => {
        values[idx] = (values[idx] || 0) + val * input.weight;
      });
    });

    return {
      type: 'integrated_additive',
      values,
      quality: this.averageQuality(inputs),
    };
  }

  private integrateDominant(
    inputs: Array<SensoryInput & { weight: number }>
  ): SensoryData {
    // Select input with highest weight
    const dominant = inputs.reduce((prev, curr) =>
      curr.weight > prev.weight ? curr : prev
    );

    return {
      type: 'integrated_dominant',
      values: dominant.data.values,
      quality: dominant.data.quality,
    };
  }

  private integrateSynergistic(
    inputs: Array<SensoryInput & { weight: number }>
  ): SensoryData {
    // Combine inputs with enhancement
    const combined = this.integrateAdditive(inputs);

    // Apply synergy boost
    const synergyBoost = 1.2;
    combined.values = combined.values.map((v) => v * synergyBoost);
    combined.quality = Math.min(1.0, (combined.quality || 0) * synergyBoost);
    combined.type = 'integrated_synergistic';

    return combined;
  }

  private integrateComplementary(
    inputs: Array<SensoryInput & { weight: number }>
  ): SensoryData {
    // Fill gaps between inputs
    const maxLength = Math.max(...inputs.map((i) => i.data.values.length));
    const values: number[] = new Array(maxLength).fill(0);
    const counts: number[] = new Array(maxLength).fill(0);

    inputs.forEach((input) => {
      input.data.values.forEach((val, idx) => {
        if (val > 0) {
          values[idx] += val * input.weight;
          counts[idx]++;
        }
      });
    });

    // Average where overlapping
    for (let i = 0; i < maxLength; i++) {
      if (counts[i] > 0) {
        values[i] /= counts[i];
      }
    }

    return {
      type: 'integrated_complementary',
      values,
      quality: this.averageQuality(inputs),
    };
  }

  private averageQuality(inputs: Array<SensoryInput & { weight: number }>): number {
    const totalWeight = inputs.reduce((sum, i) => sum + i.weight, 0);
    if (totalWeight === 0) return 0;

    return (
      inputs.reduce((sum, i) => sum + i.data.quality * i.weight, 0) / totalWeight
    );
  }

  private calculateIntegrationQuality(
    inputs: SensoryInput[],
    integrated: SensoryData
  ): number {
    const avgQuality = inputs.reduce((sum, i) => sum + i.data.quality, 0) / inputs.length;
    return Math.min(1.0, avgQuality * (integrated.quality || 1.0));
  }

  private calculateFidelity(inputs: SensoryInput[], integrated: SensoryData): number {
    // Simplified fidelity: ratio of integrated to sum of inputs
    const totalInput = inputs.reduce(
      (sum, i) => sum + i.data.values.reduce((s, v) => s + Math.abs(v), 0),
      0
    );
    const totalOutput = integrated.values.reduce((s, v) => s + Math.abs(v), 0);

    if (totalInput === 0) return 0;
    return Math.min(1.0, totalOutput / totalInput);
  }

  private calculateLatency(inputs: SensoryInput[]): number {
    // Calculate span of timestamps
    const timestamps = inputs.map((i) => i.timestamp);
    const span = Math.max(...timestamps) - Math.min(...timestamps);

    return span / 1000; // Convert to milliseconds
  }

  private performCalibration(params: CalibrationParams): number {
    // Simulate calibration accuracy based on parameters
    const baseAccuracy = 0.95;
    const sensitivityFactor = params.sensitivity * 0.05;
    const adaptationFactor = params.adaptationRate * 0.03;

    return Math.min(1.0, baseAccuracy + sensitivityFactor + adaptationFactor);
  }

  private calculateDrift(params: CalibrationParams): number {
    // Simulate drift calculation
    const baseDrift = 2.0; // %
    const sensitivityPenalty = (1 - params.sensitivity) * 3;

    return baseDrift + sensitivityPenalty;
  }

  private calculateNextCalibration(
    modality: SensoryModality,
    sensitivity: number
  ): Date {
    // Higher sensitivity requires more frequent calibration
    const baseDays = 30;
    const sensitivityFactor = 1 + (1 - sensitivity);
    const days = baseDays * sensitivityFactor;

    const next = new Date();
    next.setDate(next.getDate() + days);
    return next;
  }

  private calculateOverloadRisk(monitor: OverloadMonitor): number {
    // Risk from intensity
    const intensityRisk = (monitor.currentIntensity / monitor.maxIntensity) * 100;

    // Risk from duration
    const durationFactor = monitor.exposureDuration / monitor.safeExposureLimit;
    const durationRisk = Math.min(100, durationFactor * 50);

    // Combined risk
    return Math.min(100, intensityRisk + durationRisk);
  }

  private determineRiskLevel(
    riskPercentage: number
  ): 'safe' | 'warning' | 'critical' | 'danger' {
    if (riskPercentage >= SENSORY_CONSTANTS.OVERLOAD_THRESHOLDS.DANGER) {
      return 'danger';
    } else if (riskPercentage >= SENSORY_CONSTANTS.OVERLOAD_THRESHOLDS.CRITICAL) {
      return 'critical';
    } else if (riskPercentage >= SENSORY_CONSTANTS.OVERLOAD_THRESHOLDS.WARNING) {
      return 'warning';
    }
    return 'safe';
  }

  private calculateRecoveryProgress(monitor: OverloadMonitor): number {
    const currentRisk = this.calculateOverloadRisk(monitor);
    const normalizedRisk = currentRisk / 100;

    return Math.max(0, 1 - normalizedRisk);
  }

  private validateCrossModalMapping(
    source: SensoryModality,
    target: SensoryModality
  ): void {
    if (source === target) {
      throw new SensoryError(
        SensoryErrorCode.INCOMPATIBLE_MODALITIES,
        'Source and target modalities must be different'
      );
    }

    // Check compatibility (simplified)
    const incompatible = [
      [SensoryModality.OLFACTORY, SensoryModality.VESTIBULAR],
      [SensoryModality.GUSTATORY, SensoryModality.PROPRIOCEPTIVE],
    ];

    for (const [mod1, mod2] of incompatible) {
      if ((source === mod1 && target === mod2) || (source === mod2 && target === mod1)) {
        throw new SensoryError(
          SensoryErrorCode.INCOMPATIBLE_MODALITIES,
          `Incompatible modalities: ${source} and ${target}`
        );
      }
    }
  }

  private getCrossModalMapping(
    source: SensoryModality,
    target: SensoryModality
  ): any {
    // Return simplified mapping function
    return {
      type: 'linear',
      parameters: { scale: 1.0, offset: 0 },
    };
  }

  private performCrossModalMapping(data: SensoryData, mapping: any): SensoryData {
    // Apply linear mapping
    const scale = mapping.parameters.scale || 1.0;
    const offset = mapping.parameters.offset || 0;

    const mappedValues = data.values.map((v) => v * scale + offset);

    return {
      type: `mapped_${data.type}`,
      values: mappedValues,
      quality: data.quality * 0.9, // Slight quality loss in mapping
    };
  }

  private calculateMappingQuality(original: SensoryData, mapped: SensoryData): number {
    // Simplified quality calculation
    return Math.min(original.quality, mapped.quality);
  }

  private calculateInformationPreserved(
    original: SensoryData,
    mapped: SensoryData
  ): number {
    // Calculate correlation between original and mapped
    const origSum = original.values.reduce((s, v) => s + Math.abs(v), 0);
    const mappedSum = mapped.values.reduce((s, v) => s + Math.abs(v), 0);

    if (origSum === 0) return 0;

    return Math.min(1.0, mappedSum / origSum);
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Classify sensory enhancement (standalone)
 */
export function classifySensory(input: EnhancementInput): EnhancementClassification {
  const sdk = new SensoryEnhancementSDK();
  return sdk.classifySensory(input);
}

/**
 * Enhance modality (standalone)
 */
export function enhanceModality(params: EnhancementParams): EnhancementResult {
  const sdk = new SensoryEnhancementSDK();
  return sdk.enhanceModality(params);
}

/**
 * Calibrate perception (standalone)
 */
export function calibratePerception(params: CalibrationParams): CalibrationResult {
  const sdk = new SensoryEnhancementSDK();
  return sdk.calibratePerception(params);
}

/**
 * Prevent overload (standalone)
 */
export function preventOverload(monitor: OverloadMonitor): ProtectionStatus {
  const sdk = new SensoryEnhancementSDK();
  return sdk.preventOverload(monitor);
}

/**
 * Map cross-modal (standalone)
 */
export function mapCrossModal(
  data: SensoryData,
  sourceModality: SensoryModality,
  targetModality: SensoryModality
): MappedData {
  const sdk = new SensoryEnhancementSDK();
  return sdk.mapCrossModal(data, sourceModality, targetModality);
}

/**
 * Integrate multi-sensory (standalone)
 */
export function integrateMultiSensory(
  inputs: SensoryInput[],
  mode?: IntegrationMode
): IntegratedPercept {
  const sdk = new SensoryEnhancementSDK();
  return sdk.integrateMultiSensory(inputs, mode);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { SensoryEnhancementSDK };
export default SensoryEnhancementSDK;
