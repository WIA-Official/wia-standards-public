/**
 * WIA-AUG-003: Neural Enhancement SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Neural Enhancement Working Group
 *
 * 弘익人間 (Benefit All Humanity)
 *
 * This SDK provides tools for neural enhancement including:
 * - Neural interface classification
 * - Signal processing and feature extraction
 * - Neural pathway mapping
 * - Cognitive load management
 * - Neuroprotection validation
 * - BCI calibration
 */

import {
  NeuralInterfaceType,
  SignalType,
  EnhancementMode,
  InterfaceClassificationInput,
  InterfaceClassificationResult,
  SignalProcessingInput,
  ProcessedSignal,
  PathwayMappingInput,
  PathwayMappingResult,
  LoadManagementInput,
  LoadManagementResult,
  NeuroprotectionInput,
  NeuroprotectionResult,
  BCICalibrationInput,
  BCICalibrationResult,
  InterfaceAssessmentInput,
  InterfaceAssessmentResult,
  NeuralPathway,
  BrainRegion,
  ArtifactDetection,
  SignalFeatures,
  DecoderModel,
  CognitiveLoadLevel,
  NEURAL_CONSTANTS,
  NeuralErrorCode,
  NeuralError,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-AUG-003 Neural Enhancement SDK
 */
export class NeuralEnhancementSDK {
  private version = '1.0.0';

  constructor() {}

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  /**
   * Classify a neural interface based on specifications
   */
  classifyInterface(input: InterfaceClassificationInput): InterfaceClassificationResult {
    // Validate input
    this.validateInterfaceInput(input);

    // Calculate classification score
    const electrodeScore = Math.min(input.electrodeCount / 256, 1) * 30;
    const resolutionScore = this.getResolutionScore(input.resolution) * 30;
    const typeScore = this.getTypeScore(input.type) * 20;
    const coverageScore = this.getCoverageScore(input.coverage || 'unilateral') * 20;

    const score = electrodeScore + resolutionScore + typeScore + coverageScore;

    // Determine invasiveness
    const invasiveness = this.determineInvasiveness(input.type);

    // Generate applications
    const applications = this.generateApplications(input.type, input.location);

    // Generate safety requirements
    const safetyRequirements = this.generateSafetyRequirements(input.type, invasiveness);

    return {
      type: input.type,
      score,
      invasiveness,
      resolution: input.resolution,
      applications,
      safetyRequirements,
    };
  }

  /**
   * Process neural signal with filtering and feature extraction
   */
  processSignal(input: SignalProcessingInput): ProcessedSignal {
    // Validate signal input
    this.validateSignalInput(input);

    // Apply filtering
    const filteredData = this.applyFiltering(
      input.rawData,
      input.samplingRate,
      input.filterBand || [0.5, 200]
    );

    // Calculate signal quality
    const quality = this.calculateSignalQuality(filteredData, input.rawData);

    // Calculate SNR
    const snr = this.calculateSNR(filteredData, input.rawData);

    // Detect artifacts
    const artifacts = input.artifactRemoval
      ? this.detectArtifacts(filteredData, input.samplingRate)
      : [];

    // Extract features
    const features = this.extractFeatures(
      filteredData,
      input.samplingRate,
      input.signalType
    );

    return {
      filteredData,
      quality,
      snr,
      artifacts,
      features,
    };
  }

  /**
   * Map neural pathway between brain regions
   */
  mapPathway(input: PathwayMappingInput): PathwayMappingResult {
    // Create brain regions
    const source: BrainRegion = {
      name: input.sourceRegion,
      hemisphere: 'left',
    };

    const target: BrainRegion = {
      name: input.targetRegion,
      hemisphere: 'left',
    };

    // Create pathway
    const pathway: NeuralPathway = {
      id: `PWY-${Date.now()}`,
      source,
      target,
      intermediate: [],
      modality: this.inferModality(input.sourceRegion, input.targetRegion),
      function: `${input.sourceRegion}_to_${input.targetRegion}`,
      strength: 0.75, // Default strength
    };

    // Calculate confidence based on signal type
    const confidence = this.calculatePathwayConfidence(input.signalType, input.method || 'correlation');

    // Estimate latency
    const latency = this.estimatePathwayLatency(input.sourceRegion, input.targetRegion);

    // Calculate directional strength
    const directionalStrength = 0.8; // Placeholder

    // Generate visualization
    const visualization = {
      nodes: [source, target],
      edges: [
        {
          from: source.name,
          to: target.name,
          weight: pathway.strength,
        },
      ],
    };

    return {
      pathway,
      confidence,
      directionalStrength,
      latency,
      visualization,
    };
  }

  /**
   * Manage cognitive load and provide recommendations
   */
  manageLoad(input: LoadManagementInput): LoadManagementResult {
    // Calculate load score
    const loadScore = this.calculateLoadScore(input);

    // Determine load level
    const loadLevel = this.determineLoadLevel(loadScore);

    // Check if within safe limits
    const withinLimits = loadScore <= input.threshold;

    // Generate recommendations
    const recommendations = this.generateLoadRecommendations(loadLevel, withinLimits);

    // Determine required actions
    const actions = {
      reduceComplexity: loadScore > 0.8,
      increaseAssistance: loadScore > 0.7,
      addBreaks: loadScore > 0.6,
      simplifyFeedback: loadScore > 0.5,
    };

    return {
      loadLevel,
      loadScore,
      withinLimits,
      recommendations,
      actions,
    };
  }

  /**
   * Validate neural stimulation parameters for safety
   */
  protectNeural(input: NeuroprotectionInput): NeuroprotectionResult {
    // Calculate charge per phase
    const chargePerPhase = (input.stimulationCurrent * input.pulseWidth) / 1000; // μC

    // Calculate charge density
    const electrodeArea = input.electrodeArea || 1.0; // mm²
    const chargeDensity = (chargePerPhase * 1000) / electrodeArea; // μC/cm²

    // Get safe limit
    const safeLimit = NEURAL_CONSTANTS.STIMULATION.MAX_CHARGE_DENSITY;

    // Check safety
    const isSafe = chargeDensity <= safeLimit;

    // Calculate safety margin
    const safetyMargin = ((safeLimit - chargeDensity) / safeLimit) * 100;

    // Generate warnings
    const warnings: string[] = [];
    if (chargeDensity > safeLimit * 0.8) {
      warnings.push('Approaching charge density limit');
    }
    if (input.stimulationCurrent > NEURAL_CONSTANTS.STIMULATION.MAX_CURRENT * 0.8) {
      warnings.push('High stimulation current');
    }
    if (input.frequency > NEURAL_CONSTANTS.STIMULATION.MAX_FREQUENCY * 0.8) {
      warnings.push('High stimulation frequency');
    }

    // Generate modifications if unsafe
    const modifications: string[] = [];
    if (!isSafe) {
      const reductionFactor = safeLimit / chargeDensity;
      modifications.push(`Reduce current to ${(input.stimulationCurrent * reductionFactor).toFixed(2)} mA`);
      modifications.push(`Or reduce pulse width to ${(input.pulseWidth * reductionFactor).toFixed(0)} μs`);
      modifications.push(`Or increase electrode area to ${(electrodeArea / reductionFactor).toFixed(2)} mm²`);
    }

    return {
      isSafe,
      chargeDensity,
      chargePerPhase,
      safeLimit,
      safetyMargin,
      warnings,
      modifications,
    };
  }

  /**
   * Calibrate brain-computer interface
   */
  calibrateBCI(input: BCICalibrationInput): BCICalibrationResult {
    // Validate calibration input
    this.validateCalibrationInput(input);

    // Initialize variables
    let accuracy = 0;
    let iterations = 0;
    const maxIterations = input.maxIterations || 100;
    const convergenceHistory: number[] = [];

    // Simulate calibration process
    const featureDimensions = input.userFeedback[0]?.features.length || 10;
    const classes = [...new Set(input.userFeedback.map((d) => d.label))];

    // Iterative training
    while (iterations < maxIterations) {
      // Simulate training iteration
      accuracy = this.simulateTrainingIteration(
        input.userFeedback,
        input.adaptationRate,
        iterations
      );

      convergenceHistory.push(accuracy);
      iterations++;

      // Check convergence
      if (accuracy >= input.convergenceThreshold) {
        break;
      }
    }

    // Check success
    const success = accuracy >= input.convergenceThreshold;

    // Create decoder model
    const model: DecoderModel = {
      type: 'LDA',
      parameters: {
        adaptationRate: input.adaptationRate,
        featureDimensions,
      },
      featureDimensions,
      classes,
      version: this.version,
    };

    // Calculate confusion matrix (simulated)
    const confusionMatrix = this.generateConfusionMatrix(classes.length, accuracy);

    // Calculate metrics
    const metrics = this.calculateClassificationMetrics(confusionMatrix);

    return {
      success,
      accuracy,
      iterations,
      model,
      metrics,
      convergenceHistory,
    };
  }

  /**
   * Comprehensive interface assessment
   */
  assessInterface(input: InterfaceAssessmentInput): InterfaceAssessmentResult {
    // Classify interface
    const classification = this.classifyInterface(input.interface);

    // Predict signal quality
    const signalQuality = this.predictSignalQuality(
      input.signals.types,
      input.signals.samplingRate,
      input.interface.electrodeCount
    );

    // Calculate safety score
    const safetyScore = this.calculateSafetyScore(
      classification.invasiveness,
      input.enhancement.intensity
    );

    // Predict efficacy
    const efficacyPrediction = this.predictEfficacy(
      input.interface.type,
      input.enhancement.mode,
      input.enhancement.target
    );

    // Generate recommendations
    const recommendations = [
      ...classification.applications.slice(0, 3).map((app) => `Suitable for: ${app}`),
      `Expected signal quality: ${(signalQuality * 100).toFixed(0)}%`,
      `Safety score: ${safetyScore}/100`,
    ];

    // Required certifications
    const certifications = [
      'WIA-AUG-003',
      'WIA-AUG-013',
      ...classification.safetyRequirements,
    ];

    // Estimate calibration time
    const calibrationTime = this.estimateCalibrationTime(
      input.interface.electrodeCount,
      input.signals.types.length
    );

    return {
      classification,
      signalQuality,
      safetyScore,
      efficacyPrediction,
      recommendations,
      certifications,
      calibrationTime,
    };
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  private validateInterfaceInput(input: InterfaceClassificationInput): void {
    if (input.electrodeCount < 1) {
      throw new NeuralError(
        NeuralErrorCode.INTERFACE_ERROR,
        'Electrode count must be positive'
      );
    }
  }

  private validateSignalInput(input: SignalProcessingInput): void {
    if (input.samplingRate < NEURAL_CONSTANTS.SIGNAL.MIN_SAMPLING_RATE) {
      throw new NeuralError(
        NeuralErrorCode.INVALID_SIGNAL_TYPE,
        `Sampling rate must be at least ${NEURAL_CONSTANTS.SIGNAL.MIN_SAMPLING_RATE} Hz`
      );
    }

    if (input.rawData.length === 0) {
      throw new NeuralError(
        NeuralErrorCode.INVALID_SIGNAL_TYPE,
        'Signal data cannot be empty'
      );
    }
  }

  private validateCalibrationInput(input: BCICalibrationInput): void {
    if (input.userFeedback.length < NEURAL_CONSTANTS.CALIBRATION.MIN_TRIALS) {
      throw new NeuralError(
        NeuralErrorCode.CALIBRATION_FAILED,
        `At least ${NEURAL_CONSTANTS.CALIBRATION.MIN_TRIALS} trials required`
      );
    }
  }

  private getResolutionScore(resolution: string): number {
    const scores: Record<string, number> = {
      low: 0.25,
      medium: 0.5,
      high: 0.75,
      very_high: 1.0,
    };
    return scores[resolution] || 0.5;
  }

  private getTypeScore(type: NeuralInterfaceType): number {
    const scores: Record<NeuralInterfaceType, number> = {
      [NeuralInterfaceType.CORTICAL]: 1.0,
      [NeuralInterfaceType.SUBCORTICAL]: 0.9,
      [NeuralInterfaceType.PERIPHERAL]: 0.6,
      [NeuralInterfaceType.SPINAL]: 0.8,
    };
    return scores[type] || 0.5;
  }

  private getCoverageScore(coverage: string): number {
    const scores: Record<string, number> = {
      unilateral: 0.5,
      bilateral: 0.8,
      distributed: 1.0,
    };
    return scores[coverage] || 0.5;
  }

  private determineInvasiveness(type: NeuralInterfaceType): string {
    const invasivenessMap: Record<NeuralInterfaceType, string> = {
      [NeuralInterfaceType.CORTICAL]: 'high',
      [NeuralInterfaceType.SUBCORTICAL]: 'critical',
      [NeuralInterfaceType.PERIPHERAL]: 'moderate',
      [NeuralInterfaceType.SPINAL]: 'high',
    };
    return invasivenessMap[type] || 'moderate';
  }

  private generateApplications(type: NeuralInterfaceType, location: string): string[] {
    const baseApps: Record<NeuralInterfaceType, string[]> = {
      [NeuralInterfaceType.CORTICAL]: [
        'Motor control',
        'Sensory feedback',
        'Speech synthesis',
        'Cognitive enhancement',
      ],
      [NeuralInterfaceType.SUBCORTICAL]: [
        'Deep brain stimulation',
        'Tremor control',
        'Mood regulation',
        'Pain management',
      ],
      [NeuralInterfaceType.PERIPHERAL]: [
        'Prosthetic control',
        'Sensory restoration',
        'Pain blocking',
        'Muscle stimulation',
      ],
      [NeuralInterfaceType.SPINAL]: [
        'Paralysis recovery',
        'Bladder control',
        'Pain management',
        'Spinal cord injury rehabilitation',
      ],
    };

    return baseApps[type] || ['General neural interface'];
  }

  private generateSafetyRequirements(type: NeuralInterfaceType, invasiveness: string): string[] {
    const requirements = ['WIA-AUG-013 compliance', 'ISO 10993 biocompatibility'];

    if (invasiveness === 'high' || invasiveness === 'critical') {
      requirements.push('Extended clinical trials', 'Long-term monitoring', 'Ethics approval');
    }

    if (type === NeuralInterfaceType.CORTICAL || type === NeuralInterfaceType.SUBCORTICAL) {
      requirements.push('Neurosurgical expertise', 'MRI compatibility assessment');
    }

    return requirements;
  }

  private applyFiltering(
    data: number[],
    samplingRate: number,
    filterBand: [number, number]
  ): number[] {
    // Simplified bandpass filter (in production, use proper DSP library)
    // This is a placeholder implementation
    return data.map((value, idx) => {
      // Simple moving average as placeholder
      const windowSize = Math.floor(samplingRate / filterBand[1]);
      const start = Math.max(0, idx - windowSize);
      const end = Math.min(data.length, idx + windowSize);
      const window = data.slice(start, end);
      return window.reduce((sum, v) => sum + v, 0) / window.length;
    });
  }

  private calculateSignalQuality(filtered: number[], raw: number[]): number {
    // Calculate quality based on noise reduction
    const rawVariance = this.calculateVariance(raw);
    const filteredVariance = this.calculateVariance(filtered);
    return Math.max(0, Math.min(1, 1 - filteredVariance / rawVariance));
  }

  private calculateSNR(signal: number[], noise: number[]): number {
    const signalPower = this.calculatePower(signal);
    const noisePower = this.calculatePower(noise.map((n, i) => n - signal[i]));
    return 10 * Math.log10(signalPower / Math.max(noisePower, 1e-10));
  }

  private calculateVariance(data: number[]): number {
    const mean = data.reduce((sum, val) => sum + val, 0) / data.length;
    const variance = data.reduce((sum, val) => sum + Math.pow(val - mean, 2), 0) / data.length;
    return variance;
  }

  private calculatePower(data: number[]): number {
    return data.reduce((sum, val) => sum + val * val, 0) / data.length;
  }

  private detectArtifacts(data: number[], samplingRate: number): ArtifactDetection[] {
    // Simplified artifact detection
    const artifacts: ArtifactDetection[] = [];
    const threshold = this.calculateVariance(data) * 3;

    let inArtifact = false;
    let startSample = 0;

    data.forEach((value, idx) => {
      if (Math.abs(value) > threshold && !inArtifact) {
        inArtifact = true;
        startSample = idx;
      } else if (Math.abs(value) <= threshold && inArtifact) {
        artifacts.push({
          type: 'movement',
          startSample,
          endSample: idx,
          severity: 0.7,
        });
        inArtifact = false;
      }
    });

    return artifacts;
  }

  private extractFeatures(
    data: number[],
    samplingRate: number,
    signalType: SignalType
  ): SignalFeatures {
    // Calculate band powers (simplified)
    const bandPowers = this.calculateBandPowers(data, samplingRate);

    // Calculate time domain features
    const amplitude = Math.max(...data.map(Math.abs));
    const latency = data.findIndex((v) => Math.abs(v) > amplitude * 0.5);
    const duration = data.filter((v) => Math.abs(v) > amplitude * 0.5).length / samplingRate;

    // Calculate entropy
    const entropy = this.calculateEntropy(data);

    return {
      timeDomain: {
        amplitude,
        latency: latency / samplingRate * 1000, // ms
        duration: duration * 1000, // ms
      },
      frequencyDomain: bandPowers,
      entropy,
    };
  }

  private calculateBandPowers(data: number[], samplingRate: number) {
    // Simplified power calculation (in production, use FFT)
    const totalPower = this.calculatePower(data);

    return {
      delta: totalPower * 0.15,
      theta: totalPower * 0.20,
      alpha: totalPower * 0.30,
      beta: totalPower * 0.20,
      gamma: totalPower * 0.10,
      highGamma: totalPower * 0.05,
    };
  }

  private calculateEntropy(data: number[]): number {
    // Spectral entropy calculation (simplified)
    const powers = Object.values(this.calculateBandPowers(data, 1000));
    const totalPower = powers.reduce((sum, p) => sum + p, 0);
    const probabilities = powers.map((p) => p / totalPower);
    return -probabilities.reduce((sum, p) => sum + (p > 0 ? p * Math.log2(p) : 0), 0);
  }

  private inferModality(source: string, target: string): 'motor' | 'sensory' | 'cognitive' | 'autonomic' {
    const motorKeywords = ['motor', 'M1', 'premotor', 'muscle'];
    const sensoryKeywords = ['sensory', 'S1', 'somatosensory', 'visual'];

    const sourceTarget = `${source} ${target}`.toLowerCase();

    if (motorKeywords.some((kw) => sourceTarget.includes(kw.toLowerCase()))) {
      return 'motor';
    } else if (sensoryKeywords.some((kw) => sourceTarget.includes(kw.toLowerCase()))) {
      return 'sensory';
    }

    return 'cognitive';
  }

  private calculatePathwayConfidence(signalType: SignalType, method: string): number {
    const baseConfidence: Record<SignalType, number> = {
      [SignalType.EEG]: 0.6,
      [SignalType.ECoG]: 0.8,
      [SignalType.SPIKE]: 0.95,
      [SignalType.LFP]: 0.85,
    };

    const methodBonus: Record<string, number> = {
      correlation: 0,
      coherence: 0.05,
      granger: 0.1,
      transfer_entropy: 0.15,
    };

    return Math.min(0.99, baseConfidence[signalType] + (methodBonus[method] || 0));
  }

  private estimatePathwayLatency(source: string, target: string): number {
    // Simplified latency estimation (5-50 ms range)
    return 10 + Math.random() * 40;
  }

  private calculateLoadScore(input: LoadManagementInput): number {
    // Base load from task complexity
    const complexityScores = { low: 0.2, moderate: 0.5, high: 0.8 };
    const baseLoad = complexityScores[input.taskComplexity];

    // Adjust with current load
    return (baseLoad + input.currentLoad) / 2;
  }

  private determineLoadLevel(loadScore: number): CognitiveLoadLevel {
    const thresholds = NEURAL_CONSTANTS.COGNITIVE_LOAD;

    if (loadScore < thresholds.LOW_THRESHOLD) return 'low';
    if (loadScore < thresholds.MODERATE_THRESHOLD) return 'moderate';
    if (loadScore < thresholds.HIGH_THRESHOLD) return 'high';
    return 'overload';
  }

  private generateLoadRecommendations(level: CognitiveLoadLevel, withinLimits: boolean): string[] {
    const recommendations: string[] = [];

    if (!withinLimits) {
      recommendations.push('Cognitive load exceeds threshold');
    }

    switch (level) {
      case 'overload':
        recommendations.push('IMMEDIATE ACTION: Take break', 'Reduce task complexity significantly');
        break;
      case 'high':
        recommendations.push('Reduce task difficulty', 'Consider adding breaks');
        break;
      case 'moderate':
        recommendations.push('Monitor load trends', 'Maintain current pace');
        break;
      case 'low':
        recommendations.push('Consider increasing challenge', 'Optimize engagement');
        break;
    }

    return recommendations;
  }

  private simulateTrainingIteration(data: any[], adaptationRate: number, iteration: number): number {
    // Simulate convergence with learning curve
    const maxAccuracy = 0.85 + Math.random() * 0.1;
    const learningRate = adaptationRate * 10;
    return maxAccuracy * (1 - Math.exp(-learningRate * iteration));
  }

  private generateConfusionMatrix(numClasses: number, accuracy: number): number[][] {
    // Generate simulated confusion matrix
    const matrix: number[][] = [];
    const samplesPerClass = 100;

    for (let i = 0; i < numClasses; i++) {
      matrix[i] = [];
      for (let j = 0; j < numClasses; j++) {
        if (i === j) {
          matrix[i][j] = Math.floor(samplesPerClass * accuracy);
        } else {
          matrix[i][j] = Math.floor(samplesPerClass * (1 - accuracy) / (numClasses - 1));
        }
      }
    }

    return matrix;
  }

  private calculateClassificationMetrics(confusionMatrix: number[][]) {
    const numClasses = confusionMatrix.length;
    let totalTP = 0;
    let totalFP = 0;
    let totalFN = 0;

    for (let i = 0; i < numClasses; i++) {
      totalTP += confusionMatrix[i][i];
      totalFP += confusionMatrix.reduce((sum, row, j) => sum + (j !== i ? row[i] : 0), 0);
      totalFN += confusionMatrix[i].reduce((sum, val, j) => sum + (j !== i ? val : 0), 0);
    }

    const precision = totalTP / (totalTP + totalFP);
    const recall = totalTP / (totalTP + totalFN);
    const f1Score = 2 * (precision * recall) / (precision + recall);

    return {
      precision,
      recall,
      f1Score,
      confusionMatrix,
    };
  }

  private predictSignalQuality(types: SignalType[], samplingRate: number, electrodeCount: number): number {
    // Higher quality for more invasive signal types
    const typeQuality: Record<SignalType, number> = {
      [SignalType.EEG]: 0.6,
      [SignalType.ECoG]: 0.8,
      [SignalType.SPIKE]: 0.95,
      [SignalType.LFP]: 0.85,
    };

    const avgTypeQuality = types.reduce((sum, type) => sum + typeQuality[type], 0) / types.length;
    const samplingQuality = Math.min(1, samplingRate / 2000);
    const electrodeQuality = Math.min(1, electrodeCount / 256);

    return (avgTypeQuality * 0.5 + samplingQuality * 0.3 + electrodeQuality * 0.2);
  }

  private calculateSafetyScore(invasiveness: string, intensity: string): number {
    const invasivenessScores: Record<string, number> = {
      none: 100,
      minimal: 90,
      moderate: 75,
      high: 60,
      critical: 50,
    };

    const intensityPenalty: Record<string, number> = {
      minimal: 0,
      moderate: 10,
      high: 20,
    };

    return Math.max(0, (invasivenessScores[invasiveness] || 70) - (intensityPenalty[intensity] || 0));
  }

  private predictEfficacy(
    type: NeuralInterfaceType,
    mode: EnhancementMode,
    target: string
  ): number {
    // Base efficacy on interface type
    const typeEfficacy: Record<NeuralInterfaceType, number> = {
      [NeuralInterfaceType.CORTICAL]: 0.8,
      [NeuralInterfaceType.SUBCORTICAL]: 0.85,
      [NeuralInterfaceType.PERIPHERAL]: 0.7,
      [NeuralInterfaceType.SPINAL]: 0.75,
    };

    return typeEfficacy[type] * (0.9 + Math.random() * 0.1);
  }

  private estimateCalibrationTime(electrodeCount: number, signalTypes: number): number {
    // Base time + complexity factors
    const baseTime = 20; // minutes
    const electrodeTime = electrodeCount / 256 * 10;
    const signalTime = signalTypes * 5;

    return Math.ceil(baseTime + electrodeTime + signalTime);
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Classify neural interface (standalone)
 */
export function classifyInterface(input: InterfaceClassificationInput): InterfaceClassificationResult {
  const sdk = new NeuralEnhancementSDK();
  return sdk.classifyInterface(input);
}

/**
 * Process neural signal (standalone)
 */
export function processSignal(input: SignalProcessingInput): ProcessedSignal {
  const sdk = new NeuralEnhancementSDK();
  return sdk.processSignal(input);
}

/**
 * Map neural pathway (standalone)
 */
export function mapPathway(input: PathwayMappingInput): PathwayMappingResult {
  const sdk = new NeuralEnhancementSDK();
  return sdk.mapPathway(input);
}

/**
 * Manage cognitive load (standalone)
 */
export function manageLoad(input: LoadManagementInput): LoadManagementResult {
  const sdk = new NeuralEnhancementSDK();
  return sdk.manageLoad(input);
}

/**
 * Validate neuroprotection (standalone)
 */
export function protectNeural(input: NeuroprotectionInput): NeuroprotectionResult {
  const sdk = new NeuralEnhancementSDK();
  return sdk.protectNeural(input);
}

/**
 * Calibrate BCI (standalone)
 */
export function calibrateBCI(input: BCICalibrationInput): BCICalibrationResult {
  const sdk = new NeuralEnhancementSDK();
  return sdk.calibrateBCI(input);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { NeuralEnhancementSDK };
export default NeuralEnhancementSDK;
