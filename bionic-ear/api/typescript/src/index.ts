/**
 * WIA-AUG-009: Bionic Ear SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Human Augmentation Auditory Bionics Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

import {
  DeviceType,
  DeviceCategory,
  DeviceClassificationInput,
  DeviceClassification,
  ProcessingStrategy,
  ElectrodeConfig,
  FrequencyMap,
  FrequencyBand,
  SpeechEnhancement,
  NoiseReductionMode,
  DirectionalMode,
  MusicProgram,
  BilateralSync,
  SoundLocation,
  TinnitusStimulation,
  TinnitusQuality,
  AcousticScene,
  SceneClassification,
  DeviceMAP,
  ThresholdMeasurement,
  PatientInfo,
  BionicEarDevice,
  CompatibilityAssessment,
  HearingPerformanceOutcome,
  DeviceReport,
  BionicEarError,
  BionicEarErrorCode,
  BIONIC_EAR_CONSTANTS,
} from './types';

// ============================================================================
// Device Classification
// ============================================================================

/**
 * Classify a bionic ear device
 */
export function classifyDevice(input: DeviceClassificationInput): DeviceClassification {
  // Validate input
  if (!input.type) {
    throw new BionicEarError(
      BionicEarErrorCode.CLASSIFICATION_INVALID_INPUT,
      'Device type is required'
    );
  }

  // Calculate complexity score
  const electrodeScore = (input.electrodes || 0) * 1.5;
  const featureScore = input.features.length * 2;
  const strategyScore = getStrategyComplexity(input.processingStrategy);

  const complexityScore = electrodeScore + featureScore + strategyScore;

  // Determine category
  let category: DeviceCategory;
  if (complexityScore <= 30) category = 'Basic';
  else if (complexityScore <= 60) category = 'Standard';
  else if (complexityScore <= 90) category = 'Advanced';
  else category = 'Premium';

  // Get recommended strategies
  const recommendedStrategies = getRecommendedStrategies(input.type, input.electrodes || 0);

  return {
    type: input.type,
    hearingLossType: 'sensorineural', // Default, should be from patient data
    hearingLossDegree: 'profound', // Default, should be from patient data
    electrodeCount: input.electrodes,
    stimulationType: getStimulationType(input.type),
    category,
    complexityScore,
    recommendedStrategies,
  };
}

function getStrategyComplexity(strategy: ProcessingStrategy): number {
  const complexityMap: Record<ProcessingStrategy, number> = {
    SPEAK: 4,
    CIS: 6,
    ACE: 8,
    FSP: 9,
    HDCIS: 10,
    MP3000: 7,
  };
  return complexityMap[strategy] || 5;
}

function getStimulationType(type: DeviceType): 'electrical' | 'mechanical' | 'vibrational' | 'hybrid' {
  switch (type) {
    case 'COCHLEAR_IMPLANT':
    case 'ABI':
      return 'electrical';
    case 'MIDDLE_EAR':
      return 'mechanical';
    case 'BONE_CONDUCTION':
      return 'vibrational';
    case 'HYBRID':
      return 'hybrid';
    default:
      return 'electrical';
  }
}

function getRecommendedStrategies(type: DeviceType, electrodeCount: number): ProcessingStrategy[] {
  if (type === 'COCHLEAR_IMPLANT' || type === 'HYBRID') {
    if (electrodeCount >= 16) {
      return ['ACE', 'HDCIS', 'FSP'];
    } else if (electrodeCount >= 12) {
      return ['ACE', 'CIS'];
    } else {
      return ['SPEAK', 'CIS'];
    }
  }
  return ['CIS', 'ACE'];
}

// ============================================================================
// Sound Processor Configuration
// ============================================================================

/**
 * Configure sound processor
 */
export function configureProcessor(config: {
  deviceId: string;
  strategy: ProcessingStrategy;
  electrodes: number;
  stimulationRate: number;
  pulseWidth: number;
}): { deviceId: string; strategy: ProcessingStrategy; status: string } {
  // Validate configuration
  if (config.stimulationRate < 250 || config.stimulationRate > 5000) {
    throw new BionicEarError(
      BionicEarErrorCode.CONFIGURATION_FAILED,
      'Stimulation rate must be between 250 and 5000 Hz',
      { stimulationRate: config.stimulationRate }
    );
  }

  if (config.pulseWidth < 10 || config.pulseWidth > 400) {
    throw new BionicEarError(
      BionicEarErrorCode.CONFIGURATION_FAILED,
      'Pulse width must be between 10 and 400 microseconds',
      { pulseWidth: config.pulseWidth }
    );
  }

  return {
    deviceId: config.deviceId,
    strategy: config.strategy,
    status: 'configured',
  };
}

// ============================================================================
// Frequency Mapping
// ============================================================================

/**
 * Map frequencies to electrodes
 */
export function mapFrequencies(config: {
  deviceId: string;
  electrodeCount: number;
  frequencyRange: { low: number; high: number };
  tonotopic: boolean;
}): { deviceId: string; electrodes: FrequencyMap[] } {
  const { electrodeCount, frequencyRange, tonotopic } = config;

  const frequencyMaps: FrequencyMap[] = [];

  for (let i = 0; i < electrodeCount; i++) {
    const ratio = i / (electrodeCount - 1);

    // Logarithmic frequency distribution
    const low = frequencyRange.low * Math.pow(
      frequencyRange.high / frequencyRange.low,
      ratio
    );

    const high = frequencyRange.low * Math.pow(
      frequencyRange.high / frequencyRange.low,
      (i + 1) / (electrodeCount - 1)
    );

    const center = Math.sqrt(low * high); // Geometric mean

    // Characteristic frequency from Greenwood function
    const characteristicFreq = tonotopic
      ? greenwoodFunction(ratio)
      : center;

    frequencyMaps.push({
      electrodeNumber: i + 1,
      centerFrequency: Math.round(center),
      frequencyRange: { low: Math.round(low), high: Math.round(high) },
      characteristicFrequency: Math.round(characteristicFreq),
      gainAdjustment: 0,
    });
  }

  return {
    deviceId: config.deviceId,
    electrodes: frequencyMaps,
  };
}

/**
 * Greenwood frequency-position function for human cochlea
 */
function greenwoodFunction(normalizedPosition: number): number {
  const A = 165.4; // Hz
  const a = 2.1; // slope
  const k = 0.88; // offset

  return A * (Math.pow(10, a * (1 - normalizedPosition)) - k);
}

// ============================================================================
// Speech Recognition Optimization
// ============================================================================

/**
 * Optimize for speech recognition
 */
export function optimizeSpeech(config: {
  deviceId: string;
  noiseReduction: NoiseReductionMode;
  directionality: DirectionalMode;
  compression: 'ADRO' | 'AGC' | 'WDRC';
}): { deviceId: string; optimizations: SpeechEnhancement } {
  const optimizations: SpeechEnhancement = {
    voicedUnvoicedDetection: true,
    fundamentalFrequencyTracking: true,
    formantEnhancement: true,
    consonantEmphasis: config.noiseReduction === 'adaptive' ? 6 : 3, // dB
    pitchShifting: 0,
  };

  return {
    deviceId: config.deviceId,
    optimizations,
  };
}

// ============================================================================
// Music Perception Enhancement
// ============================================================================

/**
 * Enhance music perception
 */
export function enhanceMusic(config: {
  deviceId: string;
  pitchRefinement: boolean;
  harmonicEnhancement: boolean;
  temporalFineStructure: boolean;
}): { deviceId: string; musicProgram: MusicProgram } {
  const musicProgram: MusicProgram = {
    name: 'Music Enhanced',
    compressionRatio: 2.0,
    inputDynamicRange: 75,
    processingStrategy: 'FSP',
    microphone: 'music',
    pitchRefinement: {
      enabled: config.pitchRefinement,
      virtualChannels: 120,
      fineStructureCoding: config.temporalFineStructure,
      fundamentalFrequencyEnhancement: true,
      harmonicPreservation: config.harmonicEnhancement,
    },
    harmonicEnhancement: {
      harmonicCount: 6,
      fundamentalBoost: 3,
      harmonicBoost: 2,
      inharmonicityReduction: true,
    },
    tempoTracking: true,
  };

  return {
    deviceId: config.deviceId,
    musicProgram,
  };
}

// ============================================================================
// Bilateral Synchronization
// ============================================================================

/**
 * Synchronize bilateral implants
 */
export function syncBilateral(config: {
  leftDevice: string;
  rightDevice: string;
  syncMode: 'independent' | 'linked' | 'coordinated';
}): BilateralSync {
  return {
    leftDevice: config.leftDevice,
    rightDevice: config.rightDevice,
    syncMode: config.syncMode,
    timingAccuracy: 50, // microseconds
    interauralLevelDifference: true,
    interauralTimeDifference: true,
    bilateralBeamforming: config.syncMode === 'coordinated',
  };
}

/**
 * Calculate sound location from bilateral input
 */
export function calculateSoundLocation(
  leftSignal: number,
  rightSignal: number
): SoundLocation {
  // Simplified ITD-based localization
  const timeDifference = leftSignal - rightSignal; // Simplified
  const azimuth = Math.max(-90, Math.min(90, timeDifference * 90));

  return {
    azimuth,
    elevation: 0, // Simplified, would need HRTF for elevation
    confidence: 0.8,
  };
}

// ============================================================================
// Tinnitus Suppression
// ============================================================================

/**
 * Configure tinnitus suppression
 */
export function suppressTinnitus(config: {
  deviceId: string;
  frequency: number;
  level: 'mild' | 'moderate' | 'severe';
  quality: TinnitusQuality;
}): TinnitusStimulation {
  let pattern: 'continuous' | 'pulsed' | 'modulated';

  if (config.quality === 'tonal') {
    pattern = 'continuous';
  } else if (config.quality === 'pulsatile') {
    pattern = 'pulsed';
  } else {
    pattern = 'modulated';
  }

  const levelMap = {
    mild: 40,
    moderate: 50,
    severe: 60,
  };

  return {
    enabled: true,
    frequency: config.frequency,
    level: levelMap[config.level],
    pattern,
    modulationFrequency: pattern === 'modulated' ? 10 : undefined,
    electrodes: [10, 11, 12], // Approximate electrodes for typical tinnitus frequency
  };
}

// ============================================================================
// Environmental Sound Classification
// ============================================================================

/**
 * Classify acoustic scene
 */
export function classifyScene(
  audioFeatures: {
    rmsLevel: number;
    speechProbability: number;
    noiseLevel: number;
  }
): SceneClassification {
  let scene: AcousticScene = 'quiet';
  let confidence = 0.9;

  if (audioFeatures.speechProbability > 0.7 && audioFeatures.noiseLevel < 50) {
    scene = 'speech_in_quiet';
  } else if (audioFeatures.speechProbability > 0.7 && audioFeatures.noiseLevel >= 50) {
    scene = 'speech_in_noise';
  } else if (audioFeatures.noiseLevel >= 70) {
    scene = 'noise';
  } else if (audioFeatures.rmsLevel < 30) {
    scene = 'quiet';
  }

  return {
    scene,
    confidence,
    features: {
      rmsLevel: audioFeatures.rmsLevel,
      peakLevel: audioFeatures.rmsLevel + 10,
      dynamicRange: 40,
      spectralCentroid: 1500,
      spectralRolloff: 3000,
      spectralFlux: 0.3,
      zeroCrossingRate: 0.5,
      temporalCentroid: 0.5,
      speechProbability: audioFeatures.speechProbability,
      voicingRate: audioFeatures.speechProbability * 0.6,
      noiseLevel: audioFeatures.noiseLevel,
      reverberation: 200,
    },
  };
}

// ============================================================================
// Device Assessment
// ============================================================================

/**
 * Assess device compatibility for patient
 */
export function assessDevice(input: {
  device: {
    type: DeviceType;
    manufacturer: string;
    model: string;
    electrodes: number;
  };
  patient: {
    hearingLossType: string;
    hearingLossDegree: string;
    duration: string;
    residualHearing: string;
  };
  requirements: {
    primaryUse: string[];
    environment: string[];
    bilateralFitting: boolean;
  };
}): CompatibilityAssessment {
  // Calculate compatibility score
  let compatibility = 70; // Base score

  // Device-specific adjustments
  if (input.device.electrodes >= 16) compatibility += 10;
  if (input.device.type === 'COCHLEAR_IMPLANT') compatibility += 10;

  // Patient-specific adjustments
  if (input.patient.duration === 'postlingual') compatibility += 10;
  if (input.patient.residualHearing === 'none' && input.device.type === 'COCHLEAR_IMPLANT') {
    compatibility += 5;
  }

  // Requirements adjustments
  if (input.requirements.bilateralFitting) compatibility += 5;

  compatibility = Math.min(100, compatibility);

  // Recommend strategy
  const recommendedStrategy: ProcessingStrategy =
    input.device.electrodes >= 16 ? 'ACE' : 'CIS';

  // Estimate speech recognition
  const speechRecognitionScore = Math.min(95, compatibility - 10);

  return {
    compatibility,
    recommendedStrategy,
    speechRecognitionScore,
    estimatedTrainingTime: input.patient.duration === 'postlingual' ? 20 : 60,
    recommendations: [
      'Start with ACE processing strategy',
      'Schedule regular MAP adjustments',
      'Consider bilateral fitting for optimal performance',
    ],
    challenges: input.patient.duration === 'prelingual'
      ? ['Extended training period required', 'Lower initial speech recognition expected']
      : [],
  };
}

// ============================================================================
// Frequency Band Helpers
// ============================================================================

/**
 * Create standard frequency bands
 */
export function createFrequencyBands(): FrequencyBand {
  return {
    low: BIONIC_EAR_CONSTANTS.FREQUENCY.LOW_FREQ_RANGE.low,
    mid: BIONIC_EAR_CONSTANTS.FREQUENCY.MID_FREQ_RANGE.low,
    high: BIONIC_EAR_CONSTANTS.FREQUENCY.HIGH_FREQ_RANGE.low,
  };
}

// ============================================================================
// SDK Class
// ============================================================================

/**
 * Main Bionic Ear SDK class
 */
export class BionicEarSDK {
  private devices: Map<string, BionicEarDevice> = new Map();
  private maps: Map<string, DeviceMAP> = new Map();

  /**
   * Register a device
   */
  registerDevice(device: BionicEarDevice): void {
    this.devices.set(device.id, device);
  }

  /**
   * Get device by ID
   */
  getDevice(deviceId: string): BionicEarDevice | undefined {
    return this.devices.get(deviceId);
  }

  /**
   * Classify device
   */
  classifyDevice(input: DeviceClassificationInput): DeviceClassification {
    return classifyDevice(input);
  }

  /**
   * Map frequencies
   */
  mapFrequencies(config: {
    deviceId: string;
    electrodeCount: number;
    frequencyRange: { low: number; high: number };
    tonotopic: boolean;
  }): { deviceId: string; electrodes: FrequencyMap[] } {
    return mapFrequencies(config);
  }

  /**
   * Optimize for speech
   */
  optimizeSpeech(config: {
    deviceId: string;
    noiseReduction: NoiseReductionMode;
    directionality: DirectionalMode;
    compression: 'ADRO' | 'AGC' | 'WDRC';
  }): { deviceId: string; optimizations: SpeechEnhancement } {
    return optimizeSpeech(config);
  }

  /**
   * Enhance music
   */
  enhanceMusic(config: {
    deviceId: string;
    pitchRefinement: boolean;
    harmonicEnhancement: boolean;
    temporalFineStructure: boolean;
  }): { deviceId: string; musicProgram: MusicProgram } {
    return enhanceMusic(config);
  }

  /**
   * Sync bilateral devices
   */
  syncBilateral(config: {
    leftDevice: string;
    rightDevice: string;
    syncMode: 'independent' | 'linked' | 'coordinated';
  }): BilateralSync {
    return syncBilateral(config);
  }

  /**
   * Suppress tinnitus
   */
  suppressTinnitus(config: {
    deviceId: string;
    frequency: number;
    level: 'mild' | 'moderate' | 'severe';
    quality: TinnitusQuality;
  }): TinnitusStimulation {
    return suppressTinnitus(config);
  }

  /**
   * Classify acoustic scene
   */
  classifyScene(audioFeatures: {
    rmsLevel: number;
    speechProbability: number;
    noiseLevel: number;
  }): SceneClassification {
    return classifyScene(audioFeatures);
  }

  /**
   * Assess device compatibility
   */
  assessDevice(input: {
    device: {
      type: DeviceType;
      manufacturer: string;
      model: string;
      electrodes: number;
    };
    patient: {
      hearingLossType: string;
      hearingLossDegree: string;
      duration: string;
      residualHearing: string;
    };
    requirements: {
      primaryUse: string[];
      environment: string[];
      bilateralFitting: boolean;
    };
  }): CompatibilityAssessment {
    return assessDevice(input);
  }

  /**
   * Create frequency bands
   */
  createFrequencyBands(): FrequencyBand {
    return createFrequencyBands();
  }
}

// ============================================================================
// Exports
// ============================================================================

export * from './types';

export {
  classifyDevice,
  configureProcessor,
  mapFrequencies,
  optimizeSpeech,
  enhanceMusic,
  syncBilateral,
  calculateSoundLocation,
  suppressTinnitus,
  classifyScene,
  assessDevice,
  createFrequencyBands,
  BionicEarSDK,
};

export default BionicEarSDK;
