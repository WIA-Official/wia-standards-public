/**
 * WIA-AUG-003: Neural Enhancement - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Neural Enhancement Working Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Neural Interface Types
// ============================================================================

/**
 * Neural interface classification by anatomical location
 */
export enum NeuralInterfaceType {
  CORTICAL = 'CORTICAL',           // Cortical surface interfaces
  SUBCORTICAL = 'SUBCORTICAL',     // Deep brain structures
  PERIPHERAL = 'PERIPHERAL',       // Peripheral nerve interfaces
  SPINAL = 'SPINAL',               // Spinal cord interfaces
}

/**
 * Interface invasiveness level
 */
export type Invasiveness = 'none' | 'minimal' | 'moderate' | 'high' | 'critical';

/**
 * Spatial resolution classification
 */
export type SpatialResolution = 'low' | 'medium' | 'high' | 'very_high';

/**
 * Coverage area classification
 */
export type Coverage = 'unilateral' | 'bilateral' | 'distributed';

/**
 * Neural interface classification input
 */
export interface InterfaceClassificationInput {
  /** Type of neural interface */
  type: NeuralInterfaceType;

  /** Anatomical location */
  location: string;

  /** Number of electrodes */
  electrodeCount: number;

  /** Spatial resolution level */
  resolution: SpatialResolution;

  /** Coverage area */
  coverage?: Coverage;

  /** Target brain regions */
  targetRegions?: string[];
}

/**
 * Neural interface classification result
 */
export interface InterfaceClassificationResult {
  /** Interface type */
  type: NeuralInterfaceType;

  /** Classification score (0-100) */
  score: number;

  /** Invasiveness level */
  invasiveness: Invasiveness;

  /** Spatial resolution */
  resolution: SpatialResolution;

  /** Recommended applications */
  applications: string[];

  /** Safety requirements */
  safetyRequirements: string[];
}

// ============================================================================
// Signal Types
// ============================================================================

/**
 * Neural signal modality types
 */
export enum SignalType {
  EEG = 'EEG',       // Electroencephalography (scalp)
  ECoG = 'ECoG',     // Electrocorticography (cortical surface)
  SPIKE = 'SPIKE',   // Single-unit recordings
  LFP = 'LFP',       // Local field potentials
}

/**
 * Signal processing input
 */
export interface SignalProcessingInput {
  /** Raw signal data */
  rawData: number[];

  /** Signal type */
  signalType: SignalType;

  /** Sampling rate (Hz) */
  samplingRate: number;

  /** Filter band [low, high] (Hz) */
  filterBand?: [number, number];

  /** Enable artifact removal */
  artifactRemoval?: boolean;
}

/**
 * Processed signal output
 */
export interface ProcessedSignal {
  /** Filtered signal data */
  filteredData: number[];

  /** Signal quality metric (0-1) */
  quality: number;

  /** Signal-to-noise ratio (dB) */
  snr: number;

  /** Detected artifacts */
  artifacts: ArtifactDetection[];

  /** Extracted features */
  features: SignalFeatures;
}

/**
 * Artifact detection result
 */
export interface ArtifactDetection {
  /** Artifact type */
  type: 'movement' | 'blink' | 'muscle' | 'line_noise' | 'electrode';

  /** Start time (samples) */
  startSample: number;

  /** End time (samples) */
  endSample: number;

  /** Severity (0-1) */
  severity: number;
}

/**
 * Extracted signal features
 */
export interface SignalFeatures {
  /** Time domain features */
  timeDomain: {
    amplitude: number;
    latency: number;
    duration: number;
  };

  /** Frequency domain features (band powers) */
  frequencyDomain: {
    delta: number;    // 0.5-4 Hz
    theta: number;    // 4-8 Hz
    alpha: number;    // 8-13 Hz
    beta: number;     // 13-30 Hz
    gamma: number;    // 30-100 Hz
    highGamma: number; // 100-200 Hz
  };

  /** Spectral entropy */
  entropy: number;
}

// ============================================================================
// Enhancement Mode Types
// ============================================================================

/**
 * Neural enhancement modality
 */
export enum EnhancementMode {
  STIMULATION = 'STIMULATION',     // Active neural stimulation
  INHIBITION = 'INHIBITION',       // Targeted suppression
  MODULATION = 'MODULATION',       // Dynamic signal modulation
  AUGMENTATION = 'AUGMENTATION',   // Cognitive/sensory augmentation
}

/**
 * Enhancement configuration
 */
export interface EnhancementConfig {
  /** Enhancement mode */
  mode: EnhancementMode;

  /** Target brain region */
  targetRegion: string;

  /** Enhancement intensity (0-1) */
  intensity: number;

  /** Duration (ms) */
  duration: number;

  /** Frequency (Hz) */
  frequency?: number;

  /** Pulse parameters */
  pulse?: PulseParameters;
}

/**
 * Stimulation pulse parameters
 */
export interface PulseParameters {
  /** Current amplitude (mA) */
  current: number;

  /** Pulse width (μs) */
  pulseWidth: number;

  /** Inter-pulse interval (ms) */
  interPulseInterval: number;

  /** Charge balancing enabled */
  chargeBalanced: boolean;
}

// ============================================================================
// Neural Pathway Types
// ============================================================================

/**
 * Brain region identifier
 */
export interface BrainRegion {
  /** Region name */
  name: string;

  /** Anatomical coordinate */
  coordinate?: {
    x: number;
    y: number;
    z: number;
  };

  /** Brodmann area (if applicable) */
  brodmannArea?: number;

  /** Hemisphere */
  hemisphere?: 'left' | 'right' | 'bilateral';
}

/**
 * Neural pathway definition
 */
export interface NeuralPathway {
  /** Pathway identifier */
  id: string;

  /** Source region */
  source: BrainRegion;

  /** Target region */
  target: BrainRegion;

  /** Intermediate regions */
  intermediate: BrainRegion[];

  /** Functional modality */
  modality: 'motor' | 'sensory' | 'cognitive' | 'autonomic';

  /** Pathway function */
  function: string;

  /** Connectivity strength (0-1) */
  strength: number;
}

/**
 * Pathway mapping input
 */
export interface PathwayMappingInput {
  /** Source region identifier */
  sourceRegion: string;

  /** Target region identifier */
  targetRegion: string;

  /** Signal type for mapping */
  signalType: SignalType;

  /** Mapping method */
  method?: 'correlation' | 'coherence' | 'granger' | 'transfer_entropy';
}

/**
 * Pathway mapping result
 */
export interface PathwayMappingResult {
  /** Identified pathway */
  pathway: NeuralPathway;

  /** Mapping confidence (0-1) */
  confidence: number;

  /** Directional strength (source→target) */
  directionalStrength: number;

  /** Latency (ms) */
  latency: number;

  /** Pathway visualization data */
  visualization?: {
    nodes: BrainRegion[];
    edges: Array<{ from: string; to: string; weight: number }>;
  };
}

// ============================================================================
// Cognitive Load Types
// ============================================================================

/**
 * Cognitive load level
 */
export type CognitiveLoadLevel = 'low' | 'moderate' | 'high' | 'overload';

/**
 * Cognitive load indicators
 */
export interface CognitiveLoadIndicators {
  /** Physiological measures */
  physiological: {
    pupilDiameter: number;         // mm
    heartRateVariability: number;  // ms
    blinkRate: number;             // blinks/min
  };

  /** Neural measures */
  neural: {
    thetaPower: number;            // Frontal theta (4-8 Hz)
    alphaPower: number;            // Parietal alpha (8-13 Hz)
    betaPower: number;             // Central beta (13-30 Hz)
  };

  /** Performance measures */
  performance: {
    reactionTime: number;          // ms
    errorRate: number;             // 0-1
    taskCompletion: number;        // 0-1
  };
}

/**
 * Cognitive load management input
 */
export interface LoadManagementInput {
  /** Current task complexity */
  taskComplexity: 'low' | 'moderate' | 'high';

  /** Current cognitive load (0-1) */
  currentLoad: number;

  /** Load threshold (0-1) */
  threshold: number;

  /** Indicators (optional) */
  indicators?: CognitiveLoadIndicators;
}

/**
 * Cognitive load management result
 */
export interface LoadManagementResult {
  /** Current load level */
  loadLevel: CognitiveLoadLevel;

  /** Load score (0-1) */
  loadScore: number;

  /** Is load within safe limits */
  withinLimits: boolean;

  /** Recommendations */
  recommendations: string[];

  /** Required actions */
  actions: {
    reduceComplexity: boolean;
    increaseAssistance: boolean;
    addBreaks: boolean;
    simplifyFeedback: boolean;
  };
}

// ============================================================================
// Neuroprotection Types
// ============================================================================

/**
 * Neuroprotection validation input
 */
export interface NeuroprotectionInput {
  /** Stimulation current (mA) */
  stimulationCurrent: number;

  /** Pulse duration (μs) */
  duration: number;

  /** Stimulation frequency (Hz) */
  frequency: number;

  /** Pulse width (μs) */
  pulseWidth: number;

  /** Electrode surface area (mm²) */
  electrodeArea?: number;
}

/**
 * Neuroprotection validation result
 */
export interface NeuroprotectionResult {
  /** Is stimulation safe */
  isSafe: boolean;

  /** Charge density (μC/cm²) */
  chargeDensity: number;

  /** Charge per phase (μC) */
  chargePerPhase: number;

  /** Maximum safe limit */
  safeLimit: number;

  /** Safety margin (%) */
  safetyMargin: number;

  /** Warnings */
  warnings: string[];

  /** Required modifications */
  modifications: string[];
}

// ============================================================================
// BCI Calibration Types
// ============================================================================

/**
 * BCI calibration input
 */
export interface BCICalibrationInput {
  /** User training data */
  userFeedback: CalibrationData[];

  /** Learning/adaptation rate */
  adaptationRate: number;

  /** Convergence threshold (accuracy) */
  convergenceThreshold: number;

  /** Maximum calibration iterations */
  maxIterations?: number;
}

/**
 * Calibration training data
 */
export interface CalibrationData {
  /** Trial identifier */
  trialId: number;

  /** Neural features */
  features: number[];

  /** User intent/label */
  label: string | number;

  /** Trial timestamp */
  timestamp: Date;

  /** Trial quality (0-1) */
  quality: number;
}

/**
 * BCI calibration result
 */
export interface BCICalibrationResult {
  /** Calibration successful */
  success: boolean;

  /** Achieved accuracy (0-1) */
  accuracy: number;

  /** Number of iterations performed */
  iterations: number;

  /** Decoder model */
  model: DecoderModel;

  /** Performance metrics */
  metrics: {
    precision: number;
    recall: number;
    f1Score: number;
    confusionMatrix: number[][];
  };

  /** Convergence history */
  convergenceHistory: number[];
}

/**
 * Decoder model configuration
 */
export interface DecoderModel {
  /** Model type */
  type: 'LDA' | 'SVM' | 'ANN' | 'CNN' | 'RNN';

  /** Model parameters */
  parameters: Record<string, unknown>;

  /** Feature dimensions */
  featureDimensions: number;

  /** Output classes */
  classes: string[] | number[];

  /** Model version */
  version: string;
}

// ============================================================================
// Plasticity Adaptation Types
// ============================================================================

/**
 * Plasticity adaptation mode
 */
export type PlasticityMode = 'supervised' | 'unsupervised' | 'reinforcement';

/**
 * Adaptation configuration
 */
export interface AdaptationConfig {
  /** Adaptation mode */
  mode: PlasticityMode;

  /** Learning rate */
  learningRate: number;

  /** Update frequency (Hz) */
  updateFrequency: number;

  /** Adaptation enabled */
  enabled: boolean;
}

/**
 * Plasticity metrics
 */
export interface PlasticityMetrics {
  /** Adaptation rate */
  adaptationRate: number;

  /** Stability measure (0-1) */
  stability: number;

  /** Performance improvement */
  improvement: number;

  /** Convergence status */
  converged: boolean;
}

// ============================================================================
// Safety and Monitoring Types
// ============================================================================

/**
 * Safety alert levels
 */
export type AlertLevel = 'info' | 'warning' | 'critical' | 'emergency';

/**
 * Safety alert
 */
export interface SafetyAlert {
  /** Alert identifier */
  id: string;

  /** Alert level */
  level: AlertLevel;

  /** Alert type */
  type: 'charge_density' | 'temperature' | 'impedance' | 'signal_quality' | 'cognitive_load';

  /** Alert message */
  message: string;

  /** Timestamp */
  timestamp: Date;

  /** Suggested action */
  action: string;
}

/**
 * Monitoring data point
 */
export interface MonitoringData {
  /** Timestamp */
  timestamp: Date;

  /** Neural signal quality (0-1) */
  signalQuality: number;

  /** Electrode impedances (kΩ) */
  impedances: number[];

  /** Temperature (°C) */
  temperature: number;

  /** Power consumption (mW) */
  powerConsumption: number;

  /** Active alerts */
  alerts: SafetyAlert[];
}

// ============================================================================
// Comprehensive Assessment Types
// ============================================================================

/**
 * Complete neural interface assessment input
 */
export interface InterfaceAssessmentInput {
  /** Interface configuration */
  interface: InterfaceClassificationInput;

  /** Signal specifications */
  signals: {
    types: SignalType[];
    samplingRate: number;
    bandwidth: [number, number];
  };

  /** Enhancement configuration */
  enhancement: {
    mode: EnhancementMode;
    target: string;
    intensity: 'minimal' | 'moderate' | 'high';
  };

  /** Patient information (optional) */
  patient?: {
    age: number;
    condition: string;
    contraindications: string[];
  };
}

/**
 * Complete neural interface assessment result
 */
export interface InterfaceAssessmentResult {
  /** Interface classification */
  classification: InterfaceClassificationResult;

  /** Signal quality prediction */
  signalQuality: number;

  /** Safety score (0-100) */
  safetyScore: number;

  /** Efficacy prediction (0-1) */
  efficacyPrediction: number;

  /** Recommendations */
  recommendations: string[];

  /** Required certifications */
  certifications: string[];

  /** Estimated calibration time (minutes) */
  calibrationTime: number;
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Neural enhancement constants
 */
export const NEURAL_CONSTANTS = {
  /** Signal processing */
  SIGNAL: {
    MIN_SAMPLING_RATE: 250,        // Hz
    MAX_SAMPLING_RATE: 40000,      // Hz
    MIN_SNR: 3,                     // dB
    RECOMMENDED_SNR: 10,            // dB
  },

  /** Stimulation safety limits */
  STIMULATION: {
    MAX_CHARGE_DENSITY: 30,        // μC/cm²
    MAX_CURRENT: 5.0,              // mA
    MAX_FREQUENCY: 250,            // Hz
    MAX_PULSE_WIDTH: 500,          // μs
    MIN_INTERPHASE_GAP: 50,        // μs
  },

  /** Cognitive load thresholds */
  COGNITIVE_LOAD: {
    LOW_THRESHOLD: 0.3,
    MODERATE_THRESHOLD: 0.6,
    HIGH_THRESHOLD: 0.8,
    OVERLOAD_THRESHOLD: 0.9,
  },

  /** Temperature limits */
  TEMPERATURE: {
    MAX_INCREASE: 1.0,             // °C above baseline
    CRITICAL_TEMP: 38.0,           // °C
    BASELINE: 37.0,                // °C
  },

  /** Impedance limits */
  IMPEDANCE: {
    SCALP_MAX: 5,                  // kΩ
    CORTICAL_MAX: 100,             // kΩ
    WARNING_THRESHOLD: 1000,       // kΩ
  },

  /** Calibration */
  CALIBRATION: {
    MIN_TRIALS: 20,                // per class
    RECOMMENDED_TRIALS: 50,        // per class
    MIN_ACCURACY: 0.7,             // 70%
    TARGET_ACCURACY: 0.8,          // 80%
    MAX_LATENCY: 100,              // ms
  },
} as const;

// ============================================================================
// Error Types
// ============================================================================

/**
 * Neural enhancement error codes
 */
export enum NeuralErrorCode {
  INVALID_SIGNAL_TYPE = 'N001',
  SIGNAL_QUALITY_LOW = 'N002',
  UNSAFE_STIMULATION = 'N003',
  CALIBRATION_FAILED = 'N004',
  PATHWAY_NOT_FOUND = 'N005',
  COGNITIVE_OVERLOAD = 'N006',
  INTERFACE_ERROR = 'N007',
  NEUROPROTECTION_VIOLATION = 'N008',
}

/**
 * Neural enhancement error class
 */
export class NeuralError extends Error {
  constructor(
    public code: NeuralErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'NeuralError';
  }
}

// ============================================================================
// Export All Types
// ============================================================================

export type {
  Invasiveness,
  SpatialResolution,
  Coverage,
  InterfaceClassificationInput,
  InterfaceClassificationResult,
  SignalProcessingInput,
  ProcessedSignal,
  ArtifactDetection,
  SignalFeatures,
  EnhancementConfig,
  PulseParameters,
  BrainRegion,
  NeuralPathway,
  PathwayMappingInput,
  PathwayMappingResult,
  CognitiveLoadLevel,
  CognitiveLoadIndicators,
  LoadManagementInput,
  LoadManagementResult,
  NeuroprotectionInput,
  NeuroprotectionResult,
  BCICalibrationInput,
  CalibrationData,
  BCICalibrationResult,
  DecoderModel,
  PlasticityMode,
  AdaptationConfig,
  PlasticityMetrics,
  AlertLevel,
  SafetyAlert,
  MonitoringData,
  InterfaceAssessmentInput,
  InterfaceAssessmentResult,
};

export {
  NeuralInterfaceType,
  SignalType,
  EnhancementMode,
  NEURAL_CONSTANTS,
  NeuralErrorCode,
  NeuralError,
};
