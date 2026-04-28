/**
 * WIA-AUG-004: Sensory Enhancement - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Human Augmentation Sensory Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Sensory Modality Types
// ============================================================================

/**
 * Primary sensory modalities
 */
export enum SensoryModality {
  VISUAL = 'visual',
  AUDITORY = 'auditory',
  TACTILE = 'tactile',
  OLFACTORY = 'olfactory',
  GUSTATORY = 'gustatory',
  PROPRIOCEPTIVE = 'proprioceptive',
  VESTIBULAR = 'vestibular',
  THERMOCEPTION = 'thermoception',
  NOCICEPTION = 'nociception'
}

/**
 * Stimulus types for sensory modalities
 */
export type StimulusType = 'electromagnetic' | 'mechanical' | 'chemical' | 'thermal';

/**
 * Sensory range definition
 */
export interface SensoryRange {
  /** Minimum detectable value */
  min: number;

  /** Maximum detectable value */
  max: number;

  /** Smallest distinguishable difference */
  resolution: number;

  /** Measurement unit */
  unit: string;

  /** Sampling rate (if applicable) */
  frequency?: number;
}

/**
 * Sensory characteristics
 */
export interface SensoryCharacteristics {
  /** Sensory modality */
  modality: SensoryModality;

  /** Type of stimulus */
  stimulusType: StimulusType;

  /** Receptor type */
  receptorType: string;

  /** Normal sensory range */
  normalRange: SensoryRange;

  /** Resolution limit */
  resolutionLimit: number;

  /** Dynamic range (dB or equivalent) */
  dynamicRange: number;

  /** Response time in milliseconds */
  responseTime: number;

  /** Adaptation rate (0-1) */
  adaptationRate: number;

  /** Fatigue resistance (0-1) */
  fatigueResistance: number;

  /** Compatible modalities for integration */
  crossModalCompatibility: SensoryModality[];

  /** Potential for substitution (0-1) */
  substitutionPotential: number;
}

// ============================================================================
// Enhancement Types
// ============================================================================

/**
 * Types of sensory enhancement
 */
export enum EnhancementType {
  /** Restore impaired sense to normal */
  RESTORATION = 'restoration',

  /** Enhance normal sense beyond baseline */
  AUGMENTATION = 'augmentation',

  /** Create entirely new sensory capability */
  NEW_SENSE = 'new_sense',

  /** Replace one sense with another */
  SUBSTITUTION = 'substitution'
}

/**
 * Enhancement level classification
 */
export type EnhancementLevel = 'Level1' | 'Level2' | 'Level3' | 'Level4' | 'Level5';

/**
 * Enhancement input parameters
 */
export interface EnhancementInput {
  /** Baseline sensory range */
  baselineRange: SensoryRange;

  /** Target enhanced range */
  targetRange: SensoryRange;

  /** Sensory modality */
  modality: SensoryModality;

  /** Purpose of enhancement */
  purpose: string;

  /** Safety margin (0.8-0.95) */
  safetyMargin?: number;
}

/**
 * Enhancement classification result
 */
export interface EnhancementClassification {
  /** Enhancement type */
  type: EnhancementType;

  /** Enhancement level */
  level: EnhancementLevel;

  /** Enhancement factor */
  factor: number;

  /** Safety score (0-1) */
  safetyScore: number;

  /** Neural compatibility (0-1) */
  neuralCompatibility: number;

  /** Reversibility (0-1) */
  reversibility: number;
}

/**
 * Enhancement parameters
 */
export interface EnhancementParams {
  /** Sensory modality to enhance */
  modality: SensoryModality;

  /** Base sensory range */
  baseRange: SensoryRange;

  /** Enhancement factor (1.0-10.0) */
  enhancementFactor: number;

  /** Safety margin (0.8-0.95) */
  safetyMargin: number;

  /** Adaptation period in days */
  adaptationPeriod?: number;
}

/**
 * Enhancement result
 */
export interface EnhancementResult {
  /** Original modality */
  modality: SensoryModality;

  /** Enhancement type */
  enhancementType: EnhancementType;

  /** Original range */
  originalRange: SensoryRange;

  /** New enhanced range */
  newRange: SensoryRange;

  /** Enhancement factor achieved */
  factor: number;

  /** Safety score */
  safetyScore: number;

  /** Estimated adaptation time in hours */
  adaptationTime: number;
}

// ============================================================================
// Multi-Sensory Integration Types
// ============================================================================

/**
 * Integration modes
 */
export enum IntegrationMode {
  /** Sum of inputs */
  ADDITIVE = 'additive',

  /** One sense dominates */
  DOMINANT = 'dominant',

  /** Enhanced combination */
  SYNERGISTIC = 'synergistic',

  /** Senses compete */
  COMPETITIVE = 'competitive',

  /** Fill gaps */
  COMPLEMENTARY = 'complementary'
}

/**
 * Sensory input for integration
 */
export interface SensoryInput {
  /** Sensory modality */
  modality: SensoryModality;

  /** Sensory data */
  data: SensoryData;

  /** Timestamp in microseconds */
  timestamp: number;

  /** Priority (0-1) */
  priority: number;

  /** Reliability (0-1) */
  reliability: number;
}

/**
 * Sensory data payload
 */
export interface SensoryData {
  /** Data type */
  type: string;

  /** Raw values */
  values: number[];

  /** Data quality (0-1) */
  quality: number;

  /** Metadata */
  metadata?: Record<string, unknown>;
}

/**
 * Synchronization parameters
 */
export interface SyncParameters {
  /** Maximum acceptable latency in milliseconds */
  maxLatency: number;

  /** Required synchronization precision in milliseconds */
  syncPrecision: number;

  /** Critical synchronization flag */
  critical: boolean;
}

/**
 * Integrated percept result
 */
export interface IntegratedPercept {
  /** Contributing modalities */
  modalities: SensoryModality[];

  /** Integration mode used */
  mode: IntegrationMode;

  /** Integrated data */
  data: SensoryData;

  /** Integration quality score (0-1) */
  quality: number;

  /** Temporal synchronization (0-1) */
  synchronization: number;

  /** Signal fidelity (0-1) */
  fidelity: number;

  /** Processing latency in milliseconds */
  latency: number;
}

// ============================================================================
// Sensory Substitution Types
// ============================================================================

/**
 * Mapping method for substitution
 */
export type MappingMethod = 'linear' | 'logarithmic' | 'exponential' | 'custom';

/**
 * Sensory substitution configuration
 */
export interface SensorySubstitution {
  /** Source (missing/impaired) sense */
  sourceSense: SensoryModality;

  /** Target (replacement) sense */
  targetSense: SensoryModality;

  /** Mapping method */
  mappingMethod: MappingMethod;

  /** Substitution fidelity (0-1) */
  fidelity: number;

  /** Learning curve in hours */
  learningCurve: number;

  /** Compatibility score (0-1) */
  compatibility: number;
}

/**
 * Substitution result
 */
export interface SubstitutionResult {
  /** Source modality */
  source: SensoryModality;

  /** Target modality */
  target: SensoryModality;

  /** Mapping function */
  mapping: MappingFunction;

  /** Fidelity achieved (0-1) */
  fidelity: number;

  /** Information transfer rate (0-1) */
  informationTransfer: number;

  /** User proficiency (0-1) */
  proficiency: number;
}

/**
 * Mapping function definition
 */
export interface MappingFunction {
  /** Function type */
  type: MappingMethod;

  /** Function parameters */
  parameters: Record<string, number>;

  /** Optional lookup table */
  lut?: LookupTable;
}

/**
 * Lookup table for mapping
 */
export interface LookupTable {
  /** Input values */
  inputs: number[];

  /** Output values */
  outputs: number[];
}

// ============================================================================
// Calibration Types
// ============================================================================

/**
 * Calibration parameters
 */
export interface CalibrationParams {
  /** Sensory modality */
  modality: SensoryModality;

  /** Detection threshold */
  threshold?: number;

  /** Sensitivity (0-1) */
  sensitivity: number;

  /** Resolution */
  resolution: number;

  /** Precision/repeatability (0-1) */
  precision?: number;

  /** Adaptation rate (0-1) */
  adaptationRate: number;

  /** Fatigue compensation (0-1) */
  fatigueCompensation?: number;
}

/**
 * Calibration standard reference
 */
export interface CalibrationStandard {
  /** Standard name */
  name: string;

  /** Reference value */
  value: number;

  /** Unit */
  unit: string;

  /** Tolerance */
  tolerance: number;

  /** Certification */
  certified: boolean;
}

/**
 * Calibration result
 */
export interface CalibrationResult {
  /** Modality calibrated */
  modality: SensoryModality;

  /** Calibration status */
  isCalibrated: boolean;

  /** Accuracy achieved (0-1) */
  accuracy: number;

  /** Drift from standard (%) */
  drift: number;

  /** Calibration timestamp */
  timestamp: Date;

  /** Next calibration due */
  nextCalibration: Date;

  /** Calibration notes */
  notes?: string;
}

// ============================================================================
// Overload Protection Types
// ============================================================================

/**
 * Overload protection configuration
 */
export interface OverloadProtection {
  /** Sensory modality */
  modality: SensoryModality;

  /** Warning threshold (80% of max) */
  warningThreshold: number;

  /** Critical threshold (95% of max) */
  criticalThreshold: number;

  /** Danger threshold (100% of max) */
  dangerThreshold: number;

  /** Auto-limiting enabled */
  autoLimiting: boolean;

  /** Gradual reduction enabled */
  gradualReduction: boolean;

  /** Emergency shutoff enabled */
  emergencyShutoff: boolean;

  /** Recovery time in milliseconds */
  recoveryTime: number;

  /** Gradual reintroduction enabled */
  gradualReintroduction: boolean;
}

/**
 * Overload monitor state
 */
export interface OverloadMonitor {
  /** Current intensity level */
  currentIntensity: number;

  /** Maximum safe intensity */
  maxIntensity: number;

  /** Exposure duration in milliseconds */
  exposureDuration: number;

  /** Safe exposure limit in milliseconds */
  safeExposureLimit: number;
}

/**
 * Protection status
 */
export interface ProtectionStatus {
  /** Protection active */
  active: boolean;

  /** Current risk level */
  riskLevel: 'safe' | 'warning' | 'critical' | 'danger';

  /** Risk percentage (0-100) */
  riskPercentage: number;

  /** Actions taken */
  actionsTaken: string[];

  /** Recovery progress (0-1) */
  recoveryProgress: number;

  /** Recommended actions */
  recommendations: string[];
}

/**
 * Intensity limiter configuration
 */
export interface IntensityLimiter {
  /** Soft limit (begin reduction) */
  softLimit: number;

  /** Hard limit (absolute maximum) */
  hardLimit: number;

  /** Limiter type */
  limiterType: 'linear' | 'logarithmic' | 'exponential';

  /** Compression ratio */
  compressionRatio: number;

  /** Attack time in milliseconds */
  attackTime: number;

  /** Release time in milliseconds */
  releaseTime: number;
}

// ============================================================================
// Cross-Modal Mapping Types
// ============================================================================

/**
 * Cross-modal map configuration
 */
export interface CrossModalMap {
  /** Source modality */
  source: SensoryModality;

  /** Target modality */
  target: SensoryModality;

  /** Mapping function */
  mappingFunction: MappingFunction;

  /** Bidirectional mapping */
  bidirectional: boolean;

  /** Mapping fidelity (0-1) */
  fidelity: number;
}

/**
 * Mapped data result
 */
export interface MappedData {
  /** Original modality */
  originalModality: SensoryModality;

  /** Target modality */
  targetModality: SensoryModality;

  /** Original data */
  originalData: SensoryData;

  /** Mapped data */
  mappedData: SensoryData;

  /** Mapping quality (0-1) */
  quality: number;

  /** Information preserved (0-1) */
  informationPreserved: number;
}

/**
 * Visual-to-auditory mapping
 */
export interface VisualAuditoryMap {
  /** Brightness to pitch */
  brightness: { target: 'pitch'; range: [number, number] };

  /** Hue to timbre */
  hue: { target: 'timbre'; values: string[] };

  /** Saturation to harmonics */
  saturation: { target: 'harmonics'; range: [number, number] };

  /** X position to pan */
  position_x: { target: 'pan'; range: [number, number] };

  /** Y position to volume */
  position_y: { target: 'volume'; range: [number, number] };

  /** Motion to tempo */
  motion: { target: 'tempo'; range: [number, number] };
}

/**
 * Visual-to-tactile mapping
 */
export interface VisualTactileMap {
  /** Brightness to vibration intensity */
  brightness: { target: 'vibration_intensity'; range: [number, number] };

  /** Edges to pulse sharpness */
  edges: { target: 'pulse_sharpness'; range: [number, number] };

  /** Texture to vibration frequency */
  texture: { target: 'vibration_frequency'; range: [number, number] };

  /** Depth to pressure */
  depth: { target: 'pressure'; range: [number, number] };

  /** Motion to vibration sweep */
  motion: { target: 'vibration_sweep'; range: [number, number] };
}

// ============================================================================
// Device and System Types
// ============================================================================

/**
 * Sensory enhancement device information
 */
export interface SensoryDevice {
  /** Device identifier */
  id: string;

  /** Device name */
  name: string;

  /** Manufacturer */
  manufacturer: string;

  /** Model */
  model: string;

  /** Supported modalities */
  modalities: SensoryModality[];

  /** Enhancement capabilities */
  capabilities: EnhancementType[];

  /** Firmware version */
  firmwareVersion: string;

  /** Calibration status */
  calibrationStatus: 'valid' | 'expired' | 'required';

  /** Last calibration date */
  lastCalibration?: Date;
}

/**
 * User profile for sensory enhancement
 */
export interface UserProfile {
  /** User identifier (anonymized) */
  id: string;

  /** Age */
  age: number;

  /** Baseline sensory capabilities */
  baselineSensory: Partial<Record<SensoryModality, SensoryRange>>;

  /** Sensory impairments */
  impairments: SensoryModality[];

  /** Adaptation history */
  adaptationHistory: AdaptationRecord[];

  /** Preferences */
  preferences: UserPreferences;
}

/**
 * Adaptation record
 */
export interface AdaptationRecord {
  /** Modality */
  modality: SensoryModality;

  /** Enhancement type */
  enhancementType: EnhancementType;

  /** Start date */
  startDate: Date;

  /** Proficiency level (0-1) */
  proficiency: number;

  /** Adaptation duration in hours */
  adaptationHours: number;
}

/**
 * User preferences
 */
export interface UserPreferences {
  /** Preferred integration mode */
  integrationMode: IntegrationMode;

  /** Auto-calibration enabled */
  autoCalibration: boolean;

  /** Overload sensitivity (0-1) */
  overloadSensitivity: number;

  /** Custom mappings */
  customMappings: CrossModalMap[];
}

// ============================================================================
// Assessment and Report Types
// ============================================================================

/**
 * Sensory assessment
 */
export interface SensoryAssessment {
  /** Assessment ID */
  id: string;

  /** User profile */
  user: UserProfile;

  /** Device information */
  device: SensoryDevice;

  /** Assessment date */
  date: Date;

  /** Modality assessments */
  modalityAssessments: ModalityAssessment[];

  /** Overall score */
  overallScore: number;

  /** Recommendations */
  recommendations: string[];

  /** Certification status */
  certified: boolean;
}

/**
 * Individual modality assessment
 */
export interface ModalityAssessment {
  /** Sensory modality */
  modality: SensoryModality;

  /** Enhancement classification */
  enhancement: EnhancementClassification;

  /** Calibration result */
  calibration: CalibrationResult;

  /** Safety status */
  safety: ProtectionStatus;

  /** Performance score (0-100) */
  performanceScore: number;
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Sensory enhancement constants
 */
export const SENSORY_CONSTANTS = {
  /** Normal sensory ranges */
  NORMAL_RANGES: {
    VISUAL: { min: 380, max: 750, unit: 'nm' },
    AUDITORY: { min: 20, max: 20000, unit: 'Hz' },
    TACTILE: { resolution: 0.2, unit: 'mm' },
  },

  /** Maximum safe enhancement factors */
  MAX_ENHANCEMENT: {
    VISUAL: 2.0,
    AUDITORY: 2.5,
    TACTILE: 10.0,
    OLFACTORY: 3.0,
    GUSTATORY: 2.0,
  },

  /** Safety margins */
  SAFETY_MARGIN: {
    MIN: 0.8,
    RECOMMENDED: 0.9,
    MAX: 0.95,
  },

  /** Synchronization requirements (ms) */
  SYNC_REQUIREMENTS: {
    VISUAL_AUDITORY: 100,
    VISUAL_TACTILE: 50,
    AUDITORY_TACTILE: 50,
    PROPRIOCEPTIVE_VESTIBULAR: 20,
  },

  /** Adaptation periods (hours) */
  ADAPTATION_TIME: {
    RESTORATION: 20,
    AUGMENTATION: 50,
    NEW_SENSE: 100,
    SUBSTITUTION: 80,
  },

  /** Overload thresholds (%) */
  OVERLOAD_THRESHOLDS: {
    WARNING: 80,
    CRITICAL: 95,
    DANGER: 100,
  },
} as const;

// ============================================================================
// Error Types
// ============================================================================

/**
 * Sensory enhancement error codes
 */
export enum SensoryErrorCode {
  INVALID_MODALITY = 'SE001',
  INVALID_RANGE = 'SE002',
  ENHANCEMENT_UNSAFE = 'SE003',
  CALIBRATION_FAILED = 'SE004',
  OVERLOAD_DETECTED = 'SE005',
  SYNC_FAILURE = 'SE006',
  MAPPING_ERROR = 'SE007',
  INCOMPATIBLE_MODALITIES = 'SE008',
}

/**
 * Sensory enhancement error class
 */
export class SensoryError extends Error {
  constructor(
    public code: SensoryErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'SensoryError';
  }
}

// ============================================================================
// Export All Types
// ============================================================================

export type {
  SensoryModality,
  StimulusType,
  SensoryRange,
  SensoryCharacteristics,
  EnhancementType,
  EnhancementLevel,
  EnhancementInput,
  EnhancementClassification,
  EnhancementParams,
  EnhancementResult,
  IntegrationMode,
  SensoryInput,
  SensoryData,
  SyncParameters,
  IntegratedPercept,
  MappingMethod,
  SensorySubstitution,
  SubstitutionResult,
  MappingFunction,
  LookupTable,
  CalibrationParams,
  CalibrationStandard,
  CalibrationResult,
  OverloadProtection,
  OverloadMonitor,
  ProtectionStatus,
  IntensityLimiter,
  CrossModalMap,
  MappedData,
  VisualAuditoryMap,
  VisualTactileMap,
  SensoryDevice,
  UserProfile,
  AdaptationRecord,
  UserPreferences,
  SensoryAssessment,
  ModalityAssessment,
};

export { SENSORY_CONSTANTS, SensoryErrorCode, SensoryError };
