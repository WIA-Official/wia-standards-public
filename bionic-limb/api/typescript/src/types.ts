/**
 * WIA-AUG-007: Bionic Limb - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Human Augmentation Bionics Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Limb Classification Types
// ============================================================================

/**
 * Bionic limb types
 */
export type LimbType =
  | 'UPPER_ARM'
  | 'FOREARM'
  | 'HAND'
  | 'FINGER'
  | 'THIGH'
  | 'LOWER_LEG'
  | 'FOOT';

/**
 * Limb category based on complexity
 */
export type LimbCategory = 'Minimal' | 'Basic' | 'Moderate' | 'Advanced';

/**
 * Upper or lower limb
 */
export type LimbLocation = 'UPPER' | 'LOWER';

/**
 * Limb classification input
 */
export interface LimbClassificationInput {
  /** Type of limb */
  type: LimbType;

  /** Degrees of freedom */
  dof: number;

  /** Control method complexity (1-10) */
  controlComplexity: number;

  /** Number of sensors */
  sensorCount?: number;

  /** Control methods available */
  controlMethods?: number;
}

/**
 * Limb classification result
 */
export interface LimbClassification {
  /** Limb type */
  type: LimbType;

  /** Upper or lower limb */
  location: LimbLocation;

  /** Amputation level */
  amputationLevel: string;

  /** Residual limb length (cm) */
  residualLength?: number;

  /** Degrees of freedom */
  dof: number;

  /** Complexity category */
  category: LimbCategory;

  /** Complexity score */
  complexityScore: number;

  /** Recommended control methods */
  recommendedControls: ControlMethod[];
}

// ============================================================================
// Control System Types
// ============================================================================

/**
 * Control methods for bionic limbs
 */
export type ControlMethod =
  | 'MYOELECTRIC'
  | 'NEURAL_DIRECT'
  | 'PATTERN_RECOGNITION'
  | 'HYBRID'
  | 'BODY_POWERED';

/**
 * Control mode
 */
export type ControlMode =
  | 'DIRECT'
  | 'SEQUENTIAL'
  | 'PROPORTIONAL'
  | 'SIMULTANEOUS'
  | 'ADAPTIVE';

/**
 * Myoelectric control configuration
 */
export interface MyoelectricControl {
  /** Number of EMG electrodes */
  electrodes: number;

  /** Sampling rate (Hz) */
  samplingRate: number;

  /** Filter bandpass settings */
  filterBandpass: { low: number; high: number };

  /** Detection threshold (μV) */
  threshold: number;

  /** Maximum processing delay (ms) */
  processingDelay: number;
}

/**
 * Pattern recognition configuration
 */
export interface PatternRecognitionConfig {
  /** ML algorithm */
  algorithm: 'LDA' | 'SVM' | 'CNN' | 'LSTM';

  /** Feature types */
  features: FeatureType[];

  /** Number of trained gestures */
  gestureCount: number;

  /** Training accuracy */
  accuracy: number;

  /** Last training date */
  lastTrained: Date;
}

/**
 * Feature types for pattern recognition
 */
export type FeatureType = 'RMS' | 'MAV' | 'WL' | 'ZC' | 'SSC' | 'WFL' | 'HIST';

/**
 * Control calibration data
 */
export interface ControlCalibration {
  /** Limb identifier */
  limbId: string;

  /** User identifier */
  userId: string;

  /** Control method */
  method: ControlMethod;

  /** Calibration timestamp */
  timestamp: Date;

  /** Calibration accuracy (0-1) */
  accuracy: number;

  /** Training data points */
  trainingPoints: number;

  /** Valid until */
  validUntil: Date;

  /** Next calibration recommended */
  nextCalibration: Date;
}

// ============================================================================
// Sensory Feedback Types
// ============================================================================

/**
 * Sensory feedback types
 */
export type FeedbackType = 'PRESSURE' | 'TEMPERATURE' | 'POSITION' | 'VIBRATION' | 'SLIP';

/**
 * Feedback encoding method
 */
export type FeedbackEncoding = 'amplitude' | 'frequency' | 'spatial' | 'temporal';

/**
 * Pressure feedback configuration
 */
export interface PressureFeedback {
  /** Sensor locations */
  sensors: SensorLocation[];

  /** Feedback delivery method */
  feedbackType: 'vibration' | 'electrical' | 'mechanical';

  /** Intensity (0-1) */
  intensity: number;

  /** Active location */
  location: SensorLocation;

  /** Calibration data */
  calibration: FeedbackCalibration;
}

/**
 * Temperature feedback configuration
 */
export interface TemperatureFeedback {
  /** Temperature range */
  range: { min: number; max: number };

  /** Accuracy (degrees) */
  accuracy: number;

  /** Response time (ms) */
  responseTime: number;

  /** Safety cutoff enabled */
  safetyCutoff: boolean;
}

/**
 * Position feedback (proprioception) configuration
 */
export interface PositionFeedback {
  /** Joint being tracked */
  joint: string;

  /** Angle accuracy (degrees) */
  angleAccuracy: number;

  /** Update rate (Hz) */
  updateRate: number;

  /** Encoding method */
  encoding: FeedbackEncoding;
}

/**
 * Sensor location
 */
export type SensorLocation =
  | 'palm'
  | 'thumb'
  | 'index'
  | 'middle'
  | 'ring'
  | 'pinky'
  | 'wrist'
  | 'forearm'
  | 'heel'
  | 'toe'
  | 'ankle'
  | 'knee';

/**
 * Feedback calibration data
 */
export interface FeedbackCalibration {
  /** Minimum detectable value */
  minValue: number;

  /** Maximum value */
  maxValue: number;

  /** Calibration curve points */
  calibrationPoints: Array<{ input: number; output: number }>;

  /** Last calibrated */
  lastCalibrated: Date;
}

/**
 * Multi-modal feedback configuration
 */
export interface MultiModalFeedback {
  /** Pressure feedback config */
  pressure?: PressureFeedback;

  /** Temperature feedback config */
  temperature?: TemperatureFeedback;

  /** Position feedback config */
  position?: PositionFeedback;

  /** Priority order */
  priority: FeedbackType[];

  /** Fusion algorithm */
  fusionAlgorithm: 'weighted' | 'priority' | 'adaptive';
}

// ============================================================================
// Grip Pattern Types
// ============================================================================

/**
 * Power grip patterns
 */
export type PowerGrip = 'POWER_GRIP' | 'SPHERICAL_GRIP' | 'HOOK_GRIP' | 'LATERAL_GRIP';

/**
 * Precision grip patterns
 */
export type PrecisionGrip =
  | 'PRECISION_GRIP'
  | 'LATERAL_PINCH'
  | 'TRIPOD_PINCH'
  | 'TIP_PINCH';

/**
 * All grip patterns
 */
export type GripPattern = PowerGrip | PrecisionGrip;

/**
 * Grip configuration
 */
export interface GripConfig {
  /** Grip pattern */
  pattern: GripPattern;

  /** Grip force (Newtons) */
  force: number;

  /** Speed (0-1) */
  speed: number;

  /** Precision mode */
  precision: boolean;
}

/**
 * Grip pattern definition
 */
export interface GripPatternDefinition {
  /** Pattern identifier */
  id: string;

  /** Pattern name */
  name: string;

  /** Finger positions */
  fingerPositions: {
    thumb?: { flexion: number; abduction: number };
    index?: { flexion: number; abduction: number };
    middle?: { flexion: number; abduction: number };
    ring?: { flexion: number; abduction: number };
    pinky?: { flexion: number; abduction: number };
  };

  /** Force range */
  forceRange: { min: number; max: number };

  /** Typical applications */
  applications: string[];
}

// ============================================================================
// Gait Analysis Types (Lower Limbs)
// ============================================================================

/**
 * Gait cycle phase
 */
export type GaitPhase =
  | 'initial_contact'
  | 'loading_response'
  | 'mid_stance'
  | 'terminal_stance'
  | 'pre_swing'
  | 'initial_swing'
  | 'mid_swing'
  | 'terminal_swing';

/**
 * Spatiotemporal gait parameters
 */
export interface SpatiotemporalParameters {
  /** Stride length (cm) */
  strideLength: number;

  /** Step length (cm) */
  stepLength: number;

  /** Step width (cm) */
  stepWidth: number;

  /** Cadence (steps/min) */
  cadence: number;

  /** Walking velocity (m/s) */
  velocity: number;

  /** Stance time (% of cycle) */
  stanceTime: number;

  /** Swing time (% of cycle) */
  swingTime: number;

  /** Double support (% of cycle) */
  doubleSupport: number;
}

/**
 * Angle range
 */
export interface AngleRange {
  /** Minimum angle (degrees) */
  min: number;

  /** Maximum angle (degrees) */
  max: number;

  /** Mean angle (degrees) */
  mean: number;
}

/**
 * Kinematic gait parameters
 */
export interface KinematicParameters {
  /** Hip flexion range */
  hipFlexion: AngleRange;

  /** Knee flexion range */
  kneeFlexion: AngleRange;

  /** Ankle dorsiflexion range */
  ankleDorsiflexion: AngleRange;

  /** Pelvic tilt range */
  pelvicTilt: AngleRange;
}

/**
 * Gait symmetry metrics
 */
export interface SymmetryMetrics {
  /** Spatial symmetry (0-1) */
  spatialSymmetry: number;

  /** Temporal symmetry (0-1) */
  temporalSymmetry: number;

  /** Force symmetry (0-1) */
  forceSymmetry: number;

  /** Overall symmetry (0-1) */
  overallSymmetry: number;
}

/**
 * Gait analysis result
 */
export interface GaitAnalysis {
  /** Analysis timestamp */
  timestamp: Date;

  /** Duration analyzed (seconds) */
  duration: number;

  /** Spatiotemporal parameters */
  spatiotemporal: SpatiotemporalParameters;

  /** Kinematic parameters */
  kinematics: KinematicParameters;

  /** Symmetry metrics */
  symmetry: SymmetryMetrics;

  /** Gait efficiency (0-1) */
  efficiency: number;

  /** Gait stability (0-1) */
  stability: number;

  /** Detected anomalies */
  anomalies: string[];
}

/**
 * Terrain type
 */
export type TerrainType = 'level' | 'incline' | 'decline' | 'stairs_up' | 'stairs_down' | 'uneven';

/**
 * Microprocessor knee control
 */
export interface KneeControl {
  /** Current mode */
  mode: 'stance' | 'swing' | 'transition';

  /** Resistance level (0-1) */
  resistance: number;

  /** Current flexion angle (degrees) */
  flexionAngle: number;

  /** Extension stop angle (degrees) */
  extensionStop: number;

  /** Swing flexion target (degrees) */
  swingFlexion: number;

  /** Current terrain */
  terrain: TerrainType;
}

// ============================================================================
// Power and Battery Types
// ============================================================================

/**
 * Battery type
 */
export type BatteryType = 'LiPo' | 'Li-ion' | 'LiFePO4';

/**
 * Battery specification
 */
export interface BatterySpecification {
  /** Battery chemistry */
  type: BatteryType;

  /** Capacity (mAh) */
  capacity: number;

  /** Voltage (V) */
  voltage: number;

  /** Weight (grams) */
  weight: number;

  /** Expected charge cycles */
  cycles: number;

  /** Runtime on full charge (hours) */
  runtime: number;

  /** Charging time (hours) */
  chargingTime: number;

  /** Safety features */
  safetyFeatures: string[];
}

/**
 * Battery state
 */
export interface BatteryState {
  /** Current charge level (0-100%) */
  level: number;

  /** Voltage */
  voltage: number;

  /** Temperature (Celsius) */
  temperature: number;

  /** Charging status */
  charging: boolean;

  /** Cycle count */
  cycleCount: number;

  /** Estimated runtime (minutes) */
  estimatedRuntime: number;

  /** Health status */
  health: 'excellent' | 'good' | 'fair' | 'replace';
}

/**
 * Power management mode
 */
export type PowerMode = 'performance' | 'balanced' | 'economy';

/**
 * Power management configuration
 */
export interface PowerManagement {
  /** Current mode */
  mode: PowerMode;

  /** Battery level (0-100%) */
  batteryLevel: number;

  /** Estimated runtime (minutes) */
  estimatedRuntime: number;

  /** Power saving enabled */
  powerSaving: boolean;
}

// ============================================================================
// Maintenance Types
// ============================================================================

/**
 * Maintenance interval
 */
export type MaintenanceInterval = 'daily' | 'weekly' | 'monthly' | 'quarterly' | 'annual';

/**
 * Maintenance task
 */
export interface MaintenanceTask {
  /** Task identifier */
  id: string;

  /** Task name */
  name: string;

  /** Task type */
  type: 'inspection' | 'calibration' | 'cleaning' | 'replacement' | 'service';

  /** Frequency */
  interval: MaintenanceInterval;

  /** Estimated duration (minutes) */
  duration: number;

  /** Required by */
  requiredBy: 'user' | 'technician' | 'specialist';

  /** Last performed */
  lastPerformed?: Date;

  /** Next due */
  nextDue: Date;

  /** Status */
  status: 'pending' | 'completed' | 'overdue';
}

/**
 * Maintenance schedule
 */
export interface MaintenanceSchedule {
  /** Limb identifier */
  limbId: string;

  /** Scheduled tasks */
  tasks: MaintenanceTask[];

  /** Overdue count */
  overdueCount: number;

  /** Next maintenance date */
  nextMaintenance: Date;
}

// ============================================================================
// Device Information Types
// ============================================================================

/**
 * Bionic limb device information
 */
export interface BionicLimbDevice {
  /** Device identifier */
  id: string;

  /** Limb type */
  type: LimbType;

  /** Manufacturer */
  manufacturer: string;

  /** Model name */
  model: string;

  /** Serial number */
  serialNumber: string;

  /** Firmware version */
  firmwareVersion: string;

  /** Manufacturing date */
  manufactureDate: Date;

  /** Installation date */
  installDate?: Date;

  /** Control method */
  controlMethod: ControlMethod;

  /** Degrees of freedom */
  dof: number;

  /** Sensory feedback types */
  feedbackTypes: FeedbackType[];

  /** Battery specification */
  battery: BatterySpecification;
}

/**
 * Patient/user information (anonymized)
 */
export interface PatientInfo {
  /** Patient identifier (anonymized) */
  id: string;

  /** Age */
  age: number;

  /** Amputation level */
  amputationLevel: string;

  /** Residual limb length (cm) */
  residualLength: number;

  /** Residual limb muscle quality */
  muscleQuality: 'excellent' | 'good' | 'fair' | 'poor';

  /** Time since amputation (months) */
  timeSinceAmputation: number;

  /** Cause of amputation */
  cause?: 'trauma' | 'disease' | 'congenital' | 'other';

  /** Activity level */
  activityLevel: 'sedentary' | 'moderate' | 'active' | 'very_active';

  /** Primary activities */
  primaryActivities: string[];
}

// ============================================================================
// Assessment and Report Types
// ============================================================================

/**
 * Limb compatibility assessment
 */
export interface CompatibilityAssessment {
  /** Overall compatibility score (0-100) */
  compatibility: number;

  /** Recommended control method */
  recommendedControl: ControlMethod;

  /** Expected functionality score (0-100) */
  functionalityScore: number;

  /** Training time estimate (hours) */
  estimatedTrainingTime: number;

  /** Recommendations */
  recommendations: string[];

  /** Potential challenges */
  challenges: string[];
}

/**
 * Dexterity assessment
 */
export interface DexterityAssessment {
  /** Number of available grip patterns */
  gripPatterns: number;

  /** Average transition time (ms) */
  transitionTime: number;

  /** Force resolution (N) */
  forceResolution: number;

  /** Independent fingers/DOF */
  independentFingers: number;

  /** Overall manipulation score (0-100) */
  manipulationScore: number;
}

/**
 * Comprehensive limb report
 */
export interface LimbReport {
  /** Report identifier */
  id: string;

  /** Generation timestamp */
  generatedAt: Date;

  /** Device information */
  device: BionicLimbDevice;

  /** Patient information */
  patient: PatientInfo;

  /** Classification */
  classification: LimbClassification;

  /** Current calibration */
  calibration: ControlCalibration;

  /** Battery state */
  battery: BatteryState;

  /** Maintenance schedule */
  maintenance: MaintenanceSchedule;

  /** Performance metrics */
  performance: {
    controlAccuracy: number;
    responseTime: number;
    batteryRuntime: number;
    functionalityScore: number;
  };

  /** Gait analysis (if lower limb) */
  gaitAnalysis?: GaitAnalysis;

  /** Dexterity assessment (if upper limb) */
  dexterity?: DexterityAssessment;

  /** Overall status */
  status: 'excellent' | 'good' | 'fair' | 'needs_service';

  /** Recommendations */
  recommendations: string[];
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Bionic limb constants
 */
export const BIONIC_LIMB_CONSTANTS = {
  /** Force specifications (Newtons) */
  FORCE: {
    MIN: 5,
    MAX: 200,
    RESOLUTION: 1,
  },

  /** Battery specifications */
  BATTERY: {
    MIN_RUNTIME_HOURS: 8,
    MIN_CAPACITY_MAH: 1500,
    CRITICAL_LEVEL: 20,
    WARNING_LEVEL: 30,
  },

  /** Performance thresholds */
  PERFORMANCE: {
    MIN_CONTROL_ACCURACY: 0.85,
    TARGET_CONTROL_ACCURACY: 0.95,
    MAX_RESPONSE_TIME_MS: 300,
    TARGET_RESPONSE_TIME_MS: 150,
  },

  /** DOF ranges */
  DOF: {
    FINGER: { min: 1, max: 3 },
    HAND: { min: 3, max: 6 },
    FOREARM: { min: 6, max: 12 },
    UPPER_ARM: { min: 12, max: 18 },
    FOOT: { min: 2, max: 4 },
    LOWER_LEG: { min: 4, max: 8 },
    THIGH: { min: 8, max: 12 },
  },

  /** Gait normal ranges */
  GAIT: {
    STRIDE_LENGTH_CM: { min: 120, max: 145 },
    CADENCE_SPM: { min: 100, max: 115 },
    VELOCITY_MS: { min: 1.0, max: 1.3 },
    STANCE_PERCENT: { min: 58, max: 62 },
    SWING_PERCENT: { min: 38, max: 42 },
  },
} as const;

// ============================================================================
// Error Types
// ============================================================================

/**
 * Bionic limb error codes
 */
export enum BionicLimbErrorCode {
  CLASSIFICATION_INVALID_INPUT = 'BL001',
  CALIBRATION_FAILED = 'BL002',
  CONTROL_UNRESPONSIVE = 'BL003',
  FEEDBACK_FAILED = 'BL004',
  BATTERY_CRITICAL = 'BL005',
  SENSOR_MALFUNCTION = 'BL006',
  GRIP_FAILURE = 'BL007',
  MAINTENANCE_OVERDUE = 'BL008',
  SAFETY_LIMIT_EXCEEDED = 'BL009',
}

/**
 * Bionic limb error class
 */
export class BionicLimbError extends Error {
  constructor(
    public code: BionicLimbErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'BionicLimbError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  LimbType,
  LimbCategory,
  LimbLocation,
  LimbClassificationInput,
  LimbClassification,
  ControlMethod,
  ControlMode,
  MyoelectricControl,
  PatternRecognitionConfig,
  FeatureType,
  ControlCalibration,
  FeedbackType,
  FeedbackEncoding,
  PressureFeedback,
  TemperatureFeedback,
  PositionFeedback,
  SensorLocation,
  FeedbackCalibration,
  MultiModalFeedback,
  PowerGrip,
  PrecisionGrip,
  GripPattern,
  GripConfig,
  GripPatternDefinition,
  GaitPhase,
  SpatiotemporalParameters,
  AngleRange,
  KinematicParameters,
  SymmetryMetrics,
  GaitAnalysis,
  TerrainType,
  KneeControl,
  BatteryType,
  BatterySpecification,
  BatteryState,
  PowerMode,
  PowerManagement,
  MaintenanceInterval,
  MaintenanceTask,
  MaintenanceSchedule,
  BionicLimbDevice,
  PatientInfo,
  CompatibilityAssessment,
  DexterityAssessment,
  LimbReport,
};

export { BIONIC_LIMB_CONSTANTS, BionicLimbErrorCode, BionicLimbError };
