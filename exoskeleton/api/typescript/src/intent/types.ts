/**
 * WIA Exoskeleton Intent Detection Types
 * @version 1.0.0
 */

// ============================================================================
// Enumerations
// ============================================================================

/** User movement intents */
export enum UserIntent {
  STAND_UP = 'stand_up',
  SIT_DOWN = 'sit_down',
  WALK_FORWARD = 'walk_forward',
  WALK_BACKWARD = 'walk_backward',
  TURN_LEFT = 'turn_left',
  TURN_RIGHT = 'turn_right',
  STOP = 'stop',
  STAIR_ASCEND = 'stair_ascend',
  STAIR_DESCEND = 'stair_descend',
  STEP_OVER = 'step_over',
  KICK = 'kick',
  BALANCE = 'balance',
  IDLE = 'idle',
}

/** Intent detection sources */
export enum IntentSource {
  EMG = 'emg',           // Electromyography
  GRF = 'grf',           // Ground Reaction Force
  IMU = 'imu',           // Inertial Measurement Unit
  BUTTON = 'button',     // Physical buttons
  BCI = 'bci',           // Brain-Computer Interface
  VOICE = 'voice',       // Voice commands
}

/** Intent detection state */
export enum IntentState {
  IDLE = 'idle',
  DETECTING = 'detecting',
  CONFIRMED = 'confirmed',
  UNCERTAIN = 'uncertain',
  REJECTED = 'rejected',
  EXECUTING = 'executing',
  COMPLETED = 'completed',
}

/** Confirmation mode for intent execution */
export enum ConfirmationMode {
  AUTO = 'auto',                   // Automatic execution
  CONFIRM_ONCE = 'confirm_once',   // Confirm first detection
  CONFIRM_ALWAYS = 'confirm_always', // Always confirm
  MANUAL_ONLY = 'manual_only',     // Manual input only
}

/** Sensor fusion methods */
export enum FusionMethod {
  WEIGHTED = 'weighted',           // Weighted average
  VOTING = 'voting',               // Majority voting
  BAYESIAN = 'bayesian',           // Bayesian fusion
  DEMPSTER_SHAFER = 'dempster_shafer', // Dempster-Shafer
}

// ============================================================================
// Vector Types
// ============================================================================

export interface Vector2D {
  x: number;
  y: number;
}

export interface Vector3D {
  x: number;
  y: number;
  z: number;
}

// ============================================================================
// EMG Types
// ============================================================================

/** EMG channel configuration */
export interface EMGChannel {
  id: number;
  muscle: string;
  location: string;
  mvcValue?: number;    // Maximum Voluntary Contraction
}

/** EMG signal features */
export interface EMGFeatures {
  /** Mean Absolute Value */
  mav: number;
  /** Root Mean Square */
  rms: number;
  /** Waveform Length */
  wl: number;
  /** Zero Crossings */
  zc: number;
  /** Slope Sign Changes */
  ssc: number;
  /** Integrated EMG */
  iemg: number;
  /** Normalized activation (% MVC) */
  activation: number;
}

/** EMG data from multiple channels */
export interface EMGData {
  timestamp: number;
  channels: Map<number, EMGFeatures>;
  rawSignals?: Map<number, number[]>;
}

/** EMG processing configuration */
export interface EMGConfig {
  sampleRate: number;
  channels: EMGChannel[];
  bandpass: { low: number; high: number };
  windowSize: number;       // ms
  overlapRatio: number;     // 0-1
  notchFilter?: number;     // Hz (e.g., 50 or 60 for powerline)
}

// ============================================================================
// GRF Types
// ============================================================================

/** Ground Reaction Force data */
export interface GRFData {
  timestamp: number;
  left: {
    fx: number;             // N (anterior-posterior)
    fy: number;             // N (medial-lateral)
    fz: number;             // N (vertical)
    cop: Vector2D;          // Center of Pressure (m)
  };
  right: {
    fx: number;
    fy: number;
    fz: number;
    cop: Vector2D;
  };
}

/** GRF features for intent detection */
export interface GRFFeatures {
  copShiftX: number;        // cm
  copShiftY: number;        // cm
  fzChange: number;         // N
  asymmetry: number;        // ratio
  loadingRate: number;      // N/s
  totalVerticalForce: number;
}

/** GRF processing configuration */
export interface GRFConfig {
  sampleRate: number;
  forceThreshold: number;   // N
  copSmoothing: number;     // window size
}

// ============================================================================
// IMU Types
// ============================================================================

/** IMU sensor data */
export interface IMUData {
  timestamp: number;
  location: string;         // pelvis, thigh_l, thigh_r, shank_l, shank_r, foot_l, foot_r
  accelerometer: Vector3D;  // m/s²
  gyroscope: Vector3D;      // deg/s
  magnetometer?: Vector3D;  // μT
  orientation?: {           // Computed orientation
    roll: number;
    pitch: number;
    yaw: number;
  };
}

/** IMU features for intent detection */
export interface IMUFeatures {
  pelvisPitch: number;      // degrees
  pelvisRoll: number;       // degrees
  pelvisYaw: number;        // degrees
  pelvisYawRate: number;    // deg/s
  trunkInclination: number; // degrees
  accelerationMagnitude: number;
  jerk: number;             // m/s³
}

/** IMU processing configuration */
export interface IMUConfig {
  sampleRate: number;
  complementaryFilterAlpha: number;
  gyroDeadband: number;     // deg/s
  locations: string[];
}

// ============================================================================
// Button Types
// ============================================================================

/** Button input data */
export interface ButtonData {
  timestamp: number;
  buttonId: string;
  pressed: boolean;
  holdDuration?: number;    // ms
}

/** Button mapping configuration */
export interface ButtonConfig {
  buttons: {
    id: string;
    intent: UserIntent;
    holdTime?: number;      // ms required hold time
  }[];
}

// ============================================================================
// Intent Detection Results
// ============================================================================

/** Result from a single source */
export interface SourceResult {
  source: IntentSource;
  intent: UserIntent;
  confidence: number;       // 0-1
  latency: number;          // ms
  rawData?: unknown;
}

/** Combined feature vector */
export interface FeatureVector {
  emgFeatures?: Record<string, EMGFeatures>;
  grfFeatures?: GRFFeatures;
  imuFeatures?: IMUFeatures;
}

/** Final intent detection result */
export interface IntentDetection {
  id: string;
  timestamp: number;
  intent: UserIntent;
  confidence: number;       // 0-1
  source: IntentSource;     // Primary source
  allSources: SourceResult[];
  features?: FeatureVector;
  state: IntentState;
}

// ============================================================================
// Configuration
// ============================================================================

/** Intent detector configuration */
export interface IntentDetectorConfig {
  enabledSources: IntentSource[];
  fusionMethod: FusionMethod;
  confidenceThreshold: number;
  detectionWindow: number;  // ms
  debounceTime: number;     // ms
  maxLatency: number;       // ms
  confirmationMode: ConfirmationMode;

  // Source-specific configs
  emgConfig?: EMGConfig;
  grfConfig?: GRFConfig;
  imuConfig?: IMUConfig;
  buttonConfig?: ButtonConfig;
}

/** Default configuration */
export const DEFAULT_INTENT_CONFIG: IntentDetectorConfig = {
  enabledSources: [IntentSource.GRF, IntentSource.IMU],
  fusionMethod: FusionMethod.WEIGHTED,
  confidenceThreshold: 0.7,
  detectionWindow: 200,
  debounceTime: 500,
  maxLatency: 300,
  confirmationMode: ConfirmationMode.AUTO,

  grfConfig: {
    sampleRate: 200,
    forceThreshold: 20,
    copSmoothing: 10,
  },

  imuConfig: {
    sampleRate: 200,
    complementaryFilterAlpha: 0.98,
    gyroDeadband: 0.5,
    locations: ['pelvis', 'thigh_l', 'thigh_r', 'shank_l', 'shank_r'],
  },
};

// ============================================================================
// Intent Thresholds
// ============================================================================

/** Thresholds for different intents */
export interface IntentThresholds {
  [UserIntent.STAND_UP]: {
    emgRectusActivation: number;
    emgGluteusActivation: number;
    grfCopShiftX: number;
    holdTime: number;
  };
  [UserIntent.SIT_DOWN]: {
    emgBicepsActivation: number;
    grfCopShiftX: number;
    holdTime: number;
  };
  [UserIntent.WALK_FORWARD]: {
    grfCopShiftX: number;
    grfFzDrop: number;
    imuPelvisPitch: number;
  };
  [UserIntent.TURN_LEFT]: {
    imuPelvisYawRate: number;
    grfCopShiftY: number;
  };
  [UserIntent.TURN_RIGHT]: {
    imuPelvisYawRate: number;
    grfCopShiftY: number;
  };
  // ... other intents
}

export const DEFAULT_THRESHOLDS: Partial<IntentThresholds> = {
  [UserIntent.STAND_UP]: {
    emgRectusActivation: 0.30,
    emgGluteusActivation: 0.30,
    grfCopShiftX: 3.0,
    holdTime: 200,
  },
  [UserIntent.SIT_DOWN]: {
    emgBicepsActivation: 0.20,
    grfCopShiftX: -3.0,
    holdTime: 200,
  },
  [UserIntent.WALK_FORWARD]: {
    grfCopShiftX: 3.0,
    grfFzDrop: 20,
    imuPelvisPitch: 5.0,
  },
  [UserIntent.TURN_LEFT]: {
    imuPelvisYawRate: -10.0,
    grfCopShiftY: 5.0,
  },
  [UserIntent.TURN_RIGHT]: {
    imuPelvisYawRate: 10.0,
    grfCopShiftY: -5.0,
  },
};

// ============================================================================
// Calibration
// ============================================================================

/** Calibration data for a user */
export interface CalibrationData {
  userId: string;
  timestamp: number;
  emgBaseline?: Record<number, number>;
  emgMVC?: Record<number, number>;
  intentThresholds?: Partial<IntentThresholds>;
  modelWeights?: number[];
}

/** Calibration session state */
export interface CalibrationState {
  step: 'rest' | 'mvc' | 'rehearsal' | 'validation' | 'complete';
  progress: number;         // 0-100%
  currentMuscle?: string;
  currentIntent?: UserIntent;
  trials: number;
  successRate?: number;
}
