/**
 * WIA-AUG-014: Human-Machine Interface - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Human Augmentation Interface Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Signal Types
// ============================================================================

/**
 * Signal type classification
 */
export enum SignalType {
  // Neural (Type A)
  EEG = 'A1',
  ECoG = 'A2',
  LFP = 'A3',
  SINGLE_UNIT = 'A4',

  // Muscular (Type B)
  SURFACE_EMG = 'B1',
  INTRAMUSCULAR_EMG = 'B2',
  MMG = 'B3',

  // Biometric (Type C)
  ECG = 'C1',
  GSR = 'C2',
  RESPIRATION = 'C3',
  TEMPERATURE = 'C4',

  // Motion (Type D)
  ACCELEROMETER = 'D1',
  GYROSCOPE = 'D2',
  MAGNETOMETER = 'D3',
  FORCE = 'D4',

  // Feedback (Type E)
  HAPTIC = 'E1',
  THERMAL = 'E2',
  ELECTRICAL_STIM = 'E3',
  VISUAL_AUDIO = 'E4',
}

/**
 * Three-dimensional vector
 */
export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

/**
 * Event marker for signal synchronization
 */
export interface EventMarker {
  /** Marker timestamp (microseconds) */
  timestamp: bigint;

  /** Marker type */
  type: string;

  /** Marker value */
  value: number;

  /** Optional label */
  label?: string;
}

/**
 * Signal packet for data transmission
 */
export interface SignalPacket {
  header: {
    /** Protocol version (1-255) */
    version: number;

    /** Signal type */
    signalType: SignalType;

    /** Timestamp in microseconds since epoch */
    timestamp: bigint;

    /** Number of channels */
    channelCount: number;

    /** Samples per channel */
    sampleCount: number;

    /** Bits per sample */
    resolution: number;
  };
  metadata: {
    /** Source device UUID */
    deviceId: string;

    /** Current session UUID */
    sessionId: string;

    /** Packet sequence number */
    sequenceNumber: number;

    /** Signal quality (0-100) */
    quality: number;
  };
  payload: {
    /** Interleaved channel data */
    samples: Int32Array;

    /** Synchronization markers */
    markers: EventMarker[];

    /** CRC-32 checksum */
    checksum: number;
  };
}

// ============================================================================
// Protocol Types
// ============================================================================

/**
 * Transport packet flags
 */
export interface TransportFlags {
  /** Connection initiation */
  SYN: boolean;

  /** Acknowledgment */
  ACK: boolean;

  /** Connection termination */
  FIN: boolean;

  /** Reset */
  RST: boolean;

  /** Priority */
  PRI: boolean;

  /** Reliable delivery required */
  REL: boolean;
}

/**
 * Transport layer packet
 */
export interface TransportPacket {
  /** 32-bit sequence number */
  sequenceNumber: number;

  /** Acknowledgment number */
  acknowledgmentNumber: number;

  /** Packet flags */
  flags: TransportFlags;

  /** Flow control window */
  window: number;

  /** 16-bit checksum */
  checksum: number;

  /** Priority data pointer */
  urgentPointer: number;

  /** Payload data */
  payload: Uint8Array;
}

// ============================================================================
// Latency Types
// ============================================================================

/**
 * Latency tier classification
 */
export type LatencyTier = 1 | 2 | 3 | 4;

/**
 * Latency measurement components
 */
export interface LatencyComponents {
  /** Signal acquisition time (ms) */
  acquisition: number;

  /** Processing time (ms) */
  processing: number;

  /** Encoding time (ms) */
  encoding: number;

  /** Transmission time (ms) */
  transmission: number;

  /** Decoding time (ms) */
  decoding: number;

  /** Execution time (ms) */
  execution: number;
}

/**
 * Latency measurement result
 */
export interface LatencyMeasurement {
  /** Measurement identifier */
  measurementId: string;

  /** Target latency tier */
  tier: LatencyTier;

  /** Component breakdown */
  components: LatencyComponents;

  /** Total latency (ms) */
  total: number;

  /** Jitter (standard deviation in ms) */
  jitter: number;

  /** Whether tier requirement was met */
  compliance: boolean;

  /** Measurement timestamp */
  timestamp: Date;
}

// ============================================================================
// Intent and Command Types
// ============================================================================

/**
 * Intent categories
 */
export type IntentCategory = 'motor' | 'sensory' | 'cognitive' | 'communication';

/**
 * Decoded user intent
 */
export interface Intent {
  /** Intent category */
  category: IntentCategory;

  /** Specific action (e.g., 'grip_close', 'walk_forward') */
  action: string;

  /** Intent parameters */
  parameters: {
    /** Intensity (0-1) */
    intensity: number;

    /** Speed (0-1) */
    speed: number;

    /** Precision (0-1) */
    precision: number;

    /** Duration in ms (optional) */
    duration?: number;

    /** Spatial target (optional) */
    target?: Vector3;
  };

  /** Confidence level (0-1) */
  confidence: number;

  /** Intent timestamp */
  timestamp: bigint;

  /** Signal source ID */
  source: string;
}

/**
 * Motor command types
 */
export type CommandType = 'position' | 'velocity' | 'force' | 'impedance';

/**
 * Joint-level command
 */
export interface JointCommand {
  /** Joint identifier */
  jointId: number;

  /** Target value (position/velocity/force) */
  target: number;

  /** Motion limits */
  limits: {
    min: number;
    max: number;
    maxVelocity: number;
    maxAcceleration: number;
  };

  /** Compliance level (0 = rigid, 1 = compliant) */
  compliance: number;
}

/**
 * Motor command to device
 */
export interface MotorCommand {
  /** Target device ID */
  deviceId: string;

  /** Command type */
  commandType: CommandType;

  /** Joint commands */
  joints: JointCommand[];

  /** Synchronization parameters */
  synchronization: {
    /** Command timestamp */
    timestamp: bigint;

    /** Execution deadline */
    deadline: bigint;

    /** Command priority */
    priority: number;
  };
}

/**
 * Command execution result
 */
export interface CommandResult {
  /** Whether command was executed successfully */
  success: boolean;

  /** Execution time (ms) */
  executionTime: number;

  /** Achieved values (if measurable) */
  achieved?: {
    joints: number[];
    error: number[];
  };

  /** Error message (if failed) */
  error?: string;
}

// ============================================================================
// Feedback Types
// ============================================================================

/**
 * Feedback modality types
 */
export type FeedbackType = 'haptic' | 'thermal' | 'electrotactile' | 'visual' | 'audio';

/**
 * Haptic pattern segment
 */
export interface HapticSegment {
  /** Intensity (0-1) */
  intensity: number;

  /** Duration (ms) */
  duration: number;

  /** Ramp-up time (ms) */
  rampUp?: number;

  /** Ramp-down time (ms) */
  rampDown?: number;
}

/**
 * Haptic pattern definition
 */
export interface HapticPattern {
  /** Pattern name */
  name: string;

  /** Pattern segments */
  segments: HapticSegment[];

  /** Number of repetitions */
  repeatCount: number;
}

/**
 * Body location for feedback
 */
export interface BodyLocation {
  /** Body region */
  region: string;

  /** Location coordinates (if applicable) */
  coordinates?: Vector3;

  /** Channel/actuator ID */
  channel?: number;
}

/**
 * Haptic feedback command
 */
export interface HapticFeedback {
  /** Feedback type */
  type: 'vibration' | 'force' | 'electrotactile' | 'thermal';

  /** Feedback pattern */
  pattern: HapticPattern;

  /** Target locations */
  location: BodyLocation[];

  /** Overall intensity (0-1) */
  intensity: number;

  /** Duration (ms) */
  duration: number;

  /** Frequency in Hz (for vibration) */
  frequency?: number;

  /** Custom waveform data */
  waveform?: number[];
}

/**
 * Neural feedback parameters
 */
export interface NeuralFeedback {
  /** Stimulation type */
  type: 'cortical' | 'peripheral' | 'spinal';

  /** Target location */
  target: {
    /** Brain region or nerve name */
    location: string;

    /** Electrode channels */
    channels: number[];
  };

  /** Stimulation parameters */
  stimulation: {
    /** Amplitude (μA) */
    amplitude: number;

    /** Pulse width (μs) */
    pulseWidth: number;

    /** Frequency (Hz) */
    frequency: number;

    /** Duration (ms) */
    duration: number;

    /** Waveform type */
    waveform: 'biphasic' | 'monophasic' | 'asymmetric';
  };

  /** Safety limits */
  safety: {
    /** Maximum charge per phase (μC) */
    maxCharge: number;

    /** Maximum frequency (Hz) */
    maxFrequency: number;

    /** Duty cycle (0-1) */
    dutyCycle: number;
  };
}

// ============================================================================
// Calibration Types
// ============================================================================

/**
 * Calibration types
 */
export type CalibrationType = 'initial' | 'session' | 'adaptive' | 'recalibration';

/**
 * Calibration task definition
 */
export interface CalibrationTask {
  /** Task identifier */
  taskId: string;

  /** Task name */
  name: string;

  /** Task description */
  description: string;

  /** Expected duration in seconds */
  duration: number;

  /** User instructions */
  instructions: string[];

  /** Required signal types */
  requiredSignals: SignalType[];

  /** Number of repetitions */
  repetitions: number;

  /** Rest period between repetitions (seconds) */
  restPeriod: number;
}

/**
 * Channel quality metrics
 */
export interface ChannelQuality {
  /** Channel ID */
  channelId: number;

  /** Signal-to-noise ratio (dB) */
  snr: number;

  /** Impedance (Ohms) */
  impedance: number;

  /** Stability score (0-1) */
  stability: number;

  /** Status */
  status: 'good' | 'acceptable' | 'poor' | 'failed';
}

/**
 * Calibration model (trained decoder)
 */
export interface CalibrationModel {
  /** Model identifier */
  id: string;

  /** Model type */
  type: string;

  /** Model version */
  version: string;

  /** Training date */
  trainedAt: Date;

  /** Training accuracy */
  accuracy: number;

  /** Model parameters (serialized) */
  parameters: ArrayBuffer;
}

/**
 * Calibration result
 */
export interface CalibrationResult {
  /** Whether calibration succeeded */
  success: boolean;

  /** Overall accuracy (0-100%) */
  accuracy: number;

  /** Signal quality metrics */
  signalQuality: {
    /** Signal-to-noise ratio (dB) */
    snr: number;

    /** Stability score (0-1) */
    stability: number;

    /** Per-channel quality */
    channels: ChannelQuality[];
  };

  /** Trained model */
  model: CalibrationModel;

  /** Recommendations */
  recommendations: string[];
}

/**
 * Calibration session
 */
export interface CalibrationSession {
  /** Session identifier */
  sessionId: string;

  /** Calibration type */
  type: CalibrationType;

  /** Session start time */
  startTime: Date;

  /** Session end time (if completed) */
  endTime?: Date;

  /** Session status */
  status: 'in_progress' | 'completed' | 'failed' | 'aborted';

  /** Calibration tasks */
  tasks: CalibrationTask[];

  /** Calibration result (if completed) */
  results?: CalibrationResult;
}

// ============================================================================
// Device and Connection Types
// ============================================================================

/**
 * Device capabilities
 */
export interface DeviceCapabilities {
  /** Supported signal types */
  signalTypes: SignalType[];

  /** Supported feedback types */
  feedbackTypes: FeedbackType[];

  /** Latency tier capability */
  latencyTier: LatencyTier;

  /** Number of channels */
  channels: number;

  /** Supported sample rates */
  sampleRates: number[];
}

/**
 * Device advertisement for discovery
 */
export interface DeviceAdvertisement {
  /** Device UUID */
  deviceId: string;

  /** Device type (e.g., 'prosthetic_arm') */
  deviceType: string;

  /** Manufacturer name */
  manufacturer: string;

  /** Model name */
  model: string;

  /** Firmware version */
  version: string;

  /** Device capabilities */
  capabilities: DeviceCapabilities;

  /** Supported protocols */
  protocols: {
    hmi: string;
    security: string;
    safety: string;
  };

  /** Device status */
  status: 'available' | 'connected' | 'busy' | 'error';
}

/**
 * Device state information
 */
export interface DeviceState {
  /** Device identifier */
  deviceId: string;

  /** State timestamp */
  timestamp: bigint;

  /** Power status */
  power: {
    /** Battery level (0-100%) */
    batteryLevel: number;

    /** Whether device is charging */
    charging: boolean;

    /** Estimated runtime (minutes) */
    estimatedRuntime: number;
  };

  /** Position information */
  position: {
    /** Current joint angles */
    joints: number[];

    /** End effector position */
    endEffector: Vector3;
  };

  /** Sensor readings */
  sensors: {
    /** Force sensor values */
    force: number[];

    /** Temperature readings */
    temperature: number[];

    /** Contact sensor states */
    contact: boolean[];
  };

  /** Overall device status */
  status: 'ready' | 'active' | 'error' | 'calibrating' | 'sleeping';
}

/**
 * Connection configuration
 */
export interface ConnectionConfig {
  /** Target device ID */
  deviceId: string;

  /** Interface type */
  interface: 'neural' | 'emg' | 'hybrid';

  /** Signal configuration */
  signalConfig: {
    /** Sampling rate (Hz) */
    samplingRate: number;

    /** Number of channels */
    channels: number;

    /** Resolution (bits) */
    resolution: number;
  };

  /** Latency target */
  latencyTarget?: number;

  /** Auto-reconnect enabled */
  autoReconnect?: boolean;
}

/**
 * Active connection
 */
export interface Connection {
  /** Connection identifier */
  id: string;

  /** Device information */
  device: DeviceAdvertisement;

  /** Connection configuration */
  config: ConnectionConfig;

  /** Connection status */
  status: 'connecting' | 'connected' | 'disconnecting' | 'disconnected' | 'error';

  /** Connection established time */
  connectedAt?: Date;

  /** Current latency (ms) */
  latency?: number;

  /** Signal quality (0-100%) */
  signalQuality?: number;
}

// ============================================================================
// User Profile Types
// ============================================================================

/**
 * Feedback preferences
 */
export interface FeedbackPreferences {
  /** Haptic intensity preference (0-1) */
  hapticIntensity: number;

  /** Preferred feedback modalities */
  preferredModalities: FeedbackType[];

  /** Sensitivity settings */
  sensitivity: {
    pressure: number;
    temperature: number;
    vibration: number;
  };
}

/**
 * User profile for device personalization
 */
export interface UserProfile {
  /** Profile identifier */
  profileId: string;

  /** User identifier */
  userId: string;

  /** Profile creation date */
  created: Date;

  /** Last update date */
  updated: Date;

  /** Calibration data */
  calibration: {
    /** Signal baselines */
    signalBaselines: Map<string, number[]>;

    /** Decoder models */
    decoderModels: Map<string, CalibrationModel>;

    /** Feedback preferences */
    feedbackPreferences: FeedbackPreferences;
  };

  /** User preferences */
  preferences: {
    /** Latency vs quality priority */
    latencyPriority: 'low' | 'balanced' | 'quality';

    /** Overall feedback intensity */
    feedbackIntensity: number;

    /** Enable adaptive calibration */
    adaptiveMode: boolean;
  };

  /** Usage history */
  history: {
    /** Previously used devices */
    devices: string[];

    /** Total sessions */
    sessions: number;

    /** Total usage hours */
    totalUsageHours: number;
  };
}

// ============================================================================
// Quality Metrics Types
// ============================================================================

/**
 * Quality metrics for monitoring
 */
export interface QualityMetrics {
  /** Signal-to-noise ratio (dB) */
  signalToNoise: number;

  /** Channel correlation (0-1, target < 0.3) */
  channelCorrelation: number;

  /** Baseline stability (variance, target < 10%) */
  baselineStability: number;

  /** Artifact rate (per minute, target < 5) */
  artifactRate: number;

  /** Decoding accuracy (0-100%) */
  decodingAccuracy: number;

  /** Latency compliance (0-100%) */
  latencyCompliance: number;

  /** Feedback accuracy (0-100%) */
  feedbackAccuracy: number;
}

// ============================================================================
// Constants
// ============================================================================

/**
 * HMI-related constants
 */
export const HMI_CONSTANTS = {
  /** Latency tier thresholds (ms) */
  LATENCY_THRESHOLDS: {
    TIER_1: 10,
    TIER_2: 50,
    TIER_3: 100,
    TIER_4: 500,
  },

  /** Signal quality thresholds */
  QUALITY_THRESHOLDS: {
    SNR_MIN: 40, // dB
    CORRELATION_MAX: 0.3,
    STABILITY_MAX: 0.1,
    ARTIFACT_RATE_MAX: 5,
    ACCURACY_MIN: 90,
  },

  /** Standard haptic patterns */
  HAPTIC_PATTERNS: {
    SINGLE_PULSE: 50,
    DOUBLE_PULSE: 150,
    LONG_PULSE: 200,
    BUZZ: 500,
    HEARTBEAT: 800,
    GRIP_RAMP: 100,
    CONTACT: 30,
    SLIP_WARNING: 200,
  },

  /** Protocol version */
  PROTOCOL_VERSION: 1,

  /** Default sample rates by signal type */
  DEFAULT_SAMPLE_RATES: {
    EEG: 500,
    ECoG: 10000,
    EMG: 2000,
    MOTION: 200,
  },
} as const;

// ============================================================================
// Error Types
// ============================================================================

/**
 * HMI error codes
 */
export enum HMIErrorCode {
  CONNECTION_NOT_FOUND = 'H001',
  CONNECTION_TIMEOUT = 'H002',
  AUTHENTICATION_FAILED = 'H003',
  LOW_SIGNAL_QUALITY = 'H004',
  CHANNEL_FAILURE = 'H005',
  BUFFER_OVERFLOW = 'H006',
  LATENCY_EXCEEDED = 'H007',
  EXCESSIVE_JITTER = 'H008',
  CALIBRATION_REQUIRED = 'H009',
  CALIBRATION_FAILED = 'H010',
  INVALID_COMMAND = 'H011',
  EXECUTION_TIMEOUT = 'H012',
  FEEDBACK_FAILED = 'H013',
  SAFETY_LIMIT = 'H014',
  LOW_BATTERY = 'H015',
}

/**
 * HMI error class
 */
export class HMIError extends Error {
  constructor(
    public code: HMIErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'HMIError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  Vector3,
  EventMarker,
  SignalPacket,
  TransportFlags,
  TransportPacket,
  LatencyTier,
  LatencyComponents,
  LatencyMeasurement,
  IntentCategory,
  Intent,
  CommandType,
  JointCommand,
  MotorCommand,
  CommandResult,
  FeedbackType,
  HapticSegment,
  HapticPattern,
  BodyLocation,
  HapticFeedback,
  NeuralFeedback,
  CalibrationType,
  CalibrationTask,
  ChannelQuality,
  CalibrationModel,
  CalibrationResult,
  CalibrationSession,
  DeviceCapabilities,
  DeviceAdvertisement,
  DeviceState,
  ConnectionConfig,
  Connection,
  FeedbackPreferences,
  UserProfile,
  QualityMetrics,
};

export { SignalType, HMI_CONSTANTS, HMIErrorCode, HMIError };
