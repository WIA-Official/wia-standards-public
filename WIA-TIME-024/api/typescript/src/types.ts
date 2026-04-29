/**
 * WIA-TIME-024: Time Measurement - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Time Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Time Types
// ============================================================================

/**
 * Precision levels for time measurement
 */
export enum PrecisionLevel {
  /** Classical - millisecond precision */
  CLASSICAL = 1,
  /** Semi-classical - microsecond precision */
  SEMI_CLASSICAL = 2,
  /** Quantum corrections - nanosecond precision */
  QUANTUM_CORRECTIONS = 3,
  /** Quantum significant - picosecond precision */
  QUANTUM_SIGNIFICANT = 4,
  /** Quantum dominant - femtosecond precision */
  QUANTUM_DOMINANT = 5,
  /** Full quantum - attosecond precision */
  FULL_QUANTUM = 6,
}

/**
 * Time unit types
 */
export type TimeUnit =
  | 's'    // second
  | 'ms'   // millisecond
  | 'μs'   // microsecond
  | 'ns'   // nanosecond
  | 'ps'   // picosecond
  | 'fs'   // femtosecond
  | 'as'   // attosecond
  | 'zs'   // zeptosecond
  | 'tₚ';  // Planck time

/**
 * Atomic clock types
 */
export enum ClockType {
  QUARTZ = 'quartz',
  RUBIDIUM = 'rubidium',
  CESIUM_BEAM = 'cesium-beam',
  CESIUM_FOUNTAIN = 'cesium-fountain',
  OPTICAL_LATTICE = 'optical-lattice',
  QUANTUM = 'quantum',
}

/**
 * Reference frame for time measurement
 */
export interface ReferenceFrame {
  /** Frame identifier */
  id: string;

  /** Frame name (e.g., 'earth-surface', 'gps-orbit') */
  name: string;

  /** Velocity relative to observer (m/s) */
  velocity: number;

  /** Velocity vector components */
  velocityVector?: {
    vx: number;
    vy: number;
    vz: number;
  };

  /** Gravitational potential (m²/s²) */
  gravitationalPotential: number;

  /** Altitude above reference surface (meters) */
  altitude?: number;

  /** Is this frame inertial? */
  inertial: boolean;
}

/**
 * Timeline identifier for multi-timeline measurements
 */
export interface Timeline {
  /** Unique timeline identifier (UUID) */
  id: string;

  /** Timeline name */
  name: string;

  /** Branch point reference timestamp */
  branchPoint?: Date | number;

  /** Divergence rate from primary timeline */
  divergenceRate?: number;

  /** Timeline type */
  type: 'primary' | 'divergent' | 'parallel' | 'quantum';
}

// ============================================================================
// Measurement Types
// ============================================================================

/**
 * Time measurement parameters
 */
export interface TimeMeasurementParams {
  /** Proper time in the local frame (ISO 8601 or timestamp) */
  properTime: Date | number | string;

  /** Velocity as fraction of speed of light (0-1) */
  velocity?: number;

  /** Gravitational potential (m²/s²) */
  gravitationalPotential?: number;

  /** Reference frame for measurement */
  referenceFrame?: ReferenceFrame;

  /** Timeline for measurement */
  timeline?: Timeline;

  /** Include quantum uncertainty corrections */
  includeQuantumUncertainty?: boolean;

  /** Target precision level */
  precisionLevel?: PrecisionLevel;

  /** Clock type to use */
  clockType?: ClockType;
}

/**
 * Time measurement result
 */
export interface TimeMeasurementResult {
  /** Measured proper time */
  properTime: number;

  /** Coordinate time (corrected) */
  coordinateTime: number;

  /** Timestamp with all corrections applied */
  correctedTime: Date;

  /** Special relativity correction (seconds) */
  relativisticOffset: number;

  /** General relativity correction (seconds) */
  gravitationalOffset: number;

  /** Quantum uncertainty correction (seconds) */
  quantumOffset?: number;

  /** Total correction applied (seconds) */
  totalCorrection: number;

  /** Measurement uncertainty (seconds) */
  uncertainty: number;

  /** Measurement unit */
  unit: TimeUnit;

  /** Lorentz factor (gamma) */
  lorentzFactor: number;

  /** Reference frame used */
  referenceFrame: ReferenceFrame;

  /** Timeline */
  timeline?: Timeline;
}

/**
 * Precision time measurement with full metadata
 */
export interface PrecisionTimestamp {
  /** ISO 8601 timestamp */
  iso8601: string;

  /** Unix timestamp (milliseconds) */
  unixMs: number;

  /** Unix timestamp (nanoseconds) */
  unixNs: bigint;

  /** TAI (International Atomic Time) */
  tai: number;

  /** UTC (Coordinated Universal Time) */
  utc: Date;

  /** GPS time */
  gpsTime?: number;

  /** Proper time in local frame */
  properTime: number;

  /** Coordinate time in reference frame */
  coordinateTime: number;

  /** Uncertainty (seconds) */
  uncertainty: number;

  /** Precision level achieved */
  precision: PrecisionLevel;
}

// ============================================================================
// Atomic Clock Types
// ============================================================================

/**
 * Atomic clock calibration parameters
 */
export interface AtomicClockCalibration {
  /** Clock type */
  clockType: ClockType;

  /** Reference standard for calibration */
  referenceStandard: 'caesium-133' | 'strontium-87' | 'ytterbium-171' | 'tai' | 'gps';

  /** Target accuracy (seconds) */
  targetAccuracy: number;

  /** Measurement duration for calibration (seconds) */
  measurementDuration?: number;

  /** Environmental conditions */
  environment?: {
    temperature: number; // Kelvin
    pressure: number;    // Pascals
    magneticField: number; // Tesla
  };
}

/**
 * Atomic clock calibration result
 */
export interface CalibrationResult {
  /** Calibration successful */
  success: boolean;

  /** Frequency offset from reference (Hz) */
  frequencyOffset: number;

  /** Fractional frequency deviation */
  fractionalDeviation: number;

  /** Allan deviation at 1 second */
  allanDeviation: number;

  /** Clock drift rate (seconds/day) */
  driftRate: number;

  /** Achieved accuracy (seconds) */
  accuracy: number;

  /** Phase difference from reference (seconds) */
  phaseDifference: number;

  /** Calibration timestamp */
  calibrationTime: Date;

  /** Next calibration due date */
  nextCalibration: Date;

  /** Environmental effects detected */
  environmentalEffects?: {
    temperature: number;
    pressure: number;
    magneticField: number;
  };
}

/**
 * Clock synchronization parameters
 */
export interface ClockSyncParams {
  /** Local clock identifier */
  localClockId: string;

  /** Remote clock identifier */
  remoteClockId: string;

  /** Synchronization method */
  method: 'two-way' | 'one-way' | 'gps' | 'ntp' | 'ptp';

  /** Network delay compensation */
  compensateDelay?: boolean;

  /** Target synchronization accuracy (seconds) */
  targetAccuracy?: number;
}

/**
 * Clock synchronization result
 */
export interface ClockSyncResult {
  /** Synchronization successful */
  success: boolean;

  /** Time offset between clocks (seconds) */
  offset: number;

  /** Round-trip delay (seconds) */
  delay: number;

  /** Achieved synchronization accuracy (seconds) */
  accuracy: number;

  /** Synchronization timestamp */
  syncTime: Date;

  /** Next sync recommended time */
  nextSync: Date;
}

// ============================================================================
// Relativistic Correction Types
// ============================================================================

/**
 * Special relativity correction parameters
 */
export interface SpecialRelativityParams {
  /** Velocity (m/s or fraction of c) */
  velocity: number;

  /** Is velocity given as fraction of c? */
  velocityAsFractionOfC?: boolean;

  /** Time to correct (seconds) */
  properTime: number;
}

/**
 * Special relativity correction result
 */
export interface SpecialRelativityResult {
  /** Lorentz factor (gamma) */
  lorentzFactor: number;

  /** Time dilation factor */
  timeDilationFactor: number;

  /** Time correction (seconds) */
  correction: number;

  /** Corrected time */
  correctedTime: number;

  /** Velocity as fraction of c */
  velocityFraction: number;
}

/**
 * General relativity correction parameters
 */
export interface GeneralRelativityParams {
  /** Gravitational potential (m²/s²) */
  gravitationalPotential: number;

  /** Or: radial distance from mass (meters) */
  radius?: number;

  /** Or: mass of gravitating body (kg) */
  mass?: number;

  /** Time to correct (seconds) */
  properTime: number;
}

/**
 * General relativity correction result
 */
export interface GeneralRelativityResult {
  /** Gravitational time dilation factor */
  gravitationalFactor: number;

  /** Time correction (seconds) */
  correction: number;

  /** Corrected time */
  correctedTime: number;

  /** Gravitational potential used */
  potential: number;
}

/**
 * Combined relativistic corrections
 */
export interface RelativisticCorrections {
  /** Special relativity correction */
  specialRelativity: SpecialRelativityResult;

  /** General relativity correction */
  generalRelativity: GeneralRelativityResult;

  /** Total combined correction (seconds) */
  totalCorrection: number;

  /** Final corrected time */
  finalTime: number;
}

// ============================================================================
// Multi-Timeline Types
// ============================================================================

/**
 * Cross-timeline measurement parameters
 */
export interface CrossTimelineMeasurementParams {
  /** Source timeline */
  sourceTimeline: Timeline;

  /** Target timeline */
  targetTimeline: Timeline;

  /** Time in source timeline */
  sourceTime: Date | number;

  /** Include divergence correction */
  includeDivergence?: boolean;
}

/**
 * Cross-timeline measurement result
 */
export interface CrossTimelineMeasurementResult {
  /** Time in source timeline */
  sourceTime: number;

  /** Corresponding time in target timeline */
  targetTime: number;

  /** Timeline divergence factor */
  divergenceFactor: number;

  /** Temporal displacement */
  displacement: number;

  /** Synchronization accuracy */
  accuracy: number;

  /** Common branch point */
  branchPoint?: Date;
}

/**
 * Timeline synchronization parameters
 */
export interface TimelineSyncParams {
  /** Timelines to synchronize */
  timelines: Timeline[];

  /** Reference timeline (primary) */
  referenceTimeline?: Timeline;

  /** Synchronization method */
  method: 'branch-point' | 'beacon' | 'quantum-entanglement';

  /** Update frequency (Hz) */
  updateFrequency?: number;
}

/**
 * Timeline synchronization result
 */
export interface TimelineSyncResult {
  /** Synchronization successful */
  success: boolean;

  /** Time offsets for each timeline (seconds) */
  offsets: Map<string, number>;

  /** Coherence time (seconds) */
  coherenceTime: number;

  /** Synchronization uncertainty */
  uncertainty: number;

  /** Next sync time */
  nextSync: Date;
}

// ============================================================================
// Quantum Time Types
// ============================================================================

/**
 * Quantum time measurement parameters
 */
export interface QuantumTimeMeasurementParams {
  /** Time interval to measure */
  interval: number;

  /** Energy uncertainty (Joules) */
  energyUncertainty?: number;

  /** Temperature (Kelvin) */
  temperature?: number;

  /** Include Heisenberg uncertainty */
  includeHeisenberg?: boolean;
}

/**
 * Quantum time measurement result
 */
export interface QuantumTimeMeasurementResult {
  /** Measured interval */
  interval: number;

  /** Quantum uncertainty (seconds) */
  quantumUncertainty: number;

  /** Heisenberg limit (seconds) */
  heisenbergLimit?: number;

  /** Planck time limit reached */
  planckTimeLimitReached: boolean;

  /** Minimum measurable interval */
  minimumInterval: number;
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Physical constants for time measurement
 */
export interface PhysicalConstants {
  /** Speed of light (m/s) */
  readonly SPEED_OF_LIGHT: 299792458;

  /** Planck constant (J⋅s) */
  readonly PLANCK_CONSTANT: 1.054571817e-34;

  /** Gravitational constant (m³⋅kg⁻¹⋅s⁻²) */
  readonly GRAVITATIONAL_CONSTANT: 6.67430e-11;

  /** Planck time (s) */
  readonly PLANCK_TIME: 5.391247e-44;

  /** Cesium-133 frequency (Hz) */
  readonly CESIUM_FREQUENCY: 9192631770;

  /** Boltzmann constant (J/K) */
  readonly BOLTZMANN_CONSTANT: 1.380649e-23;
}

// ============================================================================
// Measurement Configuration
// ============================================================================

/**
 * Time measurement system configuration
 */
export interface MeasurementConfig {
  /** Default clock type */
  defaultClockType: ClockType;

  /** Default precision level */
  defaultPrecisionLevel: PrecisionLevel;

  /** Enable automatic corrections */
  autoCorrections: {
    specialRelativity: boolean;
    generalRelativity: boolean;
    quantum: boolean;
  };

  /** Calibration interval (seconds) */
  calibrationInterval: number;

  /** Synchronization interval (seconds) */
  syncInterval: number;

  /** Measurement cache size */
  cacheSize?: number;

  /** Enable parallel processing */
  parallelProcessing?: boolean;
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * Time measurement error codes
 */
export enum TimeMeasurementError {
  CLOCK_DRIFT = 'CLOCK_DRIFT',
  CALIBRATION_EXPIRED = 'CALIBRATION_EXPIRED',
  RESOLUTION_LIMIT = 'RESOLUTION_LIMIT',
  INVALID_REFERENCE_FRAME = 'INVALID_REFERENCE_FRAME',
  QUANTUM_LIMIT = 'QUANTUM_LIMIT',
  TIMELINE_DIVERGENCE = 'TIMELINE_DIVERGENCE',
  SYNC_FAILED = 'SYNC_FAILED',
  INVALID_VELOCITY = 'INVALID_VELOCITY',
  INVALID_POTENTIAL = 'INVALID_POTENTIAL',
}

/**
 * Time measurement exception
 */
export interface TimeMeasurementException {
  /** Error code */
  code: TimeMeasurementError;

  /** Error message */
  message: string;

  /** Additional error details */
  details?: Record<string, unknown>;

  /** Timestamp of error */
  timestamp: Date;
}

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Time interval with uncertainty
 */
export interface TimeInterval {
  /** Start time */
  start: number;

  /** End time */
  end: number;

  /** Duration (seconds) */
  duration: number;

  /** Uncertainty (seconds) */
  uncertainty: number;

  /** Unit */
  unit: TimeUnit;
}

/**
 * Clock drift information
 */
export interface ClockDrift {
  /** Drift rate (seconds/second) */
  rate: number;

  /** Accumulated drift (seconds) */
  accumulated: number;

  /** Last calibration time */
  lastCalibration: Date;

  /** Predicted drift at next calibration */
  predicted: number;
}

/**
 * Temporal resolution capability
 */
export interface TemporalResolution {
  /** Minimum measurable interval (seconds) */
  minimum: number;

  /** Maximum measurable interval (seconds) */
  maximum: number;

  /** Best achievable accuracy (seconds) */
  accuracy: number;

  /** Resolution class */
  class: 0 | 1 | 2 | 3 | 4 | 5 | 6;
}

// ============================================================================
// Export All Types
// ============================================================================

export type {
  ReferenceFrame,
  Timeline,
  TimeMeasurementParams,
  TimeMeasurementResult,
  PrecisionTimestamp,
  AtomicClockCalibration,
  CalibrationResult,
  ClockSyncParams,
  ClockSyncResult,
  SpecialRelativityParams,
  SpecialRelativityResult,
  GeneralRelativityParams,
  GeneralRelativityResult,
  RelativisticCorrections,
  CrossTimelineMeasurementParams,
  CrossTimelineMeasurementResult,
  TimelineSyncParams,
  TimelineSyncResult,
  QuantumTimeMeasurementParams,
  QuantumTimeMeasurementResult,
  PhysicalConstants,
  MeasurementConfig,
  TimeMeasurementException,
  TimeInterval,
  ClockDrift,
  TemporalResolution,
};

export { PrecisionLevel, ClockType, TimeMeasurementError };
