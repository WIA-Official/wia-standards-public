/**
 * WIA-SEMI-017 3D Touch SDK - Type Definitions
 * 
 * Comprehensive TypeScript types for force sensing, haptic feedback,
 * and pressure-sensitive touch interfaces.
 */

/**
 * Force level in grams-force (gf)
 */
export type ForceGrams = number;

/**
 * Normalized force value (0.0 to 1.0)
 */
export type NormalizedForce = number;

/**
 * Pressure in kiloPascals (kPa)
 */
export type PressureKPa = number;

/**
 * Timestamp in milliseconds
 */
export type TimestampMs = number;

/**
 * Force sensor types supported by WIA-SEMI-017
 */
export enum ForceSensorType {
  CAPACITIVE = 'capacitive',
  STRAIN_GAUGE = 'strain_gauge',
  PIEZOELECTRIC = 'piezoelectric',
  RESISTIVE = 'resistive',
  OPTICAL = 'optical'
}

/**
 * Haptic actuator types
 */
export enum HapticActuatorType {
  LRA = 'lra',            // Linear Resonant Actuator
  ERM = 'erm',            // Eccentric Rotating Mass
  PIEZO = 'piezoelectric', // Piezoelectric actuator
  VCA = 'voice_coil'      // Voice Coil Actuator
}

/**
 * Force gesture types
 */
export enum ForceGestureType {
  PEEK = 'peek',          // Light force press (preview)
  POP = 'pop',            // Deep force press (activate)
  LONG_PRESS = 'long_press', // Force-based long press
  HOVER = 'hover',        // Near-zero force (detected touch)
  RELEASE = 'release'     // Force released
}

/**
 * Haptic feedback patterns
 */
export enum HapticPattern {
  CLICK = 'click',        // Short, sharp haptic
  THUD = 'thud',          // Soft impact
  TICK = 'tick',          // Subtle tick
  BUZZ = 'buzz',          // Continuous vibration
  RAMP_UP = 'ramp_up',    // Increasing intensity
  RAMP_DOWN = 'ramp_down' // Decreasing intensity
}

/**
 * Haptic intensity levels
 */
export enum HapticIntensity {
  LIGHT = 'light',
  MEDIUM = 'medium',
  HEAVY = 'heavy'
}

/**
 * Force calibration point
 */
export interface ForceCalibrationPoint {
  /** Applied force in gf */
  force: ForceGrams;
  /** Raw sensor reading */
  rawValue: number;
  /** Temperature at calibration (°C) */
  temperature?: number;
}

/**
 * Force sensor configuration
 */
export interface ForceSensorConfig {
  /** Sensor type */
  type: ForceSensorType;
  /** Calibration points */
  calibration: ForceCalibrationPoint[];
  /** Maximum force (gf) */
  maxForce: ForceGrams;
  /** Minimum detectable force (gf) */
  minForce: ForceGrams;
  /** Sampling rate (Hz) */
  samplingRate: number;
  /** Resolution (number of discrete levels) */
  resolution: number;
  /** Temperature coefficient (per °C) */
  temperatureCoefficient?: number;
}

/**
 * Force thresholds for gestures
 */
export interface ForceThresholds {
  /** Touch detection threshold (gf) */
  touch: ForceGrams;
  /** Peek gesture threshold (gf) */
  peek: ForceGrams;
  /** Pop gesture threshold (gf) */
  pop: ForceGrams;
  /** Maximum force (gf) */
  max: ForceGrams;
  /** Hysteresis margin (gf) */
  hysteresis: ForceGrams;
}

/**
 * Force measurement data
 */
export interface ForceMeasurement {
  /** Force in gf */
  force: ForceGrams;
  /** Normalized force (0-1) */
  normalizedForce: NormalizedForce;
  /** Pressure in kPa */
  pressure?: PressureKPa;
  /** Timestamp */
  timestamp: TimestampMs;
  /** Touch position X (pixels) */
  x?: number;
  /** Touch position Y (pixels) */
  y?: number;
  /** Contact area (mm²) */
  contactArea?: number;
  /** Raw sensor value */
  rawValue?: number;
}

/**
 * Force gesture event
 */
export interface ForceGestureEvent {
  /** Gesture type */
  type: ForceGestureType;
  /** Force measurement */
  force: ForceMeasurement;
  /** Gesture start timestamp */
  startTime: TimestampMs;
  /** Gesture duration (ms) */
  duration?: TimestampMs;
}

/**
 * Haptic waveform definition
 */
export interface HapticWaveform {
  /** Waveform pattern */
  pattern: HapticPattern;
  /** Intensity level */
  intensity: HapticIntensity;
  /** Duration (ms) */
  duration: TimestampMs;
  /** Rise time (ms) */
  riseTime?: TimestampMs;
  /** Decay time (ms) */
  decayTime?: TimestampMs;
  /** Peak amplitude (G) */
  amplitude?: number;
}

/**
 * Haptic actuator configuration
 */
export interface HapticActuatorConfig {
  /** Actuator type */
  type: HapticActuatorType;
  /** Resonant frequency (Hz, for LRA) */
  resonantFrequency?: number;
  /** Maximum acceleration (G) */
  maxAcceleration?: number;
  /** Response time (ms) */
  responseTime: TimestampMs;
  /** Power consumption (mW) */
  powerConsumption?: number;
}

/**
 * Force sensing configuration
 */
export interface Force3DConfig {
  /** Force sensor configuration */
  sensor: ForceSensorConfig;
  /** Force thresholds */
  thresholds: ForceThresholds;
  /** Haptic actuator configuration */
  haptic?: HapticActuatorConfig;
  /** Enable adaptive thresholds */
  adaptiveThresholds?: boolean;
  /** Enable temperature compensation */
  temperatureCompensation?: boolean;
}

/**
 * Force event listener callback
 */
export type ForceEventListener = (event: ForceGestureEvent) => void;

/**
 * Haptic completion callback
 */
export type HapticCallback = () => void;
