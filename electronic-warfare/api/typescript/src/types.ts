/**
 * WIA-DEF-006: Electronic Warfare - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Defense & Security Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core EW Types
// ============================================================================

/**
 * Electronic Warfare operation type
 */
export type EWOperationType = 'EA' | 'EP' | 'ES';

/**
 * Frequency bands
 */
export type FrequencyBand = 'HF' | 'VHF' | 'UHF' | 'SHF' | 'EHF';

/**
 * Jamming types
 */
export type JammingType = 'noise' | 'deception' | 'protocol' | 'chirp' | 'swept';

/**
 * Modulation types
 */
export type ModulationType =
  | 'AM' | 'FM' | 'PM'
  | 'ASK' | 'FSK' | 'PSK' | 'QAM'
  | 'OFDM' | 'CDMA' | 'TDMA'
  | 'FHSS' | 'DSSS';

/**
 * Signal classification
 */
export type SignalType =
  | 'communication'
  | 'radar'
  | 'navigation'
  | 'datalink'
  | 'telemetry'
  | 'unknown';

// ============================================================================
// Spectrum and Frequency
// ============================================================================

/**
 * Frequency range specification
 */
export interface FrequencyRange {
  /** Start frequency in Hz */
  start: number;

  /** End frequency in Hz */
  end: number;

  /** Center frequency in Hz */
  center?: number;

  /** Bandwidth in Hz */
  bandwidth?: number;

  /** Frequency band classification */
  band?: FrequencyBand;
}

/**
 * Spectrum allocation
 */
export interface SpectrumAllocation {
  /** Allocation ID */
  id: string;

  /** Frequency range */
  range: FrequencyRange;

  /** Allocated to (unit/system) */
  allocatedTo: string;

  /** Start time */
  startTime: Date;

  /** End time */
  endTime: Date;

  /** Priority level (0-100) */
  priority: number;

  /** Emission type */
  emissionType?: string;
}

/**
 * Spectrum scan result
 */
export interface SpectrumScanResult {
  /** Scan ID */
  id: string;

  /** Scanned frequency range */
  range: FrequencyRange;

  /** Scan timestamp */
  timestamp: Date;

  /** Detected signals */
  signals: DetectedSignal[];

  /** Spectrum occupancy (0-1) */
  occupancy: number;

  /** Resolution bandwidth in Hz */
  rbw: number;
}

// ============================================================================
// Signal Detection and Analysis
// ============================================================================

/**
 * Detected signal information
 */
export interface DetectedSignal {
  /** Signal ID */
  id: string;

  /** Detection timestamp */
  timestamp: Date;

  /** Frequency in Hz */
  frequency: number;

  /** Bandwidth in Hz */
  bandwidth: number;

  /** Received power in dBm */
  power: number;

  /** Signal-to-Noise Ratio in dB */
  snr?: number;

  /** Modulation type */
  modulation?: ModulationType;

  /** Signal type classification */
  signalType?: SignalType;

  /** Duration in seconds */
  duration?: number;

  /** Pulse parameters (if pulsed) */
  pulse?: PulseParameters;

  /** Direction of arrival in degrees */
  doa?: number;

  /** Location estimate */
  location?: GeographicLocation;

  /** Classification confidence (0-1) */
  confidence: number;
}

/**
 * Pulse parameters for radar/pulsed signals
 */
export interface PulseParameters {
  /** Pulse width in microseconds */
  pulseWidth: number;

  /** Pulse repetition frequency in Hz */
  prf: number;

  /** Pulse repetition interval in seconds */
  pri: number;

  /** Duty cycle (0-1) */
  dutyCycle: number;

  /** Rise time in nanoseconds */
  riseTime?: number;

  /** Fall time in nanoseconds */
  fallTime?: number;
}

/**
 * Geographic location
 */
export interface GeographicLocation {
  /** Latitude in degrees */
  latitude: number;

  /** Longitude in degrees */
  longitude: number;

  /** Altitude in meters */
  altitude?: number;

  /** Location accuracy in meters */
  accuracy?: number;

  /** Coordinate system */
  datum?: string;
}

// ============================================================================
// Electronic Attack (EA)
// ============================================================================

/**
 * Jamming parameters
 */
export interface JammingParameters {
  /** Target frequency in Hz */
  targetFrequency: number;

  /** Target bandwidth in Hz */
  targetBandwidth?: number;

  /** Distance to target in meters */
  targetDistance: number;

  /** Target signal power in watts */
  targetPower: number;

  /** Required Jamming-to-Signal ratio in dB */
  requiredJSRatio: number;

  /** Jamming type */
  jammingType: JammingType;

  /** Jammer antenna gain in dBi */
  jammerGain?: number;

  /** Target antenna gain in dBi */
  targetGain?: number;

  /** Duration in seconds */
  duration?: number;
}

/**
 * Jamming calculation result
 */
export interface JammingResult {
  /** Required jammer power in watts */
  jammerPower: number;

  /** Effective jamming range in meters */
  effectiveRange: number;

  /** Achieved J/S ratio in dB */
  achievedJSRatio: number;

  /** Burn-through range in meters */
  burnThroughRange?: number;

  /** Power consumption in watts */
  powerConsumption: number;

  /** Estimated effectiveness (0-1) */
  effectiveness: number;

  /** Feasibility assessment */
  feasibility: 'possible' | 'difficult' | 'impossible';

  /** Warnings */
  warnings: string[];
}

/**
 * Deception jamming configuration
 */
export interface DeceptionJamming {
  /** Technique type */
  technique: 'false-target' | 'range-gate-pulloff' | 'velocity-gate-pulloff' | 'terrain-bounce';

  /** Target radar frequency in Hz */
  radarFrequency: number;

  /** False target count */
  falseTargetCount?: number;

  /** Range offset in meters */
  rangeOffset?: number;

  /** Velocity offset in m/s */
  velocityOffset?: number;

  /** Delay in seconds */
  delay?: number;

  /** Amplitude modulation factor */
  amplitudeFactor?: number;
}

// ============================================================================
// Electronic Protection (EP)
// ============================================================================

/**
 * Frequency hopping configuration
 */
export interface FrequencyHoppingConfig {
  /** Hopping pattern ID */
  id: string;

  /** Frequency channels in Hz */
  channels: number[];

  /** Hop rate in hops/second */
  hopRate: number;

  /** Dwell time per channel in milliseconds */
  dwellTime: number;

  /** Hopping sequence (channel indices) */
  sequence: number[];

  /** Sync time */
  syncTime: Date;

  /** Processing gain in dB */
  processingGain?: number;
}

/**
 * Spread spectrum parameters
 */
export interface SpreadSpectrumParams {
  /** Spreading type */
  type: 'FHSS' | 'DSSS' | 'THSS';

  /** Spreading bandwidth in Hz */
  spreadBandwidth: number;

  /** Information bandwidth in Hz */
  infoBandwidth: number;

  /** Chip rate in chips/second (for DSSS) */
  chipRate?: number;

  /** Processing gain in dB */
  processingGain: number;

  /** Jamming margin in dB */
  jammingMargin: number;
}

/**
 * Anti-jamming configuration
 */
export interface AntiJammingConfig {
  /** Technique */
  technique: 'frequency-hopping' | 'spread-spectrum' | 'adaptive-nulling' | 'error-correction';

  /** Enabled status */
  enabled: boolean;

  /** Parameters */
  parameters: Record<string, unknown>;

  /** Effectiveness against jamming types */
  effectiveness: {
    noise: number;      // 0-1
    deception: number;  // 0-1
    protocol: number;   // 0-1
  };
}

/**
 * Error correction coding parameters
 */
export interface ErrorCorrectionParams {
  /** Coding scheme */
  scheme: 'reed-solomon' | 'convolutional' | 'turbo' | 'ldpc' | 'polar';

  /** Code rate (k/n) */
  codeRate: number;

  /** Coding gain in dB */
  codingGain: number;

  /** Decoding complexity */
  complexity: 'low' | 'medium' | 'high';

  /** Latency in milliseconds */
  latency?: number;
}

// ============================================================================
// Electronic Support (ES)
// ============================================================================

/**
 * Signal intelligence parameters
 */
export interface SIGINTParameters {
  /** Frequency range to monitor */
  frequencyRange: FrequencyRange;

  /** Scan duration in seconds */
  duration: number;

  /** Resolution bandwidth in Hz */
  rbw: number;

  /** Detection threshold in dB */
  threshold: number;

  /** Direction finding enabled */
  directionFinding?: boolean;

  /** Geolocation enabled */
  geolocation?: boolean;

  /** Classification enabled */
  classification?: boolean;
}

/**
 * SIGINT collection result
 */
export interface SIGINTResult {
  /** Collection ID */
  id: string;

  /** Collection timestamp */
  timestamp: Date;

  /** Frequency in Hz */
  frequency: number;

  /** Bandwidth in Hz */
  bandwidth: number;

  /** Power in dBm */
  power: number;

  /** Modulation type */
  modulation: ModulationType | 'unknown';

  /** Signal classification */
  signalType: SignalType;

  /** Emitter location */
  emitterLocation?: GeographicLocation;

  /** Direction of arrival in degrees */
  doa?: number;

  /** Signal features */
  features?: SignalFeatures;

  /** Classification confidence (0-1) */
  confidence: number;

  /** Additional metadata */
  metadata?: Record<string, unknown>;
}

/**
 * Signal features for classification
 */
export interface SignalFeatures {
  /** Carrier frequency stability (ppm) */
  frequencyStability?: number;

  /** Phase noise in dBc/Hz */
  phaseNoise?: number;

  /** Spurious emissions */
  spurious?: number[];

  /** Symbol rate in symbols/second */
  symbolRate?: number;

  /** Constellation diagram */
  constellation?: number[][];

  /** Spectral characteristics */
  spectral?: {
    bandwidth: number;
    shape: 'rectangular' | 'gaussian' | 'raised-cosine' | 'other';
    rolloff?: number;
  };
}

/**
 * Direction finding result
 */
export interface DirectionFindingResult {
  /** DF measurement ID */
  id: string;

  /** Measurement timestamp */
  timestamp: Date;

  /** Frequency in Hz */
  frequency: number;

  /** Angle of arrival in degrees (0-360) */
  azimuth: number;

  /** Elevation angle in degrees */
  elevation?: number;

  /** Bearing accuracy in degrees */
  accuracy: number;

  /** Method used */
  method: 'phase-interferometry' | 'amplitude-comparison' | 'doppler' | 'tdoa';

  /** Confidence level (0-1) */
  confidence: number;
}

// ============================================================================
// EW Operations
// ============================================================================

/**
 * EW operation
 */
export interface EWOperation {
  /** Operation ID */
  id: string;

  /** Operation type */
  type: EWOperationType;

  /** Operation name */
  name?: string;

  /** Frequency in Hz */
  frequency: number;

  /** Bandwidth in Hz */
  bandwidth: number;

  /** Transmit power in watts */
  power?: number;

  /** Duration in seconds */
  duration: number;

  /** Start time */
  startTime: Date;

  /** End time */
  endTime?: Date;

  /** Mode/technique */
  mode: string;

  /** Target information */
  target?: {
    id?: string;
    type: string;
    location?: GeographicLocation;
    frequency?: number;
  };

  /** Operation status */
  status: 'planned' | 'active' | 'completed' | 'aborted' | 'failed';

  /** Effectiveness assessment (0-1) */
  effectiveness?: number;

  /** Results */
  results?: Record<string, unknown>;
}

/**
 * EW mission
 */
export interface EWMission {
  /** Mission ID */
  id: string;

  /** Mission name */
  name: string;

  /** Mission type */
  type: 'offensive' | 'defensive' | 'support';

  /** Objectives */
  objectives: string[];

  /** Operations */
  operations: EWOperation[];

  /** Start time */
  startTime: Date;

  /** End time */
  endTime?: Date;

  /** Mission status */
  status: 'planned' | 'active' | 'completed' | 'cancelled';

  /** Success criteria */
  successCriteria?: Record<string, unknown>;

  /** Mission effectiveness (0-1) */
  effectiveness?: number;
}

// ============================================================================
// System and Equipment
// ============================================================================

/**
 * EW system configuration
 */
export interface EWSystemConfig {
  /** System ID */
  id: string;

  /** System name */
  name: string;

  /** System type */
  type: 'jammer' | 'receiver' | 'transceiver' | 'df' | 'integrated';

  /** Frequency coverage */
  frequencyRange: FrequencyRange;

  /** Maximum transmit power in watts */
  maxPower?: number;

  /** Antenna configuration */
  antenna: AntennaConfig;

  /** Receiver sensitivity in dBm */
  sensitivity?: number;

  /** Capabilities */
  capabilities: {
    ea: boolean;
    ep: boolean;
    es: boolean;
  };

  /** Location */
  location?: GeographicLocation;

  /** Status */
  status: 'operational' | 'standby' | 'maintenance' | 'offline';
}

/**
 * Antenna configuration
 */
export interface AntennaConfig {
  /** Antenna type */
  type: 'omnidirectional' | 'directional' | 'phased-array' | 'parabolic';

  /** Gain in dBi */
  gain: number;

  /** Beamwidth in degrees */
  beamwidth?: number;

  /** Polarization */
  polarization?: 'vertical' | 'horizontal' | 'circular' | 'dual';

  /** Steering capability */
  steerable?: boolean;

  /** Azimuth range in degrees */
  azimuthRange?: [number, number];

  /** Elevation range in degrees */
  elevationRange?: [number, number];
}

// ============================================================================
// Safety and Compliance
// ============================================================================

/**
 * RF safety parameters
 */
export interface RFSafetyParams {
  /** Maximum permissible exposure in mW/cm² */
  mpe: number;

  /** Frequency in Hz */
  frequency: number;

  /** EIRP in watts */
  eirp: number;

  /** Safe distance in meters */
  safeDistance: number;

  /** Exposure time limit in minutes */
  timeLimit?: number;

  /** Safety zone radius in meters */
  safetyZone: number;
}

/**
 * EMCON level
 */
export type EMCONLevel = 1 | 2 | 3 | 4;

/**
 * EMCON configuration
 */
export interface EMCONConfig {
  /** EMCON level (1-4) */
  level: EMCONLevel;

  /** Restrictions */
  restrictions: {
    radar: boolean;
    communication: boolean;
    navigation: boolean;
    datalink: boolean;
  };

  /** Allowed systems */
  allowedSystems: string[];

  /** Start time */
  startTime: Date;

  /** End time */
  endTime?: Date;
}

/**
 * Spectrum conflict
 */
export interface SpectrumConflict {
  /** Conflict ID */
  id: string;

  /** Detection time */
  timestamp: Date;

  /** Conflicting frequency in Hz */
  frequency: number;

  /** Systems involved */
  systems: string[];

  /** Severity */
  severity: 'low' | 'medium' | 'high' | 'critical';

  /** Description */
  description: string;

  /** Resolution status */
  resolved: boolean;

  /** Resolution action */
  resolution?: string;
}

// ============================================================================
// Physical Constants and Parameters
// ============================================================================

/**
 * Physical constants for EW calculations
 */
export const EW_CONSTANTS = {
  /** Speed of light in m/s */
  SPEED_OF_LIGHT: 299792458,

  /** Boltzmann constant in J/K */
  BOLTZMANN_CONSTANT: 1.38064852e-23,

  /** Standard temperature in Kelvin */
  STANDARD_TEMP: 290,

  /** Free space impedance in Ohms */
  FREE_SPACE_IMPEDANCE: 376.73,

  /** Minimum J/S ratio for noise jamming in dB */
  MIN_JS_NOISE: 10,

  /** Minimum J/S ratio for deception jamming in dB */
  MIN_JS_DECEPTION: 20,

  /** Earth radius in meters */
  EARTH_RADIUS: 6371000,

  /** Standard atmospheric pressure in Pa */
  STANDARD_PRESSURE: 101325,
} as const;

/**
 * Frequency band definitions in Hz
 */
export const FREQUENCY_BANDS = {
  HF: { start: 3e6, end: 30e6 },
  VHF: { start: 30e6, end: 300e6 },
  UHF: { start: 300e6, end: 3e9 },
  SHF: { start: 3e9, end: 30e9 },
  EHF: { start: 30e9, end: 300e9 },
} as const;

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Result type for operations that can fail
 */
export type Result<T, E = Error> =
  | { success: true; data: T }
  | { success: false; error: E };

/**
 * Async result type
 */
export type AsyncResult<T, E = Error> = Promise<Result<T, E>>;

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-DEF-006 error codes
 */
export enum EWErrorCode {
  INSUFFICIENT_POWER = 'EW001',
  FREQUENCY_OUT_OF_RANGE = 'EW002',
  JAMMING_INEFFECTIVE = 'EW003',
  SPECTRUM_CONFLICT = 'EW004',
  SAFETY_VIOLATION = 'EW005',
  EQUIPMENT_FAILURE = 'EW006',
  INVALID_PARAMETERS = 'EW007',
  EMCON_VIOLATION = 'EW008',
  ROE_VIOLATION = 'EW009',
  CIVILIAN_INTERFERENCE = 'EW010',
}

/**
 * Electronic warfare error
 */
export class EWError extends Error {
  constructor(
    public code: EWErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'EWError';
  }
}

// ============================================================================
// Export All Types
// ============================================================================

export type {
  // Core types
  FrequencyRange,
  SpectrumAllocation,
  SpectrumScanResult,
  DetectedSignal,
  PulseParameters,
  GeographicLocation,

  // Electronic Attack
  JammingParameters,
  JammingResult,
  DeceptionJamming,

  // Electronic Protection
  FrequencyHoppingConfig,
  SpreadSpectrumParams,
  AntiJammingConfig,
  ErrorCorrectionParams,

  // Electronic Support
  SIGINTParameters,
  SIGINTResult,
  SignalFeatures,
  DirectionFindingResult,

  // Operations
  EWOperation,
  EWMission,

  // System
  EWSystemConfig,
  AntennaConfig,

  // Safety
  RFSafetyParams,
  EMCONConfig,
  SpectrumConflict,
};

export { EW_CONSTANTS, FREQUENCY_BANDS, EWErrorCode, EWError };
