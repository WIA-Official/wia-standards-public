/**
 * WIA-COMM-001: 6G Communication - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Communication Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * 3D position vector
 */
export interface Vector3D {
  x: number;
  y: number;
  z: number;
}

/**
 * Geographic coordinates
 */
export interface GeoCoordinate {
  latitude: number;
  longitude: number;
  altitude?: number;
}

/**
 * 6G frequency bands
 */
export type FrequencyBand = 'sub-thz' | 'thz-1' | 'thz-2' | 'thz-3';

/**
 * Frequency band ranges (Hz)
 */
export const FREQUENCY_BANDS = {
  'sub-thz': { min: 100e9, max: 300e9 },
  'thz-1': { min: 300e9, max: 1e12 },
  'thz-2': { min: 1e12, max: 3e12 },
  'thz-3': { min: 3e12, max: 10e12 },
} as const;

// ============================================================================
// Spectrum and Channel
// ============================================================================

/**
 * Spectrum allocation request
 */
export interface SpectrumAllocation {
  /** Frequency band */
  band: FrequencyBand;

  /** Center frequency in Hz */
  frequency: number;

  /** Bandwidth in Hz */
  bandwidth: number;

  /** Geographic region */
  region: 'global' | 'regional' | 'local';

  /** Service type */
  serviceType: 'eMBB' | 'URLLC' | 'mMTC' | 'HCS';

  /** Priority level (1-10) */
  priority?: number;
}

/**
 * Spectrum validation result
 */
export interface SpectrumValidation {
  /** Is allocation valid? */
  isValid: boolean;

  /** Allocated frequency range */
  allocation?: {
    centerFreq: number;
    bandwidth: number;
    startFreq: number;
    endFreq: number;
  };

  /** Validation errors */
  errors: string[];

  /** Warnings */
  warnings: string[];

  /** Interference level (dB) */
  interferenceLevel?: number;
}

/**
 * Channel state information
 */
export interface ChannelState {
  /** Channel matrix (complex) */
  channelMatrix: number[][];

  /** Signal-to-Noise Ratio (dB) */
  snr: number;

  /** Path loss (dB) */
  pathLoss: number;

  /** Doppler shift (Hz) */
  dopplerShift: number;

  /** Coherence time (ms) */
  coherenceTime: number;

  /** Coherence bandwidth (Hz) */
  coherenceBandwidth: number;

  /** Channel quality indicator (0-15) */
  cqi: number;
}

// ============================================================================
// Data Rate and Performance
// ============================================================================

/**
 * Data rate calculation parameters
 */
export interface DataRateParams {
  /** Carrier frequency in Hz */
  frequency: number;

  /** Bandwidth in Hz */
  bandwidth: number;

  /** Signal-to-Noise Ratio (dB) */
  snr: number;

  /** Modulation scheme */
  modulation: 'QPSK' | 'QAM-16' | 'QAM-64' | 'QAM-256' | 'QAM-1024';

  /** MIMO configuration */
  mimo?: {
    tx: number; // Transmit antennas
    rx: number; // Receive antennas
  };

  /** Coding rate (0-1) */
  codingRate?: number;
}

/**
 * Data rate calculation result
 */
export interface DataRateResult {
  /** Peak data rate (bps) */
  peakRate: number;

  /** Average data rate (bps) */
  averageRate: number;

  /** Spectral efficiency (bps/Hz) */
  spectralEfficiency: number;

  /** Number of spatial streams */
  spatialStreams: number;

  /** Modulation order */
  modulationOrder: number;

  /** Achievable with current technology */
  feasibility: 'achievable' | 'challenging' | 'theoretical';

  /** Human-readable rate */
  rateFormatted: string;
}

// ============================================================================
// Beamforming
// ============================================================================

/**
 * Beam configuration
 */
export interface BeamConfig {
  /** Unique beam identifier */
  id: string;

  /** Number of antenna elements */
  elements: number;

  /** Target azimuth angle (degrees) */
  azimuthAngle: number;

  /** Target elevation angle (degrees) */
  elevationAngle: number;

  /** Beam width (degrees) */
  beamWidth: number;

  /** Beam gain (dBi) */
  gain: number;

  /** Beamforming weights (complex) */
  weights: { real: number; imag: number }[];

  /** Beam status */
  status: 'idle' | 'forming' | 'active' | 'tracking' | 'failed';
}

/**
 * Beamforming parameters
 */
export interface BeamformingParams {
  /** Number of antenna elements */
  elements: number;

  /** Target direction */
  target: {
    azimuth: number;
    elevation: number;
  };

  /** User position (optional) */
  userPosition?: Vector3D;

  /** Base station position (optional) */
  bsPosition?: Vector3D;

  /** Desired beam width (degrees) */
  beamWidth?: number;

  /** Algorithm type */
  algorithm?: 'analog' | 'digital' | 'hybrid' | 'ai-based';
}

// ============================================================================
// Network Nodes
// ============================================================================

/**
 * 6G Base Station (gNodeB-6G)
 */
export interface BaseStation {
  /** Station identifier */
  id: string;

  /** Station location */
  location: GeoCoordinate;

  /** Operating frequency bands */
  bands: FrequencyBand[];

  /** Number of sectors */
  sectors: number;

  /** Maximum transmit power (dBm) */
  txPower: number;

  /** Antenna configuration */
  antenna: {
    elements: number;
    gain: number;
    height: number;
  };

  /** Connected IRS panels */
  irsConnections?: string[];

  /** Satellite backhaul */
  satelliteBackhaul?: boolean;

  /** Station status */
  status: 'active' | 'standby' | 'maintenance' | 'offline';
}

/**
 * User Equipment
 */
export interface UserEquipment {
  /** Device identifier */
  id: string;

  /** Device type */
  type: 'smartphone' | 'iot' | 'vehicle' | 'xr-headset' | 'industrial';

  /** Current location */
  location: GeoCoordinate;

  /** Velocity vector (m/s) */
  velocity?: Vector3D;

  /** Supported bands */
  supportedBands: FrequencyBand[];

  /** Battery level (0-100%) */
  batteryLevel?: number;

  /** QoS requirements */
  qos: {
    minDataRate: number;
    maxLatency: number;
    reliability: number;
  };
}

// ============================================================================
// Propagation and Path Loss
// ============================================================================

/**
 * THz propagation parameters
 */
export interface THzPropagation {
  /** Frequency in Hz */
  frequency: number;

  /** Distance in meters */
  distance: number;

  /** Temperature in Celsius */
  temperature?: number;

  /** Humidity (0-100%) */
  humidity?: number;

  /** Atmospheric pressure (kPa) */
  pressure?: number;

  /** Rain rate (mm/h) */
  rainRate?: number;
}

/**
 * Propagation loss result
 */
export interface PropagationLoss {
  /** Free space path loss (dB) */
  freeSpaceLoss: number;

  /** Atmospheric absorption (dB) */
  atmosphericLoss: number;

  /** Rain attenuation (dB) */
  rainLoss: number;

  /** Total path loss (dB) */
  totalLoss: number;

  /** Maximum communication range (m) */
  maxRange: number;

  /** Loss breakdown */
  breakdown: {
    oxygenLoss: number;
    waterVaporLoss: number;
    rainLoss: number;
  };
}

// ============================================================================
// Intelligent Reflecting Surface (IRS)
// ============================================================================

/**
 * IRS configuration
 */
export interface IRSConfig {
  /** IRS identifier */
  id: string;

  /** IRS location */
  location: GeoCoordinate;

  /** Panel dimensions (meters) */
  dimensions: {
    width: number;
    height: number;
  };

  /** Number of reflecting elements */
  numElements: number;

  /** Element spacing (meters) */
  elementSpacing: number;

  /** Phase shift configuration */
  phaseShifts: number[];

  /** IRS orientation */
  orientation: {
    azimuth: number;
    elevation: number;
    tilt: number;
  };

  /** IRS status */
  status: 'active' | 'calibrating' | 'standby' | 'failed';
}

/**
 * IRS optimization parameters
 */
export interface IRSOptimization {
  /** IRS configuration */
  irs: IRSConfig;

  /** Base station position */
  bsPosition: Vector3D;

  /** User equipment positions */
  uePositions: Vector3D[];

  /** Optimization objective */
  objective: 'maximize-snr' | 'minimize-interference' | 'maximize-coverage';

  /** Channel state information */
  channelState: ChannelState;

  /** Algorithm */
  algorithm?: 'gradient' | 'alternating' | 'ai-based';
}

// ============================================================================
// Holographic Communication
// ============================================================================

/**
 * Hologram data unit
 */
export interface HologramData {
  /** Hologram identifier */
  id: string;

  /** Timestamp */
  timestamp: Date;

  /** Point cloud data */
  pointCloud: {
    /** Number of points */
    numPoints: number;

    /** Positions (x, y, z) */
    positions: number[][];

    /** Colors (r, g, b) */
    colors: number[][];
  };

  /** Spatial metadata */
  metadata: {
    position: Vector3D;
    rotation: Vector3D;
    scale: Vector3D;
  };

  /** Compression format */
  compression: 'raw' | 'v-pcc' | 'g-pcc' | 'ai-based';

  /** Data size (bytes) */
  dataSize: number;

  /** Quality level (1-10) */
  quality: number;
}

/**
 * Hologram transmission requirements
 */
export interface HologramRequirements {
  /** Resolution */
  resolution: {
    width: number;
    height: number;
  };

  /** Frame rate (fps) */
  frameRate: number;

  /** Number of points per frame */
  pointsPerFrame: number;

  /** Compression ratio */
  compressionRatio: number;

  /** Maximum latency (ms) */
  maxLatency: number;

  /** Required data rate (bps) */
  requiredDataRate: number;
}

// ============================================================================
// Digital Twin
// ============================================================================

/**
 * Digital twin state
 */
export interface DigitalTwinState {
  /** Twin identifier */
  twinId: string;

  /** Timestamp */
  timestamp: Date;

  /** Physical state */
  physicalState: {
    position: Vector3D;
    velocity: Vector3D;
    acceleration: Vector3D;
    sensorData: Record<string, number>;
  };

  /** Virtual state */
  virtualState: {
    predictedPosition: Vector3D;
    plannedTrajectory: Vector3D[];
    simulationResults: Record<string, unknown>;
  };

  /** Synchronization status */
  syncStatus: {
    latency: number; // ms
    accuracy: number; // 0-1
    confidence: number; // 0-1
  };
}

/**
 * Digital twin synchronization
 */
export interface DigitalTwinSync {
  /** Twin ID */
  twinId: string;

  /** Synchronization interval (ms) */
  syncInterval: number;

  /** Required latency (ms) */
  requiredLatency: number;

  /** Data rate (bps) */
  dataRate: number;

  /** Reliability (0-1) */
  reliability: number;

  /** QoS class */
  qosClass: 'URLLC' | 'eMBB';
}

// ============================================================================
// Network Simulation
// ============================================================================

/**
 * Network simulation parameters
 */
export interface NetworkSimulation {
  /** Simulation identifier */
  id: string;

  /** Coverage area (km²) */
  area: number;

  /** Number of base stations */
  numBaseStations: number;

  /** Number of users */
  numUsers: number;

  /** Operating frequency (Hz) */
  frequency: number;

  /** Bandwidth (Hz) */
  bandwidth: number;

  /** IRS deployment */
  irsDeployment?: {
    numPanels: number;
    elementsPerPanel: number;
  };

  /** Traffic model */
  trafficModel: 'full-buffer' | 'bursty' | 'holographic' | 'digital-twin';

  /** Simulation duration (seconds) */
  duration: number;
}

/**
 * Simulation results
 */
export interface SimulationResult {
  /** Simulation ID */
  id: string;

  /** Performance metrics */
  metrics: {
    averageDataRate: number; // bps
    peakDataRate: number; // bps
    averageLatency: number; // ms
    latency99Percentile: number; // ms
    coverage: number; // %
    spectralEfficiency: number; // bps/Hz
    energyEfficiency: number; // bits/Joule
  };

  /** User statistics */
  userStats: {
    satisfied: number; // %
    blocked: number; // %
    handovers: number;
  };

  /** Resource utilization */
  resourceUtil: {
    spectrum: number; // %
    baseStations: number; // %
    irs: number; // %
  };

  /** Simulation success */
  success: boolean;

  /** Errors/warnings */
  logs: string[];
}

// ============================================================================
// AI-based Features
// ============================================================================

/**
 * AI beam prediction
 */
export interface AIBeamPrediction {
  /** User trajectory history */
  trajectory: {
    timestamp: Date;
    position: Vector3D;
    velocity: Vector3D;
  }[];

  /** Channel history */
  channelHistory: ChannelState[];

  /** Prediction horizon (ms) */
  predictionHorizon: number;

  /** Model type */
  modelType: 'lstm' | 'gru' | 'transformer' | 'cnn';
}

/**
 * AI beam prediction result
 */
export interface BeamPredictionResult {
  /** Predicted beam configuration */
  predictedBeam: BeamConfig;

  /** Prediction confidence (0-1) */
  confidence: number;

  /** Prediction time (ms) */
  predictionTime: number;

  /** Alternative beams */
  alternatives?: BeamConfig[];
}

// ============================================================================
// Security and Privacy
// ============================================================================

/**
 * Quantum-safe encryption
 */
export interface QuantumSafeEncryption {
  /** Algorithm type */
  algorithm: 'CRYSTALS-Kyber' | 'CRYSTALS-Dilithium' | 'SPHINCS+' | 'Classic-McEliece';

  /** Key size (bits) */
  keySize: number;

  /** Public key */
  publicKey: Uint8Array;

  /** Encrypted message */
  ciphertext?: Uint8Array;

  /** Security level (1-5) */
  securityLevel: number;
}

/**
 * Anomaly detection result
 */
export interface AnomalyDetection {
  /** Detection timestamp */
  timestamp: Date;

  /** Anomaly type */
  type: 'ddos' | 'intrusion' | 'jamming' | 'spoofing' | 'other';

  /** Severity (1-10) */
  severity: number;

  /** Affected nodes */
  affectedNodes: string[];

  /** Confidence (0-1) */
  confidence: number;

  /** Recommended action */
  action: 'monitor' | 'throttle' | 'block' | 'alert';
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * 6G physical constants
 */
export const SIXG_CONSTANTS = {
  /** Speed of light (m/s) */
  SPEED_OF_LIGHT: 299792458,

  /** Boltzmann constant (J/K) */
  BOLTZMANN: 1.380649e-23,

  /** Standard temperature (K) */
  STANDARD_TEMP: 290,

  /** Standard pressure (kPa) */
  STANDARD_PRESSURE: 101.325,

  /** Atmospheric refractivity */
  REFRACTIVITY: 315,
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
 * WIA-COMM-001 error codes
 */
export enum CommErrorCode {
  INVALID_FREQUENCY = 'C001',
  INSUFFICIENT_BANDWIDTH = 'C002',
  BEAM_ALIGNMENT_FAILED = 'C003',
  IRS_OPTIMIZATION_FAILED = 'C004',
  HIGH_INTERFERENCE = 'C005',
  LATENCY_EXCEEDED = 'C006',
  QOS_VIOLATION = 'C007',
  PROPAGATION_LOSS_TOO_HIGH = 'C008',
  INVALID_PARAMETERS = 'C009',
  SIMULATION_FAILED = 'C010',
}

/**
 * 6G Communication error
 */
export class SixGCommError extends Error {
  constructor(
    public code: CommErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'SixGCommError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  Vector3D,
  GeoCoordinate,
  SpectrumAllocation,
  SpectrumValidation,
  ChannelState,
  DataRateParams,
  DataRateResult,
  BeamConfig,
  BeamformingParams,
  BaseStation,
  UserEquipment,
  THzPropagation,
  PropagationLoss,
  IRSConfig,
  IRSOptimization,
  HologramData,
  HologramRequirements,
  DigitalTwinState,
  DigitalTwinSync,
  NetworkSimulation,
  SimulationResult,
  AIBeamPrediction,
  BeamPredictionResult,
  QuantumSafeEncryption,
  AnomalyDetection,
};

export { FREQUENCY_BANDS, SIXG_CONSTANTS, CommErrorCode, SixGCommError };
