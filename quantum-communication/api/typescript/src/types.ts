/**
 * WIA-COMM-006: Quantum Communication - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Quantum Communication Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Quantum Types
// ============================================================================

/**
 * Complex number representation
 */
export interface Complex {
  real: number;
  imag: number;
}

/**
 * Quantum state vector
 */
export interface QuantumState {
  /** State dimension (2 for qubit, 4 for two qubits, etc.) */
  dimension: number;

  /** Amplitude coefficients */
  amplitudes: Complex[];

  /** State description */
  description?: string;

  /** Basis representation */
  basis?: 'computational' | 'hadamard' | 'bell' | 'circular';
}

/**
 * Photon polarization state
 */
export type PolarizationState =
  | 'horizontal' // |H⟩ or |→⟩ (0°)
  | 'vertical' // |V⟩ or |↑⟩ (90°)
  | 'diagonal' // |D⟩ or |+⟩ (45°)
  | 'anti-diagonal' // |A⟩ or |−⟩ (135°)
  | 'left-circular' // |L⟩
  | 'right-circular'; // |R⟩

/**
 * Measurement basis for polarization
 */
export type MeasurementBasis =
  | 'rectilinear' // {|H⟩, |V⟩}
  | 'diagonal' // {|D⟩, |A⟩}
  | 'circular'; // {|L⟩, |R⟩}

/**
 * Bell state types
 */
export type BellState = 'phi-plus' | 'phi-minus' | 'psi-plus' | 'psi-minus';

/**
 * Encoding scheme for quantum information
 */
export type EncodingScheme =
  | 'polarization' // Photon polarization
  | 'phase' // Optical phase
  | 'time-bin' // Time-bin encoding
  | 'frequency'; // Frequency encoding

// ============================================================================
// Quantum Key Distribution (QKD) Protocols
// ============================================================================

/**
 * QKD protocol types
 */
export type QKDProtocol = 'BB84' | 'E91' | 'B92' | 'BBM92' | 'SARG04' | 'COW' | 'DPS';

/**
 * BB84 protocol parameters
 */
export interface BB84Parameters {
  /** Desired key length in bits */
  keyLength: number;

  /** Sender node identifier */
  sender: string;

  /** Receiver node identifier */
  receiver: string;

  /** Quantum channel configuration */
  channel: QuantumChannelConfig;

  /** Maximum acceptable QBER threshold */
  qberThreshold?: number;

  /** Enable privacy amplification */
  privacyAmplification?: boolean;

  /** Error correction method */
  errorCorrection?: ErrorCorrectionMethod;

  /** Measurement bases to use */
  bases?: MeasurementBasis[];

  /** Use decoy states for security */
  decoyStates?: boolean;

  /** Photon intensity levels (for decoy states) */
  intensityLevels?: number[];
}

/**
 * E91 protocol parameters
 */
export interface E91Parameters {
  /** Desired key length in bits */
  keyLength: number;

  /** Node A identifier */
  nodeA: string;

  /** Node B identifier */
  nodeB: string;

  /** Number of Bell pairs to generate */
  bellPairs: number;

  /** CHSH inequality threshold */
  chshThreshold?: number;

  /** Minimum acceptable Bell violation */
  minBellViolation?: number;

  /** Entanglement source configuration */
  entanglementSource?: EntanglementSourceConfig;
}

/**
 * B92 protocol parameters
 */
export interface B92Parameters {
  /** Desired key length in bits */
  keyLength: number;

  /** Sender node identifier */
  sender: string;

  /** Receiver node identifier */
  receiver: string;

  /** Quantum channel configuration */
  channel: QuantumChannelConfig;

  /** Maximum acceptable QBER threshold */
  qberThreshold?: number;
}

/**
 * QKD result
 */
export interface QKDResult {
  /** Generated secure key (hex encoded) */
  key: string;

  /** Key length in bits */
  keyLength: number;

  /** Quantum bit error rate */
  qber: number;

  /** Secure key rate (bits/second) */
  keyRate: number;

  /** Channel fidelity */
  fidelity: number;

  /** Security parameter (epsilon-security) */
  securityParameter: number;

  /** Protocol used */
  protocol: QKDProtocol;

  /** Timestamp */
  timestamp: Date;

  /** Additional metadata */
  metadata?: {
    rawBitsSent: number;
    siftedBits: number;
    bitsAfterErrorCorrection: number;
    privacyAmplificationRatio: number;
    channelTransmittance: number;
  };
}

/**
 * E91 result with Bell test
 */
export interface E91Result extends QKDResult {
  /** CHSH parameter S (should be > 2√2 for quantum) */
  chshParameter: number;

  /** Bell inequality violated? */
  bellViolation: boolean;

  /** Individual correlation measurements */
  correlations: {
    [basisPair: string]: number;
  };
}

/**
 * Error correction methods
 */
export type ErrorCorrectionMethod = 'cascade' | 'ldpc' | 'polar' | 'turbo' | 'bch';

/**
 * Privacy amplification methods
 */
export type PrivacyAmplificationMethod =
  | 'universal-hash'
  | 'toeplitz'
  | 'random-oracle'
  | 'extractor';

// ============================================================================
// Quantum Channels
// ============================================================================

/**
 * Quantum channel type
 */
export type QuantumChannelType =
  | 'fiber-optic'
  | 'free-space'
  | 'satellite-uplink'
  | 'satellite-downlink';

/**
 * Quantum channel configuration
 */
export interface QuantumChannelConfig {
  /** Channel type */
  type: QuantumChannelType;

  /** Wavelength in nanometers */
  wavelength: number;

  /** Distance in kilometers */
  distance: number;

  /** Channel-specific parameters */
  parameters?: FiberParameters | FreeSpaceParameters | SatelliteParameters;
}

/**
 * Fiber optic channel parameters
 */
export interface FiberParameters {
  /** Fiber type */
  fiberType: 'SMF-28' | 'DSF' | 'NZDSF' | 'custom';

  /** Loss coefficient (dB/km) */
  lossCoefficient: number;

  /** Dispersion (ps/nm/km) */
  dispersion?: number;

  /** Polarization mode dispersion (ps/√km) */
  pmd?: number;

  /** Use wavelength division multiplexing */
  wdm?: boolean;
}

/**
 * Free-space channel parameters
 */
export interface FreeSpaceParameters {
  /** Transmitter location */
  transmitter: GeoLocation;

  /** Receiver location */
  receiver: GeoLocation;

  /** Atmospheric visibility (km) */
  visibility: number;

  /** Weather conditions */
  weather: 'clear' | 'haze' | 'fog' | 'rain' | 'snow';

  /** Turbulence strength (Cn² in m^(-2/3)) */
  turbulence?: number;

  /** Use adaptive optics */
  adaptiveOptics?: boolean;

  /** Beam divergence (mrad) */
  beamDivergence?: number;
}

/**
 * Satellite channel parameters
 */
export interface SatelliteParameters {
  /** Ground station location */
  groundStation: GeoLocation;

  /** Satellite identifier */
  satelliteId: string;

  /** Satellite orbit type */
  orbitType: 'LEO' | 'MEO' | 'GEO';

  /** Satellite altitude (km) */
  altitude: number;

  /** Elevation angle (degrees) */
  elevation: number;

  /** Link type */
  linkType: 'uplink' | 'downlink' | 'inter-satellite';

  /** Atmospheric transmission */
  atmosphericTransmission?: number;
}

/**
 * Geographic location
 */
export interface GeoLocation {
  /** Latitude in degrees */
  latitude: number;

  /** Longitude in degrees */
  longitude: number;

  /** Altitude in meters */
  altitude?: number;

  /** Location name */
  name?: string;
}

/**
 * Quantum link quality metrics
 */
export interface QuantumLinkQuality {
  /** Link identifier */
  linkId: string;

  /** Timestamp of measurement */
  timestamp: Date;

  /** Channel transmittance (0-1) */
  transmittance: number;

  /** Channel fidelity (0-1) */
  fidelity: number;

  /** Quantum bit error rate */
  qber: number;

  /** Channel loss (dB) */
  loss: number;

  /** Photon count rate (Hz) */
  photonRate: number;

  /** Background noise rate (Hz) */
  noiseRate: number;

  /** Signal-to-noise ratio */
  snr: number;
}

// ============================================================================
// Photon Sources and Detectors
// ============================================================================

/**
 * Photon source types
 */
export type PhotonSourceType =
  | 'attenuated-laser'
  | 'SPDC'
  | 'quantum-dot'
  | 'parametric-down-conversion'
  | 'four-wave-mixing';

/**
 * Photon source configuration
 */
export interface PhotonSource {
  /** Source identifier */
  id: string;

  /** Source type */
  type: PhotonSourceType;

  /** Emission wavelength (nm) */
  wavelength: number;

  /** Photon emission rate (Hz) */
  emissionRate: number;

  /** Mean photon number per pulse */
  meanPhotonNumber?: number;

  /** Heralding efficiency (for SPDC) */
  heraldingEfficiency?: number;

  /** Indistinguishability (0-1) */
  indistinguishability?: number;

  /** Spectral purity */
  spectralPurity?: number;

  /** Operating temperature (K) */
  temperature?: number;

  /** Status */
  status: 'operational' | 'offline' | 'calibrating' | 'error';
}

/**
 * Single-photon detector types
 */
export type DetectorType = 'APD' | 'SNSPD' | 'PMT' | 'TES' | 'SiPM';

/**
 * Single-photon detector
 */
export interface PhotonDetector {
  /** Detector identifier */
  id: string;

  /** Detector type */
  type: DetectorType;

  /** Detection efficiency (0-1) */
  efficiency: number;

  /** Dark count rate (Hz) */
  darkCountRate: number;

  /** Timing jitter (ps) */
  timingJitter: number;

  /** Dead time (ns) */
  deadTime: number;

  /** Operating temperature (K) */
  temperature?: number;

  /** Wavelength sensitivity range (nm) */
  wavelengthRange: [number, number];

  /** Maximum count rate (Hz) */
  maxCountRate: number;

  /** Status */
  status: 'operational' | 'offline' | 'saturated' | 'blinded' | 'error';
}

// ============================================================================
// Quantum Repeaters
// ============================================================================

/**
 * Quantum memory technology
 */
export type QuantumMemoryTech =
  | 'rare-earth-ions'
  | 'atomic-ensemble'
  | 'quantum-dot'
  | 'NV-center'
  | 'trapped-ion'
  | 'superconducting';

/**
 * Quantum repeater node
 */
export interface QuantumRepeater {
  /** Node identifier */
  id: string;

  /** Geographic position */
  position: GeoLocation;

  /** Quantum memory configuration */
  memory: {
    technology: QuantumMemoryTech;
    capacity: number; // number of qubits
    coherenceTime: number; // milliseconds
    storageEfficiency: number; // 0-1
    retrievalEfficiency: number; // 0-1
    fidelity: number; // 0-1
  };

  /** Bell state measurement capability */
  bellStateMeasurement: {
    available: boolean;
    successProbability: number;
    fidelity: number;
  };

  /** Connected neighbors */
  neighbors: string[];

  /** Current utilization (0-1) */
  utilization: number;

  /** Status */
  status: 'online' | 'offline' | 'maintenance' | 'degraded';

  /** Performance metrics */
  metrics: {
    entanglementRate: number; // pairs per second
    averageFidelity: number;
    successRate: number;
    uptime: number; // percentage
  };
}

/**
 * Repeater chain configuration
 */
export interface RepeaterChain {
  /** Chain identifier */
  id: string;

  /** Start node */
  startNode: string;

  /** End node */
  endNode: string;

  /** Intermediate repeaters */
  repeaters: string[];

  /** Segment lengths (km) */
  segmentLengths: number[];

  /** Total distance (km) */
  totalDistance: number;

  /** Expected end-to-end fidelity */
  expectedFidelity: number;

  /** Expected key rate (bits/second) */
  expectedKeyRate: number;

  /** Number of swapping operations */
  swappingOperations: number;
}

// ============================================================================
// Entanglement Distribution
// ============================================================================

/**
 * Entanglement source configuration
 */
export interface EntanglementSourceConfig {
  /** Source type */
  type: 'SPDC' | 'quantum-dot' | 'trapped-ion' | 'NV-center';

  /** Crystal type (for SPDC) */
  crystal?: 'BBO' | 'KTP' | 'PPKTP' | 'LiNbO3';

  /** Pump wavelength (nm) */
  pumpWavelength?: number;

  /** Signal wavelength (nm) */
  signalWavelength?: number;

  /** Idler wavelength (nm) */
  idlerWavelength?: number;

  /** Pair generation rate (pairs/second) */
  pairRate: number;

  /** Heralding efficiency */
  heraldingEfficiency: number;
}

/**
 * Entangled photon pair
 */
export interface EntanglementPair {
  /** Unique identifier */
  id: string;

  /** Node A */
  nodeA: string;

  /** Node B */
  nodeB: string;

  /** Bell state type */
  state: BellState;

  /** Fidelity (0-1) */
  fidelity: number;

  /** Creation timestamp */
  created: Date;

  /** Expiration time */
  expires?: Date;

  /** Source of entanglement */
  source: string;

  /** Status */
  status: 'active' | 'consumed' | 'expired' | 'degraded';
}

/**
 * Entanglement distribution parameters
 */
export interface EntanglementDistributionParams {
  /** First node */
  nodeA: string;

  /** Second node */
  nodeB: string;

  /** Target fidelity */
  targetFidelity: number;

  /** Number of pairs to generate */
  pairs: number;

  /** Use purification */
  purification?: boolean;

  /** Maximum purification rounds */
  maxPurificationRounds?: number;

  /** Timeout in milliseconds */
  timeout?: number;
}

/**
 * Entanglement distribution result
 */
export interface EntanglementDistributionResult {
  /** Generated pairs */
  pairs: EntanglementPair[];

  /** Average fidelity */
  averageFidelity: number;

  /** Success rate */
  successRate: number;

  /** Total duration (ms) */
  duration: number;

  /** Purification rounds performed */
  purificationRounds?: number;
}

/**
 * Entanglement swapping operation
 */
export interface EntanglementSwapping {
  /** Input pair 1 (A-B) */
  pairAB: string;

  /** Input pair 2 (B-C) */
  pairBC: string;

  /** Intermediate node */
  intermediateNode: string;

  /** Output pair (A-C) */
  pairAC?: string;

  /** Bell measurement result */
  bellMeasurement?: BellState;

  /** Success status */
  success: boolean;

  /** Final fidelity */
  finalFidelity?: number;

  /** Timestamp */
  timestamp: Date;
}

// ============================================================================
// Security
// ============================================================================

/**
 * Authentication method
 */
export type AuthenticationMethod =
  | 'pre-shared-key'
  | 'wegman-carter'
  | 'post-quantum-signature'
  | 'device-independent';

/**
 * Security parameters
 */
export interface SecurityParameters {
  /** Target security parameter (epsilon) */
  epsilon: number;

  /** Correctness parameter */
  correctness: number;

  /** Secrecy parameter */
  secrecy: number;

  /** Privacy amplification method */
  privacyAmplification: PrivacyAmplificationMethod;

  /** Error correction method */
  errorCorrection: ErrorCorrectionMethod;

  /** Authentication method */
  authentication: AuthenticationMethod;

  /** Enable finite-key analysis */
  finiteKeyAnalysis?: boolean;

  /** Block size for finite-key */
  blockSize?: number;
}

/**
 * Security threat types
 */
export type SecurityThreat =
  | 'photon-number-splitting'
  | 'trojan-horse'
  | 'detector-blinding'
  | 'phase-remapping'
  | 'wavelength-attack'
  | 'time-shift-attack';

/**
 * Side-channel monitoring
 */
export interface SideChannelMonitoring {
  /** Monitor photon number splitting */
  pns: boolean;

  /** Monitor Trojan horse attacks */
  trojanHorse: boolean;

  /** Monitor detector blinding */
  detectorBlinding: boolean;

  /** Monitor phase remapping */
  phaseRemapping: boolean;

  /** Alert threshold */
  alertThreshold: number;

  /** Current threat level */
  threatLevel: 'none' | 'low' | 'medium' | 'high' | 'critical';

  /** Detected anomalies */
  anomalies: SecurityAnomaly[];
}

/**
 * Security anomaly
 */
export interface SecurityAnomaly {
  /** Anomaly type */
  type: SecurityThreat;

  /** Severity level */
  severity: 'low' | 'medium' | 'high' | 'critical';

  /** Detection timestamp */
  timestamp: Date;

  /** Description */
  description: string;

  /** Affected components */
  affectedComponents: string[];

  /** Recommended action */
  recommendedAction: string;
}

// ============================================================================
// Performance Monitoring
// ============================================================================

/**
 * System performance metrics
 */
export interface SystemPerformanceMetrics {
  /** Timestamp */
  timestamp: Date;

  /** Overall system health (0-1) */
  health: number;

  /** Active quantum links */
  activeLinks: number;

  /** Total photon transmission rate (Hz) */
  photonRate: number;

  /** Average QBER across all links */
  averageQBER: number;

  /** Total secure key rate (bits/second) */
  totalKeyRate: number;

  /** System uptime (percentage) */
  uptime: number;

  /** Failed operations in last hour */
  failedOperations: number;

  /** Average link fidelity */
  averageFidelity: number;
}

/**
 * QBER monitoring
 */
export interface QBERMonitoring {
  /** Channel identifier */
  channelId: string;

  /** Current QBER */
  currentQBER: number;

  /** QBER history (last N measurements) */
  history: Array<{
    timestamp: Date;
    qber: number;
  }>;

  /** Alert threshold */
  alertThreshold: number;

  /** Alert active */
  alertActive: boolean;

  /** Trend analysis */
  trend: 'stable' | 'increasing' | 'decreasing' | 'fluctuating';
}

// ============================================================================
// Post-Quantum Cryptography Integration
// ============================================================================

/**
 * Post-quantum algorithm
 */
export type PostQuantumAlgorithm =
  | 'CRYSTALS-Kyber'
  | 'CRYSTALS-Dilithium'
  | 'SPHINCS+'
  | 'FALCON'
  | 'NTRU';

/**
 * Hybrid QKD + PQC configuration
 */
export interface HybridQKDPQCConfig {
  /** Use hybrid mode */
  enabled: boolean;

  /** Post-quantum KEM algorithm */
  kemAlgorithm: PostQuantumAlgorithm;

  /** Post-quantum signature algorithm */
  signatureAlgorithm: PostQuantumAlgorithm;

  /** Key combination method */
  keyCombination: 'concatenate' | 'xor' | 'kdf';

  /** Key derivation function */
  kdf?: 'HKDF' | 'PBKDF2' | 'Argon2';
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-COMM-006 error codes
 */
export enum QuantumCommunicationErrorCode {
  INSUFFICIENT_FIDELITY = 'QC001',
  HIGH_QBER = 'QC002',
  QKD_FAILED = 'QC003',
  CHANNEL_LOSS_TOO_HIGH = 'QC004',
  DETECTOR_FAILURE = 'QC005',
  SOURCE_FAILURE = 'QC006',
  AUTHENTICATION_FAILED = 'QC007',
  ENTANGLEMENT_FAILED = 'QC008',
  REPEATER_FAILURE = 'QC009',
  BELL_VIOLATION_FAILED = 'QC010',
  SECURITY_VIOLATION = 'QC011',
  INVALID_PARAMETERS = 'QC012',
  TIMEOUT = 'QC013',
  HARDWARE_ERROR = 'QC014',
  CALIBRATION_REQUIRED = 'QC015',
}

/**
 * Quantum communication error
 */
export class QuantumCommunicationError extends Error {
  constructor(
    public code: QuantumCommunicationErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'QuantumCommunicationError';
  }
}

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
// Physical Constants
// ============================================================================

/**
 * Physical constants for quantum communication
 */
export const QUANTUM_COMM_CONSTANTS = {
  /** Speed of light (m/s) */
  SPEED_OF_LIGHT: 299792458,

  /** Planck constant (J·s) */
  PLANCK: 6.62607015e-34,

  /** Fiber attenuation at 1550nm (dB/km) */
  FIBER_LOSS_1550: 0.2,

  /** Fiber attenuation at 1310nm (dB/km) */
  FIBER_LOSS_1310: 0.3,

  /** Maximum QBER for BB84 security (individual attacks) */
  MAX_QBER_BB84_INDIVIDUAL: 0.11,

  /** Maximum QBER for BB84 security (collective attacks) */
  MAX_QBER_BB84_COLLECTIVE: 0.20,

  /** CHSH bound for quantum correlations */
  CHSH_QUANTUM: 2 * Math.sqrt(2),

  /** CHSH bound for classical correlations */
  CHSH_CLASSICAL: 2,

  /** Minimum useful entanglement fidelity */
  MIN_ENTANGLEMENT_FIDELITY: 0.5,

  /** Telecom wavelength C-band (nm) */
  WAVELENGTH_C_BAND: 1550,

  /** Telecom wavelength O-band (nm) */
  WAVELENGTH_O_BAND: 1310,

  /** Typical single-photon energy at 1550nm (J) */
  PHOTON_ENERGY_1550: 1.28e-19,
} as const;

// ============================================================================
// Export All Types
// ============================================================================

export type {
  Complex,
  QuantumState,
  PolarizationState,
  MeasurementBasis,
  BellState,
  EncodingScheme,
  QKDProtocol,
  BB84Parameters,
  E91Parameters,
  B92Parameters,
  QKDResult,
  E91Result,
  ErrorCorrectionMethod,
  PrivacyAmplificationMethod,
  QuantumChannelType,
  QuantumChannelConfig,
  FiberParameters,
  FreeSpaceParameters,
  SatelliteParameters,
  GeoLocation,
  QuantumLinkQuality,
  PhotonSourceType,
  PhotonSource,
  DetectorType,
  PhotonDetector,
  QuantumMemoryTech,
  QuantumRepeater,
  RepeaterChain,
  EntanglementSourceConfig,
  EntanglementPair,
  EntanglementDistributionParams,
  EntanglementDistributionResult,
  EntanglementSwapping,
  AuthenticationMethod,
  SecurityParameters,
  SecurityThreat,
  SideChannelMonitoring,
  SecurityAnomaly,
  SystemPerformanceMetrics,
  QBERMonitoring,
  PostQuantumAlgorithm,
  HybridQKDPQCConfig,
};

export {
  QUANTUM_COMM_CONSTANTS,
  QuantumCommunicationErrorCode,
  QuantumCommunicationError,
};

/**
 * 弘益人間 (Benefit All Humanity)
 */
