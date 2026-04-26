/**
 * WIA-QUA-016: FTL Communication - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Future Technologies Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core FTL Types
// ============================================================================

/**
 * FTL communication methods
 */
export type FTLMethod =
  | 'quantum-entanglement'
  | 'tachyonic'
  | 'alcubierre'
  | 'subspace'
  | 'wormhole'
  | 'ansible';

/**
 * Communication status
 */
export type CommunicationStatus =
  | 'initializing'
  | 'ready'
  | 'transmitting'
  | 'receiving'
  | 'processing'
  | 'complete'
  | 'failed'
  | 'timeout';

/**
 * Causality constraint mode
 */
export type CausalityMode =
  | 'strict' // Reject any potential causality violations
  | 'monitored' // Allow but log violations
  | 'unconstrained'; // No causality checks (dangerous!)

// ============================================================================
// Quantum Entanglement Types
// ============================================================================

/**
 * Quantum entanglement protocol (theoretical extensions)
 */
export type QuantumProtocol =
  | 'standard-epr' // Standard entanglement (no FTL info)
  | 'extended-epr' // Hypothetical extension (theoretical only)
  | 'nonlinear-qm' // Non-linear quantum mechanics (speculative)
  | 'pilot-wave-ftl'; // Modified Bohmian mechanics (speculative)

/**
 * Quantum channel configuration
 */
export interface QuantumChannelConfig {
  /** Source node identifier */
  source: string;

  /** Destination node identifier */
  destination: string;

  /** Distance in light-years */
  distance: number;

  /** Quantum protocol to use */
  protocol: QuantumProtocol;

  /** Number of entangled pairs */
  entanglementDensity: number;

  /** Target fidelity (0-1) */
  fidelity?: number;

  /** Enable error correction */
  errorCorrection?: boolean;
}

/**
 * Quantum transmission result
 */
export interface QuantumTransmissionResult {
  /** Unique transmission ID */
  id: string;

  /** Transmission status */
  status: CommunicationStatus;

  /** Message payload */
  message?: string;

  /** Bits transmitted */
  bitsTransmitted: number;

  /** Theoretical latency (should be 0 for true FTL) */
  latency: number; // milliseconds

  /** Fidelity achieved */
  fidelity: number;

  /** Causality status */
  causalityStatus: 'preserved' | 'violated' | 'unknown';

  /** Timestamp */
  timestamp: Date;
}

// ============================================================================
// Tachyonic Communication Types
// ============================================================================

/**
 * Tachyon field properties
 */
export interface TachyonField {
  /** Field strength in GeV */
  fieldStrength: number;

  /** Tachyon rest mass (imaginary, represented as negative) */
  massParameter: number;

  /** Velocity in units of c */
  velocity: number;

  /** Modulation scheme */
  modulation: 'amplitude' | 'frequency' | 'phase';
}

/**
 * Tachyonic channel configuration
 */
export interface TachyonicChannelConfig {
  /** Field strength in GeV */
  fieldStrength: number;

  /** Imaginary mass parameter (negative value) */
  massParameter: number;

  /** Modulation scheme */
  modulation: 'amplitude' | 'frequency' | 'phase';

  /** Carrier frequency (Hz) */
  carrierFrequency?: number;

  /** Bandwidth (Hz) */
  bandwidth?: number;

  /** Beam directionality (0-1, 1 = perfect beam) */
  directionality?: number;
}

/**
 * Tachyonic transmission parameters
 */
export interface TachyonicTransmission {
  /** Message to transmit */
  message: string | Buffer;

  /** Target coordinates (x, y, z in light-years) */
  target: [number, number, number];

  /** Priority level */
  priority: 'low' | 'normal' | 'high' | 'critical';

  /** Encoding scheme */
  encoding?: 'binary' | 'quaternary' | 'octal';

  /** Error correction code */
  errorCorrection?: 'hamming' | 'reed-solomon' | 'turbo' | 'ldpc';
}

/**
 * Tachyonic transmission result
 */
export interface TachyonicResult {
  /** Transmission ID */
  id: string;

  /** Status */
  status: CommunicationStatus;

  /** Energy consumed (Joules) */
  energyUsed: number;

  /** Tachyon particles emitted */
  particlesEmitted: number;

  /** Estimated arrival time (theoretical: 0 for FTL) */
  arrivalTime: Date;

  /** Causality check result */
  causalityViolation: boolean;

  /** Frame-dependent information */
  frameDependent?: {
    referenceFrame: string;
    temporalOrder: 'forward' | 'backward' | 'simultaneous';
  };
}

// ============================================================================
// Alcubierre Metric Types
// ============================================================================

/**
 * Warp bubble parameters
 */
export interface WarpBubble {
  /** Bubble radius (meters) */
  radius: number;

  /** Warp factor (effective velocity = warp * c) */
  warpFactor: number;

  /** Wall thickness parameter (sigma) */
  wallThickness: number;

  /** Bubble velocity (m/s) */
  velocity: number;

  /** Bubble position (x, y, z in light-years) */
  position: [number, number, number];
}

/**
 * Alcubierre channel configuration
 */
export interface AlcubierreChannelConfig {
  /** Source location (x, y, z in light-years) */
  sourceLocation: [number, number, number];

  /** Target location (x, y, z in light-years) */
  targetLocation: [number, number, number];

  /** Warp factor (velocity multiplier) */
  warpFactor: number;

  /** Negative energy density required (J/m³) */
  negativeEnergyDensity: number;

  /** Bubble radius (meters) */
  bubbleRadius?: number;

  /** Metric modulation for encoding */
  metricModulation?: 'amplitude' | 'frequency' | 'shape';
}

/**
 * Alcubierre signal
 */
export interface AlcubierreSignal {
  /** Signal data */
  data: Buffer;

  /** Compression ratio */
  compressionRatio: number;

  /** Error correction */
  errorCorrection: 'quantum-reed-solomon' | 'topological' | 'concatenated';

  /** Modulation parameters */
  modulation?: {
    type: 'metric-perturbation' | 'warp-oscillation';
    frequency: number;
    amplitude: number;
  };
}

/**
 * Alcubierre transmission result
 */
export interface AlcubierreResult {
  /** Transmission ID */
  id: string;

  /** Status */
  status: CommunicationStatus;

  /** Exotic matter used (kg equivalent) */
  exoticMatterUsed: number;

  /** Energy consumed (Joules) */
  energyUsed: number;

  /** Warp bubble stability */
  bubbleStability: number; // 0-1

  /** Metric perturbation amplitude */
  perturbationAmplitude: number;

  /** CTC (closed timelike curve) detected */
  ctcDetected: boolean;

  /** Hawking radiation effects */
  hawkingRadiation?: number; // W/m²
}

// ============================================================================
// Subspace Communication Types
// ============================================================================

/**
 * Extra dimensional configuration
 */
export interface ExtraDimensions {
  /** Number of extra dimensions */
  dimensions: number;

  /** Compactification radius (meters) */
  compactificationRadius: number;

  /** Extra dimension topology */
  topology: 'circle' | 'torus' | 'calabi-yau' | 'orbifold';

  /** Kaluza-Klein mode number */
  kkMode?: number;
}

/**
 * Subspace channel configuration
 */
export interface SubspaceChannelConfig {
  /** Number of extra dimensions to use */
  extraDimensions: number;

  /** Compactification scale (meters) */
  compactificationScale: number;

  /** Bulk path coordinates */
  bulkPath?: number[];

  /** Brane separation (if applicable) */
  braneSeparation?: number;

  /** Coupling strength to Standard Model */
  couplingStrength: number;
}

/**
 * Subspace transmission parameters
 */
export interface SubspaceTransmission {
  /** Message payload */
  payload: string | Buffer;

  /** Priority */
  priority: 'low' | 'normal' | 'high' | 'emergency';

  /** KK mode selection */
  kkModes?: number[];

  /** Bulk trajectory optimization */
  trajectoryOptimization?: 'shortest-path' | 'lowest-energy' | 'highest-fidelity';
}

/**
 * Subspace transmission result
 */
export interface SubspaceResult {
  /** Transmission ID */
  id: string;

  /** Status */
  status: CommunicationStatus;

  /** Energy to access bulk (Joules) */
  bulkAccessEnergy: number;

  /** KK modes excited */
  modesExcited: number[];

  /** Bulk scattering events */
  scatteringEvents: number;

  /** Signal degradation (0-1) */
  degradation: number;

  /** Extra-dimensional path length */
  bulkPathLength: number;
}

// ============================================================================
// Network Topology Types
// ============================================================================

/**
 * FTL network node
 */
export interface FTLNode {
  /** Node identifier */
  id: string;

  /** Node name */
  name: string;

  /** Node type */
  type: 'station' | 'relay' | 'hub' | 'gateway';

  /** Position (x, y, z in light-years) */
  position: [number, number, number];

  /** Supported FTL methods */
  supportedMethods: FTLMethod[];

  /** Connected neighbors */
  neighbors: string[];

  /** Node status */
  status: 'online' | 'offline' | 'maintenance' | 'degraded';

  /** Capabilities */
  capabilities: {
    maxDataRate: number; // bps
    maxRange: number; // light-years
    energyCapacity: number; // Joules
    simultaneousChannels: number;
  };
}

/**
 * FTL communication link
 */
export interface FTLLink {
  /** Link identifier */
  id: string;

  /** Source node */
  nodeA: string;

  /** Destination node */
  nodeB: string;

  /** FTL method used */
  method: FTLMethod;

  /** Distance (light-years) */
  distance: number;

  /** Link quality metrics */
  quality: {
    reliability: number; // 0-1
    signalIntegrity: number; // 0-1
    energyEfficiency: number; // bits per Joule
    errorRate: number; // bit error rate
  };

  /** Link status */
  status: 'active' | 'inactive' | 'congested' | 'failed';

  /** Last health check */
  lastHealthCheck: Date;
}

/**
 * Network topology
 */
export type NetworkTopology =
  | 'point-to-point'
  | 'hub-and-spoke'
  | 'mesh'
  | 'hierarchical'
  | 'hybrid';

/**
 * FTL network configuration
 */
export interface FTLNetworkConfig {
  /** Network identifier */
  id: string;

  /** Network name */
  name: string;

  /** Topology type */
  topology: NetworkTopology;

  /** All nodes in network */
  nodes: FTLNode[];

  /** All links in network */
  links: FTLLink[];

  /** Causality mode */
  causalityMode: CausalityMode;

  /** Preferred reference frame (if applicable) */
  preferredFrame?: string;

  /** Network-wide settings */
  settings: {
    enableLoadBalancing: boolean;
    enableCausalityChecks: boolean;
    emergencyShutdownEnabled: boolean;
    maxConcurrentTransmissions: number;
  };
}

// ============================================================================
// Routing and Pathfinding Types
// ============================================================================

/**
 * Routing algorithm
 */
export type RoutingAlgorithm =
  | 'dijkstra'
  | 'energy-optimal'
  | 'reliability-first'
  | 'causality-aware'
  | 'load-balanced';

/**
 * Route in FTL network
 */
export interface FTLRoute {
  /** Route identifier */
  id: string;

  /** Source node */
  source: string;

  /** Destination node */
  destination: string;

  /** Path (ordered node IDs) */
  path: string[];

  /** Links used */
  links: string[];

  /** Route metrics */
  metrics: {
    totalDistance: number; // light-years
    energyRequired: number; // Joules
    reliability: number; // 0-1
    expectedLatency: number; // ms (should be ~0 for true FTL)
  };

  /** Causality status */
  causalityStatus: 'safe' | 'risky' | 'violation';

  /** Established timestamp */
  established: Date;
}

/**
 * Routing request
 */
export interface RoutingRequest {
  /** Source node */
  source: string;

  /** Destination node */
  destination: string;

  /** Routing algorithm preference */
  algorithm?: RoutingAlgorithm;

  /** Constraints */
  constraints?: {
    maxEnergy?: number;
    minReliability?: number;
    forbiddenNodes?: string[];
    requiredMethod?: FTLMethod;
  };

  /** Priority */
  priority?: 'low' | 'normal' | 'high' | 'critical';
}

// ============================================================================
// Causality and Temporal Types
// ============================================================================

/**
 * Spacetime coordinates
 */
export interface SpacetimeCoordinates {
  /** Time coordinate (seconds) */
  t: number;

  /** Spatial coordinates (light-years) */
  x: number;
  y: number;
  z: number;

  /** Reference frame */
  frame?: string;
}

/**
 * Spacetime interval type
 */
export type IntervalType = 'timelike' | 'spacelike' | 'null';

/**
 * Causality analysis
 */
export interface CausalityAnalysis {
  /** Event A coordinates */
  eventA: SpacetimeCoordinates;

  /** Event B coordinates */
  eventB: SpacetimeCoordinates;

  /** Spacetime interval */
  interval: number;

  /** Interval type */
  intervalType: IntervalType;

  /** Causally connected? */
  causallyConnected: boolean;

  /** CTC detected? */
  ctcDetected: boolean;

  /** Temporal ordering (frame-dependent for spacelike) */
  ordering: {
    frame: string;
    order: 'A-before-B' | 'B-before-A' | 'simultaneous';
  }[];

  /** Paradox risk */
  paradoxRisk: 'none' | 'low' | 'medium' | 'high' | 'certain';
}

/**
 * Temporal paradox types
 */
export type ParadoxType =
  | 'grandfather'
  | 'bootstrap'
  | 'predestination'
  | 'ontological'
  | 'causal-loop';

/**
 * Paradox detection result
 */
export interface ParadoxDetection {
  /** Paradox detected? */
  detected: boolean;

  /** Paradox type (if detected) */
  type?: ParadoxType;

  /** Severity */
  severity: 'none' | 'low' | 'medium' | 'high' | 'catastrophic';

  /** Description */
  description: string;

  /** Recommended action */
  recommendation: 'allow' | 'warn' | 'block' | 'shutdown';
}

// ============================================================================
// Energy and Resource Types
// ============================================================================

/**
 * Energy calculation
 */
export interface EnergyCalculation {
  /** FTL method */
  method: FTLMethod;

  /** Distance (light-years) */
  distance: number;

  /** Message size (bits) */
  messageSize: number;

  /** Energy required (Joules) */
  energyRequired: number;

  /** Power required (Watts) */
  powerRequired: number;

  /** Exotic matter needed (kg, if applicable) */
  exoticMatter?: number;

  /** Feasibility assessment */
  feasibility: 'practical' | 'challenging' | 'extremely-difficult' | 'impossible';

  /** Comparison to available resources */
  comparison: {
    solarMassEquivalent?: number;
    globalEnergyYears?: number;
    particleAcceleratorYears?: number;
  };
}

/**
 * Resource availability
 */
export interface ResourceAvailability {
  /** Available energy (Joules) */
  availableEnergy: number;

  /** Exotic matter reserves (kg, if applicable) */
  exoticMatter?: number;

  /** Quantum entanglement pairs available */
  entanglementPairs?: number;

  /** Current utilization (0-1) */
  utilization: number;

  /** Recharge rate (W) */
  rechargeRate?: number;
}

// ============================================================================
// Error Correction Types
// ============================================================================

/**
 * Error correction code
 */
export type ErrorCorrectionCode =
  | 'hamming'
  | 'reed-solomon'
  | 'turbo'
  | 'ldpc'
  | 'polar'
  | 'quantum-steane'
  | 'quantum-surface'
  | 'quantum-concatenated';

/**
 * Error correction parameters
 */
export interface ErrorCorrectionParams {
  /** Code type */
  code: ErrorCorrectionCode;

  /** Code rate (information bits / total bits) */
  codeRate: number;

  /** Correctable errors */
  correctableErrors: number;

  /** Overhead (redundancy) */
  overhead: number; // factor, e.g., 2.0 = 100% overhead
}

/**
 * Channel noise model
 */
export interface NoiseModel {
  /** Noise type */
  type: 'gaussian' | 'poisson' | 'burst' | 'quantum-decoherence' | 'metric-fluctuation';

  /** Noise power spectral density (W/Hz) */
  powerSpectralDensity: number;

  /** Signal-to-noise ratio (dB) */
  snr: number;

  /** Bit error rate */
  bitErrorRate: number;
}

// ============================================================================
// Monitoring and Diagnostics Types
// ============================================================================

/**
 * System health status
 */
export interface SystemHealth {
  /** Overall health (0-1) */
  overall: number;

  /** Component status */
  components: {
    transmitter: 'operational' | 'degraded' | 'failed';
    receiver: 'operational' | 'degraded' | 'failed';
    powerSystem: 'operational' | 'degraded' | 'failed';
    causalityMonitor: 'operational' | 'degraded' | 'failed';
  };

  /** Active warnings */
  warnings: string[];

  /** Active errors */
  errors: string[];

  /** Last diagnostic */
  lastDiagnostic: Date;
}

/**
 * Performance metrics
 */
export interface PerformanceMetrics {
  /** Timestamp */
  timestamp: Date;

  /** Data rate (bps) */
  dataRate: number;

  /** Throughput (bps) */
  throughput: number;

  /** Latency (ms) */
  latency: number;

  /** Packet loss rate */
  packetLoss: number;

  /** Energy efficiency (bits/Joule) */
  energyEfficiency: number;

  /** Success rate */
  successRate: number;

  /** Uptime (percentage) */
  uptime: number;
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-QUA-016 error codes
 */
export enum FTLErrorCode {
  CAUSALITY_VIOLATION = 'FTL001',
  INSUFFICIENT_ENERGY = 'FTL002',
  EXOTIC_MATTER_UNAVAILABLE = 'FTL003',
  CHANNEL_UNAVAILABLE = 'FTL004',
  TRANSMISSION_FAILED = 'FTL005',
  CTC_DETECTED = 'FTL006',
  PARADOX_RISK = 'FTL007',
  NODE_UNREACHABLE = 'FTL008',
  METHOD_UNSUPPORTED = 'FTL009',
  SIGNAL_DEGRADED = 'FTL010',
  DECODING_FAILED = 'FTL011',
  INVALID_COORDINATES = 'FTL012',
  METRIC_INSTABILITY = 'FTL013',
  VACUUM_DECAY_RISK = 'FTL014',
  THEORETICAL_LIMITATION = 'FTL015',
}

/**
 * FTL communication error
 */
export class FTLError extends Error {
  constructor(
    public code: FTLErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'FTLError';
  }
}

// ============================================================================
// Configuration and Options Types
// ============================================================================

/**
 * SDK configuration
 */
export interface FTLSDKConfig {
  /** Enable debug logging */
  debug?: boolean;

  /** Causality mode */
  causalityMode?: CausalityMode;

  /** Preferred FTL method */
  preferredMethod?: FTLMethod;

  /** Default error correction */
  defaultErrorCorrection?: ErrorCorrectionCode;

  /** Safety settings */
  safety?: {
    enableCTCPrevention: boolean;
    enableParadoxChecks: boolean;
    maxEnergyPerTransmission: number;
    emergencyShutdownThreshold: number;
  };

  /** Performance settings */
  performance?: {
    maxConcurrentTransmissions: number;
    transmissionTimeout: number; // ms
    retryAttempts: number;
    cacheResults: boolean;
  };
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

/**
 * Event callback
 */
export type EventCallback<T = unknown> = (data: T) => void;

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Physical constants for FTL communication
 */
export const FTL_CONSTANTS = {
  /** Speed of light (m/s) */
  SPEED_OF_LIGHT: 299792458,

  /** Planck constant (J·s) */
  PLANCK: 6.62607015e-34,

  /** Planck length (m) */
  PLANCK_LENGTH: 1.616255e-35,

  /** Planck energy (GeV) */
  PLANCK_ENERGY: 1.22091e19,

  /** Gravitational constant (m³/kg·s²) */
  GRAVITATIONAL_CONSTANT: 6.67430e-11,

  /** Light-year (m) */
  LIGHT_YEAR: 9.46073e15,

  /** Parsec (m) */
  PARSEC: 3.08567758e16,

  /** Solar mass (kg) */
  SOLAR_MASS: 1.98892e30,

  /** Electron mass (kg) */
  ELECTRON_MASS: 9.10938e-31,

  /** Proton mass (kg) */
  PROTON_MASS: 1.67262e-27,

  /** Fine structure constant */
  FINE_STRUCTURE: 7.2973525693e-3,

  /** Max QBER for quantum communication */
  MAX_QBER: 0.11,

  /** Minimum entanglement fidelity */
  MIN_FIDELITY: 0.5,

  /** Typical Alcubierre energy (J) - unattainable */
  ALCUBIERRE_ENERGY: 1e45,

  /** Casimir energy density (J/m³, negative) */
  CASIMIR_ENERGY_DENSITY: -1e-3,
} as const;

// ============================================================================
// Export All Types
// ============================================================================

export type {
  FTLMethod,
  CommunicationStatus,
  CausalityMode,
  QuantumProtocol,
  QuantumChannelConfig,
  QuantumTransmissionResult,
  TachyonField,
  TachyonicChannelConfig,
  TachyonicTransmission,
  TachyonicResult,
  WarpBubble,
  AlcubierreChannelConfig,
  AlcubierreSignal,
  AlcubierreResult,
  ExtraDimensions,
  SubspaceChannelConfig,
  SubspaceTransmission,
  SubspaceResult,
  FTLNode,
  FTLLink,
  NetworkTopology,
  FTLNetworkConfig,
  RoutingAlgorithm,
  FTLRoute,
  RoutingRequest,
  SpacetimeCoordinates,
  IntervalType,
  CausalityAnalysis,
  ParadoxType,
  ParadoxDetection,
  EnergyCalculation,
  ResourceAvailability,
  ErrorCorrectionCode,
  ErrorCorrectionParams,
  NoiseModel,
  SystemHealth,
  PerformanceMetrics,
  FTLSDKConfig,
};

export { FTL_CONSTANTS, FTLErrorCode, FTLError };

/**
 * 弘益人間 (Benefit All Humanity)
 */
