/**
 * WIA-QUA-003: Quantum Network - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Quantum Network Research Group
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
  basis?: 'computational' | 'hadamard' | 'bell';
}

/**
 * Qubit polarization state
 */
export type PolarizationState =
  | 'horizontal' // |0⟩ or |→⟩
  | 'vertical' // |1⟩ or |↑⟩
  | 'diagonal' // |+⟩ or |↗⟩
  | 'anti-diagonal'; // |−⟩ or |↖⟩

/**
 * Measurement basis
 */
export type MeasurementBasis =
  | 'rectilinear' // {|0⟩, |1⟩}
  | 'diagonal' // {|+⟩, |−⟩}
  | 'circular'; // {|L⟩, |R⟩}

/**
 * Bell state types
 */
export type BellState = 'phi-plus' | 'phi-minus' | 'psi-plus' | 'psi-minus';

// ============================================================================
// Quantum Key Distribution (QKD)
// ============================================================================

/**
 * QKD protocol types
 */
export type QKDProtocol = 'BB84' | 'E91' | 'B92' | 'BBM92' | 'SARG04';

/**
 * BB84 protocol parameters
 */
export interface BB84Parameters {
  /** Desired key length in bits */
  keyLength: number;

  /** Maximum acceptable quantum bit error rate (QBER) */
  errorThreshold?: number;

  /** Sender node identifier */
  sender: string;

  /** Receiver node identifier */
  receiver: string;

  /** Communication channel type */
  channel: 'fiber-optic' | 'free-space' | 'satellite';

  /** Enable privacy amplification */
  privacyAmplification?: boolean;

  /** Enable error correction */
  errorCorrection?: boolean;

  /** Measurement bases to use */
  bases?: MeasurementBasis[];
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
  errorRate: number;

  /** Security parameter (epsilon-security) */
  securityParameter: number;

  /** Raw key rate (bits/second) */
  rawKeyRate: number;

  /** Secure key rate after processing (bits/second) */
  secureKeyRate: number;

  /** Protocol used */
  protocol: QKDProtocol;

  /** Timestamp */
  timestamp: Date;

  /** Additional metadata */
  metadata?: {
    totalBitsSent: number;
    bitsAfterSifting: number;
    bitsAfterErrorCorrection: number;
    privacyAmplificationFactor: number;
  };
}

/**
 * E91 protocol parameters
 */
export interface E91Parameters {
  /** Desired key length in bits */
  keyLength: number;

  /** Sender node identifier */
  sender: string;

  /** Receiver node identifier */
  receiver: string;

  /** CHSH inequality threshold */
  chshThreshold?: number;

  /** Number of Bell pairs to generate */
  bellPairs: number;
}

/**
 * E91 result with Bell test
 */
export interface E91Result extends QKDResult {
  /** CHSH parameter S (should be > 2 for quantum) */
  chshParameter: number;

  /** Bell inequality violated? */
  bellViolation: boolean;

  /** Individual correlation measurements */
  correlations: {
    [key: string]: number;
  };
}

// ============================================================================
// Entanglement Distribution
// ============================================================================

/**
 * Entanglement pair
 */
export interface EntanglementPair {
  /** Unique identifier */
  id: string;

  /** First node */
  nodeA: string;

  /** Second node */
  nodeB: string;

  /** Bell state type */
  state: BellState;

  /** Fidelity (0-1) */
  fidelity: number;

  /** Creation timestamp */
  created: Date;

  /** Expiration time (if using quantum memory) */
  expires?: Date;

  /** Source of entanglement */
  source: 'SPDC' | 'quantum-dot' | 'trapped-ion' | 'NV-center';

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
  fidelity: number;

  /** Number of pairs to generate */
  pairs: number;

  /** Use purification */
  purification?: boolean;

  /** Maximum attempts */
  maxAttempts?: number;

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

  /** Total time taken (ms) */
  duration: number;

  /** Number of purification rounds */
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
}

// ============================================================================
// Quantum Repeaters
// ============================================================================

/**
 * Quantum repeater node
 */
export interface QuantumRepeater {
  /** Node identifier */
  id: string;

  /** Geographic position */
  position: {
    latitude: number;
    longitude: number;
    altitude?: number;
  };

  /** Quantum memory properties */
  memory: {
    technology: 'rare-earth' | 'quantum-dot' | 'atomic-ensemble' | 'NV-center' | 'trapped-ion';
    capacity: number; // number of qubits
    coherenceTime: number; // milliseconds
    efficiency: number; // 0-1
    fidelity: number; // 0-1
  };

  /** Connected neighbors */
  neighbors: string[];

  /** Current utilization */
  utilization: number; // 0-1

  /** Status */
  status: 'online' | 'offline' | 'maintenance' | 'degraded';

  /** Performance metrics */
  metrics: {
    entanglementRate: number; // pairs per second
    averageFidelity: number;
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
}

// ============================================================================
// Quantum Network
// ============================================================================

/**
 * Quantum network node
 */
export interface QuantumNode {
  /** Unique node identifier */
  id: string;

  /** Node name */
  name: string;

  /** Node type */
  type: 'end-node' | 'repeater' | 'router' | 'memory';

  /** Geographic location */
  location?: {
    latitude: number;
    longitude: number;
    altitude?: number;
    city?: string;
    country?: string;
  };

  /** Capabilities */
  capabilities: {
    qkd: boolean;
    entanglementGen: boolean;
    quantumMemory: boolean;
    bellMeasurement: boolean;
    repeater: boolean;
  };

  /** Connected links */
  links: string[];

  /** Status */
  status: 'active' | 'inactive' | 'maintenance';
}

/**
 * Quantum link between nodes
 */
export interface QuantumLink {
  /** Unique link identifier */
  id: string;

  /** Source node */
  nodeA: string;

  /** Destination node */
  nodeB: string;

  /** Distance (km) */
  distance: number;

  /** Channel type */
  channel: 'fiber-optic' | 'free-space' | 'satellite';

  /** Wavelength (nm) */
  wavelength: number;

  /** Link quality metrics */
  quality: {
    loss: number; // dB
    fidelity: number; // 0-1
    errorRate: number; // QBER
    throughput: number; // pairs/second
  };

  /** Link status */
  status: 'up' | 'down' | 'degraded';

  /** Last health check */
  lastCheck: Date;
}

/**
 * Quantum network topology
 */
export interface QuantumNetworkTopology {
  /** Network identifier */
  id: string;

  /** Network name */
  name: string;

  /** All nodes in network */
  nodes: Map<string, QuantumNode>;

  /** All links in network */
  links: Map<string, QuantumLink>;

  /** Adjacency information */
  adjacency: Map<string, string[]>;

  /** Network-wide metrics */
  metrics: {
    totalNodes: number;
    totalLinks: number;
    averageDegree: number;
    diameter: number;
    averageFidelity: number;
  };
}

// ============================================================================
// Quantum Routing
// ============================================================================

/**
 * Routing metrics
 */
export interface RoutingMetric {
  /** Metric type */
  type: 'fidelity' | 'latency' | 'throughput' | 'cost' | 'composite';

  /** Weight for this metric (0-1) */
  weight: number;
}

/**
 * Route in quantum network
 */
export interface QuantumRoute {
  /** Route identifier */
  id: string;

  /** Source node */
  source: string;

  /** Destination node */
  destination: string;

  /** Path (ordered list of node IDs) */
  path: string[];

  /** Links used */
  links: string[];

  /** Route metrics */
  metrics: {
    totalDistance: number; // km
    endToEndFidelity: number;
    expectedLatency: number; // ms
    throughput: number; // pairs/second
  };

  /** Route priority */
  priority: 'high' | 'normal' | 'low';

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

  /** Minimum required fidelity */
  minFidelity?: number;

  /** Maximum acceptable latency (ms) */
  maxLatency?: number;

  /** Required throughput (pairs/second) */
  requiredThroughput?: number;

  /** Routing algorithm */
  algorithm?: 'dijkstra' | 'fidelity-based' | 'latency-aware' | 'load-balanced';

  /** Metric preferences */
  metrics?: RoutingMetric[];
}

// ============================================================================
// Quantum Teleportation
// ============================================================================

/**
 * Quantum teleportation parameters
 */
export interface QuantumTeleportationParams {
  /** State to teleport */
  state: QuantumState;

  /** Sender node */
  from: string;

  /** Receiver node */
  to: string;

  /** Entanglement pair ID to use */
  entanglementId: string;

  /** Verify fidelity after teleportation */
  verify?: boolean;
}

/**
 * Quantum teleportation result
 */
export interface QuantumTeleportationResult {
  /** Operation identifier */
  id: string;

  /** Success status */
  success: boolean;

  /** Teleported state */
  state?: QuantumState;

  /** Fidelity of teleportation (if verified) */
  fidelity?: number;

  /** Bell measurement outcome */
  bellMeasurement: {
    basis: BellState;
    classicalBits: string; // "00", "01", "10", or "11"
  };

  /** Correction operation applied */
  correction: 'I' | 'X' | 'Z' | 'XZ';

  /** Latency (ms) */
  latency: number;

  /** Timestamp */
  timestamp: Date;
}

// ============================================================================
// Security and Authentication
// ============================================================================

/**
 * Authentication method
 */
export type AuthenticationMethod =
  | 'pre-shared-key'
  | 'wegman-carter'
  | 'post-quantum'
  | 'device-independent';

/**
 * Security parameters
 */
export interface SecurityParameters {
  /** Target security parameter (epsilon) */
  epsilon: number;

  /** Correctness parameter (probability of error) */
  correctness: number;

  /** Privacy amplification method */
  privacyAmplification: 'universal-hash' | 'toeplitz' | 'random-oracle';

  /** Error correction method */
  errorCorrection: 'cascade' | 'ldpc' | 'polar' | 'turbo';

  /** Authentication method */
  authentication: AuthenticationMethod;
}

/**
 * Side-channel attack detection
 */
export interface SideChannelMonitoring {
  /** Monitor photon number splitting */
  pns: boolean;

  /** Monitor Trojan horse attacks */
  trojanHorse: boolean;

  /** Monitor detector blinding */
  detectorBlinding: boolean;

  /** Alert threshold */
  alertThreshold: number;

  /** Current threat level */
  threatLevel: 'none' | 'low' | 'medium' | 'high' | 'critical';

  /** Detected anomalies */
  anomalies: Array<{
    type: string;
    severity: string;
    timestamp: Date;
    description: string;
  }>;
}

// ============================================================================
// Hardware Components
// ============================================================================

/**
 * Photon source
 */
export interface PhotonSource {
  /** Source identifier */
  id: string;

  /** Source type */
  type: 'SPDC' | 'quantum-dot' | 'single-photon' | 'attenuated-laser';

  /** Emission wavelength (nm) */
  wavelength: number;

  /** Brightness (pairs/s/mW for SPDC) */
  brightness: number;

  /** Heralding efficiency (0-1) */
  heraldingEfficiency?: number;

  /** Spectral purity */
  spectralPurity?: number;

  /** Indistinguishability */
  indistinguishability?: number;

  /** Temperature (K) */
  temperature?: number;

  /** Status */
  status: 'operational' | 'offline' | 'calibrating';
}

/**
 * Single-photon detector
 */
export interface PhotonDetector {
  /** Detector identifier */
  id: string;

  /** Detector type */
  type: 'SNSPD' | 'APD' | 'PMT' | 'TES';

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

  /** Status */
  status: 'operational' | 'offline' | 'saturated' | 'blinded';
}

// ============================================================================
// Performance Monitoring
// ============================================================================

/**
 * Network performance metrics
 */
export interface NetworkPerformanceMetrics {
  /** Timestamp */
  timestamp: Date;

  /** Overall network health (0-1) */
  health: number;

  /** Active nodes */
  activeNodes: number;

  /** Active links */
  activeLinks: number;

  /** Total entanglement rate (pairs/second) */
  entanglementRate: number;

  /** Average link fidelity */
  averageFidelity: number;

  /** Average QBER */
  averageQBER: number;

  /** Secret key rate (bits/second) */
  secretKeyRate: number;

  /** Network uptime (percentage) */
  uptime: number;

  /** Failed operations in last hour */
  failedOperations: number;
}

/**
 * Link quality measurement
 */
export interface LinkQualityMeasurement {
  /** Link identifier */
  linkId: string;

  /** Measurement timestamp */
  timestamp: Date;

  /** Measured fidelity */
  fidelity: number;

  /** QBER */
  qber: number;

  /** Photon loss (dB) */
  loss: number;

  /** Throughput (pairs/second) */
  throughput: number;

  /** Measurement duration (seconds) */
  duration: number;

  /** Status */
  status: 'healthy' | 'degraded' | 'failing';
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-QUA-003 error codes
 */
export enum QuantumNetworkErrorCode {
  INSUFFICIENT_FIDELITY = 'QN001',
  ENTANGLEMENT_FAILED = 'QN002',
  QKD_FAILED = 'QN003',
  HIGH_QBER = 'QN004',
  AUTHENTICATION_FAILED = 'QN005',
  ROUTING_FAILED = 'QN006',
  NODE_UNAVAILABLE = 'QN007',
  LINK_DOWN = 'QN008',
  QUANTUM_MEMORY_FULL = 'QN009',
  TELEPORTATION_FAILED = 'QN010',
  BELL_MEASUREMENT_FAILED = 'QN011',
  INVALID_PARAMETERS = 'QN012',
  SECURITY_VIOLATION = 'QN013',
  HARDWARE_FAILURE = 'QN014',
  TIMEOUT = 'QN015',
}

/**
 * Quantum network error
 */
export class QuantumNetworkError extends Error {
  constructor(
    public code: QuantumNetworkErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'QuantumNetworkError';
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
 * Physical constants for quantum networks
 */
export const QUANTUM_CONSTANTS = {
  /** Speed of light (m/s) */
  SPEED_OF_LIGHT: 299792458,

  /** Planck constant (J·s) */
  PLANCK: 6.62607015e-34,

  /** Fiber attenuation at 1550nm (dB/km) */
  FIBER_LOSS_1550: 0.2,

  /** Fiber attenuation at 1310nm (dB/km) */
  FIBER_LOSS_1310: 0.3,

  /** Maximum QBER for BB84 security (individual attacks) */
  MAX_QBER_BB84: 0.11,

  /** CHSH bound for quantum correlations */
  CHSH_QUANTUM: 2 * Math.sqrt(2),

  /** CHSH bound for classical correlations */
  CHSH_CLASSICAL: 2,

  /** Minimum useful entanglement fidelity */
  MIN_ENTANGLEMENT_FIDELITY: 0.5,

  /** Typical detector efficiency (SNSPD) */
  DETECTOR_EFFICIENCY_SNSPD: 0.85,

  /** Typical detector efficiency (APD) */
  DETECTOR_EFFICIENCY_APD: 0.2,
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
  QKDProtocol,
  BB84Parameters,
  QKDResult,
  E91Parameters,
  E91Result,
  EntanglementPair,
  EntanglementDistributionParams,
  EntanglementDistributionResult,
  EntanglementSwapping,
  QuantumRepeater,
  RepeaterChain,
  QuantumNode,
  QuantumLink,
  QuantumNetworkTopology,
  RoutingMetric,
  QuantumRoute,
  RoutingRequest,
  QuantumTeleportationParams,
  QuantumTeleportationResult,
  AuthenticationMethod,
  SecurityParameters,
  SideChannelMonitoring,
  PhotonSource,
  PhotonDetector,
  NetworkPerformanceMetrics,
  LinkQualityMeasurement,
};

export { QUANTUM_CONSTANTS, QuantumNetworkErrorCode, QuantumNetworkError };

/**
 * 弘익人間 (Benefit All Humanity)
 */
