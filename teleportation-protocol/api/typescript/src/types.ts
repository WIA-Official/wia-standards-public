/**
 * WIA-QUA-011: Teleportation Protocol - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Quantum Teleportation Research Group
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
  basis?: 'computational' | 'hadamard' | 'bell' | 'custom';

  /** Normalization check */
  normalized?: boolean;
}

/**
 * Bell state types
 */
export type BellStateType = 'phi-plus' | 'phi-minus' | 'psi-plus' | 'psi-minus';

/**
 * Pauli operators for correction
 */
export type PauliOperator = 'I' | 'X' | 'Y' | 'Z' | 'XZ';

/**
 * Measurement basis
 */
export type MeasurementBasis =
  | 'computational'
  | 'hadamard'
  | 'bell'
  | 'circular'
  | 'custom';

// ============================================================================
// Teleportation Protocol Types
// ============================================================================

/**
 * Teleportation protocol variants
 */
export type TeleportationProtocol =
  | 'standard'
  | 'multi-qubit'
  | 'continuous-variable'
  | 'high-fidelity'
  | 'fast'
  | 'long-distance';

/**
 * Quantum teleportation parameters
 */
export interface QuantumTeleportationParams {
  /** State to teleport */
  state: QuantumState;

  /** Sender node identifier */
  from: string;

  /** Receiver node identifier */
  to: string;

  /** Protocol variant to use */
  protocol?: TeleportationProtocol;

  /** Entanglement resource ID */
  entanglementId: string;

  /** Verify fidelity after teleportation */
  verifyFidelity?: boolean;

  /** Maximum allowed latency (ms) */
  maxLatency?: number;

  /** Minimum required fidelity */
  minFidelity?: number;

  /** Enable error correction */
  errorCorrection?: boolean;
}

/**
 * Bell state measurement result
 */
export interface BellMeasurementResult {
  /** Measured Bell state */
  basis: BellStateType;

  /** Classical bits (00, 01, 10, or 11) */
  classicalBits: string;

  /** Measurement fidelity */
  fidelity: number;

  /** Success indicator */
  success: boolean;

  /** Timestamp */
  timestamp: Date;

  /** Measurement method */
  method: 'linear-optics' | 'complete-bsm' | 'ion-trap' | 'superconducting';
}

/**
 * Quantum teleportation result
 */
export interface QuantumTeleportationResult {
  /** Operation identifier */
  id: string;

  /** Success status */
  success: boolean;

  /** Teleported state (if verification enabled) */
  state?: QuantumState;

  /** Teleportation fidelity */
  fidelity?: number;

  /** Bell measurement outcome */
  bellMeasurement: BellMeasurementResult;

  /** Correction operation applied */
  correction: PauliOperator;

  /** Classical communication latency (ms) */
  classicalLatency: number;

  /** Total operation latency (ms) */
  totalLatency: number;

  /** Entanglement consumed */
  entanglementConsumed: string[];

  /** Protocol used */
  protocol: TeleportationProtocol;

  /** Timestamp */
  timestamp: Date;

  /** Error information (if failed) */
  error?: string;
}

// ============================================================================
// Multi-Qubit Teleportation
// ============================================================================

/**
 * Multi-qubit teleportation parameters
 */
export interface MultiQubitTeleportationParams {
  /** States to teleport */
  states: QuantumState[];

  /** Sender node */
  from: string;

  /** Receiver node */
  to: string;

  /** Entangled pair IDs (one per qubit) */
  entanglementIds: string[];

  /** Teleport in parallel or sequential */
  mode?: 'parallel' | 'sequential';

  /** Verify final state */
  verify?: boolean;
}

/**
 * Multi-qubit teleportation result
 */
export interface MultiQubitTeleportationResult {
  /** Operation identifier */
  id: string;

  /** Overall success */
  success: boolean;

  /** Individual qubit results */
  qubitResults: QuantumTeleportationResult[];

  /** Combined fidelity */
  combinedFidelity: number;

  /** Total latency (ms) */
  totalLatency: number;

  /** Success rate */
  successRate: number;
}

// ============================================================================
// Continuous Variable Teleportation
// ============================================================================

/**
 * Continuous variable quantum state
 */
export interface CVQuantumState {
  /** Position quadrature expectation */
  position: number;

  /** Momentum quadrature expectation */
  momentum: number;

  /** Position variance */
  positionVariance?: number;

  /** Momentum variance */
  momentumVariance?: number;

  /** State description */
  description?: string;
}

/**
 * CV teleportation parameters
 */
export interface CVTeleportationParams {
  /** CV state to teleport */
  state?: CVQuantumState;

  /** Alternative: position quadrature */
  position?: number;

  /** Alternative: momentum quadrature */
  momentum?: number;

  /** Sender node */
  from: string;

  /** Receiver node */
  to: string;

  /** EPR squeezing (dB) */
  squeezing: number;

  /** Classical gain parameter */
  gain?: number;

  /** Verify fidelity */
  verify?: boolean;
}

/**
 * CV teleportation result
 */
export interface CVTeleportationResult {
  /** Operation identifier */
  id: string;

  /** Success status */
  success: boolean;

  /** Output state */
  state: CVQuantumState;

  /** Teleportation fidelity */
  fidelity: number;

  /** Measurement results */
  measurements: {
    positionSum: number;
    momentumDiff: number;
  };

  /** Displacement applied */
  displacement: {
    position: number;
    momentum: number;
  };

  /** Latency (ms) */
  latency: number;

  /** Timestamp */
  timestamp: Date;
}

// ============================================================================
// Entanglement Resources
// ============================================================================

/**
 * Entanglement pair for teleportation
 */
export interface EntanglementPair {
  /** Unique identifier */
  id: string;

  /** Node A */
  nodeA: string;

  /** Node B */
  nodeB: string;

  /** Bell state type */
  state: BellStateType;

  /** Fidelity (0-1) */
  fidelity: number;

  /** Creation timestamp */
  created: Date;

  /** Expiration time */
  expires?: Date;

  /** Source type */
  source: 'SPDC' | 'quantum-dot' | 'trapped-ion' | 'NV-center' | 'atomic-ensemble';

  /** Status */
  status: 'active' | 'reserved' | 'consumed' | 'expired' | 'degraded';

  /** Quality metrics */
  quality?: {
    concurrence?: number;
    entanglementOfFormation?: number;
    bellViolation?: number;
  };
}

/**
 * Entanglement generation parameters
 */
export interface EntanglementGenerationParams {
  /** Node A */
  nodeA: string;

  /** Node B */
  nodeB: string;

  /** Target fidelity */
  targetFidelity: number;

  /** Number of pairs to generate */
  count: number;

  /** Generation method */
  method?: 'SPDC' | 'quantum-dot' | 'trapped-ion';

  /** Apply purification */
  purification?: boolean;
}

// ============================================================================
// Teleportation Network
// ============================================================================

/**
 * Teleportation network node
 */
export interface TeleportationNode {
  /** Node identifier */
  id: string;

  /** Node name */
  name: string;

  /** Node type */
  type: 'sender' | 'receiver' | 'repeater' | 'router';

  /** Capabilities */
  capabilities: {
    bellMeasurement: boolean;
    quantumMemory: boolean;
    statePreparation: boolean;
    correction: boolean;
  };

  /** Status */
  status: 'online' | 'offline' | 'busy' | 'maintenance';

  /** Location */
  location?: {
    latitude: number;
    longitude: number;
    altitude?: number;
  };

  /** Available entanglement */
  availableEntanglement: string[];

  /** Current load */
  load: number; // 0-1
}

/**
 * Network topology for teleportation
 */
export type NetworkTopology = 'point-to-point' | 'star' | 'mesh' | 'hierarchical';

/**
 * Teleportation network parameters
 */
export interface TeleportationNetworkParams {
  /** Network nodes */
  nodes: string[];

  /** Network topology */
  topology: NetworkTopology;

  /** Entanglement source */
  entanglementSource: 'SPDC' | 'quantum-dot' | 'trapped-ion';

  /** Target fidelity */
  targetFidelity?: number;

  /** Enable quantum repeaters */
  repeaters?: boolean;
}

/**
 * Network teleportation parameters
 */
export interface NetworkTeleportationParams {
  /** State to teleport */
  state: QuantumState;

  /** Source node */
  from: string;

  /** Destination node */
  to: string;

  /** Routing strategy */
  routing?: 'shortest-path' | 'highest-fidelity' | 'lowest-latency';

  /** Maximum hops */
  maxHops?: number;
}

/**
 * Routing path
 */
export interface TeleportationPath {
  /** Source node */
  source: string;

  /** Destination node */
  destination: string;

  /** Intermediate nodes */
  path: string[];

  /** Entanglement IDs for each hop */
  entanglementIds: string[];

  /** Expected fidelity */
  expectedFidelity: number;

  /** Expected latency (ms) */
  expectedLatency: number;

  /** Number of hops */
  hops: number;
}

// ============================================================================
// Fidelity and Verification
// ============================================================================

/**
 * Fidelity calculation parameters
 */
export interface FidelityParams {
  /** Input state */
  inputState: QuantumState;

  /** Output state */
  outputState: QuantumState;

  /** Calculation method */
  method?: 'state-overlap' | 'process-tomography' | 'witness';
}

/**
 * Fidelity result
 */
export interface FidelityResult {
  /** Fidelity value (0-1) */
  fidelity: number;

  /** Exceeds classical limit (2/3) */
  quantumAdvantage: boolean;

  /** Calculation method used */
  method: string;

  /** Confidence interval */
  confidence?: {
    lower: number;
    upper: number;
  };

  /** Additional metrics */
  metrics?: {
    purity?: number;
    concurrence?: number;
    entropy?: number;
  };
}

/**
 * Verification protocol
 */
export type VerificationProtocol =
  | 'state-tomography'
  | 'process-tomography'
  | 'witness'
  | 'direct-fidelity'
  | 'randomized-benchmarking';

/**
 * Verification parameters
 */
export interface VerificationParams {
  /** State to verify */
  state: QuantumState;

  /** Reference state */
  reference: QuantumState;

  /** Verification protocol */
  protocol: VerificationProtocol;

  /** Number of measurements */
  measurements?: number;

  /** Confidence level */
  confidence?: number;
}

/**
 * Verification result
 */
export interface VerificationResult {
  /** Verified successfully */
  verified: boolean;

  /** Measured fidelity */
  fidelity: number;

  /** Protocol used */
  protocol: VerificationProtocol;

  /** Number of measurements performed */
  measurementCount: number;

  /** Confidence level achieved */
  confidence: number;

  /** Detailed results */
  details?: Record<string, unknown>;
}

// ============================================================================
// Performance Metrics
// ============================================================================

/**
 * Teleportation performance metrics
 */
export interface TeleportationMetrics {
  /** Total teleportations attempted */
  totalAttempts: number;

  /** Successful teleportations */
  successfulTeleportations: number;

  /** Success rate */
  successRate: number;

  /** Average fidelity */
  averageFidelity: number;

  /** Average latency (ms) */
  averageLatency: number;

  /** Throughput (teleportations per second) */
  throughput: number;

  /** Entanglement consumption rate */
  entanglementRate: number;

  /** Classical bandwidth usage (bits/s) */
  classicalBandwidth: number;

  /** Timestamp */
  timestamp: Date;
}

/**
 * Benchmark parameters
 */
export interface BenchmarkParams {
  /** Protocol to benchmark */
  protocol: TeleportationProtocol;

  /** Number of trials */
  trials: number;

  /** Duration (seconds) */
  duration?: number;

  /** State types to test */
  stateTypes?: ('computational' | 'hadamard' | 'random')[];

  /** Collect detailed statistics */
  detailed?: boolean;
}

/**
 * Benchmark result
 */
export interface BenchmarkResult {
  /** Protocol tested */
  protocol: TeleportationProtocol;

  /** Number of trials completed */
  trials: number;

  /** Duration (seconds) */
  duration: number;

  /** Performance metrics */
  metrics: TeleportationMetrics;

  /** Fidelity distribution */
  fidelityDistribution?: {
    min: number;
    max: number;
    mean: number;
    median: number;
    stdDev: number;
  };

  /** Latency distribution */
  latencyDistribution?: {
    min: number;
    max: number;
    mean: number;
    median: number;
    stdDev: number;
  };
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-QUA-011 error codes
 */
export enum TeleportationErrorCode {
  INVALID_STATE = 'TP001',
  ENTANGLEMENT_UNAVAILABLE = 'TP002',
  BSM_FAILED = 'TP003',
  CORRECTION_FAILED = 'TP004',
  CLASSICAL_COMM_FAILED = 'TP005',
  VERIFICATION_FAILED = 'TP006',
  LOW_FIDELITY = 'TP007',
  TIMEOUT = 'TP008',
  NODE_UNAVAILABLE = 'TP009',
  NETWORK_ERROR = 'TP010',
  INSUFFICIENT_RESOURCES = 'TP011',
  PROTOCOL_ERROR = 'TP012',
  INVALID_PARAMETERS = 'TP013',
}

/**
 * Teleportation error
 */
export class TeleportationError extends Error {
  constructor(
    public code: TeleportationErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'TeleportationError';
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
 * Physical constants for teleportation
 */
export const TELEPORTATION_CONSTANTS = {
  /** Speed of light (m/s) */
  SPEED_OF_LIGHT: 299792458,

  /** Planck constant (J·s) */
  PLANCK: 6.62607015e-34,

  /** Classical fidelity limit */
  CLASSICAL_FIDELITY_LIMIT: 2 / 3,

  /** Minimum entanglement fidelity for quantum advantage */
  MIN_ENTANGLEMENT_FIDELITY: 0.5,

  /** Perfect fidelity threshold */
  PERFECT_FIDELITY_THRESHOLD: 0.99,

  /** Typical BSM success probability (linear optics) */
  LINEAR_OPTICS_BSM_SUCCESS: 0.5,

  /** Typical BSM success probability (complete) */
  COMPLETE_BSM_SUCCESS: 1.0,

  /** Standard number of classical bits per qubit */
  CLASSICAL_BITS_PER_QUBIT: 2,

  /** Vacuum permittivity (F/m) */
  VACUUM_PERMITTIVITY: 8.854187817e-12,

  /** Reduced Planck constant (J·s) */
  HBAR: 1.054571817e-34,
} as const;

// ============================================================================
// Export All Types
// ============================================================================

export type {
  Complex,
  QuantumState,
  BellStateType,
  PauliOperator,
  MeasurementBasis,
  TeleportationProtocol,
  QuantumTeleportationParams,
  BellMeasurementResult,
  QuantumTeleportationResult,
  MultiQubitTeleportationParams,
  MultiQubitTeleportationResult,
  CVQuantumState,
  CVTeleportationParams,
  CVTeleportationResult,
  EntanglementPair,
  EntanglementGenerationParams,
  TeleportationNode,
  NetworkTopology,
  TeleportationNetworkParams,
  NetworkTeleportationParams,
  TeleportationPath,
  FidelityParams,
  FidelityResult,
  VerificationProtocol,
  VerificationParams,
  VerificationResult,
  TeleportationMetrics,
  BenchmarkParams,
  BenchmarkResult,
};

export { TELEPORTATION_CONSTANTS, TeleportationErrorCode, TeleportationError };

/**
 * 弘益人間 (Benefit All Humanity)
 */
