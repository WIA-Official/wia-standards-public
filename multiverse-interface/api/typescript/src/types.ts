/**
 * WIA-QUA-017: Multiverse Interface - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Quantum Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * Four-dimensional spacetime coordinate
 */
export interface Spacetime4D {
  x: number;        // meters
  y: number;        // meters
  z: number;        // meters
  t: number;        // seconds
}

/**
 * Quantum state vector representation
 */
export interface QuantumState {
  representation: string;     // e.g., "|ψ⟩ = α|0⟩ + β|1⟩"
  dimensions: number;         // Hilbert space dimension
  amplitudes: Complex[];      // Probability amplitudes
  normalization: number;      // Should be 1.0
  entropy: number;            // Von Neumann entropy
  purity: number;             // Tr(ρ²), 1 = pure, < 1 = mixed
}

/**
 * Complex number
 */
export interface Complex {
  real: number;
  imaginary: number;
}

/**
 * Probability distribution over universe branches
 */
export interface ProbabilityDistribution {
  universe: UniverseAddress;
  amplitude: Complex;
  probability: number;        // |amplitude|²
  phase: number;              // arg(amplitude)
}

// ============================================================================
// Universe Addressing
// ============================================================================

/**
 * Unique address for a universe in the multiverse
 */
export interface UniverseAddress {
  // Primary identifier (SHA-256 hash of branching event)
  branchId: string;

  // Temporal coordinate
  timelineId: string;         // Format: T±offset (e.g., "T+142857")
  epoch?: number;             // Unix timestamp of divergence point

  // Quantum state descriptor
  realityState: {
    dimensions: number;       // Spacetime dimensions (typically 4)
    quantumState: string;     // Human-readable state description
    entropy: number;          // Universe entropy level (J/K)
    temperature?: number;     // Cosmic background temperature (K)
  };

  // Optional metadata
  metadata?: {
    label?: string;           // Human-friendly name
    discovered?: Date | string;
    stability?: number;       // 0-1 scale
    accessibility?: number;   // 0-1 scale
    tags?: string[];
  };
}

/**
 * Universe address with validation
 */
export interface ValidatedUniverseAddress extends UniverseAddress {
  valid: boolean;
  validationErrors: string[];
  validatedAt: Date | string;
}

// ============================================================================
// Reality Anchors
// ============================================================================

/**
 * Stable reference point in a universe
 */
export interface RealityAnchor {
  // Identification
  id: string;
  universe: UniverseAddress;
  createdAt: Date | string;

  // Stability metrics
  coherenceTime: number;      // seconds
  stability: number;          // 0-1 scale
  decayRate: number;          // per second
  halfLife: number;           // seconds

  // Physical properties
  location: {
    spacetime: Spacetime4D;
    quantumState: QuantumState;
    energy: number;           // Joules
  };

  // Maintenance
  lastRefresh: Date | string;
  nextMaintenanceRequired: Date | string;
  energyRequirement: number;  // Joules per refresh
  maintenanceInterval: number; // seconds

  // Status
  status: 'active' | 'decaying' | 'failed' | 'refreshing';
  health: number;             // 0-1 scale
}

/**
 * Configuration for creating a reality anchor
 */
export interface AnchorConfig {
  universe: UniverseAddress;
  targetStability?: number;   // Default: 0.95
  coherenceTime?: number;     // Default: 3600 seconds
  location?: Partial<Spacetime4D>;
  energyBudget?: number;      // Maximum energy in Joules
  autoRefresh?: boolean;      // Default: true
}

/**
 * Result of anchor creation
 */
export interface AnchorCreationResult {
  success: boolean;
  anchor?: RealityAnchor;
  error?: string;
  cost: {
    energy: number;           // Joules
    time: number;             // seconds
  };
}

// ============================================================================
// Divergence & Timeline
// ============================================================================

/**
 * Measurement of difference between universes
 */
export interface DivergenceMetric {
  // Primary metric (0 = identical, 1 = maximally different)
  metric: number;

  // Decomposition
  components: {
    quantum: number;          // Quantum state difference
    macroscopic: number;      // Observable physical differences
    historical: number;       // Timeline event differences
    information: number;      // Information-theoretic distance
  };

  // Statistical confidence
  confidence: number;         // 0-1 scale
  uncertainty: number;        // Measurement uncertainty
  sampleSize: number;         // Number of measurements
}

/**
 * Event that caused universe branching
 */
export interface BranchingEvent {
  // Identification
  eventId: string;
  timestamp: Date | string;

  // Parent-child relationship
  parentUniverse: UniverseAddress;
  childUniverses: UniverseAddress[];
  branchCount: number;

  // Event classification
  type: 'quantum-measurement' | 'decoherence' | 'macro-decision' | 'cosmic-event' | 'unknown';
  subtype?: string;

  // Physical details
  trigger?: {
    description: string;
    location: Spacetime4D;
    observables: Record<string, number>;
  };

  // Quantum mechanics
  quantumAmplitudes: ProbabilityDistribution[];
  totalProbability: number;   // Should be 1.0
  entanglementStrength: number; // 0-1 scale

  // Metadata
  significance: number;       // 0-1 scale (how important is this branching)
  reversibility: number;      // 0-1 scale (can branches re-converge)
}

/**
 * Timeline representation
 */
export interface Timeline {
  // Identification
  id: string;
  universe: UniverseAddress;

  // Temporal structure
  origin: {
    timestamp: Date | string;
    event: string;
  };
  current: {
    timestamp: Date | string;
    state: QuantumState;
  };

  // Branching history
  branchingEvents: BranchingEvent[];
  parentTimeline?: Timeline;
  childTimelines: Timeline[];

  // Properties
  stability: number;          // 0-1 scale
  coherence: number;          // 0-1 scale
  entropy: number;            // Boltzmann entropy
}

/**
 * Configuration for divergence detection
 */
export interface DivergenceConfig {
  referenceUniverse: UniverseAddress;
  targetUniverse: UniverseAddress;
  measurementDepth?: number;  // How many quantum measurements
  includeHistory?: boolean;   // Analyze historical events
  precision?: number;         // Measurement precision (0-1)
}

// ============================================================================
// Multiverse Navigation
// ============================================================================

/**
 * Configuration for multiverse navigator
 */
export interface NavigatorConfig {
  // Origin universe
  originUniverse: UniverseAddress | 'current';

  // Search parameters
  coherenceThreshold?: number;  // Minimum coherence (default: 0.9)
  maxDivergence?: number;       // Maximum divergence (default: 0.5)
  searchDepth?: number;         // Branching depth to explore (default: 10)

  // Performance
  maxConcurrentScans?: number;  // Parallel universe scans (default: 100)
  scanTimeout?: number;         // Milliseconds (default: 30000)

  // Caching
  enableCache?: boolean;        // Default: true
  cacheSize?: number;           // Maximum cached universes (default: 10000)
  cacheTTL?: number;            // Cache time-to-live in seconds (default: 3600)
}

/**
 * Result of universe scan
 */
export interface UniverseScanResult {
  // Summary
  totalUniverses: number;
  accessibleUniverses: number;
  scanDuration: number;         // milliseconds

  // Discovered universes
  universes: {
    address: UniverseAddress;
    divergence: DivergenceMetric;
    accessibility: number;      // 0-1 scale
    stability: number;          // 0-1 scale
  }[];

  // Branching structure
  branchingTree: {
    parent: UniverseAddress | null;
    children: UniverseAddress[];
    depth: number;
  }[];

  // Statistics
  statistics: {
    avgDivergence: number;
    stdDevDivergence: number;
    maxDivergence: number;
    minDivergence: number;
  };
}

/**
 * Query for finding specific universes
 */
export interface UniverseQuery {
  // Search criteria
  criteria: {
    eventOccurred?: string;     // Event that happened
    eventNotOccurred?: string;  // Event that didn't happen
    year?: number;              // Specific year
    dateRange?: {
      start: Date | string;
      end: Date | string;
    };
    properties?: {              // Physical properties
      [key: string]: number | string;
    };
  };

  // Constraints
  maxDivergence?: number;
  minStability?: number;
  maxResults?: number;          // Default: 100

  // Sorting
  sortBy?: 'divergence' | 'stability' | 'proximity' | 'probability';
  sortOrder?: 'asc' | 'desc';
}

/**
 * Result of universe query
 */
export interface UniverseQueryResult {
  matches: {
    universe: UniverseAddress;
    divergence: number;
    matchScore: number;         // 0-1 relevance score
    properties: Record<string, unknown>;
  }[];
  totalMatches: number;
  queryTime: number;            // milliseconds
}

// ============================================================================
// Cross-Dimensional Communication
// ============================================================================

/**
 * Protocol for inter-universe communication
 */
export type CommunicationProtocol =
  | 'quantum-entanglement'
  | 'wavefunction-modulation'
  | 'probability-amplitude-encoding'
  | 'timeline-inscription';

/**
 * Message for cross-universe transmission
 */
export interface MultiverseMessage {
  // Addressing
  from: UniverseAddress;
  to: UniverseAddress;
  messageId: string;

  // Content
  payload: {
    type: 'text' | 'binary' | 'quantum-state';
    data: string;               // base64 for binary
    encoding?: string;
  };

  // Transmission
  protocol: CommunicationProtocol;
  priority: 'low' | 'normal' | 'high' | 'urgent';
  timestamp: Date | string;

  // Security
  encrypted: boolean;
  signature?: string;
  publicKey?: string;

  // Delivery
  deliveryStatus?: 'pending' | 'sent' | 'delivered' | 'failed';
  deliveryTime?: Date | string;
  acknowledgment?: boolean;
}

/**
 * Communication channel between universes
 */
export interface CommunicationChannel {
  id: string;
  universe1: UniverseAddress;
  universe2: UniverseAddress;

  // Channel properties
  protocol: CommunicationProtocol;
  bandwidth: number;            // bits per second
  latency: number;              // milliseconds
  reliability: number;          // 0-1 scale
  encryption: boolean;

  // Status
  status: 'open' | 'closed' | 'degraded' | 'failed';
  established: Date | string;
  lastActivity: Date | string;
  messageCount: number;
}

// ============================================================================
// Identity Preservation
// ============================================================================

/**
 * Identity across multiple universes
 */
export interface MultiverseIdentity {
  // Core identity
  identityId: string;           // Unique across all universes
  originUniverse: UniverseAddress;

  // Instances across universes
  instances: {
    universe: UniverseAddress;
    state: 'alive' | 'deceased' | 'unknown' | 'never-existed';
    divergenceFromOrigin: number;
    lastVerified: Date | string;
    biometrics?: {
      dna?: string;             // Simplified representation
      consciousness?: string;   // Consciousness pattern
      memories?: string[];      // Key memory markers
    };
  }[];

  // Identity metrics
  coherence: number;            // How consistent is identity across universes
  continuity: number;           // Identity preservation quality
  variance: number;             // How much does identity vary

  // Verification
  verified: boolean;
  verificationMethod: string;
  lastVerification: Date | string;
}

/**
 * Configuration for identity verification
 */
export interface IdentityVerificationConfig {
  identityId: string;
  universes: UniverseAddress[];
  verificationMethod: 'biometric' | 'consciousness' | 'memory' | 'quantum-signature' | 'combined';
  threshold?: number;           // Minimum match threshold (default: 0.95)
}

/**
 * Result of identity verification
 */
export interface IdentityVerificationResult {
  verified: boolean;
  confidence: number;           // 0-1 scale
  matches: {
    universe: UniverseAddress;
    matchScore: number;
    differences: string[];
  }[];
  timestamp: Date | string;
}

// ============================================================================
// Measurement & Analysis
// ============================================================================

/**
 * Request for multiverse measurement
 */
export interface MeasurementRequest {
  // What to measure
  measurementType: 'divergence' | 'amplitude' | 'coherence' | 'stability' | 'entropy';
  targets: UniverseAddress[];

  // Parameters
  precision?: number;           // 0-1 scale (default: 0.9)
  repetitions?: number;         // Number of measurements (default: 100)
  integrationTime?: number;     // milliseconds (default: 1000)

  // Output
  includeRawData?: boolean;
  includeStatistics?: boolean;
  includeVisualization?: boolean;
}

/**
 * Result of multiverse measurement
 */
export interface MeasurementResult {
  measurementId: string;
  type: string;
  timestamp: Date | string;

  // Results
  values: {
    universe: UniverseAddress;
    value: number;
    uncertainty: number;
  }[];

  // Statistics
  statistics?: {
    mean: number;
    median: number;
    stdDev: number;
    min: number;
    max: number;
  };

  // Quality
  quality: {
    precision: number;
    accuracy: number;
    confidence: number;
    snr: number;                // Signal-to-noise ratio
  };

  // Raw data (optional)
  rawData?: number[][];
}

/**
 * Probability amplitude measurement
 */
export interface AmplitudeMeasurement {
  universe: UniverseAddress;
  amplitude: Complex;
  probability: number;          // |amplitude|²
  phase: number;                // arg(amplitude) in radians
  measurement: {
    method: string;
    timestamp: Date | string;
    precision: number;
    uncertainty: number;
  };
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Multiverse-related physical constants
 */
export const MULTIVERSE_CONSTANTS = {
  /** Planck constant (J·s) */
  PLANCK_CONSTANT: 6.62607015e-34,

  /** Reduced Planck constant (J·s) */
  REDUCED_PLANCK: 1.054571817e-34,

  /** Speed of light (m/s) */
  SPEED_OF_LIGHT: 299792458,

  /** Boltzmann constant (J/K) */
  BOLTZMANN: 1.380649e-23,

  /** Typical decoherence time (s) - varies by system */
  TYPICAL_DECOHERENCE_TIME: 1e-6,

  /** Universe branching rate (events per second per cubic meter) */
  BRANCHING_RATE: 1e50,

  /** Maximum theoretical divergence */
  MAX_DIVERGENCE: 1.0,

  /** Minimum detectable divergence */
  MIN_DIVERGENCE: 1e-10,
} as const;

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-QUA-017 error codes
 */
export enum MultiverseErrorCode {
  UNIVERSE_NOT_FOUND = 'MV001',
  INVALID_ADDRESS = 'MV002',
  DIVERGENCE_TOO_LARGE = 'MV003',
  ANCHOR_FAILED = 'MV004',
  COMMUNICATION_FAILED = 'MV005',
  IDENTITY_MISMATCH = 'MV006',
  TIMELINE_PARADOX = 'MV007',
  COHERENCE_LOSS = 'MV008',
  INSUFFICIENT_ENERGY = 'MV009',
  ACCESS_DENIED = 'MV010',
  MEASUREMENT_ERROR = 'MV011',
}

/**
 * Multiverse error
 */
export class MultiverseError extends Error {
  constructor(
    public code: MultiverseErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'MultiverseError';
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

/**
 * Partial deep type for nested objects
 */
export type DeepPartial<T> = {
  [P in keyof T]?: T[P] extends object ? DeepPartial<T[P]> : T[P];
};

// ============================================================================
// Export All Types
// ============================================================================

export type {
  // Core
  Spacetime4D,
  QuantumState,
  Complex,
  ProbabilityDistribution,

  // Universe addressing
  UniverseAddress,
  ValidatedUniverseAddress,

  // Reality anchors
  RealityAnchor,
  AnchorConfig,
  AnchorCreationResult,

  // Divergence & timeline
  DivergenceMetric,
  BranchingEvent,
  Timeline,
  DivergenceConfig,

  // Navigation
  NavigatorConfig,
  UniverseScanResult,
  UniverseQuery,
  UniverseQueryResult,

  // Communication
  CommunicationProtocol,
  MultiverseMessage,
  CommunicationChannel,

  // Identity
  MultiverseIdentity,
  IdentityVerificationConfig,
  IdentityVerificationResult,

  // Measurement
  MeasurementRequest,
  MeasurementResult,
  AmplitudeMeasurement,
};

export {
  MULTIVERSE_CONSTANTS,
  MultiverseErrorCode,
  MultiverseError,
};

/**
 * 弘益人間 (홍익인간) · Benefit All Humanity
 */
