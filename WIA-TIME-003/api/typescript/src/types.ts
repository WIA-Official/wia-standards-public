/**
 * WIA-TIME-003: Quantum Time Theory - Type Definitions
 *
 * 弘益人間 (Hongik Ingan) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 * @copyright 2025 SmileStory Inc. / WIA
 */

// ============================================================================
// CORE TYPES
// ============================================================================

/**
 * Complex number representation
 */
export interface Complex {
  /** Real component */
  real: number;
  /** Imaginary component */
  imaginary: number;
}

/**
 * Position in 4D spacetime
 */
export interface SpacetimePosition {
  /** X coordinate (meters) */
  x: number;
  /** Y coordinate (meters) */
  y: number;
  /** Z coordinate (meters) */
  z: number;
  /** Time coordinate (seconds) */
  t: number;
}

// ============================================================================
// QUANTUM TIME STATE
// ============================================================================

/**
 * Quantum superposition of time states
 */
export interface QuantumTimeState {
  /** Unique identifier for this quantum state */
  stateId: string;

  /** Reference timestamp (central time) in milliseconds since epoch */
  baseTimestamp: number;

  /** Array of timeline superpositions */
  timelines: TemporalSuperposition[];

  /** Coherence time (how long superposition persists) in milliseconds */
  coherenceTime: number;

  /** Creation timestamp in milliseconds since epoch */
  createdAt: number;

  /** Total wavefunction normalization (should equal 1.0) */
  normalization: number;

  /** Global phase in radians */
  globalPhase: number;
}

/**
 * Individual timeline in quantum superposition
 */
export interface TemporalSuperposition {
  /** Timeline identifier */
  timelineId: string;

  /** Timestamp for this branch in milliseconds since epoch */
  timestamp: number;

  /** Complex probability amplitude */
  amplitude: Complex;

  /** Probability (|amplitude|²) */
  probability: number;

  /** Phase relative to base in radians */
  relativePhase: number;

  /** Divergence point from base timeline (optional) */
  divergencePoint?: number;

  /** Timeline metadata (optional) */
  metadata?: Record<string, any>;
}

/**
 * Configuration for creating quantum time state
 */
export interface QuantumTimeStateConfig {
  /** Base timestamp in milliseconds */
  baseTimestamp: number;

  /** Number of superposition states to create */
  superpositionCount: number;

  /** Coherence time in milliseconds */
  coherenceTime: number;

  /** Probability distribution type */
  distribution?: 'uniform' | 'gaussian' | 'exponential';

  /** Initial phase offset in radians */
  initialPhase?: number;

  /** Time range for superposition states */
  timeRange?: {
    start: number;
    end: number;
  };
}

// ============================================================================
// TIMELINE WAVEFUNCTION
// ============================================================================

/**
 * Mathematical description of timeline wavefunction
 */
export interface TimelineWavefunction {
  /** Wavefunction ID */
  wavefunctionId: string;

  /** Associated quantum state */
  quantumState: QuantumTimeState;

  /** Wavefunction in position representation */
  psi: (t: number) => Complex;

  /** Wavefunction in momentum representation */
  psiMomentum?: (p: number) => Complex;

  /** Normalization constant */
  normalizationConstant: number;

  /** Energy eigenvalue (if eigenstate) */
  energyEigenvalue?: number;
}

// ============================================================================
// MANY-WORLDS INTERPRETATION
// ============================================================================

/**
 * Timeline branch in many-worlds interpretation
 */
export interface TimelineBranch {
  /** Branch identifier */
  branchId: string;

  /** Parent timeline (null for root) */
  parentBranchId: string | null;

  /** Branching point timestamp */
  branchPoint: number;

  /** Branching event that caused split */
  branchEvent: BranchingEvent;

  /** Child branches */
  childBranchIds: string[];

  /** Branch probability weight */
  weight: number;

  /** Branch state */
  state: 'active' | 'dormant' | 'collapsed';

  /** Branch metadata */
  metadata?: Record<string, any>;
}

/**
 * Event that causes timeline branching
 */
export interface BranchingEvent {
  /** Event type */
  type: 'quantum_measurement' | 'decision_point' | 'interaction' | 'spontaneous';

  /** Event description */
  description: string;

  /** Number of outcomes/branches */
  outcomeCount: number;

  /** Timestamp of event */
  timestamp: number;

  /** Additional event data */
  data?: Record<string, any>;
}

/**
 * Collapsed timeline after wavefunction collapse
 */
export interface CollapsedTimeline {
  /** Selected timeline after collapse */
  selectedTimeline: TemporalSuperposition;

  /** Time of collapse */
  collapseTime: number;

  /** Observer reference frame */
  observerFrame: ReferenceFrame;

  /** Other branches that were not selected */
  otherBranches: TemporalSuperposition[];

  /** Collapse mechanism */
  collapseMechanism: 'measurement' | 'decoherence' | 'spontaneous';
}

// ============================================================================
// QUANTUM ENTANGLEMENT
// ============================================================================

/**
 * Quantum entanglement across time
 */
export interface QuantumEntanglement {
  /** Entanglement identifier */
  entanglementId: string;

  /** First temporal state */
  state1: QuantumTimeState;

  /** Second temporal state (can be partial) */
  state2: QuantumTimeState | { timestamp: number };

  /** Entanglement strength (0 to 1) */
  entanglementStrength: number;

  /** Bell state type */
  bellState: 'phi_plus' | 'phi_minus' | 'psi_plus' | 'psi_minus';

  /** Time separation in milliseconds */
  timeSeparation: number;

  /** Correlation function */
  correlationFunction: (t1: number, t2: number) => number;

  /** Created timestamp */
  createdAt: number;

  /** Entanglement metadata */
  metadata?: Record<string, any>;
}

/**
 * Bell inequality test result
 */
export interface BellInequalityTest {
  /** Test identifier */
  testId: string;

  /** CHSH value (should be > 2 for quantum) */
  chshValue: number;

  /** Violation detected */
  violationDetected: boolean;

  /** Classical bound (typically 2) */
  classicalBound: number;

  /** Quantum bound (typically 2√2) */
  quantumBound: number;

  /** Measurement results */
  measurements: Array<{
    angle1: number;
    angle2: number;
    correlation: number;
  }>;
}

// ============================================================================
// TEMPORAL QUANTUM TUNNELING
// ============================================================================

/**
 * Configuration for temporal quantum tunneling
 */
export interface TemporalTunnelConfig {
  /** Particle type */
  particle: 'electron' | 'proton' | 'photon' | 'neutron' | 'custom';

  /** Particle mass in kg (required for custom) */
  mass?: number;

  /** Barrier height in eV */
  barrierHeight: number;

  /** Barrier width in meters */
  barrierWidth: number;

  /** Particle energy in eV */
  particleEnergy: number;

  /** Temperature in Kelvin */
  temperature?: number;

  /** Barrier shape */
  barrierShape?: 'rectangular' | 'triangular' | 'parabolic' | 'delta';
}

/**
 * Result of quantum tunneling calculation
 */
export interface TunnelingResult {
  /** Transmission probability (0 to 1) */
  transmissionProbability: number;

  /** Reflection probability (0 to 1) */
  reflectionProbability: number;

  /** Tunneling time in femtoseconds */
  tunnelingTime: number;

  /** Wavefunction decay inside barrier */
  wavefunctionDecay: number;

  /** Whether tunneling succeeded (probabilistic) */
  success: boolean;

  /** Wave vector inside barrier */
  barrierWaveVector?: number;

  /** Additional calculation details */
  details?: {
    wkbApproximation: boolean;
    energyRatio: number;
    penetrationDepth: number;
  };
}

// ============================================================================
// WHEELER-DEWITT EQUATION
// ============================================================================

/**
 * Wheeler-DeWitt quantum gravity state
 */
export interface WheelerDeWittState {
  /** Wavefunction in superspace */
  wavefunctionInSuperspace: (metric: Metric3D) => Complex;

  /** Spatial metric configuration */
  spatialMetric: Metric3D;

  /** Ricci scalar */
  ricciScalar: number;

  /** Cosmological constant */
  cosmologicalConstant: number;

  /** Energy constraint (should be ~0) */
  energyConstraint: number;

  /** Emergent time parameter */
  emergentTime?: number;
}

/**
 * 3D spatial metric tensor
 */
export interface Metric3D {
  /** Metric components (3x3 symmetric) */
  g11: number; g12: number; g13: number;
  g21: number; g22: number; g23: number;
  g31: number; g32: number; g33: number;
}

// ============================================================================
// QUANTUM DECOHERENCE
// ============================================================================

/**
 * Decoherence rate calculation result
 */
export interface DecoherenceRate {
  /** Decoherence rate (1/τ_D) in Hz */
  rate: number;

  /** Decoherence time τ_D in seconds */
  decoherenceTime: number;

  /** Contributing factors */
  factors: {
    /** Temperature contribution in Hz */
    thermal: number;
    /** Environmental noise contribution in Hz */
    environmental: number;
    /** Gravity-induced decoherence in Hz */
    gravitational: number;
    /** Information leakage in Hz */
    informational: number;
  };

  /** Coherence remaining as function of time */
  coherenceFunction: (t: number) => number;

  /** Threshold for complete decoherence */
  decoherenceThreshold: number;
}

/**
 * Environment configuration for decoherence
 */
export interface EnvironmentConfig {
  /** Temperature in Kelvin */
  temperature: number;

  /** Environmental noise level (0 to 1) */
  environmentalNoise: number;

  /** Gravitational field strength in m/s² */
  gravitationalField?: number;

  /** Photon number density */
  photonDensity?: number;

  /** Interaction strength with environment */
  couplingStrength?: number;
}

// ============================================================================
// OBSERVER EFFECT
// ============================================================================

/**
 * Observer in quantum mechanics
 */
export interface Observer {
  /** Observer identifier */
  observerId?: string;

  /** Observer position in spacetime */
  position: SpacetimePosition;

  /** Observer reference frame */
  referenceFrame: ReferenceFrame;

  /** Measurement apparatus */
  apparatus?: MeasurementDevice;

  /** Observer consciousness level (philosophical) */
  consciousnessLevel?: number;

  /** Observer metadata */
  metadata?: Record<string, any>;
}

/**
 * Relativistic reference frame
 */
export interface ReferenceFrame {
  /** Velocity relative to lab frame in m/s */
  velocity: {
    vx: number;
    vy: number;
    vz: number;
  };

  /** Proper time in seconds */
  properTime: number;

  /** Lorentz factor γ */
  gamma: number;

  /** Frame identifier */
  frameId?: string;
}

/**
 * Measurement device/apparatus
 */
export interface MeasurementDevice {
  /** Device type */
  type: 'detector' | 'clock' | 'interferometer' | 'spectrometer' | 'custom';

  /** Device name */
  name: string;

  /** Measurement precision */
  precision: number;

  /** Measurement basis */
  basis?: 'position' | 'momentum' | 'energy' | 'time';

  /** Device efficiency (0 to 1) */
  efficiency?: number;
}

// ============================================================================
// SCHRÖDINGER'S TIMELINE
// ============================================================================

/**
 * Schrödinger's timeline paradox
 */
export interface SchrodingersTimeline {
  /** Superposed timeline states */
  state1: TimelineState;
  state2: TimelineState;

  /** Probability amplitudes */
  amplitudes: {
    state1: Complex;
    state2: Complex;
  };

  /** Observation status */
  observed: boolean;

  /** Collapsed result (if observed) */
  collapsedState?: TimelineState;

  /** Paradox description */
  paradox: string;

  /** Superposition creation time */
  createdAt: number;
}

/**
 * Individual timeline state
 */
export interface TimelineState {
  /** State identifier */
  stateId: string;

  /** Description */
  description: string;

  /** Events in this state */
  events: HistoricalEvent[];

  /** State probability */
  probability: number;

  /** State metadata */
  metadata?: Record<string, any>;
}

/**
 * Historical event in timeline
 */
export interface HistoricalEvent {
  /** Event name */
  name: string;

  /** Event timestamp */
  timestamp: number;

  /** Event description */
  description?: string;

  /** Event significance (0 to 1) */
  significance?: number;

  /** Event data */
  data?: Record<string, any>;
}

// ============================================================================
// UTILITY TYPES
// ============================================================================

/**
 * Time range
 */
export interface TimeRange {
  /** Start time in milliseconds */
  start: number;

  /** End time in milliseconds */
  end: number;
}

/**
 * Physical constants
 */
export interface PhysicalConstants {
  /** Reduced Planck constant (J⋅s) */
  HBAR: number;

  /** Speed of light (m/s) */
  C: number;

  /** Boltzmann constant (J/K) */
  BOLTZMANN: number;

  /** Electron mass (kg) */
  ELECTRON_MASS: number;

  /** Proton mass (kg) */
  PROTON_MASS: number;

  /** Elementary charge (C) */
  ELEMENTARY_CHARGE: number;

  /** Electron volt to Joules conversion */
  EV_TO_JOULES: number;

  /** Planck time (s) */
  PLANCK_TIME: number;

  /** Planck length (m) */
  PLANCK_LENGTH: number;
}

/**
 * Error type for quantum time operations
 */
export class QuantumTimeError extends Error {
  constructor(
    message: string,
    public code: string,
    public details?: any
  ) {
    super(message);
    this.name = 'QuantumTimeError';
    Object.setPrototypeOf(this, QuantumTimeError.prototype);
  }
}

/**
 * Error codes
 */
export const ERROR_CODES = {
  NORMALIZATION_FAILED: 'QT-001',
  INVALID_PROBABILITY: 'QT-002',
  DECOHERENCE_OVERFLOW: 'QT-003',
  ENTANGLEMENT_BROKEN: 'QT-004',
  TUNNELING_IMPOSSIBLE: 'QT-005',
  CAUSALITY_VIOLATION: 'QT-006',
  INVALID_STATE: 'QT-007',
  MEASUREMENT_FAILED: 'QT-008',
  COLLAPSE_ERROR: 'QT-009',
  PHASE_INCONSISTENCY: 'QT-010'
} as const;

// ============================================================================
// VALIDATION TYPES
// ============================================================================

/**
 * Validation result
 */
export interface ValidationResult {
  /** Is valid */
  valid: boolean;

  /** Error messages */
  errors: string[];

  /** Warning messages */
  warnings: string[];
}

/**
 * Quantum state validator
 */
export interface StateValidator {
  /** Validate normalization */
  validateNormalization: (state: QuantumTimeState) => boolean;

  /** Validate probabilities */
  validateProbabilities: (state: QuantumTimeState) => boolean;

  /** Validate phases */
  validatePhases: (state: QuantumTimeState) => boolean;

  /** Full validation */
  validate: (state: QuantumTimeState) => ValidationResult;
}

// ============================================================================
// EXPORT ALL
// ============================================================================

export type {
  // Core
  Complex,
  SpacetimePosition,

  // Quantum Time State
  QuantumTimeState,
  TemporalSuperposition,
  QuantumTimeStateConfig,
  TimelineWavefunction,

  // Many-Worlds
  TimelineBranch,
  BranchingEvent,
  CollapsedTimeline,

  // Entanglement
  QuantumEntanglement,
  BellInequalityTest,

  // Tunneling
  TemporalTunnelConfig,
  TunnelingResult,

  // Wheeler-DeWitt
  WheelerDeWittState,
  Metric3D,

  // Decoherence
  DecoherenceRate,
  EnvironmentConfig,

  // Observer
  Observer,
  ReferenceFrame,
  MeasurementDevice,

  // Schrödinger's Timeline
  SchrodingersTimeline,
  TimelineState,
  HistoricalEvent,

  // Utility
  TimeRange,
  PhysicalConstants,
  ValidationResult,
  StateValidator
};
