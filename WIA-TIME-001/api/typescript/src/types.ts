/**
 * WIA-TIME-001: Time Travel Physics - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Time Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Physics Types
// ============================================================================

/**
 * Three-dimensional vector for spatial coordinates
 */
export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

/**
 * Four-dimensional spacetime coordinates
 */
export interface SpacetimePoint {
  /** Time coordinate (ISO 8601 format) */
  time: Date | string;

  /** Spatial position in meters */
  position: Vector3;

  /** Velocity vector (fraction of c) */
  velocity?: Vector3;

  /** Reference frame identifier */
  referenceFrame?: string;
}

/**
 * Temporal coordinate with enhanced metadata
 */
export interface TemporalCoordinate extends SpacetimePoint {
  /** Proper time (tau) in seconds */
  properTime?: number;

  /** Coordinate time in seconds */
  coordinateTime?: number;

  /** Lorentz factor (gamma) */
  lorentzFactor?: number;

  /** Worldline identifier */
  worldlineId?: string;
}

/**
 * Time vector connecting two points in spacetime
 */
export interface TimeVector {
  /** Origin point in spacetime */
  origin: TemporalCoordinate;

  /** Destination point in spacetime */
  destination: TemporalCoordinate;

  /** Temporal displacement in seconds (negative for past) */
  displacement: number;

  /** Spatial displacement magnitude in meters */
  spatialDistance?: number;

  /** Travel method */
  path?: 'wormhole' | 'ctc' | 'alcubierre' | 'field' | 'natural';
}

// ============================================================================
// Energy Calculations
// ============================================================================

/**
 * Parameters for time travel energy calculation
 */
export interface TimeTravelParameters {
  /** Mass to be displaced in kilograms */
  mass: number;

  /** Temporal displacement in seconds (negative for past travel) */
  displacement: number;

  /** Velocity as fraction of speed of light (0-1) */
  velocity?: number;

  /** Efficiency factor (1.0 = ideal, higher = less efficient) */
  efficiencyFactor?: number;

  /** Include field generation energy */
  includeFieldEnergy?: boolean;
}

/**
 * Energy requirement calculation result
 */
export interface EnergyRequirements {
  /** Total energy required in joules */
  energy: number;

  /** Rest mass energy (E₀ = mc²) */
  restMassEnergy: number;

  /** Temporal displacement energy */
  temporalEnergy: number;

  /** Relativistic correction energy */
  relativisticEnergy: number;

  /** Temporal displacement factor (τ) */
  temporalFactor: number;

  /** Lorentz factor (γ) */
  lorentzFactor: number;

  /** Field generation energy (if applicable) */
  fieldEnergy?: number;

  /** Feasibility assessment */
  feasibility: 'possible' | 'difficult' | 'impossible' | 'theoretical';

  /** Energy in human-readable format */
  energyFormatted: string;

  /** Comparison to known energy sources */
  comparison?: {
    source: string;
    ratio: number;
    description: string;
  }[];
}

// ============================================================================
// Temporal Field
// ============================================================================

/**
 * Temporal field configuration
 */
export interface TemporalField {
  /** Unique field identifier */
  id: string;

  /** Field center point */
  center: Vector3;

  /** Field radius in meters */
  radius: number;

  /** Total field energy in joules */
  energy: number;

  /** Temporal displacement in seconds */
  displacement: number;

  /** Field strength in N/kg */
  strength: number;

  /** Stability factor (0-1) */
  stability: number;

  /** Field status */
  status: 'initializing' | 'active' | 'unstable' | 'failing' | 'shutdown';

  /** Creation timestamp */
  created: Date;

  /** Last update timestamp */
  updated: Date;

  /** Field layers */
  layers?: {
    innerCore: FieldLayer;
    transitionLayer: FieldLayer;
    outerShell: FieldLayer;
  };
}

/**
 * Field layer configuration
 */
export interface FieldLayer {
  /** Layer radius or thickness in meters */
  radius: number;

  /** Energy density in J/m³ */
  energyDensity: number;

  /** Field strength in N/kg */
  strength: number;

  /** Layer status */
  status: 'stable' | 'fluctuating' | 'collapsing';
}

/**
 * Field generation parameters
 */
export interface FieldGenerationParams {
  /** Center point for field */
  center: Vector3;

  /** Desired radius in meters */
  radius: number;

  /** Target temporal displacement in seconds */
  displacement: number;

  /** Available energy in joules */
  availableEnergy: number;

  /** Minimum required stability (0-1) */
  minStability?: number;

  /** Ramp-up time in seconds */
  rampUpTime?: number;

  /** Ramp-down time in seconds */
  rampDownTime?: number;
}

// ============================================================================
// Wormhole Physics
// ============================================================================

/**
 * Wormhole configuration
 */
export interface WormholeConfig {
  /** Wormhole identifier */
  id: string;

  /** Throat radius in meters */
  throatRadius: number;

  /** Mouth A location */
  mouthA: TemporalCoordinate;

  /** Mouth B location */
  mouthB: TemporalCoordinate;

  /** Temporal offset between mouths in seconds */
  temporalOffset: number;

  /** Exotic matter requirement in kg */
  exoticMatterRequired: number;

  /** Traversal time in seconds */
  traversalTime: number;

  /** Stability metric (0-1) */
  stability: number;

  /** Wormhole type */
  type: 'morris-thorne' | 'schwarzschild' | 'kerr' | 'ellis' | 'custom';

  /** Metric tensor components */
  metric?: number[][];
}

/**
 * Wormhole traversal parameters
 */
export interface WormholeTraversal {
  /** Wormhole to traverse */
  wormhole: WormholeConfig;

  /** Entry velocity (fraction of c) */
  entryVelocity: number;

  /** Mass being transported in kg */
  mass: number;

  /** Desired exit time */
  exitTime?: Date;

  /** Safety margin (0-1) */
  safetyMargin?: number;
}

// ============================================================================
// Closed Timelike Curves
// ============================================================================

/**
 * Closed Timelike Curve (CTC) configuration
 */
export interface ClosedTimelikeCurve {
  /** CTC identifier */
  id: string;

  /** Worldline parametrization */
  worldline: (tau: number) => SpacetimePoint;

  /** Curve period in proper time */
  period: number;

  /** Entry point */
  entry: TemporalCoordinate;

  /** Exit point (same as entry for closed curve) */
  exit: TemporalCoordinate;

  /** Is the curve timelike? */
  isTimelike: boolean;

  /** Spacetime interval (should be < 0 for timelike) */
  interval: number;

  /** Causality status */
  causalityStatus: 'consistent' | 'paradoxical' | 'unknown';

  /** CTC type */
  type: 'godel' | 'tipler' | 'kerr' | 'van-stockum' | 'custom';
}

/**
 * CTC detection parameters
 */
export interface CTCDetectionParams {
  /** Spacetime region to search */
  region: {
    timeRange: [Date, Date];
    spatialBounds: {
      min: Vector3;
      max: Vector3;
    };
  };

  /** Metric tensor function */
  metric: (point: SpacetimePoint) => number[][];

  /** Detection tolerance */
  tolerance?: number;

  /** Maximum search iterations */
  maxIterations?: number;
}

// ============================================================================
// Temporal Displacement
// ============================================================================

/**
 * Temporal displacement operation
 */
export interface TemporalDisplacement {
  /** Operation identifier */
  id: string;

  /** Current position in spacetime */
  current: TemporalCoordinate;

  /** Target position in spacetime */
  target: TemporalCoordinate;

  /** Displacement method */
  method: 'wormhole' | 'ctc' | 'field' | 'alcubierre' | 'natural';

  /** Required energy in joules */
  energyRequired: number;

  /** Available energy in joules */
  energyAvailable?: number;

  /** Estimated duration in seconds */
  duration: number;

  /** Risk assessment */
  risk: 'low' | 'medium' | 'high' | 'extreme';

  /** Novikov consistency check */
  novikov: NovikovConsistency;

  /** Operation status */
  status: 'pending' | 'in-progress' | 'completed' | 'failed' | 'aborted';

  /** Timeline before operation */
  timelineBefore?: Timeline;

  /** Timeline after operation */
  timelineAfter?: Timeline;
}

/**
 * Timeline representation
 */
export interface Timeline {
  /** Timeline identifier */
  id: string;

  /** Timeline name/description */
  name?: string;

  /** Key events in timeline */
  events: TimelineEvent[];

  /** Timeline branch point */
  branchPoint?: Date;

  /** Parent timeline (if branched) */
  parentTimeline?: string;

  /** Probability of timeline (for MWI) */
  probability?: number;

  /** Timeline integrity score (0-1) */
  integrity: number;
}

/**
 * Event in a timeline
 */
export interface TimelineEvent {
  /** Event identifier */
  id: string;

  /** Event time */
  time: Date;

  /** Event description */
  description: string;

  /** Affected entities */
  affected?: string[];

  /** Causality chain */
  causes?: string[];

  /** Effects */
  effects?: string[];

  /** Can this event be changed? */
  mutable: boolean;
}

// ============================================================================
// Novikov Self-Consistency
// ============================================================================

/**
 * Novikov self-consistency check result
 */
export interface NovikovConsistency {
  /** Is the action consistent with timeline? */
  isConsistent: boolean;

  /** Probability of action success (0-1) */
  probability: number;

  /** Detected contradictions */
  contradictions: Contradiction[];

  /** Consistency violations */
  violations: string[];

  /** Warnings (non-blocking) */
  warnings: string[];

  /** Causality chains */
  causalityChains: CausalityChain[];

  /** Alternative consistent timelines */
  alternativeTimelines?: Timeline[];

  /** Recommendation */
  recommendation: 'proceed' | 'proceed-with-caution' | 'abort' | 'impossible';
}

/**
 * Paradox or contradiction
 */
export interface Contradiction {
  /** Contradiction type */
  type: 'grandfather' | 'bootstrap' | 'predestination' | 'ontological' | 'causal-loop' | 'other';

  /** Description */
  description: string;

  /** Severity */
  severity: 'info' | 'warning' | 'error' | 'critical';

  /** Affected events */
  events: string[];

  /** Possible resolutions */
  resolutions?: string[];
}

/**
 * Causality chain
 */
export interface CausalityChain {
  /** Chain identifier */
  id: string;

  /** Cause event */
  cause: TimelineEvent;

  /** Effect event */
  effect: TimelineEvent;

  /** Intermediate events */
  intermediates?: TimelineEvent[];

  /** Chain length (number of hops) */
  length: number;

  /** Time span in seconds */
  timeSpan: number;

  /** Is chain closed (forms loop)? */
  isClosed: boolean;
}

// ============================================================================
// Validation and Safety
// ============================================================================

/**
 * Jump validation parameters
 */
export interface JumpValidation {
  /** Target time */
  targetTime: Date;

  /** Current time */
  currentTime: Date;

  /** Available energy in joules */
  energyAvailable: number;

  /** Mass to transport in kg */
  mass: number;

  /** Maximum acceptable risk */
  maxRisk?: 'low' | 'medium' | 'high';

  /** Require Novikov consistency? */
  requireConsistency?: boolean;

  /** Field configuration */
  fieldConfig?: FieldGenerationParams;
}

/**
 * Jump validation result
 */
export interface ValidationResult {
  /** Is jump valid? */
  isValid: boolean;

  /** Errors (blocking) */
  errors: string[];

  /** Warnings (non-blocking) */
  warnings: string[];

  /** Required energy in joules */
  requiredEnergy: number;

  /** Energy deficit/surplus in joules */
  energyDelta: number;

  /** Novikov consistency check */
  novikov: NovikovConsistency;

  /** Risk assessment */
  risk: 'low' | 'medium' | 'high' | 'extreme';

  /** Safety checks */
  safetyChecks: SafetyCheck[];

  /** Recommended actions */
  recommendations: string[];
}

/**
 * Safety check result
 */
export interface SafetyCheck {
  /** Check name */
  name: string;

  /** Check status */
  status: 'pass' | 'fail' | 'warning' | 'info';

  /** Check description */
  description: string;

  /** Measured value */
  value?: number;

  /** Expected value */
  expected?: number;

  /** Threshold */
  threshold?: number;

  /** Corrective action if failed */
  correctiveAction?: string;
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Physical constants used in time travel calculations
 */
export const PHYSICS_CONSTANTS = {
  /** Speed of light in m/s */
  SPEED_OF_LIGHT: 299792458,

  /** Gravitational constant in m³/kg·s² */
  GRAVITATIONAL_CONSTANT: 6.67430e-11,

  /** Planck constant in J·s */
  PLANCK_CONSTANT: 6.62607015e-34,

  /** Planck time in seconds */
  PLANCK_TIME: 5.391247e-44,

  /** Planck length in meters */
  PLANCK_LENGTH: 1.616255e-35,

  /** Temporal coupling constant in N·s²/kg² */
  TEMPORAL_COUPLING: 6.67e-11,

  /** Minimum field stability threshold */
  MIN_STABILITY: 0.95,

  /** Causality threshold */
  CAUSALITY_THRESHOLD: 0.9999,

  /** Maximum recommended energy in joules */
  MAX_ENERGY: 1e25,

  /** Recommended displacement bounds in seconds */
  MAX_DISPLACEMENT: 100 * 365.25 * 24 * 3600, // ±100 years
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

/**
 * Time travel simulation result
 */
export interface SimulationResult {
  /** Simulation identifier */
  id: string;

  /** Input parameters */
  parameters: TimeTravelParameters;

  /** Temporal displacement operation */
  displacement: TemporalDisplacement;

  /** Energy requirements */
  energy: EnergyRequirements;

  /** Validation result */
  validation: ValidationResult;

  /** Simulation timeline */
  timeline: Timeline;

  /** Simulation duration in milliseconds */
  duration: number;

  /** Success status */
  success: boolean;

  /** Error message if failed */
  error?: string;
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-TIME-001 error codes
 */
export enum TimeErrorCode {
  INSUFFICIENT_ENERGY = 'T001',
  PARADOX_DETECTED = 'T002',
  FIELD_UNSTABLE = 'T003',
  DISPLACEMENT_TOO_LARGE = 'T004',
  CAUSALITY_VIOLATION = 'T005',
  EQUIPMENT_FAILURE = 'T006',
  INVALID_PARAMETERS = 'T007',
  TIMELINE_DIVERGENCE = 'T008',
  WORMHOLE_COLLAPSE = 'T009',
  CTC_NOT_FOUND = 'T010',
}

/**
 * Time travel error
 */
export class TimeTravelError extends Error {
  constructor(
    public code: TimeErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'TimeTravelError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  // Core types
  SpacetimePoint,
  TemporalCoordinate,
  TimeVector,

  // Energy
  TimeTravelParameters,
  EnergyRequirements,

  // Fields
  TemporalField,
  FieldLayer,
  FieldGenerationParams,

  // Wormholes
  WormholeConfig,
  WormholeTraversal,

  // CTCs
  ClosedTimelikeCurve,
  CTCDetectionParams,

  // Displacement
  TemporalDisplacement,
  Timeline,
  TimelineEvent,

  // Novikov
  NovikovConsistency,
  Contradiction,
  CausalityChain,

  // Validation
  JumpValidation,
  ValidationResult,
  SafetyCheck,

  // Simulation
  SimulationResult,
};

export { PHYSICS_CONSTANTS, TimeErrorCode, TimeTravelError };
