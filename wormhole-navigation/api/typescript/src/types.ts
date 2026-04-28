/**
 * WIA-QUA-015: Wormhole Navigation - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Quantum Physics Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Spacetime Coordinates
// ============================================================================

/**
 * 4D spacetime coordinates (t, r, θ, φ)
 */
export interface SpacetimeCoordinates {
  /** Time coordinate (seconds) */
  t: number;

  /** Radial coordinate (meters) */
  r: number;

  /** Polar angle (radians, 0 to π) */
  theta: number;

  /** Azimuthal angle (radians, 0 to 2π) */
  phi: number;
}

/**
 * Position in proper distance coordinates (t, l, θ, φ)
 */
export interface ProperCoordinates {
  /** Time coordinate (seconds) */
  t: number;

  /** Proper radial distance from throat (meters) */
  l: number;

  /** Polar angle (radians) */
  theta: number;

  /** Azimuthal angle (radians) */
  phi: number;
}

/**
 * 3D velocity vector
 */
export interface Velocity {
  /** Radial velocity (m/s) */
  vr: number;

  /** Polar velocity (m/s) */
  vtheta: number;

  /** Azimuthal velocity (m/s) */
  vphi: number;
}

/**
 * 3D acceleration vector
 */
export interface Acceleration {
  /** Radial acceleration (m/s²) */
  ar: number;

  /** Polar acceleration (m/s²) */
  atheta: number;

  /** Azimuthal acceleration (m/s²) */
  aphi: number;
}

// ============================================================================
// Metric Tensor
// ============================================================================

/**
 * Metric tensor components
 */
export interface MetricTensor {
  /** g_tt component */
  g_tt: number;

  /** g_rr or g_ll component */
  g_rr: number;

  /** g_θθ component */
  g_thetatheta: number;

  /** g_φφ component */
  g_phiphi: number;

  /** Off-diagonal components (usually zero) */
  g_tr?: number;
  g_ttheta?: number;
  g_tphi?: number;
}

/**
 * Shape function type
 */
export type ShapeFunction = (l: number) => number;

/**
 * Redshift function type
 */
export type RedshiftFunction = (l: number) => number;

// ============================================================================
// Wormhole Types
// ============================================================================

/**
 * Wormhole classification
 */
export type WormholeType =
  | 'EinsteinRosen'      // Non-traversable Schwarzschild wormhole
  | 'MorrisThorne'       // Traversable wormhole with exotic matter
  | 'VisserThinShell'    // Thin-shell wormhole
  | 'RotatingWormhole'   // Rotating (Kerr-like) wormhole
  | 'ChargedWormhole';   // Charged (Reissner-Nordström) wormhole

/**
 * Wormhole metric specification
 */
export interface WormholeMetric {
  /** Type of wormhole */
  type: WormholeType;

  /** Throat radius (meters) */
  throatRadius: number;

  /** Shape function b(l) */
  shapeFunction: ShapeFunction;

  /** Redshift function Φ(l) */
  redshiftFunction?: RedshiftFunction;

  /** Total mass (kg) */
  mass: number;

  /** Exotic matter mass (negative, kg) */
  exoticMatter: number;

  /** Wormhole name/label */
  label?: string;

  /** Additional metadata */
  metadata?: Record<string, unknown>;
}

// ============================================================================
// Exotic Matter
// ============================================================================

/**
 * Exotic matter requirements
 */
export interface ExoticMatterRequirements {
  /** Total exotic matter mass (negative, kg) */
  mass: number;

  /** Energy density at throat (J/m³) */
  energyDensity: number;

  /** Pressure components */
  pressure: {
    radial: number;      // Radial pressure (Pa)
    tangential: number;  // Tangential pressure (Pa)
  };

  /** Null Energy Condition violation */
  nullEnergyViolation: number;

  /** Distribution function */
  distribution: (l: number) => number;
}

/**
 * Energy condition violations
 */
export interface EnergyConditions {
  /** Weak Energy Condition: T_μν u^μ u^ν ≥ 0 */
  weak: boolean;

  /** Null Energy Condition: ρ + p ≥ 0 */
  null: boolean;

  /** Strong Energy Condition */
  strong: boolean;

  /** Dominant Energy Condition */
  dominant: boolean;
}

// ============================================================================
// Stability Analysis
// ============================================================================

/**
 * Stability analysis result
 */
export interface StabilityResult {
  /** Is wormhole stable? */
  isStable: boolean;

  /** Throat radius (m) */
  throatRadius: number;

  /** Rate of throat radius change (m/s) */
  throatChangeRate: number;

  /** Damping time for perturbations (s) */
  dampingTime: number;

  /** Shape function derivative at throat */
  shapeFunctionDerivative: number;

  /** Flare-out condition satisfied? */
  flareOutCondition: boolean;

  /** Perturbation eigenvalues */
  eigenvalues?: number[];

  /** Stability margin (0-1) */
  stabilityMargin: number;
}

/**
 * Perturbation parameters
 */
export interface Perturbation {
  /** Amplitude of perturbation */
  amplitude: number;

  /** Frequency (rad/s) */
  frequency: number;

  /** Damping coefficient */
  damping: number;

  /** Growth rate (negative for damping) */
  growthRate: number;
}

// ============================================================================
// Navigation
// ============================================================================

/**
 * Navigation trajectory parameters
 */
export interface TrajectoryParameters {
  /** Wormhole to navigate */
  wormhole: WormholeMetric;

  /** Entry point coordinates */
  entryPoint: SpacetimeCoordinates;

  /** Initial velocity (m/s) */
  velocity: number;

  /** Spacecraft/object mass (kg) */
  mass: number;

  /** Desired exit coordinates (optional) */
  exitPoint?: SpacetimeCoordinates;

  /** Navigation mode */
  mode?: 'fastest' | 'safest' | 'minimal-tidal';
}

/**
 * Navigation trajectory result
 */
export interface NavigationTrajectory {
  /** Entry coordinates */
  entry: SpacetimeCoordinates;

  /** Exit coordinates */
  exit: SpacetimeCoordinates;

  /** Proper time through wormhole (s) */
  properTime: number;

  /** Coordinate time (s) */
  coordinateTime: number;

  /** Worldline points */
  worldline: SpacetimeCoordinates[];

  /** Velocity profile */
  velocityProfile: Velocity[];

  /** Maximum tidal acceleration (m/s²) */
  maxTidalAcceleration: number;

  /** Energy required (J) */
  energyRequired: number;

  /** Success status */
  success: boolean;

  /** Warnings */
  warnings: string[];
}

/**
 * Entry/exit protocol
 */
export interface NavigationProtocol {
  /** Approach distance (m) */
  approachDistance: number;

  /** Recommended velocity (m/s) */
  recommendedVelocity: number;

  /** Alignment tolerance (radians) */
  alignmentTolerance: number;

  /** Monitoring frequency (Hz) */
  monitoringFrequency: number;

  /** Abort conditions */
  abortConditions: AbortConditions;
}

/**
 * Abort conditions
 */
export interface AbortConditions {
  /** Maximum tidal acceleration (m/s²) */
  maxTidalAcceleration: number;

  /** Maximum radiation dose (mSv/h) */
  maxRadiation: number;

  /** Maximum throat instability (m/s) */
  maxThroatInstability: number;

  /** Minimum stability margin */
  minStabilityMargin: number;
}

// ============================================================================
// Tidal Forces
// ============================================================================

/**
 * Tidal force analysis
 */
export interface TidalForces {
  /** Maximum acceleration (m/s²) */
  maxAcceleration: number;

  /** Acceleration gradient (m/s² per meter) */
  gradient: number;

  /** Radial tidal force (N) */
  radialForce: number;

  /** Tangential tidal force (N) */
  tangentialForce: number;

  /** Is safe for humans? */
  humanSafe: boolean;

  /** Is safe for spacecraft? */
  spacecraftSafe: boolean;

  /** Safety margin (0-1) */
  safetyMargin: number;
}

/**
 * Tidal force parameters
 */
export interface TidalParameters {
  /** Position in wormhole */
  position: ProperCoordinates;

  /** Object height/length (m) */
  objectSize: number;

  /** Object mass (kg) */
  objectMass: number;

  /** Maximum tolerable acceleration (m/s²) */
  maxTolerance: number;
}

// ============================================================================
// Safety
// ============================================================================

/**
 * Safety assessment result
 */
export interface SafetyAssessment {
  /** Overall safety status */
  safe: boolean;

  /** Individual checks */
  checks: {
    tidalForces: boolean;
    radiation: boolean;
    stability: boolean;
    structuralIntegrity: boolean;
    temperature: boolean;
  };

  /** Safety score (0-100) */
  safetyScore: number;

  /** Recommendations */
  recommendations: string[];

  /** Warnings */
  warnings: string[];

  /** Critical issues */
  criticalIssues: string[];
}

/**
 * Radiation analysis
 */
export interface RadiationAnalysis {
  /** Hawking radiation temperature (K) */
  hawkingTemperature: number;

  /** Radiation power (W) */
  radiationPower: number;

  /** Particle flux (particles/cm²·s) */
  particleFlux: number;

  /** Dose rate (mSv/h) */
  doseRate: number;

  /** Required shielding (cm lead equivalent) */
  requiredShielding: number;

  /** Safe exposure time (hours) */
  safeExposureTime: number;
}

// ============================================================================
// Coordinate Transformations
// ============================================================================

/**
 * Coordinate transformation result
 */
export interface CoordinateTransformation {
  /** Original coordinates */
  from: SpacetimeCoordinates | ProperCoordinates;

  /** Transformed coordinates */
  to: SpacetimeCoordinates | ProperCoordinates;

  /** Jacobian matrix */
  jacobian?: number[][];

  /** Transformation type */
  transformationType: 'standard-to-proper' | 'proper-to-standard' | 'entry-to-exit';
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Wormhole physics constants
 */
export const WORMHOLE_CONSTANTS = {
  /** Speed of light (m/s) */
  c: 2.99792458e8,

  /** Gravitational constant (m³/kg·s²) */
  G: 6.67430e-11,

  /** Reduced Planck constant (J·s) */
  hbar: 1.054571817e-34,

  /** Boltzmann constant (J/K) */
  k_B: 1.380649e-23,

  /** Planck length (m) */
  l_Planck: 1.616255e-35,

  /** Planck time (s) */
  t_Planck: 5.391247e-44,

  /** Planck mass (kg) */
  M_Planck: 2.176434e-8,

  /** Solar mass (kg) */
  M_Sun: 1.98892e30,

  /** Earth mass (kg) */
  M_Earth: 5.97219e24,

  /** Astronomical unit (m) */
  AU: 1.495978707e11,

  /** Light year (m) */
  ly: 9.4607304725808e15,

  /** Earth surface gravity (m/s²) */
  g_Earth: 9.80665,

  /** Schwarzschild radius of Sun (m) */
  r_s_Sun: 2954,
} as const;

/**
 * Safety limits
 */
export const SAFETY_LIMITS = {
  /** Maximum acceleration (m/s²) */
  MAX_ACCELERATION: 98.0665, // 10g

  /** Maximum gradient (m/s² per meter) */
  MAX_GRADIENT: 9.80665, // 1g/m

  /** Maximum duration (seconds) */
  MAX_DURATION: 60,

  /** Maximum radiation (mSv/h) */
  MAX_RADIATION: 1.0,

  /** Temperature range (K) */
  TEMP_MIN: 250,
  TEMP_MAX: 350,

  /** Maximum structural stress (Pa) */
  MAX_STRESS: 500e6, // 500 MPa

  /** Maximum strain rate (s⁻¹) */
  MAX_STRAIN_RATE: 1e-3,
} as const;

/**
 * Default wormhole parameters
 */
export const DEFAULT_WORMHOLE = {
  /** Minimum throat radius for human traversal (m) */
  MIN_THROAT_RADIUS: 1,

  /** Comfortable throat radius (m) */
  COMFORTABLE_THROAT_RADIUS: 10,

  /** Spacecraft throat radius (m) */
  SPACECRAFT_THROAT_RADIUS: 1000,

  /** Typical exotic matter ratio */
  EXOTIC_MATTER_RATIO: -1.5e26, // kg per meter throat radius
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
 * 3D vector
 */
export interface Vector3D {
  x: number;
  y: number;
  z: number;
}

/**
 * 4D vector (spacetime)
 */
export interface Vector4D {
  t: number;
  x: number;
  y: number;
  z: number;
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-QUA-015 error codes
 */
export enum WormholeErrorCode {
  INVALID_THROAT_RADIUS = 'W001',
  INSUFFICIENT_EXOTIC_MATTER = 'W002',
  UNSTABLE_WORMHOLE = 'W003',
  EXCESSIVE_TIDAL_FORCES = 'W004',
  UNSAFE_TRAJECTORY = 'W005',
  COORDINATE_TRANSFORMATION_FAILED = 'W006',
  SHAPE_FUNCTION_INVALID = 'W007',
  ENERGY_CONDITION_VIOLATED = 'W008',
  NAVIGATION_FAILED = 'W009',
  RADIATION_HAZARD = 'W010',
}

/**
 * Wormhole navigation error
 */
export class WormholeError extends Error {
  constructor(
    public code: WormholeErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'WormholeError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  // Coordinates
  SpacetimeCoordinates,
  ProperCoordinates,
  Velocity,
  Acceleration,

  // Metric
  MetricTensor,
  ShapeFunction,
  RedshiftFunction,

  // Wormhole
  WormholeType,
  WormholeMetric,

  // Exotic matter
  ExoticMatterRequirements,
  EnergyConditions,

  // Stability
  StabilityResult,
  Perturbation,

  // Navigation
  TrajectoryParameters,
  NavigationTrajectory,
  NavigationProtocol,
  AbortConditions,

  // Tidal forces
  TidalForces,
  TidalParameters,

  // Safety
  SafetyAssessment,
  RadiationAnalysis,

  // Transformations
  CoordinateTransformation,

  // Utility
  Vector3D,
  Vector4D,
};

export {
  WORMHOLE_CONSTANTS,
  SAFETY_LIMITS,
  DEFAULT_WORMHOLE,
  WormholeErrorCode,
  WormholeError,
};
