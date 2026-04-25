/**
 * WIA-TIME-002: Spacetime Manipulation Standard
 * TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @description Complete type definitions for spacetime manipulation operations
 *
 * 弘益人間 · Benefit All Humanity
 * WIA - World Certification Industry Association
 */

// ============================================================================
// Basic Spacetime Structures
// ============================================================================

/**
 * 4D Spacetime Point (t, x, y, z)
 */
export interface SpacetimePoint {
  /** Time coordinate (seconds) */
  t: number;
  /** X position (meters) */
  x: number;
  /** Y position (meters) */
  y: number;
  /** Z position (meters) */
  z: number;
}

/**
 * 3D Vector
 */
export interface Vector3D {
  x: number;
  y: number;
  z: number;
}

/**
 * Spacetime Region
 */
export interface SpacetimeRegion {
  origin: SpacetimePoint;
  dimensions: {
    /** Time span (seconds) */
    temporal: number;
    spatial: {
      /** X extent (meters) */
      width: number;
      /** Y extent (meters) */
      height: number;
      /** Z extent (meters) */
      depth: number;
    };
  };
}

// ============================================================================
// Metric Tensor and Curvature
// ============================================================================

/**
 * Metric Tensor g_μν
 * Describes spacetime geometry
 */
export interface MetricTensor {
  // Diagonal components
  /** Time-time component (typically -c²) */
  g00: number;
  /** Space-x component (typically +1) */
  g11: number;
  /** Space-y component (typically +1) */
  g22: number;
  /** Space-z component (typically +1) */
  g33: number;

  // Off-diagonal components (cross terms)
  /** Time-x coupling */
  g01: number;
  /** Time-y coupling */
  g02: number;
  /** Time-z coupling */
  g03: number;
  /** x-y coupling */
  g12: number;
  /** x-z coupling */
  g13: number;
  /** y-z coupling */
  g23: number;

  // Metadata
  position: SpacetimePoint;
  coordinateSystem: 'cartesian' | 'spherical' | 'schwarzschild' | 'alcubierre';
  /** Signature, e.g., "(-,+,+,+)" */
  signature: string;
}

/**
 * Christoffel Symbols Γ^ρ_μν
 */
export interface ChristoffelSymbols {
  /** 4×4×4 array representing all components */
  components: number[][][];
}

/**
 * Riemann Curvature Tensor R^ρ_σμν
 */
export interface RiemannTensor {
  /** 4×4×4×4 array representing all components */
  components: number[][][][];
}

/**
 * Curvature Tensor with derived quantities
 */
export interface CurvatureTensor {
  /** Full Riemann tensor R^ρ_σμν */
  riemann: RiemannTensor;

  /** Ricci tensor R_μν (4×4) */
  ricciTensor: number[][];

  /** Ricci scalar R */
  ricciScalar: number;

  /** Einstein tensor G_μν (4×4) */
  einsteinTensor: number[][];

  // Physical interpretation
  /** Tidal force vector */
  tidalForces: Vector3D;
  /** Radius of curvature (meters) */
  curvatureRadius: number;
}

/**
 * Stress-Energy Tensor T_μν
 */
export interface StressEnergyTensor {
  /** 4×4 tensor components */
  components: number[][];

  /** Energy density T^00 (J/m³) */
  energyDensity: number;
  /** Momentum density (kg/(m²·s)) */
  momentumDensity: Vector3D;
  /** Pressure tensor (Pa) */
  pressureTensor: number[][];

  /** Type of matter/energy */
  matterType: 'normal' | 'exotic' | 'dark' | 'vacuum';
}

// ============================================================================
// Gravity Well
// ============================================================================

/**
 * Gravity Well Configuration
 */
export interface GravityWell {
  /** Center position */
  position: SpacetimePoint;

  /** Effective mass (kg) */
  mass: number;
  /** Spatial extent/radius (meters) */
  radius: number;
  /** Mass density (kg/m³) */
  density: number;

  // Schwarzschild properties
  /** Schwarzschild radius r_s = 2GM/c² (meters) */
  schwarzschildRadius: number;
  /** Surface gravity g = GM/r² (m/s²) */
  surfaceGravity: number;

  // Field properties
  /** Metric perturbation caused by this well */
  metricPerturbation: MetricTensor;
  /** Gravitational potential Φ(r) = -GM/r */
  potentialFunction: (r: number) => number;

  // Time dilation
  /** Time dilation factor as function of radius */
  timeDilationFactor: (r: number) => number;

  // Safety
  /** Maximum tidal force (N/m) */
  tidalForceLimit: number;
  /** Whether an event horizon is present */
  eventHorizonPresent: boolean;
}

/**
 * Tidal Force Configuration
 */
export interface TidalForce {
  /** Tidal gradient ∂²Φ/∂r² (s⁻²) */
  gradient: number;
  /** Maximum allowable stress (N/m) */
  maximumStress: number;
  /** Safe limit for biological matter (N/m) */
  biologicalLimit: number;
  /** Safe limit for structural materials (N/m) */
  structuralLimit: number;
}

// ============================================================================
// Space Folding and Wormholes
// ============================================================================

/**
 * Casimir Cavity for negative energy generation
 */
export interface CasimirCavity {
  /** Distance between plates (meters) */
  plateDistance: number;
  /** Plate area (m²) */
  area: number;
  /** Material of plates */
  plateMaterial: string;

  /** Energy density ρ = -π²ℏc/(240d⁴) (J/m³) */
  energyDensity: number;
  /** Casimir force F = -π²ℏcA/(240d⁴) (N) */
  force: number;
}

/**
 * Space Fold Configuration
 */
export interface SpaceFold {
  // Endpoints
  /** Starting point A */
  pointA: SpacetimePoint;
  /** Ending point B */
  pointB: SpacetimePoint;

  // Fold parameters
  /** Compression ratio (0 to 1) */
  compressionRatio: number;
  /** Effective distance after folding (meters) */
  effectiveDistance: number;
  /** Original Euclidean distance (meters) */
  originalDistance: number;

  // Wormhole topology
  /** Minimum throat radius (meters) */
  throatRadius: number;
  /** Shape function b(r) */
  shapeFunction: (r: number) => number;

  // Exotic matter requirements
  /** Negative energy density (J/m³) */
  exoticMatterDensity: number;
  /** Exotic matter distribution field */
  exoticMatterDistribution: MatterField;

  // Stability
  /** Stability index (0 to 1, higher is more stable) */
  stabilityIndex: number;
  /** Time until collapse (seconds), null if stable */
  collapseTime: number | null;
}

/**
 * Matter Field Distribution
 */
export interface MatterField {
  /** Scalar field φ(x) */
  phi: (position: SpacetimePoint) => number;

  /** Stress-energy tensor T_μν */
  T: (position: SpacetimePoint) => number[][];

  /** Energy density ρ(x) */
  energyDensity: (position: SpacetimePoint) => number;
  /** Momentum density (kg/(m²·s)) */
  momentumDensity: (position: SpacetimePoint) => Vector3D;

  /** Pressure tensor */
  pressure: (position: SpacetimePoint) => number[][];

  // Conservation
  /** Energy conservation check */
  energyConserved: boolean;
  /** Momentum conservation check */
  momentumConserved: boolean;
}

/**
 * Stability Monitor for Space Folds
 */
export interface StabilityMonitor {
  /** Throat radius as function of time */
  throatRadius: (t: number) => number;
  /** Energy flux through throat */
  energyFlux: (t: number) => number;
  /** Stress-energy tensor field */
  stressEnergy: MatterField;

  /** Collapse risk (0 to 1) */
  collapseRisk: number;
  /** Whether intervention is required */
  interventionRequired: boolean;
  /** Recommended action */
  recommendedAction: string;
}

// ============================================================================
// Warp Bubble (Alcubierre Drive)
// ============================================================================

/**
 * Warp Bubble Configuration
 */
export interface WarpBubble {
  // Position and motion
  /** Current center position */
  position: SpacetimePoint;
  /** Velocity vector v_s (can exceed c) */
  velocity: Vector3D;
  /** Acceleration vector */
  acceleration: Vector3D;

  // Geometry
  /** Bubble radius R (meters) */
  radius: number;
  /** Wall thickness parameter σ */
  wallThickness: number;
  /** Shape function f(r_s) */
  shapeFunction: (r: number) => number;

  // Energy
  /** Energy density (J/m³), negative for exotic matter */
  energyDensity: number;
  /** Total energy required (Joules) */
  totalEnergy: number;
  /** Power requirement (Watts) */
  powerRequirement: number;

  // Spacetime zones
  /** Expansion region (behind bubble) */
  expansionRegion: SpacetimeRegion;
  /** Contraction region (in front of bubble) */
  contractionRegion: SpacetimeRegion;
  /** Flat region (inside bubble) */
  flatRegion: SpacetimeRegion;

  // Metric
  /** Alcubierre metric tensor */
  metricTensor: MetricTensor;

  // Safety
  /** Internal tidal force (should be ~0) */
  internalTidalForce: number;
  /** Stress on bubble wall */
  wallStress: number;
}

/**
 * Warp Bubble Configuration Parameters
 */
export interface WarpBubbleConfig {
  /** Desired effective velocity (m/s, can exceed c) */
  targetVelocity: number;
  /** Payload mass (kg) */
  shipMass: number;
  /** Bubble size (meters) */
  radius: number;
  /** Wall thickness parameter */
  wallThickness: number;
  /** Available exotic matter/energy (Joules) */
  energyBudget: number;
}

/**
 * Hawking Radiation from Bubble
 */
export interface HawkingEmission {
  /** Temperature T ~ ℏa/(2πck_B) (Kelvin) */
  temperature: number;
  /** Luminosity/power emitted (Watts) */
  luminosity: number;
  /** Particle flux (particles/s) */
  particleFlux: number;
  /** Dominant particle species */
  dominantSpecies: string[];
}

// ============================================================================
// Exotic Matter and Negative Energy
// ============================================================================

/**
 * Negative Energy Density Configuration
 */
export interface NegativeEnergyDensity {
  /** Energy density value ρ < 0 (J/m³) */
  value: number;

  /** Generation mechanism */
  mechanism: 'casimir' | 'squeezed' | 'vacuum' | 'quantum';

  // Casimir configuration
  casimirSetup?: {
    /** Plate distance (meters) */
    plateDistance: number;
    /** Plate area (m²) */
    plateArea: number;
    /** Plate material */
    plateMaterial: string;
    /** Calculated energy density */
    energyDensity: number;
  };

  // Squeezed vacuum configuration
  squeezedSetup?: {
    /** Squeezing parameter r */
    squeezing: number;
    /** Frequency ω (Hz) */
    frequency: number;
    /** Quantum mode */
    mode: string;
  };

  // Spatial distribution
  /** Energy density field ρ(x) */
  field: (position: SpacetimePoint) => number;
  /** Total integral ∫ρ dV (Joules) */
  totalIntegral: number;

  // Stability
  /** Quantum fluctuation amplitude */
  quantumFluctuations: number;
  /** Decay rate (1/s) */
  decayRate: number;
}

/**
 * Exotic Matter Configuration
 */
export interface ExoticMatterConfig {
  // Properties
  /** Energy density ρ < 0 (J/m³) */
  energyDensity: number;
  /** Pressure (often p < -ρc²) (Pa) */
  pressure: number;
  /** Equation of state w = p/(ρc²) */
  equationOfState: string;

  // Distribution
  /** Spatial matter field */
  spatialField: MatterField;
  /** Equivalent mass (can be negative) (kg) */
  totalMass: number;
  /** Volume occupied (m³) */
  volume: number;

  // Quantum properties
  /** Quantum state description */
  quantumState: string;
  /** Coherence time (seconds) */
  coherenceTime: number;
  /** Fluctuation amplitude */
  fluctuationAmplitude: number;

  // Production
  /** Production method */
  productionMethod: 'casimir' | 'squeezed' | 'vacuum' | 'wormhole';
  /** Production rate (kg/s equivalent) */
  productionRate: number;
  /** Energy input required (Joules) */
  energyInput: number;

  // Stability
  /** Half-life (seconds), null if stable */
  halfLife: number | null;
  /** Decay products */
  decayProducts: string[];
  /** Containment method */
  containmentMethod: string;
}

/**
 * Casimir Array (multiple cavities)
 */
export interface CasimirArray {
  /** Array of Casimir cavities */
  layers: CasimirCavity[];
  /** Total combined energy density */
  totalEnergyDensity: number;
  /** Geometric configuration */
  geometricConfiguration: string;
  /** Scaling factor for amplification */
  scaleFactor: number;
}

/**
 * Quantum Field Manipulator
 */
export interface QuantumFieldManipulator {
  /** Vacuum state */
  vacuumState: string;
  /** Field mode being manipulated */
  fieldMode: string;
  /** Excitation level */
  excitation: number;
  /** Squeezing parameter */
  squeezing: number;
  /** Resultant energy density */
  resultantEnergy: number;
}

// ============================================================================
// Spacetime Bubble
// ============================================================================

/**
 * General Spacetime Bubble
 */
export interface SpacetimeBubble {
  // Geometry
  /** Center position */
  centerPosition: SpacetimePoint;
  /** Bubble radius (meters) */
  radius: number;
  /** Wall thickness (meters) */
  wallThickness: number;

  // Metric
  /** Interior metric (usually flat) */
  interiorMetric: MetricTensor;
  /** Exterior metric (background spacetime) */
  exteriorMetric: MetricTensor;
  /** Smooth transition function */
  transitionFunction: (r: number) => MetricTensor;

  // Dynamics
  /** Velocity vector */
  velocity: Vector3D;
  /** Acceleration vector */
  acceleration: Vector3D;
  /** Angular velocity */
  rotation: Vector3D;

  // Energy
  /** Total energy (Joules) */
  totalEnergy: number;
  /** Exotic matter mass equivalent (kg) */
  exoticMatterMass: number;
  /** Power consumption (Watts) */
  powerConsumption: number;

  // Contents
  payload: {
    /** Payload mass (kg) */
    mass: number;
    /** Payload volume (m³) */
    volume: number;
    /** Contents description */
    contents: string[];
  };

  // Internal conditions
  /** Internal gravity vector (m/s²) */
  internalGravity: Vector3D;
  /** Proper time inside bubble (seconds) */
  internalTime: number;
  /** Tidal force magnitude (N/m) */
  tidalForce: number;

  // Safety
  /** Structural integrity (0 to 1) */
  integrity: number;
  /** Risk of collapse (0 to 1) */
  collapseRisk: number;
  /** Radiation level (W/m²) */
  radiationLevel: number;
}

/**
 * Bubble State for monitoring
 */
export interface BubbleState {
  position: SpacetimePoint;
  velocity: Vector3D;
  integrity: number;
  energyLevel: number;
  timestamp: number;
}

/**
 * Control Action for bubble management
 */
export interface ControlAction {
  /** Exotic matter injection rate */
  matterInjectionRate: number;
  /** Field modulation amplitude */
  fieldModulation: number;
  /** Target velocity adjustment */
  velocityAdjustment: Vector3D;
  /** Emergency shutdown flag */
  emergencyShutdown: boolean;
}

// ============================================================================
// Sensors and Control
// ============================================================================

/**
 * Metric Sensor
 */
export interface MetricSensor {
  id: string;
  position: SpacetimePoint;
  measuredMetric: MetricTensor;
  accuracy: number;
  updateRate: number; // Hz
}

/**
 * Energy Sensor
 */
export interface EnergySensor {
  id: string;
  position: SpacetimePoint;
  energyDensity: number;
  powerFlow: Vector3D;
  accuracy: number;
}

/**
 * Stress Sensor
 */
export interface StressSensor {
  id: string;
  position: SpacetimePoint;
  stressTensor: number[][];
  tidalForce: number;
  accuracy: number;
}

/**
 * Matter Injector (exotic matter source)
 */
export interface MatterInjector {
  id: string;
  position: SpacetimePoint;
  injectionRate: number;
  totalCapacity: number;
  remainingCapacity: number;
}

/**
 * Field Modulator
 */
export interface FieldModulator {
  id: string;
  position: SpacetimePoint;
  frequency: number;
  amplitude: number;
  phase: number;
}

/**
 * Bubble Controller
 */
export interface BubbleController {
  // Sensors
  metricSensors: MetricSensor[];
  energySensors: EnergySensor[];
  stressSensors: StressSensor[];

  // Actuators
  exoticMatterInjectors: MatterInjector[];
  fieldModulators: FieldModulator[];

  // Control algorithms
  stabilityControl: (state: BubbleState) => ControlAction;
  navigationControl: (target: Vector3D) => ControlAction;
  energyManagement: (budget: number) => ControlAction;

  // Emergency systems
  emergencyShutdown: () => void;
  bubbleBurst: () => void;
  payloadEject: () => void;
}

// ============================================================================
// Validation and Safety
// ============================================================================

/**
 * Spacetime Integrity Validation
 */
export interface SpacetimeIntegrity {
  // Metric properties
  /** Determinant of metric (must be negative) */
  metricDeterminant: number;
  /** Metric signature, e.g., "(-,+,+,+)" */
  metricSignature: string;
  /** Smoothness measure (derivatives bounded) */
  smoothness: number;

  // Curvature limits
  /** Maximum curvature (m⁻²) */
  maximumCurvature: number;
  /** Tidal force limit (N/m) */
  tidalForceLimit: number;
  /** Singularity check (must be false) */
  singularityCheck: boolean;

  // Energy conditions
  /** Weak energy condition ρ ≥ 0 */
  weakEnergyCondition: boolean;
  /** Null energy condition T_μν k^μ k^ν ≥ 0 */
  nullEnergyCondition: boolean;
  /** Dominant energy condition */
  dominantEnergyCondition: boolean;

  // Causality
  /** Closed timelike curves present (must be false) */
  closedTimelikeCurves: boolean;
  /** Chronology protection satisfied (must be true) */
  chronologyProtection: boolean;

  // Stability
  /** Rayleigh index (> 0 for stability) */
  rayleighIndex: number;
  /** Lyapunov exponent (< 0 for stability) */
  lyapunovExponent: number;

  // Overall assessment
  /** Is spacetime safe? */
  safe: boolean;
  /** Risk level */
  riskLevel: 'low' | 'medium' | 'high' | 'critical';
  /** Warning messages */
  warnings: string[];
  /** Recommendations */
  recommendations: string[];
}

/**
 * Validation Check Result
 */
export interface ValidationCheck {
  name: string;
  pass: boolean;
  value: any;
  expected: any;
  message?: string;
}

/**
 * Validation Result
 */
export interface ValidationResult {
  checks: ValidationCheck[];
  allPassed: boolean;
  timestamp: number;
}

/**
 * Causality Check
 */
export interface CausalityCheck {
  /** Closed timelike curves present */
  ctcPresent: boolean;
  /** Chronology violation detected */
  chronologyViolation: boolean;
  /** Causal structure valid */
  causalStructureValid: boolean;
  /** Overall safety */
  safe: boolean;
}

/**
 * Safety Limits
 */
export interface SafetyLimits {
  // Tidal forces
  /** Maximum tidal force (N/m) */
  maxTidalForce: number;
  /** Safe for biological matter (N/m) */
  biologicalTidalLimit: number;
  /** Safe for spacecraft structure (N/m) */
  structuralTidalLimit: number;

  // Radiation
  /** Maximum Hawking radiation (W/m²) */
  maxHawkingRadiation: number;
  /** Maximum particle flux (particles/(m²·s)) */
  maxParticleFlux: number;
  /** Required shielding thickness (meters) */
  shieldingRequirement: number;

  // Energy
  /** Maximum negative energy (Joules) */
  maxNegativeEnergy: number;
  /** Maximum energy density (J/m³) */
  maxEnergyDensity: number;
  /** Quantum fluctuation limit */
  quantumFluctuationLimit: number;

  // Curvature
  /** Maximum curvature (m⁻²) */
  maxCurvature: number;
  /** Maximum Ricci scalar */
  maxScalarCurvature: number;

  // Time
  /** Maximum time dilation ratio */
  maxTimeDilation: number;
  /** Maximum proper time rate */
  maxProperTimeRate: number;
}

/**
 * Emergency Protocol
 */
export interface EmergencyProtocol {
  // Triggers
  triggers: {
    /** Integrity failure threshold (0 to 1) */
    integrityFailure: number;
    /** Energy depletion threshold */
    energyDepletion: number;
    /** Curvature exceedance threshold */
    curvatureExceedance: number;
    /** Causality violation flag */
    causalityViolation: boolean;
  };

  // Actions
  actions: {
    gradualShutdown: () => void;
    emergencyCollapse: () => void;
    payloadSeparation: () => void;
    distressBeacon: () => void;
  };

  // Safeguards
  /** Automatic shutdown enabled */
  automaticShutdown: boolean;
  /** Manual override allowed */
  manualOverride: boolean;
  /** Failsafe action */
  failsafe: 'collapse' | 'maintain' | 'expand';
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Physical Constants
 */
export const CONSTANTS = {
  // Fundamental constants
  /** Speed of light (m/s) */
  SPEED_OF_LIGHT: 299792458,
  /** Gravitational constant (m³/(kg·s²)) */
  GRAVITATIONAL_CONSTANT: 6.67430e-11,
  /** Planck constant (J·s) */
  PLANCK_CONSTANT: 6.62607015e-34,
  /** Reduced Planck constant ℏ (J·s) */
  PLANCK_REDUCED: 1.054571817e-34,

  // Derived Planck units
  /** Planck length (m) */
  PLANCK_LENGTH: 1.616255e-35,
  /** Planck time (s) */
  PLANCK_TIME: 5.391247e-44,
  /** Planck mass (kg) */
  PLANCK_MASS: 2.176434e-8,
  /** Planck energy (J) */
  PLANCK_ENERGY: 1.956e9,

  // Cosmological
  /** Solar mass (kg) */
  SOLAR_MASS: 1.989e30,
  /** Earth mass (kg) */
  EARTH_MASS: 5.972e24,
  /** Astronomical unit (m) */
  ASTRONOMICAL_UNIT: 1.496e11,

  // Safety limits
  /** Maximum tidal force safe for humans (N/m) */
  MAX_TIDAL_FORCE_HUMAN: 1e6,
  /** Maximum safe curvature (m⁻²) */
  MAX_CURVATURE_SAFE: 1e-10,
} as const;

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Coordinate System Type
 */
export type CoordinateSystem = 'cartesian' | 'spherical' | 'schwarzschild' | 'alcubierre';

/**
 * Matter Type
 */
export type MatterType = 'normal' | 'exotic' | 'dark' | 'vacuum';

/**
 * Production Method for Exotic Matter
 */
export type ProductionMethod = 'casimir' | 'squeezed' | 'vacuum' | 'wormhole';

/**
 * Energy Mechanism
 */
export type EnergyMechanism = 'casimir' | 'squeezed' | 'vacuum' | 'quantum';

/**
 * Risk Level
 */
export type RiskLevel = 'low' | 'medium' | 'high' | 'critical';

/**
 * Failsafe Action
 */
export type FailsafeAction = 'collapse' | 'maintain' | 'expand';
