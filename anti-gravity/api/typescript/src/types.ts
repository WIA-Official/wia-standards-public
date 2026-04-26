/**
 * WIA-QUA-012: Anti-Gravity - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Quantum & Advanced Physics Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * Three-dimensional vector
 */
export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

/**
 * Four-dimensional spacetime coordinates
 */
export interface SpacetimeCoordinate {
  t: number;  // Time (seconds)
  x: number;  // Meters
  y: number;  // Meters
  z: number;  // Meters
}

/**
 * Metric tensor (simplified 4×4 matrix representation)
 */
export type MetricTensor = number[][];

/**
 * Energy-momentum tensor
 */
export interface StressEnergyTensor {
  energyDensity: number;           // ρ (kg/m³ or J/m³)
  pressure: Vector3;               // p (Pa)
  momentumDensity: Vector3;        // g (kg·m/s/m³)
  stressTensor: number[][];        // 3×3 spatial stress
}

// ============================================================================
// Anti-Gravity Field Types
// ============================================================================

/**
 * Anti-gravity field types
 */
export type FieldType =
  | 'electromagnetic'
  | 'exotic-matter'
  | 'quantum-vacuum'
  | 'casimir'
  | 'frame-dragging'
  | 'hybrid';

/**
 * Field geometry configurations
 */
export type FieldGeometry =
  | 'spherical'
  | 'cylindrical'
  | 'toroidal'
  | 'ellipsoidal'
  | 'custom';

/**
 * Stabilization modes
 */
export type StabilizationMode =
  | 'passive'
  | 'active'
  | 'feedback-controlled'
  | 'adaptive';

/**
 * Anti-gravity field configuration
 */
export interface AntiGravityFieldConfig {
  // Field properties
  fieldType: FieldType;
  geometry: FieldGeometry;
  strength: number;                // g units (1.0 = Earth gravity)
  radius: number;                  // meters
  thickness?: number;              // meters (for toroidal/shell)

  // Energy configuration
  energySource: 'nuclear' | 'antimatter' | 'vacuum' | 'zero-point' | 'fusion';
  powerOutput: number;             // Megawatts
  negativeEnergyDensity: number;   // J/m³ (negative value required)

  // Control parameters
  stabilization: StabilizationMode;
  feedbackControl: boolean;
  safetyInterlocks: boolean;
  maxFieldGradient: number;        // g/m

  // Environmental constraints
  temperatureRange: [number, number]; // Kelvin
  pressureRange: [number, number];    // Pascals
  magneticShielding: boolean;
  radiationShielding: boolean;
}

/**
 * Anti-gravity field state
 */
export interface AntiGravityFieldState {
  active: boolean;
  strength: number;                // Current field strength (g units)
  stability: number;               // 0-1 (1 = perfectly stable)
  powerDraw: number;               // Current power consumption (MW)
  temperature: number;             // System temperature (K)
  curvature: number;               // Spacetime curvature (1/m²)
  energyDensity: number;           // Current energy density (J/m³)
  liftForce: number;               // Total lift force (N)
  efficiency: number;              // Energy efficiency (0-1)
  warnings: string[];              // Active warnings
  errors: string[];                // Active errors
}

// ============================================================================
// Warp Drive Types
// ============================================================================

/**
 * Warp drive configuration
 */
export interface WarpDriveConfig {
  // Warp parameters
  warpFactor: number;              // Multiple of light speed (>1 = FTL)
  bubbleRadius: number;            // meters
  bubbleThickness: number;         // meters
  shapeFunction: (r: number) => number; // Warp bubble shape

  // Ship properties
  shipMass: number;                // kg
  shipVolume: number;              // m³
  passengerCapacity: number;
  cargoCapacity: number;           // kg

  // Exotic matter requirements
  exoticMatter: 'casimir-enhanced' | 'quantum-vacuum' | 'negative-mass' | 'hypothetical';
  exoticMassDensity: number;       // kg/m³ (negative)
  exoticMassTotal: number;         // kg (total exotic matter required)
  stabilityMargin: number;         // Safety factor (>1)

  // Trajectory parameters
  maxAcceleration: number;         // g units
  maxVelocity: number;             // Multiple of c
  turningRadius: number;           // meters
  emergencyShutdown: boolean;
  hawkingRadiationShielding: boolean;
}

/**
 * Warp drive state
 */
export interface WarpDriveState {
  engaged: boolean;
  warpFactor: number;              // Current warp factor
  effectiveVelocity: number;       // Velocity in units of c
  bubbleIntegrity: number;         // 0-1 (1 = perfect integrity)
  energyDensity: number;           // J/m³ (negative)
  spacetimeCurvature: number;      // 1/m²
  hawkingRadiation: number;        // W/m² (radiation intensity)
  destinationETA: number;          // seconds
  fuelRemaining: number;           // kg or % depending on source
  trajectory: {
    current: SpacetimeCoordinate;
    destination: SpacetimeCoordinate;
    velocity: Vector3;
  };
}

// ============================================================================
// Casimir Effect Types
// ============================================================================

/**
 * Casimir plate configuration
 */
export interface CasimirPlateConfig {
  // Plate geometry
  plateArea: number;               // m²
  plateSeparation: number;         // meters (typically nanometers)
  plateThickness: number;          // meters
  plateMaterial: 'gold' | 'silver' | 'aluminum' | 'copper' | 'perfect-conductor';

  // Configuration
  geometry: 'parallel-plates' | 'sphere-plate' | 'sphere-sphere' | 'cylinder';
  surfaceRoughness: number;        // meters RMS
  temperature: number;             // Kelvin

  // Dynamic Casimir
  oscillating: boolean;
  oscillationFrequency?: number;   // Hz
  oscillationAmplitude?: number;   // meters
}

/**
 * Casimir force measurement
 */
export interface CasimirMeasurement {
  force: number;                   // Newtons (negative = attractive)
  energyDensity: number;           // J/m³ (negative)
  pressure: number;                // Pa
  photonCreationRate?: number;     // photons/second (dynamic Casimir)
  uncertainty: number;             // Newtons
  temperature: number;             // K
  timestamp: Date | string;
}

// ============================================================================
// Gravitational Propulsion Types
// ============================================================================

/**
 * Propulsion system types
 */
export type PropulsionType =
  | 'field-gradient'
  | 'mass-oscillation'
  | 'asymmetric-field'
  | 'warp-drive'
  | 'frame-dragging'
  | 'hybrid';

/**
 * Gravitational propulsion configuration
 */
export interface GravitationalPropulsionConfig {
  type: PropulsionType;
  maxThrust: number;               // Newtons
  maxAcceleration: number;         // m/s²
  specificImpulse: number;         // seconds (∞ for reactionless)
  powerRequirement: number;        // Watts
  efficiency: number;              // 0-1
  reactionless: boolean;           // True if no propellant needed
}

/**
 * Thrust vector
 */
export interface ThrustVector {
  magnitude: number;               // Newtons
  direction: Vector3;              // Unit vector
  efficiency: number;              // 0-1
  powerConsumption: number;        // Watts
}

// ============================================================================
// Inertial Mass Modification Types
// ============================================================================

/**
 * Inertial mass modification configuration
 */
export interface InertialMassConfig {
  nominalMass: number;             // kg (rest mass)
  targetEffectiveMass: number;     // kg (desired effective mass)
  reductionFactor: number;         // 0-1 (0 = massless, 1 = no change)
  method: 'higgs-interaction' | 'em-field' | 'quantum-vacuum' | 'exotic-matter';
  powerRequirement: number;        // Watts
  stabilityMargin: number;         // Safety factor
}

/**
 * Effective mass measurement
 */
export interface EffectiveMassMeasurement {
  nominalMass: number;             // kg
  effectiveMass: number;           // kg
  inertialMass: number;            // kg
  gravitationalMass: number;       // kg
  reductionAchieved: number;       // 0-1
  stability: number;               // 0-1
  timestamp: Date | string;
}

// ============================================================================
// Vehicle Design Types
// ============================================================================

/**
 * Vehicle configuration types
 */
export type VehicleConfiguration =
  | 'saucer'
  | 'cylinder'
  | 'sphere'
  | 'triangle'
  | 'ellipsoid'
  | 'custom';

/**
 * Anti-gravity vehicle configuration
 */
export interface AntiGravityVehicleConfig {
  // Basic properties
  vehicleId: string;
  configuration: VehicleConfiguration;
  mass: number;                    // kg (dry mass)
  volume: number;                  // m³
  dimensions: {
    length: number;                // meters
    width: number;                 // meters
    height: number;                // meters
    diameter?: number;             // meters (for circular)
  };

  // Propulsion systems
  primaryPropulsion: GravitationalPropulsionConfig;
  backupPropulsion?: GravitationalPropulsionConfig;
  emergencyPropulsion?: {
    type: 'chemical' | 'ion' | 'nuclear';
    deltaV: number;                // m/s
  };

  // Field generators
  fieldGenerators: AntiGravityFieldConfig[];
  redundancy: number;              // Number of backup generators

  // Power systems
  primaryPower: {
    type: 'nuclear' | 'fusion' | 'antimatter' | 'zero-point';
    capacity: number;              // MWh
    maxOutput: number;             // MW
  };
  batteryBackup: {
    capacity: number;              // MWh
    duration: number;              // hours at full load
  };

  // Crew and cargo
  crewCapacity: number;
  passengerCapacity: number;
  cargoCapacity: number;           // kg

  // Safety systems
  shielding: {
    radiation: boolean;
    electromagnetic: boolean;
    particle: boolean;
    thermal: boolean;
  };
  emergencySystems: {
    ejectionSeats: boolean;
    parachutes: boolean;
    lifeboats: boolean;
    forceField: boolean;
  };
}

/**
 * Vehicle telemetry
 */
export interface VehicleTelemetry {
  timestamp: Date | string;
  position: SpacetimeCoordinate;
  velocity: Vector3;               // m/s
  acceleration: Vector3;           // m/s²
  attitude: {
    roll: number;                  // degrees
    pitch: number;                 // degrees
    yaw: number;                   // degrees
  };
  fieldStatus: AntiGravityFieldState[];
  propulsionStatus: ThrustVector;
  powerStatus: {
    generation: number;            // MW
    consumption: number;           // MW
    batteryLevel: number;          // %
  };
  environmentalStatus: {
    altitude: number;              // meters
    temperature: number;           // K
    pressure: number;              // Pa
    radiation: number;             // mSv/h
  };
  systemHealth: {
    overall: number;               // 0-1
    propulsion: number;            // 0-1
    power: number;                 // 0-1
    lifeSupport: number;           // 0-1
    navigation: number;            // 0-1
  };
}

// ============================================================================
// Energy Calculation Types
// ============================================================================

/**
 * Energy calculation request
 */
export interface EnergyCalculationRequest {
  scenario: 'hover' | 'acceleration' | 'warp-drive' | 'field-generation';
  vehicleMass: number;             // kg
  duration?: number;               // seconds
  targetVelocity?: number;         // m/s or multiples of c
  fieldStrength?: number;          // g units
  fieldVolume?: number;            // m³
}

/**
 * Energy calculation result
 */
export interface EnergyCalculationResult {
  totalEnergy: number;             // Joules
  averagePower: number;            // Watts
  peakPower: number;               // Watts
  efficiency: number;              // 0-1
  energyBreakdown: {
    field: number;                 // Joules
    propulsion: number;            // Joules
    lifeSupport: number;           // Joules
    control: number;               // Joules
    losses: number;                // Joules
  };
  feasibility: 'practical' | 'challenging' | 'theoretical' | 'impossible';
  recommendation: string;
}

// ============================================================================
// Safety Types
// ============================================================================

/**
 * Safety assessment
 */
export interface SafetyAssessment {
  assessmentId: string;
  timestamp: Date | string;
  overallRisk: 'low' | 'medium' | 'high' | 'critical';
  riskFactors: {
    fieldStrength: 'safe' | 'warning' | 'danger';
    energyLevel: 'safe' | 'warning' | 'danger';
    radiation: 'safe' | 'warning' | 'danger';
    spacetimeStability: 'safe' | 'warning' | 'danger';
    systemIntegrity: 'safe' | 'warning' | 'danger';
  };
  recommendations: string[];
  requiredActions: string[];
  emergencyProcedures: string[];
}

/**
 * Containment status
 */
export interface ContainmentStatus {
  primary: boolean;                // Primary containment active
  secondary: boolean;              // Secondary containment active
  tertiary: boolean;               // Tertiary containment active
  fieldIntegrity: number;          // 0-1
  leakageRate: number;             // J/s or particles/s
  alarms: string[];
  failsafeArmed: boolean;
}

// ============================================================================
// Measurement and Validation Types
// ============================================================================

/**
 * Field strength measurement
 */
export interface FieldMeasurement {
  measurementId: string;
  timestamp: Date | string;
  position: Vector3;               // meters from field center
  fieldStrength: Vector3;          // g units (vector)
  magnitude: number;               // g units (scalar)
  gradient: number;                // g/m
  uncertainty: number;             // g units
  quality: 'excellent' | 'good' | 'fair' | 'poor';
}

/**
 * Spacetime curvature measurement
 */
export interface CurvatureMeasurement {
  measurementId: string;
  timestamp: Date | string;
  location: SpacetimeCoordinate;
  ricciScalar: number;             // 1/m²
  ricciTensor: number[][];         // Simplified representation
  weylTensor?: number[][];         // Tidal forces
  energyDensity: number;           // J/m³
  stress: StressEnergyTensor;
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Physical constants for anti-gravity calculations
 */
export const ANTIGRAVITY_CONSTANTS = {
  /** Speed of light (m/s) */
  SPEED_OF_LIGHT: 299792458,

  /** Gravitational constant (m³ kg⁻¹ s⁻²) */
  GRAVITATIONAL_CONSTANT: 6.67430e-11,

  /** Planck constant (J·s) */
  PLANCK_CONSTANT: 6.62607015e-34,

  /** Reduced Planck constant (J·s) */
  REDUCED_PLANCK: 1.054571817e-34,

  /** Earth gravity (m/s²) */
  EARTH_GRAVITY: 9.80665,

  /** Planck length (m) */
  PLANCK_LENGTH: 1.616255e-35,

  /** Planck mass (kg) */
  PLANCK_MASS: 2.176434e-8,

  /** Planck energy (J) */
  PLANCK_ENERGY: 1.956082e9,

  /** Hubble constant (km/s/Mpc) */
  HUBBLE_CONSTANT: 70,

  /** Critical density of universe (kg/m³) */
  CRITICAL_DENSITY: 9.47e-27,

  /** Vacuum permittivity (F/m) */
  VACUUM_PERMITTIVITY: 8.854187817e-12,

  /** Vacuum permeability (H/m) */
  VACUUM_PERMEABILITY: 1.25663706212e-6,

  /** Elementary charge (C) */
  ELEMENTARY_CHARGE: 1.602176634e-19,

  /** Solar mass (kg) */
  SOLAR_MASS: 1.98892e30,

  /** Jupiter mass (kg) */
  JUPITER_MASS: 1.8982e27,
} as const;

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-QUA-012 error codes
 */
export enum AntiGravityErrorCode {
  FIELD_GENERATION_FAILED = 'AG001',
  INSUFFICIENT_POWER = 'AG002',
  EXOTIC_MATTER_SHORTAGE = 'AG003',
  CONTAINMENT_BREACH = 'AG004',
  SPACETIME_INSTABILITY = 'AG005',
  ENERGY_CONDITION_VIOLATION = 'AG006',
  HAWKING_RADIATION_CRITICAL = 'AG007',
  FIELD_COLLAPSE = 'AG008',
  NAVIGATION_ERROR = 'AG009',
  SYSTEM_OVERLOAD = 'AG010',
  SAFETY_INTERLOCK_TRIGGERED = 'AG011',
  CALIBRATION_ERROR = 'AG012',
}

/**
 * Anti-gravity system error
 */
export class AntiGravityError extends Error {
  constructor(
    public code: AntiGravityErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'AntiGravityError';
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
 * Time series data for monitoring
 */
export interface TimeSeriesData<T> {
  startTime: Date | string;
  endTime: Date | string;
  sampleRate: number;              // Hz
  dataPoints: {
    timestamp: Date | string;
    value: T;
  }[];
}

// ============================================================================
// Export All Types
// ============================================================================

export type {
  // Core
  Vector3,
  SpacetimeCoordinate,
  MetricTensor,
  StressEnergyTensor,

  // Anti-gravity fields
  FieldType,
  FieldGeometry,
  StabilizationMode,
  AntiGravityFieldConfig,
  AntiGravityFieldState,

  // Warp drive
  WarpDriveConfig,
  WarpDriveState,

  // Casimir effect
  CasimirPlateConfig,
  CasimirMeasurement,

  // Propulsion
  PropulsionType,
  GravitationalPropulsionConfig,
  ThrustVector,

  // Inertial mass
  InertialMassConfig,
  EffectiveMassMeasurement,

  // Vehicles
  VehicleConfiguration,
  AntiGravityVehicleConfig,
  VehicleTelemetry,

  // Energy
  EnergyCalculationRequest,
  EnergyCalculationResult,

  // Safety
  SafetyAssessment,
  ContainmentStatus,

  // Measurements
  FieldMeasurement,
  CurvatureMeasurement,

  // Utility
  TimeSeriesData,
};

export {
  ANTIGRAVITY_CONSTANTS,
  AntiGravityErrorCode,
  AntiGravityError,
};

/**
 * 弘益人間 (홍익인간) · Benefit All Humanity
 */
