/**
 * WIA-DEF-008: Hypersonic Weapon - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Defense Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Physics Types
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
 * Vehicle type classification
 */
export type VehicleType = 'hgv' | 'hcm' | 'interceptor' | 'research';

/**
 * Propulsion system type
 */
export type PropulsionType =
  | 'scramjet'
  | 'ramjet'
  | 'dual-mode'
  | 'rocket-boost'
  | 'combined-cycle';

/**
 * Material type for thermal protection
 */
export type MaterialType =
  | 'carbon-carbon'
  | 'rcc'
  | 'uhtc'
  | 'ablative'
  | 'tungsten'
  | 'ceramic-composite';

// ============================================================================
// Flight Parameters
// ============================================================================

/**
 * Flight conditions at a specific point
 */
export interface FlightConditions {
  /** Velocity in m/s */
  velocity: number;

  /** Mach number */
  machNumber: number;

  /** Altitude in meters */
  altitude: number;

  /** Atmospheric density in kg/m³ */
  density: number;

  /** Atmospheric temperature in Kelvin */
  temperature: number;

  /** Dynamic pressure in Pascals */
  dynamicPressure: number;

  /** Angle of attack in degrees */
  angleOfAttack?: number;

  /** Flight path angle in degrees */
  flightPathAngle?: number;
}

/**
 * Aerodynamic coefficients
 */
export interface AerodynamicCoefficients {
  /** Lift coefficient */
  CL: number;

  /** Drag coefficient */
  CD: number;

  /** Lift-to-drag ratio */
  liftToDragRatio: number;

  /** Pitching moment coefficient */
  CM?: number;

  /** Normal force coefficient */
  CN?: number;
}

/**
 * Vehicle geometry
 */
export interface VehicleGeometry {
  /** Nose radius in meters */
  noseRadius: number;

  /** Length in meters */
  length: number;

  /** Wingspan in meters */
  wingspan: number;

  /** Reference area in m² */
  referenceArea: number;

  /** Mass in kilograms */
  mass: number;

  /** Center of gravity (distance from nose) */
  centerOfGravity?: number;
}

// ============================================================================
// Thermal Analysis
// ============================================================================

/**
 * Heat flux calculation parameters
 */
export interface HeatFluxParams {
  /** Velocity in m/s */
  velocity: number;

  /** Altitude in meters */
  altitude: number;

  /** Nose radius in meters */
  noseRadius: number;

  /** Position on vehicle (0=nose, 1=tail) */
  position?: number;
}

/**
 * Heat flux calculation result
 */
export interface HeatFluxResult {
  /** Heat flux in W/m² */
  heatFlux: number;

  /** Stagnation temperature in Kelvin */
  stagnationTemperature: number;

  /** Wall temperature estimate in Kelvin */
  wallTemperature: number;

  /** Recommended material type */
  recommendedMaterial: MaterialType;

  /** Cooling requirement in kW/m² */
  coolingRequired?: number;

  /** Is active cooling needed? */
  needsActiveCooling: boolean;
}

/**
 * Thermal protection system specification
 */
export interface ThermalProtectionSystem {
  /** Material type */
  material: MaterialType;

  /** Thickness in meters */
  thickness: number;

  /** Maximum operating temperature in Kelvin */
  maxTemperature: number;

  /** Thermal conductivity in W/(m·K) */
  thermalConductivity: number;

  /** Density in kg/m³ */
  density: number;

  /** Mass per unit area in kg/m² */
  arealDensity: number;

  /** Active cooling system */
  activeCooling?: {
    type: 'regenerative' | 'transpiration' | 'film';
    coolantType: string;
    flowRate: number; // kg/s
    coolingCapacity: number; // W/m²
  };
}

// ============================================================================
// Propulsion Systems
// ============================================================================

/**
 * Scramjet engine parameters
 */
export interface ScramjetParams {
  /** Flight Mach number */
  machNumber: number;

  /** Altitude in meters */
  altitude: number;

  /** Fuel type */
  fuelType: 'hydrogen' | 'jp-7' | 'jp-10' | 'methane';

  /** Inlet area in m² */
  inletArea: number;

  /** Combustor pressure in Pa */
  combustorPressure?: number;

  /** Fuel equivalence ratio */
  equivalenceRatio?: number;
}

/**
 * Engine performance result
 */
export interface EnginePerformance {
  /** Thrust in Newtons */
  thrust: number;

  /** Specific impulse in seconds */
  specificImpulse: number;

  /** Fuel flow rate in kg/s */
  fuelFlowRate: number;

  /** Thrust-to-weight ratio */
  thrustToWeight: number;

  /** Combustion efficiency */
  efficiency: number;

  /** Operating range (min/max Mach) */
  operatingRange: {
    minMach: number;
    maxMach: number;
  };
}

// ============================================================================
// Trajectory and Guidance
// ============================================================================

/**
 * Trajectory optimization parameters
 */
export interface TrajectoryParams {
  /** Launch angle in degrees */
  launchAngle: number;

  /** Initial velocity in m/s */
  initialVelocity: number;

  /** Target range in meters */
  targetRange: number;

  /** Vehicle type */
  vehicleType: VehicleType;

  /** Lift-to-drag ratio */
  liftToDragRatio?: number;

  /** Constraints */
  constraints?: {
    maxAcceleration?: number; // g's
    maxDynamicPressure?: number; // Pa
    maxHeatFlux?: number; // W/m²
  };
}

/**
 * Trajectory optimization result
 */
export interface TrajectoryResult {
  /** Actual range achieved in meters */
  range: number;

  /** Number of atmospheric skips */
  skipCount: number;

  /** Flight time in seconds */
  flightTime: number;

  /** Maximum altitude in meters */
  maxAltitude: number;

  /** Average velocity in m/s */
  averageVelocity: number;

  /** Trajectory profile (time, altitude, velocity) */
  profile: TrajectoryPoint[];

  /** Terminal velocity in m/s */
  terminalVelocity: number;

  /** Impact angle in degrees */
  impactAngle: number;
}

/**
 * Point along trajectory
 */
export interface TrajectoryPoint {
  /** Time since launch in seconds */
  time: number;

  /** Position */
  position: Vector3;

  /** Velocity in m/s */
  velocity: Vector3;

  /** Altitude in meters */
  altitude: number;

  /** Mach number */
  machNumber: number;

  /** Heat flux in W/m² */
  heatFlux: number;

  /** Acceleration in g's */
  acceleration: number;
}

/**
 * Guidance system parameters
 */
export interface GuidanceParams {
  /** Current position */
  currentPosition: Vector3;

  /** Target position */
  targetPosition: Vector3;

  /** Current velocity */
  currentVelocity: Vector3;

  /** Navigation system type */
  navigationSystem: 'ins' | 'star-tracker' | 'terrain-matching' | 'combined';

  /** GPS available? */
  gpsAvailable: boolean;

  /** Plasma sheath active? */
  plasmaSheathActive: boolean;
}

/**
 * Guidance result
 */
export interface GuidanceResult {
  /** Required heading in degrees */
  heading: number;

  /** Required pitch in degrees */
  pitch: number;

  /** Required bank angle in degrees */
  bankAngle: number;

  /** Control deflection commands */
  controlDeflections: {
    elevator: number; // degrees
    rudder: number; // degrees
    aileron: number; // degrees
  };

  /** Predicted CEP in meters */
  predictedCEP: number;

  /** Time to impact in seconds */
  timeToImpact: number;

  /** Navigation accuracy estimate */
  accuracyEstimate: number; // meters
}

// ============================================================================
// Detection and Countermeasures
// ============================================================================

/**
 * Detectability analysis parameters
 */
export interface DetectabilityParams {
  /** Altitude in meters */
  altitude: number;

  /** Velocity in m/s */
  velocity: number;

  /** Radar cross section in m² */
  radarCrossSection: number;

  /** Surface temperature in Kelvin */
  surfaceTemperature?: number;

  /** Engine plume visible? */
  enginePlumeVisible?: boolean;

  /** Radar type */
  radarType?: 'x-band' | 's-band' | 'l-band' | 'oth';
}

/**
 * Detectability analysis result
 */
export interface DetectabilityResult {
  /** Radar detection range in meters */
  radarDetectionRange: number;

  /** Infrared detection range in meters */
  infraredDetectionRange: number;

  /** Overall stealth rating (0-1, higher is stealthier) */
  stealthRating: number;

  /** Detection time before impact in seconds */
  detectionTime: number;

  /** Recommended countermeasures */
  countermeasures: string[];

  /** Vulnerability assessment */
  vulnerability: 'low' | 'medium' | 'high';
}

/**
 * Defensive countermeasures
 */
export interface Countermeasures {
  /** Electronic warfare */
  electronicWarfare?: {
    jammingType: 'noise' | 'deception' | 'combined';
    effectiveRange: number; // meters
    powerOutput: number; // watts
  };

  /** Decoys */
  decoys?: {
    count: number;
    type: 'infrared' | 'radar' | 'combined';
    deploymentAltitude: number; // meters
  };

  /** Maneuvering */
  evasiveManeuvers?: {
    maxLateralAcceleration: number; // g's
    pullUpCapability: number; // g's
    crossRange: number; // meters
  };
}

// ============================================================================
// Simulation and Analysis
// ============================================================================

/**
 * Complete flight simulation parameters
 */
export interface SimulationParams {
  /** Vehicle configuration */
  vehicle: {
    type: VehicleType;
    geometry: VehicleGeometry;
    propulsion: PropulsionType;
    tps: MaterialType;
  };

  /** Flight profile */
  flight: {
    launchAngle: number;
    initialVelocity: number;
    targetRange: number;
  };

  /** Environment */
  environment?: {
    windSpeed?: number; // m/s
    windDirection?: number; // degrees
    atmosphericModel?: 'standard' | 'tropical' | 'polar';
  };

  /** Simulation settings */
  settings?: {
    timeStep?: number; // seconds
    maxTime?: number; // seconds
    convergenceTolerance?: number;
  };
}

/**
 * Simulation result
 */
export interface SimulationResult {
  /** Simulation ID */
  id: string;

  /** Success status */
  success: boolean;

  /** Error message if failed */
  error?: string;

  /** Trajectory data */
  trajectory: TrajectoryResult;

  /** Heat load analysis */
  thermal: {
    maxHeatFlux: number;
    maxTemperature: number;
    totalHeatLoad: number; // MJ
    tpsAdequacy: boolean;
  };

  /** Performance metrics */
  performance: {
    range: number;
    accuracy: number; // CEP in meters
    flightTime: number;
    terminalVelocity: number;
  };

  /** Detectability */
  detectability: DetectabilityResult;

  /** Defensive rating */
  defensiveRating: {
    overall: number; // 0-1
    deterrenceValue: 'low' | 'medium' | 'high' | 'strategic';
    recommendedRole: string;
  };
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Physical constants for hypersonic calculations
 */
export const HYPERSONIC_CONSTANTS = {
  /** Speed of sound at sea level (m/s) */
  SPEED_OF_SOUND_SEA_LEVEL: 343,

  /** Gravitational acceleration (m/s²) */
  GRAVITY: 9.81,

  /** Gas constant for air (J/(kg·K)) */
  GAS_CONSTANT_AIR: 287,

  /** Specific heat ratio for air */
  GAMMA_AIR: 1.4,

  /** Stefan-Boltzmann constant (W/(m²·K⁴)) */
  STEFAN_BOLTZMANN: 5.67e-8,

  /** Earth radius (m) */
  EARTH_RADIUS: 6371000,

  /** Mach 5 threshold (m/s at sea level) */
  MACH_5_THRESHOLD: 1715,

  /** Standard atmospheric pressure (Pa) */
  STANDARD_PRESSURE: 101325,

  /** Standard atmospheric temperature (K) */
  STANDARD_TEMPERATURE: 288.15,

  /** Standard atmospheric density (kg/m³) */
  STANDARD_DENSITY: 1.225,
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

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-DEF-008 error codes
 */
export enum HypersonicErrorCode {
  INVALID_PARAMETERS = 'D001',
  VELOCITY_TOO_LOW = 'D002',
  ALTITUDE_OUT_OF_RANGE = 'D003',
  THERMAL_LIMIT_EXCEEDED = 'D004',
  STRUCTURAL_FAILURE = 'D005',
  GUIDANCE_FAILURE = 'D006',
  PROPULSION_FAILURE = 'D007',
  SIMULATION_DIVERGED = 'D008',
  MATERIAL_INADEQUATE = 'D009',
  TRAJECTORY_INFEASIBLE = 'D010',
}

/**
 * Hypersonic system error
 */
export class HypersonicError extends Error {
  constructor(
    public code: HypersonicErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'HypersonicError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  // Core types
  Vector3,
  VehicleType,
  PropulsionType,
  MaterialType,

  // Flight parameters
  FlightConditions,
  AerodynamicCoefficients,
  VehicleGeometry,

  // Thermal
  HeatFluxParams,
  HeatFluxResult,
  ThermalProtectionSystem,

  // Propulsion
  ScramjetParams,
  EnginePerformance,

  // Trajectory
  TrajectoryParams,
  TrajectoryResult,
  TrajectoryPoint,
  GuidanceParams,
  GuidanceResult,

  // Detection
  DetectabilityParams,
  DetectabilityResult,
  Countermeasures,

  // Simulation
  SimulationParams,
  SimulationResult,
};

export { HYPERSONIC_CONSTANTS, HypersonicErrorCode, HypersonicError };
