/**
 * WIA-AUTO-019: Hyperloop Transportation - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Mobility Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Physics Types
// ============================================================================

/**
 * Three-dimensional position vector
 */
export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

/**
 * Geographic coordinates
 */
export interface GeoPosition {
  /** Latitude in decimal degrees */
  latitude: number;

  /** Longitude in decimal degrees */
  longitude: number;

  /** Altitude in meters above sea level */
  altitude: number;
}

/**
 * Position along hyperloop route
 */
export interface RoutePosition {
  /** Distance from route origin in meters */
  distanceFromOrigin: number;

  /** Geographic coordinates */
  geo: GeoPosition;

  /** Tube section identifier */
  sectionId?: string;
}

// ============================================================================
// Aerodynamics and Drag
// ============================================================================

/**
 * Parameters for drag force calculation
 */
export interface DragCalculation {
  /** Air pressure in pascals */
  pressure: number;

  /** Pod velocity in m/s */
  velocity: number;

  /** Drag coefficient (dimensionless) */
  dragCoefficient: number;

  /** Cross-sectional area in m² */
  crossSectionArea: number;

  /** Air temperature in Kelvin (optional) */
  temperature?: number;
}

/**
 * Drag force calculation result
 */
export interface DragResult {
  /** Drag force in newtons */
  force: number;

  /** Power required to overcome drag in watts */
  power: number;

  /** Energy consumption per 100 km in kWh */
  energyPer100km: number;

  /** Air density at given conditions in kg/m³ */
  airDensity: number;

  /** Reynolds number (dimensionless) */
  reynoldsNumber: number;
}

/**
 * Pod aerodynamic profile
 */
export interface AerodynamicProfile {
  /** Drag coefficient */
  dragCoefficient: number;

  /** Lift coefficient */
  liftCoefficient: number;

  /** Cross-sectional area in m² */
  crossSectionArea: number;

  /** Frontal area ratio (pod/tube) */
  frontalAreaRatio: number;

  /** Kantrowitz limit speed in m/s */
  kantrowitzLimit: number;
}

// ============================================================================
// Magnetic Levitation
// ============================================================================

/**
 * Levitation system parameters
 */
export interface LevitationParams {
  /** Magnetic field strength in Tesla */
  magneticFieldStrength: number;

  /** Effective pole face area in m² */
  effectiveArea: number;

  /** Pod mass in kg */
  podMass: number;

  /** Target levitation gap in mm */
  levitationGap: number;

  /** Number of electromagnets */
  numberOfMagnets?: number;
}

/**
 * Levitation force calculation result
 */
export interface LevitationResult {
  /** Levitation force in newtons */
  force: number;

  /** Required force (mass × gravity) in newtons */
  requiredForce: number;

  /** Power required in kW */
  powerRequired: number;

  /** Gap stability factor (0-1) */
  gapStability: number;

  /** Is the system stable? */
  isStable: boolean;

  /** Safety margin (force / required force) */
  safetyMargin: number;
}

/**
 * Levitation system status
 */
export interface LevitationStatus {
  /** Gap measurements in mm */
  gaps: {
    frontLeft: number;
    frontRight: number;
    rearLeft: number;
    rearRight: number;
  };

  /** Current power consumption in kW */
  powerConsumption: number;

  /** Magnet currents in amperes */
  magnetCurrents: number[];

  /** System status */
  status: 'stable' | 'adjusting' | 'unstable' | 'offline';

  /** Temperature in Celsius */
  temperature: number;
}

// ============================================================================
// Linear Induction Motor
// ============================================================================

/**
 * Linear motor parameters
 */
export interface LinearMotorParams {
  /** Magnetic field strength in Tesla */
  magneticField: number;

  /** Phase current in amperes */
  current: number;

  /** Active conductor length in meters */
  conductorLength: number;

  /** Number of phases */
  numberOfPhases: number;

  /** Power factor angle in radians */
  powerFactorAngle: number;

  /** Supply frequency in Hz */
  frequency: number;

  /** Pole pitch in meters */
  polePitch: number;
}

/**
 * Linear motor thrust result
 */
export interface ThrustResult {
  /** Thrust force in newtons */
  force: number;

  /** Synchronous speed in m/s */
  synchronousSpeed: number;

  /** Motor power in kW */
  power: number;

  /** Motor efficiency (0-1) */
  efficiency: number;

  /** Slip (dimensionless) */
  slip: number;
}

/**
 * Propulsion system status
 */
export interface PropulsionStatus {
  /** Motor current in amperes */
  motorCurrent: number;

  /** Motor voltage in volts */
  motorVoltage: number;

  /** Power output in kW */
  powerOutput: number;

  /** Motor temperature in Celsius */
  temperature: number;

  /** Operating frequency in Hz */
  frequency: number;

  /** System status */
  status: 'accelerating' | 'cruising' | 'braking' | 'offline';
}

// ============================================================================
// Energy Calculations
// ============================================================================

/**
 * Journey parameters for energy calculation
 */
export interface JourneyParams {
  /** Total distance in meters */
  distance: number;

  /** Pod mass (empty) in kg */
  podMass: number;

  /** Maximum speed in m/s */
  maxSpeed: number;

  /** Number of passengers */
  passengers: number;

  /** Cargo mass in kg */
  cargo?: number;

  /** Net elevation change in meters */
  elevationChange?: number;

  /** Average passenger mass in kg */
  passengerMass?: number;
}

/**
 * Energy consumption breakdown
 */
export interface EnergyResult {
  /** Energy for acceleration in kWh */
  acceleration: number;

  /** Energy for cruising in kWh */
  cruising: number;

  /** Energy from braking in kWh (negative = recovery) */
  braking: number;

  /** Auxiliary systems energy in kWh */
  auxiliary: number;

  /** Levitation system energy in kWh */
  levitation: number;

  /** Total energy consumed in kWh */
  total: number;

  /** Energy per passenger in kWh */
  perPassenger: number;

  /** Efficiency in kWh per 100 km */
  efficiency: number;

  /** Energy recovered percentage (0-1) */
  recoveryRate: number;
}

// ============================================================================
// Pod Design
// ============================================================================

/**
 * Pod design specifications
 */
export interface PodDesign {
  /** Pod identifier */
  id: string;

  /** Pod name/model */
  name: string;

  /** Length in meters */
  length: number;

  /** Diameter in meters */
  diameter: number;

  /** Empty mass in kg */
  mass: number;

  /** Drag coefficient */
  dragCoefficient: number;

  /** Passenger capacity */
  passengerCapacity: number;

  /** Cargo capacity in kg */
  cargoCapacity: number;

  /** Maximum operating speed in m/s */
  maxSpeed: number;

  /** Maximum acceleration in m/s² */
  maxAcceleration: number;

  /** Maximum deceleration in m/s² */
  maxDeceleration: number;

  /** Power consumption profile */
  powerProfile: {
    levitation: number;  // kW
    auxiliary: number;   // kW
    hvac: number;        // kW
    communications: number; // kW
  };
}

/**
 * Pod design validation result
 */
export interface ValidationResult {
  /** Is the design valid? */
  isValid: boolean;

  /** Validation errors (blocking) */
  errors: string[];

  /** Validation warnings (non-blocking) */
  warnings: string[];

  /** Design metrics */
  metrics: {
    /** Tube area ratio (pod area / tube area) */
    tubeAreaRatio: number;

    /** Kantrowitz limit in m/s */
    kantrowitzLimit: number;

    /** Power to weight ratio in W/kg */
    powerToWeight: number;

    /** Mass efficiency (payload / total mass) */
    massEfficiency: number;
  };

  /** Recommendations for improvement */
  recommendations: string[];
}

// ============================================================================
// Tube Infrastructure
// ============================================================================

/**
 * Tube section specifications
 */
export interface TubeSection {
  /** Section identifier */
  id: string;

  /** Section start position in meters from origin */
  startPosition: number;

  /** Section end position in meters from origin */
  endPosition: number;

  /** Inner diameter in meters */
  innerDiameter: number;

  /** Wall thickness in mm */
  wallThickness: number;

  /** Material type */
  material: 'steel' | 'aluminum' | 'composite';

  /** Current pressure in Pa */
  pressure: number;

  /** Temperature in Celsius */
  temperature: number;

  /** Section type */
  type: 'straight' | 'curve' | 'grade' | 'station';

  /** Curve radius in meters (if applicable) */
  curveRadius?: number;

  /** Grade/slope in percentage (if applicable) */
  grade?: number;
}

/**
 * Vacuum system status
 */
export interface VacuumStatus {
  /** Current pressure in Pa */
  pressure: number;

  /** Target pressure in Pa */
  targetPressure: number;

  /** Pressure trend */
  trend: 'rising' | 'falling' | 'stable';

  /** Active pump count */
  activePumps: number;

  /** Total pump capacity in m³/hr */
  totalCapacity: number;

  /** Estimated leak rate in Pa·m³/s */
  leakRate: number;

  /** System status */
  status: 'normal' | 'warning' | 'critical';
}

// ============================================================================
// Journey and Routing
// ============================================================================

/**
 * Hyperloop route information
 */
export interface Route {
  /** Route identifier */
  id: string;

  /** Route name */
  name: string;

  /** Origin station */
  origin: string;

  /** Destination station */
  destination: string;

  /** Total distance in meters */
  distance: number;

  /** Waypoints along route */
  waypoints: RoutePosition[];

  /** Maximum operating speed in m/s */
  maxSpeed: number;

  /** Average grade percentage */
  averageGrade: number;

  /** Number of curves */
  numberOfCurves: number;

  /** Estimated travel time in seconds */
  estimatedTime: number;
}

/**
 * Journey simulation parameters
 */
export interface SimulationParams {
  /** Origin station identifier */
  origin: string;

  /** Destination station identifier */
  destination: string;

  /** Pod type identifier */
  podType: string;

  /** Number of passengers */
  passengers: number;

  /** Cargo mass in kg */
  cargo?: number;

  /** Scheduled departure time */
  departureTime: Date;
}

/**
 * Journey phase
 */
export interface JourneyPhase {
  /** Phase name */
  phase: 'boarding' | 'airlock-evacuation' | 'acceleration' | 'cruising' | 'deceleration' | 'airlock-pressurization' | 'deboarding';

  /** Start time in seconds from journey start */
  startTime: number;

  /** End time in seconds from journey start */
  endTime: number;

  /** Distance traveled during phase in meters */
  distance: number;

  /** Speed at end of phase in m/s */
  speed: number;

  /** Energy consumed during phase in kWh */
  energyConsumed: number;
}

/**
 * Complete journey simulation result
 */
export interface SimulationResult {
  /** Simulation identifier */
  id: string;

  /** Route information */
  route: Route;

  /** Pod used */
  pod: PodDesign;

  /** Total duration in seconds */
  duration: number;

  /** Total distance in meters */
  distance: number;

  /** Maximum speed achieved in m/s */
  maxSpeed: number;

  /** Average speed in m/s */
  averageSpeed: number;

  /** Energy consumption breakdown */
  energyConsumption: EnergyResult;

  /** Journey timeline broken into phases */
  timeline: JourneyPhase[];

  /** Environmental impact */
  environmental: {
    /** CO₂ savings vs airplane in kg */
    co2Savings: number;

    /** Energy savings vs car in kWh */
    energySavings: number;

    /** Comparison to other modes */
    comparison: {
      mode: string;
      timeDifference: number;  // seconds
      energyDifference: number; // kWh
      co2Difference: number;    // kg
    }[];
  };

  /** Success status */
  success: boolean;

  /** Error message if failed */
  error?: string;
}

// ============================================================================
// Pod Telemetry
// ============================================================================

/**
 * Real-time pod telemetry data
 */
export interface PodTelemetry {
  /** Pod identifier */
  podId: string;

  /** Timestamp */
  timestamp: Date;

  /** Current position */
  position: RoutePosition;

  /** Velocity information */
  velocity: {
    /** Speed in m/s */
    speed: number;

    /** Acceleration in m/s² */
    acceleration: number;

    /** Direction */
    direction: 'northbound' | 'southbound' | 'eastbound' | 'westbound';
  };

  /** System statuses */
  systems: {
    /** Levitation system */
    levitation: LevitationStatus;

    /** Propulsion system */
    propulsion: PropulsionStatus;

    /** Pressure information */
    pressure: {
      /** Internal cabin pressure in Pa */
      internal: number;

      /** External tube pressure in Pa */
      external: number;

      /** Pressure differential in Pa */
      differential: number;
    };

    /** Life support */
    lifeSupport: {
      /** Oxygen level in percentage */
      oxygenLevel: number;

      /** CO₂ level in percentage */
      co2Level: number;

      /** Temperature in Celsius */
      temperature: number;

      /** Humidity in percentage */
      humidity: number;
    };
  };

  /** Current passenger count */
  passengers: number;

  /** Current cargo mass in kg */
  cargo: number;

  /** Overall pod status */
  status: 'boarding' | 'departing' | 'accelerating' | 'cruising' | 'braking' | 'arriving' | 'emergency';
}

// ============================================================================
// Station Infrastructure
// ============================================================================

/**
 * Station information
 */
export interface Station {
  /** Station identifier */
  id: string;

  /** Station name */
  name: string;

  /** Geographic location */
  location: GeoPosition;

  /** Number of platforms */
  platforms: number;

  /** Platform details */
  platformDetails: Platform[];

  /** Current tube pressure in Pa */
  tubePressure: number;

  /** Station capacity */
  capacity: {
    /** Current passenger count */
    current: number;

    /** Maximum capacity */
    maximum: number;
  };

  /** System status */
  systemStatus: 'operational' | 'degraded' | 'offline';
}

/**
 * Platform information
 */
export interface Platform {
  /** Platform identifier */
  id: string;

  /** Direction of travel */
  direction: 'northbound' | 'southbound' | 'eastbound' | 'westbound';

  /** Platform status */
  status: 'ready' | 'occupied' | 'evacuating' | 'pressurizing' | 'maintenance';

  /** Current pod (if occupied) */
  podId: string | null;

  /** Airlock pressure in Pa */
  airlockPressure: number;

  /** Next scheduled departure */
  nextDeparture: Date | null;

  /** Next scheduled arrival */
  nextArrival: Date | null;
}

// ============================================================================
// Emergency Systems
// ============================================================================

/**
 * Emergency scenario types
 */
export type EmergencyType =
  | 'power-loss'
  | 'pressure-loss'
  | 'fire'
  | 'collision-risk'
  | 'medical'
  | 'mechanical-failure'
  | 'environmental';

/**
 * Emergency event
 */
export interface EmergencyEvent {
  /** Event identifier */
  id: string;

  /** Emergency type */
  type: EmergencyType;

  /** Severity level */
  severity: 'low' | 'medium' | 'high' | 'critical';

  /** Pod identifier */
  podId: string;

  /** Position where emergency occurred */
  position: RoutePosition;

  /** Timestamp */
  timestamp: Date;

  /** Description */
  description: string;

  /** Current status */
  status: 'active' | 'responding' | 'resolved' | 'escalated';

  /** Actions taken */
  actions: string[];

  /** Estimated resolution time */
  estimatedResolution?: Date;
}

/**
 * Emergency response protocol
 */
export interface EmergencyProtocol {
  /** Protocol identifier */
  id: string;

  /** Emergency type this protocol handles */
  emergencyType: EmergencyType;

  /** Response steps */
  steps: {
    /** Step number */
    order: number;

    /** Action description */
    action: string;

    /** Time limit in seconds */
    timeLimit: number;

    /** Responsible party */
    responsible: 'automated' | 'crew' | 'control-center' | 'emergency-services';
  }[];

  /** Required equipment */
  equipment: string[];

  /** Success criteria */
  successCriteria: string[];
}

// ============================================================================
// Safety Checks
// ============================================================================

/**
 * Safety check result
 */
export interface SafetyCheck {
  /** Check name */
  name: string;

  /** Check category */
  category: 'critical' | 'important' | 'routine';

  /** Check status */
  status: 'pass' | 'fail' | 'warning' | 'info';

  /** Description */
  description: string;

  /** Measured value */
  value?: number;

  /** Expected value */
  expected?: number;

  /** Acceptable range */
  range?: {
    min: number;
    max: number;
  };

  /** Corrective action if failed */
  correctiveAction?: string;

  /** Timestamp */
  timestamp: Date;
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Physical constants used in hyperloop calculations
 */
export const PHYSICS_CONSTANTS = {
  /** Atmospheric pressure at sea level in Pa */
  ATMOSPHERIC_PRESSURE: 101325,

  /** Air density at STP in kg/m³ */
  AIR_DENSITY_STP: 1.225,

  /** Gas constant in J/mol·K */
  GAS_CONSTANT: 8.314,

  /** Molar mass of air in kg/mol */
  MOLAR_MASS_AIR: 0.029,

  /** Magnetic permeability of free space in H/m */
  MAGNETIC_PERMEABILITY: 4 * Math.PI * 1e-7,

  /** Gravitational acceleration in m/s² */
  GRAVITY: 9.81,

  /** Speed of sound in air at 20°C in m/s */
  SPEED_OF_SOUND: 340,

  /** Standard temperature in Kelvin (20°C) */
  STANDARD_TEMPERATURE: 293,

  /** Target operating pressure in Pa */
  TARGET_PRESSURE: 100,

  /** Maximum operating speed in m/s (1200 km/h) */
  MAX_SPEED: 333.33,

  /** Typical cruising speed in m/s (1000 km/h) */
  CRUISING_SPEED: 277.78,
} as const;

/**
 * Hyperloop design constants
 */
export const DESIGN_CONSTANTS = {
  /** Standard tube inner diameter in meters */
  TUBE_DIAMETER: 3.3,

  /** Standard pod diameter in meters */
  POD_DIAMETER: 2.7,

  /** Standard pod length in meters */
  POD_LENGTH: 20,

  /** Typical drag coefficient */
  DRAG_COEFFICIENT: 0.15,

  /** Maximum acceleration in m/s² (0.5g) */
  MAX_ACCELERATION: 4.9,

  /** Maximum deceleration in m/s² (1.5g) */
  MAX_DECELERATION: 14.7,

  /** Comfortable lateral acceleration in m/s² (0.15g) */
  LATERAL_ACCELERATION: 1.47,

  /** Standard levitation gap in mm */
  LEVITATION_GAP: 10,

  /** Minimum curve radius in meters */
  MIN_CURVE_RADIUS: 60000,

  /** Emergency airlock spacing in meters */
  EMERGENCY_AIRLOCK_SPACING: 500,

  /** Vacuum pump spacing in meters */
  PUMP_SPACING: 7500,
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
 * WIA-AUTO-019 error codes
 */
export enum HyperloopErrorCode {
  INVALID_PARAMETERS = 'H001',
  INSUFFICIENT_POWER = 'H002',
  PRESSURE_VIOLATION = 'H003',
  LEVITATION_UNSTABLE = 'H004',
  SPEED_LIMIT_EXCEEDED = 'H005',
  TUBE_BLOCKAGE = 'H006',
  EMERGENCY_STOP = 'H007',
  SYSTEM_OFFLINE = 'H008',
  VALIDATION_FAILED = 'H009',
  COMMUNICATION_ERROR = 'H010',
}

/**
 * Hyperloop error
 */
export class HyperloopError extends Error {
  constructor(
    public code: HyperloopErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'HyperloopError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  // Core types
  Vector3,
  GeoPosition,
  RoutePosition,

  // Aerodynamics
  DragCalculation,
  DragResult,
  AerodynamicProfile,

  // Levitation
  LevitationParams,
  LevitationResult,
  LevitationStatus,

  // Propulsion
  LinearMotorParams,
  ThrustResult,
  PropulsionStatus,

  // Energy
  JourneyParams,
  EnergyResult,

  // Pod Design
  PodDesign,
  ValidationResult,

  // Infrastructure
  TubeSection,
  VacuumStatus,

  // Journey
  Route,
  SimulationParams,
  JourneyPhase,
  SimulationResult,

  // Telemetry
  PodTelemetry,

  // Station
  Station,
  Platform,

  // Emergency
  EmergencyEvent,
  EmergencyProtocol,

  // Safety
  SafetyCheck,
};

export { PHYSICS_CONSTANTS, DESIGN_CONSTANTS, HyperloopErrorCode, HyperloopError };
