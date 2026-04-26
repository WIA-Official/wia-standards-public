/**
 * WIA-AUTO-020: Maglev Train - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Automotive & Mobility Research Group
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
 * Geographic position
 */
export interface GeographicPosition {
  /** Latitude in degrees */
  latitude: number;

  /** Longitude in degrees */
  longitude: number;

  /** Altitude in meters */
  altitude: number;

  /** Chainage (distance along track) in meters */
  chainage?: number;
}

// ============================================================================
// Levitation Systems
// ============================================================================

/**
 * Suspension system types
 */
export type SuspensionType = 'EMS' | 'EDS';

/**
 * Levitation system parameters
 */
export interface LevitationParams {
  /** Magnetic flux density in Tesla */
  fluxDensity: number;

  /** Pole face area in m² */
  poleArea: number;

  /** Suspension type */
  suspensionType: SuspensionType;

  /** Air gap in meters */
  airGap: number;

  /** Vehicle mass in kg (optional) */
  vehicleMass?: number;

  /** Number of magnets per bogie */
  magnetsPerBogie?: number;

  /** Number of bogies */
  bogies?: number;
}

/**
 * Levitation force calculation result
 */
export interface LevitationResult {
  /** Total levitation force in Newtons */
  force: number;

  /** Force per magnet in Newtons */
  forcePerMagnet: number;

  /** Power required in Watts */
  powerRequired: number;

  /** Stability factor (0-1) */
  stability: number;

  /** Gap control parameters */
  gapControl: {
    proportionalGain: number;
    integralGain: number;
    derivativeGain: number;
    targetGap: number;
    maxDeviation: number;
  };

  /** Operating parameters */
  operating: {
    current: number;           // Amperes
    voltage: number;            // Volts
    temperature: number;        // Celsius
    efficiency: number;         // 0-1
  };
}

/**
 * Gap sensor data
 */
export interface GapSensor {
  /** Sensor ID */
  id: string;

  /** Position along vehicle */
  position: Vector3;

  /** Measured gap in millimeters */
  gap: number;

  /** Target gap in millimeters */
  targetGap: number;

  /** Deviation from target */
  deviation: number;

  /** Sensor status */
  status: 'operational' | 'warning' | 'fault';

  /** Measurement timestamp */
  timestamp: Date;
}

/**
 * Electromagnet status
 */
export interface ElectromagnetStatus {
  /** Magnet ID */
  id: string;

  /** Current in Amperes */
  current: number;

  /** Voltage in Volts */
  voltage: number;

  /** Temperature in Celsius */
  temperature: number;

  /** Magnetic flux density in Tesla */
  fluxDensity: number;

  /** Operating status */
  status: 'active' | 'standby' | 'fault' | 'emergency';

  /** Power consumption in Watts */
  power: number;
}

/**
 * Superconducting magnet configuration (for EDS)
 */
export interface SuperconductingMagnet {
  /** Magnet ID */
  id: string;

  /** Magnet type */
  type: 'NbTi' | 'Nb3Sn' | 'YBCO' | 'other';

  /** Operating temperature in Kelvin */
  operatingTemp: number;

  /** Current density in A/mm² */
  currentDensity: number;

  /** Field strength in Tesla */
  fieldStrength: number;

  /** Cryogenic system */
  cryogenic: {
    coolant: 'liquid_helium' | 'liquid_nitrogen';
    temperature: number;        // Kelvin
    flowRate: number;            // L/min
    pressure: number;            // kPa
  };

  /** Cooling power in Watts */
  coolingPower: number;

  /** Status */
  status: 'normal' | 'warming' | 'quench' | 'fault';
}

// ============================================================================
// Propulsion Systems
// ============================================================================

/**
 * Motor types
 */
export type MotorType = 'LSM' | 'LIM';

/**
 * Propulsion system parameters
 */
export interface PropulsionParams {
  /** Motor type */
  motorType: MotorType;

  /** Maximum speed in km/h */
  maxSpeed: number;

  /** Maximum thrust in Newtons */
  maxThrust: number;

  /** Motor efficiency (0-1) */
  efficiency?: number;

  /** Pole pitch in meters (for LSM) */
  polePitch?: number;

  /** Number of phases */
  phases?: number;

  /** Power rating in Watts */
  powerRating?: number;
}

/**
 * Propulsion system result
 */
export interface PropulsionResult {
  /** Current thrust in Newtons */
  thrust: number;

  /** Power consumption in Watts */
  power: number;

  /** Operating frequency in Hz */
  frequency: number;

  /** Current efficiency (0-1) */
  efficiency: number;

  /** Synchronous speed in m/s */
  synchronousSpeed: number;

  /** Slip (for LIM) */
  slip?: number;

  /** Performance curve */
  speedCurve: Array<{
    speed: number;      // km/h
    thrust: number;     // N
    power: number;      // W
    efficiency: number; // 0-1
  }>;

  /** Electrical parameters */
  electrical: {
    voltage: number;    // V
    current: number;    // A
    powerFactor: number; // 0-1
    frequency: number;   // Hz
  };
}

/**
 * Linear motor configuration
 */
export interface LinearMotorConfig {
  /** Motor ID */
  id: string;

  /** Motor type */
  type: MotorType;

  /** Active length in meters */
  activeLength: number;

  /** Pole pitch in meters */
  polePitch: number;

  /** Number of poles */
  poles: number;

  /** Air gap in meters */
  airGap: number;

  /** Rated power in MW */
  ratedPower: number;

  /** Maximum thrust in kN */
  maxThrust: number;

  /** Control mode */
  controlMode: 'scalar' | 'vector' | 'direct_torque';
}

// ============================================================================
// Guideway Design
// ============================================================================

/**
 * Guideway geometry parameters
 */
export interface GuidewayParams {
  /** Maximum operating speed in km/h */
  maxSpeed: number;

  /** Curve radius in meters (optional) */
  curveRadius?: number;

  /** Gradient as ratio (0.04 = 4%) */
  gradient?: number;

  /** Track gauge in meters */
  trackGauge?: number;

  /** Lateral acceleration limit in g */
  lateralAccelLimit?: number;

  /** Vertical acceleration limit in g */
  verticalAccelLimit?: number;
}

/**
 * Guideway design result
 */
export interface GuidewayResult {
  /** Superelevation (banking) in degrees */
  superelevation: number;

  /** Minimum curve radius in meters */
  minCurveRadius: number;

  /** Minimum vertical curve radius in meters */
  minVerticalRadius: number;

  /** Maximum allowed gradient */
  maxGradient: number;

  /** Structural load in kN/m */
  structuralLoad: number;

  /** Design tolerances */
  tolerances: {
    lateral: number;    // mm
    vertical: number;   // mm
    twist: number;      // degrees/m
    surface: number;    // mm
  };

  /** Cross-section specifications */
  crossSection: {
    type: 'U-shaped' | 'box' | 'T-shaped';
    width: number;      // meters
    depth: number;      // meters
    wallThickness: number; // meters
  };

  /** Material specifications */
  materials: {
    primary: string;
    reinforcement: string;
    surfaceTreatment: string;
    reactionRail?: string;
  };
}

/**
 * Guideway segment
 */
export interface GuidewaySegment {
  /** Segment ID */
  id: string;

  /** Start position (chainage) in meters */
  startChainage: number;

  /** End position (chainage) in meters */
  endChainage: number;

  /** Segment length in meters */
  length: number;

  /** Horizontal alignment */
  horizontal: {
    type: 'straight' | 'curve' | 'transition';
    radius?: number;             // meters
    superelevation?: number;     // degrees
    transitionLength?: number;   // meters
  };

  /** Vertical alignment */
  vertical: {
    type: 'level' | 'grade' | 'curve';
    gradient?: number;           // ratio
    radius?: number;             // meters
  };

  /** Structure type */
  structure: {
    type: 'at-grade' | 'elevated' | 'tunnel' | 'bridge';
    material: string;
    crossSection: string;
    width: number;               // meters
    gauge: number;               // meters
  };

  /** Speed limits */
  limits: {
    maxSpeed: number;            // km/h
    maxLateralAccel: number;     // g
    maxVerticalAccel: number;    // g
  };

  /** Condition assessment */
  condition: {
    inspectionDate: Date;
    status: 'good' | 'fair' | 'poor' | 'critical';
    defects: Array<{
      type: string;
      severity: 'minor' | 'moderate' | 'major' | 'critical';
      location: number;          // chainage
      description: string;
    }>;
  };
}

// ============================================================================
// Power and Energy Systems
// ============================================================================

/**
 * Energy optimization parameters
 */
export interface EnergyOptimizationParams {
  /** Distance in km */
  distance: number;

  /** Average speed in km/h */
  averageSpeed: number;

  /** Train mass in kg */
  trainMass: number;

  /** Gradient profile (array of gradients along route) */
  gradient: number[];

  /** Number of station stops */
  stations: number;

  /** Passenger count */
  passengers?: number;

  /** Target arrival time */
  targetTime?: number;  // seconds
}

/**
 * Energy consumption result
 */
export interface EnergyResult {
  /** Total energy in kWh */
  totalEnergy: number;

  /** Energy per km in kWh/km */
  energyPerKm: number;

  /** Energy per seat-km in kWh/seat-km */
  energyPerSeatKm: number;

  /** Energy per passenger-km in kWh/passenger-km */
  energyPerPassengerKm: number;

  /** Regenerated energy in kWh */
  regeneratedEnergy: number;

  /** Energy recovery rate (0-1) */
  recoveryRate: number;

  /** Energy breakdown */
  breakdown: {
    levitation: number;     // kWh
    propulsion: number;     // kWh
    auxiliary: number;      // kWh
    losses: number;         // kWh
    braking: number;        // kWh (negative = recovered)
  };

  /** CO2 emissions in kg (based on energy source) */
  co2Emissions: number;

  /** Cost estimate in USD */
  costEstimate: number;
}

/**
 * Power supply configuration
 */
export interface PowerSupplyConfig {
  /** Substation ID */
  id: string;

  /** Position (chainage) in meters */
  position: number;

  /** Grid voltage in kV */
  gridVoltage: number;

  /** Traction voltage in kV */
  tractionVoltage: number;

  /** Maximum power output in MW */
  maxPower: number;

  /** Transformer rating in MVA */
  transformerRating: number;

  /** Redundancy */
  redundancy: 'N' | 'N+1' | '2N';

  /** Status */
  status: 'operational' | 'standby' | 'maintenance' | 'fault';
}

/**
 * Regenerative braking configuration
 */
export interface RegenerativeBraking {
  /** Enabled status */
  enabled: boolean;

  /** Maximum recovery power in kW */
  maxRecoveryPower: number;

  /** Efficiency (0-1) */
  efficiency: number;

  /** Braking energy ratio (0-1) */
  brakingEnergyRatio: number;

  /** Grid feedback enabled */
  gridFeedback: boolean;

  /** Energy storage */
  storage?: {
    type: 'supercapacitor' | 'flywheel' | 'battery';
    capacity: number;        // kWh
    power: number;           // kW
  };
}

// ============================================================================
// Control Systems
// ============================================================================

/**
 * Vehicle control mode
 */
export type ControlMode = 'manual' | 'ATP' | 'ATO' | 'autonomous';

/**
 * Vehicle dynamics
 */
export interface VehicleDynamics {
  /** Position */
  position: GeographicPosition;

  /** Velocity in m/s */
  velocity: number;

  /** Acceleration in m/s² */
  acceleration: number;

  /** Jerk in m/s³ */
  jerk: number;

  /** Lateral acceleration in m/s² */
  lateralAcceleration: number;

  /** Vertical acceleration in m/s² */
  verticalAcceleration: number;

  /** Heading in degrees */
  heading: number;
}

/**
 * Control system status
 */
export interface ControlSystemStatus {
  /** Control mode */
  mode: ControlMode;

  /** Levitation control active */
  levitationControl: boolean;

  /** Propulsion control active */
  propulsionControl: boolean;

  /** Guidance control active */
  guidanceControl: boolean;

  /** ATP (Automatic Train Protection) status */
  atp: {
    active: boolean;
    speedLimit: number;       // km/h
    brakingCurve: number[];   // array of speeds
    distanceToTarget: number; // meters
  };

  /** ATO (Automatic Train Operation) status */
  ato: {
    active: boolean;
    targetSpeed: number;      // km/h
    targetPosition: number;   // meters (chainage)
    scheduledArrival: Date;
    deviation: number;        // seconds
  };

  /** Communication status */
  communication: {
    type: 'CBTC' | 'ETCS' | 'proprietary';
    signalStrength: number;   // 0-100
    latency: number;          // milliseconds
    lastUpdate: Date;
  };
}

/**
 * Safety envelope
 */
export interface SafetyEnvelope {
  /** Current position in meters */
  currentPosition: number;

  /** Current speed in km/h */
  currentSpeed: number;

  /** Safe speed at position */
  safeSpeed: number;

  /** Distance to target in meters */
  distanceToTarget: number;

  /** Required braking distance in meters */
  brakingDistance: number;

  /** Safety margin in meters */
  safetyMargin: number;

  /** Is within safe envelope */
  isSafe: boolean;

  /** Warnings */
  warnings: string[];
}

// ============================================================================
// Braking Systems
// ============================================================================

/**
 * Braking system types
 */
export type BrakingType = 'regenerative' | 'eddy_current' | 'aerodynamic' | 'mechanical';

/**
 * Braking parameters
 */
export interface BrakingParams {
  /** Initial speed in km/h */
  initialSpeed: number;

  /** Target speed in km/h (0 for full stop) */
  targetSpeed: number;

  /** Available braking types */
  availableBrakes: BrakingType[];

  /** Emergency braking flag */
  emergency?: boolean;

  /** Train mass in kg */
  trainMass?: number;
}

/**
 * Braking result
 */
export interface BrakingResult {
  /** Total braking force in Newtons */
  totalForce: number;

  /** Deceleration rate in m/s² */
  deceleration: number;

  /** Stopping distance in meters */
  stoppingDistance: number;

  /** Stopping time in seconds */
  stoppingTime: number;

  /** Energy recovered in kWh */
  energyRecovered: number;

  /** Brake force breakdown */
  forces: {
    regenerative: number;     // N
    eddyCurrent: number;      // N
    aerodynamic: number;      // N
    mechanical: number;       // N
  };
}

// ============================================================================
// Vehicle Configuration
// ============================================================================

/**
 * Maglev train configuration
 */
export interface MaglevTrain {
  /** Vehicle ID */
  id: string;

  /** Train name */
  name: string;

  /** Vehicle type */
  type: string;

  /** Suspension system */
  suspensionType: SuspensionType;

  /** Propulsion system */
  propulsionType: MotorType;

  /** Number of cars */
  cars: number;

  /** Passenger capacity */
  passengerCapacity: number;

  /** Current passenger count */
  currentPassengers: number;

  /** Total mass in kg */
  mass: {
    empty: number;
    loaded: number;
    current: number;
  };

  /** Dimensions */
  dimensions: {
    length: number;      // meters
    width: number;       // meters
    height: number;      // meters
  };

  /** Performance specifications */
  performance: {
    maxSpeed: number;          // km/h
    maxAcceleration: number;   // m/s²
    maxDeceleration: number;   // m/s²
    range: number;             // km
  };

  /** Systems */
  systems: {
    levitation: LevitationParams;
    propulsion: PropulsionParams;
    powerSupply: PowerSupplyConfig;
    control: ControlSystemStatus;
  };
}

/**
 * Vehicle status
 */
export interface VehicleStatus {
  /** Vehicle reference */
  vehicle: MaglevTrain;

  /** Current dynamics */
  dynamics: VehicleDynamics;

  /** Levitation status */
  levitation: {
    active: boolean;
    gaps: GapSensor[];
    magnets: ElectromagnetStatus[];
    stability: number;         // 0-1
    power: number;             // kW
  };

  /** Propulsion status */
  propulsion: {
    active: boolean;
    thrust: number;            // N
    power: number;             // kW
    frequency: number;         // Hz
    efficiency: number;        // 0-1
  };

  /** Energy status */
  energy: {
    totalConsumption: number;  // kW
    levitation: number;        // kW
    propulsion: number;        // kW
    auxiliary: number;         // kW
    regeneration: number;      // kW (negative)
  };

  /** Operational status */
  operational: {
    mode: ControlMode;
    doorsOpen: boolean;
    emergencyBrake: boolean;
    faults: string[];
    warnings: string[];
  };

  /** Timestamp */
  timestamp: Date;
}

// ============================================================================
// Route and Journey
// ============================================================================

/**
 * Station
 */
export interface Station {
  /** Station ID */
  id: string;

  /** Station name */
  name: string;

  /** Position (chainage) in meters */
  position: number;

  /** Geographic coordinates */
  coordinates: GeographicPosition;

  /** Platform length in meters */
  platformLength: number;

  /** Passenger facilities */
  facilities: string[];

  /** Operating status */
  status: 'operational' | 'maintenance' | 'closed';
}

/**
 * Route
 */
export interface Route {
  /** Route ID */
  id: string;

  /** Route name */
  name: string;

  /** Total distance in km */
  distance: number;

  /** Stations */
  stations: Station[];

  /** Guideway segments */
  segments: GuidewaySegment[];

  /** Maximum speed in km/h */
  maxSpeed: number;

  /** Average speed in km/h */
  averageSpeed: number;

  /** Scheduled journey time in seconds */
  scheduledTime: number;

  /** Operating hours */
  operatingHours: {
    start: string;    // HH:MM
    end: string;      // HH:MM
  };
}

/**
 * Journey plan
 */
export interface JourneyPlan {
  /** Journey ID */
  id: string;

  /** Route */
  route: Route;

  /** Vehicle */
  vehicle: MaglevTrain;

  /** Departure station */
  origin: Station;

  /** Arrival station */
  destination: Station;

  /** Scheduled departure */
  scheduledDeparture: Date;

  /** Scheduled arrival */
  scheduledArrival: Date;

  /** Station stops */
  stops: Array<{
    station: Station;
    arrivalTime: Date;
    departureTime: Date;
    dwellTime: number;         // seconds
  }>;

  /** Speed profile */
  speedProfile: Array<{
    position: number;          // meters
    speed: number;             // km/h
    time: number;              // seconds
  }>;

  /** Energy estimate */
  energyEstimate: EnergyResult;
}

// ============================================================================
// Safety and Validation
// ============================================================================

/**
 * Safety check parameters
 */
export interface SafetyParams {
  /** Current speed in km/h */
  speed: number;

  /** Braking rate in m/s² */
  brakingRate: number;

  /** Reaction time in seconds */
  reactionTime: number;

  /** Safety margin in meters */
  safetyMargin: number;

  /** Gap status (array of gaps in mm) */
  gapStatus: number[];

  /** System health (0-1) */
  systemHealth: number;
}

/**
 * Safety validation result
 */
export interface SafetyResult {
  /** Overall safety status */
  isSafe: boolean;

  /** Stopping distance in meters */
  stoppingDistance: number;

  /** Gap stability check */
  gapStability: boolean;

  /** System health check */
  systemHealth: number;  // 0-1

  /** Warnings (non-critical) */
  warnings: string[];

  /** Errors (critical) */
  errors: string[];

  /** Emergency procedure (if needed) */
  emergencyProcedure?: string;

  /** Recommended actions */
  recommendations: string[];
}

// ============================================================================
// Simulation and Analysis
// ============================================================================

/**
 * Simulation parameters
 */
export interface SimulationParams {
  /** Route to simulate */
  route: Route;

  /** Vehicle configuration */
  vehicle: MaglevTrain;

  /** Number of passengers */
  passengers: number;

  /** Time step in seconds */
  timeStep?: number;

  /** Include disturbances */
  disturbances?: {
    wind: boolean;
    gradient: boolean;
    curves: boolean;
  };
}

/**
 * Simulation result
 */
export interface SimulationResult {
  /** Simulation ID */
  id: string;

  /** Input parameters */
  parameters: SimulationParams;

  /** Journey time in seconds */
  journeyTime: number;

  /** Maximum speed achieved in km/h */
  maxSpeed: number;

  /** Average speed in km/h */
  averageSpeed: number;

  /** Total energy consumption */
  energy: EnergyResult;

  /** Timeline data */
  timeline: Array<{
    time: number;              // seconds
    position: number;          // meters
    speed: number;             // km/h
    acceleration: number;      // m/s²
    power: number;             // kW
    gap: number;               // mm
  }>;

  /** Safety analysis */
  safety: {
    violations: number;
    maxAcceleration: number;   // m/s²
    maxDeceleration: number;   // m/s²
    gapStability: number;      // 0-1
  };

  /** Performance metrics */
  performance: {
    punctuality: number;       // seconds deviation
    efficiency: number;        // 0-1
    comfort: number;           // 0-1
    reliability: number;       // 0-1
  };

  /** Success status */
  success: boolean;

  /** Error message (if failed) */
  error?: string;
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Physical constants for maglev calculations
 */
export const MAGLEV_CONSTANTS = {
  /** Permeability of free space in H/m */
  PERMEABILITY_FREE_SPACE: 4 * Math.PI * 1e-7,

  /** Gravitational acceleration in m/s² */
  GRAVITATIONAL_ACCELERATION: 9.81,

  /** Air density at sea level in kg/m³ */
  AIR_DENSITY: 1.225,

  /** Speed of light in m/s (for relativistic corrections) */
  SPEED_OF_LIGHT: 299792458,

  /** Standard atmospheric pressure in kPa */
  ATMOSPHERIC_PRESSURE: 101.325,

  /** Default values */
  DEFAULT: {
    /** Default air gap for EMS in meters */
    EMS_AIR_GAP: 0.010,

    /** Default air gap for EDS in meters */
    EDS_AIR_GAP: 0.100,

    /** Default track gauge in meters */
    TRACK_GAUGE: 2.8,

    /** Default passenger mass in kg */
    PASSENGER_MASS: 75,

    /** Comfort lateral acceleration limit in g */
    COMFORT_LATERAL_ACCEL: 0.08,

    /** Comfort vertical acceleration limit in g */
    COMFORT_VERTICAL_ACCEL: 0.05,

    /** Safety margin in meters */
    SAFETY_MARGIN: 500,

    /** Reaction time in seconds */
    REACTION_TIME: 2.0,
  },
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
 * WIA-AUTO-020 error codes
 */
export enum MaglevErrorCode {
  LEVITATION_FAILURE = 'M001',
  GAP_OUT_OF_RANGE = 'M002',
  PROPULSION_FAULT = 'M003',
  POWER_FAILURE = 'M004',
  CONTROL_SYSTEM_ERROR = 'M005',
  SPEED_LIMIT_EXCEEDED = 'M006',
  GUIDEWAY_DEFECT = 'M007',
  COMMUNICATION_LOST = 'M008',
  EMERGENCY_BRAKE_ACTIVATED = 'M009',
  SAFETY_VIOLATION = 'M010',
  INVALID_PARAMETERS = 'M011',
  SYSTEM_OVERLOAD = 'M012',
}

/**
 * Maglev system error
 */
export class MaglevError extends Error {
  constructor(
    public code: MaglevErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'MaglevError';
  }
}

// ============================================================================
// Export All Types
// ============================================================================

export type {
  // Core
  Vector3,
  GeographicPosition,

  // Levitation
  LevitationParams,
  LevitationResult,
  GapSensor,
  ElectromagnetStatus,
  SuperconductingMagnet,

  // Propulsion
  PropulsionParams,
  PropulsionResult,
  LinearMotorConfig,

  // Guideway
  GuidewayParams,
  GuidewayResult,
  GuidewaySegment,

  // Energy
  EnergyOptimizationParams,
  EnergyResult,
  PowerSupplyConfig,
  RegenerativeBraking,

  // Control
  VehicleDynamics,
  ControlSystemStatus,
  SafetyEnvelope,

  // Braking
  BrakingParams,
  BrakingResult,

  // Vehicle
  MaglevTrain,
  VehicleStatus,

  // Route
  Station,
  Route,
  JourneyPlan,

  // Safety
  SafetyParams,
  SafetyResult,

  // Simulation
  SimulationParams,
  SimulationResult,
};

export { MAGLEV_CONSTANTS, MaglevErrorCode, MaglevError };
