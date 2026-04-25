/**
 * WIA-AUTO-022: Vehicle Safety - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Automotive Safety Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * Three-dimensional vector for spatial coordinates and forces
 */
export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

/**
 * Vehicle position and orientation
 */
export interface VehiclePose {
  /** Position in meters */
  position: Vector3;

  /** Heading angle in degrees (0 = north) */
  heading: number;

  /** Velocity in m/s */
  velocity: Vector3;

  /** Timestamp */
  timestamp: Date | string;
}

// ============================================================================
// Crash Assessment
// ============================================================================

/**
 * Crash assessment input parameters
 */
export interface CrashAssessmentRequest {
  /** Vehicle mass in kilograms */
  vehicleMass: number;

  /** Impact velocity in m/s */
  impactVelocity: number;

  /** Impact angle in degrees (0=frontal, 90=side, 180=rear) */
  impactAngle: number;

  /** Crumple zone length in meters */
  crumpleZoneLength: number;

  /** Occupant information */
  occupants: OccupantInfo[];

  /** Road surface condition */
  roadCondition?: 'dry' | 'wet' | 'snow' | 'ice';

  /** Barrier type for testing */
  barrierType?: 'rigid' | 'deformable' | 'pole' | 'vehicle';
}

/**
 * Occupant information for crash assessment
 */
export interface OccupantInfo {
  /** Seating position */
  position: 'driver' | 'passenger' | 'rear_left' | 'rear_right' | 'rear_center';

  /** Occupant mass in kg */
  mass: number;

  /** Occupant height in meters */
  height: number;

  /** Occupant age in years */
  age: number;

  /** Seatbelt fastened */
  beltStatus: boolean;

  /** Airbag enabled for this position */
  airbagEnabled: boolean;

  /** Occupant classification */
  classification?: 'infant' | 'child' | 'small_adult' | 'adult' | 'large_adult';
}

/**
 * Crash assessment response with injury predictions
 */
export interface CrashAssessmentResponse {
  /** Impact force in Newtons */
  impactForce: number;

  /** Average deceleration in g's */
  deceleration: number;

  /** Impact duration in seconds */
  impactDuration: number;

  /** Total energy absorbed in joules */
  energyAbsorbed: number;

  /** Injury assessment by occupant position */
  occupantInjury: {
    [position: string]: InjuryAssessment;
  };

  /** Structural integrity results */
  structuralIntegrity: StructuralIntegrity;

  /** Overall safety rating (0-5 stars) */
  safetyRating: number;

  /** Detailed scoring */
  detailedScoring?: NCAPScoring;
}

/**
 * Injury assessment metrics
 */
export interface InjuryAssessment {
  /** Head Injury Criterion (15ms window) */
  hic15: number;

  /** Head Injury Criterion (36ms window) */
  hic36?: number;

  /** Chest deflection in mm */
  chestDeflection: number;

  /** Chest acceleration (3ms clip) in g's */
  chestAcceleration?: number;

  /** Viscous criterion (m/s) */
  viscousCriterion?: number;

  /** Femur load in kN */
  femurLoad: number;

  /** Tibia index */
  tibiaIndex: number;

  /** Neck tension in kN */
  neckTension?: number;

  /** Neck compression in kN */
  neckCompression?: number;

  /** Overall injury risk */
  injuryRisk: 'low' | 'moderate' | 'high' | 'severe';

  /** Abbreviated Injury Scale (AIS) prediction */
  ais: number;

  /** Probability of specific injury levels */
  injuryProbability?: {
    ais1: number;  // Minor
    ais2: number;  // Moderate
    ais3: number;  // Serious
    ais4: number;  // Severe
    ais5: number;  // Critical
    ais6: number;  // Maximum (fatal)
  };
}

/**
 * Structural integrity assessment
 */
export interface StructuralIntegrity {
  /** Cabin intrusion in mm */
  cabinIntrusion: number;

  /** Maximum A-pillar intrusion in mm */
  aPillarIntrusion: number;

  /** Maximum B-pillar intrusion in mm */
  bPillarIntrusion: number;

  /** Door openability after crash */
  doorOpenability: boolean;

  /** Pedal displacement in mm */
  pedalDisplacement: number;

  /** Steering column intrusion in mm */
  steeringIntrusion: number;

  /** Roof strength (for rollover) */
  roofStrength?: number;

  /** Overall integrity score (0-1) */
  integrityScore: number;
}

// ============================================================================
// Active Safety Systems
// ============================================================================

/**
 * Collision warning request
 */
export interface CollisionWarningRequest {
  /** Own vehicle state */
  ownVehicle: VehicleState;

  /** Detected obstacles */
  obstacles: Obstacle[];

  /** Road condition */
  roadCondition: 'dry' | 'wet' | 'snow' | 'ice';

  /** Driver awareness (0-1) */
  driverAttention?: number;

  /** Active safety features enabled */
  systemsEnabled?: {
    aeb: boolean;
    esc: boolean;
    ldw: boolean;
    bsm: boolean;
  };
}

/**
 * Vehicle state information
 */
export interface VehicleState {
  /** Velocity in m/s */
  velocity: number;

  /** Position coordinates */
  position: { x: number; y: number };

  /** Heading in degrees */
  heading: number;

  /** Acceleration in m/s² */
  acceleration?: number;

  /** Steering angle in degrees */
  steeringAngle?: number;

  /** Brake pressure (0-1) */
  brakePressure?: number;

  /** Throttle position (0-1) */
  throttlePosition?: number;
}

/**
 * Detected obstacle information
 */
export interface Obstacle {
  /** Unique obstacle identifier */
  id: string;

  /** Obstacle type */
  type: 'vehicle' | 'pedestrian' | 'cyclist' | 'motorcycle' | 'object' | 'animal';

  /** Position relative to ego vehicle */
  position: { x: number; y: number };

  /** Velocity vector */
  velocity: { vx: number; vy: number };

  /** Object dimensions */
  dimensions: { length: number; width: number; height?: number };

  /** Detection confidence (0-1) */
  confidence: number;

  /** Tracking quality */
  trackingQuality?: 'excellent' | 'good' | 'fair' | 'poor';

  /** Lane position */
  lanePosition?: 'same' | 'adjacent_left' | 'adjacent_right' | 'oncoming' | 'crossing';
}

/**
 * Collision warning response
 */
export interface CollisionWarningResponse {
  /** Identified threats */
  threats: CollisionThreat[];

  /** Overall warning level */
  warningLevel: 0 | 1 | 2 | 3;

  /** Intervention required */
  interventionRequired: boolean;

  /** Recommended brake force (0-1) */
  recommendedBrakeForce?: number;

  /** Recommended steering correction in degrees */
  recommendedSteering?: number;

  /** Time until intervention (seconds) */
  timeToIntervention?: number;
}

/**
 * Collision threat assessment
 */
export interface CollisionThreat {
  /** Obstacle ID */
  obstacleId: string;

  /** Time to collision in seconds */
  ttc: number;

  /** Threat severity */
  severity: 'low' | 'medium' | 'high' | 'critical';

  /** Collision probability (0-1) */
  collisionProbability: number;

  /** Predicted impact point */
  impactPoint?: { x: number; y: number };

  /** Predicted impact velocity */
  impactVelocity?: number;

  /** Recommended action */
  recommendedAction: 'none' | 'warning' | 'brake' | 'emergency_brake' | 'evade';

  /** Evasion path if applicable */
  evasionPath?: Array<{ x: number; y: number }>;

  /** Can be avoided by braking alone */
  brakingAvoidable: boolean;

  /** Can be avoided by steering alone */
  steeringAvoidable: boolean;
}

// ============================================================================
// Airbag Systems
// ============================================================================

/**
 * Airbag deployment decision request
 */
export interface AirbagDeploymentRequest {
  /** Crash severity (0-10 scale) */
  crashSeverity: number;

  /** Crash type */
  crashType: 'frontal' | 'side' | 'rear' | 'rollover' | 'multiple';

  /** Impact velocity in m/s */
  impactVelocity: number;

  /** Impact angle in degrees */
  impactAngle?: number;

  /** Occupant presence by position */
  occupantPresence: boolean[];

  /** Occupant classification */
  occupantClassification: Array<'empty' | 'infant' | 'child' | 'adult'>;

  /** Belt status by position */
  beltStatus: boolean[];

  /** Crash pulse (acceleration time series in g) */
  crashPulse: number[];

  /** Time since crash detected (milliseconds) */
  timeFromCrash?: number;
}

/**
 * Airbag deployment response
 */
export interface AirbagDeploymentResponse {
  /** Deployment decisions for all airbags */
  shouldDeploy: {
    driverFrontal: AirbagDeployment;
    passengerFrontal: AirbagDeployment;
    driverSide: AirbagDeployment;
    passengerSide: AirbagDeployment;
    driverKnee?: AirbagDeployment;
    passengerKnee?: AirbagDeployment;
    curtainLeft: AirbagDeployment;
    curtainRight: AirbagDeployment;
    rearLeft?: AirbagDeployment;
    rearRight?: AirbagDeployment;
  };

  /** Pretensioner activation */
  pretensioners: {
    driver: PretensionerDeployment;
    passenger: PretensionerDeployment;
    rearLeft?: PretensionerDeployment;
    rearRight?: PretensionerDeployment;
    rearCenter?: PretensionerDeployment;
  };

  /** Reasoning for decisions */
  reasoning: string[];

  /** Total deployment time in milliseconds */
  totalDeploymentTime: number;
}

/**
 * Individual airbag deployment decision
 */
export interface AirbagDeployment {
  /** Should deploy */
  deploy: boolean;

  /** Deployment stage (0=off, 1=low, 2=high) */
  stage: 0 | 1 | 2;

  /** Deployment timing in milliseconds from crash */
  timing: number;

  /** Inflation pressure (kPa) */
  pressure?: number;

  /** Suppression reason if not deploying */
  suppressionReason?: string;
}

/**
 * Pretensioner deployment decision
 */
export interface PretensionerDeployment {
  /** Should activate */
  activate: boolean;

  /** Activation timing in milliseconds */
  timing: number;

  /** Retraction force in kN */
  force: number;

  /** Retraction distance in mm */
  retraction?: number;
}

// ============================================================================
// Child Safety
// ============================================================================

/**
 * Child restraint system information
 */
export interface ChildRestraintSystem {
  /** CRS type */
  type: 'infant_carrier' | 'convertible' | 'forward_facing' | 'booster' | 'integrated';

  /** Attachment method */
  attachment: 'isofix' | 'latch' | 'seatbelt' | 'integrated';

  /** Orientation */
  orientation: 'rear_facing' | 'forward_facing';

  /** Age group */
  ageGroup: 'group_0' | 'group_0+' | 'group_I' | 'group_II' | 'group_III';

  /** Weight range in kg */
  weightRange: { min: number; max: number };

  /** ISOFIX classification */
  isofixClass?: 'universal' | 'semi_universal' | 'specific';

  /** Has top tether */
  hasTopTether?: boolean;

  /** Has support leg */
  hasSupportLeg?: boolean;
}

/**
 * Child safety assessment request
 */
export interface ChildSafetyRequest {
  /** Child age in years */
  age: number;

  /** Child weight in kg */
  weight: number;

  /** Child height in cm */
  height: number;

  /** Restraint system used */
  restraintSystem: ChildRestraintSystem;

  /** Installation quality */
  installationQuality?: 'excellent' | 'good' | 'fair' | 'poor';

  /** Seating position */
  seatingPosition: 'front_passenger' | 'rear_left' | 'rear_right' | 'rear_center';
}

/**
 * Child safety assessment response
 */
export interface ChildSafetyResponse {
  /** Is restraint appropriate for child */
  appropriateRestraint: boolean;

  /** Is installation correct */
  correctInstallation: boolean;

  /** Safety rating (0-5) */
  safetyRating: number;

  /** Recommendations */
  recommendations: string[];

  /** Warnings */
  warnings: string[];

  /** Predicted injury metrics in crash */
  crashProtection?: {
    hic: number;
    chestAcceleration: number;
    neckTension: number;
    overallRisk: 'low' | 'moderate' | 'high';
  };
}

// ============================================================================
// Safety Testing
// ============================================================================

/**
 * NCAP crash test configuration
 */
export interface NCAPTestConfig {
  /** Test protocol */
  protocol: 'Euro_NCAP' | 'US_NCAP' | 'IIHS' | 'JNCAP' | 'ANCAP' | 'Latin_NCAP';

  /** Test year/version */
  version: string;

  /** Test type */
  testType: 'frontal_offset' | 'frontal_full' | 'side_barrier' | 'side_pole' | 'rollover' | 'pedestrian';

  /** Impact speed in km/h */
  impactSpeed: number;

  /** Test dummies used */
  dummies: DummyInfo[];

  /** Environmental conditions */
  conditions?: {
    temperature: number;
    humidity: number;
  };
}

/**
 * Test dummy information
 */
export interface DummyInfo {
  /** Dummy type */
  type: 'Hybrid_III' | 'WorldSID' | 'EuroSID' | 'SID_IIs' | 'BioRID' | 'Q_series';

  /** Dummy size */
  size: '5th_female' | '50th_male' | '95th_male' | 'child_6yo' | 'child_3yo' | 'child_18mo';

  /** Position in vehicle */
  position: string;

  /** Instrumentation channels */
  sensors?: string[];
}

/**
 * NCAP scoring breakdown
 */
export interface NCAPScoring {
  /** Overall star rating */
  overallRating: 0 | 1 | 2 | 3 | 4 | 5;

  /** Adult occupant protection */
  adultOccupant: {
    percentage: number;
    points: number;
    maxPoints: number;
    frontalImpact: number;
    sideImpact: number;
    poleImpact?: number;
    whiplash?: number;
  };

  /** Child occupant protection */
  childOccupant: {
    percentage: number;
    points: number;
    maxPoints: number;
    dynamic: number;
    vehicleAssessment: number;
    childPresenceDetection?: number;
  };

  /** Vulnerable road users */
  vulnerableRoadUsers: {
    percentage: number;
    points: number;
    maxPoints: number;
    pedestrian: number;
    cyclist?: number;
    aebPedestrian?: number;
  };

  /** Safety assist systems */
  safetyAssist: {
    percentage: number;
    points: number;
    maxPoints: number;
    aebCar: number;
    laneSupport: number;
    speedAssistance: number;
    driverMonitoring?: number;
  };
}

// ============================================================================
// Pedestrian Safety
// ============================================================================

/**
 * Pedestrian impact assessment
 */
export interface PedestrianImpactRequest {
  /** Vehicle speed in km/h */
  vehicleSpeed: number;

  /** Pedestrian type */
  pedestrianType: 'adult' | 'child';

  /** Impact location on vehicle */
  impactLocation: {
    x: number;  // Longitudinal (m from front)
    y: number;  // Lateral (m from centerline)
  };

  /** Hood/bonnet properties */
  hoodProperties: {
    stiffness: number;      // N/m
    clearance: number;      // mm to engine
    material: string;
  };

  /** Impact angle */
  impactAngle?: number;
}

/**
 * Pedestrian impact response
 */
export interface PedestrianImpactResponse {
  /** Head impact severity */
  headImpact: {
    hic: number;
    impactVelocity: number;
    rating: 'good' | 'adequate' | 'marginal' | 'poor';
  };

  /** Upper leg impact */
  upperLegImpact: {
    bendingMoment: number;  // Nm
    force: number;           // kN
    rating: 'good' | 'adequate' | 'marginal' | 'poor';
  };

  /** Lower leg impact */
  lowerLegImpact: {
    tibiaBending: number;    // Nm
    tibiaShear: number;      // kN
    mclElongation: number;   // mm
    rating: 'good' | 'adequate' | 'marginal' | 'poor';
  };

  /** Overall pedestrian protection score */
  overallScore: number;

  /** Injury risk */
  injuryRisk: 'low' | 'moderate' | 'high' | 'severe';
}

// ============================================================================
// Event Data Recording
// ============================================================================

/**
 * Event Data Recorder (EDR) snapshot
 */
export interface EDRSnapshot {
  /** Timestamp of event */
  timestamp: Date | string;

  /** Event type */
  eventType: 'crash' | 'near_miss' | 'hard_brake' | 'system_fault';

  /** Pre-crash data (5 seconds before) */
  preCrash: PreCrashData;

  /** Crash event data */
  crashEvent?: CrashEventData;

  /** Post-crash data */
  postCrash?: PostCrashData;

  /** Vehicle identification */
  vehicleId: string;

  /** VIN number */
  vin?: string;
}

/**
 * Pre-crash data recording
 */
export interface PreCrashData {
  /** Duration in seconds */
  duration: number;

  /** Sample rate in Hz */
  sampleRate: number;

  /** Time series data */
  data: Array<{
    time: number;           // Seconds before crash
    speed: number;          // km/h
    throttle: number;       // 0-100%
    brake: number;          // 0-100%
    steering: number;       // degrees
    abs: boolean;
    esc: boolean;
    aebActive: boolean;
  }>;
}

/**
 * Crash event data
 */
export interface CrashEventData {
  /** Event timestamp */
  timestamp: Date | string;

  /** Event duration in seconds */
  duration: number;

  /** Sample rate in Hz */
  sampleRate: number;

  /** Delta-V (velocity change) */
  deltaV: {
    longitudinal: number;   // km/h
    lateral: number;        // km/h
    vertical?: number;      // km/h
    resultant: number;      // km/h
  };

  /** Peak acceleration */
  peakAcceleration: {
    x: number;  // g
    y: number;  // g
    z: number;  // g
  };

  /** Airbag deployment times */
  airbagDeployment: {
    [airbagName: string]: {
      deployed: boolean;
      time: number | null;  // milliseconds
      stage: number;
    };
  };

  /** Belt pretensioner activation */
  beltPretensioner: {
    [position: string]: {
      activated: boolean;
      time: number | null;  // milliseconds
    };
  };

  /** Multiple event indicator */
  multipleEvents: boolean;

  /** Rollover indicator */
  rollover?: boolean;

  /** Number of rollovers */
  rolloverCount?: number;
}

/**
 * Post-crash data
 */
export interface PostCrashData {
  /** eCall triggered */
  eCallTriggered: boolean;

  /** eCall timestamp */
  eCallTime?: Date | string;

  /** Emergency lights activated */
  hazardLights: boolean;

  /** Doors unlocked */
  doorsUnlocked: boolean;

  /** Fuel pump status */
  fuelPumpCutoff: boolean;

  /** Battery status */
  batteryStatus: 'normal' | 'isolated' | 'disconnected';

  /** Fire detected */
  fireDetected?: boolean;
}

// ============================================================================
// Safety System Status
// ============================================================================

/**
 * Safety system monitoring
 */
export interface SafetySystemStatus {
  /** Timestamp */
  timestamp: Date | string;

  /** Vehicle identifier */
  vehicleId: string;

  /** System statuses */
  systems: {
    abs: SystemStatus;
    esc: SystemStatus;
    aeb: SystemStatus;
    ldw: SystemStatus;
    bsm: SystemStatus;
    airbags: AirbagSystemStatus;
    seatbelts: SeatbeltStatus;
  };

  /** Overall health score (0-100) */
  healthScore: number;

  /** Active warnings */
  warnings: string[];

  /** Critical faults */
  criticalFaults: string[];
}

/**
 * Individual system status
 */
export interface SystemStatus {
  /** Operational status */
  status: 'operational' | 'degraded' | 'failed' | 'disabled';

  /** Last self-test date */
  lastTest?: Date | string;

  /** Fault codes */
  faultCodes: string[];

  /** Calibration date */
  calibrationDate?: Date | string;

  /** System availability (0-100%) */
  availability?: number;
}

/**
 * Airbag system status
 */
export interface AirbagSystemStatus {
  /** Overall status */
  status: 'armed' | 'disarmed' | 'fault' | 'deployed';

  /** Individual airbag statuses */
  airbags: {
    [position: string]: {
      status: 'armed' | 'suppressed' | 'deployed' | 'fault';
      deploymentCount: number;
      suppressionReason?: string;
    };
  };

  /** Crash sensor status */
  sensors: SystemStatus;

  /** Control unit status */
  controlUnit: SystemStatus;
}

/**
 * Seatbelt status
 */
export interface SeatbeltStatus {
  /** Belt status by position */
  belts: {
    [position: string]: {
      fastened: boolean;
      pretensionerArmed: boolean;
      loadLimiterStatus: 'normal' | 'deployed' | 'fault';
      reminderActive: boolean;
    };
  };
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Vehicle safety physical constants and thresholds
 */
export const SAFETY_CONSTANTS = {
  /** Speed of light (for sensor calculations) */
  SPEED_OF_LIGHT: 299792458, // m/s

  /** Gravitational acceleration */
  GRAVITY: 9.81, // m/s²

  /** Air density (sea level, 20°C) */
  AIR_DENSITY: 1.204, // kg/m³

  /** Injury thresholds */
  INJURY_THRESHOLDS: {
    /** Head Injury Criterion (15ms) */
    HIC_15: 700,

    /** Head Injury Criterion (36ms) */
    HIC_36: 1000,

    /** Chest compression (mm) */
    CHEST_COMPRESSION: 50,

    /** Chest acceleration (3ms, g) */
    CHEST_ACCELERATION: 60,

    /** Femur load (kN) */
    FEMUR_LOAD: 10,

    /** Tibia index */
    TIBIA_INDEX: 1.3,

    /** Neck tension (kN) */
    NECK_TENSION: 3.3,

    /** Neck compression (kN) */
    NECK_COMPRESSION: 4.0,
  },

  /** Crash test speeds (km/h) */
  TEST_SPEEDS: {
    FRONTAL_OFFSET: 64,
    FRONTAL_FULL: 56,
    SIDE_BARRIER: 50,
    SIDE_POLE: 32,
    REAR_IMPACT: 16,
  },

  /** Typical vehicle parameters */
  TYPICAL_VEHICLE: {
    MASS: 1500,              // kg
    WHEELBASE: 2.7,          // m
    TRACK_WIDTH: 1.5,        // m
    CENTER_OF_GRAVITY: 0.6,  // m height
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

/**
 * Time series data point
 */
export interface TimeSeriesPoint {
  timestamp: number;
  value: number;
  unit?: string;
}

/**
 * Safety rating levels
 */
export type SafetyRating = 0 | 1 | 2 | 3 | 4 | 5;

/**
 * Injury severity levels (AIS)
 */
export type InjurySeverity = 0 | 1 | 2 | 3 | 4 | 5 | 6;

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-AUTO-022 error codes
 */
export enum SafetyErrorCode {
  INVALID_PARAMETERS = 'S001',
  SENSOR_FAILURE = 'S002',
  AIRBAG_FAULT = 'S003',
  SYSTEM_DEGRADED = 'S004',
  CALIBRATION_REQUIRED = 'S005',
  CRASH_DETECTED = 'S006',
  EMERGENCY_CONDITION = 'S007',
  DATA_CORRUPTION = 'S008',
}

/**
 * Vehicle safety error
 */
export class VehicleSafetyError extends Error {
  constructor(
    public code: SafetyErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'VehicleSafetyError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  // Core
  VehiclePose,

  // Crash Assessment
  CrashAssessmentRequest,
  CrashAssessmentResponse,
  OccupantInfo,
  InjuryAssessment,
  StructuralIntegrity,

  // Active Safety
  CollisionWarningRequest,
  CollisionWarningResponse,
  VehicleState,
  Obstacle,
  CollisionThreat,

  // Airbags
  AirbagDeploymentRequest,
  AirbagDeploymentResponse,
  AirbagDeployment,
  PretensionerDeployment,

  // Child Safety
  ChildRestraintSystem,
  ChildSafetyRequest,
  ChildSafetyResponse,

  // Testing
  NCAPTestConfig,
  DummyInfo,
  NCAPScoring,

  // Pedestrian
  PedestrianImpactRequest,
  PedestrianImpactResponse,

  // EDR
  EDRSnapshot,
  PreCrashData,
  CrashEventData,
  PostCrashData,

  // Status
  SafetySystemStatus,
  SystemStatus,
  AirbagSystemStatus,
  SeatbeltStatus,

  // Utility
  TimeSeriesPoint,
  SafetyRating,
  InjurySeverity,
};

export { SAFETY_CONSTANTS, SafetyErrorCode, VehicleSafetyError };
