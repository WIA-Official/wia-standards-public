/**
 * WIA-DEF-007: Laser Weapon Standard - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Defense Systems Working Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Physics Types
// ============================================================================

/**
 * Three-dimensional vector for spatial coordinates and velocities
 */
export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

/**
 * Spherical coordinates (range, azimuth, elevation)
 */
export interface SphericalCoordinate {
  /** Range in meters */
  range: number;

  /** Azimuth angle in radians (0 = North, clockwise) */
  azimuth: number;

  /** Elevation angle in radians (0 = horizon, positive up) */
  elevation: number;
}

/**
 * Wavelength specification
 */
export interface Wavelength {
  /** Wavelength in meters */
  value: number;

  /** Wavelength in nanometers (for display) */
  nm: number;

  /** Photon energy in joules */
  photonEnergy: number;
}

// ============================================================================
// Laser System Types
// ============================================================================

/**
 * Laser system type classification
 */
export type LaserType =
  | 'solid-state'
  | 'fiber'
  | 'chemical'
  | 'free-electron'
  | 'diode'
  | 'gas';

/**
 * Laser system configuration
 */
export interface LaserSystemConfig {
  /** System identifier */
  id: string;

  /** System name */
  name?: string;

  /** Laser type */
  type: LaserType;

  /** Maximum output power in watts */
  power: number;

  /** Operating wavelength */
  wavelength: number;

  /** Beam quality factor (M²) */
  beamQuality: number;

  /** Wall-plug efficiency (0-1) */
  efficiency: number;

  /** Cooling capacity in watts */
  coolingCapacity: number;

  /** Maximum duty cycle (0-1) */
  maxDutyCycle: number;

  /** Primary application */
  application?: 'c-ram' | 'c-uas' | 'anti-missile' | 'eod' | 'general';

  /** Operational status */
  status: 'offline' | 'standby' | 'ready' | 'active' | 'maintenance' | 'error';
}

/**
 * Laser system operational state
 */
export interface LaserSystemState {
  /** Current output power in watts */
  currentPower: number;

  /** Current duty cycle (0-1) */
  currentDutyCycle: number;

  /** Component temperatures in Celsius */
  temperatures: {
    laserHead: number;
    pumpDiodes: number;
    optics: number;
    coolant: number;
  };

  /** Total operating time in hours */
  operatingHours: number;

  /** Shots fired count */
  shotsFired: number;

  /** Last maintenance date */
  lastMaintenance?: Date;

  /** Error codes */
  errors: string[];

  /** Warning codes */
  warnings: string[];
}

// ============================================================================
// Beam Characteristics
// ============================================================================

/**
 * Beam parameters at a specific point
 */
export interface BeamParameters {
  /** Beam radius in meters */
  radius: number;

  /** Beam divergence in radians */
  divergence: number;

  /** Beam intensity in W/m² */
  intensity: number;

  /** Power in watts */
  power: number;

  /** M² beam quality factor */
  beamQuality: number;

  /** Rayleigh range in meters */
  rayleighRange: number;

  /** Beam waist radius in meters */
  waistRadius: number;
}

/**
 * Beam propagation result
 */
export interface BeamPropagation {
  /** Initial beam parameters */
  initial: BeamParameters;

  /** Final beam parameters at target */
  final: BeamParameters;

  /** Propagation distance in meters */
  distance: number;

  /** Atmospheric transmission coefficient (0-1) */
  transmission: number;

  /** Energy delivered on target in watts */
  deliveredPower: number;

  /** Spot size on target in meters */
  spotSize: number;

  /** Peak intensity on target in W/m² */
  peakIntensity: number;
}

// ============================================================================
// Atmospheric Conditions
// ============================================================================

/**
 * Atmospheric conditions for propagation modeling
 */
export interface AtmosphericConditions {
  /** Temperature in Celsius */
  temperature: number;

  /** Relative humidity (0-100) */
  humidity: number;

  /** Atmospheric pressure in Pascals */
  pressure: number;

  /** Visibility in meters */
  visibility: number;

  /** Wind speed in m/s */
  windSpeed?: number;

  /** Wind direction in radians */
  windDirection?: number;

  /** Cloud cover (0-1, 0=clear, 1=overcast) */
  cloudCover?: number;

  /** Precipitation rate in mm/hr */
  precipitation?: number;

  /** Air density in kg/m³ */
  density?: number;
}

/**
 * Atmospheric attenuation result
 */
export interface AtmosphericAttenuation {
  /** Total attenuation coefficient in km⁻¹ */
  coefficient: number;

  /** Aerosol contribution in km⁻¹ */
  aerosol: number;

  /** Molecular absorption in km⁻¹ */
  molecular: number;

  /** Scattering contribution in km⁻¹ */
  scattering: number;

  /** Transmission over distance (0-1) */
  transmission: number;

  /** Distance used for calculation in meters */
  distance: number;
}

/**
 * Thermal blooming calculation result
 */
export interface ThermalBlooming {
  /** Thermal distortion parameter (dimensionless) */
  distortionParameter: number;

  /** Beam deflection angle in radians */
  deflection: number;

  /** Effective intensity reduction factor (0-1) */
  reductionFactor: number;

  /** Critical for effective engagement */
  severity: 'negligible' | 'minor' | 'moderate' | 'severe' | 'critical';
}

// ============================================================================
// Targeting & Tracking
// ============================================================================

/**
 * Target classification
 */
export type TargetType =
  | 'rocket'
  | 'mortar'
  | 'artillery'
  | 'uav'
  | 'missile'
  | 'aircraft'
  | 'surface'
  | 'unknown';

/**
 * Target characteristics
 */
export interface Target {
  /** Unique target identifier */
  id: string;

  /** Target type */
  type: TargetType;

  /** Current position */
  position: Vector3;

  /** Velocity vector in m/s */
  velocity: Vector3;

  /** Acceleration vector in m/s² */
  acceleration?: Vector3;

  /** Range to target in meters */
  range: number;

  /** Target size (diameter/length) in meters */
  size: number;

  /** Radar cross-section in m² */
  rcs?: number;

  /** Material composition */
  material?: 'aluminum' | 'steel' | 'composite' | 'unknown';

  /** Threat level */
  threatLevel: 'low' | 'medium' | 'high' | 'critical';

  /** Track quality (0-1) */
  trackQuality: number;

  /** Detection time */
  detectionTime: Date;

  /** Last update time */
  lastUpdate: Date;

  /** Predicted impact point */
  predictedImpact?: Vector3;

  /** Time to impact in seconds */
  timeToImpact?: number;

  /** IFF status */
  iff?: 'friendly' | 'hostile' | 'neutral' | 'unknown';
}

/**
 * Track file for target tracking
 */
export interface TrackFile {
  /** Track identifier */
  trackId: string;

  /** Associated target */
  target: Target;

  /** Track history (positions over time) */
  history: Array<{
    time: Date;
    position: Vector3;
    velocity: Vector3;
  }>;

  /** Predicted trajectory */
  trajectory: Array<{
    time: Date;
    position: Vector3;
    confidence: number;
  }>;

  /** Track status */
  status: 'initiated' | 'tracking' | 'coasting' | 'lost';

  /** Track age in seconds */
  age: number;
}

// ============================================================================
// Engagement Planning
// ============================================================================

/**
 * Engagement parameters
 */
export interface EngagementParams {
  /** Target to engage */
  target: Target;

  /** Laser system to use */
  system: LaserSystemConfig;

  /** Atmospheric conditions */
  atmospheric: AtmosphericConditions;

  /** Required kill probability (0-1) */
  requiredKillProbability?: number;

  /** Maximum time to kill in seconds */
  maxTimeToKill?: number;

  /** Engagement priority */
  priority?: 'low' | 'medium' | 'high' | 'critical';
}

/**
 * Engagement solution
 */
export interface EngagementSolution {
  /** Is engagement feasible? */
  feasible: boolean;

  /** Engagement range in meters */
  range: number;

  /** Time to kill in seconds */
  timeToKill: number;

  /** Required dwell time in seconds */
  dwellTime: number;

  /** Predicted kill probability (0-1) */
  killProbability: number;

  /** Beam parameters at target */
  beamAtTarget: BeamParameters;

  /** Energy required in joules */
  energyRequired: number;

  /** Recommended aimpoint on target */
  aimpoint?: Vector3;

  /** Engagement window start time */
  windowStart: Date;

  /** Engagement window end time */
  windowEnd: Date;

  /** Risk assessment */
  risk: 'low' | 'medium' | 'high' | 'extreme';

  /** Reasons if not feasible */
  limitations?: string[];

  /** Warnings */
  warnings: string[];
}

/**
 * Engagement result
 */
export interface EngagementResult {
  /** Engagement identifier */
  id: string;

  /** Target engaged */
  target: Target;

  /** Start time */
  startTime: Date;

  /** End time */
  endTime: Date;

  /** Actual dwell time in seconds */
  dwellTime: number;

  /** Energy delivered in joules */
  energyDelivered: number;

  /** Kill assessment */
  killAssessment: 'kill' | 'probable-kill' | 'damage' | 'miss' | 'unknown';

  /** Confidence in assessment (0-1) */
  confidence: number;

  /** Observable effects */
  effects?: string[];

  /** Post-engagement track */
  postEngagementTrack?: TrackFile;

  /** Engagement success */
  success: boolean;
}

// ============================================================================
// Beam Director & Fire Control
// ============================================================================

/**
 * Beam director configuration
 */
export interface BeamDirector {
  /** Director identifier */
  id: string;

  /** Current pointing direction */
  pointing: SphericalCoordinate;

  /** Maximum slew rate in rad/s */
  maxSlewRate: number;

  /** Pointing accuracy in radians */
  pointingAccuracy: number;

  /** Tracking update rate in Hz */
  updateRate: number;

  /** Field of regard */
  fieldOfRegard: {
    azimuthMin: number;
    azimuthMax: number;
    elevationMin: number;
    elevationMax: number;
  };

  /** Gimbal status */
  status: 'ready' | 'tracking' | 'slewing' | 'parked' | 'error';

  /** Jitter RMS in radians */
  jitterRMS?: number;
}

/**
 * Adaptive optics system
 */
export interface AdaptiveOpticsSystem {
  /** AO system identifier */
  id: string;

  /** Deformable mirror actuator count */
  actuatorCount: number;

  /** Update rate in Hz */
  updateRate: number;

  /** Strehl ratio (0-1, 1=perfect) */
  strehlRatio: number;

  /** Wavefront RMS error in waves */
  wavefrontError: number;

  /** AO system status */
  status: 'off' | 'acquiring' | 'correcting' | 'locked' | 'error';

  /** Atmospheric coherence length (r₀) in meters */
  friedParameter?: number;
}

/**
 * Fire control solution
 */
export interface FireControlSolution {
  /** Target track file */
  track: TrackFile;

  /** Predicted target position at engagement time */
  predictedPosition: Vector3;

  /** Engagement time */
  engagementTime: Date;

  /** Beam director pointing command */
  pointingCommand: SphericalCoordinate;

  /** Lead angle compensation in radians */
  leadAngle: Vector3;

  /** Power level command in watts */
  powerCommand: number;

  /** Dwell time command in seconds */
  dwellTime: number;

  /** Engage authorization */
  authorized: boolean;

  /** Safety checks passed */
  safetyChecks: SafetyCheck[];
}

// ============================================================================
// Safety & Validation
// ============================================================================

/**
 * Safety check result
 */
export interface SafetyCheck {
  /** Check name */
  name: string;

  /** Check result */
  passed: boolean;

  /** Check status */
  status: 'pass' | 'fail' | 'warning' | 'info';

  /** Description */
  description: string;

  /** Measured value */
  value?: number | string;

  /** Expected/threshold value */
  threshold?: number | string;

  /** Corrective action if failed */
  correctiveAction?: string;

  /** Severity */
  severity: 'info' | 'warning' | 'error' | 'critical';
}

/**
 * Laser safety zone
 */
export interface LaserSafetyZone {
  /** Zone type */
  type: 'exclusion' | 'controlled' | 'caution';

  /** Nominal Ocular Hazard Distance in meters */
  nohd: number;

  /** Zone radius in meters */
  radius: number;

  /** Zone center point */
  center: Vector3;

  /** Is zone clear of personnel? */
  isClear: boolean;

  /** Personnel count in zone */
  personnelCount?: number;
}

/**
 * Engagement validation result
 */
export interface ValidationResult {
  /** Is engagement valid? */
  isValid: boolean;

  /** Errors (blocking) */
  errors: string[];

  /** Warnings (non-blocking) */
  warnings: string[];

  /** Information messages */
  info: string[];

  /** Safety checks */
  safetyChecks: SafetyCheck[];

  /** IFF check result */
  iffStatus: 'cleared' | 'friendly' | 'unknown' | 'failed';

  /** Collateral damage assessment */
  collateralDamage: 'none' | 'low' | 'medium' | 'high' | 'unacceptable';

  /** Background safety check */
  backgroundSafe: boolean;

  /** Recommended action */
  recommendation: 'engage' | 'engage-with-caution' | 'do-not-engage' | 'abort';
}

// ============================================================================
// Thermal Management
// ============================================================================

/**
 * Thermal state
 */
export interface ThermalState {
  /** Waste heat generation in watts */
  wasteHeat: number;

  /** Cooling capacity in watts */
  coolingCapacity: number;

  /** Coolant flow rate in L/min */
  flowRate: number;

  /** Coolant inlet temperature in Celsius */
  inletTemp: number;

  /** Coolant outlet temperature in Celsius */
  outletTemp: number;

  /** Component temperatures */
  temperatures: {
    [component: string]: number;
  };

  /** Current duty cycle (0-1) */
  dutyCycle: number;

  /** Maximum safe duty cycle (0-1) */
  maxDutyCycle: number;

  /** Thermal margin (positive = OK, negative = overtemp) */
  thermalMargin: number;

  /** Thermal status */
  status: 'cold' | 'normal' | 'warm' | 'hot' | 'overtemp' | 'critical';
}

/**
 * Thermal management plan
 */
export interface ThermalManagementPlan {
  /** Operating power in watts */
  operatingPower: number;

  /** Planned duty cycle (0-1) */
  plannedDutyCycle: number;

  /** Required cooling in watts */
  requiredCooling: number;

  /** Cool-down periods */
  coolDownPeriods: Array<{
    afterSeconds: number;
    durationSeconds: number;
  }>;

  /** Maximum continuous operation in seconds */
  maxContinuousOperation: number;

  /** Is plan feasible? */
  feasible: boolean;

  /** Warnings */
  warnings: string[];
}

// ============================================================================
// C-RAM Specific Types
// ============================================================================

/**
 * C-RAM threat assessment
 */
export interface CRAMThreat {
  /** Target information */
  target: Target;

  /** Threat classification */
  classification: 'rocket' | 'artillery' | 'mortar';

  /** Caliber/size in meters */
  caliber: number;

  /** Launch point (if detected) */
  launchPoint?: Vector3;

  /** Predicted impact point */
  impactPoint: Vector3;

  /** Time to impact in seconds */
  timeToImpact: number;

  /** Impact velocity in m/s */
  impactVelocity: number;

  /** Estimated warhead size in kg */
  warheadSize?: number;

  /** Priority for engagement */
  priority: number;

  /** Is in protected zone? */
  inProtectedZone: boolean;
}

/**
 * C-RAM engagement scenario
 */
export interface CRAMScenario {
  /** Active threats */
  threats: CRAMThreat[];

  /** Available laser systems */
  systems: LaserSystemConfig[];

  /** Current atmospheric conditions */
  atmospheric: AtmosphericConditions;

  /** Protected zones */
  protectedZones: Array<{
    center: Vector3;
    radius: number;
    priority: number;
  }>;

  /** Engagement solutions */
  solutions: EngagementSolution[];

  /** Recommended engagement sequence */
  engagementSequence: string[]; // threat IDs in order
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Physical constants for laser weapon calculations
 */
export const PHYSICS_CONSTANTS = {
  /** Speed of light in m/s */
  SPEED_OF_LIGHT: 299792458,

  /** Planck's constant in J·s */
  PLANCK_CONSTANT: 6.62607015e-34,

  /** Boltzmann constant in J/K */
  BOLTZMANN_CONSTANT: 1.380649e-23,

  /** Standard atmospheric pressure in Pa */
  STANDARD_PRESSURE: 101325,

  /** Standard temperature in Celsius */
  STANDARD_TEMPERATURE: 15,

  /** Air refractive index at STP */
  AIR_REFRACTIVE_INDEX: 1.000293,

  /** Specific heat capacity of air in J/(kg·K) */
  AIR_SPECIFIC_HEAT: 1005,

  /** Air density at STP in kg/m³ */
  AIR_DENSITY_STP: 1.225,

  /** Thermal expansion coefficient of air in K⁻¹ */
  AIR_THERMAL_EXPANSION: 0.00367,

  /** Maximum permissible exposure for eye (CW, 1 μm) in W/m² */
  MPE_EYE_CW: 5,

  /** Maximum safe beam quality */
  MAX_BEAM_QUALITY: 3.0,

  /** Minimum effective Strehl ratio */
  MIN_STREHL_RATIO: 0.3,
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
 * Simulation result
 */
export interface SimulationResult {
  /** Simulation identifier */
  id: string;

  /** Input parameters */
  parameters: EngagementParams;

  /** Engagement solution */
  solution: EngagementSolution;

  /** Beam propagation */
  propagation: BeamPropagation;

  /** Thermal blooming effects */
  thermalBlooming: ThermalBlooming;

  /** Validation result */
  validation: ValidationResult;

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
 * WIA-DEF-007 error codes
 */
export enum LaserErrorCode {
  INSUFFICIENT_POWER = 'D001',
  BEAM_QUALITY_POOR = 'D002',
  ATMOSPHERIC_ATTENUATION = 'D003',
  THERMAL_BLOOMING = 'D004',
  TARGET_OUT_OF_RANGE = 'D005',
  POINTING_ERROR = 'D006',
  THERMAL_OVERLOAD = 'D007',
  SAFETY_VIOLATION = 'D008',
  IFF_FAILURE = 'D009',
  TRACK_LOST = 'D010',
  SYSTEM_OFFLINE = 'D011',
  INVALID_PARAMETERS = 'D012',
}

/**
 * Laser weapon error
 */
export class LaserWeaponError extends Error {
  constructor(
    public code: LaserErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'LaserWeaponError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  // Core types
  Vector3,
  SphericalCoordinate,
  Wavelength,

  // Laser system
  LaserType,
  LaserSystemConfig,
  LaserSystemState,

  // Beam
  BeamParameters,
  BeamPropagation,

  // Atmospheric
  AtmosphericConditions,
  AtmosphericAttenuation,
  ThermalBlooming,

  // Targeting
  TargetType,
  Target,
  TrackFile,

  // Engagement
  EngagementParams,
  EngagementSolution,
  EngagementResult,

  // Fire control
  BeamDirector,
  AdaptiveOpticsSystem,
  FireControlSolution,

  // Safety
  SafetyCheck,
  LaserSafetyZone,
  ValidationResult,

  // Thermal
  ThermalState,
  ThermalManagementPlan,

  // C-RAM
  CRAMThreat,
  CRAMScenario,

  // Simulation
  SimulationResult,
};

export { PHYSICS_CONSTANTS, LaserErrorCode, LaserWeaponError };
