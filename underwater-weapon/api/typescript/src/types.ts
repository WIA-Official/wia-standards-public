/**
 * WIA-DEF-019: Underwater Weapon - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Defense & Security Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * Geographic coordinates with optional depth
 */
export interface GeoCoordinate {
  latitude: number;
  longitude: number;
  depth?: number; // meters below surface
}

/**
 * Three-dimensional vector
 */
export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

/**
 * Environmental conditions
 */
export interface EnvironmentalConditions {
  temperature: number; // °C
  salinity: number; // PSU (Practical Salinity Units)
  depth: number; // meters
  seaState: number; // 0-9 (Beaufort-like scale)
  currentSpeed?: number; // knots
  currentDirection?: number; // degrees
}

// ============================================================================
// Torpedo Systems
// ============================================================================

/**
 * Torpedo classification types
 */
export type TorpedoClass =
  | 'Heavy Torpedo (533mm)'
  | 'Heavy Torpedo (650mm)'
  | 'Light Torpedo (324mm)'
  | 'Supercavitating Torpedo';

/**
 * Torpedo propulsion types
 */
export type TorpedoPropulsion =
  | 'Electric'
  | 'Thermal (Otto Fuel II)'
  | 'Closed-Cycle'
  | 'Rocket';

/**
 * Torpedo guidance modes
 */
export type TorpedoGuidanceMode =
  | 'Wire-Guided'
  | 'Active Acoustic Homing'
  | 'Passive Acoustic Homing'
  | 'Wake Homing'
  | 'Inertial'
  | 'AI-Enhanced';

/**
 * Torpedo warhead types
 */
export type TorpedoWarheadType =
  | 'High Explosive (HE)'
  | 'Shaped Charge'
  | 'Dual-Purpose';

/**
 * Torpedo guidance system configuration
 */
export interface TorpedoGuidance {
  primary: TorpedoGuidanceMode;
  secondary?: TorpedoGuidanceMode;
  ai: boolean; // AI-enhanced target discrimination
  wireLength?: number; // meters (if wire-guided)
}

/**
 * Torpedo warhead configuration
 */
export interface TorpedoWarhead {
  type: TorpedoWarheadType;
  weight: number; // kg
  fuzing: 'Contact' | 'Proximity' | 'Magnetic' | 'Combined';
}

/**
 * Complete torpedo configuration
 */
export interface TorpedoConfiguration {
  class: TorpedoClass;
  diameter: number; // mm
  length: number; // meters
  weight: number; // kg
  propulsion: TorpedoPropulsion;
  speed: {
    cruise: number; // knots
    sprint?: number; // knots (if dual-speed)
  };
  range: number; // meters
  maxDepth: number; // meters
  guidance: TorpedoGuidance;
  warhead: TorpedoWarhead;
  endurance: number; // minutes
}

/**
 * Torpedo attack parameters
 */
export interface TorpedoAttackParams {
  torpedoSpeed: number; // knots
  targetSpeed: number; // knots
  targetBearing: number; // degrees (0-360)
  range: number; // meters
  depth: number; // meters
  targetDepth?: number; // meters
}

/**
 * Torpedo intercept calculation result
 */
export interface TorpedoInterceptResult {
  timeToImpact: number; // seconds
  leadAngle: number; // degrees
  fuelRequired: number; // liters or % of capacity
  probabilityOfHit: number; // 0-1
  guidanceMode: TorpedoGuidanceMode;
}

// ============================================================================
// Naval Mine Warfare
// ============================================================================

/**
 * Mine classification types
 */
export type MineType =
  | 'Contact Mine (Moored)'
  | 'Contact Mine (Bottom)'
  | 'Magnetic Influence Mine'
  | 'Acoustic Influence Mine'
  | 'Pressure Influence Mine'
  | 'Combined Influence Mine'
  | 'Rising Mine'
  | 'Intelligent Mine';

/**
 * Mine fuzing logic
 */
export type MineFuzingLogic =
  | 'Single Activation'
  | 'Ship Count'
  | 'Total Ship Count'
  | 'AND Logic'
  | 'OR Logic'
  | 'AI Decision';

/**
 * Mine deployment methods
 */
export type MineDeploymentMethod =
  | 'Surface Vessel'
  | 'Submarine'
  | 'Aircraft'
  | 'Helicopter'
  | 'UUV';

/**
 * Mine influence sensors
 */
export interface MineInfluenceSensors {
  magnetic?: boolean;
  acoustic?: boolean;
  pressure?: boolean;
  seismic?: boolean;
  ai?: boolean; // AI-based target classification
}

/**
 * Mine configuration
 */
export interface MineConfiguration {
  type: MineType;
  weight: number; // kg
  charge: number; // kg HE
  depthRange: {
    min: number; // meters
    max: number; // meters
  };
  sensors: MineInfluenceSensors;
  fuzing: MineFuzingLogic;
  lifespan: number; // years (battery/chemical life)
  safetyFeatures: {
    selfDeactivation: boolean;
    selfNeutralization: boolean;
    friendOrFoe: boolean; // IFF capability
  };
}

/**
 * Minefield layout pattern
 */
export type MinefieldPattern =
  | 'Linear (Barrier)'
  | 'Grid'
  | 'Staggered'
  | 'Concentric (Defensive)'
  | 'Random (Covert)';

/**
 * Minefield density
 */
export type MinefieldDensity = 'Low' | 'Medium' | 'High' | 'Very High';

/**
 * Minefield layout parameters
 */
export interface MinefieldLayout {
  area: {
    width: number; // meters
    length: number; // meters
  };
  mineType: MineType;
  density: MinefieldDensity;
  pattern: MinefieldPattern;
  waterDepth: number; // meters
  purpose: 'Defensive' | 'Offensive' | 'Training';
}

/**
 * Minefield design result
 */
export interface MinefieldDesign {
  totalMines: number;
  mineSpacing: number; // meters
  deploymentTime: number; // hours
  deploymentMethod: MineDeploymentMethod;
  coverage: number; // percentage of area
  estimatedEffectiveness: number; // 0-1
}

// ============================================================================
// Unmanned Underwater Vehicles (UUVs)
// ============================================================================

/**
 * UUV classification types
 */
export type UUVType =
  | 'AUV (Autonomous)'
  | 'ROV (Remotely Operated)'
  | 'Underwater Glider'
  | 'Hybrid AUV/ROV';

/**
 * UUV mission types
 */
export type UUVMissionType =
  | 'Mine Reconnaissance'
  | 'Mine Clearance'
  | 'ASW (Anti-Submarine Warfare)'
  | 'Surveillance'
  | 'Oceanographic Survey'
  | 'Cable Inspection'
  | 'Seafloor Mapping'
  | 'Search & Recovery';

/**
 * UUV navigation systems
 */
export interface UUVNavigation {
  ins: boolean; // Inertial Navigation System
  dvl: boolean; // Doppler Velocity Log
  gps: boolean; // GPS (surface only)
  acoustic: 'LBL' | 'USBL' | 'SBL' | 'None'; // Acoustic positioning
  terrainReferenced: boolean; // TRN
  magneticAnomaly: boolean; // Magnetic navigation
}

/**
 * UUV sensor suite
 */
export interface UUVSensorSuite {
  sonar: {
    sideScan: boolean;
    forwardLooking: boolean;
    syntheticAperture: boolean; // SAS
  };
  cameras: {
    still: boolean;
    video: boolean;
    lowLight: boolean;
  };
  magnetometer: boolean;
  ctd: boolean; // Conductivity-Temperature-Depth
  other?: string[];
}

/**
 * UUV configuration
 */
export interface UUVConfiguration {
  type: UUVType;
  length: number; // meters
  diameter: number; // meters
  weight: number; // kg
  maxDepth: number; // meters
  endurance: number; // hours
  speed: {
    cruise: number; // knots
    max: number; // knots
  };
  navigation: UUVNavigation;
  sensors: UUVSensorSuite;
  payload?: string; // Mission-specific payload description
}

/**
 * UUV mission parameters
 */
export interface UUVMission {
  type: UUVMissionType;
  area: GeoCoordinate & { radius: number }; // meters
  depth: {
    operating: number; // meters
    min?: number;
    max?: number;
  };
  duration: number; // hours
  surveyPattern?: 'Grid' | 'Spiral' | 'Lawn Mower' | 'Random';
  waypoints?: GeoCoordinate[];
}

/**
 * UUV mission execution result
 */
export interface UUVMissionExecution {
  missionValid: boolean;
  estimatedDistance: number; // km
  estimatedTime: number; // hours
  batteryRequired: number; // percentage
  coverageArea?: number; // km²
  warnings?: string[];
}

// ============================================================================
// Sonar Systems
// ============================================================================

/**
 * Sonar types
 */
export type SonarType =
  | 'Active Sonar (Hull-Mounted)'
  | 'Active Sonar (Towed Array)'
  | 'Passive Sonar (Hull-Mounted)'
  | 'Passive Sonar (Towed Array)'
  | 'Synthetic Aperture Sonar (SAS)'
  | 'Side-Scan Sonar'
  | 'Forward-Looking Sonar';

/**
 * Sonar configuration
 */
export interface SonarConfiguration {
  type: SonarType;
  frequency: {
    min: number; // Hz
    max: number; // Hz
    operating?: number; // Hz (center frequency)
  };
  power?: number; // dB re 1 μPa @ 1m (for active sonar)
  arrayGain?: number; // dB (for passive sonar)
  beamWidth?: number; // degrees
}

/**
 * Acoustic propagation parameters
 */
export interface AcousticPropagation {
  frequency: number; // Hz
  power: number; // dB re 1 μPa @ 1m
  environment: EnvironmentalConditions;
  bathymetry?: {
    depth: number; // meters
    bottomType: 'Sand' | 'Mud' | 'Rock' | 'Gravel';
  };
}

/**
 * Sonar detection range calculation result
 */
export interface SonarRangeResult {
  maxRange: number; // meters
  convergenceZones?: number[]; // distances in meters
  shadowZones?: { start: number; end: number }[]; // ranges in meters
  propagationMode:
    | 'Direct Path'
    | 'Surface Duct'
    | 'Deep Sound Channel'
    | 'Convergence Zone'
    | 'Bottom Bounce';
  confidenceLevel: number; // 0-1
}

// ============================================================================
// Countermeasures
// ============================================================================

/**
 * Torpedo countermeasure types
 */
export type TorpedoCountermeasureType =
  | 'Acoustic Decoy (Noisemaker)'
  | 'Acoustic Jammer'
  | 'Towed Decoy'
  | 'Mobile Decoy (Self-Propelled)'
  | 'Hard-Kill (Anti-Torpedo Torpedo)';

/**
 * Mine countermeasure types
 */
export type MineCountermeasureType =
  | 'Mine Hunting (Detection & Disposal)'
  | 'Mechanical Sweeping'
  | 'Influence Sweeping (Magnetic)'
  | 'Influence Sweeping (Acoustic)'
  | 'Influence Sweeping (Pressure)'
  | 'Mine Avoidance';

/**
 * Countermeasure configuration
 */
export interface CountermeasureConfiguration {
  type: TorpedoCountermeasureType | MineCountermeasureType;
  deploymentMethod: 'Ejected' | 'Towed' | 'Self-Propelled' | 'Fixed';
  effectiveness: number; // 0-1 (against specific threats)
  quantity?: number;
  endurance?: number; // minutes (for active decoys)
}

// ============================================================================
// Mine Clearance Operations
// ============================================================================

/**
 * Mine clearance purpose
 */
export type MineClearancePurpose =
  | 'Humanitarian'
  | 'Sea Lane Restoration'
  | 'Port Security'
  | 'Infrastructure Protection'
  | 'Environmental';

/**
 * Mine clearance method
 */
export type MineClearanceMethod =
  | 'UUV Hunting'
  | 'ROV Disposal'
  | 'Diver Clearance'
  | 'Mechanical Sweeping'
  | 'Influence Sweeping'
  | 'Controlled Detonation';

/**
 * Mine clearance operation parameters
 */
export interface MineClearanceOperation {
  area: GeoCoordinate & { radius: number }; // meters
  depth: {
    min: number; // meters
    max: number; // meters
  };
  method: MineClearanceMethod;
  safety: 'Maximum' | 'High' | 'Standard';
  purpose: MineClearancePurpose;
  estimatedMineCount?: number;
}

/**
 * Mine clearance result
 */
export interface MineClearanceResult {
  operationValid: boolean;
  estimatedDuration: number; // hours or days
  resourcesRequired: {
    uuvs?: number;
    rovs?: number;
    divers?: number;
    supportVessels?: number;
  };
  coverageArea: number; // km²
  clearanceConfidence: number; // 0-1
  safetyRating: 'Low Risk' | 'Medium Risk' | 'High Risk';
  recommendations?: string[];
}

// ============================================================================
// Underwater Acoustics
// ============================================================================

/**
 * Sound speed calculation parameters
 */
export interface SoundSpeedParams {
  temperature: number; // °C
  salinity: number; // PSU
  depth: number; // meters
}

/**
 * Sound speed calculation result
 */
export interface SoundSpeedResult {
  soundSpeed: number; // m/s
  gradient: number; // m/s per meter depth
  profile?: { depth: number; speed: number }[]; // Sound speed profile
}

/**
 * Acoustic absorption result
 */
export interface AcousticAbsorption {
  frequency: number; // Hz
  absorptionCoefficient: number; // dB/km
  rangeFor3dBLoss: number; // meters
  rangeFor10dBLoss: number; // meters
}

/**
 * Target strength parameters
 */
export interface TargetStrengthParams {
  targetType:
    | 'Large Submarine'
    | 'Small Submarine'
    | 'Surface Ship'
    | 'Mine'
    | 'UUV'
    | 'Fish School';
  aspect: 'Bow' | 'Stern' | 'Beam' | 'Unknown';
  frequency: number; // Hz
}

/**
 * Target strength result
 */
export interface TargetStrengthResult {
  targetStrength: number; // dB
  aspectDependency: boolean;
  frequencyDependency: boolean;
  range?: { min: number; max: number }; // dB range
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Physical constants for underwater acoustics and weapons
 */
export const UNDERWATER_CONSTANTS = {
  // Sound speed
  SOUND_SPEED_AVERAGE: 1500, // m/s
  SOUND_SPEED_RANGE: { min: 1450, max: 1540 }, // m/s

  // Unit conversions
  KNOTS_TO_MS: 0.514444, // m/s per knot
  KNOTS_TO_KMH: 1.852, // km/h per knot
  FEET_TO_METERS: 0.3048,
  METERS_TO_FEET: 3.28084,

  // Earth
  EARTH_RADIUS: 6371000, // meters

  // Typical sonar frequencies
  SONAR_FREQUENCIES: {
    VLF: { min: 10, max: 100 }, // Hz (very low frequency)
    LF: { min: 100, max: 1000 }, // Hz (low frequency)
    MF: { min: 1000, max: 10000 }, // Hz (medium frequency)
    HF: { min: 10000, max: 100000 }, // Hz (high frequency)
  },

  // Torpedo speeds (typical ranges in knots)
  TORPEDO_SPEEDS: {
    SLOW: { min: 25, max: 35 },
    MEDIUM: { min: 35, max: 50 },
    FAST: { min: 50, max: 65 },
    SUPERCAVITATING: { min: 200, max: 250 },
  },

  // Detection thresholds
  DETECTION_THRESHOLD: 10, // dB (signal-to-noise ratio)
  CLASSIFICATION_THRESHOLD: 15, // dB

  // Sea states
  SEA_STATES: {
    0: 'Calm (glassy)',
    1: 'Calm (rippled)',
    2: 'Smooth',
    3: 'Slight',
    4: 'Moderate',
    5: 'Rough',
    6: 'Very rough',
    7: 'High',
    8: 'Very high',
    9: 'Phenomenal',
  },
} as const;

// ============================================================================
// Error Types
// ============================================================================

/**
 * Underwater weapon system error codes
 */
export enum UnderwaterWeaponErrorCode {
  INVALID_DEPTH = 'INVALID_DEPTH',
  INVALID_RANGE = 'INVALID_RANGE',
  INVALID_FREQUENCY = 'INVALID_FREQUENCY',
  INSUFFICIENT_POWER = 'INSUFFICIENT_POWER',
  ENVIRONMENTAL_CONSTRAINT = 'ENVIRONMENTAL_CONSTRAINT',
  GUIDANCE_FAILURE = 'GUIDANCE_FAILURE',
  SENSOR_FAILURE = 'SENSOR_FAILURE',
  COMMUNICATION_LOSS = 'COMMUNICATION_LOSS',
  SAFETY_VIOLATION = 'SAFETY_VIOLATION',
  INVALID_TARGET = 'INVALID_TARGET',
}

/**
 * Underwater weapon system error
 */
export class UnderwaterWeaponError extends Error {
  code: UnderwaterWeaponErrorCode;
  details?: Record<string, unknown>;

  constructor(
    code: UnderwaterWeaponErrorCode,
    message: string,
    details?: Record<string, unknown>
  ) {
    super(message);
    this.code = code;
    this.details = details;
    this.name = 'UnderwaterWeaponError';
  }
}

// ============================================================================
// Export All Types
// ============================================================================

export type {
  // Core types exported above
};
