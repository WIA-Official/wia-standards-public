/**
 * WIA-DEF-002: Military Drone - TypeScript Type Definitions
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
 * Geographic coordinates
 */
export interface GeoCoordinate {
  latitude: number;
  longitude: number;
  altitude?: number; // meters MSL
}

/**
 * Three-dimensional vector
 */
export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

// ============================================================================
// Drone Classifications
// ============================================================================

/**
 * UAV classification by weight and capability
 */
export type DroneClass =
  | 'Class I - Micro/Mini'
  | 'Class II - Small Tactical'
  | 'Class III - Medium Tactical'
  | 'Class IV - MALE'
  | 'Class V - HALE';

/**
 * Mission type classification
 */
export type MissionType =
  | 'ISR'
  | 'Reconnaissance'
  | 'Surveillance'
  | 'CSAR'
  | 'Force Protection'
  | 'Border Security'
  | 'Disaster Response'
  | 'Medical Delivery'
  | 'Logistics'
  | 'Electronic Warfare';

/**
 * Drone classification details
 */
export interface DroneClassification {
  class: DroneClass;
  weightRange: { min: number; max: number }; // kg
  enduranceRange: { min: number; max: number }; // hours
  altitudeRange: { min: number; max: number }; // feet MSL
  rangeKm: { min: number; max: number }; // km
  payloadCapacity: { min: number; max: number }; // kg
  examples?: string[];
}

// ============================================================================
// Drone Configuration
// ============================================================================

/**
 * Complete drone system configuration
 */
export interface DroneConfiguration {
  id: string;
  name: string;
  classification: DroneClass;

  physical: {
    weight: number; // kg (empty)
    maxTakeoffWeight: number; // kg
    wingspan?: number; // meters (fixed-wing)
    length: number; // meters
    height: number; // meters
  };

  performance: {
    maxSpeed: number; // knots
    cruiseSpeed: number; // knots
    maxAltitude: number; // feet MSL
    serviceCeiling: number; // feet MSL
    endurance: number; // hours
    range: number; // km
    climbRate?: number; // ft/min
  };

  propulsion: {
    type: 'electric' | 'gasoline' | 'jet' | 'hybrid';
    fuelCapacity?: number; // liters
    batteryCapacity?: number; // Wh
    powerOutput: number; // kW
  };

  payload: {
    maxWeight: number; // kg
    sensors: SensorConfiguration;
    weapons?: WeaponSystem[];
    cargo?: CargoConfiguration;
  };

  communication: CommunicationSystem;
  autopilot: AutopilotSystem;
}

// ============================================================================
// Sensor Systems
// ============================================================================

/**
 * Electro-optical camera configuration
 */
export interface EOCamera {
  type: 'EO';
  resolution: '720p' | '1080p' | '4K' | '8K';
  sensor: 'CMOS' | 'CCD';
  zoom: {
    optical: number; // e.g., 30x
    digital: number; // e.g., 4x
  };
  fieldOfView: {
    wide: number; // degrees
    narrow: number; // degrees
  };
  frameRate: number; // fps
  stabilization: 'mechanical' | 'electronic' | 'hybrid';
  nightVision?: boolean;
}

/**
 * Infrared camera configuration
 */
export interface IRCamera {
  type: 'IR';
  spectrum: 'SWIR' | 'MWIR' | 'LWIR';
  resolution: string; // e.g., '640x480', '1280x1024'
  detector: 'cooled' | 'uncooled';
  thermalSensitivity: number; // mK (milliKelvin)
  spectralRange: [number, number]; // μm (micrometers)
  frameRate: number; // Hz
}

/**
 * Synthetic Aperture Radar configuration
 */
export interface SARSystem {
  type: 'SAR';
  frequency: 'X-band' | 'Ku-band' | 'Ka-band';
  frequencyGHz: number;
  resolution: number; // meters
  swathWidth: number; // meters
  modes: Array<'stripmap' | 'spotlight' | 'scanSAR' | 'ISAR' | 'GMTI'>;
  polarization: 'single' | 'dual' | 'quad';
  range: number; // km
}

/**
 * LIDAR system configuration
 */
export interface LIDARSystem {
  type: 'LIDAR';
  wavelength: number; // nm (905 or 1550 typical)
  range: number; // meters
  accuracy: number; // cm
  pointCloudRate: number; // points per second
  scanPattern: 'mechanical' | 'solid-state' | 'flash';
}

/**
 * Signals Intelligence system
 */
export interface SIGINTSystem {
  type: 'SIGINT';
  capabilities: Array<'COMINT' | 'ELINT' | 'DF'>;
  frequencyRange: {
    min: number; // MHz
    max: number; // MHz
  };
  bandwidth: number; // MHz
  directionFinding: boolean;
  geolocation: boolean;
}

/**
 * NBC detector system
 */
export interface NBCDetector {
  type: 'NBC';
  detects: Array<'nuclear' | 'biological' | 'chemical'>;
  responseTime: number; // seconds
  detectionRange: number; // meters
  sensitivity: string;
}

/**
 * Combined sensor configuration
 */
export interface SensorConfiguration {
  eo?: EOCamera;
  ir?: IRCamera;
  sar?: SARSystem;
  lidar?: LIDARSystem;
  sigint?: SIGINTSystem;
  nbc?: NBCDetector;
  gimballed: boolean;
  gimbalAxes?: 2 | 3;
  targetTracking?: boolean;
  autoTargeting?: boolean;
}

// ============================================================================
// Weapon Systems (Restricted Use)
// ============================================================================

/**
 * Weapon system (for authorized defensive operations only)
 */
export interface WeaponSystem {
  type: string;
  weight: number; // kg
  range: number; // km
  guidance: 'laser' | 'GPS' | 'INS' | 'imaging';
  warheadType: string;
  quantity: number;
  safetyLock: boolean;
  humanInLoop: boolean; // REQUIRED
}

// ============================================================================
// Cargo Configuration
// ============================================================================

/**
 * Cargo delivery configuration
 */
export interface CargoConfiguration {
  maxWeight: number; // kg
  maxVolume: number; // liters
  compartments: number;
  temperatureControlled: boolean;
  parachuteDrop: boolean;
  precisionDrop: boolean;
  dropAccuracy: number; // meters CEP
}

// ============================================================================
// Communication Systems
// ============================================================================

/**
 * Data link configuration
 */
export interface DataLink {
  name: string;
  type: 'LOS' | 'BLOS';
  frequency: number; // GHz
  band: 'C' | 'Ku' | 'Ka' | 'X' | 'UHF';
  range: number; // km
  dataRate: {
    command: number; // Kbps
    telemetry: number; // Kbps
    video: number; // Mbps
  };
  encryption: 'AES-256' | 'ChaCha20';
  antiJam: boolean;
  frequencyHopping: boolean;
  latency: number; // ms
}

/**
 * Communication system configuration
 */
export interface CommunicationSystem {
  primaryLink: DataLink;
  backupLinks: DataLink[];
  satelliteLink?: {
    provider: string;
    band: 'UHF' | 'X' | 'Ku' | 'Ka';
    dataRate: number; // Mbps
    latency: number; // ms
  };
  redundancy: boolean;
  automaticFailover: boolean;
}

// ============================================================================
// Autopilot Systems
// ============================================================================

/**
 * Autopilot configuration
 */
export interface AutopilotSystem {
  manufacturer: string;
  model: string;
  version: string;

  sensors: {
    imu: boolean; // Inertial Measurement Unit
    gps: boolean;
    barometer: boolean;
    magnetometer: boolean;
    airspeed: boolean;
    lidar?: boolean; // Terrain following
    radar?: boolean; // Collision avoidance
  };

  capabilities: {
    waypoint: boolean;
    loiter: boolean;
    orbit: boolean;
    terrainFollowing: boolean;
    obstacleAvoidance: boolean;
    autoTakeoff: boolean;
    autoLanding: boolean;
    returnToBase: boolean;
  };

  navigationAccuracy: {
    gps: number; // meters
    ins: number; // nm/hour drift
  };

  safetyFeatures: {
    geofencing: boolean;
    lowFuelRTB: boolean;
    lostLinkRTB: boolean;
    emergencyLanding: boolean;
    collisionAvoidance: boolean;
  };
}

// ============================================================================
// Flight Planning
// ============================================================================

/**
 * Waypoint definition
 */
export interface Waypoint {
  id: string;
  position: GeoCoordinate;
  action: 'flyby' | 'loiter' | 'orbit' | 'land' | 'takeoff';
  speed?: number; // knots
  loiterTime?: number; // seconds
  loiterRadius?: number; // meters
  orbitRadius?: number; // meters
  orbitDirection?: 'CW' | 'CCW';
}

/**
 * Flight plan
 */
export interface FlightPlan {
  id: string;
  name: string;
  created: Date;
  creator: string;

  mission: {
    type: MissionType;
    objective: string;
    priority: 'low' | 'medium' | 'high' | 'critical';
  };

  area: {
    center?: GeoCoordinate;
    radius?: number; // meters
    polygon?: GeoCoordinate[];
  };

  flightParameters: {
    altitude: number; // feet MSL
    speed: number; // knots
    estimatedDuration: number; // hours
    takeoffTime: Date;
    estimatedLandingTime: Date;
  };

  waypoints: Waypoint[];

  sensors: {
    eo: boolean;
    ir: boolean;
    sar: boolean;
    lidar: boolean;
    sigint: boolean;
    nbc: boolean;
  };

  contingencies: {
    lostLink: 'RTB' | 'loiter' | 'land';
    lowFuel: 'RTB' | 'land-nearest';
    weatherDivert: GeoCoordinate[];
  };

  approvals: {
    airspaceCleared: boolean;
    weatherAcceptable: boolean;
    commanderApproval: boolean;
    timestamp?: Date;
  };
}

// ============================================================================
// Mission Execution
// ============================================================================

/**
 * Mission status
 */
export type MissionStatus =
  | 'planning'
  | 'approved'
  | 'pre-flight'
  | 'active'
  | 'paused'
  | 'completed'
  | 'aborted'
  | 'failed';

/**
 * Real-time telemetry
 */
export interface Telemetry {
  timestamp: Date;
  position: GeoCoordinate;
  velocity: Vector3; // m/s
  heading: number; // degrees
  altitude: number; // meters MSL
  groundSpeed: number; // knots
  airspeed?: number; // knots
  verticalSpeed: number; // ft/min
  batteryLevel?: number; // percent
  fuelLevel?: number; // liters
  signalStrength: number; // dBm
  linkQuality: number; // percent
}

/**
 * Mission execution state
 */
export interface MissionExecution {
  id: string;
  flightPlan: FlightPlan;
  status: MissionStatus;

  currentState: {
    telemetry: Telemetry;
    currentWaypoint: string;
    nextWaypoint: string;
    distanceToNext: number; // meters
    timeToNext: number; // seconds
    distanceRemaining: number; // km
    timeRemaining: number; // hours
  };

  performance: {
    fuelConsumed?: number; // liters
    batteryUsed?: number; // percent
    distanceTraveled: number; // km
    timeElapsed: number; // hours
  };

  dataCollected: {
    eoImages: number;
    irImages: number;
    sarImages: number;
    lidarScans: number;
    sigintReports: number;
    nbcDetections: number;
    videoHours: number;
  };

  alerts: Alert[];
}

/**
 * Alert/warning during mission
 */
export interface Alert {
  id: string;
  timestamp: Date;
  severity: 'info' | 'warning' | 'error' | 'critical';
  type: string;
  message: string;
  acknowledged: boolean;
  resolvedAt?: Date;
}

// ============================================================================
// Mission Parameters
// ============================================================================

/**
 * Mission parameter calculation input
 */
export interface MissionParameters {
  droneClass: DroneClass;
  distance: number; // km (total route distance)
  loiterTime?: number; // hours (time orbiting target)
  altitude: number; // feet MSL
  speed?: number; // knots (default to cruise speed)
  payloadWeight: number; // kg
  weather?: {
    windSpeed: number; // knots
    windDirection: number; // degrees
    temperature: number; // Celsius
  };
}

/**
 * Mission parameter calculation result
 */
export interface MissionCalculation {
  fuelRequired?: number; // liters
  batteryRequired?: number; // Wh
  totalFlightTime: number; // hours
  maxEndurance: number; // hours
  enduranceMargin: number; // hours (fuel reserve)
  surveillanceArea: number; // km²
  feasibility: 'possible' | 'marginal' | 'impossible';
  warnings: string[];
  recommendations: string[];
}

// ============================================================================
// Defensive Operations
// ============================================================================

/**
 * Border security patrol
 */
export interface BorderPatrol {
  borderSegment: {
    start: GeoCoordinate;
    end: GeoCoordinate;
    length: number; // km
  };
  patternType: 'linear' | 'racetrack' | 'figure-eight';
  altitude: number; // feet AGL
  speed: number; // knots
  sensorCoverage: number; // km (swath width)
  revisitRate: number; // hours
}

/**
 * Force protection configuration
 */
export interface ForceProtection {
  protectedArea: {
    center: GeoCoordinate;
    radius: number; // meters
    polygon?: GeoCoordinate[];
  };
  orbitAltitude: number; // feet AGL
  orbitRadius: number; // meters
  persistence: number; // hours
  threatDetection: {
    vehicles: boolean;
    personnel: boolean;
    weapons: boolean;
    explosives: boolean;
  };
}

// ============================================================================
// Humanitarian Operations
// ============================================================================

/**
 * Disaster response mission
 */
export interface DisasterResponse {
  disasterType: 'earthquake' | 'flood' | 'wildfire' | 'hurricane' | 'tsunami';
  affectedArea: {
    center: GeoCoordinate;
    radius?: number; // km
    polygon?: GeoCoordinate[];
  };
  assessmentType: 'rapid' | 'detailed' | 'ongoing';
  priority: Array<'damage' | 'survivors' | 'infrastructure' | 'hazards'>;
  products: Array<'imagery' | '3D-model' | 'damage-map' | 'survivor-locations'>;
}

/**
 * Search and rescue mission
 */
export interface SearchAndRescue {
  searchArea: {
    center: GeoCoordinate;
    radius?: number; // km
    polygon?: GeoCoordinate[];
  };
  targetType: 'person' | 'vehicle' | 'aircraft' | 'vessel';
  searchPattern: 'expanding-square' | 'parallel-track' | 'sector';
  altitude: number; // feet AGL
  sensorMode: 'EO' | 'IR' | 'both';
  survivors: Array<{
    id: string;
    location: GeoCoordinate;
    detectionTime: Date;
    confidence: number; // 0-1
    status?: 'alive' | 'injured' | 'deceased' | 'unknown';
  }>;
}

/**
 * Medical delivery mission
 */
export interface MedicalDelivery {
  origin: GeoCoordinate;
  destination: GeoCoordinate;
  deliveryType: 'blood' | 'vaccine' | 'medication' | 'medical-kit' | 'AED';
  weight: number; // kg
  volume: number; // liters
  urgency: 'routine' | 'priority' | 'urgent' | 'emergency';
  temperatureControl?: {
    min: number; // Celsius
    max: number; // Celsius
  };
  dropType: 'landing' | 'parachute' | 'winch';
  recipientContact?: string;
}

// ============================================================================
// Safety & Compliance
// ============================================================================

/**
 * Safety check result
 */
export interface SafetyCheck {
  name: string;
  status: 'pass' | 'warning' | 'fail';
  description: string;
  value?: number;
  threshold?: number;
  correctiveAction?: string;
}

/**
 * Pre-flight checklist
 */
export interface PreFlightChecklist {
  completed: boolean;
  timestamp?: Date;
  operator: string;

  checks: {
    weather: SafetyCheck;
    airspace: SafetyCheck;
    fuel: SafetyCheck;
    communication: SafetyCheck;
    sensors: SafetyCheck;
    autopilot: SafetyCheck;
    geofence: SafetyCheck;
    emergency: SafetyCheck;
  };

  approvedBy?: string;
  approvalTimestamp?: Date;
}

/**
 * Ethical compliance check
 */
export interface EthicalCompliance {
  humanInLoop: boolean; // Required for weapon systems
  civilianProtection: {
    collateralDamageEstimate: number; // civilians at risk
    proportionalityAssessment: 'proportional' | 'disproportionate';
    legalReview: boolean;
  };
  dataPrivacy: {
    collectionJustified: boolean;
    retentionPeriod: number; // days
    accessRestricted: boolean;
    encryptionEnabled: boolean;
  };
  internationalLaw: {
    genevaCompliant: boolean;
    treatyCompliant: boolean;
    rulesOfEngagement: string;
  };
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-DEF-002 error codes
 */
export enum DroneErrorCode {
  INVALID_CONFIGURATION = 'D001',
  INSUFFICIENT_FUEL = 'D002',
  AIRSPACE_VIOLATION = 'D003',
  COMMUNICATION_LOST = 'D004',
  SENSOR_MALFUNCTION = 'D005',
  FLIGHT_PLAN_INVALID = 'D006',
  WEATHER_UNSAFE = 'D007',
  GEOFENCE_VIOLATION = 'D008',
  LOW_BATTERY = 'D009',
  SYSTEM_FAILURE = 'D010',
  ETHICAL_VIOLATION = 'D011',
}

/**
 * Drone error
 */
export class DroneError extends Error {
  constructor(
    public code: DroneErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'DroneError';
  }
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Physical and operational constants
 */
export const DRONE_CONSTANTS = {
  /** Speed of light in m/s */
  SPEED_OF_LIGHT: 299792458,

  /** Standard gravity in m/s² */
  GRAVITY: 9.80665,

  /** Standard atmosphere pressure in Pa */
  STANDARD_PRESSURE: 101325,

  /** Standard temperature in Kelvin */
  STANDARD_TEMPERATURE: 288.15,

  /** Earth radius in meters */
  EARTH_RADIUS: 6371000,

  /** Conversion factors */
  CONVERSIONS: {
    KNOTS_TO_MS: 0.514444,
    FEET_TO_METERS: 0.3048,
    NM_TO_METERS: 1852,
    KG_TO_LBS: 2.20462,
  },

  /** Fuel reserve requirements */
  FUEL_RESERVES: {
    MINIMUM_MINUTES: 30,
    ALTERNATE_MINUTES: 30,
    WEATHER_MINUTES: 45,
  },

  /** Battery safety margins */
  BATTERY: {
    MIN_SAFE_LEVEL: 20, // percent
    WARNING_LEVEL: 30, // percent
    COLD_WEATHER_LOSS: 30, // percent
  },

  /** Communication thresholds */
  COMMUNICATION: {
    MIN_SIGNAL_STRENGTH: -100, // dBm
    WARNING_STRENGTH: -90, // dBm
    MIN_LINK_QUALITY: 50, // percent
  },
} as const;

// ============================================================================
// Export All
// ============================================================================

export type {
  GeoCoordinate,
  Vector3,
  DroneClass,
  MissionType,
  DroneClassification,
  DroneConfiguration,
  EOCamera,
  IRCamera,
  SARSystem,
  LIDARSystem,
  SIGINTSystem,
  NBCDetector,
  SensorConfiguration,
  WeaponSystem,
  CargoConfiguration,
  DataLink,
  CommunicationSystem,
  AutopilotSystem,
  Waypoint,
  FlightPlan,
  MissionStatus,
  Telemetry,
  MissionExecution,
  Alert,
  MissionParameters,
  MissionCalculation,
  BorderPatrol,
  ForceProtection,
  DisasterResponse,
  SearchAndRescue,
  MedicalDelivery,
  SafetyCheck,
  PreFlightChecklist,
  EthicalCompliance,
};

export { DRONE_CONSTANTS, DroneErrorCode, DroneError };
