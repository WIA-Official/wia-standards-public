/**
 * WIA-DEF-014: Nuclear Defense - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Defense Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * Geographic location
 */
export interface GeoLocation {
  /** Latitude in degrees */
  lat: number;

  /** Longitude in degrees */
  lon: number;

  /** Altitude in meters (optional) */
  altitude?: number;

  /** Human-readable description */
  description?: string;
}

/**
 * Alert levels for radiation detection
 */
export type AlertLevel =
  | 'background'    // < 0.1 μSv/h
  | 'elevated'      // 0.1-1 μSv/h
  | 'alert'         // 1-10 μSv/h
  | 'warning'       // 10-100 μSv/h
  | 'danger'        // 100-1000 μSv/h
  | 'critical';     // > 1000 μSv/h

// ============================================================================
// Radiation Detection
// ============================================================================

/**
 * Types of radiation detectors
 */
export type SensorType =
  | 'geiger-mueller'
  | 'scintillation'
  | 'semiconductor'
  | 'ionization-chamber'
  | 'proportional-counter';

/**
 * Radiation sensor configuration
 */
export interface SensorConfig {
  /** Unique sensor identifier */
  id: string;

  /** Sensor type */
  type: SensorType;

  /** Sensor location */
  location: GeoLocation;

  /** Alert threshold in μSv/h */
  threshold: number;

  /** Measurement interval in seconds */
  interval: number;

  /** Enable automatic alerts */
  alertEnabled: boolean;

  /** Calibration date */
  calibrationDate?: Date;
}

/**
 * Isotope identification data
 */
export interface IsotopeData {
  /** Isotope symbol (e.g., "Cs-137") */
  symbol: string;

  /** Atomic number */
  atomicNumber: number;

  /** Mass number */
  massNumber: number;

  /** Energy peak in keV */
  energy: number;

  /** Confidence level (0-1) */
  confidence: number;

  /** Activity in Bq */
  activity?: number;
}

/**
 * Radiation reading from sensor
 */
export interface RadiationReading {
  /** Sensor identifier */
  sensorId: string;

  /** Reading location */
  location: GeoLocation;

  /** Timestamp of reading */
  timestamp: Date;

  /** Dose rate in μSv/h */
  doseRate: number;

  /** Cumulative dose in μSv */
  totalDose: number;

  /** Alert level classification */
  alertLevel: AlertLevel;

  /** Temperature in Celsius */
  temperature?: number;

  /** Identified isotopes */
  isotopes?: IsotopeData[];

  /** Quality indicators */
  quality: {
    /** Signal to noise ratio */
    snr: number;

    /** Measurement uncertainty (%) */
    uncertainty: number;

    /** Battery level (0-1) */
    battery?: number;
  };
}

/**
 * Radiation monitoring parameters
 */
export interface MonitoringParams {
  /** Sensor type to use */
  sensorType: SensorType;

  /** Alert threshold in μSv/h */
  threshold: number;

  /** Measurement interval in seconds */
  interval: number;

  /** Enable automatic alerts */
  alertEnabled: boolean;

  /** Location to monitor */
  location?: GeoLocation;

  /** Duration in seconds (0 = continuous) */
  duration?: number;
}

/**
 * Radiation monitoring result
 */
export interface MonitoringResult {
  /** Current reading */
  current: RadiationReading;

  /** Historical readings */
  history: RadiationReading[];

  /** Statistical summary */
  statistics: {
    mean: number;
    median: number;
    max: number;
    min: number;
    stdDev: number;
  };

  /** Monitoring status */
  status: 'active' | 'paused' | 'stopped' | 'error';

  /** Alerts triggered */
  alerts: RadiationAlert[];
}

/**
 * Radiation alert
 */
export interface RadiationAlert {
  /** Alert identifier */
  id: string;

  /** Alert timestamp */
  timestamp: Date;

  /** Alert level */
  level: AlertLevel;

  /** Dose rate at alert */
  doseRate: number;

  /** Sensor that triggered alert */
  sensorId: string;

  /** Alert location */
  location: GeoLocation;

  /** Alert message */
  message: string;

  /** Recommended actions */
  recommendations: string[];
}

// ============================================================================
// Fallout Protection
// ============================================================================

/**
 * Nuclear event parameters
 */
export interface NuclearEvent {
  /** Event unique identifier */
  id: string;

  /** Weapon yield in kilotons */
  yield: number;

  /** Burst height in meters (0 = surface, > 0 = airburst) */
  height: number;

  /** Detonation location */
  location: GeoLocation;

  /** Event timestamp */
  timestamp: Date;

  /** Fission fraction (0-1) */
  fissionFraction?: number;
}

/**
 * Fallout calculation parameters
 */
export interface FalloutParams extends NuclearEvent {
  /** Wind speed in m/s */
  windSpeed: number;

  /** Wind direction in degrees (0 = North, 90 = East) */
  windDirection: number;

  /** Atmospheric stability class */
  stabilityClass?: 'A' | 'B' | 'C' | 'D' | 'E' | 'F';

  /** Precipitation rate in mm/h */
  precipitation?: number;
}

/**
 * Dose contour for mapping
 */
export interface DoseContour {
  /** Dose rate in Sv/h */
  doseRate: number;

  /** Contour boundary points */
  boundary: GeoLocation[];

  /** Arrival time in minutes */
  arrivalTime: number;
}

/**
 * Fallout prediction result
 */
export interface FalloutPrediction {
  /** Event identifier */
  eventId: string;

  /** Time until fallout arrival in minutes */
  arrivalTime: number;

  /** Peak dose rate in Sv/h */
  peakDoseRate: number;

  /** Total dose over 24 hours in Sv */
  totalDose24h: number;

  /** Dose rate decay over time */
  decayCurve: {
    time: number[];      // hours after detonation
    doseRate: number[];  // Sv/h
  };

  /** Spatial dose distribution */
  contours: DoseContour[];

  /** Shelter recommendation */
  shelterRecommendation: {
    required: boolean;
    minimumPF: number;
    duration: number;    // hours
  };

  /** Affected area in km² */
  affectedArea: number;

  /** Affected population estimate */
  affectedPopulation?: number;
}

// ============================================================================
// Shelter Assessment
// ============================================================================

/**
 * Material specification
 */
export interface MaterialSpec {
  /** Material type */
  material: 'concrete' | 'steel' | 'lead' | 'earth' | 'wood' | 'brick' | 'water';

  /** Thickness in cm */
  thickness: number;

  /** Density in g/cm³ */
  density?: number;

  /** Number of layers */
  layers?: number;
}

/**
 * Shelter type
 */
export type ShelterType =
  | 'improvised'
  | 'basement'
  | 'underground'
  | 'purpose-built'
  | 'mobile';

/**
 * Shelter configuration
 */
export interface ShelterConfig {
  /** Shelter unique identifier */
  id?: string;

  /** Shelter type */
  type: ShelterType;

  /** Shelter dimensions in meters */
  dimensions: {
    length: number;
    width: number;
    height: number;
  };

  /** Wall specifications */
  walls: MaterialSpec;

  /** Roof/overhead protection */
  roof: MaterialSpec;

  /** Floor protection (if applicable) */
  floor?: MaterialSpec;

  /** Depth below ground in meters */
  depth?: number;

  /** Designed capacity (number of people) */
  capacity: number;

  /** Ventilation system */
  ventilation?: {
    type: 'natural' | 'mechanical' | 'filtered';
    flowRate: number;    // m³/h
    filtration: boolean;
  };

  /** Entrance configuration */
  entrance?: {
    type: 'direct' | 'labyrinth' | 'blast-door';
    shielded: boolean;
  };
}

/**
 * Shelter assessment result
 */
export interface ShelterAssessment {
  /** Shelter identifier */
  shelterId: string;

  /** Overall protection factor */
  protectionFactor: number;

  /** Component protection factors */
  components: {
    walls: number;
    roof: number;
    floor: number;
    entrance: number;
  };

  /** Gamma ray attenuation (%) */
  gammaAttenuation: number;

  /** Neutron attenuation (%) */
  neutronAttenuation?: number;

  /** Safe capacity (number of people) */
  safeCapacity: number;

  /** Air supply duration in hours */
  airSupplyHours: number;

  /** Overall rating */
  rating: 'A' | 'B' | 'C' | 'D' | 'F';

  /** Recommendations for improvement */
  recommendations: string[];

  /** Compliance with standards */
  compliance: {
    standard: string;
    compliant: boolean;
    deficiencies: string[];
  }[];
}

// ============================================================================
// EMP Protection
// ============================================================================

/**
 * Shielding types for EMP protection
 */
export type ShieldingType =
  | 'faraday-cage'
  | 'metal-enclosure'
  | 'underground'
  | 'waveguide'
  | 'none';

/**
 * EMP protection configuration
 */
export interface EMPConfig {
  /** Facility or system identifier */
  id?: string;

  /** Facility type */
  facilityType: 'data-center' | 'power-grid' | 'communication' | 'military' | 'residential';

  /** Shielding configuration */
  shielding: {
    type: ShieldingType;
    material: 'copper' | 'aluminum' | 'steel' | 'copper-mesh' | 'concrete';
    thickness: number;    // mm
    grounding: boolean;
  };

  /** Critical systems */
  criticalSystems: {
    id: string;
    type: string;
    hardened: boolean;
    backupPower: boolean;
  }[];

  /** Surge protection */
  surgeProtection: boolean;

  /** Fiber optic connections (immune to EMP) */
  fiberOptics: boolean;
}

/**
 * EMP protection assessment result
 */
export interface EMPAssessment {
  /** Facility identifier */
  facilityId: string;

  /** Shielding effectiveness in dB */
  shieldingDB: number;

  /** E1 protection level (kV/m survivable) */
  e1Protection: number;

  /** E2 protection level */
  e2Protection: number;

  /** E3 protection level */
  e3Protection: number;

  /** Number of protected systems */
  protectedCount: number;

  /** Total number of systems */
  totalCount: number;

  /** Vulnerability score (0-100, lower is better) */
  vulnerabilityScore: number;

  /** Overall protection level */
  protectionLevel: 'none' | 'minimal' | 'moderate' | 'high' | 'military-grade';

  /** Recommendations */
  recommendations: string[];

  /** Estimated cost for improvements */
  improvementCost?: number;
}

// ============================================================================
// Decontamination
// ============================================================================

/**
 * Contamination assessment
 */
export interface ContaminationAssessment {
  /** Assessment identifier */
  id: string;

  /** Location assessed */
  location: GeoLocation;

  /** Timestamp */
  timestamp: Date;

  /** Surface contamination in Bq/cm² */
  surfaceContamination: number;

  /** Contaminated area in m² */
  area: number;

  /** Predominant isotopes */
  isotopes: IsotopeData[];

  /** Contamination type */
  type: 'fixed' | 'loose' | 'mixed';

  /** Removability (0-1) */
  removability: number;
}

/**
 * Decontamination method
 */
export type DeconMethod =
  | 'water-wash'
  | 'chemical-decon'
  | 'vacuum'
  | 'stripping'
  | 'fixative'
  | 'removal';

/**
 * Decontamination protocol
 */
export interface DeconProtocol {
  /** Protocol identifier */
  id?: string;

  /** Target area in m² */
  area: number;

  /** Initial contamination level in Bq/cm² */
  initialContamination: number;

  /** Target contamination level */
  targetContamination: number;

  /** Decontamination method */
  method: DeconMethod;

  /** Isotopes to remove */
  isotopes?: string[];

  /** Surface type */
  surfaceType: 'hard' | 'soft' | 'porous' | 'soil' | 'water';

  /** Time available in hours */
  timeAvailable?: number;
}

/**
 * Decontamination result
 */
export interface DeconResult {
  /** Protocol identifier */
  protocolId: string;

  /** Decontamination factor achieved */
  deconFactor: number;

  /** Final contamination level in Bq/cm² */
  finalContamination: number;

  /** Time required in hours */
  timeRequired: number;

  /** Personnel required */
  personnelRequired: number;

  /** Water required in liters */
  waterRequired?: number;

  /** Waste generated in m³ */
  wasteGenerated: number;

  /** Success rate (%) */
  successRate: number;

  /** Recommendations */
  recommendations: string[];
}

// ============================================================================
// Emergency Response
// ============================================================================

/**
 * Emergency scenario type
 */
export type EmergencyScenario =
  | 'nuclear-detonation'
  | 'dirty-bomb'
  | 'nuclear-facility-accident'
  | 'lost-source'
  | 'transportation-accident';

/**
 * Emergency response configuration
 */
export interface EmergencyConfig {
  /** Scenario type */
  scenario: EmergencyScenario;

  /** Event location */
  location: GeoLocation;

  /** Affected population */
  population: number;

  /** Available resources */
  resources: {
    firstResponders: number;
    medicalPersonnel: number;
    deconStations: number;
    shelters: number;
  };

  /** Time since event in minutes */
  timeSinceEvent: number;
}

/**
 * Emergency response plan
 */
export interface EmergencyResponse {
  /** Response identifier */
  id: string;

  /** Response phase */
  phase: 'immediate' | 'intermediate' | 'recovery';

  /** Priority actions */
  actions: {
    priority: number;
    action: string;
    deadline: number;    // minutes
    responsible: string;
    status: 'pending' | 'in-progress' | 'completed';
  }[];

  /** Resource allocation */
  allocation: {
    resource: string;
    quantity: number;
    location: GeoLocation;
  }[];

  /** Evacuation zones */
  evacuationZones: {
    zone: string;
    radius: number;      // km
    population: number;
    priority: number;
    status: 'not-started' | 'in-progress' | 'complete';
  }[];

  /** Estimated casualties */
  casualties: {
    immediate: number;
    delayed: number;
    total: number;
  };

  /** Timeline */
  timeline: {
    event: string;
    time: number;        // minutes from start
    completed: boolean;
  }[];
}

// ============================================================================
// Medical Response
// ============================================================================

/**
 * Radiation dose estimate
 */
export interface DoseEstimate {
  /** Patient identifier */
  patientId: string;

  /** Estimated dose in Gy */
  dose: number;

  /** Confidence interval */
  confidence: {
    lower: number;
    upper: number;
    level: number;       // e.g., 0.95 for 95%
  };

  /** Estimation method */
  method: 'biodosimetry' | 'physical-dosimetry' | 'clinical-symptoms' | 'reconstruction';

  /** Time of exposure */
  exposureTime: Date;

  /** Body region */
  region: 'whole-body' | 'partial-body' | 'local';
}

/**
 * Acute Radiation Syndrome classification
 */
export type ARSSeverity = 'mild' | 'moderate' | 'severe' | 'very-severe' | 'lethal';

/**
 * Medical triage category
 */
export type TriageCategory = 'immediate' | 'delayed' | 'minimal' | 'expectant';

/**
 * Patient medical assessment
 */
export interface MedicalAssessment {
  /** Patient identifier */
  patientId: string;

  /** Dose estimate */
  doseEstimate: DoseEstimate;

  /** ARS severity */
  arsSeverity: ARSSeverity;

  /** Triage category */
  triageCategory: TriageCategory;

  /** Clinical symptoms */
  symptoms: {
    symptom: string;
    onset: number;       // hours after exposure
    severity: 'none' | 'mild' | 'moderate' | 'severe';
  }[];

  /** Laboratory results */
  labs?: {
    lymphocytes: number;  // cells/μL
    neutrophils: number;
    platelets: number;
    timestamp: Date;
  };

  /** Treatment recommendations */
  treatment: {
    medication: string;
    dose: string;
    frequency: string;
    duration: string;
  }[];

  /** Prognosis */
  prognosis: {
    survival: number;    // probability (0-1)
    expectedRecovery: number;  // days
    longTermEffects: string[];
  };
}

// ============================================================================
// System Configuration
// ============================================================================

/**
 * Nuclear defense system configuration
 */
export interface NuclearDefenseConfig {
  /** System identifier */
  systemId: string;

  /** Radiation monitoring network */
  monitoring: {
    sensors: SensorConfig[];
    updateInterval: number;
    alertThreshold: number;
  };

  /** Warning system */
  warning: {
    enabled: boolean;
    channels: ('eas' | 'wea' | 'siren' | 'social-media')[];
    notificationDelay: number;  // seconds
  };

  /** Shelter network */
  shelters: ShelterConfig[];

  /** Emergency response */
  emergency: {
    coordinationCenter: GeoLocation;
    responseTeams: number;
    deconCapacity: number;  // people per hour
  };

  /** Medical resources */
  medical: {
    hospitals: number;
    kiStockpile: number;  // doses
    specializedBeds: number;
  };
}

/**
 * System status
 */
export interface SystemStatus {
  /** Overall status */
  status: 'nominal' | 'alert' | 'emergency' | 'offline';

  /** Active sensors */
  activeSensors: number;

  /** Total sensors */
  totalSensors: number;

  /** Current alerts */
  activeAlerts: number;

  /** Shelter occupancy */
  shelterOccupancy: {
    shelterId: string;
    capacity: number;
    occupied: number;
    available: number;
  }[];

  /** Last update */
  lastUpdate: Date;

  /** System health (0-1) */
  health: number;
}

// ============================================================================
// Validation and Analysis
// ============================================================================

/**
 * Validation parameters
 */
export interface ValidationParams {
  /** System configuration to validate */
  config: NuclearDefenseConfig;

  /** Scenario to validate against */
  scenario: EmergencyConfig;

  /** Compliance standards */
  standards?: string[];

  /** Population coverage requirement (0-1) */
  coverageRequirement?: number;
}

/**
 * Validation result
 */
export interface ValidationResult {
  /** Is system valid */
  isValid: boolean;

  /** Validation errors */
  errors: string[];

  /** Validation warnings */
  warnings: string[];

  /** Coverage analysis */
  coverage: {
    population: number;     // % covered
    geographic: number;     // % area covered
    responseTime: number;   // average in minutes
  };

  /** Capacity analysis */
  capacity: {
    shelter: number;        // people
    decontamination: number; // people per hour
    medical: number;        // patients
  };

  /** Recommendations */
  recommendations: string[];

  /** Overall readiness score (0-100) */
  readinessScore: number;

  /** Compliance */
  compliance: {
    standard: string;
    compliant: boolean;
    deficiencies: string[];
  }[];
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Physical and safety constants
 */
export const NUCLEAR_DEFENSE_CONSTANTS = {
  /** Background radiation (μSv/h) */
  BACKGROUND_RADIATION: 0.1,

  /** Public annual dose limit (mSv) */
  PUBLIC_DOSE_LIMIT: 1.0,

  /** Occupational annual dose limit (mSv) */
  OCCUPATIONAL_DOSE_LIMIT: 50.0,

  /** Emergency responder single event limit (mSv) */
  EMERGENCY_DOSE_LIMIT: 100.0,

  /** LD50/30 dose (Gy) - 50% fatality in 30 days */
  LD50_30: 4.5,

  /** Minimum shelter protection factor */
  MIN_SHELTER_PF: 10,

  /** Recommended shelter protection factor */
  RECOMMENDED_SHELTER_PF: 40,

  /** Target decontamination factor for personnel */
  PERSONNEL_DECON_FACTOR: 10,

  /** Target decontamination factor for equipment */
  EQUIPMENT_DECON_FACTOR: 100,

  /** EMP shielding effectiveness target (dB) */
  EMP_SHIELDING_TARGET: 80,

  /** Alert dissemination time target (seconds) */
  ALERT_TIME_TARGET: 60,

  /** Shelter time recommendation (hours) */
  SHELTER_TIME_HOURS: 48,

  /** KI effectiveness window (hours) */
  KI_EFFECTIVENESS_HOURS: 6,
} as const;

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Result type for operations
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
 * WIA-DEF-014 error codes
 */
export enum DefenseErrorCode {
  SENSOR_MALFUNCTION = 'D001',
  COMMUNICATION_FAILURE = 'D002',
  DATA_VALIDATION_ERROR = 'D003',
  ALERT_THRESHOLD_EXCEEDED = 'D004',
  SHELTER_CAPACITY_EXCEEDED = 'D005',
  DECON_RESOURCES_EXHAUSTED = 'D006',
  MEDICAL_UNAVAILABLE = 'D007',
  INVALID_PARAMETERS = 'D008',
  SYSTEM_OFFLINE = 'D009',
  CALIBRATION_REQUIRED = 'D010',
}

/**
 * Nuclear defense system error
 */
export class NuclearDefenseError extends Error {
  constructor(
    public code: DefenseErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'NuclearDefenseError';
  }
}

// ============================================================================
// Export All Types
// ============================================================================

export type {
  // Core
  GeoLocation,
  AlertLevel,

  // Detection
  SensorType,
  SensorConfig,
  IsotopeData,
  RadiationReading,
  MonitoringParams,
  MonitoringResult,
  RadiationAlert,

  // Fallout
  NuclearEvent,
  FalloutParams,
  DoseContour,
  FalloutPrediction,

  // Shelter
  MaterialSpec,
  ShelterType,
  ShelterConfig,
  ShelterAssessment,

  // EMP
  ShieldingType,
  EMPConfig,
  EMPAssessment,

  // Decontamination
  ContaminationAssessment,
  DeconMethod,
  DeconProtocol,
  DeconResult,

  // Emergency
  EmergencyScenario,
  EmergencyConfig,
  EmergencyResponse,

  // Medical
  DoseEstimate,
  ARSSeverity,
  TriageCategory,
  MedicalAssessment,

  // System
  NuclearDefenseConfig,
  SystemStatus,

  // Validation
  ValidationParams,
  ValidationResult,
};

export { NUCLEAR_DEFENSE_CONSTANTS, DefenseErrorCode, NuclearDefenseError };
