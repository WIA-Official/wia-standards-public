/**
 * WIA-TIME-022: Emergency Retrieval - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Time Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Geometry Types
// ============================================================================

/**
 * Three-dimensional spatial coordinates
 */
export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

/**
 * Spacetime coordinates (4D)
 */
export interface SpacetimeCoordinates {
  /** Spatial position in meters */
  position: Vector3;

  /** Temporal coordinate */
  time: Date | string;

  /** Reference frame identifier */
  referenceFrame?: string;
}

// ============================================================================
// Emergency Classification
// ============================================================================

/**
 * Emergency severity levels
 */
export type EmergencySeverity = 'CRITICAL' | 'HIGH' | 'MEDIUM' | 'LOW';

/**
 * Emergency type categories
 */
export type EmergencyType =
  | 'ENERGY_DEPLETION'
  | 'TEMPORAL_FIELD_FAILURE'
  | 'MEDICAL_EMERGENCY'
  | 'PARADOX_CONTAMINATION'
  | 'EQUIPMENT_MALFUNCTION'
  | 'ENVIRONMENTAL_HAZARD'
  | 'HOSTILE_ENCOUNTER'
  | 'COMMUNICATION_LOSS'
  | 'DISPLACEMENT_SICKNESS'
  | 'OTHER';

/**
 * Emergency activation method
 */
export type ActivationMethod = 'AUTO' | 'MANUAL_BUTTON' | 'MANUAL_VOICE' | 'MANUAL_GESTURE' | 'THIRD_PARTY';

// ============================================================================
// Distress Signal
// ============================================================================

/**
 * Distress signal transmission
 */
export interface DistressSignal {
  /** Unique distress signal identifier */
  id: string;

  /** Casualty identifier */
  casualtyId: string;

  /** Emergency severity */
  severity: EmergencySeverity;

  /** Emergency type */
  type: EmergencyType;

  /** Current spacetime position */
  position: SpacetimeCoordinates;

  /** Position uncertainty (meters) */
  positionUncertainty?: number;

  /** Temporal uncertainty (seconds) */
  temporalUncertainty?: number;

  /** Vital signs if available */
  vitals?: VitalSigns;

  /** System status */
  systemStatus: SystemStatus;

  /** Distress message */
  message: string;

  /** How was emergency activated? */
  activationMethod: ActivationMethod;

  /** Activation timestamp */
  activatedAt: Date;

  /** Signal transmission details */
  signal: SignalTransmission;

  /** Contact information */
  contact?: ContactInfo;

  /** Additional metadata */
  metadata?: Record<string, unknown>;
}

/**
 * Signal transmission parameters
 */
export interface SignalTransmission {
  /** Transmission frequency (Hz) */
  frequency: number;

  /** Signal power (watts) */
  power: number;

  /** Transmission rate (Hz) */
  rate: number;

  /** Estimated range (spatial, meters) */
  spatialRange: number;

  /** Estimated range (temporal, seconds) */
  temporalRange: number;

  /** Signal-to-noise ratio */
  SNR?: number;

  /** Detected by beacons */
  detectedBy?: string[];
}

// ============================================================================
// Medical Information
// ============================================================================

/**
 * Vital signs monitoring
 */
export interface VitalSigns {
  /** Measurement timestamp */
  timestamp: Date;

  /** Heart rate (bpm) */
  heartRate?: number;

  /** Blood pressure */
  bloodPressure?: {
    systolic: number;
    diastolic: number;
  };

  /** Respiratory rate (breaths/min) */
  respiratoryRate?: number;

  /** Body temperature (Celsius) */
  temperature?: number;

  /** Oxygen saturation (%) */
  oxygenSaturation?: number;

  /** Consciousness level */
  consciousness?: 'alert' | 'verbal' | 'pain' | 'unresponsive';

  /** Temporal stability (0-1) */
  temporalStability?: number;

  /** Pain level (0-10) */
  painLevel?: number;

  /** Additional notes */
  notes?: string;
}

/**
 * System status for emergency
 */
export interface SystemStatus {
  /** Energy remaining (%) */
  energyLevel: number;

  /** Temporal field stability (0-1) */
  fieldStability?: number;

  /** Communication capability (0-1) */
  communicationCapability?: number;

  /** Equipment failures */
  failures?: EquipmentFailure[];

  /** Paradox contamination level (0-1) */
  paradoxLevel?: number;

  /** Estimated time until critical failure (seconds) */
  timeToFailure?: number;

  /** Diagnostics data */
  diagnostics?: Record<string, unknown>;
}

/**
 * Equipment failure description
 */
export interface EquipmentFailure {
  /** Component name */
  component: string;

  /** Failure type */
  type: 'complete' | 'partial' | 'degraded';

  /** Severity impact */
  severity: 'critical' | 'major' | 'minor';

  /** Failure timestamp */
  occurredAt: Date;

  /** Description */
  description?: string;
}

/**
 * Contact information
 */
export interface ContactInfo {
  /** Casualty name */
  name?: string;

  /** Emergency contact person */
  emergencyContact?: string;

  /** Time traveler ID */
  travelerId?: string;

  /** Home time period */
  homeTime?: Date | string;

  /** Organization/affiliation */
  organization?: string;

  /** Medical conditions */
  medicalConditions?: string[];

  /** Allergies */
  allergies?: string[];

  /** Blood type */
  bloodType?: string;
}

/**
 * Medical emergency category
 */
export type MedicalEmergencyCategory =
  | 'TEMPORAL_DISPLACEMENT_SICKNESS'
  | 'TEMPORAL_RADIATION_EXPOSURE'
  | 'PARADOX_CONTAMINATION'
  | 'CARDIAC_ARREST'
  | 'RESPIRATORY_FAILURE'
  | 'TRAUMA'
  | 'ENVIRONMENTAL'
  | 'NEUROLOGICAL'
  | 'OTHER';

/**
 * Medical assessment
 */
export interface MedicalAssessment {
  /** Assessment timestamp */
  timestamp: Date;

  /** Assessed by */
  assessedBy: string;

  /** Primary condition */
  primaryCondition: MedicalEmergencyCategory;

  /** Secondary conditions */
  secondaryConditions?: MedicalEmergencyCategory[];

  /** ABCDE assessment */
  abcde: {
    airway: 'patent' | 'obstructed' | 'compromised';
    breathing: 'adequate' | 'inadequate' | 'absent';
    circulation: 'stable' | 'unstable' | 'absent';
    disability: 'alert' | 'verbal' | 'pain' | 'unresponsive';
    exposure: string; // Environmental findings
  };

  /** Temporal assessment */
  temporal: {
    displacementSickness: 'none' | 'mild' | 'moderate' | 'severe';
    radiationExposure: 'none' | 'low' | 'moderate' | 'high' | 'critical';
    paradoxContamination: 'none' | 'suspected' | 'confirmed' | 'severe';
  };

  /** Treatment priority */
  priority: 'immediate' | 'urgent' | 'delayed' | 'expectant';

  /** Notes */
  notes?: string;
}

/**
 * Medical treatment record
 */
export interface MedicalTreatment {
  /** Treatment timestamp */
  timestamp: Date;

  /** Administered by */
  administeredBy: string;

  /** Treatment type */
  type: 'medication' | 'procedure' | 'intervention' | 'monitoring';

  /** Description */
  description: string;

  /** Dosage/details */
  details?: string;

  /** Patient response */
  response?: string;

  /** Complications */
  complications?: string;
}

// ============================================================================
// Rescue Mission
// ============================================================================

/**
 * Rescue mission status
 */
export type RescueMissionStatus =
  | 'ALERT'
  | 'MOBILIZING'
  | 'DEPLOYING'
  | 'ON_SCENE'
  | 'EXTRACTING'
  | 'RETURNING'
  | 'COMPLETE'
  | 'ABORTED';

/**
 * Rescue mission
 */
export interface RescueMission {
  /** Unique mission identifier */
  id: string;

  /** Associated distress signal */
  distressSignalId: string;

  /** Mission status */
  status: RescueMissionStatus;

  /** Deployed teams */
  teams: RescueTeam[];

  /** Mission coordinator */
  coordinator: string;

  /** Mission start time */
  startTime: Date;

  /** Estimated arrival time (seconds) */
  estimatedArrival?: number;

  /** Actual arrival time */
  arrivalTime?: Date;

  /** Extraction time */
  extractionTime?: Date;

  /** Completion time */
  completionTime?: Date;

  /** Mission outcome */
  outcome?: MissionOutcome;

  /** Timeline impact */
  timelineImpact?: TimelineImpact;

  /** Mission log */
  log: MissionLogEntry[];

  /** Resources used */
  resources?: ResourceUsage;
}

/**
 * Rescue team
 */
export interface RescueTeam {
  /** Team identifier */
  id: string;

  /** Team name */
  name: string;

  /** Team members */
  members: TeamMember[];

  /** Readiness level */
  readinessLevel: 'IMMEDIATE' | 'QUICK' | 'STANDARD' | 'RESERVE';

  /** Current position */
  position?: SpacetimeCoordinates;

  /** Specializations */
  specializations: TeamSpecialization[];

  /** Equipment */
  equipment: string[];

  /** Team status */
  status: 'READY' | 'DEPLOYED' | 'ON_MISSION' | 'RECOVERING' | 'UNAVAILABLE';

  /** Deployment history */
  deploymentHistory?: DeploymentHistory[];
}

/**
 * Team member
 */
export interface TeamMember {
  /** Member ID */
  id: string;

  /** Name */
  name: string;

  /** Role */
  role: 'LEADER' | 'NAVIGATOR' | 'MEDIC' | 'SIGNAL_SPECIALIST' | 'SECURITY' | 'SUPPORT';

  /** Certifications */
  certifications: string[];

  /** Experience level */
  experienceLevel: 'JUNIOR' | 'INTERMEDIATE' | 'SENIOR' | 'EXPERT';

  /** Current status */
  status: 'AVAILABLE' | 'ON_MISSION' | 'OFF_DUTY' | 'TRAINING';
}

/**
 * Team specialization
 */
export type TeamSpecialization =
  | 'MEDICAL'
  | 'TEMPORAL_ENGINEERING'
  | 'PARADOX_SPECIALIST'
  | 'COMBAT_RESCUE'
  | 'WILDERNESS'
  | 'URBAN'
  | 'MARITIME'
  | 'TIMELINE_REPAIR'
  | 'HAZMAT'
  | 'TACTICAL';

/**
 * Deployment history entry
 */
export interface DeploymentHistory {
  /** Mission ID */
  missionId: string;

  /** Deployment date */
  date: Date;

  /** Duration (seconds) */
  duration: number;

  /** Outcome */
  outcome: 'SUCCESS' | 'PARTIAL_SUCCESS' | 'FAILURE' | 'ABORTED';

  /** Notes */
  notes?: string;
}

/**
 * Mission outcome
 */
export interface MissionOutcome {
  /** Success status */
  success: boolean;

  /** Casualty status */
  casualtyStatus: 'RESCUED_ALIVE' | 'RESCUED_INJURED' | 'RESCUED_CRITICAL' | 'DECEASED' | 'NOT_FOUND';

  /** Team casualties */
  teamCasualties?: number;

  /** Summary */
  summary: string;

  /** Complications */
  complications?: string[];

  /** Lessons learned */
  lessonsLearned?: string[];
}

/**
 * Timeline impact assessment
 */
export interface TimelineImpact {
  /** Impact level */
  level: 'NONE' | 'NEGLIGIBLE' | 'MINOR' | 'MODERATE' | 'MAJOR' | 'CRITICAL';

  /** Paradox risk */
  paradoxRisk: number; // 0-1

  /** Contamination events */
  contaminationEvents: TimelineContamination[];

  /** Repair required? */
  repairRequired: boolean;

  /** Repair status */
  repairStatus?: 'NOT_STARTED' | 'IN_PROGRESS' | 'COMPLETED' | 'IMPOSSIBLE';

  /** Assessment notes */
  notes?: string;
}

/**
 * Timeline contamination event
 */
export interface TimelineContamination {
  /** Event timestamp */
  timestamp: Date;

  /** Contamination type */
  type: 'TECHNOLOGY_EXPOSURE' | 'INFORMATION_LEAK' | 'PHYSICAL_ARTIFACT' | 'WITNESSED_DISPLACEMENT' | 'HISTORICAL_FIGURE_CONTACT' | 'OTHER';

  /** Severity */
  severity: number; // 0-1

  /** Description */
  description: string;

  /** Witnesses */
  witnesses?: number;

  /** Mitigation taken */
  mitigation?: string;
}

/**
 * Mission log entry
 */
export interface MissionLogEntry {
  /** Log timestamp */
  timestamp: Date;

  /** Event type */
  type: 'STATUS_UPDATE' | 'POSITION_UPDATE' | 'COMMUNICATION' | 'MEDICAL' | 'TIMELINE' | 'COMPLICATION' | 'MILESTONE';

  /** Logged by */
  loggedBy: string;

  /** Message */
  message: string;

  /** Severity */
  severity?: 'INFO' | 'WARNING' | 'ERROR' | 'CRITICAL';

  /** Associated data */
  data?: Record<string, unknown>;
}

/**
 * Resource usage tracking
 */
export interface ResourceUsage {
  /** Temporal displacement jumps */
  jumps: number;

  /** Energy consumed (joules) */
  energyConsumed: number;

  /** Medical supplies used */
  medicalSupplies?: string[];

  /** Equipment consumed/lost */
  equipmentLost?: string[];

  /** Personnel hours */
  personnelHours: number;

  /** Estimated cost */
  estimatedCost?: number;
}

// ============================================================================
// Search and Rescue
// ============================================================================

/**
 * Search mission
 */
export interface SearchMission {
  /** Search mission ID */
  id: string;

  /** Related distress signal (if any) */
  distressSignalId?: string;

  /** Missing person ID */
  missingPersonId: string;

  /** Last known location */
  lastKnownLocation: SpacetimeCoordinates;

  /** Last known location uncertainty */
  locationUncertainty: {
    spatial: number; // meters
    temporal: number; // seconds
  };

  /** Search area */
  searchArea: SearchArea;

  /** Search pattern */
  searchPattern: SearchPattern;

  /** Search teams deployed */
  teams: RescueTeam[];

  /** Search status */
  status: 'PLANNING' | 'ACTIVE' | 'SUSPENDED' | 'COMPLETED' | 'TERMINATED';

  /** Start time */
  startTime: Date;

  /** End time */
  endTime?: Date;

  /** Search result */
  result?: SearchResult;
}

/**
 * Search area definition
 */
export interface SearchArea {
  /** Center point */
  center: SpacetimeCoordinates;

  /** Spatial radius (meters) */
  spatialRadius: number;

  /** Temporal range (seconds) */
  temporalRange: {
    past: number;
    future: number;
  };

  /** Priority zones */
  priorityZones?: PriorityZone[];
}

/**
 * Priority zone for search
 */
export interface PriorityZone {
  /** Zone center */
  center: SpacetimeCoordinates;

  /** Zone radius */
  radius: number;

  /** Priority level */
  priority: 'HIGH' | 'MEDIUM' | 'LOW';

  /** Probability of finding casualty */
  probability: number; // 0-1

  /** Reason for priority */
  reason?: string;
}

/**
 * Search pattern type
 */
export type SearchPattern =
  | 'EXPANDING_GRID'
  | 'TEMPORAL_SWEEP'
  | 'PROBABILITY_WEIGHTED'
  | 'SPIRAL'
  | 'SECTOR'
  | 'PARALLEL_TRACK';

/**
 * Search result
 */
export interface SearchResult {
  /** Found or not? */
  found: boolean;

  /** If found, location */
  foundLocation?: SpacetimeCoordinates;

  /** If found, condition */
  foundCondition?: 'ALIVE' | 'INJURED' | 'DECEASED';

  /** Search duration */
  duration: number; // seconds

  /** Area covered */
  areaCovered: {
    spatial: number; // km²
    temporal: number; // years
  };

  /** Notes */
  notes?: string;
}

// ============================================================================
// Emergency Response System
// ============================================================================

/**
 * Emergency response configuration
 */
export interface EmergencyResponseConfig {
  /** Network ID */
  networkId: string;

  /** Network name */
  name: string;

  /** Coverage area */
  coverage: 'GLOBAL' | 'CONTINENTAL' | 'REGIONAL' | 'LOCAL';

  /** Temporal range */
  temporalRange: {
    start: Date | string;
    end: Date | string;
  };

  /** Available rescue teams */
  teams: RescueTeam[];

  /** Medical facilities */
  medicalFacilities?: MedicalFacility[];

  /** Maximum response time (seconds) */
  maxResponseTime: number;

  /** Emergency hotline */
  hotline?: string;

  /** 24/7 operation */
  alwaysOperational: boolean;
}

/**
 * Medical facility
 */
export interface MedicalFacility {
  /** Facility ID */
  id: string;

  /** Facility name */
  name: string;

  /** Location */
  location: SpacetimeCoordinates;

  /** Facility type */
  type: 'EMERGENCY_ROOM' | 'ICU' | 'CLINIC' | 'FIELD_HOSPITAL' | 'SPECIALIZED';

  /** Capabilities */
  capabilities: string[];

  /** Bed capacity */
  capacity: number;

  /** Current occupancy */
  occupancy?: number;

  /** Temporal medicine capable? */
  temporalMedicineCapable: boolean;
}

/**
 * Emergency response statistics
 */
export interface EmergencyResponseStats {
  /** Reporting period */
  period: {
    start: Date;
    end: Date;
  };

  /** Total emergencies */
  totalEmergencies: number;

  /** By severity */
  bySeverity: {
    critical: number;
    high: number;
    medium: number;
    low: number;
  };

  /** By type */
  byType: Record<EmergencyType, number>;

  /** Outcomes */
  outcomes: {
    rescued: number;
    deceased: number;
    notFound: number;
    falseAlarm: number;
  };

  /** Average response time */
  avgResponseTime: {
    critical: number;
    high: number;
    medium: number;
    low: number;
  };

  /** Timeline impact */
  timelineImpact: {
    none: number;
    minor: number;
    moderate: number;
    major: number;
  };
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Emergency retrieval constants
 */
export const EMERGENCY_CONSTANTS = {
  /** Emergency signal frequencies (Hz) */
  FREQUENCIES: {
    PRIMARY: 10e12, // 10 THz
    SECONDARY: 9.95e12, // 9.95 THz
    TERTIARY: 10.05e12, // 10.05 THz
    LEGACY: 5e12, // 5 THz
  },

  /** Response time targets (seconds) */
  RESPONSE_TIMES: {
    CRITICAL: 300, // 5 minutes
    HIGH: 1800, // 30 minutes
    MEDIUM: 7200, // 2 hours
    LOW: 86400, // 24 hours
  },

  /** Signal transmission rates (Hz) */
  TRANSMISSION_RATES: {
    CRITICAL: 10, // 10 Hz
    HIGH: 1, // 1 Hz
    MEDIUM: 0.1, // 0.1 Hz
    LOW: 0.01, // 0.01 Hz
  },

  /** Signal power levels (watts) */
  SIGNAL_POWER: {
    MAXIMUM: 1e17,
    TYPICAL: 1e15,
    LOW_POWER: 1e12,
  },

  /** Energy depletion thresholds (%) */
  ENERGY_THRESHOLDS: {
    AUTO_EMERGENCY: 5,
    WARNING: 10,
    LOW: 20,
  },

  /** Paradox risk thresholds */
  PARADOX_THRESHOLDS: {
    SAFE: 0.1,
    CAUTION: 0.3,
    DANGER: 0.6,
  },

  /** Search area expansion rate (m/s) */
  SEARCH_EXPANSION_RATE: 1000,

  /** Minimum search team size */
  MIN_TEAM_SIZE: 3,

  /** Optimal search team size */
  OPTIMAL_TEAM_SIZE: 5,

  /** Minimum beacons for triangulation */
  MIN_BEACONS: 4,
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
 * WIA-TIME-022 error codes
 */
export enum EmergencyErrorCode {
  INVALID_DISTRESS_SIGNAL = 'E001',
  NO_TEAMS_AVAILABLE = 'E002',
  POSITION_UNCERTAIN = 'E003',
  TIMELINE_UNSAFE = 'E004',
  PARADOX_RISK_TOO_HIGH = 'E005',
  COMMUNICATION_FAILURE = 'E006',
  EXTRACTION_FAILED = 'E007',
  MEDICAL_CRITICAL = 'E008',
  SEARCH_TIMEOUT = 'E009',
  CASUALTY_NOT_FOUND = 'E010',
  AUTHORIZATION_DENIED = 'E011',
}

/**
 * Emergency error class
 */
export class EmergencyError extends Error {
  constructor(
    public code: EmergencyErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'EmergencyError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  // Core types
  Vector3,
  SpacetimeCoordinates,

  // Emergency classification
  EmergencySeverity,
  EmergencyType,
  ActivationMethod,

  // Distress signal
  DistressSignal,
  SignalTransmission,

  // Medical
  VitalSigns,
  SystemStatus,
  EquipmentFailure,
  ContactInfo,
  MedicalEmergencyCategory,
  MedicalAssessment,
  MedicalTreatment,

  // Rescue mission
  RescueMissionStatus,
  RescueMission,
  RescueTeam,
  TeamMember,
  TeamSpecialization,
  DeploymentHistory,
  MissionOutcome,
  TimelineImpact,
  TimelineContamination,
  MissionLogEntry,
  ResourceUsage,

  // Search and rescue
  SearchMission,
  SearchArea,
  PriorityZone,
  SearchPattern,
  SearchResult,

  // Emergency response system
  EmergencyResponseConfig,
  MedicalFacility,
  EmergencyResponseStats,
};

export { EMERGENCY_CONSTANTS, EmergencyErrorCode, EmergencyError };
