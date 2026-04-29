/**
 * WIA-TIME-021: Return Protocol - TypeScript Type Definitions
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
  /** Spatial coordinates in meters */
  position: Vector3;

  /** Temporal coordinate */
  time: Date | string;

  /** Reference frame identifier */
  referenceFrame?: string;
}

// ============================================================================
// Origin Lock Types
// ============================================================================

/**
 * Origin lock status
 */
export type OriginLockStatus =
  | 'active'
  | 'degrading'
  | 'critical'
  | 'expired'
  | 'failed';

/**
 * Lock strength quality level
 */
export type LockStrengthLevel =
  | 'excellent' // 99.9-100%
  | 'good' // 95-99.9%
  | 'fair' // 90-95%
  | 'poor' // 80-90%
  | 'critical'; // <80%

/**
 * Quantum signature for traveler identification
 */
export interface QuantumSignature {
  /** Biological quantum state hash */
  biological: string;

  /** Consciousness quantum state hash */
  consciousness: string;

  /** Personal timeline state hash */
  timeline: string;

  /** Signature captured timestamp */
  capturedAt: Date;

  /** Signature strength (0-1) */
  strength: number;
}

/**
 * Origin lock configuration
 */
export interface OriginLock {
  /** Unique lock identifier */
  id: string;

  /** Traveler identifier */
  travelerID: string;

  /** Spatial coordinates of origin */
  position: Vector3;

  /** Temporal coordinate of origin (departure time) */
  timestamp: Date;

  /** Timeline identifier */
  timelineID: string;

  /** Traveler's quantum signature */
  quantumSignature: QuantumSignature;

  /** Lock strength (0-100%) */
  strength: number;

  /** Lock strength quality level */
  strengthLevel: LockStrengthLevel;

  /** Energy reserved for lock maintenance (joules) */
  energyReserve: number;

  /** Lock creation timestamp */
  createdAt: Date;

  /** Lock expiration timestamp */
  expiresAt: Date;

  /** Current lock status */
  status: OriginLockStatus;

  /** Backup lock identifiers */
  backupLocks?: string[];

  /** Last maintenance timestamp */
  lastMaintenance?: Date;

  /** Custom metadata */
  metadata?: Record<string, unknown>;
}

/**
 * Origin lock creation parameters
 */
export interface OriginLockRequest {
  /** Lock identifier */
  id: string;

  /** Traveler identifier */
  travelerID: string;

  /** Origin spatial position */
  position: Vector3;

  /** Origin timestamp */
  timestamp: Date;

  /** Timeline identifier */
  timelineID: string;

  /** Traveler's quantum signature */
  quantumSignature: QuantumSignature;

  /** Energy reserve for lock maintenance (joules) */
  energyReserve: number;

  /** Number of backup locks (0-2) */
  backupLocks?: number;

  /** Custom metadata */
  metadata?: Record<string, unknown>;
}

/**
 * Origin lock creation response
 */
export interface OriginLockResponse {
  /** Success status */
  success: boolean;

  /** Created lock ID */
  lockID: string;

  /** Initial lock strength (%) */
  strength: number;

  /** Lock creation timestamp */
  createdAt: Date;

  /** Lock expiration timestamp */
  expiresAt: Date;

  /** Backup lock IDs if created */
  backupLockIDs?: string[];

  /** Warnings */
  warnings?: string[];

  /** Errors (if failed) */
  errors?: string[];
}

/**
 * Lock maintenance data
 */
export interface LockMaintenanceData {
  /** Lock identifier */
  lockID: string;

  /** Maintenance timestamp */
  timestamp: Date;

  /** Strength before maintenance */
  strengthBefore: number;

  /** Strength after maintenance */
  strengthAfter: number;

  /** Energy consumed (joules) */
  energyConsumed: number;

  /** Spatial drift corrected (meters) */
  spatialDrift: number;

  /** Temporal drift corrected (seconds) */
  temporalDrift: number;

  /** Interference detected */
  interference: number;

  /** Maintenance successful */
  success: boolean;

  /** Actions taken */
  actions: string[];
}

// ============================================================================
// Return Path Types
// ============================================================================

/**
 * Return path type
 */
export type ReturnPathType =
  | 'direct' // Straight line through spacetime
  | 'waypoint' // Via intermediate waypoints
  | 'spiral' // Following temporal currents
  | 'emergency'; // Fastest regardless of optimization

/**
 * Path optimization priority
 */
export type PathPriority = 'safest' | 'fastest' | 'efficient';

/**
 * Temporal path segment
 */
export interface PathSegment {
  /** Segment start coordinates */
  start: SpacetimeCoordinates;

  /** Segment end coordinates */
  end: SpacetimeCoordinates;

  /** Segment energy requirement (joules) */
  energy: number;

  /** Segment duration (seconds) */
  duration: number;

  /** Segment risk score (0-1) */
  risk: number;

  /** Obstacles in segment */
  obstacles?: string[];
}

/**
 * Complete return path
 */
export interface ReturnPath {
  /** Path identifier */
  pathID: string;

  /** Origin lock ID */
  lockID: string;

  /** Path type */
  pathType: ReturnPathType;

  /** Origin coordinates */
  origin: SpacetimeCoordinates;

  /** Current position */
  current: SpacetimeCoordinates;

  /** Path segments */
  segments: PathSegment[];

  /** Waypoints (if waypoint path) */
  waypoints?: SpacetimeCoordinates[];

  /** Total energy required (joules) */
  energyRequired: number;

  /** Total duration (seconds) */
  duration: number;

  /** Overall risk score (0-1) */
  riskScore: number;

  /** Safety score (0-100) */
  safetyScore: number;

  /** Path calculated timestamp */
  calculatedAt: Date;

  /** Path valid until */
  validUntil: Date;
}

/**
 * Return path calculation request
 */
export interface ReturnPathRequest {
  /** Origin lock identifier */
  lockID: string;

  /** Current spacetime position */
  currentPosition: SpacetimeCoordinates;

  /** Optimization priority */
  priority?: PathPriority;

  /** Path constraints */
  constraints?: {
    /** Maximum energy available (joules) */
    maxEnergy?: number;

    /** Maximum acceptable risk (0-1) */
    maxRisk?: number;

    /** Maximum duration (seconds) */
    maxDuration?: number;

    /** Avoid specific regions */
    avoidRegions?: SpacetimeCoordinates[];
  };
}

/**
 * Return path calculation response
 */
export interface ReturnPathResponse {
  /** Success status */
  success: boolean;

  /** Calculated path */
  path: ReturnPath;

  /** Alternative paths */
  alternatives?: ReturnPath[];

  /** Warnings */
  warnings?: string[];

  /** Errors (if failed) */
  errors?: string[];
}

// ============================================================================
// Return Execution Types
// ============================================================================

/**
 * Return execution status
 */
export type ReturnStatus =
  | 'initiated'
  | 'calculating_path'
  | 'phase_matching'
  | 'executing_jump'
  | 'reentry'
  | 'stabilizing'
  | 'verifying'
  | 'complete'
  | 'failed';

/**
 * Authorization token for return execution
 */
export interface AuthorizationToken {
  /** Token string */
  token: string;

  /** Token type */
  type: 'biometric' | 'quantum' | 'temporal' | 'multi_factor';

  /** Issued timestamp */
  issuedAt: Date;

  /** Expiration timestamp */
  expiresAt: Date;

  /** Issuer */
  issuer: string;
}

/**
 * Return execution request
 */
export interface ReturnExecutionRequest {
  /** Origin lock identifier */
  lockID: string;

  /** Traveler identifier */
  travelerID: string;

  /** Return path to execute */
  path: ReturnPath;

  /** Authorization token */
  authorization: AuthorizationToken;

  /** Emergency return flag */
  emergency?: boolean;
}

/**
 * Return execution response
 */
export interface ReturnExecutionResponse {
  /** Success status */
  success: boolean;

  /** Return execution identifier */
  returnID: string;

  /** Current status */
  status: ReturnStatus;

  /** Execution progress (0-100%) */
  progress: number;

  /** Estimated completion time */
  estimatedCompletion: Date;

  /** Real-time updates */
  updates?: string[];

  /** Errors (if any) */
  errors?: string[];
}

/**
 * Return execution telemetry
 */
export interface ReturnTelemetry {
  /** Return execution ID */
  returnID: string;

  /** Current timestamp */
  timestamp: Date;

  /** Current position */
  currentPosition: SpacetimeCoordinates;

  /** Current velocity */
  velocity: Vector3;

  /** Energy remaining (%) */
  energyRemaining: number;

  /** Phase alignment (radians) */
  phaseAlignment: number;

  /** Lock strength (%) */
  lockStrength: number;

  /** Health status */
  healthStatus: 'normal' | 'warning' | 'critical';

  /** System warnings */
  warnings: string[];
}

// ============================================================================
// Verification Types
// ============================================================================

/**
 * Biometric data for verification
 */
export interface BiometricData {
  /** Fingerprint scan */
  fingerprint?: string;

  /** Iris scan */
  iris?: string;

  /** DNA sequence hash */
  dna?: string;

  /** Facial recognition */
  facial?: string;

  /** Capture timestamp */
  capturedAt: Date;
}

/**
 * Memory challenge response
 */
export interface MemoryChallengeResponse {
  /** Secret passphrase */
  passphrase?: string;

  /** Departure details */
  departureDetails?: string;

  /** Personal memories */
  memories?: string[];

  /** Journey details */
  journeyDetails?: string;

  /** Emergency code word */
  emergencyCode?: string;
}

/**
 * Timeline fingerprint
 */
export interface TimelineFingerprint {
  /** Timeline identifier */
  timelineID: string;

  /** Historical events hash */
  historicalEvents: string;

  /** Physical constants hash */
  physicalConstants: string;

  /** Local state hash */
  localState: string;

  /** Fingerprint timestamp */
  timestamp: Date;
}

/**
 * Verification request
 */
export interface ReturnVerificationRequest {
  /** Origin lock identifier */
  lockID: string;

  /** Traveler identifier */
  travelerID: string;

  /** Biometric scans */
  biometrics: BiometricData;

  /** Current quantum signature */
  quantumSignature: QuantumSignature;

  /** Memory challenge responses */
  memoryChallenge: MemoryChallengeResponse;

  /** Timeline fingerprint */
  timelineFingerprint?: TimelineFingerprint;
}

/**
 * Verification response
 */
export interface ReturnVerificationResponse {
  /** Success status */
  success: boolean;

  /** Overall verification score (0-100%) */
  verificationScore: number;

  /** Individual verification scores */
  scores: {
    /** Biometric match (0-100%) */
    biometric: number;

    /** Quantum signature match (0-100%) */
    quantum: number;

    /** Memory challenge score (0-100%) */
    memory: number;

    /** Timeline match (0-100%) */
    timeline: number;

    /** Temporal signature match (0-100%) */
    temporal: number;
  };

  /** Verification approved (score ≥ 95%) */
  approved: boolean;

  /** Warnings */
  warnings?: string[];

  /** Verification timestamp */
  timestamp: Date;
}

// ============================================================================
// Health Check Types
// ============================================================================

/**
 * Vital signs
 */
export interface VitalSigns {
  /** Heart rate (bpm) */
  heartRate: number;

  /** Blood pressure (systolic/diastolic mmHg) */
  bloodPressure: { systolic: number; diastolic: number };

  /** Body temperature (Celsius) */
  temperature: number;

  /** Oxygen saturation (%) */
  oxygenSaturation: number;

  /** Respiratory rate (breaths/min) */
  respiratoryRate: number;

  /** Consciousness level */
  consciousness: 'alert' | 'drowsy' | 'unconscious';

  /** Measurement timestamp */
  timestamp: Date;
}

/**
 * Temporal sickness severity
 */
export type TemporalSicknessSeverity = 'mild' | 'moderate' | 'severe' | 'critical';

/**
 * Temporal sickness assessment
 */
export interface TemporalSicknessAssessment {
  /** Severity level */
  severity: TemporalSicknessSeverity;

  /** Symptoms present */
  symptoms: string[];

  /** Symptom onset time */
  onsetTime: Date;

  /** Treatment administered */
  treatment?: string;

  /** Expected recovery time (hours) */
  recoveryTime: number;
}

/**
 * Cellular health data
 */
export interface CellularHealth {
  /** Telomere length variation (%) */
  telomereVariation: number;

  /** DNA damage level (%) */
  dnaDamage: number;

  /** Mitochondrial function (% of baseline) */
  mitochondrialFunction: number;

  /** Cellular metabolism (% of baseline) */
  metabolism: number;

  /** Stem cell viability (%) */
  stemCellViability: number;

  /** Assessment timestamp */
  timestamp: Date;
}

/**
 * Psychological evaluation
 */
export interface PsychologicalEvaluation {
  /** Cognitive function score (0-100) */
  cognitiveScore: number;

  /** Emotional state score (0-100) */
  emotionalScore: number;

  /** Reality perception score (0-100) */
  realityScore: number;

  /** Temporal orientation score (0-100) */
  orientationScore: number;

  /** Identity continuity score (0-100) */
  identityScore: number;

  /** Trauma screening score (0-100) */
  traumaScore: number;

  /** Overall psychological score (0-100) */
  overallScore: number;

  /** Evaluation timestamp */
  timestamp: Date;

  /** Notes from evaluator */
  notes?: string;
}

/**
 * Health check result
 */
export interface HealthCheckResult {
  /** Traveler identifier */
  travelerID: string;

  /** Check timestamp */
  timestamp: Date;

  /** Check type */
  checkType: 'immediate' | 'standard' | 'extended';

  /** Vital signs */
  vitalSigns: VitalSigns;

  /** Temporal sickness assessment */
  temporalSickness?: TemporalSicknessAssessment;

  /** Cellular health */
  cellularHealth?: CellularHealth;

  /** Psychological evaluation */
  psychological?: PsychologicalEvaluation;

  /** Overall health status */
  healthStatus: 'excellent' | 'good' | 'fair' | 'poor' | 'critical';

  /** Health score (0-100) */
  healthScore: number;

  /** Medical recommendations */
  recommendations: string[];

  /** Approval for lock release */
  approvedForRelease: boolean;
}

// ============================================================================
// Return Window Types
// ============================================================================

/**
 * Return window quality level
 */
export type WindowQuality =
  | 'optimal' // W ≥ 0.95
  | 'good' // 0.75 ≤ W < 0.95
  | 'fair' // 0.50 ≤ W < 0.75
  | 'poor' // 0.25 ≤ W < 0.50
  | 'critical'; // W < 0.25

/**
 * Return window status
 */
export interface ReturnWindow {
  /** Origin lock identifier */
  lockID: string;

  /** Origin timestamp */
  originTime: Date;

  /** Current timestamp */
  currentTime: Date;

  /** Time difference from origin (seconds) */
  timeDelta: number;

  /** Window quality score (0-1) */
  qualityScore: number;

  /** Window quality level */
  qualityLevel: WindowQuality;

  /** Optimal return period start */
  optimalStart: Date;

  /** Optimal return period end */
  optimalEnd: Date;

  /** Maximum safe return time */
  maxSafeReturn: Date;

  /** Window closing in (seconds) */
  timeRemaining: number;

  /** Window status */
  status: 'open' | 'closing' | 'critical' | 'closed';

  /** Extension count */
  extensions: number;

  /** Can be extended */
  canExtend: boolean;

  /** Last updated */
  lastUpdated: Date;
}

/**
 * Window extension request
 */
export interface WindowExtensionRequest {
  /** Origin lock identifier */
  lockID: string;

  /** Extension duration (days) */
  extensionDays: number;

  /** Reason for extension */
  reason: string;

  /** Authorization */
  authorization: AuthorizationToken;
}

/**
 * Window extension response
 */
export interface WindowExtensionResponse {
  /** Success status */
  success: boolean;

  /** New expiration date */
  newExpiration: Date;

  /** Energy cost (joules) */
  energyCost: number;

  /** Extensions remaining */
  extensionsRemaining: number;

  /** Warnings */
  warnings?: string[];

  /** Errors (if failed) */
  errors?: string[];
}

// ============================================================================
// Emergency Return Types
// ============================================================================

/**
 * Emergency return trigger
 */
export type EmergencyTrigger =
  | 'low_energy' // <15% energy
  | 'lock_degradation' // <85% lock strength
  | 'medical_emergency'
  | 'equipment_failure'
  | 'paradox_detected'
  | 'timeline_collapse'
  | 'window_closure'
  | 'manual_activation';

/**
 * Emergency return request
 */
export interface EmergencyReturnRequest {
  /** Origin lock identifier */
  lockID: string;

  /** Traveler identifier */
  travelerID: string;

  /** Emergency trigger */
  trigger: EmergencyTrigger;

  /** Current position */
  currentPosition: SpacetimeCoordinates;

  /** Emergency details */
  details: string;

  /** Manual activation */
  manualActivation: boolean;
}

/**
 * Emergency return response
 */
export interface EmergencyReturnResponse {
  /** Success status */
  success: boolean;

  /** Emergency return ID */
  emergencyReturnID: string;

  /** Emergency path calculated */
  emergencyPath: ReturnPath;

  /** Estimated arrival time */
  estimatedArrival: Date;

  /** Medical team dispatched */
  medicalTeamDispatched: boolean;

  /** Risk assessment */
  riskAssessment: string;

  /** Instructions for traveler */
  instructions: string[];

  /** Status updates */
  statusUpdates: string[];
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Physical and system constants
 */
export const RETURN_CONSTANTS = {
  /** Speed of light (m/s) */
  SPEED_OF_LIGHT: 299792458,

  /** Temporal binding energy (J) */
  TEMPORAL_BINDING_ENERGY: 1e24,

  /** Lock decay constant (s⁻¹) */
  LOCK_DECAY_CONSTANT: 3.17e-10,

  /** Lock strength thresholds (%) */
  LOCK_STRENGTH: {
    EXCELLENT: 99.9,
    GOOD: 95,
    FAIR: 90,
    POOR: 80,
    CRITICAL: 80,
  },

  /** Return window parameters */
  WINDOW: {
    OPTIMAL_DAYS: 7,
    MAX_DAYS: 90,
    STANDARD_DEVIATION_DAYS: 7,
  },

  /** Phase matching tolerances (radians) */
  PHASE_TOLERANCE: {
    PERFECT: 0.001,
    GOOD: 0.01,
    FAIR: 0.1,
    EMERGENCY: 0.1,
  },

  /** Verification thresholds (%) */
  VERIFICATION: {
    APPROVAL_THRESHOLD: 95,
    BIOMETRIC_MIN: 90,
    QUANTUM_MIN: 99.9,
    MEMORY_MIN: 90,
    TIMELINE_MIN: 99.9,
  },

  /** Energy reserves (%) */
  ENERGY_RESERVE: {
    PRIMARY: 10,
    EMERGENCY: 5,
    BACKUP: 5,
  },

  /** Health check thresholds */
  HEALTH: {
    TELOMERE_VARIATION_MAX: 1, // %
    DNA_DAMAGE_MAX: 0.01, // %
    MITOCHONDRIAL_MIN: 95, // %
    METABOLISM_RANGE: 5, // ±%
    PSYCHOLOGICAL_MIN: 85, // score
  },

  /** Emergency return parameters */
  EMERGENCY: {
    ENERGY_TRIGGER: 15, // %
    LOCK_STRENGTH_TRIGGER: 85, // %
    SPATIAL_DEVIATION_MAX: 100, // meters
    TEMPORAL_DEVIATION_MAX: 3600, // seconds
    PHASE_TOLERANCE_MAX: 0.1, // radians
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
 * WIA-TIME-021 error codes
 */
export enum ReturnErrorCode {
  LOCK_NOT_FOUND = 'R001',
  LOCK_STRENGTH_LOW = 'R002',
  NO_VALID_PATH = 'R003',
  INSUFFICIENT_ENERGY = 'R004',
  WINDOW_CLOSED = 'R005',
  VERIFICATION_FAILED = 'R006',
  PHASE_MISMATCH = 'R007',
  TIMELINE_MISMATCH = 'R008',
  HEALTH_CHECK_FAILED = 'R009',
  EMERGENCY_TRIGGERED = 'R010',
}

/**
 * Return error class
 */
export class ReturnError extends Error {
  constructor(
    public code: ReturnErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'ReturnError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  // Core
  Vector3,
  SpacetimeCoordinates,

  // Origin Lock
  OriginLockStatus,
  LockStrengthLevel,
  QuantumSignature,
  OriginLock,
  OriginLockRequest,
  OriginLockResponse,
  LockMaintenanceData,

  // Return Path
  ReturnPathType,
  PathPriority,
  PathSegment,
  ReturnPath,
  ReturnPathRequest,
  ReturnPathResponse,

  // Return Execution
  ReturnStatus,
  AuthorizationToken,
  ReturnExecutionRequest,
  ReturnExecutionResponse,
  ReturnTelemetry,

  // Verification
  BiometricData,
  MemoryChallengeResponse,
  TimelineFingerprint,
  ReturnVerificationRequest,
  ReturnVerificationResponse,

  // Health Check
  VitalSigns,
  TemporalSicknessSeverity,
  TemporalSicknessAssessment,
  CellularHealth,
  PsychologicalEvaluation,
  HealthCheckResult,

  // Return Window
  WindowQuality,
  ReturnWindow,
  WindowExtensionRequest,
  WindowExtensionResponse,

  // Emergency
  EmergencyTrigger,
  EmergencyReturnRequest,
  EmergencyReturnResponse,
};

export { RETURN_CONSTANTS, ReturnErrorCode, ReturnError };
