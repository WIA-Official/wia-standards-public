/**
 * WIA-DEF-001: Unmanned Weapon - TypeScript Type Definitions
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
 * Three-dimensional coordinate (latitude, longitude, altitude)
 */
export interface Coordinate3D {
  latitude: number;
  longitude: number;
  altitude: number;  // meters above sea level
}

/**
 * Three-dimensional vector
 */
export interface Vector3D {
  x: number;
  y: number;
  z: number;
}

/**
 * Geographic bounds for operational areas
 */
export interface GeographicBounds {
  north: number;  // latitude
  south: number;
  east: number;   // longitude
  west: number;
  minAltitude?: number;
  maxAltitude?: number;
}

// ============================================================================
// Weapon Platform Types
// ============================================================================

/**
 * Unmanned weapon system types
 */
export type WeaponType = 'UGV' | 'UAV' | 'USV' | 'UUV' | 'Sentry' | 'Loitering';

/**
 * Autonomy levels (0-4)
 */
export type AutonomyLevel = 0 | 1 | 2 | 3 | 4;

/**
 * Weapon platform role
 */
export type WeaponRole =
  | 'air-defense'
  | 'perimeter-security'
  | 'anti-armor'
  | 'counter-uas'
  | 'isr'
  | 'electronic-warfare'
  | 'mine-countermeasure'
  | 'force-protection';

/**
 * Weapon platform configuration
 */
export interface WeaponConfig {
  /** Unique system identifier */
  id: string;

  /** Platform type */
  type: WeaponType;

  /** Operational role */
  role: WeaponRole;

  /** Autonomy level (0-4) */
  autonomyLevel: AutonomyLevel;

  /** Payload configuration */
  payload: PayloadConfig;

  /** Sensor suite */
  sensors: SensorConfig[];

  /** Maximum operational range in meters */
  range: number;

  /** Maximum speed in m/s */
  maxSpeed: number;

  /** Endurance in seconds */
  endurance: number;

  /** Mass in kilograms */
  mass?: number;

  /** Current status */
  status: 'standby' | 'active' | 'engaged' | 'returning' | 'maintenance' | 'fault';
}

// ============================================================================
// Payload Configuration
// ============================================================================

/**
 * Payload types
 */
export type PayloadType =
  | 'kinetic-missile'
  | 'kinetic-gun'
  | 'non-lethal'
  | 'electronic-warfare'
  | 'directed-energy'
  | 'interceptor'
  | 'surveillance';

/**
 * Payload configuration
 */
export interface PayloadConfig {
  /** Payload type */
  type: PayloadType;

  /** Number of rounds/missiles */
  count: number;

  /** Effective range in meters */
  range: number;

  /** Lethal radius in meters (0 for non-lethal) */
  lethalRadius?: number;

  /** Injury radius in meters */
  injuryRadius?: number;

  /** Guidance type */
  guidance?: 'gps' | 'laser' | 'optical' | 'radar' | 'rf-homing' | 'unguided';

  /** Probability of kill (0-1) */
  pk?: number;
}

// ============================================================================
// Sensor Configuration
// ============================================================================

/**
 * Sensor types
 */
export type SensorType =
  | 'radar'
  | 'eo'         // Electro-Optical
  | 'ir'         // Infrared
  | 'acoustic'
  | 'lidar'
  | 'rf'         // Radio Frequency
  | 'magnetic'
  | 'seismic';

/**
 * Sensor configuration
 */
export interface SensorConfig {
  /** Sensor type */
  type: SensorType;

  /** Detection range in meters */
  range: number;

  /** Field of view in degrees */
  fov?: number;

  /** Resolution (context-dependent) */
  resolution?: number;

  /** Confidence/accuracy (0-1) */
  accuracy: number;

  /** Update rate in Hz */
  updateRate?: number;

  /** Is sensor currently operational */
  operational: boolean;
}

// ============================================================================
// Target Types
// ============================================================================

/**
 * Target classification
 */
export type TargetClass =
  | 'hostile-aircraft'
  | 'hostile-uav'
  | 'hostile-missile'
  | 'hostile-vehicle'
  | 'hostile-vessel'
  | 'hostile-personnel'
  | 'unknown'
  | 'civilian'
  | 'friendly';

/**
 * IFF (Identification Friend or Foe) status
 */
export type IFFStatus = 'friendly' | 'neutral' | 'hostile' | 'unknown';

/**
 * Target information
 */
export interface Target {
  /** Unique target identifier */
  id: string;

  /** Target classification */
  classification: TargetClass;

  /** Current position */
  position: Coordinate3D;

  /** Velocity vector in m/s */
  velocity: Vector3D;

  /** Heading in degrees (0-360) */
  heading?: number;

  /** IFF status */
  iffStatus: IFFStatus;

  /** Classification confidence (0-1) */
  certainty: number;

  /** First detection timestamp */
  firstDetected: Date;

  /** Last update timestamp */
  lastUpdated: Date;

  /** Radar cross-section (if applicable) */
  rcs?: number;

  /** Visual signature data */
  signature?: TargetSignature;
}

/**
 * Target signature data
 */
export interface TargetSignature {
  /** Visual features */
  visual?: string;

  /** Thermal signature */
  thermal?: number[];

  /** Acoustic signature */
  acoustic?: number[];

  /** RF emissions */
  rf?: number[];
}

// ============================================================================
// Threat Assessment
// ============================================================================

/**
 * Threat assessment result
 */
export interface ThreatAssessment {
  /** Target being assessed */
  target: Target;

  /** Threat level (0-1) */
  threatLevel: number;

  /** Threat classification */
  classification: 'low' | 'medium' | 'high' | 'critical';

  /** Component scores */
  scores: {
    capability: number;   // Destructive potential (0-1)
    intent: number;       // Assessed hostile intent (0-1)
    proximity: number;    // Distance threat (0-1, inverse distance)
    velocity: number;     // Speed threat (0-1)
  };

  /** Time to impact in seconds (if applicable) */
  timeToImpact?: number;

  /** Recommended action */
  recommendedAction: 'monitor' | 'track' | 'prepare' | 'engage';

  /** Assessment timestamp */
  timestamp: Date;
}

// ============================================================================
// Engagement Authorization
// ============================================================================

/**
 * Engagement request
 */
export interface EngagementRequest {
  /** Target to engage */
  target: Target;

  /** Weapon system requesting engagement */
  weaponId: string;

  /** Threat assessment */
  threatAssessment: ThreatAssessment;

  /** Proposed weapon/payload */
  proposedWeapon: string;

  /** Estimated time to engage */
  timeToEngage: number;

  /** Human approval provided (for Level 2+) */
  humanApproved?: boolean;

  /** Requesting operator ID */
  operatorId?: string;
}

/**
 * Engagement authorization result
 */
export interface EngagementAuthorization {
  /** Is engagement authorized */
  authorized: boolean;

  /** Engagement score (0-1) */
  engagementScore: number;

  /** Reasoning for decision */
  reasoning: string[];

  /** Errors preventing authorization */
  errors: string[];

  /** Warnings (non-blocking) */
  warnings: string[];

  /** Selected weapon/payload */
  weaponSelected?: string;

  /** Estimated collateral damage (0-1) */
  estimatedCollateralDamage: number;

  /** Ethical compliance check */
  ethicalCompliance: EthicalCheck;

  /** Proportionality score */
  proportionality: number;

  /** Requires human approval */
  humanApprovalRequired: boolean;

  /** Authorization timestamp */
  timestamp: Date;
}

// ============================================================================
// Ethical Compliance
// ============================================================================

/**
 * Ethical compliance check result
 */
export interface EthicalCheck {
  /** Is action ethically compliant */
  compliant: boolean;

  /** IHL (International Humanitarian Law) compliance */
  ihl: {
    distinction: boolean;      // Can distinguish combatant/civilian
    proportionality: boolean;  // Proportional response
    necessity: boolean;        // Necessary action
    humanity: boolean;         // Minimizes suffering
  };

  /** Specific violations detected */
  violations: EthicalViolation[];

  /** Recommendations */
  recommendations: string[];

  /** Override allowed (emergency only) */
  overrideAllowed: boolean;
}

/**
 * Ethical violation
 */
export interface EthicalViolation {
  /** Violation type */
  type:
    | 'civilian-targeting'
    | 'indiscriminate-weapon'
    | 'excessive-force'
    | 'no-fire-zone'
    | 'friendly-fire'
    | 'perfidy'
    | 'proportionality';

  /** Severity level */
  severity: 'info' | 'warning' | 'critical' | 'prohibited';

  /** Description */
  description: string;

  /** Affected entities */
  affected?: string[];

  /** Possible remediation */
  remediation?: string;
}

// ============================================================================
// Rules of Engagement (ROE)
// ============================================================================

/**
 * Rules of Engagement configuration
 */
export interface RulesOfEngagement {
  /** ROE identifier */
  id: string;

  /** ROE name/description */
  name: string;

  /** Maximum authorized autonomy level */
  maxAutonomyLevel: AutonomyLevel;

  /** Weapons free zones (can engage without additional approval) */
  weaponsFreeZones: Geofence[];

  /** Weapons tight zones (can engage only specific targets) */
  weaponsTightZones: Geofence[];

  /** Weapons hold zones (cannot engage without higher approval) */
  weaponsHoldZones: Geofence[];

  /** No-fire zones (absolutely no engagement) */
  noFireZones: Geofence[];

  /** Authorized target types */
  authorizedTargets: TargetClass[];

  /** Prohibited target types */
  prohibitedTargets: TargetClass[];

  /** Maximum acceptable collateral damage (0-1) */
  maxCollateralDamage: number;

  /** Escalation of force required */
  escalationRequired: boolean;

  /** Valid from timestamp */
  validFrom: Date;

  /** Valid until timestamp */
  validUntil: Date;

  /** Issuing authority */
  authority: string;
}

/**
 * Geofence definition
 */
export interface Geofence {
  /** Geofence identifier */
  id: string;

  /** Geofence name */
  name?: string;

  /** Fence type */
  type: 'polygon' | 'circle' | 'corridor' | 'volume';

  /** Boundary coordinates */
  coordinates: Coordinate3D[];

  /** Center point (for circle type) */
  center?: Coordinate3D;

  /** Radius in meters (for circle type) */
  radius?: number;

  /** Altitude floor (meters) */
  minAltitude?: number;

  /** Altitude ceiling (meters) */
  maxAltitude?: number;

  /** Valid time range */
  validFrom?: Date;
  validUntil?: Date;
}

// ============================================================================
// Swarm Coordination
// ============================================================================

/**
 * Swarm configuration
 */
export interface SwarmConfig {
  /** Swarm identifier */
  id: string;

  /** Swarm name */
  name?: string;

  /** Swarm leader unit ID */
  leaderId: string;

  /** Member unit IDs */
  memberIds: string[];

  /** Swarm formation */
  formation: SwarmFormation;

  /** Coordination protocol */
  protocol: 'hierarchical' | 'distributed' | 'consensus';

  /** Communication mesh */
  meshNetwork: MeshNetwork;

  /** Swarm mission */
  mission: SwarmMission;

  /** Created timestamp */
  created: Date;

  /** Current status */
  status: 'forming' | 'active' | 'engaged' | 'dispersed' | 'returning';
}

/**
 * Swarm formation types
 */
export type SwarmFormation =
  | 'line-abreast'
  | 'wedge'
  | 'diamond'
  | 'column'
  | 'swarm'
  | 'custom';

/**
 * Mesh network configuration
 */
export interface MeshNetwork {
  /** Network protocol */
  protocol: string;

  /** Encryption enabled */
  encrypted: boolean;

  /** Link quality threshold (0-1) */
  minLinkQuality: number;

  /** Maximum hop count */
  maxHops: number;

  /** Update rate in Hz */
  updateRate: number;
}

/**
 * Swarm mission
 */
export interface SwarmMission {
  /** Mission type */
  type: 'patrol' | 'intercept' | 'area-denial' | 'escort' | 'search';

  /** Mission objective */
  objective: string;

  /** Operational area */
  area: Geofence;

  /** Priority targets */
  targets?: Target[];

  /** Mission duration in seconds */
  duration?: number;

  /** Success criteria */
  successCriteria?: string[];
}

/**
 * Swarm task allocation
 */
export interface SwarmTaskAllocation {
  /** Task identifier */
  taskId: string;

  /** Assigned unit ID */
  unitId: string;

  /** Task type */
  taskType: 'patrol' | 'intercept' | 'observe' | 'engage' | 'support';

  /** Task priority (0-1) */
  priority: number;

  /** Task location */
  location: Coordinate3D;

  /** Estimated completion time */
  estimatedCompletion?: Date;

  /** Task status */
  status: 'assigned' | 'in-progress' | 'completed' | 'failed';
}

// ============================================================================
// Safety and Compliance
// ============================================================================

/**
 * Safety check result
 */
export interface SafetyCheck {
  /** Check name */
  name: string;

  /** Check status */
  status: 'pass' | 'fail' | 'warning' | 'info';

  /** Description */
  description: string;

  /** Measured value */
  value?: number;

  /** Expected value */
  expected?: number;

  /** Threshold */
  threshold?: number;

  /** Corrective action if failed */
  correctiveAction?: string;

  /** Check timestamp */
  timestamp: Date;
}

/**
 * System diagnostic result
 */
export interface SystemDiagnostic {
  /** System ID */
  systemId: string;

  /** Overall health (0-1) */
  health: number;

  /** Component statuses */
  components: {
    propulsion: ComponentStatus;
    weapons: ComponentStatus;
    sensors: ComponentStatus;
    communications: ComponentStatus;
    navigation: ComponentStatus;
    power: ComponentStatus;
  };

  /** Active faults */
  faults: SystemFault[];

  /** Last diagnostic timestamp */
  timestamp: Date;
}

/**
 * Component status
 */
export interface ComponentStatus {
  /** Component name */
  name: string;

  /** Operational status */
  operational: boolean;

  /** Health score (0-1) */
  health: number;

  /** Error messages */
  errors: string[];

  /** Maintenance required */
  maintenanceRequired: boolean;
}

/**
 * System fault
 */
export interface SystemFault {
  /** Fault code */
  code: string;

  /** Severity */
  severity: 'info' | 'warning' | 'critical' | 'fatal';

  /** Fault description */
  description: string;

  /** Affected subsystem */
  subsystem: string;

  /** Corrective action */
  correctiveAction?: string;

  /** Fault detected timestamp */
  detectedAt: Date;
}

// ============================================================================
// Logging and Audit
// ============================================================================

/**
 * Engagement log entry
 */
export interface EngagementLog {
  /** Log entry ID */
  id: string;

  /** Timestamp */
  timestamp: Date;

  /** System ID that engaged */
  systemId: string;

  /** Target ID */
  targetId: string;

  /** Engagement authorization */
  authorization: EngagementAuthorization;

  /** Weapon used */
  weaponUsed: string;

  /** Engagement outcome */
  outcome: 'success' | 'failure' | 'abort' | 'miss';

  /** Battle damage assessment */
  bda?: BattleDamageAssessment;

  /** Sensor recordings */
  sensorData?: string;  // Link to stored data

  /** Human operator involved */
  operatorId?: string;

  /** Review status */
  reviewStatus: 'pending' | 'reviewed' | 'investigated' | 'cleared';

  /** Review notes */
  reviewNotes?: string[];
}

/**
 * Battle Damage Assessment
 */
export interface BattleDamageAssessment {
  /** Target neutralized */
  targetNeutralized: boolean;

  /** Collateral damage occurred */
  collateralDamage: boolean;

  /** Civilian casualties */
  civilianCasualties: number;

  /** Infrastructure damage */
  infrastructureDamage: string[];

  /** Secondary threats detected */
  secondaryThreats: string[];

  /** Assessment confidence (0-1) */
  confidence: number;

  /** Assessment method */
  method: 'sensor' | 'human-observation' | 'intelligence' | 'estimated';
}

/**
 * Audit trail entry
 */
export interface AuditEntry {
  /** Entry ID */
  id: string;

  /** Timestamp */
  timestamp: Date;

  /** Event type */
  eventType:
    | 'system-activation'
    | 'mission-start'
    | 'target-detected'
    | 'engagement-requested'
    | 'engagement-authorized'
    | 'engagement-executed'
    | 'mission-complete'
    | 'system-fault'
    | 'human-override'
    | 'roi-violation'
    | 'ethical-violation';

  /** Actor (system or human) */
  actor: string;

  /** Action description */
  action: string;

  /** Related entities */
  entities?: string[];

  /** Data snapshot */
  data?: Record<string, unknown>;

  /** Compliance status */
  compliant: boolean;
}

// ============================================================================
// Command and Control
// ============================================================================

/**
 * Command message
 */
export interface Command {
  /** Command ID */
  id: string;

  /** Command type */
  type:
    | 'activate'
    | 'standby'
    | 'engage'
    | 'abort'
    | 'return-to-base'
    | 'patrol'
    | 'hold-position'
    | 'self-destruct';

  /** Target system ID */
  systemId: string;

  /** Command parameters */
  parameters?: Record<string, unknown>;

  /** Issuing authority */
  issuedBy: string;

  /** Priority */
  priority: 'low' | 'medium' | 'high' | 'critical';

  /** Issued timestamp */
  issuedAt: Date;

  /** Expiration timestamp */
  expiresAt?: Date;

  /** Acknowledgment required */
  ackRequired: boolean;
}

/**
 * Command acknowledgment
 */
export interface CommandAcknowledgment {
  /** Command ID being acknowledged */
  commandId: string;

  /** Acknowledgment status */
  status: 'received' | 'executing' | 'completed' | 'failed' | 'rejected';

  /** Response message */
  message?: string;

  /** Timestamp */
  timestamp: Date;
}

/**
 * Telemetry data
 */
export interface Telemetry {
  /** System ID */
  systemId: string;

  /** Current position */
  position: Coordinate3D;

  /** Velocity */
  velocity: Vector3D;

  /** Heading */
  heading: number;

  /** Altitude */
  altitude: number;

  /** Fuel/battery level (0-1) */
  fuelLevel: number;

  /** System health (0-1) */
  health: number;

  /** Active targets */
  activeTargets: string[];

  /** Current mission status */
  missionStatus: string;

  /** Timestamp */
  timestamp: Date;
}

// ============================================================================
// Error Codes
// ============================================================================

/**
 * WIA-DEF-001 error codes
 */
export enum DefenseErrorCode {
  TARGET_IDENTIFICATION_FAILURE = 'D001',
  IFF_MALFUNCTION = 'D002',
  GEOFENCE_VIOLATION = 'D003',
  COMMUNICATION_LOSS = 'D004',
  ETHICAL_VIOLATION = 'D005',
  SENSOR_MALFUNCTION = 'D006',
  WEAPON_MALFUNCTION = 'D007',
  SWARM_COHERENCE_LOST = 'D008',
  UNAUTHORIZED_ENGAGEMENT = 'D009',
  COLLATERAL_DAMAGE_EXCEEDED = 'D010',
}

/**
 * Defense system error
 */
export class DefenseError extends Error {
  constructor(
    public code: DefenseErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'DefenseError';
  }
}

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
// Constants
// ============================================================================

/**
 * Defense system constants
 */
export const DEFENSE_CONSTANTS = {
  /** Minimum target certainty for engagement */
  MIN_TARGET_CERTAINTY: 0.95,

  /** Minimum threat level for autonomous engagement */
  MIN_THREAT_LEVEL: 0.8,

  /** Minimum engagement score (Level 3) */
  MIN_ENGAGEMENT_SCORE_L3: 0.95,

  /** Minimum engagement score (Level 4) */
  MIN_ENGAGEMENT_SCORE_L4: 0.98,

  /** Maximum acceptable collateral damage */
  MAX_COLLATERAL_DAMAGE: 0.05,

  /** Communication timeout (seconds) */
  COMMUNICATION_TIMEOUT: 30,

  /** Dead man switch timeout (seconds) */
  DEAD_MAN_TIMEOUT: 300,

  /** Maximum swarm size */
  MAX_SWARM_SIZE: 50,

  /** Minimum swarm coherence */
  MIN_SWARM_COHERENCE: 0.6,

  /** IFF interrogation timeout (milliseconds) */
  IFF_TIMEOUT: 500,
} as const;

// ============================================================================
// Export All
// ============================================================================

export type {
  Coordinate3D,
  Vector3D,
  GeographicBounds,
  WeaponType,
  AutonomyLevel,
  WeaponRole,
  WeaponConfig,
  PayloadType,
  PayloadConfig,
  SensorType,
  SensorConfig,
  TargetClass,
  IFFStatus,
  Target,
  TargetSignature,
  ThreatAssessment,
  EngagementRequest,
  EngagementAuthorization,
  EthicalCheck,
  EthicalViolation,
  RulesOfEngagement,
  Geofence,
  SwarmConfig,
  SwarmFormation,
  MeshNetwork,
  SwarmMission,
  SwarmTaskAllocation,
  SafetyCheck,
  SystemDiagnostic,
  ComponentStatus,
  SystemFault,
  EngagementLog,
  BattleDamageAssessment,
  AuditEntry,
  Command,
  CommandAcknowledgment,
  Telemetry,
};

export { DEFENSE_CONSTANTS, DefenseErrorCode, DefenseError };
