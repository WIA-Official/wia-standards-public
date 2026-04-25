/**
 * WIA-DEF-020: Autonomous Weapon Ethics - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Defense Ethics Working Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core System Types
// ============================================================================

/**
 * Geographic coordinate
 */
export interface GeoCoordinate {
  /** Latitude in decimal degrees */
  lat: number;

  /** Longitude in decimal degrees */
  lon: number;

  /** Altitude in meters (optional) */
  alt?: number;
}

/**
 * Geographic boundary polygon
 */
export interface GeoBoundary {
  /** Boundary type */
  type: 'polygon' | 'circle' | 'corridor';

  /** Coordinates defining the boundary */
  coordinates: GeoCoordinate[];

  /** Radius in meters (for circle type) */
  radius?: number;

  /** Description */
  description?: string;
}

/**
 * Autonomy level classification (0-5)
 */
export enum AutonomyLevel {
  /** Level 0: Human-Operated */
  HUMAN_OPERATED = 0,

  /** Level 1: Human-Assisted */
  HUMAN_ASSISTED = 1,

  /** Level 2: Human-Supervised */
  HUMAN_SUPERVISED = 2,

  /** Level 3: Human-Delegated */
  HUMAN_DELEGATED = 3,

  /** Level 4: Human-Constrained */
  HUMAN_CONSTRAINED = 4,

  /** Level 5: Fully Autonomous (PROHIBITED) */
  FULLY_AUTONOMOUS = 5,
}

/**
 * Autonomous Weapon System configuration
 */
export interface AutonomousWeaponSystem {
  /** Unique system identifier */
  systemId: string;

  /** System name/designation */
  name: string;

  /** Autonomy level (0-4, Level 5 prohibited) */
  autonomyLevel: AutonomyLevel;

  /** System capabilities */
  capabilities: SystemCapability[];

  /** Operational constraints */
  constraints: SystemConstraints;

  /** Ethics framework configuration */
  ethicsFramework: EthicsFrameworkConfig;

  /** Current operational status */
  status: SystemStatus;

  /** Certification information */
  certification?: CertificationInfo;

  /** Deployment information */
  deployment?: DeploymentInfo;
}

/**
 * System capability
 */
export type SystemCapability =
  | 'detection'
  | 'tracking'
  | 'classification'
  | 'recommendation'
  | 'autonomous-engagement'
  | 'override'
  | 'self-defense';

/**
 * System operational status
 */
export type SystemStatus =
  | 'offline'
  | 'standby'
  | 'active'
  | 'engaged'
  | 'malfunction'
  | 'emergency-shutdown'
  | 'maintenance';

/**
 * System constraints
 */
export interface SystemConstraints {
  /** Require human authorization for engagement */
  requireHumanAuthorization: boolean;

  /** Maximum engagement range in meters */
  maxEngagementRange: number;

  /** Prohibited target types */
  prohibitedTargets: TargetType[];

  /** Geographic boundaries */
  geographicBounds?: GeoBoundary[];

  /** Temporal constraints */
  temporalBounds?: {
    validFrom: Date;
    validUntil: Date;
  };

  /** Maximum civilian probability threshold (0-1) */
  maxCivilianProbability: number;

  /** Minimum proportionality ratio */
  minProportionalityRatio: number;

  /** Override response time requirement (seconds) */
  overrideResponseTime: number;
}

// ============================================================================
// Target Types
// ============================================================================

/**
 * Target type classification
 */
export type TargetType =
  | 'military-personnel'
  | 'military-vehicle'
  | 'military-aircraft'
  | 'military-structure'
  | 'weapons-system'
  | 'dual-use'
  | 'civilian'
  | 'medical'
  | 'cultural-property'
  | 'pow'
  | 'journalist'
  | 'humanitarian-worker'
  | 'unknown';

/**
 * Target information
 */
export interface Target {
  /** Unique target identifier */
  id: string;

  /** Target type classification */
  type: TargetType;

  /** Geographic location */
  location: GeoCoordinate;

  /** Movement velocity (m/s) */
  velocity?: {
    speed: number;
    heading: number; // degrees
  };

  /** Classification confidence (0-1) */
  confidence: number;

  /** Combatant status assessment */
  combatantStatus: CombatantStatus;

  /** Additional metadata */
  metadata?: {
    visualSignature?: string;
    thermalSignature?: string;
    radarSignature?: string;
    behavioralIndicators?: string[];
    intelligence?: string;
  };

  /** Detection timestamp */
  detectedAt: Date;

  /** Last update timestamp */
  updatedAt: Date;
}

/**
 * Combatant status
 */
export type CombatantStatus =
  | 'combatant'
  | 'civilian'
  | 'protected-person'
  | 'hors-de-combat'
  | 'uncertain';

/**
 * Protected site information
 */
export interface ProtectedSite {
  /** Site identifier */
  id: string;

  /** Site type */
  type: 'hospital' | 'school' | 'religious' | 'cultural' | 'refugee-camp' | 'safe-zone';

  /** Site name */
  name: string;

  /** Geographic location */
  location: GeoCoordinate;

  /** Protection buffer radius in meters */
  bufferRadius: number;

  /** Protection status */
  status: 'active' | 'temporary' | 'disputed';

  /** Authoritative source */
  source?: string;
}

// ============================================================================
// Meaningful Human Control (MHC)
// ============================================================================

/**
 * Meaningful Human Control assessment
 */
export interface MHCAssessment {
  /** Overall MHC compliance */
  isCompliant: boolean;

  /** Overall MHC score (0-1) */
  score: number;

  /** Detailed factor scores */
  factors: {
    informationQuality: number; // 0-1
    understanding: number; // 0-1
    timeAdequacy: number; // 0-1
    authority: number; // 0-1
    accountability: number; // 0-1
  };

  /** Violations or concerns */
  violations: string[];

  /** Warnings */
  warnings: string[];

  /** Recommendations */
  recommendations: string[];

  /** Assessment timestamp */
  assessedAt: Date;
}

/**
 * Operator information
 */
export interface OperatorInfo {
  /** Operator ID */
  operatorId: string;

  /** Operator name */
  name: string;

  /** Qualification level */
  qualification: 'trainee' | 'certified' | 'expert';

  /** Training hours completed */
  trainingHours: number;

  /** Certifications */
  certifications: string[];

  /** Current situational awareness level */
  situationalAwareness?: 'low' | 'medium' | 'high';

  /** Override capability status */
  overrideCapable: boolean;
}

// ============================================================================
// Ethical Framework
// ============================================================================

/**
 * Ethics framework configuration
 */
export interface EthicsFrameworkConfig {
  /** Enable IHL compliance checks */
  enableIHLChecks: boolean;

  /** Require Meaningful Human Control */
  requireMHC: boolean;

  /** Civilian protection level */
  civilianProtectionLevel: 'standard' | 'enhanced' | 'maximum';

  /** Ethical principles to enforce */
  principles: EthicalPrinciple[];

  /** Decision algorithm version */
  algorithmVersion: string;

  /** Custom constraints */
  customConstraints?: Record<string, unknown>;
}

/**
 * Ethical principle
 */
export type EthicalPrinciple =
  | 'human-dignity'
  | 'distinction'
  | 'proportionality'
  | 'precaution'
  | 'military-necessity'
  | 'accountability';

/**
 * Ethical evaluation result
 */
export interface EthicalEvaluation {
  /** Decision outcome */
  decision: 'APPROVE' | 'DENY' | 'AWAIT_HUMAN' | 'ESCALATE';

  /** Primary reason for decision */
  primaryReason: string;

  /** Detailed explanation */
  explanation: {
    distinctionScore: number;
    proportionalityRatio: number;
    precautionsScore: number;
    militaryNecessity: boolean;
    contributingFactors: Array<{
      factor: string;
      impact: number;
      description: string;
    }>;
  };

  /** Violations detected */
  violations: EthicalViolation[];

  /** Warnings */
  warnings: string[];

  /** Conditions for approval */
  conditions?: {
    maxForce?: string;
    precautions?: string[];
    monitoring?: string;
    reviewRequired?: boolean;
  };

  /** Evaluation timestamp */
  evaluatedAt: Date;
}

/**
 * Ethical violation
 */
export interface EthicalViolation {
  /** Violation type */
  type: 'distinction' | 'proportionality' | 'precaution' | 'necessity' | 'protected-status';

  /** Severity level */
  severity: 'info' | 'warning' | 'error' | 'critical';

  /** Description */
  description: string;

  /** Affected principle */
  principle: EthicalPrinciple;

  /** Remediation suggestions */
  remediation?: string[];
}

// ============================================================================
// International Humanitarian Law (IHL)
// ============================================================================

/**
 * IHL compliance check result
 */
export interface IHLComplianceResult {
  /** Overall compliance status */
  isCompliant: boolean;

  /** Individual principle compliance */
  principles: {
    distinction: IHLPrincipleCheck;
    proportionality: IHLPrincipleCheck;
    precaution: IHLPrincipleCheck;
    militaryNecessity: IHLPrincipleCheck;
  };

  /** Detected violations */
  violations: IHLViolation[];

  /** Geneva Convention articles checked */
  conventionsChecked: string[];

  /** Compliance score (0-1) */
  complianceScore: number;

  /** Assessment timestamp */
  assessedAt: Date;
}

/**
 * IHL principle check
 */
export interface IHLPrincipleCheck {
  /** Compliance status */
  compliant: boolean;

  /** Confidence score (0-1) */
  confidence: number;

  /** Details */
  details: string;

  /** Issues identified */
  issues: string[];
}

/**
 * IHL violation
 */
export interface IHLViolation {
  /** Violation type */
  type:
    | 'indiscriminate-attack'
    | 'civilian-targeting'
    | 'protected-site'
    | 'excessive-force'
    | 'perfidy'
    | 'prohibited-weapon';

  /** Severity */
  severity: 'minor' | 'serious' | 'grave';

  /** Description */
  description: string;

  /** Relevant convention/protocol */
  legalBasis: string;

  /** Potential war crime classification */
  warCrime: boolean;
}

/**
 * Proportionality assessment
 */
export interface ProportionalityAssessment {
  /** Military advantage estimate (0-100) */
  militaryAdvantage: number;

  /** Estimated civilian casualties */
  estimatedCivilianCasualties: number;

  /** Estimated civilian property damage (0-100) */
  estimatedPropertyDamage: number;

  /** Urgency factor (1.0-2.0) */
  urgencyFactor: number;

  /** Proportionality ratio */
  proportionalityRatio: number;

  /** Is proportional? */
  isProportional: boolean;

  /** Assessment details */
  details: string;
}

/**
 * Precaution measures
 */
export interface PrecautionMeasures {
  /** Precautions score (0-1) */
  score: number;

  /** Measures taken */
  measures: Array<{
    type: string;
    description: string;
    effectiveness: number; // 0-1
  }>;

  /** Target verification completed */
  targetVerified: boolean;

  /** Alternative methods considered */
  alternativesConsidered: string[];

  /** Warning issued */
  warningIssued: boolean;

  /** Civilian evacuation time allowed */
  evacuationTime?: number; // seconds
}

// ============================================================================
// Engagement Decision
// ============================================================================

/**
 * Engagement decision request
 */
export interface EngagementRequest {
  /** Request ID */
  requestId: string;

  /** Target information */
  target: Target;

  /** Battlefield context */
  context: BattlefieldContext;

  /** Human operator information */
  operator?: OperatorInfo;

  /** Requested engagement type */
  engagementType: 'lethal' | 'non-lethal' | 'warning';

  /** Urgency level */
  urgency: 'low' | 'medium' | 'high' | 'critical';

  /** Request timestamp */
  requestedAt: Date;
}

/**
 * Battlefield context
 */
export interface BattlefieldContext {
  /** Geographic location */
  location: GeoCoordinate;

  /** Estimated civilian presence probability (0-1) */
  civilianPresence: number;

  /** Nearby protected sites */
  protectedSites: ProtectedSite[];

  /** Current Rules of Engagement */
  roe: RulesOfEngagement;

  /** Weather conditions */
  weather?: {
    visibility: 'clear' | 'reduced' | 'poor';
    conditions: string[];
  };

  /** Time of day */
  timeOfDay: 'day' | 'night' | 'twilight';

  /** Threat level */
  threatLevel: 'low' | 'medium' | 'high' | 'critical';
}

/**
 * Rules of Engagement (ROE)
 */
export interface RulesOfEngagement {
  /** Mission ID */
  missionId: string;

  /** Valid time range */
  validFrom: Date;
  validUntil: Date;

  /** Geographic boundaries */
  geographicBounds: GeoBoundary[];

  /** Authorized target types */
  authorizedTargets: Array<{
    type: TargetType;
    confidenceThreshold: number;
    maxEngagementRange: number;
  }>;

  /** Prohibited targets */
  prohibitedTargets: TargetType[];

  /** Engagement constraints */
  constraints: {
    maxCivilianProbability: number;
    minProportionalityRatio: number;
    requireHumanApproval: boolean;
  };

  /** Override authority */
  overrideAuthority: string[];

  /** Escalation procedures */
  escalationProcedures?: string;
}

/**
 * Engagement decision result
 */
export interface EngagementDecision {
  /** Decision ID */
  decisionId: string;

  /** Request that triggered this decision */
  requestId: string;

  /** Decision outcome */
  outcome: 'APPROVED' | 'DENIED' | 'AWAITING_HUMAN' | 'ESCALATED';

  /** Justification */
  justification: string;

  /** Reasons for denial (if denied) */
  reasons?: string[];

  /** Ethical evaluation */
  ethicalEvaluation: EthicalEvaluation;

  /** IHL compliance */
  ihlCompliance: IHLComplianceResult;

  /** MHC assessment */
  mhcAssessment?: MHCAssessment;

  /** Proportionality assessment */
  proportionality: ProportionalityAssessment;

  /** Precaution measures */
  precautions: PrecautionMeasures;

  /** Recommended weapon/munition */
  recommendedWeapon?: string;

  /** Conditions for engagement */
  conditions?: string[];

  /** Human approval required */
  humanApprovalRequired: boolean;

  /** Human operator decision (if applicable) */
  humanDecision?: {
    approved: boolean;
    operatorId: string;
    decidedAt: Date;
    rationale: string;
  };

  /** Decision timestamp */
  decidedAt: Date;
}

// ============================================================================
// Accountability & Audit
// ============================================================================

/**
 * Engagement log entry
 */
export interface EngagementLogEntry {
  /** Log entry ID */
  entryId: string;

  /** Engagement decision ID */
  decisionId: string;

  /** System ID */
  systemId: string;

  /** Operator ID (if applicable) */
  operatorId?: string;

  /** Target information */
  target: Target;

  /** Engagement outcome */
  outcome: 'completed' | 'aborted' | 'failed' | 'overridden';

  /** Engagement details */
  details: {
    weaponUsed?: string;
    launchTime?: Date;
    impactTime?: Date;
    impactLocation?: GeoCoordinate;
  };

  /** Battle damage assessment */
  battleDamageAssessment?: {
    targetDestroyed: boolean;
    collateralDamage: 'none' | 'minimal' | 'moderate' | 'severe';
    civilianCasualties: number;
    propertyDamage: string;
  };

  /** Post-engagement review */
  review?: {
    conducted: boolean;
    reviewedAt?: Date;
    findings?: string;
    lessonsLearned?: string[];
  };

  /** Cryptographic signature for tamper-proofing */
  signature: string;

  /** Timestamp */
  timestamp: Date;
}

/**
 * Audit report
 */
export interface AuditReport {
  /** Report ID */
  reportId: string;

  /** System ID */
  systemId: string;

  /** Audit period */
  period: {
    from: Date;
    to: Date;
  };

  /** Total engagements */
  totalEngagements: number;

  /** Engagement breakdown */
  engagements: {
    approved: number;
    denied: number;
    humanOverridden: number;
  };

  /** Compliance metrics */
  compliance: {
    ihlCompliance: number; // percentage
    mhcCompliance: number; // percentage
    ethicsCompliance: number; // percentage
  };

  /** Violations detected */
  violations: Array<{
    type: string;
    count: number;
    severity: string;
  }>;

  /** Civilian protection metrics */
  civilianProtection: {
    engagementsNearCivilians: number;
    civilianCasualties: number;
    falsePositives: number;
  };

  /** System performance */
  performance: {
    targetClassificationAccuracy: number;
    overrideResponseTime: number; // average in seconds
    systemAvailability: number; // percentage
  };

  /** Recommendations */
  recommendations: string[];

  /** Auditor information */
  auditor: {
    id: string;
    name: string;
    organization: string;
  };

  /** Report generated timestamp */
  generatedAt: Date;
}

/**
 * Certification information
 */
export interface CertificationInfo {
  /** Certification ID */
  certificationId: string;

  /** Certifying authority */
  authority: string;

  /** Certification type */
  type: 'technical' | 'ethical' | 'legal' | 'comprehensive';

  /** Issued date */
  issuedDate: Date;

  /** Expiry date */
  expiryDate: Date;

  /** Status */
  status: 'valid' | 'expired' | 'revoked' | 'suspended';

  /** Certification conditions */
  conditions?: string[];
}

/**
 * Deployment information
 */
export interface DeploymentInfo {
  /** Deployment ID */
  deploymentId: string;

  /** Mission name */
  missionName: string;

  /** Geographic area of operation */
  areaOfOperation: GeoBoundary;

  /** Deployment start */
  deployedAt: Date;

  /** Expected end */
  expectedEndAt?: Date;

  /** Actual end */
  actualEndAt?: Date;

  /** Commanding officer */
  commandingOfficer: {
    id: string;
    name: string;
    rank: string;
  };

  /** Authorization */
  authorization: {
    authorizedBy: string;
    authorizedAt: Date;
    authorizationDocument: string;
  };
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

/**
 * WIA-DEF-020 error codes
 */
export enum DefenseErrorCode {
  INVALID_AUTONOMY_LEVEL = 'DEF001',
  MHC_VIOLATION = 'DEF002',
  IHL_VIOLATION = 'DEF003',
  ETHICAL_CONSTRAINT_VIOLATION = 'DEF004',
  PROHIBITED_TARGET = 'DEF005',
  INSUFFICIENT_DISTINCTION = 'DEF006',
  DISPROPORTIONATE_FORCE = 'DEF007',
  INADEQUATE_PRECAUTIONS = 'DEF008',
  CERTIFICATION_EXPIRED = 'DEF009',
  OPERATOR_NOT_QUALIFIED = 'DEF010',
  GEOGRAPHIC_CONSTRAINT_VIOLATION = 'DEF011',
  TEMPORAL_CONSTRAINT_VIOLATION = 'DEF012',
  SYSTEM_MALFUNCTION = 'DEF013',
  OVERRIDE_FAILURE = 'DEF014',
  AUDIT_TRAIL_CORRUPTION = 'DEF015',
}

/**
 * Defense ethics error
 */
export class DefenseEthicsError extends Error {
  constructor(
    public code: DefenseErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'DefenseEthicsError';
  }
}

// ============================================================================
// Export All Types
// ============================================================================

export type {
  GeoCoordinate,
  GeoBoundary,
  AutonomousWeaponSystem,
  SystemConstraints,
  Target,
  ProtectedSite,
  MHCAssessment,
  OperatorInfo,
  EthicsFrameworkConfig,
  EthicalEvaluation,
  IHLComplianceResult,
  ProportionalityAssessment,
  PrecautionMeasures,
  EngagementRequest,
  BattlefieldContext,
  RulesOfEngagement,
  EngagementDecision,
  EngagementLogEntry,
  AuditReport,
  CertificationInfo,
  DeploymentInfo,
};

export {
  AutonomyLevel,
  DefenseErrorCode,
  DefenseEthicsError,
};
