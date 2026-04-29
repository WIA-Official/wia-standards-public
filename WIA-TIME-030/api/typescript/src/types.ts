/**
 * WIA-TIME-030: Time Travel Ethics - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Ethics Committee
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Ethics Types
// ============================================================================

/**
 * Ethical principles for time travel
 */
export enum EthicalPrinciple {
  NON_MALEFICENCE = 'non-maleficence',
  BENEFICENCE = 'beneficence',
  JUSTICE = 'justice',
  RESPECT_FOR_AUTONOMY = 'respect-for-autonomy',
  HISTORICAL_PRESERVATION = 'historical-preservation',
  TEMPORAL_PRIVACY = 'temporal-privacy',
}

/**
 * Interference levels for temporal operations
 */
export enum InterferenceLevel {
  /** Absolute non-interference - prohibited */
  LEVEL_1_ABSOLUTE = 1,
  /** Conditional interaction - requires enhanced review */
  LEVEL_2_CONDITIONAL = 2,
  /** Permitted intervention - emergency review */
  LEVEL_3_PERMITTED = 3,
}

/**
 * Temporal operation purposes
 */
export enum OperationPurpose {
  SCIENTIFIC_RESEARCH = 'scientific-research',
  ARCHAEOLOGICAL_STUDY = 'archaeological-study',
  HISTORICAL_DOCUMENTATION = 'historical-documentation',
  MEDICAL_INTERVENTION = 'medical-intervention',
  DISASTER_PREVENTION = 'disaster-prevention',
  ENVIRONMENTAL_PROTECTION = 'environmental-protection',
  EDUCATIONAL = 'educational',
  PERSONAL_RESEARCH = 'personal-research',
  KNOWLEDGE_RECOVERY = 'knowledge-recovery',
}

/**
 * Intervention types
 */
export type InterventionType =
  | 'none' // Observer only
  | 'minimal' // Minor interaction
  | 'moderate' // Significant interaction
  | 'major' // Historical event modification
  | 'catastrophic'; // Timeline alteration

// ============================================================================
// Temporal Rights
// ============================================================================

/**
 * Individual temporal rights
 */
export interface TemporalRights {
  /** Right to privacy from temporal observation */
  temporalPrivacy: boolean;

  /** Right to non-interference in personal timeline */
  nonInterference: boolean;

  /** Right to dignity and respectful treatment */
  temporalDignity: boolean;

  /** Right to informed consent for interaction */
  informedConsent: boolean;

  /** Additional rights */
  customRights?: string[];
}

/**
 * Collective temporal rights for societies/civilizations
 */
export interface CollectiveTemporalRights {
  /** Right to historical integrity */
  historicalIntegrity: boolean;

  /** Right to cultural heritage protection */
  culturalHeritage: boolean;

  /** Right to timeline sovereignty */
  timelineSovereignty: boolean;

  /** Right to self-determination */
  selfDetermination: boolean;

  /** Additional collective rights */
  customRights?: string[];
}

// ============================================================================
// Temporal Operation Request
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
  alt?: number;

  /** Location description */
  description?: string;
}

/**
 * Temporal operation request
 */
export interface TemporalOperationRequest {
  /** Unique request ID */
  id?: string;

  /** Traveler ID */
  traveler: string;

  /** Target date and time */
  targetDate: Date | string;

  /** Operation purpose */
  purpose: OperationPurpose | string;

  /** Intervention level requested */
  interventionLevel: InterventionType;

  /** Duration in seconds */
  duration: number;

  /** Target location */
  location: GeoLocation;

  /** Detailed justification */
  justification?: string;

  /** Expected benefits */
  expectedBenefits?: string[];

  /** Alternative methods considered */
  alternatives?: string[];

  /** Risk mitigation plan */
  riskMitigation?: string[];

  /** Number of travelers */
  travelerCount?: number;

  /** Equipment list */
  equipment?: string[];

  /** Special requests */
  specialRequests?: string[];
}

// ============================================================================
// Historical Protection
// ============================================================================

/**
 * Protection levels for historical periods/events
 */
export enum ProtectionLevel {
  /** Absolute protection - access prohibited */
  ABSOLUTE = 'absolute',
  /** Enhanced protection - strict approval required */
  ENHANCED = 'enhanced',
  /** Standard protection - normal review */
  STANDARD = 'standard',
  /** Minimal protection - light review */
  MINIMAL = 'minimal',
  /** No protection - unrestricted (future events) */
  NONE = 'none',
}

/**
 * Historical significance levels
 */
export enum HistoricalSignificance {
  CRITICAL = 'critical', // Major historical events
  HIGH = 'high', // Significant events
  MODERATE = 'moderate', // Notable events
  LOW = 'low', // Minor events
  MINIMAL = 'minimal', // Everyday occurrences
}

/**
 * Protected historical period
 */
export interface ProtectedPeriod {
  /** Unique identifier */
  id: string;

  /** Period name */
  name: string;

  /** Start date */
  startDate: Date;

  /** End date */
  endDate: Date;

  /** Protection level */
  protectionLevel: ProtectionLevel;

  /** Reason for protection */
  reason: string;

  /** Historical significance */
  significance: HistoricalSignificance;

  /** Geographic scope (optional) */
  geographicScope?: {
    regions: string[];
    global: boolean;
  };

  /** Exception conditions */
  exceptions?: string[];
}

/**
 * Protected historical event
 */
export interface ProtectedEvent {
  /** Event identifier */
  id: string;

  /** Event name */
  name: string;

  /** Event date/time */
  date: Date;

  /** Event location */
  location: GeoLocation;

  /** Protection level */
  protectionLevel: ProtectionLevel;

  /** Historical significance */
  significance: HistoricalSignificance;

  /** Event category */
  category:
    | 'war'
    | 'political'
    | 'scientific'
    | 'cultural'
    | 'natural-disaster'
    | 'technological'
    | 'social'
    | 'other';

  /** Affected population */
  affectedPopulation?: number;

  /** Why this event is protected */
  protectionReason: string;

  /** Related events */
  relatedEvents?: string[];
}

// ============================================================================
// Prohibited Actions
// ============================================================================

/**
 * Prohibited action categories
 */
export enum ProhibitedActionCategory {
  FINANCIAL_EXPLOITATION = 'financial-exploitation',
  HISTORICAL_MANIPULATION = 'historical-manipulation',
  TEMPORAL_COLONIZATION = 'temporal-colonization',
  PREDATORY_TOURISM = 'predatory-tourism',
  IDENTITY_THEFT = 'identity-theft',
  TECHNOLOGY_TRANSFER = 'technology-transfer',
  BIOLOGICAL_CONTAMINATION = 'biological-contamination',
  PARADOX_CREATION = 'paradox-creation',
  PERSONAL_GAIN = 'personal-gain',
  CULTURAL_APPROPRIATION = 'cultural-appropriation',
}

/**
 * Prohibited action definition
 */
export interface ProhibitedAction {
  /** Action identifier */
  id: string;

  /** Action category */
  category: ProhibitedActionCategory;

  /** Action description */
  description: string;

  /** Specific examples */
  examples: string[];

  /** Severity if violated */
  severity: ViolationSeverity;

  /** Legal statute reference */
  statute?: string;

  /** Exceptions (if any) */
  exceptions?: string[];
}

// ============================================================================
// Ethical Review
// ============================================================================

/**
 * Review types
 */
export enum ReviewType {
  STANDARD = 'standard',
  ENHANCED = 'enhanced',
  EMERGENCY = 'emergency',
  SUPREME_COUNCIL = 'supreme-council',
}

/**
 * Review decision types
 */
export enum ReviewDecision {
  APPROVED_UNCONDITIONAL = 'approved-unconditional',
  APPROVED_CONDITIONAL = 'approved-conditional',
  CONDITIONALLY_APPROVED_PENDING = 'conditionally-approved-pending',
  DENIED_WITH_APPEAL = 'denied-with-appeal',
  PROHIBITED_PERMANENT = 'prohibited-permanent',
}

/**
 * Ethical review board member
 */
export interface ReviewBoardMember {
  /** Member ID */
  id: string;

  /** Member name */
  name: string;

  /** Member role */
  role:
    | 'philosopher'
    | 'historian'
    | 'scientist'
    | 'legal-expert'
    | 'public-advocate'
    | 'medical-ethicist'
    | 'cultural-representative';

  /** Expertise areas */
  expertise: string[];

  /** Conflicts of interest */
  conflicts?: string[];

  /** Term start date */
  termStart: Date;

  /** Term end date */
  termEnd: Date;
}

/**
 * Ethical review request
 */
export interface EthicalReview {
  /** Review ID */
  id: string;

  /** Operation request being reviewed */
  operation: TemporalOperationRequest;

  /** Review type */
  reviewType: ReviewType;

  /** Board members */
  board: ReviewBoardMember[];

  /** Review criteria scores */
  scores: {
    scientificMerit: number; // 0-30
    ethicalCompliance: number; // 0-30
    feasibility: number; // 0-20
    riskBenefitBalance: number; // 0-20
  };

  /** Total score (0-100) */
  totalScore: number;

  /** Decision */
  decision: ReviewDecision;

  /** Conditions (if conditional approval) */
  conditions?: string[];

  /** Violations detected */
  violations?: string[];

  /** Reasoning */
  reasoning: string;

  /** Dissenting opinions */
  dissentingOpinions?: {
    member: string;
    opinion: string;
  }[];

  /** Review date */
  reviewDate: Date;

  /** Decision date */
  decisionDate?: Date;

  /** Appeal deadline */
  appealDeadline?: Date;
}

/**
 * Ethics validation result
 */
export interface EthicsValidationResult {
  /** Is operation approved? */
  approved: boolean;

  /** Risk level */
  riskLevel: 'low' | 'medium' | 'high' | 'extreme';

  /** Interference level */
  interferenceLevel: InterferenceLevel;

  /** Violations (blocking) */
  violations: string[];

  /** Warnings (non-blocking) */
  warnings: string[];

  /** Conditions for approval */
  conditions: string[];

  /** Required monitoring */
  monitoring: string[];

  /** Prohibited actions list */
  prohibitedActions: ProhibitedAction[];

  /** Recommendation */
  recommendation: 'proceed' | 'proceed-with-caution' | 'modify' | 'abort';

  /** Rationale */
  rationale: string;
}

// ============================================================================
// Observer Protocols
// ============================================================================

/**
 * Observer status
 */
export enum ObserverStatus {
  PASSIVE_OBSERVER = 'passive-observer',
  LIMITED_INTERACTION = 'limited-interaction',
  ACTIVE_PARTICIPANT = 'active-participant',
  EMERGENCY_INTERVENTION = 'emergency-intervention',
}

/**
 * Observer protocol requirements
 */
export interface ObserverProtocol {
  /** Observer status level */
  status: ObserverStatus;

  /** Physical interaction allowed? */
  physicalInteractionAllowed: boolean;

  /** Communication allowed? */
  communicationAllowed: boolean;

  /** Equipment restrictions */
  equipmentRestrictions: string[];

  /** Stealth requirements */
  stealthRequirements: {
    visualConcealment: boolean;
    audioConcealment: boolean;
    emConcealment: boolean;
    biologicalIsolation: boolean;
  };

  /** Emergency intervention permitted? */
  emergencyInterventionPermitted: boolean;

  /** Maximum observation duration (seconds) */
  maxDuration: number;

  /** Required certifications */
  requiredCertifications: string[];

  /** Special conditions */
  specialConditions?: string[];
}

/**
 * Stealth technology configuration
 */
export interface StealthConfiguration {
  /** Visual cloaking enabled */
  visualCloaking: boolean;

  /** EM signature suppression */
  emSuppression: boolean;

  /** Acoustic masking */
  acousticMasking: boolean;

  /** Thermal signature reduction */
  thermalReduction: boolean;

  /** Quantum stealth mode */
  quantumStealth: boolean;

  /** Period-appropriate disguise */
  disguise?: {
    clothing: string;
    appearance: string;
    language: string;
    culturalMannerisms: string[];
  };
}

// ============================================================================
// Violations and Enforcement
// ============================================================================

/**
 * Violation severity levels
 */
export enum ViolationSeverity {
  MINOR = 'minor',
  MODERATE = 'moderate',
  SEVERE = 'severe',
  CRITICAL = 'critical',
  CATASTROPHIC = 'catastrophic',
}

/**
 * Violation record
 */
export interface ViolationRecord {
  /** Violation ID */
  id: string;

  /** Traveler ID */
  traveler: string;

  /** Operation ID */
  operationId: string;

  /** Violation type */
  category: ProhibitedActionCategory;

  /** Severity */
  severity: ViolationSeverity;

  /** Date/time of violation */
  violationDate: Date;

  /** Description */
  description: string;

  /** Timeline impact */
  timelineImpact: {
    detected: boolean;
    magnitude: 'none' | 'minimal' | 'moderate' | 'severe' | 'catastrophic';
    description: string;
    remediated: boolean;
  };

  /** Investigation status */
  investigationStatus: 'pending' | 'ongoing' | 'completed' | 'closed';

  /** Evidence */
  evidence?: string[];

  /** Witnesses */
  witnesses?: string[];

  /** Determination */
  determination?: {
    guilty: boolean;
    intent: 'accidental' | 'negligent' | 'reckless' | 'intentional';
    mitigatingFactors: string[];
    aggravatingFactors: string[];
  };

  /** Consequences */
  consequences?: ViolationConsequences;

  /** Appeal status */
  appeal?: {
    filed: boolean;
    filingDate?: Date;
    status?: 'pending' | 'granted' | 'denied';
    decision?: string;
  };
}

/**
 * Violation consequences
 */
export interface ViolationConsequences {
  /** License suspension (months, 0 = none, -1 = permanent) */
  suspension: number;

  /** Fines (USD) */
  fine: number;

  /** Criminal charges filed */
  criminalCharges: boolean;

  /** Charges description */
  charges?: string[];

  /** Imprisonment (months) */
  imprisonment?: number;

  /** Temporal incarceration */
  temporalIncarceration?: {
    duration: number; // months
    type: 'isolation' | 'timeline-locked' | 'indefinite';
  };

  /** Remediation costs (USD) */
  remediationCosts?: number;

  /** Mandatory retraining (hours) */
  retraining?: number;

  /** Probation period (months) */
  probation?: number;

  /** Permanent record */
  permanentRecord: boolean;

  /** Other penalties */
  other?: string[];
}

// ============================================================================
// Monitoring and Compliance
// ============================================================================

/**
 * Monitoring configuration
 */
export interface MonitoringConfiguration {
  /** Real-time location tracking */
  locationTracking: boolean;

  /** Activity logging */
  activityLogging: boolean;

  /** Communication monitoring */
  communicationMonitoring: boolean;

  /** Vital signs monitoring */
  vitalSignsMonitoring: boolean;

  /** Timeline integrity checking */
  timelineIntegrityChecking: boolean;

  /** Equipment usage logging */
  equipmentLogging: boolean;

  /** Automated compliance checking */
  automatedCompliance: boolean;

  /** Alert thresholds */
  alertThresholds: {
    interferenceLevel: number;
    timelineDeviation: number;
    prohibitedActionProximity: number;
  };

  /** Recording requirements */
  recording: {
    video: boolean;
    audio: boolean;
    environmental: boolean;
    allInteractions: boolean;
  };
}

/**
 * Compliance report
 */
export interface ComplianceReport {
  /** Report ID */
  id: string;

  /** Traveler/organization ID */
  subject: string;

  /** Report period */
  period: {
    start: Date;
    end: Date;
  };

  /** Operations conducted */
  operations: {
    total: number;
    approved: number;
    denied: number;
    pending: number;
  };

  /** Violations */
  violations: {
    total: number;
    minor: number;
    moderate: number;
    severe: number;
    critical: number;
    catastrophic: number;
  };

  /** Training compliance */
  training: {
    current: boolean;
    lastUpdated: Date;
    hoursCompleted: number;
    hoursRequired: number;
  };

  /** Overall compliance score (0-100) */
  complianceScore: number;

  /** Status */
  status: 'compliant' | 'non-compliant' | 'under-review' | 'suspended';

  /** Recommendations */
  recommendations: string[];

  /** Next audit date */
  nextAudit: Date;
}

// ============================================================================
// Training and Certification
// ============================================================================

/**
 * Certification levels
 */
export enum CertificationLevel {
  BASIC = 'basic',
  ADVANCED = 'advanced',
  SPECIALIST = 'specialist',
  INSTRUCTOR = 'instructor',
}

/**
 * Training certification
 */
export interface TrainingCertification {
  /** Certification ID */
  id: string;

  /** Traveler ID */
  traveler: string;

  /** Certification level */
  level: CertificationLevel;

  /** Issue date */
  issueDate: Date;

  /** Expiration date */
  expirationDate: Date;

  /** Is current/valid? */
  isCurrent: boolean;

  /** Training hours completed */
  hoursCompleted: number;

  /** Exam scores */
  examScores: {
    written: number;
    practical: number;
    ethics: number;
  };

  /** Specializations */
  specializations?: string[];

  /** Restrictions */
  restrictions?: string[];

  /** Continuing education */
  continuingEducation: {
    lastCompleted: Date;
    hoursThisYear: number;
    hoursRequired: number;
  };
}

// ============================================================================
// Interference Assessment
// ============================================================================

/**
 * Interference assessment parameters
 */
export interface InterferenceAssessment {
  /** Action being assessed */
  action: string;

  /** Timeframe of action */
  timeframe: Date | string;

  /** Location of action */
  location?: GeoLocation;

  /** Historical significance */
  historicalSignificance: HistoricalSignificance;

  /** Affected individuals */
  affectedIndividuals?: number;

  /** Cascading potential */
  cascadingPotential: 'none' | 'low' | 'medium' | 'high' | 'extreme';

  /** Reversibility */
  reversibility: 'fully' | 'partially' | 'not-reversible';
}

/**
 * Interference assessment result
 */
export interface InterferenceResult {
  /** Interference level determined */
  level: InterferenceLevel;

  /** Is action permitted? */
  permitted: boolean;

  /** Required review type */
  reviewType: ReviewType;

  /** Risk score (0-100) */
  riskScore: number;

  /** Timeline impact probability */
  timelineImpactProbability: number; // 0-1

  /** Potential consequences */
  potentialConsequences: string[];

  /** Required safeguards */
  requiredSafeguards: string[];

  /** Alternative approaches */
  alternatives: string[];

  /** Recommendation */
  recommendation: string;
}

// ============================================================================
// Timeline Impact
// ============================================================================

/**
 * Timeline deviation
 */
export interface TimelineDeviation {
  /** Deviation ID */
  id: string;

  /** When detected */
  detectionDate: Date;

  /** Deviation location in timeline */
  timelineLocation: Date;

  /** Magnitude */
  magnitude: 'negligible' | 'minor' | 'moderate' | 'major' | 'critical';

  /** Description */
  description: string;

  /** Source operation (if known) */
  sourceOperation?: string;

  /** Affected events */
  affectedEvents: string[];

  /** Cascade effects */
  cascadeEffects?: string[];

  /** Self-correcting? */
  selfCorrecting: boolean;

  /** Remediation status */
  remediation: {
    required: boolean;
    status: 'pending' | 'in-progress' | 'completed' | 'failed';
    actions?: string[];
    success?: boolean;
  };
}

/**
 * Timeline integrity check
 */
export interface TimelineIntegrityCheck {
  /** Check ID */
  id: string;

  /** Check timestamp */
  timestamp: Date;

  /** Timeline period checked */
  period: {
    start: Date;
    end: Date;
  };

  /** Integrity score (0-100) */
  integrityScore: number;

  /** Deviations detected */
  deviations: TimelineDeviation[];

  /** Causality consistency */
  causalityConsistent: boolean;

  /** Paradoxes detected */
  paradoxes: string[];

  /** Historical accuracy */
  historicalAccuracy: number; // 0-1

  /** Status */
  status: 'normal' | 'warning' | 'critical' | 'emergency';

  /** Recommendations */
  recommendations: string[];
}

// ============================================================================
// Special Cases
// ============================================================================

/**
 * Emergency intervention authorization
 */
export interface EmergencyIntervention {
  /** Authorization ID */
  id: string;

  /** Requester */
  requester: string;

  /** Emergency type */
  emergencyType:
    | 'natural-disaster'
    | 'medical-emergency'
    | 'technological-failure'
    | 'temporal-anomaly'
    | 'timeline-collapse'
    | 'other';

  /** Threat assessment */
  threat: {
    severity: 'low' | 'medium' | 'high' | 'critical';
    imminence: 'immediate' | 'hours' | 'days';
    affectedPopulation: number;
    geographicScope: string;
  };

  /** Proposed intervention */
  intervention: string;

  /** Timeline impact estimate */
  estimatedImpact: 'minimal' | 'moderate' | 'significant' | 'major';

  /** Alternative solutions considered */
  alternatives: string[];

  /** Authorization status */
  status: 'pending' | 'approved' | 'denied' | 'expired';

  /** Approver */
  approver?: string;

  /** Approval timestamp */
  approvalTimestamp?: Date;

  /** Conditions */
  conditions?: string[];

  /** Expiration */
  expiration: Date;
}

/**
 * Knowledge recovery authorization
 */
export interface KnowledgeRecoveryAuthorization {
  /** Authorization ID */
  id: string;

  /** Target knowledge */
  target: {
    description: string;
    location: GeoLocation;
    timeframe: Date;
    type: 'document' | 'oral-tradition' | 'artifact' | 'technology' | 'other';
  };

  /** Historical significance */
  significance: HistoricalSignificance;

  /** Cultural sensitivity */
  culturalSensitivity: {
    indigenousKnowledge: boolean;
    sacredInformation: boolean;
    restrictedAccess: boolean;
    communityConsent: boolean;
  };

  /** Recovery method */
  method: 'photography' | 'recording' | 'documentation' | 'preservation';

  /** Public access plan */
  publicAccess: {
    openAccess: boolean;
    restrictions?: string[];
    embargoperiod?: number; // months
    repository: string;
  };

  /** Authorization status */
  status: 'pending' | 'approved' | 'denied' | 'completed';

  /** Conditions */
  conditions: string[];
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Ethics constants
 */
export const ETHICS_CONSTANTS = {
  /** Minimum compliance score for operations (0-100) */
  MIN_COMPLIANCE_SCORE: 70,

  /** Maximum allowed violations per year (minor) */
  MAX_MINOR_VIOLATIONS: 3,

  /** Maximum allowed violations per year (moderate) */
  MAX_MODERATE_VIOLATIONS: 1,

  /** Certification validity period (months) */
  CERTIFICATION_VALIDITY: 36,

  /** Continuing education hours per year */
  CONTINUING_ED_HOURS: 8,

  /** Standard review timeline (days) */
  STANDARD_REVIEW_DAYS: 30,

  /** Enhanced review timeline (days) */
  ENHANCED_REVIEW_DAYS: 90,

  /** Emergency review timeline (hours) */
  EMERGENCY_REVIEW_HOURS: 24,

  /** Appeal deadline (days) */
  APPEAL_DEADLINE_DAYS: 30,

  /** Maximum observation duration (hours) */
  MAX_OBSERVATION_DURATION: 72,

  /** Recent history protection period (years) */
  RECENT_HISTORY_YEARS: 100,
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
 * WIA-TIME-030 error codes
 */
export enum EthicsErrorCode {
  PROHIBITED_ACTION = 'E001',
  INTERFERENCE_VIOLATION = 'E002',
  PROTECTION_VIOLATION = 'E003',
  CERTIFICATION_EXPIRED = 'E004',
  INSUFFICIENT_REVIEW = 'E005',
  TIMELINE_IMPACT_DETECTED = 'E006',
  OBSERVER_PROTOCOL_VIOLATION = 'E007',
  FINANCIAL_EXPLOITATION_DETECTED = 'E008',
  UNAUTHORIZED_INTERVENTION = 'E009',
  COMPLIANCE_FAILURE = 'E010',
}

/**
 * Time travel ethics error
 */
export class TimeTravelEthicsError extends Error {
  constructor(
    public code: EthicsErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'TimeTravelEthicsError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  TemporalRights,
  CollectiveTemporalRights,
  GeoLocation,
  TemporalOperationRequest,
  ProtectedPeriod,
  ProtectedEvent,
  ProhibitedAction,
  ReviewBoardMember,
  EthicalReview,
  EthicsValidationResult,
  ObserverProtocol,
  StealthConfiguration,
  ViolationRecord,
  ViolationConsequences,
  MonitoringConfiguration,
  ComplianceReport,
  TrainingCertification,
  InterferenceAssessment,
  InterferenceResult,
  TimelineDeviation,
  TimelineIntegrityCheck,
  EmergencyIntervention,
  KnowledgeRecoveryAuthorization,
};

export {
  EthicalPrinciple,
  InterferenceLevel,
  OperationPurpose,
  ProtectionLevel,
  HistoricalSignificance,
  ProhibitedActionCategory,
  ReviewType,
  ReviewDecision,
  ObserverStatus,
  ViolationSeverity,
  CertificationLevel,
  ETHICS_CONSTANTS,
  EthicsErrorCode,
  TimeTravelEthicsError,
};
