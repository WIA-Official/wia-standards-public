/**
 * WIA-AUG-015: Transhumanism Protocol - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Transhumanism Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Enhancement Stage Types
// ============================================================================

/**
 * Human enhancement stages according to transhumanist taxonomy
 */
export enum EnhancementStage {
  /** Baseline human (no enhancement) */
  BASELINE = 'BASELINE',

  /** Human Plus 1: Minor augmentations */
  H_PLUS_1 = 'H_PLUS_1',

  /** Human Plus 2: Significant augmentations */
  H_PLUS_2 = 'H_PLUS_2',

  /** Human Plus 3: Major augmentations, approaching posthuman */
  H_PLUS_3 = 'H_PLUS_3',

  /** Posthuman: Beyond human biological constraints */
  POSTHUMAN = 'POSTHUMAN',
}

/**
 * Capability domains for enhancement
 */
export enum CapabilityDomain {
  /** Physical capabilities (strength, endurance, healing) */
  PHYSICAL = 'PHYSICAL',

  /** Cognitive capabilities (intelligence, memory, processing) */
  COGNITIVE = 'COGNITIVE',

  /** Sensory capabilities (vision, hearing, new senses) */
  SENSORY = 'SENSORY',

  /** Lifespan and longevity */
  LIFESPAN = 'LIFESPAN',

  /** Emotional regulation and control */
  EMOTIONAL = 'EMOTIONAL',

  /** Consciousness and subjective experience */
  CONSCIOUSNESS = 'CONSCIOUSNESS',
}

/**
 * Transition types for moving between stages
 */
export enum TransitionType {
  /** Gradual biological enhancement over time */
  GRADUAL = 'GRADUAL',

  /** Hybrid biological-synthetic integration */
  HYBRID = 'HYBRID',

  /** Mind uploading to digital substrate */
  UPLOAD = 'UPLOAD',

  /** Human-AI consciousness merger */
  MERGER = 'MERGER',

  /** Complete substrate change (e.g., synthetic body) */
  SUBSTRATE_CHANGE = 'SUBSTRATE_CHANGE',
}

/**
 * Risk levels for transhuman transitions
 */
export enum RiskLevel {
  LOW = 'LOW',
  MODERATE = 'MODERATE',
  HIGH = 'HIGH',
  EXTREME = 'EXTREME',
}

/**
 * Consciousness continuity preservation methods
 */
export enum ContinuityMethod {
  /** Gradual neuron replacement */
  GRADUAL_REPLACEMENT = 'GRADUAL_REPLACEMENT',

  /** Whole brain emulation */
  WHOLE_BRAIN_EMULATION = 'WHOLE_BRAIN_EMULATION',

  /** Consciousness streaming */
  STREAMING = 'STREAMING',

  /** Pattern preservation and reconstruction */
  PATTERN_PRESERVATION = 'PATTERN_PRESERVATION',

  /** Quantum state transfer */
  QUANTUM_TRANSFER = 'QUANTUM_TRANSFER',
}

// ============================================================================
// Capability Measurement Types
// ============================================================================

/**
 * Capability level measurement
 */
export interface CapabilityLevel {
  /** Capability domain */
  domain: CapabilityDomain;

  /** Current level (1.0 = baseline human average) */
  currentLevel: number;

  /** Baseline level before any enhancement */
  baselineLevel: number;

  /** Enhancement factor (currentLevel / baselineLevel) */
  enhancementFactor: number;

  /** Maximum theoretical level */
  theoreticalMax: number;

  /** Safe operational limit */
  safeLimit: number;

  /** Measurement timestamp */
  timestamp: Date;
}

/**
 * Comprehensive capability assessment
 */
export interface CapabilityAssessment {
  /** Assessment ID */
  assessmentId: string;

  /** Subject ID */
  subjectId: string;

  /** Current enhancement stage */
  currentStage: EnhancementStage;

  /** Capability levels by domain */
  capabilities: Record<CapabilityDomain, CapabilityLevel>;

  /** Overall enhancement index */
  enhancementIndex: number;

  /** Substrate type (biological, hybrid, digital, etc.) */
  substrate: SubstrateType;

  /** Assessment date */
  assessmentDate: Date;
}

/**
 * Substrate types
 */
export enum SubstrateType {
  /** Pure biological */
  BIOLOGICAL = 'BIOLOGICAL',

  /** Biological with technological augmentation */
  BIO_TECH_HYBRID = 'BIO_TECH_HYBRID',

  /** Primarily digital with biological interface */
  DIGITAL_BIO_HYBRID = 'DIGITAL_BIO_HYBRID',

  /** Pure digital/computational */
  DIGITAL = 'DIGITAL',

  /** Quantum computational substrate */
  QUANTUM = 'QUANTUM',

  /** Novel synthetic biology */
  SYNTHETIC_BIO = 'SYNTHETIC_BIO',
}

// ============================================================================
// Transhuman Assessment Types
// ============================================================================

/**
 * Transhuman assessment result
 */
export interface TranshumanAssessment {
  /** Assessment identifier */
  assessmentId: string;

  /** Subject identifier */
  subjectId: string;

  /** Current enhancement stage */
  current_stage: EnhancementStage;

  /** Capability assessment */
  capabilities: CapabilityAssessment;

  /** Risk assessment */
  risks: RiskAssessment;

  /** Recommended timeline for next stage */
  timeline: TransitionTimeline;

  /** Identity continuity score (0-1) */
  identityContinuity: number;

  /** Consciousness integrity score (0-1) */
  consciousnessIntegrity: number;

  /** Assessment timestamp */
  timestamp: Date;
}

/**
 * Risk assessment for transhuman transition
 */
export interface RiskAssessment {
  /** Overall risk level */
  overallRisk: RiskLevel;

  /** Physical risks */
  physical: {
    level: RiskLevel;
    factors: string[];
    mitigations: string[];
  };

  /** Cognitive risks */
  cognitive: {
    level: RiskLevel;
    factors: string[];
    mitigations: string[];
  };

  /** Identity risks */
  identity: {
    level: RiskLevel;
    factors: string[];
    mitigations: string[];
  };

  /** Social risks */
  social: {
    level: RiskLevel;
    factors: string[];
    mitigations: string[];
  };

  /** Existential risks */
  existential: {
    level: RiskLevel;
    factors: string[];
    mitigations: string[];
  };

  /** Risk mitigation plan */
  mitigationPlan: string[];

  /** Contingency procedures */
  contingencies: string[];
}

/**
 * Transition timeline
 */
export interface TransitionTimeline {
  /** Current stage */
  currentStage: EnhancementStage;

  /** Target stage */
  targetStage: EnhancementStage;

  /** Estimated duration (days) */
  estimatedDuration: number;

  /** Recommended transition type */
  recommendedType: TransitionType;

  /** Phases */
  phases: TransitionPhase[];

  /** Milestones */
  milestones: Milestone[];

  /** Critical checkpoints */
  checkpoints: Checkpoint[];
}

/**
 * Transition phase
 */
export interface TransitionPhase {
  /** Phase name */
  name: string;

  /** Phase number */
  phase: number;

  /** Duration (days) */
  duration: number;

  /** Target capabilities */
  targetCapabilities: Partial<Record<CapabilityDomain, number>>;

  /** Required procedures */
  procedures: string[];

  /** Success criteria */
  successCriteria: string[];

  /** Risks */
  risks: string[];
}

/**
 * Milestone in transition
 */
export interface Milestone {
  /** Milestone ID */
  id: string;

  /** Milestone name */
  name: string;

  /** Target date */
  targetDate: Date;

  /** Completion status */
  completed: boolean;

  /** Success criteria */
  criteria: string[];

  /** Validation method */
  validation: string;
}

/**
 * Checkpoint for monitoring
 */
export interface Checkpoint {
  /** Checkpoint ID */
  id: string;

  /** Checkpoint name */
  name: string;

  /** Schedule (days from start) */
  scheduledDay: number;

  /** Assessments required */
  assessments: string[];

  /** Go/No-go criteria */
  goNoGoCriteria: string[];

  /** Rollback procedures if needed */
  rollbackProcedures: string[];
}

// ============================================================================
// Transition Planning Types
// ============================================================================

/**
 * Transition plan
 */
export interface TransitionPlan {
  /** Plan ID */
  planId: string;

  /** Subject ID */
  subjectId: string;

  /** Transition type */
  type: TransitionType;

  /** From stage */
  fromStage: EnhancementStage;

  /** To stage */
  toStage: EnhancementStage;

  /** Timeline */
  timeline: TransitionTimeline;

  /** Technologies required */
  technologies: Technology[];

  /** Continuity preservation method */
  continuityMethod: ContinuityMethod;

  /** Risk assessment */
  riskAssessment: RiskAssessment;

  /** Ethical considerations */
  ethicalConsiderations: string[];

  /** Reversibility */
  reversible: boolean;

  /** Backup plan */
  backupPlan?: string;

  /** Created date */
  createdDate: Date;
}

/**
 * Technology requirement
 */
export interface Technology {
  /** Technology name */
  name: string;

  /** Technology type */
  type: TechnologyType;

  /** Maturity level (TRL 1-9) */
  maturityLevel: number;

  /** Availability */
  available: boolean;

  /** Required capabilities */
  capabilities: string[];

  /** Safety rating (0-1) */
  safetyRating: number;

  /** Estimated cost */
  cost?: number;
}

/**
 * Technology types
 */
export enum TechnologyType {
  GENETIC_ENGINEERING = 'GENETIC_ENGINEERING',
  NANOTECHNOLOGY = 'NANOTECHNOLOGY',
  NEURAL_INTERFACE = 'NEURAL_INTERFACE',
  PROSTHETIC = 'PROSTHETIC',
  COGNITIVE_ENHANCEMENT = 'COGNITIVE_ENHANCEMENT',
  LIFE_EXTENSION = 'LIFE_EXTENSION',
  MIND_UPLOADING = 'MIND_UPLOADING',
  ARTIFICIAL_GENERAL_INTELLIGENCE = 'ARTIFICIAL_GENERAL_INTELLIGENCE',
  QUANTUM_COMPUTING = 'QUANTUM_COMPUTING',
  SYNTHETIC_BIOLOGY = 'SYNTHETIC_BIOLOGY',
}

// ============================================================================
// Consciousness and Identity Types
// ============================================================================

/**
 * Consciousness continuity assessment
 */
export interface ContinuityAssessment {
  /** Assessment ID */
  assessmentId: string;

  /** Subject ID */
  subjectId: string;

  /** Continuity score (0-1, 1 = perfect continuity) */
  continuityScore: number;

  /** Identity preservation score (0-1) */
  identityScore: number;

  /** Memory integrity (0-1) */
  memoryIntegrity: number;

  /** Personality stability (0-1) */
  personalityStability: number;

  /** Self-recognition score (0-1) */
  selfRecognition: number;

  /** Subjective continuity (self-reported, 0-1) */
  subjectiveContinuity: number;

  /** Neural pattern similarity (0-1) */
  neuralPatternSimilarity: number;

  /** Timestamp */
  timestamp: Date;
}

/**
 * Identity preservation protocol
 */
export interface IdentityProtocol {
  /** Protocol ID */
  protocolId: string;

  /** Protocol name */
  name: string;

  /** Preservation methods */
  methods: ContinuityMethod[];

  /** Memory backup strategy */
  memoryBackup: MemoryBackupStrategy;

  /** Personality anchoring */
  personalityAnchoring: string[];

  /** Validation tests */
  validationTests: string[];

  /** Rollback triggers */
  rollbackTriggers: string[];

  /** Minimum continuity threshold */
  minimumContinuity: number;
}

/**
 * Memory backup strategy
 */
export interface MemoryBackupStrategy {
  /** Backup frequency */
  frequency: 'continuous' | 'hourly' | 'daily' | 'weekly';

  /** Backup locations */
  locations: string[];

  /** Redundancy level */
  redundancy: number;

  /** Encryption enabled */
  encrypted: boolean;

  /** Verification method */
  verification: string;

  /** Retention period (days) */
  retentionPeriod: number;
}

// ============================================================================
// Morphological Freedom Types
// ============================================================================

/**
 * Morphological freedom request
 */
export interface MorphologyRequest {
  /** Request ID */
  requestId: string;

  /** Subject ID */
  subjectId: string;

  /** Desired morphology */
  targetMorphology: MorphologyType;

  /** Current morphology */
  currentMorphology: MorphologyType;

  /** Justification */
  justification: string;

  /** Reversibility required */
  reversibilityRequired: boolean;

  /** Consent confirmation */
  consent: ConsentRecord;

  /** Request date */
  requestDate: Date;
}

/**
 * Morphology types
 */
export enum MorphologyType {
  /** Standard human biological form */
  STANDARD_HUMAN = 'STANDARD_HUMAN',

  /** Enhanced human (still recognizable as human) */
  ENHANCED_HUMAN = 'ENHANCED_HUMAN',

  /** Hybrid human-machine */
  HYBRID_CYBORG = 'HYBRID_CYBORG',

  /** Primarily synthetic/robotic */
  SYNTHETIC = 'SYNTHETIC',

  /** Digital/uploaded consciousness */
  DIGITAL_ENTITY = 'DIGITAL_ENTITY',

  /** Novel biological form */
  NOVEL_BIOLOGICAL = 'NOVEL_BIOLOGICAL',

  /** Non-humanoid form */
  NON_HUMANOID = 'NON_HUMANOID',

  /** Distributed consciousness across multiple substrates */
  DISTRIBUTED = 'DISTRIBUTED',
}

/**
 * Consent record
 */
export interface ConsentRecord {
  /** Consent ID */
  consentId: string;

  /** Subject ID */
  subjectId: string;

  /** Consent type */
  type: 'informed' | 'ongoing' | 'revocable';

  /** Consent given */
  granted: boolean;

  /** Consent date */
  date: Date;

  /** Witness IDs */
  witnesses?: string[];

  /** Revocable */
  revocable: boolean;

  /** Expiration date */
  expirationDate?: Date;

  /** Digital signature */
  signature: string;
}

// ============================================================================
// Governance Types
// ============================================================================

/**
 * Species transition governance
 */
export interface GovernanceFramework {
  /** Framework ID */
  frameworkId: string;

  /** Framework name */
  name: string;

  /** Ethical guidelines */
  ethicalGuidelines: string[];

  /** Regulatory requirements */
  regulatoryRequirements: string[];

  /** Safety standards */
  safetyStandards: string[];

  /** Oversight bodies */
  oversightBodies: string[];

  /** Approval process */
  approvalProcess: ApprovalProcess;

  /** Monitoring requirements */
  monitoringRequirements: string[];

  /** Intervention criteria */
  interventionCriteria: string[];
}

/**
 * Approval process
 */
export interface ApprovalProcess {
  /** Required approvals */
  requiredApprovals: string[];

  /** Review boards */
  reviewBoards: string[];

  /** Assessment criteria */
  criteria: string[];

  /** Minimum approval threshold */
  threshold: number;

  /** Appeal process available */
  appealAvailable: boolean;

  /** Typical duration (days) */
  typicalDuration: number;
}

/**
 * Existential risk assessment
 */
export interface ExistentialRisk {
  /** Risk ID */
  riskId: string;

  /** Risk category */
  category: ExistentialRiskCategory;

  /** Probability (0-1) */
  probability: number;

  /** Impact severity (0-1) */
  severity: number;

  /** Risk score (probability × severity) */
  riskScore: number;

  /** Description */
  description: string;

  /** Mitigation strategies */
  mitigations: string[];

  /** Monitoring indicators */
  indicators: string[];

  /** Emergency protocols */
  emergencyProtocols: string[];
}

/**
 * Existential risk categories
 */
export enum ExistentialRiskCategory {
  /** Loss of human values */
  VALUE_DRIFT = 'VALUE_DRIFT',

  /** Uncontrolled AI emergence */
  AI_ALIGNMENT = 'AI_ALIGNMENT',

  /** Irreversible identity loss */
  IDENTITY_LOSS = 'IDENTITY_LOSS',

  /** Consciousness fragmentation */
  CONSCIOUSNESS_FRAGMENTATION = 'CONSCIOUSNESS_FRAGMENTATION',

  /** Species extinction */
  EXTINCTION = 'EXTINCTION',

  /** Social collapse */
  SOCIAL_COLLAPSE = 'SOCIAL_COLLAPSE',

  /** Technology misuse */
  TECHNOLOGY_MISUSE = 'TECHNOLOGY_MISUSE',

  /** Substrate failure */
  SUBSTRATE_FAILURE = 'SUBSTRATE_FAILURE',
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Transhumanism protocol constants
 */
export const TRANSHUMAN_CONSTANTS = {
  /** Enhancement stage thresholds */
  STAGE_THRESHOLDS: {
    H_PLUS_1: 1.5, // 1.5x baseline average capability
    H_PLUS_2: 3.0, // 3x baseline
    H_PLUS_3: 10.0, // 10x baseline
    POSTHUMAN: 100.0, // 100x baseline
  },

  /** Minimum continuity scores */
  MIN_CONTINUITY: {
    IDENTITY: 0.95, // 95% identity preservation
    MEMORY: 0.90, // 90% memory integrity
    PERSONALITY: 0.85, // 85% personality stability
    CONSCIOUSNESS: 0.98, // 98% consciousness integrity
  },

  /** Risk thresholds */
  RISK_THRESHOLDS: {
    ACCEPTABLE: 0.3,
    WARNING: 0.5,
    CRITICAL: 0.7,
    UNACCEPTABLE: 0.9,
  },

  /** Maximum enhancement rates (per domain, per year) */
  MAX_ENHANCEMENT_RATE: {
    PHYSICAL: 2.0, // 2x per year
    COGNITIVE: 1.5, // 1.5x per year
    SENSORY: 2.5, // 2.5x per year
    LIFESPAN: 1.2, // 1.2x per year
    EMOTIONAL: 1.3, // 1.3x per year
    CONSCIOUSNESS: 1.1, // 1.1x per year (most conservative)
  },

  /** Transition safety parameters */
  TRANSITION_SAFETY: {
    MIN_PREPARATION_DAYS: 90,
    MIN_MONITORING_DAYS: 365,
    MAX_PARALLEL_DOMAINS: 2, // Max 2 domains enhanced simultaneously
    REQUIRED_CHECKPOINTS: 5,
    MIN_ROLLBACK_WINDOW_DAYS: 30,
  },
} as const;

// ============================================================================
// Error Types
// ============================================================================

/**
 * Transhumanism protocol error codes
 */
export enum TranshumanErrorCode {
  INVALID_STAGE = 'TH001',
  INVALID_TRANSITION = 'TH002',
  CONTINUITY_BREACH = 'TH003',
  IDENTITY_LOSS = 'TH004',
  RISK_THRESHOLD_EXCEEDED = 'TH005',
  CONSENT_REQUIRED = 'TH006',
  TECHNOLOGY_UNAVAILABLE = 'TH007',
  GOVERNANCE_VIOLATION = 'TH008',
  EXISTENTIAL_RISK = 'TH009',
  SUBSTRATE_INCOMPATIBLE = 'TH010',
}

/**
 * Transhumanism protocol error
 */
export class TranshumanError extends Error {
  constructor(
    public code: TranshumanErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'TranshumanError';
  }
}

// ============================================================================
// Export All Types
// ============================================================================

export type {
  CapabilityLevel,
  CapabilityAssessment,
  TranshumanAssessment,
  RiskAssessment,
  TransitionTimeline,
  TransitionPhase,
  Milestone,
  Checkpoint,
  TransitionPlan,
  Technology,
  ContinuityAssessment,
  IdentityProtocol,
  MemoryBackupStrategy,
  MorphologyRequest,
  ConsentRecord,
  GovernanceFramework,
  ApprovalProcess,
  ExistentialRisk,
};

export {
  EnhancementStage,
  CapabilityDomain,
  TransitionType,
  RiskLevel,
  ContinuityMethod,
  SubstrateType,
  TechnologyType,
  MorphologyType,
  ExistentialRiskCategory,
  TRANSHUMAN_CONSTANTS,
  TranshumanErrorCode,
  TranshumanError,
};
