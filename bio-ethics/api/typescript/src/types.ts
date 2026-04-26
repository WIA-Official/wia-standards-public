/**
 * WIA-BIO-018: Bio-Ethics - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Bioethics Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Ethical Principles
// ============================================================================

/**
 * The four pillars of bioethics
 */
export enum EthicalPrinciple {
  AUTONOMY = 'autonomy',
  BENEFICENCE = 'beneficence',
  NON_MALEFICENCE = 'non-maleficence',
  JUSTICE = 'justice',
}

/**
 * Ethical principle assessment
 */
export interface PrincipleAssessment {
  /** Principle being assessed */
  principle: EthicalPrinciple;

  /** Score (0-1) */
  score: number;

  /** Supporting evidence */
  evidence: string[];

  /** Concerns or violations */
  concerns?: string[];
}

// ============================================================================
// Informed Consent
// ============================================================================

/**
 * Participant capacity levels
 */
export type ParticipantCapacity =
  | 'full' // Can provide autonomous consent
  | 'borderline' // Can consent with support
  | 'diminished' // Needs LAR, can assent
  | 'severely-impaired'; // Needs LAR, limited assent

/**
 * Informed consent form elements
 */
export interface ConsentFormElements {
  /** Research purpose and procedures disclosed */
  purposeDisclosed: boolean;

  /** Duration of participation explained */
  durationExplained: boolean;

  /** Risks disclosed */
  risksDisclosed: boolean;

  /** Benefits explained */
  benefitsExplained: boolean;

  /** Alternatives provided */
  alternativesProvided: boolean;

  /** Withdrawal rights explained */
  withdrawalRights: boolean;

  /** Privacy protection described */
  privacyProtection: boolean;

  /** Compensation policy explained */
  compensationExplained: boolean;

  /** Contact information provided */
  contactProvided: boolean;

  /** Future use of data/samples disclosed */
  futureUseDisclosed: boolean;
}

/**
 * Comprehension assessment
 */
export interface ComprehensionAssessment {
  /** Assessment method */
  method: 'teach-back' | 'quiz' | 'questionnaire' | 'interview';

  /** Score (0-1) */
  score: number;

  /** Number of questions answered correctly */
  correctResponses: number;

  /** Total number of questions */
  totalQuestions: number;

  /** Assessor identifier */
  assessor: string;

  /** Assessment date */
  date: Date;

  /** Passed (score >= 0.8) */
  passed: boolean;
}

/**
 * Signature information
 */
export interface SignatureInfo {
  /** Signed */
  signed: boolean;

  /** Signer name */
  name?: string;

  /** Signature date */
  date: Date;

  /** Signature method */
  method?: 'handwritten' | 'digital' | 'electronic' | 'verbal';

  /** Signature verification */
  verified?: boolean;
}

/**
 * Informed consent document
 */
export interface InformedConsent {
  /** Unique consent identifier */
  consentId: string;

  /** Participant identifier */
  participantId: string;

  /** Study identifier */
  studyId: string;

  /** Consent form version */
  version: string;

  /** Language */
  language: string;

  /** Consent date */
  consentDate: Date;

  /** Form elements completed */
  elements: ConsentFormElements;

  /** Participant capacity */
  participantCapacity: ParticipantCapacity;

  /** Comprehension assessment */
  comprehension?: ComprehensionAssessment;

  /** Voluntariness score (0-1) */
  voluntarinessScore: number;

  /** Signatures */
  signatures: {
    participant: SignatureInfo;
    legallyAuthorizedRepresentative?: SignatureInfo;
    witness?: SignatureInfo;
    investigator: SignatureInfo;
  };

  /** Witness present */
  witnessPresent: boolean;

  /** Consent validity */
  isValid?: boolean;

  /** Compliance score (0-1) */
  complianceScore?: number;
}

/**
 * Consent validation parameters
 */
export interface ConsentValidation {
  /** Participant ID */
  participantId: string;

  /** Study ID */
  studyId: string;

  /** Consent form elements */
  consentForm: ConsentFormElements;

  /** Participant capacity */
  participantCapacity?: ParticipantCapacity;

  /** Comprehension score (0-1) */
  comprehensionScore?: number;

  /** Witness present */
  witnessPresent: boolean;

  /** Signature information */
  signature: {
    participant: boolean;
    witness?: boolean;
    lar?: boolean;
    date: Date;
  };
}

/**
 * Consent validation result
 */
export interface ConsentValidationResult {
  /** Is consent valid */
  isValid: boolean;

  /** Compliance score (0-1) */
  complianceScore: number;

  /** Violations found */
  violations: string[];

  /** Warnings */
  warnings: string[];

  /** Recommendations */
  recommendations: string[];

  /** Required actions */
  requiredActions?: string[];
}

// ============================================================================
// IRB (Institutional Review Board)
// ============================================================================

/**
 * IRB review level
 */
export type IRBReviewLevel = 'exempt' | 'expedited' | 'full-board';

/**
 * IRB decision
 */
export type IRBDecision =
  | 'approved'
  | 'approved-with-modifications'
  | 'deferred'
  | 'rejected'
  | 'revisions-required';

/**
 * Study participant information
 */
export interface StudyParticipants {
  /** Target population */
  targetPopulation: string;

  /** Sample size */
  sampleSize: number;

  /** Inclusion criteria */
  inclusionCriteria: string[];

  /** Exclusion criteria */
  exclusionCriteria: string[];

  /** Vulnerable populations included */
  vulnerablePopulations?: VulnerablePopulation[];
}

/**
 * IRB protocol
 */
export interface IRBProtocol {
  /** Protocol ID */
  protocolId: string;

  /** Study title */
  title: string;

  /** Principal investigator */
  principalInvestigator: string;

  /** Institution */
  institution: string;

  /** Study type */
  studyType: string;

  /** Research objectives */
  objectives: string;

  /** Methodology */
  methodology: string;

  /** Participant information */
  participants: StudyParticipants;

  /** Potential risks */
  risks: string[];

  /** Potential benefits */
  benefits: string[];

  /** Consent process description */
  consentProcess: string;

  /** Data management plan */
  dataManagement: string;

  /** Risk level */
  riskLevel: RiskLevel;

  /** Submission date */
  submissionDate: Date;
}

/**
 * IRB submission
 */
export interface IRBSubmission {
  /** Submission ID */
  submissionId: string;

  /** Protocol */
  protocol: IRBProtocol;

  /** Review level */
  reviewLevel: IRBReviewLevel;

  /** Status */
  status:
    | 'submitted'
    | 'under-review'
    | 'approved'
    | 'rejected'
    | 'revisions-required'
    | 'withdrawn';

  /** Review date */
  reviewDate?: Date;

  /** Decision */
  decision?: IRBDecision;

  /** Decision rationale */
  rationale?: string;

  /** Required modifications */
  modifications?: string[];

  /** Approval expiration date */
  expirationDate?: Date;
}

// ============================================================================
// Risk Assessment
// ============================================================================

/**
 * Risk level
 */
export type RiskLevel =
  | 'minimal' // No greater than ordinary life
  | 'minor-increase' // Slightly above minimal
  | 'greater-than-minimal' // Significant risk
  | 'high'; // Substantial risk

/**
 * Risk category
 */
export enum RiskCategory {
  PHYSICAL = 'physical',
  PSYCHOLOGICAL = 'psychological',
  SOCIAL = 'social',
  ECONOMIC = 'economic',
  PRIVACY = 'privacy',
  LEGAL = 'legal',
}

/**
 * Individual risk
 */
export interface Risk {
  /** Risk description */
  description: string;

  /** Risk category */
  category: RiskCategory;

  /** Probability (0-1) */
  probability: number;

  /** Severity (0-10) */
  severity: number;

  /** Risk score (probability × severity) */
  score: number;

  /** Mitigation strategies */
  mitigation?: string[];
}

/**
 * Benefit category
 */
export enum BenefitCategory {
  DIRECT_PARTICIPANT = 'direct-participant',
  KNOWLEDGE = 'knowledge',
  SOCIETAL = 'societal',
  SCIENTIFIC = 'scientific',
}

/**
 * Individual benefit
 */
export interface Benefit {
  /** Benefit description */
  description: string;

  /** Benefit category */
  category: BenefitCategory;

  /** Expected magnitude (0-10) */
  magnitude: number;

  /** Probability of achieving (0-1) */
  probability: number;

  /** Benefit score (probability × magnitude) */
  score: number;
}

/**
 * Risk-benefit analysis
 */
export interface RiskBenefitAnalysis {
  /** Analysis ID */
  analysisId: string;

  /** Study ID */
  studyId: string;

  /** Identified risks */
  risks: Risk[];

  /** Total risk score */
  totalRiskScore: number;

  /** Identified benefits */
  benefits: Benefit[];

  /** Total benefit score */
  totalBenefitScore: number;

  /** Risk-benefit ratio */
  riskBenefitRatio: number;

  /** Justice factor (0-1) */
  justiceFactor: number;

  /** Ethical acceptability */
  isEthicallyAcceptable: boolean;

  /** Recommendation */
  recommendation: 'approve' | 'approve-with-modifications' | 'defer' | 'reject';

  /** Rationale */
  rationale: string;
}

/**
 * Ethical risk assessment parameters
 */
export interface EthicalRiskAssessment {
  /** Study type */
  studyType: string;

  /** Target population */
  targetPopulation: VulnerablePopulation | 'general-adults';

  /** Intervention type */
  interventionType:
    | 'observational'
    | 'drug'
    | 'device'
    | 'behavioral'
    | 'surgical'
    | 'gene-therapy'
    | 'other';

  /** Potential benefits */
  potentialBenefits: string[];

  /** Potential risks */
  potentialRisks: string[];

  /** Risk level */
  riskLevel: RiskLevel;
}

/**
 * Ethical risk assessment result
 */
export interface EthicalRiskResult {
  /** Ethical score (0-1) */
  ethicalScore: number;

  /** Risk-benefit ratio */
  riskBenefitRatio: number;

  /** Recommendation */
  recommendation: 'approve' | 'revise' | 'reject';

  /** Required protections */
  requiredProtections: string[];

  /** Warnings */
  warnings: string[];

  /** Rationale */
  rationale: string;
}

// ============================================================================
// Vulnerable Populations
// ============================================================================

/**
 * Vulnerable population types
 */
export type VulnerablePopulation =
  | 'children'
  | 'pregnant-women'
  | 'prisoners'
  | 'cognitively-impaired'
  | 'economically-disadvantaged'
  | 'educationally-disadvantaged'
  | 'terminally-ill'
  | 'ethnic-minorities'
  | 'refugees';

/**
 * Enhanced protection requirements
 */
export interface EnhancedProtection {
  /** Population requiring protection */
  population: VulnerablePopulation;

  /** Required protections */
  protections: string[];

  /** Additional consent requirements */
  consentRequirements: string[];

  /** IRB review requirements */
  irbRequirements: string[];

  /** Risk limitations */
  riskLimitations: string[];
}

/**
 * Child research category (45 CFR 46.404-407)
 */
export type ChildResearchCategory =
  | 'category-1' // Minimal risk
  | 'category-2' // Minor increase, prospect of direct benefit
  | 'category-3' // Minor increase, vital knowledge
  | 'category-4' // Otherwise not approvable, but opportunity to understand/prevent/treat

/**
 * Child assent requirements
 */
export interface ChildAssent {
  /** Child age */
  age: number;

  /** Assent required (typically ≥7 years) */
  assentRequired: boolean;

  /** Assent obtained */
  assentObtained?: boolean;

  /** Dissent respected */
  dissentRespected: boolean;

  /** Parental permission */
  parentalPermission: {
    oneParent: boolean;
    bothParents?: boolean;
  };
}

// ============================================================================
// Genetic Data Ethics
// ============================================================================

/**
 * Data protection level
 */
export type DataProtectionLevel =
  | 'anonymous' // No identifiers, cannot re-identify
  | 'de-identified' // Coded, can re-identify with key
  | 'confidential' // Identified but protected
  | 'public'; // Openly shared

/**
 * Genetic data record
 */
export interface GeneticData {
  /** Data ID */
  dataId: string;

  /** Participant ID */
  participantId: string;

  /** Data type */
  dataType: 'genome' | 'exome' | 'targeted-panel' | 'snp-array' | 'other';

  /** Protection level */
  protectionLevel: DataProtectionLevel;

  /** Encryption method */
  encryption?: string;

  /** Access controls */
  accessControls: string[];

  /** Consent for secondary use */
  secondaryUseConsent: boolean;

  /** Consent for data sharing */
  dataSharingConsent: boolean;

  /** Return of results policy */
  returnOfResults: 'all' | 'actionable' | 'requested' | 'none';
}

/**
 * Incidental finding
 */
export interface IncidentalFinding {
  /** Finding ID */
  findingId: string;

  /** Participant ID */
  participantId: string;

  /** Gene or variant */
  gene: string;

  /** Clinical significance */
  significance: 'pathogenic' | 'likely-pathogenic' | 'uncertain' | 'likely-benign' | 'benign';

  /** Actionable (treatment/prevention available) */
  actionable: boolean;

  /** Should return to participant */
  shouldReturn: boolean;

  /** Participant wants to know */
  participantWantsToKnow?: boolean;

  /** Clinically confirmed */
  clinicallyConfirmed: boolean;

  /** Family implications */
  familyImplications?: string;

  /** Genetic counseling provided */
  geneticCounselingProvided?: boolean;
}

// ============================================================================
// Gene Editing Ethics
// ============================================================================

/**
 * Gene editing type
 */
export type GeneEditingType =
  | 'somatic' // Non-heritable, targets body cells
  | 'germline'; // Heritable, targets reproductive cells/embryos

/**
 * Gene editing purpose
 */
export type GeneEditingPurpose =
  | 'therapeutic' // Treat or prevent disease
  | 'enhancement' // Improve normal traits
  | 'research'; // Scientific investigation

/**
 * Gene editing evaluation
 */
export interface GeneEditingEvaluation {
  /** Evaluation ID */
  evaluationId: string;

  /** Editing type */
  editingType: GeneEditingType;

  /** Purpose */
  purpose: GeneEditingPurpose;

  /** Target disease or trait */
  target: string;

  /** Scientific validity */
  scientificValidity: {
    preclinicalEvidence: boolean;
    mechanismUnderstood: boolean;
    safetyDemonstrated: boolean;
  };

  /** Risk assessment */
  risks: {
    offTargetEffects: number; // 0-1
    immuneResponse: number; // 0-1
    mosaicism: number; // 0-1
    longTermUnknown: number; // 0-1
  };

  /** Ethical considerations */
  ethical: {
    consentObtained: boolean;
    benefitsOutweighRisks: boolean;
    noAlternatives: boolean;
    equitableAccess: boolean;
  };

  /** Regulatory compliance */
  regulatoryCompliance: boolean;

  /** International consensus */
  internationalConsensus?: boolean;
}

/**
 * Gene editing decision
 */
export interface GeneEditingDecision {
  /** Decision */
  decision: 'approve' | 'approve-research-only' | 'defer' | 'reject';

  /** Rationale */
  rationale: string;

  /** Conditions */
  conditions?: string[];

  /** Required oversight */
  oversight: string[];

  /** Monitoring requirements */
  monitoring: string[];
}

// ============================================================================
// Adverse Events
// ============================================================================

/**
 * Adverse event severity
 */
export type AESeverity =
  | 'mild' // Minimal discomfort
  | 'moderate' // Interferes with daily activities
  | 'severe' // Prevents daily activities
  | 'life-threatening'
  | 'fatal';

/**
 * Causality assessment
 */
export type Causality =
  | 'unrelated'
  | 'unlikely'
  | 'possible'
  | 'probable'
  | 'definite';

/**
 * Adverse event
 */
export interface AdverseEvent {
  /** Event ID */
  eventId: string;

  /** Participant ID (de-identified) */
  participantId: string;

  /** Study ID */
  studyId: string;

  /** Event description */
  description: string;

  /** Onset date */
  onsetDate: Date;

  /** Resolution date */
  resolutionDate?: Date;

  /** Severity */
  severity: AESeverity;

  /** Causality to intervention */
  causality: Causality;

  /** Outcome */
  outcome: 'recovered' | 'recovering' | 'not-recovered' | 'fatal' | 'unknown';

  /** Actions taken */
  actionsTaken: string[];

  /** Reported to IRB */
  reportedToIRB: boolean;

  /** Report date */
  reportDate?: Date;
}

// ============================================================================
// Compliance and Monitoring
// ============================================================================

/**
 * Protocol deviation
 */
export interface ProtocolDeviation {
  /** Deviation ID */
  deviationId: string;

  /** Study ID */
  studyId: string;

  /** Deviation description */
  description: string;

  /** Date occurred */
  dateOccurred: Date;

  /** Category */
  category: 'major' | 'minor';

  /** Impact on participants */
  participantImpact: 'none' | 'minimal' | 'moderate' | 'significant';

  /** Corrective actions */
  correctiveActions: string[];

  /** Reported to IRB */
  reportedToIRB: boolean;
}

/**
 * Audit result
 */
export interface AuditResult {
  /** Audit ID */
  auditId: string;

  /** Study ID */
  studyId: string;

  /** Audit date */
  auditDate: Date;

  /** Auditor */
  auditor: string;

  /** Findings */
  findings: {
    category: string;
    description: string;
    severity: 'critical' | 'major' | 'minor';
  }[];

  /** Compliance score (0-1) */
  complianceScore: number;

  /** Recommendations */
  recommendations: string[];

  /** Follow-up required */
  followUpRequired: boolean;
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
// Error Types
// ============================================================================

/**
 * WIA-BIO-018 error codes
 */
export enum BioEthicsErrorCode {
  INCOMPLETE_CONSENT = 'E001',
  INSUFFICIENT_COMPREHENSION = 'E002',
  COERCION_DETECTED = 'E003',
  CAPACITY_CONCERNS = 'E004',
  IRB_APPROVAL_REQUIRED = 'E005',
  PRIVACY_VIOLATION = 'E006',
  UNACCEPTABLE_RISK = 'E007',
  VULNERABLE_POPULATION = 'E008',
  INVALID_GENE_EDITING = 'E009',
  REGULATORY_NONCOMPLIANCE = 'E010',
}

/**
 * Bio-ethics error
 */
export class BioEthicsError extends Error {
  constructor(
    public code: BioEthicsErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'BioEthicsError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  // Consent
  ConsentFormElements,
  ComprehensionAssessment,
  SignatureInfo,
  InformedConsent,
  ConsentValidation,
  ConsentValidationResult,

  // IRB
  StudyParticipants,
  IRBProtocol,
  IRBSubmission,

  // Risk
  Risk,
  Benefit,
  RiskBenefitAnalysis,
  EthicalRiskAssessment,
  EthicalRiskResult,

  // Vulnerable Populations
  EnhancedProtection,
  ChildAssent,

  // Genetic Data
  GeneticData,
  IncidentalFinding,

  // Gene Editing
  GeneEditingEvaluation,
  GeneEditingDecision,

  // Adverse Events
  AdverseEvent,
  ProtocolDeviation,
  AuditResult,

  // Principles
  PrincipleAssessment,
};

export {
  EthicalPrinciple,
  RiskCategory,
  BenefitCategory,
  BioEthicsErrorCode,
  BioEthicsError,
};
