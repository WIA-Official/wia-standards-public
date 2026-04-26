/**
 * WIA-AUG-012: Augmentation Ethics - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Human Augmentation Ethics Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Ethical Principles Types
// ============================================================================

/**
 * Six core ethical principles for human augmentation
 */
export enum EthicalPrinciple {
  AUTONOMY = 'AUTONOMY',
  BENEFICENCE = 'BENEFICENCE',
  NON_MALEFICENCE = 'NON_MALEFICENCE',
  JUSTICE = 'JUSTICE',
  DIGNITY = 'DIGNITY',
  AUTHENTICITY = 'AUTHENTICITY'
}

/**
 * Scores for each ethical principle (0-10 scale)
 */
export interface PrincipleScores {
  autonomy: number;
  beneficence: number;
  nonMaleficence: number;
  justice: number;
  dignity: number;
  authenticity: number;
}

/**
 * Ethical concern with severity and description
 */
export interface EthicalConcern {
  principle: EthicalPrinciple;
  severity: 'low' | 'moderate' | 'high' | 'critical';
  description: string;
  mitigation?: string;
}

// ============================================================================
// Consent Types
// ============================================================================

/**
 * Informed consent levels based on augmentation type
 */
export enum ConsentLevel {
  BASIC = 'BASIC',                     // Standard medical consent
  ENHANCED = 'ENHANCED',               // Detailed risks/benefits
  COMPREHENSIVE = 'COMPREHENSIVE',     // Full long-term implications
  EXPERIMENTAL = 'EXPERIMENTAL'        // Complete uncertainty disclosure
}

/**
 * Consent documentation
 */
export interface ConsentDocumentation {
  level: ConsentLevel;
  dateObtained: Date;

  /** Information disclosed */
  disclosures: {
    nature: boolean;
    benefits: boolean;
    risks: boolean;
    alternatives: boolean;
    longTerm: boolean;
    identity: boolean;
    social: boolean;
    reversibility: boolean;
  };

  /** Comprehension verification */
  comprehension: {
    assessed: boolean;
    score: number;  // 0-10
    method: string;
  };

  /** Voluntariness assessment */
  voluntariness: {
    coercionScreened: boolean;
    coercionDetected: boolean;
    pressure: 'none' | 'minimal' | 'moderate' | 'high';
  };

  /** Capacity evaluation */
  capacity: {
    evaluated: boolean;
    hasCapacity: boolean;
    evaluator: string;
    date: Date;
  };

  /** Cooling-off period */
  coolingOff: {
    required: boolean;
    duration: number;  // days
    completed: boolean;
  };

  /** Signatures */
  signatures: {
    subject: string;
    witness?: string;
    ethicsOfficer?: string;
    provider: string;
  };

  /** Sessions (for comprehensive/experimental) */
  sessions?: ConsentSession[];
}

/**
 * Individual consent session record
 */
export interface ConsentSession {
  date: Date;
  duration: number;  // minutes
  topics: string[];
  questionsAddressed: string[];
  comprehensionVerified: boolean;
  notes: string;
}

// ============================================================================
// Augmentation Classification Types
// ============================================================================

/**
 * Augmentation type classification
 */
export enum AugmentationType {
  THERAPEUTIC = 'THERAPEUTIC',     // Treating disease/disability
  RESTORATIVE = 'RESTORATIVE',     // Restoring normal function
  ENHANCEMENT = 'ENHANCEMENT',     // Exceeding normal capabilities
  EXPERIMENTAL = 'EXPERIMENTAL'    // Unproven/research-stage
}

/**
 * Classification input parameters
 */
export interface AugmentationClassificationInput {
  currentFunction: number;      // 0-100, baseline = 50
  targetFunction: number;        // 0-100
  medicalNecessity: boolean;
  pathologyPresent: boolean;
  provenEffective: boolean;
}

/**
 * Detailed augmentation information
 */
export interface AugmentationDetails {
  type: AugmentationType;
  category: 'cognitive' | 'physical' | 'sensory' | 'emotional' | 'other';
  description: string;
  reversibility: number;         // 0-1 (0 = irreversible, 1 = fully reversible)
  riskLevel: 'minimal' | 'low' | 'moderate' | 'high' | 'severe';
  duration: 'temporary' | 'long-term' | 'permanent';
  invasiveness: 'none' | 'minimal' | 'moderate' | 'high';
}

// ============================================================================
// Equity and Access Types
// ============================================================================

/**
 * Access barriers assessment
 */
export interface AccessBarriers {
  economic: number;      // 0-10
  geographic: number;
  social: number;
  systemic: number;
}

/**
 * Distribution metric for equity analysis
 */
export interface DistributionMetric {
  giniCoefficient: number;           // 0-1 (0 = perfect equality)
  representationRatio: number;        // Actual / Expected
  disparityScore: number;             // 0-100
}

/**
 * Equity assessment result
 */
export interface EquityAssessment {
  accessBarriers: AccessBarriers;

  demographicDistribution: {
    income: DistributionMetric;
    race: DistributionMetric;
    geography: DistributionMetric;
    disability: DistributionMetric;
  };

  equityScore: number;  // 0-100
  concerns: string[];
  mitigations: string[];
  compliant: boolean;
}

/**
 * Equity context for assessment
 */
export interface EquityContext {
  augmentationType: AugmentationType;
  cost: number;
  availability: 'universal' | 'widespread' | 'limited' | 'rare';
  insuranceCoverage: boolean;
  subsidiesAvailable: boolean;
  geographicRestrictions: string[];
}

// ============================================================================
// Coercion Types
// ============================================================================

/**
 * Context where coercion may occur
 */
export type CoercionContext =
  | 'occupational'
  | 'military'
  | 'educational'
  | 'social'
  | 'familial'
  | 'medical'
  | 'research';

/**
 * Coercion indicators
 */
export interface CoercionIndicators {
  mandatoryRequirement: boolean;
  employmentConsequence: boolean;
  peerPressure: boolean;
  authorityPressure: boolean;
  financialIncentive: boolean;
  limitedAlternatives: boolean;
  powerImbalance: boolean;
  timeConstraint: boolean;
}

/**
 * Coercion risk assessment
 */
export interface CoercionRisk {
  context: CoercionContext;
  indicators: CoercionIndicators;
  riskLevel: 'none' | 'low' | 'moderate' | 'high' | 'severe';
  score: number;  // 0-10
}

/**
 * Coercion check result
 */
export interface CoercionCheckResult {
  coercionDetected: boolean;
  riskLevel: 'none' | 'low' | 'moderate' | 'high' | 'severe';
  concerns: string[];
  interventions: string[];
  proceedRecommendation: boolean;
}

// ============================================================================
// Identity and Authenticity Types
// ============================================================================

/**
 * Identity dimensions that may be impacted
 */
export interface IdentityDimensions {
  psychological: {
    memory: number;          // Impact 0-10
    personality: number;
    consciousness: number;
    emotions: number;
  };

  physical: {
    embodiment: number;
    appearance: number;
    capabilities: number;
    sensorimotor: number;
  };

  narrative: {
    continuity: number;
    meaning: number;
    autobiography: number;
  };

  social: {
    relationships: number;
    roles: number;
    community: number;
    identity: number;
  };

  values: {
    beliefs: number;
    commitments: number;
    goals: number;
    worldview: number;
  };
}

/**
 * Identity impact assessment result
 */
export interface IdentityImpact {
  dimensions: IdentityDimensions;
  overallScore: number;  // 0-100
  level: 'minimal' | 'moderate' | 'substantial' | 'severe';
  concerns: string[];
  recommendations: string[];
  authenticityPreserved: boolean;
}

/**
 * Authenticity criteria evaluation
 */
export interface AuthenticityAssessment {
  valueAlignment: boolean;
  narrativeCoherence: boolean;
  psychologicalContinuity: boolean;
  relationalIdentity: boolean;
  selfRecognition: boolean;
  overallAuthenticity: boolean;
  concerns: string[];
}

// ============================================================================
// Reversibility Types
// ============================================================================

/**
 * Reversibility classification levels
 */
export enum ReversibilityLevel {
  FULLY_REVERSIBLE = 'FULLY_REVERSIBLE',           // 90-100%
  LARGELY_REVERSIBLE = 'LARGELY_REVERSIBLE',       // 70-89%
  PARTIALLY_REVERSIBLE = 'PARTIALLY_REVERSIBLE',   // 40-69%
  MINIMALLY_REVERSIBLE = 'MINIMALLY_REVERSIBLE',   // 10-39%
  IRREVERSIBLE = 'IRREVERSIBLE'                    // 0-9%
}

/**
 * Reversibility profile for augmentation
 */
export interface ReversibilityProfile {
  level: ReversibilityLevel;
  restorationPercentage: number;  // 0-100

  reversalProcess: {
    surgical: boolean;
    duration: number;  // days
    risk: 'low' | 'moderate' | 'high';
    cost: number;
    availability: boolean;
  };

  permanentChanges: string[];

  recovery: {
    physical: number;       // days
    psychological: number;  // days
    functional: number;     // days
  };

  justificationRequired: boolean;
  justification?: string;
}

// ============================================================================
// Vulnerable Population Types
// ============================================================================

/**
 * Vulnerable population categories
 */
export type VulnerablePopulation =
  | 'children'
  | 'cognitive_impairment'
  | 'economically_disadvantaged'
  | 'institutionalized'
  | 'minority'
  | 'military'
  | 'prisoners'
  | 'refugees';

/**
 * Subject profile
 */
export interface SubjectProfile {
  id: string;  // anonymized
  age: number;

  /** Decision-making capacity */
  hasDecisionCapacity: boolean;
  capacityAssessment?: CapacityAssessment;

  /** Vulnerable population status */
  isVulnerable: boolean;
  vulnerableCategories: VulnerablePopulation[];

  /** Socioeconomic factors */
  socioeconomicStatus: 'low' | 'middle' | 'high';
  hasInsurance: boolean;

  /** Health status */
  healthStatus: 'excellent' | 'good' | 'fair' | 'poor';
  medicalConditions: string[];

  /** Cultural/linguistic */
  primaryLanguage: string;
  culturalConsiderations: string[];
}

/**
 * Decision-making capacity assessment
 */
export interface CapacityAssessment {
  date: Date;
  evaluator: string;

  domains: {
    understanding: number;    // 0-10
    appreciation: number;
    reasoning: number;
    expression: number;
  };

  overallCapacity: boolean;
  notes: string;
  recommendations: string[];
}

/**
 * Vulnerable population protections
 */
export interface VulnerableProtections {
  required: boolean;
  categories: VulnerablePopulation[];

  additionalRequirements: {
    independentAdvocacy: boolean;
    ethicsReview: boolean;
    judicialOversight: boolean;
    extendedCoolingOff: boolean;
    communityConsultation: boolean;
    surrogateConsent: boolean;
  };

  restrictions: {
    enhancementProhibited: boolean;
    experimentalProhibited: boolean;
    irreversibleProhibited: boolean;
    therapeuticOnly: boolean;
  };

  supportServices: string[];
}

// ============================================================================
// Decision Context Types
// ============================================================================

/**
 * Context of augmentation decision
 */
export interface DecisionContext {
  isPressured: boolean;
  isInformed: boolean;
  hasAlternatives: boolean;
  timeConstrained: boolean;

  coercionContext?: CoercionContext;
  externalFactors: string[];

  alternatives: {
    nonAugmentation: string[];
    alternativeAugmentations: string[];
    considered: boolean;
  };
}

// ============================================================================
// Ethical Assessment Types
// ============================================================================

/**
 * Comprehensive ethical assessment request
 */
export interface EthicalAssessmentRequest {
  augmentation: AugmentationDetails;
  subject: SubjectProfile;
  context: DecisionContext;
  consent?: ConsentDocumentation;
}

/**
 * Ethical assessment result
 */
export interface EthicalAssessment {
  /** Overall compliance */
  compliant: boolean;

  /** Principle scores */
  principleScores: PrincipleScores;

  /** Principles satisfied (score >= 7.0) */
  principlesSatisfied: EthicalPrinciple[];

  /** Concerns identified */
  concerns: EthicalConcern[];

  /** Recommendations */
  recommendations: string[];

  /** Required actions before proceeding */
  requiredActions: RequiredAction[];

  /** Approval level */
  approvalLevel: 'automatic' | 'standard' | 'enhanced' | 'prohibited';

  /** Additional assessments */
  consentValid: boolean;
  equityAssessed: boolean;
  coercionChecked: boolean;
  identityEvaluated: boolean;
  reversibilityReviewed: boolean;
  vulnerableProtected: boolean;
}

/**
 * Required action before proceeding
 */
export interface RequiredAction {
  action: string;
  priority: 'low' | 'medium' | 'high' | 'critical';
  deadline?: Date;
  responsible: string;
  status: 'pending' | 'in_progress' | 'completed';
}

// ============================================================================
// Consent Validation Types
// ============================================================================

/**
 * Consent validation request
 */
export interface ConsentValidationRequest {
  subjectId: string;
  augmentationType: AugmentationType;
  requiredLevel: ConsentLevel;
  providedConsent: ConsentDocumentation;
}

/**
 * Consent validation result
 */
export interface ConsentValidationResult {
  valid: boolean;
  level: ConsentLevel;
  gaps: string[];
  warnings: string[];
  recommendations: string[];
  compliant: boolean;
}

// ============================================================================
// Ethics Committee Types
// ============================================================================

/**
 * Ethics committee review
 */
export interface EthicsReview {
  id: string;
  date: Date;
  committee: string;

  reviewType: 'standard' | 'enhanced' | 'expedited' | 'ongoing';

  decision: 'approved' | 'conditional' | 'denied' | 'deferred';
  conditions?: string[];

  rationale: string;
  vote: {
    approve: number;
    deny: number;
    abstain: number;
  };

  followUp?: {
    required: boolean;
    frequency: string;
    duration: number;  // days
  };
}

// ============================================================================
// Report Types
// ============================================================================

/**
 * Comprehensive ethics report
 */
export interface EthicsReport {
  id: string;
  generatedAt: Date;

  augmentation: AugmentationDetails;
  subject: SubjectProfile;

  ethicalAssessment: EthicalAssessment;
  consentValidation: ConsentValidationResult;
  equityAssessment: EquityAssessment;
  coercionCheck: CoercionCheckResult;
  identityImpact: IdentityImpact;
  reversibilityProfile: ReversibilityProfile;
  vulnerableProtections?: VulnerableProtections;

  ethicsReview?: EthicsReview;

  overallRecommendation: 'proceed' | 'conditional' | 'do_not_proceed';
  summary: string;
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Ethics-related constants
 */
export const ETHICS_CONSTANTS = {
  /** Principle score thresholds */
  PRINCIPLE_THRESHOLDS: {
    SATISFIED: 7.0,
    WARNING: 5.0,
    FAILED: 0,
  },

  /** Overall compliance threshold */
  OVERALL_COMPLIANCE_THRESHOLD: 8.0,

  /** Capacity assessment thresholds */
  CAPACITY_THRESHOLDS: {
    UNDERSTANDING: 7.0,
    APPRECIATION: 7.0,
    REASONING: 7.0,
    EXPRESSION: 6.0,
  },

  /** Coercion risk thresholds */
  COERCION_THRESHOLDS: {
    NONE: 1,
    LOW: 4,
    MODERATE: 7,
    HIGH: 10,
  },

  /** Identity impact thresholds */
  IDENTITY_THRESHOLDS: {
    MINIMAL: 25,
    MODERATE: 50,
    SUBSTANTIAL: 75,
  },

  /** Equity score thresholds */
  EQUITY_THRESHOLDS: {
    EQUITABLE: 80,
    CONCERNING: 60,
  },

  /** Reversibility percentages */
  REVERSIBILITY_THRESHOLDS: {
    FULLY: 90,
    LARGELY: 70,
    PARTIALLY: 40,
    MINIMALLY: 10,
  },

  /** Cooling-off periods (days) */
  COOLING_OFF_PERIODS: {
    BASIC: 1,
    ENHANCED: 7,
    COMPREHENSIVE: 30,
    EXPERIMENTAL: 30,
  },

  /** Age thresholds */
  AGE_THRESHOLDS: {
    CHILD: 16,
    ADOLESCENT: 18,
    ADULT: 18,
  },
} as const;

// ============================================================================
// Error Types
// ============================================================================

/**
 * Ethics error codes
 */
export enum EthicsErrorCode {
  PRINCIPLE_VIOLATION = 'E001',
  CONSENT_INVALID = 'E002',
  COERCION_DETECTED = 'E003',
  CAPACITY_LACKING = 'E004',
  VULNERABLE_UNPROTECTED = 'E005',
  EQUITY_VIOLATION = 'E006',
  IDENTITY_THREAT = 'E007',
  IRREVERSIBILITY_UNJUSTIFIED = 'E008',
  PROHIBITED_AUGMENTATION = 'E009',
}

/**
 * Ethics error class
 */
export class EthicsError extends Error {
  constructor(
    public code: EthicsErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'EthicsError';
  }
}

// ============================================================================
// Export All Types
// ============================================================================

export type {
  PrincipleScores,
  EthicalConcern,
  ConsentDocumentation,
  ConsentSession,
  AugmentationClassificationInput,
  AugmentationDetails,
  AccessBarriers,
  DistributionMetric,
  EquityAssessment,
  EquityContext,
  CoercionIndicators,
  CoercionRisk,
  CoercionCheckResult,
  IdentityDimensions,
  IdentityImpact,
  AuthenticityAssessment,
  ReversibilityProfile,
  SubjectProfile,
  CapacityAssessment,
  VulnerableProtections,
  DecisionContext,
  EthicalAssessmentRequest,
  EthicalAssessment,
  RequiredAction,
  ConsentValidationRequest,
  ConsentValidationResult,
  EthicsReview,
  EthicsReport,
};

export {
  EthicalPrinciple,
  ConsentLevel,
  AugmentationType,
  ReversibilityLevel,
  ETHICS_CONSTANTS,
  EthicsErrorCode,
  EthicsError,
};
