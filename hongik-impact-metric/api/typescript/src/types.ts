/**
 * WIA-CORE-005: Hongik Impact Metric - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Core Standards Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * Seven dimensions of Hongik impact assessment
 */
export enum ImpactDimension {
  SOCIAL_GOOD = 'social_good',
  ACCESSIBILITY = 'accessibility',
  SUSTAINABILITY = 'sustainability',
  HEALTH_WELLBEING = 'health_wellbeing',
  ECONOMIC_EQUITY = 'economic_equity',
  EDUCATION = 'education',
  INNOVATION = 'innovation',
}

/**
 * Impact classification levels
 */
export enum ImpactClassification {
  EXCEPTIONAL = 'exceptional', // 900-1000
  ELITE = 'elite', // 800-899
  HIGH = 'high', // 700-799
  GOOD = 'good', // 600-699
  MODERATE = 'moderate', // 500-599
  FAIR = 'fair', // 400-499
  LIMITED = 'limited', // 300-399
  MINIMAL = 'minimal', // 200-299
  POOR = 'poor', // 100-199
  HARMFUL = 'harmful', // 0-99
}

/**
 * Certification levels
 */
export enum CertificationLevel {
  DIAMOND = 'diamond', // 900-1000
  PLATINUM = 'platinum', // 800-899
  GOLD = 'gold', // 700-799
  SILVER = 'silver', // 600-699
  BRONZE = 'bronze', // 400-599
  NONE = 'none', // <400
}

/**
 * WCAG compliance levels
 */
export enum WCAGLevel {
  NONE = 'none',
  A_PARTIAL = 'a_partial',
  A_FULL = 'a_full',
  AA_PARTIAL = 'aa_partial',
  AA_FULL = 'aa_full',
  AAA_PARTIAL = 'aaa_partial',
  AAA_FULL = 'aaa_full',
}

/**
 * Project duration categories
 */
export enum ProjectDuration {
  TEMPORARY = 'temporary', // <1 year
  SHORT_TERM = 'short_term', // 1-3 years
  MEDIUM_TERM = 'medium_term', // 3-10 years
  LONG_TERM = 'long_term', // 10-30 years
  PERMANENT = 'permanent', // >30 years
}

// ============================================================================
// Impact Assessment Types
// ============================================================================

/**
 * Complete impact assessment for a project
 */
export interface ImpactAssessment {
  /** Project identifier */
  projectId: string;

  /** Project name */
  projectName: string;

  /** Overall Hongik Impact Score (0-1000) */
  hongikScore: number;

  /** Confidence interval (95%) */
  confidenceInterval: {
    lower: number;
    upper: number;
  };

  /** Impact classification */
  classification: ImpactClassification;

  /** Certification level */
  certificationLevel: CertificationLevel;

  /** Scores for each dimension */
  dimensions: DimensionScores;

  /** Stakeholder analysis */
  stakeholders: StakeholderAnalysis;

  /** Temporal factors */
  temporal: TemporalFactors;

  /** Per-capita impact */
  perCapitaImpact: number;

  /** Assessment date */
  assessmentDate: Date;

  /** Next review date */
  nextReviewDate: Date;

  /** Assessment metadata */
  metadata: AssessmentMetadata;
}

/**
 * Scores for all seven dimensions (0-1 each)
 */
export interface DimensionScores {
  /** Social Good score (0-1) */
  socialGood: number;

  /** Accessibility score (0-1) */
  accessibility: number;

  /** Sustainability score (0-1) */
  sustainability: number;

  /** Health & Wellbeing score (0-1) */
  healthWellbeing: number;

  /** Economic Equity score (0-1) */
  economicEquity: number;

  /** Education & Growth score (0-1) */
  education: number;

  /** Innovation & Progress score (0-1) */
  innovation: number;
}

/**
 * Detailed breakdown for Social Good dimension
 */
export interface SocialGoodMetrics {
  /** Community engagement (0-1) */
  communityEngagement: number;

  /** Social equity (0-1) */
  socialEquity: number;

  /** Public benefit (0-1) */
  publicBenefit: number;

  /** Cultural impact (0-1) */
  culturalImpact: number;

  /** Overall social good score */
  overall: number;
}

/**
 * Detailed breakdown for Accessibility dimension
 */
export interface AccessibilityMetrics {
  /** Physical accessibility (0-1) */
  physical: number;

  /** Digital accessibility (0-1) */
  digital: number;

  /** Language & cultural (0-1) */
  language: number;

  /** Economic accessibility (0-1) */
  economic: number;

  /** Geographic accessibility (0-1) */
  geographic: number;

  /** WCAG compliance level */
  wcagLevel: WCAGLevel;

  /** Number of languages supported */
  languagesSupported: number;

  /** Offline support available */
  offlineSupport: boolean;

  /** Low-bandwidth optimization */
  lowBandwidth: boolean;

  /** Overall accessibility score */
  overall: number;
}

/**
 * Detailed breakdown for Sustainability dimension
 */
export interface SustainabilityMetrics {
  /** Carbon footprint score (0-1, higher = better) */
  carbonFootprint: number;

  /** Net carbon emissions (tons CO2e/year, negative is good) */
  netEmissions: number;

  /** Resource efficiency (0-1) */
  resourceEfficiency: number;

  /** Ecosystem impact (0-1) */
  ecosystemImpact: number;

  /** Long-term viability (0-1) */
  longTermViability: number;

  /** Adaptability (0-1) */
  adaptability: number;

  /** Renewable energy percentage (0-100) */
  renewableEnergyPercent: number;

  /** Overall sustainability score */
  overall: number;
}

/**
 * Detailed breakdown for Health & Wellbeing dimension
 */
export interface HealthMetrics {
  /** Disease prevention (0-1) */
  diseasePrevention: number;

  /** Mental health (0-1) */
  mentalHealth: number;

  /** Nutrition & food security (0-1) */
  nutrition: number;

  /** Safety (0-1) */
  safety: number;

  /** Quality of life (0-1) */
  qualityOfLife: number;

  /** QALYs (Quality-Adjusted Life Years) gained */
  qalysGained?: number;

  /** Mortality reduction percentage */
  mortalityReduction?: number;

  /** Overall health score */
  overall: number;
}

/**
 * Detailed breakdown for Economic Equity dimension
 */
export interface EconomicEquityMetrics {
  /** Income equality (0-1) */
  incomeEquality: number;

  /** Wealth distribution (0-1) */
  wealthDistribution: number;

  /** Economic opportunity (0-1) */
  economicOpportunity: number;

  /** Poverty reduction (0-1) */
  povertyReduction: number;

  /** Gini coefficient before */
  giniBefore?: number;

  /** Gini coefficient after */
  giniAfter?: number;

  /** Living wage provision */
  livingWage: boolean;

  /** Overall economic equity score */
  overall: number;
}

/**
 * Detailed breakdown for Education dimension
 */
export interface EducationMetrics {
  /** Educational access (0-1) */
  access: number;

  /** Learning quality (0-1) */
  quality: number;

  /** Knowledge sharing (0-1) */
  knowledgeSharing: number;

  /** Capacity building (0-1) */
  capacityBuilding: number;

  /** Number of learners reached */
  learnersReached: number;

  /** Open educational resources */
  openResources: boolean;

  /** Overall education score */
  overall: number;
}

/**
 * Detailed breakdown for Innovation dimension
 */
export interface InnovationMetrics {
  /** Technological breakthrough (0-1) */
  technologicalBreakthrough: number;

  /** Scientific contribution (0-1) */
  scientificContribution: number;

  /** Creative innovation (0-1) */
  creativeInnovation: number;

  /** Problem-solving (0-1) */
  problemSolving: number;

  /** Future potential (0-1) */
  futurePotential: number;

  /** Number of patents */
  patents?: number;

  /** Research publications */
  publications?: number;

  /** Overall innovation score */
  overall: number;
}

// ============================================================================
// Stakeholder Analysis Types
// ============================================================================

/**
 * Stakeholder analysis
 */
export interface StakeholderAnalysis {
  /** Direct beneficiaries count */
  directBeneficiaries: number;

  /** Indirect beneficiaries count */
  indirectBeneficiaries: number;

  /** Future beneficiaries count */
  futureBeneficiaries: number;

  /** Total affected beneficiaries */
  totalBeneficiaries: number;

  /** Weighted beneficiaries (accounting for vulnerability) */
  weightedBeneficiaries: number;

  /** Stakeholder groups */
  groups: StakeholderGroup[];

  /** Negative stakeholders (adversely affected) */
  negativeStakeholders?: NegativeStakeholder[];

  /** Stakeholder reach factor (1-10) */
  reachFactor: number;
}

/**
 * Stakeholder group
 */
export interface StakeholderGroup {
  /** Group identifier */
  id: string;

  /** Group name */
  name: string;

  /** Number of people in group */
  count: number;

  /** Vulnerability factor (1.0-1.6) */
  vulnerabilityFactor: number;

  /** Age group */
  ageGroup?: AgeGroup;

  /** Special characteristics */
  characteristics: string[];

  /** Impact on this group (0-1) */
  impactScore: number;
}

/**
 * Age group categories
 */
export enum AgeGroup {
  CHILDREN = 'children', // 0-12
  YOUTH = 'youth', // 13-24
  ADULTS = 'adults', // 25-64
  ELDERLY = 'elderly', // 65+
}

/**
 * Negative stakeholder (adversely affected)
 */
export interface NegativeStakeholder {
  /** Stakeholder identifier */
  id: string;

  /** Stakeholder name */
  name: string;

  /** Number of people affected */
  count: number;

  /** Negative impact score (0-1) */
  negativeImpact: number;

  /** Mitigation measures in place */
  mitigationMeasures: string[];

  /** Residual negative impact after mitigation */
  residualImpact: number;
}

// ============================================================================
// Temporal Analysis Types
// ============================================================================

/**
 * Temporal factors
 */
export interface TemporalFactors {
  /** Project duration category */
  duration: ProjectDuration;

  /** Expected lifespan in years */
  lifespanYears: number;

  /** Temporal factor multiplier (0.5-1.5) */
  temporalFactor: number;

  /** Impact trajectory over time */
  trajectory?: ImpactTrajectory;

  /** Year-over-year improvement */
  yoyImprovement?: number;

  /** Improvement bonus (0-0.1) */
  improvementBonus?: number;
}

/**
 * Impact trajectory over time
 */
export interface ImpactTrajectory {
  /** Baseline HIS at project start */
  baseline: number;

  /** Annual growth rate */
  growthRate: number;

  /** Historical data points */
  history: ImpactDataPoint[];

  /** Projected future points */
  projections: ImpactDataPoint[];

  /** Decay constant (for non-permanent projects) */
  decayConstant?: number;
}

/**
 * Impact data point at specific time
 */
export interface ImpactDataPoint {
  /** Date of measurement */
  date: Date;

  /** HIS at this time */
  score: number;

  /** Number of beneficiaries at this time */
  beneficiaries: number;

  /** Notes or context */
  notes?: string;
}

// ============================================================================
// Assessment Metadata Types
// ============================================================================

/**
 * Assessment metadata
 */
export interface AssessmentMetadata {
  /** Assessment method */
  method: AssessmentMethod;

  /** Data sources used */
  dataSources: DataSource[];

  /** Verification level */
  verificationLevel: VerificationLevel;

  /** Assessor organization */
  assessor: string;

  /** Peer reviewers */
  reviewers?: string[];

  /** Audit firm (if applicable) */
  auditor?: string;

  /** Confidence score (0-1) */
  confidence: number;

  /** Data quality score (0-1) */
  dataQuality: number;

  /** Sample size (if survey-based) */
  sampleSize?: number;

  /** Notes and caveats */
  notes?: string;
}

/**
 * Assessment method
 */
export enum AssessmentMethod {
  RCT = 'rct', // Randomized Controlled Trial
  QUASI_EXPERIMENTAL = 'quasi_experimental',
  LONGITUDINAL = 'longitudinal',
  CROSS_SECTIONAL = 'cross_sectional',
  ADMINISTRATIVE_DATA = 'administrative_data',
  SELF_REPORTED = 'self_reported',
}

/**
 * Data source
 */
export interface DataSource {
  /** Source name */
  name: string;

  /** Source type */
  type: 'primary' | 'secondary' | 'tertiary';

  /** URL or reference */
  reference?: string;

  /** Reliability score (0-1) */
  reliability: number;

  /** Date of data collection */
  date: Date;
}

/**
 * Verification level
 */
export enum VerificationLevel {
  SELF_ASSESSMENT = 'self_assessment',
  PEER_REVIEW = 'peer_review',
  STAKEHOLDER_VALIDATION = 'stakeholder_validation',
  INDEPENDENT_AUDIT = 'independent_audit',
  CERTIFIED = 'certified',
}

// ============================================================================
// Quick Assessment Types
// ============================================================================

/**
 * Simplified input for quick assessment
 */
export interface QuickAssessmentInput {
  /** Social good score (0-1) */
  socialGood: number;

  /** Accessibility score (0-1) */
  accessibility: number;

  /** Sustainability score (0-1) */
  sustainability: number;

  /** Health & wellbeing score (0-1) */
  healthWellbeing: number;

  /** Economic equity score (0-1) */
  economicEquity: number;

  /** Education score (0-1) */
  education: number;

  /** Innovation score (0-1) */
  innovation: number;

  /** Total beneficiaries */
  beneficiaries: number;

  /** Project duration (optional) */
  duration?: ProjectDuration;
}

/**
 * Quick assessment result
 */
export interface QuickAssessmentResult {
  /** Total Hongik Impact Score (0-1000) */
  total: number;

  /** Per-capita impact */
  perCapita: number;

  /** Classification */
  classification: ImpactClassification;

  /** Weighted dimension scores */
  weightedScores: {
    socialGood: number;
    accessibility: number;
    sustainability: number;
    healthWellbeing: number;
    economicEquity: number;
    education: number;
    innovation: number;
  };

  /** Stakeholder reach factor */
  reachFactor: number;

  /** Temporal factor */
  temporalFactor: number;
}

// ============================================================================
// Comparison Types
// ============================================================================

/**
 * Project comparison result
 */
export interface ProjectComparison {
  /** Projects being compared */
  projects: ImpactAssessment[];

  /** Ranked by overall HIS */
  ranking: ProjectRank[];

  /** Dimension-by-dimension comparison */
  dimensionComparison: DimensionComparison[];

  /** Insights and recommendations */
  insights: string[];
}

/**
 * Project ranking
 */
export interface ProjectRank {
  /** Rank (1 = highest) */
  rank: number;

  /** Project ID */
  projectId: string;

  /** Project name */
  projectName: string;

  /** HIS */
  score: number;

  /** Classification */
  classification: ImpactClassification;
}

/**
 * Dimension comparison across projects
 */
export interface DimensionComparison {
  /** Dimension name */
  dimension: ImpactDimension;

  /** Scores for each project */
  scores: {
    projectId: string;
    score: number;
  }[];

  /** Highest scoring project */
  leader: string;

  /** Average score across projects */
  average: number;
}

// ============================================================================
// Report Types
// ============================================================================

/**
 * Impact report
 */
export interface ImpactReport {
  /** Report ID */
  reportId: string;

  /** Report generation date */
  generatedAt: Date;

  /** Report period */
  period: {
    start: Date;
    end: Date;
  };

  /** Impact assessment */
  assessment: ImpactAssessment;

  /** Detailed metrics by dimension */
  detailedMetrics: {
    socialGood: SocialGoodMetrics;
    accessibility: AccessibilityMetrics;
    sustainability: SustainabilityMetrics;
    health: HealthMetrics;
    economicEquity: EconomicEquityMetrics;
    education: EducationMetrics;
    innovation: InnovationMetrics;
  };

  /** Case studies and testimonials */
  caseStudies?: CaseStudy[];

  /** Recommendations for improvement */
  recommendations: Recommendation[];

  /** Certification status */
  certificationStatus: CertificationStatus;
}

/**
 * Case study
 */
export interface CaseStudy {
  /** Case study ID */
  id: string;

  /** Title */
  title: string;

  /** Description */
  description: string;

  /** Beneficiary testimonial */
  testimonial?: string;

  /** Impact demonstrated */
  impactDemonstrated: string[];

  /** Supporting evidence */
  evidence: string[];
}

/**
 * Recommendation for improvement
 */
export interface Recommendation {
  /** Recommendation ID */
  id: string;

  /** Dimension to improve */
  dimension: ImpactDimension;

  /** Current score */
  currentScore: number;

  /** Potential score with improvement */
  potentialScore: number;

  /** Recommended actions */
  actions: string[];

  /** Estimated effort */
  effort: 'low' | 'medium' | 'high';

  /** Estimated impact */
  impactIncrease: number;

  /** Priority */
  priority: 'low' | 'medium' | 'high';
}

/**
 * Certification status
 */
export interface CertificationStatus {
  /** Certified? */
  certified: boolean;

  /** Certification level */
  level: CertificationLevel;

  /** Certification date */
  certifiedAt?: Date;

  /** Expiry date */
  expiresAt?: Date;

  /** Certification body */
  certifiedBy?: string;

  /** Certification ID */
  certificationId?: string;

  /** Requirements for next level */
  nextLevelRequirements?: string[];
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Dimension weights (must sum to 1.0)
 */
export const DIMENSION_WEIGHTS = {
  [ImpactDimension.SOCIAL_GOOD]: 0.20,
  [ImpactDimension.ACCESSIBILITY]: 0.20,
  [ImpactDimension.SUSTAINABILITY]: 0.15,
  [ImpactDimension.HEALTH_WELLBEING]: 0.15,
  [ImpactDimension.ECONOMIC_EQUITY]: 0.10,
  [ImpactDimension.EDUCATION]: 0.10,
  [ImpactDimension.INNOVATION]: 0.10,
} as const;

/**
 * WCAG level scores
 */
export const WCAG_SCORES = {
  [WCAGLevel.NONE]: 0.0,
  [WCAGLevel.A_PARTIAL]: 0.3,
  [WCAGLevel.A_FULL]: 0.5,
  [WCAGLevel.AA_PARTIAL]: 0.7,
  [WCAGLevel.AA_FULL]: 0.85,
  [WCAGLevel.AAA_PARTIAL]: 0.95,
  [WCAGLevel.AAA_FULL]: 1.0,
} as const;

/**
 * Vulnerability factors by group
 */
export const VULNERABILITY_FACTORS = {
  [AgeGroup.CHILDREN]: 1.5,
  [AgeGroup.YOUTH]: 1.2,
  [AgeGroup.ADULTS]: 1.0,
  [AgeGroup.ELDERLY]: 1.3,
  DISABLED: 1.4,
  EXTREME_POVERTY: 1.6,
  REFUGEES: 1.5,
  INDIGENOUS: 1.3,
} as const;

/**
 * Temporal factor multipliers
 */
export const TEMPORAL_FACTORS = {
  [ProjectDuration.TEMPORARY]: 0.55,
  [ProjectDuration.SHORT_TERM]: 0.75,
  [ProjectDuration.MEDIUM_TERM]: 0.95,
  [ProjectDuration.LONG_TERM]: 1.2,
  [ProjectDuration.PERMANENT]: 1.45,
} as const;

/**
 * Global population (for reach factor calculation)
 */
export const GLOBAL_POPULATION = 8_000_000_000;

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-CORE-005 error codes
 */
export enum HongikErrorCode {
  INVALID_SCORE = 'H001',
  INVALID_DIMENSION = 'H002',
  MISSING_REQUIRED_DATA = 'H003',
  INSUFFICIENT_BENEFICIARIES = 'H004',
  INVALID_WEIGHTS = 'H005',
  VERIFICATION_FAILED = 'H006',
  CERTIFICATION_DENIED = 'H007',
  DATA_QUALITY_LOW = 'H008',
}

/**
 * Hongik Impact error
 */
export class HongikError extends Error {
  constructor(
    public code: HongikErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'HongikError';
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
// Export All
// ============================================================================

export type {
  ImpactAssessment,
  DimensionScores,
  SocialGoodMetrics,
  AccessibilityMetrics,
  SustainabilityMetrics,
  HealthMetrics,
  EconomicEquityMetrics,
  EducationMetrics,
  InnovationMetrics,
  StakeholderAnalysis,
  StakeholderGroup,
  NegativeStakeholder,
  TemporalFactors,
  ImpactTrajectory,
  ImpactDataPoint,
  AssessmentMetadata,
  DataSource,
  QuickAssessmentInput,
  QuickAssessmentResult,
  ProjectComparison,
  ProjectRank,
  DimensionComparison,
  ImpactReport,
  CaseStudy,
  Recommendation,
  CertificationStatus,
};

export {
  ImpactDimension,
  ImpactClassification,
  CertificationLevel,
  WCAGLevel,
  ProjectDuration,
  AgeGroup,
  AssessmentMethod,
  VerificationLevel,
  HongikErrorCode,
  HongikError,
  DIMENSION_WEIGHTS,
  WCAG_SCORES,
  VULNERABILITY_FACTORS,
  TEMPORAL_FACTORS,
  GLOBAL_POPULATION,
};
