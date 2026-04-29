/**
 * WIA-TIME-029: Time Adaptation - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Time Adaptation Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Adaptation Types
// ============================================================================

/**
 * Time period identifier
 */
export interface TimePeriod {
  /** ISO 8601 date or year */
  date: Date | string;

  /** Era name (e.g., "Victorian Era", "Roaring Twenties") */
  eraName?: string;

  /** Geographic location */
  location?: GeographicLocation;

  /** Historical context tags */
  tags?: string[];
}

/**
 * Geographic location
 */
export interface GeographicLocation {
  /** Country name */
  country: string;

  /** Region/state/province */
  region?: string;

  /** City */
  city?: string;

  /** Geographic coordinates */
  coordinates?: {
    latitude: number;
    longitude: number;
  };
}

/**
 * Traveler profile
 */
export interface TravelerProfile {
  /** Unique traveler identifier */
  id?: string;

  /** Age in years */
  age: number;

  /** Education level */
  educationLevel: 'basic' | 'intermediate' | 'advanced' | 'expert';

  /** Native language(s) */
  nativeLanguages: string[];

  /** Additional language skills */
  additionalLanguages?: LanguageSkill[];

  /** Professional specializations */
  specializations?: string[];

  /** Previous temporal travel experience */
  previousTravels?: PreviousTravel[];

  /** Psychological profile */
  psychologicalProfile?: PsychologicalProfile;

  /** Medical conditions */
  medicalConditions?: string[];

  /** Cultural background */
  culturalBackground?: string[];
}

/**
 * Language skill level
 */
export interface LanguageSkill {
  /** Language name */
  language: string;

  /** Proficiency level */
  proficiency: 'beginner' | 'intermediate' | 'advanced' | 'fluent' | 'native';

  /** Can read? */
  reading?: boolean;

  /** Can write? */
  writing?: boolean;

  /** Can speak? */
  speaking?: boolean;

  /** Can understand speech? */
  listening?: boolean;
}

/**
 * Previous temporal travel
 */
export interface PreviousTravel {
  /** Destination era */
  destinationEra: TimePeriod;

  /** Duration in days */
  duration: number;

  /** Adaptation success score (0-100) */
  adaptationScore?: number;

  /** Key challenges faced */
  challenges?: string[];

  /** Skills acquired */
  skillsAcquired?: string[];
}

/**
 * Psychological profile
 */
export interface PsychologicalProfile {
  /** Adaptability score (0-100) */
  adaptability: number;

  /** Stress tolerance (0-100) */
  stressTolerance: number;

  /** Cultural openness (0-100) */
  culturalOpenness: number;

  /** Social comfort (0-100) */
  socialComfort: number;

  /** Learning agility (0-100) */
  learningAgility: number;

  /** Risk tolerance */
  riskTolerance: 'low' | 'medium' | 'high';

  /** Personality type (e.g., MBTI) */
  personalityType?: string;
}

// ============================================================================
// Adaptation Program
// ============================================================================

/**
 * Adaptation program configuration
 */
export interface AdaptationProgram {
  /** Program identifier */
  id: string;

  /** Traveler profile */
  traveler: TravelerProfile;

  /** Origin time period */
  originEra: TimePeriod;

  /** Target time period */
  targetEra: TimePeriod;

  /** Planned duration in target era (days) */
  plannedDuration: number;

  /** Training modules */
  trainingModules: TrainingModule[];

  /** Pre-departure phase */
  preDeparture: PreDeparturePhase;

  /** In-era support */
  inEraSupport: InEraSupport;

  /** Re-adaptation plan */
  reAdaptation: ReAdaptationPlan;

  /** Program status */
  status: 'planning' | 'training' | 'active' | 'completed' | 'cancelled';

  /** Created timestamp */
  created: Date;

  /** Last updated */
  updated: Date;
}

/**
 * Training module
 */
export interface TrainingModule {
  /** Module identifier */
  id: string;

  /** Module name */
  name: string;

  /** Category */
  category: AdaptationCategory;

  /** Description */
  description: string;

  /** Duration in hours */
  duration: number;

  /** Required completion percentage */
  requiredCompletion: number;

  /** Current progress (0-100) */
  progress?: number;

  /** Learning materials */
  materials: LearningMaterial[];

  /** Assessment tests */
  assessments?: Assessment[];

  /** Prerequisites */
  prerequisites?: string[];

  /** Status */
  status: 'not-started' | 'in-progress' | 'completed' | 'failed';
}

/**
 * Adaptation category
 */
export type AdaptationCategory =
  | 'language'
  | 'technology'
  | 'social-norms'
  | 'daily-living'
  | 'legal-political'
  | 'health-medical'
  | 'economics'
  | 'religion-philosophy'
  | 'history'
  | 'geography'
  | 'arts-culture'
  | 'science';

/**
 * Learning material
 */
export interface LearningMaterial {
  /** Material identifier */
  id: string;

  /** Material type */
  type: 'video' | 'text' | 'audio' | 'interactive' | 'vr-simulation' | 'ar-overlay';

  /** Title */
  title: string;

  /** Description */
  description?: string;

  /** Content URL or data */
  content: string;

  /** Duration in minutes */
  duration?: number;

  /** Difficulty level */
  difficulty?: 'beginner' | 'intermediate' | 'advanced';

  /** Completed? */
  completed?: boolean;
}

/**
 * Assessment test
 */
export interface Assessment {
  /** Assessment identifier */
  id: string;

  /** Assessment name */
  name: string;

  /** Assessment type */
  type: 'quiz' | 'practical' | 'simulation' | 'oral' | 'written';

  /** Questions or tasks */
  items: AssessmentItem[];

  /** Passing score (0-100) */
  passingScore: number;

  /** Attempts allowed */
  attemptsAllowed: number;

  /** Current score */
  score?: number;

  /** Attempts used */
  attemptsUsed?: number;

  /** Status */
  status: 'pending' | 'passed' | 'failed' | 'in-progress';
}

/**
 * Assessment item
 */
export interface AssessmentItem {
  /** Item identifier */
  id: string;

  /** Question or task description */
  prompt: string;

  /** Item type */
  type: 'multiple-choice' | 'true-false' | 'short-answer' | 'essay' | 'practical';

  /** Options (for multiple choice) */
  options?: string[];

  /** Correct answer(s) */
  correctAnswer?: string | string[];

  /** Points value */
  points: number;

  /** User answer */
  userAnswer?: string | string[];

  /** Correct? */
  correct?: boolean;
}

/**
 * Pre-departure training phase
 */
export interface PreDeparturePhase {
  /** Training start date */
  startDate: Date;

  /** Training end date */
  endDate: Date;

  /** Total training hours */
  totalHours: number;

  /** Completed hours */
  completedHours: number;

  /** Required modules */
  requiredModules: string[];

  /** Optional modules */
  optionalModules?: string[];

  /** Readiness assessment */
  readinessAssessment?: ReadinessAssessment;

  /** Status */
  status: 'scheduled' | 'in-progress' | 'completed' | 'insufficient';
}

/**
 * Readiness assessment
 */
export interface ReadinessAssessment {
  /** Assessment date */
  date: Date;

  /** Overall readiness score (0-100) */
  overallScore: number;

  /** Category scores */
  categoryScores: Record<AdaptationCategory, number>;

  /** Recommendation */
  recommendation: 'ready' | 'conditional' | 'not-ready' | 'extended-training';

  /** Identified gaps */
  gaps?: string[];

  /** Strengths */
  strengths?: string[];

  /** Additional training needed (hours) */
  additionalTraining?: number;
}

/**
 * In-era support system
 */
export interface InEraSupport {
  /** Support coordinator contact */
  coordinator?: ContactInfo;

  /** Local mentor */
  mentor?: MentorInfo;

  /** Emergency contacts */
  emergencyContacts: ContactInfo[];

  /** Check-in schedule */
  checkInSchedule: CheckInSchedule;

  /** Real-time translation support */
  translationSupport: boolean;

  /** Cultural advisor availability */
  culturalAdvisor: boolean;

  /** Medical support */
  medicalSupport: MedicalSupport;

  /** Safe haven locations */
  safeHavens?: SafeHavenLocation[];
}

/**
 * Contact information
 */
export interface ContactInfo {
  /** Contact name */
  name: string;

  /** Role */
  role: string;

  /** Communication method */
  method: string;

  /** Contact details */
  details: string;

  /** Availability */
  availability?: string;
}

/**
 * Mentor information
 */
export interface MentorInfo extends ContactInfo {
  /** Expertise areas */
  expertise: string[];

  /** Languages spoken */
  languages: string[];

  /** Background */
  background?: string;
}

/**
 * Check-in schedule
 */
export interface CheckInSchedule {
  /** Frequency */
  frequency: 'daily' | 'weekly' | 'biweekly' | 'monthly' | 'as-needed';

  /** Method */
  method: 'temporal-comm' | 'in-person' | 'written' | 'mixed';

  /** Next check-in */
  nextCheckIn?: Date;

  /** Missed check-ins */
  missedCheckIns?: number;
}

/**
 * Medical support
 */
export interface MedicalSupport {
  /** Medical facility locations */
  facilities: GeographicLocation[];

  /** Emergency protocol */
  emergencyProtocol: string;

  /** Medication compatibility */
  medicationInfo?: string;

  /** Disease risk briefing */
  diseaseRisks?: DiseaseRisk[];
}

/**
 * Disease risk
 */
export interface DiseaseRisk {
  /** Disease name */
  disease: string;

  /** Risk level */
  riskLevel: 'low' | 'medium' | 'high' | 'critical';

  /** Preventive measures */
  prevention: string[];

  /** Symptoms */
  symptoms?: string[];

  /** Treatment availability */
  treatmentAvailable: boolean;
}

/**
 * Safe haven location
 */
export interface SafeHavenLocation {
  /** Location name */
  name: string;

  /** Address or description */
  address: string;

  /** Coordinates */
  coordinates?: {
    latitude: number;
    longitude: number;
  };

  /** Access code or password */
  accessCode?: string;

  /** Available resources */
  resources: string[];

  /** Contact person */
  contact?: ContactInfo;
}

/**
 * Re-adaptation plan
 */
export interface ReAdaptationPlan {
  /** Re-entry date */
  reEntryDate?: Date;

  /** Decompression period (days) */
  decompressionPeriod: number;

  /** Debriefing sessions */
  debriefingSessions: number;

  /** Counseling availability */
  counselingAvailable: boolean;

  /** Skill integration */
  skillIntegration?: SkillIntegration[];

  /** Reverse culture shock mitigation */
  reverseCultureShockSupport: boolean;
}

/**
 * Skill integration
 */
export interface SkillIntegration {
  /** Skill acquired */
  skill: string;

  /** Applicable in origin era? */
  applicable: boolean;

  /** Integration strategy */
  strategy?: string;

  /** Value assessment */
  value?: 'high' | 'medium' | 'low';
}

// ============================================================================
// Culture Shock Assessment
// ============================================================================

/**
 * Culture shock assessment parameters
 */
export interface CultureShockAssessment {
  /** Temporal displacement in seconds */
  temporalDisplacement: number;

  /** Cultural distance index (0-1) */
  culturalDistance: number;

  /** Technological gap (0-1) */
  technologicalGap: number;

  /** Language barrier (0-1) */
  languageBarrier?: number;

  /** Social structure difference (0-1) */
  socialDifference?: number;

  /** Traveler adaptability (0-100) */
  travelerAdaptability?: number;
}

/**
 * Culture shock risk result
 */
export interface CultureShockRisk {
  /** Overall risk level */
  riskLevel: 'minimal' | 'low' | 'moderate' | 'high' | 'severe' | 'extreme';

  /** Risk score (0-100) */
  riskScore: number;

  /** Adaptation difficulty (1-10) */
  adaptationDifficulty: number;

  /** Recommended training duration (weeks) */
  trainingDuration: number;

  /** Critical challenges */
  criticalChallenges: string[];

  /** Mitigation strategies */
  mitigationStrategies: string[];

  /** Success probability (0-1) */
  successProbability: number;
}

// ============================================================================
// Adaptation Progress Tracking
// ============================================================================

/**
 * Adaptation progress metrics
 */
export interface AdaptationMetrics {
  /** Measurement date */
  date: Date;

  /** Days in target era */
  daysInEra: number;

  /** Cultural competency score (0-100) */
  culturalCompetency: number;

  /** Language proficiency (0-100) */
  languageProficiency: number;

  /** Social integration (0-100) */
  socialIntegration: number;

  /** Psychological wellbeing (0-100) */
  psychologicalWellbeing: number;

  /** Daily functioning (0-100) */
  dailyFunctioning: number;

  /** Technology adaptation (0-100) */
  technologyAdaptation: number;

  /** Overall adaptation score (0-100) */
  overallScore: number;

  /** Current phase */
  currentPhase: AdaptationPhase;

  /** Incidents or challenges */
  incidents?: AdaptationIncident[];

  /** Achievements */
  achievements?: string[];
}

/**
 * Adaptation phase
 */
export type AdaptationPhase =
  | 'honeymoon'
  | 'culture-shock'
  | 'adjustment'
  | 'adaptation'
  | 'mastery'
  | 're-entry';

/**
 * Adaptation incident
 */
export interface AdaptationIncident {
  /** Incident date */
  date: Date;

  /** Incident type */
  type: 'misunderstanding' | 'social-faux-pas' | 'legal-issue' | 'health-issue' |
        'technology-failure' | 'language-barrier' | 'safety-concern' | 'other';

  /** Severity */
  severity: 'minor' | 'moderate' | 'serious' | 'critical';

  /** Description */
  description: string;

  /** Resolution */
  resolution?: string;

  /** Lessons learned */
  lessonsLearned?: string[];

  /** Resolved? */
  resolved: boolean;
}

/**
 * Progress report
 */
export interface ProgressReport {
  /** Report identifier */
  id: string;

  /** Traveler identifier */
  travelerId: string;

  /** Report date */
  date: Date;

  /** Reporting period */
  period: {
    start: Date;
    end: Date;
  };

  /** Current metrics */
  metrics: AdaptationMetrics;

  /** Trend analysis */
  trends: TrendAnalysis;

  /** Recommendations */
  recommendations: string[];

  /** Concerns */
  concerns?: string[];

  /** Next steps */
  nextSteps: string[];
}

/**
 * Trend analysis
 */
export interface TrendAnalysis {
  /** Overall trajectory */
  trajectory: 'improving' | 'stable' | 'declining' | 'fluctuating';

  /** Improvement rate (points per week) */
  improvementRate: number;

  /** Projected mastery date */
  projectedMastery?: Date;

  /** Areas of strength */
  strengths: string[];

  /** Areas needing attention */
  weaknesses: string[];

  /** Comparison to expected progress */
  vsExpected: 'ahead' | 'on-track' | 'behind' | 'significantly-behind';
}

// ============================================================================
// Language Adaptation
// ============================================================================

/**
 * Language adaptation plan
 */
export interface LanguageAdaptationPlan {
  /** Target language */
  targetLanguage: string;

  /** Historical variant */
  historicalVariant: string;

  /** Native language */
  nativeLanguage: string;

  /** Learning path */
  learningPath: LanguageLearningModule[];

  /** Dialect focus */
  dialectFocus?: string[];

  /** Slang and idioms */
  slangTraining: boolean;

  /** Written form training */
  writtenFormTraining: boolean;

  /** Pronunciation coaching */
  pronunciationCoaching: boolean;

  /** Estimated time to proficiency (hours) */
  estimatedHours: number;
}

/**
 * Language learning module
 */
export interface LanguageLearningModule {
  /** Module identifier */
  id: string;

  /** Module name */
  name: string;

  /** Focus area */
  focus: 'vocabulary' | 'grammar' | 'pronunciation' | 'conversation' |
         'reading' | 'writing' | 'listening' | 'cultural-context';

  /** Difficulty level */
  level: 'beginner' | 'intermediate' | 'advanced';

  /** Duration in hours */
  duration: number;

  /** Practice exercises */
  exercises: number;

  /** Progress (0-100) */
  progress: number;

  /** Mastery score (0-100) */
  masteryScore?: number;
}

// ============================================================================
// Historical Context
// ============================================================================

/**
 * Historical context briefing
 */
export interface HistoricalContext {
  /** Time period */
  period: TimePeriod;

  /** Major events */
  majorEvents: HistoricalEvent[];

  /** Social structure */
  socialStructure: SocialStructure;

  /** Technology level */
  technologyLevel: TechnologyLevel;

  /** Cultural norms */
  culturalNorms: CulturalNorm[];

  /** Political situation */
  politicalSituation: PoliticalContext;

  /** Economic conditions */
  economicConditions: EconomicContext;

  /** Daily life overview */
  dailyLife: DailyLifeContext;
}

/**
 * Historical event
 */
export interface HistoricalEvent {
  /** Event name */
  name: string;

  /** Date */
  date: Date | string;

  /** Description */
  description: string;

  /** Impact level */
  impact: 'minor' | 'moderate' | 'major' | 'transformative';

  /** Relevance to traveler */
  relevance?: string;

  /** Related events */
  relatedEvents?: string[];
}

/**
 * Social structure
 */
export interface SocialStructure {
  /** Class system description */
  classSystem: string;

  /** Social mobility */
  mobility: 'rigid' | 'limited' | 'moderate' | 'fluid';

  /** Gender roles */
  genderRoles: string;

  /** Family structure */
  familyStructure: string;

  /** Social etiquette */
  etiquette: string[];

  /** Taboos */
  taboos: string[];
}

/**
 * Technology level
 */
export interface TechnologyLevel {
  /** Era description */
  era: string;

  /** Communication methods */
  communication: string[];

  /** Transportation */
  transportation: string[];

  /** Energy sources */
  energy: string[];

  /** Medical technology */
  medical: string[];

  /** Common devices */
  devices: string[];

  /** Technology gap from origin (0-1) */
  gapFromOrigin: number;
}

/**
 * Cultural norm
 */
export interface CulturalNorm {
  /** Norm category */
  category: string;

  /** Description */
  description: string;

  /** Importance */
  importance: 'low' | 'medium' | 'high' | 'critical';

  /** Violation consequences */
  consequences?: string;

  /** Compliance tips */
  tips?: string[];
}

/**
 * Political context
 */
export interface PoliticalContext {
  /** Government type */
  governmentType: string;

  /** Current leaders */
  leaders?: string[];

  /** Key policies */
  keyPolicies: string[];

  /** Citizen rights */
  citizenRights: string[];

  /** Restrictions */
  restrictions: string[];

  /** Political stability */
  stability: 'unstable' | 'volatile' | 'moderate' | 'stable';
}

/**
 * Economic context
 */
export interface EconomicContext {
  /** Economic system */
  system: string;

  /** Currency */
  currency: string;

  /** Typical wages */
  typicalWages?: string;

  /** Cost of living */
  costOfLiving?: string;

  /** Common occupations */
  occupations: string[];

  /** Trade practices */
  tradePractices: string[];
}

/**
 * Daily life context
 */
export interface DailyLifeContext {
  /** Typical daily schedule */
  schedule: string;

  /** Food and dining */
  foodCulture: string;

  /** Clothing norms */
  clothingNorms: string;

  /** Housing */
  housing: string;

  /** Hygiene practices */
  hygiene: string;

  /** Entertainment */
  entertainment: string[];

  /** Common challenges */
  challenges: string[];
}

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Adaptation success threshold
 */
export const ADAPTATION_THRESHOLDS = {
  minimal: 40,      // Survival level
  functional: 60,   // Basic competence
  proficient: 75,   // Full integration
  expert: 90,       // Native-level mastery
} as const;

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
 * WIA-TIME-029 error codes
 */
export enum AdaptationErrorCode {
  INVALID_PROFILE = 'TA001',
  INSUFFICIENT_TRAINING = 'TA002',
  HIGH_CULTURE_SHOCK_RISK = 'TA003',
  LANGUAGE_BARRIER = 'TA004',
  MEDICAL_CONTRAINDICATION = 'TA005',
  ASSESSMENT_FAILED = 'TA006',
  ADAPTATION_FAILURE = 'TA007',
  PSYCHOLOGICAL_DISTRESS = 'TA008',
  SAFETY_VIOLATION = 'TA009',
  SUPPORT_UNAVAILABLE = 'TA010',
}

/**
 * Time adaptation error
 */
export class TimeAdaptationError extends Error {
  constructor(
    public code: AdaptationErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'TimeAdaptationError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  // Core types
  TimePeriod,
  GeographicLocation,
  TravelerProfile,
  LanguageSkill,
  PreviousTravel,
  PsychologicalProfile,

  // Adaptation program
  AdaptationProgram,
  TrainingModule,
  LearningMaterial,
  Assessment,
  AssessmentItem,
  PreDeparturePhase,
  ReadinessAssessment,
  InEraSupport,
  ContactInfo,
  MentorInfo,
  CheckInSchedule,
  MedicalSupport,
  DiseaseRisk,
  SafeHavenLocation,
  ReAdaptationPlan,
  SkillIntegration,

  // Culture shock
  CultureShockAssessment,
  CultureShockRisk,

  // Progress tracking
  AdaptationMetrics,
  AdaptationIncident,
  ProgressReport,
  TrendAnalysis,

  // Language
  LanguageAdaptationPlan,
  LanguageLearningModule,

  // Historical context
  HistoricalContext,
  HistoricalEvent,
  SocialStructure,
  TechnologyLevel,
  CulturalNorm,
  PoliticalContext,
  EconomicContext,
  DailyLifeContext,
};

export {
  ADAPTATION_THRESHOLDS,
  AdaptationErrorCode,
  TimeAdaptationError,
};
