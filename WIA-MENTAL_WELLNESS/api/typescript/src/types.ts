/**
 * WIA-MENTAL_WELLNESS TypeScript Type Definitions
 * Version: 1.0.0
 * Philosophy: 弘益人間 (Benefit All Humanity)
 *
 * Comprehensive type definitions for mental wellness systems including
 * mood tracking, therapy sessions, assessments, crisis intervention,
 * and mindfulness practices.
 */

// ============================================================================
// Core Types
// ============================================================================

export type UUID = string;
export type ISO8601 = string;
export type EncryptedString = string;

export interface WIAMentalWellnessRecord {
  type: "WIA-MENTAL_WELLNESSRecord";
  version: string;
  id: UUID;
  timestamp: ISO8601;
  userId: string; // Anonymized
  data: {
    recordType: "mood" | "assessment" | "session" | "intervention" | "mindfulness";
    content: Record<string, any>;
    metadata: Record<string, any>;
  };
  privacy: PrivacySettings;
  signature?: string;
}

export interface PrivacySettings {
  encryptionLevel: "high" | "medium" | "standard";
  consentId: UUID;
  dataRetention: string; // ISO 8601 duration
  deIdentified?: boolean;
  hipaaCompliant?: boolean;
}

// ============================================================================
// Mood Tracking Types
// ============================================================================

export type MoodPrimary =
  | "happy"
  | "sad"
  | "anxious"
  | "calm"
  | "angry"
  | "neutral"
  | "mixed"
  | "depressed"
  | "excited"
  | "fearful"
  | "frustrated"
  | "content";

export interface MoodRecord {
  type: "MoodRecord";
  moodId: UUID;
  timestamp: ISO8601;
  userId: string;
  mood: MoodData;
  context?: ContextData;
  physiological?: PhysiologicalData;
}

export interface MoodData {
  primary: MoodPrimary;
  intensity: number; // 1-10
  valence: number; // -10 to 10 (negative to positive)
  arousal: number; // 0-10 (calm to excited)
  dominance: number; // 0-10 (submissive to dominant)
  tags?: string[];
  triggers?: string[];
  coping?: string[];
}

export interface ContextData {
  location?: "home" | "work" | "transit" | "social" | "other";
  activity?: string;
  social?: "alone" | "family" | "friends" | "colleagues" | "strangers";
  weather?: string;
  sleep?: {
    hours: number; // 0-24
    quality: number; // 1-5
  };
}

export interface PhysiologicalData {
  heartRate?: number; // bpm
  hrv?: number; // ms
  respirationRate?: number; // breaths per minute
  skinConductance?: number; // microsiemens
  cortisol?: number; // nmol/L
  bloodPressure?: {
    systolic: number;
    diastolic: number;
  };
}

export interface MoodAnalytics {
  period: "day" | "week" | "month" | "year";
  moodDistribution: Record<MoodPrimary, number>;
  averageIntensity: number;
  averageValence: number;
  trends: {
    improving: boolean;
    stable: boolean;
    declining: boolean;
  };
  patterns: MoodPattern[];
  recommendations: string[];
}

export interface MoodPattern {
  pattern: string;
  confidence: number; // 0-1
  occurrences: number;
  timeOfDay?: string;
  dayOfWeek?: string;
  associatedTriggers?: string[];
}

// ============================================================================
// Assessment Types
// ============================================================================

export type AssessmentType =
  | "PHQ-9"      // Depression (Patient Health Questionnaire)
  | "GAD-7"      // Anxiety (Generalized Anxiety Disorder)
  | "PSS"        // Perceived Stress Scale
  | "BDI"        // Beck Depression Inventory
  | "DASS-21"    // Depression Anxiety Stress Scales
  | "WHO-5"      // Well-being Index
  | "PCL-5"      // PTSD Checklist
  | "MDQ"        // Mood Disorder Questionnaire
  | "AUDIT"      // Alcohol Use Disorders Identification Test
  | "ISI"        // Insomnia Severity Index
  | "OCI-R"      // Obsessive-Compulsive Inventory
  | "YBOCS"      // Yale-Brown Obsessive Compulsive Scale
  | "SPIN";      // Social Phobia Inventory

export type AssessmentInterpretation =
  | "minimal"
  | "mild"
  | "moderate"
  | "moderately-severe"
  | "severe"
  | "extreme";

export type RiskLevel = "low" | "medium" | "high" | "critical";

export interface AssessmentRecord {
  type: "AssessmentRecord";
  assessmentId: UUID;
  timestamp: ISO8601;
  userId: string;
  assessment: AssessmentData;
  clinician?: ClinicianReview;
}

export interface AssessmentData {
  name: AssessmentType;
  version: string;
  questions: AssessmentQuestion[];
  score: AssessmentScore;
  completedAt: ISO8601;
  durationSeconds: number;
}

export interface AssessmentQuestion {
  id: string;
  text: string;
  response: number | string;
  scale: string; // e.g., "0-3", "1-5"
  timeTaken?: number; // seconds
  skipped?: boolean;
}

export interface AssessmentScore {
  total: number;
  subscales?: Record<string, number>;
  percentile?: number; // 0-100
  interpretation: AssessmentInterpretation;
  riskLevel: RiskLevel;
  clinicalCutoff?: number;
  reliabilityCheck?: {
    passed: boolean;
    inconsistencies: string[];
  };
}

export interface ClinicianReview {
  reviewed: boolean;
  reviewerId?: string;
  reviewedAt?: ISO8601;
  notes?: EncryptedString;
  followUp: boolean;
  recommendations?: string[];
}

export interface AssessmentHistory {
  assessments: AssessmentRecord[];
  longitudinalData: {
    firstAssessment: ISO8601;
    totalAssessments: number;
    averageScore: number;
    trend: "improving" | "stable" | "declining";
    changePoints?: ISO8601[]; // Significant changes
  };
}

// ============================================================================
// Therapy Session Types
// ============================================================================

export type TherapyModality =
  | "CBT"              // Cognitive Behavioral Therapy
  | "DBT"              // Dialectical Behavior Therapy
  | "ACT"              // Acceptance and Commitment Therapy
  | "EMDR"             // Eye Movement Desensitization
  | "psychodynamic"
  | "humanistic"
  | "integrative"
  | "mindfulness-based"
  | "exposure-therapy"
  | "interpersonal"
  | "family-systems"
  | "psychoanalytic";

export type SessionType = "individual" | "group" | "couple" | "family";
export type SessionFormat = "in-person" | "video" | "audio" | "text";
export type SessionPhase = "assessment" | "active" | "maintenance" | "termination";

export interface TherapySessionRecord {
  type: "TherapySessionRecord";
  sessionId: UUID;
  timestamp: ISO8601;
  userId: string;
  therapistId: string;
  session: SessionDetails;
  content: SessionContent;
  outcomes: SessionOutcomes;
  notes?: SessionNotes;
}

export interface SessionDetails {
  modality: TherapyModality;
  type: SessionType;
  format: SessionFormat;
  duration: number; // minutes
  sessionNumber: number;
  phase: SessionPhase;
  scheduled: ISO8601;
  started?: ISO8601;
  ended?: ISO8601;
}

export interface SessionContent {
  topics: string[];
  techniques: string[];
  homework: string[];
  goals: TherapyGoal[];
  skillsPracticed?: string[];
  worksheetsCompleted?: string[];
}

export interface TherapyGoal {
  id: UUID;
  description: string;
  progress: number; // 0-100
  status: "not-started" | "in-progress" | "achieved" | "modified" | "discontinued";
  targetDate?: ISO8601;
  barriers?: string[];
  facilitators?: string[];
}

export interface SessionOutcomes {
  clientSatisfaction: number; // 1-5
  therapeuticAlliance: number; // 1-7 (Working Alliance Inventory)
  symptomChange: number; // -10 to 10
  functionalImprovement: number; // 1-5
  homeworkCompliance?: number; // 0-100
  sessionProductivity?: number; // 1-10
}

export interface SessionNotes {
  subjective: EncryptedString; // Patient's report
  objective: EncryptedString; // Therapist's observations
  assessment: EncryptedString; // Clinical assessment
  plan: EncryptedString; // Treatment plan
  format?: "SOAP" | "DAP" | "BIRP"; // Note format
}

export interface TherapyProgress {
  totalSessions: number;
  startDate: ISO8601;
  currentPhase: SessionPhase;
  goalsAchieved: number;
  goalsInProgress: number;
  averageSatisfaction: number;
  symptomTrajectory: "improving" | "stable" | "declining";
  recommendedActions?: string[];
}

// ============================================================================
// Crisis Intervention Types
// ============================================================================

export type CrisisSeverity = "low" | "medium" | "high" | "imminent";

export type CrisisType =
  | "suicidal"
  | "self-harm"
  | "panic"
  | "psychosis"
  | "substance"
  | "trauma"
  | "domestic-violence"
  | "homicidal";

export type SuicidalIdeation =
  | "none"
  | "passive"        // Wishes to be dead but no plan
  | "active"         // Thoughts of suicide
  | "intent"         // Intent to act
  | "plan"           // Has a plan
  | "means";         // Has means and plan

export type Disposition =
  | "self-care"
  | "outpatient"
  | "intensive-outpatient"
  | "partial-hospitalization"
  | "inpatient"
  | "hospitalization";

export interface CrisisInterventionRecord {
  type: "CrisisInterventionRecord";
  crisisId: UUID;
  timestamp: ISO8601;
  userId: string;
  crisis: CrisisDetails;
  assessment: CrisisAssessment;
  intervention: CrisisIntervention;
  followUp: CrisisFollowUp;
}

export interface CrisisDetails {
  severity: CrisisSeverity;
  type: CrisisType;
  triggers?: string[];
  symptoms?: string[];
  riskFactors: string[];
  protectiveFactors: string[];
  onsetTime?: ISO8601;
  duration?: number; // minutes
}

export interface CrisisAssessment {
  suicidalIdeation: SuicidalIdeation;
  selfHarmRisk: CrisisSeverity;
  homicidalIdeation: "none" | "present";
  impulsivity: number; // 1-10
  substanceUse: boolean;
  substanceType?: string;
  support: "strong" | "moderate" | "weak" | "none";
  mentalStatus?: {
    orientation: string;
    coherence: string;
    mood: string;
    affect: string;
  };
}

export interface CrisisIntervention {
  responder: "ai" | "counselor" | "crisis-team" | "emergency-services";
  responderId?: string;
  responseTime: number; // seconds
  actions: string[];
  safetyPlan?: SafetyPlan;
  disposition: Disposition;
  referrals?: string[];
  medicationChanges?: string[];
}

export interface SafetyPlan {
  warningSignsIdentified: string[];
  internalCopingStrategies: string[];
  socialContacts: {
    name: string; // Anonymized
    relationship: string;
    phone?: string; // Encrypted
  }[];
  professionalContacts: {
    name: string;
    role: string;
    phone: string;
    available: string;
  }[];
  environmentSafety: string[];
  reasonsForLiving: string[];
  emergencyNumbers: {
    crisis_line: string;
    emergency: string;
    local_crisis_team?: string;
  };
}

export interface CrisisFollowUp {
  scheduled: ISO8601[];
  contacts: string[];
  monitoring: string;
  checkIns: {
    timestamp: ISO8601;
    status: string;
    concerns?: string[];
  }[];
}

// ============================================================================
// Mindfulness & Meditation Types
// ============================================================================

export type MeditationType =
  | "meditation"
  | "breathing"
  | "body-scan"
  | "yoga"
  | "tai-chi"
  | "walking"
  | "progressive-muscle-relaxation";

export type MeditationTechnique =
  | "mindfulness"
  | "transcendental"
  | "vipassana"
  | "loving-kindness"
  | "zen"
  | "guided-imagery"
  | "mantra";

export interface MindfulnessSessionRecord {
  type: "MindfulnessSessionRecord";
  sessionId: UUID;
  timestamp: ISO8601;
  userId: string;
  session: MindfulnessSession;
  experience: MeditationExperience;
  biometrics?: PhysiologicalData;
}

export interface MindfulnessSession {
  type: MeditationType;
  technique?: MeditationTechnique;
  duration: number; // seconds
  targetDuration: number; // seconds
  guided: boolean;
  instructor?: string;
  audioUrl?: string;
  environmentContext?: string;
}

export interface MeditationExperience {
  focusQuality: number; // 1-10
  distractions: number; // 0-10
  emotionalState: {
    before: string;
    after: string;
    change: number; // -10 to 10
  };
  physicalSensations: string[];
  insights: string[];
  challenges?: string[];
  achievements?: string[];
}

export interface MindfulnessProgress {
  totalSessions: number;
  totalMinutes: number;
  averageDuration: number;
  currentStreak: number;
  longestStreak: number;
  favoriteType: MeditationType;
  averageFocus: number;
  improvements: {
    focusQuality: number;
    stressReduction: number;
    emotionalRegulation: number;
  };
}

// ============================================================================
// Reports & Analytics Types
// ============================================================================

export type ReportType =
  | "comprehensive"
  | "mood-only"
  | "therapy-progress"
  | "crisis-summary"
  | "wellness-dashboard";

export interface WellnessReport {
  reportId: UUID;
  userId: string;
  generatedAt: ISO8601;
  reportType: ReportType;
  dateRange: {
    start: ISO8601;
    end: ISO8601;
  };
  summary: WellnessSummary;
  sections: ReportSection[];
  recommendations: string[];
  downloadUrl?: string;
  expiresAt?: ISO8601;
}

export interface WellnessSummary {
  overallWellness: number; // 0-100
  moodStability: "excellent" | "good" | "moderate" | "poor";
  therapyProgress: "excellent" | "good" | "moderate" | "needs-attention";
  riskLevel: RiskLevel;
  engagementScore: number; // 0-100
  adherenceScore?: number; // 0-100
}

export interface ReportSection {
  title: string;
  type: string;
  data: Record<string, any>;
  visualizations?: {
    type: "chart" | "graph" | "table";
    config: Record<string, any>;
  }[];
  narrative?: string;
}

// ============================================================================
// User & Consent Types
// ============================================================================

export interface UserProfile {
  userId: UUID;
  demographics?: {
    ageRange?: string; // e.g., "25-34"
    gender?: string;
    timezone: string;
    language: string;
  };
  preferences: {
    notifications: boolean;
    reminderFrequency?: "daily" | "weekly" | "custom";
    privacyLevel: "maximum" | "high" | "standard";
    dataSharing: DataSharingPreferences;
  };
  clinical?: {
    diagnoses?: string[]; // Encrypted
    medications?: string[]; // Encrypted
    allergies?: string[]; // Encrypted
    emergencyContact?: EncryptedString;
  };
}

export interface DataSharingPreferences {
  therapist: boolean;
  research: boolean;
  qualityImprovement: boolean;
  emergencyContacts: boolean;
  specificProviders?: string[];
}

export interface ConsentRecord {
  consentId: UUID;
  userId: string;
  timestamp: ISO8601;
  consentType: "treatment" | "data-collection" | "data-sharing" | "research" | "crisis-protocol";
  granted: boolean;
  scope?: string[];
  expiresAt?: ISO8601;
  revocable: boolean;
  signature: string;
  version: string;
}

// ============================================================================
// API Request/Response Types
// ============================================================================

export interface APIResponse<T = any> {
  success: boolean;
  data?: T;
  error?: APIError;
  timestamp: ISO8601;
  requestId?: UUID;
}

export interface APIError {
  code: string;
  message: string;
  details?: string;
  timestamp: ISO8601;
  retryable?: boolean;
}

export interface PaginatedResponse<T> {
  success: boolean;
  count: number;
  data: T[];
  pagination: {
    limit: number;
    offset: number;
    total: number;
    hasMore: boolean;
  };
}

// ============================================================================
// Configuration Types
// ============================================================================

export interface MentalWellnessConfig {
  apiKey: string;
  baseUrl?: string;
  userId?: string;
  timeout?: number; // milliseconds
  retryAttempts?: number;
  privacyLevel?: "maximum" | "high" | "standard";
  encryptionEnabled?: boolean;
}

// ============================================================================
// Biometric Integration Types
// ============================================================================

export interface BiometricCorrelation {
  moodVsBiometrics: {
    heartRate: number; // correlation coefficient -1 to 1
    hrv: number;
    sleep: number;
    activity: number;
  };
  insights: string[];
  recommendations: string[];
  confidence: number; // 0-1
}

export interface WearableData {
  deviceType: string;
  timestamp: ISO8601;
  heartRate?: number;
  hrv?: number;
  steps?: number;
  activeMinutes?: number;
  sleepData?: {
    duration: number;
    quality: number;
    stages: {
      deep: number;
      light: number;
      rem: number;
      awake: number;
    };
  };
  stressLevel?: number;
}

// ============================================================================
// Treatment Plan Types
// ============================================================================

export interface TreatmentPlan {
  planId: UUID;
  userId: string;
  therapistId: string;
  createdAt: ISO8601;
  updatedAt: ISO8601;
  diagnosis: {
    primary: string;
    secondary?: string[];
    icd10Codes: string[];
  };
  goals: TherapyGoal[];
  interventions: {
    type: string;
    frequency: string;
    duration: string;
  }[];
  expectedOutcomes: string[];
  reviewDate: ISO8601;
  status: "active" | "completed" | "discontinued";
}

// ============================================================================
// Export all types
// ============================================================================

export default {
  // Main record types
  WIAMentalWellnessRecord,
  MoodRecord,
  AssessmentRecord,
  TherapySessionRecord,
  CrisisInterventionRecord,
  MindfulnessSessionRecord,

  // Configuration
  MentalWellnessConfig,

  // Reports
  WellnessReport,

  // User
  UserProfile,
  ConsentRecord,
};
