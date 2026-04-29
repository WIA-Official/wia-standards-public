/**
 * WIA-MENTAL-001: Digital Therapy Standard
 * TypeScript Type Definitions
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * @version 1.0.0
 * @license MIT
 */

// ============================================================================
// Core Types
// ============================================================================

export type TherapyType =
  | 'CBT'   // Cognitive Behavioral Therapy
  | 'DBT'   // Dialectical Behavior Therapy
  | 'ACT'   // Acceptance and Commitment Therapy
  | 'IPT'   // Interpersonal Therapy
  | 'MBCT'  // Mindfulness-Based Cognitive Therapy
  | 'PE'    // Prolonged Exposure Therapy
  | 'CPT'   // Cognitive Processing Therapy
  | 'EMDR'; // Eye Movement Desensitization and Reprocessing

export type SessionStatus =
  | 'scheduled'
  | 'active'
  | 'paused'
  | 'completed'
  | 'cancelled'
  | 'no-show';

export type SeverityLevel =
  | 'minimal'
  | 'mild'
  | 'moderate'
  | 'moderately-severe'
  | 'severe';

export type ComplianceStandard =
  | 'HIPAA'
  | 'GDPR'
  | 'CCPA'
  | 'PIPEDA'
  | 'SOC2';

// ============================================================================
// Session Configuration
// ============================================================================

export interface TherapySessionConfig {
  /** Unique session identifier */
  sessionId?: string;

  /** Type of therapy being delivered */
  type: TherapyType;

  /** Patient identifier */
  patientId: string;

  /** Therapist/provider identifier (optional for self-guided) */
  therapistId?: string;

  /** Session duration in minutes */
  duration?: number;

  /** Privacy and security configuration */
  privacy: PrivacyConfig;

  /** Enable adaptive content based on patient responses */
  adaptive?: boolean;

  /** Language preference (ISO 639-1 code) */
  language?: string;

  /** Custom metadata */
  metadata?: Record<string, any>;
}

export interface PrivacyConfig {
  /** Encryption algorithm (e.g., 'AES-256') */
  encryption: string;

  /** Compliance standards to adhere to */
  compliance: ComplianceStandard[];

  /** Enable anonymization of sensitive data */
  anonymize?: boolean;

  /** Data retention period in days */
  retentionDays?: number;

  /** Allow data sharing with research (de-identified) */
  allowResearch?: boolean;
}

// ============================================================================
// Session Parameters
// ============================================================================

export interface SessionStartParams {
  /** Specific therapeutic module to run */
  module: string;

  /** Session duration in minutes */
  duration: number;

  /** Enable adaptive difficulty/content */
  adaptive?: boolean;

  /** Focus area for the session */
  focusArea?: FocusArea;

  /** Initial mood/state assessment */
  initialState?: PatientState;
}

export type FocusArea =
  | 'anxiety'
  | 'depression'
  | 'trauma'
  | 'stress'
  | 'sleep'
  | 'relationships'
  | 'self-esteem'
  | 'anger'
  | 'grief'
  | 'addiction';

// ============================================================================
// Patient Data
// ============================================================================

export interface PatientState {
  /** Current mood rating (1-10) */
  mood: number;

  /** Anxiety level (1-10) */
  anxiety: number;

  /** Energy level (1-10) */
  energy: number;

  /** Sleep quality (1-10) */
  sleep: number;

  /** Additional symptoms */
  symptoms?: string[];

  /** Free-text notes from patient */
  notes?: string;

  /** Timestamp of assessment */
  timestamp: Date;
}

export interface PatientProfile {
  /** Unique patient identifier */
  patientId: string;

  /** Age (optional, for age-appropriate content) */
  age?: number;

  /** Primary diagnosis/concern */
  primaryConcern: FocusArea;

  /** Additional concerns */
  secondaryConcerns?: FocusArea[];

  /** Medication information */
  medications?: Medication[];

  /** Previous mental health treatment */
  treatmentHistory?: TreatmentHistory[];

  /** Preferences for therapy */
  preferences?: PatientPreferences;

  /** Emergency contact information */
  emergencyContact?: EmergencyContact;
}

export interface Medication {
  name: string;
  dosage: string;
  frequency: string;
  startDate: Date;
  prescribedFor: string;
}

export interface TreatmentHistory {
  type: string;
  startDate: Date;
  endDate?: Date;
  provider: string;
  outcome?: string;
  notes?: string;
}

export interface PatientPreferences {
  /** Preferred communication style */
  communicationStyle?: 'direct' | 'gentle' | 'motivational';

  /** Preferred session time of day */
  preferredTime?: 'morning' | 'afternoon' | 'evening';

  /** Reminders enabled */
  reminders?: boolean;

  /** Homework/practice reminders */
  homeworkReminders?: boolean;
}

export interface EmergencyContact {
  name: string;
  relationship: string;
  phone: string;
  email?: string;
}

// ============================================================================
// Assessment Tools
// ============================================================================

export interface Assessment {
  /** Assessment type/name */
  type: AssessmentType;

  /** Score achieved */
  score: number;

  /** Severity interpretation */
  severity: SeverityLevel;

  /** Detailed results */
  results: AssessmentResult[];

  /** Timestamp of assessment */
  timestamp: Date;

  /** Clinician recommendations */
  recommendations?: string[];
}

export type AssessmentType =
  | 'PHQ-9'      // Depression
  | 'GAD-7'      // Anxiety
  | 'PCL-5'      // PTSD
  | 'MDQ'        // Mood Disorder
  | 'AUDIT'      // Alcohol Use
  | 'DAST-10'    // Drug Abuse
  | 'ISI'        // Insomnia Severity
  | 'PSS'        // Perceived Stress
  | 'WHODAS'     // Disability Assessment
  | 'Custom';

export interface AssessmentResult {
  question: string;
  response: number | string;
  domain?: string;
}

// ============================================================================
// Progress Tracking
// ============================================================================

export interface ProgressMetrics {
  /** Overall effectiveness percentage */
  effectiveness: number;

  /** Patient engagement score */
  engagement: number;

  /** Symptom improvement percentage */
  symptomImprovement: number;

  /** Number of completed sessions */
  sessionsCompleted: number;

  /** Number of missed sessions */
  sessionsMissed: number;

  /** Homework completion rate */
  homeworkCompletion: number;

  /** Skills acquisition score */
  skillsAcquired: number;

  /** Trend direction */
  trend: 'improving' | 'stable' | 'declining';

  /** Detailed metrics by domain */
  domainMetrics?: DomainMetrics[];

  /** Last updated timestamp */
  lastUpdated: Date;
}

export interface DomainMetrics {
  domain: string;
  baseline: number;
  current: number;
  change: number;
  percentChange: number;
}

// ============================================================================
// Therapy Session
// ============================================================================

export interface TherapySession {
  /** Unique session identifier */
  sessionId: string;

  /** Session configuration */
  config: TherapySessionConfig;

  /** Session start time */
  startTime: Date;

  /** Session end time */
  endTime?: Date;

  /** Current status */
  status: SessionStatus;

  /** Activities completed during session */
  activities: SessionActivity[];

  /** Patient state at start */
  preSessionState: PatientState;

  /** Patient state at end */
  postSessionState?: PatientState;

  /** Homework assigned */
  homework?: Homework[];

  /** Session notes */
  notes?: string[];

  /** Therapist observations (if applicable) */
  therapistNotes?: string;
}

export interface SessionActivity {
  type: string;
  startTime: Date;
  endTime?: Date;
  completed: boolean;
  data?: Record<string, any>;
}

export interface Homework {
  id: string;
  title: string;
  description: string;
  dueDate: Date;
  completed: boolean;
  completedDate?: Date;
  notes?: string;
}

// ============================================================================
// API Response Types
// ============================================================================

export interface APIResponse<T> {
  success: boolean;
  data?: T;
  error?: APIError;
  timestamp: Date;
}

export interface APIError {
  code: string;
  message: string;
  details?: Record<string, any>;
}

// ============================================================================
// Intervention Recommendations
// ============================================================================

export interface Recommendation {
  type: 'therapy' | 'medication' | 'lifestyle' | 'referral' | 'crisis';
  priority: 'low' | 'medium' | 'high' | 'urgent';
  title: string;
  description: string;
  rationale: string;
  resources?: Resource[];
}

export interface Resource {
  type: 'article' | 'video' | 'exercise' | 'app' | 'hotline';
  title: string;
  url?: string;
  description: string;
}

// ============================================================================
// Crisis Detection
// ============================================================================

export interface CrisisAlert {
  level: 'warning' | 'high' | 'critical';
  triggers: string[];
  recommendation: string;
  resources: CrisisResource[];
  timestamp: Date;
  acknowledged: boolean;
}

export interface CrisisResource {
  name: string;
  phone: string;
  available: string;
  description: string;
}

// ============================================================================
// Analytics & Reporting
// ============================================================================

export interface TherapyAnalytics {
  totalSessions: number;
  averageDuration: number;
  completionRate: number;
  patientSatisfaction: number;
  clinicalOutcomes: OutcomeMetrics;
  demographicBreakdown?: DemographicData;
}

export interface OutcomeMetrics {
  remissionRate: number;
  responseRate: number;
  deteriorationRate: number;
  dropoutRate: number;
}

export interface DemographicData {
  ageDistribution: Record<string, number>;
  genderDistribution: Record<string, number>;
  diagnosisDistribution: Record<string, number>;
}

// ============================================================================
// Export Main Class Interface
// ============================================================================

export interface IDigitalTherapySession {
  start(params: SessionStartParams): Promise<void>;
  pause(): Promise<void>;
  resume(): Promise<void>;
  end(): Promise<void>;
  trackProgress(): Promise<ProgressMetrics>;
  runAssessment(type: AssessmentType): Promise<Assessment>;
  updateState(state: Partial<PatientState>): Promise<void>;
  getRecommendations(): Promise<Recommendation[]>;
  checkCrisis(): Promise<CrisisAlert | null>;
}

/**
 * 弘益人間 (홍익인간)
 * Benefit All Humanity
 *
 * Mental Health Matters
 */
