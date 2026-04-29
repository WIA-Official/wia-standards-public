/**
 * WIA EDU Standard - TypeScript Type Definitions
 * Educational Platform Standard
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

export interface WIAEDU {
  standard: 'WIA-EDU';
  version: string;
  institution: EducationalInstitution;
  programs: EducationalProgram[];
  courses: Course[];
  learners: Learner[];
  instructors: Instructor[];
  assessments: Assessment[];
  credentials: Credential[];
  analytics: LearningAnalytics;
  extensions?: Record<string, unknown>;
}

export interface EducationalInstitution {
  id: string;
  name: string;
  type: InstitutionType;
  accreditation: AccreditationInfo[];
  location: LocationInfo;
  contact: ContactInfo;
  status: 'active' | 'inactive' | 'pending';
  createdAt: string;
}

export type InstitutionType = 'university' | 'college' | 'k12' | 'vocational' | 'online-platform' | 'corporate' | 'nonprofit';

export interface AccreditationInfo {
  body: string;
  status: 'accredited' | 'pending' | 'probation' | 'revoked';
  validUntil: string;
  scope: string[];
}

export interface LocationInfo {
  country: string;
  region?: string;
  city?: string;
  address?: string;
  timezone: string;
}

export interface ContactInfo {
  email: string;
  phone?: string;
  website?: string;
}

// ============================================================================
// Program Types
// ============================================================================

export interface EducationalProgram {
  id: string;
  name: string;
  type: ProgramType;
  level: EducationLevel;
  duration: DurationInfo;
  requirements: ProgramRequirements;
  outcomes: LearningOutcome[];
  curriculum: Curriculum;
  status: 'active' | 'archived' | 'development';
}

export type ProgramType = 'degree' | 'certificate' | 'diploma' | 'bootcamp' | 'workshop' | 'mooc' | 'micro-credential';
export type EducationLevel = 'primary' | 'secondary' | 'undergraduate' | 'graduate' | 'doctoral' | 'professional' | 'continuing';

export interface DurationInfo {
  value: number;
  unit: 'hours' | 'days' | 'weeks' | 'months' | 'years';
  flexible: boolean;
}

export interface ProgramRequirements {
  prerequisites: string[];
  admissionCriteria: string[];
  minimumCredits?: number;
  minimumGPA?: number;
}

export interface LearningOutcome {
  id: string;
  description: string;
  category: OutcomeCategory;
  level: BloomLevel;
  assessmentMethods: string[];
}

export type OutcomeCategory = 'knowledge' | 'skill' | 'competency' | 'attitude' | 'behavior';
export type BloomLevel = 'remember' | 'understand' | 'apply' | 'analyze' | 'evaluate' | 'create';

export interface Curriculum {
  structure: CurriculumStructure;
  modules: CurriculumModule[];
  totalCredits: number;
  electiveCredits: number;
}

export interface CurriculumStructure {
  type: 'linear' | 'modular' | 'competency-based' | 'flexible';
  pathways?: string[];
}

export interface CurriculumModule {
  id: string;
  name: string;
  courses: string[];
  required: boolean;
  credits: number;
}

// ============================================================================
// Course Types
// ============================================================================

export interface Course {
  id: string;
  code: string;
  name: string;
  description: string;
  credits: number;
  format: CourseFormat;
  schedule: CourseSchedule;
  capacity: CourseCapacity;
  materials: CourseMaterial[];
  objectives: string[];
  syllabus: string;
  status: 'open' | 'closed' | 'in-progress' | 'completed';
}

export type CourseFormat = 'in-person' | 'online-sync' | 'online-async' | 'hybrid' | 'blended';

export interface CourseSchedule {
  startDate: string;
  endDate: string;
  sessions: SessionInfo[];
  selfPaced: boolean;
}

export interface SessionInfo {
  day: string;
  startTime: string;
  endTime: string;
  location?: string;
  online?: boolean;
}

export interface CourseCapacity {
  minimum: number;
  maximum: number;
  enrolled: number;
  waitlist: number;
}

export interface CourseMaterial {
  id: string;
  type: MaterialType;
  title: string;
  url?: string;
  required: boolean;
  accessible: boolean;
}

export type MaterialType = 'textbook' | 'video' | 'article' | 'simulation' | 'lab' | 'software' | 'other';

// ============================================================================
// Learner Types
// ============================================================================

export interface Learner {
  id: string;
  profile: LearnerProfile;
  enrollments: Enrollment[];
  progress: LearningProgress;
  achievements: Achievement[];
  preferences: LearningPreferences;
}

export interface LearnerProfile {
  name: string;
  email: string;
  studentId?: string;
  status: 'active' | 'inactive' | 'graduated' | 'suspended';
  enrolledSince: string;
}

export interface Enrollment {
  courseId: string;
  programId?: string;
  status: 'enrolled' | 'completed' | 'withdrawn' | 'failed';
  grade?: string;
  completedAt?: string;
}

export interface LearningProgress {
  overallCompletion: number;
  coursesCompleted: number;
  creditsEarned: number;
  currentGPA?: number;
  milestones: ProgressMilestone[];
}

export interface ProgressMilestone {
  id: string;
  name: string;
  achieved: boolean;
  date?: string;
}

export interface Achievement {
  id: string;
  type: 'badge' | 'certificate' | 'award' | 'recognition';
  name: string;
  description: string;
  earnedAt: string;
  issuer: string;
}

export interface LearningPreferences {
  style: LearningStyle[];
  pace: 'slow' | 'moderate' | 'fast';
  accessibility: AccessibilityNeeds[];
  language: string;
}

export type LearningStyle = 'visual' | 'auditory' | 'reading' | 'kinesthetic';
export type AccessibilityNeeds = 'screen-reader' | 'captions' | 'large-text' | 'high-contrast' | 'keyboard-nav';

// ============================================================================
// Instructor Types
// ============================================================================

export interface Instructor {
  id: string;
  profile: InstructorProfile;
  qualifications: Qualification[];
  courses: string[];
  availability: AvailabilitySchedule;
  ratings: InstructorRating;
}

export interface InstructorProfile {
  name: string;
  email: string;
  title: string;
  department?: string;
  bio?: string;
  specializations: string[];
}

export interface Qualification {
  type: 'degree' | 'certification' | 'license' | 'experience';
  name: string;
  institution?: string;
  year: number;
}

export interface AvailabilitySchedule {
  officeHours: SessionInfo[];
  consultationBooking: boolean;
}

export interface InstructorRating {
  overall: number;
  teaching: number;
  responsiveness: number;
  reviewCount: number;
}

// ============================================================================
// Assessment Types
// ============================================================================

export interface Assessment {
  id: string;
  courseId: string;
  type: AssessmentType;
  name: string;
  description: string;
  weight: number;
  dueDate: string;
  config: AssessmentConfig;
  rubric?: Rubric;
  status: 'draft' | 'published' | 'closed' | 'graded';
}

export type AssessmentType = 'quiz' | 'exam' | 'assignment' | 'project' | 'presentation' | 'discussion' | 'peer-review' | 'portfolio';

export interface AssessmentConfig {
  duration?: number;
  attempts?: number;
  randomized?: boolean;
  proctored?: boolean;
  openBook?: boolean;
  groupWork?: boolean;
}

export interface Rubric {
  id: string;
  criteria: RubricCriterion[];
  totalPoints: number;
}

export interface RubricCriterion {
  name: string;
  description: string;
  maxPoints: number;
  levels: RubricLevel[];
}

export interface RubricLevel {
  name: string;
  points: number;
  description: string;
}

// ============================================================================
// Credential Types
// ============================================================================

export interface Credential {
  id: string;
  type: CredentialType;
  name: string;
  recipient: string;
  issuer: string;
  issuedAt: string;
  expiresAt?: string;
  verification: VerificationInfo;
  skills: string[];
  achievements: string[];
}

export type CredentialType = 'degree' | 'certificate' | 'badge' | 'micro-credential' | 'transcript';

export interface VerificationInfo {
  method: 'blockchain' | 'digital-signature' | 'qr-code' | 'url';
  verificationUrl?: string;
  hash?: string;
  status: 'valid' | 'expired' | 'revoked';
}

// ============================================================================
// Analytics Types
// ============================================================================

export interface LearningAnalytics {
  engagement: EngagementMetrics;
  performance: PerformanceMetrics;
  retention: RetentionMetrics;
  predictions: PredictiveInsights;
}

export interface EngagementMetrics {
  activeUsers: number;
  averageSessionDuration: number;
  contentInteractions: number;
  discussionParticipation: number;
}

export interface PerformanceMetrics {
  averageGrade: number;
  passRate: number;
  completionRate: number;
  outcomeAchievement: number;
}

export interface RetentionMetrics {
  enrollmentRetention: number;
  courseCompletion: number;
  programCompletion: number;
  churnRate: number;
}

export interface PredictiveInsights {
  atRiskLearners: string[];
  successPredictions: { learnerId: string; probability: number }[];
  recommendedInterventions: Intervention[];
}

export interface Intervention {
  type: 'tutoring' | 'counseling' | 'resources' | 'pacing-adjustment';
  targetLearners: string[];
  priority: 'high' | 'medium' | 'low';
}

// ============================================================================
// API Types
// ============================================================================

export interface APIConfig {
  baseURL: string;
  apiKey?: string;
  timeout?: number;
}

export interface InstitutionResponse {
  id: string;
  name: string;
  type: InstitutionType;
  programCount: number;
  learnerCount: number;
  links: { self: string };
}

export interface ValidationResult {
  valid: boolean;
  errors?: { path: string; message: string }[];
}

export interface PaginatedResponse<T> {
  data: T[];
  pagination: { total: number; limit: number; offset: number; hasMore: boolean };
}
