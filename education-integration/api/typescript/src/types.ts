/**
 * WIA-UNI-010: Education Integration Standard
 * TypeScript Type Definitions
 *
 * @package @wia/education-integration
 * @version 1.0.0
 * @license MIT
 */

// ============================================================================
// Core Types
// ============================================================================

export type Region = 'north' | 'south';
export type Language = 'ko-KR' | 'ko-KP' | 'unified-korean';
export type EducationLevel = 'elementary' | 'middle' | 'high' | 'university' | 'vocational';

// ============================================================================
// Student & Teacher Types
// ============================================================================

export interface Student {
  id: string;
  name: string;
  region: Region;
  school: string;
  grade: number;
  level: EducationLevel;
  dateOfBirth: Date;
  parentalConsent: boolean;
  metadata?: Record<string, any>;
}

export interface Teacher {
  id: string;
  name: string;
  region: Region;
  school: string;
  subjects: string[];
  certificationLevel: 1 | 2 | 3 | 'master';
  yearsExperience: number;
  integrationTraining: boolean;
}

// ============================================================================
// Curriculum Types
// ============================================================================

export interface Subject {
  id: string;
  name: string;
  nameKo: string;
  category: 'core' | 'elective' | 'vocational' | 'enrichment';
  compatibilityScore: number; // 0-100
  gradeRange: [number, number];
}

export interface CurriculumMapping {
  subject: Subject;
  northCurriculum: CurriculumContent;
  southCurriculum: CurriculumContent;
  unifiedObjectives: LearningObjective[];
  harmonizationStrategy: string;
}

export interface CurriculumContent {
  topics: string[];
  learningObjectives: string[];
  assessmentMethods: string[];
  resources: Resource[];
}

export interface LearningObjective {
  id: string;
  description: string;
  bloomLevel: 'remember' | 'understand' | 'apply' | 'analyze' | 'evaluate' | 'create';
  assessmentCriteria: string[];
}

export interface Resource {
  type: 'textbook' | 'video' | 'interactive' | 'document' | 'simulation';
  title: string;
  url?: string;
  provider: string;
  language: Language;
}

// ============================================================================
// Virtual Classroom Types
// ============================================================================

export interface VirtualClassroom {
  id: string;
  subject: string;
  grade: number;
  teachers: {
    north?: Teacher;
    south?: Teacher;
  };
  students: {
    north: Student[];
    south: Student[];
  };
  schedule: ClassSchedule;
  features: ClassroomFeature[];
  status: 'planned' | 'active' | 'completed' | 'cancelled';
  createdAt: Date;
  metadata?: Record<string, any>;
}

export interface ClassSchedule {
  frequency: 'daily' | 'weekly' | 'biweekly' | 'monthly';
  dayOfWeek?: number; // 0-6
  time: string; // HH:MM format
  duration: number; // minutes
  timezone: string;
  startDate: Date;
  endDate?: Date;
}

export type ClassroomFeature =
  | 'video-conferencing'
  | 'collaborative-documents'
  | 'breakout-rooms'
  | 'screen-sharing'
  | 'whiteboard'
  | 'chat'
  | 'recording'
  | 'translation'
  | 'polls'
  | 'assignments';

export interface ClassSession {
  id: string;
  classroomId: string;
  scheduledAt: Date;
  actualStart?: Date;
  actualEnd?: Date;
  attendance: Attendance[];
  activities: Activity[];
  recording?: Recording;
  notes?: string;
}

export interface Attendance {
  studentId: string;
  present: boolean;
  duration?: number; // minutes
  engagement?: number; // 0-100 score
}

export interface Activity {
  type: 'lecture' | 'discussion' | 'group-work' | 'presentation' | 'assessment';
  description: string;
  startTime: Date;
  duration: number;
  participants?: string[];
}

export interface Recording {
  url: string;
  duration: number;
  size: number; // bytes
  encryption: boolean;
  expiresAt: Date;
}

// ============================================================================
// Exchange Program Types
// ============================================================================

export interface ExchangeProgram {
  id: string;
  type: 'virtual' | 'short-term' | 'semester' | 'year';
  participant: Student | Teacher;
  homeRegion: Region;
  destinationRegion: Region;
  startDate: Date;
  endDate: Date;
  status: 'applied' | 'approved' | 'active' | 'completed' | 'cancelled';
  hostInstitution?: string;
  emergencyContact: Contact;
}

export interface Contact {
  name: string;
  relationship: string;
  phone: string;
  email: string;
  address?: string;
}

// ============================================================================
// Credential Types
// ============================================================================

export interface Credential {
  id: string;
  type: 'diploma' | 'degree' | 'certificate' | 'badge' | 'transcript';
  holder: Student | Teacher;
  issuer: Institution;
  issuedAt: Date;
  expiresAt?: Date;
  blockchain?: BlockchainInfo;
  recognitionScope: 'inter-korean' | 'regional' | 'international';
  metadata?: Record<string, any>;
}

export interface BlockchainInfo {
  network: string;
  contractAddress: string;
  tokenId?: string;
  transactionHash: string;
  verificationUrl: string;
}

export interface Institution {
  id: string;
  name: string;
  nameKo: string;
  region: Region;
  type: 'school' | 'university' | 'vocational' | 'training-center';
  accreditation: AccreditationInfo;
}

export interface AccreditationInfo {
  status: 'accredited' | 'provisional' | 'not-accredited';
  tier: 1 | 2 | 3;
  validUntil: Date;
  accreditingBody: string;
}

// ============================================================================
// Learning Management System Types
// ============================================================================

export interface Course {
  id: string;
  title: string;
  titleKo: string;
  description: string;
  subject: Subject;
  level: EducationLevel;
  instructor: Teacher;
  credits: number;
  duration: number; // weeks
  modules: Module[];
  assessments: Assessment[];
  enrolled: Student[];
}

export interface Module {
  id: string;
  title: string;
  order: number;
  content: Content[];
  estimatedHours: number;
  learningObjectives: LearningObjective[];
}

export interface Content {
  id: string;
  type: 'video' | 'document' | 'interactive' | 'quiz' | 'discussion';
  title: string;
  url?: string;
  data?: any;
  duration?: number;
  required: boolean;
}

export interface Assessment {
  id: string;
  type: 'quiz' | 'exam' | 'project' | 'presentation' | 'peer-review';
  title: string;
  description: string;
  dueDate?: Date;
  weight: number; // percentage of final grade
  passingScore: number;
  rubric?: Rubric;
}

export interface Rubric {
  criteria: Criterion[];
  totalPoints: number;
}

export interface Criterion {
  name: string;
  description: string;
  points: number;
  levels: RubricLevel[];
}

export interface RubricLevel {
  name: string;
  description: string;
  points: number;
}

// ============================================================================
// Analytics & Reporting Types
// ============================================================================

export interface LearningAnalytics {
  studentId: string;
  courseId: string;
  progress: number; // 0-100 percentage
  timeSpent: number; // minutes
  assessmentScores: number[];
  averageScore: number;
  engagement: number; // 0-100 score
  predictions: {
    completion: number; // probability 0-1
    finalGrade: number;
    atRisk: boolean;
  };
  recommendations: string[];
}

export interface ProgramMetrics {
  participatingSchools: number;
  enrolledStudents: number;
  certifiedTeachers: number;
  coursesOffered: number;
  completionRate: number;
  satisfactionScore: number;
  learningOutcomes: OutcomeMetrics;
}

export interface OutcomeMetrics {
  academicAchievement: number;
  culturalUnderstanding: number;
  friendshipFormation: number;
  skillDevelopment: number;
}

// ============================================================================
// API Request/Response Types
// ============================================================================

export interface APIResponse<T> {
  success: boolean;
  data?: T;
  error?: {
    code: string;
    message: string;
    details?: any;
  };
  metadata?: {
    timestamp: Date;
    requestId: string;
    version: string;
  };
}

export interface PaginatedResponse<T> extends APIResponse<T[]> {
  pagination: {
    page: number;
    pageSize: number;
    total: number;
    totalPages: number;
  };
}

// ============================================================================
// Configuration Types
// ============================================================================

export interface EducationIntegrationConfig {
  apiEndpoint: string;
  apiKey?: string;
  region?: Region;
  language?: Language;
  features?: ClassroomFeature[];
  security?: {
    encryption: boolean;
    authentication: 'basic' | 'oauth' | 'jwt';
    mfa: boolean;
  };
  platform?: {
    lms: boolean;
    virtualClassroom: boolean;
    credentials: boolean;
    analytics: boolean;
  };
}

// ============================================================================
// Event Types
// ============================================================================

export interface PlatformEvent {
  type: 'classroom.started' | 'classroom.ended' | 'student.enrolled' | 'credential.issued' | 'assessment.submitted';
  timestamp: Date;
  actor: string; // user ID
  resource: string; // resource ID
  data?: Record<string, any>;
}

// ============================================================================
// Utility Types
// ============================================================================

export type AsyncResult<T> = Promise<APIResponse<T>>;
export type Nullable<T> = T | null;
export type Optional<T> = T | undefined;
