/**
 * WIA-EDU-009: Learning Management System Standard
 * TypeScript Type Definitions
 *
 * @package @wia/lms
 * @version 1.0.0
 * @license MIT
 */

// ============================================================================
// Core Types
// ============================================================================

export type UserRole = 'student' | 'instructor' | 'ta' | 'observer' | 'designer' | 'admin';
export type EnrollmentStatus = 'active' | 'completed' | 'dropped' | 'pending' | 'waitlisted';
export type CourseFormat = 'online' | 'hybrid' | 'in-person';
export type GradingScheme = 'points' | 'weighted' | 'letter' | 'pass-fail' | 'competency';

// ============================================================================
// User Types
// ============================================================================

export interface User {
  id: string;
  email: string;
  name: string;
  username: string;
  avatar?: string;
  timezone: string;
  locale: string;
  createdAt: Date;
  lastLogin?: Date;
}

export interface Student extends User {
  studentId: string;
  program?: string;
  major?: string;
  yearLevel?: number;
  gpa?: number;
}

export interface Instructor extends User {
  employeeId: string;
  department: string;
  title: string;
  bio?: string;
  officeHours?: OfficeHours[];
}

export interface OfficeHours {
  dayOfWeek: number; // 0-6 (Sunday-Saturday)
  startTime: string; // HH:mm format
  endTime: string;
  location: string;
  virtual?: boolean;
  zoomLink?: string;
}

// ============================================================================
// Course Types
// ============================================================================

export interface Course {
  id: string;
  code: string;
  title: string;
  description: string;
  format: CourseFormat;
  credits: number;
  term: string;
  startDate: Date;
  endDate: Date;
  instructors: Instructor[];
  maxStudents?: number;
  enrollmentCount: number;
  syllabus?: string;
  prerequisites?: string[];
  learningObjectives?: string[];
  gradingScheme: GradingScheme;
  published: boolean;
}

export interface Module {
  id: string;
  courseId: string;
  title: string;
  description?: string;
  position: number;
  unlockAt?: Date;
  prerequisites?: string[]; // Module IDs
  items: ModuleItem[];
}

export interface ModuleItem {
  id: string;
  type: 'assignment' | 'quiz' | 'discussion' | 'page' | 'file' | 'url' | 'external-tool';
  title: string;
  contentId: string;
  position: number;
  required: boolean;
  completionRequirement?: CompletionRequirement;
}

export interface CompletionRequirement {
  type: 'must_view' | 'must_submit' | 'must_score' | 'must_contribute';
  minScore?: number;
}

// ============================================================================
// Enrollment Types
// ============================================================================

export interface Enrollment {
  id: string;
  userId: string;
  courseId: string;
  role: UserRole;
  status: EnrollmentStatus;
  enrolledAt: Date;
  completedAt?: Date;
  currentGrade?: number;
  finalGrade?: string;
}

export interface WaitlistEntry {
  id: string;
  userId: string;
  courseId: string;
  position: number;
  createdAt: Date;
}

// ============================================================================
// Assignment Types
// ============================================================================

export type AssignmentType = 'online_upload' | 'online_text' | 'external_tool' | 'on_paper' | 'discussion';
export type SubmissionStatus = 'submitted' | 'graded' | 'late' | 'missing' | 'pending_review';

export interface Assignment {
  id: string;
  courseId: string;
  title: string;
  description: string;
  instructions?: string;
  assignmentType: AssignmentType;
  points: number;
  dueDate: Date;
  availableFrom?: Date;
  availableUntil?: Date;
  allowLateSubmissions: boolean;
  latePolicy?: LatePolicy;
  submissionTypes: string[];
  allowedExtensions?: string[];
  peerReviews?: PeerReviewConfig;
  rubric?: Rubric;
  groupAssignment: boolean;
  published: boolean;
}

export interface LatePolicy {
  deductionType: 'percentage' | 'points';
  deductionAmount: number;
  deductionInterval: 'hour' | 'day';
  maxDeduction?: number;
}

export interface Submission {
  id: string;
  assignmentId: string;
  userId: string;
  submittedAt: Date;
  status: SubmissionStatus;
  attemptNumber: number;
  late: boolean;
  files?: FileAttachment[];
  textContent?: string;
  externalToolUrl?: string;
  score?: number;
  grade?: string;
  feedback?: string;
  rubricAssessment?: RubricAssessment;
}

export interface FileAttachment {
  id: string;
  filename: string;
  contentType: string;
  size: number;
  url: string;
  uploadedAt: Date;
}

// ============================================================================
// Rubric Types
// ============================================================================

export type RubricType = 'analytic' | 'holistic' | 'single-point';

export interface Rubric {
  id: string;
  title: string;
  type: RubricType;
  criteria: RubricCriterion[];
  pointsPossible: number;
}

export interface RubricCriterion {
  id: string;
  description: string;
  points: number;
  ratings: RubricRating[];
  learningOutcomeId?: string;
}

export interface RubricRating {
  id: string;
  description: string;
  points: number;
}

export interface RubricAssessment {
  rubricId: string;
  assessments: CriterionAssessment[];
  totalScore: number;
  comments?: string;
}

export interface CriterionAssessment {
  criterionId: string;
  ratingId: string;
  points: number;
  comments?: string;
}

// ============================================================================
// Peer Review Types
// ============================================================================

export interface PeerReviewConfig {
  enabled: boolean;
  assignmentCount: number; // How many peers each student reviews
  anonymous: boolean;
  dueDate?: Date;
  requireInitialSubmission: boolean;
}

export interface PeerReview {
  id: string;
  submissionId: string;
  reviewerId: string;
  rubricAssessment?: RubricAssessment;
  comments: string;
  submittedAt?: Date;
}

// ============================================================================
// Quiz Types
// ============================================================================

export type QuestionType =
  | 'multiple_choice'
  | 'true_false'
  | 'fill_in_blank'
  | 'essay'
  | 'matching'
  | 'numerical'
  | 'calculated'
  | 'hotspot'
  | 'file_upload';

export interface Quiz {
  id: string;
  courseId: string;
  title: string;
  description?: string;
  instructions?: string;
  quizType: 'practice' | 'graded' | 'survey';
  points: number;
  timeLimit?: number; // minutes
  allowedAttempts: number;
  shuffleQuestions: boolean;
  shuffleAnswers: boolean;
  dueDate?: Date;
  availableFrom?: Date;
  availableUntil?: Date;
  showCorrectAnswers: boolean;
  showCorrectAnswersAt?: Date;
  oneQuestionAtATime: boolean;
  cantGoBack: boolean;
  questions: Question[];
  published: boolean;
}

export interface Question {
  id: string;
  type: QuestionType;
  text: string;
  points: number;
  answers?: Answer[];
  correctFeedback?: string;
  incorrectFeedback?: string;
  neutralFeedback?: string;
}

export interface Answer {
  id: string;
  text: string;
  correct: boolean;
  weight?: number; // For partial credit
}

export interface QuizSubmission {
  id: string;
  quizId: string;
  userId: string;
  attemptNumber: number;
  startedAt: Date;
  submittedAt?: Date;
  score?: number;
  answers: QuestionAnswer[];
  timeSpent: number; // seconds
}

export interface QuestionAnswer {
  questionId: string;
  answerId?: string; // For multiple choice
  text?: string; // For essay/fill-in
  correct?: boolean;
  points: number;
}

// ============================================================================
// Discussion Types
// ============================================================================

export type DiscussionType = 'threaded' | 'flat';

export interface Discussion {
  id: string;
  courseId: string;
  title: string;
  message: string;
  discussionType: DiscussionType;
  allowRating: boolean;
  requireInitialPost: boolean;
  graded: boolean;
  points?: number;
  dueDate?: Date;
  locked: boolean;
  published: boolean;
}

export interface DiscussionPost {
  id: string;
  discussionId: string;
  userId: string;
  parentId?: string; // For replies
  message: string;
  postedAt: Date;
  updatedAt?: Date;
  likes: number;
  replies: DiscussionPost[];
}

// ============================================================================
// Grading Types
// ============================================================================

export interface Gradebook {
  courseId: string;
  students: StudentGrade[];
  assignments: Assignment[];
  gradingScheme: GradingScheme;
  categories?: GradeCategory[];
}

export interface GradeCategory {
  id: string;
  name: string;
  weight: number; // Percentage (0-100)
  dropLowest?: number;
}

export interface StudentGrade {
  userId: string;
  currentGrade: number;
  currentLetter?: string;
  finalGrade?: number;
  finalLetter?: string;
  assignmentGrades: AssignmentGrade[];
}

export interface AssignmentGrade {
  assignmentId: string;
  score?: number;
  maxScore: number;
  grade?: string;
  submittedAt?: Date;
  gradedAt?: Date;
  late: boolean;
  missing: boolean;
  excused: boolean;
}

// ============================================================================
// Analytics Types
// ============================================================================

export interface CourseAnalytics {
  courseId: string;
  enrollmentCount: number;
  activeStudents: number;
  averageGrade: number;
  completionRate: number;
  engagementMetrics: EngagementMetrics;
  performanceMetrics: PerformanceMetrics;
}

export interface EngagementMetrics {
  avgPageViews: number;
  avgTimeOnTask: number; // minutes
  discussionParticipation: number; // percentage
  assignmentSubmissionRate: number; // percentage
}

export interface PerformanceMetrics {
  averageScore: number;
  medianScore: number;
  highestScore: number;
  lowestScore: number;
  standardDeviation: number;
  gradeDistribution: Record<string, number>;
}

export interface StudentAnalytics {
  userId: string;
  courseId: string;
  currentGrade: number;
  predictedGrade?: number;
  riskLevel: 'low' | 'medium' | 'high';
  engagementScore: number; // 0-100
  lastActivity: Date;
  pageViews: number;
  timeOnTask: number; // minutes
  assignmentsCompleted: number;
  assignmentsMissing: number;
  discussionPosts: number;
}

// ============================================================================
// Notification Types
// ============================================================================

export type NotificationChannel = 'email' | 'sms' | 'push' | 'in-app';
export type NotificationType =
  | 'assignment_created'
  | 'assignment_due_soon'
  | 'assignment_graded'
  | 'discussion_post'
  | 'announcement'
  | 'course_updated';

export interface Notification {
  id: string;
  userId: string;
  type: NotificationType;
  title: string;
  message: string;
  channel: NotificationChannel;
  read: boolean;
  createdAt: Date;
  actionUrl?: string;
}

export interface NotificationPreferences {
  userId: string;
  channels: NotificationChannel[];
  frequency: 'immediate' | 'daily' | 'weekly';
  quietHours?: {
    start: string; // HH:mm
    end: string;
  };
  preferences: Record<NotificationType, boolean>;
}

// ============================================================================
// LTI Types
// ============================================================================

export interface LTITool {
  id: string;
  name: string;
  consumerKey: string;
  sharedSecret: string;
  launchUrl: string;
  icon?: string;
  description?: string;
  customFields?: Record<string, string>;
  privacyLevel: 'anonymous' | 'name_only' | 'public';
}

export interface LTILaunchRequest {
  toolId: string;
  userId: string;
  courseId?: string;
  assignmentId?: string;
  resourceLinkId: string;
  returnUrl: string;
}

// ============================================================================
// Configuration Types
// ============================================================================

export interface LMSConfig {
  apiEndpoint: string;
  apiKey: string;
  apiSecret?: string;
  timeout?: number;
  retries?: number;
  locale?: string;
  timezone?: string;
}

// ============================================================================
// API Response Types
// ============================================================================

export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: ApiError;
  pagination?: Pagination;
}

export interface ApiError {
  code: string;
  message: string;
  details?: Record<string, any>;
}

export interface Pagination {
  page: number;
  perPage: number;
  totalPages: number;
  totalCount: number;
}

// ============================================================================
// Export all types
// ============================================================================

export default {
  // Re-export everything for convenience
};
