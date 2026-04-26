/**
 * WIA-EDU-002: E-Learning Standard Types
 *
 * 弘益人間 (Hongik Ingan) - Benefit All Humanity
 */

export enum CourseType {
  SelfPaced = 'self-paced',
  InstructorLed = 'instructor-led',
  Blended = 'blended'
}

export enum CourseCategory {
  Technology = 'technology',
  Business = 'business',
  Design = 'design',
  Science = 'science',
  Language = 'language',
  Health = 'health',
  Arts = 'arts'
}

export enum CourseDifficulty {
  Beginner = 'beginner',
  Intermediate = 'intermediate',
  Advanced = 'advanced',
  Expert = 'expert'
}

export enum ContentFormat {
  Video = 'video',
  Audio = 'audio',
  Document = 'document',
  Quiz = 'quiz',
  Assignment = 'assignment',
  Interactive = 'interactive',
  SCORM = 'scorm',
  xAPI = 'xapi'
}

export enum EnrollmentStatus {
  Active = 'active',
  Completed = 'completed',
  Dropped = 'dropped',
  Suspended = 'suspended'
}

export enum QuizQuestionType {
  MultipleChoice = 'multiple-choice',
  TrueFalse = 'true-false',
  FillInBlank = 'fill-in-blank',
  Essay = 'essay',
  Code = 'code'
}

export interface Course {
  id: string;
  title: string;
  description: string;
  category: CourseCategory;
  difficulty: CourseDifficulty;
  type: CourseType;
  duration: string;
  price: number;
  currency: string;
  instructor: Instructor;
  syllabus: Lesson[];
  prerequisites?: string[];
  tags?: string[];
  thumbnail?: string;
  rating?: number;
  enrollmentCount?: number;
  createdAt: Date;
  updatedAt: Date;
}

export interface Instructor {
  id: string;
  name: string;
  email: string;
  bio?: string;
  avatar?: string;
  expertise?: string[];
}

export interface Student {
  id: string;
  name: string;
  email: string;
  avatar?: string;
  enrolledCourses?: Enrollment[];
  certificates?: Certificate[];
  createdAt: Date;
}

export interface Lesson {
  id: string;
  courseId: string;
  title: string;
  description?: string;
  type: ContentFormat;
  content: LessonContent;
  order: number;
  duration?: number;
  isRequired: boolean;
}

export interface LessonContent {
  url?: string;
  text?: string;
  html?: string;
  scormPackage?: string;
  quiz?: Quiz;
}

export interface Quiz {
  id: string;
  title: string;
  description?: string;
  questions: Question[];
  passingScore: number;
  timeLimit?: number;
  maxAttempts?: number;
}

export interface Question {
  id: string;
  type: QuizQuestionType;
  text: string;
  options?: string[];
  correctAnswer: string | string[];
  points: number;
  explanation?: string;
}

export interface Enrollment {
  id: string;
  studentId: string;
  courseId: string;
  status: EnrollmentStatus;
  progress: Progress;
  enrolledAt: Date;
  completedAt?: Date;
}

export interface Progress {
  percentage: number;
  lessonsCompleted: number;
  totalLessons: number;
  quizScores: QuizScore[];
  timeSpent: number;
  lastAccessedAt?: Date;
}

export interface QuizScore {
  quizId: string;
  score: number;
  maxScore: number;
  percentage: number;
  attempts: number;
  completedAt: Date;
}

export interface Certificate {
  id: string;
  studentId: string;
  courseId: string;
  studentName: string;
  courseTitle: string;
  issuedAt: Date;
  expiresAt?: Date;
  blockchainHash?: string;
  verificationUrl: string;
  pdfUrl?: string;
}

export interface LiveSession {
  id: string;
  courseId: string;
  title: string;
  description?: string;
  instructorId: string;
  scheduledTime: Date;
  duration: number;
  maxParticipants: number;
  participants: string[];
  recordingUrl?: string;
  joinUrl: string;
  features: SessionFeatures;
}

export interface SessionFeatures {
  camera: boolean;
  microphone: boolean;
  screenShare: boolean;
  whiteboard: boolean;
  chat: boolean;
  breakoutRooms: boolean;
  recording: boolean;
}

export interface Analytics {
  studentId?: string;
  courseId?: string;
  metrics: {
    engagement: number;
    quizScores: number[];
    timeSpent: number;
    completionRate: number;
    dropOffPoints?: string[];
  };
  insights?: string[];
}

export interface xAPIStatement {
  actor: {
    mbox: string;
    name: string;
  };
  verb: {
    id: string;
    display: { [key: string]: string };
  };
  object: {
    id: string;
    definition?: {
      name?: { [key: string]: string };
      description?: { [key: string]: string };
    };
  };
  result?: {
    score?: {
      scaled: number;
      raw?: number;
      min?: number;
      max?: number;
    };
    completion?: boolean;
    success?: boolean;
    duration?: string;
  };
  timestamp?: Date;
}

export interface ELearningConfig {
  apiKey: string;
  baseUrl?: string;
  environment?: 'development' | 'staging' | 'production';
  timeout?: number;
}
