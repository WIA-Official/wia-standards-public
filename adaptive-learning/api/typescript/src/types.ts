/**
 * WIA-EDU-003 Adaptive Learning Standard - TypeScript Type Definitions
 *
 * @packageDocumentation
 * Philosophy: 弘益人間 (홍익인간) - Benefit All Humanity
 */

/**
 * Learning style categories based on VARK model
 */
export enum LearningStyle {
  VISUAL = 'visual',
  AUDITORY = 'auditory',
  READING_WRITING = 'reading_writing',
  KINESTHETIC = 'kinesthetic'
}

/**
 * Content format types
 */
export enum ContentFormat {
  VIDEO = 'video',
  AUDIO = 'audio',
  TEXT = 'text',
  INTERACTIVE = 'interactive',
  SIMULATION = 'simulation',
  QUIZ = 'quiz',
  MULTIMODAL = 'multimodal'
}

/**
 * Mastery levels for topic progression
 */
export enum MasteryLevel {
  BASIC = 'basic',
  PROFICIENT = 'proficient',
  ADVANCED = 'advanced'
}

/**
 * Difficulty scale (1-10)
 */
export type DifficultyLevel = 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10;

/**
 * VARK learning style profile
 */
export interface VARKProfile {
  visual: number;
  auditory: number;
  reading_writing: number;
  kinesthetic: number;
}

/**
 * Learning style detection result
 */
export interface LearningStyleDetection {
  vark_profile: VARKProfile;
  dominant_style: LearningStyle;
  detection_method: 'questionnaire' | 'behavioral' | 'hybrid';
  confidence: number;
  last_assessed: Date;
}

/**
 * Subject proficiency information
 */
export interface SubjectProficiency {
  subject_id: string;
  proficiency: number; // 0-1
  topics_mastered: number;
  topics_in_progress: number;
  topics_locked: number;
  last_activity: Date;
}

/**
 * Learner goal with milestones
 */
export interface LearnerGoal {
  goal_id: string;
  type: 'certification' | 'skill_development' | 'career' | 'personal';
  description: string;
  target_date: Date;
  progress: number; // 0-1
  milestones: Milestone[];
}

/**
 * Goal milestone
 */
export interface Milestone {
  milestone_id: string;
  description: string;
  completed: boolean;
  completed_at?: Date;
  estimated_completion?: Date;
}

/**
 * Performance metrics for a learner
 */
export interface PerformanceMetrics {
  overall_accuracy: number; // 0-1
  questions_answered: number;
  correct_answers: number;
  current_streak: number;
  longest_streak: number;
  average_time_per_question_seconds: number;
  total_learning_time_minutes: number;
  last_7_days: {
    sessions: number;
    total_minutes: number;
    accuracy: number;
    topics_mastered: number;
  };
}

/**
 * Current difficulty state
 */
export interface DifficultyState {
  level: DifficultyLevel;
  scale: string;
  last_adjusted: Date;
  adjustment_reason: 'high_performance' | 'low_performance' | 'streak' | 'initial';
}

/**
 * Complete learner profile
 */
export interface LearnerProfile {
  learner_id: string;
  version: string;
  created_at: Date;
  updated_at: Date;

  demographics?: {
    age_range?: string;
    education_level?: string;
    primary_language?: string;
    timezone?: string;
  };

  learning_style: LearningStyleDetection;
  knowledge_state: {
    overall_level: 'beginner' | 'intermediate' | 'advanced' | 'expert';
    subjects: SubjectProficiency[];
  };

  preferences: {
    preferred_content_formats: ContentFormat[];
    session_duration_minutes: number;
    study_times: string[];
    notification_preferences: {
      reminders: boolean;
      achievements: boolean;
      recommendations: boolean;
    };
  };

  goals: LearnerGoal[];
  performance_metrics: PerformanceMetrics;
  current_difficulty: DifficultyState;
}

/**
 * Content metadata for learning materials
 */
export interface ContentMetadata {
  content_id: string;
  title: string;
  type: ContentFormat;
  format: string;
  duration_seconds?: number;
  difficulty_level: DifficultyLevel;
  difficulty_scale: string;

  taxonomy: {
    subject: string;
    topic: string;
    subtopic?: string;
    learning_objectives: string[];
  };

  prerequisites: string[];
  suitable_learning_styles: LearningStyle[];
  engagement_score: number; // 0-1
  completion_rate: number; // 0-1
  average_mastery_improvement: number; // 0-1

  multilingual?: {
    available_languages: string[];
    default_language: string;
  };
}

/**
 * Content recommendation with reasoning
 */
export interface ContentRecommendation {
  content_id: string;
  title: string;
  type: ContentFormat;
  difficulty: DifficultyLevel;
  reason: 'matches_learning_style' | 'optimal_difficulty' | 'prerequisite' | 'goal_aligned' | 'collaborative_filtering';
  priority: number; // 0-1
  estimated_time_minutes: number;
}

/**
 * Assessment question response
 */
export interface AssessmentResponse {
  question_id: string;
  answer: string | number | boolean;
  correct: boolean;
  time_spent_seconds: number;
  hints_used: number;
  confidence?: 'low' | 'medium' | 'high';
}

/**
 * Assessment result
 */
export interface AssessmentResult {
  assessment_id: string;
  difficulty_adjusted: boolean;
  new_difficulty: DifficultyLevel;
  mastery_progress: number; // 0-1
  feedback?: string;
}

/**
 * Topic mastery criteria
 */
export interface MasteryCriteria {
  level: MasteryLevel;
  min_accuracy: number; // 0-1
  min_questions: number;
  retention_check_1_day?: number; // 0-1
  retention_check_1_week?: number; // 0-1
  retention_check_1_month?: number; // 0-1
}

/**
 * Topic with prerequisites and mastery info
 */
export interface Topic {
  topic_id: string;
  title: string;
  description: string;
  prerequisites: {
    topic_id: string;
    required_mastery_level: MasteryLevel;
  }[];
  unlocks: string[];
  mastery_criteria: MasteryCriteria;
}

/**
 * Mastery status for a topic
 */
export interface MasteryStatus {
  topic_id: string;
  mastered: boolean;
  accuracy: number; // 0-1
  questions_completed: number;
  retention_verified: boolean;
  certified_date?: Date;
}

/**
 * Learning analytics for insights
 */
export interface LearningAnalytics {
  learner_id: string;
  timeframe: string;

  learning_velocity: {
    topics_per_week: number;
    trend: 'increasing' | 'stable' | 'decreasing';
  };

  engagement: {
    total_sessions: number;
    total_minutes: number;
    average_session_minutes: number;
    peak_hours: number[];
  };

  performance: {
    overall_accuracy: number;
    by_subject: {
      [subject: string]: number;
    };
  };

  predictions: {
    goal_completion_probability: number;
    estimated_completion_date: Date;
    at_risk_topics: string[];
  };
}

/**
 * Adaptive learning configuration
 */
export interface AdaptiveLearningConfig {
  api_base_url: string;
  api_key: string;
  version: string;
  timeout_ms?: number;
  retry_attempts?: number;
}

/**
 * API response wrapper
 */
export interface APIResponse<T> {
  success: boolean;
  data?: T;
  error?: {
    code: string;
    message: string;
    details?: any;
  };
  metadata?: {
    request_id: string;
    timestamp: Date;
  };
}

/**
 * Pagination parameters
 */
export interface PaginationParams {
  page?: number;
  limit?: number;
  offset?: number;
}

/**
 * Paginated response
 */
export interface PaginatedResponse<T> {
  items: T[];
  total: number;
  page: number;
  limit: number;
  has_more: boolean;
}
