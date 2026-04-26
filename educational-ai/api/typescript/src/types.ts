/**
 * WIA-EDU-005: Educational AI Standard - TypeScript Types
 * 弘익人間 (홍익인간) - Benefit All Humanity
 */

// ===== Core Types =====

export type LearningStyle = 'visual' | 'auditory' | 'kinesthetic' | 'reading';
export type Pace = 'fast' | 'medium' | 'slow';
export type AssignmentType = 'essay' | 'code' | 'math' | 'multiple-choice';
export type Difficulty = number; // 0-1

// ===== Student Profile =====

export interface StudentProfile {
  studentId: string;
  name: string;
  gradeLevel: string;
  learningStyle: LearningStyle;
  pace: Pace;
  knowledgeState: KnowledgeMap;
  masteryLevels: Record<string, number>; // topic -> mastery (0-1)
  engagementMetrics: EngagementData;
  preferences: StudentPreferences;
}

export interface KnowledgeMap {
  concepts: ConceptNode[];
  relationships: ConceptRelationship[];
}

export interface ConceptNode {
  id: string;
  name: string;
  mastery: number; // 0-1
  lastPracticed: Date;
}

export interface ConceptRelationship {
  prerequisiteId: string;
  dependentId: string;
  strength: number; // 0-1
}

export interface EngagementData {
  totalTimeMinutes: number;
  questionsAsked: number;
  problemsSolved: number;
  averageSessionTime: number;
  lastActive: Date;
}

export interface StudentPreferences {
  language: string;
  timezone: string;
  notificationsEnabled: boolean;
  darkMode: boolean;
}

// ===== AI Tutor =====

export interface TutorConfig {
  subject: string;
  gradeLevel: string;
  personality: TutorPersonality;
  language: string;
}

export interface TutorPersonality {
  patience: 'low' | 'medium' | 'high';
  encouragement: 'minimal' | 'moderate' | 'frequent';
  hintingStrategy: 'direct' | 'socratic' | 'guided-discovery';
  formality: 'casual' | 'professional';
}

export interface TutorSession {
  sessionId: string;
  studentId: string;
  startTime: Date;
  endTime?: Date;
  topic: string;
  messages: Message[];
  learningGoals: string[];
}

export interface Message {
  id: string;
  role: 'student' | 'tutor' | 'system';
  content: string;
  timestamp: Date;
  metadata?: Record<string, any>;
}

export interface TutorResponse {
  content: string;
  type: 'explanation' | 'question' | 'hint' | 'encouragement' | 'correction';
  confidence: number; // 0-1
  suggestedActions?: string[];
}

// ===== Automated Grading =====

export interface GradingRequest {
  assignmentId: string;
  studentId: string;
  submissionType: AssignmentType;
  content: string;
  rubric?: Rubric;
  metadata?: Record<string, any>;
}

export interface Rubric {
  criteria: RubricCriterion[];
  totalPoints: number;
}

export interface RubricCriterion {
  name: string;
  description: string;
  maxPoints: number;
  weight: number; // 0-1
}

export interface GradingResponse {
  submissionId: string;
  score: number;
  maxScore: number;
  percentage: number;
  feedback: Feedback[];
  rubricScores?: Record<string, number>;
  gradedAt: Date;
  gradingTime: number; // milliseconds
}

export interface Feedback {
  type: 'strength' | 'improvement' | 'error' | 'suggestion';
  criterion?: string;
  message: string;
  location?: {
    line?: number;
    start?: number;
    end?: number;
  };
}

// ===== Personalization =====

export interface RecommendationRequest {
  studentId: string;
  currentTopic?: string;
  context: LearningContext;
  count?: number;
}

export interface LearningContext {
  timeAvailable: number; // minutes
  objectives: string[];
  restrictions?: string[];
}

export interface RecommendationResponse {
  nextTopics: Topic[];
  resources: LearningResource[];
  estimatedTime: number; // minutes
  reasoning: string;
}

export interface Topic {
  id: string;
  name: string;
  description: string;
  difficulty: Difficulty;
  prerequisites: string[];
  estimatedTime: number; // minutes
}

export interface LearningResource {
  id: string;
  type: 'video' | 'article' | 'interactive' | 'quiz' | 'practice';
  title: string;
  url: string;
  duration: number; // minutes
  difficulty: Difficulty;
  rating?: number; // 0-5
}

// ===== Content Generation =====

export interface ContentGenerationRequest {
  type: 'problem' | 'quiz' | 'explanation' | 'example';
  topic: string;
  difficulty: Difficulty;
  count?: number;
  constraints?: ContentConstraints;
}

export interface ContentConstraints {
  maxLength?: number;
  includeHints?: boolean;
  includeExplanations?: boolean;
  format?: string;
}

export interface GeneratedContent {
  items: ContentItem[];
  metadata: {
    generatedAt: Date;
    model: string;
    quality: number; // 0-1
  };
}

export interface ContentItem {
  id: string;
  content: string;
  type: string;
  difficulty: Difficulty;
  solution?: string;
  explanation?: string;
  hints?: string[];
}

// ===== Learning Analytics =====

export interface AnalyticsQuery {
  studentId?: string;
  classId?: string;
  timeRange: DateRange;
  metrics: Metric[];
}

export interface DateRange {
  start: Date;
  end: Date;
}

export type Metric = 
  | 'mastery'
  | 'engagement'
  | 'time-on-task'
  | 'completion-rate'
  | 'performance'
  | 'progress-rate';

export interface AnalyticsResponse {
  summary: MetricSummary;
  trends: TrendData[];
  predictions: Prediction[];
  recommendations: Intervention[];
}

export interface MetricSummary {
  metric: Metric;
  value: number;
  change: number; // percentage change
  percentile?: number;
}

export interface TrendData {
  metric: Metric;
  dataPoints: DataPoint[];
  trend: 'increasing' | 'decreasing' | 'stable';
}

export interface DataPoint {
  timestamp: Date;
  value: number;
}

export interface Prediction {
  type: 'success' | 'at-risk' | 'mastery-time';
  confidence: number; // 0-1
  value: any;
  reasoning: string;
}

export interface Intervention {
  type: 'content' | 'support' | 'pacing' | 'notification';
  priority: 'low' | 'medium' | 'high' | 'urgent';
  description: string;
  action: string;
}

// ===== Configuration =====

export interface EducationalAIConfig {
  apiKey: string;
  baseUrl?: string;
  timeout?: number;
  retries?: number;
  debug?: boolean;
}

// ===== API Response =====

export interface APIResponse<T> {
  success: boolean;
  data?: T;
  error?: APIError;
  metadata: {
    requestId: string;
    timestamp: Date;
    processingTime: number;
  };
}

export interface APIError {
  code: string;
  message: string;
  details?: Record<string, any>;
}
