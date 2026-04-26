/**
 * WIA-EDU-020: Content AI Standard - TypeScript Types
 * 弘益人間 (홍익인간) - Benefit All Humanity
 */

// ===== Core Types =====

export type ContentType = 'lesson' | 'problem' | 'assessment' | 'explanation' | 'video' | 'interactive';
export type Subject = 'mathematics' | 'science' | 'english' | 'social-studies' | 'computer-science' | string;
export type LearningStyle = 'visual' | 'auditory' | 'kinesthetic' | 'reading';
export type Pace = 'slow' | 'medium' | 'fast' | 'accelerated';
export type ContentFormat = 'text' | 'video' | 'audio' | 'interactive' | 'game';

// ===== Content Generation =====

export interface ContentGenerationRequest {
  contentType: ContentType;
  subject: Subject;
  topic: string;
  gradeLevel: string;
  objectives: string[];
  constraints?: ContentConstraints;
  context?: Record<string, any>;
}

export interface ContentConstraints {
  duration?: number;        // minutes
  difficulty?: number;      // 0-1
  length?: number;          // words/items
  format?: string;
  includeAssessment?: boolean;
  differentiation?: boolean;
}

export interface GeneratedContent {
  id: string;
  title: string;
  type: ContentType;
  sections: ContentSection[];
  assessments?: Assessment[];
  resources?: Resource[];
  standards?: string[];
  metadata: ContentMetadata;
  generatedAt: Date;
}

export interface ContentSection {
  id: string;
  type: 'introduction' | 'instruction' | 'practice' | 'assessment' | 'closure';
  title: string;
  content: string;
  duration?: number;
  visuals?: Visual[];
  activities?: Activity[];
}

export interface ContentMetadata {
  difficulty: number;       // 0-1
  readingLevel: number;     // Lexile or grade level
  estimatedTime: number;    // minutes
  wordCount?: number;
  quality?: QualityMetrics;
}

// ===== Lesson Generation =====

export interface LessonGenerationRequest {
  subject: Subject;
  topic: string;
  gradeLevel: string;
  objectives: string[];
  duration?: number;
  includeAssessment?: boolean;
  differentiation?: boolean;
}

export interface Lesson extends GeneratedContent {
  type: 'lesson';
  teachingNotes?: string;
  materials?: string[];
  differentiationStrategies?: DifferentiationStrategy[];
}

export interface DifferentiationStrategy {
  level: 'advanced' | 'on-level' | 'struggling';
  adaptations: string[];
}

// ===== Problem Generation =====

export interface ProblemGenerationRequest {
  topic: string;
  difficulty: number;       // 0-1
  count: number;
  includeHints?: boolean;
  includeSolutions?: boolean;
  format?: 'multiple-choice' | 'short-answer' | 'essay' | 'interactive';
}

export interface Problem {
  id: string;
  question: string;
  hints?: string[];
  solution?: Solution;
  difficulty: number;
  tags: string[];
  metadata?: ProblemMetadata;
}

export interface Solution {
  answer: string;
  steps?: string[];
  explanation?: string;
  workingOut?: string;
}

export interface ProblemMetadata {
  bloomLevel?: string;
  cognitiveDepth?: number;
  estimatedTime?: number;
}

// ===== Assessment =====

export interface AssessmentGenerationRequest {
  topic: string;
  questionTypes: QuestionType[];
  questionCount: number;
  difficulty: number;
  standards?: string[];
  includeRubric?: boolean;
}

export type QuestionType = 'multiple-choice' | 'short-answer' | 'essay' | 'true-false' | 'matching';

export interface Assessment {
  id: string;
  title: string;
  questions: Question[];
  rubric?: Rubric;
  estimatedTime: number;
  totalPoints: number;
}

export interface Question {
  id: string;
  type: QuestionType;
  question: string;
  options?: string[];
  correctAnswer?: string | string[];
  points: number;
  feedback?: string;
}

export interface Rubric {
  criteria: RubricCriterion[];
  totalPoints: number;
}

export interface RubricCriterion {
  name: string;
  description: string;
  maxPoints: number;
  weight: number;
  levels?: RubricLevel[];
}

export interface RubricLevel {
  score: number;
  description: string;
}

// ===== Multi-Modal Content =====

export interface VideoGenerationRequest {
  topic: string;
  script?: string;
  duration: number;         // seconds
  style: 'animated' | 'slideshow' | 'talking-head' | 'screencast';
  voiceConfig?: VoiceConfig;
  visualConfig?: VisualConfig;
  captions?: boolean;
  chapters?: boolean;
}

export interface VoiceConfig {
  gender: 'male' | 'female' | 'neutral';
  accent: string;           // e.g., 'en-US', 'en-GB'
  speed: number;            // 0.5-2.0
  language: string;
}

export interface VisualConfig {
  resolution: '720p' | '1080p' | '4k';
  fps: 24 | 30 | 60;
  theme: string;
  backgroundColor?: string;
}

export interface Video {
  id: string;
  videoUrl: string;
  thumbnailUrl: string;
  captionsUrl?: string;
  transcript: string;
  duration: number;
  metadata: VideoMetadata;
}

export interface VideoMetadata {
  resolution: string;
  size: number;             // bytes
  codec: string;
  format: string;
}

export interface InteractiveContentRequest {
  type: 'simulation' | 'game' | 'quiz' | 'drag-drop' | 'virtual-lab';
  topic: string;
  learningObjectives: string[];
  difficulty: number;
  config: InteractiveConfig;
}

export interface InteractiveConfig {
  mechanics: string[];
  feedback: FeedbackConfig;
  scoring: ScoringConfig;
  accessibility: AccessibilityConfig;
}

export interface FeedbackConfig {
  immediate: boolean;
  detailed: boolean;
  hints: boolean;
}

export interface ScoringConfig {
  pointsPerCorrect: number;
  penaltyForIncorrect: number;
  bonusForSpeed?: boolean;
}

export interface AccessibilityConfig {
  keyboardNavigation: boolean;
  screenReader: boolean;
  highContrast: boolean;
  audioDescriptions: boolean;
}

// ===== Personalization =====

export interface LearnerProfile {
  id: string;
  demographics: Demographics;
  learningCharacteristics: LearningCharacteristics;
  preferences: LearnerPreferences;
  performance: PerformanceData;
}

export interface Demographics {
  ageRange: string;
  gradeLevel: string;
  language: string;
  timezone: string;
}

export interface LearningCharacteristics {
  readingLevel: number;
  learningStyle: LearningStyle[];
  pace: Pace;
  strengths: string[];
  challenges: string[];
}

export interface LearnerPreferences {
  interests: string[];
  contentFormats: ContentFormat[];
  accessibility: AccessibilityNeeds;
}

export interface AccessibilityNeeds {
  visualImpairment?: boolean;
  hearingImpairment?: boolean;
  motorImpairment?: boolean;
  cognitiveNeeds?: string[];
  preferredFontSize?: number;
  screenReader?: boolean;
}

export interface PerformanceData {
  masteryLevels: Record<string, number>;
  engagementMetrics: EngagementMetrics;
  learningHistory: LearningEvent[];
}

export interface EngagementMetrics {
  totalTimeMinutes: number;
  questionsAsked: number;
  problemsSolved: number;
  averageSessionTime: number;
  lastActive: Date;
}

export interface LearningEvent {
  contentId: string;
  timestamp: Date;
  action: 'viewed' | 'completed' | 'practiced' | 'assessed';
  performance?: number;
}

export interface PersonalizationRequest {
  contentId: string;
  learnerProfile: LearnerProfile;
  adaptations: AdaptationOptions;
}

export interface AdaptationOptions {
  readingLevel?: boolean;
  learningStyle?: boolean;
  pace?: boolean;
  interests?: boolean;
  language?: boolean;
}

export interface PersonalizationResponse {
  personalizedContentId: string;
  content: GeneratedContent;
  adaptationLog: AdaptationRecord[];
  recommendedNext: Recommendation[];
}

export interface AdaptationRecord {
  type: string;
  from: any;
  to: any;
  changes: string[];
}

export interface Recommendation {
  contentId: string;
  title: string;
  type: ContentType;
  relevance: number;
  estimatedTime: number;
  difficulty: number;
  reason: string;
}

// ===== Translation =====

export interface TranslationRequest {
  contentId: string;
  sourceLanguage: string;
  targetLanguages: string[];
  culturalAdaptation?: boolean;
  preserveFormatting?: boolean;
  glossary?: Record<string, Record<string, string>>;
}

export interface TranslationResponse {
  translations: LanguageVersion[];
  quality: TranslationQuality[];
  warnings?: string[];
}

export interface LanguageVersion {
  language: string;
  contentId: string;
  content: GeneratedContent;
  culturalAdaptations: CulturalAdaptation[];
  reviewStatus: 'auto' | 'reviewed' | 'native';
}

export interface CulturalAdaptation {
  type: string;
  original: string;
  adapted: string;
  reason?: string;
}

export interface TranslationQuality {
  language: string;
  bleuScore: number;
  reviewStatus: string;
  confidence: number;
}

// ===== Analytics =====

export interface ContentAnalyticsRequest {
  contentId: string;
  timeRange: DateRange | string;
  metrics?: string[];
}

export interface DateRange {
  start: Date;
  end: Date;
}

export interface ContentAnalytics {
  contentId: string;
  timeRange: DateRange;
  engagement: EngagementAnalytics;
  effectiveness: EffectivenessMetrics;
  usage: UsageMetrics;
  quality?: QualityMetrics;
}

export interface EngagementAnalytics {
  views: number;
  completionRate: number;
  averageTimeOnContent: number;
  interactionRate: number;
  returnRate: number;
}

export interface EffectivenessMetrics {
  averageLearningGain: number;
  retentionRate: number;
  transferRate?: number;
  satisfactionScore: number;
}

export interface UsageMetrics {
  totalStudents: number;
  activeStudents: number;
  completedStudents: number;
  averageAttempts: number;
}

export interface QualityMetrics {
  accuracy: number;         // 0-1, factual correctness
  clarity: number;          // 0-1, readability
  completeness: number;     // 0-1, coverage
  pedagogical: number;      // 0-1, teaching effectiveness
  accessibility: number;    // 0-1, WCAG compliance
  engagement: number;       // 0-1, predicted engagement
  overall: number;          // 0-1, weighted average
}

// ===== A/B Testing =====

export interface ABTestConfig {
  name: string;
  variants: ContentVariant[];
  metrics: string[];
  sampleSize: number;
  duration: number;         // days
  successCriteria?: SuccessCriteria;
}

export interface ContentVariant {
  name: string;
  contentId: string;
  weight: number;           // traffic allocation 0-1
}

export interface SuccessCriteria {
  metric: string;
  threshold: number;
  confidence: number;       // statistical confidence 0-1
}

export interface ABTestResult {
  testId: string;
  status: 'running' | 'completed' | 'stopped';
  startDate: Date;
  endDate?: Date;
  results: VariantResult[];
  winner?: string;
  statisticalSignificance?: number;
}

export interface VariantResult {
  variantName: string;
  sampleSize: number;
  metrics: Record<string, number>;
  conversionRate?: number;
}

// ===== Common Types =====

export interface Visual {
  id: string;
  type: 'image' | 'diagram' | 'chart' | 'video';
  url: string;
  altText: string;
  caption?: string;
}

export interface Activity {
  id: string;
  type: 'exercise' | 'discussion' | 'project' | 'game';
  title: string;
  instructions: string;
  duration?: number;
  materials?: string[];
}

export interface Resource {
  id: string;
  type: 'reading' | 'video' | 'website' | 'tool';
  title: string;
  url?: string;
  description?: string;
}

// ===== API Response Types =====

export interface APIResponse<T> {
  data: T;
  metadata?: ResponseMetadata;
}

export interface ResponseMetadata {
  requestId: string;
  timestamp: Date;
  processingTime: number;
  version: string;
}

export interface APIError {
  code: string;
  message: string;
  details?: Record<string, any>;
  timestamp: Date;
  requestId?: string;
}

// ===== Configuration =====

export interface ContentAIConfig {
  apiKey: string;
  environment?: 'production' | 'sandbox';
  timeout?: number;
  retries?: number;
  baseUrl?: string;
  debug?: boolean;
}
