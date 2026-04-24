/**
 * WIA Cognitive AAC Standard
 * TypeScript Type Definitions
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// ============================================================================
// Basic Types
// ============================================================================

export type Timestamp = string;
export type UUID = string;
export type LanguageCode = string;

// ============================================================================
// User Cognitive Profile Types
// ============================================================================

export interface CognitiveProfile {
  profileId: string;
  userId: string;
  assessmentDate: Timestamp;
  cognitiveAreas: CognitiveAreaAssessment[];
  overallLevel: CognitiveLevel;
  recommendations: CognitiveRecommendation[];
  adaptations: CognitiveAdaptation[];
  lastUpdated: Timestamp;
}

export interface CognitiveAreaAssessment {
  area: CognitiveArea;
  level: CognitiveLevel;
  score: number;
  standardScore?: number;
  percentile?: number;
  strengths: string[];
  challenges: string[];
  notes?: string;
}

export enum CognitiveArea {
  ATTENTION = 'attention',
  MEMORY_WORKING = 'memory_working',
  MEMORY_LONG_TERM = 'memory_long_term',
  MEMORY_VISUAL = 'memory_visual',
  LANGUAGE_RECEPTIVE = 'language_receptive',
  LANGUAGE_EXPRESSIVE = 'language_expressive',
  EXECUTIVE_FUNCTION = 'executive_function',
  PROCESSING_SPEED = 'processing_speed',
  VISUAL_SPATIAL = 'visual_spatial',
  REASONING = 'reasoning',
  SOCIAL_COGNITION = 'social_cognition',
}

export enum CognitiveLevel {
  PROFOUND = 'profound',
  SEVERE = 'severe',
  MODERATE = 'moderate',
  MILD = 'mild',
  BORDERLINE = 'borderline',
  AVERAGE = 'average',
  ABOVE_AVERAGE = 'above_average',
}

export interface CognitiveRecommendation {
  recommendationId: string;
  area: CognitiveArea;
  priority: Priority;
  description: string;
  strategies: string[];
  tools: string[];
  resources: Resource[];
}

export enum Priority {
  CRITICAL = 'critical',
  HIGH = 'high',
  MEDIUM = 'medium',
  LOW = 'low',
}

export interface Resource {
  resourceId: string;
  type: ResourceType;
  title: string;
  url?: string;
  description?: string;
}

export enum ResourceType {
  DOCUMENT = 'document',
  VIDEO = 'video',
  TOOL = 'tool',
  TRAINING = 'training',
  EXTERNAL = 'external',
}

// ============================================================================
// Cognitive Adaptation Types
// ============================================================================

export interface CognitiveAdaptation {
  adaptationId: string;
  name: string;
  targetAreas: CognitiveArea[];
  settings: AdaptationSettings;
  triggers: AdaptationTrigger[];
  active: boolean;
}

export interface AdaptationSettings {
  complexity: ComplexityLevel;
  pacing: PacingSettings;
  support: SupportSettings;
  presentation: PresentationSettings;
  feedback: FeedbackSettings;
}

export enum ComplexityLevel {
  MINIMAL = 'minimal',
  SIMPLE = 'simple',
  MODERATE = 'moderate',
  STANDARD = 'standard',
  ADVANCED = 'advanced',
}

export interface PacingSettings {
  autoAdvance: boolean;
  advanceDelay: number;
  userControlled: boolean;
  breakReminders: boolean;
  breakInterval: number;
  timeoutExtension: number;
}

export interface SupportSettings {
  visualCues: boolean;
  auditoryCues: boolean;
  tactileCues: boolean;
  contextHelp: boolean;
  stepByStep: boolean;
  errorPrevention: boolean;
  confirmationRequired: boolean;
}

export interface PresentationSettings {
  fontSize: FontSize;
  fontFamily: string;
  contrast: ContrastLevel;
  colorScheme: ColorScheme;
  iconStyle: IconStyle;
  animation: AnimationLevel;
  layoutDensity: LayoutDensity;
}

export enum FontSize {
  EXTRA_SMALL = 'xs',
  SMALL = 'sm',
  MEDIUM = 'md',
  LARGE = 'lg',
  EXTRA_LARGE = 'xl',
  HUGE = 'xxl',
}

export enum ContrastLevel {
  LOW = 'low',
  NORMAL = 'normal',
  HIGH = 'high',
  MAXIMUM = 'maximum',
}

export enum ColorScheme {
  LIGHT = 'light',
  DARK = 'dark',
  HIGH_CONTRAST = 'high_contrast',
  SEPIA = 'sepia',
  CUSTOM = 'custom',
}

export enum IconStyle {
  SIMPLE = 'simple',
  DETAILED = 'detailed',
  PHOTOGRAPHIC = 'photographic',
  SYMBOLIC = 'symbolic',
}

export enum AnimationLevel {
  NONE = 'none',
  MINIMAL = 'minimal',
  STANDARD = 'standard',
  FULL = 'full',
}

export enum LayoutDensity {
  SPARSE = 'sparse',
  NORMAL = 'normal',
  COMPACT = 'compact',
}

export interface FeedbackSettings {
  immediateFeedback: boolean;
  feedbackType: FeedbackType[];
  encouragement: boolean;
  errorExplanation: boolean;
  progressTracking: boolean;
  celebrationEffects: boolean;
}

export enum FeedbackType {
  VISUAL = 'visual',
  AUDITORY = 'auditory',
  HAPTIC = 'haptic',
  VERBAL = 'verbal',
}

export interface AdaptationTrigger {
  triggerId: string;
  type: TriggerType;
  condition: TriggerCondition;
  action: TriggerAction;
}

export enum TriggerType {
  PERFORMANCE = 'performance',
  TIME = 'time',
  FATIGUE = 'fatigue',
  ERROR_RATE = 'error_rate',
  USER_REQUEST = 'user_request',
  CONTEXT = 'context',
}

export interface TriggerCondition {
  metric: string;
  operator: ComparisonOperator;
  value: number | string;
  duration?: number;
}

export enum ComparisonOperator {
  EQUALS = 'eq',
  NOT_EQUALS = 'ne',
  GREATER = 'gt',
  LESS = 'lt',
  GREATER_EQUAL = 'gte',
  LESS_EQUAL = 'lte',
}

export interface TriggerAction {
  type: ActionType;
  parameters: Record<string, unknown>;
}

export enum ActionType {
  ADJUST_COMPLEXITY = 'adjust_complexity',
  OFFER_BREAK = 'offer_break',
  PROVIDE_SUPPORT = 'provide_support',
  CHANGE_MODALITY = 'change_modality',
  ALERT_CAREGIVER = 'alert_caregiver',
  LOG_EVENT = 'log_event',
}

// ============================================================================
// Communication Types
// ============================================================================

export interface CognitiveSymbol extends Symbol {
  cognitiveLoad: CognitiveLoadLevel;
  abstractionLevel: AbstractionLevel;
  associatedConcepts: string[];
  prerequisiteKnowledge: string[];
  scaffolding: ScaffoldingInfo[];
}

export interface Symbol {
  symbolId: string;
  type: SymbolType;
  label: string;
  description?: string;
  imageUrl?: string;
  audioUrl?: string;
  category: string;
  tags: string[];
}

export enum SymbolType {
  PICTOGRAM = 'pictogram',
  ICON = 'icon',
  TEXT = 'text',
  PHOTO = 'photo',
  COMBINATION = 'combination',
}

export enum CognitiveLoadLevel {
  VERY_LOW = 'very_low',
  LOW = 'low',
  MEDIUM = 'medium',
  HIGH = 'high',
  VERY_HIGH = 'very_high',
}

export enum AbstractionLevel {
  CONCRETE = 'concrete',
  SEMI_CONCRETE = 'semi_concrete',
  SEMI_ABSTRACT = 'semi_abstract',
  ABSTRACT = 'abstract',
}

export interface ScaffoldingInfo {
  level: number;
  description: string;
  supportType: SupportType;
  enabled: boolean;
}

export enum SupportType {
  VISUAL_HINT = 'visual_hint',
  AUDIO_PROMPT = 'audio_prompt',
  DEMONSTRATION = 'demonstration',
  PARTIAL_COMPLETION = 'partial_completion',
  CHOICE_REDUCTION = 'choice_reduction',
}

// ============================================================================
// Learning and Progress Types
// ============================================================================

export interface LearningProgress {
  userId: string;
  skillId: string;
  currentLevel: number;
  targetLevel: number;
  startDate: Timestamp;
  lastPractice: Timestamp;
  practiceCount: number;
  successRate: number;
  masteryStatus: MasteryStatus;
  history: ProgressEntry[];
}

export enum MasteryStatus {
  NOT_STARTED = 'not_started',
  EMERGING = 'emerging',
  DEVELOPING = 'developing',
  PROFICIENT = 'proficient',
  MASTERED = 'mastered',
  MAINTENANCE = 'maintenance',
}

export interface ProgressEntry {
  date: Timestamp;
  activity: string;
  performance: number;
  duration: number;
  supportUsed: SupportType[];
  notes?: string;
}

export interface LearningGoal {
  goalId: string;
  userId: string;
  title: string;
  description: string;
  targetSkills: string[];
  startDate: Timestamp;
  targetDate: Timestamp;
  status: GoalStatus;
  milestones: Milestone[];
  progress: number;
}

export enum GoalStatus {
  PLANNED = 'planned',
  IN_PROGRESS = 'in_progress',
  ACHIEVED = 'achieved',
  PAUSED = 'paused',
  MODIFIED = 'modified',
}

export interface Milestone {
  milestoneId: string;
  title: string;
  criteria: string;
  targetDate: Timestamp;
  completedDate?: Timestamp;
  status: MilestoneStatus;
}

export enum MilestoneStatus {
  PENDING = 'pending',
  IN_PROGRESS = 'in_progress',
  COMPLETED = 'completed',
  SKIPPED = 'skipped',
}

// ============================================================================
// Session and Analytics Types
// ============================================================================

export interface CognitiveSession {
  sessionId: string;
  userId: string;
  startTime: Timestamp;
  endTime?: Timestamp;
  activities: ActivityLog[];
  cognitiveMetrics: CognitiveMetrics;
  adaptationsApplied: string[];
  notes?: string;
}

export interface ActivityLog {
  activityId: string;
  type: ActivityType;
  startTime: Timestamp;
  endTime: Timestamp;
  performance: PerformanceData;
  events: ActivityEvent[];
}

export enum ActivityType {
  COMMUNICATION = 'communication',
  LEARNING = 'learning',
  ASSESSMENT = 'assessment',
  PRACTICE = 'practice',
  FREE_EXPLORATION = 'free_exploration',
}

export interface PerformanceData {
  accuracy: number;
  speed: number;
  efficiency: number;
  independence: number;
  engagement: number;
}

export interface ActivityEvent {
  timestamp: Timestamp;
  type: string;
  data: Record<string, unknown>;
}

export interface CognitiveMetrics {
  attentionSpan: number;
  responseTime: number;
  errorRate: number;
  fatigueLevel: number;
  engagementLevel: number;
  frustrationLevel: number;
  supportUtilization: number;
}

// ============================================================================
// API Types
// ============================================================================

export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: ApiError;
  timestamp: Timestamp;
  requestId: string;
}

export interface ApiError {
  code: string;
  message: string;
  details?: Record<string, unknown>;
}

export interface PaginationParams {
  page: number;
  pageSize: number;
  sortBy?: string;
  sortOrder?: 'asc' | 'desc';
}

export interface PaginatedResponse<T> {
  items: T[];
  total: number;
  page: number;
  pageSize: number;
  totalPages: number;
}

// ============================================================================
// Event Types
// ============================================================================

export enum EventType {
  ADAPTATION_TRIGGERED = 'adaptation_triggered',
  PROGRESS_UPDATE = 'progress_update',
  GOAL_ACHIEVED = 'goal_achieved',
  FATIGUE_DETECTED = 'fatigue_detected',
  ERROR_PATTERN = 'error_pattern',
  SUPPORT_NEEDED = 'support_needed',
}

export interface BaseEvent {
  type: EventType;
  timestamp: Timestamp;
  sessionId?: string;
  userId: string;
}

export type EventHandler<T extends BaseEvent> = (event: T) => void | Promise<void>;
