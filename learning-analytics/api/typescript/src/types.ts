/**
 * WIA-EDU-004: Learning Analytics Standard - TypeScript Types
 * 
 * @category Types
 * @module types
 */

/**
 * Configuration options for LearningAnalytics client
 */
export interface LearningAnalyticsConfig {
  /** Institution unique identifier */
  institutionId: string;
  
  /** API endpoint base URL */
  apiEndpoint?: string;
  
  /** API authentication key */
  apiKey?: string;
  
  /** Privacy mode: 'standard' | 'strict' | 'minimal' */
  privacyMode?: 'standard' | 'strict' | 'minimal';
  
  /** Enable predictive analytics features */
  enablePredictive?: boolean;
  
  /** Data retention period in years */
  dataRetention?: number;
  
  /** Request timeout in milliseconds */
  timeout?: number;
}

/**
 * Student identifier and metadata
 */
export interface Student {
  /** Unique student identifier */
  id: string;
  
  /** Student email address (optional) */
  email?: string;
  
  /** Student enrollment status */
  enrollmentStatus?: 'active' | 'inactive' | 'graduated' | 'withdrawn';
  
  /** Additional metadata */
  metadata?: Record<string, any>;
}

/**
 * Course information
 */
export interface Course {
  /** Unique course identifier */
  id: string;
  
  /** Course name/title */
  name: string;
  
  /** Course code (e.g., 'CS-101') */
  code?: string;
  
  /** Academic term */
  term?: string;
  
  /** Instructor ID(s) */
  instructors?: string[];
}

/**
 * Assessment result
 */
export interface Assessment {
  /** Assessment type */
  type: 'quiz' | 'homework' | 'exam' | 'project' | 'discussion' | 'other';
  
  /** Student's score */
  score: number;
  
  /** Maximum possible score */
  maxScore: number;
  
  /** Assessment date */
  date: string | Date;
  
  /** Assessment name/title */
  name?: string;
  
  /** Time spent in minutes */
  timeSpent?: number;
  
  /** Number of attempts */
  attempts?: number;
}

/**
 * Performance tracking request
 */
export interface PerformanceRequest {
  /** Student identifier */
  studentId: string;
  
  /** Course identifier */
  courseId: string;
  
  /** List of assessments */
  assessments: Assessment[];
  
  /** Include trend analysis */
  includeTrends?: boolean;
  
  /** Include recommendations */
  includeRecommendations?: boolean;
}

/**
 * Performance analytics response
 */
export interface PerformanceAnalytics {
  /** Student ID */
  studentId: string;
  
  /** Course ID */
  courseId: string;
  
  /** Average score percentage */
  averageScore: number;
  
  /** Total assessments completed */
  assessmentsCompleted: number;
  
  /** Completion rate (0-1) */
  completionRate: number;
  
  /** Performance trend (positive/negative percentage) */
  trend?: number;
  
  /** Strengths identified */
  strengths?: string[];
  
  /** Areas for improvement */
  weaknesses?: string[];
  
  /** Personalized recommendations */
  recommendations?: string[];
  
  /** Time series data */
  timeSeries?: TimeSeriesPoint[];
}

/**
 * Time series data point
 */
export interface TimeSeriesPoint {
  /** Date/timestamp */
  date: string | Date;
  
  /** Value at this point */
  value: number;
  
  /** Optional label */
  label?: string;
}

/**
 * Engagement metrics request
 */
export interface EngagementRequest {
  /** Student ID or course ID (for class analytics) */
  studentId?: string;
  
  /** Course identifier */
  courseId: string;
  
  /** Analysis timeframe */
  timeframe?: 'last-7-days' | 'last-30-days' | 'this-month' | 'this-semester' | 'custom';
  
  /** Custom date range (if timeframe is 'custom') */
  dateRange?: {
    start: string | Date;
    end: string | Date;
  };
  
  /** Metrics to include */
  metrics?: Array<'participation' | 'resource-access' | 'time-on-task' | 'collaboration' | 'discussion'>;
}

/**
 * Engagement analytics response
 */
export interface EngagementAnalytics {
  /** Student or course ID */
  id: string;
  
  /** Overall engagement score (0-100) */
  overallEngagement: number;
  
  /** Participation rate (0-1) */
  participationRate?: number;
  
  /** Average time on task (hours) */
  averageTimeOnTask?: number;
  
  /** Resource access count */
  resourceAccessCount?: number;
  
  /** Discussion posts/replies */
  discussionActivity?: number;
  
  /** Collaboration score */
  collaborationScore?: number;
  
  /** Engagement trend */
  trend?: 'increasing' | 'stable' | 'declining';
  
  /** Detailed metrics */
  detailedMetrics?: Record<string, number>;
}

/**
 * Predictive analytics request
 */
export interface PredictionRequest {
  /** Student identifier */
  studentId: string;
  
  /** Course identifier */
  courseId: string;
  
  /** Model type */
  modelType: 'final-grade-prediction' | 'dropout-risk' | 'success-probability' | 'time-to-completion';
  
  /** Include intervention recommendations */
  includeInterventions?: boolean;
  
  /** Confidence threshold (0-1) */
  confidenceThreshold?: number;
}

/**
 * Prediction result
 */
export interface PredictionResult {
  /** Student ID */
  studentId: string;
  
  /** Course ID */
  courseId: string;
  
  /** Model type used */
  modelType: string;
  
  /** Predicted outcome */
  prediction: number | string;
  
  /** Confidence level (0-1) */
  confidence: number;
  
  /** Risk level */
  riskLevel?: 'low' | 'medium' | 'high';
  
  /** Contributing factors */
  factors?: Array<{
    factor: string;
    importance: number;
    direction: 'positive' | 'negative';
  }>;
  
  /** Recommended interventions */
  interventions?: Array<{
    type: string;
    description: string;
    priority: 'low' | 'medium' | 'high';
  }>;
  
  /** Model metadata */
  modelMetadata?: {
    version: string;
    accuracy: number;
    lastUpdated: string;
  };
}

/**
 * Recommendation request
 */
export interface RecommendationRequest {
  /** Student identifier */
  studentId: string;
  
  /** Course identifier */
  courseId: string;
  
  /** Include learning resources */
  includeResources?: boolean;
  
  /** Personalization level */
  personalized?: boolean;
  
  /** Recommendation type */
  type?: 'study-strategies' | 'resources' | 'peer-connections' | 'interventions' | 'all';
}

/**
 * Recommendation result
 */
export interface Recommendations {
  /** Student ID */
  studentId: string;
  
  /** Course ID */
  courseId: string;
  
  /** Study strategy recommendations */
  studyStrategies?: string[];
  
  /** Recommended learning resources */
  resources?: Array<{
    title: string;
    type: 'video' | 'article' | 'exercise' | 'tutorial' | 'other';
    url?: string;
    relevance: number;
  }>;
  
  /** Peer connection suggestions */
  peerConnections?: Array<{
    studentId: string;
    reason: string;
    compatibility: number;
  }>;
  
  /** Intervention recommendations */
  interventions?: Array<{
    type: string;
    description: string;
    urgency: 'low' | 'medium' | 'high';
  }>;
}

/**
 * Analytics event for tracking
 */
export interface AnalyticsEvent {
  /** Event type */
  eventType: string;
  
  /** Student identifier */
  studentId: string;
  
  /** Course identifier */
  courseId?: string;
  
  /** Event timestamp */
  timestamp: Date | string;
  
  /** Event properties */
  properties?: Record<string, any>;
  
  /** Session identifier */
  sessionId?: string;
}

/**
 * Batch analytics request
 */
export interface BatchAnalyticsRequest {
  /** Multiple events */
  events: AnalyticsEvent[];
  
  /** Processing mode */
  mode?: 'synchronous' | 'asynchronous';
}

/**
 * Dashboard configuration
 */
export interface DashboardConfig {
  /** Dashboard type */
  type: 'student' | 'instructor' | 'administrator';
  
  /** User identifier */
  userId: string;
  
  /** Widgets to include */
  widgets?: string[];
  
  /** Date range */
  dateRange?: {
    start: string | Date;
    end: string | Date;
  };
  
  /** Refresh interval in seconds */
  refreshInterval?: number;
}

/**
 * Error response from API
 */
export interface AnalyticsError {
  /** Error code */
  code: string;
  
  /** Error message */
  message: string;
  
  /** Additional details */
  details?: any;
  
  /** Timestamp */
  timestamp: string;
}

/**
 * API response wrapper
 */
export interface ApiResponse<T> {
  /** Success status */
  success: boolean;
  
  /** Response data */
  data?: T;
  
  /** Error information */
  error?: AnalyticsError;
  
  /** Request metadata */
  metadata?: {
    requestId: string;
    timestamp: string;
    processingTime: number;
  };
}
