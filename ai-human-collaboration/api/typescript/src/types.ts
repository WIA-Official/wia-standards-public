/**
 * WIA-AI-015: AI-Human Collaboration Standard - TypeScript Types
 * 弘益人間 (Hongik Ingan) · Benefit All Humanity
 */

// ============================================================================
// Core Types
// ============================================================================

export type AgentType = 'ai' | 'human';
export type TaskStatus = 'pending' | 'processing' | 'ai_decided' | 'human_review' | 'completed' | 'error';
export type Priority = 'low' | 'medium' | 'high' | 'critical';
export type EscalationStrategy = 'uncertainty' | 'complexity' | 'stakes' | 'random';
export type DecisionAction = 'accept' | 'reject' | 'modify';

// ============================================================================
// Collaboration Session
// ============================================================================

export interface AIAgent {
  agent_id: string;
  agent_type: 'classifier' | 'predictor' | 'generator' | 'analyzer';
  model_version: string;
  capabilities: string[];
}

export interface HumanAgent {
  agent_id: string;
  role: 'reviewer' | 'expert' | 'approver';
  expertise_areas: string[];
}

export interface SessionConfiguration {
  confidence_threshold: number; // 0-1
  escalation_strategy: EscalationStrategy;
  feedback_enabled: boolean;
}

export interface CollaborationSession {
  session_id: string;
  created_at: string; // ISO 8601
  status: 'active' | 'paused' | 'completed' | 'failed';
  participants: {
    ai_agents: AIAgent[];
    human_agents: HumanAgent[];
  };
  configuration: SessionConfiguration;
  metadata?: Record<string, any>;
}

// ============================================================================
// Task
// ============================================================================

export interface TaskInput {
  type: string;
  content: Record<string, any>;
  features: Record<string, any>;
  metadata?: Record<string, any>;
}

export interface ComplexityAssessment {
  overall_complexity: number; // 0-1
  novelty_score: number; // 0-1
  feature_diversity: number;
  constraint_conflicts: number;
}

export interface Task {
  task_id: string;
  session_id: string;
  created_at: string;
  updated_at: string;
  status: TaskStatus;
  priority: Priority;
  input_data: TaskInput;
  complexity_assessment?: ComplexityAssessment;
  deadline?: string;
  constraints?: {
    must_be_human?: boolean;
    must_be_ai?: boolean;
    required_expertise?: string[];
  };
}

// ============================================================================
// AI Prediction
// ============================================================================

export interface FeatureImportance {
  feature_name: string;
  importance: number; // 0-1
  direction: 'positive' | 'negative';
}

export interface SimilarCase {
  case_id: string;
  similarity: number; // 0-1
  outcome: string;
}

export interface Explanation {
  feature_importance: FeatureImportance[];
  similar_cases: SimilarCase[];
  reasoning: string;
  contrastive?: {
    alternative: string;
    reasons: string[];
  };
  counterfactual?: {
    changes_needed: Array<{
      feature: string;
      current_value: any;
      required_value: any;
    }>;
  };
}

export interface Prediction {
  prediction_id: string;
  task_id: string;
  timestamp: string;
  agent_id: string;
  model_version: string;
  prediction: {
    class: string | number | Record<string, any>;
    confidence: number; // 0-1
    probability_distribution?: Record<string, number>;
  };
  explanation: Explanation;
  metadata?: {
    inference_time_ms?: number;
    model_uncertainty?: number;
    out_of_distribution_score?: number;
  };
}

// ============================================================================
// Human Decision
// ============================================================================

export interface Decision {
  decision_id: string;
  task_id: string;
  prediction_id?: string;
  timestamp: string;
  reviewer_id: string;
  decision: {
    action: DecisionAction;
    final_output: any;
    confidence?: number; // 0-1
    rationale?: string;
    time_spent_seconds: number;
  };
  ai_interaction: {
    viewed_prediction: boolean;
    viewed_explanation: boolean;
    modified_ai_suggestion: boolean;
    disagreed_with_ai: boolean;
  };
  feedback?: {
    ai_helpfulness?: number; // 1-5
    suggestion_quality?: number; // 1-5
    explanation_clarity?: number; // 1-5
    comments?: string;
  };
}

// ============================================================================
// Escalation
// ============================================================================

export type EscalationTrigger = 'confidence' | 'complexity' | 'stakes' | 'policy' | 'random' | 'manual';

export interface EscalationEvent {
  escalation_id: string;
  task_id: string;
  prediction_id: string;
  timestamp: string;
  trigger_type: EscalationTrigger;
  trigger_details: {
    confidence_below_threshold?: boolean;
    threshold_value?: number;
    complexity_score?: number;
    stakes_level?: Priority;
    policy_rule_triggered?: string;
  };
  assigned_to: string;
  priority: Priority;
  context?: Record<string, any>;
}

// ============================================================================
// Metrics
// ============================================================================

export interface ProductivityMetrics {
  tasks_completed: number;
  throughput_per_hour: number;
  average_cycle_time_seconds: number;
  efficiency_gain_percent: number;
}

export interface QualityMetrics {
  accuracy: number; // 0-1
  precision: number; // 0-1
  recall: number; // 0-1
  f1_score: number; // 0-1
  error_rate: number; // 0-1
}

export interface CollaborationMetrics {
  ai_acceptance_rate: number; // 0-1
  ai_modification_rate: number; // 0-1
  ai_rejection_rate: number; // 0-1
  average_review_time_seconds: number;
  feedback_quality_score: number; // 0-1
}

export interface UserExperienceMetrics {
  satisfaction_score: number; // 1-5
  net_promoter_score: number; // -100 to 100
  adoption_rate: number; // 0-1
  engagement_rate: number; // 0-1
}

export interface PerformanceMetrics {
  metrics_id: string;
  session_id?: string;
  period_start: string;
  period_end: string;
  productivity: ProductivityMetrics;
  quality: QualityMetrics;
  collaboration: CollaborationMetrics;
  user_experience: UserExperienceMetrics;
}

// ============================================================================
// API Request/Response Types
// ============================================================================

export interface CreateSessionRequest {
  configuration: SessionConfiguration;
  participants: {
    ai_agents: AIAgent[];
    human_agents: HumanAgent[];
  };
  metadata?: Record<string, any>;
}

export interface CreateSessionResponse {
  session_id: string;
  status: string;
  created_at: string;
}

export interface SubmitTaskRequest {
  input_data: TaskInput;
  priority?: Priority;
  deadline?: string;
  constraints?: Task['constraints'];
}

export interface SubmitTaskResponse {
  task_id: string;
  status: TaskStatus;
  estimated_completion?: string;
}

export interface SubmitPredictionRequest {
  agent_id: string;
  prediction: Prediction['prediction'];
  explanation: Explanation;
}

export interface SubmitPredictionResponse {
  prediction_id: string;
  escalation_required: boolean;
  assigned_reviewer?: string;
}

export interface SubmitDecisionRequest {
  reviewer_id: string;
  decision: Decision['decision'];
  feedback?: Decision['feedback'];
}

export interface SubmitDecisionResponse {
  decision_id: string;
  task_completed: boolean;
}

export interface ReviewQueueQuery {
  reviewer_id: string;
  priority?: Priority;
  limit?: number;
}

export interface ReviewQueueResponse {
  queue_depth: number;
  tasks: Array<{
    task_id: string;
    priority: Priority;
    ai_prediction: Prediction;
    context: Record<string, any>;
  }>;
}

// ============================================================================
// Configuration Types
// ============================================================================

export interface CollaborationConfig {
  apiKey: string;
  baseUrl?: string;
  timeout?: number;
  retryAttempts?: number;
  enableLogging?: boolean;
}

export interface ProcessingOptions {
  requireHumanReview?: boolean;
  skipAI?: boolean;
  customThreshold?: number;
  priority?: Priority;
}

// ============================================================================
// Error Types
// ============================================================================

export interface CollaborationError {
  code: string;
  message: string;
  details?: Record<string, any>;
}

// ============================================================================
// Event Types (for WebSocket/Streaming)
// ============================================================================

export type EventType = 'task.created' | 'task.completed' | 'escalation.created' | 'decision.submitted' | 'metrics.updated';

export interface CollaborationEvent {
  event_id: string;
  event_type: EventType;
  timestamp: string;
  data: Record<string, any>;
}
