/**
 * WIA-AI-015: AI-Human Collaboration Standard - TypeScript SDK
 * 弘益人間 (Hongik Ingan) · Benefit All Humanity
 *
 * @packageDocumentation
 */

import axios, { AxiosInstance } from 'axios';
import * as Types from './types';

export * from './types';

// ============================================================================
// Main Collaboration Service Class
// ============================================================================

export class CollaborationService {
  private client: AxiosInstance;
  private config: Types.CollaborationConfig;

  constructor(config: Types.CollaborationConfig) {
    this.config = {
      baseUrl: 'https://api.wia-official.org/v1/collaboration',
      timeout: 30000,
      retryAttempts: 3,
      enableLogging: false,
      ...config
    };

    this.client = axios.create({
      baseURL: this.config.baseUrl,
      timeout: this.config.timeout,
      headers: {
        'Authorization': `Bearer ${this.config.apiKey}`,
        'Content-Type': 'application/json'
      }
    });

    // Add response interceptor for error handling
    this.client.interceptors.response.use(
      response => response,
      error => {
        if (this.config.enableLogging) {
          console.error('Collaboration API Error:', error);
        }
        throw this.handleError(error);
      }
    );
  }

  // ==========================================================================
  // Session Management
  // ==========================================================================

  async createSession(request: Types.CreateSessionRequest): Promise<Types.CreateSessionResponse> {
    const response = await this.client.post<Types.CreateSessionResponse>('/sessions', request);
    return response.data;
  }

  async getSession(sessionId: string): Promise<Types.CollaborationSession> {
    const response = await this.client.get<Types.CollaborationSession>(`/sessions/${sessionId}`);
    return response.data;
  }

  async closeSession(sessionId: string): Promise<void> {
    await this.client.post(`/sessions/${sessionId}/close`);
  }

  // ==========================================================================
  // Task Management
  // ==========================================================================

  async submitTask(
    sessionId: string,
    request: Types.SubmitTaskRequest
  ): Promise<Types.SubmitTaskResponse> {
    const response = await this.client.post<Types.SubmitTaskResponse>(
      `/sessions/${sessionId}/tasks`,
      request
    );
    return response.data;
  }

  async getTask(taskId: string): Promise<Types.Task> {
    const response = await this.client.get<Types.Task>(`/tasks/${taskId}`);
    return response.data;
  }

  async getTaskStatus(taskId: string): Promise<Types.TaskStatus> {
    const task = await this.getTask(taskId);
    return task.status;
  }

  // ==========================================================================
  // AI Predictions
  // ==========================================================================

  async submitPrediction(
    taskId: string,
    request: Types.SubmitPredictionRequest
  ): Promise<Types.SubmitPredictionResponse> {
    const response = await this.client.post<Types.SubmitPredictionResponse>(
      `/tasks/${taskId}/predictions`,
      request
    );
    return response.data;
  }

  async getPrediction(predictionId: string): Promise<Types.Prediction> {
    const response = await this.client.get<Types.Prediction>(`/predictions/${predictionId}`);
    return response.data;
  }

  // ==========================================================================
  // Human Review
  // ==========================================================================

  async getReviewQueue(query: Types.ReviewQueueQuery): Promise<Types.ReviewQueueResponse> {
    const response = await this.client.get<Types.ReviewQueueResponse>('/review/queue', {
      params: query
    });
    return response.data;
  }

  async submitDecision(
    taskId: string,
    request: Types.SubmitDecisionRequest
  ): Promise<Types.SubmitDecisionResponse> {
    const response = await this.client.post<Types.SubmitDecisionResponse>(
      `/tasks/${taskId}/decisions`,
      request
    );
    return response.data;
  }

  async getDecision(decisionId: string): Promise<Types.Decision> {
    const response = await this.client.get<Types.Decision>(`/decisions/${decisionId}`);
    return response.data;
  }

  // ==========================================================================
  // Metrics
  // ==========================================================================

  async getMetrics(sessionId: string, period: string = '7d'): Promise<Types.PerformanceMetrics> {
    const response = await this.client.get<Types.PerformanceMetrics>(
      `/sessions/${sessionId}/metrics`,
      { params: { period } }
    );
    return response.data;
  }

  // ==========================================================================
  // High-Level Helper Methods
  // ==========================================================================

  /**
   * Process a task with AI-human collaboration
   * Automatically handles escalation and routing
   */
  async processTask(options: {
    sessionId: string;
    inputData: Types.TaskInput;
    aiPredict: (input: Types.TaskInput) => Promise<Types.Prediction['prediction'] & { explanation: Types.Explanation }>;
    humanReview?: (task: Types.Task, prediction: Types.Prediction) => Promise<Types.Decision['decision']>;
    priority?: Types.Priority;
  }): Promise<{
    task: Types.Task;
    prediction?: Types.Prediction;
    decision?: Types.Decision;
    escalated: boolean;
  }> {
    // Submit task
    const taskResponse = await this.submitTask(options.sessionId, {
      input_data: options.inputData,
      priority: options.priority || 'medium'
    });

    // Get AI prediction
    const aiOutput = await options.aiPredict(options.inputData);

    // Submit prediction
    const predictionResponse = await this.submitPrediction(taskResponse.task_id, {
      agent_id: 'custom-ai-agent',
      prediction: aiOutput,
      explanation: aiOutput.explanation
    });

    // Check if escalation required
    if (predictionResponse.escalation_required && options.humanReview) {
      const task = await this.getTask(taskResponse.task_id);
      const prediction = await this.getPrediction(predictionResponse.prediction_id);

      // Get human decision
      const humanDecisionData = await options.humanReview(task, prediction);

      // Submit decision
      const decisionResponse = await this.submitDecision(taskResponse.task_id, {
        reviewer_id: 'human-reviewer',
        decision: humanDecisionData
      });

      const decision = await this.getDecision(decisionResponse.decision_id);

      return {
        task,
        prediction,
        decision,
        escalated: true
      };
    }

    // Auto-approved
    const task = await this.getTask(taskResponse.task_id);
    const prediction = await this.getPrediction(predictionResponse.prediction_id);

    return {
      task,
      prediction,
      escalated: false
    };
  }

  // ==========================================================================
  // Error Handling
  // ==========================================================================

  private handleError(error: any): Types.CollaborationError {
    if (error.response) {
      // Server responded with error
      return {
        code: error.response.data?.error?.code || 'SERVER_ERROR',
        message: error.response.data?.error?.message || error.message,
        details: error.response.data?.error?.details
      };
    } else if (error.request) {
      // Request made but no response
      return {
        code: 'NETWORK_ERROR',
        message: 'No response from server',
        details: { originalError: error.message }
      };
    } else {
      // Error setting up request
      return {
        code: 'CLIENT_ERROR',
        message: error.message,
        details: { originalError: error }
      };
    }
  }
}

// ============================================================================
// Helper Classes
// ============================================================================

/**
 * Human-in-the-Loop workflow manager
 */
export class HumanInTheLoop {
  private service: CollaborationService;
  private confidenceThreshold: number;

  constructor(service: CollaborationService, confidenceThreshold: number = 0.75) {
    this.service = service;
    this.confidenceThreshold = confidenceThreshold;
  }

  /**
   * Determine if task should escalate to human review
   */
  shouldEscalate(prediction: Types.Prediction): boolean {
    return prediction.prediction.confidence < this.confidenceThreshold;
  }

  /**
   * Process with automatic escalation logic
   */
  async process(
    sessionId: string,
    inputData: Types.TaskInput,
    aiModel: (input: Types.TaskInput) => Promise<any>,
    humanReviewer?: (task: Types.Task, prediction: Types.Prediction) => Promise<any>
  ): Promise<any> {
    return this.service.processTask({
      sessionId,
      inputData,
      aiPredict: aiModel,
      humanReview: humanReviewer
    });
  }
}

/**
 * Task allocator for distributing work between AI and humans
 */
export class TaskAllocator {
  /**
   * Allocate task to best agent based on capabilities
   */
  allocate(
    task: Types.Task,
    aiAgents: Types.AIAgent[],
    humanAgents: Types.HumanAgent[]
  ): { agentType: Types.AgentType; agentId: string } {
    // Simple allocation logic - can be customized
    if (task.constraints?.must_be_human) {
      return {
        agentType: 'human',
        agentId: humanAgents[0]?.agent_id || 'default-human'
      };
    }

    if (task.constraints?.must_be_ai) {
      return {
        agentType: 'ai',
        agentId: aiAgents[0]?.agent_id || 'default-ai'
      };
    }

    // Allocate based on complexity
    if (task.complexity_assessment && task.complexity_assessment.overall_complexity > 0.7) {
      return {
        agentType: 'human',
        agentId: humanAgents[0]?.agent_id || 'default-human'
      };
    }

    return {
      agentType: 'ai',
      agentId: aiAgents[0]?.agent_id || 'default-ai'
    };
  }
}

/**
 * Metrics analyzer for monitoring collaboration performance
 */
export class MetricsAnalyzer {
  /**
   * Calculate overall collaboration health score
   */
  calculateHealthScore(metrics: Types.PerformanceMetrics): number {
    const weights = {
      quality: 0.4,
      productivity: 0.3,
      userExperience: 0.3
    };

    const qualityScore = (
      metrics.quality.accuracy +
      metrics.quality.precision +
      metrics.quality.recall
    ) / 3;

    const productivityScore = Math.min(
      metrics.productivity.efficiency_gain_percent / 100,
      1.0
    );

    const uxScore = (
      metrics.user_experience.satisfaction_score / 5 +
      (metrics.user_experience.net_promoter_score + 100) / 200 +
      metrics.user_experience.adoption_rate
    ) / 3;

    return (
      qualityScore * weights.quality +
      productivityScore * weights.productivity +
      uxScore * weights.userExperience
    );
  }

  /**
   * Identify areas for improvement
   */
  identifyIssues(metrics: Types.PerformanceMetrics): string[] {
    const issues: string[] = [];

    if (metrics.quality.accuracy < 0.85) {
      issues.push('Low accuracy - consider model retraining');
    }

    if (metrics.collaboration.ai_rejection_rate > 0.3) {
      issues.push('High AI rejection rate - review confidence calibration');
    }

    if (metrics.user_experience.satisfaction_score < 3.5) {
      issues.push('Low user satisfaction - investigate UX issues');
    }

    if (metrics.productivity.efficiency_gain_percent < 20) {
      issues.push('Low efficiency gains - optimize workflows');
    }

    return issues;
  }
}

// ============================================================================
// Factory Functions
// ============================================================================

/**
 * Create a configured collaboration service instance
 */
export function createCollaborationService(
  apiKey: string,
  options?: Partial<Types.CollaborationConfig>
): CollaborationService {
  return new CollaborationService({
    apiKey,
    ...options
  });
}

/**
 * Create a human-in-the-loop manager
 */
export function createHITL(
  service: CollaborationService,
  confidenceThreshold?: number
): HumanInTheLoop {
  return new HumanInTheLoop(service, confidenceThreshold);
}

// ============================================================================
// Constants
// ============================================================================

export const DEFAULT_CONFIG = {
  CONFIDENCE_THRESHOLD: 0.75,
  TIMEOUT_MS: 30000,
  RETRY_ATTEMPTS: 3
};

export const PRIORITY_LEVELS = {
  LOW: 'low' as Types.Priority,
  MEDIUM: 'medium' as Types.Priority,
  HIGH: 'high' as Types.Priority,
  CRITICAL: 'critical' as Types.Priority
};

// ============================================================================
// Version
// ============================================================================

export const VERSION = '1.0.0';
export const STANDARD = 'WIA-AI-015';
export const PHILOSOPHY = '弘益人間 (Hongik Ingan) · Benefit All Humanity';
