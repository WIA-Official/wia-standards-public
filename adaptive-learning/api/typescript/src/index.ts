/**
 * WIA-EDU-003 Adaptive Learning SDK
 *
 * Philosophy: 弘익人間 (홍익인간) - Benefit All Humanity
 *
 * This SDK provides a complete implementation of the WIA-EDU-003 Adaptive Learning Standard,
 * enabling developers to build personalized learning experiences that adapt to each learner's
 * unique needs, styles, and goals.
 *
 * @packageDocumentation
 */

import axios, { AxiosInstance } from 'axios';
import {
  LearnerProfile,
  ContentRecommendation,
  AssessmentResponse,
  AssessmentResult,
  MasteryStatus,
  LearningAnalytics,
  AdaptiveLearningConfig,
  APIResponse,
  ContentMetadata,
  Topic,
  LearningStyle,
  ContentFormat,
  MasteryLevel,
  PaginatedResponse,
  PaginationParams
} from './types';

/**
 * Main SDK class for interacting with WIA-EDU-003 Adaptive Learning API
 *
 * @example
 * ```typescript
 * const sdk = new AdaptiveLearningSDK({
 *   api_base_url: 'https://api.wia.org/v1',
 *   api_key: 'your_api_key',
 *   version: '1.0.0'
 * });
 *
 * // Create learner profile
 * const learner = await sdk.createLearner({
 *   user_id: 'user_123',
 *   initial_assessment: {
 *     learning_style: LearningStyle.VISUAL,
 *     knowledge_level: 'intermediate'
 *   }
 * });
 * ```
 */
export class AdaptiveLearningSDK {
  private client: AxiosInstance;
  private config: AdaptiveLearningConfig;

  /**
   * Initialize the Adaptive Learning SDK
   *
   * @param config - SDK configuration including API URL and credentials
   */
  constructor(config: AdaptiveLearningConfig) {
    this.config = config;

    this.client = axios.create({
      baseURL: config.api_base_url,
      timeout: config.timeout_ms || 30000,
      headers: {
        'Authorization': `Bearer ${config.api_key}`,
        'Content-Type': 'application/json',
        'X-WIA-Version': config.version
      }
    });
  }

  /**
   * Create a new learner profile
   *
   * @param params - Initial learner information
   * @returns Created learner profile
   */
  async createLearner(params: {
    user_id: string;
    initial_assessment?: {
      vark_responses?: number[];
      learning_style?: LearningStyle;
      knowledge_level?: 'beginner' | 'intermediate' | 'advanced';
      goals?: string[];
    };
  }): Promise<LearnerProfile> {
    const response = await this.client.post<APIResponse<LearnerProfile>>('/learners', params);

    if (!response.data.success || !response.data.data) {
      throw new Error(response.data.error?.message || 'Failed to create learner');
    }

    return response.data.data;
  }

  /**
   * Get learner profile by ID
   *
   * @param learner_id - Unique learner identifier
   * @returns Learner profile
   */
  async getLearner(learner_id: string): Promise<LearnerProfile> {
    const response = await this.client.get<APIResponse<LearnerProfile>>(`/learners/${learner_id}`);

    if (!response.data.success || !response.data.data) {
      throw new Error(response.data.error?.message || 'Failed to get learner');
    }

    return response.data.data;
  }

  /**
   * Update learner profile
   *
   * @param learner_id - Unique learner identifier
   * @param updates - Partial profile updates
   * @returns Updated learner profile
   */
  async updateLearner(
    learner_id: string,
    updates: Partial<LearnerProfile>
  ): Promise<LearnerProfile> {
    const response = await this.client.patch<APIResponse<LearnerProfile>>(
      `/learners/${learner_id}`,
      updates
    );

    if (!response.data.success || !response.data.data) {
      throw new Error(response.data.error?.message || 'Failed to update learner');
    }

    return response.data.data;
  }

  /**
   * Get personalized content recommendations for a learner
   *
   * @param learner_id - Unique learner identifier
   * @param params - Recommendation parameters
   * @returns Array of recommended content
   */
  async getRecommendations(
    learner_id: string,
    params?: {
      subject?: string;
      topic?: string;
      limit?: number;
      include_review?: boolean;
      preferred_formats?: ContentFormat[];
    }
  ): Promise<ContentRecommendation[]> {
    const response = await this.client.get<APIResponse<ContentRecommendation[]>>(
      `/learners/${learner_id}/recommendations`,
      { params }
    );

    if (!response.data.success || !response.data.data) {
      throw new Error(response.data.error?.message || 'Failed to get recommendations');
    }

    return response.data.data;
  }

  /**
   * Record an assessment response
   *
   * @param learner_id - Unique learner identifier
   * @param assessment - Assessment response data
   * @returns Assessment result with difficulty adjustments
   */
  async recordAssessment(
    learner_id: string,
    assessment: AssessmentResponse
  ): Promise<AssessmentResult> {
    const response = await this.client.post<APIResponse<AssessmentResult>>(
      `/learners/${learner_id}/assessments`,
      assessment
    );

    if (!response.data.success || !response.data.data) {
      throw new Error(response.data.error?.message || 'Failed to record assessment');
    }

    return response.data.data;
  }

  /**
   * Check mastery status for a topic
   *
   * @param learner_id - Unique learner identifier
   * @param topic_id - Topic to check
   * @returns Mastery status
   */
  async checkMastery(learner_id: string, topic_id: string): Promise<MasteryStatus> {
    const response = await this.client.get<APIResponse<MasteryStatus>>(
      `/learners/${learner_id}/mastery/${topic_id}`
    );

    if (!response.data.success || !response.data.data) {
      throw new Error(response.data.error?.message || 'Failed to check mastery');
    }

    return response.data.data;
  }

  /**
   * Get learning analytics for a learner
   *
   * @param learner_id - Unique learner identifier
   * @param params - Analytics parameters
   * @returns Learning analytics data
   */
  async getAnalytics(
    learner_id: string,
    params?: {
      timeframe?: string;
      include_predictions?: boolean;
    }
  ): Promise<LearningAnalytics> {
    const response = await this.client.get<APIResponse<LearningAnalytics>>(
      `/learners/${learner_id}/analytics`,
      { params }
    );

    if (!response.data.success || !response.data.data) {
      throw new Error(response.data.error?.message || 'Failed to get analytics');
    }

    return response.data.data;
  }

  /**
   * Get content metadata by ID
   *
   * @param content_id - Unique content identifier
   * @returns Content metadata
   */
  async getContent(content_id: string): Promise<ContentMetadata> {
    const response = await this.client.get<APIResponse<ContentMetadata>>(`/content/${content_id}`);

    if (!response.data.success || !response.data.data) {
      throw new Error(response.data.error?.message || 'Failed to get content');
    }

    return response.data.data;
  }

  /**
   * Search for content
   *
   * @param params - Search parameters
   * @returns Paginated content results
   */
  async searchContent(
    params: {
      query?: string;
      subject?: string;
      topic?: string;
      difficulty_min?: number;
      difficulty_max?: number;
      formats?: ContentFormat[];
      learning_styles?: LearningStyle[];
    } & PaginationParams
  ): Promise<PaginatedResponse<ContentMetadata>> {
    const response = await this.client.get<APIResponse<PaginatedResponse<ContentMetadata>>>(
      '/content/search',
      { params }
    );

    if (!response.data.success || !response.data.data) {
      throw new Error(response.data.error?.message || 'Failed to search content');
    }

    return response.data.data;
  }

  /**
   * Get topic information including prerequisites
   *
   * @param topic_id - Unique topic identifier
   * @returns Topic data
   */
  async getTopic(topic_id: string): Promise<Topic> {
    const response = await this.client.get<APIResponse<Topic>>(`/topics/${topic_id}`);

    if (!response.data.success || !response.data.data) {
      throw new Error(response.data.error?.message || 'Failed to get topic');
    }

    return response.data.data;
  }

  /**
   * Get topics available for a learner based on mastery
   *
   * @param learner_id - Unique learner identifier
   * @param params - Filter parameters
   * @returns Available topics
   */
  async getAvailableTopics(
    learner_id: string,
    params?: {
      subject?: string;
      include_locked?: boolean;
    }
  ): Promise<Topic[]> {
    const response = await this.client.get<APIResponse<Topic[]>>(
      `/learners/${learner_id}/topics`,
      { params }
    );

    if (!response.data.success || !response.data.data) {
      throw new Error(response.data.error?.message || 'Failed to get available topics');
    }

    return response.data.data;
  }

  /**
   * Track learner interaction with content
   *
   * @param learner_id - Unique learner identifier
   * @param interaction - Interaction data
   */
  async trackInteraction(
    learner_id: string,
    interaction: {
      content_id: string;
      action: 'start' | 'pause' | 'resume' | 'complete' | 'skip';
      time_spent?: number;
      engagement?: number;
      timestamp?: Date;
    }
  ): Promise<void> {
    const response = await this.client.post<APIResponse<void>>(
      `/learners/${learner_id}/interactions`,
      interaction
    );

    if (!response.data.success) {
      throw new Error(response.data.error?.message || 'Failed to track interaction');
    }
  }
}

/**
 * Convenience class for managing a single learner session
 */
export class LearnerSession {
  private sdk: AdaptiveLearningSDK;
  private learner_id: string;
  private profile: LearnerProfile | null = null;

  constructor(sdk: AdaptiveLearningSDK, learner_id: string) {
    this.sdk = sdk;
    this.learner_id = learner_id;
  }

  /**
   * Load learner profile
   */
  async load(): Promise<LearnerProfile> {
    this.profile = await this.sdk.getLearner(this.learner_id);
    return this.profile;
  }

  /**
   * Get cached profile or load if not cached
   */
  async getProfile(): Promise<LearnerProfile> {
    if (!this.profile) {
      return await this.load();
    }
    return this.profile;
  }

  /**
   * Get recommendations for this learner
   */
  async getRecommendations(params?: Parameters<AdaptiveLearningSDK['getRecommendations']>[1]) {
    return await this.sdk.getRecommendations(this.learner_id, params);
  }

  /**
   * Record assessment for this learner
   */
  async recordAssessment(assessment: AssessmentResponse) {
    return await this.sdk.recordAssessment(this.learner_id, assessment);
  }

  /**
   * Check mastery for a topic
   */
  async checkMastery(topic_id: string) {
    return await this.sdk.checkMastery(this.learner_id, topic_id);
  }

  /**
   * Get analytics for this learner
   */
  async getAnalytics(params?: Parameters<AdaptiveLearningSDK['getAnalytics']>[1]) {
    return await this.sdk.getAnalytics(this.learner_id, params);
  }

  /**
   * Track interaction
   */
  async trackInteraction(interaction: Parameters<AdaptiveLearningSDK['trackInteraction']>[1]) {
    return await this.sdk.trackInteraction(this.learner_id, interaction);
  }
}

// Export all types
export * from './types';

// Export version
export const VERSION = '1.0.0';

// Export default
export default AdaptiveLearningSDK;
