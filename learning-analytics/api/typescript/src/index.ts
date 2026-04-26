/**
 * WIA-EDU-004: Learning Analytics SDK
 * 
 * A comprehensive TypeScript/JavaScript SDK for implementing learning analytics
 * following the WIA-EDU-004 standard.
 * 
 * @packageDocumentation
 * @module @wia/learning-analytics
 * 
 * @example
 * ```typescript
 * import { LearningAnalytics } from '@wia/learning-analytics';
 * 
 * const analytics = new LearningAnalytics({
 *   institutionId: 'university-123',
 *   apiKey: 'your-api-key',
 *   privacyMode: 'strict'
 * });
 * 
 * const performance = await analytics.trackPerformance({
 *   studentId: 'student-456',
 *   courseId: 'math-101',
 *   assessments: [...]
 * });
 * ```
 */

import axios, { AxiosInstance } from 'axios';
import { v4 as uuidv4 } from 'uuid';

import type {
  LearningAnalyticsConfig,
  PerformanceRequest,
  PerformanceAnalytics,
  EngagementRequest,
  EngagementAnalytics,
  PredictionRequest,
  PredictionResult,
  RecommendationRequest,
  Recommendations,
  AnalyticsEvent,
  BatchAnalyticsRequest,
  DashboardConfig,
  ApiResponse
} from './types';

/**
 * Main Learning Analytics SDK class
 * 
 * 弘益人間 (Hongik Ingan) - Benefit All Humanity through data-driven education
 */
export class LearningAnalytics {
  private config: Required<LearningAnalyticsConfig>;
  private httpClient: AxiosInstance;

  /**
   * Initialize Learning Analytics SDK
   * 
   * @param config - Configuration options
   * 
   * @example
   * ```typescript
   * const analytics = new LearningAnalytics({
   *   institutionId: 'university-123',
   *   apiEndpoint: 'https://api.wia-analytics.org',
   *   apiKey: 'your-api-key',
   *   privacyMode: 'strict',
   *   enablePredictive: true
   * });
   * ```
   */
  constructor(config: LearningAnalyticsConfig) {
    // Set defaults
    this.config = {
      institutionId: config.institutionId,
      apiEndpoint: config.apiEndpoint || 'https://api.wia-analytics.org/v1',
      apiKey: config.apiKey || '',
      privacyMode: config.privacyMode || 'standard',
      enablePredictive: config.enablePredictive !== false,
      dataRetention: config.dataRetention || 7,
      timeout: config.timeout || 30000
    };

    // Initialize HTTP client
    this.httpClient = axios.create({
      baseURL: this.config.apiEndpoint,
      timeout: this.config.timeout,
      headers: {
        'Content-Type': 'application/json',
        'X-WIA-Institution': this.config.institutionId,
        'X-WIA-Privacy-Mode': this.config.privacyMode,
        ...(this.config.apiKey && { 'Authorization': `Bearer ${this.config.apiKey}` })
      }
    });
  }

  /**
   * Track student performance
   * 
   * @param request - Performance tracking request
   * @returns Performance analytics results
   * 
   * @example
   * ```typescript
   * const performance = await analytics.trackPerformance({
   *   studentId: 'student-456',
   *   courseId: 'math-101',
   *   assessments: [
   *     { type: 'quiz', score: 85, maxScore: 100, date: '2025-01-15' },
   *     { type: 'homework', score: 92, maxScore: 100, date: '2025-01-18' }
   *   ],
   *   includeTrends: true,
   *   includeRecommendations: true
   * });
   * ```
   */
  async trackPerformance(request: PerformanceRequest): Promise<PerformanceAnalytics> {
    try {
      const response = await this.httpClient.post<ApiResponse<PerformanceAnalytics>>(
        '/performance/track',
        request
      );
      
      if (!response.data.success || !response.data.data) {
        throw new Error(response.data.error?.message || 'Performance tracking failed');
      }
      
      return response.data.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Analyze student engagement
   * 
   * @param request - Engagement analysis request
   * @returns Engagement analytics results
   * 
   * @example
   * ```typescript
   * const engagement = await analytics.analyzeEngagement({
   *   studentId: 'student-456',
   *   courseId: 'math-101',
   *   timeframe: 'last-30-days',
   *   metrics: ['participation', 'resource-access', 'time-on-task']
   * });
   * ```
   */
  async analyzeEngagement(request: EngagementRequest): Promise<EngagementAnalytics> {
    try {
      const response = await this.httpClient.post<ApiResponse<EngagementAnalytics>>(
        '/engagement/analyze',
        request
      );
      
      if (!response.data.success || !response.data.data) {
        throw new Error(response.data.error?.message || 'Engagement analysis failed');
      }
      
      return response.data.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Generate outcome predictions
   * 
   * @param request - Prediction request
   * @returns Prediction results with confidence and recommendations
   * 
   * @example
   * ```typescript
   * const prediction = await analytics.predictOutcome({
   *   studentId: 'student-456',
   *   courseId: 'math-101',
   *   modelType: 'final-grade-prediction',
   *   includeInterventions: true
   * });
   * 
   * console.log('Predicted grade:', prediction.prediction);
   * console.log('Risk level:', prediction.riskLevel);
   * console.log('Interventions:', prediction.interventions);
   * ```
   */
  async predictOutcome(request: PredictionRequest): Promise<PredictionResult> {
    if (!this.config.enablePredictive) {
      throw new Error('Predictive analytics not enabled. Set enablePredictive: true in config.');
    }

    try {
      const response = await this.httpClient.post<ApiResponse<PredictionResult>>(
        '/predictive/predict',
        request
      );
      
      if (!response.data.success || !response.data.data) {
        throw new Error(response.data.error?.message || 'Prediction failed');
      }
      
      return response.data.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Get personalized recommendations
   * 
   * @param request - Recommendation request
   * @returns Personalized recommendations for student
   * 
   * @example
   * ```typescript
   * const recommendations = await analytics.getRecommendations({
   *   studentId: 'student-456',
   *   courseId: 'math-101',
   *   includeResources: true,
   *   personalized: true,
   *   type: 'all'
   * });
   * ```
   */
  async getRecommendations(request: RecommendationRequest): Promise<Recommendations> {
    try {
      const response = await this.httpClient.post<ApiResponse<Recommendations>>(
        '/recommendations/get',
        request
      );
      
      if (!response.data.success || !response.data.data) {
        throw new Error(response.data.error?.message || 'Recommendations failed');
      }
      
      return response.data.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Track analytics event
   * 
   * @param event - Analytics event to track
   * 
   * @example
   * ```typescript
   * await analytics.trackEvent({
   *   eventType: 'video_viewed',
   *   studentId: 'student-456',
   *   courseId: 'math-101',
   *   timestamp: new Date(),
   *   properties: {
   *     videoId: 'intro-to-calculus',
   *     duration: 450,
   *     completionPercentage: 95
   *   }
   * });
   * ```
   */
  async trackEvent(event: AnalyticsEvent): Promise<void> {
    try {
      const response = await this.httpClient.post<ApiResponse<void>>(
        '/events/track',
        {
          ...event,
          timestamp: event.timestamp instanceof Date ? event.timestamp.toISOString() : event.timestamp
        }
      );
      
      if (!response.data.success) {
        throw new Error(response.data.error?.message || 'Event tracking failed');
      }
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Track multiple events in batch
   * 
   * @param request - Batch tracking request
   * 
   * @example
   * ```typescript
   * await analytics.trackEventsBatch({
   *   events: [
   *     { eventType: 'login', studentId: 'student-456', timestamp: new Date() },
   *     { eventType: 'page_view', studentId: 'student-456', courseId: 'math-101', timestamp: new Date() }
   *   ],
   *   mode: 'asynchronous'
   * });
   * ```
   */
  async trackEventsBatch(request: BatchAnalyticsRequest): Promise<void> {
    try {
      const response = await this.httpClient.post<ApiResponse<void>>(
        '/events/batch',
        request
      );
      
      if (!response.data.success) {
        throw new Error(response.data.error?.message || 'Batch tracking failed');
      }
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Generate dashboard configuration
   * 
   * @param config - Dashboard configuration
   * @returns Dashboard URL or configuration
   * 
   * @example
   * ```typescript
   * const dashboardUrl = await analytics.generateDashboard({
   *   type: 'student',
   *   userId: 'student-456',
   *   widgets: ['performance', 'engagement', 'recommendations'],
   *   refreshInterval: 300
   * });
   * ```
   */
  async generateDashboard(config: DashboardConfig): Promise<string> {
    try {
      const response = await this.httpClient.post<ApiResponse<{ url: string }>>(
        '/dashboard/generate',
        config
      );
      
      if (!response.data.success || !response.data.data) {
        throw new Error(response.data.error?.message || 'Dashboard generation failed');
      }
      
      return response.data.data.url;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Handle and format errors
   * 
   * @param error - Original error
   * @returns Formatted error
   */
  private handleError(error: any): Error {
    if (axios.isAxiosError(error)) {
      const message = error.response?.data?.error?.message || error.message;
      return new Error(`Learning Analytics API Error: ${message}`);
    }
    return error instanceof Error ? error : new Error('Unknown error occurred');
  }
}

// Export types
export * from './types';

// Default export
export default LearningAnalytics;
