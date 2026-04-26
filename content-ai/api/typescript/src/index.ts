/**
 * WIA-EDU-020: Content AI Standard - TypeScript SDK
 * 弘益人間 (홍익인간) - Benefit All Humanity
 */

import axios, { AxiosInstance, AxiosError } from 'axios';
import type {
  ContentAIConfig,
  LessonGenerationRequest,
  Lesson,
  ProblemGenerationRequest,
  Problem,
  AssessmentGenerationRequest,
  Assessment,
  VideoGenerationRequest,
  Video,
  InteractiveContentRequest,
  PersonalizationRequest,
  PersonalizationResponse,
  TranslationRequest,
  TranslationResponse,
  ContentAnalyticsRequest,
  ContentAnalytics,
  ABTestConfig,
  ABTestResult,
  APIResponse,
  APIError,
  GeneratedContent,
  LearnerProfile,
  QualityMetrics
} from './types';

export * from './types';

/**
 * Main Content AI client
 */
export class ContentAI {
  private client: AxiosInstance;
  private config: Required<ContentAIConfig>;

  constructor(config: ContentAIConfig) {
    this.config = {
      environment: 'production',
      timeout: 30000,
      retries: 3,
      baseUrl: 'https://api.wia.org/v1/content-ai',
      debug: false,
      ...config
    };

    if (this.config.environment === 'sandbox') {
      this.config.baseUrl = 'https://sandbox-api.wia.org/v1/content-ai';
    }

    this.client = axios.create({
      baseURL: this.config.baseUrl,
      timeout: this.config.timeout,
      headers: {
        'Authorization': `Bearer ${this.config.apiKey}`,
        'X-API-Key': this.config.apiKey,
        'Content-Type': 'application/json',
        'User-Agent': 'WIA-Content-AI-SDK/1.0.0'
      }
    });

    // Add retry interceptor
    this.setupRetryInterceptor();

    // Add debug logging if enabled
    if (this.config.debug) {
      this.setupDebugLogging();
    }
  }

  // ===== Content Generation =====

  /**
   * Generate a complete lesson plan
   */
  async generateLesson(request: LessonGenerationRequest): Promise<Lesson> {
    try {
      const response = await this.client.post<APIResponse<Lesson>>('/generate/lesson', request);
      return response.data.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Generate practice problems
   */
  async generateProblems(request: ProblemGenerationRequest): Promise<Problem[]> {
    try {
      const response = await this.client.post<APIResponse<{ problems: Problem[] }>>('/generate/problems', request);
      return response.data.data.problems;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Generate an assessment
   */
  async generateAssessment(request: AssessmentGenerationRequest): Promise<Assessment> {
    try {
      const response = await this.client.post<APIResponse<Assessment>>('/generate/assessment', request);
      return response.data.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Generate video content
   */
  async generateVideo(request: VideoGenerationRequest): Promise<Video> {
    try {
      const response = await this.client.post<APIResponse<Video>>('/generate/video', request);
      return response.data.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Generate interactive content
   */
  async generateInteractive(request: InteractiveContentRequest): Promise<GeneratedContent> {
    try {
      const response = await this.client.post<APIResponse<GeneratedContent>>('/generate/interactive', request);
      return response.data.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  // ===== Personalization =====

  /**
   * Personalize content for a learner
   */
  async personalizeContent(request: PersonalizationRequest): Promise<PersonalizationResponse> {
    try {
      const response = await this.client.post<APIResponse<PersonalizationResponse>>('/personalize', request);
      return response.data.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Get content recommendations for a learner
   */
  async getRecommendations(studentId: string, context?: any): Promise<any> {
    try {
      const response = await this.client.post('/recommendations', {
        studentId,
        context
      });
      return response.data.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Get learner profile
   */
  async getStudentProfile(studentId: string): Promise<LearnerProfile> {
    try {
      const response = await this.client.get<APIResponse<LearnerProfile>>(`/students/${studentId}/profile`);
      return response.data.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  // ===== Translation =====

  /**
   * Translate content to multiple languages
   */
  async translateContent(request: TranslationRequest): Promise<TranslationResponse> {
    try {
      const response = await this.client.post<APIResponse<TranslationResponse>>('/translate', request);
      return response.data.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  // ===== Analytics =====

  /**
   * Get content analytics
   */
  async getContentAnalytics(request: ContentAnalyticsRequest): Promise<ContentAnalytics> {
    try {
      const params: any = {
        timeRange: typeof request.timeRange === 'string' ? request.timeRange : undefined
      };

      if (request.metrics) {
        params.metrics = request.metrics.join(',');
      }

      const response = await this.client.get<APIResponse<ContentAnalytics>>(
        `/analytics/${request.contentId}`,
        { params }
      );
      return response.data.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Get usage statistics
   */
  async getUsage(timeRange?: string): Promise<any> {
    try {
      const response = await this.client.get('/usage', {
        params: { timeRange }
      });
      return response.data.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  // ===== A/B Testing =====

  /**
   * Create an A/B test
   */
  async createABTest(config: ABTestConfig): Promise<ABTestResult> {
    try {
      const response = await this.client.post<APIResponse<ABTestResult>>('/ab-test', config);
      return response.data.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Get A/B test results
   */
  async getABTestResults(testId: string): Promise<ABTestResult> {
    try {
      const response = await this.client.get<APIResponse<ABTestResult>>(`/ab-test/${testId}`);
      return response.data.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  // ===== Quality Assurance =====

  /**
   * Validate content quality
   */
  async validateContent(content: any, checks: string[]): Promise<QualityMetrics> {
    try {
      const response = await this.client.post<APIResponse<QualityMetrics>>('/validate', {
        content,
        checks
      });
      return response.data.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  // ===== Content Management =====

  /**
   * Get content by ID
   */
  async getContent(contentId: string): Promise<GeneratedContent> {
    try {
      const response = await this.client.get<APIResponse<GeneratedContent>>(`/content/${contentId}`);
      return response.data.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Update content
   */
  async updateContent(contentId: string, updates: Partial<GeneratedContent>): Promise<GeneratedContent> {
    try {
      const response = await this.client.put<APIResponse<GeneratedContent>>(`/content/${contentId}`, updates);
      return response.data.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Delete content
   */
  async deleteContent(contentId: string): Promise<void> {
    try {
      await this.client.delete(`/content/${contentId}`);
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * List content with filters
   */
  async listContent(filters?: any): Promise<GeneratedContent[]> {
    try {
      const response = await this.client.get<APIResponse<{ items: GeneratedContent[] }>>('/content', {
        params: filters
      });
      return response.data.data.items;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  // ===== Helper Methods =====

  private setupRetryInterceptor(): void {
    this.client.interceptors.response.use(
      response => response,
      async (error: AxiosError) => {
        const config: any = error.config;

        // Don't retry if we've exceeded max retries
        if (!config || config.__retryCount >= this.config.retries) {
          return Promise.reject(error);
        }

        config.__retryCount = config.__retryCount || 0;

        // Only retry on specific errors
        const shouldRetry =
          error.response?.status === 429 || // Rate limit
          error.response?.status === 503 || // Service unavailable
          error.code === 'ECONNABORTED' ||  // Timeout
          error.code === 'ETIMEDOUT';

        if (!shouldRetry) {
          return Promise.reject(error);
        }

        config.__retryCount += 1;

        // Exponential backoff
        const delay = Math.pow(2, config.__retryCount) * 1000;
        await this.sleep(delay);

        return this.client(config);
      }
    );
  }

  private setupDebugLogging(): void {
    this.client.interceptors.request.use(request => {
      console.log('[ContentAI Request]', {
        method: request.method,
        url: request.url,
        data: request.data
      });
      return request;
    });

    this.client.interceptors.response.use(
      response => {
        console.log('[ContentAI Response]', {
          status: response.status,
          data: response.data
        });
        return response;
      },
      error => {
        console.error('[ContentAI Error]', {
          message: error.message,
          response: error.response?.data
        });
        return Promise.reject(error);
      }
    );
  }

  private handleError(error: any): Error {
    if (axios.isAxiosError(error)) {
      const apiError: APIError = error.response?.data?.error || {
        code: 'UNKNOWN_ERROR',
        message: error.message,
        timestamp: new Date()
      };

      const err = new Error(apiError.message) as any;
      err.code = apiError.code;
      err.details = apiError.details;
      err.requestId = apiError.requestId;
      return err;
    }

    return error;
  }

  private sleep(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }
}

/**
 * Adapter classes for LMS integration
 */
export class CanvasAdapter {
  constructor(private config: { canvasUrl: string; apiKey: string }) {}

  async createModule(params: any): Promise<any> {
    // Implementation for Canvas integration
    throw new Error('Not implemented - integrate with Canvas REST API');
  }

  async createAssignment(params: any): Promise<any> {
    // Implementation for Canvas assignments
    throw new Error('Not implemented');
  }
}

export class MoodleAdapter {
  constructor(private config: { moodleUrl: string; token: string }) {}

  async createActivity(params: any): Promise<any> {
    // Implementation for Moodle integration
    throw new Error('Not implemented - integrate with Moodle Web Services');
  }
}

export class WordPressAdapter {
  constructor(private config: { siteUrl: string; credentials: any }) {}

  async createPost(params: any): Promise<any> {
    // Implementation for WordPress integration
    throw new Error('Not implemented - integrate with WordPress REST API');
  }
}

/**
 * Utility functions
 */
export class ContentAIUtils {
  /**
   * Calculate reading level using Flesch-Kincaid
   */
  static calculateReadingLevel(text: string): number {
    // Simple implementation - in production use proper NLP library
    const sentences = text.split(/[.!?]+/).length;
    const words = text.split(/\s+/).length;
    const syllables = this.countSyllables(text);

    const fleschKincaid =
      0.39 * (words / sentences) +
      11.8 * (syllables / words) -
      15.59;

    return Math.max(1, Math.min(12, Math.round(fleschKincaid)));
  }

  /**
   * Count syllables in text (simplified)
   */
  private static countSyllables(text: string): number {
    const words = text.toLowerCase().split(/\s+/);
    let syllables = 0;

    for (const word of words) {
      syllables += word.replace(/[^aeiou]/g, '').length || 1;
    }

    return syllables;
  }

  /**
   * Generate cache key for requests
   */
  static generateCacheKey(params: any): string {
    return JSON.stringify(params);
  }

  /**
   * Chunk array into batches
   */
  static chunk<T>(array: T[], size: number): T[][] {
    const chunks: T[][] = [];
    for (let i = 0; i < array.length; i += size) {
      chunks.push(array.slice(i, i + size));
    }
    return chunks;
  }
}

// Default export
export default ContentAI;
