/**
 * WIA-UNI-012: Language Bridge Standard
 * TypeScript SDK
 *
 * @package @wia/language-bridge
 * @version 1.0.0
 * @license MIT
 *
 * Philosophy: 弘益人間 (홍익인간) · Benefit All Humanity
 */

import axios, { AxiosInstance } from 'axios';
import type * as Types from './types';

export * from './types';

// ============================================================================
// Main Language Bridge Class
// ============================================================================

export class LanguageBridge {
  private client: AxiosInstance;
  private config: Types.LanguageBridgeConfig;
  private cache: Map<string, any>;

  constructor(config: Types.LanguageBridgeConfig = {}) {
    this.config = {
      apiEndpoint: config.apiEndpoint || 'https://api.wia-official.org/v1/language-bridge',
      enableCaching: config.enableCaching ?? true,
      cacheDuration: config.cacheDuration || 3600,
      timeout: config.timeout || 30000,
      offlineMode: config.offlineMode || false,
      logLevel: config.logLevel || 'info',
      ...config
    };

    this.client = axios.create({
      baseURL: this.config.apiEndpoint,
      timeout: this.config.timeout,
      headers: {
        'Content-Type': 'application/json',
        ...(this.config.apiKey && { 'Authorization': `Bearer ${this.config.apiKey}` })
      }
    });

    this.cache = new Map();
  }

  // ==========================================================================
  // Translation Methods
  // ==========================================================================

  /**
   * Translate text between North and South Korean dialects
   */
  async translate(request: Types.TranslationRequest): Promise<Types.TranslationResult> {
    const cacheKey = this.getCacheKey('translate', request);

    if (this.config.enableCaching) {
      const cached = this.getFromCache(cacheKey);
      if (cached) return cached;
    }

    const response = await this.client.post<Types.APIResponse<Types.TranslationResult>>('/translate', request);

    if (!response.data.success) {
      throw new Error(response.data.error?.message || 'Translation failed');
    }

    const result = response.data.data!;

    if (this.config.enableCaching) {
      this.setCache(cacheKey, result);
    }

    return result;
  }

  /**
   * Batch translate multiple texts
   */
  async batchTranslate(request: Types.BatchTranslationRequest): Promise<Types.BatchTranslationResult> {
    const response = await this.client.post<Types.APIResponse<Types.BatchTranslationResult>>('/translate/batch', request);

    if (!response.data.success) {
      throw new Error(response.data.error?.message || 'Batch translation failed');
    }

    return response.data.data!;
  }

  /**
   * Translate speech/voice
   */
  async translateVoice(request: Types.VoiceTranslationRequest): Promise<Types.VoiceTranslationResult> {
    const formData = new FormData();
    formData.append('audio', request.audioData);
    formData.append('sourceDialect', request.sourceDialect);
    formData.append('targetDialect', request.targetDialect);
    if (request.outputFormat) formData.append('outputFormat', request.outputFormat);

    const response = await this.client.post<Types.APIResponse<Types.VoiceTranslationResult>>(
      '/translate/voice',
      formData,
      { headers: { 'Content-Type': 'multipart/form-data' } }
    );

    if (!response.data.success) {
      throw new Error(response.data.error?.message || 'Voice translation failed');
    }

    return response.data.data!;
  }

  // ==========================================================================
  // Dictionary Methods
  // ==========================================================================

  /**
   * Look up a word in the unified dictionary
   */
  async lookup(term: string, dialect?: Types.Dialect): Promise<Types.DictionaryEntry | null> {
    const cacheKey = this.getCacheKey('lookup', { term, dialect });

    if (this.config.enableCaching) {
      const cached = this.getFromCache(cacheKey);
      if (cached) return cached;
    }

    const response = await this.client.get<Types.APIResponse<Types.DictionaryEntry>>(
      `/dictionary/lookup`,
      { params: { term, dialect } }
    );

    if (!response.data.success) {
      return null;
    }

    const result = response.data.data!;

    if (this.config.enableCaching) {
      this.setCache(cacheKey, result);
    }

    return result;
  }

  /**
   * Search the dictionary
   */
  async search(request: Types.SearchRequest): Promise<Types.SearchResult> {
    const response = await this.client.post<Types.APIResponse<Types.SearchResult>>('/dictionary/search', request);

    if (!response.data.success) {
      throw new Error(response.data.error?.message || 'Search failed');
    }

    return response.data.data!;
  }

  /**
   * Get random dictionary entry (for learning)
   */
  async getRandomEntry(category?: string): Promise<Types.DictionaryEntry> {
    const response = await this.client.get<Types.APIResponse<Types.DictionaryEntry>>(
      '/dictionary/random',
      { params: { category } }
    );

    if (!response.data.success) {
      throw new Error(response.data.error?.message || 'Failed to get random entry');
    }

    return response.data.data!;
  }

  /**
   * Contribute to the dictionary
   */
  async contribute(contribution: Types.ContributionRequest): Promise<Types.ContributionStatus> {
    const response = await this.client.post<Types.APIResponse<Types.ContributionStatus>>(
      '/dictionary/contribute',
      contribution
    );

    if (!response.data.success) {
      throw new Error(response.data.error?.message || 'Contribution failed');
    }

    return response.data.data!;
  }

  // ==========================================================================
  // Dialect Mapping Methods
  // ==========================================================================

  /**
   * Get dialect differences by category
   */
  async getDialectDifferences(request: Types.DialectMappingRequest = {}): Promise<Types.DialectMappingResult> {
    const cacheKey = this.getCacheKey('dialect-map', request);

    if (this.config.enableCaching) {
      const cached = this.getFromCache(cacheKey);
      if (cached) return cached;
    }

    const response = await this.client.get<Types.APIResponse<Types.DialectMappingResult>>(
      '/dialect/map',
      { params: request }
    );

    if (!response.data.success) {
      throw new Error(response.data.error?.message || 'Failed to get dialect differences');
    }

    const result = response.data.data!;

    if (this.config.enableCaching) {
      this.setCache(cacheKey, result);
    }

    return result;
  }

  // ==========================================================================
  // Learning Methods
  // ==========================================================================

  /**
   * Get a learning lesson by ID
   */
  async getLesson(lessonId: string): Promise<Types.LearningLesson> {
    const cacheKey = this.getCacheKey('lesson', { lessonId });

    if (this.config.enableCaching) {
      const cached = this.getFromCache(cacheKey);
      if (cached) return cached;
    }

    const response = await this.client.get<Types.APIResponse<Types.LearningLesson>>(
      `/learning/lessons/${lessonId}`
    );

    if (!response.data.success) {
      throw new Error(response.data.error?.message || 'Failed to get lesson');
    }

    const result = response.data.data!;

    if (this.config.enableCaching) {
      this.setCache(cacheKey, result);
    }

    return result;
  }

  /**
   * Get user's learning progress
   */
  async getProgress(userId: string): Promise<Types.LearningProgress> {
    const response = await this.client.get<Types.APIResponse<Types.LearningProgress>>(
      `/learning/progress/${userId}`
    );

    if (!response.data.success) {
      throw new Error(response.data.error?.message || 'Failed to get progress');
    }

    return response.data.data!;
  }

  /**
   * Submit lesson completion/assessment
   */
  async submitAssessment(userId: string, lessonId: string, answers: any[]): Promise<{ score: number; passed: boolean }> {
    const response = await this.client.post<Types.APIResponse<{ score: number; passed: boolean }>>(
      `/learning/lessons/${lessonId}/submit`,
      { userId, answers }
    );

    if (!response.data.success) {
      throw new Error(response.data.error?.message || 'Failed to submit assessment');
    }

    return response.data.data!;
  }

  // ==========================================================================
  // Utility Methods
  // ==========================================================================

  /**
   * Clear the cache
   */
  clearCache(): void {
    this.cache.clear();
  }

  /**
   * Get API health status
   */
  async healthCheck(): Promise<boolean> {
    try {
      const response = await this.client.get('/health');
      return response.status === 200;
    } catch {
      return false;
    }
  }

  // ==========================================================================
  // Private Methods
  // ==========================================================================

  private getCacheKey(method: string, params: any): string {
    return `${method}:${JSON.stringify(params)}`;
  }

  private getFromCache(key: string): any | null {
    const item = this.cache.get(key);
    if (!item) return null;

    const now = Date.now();
    if (now - item.timestamp > (this.config.cacheDuration! * 1000)) {
      this.cache.delete(key);
      return null;
    }

    return item.data;
  }

  private setCache(key: string, data: any): void {
    this.cache.set(key, {
      data,
      timestamp: Date.now()
    });
  }
}

// ============================================================================
// Convenience Functions
// ============================================================================

/**
 * Quick translation function
 */
export async function quickTranslate(
  text: string,
  from: Types.Dialect,
  to: Types.Dialect,
  apiKey?: string
): Promise<string> {
  const bridge = new LanguageBridge({ apiKey });
  const result = await bridge.translate({
    text,
    sourceDialect: from,
    targetDialect: to
  });
  return result.translated;
}

/**
 * Quick dictionary lookup
 */
export async function quickLookup(term: string, apiKey?: string): Promise<Types.DictionaryEntry | null> {
  const bridge = new LanguageBridge({ apiKey });
  return await bridge.lookup(term);
}

// ============================================================================
// Default Export
// ============================================================================

export default LanguageBridge;
