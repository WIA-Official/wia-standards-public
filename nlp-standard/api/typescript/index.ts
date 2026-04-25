/**
 * WIA-AI-023 NLP Standard - TypeScript SDK
 * Version: 1.0.0
 *
 * 弘익人間 (Hongik Ingan) - Benefit All Humanity
 * © 2025 WIA - World Certification Industry Association
 */

import type {
  ClientConfig,
  TokenizationRequest,
  TokenizationResponse,
  NERRequest,
  NERResponse,
  SentimentRequest,
  SentimentResponse,
  ClassificationRequest,
  ClassificationResponse,
  GenerationRequest,
  GenerationResponse,
  SummarizationRequest,
  SummarizationResponse,
  ServiceInfo,
  HealthStatus,
  BatchRequest,
  BatchResponse,
  WIAError,
  AuthenticationError,
  RateLimitError,
  ServerError,
  ValidationError
} from './types';

export * from './types';

/**
 * WIA-AI-023 NLP Client
 *
 * @example
 * ```typescript
 * const client = new WIANLPClient({
 *   baseURL: 'https://api.example.com/nlp/v1',
 *   apiKey: 'your-api-key'
 * });
 *
 * const result = await client.analyzeSentiment({
 *   text: 'This is wonderful!',
 *   language: 'en'
 * });
 * ```
 */
export class WIANLPClient {
  private config: Required<ClientConfig>;

  constructor(config: ClientConfig) {
    this.config = {
      baseURL: config.baseURL,
      apiKey: config.apiKey || '',
      bearerToken: config.bearerToken || '',
      timeout: config.timeout || 30000,
      retries: config.retries || 3,
      headers: {
        'Content-Type': 'application/json',
        ...config.headers
      }
    };
  }

  /**
   * Tokenize text into tokens
   */
  async tokenize(
    text: string,
    language: string = 'en',
    config?: TokenizationRequest['config']
  ): Promise<TokenizationResponse> {
    const request: TokenizationRequest = {
      standard: 'WIA-AI-023',
      version: '1.0',
      task: 'tokenization',
      input: { text, language },
      config
    };

    return this.request<TokenizationResponse>('/tokenize', request);
  }

  /**
   * Extract named entities from text
   */
  async extractEntities(
    text: string,
    language: string = 'en',
    config?: NERRequest['config']
  ): Promise<NERResponse> {
    const request: NERRequest = {
      standard: 'WIA-AI-023',
      version: '1.0',
      task: 'named_entity_recognition',
      input: { text, language },
      config
    };

    return this.request<NERResponse>('/ner', request);
  }

  /**
   * Analyze sentiment of text
   */
  async analyzeSentiment(
    text: string,
    language: string = 'en',
    config?: SentimentRequest['config']
  ): Promise<SentimentResponse> {
    const request: SentimentRequest = {
      standard: 'WIA-AI-023',
      version: '1.0',
      task: 'sentiment_analysis',
      input: { text, language },
      config
    };

    return this.request<SentimentResponse>('/sentiment', request);
  }

  /**
   * Classify text into categories
   */
  async classify(
    text: string,
    language: string = 'en',
    config?: ClassificationRequest['config']
  ): Promise<ClassificationResponse> {
    const request: ClassificationRequest = {
      standard: 'WIA-AI-023',
      version: '1.0',
      task: 'text_classification',
      input: { text, language },
      config
    };

    return this.request<ClassificationResponse>('/classify', request);
  }

  /**
   * Generate text from prompt
   */
  async generate(
    prompt: string,
    language: string = 'en',
    config?: GenerationRequest['config']
  ): Promise<GenerationResponse> {
    const request: GenerationRequest = {
      standard: 'WIA-AI-023',
      version: '1.0',
      task: 'text_generation',
      input: { prompt, language },
      config
    };

    return this.request<GenerationResponse>('/generate', request);
  }

  /**
   * Summarize text
   */
  async summarize(
    text: string,
    language: string = 'en',
    config?: SummarizationRequest['config']
  ): Promise<SummarizationResponse> {
    const request: SummarizationRequest = {
      standard: 'WIA-AI-023',
      version: '1.0',
      task: 'summarization',
      input: { text, language },
      config
    };

    return this.request<SummarizationResponse>('/summarize', request);
  }

  /**
   * Process batch of requests
   */
  async batch(requests: BatchRequest['requests']): Promise<BatchResponse> {
    const request: BatchRequest = { requests };
    return this.request<BatchResponse>('/batch', request);
  }

  /**
   * Get service information
   */
  async getServiceInfo(): Promise<ServiceInfo> {
    return this.request<ServiceInfo>('/info', null, 'GET');
  }

  /**
   * Check service health
   */
  async checkHealth(): Promise<HealthStatus> {
    return this.request<HealthStatus>('/health', null, 'GET');
  }

  /**
   * Make HTTP request
   */
  private async request<T>(
    endpoint: string,
    body: any,
    method: 'GET' | 'POST' = 'POST',
    attempt: number = 1
  ): Promise<T> {
    const url = `${this.config.baseURL}${endpoint}`;
    const headers = { ...this.config.headers };

    // Add authentication
    if (this.config.bearerToken) {
      headers['Authorization'] = `Bearer ${this.config.bearerToken}`;
    } else if (this.config.apiKey) {
      headers['X-API-Key'] = this.config.apiKey;
    }

    const options: RequestInit = {
      method,
      headers,
      signal: AbortSignal.timeout(this.config.timeout)
    };

    if (body && method === 'POST') {
      options.body = JSON.stringify(body);
    }

    try {
      const response = await fetch(url, options);
      const data = await response.json();

      if (!response.ok) {
        throw this.createError(response.status, data);
      }

      return data as T;
    } catch (error: any) {
      // Retry on network errors
      if (attempt < this.config.retries && this.isRetryable(error)) {
        await this.delay(Math.pow(2, attempt) * 1000);
        return this.request<T>(endpoint, body, method, attempt + 1);
      }

      throw error;
    }
  }

  /**
   * Create appropriate error type
   */
  private createError(status: number, data: any): Error {
    const message = data.status?.message || data.message || 'Unknown error';
    const details = data.status?.details || data.details;
    const requestId = data.metadata?.request_id;

    switch (status) {
      case 400:
        return new ValidationError(message, details);
      case 401:
      case 403:
        return new AuthenticationError(message, details);
      case 429:
        const retryAfter = parseInt(data.retry_after || '60');
        return new RateLimitError(message, retryAfter);
      case 500:
      case 503:
        return new ServerError(message, details);
      default:
        return new Error(message);
    }
  }

  /**
   * Check if error is retryable
   */
  private isRetryable(error: any): boolean {
    return (
      error.name === 'AbortError' ||
      error.name === 'ServerError' ||
      error.code === 'ECONNRESET' ||
      error.code === 'ETIMEDOUT'
    );
  }

  /**
   * Delay helper for retries
   */
  private delay(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }
}

/**
 * Default export
 */
export default WIANLPClient;
