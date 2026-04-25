/**
 * WIA-AI-023 NLP Standard - TypeScript Type Definitions
 * Version: 1.0.0
 *
 * 弘益人間 (Hongik Ingan) - Benefit All Humanity
 * © 2025 WIA - World Certification Industry Association
 */

// ============================================================================
// Base Types
// ============================================================================

export interface WIAStandard {
  standard: 'WIA-AI-023';
  version: string;
}

export interface RequestMetadata {
  request_id?: string;
  timestamp?: string;
  client_info?: Record<string, any>;
}

export interface ResponseMetadata extends RequestMetadata {
  processing_time_ms?: number;
  model?: string;
  confidence?: number;
}

export interface Status {
  code: number;
  message: string;
  details?: string;
}

// ============================================================================
// Base Request/Response
// ============================================================================

export interface BaseRequest extends WIAStandard {
  task: string;
  input: {
    text: string;
    language: string;
    encoding?: string;
  };
  config?: Record<string, any>;
  metadata?: RequestMetadata;
}

export interface BaseResponse extends WIAStandard {
  output: Record<string, any>;
  metadata: ResponseMetadata;
  status: Status;
}

// ============================================================================
// Tokenization
// ============================================================================

export interface TokenizationConfig {
  method?: 'word' | 'subword' | 'character';
  preserve_case?: boolean;
  include_offsets?: boolean;
}

export interface TokenOffset {
  start: number;
  end: number;
}

export interface TokenizationRequest extends BaseRequest {
  task: 'tokenization';
  config?: TokenizationConfig;
}

export interface TokenizationResponse extends BaseResponse {
  output: {
    tokens: string[];
    token_count: number;
    offsets?: TokenOffset[];
  };
}

// ============================================================================
// Named Entity Recognition
// ============================================================================

export type EntityType =
  | 'PERSON'
  | 'ORGANIZATION'
  | 'LOCATION'
  | 'DATE'
  | 'TIME'
  | 'MONEY'
  | 'PRODUCT'
  | 'EVENT'
  | 'MISC';

export interface Entity {
  text: string;
  type: EntityType;
  start: number;
  end: number;
  confidence: number;
}

export interface NERConfig {
  entity_types?: EntityType[];
  aggregation?: 'simple' | 'first' | 'max';
  threshold?: number;
}

export interface NERRequest extends BaseRequest {
  task: 'named_entity_recognition';
  config?: NERConfig;
}

export interface NERResponse extends BaseResponse {
  output: {
    entities: Entity[];
    entity_count: number;
  };
}

// ============================================================================
// Sentiment Analysis
// ============================================================================

export type Sentiment = 'positive' | 'negative' | 'neutral';

export interface SentimentScores {
  positive: number;
  neutral: number;
  negative: number;
}

export interface SentimentConfig {
  granularity?: 'document' | 'sentence';
  return_scores?: boolean;
}

export interface SentimentRequest extends BaseRequest {
  task: 'sentiment_analysis';
  config?: SentimentConfig;
}

export interface SentimentResponse extends BaseResponse {
  output: {
    sentiment: Sentiment;
    confidence: number;
    scores?: SentimentScores;
    polarity?: number;
    subjectivity?: number;
  };
}

// ============================================================================
// Text Classification
// ============================================================================

export interface ClassificationPrediction {
  category: string;
  confidence: number;
  rank: number;
}

export interface ClassificationConfig {
  categories?: string[];
  top_k?: number;
  threshold?: number;
}

export interface ClassificationRequest extends BaseRequest {
  task: 'text_classification';
  config?: ClassificationConfig;
}

export interface ClassificationResponse extends BaseResponse {
  output: {
    predictions: ClassificationPrediction[];
    primary_category: string;
  };
}

// ============================================================================
// Text Generation
// ============================================================================

export interface GenerationConfig {
  max_length?: number;
  min_length?: number;
  temperature?: number;
  top_p?: number;
  top_k?: number;
  num_return_sequences?: number;
  do_sample?: boolean;
  stop_sequences?: string[];
}

export interface GenerationRequest extends BaseRequest {
  task: 'text_generation';
  input: {
    prompt: string;
    language: string;
  };
  config?: GenerationConfig;
}

export interface GenerationResponse extends BaseResponse {
  output: {
    generated_texts: string[];
    generation_count: number;
  };
}

// ============================================================================
// Summarization
// ============================================================================

export interface SummarizationConfig {
  method?: 'abstractive' | 'extractive';
  max_length?: number;
  min_length?: number;
  num_sentences?: number;
}

export interface SummarizationRequest extends BaseRequest {
  task: 'summarization';
  config?: SummarizationConfig;
}

export interface SummarizationResponse extends BaseResponse {
  output: {
    summary: string;
    summary_length: number;
    compression_ratio: number;
    key_points?: string[];
  };
}

// ============================================================================
// Batch Processing
// ============================================================================

export interface BatchRequest {
  requests: BaseRequest[];
}

export interface BatchResponseItem {
  index: number;
  status: 'success' | 'error';
  output?: Record<string, any>;
  error?: Status;
}

export interface BatchResponse {
  responses: BatchResponseItem[];
}

// ============================================================================
// Service Info
// ============================================================================

export interface ServiceInfo extends WIAStandard {
  service: {
    name: string;
    vendor: string;
    api_version: string;
  };
  supported_tasks: string[];
  supported_languages: string[];
  rate_limits?: {
    requests_per_hour: number;
    requests_per_day: number;
  };
}

export interface HealthStatus {
  status: 'healthy' | 'degraded' | 'unhealthy';
  timestamp: string;
  version: string;
  uptime_seconds?: number;
}

// ============================================================================
// Client Configuration
// ============================================================================

export interface ClientConfig {
  baseURL: string;
  apiKey?: string;
  bearerToken?: string;
  timeout?: number;
  retries?: number;
  headers?: Record<string, string>;
}

// ============================================================================
// Error Types
// ============================================================================

export class WIAError extends Error {
  constructor(
    message: string,
    public code: number,
    public details?: string,
    public requestId?: string
  ) {
    super(message);
    this.name = 'WIAError';
  }
}

export class ValidationError extends WIAError {
  constructor(message: string, details?: string) {
    super(message, 400, details);
    this.name = 'ValidationError';
  }
}

export class AuthenticationError extends WIAError {
  constructor(message: string, details?: string) {
    super(message, 401, details);
    this.name = 'AuthenticationError';
  }
}

export class RateLimitError extends WIAError {
  constructor(message: string, public retryAfter?: number) {
    super(message, 429, `Retry after ${retryAfter} seconds`);
    this.name = 'RateLimitError';
  }
}

export class ServerError extends WIAError {
  constructor(message: string, details?: string) {
    super(message, 500, details);
    this.name = 'ServerError';
  }
}
