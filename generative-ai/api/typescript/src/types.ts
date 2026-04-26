/**
 * WIA-AI-026 Generative AI TypeScript SDK - Type Definitions
 *
 * This file contains all TypeScript type definitions for the WIA Generative AI SDK.
 *
 * @package @wia/generative-ai
 * @version 1.0.0
 * @license MIT
 */

// ============================================================================
// Common Types
// ============================================================================

export type ModelType = 'text' | 'image' | 'audio' | 'video' | 'multimodal';

export type GenerationStatus = 'pending' | 'processing' | 'completed' | 'failed';

export interface APIConfig {
  apiKey: string;
  baseUrl?: string;
  timeout?: number;
  retries?: number;
  headers?: Record<string, string>;
}

export interface GenerationMetadata {
  id: string;
  model: string;
  timestamp: number;
  latency: number;
  status: GenerationStatus;
}

// ============================================================================
// Text Generation Types
// ============================================================================

export interface TextGenerationRequest {
  prompt: string;
  model?: string;
  maxTokens?: number;
  temperature?: number;
  topP?: number;
  topK?: number;
  stopSequences?: string[];
  presencePenalty?: number;
  frequencyPenalty?: number;
  systemPrompt?: string;
  conversationHistory?: Message[];
  stream?: boolean;
}

export interface Message {
  role: 'system' | 'user' | 'assistant';
  content: string;
  name?: string;
}

export interface TextGenerationResponse {
  id: string;
  model: string;
  text: string;
  finishReason: 'stop' | 'length' | 'content_filter' | 'error';
  usage: TokenUsage;
  metadata: GenerationMetadata;
}

export interface TokenUsage {
  promptTokens: number;
  completionTokens: number;
  totalTokens: number;
}

export interface StreamChunk {
  id: string;
  delta: string;
  finishReason?: string;
}

// ============================================================================
// Image Generation Types
// ============================================================================

export interface ImageGenerationRequest {
  prompt: string;
  negativePrompt?: string;
  model?: string;
  width?: number;
  height?: number;
  numImages?: number;
  steps?: number;
  guidanceScale?: number;
  seed?: number;
  scheduler?: 'ddpm' | 'ddim' | 'euler' | 'euler_a' | 'lms';
  style?: string;
  lora?: LoRAConfig;
  controlNet?: ControlNetConfig;
}

export interface LoRAConfig {
  model: string;
  weight: number;
}

export interface ControlNetConfig {
  type: 'canny' | 'depth' | 'pose' | 'normal' | 'seg';
  image: string;
  weight: number;
}

export interface ImageGenerationResponse {
  id: string;
  images: ImageOutput[];
  metadata: GenerationMetadata & {
    seed: number;
  };
}

export interface ImageOutput {
  url: string;
  base64?: string;
  width: number;
  height: number;
  format: 'png' | 'jpg' | 'webp';
}

export interface ImageEditRequest {
  image: string;
  mask?: string;
  prompt: string;
  negativePrompt?: string;
  strength?: number;
  guidanceScale?: number;
}

// ============================================================================
// Audio Generation Types
// ============================================================================

export interface AudioGenerationRequest {
  text?: string;
  prompt?: string;
  type: 'speech' | 'music' | 'sound_effect';
  voice?: string;
  language?: string;
  duration?: number;
  temperature?: number;
  format?: 'mp3' | 'wav' | 'ogg' | 'flac';
  sampleRate?: number;
}

export interface AudioGenerationResponse {
  id: string;
  audioUrl: string;
  duration: number;
  format: string;
  metadata: GenerationMetadata & {
    sampleRate: number;
    bitrate: number;
  };
}

// ============================================================================
// Video Generation Types (Phase 2+)
// ============================================================================

export interface VideoGenerationRequest {
  prompt: string;
  duration?: number;
  fps?: number;
  width?: number;
  height?: number;
  style?: string;
  seed?: number;
}

export interface VideoGenerationResponse {
  id: string;
  videoUrl: string;
  thumbnailUrl: string;
  duration: number;
  fps: number;
  width: number;
  height: number;
  metadata: GenerationMetadata;
}

// ============================================================================
// Multimodal Types (Phase 2+)
// ============================================================================

export type ModalityInput = TextInput | ImageInput | AudioInput;

export interface TextInput {
  type: 'text';
  content: string;
}

export interface ImageInput {
  type: 'image';
  url: string;
}

export interface AudioInput {
  type: 'audio';
  url: string;
}

export interface MultimodalRequest {
  inputs: ModalityInput[];
  outputModality: ModelType;
  parameters?: Record<string, any>;
}

export interface MultimodalResponse {
  id: string;
  output: any;
  metadata: GenerationMetadata;
}

// ============================================================================
// Safety and Moderation Types
// ============================================================================

export interface SafetyConfig {
  enableInputFilter: boolean;
  enableOutputFilter: boolean;
  customRules?: SafetyRule[];
}

export interface SafetyRule {
  category: string;
  threshold: number;
  action: 'allow' | 'flag' | 'block';
}

export interface SafetyResult {
  safe: boolean;
  categories: SafetyCategory[];
  overallScore: number;
}

export interface SafetyCategory {
  category: 'violence' | 'nsfw' | 'hate_speech' | 'self_harm' | 'child_safety' | 'pii';
  score: number;
  flagged: boolean;
}

// ============================================================================
// Fine-Tuning Types (Phase 2+)
// ============================================================================

export interface FineTuneRequest {
  baseModel: string;
  trainingData: TrainingExample[];
  validationData?: TrainingExample[];
  hyperparameters?: FineTuneHyperparameters;
  name?: string;
}

export interface TrainingExample {
  input: string;
  output: string;
}

export interface FineTuneHyperparameters {
  epochs?: number;
  batchSize?: number;
  learningRate?: number;
  loraRank?: number;
  loraAlpha?: number;
}

export interface FineTuneJob {
  id: string;
  status: 'pending' | 'running' | 'completed' | 'failed';
  progress: number;
  modelId?: string;
  metrics?: TrainingMetrics;
  createdAt: number;
  completedAt?: number;
}

export interface TrainingMetrics {
  loss: number;
  accuracy: number;
  validationLoss?: number;
  validationAccuracy?: number;
}

// ============================================================================
// Batch Processing Types
// ============================================================================

export interface BatchRequest {
  requests: (TextGenerationRequest | ImageGenerationRequest | AudioGenerationRequest)[];
  parallel?: boolean;
  maxConcurrency?: number;
}

export interface BatchResponse {
  id: string;
  results: any[];
  completed: number;
  failed: number;
  totalTime: number;
}

// ============================================================================
// Webhook Types
// ============================================================================

export interface WebhookConfig {
  url: string;
  events: WebhookEvent[];
  secret?: string;
}

export type WebhookEvent =
  | 'generation.completed'
  | 'generation.failed'
  | 'finetune.completed'
  | 'finetune.failed'
  | 'content.flagged';

export interface WebhookPayload {
  event: WebhookEvent;
  data: any;
  timestamp: number;
  signature: string;
}

// ============================================================================
// Error Types
// ============================================================================

export class GenerativeAIError extends Error {
  constructor(
    message: string,
    public code: string,
    public statusCode?: number,
    public details?: any
  ) {
    super(message);
    this.name = 'GenerativeAIError';
  }
}

export interface ErrorResponse {
  error: {
    code: string;
    message: string;
    details?: any;
  };
}

// ============================================================================
// Analytics Types
// ============================================================================

export interface UsageStatistics {
  period: 'day' | 'week' | 'month';
  textGenerations: number;
  imageGenerations: number;
  audioGenerations: number;
  totalTokens: number;
  totalCost: number;
  averageLatency: number;
}

export interface ModelPerformance {
  model: string;
  requestCount: number;
  successRate: number;
  averageLatency: number;
  p95Latency: number;
  p99Latency: number;
}

// ============================================================================
// Utility Types
// ============================================================================

export type DeepPartial<T> = {
  [P in keyof T]?: T[P] extends object ? DeepPartial<T[P]> : T[P];
};

export type Awaitable<T> = T | Promise<T>;

// ============================================================================
// Re-exports
// ============================================================================

export * from './types';
