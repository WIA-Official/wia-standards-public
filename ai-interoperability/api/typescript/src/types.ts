/**
 * WIA AI Interoperability Standard - TypeScript Types
 * Version: 1.0
 *
 * Philosophy: 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * This module defines TypeScript types for the WIA AI Interoperability Standard,
 * enabling seamless communication between different AI systems.
 *
 * © 2025 SmileStory Inc. / WIA
 */

// ============================================================================
// Core Message Types (Phase 1: Data Format)
// ============================================================================

/**
 * Sender type enumeration
 */
export type SenderType = 'human' | 'ai' | 'system';

/**
 * Content type enumeration
 */
export type ContentType = 'text' | 'multimodal';

/**
 * Content part type enumeration
 */
export type ContentPartType =
  | 'text'
  | 'image'
  | 'audio'
  | 'video'
  | 'code'
  | 'tool_use'
  | 'tool_result';

/**
 * AI provider enumeration
 */
export type AIProvider = 'anthropic' | 'openai' | 'google' | 'meta' | 'custom';

/**
 * Priority levels
 */
export type Priority = 'high' | 'normal' | 'low';

/**
 * Message sender information
 */
export interface Sender {
  /** Type of sender */
  type: SenderType;
  /** Unique sender identifier */
  id: string;
  /** Display name (optional) */
  name?: string;
  /** Custom sender metadata */
  metadata?: Record<string, any>;
}

/**
 * Content part in a message
 */
export interface ContentPart {
  /** Type of content part */
  type: ContentPartType;
  /** Actual content data (string, object, base64, URL, etc.) */
  data: any;
  /** Part-specific metadata */
  metadata?: Record<string, any>;
}

/**
 * Message content
 */
export interface Content {
  /** Content type */
  type: ContentType;
  /** Content parts (supports multiple modalities) */
  parts: ContentPart[];
}

/**
 * Conversation context
 */
export interface Context {
  /** Unique conversation identifier */
  conversationId?: string;
  /** Thread identifier for branching conversations */
  threadId?: string;
  /** IDs of previous messages in sequence */
  previousMessageIds?: string[];
  /** Custom context metadata */
  metadata?: Record<string, any>;
}

/**
 * AI model information
 */
export interface Model {
  /** AI provider name */
  provider: AIProvider | string;
  /** Specific model identifier */
  modelId: string;
  /** Model version */
  version?: string;
  /** List of model capabilities */
  capabilities?: string[];
}

/**
 * User intent information
 */
export interface Intent {
  /** Primary intent classification */
  primary: string;
  /** Confidence score (0-1) */
  confidence?: number;
  /** Intent-specific parameters */
  parameters?: Record<string, any>;
}

/**
 * Message metadata
 */
export interface MessageMetadata {
  /** Message language */
  language?: string;
  /** Message priority */
  priority?: Priority;
  /** Message tags */
  tags?: string[];
  /** Custom metadata */
  custom?: Record<string, any>;
  /** Additional fields */
  [key: string]: any;
}

/**
 * Core AI Message structure
 * This is the fundamental data structure for AI interoperability
 */
export interface AIMessage {
  /** WIA standard version */
  version: string;
  /** Unique message identifier */
  messageId: string;
  /** ISO 8601 timestamp */
  timestamp: string;
  /** Message sender information */
  sender: Sender;
  /** Message content */
  content: Content;
  /** Conversation context (optional) */
  context?: Context;
  /** AI model information (optional, for AI senders) */
  model?: Model;
  /** Interpreted user intent (optional) */
  intent?: Intent;
  /** Additional metadata */
  metadata?: MessageMetadata;
}

// ============================================================================
// API Types (Phase 2: API)
// ============================================================================

/**
 * Message sending options
 */
export interface SendOptions {
  /** Enable streaming response */
  stream?: boolean;
  /** Maximum tokens to generate */
  maxTokens?: number;
  /** Sampling temperature (0-1) */
  temperature?: number;
  /** Sequences to stop generation */
  stopSequences?: string[];
  /** Nucleus sampling parameter */
  topP?: number;
  /** Top-K sampling parameter */
  topK?: number;
}

/**
 * Token usage information
 */
export interface Usage {
  /** Input tokens consumed */
  inputTokens: number;
  /** Output tokens generated */
  outputTokens: number;
  /** Total tokens (input + output) */
  totalTokens: number;
}

/**
 * API response metadata
 */
export interface ResponseMetadata {
  /** Unique request identifier */
  requestId: string;
  /** Response timestamp */
  timestamp: string;
  /** API version */
  version: string;
}

/**
 * Error details
 */
export interface ErrorDetails {
  /** Error code */
  code: string;
  /** Error message */
  message: string;
  /** Additional error details */
  details?: Record<string, any>;
}

/**
 * Standard API response
 */
export interface APIResponse<T = any> {
  /** Success status */
  success: boolean;
  /** Response data */
  data: T | null;
  /** Response metadata */
  metadata: ResponseMetadata;
  /** Error information (if failed) */
  error?: ErrorDetails;
}

/**
 * Message send response data
 */
export interface MessageResponse {
  /** AI response message */
  message: AIMessage;
  /** Token usage information */
  usage?: Usage;
}

/**
 * Pagination information
 */
export interface Pagination {
  /** Total number of items */
  total?: number;
  /** Number of items per page */
  limit: number;
  /** Current offset */
  offset?: number;
  /** Has more items */
  hasMore: boolean;
  /** Next cursor (for cursor-based pagination) */
  nextCursor?: string;
  /** Previous cursor */
  previousCursor?: string;
}

/**
 * Conversation object
 */
export interface Conversation {
  /** Unique conversation identifier */
  conversationId: string;
  /** Conversation title */
  title?: string;
  /** Creation timestamp */
  createdAt: string;
  /** Last update timestamp */
  updatedAt: string;
  /** Number of messages */
  messageCount?: number;
  /** Conversation metadata */
  metadata?: Record<string, any>;
}

/**
 * Model capabilities
 */
export interface ModelCapabilities {
  /** Supported modalities */
  modalities: string[];
  /** Maximum output tokens */
  maxTokens: number;
  /** Supported languages */
  supportedLanguages: string[];
  /** Feature list */
  features: string[];
  /** Streaming support */
  streaming: boolean;
  /** Context window size */
  contextWindow: number;
}

/**
 * Model pricing information
 */
export interface ModelPricing {
  /** Input token price */
  inputTokenPrice: number;
  /** Output token price */
  outputTokenPrice: number;
  /** Currency */
  currency: string;
  /** Price unit */
  unit: string;
}

/**
 * Model information
 */
export interface ModelInfo {
  /** Model identifier */
  modelId: string;
  /** Provider name */
  provider: string;
  /** Model version */
  version?: string;
  /** Display name */
  name?: string;
  /** Model description */
  description?: string;
  /** Model capabilities */
  capabilities?: ModelCapabilities;
  /** Pricing information */
  pricing?: ModelPricing;
}

// ============================================================================
// Protocol Types (Phase 3: Protocol)
// ============================================================================

/**
 * WebSocket frame types
 */
export type FrameType = 'message' | 'event' | 'control';

/**
 * Event types
 */
export type EventType = 'typing' | 'thinking' | 'tool_use' | 'error' | 'connection';

/**
 * Control commands
 */
export type ControlCommand =
  | 'ping'
  | 'pong'
  | 'subscribe'
  | 'unsubscribe'
  | 'close'
  | 'handshake'
  | 'handshake_ack';

/**
 * Delivery mode
 */
export type DeliveryMode = 'immediate' | 'queued';

/**
 * WebSocket frame envelope
 */
export interface Frame<T = any> {
  /** Frame type */
  type: FrameType;
  /** Unique frame identifier */
  id: string;
  /** Frame timestamp */
  timestamp: string;
  /** Frame payload */
  payload: T;
}

/**
 * Message frame payload
 */
export interface MessageFramePayload {
  /** AI message */
  message: AIMessage;
  /** Delivery mode */
  deliveryMode?: DeliveryMode;
  /** Message priority */
  priority?: Priority;
}

/**
 * Event frame payload
 */
export interface EventFramePayload {
  /** Event type */
  eventType: EventType;
  /** Event data */
  data: Record<string, any>;
}

/**
 * Control frame payload
 */
export interface ControlFramePayload {
  /** Control command */
  command: ControlCommand;
  /** Command parameters */
  parameters?: Record<string, any>;
}

/**
 * Stream event types
 */
export type StreamEventType =
  | 'message_start'
  | 'content_block_start'
  | 'content_block_delta'
  | 'content_block_stop'
  | 'message_stop';

/**
 * Stream event
 */
export interface StreamEvent {
  /** Event type */
  event: StreamEventType;
  /** Event data */
  data: any;
}

// ============================================================================
// Integration Types (Phase 4: Integration)
// ============================================================================

/**
 * Platform adapter configuration
 */
export interface AdapterConfig {
  /** AI platform */
  platform: AIProvider | string;
  /** API key */
  apiKey: string;
  /** Base URL (optional) */
  baseURL?: string;
  /** API version (optional) */
  version?: string;
  /** Additional options */
  options?: {
    /** Request timeout in milliseconds */
    timeout?: number;
    /** Number of retries */
    retries?: number;
    /** Default model to use */
    defaultModel?: string;
    /** Rate limit configuration */
    rateLimits?: RateLimitConfig;
  };
}

/**
 * Rate limit configuration
 */
export interface RateLimitConfig {
  /** Requests per minute */
  requestsPerMinute?: number;
  /** Tokens per minute */
  tokensPerMinute?: number;
  /** Concurrent requests */
  concurrentRequests?: number;
}

/**
 * Platform capabilities
 */
export interface PlatformCapabilities {
  /** Text support */
  textSupport: boolean;
  /** Image support */
  imageSupport: boolean;
  /** Audio support */
  audioSupport: boolean;
  /** Video support */
  videoSupport: boolean;
  /** Tool use support */
  toolUseSupport: boolean;
  /** Streaming support */
  streamingSupport: boolean;
  /** Maximum context window */
  maxContextWindow?: number;
  /** Supported models */
  supportedModels?: string[];
}

/**
 * Validation error
 */
export interface ValidationError {
  /** Field name */
  field: string;
  /** Error message */
  message: string;
  /** Error code */
  code?: string;
}

/**
 * Validation warning
 */
export interface ValidationWarning {
  /** Field name */
  field: string;
  /** Warning message */
  message: string;
}

/**
 * Validation result
 */
export interface ValidationResult {
  /** Is message valid */
  valid: boolean;
  /** Validation errors */
  errors?: ValidationError[];
  /** Validation warnings */
  warnings?: ValidationWarning[];
  /** Platform compatibility */
  compatibility: PlatformCapabilities;
}

/**
 * Stream options
 */
export interface StreamOptions extends SendOptions {
  /** Always true for streaming */
  stream: true;
  /** Callback for each chunk */
  onChunk?: (chunk: AIMessage) => void;
  /** Callback on stream complete */
  onComplete?: (message: AIMessage) => void;
  /** Callback on error */
  onError?: (error: Error) => void;
}

// ============================================================================
// Client Configuration
// ============================================================================

/**
 * WIA Client configuration
 */
export interface WIAClientConfig {
  /** API key for authentication */
  apiKey: string;
  /** Base URL (default: https://api.wia-ai-interop.org/v1) */
  baseURL?: string;
  /** Default model to use */
  defaultModel?: string;
  /** Request timeout in milliseconds */
  timeout?: number;
  /** Enable debug logging */
  debug?: boolean;
  /** Custom headers */
  headers?: Record<string, string>;
}

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Create a simple text message
 */
export interface CreateMessageParams {
  /** Message text content */
  content: string;
  /** Sender ID */
  senderId: string;
  /** Sender name (optional) */
  senderName?: string;
  /** Sender type (default: 'human') */
  senderType?: SenderType;
  /** Conversation ID (optional) */
  conversationId?: string;
  /** Additional metadata */
  metadata?: MessageMetadata;
}

/**
 * List messages parameters
 */
export interface ListMessagesParams {
  /** Conversation ID */
  conversationId: string;
  /** Number of messages to return */
  limit?: number;
  /** Paginate before this message ID */
  before?: string;
  /** Paginate after this message ID */
  after?: string;
  /** Sort order */
  order?: 'asc' | 'desc';
}

/**
 * Create conversation parameters
 */
export interface CreateConversationParams {
  /** Conversation title */
  title?: string;
  /** Conversation metadata */
  metadata?: Record<string, any>;
}

/**
 * List conversations parameters
 */
export interface ListConversationsParams {
  /** Number of conversations to return */
  limit?: number;
  /** Pagination offset */
  offset?: number;
  /** Sort order */
  order?: 'asc' | 'desc';
}

// ============================================================================
// Export all types
// ============================================================================

export type {
  // Core types are exported above
};

/**
 * Type guard to check if a message is valid
 */
export function isAIMessage(obj: any): obj is AIMessage {
  return (
    typeof obj === 'object' &&
    obj !== null &&
    typeof obj.version === 'string' &&
    typeof obj.messageId === 'string' &&
    typeof obj.timestamp === 'string' &&
    typeof obj.sender === 'object' &&
    typeof obj.content === 'object' &&
    Array.isArray(obj.content.parts)
  );
}

/**
 * Type guard to check if a frame is valid
 */
export function isFrame(obj: any): obj is Frame {
  return (
    typeof obj === 'object' &&
    obj !== null &&
    typeof obj.type === 'string' &&
    typeof obj.id === 'string' &&
    typeof obj.timestamp === 'string' &&
    typeof obj.payload === 'object'
  );
}
