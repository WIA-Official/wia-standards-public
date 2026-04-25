/**
 * WIA-CORE-007: Universal Protocol - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Core Standards Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Message Types
// ============================================================================

/**
 * Message type enumeration
 */
export type MessageType = 'request' | 'response' | 'event' | 'stream';

/**
 * Message status enumeration
 */
export type MessageStatus = 'success' | 'error' | 'pending';

/**
 * Transport type enumeration
 */
export type TransportType = 'http' | 'https' | 'websocket' | 'tcp' | 'udp' | 'grpc';

/**
 * Serialization format
 */
export type SerializationFormat = 'json' | 'messagepack' | 'protobuf';

/**
 * Base message structure
 */
export interface Message {
  /** Unique message identifier (UUID v4) */
  id: string;

  /** Protocol version (semver) */
  version: string;

  /** Message type */
  type: MessageType;

  /** Method name in dot notation (optional) */
  method?: string;

  /** Message headers */
  headers?: Record<string, string>;

  /** Message payload */
  payload?: any;

  /** Message metadata */
  metadata?: MessageMetadata;

  /** Response status (for response messages) */
  status?: MessageStatus;

  /** Error information (for error responses) */
  error?: ErrorInfo;
}

/**
 * Message metadata
 */
export interface MessageMetadata {
  /** Message timestamp (ISO 8601) */
  timestamp?: string;

  /** Source identifier */
  source?: string;

  /** Destination identifier */
  destination?: string;

  /** Correlation ID for request tracing */
  correlation_id?: string;

  /** Time-to-live in milliseconds */
  ttl?: number;

  /** Message sequence number (for streams) */
  sequence?: number;

  /** Total number of messages (for streams) */
  total?: number;

  /** Whether this is the final message in a stream */
  final?: boolean;

  /** Processing time in milliseconds */
  processing_time?: number;

  /** Custom metadata fields */
  [key: string]: any;
}

/**
 * Error information
 */
export interface ErrorInfo {
  /** Error code */
  code: string;

  /** Human-readable error message */
  message: string;

  /** Additional error details */
  details?: any;

  /** Trace ID for debugging */
  trace_id?: string;

  /** Stack trace (development only) */
  stack?: string;
}

// ============================================================================
// Request/Response Types
// ============================================================================

/**
 * RPC request message
 */
export interface RequestMessage extends Message {
  type: 'request';
  method: string;
  payload?: any;
}

/**
 * RPC response message
 */
export interface ResponseMessage extends Message {
  type: 'response';
  status: MessageStatus;
  payload?: any;
  error?: ErrorInfo;
}

/**
 * Event message
 */
export interface EventMessage extends Message {
  type: 'event';
  method: string;
  payload?: any;
}

/**
 * Stream message
 */
export interface StreamMessage extends Message {
  type: 'stream';
  method: string;
  payload?: any;
  metadata: MessageMetadata & {
    sequence: number;
    total?: number;
    final: boolean;
  };
}

// ============================================================================
// Transport Configuration
// ============================================================================

/**
 * Transport configuration
 */
export interface TransportConfig {
  /** Transport type */
  type: TransportType;

  /** Server endpoint URL */
  endpoint: string;

  /** Connection timeout in milliseconds */
  timeout?: number;

  /** Enable compression */
  compression?: boolean;

  /** Compression algorithm */
  compressionAlgorithm?: 'gzip' | 'brotli';

  /** Enable TLS/SSL */
  secure?: boolean;

  /** TLS options */
  tls?: TLSOptions;

  /** Custom headers */
  headers?: Record<string, string>;

  /** Reconnect options */
  reconnect?: ReconnectOptions;
}

/**
 * TLS/SSL options
 */
export interface TLSOptions {
  /** Client certificate */
  cert?: string | Buffer;

  /** Client private key */
  key?: string | Buffer;

  /** CA certificate */
  ca?: string | Buffer;

  /** Reject unauthorized certificates */
  rejectUnauthorized?: boolean;
}

/**
 * Reconnect options
 */
export interface ReconnectOptions {
  /** Enable automatic reconnection */
  enabled: boolean;

  /** Maximum reconnection attempts */
  maxAttempts?: number;

  /** Initial delay in milliseconds */
  initialDelay?: number;

  /** Maximum delay in milliseconds */
  maxDelay?: number;

  /** Backoff strategy */
  backoff?: 'linear' | 'exponential' | 'constant';
}

// ============================================================================
// Protocol Configuration
// ============================================================================

/**
 * Universal Protocol configuration
 */
export interface ProtocolConfig {
  /** Transport configuration */
  transport: TransportConfig;

  /** Protocol version */
  version?: string;

  /** Serialization format */
  serialization?: SerializationFormat;

  /** Authentication configuration */
  auth?: AuthConfig;

  /** Middleware functions */
  middleware?: Middleware[];

  /** Default request timeout */
  defaultTimeout?: number;

  /** Enable debug logging */
  debug?: boolean;

  /** Custom options */
  [key: string]: any;
}

/**
 * Authentication configuration
 */
export interface AuthConfig {
  /** Authentication type */
  type: 'bearer' | 'apikey' | 'oauth2' | 'mtls' | 'none';

  /** Bearer token */
  token?: string;

  /** API key */
  apiKey?: string;

  /** OAuth2 configuration */
  oauth2?: OAuth2Config;

  /** Custom authentication headers */
  headers?: Record<string, string>;
}

/**
 * OAuth2 configuration
 */
export interface OAuth2Config {
  /** Client ID */
  clientId: string;

  /** Client secret */
  clientSecret?: string;

  /** Access token */
  accessToken?: string;

  /** Refresh token */
  refreshToken?: string;

  /** Token endpoint */
  tokenEndpoint?: string;

  /** Scopes */
  scopes?: string[];
}

// ============================================================================
// RPC Types
// ============================================================================

/**
 * RPC call options
 */
export interface CallOptions {
  /** Request timeout in milliseconds */
  timeout?: number;

  /** Custom headers */
  headers?: Record<string, string>;

  /** Retry policy */
  retry?: RetryPolicy;

  /** Callback for response */
  callback?: (response: ResponseMessage) => void;
}

/**
 * Retry policy
 */
export interface RetryPolicy {
  /** Maximum retry attempts */
  maxAttempts: number;

  /** Initial delay in milliseconds */
  initialDelay: number;

  /** Maximum delay in milliseconds */
  maxDelay: number;

  /** Backoff strategy */
  backoff: 'linear' | 'exponential' | 'constant';

  /** Retryable error codes */
  retryableErrors: string[];
}

/**
 * Method handler function
 */
export type MethodHandler = (payload: any, context: CallContext) => Promise<any> | any;

/**
 * Call context
 */
export interface CallContext {
  /** Original message */
  message: RequestMessage;

  /** Request headers */
  headers: Record<string, string>;

  /** Client information */
  client?: ClientInfo;

  /** Custom context data */
  [key: string]: any;
}

/**
 * Client information
 */
export interface ClientInfo {
  /** Client ID */
  id: string;

  /** Client IP address */
  ip?: string;

  /** Client user agent */
  userAgent?: string;

  /** Authentication info */
  auth?: any;
}

// ============================================================================
// Event Types
// ============================================================================

/**
 * Event handler function
 */
export type EventHandler = (event: EventMessage) => void;

/**
 * Event subscription
 */
export interface Subscription {
  /** Subscription ID */
  id: string;

  /** Channel name */
  channel: string;

  /** Event handler */
  handler: EventHandler;

  /** Unsubscribe function */
  unsubscribe: () => void;
}

/**
 * Event publish options
 */
export interface PublishOptions {
  /** Target channels */
  channels?: string[];

  /** Custom headers */
  headers?: Record<string, string>;

  /** Event priority */
  priority?: 'low' | 'normal' | 'high';

  /** Persist event */
  persist?: boolean;
}

// ============================================================================
// Stream Types
// ============================================================================

/**
 * Stream interface
 */
export interface Stream {
  /** Stream ID */
  id: string;

  /** Stream method */
  method: string;

  /** Send data chunk */
  send(data: any): Promise<void>;

  /** End stream */
  end(): Promise<void>;

  /** Listen for data */
  on(event: 'data', handler: (data: any) => void): void;

  /** Listen for end */
  on(event: 'end', handler: () => void): void;

  /** Listen for error */
  on(event: 'error', handler: (error: Error) => void): void;

  /** Remove event listener */
  off(event: string, handler: Function): void;

  /** Destroy stream */
  destroy(): void;
}

/**
 * Stream options
 */
export interface StreamOptions {
  /** Chunk size in bytes */
  chunkSize?: number;

  /** High water mark */
  highWaterMark?: number;

  /** Enable backpressure */
  backpressure?: boolean;
}

// ============================================================================
// Middleware Types
// ============================================================================

/**
 * Middleware function
 */
export type Middleware = (
  message: Message,
  next: () => Promise<Message>
) => Promise<Message>;

/**
 * Middleware context
 */
export interface MiddlewareContext {
  /** Original message */
  message: Message;

  /** Protocol instance */
  protocol: any;

  /** Custom context data */
  [key: string]: any;
}

// ============================================================================
// Server Types
// ============================================================================

/**
 * Server configuration
 */
export interface ServerConfig {
  /** Server port */
  port: number;

  /** Server host */
  host?: string;

  /** Transport type */
  transport: TransportType;

  /** TLS options */
  tls?: TLSOptions;

  /** CORS configuration */
  cors?: CORSConfig;

  /** Rate limiting */
  rateLimit?: RateLimitConfig;

  /** Custom options */
  [key: string]: any;
}

/**
 * CORS configuration
 */
export interface CORSConfig {
  /** Allowed origins */
  origins: string[] | '*';

  /** Allowed methods */
  methods?: string[];

  /** Allowed headers */
  headers?: string[];

  /** Allow credentials */
  credentials?: boolean;

  /** Max age */
  maxAge?: number;
}

/**
 * Rate limiting configuration
 */
export interface RateLimitConfig {
  /** Maximum requests per window */
  limit: number;

  /** Time window in milliseconds */
  window: number;

  /** Rate limit key function */
  keyGenerator?: (context: CallContext) => string;

  /** Rate limit exceeded handler */
  onLimitExceeded?: (context: CallContext) => void;
}

/**
 * Server options
 */
export interface ServerOptions extends ServerConfig {
  /** Server name */
  name?: string;

  /** Maximum connections */
  maxConnections?: number;

  /** Keep-alive timeout */
  keepAliveTimeout?: number;

  /** Request timeout */
  requestTimeout?: number;
}

// ============================================================================
// Protocol Negotiation
// ============================================================================

/**
 * Protocol capabilities
 */
export interface ProtocolCapabilities {
  /** Supported protocol versions */
  versions: string[];

  /** Supported transports */
  transports: TransportType[];

  /** Supported serialization formats */
  serialization: SerializationFormat[];

  /** Supported compression algorithms */
  compression?: string[];

  /** Supported encryption methods */
  encryption?: string[];

  /** Maximum message size */
  max_message_size?: number;

  /** Streaming support */
  streaming?: boolean;

  /** Bidirectional support */
  bidirectional?: boolean;

  /** Custom capabilities */
  [key: string]: any;
}

/**
 * Negotiation request
 */
export interface NegotiationRequest {
  /** Requested versions */
  versions: string[];

  /** Requested features */
  features?: string[];

  /** Client capabilities */
  capabilities?: Partial<ProtocolCapabilities>;
}

/**
 * Negotiation response
 */
export interface NegotiationResponse {
  /** Agreed version */
  version: string;

  /** Enabled features */
  features?: string[];

  /** Server capabilities */
  capabilities?: ProtocolCapabilities;
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * Protocol error codes
 */
export enum ProtocolErrorCode {
  INVALID_REQUEST = 'ERR_INVALID_REQUEST',
  UNAUTHORIZED = 'ERR_UNAUTHORIZED',
  FORBIDDEN = 'ERR_FORBIDDEN',
  NOT_FOUND = 'ERR_NOT_FOUND',
  METHOD_NOT_ALLOWED = 'ERR_METHOD_NOT_ALLOWED',
  TIMEOUT = 'ERR_TIMEOUT',
  RATE_LIMIT = 'ERR_RATE_LIMIT',
  INTERNAL = 'ERR_INTERNAL',
  NOT_IMPLEMENTED = 'ERR_NOT_IMPLEMENTED',
  SERVICE_UNAVAILABLE = 'ERR_SERVICE_UNAVAILABLE',
  GATEWAY_TIMEOUT = 'ERR_GATEWAY_TIMEOUT',
  INVALID_MESSAGE = 'ERR_INVALID_MESSAGE',
  SERIALIZATION_ERROR = 'ERR_SERIALIZATION_ERROR',
  TRANSPORT_ERROR = 'ERR_TRANSPORT_ERROR',
  CONNECTION_CLOSED = 'ERR_CONNECTION_CLOSED',
}

/**
 * Protocol error class
 */
export class ProtocolError extends Error {
  constructor(
    public code: ProtocolErrorCode | string,
    message: string,
    public details?: any,
    public traceId?: string
  ) {
    super(message);
    this.name = 'ProtocolError';
  }

  toJSON(): ErrorInfo {
    return {
      code: this.code,
      message: this.message,
      details: this.details,
      trace_id: this.traceId,
    };
  }
}

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Result type for operations that can fail
 */
export type Result<T, E = ProtocolError> =
  | { success: true; data: T }
  | { success: false; error: E };

/**
 * Async result type
 */
export type AsyncResult<T, E = ProtocolError> = Promise<Result<T, E>>;

/**
 * Connection state
 */
export enum ConnectionState {
  DISCONNECTED = 'disconnected',
  CONNECTING = 'connecting',
  CONNECTED = 'connected',
  RECONNECTING = 'reconnecting',
  CLOSING = 'closing',
  CLOSED = 'closed',
  ERROR = 'error',
}

/**
 * Connection info
 */
export interface ConnectionInfo {
  /** Connection state */
  state: ConnectionState;

  /** Endpoint */
  endpoint: string;

  /** Transport type */
  transport: TransportType;

  /** Connection established timestamp */
  connectedAt?: Date;

  /** Last activity timestamp */
  lastActivity?: Date;

  /** Ping latency in milliseconds */
  latency?: number;

  /** Number of messages sent */
  messagesSent?: number;

  /** Number of messages received */
  messagesReceived?: number;
}

/**
 * Statistics
 */
export interface ProtocolStats {
  /** Total requests sent */
  requestsSent: number;

  /** Total responses received */
  responsesReceived: number;

  /** Total events received */
  eventsReceived: number;

  /** Total errors */
  errors: number;

  /** Average latency in milliseconds */
  averageLatency: number;

  /** Success rate (0-1) */
  successRate: number;

  /** Uptime in milliseconds */
  uptime: number;
}

// ============================================================================
// Export All
// ============================================================================

export type {
  // Core
  Message,
  MessageMetadata,
  ErrorInfo,
  RequestMessage,
  ResponseMessage,
  EventMessage,
  StreamMessage,

  // Transport
  TransportConfig,
  TLSOptions,
  ReconnectOptions,

  // Protocol
  ProtocolConfig,
  AuthConfig,
  OAuth2Config,

  // RPC
  CallOptions,
  RetryPolicy,
  MethodHandler,
  CallContext,
  ClientInfo,

  // Events
  EventHandler,
  Subscription,
  PublishOptions,

  // Streams
  Stream,
  StreamOptions,

  // Middleware
  Middleware,
  MiddlewareContext,

  // Server
  ServerConfig,
  ServerOptions,
  CORSConfig,
  RateLimitConfig,

  // Negotiation
  ProtocolCapabilities,
  NegotiationRequest,
  NegotiationResponse,

  // Connection
  ConnectionInfo,

  // Statistics
  ProtocolStats,
};

export {
  ProtocolErrorCode,
  ProtocolError,
  ConnectionState,
};
