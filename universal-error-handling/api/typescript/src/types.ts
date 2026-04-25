/**
 * WIA-CORE-010: Universal Error Handling - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Core Standards Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Error Types
// ============================================================================

/**
 * Error severity levels
 */
export enum ErrorSeverity {
  CRITICAL = 5,  // System failure, immediate action required
  ERROR = 4,     // Operation failed, user intervention needed
  WARNING = 3,   // Potential issue, degraded functionality
  INFO = 2,      // Informational, no action required
  DEBUG = 1,     // Debugging information
}

/**
 * Error categories
 */
export enum ErrorCategory {
  AUTH = 'AUTH',               // Authentication/Authorization
  VALIDATION = 'VALIDATION',   // Input validation
  NETWORK = 'NETWORK',         // Network/Communication
  DATABASE = 'DATABASE',       // Data persistence
  RESOURCE = 'RESOURCE',       // Resource management
  PERMISSION = 'PERMISSION',   // Access control
  SYSTEM = 'SYSTEM',           // System failures
  EXTERNAL = 'EXTERNAL',       // External service errors
  BUSINESS = 'BUSINESS',       // Business logic
  UNKNOWN = 'UNKNOWN',         // Unclassified errors
}

/**
 * Base WIA Error interface
 */
export interface WIAError {
  /** WIA error code (e.g., WIA-CORE-AUTH-001) */
  code: string;

  /** Human-readable error message */
  message: string;

  /** Error severity level */
  severity: ErrorSeverity;

  /** Timestamp when error occurred */
  timestamp: Date;

  /** Additional error context */
  context?: ErrorContext;

  /** Stack trace */
  stack?: string[];

  /** Original error that caused this error */
  cause?: Error | WIAError;

  /** Recovery information and suggestions */
  recovery?: RecoveryInfo;

  /** Custom metadata */
  metadata?: Record<string, unknown>;
}

/**
 * Error context with request and system information
 */
export interface ErrorContext {
  // Request context
  requestId?: string;
  userId?: string;
  sessionId?: string;
  ipAddress?: string;
  userAgent?: string;

  // System context
  service?: string;
  environment?: string;
  version?: string;
  hostname?: string;
  processId?: number;

  // Operation context
  operation?: string;
  resource?: string;
  resourceId?: string;
  method?: string;
  parameters?: Record<string, unknown>;

  // Location context
  file?: string;
  line?: number;
  column?: number;
  function?: string;

  // Error-specific data
  details?: Record<string, unknown>;

  // Custom fields
  [key: string]: unknown;
}

/**
 * Recovery information and strategies
 */
export interface RecoveryInfo {
  /** Can this error be automatically retried? */
  retryable: boolean;

  /** Retry configuration */
  retry?: RetryConfig;

  /** User-facing suggestions */
  suggestions?: string[];

  /** Programmatic recovery strategy */
  recovery?: RecoveryStrategy;

  /** Documentation URL for this error */
  documentationUrl?: string;

  /** Support contact information */
  supportContact?: string;

  /** Estimated time to resolution */
  estimatedResolutionTime?: string;
}

/**
 * Retry configuration
 */
export interface RetryConfig {
  /** Maximum number of retry attempts */
  maxAttempts: number;

  /** Backoff strategy */
  backoff: 'linear' | 'exponential' | 'fixed';

  /** Initial delay in milliseconds */
  delayMs: number;

  /** Maximum delay in milliseconds */
  maxDelayMs?: number;

  /** Add random jitter to prevent thundering herd */
  jitter?: boolean;

  /** Condition to determine if retry should occur */
  condition?: (error: WIAError, attempt: number) => boolean;

  /** Callback on each retry attempt */
  onRetry?: (attempt: number, error: WIAError) => void;
}

/**
 * Recovery strategy
 */
export interface RecoveryStrategy {
  /** Recovery strategy type */
  strategy: 'retry' | 'fallback' | 'ignore' | 'escalate' | 'degrade';

  /** Fallback value to use */
  fallbackValue?: unknown;

  /** Fallback function to execute */
  fallbackFunction?: () => Promise<unknown>;

  /** Degradation level if using degrade strategy */
  degradationLevel?: 'reduced' | 'minimal' | 'offline';

  /** Escalation target if using escalate strategy */
  escalationTarget?: string;
}

// ============================================================================
// Error Creation and Configuration
// ============================================================================

/**
 * Configuration for creating an error
 */
export interface ErrorConfig {
  /** Error code */
  code: string;

  /** Error message */
  message: string;

  /** Severity level (defaults to ERROR) */
  severity?: ErrorSeverity;

  /** Error context */
  context?: Partial<ErrorContext>;

  /** Original error */
  cause?: Error | WIAError;

  /** Recovery information */
  recovery?: RecoveryInfo;

  /** Custom metadata */
  metadata?: Record<string, unknown>;

  /** Enable automatic sanitization of sensitive data */
  sanitize?: boolean;
}

/**
 * Error definition in the registry
 */
export interface ErrorDefinition {
  /** Error code */
  code: string;

  /** Error category */
  category: ErrorCategory;

  /** Default severity */
  severity: ErrorSeverity;

  /** Description */
  description: string;

  /** Is this error recoverable? */
  recoverable: boolean;

  /** HTTP status code mapping */
  httpStatus?: number;

  /** Example scenarios */
  examples?: string[];

  /** Version when introduced */
  since?: string;

  /** Tags for categorization */
  tags?: string[];
}

// ============================================================================
// Error Handling
// ============================================================================

/**
 * Error handler options
 */
export interface ErrorHandlerOptions {
  /** Minimum log level */
  logLevel?: ErrorSeverity;

  /** Report errors to monitoring system */
  reportErrors?: boolean;

  /** Include stack trace in logs */
  includeStackTrace?: boolean;

  /** Sanitize sensitive data */
  sanitizeSensitiveData?: boolean;

  /** Custom error handler */
  onError?: (error: WIAError) => void | Promise<void>;

  /** Rate limiting configuration */
  rateLimit?: RateLimitConfig;

  /** Integration configurations */
  integrations?: IntegrationConfig;
}

/**
 * Rate limiting configuration
 */
export interface RateLimitConfig {
  /** Maximum errors per window */
  maxErrors: number;

  /** Time window in milliseconds */
  window: number;

  /** Action to take when limit exceeded */
  action: 'throttle' | 'block' | 'alert';

  /** Callback when rate limit exceeded */
  onLimitExceeded?: (errors: WIAError[]) => void;
}

/**
 * Integration configuration for monitoring systems
 */
export interface IntegrationConfig {
  /** Sentry configuration */
  sentry?: {
    dsn: string;
    environment?: string;
    release?: string;
  };

  /** Datadog configuration */
  datadog?: {
    apiKey: string;
    service?: string;
    env?: string;
  };

  /** Custom webhook */
  webhook?: {
    url: string;
    headers?: Record<string, string>;
  };
}

/**
 * Handle error options
 */
export interface HandleErrorOptions {
  /** Report error to monitoring */
  report?: boolean;

  /** Attempt recovery */
  recover?: boolean;

  /** Recovery configuration */
  recovery?: RecoveryConfig;

  /** Log the error */
  log?: boolean;

  /** Transform error for API response */
  transform?: boolean;
}

/**
 * Recovery configuration
 */
export interface RecoveryConfig {
  /** Retry configuration */
  retry?: RetryConfig;

  /** Fallback configuration */
  fallback?: FallbackConfig;

  /** Circuit breaker configuration */
  circuitBreaker?: CircuitBreakerConfig;
}

/**
 * Fallback configuration
 */
export interface FallbackConfig {
  /** Fallback type */
  type: 'value' | 'function' | 'cache';

  /** Static value to return */
  value?: unknown;

  /** Function to execute */
  function?: () => Promise<unknown>;

  /** Cache key for cached fallback */
  cacheKey?: string;

  /** Cache TTL in seconds */
  cacheTTL?: number;
}

/**
 * Circuit breaker configuration
 */
export interface CircuitBreakerConfig {
  /** Number of failures before opening circuit */
  failureThreshold: number;

  /** Time to wait before trying to close circuit (ms) */
  resetTimeout: number;

  /** Number of requests to test in half-open state */
  halfOpenRequests: number;

  /** Callback when circuit opens */
  onOpen?: () => void;

  /** Callback when circuit closes */
  onClose?: () => void;

  /** Callback when circuit is half-open */
  onHalfOpen?: () => void;
}

/**
 * Handled error result
 */
export interface HandledError<T = unknown> {
  /** Was the error successfully recovered? */
  recovered: boolean;

  /** Result if recovered */
  result?: T;

  /** Original or transformed error */
  error: WIAError;

  /** Number of retry attempts made */
  attempts?: number;

  /** Recovery strategy used */
  strategy?: string;
}

// ============================================================================
// Error Propagation
// ============================================================================

/**
 * Service error with propagation information
 */
export interface ServiceError extends WIAError {
  /** Service that originated the error */
  originService: string;

  /** Environment where error occurred */
  originEnvironment: string;

  /** Original request ID */
  originRequestId: string;

  /** Propagation path through services */
  propagationPath: string[];

  /** Number of service hops */
  hops: number;

  /** Distributed tracing ID */
  traceId?: string;

  /** Span ID for tracing */
  spanId?: string;
}

/**
 * Error transformation options
 */
export interface ErrorTransformOptions {
  /** Remove sensitive information */
  sanitize?: boolean;

  /** Include stack trace */
  includeStack?: boolean;

  /** Include full context */
  includeContext?: boolean;

  /** Map to HTTP status */
  includeHttpStatus?: boolean;

  /** Target format */
  format?: 'api' | 'log' | 'user' | 'internal';
}

// ============================================================================
// Error Reporting and Logging
// ============================================================================

/**
 * Error log entry
 */
export interface ErrorLog {
  /** Timestamp */
  timestamp: Date;

  /** Log level */
  level: 'CRITICAL' | 'ERROR' | 'WARNING' | 'INFO' | 'DEBUG';

  /** Error object */
  error: WIAError;

  /** Environment */
  environment: string;

  /** Service name */
  service: string;

  /** Service version */
  version: string;

  /** Trace ID */
  traceId?: string;

  /** Span ID */
  spanId?: string;

  /** Tags for categorization */
  tags?: string[];

  /** Fingerprint for error grouping */
  fingerprint?: string;
}

/**
 * Error metrics
 */
export interface ErrorMetrics {
  /** Total error count */
  totalErrors: number;

  /** Errors by code */
  errorsByCode: Record<string, number>;

  /** Errors by severity */
  errorsBySeverity: Record<string, number>;

  /** Errors by category */
  errorsByCategory: Record<string, number>;

  /** Error rate (errors per minute) */
  errorRate: number;

  /** Average resolution time */
  avgResolutionTime?: number;

  /** Most common errors */
  topErrors: Array<{
    code: string;
    count: number;
    percentage: number;
  }>;
}

/**
 * Error aggregation configuration
 */
export interface ErrorAggregationConfig {
  /** Group errors by */
  groupBy: 'code' | 'message' | 'fingerprint' | 'service' | 'user';

  /** Time window configuration */
  window: {
    /** Window size in milliseconds */
    size: number;
    /** Slide interval in milliseconds */
    slide: number;
  };

  /** Thresholds for alerting */
  thresholds: {
    /** Error count threshold */
    count: number;
    /** Error rate threshold (errors per second) */
    rate: number;
    /** Unique error types threshold */
    unique: number;
  };

  /** Callback when threshold exceeded */
  onThresholdExceeded?: (errors: WIAError[]) => void;
}

// ============================================================================
// User-Facing Error Messages
// ============================================================================

/**
 * User-facing error message
 */
export interface UserErrorMessage {
  /** Localized messages */
  messages: Record<string, string>;

  /** Message template */
  template?: string;

  /** Template variables */
  variables?: Record<string, unknown>;

  /** Actions user can take */
  actions?: UserAction[];

  /** Help resources */
  helpUrl?: string;

  /** Support contact */
  supportContact?: string;

  /** Show technical details? */
  showTechnicalDetails?: boolean;
}

/**
 * User action for error resolution
 */
export interface UserAction {
  /** Action label */
  label: string;

  /** Action URL */
  url?: string;

  /** Handler function name */
  handler?: string;

  /** Action type */
  type?: 'primary' | 'secondary' | 'link';

  /** Is this a destructive action? */
  destructive?: boolean;
}

// ============================================================================
// API Error Response
// ============================================================================

/**
 * API error response format
 */
export interface APIErrorResponse {
  /** Error object */
  error: {
    /** Error code */
    code: string;
    /** User-friendly message */
    message: string;
    /** Severity level */
    severity: ErrorSeverity;
    /** Timestamp */
    timestamp: string;
    /** Request context (sanitized) */
    context?: {
      requestId?: string;
      [key: string]: unknown;
    };
    /** Recovery suggestions */
    recovery?: {
      suggestions?: string[];
      documentationUrl?: string;
    };
  };

  /** HTTP status code */
  status: number;

  /** Trace ID for debugging */
  traceId?: string;
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Standard error code patterns
 */
export const ERROR_CODE_PATTERN = /^WIA-[A-Z0-9]+-[A-Z0-9]+-\d{3}$/;

/**
 * Sensitive field names to redact
 */
export const SENSITIVE_FIELDS = [
  'password',
  'token',
  'apiKey',
  'api_key',
  'secret',
  'secretKey',
  'secret_key',
  'privateKey',
  'private_key',
  'creditCard',
  'credit_card',
  'ssn',
  'socialSecurity',
  'authorization',
  'bearer',
] as const;

/**
 * HTTP status code mapping
 */
export const ERROR_HTTP_STATUS_MAP: Record<string, number> = {
  AUTH: 401,
  PERMISSION: 403,
  VALIDATION: 400,
  RESOURCE: 404,
  NETWORK: 503,
  DATABASE: 500,
  SYSTEM: 500,
  EXTERNAL: 502,
  BUSINESS: 409,
  UNKNOWN: 500,
} as const;

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Result type for operations that can fail
 */
export type Result<T, E = WIAError> =
  | { success: true; data: T }
  | { success: false; error: E };

/**
 * Async result type
 */
export type AsyncResult<T, E = WIAError> = Promise<Result<T, E>>;

/**
 * Error filter for querying
 */
export interface ErrorFilter {
  /** Filter by error code */
  code?: string | RegExp;

  /** Filter by category */
  category?: ErrorCategory;

  /** Filter by severity */
  severity?: ErrorSeverity;

  /** Filter by time range */
  timeRange?: {
    start: Date;
    end: Date;
  };

  /** Filter by service */
  service?: string;

  /** Filter by user */
  userId?: string;

  /** Custom filter function */
  custom?: (error: WIAError) => boolean;
}

// ============================================================================
// Error Class
// ============================================================================

/**
 * WIA Error class implementation
 */
export class WIAErrorClass extends Error implements WIAError {
  code: string;
  severity: ErrorSeverity;
  timestamp: Date;
  context?: ErrorContext;
  stack?: string[];
  cause?: Error | WIAError;
  recovery?: RecoveryInfo;
  metadata?: Record<string, unknown>;

  constructor(config: ErrorConfig) {
    super(config.message);
    this.name = 'WIAError';
    this.code = config.code;
    this.severity = config.severity ?? ErrorSeverity.ERROR;
    this.timestamp = new Date();
    this.context = config.context;
    this.cause = config.cause;
    this.recovery = config.recovery;
    this.metadata = config.metadata;

    // Capture stack trace
    if (Error.captureStackTrace) {
      Error.captureStackTrace(this, WIAErrorClass);
    }

    // Convert stack to array
    this.stack = this.stack?.split('\n').map((line) => line.trim());
  }

  /**
   * Convert error to JSON format
   */
  toJSON(): Record<string, unknown> {
    return {
      code: this.code,
      message: this.message,
      severity: ErrorSeverity[this.severity],
      timestamp: this.timestamp.toISOString(),
      context: this.context,
      stack: this.stack,
      recovery: this.recovery,
      metadata: this.metadata,
    };
  }

  /**
   * Get user-friendly message
   */
  getUserMessage(): string {
    return this.recovery?.suggestions?.[0] ?? this.message;
  }

  /**
   * Check if error is retryable
   */
  isRetryable(): boolean {
    return this.recovery?.retryable ?? false;
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  // Core types
  WIAError,
  ErrorContext,
  RecoveryInfo,
  RetryConfig,
  RecoveryStrategy,

  // Configuration
  ErrorConfig,
  ErrorDefinition,
  ErrorHandlerOptions,
  HandleErrorOptions,
  RecoveryConfig,
  FallbackConfig,
  CircuitBreakerConfig,

  // Handling
  HandledError,
  ServiceError,
  ErrorTransformOptions,

  // Logging and reporting
  ErrorLog,
  ErrorMetrics,
  ErrorAggregationConfig,

  // User-facing
  UserErrorMessage,
  UserAction,
  APIErrorResponse,

  // Utilities
  ErrorFilter,
};

export {
  ErrorSeverity,
  ErrorCategory,
  WIAErrorClass,
  ERROR_CODE_PATTERN,
  SENSITIVE_FIELDS,
  ERROR_HTTP_STATUS_MAP,
};
