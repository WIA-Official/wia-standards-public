/**
 * WIA-CORE-010: Universal Error Handling SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Core Standards Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides comprehensive error handling capabilities including:
 * - Standardized error creation and management
 * - Error recovery and retry mechanisms
 * - Error propagation and transformation
 * - Error logging and reporting
 * - Integration with monitoring systems
 */

import {
  WIAError,
  ErrorConfig,
  ErrorSeverity,
  ErrorCategory,
  ErrorHandlerOptions,
  HandleErrorOptions,
  HandledError,
  RecoveryConfig,
  RetryConfig,
  ErrorDefinition,
  ErrorFilter,
  ErrorMetrics,
  APIErrorResponse,
  UserErrorMessage,
  WIAErrorClass,
  SENSITIVE_FIELDS,
  ERROR_HTTP_STATUS_MAP,
  ERROR_CODE_PATTERN,
  ErrorContext,
  ServiceError,
} from './types';

// ============================================================================
// Error Creation
// ============================================================================

/**
 * Create a standardized WIA error
 *
 * @param config - Error configuration
 * @returns WIA error object
 */
export function createError(config: ErrorConfig): WIAError {
  // Validate error code format
  if (!ERROR_CODE_PATTERN.test(config.code)) {
    console.warn(`Error code ${config.code} does not match WIA pattern`);
  }

  // Sanitize context if requested
  const context = config.sanitize
    ? sanitizeContext(config.context)
    : config.context;

  // Create error instance
  return new WIAErrorClass({
    ...config,
    context,
  });
}

/**
 * Wrap a standard Error into a WIA error
 *
 * @param error - Standard JavaScript Error
 * @param code - WIA error code
 * @param severity - Error severity
 * @returns WIA error object
 */
export function wrapError(
  error: Error,
  code = 'WIA-CORE-UNKNOWN-999',
  severity: ErrorSeverity = ErrorSeverity.ERROR
): WIAError {
  return createError({
    code,
    message: error.message,
    severity,
    cause: error,
    context: {
      originalName: error.name,
      originalStack: error.stack,
    },
  });
}

// ============================================================================
// Error Handler Class
// ============================================================================

/**
 * Main error handler class
 */
export class ErrorHandler {
  private options: ErrorHandlerOptions;
  private errorCounts = new Map<string, number>();
  private errorRegistry = new Map<string, ErrorDefinition>();
  private lastWindowReset = Date.now();

  constructor(options: ErrorHandlerOptions = {}) {
    this.options = {
      logLevel: ErrorSeverity.WARNING,
      reportErrors: true,
      includeStackTrace: true,
      sanitizeSensitiveData: true,
      ...options,
    };
  }

  /**
   * Handle an error with configured options
   *
   * @param error - Error to handle
   * @param options - Handle options
   * @returns Handled error result
   */
  async handle<T = unknown>(
    error: Error | WIAError,
    options: HandleErrorOptions = {}
  ): Promise<HandledError<T>> {
    // Normalize error to WIA format
    const wiaError = this.normalizeError(error);

    // Check rate limit
    if (!this.checkRateLimit(wiaError)) {
      console.warn('Error rate limit exceeded for:', wiaError.code);
      if (this.options.rateLimit?.onLimitExceeded) {
        this.options.rateLimit.onLimitExceeded([wiaError]);
      }
    }

    // Log error
    if (options.log !== false) {
      this.log(wiaError);
    }

    // Report error
    if (options.report && this.options.reportErrors) {
      await this.report(wiaError);
    }

    // Attempt recovery
    if (options.recover && options.recovery) {
      return await this.attemptRecovery<T>(wiaError, options.recovery);
    }

    return {
      recovered: false,
      error: wiaError,
    };
  }

  /**
   * Catch and handle an error (alias for handle)
   */
  async catch<T = unknown>(
    error: Error | WIAError,
    options?: HandleErrorOptions
  ): Promise<HandledError<T>> {
    return this.handle(error, options);
  }

  /**
   * Log error based on severity
   *
   * @param error - Error to log
   */
  private log(error: WIAError): void {
    if (error.severity < (this.options.logLevel ?? ErrorSeverity.WARNING)) {
      return;
    }

    const logData = {
      code: error.code,
      message: error.message,
      severity: ErrorSeverity[error.severity],
      timestamp: error.timestamp.toISOString(),
      context: error.context,
      ...(this.options.includeStackTrace && { stack: error.stack }),
    };

    switch (error.severity) {
      case ErrorSeverity.CRITICAL:
      case ErrorSeverity.ERROR:
        console.error('[WIA-ERROR]', logData);
        break;
      case ErrorSeverity.WARNING:
        console.warn('[WIA-WARNING]', logData);
        break;
      case ErrorSeverity.INFO:
        console.info('[WIA-INFO]', logData);
        break;
      case ErrorSeverity.DEBUG:
        console.debug('[WIA-DEBUG]', logData);
        break;
    }

    // Call custom error handler
    if (this.options.onError) {
      this.options.onError(error);
    }
  }

  /**
   * Report error to monitoring systems
   *
   * @param error - Error to report
   */
  private async report(error: WIAError): Promise<void> {
    // Report to Sentry
    if (this.options.integrations?.sentry) {
      // In a real implementation, would integrate with Sentry SDK
      console.log('Reporting to Sentry:', error.code);
    }

    // Report to Datadog
    if (this.options.integrations?.datadog) {
      // In a real implementation, would integrate with Datadog SDK
      console.log('Reporting to Datadog:', error.code);
    }

    // Report to webhook
    if (this.options.integrations?.webhook) {
      try {
        await fetch(this.options.integrations.webhook.url, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
            ...this.options.integrations.webhook.headers,
          },
          body: JSON.stringify(error),
        });
      } catch (err) {
        console.error('Failed to send error to webhook:', err);
      }
    }
  }

  /**
   * Normalize any error to WIA format
   *
   * @param error - Error to normalize
   * @returns WIA error
   */
  private normalizeError(error: Error | WIAError): WIAError {
    if ('code' in error && ERROR_CODE_PATTERN.test(error.code)) {
      return error as WIAError;
    }

    return wrapError(error);
  }

  /**
   * Check if error rate limit is exceeded
   *
   * @param error - Error to check
   * @returns True if within limits
   */
  private checkRateLimit(error: WIAError): boolean {
    if (!this.options.rateLimit) {
      return true;
    }

    // Reset window if needed
    const now = Date.now();
    if (now - this.lastWindowReset >= this.options.rateLimit.window) {
      this.errorCounts.clear();
      this.lastWindowReset = now;
    }

    // Increment error count
    const key = `${error.code}:${error.context?.userId ?? 'anonymous'}`;
    const count = (this.errorCounts.get(key) ?? 0) + 1;
    this.errorCounts.set(key, count);

    return count <= this.options.rateLimit.maxErrors;
  }

  /**
   * Attempt error recovery
   *
   * @param error - Error to recover from
   * @param config - Recovery configuration
   * @returns Recovery result
   */
  private async attemptRecovery<T>(
    error: WIAError,
    config: RecoveryConfig
  ): Promise<HandledError<T>> {
    // Try fallback first if available
    if (config.fallback) {
      try {
        const result = await this.executeFallback<T>(config.fallback);
        return {
          recovered: true,
          result,
          error,
          strategy: 'fallback',
        };
      } catch (fallbackError) {
        console.warn('Fallback failed:', fallbackError);
      }
    }

    return {
      recovered: false,
      error,
    };
  }

  /**
   * Execute fallback strategy
   *
   * @param config - Fallback configuration
   * @returns Fallback result
   */
  private async executeFallback<T>(config: any): Promise<T> {
    if (config.type === 'value') {
      return config.value as T;
    }

    if (config.type === 'function' && config.function) {
      return await config.function();
    }

    if (config.type === 'cache' && config.cacheKey) {
      // In a real implementation, would fetch from cache
      throw new Error('Cache fallback not implemented');
    }

    throw new Error('Invalid fallback configuration');
  }

  /**
   * Register an error definition
   *
   * @param definition - Error definition
   */
  registerError(definition: ErrorDefinition): void {
    this.errorRegistry.set(definition.code, definition);
  }

  /**
   * Lookup error definition
   *
   * @param code - Error code
   * @returns Error definition or null
   */
  lookupError(code: string): ErrorDefinition | null {
    return this.errorRegistry.get(code) ?? null;
  }
}

// ============================================================================
// Error Handling Functions
// ============================================================================

/**
 * Handle error with retry logic
 *
 * @param operation - Operation to execute
 * @param options - Retry options
 * @returns Operation result
 */
export async function handleWithRetry<T>(
  operation: () => Promise<T>,
  options: RetryConfig
): Promise<T> {
  let lastError: WIAError;
  let attempt = 0;

  while (attempt < options.maxAttempts) {
    try {
      return await operation();
    } catch (error) {
      lastError = error instanceof WIAErrorClass ? error : wrapError(error as Error);
      attempt++;

      // Check if we should retry
      if (options.condition && !options.condition(lastError, attempt)) {
        break;
      }

      // Don't wait after last attempt
      if (attempt >= options.maxAttempts) {
        break;
      }

      // Calculate backoff delay
      const delay = calculateBackoff(attempt, options);

      // Call onRetry callback
      if (options.onRetry) {
        options.onRetry(attempt, lastError);
      }

      // Wait before retrying
      await sleep(delay);
    }
  }

  throw lastError!;
}

/**
 * Calculate backoff delay
 *
 * @param attempt - Current attempt number
 * @param config - Retry configuration
 * @returns Delay in milliseconds
 */
function calculateBackoff(attempt: number, config: RetryConfig): number {
  let delay: number;

  switch (config.backoff) {
    case 'linear':
      delay = config.delayMs * attempt;
      break;
    case 'exponential':
      delay = config.delayMs * Math.pow(2, attempt - 1);
      break;
    case 'fixed':
    default:
      delay = config.delayMs;
      break;
  }

  // Apply max delay cap
  if (config.maxDelayMs) {
    delay = Math.min(delay, config.maxDelayMs);
  }

  // Add jitter if enabled
  if (config.jitter) {
    const jitter = Math.random() * 0.3 * delay; // ±30% jitter
    delay = delay + jitter - 0.15 * delay;
  }

  return Math.max(0, delay);
}

/**
 * Sleep for specified milliseconds
 *
 * @param ms - Milliseconds to sleep
 */
function sleep(ms: number): Promise<void> {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

// ============================================================================
// Error Transformation
// ============================================================================

/**
 * Transform error to API response format
 *
 * @param error - Error to transform
 * @returns API error response
 */
export function toAPIError(error: WIAError): APIErrorResponse {
  const category = extractCategory(error.code);
  const httpStatus = ERROR_HTTP_STATUS_MAP[category] ?? 500;

  return {
    error: {
      code: error.code,
      message: error.message,
      severity: error.severity,
      timestamp: error.timestamp.toISOString(),
      context: {
        requestId: error.context?.requestId,
      },
      recovery: {
        suggestions: error.recovery?.suggestions,
        documentationUrl: error.recovery?.documentationUrl,
      },
    },
    status: httpStatus,
    traceId: error.context?.requestId,
  };
}

/**
 * Transform error to user-friendly message
 *
 * @param error - Error to transform
 * @param locale - Target locale
 * @returns User error message
 */
export function toUserMessage(
  error: WIAError,
  locale = 'en'
): UserErrorMessage {
  return {
    messages: {
      [locale]: error.message,
      // In a real implementation, would have translations
    },
    actions: error.recovery?.suggestions?.map((suggestion) => ({
      label: suggestion,
      type: 'secondary' as const,
    })),
    helpUrl: error.recovery?.documentationUrl,
    supportContact: error.recovery?.supportContact,
  };
}

/**
 * Sanitize error context to remove sensitive data
 *
 * @param context - Error context
 * @returns Sanitized context
 */
export function sanitizeContext(
  context?: Partial<ErrorContext>
): ErrorContext | undefined {
  if (!context) return undefined;

  const sanitized = { ...context };

  // Remove sensitive fields
  for (const key of Object.keys(sanitized)) {
    if (SENSITIVE_FIELDS.includes(key as any)) {
      (sanitized as any)[key] = '[REDACTED]';
    }

    // Check nested objects
    if (typeof (sanitized as any)[key] === 'object') {
      (sanitized as any)[key] = sanitizeContext((sanitized as any)[key]);
    }
  }

  return sanitized as ErrorContext;
}

// ============================================================================
// Error Metrics
// ============================================================================

/**
 * Error metrics tracker
 */
export class ErrorMetricsTracker {
  private errors: WIAError[] = [];
  private startTime = Date.now();

  /**
   * Record an error
   *
   * @param error - Error to record
   */
  recordError(error: WIAError): void {
    this.errors.push(error);
  }

  /**
   * Get error metrics
   *
   * @returns Error metrics
   */
  getMetrics(): ErrorMetrics {
    const errorsByCode: Record<string, number> = {};
    const errorsBySeverity: Record<string, number> = {};
    const errorsByCategory: Record<string, number> = {};

    for (const error of this.errors) {
      // By code
      errorsByCode[error.code] = (errorsByCode[error.code] ?? 0) + 1;

      // By severity
      const severity = ErrorSeverity[error.severity];
      errorsBySeverity[severity] = (errorsBySeverity[severity] ?? 0) + 1;

      // By category
      const category = extractCategory(error.code);
      errorsByCategory[category] = (errorsByCategory[category] ?? 0) + 1;
    }

    // Calculate error rate
    const elapsedMinutes = (Date.now() - this.startTime) / 60000;
    const errorRate = this.errors.length / Math.max(elapsedMinutes, 1);

    // Get top errors
    const topErrors = Object.entries(errorsByCode)
      .sort((a, b) => b[1] - a[1])
      .slice(0, 10)
      .map(([code, count]) => ({
        code,
        count,
        percentage: (count / this.errors.length) * 100,
      }));

    return {
      totalErrors: this.errors.length,
      errorsByCode,
      errorsBySeverity,
      errorsByCategory,
      errorRate,
      topErrors,
    };
  }

  /**
   * Filter errors
   *
   * @param filter - Error filter
   * @returns Filtered errors
   */
  filterErrors(filter: ErrorFilter): WIAError[] {
    return this.errors.filter((error) => {
      // Filter by code
      if (filter.code) {
        if (typeof filter.code === 'string' && error.code !== filter.code) {
          return false;
        }
        if (filter.code instanceof RegExp && !filter.code.test(error.code)) {
          return false;
        }
      }

      // Filter by severity
      if (filter.severity && error.severity !== filter.severity) {
        return false;
      }

      // Filter by time range
      if (filter.timeRange) {
        if (
          error.timestamp < filter.timeRange.start ||
          error.timestamp > filter.timeRange.end
        ) {
          return false;
        }
      }

      // Filter by service
      if (filter.service && error.context?.service !== filter.service) {
        return false;
      }

      // Filter by user
      if (filter.userId && error.context?.userId !== filter.userId) {
        return false;
      }

      // Custom filter
      if (filter.custom && !filter.custom(error)) {
        return false;
      }

      return true;
    });
  }

  /**
   * Clear all recorded errors
   */
  clear(): void {
    this.errors = [];
    this.startTime = Date.now();
  }
}

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * Extract category from error code
 *
 * @param code - Error code
 * @returns Category
 */
function extractCategory(code: string): string {
  const parts = code.split('-');
  return parts.length >= 3 ? parts[2] : 'UNKNOWN';
}

/**
 * Validate error code format
 *
 * @param code - Error code to validate
 * @returns True if valid
 */
export function validateErrorCode(code: string): boolean {
  return ERROR_CODE_PATTERN.test(code);
}

/**
 * Parse error code components
 *
 * @param code - Error code
 * @returns Parsed components
 */
export function parseErrorCode(code: string): {
  prefix: string;
  domain: string;
  category: string;
  number: string;
} | null {
  const match = code.match(/^(WIA)-([A-Z0-9]+)-([A-Z0-9]+)-(\d{3})$/);
  if (!match) return null;

  return {
    prefix: match[1],
    domain: match[2],
    category: match[3],
    number: match[4],
  };
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export {
  ErrorHandler,
  ErrorMetricsTracker,
  createError,
  wrapError,
  handleWithRetry,
  toAPIError,
  toUserMessage,
  sanitizeContext,
  validateErrorCode,
  parseErrorCode,
};

export default ErrorHandler;
