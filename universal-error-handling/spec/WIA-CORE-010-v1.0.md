# WIA-CORE-010: Universal Error Handling Specification v1.0

> **Standard ID:** WIA-CORE-010
> **Version:** 1.0.0
> **Published:** 2025-12-27
> **Status:** Active
> **Authors:** WIA Core Standards Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Error Code Structure](#2-error-code-structure)
3. [Error Taxonomy](#3-error-taxonomy)
4. [Error Object Specification](#4-error-object-specification)
5. [Error Propagation](#5-error-propagation)
6. [Error Recovery](#6-error-recovery)
7. [Error Reporting](#7-error-reporting)
8. [Implementation Guidelines](#8-implementation-guidelines)
9. [Security Considerations](#9-security-considerations)
10. [References](#10-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines a universal error handling framework for all WIA standards, providing:
- Consistent error representation across systems
- Standardized error codes and categories
- Error propagation and recovery mechanisms
- Integration with logging and monitoring systems

### 1.2 Scope

The standard covers:
- Error code structure and naming conventions
- Error object format and required fields
- Error severity levels and categorization
- Error handling patterns and best practices
- Integration points with other WIA standards

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to make error handling predictable, debuggable, and user-friendly, reducing frustration and improving system reliability for developers and end-users alike.

### 1.4 Terminology

- **Error Code**: Unique identifier for an error type
- **Error Context**: Additional metadata about error circumstances
- **Error Severity**: Importance/urgency level of an error
- **Error Recovery**: Automated or manual steps to resolve an error
- **Error Propagation**: Flow of error information through system layers

---

## 2. Error Code Structure

### 2.1 Code Format

All WIA error codes follow this structure:

```
WIA-{DOMAIN}-{CATEGORY}-{NUMBER}
```

Where:
- `WIA`: Prefix identifying WIA standards
- `DOMAIN`: 2-8 uppercase letters identifying the standard domain
- `CATEGORY`: 2-12 uppercase letters identifying error category
- `NUMBER`: 3-digit number (001-999) for specific error

### 2.2 Examples

```
WIA-CORE-AUTH-001          # Core authentication error
WIA-CORE-VALIDATION-100    # Core validation error
WIA-TIME-PARADOX-001       # Time travel paradox error
WIA-QUANTUM-DECOHERENCE-050 # Quantum decoherence error
```

### 2.3 Domain Codes

| Domain | Description | Example Standard |
|--------|-------------|------------------|
| CORE | Core/universal standards | WIA-CORE-010 |
| TIME | Temporal standards | WIA-TIME-001 |
| QUANTUM | Quantum computing | WIA-QUANTUM-001 |
| BIO | Biotechnology | WIA-BIO-001 |
| DEF | Defense systems | WIA-DEF-001 |
| SOCIAL | Social protocols | WIA-SOCIAL |
| INTENT | Intent processing | WIA-INTENT |
| API | API standards | WIA-OMNI-API |

### 2.4 Category Codes

Standard categories for CORE domain:

| Category | Code Range | Description |
|----------|------------|-------------|
| AUTH | 001-099 | Authentication/Authorization |
| VALIDATION | 100-199 | Input validation |
| NETWORK | 200-299 | Network/Communication |
| DATABASE | 300-399 | Data persistence |
| RESOURCE | 400-499 | Resource management |
| PERMISSION | 500-599 | Access control |
| SYSTEM | 600-699 | System failures |
| EXTERNAL | 700-799 | External service errors |
| BUSINESS | 800-899 | Business logic |
| UNKNOWN | 900-999 | Unclassified errors |

---

## 3. Error Taxonomy

### 3.1 Severity Levels

```typescript
enum ErrorSeverity {
  CRITICAL = 5,  // System failure, immediate action required
  ERROR = 4,     // Operation failed, intervention needed
  WARNING = 3,   // Potential issue, degraded functionality
  INFO = 2,      // Informational, no action required
  DEBUG = 1      // Debugging information
}
```

#### 3.1.1 CRITICAL (Level 5)

- System-wide failure
- Data corruption risk
- Security breach
- Immediate escalation required

Example: Database cluster failure, authentication system compromise

#### 3.1.2 ERROR (Level 4)

- Operation failed completely
- User intervention needed
- No automatic recovery possible
- Affects user experience

Example: Failed payment processing, invalid user credentials

#### 3.1.3 WARNING (Level 3)

- Potential issue detected
- Degraded functionality
- Automatic recovery possible
- May require attention

Example: High memory usage, slow API response times

#### 3.1.4 INFO (Level 2)

- Informational messages
- Normal operational events
- No action required
- Useful for auditing

Example: User login event, configuration change

#### 3.1.5 DEBUG (Level 1)

- Debugging information
- Development/testing only
- Verbose details
- Should be disabled in production

Example: Variable values, execution flow details

### 3.2 Error Categories by HTTP Status

| HTTP Status | WIA Category | Description |
|-------------|--------------|-------------|
| 400 | VALIDATION | Bad request, invalid input |
| 401 | AUTH | Unauthorized, authentication required |
| 403 | PERMISSION | Forbidden, insufficient permissions |
| 404 | RESOURCE | Not found |
| 408 | NETWORK | Request timeout |
| 409 | BUSINESS | Conflict, business rule violation |
| 429 | RESOURCE | Too many requests, rate limit |
| 500 | SYSTEM | Internal server error |
| 502 | NETWORK | Bad gateway |
| 503 | RESOURCE | Service unavailable |
| 504 | NETWORK | Gateway timeout |

---

## 4. Error Object Specification

### 4.1 Base Error Interface

```typescript
interface WIAError {
  // Required fields
  code: string;              // WIA error code
  message: string;           // Human-readable message
  severity: ErrorSeverity;   // Severity level
  timestamp: Date;           // When error occurred

  // Optional fields
  context?: ErrorContext;    // Additional context
  stack?: string[];          // Stack trace
  cause?: Error | WIAError;  // Original error
  recovery?: RecoveryInfo;   // Recovery suggestions
  metadata?: Record<string, unknown>;  // Custom metadata
}
```

### 4.2 Error Context

```typescript
interface ErrorContext {
  // Request context
  requestId?: string;
  userId?: string;
  sessionId?: string;
  ipAddress?: string;

  // System context
  service?: string;
  environment?: string;
  version?: string;
  hostname?: string;

  // Operation context
  operation?: string;
  resource?: string;
  parameters?: Record<string, unknown>;

  // Error-specific data
  details?: Record<string, unknown>;
}
```

### 4.3 Recovery Information

```typescript
interface RecoveryInfo {
  // Is automatic retry possible?
  retryable: boolean;

  // Retry configuration
  retry?: {
    maxAttempts: number;
    backoff: 'linear' | 'exponential' | 'fixed';
    delayMs: number;
    condition?: (error: WIAError) => boolean;
  };

  // User-facing suggestions
  suggestions?: string[];

  // Programmatic recovery
  recovery?: {
    strategy: 'retry' | 'fallback' | 'ignore' | 'escalate';
    fallbackValue?: unknown;
    fallbackFunction?: string;
  };

  // Documentation
  documentationUrl?: string;
  supportContact?: string;
}
```

### 4.4 JSON Representation

```json
{
  "error": {
    "code": "WIA-CORE-AUTH-001",
    "message": "Authentication failed: Invalid credentials",
    "severity": "ERROR",
    "timestamp": "2025-12-27T10:30:00.000Z",
    "context": {
      "requestId": "req_abc123",
      "userId": "user_456",
      "service": "AuthService",
      "operation": "login",
      "details": {
        "username": "john@example.com",
        "ipAddress": "192.168.1.100",
        "userAgent": "Mozilla/5.0..."
      }
    },
    "recovery": {
      "retryable": true,
      "retry": {
        "maxAttempts": 3,
        "backoff": "exponential",
        "delayMs": 1000
      },
      "suggestions": [
        "Verify your username and password",
        "Check if Caps Lock is on",
        "Try resetting your password"
      ],
      "documentationUrl": "https://docs.wiastandards.com/auth/troubleshooting"
    },
    "stack": [
      "at AuthService.authenticate (auth.service.ts:45:12)",
      "at AuthController.login (auth.controller.ts:23:18)",
      "at Layer.handle [as handle_request] (router/layer.js:95:5)"
    ]
  }
}
```

---

## 5. Error Propagation

### 5.1 Propagation Principles

1. **Preserve Original Error**: Always maintain reference to root cause
2. **Add Context**: Enrich error with layer-specific information
3. **Transform Appropriately**: Convert internal errors to public-safe errors
4. **Maintain Causality Chain**: Link errors through `cause` field

### 5.2 Propagation Pattern

```typescript
// Layer 1: Database
function findUser(id: string): User {
  try {
    return db.query('SELECT * FROM users WHERE id = ?', [id]);
  } catch (dbError) {
    throw new WIAError({
      code: 'WIA-CORE-DATABASE-301',
      message: 'Database query failed',
      severity: 'ERROR',
      cause: dbError,
      context: { operation: 'findUser', userId: id }
    });
  }
}

// Layer 2: Service
function getUser(id: string): User {
  try {
    return findUser(id);
  } catch (error) {
    throw new WIAError({
      code: 'WIA-CORE-SYSTEM-601',
      message: 'User retrieval failed',
      severity: 'ERROR',
      cause: error,
      context: { service: 'UserService', operation: 'getUser' }
    });
  }
}

// Layer 3: API
async function handleGetUserRequest(req, res) {
  try {
    const user = getUser(req.params.id);
    res.json(user);
  } catch (error) {
    const apiError = transformToAPIError(error);
    res.status(apiError.httpStatus).json(apiError);
  }
}
```

### 5.3 Error Transformation

```typescript
function transformToAPIError(error: WIAError): APIError {
  // Remove sensitive information
  const publicError = sanitizeError(error);

  // Map to HTTP status
  const httpStatus = mapErrorToHTTPStatus(error.code);

  // Add API-specific fields
  return {
    error: {
      code: publicError.code,
      message: publicError.message,
      severity: publicError.severity,
      timestamp: publicError.timestamp,
      // Only include safe context
      context: {
        requestId: publicError.context?.requestId,
        // Exclude sensitive fields
      },
      recovery: publicError.recovery
    },
    httpStatus
  };
}
```

### 5.4 Cross-Service Propagation

When errors cross service boundaries:

```typescript
interface ServiceError extends WIAError {
  // Original service information
  originService: string;
  originEnvironment: string;
  originRequestId: string;

  // Propagation path
  propagationPath: string[];  // ['ServiceA', 'ServiceB', 'ServiceC']
  hops: number;               // Number of service hops

  // Distributed tracing
  traceId: string;
  spanId: string;
}
```

---

## 6. Error Recovery

### 6.1 Recovery Strategies

#### 6.1.1 Retry Strategy

```typescript
interface RetryStrategy {
  maxAttempts: number;
  backoff: 'linear' | 'exponential' | 'fixed';
  delayMs: number;
  maxDelayMs?: number;
  jitter?: boolean;  // Add randomness to prevent thundering herd
  condition?: (error: WIAError, attempt: number) => boolean;
}

// Exponential backoff example
const retry: RetryStrategy = {
  maxAttempts: 5,
  backoff: 'exponential',
  delayMs: 1000,
  maxDelayMs: 30000,
  jitter: true,
  condition: (error, attempt) => {
    // Only retry network errors
    return error.code.includes('NETWORK') && attempt < 5;
  }
};
```

#### 6.1.2 Fallback Strategy

```typescript
interface FallbackStrategy {
  type: 'value' | 'function' | 'cache';
  value?: unknown;
  function?: () => Promise<unknown>;
  cacheKey?: string;
  cacheTTL?: number;
}

// Example: Return cached data on error
const fallback: FallbackStrategy = {
  type: 'cache',
  cacheKey: 'user_profile_123',
  cacheTTL: 300  // 5 minutes
};
```

#### 6.1.3 Circuit Breaker

```typescript
interface CircuitBreaker {
  failureThreshold: number;    // Open after N failures
  resetTimeout: number;         // Try closing after N ms
  halfOpenRequests: number;     // Test with N requests
  onOpen?: () => void;
  onClose?: () => void;
  onHalfOpen?: () => void;
}

const circuitBreaker: CircuitBreaker = {
  failureThreshold: 5,
  resetTimeout: 60000,  // 1 minute
  halfOpenRequests: 3
};
```

### 6.2 Recovery Implementation

```typescript
async function executeWithRecovery<T>(
  operation: () => Promise<T>,
  options: RecoveryOptions
): Promise<T> {
  let lastError: WIAError;
  let attempt = 0;

  while (attempt < options.retry.maxAttempts) {
    try {
      return await operation();
    } catch (error) {
      lastError = error instanceof WIAError ? error : wrapError(error);
      attempt++;

      // Check if retryable
      if (!options.retry.condition?.(lastError, attempt)) {
        break;
      }

      // Calculate backoff delay
      const delay = calculateBackoff(attempt, options.retry);
      await sleep(delay);

      // Log retry attempt
      logger.warn('Retrying operation', {
        attempt,
        error: lastError.code,
        nextDelay: delay
      });
    }
  }

  // All retries exhausted, try fallback
  if (options.fallback) {
    logger.info('Using fallback after retry exhaustion');
    return await executeFallback(options.fallback);
  }

  // No recovery possible
  throw lastError;
}
```

### 6.3 Graceful Degradation

```typescript
interface DegradationStrategy {
  // Levels of service degradation
  levels: {
    full: () => Promise<unknown>;      // Full functionality
    reduced: () => Promise<unknown>;   // Reduced functionality
    minimal: () => Promise<unknown>;   // Minimal functionality
    offline: () => Promise<unknown>;   // Offline mode
  };

  // Thresholds for degradation
  thresholds: {
    errorRate: number;      // Error rate to trigger degradation
    responseTime: number;   // Response time threshold
    resourceUsage: number;  // Resource usage threshold
  };
}
```

---

## 7. Error Reporting

### 7.1 Logging Standards

```typescript
interface ErrorLog {
  // Standard fields
  timestamp: Date;
  level: 'ERROR' | 'WARNING' | 'INFO' | 'DEBUG';
  error: WIAError;

  // Environment
  environment: string;
  service: string;
  version: string;

  // Tracing
  traceId?: string;
  spanId?: string;

  // Additional context
  tags?: string[];
  fingerprint?: string;  // For error grouping
}
```

### 7.2 Error Monitoring

Integration with monitoring systems:

```typescript
interface ErrorMonitoring {
  // Error tracking
  trackError(error: WIAError): void;
  trackErrorRate(rate: number): void;

  // Alerting
  sendAlert(error: WIAError, channel: string): void;
  createIncident(error: WIAError): string;

  // Metrics
  recordMetric(name: string, value: number, tags?: Record<string, string>): void;

  // Integration points
  integrations: {
    sentry?: SentryConfig;
    datadog?: DatadogConfig;
    prometheus?: PrometheusConfig;
    cloudwatch?: CloudWatchConfig;
  };
}
```

### 7.3 Error Aggregation

```typescript
interface ErrorAggregation {
  // Group similar errors
  groupBy: 'code' | 'message' | 'fingerprint' | 'service';

  // Time windows
  window: {
    size: number;      // Window size in ms
    slide: number;     // Slide interval in ms
  };

  // Thresholds
  thresholds: {
    count: number;     // Error count threshold
    rate: number;      // Error rate threshold
    unique: number;    // Unique error types threshold
  };

  // Actions
  onThresholdExceeded: (errors: WIAError[]) => void;
}
```

### 7.4 User-Facing Error Messages

```typescript
interface UserErrorMessage {
  // Localized messages
  messages: {
    [locale: string]: string;
  };

  // Message templates
  template?: string;
  variables?: Record<string, unknown>;

  // Actions user can take
  actions?: {
    label: string;
    url?: string;
    handler?: string;
  }[];

  // Help resources
  helpUrl?: string;
  supportContact?: string;
}
```

---

## 8. Implementation Guidelines

### 8.1 Required Components

Any WIA-CORE-010 compliant system must include:

1. **Error Factory**: Create standardized WIAError objects
2. **Error Handler**: Process and route errors appropriately
3. **Error Logger**: Log errors with proper context
4. **Error Reporter**: Report errors to monitoring systems
5. **Recovery Manager**: Implement recovery strategies

### 8.2 API Interface

#### 8.2.1 Create Error

```typescript
function createError(config: ErrorConfig): WIAError {
  return {
    code: config.code,
    message: config.message,
    severity: config.severity ?? 'ERROR',
    timestamp: new Date(),
    context: sanitizeContext(config.context),
    stack: captureStackTrace(),
    cause: config.cause,
    recovery: config.recovery,
    metadata: config.metadata
  };
}
```

#### 8.2.2 Handle Error

```typescript
async function handleError(
  error: Error | WIAError,
  options?: HandleOptions
): Promise<HandledError> {
  // Normalize error
  const wiaError = normalizeError(error);

  // Log error
  logger.error(wiaError);

  // Report to monitoring
  if (options?.report) {
    await errorReporter.report(wiaError);
  }

  // Attempt recovery
  if (options?.recover) {
    return await attemptRecovery(wiaError, options.recovery);
  }

  return { error: wiaError, recovered: false };
}
```

### 8.3 Error Code Registry

Maintain a central registry of all error codes:

```typescript
interface ErrorCodeRegistry {
  // Register new error code
  register(definition: ErrorDefinition): void;

  // Lookup error code
  lookup(code: string): ErrorDefinition | null;

  // Validate error code format
  validate(code: string): boolean;

  // List all error codes
  list(filter?: ErrorFilter): ErrorDefinition[];
}

interface ErrorDefinition {
  code: string;
  category: string;
  severity: ErrorSeverity;
  description: string;
  recoverable: boolean;
  httpStatus?: number;
  examples?: string[];
  since?: string;  // Version when introduced
}
```

### 8.4 Testing Error Handling

```typescript
describe('Error Handling', () => {
  it('should create WIA-compliant error', () => {
    const error = createError({
      code: 'WIA-CORE-TEST-001',
      message: 'Test error',
      severity: 'ERROR'
    });

    expect(error.code).toMatch(/^WIA-[A-Z]+-[A-Z]+-\d{3}$/);
    expect(error.severity).toBe('ERROR');
    expect(error.timestamp).toBeInstanceOf(Date);
  });

  it('should retry on transient errors', async () => {
    let attempts = 0;
    const operation = async () => {
      attempts++;
      if (attempts < 3) {
        throw createError({
          code: 'WIA-CORE-NETWORK-200',
          message: 'Network timeout',
          severity: 'WARNING'
        });
      }
      return 'success';
    };

    const result = await handleError(operation, {
      retry: { maxAttempts: 3, backoff: 'fixed', delayMs: 100 }
    });

    expect(result).toBe('success');
    expect(attempts).toBe(3);
  });
});
```

---

## 9. Security Considerations

### 9.1 Sensitive Data Sanitization

```typescript
const SENSITIVE_FIELDS = [
  'password',
  'token',
  'apiKey',
  'secret',
  'creditCard',
  'ssn',
  'privateKey'
];

function sanitizeContext(context: ErrorContext): ErrorContext {
  const sanitized = { ...context };

  for (const key in sanitized) {
    if (SENSITIVE_FIELDS.includes(key)) {
      sanitized[key] = '[REDACTED]';
    }
  }

  return sanitized;
}
```

### 9.2 Information Disclosure Prevention

```typescript
function createPublicError(error: WIAError): PublicError {
  // Only expose safe information to clients
  return {
    code: error.code,
    message: getPublicMessage(error),
    severity: error.severity,
    timestamp: error.timestamp,
    recovery: {
      suggestions: error.recovery?.suggestions,
      documentationUrl: error.recovery?.documentationUrl
    }
    // DO NOT expose: stack, context details, cause chain
  };
}
```

### 9.3 Error Rate Limiting

```typescript
class ErrorRateLimiter {
  private errorCounts = new Map<string, number>();

  checkRateLimit(error: WIAError): boolean {
    const key = `${error.code}:${error.context?.userId}`;
    const count = this.errorCounts.get(key) ?? 0;

    if (count > MAX_ERRORS_PER_MINUTE) {
      // Possible attack or malfunction
      this.triggerSecurityAlert(error);
      return false;
    }

    this.errorCounts.set(key, count + 1);
    return true;
  }
}
```

---

## 10. References

### 10.1 Related Standards

- RFC 7807: Problem Details for HTTP APIs
- JSON:API Error Objects
- OpenAPI 3.0 Error Responses
- Google API Error Model
- Microsoft REST API Guidelines - Error Handling

### 10.2 WIA Standards

- WIA-INTENT: Intent-based error resolution
- WIA-OMNI-API: Universal API error format
- WIA-LOGGING: Centralized logging standard
- WIA-TELEMETRY: Observability and monitoring
- WIA-SECURITY: Security and privacy guidelines

### 10.3 Error Code Ranges

Complete error code registry available at:
https://errors.wiastandards.com/registry

---

## Appendix A: Common Error Codes

### Authentication (AUTH: 001-099)

| Code | Description | HTTP | Severity |
|------|-------------|------|----------|
| WIA-CORE-AUTH-001 | Invalid credentials | 401 | ERROR |
| WIA-CORE-AUTH-002 | Token expired | 401 | ERROR |
| WIA-CORE-AUTH-003 | Token invalid | 401 | ERROR |
| WIA-CORE-AUTH-004 | Session expired | 401 | WARNING |
| WIA-CORE-AUTH-005 | MFA required | 401 | INFO |

### Validation (VALIDATION: 100-199)

| Code | Description | HTTP | Severity |
|------|-------------|------|----------|
| WIA-CORE-VALIDATION-100 | Invalid input | 400 | ERROR |
| WIA-CORE-VALIDATION-101 | Missing required field | 400 | ERROR |
| WIA-CORE-VALIDATION-102 | Invalid format | 400 | ERROR |
| WIA-CORE-VALIDATION-103 | Value out of range | 400 | ERROR |
| WIA-CORE-VALIDATION-104 | Type mismatch | 400 | ERROR |

### Network (NETWORK: 200-299)

| Code | Description | HTTP | Severity |
|------|-------------|------|----------|
| WIA-CORE-NETWORK-200 | Network timeout | 408 | WARNING |
| WIA-CORE-NETWORK-201 | Connection refused | 503 | ERROR |
| WIA-CORE-NETWORK-202 | DNS resolution failed | 503 | ERROR |
| WIA-CORE-NETWORK-502 | Bad gateway | 502 | ERROR |
| WIA-CORE-NETWORK-504 | Gateway timeout | 504 | WARNING |

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-CORE-010 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
