# WIA-CORE-010 PHASE 2 Specification
## Error Propagation and Recovery Strategies

**Version:** 1.0.0
**Status:** Active
**Date:** 2025-12-27
**Depends On:** PHASE-1

---

## 1. Overview

PHASE 2 defines how errors propagate through distributed systems and establishes standardized recovery strategies. This phase ensures errors maintain full context as they traverse service boundaries while enabling automated recovery.

**Objectives:**
- Define error propagation patterns for distributed systems
- Establish 6 core recovery strategies
- Implement correlation ID system for distributed tracing
- Provide cross-language error serialization formats

**Deliverables:**
- Error propagation specification
- Recovery strategy implementation guide
- Correlation ID standard
- Serialization format specifications (JSON, Protocol Buffers, MessagePack)

---

## 2. Error Propagation

### 2.1 Correlation IDs

**Purpose:** Link related errors across distributed system boundaries.

**Format:**
```
{prefix}-{uuid}
```

**Generation Rules:**
1. Generated at entry point (API gateway, frontend)
2. Propagated through all downstream services
3. Included in all log entries, errors, and traces
4. Never modified during propagation

**Implementation:**
```typescript
// Entry point generates ID
const correlationId = `trace-${uuidv4()}`;

// Propagate via HTTP headers
headers['X-Correlation-ID'] = correlationId;

// Include in all errors
throw new WIAError({
  code: 'WIA-API-INTERNAL-001',
  correlationId: correlationId
});
```

### 2.2 Propagation Path Tracking

Track the path an error travels through services:

```json
{
  "code": "WIA-DB-TIMEOUT-015",
  "propagationPath": [
    {
      "service": "frontend",
      "timestamp": "2025-12-27T10:00:00.000Z",
      "operation": "submitOrder"
    },
    {
      "service": "api-gateway",
      "timestamp": "2025-12-27T10:00:00.100Z",
      "operation": "routeRequest"
    },
    {
      "service": "order-service",
      "timestamp": "2025-12-27T10:00:00.200Z",
      "operation": "createOrder"
    },
    {
      "service": "database",
      "timestamp": "2025-12-27T10:00:30.200Z",
      "operation": "executeQuery",
      "error": "timeout"
    }
  ]
}
```

### 2.3 Cross-Language Serialization

#### JSON Format (Universal)
```json
{
  "code": "WIA-AUTH-INVALID-001",
  "message": "Invalid credentials",
  "severity": "ERROR",
  "timestamp": "2025-12-27T10:00:00.000Z",
  "correlationId": "trace-abc123",
  "recovery": "FAIL_FAST",
  "context": {
    "userId": "user_123",
    "attemptNumber": 3
  }
}
```

#### Protocol Buffers (High Performance)
```protobuf
message WIAError {
  string code = 1;
  string message = 2;
  Severity severity = 3;
  string timestamp = 4;
  string correlation_id = 5;
  Recovery recovery = 6;
  map<string, string> context = 7;
  string stack_trace = 8;
  WIAError caused_by = 9;
}

enum Severity {
  CRITICAL = 0;
  ERROR = 1;
  WARNING = 2;
  INFO = 3;
}

enum Recovery {
  RETRY_WITH_BACKOFF = 0;
  FALLBACK = 1;
  CIRCUIT_BREAKER = 2;
  GRACEFUL_DEGRADATION = 3;
  FAIL_FAST = 4;
  MANUAL = 5;
}
```

### 2.4 HTTP Header Propagation

Required headers for error propagation:

```
X-Correlation-ID: trace-abc123
X-Request-ID: req-def456
X-B3-TraceId: 463ac35c9f6413ad         # Zipkin compatible
X-B3-SpanId: a2fb4a1d1a96d312          # Zipkin compatible
X-Cloud-Trace-Context: ...              # Google Cloud compatible
```

---

## 3. Recovery Strategies

### 3.1 RETRY_WITH_BACKOFF

**Use Case:** Temporary failures likely to resolve quickly.

**Implementation:**
```typescript
async function retryWithBackoff<T>(
  operation: () => Promise<T>,
  options: {
    maxAttempts?: number;
    baseDelay?: number;
    maxDelay?: number;
    jitter?: boolean;
  } = {}
): Promise<T> {
  const {
    maxAttempts = 5,
    baseDelay = 1000,
    maxDelay = 30000,
    jitter = true
  } = options;

  for (let attempt = 1; attempt <= maxAttempts; attempt++) {
    try {
      return await operation();
    } catch (error) {
      if (attempt === maxAttempts) throw error;
      if (error.recovery !== 'RETRY_WITH_BACKOFF') throw error;

      // Calculate delay: 2^attempt * baseDelay
      let delay = Math.min(Math.pow(2, attempt) * baseDelay, maxDelay);

      // Add jitter (random 0-20% of delay)
      if (jitter) {
        delay += Math.random() * delay * 0.2;
      }

      console.log(`Retry attempt ${attempt}/${maxAttempts} after ${delay}ms`);
      await sleep(delay);
    }
  }

  throw new Error('Max retries exceeded');
}
```

**Configuration:**
- Max attempts: 3-10
- Base delay: 1000ms
- Max delay: 30000ms
- Jitter: 10-20% of delay

### 3.2 FALLBACK

**Use Case:** Primary method failed but alternative exists.

**Implementation:**
```typescript
async function withFallback<T>(
  primary: () => Promise<T>,
  fallback: () => Promise<T>,
  options?: { timeout?: number }
): Promise<T> {
  try {
    return await primary();
  } catch (error) {
    if (error.recovery !== 'FALLBACK') throw error;

    console.log('Primary failed, using fallback');
    return await fallback();
  }
}

// Usage
const data = await withFallback(
  () => database.query('SELECT * FROM users'),
  () => cache.get('users')
);
```

**Fallback Hierarchy:**
1. In-memory cache (fastest)
2. Distributed cache (Redis, Memcached)
3. Read replica database
4. Backup service/region
5. Static fallback data
6. Degraded mode

### 3.3 CIRCUIT_BREAKER

**Use Case:** Service consistently failing, prevent cascading failures.

**States:**
- **CLOSED:** Normal operation, requests pass through
- **OPEN:** Failing, requests immediately rejected
- **HALF_OPEN:** Testing if service recovered

**Implementation:**
```typescript
class CircuitBreaker {
  private state: 'CLOSED' | 'OPEN' | 'HALF_OPEN' = 'CLOSED';
  private failureCount: number = 0;
  private lastFailureTime: number = 0;

  constructor(
    private options: {
      failureThreshold: number;      // Open after N failures
      timeout: number;               // Stay open for N ms
      resetTimeout: number;          // Test recovery after N ms
    }
  ) {}

  async execute<T>(operation: () => Promise<T>): Promise<T> {
    if (this.state === 'OPEN') {
      if (Date.now() - this.lastFailureTime > this.options.resetTimeout) {
        this.state = 'HALF_OPEN';
      } else {
        throw new WIAError({
          code: 'WIA-CIRCUIT-BREAKER-001',
          message: 'Circuit breaker is OPEN',
          severity: 'WARNING',
          recovery: 'FALLBACK'
        });
      }
    }

    try {
      const result = await operation();
      this.onSuccess();
      return result;
    } catch (error) {
      this.onFailure();
      throw error;
    }
  }

  private onSuccess() {
    this.failureCount = 0;
    this.state = 'CLOSED';
  }

  private onFailure() {
    this.failureCount++;
    this.lastFailureTime = Date.now();

    if (this.failureCount >= this.options.failureThreshold) {
      this.state = 'OPEN';
      console.log(`Circuit breaker OPEN after ${this.failureCount} failures`);
    }
  }
}
```

### 3.4 GRACEFUL_DEGRADATION

**Use Case:** Non-critical feature failed, continue with core functionality.

**Implementation:**
```typescript
class ServiceWithDegradation {
  async processOrder(order: Order): Promise<Result> {
    const result: Result = {
      order,
      features: {},
      degraded: false
    };

    // Core: Process payment (required)
    try {
      result.payment = await this.paymentService.process(order.total);
    } catch (error) {
      // Payment is critical - cannot degrade
      throw error;
    }

    // Enhancement: Apply recommendations (optional)
    try {
      result.features.recommendations = await this.recommendationService.get(order.userId);
    } catch (error) {
      if (error.recovery === 'GRACEFUL_DEGRADATION') {
        console.warn('Recommendations unavailable, proceeding without them');
        result.features.recommendations = [];
        result.degraded = true;
      }
    }

    // Enhancement: Send email (optional)
    try {
      await this.emailService.send(order);
    } catch (error) {
      if (error.recovery === 'GRACEFUL_DEGRADATION') {
        console.warn('Email service down, queueing for later');
        await this.queue.add('email', order);
        result.emailQueued = true;
      }
    }

    return result;
  }
}
```

### 3.5 FAIL_FAST

**Use Case:** Error cannot be recovered, fail immediately.

**Implementation:**
```typescript
function validateInput(data: unknown): ValidatedData {
  if (!data || typeof data !== 'object') {
    throw new WIAError({
      code: 'WIA-VALIDATE-INVALID-001',
      message: 'Invalid input data',
      severity: 'ERROR',
      recovery: 'FAIL_FAST'  // Don't retry invalid input
    });
  }

  // More validation...
  return data as ValidatedData;
}

// Usage
try {
  const validated = validateInput(userInput);
} catch (error) {
  if (error.recovery === 'FAIL_FAST') {
    // Return error to user immediately, no retry
    return res.status(400).json({ error: error.message });
  }
}
```

### 3.6 MANUAL

**Use Case:** Automated recovery not possible, requires human intervention.

**Implementation:**
```typescript
async function detectDataCorruption(fileId: string) {
  const file = await storage.get(fileId);
  const computed = await computeChecksum(file.data);

  if (computed !== file.expectedChecksum) {
    const error = new WIAError({
      code: 'WIA-STORAGE-CORRUPTION-001',
      message: 'File checksum mismatch',
      severity: 'CRITICAL',
      recovery: 'MANUAL',
      context: {
        fileId,
        expectedChecksum: file.expectedChecksum,
        computedChecksum: computed,
        fileSize: file.data.length
      }
    });

    // Alert operations team
    await alerting.page({
      severity: 'CRITICAL',
      title: 'Data corruption detected',
      error,
      runbook: 'https://wiki.company.com/data-corruption'
    });

    // Create incident ticket
    await ticketing.create({
      priority: 'P0',
      title: `Data corruption: ${fileId}`,
      assignee: 'data-integrity-team',
      error: error.toJSON()
    });

    throw error;
  }
}
```

---

## 4. Recovery Strategy Selection Guide

| Error Type | Primary Strategy | Fallback Strategy |
|------------|-----------------|-------------------|
| Network timeout | RETRY_WITH_BACKOFF | FALLBACK to cache |
| Database connection | RETRY_WITH_BACKOFF | CIRCUIT_BREAKER |
| Invalid input | FAIL_FAST | None |
| Rate limit | RETRY_WITH_BACKOFF | None |
| Service unavailable | CIRCUIT_BREAKER | FALLBACK to backup |
| Non-critical feature | GRACEFUL_DEGRADATION | None |
| Data corruption | MANUAL | None |
| Cache miss | FALLBACK to database | None |

---

## 5. Monitoring and Observability

### 5.1 Required Metrics

- **Error rate by recovery strategy**
- **Recovery success rate**
- **Circuit breaker state changes**
- **Retry attempts and success rate**
- **Fallback usage frequency**
- **Manual intervention incidents**

### 5.2 Logging Requirements

All errors MUST be logged with:
- Full error object (JSON serialized)
- Correlation ID
- Service name and version
- Timestamp (ISO 8601)
- Propagation path
- Recovery strategy applied
- Outcome (success/failure)

---

## 6. Testing Requirements

### 6.1 Recovery Strategy Tests

- Test each recovery strategy in isolation
- Verify retry backoff calculations
- Test circuit breaker state transitions
- Validate fallback execution
- Test graceful degradation paths

### 6.2 Chaos Engineering

- Inject failures to test recovery
- Verify correlation ID propagation
- Test cross-service error handling
- Validate monitoring and alerting

---

## 7. Next Steps

PHASE 2 completion enables:
- **PHASE 3:** Multi-language SDK implementations
- **PHASE 4:** Advanced features (ML-based recovery, predictive failure detection)

---

**Approval:** WIA Standards Committee
**Effective Date:** 2025-12-27
**Review Cycle:** Quarterly
