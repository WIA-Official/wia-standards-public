# WIA-CORE-010 PHASE 1 Specification
## Core Error Codes and Severity Levels

**Version:** 1.0.0
**Status:** Active
**Date:** 2025-12-27

---

## 1. Overview

PHASE 1 establishes the foundational elements of WIA-CORE-010: universal error codes and severity levels. This phase defines the core vocabulary that all subsequent phases build upon.

**Objectives:**
- Define error code structure and naming conventions
- Establish four severity levels (CRITICAL, ERROR, WARNING, INFO)
- Create initial catalog of 100+ standard error codes
- Provide reference implementations in 5 core languages

**Deliverables:**
- Error code specification document
- Severity level classification guide
- Standard error code catalog (100+ codes)
- Reference implementations (JavaScript, Python, Go, Rust, Java)

---

## 2. Error Code Structure

### 2.1 Format Specification

```
WIA-{DOMAIN}-{CATEGORY}-{NUMBER}
```

**Components:**
- **WIA**: Standard prefix (3 chars, uppercase)
- **DOMAIN**: System area (3-10 chars, uppercase, alpha only)
- **CATEGORY**: Error type (3-15 chars, uppercase, alpha/underscore)
- **NUMBER**: Unique identifier (001-999, zero-padded)

**Examples:**
```
WIA-AUTH-INVALID-001        ✓ Valid
WIA-DB-TIMEOUT-015          ✓ Valid
WIA-NET-CONNECTION-008      ✓ Valid
WIA-API-RATE_LIMIT-007      ✓ Valid

WIA_AUTH_INVALID_001        ✗ Invalid (underscores instead of hyphens)
WIA-AUTH-INVALID-1          ✗ Invalid (not zero-padded)
WIA-AUTHENTICATION-INV-001  ✗ Invalid (domain too long)
```

### 2.2 Domain Definitions

| Domain | Description | Use Cases |
|--------|-------------|-----------|
| AUTH | Authentication & Authorization | Login failures, token validation, permissions |
| DB | Database Operations | Query failures, connection issues, constraints |
| NET | Network Operations | Timeouts, connection failures, DNS errors |
| FILE | File System Operations | File not found, permission denied, disk full |
| API | API Operations | Rate limiting, invalid requests, service unavailable |
| VALIDATE | Data Validation | Invalid format, missing fields, constraint violations |
| CRYPTO | Cryptographic Operations | Encryption failures, signature verification |
| CACHE | Caching Operations | Cache misses, eviction, invalidation |
| QUEUE | Message Queue Operations | Queue full, message expired, delivery failure |
| STORAGE | Storage Operations | Upload failures, quota exceeded, corruption |

### 2.3 Category Definitions

| Category | Meaning | When to Use |
|----------|---------|-------------|
| INVALID | Input doesn't meet requirements | Invalid credentials, malformed data |
| NOTFOUND | Requested resource doesn't exist | 404 scenarios, missing records |
| TIMEOUT | Operation exceeded time limit | Network timeouts, query timeouts |
| CONNECTION | Failed to establish connection | Database connection, network connection |
| PERMISSION | Insufficient permissions | Authorization failures, ACL violations |
| CONFLICT | Resource state conflict | Unique constraint violations, optimistic locking |
| RATE_LIMIT | Too many requests | API throttling, DDoS protection |
| INTERNAL | Unexpected internal error | Unhandled exceptions, bugs |

### 2.4 Number Allocation

**Ranges:**
- 001-099: Reserved for WIA standard errors
- 100-899: Available for organization-specific custom errors
- 900-999: Reserved for experimental/temporary errors

**Allocation Rules:**
1. Numbers are assigned sequentially within each domain-category combination
2. Once assigned, error codes are immutable (never change meaning)
3. Deprecated codes remain in catalog with deprecation notice
4. New variants get next available number in sequence

---

## 3. Severity Levels

### 3.1 Severity Definitions

#### CRITICAL 🔴
**Definition:** System failure or data loss scenarios requiring immediate human intervention.

**Characteristics:**
- Service is completely unavailable or severely degraded
- Data integrity at risk
- Immediate financial or reputational impact
- Affects all or most users

**Response:**
- Response time: < 5 minutes
- Notification: Page on-call engineer, executive escalation
- Action: Immediate investigation and remediation

**Examples:**
```
WIA-DB-CONNECTION-001 (CRITICAL)
- Database server completely unavailable
- All write operations failing
- Potential data loss

WIA-STORAGE-CORRUPTION-001 (CRITICAL)
- Data corruption detected
- File checksums don't match
- Requires manual data recovery

WIA-CRYPTO-KEY_COMPROMISED-001 (CRITICAL)
- Security breach detected
- Encryption keys potentially compromised
- Immediate key rotation required
```

#### ERROR 🟠
**Definition:** Operation failed but overall system remains functional. Users are impacted but service is available for other operations.

**Characteristics:**
- Specific feature or operation failed
- Affects individual users or requests
- System continues operating
- May be recoverable with retry

**Response:**
- Response time: Within 1 hour
- Notification: Alert team, create ticket
- Action: Investigate and fix, may retry automatically

**Examples:**
```
WIA-AUTH-INVALID-001 (ERROR)
- Invalid username or password
- User cannot login
- Other users unaffected

WIA-DB-TIMEOUT-015 (ERROR)
- Database query timeout
- Single request failed
- Database still operational

WIA-API-RATE_LIMIT-007 (ERROR)
- User exceeded rate limit
- Temporary restriction
- Retry after cooldown period
```

#### WARNING 🟡
**Definition:** Potential issues that haven't caused failures yet but might lead to problems. System continues operating but performance may be degraded.

**Characteristics:**
- No immediate user impact
- Performance degradation possible
- Approaching resource limits
- Requires monitoring

**Response:**
- Response time: Within 24 hours
- Notification: Daily digest, team dashboard
- Action: Review and plan remediation

**Examples:**
```
WIA-DB-CONNECTION-002 (WARNING)
- Database connection pool at 80% capacity
- Performance may degrade soon
- Consider scaling up

WIA-CACHE-MISS-012 (WARNING)
- Cache miss rate above threshold
- Increased database load
- May need cache warming

WIA-API-DEPRECATED-003 (WARNING)
- Using deprecated API version
- Still functional but will be removed
- Migrate to new version
```

#### INFO 🔵
**Definition:** Informational events providing operational visibility. Not errors in traditional sense but important state changes worth logging.

**Characteristics:**
- Expected behavior
- No user impact
- Operational visibility
- Used for analytics and auditing

**Response:**
- Response time: Review during retrospectives
- Notification: Logged only, aggregated in metrics
- Action: None required

**Examples:**
```
WIA-CACHE-MISS-001 (INFO)
- Cache miss on cold cache
- Expected behavior
- No action needed

WIA-FILE-NOTFOUND-003 (INFO)
- User requested non-existent file
- Valid query with empty result
- Informational only

WIA-API-FALLBACK-001 (INFO)
- Fallback mechanism activated
- Graceful degradation working
- System operating normally
```

---

## 4. Standard Error Catalog

### 4.1 Authentication Errors (AUTH)

```markdown
WIA-AUTH-INVALID-001
Description: Invalid username or password
Severity: ERROR
Recovery: FAIL_FAST
HTTP Status: 401
Message: "Invalid username or password. Please check your credentials and try again."

WIA-AUTH-INVALID-002
Description: Expired authentication token
Severity: ERROR
Recovery: RETRY_WITH_BACKOFF
HTTP Status: 401
Message: "Your session has expired. Please login again."

WIA-AUTH-INVALID-003
Description: Malformed JWT token
Severity: ERROR
Recovery: FAIL_FAST
HTTP Status: 401
Message: "Invalid authentication token format."

WIA-AUTH-PERMISSION-001
Description: Insufficient permissions for operation
Severity: ERROR
Recovery: FAIL_FAST
HTTP Status: 403
Message: "You don't have permission to perform this action."

WIA-AUTH-PERMISSION-002
Description: Role not authorized
Severity: ERROR
Recovery: FAIL_FAST
HTTP Status: 403
Message: "Your user role is not authorized for this resource."

WIA-AUTH-RATE_LIMIT-001
Description: Too many authentication attempts
Severity: WARNING
Recovery: RETRY_WITH_BACKOFF
HTTP Status: 429
Message: "Too many login attempts. Please wait before trying again."
```

### 4.2 Database Errors (DB)

```markdown
WIA-DB-CONNECTION-001
Description: Failed to connect to database
Severity: CRITICAL
Recovery: RETRY_WITH_BACKOFF
HTTP Status: 503
Message: "Database service temporarily unavailable."

WIA-DB-CONNECTION-002
Description: Connection pool exhausted
Severity: CRITICAL
Recovery: CIRCUIT_BREAKER
HTTP Status: 503
Message: "Database connection pool exhausted. Service degraded."

WIA-DB-TIMEOUT-015
Description: Query execution timeout
Severity: ERROR
Recovery: RETRY_WITH_BACKOFF
HTTP Status: 504
Message: "Database query timeout. Please try again."

WIA-DB-NOTFOUND-001
Description: Record not found
Severity: INFO
Recovery: FAIL_FAST
HTTP Status: 404
Message: "Requested record does not exist."

WIA-DB-CONFLICT-001
Description: Unique constraint violation
Severity: ERROR
Recovery: FAIL_FAST
HTTP Status: 409
Message: "Record already exists with these values."

WIA-DB-CONFLICT-002
Description: Optimistic locking failure
Severity: ERROR
Recovery: RETRY_WITH_BACKOFF
HTTP Status: 409
Message: "Record was modified by another process. Please refresh and try again."
```

### 4.3 Network Errors (NET)

```markdown
WIA-NET-TIMEOUT-015
Description: Network request timeout
Severity: ERROR
Recovery: RETRY_WITH_BACKOFF
HTTP Status: 504
Message: "Network request timed out. Please try again."

WIA-NET-CONNECTION-008
Description: Connection refused
Severity: CRITICAL
Recovery: CIRCUIT_BREAKER
HTTP Status: 503
Message: "Unable to connect to remote service."

WIA-NET-CONNECTION-009
Description: DNS resolution failed
Severity: CRITICAL
Recovery: FALLBACK
HTTP Status: 503
Message: "DNS resolution failed for service endpoint."

WIA-NET-INVALID-001
Description: Invalid URL format
Severity: ERROR
Recovery: FAIL_FAST
HTTP Status: 400
Message: "Invalid URL format provided."
```

---

## 5. Implementation Guidelines

### 5.1 Error Object Structure

All WIA errors MUST include:

```json
{
  "code": "WIA-DOMAIN-CATEGORY-NUMBER",
  "message": "Human-readable error message",
  "severity": "CRITICAL|ERROR|WARNING|INFO",
  "timestamp": "ISO 8601 timestamp",
  "correlationId": "Unique request/trace ID",
  "recovery": "Recovery strategy",
  "context": {
    // Additional error-specific data
  },
  "stackTrace": "Optional stack trace",
  "causedBy": {
    // Optional nested original error
  }
}
```

### 5.2 Validation Rules

Implementations MUST validate:
1. Error code matches WIA-{DOMAIN}-{CATEGORY}-{NUMBER} format
2. Severity is one of: CRITICAL, ERROR, WARNING, INFO
3. Timestamp is valid ISO 8601 format
4. Correlation ID is present and non-empty
5. Recovery strategy is valid

### 5.3 Backward Compatibility

- Error codes are immutable once published
- New codes can be added without breaking changes
- Deprecated codes remain in catalog with notice
- Major version changes can reorganize code space

---

## 6. Testing Requirements

### 6.1 Unit Tests

- Validate error code format
- Verify severity levels
- Test error serialization/deserialization
- Validate required fields

### 6.2 Integration Tests

- Error propagation across services
- Correlation ID preservation
- HTTP status code mapping
- Error logging and monitoring

---

## 7. Metrics and Monitoring

Required metrics:
- Error count by code
- Error count by severity
- Error distribution by service
- Mean time to resolution (MTTR)
- Recovery strategy success rate

---

## 8. Next Steps

PHASE 1 completion enables:
- **PHASE 2:** Error propagation and recovery strategies
- **PHASE 3:** Multi-language SDKs
- **PHASE 4:** Advanced features and ecosystem

---

**Approval:** WIA Standards Committee
**Effective Date:** 2025-12-27
**Review Cycle:** Quarterly
