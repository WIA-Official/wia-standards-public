# ⚠️ WIA-CORE-010: Universal Error Handling

[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)]()
[![License](https://img.shields.io/badge/license-MIT-green.svg)]()
[![Standard](https://img.shields.io/badge/WIA-Standard-purple.svg)]()


> **Standard ID:** WIA-CORE-010
> **Version:** 1.0.0
> **Status:** Active
> **Category:** CORE / Universal Integration
> **Color:** Indigo (#6366F1)

---

## 🌟 Overview

The WIA-CORE-010 standard defines a comprehensive framework for universal error handling across all WIA systems. It provides standardized error codes, error propagation mechanisms, recovery strategies, and reporting protocols to ensure consistent error management throughout the entire WIA ecosystem.

**홍익인간 (弘益人間) - Benefit All Humanity** - This standard aims to create a robust, predictable error handling system that benefits developers and users by providing clear, actionable error information while maintaining system stability and reliability.

## 🎯 Key Features

- **Universal Error Codes**: Standardized error code system across all WIA standards
- **Error Categorization**: Hierarchical error taxonomy with severity levels
- **Error Propagation**: Consistent error flow through system boundaries
- **Recovery Strategies**: Built-in error recovery and retry mechanisms
- **Context Preservation**: Rich error context with stack traces and metadata
- **Multi-Language Support**: Error messages in multiple languages
- **Integration Ready**: Seamless integration with logging, monitoring, and alerting systems

## 📊 Core Concepts

### 1. Error Code Structure

```
WIA-{DOMAIN}-{CATEGORY}-{NUMBER}
```

Examples:
- `WIA-CORE-AUTH-001`: Authentication failure
- `WIA-CORE-NETWORK-502`: Bad gateway
- `WIA-CORE-VALIDATION-100`: Invalid input parameter

### 2. Error Severity Levels

```
CRITICAL   (5): System failure, immediate action required
ERROR      (4): Operation failed, user intervention needed
WARNING    (3): Potential issue, degraded functionality
INFO       (2): Informational, no action required
DEBUG      (1): Debugging information
```

### 3. Error Response Format

```json
{
  "error": {
    "code": "WIA-CORE-AUTH-001",
    "message": "Authentication failed",
    "severity": "ERROR",
    "timestamp": "2025-12-27T00:00:00Z",
    "context": {},
    "stack": [],
    "recovery": {}
  }
}
```

## 🔧 Components

### TypeScript SDK

```typescript
import {
  ErrorHandler,
  WIAError,
  createError,
  handleError,
  recoverFromError
} from '@wia/core-010';

// Create standardized error
const error = createError({
  code: 'WIA-CORE-AUTH-001',
  message: 'Invalid credentials',
  severity: 'ERROR',
  context: { userId: '12345' }
});

// Handle error with recovery
const result = await handleError(error, {
  retry: { maxAttempts: 3, backoff: 'exponential' },
  fallback: () => defaultValue,
  notify: true
});

// Error handler instance
const handler = new ErrorHandler({
  logLevel: 'ERROR',
  reportErrors: true,
  includeStackTrace: true
});

handler.catch(error);
```

### CLI Tool

```bash
# Validate error code
wia-core-010 validate --code "WIA-CORE-AUTH-001"

# List all error codes
wia-core-010 list --domain CORE --category AUTH

# Parse error from JSON
wia-core-010 parse --file error.json

# Test error recovery
wia-core-010 test-recovery --code "WIA-CORE-NETWORK-502"

# Generate error documentation
wia-core-010 docs --output ./error-docs
```

## 📚 Documentation

| Document | Description |
|----------|-------------|
| [WIA-CORE-010-v1.0.md](./spec/WIA-CORE-010-v1.0.md) | Complete specification with error taxonomy |
| [TypeScript SDK](./api/typescript/) | Full TypeScript implementation |
| [CLI Tool](./cli/wia-core-010.sh) | Command-line interface |

## 🚀 Quick Start

### Installation

```bash
# Clone repository
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/standards/universal-error-handling

# Run installation script
./install.sh

# Verify installation
wia-core-010 --version
```

### TypeScript Usage

```bash
# Install via npm
npm install @wia/core-010

# Or yarn
yarn add @wia/core-010
```

```typescript
import { WIAError, ErrorHandler } from '@wia/core-010';

// Create custom error handler
const errorHandler = new ErrorHandler({
  logLevel: 'ERROR',
  reportErrors: true,
  onError: (error) => {
    console.error('Error occurred:', error.toJSON());
  }
});

// Use in try-catch
try {
  throw new WIAError({
    code: 'WIA-CORE-VALIDATION-100',
    message: 'Invalid input: age must be positive',
    severity: 'ERROR',
    context: { field: 'age', value: -5 }
  });
} catch (error) {
  const handled = errorHandler.handle(error);
  if (handled.recovered) {
    console.log('Error recovered:', handled.result);
  }
}
```

## 🔍 Error Code Categories

### CORE Categories

| Category | Code Range | Description |
|----------|------------|-------------|
| AUTH | 001-099 | Authentication and authorization errors |
| VALIDATION | 100-199 | Input validation and data verification errors |
| NETWORK | 200-299 | Network and communication errors |
| DATABASE | 300-399 | Database and data persistence errors |
| RESOURCE | 400-499 | Resource management errors (memory, disk, etc.) |
| PERMISSION | 500-599 | Permission and access control errors |
| SYSTEM | 600-699 | System-level errors and failures |
| EXTERNAL | 700-799 | Third-party service and integration errors |
| BUSINESS | 800-899 | Business logic and domain errors |
| UNKNOWN | 900-999 | Unclassified or unexpected errors |

## ⚠️ Best Practices

### 1. Error Creation

```typescript
// Good: Specific, actionable error
const error = createError({
  code: 'WIA-CORE-VALIDATION-101',
  message: 'Email address must be valid',
  severity: 'ERROR',
  context: {
    field: 'email',
    value: 'invalid-email',
    expected: 'RFC 5322 compliant email'
  },
  recovery: {
    suggestion: 'Please provide a valid email address',
    retryable: false
  }
});

// Bad: Vague, unhelpful error
const error = new Error('Something went wrong');
```

### 2. Error Propagation

```typescript
// Good: Preserve error context while adding information
try {
  await riskyOperation();
} catch (originalError) {
  throw createError({
    code: 'WIA-CORE-SYSTEM-601',
    message: 'Operation failed in user service',
    severity: 'ERROR',
    cause: originalError,
    context: { service: 'UserService', operation: 'createUser' }
  });
}
```

### 3. Error Recovery

```typescript
// Good: Graceful degradation with recovery
const result = await handleError(error, {
  retry: {
    maxAttempts: 3,
    backoff: 'exponential',
    condition: (err) => err.code.includes('NETWORK')
  },
  fallback: async () => {
    // Return cached data or default value
    return await getCachedData();
  },
  onRetry: (attempt) => {
    console.log(`Retry attempt ${attempt}`);
  }
});
```

## 🌐 WIA Integration

This standard integrates with:
- **WIA-INTENT**: Intent-based error recovery suggestions
- **WIA-OMNI-API**: Universal error response format
- **WIA-SOCIAL**: Error reporting and community support
- **WIA-LOGGING**: Centralized error logging and monitoring
- **WIA-TELEMETRY**: Error metrics and analytics

## 📖 Use Cases

1. **API Development**: Consistent error responses across all API endpoints
2. **Microservices**: Standardized error propagation between services
3. **User Interfaces**: User-friendly error messages with recovery suggestions
4. **System Monitoring**: Automated error detection and alerting
5. **Debugging**: Rich error context for faster issue resolution
6. **Multi-Language Apps**: Internationalized error messages
7. **Distributed Systems**: Error tracking across service boundaries

## 🔒 Error Security

### Sensitive Data Handling

```typescript
// Automatically sanitize sensitive data
const error = createError({
  code: 'WIA-CORE-AUTH-001',
  message: 'Authentication failed',
  context: {
    password: 'secret123',  // Will be redacted
    token: 'abc123xyz'      // Will be redacted
  },
  sanitize: true  // Enable automatic sanitization
});

// Output: { password: '[REDACTED]', token: '[REDACTED]' }
```

### Error Rate Limiting

```typescript
// Prevent error spam and DoS
const handler = new ErrorHandler({
  rateLimit: {
    maxErrors: 100,
    window: 60000,  // 1 minute
    action: 'throttle'
  }
});
```

## 📊 Error Metrics

Track error patterns and trends:

```typescript
import { ErrorMetrics } from '@wia/core-010';

const metrics = new ErrorMetrics();

// Record error
metrics.recordError(error);

// Get statistics
const stats = metrics.getStats();
console.log(stats);
// {
//   totalErrors: 150,
//   errorsByCode: { 'WIA-CORE-AUTH-001': 45, ... },
//   errorsBySeverity: { 'ERROR': 100, 'WARNING': 50 },
//   errorRate: 2.5  // errors per minute
// }
```

## 🤝 Contributing

Contributions welcome! Please see our [Contributing Guide](https://github.com/WIA-Official/wia-standards/CONTRIBUTING.md).

## 📄 License

MIT License - see [LICENSE](https://github.com/WIA-Official/wia-standards/LICENSE)

## 🔗 Links

- **Website**: [wiastandards.com](https://wiastandards.com)
- **GitHub**: [github.com/WIA-Official/wia-standards](https://github.com/WIA-Official/wia-standards)
- **Documentation**: [docs.wiastandards.com](https://docs.wiastandards.com)
- **Error Code Registry**: [errors.wiastandards.com](https://errors.wiastandards.com)

---

**홍익인간 (弘益人間) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 SmileStory Inc. / WIA*
*MIT License*

---

**홍익인간 (弘益人間) - Benefit All Humanity** 🌍
