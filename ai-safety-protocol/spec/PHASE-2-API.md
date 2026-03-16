# WIA-AI-010 Phase 2: Safety Testing API
## AI Safety Protocol API Specification

**Version:** 1.0
**Status:** Official Standard
**Last Updated:** 2025-12-25

---

## Overview

This specification defines the API for AI safety testing, monitoring, and enforcement. The API provides programmatic access to safety testing tools, guardrail configuration, and continuous monitoring capabilities.

**弘益人間** (Benefit All Humanity) - Through accessible APIs, we democratize AI safety for all developers.

---

## 1. Core API Endpoints

### 1.1 Safety Testing API

#### Test Single Input
```
POST /api/v1/safety/test
```

**Request:**
```json
{
  "input": "text or structured data to test",
  "testTypes": ["adversarial", "toxicity", "bias", "privacy"],
  "config": {
    "strictness": "strict | balanced | permissive",
    "returnDetails": true
  }
}
```

**Response:**
```json
{
  "safe": true,
  "overallScore": 0.95,
  "results": {
    "adversarial": {
      "score": 0.98,
      "detections": [],
      "passed": true
    },
    "toxicity": {
      "score": 0.92,
      "categories": {
        "hate": 0.05,
        "violence": 0.03,
        "sexual": 0.02
      },
      "passed": true
    },
    "bias": {
      "score": 0.94,
      "fairnessMetrics": {},
      "passed": true
    },
    "privacy": {
      "score": 0.96,
      "piiDetected": false,
      "passed": true
    }
  },
  "recommendations": [],
  "processingTimeMs": 150
}
```

#### Batch Testing
```
POST /api/v1/safety/test/batch
```

**Request:**
```json
{
  "tests": [
    {"id": "1", "input": "test input 1"},
    {"id": "2", "input": "test input 2"}
  ],
  "testTypes": ["adversarial", "toxicity"],
  "config": {"strictness": "balanced"}
}
```

**Response:**
```json
{
  "results": [
    {"id": "1", "safe": true, "score": 0.95},
    {"id": "2", "safe": false, "score": 0.45, "violations": ["toxicity"]}
  ],
  "summary": {
    "total": 2,
    "passed": 1,
    "failed": 1
  }
}
```

### 1.2 Adversarial Testing API

#### Generate Adversarial Examples
```
POST /api/v1/adversarial/generate
```

**Request:**
```json
{
  "input": "original input",
  "attackTypes": ["fgsm", "pgd", "prompt-injection"],
  "epsilon": 0.03,
  "targetModel": "model-id"
}
```

**Response:**
```json
{
  "adversarialExamples": [
    {
      "attackType": "fgsm",
      "perturbedInput": "adversarial input",
      "perturbationNorm": 0.028,
      "successfulAttack": false
    }
  ],
  "robustnessScore": 0.92
}
```

#### Test Robustness
```
POST /api/v1/adversarial/test-robustness
```

**Request:**
```json
{
  "modelEndpoint": "https://api.example.com/model",
  "testSet": "reference to test dataset",
  "attacks": ["fgsm", "pgd", "carlini-wagner"],
  "epsilonValues": [0.01, 0.03, 0.1]
}
```

**Response:**
```json
{
  "robustnessReport": {
    "cleanAccuracy": 0.95,
    "adversarialAccuracy": {
      "fgsm_0.01": 0.89,
      "fgsm_0.03": 0.82,
      "pgd_0.03": 0.78
    },
    "certification": "passed",
    "benchmark": "WIA-Robustness-v1"
  }
}
```

### 1.3 Content Filtering API

#### Filter Content
```
POST /api/v1/content/filter
```

**Request:**
```json
{
  "content": "text to filter",
  "categories": ["toxicity", "hate", "violence", "sexual", "pii"],
  "threshold": 0.7,
  "redactPII": true
}
```

**Response:**
```json
{
  "allowed": true,
  "filtered": false,
  "scores": {
    "toxicity": 0.12,
    "hate": 0.05,
    "violence": 0.08,
    "sexual": 0.03
  },
  "piiDetections": [],
  "filteredContent": "original content (or redacted)",
  "reason": null
}
```

### 1.4 Alignment Verification API

#### Verify Alignment
```
POST /api/v1/alignment/verify
```

**Request:**
```json
{
  "prompt": "user prompt",
  "response": "model response",
  "principles": ["helpful", "harmless", "honest"],
  "context": {}
}
```

**Response:**
```json
{
  "aligned": true,
  "scores": {
    "helpful": 0.92,
    "harmless": 0.98,
    "honest": 0.94
  },
  "violations": [],
  "recommendations": []
}
```

### 1.5 Red Team Testing API

#### Submit Attack Attempt
```
POST /api/v1/redteam/attack
```

**Request:**
```json
{
  "attackVector": "prompt-injection",
  "payload": "adversarial prompt",
  "goal": "bypass-safety-filters",
  "metadata": {
    "tester": "red-team-alpha",
    "technique": "multi-turn-manipulation"
  }
}
```

**Response:**
```json
{
  "attemptId": "uuid",
  "blocked": true,
  "detectedBy": ["input-guardrail", "content-filter"],
  "severity": "high",
  "logged": true,
  "feedback": "Attack blocked at input validation layer"
}
```

---

## 2. Guardrail Configuration API

### 2.1 Configure Guardrails
```
POST /api/v1/guardrails/configure
```

**Request:**
```json
{
  "guardrails": {
    "input": {
      "enabled": true,
      "rules": [
        {
          "type": "prompt-injection-detection",
          "threshold": 0.8,
          "action": "block"
        }
      ]
    },
    "output": {
      "enabled": true,
      "rules": [
        {
          "type": "toxicity-filter",
          "threshold": 0.7,
          "action": "filter"
        }
      ]
    },
    "rate-limiting": {
      "requestsPerHour": 1000,
      "requestsPerDay": 10000
    }
  }
}
```

**Response:**
```json
{
  "configId": "uuid",
  "applied": true,
  "activeGuardrails": 8,
  "effectiveFrom": "2025-12-25T00:00:00Z"
}
```

### 2.2 Test Guardrails
```
POST /api/v1/guardrails/test
```

**Request:**
```json
{
  "configId": "uuid",
  "testCases": [
    {"input": "benign input", "expectedOutcome": "allow"},
    {"input": "malicious input", "expectedOutcome": "block"}
  ]
}
```

**Response:**
```json
{
  "testResults": [
    {"passed": true, "outcome": "allow"},
    {"passed": true, "outcome": "block"}
  ],
  "accuracy": 1.0,
  "falsePositives": 0,
  "falseNegatives": 0
}
```

---

## 3. Monitoring and Analytics API

### 3.1 Get Safety Metrics
```
GET /api/v1/metrics/safety?timeRange=24h
```

**Response:**
```json
{
  "timeRange": {
    "start": "2025-12-24T00:00:00Z",
    "end": "2025-12-25T00:00:00Z"
  },
  "metrics": {
    "totalRequests": 100000,
    "blockedRequests": 150,
    "blockRate": 0.0015,
    "safetyScore": 99.85,
    "categories": {
      "toxicity": {"violations": 45, "rate": 0.00045},
      "adversarial": {"violations": 67, "rate": 0.00067},
      "pii": {"violations": 38, "rate": 0.00038}
    },
    "latency": {
      "p50": 85,
      "p95": 180,
      "p99": 250
    }
  }
}
```

### 3.2 Get Incident Reports
```
GET /api/v1/incidents?status=open&severity=high
```

**Response:**
```json
{
  "incidents": [
    {
      "id": "INC-2025-0001",
      "severity": "high",
      "status": "investigating",
      "occurred": "2025-12-25T10:30:00Z",
      "category": "safety-violation",
      "summary": "Multiple adversarial attack attempts detected"
    }
  ],
  "total": 1,
  "page": 1
}
```

---

## 4. Benchmark API

### 4.1 Run Benchmark
```
POST /api/v1/benchmark/run
```

**Request:**
```json
{
  "benchmarkName": "WIA-Safety-Bench-v1",
  "tier": 2,
  "modelEndpoint": "https://api.example.com/model",
  "config": {
    "parallel": true,
    "maxConcurrency": 10
  }
}
```

**Response:**
```json
{
  "benchmarkId": "uuid",
  "status": "running",
  "estimatedCompletionTime": "2025-12-25T12:00:00Z",
  "progress": 0,
  "resultsUrl": "/api/v1/benchmark/results/uuid"
}
```

### 4.2 Get Benchmark Results
```
GET /api/v1/benchmark/results/{benchmarkId}
```

**Response:**
```json
{
  "benchmarkId": "uuid",
  "status": "completed",
  "report": {
    "overallScore": 93.5,
    "categoryScores": {
      "robustness": 91,
      "alignment": 95,
      "toxicity": 94
    },
    "certification": "certified",
    "detailedResults": "full benchmark report object"
  }
}
```

---

## 5. Authentication and Authorization

### 5.1 API Key Authentication
```
Authorization: Bearer YOUR_API_KEY
```

### 5.2 Rate Limiting
- Free tier: 100 requests/hour
- Professional: 1,000 requests/hour
- Enterprise: Custom limits

### 5.3 Scopes
- `safety:read` - Read safety data
- `safety:test` - Run safety tests
- `guardrails:configure` - Configure guardrails
- `benchmark:run` - Run benchmarks
- `admin` - Full access

---

## 6. SDKs and Client Libraries

Official SDKs available for:
- TypeScript/JavaScript
- Python
- Rust
- Go
- Java

Example (TypeScript):
```typescript
import { SafetyProtocol } from '@wia/ai-safety-protocol';

const client = new SafetyProtocol({
  apiKey: process.env.WIA_API_KEY
});

const result = await client.safety.test({
  input: "Test input",
  testTypes: ["adversarial", "toxicity"]
});

console.log(result.safe); // true/false
```

---

## 7. Webhooks

### 7.1 Configure Webhook
```
POST /api/v1/webhooks
```

**Request:**
```json
{
  "url": "https://your-server.com/webhook",
  "events": ["safety-violation", "high-severity-incident"],
  "secret": "webhook-secret-for-verification"
}
```

### 7.2 Webhook Payload
```json
{
  "event": "safety-violation",
  "timestamp": "2025-12-25T12:00:00Z",
  "data": {
    "violationType": "toxicity",
    "severity": "high",
    "details": {}
  },
  "signature": "hmac-sha256-signature"
}
```

---

## Error Handling

### Standard Error Response
```json
{
  "error": {
    "code": "INVALID_INPUT",
    "message": "The provided input failed validation",
    "details": {},
    "requestId": "uuid",
    "documentation": "https://docs.wia.org/errors/INVALID_INPUT"
  }
}
```

### Error Codes
- `INVALID_INPUT` - Input validation failed
- `RATE_LIMIT_EXCEEDED` - Too many requests
- `AUTHENTICATION_FAILED` - Invalid API key
- `INSUFFICIENT_PERMISSIONS` - Missing required scopes
- `INTERNAL_ERROR` - Server error
- `SERVICE_UNAVAILABLE` - Temporary outage

---

**弘益人間** - Accessible APIs enable developers worldwide to build safer AI systems.

© 2025 SmileStory Inc. / WIA
WIA-AI-010 Phase 2: Safety Testing API v1.0
