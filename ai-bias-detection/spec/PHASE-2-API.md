# WIA-AI-013: AI Bias Detection Standard
## Phase 2: API Specification

**Version:** 1.0
**Status:** Final
**Last Updated:** 2025-01-08
**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

This specification defines REST APIs for AI bias detection services. The API enables standardized access to bias detection, fairness assessment, and model auditing capabilities.

### 1.1 Base Principles

- **RESTful Design**: Standard HTTP methods and status codes
- **JSON Communication**: Request and response bodies in JSON
- **Stateless**: Each request contains all necessary information
- **Versioned**: API version in URL path
- **Secure**: HTTPS required, authentication mandatory

### 1.2 Base URL

```
https://api.example.com/wia-ai-013/v1
```

---

## 2. Authentication

### 2.1 API Key Authentication

```http
Authorization: Bearer <api_key>
```

### 2.2 OAuth 2.0

Supported for enterprise integrations:

```http
Authorization: Bearer <access_token>
```

---

## 3. Core Endpoints

### 3.1 Bias Detection

#### Analyze Model Bias

```http
POST /bias/analyze
Content-Type: application/json
Authorization: Bearer <api_key>

{
  "modelId": "string",
  "datasetId": "string",
  "protectedAttributes": ["gender", "race"],
  "fairnessMetrics": ["demographic_parity", "equalized_odds"],
  "options": {
    "includeIntersectional": true,
    "thresholds": {
      "demographicParityRatio": 0.8
    }
  }
}
```

**Response (202 Accepted):**

```json
{
  "jobId": "job-12345",
  "status": "processing",
  "estimatedCompletionTime": "2025-01-08T10:35:00Z",
  "_links": {
    "self": "/bias/jobs/job-12345",
    "cancel": "/bias/jobs/job-12345/cancel"
  }
}
```

#### Get Analysis Results

```http
GET /bias/jobs/{jobId}
Authorization: Bearer <api_key>
```

**Response (200 OK):**

```json
{
  "jobId": "job-12345",
  "status": "completed",
  "result": {
    /* BiasDetectionReport as defined in Phase 1 */
  },
  "completedAt": "2025-01-08T10:34:30Z"
}
```

### 3.2 Fairness Metrics

#### Calculate Specific Metric

```http
POST /metrics/{metricName}
Content-Type: application/json

{
  "predictions": [0, 1, 1, 0, ...],
  "actualLabels": [0, 1, 0, 0, ...],
  "protectedAttribute": [0, 0, 1, 1, ...],
  "groups": {
    "0": "male",
    "1": "female"
  }
}
```

**Response (200 OK):**

```json
{
  "metric": "demographic_parity_ratio",
  "value": 0.85,
  "threshold": 0.8,
  "status": "pass",
  "byGroup": {
    "male": {"positiveRate": 0.45, "sampleSize": 500},
    "female": {"positiveRate": 0.38, "sampleSize": 500}
  },
  "calculatedAt": "2025-01-08T10:30:00Z"
}
```

#### Get Available Metrics

```http
GET /metrics
```

**Response (200 OK):**

```json
{
  "metrics": [
    {
      "name": "demographic_parity",
      "displayName": "Demographic Parity",
      "description": "Requires equal positive prediction rates across groups",
      "category": "group_fairness",
      "defaultThreshold": 0.8
    },
    {
      "name": "equalized_odds",
      "displayName": "Equalized Odds",
      "description": "Requires equal TPR and FPR across groups",
      "category": "group_fairness",
      "defaultThreshold": 0.1
    }
  ]
}
```

### 3.3 Dataset Analysis

#### Analyze Dataset for Bias

```http
POST /datasets/analyze
Content-Type: application/json

{
  "datasetId": "string",
  "protectedAttributes": ["gender", "race"],
  "checks": [
    "representation_bias",
    "label_bias",
    "proxy_features"
  ]
}
```

**Response (200 OK):**

```json
{
  "datasetId": "dataset-001",
  "representationBias": {
    "status": "warning",
    "findings": [
      {
        "attribute": "gender",
        "imbalanceRatio": 0.65,
        "recommendation": "Increase female representation"
      }
    ]
  },
  "labelBias": {
    "status": "pass",
    "findings": []
  },
  "proxyFeatures": {
    "status": "fail",
    "findings": [
      {
        "feature": "zipcode",
        "correlatesWithprotectedAttribute": "race",
        "correlation": 0.78,
        "recommendation": "Review or remove"
      }
    ]
  }
}
```

### 3.4 Model Monitoring

#### Register Model for Monitoring

```http
POST /monitoring/models
Content-Type: application/json

{
  "modelId": "string",
  "modelName": "string",
  "protectedAttributes": ["gender", "race"],
  "monitoringConfig": {
    "frequency": "daily",
    "metrics": ["demographic_parity", "equalized_odds"],
    "thresholds": {
      "demographicParityRatio": 0.8
    },
    "alertWebhook": "https://example.com/alerts"
  }
}
```

**Response (201 Created):**

```json
{
  "monitoringId": "mon-12345",
  "modelId": "model-001",
  "status": "active",
  "nextScheduledRun": "2025-01-09T00:00:00Z",
  "_links": {
    "self": "/monitoring/models/mon-12345",
    "metrics": "/monitoring/models/mon-12345/metrics"
  }
}
```

#### Get Monitoring Metrics

```http
GET /monitoring/models/{monitoringId}/metrics
```

**Response (200 OK):**

```json
{
  "monitoringId": "mon-12345",
  "modelId": "model-001",
  "timeRange": {
    "start": "2025-01-01T00:00:00Z",
    "end": "2025-01-08T00:00:00Z"
  },
  "metrics": [
    {
      "timestamp": "2025-01-08T00:00:00Z",
      "demographicParity": 0.85,
      "equalizedOdds": 0.08,
      "status": "pass"
    },
    {
      "timestamp": "2025-01-07T00:00:00Z",
      "demographicParity": 0.87,
      "equalizedOdds": 0.09,
      "status": "pass"
    }
  ],
  "alerts": [
    {
      "timestamp": "2025-01-06T12:30:00Z",
      "metric": "demographic_parity",
      "value": 0.75,
      "threshold": 0.8,
      "severity": "warning"
    }
  ]
}
```

### 3.5 Mitigation Strategies

#### Get Mitigation Recommendations

```http
POST /mitigation/recommend
Content-Type: application/json

{
  "biasType": "demographic",
  "phase": "pre-processing",
  "context": {
    "industry": "finance",
    "useCase": "lending",
    "currentMetrics": {
      "demographicParity": 0.65
    }
  }
}
```

**Response (200 OK):**

```json
{
  "recommendations": [
    {
      "id": "rec-001",
      "strategy": "reweighting",
      "phase": "pre-processing",
      "description": "Apply sample weights to balance representation",
      "expectedImpact": "Improve demographic parity by 0.15-0.20",
      "implementationEffort": "medium",
      "code Example": "https://docs.wia.org/examples/reweighting",
      "applicability": 0.9
    },
    {
      "id": "rec-002",
      "strategy": "threshold_optimization",
      "phase": "post-processing",
      "description": "Use group-specific decision thresholds",
      "expectedImpact": "Improve demographic parity by 0.10-0.15",
      "implementationEffort": "low",
      "codeExample": "https://docs.wia.org/examples/thresholds",
      "applicability": 0.85
    }
  ]
}
```

---

## 4. Batch Operations

### 4.1 Batch Analysis

```http
POST /bias/analyze/batch
Content-Type: application/json

{
  "models": [
    {"modelId": "model-001", "datasetId": "dataset-001"},
    {"modelId": "model-002", "datasetId": "dataset-002"}
  ],
  "protectedAttributes": ["gender", "race"],
  "fairnessMetrics": ["demographic_parity"]
}
```

**Response (202 Accepted):**

```json
{
  "batchId": "batch-12345",
  "totalJobs": 2,
  "status": "processing",
  "_links": {
    "self": "/bias/batch/batch-12345",
    "jobs": "/bias/batch/batch-12345/jobs"
  }
}
```

---

## 5. Webhooks

### 5.1 Event Types

- `analysis.completed`: Bias analysis job completed
- `analysis.failed`: Bias analysis job failed
- `monitoring.alert`: Fairness threshold violated
- `dataset.updated`: Dataset analysis updated

### 5.2 Webhook Payload

```json
{
  "eventType": "monitoring.alert",
  "eventId": "event-12345",
  "timestamp": "2025-01-08T10:30:00Z",
  "data": {
    "monitoringId": "mon-12345",
    "modelId": "model-001",
    "metric": "demographic_parity",
    "value": 0.75,
    "threshold": 0.8,
    "severity": "warning"
  }
}
```

---

## 6. Error Handling

### 6.1 Error Response Format

```json
{
  "error": {
    "code": "INVALID_REQUEST",
    "message": "Protected attribute 'gender' not found in dataset",
    "details": {
      "availableAttributes": ["age", "race", "income"]
    },
    "requestId": "req-12345",
    "timestamp": "2025-01-08T10:30:00Z"
  }
}
```

### 6.2 Error Codes

- `INVALID_REQUEST`: Invalid request parameters
- `UNAUTHORIZED`: Authentication failed
- `FORBIDDEN`: Insufficient permissions
- `NOT_FOUND`: Resource not found
- `RATE_LIMIT_EXCEEDED`: Too many requests
- `INTERNAL_ERROR`: Server error
- `DATASET_TOO_SMALL`: Insufficient samples for analysis
- `PROTECTED_ATTRIBUTE_MISSING`: Required attribute not provided

---

## 7. Rate Limiting

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 995
X-RateLimit-Reset: 1641600000
```

Default limits:
- 1000 requests per hour per API key
- 10 concurrent analysis jobs
- 100 MB maximum request size

---

## 8. Pagination

For list endpoints:

```http
GET /bias/jobs?page=2&limit=50
```

**Response:**

```json
{
  "jobs": [ /* array of jobs */ ],
  "pagination": {
    "page": 2,
    "limit": 50,
    "total": 150,
    "totalPages": 3
  },
  "_links": {
    "first": "/bias/jobs?page=1&limit=50",
    "prev": "/bias/jobs?page=1&limit=50",
    "next": "/bias/jobs?page=3&limit=50",
    "last": "/bias/jobs?page=3&limit=50"
  }
}
```

---

## 9. Compliance

### 9.1 弘益人間 Requirements

APIs must:

- Provide clear documentation
- Return helpful error messages
- Support accessibility
- Respect data privacy
- Enable transparency

### 9.2 Certification

To be WIA-AI-013 Phase 2 certified:

- ✓ Implement all core endpoints
- ✓ Support standard data formats (Phase 1)
- ✓ Provide comprehensive API documentation
- ✓ Pass API compatibility test suite
- ✓ Support webhooks for async operations

---

**Document Version:** 1.0
**Effective Date:** 2025-01-08
**Maintained By:** WIA Standards Committee
**License:** CC BY 4.0

弘益人間 · Benefit All Humanity

---

## Annex A — Conformance Tier Matrix

WIA conformance for ai-bias-detection is evaluated across three tiers:

| Tier | Scope | Mandatory artifacts | Audit cadence |
|------|-------|--------------------|----------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | OpenAPI 3.0 contract, JSON Schema validation report, security threat model | Annual self-review |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Tier 1 artifacts + signed third-party assessor report against this PHASE | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Tier 2 artifacts + WIA accreditation, ISO/IEC 17065:2012 conformity assessment, evidence retention ≥ 7 years | Every 12 months |

Implementations MUST disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page. Tier downgrade events MUST be reported to the WIA registry within 30 days.

---

## Annex B — Cross-Walk to International Standards

The PHASE specification reuses or normatively references published standards alongside this document. Implementers SHOULD review the relevant standards alongside this PHASE document; where a conflict exists, the more specific WIA requirement governs unless explicitly superseded by a binding national regulation.

- ISO/IEC 17065:2012 — Conformity assessment — Requirements for bodies certifying products, processes and services
- ISO/IEC 27001:2022 — Information security management systems
- IETF RFC 9457 — Problem Details for HTTP APIs
- IETF RFC 7519 — JSON Web Token (JWT)
- IETF RFC 6749 — The OAuth 2.0 Authorization Framework
- W3C PROV-DM — provenance data model

This cross-walk is informative only. WIA does not republish the referenced documents; readers MUST obtain authoritative copies from the issuing body. Cross-walk entries are reviewed at every minor version of this PHASE.

---

## Annex C — Reference Implementations and Test Vectors

### C.1 Reference Implementations

WIA does not require implementers to use a particular library, but maintains pointers to the canonical reference implementation directories under the WIA-Official GitHub organization:

- `wia-standards/standards/ai-bias-detection/api/` — TypeScript SDK skeleton
- `wia-standards/standards/ai-bias-detection/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/ai-bias-detection/simulator/` — interactive browser-based simulator for the PHASE protocol

Each reference artifact ships with an MIT license and is intended as a starting point, not as a production-grade implementation.

### C.2 Test Vectors

A normative set of request/response pairs covering the schemas defined in this PHASE is published alongside this document. Implementations claiming Tier 2 or Tier 3 conformance MUST pass every published test vector in both serialization (request) and deserialization (response) directions, and MUST publish a signed report identifying which vectors were exercised.

Test vectors are versioned independently of the PHASE document; refer to the `test-vectors/` directory for the active version.

---

## Annex D — Open Questions and Future Work

This PHASE document captures the consensus position at v1.0. The following items are tracked for future minor or major revisions:

1. **Schema versioning policy** — formal MUST/SHOULD rules for backward-compatible vs. breaking changes between minor releases.
2. **Privacy-preserving aggregation** — guidance on differential privacy and secure aggregation patterns for telemetry channels, without prescribing a specific algorithm.
3. **Multilingual error catalogs** — localization strategy for the standard error codes defined in this PHASE.
4. **Long-term retention** — alignment with sectoral retention regulations across jurisdictions.
5. **Sustainability disclosure** — optional fields for energy and emissions reporting tied to operations covered by this PHASE.

Items in this annex are non-normative. Comments and proposals are accepted via the GitHub issues tracker on the WIA-Official organization.

---
