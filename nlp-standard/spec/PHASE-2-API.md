# WIA-AI-023 NLP Standard - Phase 2: API Interface

**Version:** 1.0
**Status:** Complete
**Last Updated:** 2025-12-25

---

## 1. Overview

Phase 2 defines RESTful API specifications for NLP services, ensuring consistent interfaces across different implementations. This enables plug-and-play interoperability and simplifies integration.

### 1.1 Design Principles

- RESTful architecture
- Stateless communication
- Standard HTTP methods
- JSON payloads
- Versioned endpoints
- Authentication and authorization

## 2. API Endpoints

### 2.1 Base URL Structure

```
https://api.example.com/nlp/v1/{task}
```

### 2.2 Core Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/tokenize` | POST | Tokenize text |
| `/ner` | POST | Named entity recognition |
| `/sentiment` | POST | Sentiment analysis |
| `/classify` | POST | Text classification |
| `/generate` | POST | Text generation |
| `/summarize` | POST | Text summarization |
| `/translate` | POST | Machine translation |
| `/embedding` | POST | Get text embeddings |

### 2.3 Utility Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/health` | GET | Health check |
| `/info` | GET | Service information |
| `/models` | GET | List available models |
| `/languages` | GET | List supported languages |

## 3. Request/Response Specifications

### 3.1 Tokenization API

**Endpoint:** `POST /nlp/v1/tokenize`

**Request:**
```http
POST /nlp/v1/tokenize HTTP/1.1
Host: api.example.com
Content-Type: application/json
Authorization: Bearer <token>

{
  "text": "Natural language processing is transforming AI.",
  "language": "en",
  "method": "word",
  "include_offsets": true
}
```

**Response:**
```http
HTTP/1.1 200 OK
Content-Type: application/json

{
  "standard": "WIA-AI-023",
  "version": "1.0",
  "output": {
    "tokens": ["Natural", "language", "processing", "is", "transforming", "AI", "."],
    "token_count": 7
  },
  "metadata": {
    "request_id": "550e8400-e29b-41d4-a716-446655440000",
    "processing_time_ms": 12
  }
}
```

### 3.2 NER API

**Endpoint:** `POST /nlp/v1/ner`

**Request Headers:**
```
Content-Type: application/json
Authorization: Bearer <token>
X-Request-ID: <optional-client-id>
```

**Request Body:**
```json
{
  "text": "Apple CEO Tim Cook announced new products in Cupertino.",
  "language": "en",
  "entity_types": ["PERSON", "ORGANIZATION", "LOCATION"]
}
```

**Response:** (See Phase 1 NER format)

### 3.3 Sentiment Analysis API

**Endpoint:** `POST /nlp/v1/sentiment`

**Request:**
```json
{
  "text": "This is absolutely wonderful!",
  "language": "en",
  "return_scores": true
}
```

### 3.4 Text Classification API

**Endpoint:** `POST /nlp/v1/classify`

**Request:**
```json
{
  "text": "Scientists discovered a new planet.",
  "language": "en",
  "categories": ["Science", "Technology", "Business", "Sports"],
  "top_k": 2
}
```

### 3.5 Text Generation API

**Endpoint:** `POST /nlp/v1/generate`

**Request:**
```json
{
  "prompt": "Natural language processing enables",
  "max_length": 100,
  "temperature": 0.8,
  "top_p": 0.95
}
```

### 3.6 Summarization API

**Endpoint:** `POST /nlp/v1/summarize`

**Request:**
```json
{
  "text": "Long document text...",
  "method": "abstractive",
  "max_length": 150,
  "min_length": 50
}
```

## 4. Authentication

### 4.1 Bearer Token Authentication

```http
Authorization: Bearer <access_token>
```

### 4.2 API Key Authentication

```http
X-API-Key: <api_key>
```

### 4.3 OAuth 2.0

Supports standard OAuth 2.0 flows:
- Client Credentials
- Authorization Code
- Refresh Token

## 5. Rate Limiting

### 5.1 Rate Limit Headers

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 999
X-RateLimit-Reset: 1640995200
```

### 5.2 Rate Limit Response

When exceeded:
```http
HTTP/1.1 429 Too Many Requests
Retry-After: 60

{
  "status": {
    "code": 429,
    "message": "Rate limit exceeded",
    "details": "Limit of 1000 requests per hour exceeded"
  }
}
```

## 6. Batch Processing

### 6.1 Batch Endpoint

**Endpoint:** `POST /nlp/v1/batch`

**Request:**
```json
{
  "requests": [
    {
      "task": "sentiment",
      "input": {"text": "Great product!", "language": "en"}
    },
    {
      "task": "ner",
      "input": {"text": "Tim Cook leads Apple", "language": "en"}
    }
  ]
}
```

**Response:**
```json
{
  "responses": [
    {
      "index": 0,
      "status": "success",
      "output": {"sentiment": "positive", "confidence": 0.95}
    },
    {
      "index": 1,
      "status": "success",
      "output": {"entities": [...]}
    }
  ]
}
```

## 7. Error Handling

### 7.1 Error Response Format

```json
{
  "status": {
    "code": 400,
    "message": "Invalid request",
    "details": "Missing required field: text"
  },
  "metadata": {
    "request_id": "uuid",
    "timestamp": "ISO 8601"
  }
}
```

### 7.2 HTTP Status Codes

| Code | Status | Description |
|------|--------|-------------|
| 200 | OK | Success |
| 201 | Created | Resource created |
| 400 | Bad Request | Invalid input |
| 401 | Unauthorized | Authentication required |
| 403 | Forbidden | Insufficient permissions |
| 404 | Not Found | Endpoint not found |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Server Error | Server error |
| 503 | Service Unavailable | Temporary unavailable |

## 8. Versioning

### 8.1 URL Versioning

```
/nlp/v1/sentiment  (version 1)
/nlp/v2/sentiment  (version 2)
```

### 8.2 Header Versioning (Optional)

```http
API-Version: 1.0
```

## 9. CORS Support

### 9.1 CORS Headers

```http
Access-Control-Allow-Origin: *
Access-Control-Allow-Methods: GET, POST, OPTIONS
Access-Control-Allow-Headers: Content-Type, Authorization
Access-Control-Max-Age: 86400
```

## 10. Service Discovery

### 10.1 Info Endpoint

**Endpoint:** `GET /nlp/v1/info`

**Response:**
```json
{
  "standard": "WIA-AI-023",
  "version": "1.0",
  "service": {
    "name": "NLP Service",
    "vendor": "Example Corp",
    "api_version": "1.0.0"
  },
  "supported_tasks": [
    "tokenization",
    "ner",
    "sentiment",
    "classification",
    "generation",
    "summarization"
  ],
  "supported_languages": ["en", "ko", "es", "fr", "de"],
  "rate_limits": {
    "requests_per_hour": 1000,
    "requests_per_day": 10000
  }
}
```

### 10.2 Health Check

**Endpoint:** `GET /nlp/v1/health`

**Response:**
```json
{
  "status": "healthy",
  "timestamp": "2025-12-25T10:30:00Z",
  "version": "1.0.0",
  "uptime_seconds": 86400
}
```

## 11. Compliance Checklist

- [ ] All endpoints follow RESTful conventions
- [ ] Authentication implemented
- [ ] Rate limiting configured
- [ ] CORS headers set appropriately
- [ ] Error responses follow standard format
- [ ] API versioning implemented
- [ ] Health check endpoint available
- [ ] Service info endpoint available
- [ ] Batch processing supported
- [ ] Documentation available

---

**Previous:** [Phase 1: Data Format](PHASE-1-DATA-FORMAT.md)
**Next:** [Phase 3: Protocol](PHASE-3-PROTOCOL.md)

**弘益人間** · Benefit All Humanity

© 2025 WIA - World Certification Industry Association

---

## Annex A — Conformance Tier Matrix

WIA conformance for nlp-standard is evaluated across three tiers:

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

- `wia-standards/standards/nlp-standard/api/` — TypeScript SDK skeleton
- `wia-standards/standards/nlp-standard/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/nlp-standard/simulator/` — interactive browser-based simulator for the PHASE protocol

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


## Annex E — Implementation Notes for PHASE-2-API

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-2-API.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

