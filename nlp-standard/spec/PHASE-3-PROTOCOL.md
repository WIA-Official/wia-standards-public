# WIA-AI-023 NLP Standard - Phase 3: Protocol

**Version:** 1.0
**Status:** Complete
**Last Updated:** 2025-12-25

---

## 1. Overview

Phase 3 defines communication protocols, security requirements, and operational standards for NLP systems. This ensures reliable, secure, and efficient interactions between NLP services and clients.

### 1.1 Protocol Stack

- **Transport:** HTTPS (TLS 1.2+)
- **Application:** HTTP/1.1, HTTP/2
- **Data Format:** JSON
- **Streaming:** Server-Sent Events (SSE), WebSockets
- **Message Queue:** AMQP, Kafka (optional)

## 2. Communication Patterns

### 2.1 Request-Response (Synchronous)

Standard HTTP request-response for short-running tasks:

```http
POST /nlp/v1/sentiment HTTP/1.1
Host: api.example.com
Content-Type: application/json

{"text": "Great product!"}
```

### 2.2 Long-Running Tasks (Asynchronous)

For expensive operations:

**Request:**
```http
POST /nlp/v1/summarize HTTP/1.1
X-Async: true

{"text": "Very long document..."}
```

**Response:**
```http
HTTP/1.1 202 Accepted
Location: /nlp/v1/jobs/abc123

{
  "job_id": "abc123",
  "status": "processing",
  "estimated_time_seconds": 30
}
```

**Status Check:**
```http
GET /nlp/v1/jobs/abc123
```

### 2.3 Streaming (Real-time)

**Server-Sent Events:**
```http
GET /nlp/v1/generate/stream
Accept: text/event-stream

data: {"token": "Natural", "index": 0}
data: {"token": "language", "index": 1}
data: {"token": "processing", "index": 2}
```

**WebSocket:**
```javascript
ws = new WebSocket('wss://api.example.com/nlp/v1/stream');
ws.send(JSON.stringify({task: 'generate', prompt: 'NLP enables'}));
```

## 3. Security

### 3.1 Transport Security

- **Required:** TLS 1.2 or higher
- **Recommended:** TLS 1.3
- **Certificate:** Valid SSL/TLS certificate
- **Cipher Suites:** Strong encryption only

### 3.2 Authentication

**Bearer Token:**
```http
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
```

**API Key:**
```http
X-API-Key: wia_ai_023_key_abc123def456
```

**OAuth 2.0:**
```http
Authorization: Bearer <oauth_access_token>
```

### 3.3 Data Privacy

- **PII Detection:** Automatic detection and redaction
- **Data Encryption:** At rest and in transit
- **Data Retention:** Configurable retention policies
- **GDPR Compliance:** Right to deletion support
- **Logging:** Sanitized logs (no sensitive data)

### 3.4 Input Sanitization

```json
{
  "sanitization": {
    "remove_html": true,
    "remove_urls": false,
    "detect_pii": true,
    "max_length": 100000
  }
}
```

## 4. Protocol Features

### 4.1 Compression

**Request:**
```http
Content-Encoding: gzip
Accept-Encoding: gzip, deflate
```

### 4.2 Caching

**Cache Headers:**
```http
Cache-Control: public, max-age=3600
ETag: "33a64df551425fcc55e4d42a148795d9f25f89d4"
```

### 4.3 Conditional Requests

```http
If-None-Match: "33a64df551425fcc55e4d42a148795d9f25f89d4"
If-Modified-Since: Wed, 21 Oct 2025 07:28:00 GMT
```

## 5. Message Format

### 5.1 Request Message

```json
{
  "header": {
    "standard": "WIA-AI-023",
    "version": "1.0",
    "message_id": "uuid",
    "timestamp": "ISO 8601",
    "client_id": "string"
  },
  "body": {
    "task": "string",
    "input": {},
    "config": {}
  }
}
```

### 5.2 Response Message

```json
{
  "header": {
    "standard": "WIA-AI-023",
    "version": "1.0",
    "message_id": "uuid",
    "correlation_id": "request_message_id",
    "timestamp": "ISO 8601"
  },
  "body": {
    "output": {},
    "metadata": {}
  },
  "status": {
    "code": 200,
    "message": "Success"
  }
}
```

## 6. Error Handling

### 6.1 Error Propagation

```json
{
  "status": {
    "code": 500,
    "message": "Internal processing error",
    "details": "Model inference failed",
    "trace_id": "trace-abc123",
    "retry_after": 60
  },
  "errors": [
    {
      "field": "text",
      "message": "Text exceeds maximum length",
      "code": "TEXT_TOO_LONG"
    }
  ]
}
```

### 6.2 Retry Logic

**Exponential Backoff:**
```
Attempt 1: Wait 1 second
Attempt 2: Wait 2 seconds
Attempt 3: Wait 4 seconds
Attempt 4: Wait 8 seconds
Max attempts: 5
```

**Retry Header:**
```http
Retry-After: 60
X-RateLimit-Reset: 1640995200
```

## 7. Monitoring and Observability

### 7.1 Request Tracing

**Trace Headers:**
```http
X-Trace-ID: trace-abc123
X-Span-ID: span-xyz789
X-Parent-Span-ID: span-parent-456
```

### 7.2 Metrics

Required metrics:
- Request count
- Error rate
- Response time (P50, P95, P99)
- Throughput (requests/second)
- Model confidence distribution

### 7.3 Health Checks

**Liveness:**
```http
GET /health/live
```

**Readiness:**
```http
GET /health/ready
```

## 8. Performance Optimization

### 8.1 Connection Pooling

- Keep-alive connections
- Connection reuse
- Maximum concurrent connections

### 8.2 Request Batching

```json
{
  "batch": {
    "requests": [...],
    "max_batch_size": 32,
    "timeout_ms": 5000
  }
}
```

### 8.3 Response Compression

```http
Content-Encoding: gzip
Content-Length: 1234
```

## 9. Versioning and Compatibility

### 9.1 Backward Compatibility

- Support previous major version
- Deprecation warnings
- Migration guides

### 9.2 Version Negotiation

```http
Accept: application/vnd.wia.nlp.v1+json
API-Version: 1.0
```

## 10. Service Level Agreement (SLA)

### 10.1 Availability

- **Target:** 99.9% uptime
- **Maintenance Windows:** Scheduled, announced 7 days prior
- **Incident Response:** <15 minutes acknowledgment

### 10.2 Performance

- **Latency:** P95 < 200ms (simple tasks)
- **Throughput:** 1000+ requests/second
- **Accuracy:** Task-dependent minimum thresholds

## 11. Compliance Checklist

- [ ] TLS 1.2+ enforced
- [ ] Authentication implemented
- [ ] PII detection enabled
- [ ] Request/response logging (sanitized)
- [ ] Error handling with retry logic
- [ ] Health check endpoints
- [ ] Request tracing headers
- [ ] Compression supported
- [ ] Rate limiting configured
- [ ] Documentation complete

---

**Previous:** [Phase 2: API Interface](PHASE-2-API.md)
**Next:** [Phase 4: Integration](PHASE-4-INTEGRATION.md)

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


## Annex E — Implementation Notes for PHASE-3-PROTOCOL

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-3-PROTOCOL.

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

