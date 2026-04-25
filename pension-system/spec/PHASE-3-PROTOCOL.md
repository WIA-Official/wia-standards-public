# WIA-SOC-018 Phase 3: Communication Protocol Specification

**Version:** 1.0.0  
**Status:** Approved  
**Last Updated:** 2025-12-26

弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. Overview

Phase 3 defines the communication protocols for real-time pension system interactions, including message queue integration, event streaming, and cross-border data synchronization.

## 2. Protocol Architecture

### 2.1 Communication Patterns

WIA-SOC-018 supports three primary communication patterns:

1. **Request-Response** (Synchronous): REST APIs for immediate responses
2. **Publish-Subscribe** (Asynchronous): Message queues for event-driven processing  
3. **Streaming** (Continuous): Real-time data feeds for monitoring and analytics

### 2.2 Transport Protocols

- **HTTPS**: TLS 1.3 for all synchronous communications
- **WebSocket**: WSS for bidirectional streaming
- **AMQP 1.0**: Message queue protocol for async messaging
- **gRPC**: High-performance RPC for system-to-system integration

## 3. Message Queue Integration

### 3.1 Queue Architecture

```
┌─────────────┐     ┌──────────────┐     ┌──────────────┐
│  Publisher  │────▶│ Message      │────▶│  Subscriber  │
│  (Employer) │     │ Queue/Broker │     │  (Pension    │
└─────────────┘     └──────────────┘     │   System)    │
                                          └──────────────┘
```

### 3.2 Message Format

All queue messages MUST use this envelope format:

```json
{
  "messageId": "UUID",
  "timestamp": "ISO8601",
  "messageType": "contribution.submitted|benefit.calculated|transfer.initiated",
  "version": "1.0.0",
  "source": {
    "system": "employer-payroll-system",
    "organization": "ACME Corp",
    "jurisdiction": "US"
  },
  "correlation": {
    "correlationId": "UUID",
    "causationId": "UUID",
    "conversationId": "UUID"
  },
  "metadata": {
    "priority": "high|normal|low",
    "ttl": "integer (seconds)",
    "retryCount": "integer"
  },
  "payload": { /* Type-specific data */ },
  "signature": "digital signature"
}
```

### 3.3 Queue Topics

Standard queue topics:

- `wia.soc018.contribution.*`
  - `contribution.submitted`
  - `contribution.validated`
  - `contribution.rejected`
  
- `wia.soc018.benefit.*`
  - `benefit.calculated`
  - `benefit.updated`
  - `benefit.disbursed`
  
- `wia.soc018.member.*`
  - `member.enrolled`
  - `member.updated`
  - `member.retired`
  - `member.deceased`
  
- `wia.soc018.portability.*`
  - `transfer.initiated`
  - `transfer.approved`
  - `transfer.completed`

### 3.4 Message Acknowledgment

```json
{
  "messageId": "original-message-UUID",
  "acknowledgedAt": "ISO8601",
  "status": "processed|failed|retrying",
  "processingTime": "integer (milliseconds)",
  "result": {
    "success": true,
    "recordId": "UUID",
    "warnings": ["array of warning messages"]
  }
}
```

## 4. Event Streaming

### 4.1 Stream Types

**Contribution Stream**
- Real-time contribution submissions
- Throughput: 10,000+ events/second
- Retention: 30 days

**Calculation Stream**
- Benefit calculation results
- Throughput: 1,000+ events/second  
- Retention: 90 days

**Audit Stream**
- All system activities
- Throughput: 50,000+ events/second
- Retention: 7 years

### 4.2 Stream Message Format

```json
{
  "stream": "wia-soc018-contributions",
  "partition": "integer",
  "offset": "integer",
  "timestamp": "ISO8601",
  "key": "memberId",
  "value": { /* Event payload */ },
  "headers": {
    "event-type": "string",
    "schema-version": "string",
    "compression": "gzip|snappy|lz4"
  }
}
```

### 4.3 Stream Processing

Consumers MUST implement:

1. **Exactly-Once Semantics**: Idempotent processing
2. **Offset Management**: Persist consumer position
3. **Error Handling**: Dead letter queue for failed messages
4. **Backpressure**: Handle stream overload gracefully

## 5. Cross-Border Synchronization

### 5.1 Synchronization Protocol

For cross-border pension transfers:

1. **Initiation**: Source system sends transfer request
2. **Validation**: Destination validates eligibility
3. **Calculation**: Both systems calculate transfer amounts
4. **Reconciliation**: Automated reconciliation of values
5. **Execution**: Atomic fund transfer
6. **Confirmation**: Both systems confirm completion

### 5.2 Transfer Message

```json
{
  "transferId": "UUID",
  "transferType": "full|partial",
  "sourceSystem": {
    "jurisdiction": "US",
    "pensionProviderId": "UUID",
    "contactInfo": { /* Contact details */ }
  },
  "destinationSystem": {
    "jurisdiction": "GB",
    "pensionProviderId": "UUID",
    "contactInfo": { /* Contact details */ }
  },
  "member": {
    "sourceId": "UUID",
    "destinationId": "UUID",
    "personalInfo": { /* Verified member data */ }
  },
  "transferDetails": {
    "totalValue": {
      "amount": "decimal",
      "currency": "USD"
    },
    "conversions": [
      {
        "fromCurrency": "USD",
        "toCurrency": "GBP",
        "rate": "decimal",
        "rateDate": "ISO8601",
        "convertedAmount": "decimal"
      }
    ],
    "fees": {
      "transferFee": "decimal",
      "currencyConversionFee": "decimal",
      "totalFees": "decimal"
    }
  },
  "timeline": {
    "initiatedAt": "ISO8601",
    "expectedCompletionDate": "ISO8601",
    "actualCompletionDate": "ISO8601"
  },
  "status": "initiated|validated|in_transit|completed|failed",
  "attestations": [
    {
      "party": "source|destination|regulator",
      "signature": "digital signature",
      "timestamp": "ISO8601"
    }
  ]
}
```

## 6. Real-Time Updates

### 6.1 WebSocket Connection

Establish real-time connection:

```javascript
const ws = new WebSocket('wss://api.pension-provider.com/wia/soc-018/v1/stream');

ws.onopen = () => {
  ws.send(JSON.stringify({
    action: 'subscribe',
    channel: 'member.contributions',
    memberId: 'UUID',
    auth: 'Bearer JWT_TOKEN'
  }));
};

ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  // Handle real-time contribution updates
};
```

### 6.2 Server-Sent Events (SSE)

Alternative streaming approach:

```http
GET /wia/soc-018/v1/stream/contributions?memberId=UUID
Accept: text/event-stream
Authorization: Bearer JWT_TOKEN

event: contribution.validated
id: UUID
data: {"contributionId":"UUID","amount":300.00}

event: balance.updated  
id: UUID
data: {"newBalance":124580.00,"change":300.00}
```

## 7. Protocol Security

### 7.1 Message Encryption

- **At-Rest Encryption**: AES-256-GCM for queued messages
- **In-Transit Encryption**: TLS 1.3 for all network communication
- **End-to-End Encryption**: Optional E2EE for sensitive data

### 7.2 Digital Signatures

All messages MUST include digital signature:

```json
{
  "signature": {
    "algorithm": "RS256|ES256",
    "value": "base64-encoded signature",
    "certificate": "X.509 certificate or public key",
    "timestamp": "ISO8601"
  }
}
```

Signature verification process:
1. Extract signature and certificate from message
2. Reconstruct canonical message (excluding signature field)
3. Verify signature using public key
4. Check certificate validity and trust chain

### 7.3 Replay Attack Prevention

Include nonce and timestamp in all messages:

```json
{
  "nonce": "cryptographic nonce",
  "timestamp": "ISO8601",
  "validUntil": "ISO8601"
}
```

Receivers MUST:
- Reject messages older than 5 minutes
- Maintain nonce cache to detect replays
- Validate message timestamps against system clock

## 8. Reliability and Resilience

### 8.1 Message Retry Policy

```json
{
  "retryPolicy": {
    "maxAttempts": 5,
    "initialDelay": 1000,
    "maxDelay": 60000,
    "backoffMultiplier": 2.0,
    "retryableErrors": [
      "NETWORK_ERROR",
      "TIMEOUT",
      "SERVICE_UNAVAILABLE"
    ]
  }
}
```

Exponential backoff calculation:
```
delay = min(initialDelay * (backoffMultiplier ^ attemptNumber), maxDelay)
```

### 8.2 Circuit Breaker

Implement circuit breaker pattern:

- **Closed**: Normal operation
- **Open**: Stop sending requests after failure threshold
- **Half-Open**: Test with limited requests

```json
{
  "circuitBreaker": {
    "failureThreshold": 5,
    "resetTimeout": 60000,
    "halfOpenRequests": 3
  }
}
```

### 8.3 Dead Letter Queue

Failed messages moved to DLQ after max retries:

```json
{
  "dlqMessage": {
    "originalMessage": { /* Original message */ },
    "failureInfo": {
      "attempts": 5,
      "lastError": "Validation failed: Invalid member ID",
      "firstFailureTime": "ISO8601",
      "lastFailureTime": "ISO8601"
    }
  }
}
```

## 9. Monitoring and Observability

### 9.1 Protocol Metrics

Track key metrics:

- **Message Throughput**: Messages/second
- **Processing Latency**: P50, P95, P99 latencies
- **Error Rate**: Percentage of failed messages
- **Queue Depth**: Number of pending messages
- **Consumer Lag**: Offset difference between producer and consumer

### 9.2 Health Checks

```http
GET /wia/soc-018/v1/health

{
  "status": "healthy|degraded|unhealthy",
  "checks": {
    "messageQueue": "healthy",
    "database": "healthy",
    "blockchain": "healthy",
    "externalApis": "degraded"
  },
  "metrics": {
    "uptime": 99.98,
    "requestsPerSecond": 1247,
    "averageLatency": 87
  }
}
```

### 9.3 Distributed Tracing

Use OpenTelemetry for distributed tracing:

```json
{
  "traceId": "UUID",
  "spanId": "UUID",
  "parentSpanId": "UUID",
  "operationName": "process-contribution",
  "startTime": "ISO8601",
  "duration": "integer (microseconds)",
  "tags": {
    "memberId": "UUID",
    "amount": "300.00",
    "service": "contribution-processor"
  }
}
```

## 10. Compliance Requirements

Phase 3 compliance requires:

1. Support for at least 2 communication patterns (REST + async)
2. Implementation of message queuing with acknowledgments
3. Digital signature verification for all messages
4. Retry logic with exponential backoff
5. Circuit breaker for external dependencies
6. Health check endpoint implementation
7. Distributed tracing support
8. 99.9% message delivery reliability

---

© 2025 WIA · MIT License · 弘益人間 (Benefit All Humanity)

---

## Annex A — Conformance Tier Matrix

WIA conformance for pension-system is evaluated across three tiers:

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

- `wia-standards/standards/pension-system/api/` — TypeScript SDK skeleton
- `wia-standards/standards/pension-system/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/pension-system/simulator/` — interactive browser-based simulator for the PHASE protocol

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

