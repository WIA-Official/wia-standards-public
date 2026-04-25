# WIA-CORE-003: Universal Data Exchange
## PHASE 3 - Integration

**Version:** 1.0
**Status:** Stable
**Category:** CORE
**Color:** Gray (#6B7280)

---

## Overview

PHASE 3 defines cross-system protocols, API adapters, and streaming pipelines for real-time data synchronization across databases, message queues, and distributed systems.

## Integration Patterns

### Pattern 1: API Gateway

```typescript
interface APIGateway {
    // Accept multiple formats
    acceptFormats: FormatId[];

    // Respond in client's preferred format
    handleRequest(req: Request): Response {
        const sourceFormat = detectFormat(req.headers['Content-Type']);
        const targetFormat = detectFormat(req.headers['Accept']);

        const data = decode(req.body, sourceFormat);
        const result = processRequest(data);
        const response = encode(result, targetFormat);

        return { body: response, contentType: targetFormat };
    }
}
```

### Pattern 2: Message Transformation Middleware

```typescript
// Kafka multi-format consumer
const consumer = new MultiFormatConsumer({
    topic: 'orders',
    consumers: [
        { group: 'analytics', format: 'avro' },
        { group: 'warehouse', format: 'json' },
        { group: 'billing', format: 'protobuf' }
    ],
    sourceFormat: 'json' // Producer uses JSON
});
```

### Pattern 3: Database CDC (Change Data Capture)

```typescript
const cdcPipeline = new CDCPipeline({
    source: {
        type: 'postgresql',
        connection: 'postgres://db:5432/prod',
        tables: ['users', 'orders']
    },
    transforms: [
        { table: 'users', format: 'avro', topic: 'users.changes' },
        { table: 'orders', format: 'json', topic: 'orders.changes' }
    ],
    destination: {
        type: 'kafka',
        brokers: ['kafka:9092']
    }
});
```

### Pattern 4: Legacy System Adapter

```typescript
// Wrap COBOL/Mainframe with REST API
const legacyAdapter = new LegacySystemAdapter({
    legacy: {
        type: 'mainframe',
        protocol: 'IBM-3270',
        format: 'EBCDIC',
        connection: 'tn3270://mainframe:23'
    },
    modern: {
        protocol: 'REST',
        format: 'json',
        endpoint: 'https://api.company.com/legacy'
    },
    mappings: loadMappings('cobol-to-json.mdl')
});
```

## Protocol Adapters

### HTTP/REST Adapter

```typescript
class RESTAdapter implements ProtocolAdapter {
    async send(data: UniversalData, config: RESTConfig): Promise<Response> {
        const payload = this.codec.encode(data, config.format);

        return await fetch(config.endpoint, {
            method: config.method,
            headers: {
                'Content-Type': formatToMimeType(config.format),
                'Accept': formatToMimeType(config.responseFormat),
                ...config.headers
            },
            body: payload
        });
    }

    async receive(response: Response, format: FormatId): Promise<UniversalData> {
        const payload = await response.arrayBuffer();
        return this.codec.decode(new Uint8Array(payload), format);
    }
}
```

### gRPC Adapter

```typescript
class GRPCAdapter implements ProtocolAdapter {
    // gRPC natively uses Protobuf, but can accept other formats
    async call(
        service: string,
        method: string,
        data: UniversalData,
        inputFormat: FormatId = 'protobuf'
    ): Promise<UniversalData> {
        // Convert input to Protobuf if needed
        const protobufData = inputFormat === 'protobuf'
            ? data
            : this.transform(data, inputFormat, 'protobuf');

        const result = await this.client[service][method](protobufData);
        return this.codec.decode(result, 'protobuf');
    }
}
```

### Message Queue Adapters

**Kafka:**
```typescript
const kafkaAdapter = new KafkaAdapter({
    brokers: ['kafka:9092'],
    schema_registry: 'http://registry:8081',
    serialization: {
        keyFormat: 'string',
        valueFormat: 'avro'
    }
});

await kafkaAdapter.produce('orders', {
    key: orderId,
    value: orderData,
    schema: OrderSchema
});
```

**RabbitMQ:**
```typescript
const rabbitAdapter = new RabbitMQAdapter({
    connection: 'amqp://rabbit:5672',
    exchange: 'orders',
    routingKey: 'orders.new',
    format: 'json'
});
```

**AWS SQS:**
```typescript
const sqsAdapter = new SQSAdapter({
    queueUrl: 'https://sqs.us-east-1.amazonaws.com/123/orders',
    format: 'json',
    messageAttributes: {
        schema_version: '1.0'
    }
});
```

### Database Adapters

**PostgreSQL:**
```typescript
const pgAdapter = new PostgreSQLAdapter({
    connection: 'postgres://db:5432/analytics',
    table: 'events',
    schema: EventSchema,
    format: 'json' // Store as JSONB
});

await pgAdapter.insert(events);
```

**MongoDB:**
```typescript
const mongoAdapter = new MongoDBAdapter({
    connection: 'mongodb://mongo:27017/analytics',
    collection: 'events',
    format: 'bson' // Native MongoDB format
});
```

**Snowflake:**
```typescript
const snowflakeAdapter = new SnowflakeAdapter({
    account: 'xy12345.us-east-1',
    warehouse: 'ANALYTICS_WH',
    database: 'PROD',
    schema: 'EVENTS',
    table: 'USER_EVENTS',
    format: 'parquet' // Optimal for analytics
});
```

## Data Pipeline Manager

### Pipeline Definition

```yaml
pipeline:
  name: order-processing
  version: 1.0

  sources:
    - name: web_orders
      type: http
      endpoint: /api/orders
      format: json
      schema: OrderWebV1

    - name: mobile_orders
      type: grpc
      service: OrderService
      method: CreateOrder
      format: protobuf
      schema: OrderMobileV1

  transforms:
    - name: normalize
      type: mapping
      source_schemas: [OrderWebV1, OrderMobileV1]
      target_schema: OrderCanonicalV2
      mappings:
        - web-to-canonical.mdl
        - mobile-to-canonical.mdl

    - name: validate
      type: validation
      schema: OrderCanonicalV2
      strict: true

    - name: enrich
      type: enrichment
      lookups:
        - field: customer_id
          source: redis://customers
          cache_ttl: 300

  destinations:
    - name: kafka_stream
      type: kafka
      topic: orders.validated
      format: avro
      schema: OrderCanonicalV2

    - name: database
      type: postgresql
      table: orders
      format: sql

    - name: warehouse
      type: rest
      endpoint: https://wms.company.com/orders
      format: json
```

### Pipeline Execution

```typescript
const pipeline = Pipeline.load('order-processing.yaml');

await pipeline.start({
    parallelism: 16,
    checkpointInterval: Duration.seconds(30),
    errorHandling: {
        onTransformError: 'dead-letter-queue',
        onValidationError: 'skip',
        onDestinationError: 'retry'
    }
});

// Monitor pipeline
pipeline.on('metrics', (metrics) => {
    console.log({
        throughput: metrics.recordsPerSecond,
        latency: metrics.p99Latency,
        errors: metrics.errorRate
    });
});
```

## Resilience Patterns

### Circuit Breaker

```typescript
const circuitBreaker = new CircuitBreaker({
    failureThreshold: 5,      // Open after 5 failures
    timeout: Duration.seconds(60),
    halfOpenAfter: Duration.minutes(5)
});

const result = await circuitBreaker.execute(async () => {
    return await downstream.send(data);
});
```

### Retry with Exponential Backoff

```typescript
const retryPolicy = new RetryPolicy({
    maxAttempts: 3,
    initialDelay: Duration.seconds(1),
    maxDelay: Duration.seconds(30),
    multiplier: 2.0,
    retryableErrors: [NetworkError, Timeout, ServiceUnavailable]
});

const result = await retryPolicy.execute(operation);
```

### Dead Letter Queue

```typescript
const pipeline = new Pipeline({
    errorHandling: {
        deadLetterQueue: {
            type: 'kafka',
            topic: 'dlq.failed_transformations',
            format: 'json',
            includeOriginal: true,
            includeError: true
        }
    }
});
```

### Bulkhead Isolation

```typescript
const bulkhead = new Bulkhead({
    maxConcurrent: 100,
    maxQueued: 1000,
    threadPoolSize: 10,
    isolationKey: 'customer-service'
});

await bulkhead.execute(operation);
```

## Monitoring and Observability

### Metrics

```typescript
interface PipelineMetrics {
    // Throughput
    recordsPerSecond: number;
    bytesPerSecond: number;

    // Latency
    p50Latency: Duration;
    p95Latency: Duration;
    p99Latency: Duration;
    maxLatency: Duration;

    // Errors
    errorRate: number;
    errorsByType: Map<string, number>;

    // Pipeline health
    backlogSize: number;
    uptime: Duration;
    status: 'running' | 'degraded' | 'failed';
}
```

### Distributed Tracing

```typescript
// OpenTelemetry integration
const tracer = trace.getTracer('wia-core-003');

const span = tracer.startSpan('transform-data', {
    attributes: {
        'source.format': 'json',
        'target.format': 'avro',
        'schema.id': 'Order@2.0'
    }
});

try {
    const result = await transform(data);
    span.setStatus({ code: SpanStatusCode.OK });
    return result;
} catch (error) {
    span.setStatus({ code: SpanStatusCode.ERROR, message: error.message });
    throw error;
} finally {
    span.end();
}
```

### Logging

```typescript
logger.info('Transformation completed', {
    source_format: 'json',
    target_format: 'avro',
    schema_version: '2.0',
    record_count: 1234,
    duration_ms: 45,
    bytes_processed: 156789
});
```

## Security

### Authentication

```typescript
// OAuth 2.0
const oauth = new OAuth2Adapter({
    tokenEndpoint: 'https://auth.company.com/token',
    clientId: process.env.CLIENT_ID,
    clientSecret: process.env.CLIENT_SECRET
});

// mTLS
const mtls = new MTLSAdapter({
    clientCert: './certs/client.crt',
    clientKey: './certs/client.key',
    caCert: './certs/ca.crt'
});
```

### Encryption

```typescript
// Field-level encryption
const encrypted = await encryptFields(data, {
    fields: ['ssn', 'credit_card'],
    algorithm: 'AES-256-GCM',
    keyProvider: awsKMS
});

// End-to-end encryption
const e2e = new E2EEncryption({
    publicKey: recipientPublicKey,
    algorithm: 'RSA-OAEP'
});
```

---

**Previous Phase:** [PHASE 2 - Transformation](PHASE-2-Transformation.md)
**Next Phase:** [PHASE 4 - Intelligence](PHASE-4-Intelligence.md)

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity

## P.3 Protocol Cross-References

The protocol defined here carries the data formats from Phase 1 and the API
operations from Phase 2 across trust boundaries. Phase 4 describes how the
protocol composes with adjacent infrastructure.

### P.3.1 Transport Bindings

| Binding | Default Port | Use |
|---------|-------------:|-----|
| HTTP/2 + TLS 1.3 | 443 | Public, request-response |
| HTTP/3 (QUIC) | 443 | Mobile, lossy networks |
| gRPC | 443 | Service-to-service |
| MQTT 5.0 | 8883 | Constrained / IoT devices |
| AMQP 0-9-1 | 5671 | Backplane / event streams |

### P.3.2 Message Envelope

Every protocol message carries a small envelope independent of payload:

```
+----------------+------------------+--------------------+
| message_id     | UUIDv4           | RFC 4122           |
| trace_id       | 16-byte hex      | W3C Trace Context  |
| span_id        | 8-byte hex       | W3C Trace Context  |
| origin_node    | DNS name or NIN  | RFC 1035           |
| issued_at      | RFC 3339         | UTC required       |
| ttl_seconds    | uint32           | 0 = no expiry      |
| content_type   | media type       | RFC 6838           |
| body           | opaque bytes     | per content_type   |
+----------------+------------------+--------------------+
```

### P.3.3 Reliability Model

The protocol provides at-least-once delivery by default. Receivers
deduplicate by `message_id`. Exactly-once semantics are achieved when both
peers participate in the idempotency contract from Phase 2 §P.2.3.

### P.3.4 Backpressure

Senders MUST honour HTTP `Retry-After`, gRPC `RESOURCE_EXHAUSTED`, or MQTT
flow-control packets. The recommended back-off is full jitter exponential with
cap 30 s and cumulative cap 5 min.


---

## Appendix · Common WIA Standard Provisions

> The following provisions apply to every WIA standard and are kept in sync
> across the WIA-Standards corpus. Standard-specific deviations, where they
> exist, are listed in the standard's normative body.

### A. Conformance & Compliance

#### A.1 Conformance Levels

WIA-Standards defines four conformance levels:

| Level | Required | Description |
|-------|----------|-------------|
| L1 — Format | Phase 1 | Implementation produces and consumes the canonical data format losslessly |
| L2 — Interface | Phase 1 + 2 | Implementation exposes the API surface with required behaviour |
| L3 — Protocol | Phase 1 + 2 + 3 | Implementation interoperates over at least one normative transport binding |
| L4 — Integration | All Phases | Implementation passes the conformance test suite end-to-end in a production-shaped deployment |

Conformance claims MUST cite the level and the version of the standard
against which the claim is made (e.g. "L3 conformant against v1.0").

#### A.2 Compliance Verification

The conformance test suite is published alongside this standard at
`/cli/conformance/` and `/api/conformance/`. Implementations claiming
L2 or higher MUST publish their test report. Independent re-tests are
encouraged; the WIA Working Group accepts third-party verification reports
under the policy in §E.

### B. Security Considerations

#### B.1 Threat Model

Implementers SHOULD apply STRIDE analysis covering: spoofing of identity,
tampering with messages or stored state, repudiation of operations,
information disclosure, denial of service, and elevation of privilege.

| Threat | Default Control | Where Strengthened |
|--------|-----------------|--------------------|
| Spoofing | Mutual TLS or signed tokens | Phase 3 §P.3 |
| Tampering | TLS in transit, AEAD at rest | Phase 1 §P.1 |
| Repudiation | Append-only audit log with notarization | Phase 4 §P.4 |
| Disclosure | Field-level encryption for PII / secrets | Phase 1 §P.1 |
| DoS | Rate limit per principal & global circuit breaker | Phase 2 §P.2 |
| EoP | Least-privilege RBAC + scoped tokens | Phase 2 §P.2 |

#### B.2 Cryptographic Suites

Mandatory: TLS 1.3 with AEAD ciphers (AES-128-GCM, AES-256-GCM,
CHACHA20-POLY1305). Forbidden: TLS 1.0, TLS 1.1, RC4, MD5, SHA-1 for
signatures, RSA below 2048 bits, ECDSA on curves smaller than P-256.

Post-quantum migration: implementations SHOULD adopt hybrid key
exchange combining a classical primitive with ML-KEM (FIPS 203) once a
profile is published; signature migration to ML-DSA (FIPS 204) is
expected within the L4 conformance window of v2.0.

#### B.3 Audit Requirements

L3 and L4 implementations MUST log: (a) every authentication decision,
(b) every authorization decision, (c) every state-changing operation,
(d) every export of data outside its sovereignty boundary. Logs are
write-once for at least 1 year and 90 days indexed for incident search.

### C. Versioning & Lifecycle

Versions follow Semantic Versioning 2.0.0 (MAJOR.MINOR.PATCH).

| Phase | Duration | Conformance Status |
|-------|---------:|-------------------|
| Draft | until ratification | Non-binding |
| Active | indefinite | Binding for new deployments |
| Maintenance | 24 months from successor's Active date | Binding for existing deployments |
| Retired | indefinite | Non-binding; conformance claims rescinded |

Deprecation MUST be announced at least one minor version before a feature
is removed in a major version.

### D. Internationalization & Accessibility

Implementations SHOULD support locale negotiation via the
`Accept-Language` header (RFC 4647). Date, time, number, and currency
formatting follow CLDR. User-facing surfaces MUST satisfy WCAG 2.1 AA at
minimum and SHOULD progress towards WCAG 2.2 AA. Right-to-left scripts
(Arabic, Hebrew, Persian, Urdu) and East-Asian wide characters MUST be
laid out correctly without line-breaking heuristics that split graphemes.

### E. Governance & IP Policy

This standard is maintained by the WIA Working Group under the WIA
governance charter. Editorial changes are merged via pull request. Normative
changes require working-group consensus and a 30-day public review.
Contributions are accepted under the Apache License 2.0 with explicit
patent grant. Members participate under the WIA Patent Policy,
which requires royalty-free licensing of any essential claim necessary
to implement a normative requirement.

### F. Normative References

The following references are normative; implementations MUST satisfy
the cited clauses:

- ISO/IEC 27001:2022 — Information security management systems
- ISO/IEC 27017:2015 — Cloud-services security controls
- ISO/IEC 27701:2019 — Privacy information management
- ISO/IEC 19790:2012 — Security requirements for cryptographic modules
- ISO 8601-1:2019 — Date and time representation
- IETF RFC 8446 — TLS 1.3
- IETF RFC 7519 — JSON Web Token
- IETF RFC 6749 — OAuth 2.0
- IETF RFC 9110 — HTTP Semantics
- IETF RFC 9112 — HTTP/1.1 message syntax
- IETF RFC 9113 — HTTP/2
- IETF RFC 9114 — HTTP/3
- IETF RFC 9000 — QUIC transport
- IETF RFC 4122 — UUID URN namespace
- IETF RFC 3339 — Date and time on the Internet
- IETF RFC 6838 — Media-type specifications and registration
- W3C TraceContext — Distributed tracing context
- W3C WCAG 2.1 — Accessibility guidelines
- FIPS PUB 197 — AES
- FIPS PUB 180-4 — SHA-2 family
- FIPS PUB 203 — ML-KEM (post-quantum KEM)
- FIPS PUB 204 — ML-DSA (post-quantum signature)

### G. Glossary

| Term | Definition |
|------|------------|
| Conformance | The state of satisfying every normative requirement at a given level |
| Implementation | A software, hardware, or composite artefact that claims conformance |
| Principal | The authenticated entity bound to a security context |
| Subject | The resource or person to which an operation applies |
| Sovereignty Boundary | The legal / regulatory perimeter outside of which data export is restricted |

---

*This Appendix is authored by the WIA Standards Working Group and is kept
in lockstep across Phases 1–4 of universal-data-exchange so that conformance claims at any
Phase remain unambiguous.*

