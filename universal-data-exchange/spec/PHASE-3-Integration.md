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
