# WIA-DATA-013: Streaming Data Standard
## PHASE 1: Data Format Specification

### Version: 1.0
### Status: Active
### Last Updated: 2025-01-15

---

## 1. Overview

This document defines the data format specifications for streaming data systems, including event serialization, schema management, and data modeling standards.

## 2. Event Structure

### 2.1 Standard Event Format

All streaming events MUST conform to the following structure:

```json
{
  "eventId": "string (UUID v4)",
  "eventType": "string (enum)",
  "timestamp": "ISO 8601 datetime",
  "version": "string (semantic version)",
  "source": {
    "system": "string",
    "component": "string",
    "instance": "string (optional)"
  },
  "data": {
    "...": "event-specific payload"
  },
  "metadata": {
    "correlationId": "string (UUID v4)",
    "causationId": "string (UUID v4, optional)",
    "userId": "string (optional)",
    "sessionId": "string (optional)"
  }
}
```

### 2.2 Required Fields

- **eventId**: Unique identifier for the event (UUID v4)
- **eventType**: Type/category of the event (e.g., `user.registered`, `order.placed`)
- **timestamp**: When the event occurred (ISO 8601 format with timezone)
- **version**: Schema version for backward compatibility
- **source**: System generating the event
- **data**: Event-specific payload

### 2.3 Optional Fields

- **metadata**: Additional context (correlation IDs, user info, etc.)

## 3. Serialization Formats

### 3.1 Supported Formats

#### JSON (JavaScript Object Notation)
- **Use Case**: Human-readable, schema-flexible
- **Pros**: Easy debugging, wide support
- **Cons**: Larger size, slower parsing
- **When to Use**: Development, low-volume streams, interoperability

#### Avro (Apache Avro)
- **Use Case**: Schema evolution, compact binary
- **Pros**: Schema validation, efficient, backward/forward compatible
- **Cons**: Requires schema registry
- **When to Use**: High-volume production systems

```json
{
  "type": "record",
  "name": "UserEvent",
  "namespace": "com.wia.events",
  "fields": [
    {"name": "eventId", "type": "string"},
    {"name": "eventType", "type": "string"},
    {"name": "timestamp", "type": "long", "logicalType": "timestamp-millis"},
    {"name": "userId", "type": "string"},
    {"name": "data", "type": "string"}
  ]
}
```

#### Protocol Buffers (Protobuf)
- **Use Case**: Type-safe, compact
- **Pros**: Strong typing, efficient, code generation
- **Cons**: Less flexible than JSON
- **When to Use**: Microservices, gRPC integration

```protobuf
syntax = "proto3";

message UserEvent {
  string event_id = 1;
  string event_type = 2;
  int64 timestamp = 3;
  string user_id = 4;
  bytes data = 5;
}
```

### 3.2 Format Selection Matrix

| Criteria | JSON | Avro | Protobuf |
|----------|------|------|----------|
| Readability | High | Low | Low |
| Size Efficiency | Low | High | High |
| Schema Evolution | Manual | Excellent | Good |
| Performance | Moderate | High | High |
| Tooling | Ubiquitous | Good | Good |

## 4. Schema Management

### 4.1 Schema Registry

All production streaming systems MUST use a schema registry:

- **Confluent Schema Registry**: For Kafka ecosystems
- **AWS Glue Schema Registry**: For AWS environments
- **Azure Schema Registry**: For Azure Event Hubs

### 4.2 Schema Versioning

Follow semantic versioning (MAJOR.MINOR.PATCH):

- **MAJOR**: Breaking changes (incompatible)
- **MINOR**: Backward-compatible additions
- **PATCH**: Backward-compatible fixes

### 4.3 Compatibility Modes

| Mode | Description | Use Case |
|------|-------------|----------|
| BACKWARD | New consumers, old producers | Consumer upgrades first |
| FORWARD | Old consumers, new producers | Producer upgrades first |
| FULL | Both directions compatible | Flexible evolution |
| NONE | No compatibility checks | Development only |

## 5. Data Types

### 5.1 Primitive Types

- **string**: UTF-8 encoded text
- **int32/int64**: Signed integers
- **float/double**: Floating-point numbers
- **boolean**: true/false
- **bytes**: Binary data
- **timestamp**: Unix epoch (milliseconds)

### 5.2 Complex Types

- **array**: Ordered collection
- **map**: Key-value pairs
- **union**: Multiple possible types
- **enum**: Predefined values

### 5.3 Logical Types

- **timestamp-millis**: Unix timestamp in milliseconds
- **timestamp-micros**: Unix timestamp in microseconds
- **date**: Days since Unix epoch
- **time-millis**: Milliseconds since midnight
- **uuid**: UUID string
- **decimal**: Arbitrary precision decimal

## 6. Encoding and Compression

### 6.1 Character Encoding

All text fields MUST use UTF-8 encoding.

### 6.2 Compression

Supported compression algorithms:

| Algorithm | Compression Ratio | Speed | CPU Usage |
|-----------|-------------------|-------|-----------|
| None | 1x | Fastest | Minimal |
| GZIP | 3-4x | Slow | High |
| Snappy | 2x | Fast | Low |
| LZ4 | 2-3x | Very Fast | Low |
| ZSTD | 3-4x | Fast | Medium |

**Recommendation**: Use Snappy or LZ4 for streaming (balance of speed and compression).

## 7. Event Naming Conventions

### 7.1 Event Type Format

Events MUST follow the pattern: `{domain}.{entity}.{action}`

Examples:
- `user.account.created`
- `order.payment.completed`
- `inventory.stock.depleted`

### 7.2 Naming Rules

- Use lowercase
- Use periods (.) as separators
- Use past tense for actions
- Be specific and descriptive
- Avoid abbreviations

## 8. Data Quality Standards

### 8.1 Validation Rules

All events MUST:
- Have unique eventId
- Have valid timestamp (within acceptable time window)
- Conform to registered schema
- Have non-empty required fields
- Pass business rule validation

### 8.2 Error Handling

Invalid events SHOULD:
- Be sent to Dead Letter Queue (DLQ)
- Include validation errors in metadata
- Be logged for monitoring
- Trigger alerts if error rate exceeds threshold

## 9. Examples

### 9.1 User Registration Event (JSON)

```json
{
  "eventId": "550e8400-e29b-41d4-a716-446655440000",
  "eventType": "user.account.created",
  "timestamp": "2025-01-15T10:30:00.000Z",
  "version": "1.0.0",
  "source": {
    "system": "user-service",
    "component": "registration-api",
    "instance": "pod-123"
  },
  "data": {
    "userId": "user-12345",
    "email": "user@example.com",
    "accountType": "premium",
    "registrationMethod": "email"
  },
  "metadata": {
    "correlationId": "660e8400-e29b-41d4-a716-446655440001",
    "sessionId": "session-abc-123"
  }
}
```

### 9.2 Order Placed Event (Avro)

```json
{
  "type": "record",
  "name": "OrderPlaced",
  "namespace": "com.wia.orders",
  "fields": [
    {"name": "eventId", "type": "string"},
    {"name": "eventType", "type": "string", "default": "order.placed"},
    {"name": "timestamp", "type": "long", "logicalType": "timestamp-millis"},
    {"name": "orderId", "type": "string"},
    {"name": "customerId", "type": "string"},
    {"name": "totalAmount", "type": "double"},
    {
      "name": "items",
      "type": {
        "type": "array",
        "items": {
          "type": "record",
          "name": "OrderItem",
          "fields": [
            {"name": "productId", "type": "string"},
            {"name": "quantity", "type": "int"},
            {"name": "price", "type": "double"}
          ]
        }
      }
    }
  ]
}
```

## 10. Compliance

Systems implementing this standard MUST:
- ✅ Use one of the approved serialization formats
- ✅ Register all schemas in a schema registry
- ✅ Follow event naming conventions
- ✅ Implement data validation
- ✅ Handle invalid events appropriately
- ✅ Support schema evolution

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
