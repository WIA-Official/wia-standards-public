# WIA-DATA-009: Master Data Management Standard
## PHASE 3: Protocol Specification

**Version:** 1.0.0  
**Status:** Draft  
**Last Updated:** 2025-12-26

---

## Overview

This phase defines communication protocols, data synchronization mechanisms, and integration patterns for MDM systems.

## 1. Communication Protocols

### 1.1 Supported Protocols

- **HTTPS**: Primary protocol for all API communications
- **WebSocket**: Real-time bidirectional communication
- **gRPC**: High-performance RPC for internal services
- **AMQP**: Message queue integration (RabbitMQ, Azure Service Bus)
- **Kafka**: Event streaming platform integration

### 1.2 Protocol Selection Guidelines

| Use Case | Recommended Protocol | Rationale |
|----------|---------------------|-----------|
| CRUD Operations | HTTPS REST | Standard, widely supported |
| Real-time Updates | WebSocket | Bidirectional, low latency |
| Service-to-Service | gRPC | High performance, type safety |
| Event Broadcasting | Kafka | Scalable, durable, replay capability |
| Job Queuing | AMQP | Reliable message delivery |

## 2. Data Synchronization Patterns

### 2.1 Real-Time Synchronization

**Change Data Capture (CDC) Pattern:**

```
Source System → CDC Agent → MDM Hub → Event Bus → Target Systems
```

**Implementation:**
- Database triggers capture changes
- CDC agent publishes change events
- MDM hub processes and enriches data
- Target systems subscribe to relevant events

**Event Format:**
```json
{
  "eventId": "uuid",
  "eventType": "entity.created | entity.updated | entity.deleted",
  "timestamp": "ISO8601",
  "entityType": "customer",
  "entityId": "uuid",
  "changes": {
    "before": {},
    "after": {}
  }
}
```

### 2.2 Batch Synchronization

**ETL Pattern:**
```
Source Systems → Extract → Transform → MDM Hub → Load → Target Systems
```

**Schedule Options:**
- **Hourly**: Near real-time for critical data
- **Daily**: Standard for most master data
- **Weekly**: Reference data and historical archives

**Batch File Format:**
```json
{
  "batchId": "uuid",
  "batchType": "full | incremental",
  "timestamp": "ISO8601",
  "entities": [...]
}
```

### 2.3 Hybrid Synchronization

Combines real-time for critical changes with batch for bulk updates:

- **Real-time**: Customer contact updates, product pricing
- **Batch**: Historical data, bulk imports, analytics sync

## 3. Event-Driven Architecture

### 3.1 Event Types

| Event Type | Description | Trigger |
|------------|-------------|---------|
| `entity.created` | New entity created | POST /entities |
| `entity.updated` | Entity modified | PUT /entities/{id} |
| `entity.deleted` | Entity soft/hard deleted | DELETE /entities/{id} |
| `entity.merged` | Multiple entities merged | POST /entities/merge |
| `entity.split` | Entity split into multiple | POST /entities/split |
| `entity.matched` | Match candidate identified | Matching algorithm |
| `quality.issue.detected` | Quality rule violation | Validation |
| `quality.issue.resolved` | Quality issue fixed | Steward action |

### 3.2 Event Publishing

**Kafka Topic Structure:**
```
mdm.{entityType}.{eventType}

Examples:
- mdm.customer.created
- mdm.product.updated
- mdm.supplier.deleted
```

**Event Message:**
```json
{
  "eventId": "uuid",
  "eventType": "entity.updated",
  "timestamp": "2025-12-26T10:30:00Z",
  "source": "mdm-hub",
  "entityType": "customer",
  "entityId": "a1b2c3d4...",
  "version": 2,
  "payload": {
    "changes": [...]
  },
  "metadata": {
    "correlationId": "uuid",
    "causationId": "uuid",
    "userId": "user@example.com"
  }
}
```

### 3.3 Event Consumption

**Consumer Groups:**
- CRM systems subscribe to customer events
- E-commerce subscribes to product events
- Analytics subscribes to all events

**At-Least-Once Delivery:**
- Consumers must be idempotent
- Use message deduplication based on eventId

## 4. Data Integration Patterns

### 4.1 Hub-and-Spoke Pattern

```
     ┌─────────┐
     │   CRM   │─────┐
     └─────────┘     │
                     ▼
     ┌─────────┐  ┌──────┐  ┌─────────┐
     │   ERP   │─▶│ MDM  │◀─│E-Commerce│
     └─────────┘  │ Hub  │  └─────────┘
                  └──────┘
                     ▲
     ┌─────────┐     │
     │Analytics│─────┘
     └─────────┘
```

**Benefits:**
- Centralized data management
- Simplified integration (N systems = N connections, not N²)
- Single source of truth

### 4.2 Publish-Subscribe Pattern

```
MDM Hub → Event Bus (Kafka) → Multiple Subscribers
```

**Benefits:**
- Decoupled systems
- Scalable event distribution
- Event replay capability

### 4.3 API Gateway Pattern

```
Client Apps → API Gateway → MDM Services
                  ↓
          [Auth, Rate Limit, Cache]
```

**Benefits:**
- Centralized authentication
- Rate limiting and throttling
- Response caching
- Request/response transformation

## 5. Conflict Resolution

### 5.1 Conflict Detection

Conflicts occur when:
- Same entity updated in multiple systems simultaneously
- Offline systems synchronize stale data
- Manual overrides conflict with automated processes

### 5.2 Resolution Strategies

| Strategy | Description | Use Case |
|----------|-------------|----------|
| Last Write Wins | Most recent timestamp wins | Low-value attributes |
| Highest Quality Source | Trust score determines winner | Critical data fields |
| Manual Review | Queue for steward resolution | High-value conflicts |
| Merge | Combine non-conflicting changes | Complementary updates |
| Version Vector | Track causality | Distributed systems |

### 5.3 Conflict Event

```json
{
  "conflictId": "uuid",
  "entityId": "uuid",
  "conflictType": "concurrent_update",
  "detectedAt": "ISO8601",
  "conflictingVersions": [
    {
      "source": "CRM",
      "timestamp": "2025-12-26T10:30:00Z",
      "changes": {...}
    },
    {
      "source": "ERP",
      "timestamp": "2025-12-26T10:30:01Z",
      "changes": {...}
    }
  ],
  "resolutionStrategy": "manual_review",
  "assignedTo": "data.steward@example.com"
}
```

## 6. Data Transformation

### 6.1 Transformation Pipeline

```
Source Data → Validation → Standardization → Enrichment → MDM Format
```

**Validation:**
- Schema validation
- Business rule validation
- Data type checking

**Standardization:**
- Address standardization (USPS, Canada Post)
- Phone number formatting (E.164)
- Name parsing and standardization

**Enrichment:**
- Append missing data from third-party sources
- Geocoding addresses
- Firmographic data append

### 6.2 Transformation Rules

```json
{
  "ruleId": "uuid",
  "ruleName": "Standardize Phone Numbers",
  "sourceField": "phone",
  "targetField": "phoneE164",
  "transformation": {
    "type": "format",
    "format": "E.164",
    "defaultCountryCode": "+82"
  }
}
```

## 7. Security Protocols

### 7.1 Transport Security

- **TLS 1.3**: All data in transit encrypted
- **Certificate Pinning**: Mobile apps pin MDM certificates
- **mTLS**: Mutual TLS for service-to-service communication

### 7.2 Data Encryption

- **At Rest**: AES-256 encryption for sensitive fields
- **In Transit**: TLS 1.3
- **Key Management**: Azure Key Vault / AWS KMS / HashiCorp Vault

### 7.3 Access Control

```json
{
  "userId": "user@example.com",
  "roles": ["data_steward"],
  "permissions": {
    "customer": ["read", "write"],
    "product": ["read"],
    "supplier": ["read", "write"]
  },
  "dataScope": {
    "regions": ["APAC", "NA"],
    "businessUnits": ["retail"]
  }
}
```

## 8. Protocol Compliance

### 8.1 Data Formats

- **JSON**: Default for REST APIs
- **Protocol Buffers**: gRPC services
- **Avro**: Kafka messages

### 8.2 Standards Compliance

- **ISO 8601**: Date/time formats
- **RFC 5322**: Email addresses
- **E.164**: Phone numbers
- **ISO 3166**: Country codes
- **ISO 4217**: Currency codes

## 9. Performance Optimization

### 9.1 Caching Strategy

```
Client → CDN → API Gateway → Cache (Redis) → MDM Services
```

**Cache Policies:**
- **Reference Data**: 24 hour TTL
- **Master Data**: 1 hour TTL
- **Golden Records**: 5 minute TTL

### 9.2 Connection Pooling

- **HTTP**: Keep-alive connections
- **Database**: Connection pool size = (cores × 2) + 1
- **Kafka**: Reuse producer/consumer instances

## 10. Monitoring and Observability

### 10.1 Metrics to Track

- **Throughput**: Messages/sec, Requests/sec
- **Latency**: p50, p95, p99 response times
- **Error Rate**: 4xx, 5xx error percentages
- **Data Quality**: Quality scores, issue counts

### 10.2 Distributed Tracing

- **OpenTelemetry**: Standard for tracing
- **Trace Context**: Propagate across service boundaries
- **Correlation IDs**: Track request flows

## Summary

This protocol specification ensures reliable, secure, and efficient communication and data synchronization across MDM implementations.
