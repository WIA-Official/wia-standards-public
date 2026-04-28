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

---

## Z.1 Audit transport and observability hooks (Phase 1)

Every Phase 1 envelope SHOULD emit a structured log line at the
host's audit transport: timestamp per RFC 3339, host identifier,
tenant identifier, envelope class, envelope identifier, operation
outcome, and a W3C Trace Context `traceparent` propagated end-to-end
so a single operation can be reconstructed across hosts. Phase 2
surfaces this trace identifier as the `X-WIA-Trace-Id` response
header. Phase 3 protocol exchanges propagate the trace identifier
inside the exchange envelope so that a federation crossing remains
correlatable end-to-end. Phase 4 integrators consume the audit
stream into the operator's SIEM (Splunk, Elastic, Sumo Logic,
Wazuh, Microsoft Sentinel) per OpenTelemetry semantic conventions,
with `wia.standard.slug` = `streaming-data` and `wia.standard.phase` =
`1` as required attributes. The audit envelope follows the
canonical W3C Trace Context binary format on the wire when the
host operates over a binary protocol (e.g., gRPC over HTTP/2 or
MQTT 5) and the canonical W3C Trace Context text format when the
host operates over a text protocol (e.g., HTTP/1.1 or REST/JSON).

## Z.2 Cross-standard composition (Phase 1)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 Sec 5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, one signing key
chain, and one audit transport rather than maintaining N parallel
implementations. The composition also lets the operator's SIEM
correlate per-tenant audit records across multiple standards
without per-standard schema-mapping work.

## Z.3 Capabilities discovery and SemVer (Phase 1)

Hosts SHOULD publish a capabilities document at
`/.well-known/wia-streaming-data-capabilities` enumerating per-endpoint
optionality. Clients MUST treat unsupported capabilities as absent
rather than as an error condition; a client that needs a capability
the host does not advertise MUST surface a clear configuration
error rather than silently degrade. Hosts moving from one minor
version to the next MUST publish the change in the host's release
notes with the per-capability migration window per IETF RFC 8594
(Sunset header) + RFC 9745 (Deprecation header) + RFC 9651
(Structured Field Values) so machine consumers can plan migration
without waiting for human-channel notification.

## Z.4 Privacy envelope per per-jurisdiction law (Phase 1)

Phase 1 envelopes that carry personal data MUST honour the
operator's per-jurisdiction privacy law (EU GDPR per Regulation
2016/679; UK GDPR per UK Data Protection Act 2018; California
CPRA per Cal. Civ. Code Sec 1798.100; Brazil LGPD per Lei 13.709/2018;
Canada PIPEDA per S.C. 2000 c.5; Korea PIPA per 개인정보 보호법;
Japan APPI per 個人情報の保護に関する法律; Australia Privacy Act
1988 per Cth) including data-minimisation, purpose-limitation,
storage-limitation, integrity + confidentiality, and accountability
principles. Subject-rights endpoints (access, rectification,
erasure, portability, restriction, objection) compose with
WIA-OMNI-API Sec 5 subject-rights surface and need not be
re-implemented per-standard.

## Z.5 DR / continuity envelope per ISO 22301 (Phase 1)

Hosts running this Phase MUST publish a continuity-of-operations
envelope per ISO 22301:2019 + ISO/IEC 27031 + NIST SP 800-34 Rev 1
covering: per-host RTO (Recovery Time Objective) per the operator's
business-impact analysis; per-host RPO (Recovery Point Objective)
tied to the host's audit-stream replication policy; per-host
backup envelope (per-region cross-replicated immutable backup with
the per-tier retention envelope per the per-jurisdiction record-
retention policy); per-host failover-rehearsal envelope (typically
quarterly per the operator's BC/DR program); per-host vendor-exit
envelope so the operator can migrate the host to an alternate
implementation without losing audit-trail continuity.

## Z.6 Supply-chain envelope per SLSA (Phase 1)

Every host implementation MUST publish a software-bill-of-materials
(SBOM) per SPDX 2.3 / 3.0 (per ISO/IEC 5962 + Linux Foundation SPDX)
or CycloneDX 1.6 (per OWASP Foundation). The SBOM enumerates every
direct + transitive dependency with the per-component name +
version + licence + supplier + per-component hash + per-component
PURL (Package URL per package-url spec) + per-component CPE
(Common Platform Enumeration per NIST). Supply-chain attestation
follows in-toto per CNCF in-toto + SLSA (Supply-chain Levels for
Software Artifacts) per OpenSSF SLSA Framework — typically targeting
SLSA Level 3 for hosted production deployments.

弘益人間 — Benefit All Humanity.
