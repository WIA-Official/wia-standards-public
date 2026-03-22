# Identity Theft Prevention — Phase 1: Data Format Specification

> **Version:** 1.0.0
> **Status:** Official
> **Last Updated:** 2025-01-01
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

Phase 1 defines standardized data formats for Identity Theft Prevention systems. This specification establishes the core data model, field definitions, validation rules, and serialization formats to ensure interoperability across authentication systems and identity platforms.

---

## 2. Data Model

### 2.1 Core Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "WIA Identity Theft Prevention Data Format",
  "type": "object",
  "required": ["standard", "version", "timestamp", "data"],
  "properties": {
    "standard": {
      "type": "string",
      "const": "WIA-IDENTITY_THEFT_PREVENTION"
    },
    "version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+\\.\\d+$"
    },
    "timestamp": {
      "type": "string",
      "format": "date-time"
    },
    "metadata": {
      "type": "object",
      "properties": {
        "source": {"type": "string"},
        "encoding": {"type": "string", "enum": ["utf-8", "binary", "base64"]},
        "checksum": {"type": "string"}
      }
    },
    "data": {
      "type": "object",
      "description": "Primary identity verification data payload"
    }
  }
}
```

### 2.2 Field Definitions

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| standard | string | Yes | Standard identifier (WIA-IDENTITY_THEFT_PREVENTION) |
| version | string | Yes | Specification version (SemVer) |
| timestamp | string | Yes | ISO 8601 timestamp |
| metadata.source | string | No | Data source identifier |
| metadata.encoding | string | No | Content encoding type |
| metadata.checksum | string | No | SHA-256 integrity hash |
| data | object | Yes | Domain-specific identity verification data |

---

## 3. Data Types

### 3.1 Enumerations

| Enum | Values | Description |
|------|--------|-------------|
| Status | `active`, `inactive`, `pending`, `error` | Operational status |
| Priority | `critical`, `high`, `medium`, `low` | Processing priority |
| DataQuality | `verified`, `provisional`, `estimated` | Data quality level |

### 3.2 Measurement Units

All measurements follow SI units unless domain-specific conventions apply. Timestamps use ISO 8601 with UTC timezone.

---

## 4. Validation Rules

Implementations MUST:
1. Validate all required fields before processing
2. Reject payloads with unknown `standard` identifiers
3. Verify version compatibility (major version must match)
4. Validate timestamp format and reject future-dated entries beyond tolerance
5. Compute and verify checksums when provided

Implementations SHOULD:
1. Support partial updates via JSON Patch (RFC 6902)
2. Log validation failures with diagnostic details
3. Provide human-readable error descriptions

---

## 5. Serialization

### 5.1 JSON (Primary)
- UTF-8 encoding required
- Maximum payload size: 16 MB
- Compression: gzip or brotli recommended for payloads > 1 KB

### 5.2 Binary (Protocol Buffers)
- Protobuf schema provided for high-throughput applications
- Compatible with gRPC transport (see Phase 3)

### 5.3 File Extensions
- `.wia-identity-theft-preve.json` — JSON format
- `.wia-identity-theft-preve.pb` — Protocol Buffers

---

## 6. Examples

### Minimal Valid Payload

```json
{
  "standard": "WIA-IDENTITY_THEFT_PREVENTION",
  "version": "1.0.0",
  "timestamp": "2025-01-15T10:30:00Z",
  "data": {}
}
```

---

**© 2025 SmileStory Inc. / WIA - World Certification Industry Association**
**弘益人間 · Benefit All Humanity**
