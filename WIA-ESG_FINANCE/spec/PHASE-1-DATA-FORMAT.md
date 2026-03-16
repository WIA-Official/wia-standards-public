# WIA-ESG_FINANCE: Phase 1 - Data Format Specification
**Version:** 1.0
**Status:** Official
**Philosophy:** 弘益人間 (Benefit All Humanity)

## 1. Overview

This document specifies the data format requirements for ESG FINANCE. All implementations MUST follow these specifications to ensure interoperability across systems.

## 2. Data Structures

### 2.1 Primary Record Format

```json
{
  "type": "WIA-ESG_FINANCERecord",
  "version": "1.0",
  "id": "string (UUID v4)",
  "timestamp": "ISO 8601",
  "data": {
    "category": "string",
    "value": "any",
    "metadata": {}
  },
  "signature": "string (Ed25519)"
}
```

### 2.2 Field Definitions

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| type | string | Yes | Record type identifier |
| version | string | Yes | Specification version |
| id | UUID | Yes | Unique record identifier |
| timestamp | ISO 8601 | Yes | Record creation time |
| data | object | Yes | Primary data payload |
| signature | string | No | Cryptographic signature |

## 3. Data Types

### 3.1 Core Types

| Type | Format | Example |
|------|--------|---------|
| String | UTF-8 | "Hello World" |
| Number | IEEE 754 | 3.14159 |
| Boolean | true/false | true |
| Date | ISO 8601 | "2025-01-01T00:00:00Z" |
| UUID | RFC 4122 | "550e8400-e29b-41d4-a716-446655440000" |

### 3.2 Extended Types

```typescript
interface ExtendedData {
  binaryData: ArrayBuffer;
  coordinates: GeoCoordinate;
  range: NumberRange;
  collection: DataCollection;
}

interface GeoCoordinate {
  latitude: number;   // -90 to 90
  longitude: number;  // -180 to 180
  altitude?: number;  // meters
}

interface NumberRange {
  min: number;
  max: number;
  step?: number;
}
```

## 4. Encoding Requirements

### 4.1 Character Encoding
- All text MUST be UTF-8 encoded
- No BOM (Byte Order Mark) allowed
- Line endings: LF (Unix style)

### 4.2 Binary Encoding
- Use Base64 for binary data in JSON
- Use Protocol Buffers for high-performance scenarios
- Big-endian byte order for network transmission

## 5. Validation Rules

### 5.1 Schema Validation
```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "type": "object",
  "required": ["type", "version", "id", "timestamp", "data"],
  "properties": {
    "type": { "type": "string", "const": "WIA-ESG_FINANCERecord" },
    "version": { "type": "string", "pattern": "^\\d+\\.\\d+$" },
    "id": { "type": "string", "format": "uuid" },
    "timestamp": { "type": "string", "format": "date-time" }
  }
}
```

### 5.2 Validation Errors

| Code | Message | Resolution |
|------|---------|------------|
| E001 | Invalid type | Use correct record type |
| E002 | Missing required field | Add all required fields |
| E003 | Invalid format | Check field format specifications |
| E004 | Schema mismatch | Validate against JSON schema |

## 6. Versioning

### 6.1 Version Format
- Major.Minor (e.g., 1.0, 2.1)
- Major version: Breaking changes
- Minor version: Backward-compatible additions

### 6.2 Migration Path
- v1.0 → v1.1: Add optional fields
- v1.x → v2.0: Schema restructuring

## 7. Examples

### 7.1 Basic Record
```json
{
  "type": "WIA-ESG_FINANCERecord",
  "version": "1.0",
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "timestamp": "2025-01-01T00:00:00Z",
  "data": {
    "category": "standard",
    "value": "example data",
    "metadata": {
      "source": "system",
      "priority": "normal"
    }
  }
}
```

---

**弘益人間 (Benefit All Humanity)**

*© 2025 WIA - World Certification Industry Association*
*MIT License*
