# WIA-CORE-008: Universal Metadata - PHASE 1 Specification

**Version:** 1.0.0
**Status:** Stable
**Last Updated:** December 2025
**Phase:** Foundation & Core Data Model

## Overview

PHASE 1 defines the foundational elements of the Universal Metadata standard, including the core data model, required fields, data types, and basic validation rules. This phase establishes the minimum viable metadata structure that all implementations must support.

## Core Data Model

### 1.1 Metadata Record Structure

Every Universal Metadata record MUST be a valid JSON object containing the following structure:

```json
{
  "standard": "WIA-CORE-008",
  "version": "1.0",
  "id": "string (URI format recommended)",
  "title": "string (10-200 characters)",
  "language": "string (ISO 639-1 or 639-3)",
  "domain": "string",
  "created": "ISO 8601 datetime",
  "modified": "ISO 8601 datetime"
}
```

### 1.2 Required Fields Specification

#### id (REQUIRED)
- **Type:** String
- **Format:** URI recommended (e.g., `https://example.org/resources/12345`)
- **Constraints:**
  - MUST be globally unique
  - MUST NOT exceed 255 characters
  - SHOULD be persistent and dereferenceable
  - MAY use UUID, DOI, ARK, or custom URI scheme

**Example:**
```json
"id": "https://doi.org/10.1234/example.2025.001"
```

#### title (REQUIRED)
- **Type:** String
- **Constraints:**
  - MUST be between 10 and 200 characters
  - MUST NOT contain only whitespace
  - SHOULD be descriptive and human-readable
  - MAY include punctuation and special characters

**Example:**
```json
"title": "Introduction to Universal Metadata Standards"
```

#### language (REQUIRED)
- **Type:** String
- **Format:** ISO 639-1 (two-letter) or ISO 639-3 (three-letter) language code
- **Constraints:**
  - MUST be a valid ISO 639 code
  - SHOULD use ISO 639-1 when available
  - MUST be lowercase

**Example:**
```json
"language": "en"
```

#### domain (REQUIRED)
- **Type:** String
- **Constraints:**
  - MUST be one of the predefined domain values OR a custom namespaced domain
  - MUST be lowercase
  - MUST NOT contain spaces (use hyphens or underscores)

**Standard Domains:**
- education
- healthcare
- research
- finance
- technology
- media
- government
- business
- culture
- environment

**Example:**
```json
"domain": "research"
```

#### created (REQUIRED)
- **Type:** String
- **Format:** ISO 8601 datetime with timezone
- **Constraints:**
  - MUST be a valid ISO 8601 timestamp
  - MUST include timezone information
  - MUST NOT be in the future

**Example:**
```json
"created": "2025-01-15T10:30:00Z"
```

#### modified (REQUIRED)
- **Type:** String
- **Format:** ISO 8601 datetime with timezone
- **Constraints:**
  - MUST be a valid ISO 8601 timestamp
  - MUST include timezone information
  - MUST be greater than or equal to `created`
  - MUST be updated when any field changes

**Example:**
```json
"modified": "2025-01-20T14:45:00Z"
```

## 1.3 Data Types and Validation

### String Type
- Encoding: UTF-8
- Null values: NOT permitted for required fields
- Empty strings: NOT permitted for required fields
- Whitespace: Leading/trailing whitespace SHOULD be trimmed

### DateTime Type
- Format: ISO 8601 (YYYY-MM-DDTHH:MM:SSZ or with timezone offset)
- Precision: Millisecond precision is supported but not required
- Timezone: MUST be specified (UTC recommended)

### URI Type
- Format: RFC 3986 compliant URIs
- Schemes: http, https, doi, urn, uuid, ark all supported
- Validation: MUST be syntactically valid URI

## 1.4 Minimal Valid Record

The absolute minimum valid Universal Metadata record:

```json
{
  "standard": "WIA-CORE-008",
  "version": "1.0",
  "id": "https://example.org/minimal/001",
  "title": "Minimal Record Example",
  "language": "en",
  "domain": "general",
  "created": "2025-01-01T00:00:00Z",
  "modified": "2025-01-01T00:00:00Z"
}
```

## 1.5 Validation Rules

### Field Presence Validation
1. All REQUIRED fields MUST be present
2. Field names MUST be exactly as specified (case-sensitive)
3. Additional fields are permitted (see PHASE 2 for recommended fields)

### Data Type Validation
1. String fields MUST be valid UTF-8
2. DateTime fields MUST parse as valid ISO 8601
3. URI fields MUST parse as valid URIs

### Value Validation
1. `title` length: 10 <= length <= 200
2. `language` MUST be valid ISO 639 code
3. `domain` MUST be valid predefined or namespaced domain
4. `created` <= `modified`
5. `created` and `modified` MUST NOT be future dates

### Uniqueness Validation
1. `id` MUST be globally unique within the system
2. Duplicate `id` values MUST be rejected

## 1.6 Error Handling

### Validation Errors
When validation fails, implementations MUST return structured error information:

```json
{
  "valid": false,
  "errors": [
    {
      "field": "title",
      "code": "LENGTH_CONSTRAINT",
      "message": "Title must be between 10 and 200 characters",
      "value": "Short"
    }
  ]
}
```

### Error Codes
- `MISSING_REQUIRED_FIELD`: Required field not present
- `INVALID_DATA_TYPE`: Field value does not match required type
- `LENGTH_CONSTRAINT`: String length outside permitted range
- `FORMAT_ERROR`: Value does not match required format
- `DUPLICATE_ID`: ID already exists in system
- `INVALID_DATETIME`: DateTime does not parse as ISO 8601
- `FUTURE_DATETIME`: DateTime is in the future (not permitted for created/modified)
- `TEMPORAL_INCONSISTENCY`: modified < created

## 1.7 Serialization Formats

### JSON (Primary)
- Default serialization format
- UTF-8 encoding required
- Pretty printing recommended for human readability
- Compact JSON acceptable for transmission

### JSON Schema
A JSON Schema is provided for validation:

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "type": "object",
  "required": ["standard", "version", "id", "title", "language", "domain", "created", "modified"],
  "properties": {
    "standard": {
      "type": "string",
      "const": "WIA-CORE-008"
    },
    "version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+"
    },
    "id": {
      "type": "string",
      "minLength": 1,
      "maxLength": 255
    },
    "title": {
      "type": "string",
      "minLength": 10,
      "maxLength": 200
    },
    "language": {
      "type": "string",
      "pattern": "^[a-z]{2,3}$"
    },
    "domain": {
      "type": "string",
      "pattern": "^[a-z][a-z0-9_-]*$"
    },
    "created": {
      "type": "string",
      "format": "date-time"
    },
    "modified": {
      "type": "string",
      "format": "date-time"
    }
  }
}
```

## 1.8 Implementation Requirements

### MUST Requirements
1. Implementations MUST validate all required fields
2. Implementations MUST reject invalid metadata
3. Implementations MUST preserve field order when possible
4. Implementations MUST handle UTF-8 encoding correctly
5. Implementations MUST provide structured error messages

### SHOULD Requirements
1. Implementations SHOULD support JSON Schema validation
2. Implementations SHOULD log validation errors
3. Implementations SHOULD provide helpful error messages
4. Implementations SHOULD support batch validation

### MAY Requirements
1. Implementations MAY support additional serialization formats (XML, YAML)
2. Implementations MAY provide auto-correction for common errors
3. Implementations MAY support partial validation (field-by-field)

## 1.9 Migration Path

For systems migrating to Universal Metadata:

1. **Identify Existing Fields:** Map existing metadata fields to required fields
2. **Generate IDs:** Create globally unique identifiers for existing records
3. **Set Timestamps:** Use creation/modification dates or current time if unavailable
4. **Language Detection:** Detect or assign language codes
5. **Domain Classification:** Categorize content into appropriate domains
6. **Validate:** Run validation and address errors before full migration

## 1.10 Compliance Testing

To verify PHASE 1 compliance:

1. Create minimal valid record
2. Attempt to create record missing each required field (should fail)
3. Attempt to create record with invalid data types (should fail)
4. Attempt to create record with constraint violations (should fail)
5. Create record with future dates (should fail)
6. Create record with temporal inconsistency (should fail)
7. Verify error messages are structured and helpful

## 1.11 Examples

### Valid Education Resource
```json
{
  "standard": "WIA-CORE-008",
  "version": "1.0",
  "id": "https://university.edu/courses/cs101",
  "title": "Introduction to Computer Science - Spring 2025",
  "language": "en",
  "domain": "education",
  "created": "2024-12-01T09:00:00Z",
  "modified": "2025-01-15T14:30:00Z"
}
```

### Valid Research Paper
```json
{
  "standard": "WIA-CORE-008",
  "version": "1.0",
  "id": "https://doi.org/10.1234/research.2025.042",
  "title": "Novel Approaches to Metadata Quality Assessment",
  "language": "en",
  "domain": "research",
  "created": "2025-01-10T00:00:00Z",
  "modified": "2025-01-10T00:00:00Z"
}
```

### Valid Multilingual Content
```json
{
  "standard": "WIA-CORE-008",
  "version": "1.0",
  "id": "urn:uuid:550e8400-e29b-41d4-a716-446655440000",
  "title": "양자 컴퓨팅의 미래 전망",
  "language": "ko",
  "domain": "technology",
  "created": "2025-01-20T03:00:00+09:00",
  "modified": "2025-01-20T03:00:00+09:00"
}
```

## Next Phase

PHASE 2 introduces recommended fields, quality metrics, and extension mechanisms. See [PHASE2.md](./PHASE2.md) for details.

---

**© 2025 SmileStory Inc. / WIA**
**弘益人間 (홍익인간) · Benefit All Humanity**
