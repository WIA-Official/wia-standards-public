# WIA Cryo-Preservation Data Format Standard
## Phase 1 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #06B6D4 (Cyan)

---

## Table of Contents

1. [Overview](#overview)
2. [Terminology](#terminology)
3. [Base Structure](#base-structure)
4. [Data Schema](#data-schema)
5. [Field Specifications](#field-specifications)
6. [Data Types](#data-types)
7. [Validation Rules](#validation-rules)
8. [Examples](#examples)
9. [Version History](#version-history)

---

## Overview

### 1.1 Purpose

The WIA Cryo-Preservation Data Format Standard defines a unified JSON-based format for recording, transmitting, and managing cryopreservation data across healthcare facilities, research institutions, and cryonics organizations worldwide.

**Core Objectives**:
- Standardize cryopreservation records across all facilities
- Enable interoperability between different cryonics providers
- Ensure data integrity and traceability throughout the preservation lifecycle
- Support regulatory compliance and audit requirements

### 1.2 Scope

This standard covers the following data domains:

| Domain | Description |
|--------|-------------|
| Patient Records | Personal and medical information of cryopreserved individuals |
| Preservation Procedures | Details of vitrification and cooling processes |
| Storage Conditions | Environmental monitoring and maintenance data |
| Chain of Custody | Transfer and handling records |
| Quality Metrics | Preservation quality indicators |

### 1.3 Design Principles

1. **Immutability**: Once created, records cannot be modified, only appended
2. **Traceability**: Complete audit trail for all operations
3. **Interoperability**: Compatible with existing healthcare standards (HL7, FHIR)
4. **Security**: End-to-end encryption for sensitive data
5. **Validation**: JSON Schema enforcement for data integrity

---

## Terminology

### 2.1 Core Terms

| Term | Definition |
|------|------------|
| **Subject** | The individual undergoing cryopreservation |
| **Vitrification** | Glass-like solidification without ice crystal formation |
| **Perfusion** | Process of replacing blood with cryoprotectant |
| **Dewar** | Vacuum-insulated container for liquid nitrogen storage |
| **CPA** | Cryoprotective Agent - chemicals preventing ice damage |
| **Cool-down** | Controlled temperature reduction process |
| **Long-term Storage** | Maintenance at cryogenic temperatures (-196°C) |

### 2.2 Data Types

| Type | Description | Example |
|------|-------------|---------|
| `string` | UTF-8 encoded text | `"WIA-CRYO-001"` |
| `number` | IEEE 754 double precision | `-196.0`, `3.14159` |
| `integer` | Signed 64-bit integer | `1`, `-273` |
| `boolean` | Boolean value | `true`, `false` |
| `timestamp` | ISO 8601 datetime | `"2025-01-15T10:30:00Z"` |
| `uuid` | UUID v4 identifier | `"550e8400-e29b-41d4-a716-446655440000"` |

### 2.3 Field Requirements

| Marker | Meaning |
|--------|---------|
| **REQUIRED** | Field must be present |
| **OPTIONAL** | Field may be omitted |
| **CONDITIONAL** | Required under specific conditions |

---

## Base Structure

### 3.1 Message Format

All WIA Cryo-Preservation messages follow this base structure:

```json
{
  "$schema": "https://wia.live/cryo-preservation/v1/schema.json",
  "version": "1.0.0",
  "messageId": "uuid-v4-string",
  "messageType": "preservation_record",
  "timestamp": {
    "created": "2025-01-15T10:30:00Z",
    "modified": "2025-01-15T10:30:00Z"
  },
  "facility": {
    "id": "FAC-001",
    "name": "WIA Cryonics Institute",
    "location": {
      "country": "KR",
      "city": "Seoul"
    }
  },
  "subject": {
    "id": "SUBJ-001",
    "anonymizedId": "ANON-550e8400"
  },
  "data": {
    // Domain-specific data
  },
  "meta": {
    "hash": "sha256-hash",
    "signature": "digital-signature",
    "previousHash": "previous-record-hash"
  }
}
```

### 3.2 Field Details

#### 3.2.1 `$schema` (OPTIONAL)

```
Type: string
Format: URI
Description: JSON Schema location for validation
Example: "https://wia.live/cryo-preservation/v1/schema.json"
```

#### 3.2.2 `version` (REQUIRED)

```
Type: string
Format: Semantic Versioning (MAJOR.MINOR.PATCH)
Description: Specification version
Example: "1.0.0"
```

#### 3.2.3 `messageId` (REQUIRED)

```
Type: string
Format: UUID v4
Description: Unique identifier for this message
Example: "550e8400-e29b-41d4-a716-446655440000"
```

#### 3.2.4 `messageType` (REQUIRED)

```
Type: string
Description: Type of message
Valid values:
  - "preservation_record"  : Main preservation data
  - "status_update"        : Status change notification
  - "quality_report"       : Quality assessment
  - "transfer_record"      : Custody transfer
  - "maintenance_log"      : Facility maintenance
```

---

## Data Schema

### 4.1 Complete JSON Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wia.live/cryo-preservation/v1/schema.json",
  "title": "WIA Cryo-Preservation Record",
  "type": "object",
  "required": ["version", "messageId", "messageType", "timestamp", "facility", "subject", "data"],
  "properties": {
    "version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+\\.\\d+$"
    },
    "messageId": {
      "type": "string",
      "format": "uuid"
    },
    "messageType": {
      "type": "string",
      "enum": ["preservation_record", "status_update", "quality_report", "transfer_record", "maintenance_log"]
    },
    "timestamp": {
      "type": "object",
      "required": ["created"],
      "properties": {
        "created": { "type": "string", "format": "date-time" },
        "modified": { "type": "string", "format": "date-time" }
      }
    },
    "facility": {
      "type": "object",
      "required": ["id", "name"],
      "properties": {
        "id": { "type": "string" },
        "name": { "type": "string" },
        "location": {
          "type": "object",
          "properties": {
            "country": { "type": "string", "pattern": "^[A-Z]{2}$" },
            "city": { "type": "string" },
            "address": { "type": "string" },
            "coordinates": {
              "type": "object",
              "properties": {
                "latitude": { "type": "number", "minimum": -90, "maximum": 90 },
                "longitude": { "type": "number", "minimum": -180, "maximum": 180 }
              }
            }
          }
        },
        "certification": {
          "type": "array",
          "items": { "type": "string" }
        }
      }
    },
    "subject": {
      "type": "object",
      "required": ["id"],
      "properties": {
        "id": { "type": "string" },
        "anonymizedId": { "type": "string" },
        "consentId": { "type": "string" },
        "demographics": {
          "type": "object",
          "properties": {
            "birthYear": { "type": "integer" },
            "biologicalSex": { "type": "string", "enum": ["male", "female", "other"] },
            "bloodType": { "type": "string" }
          }
        }
      }
    },
    "data": {
      "type": "object"
    },
    "meta": {
      "type": "object",
      "properties": {
        "hash": { "type": "string" },
        "signature": { "type": "string" },
        "previousHash": { "type": "string" },
        "version": { "type": "integer" }
      }
    }
  }
}
```

### 4.2 Preservation Data Schema

```json
{
  "data": {
    "preservationType": "whole_body",
    "status": "long_term_storage",
    "timeline": {
      "pronouncement": "2025-01-15T08:00:00Z",
      "stabilization_start": "2025-01-15T08:15:00Z",
      "perfusion_start": "2025-01-15T10:00:00Z",
      "perfusion_complete": "2025-01-15T14:00:00Z",
      "cooldown_start": "2025-01-15T14:30:00Z",
      "cooldown_complete": "2025-01-16T02:30:00Z",
      "storage_start": "2025-01-16T03:00:00Z"
    },
    "perfusion": {
      "cryoprotectant": "M22",
      "concentration": 0.7,
      "volume_liters": 15.5,
      "duration_minutes": 240,
      "temperature_celsius": 0
    },
    "cooldown": {
      "method": "controlled_rate",
      "rate_celsius_per_minute": -1.0,
      "intermediate_holds": [
        { "temperature": -40, "duration_minutes": 30 },
        { "temperature": -80, "duration_minutes": 60 }
      ],
      "final_temperature": -196
    },
    "storage": {
      "container_type": "dewar",
      "container_id": "DEW-001",
      "position": "A1-05",
      "medium": "liquid_nitrogen",
      "temperature_celsius": -196
    },
    "quality": {
      "vitrification_score": 0.92,
      "tissue_integrity": 0.88,
      "cpa_distribution": 0.95,
      "assessment_date": "2025-01-16T06:00:00Z"
    }
  }
}
```

---

## Field Specifications

### 5.1 Preservation Type

| Field | Type | Required | Description | Example |
|-------|------|----------|-------------|---------|
| `preservationType` | string | REQUIRED | Type of preservation | `"whole_body"` |
| `status` | string | REQUIRED | Current preservation status | `"long_term_storage"` |

**Valid preservationType values:**

| Value | Description |
|-------|-------------|
| `whole_body` | Complete body preservation |
| `neuro` | Head/brain only preservation |
| `tissue_sample` | Individual tissue samples |
| `organ` | Specific organ preservation |

**Valid status values:**

| Value | Description |
|-------|-------------|
| `pending` | Awaiting preservation |
| `stabilization` | Initial stabilization in progress |
| `perfusion` | Cryoprotectant perfusion in progress |
| `cooldown` | Temperature reduction in progress |
| `long_term_storage` | Maintained at storage temperature |
| `transferred` | Moved to another facility |
| `revived` | Successfully reanimated |

### 5.2 Timeline Fields

| Field | Type | Required | Description | Example |
|-------|------|----------|-------------|---------|
| `pronouncement` | timestamp | REQUIRED | Legal time of death | `"2025-01-15T08:00:00Z"` |
| `stabilization_start` | timestamp | REQUIRED | Start of stabilization | `"2025-01-15T08:15:00Z"` |
| `perfusion_start` | timestamp | CONDITIONAL | Start of perfusion | `"2025-01-15T10:00:00Z"` |
| `perfusion_complete` | timestamp | CONDITIONAL | End of perfusion | `"2025-01-15T14:00:00Z"` |
| `cooldown_start` | timestamp | REQUIRED | Start of cooling | `"2025-01-15T14:30:00Z"` |
| `cooldown_complete` | timestamp | REQUIRED | End of cooling | `"2025-01-16T02:30:00Z"` |
| `storage_start` | timestamp | REQUIRED | Start of long-term storage | `"2025-01-16T03:00:00Z"` |

### 5.3 Perfusion Fields

| Field | Type | Required | Description | Example |
|-------|------|----------|-------------|---------|
| `cryoprotectant` | string | REQUIRED | CPA compound name | `"M22"` |
| `concentration` | number | REQUIRED | Final concentration (0.0-1.0) | `0.7` |
| `volume_liters` | number | REQUIRED | Total volume used | `15.5` |
| `duration_minutes` | integer | REQUIRED | Total perfusion time | `240` |
| `temperature_celsius` | number | REQUIRED | Perfusion temperature | `0` |

### 5.4 Quality Metrics

| Field | Type | Required | Description | Range |
|-------|------|----------|-------------|-------|
| `vitrification_score` | number | OPTIONAL | Quality of vitrification | 0.0-1.0 |
| `tissue_integrity` | number | OPTIONAL | Tissue preservation quality | 0.0-1.0 |
| `cpa_distribution` | number | OPTIONAL | CPA penetration uniformity | 0.0-1.0 |
| `assessment_date` | timestamp | CONDITIONAL | Date of quality assessment | ISO 8601 |

---

## Data Types

### 6.1 Custom Types

#### PreservationType

```typescript
type PreservationType =
  | 'whole_body'
  | 'neuro'
  | 'tissue_sample'
  | 'organ';
```

#### PreservationStatus

```typescript
type PreservationStatus =
  | 'pending'
  | 'stabilization'
  | 'perfusion'
  | 'cooldown'
  | 'long_term_storage'
  | 'transferred'
  | 'revived';
```

#### Temperature

```typescript
interface Temperature {
  value: number;      // Celsius
  unit: 'celsius';
  precision?: number; // Decimal places
}
```

### 6.2 Enum Values

#### Cryoprotectant Types

| Code | Name | Description |
|------|------|-------------|
| `M22` | M22 Vitrification Solution | Most advanced, lowest toxicity |
| `VM1` | Vitrification Mixture 1 | Standard solution |
| `DMSO` | Dimethyl Sulfoxide | Basic CPA |
| `glycerol` | Glycerol | Traditional CPA |
| `EG` | Ethylene Glycol | Common component |
| `custom` | Custom Mixture | Proprietary formulation |

#### Container Types

| Code | Description |
|------|-------------|
| `dewar` | Vacuum-insulated Dewar flask |
| `cryostat` | Temperature-controlled storage |
| `transport_container` | Portable cryogenic vessel |
| `sample_vial` | Small sample container |

---

## Validation Rules

### 7.1 Required Field Validation

| Rule ID | Field | Validation |
|---------|-------|------------|
| VAL-001 | `version` | Must match `^\d+\.\d+\.\d+$` |
| VAL-002 | `messageId` | Must be valid UUID v4 |
| VAL-003 | `timestamp.created` | Must be valid ISO 8601 |
| VAL-004 | `facility.id` | Must not be empty |
| VAL-005 | `subject.id` | Must not be empty |

### 7.2 Business Logic Validation

| Rule ID | Description | Error Code |
|---------|-------------|------------|
| BUS-001 | `cooldown_start` must be after `perfusion_complete` | `ERR_TIMELINE_ORDER` |
| BUS-002 | `storage_start` must be after `cooldown_complete` | `ERR_TIMELINE_ORDER` |
| BUS-003 | `concentration` must be between 0.0 and 1.0 | `ERR_INVALID_RANGE` |
| BUS-004 | `final_temperature` must be ≤ -196°C | `ERR_INVALID_TEMPERATURE` |
| BUS-005 | Quality scores must be between 0.0 and 1.0 | `ERR_INVALID_SCORE` |

### 7.3 Error Codes

| Code | Message | Description |
|------|---------|-------------|
| `ERR_INVALID_FORMAT` | Invalid data format | JSON parsing failed |
| `ERR_MISSING_FIELD` | Required field missing | Required field not present |
| `ERR_INVALID_TYPE` | Invalid field type | Type mismatch |
| `ERR_TIMELINE_ORDER` | Invalid timeline sequence | Events out of order |
| `ERR_INVALID_RANGE` | Value out of range | Value exceeds limits |
| `ERR_INVALID_TEMPERATURE` | Invalid temperature | Temperature constraint violated |
| `ERR_INVALID_SCORE` | Invalid quality score | Score outside 0.0-1.0 |

---

## Examples

### 8.1 Valid Preservation Record

```json
{
  "$schema": "https://wia.live/cryo-preservation/v1/schema.json",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440001",
  "messageType": "preservation_record",
  "timestamp": {
    "created": "2025-01-15T10:30:00Z",
    "modified": "2025-01-15T10:30:00Z"
  },
  "facility": {
    "id": "FAC-KR-001",
    "name": "WIA Cryonics Institute Seoul",
    "location": {
      "country": "KR",
      "city": "Seoul",
      "coordinates": {
        "latitude": 37.5665,
        "longitude": 126.9780
      }
    },
    "certification": ["ISO-9001", "WIA-CRYO-CERTIFIED"]
  },
  "subject": {
    "id": "SUBJ-2025-001",
    "anonymizedId": "ANON-550e8400",
    "consentId": "CONSENT-2024-1234",
    "demographics": {
      "birthYear": 1950,
      "biologicalSex": "male",
      "bloodType": "A+"
    }
  },
  "data": {
    "preservationType": "whole_body",
    "status": "long_term_storage",
    "timeline": {
      "pronouncement": "2025-01-15T08:00:00Z",
      "stabilization_start": "2025-01-15T08:15:00Z",
      "perfusion_start": "2025-01-15T10:00:00Z",
      "perfusion_complete": "2025-01-15T14:00:00Z",
      "cooldown_start": "2025-01-15T14:30:00Z",
      "cooldown_complete": "2025-01-16T02:30:00Z",
      "storage_start": "2025-01-16T03:00:00Z"
    },
    "perfusion": {
      "cryoprotectant": "M22",
      "concentration": 0.7,
      "volume_liters": 15.5,
      "duration_minutes": 240,
      "temperature_celsius": 0
    },
    "cooldown": {
      "method": "controlled_rate",
      "rate_celsius_per_minute": -1.0,
      "intermediate_holds": [
        { "temperature": -40, "duration_minutes": 30 },
        { "temperature": -80, "duration_minutes": 60 }
      ],
      "final_temperature": -196
    },
    "storage": {
      "container_type": "dewar",
      "container_id": "DEW-KR-001",
      "position": "A1-05",
      "medium": "liquid_nitrogen",
      "temperature_celsius": -196
    },
    "quality": {
      "vitrification_score": 0.92,
      "tissue_integrity": 0.88,
      "cpa_distribution": 0.95,
      "assessment_date": "2025-01-16T06:00:00Z"
    }
  },
  "meta": {
    "hash": "sha256:a5b9c3d4e5f6...",
    "signature": "eyJhbGciOiJFUzI1NiIs...",
    "version": 1
  }
}
```

### 8.2 Valid Status Update

```json
{
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440002",
  "messageType": "status_update",
  "timestamp": {
    "created": "2025-01-20T09:00:00Z"
  },
  "facility": {
    "id": "FAC-KR-001",
    "name": "WIA Cryonics Institute Seoul"
  },
  "subject": {
    "id": "SUBJ-2025-001"
  },
  "data": {
    "previousStatus": "cooldown",
    "newStatus": "long_term_storage",
    "reason": "Cooldown process completed successfully",
    "verifiedBy": "STAFF-001",
    "notes": "All quality metrics within acceptable range"
  },
  "meta": {
    "previousHash": "sha256:a5b9c3d4e5f6..."
  }
}
```

### 8.3 Valid Quality Report

```json
{
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440003",
  "messageType": "quality_report",
  "timestamp": {
    "created": "2025-02-15T10:00:00Z"
  },
  "facility": {
    "id": "FAC-KR-001",
    "name": "WIA Cryonics Institute Seoul"
  },
  "subject": {
    "id": "SUBJ-2025-001"
  },
  "data": {
    "assessmentType": "monthly_review",
    "storageConditions": {
      "temperature_celsius": -196.2,
      "temperature_variance": 0.3,
      "liquid_nitrogen_level": 0.85
    },
    "quality": {
      "vitrification_score": 0.92,
      "tissue_integrity": 0.88,
      "overall_status": "excellent"
    },
    "inspector": "STAFF-002",
    "nextReviewDate": "2025-03-15"
  }
}
```

### 8.4 Invalid Example - Missing Required Field

```json
{
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440004",
  "messageType": "preservation_record",
  "timestamp": {
    "created": "2025-01-15T10:30:00Z"
  },
  "facility": {
    "name": "WIA Cryonics Institute Seoul"
  },
  "data": {}
}
```

**Error**: `ERR_MISSING_FIELD` - Missing required fields: `facility.id`, `subject`

### 8.5 Invalid Example - Timeline Order Violation

```json
{
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440005",
  "messageType": "preservation_record",
  "timestamp": { "created": "2025-01-15T10:30:00Z" },
  "facility": { "id": "FAC-001", "name": "Test" },
  "subject": { "id": "SUBJ-001" },
  "data": {
    "preservationType": "whole_body",
    "status": "long_term_storage",
    "timeline": {
      "pronouncement": "2025-01-15T08:00:00Z",
      "cooldown_start": "2025-01-15T10:00:00Z",
      "perfusion_start": "2025-01-15T14:00:00Z"
    }
  }
}
```

**Error**: `ERR_TIMELINE_ORDER` - `perfusion_start` must be before `cooldown_start`

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01 | Initial release |

---

## Appendix A: Related Standards

| Standard | Relationship |
|----------|--------------|
| WIA Cryo-Identity | Subject identification linking |
| WIA Cryo-Consent | Consent record references |
| WIA Cryo-Facility | Facility certification data |
| HL7 FHIR | Healthcare data interoperability |
| ISO 27001 | Information security management |

---

<div align="center">

**WIA Cryo-Preservation Data Format Standard v1.0.0**

**弘益人間 (홍익인간)** - Benefit All Humanity

---

**© 2025 WIA Standards Committee**

**MIT License**

</div>
