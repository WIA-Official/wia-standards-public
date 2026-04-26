# WIA Access Control System - Phase 1: Data Format Specification
## Version 1.0

### Document Information
- **Standard:** WIA-ACS (World Industry Association - Access Control System)
- **Phase:** 1 (Data Format)
- **Version:** 1.0
- **Status:** Approved
- **Date:** 2025-12-26
- **Philosophy:** 弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. Introduction

Phase 1 of the WIA Access Control System standard defines standardized data formats for all entities within an access control ecosystem. This foundational phase enables data exchange, reporting, and migration between heterogeneous systems without requiring full API integration.

### 1.1 Scope

This specification covers:
- User entity schema
- Credential entity schema
- Role and permission models
- Access rule schema
- Audit event schema
- Data validation rules
- Import/export formats

### 1.2 Goals

- **Interoperability:** Enable data exchange between different vendors
- **Consistency:** Ensure uniform data representation across systems
- **Extensibility:** Allow for custom attributes while maintaining compatibility
- **Validation:** Provide comprehensive validation rules to ensure data integrity
- **Migration:** Facilitate migration from legacy systems to WIA-ACS compliant implementations

---

## 2. Data Format Requirements

### 2.1 Encoding

All WIA-ACS data MUST be encoded in **UTF-8** without BOM (Byte Order Mark).

### 2.2 Format

The primary data format is **JSON** (JavaScript Object Notation) as defined in [RFC 8259](https://tools.ietf.org/html/rfc8259).

Alternative formats for specific use cases:
- **CSV** (RFC 4180) for bulk import/export
- **NDJSON** (Newline Delimited JSON) for streaming large datasets
- **XML** (optional) for legacy system compatibility

### 2.3 Schema Definition

All schemas are defined using **JSON Schema Draft 2020-12** and published at:
```
https://wia-acs.org/schemas/v1/{entity}.json
```

---

## 3. User Entity

### 3.1 User Schema

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "$id": "https://wia-acs.org/schemas/v1/user.json",
  "title": "WIA-ACS User",
  "type": "object",
  "required": ["user_id", "status", "identity", "roles"],
  "properties": {
    "user_id": {
      "type": "string",
      "pattern": "^usr-[0-9]{8}-[a-z0-9]{6}$",
      "description": "Globally unique user identifier"
    },
    "external_id": {
      "type": "string",
      "maxLength": 255,
      "description": "Identifier in external system (HR, LDAP, etc.)"
    },
    "status": {
      "type": "string",
      "enum": ["active", "suspended", "terminated"],
      "description": "User account status"
    },
    "identity": {
      "type": "object",
      "required": ["given_name", "family_name", "email"],
      "properties": {
        "given_name": {"type": "string", "minLength": 1, "maxLength": 100},
        "family_name": {"type": "string", "minLength": 1, "maxLength": 100},
        "middle_name": {"type": "string", "maxLength": 100},
        "preferred_name": {"type": "string", "maxLength": 100},
        "email": {"type": "string", "format": "email", "maxLength": 255},
        "email_verified": {"type": "boolean"},
        "phone": {"type": "string", "pattern": "^\\+?[1-9]\\d{1,14}$"},
        "phone_verified": {"type": "boolean"}
      }
    },
    "employment": {
      "type": "object",
      "properties": {
        "employee_id": {"type": "string", "maxLength": 50},
        "department": {"type": "string", "maxLength": 100},
        "title": {"type": "string", "maxLength": 100},
        "manager_id": {"type": "string", "pattern": "^usr-[0-9]{8}-[a-z0-9]{6}$"},
        "start_date": {"type": "string", "format": "date"},
        "end_date": {"type": "string", "format": "date"},
        "employment_type": {
          "type": "string",
          "enum": ["full_time", "part_time", "contractor", "intern", "temporary"]
        }
      }
    },
    "roles": {
      "type": "array",
      "minItems": 1,
      "items": {
        "type": "string",
        "pattern": "^role-[a-z0-9-]+$"
      }
    },
    "attributes": {
      "type": "object",
      "description": "Custom attributes for ABAC policies"
    },
    "metadata": {
      "type": "object",
      "properties": {
        "created_at": {"type": "string", "format": "date-time"},
        "updated_at": {"type": "string", "format": "date-time"},
        "created_by": {"type": "string"},
        "updated_by": {"type": "string"}
      }
    }
  }
}
```

### 3.2 User Status Transitions

```
active → suspended (reversible)
active → terminated (irreversible)
suspended → active (requires approval)
terminated → (no transitions allowed)
```

### 3.3 User Validation Rules

1. **user_id** must be globally unique and immutable
2. **email** must be validated before use in authentication
3. **status** transitions must follow defined state machine
4. **roles** array must contain at least one valid role
5. **phone** must follow E.164 international format when provided
6. **created_at** cannot be in the future
7. **updated_at** must be >= created_at

---

## 4. Credential Entity

### 4.1 Credential Schema

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "$id": "https://wia-acs.org/schemas/v1/credential.json",
  "title": "WIA-ACS Credential",
  "type": "object",
  "required": ["credential_id", "user_id", "credential_type", "status", "issued_at", "expires_at"],
  "properties": {
    "credential_id": {
      "type": "string",
      "pattern": "^cred-[0-9]{8}-[a-z0-9]{6}$"
    },
    "user_id": {
      "type": "string",
      "pattern": "^usr-[0-9]{8}-[a-z0-9]{6}$"
    },
    "credential_type": {
      "type": "string",
      "enum": [
        "magnetic_stripe",
        "proximity",
        "smart_card",
        "mobile",
        "biometric",
        "pin_code",
        "otp",
        "certificate"
      ]
    },
    "status": {
      "type": "string",
      "enum": ["active", "suspended", "revoked", "expired"]
    },
    "issued_at": {"type": "string", "format": "date-time"},
    "expires_at": {"type": "string", "format": "date-time"},
    "activates_at": {"type": "string", "format": "date-time"},
    "encoding": {
      "type": "object",
      "description": "Credential-specific encoding data"
    },
    "factors": {
      "type": "array",
      "items": {
        "type": "object",
        "required": ["factor_type", "method", "strength"],
        "properties": {
          "factor_type": {
            "type": "string",
            "enum": ["something_you_know", "something_you_have", "something_you_are", "somewhere_you_are"]
          },
          "method": {"type": "string"},
          "strength": {"type": "string", "enum": ["low", "medium", "high"]},
          "pin_hash": {"type": "string"}
        }
      }
    },
    "access_profiles": {
      "type": "array",
      "items": {"type": "string"}
    },
    "restrictions": {
      "type": "object",
      "properties": {
        "time_zones": {"type": "array"},
        "locations": {"type": "array"},
        "ip_ranges": {"type": "array"}
      }
    }
  }
}
```

### 4.2 Credential Types

| Type | Technology | Security Level | Use Case |
|------|------------|---------------|----------|
| magnetic_stripe | Magnetic card | Low | Legacy systems |
| proximity | 125 kHz RFID | Low-Medium | Basic access |
| smart_card | 13.56 MHz contactless | High | PIV/CAC, secure access |
| mobile | BLE, NFC | Medium-High | Smartphone credentials |
| biometric | Fingerprint, face, iris | High | High-security areas |
| pin_code | Numeric/alphanumeric | Low-Medium | Secondary factor |
| otp | TOTP/HOTP | High | MFA |
| certificate | X.509 digital certificate | High | PKI authentication |

### 4.3 Credential Lifecycle

```
issued (active) → suspended → active (resumable)
issued (active) → revoked (permanent)
issued (active) → expired (time-based)
```

---

## 5. Role and Permission Models

### 5.1 Role Schema (RBAC)

```json
{
  "role_id": "role-engineering",
  "name": "Engineering Department",
  "description": "Access rights for engineering team",
  "role_type": "department",
  "parent_roles": ["role-employee"],
  "permissions": [
    {
      "resource_type": "door",
      "resource_id": "door-eng-lab",
      "actions": ["entry", "exit"],
      "conditions": {"time_range": "business_hours"}
    }
  ]
}
```

### 5.2 Permission Schema (ABAC)

```json
{
  "permission_id": "perm-server-room",
  "effect": "allow",
  "principals": {
    "user_attributes": {
      "department": ["IT", "Security"],
      "clearance_level": {"gte": 4}
    }
  },
  "resources": {
    "resource_type": "door",
    "resource_ids": ["door-server-room-a"]
  },
  "actions": ["entry", "exit"],
  "conditions": {
    "time": {"business_hours": true},
    "context": {"mfa_verified_within_minutes": 15}
  }
}
```

---

## 6. Audit Event Schema

```json
{
  "event_id": "evt-20251226-143217-abc123",
  "event_type": "access_granted",
  "category": "authentication",
  "severity": "info",
  "timestamp": "2025-12-26T14:32:17.234Z",
  "actor": {
    "user_id": "usr-001",
    "credential_id": "cred-xyz"
  },
  "target": {
    "resource_type": "door",
    "resource_id": "door-101"
  },
  "action": "entry_granted",
  "result": "success",
  "integrity": {
    "previous_event_hash": "sha256:abc...",
    "this_event_hash": "sha256:def...",
    "signature": "rsa-sha256:BASE64..."
  }
}
```

---

## 7. Data Validation

### 7.1 Validation Rules

All implementations MUST:
1. Validate all JSON against published schemas
2. Enforce required fields
3. Validate enum values
4. Check string patterns (regex)
5. Validate date/time formats (ISO 8601 / RFC 3339)
6. Enforce referential integrity (foreign keys)
7. Validate temporal constraints (expires_at > issued_at)

### 7.2 Error Handling

Invalid data MUST return descriptive error messages:
```json
{
  "error": {
    "code": "VALIDATION_FAILED",
    "message": "Data validation failed",
    "field": "expires_at",
    "details": "Field 'expires_at' must be after 'issued_at'"
  }
}
```

---

## 8. Import/Export Formats

### 8.1 CSV Format

Headers required. UTF-8 with BOM for Excel compatibility.
```csv
user_id,external_id,status,given_name,family_name,email,department,roles
usr-001,E-12345,active,John,Doe,john@example.com,Engineering,"role-employee|role-engineering"
```

### 8.2 NDJSON Format

One JSON object per line for streaming:
```
{"user_id":"usr-001","status":"active",...}
{"user_id":"usr-002","status":"active",...}
```

---

## 9. Internationalization

### 9.1 Character Encoding

All text fields support full Unicode (UTF-8).

### 9.2 Localized Fields

Display names may include localized versions:
```json
{
  "name": "Server Room",
  "name_i18n": {
    "en": "Server Room",
    "ko": "서버실",
    "ja": "サーバールーム"
  }
}
```

---

## 10. Compliance

Phase 1 Data Format compliance requires:
- [ ] All entities use defined JSON schemas
- [ ] Validation rules enforced
- [ ] Import/export functionality implemented
- [ ] UTF-8 encoding throughout
- [ ] Audit events include integrity fields

---

## Appendix A: Example Data

Complete examples available at: https://github.com/wia-official/wia-acs/tree/main/examples/phase1

---

**Document Control**
- Author: WIA Technical Committee
- Approved: 2025-12-26
- Next Review: 2026-12-26
- License: CC BY 4.0

© 2025 SmileStory Inc. / WIA
弘益人間 - Benefit All Humanity


## Annex E — Implementation Notes for PHASE-1-DATA-FORMAT

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-1-DATA-FORMAT.

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

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.
