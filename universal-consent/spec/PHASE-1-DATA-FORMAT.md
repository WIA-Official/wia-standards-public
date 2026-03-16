# WIA-CORE-002 PHASE 1: Data Format Specification

**Version:** 1.0  
**Status:** Stable  
**Last Updated:** January 2025

## Overview

Phase 1 defines the standardized data format for consent records in WIA-CORE-002. This specification establishes the foundation for interoperable, auditable, and compliant consent management across all systems and jurisdictions.

## Design Principles

1. **Completeness:** Capture all legally required information
2. **Interoperability:** JSON-based format for universal compatibility
3. **Extensibility:** Support custom fields without breaking compatibility
4. **Auditability:** Comprehensive audit trail for all consent operations
5. **Multi-jurisdiction:** Support multiple regulatory frameworks simultaneously

## Core Data Structure

### Consent Record Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "WIA-CORE-002 Consent Record",
  "type": "object",
  "required": [
    "consentId",
    "userId",
    "version",
    "timestamp",
    "status",
    "purposes",
    "legalBasis",
    "jurisdiction"
  ],
  "properties": {
    "consentId": {
      "type": "string",
      "format": "uuid",
      "description": "Globally unique consent identifier"
    },
    "userId": {
      "type": "string",
      "description": "User identifier (email, UUID, or internal ID)"
    },
    "version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+$",
      "description": "WIA-CORE-002 version (e.g., '1.0')"
    },
    "standard": {
      "type": "string",
      "const": "WIA-CORE-002",
      "description": "Standard identifier"
    },
    "timestamp": {
      "type": "string",
      "format": "date-time",
      "description": "ISO 8601 timestamp of consent grant"
    },
    "expiresAt": {
      "type": "string",
      "format": "date-time",
      "description": "ISO 8601 timestamp of consent expiration"
    },
    "status": {
      "type": "string",
      "enum": ["pending", "active", "revoked", "expired", "suspended", "superseded"],
      "description": "Current consent status"
    },
    "jurisdiction": {
      "type": "string",
      "description": "Applicable jurisdiction (ISO 3166-1 alpha-2 or special codes)"
    },
    "legalBasis": {
      "type": "string",
      "enum": ["consent", "contract", "legal-obligation", "legitimate-interest", "vital-interest", "public-interest"],
      "description": "Legal basis for processing"
    }
  }
}
```

## Purpose Specification

Each consent must specify one or more purposes:

```json
"purposes": [
  {
    "purposeId": "marketing-email",
    "purposeName": "Email Marketing Communications",
    "description": "Receive promotional emails about products and services",
    "granted": true,
    "categories": ["email", "marketing", "promotional"],
    "dataCategories": ["email-address", "name", "preferences"],
    "retentionPeriod": "P2Y",
    "thirdParties": ["marketing-platform-inc"]
  }
]
```

### Standard Purpose Categories

- `marketing` - Marketing communications
- `analytics` - Usage analytics and monitoring
- `personalization` - Content personalization
- `advertising` - Targeted advertising
- `social-media` - Social media integration
- `research` - Product research and development
- `security` - Security and fraud prevention
- `support` - Customer support
- `legal` - Legal compliance
- `contract` - Contract fulfillment

### Standard Data Categories

- `contact-info` - Email, phone, address
- `identification` - Name, date of birth, government ID
- `financial` - Credit card, bank account, transactions
- `health` - Medical records, health conditions
- `biometric` - Fingerprints, facial recognition, DNA
- `location` - GPS coordinates, IP address
- `behavioral` - Browsing history, purchase history
- `preferences` - User settings, favorites
- `communications` - Emails, messages, calls
- `employment` - Job title, employer, work history

## Metadata Structure

```json
"metadata": {
  "source": "web-signup-form",
  "sourceUrl": "https://example.com/signup",
  "ipAddress": "192.0.2.1",
  "userAgent": "Mozilla/5.0...",
  "consentFormVersion": "2.3",
  "privacyPolicyVersion": "4.1",
  "privacyPolicyUrl": "https://example.com/privacy/v4.1",
  "termsVersion": "3.2",
  "language": "en"
}
```

## Audit Trail

```json
"auditTrail": [
  {
    "timestamp": "2025-01-15T10:30:00Z",
    "action": "created",
    "actor": "system",
    "ipAddress": "192.0.2.1",
    "previousState": null,
    "newState": "pending",
    "changes": null,
    "reason": "User signup initiated"
  }
]
```

### Standard Actions

- `created` - Consent record created
- `verified` - Consent verified (e.g., email confirmation)
- `updated` - Consent preferences updated
- `revoked` - Consent withdrawn
- `expired` - Consent expired
- `suspended` - Consent temporarily suspended
- `superseded` - Replaced by newer consent

## User Information

For minors and special cases:

```json
"userInfo": {
  "age": 14,
  "isMinor": true,
  "parentalConsent": {
    "required": true,
    "obtained": true,
    "parentId": "parent-456789",
    "parentEmail": "parent@example.com",
    "verificationMethod": "credit-card-check",
    "verifiedAt": "2025-01-15T10:40:00Z"
  },
  "ageVerificationMethod": "self-reported",
  "language": "en",
  "region": "EU"
}
```

## Validation Rules

1. `consentId` must be a valid UUID v4
2. `timestamp` must be ISO 8601 format and not in the future
3. `expiresAt` (if present) must be after `timestamp`
4. `status` must be one of the defined enum values
5. `purposes` array must contain at least one purpose
6. Each purpose must have `purposeId`, `purposeName`, `description`, `granted`, and `dataCategories`
7. `jurisdiction` must be a valid ISO 3166-1 alpha-2 code or special code
8. For minors, parental consent information must be present if required by jurisdiction
9. `auditTrail` must have at least one entry (creation event)
10. All timestamps must use UTC timezone

## Extensibility

Organizations can add custom fields in a `custom` namespace:

```json
"custom": {
  "organizationId": "org-123",
  "department": "marketing",
  "campaignId": "summer-2025",
  "internalReferenceId": "ref-xyz-789"
}
```

## Multi-Language Support

Purpose names and descriptions can be provided in multiple languages:

```json
"purposeName": {
  "en": "Email Marketing Communications",
  "es": "Comunicaciones de Marketing por Correo Electrónico",
  "de": "E-Mail-Marketing-Kommunikation",
  "fr": "Communications Marketing par E-mail",
  "ko": "이메일 마케팅 커뮤니케이션"
}
```

## Retention Periods

Use ISO 8601 duration format:

- `P1Y` - 1 year
- `P2Y` - 2 years
- `P6M` - 6 months
- `P90D` - 90 days
- `P7Y` - 7 years
- `INDEFINITE` - No expiration

## Compliance Notes

### GDPR Compliance

- Article 7: Conditions for consent → `purposes[].granted`, `auditTrail`
- Article 13: Information to be provided → `metadata.privacyPolicyUrl`
- Article 30: Records of processing activities → Complete consent record

### CCPA Compliance

- Right to Opt-Out → `purposes[].granted = false`
- Right to Know → Complete consent record with audit trail
- Right to Delete → Status change to `revoked` with deletion timestamp

### PIPEDA Compliance

- Meaningful consent → `metadata` with full context
- Purpose specification → `purposes` array with clear descriptions
- Retention limits → `retentionPeriod` for each purpose

## Version History

- **1.0** (January 2025) - Initial stable release
  - Core data structure defined
  - Purpose and metadata schemas
  - Audit trail specification
  - Multi-language support

---

**Next:** [PHASE 2: API Interface](PHASE-2-API-INTERFACE.md)

© 2025 SmileStory Inc. / WIA · 弘益人間 (Benefit All Humanity)
