# WIA Cryo-Legal Standard - Phase 1: Data Format Specification

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #06B6D4 (Cyan)

---

## 1. Overview

### 1.1 Purpose

The WIA Cryo-Legal Standard defines the data format for managing legal frameworks, documentation, and compliance requirements related to cryopreservation procedures. This specification ensures standardized handling of legal documents, consent records, jurisdiction mapping, and regulatory compliance across international cryopreservation facilities.

### 1.2 Scope

This standard covers:

| Category | Description |
|----------|-------------|
| Legal Documents | Contracts, wills, advance directives |
| Jurisdiction Mapping | Country-specific legal requirements |
| Compliance Records | Regulatory audit trails |
| Consent Verification | Legal consent chain validation |
| Trust Management | Cryopreservation trust structures |

### 1.3 Design Principles

1. **Legal Integrity**: Cryptographic verification of all documents
2. **Jurisdictional Awareness**: Multi-jurisdiction support
3. **Temporal Validity**: Long-term document validity management
4. **Auditability**: Complete legal audit trails
5. **Privacy Compliance**: GDPR, HIPAA, and international privacy law support

---

## 2. Data Schema

### 2.1 CryoLegalDocument Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wia.live/cryo-legal/v1/document.schema.json",
  "title": "WIA Cryo-Legal Document",
  "type": "object",
  "required": [
    "documentId",
    "version",
    "documentType",
    "jurisdiction",
    "createdAt",
    "parties",
    "content",
    "signatures"
  ],
  "properties": {
    "$schema": {
      "type": "string",
      "format": "uri"
    },
    "documentId": {
      "type": "string",
      "format": "uuid",
      "description": "Unique document identifier"
    },
    "version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+\\.\\d+$"
    },
    "documentType": {
      "type": "string",
      "enum": [
        "cryopreservation_contract",
        "advance_directive",
        "last_will",
        "trust_document",
        "consent_form",
        "power_of_attorney",
        "medical_directive",
        "revival_instruction",
        "asset_disposition",
        "identity_verification"
      ]
    },
    "jurisdiction": {
      "type": "object",
      "required": ["primaryCountry", "governingLaw"],
      "properties": {
        "primaryCountry": {
          "type": "string",
          "pattern": "^[A-Z]{2}$"
        },
        "secondaryCountries": {
          "type": "array",
          "items": { "type": "string", "pattern": "^[A-Z]{2}$" }
        },
        "governingLaw": {
          "type": "string"
        },
        "disputeResolution": {
          "type": "string",
          "enum": ["arbitration", "litigation", "mediation"]
        },
        "venue": {
          "type": "string"
        }
      }
    },
    "createdAt": {
      "type": "string",
      "format": "date-time"
    },
    "effectiveDate": {
      "type": "string",
      "format": "date-time"
    },
    "expirationDate": {
      "type": "string",
      "format": "date-time"
    },
    "parties": {
      "type": "array",
      "minItems": 1,
      "items": {
        "$ref": "#/definitions/Party"
      }
    },
    "content": {
      "$ref": "#/definitions/DocumentContent"
    },
    "signatures": {
      "type": "array",
      "items": {
        "$ref": "#/definitions/Signature"
      }
    },
    "witnesses": {
      "type": "array",
      "items": {
        "$ref": "#/definitions/Witness"
      }
    },
    "notarization": {
      "$ref": "#/definitions/Notarization"
    },
    "attachments": {
      "type": "array",
      "items": {
        "$ref": "#/definitions/Attachment"
      }
    },
    "auditLog": {
      "type": "array",
      "items": {
        "$ref": "#/definitions/AuditEntry"
      }
    },
    "metadata": {
      "type": "object",
      "properties": {
        "language": { "type": "string" },
        "classification": { "type": "string" },
        "retentionPeriod": { "type": "string" },
        "tags": { "type": "array", "items": { "type": "string" } }
      }
    }
  },
  "definitions": {
    "Party": {
      "type": "object",
      "required": ["partyId", "role", "identity"],
      "properties": {
        "partyId": { "type": "string", "format": "uuid" },
        "role": {
          "type": "string",
          "enum": ["subject", "facility", "trustee", "beneficiary", "executor", "witness", "notary", "legal_representative"]
        },
        "identity": {
          "type": "object",
          "properties": {
            "type": { "type": "string", "enum": ["individual", "organization"] },
            "legalName": { "type": "string" },
            "identificationNumber": { "type": "string" },
            "identificationType": { "type": "string" },
            "dateOfBirth": { "type": "string", "format": "date" },
            "nationality": { "type": "string" },
            "address": { "$ref": "#/definitions/Address" }
          }
        },
        "contact": {
          "type": "object",
          "properties": {
            "email": { "type": "string", "format": "email" },
            "phone": { "type": "string" },
            "preferredMethod": { "type": "string" }
          }
        }
      }
    },
    "Address": {
      "type": "object",
      "properties": {
        "street": { "type": "string" },
        "city": { "type": "string" },
        "state": { "type": "string" },
        "postalCode": { "type": "string" },
        "country": { "type": "string", "pattern": "^[A-Z]{2}$" }
      }
    },
    "DocumentContent": {
      "type": "object",
      "required": ["format", "body"],
      "properties": {
        "format": { "type": "string", "enum": ["plaintext", "markdown", "html", "pdf_base64"] },
        "body": { "type": "string" },
        "sections": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "sectionId": { "type": "string" },
              "title": { "type": "string" },
              "content": { "type": "string" },
              "required": { "type": "boolean" }
            }
          }
        },
        "hash": { "type": "string" },
        "hashAlgorithm": { "type": "string", "enum": ["SHA-256", "SHA-384", "SHA-512"] }
      }
    },
    "Signature": {
      "type": "object",
      "required": ["signerId", "timestamp", "signatureData"],
      "properties": {
        "signerId": { "type": "string", "format": "uuid" },
        "timestamp": { "type": "string", "format": "date-time" },
        "signatureType": {
          "type": "string",
          "enum": ["electronic", "digital", "handwritten_scanned", "biometric"]
        },
        "signatureData": { "type": "string" },
        "certificate": { "type": "string" },
        "ipAddress": { "type": "string" },
        "deviceInfo": { "type": "string" },
        "verificationStatus": {
          "type": "string",
          "enum": ["pending", "verified", "failed", "revoked"]
        }
      }
    },
    "Witness": {
      "type": "object",
      "required": ["witnessId", "identity", "timestamp"],
      "properties": {
        "witnessId": { "type": "string", "format": "uuid" },
        "identity": { "$ref": "#/definitions/Party/properties/identity" },
        "timestamp": { "type": "string", "format": "date-time" },
        "attestation": { "type": "string" },
        "signature": { "$ref": "#/definitions/Signature" }
      }
    },
    "Notarization": {
      "type": "object",
      "properties": {
        "notaryId": { "type": "string" },
        "notaryName": { "type": "string" },
        "commission": { "type": "string" },
        "jurisdiction": { "type": "string" },
        "timestamp": { "type": "string", "format": "date-time" },
        "seal": { "type": "string" },
        "certificate": { "type": "string" }
      }
    },
    "Attachment": {
      "type": "object",
      "required": ["attachmentId", "filename", "mimeType"],
      "properties": {
        "attachmentId": { "type": "string", "format": "uuid" },
        "filename": { "type": "string" },
        "mimeType": { "type": "string" },
        "size": { "type": "integer" },
        "hash": { "type": "string" },
        "url": { "type": "string", "format": "uri" },
        "description": { "type": "string" }
      }
    },
    "AuditEntry": {
      "type": "object",
      "required": ["timestamp", "action", "actorId"],
      "properties": {
        "timestamp": { "type": "string", "format": "date-time" },
        "action": {
          "type": "string",
          "enum": ["created", "viewed", "modified", "signed", "notarized", "revoked", "archived"]
        },
        "actorId": { "type": "string" },
        "details": { "type": "string" },
        "ipAddress": { "type": "string" }
      }
    }
  }
}
```

---

## 3. Field Specifications

### 3.1 Core Fields

| Field | Type | Required | Description | Example |
|-------|------|----------|-------------|---------|
| `documentId` | string (UUID) | Yes | Unique identifier | `"550e8400-e29b-41d4-a716-446655440000"` |
| `version` | string | Yes | Semantic version | `"1.0.0"` |
| `documentType` | enum | Yes | Type of legal document | `"cryopreservation_contract"` |
| `jurisdiction.primaryCountry` | string | Yes | ISO 3166-1 alpha-2 code | `"US"` |
| `jurisdiction.governingLaw` | string | Yes | Applicable law | `"State of Arizona"` |
| `createdAt` | datetime | Yes | Creation timestamp | `"2025-01-15T10:30:00Z"` |
| `effectiveDate` | datetime | No | When document takes effect | `"2025-02-01T00:00:00Z"` |
| `expirationDate` | datetime | No | Document expiration | `null` (perpetual) |

### 3.2 Party Fields

| Field | Type | Required | Description | Example |
|-------|------|----------|-------------|---------|
| `parties[].partyId` | string (UUID) | Yes | Party identifier | `"a1b2c3d4-..."` |
| `parties[].role` | enum | Yes | Role in document | `"subject"` |
| `parties[].identity.type` | enum | Yes | Individual or organization | `"individual"` |
| `parties[].identity.legalName` | string | Yes | Full legal name | `"John Michael Smith"` |
| `parties[].identity.identificationNumber` | string | No | ID number | `"123-45-6789"` |
| `parties[].identity.dateOfBirth` | date | No | Birth date | `"1980-05-15"` |
| `parties[].identity.nationality` | string | No | Country of citizenship | `"US"` |

### 3.3 Signature Fields

| Field | Type | Required | Description | Example |
|-------|------|----------|-------------|---------|
| `signatures[].signerId` | string (UUID) | Yes | Signer's party ID | `"a1b2c3d4-..."` |
| `signatures[].timestamp` | datetime | Yes | Signing time | `"2025-01-15T14:30:00Z"` |
| `signatures[].signatureType` | enum | Yes | Type of signature | `"digital"` |
| `signatures[].signatureData` | string | Yes | Base64 encoded signature | `"MEUCIQDx..."` |
| `signatures[].certificate` | string | No | X.509 certificate | `"-----BEGIN CERTIFICATE-----..."` |
| `signatures[].verificationStatus` | enum | No | Verification state | `"verified"` |

---

## 4. Data Types

### 4.1 Document Types

| Value | Description |
|-------|-------------|
| `cryopreservation_contract` | Main service agreement with facility |
| `advance_directive` | Medical care preferences |
| `last_will` | Testament with cryopreservation provisions |
| `trust_document` | Financial trust for cryopreservation |
| `consent_form` | Informed consent documentation |
| `power_of_attorney` | Legal representative authorization |
| `medical_directive` | Healthcare proxy instructions |
| `revival_instruction` | Post-revival care preferences |
| `asset_disposition` | Asset management instructions |
| `identity_verification` | Identity proof documentation |

### 4.2 Party Roles

| Value | Description |
|-------|-------------|
| `subject` | Person to be cryopreserved |
| `facility` | Cryopreservation service provider |
| `trustee` | Trust administrator |
| `beneficiary` | Trust beneficiary |
| `executor` | Will executor |
| `witness` | Document witness |
| `notary` | Notary public |
| `legal_representative` | Attorney or legal agent |

### 4.3 Verification Status

| Value | Description |
|-------|-------------|
| `pending` | Awaiting verification |
| `verified` | Successfully verified |
| `failed` | Verification failed |
| `revoked` | Previously valid, now revoked |

---

## 5. Validation Rules

### 5.1 Document Validation

| Rule ID | Field | Validation | Error Code |
|---------|-------|------------|------------|
| VAL-001 | `documentId` | Must be valid UUID v4 | `INVALID_DOCUMENT_ID` |
| VAL-002 | `version` | Must match semantic versioning | `INVALID_VERSION` |
| VAL-003 | `jurisdiction.primaryCountry` | Must be valid ISO 3166-1 alpha-2 | `INVALID_COUNTRY_CODE` |
| VAL-004 | `createdAt` | Must be valid ISO 8601 datetime | `INVALID_DATETIME` |
| VAL-005 | `effectiveDate` | Must be >= createdAt if present | `INVALID_EFFECTIVE_DATE` |
| VAL-006 | `parties` | Must have at least one party | `NO_PARTIES` |
| VAL-007 | `signatures` | All required signers must sign | `MISSING_SIGNATURES` |

### 5.2 Signature Validation

| Rule ID | Field | Validation | Error Code |
|---------|-------|------------|------------|
| SIG-001 | `signerId` | Must reference valid party | `INVALID_SIGNER` |
| SIG-002 | `timestamp` | Must be <= current time | `FUTURE_SIGNATURE` |
| SIG-003 | `signatureData` | Must be valid Base64 | `INVALID_SIGNATURE_DATA` |
| SIG-004 | `certificate` | Must be valid X.509 if digital | `INVALID_CERTIFICATE` |

### 5.3 Error Codes

| Code | Message | Description |
|------|---------|-------------|
| `INVALID_DOCUMENT_ID` | Document ID format invalid | UUID format required |
| `INVALID_VERSION` | Version format invalid | Use MAJOR.MINOR.PATCH |
| `INVALID_COUNTRY_CODE` | Country code not recognized | Use ISO 3166-1 alpha-2 |
| `INVALID_DATETIME` | DateTime format invalid | Use ISO 8601 format |
| `INVALID_EFFECTIVE_DATE` | Effective date before creation | Effective must be >= created |
| `NO_PARTIES` | No parties defined | At least one party required |
| `MISSING_SIGNATURES` | Required signatures missing | All parties must sign |
| `INVALID_SIGNER` | Signer not in parties list | Signer must be a party |
| `FUTURE_SIGNATURE` | Signature timestamp in future | Cannot sign in future |
| `INVALID_SIGNATURE_DATA` | Signature data corrupted | Valid Base64 required |
| `INVALID_CERTIFICATE` | Certificate validation failed | Valid X.509 required |

---

## 6. Examples

### 6.1 Valid Cryopreservation Contract

```json
{
  "$schema": "https://wia.live/cryo-legal/v1/document.schema.json",
  "documentId": "550e8400-e29b-41d4-a716-446655440000",
  "version": "1.0.0",
  "documentType": "cryopreservation_contract",
  "jurisdiction": {
    "primaryCountry": "US",
    "governingLaw": "State of Arizona",
    "disputeResolution": "arbitration",
    "venue": "Phoenix, Arizona"
  },
  "createdAt": "2025-01-15T10:30:00Z",
  "effectiveDate": "2025-02-01T00:00:00Z",
  "parties": [
    {
      "partyId": "a1b2c3d4-e5f6-7890-abcd-ef1234567890",
      "role": "subject",
      "identity": {
        "type": "individual",
        "legalName": "John Michael Smith",
        "identificationNumber": "DL-AZ-123456",
        "identificationType": "driver_license",
        "dateOfBirth": "1980-05-15",
        "nationality": "US",
        "address": {
          "street": "123 Main Street",
          "city": "Scottsdale",
          "state": "AZ",
          "postalCode": "85251",
          "country": "US"
        }
      },
      "contact": {
        "email": "john.smith@email.com",
        "phone": "+1-480-555-0123",
        "preferredMethod": "email"
      }
    },
    {
      "partyId": "b2c3d4e5-f6a7-8901-bcde-f23456789012",
      "role": "facility",
      "identity": {
        "type": "organization",
        "legalName": "CryoLife Preservation Inc.",
        "identificationNumber": "EIN-12-3456789",
        "identificationType": "ein",
        "address": {
          "street": "456 Research Parkway",
          "city": "Scottsdale",
          "state": "AZ",
          "postalCode": "85260",
          "country": "US"
        }
      }
    }
  ],
  "content": {
    "format": "markdown",
    "body": "# Cryopreservation Services Agreement\n\n## Article 1: Services\n\nThe Facility agrees to provide whole-body cryopreservation services...",
    "sections": [
      {
        "sectionId": "art-1",
        "title": "Services",
        "content": "The Facility agrees to provide whole-body cryopreservation services to the Subject upon legal death...",
        "required": true
      },
      {
        "sectionId": "art-2",
        "title": "Payment Terms",
        "content": "Subject agrees to fund cryopreservation through life insurance policy assignment...",
        "required": true
      }
    ],
    "hash": "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855",
    "hashAlgorithm": "SHA-256"
  },
  "signatures": [
    {
      "signerId": "a1b2c3d4-e5f6-7890-abcd-ef1234567890",
      "timestamp": "2025-01-15T14:30:00Z",
      "signatureType": "digital",
      "signatureData": "MEUCIQDxN2qc8Kj2c5y...",
      "certificate": "-----BEGIN CERTIFICATE-----\nMIIDXTCCAkWgAwIBAgIJAJC1...",
      "verificationStatus": "verified"
    },
    {
      "signerId": "b2c3d4e5-f6a7-8901-bcde-f23456789012",
      "timestamp": "2025-01-15T15:00:00Z",
      "signatureType": "digital",
      "signatureData": "MEQCIGhKl4R2m8f...",
      "certificate": "-----BEGIN CERTIFICATE-----\nMIIDYTCCAkmgAwIBAgIJAKD2...",
      "verificationStatus": "verified"
    }
  ],
  "witnesses": [
    {
      "witnessId": "c3d4e5f6-a7b8-9012-cdef-345678901234",
      "identity": {
        "type": "individual",
        "legalName": "Mary Jane Wilson"
      },
      "timestamp": "2025-01-15T14:35:00Z",
      "attestation": "I witnessed the Subject sign this document voluntarily."
    }
  ],
  "notarization": {
    "notaryId": "NP-AZ-12345",
    "notaryName": "Sarah Johnson",
    "commission": "Arizona Notary #12345",
    "jurisdiction": "US-AZ",
    "timestamp": "2025-01-15T16:00:00Z",
    "seal": "base64-encoded-seal-image"
  },
  "auditLog": [
    {
      "timestamp": "2025-01-15T10:30:00Z",
      "action": "created",
      "actorId": "system",
      "details": "Document created from template v2.1"
    },
    {
      "timestamp": "2025-01-15T14:30:00Z",
      "action": "signed",
      "actorId": "a1b2c3d4-e5f6-7890-abcd-ef1234567890",
      "details": "Subject signed digitally"
    }
  ],
  "metadata": {
    "language": "en-US",
    "classification": "confidential",
    "retentionPeriod": "perpetual",
    "tags": ["cryopreservation", "whole-body", "arizona"]
  }
}
```

### 6.2 Valid Trust Document

```json
{
  "$schema": "https://wia.live/cryo-legal/v1/document.schema.json",
  "documentId": "660f9511-f30c-52e5-b827-557766551111",
  "version": "1.0.0",
  "documentType": "trust_document",
  "jurisdiction": {
    "primaryCountry": "US",
    "governingLaw": "State of Delaware",
    "disputeResolution": "litigation",
    "venue": "Wilmington, Delaware"
  },
  "createdAt": "2025-01-20T09:00:00Z",
  "parties": [
    {
      "partyId": "d4e5f6a7-b8c9-0123-def4-567890123456",
      "role": "subject",
      "identity": {
        "type": "individual",
        "legalName": "Robert James Miller",
        "nationality": "US"
      }
    },
    {
      "partyId": "e5f6a7b8-c9d0-1234-ef56-789012345678",
      "role": "trustee",
      "identity": {
        "type": "organization",
        "legalName": "Delaware Trust Company"
      }
    }
  ],
  "content": {
    "format": "markdown",
    "body": "# Cryopreservation Maintenance Trust\n\nThis trust is established for the purpose of funding long-term cryopreservation...",
    "hash": "a1b2c3d4e5f6789012345678901234567890abcdef1234567890abcdef123456",
    "hashAlgorithm": "SHA-256"
  },
  "signatures": [
    {
      "signerId": "d4e5f6a7-b8c9-0123-def4-567890123456",
      "timestamp": "2025-01-20T10:00:00Z",
      "signatureType": "digital",
      "signatureData": "MEUCIQCabc123...",
      "verificationStatus": "verified"
    }
  ],
  "metadata": {
    "language": "en-US",
    "classification": "confidential",
    "retentionPeriod": "perpetual",
    "tags": ["trust", "maintenance-fund", "delaware"]
  }
}
```

### 6.3 Valid Advance Directive

```json
{
  "$schema": "https://wia.live/cryo-legal/v1/document.schema.json",
  "documentId": "771a0622-a41d-63f6-c938-668877662222",
  "version": "1.0.0",
  "documentType": "advance_directive",
  "jurisdiction": {
    "primaryCountry": "US",
    "secondaryCountries": ["CA", "GB"],
    "governingLaw": "State of California"
  },
  "createdAt": "2025-01-25T11:00:00Z",
  "effectiveDate": "2025-01-25T11:00:00Z",
  "parties": [
    {
      "partyId": "f6a7b8c9-d0e1-2345-fa67-890123456789",
      "role": "subject",
      "identity": {
        "type": "individual",
        "legalName": "Emily Chen Wang"
      }
    }
  ],
  "content": {
    "format": "markdown",
    "body": "# Advance Healthcare Directive for Cryopreservation\n\nI direct that upon determination of legal death, cryopreservation procedures begin immediately...",
    "hash": "b2c3d4e5f6a789012345678901234567890abcdef1234567890abcdef1234567",
    "hashAlgorithm": "SHA-256"
  },
  "signatures": [
    {
      "signerId": "f6a7b8c9-d0e1-2345-fa67-890123456789",
      "timestamp": "2025-01-25T11:30:00Z",
      "signatureType": "electronic",
      "signatureData": "SGVsbG8gV29ybGQ=",
      "verificationStatus": "verified"
    }
  ],
  "metadata": {
    "language": "en-US",
    "classification": "confidential",
    "retentionPeriod": "perpetual"
  }
}
```

### 6.4 Invalid Example: Missing Required Fields

```json
{
  "documentId": "invalid-uuid-format",
  "version": "1.0",
  "documentType": "contract",
  "jurisdiction": {
    "primaryCountry": "USA"
  },
  "parties": []
}
```

**Validation Errors:**
- `INVALID_DOCUMENT_ID`: documentId must be UUID v4
- `INVALID_VERSION`: version must be MAJOR.MINOR.PATCH
- `INVALID_DOCUMENT_TYPE`: "contract" not in enum
- `INVALID_COUNTRY_CODE`: "USA" should be "US"
- `NO_PARTIES`: parties array is empty
- `MISSING_REQUIRED_FIELD`: createdAt is required

### 6.5 Invalid Example: Future Signature

```json
{
  "documentId": "882b1733-b52e-74a7-da49-779988773333",
  "version": "1.0.0",
  "documentType": "consent_form",
  "jurisdiction": {
    "primaryCountry": "US",
    "governingLaw": "Federal"
  },
  "createdAt": "2025-01-01T00:00:00Z",
  "parties": [
    {
      "partyId": "a7b8c9d0-e1f2-3456-ab78-901234567890",
      "role": "subject",
      "identity": {
        "type": "individual",
        "legalName": "Test User"
      }
    }
  ],
  "content": {
    "format": "plaintext",
    "body": "Test content"
  },
  "signatures": [
    {
      "signerId": "a7b8c9d0-e1f2-3456-ab78-901234567890",
      "timestamp": "2099-12-31T23:59:59Z",
      "signatureType": "electronic",
      "signatureData": "dGVzdA=="
    }
  ]
}
```

**Validation Errors:**
- `FUTURE_SIGNATURE`: signature timestamp is in the future

---

## 7. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01 | Initial release |

---

<div align="center">

**WIA Cryo-Legal Standard v1.0.0**

Phase 1: Data Format Specification

**弘益人間 (홍익인간)** · Benefit All Humanity

---

© 2025 WIA Standards Committee

MIT License

</div>
