# WIA-UNI-002: Phase 1 - Data Format Specification

**Version:** 1.0
**Status:** Complete
**Last Updated:** 2025-01-15

---

## 1. Overview

This document specifies the data format for the WIA-UNI-002 Unified ID System. It defines the JSON schema, field mappings, validation rules, and serialization formats required for interoperable identity records across the unified Korean territory.

## 2. Core Data Schema

### 2.1 Unified ID Record

```json
{
  "$schema": "https://wiastandards.com/schemas/uni-002/v1.0/unified-id.json",
  "version": "1.0",
  "standard": "WIA-UNI-002",
  "unifiedId": "UNI-KR-900515-1234",
  "personalInfo": {
    "familyName": "Kim",
    "givenName": "MinJun",
    "fullName": "Kim MinJun",
    "birthDate": "1990-05-15",
    "gender": "M"
  },
  "origin": {
    "region": "south",
    "previousId": "900515-1234567",
    "registrationDate": "2025-01-15T09:00:00Z"
  },
  "biometric": {
    "fingerprintHash": "sha256:a1b2c3d4e5f6...",
    "facialHash": "sha256:g7h8i9j0k1l2...",
    "irisHash": null
  },
  "issuedAt": "2025-01-15T09:30:00Z",
  "expiresAt": "2035-01-15T00:00:00Z",
  "issuer": "did:wia:unified-korea-authority",
  "status": "active",
  "privacyLevel": "protected"
}
```

## 3. Field Definitions

### 3.1 Required Fields

| Field | Type | Format | Description |
|-------|------|--------|-------------|
| `version` | string | `x.y` | Schema version |
| `standard` | string | `WIA-UNI-002` | Standard identifier |
| `unifiedId` | string | `UNI-KR-YYMMDD-NNNN` | Unique unified ID |
| `personalInfo` | object | - | Personal information |
| `personalInfo.familyName` | string | UTF-8 | Family name (surname) |
| `personalInfo.givenName` | string | UTF-8 | Given name |
| `personalInfo.birthDate` | string | ISO 8601 date | Birth date |
| `biometric` | object | - | Biometric hashes |
| `issuedAt` | string | ISO 8601 datetime | Issuance timestamp |
| `issuer` | string | DID | Issuing authority DID |
| `status` | string | enum | Credential status |

### 3.2 Optional Fields

| Field | Type | Format | Description |
|-------|------|--------|-------------|
| `personalInfo.gender` | string | M/F/X | Gender |
| `origin.region` | string | enum | Origin region (encrypted) |
| `origin.previousId` | string | - | Legacy ID (encrypted) |
| `biometric.irisHash` | string | hex | Iris biometric hash |
| `expiresAt` | string | ISO 8601 | Expiration date |

## 4. Unified ID Format

### 4.1 Structure

```
UNI-KR-YYMMDD-NNNN
```

- **UNI**: Universal prefix
- **KR**: Country code (Korea)
- **YYMMDD**: Birth date (last 2 digits of year, month, day)
- **NNNN**: Random 4-digit serial (NOT sequential)

### 4.2 Generation Algorithm

```python
def generate_unified_id(birth_date):
    import random
    yy = str(birth_date.year)[-2:]
    mm = f"{birth_date.month:02d}"
    dd = f"{birth_date.day:02d}"
    serial = f"{random.randint(1000, 9999)}"
    return f"UNI-KR-{yy}{mm}{dd}-{serial}"
```

## 5. Biometric Hashing

### 5.1 Fingerprint Processing

1. **Capture**: High-resolution scan (500+ DPI)
2. **Feature Extraction**: Extract minutiae points
3. **Template Generation**: ISO/IEC 19794-2 format
4. **Hashing**: SHA-256 hash of template

```python
biometric.fingerprintHash = "sha256:" + sha256(fingerprint_template).hexdigest()
```

### 5.2 Facial Recognition

1. **Capture**: 2D color photo + optional 3D scan
2. **Embedding**: Deep learning model (128D vector)
3. **Hashing**: Keyed hash with secret key

## 6. Validation Rules

### 6.1 Required Field Validation

- All required fields must be present
- `unifiedId` must match regex: `^UNI-KR-\d{6}-\d{4}$`
- `birthDate` must be valid ISO 8601 date in the past
- At least one biometric hash must be present

### 6.2 Data Type Validation

- Strings must be UTF-8 encoded
- Dates must be ISO 8601 format
- Enums must match allowed values
- DIDs must conform to W3C DID specification

## 7. Encryption for Sensitive Fields

### 7.1 Encrypted Personal Info

```json
{
  "personalInfo": {
    "encrypted": true,
    "algorithm": "AES-256-GCM",
    "ciphertext": "base64-encoded-ciphertext",
    "iv": "base64-encoded-iv",
    "keyId": "did:wia:unified-korea-authority#key-1"
  }
}
```

### 7.2 Key Management

- Encryption keys stored in Hardware Security Module (HSM)
- Key rotation every 90 days
- Decryption keys accessible only to authorized entities

## 8. Interoperability Formats

### 8.1 Supported Serializations

| Format | Use Case | Features |
|--------|----------|----------|
| JSON | APIs, web apps | Human-readable |
| JSON-LD | Verifiable Credentials | Linked data |
| Protocol Buffers | High-performance | Compact, fast |
| CBOR | IoT, offline | Binary, minimal overhead |
| QR Code | Physical verification | Camera-readable |

## 9. Versioning

### 9.1 Semantic Versioning

- **Major version**: Breaking changes (1.0 → 2.0)
- **Minor version**: Backward-compatible additions (1.0 → 1.1)
- **Patch version**: Bug fixes (1.0.0 → 1.0.1)

### 9.2 Compatibility

- Implementations MUST support current version
- Implementations SHOULD support one previous major version

## 10. Test Vectors

### 10.1 Valid Record Example

See Section 2.1 for a complete valid example.

### 10.2 Invalid Record Examples

**Missing required field:**
```json
{
  "version": "1.0",
  "standard": "WIA-UNI-002"
  // ERROR: Missing unifiedId, personalInfo, etc.
}
```

**Invalid ID format:**
```json
{
  "unifiedId": "INVALID-ID"
  // ERROR: Does not match UNI-KR-YYMMDD-NNNN pattern
}
```

---

## Appendix A: JSON Schema

Full JSON Schema available at:
https://wiastandards.com/schemas/uni-002/v1.0/unified-id.json

## Appendix B: Migration Tools

Legacy data migration tools available at:
https://github.com/WIA-Official/uni-002-migration-tools

---

**弘益人間 (Benefit All Humanity)**

*WIA - World Certification Industry Association*
*© 2025 MIT License*
