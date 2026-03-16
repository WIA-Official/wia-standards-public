# WIA-SOC-015: Phase 1 - Ballot Data Format Specification

**Version:** 1.0  
**Status:** FINAL  
**Last Updated:** 2025-01-15  
**Standards Body:** World Certification Industry Association (WIA)

---

## 1. Overview

Phase 1 of WIA-SOC-015 establishes standardized data formats for ballots, candidates, contests, and election results. This specification enables interoperability across different voting systems by defining common JSON schemas, validation rules, and encoding standards.

### 1.1 Purpose

- Enable data exchange between different voting system vendors
- Provide consistent ballot representation across jurisdictions
- Support multilingual and accessible ballot formats
- Establish foundation for Phases 2-4 functionality

### 1.2 Scope

This specification covers:
- Ballot data structures
- Candidate and contest definitions
- Election metadata
- Result data formats
- Multilingual support
- Accessibility annotations
- Security metadata

---

## 2. Core Data Structures

### 2.1 Ballot Definition

A ballot is represented as a JSON object conforming to the following schema:

```json
{
  "$schema": "https://wia-standards.org/schemas/voting/ballot-v1.0.json",
  "ballotId": "string (required, unique identifier)",
  "version": "string (required, WIA-SOC-015 version)",
  "electionId": "string (required, election identifier)",
  "electionName": "object (required, i18n)",
  "jurisdiction": "object (required)",
  "metadata": "object (required)",
  "contests": "array (required)",
  "accessibility": "object (optional)",
  "security": "object (required)"
}
```

### 2.2 Contest Definition

Each contest on a ballot is defined as:

```json
{
  "contestId": "string (required)",
  "contestType": "enum: candidate|referendum|recall",
  "contestName": "object (required, i18n)",
  "votingMethod": "enum: single-choice|multiple-choice|ranked-choice",
  "maxSelections": "number (required)",
  "minSelections": "number (optional, default: 0)",
  "writeInAllowed": "boolean (optional, default: false)",
  "candidates": "array (required for candidate type)",
  "ballotStyle": "object (optional)"
}
```

### 2.3 Candidate Definition

Candidates are represented as:

```json
{
  "candidateId": "string (required, unique)",
  "name": "string (required)",
  "party": "string (optional)",
  "photoUrl": "string (optional, URL)",
  "biography": "object (optional, i18n)",
  "contactInfo": "object (optional)"
}
```

---

## 3. Field Specifications

### 3.1 Required Fields

| Field | Type | Description | Constraints |
|-------|------|-------------|-------------|
| ballotId | string | Unique ballot identifier | Max 255 chars, alphanumeric+dash |
| version | string | WIA standard version | Format: WIA-SOC-015-v{major}.{minor} |
| electionId | string | Election identifier | Max 255 chars, unique within jurisdiction |
| electionName | object | Multilingual election name | Must include 'en' key minimum |
| contests | array | List of contests | Min 1 contest required |

### 3.2 Multilingual Support (i18n)

All user-facing text must be provided as internationalization objects:

```json
{
  "en": "Presidential Election 2025",
  "ko": "2025년 대통령 선거",
  "es": "Elección Presidencial 2025",
  "fr": "Élection présidentielle 2025"
}
```

**Requirements:**
- Must include 'en' (English) as baseline
- Should include languages spoken by >5% of jurisdiction
- Platinum certification requires all 99 WIA-supported languages
- Use ISO 639-1 two-letter language codes

---

## 4. Validation Rules

### 4.1 Schema Validation

All ballot data must pass JSON Schema validation before acceptance.

**Schema URL:** `https://wia-standards.org/schemas/voting/ballot-v1.0.json`

### 4.2 Business Logic Validation

| Rule ID | Rule Description | Error Code |
|---------|------------------|------------|
| VAL-001 | Election dates must be in future at creation | FUTURE_DATE_REQUIRED |
| VAL-002 | maxSelections >= minSelections | INVALID_SELECTION_RANGE |
| VAL-003 | No duplicate candidateId within contest | DUPLICATE_CANDIDATE |
| VAL-004 | All i18n objects must include 'en' key | MISSING_ENGLISH |
| VAL-005 | Digital signatures must be present and valid | INVALID_SIGNATURE |
| VAL-006 | Contest array cannot be empty | NO_CONTESTS |
| VAL-007 | Jurisdiction country code must be ISO 3166-1 | INVALID_COUNTRY_CODE |

---

## 5. Accessibility Annotations

### 5.1 Screen Reader Support

```json
{
  "accessibility": {
    "screenReader": {
      "ballotDescription": {
        "en": "Presidential Election Ballot containing 3 contests",
        "ko": "3개의 경선을 포함한 대통령 선거 투표용지"
      },
      "navigationHints": {
        "en": "Use arrow keys to navigate between contests"
      }
    },
    "audioFormat": {
      "available": true,
      "audioUrl": "https://cdn.example.com/ballot-audio-en.mp3"
    },
    "tactileFormat": {
      "available": true,
      "brailleUrl": "https://cdn.example.com/ballot-braille.brf"
    }
  }
}
```

### 5.2 Visual Accommodations

- High contrast color schemes
- Scalable text (up to 36pt)
- Symbol/icon support for candidates
- Color-blind friendly palettes

---

## 6. Security Metadata

### 6.1 Digital Signatures

Every ballot must include cryptographic signature:

```json
{
  "security": {
    "digitalSignature": {
      "algorithm": "ECDSA-P256",
      "publicKey": "base64-encoded-key",
      "signature": "base64-encoded-signature",
      "timestamp": "2025-01-15T10:00:00Z",
      "signer": "Election Authority CA"
    }
  }
}
```

### 6.2 Hash Chains

Ballots participate in hash chain for integrity:

```json
{
  "hashChain": {
    "algorithm": "SHA3-256",
    "ballotHash": "hex-encoded-hash",
    "previousHash": "hex-encoded-previous-hash",
    "blockchainTxId": "blockchain-transaction-id"
  }
}
```

---

## 7. Result Data Format

### 7.1 Result Structure

```json
{
  "resultId": "string (required)",
  "version": "WIA-SOC-015-v1.0",
  "electionId": "string (required)",
  "contestId": "string (required)",
  "reportingUnit": {
    "type": "enum: precinct|district|state|national",
    "id": "string",
    "name": "string"
  },
  "timestamp": "ISO 8601 datetime",
  "status": "enum: preliminary|final|certified",
  "totalBallots": "number",
  "results": [
    {
      "candidateId": "string",
      "votes": "number",
      "percentage": "number (0-100)"
    }
  ],
  "auditData": {
    "ballotsAudited": "number",
    "discrepancies": "number",
    "auditHash": "string",
    "blockchainProof": "string"
  }
}
```

---

## 8. Extension Mechanisms

### 8.1 Custom Fields

Jurisdictions may add custom fields under "extensions" object:

```json
{
  "ballotId": "BALLOT-2025-001",
  "version": "WIA-SOC-015-v1.0",
  "extensions": {
    "jurisdiction-specific": {
      "customField1": "value",
      "customField2": 123
    }
  }
}
```

**Requirements:**
- Extensions must not duplicate standard fields
- Extensions must not contradict standard requirements
- Core functionality must work without extension data

---

## 9. Implementation Guidelines

### 9.1 Data Storage

- Use encryption at rest (AES-256 minimum)
- Implement immutable storage (append-only)
- Maintain multiple redundant copies
- Log all data access with audit trail

### 9.2 Data Migration

When converting legacy data:
1. Analyze existing data structures
2. Create mapping rules to WIA format
3. Implement validation to ensure data quality
4. Test migration with sample data
5. Maintain legacy data alongside WIA format during transition

### 9.3 Performance Optimization

- Use compression for ballot transmission (gzip/brotli)
- Implement caching for frequently accessed ballots
- Distribute static content via CDN
- Index key fields for efficient querying

---

## 10. Conformance Requirements

### 10.1 Bronze Certification

To achieve Bronze certification, systems must:
- Accept and validate WIA-SOC-015 ballot JSON
- Support English language minimum
- Implement required field validation
- Generate compliant result data

### 10.2 Silver/Gold/Platinum Additional Requirements

Higher certification levels add:
- Multi-language support (Silver: 10+, Gold: 50+, Platinum: 99)
- Full accessibility annotations
- Advanced security metadata
- Extension mechanism support

---

## 11. Examples

### 11.1 Complete Ballot Example

See Appendix A for full ballot example including all required and optional fields.

### 11.2 Result Example

See Appendix B for complete result data example.

---

## 12. References

- [JSON Schema Specification](https://json-schema.org/)
- [ISO 639-1 Language Codes](https://www.iso.org/iso-639-language-codes.html)
- [ISO 3166-1 Country Codes](https://www.iso.org/iso-3166-country-codes.html)
- [ISO 8601 Date/Time Format](https://www.iso.org/iso-8601-date-and-time-format.html)

---

**© 2025 World Certification Industry Association (WIA)**  
**弘益人間 (Hongik Ingan) - Benefit All Humanity**  
**License:** MIT License
