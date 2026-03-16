# WIA-LEGAL-001: Digital Court - Phase 1: Data Format

**Version:** 1.0
**Status:** Complete
**Last Updated:** 2025-12-27

## Overview

Phase 1 defines the standardized data formats for digital court proceedings, including case information, filing documents, evidence metadata, and hearing records.

## 1. Case Data Schema

### 1.1 Core Case Structure

```json
{
  "standard": "WIA-LEGAL-001",
  "version": "1.0",
  "caseId": "string (UUID)",
  "caseNumber": "string (jurisdiction-specific format)",
  "caseType": "enum",
  "court": {
    "courtId": "string",
    "courtName": "string",
    "jurisdiction": "string",
    "level": "enum: district|appellate|supreme"
  },
  "parties": {
    "plaintiffs": ["Party"],
    "defendants": ["Party"],
    "attorneys": ["Attorney"],
    "judges": ["Judge"]
  },
  "filingDate": "ISO 8601 timestamp",
  "status": "enum",
  "metadata": "CaseMetadata",
  "timestamp": "ISO 8601 timestamp"
}
```

### 1.2 Party Data Structure

```json
{
  "partyId": "string (UUID)",
  "type": "enum: individual|corporation|government|ngo",
  "name": "string",
  "legalName": "string",
  "did": "string (DID)",
  "contact": {
    "email": "string",
    "phone": "string",
    "address": "Address"
  },
  "representative": "Attorney (optional)",
  "role": "enum: plaintiff|defendant|third-party"
}
```

### 1.3 Document Filing Format

```json
{
  "documentId": "string (UUID)",
  "caseId": "string (UUID)",
  "type": "enum: pleading|motion|evidence|order|transcript",
  "title": "string",
  "filedBy": "string (partyId)",
  "filedDate": "ISO 8601 timestamp",
  "confidentiality": "enum: public|sealed|confidential",
  "format": "string (MIME type)",
  "hash": "string (SHA-256)",
  "signature": "DigitalSignature",
  "metadata": {
    "pageCount": "number",
    "wordCount": "number",
    "language": "string (ISO 639-1)",
    "tags": ["string"]
  },
  "storageLocation": "string (URI)",
  "timestamp": "ISO 8601 timestamp"
}
```

## 2. Evidence Metadata Format

### 2.1 Digital Evidence

```json
{
  "evidenceId": "string (UUID)",
  "caseId": "string (UUID)",
  "type": "enum: document|photo|video|audio|digital|physical",
  "description": "string",
  "submittedBy": "string (partyId)",
  "submittedDate": "ISO 8601 timestamp",
  "chainOfCustody": ["CustodyRecord"],
  "authenticity": {
    "verified": "boolean",
    "method": "string",
    "verifiedBy": "string",
    "verifiedDate": "ISO 8601 timestamp"
  },
  "forensics": {
    "hash": "string",
    "metadata": "object",
    "analysisReport": "string (URI)"
  },
  "admissibility": {
    "status": "enum: pending|admitted|excluded",
    "rulingDate": "ISO 8601 timestamp",
    "rulingJudge": "string"
  }
}
```

## 3. Hearing Record Format

### 3.1 Virtual Hearing Data

```json
{
  "hearingId": "string (UUID)",
  "caseId": "string (UUID)",
  "type": "enum: preliminary|trial|motion|sentencing|appeal",
  "scheduledDate": "ISO 8601 timestamp",
  "duration": "number (minutes)",
  "mode": "enum: in-person|virtual|hybrid",
  "participants": {
    "judge": "Judge",
    "parties": ["Party"],
    "attorneys": ["Attorney"],
    "witnesses": ["Witness"],
    "observers": ["Observer"]
  },
  "recording": {
    "enabled": "boolean",
    "videoUrl": "string (URI)",
    "audioUrl": "string (URI)",
    "transcriptUrl": "string (URI)"
  },
  "proceedings": {
    "startTime": "ISO 8601 timestamp",
    "endTime": "ISO 8601 timestamp",
    "events": ["HearingEvent"]
  },
  "outcome": {
    "ruling": "string",
    "orders": ["CourtOrder"],
    "nextHearing": "ISO 8601 timestamp (optional)"
  }
}
```

## 4. Court Order Format

```json
{
  "orderId": "string (UUID)",
  "caseId": "string (UUID)",
  "type": "enum: procedural|substantive|temporary|final",
  "title": "string",
  "issuedBy": "string (judgeId)",
  "issuedDate": "ISO 8601 timestamp",
  "effectiveDate": "ISO 8601 timestamp",
  "expirationDate": "ISO 8601 timestamp (optional)",
  "content": "string",
  "conditions": ["string"],
  "signature": "DigitalSignature",
  "enforcement": {
    "status": "enum: pending|active|completed|appealed",
    "enforcedBy": "string",
    "complianceDeadline": "ISO 8601 timestamp"
  }
}
```

## 5. Validation Rules

### 5.1 Required Fields

All case records MUST include:
- `caseId` (UUID v4)
- `caseNumber` (jurisdiction-specific)
- `caseType`
- `filingDate`
- `court` information
- At least one plaintiff and one defendant

### 5.2 Data Integrity

- All timestamps MUST be in ISO 8601 format with timezone
- All document hashes MUST use SHA-256 or stronger
- All monetary amounts MUST include currency code (ISO 4217)
- All dates MUST be timezone-aware

### 5.3 Security Requirements

- Personally Identifiable Information (PII) MUST be encrypted
- Sealed documents MUST have restricted access controls
- All signatures MUST use approved cryptographic algorithms
- Audit logs MUST be maintained for all data access

## 6. Examples

### 6.1 Complete Case Example

```json
{
  "standard": "WIA-LEGAL-001",
  "version": "1.0",
  "caseId": "550e8400-e29b-41d4-a716-446655440000",
  "caseNumber": "2025-CV-001234",
  "caseType": "civil",
  "court": {
    "courtId": "court-nysd-001",
    "courtName": "Southern District of New York",
    "jurisdiction": "US-NY",
    "level": "district"
  },
  "parties": {
    "plaintiffs": [{
      "partyId": "party-001",
      "type": "individual",
      "name": "John Doe",
      "legalName": "John Michael Doe",
      "did": "did:wia:legal:johndoe123",
      "role": "plaintiff"
    }],
    "defendants": [{
      "partyId": "party-002",
      "type": "corporation",
      "name": "Acme Corp",
      "legalName": "Acme Corporation Inc.",
      "did": "did:wia:legal:acmecorp",
      "role": "defendant"
    }]
  },
  "filingDate": "2025-01-15T09:00:00Z",
  "status": "active",
  "timestamp": "2025-01-15T09:00:00Z"
}
```

## 7. Interoperability

### 7.1 Cross-Jurisdiction Mapping

Data format supports mapping to:
- US Federal Court CM/ECF system
- UK Courts Digital Case System
- EU e-Justice Portal
- International Court of Justice systems

### 7.2 WIA Standard Integration

- **WIA-LEGAL-004**: Digital Evidence format compatibility
- **WIA-LEGAL-008**: E-Notary signature integration
- **WIA-BLOCKCHAIN-001**: Immutable record storage

---

**弘益人間 (Hongik Ingan)** - *Benefit All Humanity*

© 2025 WIA - World Certification Industry Association | MIT License
