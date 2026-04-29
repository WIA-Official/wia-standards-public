# WIA-LEGAL_TECH: PHASE 1 - Data Format Specification

**Version:** 1.0
**Status:** Official
**Philosophy:** 弘益人間 (Hongik Ingan - Benefit All Humanity)

## Overview

This specification defines standardized data formats for legal technology systems, enabling interoperability across legal document management, contract lifecycle management, case management, and AI-powered legal analytics platforms.

## 1. Legal Document Format

### 1.1 Document Metadata Standard

```json
{
  "documentId": "string (UUID)",
  "documentType": "contract|brief|motion|pleading|evidence|memo|opinion|agreement",
  "title": "string",
  "jurisdiction": "string (ISO 3166-1 alpha-2)",
  "language": "string (ISO 639-1)",
  "version": "string (semver)",
  "status": "draft|review|approved|executed|archived|void",
  "classification": "public|internal|confidential|privileged",
  "practiceArea": "string[]",
  "createdAt": "ISO 8601 timestamp",
  "modifiedAt": "ISO 8601 timestamp",
  "effectiveDate": "ISO 8601 date",
  "expirationDate": "ISO 8601 date",
  "parties": [{
    "partyId": "string",
    "role": "plaintiff|defendant|client|opposing|witness|expert",
    "entityType": "individual|corporation|government|organization",
    "name": "string",
    "contact": "object"
  }],
  "authors": [{
    "authorId": "string",
    "name": "string",
    "role": "attorney|paralegal|clerk|associate|partner",
    "barNumber": "string",
    "jurisdiction": "string"
  }],
  "relatedDocuments": [{
    "documentId": "string",
    "relationship": "supersedes|amends|references|exhibits|incorporates"
  }],
  "checksums": {
    "sha256": "string",
    "md5": "string"
  },
  "digitalSignatures": [{
    "signerId": "string",
    "algorithm": "RSA|ECDSA",
    "timestamp": "ISO 8601 timestamp",
    "signature": "string (base64)"
  }]
}
```

### 1.2 Practice Area Taxonomy

```yaml
practice_areas:
  - corporate:
      - mergers_acquisitions
      - corporate_governance
      - securities
      - business_formation
  - litigation:
      - civil_litigation
      - criminal_defense
      - appellate
      - arbitration
  - intellectual_property:
      - patents
      - trademarks
      - copyrights
      - trade_secrets
  - employment:
      - labor_relations
      - employment_discrimination
      - wage_hour
  - real_estate:
      - commercial
      - residential
      - land_use
      - construction
  - family:
      - divorce
      - custody
      - adoption
  - regulatory:
      - compliance
      - healthcare
      - environmental
      - privacy_data_protection
```

## 2. Contract Data Structure

### 2.1 Contract Schema

```json
{
  "contractId": "string (UUID)",
  "contractType": "nda|msa|sow|employment|lease|purchase|license|partnership",
  "title": "string",
  "parties": [{
    "partyId": "string",
    "role": "buyer|seller|licensor|licensee|employer|employee|landlord|tenant",
    "entityName": "string",
    "legalName": "string",
    "jurisdiction": "string",
    "address": "object",
    "representative": {
      "name": "string",
      "title": "string",
      "authority": "signatory|witness|advisor"
    }
  }],
  "terms": {
    "effectiveDate": "ISO 8601 date",
    "expirationDate": "ISO 8601 date",
    "autoRenewal": "boolean",
    "renewalTerm": "string (duration)",
    "terminationNotice": "string (duration)",
    "governingLaw": "string (jurisdiction)",
    "venue": "string"
  },
  "financialTerms": [{
    "type": "payment|fee|penalty|escrow|deposit",
    "amount": "number",
    "currency": "string (ISO 4217)",
    "schedule": "one_time|recurring|milestone_based",
    "dueDate": "ISO 8601 date",
    "frequency": "daily|weekly|monthly|quarterly|annually"
  }],
  "clauses": [{
    "clauseId": "string",
    "type": "confidentiality|indemnification|limitation_of_liability|warranty|ip_rights|termination|force_majeure|dispute_resolution",
    "title": "string",
    "text": "string",
    "risk": "low|medium|high|critical",
    "standardDeviation": "number",
    "aiAnalysis": {
      "favorability": "favorable|neutral|unfavorable",
      "riskScore": "number (0-100)",
      "suggestions": "string[]",
      "precedents": "string[]"
    }
  }],
  "obligations": [{
    "obligationId": "string",
    "partyId": "string",
    "type": "deliverable|action|payment|reporting",
    "description": "string",
    "deadline": "ISO 8601 date",
    "status": "pending|in_progress|completed|overdue|waived",
    "dependencies": "string[]"
  }],
  "milestones": [{
    "milestoneId": "string",
    "name": "string",
    "dueDate": "ISO 8601 date",
    "completionCriteria": "string",
    "status": "pending|achieved|missed"
  }],
  "amendments": [{
    "amendmentId": "string",
    "date": "ISO 8601 date",
    "description": "string",
    "modifiedClauses": "string[]"
  }],
  "attachments": [{
    "attachmentId": "string",
    "type": "exhibit|schedule|addendum",
    "title": "string",
    "fileUrl": "string"
  }]
}
```

### 2.2 Clause Library Format

```json
{
  "clauseLibraryId": "string",
  "version": "string",
  "clauses": [{
    "id": "string",
    "category": "string",
    "name": "string",
    "template": "string (with {{variables}})",
    "variables": [{
      "name": "string",
      "type": "string|number|date|boolean|enum",
      "required": "boolean",
      "default": "any",
      "validation": "regex|range|enum_values"
    }],
    "jurisdiction": "string[]",
    "riskLevel": "low|medium|high",
    "commonIn": "string[] (contract types)",
    "alternatives": "string[] (alternative clause IDs)",
    "lastUpdated": "ISO 8601 timestamp",
    "precedents": [{
      "caseId": "string",
      "citation": "string",
      "jurisdiction": "string",
      "year": "number"
    }]
  }]
}
```

## 3. Case Management Data Format

### 3.1 Case Record Structure

```json
{
  "caseId": "string (UUID)",
  "caseNumber": "string",
  "court": "string",
  "jurisdiction": "string",
  "caseType": "civil|criminal|administrative|appellate",
  "status": "filed|active|settled|dismissed|judgment|appeal",
  "filingDate": "ISO 8601 date",
  "trialDate": "ISO 8601 date",
  "parties": [{
    "partyId": "string",
    "role": "plaintiff|defendant|petitioner|respondent",
    "name": "string",
    "representation": {
      "attorneyId": "string",
      "firmName": "string",
      "barNumber": "string"
    }
  }],
  "claims": [{
    "claimId": "string",
    "type": "string",
    "description": "string",
    "damagesRequested": {
      "amount": "number",
      "currency": "string",
      "type": "compensatory|punitive|statutory"
    }
  }],
  "docket": [{
    "entryId": "string",
    "date": "ISO 8601 date",
    "type": "filing|motion|order|hearing|trial",
    "description": "string",
    "documentId": "string",
    "filedBy": "string"
  }],
  "deadlines": [{
    "deadlineId": "string",
    "type": "discovery|motion|response|trial",
    "dueDate": "ISO 8601 date",
    "description": "string",
    "status": "upcoming|met|missed|extended"
  }],
  "discovery": {
    "interrogatories": "number",
    "depositions": [{
      "deponentName": "string",
      "date": "ISO 8601 date",
      "transcriptId": "string"
    }],
    "documentsProduced": "number",
    "requestsForAdmission": "number"
  },
  "evidence": [{
    "evidenceId": "string",
    "type": "document|physical|digital|testimonial",
    "description": "string",
    "custodian": "string",
    "chainOfCustody": "object[]",
    "admissible": "boolean",
    "exhibits": "string[]"
  }]
}
```

## 4. E-Discovery Data Format

### 4.1 Document Review Format

```json
{
  "reviewId": "string",
  "documentId": "string",
  "reviewerInfo": {
    "reviewerId": "string",
    "name": "string",
    "role": "attorney|paralegal|contract_reviewer",
    "timestamp": "ISO 8601 timestamp"
  },
  "coding": {
    "responsive": "boolean",
    "privileged": "boolean",
    "confidential": "boolean",
    "hotDocument": "boolean",
    "relevance": "number (1-10)",
    "issues": "string[]",
    "custodian": "string",
    "dateRange": {
      "start": "ISO 8601 date",
      "end": "ISO 8601 date"
    }
  },
  "tags": "string[]",
  "notes": "string",
  "redactions": [{
    "reason": "privileged|pii|confidential|irrelevant",
    "location": "page|paragraph|line",
    "coordinates": "object"
  }],
  "aiAssistance": {
    "predictiveRelevance": "number (0-1)",
    "suggestedTags": "string[]",
    "similarDocuments": "string[]",
    "keyPhrases": "string[]"
  }
}
```

## 5. Legal Analytics Data Format

### 5.1 Legal Research Result Format

```json
{
  "researchId": "string",
  "query": "string",
  "timestamp": "ISO 8601 timestamp",
  "results": [{
    "resultId": "string",
    "type": "case_law|statute|regulation|article|treatise",
    "citation": "string",
    "title": "string",
    "court": "string",
    "date": "ISO 8601 date",
    "jurisdiction": "string",
    "relevanceScore": "number (0-100)",
    "keyPassages": [{
      "text": "string",
      "page": "number",
      "context": "string"
    }],
    "citedBy": "number",
    "treatment": "positive|negative|distinguished|overruled|followed",
    "shepardSignal": "red|yellow|green"
  }],
  "relatedSearches": "string[]",
  "filters": {
    "jurisdiction": "string[]",
    "dateRange": "object",
    "court": "string[]",
    "practiceArea": "string[]"
  }
}
```

## 6. Compliance & Risk Data Format

### 6.1 Compliance Check Record

```json
{
  "checkId": "string",
  "documentId": "string",
  "framework": "gdpr|ccpa|sox|hipaa|fcpa|aml|kyc",
  "timestamp": "ISO 8601 timestamp",
  "status": "compliant|non_compliant|needs_review",
  "score": "number (0-100)",
  "findings": [{
    "findingId": "string",
    "severity": "critical|high|medium|low",
    "category": "missing_clause|non_standard_language|prohibited_term|regulatory_violation",
    "description": "string",
    "location": "string",
    "recommendation": "string",
    "reference": "string (regulation section)"
  }],
  "requiredClauses": [{
    "clauseType": "string",
    "status": "present|missing|incomplete",
    "requirement": "string"
  }],
  "prohibitedTerms": [{
    "term": "string",
    "location": "string",
    "severity": "critical|high|medium|low"
  }]
}
```

## 7. File Format Support

### 7.1 Supported Document Formats

| Format | Extension | Use Case | AI Processing |
|--------|-----------|----------|---------------|
| PDF | .pdf | Final documents, exhibits | OCR, text extraction |
| DOCX | .docx | Draft documents, templates | Direct text access |
| RTF | .rtf | Cross-platform documents | Text extraction |
| TXT | .txt | Plain text documents | Direct access |
| HTML | .html | Web-based documents | DOM parsing |
| TIFF | .tiff | Scanned documents | OCR required |
| MSG | .msg | Email evidence | Metadata extraction |
| PST | .pst | Email archives | Email parsing |
| JSON | .json | Structured data | Direct parsing |
| XML | .xml | Structured legal data | Schema validation |

### 7.2 Metadata Extraction Requirements

All document formats must support extraction of:
- Creation date/time
- Last modification date/time
- Author information
- Version history
- Digital signatures
- Page count
- Word count
- Embedded metadata

## 8. Data Exchange Format

### 8.1 Legal Data Package (LDP)

```json
{
  "ldpVersion": "1.0",
  "packageId": "string (UUID)",
  "created": "ISO 8601 timestamp",
  "sender": {
    "organizationId": "string",
    "name": "string",
    "contact": "object"
  },
  "receiver": {
    "organizationId": "string",
    "name": "string"
  },
  "encryption": {
    "algorithm": "AES-256|RSA-2048",
    "keyId": "string"
  },
  "manifest": [{
    "fileId": "string",
    "filename": "string",
    "fileType": "string",
    "size": "number (bytes)",
    "checksum": "string (SHA-256)",
    "metadata": "object"
  }],
  "index": {
    "documents": "number",
    "totalSize": "number (bytes)",
    "dateRange": "object"
  }
}
```

## 9. Validation Rules

### 9.1 Data Quality Requirements

- All dates must be in ISO 8601 format
- All currency amounts must include ISO 4217 currency code
- All UUIDs must be version 4
- All checksums must use SHA-256
- All timestamps must include timezone
- All jurisdictions must use ISO 3166-1 alpha-2 codes
- All text must be UTF-8 encoded
- All documents must have unique identifiers
- All parties must have valid contact information
- All financial terms must be unambiguous

### 9.2 Schema Validation

All data formats must validate against JSON Schema or equivalent schema validators. Validation must occur:
- On data creation
- Before data transmission
- On data receipt
- Before database insertion

---

**Document Control**
- Created: 2026-01-12
- Version: 1.0
- Status: Official
- Next Review: 2026-07-12

**Copyright © 2025 WIA (World Certification Industry Association)**
弘益人間 · Benefit All Humanity
