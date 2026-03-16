# WIA-UNI-014: Legal System Harmonization - v1.0 Specification

## Phase 1: Legal Data Format

**Version:** 1.0
**Status:** Final
**Date:** 2025-01-15
**Category:** UNI (Unification/Peace)

---

## 1. Overview

This specification defines standardized data formats for legal documents in the WIA-UNI-014 Legal System Harmonization standard. It enables interoperability between North and South Korean legal systems by providing common structures for representing legal data.

### 1.1 Goals

- Standardize legal document representation across jurisdictions
- Enable seamless data exchange between legal systems
- Support automated legal document processing
- Preserve legal document integrity and authenticity
- Facilitate legal system harmonization

### 1.2 Scope

This specification covers:
- Legal document schema (JSON/XML)
- Document type specifications (laws, judgments, contracts, property records)
- Metadata standards
- Validation rules
- Identifier schemes

## 2. Core Legal Document Schema

### 2.1 Base Structure

All legal documents follow this base JSON schema:

```json
{
  "$schema": "https://wiastandards.com/schemas/uni-014/v1.0/legal-document.json",
  "standard": "WIA-UNI-014",
  "version": "1.0",
  "document": {
    "id": "WIA-UNI-014-LEGAL-{UUID}",
    "type": "law|regulation|judgment|contract|property|certificate",
    "jurisdiction": {
      "system": "north|south|unified",
      "level": "national|provincial|municipal",
      "authority": "string"
    },
    "metadata": { /* See section 3 */ },
    "content": { /* See section 4 */ },
    "relationships": { /* See section 5 */ },
    "harmonization": { /* See section 6 */ },
    "verification": { /* See section 7 */ }
  }
}
```

### 2.2 Required Fields

- `standard`: Must be "WIA-UNI-014"
- `version`: Schema version (currently "1.0")
- `document.id`: Unique document identifier
- `document.type`: Document type from enumerated list
- `document.jurisdiction.system`: Jurisdiction system
- `document.metadata.title`: Document title
- `document.metadata.issueDate`: ISO-8601 date

### 2.3 Optional Fields

- `document.relationships`: Links to related documents
- `document.harmonization`: Harmonization status and conflicts
- `document.verification`: Cryptographic verification data

## 3. Metadata Specification

### 3.1 Required Metadata

```json
"metadata": {
  "title": "Official document title",
  "issueDate": "2025-01-15T09:00:00Z",
  "status": "active|superseded|repealed|draft",
  "language": "ko|en"
}
```

### 3.2 Extended Metadata

```json
"metadata": {
  "shortTitle": "Common abbreviated title",
  "effectiveDate": "2025-02-01T00:00:00Z",
  "classification": "public|restricted|confidential",
  "keywords": ["property", "ownership", "transfer"],
  "legalAreas": ["civil law", "property law"]
}
```

## 4. Content Structure

### 4.1 Text Content

```json
"content": {
  "text": "Full legal text in specified language",
  "structure": {
    "chapters": [
      {
        "number": 1,
        "title": "Chapter title",
        "articles": []
      }
    ]
  }
}
```

### 4.2 Multilingual Support

```json
"content": {
  "translations": [
    {
      "language": "ko",
      "text": "한국어 법률 텍스트",
      "official": true
    },
    {
      "language": "en",
      "text": "English legal text",
      "official": false
    }
  ],
  "authoritative": "ko"
}
```

## 5. Document Relationships

```json
"relationships": {
  "amends": ["WIA-UNI-014-LEGAL-abc"],
  "supersedes": ["WIA-UNI-014-LEGAL-xyz"],
  "references": ["WIA-UNI-014-LEGAL-def"],
  "implementedBy": ["WIA-UNI-014-LEGAL-ghi"]
}
```

## 6. Harmonization Metadata

```json
"harmonization": {
  "northCompliance": true|false,
  "southCompliance": true|false,
  "conflicts": [
    {
      "type": "property_ownership_model",
      "description": "Conflict description",
      "severity": "high|medium|low"
    }
  ],
  "resolutionStatus": "unresolved|proposed|resolved",
  "harmonizedVersion": "WIA-UNI-014-LEGAL-harmonized-123"
}
```

## 7. Verification and Security

### 7.1 Document Hashing

```json
"verification": {
  "hash": "SHA-256 hash of document content",
  "algorithm": "SHA-256",
  "timestamp": "2025-01-15T09:00:00Z"
}
```

### 7.2 Digital Signatures

```json
"verification": {
  "authority": "WIA Unified Legal Registry",
  "signature": "Base64-encoded digital signature",
  "certificate": "X.509 certificate chain",
  "blockchain": {
    "network": "WIA Legal Chain",
    "txHash": "0x...",
    "blockNumber": 12345
  }
}
```

## 8. Document Type Specifications

### 8.1 Laws and Regulations

Additional fields for legislative documents:

```json
"lawSpecific": {
  "category": "civil|criminal|administrative|constitutional",
  "code": "Official law number",
  "enactmentProcess": {
    "proposedBy": "Legislature|Executive",
    "passedDate": "2025-01-10",
    "promulgatedDate": "2025-01-15",
    "effectiveDate": "2025-02-01"
  }
}
```

### 8.2 Court Judgments

```json
"judgmentSpecific": {
  "court": {
    "name": "Supreme Court",
    "level": "supreme|appellate|trial"
  },
  "case": {
    "caseNumber": "2025-민-12345",
    "parties": {
      "plaintiff": ["WIA-UNI-001-ID-{UUID}"],
      "defendant": ["WIA-UNI-001-ID-{UUID}"]
    },
    "judgmentDate": "2025-01-20"
  },
  "outcome": {
    "decision": "granted|denied|partiallyGranted"
  }
}
```

### 8.3 Contracts

```json
"contractSpecific": {
  "contractType": "sales|lease|employment|service",
  "parties": [
    {
      "id": "WIA-UNI-001-ID-{UUID}",
      "role": "buyer|seller|lessor|lessee"
    }
  ],
  "terms": {
    "consideration": {
      "amount": 1000000,
      "currency": "KRW"
    }
  }
}
```

### 8.4 Property Records

```json
"propertySpecific": {
  "propertyId": "WIA-UNI-014-PROP-{UUID}",
  "propertyType": "land|building|apartment|commercial",
  "location": {
    "address": "Full address",
    "coordinates": {
      "latitude": 37.5665,
      "longitude": 126.9780
    }
  },
  "ownership": {
    "currentOwner": "WIA-UNI-001-ID-{UUID}",
    "ownershipType": "private|collective|state|mixed"
  }
}
```

## 9. Validation Rules

### 9.1 Structural Validation

All documents MUST:
- Conform to JSON Schema
- Include all required fields
- Use valid enumeration values
- Follow identifier format specifications

### 9.2 Business Logic Validation

- Effective date MUST be >= issue date
- Superseding documents MUST have later dates
- Referenced documents MUST exist in registry
- Party IDs MUST be valid WIA-UNI-001 identifiers

### 9.3 Legal Compliance

- Documents MUST comply with jurisdiction requirements
- Authority MUST have jurisdiction to issue document
- Required signatures/approvals MUST be present

## 10. Identifier Specifications

### 10.1 Document Identifiers

Format: `WIA-UNI-014-LEGAL-{UUID}`

Example: `WIA-UNI-014-LEGAL-a1b2c3d4-e5f6-47g8-h9i0-j1k2l3m4n5o6`

### 10.2 Property Identifiers

Format: `WIA-UNI-014-PROP-{UUID}`

### 10.3 Contract Identifiers

Format: `WIA-UNI-014-CONTRACT-{UUID}`

## 11. Conformance

An implementation conforms to this specification if:

1. It can generate documents conforming to the schema
2. It can validate documents against the schema
3. It correctly handles all required fields
4. It implements all validation rules
5. It generates valid identifiers

---

**© 2025 WIA - World Certification Industry Association**
**弘益人間 · Benefit All Humanity**
**License:** MIT
