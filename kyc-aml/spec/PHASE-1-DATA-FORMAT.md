# WIA-FIN-011 KYC/AML Standard
## Phase 1: Data Format Specification v1.0.0

**Status:** ✅ Complete  
**Last Updated:** 2025-12-25  
**Maintainer:** WIA Standards Committee

---

## 1. Overview

This specification defines the standardized data formats, structures, and schemas for Know Your Customer (KYC) and Anti-Money Laundering (AML) data exchange. It provides a comprehensive framework for representing customer information, verification data, risk assessments, and compliance records in a consistent, interoperable manner.

### 1.1 Objectives

- Define standardized data structures for all KYC/AML entities
- Ensure interoperability between different systems and institutions
- Support regulatory reporting requirements across jurisdictions
- Enable efficient data sharing while maintaining privacy and security
- Facilitate automated processing and analysis

### 1.2 Scope

This phase covers:
- Customer identity data structures
- Beneficial ownership representations
- Document metadata and verification records
- Risk assessment data formats
- Transaction monitoring data structures
- Suspicious activity report formats
- Audit trail and compliance records

---

## 2. Core Data Types

### 2.1 Person Data Structure

```json
{
  "personId": "uuid",
  "personalInfo": {
    "legalName": {
      "firstName": "string",
      "middleName": "string",
      "lastName": "string",
      "suffix": "string",
      "fullName": "string"
    },
    "formerNames": [
      {
        "name": "string",
        "effectiveDate": "ISO8601 date",
        "endDate": "ISO8601 date"
      }
    ],
    "dateOfBirth": "ISO8601 date",
    "placeOfBirth": {
      "city": "string",
      "state": "string",
      "country": "ISO 3166-1 alpha-3"
    },
    "gender": "enum: M|F|X|U",
    "nationality": ["ISO 3166-1 alpha-3"],
    "citizenship": ["ISO 3166-1 alpha-3"]
  },
  "identificationDocuments": [
    {
      "documentType": "enum: passport|drivers_license|national_id|other",
      "documentNumber": "string",
      "issuingCountry": "ISO 3166-1 alpha-3",
      "issuingAuthority": "string",
      "issueDate": "ISO8601 date",
      "expiryDate": "ISO8601 date",
      "verificationStatus": "enum: verified|pending|failed|expired",
      "verificationDate": "ISO8601 timestamp",
      "verificationMethod": "string",
      "documentImages": [
        {
          "imageType": "enum: front|back|full_page",
          "imageUrl": "uri",
          "imageHash": "sha256",
          "uploadDate": "ISO8601 timestamp"
        }
      ]
    }
  ],
  "addresses": [
    {
      "addressType": "enum: residential|mailing|business",
      "isPrimary": "boolean",
      "streetAddress1": "string",
      "streetAddress2": "string",
      "city": "string",
      "stateProvince": "string",
      "postalCode": "string",
      "country": "ISO 3166-1 alpha-3",
      "effectiveDate": "ISO8601 date",
      "endDate": "ISO8601 date",
      "verificationStatus": "enum: verified|unverified",
      "verificationMethod": "string",
      "verificationDate": "ISO8601 timestamp"
    }
  ],
  "contactInfo": {
    "emails": [
      {
        "email": "email format",
        "emailType": "enum: personal|work",
        "isPrimary": "boolean",
        "verified": "boolean",
        "verificationDate": "ISO8601 timestamp"
      }
    ],
    "phoneNumbers": [
      {
        "phoneNumber": "E.164 format",
        "phoneType": "enum: mobile|home|work",
        "isPrimary": "boolean",
        "verified": "boolean",
        "verificationDate": "ISO8601 timestamp"
      }
    ]
  },
  "taxIdentifiers": [
    {
      "taxIdType": "enum: ssn|tin|vat|other",
      "taxId": "string",
      "issuingCountry": "ISO 3166-1 alpha-3",
      "verified": "boolean"
    }
  ],
  "occupation": {
    "employmentStatus": "enum: employed|self_employed|retired|student|unemployed",
    "occupation": "string",
    "industryCode": "NAICS code",
    "employer": "string",
    "employerAddress": "address object",
    "positionTitle": "string",
    "yearsEmployed": "integer"
  },
  "politicallyExposedPerson": {
    "isPEP": "boolean",
    "pepType": "enum: domestic|foreign|international_organization",
    "pepRole": "string",
    "pepCountry": "ISO 3166-1 alpha-3",
    "pepStartDate": "ISO8601 date",
    "pepEndDate": "ISO8601 date",
    "isFormerPEP": "boolean",
    "relationship": "enum: self|family_member|close_associate"
  }
}
```

### 2.2 Legal Entity Data Structure

```json
{
  "entityId": "uuid",
  "entityInfo": {
    "legalName": "string",
    "tradingNames": ["string"],
    "formerNames": [
      {
        "name": "string",
        "effectiveDate": "ISO8601 date",
        "endDate": "ISO8601 date"
      }
    ],
    "entityType": "enum: corporation|llc|partnership|trust|foundation|ngo|other",
    "incorporationDate": "ISO8601 date",
    "jurisdictionOfIncorporation": "ISO 3166-1 alpha-3",
    "registrationNumber": "string",
    "taxIdentifiers": [
      {
        "taxIdType": "enum: ein|vat|other",
        "taxId": "string",
        "issuingCountry": "ISO 3166-1 alpha-3"
      }
    ],
    "businessAddresses": [
      {
        "addressType": "enum: registered|principal|branch",
        "address": "address object",
        "effectiveDate": "ISO8601 date"
      }
    ],
    "contactInfo": {
      "mainPhone": "E.164 format",
      "mainEmail": "email format",
      "website": "uri"
    },
    "businessActivities": {
      "primaryActivity": "string",
      "industryCode": "NAICS code",
      "description": "string",
      "licenses": [
        {
          "licenseType": "string",
          "licenseNumber": "string",
          "issuingAuthority": "string",
          "issueDate": "ISO8601 date",
          "expiryDate": "ISO8601 date"
        }
      ]
    }
  },
  "ownershipStructure": {
    "beneficialOwners": [
      {
        "ownerId": "uuid",
        "personRef": "person object or reference",
        "ownershipPercentage": "decimal",
        "ownershipType": "enum: direct|indirect",
        "controlMechanism": "enum: equity|voting_rights|other",
        "effectiveDate": "ISO8601 date"
      }
    ],
    "controllingPersons": [
      {
        "personRef": "person object or reference",
        "controlType": "enum: senior_officer|authorized_signatory|other",
        "position": "string"
      }
    ],
    "corporateStructure": [
      {
        "parentEntity": "entity reference",
        "childEntity": "entity reference",
        "ownershipPercentage": "decimal",
        "relationshipType": "enum: subsidiary|affiliate|branch"
      }
    ]
  },
  "financialInfo": {
    "annualRevenue": {
      "amount": "decimal",
      "currency": "ISO 4217",
      "fiscalYear": "integer"
    },
    "employeeCount": "integer",
    "sourceOfFunds": "string",
    "expectedTransactionVolume": {
      "volumeRange": "enum: low|medium|high|very_high",
      "monthlyAmount": "decimal",
      "currency": "ISO 4217"
    }
  }
}
```

### 2.3 Customer Due Diligence (CDD) Record

```json
{
  "cddId": "uuid",
  "customerId": "uuid",
  "customerType": "enum: individual|entity",
  "cddType": "enum: standard|simplified|enhanced",
  "cddDate": "ISO8601 timestamp",
  "performedBy": {
    "userId": "uuid",
    "userName": "string",
    "department": "string"
  },
  "identificationVerification": {
    "method": "enum: documentary|non_documentary|biometric|digital_identity",
    "verificationStatus": "enum: passed|failed|partial",
    "verificationDate": "ISO8601 timestamp",
    "documentsReviewed": ["document references"],
    "dataSourcesUsed": ["string"],
    "verificationNotes": "string"
  },
  "beneficialOwnershipVerification": {
    "verificationStatus": "enum: complete|incomplete|not_applicable",
    "beneficialOwnersIdentified": "integer",
    "verificationDate": "ISO8601 timestamp",
    "ownershipChart": "document reference",
    "verificationNotes": "string"
  },
  "sourceOfFundsWealth": {
    "sourceOfFunds": "enum: employment|business|inheritance|investment|other",
    "sourceOfFundsDescription": "string",
    "sourceOfWealth": "string",
    "sourceOfWealthDescription": "string",
    "verificationEvidence": ["document references"],
    "verificationStatus": "enum: verified|partially_verified|unverified"
  },
  "purposeOfRelationship": {
    "accountPurpose": "string",
    "intendedAccountActivity": "string",
    "expectedTransactionTypes": ["string"],
    "expectedGeographicScope": ["ISO 3166-1 alpha-3"]
  },
  "riskAssessment": {
    "overallRiskRating": "enum: low|medium|high|prohibited",
    "riskScore": "integer (0-100)",
    "riskFactors": [
      {
        "factorType": "enum: customer|geographic|product|delivery_channel",
        "factorDescription": "string",
        "riskLevel": "enum: low|medium|high",
        "riskScore": "integer"
      }
    ],
    "mitigatingFactors": ["string"],
    "aggravatingFactors": ["string"],
    "assessmentDate": "ISO8601 timestamp",
    "assessedBy": "user reference",
    "approvalRequired": "boolean",
    "approvedBy": "user reference",
    "approvalDate": "ISO8601 timestamp"
  },
  "screeningResults": {
    "sanctionsScreening": {
      "screeningDate": "ISO8601 timestamp",
      "listsChecked": ["string"],
      "matchesFound": "integer",
      "matches": [
        {
          "listName": "string",
          "matchedName": "string",
          "matchScore": "decimal",
          "matchStatus": "enum: true_match|false_positive|under_review",
          "reviewedBy": "user reference",
          "reviewDate": "ISO8601 timestamp",
          "reviewNotes": "string"
        }
      ],
      "clearanceStatus": "enum: clear|hit|under_review"
    },
    "pepScreening": {
      "screeningDate": "ISO8601 timestamp",
      "pepStatus": "enum: not_pep|pep|former_pep",
      "pepDetails": "object",
      "clearanceStatus": "enum: clear|hit|under_review"
    },
    "adverseMediaScreening": {
      "screeningDate": "ISO8601 timestamp",
      "articlesFound": "integer",
      "relevantArticles": [
        {
          "source": "string",
          "date": "ISO8601 date",
          "title": "string",
          "summary": "string",
          "relevanceScore": "decimal",
          "category": "enum: financial_crime|corruption|sanctions|other"
        }
      ],
      "riskIndicator": "enum: none|low|medium|high"
    }
  },
  "ongoingMonitoring": {
    "reviewFrequency": "enum: continuous|monthly|quarterly|annually|biannually",
    "lastReviewDate": "ISO8601 timestamp",
    "nextReviewDate": "ISO8601 date",
    "monitoringAlerts": "integer",
    "transactionVolume": {
      "period": "string",
      "volumeMetrics": "object"
    }
  },
  "documentation": [
    {
      "documentType": "string",
      "documentRef": "uri",
      "uploadDate": "ISO8601 timestamp",
      "expiryDate": "ISO8601 date"
    }
  ],
  "complianceNotes": "string",
  "recordStatus": "enum: active|inactive|closed",
  "recordVersion": "integer",
  "lastUpdated": "ISO8601 timestamp"
}
```

### 2.4 Transaction Data Structure

```json
{
  "transactionId": "uuid",
  "transactionType": "enum: deposit|withdrawal|transfer|wire|payment|exchange",
  "transactionDate": "ISO8601 timestamp",
  "valueDate": "ISO8601 date",
  "amount": {
    "value": "decimal",
    "currency": "ISO 4217"
  },
  "convertedAmount": {
    "value": "decimal",
    "currency": "ISO 4217",
    "exchangeRate": "decimal"
  },
  "originator": {
    "customerId": "uuid",
    "accountNumber": "string",
    "name": "string",
    "address": "address object",
    "identificationNumber": "string"
  },
  "beneficiary": {
    "customerId": "uuid",
    "accountNumber": "string",
    "name": "string",
    "address": "address object",
    "identificationNumber": "string"
  },
  "intermediaries": [
    {
      "institutionName": "string",
      "institutionIdentifier": "BIC/SWIFT",
      "country": "ISO 3166-1 alpha-3"
    }
  ],
  "transactionDetails": {
    "paymentPurpose": "string",
    "description": "string",
    "referenceNumber": "string",
    "relatedTransactions": ["uuid"]
  },
  "geographicInfo": {
    "originatingCountry": "ISO 3166-1 alpha-3",
    "destinationCountry": "ISO 3166-1 alpha-3",
    "transactionLocation": "string"
  },
  "channelInfo": {
    "deliveryChannel": "enum: branch|atm|online|mobile|phone",
    "initiationMethod": "enum: customer|automated|third_party",
    "deviceInfo": {
      "deviceId": "string",
      "ipAddress": "IP address",
      "geolocation": "coordinates"
    }
  },
  "riskIndicators": {
    "riskScore": "integer (0-100)",
    "riskLevel": "enum: low|medium|high",
    "riskFactors": ["string"],
    "unusualIndicators": ["string"]
  },
  "monitoringStatus": {
    "reviewed": "boolean",
    "reviewedBy": "user reference",
    "reviewDate": "ISO8601 timestamp",
    "alerts": [
      {
        "alertId": "uuid",
        "alertType": "string",
        "alertDate": "ISO8601 timestamp",
        "alertStatus": "enum: open|investigating|closed",
        "disposition": "enum: false_positive|suspicious|escalated"
      }
    ]
  },
  "complianceFlags": {
    "ctrFiled": "boolean",
    "ctrFilingDate": "ISO8601 timestamp",
    "sarFiled": "boolean",
    "sarFilingDate": "ISO8601 timestamp",
    "blocked": "boolean",
    "blockReason": "string"
  }
}
```

### 2.5 Suspicious Activity Report (SAR) Structure

```json
{
  "sarId": "uuid",
  "filingInstitution": {
    "institutionName": "string",
    "institutionIdentifier": "string",
    "institutionAddress": "address object",
    "contactPerson": {
      "name": "string",
      "title": "string",
      "phone": "E.164 format",
      "email": "email format"
    }
  },
  "reportType": "enum: initial|continuing|corrected|late",
  "reportDate": "ISO8601 timestamp",
  "regulatoryFilingId": "string",
  "filedWith": ["string"],
  "subject": {
    "subjectType": "enum: individual|entity|both",
    "subjects": [
      {
        "subjectId": "uuid",
        "role": "enum: primary|secondary",
        "customerRef": "customer reference",
        "relationshipToInstitution": "enum: customer|employee|other",
        "accountsInvolved": ["string"]
      }
    ]
  },
  "suspiciousActivity": {
    "activityType": [
      "enum: structuring|layering|trade_based_ml|terrorist_financing|fraud|identity_theft|elder_abuse|human_trafficking|cybercrime|sanctions_violation|other"
    ],
    "activityStartDate": "ISO8601 date",
    "activityEndDate": "ISO8601 date",
    "totalAmount": {
      "value": "decimal",
      "currency": "ISO 4217"
    },
    "transactionsInvolved": ["transaction references"],
    "narrative": "string (detailed description)",
    "redFlags": ["string"],
    "investigationSummary": "string"
  },
  "documentationAttached": [
    {
      "documentType": "string",
      "documentRef": "uri",
      "description": "string"
    }
  ],
  "priorReports": [
    {
      "priorSARId": "uuid",
      "priorFilingDate": "ISO8601 timestamp",
      "relationship": "string"
    }
  ],
  "lawEnforcementContact": {
    "contacted": "boolean",
    "contactDate": "ISO8601 timestamp",
    "agency": "string",
    "caseNumber": "string"
  },
  "internalTracking": {
    "caseNumber": "string",
    "assignedTo": "user reference",
    "investigationStartDate": "ISO8601 timestamp",
    "investigationCloseDate": "ISO8601 timestamp",
    "approvals": [
      {
        "approverName": "string",
        "approverTitle": "string",
        "approvalDate": "ISO8601 timestamp"
      }
    ]
  },
  "confidentiality": {
    "restrictedAccess": "boolean",
    "accessList": ["user references"],
    "safeguards": "string"
  }
}
```

---

## 3. Data Exchange Formats

### 3.1 Supported Formats

The standard supports multiple data exchange formats:

- **JSON**: Primary format for API interactions
- **XML**: For legacy system integration and regulatory reporting
- **CSV**: For bulk data transfers (limited fields)
- **Protocol Buffers**: For high-performance binary serialization

### 3.2 Data Validation Rules

All data exchanges must comply with:

1. **Schema Validation**: Data must conform to defined schemas
2. **Data Type Validation**: Correct types for all fields
3. **Format Validation**: Proper formats for dates, currencies, identifiers
4. **Business Rule Validation**: Logical consistency checks
5. **Required Field Validation**: All mandatory fields present
6. **Referential Integrity**: Valid references to related entities

### 3.3 Character Encoding

- Default encoding: UTF-8
- All text fields support Unicode
- Special character handling defined for names and addresses

---

## 4. Data Security and Privacy

### 4.1 Sensitive Data Classification

Data elements are classified by sensitivity:

- **Public**: Entity names, addresses (redacted as needed)
- **Internal**: Risk ratings, relationship details
- **Confidential**: Identification numbers, financial details
- **Restricted**: SAR information, investigation details

### 4.2 Data Protection Requirements

- PII must be encrypted at rest and in transit
- Access controls based on sensitivity classification
- Audit logging for all data access
- Data minimization principles applied
- Right to be forgotten accommodations (where legally permissible)

---

## 5. Extensibility and Custom Fields

### 5.1 Custom Field Support

The standard allows custom fields through:

```json
{
  "customFields": {
    "fieldName": {
      "value": "any",
      "dataType": "string",
      "description": "string",
      "source": "string"
    }
  }
}
```

### 5.2 Jurisdiction-Specific Extensions

Jurisdictions may define additional required fields through registered extensions:

```json
{
  "jurisdictionExtensions": {
    "jurisdictionCode": "ISO 3166-1 alpha-3",
    "regulatoryRequirements": {
      "fieldName": "value"
    }
  }
}
```

---

## 6. Versioning and Compatibility

### 6.1 Version Numbering

Format: MAJOR.MINOR.PATCH

- MAJOR: Breaking changes
- MINOR: New features, backward compatible
- PATCH: Bug fixes, clarifications

### 6.2 Backward Compatibility

- Implementations must support current and previous major version
- Deprecated fields marked with deprecation date
- Migration guides provided for major version changes

---

## 7. Implementation Guidelines

### 7.1 Minimum Viable Implementation

Core required elements for compliance:
- Person and Entity structures
- CDD record keeping
- Transaction monitoring data
- SAR reporting capability

### 7.2 Best Practices

- Use provided reference implementations
- Implement comprehensive validation
- Maintain detailed audit trails
- Support data portability
- Plan for schema evolution

---

## Appendix A: Code Lists and Enumerations

### A.1 Country Codes
ISO 3166-1 alpha-3 codes

### A.2 Currency Codes
ISO 4217 codes

### A.3 Industry Codes
NAICS (North American Industry Classification System)

### A.4 Document Types
Standardized document type identifiers

### A.5 Risk Categories
Defined risk classification levels

---

## Appendix B: Sample Data

See separate files for complete examples of each data structure.

---

## Revision History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-12-25 | Initial release |

---

**Document Control**  
Classification: Public  
Distribution: Unrestricted  
© 2025 WIA (World Certification Industry Association)
