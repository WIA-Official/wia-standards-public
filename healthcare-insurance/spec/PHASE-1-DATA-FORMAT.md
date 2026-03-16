# WIA-SOC-019 PHASE 1: Data Format Specification

## Healthcare Insurance Standard - Data Format and Structure

**Version:** 1.0
**Status:** PUBLISHED
**Last Updated:** 2025-12-26

---

## 1. Overview

This document specifies the data formats, structures, and schemas for healthcare insurance systems under the WIA-SOC-019 standard. It defines how insurance data should be organized, encoded, and represented across enrollment, claims, benefits, and provider network management.

### 1.1 Objectives

- Establish standard data formats for universal insurance coverage
- Enable interoperability across insurance systems globally
- Support efficient claims processing and adjudication
- Facilitate real-time eligibility verification
- Enable cross-border healthcare coverage
- Ensure privacy and security of sensitive health data

### 1.2 Scope

This specification covers:
- Member enrollment and demographic data
- Coverage and benefits structures
- Claims and service records
- Provider network data
- Premium and payment information
- Cross-border healthcare transactions

---

## 2. Core Data Elements

### 2.1 Member Enrollment Data

#### 2.1.1 Personal Information

```json
{
  "member": {
    "memberId": "string (UUID)",
    "nationalId": "string (country-specific format)",
    "personalInfo": {
      "firstName": "string",
      "middleName": "string (optional)",
      "lastName": "string",
      "dateOfBirth": "date (YYYY-MM-DD)",
      "gender": "enum (MALE, FEMALE, OTHER, UNKNOWN)",
      "maritalStatus": "enum (SINGLE, MARRIED, DIVORCED, WIDOWED, SEPARATED, DOMESTIC_PARTNER)"
    },
    "contact": {
      "address": {
        "street": "string",
        "city": "string",
        "state": "string",
        "postalCode": "string",
        "country": "string (ISO 3166-1 alpha-3)"
      },
      "phone": "string (E.164 format)",
      "email": "string (RFC 5322)",
      "preferredLanguage": "string (ISO 639-1)"
    }
  }
}
```

#### 2.1.2 Coverage Information

```json
{
  "coverage": {
    "coverageId": "string (UUID)",
    "memberId": "string (UUID reference)",
    "policyNumber": "string",
    "groupNumber": "string (optional)",
    "planType": "enum (INDIVIDUAL, FAMILY, GROUP, GOVERNMENT, MEDICARE, MEDICAID)",
    "coverageLevel": "enum (BRONZE, SILVER, GOLD, PLATINUM, CATASTROPHIC, BASIC, COMPREHENSIVE)",
    "effectiveDate": "date (YYYY-MM-DD)",
    "terminationDate": "date (YYYY-MM-DD, optional)",
    "status": "enum (ACTIVE, SUSPENDED, TERMINATED, PENDING)",
    "deductible": {
      "individual": "decimal (amount)",
      "family": "decimal (amount)",
      "currency": "string (ISO 4217)"
    },
    "outOfPocketMax": {
      "individual": "decimal",
      "family": "decimal",
      "currency": "string"
    },
    "dependents": [
      {
        "dependentId": "string (UUID)",
        "relationship": "enum (SPOUSE, CHILD, PARENT, DOMESTIC_PARTNER, OTHER)",
        "firstName": "string",
        "lastName": "string",
        "dateOfBirth": "date"
      }
    ]
  }
}
```

#### 2.1.3 Pre-existing Conditions

```json
{
  "healthHistory": {
    "memberId": "string (UUID reference)",
    "conditions": [
      {
        "conditionId": "string (UUID)",
        "diagnosisCode": "string (ICD-10 or ICD-11)",
        "description": "string",
        "diagnosisDate": "date",
        "status": "enum (ACTIVE, RESOLVED, CHRONIC, MANAGED)",
        "severity": "enum (MILD, MODERATE, SEVERE, CRITICAL)"
      }
    ],
    "medications": [
      {
        "medicationId": "string (UUID)",
        "drugCode": "string (RxNorm or NDC)",
        "name": "string",
        "dosage": "string",
        "frequency": "string",
        "startDate": "date",
        "endDate": "date (optional)"
      }
    ],
    "allergies": [
      {
        "allergyId": "string (UUID)",
        "allergen": "string",
        "allergyType": "enum (DRUG, FOOD, ENVIRONMENTAL, OTHER)",
        "severity": "enum (MILD, MODERATE, SEVERE, LIFE_THREATENING)",
        "reaction": "string"
      }
    ]
  }
}
```

### 2.2 Claims Data

#### 2.2.1 Professional Claims

```json
{
  "claim": {
    "claimId": "string (UUID)",
    "claimNumber": "string (human-readable)",
    "memberId": "string (UUID reference)",
    "providerId": "string (UUID reference)",
    "claimType": "enum (PROFESSIONAL, INSTITUTIONAL, DENTAL, PHARMACY, VISION)",
    "submissionDate": "datetime (ISO 8601)",
    "serviceDate": {
      "from": "date",
      "to": "date"
    },
    "diagnosis": [
      {
        "code": "string (ICD-10/11)",
        "type": "enum (PRIMARY, SECONDARY, ADMITTING)",
        "description": "string"
      }
    ],
    "procedures": [
      {
        "procedureCode": "string (CPT, HCPCS)",
        "modifiers": ["array of strings"],
        "description": "string",
        "serviceDate": "date",
        "quantity": "integer",
        "chargedAmount": {
          "amount": "decimal",
          "currency": "string"
        },
        "allowedAmount": {
          "amount": "decimal",
          "currency": "string"
        }
      }
    ],
    "totalCharged": {
      "amount": "decimal",
      "currency": "string"
    },
    "status": "enum (SUBMITTED, IN_REVIEW, APPROVED, DENIED, PARTIAL, APPEALED, PAID)",
    "adjudication": {
      "adjudicatedDate": "datetime",
      "approvedAmount": "decimal",
      "deniedAmount": "decimal",
      "patientResponsibility": {
        "copay": "decimal",
        "coinsurance": "decimal",
        "deductible": "decimal",
        "total": "decimal"
      },
      "insurancePayment": "decimal",
      "denialReasons": [
        {
          "code": "string",
          "description": "string"
        }
      ]
    }
  }
}
```

#### 2.2.2 Institutional Claims

```json
{
  "institutionalClaim": {
    "claimId": "string (UUID)",
    "facilityId": "string (UUID reference)",
    "admissionDate": "datetime",
    "dischargeDate": "datetime",
    "admissionType": "enum (EMERGENCY, URGENT, ELECTIVE, NEWBORN, TRAUMA)",
    "admissionSource": "enum (PHYSICIAN_REFERRAL, CLINIC_REFERRAL, TRANSFER, EMERGENCY, COURT_LAW, INFORMATION_NOT_AVAILABLE)",
    "dischargeStatus": "enum (HOME, ACUTE_CARE, SKILLED_NURSING, INTERMEDIATE_CARE, HOSPICE, LEFT_AMA, DIED, DISCHARGE_TO_COURT)",
    "drgCode": "string (DRG code)",
    "roomAndBoard": [
      {
        "roomType": "enum (PRIVATE, SEMI_PRIVATE, WARD, ICU, CCU, NURSERY)",
        "fromDate": "date",
        "toDate": "date",
        "days": "integer",
        "rate": "decimal",
        "charges": "decimal"
      }
    ],
    "services": [
      {
        "revenueCode": "string",
        "description": "string",
        "hcpcsCode": "string",
        "quantity": "decimal",
        "charges": "decimal"
      }
    ]
  }
}
```

### 2.3 Provider Network Data

#### 2.3.1 Provider Profile

```json
{
  "provider": {
    "providerId": "string (UUID)",
    "npi": "string (National Provider Identifier)",
    "providerType": "enum (PHYSICIAN, HOSPITAL, CLINIC, PHARMACY, LAB, IMAGING, DME, MENTAL_HEALTH, DENTAL, VISION)",
    "demographics": {
      "name": "string",
      "organizationName": "string (for facilities)",
      "taxId": "string",
      "address": {
        "street": "string",
        "city": "string",
        "state": "string",
        "postalCode": "string",
        "country": "string"
      },
      "phone": "string",
      "fax": "string",
      "email": "string",
      "website": "string"
    },
    "credentials": {
      "licenseNumber": "string",
      "licenseState": "string",
      "licenseExpiration": "date",
      "boardCertifications": [
        {
          "specialty": "string",
          "boardName": "string",
          "certificationDate": "date",
          "expirationDate": "date"
        }
      ],
      "deaNumber": "string (for prescribers)",
      "medicareNumber": "string",
      "medicaidNumber": "string"
    },
    "specialties": [
      {
        "specialtyCode": "string (taxonomy code)",
        "specialtyName": "string",
        "isPrimary": "boolean"
      }
    ],
    "networkStatus": {
      "status": "enum (IN_NETWORK, OUT_OF_NETWORK, PENDING, TERMINATED)",
      "effectiveDate": "date",
      "terminationDate": "date (optional)",
      "contractId": "string",
      "paymentTier": "string"
    },
    "qualityMetrics": {
      "patientSatisfactionScore": "decimal (1-5)",
      "numberOfPatients": "integer",
      "numberOfReviews": "integer",
      "hedisScores": {
        "measure": "string",
        "score": "decimal"
      },
      "accreditations": [
        {
          "accreditingBody": "string",
          "accreditationType": "string",
          "effectiveDate": "date",
          "expirationDate": "date"
        }
      ]
    }
  }
}
```

### 2.4 Premium and Payment Data

#### 2.4.1 Premium Calculation

```json
{
  "premium": {
    "premiumId": "string (UUID)",
    "memberId": "string (UUID reference)",
    "billingPeriod": {
      "from": "date",
      "to": "date"
    },
    "basePremium": "decimal",
    "riskAdjustments": {
      "ageFactor": "decimal",
      "genderFactor": "decimal",
      "geographicFactor": "decimal",
      "healthStatusFactor": "decimal",
      "tobaccoSurcharge": "decimal"
    },
    "subsidies": {
      "governmentSubsidy": "decimal",
      "employerContribution": "decimal",
      "otherSubsidies": "decimal"
    },
    "totalPremium": "decimal",
    "memberResponsibility": "decimal",
    "paymentFrequency": "enum (MONTHLY, QUARTERLY, SEMI_ANNUAL, ANNUAL)",
    "currency": "string"
  }
}
```

#### 2.4.2 Payment Transaction

```json
{
  "payment": {
    "paymentId": "string (UUID)",
    "paymentType": "enum (PREMIUM, CLAIM, REFUND, ADJUSTMENT)",
    "payerId": "string (UUID)",
    "payeeId": "string (UUID)",
    "amount": "decimal",
    "currency": "string",
    "paymentDate": "date",
    "paymentMethod": "enum (CREDIT_CARD, DEBIT_CARD, ACH, WIRE_TRANSFER, CHECK, CASH)",
    "transactionId": "string",
    "status": "enum (PENDING, COMPLETED, FAILED, REVERSED, REFUNDED)",
    "referenceNumber": "string"
  }
}
```

### 2.5 Cross-Border Healthcare Data

#### 2.5.1 International Coverage

```json
{
  "crossBorderCoverage": {
    "coverageId": "string (UUID)",
    "memberId": "string (UUID reference)",
    "homeCountry": "string (ISO 3166-1 alpha-3)",
    "coveredCountries": ["array of country codes"],
    "emergencyCoverageOnly": "boolean",
    "prePlannedCareCoverage": "boolean",
    "coverageLimits": {
      "annualLimit": "decimal",
      "perVisitLimit": "decimal",
      "daysPerYear": "integer"
    },
    "reciprocalAgreements": [
      {
        "partnerCountry": "string",
        "agreementType": "enum (FULL_RECIPROCITY, LIMITED, EMERGENCY_ONLY)",
        "effectiveDate": "date",
        "expirationDate": "date"
      }
    ],
    "travelAssistance": {
      "emergencyEvacuation": "boolean",
      "medicalRepatriation": "boolean",
      "translationServices": "boolean",
      "legalAssistance": "boolean"
    }
  }
}
```

---

## 3. Data Encoding Standards

### 3.1 Medical Coding Systems

- **Diagnoses**: ICD-10-CM (US), ICD-10 (International), ICD-11 (WHO)
- **Procedures**: CPT (US), HCPCS (US), ICD-10-PCS (US Inpatient), ICHI (International)
- **Medications**: RxNorm, NDC (US), ATC (International)
- **Lab Tests**: LOINC
- **Clinical Terms**: SNOMED CT

### 3.2 Administrative Code Sets

- **Provider Taxonomy**: NUCC Health Care Provider Taxonomy
- **Place of Service**: CMS Place of Service Codes
- **Revenue Codes**: UB-04 Revenue Codes
- **DRG**: MS-DRG (Medicare Severity), APR-DRG (All Patient Refined)

### 3.3 Geographic Coding

- **Countries**: ISO 3166-1 alpha-3
- **States/Provinces**: ISO 3166-2
- **Postal Codes**: Country-specific formats
- **Coordinates**: WGS84 decimal degrees

---

## 4. File Formats and Serialization

### 4.1 Supported Formats

- **JSON**: Primary format for APIs and modern systems
- **XML**: Secondary format for legacy system compatibility
- **EDI X12**: Healthcare transactions (837, 835, 270/271, 276/277, 278)
- **HL7 FHIR**: Modern healthcare interoperability standard
- **CSV**: Bulk data export and reporting

### 4.2 Character Encoding

- **Primary**: UTF-8
- **Acceptable**: UTF-16 (with BOM)
- **Legacy**: ISO-8859-1 (discouraged, conversion required)

### 4.3 Date and Time Formats

- **Dates**: ISO 8601 (YYYY-MM-DD)
- **Times**: ISO 8601 (HH:MM:SS with timezone)
- **DateTime**: ISO 8601 (YYYY-MM-DDTHH:MM:SS±HH:MM)

---

## 5. Data Quality Requirements

### 5.1 Validation Rules

- All required fields must be present
- Data types must match specifications
- Code values must exist in referenced code sets
- Cross-references must resolve to valid entities
- Dates must be logical (e.g., service date before claim submission)

### 5.2 Data Integrity

- Primary keys must be unique
- Foreign keys must reference existing records
- Referential integrity must be maintained
- Audit trails must track all changes

### 5.3 Data Retention

- Active member data: Maintained while coverage is active
- Claims data: Minimum 7 years after service date
- Payment records: Minimum 7 years for tax purposes
- Audit logs: Minimum 3 years

---

## 6. Privacy and Security

### 6.1 Personally Identifiable Information (PII)

Sensitive fields requiring protection:
- National ID numbers
- Full names and dates of birth
- Contact information
- Health conditions and diagnoses
- Payment information

### 6.2 Encryption Requirements

- Data at rest: AES-256
- Data in transit: TLS 1.3 or higher
- Key management: FIPS 140-2 compliant

### 6.3 Access Controls

- Role-based access control (RBAC)
- Principle of least privilege
- Multi-factor authentication for sensitive operations
- Audit logging of all data access

---

## 7. Version Control and Change Management

### 7.1 Schema Versioning

- Major version: Breaking changes to data structure
- Minor version: Backward-compatible additions
- Patch version: Bug fixes and clarifications

### 7.2 Backward Compatibility

- New fields are optional for one version cycle
- Deprecated fields remain supported for two version cycles
- Migration guides provided for breaking changes

### 7.3 Change Documentation

- All schema changes documented in changelog
- Migration scripts provided for database changes
- API version negotiation supported

---

## 8. Implementation Guidance

### 8.1 Database Schema

Recommended approach:
- Relational database (PostgreSQL, MySQL) for transactional data
- Document store (MongoDB, CouchDB) for flexible schemas
- Data warehouse (Snowflake, BigQuery) for analytics
- Cache layer (Redis) for high-frequency lookups

### 8.2 API Design

- RESTful endpoints for CRUD operations
- GraphQL for flexible queries
- WebSocket for real-time updates
- Batch APIs for bulk operations

### 8.3 Testing and Validation

- Unit tests for data validation logic
- Integration tests for cross-system data exchange
- Performance tests for high-volume scenarios
- Security tests for vulnerability assessment

---

## 9. Compliance and Standards Alignment

### 9.1 Regulatory Compliance

- HIPAA (US) - Privacy and Security Rules
- GDPR (EU) - Data protection and privacy
- PIPEDA (Canada) - Personal information protection
- Local data protection laws

### 9.2 Industry Standards

- HL7 FHIR Release 4
- DICOM for medical imaging
- IHE Integration Profiles
- ISO/HL7 27931:2009 - Data quality

---

## 10. Appendices

### Appendix A: Sample Data

See `/examples` directory for complete sample datasets.

### Appendix B: Code Value Sets

See `/valuesets` directory for complete code value definitions.

### Appendix C: Validation Schemas

JSON Schema and XML Schema definitions available in `/schemas`.

---

**Document Control**

- **Author**: WIA Standards Committee
- **Version**: 1.0
- **Date**: 2025-12-26
- **Status**: Published
- **Next Review**: 2026-12-26

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
