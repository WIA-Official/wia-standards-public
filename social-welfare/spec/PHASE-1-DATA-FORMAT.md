# WIA Social Welfare Standard
## Phase 1: Data Format Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-12-26
**Author:** WIA Technical Committee
**Standard ID:** WIA-SOC-017

---

## Table of Contents

1. [Introduction](#introduction)
2. [Welfare Program Data Model](#welfare-program-data-model)
3. [JSON Schema Definitions](#json-schema-definitions)
4. [Benefit Calculation](#benefit-calculation)
5. [Eligibility Verification](#eligibility-verification)
6. [Case Management Records](#case-management-records)
7. [Validation Rules](#validation-rules)
8. [Example Payloads](#example-payloads)
9. [Privacy and Security](#privacy-and-security)
10. [Error Handling](#error-handling)

---

## 1. Introduction

This specification defines the standardized data format for social welfare program management, benefit distribution, needs assessment, case management, and fraud prevention within the WIA ecosystem.

### 1.1 Design Principles

1. **Privacy First**: Protect sensitive personal information through encryption and access controls
2. **Interoperability**: Enable seamless data exchange between welfare agencies
3. **Accuracy**: Ensure precise benefit calculations and eligibility determinations
4. **Auditability**: Maintain comprehensive records for compliance and oversight
5. **Accessibility**: Support multiple languages and accessibility requirements

### 1.2 Core Philosophy

**弘益人間 (Hongik Ingan)** - Benefit All Humanity

This standard ensures that welfare services are delivered efficiently, equitably, and with respect for beneficiary dignity.

### 1.3 Scope

This specification covers:
- Beneficiary profile data
- Program eligibility criteria
- Benefit calculation methods
- Case management records
- Fraud detection data
- Outcome measurement metrics

---

## 2. Welfare Program Data Model

### 2.1 Core Data Types

#### 2.1.1 BeneficiaryProfile

```typescript
interface BeneficiaryProfile {
  /** Unique identifier */
  beneficiaryId: string;
  
  /** Personal information */
  personalInfo: PersonalInformation;
  
  /** Household composition */
  household: HouseholdComposition;
  
  /** Financial information */
  financial: FinancialStatus;
  
  /** Employment status */
  employment: EmploymentStatus;
  
  /** Health and disability */
  health?: HealthInformation;
  
  /** Contact information */
  contact: ContactInformation;
  
  /** Privacy preferences */
  privacy: PrivacySettings;
  
  /** Metadata */
  metadata: RecordMetadata;
}
```

#### 2.1.2 WelfareProgram

```typescript
interface WelfareProgram {
  /** Program identifier */
  programId: string;
  
  /** Program name */
  name: string;
  
  /** Program type */
  type: ProgramType;
  
  /** Eligibility criteria */
  eligibility: EligibilityCriteria;
  
  /** Benefit structure */
  benefits: BenefitStructure;
  
  /** Application process */
  application: ApplicationProcess;
  
  /** Recertification requirements */
  recertification: RecertificationRules;
  
  /** Program status */
  status: ProgramStatus;
}

enum ProgramType {
  FOOD_ASSISTANCE = "FOOD_ASSISTANCE",
  HOUSING_SUPPORT = "HOUSING_SUPPORT",
  MEDICAL_AID = "MEDICAL_AID",
  CHILDCARE = "CHILDCARE",
  DISABILITY = "DISABILITY",
  UNEMPLOYMENT = "UNEMPLOYMENT",
  ELDERLY_CARE = "ELDERLY_CARE",
  EDUCATION = "EDUCATION",
  EMERGENCY_AID = "EMERGENCY_AID"
}
```

#### 2.1.3 BenefitApplication

```typescript
interface BenefitApplication {
  /** Application identifier */
  applicationId: string;
  
  /** Beneficiary reference */
  beneficiaryId: string;
  
  /** Program reference */
  programId: string;
  
  /** Application timestamp */
  submittedAt: Timestamp;
  
  /** Application data */
  data: ApplicationData;
  
  /** Supporting documents */
  documents: Document[];
  
  /** Status */
  status: ApplicationStatus;
  
  /** Decision */
  decision?: EligibilityDecision;
  
  /** Processing history */
  history: ProcessingHistory[];
}

enum ApplicationStatus {
  SUBMITTED = "SUBMITTED",
  UNDER_REVIEW = "UNDER_REVIEW",
  PENDING_VERIFICATION = "PENDING_VERIFICATION",
  APPROVED = "APPROVED",
  DENIED = "DENIED",
  WITHDRAWN = "WITHDRAWN"
}
```

---

## 3. JSON Schema Definitions

### 3.1 Financial Status

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "type": "object",
  "properties": {
    "monthlyIncome": {
      "type": "number",
      "minimum": 0,
      "description": "Total monthly income in USD"
    },
    "incomeSource": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "source": {
            "type": "string",
            "enum": ["EMPLOYMENT", "SELF_EMPLOYMENT", "SOCIAL_SECURITY", "PENSION", "INVESTMENTS", "OTHER"]
          },
          "amount": {
            "type": "number",
            "minimum": 0
          },
          "frequency": {
            "type": "string",
            "enum": ["WEEKLY", "BIWEEKLY", "MONTHLY", "ANNUALLY"]
          }
        },
        "required": ["source", "amount", "frequency"]
      }
    },
    "assets": {
      "type": "object",
      "properties": {
        "savings": { "type": "number", "minimum": 0 },
        "realEstate": { "type": "number", "minimum": 0 },
        "vehicles": { "type": "number", "minimum": 0 },
        "investments": { "type": "number", "minimum": 0 },
        "otherAssets": { "type": "number", "minimum": 0 }
      }
    }
  },
  "required": ["monthlyIncome", "incomeSource"]
}
```

---

## 4. Benefit Calculation

### 4.1 Calculation Formula

```typescript
interface BenefitCalculation {
  /** Program identifier */
  programId: string;
  
  /** Household size */
  householdSize: number;
  
  /** Gross monthly income */
  grossIncome: number;
  
  /** Allowable deductions */
  deductions: Deductions;
  
  /** Net income */
  netIncome: number;
  
  /** Income limit */
  incomeLimit: number;
  
  /** Maximum benefit */
  maximumBenefit: number;
  
  /** Calculated benefit */
  benefitAmount: number;
  
  /** Calculation method */
  method: CalculationMethod;
}

interface Deductions {
  earnedIncomeDeduction: number;
  dependentCare: number;
  medicalExpenses: number;
  housingCosts: number;
  standardDeduction: number;
  total: number;
}
```

### 4.2 Example Calculation

```typescript
function calculateBenefit(input: CalculationInput): BenefitCalculation {
  const netIncome = input.grossIncome - calculateDeductions(input);
  const incomePercentage = netIncome / input.incomeLimit;
  const reductionFactor = 0.3 * incomePercentage;
  const benefitAmount = Math.max(0, input.maximumBenefit * (1 - reductionFactor));
  
  return {
    programId: input.programId,
    householdSize: input.householdSize,
    grossIncome: input.grossIncome,
    deductions: calculateDeductions(input),
    netIncome,
    incomeLimit: input.incomeLimit,
    maximumBenefit: input.maximumBenefit,
    benefitAmount: Math.round(benefitAmount),
    method: "GRADUATED_BENEFIT"
  };
}
```

---

## 5. Eligibility Verification

### 5.1 Verification Data

```typescript
interface VerificationData {
  /** Verification type */
  type: VerificationType;
  
  /** Data source */
  source: DataSource;
  
  /** Verification result */
  result: VerificationResult;
  
  /** Timestamp */
  verifiedAt: Timestamp;
  
  /** Validity period */
  validUntil?: Timestamp;
  
  /** Verification details */
  details: Record<string, any>;
}

enum VerificationType {
  IDENTITY = "IDENTITY",
  INCOME = "INCOME",
  EMPLOYMENT = "EMPLOYMENT",
  RESIDENCY = "RESIDENCY",
  CITIZENSHIP = "CITIZENSHIP",
  DISABILITY = "DISABILITY",
  HOUSEHOLD_COMPOSITION = "HOUSEHOLD_COMPOSITION"
}

enum VerificationResult {
  VERIFIED = "VERIFIED",
  UNVERIFIED = "UNVERIFIED",
  PENDING = "PENDING",
  FAILED = "FAILED",
  REQUIRES_MANUAL_REVIEW = "REQUIRES_MANUAL_REVIEW"
}
```

---

## 6. Case Management Records

### 6.1 Case Record

```typescript
interface CaseRecord {
  /** Case identifier */
  caseId: string;
  
  /** Beneficiary reference */
  beneficiaryId: string;
  
  /** Case manager */
  caseManager: StaffReference;
  
  /** Enrolled programs */
  programs: ProgramEnrollment[];
  
  /** Case notes */
  notes: CaseNote[];
  
  /** Action items */
  actionItems: ActionItem[];
  
  /** Case status */
  status: CaseStatus;
  
  /** Opened date */
  openedAt: Timestamp;
  
  /** Closed date */
  closedAt?: Timestamp;
}

interface CaseNote {
  noteId: string;
  author: StaffReference;
  timestamp: Timestamp;
  category: NoteCategory;
  content: string;
  confidential: boolean;
}
```

---

## 7. Validation Rules

### 7.1 Data Validation

```typescript
const validationRules = {
  personalInfo: {
    ssn: /^\d{3}-\d{2}-\d{4}$/,
    phone: /^\+?1?\d{10,}$/,
    email: /^[^\s@]+@[^\s@]+\.[^\s@]+$/,
    zipCode: /^\d{5}(-\d{4})?$/
  },
  financial: {
    income: { min: 0, max: 1000000 },
    assets: { min: 0, max: 10000000 }
  },
  household: {
    size: { min: 1, max: 20 }
  }
};
```

---

## 8. Example Payloads

### 8.1 Benefit Application

```json
{
  "applicationId": "APP-2025-001234",
  "beneficiaryId": "BEN-789456",
  "programId": "PROG-FOOD-001",
  "submittedAt": "2025-01-15T10:30:00Z",
  "data": {
    "household": {
      "size": 4,
      "members": [
        {
          "name": "Jane Doe",
          "relationship": "APPLICANT",
          "age": 35,
          "disability": false
        },
        {
          "name": "John Doe",
          "relationship": "SPOUSE",
          "age": 37,
          "disability": false
        },
        {
          "name": "Child One",
          "relationship": "CHILD",
          "age": 8,
          "disability": false
        },
        {
          "name": "Child Two",
          "relationship": "CHILD",
          "age": 5,
          "disability": false
        }
      ]
    },
    "financial": {
      "monthlyIncome": 2400,
      "incomeSource": [
        {
          "source": "EMPLOYMENT",
          "amount": 2400,
          "frequency": "MONTHLY"
        }
      ]
    }
  },
  "status": "SUBMITTED"
}
```

---

## 9. Privacy and Security

### 9.1 Data Protection

All personal information must be:
- Encrypted at rest (AES-256)
- Encrypted in transit (TLS 1.3+)
- Access-controlled (role-based)
- Audit-logged (all access recorded)
- Retention-limited (per data retention policy)

### 9.2 PII Handling

Personally Identifiable Information (PII) includes:
- Social Security Numbers
- Date of Birth
- Full Legal Names
- Addresses
- Contact Information
- Medical Information
- Financial Information

---

## 10. Error Handling

### 10.1 Error Response Format

```json
{
  "error": {
    "code": "ELIGIBILITY_VERIFICATION_FAILED",
    "message": "Income verification could not be completed",
    "details": {
      "field": "financial.monthlyIncome",
      "reason": "Source system unavailable",
      "retryable": true
    },
    "timestamp": "2025-01-15T10:30:00Z",
    "requestId": "REQ-123456"
  }
}
```

---

弘益人間 (Hongik Ingan) - Benefit All Humanity

© 2025 SmileStory Inc. / WIA

## P.1 Data Format Cross-References

This Phase defines the canonical data types referenced by the API surface (Phase 2),
the wire protocol (Phase 3), and integration scenarios (Phase 4). Implementations
MUST round-trip every canonical type through serialization and deserialization
without loss of precision or semantics.

### P.1.1 Canonical Encoding Rules

1. UTF-8 is the required character encoding for textual fields.
2. Numeric fields use IEEE 754 binary64 unless explicitly marked as fixed-point.
3. Timestamps use RFC 3339 with timezone offset; durations use ISO 8601.
4. UUIDs follow RFC 4122 v4 unless deterministic IDs are required.
5. Binary payloads are encoded as Base64 (RFC 4648 §4) in JSON contexts and as
   raw octet strings in Protocol Buffers / CBOR contexts.

### P.1.2 Schema Evolution

Schema changes follow these compatibility classes:

| Class | Allowed Changes | Wire-Compat |
|-------|-----------------|-------------|
| Patch | Doc fixes, examples, validator tightening within existing range | Forward & backward |
| Minor | New optional fields, new enum values with default fallback        | Forward |
| Major | Field rename, type change, removal, semantics change              | None |

### P.1.3 Validation Order

Validators MUST apply checks in this order: (1) syntactic well-formedness,
(2) schema conformance, (3) cross-field invariants, (4) external referential
integrity, (5) policy / authorization. A failure short-circuits subsequent
checks; the response message identifies the first failing rule by ID.


---

## Appendix · Common WIA Standard Provisions

> The following provisions apply to every WIA standard and are kept in sync
> across the WIA-Standards corpus. Standard-specific deviations, where they
> exist, are listed in the standard's normative body.

### A. Conformance & Compliance

#### A.1 Conformance Levels

WIA-Standards defines four conformance levels:

| Level | Required | Description |
|-------|----------|-------------|
| L1 — Format | Phase 1 | Implementation produces and consumes the canonical data format losslessly |
| L2 — Interface | Phase 1 + 2 | Implementation exposes the API surface with required behaviour |
| L3 — Protocol | Phase 1 + 2 + 3 | Implementation interoperates over at least one normative transport binding |
| L4 — Integration | All Phases | Implementation passes the conformance test suite end-to-end in a production-shaped deployment |

Conformance claims MUST cite the level and the version of the standard
against which the claim is made (e.g. "L3 conformant against v1.0").

#### A.2 Compliance Verification

The conformance test suite is published alongside this standard at
`/cli/conformance/` and `/api/conformance/`. Implementations claiming
L2 or higher MUST publish their test report. Independent re-tests are
encouraged; the WIA Working Group accepts third-party verification reports
under the policy in §E.

### B. Security Considerations

#### B.1 Threat Model

Implementers SHOULD apply STRIDE analysis covering: spoofing of identity,
tampering with messages or stored state, repudiation of operations,
information disclosure, denial of service, and elevation of privilege.

| Threat | Default Control | Where Strengthened |
|--------|-----------------|--------------------|
| Spoofing | Mutual TLS or signed tokens | Phase 3 §P.3 |
| Tampering | TLS in transit, AEAD at rest | Phase 1 §P.1 |
| Repudiation | Append-only audit log with notarization | Phase 4 §P.4 |
| Disclosure | Field-level encryption for PII / secrets | Phase 1 §P.1 |
| DoS | Rate limit per principal & global circuit breaker | Phase 2 §P.2 |
| EoP | Least-privilege RBAC + scoped tokens | Phase 2 §P.2 |

#### B.2 Cryptographic Suites

Mandatory: TLS 1.3 with AEAD ciphers (AES-128-GCM, AES-256-GCM,
CHACHA20-POLY1305). Forbidden: TLS 1.0, TLS 1.1, RC4, MD5, SHA-1 for
signatures, RSA below 2048 bits, ECDSA on curves smaller than P-256.

Post-quantum migration: implementations SHOULD adopt hybrid key
exchange combining a classical primitive with ML-KEM (FIPS 203) once a
profile is published; signature migration to ML-DSA (FIPS 204) is
expected within the L4 conformance window of v2.0.

#### B.3 Audit Requirements

L3 and L4 implementations MUST log: (a) every authentication decision,
(b) every authorization decision, (c) every state-changing operation,
(d) every export of data outside its sovereignty boundary. Logs are
write-once for at least 1 year and 90 days indexed for incident search.

### C. Versioning & Lifecycle

Versions follow Semantic Versioning 2.0.0 (MAJOR.MINOR.PATCH).

| Phase | Duration | Conformance Status |
|-------|---------:|-------------------|
| Draft | until ratification | Non-binding |
| Active | indefinite | Binding for new deployments |
| Maintenance | 24 months from successor's Active date | Binding for existing deployments |
| Retired | indefinite | Non-binding; conformance claims rescinded |

Deprecation MUST be announced at least one minor version before a feature
is removed in a major version.

### D. Internationalization & Accessibility

Implementations SHOULD support locale negotiation via the
`Accept-Language` header (RFC 4647). Date, time, number, and currency
formatting follow CLDR. User-facing surfaces MUST satisfy WCAG 2.1 AA at
minimum and SHOULD progress towards WCAG 2.2 AA. Right-to-left scripts
(Arabic, Hebrew, Persian, Urdu) and East-Asian wide characters MUST be
laid out correctly without line-breaking heuristics that split graphemes.

### E. Governance & IP Policy

This standard is maintained by the WIA Working Group under the WIA
governance charter. Editorial changes are merged via pull request. Normative
changes require working-group consensus and a 30-day public review.
Contributions are accepted under the Apache License 2.0 with explicit
patent grant. Members participate under the WIA Patent Policy,
which requires royalty-free licensing of any essential claim necessary
to implement a normative requirement.

### F. Normative References

The following references are normative; implementations MUST satisfy
the cited clauses:

- ISO/IEC 27001:2022 — Information security management systems
- ISO/IEC 27017:2015 — Cloud-services security controls
- ISO/IEC 27701:2019 — Privacy information management
- ISO/IEC 19790:2012 — Security requirements for cryptographic modules
- ISO 8601-1:2019 — Date and time representation
- IETF RFC 8446 — TLS 1.3
- IETF RFC 7519 — JSON Web Token
- IETF RFC 6749 — OAuth 2.0
- IETF RFC 9110 — HTTP Semantics
- IETF RFC 9112 — HTTP/1.1 message syntax
- IETF RFC 9113 — HTTP/2
- IETF RFC 9114 — HTTP/3
- IETF RFC 9000 — QUIC transport
- IETF RFC 4122 — UUID URN namespace
- IETF RFC 3339 — Date and time on the Internet
- IETF RFC 6838 — Media-type specifications and registration
- W3C TraceContext — Distributed tracing context
- W3C WCAG 2.1 — Accessibility guidelines
- FIPS PUB 197 — AES
- FIPS PUB 180-4 — SHA-2 family
- FIPS PUB 203 — ML-KEM (post-quantum KEM)
- FIPS PUB 204 — ML-DSA (post-quantum signature)

### G. Glossary

| Term | Definition |
|------|------------|
| Conformance | The state of satisfying every normative requirement at a given level |
| Implementation | A software, hardware, or composite artefact that claims conformance |
| Principal | The authenticated entity bound to a security context |
| Subject | The resource or person to which an operation applies |
| Sovereignty Boundary | The legal / regulatory perimeter outside of which data export is restricted |

---

*This Appendix is authored by the WIA Standards Working Group and is kept
in lockstep across Phases 1–4 of social-welfare so that conformance claims at any
Phase remain unambiguous.*

