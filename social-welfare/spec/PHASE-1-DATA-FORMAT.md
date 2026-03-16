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
