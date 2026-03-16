# WIA-SOC-018 Phase 1: Pension Data Format Specification

**Version:** 1.0.0  
**Status:** Approved  
**Last Updated:** 2025-12-26

弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. Overview

Phase 1 defines the standardized data formats for pension systems, including member records, contribution data, benefit calculations, and fund information. All data MUST use JSON-LD format for semantic interoperability and blockchain anchoring for immutability.

## 2. Core Data Types

### 2.1 Member Identity

```json
{
  "@context": "https://wiastandards.com/soc-018/v1",
  "@type": "PensionMember",
  "memberId": "WIA-PENSION-GLOBAL-UUID",
  "personalInfo": {
    "firstName": "string",
    "lastName": "string",
    "dateOfBirth": "ISO8601 date",
    "citizenship": "ISO 3166-1 alpha-2",
    "taxId": "string",
    "socialSecurityNumber": "encrypted string"
  },
  "contactInfo": {
    "email": "string",
    "phone": "E.164 format",
    "address": {
      "street": "string",
      "city": "string",
      "state": "string",
      "postalCode": "string",
      "country": "ISO 3166-1 alpha-2"
    }
  },
  "enrollmentDate": "ISO8601 datetime",
  "status": "active|inactive|retired|deceased",
  "preferences": {
    "language": "ISO 639-1",
    "communicationMethod": "email|sms|mail|app",
    "investmentRiskProfile": "conservative|moderate|aggressive"
  }
}
```

### 2.2 Contribution Record

```json
{
  "@type": "PensionContribution",
  "contributionId": "UUID",
  "memberId": "WIA-PENSION-GLOBAL-UUID",
  "timestamp": "ISO8601 datetime",
  "contributionPeriod": {
    "startDate": "ISO8601 date",
    "endDate": "ISO8601 date"
  },
  "employerInfo": {
    "employerId": "UUID",
    "employerName": "string",
    "employerTaxId": "string",
    "jurisdiction": "ISO 3166-1 alpha-2"
  },
  "earnings": {
    "grossEarnings": "decimal",
    "pensionableEarnings": "decimal",
    "currency": "ISO 4217"
  },
  "contributions": {
    "employeeAmount": "decimal",
    "employeeRate": "decimal (percentage)",
    "employerAmount": "decimal",
    "employerRate": "decimal (percentage)",
    "totalContribution": "decimal"
  },
  "pensionScheme": {
    "schemeId": "UUID",
    "schemeName": "string",
    "schemeType": "defined_benefit|defined_contribution|hybrid"
  },
  "paymentInfo": {
    "paymentMethod": "ach|wire|crypto|other",
    "paymentDate": "ISO8601 date",
    "paymentReference": "string"
  },
  "validationStatus": {
    "status": "pending|validated|rejected|adjusted",
    "validatedAt": "ISO8601 datetime",
    "validatedBy": "string",
    "notes": "string"
  },
  "blockchainAnchor": {
    "txHash": "string",
    "blockNumber": "integer",
    "network": "ethereum|polygon|other"
  }
}
```

### 2.3 Benefit Calculation

```json
{
  "@type": "PensionBenefitCalculation",
  "calculationId": "UUID",
  "memberId": "WIA-PENSION-GLOBAL-UUID",
  "calculationDate": "ISO8601 datetime",
  "retirementAge": "integer",
  "serviceCreditYears": "decimal",
  "earningsHistory": {
    "finalAverageSalary": "decimal",
    "careerAverageEarnings": "decimal",
    "averagePeriodYears": "integer",
    "currency": "ISO 4217"
  },
  "benefitFormula": {
    "accrualRate": "decimal (percentage)",
    "multiplier": "decimal",
    "integrationOffset": "decimal",
    "colaAdjustment": "decimal (percentage)"
  },
  "calculatedBenefits": {
    "monthlyBenefit": "decimal",
    "annualBenefit": "decimal",
    "lumpsumOption": "decimal",
    "replacementRate": "decimal (percentage)"
  },
  "adjustments": {
    "earlyRetirementReduction": "decimal (percentage)",
    "delayedRetirementCredit": "decimal (percentage)",
    "survivorBenefitReduction": "decimal (percentage)",
    "socialSecurityOffset": "decimal"
  },
  "projections": [
    {
      "age": "integer",
      "monthlyBenefit": "decimal",
      "cumulativeBenefit": "decimal",
      "adjustmentFactors": "object"
    }
  ]
}
```

### 2.4 Fund Allocation

```json
{
  "@type": "PensionFundAllocation",
  "allocationId": "UUID",
  "memberId": "WIA-PENSION-GLOBAL-UUID",
  "effectiveDate": "ISO8601 date",
  "totalBalance": {
    "amount": "decimal",
    "currency": "ISO 4217"
  },
  "assetAllocation": [
    {
      "assetClass": "domestic_equity|international_equity|fixed_income|real_estate|alternatives",
      "allocationPercentage": "decimal (percentage)",
      "currentValue": "decimal",
      "numberOfUnits": "decimal",
      "unitPrice": "decimal",
      "returnYTD": "decimal (percentage)",
      "returnAnnualized": "decimal (percentage)"
    }
  ],
  "riskMetrics": {
    "standardDeviation": "decimal",
    "sharpeRatio": "decimal",
    "beta": "decimal",
    "var95": "decimal"
  },
  "rebalancingRules": {
    "method": "calendar|threshold|tactical",
    "frequency": "monthly|quarterly|annually",
    "thresholdPercentage": "decimal"
  }
}
```

## 3. Data Validation Rules

### 3.1 Member Data Validation

- **memberId**: MUST be globally unique UUID
- **dateOfBirth**: MUST be valid date, age >= 18 and <= 120
- **taxId**: MUST conform to jurisdiction-specific format
- **email**: MUST be valid RFC 5322 email address
- **phone**: MUST be valid E.164 format

### 3.2 Contribution Data Validation

- **contributionAmount**: MUST be >= 0.01 in contribution currency
- **contributionRate**: MUST be between 0 and 100 (percentage)
- **pensionableEarnings**: MUST be <= grossEarnings
- **totalContribution**: MUST equal employeeAmount + employerAmount
- **timestamp**: MUST NOT be future dated

### 3.3 Benefit Calculation Validation

- **serviceCreditYears**: MUST be >= 0 and <= 60
- **accrualRate**: MUST be between 0.01 and 0.05 (1-5%)
- **monthlyBenefit**: MUST be > 0
- **replacementRate**: MUST be between 0 and 1.5 (0-150%)

## 4. Data Security Requirements

### 4.1 Encryption Standards

- **At Rest**: AES-256 encryption for all PII
- **In Transit**: TLS 1.3 minimum for all data transmissions
- **Sensitive Fields**: Additional field-level encryption for SSN, tax IDs

### 4.2 Access Control

- **Authentication**: Multi-factor authentication required
- **Authorization**: Role-based access control (RBAC)
- **Audit Trail**: All data access logged with user, timestamp, action

### 4.3 Data Retention

- **Active Members**: Indefinite retention
- **Retired Members**: Lifetime + 100 years
- **Deceased Members**: 50 years post-death
- **Audit Logs**: 10 years minimum

## 5. Blockchain Integration

### 5.1 Contribution Anchoring

Every contribution record MUST be anchored to blockchain:

```json
{
  "contributionHash": "SHA-256 hash of contribution record",
  "blockchainTx": {
    "network": "ethereum|polygon|hyperledger",
    "txHash": "blockchain transaction hash",
    "blockNumber": "integer",
    "timestamp": "ISO8601 datetime",
    "gasUsed": "integer (optional)"
  }
}
```

### 5.2 Verification Process

1. Generate SHA-256 hash of contribution record (excluding blockchainAnchor field)
2. Submit hash to blockchain smart contract
3. Record transaction hash and block number
4. Enable anyone to verify contribution authenticity

## 6. Interoperability Standards

### 6.1 JSON-LD Context

All pension data MUST include JSON-LD context for semantic web compatibility:

```json
{
  "@context": {
    "@vocab": "https://wiastandards.com/soc-018/v1/",
    "xsd": "http://www.w3.org/2001/XMLSchema#",
    "member": {
      "@id": "member",
      "@type": "@id"
    },
    "contribution": {
      "@id": "contribution",
      "@type": "@id"
    },
    "amount": {
      "@id": "amount",
      "@type": "xsd:decimal"
    }
  }
}
```

### 6.2 Data Exchange Format

Standard data exchange MUST use:
- **Format**: JSON-LD with RDF serialization
- **Transport**: HTTPS REST APIs or message queues (AMQP, Kafka)
- **Compression**: gzip or brotli compression supported
- **Character Encoding**: UTF-8

## 7. Implementation Compliance

Systems claiming WIA-SOC-018 Phase 1 compliance MUST:

1. Implement all core data types (sections 2.1-2.4)
2. Enforce all validation rules (section 3)
3. Meet security requirements (section 4)
4. Support blockchain anchoring (section 5)
5. Provide JSON-LD formatted data (section 6)
6. Pass compliance test suite (available at wiastandards.com)

---

© 2025 WIA · MIT License · 弘益人間 (Benefit All Humanity)
