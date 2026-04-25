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

---

## Annex A — Conformance Tier Matrix

WIA conformance for pension-system is evaluated across three tiers:

| Tier | Scope | Mandatory artifacts | Audit cadence |
|------|-------|--------------------|----------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | OpenAPI 3.0 contract, JSON Schema validation report, security threat model | Annual self-review |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Tier 1 artifacts + signed third-party assessor report against this PHASE | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Tier 2 artifacts + WIA accreditation, ISO/IEC 17065:2012 conformity assessment, evidence retention ≥ 7 years | Every 12 months |

Implementations MUST disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page. Tier downgrade events MUST be reported to the WIA registry within 30 days.

---

## Annex B — Cross-Walk to International Standards

The PHASE specification reuses or normatively references published standards alongside this document. Implementers SHOULD review the relevant standards alongside this PHASE document; where a conflict exists, the more specific WIA requirement governs unless explicitly superseded by a binding national regulation.

- ISO/IEC 17065:2012 — Conformity assessment — Requirements for bodies certifying products, processes and services
- ISO/IEC 27001:2022 — Information security management systems
- IETF RFC 9457 — Problem Details for HTTP APIs
- IETF RFC 7519 — JSON Web Token (JWT)
- IETF RFC 6749 — The OAuth 2.0 Authorization Framework
- W3C PROV-DM — provenance data model

This cross-walk is informative only. WIA does not republish the referenced documents; readers MUST obtain authoritative copies from the issuing body. Cross-walk entries are reviewed at every minor version of this PHASE.

---

## Annex C — Reference Implementations and Test Vectors

### C.1 Reference Implementations

WIA does not require implementers to use a particular library, but maintains pointers to the canonical reference implementation directories under the WIA-Official GitHub organization:

- `wia-standards/standards/pension-system/api/` — TypeScript SDK skeleton
- `wia-standards/standards/pension-system/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/pension-system/simulator/` — interactive browser-based simulator for the PHASE protocol

Each reference artifact ships with an MIT license and is intended as a starting point, not as a production-grade implementation.

### C.2 Test Vectors

A normative set of request/response pairs covering the schemas defined in this PHASE is published alongside this document. Implementations claiming Tier 2 or Tier 3 conformance MUST pass every published test vector in both serialization (request) and deserialization (response) directions, and MUST publish a signed report identifying which vectors were exercised.

Test vectors are versioned independently of the PHASE document; refer to the `test-vectors/` directory for the active version.

---

## Annex D — Open Questions and Future Work

This PHASE document captures the consensus position at v1.0. The following items are tracked for future minor or major revisions:

1. **Schema versioning policy** — formal MUST/SHOULD rules for backward-compatible vs. breaking changes between minor releases.
2. **Privacy-preserving aggregation** — guidance on differential privacy and secure aggregation patterns for telemetry channels, without prescribing a specific algorithm.
3. **Multilingual error catalogs** — localization strategy for the standard error codes defined in this PHASE.
4. **Long-term retention** — alignment with sectoral retention regulations across jurisdictions.
5. **Sustainability disclosure** — optional fields for energy and emissions reporting tied to operations covered by this PHASE.

Items in this annex are non-normative. Comments and proposals are accepted via the GitHub issues tracker on the WIA-Official organization.

---


## Annex E — Implementation Notes for PHASE-1-DATA-FORMAT

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-1-DATA-FORMAT.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

