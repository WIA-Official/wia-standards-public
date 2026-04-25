# WIA-SOC-018 Phase 4: Integration Specification

**Version:** 1.0.0  
**Status:** Approved  
**Last Updated:** 2025-12-26

弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. Overview

Phase 4 defines integration patterns for pension systems with external platforms including payroll systems, HR platforms, financial institutions, government agencies, and blockchain networks.

## 2. Integration Architecture

### 2.1 Integration Layers

```
┌──────────────────────────────────────────────┐
│         Presentation Layer                    │
│  (Member Portal, Mobile App, Admin UI)       │
└──────────────────────────────────────────────┘
                     ↓
┌──────────────────────────────────────────────┐
│         API Gateway Layer                     │
│  (Authentication, Rate Limiting, Routing)    │
└──────────────────────────────────────────────┘
                     ↓
┌──────────────────────────────────────────────┐
│         Business Logic Layer                  │
│  (Contribution Processing, Benefit Calc)     │
└──────────────────────────────────────────────┘
                     ↓
┌──────────────────────────────────────────────┐
│         Integration Layer                     │
│  (Adapters, Connectors, Data Transformation) │
└──────────────────────────────────────────────┘
                     ↓
┌──────────────────────────────────────────────┐
│         External Systems                      │
│  (Payroll, Banks, Blockchain, Regulators)    │
└──────────────────────────────────────────────┘
```

### 2.2 Integration Patterns

**Adapter Pattern**: Wrap external APIs with WIA-SOC-018 interface
**Event-Driven**: React to external system events
**Batch Processing**: Scheduled bulk data exchange
**Real-Time Sync**: Immediate data synchronization

## 3. Payroll System Integration

### 3.1 Payroll Data Flow

```
Employer Payroll → Contribution Submission → Pension System
                                          ↓
                                      Validation
                                          ↓
                                   Fund Allocation
                                          ↓
                               Member Account Update
```

### 3.2 Integration Methods

**Method 1: SFTP File Drop**

Payroll system drops contribution file to SFTP server:

```
/incoming/contributions/employer_{id}_{date}.json
```

File format:
```json
{
  "employer": {
    "employerId": "UUID",
    "payrollPeriod": {
      "startDate": "2025-01-01",
      "endDate": "2025-01-31"
    }
  },
  "contributions": [
    { /* Individual contribution records */ }
  ],
  "summary": {
    "totalEmployees": 500,
    "totalContributions": 300000.00,
    "checksum": "SHA-256 hash"
  }
}
```

**Method 2: REST API Integration**

Direct API calls from payroll system:

```http
POST /wia/soc-018/v1/contributions/batch
Authorization: Bearer {JWT}
Content-Type: application/json

{
  "contributions": [ /* Array of contributions */ ]
}
```

**Method 3: Message Queue**

Publish contributions to queue topic:

```
Topic: wia.soc018.payroll.contributions
Message: { /* Contribution data */ }
```

### 3.3 Payroll Reconciliation

Daily reconciliation process:

1. **Expected vs Actual**: Compare expected contributions to received
2. **Discrepancy Detection**: Identify missing or incorrect amounts
3. **Exception Handling**: Flag discrepancies for investigation
4. **Automated Follow-up**: Email/API notifications to payroll system

## 4. HR Platform Integration

### 4.1 Employee Lifecycle Events

Integrate pension enrollment with HR events:

| HR Event | Pension Action |
|----------|----------------|
| New Hire | Auto-enroll if eligible |
| Termination | Update status, calculate vesting |
| Promotion/Raise | Update contribution calculations |
| Leave of Absence | Suspend contributions, maintain benefits |
| Retirement | Initiate benefit disbursement |

### 4.2 HRIS Sync Protocol

```json
{
  "eventType": "employee.hired",
  "timestamp": "ISO8601",
  "employee": {
    "employeeId": "HR-12345",
    "personalInfo": {
      "firstName": "Jane",
      "lastName": "Smith",
      "dateOfBirth": "1990-03-15",
      "taxId": "987-65-4321"
    },
    "employment": {
      "hireDate": "2025-01-15",
      "jobTitle": "Software Engineer",
      "department": "Engineering",
      "salary": 85000.00,
      "employmentType": "full-time"
    },
    "benefits": {
      "pensionEligibilityDate": "2025-04-15",
      "defaultContributionRate": 0.06
    }
  }
}
```

### 4.3 Identity Management

Sync employee identities between HR and pension systems:

- **SSO Integration**: SAML 2.0 or OAuth 2.0
- **Directory Sync**: LDAP/Active Directory integration
- **Role Mapping**: HR roles → Pension system permissions

## 5. Financial Institution Integration

### 5.1 Bank Account Verification

Verify member bank accounts for benefit disbursement:

```http
POST /wia/soc-018/v1/verify-bank-account
Content-Type: application/json

{
  "memberId": "UUID",
  "bankAccount": {
    "accountNumber": "encrypted",
    "routingNumber": "encrypted",
    "accountType": "checking|savings",
    "accountHolderName": "string"
  }
}
```

Use micro-deposits or instant verification services (Plaid, MX).

### 5.2 Payment Initiation

Initiate pension benefit payments:

```json
{
  "paymentId": "UUID",
  "paymentType": "monthly_pension|lumpsum|partial_withdrawal",
  "memberId": "UUID",
  "amount": {
    "value": "decimal",
    "currency": "ISO 4217"
  },
  "bankAccount": {
    "accountNumber": "encrypted",
    "routingNumber": "encrypted"
  },
  "paymentMethod": "ACH|Wire|FedNow|RTP",
  "paymentDate": "ISO8601",
  "memo": "Pension payment for {month}"
}
```

### 5.3 Payment Status Tracking

```json
{
  "paymentId": "UUID",
  "status": "pending|processing|completed|failed|returned",
  "timeline": {
    "initiated": "ISO8601",
    "processed": "ISO8601",
    "settled": "ISO8601"
  },
  "confirmationNumber": "string",
  "failureReason": "insufficient_funds|invalid_account|other"
}
```

## 6. Government & Regulatory Integration

### 6.1 Tax Reporting

Annual tax reporting to IRS (1099-R in US):

```json
{
  "taxYear": 2025,
  "reportType": "1099-R",
  "pension Provider": {
    "tin": "XX-XXXXXXX",
    "name": "Pension Provider Inc",
    "address": { /* Address */ }
  },
  "recipient": {
    "ssn": "XXX-XX-XXXX",
    "name": "John Doe",
    "address": { /* Address */ }
  },
  "distributions": {
    "grossDistribution": 29400.00,
    "taxableAmount": 29400.00,
    "federalTaxWithheld": 4410.00,
    "distributionCode": "7",
    "iraRothConversion": 0.00
  }
}
```

### 6.2 Regulatory Reporting

Submit required reports to pension regulators:

- **Form 5500** (US): Annual pension plan report
- **TPR Returns** (UK): Pension scheme returns
- **Annual Compliance Statements**: Multi-jurisdiction reporting

```json
{
  "reportType": "annual_compliance",
  "reportingPeriod": {
    "startDate": "2025-01-01",
    "endDate": "2025-12-31"
  },
  "pensionScheme": {
    "schemeId": "UUID",
    "registrationNumber": "string",
    "jurisdiction": "ISO 3166-1"
  },
  "metrics": {
    "activeMembersCount": 50000,
    "retiredMembersCount": 12000,
    "totalAssets": 2500000000.00,
    "totalLiabilities": 2300000000.00,
    "fundingRatio": 1.087,
    "contributionsReceived": 250000000.00,
    "benefitsPaid": 180000000.00,
    "investmentReturn": 0.084
  }
}
```

### 6.3 Benefit Verification

Social security offices verify pension benefits:

```http
POST /wia/soc-018/v1/verify-benefits
Authorization: Government-API-Key

{
  "memberId": "UUID",
  "verificationPurpose": "social_security_offset",
  "requestedInfo": [
    "monthly_benefit_amount",
    "service_credit_years",
    "benefit_start_date"
  ]
}
```

## 7. Blockchain Integration

### 7.1 Smart Contract Interaction

Deploy and interact with pension smart contracts:

```solidity
// Simplified Ethereum smart contract
contract PensionContribution {
    mapping(bytes32 => bytes32) public contributionHashes;
    
    event ContributionAnchored(
        bytes32 indexed contributionId,
        bytes32 contributionHash,
        uint256 timestamp
    );
    
    function anchorContribution(
        bytes32 contributionId,
        bytes32 hash
    ) external {
        require(contributionHashes[contributionId] == 0, "Already anchored");
        contributionHashes[contributionId] = hash;
        emit ContributionAnchored(contributionId, hash, block.timestamp);
    }
    
    function verifyContribution(
        bytes32 contributionId,
        bytes32 hash
    ) external view returns (bool) {
        return contributionHashes[contributionId] == hash;
    }
}
```

### 7.2 Blockchain Networks

Supported blockchain networks:

- **Ethereum**: Primary network for production
- **Polygon**: Low-cost alternative for high-frequency anchoring
- **Hyperledger Fabric**: Private/permissioned deployments

### 7.3 Contribution Verification

Anyone can verify contribution authenticity:

```javascript
const web3 = new Web3('https://eth-mainnet.g.alchemy.com/v2/API_KEY');
const contract = new web3.eth.Contract(ABI, CONTRACT_ADDRESS);

const contributionId = web3.utils.keccak256('contribution-UUID');
const contributionHash = web3.utils.keccak256(JSON.stringify(contributionData));

const isValid = await contract.methods
    .verifyContribution(contributionId, contributionHash)
    .call();

console.log('Contribution verified:', isValid);
```

## 8. Third-Party Service Integration

### 8.1 Identity Verification

Integrate with identity verification services:

- **Stripe Identity**: KYC verification
- **Jumio**: Document verification
- **Onfido**: Biometric verification

```json
{
  "verificationSession": {
    "memberId": "UUID",
    "provider": "stripe_identity",
    "verificationType": "id_document",
    "status": "verified|failed|pending",
    "verifiedData": {
      "fullName": "John Doe",
      "dateOfBirth": "1985-05-15",
      "documentType": "drivers_license",
      "documentNumber": "encrypted",
      "issuingCountry": "US"
    }
  }
}
```

### 8.2 Investment Data Providers

Integrate market data and investment information:

- **Bloomberg**: Professional market data
- **Morningstar**: Fund performance data
- **FactSet**: Investment analytics

### 8.3 Communication Platforms

Multi-channel member communication:

- **Twilio**: SMS and voice notifications
- **SendGrid**: Email delivery
- **Firebase**: Push notifications

## 9. Integration Testing

### 9.1 Test Environments

- **Sandbox**: Isolated testing environment
- **Staging**: Pre-production validation
- **Production**: Live environment

### 9.2 Integration Test Scenarios

1. **Happy Path**: Standard successful integration flow
2. **Error Handling**: Network failures, invalid data, timeouts
3. **Edge Cases**: Boundary conditions, unusual scenarios
4. **Performance**: Load testing, stress testing
5. **Security**: Penetration testing, vulnerability scanning

### 9.3 Certification Process

Integration certification requires:

1. Successful completion of 100+ integration test cases
2. Performance benchmarks met (response time, throughput)
3. Security audit passed
4. Documentation complete (API specs, integration guides)
5. Production readiness review

## 10. Migration and Onboarding

### 10.1 Data Migration

Migrate existing pension data to WIA-SOC-018:

```
Step 1: Data Extract → Extract data from legacy system
Step 2: Data Transform → Convert to WIA-SOC-018 format
Step 3: Data Validate → Verify data quality and completeness
Step 4: Data Load → Import into new system
Step 5: Reconciliation → Verify migration accuracy
Step 6: Cutover → Switch to new system
```

### 10.2 Parallel Running

Run old and new systems in parallel:

- Duration: 3-6 months
- Compare results for consistency
- Gradually shift traffic to new system
- Maintain fallback capability

### 10.3 Rollback Plan

Prepare comprehensive rollback procedures:

- Database backups before migration
- Traffic routing controls
- Quick rollback scripts
- Communication templates

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


## Annex E — Implementation Notes for PHASE-4-INTEGRATION

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-4-INTEGRATION.

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

