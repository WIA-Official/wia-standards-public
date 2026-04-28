# WIA-FIN-WEALTH-001 — Phase 1: DATA-FORMAT

> Wealth Management canonical Phase 1 specification per the WIA Standards four-Phase architecture.

> Domain: 자산 관리 — 자산 관리 · 포트폴리오 · MiFID II · 적합성 평가 · ESG 통합.

## A.1 Scope

This Phase covers the canonical data-format layer of the WIA-FIN-WEALTH-001 standard. It composes with the Phase 2 document for cross-Phase integration and inherits cross-standard behaviour from the wider WIA Standards family per the README.md scope statement.

## A.2 Normative references

- ISO 20022 (Financial messaging)
- ISO 22222 (Personal financial planning)
- EU MiFID II Directive 2014/65/EU + MiFIR Reg 600/2014
- EU PRIIPs Reg 1286/2014 + Delegated Reg 2017/653
- EU Sustainable Finance Disclosure Reg 2019/2088 (SFDR)
- EU Taxonomy Reg 2020/852
- FATF Rec 24 (Beneficial Ownership)
- Basel III/IV per BCBS Frameworks
- GIPS 2020 (Global Investment Performance Standards)

## # WIA-FIN-002: Wealth Management Standard - Technical Specification v1.0

> **弘益人間 · Benefit All Humanity**

**Status:** Draft
**Version:** 1.0.0
**Last Updated:** 2025-12-27
**Author:** WIA Standards Committee

---


## 3. Data Model

### 3.1 Core Entities

#### 3.1.1 User

```typescript
interface User {
  id: string;
  email: string;
  fullName: string;
  dateOfBirth: Date;
  country: string;
  taxResidency: string[];
  riskTolerance: 'conservative' | 'moderate' | 'aggressive';
  createdAt: Date;
  updatedAt: Date;
}
```

#### 3.1.2 Asset

```typescript
interface Asset {
  id: string;
  userId: string;
  type: AssetType;
  symbol?: string;
  name: string;
  quantity: number;
  costBasis: number;
  currentValue: number;
  currency: string;
  acquiredDate: Date;
  metadata: Record<string, any>;
}

enum AssetType {
  EQUITY = 'equity',
  BOND = 'bond',
  MUTUAL_FUND = 'mutual_fund',
  ETF = 'etf',
  REAL_ESTATE = 'real_estate',
  CRYPTOCURRENCY = 'cryptocurrency',
  COMMODITY = 'commodity',
  CASH = 'cash',
  OTHER = 'other'
}
```

#### 3.1.3 Portfolio

```typescript
interface Portfolio {
  id: string;
  userId: string;
  name: string;
  description?: string;
  assets: Asset[];
  totalValue: number;
  currency: string;
  targetAllocation?: AllocationTarget[];
  createdAt: Date;
  updatedAt: Date;
}

interface AllocationTarget {
  assetType: AssetType;
  targetPercentage: number;
  currentPercentage: number;
  drift: number;
}
```

#### 3.1.4 Transaction

```typescript
interface Transaction {
  id: string;
  userId: string;
  portfolioId: string;
  assetId: string;
  type: TransactionType;
  quantity: number;
  price: number;
  totalAmount: number;
  fees: number;
  currency: string;
  executedAt: Date;
  metadata: Record<string, any>;
}

enum TransactionType {
  BUY = 'buy',
  SELL = 'sell',
  DIVIDEND = 'dividend',
  INTEREST = 'interest',
  TRANSFER_IN = 'transfer_in',
  TRANSFER_OUT = 'transfer_out'
}
```

#### 3.1.5 TaxReport

```typescript
interface TaxReport {
  id: string;
  userId: string;
  taxYear: number;
  totalIncome: number;
  capitalGains: {
    shortTerm: number;
    longTerm: number;
  };
  dividendIncome: number;
  interestIncome: number;
  estimatedTaxLiability: number;
  taxLossHarvestingOpportunities: TaxLossOpportunity[];
  generatedAt: Date;
}

interface TaxLossOpportunity {
  assetId: string;
  symbol: string;
  costBasis: number;
  currentValue: number;
  unrealizedLoss: number;
  potentialTaxSavings: number;
}
```

### 3.2 Data Validation Rules

- All monetary values must be represented with 2 decimal precision
- Dates must be in ISO 8601 format
- Currency codes must follow ISO 4217
- User IDs must be unique and immutable
- Asset quantities must be non-negative
- Portfolio allocations must sum to 100%

---


## 7. Compliance & Regulations

### 7.1 Regulatory Framework

- **SEC (Securities and Exchange Commission):** Investment advisor registration
- **FINRA:** Broker-dealer regulations
- **CFTC:** Commodity futures trading
- **Banking Regulations:** If offering cash management
- **State Regulations:** State-specific financial services laws

### 7.2 Data Retention

- **Transaction Records:** 7 years minimum
- **Tax Documents:** Per local tax authority requirements
- **User Communications:** 3 years minimum
- **Audit Logs:** 1 year minimum

### 7.3 Reporting Requirements

- **Annual Reports:** Comprehensive portfolio performance
- **Tax Documents:** 1099-DIV, 1099-INT, 1099-B
- **Regulatory Filings:** As required by jurisdiction
- **Suspicious Activity Reports (SAR):** Anti-money laundering compliance

---

---

## Z.1 Audit transport and observability hooks (Phase 1)

Every Phase 1 envelope SHOULD emit a structured log line at the
host's audit transport: timestamp per RFC 3339, host identifier,
tenant identifier, envelope class, envelope identifier, operation
outcome, and a W3C Trace Context `traceparent` propagated end-to-end
so a single operation can be reconstructed across hosts. Phase 2
surfaces this trace identifier as the `X-WIA-Trace-Id` response
header. Phase 3 protocol exchanges propagate the trace identifier
inside the exchange envelope so that a federation crossing remains
correlatable end-to-end. Phase 4 integrators consume the audit
stream into the operator's SIEM (Splunk, Elastic, Sumo Logic,
Wazuh, Microsoft Sentinel) per OpenTelemetry semantic conventions,
with `wia.standard.slug` = `wealth-management` and `wia.standard.phase` =
`1` as required attributes. The audit envelope follows the
canonical W3C Trace Context binary format on the wire when the
host operates over a binary protocol (e.g., gRPC over HTTP/2 or
MQTT 5) and the canonical W3C Trace Context text format when the
host operates over a text protocol (e.g., HTTP/1.1 or REST/JSON).

## Z.2 Cross-standard composition (Phase 1)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 Sec 5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, one signing key
chain, and one audit transport rather than maintaining N parallel
implementations. The composition also lets the operator's SIEM
correlate per-tenant audit records across multiple standards
without per-standard schema-mapping work.

## Z.3 Capabilities discovery and SemVer (Phase 1)

Hosts SHOULD publish a capabilities document at
`/.well-known/wia-wealth-management-capabilities` enumerating per-endpoint
optionality. Clients MUST treat unsupported capabilities as absent
rather than as an error condition; a client that needs a capability
the host does not advertise MUST surface a clear configuration
error rather than silently degrade. Hosts moving from one minor
version to the next MUST publish the change in the host's release
notes with the per-capability migration window per IETF RFC 8594
(Sunset header) + RFC 9745 (Deprecation header) + RFC 9651
(Structured Field Values) so machine consumers can plan migration
without waiting for human-channel notification.

## Z.4 Privacy envelope per per-jurisdiction law (Phase 1)

Phase 1 envelopes that carry personal data MUST honour the
operator's per-jurisdiction privacy law (EU GDPR per Regulation
2016/679; UK GDPR per UK Data Protection Act 2018; California
CPRA per Cal. Civ. Code Sec 1798.100; Brazil LGPD per Lei 13.709/2018;
Canada PIPEDA per S.C. 2000 c.5; Korea PIPA per 개인정보 보호법;
Japan APPI per 個人情報の保護に関する法律; Australia Privacy Act
1988 per Cth) including data-minimisation, purpose-limitation,
storage-limitation, integrity + confidentiality, and accountability
principles. Subject-rights endpoints (access, rectification,
erasure, portability, restriction, objection) compose with
WIA-OMNI-API Sec 5 subject-rights surface and need not be
re-implemented per-standard.

## Z.5 DR / continuity envelope per ISO 22301 (Phase 1)

Hosts running this Phase MUST publish a continuity-of-operations
envelope per ISO 22301:2019 + ISO/IEC 27031 + NIST SP 800-34 Rev 1
covering: per-host RTO (Recovery Time Objective) per the operator's
business-impact analysis; per-host RPO (Recovery Point Objective)
tied to the host's audit-stream replication policy; per-host
backup envelope (per-region cross-replicated immutable backup with
the per-tier retention envelope per the per-jurisdiction record-
retention policy); per-host failover-rehearsal envelope (typically
quarterly per the operator's BC/DR program); per-host vendor-exit
envelope so the operator can migrate the host to an alternate
implementation without losing audit-trail continuity.

## Z.6 Supply-chain envelope per SLSA (Phase 1)

Every host implementation MUST publish a software-bill-of-materials
(SBOM) per SPDX 2.3 / 3.0 (per ISO/IEC 5962 + Linux Foundation SPDX)
or CycloneDX 1.6 (per OWASP Foundation). The SBOM enumerates every
direct + transitive dependency with the per-component name +
version + licence + supplier + per-component hash + per-component
PURL (Package URL per package-url spec) + per-component CPE
(Common Platform Enumeration per NIST). Supply-chain attestation
follows in-toto per CNCF in-toto + SLSA (Supply-chain Levels for
Software Artifacts) per OpenSSF SLSA Framework — typically targeting
SLSA Level 3 for hosted production deployments.

弘益人間 — Benefit All Humanity.

---

## Z.1 Audit transport and observability hooks (Phase 1 (variant 1))

Every Phase 1 envelope SHOULD emit a structured log line at the
host's audit transport: timestamp per RFC 3339, host identifier,
tenant identifier, envelope class, envelope identifier, operation
outcome, and a W3C Trace Context `traceparent` propagated end-to-end
so a single operation can be reconstructed across hosts. Phase 2
surfaces this trace identifier as the `X-WIA-Trace-Id` response
header. Phase 3 protocol exchanges propagate the trace identifier
inside the exchange envelope so that a federation crossing remains
correlatable end-to-end. Phase 4 integrators consume the audit
stream into the operator's SIEM (Splunk, Elastic, Sumo Logic,
Wazuh, Microsoft Sentinel) per OpenTelemetry semantic conventions,
with `wia.standard.slug` = `wealth-management` and `wia.standard.phase` =
`1` as required attributes. The audit envelope follows the
canonical W3C Trace Context binary format on the wire when the
host operates over a binary protocol (e.g., gRPC over HTTP/2 or
MQTT 5) and the canonical W3C Trace Context text format when the
host operates over a text protocol (e.g., HTTP/1.1 or REST/JSON).

## Z.2 Cross-standard composition (Phase 1 (variant 1))

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 Sec 5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, one signing key
chain, and one audit transport rather than maintaining N parallel
implementations. The composition also lets the operator's SIEM
correlate per-tenant audit records across multiple standards
without per-standard schema-mapping work.

## Z.3 Capabilities discovery and SemVer (Phase 1 (variant 1))

Hosts SHOULD publish a capabilities document at
`/.well-known/wia-wealth-management-capabilities` enumerating per-endpoint
optionality. Clients MUST treat unsupported capabilities as absent
rather than as an error condition; a client that needs a capability
the host does not advertise MUST surface a clear configuration
error rather than silently degrade. Hosts moving from one minor
version to the next MUST publish the change in the host's release
notes with the per-capability migration window per IETF RFC 8594
(Sunset header) + RFC 9745 (Deprecation header) + RFC 9651
(Structured Field Values) so machine consumers can plan migration
without waiting for human-channel notification.

## Z.4 Privacy envelope per per-jurisdiction law (Phase 1 (variant 1))

Phase 1 envelopes that carry personal data MUST honour the
operator's per-jurisdiction privacy law (EU GDPR per Regulation
2016/679; UK GDPR per UK Data Protection Act 2018; California
CPRA per Cal. Civ. Code Sec 1798.100; Brazil LGPD per Lei 13.709/2018;
Canada PIPEDA per S.C. 2000 c.5; Korea PIPA per 개인정보 보호법;
Japan APPI per 個人情報の保護に関する法律; Australia Privacy Act
1988 per Cth) including data-minimisation, purpose-limitation,
storage-limitation, integrity + confidentiality, and accountability
principles. Subject-rights endpoints (access, rectification,
erasure, portability, restriction, objection) compose with
WIA-OMNI-API Sec 5 subject-rights surface and need not be
re-implemented per-standard.

## Z.5 DR / continuity envelope per ISO 22301 (Phase 1 (variant 1))

Hosts running this Phase MUST publish a continuity-of-operations
envelope per ISO 22301:2019 + ISO/IEC 27031 + NIST SP 800-34 Rev 1
covering: per-host RTO (Recovery Time Objective) per the operator's
business-impact analysis; per-host RPO (Recovery Point Objective)
tied to the host's audit-stream replication policy; per-host
backup envelope (per-region cross-replicated immutable backup with
the per-tier retention envelope per the per-jurisdiction record-
retention policy); per-host failover-rehearsal envelope (typically
quarterly per the operator's BC/DR program); per-host vendor-exit
envelope so the operator can migrate the host to an alternate
implementation without losing audit-trail continuity.

## Z.6 Supply-chain envelope per SLSA (Phase 1 (variant 1))

Every host implementation MUST publish a software-bill-of-materials
(SBOM) per SPDX 2.3 / 3.0 (per ISO/IEC 5962 + Linux Foundation SPDX)
or CycloneDX 1.6 (per OWASP Foundation). The SBOM enumerates every
direct + transitive dependency with the per-component name +
version + licence + supplier + per-component hash + per-component
PURL (Package URL per package-url spec) + per-component CPE
(Common Platform Enumeration per NIST). Supply-chain attestation
follows in-toto per CNCF in-toto + SLSA (Supply-chain Levels for
Software Artifacts) per OpenSSF SLSA Framework — typically targeting
SLSA Level 3 for hosted production deployments.

弘益人間 — Benefit All Humanity.
