# WIA-FIN-WEALTH-001 — Phase 2: API

> Wealth Management canonical Phase 2 specification per the WIA Standards four-Phase architecture.

> Domain: 자산 관리 — 자산 관리 · 포트폴리오 · MiFID II · 적합성 평가 · ESG 통합.

## A.1 Scope

This Phase covers the canonical api layer of the WIA-FIN-WEALTH-001 standard. It composes with the Phase 3 document for cross-Phase integration and inherits cross-standard behaviour from the wider WIA Standards family per the README.md scope statement.

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

## Table of Contents

1. [Introduction](#introduction)
2. [Scope](#scope)
3. [Data Model](#data-model)
4. [API Specification](#api-specification)
5. [Security Requirements](#security-requirements)
6. [Integration Patterns](#integration-patterns)
7. [Compliance & Regulations](#compliance--regulations)
8. [Performance Requirements](#performance-requirements)
9. [Testing & Validation](#testing--validation)
10. [Appendix](#appendix)

---


## 4. API Specification

### 4.1 RESTful API Endpoints

#### 4.1.1 Portfolio Management

```
GET    /api/v1/portfolios
GET    /api/v1/portfolios/:id
POST   /api/v1/portfolios
PUT    /api/v1/portfolios/:id
DELETE /api/v1/portfolios/:id
```

#### 4.1.2 Asset Management

```
GET    /api/v1/assets
GET    /api/v1/assets/:id
POST   /api/v1/assets
PUT    /api/v1/assets/:id
DELETE /api/v1/assets/:id
GET    /api/v1/assets/:id/valuation
```

#### 4.1.3 Analytics

```
GET    /api/v1/analytics/performance
GET    /api/v1/analytics/risk-metrics
GET    /api/v1/analytics/allocation
POST   /api/v1/analytics/optimize
```

#### 4.1.4 Tax Services

```
GET    /api/v1/tax/reports/:year
POST   /api/v1/tax/harvest-opportunities
POST   /api/v1/tax/execute-harvest
```

### 4.2 Request/Response Examples

#### Get Portfolio

**Request:**
```http
GET /api/v1/portfolios/port_123 HTTP/1.1
Host: api.wia.org
Authorization: Bearer {access_token}
```

**Response:**
```json
{
  "id": "port_123",
  "userId": "user_456",
  "name": "Main Portfolio",
  "totalValue": 1247850.00,
  "currency": "USD",
  "assets": [
    {
      "id": "asset_789",
      "type": "equity",
      "symbol": "AAPL",
      "name": "Apple Inc.",
      "quantity": 1020,
      "currentValue": 124785.00
    }
  ],
  "performance": {
    "ytd": 0.114,
    "oneMonth": 0.035,
    "oneYear": 0.187
  }
}
```

#### Create Asset

**Request:**
```http
POST /api/v1/assets HTTP/1.1
Host: api.wia.org
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "portfolioId": "port_123",
  "type": "equity",
  "symbol": "MSFT",
  "quantity": 50,
  "costBasis": 20765.00
}
```

**Response:**
```json
{
  "id": "asset_890",
  "portfolioId": "port_123",
  "type": "equity",
  "symbol": "MSFT",
  "name": "Microsoft Corporation",
  "quantity": 50,
  "costBasis": 20765.00,
  "currentValue": 21530.00,
  "unrealizedGain": 765.00,
  "createdAt": "2025-12-27T14:32:15Z"
}
```

### 4.3 GraphQL Schema

```graphql
type Query {
  user(id: ID!): User
  portfolio(id: ID!): Portfolio
  portfolios(userId: ID!): [Portfolio!]!
  asset(id: ID!): Asset
  analytics(portfolioId: ID!, period: String!): Analytics
  taxReport(userId: ID!, year: Int!): TaxReport
}

type Mutation {
  createPortfolio(input: CreatePortfolioInput!): Portfolio!
  updatePortfolio(id: ID!, input: UpdatePortfolioInput!): Portfolio!
  deletePortfolio(id: ID!): Boolean!
  addAsset(input: AddAssetInput!): Asset!
  removeAsset(id: ID!): Boolean!
  rebalancePortfolio(portfolioId: ID!): RebalanceResult!
  executeTaxHarvest(opportunityId: ID!): Transaction!
}

type Subscription {
  portfolioValueUpdated(portfolioId: ID!): Portfolio!
  marketDataUpdated(symbols: [String!]!): [MarketData!]!
}
```

---


## 8. Performance Requirements

### 8.1 Scalability

- Support 1,000,000+ concurrent users
- Handle 10,000+ requests per second
- Process 100,000+ portfolio valuations per minute
- Store 1 billion+ historical transactions

### 8.2 Latency

- API response time: < 200ms (p95)
- Real-time quote updates: < 100ms
- Portfolio valuation: < 500ms
- Report generation: < 5 seconds

### 8.3 Availability

- System uptime: 99.95% (excluding scheduled maintenance)
- Data backup: Every 6 hours
- Disaster recovery: RPO < 1 hour, RTO < 4 hours
- Multi-region deployment for redundancy

---

---

## Z.1 Audit transport and observability hooks (Phase 2)

Every Phase 2 envelope SHOULD emit a structured log line at the
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
`2` as required attributes. The audit envelope follows the
canonical W3C Trace Context binary format on the wire when the
host operates over a binary protocol (e.g., gRPC over HTTP/2 or
MQTT 5) and the canonical W3C Trace Context text format when the
host operates over a text protocol (e.g., HTTP/1.1 or REST/JSON).

## Z.2 Cross-standard composition (Phase 2)

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

## Z.3 Capabilities discovery and SemVer (Phase 2)

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

## Z.4 Privacy envelope per per-jurisdiction law (Phase 2)

Phase 2 envelopes that carry personal data MUST honour the
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

## Z.5 DR / continuity envelope per ISO 22301 (Phase 2)

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

## Z.6 Supply-chain envelope per SLSA (Phase 2)

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

## Z.1 Audit transport and observability hooks (Phase 2 (variant 1))

Every Phase 2 envelope SHOULD emit a structured log line at the
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
`2` as required attributes. The audit envelope follows the
canonical W3C Trace Context binary format on the wire when the
host operates over a binary protocol (e.g., gRPC over HTTP/2 or
MQTT 5) and the canonical W3C Trace Context text format when the
host operates over a text protocol (e.g., HTTP/1.1 or REST/JSON).

## Z.2 Cross-standard composition (Phase 2 (variant 1))

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

## Z.3 Capabilities discovery and SemVer (Phase 2 (variant 1))

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

## Z.4 Privacy envelope per per-jurisdiction law (Phase 2 (variant 1))

Phase 2 envelopes that carry personal data MUST honour the
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

## Z.5 DR / continuity envelope per ISO 22301 (Phase 2 (variant 1))

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

## Z.6 Supply-chain envelope per SLSA (Phase 2 (variant 1))

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
