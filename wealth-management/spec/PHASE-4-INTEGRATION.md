# WIA-FIN-WEALTH-001 — Phase 4: INTEGRATION

> Wealth Management canonical Phase 4 specification per the WIA Standards four-Phase architecture.

> Domain: 자산 관리 — 자산 관리 · 포트폴리오 · MiFID II · 적합성 평가 · ESG 통합.

## A.1 Scope

This Phase covers the canonical integration layer of the WIA-FIN-WEALTH-001 standard. It composes with the Phase 1 document for cross-Phase integration and inherits cross-standard behaviour from the wider WIA Standards family per the README.md scope statement.

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

## 2. Scope

### 2.1 In Scope

- Asset tracking and valuation
- Portfolio management and optimization
- Tax planning and optimization
- Estate planning and wealth transfer
- Performance analytics and reporting
- Security and compliance
- Multi-currency and multi-account support

### 2.2 Out of Scope

- Direct trading execution (delegated to brokerage APIs)
- Banking services (checking, savings accounts)
- Insurance products
- Lending and borrowing
- Payment processing

---


## 6. Integration Patterns

### 6.1 Market Data Integration

```typescript
interface MarketDataProvider {
  getQuote(symbol: string): Promise<Quote>;
  getHistoricalData(symbol: string, period: string): Promise<HistoricalData>;
  subscribeToRealtime(symbols: string[]): EventStream<Quote>;
}
```

### 6.2 Brokerage Integration

```typescript
interface BrokerageAdapter {
  getAccount(): Promise<Account>;
  getPositions(): Promise<Position[]>;
  placeOrder(order: Order): Promise<OrderConfirmation>;
  getOrderStatus(orderId: string): Promise<OrderStatus>;
}
```

### 6.3 Tax Service Integration

```typescript
interface TaxServiceProvider {
  calculateTaxLiability(income: Income[], deductions: Deduction[]): Promise<TaxLiability>;
  generateTaxForms(userId: string, year: number): Promise<TaxForms>;
  findHarvestingOpportunities(portfolio: Portfolio): Promise<TaxLossOpportunity[]>;
}
```

---


## 10. Appendix

### 10.1 Sample Code

#### TypeScript SDK Usage

```typescript
import { WealthManagementSDK } from '@wia/wealth-management';

const sdk = new WealthManagementSDK({
  apiKey: process.env.WIA_API_KEY,
  environment: 'production'
});

// Get user portfolio
const portfolio = await sdk.getPortfolio('port_123');

// Add new asset
const asset = await sdk.addAsset({
  portfolioId: 'port_123',
  type: 'equity',
  symbol: 'AAPL',
  quantity: 100,
  costBasis: 17500.00
});

// Get performance analytics
const analytics = await sdk.getAnalytics({
  portfolioId: 'port_123',
  period: 'ytd'
});

// Execute tax-loss harvesting
const opportunities = await sdk.getTaxHarvestOpportunities('user_456');
if (opportunities.length > 0) {
  await sdk.executeTaxHarvest(opportunities[0].id);
}
```

### 10.2 References

- [ISO 4217 Currency Codes](https://www.iso.org/iso-4217-currency-codes.html)
- [ISO 8601 Date Format](https://www.iso.org/iso-8601-date-and-time-format.html)
- [OAuth 2.0 Specification](https://oauth.net/2/)
- [JWT RFC 7519](https://datatracker.ietf.org/doc/html/rfc7519)
- [OpenAPI Specification](https://swagger.io/specification/)

### 10.3 Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-12-27 | Initial release |

### 10.4 Contributing

Contributions to this standard are welcome. Please submit proposals through the WIA Standards Committee.

**Contact:**
- Email: standards@wia.org
- GitHub: https://github.com/WIA-Official/wia-standards
- Website: https://wia.org

---

**© 2025 SmileStory Inc. / WIA**
弘益人間 (홍익인간) · Benefit All Humanity

**License:** MIT License
**SPDX-License-Identifier:** MIT


---

---

## Z.1 Audit transport and observability hooks (Phase 4)

Every Phase 4 envelope SHOULD emit a structured log line at the
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
`4` as required attributes. The audit envelope follows the
canonical W3C Trace Context binary format on the wire when the
host operates over a binary protocol (e.g., gRPC over HTTP/2 or
MQTT 5) and the canonical W3C Trace Context text format when the
host operates over a text protocol (e.g., HTTP/1.1 or REST/JSON).

## Z.2 Cross-standard composition (Phase 4)

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

## Z.3 Capabilities discovery and SemVer (Phase 4)

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

## Z.4 Privacy envelope per per-jurisdiction law (Phase 4)

Phase 4 envelopes that carry personal data MUST honour the
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

## Z.5 DR / continuity envelope per ISO 22301 (Phase 4)

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

## Z.6 Supply-chain envelope per SLSA (Phase 4)

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

## Z.1 Audit transport and observability hooks (Phase 4 (variant 1))

Every Phase 4 envelope SHOULD emit a structured log line at the
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
`4` as required attributes. The audit envelope follows the
canonical W3C Trace Context binary format on the wire when the
host operates over a binary protocol (e.g., gRPC over HTTP/2 or
MQTT 5) and the canonical W3C Trace Context text format when the
host operates over a text protocol (e.g., HTTP/1.1 or REST/JSON).

## Z.2 Cross-standard composition (Phase 4 (variant 1))

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

## Z.3 Capabilities discovery and SemVer (Phase 4 (variant 1))

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

## Z.4 Privacy envelope per per-jurisdiction law (Phase 4 (variant 1))

Phase 4 envelopes that carry personal data MUST honour the
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

## Z.5 DR / continuity envelope per ISO 22301 (Phase 4 (variant 1))

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

## Z.6 Supply-chain envelope per SLSA (Phase 4 (variant 1))

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
