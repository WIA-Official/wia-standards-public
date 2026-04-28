# WIA-FIN-004 — Phase 4: Integration

> RegTech canonical Phase 4: ecosystem integration (FATF + BSA + EU AMLR + KR/JP/UK + IVMS 101 + GDPR + SupTech + DLT).

# WIA-FIN-004: RegTech Standard Specification v1.0

**Version:** 1.0.0
**Status:** Final
**Date:** January 2025
**Authors:** WIA Standards Committee
**Organization:** World Certification Industry Association (WIA)

---

## Abstract

This specification defines the WIA-FIN-004 RegTech Standard for regulatory technology and compliance automation in financial services. It provides comprehensive data formats, API specifications, security protocols, and integration guidelines for implementing automated compliance, AML/KYC verification, and real-time monitoring systems.

**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Scope](#2-scope)
3. [Normative References](#3-normative-references)
4. [Terms and Definitions](#4-terms-and-definitions)
5. [Data Formats](#5-data-formats)
6. [API Specifications](#6-api-specifications)
7. [Security Requirements](#7-security-requirements)
8. [Integration Guidelines](#8-integration-guidelines)
9. [Compliance](#9-compliance)
10. [Examples](#10-examples)

---


## 8. Integration Guidelines

### 8.1 Implementation Phases

1. **Assessment:** Evaluate current compliance processes
2. **Design:** Plan system architecture and data flows
3. **Development:** Implement APIs and integrations
4. **Testing:** Validate compliance rules and workflows
5. **Deployment:** Roll out to production environment
6. **Monitoring:** Continuous monitoring and optimization

### 8.2 Multi-jurisdiction Support

Implementations **MUST** support:

- Jurisdiction-specific compliance rules
- Localized regulatory report formats
- Regional data residency requirements
- Cross-border transaction monitoring

### 8.3 Legacy System Integration

Systems **SHOULD** provide:

- Adapter patterns for legacy system integration
- Data transformation utilities
- Backward compatibility where possible

---



## 10. Examples

### 10.1 Complete Compliance Check Flow

```typescript
// 1. Initialize client
const client = new RegTechClient({
  apiKey: 'your-api-key',
  jurisdiction: 'US'
});

// 2. Check transaction compliance
const result = await client.checkCompliance({
  transactionId: 'TXN-001',
  amount: 15000,
  currency: 'USD',
  customerRiskProfile: 'medium'
});

// 3. Handle result
if (result.status === 'fail') {
  // Block transaction
  await blockTransaction(result.transactionId);
} else if (result.requiresSAR) {
  // Generate SAR
  await client.submitReport({
    reportType: 'sar',
    reportId: 'SAR-2025-001',
    data: result.details
  });
}
```

### 10.2 KYC Verification Flow

```typescript
// Verify customer documents
const kycResult = await client.verifyKYC({
  customerId: 'CUST-001',
  documents: [{
    type: 'passport',
    documentNumber: 'P12345678',
    issuingCountry: 'US'
  }]
});

if (kycResult.status === 'verified') {
  // Onboard customer
  await onboardCustomer('CUST-001');
} else if (kycResult.status === 'pending') {
  // Request additional documents
  await requestAdditionalDocuments('CUST-001');
}
```

---

## Appendix A: Change Log

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01 | Initial release |

---

## Appendix B: Contributors

This specification was developed by the WIA Standards Committee with contributions from:

- Financial institutions
- RegTech solution providers
- Regulatory authorities
- Compliance professionals
- Technology experts

---

**© 2025 SmileStory Inc. / WIA (World Certification Industry Association)**
**弘益人間 (홍익인간) · Benefit All Humanity**

For questions or feedback: standards@wia.org



---

## A.1 Regulatory cross-walk

| Concern                       | Standard                                  |
|-------------------------------|-------------------------------------------|
| FATF AML / CFT framework      | FATF 40 Recommendations                   |
| Travel Rule                   | FATF Recommendation 16                    |
| US BSA + USA PATRIOT Act      | 31 USC 5311 / 31 CFR Chapter X            |
| US OFAC sanctions             | 31 CFR Chapter V + 50 USC 1701-1707       |
| EU AML — current              | 6th AML Directive 2018/1673 + AMLR 2024   |
| EU MAR market-abuse           | Regulation (EU) 596/2014                  |
| EU MiFID II investor-protection| Directive 2014/65/EU                     |
| KR Specific Financial Info Act| KR Act No 17112                            |
| KR FSC sanctions              | Public Notice of Designated Persons       |
| JP APG / FSA AML              | Act on Prevention of Transfer of Criminal Proceeds |
| UK MLR 2017                   | The Money Laundering, Terrorist Financing Regulations 2017 |
| Common reporting (CRS)        | OECD CRS                                  |
| FATCA                         | 26 USC 6038D + IGA                        |
| GDPR                          | Regulation (EU) 2016/679                  |
| Travel-rule message format    | IVMS 101                                  |
| Country / sub-division codes  | ISO 3166-1 / -2                           |
| Currency                      | ISO 4217                                  |
| Audit-trail signatures        | IETF RFC 7515 (JWS) / 9162 (CT v2)        |

This cross-walk is informative; implementers obtain authoritative copies from the issuing body.

## A.2 SupTech and regulator integration

SupTech (Supervisory Technology) integration captures the regulator-side data-collection envelope (e.g., FCA Digital Regulatory Reporting; MAS API Exchange; KR FSS Open Compliance API; EBA EUCLID), the streaming-and-batch ingestion envelope (per-regulator-preferred frequency), the data-quality-of-record envelope (per-regulator data-validation rules), and the regulator-side privacy envelope (typically the regulator can re-identify a subset of records under documented legal basis; SupTech operators pin the lawful-basis-of-record).

## A.3 Cross-border data and privacy integration

Cross-border data integration captures the per-jurisdiction data-flow envelope (GDPR Chapter V — Standard Contractual Clauses 2021 + adequacy decisions where applicable; APEC CBPR; bilateral arrangements; subject explicit consent under Article 49 GDPR for narrow specific circumstances). For Travel Rule the operator's per-pair VASP data-exchange envelope honours the recipient-jurisdiction's data-residency rule. The integration envelope captures the operator's data-protection-impact-assessment (DPIA) outcome where automated processing produces legal effects on the data subject.

## A.4 Blockchain / DLT integration

Blockchain integration captures the on-chain audit-anchor envelope (per-day Merkle-root commitment to a public chain — Bitcoin / Ethereum / Polygon / per-tenant permissioned chain; the signed-receipt envelope returned to the operator), the on-chain Travel Rule envelope where adopted (Sygna Bridge / TRP / VerifyVASP / OpenVASP per the operator's choice), the decentralised-identifier envelope per W3C DID + Verifiable Credentials Data Model 2.0, and the privacy-preserving compliance envelope (zero-knowledge proofs of compliance per zk-SNARK / zk-STARK research-track standards). On-chain integration honours the regulator's on-chain-data-protection envelope.

## A.5 Future directions

Active research tracks: federated-learning AML models that train across banks without sharing transaction-level data; privacy-preserving sanctions-screening via private set intersection; verifiable on-chain Travel Rule with zero-knowledge proofs; AI-native autonomous compliance agents under documented kill-switch and human-in-the-loop envelopes; quantum-resistant cryptography for long-lived audit anchors per NIST FIPS 203/204/205; cross-border SupTech data-exchange protocols with privacy-preserving aggregation. The standard's roadmap envelope (`POST /standards/v1/proposals`) tracks active proposals through the WIA Committee voting process per Phase 4 §Z.4.

## A.6 Reference list

- FATF 40 Recommendations + Methodology
- FATF Recommendation 16 — Wire transfers (Travel Rule)
- 31 USC 5311 — Bank Secrecy Act
- 31 CFR Chapter X — FinCEN regulations
- 31 CFR Chapter V — OFAC regulations
- US 50 USC 1701-1707 — IEEPA
- EU AMLR / 6AMLD (2018/1673) / 5AMLD / 4AMLD chain
- EU Regulation (EU) 596/2014 — Market Abuse Regulation
- EU Directive 2014/65/EU — MiFID II
- KR Specific Financial Information Act
- JP Act on Prevention of Transfer of Criminal Proceeds
- UK Money Laundering Regulations 2017
- OECD Common Reporting Standard
- US 26 USC 6038D + FATCA Inter-Governmental Agreements
- IVMS 101 — interVASP Messaging Standard
- IETF RFC 7515 (JWS) / RFC 7516 (JWE) / RFC 9162 (CT v2)
- W3C VC Data Model 2.0 + DID Core 1.0
- ISO 20022 — Financial services messaging
- ISO 17442 — Legal Entity Identifier (LEI)
- NIST FIPS 203 / 204 / 205 — Post-Quantum Cryptography
- UCP 600 (ICC) — Uniform Customs and Practice for Documentary Credits
- ISP98 (ICC) — International Standby Practices
- URC 522 (ICC) — Uniform Rules for Collections
- FATF Trade-Based Money Laundering guidance (2006 + 2008 + 2012 risk indicators)
- ISO/IEC 30107-3 — biometric presentation attack detection

## A.7 Industry and inter-bank integration

Industry integration captures the per-association engagement envelope (Wolfsberg Group AML principles; Basel Committee on Banking Supervision guidelines; EBA / ECB / FSB equivalents; KR FSC + FSS guidelines; per-jurisdiction banking-association engagement). Inter-bank integration captures the SWIFT-network engagement envelope (Customer Security Programme — CSP; KYC Registry; SWIFT GPI for payment-tracking; Stop and Recall Service for SAR-driven cross-border holds) and the Open Banking integration envelope per UK CMA Open Banking + EU PSD2 where the operator's scope includes payment-services regulated entities.

## A.8 Sanctions-and-screening operations integration

Sanctions integration captures the per-list-source operations envelope (per-list refresh cadence; per-list watchlist-of-record envelope), the per-screening-engine vendor envelope (in-house engine; vendor — Refinitiv World-Check / LexisNexis Bridger / FICO Tonbeller / Oracle Mantas / NICE Actimize / SAS AML), the per-tenant calibration envelope (per-list per-engine threshold tuning), and the per-region operations team envelope (per-region triage centres with the documented coverage hours and escalation chain).

## A.9 Disclosure and transparency integration

Disclosure integration honours the per-jurisdiction transparency requirement (US Annual Money Laundering Report; EU AMLR transparency annex; KR Annual KYC report to FSC + KOFIU; per-jurisdiction equivalents) and the customer-facing transparency envelope (right to know what data the operator holds; data-portability per GDPR Article 20 where applicable; the privacy-policy envelope with the documented purposes-of-processing). The integration envelope captures the operator's commitments to each regulator and to the customer relationship.


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/regtech/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-regtech-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/regtech-host:1.0.0` ships every regtech envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/regtech.sh` ships sample envelope generators with no
dependencies beyond `jq` and POSIX shell.

## Z.2 Cross-standard composition (recap)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 §5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, one signing key
chain, and one audit transport rather than maintaining N parallel
implementations.

## Z.3 Implementation runbook

A first implementation typically follows: stand up the reference
container; run the conformance suite against it end-to-end;
replace the mock backend with a real backend one endpoint at a
time; wire audit-log replication out to the operations sink;
onboard a single trusted peer for federation; expand to multiple
peers; promote to production with the warning-envelope subscription
enabled and the runbook in §Z.5 followed.

## Z.4 Backwards-compatibility promise

Within the 1.x line, every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable
and MUST continue to honour the documented status codes and content
shapes. Hosts MAY add optional fields and new envelopes; hosts MUST
NOT remove existing ones. Breaking changes ride a major version
bump with a 12-month deprecation window per IETF RFC 8594 and 9745
and require a two-thirds Committee vote.

## Z.5 Closing implementer note

This Phase fits inside the WIA Standards four-Phase architecture:
Phase 1 envelopes are the wire-format contract; Phase 2 surfaces
them through HTTPS; Phase 3 wraps them in protocol exchanges that
cross trust boundaries; Phase 4 integrates with the broader
ecosystem. Regtech deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

弘益人間 — Benefit All Humanity.
