# WIA-micro-lending PHASE 4 — Integration Specification

**Standard:** WIA-micro-lending
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable

This PHASE specifies how WIA-micro-lending integrates
with adjacent regulatory, payment, credit-bureau,
investor-reporting, social-performance, and supervisory
systems: central-bank reporting gateways, financial
intelligence units (FIU), payment rails (mobile money
and bank rails over ISO 20022), credit-bureau
exchanges, IFC E&S reporting, SPI4 social-performance
reporting, sanctions-list providers (UN, US OFAC, EU
CSL, UK OFSI, KR MOFA), MIX Market reporting (CGAP /
SPTF inventory), open-banking / open-finance APIs, and
the IDMP-equivalent product / counterparty taxonomies.
It also specifies the operational binding to companion
WIA standards.

References (CITATION-POLICY ALLOW only):
- BIS — payment-systems oversight; Basel Core Principles
- FATF Recommendations; FATF Recommendation 16 (wire transfers)
- ISO 20022 — pacs / pain / camt message families
- IETF RFC 9110 (HTTP), RFC 7515 (JWS), RFC 8259 (JSON), RFC 8785 (JCS)
- ITU-T X.1216 — security framework for digital financial services
- IFC Performance Standards (PS 1, PS 2)
- SPI4 (CERISE+SPTF) reporting taxonomy
- Client Protection Pathway (CPP)
- IFRS 9 — Financial Instruments
- ISO 17442 — LEI; ISO 4217; ISO 3166; BCP 47
- US Reg E (12 CFR 1005), Reg Z (12 CFR 1026)
- EU PSD2 (2015/2366); EU PSD3 proposal
- UK Open Banking, NextGenPSD2 (Berlin Group)
- Sanctions providers — UN Sanctions Lists, US OFAC SDN, EU Consolidated List, UK OFSI, KR MOFA
- HL7 FHIR R5 (where bank-to-clinical interop applies, e.g. bancassurance)
- CGAP MIX-Market reporting (legacy / SPI4 inventory)

---

## §1 Central-bank reporting integration

| Authority             | Reporting interface                            |
|-----------------------|------------------------------------------------|
| Central bank (KE BoK) | KE-BoK supervision portal                       |
| Central bank (NG CBN) | CBN supervisory portal                         |
| Central bank (PH BSP) | BSP FIES                                       |
| Central bank (IN RBI) | RBI XBRL reporting                              |
| Central bank (MX CNBV)| CNBV reporting portal                          |
| Central bank (US OCC) | OCC large-exposure reporting (where bound)     |

Each authority's profile is recorded in the regulator-
binding reference record; submissions sign with the
sponsor's reporting key and the regulator-issued
gateway acknowledgement identifier records on the
report.

## §2 FIU integration

| Authority                | Submission profile                            |
|--------------------------|-----------------------------------------------|
| FIU (UN)                 | goAML (most common globally)                   |
| FIU (US FinCEN)          | BSA E-Filing                                   |
| FIU (UK NCA)             | NCA SAR Online                                 |
| FIU (KR KOFIU)           | KOFIU online portal                            |
| FIU (JP JAFIC)           | JAFIC submission                               |
| FIU (NG NFIU)            | NFIU portal                                    |

STR / CTR submissions follow the FIU's profile; the
implementation records the FIU acknowledgement
reference and the period covered.

## §3 Payment rail integration

| Rail                     | Profile                                       |
|--------------------------|-----------------------------------------------|
| Bank rail (cross-border) | ISO 20022 pacs.008 / pain.001                  |
| Bank rail (domestic)     | local-net (e.g. SEPA, ACH, RTGS, FAST)         |
| Mobile money             | operator-specific (M-PESA Daraja, Airtel       |
|                          | Money APIs, MTN MoMo APIs, Wave, bKash, GCash) |
| Card rail                | EMVCo / scheme-specific                        |
| Open banking / open      | UK OBIE; NextGenPSD2; AU CDR                   |
| finance                  |                                                |

The disbursement / repayment record cites the rail
identifier; reconciliation matches operator-side daily
reports to ledger postings.

## §4 Credit-bureau integration

| Bureau type           | Profile                                          |
|-----------------------|--------------------------------------------------|
| National public bureau| per-country regulator profile                    |
| Private credit bureau | per-bureau profile                                |
| Microfinance-specific | sector-specific bureau where present              |

Reporting cadence is per bureau (often monthly). The
implementation records the bureau acknowledgement
identifier and the period covered.

## §5 Sanctions-list integration

| List provider          | Update profile                                  |
|------------------------|-------------------------------------------------|
| UN Sanctions Lists     | UN portal (consolidated)                         |
| US OFAC SDN            | OFAC SDN list (XML / delta / full)               |
| EU Consolidated List   | EU FSF download                                  |
| UK OFSI                | UK consolidated list                             |
| KR MOFA                | KR MOFA list                                     |
| In-house watchlist     | sponsor-internal                                 |

Screening events cite the list version; the periodic
re-screen re-runs over the active customer base when
a list update publishes.

## §6 Investor / IFC reporting integration

| Investor reporting kind | Binding                                        |
|-------------------------|------------------------------------------------|
| IFC E&S reporting       | IFC PS 1 / PS 2 mapping                         |
| SPI4 social audit       | CERISE+SPTF Universal Standards report          |
| Investor portfolio data | sponsor-defined NDJSON / Parquet feed           |
| Public MIX Market       | legacy / SPI4 inventory profile                 |

Investor reports sign with the sponsor's investor-
reporting key; mismatches between regulator-reported
PAR and investor-reported PAR raise a discrepancy
event for resolution.

## §7 Open banking / open finance integration

Implementations may consume bank-rail data through
open-banking APIs (UK OBIE, NextGenPSD2 Berlin Group,
EU PSD2 RTS-AISP, AU CDR). This is most relevant for
SME loans where the customer's bank-statement view
informs the income-verification step. Consent flows
follow the relevant regulator's authentication
requirements.

## §8 Cross-domain WIA bindings

| Companion standard          | Binding purpose                                |
|-----------------------------|------------------------------------------------|
| WIA-mobile-payment          | disbursement / repayment rail                  |
| WIA-cross-border-payment    | international remittance                       |
| WIA-kyc-aml                 | screening pipeline                             |
| WIA-credit-scoring          | model governance                               |
| WIA-financial-inclusion     | outcomes attribution                           |
| WIA-financial-data-exchange | counterparty / lender records                  |
| WIA-cbdc                    | retail-CBDC rail (where applicable)            |
| WIA-master-data-management  | counterparty / customer master                 |

Each binding identifies the consumed PHASE.

## §9 Long-term archival

| Authority / context           | Retention                              |
|-------------------------------|----------------------------------------|
| Central-bank inspection        | 5-7 years per national rules           |
| FATF — KYC records            | minimum 5 years post-relationship       |
| Tax records                    | per jurisdiction (typ. 7-10 years)     |
| FIU — STR records              | typically ≥ 5 years                    |
| Customer-protection complaint  | per regulator (typ. ≥ 3-5 years)       |
| IFRS 9 ECL evidence            | per audit cycle                         |

Personal-data deletion under privacy law tombstones
the customer-record payload while preserving the
audit-chain hash.

## §10 Conformance test suite

The reference test suite covers:

- KYC / CDD gate enforcement before account-open
- sanctions / PEP screening at customer-onboarding and
  before each disbursement
- four-eyes approval on large loans
- ISO 20022 pacs.008 round-trip with mobile-money rail
- end-of-day reconciliation matching operator statements
- restructuring decision with IFRS 9 stage transition
- write-off propagation to bureau and PAR report
- STR generation on a flagged transaction pattern
- customer-protection disclosure round-trip with
  acceptance proof
- collections-conduct cap enforcement

## §11 Internationalisation

User-facing strings (disclosures, repayment-schedule
text, dispute-resolution contact details) carry BCP 47
language tags. Country-specific regulator paths are
resolved by the customer's primary jurisdiction (ISO
3166-1 alpha-3).

## §12 Security and privacy posture

- Transport: TLS 1.3 with mutual TLS for sponsor /
  rail / regulator transmissions
- Authentication: client_credentials with key
  attestation for sponsor back-end; OAuth 2 with PKCE
  + device attestation for agent / field-officer apps
- At-rest: AES-256-GCM with sponsor-controlled KMS;
  per-customer key wrapping
- Audit: tamper-evident chain (PHASE 3 §11) exportable
  per ISO/IEC 27037 forensic-evidence guidance
- Privacy: customer identifiers opaque cross-domain;
  FATF-required CDD records held under regulator-
  approved DPIA
- Connectivity: agent / field-officer apps operate
  offline-first; transactions queue locally and submit
  on reconnect; the queue is hash-chained

## §13 Operational metrics

Sponsors / investors report (informationally) on the
WIA registry:

- portfolio at risk (PAR>30, PAR>90)
- write-off ratio
- restructuring rate
- approval / rejection rate (with model bias slice)
- complaint volume vs. SLA
- sanctions-screening hit rate

## §14 Recovery and continuity

- API outage — agent / field apps queue locally;
  reconcile on reconnect
- rail-operator outage — disbursements queued; daily
  reconciliation flags pending; investor / regulator
  notice if outage persists
- KMS outage — sealed back-up keys per sponsor's BCP
- regulator-gateway outage — submissions queue and
  replay; FIU clock paused only on regulator-declared
  outage

## Annex A — Worked end-to-end example (informative)

A microfinance institution operating in three sub-
Saharan markets onboards 25,000 customers through
agent-network field officers. KYC uses national
identification cards verified through the country's
identity-management system. Loans disburse over M-
PESA / Airtel Money / MTN MoMo rails. Repayments
collect over the same rails. End-of-day reconciliation
matches operator statements; mismatches feed
stewardship tasks. The MIS submits PAR aging monthly
to each central bank via the supervisory portal; STRs
file with the relevant FIU through goAML; SPI4 social
performance reports annually to investors. A pre-
holiday liquidity event triggers a restructuring
decision under §7; the restructuring committee
approves a 3-month payment holiday for affected
customers; IFRS 9 ECL provisioning increases.

## Annex B — Conformance disclosure

Implementations declare the central-bank, FIU, rail-
operator, and bureau bindings supported, the
sanctions-list updates honoured, the IFRS 9 stage-
transition rules applied, and the SPI4 audit version
implemented. Disclosure is machine-readable at
`/.well-known/wia-ml-conformance.json`.

## Annex C — Versioning

Adding a new rail-operator binding is minor; changing
the ISO 20022 message version is major.
