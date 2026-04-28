# WIA-financial-data-exchange PHASE 4 — INTEGRATION Specification

**Standard:** WIA-financial-data-exchange
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how a financial-data-exchange
operator integrates with the systems that surround
the financial-data lifecycle: the SWIFT network and
GPI; the wholesale-payment systems (TARGET2, CHIPS,
Fedwire, BOK-Wire+); the central counterparties
(CCPs) and central securities depositories (CSDs);
the trading venues (regulated market, MTF, OTF,
ATS, OTC); the trade repositories (ARM / APA / TR
under EMIR / MiFIR / Dodd-Frank); the open-banking
ecosystem (PSD2 AISP/PISP/CBPII, Open Banking UK,
FDX, KR 마이데이터); the supervisory authority for
the operating jurisdiction; the FATF VASP / Travel-
Rule network; the corporate-treasury and ERP
ecosystem; the external auditor and ISO/IEC 27001
+ ISO 22301 + SOC 2 certification body; and the
long-term archive.

References (CITATION-POLICY ALLOW only):

- ISO 20022, ISO 9362, ISO 13616, ISO 17442, ISO
  6166, ISO 10962, ISO 11649, ISO 10383, ISO 18774,
  ISO 22301:2019
- IETF RFC 8259 / 9457 / 8615 / 8288 / 9421
- ISO/IEC 27001:2022, ISO/IEC 27018:2019, ISO/IEC
  27701:2019
- ISO/IEC 17021-1:2015, ISO/IEC 17065:2012
- FIX 5.0 SP2 + FIX FAST + FIX Orchestra
- SWIFT MT + MX + GPI + CSP/CSCF
- FpML 5.x
- FDX API v6.0
- EU PSD2 (Directive (EU) 2015/2366) + RTS-SCA Reg
  (EU) 2018/389 + EU PSD3 + EU PSR proposals + EU
  FIDA proposal + EU EMIR Reg 648/2012 + EU MiFIR
  Reg 600/2014 + EU CSDR Reg 909/2014 + EU DORA
  Reg 2022/2554 + EU Settlement Finality Directive
  98/26/EC + EU Wire Transfer Reg 2023/1113
- UK OBIE Read/Write + UK Open Finance LTRF + UK
  PRA / FCA
- US CFPB §1033 final rule + 12 CFR Part 1033 + US
  FFIEC IT Exam + US SEC Rule 17a-4 + US CFTC + US
  FRB / OCC / FDIC / NCUA
- US Gramm-Leach-Bliley Act + Reg P
- KR 신용정보법 + KR 마이데이터 표준 API + KR
  전자금융거래법 + KR 전자금융감독규정 + KR FSC + KR
  FSS + KR 금융보안원 + KR 자본시장법 + 특정금융
  정보법 + KoFIU + 한국은행 BOK-Wire+ + KSD
- TARGET2 + T2-T2S + CHIPS + Fedwire OC 6 + CLS

---

## §1 SWIFT Network and GPI Integration

The operator's SWIFT integration:

- BIC / SWIFTNet connectivity (Alliance Access /
  Lite2 / Microgateway).
- SWIFT FIN for legacy MT messages.
- SWIFT InterAct for ISO 20022 messages.
- SWIFT GPI tracker for cross-border customer-
  credit-transfer status (UETR-keyed).
- SWIFT GPI CCT Inst for instant cross-border.
- SWIFT CSP / CSCF annual self-attestation +
  optional independent assessment.
- SWIFT MT-to-ISO-20022 industry coexistence
  cutover programme participation.

## §2 Wholesale-Payment-System Integration

The operator's wholesale-payment-system integration:

- TARGET2 / T2-T2S — Eurosystem participation
  through the operator's central-bank account
  access.
- CHIPS — US-dollar large-value clearing among US-
  banking participants.
- Fedwire (Federal Reserve) — US-dollar real-time
  gross settlement under Operating Circular 6.
- BOK-Wire+ (Bank of Korea) — KRW real-time gross
  settlement.
- CHAPS (UK) + BOJ-NET (JP) — additional
  jurisdictional RTGS systems.
- CLS — for FX-funding leg PvP settlement (cross-
  reference WIA-cross-border-payment).

## §3 CCP / CSD Integration

The operator's post-trade infrastructure integration:

- CCP integration — for cleared trades (EU EMIR;
  US Dodd-Frank Title VII; KR 청산결제) the operator
  maintains the clearing-member account, posts
  initial-and-variation margin, and contributes to
  the default fund.
- CSD integration — for settlement (EU CSDR Reg
  909/2014; T2-T2S; DTC / NSCC / OCC in US; KSD
  in KR) the operator maintains the settlement-
  account and reconciles per the published
  cycle.
- Custodian integration — for asset segregation
  per EU CSDR Article 38 + EMIR Article 39 +
  Dodd-Frank Title VII.

## §4 Trading-Venue Integration

For trading-venue-class operators or institutional
clients participating:

- Regulated market / MTF / OTF / ATS connectivity
  per FIX 5.0 SP2.
- Pre-trade transparency (ENS / order-book) under
  MiFIR Article 3 + 8 + waivers.
- Post-trade transparency (APA-published trades).
- ARM (Approved Reporting Mechanism) under MiFIR
  Article 26.
- US Reg NMS + Reg ATS + CAT + TRACE for US-
  jurisdiction.

## §5 Trade-Repository Integration

For OTC-derivative reporting:

- EU EMIR trade-repositories (DTCC / KDPW / Regis-
  TR / UnaVista).
- US Dodd-Frank SDR (Swap Data Repository).
- KR 청산결제 / FSC swap reporting.
- UPI (Unique Product Identifier) per ISO 4914 +
  UTI per CPMI-IOSCO.

## §6 Open-Banking Ecosystem Integration

For PSD2 / Open Banking UK / FDX / KR 마이데이터:

- AISP / PISP / CBPII registration with the home-
  Member-State NCA + EBA register; OBIE Conformance
  Tool certification (UK).
- FDX — US-jurisdiction Authorized Data Provider
  registration + Standard Setting Body recognition
  per CFPB §1033.
- KR 마이데이터 — 본인신용정보관리업 license + 표준
  API conformance + 신용정보원 + KR 금융보안원
  certification.
- ENS / WIA-anti-money-laundering integration for
  AML / sanctions screening.

## §7 Supervisory-Authority Integration

For US-jurisdiction operators:

- US SEC / CFTC / FRB / OCC / FDIC / NCUA / FFIEC
  / CFPB.

For EU-jurisdiction operators:

- EU EBA + ESMA + ECB Single Supervisory Mechanism
  (SSM) for significant institutions + Member-State
  NCA.

For UK-jurisdiction operators:

- UK PRA + FCA + Bank of England.

For KR-jurisdiction operators:

- KR FSC + FSS + FIU + 금융보안원 + 자본시장법
  enforcement + 한국은행.

## §8 FATF VASP / Travel-Rule Network Integration

Cross-domain reference to WIA-cross-border-payment
+ WIA-blockchain-intro for VASP operators:

- FATF Recommendation 16 Travel Rule integration
  via IVMS 101 + TRP / TRISA / OpenVASP / Sygna
  Bridge.
- US FinCEN BSA + 31 CFR 1010.410(e)(f) Travel
  Rule.
- EU Wire Transfer Reg (EU) 2023/1113.
- KR 특정금융정보법 Travel Rule.

## §9 Corporate-Treasury and ERP Integration

For corporate-treasury operators:

- Bank-to-corporate H2H connectivity (SWIFT FileAct
  / EBICS / Open Banking).
- ISO 20022 pain.001 customer-credit-transfer-
  initiation and pain.008 direct-debit-initiation
  feeding the operator's bank.
- camt.052 / camt.053 / camt.054 bank-to-customer
  account statements ingested into the ERP.
- TWIST + ANSI ASC X9 corporate-treasury messaging
  legacy.

## §10 External Audit and ISMS Certification

The operator's ISMS is certified against ISO/IEC
27001:2022 + ISO/IEC 27018 + ISO/IEC 27701. ISO
22301 BCMS certification for business-continuity.
SOC 2 Type II report for service-organisation
controls. PCI DSS v4 attestation-of-compliance for
card-data scope. The certification body operates
under ISO/IEC 17021-1; the conformity-assessment
body for WIA-financial-data-exchange operates under
ISO/IEC 17065.

## §11 Long-Term Archival Integration

Records governed by the operator's retention
horizons (US SEC 17a-4 three- or six-year WORM;
EU UCC five-year + EMIR ten-year for trades;
PSD2 five-year for transactions; KR 신용정보법 5-
year + 자본시장법 10-year; ISO 22301 BCMS audit
trail) are migrated to the long-term archive at
the close of the active retention window.

## §12 Identity-Federation and Access Integration

For consent-driven open-banking access:

- OAuth 2.1 + FAPI 2.0 Security Profile (mTLS or
  private_key_jwt + DPoP).
- Pushed Authorization Requests (RFC 9126) +
  Rich Authorization Requests (RFC 9396).
- OpenID Connect 1.0 for identity assertion.
- FAPI-CIBA Profile for decoupled authentication.
- eIDAS 2.0 (EU) + KR 디지털신원지갑 + UK Digital
  Identity for cross-jurisdiction identity
  federation.

## §13 ESG and Sustainability-Reporting Integration

For ESG-disclosure-subject operators:

- SFDR Article 6 / 8 / 9 product classification
  feeding the operator's distribution channel
  (cross-reference WIA-esg-finance).
- ISSB IFRS S1 / S2 climate disclosure for the
  operator's annual report.
- EU CSRD / ESRS double-materiality assessment
  for in-scope EU operators.
- EU Taxonomy Regulation (EU) 2020/852 alignment
  reporting.
- EU CSDDD (EU) 2024/1760 supply-chain-due-
  diligence integration.

## §14 Tokenisation and DLT Pilot Integration

For operators piloting tokenised financial
instruments under the EU DLT Pilot Regime:

- EU Reg (EU) 2022/858 DLT Pilot Regime regulatory
  sandbox participation.
- Tokenised-securities issuance + secondary-
  trading on a DLT MTF / DLT TSS.
- Cross-domain reference to WIA-blockchain-intro
  for the underlying DLT platform discipline.
- Bilateral integration with traditional CSD via
  the operator's bridge layer.

## §15 Real-Time Gross Settlement (RTGS) Integration

For wholesale-payment operators that settle in central-
bank money:

- TARGET2 / T2 (Eurosystem) — operator's participant
  BIC, the Single Shared Platform connection, and the
  per-day liquidity-management discipline aligned with
  the ECB's TARGET Guideline.
- Fedwire Funds Service — operator's ABA + Fedwire
  participant identifier, the FedLine connection, and
  the per-day operating window per Federal Reserve
  Operating Circular 6.
- CHAPS (Bank of England) — operator's CHAPS Direct
  Participant identifier and the SWIFT FIN connection.
- BOJ-NET (Bank of Japan) — operator's BOJ-NET
  participant identifier and the cutoff-time discipline.
- BOK-Wire+ (Bank of Korea) — operator's BOK-Wire+
  identifier and the per-day liquidity-management
  arrangement coordinated with KFTC retail clearing.
- CNAPS (China) — for KR-jurisdiction operators with
  CNH settlement obligations.

The RTGS integration carries the participant identifier,
the operating window, the queue-management policy
configured at the central-bank platform, the per-day
liquidity-position monitoring, and the contingency-
arrangement reference for outage scenarios.

## §16 Conformance

Implementations claiming PHASE-4 conformance maintain
the SWIFT, wholesale-payment, CCP / CSD, trading-
venue, trade-repository, open-banking, supervisory,
Travel-Rule (where applicable), corporate-treasury,
and RTGS integrations, hold the ISO/IEC 27001 + ISO
22301 + (where applicable) SOC 2 + PCI DSS
certifications, and operate the long-term archival
integration described above.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-financial-data-exchange
- **Last Updated:** 2026-04-29
