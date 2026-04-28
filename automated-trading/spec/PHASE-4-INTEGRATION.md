# WIA-automated-trading PHASE 4 — INTEGRATION Specification

**Standard:** WIA-automated-trading
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how an automated-trading
operator integrates with the systems that surround
algorithmic order flow: the trading venue and its
conformance-test, market-data, and surveillance
endpoints; the post-trade infrastructure (CCP,
clearing house, settlement system); the supervisory
authority for the operating jurisdiction (ESMA +
Member-State NCA in EU; SEC + FINRA + CFTC in US;
KR FSC + FSS); the consolidated tape provider (CTP);
the SEC Consolidated Audit Trail (CAT); the FINRA
TRACE channel for fixed-income; the operator's ARM
and APA channels for MiFIR Article 26 / 27 reporting;
the operator's ICT-third-party providers under DORA;
and the long-term archive that preserves trading
records past the active retention horizon.

References (CITATION-POLICY ALLOW only):

- FIX 5.0 SP2, FIX FAST, FIX Orchestra
- ISO 20022, ISO 10383 MIC, ISO 17442 LEI
- IETF RFC 8259 / 9457 / 8615 / 8288 / 9421
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 17021-1:2015, ISO/IEC 17065:2012
- ISO 8601
- W3C Verifiable Credentials Data Model 2.0
  (optional)
- EU MiFID II + MiFIR + RTS 6 + RTS 7 + RTS 25
- EU MAR + EU MiCA
- EU DORA Regulation (EU) 2022/2554
- EU EMIR Regulation (EU) 648/2012 for OTC-derivative
  clearing
- EU CSDR Regulation (EU) 909/2014 for settlement
  discipline
- US SEC Reg SCI, SEC Rule 15c3-5, SEC Reg ATS, SEC
  Reg NMS, SEC Rule 613 CAT, SEC Rule 17a-4
- US FINRA Rules 3110, 4511, 5210, 6140
- US CFTC 17 CFR 1.81
- US Dodd-Frank for OTC-derivative clearing
- KR 자본시장법 + KRX 회원규정 + KSD (한국예탁결제원)
  settlement
- TIBER-EU framework

---

## §1 Trading Venue Integration

The operator's venue integration covers:

- The FIX session establishment with the venue's
  SenderCompID / TargetCompID negotiation, message-
  rate budget, and resilience profile (MiFID II RTS
  7).
- The market-data feed subscription (level 1 / level
  2 / depth-of-book).
- The conformance-test environment access for
  algorithm certification (RTS 6 Article 5).
- The venue's market-abuse surveillance feedback
  channel — the venue surfaces patterns it
  identifies and the operator engages with the
  venue's market-control function.
- The venue's emergency-stop discipline under RTS 7
  Article 18.

For US-jurisdiction venues (NYSE, Nasdaq, CBOE,
IEX, Members Exchange) the equivalent SEC Reg ATS /
Reg NMS / Reg SCI integration applies. For KR-
jurisdiction venues the KRX 회원규정 + 자동화 매매
시스템 점검 discipline applies. For crypto-asset
trading platforms the EU MiCA + the platform's
operating rules apply.

## §2 Post-Trade Clearing and Settlement Integration

For cleared trades the operator integrates with:

- The central counterparty (CCP) for clearing
  discipline (EMIR for EU; Dodd-Frank Title VII for
  US; KR KRX 청산결제 for KR-listed derivatives).
- The clearing-house margining and default-fund
  contributions.
- The settlement system (T2S in EU; DTC, NSCC, OCC
  in US; KSD in KR) under CSDR (Regulation (EU)
  909/2014) for EU-jurisdiction settlement.
- The custodian for asset segregation and reconcili-
  ation.

## §3 Supervisory-Authority Integration

For EU-regulated operators:

- ESMA — for EU-wide thematic reviews, Q&A
  guidance, and direct supervision of certain
  CCPs / TRs.
- Member-State NCA — for the firm's authorisation,
  ongoing supervision, transaction-report
  consumption (MiFIR Article 26), market-abuse
  enforcement (MAR), and DORA supervision.

For US-regulated operators:

- SEC — for Reg SCI, Reg NMS, Reg ATS, Rule 15c3-5,
  Rule 17a-4, Rule 613 CAT, Rule 605 / 606 best-
  execution reports.
- FINRA — for member-firm supervision (Rule 3110),
  recordkeeping (Rule 4511), publication of trades
  (Rule 5210), surveillance (Rule 6140).
- CFTC — for futures and swap-dealer-related
  algorithmic trading (17 CFR 1.81; Part 38 / 40
  designated-contract-market rules).

For KR-regulated operators:

- KR FSC — for the firm's authorisation under
  자본시장법 and the FSC's published guidance.
- KR FSS — for ongoing supervision and inspection.
- KRX — for member-firm conformance.
- KOFIA (한국금융투자협회) — for industry guidance
  and self-regulation.

## §4 Consolidated Tape Provider (CTP) Integration

For EU-regulated equity flows the consolidated tape
provider (under MiFIR Article 27d once authorised)
publishes the post-trade tape; the operator's
APA channel feeds the CTP. For US equity flows the
NYSE / Nasdaq / FINRA SIP feeds the consolidated
tape under Reg NMS Rules 600 to 612.

## §5 SEC Consolidated Audit Trail (CAT) Integration

For US-jurisdiction operators the SEC CAT under
Rule 613 is the canonical recordkeeper for orders
in NMS securities and OTC equity securities. The
operator's CAT reporter integration follows the
CAT NMS Plan technical specifications; data is
reported by the operator's reporting agent or
directly to the CAT processor.

## §6 ARM and APA Integration

For MiFIR Article 26 transaction reporting and
Article 27 reference-data reporting the operator
integrates with an Approved Reporting Mechanism
(ARM) and (for pre-trade / post-trade publication)
an Approved Publication Arrangement (APA). The
ARM forwards transaction reports to the Member-
State NCA on the operator's behalf; the APA
publishes pre-trade quotes and post-trade
transactions for transparency.

## §7 ICT Third-Party Provider Integration (DORA)

For EU-regulated operators DORA Articles 28 to 30
require the operator to:

- Map ICT third-party providers (cloud providers,
  market-data vendors, FIX engine vendors,
  surveillance vendors).
- Identify ICT third-party providers supporting
  critical or important functions and place them
  in the contractual register.
- Apply the DORA contractual minima (data
  protection, security, business continuity,
  audit access, exit strategies).
- Report critical ICT third-party providers to the
  Member-State NCA so that ESMA / EBA / EIOPA can
  designate critical ICT third-party providers
  under DORA Title V for direct supervision.

## §8 Threat-Led Penetration Testing Integration

The operator's threat-led penetration testing
(TIBER-EU framework in EU, the equivalent programmes
elsewhere) is exercised on the supervisor's cadence:

- The lead threat intelligence provider is
  contracted under the operator's TIBER-EU process.
- The red team performs the test under the
  TIBER-EU rules of engagement.
- The findings feed the operator's remediation
  cycle and the DORA Article 26 advanced-resilience-
  testing report.

## §9 External Audit and ISMS Certification

The operator's ISMS is certified against ISO/IEC
27001:2022 with the scope explicitly extending to
the FIX, ISO 20022, market-data, and surveillance
endpoints. The certification body operates under
ISO/IEC 17021-1; the conformity-assessment body for
WIA-automated-trading operates under ISO/IEC 17065.

## §10 Long-Term Archival Integration

Records governed by the operator's retention
horizons (MiFID II RTS 6 Article 8 / RTS 25 /
MiFIR Article 26 five years; SEC Rule 17a-4 three-
or six-year WORM retention; FINRA Rule 4511 six
years; KR 자본시장법 보존 의무) are migrated to
the long-term archive at the close of the active
retention window. The archive preserves the
algorithm registry snapshot, the order-and-
execution record, the FIX message stream, the
transaction-report record, the surveillance-alert
record, the conformance-test report, the resilience-
drill record, and the audit-event trail.

## §11 Crypto-Asset Trading Platform Integration (MiCA)

For operators operating crypto-asset trading
platforms under EU MiCA Regulation (EU) 2023/1114:

- The crypto-asset trading platform's authorisation
  is granted by the Member-State NCA under MiCA
  Title V.
- The white-paper-based admission discipline applies
  for asset-referenced and e-money tokens (MiCA
  Title III + IV).
- Market-abuse rules under MiCA Title VI extend the
  EU MAR discipline.
- ESMA register entry under MiCA Article 110 makes
  the platform discoverable to clients and other
  authorities.

## §12 Member-Firm Resilience Reporting Integration

The operator's resilience-reporting integration
covers:

- DORA Article 19 major-ICT-incident reporting to
  the Member-State NCA.
- SEC Reg SCI Rule 1002 immediate notification and
  Rule 1003 disclosure to members.
- FINRA Rule 4530 reportable events.
- KR FSS 전자금융 사고 보고 (electronic-financial-
  incident reporting under KR 전자금융거래법).

## §13 Conformance

Implementations claiming PHASE-4 conformance maintain
the venue and post-trade-infrastructure integrations,
exercise the supervisory-authority filing
obligations, integrate with the operator's chosen
ARM / APA / CAT / TRACE channels, hold the ISO/IEC
27001 certification, exercise the DORA ICT-third-
party-risk management discipline (where applicable),
and operate the long-term archival integration
described above.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-automated-trading
- **Last Updated:** 2026-04-28
