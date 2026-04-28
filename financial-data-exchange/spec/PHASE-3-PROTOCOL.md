# WIA-financial-data-exchange PHASE 3 — PROTOCOL Specification

**Standard:** WIA-financial-data-exchange
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern a
financial-data-exchange operator: the ISO 20022
message-discipline (per business-area implementation
guidelines + the SWIFT MT-to-ISO-20022 industry
coexistence cutover); the FIX 5.0 SP2 trading
discipline; the Strong Customer Authentication and
exemption discipline (PSD2 RTS Articles 4-18); the
Open Banking discipline (PSD2 + Open Banking UK +
FDX + KR 마이데이터 + US §1033); the consent-
withdrawal and customer-rights discipline; the
sanctions-screening + AML discipline; the
operational-resilience discipline (DORA + FFIEC IT
Exam + KR 전자금융감독규정); the supervisory and
oversight cooperation discipline.

References (CITATION-POLICY ALLOW only):

- ISO 9001:2015, ISO/IEC 27001:2022, ISO 22301:2019
  BCMS
- ISO 20022 + 9362 + 13616 + 17442 + 6166 + 10962
  + 11649 + 10383 + 18774
- FIX 5.0 SP2 + FIX FAST + FIXatdl + FIX Orchestra
- SWIFT MT + MX + GPI + CSP/CSCF
- IETF RFC 5905 (NTPv4), RFC 9421 (HTTP Message
  Signatures), RFC 9457 (Problem Details), RFC
  6749 (OAuth 2.0), RFC 8252, RFC 9126, RFC 9396,
  RFC 9449
- OpenID FAPI 2.0 Security Profile + Message
  Signing + CIBA Profile
- EU PSD2 (Directive (EU) 2015/2366) + Commission
  Delegated Reg (EU) 2018/389 RTS-SCA Articles
  1-36
- EU PSD3 proposal + EU PSR proposal
- EU FIDA proposal
- EU EBA RTS on SCA + RTS v2 + EBA Final Guidelines
  on outsourcing + EBA Final Report on incident
  reporting under PSD2
- UK OBIE Read/Write API specifications + UK Open
  Finance LTRF (Long-Term Regulatory Framework)
- US CFPB §1033 final rule + 12 CFR Part 1033 (2024)
- US FFIEC IT Examination Handbook
- US Reg E (12 CFR Part 1005) + Reg DD (12 CFR
  Part 1030)
- US OFAC SDN + EU Consolidated List + UN Security
  Council sanctions
- FATF Recommendations 10 / 11 / 13 / 16 / 20 / 21
  + Travel Rule
- EU DORA (Reg (EU) 2022/2554) ICT-third-party
  risk + incident reporting + threat-led
  penetration testing
- KR 신용정보의 이용 및 보호에 관한 법률 + KR 마이
  데이터 표준 API + KR 전자금융거래법 + KR 전자금융
  감독규정 + KR 금융보안원 (FSI) 가이드 + KR 자금
  세탁방지(특정금융정보법)

---

## §1 ISO 20022 Message-Discipline

The ISO 20022 message-discipline:

- Per-business-area implementation guidelines (CGI-MP
  Common Global Implementation Market Practice for
  pacs / pain / camt; SMPG Securities Market
  Practice Group for setr / semt / seev; ISITC for
  US securities-services).
- Message-version pinning per the operator's bilateral
  CT-and-MT migration timeline.
- Canonical-ISO-20022 schema validation at the
  inbound gateway.
- The SWIFT MT-to-ISO-20022 industry coexistence
  cutover programme observed for cross-border
  payments (the operator publishes its outbound
  pacs.008 / pacs.009 cutover date and the inbound
  translation shim).

## §2 FIX 5.0 SP2 Trading Discipline

The FIX trading discipline:

- Session-level resilience — re-connection, sequence-
  number recovery, possible-resend processing.
- Application-level message integrity via FIX
  Orchestra-defined rules and the venue's
  conformance certification.
- Pre-trade risk controls per WIA-automated-trading
  cross-reference.
- Post-trade reporting cadence per the venue's
  publication-rules.

## §3 Strong-Customer-Authentication and Exemption
       Discipline

The PSD2 SCA discipline (RTS Articles 4-18):

- Article 4 — SCA elements (knowledge / possession
  / inherence) — at least two independent factors.
- Article 5 — dynamic-linking for payment-SCA: the
  authentication code is uniquely linked to the
  payment amount and the payee.
- Articles 10-12 — confidentiality and integrity of
  the user's personalised security credentials.
- Article 13 — trusted beneficiaries exemption.
- Article 14 — recurring-transaction exemption.
- Article 16 — low-value-payment exemption (under
  EUR 30 with cumulative caps).
- Article 17 — corporate-payment exemption.
- Article 18 — transaction-risk-analysis (TRA)
  exemption — per-PSP fraud-rate-banded.

For US §1033 the equivalent is OAuth 2.0 + FAPI
2.0 Security Profile baseline; KR 마이데이터 also
applies FAPI-aligned authentication.

## §4 Open-Banking Discipline

The open-banking discipline aligned across regimes:

- EU PSD2 — AISP / PISP / CBPII registered with
  the home-Member-State NCA + EBA register.
- UK OBIE — the Open Banking Implementation Entity
  Read/Write API specification + the OBIE
  Conformance Tool.
- US §1033 (CFPB final rule 2024) — Authorized
  Data Provider obligations + Standard Setting
  Body recognition + permissioned data access via
  the FDX standard.
- KR 마이데이터 — 본인신용정보관리업 license +
  표준 API + 전송요구권.
- FDX (Financial Data Exchange) — US-led API
  v6.0 + Common Standard.

The operator's consent-and-token discipline retains
PSD2 RTS Article 10 180-day cap unless renewed via
SCA; US §1033 has its own 12-month re-authorisation
cadence.

## §5 Consent-Withdrawal and Customer-Rights
       Discipline

PSD2 Article 64 customer right to withdraw consent;
US §1033 right to revoke; KR 마이데이터 전송요구권
withdrawal; FDX recipient-driven consent.

The operator's discipline:

- Withdrawal channel as easy as the capture channel.
- Immediate effect on token / API access.
- No fee for withdrawal.
- Audit-log of withdrawal preserved for the
  retention horizon.

## §6 Sanctions-Screening and AML Discipline

Cross-domain reference to WIA-cross-border-payment
+ WIA-anti-money-laundering disciplines:

- Originator and beneficiary screening at message
  acceptance.
- OFAC SDN + non-SDN + 50% rule (US-jurisdiction).
- EU Consolidated List + UK HMT (UK-jurisdiction).
- KR 외교부 제재 + KoFIU (KR-jurisdiction).
- UN Security Council Sanctions List.
- FATF Recommendation 16 Travel Rule for cross-
  border wire transfers.

## §7 Operational-Resilience Discipline

The operational-resilience discipline:

- EU DORA (Reg (EU) 2022/2554) — for EU-regulated
  financial entities the ICT-third-party-risk-
  management, incident-reporting, threat-led
  penetration testing (TIBER-EU), and resilience-
  testing programme.
- US FFIEC IT Examination Handbook — for US-
  regulated banks, credit unions, and savings
  associations the operational-resilience
  expectations.
- ISO 22301:2019 BCMS — the operator's business-
  continuity-management system.
- KR 전자금융감독규정 + KR 금융보안원 가이드 — KR-
  jurisdiction operational-resilience.
- SWIFT Customer Security Programme (CSP) +
  Customer Security Controls Framework (CSCF)
  annual self-attestation + (where elected)
  independent assessment.

## §8 Identity, Time, and Audit Discipline

NTPv4 stratum-2 or better is the operator's clock
baseline. UTC traceability per MiFID II RTS 25
applies for the trading-execution surface (cross-
domain reference to WIA-automated-trading).

Audit-events emitted for every message, payment,
SCA, consent, trade, and supervisory correspondence.

## §9 Customer-Information-Security Discipline

The customer-information-security discipline:

- ISO/IEC 27001:2022 ISMS baseline.
- ISO/IEC 27018:2019 PII in public clouds.
- ISO/IEC 27701:2019 PIMS.
- PCI DSS v4 for card-data scope.
- KR 금융보안원 (FSI) 가이드 + 전자금융감독규정.
- US Gramm-Leach-Bliley Act + Reg P (12 CFR Part
  1016) Privacy of Consumer Financial Information.

## §10 Settlement-Finality and Liquidity Discipline

Cross-domain reference to WIA-cross-border-payment
+ WIA-automated-trading:

- Settlement finality on TARGET2 / CHIPS / Fedwire
  / BOK-Wire+ per the system's posted finality
  rules (TARGET2 + Settlement Finality Directive
  98/26/EC; CHIPS Rules; Fedwire Operating Circular
  6).
- T+1 securities settlement for US-jurisdiction
  (post-2024-05 transition); EU CSDR T+2 with
  T+1 in scope under the EU SP4T1 Action Plan.
- Liquidity-risk discipline per CPMI PFMI Principle
  7 + Basel III LCR / NSFR.
- Intraday liquidity management per BCBS 248
  monitoring tools.

## §11 Operational-Risk and Cyber-Resilience
        Discipline

The operational-risk and cyber-resilience discipline:

- Basel III + Basel IV operational-risk capital
  framework.
- DORA Articles 5-23 ICT-risk-management framework
  (where EU-jurisdiction).
- DORA Article 24-27 advanced-resilience-testing
  programme + threat-led penetration testing per
  TIBER-EU.
- DORA Articles 28-30 ICT-third-party-risk-
  management.
- US Reg SCI for SCI-entities + FFIEC IT Exam.
- KR 전자금융감독규정 + 금융보안원 가이드.
- ISO/IEC 27031 ICT-readiness for business
  continuity.

## §12 Card-Networks-and-Tokenisation Discipline

For card-issuing and acquiring operators:

- PCI DSS v4 compliance for card-data scope.
- EMVCo Tokenisation Specification for network
  tokenisation.
- 3-D Secure 2.x (EMVCo) for cardholder
  authentication aligned with PSD2 SCA.
- Card-network rule books (Visa Core Rules /
  Mastercard Rules) for chargeback and dispute
  processing.

## §13 Reference-Data Distribution Discipline

For reference-data operators:

- ISIN allocation per ISO 6166 by the National
  Numbering Agency.
- LEI issuance per ISO 17442:2020 by the GLEIF-
  accredited Local Operating Unit.
- BIC issuance by SWIFT under ISO 9362.
- ISO 10962 CFI per the issuing CSD.
- Reference-data subscription distribution via
  ISO 20022 reda messages.

## §14 Conformance

Implementations claiming PHASE-3 conformance enforce
the discipline at every relevant decision point,
satisfy the ISO 20022 + FIX + SWIFT message-
discipline applicable to the operator, exercise the
PSD2 / §1033 / 마이데이터 / FDX SCA + consent
discipline on the operator's open-banking surface,
exercise the operational-resilience discipline (DORA
/ FFIEC / KR 전자금융감독규정), and exercise the
sanctions-screening + AML discipline.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-financial-data-exchange
- **Last Updated:** 2026-04-29
