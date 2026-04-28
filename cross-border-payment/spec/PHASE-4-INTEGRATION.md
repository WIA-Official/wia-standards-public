# WIA-cross-border-payment PHASE 4 — INTEGRATION Specification

**Standard:** WIA-cross-border-payment
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how a cross-border payment
operator integrates with the systems that surround the
cross-border value-transfer lifecycle: the SWIFT
network and SWIFT GPI; the major large-value-payment
systems (TARGET2 / T2-T2S, CHIPS, Fedwire) and the
operating jurisdictions' RTGS systems; the operator's
correspondent-bank network and the bilateral CBDDQ
exchange; the financial-intelligence units (FIUs) for
SAR / STR filing; the supervisory financial-services
authority and the central bank for the operating
jurisdiction; the customer channels (corporate cash-
management portal, retail remittance app); the
external auditor and the ISO/IEC 27001 certification
body; the central counterparty for FX-funding leg
where the cross-border path uses CLS or an equivalent
PvP settlement mechanism; and the long-term archive
that preserves wire-message and SAR / STR records past
the active retention horizon.

References (CITATION-POLICY ALLOW only):

- ISO 20022, ISO 4217, ISO 3166-1, ISO 9362, ISO
  13616, ISO 17442
- IETF RFC 8259 / 9457 / 8615 / 8288 / 9421
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 17021-1:2015 (audit and certification)
- ISO/IEC 17065:2012 (conformity-assessment bodies)
- ISO 8601 (date and time)
- W3C Verifiable Credentials Data Model 2.0 (optional
  for re-issuance of attestations)
- CPMI Principles for Financial Market
  Infrastructures
- CPMI cross-border payments roadmap
- FATF Recommendations 13, 16, 20, 21
- Wolfsberg CBDDQ + AML Principles for Correspondent
  Banking
- US BSA + 31 CFR Chapter X, OFAC programmes
- EU AMLR (EU) 2024/1624, AMLA Reg (EU) 2024/1620,
  EU Wire Transfer Reg (EU) 2023/1113, EU SEPA
  Reg 260/2012, EU Cross-Border Payments Reg
  2021/1230
- KR 특정금융정보법 + KoFIU
- UK MLR 2017 + NCA-UKFIU
- TARGET2 + T2-T2S, CHIPS, Fedwire, CLS

---

## §1 SWIFT Network and GPI Integration

The operator integrates with the SWIFT network through:

- The operator's BIC and SWIFTNet connectivity
  (Alliance Access / Alliance Lite2 / SWIFT
  Microgateway, depending on the operator's
  scale).
- The SWIFT FIN service for legacy MT 103 / 202 /
  202 COV exchange.
- The SWIFT InterAct service for ISO 20022 pacs /
  camt / pain message exchange (the SWIFT MT-to-
  ISO-20022 industry coexistence cutover programme
  governs the operator's migration path).
- The SWIFT GPI tracker for end-to-end customer-
  credit-transfer status (UETR-keyed).
- The SWIFT GPI Customer Credit Transfer Inst
  (CCT Inst) discipline for instant cross-border
  payments where the corridor and the correspondent
  support the service.
- The SWIFT Sanctions Screening service (where
  subscribed) as an external screening engine
  feeding PHASE-1 §7.

## §2 Large-Value Payment System Integration

The operator integrates with the relevant large-value
payment systems for settlement:

- TARGET2 / T2-T2S (the Eurosystem's consolidated
  platform) for euro-denominated settlement; the
  operator's TARGET2 participation requires
  Eurosystem central-bank account access.
- CHIPS for US-dollar large-value clearing among
  US-banking participants.
- Fedwire (Federal Reserve) for US-dollar real-time
  gross settlement; Fedwire Operating Circular 6
  governs the participation.
- The jurisdictional RTGS systems (BOK-Wire+ in KR,
  CHAPS in the UK, BOJ-NET in JP, etc.) for the
  domestic leg of the cross-border path.

The operator's PFMI compliance — Principles 1, 7, 8,
9, 20, 21, 22, 23 — is documented per system.

## §3 Correspondent-Bank Integration

For each correspondent the operator maintains:

- The bilateral correspondent-banking agreement.
- The correspondent's BIC and LEI.
- The correspondent's CBDDQ v1.4 response (PHASE-1
  §8) and the operator's review record.
- The bilateral channel — SWIFT, ISO 20022 over
  bilateral connectivity, or a third-party multi-
  bank network.
- The reconciliation cadence — daily for high-
  volume correspondents, weekly or monthly for
  lower-volume.
- The communication-procedure agreement for
  exception handling and investigations.

## §4 Financial-Intelligence Unit Integration

The operator's SAR / STR filing surface integrates
with the operating jurisdiction's FIU:

- US FinCEN — SAR filing through the BSA E-Filing
  System.
- KR KoFIU — STR filing through the KoFIU electronic
  reporting channel under 특정금융정보법.
- UK NCA-UKFIU — SAR filing through the SAR Online
  channel under POCA 2002.
- EU AMLA — for EU-wide cross-border patterns the
  AMLA cross-border filing channel applies after
  AMLA Regulation (EU) 2024/1620 takes effect.
- Other jurisdictions — the operator's compliance
  procedure documents the per-jurisdiction filing
  channel.

The FATF Recommendation 21 tipping-off discipline
restricts the operator's downstream disclosure.

## §5 Supervisory Authority and Central-Bank
       Integration

The supervisory financial-services authority and the
central bank for the operating jurisdiction are the
integration counterparties for examination, payment-
system access, and incident reporting:

- US — Office of the Comptroller of the Currency
  (OCC), Federal Reserve, FDIC, FinCEN,
  state banking regulators.
- EU — the Member-State NCA, the European Central
  Bank where the operator is significant, EBA where
  cross-border payment-services scope applies,
  AMLA for AML supervision under Regulation (EU)
  2024/1620.
- KR — Financial Services Commission (FSC), Financial
  Supervisory Service (FSS), Bank of Korea.
- UK — PRA / FCA, Bank of England.
- Other — the relevant prudential and conduct
  supervisor.

The operator's incident-reporting discipline (cyber-
incident, payment-system outage, sanctions-violation)
follows the supervisory authority's published
notification timeframes.

## §6 Customer-Channel Integration

The operator's customer channels include:

- The corporate cash-management portal (host-to-
  host file-based instruction submission, ISO 20022
  pain.001 customer-credit-transfer-initiation, and
  the corporate-portal browser surface).
- The retail remittance app (mobile-channel
  initiation, KYC capture, beneficiary management,
  and status visibility).
- The agent-network channel for cash-out scenarios
  (where the operator partners with money-services
  businesses to deliver beneficiary-side cash).

Customer authentication aligns with the operating
jurisdiction's strong-customer-authentication
requirements (PSD2 SCA in the EU, the equivalent
authentication discipline elsewhere).

## §7 CLS and PvP Settlement Integration

For FX-funding legs where payment-versus-payment
(PvP) settlement is used the operator integrates
with CLS (Continuous Linked Settlement) or the
equivalent PvP arrangement. CLS settlement reduces
Herstatt risk on the FX leg of the cross-border
payment path; the operator's settlement record
references the CLS settlement-instruction
identifier.

## §8 External Audit and ISMS Certification

The operator's information security management
system is certified against ISO/IEC 27001:2022 with
the scope explicitly extending to the SWIFT, ISO
20022, and customer-channel endpoints. The
certification body operates under ISO/IEC 17021-1;
the conformity-assessment body operates under ISO/
IEC 17065. The SWIFT Customer Security Programme
(CSP) self-attestation against the SWIFT Customer
Security Controls Framework (CSCF) applies for
SWIFT-connected operators.

## §9 Long-Term Archival Integration

Records governed by the operator's retention horizons
(FATF Recommendation 11 five years from the date of
transaction; US 31 CFR 1010.430 five years; EU AMLR
Article 46 five years extendable to ten; KR 특정금융
정보법 retention discipline) are migrated to the
long-term archive at the close of the active
retention window. The archive preserves the wire-
message record (with cryptographic digest), the
travel-rule record, the screening record, the CBDDQ
record, the SAR / STR record (preserving FATF
Recommendation 21 access restrictions), the
settlement record, and the audit-event trail.

## §10 Cross-Border Payments Roadmap and CBDC
        Interoperability

For operators participating in the CPMI cross-border
payments roadmap initiatives (the BIS-coordinated
programme on cross-border payment cost, speed,
access, and transparency) the operator's integration
includes:

- The Project mBridge multi-CBDC platform pilot
  participation, where applicable.
- Bilateral CBDC interoperability arrangements with
  the issuing central bank, where the operator
  integrates a wholesale-CBDC settlement leg.
- Conditional-payment / programmable-payment
  pilots, where the operator integrates a
  jurisdiction-specific CBDC programmable layer.

CBDC integration is governed by the issuing central
bank's published technical and policy framework; the
operator's records preserve the CBDC settlement
identifier alongside the conventional settlement
record.

## §11 ISO 20022 Migration and Coexistence

The operator's ISO 20022 migration discipline manages
the SWIFT industry coexistence cutover for cross-
border payments:

- The operator's migration plan documents the cutover
  date for outbound pacs.008 / pacs.009 (replacing
  outbound MT 103 / MT 202 / MT 202 COV) and the
  inbound translation shim that the operator
  operates while correspondents complete their own
  migrations.
- The translation shim preserves the Recommendation
  16 travel-rule fields without truncation; the EU
  Wire Transfer Regulation 2023/1113 Article 4
  fields are preserved end-to-end.
- The operator's SWIFT MT-to-ISO-20022 mapping
  follows the SWIFT-published market-practice
  guidance for charges, remittance information, and
  party identification.
- Reconciliation between the inbound MT-translated-
  to-ISO-20022 record and the operator's downstream
  ISO 20022 path is exercised on the daily
  reconciliation pass.

## §12 Operational Resilience and Cyber-Resilience

The operator's operational and cyber-resilience
discipline aligns with the operating jurisdiction's
supervisory expectations:

- US operators apply the Federal Banking Agencies
  Sound Practices to Strengthen Operational
  Resilience.
- EU operators apply the Digital Operational
  Resilience Act (DORA, Regulation (EU) 2022/2554)
  for ICT-third-party-risk management, incident
  reporting, threat-led penetration testing, and
  resilience testing.
- KR operators apply the Financial Information Act
  (전자금융거래법) and the FSC's electronic financial
  supervision regulations.
- SWIFT-connected operators apply the SWIFT Customer
  Security Programme (CSP) and submit annual self-
  attestations against the Customer Security
  Controls Framework (CSCF).
- Threat-led penetration testing on the cross-border
  payment estate is exercised on the supervisor's
  cadence (TIBER-EU framework in EU, the equivalent
  programme elsewhere).

## §13 Conformance

Implementations claiming PHASE-4 conformance maintain
the SWIFT / payment-system / correspondent-bank
integrations, exercise the FIU and supervisory-
authority filing obligations, expose the customer
channels with the operating jurisdiction's strong-
customer-authentication discipline, hold the ISO/IEC
27001 certification, and operate the long-term
archival integration described above.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-cross-border-payment
- **Last Updated:** 2026-04-28
