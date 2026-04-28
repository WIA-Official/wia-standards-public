# WIA-cross-border-payment PHASE 3 — PROTOCOL Specification

**Standard:** WIA-cross-border-payment
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern a
cross-border payment operator: the FATF Recommendation
16 travel-rule discipline that ensures the originator's
and beneficiary's identification fields travel with
every wire; the correspondent-banking due-diligence
discipline (FATF Recommendation 13, Wolfsberg CBDDQ);
the sanctions-screening and OFAC discipline; the
suspicious-transaction reporting discipline (FATF
Recommendations 20 / 21); the settlement-finality
discipline (CPMI PFMI Principle 8); the SWIFT GPI
tracking discipline; the SEPA / EU-internal-area
discipline that gates EU-area transfers; and the post-
incident reconciliation and continuity-of-payments
discipline.

References (CITATION-POLICY ALLOW only):

- ISO 9001:2015 (quality management systems)
- ISO/IEC 27001:2022 (information security management)
- ISO 20022, ISO 4217, ISO 3166-1, ISO 9362, ISO
  13616, ISO 17442
- IETF RFC 5905 (NTPv4), RFC 9421 (HTTP Message
  Signatures), RFC 9457 (Problem Details)
- CPMI Principles for Financial Market
  Infrastructures (PFMI) — Principle 1 legal basis,
  Principle 7 liquidity risk, Principle 8 settlement
  finality, Principle 9 money settlement, Principle
  20 FMI links, Principle 21 efficiency and
  effectiveness, Principle 22 communication
  procedures and standards, Principle 23 disclosure
  of rules
- CPMI cross-border payments roadmap
- FATF Recommendations 1, 10, 11, 13, 16, 20, 21,
  22, 25
- FATF Methodology (the assessment methodology for
  Recommendations 16 / 13 / 20)
- Wolfsberg CBDDQ v1.4 + Wolfsberg AML Principles
  for Correspondent Banking
- US BSA + 31 CFR Chapter X (FinCEN)
- US OFAC SDN + non-SDN programmes; OFAC 50%-rule
- EU AMLR (Regulation (EU) 2024/1624) Articles 36
  to 41 (correspondent-relationship CDD), Article
  46 (record-keeping)
- EU AMLA Regulation (Regulation (EU) 2024/1620)
- EU Wire Transfer Regulation (Regulation (EU)
  2023/1113) Articles 4 to 8
- EU SEPA Regulation (Regulation (EU) No 260/2012)
- EU Cross-Border Payments Regulation (Regulation
  (EU) 2021/1230)
- KR 특정금융정보법 + KR KoFIU reporting discipline
- UK MLR 2017 + NCA-UKFIU SAR reporting
- TARGET2 + T2-T2S settlement finality (Settlement
  Finality Directive 98/26/EC implementation)
- CHIPS Rules + Fedwire Operating Circular 6

---

## §1 Travel-Rule Discipline (FATF Recommendation 16)

The travel-rule discipline ensures the originator's
and beneficiary's identification fields travel with
the instruction throughout the correspondent chain:

1. The operator validates that the originator's name,
   account identifier, address (or, in lieu of
   address, date-of-birth and unique customer
   identifier), and the beneficiary's name and
   account identifier are present at submission.
2. For EU-side transfers above EUR 1 000 the EU Wire
   Transfer Regulation 2023/1113 Article 4 mandates
   the originator's address and the beneficiary's
   address; the operator validates the additional
   fields.
3. For US-side transfers FinCEN's Recordkeeping and
   Travel Rule (31 CFR 1010.410(e) and (f)) imposes
   the equivalent discipline.
4. The wire message persists the travel-rule fields
   in the canonical pacs.008 / pacs.009 / MT 103 /
   MT 202 fields; intermediaries forward the fields
   without alteration.
5. Inbound messages with missing fields are
   remediated through the operator's travel-rule
   completeness procedure (request additional
   information from the previous correspondent;
   reject the inbound where remediation is not
   feasible).

## §2 Correspondent-Banking Due-Diligence Discipline

The operator's CBDDQ discipline aligns with FATF
Recommendation 13:

- For each correspondent the operator collects the
  Wolfsberg CBDDQ v1.4 questionnaire response and
  the supporting evidence (the correspondent's AML
  programme, sanctions programme, ownership
  structure, regulatory licensing).
- The operator's compliance officer reviews the
  CBDDQ and approves the correspondent relationship,
  approves with conditions (enhanced due diligence
  or sub-set of services), or declines.
- The CBDDQ is refreshed on a cadence aligned with
  the correspondent's risk rating; high-risk
  correspondents are refreshed annually, lower-risk
  on the operator's published cadence.
- Nested correspondent relationships (a third
  correspondent accessing the operator's services
  through the operator's direct correspondent) are
  reviewed under FATF Recommendation 13 and the EU
  AMLR Article 38 nested-relationships discipline.

## §3 Sanctions-Screening Discipline

The sanctions-screening discipline:

- Originator and beneficiary names, account
  identifiers, and address fields are screened
  against US OFAC SDN and non-SDN programmes; UN
  consolidated sanctions list; EU FSF; UK HMT; and
  any jurisdictional sanctions programme that
  applies.
- Screening occurs at submission, before settlement,
  and on the post-settlement reconciliation pass for
  late-added designations.
- Potential matches are routed to the operator's
  compliance review queue; the four-eyes principle
  applies — the reviewer who clears a potential
  match cannot be the reviewer who approved the
  customer at onboarding.
- Confirmed matches are blocked; the operator
  preserves the funds per the OFAC blocking
  discipline (US) or the equivalent jurisdictional
  freeze and reports to the supervisory authority.
- The OFAC 50% rule (entities owned 50% or more by
  designated parties) applies; the operator's
  ownership-resolution discipline searches for
  beneficial-ownership data.

## §4 Suspicious-Transaction Reporting Discipline

The SAR / STR discipline:

- Front-line staff or the operator's transaction-
  monitoring system surfaces a suspicious pattern
  (structuring, unusual geography, unusual
  beneficiary, unexplained urgency, mismatched
  identification).
- The operator's compliance officer reviews and —
  where suspicion is supported — files the SAR / STR
  to the FIU (US FinCEN, KR KoFIU, UK NCA-UKFIU,
  EU AMLA-cross-border).
- The FATF Recommendation 21 tipping-off discipline
  applies — the operator does not disclose the SAR
  / STR to the customer or to a third party.
- The operator preserves the SAR / STR record and
  the underlying transaction record for the
  jurisdictional retention horizon (typically five
  years from the date of the report).

## §5 Settlement-Finality and Liquidity Discipline

The settlement-finality discipline aligns with CPMI
PFMI Principle 8:

- Settlement on TARGET2, CHIPS, or Fedwire is final
  and irrevocable on the system's posted finality
  rules (TARGET2 / Settlement Finality Directive
  98/26/EC implementation; CHIPS Rules; Fedwire
  Operating Circular 6).
- The operator's liquidity-risk discipline (CPMI
  PFMI Principle 7) ensures sufficient intraday
  liquidity to meet payment-system obligations.
- For provisional settlement (correspondent banking
  with end-of-day net settlement) the operator's
  reconciliation pass converts provisional to final
  on the next-day settlement cycle.

## §6 SWIFT GPI Tracking Discipline

For SWIFT-connected operators the SWIFT GPI tracking
discipline applies:

- Every customer credit transfer carries a UETR
  (Unique End-to-end Transaction Reference, RFC 4122
  UUIDv4 syntax).
- Each correspondent updates the GPI tracker with
  its leg of the transfer (received, processed,
  forwarded, paid, returned).
- The originator and the beneficiary can query the
  end-to-end status through their respective banks'
  GPI-enabled customer channels.

## §7 EU SEPA and Intra-Area Discipline

For euro-denominated EU intra-area transfers:

- SEPA Regulation (EU) No 260/2012 imposes the
  IBAN-only and BIC-optional discipline; the
  operator's SEPA processing path is distinct from
  the cross-border-outside-SEPA path.
- EU Cross-Border Payments Regulation (EU) 2021/1230
  Article 3 mandates that euro cross-border charges
  to consumers be no higher than equivalent domestic
  charges.
- For non-euro SEPA-area currencies the operator's
  SEPA processing path applies the regulatory
  pricing transparency disciplines under Articles
  3a and 3b (currency-conversion charges
  disclosure).

## §8 KR / UK / Other Jurisdictional Discipline

- KR-jurisdiction operators apply 특정금융정보법 reporting
  to KoFIU; the FIU's reporting wire format is the
  operator's outbound channel.
- UK-jurisdiction operators apply MLR 2017 + NCA-
  UKFIU SAR reporting and the equivalent screening
  against UK HMT.
- Other jurisdictions apply their own AML/CFT primary
  statute; the operator's compliance procedure
  documents the per-jurisdiction reporting
  obligations.

## §9 Identity, Time, and Audit Discipline

NTPv4 stratum-2 or better is the operator's clock
baseline. Audit-events are emitted for every
instruction submission, screening pass, CBDDQ review,
SAR / STR filing, settlement event, and investigation
resolution. Audit logs are integrity-protected per
the operator's tamper-evident mechanism.

## §10 Continuity-of-Payments Discipline

The operator's continuity discipline addresses
correspondent-bank failure, payment-system outage,
and cyber-incident scenarios:

- The operator's RTO / RPO targets for the cross-
  border payment service align with the operating
  jurisdiction's supervisory expectations.
- Alternate correspondent routing is documented and
  exercised on the operator's drill cadence.
- Cyber-incident response follows the operator's
  CSIRT plan and includes notification to the
  payment system, the operator's correspondents, and
  the supervisory authority on the published
  notification timeframes.

## §11 Conformance

Implementations claiming PHASE-3 conformance enforce
the discipline at every relevant decision point,
maintain the CBDDQ records on the Wolfsberg cadence,
file SAR / STR reports within the statutory window,
exercise the OFAC 50% rule, and preserve the records
on the FATF Recommendation 11 record-keeping discipline
(five years from the date of the transaction or the
date of the relationship's termination, whichever is
later).

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-cross-border-payment
- **Last Updated:** 2026-04-28
