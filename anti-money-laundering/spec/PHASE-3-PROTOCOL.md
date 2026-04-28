# WIA-anti-money-laundering PHASE 3 — PROTOCOL Specification

**Standard:** WIA-anti-money-laundering
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern an
obliged entity under the FATF 40 Recommendations as
transposed into the operating jurisdiction's law: the
risk-based-approach discipline (FATF Recommendation 1),
the customer due diligence discipline (Recommendation
10), the higher-risk-customer / PEP discipline
(Recommendation 12), the correspondent-banking
discipline (Recommendation 13), the new-technologies
discipline (Recommendation 15) including virtual-asset
service providers, the wire-transfer discipline
(Recommendation 16, "Travel Rule"), the suspicious-
transaction-reporting discipline (Recommendation 20),
the tipping-off prohibition (Recommendation 21), the
DNFBP discipline (Recommendations 22 and 23), the
beneficial-ownership-transparency discipline
(Recommendations 24 and 25), the sanctions-screening
discipline (Recommendations 6 and 7), and the
supervisor and FIU correspondence discipline.

References (CITATION-POLICY ALLOW only):

- ISO 9001:2015 (quality management systems)
- ISO/IEC 27001:2022 (information security management)
- ISO 8601 (date and time)
- ISO 20022 (financial-services messaging)
- ISO 17442 (LEI)
- IETF RFC 5905 (NTPv4)
- IETF RFC 9457 (Problem Details)
- FATF 40 Recommendations
- FATF Methodology
- FATF Recommendation 6 (UN Security Council resolutions
  against terrorism financing) and Recommendation 7
  (UN Security Council resolutions against
  proliferation financing)
- FATF Recommendation 10 (CDD), 11 (record retention),
  12 (PEPs), 13 (correspondent banking), 15 (new
  technologies / VASPs), 16 (wire transfers / Travel
  Rule), 18 (internal controls), 20 (STR), 21 (tipping
  off prohibition), 22 (DNFBP CDD), 23 (DNFBP STR),
  24 (legal-person beneficial ownership), 25 (legal-
  arrangement beneficial ownership)
- US Bank Secrecy Act (BSA) and 31 CFR Chapter X
- US OFAC SDN and consolidated sanctions lists
- EU AML Regulation (EU) 2024/1624 and EU Authority
  Regulation (EU) 2024/1620 (AMLA)
- Directive (EU) 2018/1673 on combating money
  laundering by criminal law
- KR Specific Financial Information Act
- KR Act on Prohibition of Financing of Terrorism
- UK MLR 2017 (Money Laundering, Terrorist Financing
  and Transfer of Funds (Information on the Payer)
  Regulations 2017)
- Wolfsberg Group Anti-Money Laundering Principles
  (cited as a community-recognised cross-border
  banking baseline; non-binding)

---

## §1 Risk-Based-Approach Discipline (FATF Recommendation 1)

The obliged entity's AML/CFT programme is calibrated to
the risks the obliged entity faces. The discipline:

- enterprise-wide risk assessment (EWRA) covering
  customer types, geographic exposure, products and
  services, delivery channels;
- per-customer risk rating (PHASE-1 §3 `riskRating`)
  applied as the basis for CDD intensity;
- per-product risk assessment (private banking,
  correspondent banking, virtual-asset products,
  cross-border wires carry inherently elevated risk);
- per-jurisdiction geographic risk assessment using
  FATF's high-risk and other monitored jurisdictions
  list as one input alongside the obliged entity's
  internal geographic risk model;
- annual EWRA refresh and event-triggered refresh
  (regulatory change, new product launch, FATF list
  update, a major sanctions designation that affects
  a corridor).

## §2 Customer Due Diligence Discipline (Recommendation 10)

CDD is performed at:

- onboarding;
- periodically per the customer's risk rating;
- on event triggers (suspicious activity, beneficial-
  ownership change, sanctions hit, regulatory inquiry,
  customer change of jurisdiction);
- when there is doubt about the veracity or adequacy
  of previously obtained data.

CDD scope: identification of the customer; verification
of identity using reliable, independent source
documents, data or information; identification of the
beneficial owner; understanding the purpose and
intended nature of the business relationship; ongoing
monitoring.

For natural persons the obliged entity captures legal
name, date of birth, residential address, nationality,
and a government-issued identification document. For
legal persons the obliged entity captures legal name,
registered address, principal place of business, the
legal form, the proof of existence (extract from a
public register), the powers that regulate and bind
the legal person, and the natural-person beneficial
owner identity. The Recommendation 24/25 beneficial-
owner threshold is the operating jurisdiction's
adopted threshold (the EU AML Regulation 2024/1624
sets a 25% threshold; the operating jurisdiction's
implementation governs).

## §3 Higher-Risk and PEP Discipline (Recommendation 12)

PEPs (per FATF Recommendation 12) are subject to
enhanced due diligence:

- senior-management approval for establishing or
  continuing the relationship;
- enhanced source-of-wealth and source-of-funds
  verification;
- enhanced ongoing monitoring of the relationship.

The PEP discipline applies to family members and
known close associates of PEPs. Domestic PEP / foreign
PEP / international-organisation PEP distinctions
follow the operating jurisdiction's transposition
(some jurisdictions apply only foreign-PEP enhancement
while some apply EDD to all PEP categories).

## §4 Correspondent-Banking Discipline (Recommendation 13)

Cross-border correspondent banking carries heightened
risk because the obliged entity does not have direct
visibility into the respondent bank's customers. The
discipline:

- prohibition on shell-bank correspondent relationships
  per Recommendation 13;
- documented respondent-bank AML attestation review;
- senior-management approval before establishing the
  relationship;
- enhanced ongoing monitoring of the relationship's
  payment volume and corridor risk;
- per-relationship review on a documented cadence
  (typically annual) and on event-triggered re-review.

The Wolfsberg Group's Correspondent Banking Due
Diligence Questionnaire (CBDDQ) is widely used as the
respondent-attestation template; the discipline is the
attestation review, not a specific template.

## §5 New-Technologies Discipline (Recommendation 15)

VASPs and the obliged entity's interaction with VASPs
follow Recommendation 15:

- VASPs are subject to AML/CFT supervision in the
  operating jurisdiction (per the operating
  jurisdiction's VASP licensing regime — KR's
  Specific Financial Information Act VASP regime;
  the EU's MiCA/AMLR VASP framework; FinCEN's MSB
  VASP framework in the US);
- the obliged entity's interaction with VASPs is
  treated as enhanced-risk and triggers EDD;
- on-chain transactions carry the Travel Rule fields
  per Recommendation 16 cross-walk, with the wallet
  address representing the account number.

## §6 Wire-Transfer Discipline (Recommendation 16, "Travel Rule")

Cross-border wire transfers above the operating
jurisdiction's de minimis threshold (the FATF
recommendation cites USD/EUR 1,000 as the suggested
de minimis; the operating jurisdiction's threshold
governs) carry:

- originator information (name, account number /
  wallet address, address);
- beneficiary information (name, account number /
  wallet address);
- the originator and beneficiary information
  accompanies the transaction across the chain (at
  the originator's bank, intermediary banks, and the
  beneficiary's bank).

For the SWIFT network, the originator and beneficiary
fields appear in MT 103 fields 50 and 59 (for the
direct customer credit transfer) and in MT 202 COV
fields 50 and 59 of the underlying customer credit
transfer (for cover payments). The MT-to-ISO 20022
migration moves the fields to pacs.008 / pacs.009
structured fields.

## §7 Suspicious-Transaction Discipline (Recommendation 20)

When the obliged entity suspects, or has reasonable
grounds to suspect, that funds are the proceeds of a
criminal activity, or are related to terrorism
financing, the obliged entity files a suspicious-
transaction report ("STR" in EU/KR; "SAR" in US/UK).
The discipline:

- a clear internal escalation path from frontline
  staff and transaction monitoring to the compliance
  officer / MLRO who decides on the filing;
- the filing is made promptly per the operating
  jurisdiction's deadline (e.g. FinCEN BSA SAR within
  30 days of detection; KoFIU STR without delay);
- the content includes the customer relationship, the
  specific transactions, the suspicion basis, and the
  hypothesised predicate offense;
- the filing is preserved per the FIU's required
  retention.

## §8 Tipping-Off Prohibition (Recommendation 21)

The obliged entity does not disclose to the customer
or to any third party (other than the FIU and law
enforcement under the operating jurisdiction's
information-sharing rules) the fact that an STR / SAR
has been filed or that an investigation case has
opened. The tipping-off prohibition is encoded as an
access-control invariant in the API (PHASE-2 §7 and
§10). Information-sharing within the same financial
group is permitted under Recommendation 18 and the
operating jurisdiction's transposition (e.g. the EU
AMLR 2024/1624 group-information-sharing provisions).

## §9 Sanctions Screening Discipline (Recommendations 6 and 7)

Recommendation 6 obligates implementation of UN
Security Council targeted financial sanctions
relating to terrorism and terrorism financing;
Recommendation 7 obligates implementation of UN
Security Council targeted financial sanctions
relating to proliferation financing. The obliged
entity's sanctions discipline:

- per-customer screening at onboarding and on each
  CDD refresh;
- per-transaction screening for cross-border
  transactions;
- per-list screening across UN, the operating
  jurisdiction's national list, and the operating
  jurisdiction's coalition lists (US OFAC, EU
  consolidated, UK OFSI, KR MOFA);
- "without delay" freezing on confirmed match per
  Recommendation 6 / 7 — the obliged entity does not
  await court order before applying the freeze;
- false-positive review with documented rationale
  (sanctions screening is a noisy signal — name-
  collision frequency drives a high false-positive
  rate that the obliged entity manages with secondary
  data points like date of birth and address).

## §10 Records Retention (Recommendation 11)

CDD records, transaction records, sanctions screenings,
STR / SAR filings, and supervisor correspondence
retain for at least five years after the end of the
business relationship or the date of the transaction —
the FATF Recommendation 11 baseline — extended where
the operating jurisdiction's law requires longer
retention or where law enforcement requests preservation
beyond the baseline.

## §11 Time Synchronisation

Operator clocks synchronise per RFC 5905 (NTPv4) so
that transaction-monitoring rule windows, the
operating jurisdiction's STR / SAR filing deadlines
(e.g. FinCEN's 30-day SAR filing deadline), and
"without delay" freezing under Recommendations 6 and 7
are consistent across the obliged entity's runtime
fleet.

## §12 DNFBP Discipline (Recommendations 22 and 23)

DNFBPs (casinos, real-estate agents, dealers in
precious metals and stones, lawyers and notaries
within the scope of the FATF guidance, accountants,
trust-and-company-service providers) follow the same
risk-based AML/CFT discipline as financial-institution
obliged entities, with adaptation for the sector's
business model (e.g. real-estate-transaction triggers,
lawyer/notary professional-privilege carve-out per the
operating jurisdiction's transposition).

## §13 Beneficial-Ownership-Transparency Discipline (Recommendations 24 and 25)

Legal-person and legal-arrangement beneficial-
ownership identification follows Recommendations 24
and 25:

- the obliged entity captures the natural-person
  beneficial-owner identity at onboarding and on each
  ownership change;
- the obliged entity uses the operating jurisdiction's
  beneficial-ownership register where one exists (EU
  AML Regulation 2024/1624 requires Member State
  central registers; the UK Companies House Register
  of People with Significant Control; FinCEN's
  Beneficial Ownership Information Reporting under
  the Corporate Transparency Act in the US);
- the obliged entity does not rely solely on the
  register's content but uses it as one of several
  inputs.

## §14 Internal-Controls Discipline (Recommendation 18)

Per Recommendation 18 the obliged entity maintains:

- written internal AML/CFT policies and procedures;
- senior-management designation of an AML compliance
  officer / MLRO with the seniority and access to
  effectively discharge the role;
- ongoing employee training on the AML/CFT programme;
- independent audit function — internal audit or an
  external auditor — testing the AML/CFT programme;
- pre-employment screening of staff in sensitive
  functions.

## §15 Supervisor and FIU Correspondence Discipline

The obliged entity's correspondence discipline:

- per-supervisor inquiry response per the supervisor's
  response SLA;
- per-supervisor on-site examination cooperation;
- per-supervisor enforcement-action response with
  documented remediation plan;
- per-FIU information-request response under the
  operating jurisdiction's information-sharing rules;
- per-FIU feedback consumption (FIUs publish typology
  reports and feedback on STR / SAR quality that the
  obliged entity uses to refine transaction
  monitoring).

## §16 Quality Dossier and Conformance

The obliged entity's AML/CFT quality dossier records
the governing frameworks, the EWRA, the AML
programme document, the MLRO appointment, the FIU
filing inventory, the supervisor correspondence, the
sanctions-list-update calendar, the transaction-
monitoring rule library, the DNFBP-class adaptations
(if applicable), and the audit-cycle outcomes. The
dossier is reviewed at least annually by the
compliance officer / MLRO and is provided to the
supervisor on request.

A programme conformant with WIA-anti-money-laundering
publishes its MLRO contact, its public AML/CFT
disclosure where the operating jurisdiction requires
it (e.g. the UK MLR 2017 sectoral disclosures), and
answers an annual self-assessment that maps each
clause of this PHASE to the obliged entity's
implementation.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-anti-money-laundering
- **Last Updated:** 2026-04-28
