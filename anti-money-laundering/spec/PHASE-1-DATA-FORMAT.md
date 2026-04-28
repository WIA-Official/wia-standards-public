# WIA-anti-money-laundering PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-anti-money-laundering
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format layer for
WIA-anti-money-laundering. The standard covers persistent
record shapes for financial institutions and other
obliged entities subject to anti-money-laundering and
counter-terrorist-financing (AML/CFT) obligations under
the Financial Action Task Force (FATF) 40
Recommendations as transposed into the operating
jurisdiction's legal regime — customer due diligence
(CDD), enhanced due diligence (EDD) for higher-risk
relationships, beneficial-ownership identification,
politically-exposed-person (PEP) screening, sanctions
screening (UN, EU, OFAC, HM Treasury, MOFA-KR),
transaction monitoring, suspicious-transaction reports
(STR / SAR), currency-transaction reports (CTR), and
correspondent-banking relationships. The format is
consumed by the obliged entity's compliance officer,
the operating jurisdiction's financial-intelligence
unit (FIU), the operating jurisdiction's AML/CFT
supervisor, and the obliged entity's external auditors.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time representation)
- ISO/IEC 11578 (UUID)
- ISO/IEC 27001:2022 (information security management)
- ISO 20022 (financial-services messaging; the
  successor to MT in cross-border payments and
  securities messaging)
- ISO 17442 (Legal Entity Identifier — LEI)
- ISO 4217 (currency codes)
- IETF RFC 4122 (UUID URN)
- IETF RFC 8259 (JSON)
- IETF RFC 9457 (Problem Details)
- FATF 40 Recommendations (cited normatively for the
  international AML/CFT framework)
- FATF Methodology for Assessing Technical Compliance
  with the FATF Recommendations and the Effectiveness
  of AML/CFT Systems
- US Bank Secrecy Act (BSA) and 31 CFR Chapter X
  (FinCEN regulations); cited where the operating
  jurisdiction is the United States
- US OFAC Specially Designated Nationals (SDN) and
  consolidated sanctions lists (cited as the canonical
  US sanctions screening source)
- EU AML Regulation (EU) 2024/1624; EU AMLD6 framework
  regulation (Regulation (EU) 2024/1620 establishing
  the Authority for Anti-Money Laundering and
  Countering the Financing of Terrorism, AMLA);
  Directive (EU) 2018/1673 on combating money
  laundering by criminal law
- EU consolidated sanctions list (cited as the
  canonical EU restrictive-measures source)
- KR Specific Financial Information Act
  ("특정금융정보법"; cited where the operating
  jurisdiction is Korea, as the legal basis for the
  Korea Financial Intelligence Unit (KoFIU) reporting
  regime)
- KR Act on Prohibition of Financing of Terrorism
  (cited for Korea CFT obligations)
- UK Money Laundering, Terrorist Financing and Transfer
  of Funds (Information on the Payer) Regulations 2017
  (the "MLR 2017"; cited where the operating
  jurisdiction is the UK)
- SWIFT MT 103 (single customer credit transfer)
- SWIFT MT 202 / MT 202 COV (general financial
  institution transfer / cover payment); cited where
  the operating environment uses MT in parallel with
  ISO 20022
- ISO 20022 pacs.008 / pacs.009 / camt.* (cited as the
  successor to MT messaging on the SWIFT network's
  MT-to-ISO 20022 migration timetable)
- W3C Verifiable Credentials Data Model 2.0 (used in
  PHASE-4 for optional attestation re-issuance)

---

## §1 Scope

This PHASE defines persistent shapes for the artefacts
an obliged entity manages under the FATF 40
Recommendations as transposed into the operating
jurisdiction's law. Implementations covered include:

- Banks and other deposit-taking institutions.
- Money-transmitter and payment-service-provider
  obliged entities.
- Securities brokers and asset managers.
- Insurance underwriters offering life-insurance and
  investment-linked products.
- Designated non-financial businesses and professions
  (DNFBPs): casinos, real-estate agents, dealers in
  precious metals and stones, lawyers, accountants,
  trust-and-company-service providers (per FATF
  Recommendation 22).
- Virtual-asset service providers (VASPs) per FATF
  Recommendation 15.

Cross-border wire-transfer information requirements per
FATF Recommendation 16 ("Travel Rule") and the
analogous EU Wire Transfer Regulation are encoded in
PHASE-3 §6; this PHASE addresses the record shapes.

## §2 Programme Identifier

```
programmeId          : string (uuidv7)
obligedEntityName    : string (legal name of the
                       obliged entity)
legalEntityIdentifier : string (ISO 17442 LEI; obliged
                       entities operating in
                       jurisdictions that mandate LEI
                       under the FSB recommendation)
operatingJurisdiction : array of string (ISO 3166-1
                       country codes of the obliged
                       entity's operations)
governingFrameworks  : array of enum ("FATF-40-Recs" |
                       "US-BSA-31-CFR-X" |
                       "EU-AMLD6-Reg-2024-1624" |
                       "KR-Specific-Financial-
                       Information-Act" |
                       "UK-MLR-2017" |
                       "user-defined")
mlroReference        : string (the Money Laundering
                       Reporting Officer's contact
                       reference, per FATF
                       Recommendation 18 / national
                       equivalent — KR's compliance
                       officer designated under the
                       Specific Financial Information
                       Act)
fiuReference         : string (operating jurisdiction's
                       FIU identity — FinCEN for US,
                       KoFIU for KR, the operating
                       Member State's FIU for EU,
                       NCA-UKFIU for UK)
amlSupervisor        : string (operating jurisdiction's
                       AML/CFT supervisor — FinCEN +
                       Federal Reserve / OCC / FDIC /
                       SEC / CFTC for US per the
                       obliged entity's primary
                       prudential regulator; AMLA for
                       EU once operational; FSC + FSS
                       for KR; FCA / HMRC / Gambling
                       Commission for UK per the
                       obliged entity's class)
programmeStatus      : enum ("design" | "operating" |
                       "limited-rollout" |
                       "wind-down" | "archived")
```

## §3 Customer Due Diligence (CDD) Record

Per FATF Recommendation 10. Record shape:

```
cddRecord:
  customerId         : string (uuidv7)
  programmeId        : string (uuidv7)
  customerKind       : enum ("natural-person" |
                       "legal-person" | "trust" |
                       "partnership" | "non-profit" |
                       "government-or-soe")
  identificationDocs : array of object (each carrying
                       document kind, issuing
                       authority, document number, and
                       a content-addressed reference to
                       the verified copy)
  identityVerifiedAt : string (ISO 8601)
  riskRating         : enum ("simplified-low" |
                       "standard" | "enhanced-high")
                       (per FATF Recommendation 10
                       risk-based approach)
  beneficialOwners   : array of object (per FATF
                       Recommendation 24/25 for legal
                       persons / arrangements; each
                       entry carries the natural-
                       person identity, the ownership
                       or control percentage, and the
                       basis for the determination)
  pepStatus          : enum ("not-pep" |
                       "domestic-pep" |
                       "foreign-pep" |
                       "international-organisation-pep"
                       | "pep-family-member" |
                       "pep-close-associate")
  sanctionsScreeningRef : string (URI of the most-
                       recent sanctions screening
                       result)
  sourceOfFunds      : string (URI of the documented
                       source-of-funds narrative; for
                       enhanced-high risk relationships
                       per FATF Recommendation 10
                       enhanced CDD)
  ongoingMonitoringFrequency : enum ("monthly" |
                       "quarterly" | "semi-annual" |
                       "annual" | "event-triggered")
  refreshDueAt       : string (ISO 8601)
```

## §4 Enhanced Due Diligence (EDD) Annotation

For enhanced-high risk relationships (PEPs, high-risk
third countries per FATF, complex unusually large
transactions without an apparent economic purpose,
correspondent banking with respondent banks in higher-
risk jurisdictions):

```
eddAnnotation:
  customerRef        : string (CDD record reference)
  triggeringFactor   : enum ("pep-status" |
                       "high-risk-third-country" |
                       "complex-unusual-transaction" |
                       "correspondent-banking-respondent"
                       | "private-banking" |
                       "vasp-counterparty" |
                       "non-face-to-face-onboarding" |
                       "user-defined")
  seniorManagementApprovalRef : string (URI of the
                       senior-management approval per
                       FATF Recommendation 12 for PEPs)
  enhancedSourceOfWealthRef : string (URI of the
                       documented source-of-wealth
                       narrative)
  enhancedMonitoringPlanRef : string (URI of the
                       enhanced ongoing-monitoring
                       plan)
```

## §5 Sanctions Screening Record

```
sanctionsScreening:
  screeningId        : string (uuidv7)
  subjectRef         : string (CDD record or
                       transaction record reference)
  screeningTimestamp : string (ISO 8601)
  listsScreened      : array of enum ("UN-Security-
                       Council-Consolidated-Sanctions"
                       | "EU-Consolidated-Sanctions" |
                       "US-OFAC-SDN" |
                       "US-OFAC-non-SDN-Sectoral" |
                       "UK-HM-Treasury-OFSI-
                       Consolidated" |
                       "KR-MOFA-Sanctions-List" |
                       "operator-internal-watchlist" |
                       "user-defined")
  matchKind          : enum ("no-match" | "potential-
                       match-pending-review" |
                       "confirmed-match-blocked" |
                       "confirmed-false-positive")
  reviewerRef        : string (compliance reviewer
                       reference; absent for no-match)
  decisionRationaleRef : string (URI of the documented
                       review rationale; absent for
                       no-match)
```

## §6 Transaction Record

```
transaction:
  transactionId      : string (uuidv7)
  programmeId        : string (uuidv7)
  customerRef        : string (CDD record reference)
  counterpartyRef    : string (counterparty CDD record
                       reference if known; otherwise
                       opaque counterparty identity)
  bookedAt           : string (ISO 8601)
  amount             : object (currency per ISO 4217 +
                       amount + fx rate at booking)
  rail               : enum ("swift-mt-103" |
                       "swift-mt-202" |
                       "swift-mt-202-cov" |
                       "iso-20022-pacs-008" |
                       "iso-20022-pacs-009" |
                       "domestic-rtgs" |
                       "domestic-ach" |
                       "card-network" |
                       "virtual-asset-on-chain" |
                       "cash" |
                       "user-defined")
  travelRuleFields   : object (FATF Recommendation 16
                       wire-transfer information —
                       originator and beneficiary name,
                       account number / wallet address,
                       physical address; cross-border
                       transactions above the operating
                       jurisdiction's de minimis
                       threshold include all fields)
  monitoringRulesHit : array of string (transaction-
                       monitoring rule identifiers
                       that flagged the transaction)
  caseRef            : string (URI of the alert-and-
                       case record if the transaction
                       triggered a case; absent
                       otherwise)
```

## §7 Suspicious-Transaction Report (STR / SAR) Record

```
suspiciousTransactionReport:
  reportId           : string (uuidv7)
  programmeId        : string (uuidv7)
  filedAt            : string (ISO 8601)
  fiuRef             : enum ("us-fincen-sar" |
                       "kr-kofiu-str" |
                       "eu-member-state-fiu" |
                       "uk-nca-ukfiu" |
                       "user-defined")
  fiuFiledAcknowledgement : string (FIU-issued
                       acknowledgement reference;
                       absent until FIU acknowledges)
  triggerKind        : enum ("transaction-monitoring-
                       rule" | "compliance-officer-
                       review" | "frontline-staff-
                       referral" | "law-enforcement-
                       request" | "external-tip" |
                       "user-defined")
  underlyingPredicateOffenseHypothesis : string
                       (compliance officer's
                       hypothesised predicate offense
                       under the operating
                       jurisdiction's predicate-
                       offense list — drug-trafficking,
                       fraud, tax-evasion, corruption,
                       terrorism-financing, etc.;
                       hypothesis only — adjudication
                       is the FIU's and law
                       enforcement's responsibility)
  reportNarrativeRef : string (URI of the redacted
                       narrative; the narrative cites
                       the specific transactions, the
                       customer relationship, and the
                       compliance officer's
                       suspicion basis)
  tippingOffPrecaution : boolean (true once the
                       compliance officer has
                       confirmed no tipping-off has
                       occurred per FATF
                       Recommendation 21)
```

## §8 Currency-Transaction Report (CTR) Record

For jurisdictions that mandate CTR filing above a de
minimis threshold (US BSA $10,000, KR over the
operating jurisdiction's threshold per the Specific
Financial Information Act):

```
ctrRecord:
  ctrId              : string (uuidv7)
  programmeId        : string (uuidv7)
  filedAt            : string (ISO 8601)
  transactionRef     : string (transaction record
                       reference)
  thresholdJurisdiction : string (ISO 3166-1; the
                       jurisdiction whose threshold
                       applies)
  fiuFiledAcknowledgement : string (FIU acknowledgement
                       reference; absent until FIU
                       acknowledges)
```

## §9 Correspondent-Banking Relationship Record

Per FATF Recommendation 13:

```
correspondentBanking:
  relationshipId     : string (uuidv7)
  programmeId        : string (uuidv7)
  respondentBankIdentity : string (respondent legal
                       entity identifier)
  respondentJurisdiction : string (ISO 3166-1)
  relationshipKind   : enum ("nested-cover-payment-
                       eligible" | "vostro-only" |
                       "nostro-only" | "two-way")
  respondentAmlAttestationRef : string (URI of the
                       respondent's AML programme
                       attestation; the obliged entity
                       reviews the attestation per its
                       correspondent-banking
                       discipline)
  shellBankPolicy    : enum ("respondent-not-shell" |
                       "respondent-may-permit-shell-
                       use") (per FATF Recommendation
                       13 the obliged entity declines
                       shell-bank correspondent
                       relationships)
  seniorManagementApprovalRef : string (URI of the
                       senior-management approval
                       record)
  reviewCadence      : enum ("annual" | "biennial" |
                       "ad-hoc-on-event")
```

## §10 Investigation Case Record

```
investigationCase:
  caseId             : string (uuidv7)
  programmeId        : string (uuidv7)
  openedAt           : string (ISO 8601)
  caseKind           : enum ("transaction-monitoring-
                       alert" | "sanctions-hit" |
                       "compliance-officer-referral"
                       | "law-enforcement-inquiry" |
                       "user-defined")
  customerRef        : string (CDD record reference)
  alertedTransactions : array of string (transaction
                       record references)
  caseStatus         : enum ("open" |
                       "pending-additional-evidence" |
                       "closed-no-suspicion" |
                       "closed-str-filed" |
                       "closed-customer-exited")
  exitDecisionRef    : string (URI of the customer-
                       exit decision narrative; absent
                       unless the obliged entity
                       declined to continue the
                       relationship)
  freezeOrSeizureRef : string (URI of the freeze or
                       seizure-order acknowledgement;
                       absent unless an order is in
                       force)
```

## §11 Conformance

Implementations claiming PHASE-1 conformance emit each
of the records defined above for every operating
programme, maintain CDD records for the duration of
the customer relationship plus the operating
jurisdiction's records-retention horizon (typically
five years per FATF Recommendation 11 / equivalent
national rule), and preserve STR / SAR filings per
the FIU's required retention.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-anti-money-laundering
- **Last Updated:** 2026-04-28
