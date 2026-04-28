# WIA-cross-border-payment PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-cross-border-payment
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format layer
for WIA-cross-border-payment. The standard covers
persistent record shapes for the lifecycle of a cross-
border payment instruction — the originating customer
and the originating institution, the beneficiary and
the beneficiary institution, the routing and
correspondent-banking chain that conveys value between
the two, the FATF Recommendation 16 travel-rule
information that travels with the instruction, the
sanctions and adverse-media screening outcomes, the
Cross-Border CDD Questionnaire (CBDDQ) the institutions
exchange, the settlement event that closes the
instruction on the books of the relevant payment
system or correspondent network, and the post-
settlement reconciliation and reporting record.
Records are consumed by both the originating and the
beneficiary institutions, by the correspondent
network's operator, by the operating jurisdictions'
financial-intelligence units (FIUs) under the suspicious-
transaction reporting basis, by external auditors, and
by the supervisory financial-services authority for
the operating jurisdiction's payment-services and
anti-money-laundering examinations.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time representation)
- ISO/IEC 11578 (UUID) and IETF RFC 4122 (UUID URN)
- ISO/IEC 27001:2022 (information security management)
- ISO 4217 (currency codes)
- ISO 3166-1 (country codes)
- ISO 9362 (BIC — Business Identifier Code)
- ISO 13616 (IBAN — International Bank Account
  Number)
- ISO 17442 (LEI — Legal Entity Identifier)
- ISO 20022 (the Universal financial industry message
  scheme; the operating wire format for cross-border
  payments — pacs.008 customer credit transfer,
  pacs.009 financial-institution credit transfer,
  pacs.002 status report, camt.054 bank-to-customer
  debit-credit notification, camt.029 resolution of
  investigation)
- IETF RFC 8259 (JSON), RFC 9457 (Problem Details)
- SWIFT MT 103 / 202 / 202 COV (the legacy customer
  credit transfer, financial-institution credit
  transfer, and cover payment messages, retained where
  the operating chain has not migrated to ISO 20022
  by the SWIFT MT-to-ISO-20022 industry coexistence
  cutover)
- SWIFT FIN, SWIFTNet, SWIFT GPI tracking and CCT
  Inst confirmation discipline
- CPMI Principles for Financial Market
  Infrastructures (PFMI)
- CPMI cross-border payments roadmap (the Bank for
  International Settlements' published programme of
  work on cross-border payment cost, speed, access,
  and transparency)
- FATF Recommendations (especially Recommendation
  10 customer due diligence; Recommendation 11
  record-keeping; Recommendation 13 correspondent
  banking; Recommendation 16 wire transfers;
  Recommendation 20 suspicious-transaction reporting;
  Recommendation 21 tipping-off; Recommendation 22
  designated non-financial businesses; Recommendation
  25 transparency of legal arrangements)
- Wolfsberg Cross-Border CDD Questionnaire (CBDDQ
  v1.4) and the Wolfsberg Anti-Money-Laundering
  Principles for Correspondent Banking
- US BSA + 31 CFR Chapter X (FinCEN), OFAC SDN
  programmes
- EU AMLR (Regulation (EU) 2024/1624) Article 38
  cross-border correspondent-relationship CDD;
  AMLA Regulation (EU) 2024/1620
- EU SEPA Regulation (Regulation (EU) No 260/2012)
  for euro-denominated EU intra-area transfers
- EU Cross-Border Payments Regulation (Regulation
  (EU) 2021/1230)
- EU Wire Transfer Regulation (Regulation (EU) 2023/
  1113) implementing FATF Recommendation 16 in the
  EU
- KR 특정금융정보법 (the operating jurisdiction's AML
  primary statute) and the KR Korea Financial
  Intelligence Unit (KoFIU) reporting discipline
- TARGET2 / T2-T2S consolidated platform, CHIPS,
  Fedwire — the major large-value-payment systems
  the operator may interact with

---

## §1 Scope

This PHASE defines persistent shapes for the artefacts
a cross-border payment operator (an originating bank,
a beneficiary bank, a correspondent bank, a payment
service provider, a money-services business, or a
multilateral cross-border payment system) maintains:

- The instruction record — the customer-facing payment
  instruction the originator submits.
- The wire message record — the ISO 20022 or SWIFT MT
  message that carries the instruction along the
  correspondent chain.
- The party-and-identifier records — the originating
  customer, the beneficiary, and the chain of
  intermediary institutions, each with their FATF
  Recommendation 16 identification fields.
- The screening record — the sanctions and adverse-
  media screening outcomes for each party.
- The CBDDQ record — the cross-border CDD
  questionnaire exchanged between correspondents.
- The settlement record — the on-book settlement
  event in the relevant payment system.
- The status, exception, and investigation record —
  the lifecycle events between submission and
  finalisation.
- The post-settlement record — the reconciliation,
  notification, and reporting record.

## §2 Programme Identifier

```
programmeId          : string (uuidv7)
operatorName         : string (legal name of the
                       operator)
operatorRole         : enum ("originating-bank" |
                       "beneficiary-bank" |
                       "correspondent-bank" |
                       "intermediary-bank" |
                       "payment-service-provider" |
                       "money-services-business" |
                       "multilateral-payment-system" |
                       "user-defined")
operatorBic          : string (ISO 9362; absent for
                       non-SWIFT-connected operators)
operatorLei          : string (ISO 17442)
operatorJurisdiction : array of string (ISO 3166-1)
governingFrameworks  : array of enum ("ISO-20022" |
                       "SWIFT-MT-LEGACY" |
                       "SWIFT-GPI" | "SWIFT-CCT-INST"
                       | "CPMI-PFMI" | "FATF-REC-16"
                       | "WOLFSBERG-CBDDQ-1-4" |
                       "US-BSA-31-CFR-X" |
                       "OFAC-SDN" | "EU-AMLR-2024-
                       1624" | "EU-SEPA-260-2012" |
                       "EU-CROSS-BORDER-PAYMENTS-
                       2021-1230" | "EU-WIRE-
                       TRANSFER-2023-1113" |
                       "KR-특정금융정보법" |
                       "TARGET2" | "CHIPS" |
                       "FEDWIRE" | "user-defined")
programmeStatus      : enum ("design" | "operating" |
                       "limited-rollout" | "wind-down"
                       | "archived")
```

## §3 Instruction Record

```
instructionRecord:
  instructionId      : string (uuidv7; the operator's
                       internal reference)
  endToEndId         : string (the end-to-end
                       reference the originator
                       supplies — ISO 20022 EndToEndId)
  txId               : string (the operator's
                       transaction identifier — ISO
                       20022 TxId)
  uetr               : string (Unique End-to-end
                       Transaction Reference, UUIDv4
                       per SWIFT GPI tracking)
  submittedAt        : string (ISO 8601)
  requestedSettlementDate : string (ISO 8601 date)
  instructedAmount   : object (ISO 4217 currency +
                       amount)
  settlementAmount   : object (ISO 4217 currency +
                       amount; differs from instructed
                       amount where currency conversion
                       is in scope)
  serviceLevel       : enum ("standard" | "priority" |
                       "instant" | "user-defined")
  chargeBearer       : enum ("DEBT" | "CRED" | "SHAR"
                       | "SLEV")
  purposeCode        : string (ISO 20022 ExternalPurpose
                       code, e.g. "CORT" trade
                       settlement, "INTC" intra-company
                       transfer, "GOVT" government
                       payment)
```

## §4 Party Records

```
partyRecord:
  partyId            : string (uuidv7)
  partyRole          : enum ("debtor" | "debtor-agent"
                       | "creditor" | "creditor-agent"
                       | "intermediary-agent-1" |
                       "intermediary-agent-2" |
                       "instructing-agent" |
                       "instructed-agent" |
                       "ultimate-debtor" |
                       "ultimate-creditor")
  partyKind          : enum ("natural-person" |
                       "legal-entity" |
                       "unincorporated-association")
  partyName          : string
  partyAddress       : object (structured address —
                       FATF Recommendation 16 requires
                       the originator's address; the
                       EU Wire Transfer Regulation
                       2023/1113 Article 4 mandates
                       the same on the EU side)
  identification:
    iban             : string (ISO 13616; absent for
                       non-IBAN jurisdictions)
    accountIdentifier : string (the jurisdictional
                       account identifier where IBAN
                       does not apply)
    bic              : string (ISO 9362; for parties
                       that are themselves financial
                       institutions)
    lei              : string (ISO 17442; preferred
                       for legal-entity parties)
    nationalId       : object (the jurisdictional
                       identity-document identifier
                       where the institution captured
                       it during onboarding)
    dateOfBirth      : string (ISO 8601 date; FATF
                       Recommendation 16 mandates the
                       originator's date-of-birth as
                       a fallback when no account
                       number is involved)
```

## §5 Wire Message Record

The wire message record persists the ISO 20022 or
SWIFT MT message that conveys the instruction along
the correspondent chain:

```
wireMessageRecord:
  messageId          : string (uuidv7; the operator's
                       internal reference)
  instructionRef     : string (PHASE-1 §3)
  messageKind        : enum ("iso20022-pacs-008" |
                       "iso20022-pacs-009" |
                       "iso20022-pacs-002" |
                       "iso20022-camt-054" |
                       "iso20022-camt-029" |
                       "swift-mt-103" |
                       "swift-mt-202" |
                       "swift-mt-202-cov" |
                       "user-defined")
  direction          : enum ("inbound" | "outbound")
  counterpartyBic    : string (ISO 9362)
  sentAt             : string (ISO 8601; absent for
                       inbound messages)
  receivedAt         : string (ISO 8601; absent for
                       outbound messages)
  cryptographicDigest : string (the SHA-256 of the
                       canonical wire payload, used
                       for evidence preservation)
  payloadRef         : string (URI of the canonical
                       wire payload — encrypted at
                       rest)
```

## §6 Travel-Rule Information Record

The travel-rule information record encodes the FATF
Recommendation 16 mandatory information that travels
with the instruction throughout the correspondent
chain. The EU Wire Transfer Regulation 2023/1113
operationalises the travel rule on the EU side:

```
travelRuleRecord:
  recordId           : string (uuidv7)
  instructionRef     : string
  originatorInfo:
    name             : string
    address          : object
    accountIdentifier : string
    nationalId       : object (where applicable)
    dateOfBirth      : string (ISO 8601 date — FATF
                       fallback)
    placeOfBirth     : string (FATF fallback)
  beneficiaryInfo:
    name             : string
    accountIdentifier : string
    address          : object (for EU inbound
                       transfers above EUR 1 000 the
                       beneficiary's address is also
                       required per Reg 2023/1113)
  travelRuleStatus   : enum ("complete" | "missing-
                       fields" | "remediated" |
                       "rejected-incomplete")
```

## §7 Sanctions-Screening and Adverse-Media Record

```
screeningRecord:
  screeningId        : string (uuidv7)
  partyRef           : string (PHASE-1 §4)
  screeningEngineRef : string (the operator's
                       sanctions / adverse-media
                       screening engine reference)
  screenedAt         : string (ISO 8601)
  watchlistsApplied  : array of enum ("ofac-sdn" |
                       "ofac-non-sdn-pl" | "un-
                       consolidated" | "eu-fsf" |
                       "uk-hmt" | "kr-제재대상" |
                       "pep-screening" | "adverse-
                       media" | "user-defined")
  outcomeKind        : enum ("clear" | "potential-
                       match-pending-review" |
                       "confirmed-match" |
                       "false-positive-cleared")
  reviewerRef        : string (the human reviewer for
                       potential matches; absent for
                       clear outcomes)
  rationaleRef       : string (URI of the reviewer's
                       rationale narrative)
```

## §8 CBDDQ Record

```
cbddqRecord:
  cbddqId            : string (uuidv7)
  correspondentRef   : string (the correspondent's
                       BIC and LEI)
  versionApplied     : enum ("wolfsberg-cbddq-1-4" |
                       "user-defined")
  receivedAt         : string (ISO 8601)
  reviewedBy         : string (the operator's
                       compliance officer)
  reviewOutcome      : enum ("approved" | "approved-
                       with-conditions" |
                       "approved-enhanced-due-
                       diligence" | "declined")
  effectiveFrom      : string (ISO 8601)
  effectiveUntil     : string (ISO 8601; absent until
                       superseded — Wolfsberg
                       guidance suggests a refresh
                       cadence aligned with the
                       correspondent's risk rating)
```

## §9 Settlement Record

```
settlementRecord:
  settlementId       : string (uuidv7)
  instructionRef     : string
  paymentSystem      : enum ("target2" | "chips" |
                       "fedwire" | "ach-correspondent"
                       | "swift-correspondent" |
                       "card-network" | "rtgs-
                       jurisdictional" | "instant-
                       payment-system" | "user-
                       defined")
  settledAt          : string (ISO 8601)
  finalityKind       : enum ("settled-final" |
                       "provisional-pending-
                       reconciliation" | "rejected")
  systemReference    : string (the payment system's
                       internal reference for the
                       settlement event)
```

## §10 Investigation Record (camt.029)

```
investigationRecord:
  investigationId    : string (uuidv7)
  instructionRef     : string
  raisedBy           : enum ("originator" |
                       "beneficiary" | "correspondent"
                       | "operator-internal")
  raisedAt           : string (ISO 8601)
  investigationKind  : enum ("non-receipt" |
                       "amount-mismatch" |
                       "duplicate-payment" |
                       "incorrect-beneficiary" |
                       "fraud-suspected" |
                       "user-defined")
  resolutionKind     : enum ("payment-traced-
                       delivered" | "payment-
                       returned" | "payment-amended"
                       | "fraud-confirmed-reported"
                       | "user-defined")
  resolvedAt         : string (ISO 8601; absent until
                       resolved)
```

## §11 Suspicious-Activity Reporting Record

```
sarRecord:
  sarId              : string (uuidv7)
  instructionRef     : string
  jurisdictionalReportingAuthority : enum ("us-fincen
                       -sar" | "kr-kofiu-str" |
                       "uk-nca-ukfiu-sar" |
                       "eu-amla-cross-border" |
                       "user-defined")
  filedAt            : string (ISO 8601)
  filingNarrativeRef : string (URI of the SAR / STR
                       narrative — preserved under the
                       FATF Recommendation 21 tipping-
                       off discipline)
  fiuAcknowledgementRef : string (URI of the FIU
                       acknowledgement; absent until
                       acknowledged)
```

## §12 Conformance

Implementations claiming PHASE-1 conformance maintain
each of the records defined above for every cross-
border payment instruction the operator participates
in, preserve the wire message record under the FATF
Recommendation 11 record-keeping discipline (five years
from the date of the transaction), and emit suspicious-
transaction reports to the operating jurisdiction's
FIU within the statutory window without prior
notification to the customer per FATF Recommendation
21.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-cross-border-payment
- **Last Updated:** 2026-04-28
