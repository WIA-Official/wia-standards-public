# WIA-financial-inclusion PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-financial-inclusion
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format layer for
WIA-financial-inclusion. The standard covers persistent
record shapes for financial-inclusion programmes
operated by financial institutions, payment-service
providers, microfinance institutions, mobile-money
operators, and civil-society organisations — programmes
that bring under-served and unbanked populations into
formal financial systems through basic transaction
accounts (per the World Bank's UFA-2020 framework
operationalised by national financial-inclusion
strategies), simplified customer due diligence under
the FATF risk-based approach (Recommendation 10
simplified CDD for proven low-risk relationships),
consumer-protection disclosure under the operating
jurisdiction's regime, fee-transparency, dispute
resolution, and fair-lending discipline. The format is
consumed by the operating jurisdiction's financial-
inclusion strategy coordinator, the operating
jurisdiction's central bank or financial-inclusion
authority, the operating jurisdiction's consumer-
protection regulator, the operating jurisdiction's
AML/CFT supervisor (where simplified CDD operates
under the FATF Recommendation 10 risk-based-approach
framework), civil-society partner organisations, and
the programme's external evaluators.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time representation)
- ISO/IEC 11578 (UUID)
- ISO/IEC 27001:2022 (information security management)
- ISO 20022 (financial-services messaging)
- ISO 17442 (Legal Entity Identifier — LEI)
- ISO 4217 (currency codes)
- IETF RFC 4122 (UUID URN)
- IETF RFC 8259 (JSON)
- IETF RFC 9457 (Problem Details)
- World Bank Universal Financial Access 2020 (UFA-
  2020) framework — cited as the international
  financial-inclusion baseline that informs many
  national financial-inclusion strategies
- World Bank Findex Database (cited as the canonical
  cross-country financial-inclusion measurement
  reference)
- G20 Financial Inclusion Action Plan + GPFI High-
  Level Principles for Digital Financial Inclusion
- AFI (Alliance for Financial Inclusion) Maya
  Declaration (cited where the operating
  jurisdiction is an AFI member)
- FATF 40 Recommendations — particularly
  Recommendation 1 (risk-based approach),
  Recommendation 10 (simplified CDD for proven low-
  risk customer relationships), and Recommendation
  16 (Travel Rule for cross-border wire transfers)
- US BSA 31 CFR Part 1020 — simplified due diligence
  for low-risk customer relationships at depository
  institutions where the operating jurisdiction is
  the US
- EU AML Regulation (EU) 2024/1624 — simplified due
  diligence under Article 33 for low-risk
  relationships
- KR Specific Financial Information Act ("특정금융정보법")
  — simplified due diligence under the operating
  ministry's risk-based-approach guidance
- US Community Reinvestment Act (CRA) and 12 CFR
  Part 25 — cited for fair-lending obligations in
  US-jurisdiction
- US Equal Credit Opportunity Act (ECOA) and
  Regulation B (12 CFR Part 1002) — cited for fair-
  lending non-discrimination
- US Truth in Lending Act (TILA) and Regulation Z
  (12 CFR Part 1026) — cited for consumer-credit
  disclosure
- US Truth in Savings Act (TISA) and Regulation DD
  (12 CFR Part 1030) — cited for deposit-account
  disclosure
- EU Payment Accounts Directive (Directive 2014/92/
  EU) — cited for the EU's right-of-access-to-a-
  payment-account-with-basic-features regime
- EU Consumer Credit Directive (Directive (EU)
  2023/2225) — cited for EU consumer-credit
  disclosure
- KR Specialized Credit Finance Business Act and
  Banking Act — cited for KR consumer-credit
  disclosure
- W3C Verifiable Credentials Data Model 2.0 (used
  in PHASE-4 for optional re-issuance)

---

## §1 Scope

This PHASE defines persistent shapes for the artefacts
a financial-inclusion programme manages.
Implementations covered include:

- Bank-led basic-account programmes (the operating
  jurisdiction's right-to-a-basic-payment-account
  regime — EU Payment Accounts Directive's right of
  access, US BSA-fingerprint basic checking
  programmes, KR basic-savings-account regime).
- Microfinance-institution programmes (the operating
  jurisdiction's microfinance-institution licensing
  regime, including group-lending, individual-lending,
  and savings-and-credit-cooperative models).
- Mobile-money-operator programmes (mobile-network-
  operator-issued mobile-money under the operating
  jurisdiction's mobile-money licensing regime).
- Payment-service-provider programmes (non-bank
  payment-service providers under the operating
  jurisdiction's PSP licensing regime — EU Payment
  Services Directive 2 (EU 2015/2366), US money-
  transmitter licensing per state, KR Electronic
  Financial Transactions Act).
- Civil-society-led financial-literacy programmes
  that complement formal financial-inclusion through
  community-based education.

Anti-money-laundering full CDD records remain governed
by WIA-anti-money-laundering; this PHASE addresses
financial-inclusion-specific record shapes (simplified
CDD evidence, basic-account access records, fair-
lending records, consumer-protection disclosure
records).

## §2 Programme Identifier

```
programmeId          : string (uuidv7)
programmeOperator    : string (legal name of the
                       programme operator)
operatorClass        : enum ("bank-deposit-taking" |
                       "non-bank-psp" |
                       "microfinance-institution" |
                       "mobile-money-operator" |
                       "savings-and-credit-cooperative"
                       | "civil-society-financial-
                       literacy" | "user-defined")
operatingJurisdiction : array of string (ISO 3166-1
                       country codes of the
                       programme's operations)
governingFrameworks  : array of enum ("World-Bank-
                       UFA-2020" | "G20-GPFI-Digital-
                       Financial-Inclusion-
                       Principles" | "AFI-Maya-
                       Declaration" |
                       "operating-jurisdiction-
                       financial-inclusion-strategy"
                       | "FATF-Recs-with-Simplified-
                       CDD" | "EU-Payment-Accounts-
                       Directive-2014-92" |
                       "EU-Payment-Services-Directive-2"
                       | "EU-Consumer-Credit-Directive-
                       2023-2225" |
                       "US-BSA-Simplified-CDD" |
                       "US-CRA-12-CFR-25" |
                       "US-ECOA-Regulation-B" |
                       "US-TILA-Regulation-Z" |
                       "US-TISA-Regulation-DD" |
                       "KR-Specific-Financial-
                       Information-Act-Simplified-CDD"
                       | "user-defined")
nationalStrategyRef  : string (URI of the operating
                       jurisdiction's National
                       Financial Inclusion Strategy
                       reference; absent if the
                       jurisdiction does not maintain
                       a published NFIS)
amlSupervisor        : string (operating jurisdiction's
                       AML/CFT supervisor identifier;
                       cross-walked with WIA-anti-
                       money-laundering)
consumerProtectionAuthority : string (operating
                       jurisdiction's consumer-
                       protection authority — US CFPB
                       for federal-level US, EU
                       Member State consumer
                       authorities, KR Financial
                       Services Commission consumer-
                       protection arm, etc.)
programmeStatus      : enum ("design" | "pilot-cohort"
                       | "operating" | "wind-down" |
                       "archived")
```

## §3 Basic-Account-Access Record

```
basicAccountAccess:
  accessId           : string (uuidv7)
  programmeId        : string (uuidv7)
  customerRef        : string (CDD record reference;
                       cross-walked with WIA-anti-
                       money-laundering)
  accountKind        : enum ("basic-payment-account-
                       eu-pad" | "basic-checking-us-
                       bsa" | "basic-savings-kr" |
                       "mobile-money-tier-1" |
                       "mobile-money-tier-2" |
                       "mobile-money-tier-3" |
                       "microfinance-savings" |
                       "user-defined")
  openedAt           : string (ISO 8601)
  simplifiedCddBasis : enum ("low-balance-and-low-
                       transaction-volume" | "
                       government-benefit-recipient" |
                       "tier-based-mobile-money" |
                       "no-simplified-cdd-applied" |
                       "user-defined")
  identificationDocs : array of object (the
                       relaxed identification documents
                       accepted under simplified CDD;
                       e.g. national-ID-only without
                       proof-of-address for tier-1
                       mobile money in jurisdictions
                       that permit this relaxation;
                       per the operating jurisdiction's
                       financial-inclusion-aligned
                       documentary requirements)
  thresholdLimits    : object (transaction value and
                       balance limits the simplified-
                       CDD relaxation imposes; e.g.
                       per-month aggregate transaction
                       cap, per-transaction cap, per-
                       balance cap; these limits are
                       set by the operating
                       jurisdiction's regulator)
  upgradePathway     : string (URI of the per-customer
                       upgrade pathway to standard
                       CDD when the customer exceeds
                       the simplified-CDD thresholds)
```

## §4 Consumer-Protection Disclosure Record

```
consumerDisclosure:
  disclosureId       : string (uuidv7)
  programmeId        : string (uuidv7)
  customerRef        : string (CDD record reference)
  disclosedAt        : string (ISO 8601)
  disclosureKind     : enum ("us-tila-pre-
                       application" | "us-tila-pre-
                       consummation" | "us-tisa-
                       deposit-account" | "eu-
                       consumer-credit-pre-
                       contractual" | "eu-payment-
                       accounts-directive-fee-
                       information" | "kr-banking-
                       pre-contractual" | "user-
                       defined")
  disclosureLanguage : string (BCP 47 language tag —
                       the operating jurisdiction's
                       language requirements may
                       require disclosure in the
                       customer's preferred language
                       where the customer's primary
                       language is one of the
                       jurisdiction's recognised
                       languages)
  disclosureArtefactRef : string (URI of the
                       disclosure document as
                       presented to the customer)
  acknowledgementCapturedAt : string (ISO 8601;
                       absent if the disclosure
                       regime requires presentation
                       only without acknowledgement)
```

## §5 Fee-Transparency Record

```
feeSchedule:
  feeScheduleId      : string (uuidv7)
  programmeId        : string (uuidv7)
  publishedAt        : string (ISO 8601)
  effectiveFrom      : string (ISO 8601)
  effectiveUntil     : string (ISO 8601; absent until
                       superseded)
  feeItems           : array of object (per-item
                       fee, amount per ISO 4217
                       currency, applicability
                       conditions, waivability under
                       the operating jurisdiction's
                       basic-account regime — e.g.
                       EU PAD basic-payment-account
                       fees are capped or waived
                       per the Member State's
                       transposition)
  glossaryRef        : string (URI of the fee-glossary
                       publication; the EU PAD
                       requires a standardised fee-
                       glossary publication)
  feeInformationDocumentRef : string (URI of the EU
                       PAD Fee Information Document /
                       equivalent regulator-
                       prescribed disclosure)
```

## §6 Dispute-Resolution Record

```
disputeResolution:
  disputeId          : string (uuidv7)
  programmeId        : string (uuidv7)
  filedAt            : string (ISO 8601)
  customerRef        : string (CDD record reference)
  disputeKind        : enum ("unauthorised-
                       transaction" | "merchant-
                       dispute" | "fee-dispute" |
                       "account-closure-dispute" |
                       "credit-decision-dispute" |
                       "user-defined")
  internalResolutionDeadlineRef : string (URI of the
                       operating jurisdiction's
                       internal resolution-deadline
                       rule — EU PSD2 Article 101
                       fifteen-business-day rule for
                       payment disputes, US Reg E /
                       Reg Z dispute deadlines for
                       US-jurisdiction)
  internalResolvedAt : string (ISO 8601; absent
                       until resolved)
  internalResolutionDecision : enum ("upheld-original-
                       decision" | "reversed-and-
                       refunded" | "partial-refund" |
                       "withdrawn-by-customer" |
                       "absent-pending")
  externalEscalationRef : string (URI of the operating
                       jurisdiction's alternative
                       dispute resolution body
                       reference; absent unless the
                       customer escalates externally
                       — Financial Ombudsman Service
                       UK, FOS-AU, EU Member State
                       FIN-NET network ADR bodies,
                       CFPB Consumer Response in the
                       US for federal-level
                       consumer-finance issues)
```

## §7 Fair-Lending Decision Record

For programmes that extend credit:

```
creditDecision:
  decisionId         : string (uuidv7)
  programmeId        : string (uuidv7)
  customerRef        : string (CDD record reference)
  applicationReceivedAt : string (ISO 8601)
  decisionAt         : string (ISO 8601)
  decisionOutcome    : enum ("approved" | "approved-
                       with-conditions" |
                       "counter-offer" | "denied" |
                       "withdrawn-by-applicant" |
                       "incomplete-information")
  adverseActionNoticeRef : string (URI of the adverse-
                       action notice — the US ECOA
                       Reg B § 1002.9 adverse-action
                       notice in the US, the EU
                       Consumer Credit Directive
                       Article 9 adverse-decision
                       transparency in the EU,
                       equivalent national rules
                       elsewhere; absent unless the
                       outcome is `denied` or
                       `counter-offer`)
  fairLendingFrameworkRef : string (URI of the operating
                       jurisdiction's fair-lending
                       framework reference — US ECOA /
                       FHA / CRA, EU Consumer Credit
                       Directive Article 6 creditor's
                       general information requirement
                       and Article 18 creditworthiness
                       assessment; the framework
                       prohibits discrimination on
                       protected characteristics)
```

## §8 Programme-Outcome Aggregate Record

```
programmeOutcome:
  outcomeId          : string (uuidv7)
  programmeId        : string (uuidv7)
  reportPeriodStart  : string (ISO 8601)
  reportPeriodEnd    : string (ISO 8601)
  newAccountsOpenedCount : integer
  activeAccountsCount : integer (per the operating
                       jurisdiction's "active account"
                       definition — typically a
                       transaction within the prior
                       12 months for the World Bank
                       Findex methodology)
  per-DemographicSummary : array of object (per-
                       demographic-cohort enrolled and
                       active counts where the
                       operating jurisdiction's
                       demographic-reporting rules
                       require the breakdown — e.g.
                       US CRA Public File, EU SME-
                       focused breakdown, KR
                       inclusive-banking statistics)
  reportArtefactRef  : string (URI of the rendered
                       report; the report is
                       published to the financial-
                       inclusion strategy coordinator
                       and the operating jurisdiction's
                       central bank per the reporting
                       template)
```

## §9 Customer-Education-Activity Record

For programmes that include financial-literacy
education:

```
educationActivity:
  activityId         : string (uuidv7)
  programmeId        : string (uuidv7)
  conductedAt        : string (ISO 8601)
  activityKind       : enum ("classroom-instructor-
                       led" | "branch-walk-in-
                       counselling" | "mobile-money-
                       agent-coaching" | "self-paced-
                       online-module" | "community-
                       event" | "user-defined")
  curriculumRef      : string (URI of the curriculum;
                       cross-walked with WIA-digital-
                       citizenship financial-literacy
                       modules where the programmes
                       partner)
  participantCount   : integer
  inclusiveAccessProvisions : array of string (
                       interpreter, large-print
                       handouts, plain-language
                       summary, accessibility
                       provisions for participants
                       with declared accessibility
                       needs)
```

## §10 Conformance

Implementations claiming PHASE-1 conformance emit each
of the records defined above for every operating
programme, retain basic-account-access records for
the operating jurisdiction's records-retention horizon
(typically five years per FATF Recommendation 11),
and preserve programme-outcome aggregate records for
the operating jurisdiction's public-archive horizon.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-financial-inclusion
- **Last Updated:** 2026-04-28
