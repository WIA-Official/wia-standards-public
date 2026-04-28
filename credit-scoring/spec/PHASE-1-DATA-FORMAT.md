# WIA-credit-scoring PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-credit-scoring
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format layer
for WIA-credit-scoring. The standard covers persistent
record shapes for the lifecycle of a consumer-credit
scoring programme — the consumer's identity and
identifier bindings; the credit-bureau-supplied
tradeline records that feed the scoring model; the
scoring model's input feature vector; the model's
score, score band, reason codes, and adverse-action
narrative; the lender's creditworthiness-assessment
outcome record; the consumer-facing disclosures
required by the operating jurisdiction; and the
disclosure-and-redress correspondence record. Records
are consumed by the lender's underwriting and
servicing functions, by the consumer through the right-
to-explanation surface, by external auditors and
model-validation reviewers, by the supervisory
authority for the operating jurisdiction's fair-
lending and consumer-credit examinations, and — where
the lender uses an automated decision under GDPR
Article 22 — by the data subject's review channel.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time representation)
- ISO/IEC 11578 (UUID) and IETF RFC 4122 (UUID URN)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 42001:2023 (AI management system)
- ISO/IEC 22989:2022 (AI concepts and terminology)
- ISO/IEC 23053:2022 (framework for AI systems using
  ML)
- ISO/IEC 24029-2:2023 (AI robustness assessment
  via formal methods)
- ISO/IEC 27701:2019 (PIMS)
- IETF RFC 8259 (JSON), RFC 9457 (Problem Details)
- US ECOA (Equal Credit Opportunity Act, 15 USC 1691)
  and Regulation B (12 CFR Part 1002)
- US FCRA (Fair Credit Reporting Act, 15 USC 1681)
  and Regulation V (12 CFR Part 1022)
- US FHA (Fair Housing Act, 42 USC 3601)
- US Truth-in-Lending Act and Regulation Z (12 CFR
  Part 1026)
- US CFPB Examination Manual sections on automated
  underwriting and ECOA / FCRA examinations
- EU CCD recast (Directive (EU) 2023/2225) Articles
  6 (pre-contractual information), 9 (creditworthiness
  database), 18 (creditworthiness assessment), 26
  (compliant complaint-handling)
- EU Mortgage Credit Directive (Directive 2014/17/EU)
  Articles 18 to 20 (creditworthiness assessment)
- EU AI Act 2024 (Regulation (EU) 2024/1689) Annex
  III §5(b) "AI systems intended to be used to
  evaluate the creditworthiness of natural persons or
  establish their credit score" — high-risk
  classification
- EU GDPR (Regulation (EU) 2016/679) Articles 5, 6,
  9, 12 to 22, 24, 25, 32, 35, 47
- KR Credit Information Use and Protection Act
  (신용정보의 이용 및 보호에 관한 법률), the operating
  jurisdiction's primary statute
- KR Banking Act, KR 여신전문금융업법, KR 금융소비자
  보호법, the supplementary statutes
- Basel Committee on Banking Supervision Principles
  for the Sound Management of Operational Risk and
  the BCBS sound-practices guidance on model risk
  governance
- NIST AI RMF 1.0 (the risk-management framework
  applied to AI systems used in credit decisions)

---

## §1 Scope

This PHASE defines persistent shapes for the artefacts
a credit-scoring operator (the lender, a credit-bureau,
or a third-party scoring provider) maintains:

- The consumer-identity and identifier-binding record.
- The tradeline / credit-history record.
- The model-input feature record.
- The model-score and reason-code record.
- The creditworthiness-assessment outcome record.
- The adverse-action notice record.
- The disclosure and redress correspondence record.
- The model-governance record (training data set,
  validation, monitoring).

GDPR Article 22 (automated individual decision making)
applies where the credit decision is fully automated;
the additional safeguards required by Article 22(3)
are encoded in PHASE-3 §6.

## §2 Programme Identifier

```
programmeId          : string (uuidv7)
operatorName         : string (legal name of the
                       lender, credit-bureau, or
                       scoring provider)
operatorRole         : enum ("lender-bank" | "lender-
                       non-bank" | "credit-bureau" |
                       "scoring-provider" | "fintech-
                       intermediary" | "user-defined")
operatorJurisdiction : array of string (ISO 3166-1
                       country codes of the operator's
                       establishments)
governingFrameworks  : array of enum ("US-ECOA-REG-B"
                       | "US-FCRA-REG-V" | "US-FHA" |
                       "US-TILA-REG-Z" | "US-CFPB-
                       EXAMINATION" | "EU-CCD-2023-
                       2225" | "EU-MORTGAGE-CREDIT-
                       DIRECTIVE" | "EU-AI-ACT-ANNEX-
                       III-CREDIT" | "EU-GDPR-ART-22"
                       | "KR-CREDIT-INFO-ACT" |
                       "KR-BANKING-ACT" | "KR-여신전문
                       금융업법" | "KR-금융소비자
                       보호법" | "BASEL-MODEL-RISK" |
                       "NIST-AI-RMF" | "ISO-IEC-
                       42001" | "user-defined")
productKind          : enum ("consumer-credit-card" |
                       "personal-loan" | "auto-loan" |
                       "mortgage" | "small-business-
                       loan" | "buy-now-pay-later" |
                       "trade-credit" | "user-defined")
modelGovernanceTier  : enum ("simple-rule-set" |
                       "statistical-scoring-model" |
                       "machine-learning-model" |
                       "deep-learning-model")
programmeStatus      : enum ("design" | "validated" |
                       "operating" | "monitored" |
                       "wind-down" | "archived")
```

The operator's `governingFrameworks` and
`modelGovernanceTier` together determine the
applicable model-risk discipline (PHASE-3 §3) and the
applicable consumer-disclosure discipline (PHASE-3
§5).

## §3 Consumer Identity Record

```
consumerRecord:
  consumerId         : string (uuidv7; the operator's
                       internal customer identifier)
  jurisdictionalIdentifier : object (KR resident
                       registration number, US Social
                       Security Number, EU Member State
                       national identifier; encrypted
                       at rest, accessed only by the
                       operator's authorised role)
  givenName          : array of string
  familyName         : string
  birthDate          : string (ISO 8601 date)
  residentialAddress : object (per-jurisdiction
                       structured address)
  consentReferences  : array of string (PHASE-1 §10
                       consent-record references)
```

## §4 Tradeline Record

The tradeline record encodes a single credit account's
history as supplied by the credit-bureau or assembled
from the lender's own portfolio:

```
tradelineRecord:
  tradelineId        : string (uuidv7)
  consumerRef        : string (PHASE-1 §3)
  accountKind        : enum ("revolving-card" |
                       "installment-loan" |
                       "mortgage" | "auto-loan" |
                       "student-loan" | "open-account"
                       | "collection-account" |
                       "public-record" | "user-
                       defined")
  furnisherRef       : string (the data furnisher
                       under FCRA 15 USC 1681s-2; the
                       tradeline's source of record)
  openedAt           : string (ISO 8601 date)
  closedAt           : string (ISO 8601 date; absent
                       for active accounts)
  highBalance        : object (currency-quantified)
  currentBalance     : object (currency-quantified)
  creditLimit        : object (currency-quantified;
                       absent for installment loans)
  paymentHistory     : array of object (per-month
                       status — current, 30 days
                       past due, 60 days past due,
                       90+ days past due, charged-
                       off, settled)
  derogatoryFlags    : array of enum ("bankruptcy" |
                       "foreclosure" | "tax-lien" |
                       "judgment" | "collection" |
                       "charge-off" | "user-defined")
  disputeStatus      : enum ("none" | "consumer-
                       disputed" | "furnisher-
                       investigating" | "resolved-
                       updated" | "resolved-no-change")
                       (FCRA 15 USC 1681i dispute
                       status)
```

## §5 Model-Input Feature Record

```
featureVector:
  vectorId           : string (uuidv7)
  consumerRef        : string
  featureFamily      : enum ("payment-history" |
                       "amounts-owed" | "length-of-
                       history" | "new-credit" |
                       "credit-mix" | "alternative-
                       data" | "user-defined")
  featureName        : string (the operator's feature
                       catalogue name; the catalogue
                       cross-references the feature to
                       its source — credit-bureau
                       furnished, lender-internal, or
                       alternative-data-provider)
  featureValue       : object (numeric, categorical,
                       or boolean)
  effectiveAt        : string (ISO 8601)
  modelRef           : string (PHASE-1 §6 model
                       reference)
```

The operator's feature catalogue is reviewed on the
fair-lending discipline (PHASE-3 §4) so that disparate-
impact concerns are surfaced before a feature enters
production.

## §6 Model and Model-Version Record

```
modelRecord:
  modelId            : string (uuidv7)
  programmeRef       : string
  modelKind          : enum ("logistic-regression" |
                       "gradient-boosted-tree" |
                       "random-forest" | "neural-
                       network" | "ensemble" |
                       "rule-based" | "user-defined")
  trainingDataSetRef : string (URI of the operator's
                       training data set manifest;
                       PHASE-3 §3 training-data
                       discipline applies)
  validationReportRef : string (URI of the validation
                       report — Basel-aligned model-
                       risk-management documentation)
  performanceMetrics : object (the model's KS, AUC,
                       PSI, demographic-parity ratio,
                       and false-negative-rate
                       differential at the operator's
                       declared decision threshold)
  approvedAt         : string (ISO 8601)
  approvingCommittee : string (the operator's model-
                       risk-committee identifier)
  retiredAt          : string (ISO 8601; absent until
                       retired)
```

## §7 Score and Reason-Code Record

```
scoreRecord:
  scoreId            : string (uuidv7)
  consumerRef        : string
  modelRef           : string
  computedAt         : string (ISO 8601)
  scoreValue         : number (model output, on the
                       operator's score scale)
  scoreBand          : enum ("excellent" | "good" |
                       "fair" | "subprime" | "deep-
                       subprime" | "user-defined")
  reasonCodes        : array of object (top-N feature
                       contributions to the score;
                       FCRA 15 USC 1681m(a) requires
                       the principal reasons for an
                       adverse action)
  inputVectorRef     : string (PHASE-1 §5)
```

## §8 Creditworthiness Assessment Record

```
creditworthinessAssessment:
  assessmentId       : string (uuidv7)
  consumerRef        : string
  productRef         : string (the requested credit
                       product reference)
  scoreRef           : string
  assessmentBasis    : enum ("us-reg-b-ecoa-effects-
                       test" | "us-tila-reg-z-
                       ability-to-repay" | "eu-ccd-
                       2023-2225-art-18" | "eu-mcd-
                       art-18" | "kr-금융소비자보호법-
                       적합성-확인" | "user-defined")
  decisionKind       : enum ("approve" | "approve-
                       with-conditions" | "decline" |
                       "counter-offer" | "manual-
                       review")
  decidedAt          : string (ISO 8601)
  rationaleRef       : string (URI of the rationale
                       narrative; required for adverse
                       actions and for GDPR Article
                       22(3) right-to-explanation
                       responses)
  humanReviewerRef   : string (the practitioner who
                       performed the manual review;
                       absent for fully automated
                       decisions)
```

## §9 Adverse-Action Notice Record

The adverse-action notice is the consumer-facing
disclosure required when the lender denies credit or
offers materially less favourable terms:

```
adverseActionNotice:
  noticeId           : string (uuidv7)
  consumerRef        : string
  assessmentRef      : string
  noticeKind         : enum ("us-ecoa-reg-b-adverse-
                       action" | "us-fcra-1681m-
                       adverse-action" | "eu-ccd-
                       2023-2225-art-18-rejection-
                       reasoning" | "kr-credit-info-
                       act-rejection-reasoning" |
                       "user-defined")
  principalReasons   : array of string (the principal
                       reasons drawn from the score's
                       reason codes; ECOA Reg B 12
                       CFR 1002.9 specifies the
                       discipline)
  bureauReference    : object (the credit-bureau the
                       lender relied on, the bureau's
                       contact information, and the
                       consumer's right to obtain a
                       free copy of the report under
                       FCRA 15 USC 1681j(a))
  redressChannelRef  : string (URI of the redress
                       channel)
  deliveredAt        : string (ISO 8601)
```

## §10 Consent and Disclosure Correspondence Record

```
consentRecord:
  consentId          : string (uuidv7)
  consumerRef        : string
  consentBasis       : enum ("us-fcra-1681b-permissible-
                       purpose" | "us-glba-7-ndnpi" |
                       "eu-gdpr-art-6-1-b-contract" |
                       "eu-gdpr-art-6-1-f-legitimate-
                       interest" | "kr-credit-info-
                       act-art-32-credit-info-use" |
                       "kr-pipa-art-15-collection-
                       consent" | "user-defined")
  scope              : array of string (the categories
                       of credit information the
                       consumer authorises)
  capturedAt         : string (ISO 8601)
  withdrawalAt       : string (ISO 8601; absent until
                       withdrawn)
  withdrawalChannel  : string (URI of the channel —
                       under GDPR Article 7(3) the
                       channel must be as easy as the
                       capture channel)
```

## §11 Conformance

Implementations claiming PHASE-1 conformance maintain
the records defined above for every consumer assessed,
preserve the model-governance records on the operating
retention horizon, and emit the adverse-action notice
within the operating jurisdiction's mandated time-
window (US ECOA Reg B 12 CFR 1002.9 within 30 days;
EU CCD 2023/2225 Article 18 without undue delay; KR
Credit Information Use and Protection Act within the
statutory window).

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-credit-scoring
- **Last Updated:** 2026-04-28
