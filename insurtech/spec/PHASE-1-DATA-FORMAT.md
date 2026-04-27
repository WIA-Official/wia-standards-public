# WIA-insurtech PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-insurtech
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format layer for
WIA-insurtech. The standard covers the records that flow across
an insurance-technology programme — policy origination, premium
quotation, underwriting evidence, policy issuance, claim
notification and adjustment, fraud-control evidence, reinsurance
cession, and consumer disclosure — among insurers, brokers,
managing general agents, reinsurers, regulators, and the
consumer-facing platforms that present coverage and claims to
policyholders.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time representation)
- ISO 4217 (currency codes)
- ISO 3166-1 / 3166-2 (country and subdivision codes)
- ISO/IEC 11578 (UUID)
- ISO/IEC 17025:2017 (testing and calibration laboratories,
  cited where forensic loss-adjusters operate accredited
  laboratories)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 27701:2019 (privacy information management)
- ISO 22301:2019 (business continuity management)
- ISO 31000:2018 (risk management)
- IETF RFC 4122 (UUID URN)
- IETF RFC 8259 (JSON)
- IETF RFC 9457 (Problem Details)
- ACORD Reference Architecture (insurance industry data model;
  cited normatively for ACORD XML / JSON message envelopes)
- IFRS 17 (Insurance Contracts) and the operating jurisdiction's
  GAAP equivalent
- Solvency II Directive (EU) and equivalent solvency frameworks
  in other jurisdictions

---

## §1 Scope

This PHASE defines persistent shapes for records produced and
consumed by an insurtech operator across the policy and claim
lifecycle. Implementations covered include:

- Origination platforms (broker portals, embedded-insurance
  marketplaces, agent-tablet apps).
- Underwriting platforms (rules engines, machine-learning risk
  scoring services).
- Policy administration systems (PAS).
- Claims administration systems and adjuster workflows.
- Reinsurance ceded-bookkeeping platforms.
- Consumer-facing self-service portals.
- Regulator submission gateways for solvency, market-conduct,
  and consumer-protection reporting.

Captive and self-insurance vehicles, surety bonds outside the
insurance regulator's scope, and parametric trigger services
(weather-derivative, catastrophe-bond) are treated through
adjacent WIA standards and are out of scope here.

## §2 Programme Identifier

```
programmeId          : string (uuidv7)
programmeOperator    : string (institutional identifier of the
                         primary insurer or MGA)
programmeRegistered  : string (ISO 8601 / RFC 3339)
linesOfBusiness      : array of enum ("p&c-personal" |
                         "p&c-commercial" | "life-individual" |
                         "life-group" | "health-individual" |
                         "health-group" | "specialty" |
                         "reinsurance-assumed" | "marine" |
                         "aviation")
jurisdictionScope    : array of string (ISO 3166-1 / 3166-2)
solvencyRegimeRef    : string (regulator-issued solvency licence
                         reference)
programmeStatus      : enum ("draft" | "operating" |
                         "run-off" | "in-resolution" |
                         "archived")
```

## §3 Party Record (Policyholder, Insured, Beneficiary)

The DATA-FORMAT layer never carries direct PII; party identity is
held in the operator's CRM and exposed through opaque tokens.

```
party:
  partyId            : string (uuidv7)
  partyKind          : enum ("individual" | "household" |
                         "small-business" | "mid-market-business"
                         | "large-corporate" | "government" |
                         "trust" | "other")
  partyToken         : string (opaque token mapped in operator
                         CRM to the party's contact and
                         identity records)
  jurisdictionOfDom  : string (ISO 3166)
  kycStatus          : enum ("not-required" | "pending" |
                         "passed" | "review" | "failed")
  kycEvidenceRef     : string (URI of the KYC evidence package
                         held by the operator's compliance
                         system; never duplicated here)
  pepFlag            : boolean (politically-exposed-person flag
                         per the operator's screening policy)
  sanctionsScreenAt  : string (ISO 8601; last sanctions-list
                         screening time)
```

## §4 Risk Object Record

A risk object is the thing being insured — a vehicle, a property,
a person, a vessel, a piece of equipment, a corporate liability
exposure. Risk objects carry stable identifiers so that policy
revisions, endorsements, and claims all reference the same risk.

```
riskObject:
  riskObjectId      : string (uuidv7)
  riskClass         : enum ("vehicle" | "property-residential" |
                         "property-commercial" | "person" |
                         "vessel" | "aircraft" | "equipment" |
                         "liability-exposure" | "other")
  identifyingTokens : object (per-class identifying tokens —
                         vehicle VIN hash, property parcel ID,
                         person opaque token, hull MMSI, etc.)
  exposureUnit      : string (per-class exposure unit, e.g.
                         "vehicle-year", "tiv-usd",
                         "insured-life-year", "ton-mile")
  riskClassifiers   : object (per-class underwriting classifiers
                         — vehicle make/model, property
                         construction class, occupation code,
                         hull type)
```

## §5 Coverage Record

Coverages are the lines of insurance attached to a risk object.
Each coverage has a peril set, a limit, a deductible, and a
premium component.

```
coverage:
  coverageId        : string (uuidv7)
  policyId          : string (uuidv7; PHASE-1 §6)
  riskObjectId      : string (uuidv7)
  productCode       : string (insurer-internal product code,
                         registered in the product register
                         described in PHASE-3 §3)
  coverageKind      : enum ("comprehensive-auto" |
                         "auto-liability" | "auto-collision" |
                         "homeowners-bldg" | "homeowners-contents"
                         | "homeowners-liability" | "term-life" |
                         "permanent-life" | "individual-disability"
                         | "group-disability" | "medical-expense"
                         | "general-liability" | "professional-
                           liability" | "marine-hull" |
                         "aviation-hull" | "user-defined")
  perilSet          : array of string (peril codes from the
                         operator's peril register)
  sumInsured        : object (currency code per ISO 4217 and
                         numeric amount or limit structure for
                         per-occurrence / aggregate / sub-limits)
  deductible        : object (currency, amount, and structure)
  coveragePeriod    : object (effectiveAt / expiresAt per ISO
                         8601, or open-ended for life policies)
  premiumComponent  : object (gross premium, taxes, levies,
                         broker commission split)
```

## §6 Policy Record

The policy binds a coverage set to a policyholder. Endorsements
are append-only revisions that reference the prior policy
revision.

```
policy:
  policyId           : string (uuidv7)
  programmeId        : string (uuidv7)
  policyNumber       : string (operator-internal policy number)
  policyholderRef    : string (party UUID)
  insuredsRef        : array of string (party UUIDs)
  beneficiariesRef   : array of string (party UUIDs; for life and
                         related coverages)
  coveragesRef       : array of string (coverage UUIDs)
  effectiveAt        : string (ISO 8601)
  expiresAt          : string (ISO 8601; absent for open-ended)
  renewalDispositionRef : string (URI of the renewal-disposition
                         record; absent until renewal cycle
                         starts)
  endorsementChainRef : string (URI of the prior policy revision;
                         absent for the first issuance)
  policyStatus       : enum ("quoted" | "bound" | "in-force" |
                         "lapsed" | "cancelled" | "expired" |
                         "rescinded")
```

## §7 Underwriting Decision Record

Underwriting decisions capture the rule-engine path and any
machine-learning model outputs that influenced the decision, so
that consumer adverse-action notices and regulator audits can
reconstruct the basis.

```
underwritingDecision:
  decisionId        : string (uuidv7)
  policyId          : string (uuidv7)
  decidedAt         : string (ISO 8601)
  decisionOutcome   : enum ("accept" | "decline" |
                         "refer-to-underwriter" |
                         "accept-with-conditions" |
                         "tier-shift")
  rulesEnginePathRef : string (content-addressed URI of the
                         executed rules-engine trace)
  modelOutputs      : array of object (each model's identifier,
                         version, output score, and the
                         operator's adverse-action explanation
                         template reference)
  rationaleHash     : string (SHA-256 of the rendered rationale
                         text presented to the consumer when
                         applicable)
```

## §8 Claim Record

```
claim:
  claimId            : string (uuidv7)
  policyId           : string (uuidv7)
  reportedAt         : string (ISO 8601)
  occurredAt         : string (ISO 8601; date and time of loss)
  reportedBy         : enum ("policyholder" | "insured" |
                         "third-party" | "broker" |
                         "automated-detection")
  perilCode          : string (operator's peril register code)
  causeNarrativeRef  : string (content-addressed URI of the
                         narrative; PII-redacted before any
                         third-party disclosure)
  reserve            : object (case reserve by coverage and by
                         loss component, in ISO 4217 currency)
  paymentsToDate     : object (per-coverage cumulative paid)
  claimStatus        : enum ("opened" | "investigating" |
                         "in-litigation" | "settling" |
                         "settled" | "closed-no-payment" |
                         "reopened")
  litigationFlag     : boolean
  fraudInvestigationRef : string (URI of the fraud
                         investigation when one is opened)
```

## §9 Reinsurance Cession Record

```
cession:
  cessionId          : string (uuidv7)
  treatyRef          : string (URI of the reinsurance treaty)
  cededPolicies      : array of string (policy UUIDs; or, for
                         facultative cessions, individual
                         policies referenced by UUID)
  cededLayer         : object (attachment / limit / share)
  cededPremium       : object (per ISO 4217 currency)
  cededLossesToDate  : object (per ISO 4217 currency)
  bordereauRef       : string (content-addressed URI of the
                         bordereau the operator emits to the
                         reinsurer at the agreed cadence)
```

## §10 Consumer Disclosure Record

```
consumerDisclosure:
  disclosureId       : string (uuidv7)
  policyId           : string (uuidv7; or claimId)
  disclosureKind     : enum ("policy-document" | "key-facts" |
                         "complaints-procedure" |
                         "renewal-notice" | "rate-change-notice"
                         | "adverse-action-notice" |
                         "claim-decision-letter")
  renderedAt         : string (ISO 8601)
  channel            : enum ("postal" | "email" | "in-app" |
                         "broker-handed")
  artefactRef        : string (content-addressed URI of the
                         rendered artefact in the consumer's
                         preferred language)
  acknowledgedAt     : string (ISO 8601; absent until consumer
                         acknowledged where acknowledgement is
                         required)
```

## §11 Premium Receipt and Accounting Record

Premium receipts and the corresponding accounting entries are
recorded so that IFRS 17 (or local-GAAP equivalent) reporting can
reconstruct premium recognition, deferred-acquisition costs, and
unearned-premium reserves on a per-policy basis.

```
premiumReceipt:
  receiptId          : string (uuidv7)
  policyId           : string (uuidv7)
  receivedAt         : string (ISO 8601)
  channel            : enum ("direct-debit" | "card-payment" |
                         "wire-transfer" | "broker-account" |
                         "agent-cash-receipt" |
                         "payroll-deduction")
  amount             : object (currency code per ISO 4217 and
                         numeric amount)
  allocation         : array of object (per-coverage premium /
                         tax / levy split with each amount)
  ifrs17ContractGroup: string (operator's IFRS 17 contract-group
                         identifier; supports profitability
                         tracking at the level the standard
                         requires)
```

## §12 Conformance

Implementations claiming PHASE-1 conformance emit each of the
records defined above for every active programme and honour the
party-token PII rule in §3.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-insurtech
- **Last Updated:** 2026-04-28
