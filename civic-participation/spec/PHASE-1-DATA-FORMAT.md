# WIA-civic-participation PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-civic-participation
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format
layer for WIA-civic-participation. The standard
covers the persistent record shapes that a
public-administration body operating a citizen-
engagement platform (a national government's
e-petition system, a municipal participatory-
budgeting platform, a regional public-
consultation portal, a parliamentary written-
contribution intake, a local council citizen-
assembly secretariat), a third-party civic-tech
service operator, an academic or civil-society
observer running a transparency-of-engagement
audit, and a public-procurement authority running
an inclusive-engagement programme maintain when
publishing a consultation, recording a citizen
submission, anchoring the consultation outcome to
a deliberative record, tracking the per-citizen
participation trail under privacy-by-design
principles, and integrating the engagement record
into a smart-sustainable-city KPI dashboard.
Records are consumed by the citizen submitter,
by the elected representative receiving the
consultation outcome, by the civil-society
auditor publishing the post-consultation
analysis, by the e-government interoperability
service, and — where the platform handles
sensitive-category data — by the supervisory
data-protection authority overseeing the
processing.

References (CITATION-POLICY ALLOW only):

- ITU-T Recommendation Y.4900/L.1600 (Overview
  of key performance indicators in smart
  sustainable cities) and ITU-T Y.4901 / Y.4902
  / Y.4903 (the per-domain KPI series cited
  normatively for the engagement-related KPIs
  in §4 and §7)
- ISO 37120:2018 (sustainable cities and
  communities — indicators for city services
  and quality of life) and ISO 37122:2019
  (indicators for smart cities) and ISO 37123:
  2019 (indicators for resilient cities)
- ISO 37100:2016 (sustainable cities and
  communities — vocabulary) and ISO 37101:2016
  (sustainable development in communities —
  management system for sustainable development)
- ISO 18091:2019 (quality management systems —
  guidelines for the application of ISO 9001 in
  local government) — the local-government
  QMS that anchors the operator's process
  governance
- W3C Open Digital Rights Language (ODRL) 2.2
  Information Model and W3C ODRL Vocabulary &
  Expression 2.2 (cited normatively for the
  rights-and-policy expression carried by the
  consultation envelope in §3 and §4)
- W3C Data Catalog Vocabulary (DCAT) v3 (cited
  for the per-consultation dataset description
  in §4)
- W3C Decentralised Identifiers (DIDs) v1.0 and
  W3C Verifiable Credentials Data Model v2.0
  (cited where the operator binds a citizen
  identity to a verifiable-credential issued by
  a national identity provider)
- UN Department of Economic and Social Affairs
  E-Government Survey (the biennial e-
  government and e-participation index reference)
- OECD Recommendation of the Council on Open
  Government (OECD/LEGAL/0438) and OECD Citizen
  Engagement: A Guide to Public Engagement
  (the framework reference for the
  engagement-classification taxonomy in §3)
- World Bank GovTech Maturity Index (the cross-
  jurisdictional benchmark cited where the
  operator publishes a maturity-comparator)
- IETF RFC 8259 (JSON), RFC 4122 (UUID), ISO
  8601 (date and time)
- ISO/IEC 27001:2022 (information-security
  management — used for the chain-of-custody
  record discipline in §8)
- EU Regulation (EU) 2018/1724 (Single Digital
  Gateway) and EU Regulation (EU) 2024/903
  (Interoperable Europe Act) — the EU-level
  framework cited where the operator
  participates in the Single Digital Gateway
- EU GDPR (Regulation (EU) 2016/679) Articles
  6, 9, 12 to 22 (cited for the processing
  basis of citizen-submission data)
- KR 행정기본법 (Framework Act on Administrative
  Affairs) and KR 정보공개법 (Official
  Information Disclosure Act)

---

## §1 Scope

This PHASE defines persistent shapes for the
artefacts exchanged when a public-administration
body opens a consultation, ingests citizen
submissions, deliberates against a documented
decision-record, and publishes the outcome.
Implementations covered include:

- A national e-petition platform processing
  citizen petitions against a defined
  threshold-and-response discipline.
- A municipal participatory-budgeting platform
  publishing the per-cycle citizen-proposed
  project list and the per-project deliberation
  record.
- A regional public-consultation portal under
  the EU Single Digital Gateway aggregating
  cross-border citizen submissions.
- A parliamentary written-contribution intake
  binding the citizen submission to a per-
  legislator analysis dossier.
- A local council citizen-assembly secretariat
  operating a randomly-selected citizen-panel
  deliberation under a sortition discipline.
- A smart-city dashboard publishing engagement
  KPIs under ITU-T Y.4900 / ISO 37120/37122.

The petition record, the participatory-budget
proposal, the public-consultation submission,
and the citizen-assembly deliberation record
receive distinct encodings in this PHASE; the
additional safeguards required by each engagement
regime are encoded in PHASE-3 §4.

## §2 Programme Identifier

```
programmeId          : string (uuidv7)
operatorName         : string (legal name of the
                       operator — national
                       government body, municipal
                       authority, regional
                       authority, parliament,
                       council secretariat, or
                       civic-tech service operator)
operatorRole         : enum ("national-government"
                       | "municipal-authority" |
                       "regional-authority" |
                       "parliament-secretariat" |
                       "council-secretariat" |
                       "civic-tech-service" |
                       "transparency-observer" |
                       "user-defined")
operatorJurisdiction : array of string (ISO 3166-1
                       country codes; municipal
                       authorities additionally
                       carry the ISO 3166-2
                       subdivision code)
governingFrameworks  : array of enum ("ITU-T-Y-
                       4900" | "ISO-37120" |
                       "ISO-37122" | "ISO-37123" |
                       "ISO-37100" | "ISO-37101" |
                       "ISO-18091" | "W3C-ODRL" |
                       "W3C-DCAT-3" |
                       "W3C-DID-1" |
                       "W3C-VC-2" |
                       "OECD-OPEN-GOV-0438" |
                       "OECD-CITIZEN-ENGAGEMENT" |
                       "UN-DESA-EGOV-SURVEY" |
                       "WB-GOVTECH-MATURITY" |
                       "EU-SDG-2018-1724" |
                       "EU-INTEROP-2024-903" |
                       "EU-GDPR" |
                       "KR-행정기본법" |
                       "KR-정보공개법" |
                       "user-defined")
engagementMaturity   : object (the World Bank
                       GovTech Maturity Index
                       score, the OECD OURdata
                       Index score, and the UN
                       DESA E-Participation Index
                       score where applicable)
programmeStatus      : enum ("design" | "operating"
                       | "limited-rollout" |
                       "wind-down" | "archived")
```

## §3 Consultation Record

```
consultationRecord:
  consultationId     : string (uuidv7)
  publisher          : string (the operator's
                       legal-entity reference)
  consultationType   : enum ("e-petition" |
                       "participatory-budgeting"
                       | "public-consultation" |
                       "written-contribution" |
                       "citizen-assembly" |
                       "binding-referendum" |
                       "advisory-poll" |
                       "user-defined")
  engagementLadder   : enum ("inform" | "consult"
                       | "involve" | "collaborate"
                       | "empower")
  subject            : object (the policy
                       question, the legislative
                       proposal reference, the
                       budget allocation
                       proposal, or the planning
                       application reference
                       under deliberation)
  rightsExpression   : object (the W3C ODRL 2.2
                       policy expression carrying
                       the consultation's permission
                       to redistribute, to use for
                       research, to anonymise, or
                       to retain after closure)
  openingDate        : string (ISO 8601 date-time)
  closingDate        : string (ISO 8601 date-time)
  responseSchedule   : object (the operator's
                       commitment to publish the
                       per-submission acknowledgment
                       and the consultation
                       outcome at a declared
                       timeline)
  jurisdictionalScope : enum ("national" |
                       "regional" | "municipal" |
                       "ward" | "supranational"
                       | "user-defined")
```

## §4 Submission Record

```
submissionRecord:
  submissionId       : string (uuidv7)
  consultationRef    : string (PHASE-1 §3 record
                       reference)
  submitter          : object (the submitter's
                       declared identity envelope
                       — anonymous, pseudonymous,
                       verified-citizen via a
                       national identity provider
                       carrying a W3C Verifiable
                       Credential, or institutional
                       — each carrying the
                       declared scope of identity
                       disclosure)
  submissionType     : enum ("free-text" |
                       "structured-form" |
                       "vote" | "ranked-choice" |
                       "budget-allocation" |
                       "amendment-text" |
                       "supporting-evidence" |
                       "user-defined")
  body               : object (the submission
                       payload under the schema
                       declared by the
                       consultation envelope —
                       free text encoded per RFC
                       8259, structured-form
                       fields, vote tally,
                       ranked-choice ordering,
                       per-line-item budget
                       allocation)
  attachments        : array of object (per-
                       attachment file reference
                       under content-addressable
                       URI; each attachment
                       carries its declared mime
                       type and the SHA-256 hex
                       digest)
  consentDirective   : object (the GDPR Article
                       6/9 processing basis
                       declared by the submitter,
                       and the per-purpose consent
                       for downstream re-use such
                       as research-use or open-
                       data publication)
  submissionTimestamp : string (ISO 8601)
```

## §5 Deliberation Record

```
deliberationRecord:
  deliberationId     : string (uuidv7)
  consultationRef    : string (PHASE-1 §3 record
                       reference)
  deliberativeForum  : enum ("legislator-analysis"
                       | "council-committee" |
                       "citizen-panel-sortition"
                       | "expert-panel" |
                       "public-hearing" |
                       "user-defined")
  participantSet     : array of object (per-
                       participant role —
                       elected representative,
                       sortition-selected citizen,
                       expert advisor, civil-
                       society observer — each
                       carrying the participant's
                       declared role and the
                       conflict-of-interest
                       declaration where
                       applicable)
  agenda             : object (the per-session
                       agenda binding the
                       deliberation to the
                       submissions referenced)
  decisionRecord     : object (the documented
                       conclusion of the
                       deliberation — accept,
                       accept-with-modification,
                       defer, reject; carrying
                       the cited submission set
                       that informed the
                       decision and the per-
                       decision rationale)
```

## §6 Outcome Publication Record

```
outcomeRecord:
  outcomeId          : string (uuidv7)
  consultationRef    : string (PHASE-1 §3 record
                       reference)
  publicationType    : enum ("response-to-each-
                       submission" | "aggregated-
                       summary" | "decision-with-
                       rationale" | "follow-up-
                       action" | "user-defined")
  publicationDate    : string (ISO 8601)
  publicationChannel : enum ("operator-portal" |
                       "official-gazette" |
                       "single-digital-gateway"
                       | "broadcast" | "user-
                       defined")
  followUpActions    : array of object (the
                       commitments declared in
                       the outcome — legislative
                       amendment to be tabled,
                       regulatory amendment to be
                       proposed, budget
                       allocation to be
                       requested, programme to be
                       launched)
```

## §7 Engagement KPI Record

```
kpiRecord:
  kpiId              : string (uuidv7)
  programmeRef       : string (the programme
                       identifier)
  reportingPeriod    : object (start and end
                       dates per ISO 8601)
  kpiBundle          : object (per-indicator
                       value — the ITU-T Y.4900
                       smart-city engagement
                       KPI, the ISO 37120 §15
                       (Recreation) / §22
                       (Telecommunication and
                       Innovation) per-indicator
                       value, the ISO 37122
                       smart-city participation-
                       intensity indicator, the
                       UN DESA E-Participation
                       Index sub-component, and
                       the OECD OURdata Index
                       sub-component as
                       applicable)
```

## §8 Chain-of-Custody Record

```
custodyRecord:
  custodyId          : string (uuidv7)
  artefactRef        : string (the consultation,
                       submission, deliberation,
                       outcome, or KPI record
                       identifier)
  custodyEvent       : enum ("consultation-
                       opened" | "submission-
                       received" | "submission-
                       acknowledged" |
                       "deliberation-conducted"
                       | "decision-recorded" |
                       "outcome-published" |
                       "kpi-published" |
                       "redaction-applied" |
                       "withdrawal" | "user-
                       defined")
  eventTimestamp     : string (ISO 8601 date-time)
  performingParty    : string (legal entity)
  hashOfArtefacts    : string (SHA-256 hex digest)
```

## §9 Manifest

Implementations publish a signed manifest carrying
`standardSlug` (constant value "civic-
participation"), `version`, `implementation`,
the operator's `engagementMaturity` envelope,
and the `profile` declaration that selects which
of the optional records (deliberation, outcome,
KPI) the implementation supports. The manifest
is signed using a key whose public part is
published on the operator's
`.well-known/wia/civic-participation/` discovery
endpoint declared in PHASE-2.
