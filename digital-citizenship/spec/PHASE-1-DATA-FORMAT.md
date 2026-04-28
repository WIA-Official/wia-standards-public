# WIA-digital-citizenship PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-digital-citizenship
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format layer for
WIA-digital-citizenship. The standard covers persistent
record shapes for digital-citizenship programmes
operated by public-sector and civil-society
organisations — programmes that build the digital
literacy, online-safety, civic-participation, and
inclusive-access competencies of citizens — under the
operating jurisdiction's curriculum framework, the
operating jurisdiction's privacy regime applied to
minor learners (US COPPA where the operating
jurisdiction is the US, UK Age-Appropriate Design Code
where the operating jurisdiction is the UK, EU GDPR
Article 8 children-of-information-society-services
discipline elsewhere in the EU), and the operating
jurisdiction's accessibility regime (Section 508
where US, EU Web Accessibility Directive 2016/2102
elsewhere in the EU, equivalent national rules
elsewhere). The format is consumed by programme
operators, the operating jurisdiction's education
ministry or programme funder, civil-society partner
organisations, and the parents and guardians of
minor learners.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time representation)
- ISO/IEC 11578 (UUID)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 27018:2019 (PII in public clouds)
- ISO/IEC 27701:2019 (privacy information management)
- ISO/IEC 24751-1:2008 (individualised adaptability and
  accessibility in e-learning)
- ISO/IEC 19796-1:2005 (information technology —
  learning, education and training — quality
  management)
- IETF RFC 4122 (UUID URN)
- IETF RFC 8259 (JSON)
- IETF RFC 9457 (Problem Details)
- W3C Web Content Accessibility Guidelines (WCAG)
  2.2 Level AA (cited as the canonical accessibility
  baseline)
- W3C Verifiable Credentials Data Model 2.0 (used in
  PHASE-4 for optional re-issuance)
- UNESCO Digital Literacy Global Framework (cited as
  the operating-jurisdiction-agnostic literacy
  reference; the operating jurisdiction's curriculum
  governs adoption)
- UN Convention on the Rights of the Child Article
  17 + General Comment No. 25 on children's rights
  in relation to the digital environment (cited as
  the international children's-rights baseline that
  the operating jurisdiction transposes)
- US COPPA (Children's Online Privacy Protection Act)
  + 16 CFR Part 312 (cited where the operating
  jurisdiction is the US)
- UK Age-Appropriate Design Code (the ICO's code on
  online services likely to be accessed by children)
- EU GDPR Article 8 (cited where the operating
  jurisdiction is an EU Member State, with the
  Member State's age threshold ranging from 13 to 16
  per Article 8(1))
- EU Digital Education Action Plan 2021-2027 (cited
  as the EU policy framework that funds digital-
  citizenship programmes in EU Member States)
- US Section 508 of the Rehabilitation Act + 36 CFR
  Part 1194 (cited where the operating jurisdiction
  is the US)
- EU Web Accessibility Directive 2016/2102 (cited
  where the programme operator is a public-sector
  body in the EU)
- EU European Accessibility Act 2019/882 (cited
  where the programme operator's product surface is
  in scope of the EAA, in force June 2025)

---

## §1 Scope

This PHASE defines persistent shapes for the artefacts
a digital-citizenship programme manages. Implementations
covered include:

- School-based digital-citizenship curricula operated
  by public-school authorities.
- Library-based digital-literacy programmes operated
  by public libraries and library consortia.
- Civil-society digital-citizenship programmes
  operated by NGOs serving older-adult learners,
  refugee learners, and other underserved populations.
- Public-sector e-government literacy programmes that
  build citizen capacity to use the operating
  jurisdiction's digital public services.
- Online-safety education programmes (cyberbullying
  awareness, mis- and dis-information literacy,
  privacy literacy, financial-fraud awareness).
- Civic-participation literacy programmes (e-voting
  awareness where the operating jurisdiction operates
  e-voting, public-consultation participation,
  freedom-of-information-request literacy).

Programmes that mix in workforce-development outcomes
(employer-recognised IT certifications) are addressed
through the operating jurisdiction's workforce-
development records system; this PHASE addresses the
citizenship-outcome record shapes only.

## §2 Programme Identifier

```
programmeId          : string (uuidv7)
programmeOperator    : string (legal name of the
                       programme operator)
operatingJurisdiction : array of string (ISO 3166-1
                       country codes of the
                       programme's operations)
operatorClass        : enum ("public-school" |
                       "public-library" |
                       "ngo-civil-society" |
                       "public-sector-e-government" |
                       "private-not-for-profit" |
                       "user-defined")
funderClass          : enum ("national-education-
                       ministry" | "subnational-
                       education-authority" |
                       "national-library-funder" |
                       "philanthropic-grant" |
                       "eu-digital-education-action-
                       plan" | "private-corporate-
                       social-responsibility" |
                       "user-defined")
governingFrameworks  : array of enum ("UNESCO-
                       Digital-Literacy-Framework" |
                       "operating-jurisdiction-
                       curriculum" |
                       "WCAG-2-2-Level-AA" |
                       "US-COPPA-16-CFR-312" |
                       "UK-Age-Appropriate-Design-Code"
                       | "EU-GDPR-Art-8" |
                       "US-Section-508" |
                       "EU-Web-Accessibility-
                       Directive-2016-2102" |
                       "EU-European-Accessibility-Act-
                       2019-882" |
                       "user-defined")
learnerAgeRange      : object (minAge + maxAge; the
                       operating jurisdiction's age-
                       of-consent threshold under EU
                       GDPR Article 8 / US COPPA / UK
                       Age-Appropriate Design Code
                       informs the per-learner
                       privacy-record requirements)
programmeStatus      : enum ("design" |
                       "pilot-cohort" | "operating" |
                       "wind-down" | "archived")
```

## §3 Curriculum-Module Record

```
curriculumModule:
  moduleId           : string (uuidv7)
  programmeId        : string (uuidv7)
  moduleName         : string
  competencyArea     : enum ("digital-literacy-
                       foundational" | "online-
                       safety" | "privacy-literacy" |
                       "mis-and-dis-information-
                       literacy" | "civic-
                       participation" | "inclusive-
                       digital-access" | "creative-
                       expression" | "digital-well-
                       being" | "user-defined")
  unescoFrameworkRef : string (URI of the UNESCO
                       Digital Literacy Global
                       Framework competency reference,
                       where the programme adopts
                       UNESCO competency mapping)
  jurisdictionCurriculumRef : string (URI of the
                       operating jurisdiction's
                       curriculum-framework
                       competency the module aligns
                       to)
  learnerOutcomes    : array of string (learner-
                       outcome statements drafted in
                       age-appropriate language)
  deliveryMode       : enum ("classroom-instructor-
                       led" | "library-workshop" |
                       "self-paced-online" |
                       "blended-classroom-and-online"
                       | "community-event" |
                       "user-defined")
  accessibilityProfileRef : string (URI of the WCAG
                       2.2 conformance evidence and
                       per-disability adaptation
                       evidence for the module
                       artefacts — captioning, sign-
                       language interpretation, large-
                       print, braille, plain-language
                       summary)
```

## §4 Cohort-Enrolment Record

```
cohort:
  cohortId           : string (uuidv7)
  programmeId        : string (uuidv7)
  enrolmentOpenAt    : string (ISO 8601)
  enrolmentCloseAt   : string (ISO 8601)
  cohortStartAt      : string (ISO 8601)
  cohortEndAt        : string (ISO 8601)
  enrolmentChannel   : enum ("school-class-
                       enrolment" | "library-walk-in"
                       | "ngo-referral" | "public-
                       sector-portal" | "self-service-
                       online" | "user-defined")
  parentalConsentDiscipline : enum ("not-required-
                       above-age-threshold" |
                       "required-per-coppa" |
                       "required-per-gdpr-art-8" |
                       "required-per-uk-aadc" |
                       "user-defined")
  inclusiveAccessProvisions : array of string (per-
                       provision references — assistive
                       technology loans, transport
                       support, on-site interpreter,
                       on-site childcare, refreshment)
```

## §5 Learner Record

```
learner:
  learnerId          : string (uuidv7)
  programmeId        : string (uuidv7)
  cohortRef          : string (cohort identifier)
  enrolledAt         : string (ISO 8601)
  ageAtEnrolment     : integer (in years)
  parentalConsentRef : string (URI of the parental
                       consent capture, where the
                       parentalConsentDiscipline
                       requires consent)
  inclusiveAccessNeedsRef : string (URI of the
                       declared accessibility needs;
                       the programme's inclusive-
                       access provisions are
                       allocated against this record)
  withdrawalAt       : string (ISO 8601; absent
                       unless withdrawn)
  withdrawalReasonRef : string (URI; absent unless
                       withdrawn)
```

## §6 Module-Completion Record

```
moduleCompletion:
  completionId       : string (uuidv7)
  learnerRef         : string (learner identifier)
  moduleRef          : string (curriculum-module
                       identifier)
  completedAt        : string (ISO 8601)
  competencyAttestation : enum ("instructor-attested-
                       completion" | "self-attested-
                       completion" | "assessed-and-
                       verified" | "informal-
                       participation")
  artefactRef        : string (URI of the learner's
                       completion artefact — for
                       creative-expression modules
                       this is the learner's project;
                       for assessed modules this is
                       the assessment record)
```

## §7 Programme-Outcome Aggregate Record

```
programmeOutcome:
  outcomeId          : string (uuidv7)
  programmeId        : string (uuidv7)
  reportPeriodStart  : string (ISO 8601)
  reportPeriodEnd    : string (ISO 8601)
  enrolledCount      : integer
  completedCount     : integer
  per-AgeBandSummary : array of object (per-age-band
                       enrolled and completed counts;
                       age-band granularity per the
                       operating jurisdiction's
                       reporting-statistics rules)
  per-CompetencyAreaSummary : array of object (per-
                       competency-area enrolled and
                       completed counts)
  inclusiveAccessUseSummary : array of object (per-
                       provision use counts —
                       assistive technology loans,
                       interpreter requests, etc.)
  reportArtefactRef  : string (URI of the rendered
                       report; the report is published
                       to the programme's funder per
                       the funder's reporting
                       requirements)
```

## §8 Online-Safety Incident Record

For programmes that include online-safety content
where learner experience generates incidents
worth recording:

```
onlineSafetyIncident:
  incidentId         : string (uuidv7)
  programmeId        : string (uuidv7)
  reportedAt         : string (ISO 8601)
  reporterRef        : string (the reporter — the
                       learner, the learner's parent
                       or guardian, the programme
                       instructor, a peer)
  incidentClassification : enum ("cyberbullying" |
                       "harassment" | "exposure-to-
                       harmful-content" | "scam-or-
                       fraud-attempt" | "grooming-
                       attempt" | "image-based-abuse"
                       | "user-defined")
  triagedAt          : string (ISO 8601)
  triageOutcome      : enum ("internal-resolution-
                       complete" | "escalated-to-
                       parent-or-guardian" |
                       "escalated-to-school-
                       administration" | "escalated-
                       to-online-safety-authority" |
                       "escalated-to-law-enforcement"
                       | "user-defined")
  pastoralSupportRef : string (URI of the pastoral
                       support narrative; the
                       programme's safeguarding lead
                       coordinates the support)
```

## §9 Public-Sector E-Government Literacy Record

For programmes whose operatorClass is `public-sector-
e-government`, a per-service-domain record:

```
eGovernmentLiteracyTopic:
  topicId            : string (uuidv7)
  programmeId        : string (uuidv7)
  serviceDomain      : enum ("identity-and-
                       authentication" | "tax-
                       filing" | "welfare-benefits-
                       application" | "healthcare-
                       enrolment" | "voting-
                       registration" | "freedom-of-
                       information-request" |
                       "complaint-and-feedback" |
                       "user-defined")
  jurisdictionPortalRef : string (URI of the
                       operating jurisdiction's e-
                       government portal the topic
                       teaches)
  accessibilityProfileRef : string (URI of the
                       portal's WCAG 2.2 / EU Web
                       Accessibility Directive
                       conformance reference)
  learnerOutcomeMeasure : enum ("self-attested-
                       completion-of-portal-task" |
                       "instructor-observed-
                       completion" | "post-
                       programme-survey-confidence-
                       score" | "user-defined")
```

## §10 Conformance

Implementations claiming PHASE-1 conformance emit
each of the records defined above for every operating
programme, retain learner records for the operating
jurisdiction's records-retention horizon (typically
seven years for public-school records, shorter for
public-library and NGO programmes per the operator's
data-retention policy and the operating jurisdiction's
privacy regime), and preserve programme-outcome
aggregate records for the operating jurisdiction's
public-archive horizon.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-digital-citizenship
- **Last Updated:** 2026-04-28
