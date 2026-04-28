# WIA-education-integration PHASE 1 — Data Format Specification

**Standard:** WIA-education-integration
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format layer
for WIA-education-integration. The standard covers
the persistent record shapes that an education-
integration operator (a national education ministry,
a multilateral education-cooperation programme, an
international school, an education-development
agency, an inter-institutional credit-recognition
authority, a refugee-education programme, an
adult-education or continuing-education provider, a
cross-border-mobility scholarship operator, an
education-data warehouse) maintains when registering
an education programme, classifying a learner's
qualification under the UNESCO ISCED 2011
classification, recording a credit-recognition
agreement, declaring a learning-mobility flow, and
binding a learner's academic record to a recognition
authority's register. Records are consumed by the
host institution receiving an inbound learner, by
the home institution releasing the outbound learner,
by the credit-recognition authority issuing the
recognition decision, by the supervisory ministry
enforcing the national qualifications framework, and
by the multilateral organisation collecting the
education-statistics return.

References (CITATION-POLICY ALLOW only):

- UNESCO Institute for Statistics ISCED 2011
  (International Standard Classification of
  Education) and ISCED-F 2013 (fields of
  education and training)
- UNESCO Convention on the Recognition of Studies,
  Diplomas and Degrees in Higher Education in the
  Asia and the Pacific (the Tokyo Convention,
  2011), the UNESCO Global Convention on the
  Recognition of Qualifications concerning
  Higher Education (2019), the Council of Europe
  / UNESCO Lisbon Recognition Convention (1997)
- UNESCO 2030 Education Framework for Action
- OECD Programme for International Student
  Assessment (PISA), OECD Education at a Glance
  indicator framework
- ISO/IEC 19796-1:2005 (information technology
  for learning, education and training — quality
  management, assurance and metrics)
- ISO/IEC 23988:2007 (information technology — a
  code of practice for the use of information
  technology in the delivery of assessments)
- ISO 21001:2018 (educational organisations
  management system requirements)
- ISO/IEC 36000-1:2024 (information technology
  for learning, education and training — overview)
- ISO/IEC 19778-1:2015 / -2 / -3 (collaborative
  workplace data model — collaborative
  environment)
- ISO/IEC 24751-1:2008 (individualized
  adaptability and accessibility in e-learning)
- IMS Global Learning Consortium standards (LTI
  1.3 / Advantage, OneRoster 1.2, Caliper
  Analytics 1.2, QTI 3.0, Common Cartridge 1.3,
  Open Badges 3.0)
- IEEE 1484.12.1-2020 (Learning Object Metadata)
- ADL xAPI 2.0 (the Experience API specification)
- Europass Qualifications Framework, the European
  Qualifications Framework for Lifelong Learning
  (EQF) Recommendation 2017/C 189/03
- W3C Verifiable Credentials Data Model v2.0,
  W3C Decentralized Identifiers v1.0
- IETF RFC 8259 (JSON), RFC 4122 (UUID), ISO 8601
  (date-time)
- ISO 3166-1 alpha-3 (country codes), ISO 639-3
  (language codes), ISO 4217 (currency codes)
- ISO/IEC 27001:2022 (information security
  management — used for the chain-of-custody
  record discipline in §8)
- KS X ISO/IEC 19796-1 (Korean adoption of
  ISO/IEC 19796-1) and KS X ISO 21001 (Korean
  adoption of ISO 21001)

---

## §1 Scope

This PHASE defines persistent shapes for the
artefacts exchanged when a learner's education
record is registered, the qualification is
classified per ISCED 2011, the inter-institutional
credit-recognition is declared, the cross-border
learning mobility is recorded, and the chain-of-
custody trail of the academic record is anchored.
Implementations covered include:

- A national-education-ministry programme
  registering its primary, lower-secondary,
  upper-secondary, post-secondary non-tertiary,
  short-cycle tertiary, bachelor, master, and
  doctoral programmes against ISCED 2011 levels
  0 to 8.
- A multilateral education-cooperation programme
  (the Erasmus+ programme operated by the
  European Commission, the UNESCO UNITWIN /
  UNESCO Chairs network, the OECD Centre for
  Effective Learning Environments) registering
  its inter-institutional mobility flows.
- An international school operating across two or
  more national qualifications frameworks (the
  International Baccalaureate, the Cambridge
  International Examinations, the Advanced
  Placement programme of the College Board)
  binding its learners' qualifications to each
  national framework's recognition table.
- An education-development agency (the Korea
  International Cooperation Agency's KOICA
  education programmes, the German GIZ education
  programmes, the Japanese JICA education
  programmes) registering its training-programme
  flow.
- An inter-institutional credit-recognition
  authority (a Bologna-Process accredited body, a
  Lisbon Recognition Convention competent
  authority, an Asia-Pacific Tokyo Convention
  competent authority) issuing recognition
  decisions.

The cross-jurisdictional recognition envelope
under the Lisbon and Tokyo Conventions, the
ISCED 2011 classification envelope, and the
quality-management envelope under ISO 21001 and
ISO/IEC 19796-1 receive distinct encodings in
this PHASE; the additional safeguards required
by each programme are encoded in PHASE-3 §3.

## §2 Programme Identifier

```
programmeId          : string (uuidv7)
operatorName         : string (legal name of the
                       operator — national
                       education ministry,
                       multilateral programme
                       secretariat, international
                       school, education-
                       development agency,
                       recognition authority)
operatorRole         : enum ("education-ministry"
                       | "multilateral-programme"
                       | "international-school" |
                       "education-development-
                       agency" | "recognition-
                       authority" | "user-defined")
governingFrameworks  : array of enum ("ISCED-2011"
                       | "ISCED-F-2013" |
                       "ISO-21001" |
                       "ISO-IEC-19796-1" |
                       "ISO-IEC-23988" |
                       "EQF" |
                       "Lisbon-Recognition-
                       Convention-1997" |
                       "Tokyo-Recognition-
                       Convention-2011" |
                       "Global-Recognition-
                       Convention-2019" |
                       "OECD-PISA" |
                       "ADL-xAPI-2.0" |
                       "IMS-LTI-1.3" |
                       "IMS-Caliper-1.2" |
                       "IMS-OneRoster-1.2" |
                       "IMS-QTI-3.0" |
                       "IMS-OpenBadges-3.0" |
                       "IEEE-1484.12.1-2020" |
                       "W3C-VC-2.0" |
                       "W3C-DID-1.0" |
                       "user-defined")
accreditationStatus  : object (the ISO 21001
                       certification reference for
                       educational-organisation
                       management-system operators,
                       the ISO/IEC 19796-1
                       conformance reference for
                       e-learning quality, the
                       Lisbon / Tokyo / Global
                       Convention competent-
                       authority designation
                       reference)
programmeStatus      : enum ("design" | "operating"
                       | "limited-rollout" |
                       "wind-down" | "archived")
```

## §3 ISCED 2011 Programme Record

```
iscedRecord:
  iscedRecordId      : string (uuidv7)
  programmeRef       : string (PHASE-1 §2 record
                       reference)
  iscedLevel         : enum (UNESCO ISCED 2011
                       Levels 0 through 8 —
                       0 early-childhood
                       education, 1 primary, 2
                       lower-secondary, 3 upper-
                       secondary, 4 post-secondary
                       non-tertiary, 5 short-cycle
                       tertiary, 6 bachelor or
                       equivalent, 7 master or
                       equivalent, 8 doctoral or
                       equivalent)
  iscedField         : object (UNESCO ISCED-F 2013
                       narrow-field-of-education
                       three-digit classification —
                       for example "061 Information
                       and Communication
                       Technologies (ICTs)" or
                       "0114 Teacher training
                       with subject specialisation")
  duration           : object (declared duration in
                       years and ECTS credits or
                       equivalent national credit
                       unit)
  awardName          : string (the qualification
                       awarded — for example
                       "Bachelor of Science", "고
                       등학교 졸업장", "Diplôme
                       National de Master")
  awardLanguage      : string (BCP 47 language tag
                       — for example "en", "ko",
                       "fr", "ja", "ar")
  nationalFrameworkRef : object (the per-country
                       national qualifications
                       framework binding —
                       Korean Qualifications
                       Framework KQF level
                       reference, EQF level
                       reference, Australian
                       Qualifications Framework
                       AQF reference, US Higher
                       Education degree level
                       reference)
```

## §4 Curriculum Record

```
curriculumRecord:
  curriculumId       : string (uuidv7)
  programmeRef       : string (PHASE-1 §2 record
                       reference)
  iscedRecordRef     : string (PHASE-1 §3 record
                       reference)
  curriculumName     : string (localised
                       curriculum name)
  modules            : array of object (per-module
                       descriptor — module
                       identifier, module title,
                       module's ECTS credit value
                       or national-credit
                       equivalent, module's
                       learning outcomes per the
                       Bologna learning-outcomes
                       framework, module's
                       prerequisite identifiers)
  totalCredits       : number (ECTS credits or
                       national credit unit)
  qualityAssuranceRef : string (the ISO 21001
                       quality-assurance reference
                       for the curriculum)
```

## §5 Learner Record

```
learnerRecord:
  learnerId          : string (uuidv7) — pseudonymous
                       per the operator's privacy-
                       discipline declaration in
                       §8
  homeInstitutionRef : string (the learner's home
                       institution's PHASE-1 §2
                       reference)
  hostInstitutionRef : string (the learner's host
                       institution's PHASE-1 §2
                       reference, where mobility
                       applies)
  enrolmentRef       : object (the per-programme
                       enrolment identifier and
                       the enrolment date)
  iscedLevel         : enum (the learner's current
                       ISCED 2011 level)
  citizenshipCountry : string (ISO 3166-1 alpha-3
                       — used for the recognition-
                       authority verification, not
                       for any commercial purpose)
  preferredLanguage  : string (BCP 47 language tag)
  accessibilityProfile : object (per ISO/IEC 24751
                       individualised-adaptability
                       declaration)
```

## §6 Recognition-Decision Record

```
recognitionRecord:
  recognitionId      : string (uuidv7)
  applicantRef       : string (PHASE-1 §5 record
                       reference)
  homeQualificationRef : string (the qualification
                       presented for recognition,
                       referenced through the
                       home institution's PHASE-1
                       §3 record)
  decisionAuthorityRef : string (the recognition
                       authority's PHASE-1 §2
                       reference)
  applicableConvention : enum ("Lisbon-Recognition-
                       Convention-1997" |
                       "Tokyo-Recognition-
                       Convention-2011" |
                       "Global-Recognition-
                       Convention-2019" | "user-
                       defined")
  decisionOutcome    : enum ("recognised-fully" |
                       "recognised-with-conditions"
                       | "partially-recognised" |
                       "not-recognised")
  decisionRationale  : string (per Lisbon Article
                       III.3, the recognition
                       authority's substantive
                       reasoning)
  appealRights       : object (per Lisbon Article
                       III.5, the applicant's
                       appeal-and-review rights)
  decisionDate       : string (ISO 8601 date-time)
  decisionExpiry     : string (ISO 8601 date-time
                       — where the decision is
                       time-limited)
```

## §7 Mobility Record

```
mobilityRecord:
  mobilityId         : string (uuidv7)
  learnerRef         : string (PHASE-1 §5 record
                       reference)
  programmeType      : enum ("erasmus-plus-credit"
                       | "erasmus-plus-degree" |
                       "joint-degree" |
                       "exchange-semester" |
                       "summer-school" |
                       "research-stay" |
                       "internship" | "user-
                       defined")
  fromInstitutionRef : string (the sending
                       institution's PHASE-1 §2
                       reference)
  toInstitutionRef   : string (the receiving
                       institution's PHASE-1 §2
                       reference)
  fromCountry        : string (ISO 3166-1 alpha-3)
  toCountry          : string (ISO 3166-1 alpha-3)
  startDate          : string (ISO 8601)
  endDate            : string (ISO 8601)
  creditsEarned      : number (ECTS or national
                       credit unit)
  learningAgreementRef : string (URL of the
                       countersigned learning
                       agreement)
```

## §8 Chain-of-Custody Record

```
custodyRecord:
  custodyId          : string (uuidv7)
  artefactRef        : string (the recognition
                       decision, mobility record,
                       or curriculum identifier)
  custodyEvent       : enum ("recognition-decision-
                       issued" | "mobility-
                       agreement-signed" |
                       "curriculum-published" |
                       "transcript-issued" |
                       "diploma-issued" |
                       "credential-revoked" |
                       "user-defined")
  eventTimestamp     : string (ISO 8601 date-time)
  performingParty    : string (legal entity
                       reference)
  hashOfArtefacts    : string (SHA-256 hex digest)
```

## §9 Manifest

Implementations publish a signed manifest carrying
`standardSlug` (constant value "education-
integration"), `version`, `implementation`, the
operator's `accreditationStatus`, and the
`profile` declaration that selects which of the
optional records (curriculum, recognition,
mobility) the implementation supports. The
manifest is signed using a key whose public part
is published on the operator's
`.well-known/wia/education-integration/`
discovery endpoint declared in PHASE-2.

弘益人間 (Hongik Ingan) — Benefit All Humanity
