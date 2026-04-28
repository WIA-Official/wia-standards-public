# WIA-family-reunion-data PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-family-reunion-data
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format
layer for WIA-family-reunion-data. The standard
covers the persistent record shapes that a
humanitarian operator (a Red Cross or Red
Crescent national society running a Restoring
Family Links service, the UNHCR field office
operating a ProGres registration, an IOM field
office running a Displacement Tracking Matrix
operation, a national Ministry of the Interior
operating a missing-persons register, an inter-
country adoption central authority, a child-
protection authority running a separated-or-
unaccompanied-minor case file, a war-graves
commission), an inter-agency coordination body
(the Inter-Agency Standing Committee, the
UNHCR Operational Data Portal), an academic or
civil-society observer auditing the
humanitarian-data discipline, and a
supervisory data-protection authority oversee-
ing GDPR Article 9 special-category processing
maintain when registering a separated person,
recording a tracing request, anchoring a
positive identification, transferring the
case file to a partner agency under a documented
data-sharing agreement, and tracking the per-
case chain-of-custody trail. Records are
consumed by the registered person, by the
named relatives requesting reunion, by the
partner agency receiving the transferred case,
by the destination state's reception
authorities, and — where the person is a
minor — by the child-protection authority and
the appointed guardian.

References (CITATION-POLICY ALLOW only):

- 1949 Geneva Conventions and their 1977
  Additional Protocols — the international
  humanitarian law framework cited normatively
  for the per-case humanitarian principle
  declaration in §3
- 1951 Convention Relating to the Status of
  Refugees and its 1967 Protocol — the
  refugee-protection framework cited where the
  registered person is a refugee or asylum-
  seeker
- 1989 United Nations Convention on the
  Rights of the Child (CRC), particularly
  Articles 7 (registration, name and
  nationality), 8 (preservation of identity),
  9 (separation from parents), 10 (family
  reunification), 11 (illicit transfer abroad),
  20 (children deprived of family
  environment), 22 (refugee children), and 35
  (abduction)
- 1993 Hague Convention on Protection of
  Children and Co-operation in respect of
  Inter-country Adoption — the framework
  cited where the case is in scope of inter-
  country adoption
- 1980 Hague Convention on the Civil Aspects
  of International Child Abduction — cited
  where the case involves cross-border child
  abduction
- 1963 Vienna Convention on Consular Relations
  (Article 36) — cited where the case
  requires consular notification
- 1961 Convention on the Reduction of
  Statelessness — cited where the registered
  person is stateless
- ICRC Restoring Family Links (RFL) Strategy
  for the International Red Cross and Red
  Crescent Movement (2008-2018, 2020-2025) —
  the governing framework for the RFL
  service of national Red Cross and Red
  Crescent societies
- ICRC Professional Standards for Protection
  Work — the methodological framework cited
  for the per-case protection-monitoring
  envelope
- UNHCR ProGres v4 (Profile Global
  Registration System) — the UNHCR
  registration database schema cited
  normatively for the per-individual
  registration envelope in §4
- UNHCR Policy on the Protection of Personal
  Data of Persons of Concern (2015 and its
  2018 update)
- IOM Displacement Tracking Matrix (DTM)
  Methodological Framework
- IOM Data Protection Manual
- Humanitarian Exchange Language (HXL)
  Standard maintained by the OCHA Centre for
  Humanitarian Data
- Sphere Handbook (Humanitarian Charter and
  Minimum Standards in Humanitarian
  Response, edition cited in the operator's
  governance declaration)
- ISO 19115-1:2014 / 19115-2:2019 (geographic
  information metadata) and ISO 19139-1:2019
  (XML implementation)
- ISO 6709:2008 (geographic point location)
- ISO 3166-1 (country codes), ISO 3166-2
  (subdivision codes), ISO 639 (language
  codes), ISO 4217 (currency codes), ISO 5218
  (sex code)
- ISO 8601 (date and time), IETF RFC 8259
  (JSON), RFC 4122 (UUID)
- W3C Verifiable Credentials Data Model v2.0
  (cited where the operator binds the
  registered person's identity to a verifiable
  credential)
- W3C Open Digital Rights Language (ODRL) 2.2
  (cited for the rights expression carried by
  the case envelope)
- ISO/IEC 27001:2022 and ISO/IEC 27018:2019
  (PII protection in public clouds)
- IETF RFC 6234 (SHA-256), RFC 8032 (Ed25519)
- EU Regulation (EU) 2016/679 (GDPR), Articles
  6, 9 (special categories of personal data —
  including health, biometric, ethnicity, and
  religious-belief data carried by the case
  envelope), 12-22, 46-49 (cross-border
  transfer), 89 (archiving in the public
  interest)
- EU Council Directive 2003/86/EC (right to
  family reunification)
- KR 출입국관리법 (Immigration Control Act)
  and KR 난민법 (Refugee Act)

---

## §1 Scope

This PHASE defines persistent shapes for the
artefacts exchanged when a humanitarian
operator registers a separated person, opens a
tracing request from a named relative, anchors
a positive identification between two registered
persons, transfers the case file to a partner
agency, integrates the case into an inter-
agency coordination dashboard under HXL, and
publishes the per-case outcome under privacy-
preserving safeguards. Implementations covered
include:

- A national Red Cross or Red Crescent society
  running an RFL service that registers
  refugees, internally displaced persons,
  separated migrants, missing persons in armed
  conflict, and disaster-separated families.
- A UNHCR field office operating a ProGres v4
  registration during a refugee influx.
- An IOM field office running a DTM operation
  during a population displacement.
- A child-protection authority running a
  separated-or-unaccompanied-minor case file
  under the 1989 CRC framework.
- An inter-country adoption central authority
  under the 1993 Hague Convention.
- A war-graves commission registering identified
  remains and reconciling them with named-
  relatives' tracing requests.
- A national Ministry of the Interior operating
  a missing-persons register coordinating with
  international counterparts under Interpol's
  Yellow Notice mechanism.

The RFL case file, the UNHCR ProGres
registration, the IOM DTM site assessment, the
Hague-1993 inter-country adoption record, and
the war-graves identification record receive
distinct encodings in this PHASE; the
additional safeguards required by each
humanitarian regime are encoded in PHASE-3 §3.

## §2 Programme Identifier

```
programmeId          : string (uuidv7)
operatorName         : string (legal name of the
                       operator — national Red
                       Cross / Red Crescent
                       society, UNHCR office, IOM
                       office, Ministry of the
                       Interior, child-protection
                       authority, inter-country
                       adoption central authority,
                       or war-graves commission)
operatorRole         : enum ("rcrc-national-
                       society" | "unhcr-field-
                       office" | "iom-field-
                       office" | "moi-missing-
                       persons-register" |
                       "child-protection-
                       authority" | "icca-1993" |
                       "war-graves-commission" |
                       "inter-agency-coordination"
                       | "user-defined")
operatorJurisdiction : array of string (ISO
                       3166-1 country codes)
governingFrameworks  : array of enum ("GENEVA-
                       CONVENTIONS-1949-AP" |
                       "REFUGEE-CONVENTION-1951" |
                       "CRC-1989" |
                       "HAGUE-1993-ICA" |
                       "HAGUE-1980-ICCA" |
                       "VCCR-1963-ART-36" |
                       "STATELESSNESS-1961" |
                       "ICRC-RFL-STRATEGY" |
                       "ICRC-PSP" |
                       "UNHCR-PROGRES-V4" |
                       "UNHCR-PERSONAL-DATA-
                       2015" |
                       "IOM-DTM-METHOD" |
                       "IOM-DATA-PROTECTION" |
                       "HXL-STANDARD" |
                       "SPHERE-HANDBOOK" |
                       "ISO-19115" |
                       "ISO-27001" |
                       "ISO-27018" |
                       "EU-GDPR" |
                       "EU-FAMILY-REUNION-2003-
                       86" |
                       "KR-출입국관리법" |
                       "KR-난민법" |
                       "user-defined")
icrcCoordinationRef  : string (the operator's
                       ICRC-coordinated case-
                       file network reference,
                       where applicable)
programmeStatus      : enum ("design" |
                       "operating" | "limited-
                       rollout" | "wind-down" |
                       "archived")
```

## §3 Person Registration Record

```
personRecord:
  personId           : string (uuidv7; the
                       operator's internal
                       person-record identifier)
  registrationContext : enum ("rfl-self-
                       registration" |
                       "unhcr-progres-individual"
                       | "iom-dtm-individual" |
                       "missing-persons-tracing-
                       requested-by-relative" |
                       "icca-1993-adoption" |
                       "war-graves-remains-
                       identification" |
                       "user-defined")
  identifierBindings : array of object (per-
                       authority identifier —
                       the UNHCR ProGres
                       individual identifier,
                       the IOM DTM individual
                       identifier, the operator's
                       case file number, the
                       Interpol Yellow Notice
                       identifier where
                       applicable; each carrying
                       the issuing authority and
                       the scope of use)
  givenName          : array of string
  familyName         : string
  alternativeNames   : array of string (per the
                       person's declared
                       linguistic context)
  dateOfBirth        : object (per ISO 8601;
                       partial-date support
                       where the person's
                       documentation does not
                       carry a full date)
  placeOfBirth       : object (ISO 3166-1
                       country code, ISO 3166-2
                       subdivision code, declared
                       locality)
  sexCode            : enum (per ISO 5218 —
                       0 not known, 1 male, 2
                       female, 9 not applicable)
  nationalityCodes   : array of string (ISO
                       3166-1; carrying the
                       statelessness flag where
                       applicable per the 1961
                       Convention)
  spokenLanguages    : array of string (BCP 47
                       language tags)
  protectionStatus   : enum ("refugee-1951" |
                       "asylum-seeker" |
                       "stateless-1961" |
                       "internally-displaced" |
                       "separated-migrant" |
                       "missing-conflict-
                       related" | "missing-
                       disaster-related" |
                       "separated-minor-
                       unaccompanied" |
                       "user-defined")
  vulnerabilityFlags : array of enum
                       ("unaccompanied-minor" |
                       "separated-minor" |
                       "elderly-without-care" |
                       "person-with-disability" |
                       "survivor-of-violence" |
                       "torture-survivor" |
                       "single-parent-with-
                       dependants" | "user-
                       defined")
  consentDirective   : object (the GDPR Article
                       9(2) processing basis,
                       the UNHCR Personal Data
                       Policy purpose
                       declaration, and the per-
                       channel sharing consent
                       envelope)
```

## §4 Tracing Request Record

```
tracingRequest:
  tracingId          : string (uuidv7)
  requesterRef       : string (the requesting
                       person's PHASE-1 §3
                       reference, where
                       registered, or the
                       requester's declared
                       identity envelope where
                       not yet registered)
  soughtPersonRef    : string (a partial PHASE-1
                       §3 reference for the
                       sought person, or a
                       declared identity-
                       descriptor envelope
                       where no record is yet
                       known)
  relationshipDeclared : enum ("parent-child" |
                       "sibling" | "spouse-or-
                       partner" | "extended-
                       family" | "user-defined")
  separationContext  : object (the separation
                       date, the separation
                       location, the
                       circumstances declared by
                       the requester)
  tracingChannels    : array of enum ("rcrc-
                       network" | "unhcr-
                       protection-network" |
                       "moi-missing-persons-
                       register" | "icrc-online-
                       trace" | "interpol-yellow-
                       notice" | "user-defined")
  rfStatus           : enum ("open" | "positive-
                       identification" |
                       "presumed-deceased" |
                       "closed-no-result" |
                       "closed-by-requester" |
                       "user-defined")
```

## §5 Identification Anchor Record

```
identificationAnchor:
  anchorId           : string (uuidv7)
  partyAref          : string (PHASE-1 §3
                       reference for one party
                       to the identification)
  partyBref          : string (PHASE-1 §3
                       reference for the other
                       party)
  matchEvidence      : object (the documented
                       evidence — biometric
                       match outcome under the
                       operator's biometric
                       policy, photographic
                       cross-match, narrative
                       cross-match,
                       documentary cross-match,
                       relative-confirmation)
  attestation        : object (the operator's
                       legal-entity attestation
                       and the supervising
                       protection-officer
                       reference under ICRC PSP)
```

## §6 Inter-Agency Transfer Record

```
transferRecord:
  transferId         : string (uuidv7)
  caseRef            : string (the PHASE-1 §3
                       case reference)
  fromOperator       : string (the operator's
                       legal-entity reference)
  toOperator         : string (the receiving
                       operator's legal-entity
                       reference)
  legalBasis         : object (the GDPR Article
                       46-49 transfer basis, the
                       operator's data-sharing
                       agreement reference, the
                       receiving authority's
                       reception arrangement)
  consentRef         : string (the per-case
                       consent declaration)
```

## §7 HXL Coordination Record

```
hxlRecord:
  hxlId              : string (uuidv7)
  coordinationContext : enum ("ocha-3w" |
                       "iasc-cluster" |
                       "iom-dtm-site" |
                       "unhcr-pip" | "user-
                       defined")
  hxlTagVersion      : enum ("HXL-1-1" | "user-
                       defined")
  datasetUri         : string (URI of the
                       per-coordination dataset
                       carrying HXL-tagged rows)
  refreshFrequency   : enum ("realtime" |
                       "daily" | "weekly" |
                       "monthly" | "user-
                       defined")
```

## §8 Chain-of-Custody Record

```
custodyRecord:
  custodyId          : string (uuidv7)
  artefactRef        : string (the person,
                       tracing, anchor, transfer,
                       or HXL record identifier)
  custodyEvent       : enum ("registered" |
                       "tracing-opened" |
                       "tracing-closed" |
                       "identification-anchored"
                       | "transferred" |
                       "case-closed" |
                       "redaction-applied" |
                       "withdrawn" | "user-
                       defined")
  eventTimestamp     : string (ISO 8601 date-
                       time)
  performingParty    : string (legal entity)
  hashOfArtefacts    : string (SHA-256 hex
                       digest per RFC 6234)
```

## §9 Manifest

Implementations publish a signed manifest
carrying `standardSlug` (constant value
"family-reunion-data"), `version`,
`implementation`, the operator's
`icrcCoordinationRef` envelope, and the
`profile` declaration that selects which of
the optional records (tracing, anchor,
transfer, HXL) the implementation supports.
The manifest is signed using a key whose
public part is published on the operator's
`.well-known/wia/family-reunion-data/`
discovery endpoint declared in PHASE-2.
