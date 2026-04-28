# WIA-lms PHASE 1 — Data Format Specification

**Standard:** WIA-lms
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable

This PHASE defines the canonical data format for
WIA-lms, the cross-platform learning management
system interoperability standard. The records bind
every course, enrolment, gradebook entry, content
package, assessment, and credential to a documented
LTI / Caliper / QTI / OneRoster contract so that
institutions, vendors, and credential consumers can
move between platforms without losing context, audit
trails, or learner outcomes.

References (CITATION-POLICY ALLOW only):
- IMS LTI 1.3 / Advantage (Deep Linking 2.0, NRPS 2.0, AGS 2.0)
- IMS Common Cartridge 1.3 / Thin Common Cartridge
- IMS QTI 3.0 (Question and Test Interoperability)
- IMS Caliper Analytics 1.2
- IMS OneRoster 1.2 (REST and CSV bindings)
- IMS Open Badges 3.0
- IEEE 1484.12.1 (LOM — Learning Object Metadata)
- IEEE 1484.20.1 (xAPI 1.0.3)
- IEEE 1484.11.2 (CMI Data Model)
- ADL SCORM 2004 4th Edition (legacy interop)
- AICC HACP / cmi5 (legacy / xAPI bridge)
- ISO/IEC 12785-1..-3 (Content packaging)
- ISO/IEC 19778-1..-3 (Collaborative technology — collaborative workplace)
- ISO/IEC 23988 (IT-supported assessment delivery)
- W3C Verifiable Credentials Data Model 2.0
- W3C Decentralized Identifiers 1.0
- W3C WCAG 2.2, ISO 14289-1 PDF/UA
- IETF RFC 8259 (JSON), RFC 8785 (JCS), RFC 7515 (JWS), RFC 4122 (UUID)
- ISO/IEC 27001:2022, ISO/IEC 27701:2019

---

## §1 Scope

This PHASE applies to records that describe a
course, an enrolment, a content asset, an assessment,
a gradebook entry, a credential, an instructor or
admin actor, and an institutional configuration in
an LMS deployment. It covers institutional LMSs (HE,
K–12), corporate training LMSs, and sovereign
education-ministry LMSs.

In scope: institution record, course record,
enrolment record, content-package record, activity
record, assessment record, gradebook record,
credential record, integration record (LTI tools),
and the cross-references binding each record to its
authoritative LTI / Caliper / QTI / OneRoster
artefact.

Out of scope: pedagogical curriculum design (handled
by sovereign education ministries and provider
academic governance); learner identity verification
at high-stakes assessment (handled by sovereign
testing authorities).

## §2 Institution record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `institutionRef`     | UUID (RFC 4122) opaque                          |
| `lei`                | ISO 17442 LEI when applicable                   |
| `name`               | localised name (BCP 47 keys)                    |
| `country`            | ISO 3166-1 alpha-2                              |
| `subdivision`        | ISO 3166-2                                      |
| `kind`               | `K-12`, `higher-education`, `corporate`,        |
|                      | `government`, `non-profit`                      |
| `accreditationRef[]` | URIs to accreditation evidence                  |
| `defaultLocale`      | BCP 47 tag                                      |
| `signingKeyRef`      | JWKS URL                                        |

## §3 Course record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `courseRef`          | URI                                             |
| `institutionRef`     | this PHASE §2                                   |
| `sourcedId`          | OneRoster `sourcedId`                            |
| `term`               | OneRoster academic session reference            |
| `subjectArea`        | LOM `discipline` URI                            |
| `level`              | learner level (K–12 grade, HE year)             |
| `credits`            | credit hours per institutional convention       |
| `lomRef`             | IEEE 1484.12.1 LOM record                       |
| `cartridgeRef`       | optional Common Cartridge 1.3 import URI         |
| `instructorRefs[]`   | actor references                                |

## §4 Enrolment record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `enrolmentRef`       | UUID                                            |
| `learnerRef`         | actor reference                                 |
| `courseRef`          | this PHASE §3                                   |
| `role`               | OneRoster role (`student`, `teacher`,           |
|                      | `aide`, `parent`, `guardian`, `relative`,       |
|                      | `administrator`, `proctor`)                     |
| `state`              | `active`, `inactive`, `completed`, `withdrawn`  |
| `beginDate`          | ISO 8601                                        |
| `endDate`            | ISO 8601                                        |
| `accommodationRef[]` | accommodation record references                 |

## §5 Content-package record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `contentRef`         | URI                                             |
| `format`             | `common-cartridge-1.3`, `scorm-2004-4ed`,       |
|                      | `cmi5`, `epub3`, `qti-3.0`, `web-bundle`,       |
|                      | `lti-deep-linking-2.0`                          |
| `packagingRef`       | ISO/IEC 12785 manifest URI when applicable      |
| `lomRef`             | IEEE 1484.12.1 LOM record                       |
| `digestRef`          | SHA-512 of the canonical archive                |
| `signatureRef`       | URI to detached JWS                             |
| `accessibility`      | WCAG 2.2 AA / AAA conformance evidence URI      |

## §6 Activity record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `activityRef`        | URI                                             |
| `courseRef`          | this PHASE §3                                   |
| `kind`               | `lti-launch`, `assessment`, `discussion`,       |
|                      | `assignment`, `media`, `external-resource`      |
| `ltiTool`            | optional LTI tool reference                     |
| `dueDate`            | ISO 8601                                        |
| `points`             | maximum points (numeric)                        |
| `caliperEntity`      | IMS Caliper Entity URI                          |
| `xapiActivity`       | xAPI Activity URI                               |

## §7 Assessment record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `assessmentRef`      | UUID                                            |
| `qtiTestRef`         | IMS QTI 3.0 test reference                      |
| `delivery`           | `proctored`, `remote-proctored`, `self`,        |
|                      | `automated-only`                                |
| `attempts`           | maximum attempts permitted                      |
| `timeLimit`          | ISO 8601 duration                               |
| `accommodationRefs[]`| accommodation references                        |

## §8 Gradebook record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `gradebookRef`       | UUID                                            |
| `courseRef`          | this PHASE §3                                   |
| `learnerRef`         | actor reference                                 |
| `activityRef`        | this PHASE §6                                   |
| `score`              | numeric                                         |
| `maxScore`           | numeric                                         |
| `gradeLabel`         | letter / pass-fail / mastery scale value        |
| `submittedAt`        | ISO 8601                                        |
| `gradedAt`           | ISO 8601                                        |
| `graderRef`          | actor reference                                 |
| `agsLineItemRef`     | LTI Advantage AGS line-item reference           |

## §9 Credential record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `credentialRef`      | URI per W3C VC 2.0                              |
| `learnerRef`         | actor reference                                 |
| `issuerRef`          | DID per W3C DID 1.0                             |
| `credentialSchema`   | JSON Schema URI                                 |
| `awardedAt`          | ISO 8601                                        |
| `evidenceRef[]`      | gradebook / assessment record references        |
| `proof`              | W3C Data Integrity proof                        |

## §10 Integration record (LTI tools)

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `toolRef`            | URI                                             |
| `clientId`           | OAuth 2.0 client identifier                     |
| `loginInitiationUrl` | LTI 1.3 OIDC initiation URL                     |
| `targetLinkUri`      | LTI 1.3 launch target                           |
| `keySetUrl`          | tool's JWKS URL                                 |
| `services[]`         | enabled Advantage services (`deep-linking`,     |
|                      | `nrps`, `ags`, `proctoring`)                    |
| `caliperSensorRef`   | Caliper sensor identity if the tool emits       |

## §11 Cross-domain references (informative)

- WIA-language-learning — language proficiency LTI
- WIA-learning-analytics — Caliper / xAPI sources
- WIA-prompts — embedded LLM tutoring activities
- WIA-multiverse-interface — XR learning activities

## Annex A — Conformance disclosure

Implementations declare the IMS / IEEE / ADL versions
they support, the Caliper Profile set, the LOM
application profile, and the JWS key set used to sign
content packages and credentials.

## Annex B — Worked enrolment record (informative)

```json
{
  "enrolmentRef": "f63f4f04-9a76-4d2c-9f53-2c4d09a1bbb1",
  "learnerRef": "actor-072",
  "courseRef": "https://lms.example.org/v1/courses/CS-101-2026",
  "role": "student",
  "state": "active",
  "beginDate": "2026-03-02",
  "endDate": "2026-06-15"
}
```

## Annex C — Versioning

Field additions are minor; field removals or
semantic redefinition require a major bump
synchronised with the corresponding IMS / IEEE /
ADL major release.

## Annex D — Conformance level

Conformance is "Core" (institution + course +
enrolment + content + activity + gradebook) or
"Full" (adds assessment, credential, and integration
records).

## Annex E — Privacy

Personal data flows through the institution's privacy
regime (FERPA, GDPR, K-PIPA, COPPA). Pseudonymised
learner records carry re-identification keys held by
the institution.

## Annex F — Accessibility

Content packages MUST declare WCAG 2.2 AA conformance
evidence. PDF artefacts conform to ISO 14289-1
PDF/UA-1.

## Annex G — Legacy SCORM and AICC

Legacy SCORM 2004 4th Edition and AICC HACP packages
are accepted at ingest; the registry transcodes their
launch sequence to LTI 1.3 / cmi5 for analytics
continuity. The original package signature is
preserved for audit reproducibility.

## Annex H — Term and academic session record

OneRoster academic sessions (terms, semesters,
trimesters, school years) are catalogued as:

| Field            | Source / Binding                            |
|------------------|---------------------------------------------|
| `sessionRef`     | UUID                                        |
| `sourcedId`      | OneRoster `sourcedId`                       |
| `kind`           | `term`, `semester`, `trimester`,            |
|                  | `gradingPeriod`, `schoolYear`               |
| `title`          | localised name                              |
| `startDate`      | ISO 8601                                    |
| `endDate`        | ISO 8601                                    |
| `parentSession`  | optional parent session reference           |

Term records bind courses to their academic context
and feed into gradebook posting windows.

## Annex I — Mastery transcript

Mastery-based programmes record per-competency
scores in addition to numeric grades:

| Field            | Source / Binding                            |
|------------------|---------------------------------------------|
| `competencyRef`  | URI per CASE (Competencies and Academic     |
|                  | Standards Exchange) or sovereign equivalent  |
| `mastery`        | `not-yet`, `approaching`, `meets`,          |
|                  | `exceeds`, `extending`                       |
| `evidenceRef[]`  | gradebook / activity references             |

Mastery records reference the gradebook entries
that justify the mastery determination.

## Annex J — Cohort discovery

Cohorts (named subgroups of enrolees, e.g. honours
section, ESL learners) are recorded as SHACL shapes
over the enrolment record. Cohort definitions are
versioned with Semantic Versioning 2.0.0.

弘益人間 (Hongik Ingan) — Benefit All Humanity
