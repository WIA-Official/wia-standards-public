# WIA-language-learning PHASE 1 — Data Format Specification

**Standard:** WIA-language-learning
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable

This PHASE defines the canonical data format for
WIA-language-learning, the cross-provider language
education and proficiency assessment interoperability
standard. The records bind every learner, course,
session, assessment, score, accommodation, and
credential to a documented framework, an instructor
qualification, and a provenance trail so that
downstream registries, education ministries, and
employers can reason about a learner's path without
provider lock-in.

References (CITATION-POLICY ALLOW only):
- Council of Europe CEFR (Common European Framework of Reference for Languages, Companion Volume)
- ACTFL Proficiency Guidelines (Speaking, Writing, Reading, Listening)
- ALTE Framework (Association of Language Testers in Europe)
- ICAO Doc 9835 (Language Proficiency Requirements; Levels 1–6)
- IETF BCP 47 (RFC 5646), RFC 4647
- ISO 639-1, 639-2, 639-3 (Language codes)
- ISO 15924 (Script codes)
- ISO 3166-1 alpha-2 (Country codes)
- ISO/IEC 24752 (User interface — Universal Remote Console)
- ASTM E2486-12 (Standard Practice for Distance Learning)
- IEEE 1484.12.1 (LOM — Learning Object Metadata)
- IEEE 1484.20.1 (xAPI — Experience API)
- IMS Caliper Analytics 1.2
- IMS LTI 1.3 / Advantage, IMS OneRoster 1.2, IMS QTI 3.0
- IMS Common Cartridge 1.3, IMS Open Badges 3.0
- W3C Verifiable Credentials Data Model 2.0, W3C Decentralized Identifiers 1.0
- Schema.org `EducationalOccupationalCredential`
- ISO 11669 (Translation projects — guidance on assessment alignment)
- ISO/IEC 23988 (IT-supported assessment delivery)

---

## §1 Scope

This PHASE applies to records that describe a learner's
language-learning activity and the artefacts produced
by it: enrolment, course delivery, session attendance,
formative and summative assessment, proficiency-level
attestation, accommodation, and credential issuance.
The records cover classroom, blended, and fully online
modes; receptive and productive skills; spoken, written,
and signed languages; and learner-driven self-study
with automated assessment.

In scope: learner record, course record, session
record, instructor record, assessment record, item
record (per IMS QTI 3.0), score record, accommodation
record, credential record, and the cross-references
binding each artefact to the framework level it asserts.

Out of scope: pedagogy choices and curriculum content
(handled by sovereign education ministries and provider
academic governance); learner identity verification at
high-stakes assessment (handled by sovereign testing
authorities such as ETS, Cambridge Assessment, BKAI).

## §2 Learner record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `learnerRef`         | UUID (RFC 4122) opaque                          |
| `pseudonymRef`       | provider-local code                             |
| `firstLanguage[]`    | BCP 47 tags                                     |
| `targetLanguage[]`   | BCP 47 tags learner is studying                 |
| `entryLevel`         | CEFR / ACTFL / ALTE / ICAO entry level          |
| `goalLevel`          | declared target proficiency level               |
| `accommodations[]`   | accommodation records (this PHASE §8)           |
| `birthYear`          | year only                                       |
| `consentRef`         | active consent record per applicable regime     |

`learnerRef` is the only invariant identifier; the
pseudonym is rotated by provider policy. Personal data
beyond birth year, residence ISO 3166-1 alpha-2, and
declared accommodations is not carried in the canonical
record.

## §3 Instructor record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `instructorRef`      | UUID                                            |
| `qualifications[]`   | sovereign teacher certification (CELTA, DELTA,  |
|                      | TKT, MA TESOL, KOTESOL, JTE, KOJTL),            |
|                      | university teaching credentials, ICAO ELPAC     |
|                      | rater certificate                               |
| `teachingPair[]`     | ordered pair {firstLanguage, targetLanguage}    |
| `frameworkAuthority` | declared framework the instructor rates against |
|                      | (CEFR, ACTFL, ALTE, ICAO Doc 9835)              |
| `signLanguage`       | optional ISO 639-3 sign-language code           |
| `medicalCertified`   | optional certification for medical-language     |
|                      | settings                                        |

A single instructor MAY carry multiple framework
authorities; assessments MUST declare which framework
the instructor is rating under for that assessment.

## §4 Course record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `courseRef`          | URI                                             |
| `targetLanguage`     | BCP 47 tag                                      |
| `level`              | framework level (CEFR A1..C2, ACTFL Novice..    |
|                      | Distinguished, ALTE 0..5, ICAO 1..6)            |
| `lomRef`             | IEEE 1484.12.1 LOM record                       |
| `commonCartridgeRef` | optional IMS Common Cartridge URI               |
| `qtiRef`             | optional IMS QTI 3.0 assessment URI             |
| `mode`               | `face-to-face`, `blended`, `online-sync`,       |
|                      | `online-async`, `self-study`                    |
| `volume`             | learning hours per CEFR Companion Volume        |
| `instructorRefs[]`   | course instructors                              |

## §5 Session record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `sessionRef`         | UUID                                            |
| `courseRef`          | this PHASE §4                                   |
| `learnerRef`         | this PHASE §2                                   |
| `instructorRef`      | this PHASE §3                                   |
| `startTime`          | ISO 8601 with timezone                          |
| `endTime`            | ISO 8601 with timezone                          |
| `mode`               | matches the course mode                         |
| `xapiStatement[]`    | xAPI statements emitted in-session              |
| `caliperEnvelope[]`  | IMS Caliper events emitted in-session           |
| `attendanceState`    | `present`, `late`, `absent`, `excused`          |

## §6 Assessment record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `assessmentRef`      | UUID                                            |
| `learnerRef`         | this PHASE §2                                   |
| `framework`          | CEFR / ACTFL / ALTE / ICAO                      |
| `skillProfile`       | `listening`, `reading`, `speaking`, `writing`,  |
|                      | `interaction`, `mediation` (per CEFR Companion  |
|                      | Volume); ICAO sub-skills as applicable          |
| `delivery`           | `proctored`, `remote-proctored`, `self`,        |
|                      | `automated-only`                                |
| `qtiTestRef`         | IMS QTI 3.0 test reference if computer-based    |
| `raterRefs[]`        | rater identifiers; ≥2 for high-stakes           |
| `assessmentDate`     | ISO 8601                                        |
| `accommodationRef[]` | accommodations applied                          |

## §7 Score record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `scoreRef`           | UUID                                            |
| `assessmentRef`      | this PHASE §6                                   |
| `framework`          | CEFR / ACTFL / ALTE / ICAO                      |
| `level`              | the asserted level                              |
| `subscores[]`        | per-skill levels (e.g. `listening: B2`,         |
|                      | `speaking: B1`)                                 |
| `rawScore`           | numeric raw                                     |
| `cutScoreRef`        | citation to the published cut-score table       |
| `interRaterDelta`    | for human-rated speaking/writing: max delta     |
|                      | among raters                                    |
| `aiAssistMethod`     | `none`, `automated-scoring`,                    |
|                      | `automated-scoring-with-human-review`           |
| `issuedAt`           | ISO 8601                                        |

The published cut-score table is referenced by URL so
that historical scores can be re-aligned if the table
is amended.

## §8 Accommodation record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `accommodationRef`   | UUID                                            |
| `kind`               | extra time, screen reader, sign-language        |
|                      | interpretation, scribe, separate room, large    |
|                      | print, alternate format, assistive listening    |
| `evidenceRef`        | URI to medical / educational evidence document  |
| `approvedBy`         | sovereign accommodation authority or provider   |
|                      | accommodations office                           |
| `validFromTo`        | window during which the accommodation applies   |

Accommodation records are confidential; access is
controlled by the deployment's privacy policy.

## §9 Credential record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `credentialRef`      | URI per W3C VC 2.0                              |
| `learnerRef`         | this PHASE §2                                   |
| `issuerRef`          | DID per W3C DID 1.0                             |
| `credentialSchema`   | JSON Schema URI                                 |
| `assertedLevel`      | framework level                                 |
| `validFrom`          | ISO 8601                                        |
| `validUntil`         | optional ISO 8601                               |
| `evidenceRef[]`      | assessment / score record references            |
| `proof`              | W3C Data Integrity proof                        |

Credentials follow Open Badges 3.0 when issued at
informal levels and Verifiable Credentials 2.0 when
issued by sovereign authorities.

## §10 Cross-domain references (informative)

- WIA-learning-analytics — Caliper / xAPI dashboards
- WIA-lms — LTI hand-off into a learning platform
- WIA-language-bridge — interpretation in classroom
- WIA-prompts — prompt-mediated tutoring artefacts

## Annex A — Conformance disclosure

Implementations declare the framework set they support,
the LOM/QTI versions in use, and the JWS key set used
to sign assessment, score, and credential records.

## Annex B — Framework alignment table (informative)

| CEFR | ACTFL                        | ALTE | ICAO |
|------|------------------------------|------|------|
| A1   | Novice Mid                   | 0    | 1    |
| A2   | Novice High / Intermediate Low | 1  | 2    |
| B1   | Intermediate Mid             | 2    | 3    |
| B2   | Intermediate High / Advanced Low | 3 | 4    |
| C1   | Advanced Mid / Advanced High | 4    | 5    |
| C2   | Superior / Distinguished     | 5    | 6    |

The alignment is informative; high-stakes assessments
MUST cite the framework's own scoring guide.

## Annex C — Worked score record (informative)

```json
{
  "scoreRef": "f63f4f04-9a76-4d2c-9f53-2c4d09a1bbb1",
  "assessmentRef": "asm-2026-04-28-001",
  "framework": "CEFR",
  "level": "B2",
  "subscores": [
    {"skill": "listening", "level": "B2"},
    {"skill": "reading",   "level": "C1"},
    {"skill": "speaking",  "level": "B1"},
    {"skill": "writing",   "level": "B2"}
  ],
  "issuedAt": "2026-04-28T11:32:00+09:00"
}
```

## Annex D — Versioning

This PHASE follows the WIA governance procedure. Field
additions are minor; field removals or semantic
redefinition require a major bump synchronised with
Council of Europe Companion Volume revisions.

## Annex E — Conformance level

Conformance is "Core" (learner + course + session +
assessment + score) or "Full" (adds accommodation and
credential records signed under W3C VC 2.0).

## Annex F — Sign-language proficiency

Sign-language learning records reference an ISO 639-3
sign code. Frameworks for sign-language proficiency
(SL-CEFR, ASLPI levels, RID-NIC-equivalent) are
recorded under `framework` with the framework's
authoritative URL.

## Annex G — Privacy and consent

Personal data of minors is processed under the
applicable child-online-protection regime (COPPA,
K-PIPA Article 22-2, GDPR Art. 8). Provider deployments
declare the regime at registration. Score records that
identify a minor MUST be transmitted only to the
identified parent or guardian principal.

## Annex H — Inter-rater reliability targets

| Stakes        | Min raters | Target inter-rater agreement (κ)   |
|---------------|------------|------------------------------------|
| Low (formative)   | 1     | n/a                                |
| Medium            | 2     | κ ≥ 0.6                            |
| High (sovereign)  | 3     | κ ≥ 0.8 with discrepancy resolution|

弘益人間 (Hongik Ingan) — Benefit All Humanity
