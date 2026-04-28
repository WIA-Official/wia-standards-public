# WIA-language-bridge PHASE 1 — Data Format Specification

**Standard:** WIA-language-bridge
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable

This PHASE defines the canonical data format for
WIA-language-bridge, the cross-language interpretation
and translation interoperability standard. The records
bind every translation, interpretation session, glossary
entry, and quality measurement to a documented language
identifier, a translator or interpreter qualification,
a domain ontology, and a provenance trail so that
downstream consumers can reproduce, audit, and
re-translate any cross-language exchange.

References (CITATION-POLICY ALLOW only):
- IETF BCP 47 (Tags for Identifying Languages), RFC 5646, RFC 4647
- ISO 639-1, 639-2, 639-3 (Language codes)
- ISO 15924 (Script codes)
- ISO 3166-1 alpha-2, ISO 3166-2 (Country and subdivision codes)
- Unicode CLDR (Common Locale Data Repository, latest release)
- Unicode 15.1, UTS #35 LDML
- W3C Internationalization Tag Set (ITS) 2.0
- OASIS XLIFF 2.1 (XML Localisation Interchange File Format)
- LISA TBX-Basic (TermBase eXchange) ISO 30042:2019
- LISA TMX 1.4b (Translation Memory eXchange)
- ASTM F2575-14 (Standard Guide for Quality Assurance in Translation)
- ASTM F2089-15 (Language Interpreting Services)
- ISO 17100:2015 (Translation services), ISO 18587:2017 (Post-editing of MT)
- ISO 13611:2014 (Community interpreting), ISO 18841:2018 (Interpreting services)
- ITU-T F.745 (Multilingual conversational service)
- ICAO Annex 10 / Doc 9835 (English language proficiency requirements)
- HL7 FHIR R5 (Communication, Patient.communication, Practitioner.communication)

---

## §1 Scope

This PHASE applies to records that bind a source-language
artifact to a target-language artifact under a translation
or interpretation event. The artifact may be a written
document, an audio recording, a real-time interpreted
exchange, a sign-language relay, a Braille rendering, or
a synthetic-voice rendition produced by automatic
speech-to-text and machine translation pipelines.

In scope: language tag record, translator/interpreter
record, source segment record, target segment record,
translation memory record, glossary (TBX) record,
quality measurement record, session record (interpretation),
and the cross-references binding each segment to its
provenance and to its quality grade.

Out of scope: the natural-language understanding
algorithm itself (handled by the implementation's
internal model card), and language proficiency
certification programmes (governed by sovereign
education ministries).

## §2 Language tag record

Every artifact carries:

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `tag`                | BCP 47 well-formed tag (RFC 5646 §2)            |
| `primaryLanguage`    | ISO 639-1 if assigned, else ISO 639-3           |
| `script`             | ISO 15924 four-letter code                      |
| `region`             | ISO 3166-1 alpha-2 or UN M.49                   |
| `variant`            | BCP 47 registered variant (e.g. `1996`,         |
|                      | `valencia`)                                     |
| `extension[]`        | BCP 47 extension subtags (`u-` Unicode, `t-`    |
|                      | transformed content)                            |
| `privateUse[]`       | `x-`-prefixed subtags (auditor-scoped only)     |

Tags MUST be canonicalised per RFC 5646 §4.5 prior to
record signing. Lookup matching follows RFC 4647 (best-
fit or filtering).

## §3 Translator / interpreter record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `practitionerRef`    | UUID (RFC 4122) opaque identifier               |
| `name`               | legal or professional name                      |
| `qualifications[]`   | ISO 17100 §3.1.4 competencies (T-Q1..T-Q5),     |
|                      | ISO 18841 interpreting competencies, or         |
|                      | sovereign certification (ATA, NAATI, CIOL,      |
|                      | DipTrans, KICE, JTF, AAEC)                      |
| `workingPair[]`      | ordered pair {source-tag, target-tag}; one      |
|                      | record per directional pair                     |
| `domains[]`          | ISO 17100 specialisation tag (legal, medical,   |
|                      | technical, literary, audiovisual, software)     |
| `mtPostEditing`      | ISO 18587 PE conformance level                  |
| `signLanguage`       | optional ISO 639-3 code (asl, kvk, gss, jsl)    |
| `medicalCertified`   | optional CCHI / NBCMI / KSMI certification      |

A practitioner record MAY reference more than one
sovereign certificate; the record reproduces the
certificate identifier verbatim and does not assert
equivalence between certificates from different
authorities.

## §4 Source segment record

A segment is the smallest replayable unit of source
content.

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `segmentRef`         | UUID                                            |
| `sourceTag`          | language tag (this PHASE §2)                    |
| `text`               | UTF-8; for audio, optional WebVTT/SRT cue text  |
| `audioRef`           | optional URI to PCM/Opus recording              |
| `start`              | character offset or audio timestamp             |
| `end`                | character offset or audio timestamp             |
| `domainRef`          | domain ontology code (UNTERM, IATE, MeSH,       |
|                      | ICD-11, NACE, NAICS, CPC, GACS)                 |
| `documentRef`        | URI to the parent document or stream            |
| `itsAttributes`      | W3C ITS 2.0 data categories applied             |

Segmentation rules follow Unicode Standard Annex #29
(Text Segmentation) for written content, and Voice
Activity Detection windows for audio content.

## §5 Target segment record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `targetSegmentRef`   | UUID                                            |
| `sourceSegmentRef`   | this PHASE §4                                   |
| `targetTag`          | language tag                                    |
| `text`               | UTF-8                                           |
| `practitionerRef`    | this PHASE §3                                   |
| `productionMode`     | `human`, `human-edited-mt`, `mt-only`           |
| `mtEngineRef`        | optional URI to engine model card               |
| `confidence`         | 0..1; required for `mt-only`                    |
| `qualityRef[]`       | this PHASE §8 quality records                   |
| `glossaryHits[]`     | TBX entries enforced (this PHASE §7)            |
| `signedAt`           | ISO 8601 timestamp; required if practitioner    |
|                      | attests authorship                              |

Target segments produced under post-editing carry both
the original MT proposal and the human-edited final;
auditors compare the two for productivity studies and
for ISO 18587 conformance evidence.

## §6 Translation memory record (TMX-aligned)

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `tmxRef`             | URI                                             |
| `tu[]`               | translation unit list per TMX 1.4b              |
| `srclang`            | declared source language (TMX `srclang`)        |
| `creationtool`       | tool that produced the TM                       |
| `creationtoolversion`| tool version                                    |
| `segtype`            | `block`, `paragraph`, `sentence`, `phrase`      |
| `o-tmf`              | original TM format (e.g., XLIFF, OmegaT, Trados)|
| `adminlang`          | language of comments / metadata                 |

A TM record is signed (RFC 7515) so that downstream
consumers can verify that the TM has not been amended
since publication.

## §7 Glossary record (TBX-aligned)

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `tbxRef`             | URI                                             |
| `concept[]`          | ISO 30042:2019 concept entries                  |
| `domainRef`          | concept system reference (IATE, UNTERM, FAO,    |
|                      | WHO, MeSH, SNOMED-CT)                           |
| `language[]`         | per-language sub-entries with PoS, gender,      |
|                      | usage status (preferred, admitted, deprecated)  |
| `definitionRef`      | citation to authoritative definition            |
| `crossReference[]`   | related, broader, narrower, opposite concepts   |

Glossary records are mandatory for medical, legal, and
safety-critical domains; absence triggers a quality
warning at the §8 quality measurement.

## §8 Quality measurement record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `qualityRef`         | UUID                                            |
| `targetSegmentRef`   | this PHASE §5                                   |
| `framework`          | MQM 2.0 (Multidimensional Quality Metrics) or   |
|                      | DQF (Dynamic Quality Framework) or LISA QA      |
| `errors[]`           | error list with category (accuracy, fluency,    |
|                      | terminology, style, locale, design),            |
|                      | severity (minor, major, critical), span         |
| `score`              | normalised 0..100 per framework rules           |
| `reviewerRef`        | reviewer practitionerRef (must differ from      |
|                      | the producing practitioner)                     |
| `reviewType`         | `monolingual`, `bilingual`, `back-translation`  |
| `assessmentDate`     | ISO 8601                                        |

Quality records bind to the target segment and to the
TBX entries that should have been enforced; auditors
join these records to validate ISO 17100 production
process compliance.

## §9 Interpretation session record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `sessionRef`         | UUID                                            |
| `mode`               | `consecutive`, `simultaneous`, `whispered`,     |
|                      | `relay`, `over-the-phone`, `video-remote`,      |
|                      | `sign-language`                                 |
| `setting`            | `conference`, `legal`, `medical`, `community`,  |
|                      | `educational`, `media`, `military`              |
| `startTime`          | ISO 8601 with timezone                          |
| `endTime`            | ISO 8601 with timezone                          |
| `practitioners[]`    | practitioner references; ≥2 for ≥30 min         |
|                      | simultaneous (ISO 18841 fatigue rule)           |
| `audioRef`           | URI; presence is mandatory for legal/medical    |
| `consentRef`         | URI to consent record per applicable privacy    |
|                      | regime                                          |

ISO 18841 partner-rotation rules (typically 20–30 min)
are enforced by binding the session to a rotation
schedule.

## §10 Cross-domain references (informative)

- WIA-js — for ECMA-402 locale negotiation in clients
- WIA-language-learning — for proficiency assessment
- WIA-pubscript — for publication-time localisation
- WIA-prompts — for LLM prompt translation provenance

## Annex A — Conformance disclosure

Implementations declare the JSON-Schema URIs they
support, the canonicalisation form (RFC 8785), and the
JWS key set used to sign segment, TM, and glossary
records.

## Annex B — BCP 47 canonicalisation examples

| Input               | Canonical                                       |
|---------------------|-------------------------------------------------|
| `iw`                | `he`                                            |
| `zh-Hans-CN`        | `zh-Hans-CN`                                    |
| `EN-us`             | `en-US`                                         |
| `ja-Jpan-JP`        | `ja-JP` (ISO 15924 default suppressed)          |
| `ko-Kore-KR`        | `ko-KR` (ISO 15924 default suppressed)          |

## Annex C — Worked target segment record (informative)

```json
{
  "targetSegmentRef": "f63f4f04-9a76-4d2c-9f53-2c4d09a1bbb1",
  "sourceSegmentRef": "src-1024",
  "targetTag": "ko-KR",
  "text": "환자는 수술 후 4일째에 퇴원하였다.",
  "practitionerRef": "ATA-12345",
  "productionMode": "human-edited-mt",
  "qualityRef": ["mqm-2026-04-28-001"]
}
```

## Annex D — Versioning

This PHASE follows the WIA governance procedure. Field
additions are minor; field removals or semantic
redefinition require a major bump. BCP 47 subtag
registry updates are tracked editorially.

## Annex E — Conformance level

Conformance is "Core" (language tag + practitioner +
source segment + target segment + quality) or "Full"
(adds TM, TBX, and interpretation session records).

## Annex F — Sign-language carriage

Sign-language target segments reference an ISO 639-3
sign-language code (`asl`, `kvk`, `gss`, `jsl`,
`bzs`, etc.) and a video reference whose codec MUST be
declared in the segment record. Sign-language
practitioners are credentialed against RID, NIA, KSLI,
or sovereign-equivalent registers.

## Annex G — Privacy and consent

Translation and interpretation in healthcare, legal, or
asylum settings require an active consent record under
the applicable privacy regime (HIPAA, GDPR, K-PIPA,
LGPD). Audio retention windows MUST be declared at
session start; the default for medical interpretation
is 6 years.

## Annex H — Quality framework binding

| Framework | Use                                                |
|-----------|----------------------------------------------------|
| MQM 2.0   | default; covers 7 dimensions                       |
| DQF       | TAUS-aligned; productivity studies                 |
| LISA QA   | legacy; reproduced for archival traceability       |

弘益人間 (Hongik Ingan) — Benefit All Humanity
