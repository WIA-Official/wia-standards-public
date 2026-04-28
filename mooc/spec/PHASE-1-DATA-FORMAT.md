# WIA-mooc PHASE 1 — Data Format Specification

**Standard:** WIA-mooc
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable

This PHASE defines the canonical data format for
massive-open-online-course (MOOC) operations covering
course / cohort / module / lesson / activity records,
learner records, learning-event capture, assessment
items and attempts, peer-assessment artefacts,
discussion records, video / interactive content
manifests, and progress / completion records. The
format aligns with 1EdTech (formerly IMS Global)
Caliper Analytics, ADL xAPI, IMS LTI 1.3, IMS Common
Cartridge, and the W3C accessibility specifications so
content authored once exchanges across MOOC platforms,
learning-record stores, and downstream credentialing.

References (CITATION-POLICY ALLOW only):
- 1EdTech Caliper Analytics 1.2 — learning-event payload
- ADL xAPI 2.0 — Tin Can / Experience API
- 1EdTech LTI 1.3 — Learning Tools Interoperability
- 1EdTech LTI Advantage — Names and Role Provisioning, Assignment and Grade Services, Deep Linking
- 1EdTech Common Cartridge 1.3 — content packaging
- 1EdTech QTI 3.0 — Question and Test Interoperability
- 1EdTech CASE 1.1 — Competency and Academic Standards Exchange
- ISO/IEC 19796-1 — IT for learning, education, and training: quality management
- W3C WCAG 2.2; EN 301 549 v3.2.1; Section 508 (US)
- W3C Web Annotation Data Model (where annotations bind)
- ISO/IEC 23053 — framework for AI systems (where adaptive learning binds AI)
- IETF RFC 8259 (JSON), RFC 8785 (JCS), RFC 4122 (UUID), RFC 7515 (JWS)
- HTML Living Standard; ECMAScript (ECMA-262 latest); WebVTT (W3C)
- W3C Verifiable Credentials Data Model 2.0 (where micro-credentials issue)

---

## §1 Scope

This PHASE applies to MOOC platforms that publish
courses to wide audiences, support per-cohort or
self-paced enrolment, capture learning events, run
assessments, assign credentials or course-completion
attestations, and report on learner outcomes.

In scope: course, cohort, module, lesson, activity,
content-asset, learner, enrolment, learning-event,
assessment-item, attempt, response, peer-assessment,
discussion, progress, completion, accessibility-
assertion, and credential-binding records, plus
cross-references binding course outcomes to credentials.

Out of scope: pure broadcast video without learning-
event capture (handled by streaming-media standards)
and full-degree academic transcripts spanning
multiple courses (handled by academic-credential
standards).

## §2 Course record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `courseRef`          | UUID (RFC 4122)                                 |
| `title`              | localised label (BCP 47)                        |
| `summary`            | localised text                                  |
| `language`           | BCP 47 primary delivery language                 |
| `subjectArea`        | ISCED-F 2013 code                                |
| `levelCode`          | EQF / ISCED level (where bound)                  |
| `competencyMap`      | CASE 1.1 framework reference + competency list  |
| `prerequisites[]`    | upstream `courseRef` or external requirement     |
| `pacing`             | self-paced / cohort-based / hybrid                |
| `notionalLearning`   | hours estimated                                  |
| `creditValue`        | ECTS / national equivalent (where awardable)    |
| `accessibilityRef`   | accessibility assertion record (this PHASE §11) |

## §3 Cohort record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `cohortRef`          | UUID                                            |
| `courseRef`          | §2                                              |
| `cohortLabel`        | human-readable identifier (e.g. "2026-Q2")      |
| `enrolmentOpen`      | ISO 8601                                        |
| `enrolmentClose`     | ISO 8601                                        |
| `startsAt`           | ISO 8601                                        |
| `endsAt`             | ISO 8601                                        |
| `instructorRefs[]`   | identities of facilitators                       |

## §4 Module / lesson / activity records

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `moduleRef`          | UUID                                            |
| `courseRef`          | §2                                              |
| `position`           | integer ordering                                |
| `learningOutcomes[]` | per-module learning outcomes (Bloom verbs)      |

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `lessonRef`          | UUID                                            |
| `moduleRef`          | §4                                              |
| `kind`               | `video`, `reading`, `quiz`, `interactive`,      |
|                      | `discussion`, `peer-review`, `code-lab`,        |
|                      | `live-session`                                  |
| `contentRef`         | content-asset reference (see §10)                |
| `position`           | integer                                         |

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `activityRef`        | UUID                                            |
| `lessonRef`          | §4                                              |
| `kind`               | `view`, `submit`, `respond`, `peer-grade`,      |
|                      | `attempt`, `bookmark`, `note`                   |
| `weight`             | numeric — towards course pass / completion      |

## §5 Learner record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `learnerRef`         | UUID                                            |
| `pseudonymRef`       | platform-local pseudonym                         |
| `country`            | ISO 3166-1 alpha-3 (self-declared, optional)    |
| `language`           | BCP 47 preferred language                        |
| `accessibilityNeeds` | per W3C Personalization Semantics taxonomy       |
|                      | (where the learner discloses)                   |
| `consentRef`         | data-processing consent reference                |

Personally-identifying attributes live in the
sponsor's identity vault; the learner record carries
opaque references.

## §6 Enrolment record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `enrolmentRef`       | UUID                                            |
| `learnerRef`         | §5                                              |
| `cohortRef`          | §3                                              |
| `enrolledAt`         | ISO 8601                                        |
| `track`              | `audit`, `verified`, `paid`, `scholarship`,     |
|                      | `enterprise`                                    |
| `verificationRef`    | identity-verification reference (where track    |
|                      | implies verified attestation)                    |

## §7 Learning-event record (Caliper / xAPI)

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `eventRef`           | UUID                                            |
| `actor`              | learner reference                                |
| `verb`               | Caliper action / xAPI verb (e.g. `Started`,     |
|                      | `Submitted`, `Graded`, `Resumed`, `Paused`,     |
|                      | `Bookmarked`, `Annotated`)                      |
| `object`             | activity / lesson / item reference               |
| `context`            | course / cohort / session / app references       |
| `eventTime`          | ISO 8601                                        |
| `extensions`         | per-platform extensions (versioned)              |

Learning events emit as Caliper envelopes (preferred)
or xAPI statements (where ADL xAPI is the authoritative
record store). Both forms are supported for
multilateral interoperability.

## §8 Assessment-item and attempt records

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `itemRef`            | UUID                                            |
| `kind`               | `multiple-choice`, `multi-select`, `match`,     |
|                      | `numeric`, `short-answer`, `essay`, `code`,     |
|                      | `peer-assessed`, `oral`                         |
| `qti`                | QTI 3.0 payload reference                        |
| `competencyRef[]`    | CASE 1.1 competencies covered                    |
| `difficulty`         | calibrated difficulty (per IRT, where computed) |
| `discrimination`     | calibrated discrimination                        |
| `accessibilityRef`   | item-level accessibility assertion               |

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `attemptRef`         | UUID                                            |
| `learnerRef`         | §5                                              |
| `itemRef`            | as above                                        |
| `response`           | learner response payload (QTI 3.0)               |
| `score`              | numeric (signed normalised)                      |
| `judgement`          | `correct`, `partial`, `incorrect`, `pending`    |
| `attemptTime`        | ISO 8601                                        |
| `proctoringRef`      | proctoring artefact (where applicable)           |

## §9 Peer-assessment and discussion records

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `peerAssessmentRef`  | UUID                                            |
| `submittedBy`        | learner ref                                     |
| `assessedBy`         | learner ref (anonymous to author)                |
| `rubricRef`          | rubric reference                                 |
| `score`              | per-criterion score                              |
| `narrative`          | feedback text (BCP 47)                          |

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `discussionRef`      | UUID                                            |
| `parentRef`          | thread parent                                    |
| `author`             | learner ref                                     |
| `body`               | text + media (Web Annotation Data Model)         |
| `moderationFlags`    | per moderation policy                            |

## §10 Content-asset record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `assetRef`           | UUID                                            |
| `kind`               | `video`, `audio`, `pdf`, `html`, `epub`,        |
|                      | `interactive-html`, `simulation`, `dataset`     |
| `mediaUri`           | content-addressed URI                            |
| `mimeType`           | per IANA registry                               |
| `durationSeconds`    | for video / audio                                |
| `transcriptRef`      | WebVTT / SRT / SMI (transcript / caption)        |
| `signLanguageVideo`  | for accessibility (KSL / ASL / etc track)        |
| `audioDescription`   | descriptive-audio track                          |
| `bitrateLadder`      | for adaptive streaming (HLS / DASH)              |
| `licence`            | SPDX licence identifier                          |

## §11 Accessibility-assertion record

Per WCAG 2.2 / EN 301 549 conformance:

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `accessibilityRef`   | UUID                                            |
| `wcagLevel`          | `A`, `AA`, `AAA`                                |
| `wcagCriteria`       | conformance per criterion (1.1.1 — 4.1.3)        |
| `en301549Annex`      | per Annex A items                                |
| `auditorRef`         | accessibility auditor identity                   |
| `auditDate`          | ISO 8601                                        |
| `remediationPlan`    | per-failure tracking                             |

## §12 Progress / completion record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `progressRef`        | UUID                                            |
| `learnerRef`         | §5                                              |
| `cohortRef`          | §3                                              |
| `completionPct`      | per-activity-weighted                            |
| `assessmentScore`    | course-level score                               |
| `passed`             | boolean per pass policy                          |
| `completedAt`        | ISO 8601                                        |
| `credentialRef`      | bound micro-credential issuance                  |

## §13 Cross-domain references (informative)

- WIA-micro-credential — for completion-credential issuance
- WIA-virtual-classroom — for live-session integration
- WIA-learning-analytics — for cohort-level analyses
- WIA-content-ai — for AI-generated assessment items

## Annex A — Worked Caliper event (informative)

```json
{
  "@context": "http://purl.imsglobal.org/ctx/caliper/v1p2",
  "id": "urn:uuid:7c0d9b0f-...",
  "type": "AssessmentEvent",
  "actor": {"id":"https://platform.example/users/learner-007","type":"Person"},
  "action": "Submitted",
  "object": {"id":"https://platform.example/items/quiz-101","type":"AssessmentItem"},
  "eventTime": "2026-04-12T09:14:00Z"
}
```

## Annex B — Conformance disclosure

Implementations declare the Caliper / xAPI versions
served, the LTI 1.3 services supported (NRPS / AGS /
Deep Linking), the QTI revision, the WCAG / EN 301 549
audit results, and the credential-binding profiles
(W3C VC / Open Badges 3.0).

## Annex C — Versioning

Field additions are minor; semantic redefinition is
major.
