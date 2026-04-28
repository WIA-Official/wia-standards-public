# WIA-mooc PHASE 3 — Protocol Specification

**Standard:** WIA-mooc
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable

This PHASE defines the operational protocols binding
records and API resources into auditable lifecycles:
course / cohort lifecycle, content-publication and
versioning, enrolment lifecycle, learning-event
ingestion contract, assessment-attempt governance with
proctoring intersection, peer-assessment workflow,
discussion moderation, accessibility-assertion
publication, completion and credential issuance, and
the audit-event chain. The protocols are framed so an
accreditation body, a sponsor's QA cycle, or a
regulator examining adaptive-learning AI can reconstruct
any cohort from the event log.

References (CITATION-POLICY ALLOW only):
- 1EdTech Caliper Analytics 1.2; ADL xAPI 2.0
- 1EdTech LTI 1.3 / LTI Advantage; QTI 3.0
- ISO/IEC 19796-1 — Quality management for learning, education, and training
- ISO 21001 — educational organisations management system
- ENQA ESG 2015 — Standards and Guidelines for Quality Assurance
- W3C WCAG 2.2; EN 301 549 v3.2.1
- ISO/IEC 23053 — framework for AI systems (where adaptive learning binds AI)
- ISO/IEC 27037 — digital evidence preservation
- IETF RFC 5424 (Syslog), RFC 7515 (JWS), RFC 8785 (JCS)

---

## §1 Course / cohort lifecycle

```
course: drafted → reviewed → published → versioned → archived
                                  │
                                  └→ retired

cohort: opened → in-flight → completed → archived
                       │
                       └→ extended (per policy)
```

Course versioning is mandatory on outcome / assessment
changes; cohorts that started under v1.0 complete on
v1.0 unless the change is a remediation that the
sponsor declares retroactive.

## §2 Content-publication and versioning

| Asset kind         | Versioning protocol                              |
|--------------------|--------------------------------------------------|
| Video              | re-encode to bitrate ladder; manifest digest pins |
| Reading            | EPUB / HTML version; content-addressed URI        |
| Quiz / item        | QTI 3.0 payload version + calibration version    |
| Interactive        | content-addressed bundle; version locked          |
| Live session       | recording asset + transcript + sign-language      |

## §3 Enrolment lifecycle

```
applied → enrolled → active → completed
              │           │
              └→ withdrawn │
                          └→ expired (cohort-end without completion)
```

Withdrawal is logical: the enrolment marks `withdrawn`
with the reason; learning-events to date are retained
unless the learner exercises an erasure right.

## §4 Learning-event ingestion contract

| Concern                | Contract                                       |
|------------------------|------------------------------------------------|
| Ordering               | event-sourced; out-of-order tolerated           |
| Idempotency            | `Idempotency-Key` per request                  |
| Schema evolution       | per-version envelope                            |
| Privacy                | learner reference is opaque; PII separate       |
| Throttling             | per-implementation rate limit + retry header    |

Caliper and xAPI envelopes are accepted concurrently;
the implementation maps to one canonical store so
analytics consume a single shape.

## §5 Assessment-attempt governance

```
launched → in-progress → submitted → scored → reviewed
                                │           │
                                └→ aborted   └→ contested → reviewed
                                              (rubric panel)
```

Proctoring integration:

| Proctoring tier        | Trigger                                       |
|------------------------|-----------------------------------------------|
| Honor-code only        | low-stakes attempt                             |
| AI-assisted proctoring | medium-stakes attempt                          |
| Human-proctor live     | high-stakes / verified attempt                 |
| In-person centre       | regulator-required                             |

A proctoring-required attempt that opens without an
active proctoring binding rejects with `403`.

## §6 Peer-assessment workflow

```
submission-window-open → submission-locked → peer-grade-window-open
                                                        │
                                                        └→ aggregated → returned
```

The protocol enforces a minimum number of peer grades
per submission (typically 3-5 per platform policy);
calibration submissions seed inter-rater reliability
(Kendall's W or Krippendorff's α).

## §7 Discussion moderation

```
posted → published / hidden / removed → restored (appeal)
```

Moderation policy aligns with the platform's code of
conduct; community-flag actions queue stewardship
tasks. Moderation events emit audit entries so an
appeal review can reconstruct the decision.

## §8 Accessibility-assertion publication

Publications follow:

```
audit-scheduled → audit-conducted → assertion-drafted →
  remediation (where applicable) → assertion-published
```

The assertion publishes to the course public pages
and to platform-wide search filters. An assertion at
WCAG 2.2 Level AA is the minimum publishable level
for a course distributed in the EU per EN 301 549.

## §9 Completion and credential issuance

```
completion-pending → completion-validated → credential-issued
```

Completion validation runs the course pass policy
(weighted activities + assessments + proctoring
gates). Issuance binds to WIA-micro-credential PHASE
1 §5; the issuance event extends the audit chain.

## §10 Adaptive-learning governance (AI-bound)

When the platform applies adaptive-learning algorithms
the protocol records:

- model identity (semantic version + container digest)
- per-learner adaptation events (path divergence,
  difficulty adjustment, hint provision)
- bias monitoring (demographic parity in adaptation
  paths)
- learner consent for adaptive treatment
- right to opt-out

Adaptive recommendations sign with the model
attestation key so an audit can trace which
recommendation informed which event.

## §11 Audit event chain

| Field          | Meaning                                                 |
|----------------|---------------------------------------------------------|
| `eventId`      | UUID                                                    |
| `eventTime`    | ISO 8601 with timezone                                  |
| `actor`        | identity (learner / instructor / platform / proctor)    |
| `resourceRef`  | URI of the resource that changed                        |
| `action`       | created / submitted / scored / completed / issued       |
| `priorHash`    | SHA-256 of the prior event payload                      |
| `signature`    | RFC 7515 JWS over the canonical event payload (RFC 8785)|

## §12 Reproducibility

A cohort outcome is `reproducible-strong` when the
course version, content-asset digests, item
calibrations, peer-rubric versions, adaptive-model
versions, and pass-policy version are all content-
addressed; `weak` when any is absent.

## §13 Privacy and learner rights

| Right                | Action                                          |
|----------------------|-------------------------------------------------|
| Access               | export of learning events + assessments         |
| Rectification        | correct identity / preference attributes         |
| Erasure              | tombstone learning-event payload; keep audit    |
|                      | hash; aggregated metrics retained                |
| Portability          | xAPI / Caliper export to learner-chosen LRS     |
| Restriction          | flag prevents downstream processing             |
| Opt-out (adaptive)   | adaptation disabled; baseline path delivered    |

Right events emit dedicated audit entries.

## Annex A — Worked completion example (informative)

A cohort runs a 6-week "Statistics 101" MOOC. The pass
policy requires ≥ 70 % weighted score across weekly
quizzes (50 %), a peer-graded project (30 %), and a
proctored final exam (20 %). Learner L-007's quiz
average is 86 %, peer-grade average is 78 %, and final
exam (live human-proctored) is 74 %. Weighted total is
80 %. The pass record emits; WIA-micro-credential
issues a Statistics 101 micro-credential bound to the
learner's DID.

## Annex B — Conformance disclosure

Implementations declare the audit-chain schema
version, the JWS algorithm registry, the Caliper /
xAPI revisions accepted, the LTI 1.3 services, the
WCAG / EN 301 549 audit results, and the adaptive-
learning model registry version.

## Annex C — Versioning

Field additions are minor; semantic redefinition is
major.

## Annex D — Quality-assurance cycle

Per ISO/IEC 19796-1 the QA cycle covers:

| Cycle phase           | Cadence                                       |
|-----------------------|-----------------------------------------------|
| Course design review  | per major version                              |
| Item bank review      | annual                                        |
| Inter-rater calibration| each cohort start                              |
| Accessibility audit   | per major version + annual re-audit            |
| Learning-outcome audit| per cohort end                                 |

QA artefacts (review minutes, item statistics, rubric
calibration) feed the cohort's audit chain.

## Annex E — Time-source declaration

Audit-chain timestamps cite the time-source authority
(NTP stratum-1, NIST, KASI, KRISS, PTB).

## Annex F — Item-bank fairness sampling

Per ISO/IEC 19796-1 the implementation samples item
performance across protected demographic slices to
detect differential item functioning (DIF). Items
with significant DIF are flagged for review and
either revised or retired.

| Coefficient                | Acceptable threshold        |
|----------------------------|------------------------------|
| Mantel-Haenszel χ²         | non-significant per α        |
| Standardised difference    | within published threshold   |
| Per-slice accuracy         | within ±5 % of overall mean   |

DIF reviews emit audit entries; retired items move to
an archived state and cannot be reused.

## Annex G — Cohort-level metrics

The protocol records cohort-level metrics so QA
audits can examine cohort outcomes:

- enrolment vs. completion ratio
- weekly drop-off curve
- per-module average score
- inter-rater reliability (peer assessment)
- proctoring-flag distribution
- accessibility-track utilisation
- adaptive-learning opt-in rate

Metrics export at the cohort-end event so analytics
consumers see complete cohort data.

## Annex H — Operator-credential binding

| Credential                | Source                                 |
|---------------------------|----------------------------------------|
| Course author / instructor| sponsor + recognising body              |
| Assessment item author    | sponsor + content-quality SOP           |
| Accessibility auditor     | per accessibility-audit body            |
| Proctor                   | proctoring service + sponsor            |
| Peer-grader               | learner with calibration submission     |

A signing event without an active credential rejects.