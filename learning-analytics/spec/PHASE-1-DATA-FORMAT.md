# WIA-learning-analytics PHASE 1 — Data Format Specification

**Standard:** WIA-learning-analytics
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable

This PHASE defines the canonical data format for
WIA-learning-analytics, the cross-platform learning
analytics interoperability standard. The records bind
every learner action, learning resource, course
activity, predictive intervention, and accessible
analytics dashboard to a documented xAPI profile, IMS
Caliper sensor, or LOM resource so that downstream
researchers, educators, and learners can audit how
analytics influence pedagogical decisions.

References (CITATION-POLICY ALLOW only):
- IEEE 1484.20.1 (xAPI — Experience API 1.0.3)
- IEEE 1484.12.1 (LOM — Learning Object Metadata)
- ISO/IEC 20748-1..-4 (Learning analytics interoperability)
- ISO/IEC 23988 (IT-supported assessment delivery)
- IMS Caliper Analytics 1.2 (Sensor API, Profile)
- IMS LTI 1.3 / Advantage (Deep Linking, NRPS, AGS)
- IMS OneRoster 1.2, IMS QTI 3.0
- IMS Open Badges 3.0, W3C VC Data Model 2.0
- xAPI Profile Server profiles (cmi5, ADL, Open Cmi5)
- SoLAR Learning Analytics Reference Model (LARM)
- W3C Decentralized Identifiers 1.0, W3C SHACL
- IETF RFC 8259 (JSON), RFC 8785 (JCS), RFC 7515 (JWS), RFC 4122 (UUID)
- ISO/IEC 27001:2022, ISO/IEC 27701:2019
- EU GDPR Articles 13, 14, 15, 22 (automated decision-making)
- US FERPA, US PPRA; K-PIPA, LGPD, PIPL, COPPA

---

## §1 Scope

This PHASE applies to records that capture learner
behaviour, the analytics derived from it, the
interventions triggered by analytics, and the
disclosure of analytics back to learners and
instructors. It covers formative classroom contexts,
self-directed online learning, sovereign-scale
education ministries, and corporate training
programmes.

In scope: actor record, action record, object record,
context record, statement record (xAPI / Caliper),
profile record, model record (predictive analytics),
intervention record, dashboard record, and the
cross-references binding each statement to the
profile under which it is interpreted.

Out of scope: pedagogical curriculum design and
classroom assessment policy (handled by sovereign
education ministries); analytics models trained on
sensitive third-party data (governed by the third-
party data agreement).

## §2 Actor record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `actorRef`           | UUID (RFC 4122) opaque                          |
| `objectType`         | `Agent`, `Group` per xAPI 1.0.3                 |
| `account.homePage`   | the LRS or Caliper sensor authority URL          |
| `account.name`       | local identifier within `homePage`               |
| `did`                | optional DID per W3C DID 1.0                     |
| `pseudonymRef`       | provider-local code; never the legal name        |
| `roleRef`            | `learner`, `instructor`, `proctor`, `auditor`,   |
|                      | `parent-or-guardian`                             |
| `consentRef`         | active analytics consent record                  |

xAPI mailbox identifiers (`mbox`, `mbox_sha1sum`) are
forbidden by this PHASE; records that carry them are
rejected by the LRS validator.

## §3 Action (verb) record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `verbRef`            | URI; xAPI verb (e.g. ADL `experienced`,          |
|                      | `attempted`, `completed`, `passed`, `failed`)    |
| `display`            | localised label (BCP 47 keys)                    |
| `profileRef`         | xAPI Profile that defines the verb's semantics   |

Implementations MUST register every verb URI under a
profile (this PHASE §6); ad-hoc verbs are recorded as
`claimed` and excluded from federated analytics until
profiled.

## §4 Object record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `objectRef`          | URI; an Activity, Statement, or Group reference |
| `objectType`         | `Activity`, `StatementRef`, `Agent`, `Group`,   |
|                      | `SubStatement`                                   |
| `definition.type`    | LOM `educationalUse` URI or Caliper Entity type |
| `definition.name`    | localised name (BCP 47 keys)                    |
| `lomRef`             | optional IEEE 1484.12.1 LOM record              |
| `caliperEntity`      | optional Caliper Entity URI                     |
| `extensions`         | profile-defined extensions only                 |

Activities reference their LOM record so that
analytics joins can correlate behaviour with
educational metadata (level, age range, language).

## §5 Statement (xAPI / Caliper) record

A statement is the smallest replayable unit of
analytics.

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `statementRef`       | UUID                                            |
| `actor`              | this PHASE §2                                   |
| `verb`               | this PHASE §3                                   |
| `object`             | this PHASE §4                                   |
| `result.score`       | scaled 0..1 with raw / min / max if applicable  |
| `result.success`     | boolean                                         |
| `result.duration`    | ISO 8601 duration                                |
| `result.response`    | string; truncated to 4096 octets                |
| `context.registration` | UUID linking related statements                |
| `context.platform`   | LRS or Caliper sensor identity                  |
| `context.profile`    | xAPI Profile reference                          |
| `timestamp`          | ISO 8601 with timezone offset                   |
| `stored`             | ISO 8601 — set by the LRS                       |
| `authority`          | signing actor (LRS or sensor authority)         |

Caliper envelopes wrap multiple statements with a
`sensor`, `sendTime`, and `dataVersion`; the registry
records the envelope as a single ingest event.

## §6 Profile record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `profileRef`         | URI per xAPI Profile Specification               |
| `version`            | Semantic Versioning 2.0.0                       |
| `concepts[]`         | verb / activity-type / extension definitions    |
| `templates[]`        | statement templates (constraints over §5)        |
| `patterns[]`         | event-pattern grammar (sequence, alternates)    |
| `seeAlso`            | URI to authoritative profile documentation      |

A statement is `profile-conformant` if it satisfies
at least one template in the profile and any pattern
in which it appears matches.

## §7 Model record (predictive analytics)

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `modelRef`           | UUID                                            |
| `modelCardRef`       | URI to model card (ISO/IEC 23053 / 23894 form)  |
| `family`             | classification, regression, sequence, anomaly,  |
|                      | recommendation                                  |
| `inputProfile`       | xAPI / Caliper profile the model consumes       |
| `outputContract`     | the prediction shape (probability, ranked list, |
|                      | numeric score, categorical label)               |
| `trainingScope`      | data-collection window, opt-in basis            |
| `fairnessAuditRef`   | URI to fairness audit report                    |
| `eulaAttestationRef` | EU AI Act conformity assessment if high-risk    |
|                      | (Annex III §3) per Article 43                   |

Models that are EU AI Act high-risk MUST publish the
post-market monitoring plan referenced from
`fairnessAuditRef`.

## §8 Intervention record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `interventionRef`    | UUID                                            |
| `triggerStatement[]` | statementRef list that triggered the action     |
| `modelRef`           | this PHASE §7                                    |
| `actionType`         | `nudge`, `flag-to-instructor`, `auto-block`,    |
|                      | `recommend-resource`, `escalate-to-counsellor`  |
| `actionTarget`       | `actorRef` recipient                             |
| `humanReviewRef`     | identifier of the human reviewer where required |
|                      | by GDPR Article 22 / EU AI Act                  |
| `outcomeRef`         | downstream outcome statementRef                 |

Interventions of type `auto-block` are forbidden for
high-stakes academic decisions without a human
reviewer signature.

## §9 Dashboard record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `dashboardRef`       | URI                                             |
| `audience`           | `learner`, `instructor`, `programme`,           |
|                      | `institution`                                   |
| `dataScope`          | `self`, `class`, `cohort`, `aggregate`          |
| `accessibilityRef`   | WCAG 2.2 AA conformance evidence URI            |
| `explainabilityRef`  | URI to per-card explanation (model rationale,   |
|                      | confidence interval)                            |
| `optOutPath`         | URI describing how the audience opts out        |

Learner-facing dashboards MUST honour the opt-out
without imposing a downstream cost.

## §10 Cross-domain references (informative)

- WIA-language-learning — proficiency analytics
- WIA-lms — Caliper / xAPI sources at the platform
- WIA-prompts — analytics for prompted tutoring
- WIA-multiverse-interface — XR learning analytics

## Annex A — Conformance disclosure

Implementations declare the xAPI / Caliper schema URIs
they accept, the canonicalisation form (RFC 8785), and
the JWS key set used to sign statements and dashboards.

## Annex B — Statement size limits

Statement payloads are bounded:

- `result.response` ≤ 4096 octets;
- `extensions` total size ≤ 16 KB;
- Caliper envelope ≤ 1 MB.

Oversized statements are split or dropped at ingest;
the splitter writes a `compound:true` extension into
the originating statement.

## Annex C — Worked statement (informative)

```json
{
  "statementRef": "f63f4f04-9a76-4d2c-9f53-2c4d09a1bbb1",
  "actor": {"objectType": "Agent",
            "account": {"homePage": "https://la.wiastandards.com",
                        "name": "learner-072"}},
  "verb": {"id": "http://adlnet.gov/expapi/verbs/completed"},
  "object": {"id": "https://la.wiastandards.com/activities/quiz/01",
             "definition": {"type": "http://adlnet.gov/expapi/activities/assessment"}},
  "result": {"score": {"scaled": 0.84}, "success": true},
  "timestamp": "2026-04-28T11:32:00+09:00"
}
```

## Annex D — Versioning

Field additions are minor; field removals or semantic
redefinition require a major bump synchronised with
the corresponding xAPI / Caliper version.

## Annex E — Conformance level

Conformance is "Core" (actor + action + object +
statement + profile) or "Full" (adds model,
intervention, and dashboard records under EU AI Act
or sovereign-equivalent governance).

## Annex F — Pseudonymisation

Statements are pseudonymised before the analytics
engine consumes them. Re-identification keys are
held under the deployment's privacy regime; auditors
access keys only under a documented court or
authority order.

## Annex G — Children and minors

Records identifying a minor follow the deployment's
child-online-protection regime (COPPA, K-PIPA Article
22-2, GDPR Article 8). Predictive interventions on
minors require an additional guardian-consent record.

## Annex H — Statement-level retention

Statements are retained per the deployment's policy.
Retention windows are recorded at submission and
verified at audit; statements past their window are
hard-deleted, with an aggregate counter retained for
ministry reporting.

## Annex I — Cross-platform actor reconciliation

Actors registered under multiple `homePage` authorities
are reconciled via the registry's pseudonym-resolution
service. Reconciliation events are recorded in the
audit feed and surface to the actor in their data
subject access export.

## Annex J — Cohort definition primitives

Cohorts are defined by SHACL shapes over the actor
record schema. Common primitives:

- enrolment programme code;
- declared accommodations;
- cohort entry term;
- declared first-language tag.

Cohort definitions exclude any field whose use would
violate the deployment's privacy regime (e.g.
explicit demographic categorisation under GDPR
Article 9).

弘益人間 (Hongik Ingan) — Benefit All Humanity
