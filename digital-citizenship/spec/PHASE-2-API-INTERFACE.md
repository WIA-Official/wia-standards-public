# WIA-digital-citizenship PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-digital-citizenship
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTPS API contract that a
digital-citizenship programme exposes for the records
defined in PHASE-1. Consumers include the programme
operator, the operating jurisdiction's education
ministry or programme funder, civil-society partner
organisations, the parents and guardians of minor
learners (for the per-learner record they are entitled
to under the operating jurisdiction's privacy regime),
the operating jurisdiction's online-safety authority
(where the programme operates an escalation channel
for online-safety incidents), and the programme's
external evaluators.

References (CITATION-POLICY ALLOW only):

- IETF RFC 9110 (HTTP Semantics)
- IETF RFC 9111 (HTTP Caching)
- IETF RFC 9457 (Problem Details)
- IETF RFC 6901 / 6902 (JSON Pointer / Patch)
- IETF RFC 8288 (Web Linking)
- IETF RFC 8259 (JSON)
- IETF RFC 9421 (HTTP Message Signatures)
- ISO 8601 (date and time)
- ISO/IEC 27001:2022 (information security management)
- W3C Trace Context
- W3C WCAG 2.2 Level AA
- US COPPA + 16 CFR Part 312
- UK Age-Appropriate Design Code
- EU GDPR Article 8

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published by the
programme. Versioning uses `/v1/` path segments. The
OpenAPI 3.1 document at `/v1/openapi.json` is canonical.

This API is the programme-facing facade for digital-
citizenship records. Learner-facing experience
(curriculum delivery, online module navigation,
assessment activities) flows through the programme's
learning-platform surface; this API records the
artefacts of regulatory or funder reporting
significance.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-digital-citizenship",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "programmes":             "/v1/programmes",
    "curriculumModules":      "/v1/curriculum-modules",
    "cohorts":                "/v1/cohorts",
    "learners":               "/v1/learners",
    "moduleCompletions":      "/v1/module-completions",
    "programmeOutcomes":      "/v1/programme-outcomes",
    "onlineSafetyIncidents":  "/v1/online-safety-incidents",
    "eGovernmentLiteracyTopics": "/v1/e-government-topics",
    "evidence":               "/v1/evidence",
    "openapi":                "/v1/openapi.json"
  }
}
```

## §3 Programme Lifecycle

```
POST   /v1/programmes              — register a programme
GET    /v1/programmes/{pid}        — retrieve programme
PATCH  /v1/programmes/{pid}/status — advance status
PATCH  /v1/programmes/{pid}/learner-age-range
                                   — record learner-
                                     age-range; the
                                     programme's
                                     parental-consent
                                     discipline is
                                     derived from the
                                     range against
                                     the operating
                                     jurisdiction's
                                     age-of-consent
                                     threshold
```

Programmes whose `learnerAgeRange.minAge` is below
the operating jurisdiction's age-of-consent threshold
without a `parentalConsentDiscipline` field on the
cohort return `409` with type
`urn:wia:digital-citizenship:parental-consent-
discipline-required`.

## §4 Curriculum Modules

```
POST   /v1/programmes/{pid}/curriculum-modules
                                   — register a
                                     module
PATCH  /v1/curriculum-modules/{mid}/accessibility-profile
                                   — record WCAG 2.2
                                     conformance
                                     evidence reference
GET    /v1/curriculum-modules/{mid}
                                   — retrieve module
```

Module submissions without `accessibilityProfileRef`
return `422` with type
`urn:wia:digital-citizenship:wcag-evidence-required`.

## §5 Cohorts

```
POST   /v1/programmes/{pid}/cohorts
                                   — register a cohort
PATCH  /v1/cohorts/{cid}/status    — advance cohort
                                     status
GET    /v1/cohorts/{cid}           — retrieve cohort
```

Cohorts whose `parentalConsentDiscipline` is
`required-per-coppa` / `required-per-gdpr-art-8` /
`required-per-uk-aadc` enforce parental consent at
learner enrolment per §6.

## §6 Learners

```
POST   /v1/cohorts/{cid}/learners  — enrol a learner
PATCH  /v1/learners/{lid}/parental-consent
                                   — record parental
                                     consent capture
PATCH  /v1/learners/{lid}/inclusive-access-needs
                                   — record declared
                                     accessibility
                                     needs
PATCH  /v1/learners/{lid}/withdrawal
                                   — record learner
                                     withdrawal
GET    /v1/learners/{lid}          — retrieve learner
```

Learner submissions on a cohort with
`parentalConsentDiscipline` requiring parental consent
without a `parentalConsentRef` return `409` with type
`urn:wia:digital-citizenship:parental-consent-
required`.

## §7 Module Completions

```
POST   /v1/learners/{lid}/module-completions
                                   — register completion
PATCH  /v1/module-completions/{cid}/competency-attestation
                                   — update attestation
                                     level
GET    /v1/module-completions/{cid}
                                   — retrieve completion
```

Module-completion records inherit the learner's
parental-consent state; completions on learners who
have withdrawn are accepted only as backfill within
the operator's data-retention rules.

## §8 Programme Outcomes

```
POST   /v1/programmes/{pid}/programme-outcomes
                                   — register a
                                     reporting-period
                                     outcome aggregate
GET    /v1/programme-outcomes/{oid}
                                   — retrieve outcome
GET    /v1/programmes/{pid}/programme-outcomes?period=...
                                   — query outcomes
                                     by period
```

Outcome aggregates carry only aggregate counts; per-
learner data does not appear in outcome payloads,
mirroring the operating jurisdiction's privacy
discipline for minor-learner reporting.

## §9 Online-Safety Incidents

```
POST   /v1/programmes/{pid}/online-safety-incidents
                                   — register an
                                     incident
PATCH  /v1/online-safety-incidents/{iid}/triage
                                   — record triage
                                     outcome
PATCH  /v1/online-safety-incidents/{iid}/pastoral-support
                                   — link pastoral
                                     support narrative
GET    /v1/online-safety-incidents/{iid}
                                   — retrieve incident
```

Online-safety-incident submissions whose
`incidentClassification` is `image-based-abuse` or
`grooming-attempt` are gated to the programme's
safeguarding-lead role and trigger the operating
jurisdiction's online-safety-authority escalation
workflow.

## §10 E-Government Literacy Topics

```
POST   /v1/programmes/{pid}/e-government-topics
                                   — register a topic
PATCH  /v1/e-government-topics/{tid}/portal-accessibility
                                   — record portal
                                     WCAG conformance
                                     reference
GET    /v1/e-government-topics/{tid}
                                   — retrieve topic
```

Topics targeting public-sector portals without an
`accessibilityProfileRef` return `422` with type
`urn:wia:digital-citizenship:portal-accessibility-
evidence-required`.

## §11 Errors, Authentication, Caching, Audit

Errors: `application/problem+json` per RFC 9457 with
the types named above plus
`urn:wia:digital-citizenship:evidence-mismatch`.
Authentication: mutually-authenticated TLS for funder,
education-ministry, civil-society-partner, online-
safety-authority, and external-evaluator consumers;
parental access to the per-learner record uses the
operator's identity-verified parent-portal flow.
Caching: stable resources (closed cohorts, archived
programmes, completed module artefacts) cacheable
with `Cache-Control: max-age=31536000, immutable`.
Audit logs carry `programmeId`, `cohortId`,
`learnerId`, `traceId`, the issuing client
certificate's subject, and the programme's clock skew
vs the operating jurisdiction's NTP service.

## §12 Streaming Subscription, Bulk, Pagination, Provenance

SSE at `/v1/programmes/{pid}/events` for programme-
wide events (cohort opened, online-safety-incident
escalated, module accessibility profile updated,
funder reporting period elapsed). Subscribers
reconnect via `Last-Event-ID`. Bulk endpoints:
`/v1/bulk/learners`, `/v1/bulk/module-completions`,
`/v1/bulk/programme-outcomes`. Cursor-based
pagination via `cursor` and `Link` headers. Provenance
via `/v1/provenance/{recordId}` emits the in-toto
attestation chain for any record.

## §13 Worked Example: School-Cohort Online-Safety Pathway

1. Public-school programme registers a curriculum
   module on online-safety with WCAG 2.2 Level AA
   evidence (captioning, sign-language interpretation,
   plain-language summary).
2. Cohort opens with `parentalConsentDiscipline=
   required-per-gdpr-art-8` (operating jurisdiction
   is an EU Member State with Article 8 age threshold
   in force).
3. Learners enrol; parental consent captured per
   learner under the GDPR Article 8 / Article 7
   discipline.
4. Module delivery proceeds; learner-experience
   incidents are reported via `POST /online-safety-
   incidents`.
5. Triage by the programme's safeguarding lead;
   incidents of classification `image-based-abuse`
   are escalated to the operating jurisdiction's
   online-safety authority.
6. End of cohort: aggregate outcome registered via
   `POST /programme-outcomes` and reported to the
   programme's funder.

## §14 Aggregate and Provenance Endpoints

```
GET    /v1/provenance/{recordId}
GET    /v1/aggregate/enrolment-volume?period=...
GET    /v1/aggregate/completion-rate?competencyArea=...&period=...
GET    /v1/aggregate/online-safety-incident-rate?period=...&kind=...
GET    /v1/aggregate/inclusive-access-provision-use?period=...
```

## §15 Conformance

A conformant server passes the test vectors published
under `tests/phase-vectors/phase-2-api-interface/`,
emits an OpenAPI 3.1 document, signs evidence packages
per RFC 9421, refuses curriculum-module submissions
without WCAG 2.2 evidence, refuses learner enrolments
on cohorts whose parental-consent discipline requires
consent without consent records, and gates online-
safety incidents per the safeguarding role-based-
access-control discipline.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-digital-citizenship
- **Last Updated:** 2026-04-28
