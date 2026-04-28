# WIA-mooc PHASE 2 — API Interface Specification

**Standard:** WIA-mooc
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable

This PHASE defines the resource-oriented API surface
for MOOC operations: course / cohort / module / lesson
catalogue management, learner enrolment, learning-event
ingest (Caliper / xAPI), assessment-item authoring and
attempt capture, peer-assessment submission and
review, discussion-thread retrieval, content-asset
serving with adaptive streaming, accessibility-
assertion publication, progress and completion
retrieval, and credential-binding issuance.

References (CITATION-POLICY ALLOW only):
- IETF RFC 9110 (HTTP), RFC 9112 (HTTP/1.1), RFC 9113 (HTTP/2)
- IETF RFC 9457 (Problem Details for HTTP APIs)
- IETF RFC 8259 (JSON), RFC 8785 (JCS), RFC 4122 (UUID), RFC 9530 (Content-Digest)
- IETF RFC 6749, RFC 7636 (PKCE), RFC 8414 (OAuth Authorization Server Metadata)
- 1EdTech Caliper Analytics 1.2 — sensor / endpoint convention
- ADL xAPI 2.0 — LRS API
- 1EdTech LTI 1.3 / LTI Advantage (NRPS, AGS, Deep Linking)
- 1EdTech QTI 3.0 — Question and Test Interoperability
- W3C Web Annotation Protocol; W3C Activity Streams 2.0
- HLS / DASH — adaptive media manifest
- W3C WCAG 2.2 — accessibility assertions

---

## §1 Endpoint root

API root is implementation-controlled. All endpoints
are TLS 1.3 (RFC 8446). LTI 1.3 endpoints honour the
LTI security framework; learner endpoints use OAuth 2
with PKCE.

## §2 Course / cohort endpoints

```
POST   /v1/courses                       create course
GET    /v1/courses/{ref}                 retrieve
PATCH  /v1/courses/{ref}                 amend
GET    /v1/courses?subject=&level=       list / filter
POST   /v1/cohorts                       create cohort
PATCH  /v1/cohorts/{ref}                 amend
POST   /v1/cohorts/{ref}/open-enrolment  open registration
```

Course versioning is mandatory on outcome / assessment
changes; outstanding enrolments operate on the version
they began.

## §3 Module / lesson / activity endpoints

```
POST   /v1/courses/{c}/modules           add module
POST   /v1/courses/{c}/modules/{m}/lessons add lesson
POST   /v1/courses/{c}/lessons/{l}/activities add activity
PATCH  /v1/lessons/{ref}                 amend lesson
GET    /v1/lessons/{ref}                 retrieve
DELETE /v1/lessons/{ref}                 retire lesson
```

## §4 Learner / enrolment endpoints

```
POST   /v1/learners                      register learner
GET    /v1/learners/{ref}                retrieve
POST   /v1/enrolments                    enrol learner in cohort
PATCH  /v1/enrolments/{ref}              upgrade / downgrade track
DELETE /v1/enrolments/{ref}              withdraw
GET    /v1/cohorts/{ref}/enrolments      list cohort
```

## §5 Learning-event endpoints (Caliper / xAPI)

```
POST   /v1/events                        Caliper sensor endpoint
POST   /v1/xapi/statements               xAPI LRS endpoint
GET    /v1/events?actor=&time=           list / filter
GET    /v1/events/$export                bulk async export
```

Caliper accepts the v1.2 envelope; xAPI accepts v2.0
PUT statements. Implementations may dual-emit (one
event triggers both forms) for downstream compatibility.

## §6 Assessment endpoints

```
POST   /v1/items                         author item
GET    /v1/items/{ref}                   retrieve QTI 3.0 payload
POST   /v1/attempts                      record attempt
GET    /v1/attempts/{ref}                retrieve
POST   /v1/attempts/{ref}/score          score (auto or rater-applied)
GET    /v1/items/{ref}/calibration       retrieve calibrated difficulty
```

Calibration recomputes on a sponsor-policy clock;
calibration changes are versioned so prior attempts
honour the calibration in force at the time.

## §7 Peer-assessment endpoints

```
POST   /v1/peer-assessments              submit work for peer review
POST   /v1/peer-assessments/{ref}/grade  peer grader submits review
GET    /v1/peer-assessments/{ref}        retrieve (gated)
GET    /v1/peer-assessments?learner=     list per learner
```

Peer-grader assignments randomise per the rubric's
distribution policy; raters and authors are mutually
anonymised.

## §8 Discussion endpoints

```
POST   /v1/discussions                   post thread / reply
GET    /v1/discussions/{ref}             retrieve
PATCH  /v1/discussions/{ref}             edit (audited)
DELETE /v1/discussions/{ref}             moderation removal
GET    /v1/discussions?course=&since=    list
```

Discussion records honour W3C Web Annotation Data
Model where the discussion is anchored to a specific
content position.

## §9 Content-asset endpoints

```
POST   /v1/assets                        register asset metadata
POST   /v1/assets/{ref}/upload           chunked upload (Range)
GET    /v1/assets/{ref}                  retrieve metadata
GET    /v1/assets/{ref}/manifest.m3u8    HLS manifest
GET    /v1/assets/{ref}/manifest.mpd     DASH manifest
GET    /v1/assets/{ref}/transcript.vtt   WebVTT transcript
GET    /v1/assets/{ref}/sign-language    sign-language track
```

Manifests are signed (RFC 7515) and include the
content digest so the player can verify the manifest
binds to the expected content.

## §10 Accessibility-assertion endpoints

```
POST   /v1/accessibility-assertions      record assertion
GET    /v1/accessibility-assertions/{ref}  retrieve
GET    /v1/courses/{ref}/accessibility   list assertions
```

Auditor signs the assertion; the assertion publishes
to the course public pages so prospective learners
see the conformance level.

## §11 Progress / completion endpoints

```
GET    /v1/learners/{ref}/progress       per-course progress
POST   /v1/completions                   record completion (instructor / system)
GET    /v1/completions/{ref}             retrieve
POST   /v1/completions/{ref}/credential  bind / issue credential
```

## §12 Error model (RFC 9457)

```json
{
  "type":   "urn:wia:mooc:problem:enrolment-closed",
  "title":  "Cohort enrolment is closed",
  "status": 409,
  "detail": "Cohort 2026-Q2 closed enrolment on 2026-04-10",
  "instance": "/v1/enrolments"
}
```

Common type URIs:

| Type URI suffix              | HTTP | Meaning                                       |
|------------------------------|-----:|-----------------------------------------------|
| `enrolment-closed`           | 409  | cohort registration window closed             |
| `prerequisite-not-met`       | 422  | learner missing prerequisite                  |
| `qti-malformed`              | 422  | QTI payload fails validation                  |
| `peer-grade-incomplete`      | 422  | required peer-grades not yet submitted        |
| `accessibility-failed`       | 422  | content fails declared WCAG level             |
| `proctoring-required`        | 403  | attempt requires proctoring binding           |

## §13 Bulk export

```
GET  /v1/$export?_type=Enrolment,Event,Attempt,Completion
GET  /v1/$status/{exportId}
GET  /v1/$result/{exportId}/{file}
```

Output is NDJSON (Caliper or xAPI form per query).

## §14 Audit headers

| Header                  | Meaning                                       |
|-------------------------|-----------------------------------------------|
| `X-Request-Id`          | client-set, echoed                            |
| `X-Audit-Event-Id`      | server-set, links to PHASE 3 audit chain      |
| `Content-Digest`        | RFC 9530 SHA-256 of the response body         |

## Annex A — OpenAPI reference

A canonical OpenAPI 3.1 description is published at
`api/openapi-3.1.yaml`.

## Annex B — Worked LTI 1.3 launch (informative)

The MOOC platform consumes external tools through LTI
1.3 with LTI Advantage:

```
Tool catalogue → Deep Linking 2.0 selection →
LTI Resource Link launch → NRPS roster sync →
AGS grade returns
```

Each launch carries a JWT signed by the issuer
platform; the tool verifies via the JWKS published in
the platform's `tool-platform-config`.

## Annex C — Webhook surface

Implementations expose webhooks for `enrolment-
completed`, `attempt-submitted`, `cohort-finished`,
and `credential-issued` events. Payloads sign with
RFC 7515 JWS; receivers verify against
`/.well-known/wia-mooc-keys.json`. Delivery is at-
least-once; receivers are expected to be idempotent on
`eventId`.

## Annex D — Conformance disclosure

Implementations declare the Caliper / xAPI versions
served, the LTI 1.3 services supported, the QTI
revision, the WCAG / EN 301 549 audit results, and
the credential-binding profiles.

## Annex E — Async export pattern

```
POST   /v1/$export                      → 202 Accepted, Content-Location
GET    /v1/$status/{id}                 → 202 in-progress / 200 manifest
GET    /v1/$result/{id}/{file}          → 200 NDJSON
DELETE /v1/$status/{id}                 → 202 cancellation
```

The 202 response carries `Retry-After` for the
polling client and `X-Progress` for human-readable
progress indication.

## Annex F — Adaptive-recommendation request

For adaptive-learning platforms an instructional
recommender exposes:

```
POST   /v1/recommendations             query the recommender
GET    /v1/recommendations/{ref}       retrieve persisted recommendation
```

Request payload:

```json
{
  "learnerRef": "L-007",
  "courseRef": "Statistics-101",
  "lastEventRef": "evt-2026-04-12-...",
  "policyRef": "wia-mooc://policy/adaptive-2026"
}
```

Response carries the recommended next activity
reference, the model attestation, and the
recommendation rationale (for explainability where the
model is explainable).

## Annex G — Captioning-pipeline endpoint

```
POST   /v1/captioning-jobs             submit asset for captioning
GET    /v1/captioning-jobs/{ref}       retrieve job state
PATCH  /v1/captioning-jobs/{ref}       human-reviewer corrections
```

Captioning-jobs follow an automatic-then-human-reviewed
pipeline; corrections sign with the reviewer key so
the final transcript binds to the human reviewer.
