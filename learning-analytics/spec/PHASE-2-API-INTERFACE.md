# WIA-learning-analytics PHASE 2 — API Interface Specification

**Standard:** WIA-learning-analytics
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable

This PHASE defines the API surfaces that WIA-learning-
analytics participants expose so that learning record
stores (LRSs), Caliper sensors, learning management
systems, predictive-model providers, dashboard
authors, and audit authorities can submit statements,
register profiles, run model evaluations, deliver
interventions, and reconcile dashboards through a
single contract.

References (CITATION-POLICY ALLOW only):
- OpenAPI Specification 3.1
- IETF RFC 9110 (HTTP Semantics), RFC 9457 (Problem Details)
- IETF RFC 7515 (JWS), RFC 7519 (JWT)
- IETF RFC 9421 (HTTP Message Signatures)
- IEEE 1484.20.1 xAPI 1.0.3 (LRS contract)
- IMS Caliper 1.2 (Sensor API)
- ISO/IEC 20748-3 Reference Architecture
- IMS LTI 1.3 / Advantage (NRPS, AGS)

---

## §1 Scope

This PHASE specifies the HTTP-based interfaces between
sensors (LMSs, browsers, mobile clients, immersive
runtimes), LRSs, profile servers, model providers,
dashboards, and audit authorities. The PHASE does not
specify the in-process analytics algorithm or the
on-disk storage layout of the LRS.

## §2 Operation groups

| Prefix              | Group                                           |
|---------------------|-------------------------------------------------|
| `/v1/statements`    | xAPI / Caliper statement ingest and query        |
| `/v1/profiles`      | xAPI Profile registry                            |
| `/v1/activities`    | activity (object) registry                       |
| `/v1/actors`        | actor pseudonymisation registry                  |
| `/v1/models`        | predictive-model registry and evaluation         |
| `/v1/interventions` | intervention orchestration                       |
| `/v1/dashboards`    | dashboard registry and access policy             |
| `/v1/registry`      | registry directory                               |

## §3 Authentication

xAPI ingest endpoints accept the xAPI Basic and OAuth
2.0 client-credentials grants. Caliper endpoints
accept Bearer JWT (RFC 7519). Authorisation scopes
(`statements:write`, `statements:read`, `profiles:write`,
`models:evaluate`) are advertised in the discovery
document.

## §4 Statement operations

### 4.1 Submit statements

```
PUT /v1/statements
POST /v1/statements
```

Body: a single xAPI statement or an array of
statements, or a Caliper envelope. Response: 204 with
`X-Experience-API-Statement-ID` header listing the
assigned statement IDs.

### 4.2 Query

```
GET /v1/statements?actor=<json>&verb=<URI>&since=<ISO8601>&until=<ISO8601>
```

Returns matching statements per xAPI 1.0.3 §7.2.4.
Pagination uses the `more` field with an opaque cursor.

### 4.3 Voiding

A statement is voided by emitting a new statement with
verb `http://adlnet.gov/expapi/verbs/voided` and
object `{"objectType": "StatementRef", "id": <ref>}`.

## §5 Profile operations

### 5.1 Publish profile

```
POST /v1/profiles
Content-Type: application/json
```

Body: an xAPI Profile JSON document. The registry
validates against xAPI Profile schema and rejects
profiles whose concepts collide with existing
profiles unless the publisher owns both.

### 5.2 Lookup

```
GET /v1/profiles/{profileRef}
```

Returns the canonical profile document. Conditional
GET via `ETag` is honoured.

### 5.3 Conformance check

```
POST /v1/profiles/{profileRef}/check
```

Body: an array of statements. Response: per-statement
verdict (`conformant`, `non-conformant`) with the
specific template or pattern violated.

## §6 Activity operations

### 6.1 Register activity

```
POST /v1/activities
```

Body: object record (PHASE-1 §4) with attached LOM
metadata.

### 6.2 Lookup

```
GET /v1/activities/{activityRef}
```

Returns the canonical activity record and any
authoritative profile linkage.

## §7 Actor operations

### 7.1 Resolve pseudonym

```
POST /v1/actors/resolve
```

Body: a `homePage` + `name` pair. Response: the
opaque `actorRef`. Resolution is rate-limited to
prevent enumeration.

### 7.2 Withdraw consent

```
DELETE /v1/actors/{actorRef}/consent
```

Marks the actor's analytics consent withdrawn. New
statements about this actor are rejected; historical
statements remain visible to the actor and are
embargoed from federated analytics.

## §8 Model operations

### 8.1 Register model

```
POST /v1/models
```

Body: model record (PHASE-1 §7) with model card URL.

### 8.2 Evaluate

```
POST /v1/models/{modelRef}/evaluate
```

Body: an actor reference and a context window.
Response: the model's prediction with confidence
interval and `humanReviewRequired` boolean per
GDPR Article 22 / EU AI Act.

## §9 Intervention operations

### 9.1 Schedule

```
POST /v1/interventions
```

Body: intervention record (PHASE-1 §8). Response:
`interventionRef` and the assigned reviewer pool if
human review is required.

### 9.2 Outcome

```
POST /v1/interventions/{interventionRef}/outcome
```

Body: outcome statementRef and an instructor
narrative. Outcomes feed back into the model
registry for ongoing fairness audits.

## §10 Dashboard operations

### 10.1 Register dashboard

```
POST /v1/dashboards
```

Body: dashboard record (PHASE-1 §9) with WCAG 2.2 AA
conformance evidence URL.

### 10.2 Access policy

```
GET /v1/dashboards/{dashboardRef}/policy
```

Returns the access policy: audiences, opt-out path,
and the profile-derived data scope.

## §11 Error semantics

Errors are `application/problem+json` (RFC 9457)
namespaced under
`https://wiastandards.com/errors/learning-analytics/`.

## §12 Caching and rate limits

Statement query endpoints are not cached. Profile,
activity, and model registry endpoints carry strong
`ETag` and short max-age. Dashboard policy is cached
for 5 minutes with `stale-while-revalidate`.

## Annex A — OpenAPI 3.1 fragment

```yaml
openapi: 3.1.0
info: {title: WIA-learning-analytics API, version: 1.0.0}
paths:
  /v1/statements:
    post:
      summary: Submit xAPI statements
      requestBody:
        required: true
        content:
          application/json:
            schema: {$ref: 'StatementArray.schema.json'}
      responses:
        '204': {description: Stored}
```

## Annex B — Idempotency

Statement IDs are client-provided UUIDs. The LRS
deduplicates on ID; resubmitting an existing
statement returns 204 with the original ID.

## Annex C — Webhook subscriptions

Subscribers receive events on `statement.stored`,
`profile.published`, `model.registered`,
`intervention.scheduled`, `dashboard.opted-out`.

## Annex D — Federation

Federated LRSs propagate statements across peers when
both peers declare the same profile in common. Cross-
LRS queries follow `X-WIA-Federation-Path`.

## Annex E — Statement-level signing

A statement MAY be signed by attaching a JWS over the
canonical statement (RFC 8785). The signature appears
as an attachment with `usageType =
http://adlnet.gov/expapi/attachments/signature`.

## Annex F — Bulk export

`POST /v1/registry/export` returns a signed URL to a
`tar.zst` of the deployment's statements filtered by
profile and date range. PII fields are redacted.

## Annex G — Sandbox endpoints

`/v1/sandbox` mirrors the production surface with
synthetic actors and ephemeral state.

## Annex H — Quotas

Per-deployment publish quotas default to 10,000
statements per minute and 5,000,000 per day.
`X-Quota-Remaining` is surfaced on every write.

## Annex I — Webhook payload shape

```json
{
  "event": "intervention.scheduled",
  "interventionRef": "f63f4f04-...",
  "modelRef": "model-2026-04-28",
  "actorRef": "actor-072",
  "humanReviewRequired": true
}
```

## Annex J — Audit feed

`GET /v1/registry/audit?since=<timestamp>` returns
mutating-operation events for the deployment. JWT
scope `audit-feed:read` required.

## Annex K — Reviewer queue

```
GET /v1/interventions/queue?reviewerRef={ref}
```

Returns the reviewer's pending queue with deadlines.
Reviewers acknowledge interventions before delivery.

## Annex L — Data subject access

```
GET /v1/actors/{actorRef}/export
```

Returns the actor's full data subject access export
per GDPR Article 15. Requests are rate-limited.

## Annex M — Profile pattern engine

```
POST /v1/profiles/{profileRef}/match
```

Body: an ordered statement sequence and a target
pattern. Response: a verdict (`match`, `partial`,
`mismatch`) with the position of divergence. The
pattern engine is reproducible from the open-source
implementation referenced in the response header
`X-WIA-PatternEngine-Source`.

## Annex N — Cohort definition

```
POST /v1/registry/cohorts
```

Body: a SHACL shape constraining the actor record
schema. Response: an opaque `cohortRef` that can be
referenced from dashboard records and aggregate
queries. Cohorts are versioned with Semantic
Versioning 2.0.0; redefining a cohort produces a new
ref with the prior cohort tombstoned.

## Annex O — Statement attachments

xAPI statements MAY carry attachments (e.g. signed
evidence, candidate response media). The LRS persists
attachments out-of-band and surfaces them via:

```
GET /v1/statements/{statementRef}/attachments/{usageType}
```

Attachment content type is whatever the publisher
declared at submission. Access policy follows the
statement's `actor` consent state.

## Annex P — Profile authority

The profile registry distinguishes between
`registry-published` profiles (signed by the
registry), `community-published` profiles (signed by
their author), and `imported` profiles (cloned from
xAPI Profile Server with the upstream signature
preserved). Consumers MAY filter by authority class
when querying.

## Annex Q0 — Statement signing endpoints

Publishers may submit statements pre-signed by
attaching a JWS attachment with `usageType =
http://adlnet.gov/expapi/attachments/signature`. The
LRS verifies the signature against the publisher's
JWKS before persisting. Statements that fail signature
verification are rejected with a Problem Details
fragment listing the JWS error.

## Annex Q1 — Researcher access endpoint

Researchers retrieve de-identified statement extracts
via:

```
POST /v1/registry/researcher-extract
```

Body: a SHACL shape, a date range, and a project
identifier resolvable in the researcher access
catalogue. Response: a short-lived signed URL to a
`tar.zst` archive of redacted extracts. Audit
records every extraction in the deployment's audit
feed.

## Annex Q — Bulk actor opt-out

For institutional opt-out events (a class graduates,
a programme closes), administrators submit a bulk
opt-out via:

```
POST /v1/actors/bulk-opt-out
```

Body: a list of `actorRef` plus a justification.
The LRS marks each actor's consent withdrawn and
queues a delayed export of historical statements to
each actor's verified contact channel.

## Annex R — Profile dependency resolution

Profiles may declare dependencies on other profiles
(`prefLabel: dependsOn`). The registry resolves
dependencies transitively at lookup time and returns
the closure as a single response. Cyclic
dependencies are forbidden; the registry rejects
profiles whose dependency graph contains a cycle.

## Annex S — Statement state surface

The xAPI State API is exposed at:

```
PUT /v1/activities/state?stateId=<id>&activityId=<id>&agent=<json>
GET /v1/activities/state?stateId=<id>&activityId=<id>&agent=<json>
```

State surfaces are bound to the actor's consent and
are deleted when consent is withdrawn.

## Annex T — Aggregate query endpoint

```
POST /v1/registry/aggregate
```

Body: a SHACL shape over the cohort, a verb URI, and
a date range. Response: aggregate counters (count,
mean, percentiles) honouring the deployment's
k-anonymity floor. Cells with counts below the floor
are surfaced as `< k` rather than the actual value.

弘益人間 (Hongik Ingan) — Benefit All Humanity
