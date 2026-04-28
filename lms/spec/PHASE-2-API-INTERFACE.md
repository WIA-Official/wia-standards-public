# WIA-lms PHASE 2 — API Interface Specification

**Standard:** WIA-lms
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable

This PHASE defines the API surfaces that WIA-lms
participants expose so that institutions, vendors,
LTI tools, content publishers, SIS systems, and
audit authorities can manage courses, enrolments,
content packages, gradebooks, credentials, and
LTI-tool integrations through a single contract.

References (CITATION-POLICY ALLOW only):
- OpenAPI Specification 3.1
- IETF RFC 9110 (HTTP Semantics), RFC 9457 (Problem Details)
- IETF RFC 7515 (JWS), RFC 7519 (JWT)
- IETF RFC 9421 (HTTP Message Signatures)
- IMS LTI 1.3 / Advantage (Deep Linking, NRPS, AGS, Proctoring)
- IMS Common Cartridge 1.3, IMS QTI 3.0, IMS Caliper 1.2
- IMS OneRoster 1.2 (REST and CSV bindings)
- W3C VC 2.0, W3C DID 1.0
- IEEE 1484.20.1 xAPI 1.0.3, IEEE 1484.11.2 CMI

---

## §1 Scope

This PHASE specifies the HTTP-based interfaces
between LMS deployments, SIS systems, LTI tools,
content publishers, credential issuers, and audit
authorities.

## §2 Operation groups

| Prefix              | Group                                           |
|---------------------|-------------------------------------------------|
| `/v1/institutions`  | institution registry                             |
| `/v1/courses`       | course catalogue                                 |
| `/v1/enrolments`    | enrolment lifecycle                              |
| `/v1/content`       | content package upload, retrieval                |
| `/v1/activities`    | activity registry, LTI launch                    |
| `/v1/assessments`   | QTI assessment delivery                          |
| `/v1/gradebook`     | gradebook entries, AGS line items                |
| `/v1/credentials`   | credential issuance, verification                |
| `/v1/tools`         | LTI tool registration                            |
| `/v1/registry`      | registry directory                               |

## §3 Authentication

LTI 1.3 launches use OIDC; service calls use OAuth
2.0 client-credentials with JWT-bearer grants.
OneRoster sync uses OAuth 2.0. Caliper sensors use
Bearer JWT.

## §4 Course operations

### 4.1 Publish course

```
POST /v1/courses
```

Body: course record (PHASE-1 §3) with attached LOM.
Response: `courseRef`.

### 4.2 Import Common Cartridge

```
POST /v1/courses/import
Content-Type: application/vnd.imscc+zip
```

Body: a Common Cartridge 1.3 archive. Response: a
job reference for asynchronous import.

### 4.3 OneRoster sync

```
PUT /v1/courses/oneroster/{sourcedId}
```

Accepts OneRoster 1.2 REST payloads.

## §5 Enrolment operations

### 5.1 Create enrolment

```
POST /v1/enrolments
```

Body: enrolment record (PHASE-1 §4).

### 5.2 Bulk enrolment

```
POST /v1/enrolments/bulk
Content-Type: application/vnd.ims.oneroster+csv
```

Body: OneRoster 1.2 CSV bundle (`users.csv`,
`enrollments.csv`, `classes.csv`).

### 5.3 NRPS (Names and Role Provisioning)

```
GET /v1/courses/{courseRef}/memberships
```

Returns IMS LTI Advantage NRPS 2.0 membership list.

## §6 Content operations

### 6.1 Upload content package

```
POST /v1/content
```

Body: a content package archive with manifest. The
LMS validates the package against its declared
format.

### 6.2 Lookup

```
GET /v1/content/{contentRef}
```

Returns the canonical content record. Conditional
GET via `ETag`.

## §7 Activity operations

### 7.1 Create activity

```
POST /v1/activities
```

Body: activity record (PHASE-1 §6).

### 7.2 LTI launch

```
GET /v1/activities/{activityRef}/lti-launch
```

Returns the LTI 1.3 launch payload with OIDC
parameters.

### 7.3 Deep Linking

```
GET /v1/courses/{courseRef}/deep-linking
```

Initiates an LTI Deep Linking 2.0 round-trip with
the configured tool.

## §8 Assessment operations

### 8.1 Schedule

```
POST /v1/assessments
```

Body: assessment record (PHASE-1 §7).

### 8.2 Deliver QTI test

```
GET /v1/assessments/{assessmentRef}/qti
```

Returns the IMS QTI 3.0 test bound to the
assessment.

### 8.3 Submit response

```
POST /v1/assessments/{assessmentRef}/responses
```

Body: QTI 3.0 result.

## §9 Gradebook operations

### 9.1 Post score

```
POST /v1/gradebook
```

Body: gradebook record (PHASE-1 §8).

### 9.2 AGS line item

```
PUT /v1/courses/{courseRef}/lineitems/{lineItemRef}/scores
```

LTI Advantage AGS 2.0 score endpoint.

### 9.3 Gradebook export

```
GET /v1/courses/{courseRef}/gradebook
```

Returns the course gradebook in JSON or CSV per
`Accept`.

## §10 Credential operations

### 10.1 Issue

```
POST /v1/credentials
```

Body: credential payload per W3C VC 2.0.

### 10.2 Verify

```
POST /v1/credentials/verify
```

Body: a Verifiable Presentation per VC 2.0.

### 10.3 Revoke

```
PUT /v1/credentials/{credentialRef}/revoke
```

Body: justification. Adds the credential to the
issuer's revocation list per W3C VC 2.0
BitstringStatusList profile.

## §11 Tool operations

### 11.1 Register

```
POST /v1/tools
```

Body: integration record (PHASE-1 §10).

### 11.2 Lookup

```
GET /v1/tools/{toolRef}
```

Returns the tool record with current LTI 1.3
parameters.

## §12 Error semantics

Errors are `application/problem+json` (RFC 9457)
namespaced under
`https://wiastandards.com/errors/lms/`.

## §13 Caching and rate limits

Course catalogue endpoints are cached. Gradebook
and credential endpoints are not. Rate-limit
headers follow draft-ietf-httpapi-ratelimit-headers.

## Annex A — OpenAPI 3.1 fragment

```yaml
openapi: 3.1.0
info: {title: WIA-lms API, version: 1.0.0}
paths:
  /v1/courses:
    post:
      summary: Publish a course
      requestBody:
        required: true
        content:
          application/json:
            schema: {$ref: 'CourseRecord.schema.json'}
      responses:
        '201': {description: Course published}
```

## Annex B — Idempotency

Mutating operations honour `Idempotency-Key` for
24h.

## Annex C — Webhook subscriptions

Subscribers receive events on `course.published`,
`enrolment.changed`, `gradebook.posted`,
`credential.issued`, `tool.registered`. Delivery is
signed with HMAC-SHA-256.

## Annex D — Federation

Federation between sister-institution LMSs follows
the discovery contract in PHASE-3. Cross-LMS
queries propagate via federated NRPS 2.0.

## Annex E — Bulk export

`POST /v1/registry/export` returns a signed URL to
a `tar.zst` of the deployment's records filtered
by date range and course.

## Annex F — Sandbox

`/v1/sandbox` mirrors production with synthetic
courses and ephemeral state.

## Annex G — Quotas

Per-deployment quotas: 5,000 enrolments / hour;
50,000 / day.

## Annex H — Audit feed

`GET /v1/registry/audit?since=<timestamp>` returns
mutating-operation events; `audit-feed:read` JWT
scope required.

## Annex I — Public introspection

`GET /v1/registry/stats` returns aggregate
counters.

## Annex J — Reviewer queue

```
GET /v1/assessments/queue?graderRef={ref}
```

Returns the grader's pending queue.

## Annex K — Webhook payload shape

```json
{
  "event": "credential.issued",
  "credentialRef": "https://lms.example.org/credentials/...",
  "learnerRef": "actor-072",
  "issuedAt": "2026-04-28T11:32:00+09:00"
}
```

## Annex L — Bulk credential issuance

```
POST /v1/credentials/batch
Content-Type: application/jsonl
```

Body: a JSON-Lines stream of credentials.

## Annex M — Academic session operations

```
POST /v1/sessions
GET  /v1/sessions/{sessionRef}
```

Submit and retrieve OneRoster academic session
records (terms, semesters, school years).

## Annex N — Mastery transcript endpoints

```
POST /v1/courses/{courseRef}/mastery
GET  /v1/courses/{courseRef}/mastery?learnerRef={ref}
```

Submit and retrieve mastery records bound to
gradebook entries.

## Annex O — Cohort management

```
POST /v1/courses/{courseRef}/cohorts
GET  /v1/courses/{courseRef}/cohorts/{cohortRef}/members
```

Define cohorts via SHACL shapes; resolve membership
on demand.

## Annex P — Tool platform configuration

```
GET /v1/tools/{toolRef}/platform-config
```

Returns the LTI 1.3 platform configuration the tool
should consume during registration: client_id,
deployment_id, deep linking and AGS endpoints.

## Annex Q — Reverse content-package export

```
GET /v1/courses/{courseRef}/cartridge
```

Returns the course as a Common Cartridge 1.3 archive
suitable for re-import at a sister institution.

## Annex R — Privacy export

```
GET /v1/learners/{learnerRef}/export
```

Returns the learner's full data subject access
export per GDPR Article 15 / FERPA / K-PIPA Article
35. Rate-limited.

## Annex S — Calendar publication

```
GET /v1/courses/{courseRef}/calendar.ics
```

Returns the course's events (lectures, due dates,
office hours) as an iCalendar (RFC 5545) feed for
subscription by learners' calendar clients. The
feed is signed (RFC 9421) so that cached copies can
be verified against the publishing LMS.

## Annex T — Module-level navigation

```
GET /v1/courses/{courseRef}/modules
```

Returns the ordered module-and-item navigation
graph (Common Cartridge `imsmanifest.xml`-derived).
Pagination applies once the module count exceeds
100.

## Annex U — Discussion thread surface

```
GET /v1/activities/{activityRef}/threads
POST /v1/activities/{activityRef}/threads
```

Discussion threads bound to discussion-kind
activities. Posts carry `parent`, `author`,
`createdAt` per a thin CRDT-friendly schema.

## Annex V — Per-course caching directives

```
GET /v1/courses/{courseRef}/caching
```

Returns the cache directives for the course's
public surfaces (catalogue page, module list,
read-only roster). Used by edge caches that front
the LMS.

## Annex W — Notification delivery

```
POST /v1/notifications
```

Body: a notification record with target audience
SHACL filter, message, severity. Response: a
notification reference and the queued delivery
schedule. Notification delivery is auditable and
respects the recipient's quiet-hours configuration.

## Annex X — Course copy

```
POST /v1/courses/{srcRef}/copy
```

Body: target term, instructor mapping, content
inclusion filter. Response: a job reference for the
asynchronous copy. Copies preserve content packages
and activity structure but reset enrolments and
gradebook entries.

## Annex Y — Roster diff

```
POST /v1/courses/{courseRef}/roster/diff
```

Body: an external roster snapshot. Response: the
delta against the current LMS roster. Used during
SIS migration to verify no learners are dropped in
the cut-over.

## Annex Z — Activity completion query

```
GET /v1/courses/{courseRef}/completion?learnerRef={ref}
```

Returns per-activity completion state for the
learner: not-started, in-progress, completed,
mastered. Used by external dashboards and by
adaptive-learning recommenders.

## Annex AA — Grading reconciliation

```
POST /v1/courses/{courseRef}/gradebook/reconcile
```

Recomputes course-level grade rollups against the
gradebook entry log. Run after bulk imports or AGS
back-fills to surface drift between rollups and
underlying entries.

## Annex AB — Compliance attestations

```
GET /v1/registry/attestations
```

Returns the deployment's active compliance
attestations (FERPA, GDPR, K-PIPA, COPPA, ISO/IEC
27701) with their effective and expiry dates. Used
by procurement and legal teams during vendor
review.

弘益人間 (Hongik Ingan) — Benefit All Humanity
