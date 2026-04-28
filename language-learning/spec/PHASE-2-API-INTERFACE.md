# WIA-language-learning PHASE 2 — API Interface Specification

**Standard:** WIA-language-learning
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable

This PHASE defines the API surfaces that WIA-language-
learning participants expose so that providers,
sovereign testing authorities, learning management
systems, employers, and credential verifiers can
exchange enrolments, sessions, assessments, scores,
accommodations, and credentials through a single
contract.

References (CITATION-POLICY ALLOW only):
- OpenAPI Specification 3.1
- IETF RFC 9110 (HTTP Semantics), RFC 9457 (Problem Details)
- IETF RFC 7515 (JWS), RFC 7519 (JWT)
- IEEE 1484.20.1 (xAPI), IMS Caliper 1.2
- IMS LTI 1.3 / Advantage, IMS OneRoster 1.2, IMS QTI 3.0
- IMS Open Badges 3.0
- W3C Verifiable Credentials Data Model 2.0
- W3C Decentralized Identifiers 1.0
- ISO/IEC 23988 (Assessment delivery)

---

## §1 Scope

This PHASE specifies the HTTP-based interfaces between
language-learning providers, sovereign testing
authorities, learning management systems (via LTI
1.3), and credential verifiers. The PHASE does not
specify pedagogy or item-bank content; those remain
provider-internal.

## §2 Operation groups

| Prefix              | Group                                           |
|---------------------|-------------------------------------------------|
| `/v1/learners`      | learner record management                        |
| `/v1/courses`       | course catalogue                                 |
| `/v1/sessions`      | session lifecycle and attendance                 |
| `/v1/assessments`   | assessment delivery                              |
| `/v1/scores`        | score issuance and lookup                        |
| `/v1/credentials`   | credential issuance and verification             |
| `/v1/accommodations`| accommodation request and approval               |
| `/v1/registry`      | registry directory                               |

## §3 Authentication

Read endpoints accept anonymous or JWT-bearer access
per the deployment's policy. Write endpoints require
a JWT bound to a provider, sovereign authority, or
learner identity. High-stakes assessment endpoints
also require a remote-proctoring identity assertion.

## §4 Learner operations

### 4.1 Register learner

```
POST /v1/learners
```

Body: learner record (PHASE-1 §2). Response:
`learnerRef`. Personal data minimisation rules apply;
the registry rejects records containing fields not
declared in PHASE-1 §2.

### 4.2 Lookup

```
GET /v1/learners/{learnerRef}
```

Returns the canonical learner record. Field exposure
follows the requesting principal's role (learner self,
guardian, instructor, auditor).

## §5 Course operations

### 5.1 Publish course

```
POST /v1/courses
```

Body: course record (PHASE-1 §4) with attached LOM
(IEEE 1484.12.1) JSON.

### 5.2 Search

```
GET /v1/courses?targetLanguage=ko&level=B2&mode=online-async
```

Returns courses matching the filter.

### 5.3 LTI launch

```
GET /v1/courses/{courseRef}/lti-launch
```

Returns an LTI 1.3 launch payload bound to the
requesting LMS context.

## §6 Session operations

### 6.1 Open session

```
POST /v1/sessions
```

Body: session record (PHASE-1 §5) excluding `endTime`.

### 6.2 Append xAPI / Caliper

```
POST /v1/sessions/{sessionRef}/xapi
POST /v1/sessions/{sessionRef}/caliper
```

Append xAPI statements or Caliper envelopes.

### 6.3 Close session

```
PUT /v1/sessions/{sessionRef}/close
```

Body: `endTime` and final attendance state.

## §7 Assessment operations

### 7.1 Schedule assessment

```
POST /v1/assessments
```

Body: assessment record (PHASE-1 §6) without
`assessmentDate`. Response: scheduled date and
delivery channel (proctored seat, remote proctor,
self-delivery URL).

### 7.2 Deliver QTI test

```
GET /v1/assessments/{assessmentRef}/qti
```

Returns the IMS QTI 3.0 test bound to the assessment.
Items are randomised per the test's blueprint.

### 7.3 Submit response

```
POST /v1/assessments/{assessmentRef}/responses
```

Body: QTI 3.0 result XML / JSON.

## §8 Score operations

### 8.1 Issue score

```
POST /v1/scores
```

Body: score record (PHASE-1 §7) signed by the rater(s)
and, for high-stakes, the issuing authority.

### 8.2 Lookup

```
GET /v1/scores/{scoreRef}
```

Returns the canonical score record.

### 8.3 Re-rate request

```
POST /v1/scores/{scoreRef}/re-rate
```

Body: justification and additional rater scope.
Response: a new `scoreRef` referencing the original.

## §9 Credential operations

### 9.1 Issue

```
POST /v1/credentials
```

Body: credential payload per W3C VC 2.0. Response:
`credentialRef` plus the verification URL.

### 9.2 Verify

```
POST /v1/credentials/verify
```

Body: a Verifiable Presentation per VC 2.0. Response:
the verification verdict, the issuer DID resolution,
and the proof check.

### 9.3 Revoke

```
PUT /v1/credentials/{credentialRef}/revoke
```

Body: justification. The registry adds the credential
to the issuer's revocation list per W3C VC 2.0
revocation profile.

## §10 Accommodation operations

### 10.1 Request

```
POST /v1/accommodations
```

Body: accommodation record (PHASE-1 §8) with evidence
URI.

### 10.2 Approve

```
PUT /v1/accommodations/{accommodationRef}/approve
```

Body: approving authority and validity window.

## §11 Error semantics

Errors are `application/problem+json` (RFC 9457)
namespaced under
`https://wiastandards.com/errors/language-learning/`.

## §12 Caching and rate limits

Course catalogue and credential verification endpoints
are cached. Score and assessment endpoints are not.

## Annex A — OpenAPI 3.1 fragment

```yaml
openapi: 3.1.0
info: {title: WIA-language-learning API, version: 1.0.0}
paths:
  /v1/scores:
    post:
      summary: Issue a score
      requestBody:
        required: true
        content:
          application/json:
            schema: {$ref: 'ScoreRecord.schema.json'}
      responses:
        '201': {description: Score issued}
```

## Annex B — Idempotency

Mutating operations honour `Idempotency-Key`. Results
are persisted for 24h.

## Annex C — Webhook subscriptions

Subscribers receive events on `assessment.completed`,
`score.issued`, `credential.issued`,
`credential.revoked`, `accommodation.approved`.
Delivery is signed with HMAC-SHA-256.

## Annex D — LTI 1.3 / Advantage compatibility

The course catalogue exposes LTI 1.3 platform metadata
under `/v1/courses/.well-known/lti`. Deep Linking 2.0,
Names and Role Provisioning Services 2.0, and
Assignment and Grade Services 2.0 are supported.

## Annex E — OneRoster compatibility

Class-roster sync follows IMS OneRoster 1.2 with the
`enrollments`, `users`, `classes`, `courses` resources
mapped to PHASE-1 §2..§4 records.

## Annex F — Public introspection

`GET /v1/registry/stats` returns aggregate counters
(learner count, supported tags, supported frameworks,
high-stakes assessments per quarter). Counters are
eventually consistent.

## Annex G — Bulk export

`POST /v1/registry/export` returns a signed URL
referencing a `tar.zst` of the deployment's records
filtered by date range and framework. PII is redacted
for non-authoritative requesters.

## Annex H — Sandbox endpoints

`/v1/sandbox` mirrors the production surface with
synthetic learners, fictional levels, and ephemeral
state. Sandbox responses carry `X-WIA-Sandbox: true`.
Sandbox state clears on a 24h rolling window.

## Annex I — Quotas

Per-provider publish quotas default to 5,000 score
issuances per hour and 100,000 per day. The registry
surfaces remaining quota in `X-Quota-Remaining`.

## Annex J — Webhook payload shape

```json
{
  "event": "score.issued",
  "scoreRef": "f63f4f04-...",
  "issuedAt": "2026-04-28T11:32:00+09:00",
  "framework": "CEFR",
  "level": "B2"
}
```

## Annex K — Score reissuance

A score MAY be reissued when a clerical error is
discovered, when a re-rate panel revises the original
verdict, or when a sovereign authority compels
correction. Reissuance creates a new `scoreRef` that
references the original via the `supersedes` field.
The original is tombstoned but remains
cryptographically verifiable so that historical
verifiers can resolve the chain. Reissuance windows
are bounded by the framework's authoritative
appeal-window rules — typically 90 days for CEFR-
aligned high-stakes assessments and 60 days for ACTFL.

## Annex L — Item-bank exposure control

The QTI test endpoint serves items only within the
declared assessment window. After the window closes,
items are eligible for retirement to the public
practice bank or hold-back to the secure bank per
the test publisher's policy. Item exposure metrics
are published in the registry's audit feed but not
bound to individual candidates.

## Annex M — Roster reconciliation

OneRoster 1.2 sync is reconciled nightly. The
registry surfaces drift (mismatched class membership,
stale enrolment) in the `/v1/registry/onerost/drift`
endpoint so that LMS owners can act before high-
stakes events.

## Annex N — Bulk credential issuance

Issuers MAY submit a batch of credentials in a single
request:

```
POST /v1/credentials/batch
Content-Type: application/jsonl
```

Body: a JSON-Lines stream of credential payloads.
Response: a per-line verdict with the issued
`credentialRef` or a Problem Details fragment.

## Annex O — Sovereign authority registration

Sovereign testing authorities register their issuer
DID and signing key set via:

```
POST /v1/registry/authorities
```

Body: authority record with declared frameworks,
working language pairs, and a verified domain
attestation (DNS TXT or ACME-bound HTTPS) tying the
DID to the authority's public domain. Registration is
gated by the registry's authority-vetting committee;
a self-asserted authority appears as `claimed` until
the committee promotes it to `verified`.

## Annex P — Re-examination and retake policy

Each framework declares its retake policy in the
publisher's metadata:

| Framework | Min wait | Max attempts / 12mo |
|-----------|----------|----------------------|
| CEFR (Goethe) | 21 days | typically 3      |
| ACTFL OPI     | 60 days | 4                |
| IELTS         | none    | unlimited        |
| TOPIK         | 60 days | varies by edition|
| ICAO ELP      | sovereign | typically 2    |

Retake records carry a `retakeOf` reference linking
each attempt to the candidate's prior attempts.

## Annex Q — Audit trail introspection

```
GET /v1/registry/audit?since=<timestamp>
```

Returns the audit feed of mutating operations across
the deployment within the requested window. Auditors
authenticate with a JWT scoped to the
`audit-feed:read` permission.

## Annex R — Anonymous practice runs

Self-study learners MAY run practice assessments
without a learner record. The practice endpoint
issues an ephemeral session token bound to the
device fingerprint; no personal data is collected;
practice scores are not surfaced as authoritative
proficiency claims.

## Annex S0 — Webhook delivery retry policy

Webhook deliveries follow at-least-once semantics with
exponential backoff (2, 4, 8, 16, 32, 64, 128, 256,
512 seconds) capped at 9 attempts. After exhaustion
the delivery enters a dead-letter queue inspectable
at `/v1/registry/webhooks/{id}/deliveries/dead`.

## Annex S — Item statistics feed

The registry publishes anonymised item statistics
(p-value, point-biserial discrimination, time-on-
item) to assessment authorities under a separate
`item-stats:read` JWT scope. The feed supports
psychometric calibration without exposing
candidate-identifying response data.

弘益人間 (Hongik Ingan) — Benefit All Humanity
