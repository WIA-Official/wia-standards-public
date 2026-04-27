# WIA-intent-lang PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-intent-lang
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTPS API contract that an
intent-language operator exposes for the records in PHASE-1.
Consumers include intent authoring tools, planner runtimes,
execution agents, evaluation services, observability platforms,
and the operator's own intent registry analytics.

References (CITATION-POLICY ALLOW only):

- IETF RFC 9110 (HTTP Semantics)
- IETF RFC 9111 (HTTP Caching)
- IETF RFC 9457 (Problem Details)
- IETF RFC 6901 / 6902 (JSON Pointer / Patch)
- IETF RFC 8288 (Web Linking)
- IETF RFC 8259 (JSON)
- IETF RFC 9421 (HTTP Message Signatures)
- ISO 8601 (date and time)
- W3C Trace Context
- OpenAPI Specification 3.1

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published by the operator.
Versioning uses `/v1/` path segments. The OpenAPI 3.1 document
at `/v1/openapi.json` is canonical.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-intent-lang",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "programmes":       "/v1/programmes",
    "intents":          "/v1/intents",
    "predicates":       "/v1/predicates",
    "plans":            "/v1/plans",
    "executionTraces":  "/v1/execution-traces",
    "evaluations":      "/v1/evaluations",
    "grammar":          "/v1/grammar",
    "shacl":            "/v1/shacl",
    "evidence":         "/v1/evidence",
    "openapi":          "/v1/openapi.json"
  }
}
```

## §3 Programme Lifecycle

```
POST   /v1/programmes               — register a programme
GET    /v1/programmes/{pid}         — retrieve programme
PATCH  /v1/programmes/{pid}/status  — advance status
```

## §4 Intent Lifecycle

```
POST   /v1/programmes/{pid}/intents   — register an intent
GET    /v1/intents/{iid}              — retrieve an intent
PATCH  /v1/intents/{iid}/status       — advance status
PATCH  /v1/intents/{iid}/budget       — update budget envelope
                                          while in `draft`
PATCH  /v1/intents/{iid}/withdraw     — withdraw the intent
```

Intent submissions that fail SHACL validation return `422 Un-
processable Entity` with type `urn:wia:intent-lang:shacl-fail`
and a `validationDetails` extension that lists the failing
constraint paths.

## §5 Predicates

```
POST   /v1/intents/{iid}/predicates   — append a predicate
GET    /v1/predicates/{prid}          — retrieve a predicate
POST   /v1/predicates/parse           — parse a predicate
                                          source text and return
                                          the AST without
                                          persisting
```

Parse failures return `422` with type
`urn:wia:intent-lang:parse-fail` with a `position` extension
that points to the error location in the source text.

## §6 Plans

```
POST   /v1/intents/{iid}/plans        — register a plan
GET    /v1/plans/{plid}               — retrieve a plan
PATCH  /v1/plans/{plid}/status        — advance plan status
PATCH  /v1/plans/{plid}/supersede     — supersede a plan with
                                          a successor reference
```

A plan that materialises an intent already at `completed` or
`failed` is rejected with `409 Conflict` and type
`urn:wia:intent-lang:intent-terminal`.

## §7 Execution Traces

```
POST   /v1/plans/{plid}/execution-traces
                                       — open a trace
PATCH  /v1/execution-traces/{tid}/action-events
                                       — append an action event
PATCH  /v1/execution-traces/{tid}/end  — close the trace with
                                          outcome and budget
                                          actuals
GET    /v1/execution-traces/{tid}      — retrieve the trace
```

Action-event submissions that reference a node identifier not
in the plan's DAG return `422` with type
`urn:wia:intent-lang:unknown-action`.

## §8 Evaluations

```
POST   /v1/execution-traces/{tid}/evaluations
                                       — register an evaluation
GET    /v1/evaluations/{evid}          — retrieve an evaluation
GET    /v1/intents/{iid}/evaluation-history
                                       — list evaluations
                                          across the intent's
                                          lineage of plans
```

Evaluations of overall verdict `intent-failed` automatically
trigger an outbound notification to the intent's
`fallbackPolicy` handler through the integration described in
PHASE-4 §6.

## §9 Grammar and SHACL Endpoints

```
GET    /v1/grammar/{minor}             — retrieve the canonical
                                          predicate grammar at
                                          a given minor revision
GET    /v1/shacl/{minor}               — retrieve the SHACL
                                          shapes that govern
                                          intent validation at
                                          a given minor revision
```

Both endpoints return content-addressed artefacts so that
authors can pin the grammar and SHACL revision against which
their authoring tooling was certified.

## §10 Errors

All error responses are `application/problem+json` per RFC 9457.
Defined types include:

- `urn:wia:intent-lang:shacl-fail`
- `urn:wia:intent-lang:parse-fail`
- `urn:wia:intent-lang:intent-terminal`
- `urn:wia:intent-lang:unknown-action`
- `urn:wia:intent-lang:budget-envelope-implausible`
- `urn:wia:intent-lang:evidence-mismatch`

## §11 Authentication and Authorisation

Mutually-authenticated TLS for authoring tools, planner runtimes,
execution agents, and audit consumers. Public read-only access
is provided for grammar and SHACL artefacts; everything else
requires a client certificate that the operator's authorisation
matrix maps to the relevant API scope.

## §12 Caching and Concurrency

Stable resources (immutable grammar / SHACL artefacts, completed
intents and their evaluations, signed evidence packages) are
cacheable with `Cache-Control: max-age=31536000, immutable`.
Mutable resources (in-flight intents and traces) are cacheable
for 60 seconds. ETags are mandatory on every PATCH endpoint.

## §13 Streaming

Consumers subscribe via Server-Sent Events at:

- `/v1/intents/{iid}/events` — intent-scoped events (status
  changes, plan transitions, evaluation outcomes).
- `/v1/programmes/{pid}/events` — programme-wide events
  (budget-envelope alerts, fallback-handler escalations,
  grammar / SHACL refresh notifications).

Subscribers reconnect via the `Last-Event-ID` header (W3C
EventSource semantics).

## §14 Worked Example: Author to Evaluation

1. An author submits an intent draft with goal, budget,
   pre/post-conditions, and acceptance criteria.
2. The operator's linter validates the intent against SHACL
   and returns a successful response on first attempt or a
   problem document with constraint failures.
3. The author advances the intent to `approved` and the
   programme's planner materialises a plan.
4. The execution agent opens a trace, appends action events as
   it executes the plan's DAG, and closes the trace with the
   outcome and actual budget consumed.
5. The evaluator scores the trace against the intent's
   acceptance criteria and emits an evaluation record.

## §15 Observation Artefact and Compensation Endpoints

```
POST   /v1/execution-traces/{tid}/observation-artefacts
                                       — register an observation
                                          artefact reference
GET    /v1/observation-artefacts/{oid} — retrieve artefact
                                          metadata
POST   /v1/execution-traces/{tid}/compensations
                                       — register a compensation
                                          record
GET    /v1/compensations/{cmpid}       — retrieve compensation
```

Observation-artefact submissions whose `privacyClass` is
`subject-restricted` or `regulator-restricted` are gated by the
operator's per-class authorisation matrix; non-authorised
fetches return `403 Forbidden` with type
`urn:wia:intent-lang:artefact-class-restricted`.

## §16 Bulk Operations and Pagination

Bulk endpoints accept arrays for high-volume action-event
streaming and observation-artefact ingest:

```
POST   /v1/bulk/action-events             — batched action events
POST   /v1/bulk/observation-artefacts     — batched artefacts
GET    /v1/bulk/{operationId}             — operation status
```

Cursor-based pagination uses the `cursor` query parameter and
`Link` headers (RFC 8288) with cursors persisted for at least
24 hours.

## §17 Provenance Endpoint

```
GET    /v1/provenance/{recordId}    — retrieve provenance entry
                                       for any PHASE-1 record
```

Provenance entries trace a record to its parents (programme,
intent, plan, trace, evaluation, artefact, compensation) so
that audit reviewers can walk the chain end-to-end without
bespoke joins.

## §18 Privacy-Preserving Aggregation

Aggregate consumers (research collaboratives, programme
benchmarking services, public-policy analysts) fetch
population-level statistics through aggregation endpoints:

```
GET    /v1/aggregate/intent-success-rate?domain=...&period=...
GET    /v1/aggregate/budget-overrun-rate?domain=...&period=...
GET    /v1/aggregate/fallback-trigger-rate?policy=...&period=...
```

Out-of-policy queries (cohort below threshold, per-author detail
requested without authorisation) return `403 Forbidden` with
type `urn:wia:intent-lang:cohort-too-small`.

## §19 Reviewer and Approver Endpoints

```
POST   /v1/intents/{iid}/review        — submit an intent
                                          review (reviewer
                                          authorisation
                                          required)
POST   /v1/intents/{iid}/approve       — submit an intent
                                          approval (approver
                                          authorisation
                                          required)
GET    /v1/intents/{iid}/review-chain  — retrieve the review-
                                          and-approve chain for
                                          the intent
```

Approval submissions for an intent that already holds an
approver-of-record return `409 Conflict` with type
`urn:wia:intent-lang:already-approved`; reviewer or approver
self-approvals violating segregation-of-duties return `403
Forbidden` with type
`urn:wia:intent-lang:segregation-of-duties`.

## §20 Re-Planning Endpoint

```
POST   /v1/intents/{iid}/replan        — request a fresh plan
                                          from a successor
                                          planner; the prior
                                          plan is superseded
                                          and remains
                                          addressable for
                                          retrospective audit
GET    /v1/intents/{iid}/plan-history  — list plan revisions
                                          across the intent's
                                          lifecycle
```

Re-plan requests for an intent at terminal status (`completed`,
`failed`, `withdrawn`) return `409 Conflict` with type
`urn:wia:intent-lang:intent-terminal`.

## §21 Conformance

A conformant server passes the test vectors published under
`tests/phase-vectors/phase-2-api-interface/`, emits an OpenAPI
3.1 document, signs evidence packages per RFC 9421, and rejects
intents that fail SHACL validation before persisting them.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-intent-lang
- **Last Updated:** 2026-04-28
