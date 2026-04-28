# WIA-master-data-management PHASE 2 — API Interface Specification

**Standard:** WIA-master-data-management
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable

This PHASE defines the resource-oriented API surface
for master-data management operations: domain-record
ingest and retrieval, golden-record construction,
match-cluster review, hierarchy management,
stewardship-task workflow, change-history retrieval,
quality-rule registration, and downstream consumer
publication. The API supports two architectural
patterns — registry-style (read-aside) and
transactional (read-and-write) — and a coexistence
profile that bridges them.

References (CITATION-POLICY ALLOW only):
- IETF RFC 9110 (HTTP), RFC 9112 (HTTP/1.1), RFC 9113 (HTTP/2)
- IETF RFC 9457 (Problem Details for HTTP APIs)
- IETF RFC 8259 (JSON), RFC 8785 (JCS), RFC 4122 (UUID), RFC 9530 (Content-Digest)
- IETF RFC 5789 (PATCH), RFC 6902 (JSON Patch), RFC 7396 (JSON Merge Patch)
- HL7 SMART App Launch 2.0 (clinical contexts where party = Patient)
- HL7 FHIR R5 — Patient, Practitioner, Organization, Location, Substance
- W3C SHACL (validation), W3C SKOS (controlled vocabulary)
- ISO/IEC 11179-3 — metadata-registry registry metamodel

---

## §1 Endpoint root

API root is implementation-controlled. All endpoints
are TLS 1.3 (RFC 8446). For clinical contexts the API
participates in SMART on FHIR launch flows; for
sponsor-to-sponsor machine integrations the API uses
client_credentials with key attestation.

## §2 Domain endpoints

For each domain `{d}` ∈ {parties, products, locations,
assets, accounts, reference-sets} the API exposes:

```
POST   /v1/{d}                          create record
GET    /v1/{d}/{ref}                    retrieve current state
PATCH  /v1/{d}/{ref}                    JSON Patch (RFC 6902) or Merge Patch
DELETE /v1/{d}/{ref}                    soft delete (status = retired)
GET    /v1/{d}                          list / filter
GET    /v1/{d}/{ref}/history            change-history (paged)
GET    /v1/{d}/{ref}/sources            source-records contributing
```

`PATCH` requests carry an `If-Match: <etag>` header to
guard against lost updates; the server rejects with
`412 Precondition Failed` on stale.

## §3 Golden-record endpoints

```
POST   /v1/golden-records/$rebuild      trigger rebuild for a domain or cluster
GET    /v1/golden-records/{ref}         retrieve
GET    /v1/golden-records/{ref}/lineage source-record graph
PATCH  /v1/golden-records/{ref}         per-attribute survivorship override
                                       (steward-only, audited)
```

Survivorship overrides are recorded as steward actions
on the change-history; they cannot be silently mutated.

## §4 Match-cluster endpoints

```
POST   /v1/match-clusters/$run          run / re-run match for a domain
GET    /v1/match-clusters/{ref}         retrieve
POST   /v1/match-clusters/{ref}/merge   apply merge (steward decision)
POST   /v1/match-clusters/{ref}/split   apply split (steward decision)
POST   /v1/match-clusters/{ref}/hold    hold for further review
GET    /v1/match-clusters?status=       list / filter
```

Merge / split actions emit audit events and re-run the
golden-record build for the affected cluster.

## §5 Hierarchy endpoints

```
POST   /v1/hierarchies                  create hierarchy version
GET    /v1/hierarchies/{ref}            retrieve current version
GET    /v1/hierarchies/{ref}/version/{v} retrieve effective-dated version
POST   /v1/hierarchies/{ref}/promote    promote draft to active
GET    /v1/hierarchies/{ref}/walk       traverse (DFS / BFS, depth limit)
```

Hierarchy versions are effective-dated (PHASE 1 §10);
the active version at a given date is resolvable via
`?at=<ISO-8601>`.

## §6 Stewardship-task endpoints

```
POST   /v1/tasks                        create steward task
GET    /v1/tasks/{ref}                  retrieve
PATCH  /v1/tasks/{ref}                  update status / assignment
POST   /v1/tasks/{ref}/comment          append steward comment
GET    /v1/tasks?assignee=&priority=    list / filter
```

## §7 Quality-rule endpoints

```
POST   /v1/quality-rules                register rule
GET    /v1/quality-rules/{ref}          retrieve
PATCH  /v1/quality-rules/{ref}          amend (active / retired)
POST   /v1/quality-rules/$evaluate      evaluate corpus (async)
GET    /v1/quality-rules/$evaluate/{run-id} retrieve evaluation result
```

Evaluations return per-rule violations bound to the
record references that fail; violations open
stewardship tasks per the configured automation.

## §8 Reference-data endpoints

```
POST   /v1/reference-sets                       register reference set
GET    /v1/reference-sets/{ref}                  retrieve set metadata
GET    /v1/reference-sets/{ref}/codes            list codes
GET    /v1/reference-sets/{ref}/mappings/{tgtSet} cross-walk to another set
```

Cross-walk mappings expose SKOS exact / close / broad /
narrow / related match relations; consumers select the
appropriate semantic.

## §9 Bulk export

```
GET  /v1/$export?_type=Party,Product,Location,Asset,Account
GET  /v1/$status/{exportId}
GET  /v1/$result/{exportId}/{file}
```

Export output is NDJSON. For clinical-context
implementations the FHIR Bulk Data profile is the
authoritative export channel for `Patient`,
`Organization`, `Practitioner`, and `Location`.

## §10 Error model (RFC 9457)

```json
{
  "type":   "urn:wia:mdm:problem:lost-update",
  "title":  "Optimistic concurrency violation",
  "status": 412,
  "detail": "If-Match etag does not match current resource version",
  "instance": "/v1/parties/p-2026-001"
}
```

Common type URIs:

| Type URI suffix              | HTTP | Meaning                                       |
|------------------------------|-----:|-----------------------------------------------|
| `lost-update`                | 412  | etag-precondition mismatch                    |
| `quality-rule-violation`     | 422  | record fails an active critical rule          |
| `cluster-locked`             | 409  | match-cluster under steward review            |
| `survivorship-override-only-steward` | 403 | non-steward attempted survivorship override |
| `hierarchy-cycle`            | 422  | hierarchy modification would create a cycle   |
| `reference-deprecated`       | 410  | reference-data code retired                   |

## §11 Pagination

Cursor-paginated list endpoints return:

```json
{
  "items": [...],
  "nextCursor": "eyJsYXN0Ijoi..."
}
```

## §12 Audit headers

| Header                  | Meaning                                       |
|-------------------------|-----------------------------------------------|
| `X-Request-Id`          | client-set, echoed by server                  |
| `X-Audit-Event-Id`      | server-set, links to PHASE 3 audit chain      |
| `X-Trace-Id`            | W3C Trace Context (`traceparent`)             |
| `Content-Digest`        | RFC 9530 SHA-256 of the response body         |
| `ETag`                  | weak (W/"…") version tag for optimistic        |
|                         | concurrency                                   |

## §13 Versioning

Resource paths are version-prefixed (`/v1/...`).

## §14 Authentication and scopes (informative)

For sponsor-to-sponsor machine flows:

```
mdm:party.read
mdm:party.write
mdm:product.write
mdm:cluster.review
mdm:hierarchy.promote
mdm:steward.assign
mdm:quality-rule.evaluate
mdm:export.read
```

Steward actions (cluster merge / split, survivorship
override, hierarchy promote) require scopes elevated
beyond standard write.

## Annex A — OpenAPI reference

A canonical OpenAPI 3.1 description is published at
`api/openapi-3.1.yaml`.

## Annex B — Worked merge action (informative)

```http
POST /v1/match-clusters/c-2026-04-12-007/merge HTTP/1.1
Authorization: Bearer ...
WIA-MDM-Schema-Version: 1.0
Idempotency-Key: 7c0d9b0f-...

{
  "stewardRef": "steward:KE-007",
  "rationale":  "Confirmed identity via LEI 5493001RKR55V4X61F71",
  "decision":   "merge",
  "survivorshipOverride": {"legalName": "Acme Inc."}
}
```

Response 202 returns the queued rebuild identifier so
the caller can poll for golden-record refresh.

## Annex C — Async export pattern

```
POST   /v1/$export                      → 202 with Content-Location
GET    /v1/$status/{id}                 → 202 in-progress / 200 manifest
GET    /v1/$result/{id}/{file}          → 200 NDJSON
DELETE /v1/$status/{id}                 → 202 cancellation
```

## Annex D — Conformance disclosure

Implementations declare the OpenAPI revision served,
the SHACL shape catalogue version, the FHIR R5 IG
profiles supported (where applicable), and the
canonicalisation form (RFC 8785).

## Annex E — Bitemporal query parameters

Domain endpoints support bitemporal lookups:

```
GET /v1/parties/{ref}?validAt=2026-01-15T00:00:00Z
GET /v1/parties/{ref}?recordedAt=2026-04-12T09:14:00Z
GET /v1/parties/{ref}?validAt=...&recordedAt=...
```

`validAt` reconstructs the modelled fact at a point in
real-world time; `recordedAt` reconstructs the system's
belief at a point in record time. Combined queries
support audit reconstructions ("what did the system
know on date X about the state of the world on date Y").

## Annex F — Subject-rights endpoints

For privacy-rights operations (PHASE 4 §6) the API
exposes:

```
GET    /v1/subject-rights/{partyRef}/access      data subject access export
POST   /v1/subject-rights/{partyRef}/rectification  request rectification
POST   /v1/subject-rights/{partyRef}/erasure     request erasure (GDPR Art. 17)
POST   /v1/subject-rights/{partyRef}/portability portable export
POST   /v1/subject-rights/{partyRef}/restriction restrict processing
POST   /v1/subject-rights/{partyRef}/objection   record objection
GET    /v1/subject-rights/{partyRef}/queue       open requests for this party
```

Subject-rights endpoints emit dedicated audit events
gated by an elevated steward scope (`mdm:subject-
rights.handle`).

## Annex G — Webhook surface

Implementations expose webhooks for `golden-record-
updated`, `cluster-merged`, `cluster-split`, `quality-
violation`, and `subject-rights-resolved` events.
Payloads sign with RFC 7515 JWS; receivers verify
against `/.well-known/wia-mdm-keys.json`. Delivery is
at-least-once; receivers are expected to be idempotent
on `eventId`. Webhook receivers may register a content-
filter expression (SQL-like predicate) so the
implementation only delivers matching events.
