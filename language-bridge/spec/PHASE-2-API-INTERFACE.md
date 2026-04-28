# WIA-language-bridge PHASE 2 — API Interface Specification

**Standard:** WIA-language-bridge
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable

This PHASE defines the API surfaces that WIA-language-
bridge participants expose so that authoring tools,
content management systems, machine-translation
providers, language service providers, and audit
authorities can submit segments, retrieve translations,
manage glossaries and translation memory, run quality
reviews, and reconcile interpretation sessions through
a single contract.

References (CITATION-POLICY ALLOW only):
- OpenAPI Specification 3.1
- IETF RFC 9110 (HTTP Semantics), RFC 9457 (Problem Details)
- IETF RFC 7515 (JWS), RFC 7519 (JWT), RFC 9421 (HTTP Message Signatures)
- IETF RFC 9530 (Digest Fields)
- OASIS XLIFF 2.1, TBX 30042:2019, TMX 1.4b
- W3C ITS 2.0 (Internationalization Tag Set)
- ISO 17100:2015, ISO 18587:2017, ISO 18841:2018
- HL7 FHIR R5 Communication / CommunicationRequest

---

## §1 Scope

This PHASE specifies the HTTP-based interfaces between
content owners, translation/interpretation providers,
language-resource registries, and conformance auditors.
The PHASE does not specify the in-memory translation
algorithm or the streaming wire format used by remote
simultaneous interpreting (RSI) platforms; those are
covered in PHASE-3.

## §2 Operation groups

| Prefix              | Group                                           |
|---------------------|-------------------------------------------------|
| `/v1/segments`      | source / target segment ingest and retrieval     |
| `/v1/jobs`          | job orchestration (translation, post-editing,    |
|                     | interpretation booking)                          |
| `/v1/practitioners` | translator / interpreter credential register     |
| `/v1/tm`            | translation memory upload, query                 |
| `/v1/glossaries`    | terminology base management (TBX)                |
| `/v1/quality`       | MQM / DQF quality measurement                    |
| `/v1/sessions`      | interpretation session lifecycle                 |
| `/v1/registry`      | registry directory + identity                    |

## §3 Authentication

Read endpoints accept anonymous or bearer-token access
per the publisher's policy. Write endpoints require a
JWT bearer (RFC 7519) bound to the publisher's
practitioner identity or to a corporate language-
service identity.

## §4 Segment operations

### 4.1 Submit source

```
POST /v1/segments
Content-Type: application/json
```

Body: a source segment record (PHASE-1 §4). Response:
the assigned `segmentRef` and the conformance state
of the BCP 47 tag.

### 4.2 Submit target

```
POST /v1/segments/{segmentRef}/targets
```

Body: a target segment record (PHASE-1 §5). Response:
`targetSegmentRef`, plus a Problem Details (RFC 9457)
report listing any TBX violations or ITS conformance
warnings.

### 4.3 Search

```
GET /v1/segments?text=<q>&sourceTag=<bcp47>&domainRef=<ontology>
```

Returns segments matching the query. Pagination is
mandatory beyond 100 results.

## §5 Job operations

### 5.1 Create job

```
POST /v1/jobs
```

Body fields: `sourceTag`, `targetTag`, `domainRef`,
`deadline`, `qualityFramework`, `volume` (chars or
words), `mode` (`translation`, `post-editing`,
`interpretation`).

Response: `jobRef`, the auto-matched practitioner pool,
and an estimated price band (informative).

### 5.2 Assign practitioner

```
PUT /v1/jobs/{jobRef}/practitioner
```

Body: `practitionerRef`. Response: confirmation,
including an ISO 17100 §3.1.4 competency check
verdict.

### 5.3 Job status

```
GET /v1/jobs/{jobRef}
```

Returns lifecycle state: `created`, `assigned`,
`in-progress`, `delivered`, `accepted`, `rejected`,
`closed`. Each transition carries a timestamp and a
practitioner attribution.

## §6 Practitioner operations

### 6.1 Register

```
POST /v1/practitioners
```

Body: practitioner record (PHASE-1 §3) including
sovereign certifications. The registry verifies each
certificate ID against the issuing authority where an
authoritative API is available; otherwise the entry is
recorded as `claimed`.

### 6.2 Lookup

```
GET /v1/practitioners?qualifications=ATA&pair=en-US:ko-KR
```

Returns practitioners whose declared `workingPair`
includes the pair and whose qualifications match the
filter.

## §7 TM operations

### 7.1 Upload

```
POST /v1/tm
Content-Type: application/x-tmx+xml
```

Body: TMX 1.4b document. The registry verifies XML
schema and rejects malformed `<srclang>` declarations.

### 7.2 Concordance search

```
POST /v1/tm/{tmxRef}/concordance
```

Body: source segment text, fuzzy threshold (0..100).
Response: ranked TM matches with similarity score and
provenance.

## §8 Glossary operations

### 8.1 Upload

```
POST /v1/glossaries
Content-Type: application/x-tbx+xml
```

Body: TBX-Basic document per ISO 30042:2019.

### 8.2 Concept lookup

```
GET /v1/glossaries/{tbxRef}/concepts/{conceptId}
```

Returns the concept entry with all language sub-entries.

### 8.3 Concordance against segment

```
POST /v1/glossaries/{tbxRef}/check
```

Body: a target segment. Response: the list of TBX
preferred terms that should have been used and the
deprecated terms detected.

## §9 Quality operations

### 9.1 Submit measurement

```
POST /v1/quality
```

Body: quality measurement record (PHASE-1 §8).
Response: the registry's verdict against the declared
job acceptance threshold.

### 9.2 Reviewer queue

```
GET /v1/quality/queue?reviewerRef={ref}
```

Returns the reviewer's pending queue with deadlines.

## §10 Session operations

### 10.1 Open session

```
POST /v1/sessions
```

Body: session record (PHASE-1 §9) excluding `endTime`.
Response: assigned `sessionRef` and rotation schedule.

### 10.2 Append events

```
POST /v1/sessions/{sessionRef}/events
```

Body: a list of session events (rotation, technical
issue, comfort break, term lookup). Events are
append-only.

### 10.3 Close session

```
PUT /v1/sessions/{sessionRef}/close
```

Body: `endTime`, audio archive URI, and per-practitioner
billable minutes.

## §11 Error semantics

Errors are `application/problem+json` (RFC 9457). Type
URLs are namespaced under
`https://wiastandards.com/errors/language-bridge/`.

## §12 Caching and rate limits

Read endpoints carry strong `ETag` and
`Cache-Control: public, max-age=300, stale-while-
revalidate=60`. Concordance and quality endpoints are
not cached. Rate-limit headers follow draft-ietf-
httpapi-ratelimit-headers convention.

## Annex A — OpenAPI 3.1 fragment

```yaml
openapi: 3.1.0
info:
  title: WIA-language-bridge API
  version: 1.0.0
paths:
  /v1/jobs:
    post:
      summary: Create a translation or interpretation job
      requestBody:
        required: true
        content:
          application/json:
            schema: {$ref: 'JobCreate.schema.json'}
      responses:
        '201':
          description: Job created
          content:
            application/json:
              schema: {$ref: 'JobRecord.schema.json'}
```

## Annex B — Federation

Federation between language-service registries follows
the discovery contract in PHASE-3. Cross-registry
queries carry an `X-WIA-Federation-Path` header.

## Annex C — Idempotency

Mutating operations honour `Idempotency-Key`; the
registry persists results for 24h to allow retry without
duplicate creation.

## Annex D — Bulk submission

For large translation batches, publishers MAY submit a
manifest URL referencing a tarball of XLIFF files and
TM/TBX accompaniments. The registry returns a 202
Accepted with a polling URL.

## Annex E — Webhook subscriptions

Subscribers receive events on `job.assigned`,
`job.delivered`, `quality.failed`, `session.opened`,
`session.closed`. Delivery is signed with HMAC-SHA-256
in `X-WIA-Signature: sha256=<hex>`.

## Annex F — FHIR Communication binding

When the bridge is invoked in healthcare contexts, the
job request MAY be sourced from a FHIR R5
`CommunicationRequest`; the response MAY be returned as
a `Communication` resource. The mapping is informative
and is published at `/v1/registry/fhir-mapping`.

## Annex G — XLIFF round-trip

A job that ingests XLIFF 2.1 input MUST round-trip the
file: every `<unit>` in the input appears in the output
even if untranslated, with the `state` attribute set to
`translated`, `reviewed`, or `final` per ISO 17100
process step.

## Annex H — Public introspection

`GET /v1/registry/stats` returns aggregate counters
(practitioner count, working-pair coverage, MT engine
list, federation peer count). Counters are eventually
consistent.

## Annex I — Working-pair coverage report

```
GET /v1/registry/coverage?source=<bcp47>&target=<bcp47>
```

Returns the practitioner pool serving the pair, the
median time-to-acceptance for new jobs, and the
declared TM and TBX coverage for the pair. The report
is informative and intended for capacity planning;
under-served pairs are surfaced as a discoverability
hint to language service providers.

## Annex J — Quality reviewer pool

```
GET /v1/practitioners?reviewerOnly=true&framework=MQM2.0
```

Returns practitioners declared as MQM 2.0 reviewers.
The PHASE-1 §8 separation-of-duty rule applies: a
reviewer assigned to a job MUST NOT have produced
target segments for the same job.

## Annex K — Bulk export

```
POST /v1/registry/export
```

Body: a filter over jobs (deployment, date range,
language pair, framework). Response: a short-lived
signed URL pointing to a `tar.zst` containing one
folder per matching job (`source.xliff`,
`target.xliff`, `tm.tmx`, `tbx.tbx`,
`quality.json`, `audit.json`). Audio archives are
fetched separately to keep the export manageable.

## Annex L — Sandbox endpoints

`/v1/sandbox` mirrors the production surface with
synthetic practitioners, fictional language tags, and
ephemeral state. Sandbox responses carry
`X-WIA-Sandbox: true`. Sandbox state is cleared on a
24h rolling window.

## Annex M — Audio archive endpoints

```
GET /v1/sessions/{sessionRef}/audio
```

Returns the session's audio archive when the requester
is the owning deployment, the assigned reviewer, or an
auditor with a JWT scoped to the session's
`auditorRole`. Other requesters receive 403. The audio
is served in Opus by default with a `Content-Disposition`
attachment hint so that downloads are explicit, not
incidental from a casual browser preview.

## Annex N — Glossary versioning

Glossaries are versioned with Semantic Versioning 2.0.0.
A minor bump permits new concepts; a major bump is
required when an existing concept's preferred term
changes language coverage. The registry preserves
prior major versions for at least one calendar year so
that historical jobs reference the glossary version
that was active at job creation.

## Annex O — Quotas

Per-deployment publish quotas default to 10,000
segments per hour and 1,000,000 segments per day.
Interpretation session creation defaults to 100
sessions per hour. The registry surfaces quota
remaining in `X-Quota-Remaining` on every write
response and returns 429 with `Retry-After` once a
quota is exhausted.

## Annex P — Webhook payload shape

```json
{
  "event": "job.delivered",
  "jobRef": "f63f4f04-...",
  "deliveredAt": "2026-04-28T11:32:00+09:00",
  "segments": 1042,
  "qualityRefs": ["mqm-..."]
}
```

Webhook payloads are canonicalised per RFC 8785 before
the HMAC-SHA-256 signature is computed.

弘益人間 (Hongik Ingan) — Benefit All Humanity
