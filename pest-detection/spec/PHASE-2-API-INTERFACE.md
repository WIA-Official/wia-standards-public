# WIA-pest-detection PHASE 2 — API Interface Specification

**Standard:** WIA-pest-detection
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable

This PHASE defines the resource-oriented API surface
for pest-detection operations: field-unit registration,
observation and trap intake, sample submission and
diagnostic-result retrieval, edge-inference posting,
remote-sensing imagery binding, alert dispatch,
intervention-decision logging, and NPPO reporting
binding. The API is engineered for both
edge / offline-first usage (drones, field sensors,
mobile scouts on intermittent connectivity) and
authority-grade exchange (laboratory ↔ NPPO ↔ IPPC).

References (CITATION-POLICY ALLOW only):
- IETF RFC 9110 (HTTP), RFC 9112 (HTTP/1.1), RFC 9113 (HTTP/2)
- IETF RFC 9457 (Problem Details for HTTP APIs)
- IETF RFC 8259 (JSON), RFC 8785 (JCS), RFC 4122 (UUID), RFC 9530 (Content-Digest)
- IETF RFC 7233 (HTTP Range Requests) — for chunked imagery upload
- OGC Sensor Web Enablement, OGC SensorThings API
- OGC Web Map Tile Service (WMTS), OGC Web Coverage Service (WCS)
- ISO 19115-2 — geographic-information metadata, ISO 19139 XML
- ISO 11783 — ISOBUS / agricultural electronics
- ISO/IEC 18004 (QR code) — chain-of-custody label
- W3C Web of Things (WoT) Thing Description 1.1 — for sensor binding

---

## §1 Endpoint root

API root is implementation-controlled. All endpoints
are TLS 1.3 (RFC 8446). Edge clients participate in a
mutual-TLS profile with sponsor-issued certificates so
unmanned devices identify cryptographically. NPPO
laboratory ↔ NPPO authority links use the relevant
sovereign authentication framework.

## §2 Field-unit endpoints

```
POST   /v1/field-units                       register field unit
GET    /v1/field-units/{ref}                 retrieve
PATCH  /v1/field-units/{ref}                 amend non-identity fields
GET    /v1/field-units?crop=&region=         list / filter
POST   /v1/field-units/{ref}/phenology       record phenology stage update
```

Field-unit geometry is immutable once first observation
records bind to it; geometry edits open a new field-
unit version linked via `precedingRef`.

## §3 Observation endpoints

```
POST   /v1/observations                      record observation
POST   /v1/observations/$batch               batch ingest (NDJSON)
GET    /v1/observations/{ref}                retrieve
GET    /v1/observations?fieldUnit=&time=     list / filter
DELETE /v1/observations/{ref}                soft delete (status=excluded)
```

Batch ingest is supported for offline-first scouts:
the device queues observations locally and submits a
batch on reconnect. Each batch line carries an idempotency
hash so resubmission is safe.

## §4 Trap endpoints

```
POST   /v1/traps                             register trap
PATCH  /v1/traps/{ref}                       amend (lure swap, position)
POST   /v1/traps/{ref}/service               record service event
GET    /v1/traps/{ref}                       retrieve
GET    /v1/traps?kind=&active=               list / filter
```

Service events are append-only and record the operator,
the time, and the catch description.

## §5 Sample and diagnostic endpoints

```
POST   /v1/samples                           register sample
POST   /v1/samples/{ref}/custody             append chain-of-custody event
POST   /v1/samples/{ref}/dispatch            record dispatch to lab
POST   /v1/diagnostics                       record diagnostic result
GET    /v1/diagnostics/{ref}                 retrieve
GET    /v1/diagnostics?sample=&lab=          list / filter
```

Diagnostic records signed by an ISO 17025-accredited
laboratory (RFC 7515 JWS over the canonical payload)
become the authoritative record for NPPO reporting.

## §6 Edge-inference endpoints

```
POST   /v1/inferences                        record edge-AI inference
GET    /v1/inferences/{ref}                  retrieve
GET    /v1/inferences?fieldUnit=&model=      list / filter
```

Inference requests carry the model identifier, the
container image digest, the input-media digest, and
the predictions. Implementations reject inferences
without a content-addressed model identifier.

## §7 Remote-sensing endpoints

```
POST   /v1/imagery                           register imagery (metadata)
POST   /v1/imagery/{ref}/upload              chunked upload (Range requests)
GET    /v1/imagery/{ref}                     retrieve metadata
GET    /v1/imagery/{ref}/coverage.geojson    coverage polygon
GET    /v1/imagery/{ref}/derived/{index}     derived index (NDVI / NDRE)
```

Imagery uploads are content-addressed; raw bytes never
leave the implementation's object store except through
authenticated authorisation grants.

## §8 Alert endpoints

```
POST   /v1/alerts                            dispatch alert
GET    /v1/alerts/{ref}                      retrieve
POST   /v1/alerts/{ref}/acknowledge          recipient acknowledgement
GET    /v1/alerts?severity=&recipient=       list / filter
```

Alert dispatch supports SMS, email, push notification,
and webhook delivery; each dispatch records the
delivery method and the carrier-receipt token where
available.

## §9 Intervention-decision endpoints

```
POST   /v1/decisions                         record intervention decision
PATCH  /v1/decisions/{ref}/application       record applied dose
GET    /v1/decisions/{ref}                   retrieve
GET    /v1/decisions?fieldUnit=&kind=        list / filter
```

A chemical intervention decision binds the active-
ingredient identifier (ISO 1750), the local-registry
product code, the MRL identifier, and the pre-harvest
interval; the application record extends the decision
with the actual applied dose, the applicator credential,
and the application-time weather summary.

## §10 NPPO reporting endpoints

```
POST   /v1/nppo-reports                      file NPPO report
GET    /v1/nppo-reports/{ref}                retrieve
PATCH  /v1/nppo-reports/{ref}/status         update status (e.g.            
                                             eradication-declared)
```

Once filed, an NPPO report submits to the IPPC
International Phytosanitary Portal via the NPPO
authority's official channel; the WIA record stores
the IPPC submission identifier.

## §11 Error model (RFC 9457)

```json
{
  "type":   "urn:wia:pest:problem:diagnostic-required",
  "title":  "NPPO report requires accredited diagnostic confirmation",
  "status": 409,
  "detail": "Cannot file NPPO report without ISO 17025 lab signature",
  "instance": "/v1/nppo-reports"
}
```

Common type URIs:

| Type URI suffix              | HTTP | Meaning                                       |
|------------------------------|-----:|-----------------------------------------------|
| `diagnostic-required`        | 409  | NPPO filing requires lab confirmation         |
| `mrl-violation`              | 422  | intervention exceeds MRL                      |
| `phi-not-met`                | 422  | application within PHI window                 |
| `applicator-uncredentialed`  | 403  | applicator lacks active credential            |
| `model-digest-missing`       | 422  | inference posted without container digest     |
| `geometry-locked`            | 409  | field-unit geometry attempted to mutate       |

## §12 Pagination

Cursor-paginated list endpoints; cursors are opaque
base64url JSON.

## §13 Bulk export

```
GET  /v1/$export?_type=Observation,Diagnostic,NppoReport
GET  /v1/$status/{exportId}
GET  /v1/$result/{exportId}/{file}
```

Output is NDJSON, one record per line.

## §14 Audit headers

| Header                  | Meaning                                       |
|-------------------------|-----------------------------------------------|
| `X-Request-Id`          | client-set, echoed by server                  |
| `X-Audit-Event-Id`      | server-set, links to PHASE 3 audit chain      |
| `Content-Digest`        | RFC 9530 SHA-256 of the response body         |
| `X-Trace-Id`            | W3C Trace Context (`traceparent`)             |

## §15 Versioning

Resource paths are version-prefixed (`/v1/...`).

## Annex A — OpenAPI reference

A canonical OpenAPI 3.1 description is published at
`api/openapi-3.1.yaml`.

## Annex B — Worked observation submission (informative)

```http
POST /v1/observations HTTP/1.1
Authorization: Bearer ...
WIA-PEST-Schema-Version: 1.0
Idempotency-Key: 7a8c0b1f-...

{
  "fieldUnitRef": "fu-2026-mango-orchard-12",
  "observationTime": "2026-04-12T08:14:00+05:30",
  "observerKind": "scout",
  "pestCandidate": ["EPPO:STENNS","EPPO:BACTSP"],
  "count": 4,
  "severityScale": "EPPO-PP1/152(4)-3",
  "lifeStage": "adult",
  "mediaRef": "wia-pest://media/scout-photo-2026-04-12-0814"
}
```

## Annex C — Async export pattern

```
POST   /v1/$export                      → 202 Accepted, Content-Location
GET    /v1/$status/{id}                 → 202 in-progress / 200 manifest
GET    /v1/$result/{id}/{file}          → 200 NDJSON
DELETE /v1/$status/{id}                 → 202 cancellation
```

## Annex D — Webhook surface

Implementations may expose webhooks for `alert-
dispatched`, `diagnostic-confirmed`, `nppo-report-
filed`, and `intervention-applied` events. Payloads
sign with RFC 7515 JWS; receivers verify against
`/.well-known/wia-pest-keys.json`. Delivery is at-
least-once; receivers are expected to be idempotent on
`eventId`.

## Annex E — Conformance disclosure

Implementations declare the OpenAPI revision served,
the OGC SensorThings profile supported, the NPPO
bindings supported, the EPPO Code dictionary version,
and the FHIR Bulk Data profile version (where bound).
