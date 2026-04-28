# WIA-physical-enhancement PHASE 2 — API Interface Specification

**Standard:** WIA-physical-enhancement
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable

This PHASE defines the resource-oriented API surface
for physical-enhancement operations: subject enrolment,
intervention management, session lifecycle, performance
capture, consent / ethics / TUE binding, sport-
governance integration, adverse-event reporting, and
device telemetry. The API supports clinical, occupational,
and sport contexts on a single shape; per-context fields
are gated by the `useContext` field on each request.

References (CITATION-POLICY ALLOW only):
- IETF RFC 9110 (HTTP), RFC 9112 (HTTP/1.1), RFC 9113 (HTTP/2)
- IETF RFC 9457 (Problem Details for HTTP APIs)
- IETF RFC 8259 (JSON), RFC 8785 (JCS), RFC 4122 (UUID), RFC 9530 (Content-Digest)
- HL7 SMART App Launch 2.0
- HL7 FHIR R5 — Procedure, Observation, DeviceUseStatement, Bulk Data
- WADA ADAMS data-exchange profile (TUE submission interface, where exposed)
- ISO/IEC 27033-2 — network security guidance
- 21 CFR Part 11 — electronic records / signatures (US clinical / sport laboratory)

---

## §1 Endpoint root

API root is implementation-controlled. All endpoints
are TLS 1.3 (RFC 8446). Clinical contexts use SMART on
FHIR launch flows; sport-governance contexts use a
mutual-TLS profile per the IF / NADO interconnection
agreement; occupational contexts use sponsor-issued
client_credentials with key attestation.

## §2 Subject endpoints

```
POST   /v1/subjects                    enrol new subject
GET    /v1/subjects/{ref}              retrieve subject record
PATCH  /v1/subjects/{ref}              amend non-identity fields
DELETE /v1/subjects/{ref}              withdraw subject
GET    /v1/subjects/{ref}/sessions     list sessions for subject
```

Sport-context subjects additionally expose:

```
GET    /v1/subjects/{ref}/tues          active TUE records
GET    /v1/subjects/{ref}/whereabouts   testing-pool whereabouts (gated)
```

## §3 Intervention endpoints

```
POST   /v1/interventions               register intervention
GET    /v1/interventions/{ref}         retrieve
PATCH  /v1/interventions/{ref}         amend
GET    /v1/interventions               list, filter by `kind`,
                                       `regulatoryClass`, `protocolRef`
```

Protocol-locked fields (kind, productRef, protocolRef,
doseSchedule, regulatoryClass) require a new version on
amendment.

## §4 Session endpoints

```
POST   /v1/sessions                    open session
PATCH  /v1/sessions/{ref}              update during session
POST   /v1/sessions/{ref}/close        close session
POST   /v1/sessions/{ref}/telemetry    stream telemetry chunk
GET    /v1/sessions/{ref}              retrieve
GET    /v1/sessions                    list, filter by subject /
                                       intervention / range
```

Telemetry chunks carry timestamps, sensor identifiers,
and SHA-256 checksums; the implementation reassembles
chunks into the session physiological-log artefact.

## §5 Performance endpoints

```
POST   /v1/performance                 record measurement
GET    /v1/performance/{ref}           retrieve
GET    /v1/performance                 list / filter
```

Each performance record carries a rater UUID independent
of the operator UUID to preserve blind-rater independence.

## §6 Consent endpoints

```
POST   /v1/consents                    create consent
GET    /v1/consents/{ref}              retrieve
POST   /v1/consents/{ref}/withdraw     subject-initiated withdrawal
GET    /v1/consents                    list active consents
```

## §7 Ethics-approval endpoints

```
POST   /v1/ethics-approvals            register approval
GET    /v1/ethics-approvals/{ref}      retrieve
GET    /v1/ethics-approvals?protocol=  list per protocol
```

## §8 Sport-governance endpoints

```
POST   /v1/sport-bindings              register sport-governance binding
GET    /v1/sport-bindings/{ref}        retrieve
POST   /v1/tues                        create TUE record
GET    /v1/tues/{ref}                  retrieve
POST   /v1/tues/{ref}/decision         record IF / NADO decision
```

Sport-governance endpoints are gated by an additional
scope (`sport:tue.write`) and audit events are mirrored
to the IF / NADO ADAMS submission queue.

## §9 Adverse-event endpoints

```
POST   /v1/adverse-events              report AE / SAE
GET    /v1/adverse-events/{ref}        retrieve
PATCH  /v1/adverse-events/{ref}        update follow-up
GET    /v1/adverse-events?serious=true list SAEs only
```

## §10 Device endpoints

```
POST   /v1/devices                     register device
GET    /v1/devices/{ref}               retrieve
POST   /v1/devices/{ref}/calibrate     record calibration event
PATCH  /v1/devices/{ref}               update software / firmware
```

## §11 Error model (RFC 9457)

```json
{
  "type":   "urn:wia:pe:problem:tue-required",
  "title":  "Sport-context intervention requires TUE",
  "status": 403,
  "detail": "Subject PE-A07 has no active TUE for prohibited substance / method",
  "instance": "/v1/sessions"
}
```

Common type URIs:

| Type URI suffix              | HTTP | Meaning                                       |
|------------------------------|-----:|-----------------------------------------------|
| `tue-required`               | 403  | sport-prohibited intervention without TUE     |
| `tue-expired`                | 409  | TUE has lapsed                                |
| `class-mismatch`             | 409  | adaptive-class binding inconsistent           |
| `consent-missing`            | 403  | no active consent for protocol-version        |
| `ethics-expired`             | 409  | IRB / IEC approval has lapsed                 |
| `device-calibration-stale`   | 409  | device past calibration interval              |
| `serious-ae-unreported`      | 409  | session-close blocked while open SAE pending  |
| `whereabouts-incomplete`     | 422  | testing-pool whereabouts data missing         |

## §12 Bulk export (FHIR alignment)

```
GET  /v1/$export?_type=Procedure,Observation,DeviceUseStatement
GET  /v1/$status/{exportId}
GET  /v1/$result/{exportId}/{file}
```

## §13 Audit headers

| Header                    | Meaning                                       |
|---------------------------|-----------------------------------------------|
| `X-Request-Id`            | client-set, echoed by server                  |
| `X-Audit-Event-Id`        | server-set, links to PHASE 3 audit chain      |
| `X-Trace-Id`              | W3C Trace Context (`traceparent`)             |
| `Content-Digest`          | RFC 9530 SHA-256 of the response body         |

## §14 Versioning

Resource paths are version-prefixed (`/v1/...`).

## §15 Authentication and scopes (informative)

For clinical contexts:

```
patient/Procedure.read
patient/Observation.read
user/AdverseEvent.write
user/DeviceUseStatement.write
system/Bulk.export
```

For sport contexts (in addition):

```
sport:tue.read
sport:tue.write
sport:whereabouts.read
sport:adams.submit
```

## Annex A — OpenAPI reference

A canonical OpenAPI 3.1 document is published at
`api/openapi-3.1.yaml`.

## Annex B — Worked TUE registration (informative)

```http
POST /v1/tues HTTP/1.1
Authorization: Bearer ...
WIA-PE-Schema-Version: 1.0

{
  "subjectRef": "PE-A07",
  "substance":  "Salbutamol",
  "atc":        "R03AC02",
  "indication": "Exercise-induced bronchoconstriction",
  "supportingDocumentRef": "wia-pe://document/spirometry-2026-04-01"
}
```

Response 201 returns the TUE identifier and the queued
ADAMS submission reference.

## Annex C — Conformance disclosure

Implementations declare the OpenAPI revision served,
the SMART scopes supported, the IF / NADO bindings
supported, and the FHIR Bulk Data profile version.

## Annex D — Async export pattern

```
POST   /v1/$export                      → 202 Accepted, Content-Location
GET    /v1/$status/{id}                 → 202 in-progress / 200 manifest
GET    /v1/$result/{id}/{file}          → 200 NDJSON
DELETE /v1/$status/{id}                 → 202 cancellation
```

## Annex E — Telemetry streaming endpoint

For powered exoskeletons and other devices that emit
high-rate telemetry the endpoint accepts one of three
profiles:

| Profile         | Transport                                     |
|-----------------|-----------------------------------------------|
| HTTP chunked    | HTTP/1.1 Transfer-Encoding: chunked           |
| HTTP/2 streams  | HTTP/2 framed body                            |
| WebSocket       | RFC 6455 with TLS 1.3                         |

Each chunk carries:

```json
{
  "chunkSeq":    42,
  "chunkTime":   "2026-04-12T10:14:38.512+09:00",
  "samples":     [{"t":0.000,"jntL":12.4,"jntR":11.8,"emg1":0.42,...}, ...],
  "chunkDigest": "sha-256:..."
}
```

Reassembly is sequence-checked; missing chunks return
`409 chunk-out-of-order` with the expected next
sequence number. The implementation reassembles the
session telemetry into the BIDS-aligned physiological-
log artefact at session close.

## Annex F — Webhook surface

Implementations may expose webhooks for session-finish,
ae-filed, tue-status-change, and device-recall events.
Payloads sign with RFC 7515 JWS; receivers verify
against a published JWK set at `/.well-known/wia-pe-
keys.json`. Delivery is at-least-once; receivers are
expected to be idempotent on `eventId`.

## Annex G — Workflow dispatch (for digital-training arms)

```
POST   /v1/training-workflows           dispatch a session-prescription run
GET    /v1/training-workflows/{ref}     retrieve status / result manifest
```

Workflow dispatch records the workflow URI, the
container image digest, the input prescription
parameters, and the output artefact digests. Workflows
that fail are retried at most three times before
operator review.
