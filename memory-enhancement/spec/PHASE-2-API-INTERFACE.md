# WIA-memory-enhancement PHASE 2 — API Interface Specification

**Standard:** WIA-memory-enhancement
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable

This PHASE defines the resource-oriented API surface for
memory-enhancement operations: subject enrolment,
intervention management, session lifecycle, outcome
capture, consent handling, ethics-approval registry,
adverse-event reporting, and device binding. The API
is shaped so that ICH E6 (R3) GCP source-data and source-
document principles are honoured: every record exposed
through the API resolves to a sponsor-controlled audit
trail, and every state change is replayable from the
event log defined in PHASE 3.

References (CITATION-POLICY ALLOW only):
- IETF RFC 9110 (HTTP semantics), RFC 9112 (HTTP/1.1), RFC 9113 (HTTP/2)
- IETF RFC 9457 (Problem Details for HTTP APIs)
- IETF RFC 8259 (JSON), RFC 8785 (JCS), RFC 7515 (JWS), RFC 4122 (UUID)
- IETF RFC 6749, RFC 7636 (PKCE), RFC 8414 (OAuth Authorization Server Metadata)
- HL7 SMART App Launch 2.0 (clinical OAuth profile)
- HL7 FHIR R5 — RESTful API and Bulk Data Access
- ISO/IEC 27033-2 Network security guidance
- ISO/IEC 18004 (QR code) — used for consent QR exchange in §11
- 21 CFR Part 11 (US) electronic records and signatures
- IDMP (ISO 11615 / 11616 / 11238 / 11239 / 11240) for product identification

---

## §1 Endpoint root and identification

API root is implementation-controlled. All endpoints are
served over TLS 1.3 (RFC 8446) and authenticated using
SMART on FHIR launch profiles (HL7 SMART 2.0) for clinical
contexts, or `client_credentials` for sponsor-to-sponsor
machine-to-machine flows.

Resource URIs follow the form:

```
{root}/v1/{resource}/{id}
```

Where `{id}` is a UUID per RFC 4122 and never the
subject's legal identifier.

## §2 Subject endpoints

```
POST   /v1/subjects                    enrol new subject
GET    /v1/subjects/{subjectRef}       retrieve subject record
PATCH  /v1/subjects/{subjectRef}       amend non-identity fields
DELETE /v1/subjects/{subjectRef}       withdraw subject
GET    /v1/subjects/{ref}/sessions     list sessions for subject
```

Withdrawal is logical: the resource is marked
`withdrawn=true` with `withdrawnAt`. Pre-existing de-
identified analytic exports are not retracted; new data
collection ceases immediately.

## §3 Intervention endpoints

```
POST   /v1/interventions               register intervention
GET    /v1/interventions/{ref}         retrieve intervention
PATCH  /v1/interventions/{ref}         amend (protocol-locked fields  
                                       require new version)
GET    /v1/interventions               list, filter by `kind`,
                                       `protocolRef`, `productRef`
```

Protocol-locked fields (kind, productRef, protocolRef,
doseSchedule, riskClassification) cannot be patched in
place; an amendment opens a new intervention version
linked to the prior version via `precedingRef`.

## §4 Session endpoints

```
POST   /v1/sessions                    open session
PATCH  /v1/sessions/{ref}              update during session  
                                       (delivered-dose, deviation)
POST   /v1/sessions/{ref}/close        close session
GET    /v1/sessions/{ref}              retrieve
GET    /v1/sessions                    list, filter by subjectRef,  
                                       interventionRef, range
```

Open / Close are idempotent: a re-issued open with the
same `Idempotency-Key` (RFC draft `idempotency-key-header`)
returns the existing resource.

## §5 Outcome endpoints

```
POST   /v1/outcomes                    record outcome
GET    /v1/outcomes/{ref}              retrieve
GET    /v1/outcomes                    list, filter by sessionRef,  
                                       subjectRef, instrument
```

Each `POST /v1/outcomes` carries the rater UUID
distinct from the operator UUID of the upstream session
to preserve PROBE / blind-rater independence (per ICH
E9 estimands framework).

## §6 Consent endpoints

```
POST   /v1/consents                    create consent
GET    /v1/consents/{ref}              retrieve
POST   /v1/consents/{ref}/withdraw     subject-initiated withdrawal
GET    /v1/consents                    list active consents per  
                                       subjectRef
```

The consent payload includes the document hash (SHA-256
of the IRB-approved PDF) and the JWS signature payload.
The withdrawal endpoint requires the subject's authentic-
ation token or a witnessed proxy form attached as a
Multipart body.

## §7 Ethics-approval endpoints

```
POST   /v1/ethics-approvals            register approval
GET    /v1/ethics-approvals/{ref}      retrieve
GET    /v1/ethics-approvals?protocol=  list per protocol
```

Approvals are tied to a protocol version; protocol
amendments must reference a fresh approval before
sessions resume on the amended protocol.

## §8 Adverse-event endpoints

```
POST   /v1/adverse-events              report AE / SAE
GET    /v1/adverse-events/{ref}        retrieve
PATCH  /v1/adverse-events/{ref}        update follow-up info
GET    /v1/adverse-events?serious=true list SAEs only
```

`POST /v1/adverse-events` returns the locally-assigned
ICSR identifier. SAEs are propagated to regulators per
PHASE 3 §6 within the regulatory clock (FDA MedWatch
3500A — 15 calendar days; ICH E2A — 7 days for fatal /
life-threatening unexpected serious AE; 15 days for
other serious unexpected AE).

## §9 Device endpoints

```
POST   /v1/devices                     register device
GET    /v1/devices/{ref}               retrieve
POST   /v1/devices/{ref}/calibrate     record calibration event
PATCH  /v1/devices/{ref}               update software-version
```

A new calibration event becomes the active reference;
attempting to open a session against a device whose
last calibration is older than the protocol threshold
returns HTTP 409 with a Problem Details body of type
`urn:wia:me:problem:device-calibration-stale`.

## §10 Error model

All non-2xx responses return RFC 9457 Problem Details:

```json
{
  "type":   "urn:wia:me:problem:withdrawn-subject",
  "title":  "Subject has been withdrawn",
  "status": 409,
  "detail": "Subject ME-007 was withdrawn 2026-03-30; new sessions cannot open",
  "instance": "/v1/sessions"
}
```

Standard error type URIs include:

| Type URI suffix              | HTTP | Meaning                                         |
|------------------------------|-----:|-------------------------------------------------|
| `consent-missing`            | 403  | no active consent for subject + protocol-version |
| `ethics-expired`             | 409  | IRB approval has lapsed                         |
| `protocol-deviation-required`| 422  | open requires a deviation record                |
| `device-calibration-stale`   | 409  | device past calibration interval                |
| `serious-ae-unreported`      | 409  | session-close blocked while open SAE pending    |
| `withdrawn-subject`          | 409  | subject has withdrawn consent                   |

## §11 Consent-by-QR (informative)

For low-bandwidth field studies, the consent payload is
compressed and encoded into an ISO/IEC 18004 QR code
that the subject's device scans. The QR encodes a
`urn:wia:me:consent:{consentRef}` resolvable at the API
root and the SHA-256 hash of the consent document. This
preserves verifiability without round-tripping the full
PDF over slow links.

## §12 Bulk export (FHIR alignment)

Sponsor analytics use the FHIR R5 Bulk Data Access
async pattern:

```
GET  /v1/$export?_type=Procedure,Observation,AdverseEvent
GET  /v1/$status/{exportId}
GET  /v1/$result/{exportId}/{file}
```

Exports are de-identified per HIPAA Safe Harbor or EU
GDPR pseudonymisation; re-identification keys remain
on-prem with the sponsor.

## §13 Pagination

List endpoints return cursor-paginated responses:

```json
{
  "items": [...],
  "nextCursor": "eyJsYXN0IjoiMjAyNi0wNC0xMlQwOToxNDowMFoifQ"
}
```

Cursors are opaque base64-url JSON and have no stability
guarantee across schema version bumps.

## §14 Rate limits and quotas

API gateways enforce RFC 6585 / RFC 9110 status `429 Too
Many Requests` with a `Retry-After` header. SAE
notification endpoints are exempt from rate limits to
preserve regulatory clock compliance.

## Annex A — OpenAPI reference

A canonical OpenAPI 3.1 description of the endpoints in
this PHASE is published alongside the standard at
`api/openapi-3.1.yaml`. The OpenAPI document is the
authoritative wire-format reference; this PHASE is the
authoritative semantic reference.

## Annex B — SMART on FHIR scopes (informative)

```
patient/Procedure.read
patient/Observation.read
user/AdverseEvent.write
user/ResearchSubject.write
system/Bulk.export
```

## Annex C — Versioning

API resource paths are version-prefixed (`/v1/...`).
Major version bumps are signalled by changing the
prefix; minor and patch revisions are documented in
the OpenAPI changelog.

## Annex D — Conformance disclosure

Implementations declare the OpenAPI revision they serve,
the SMART scopes they require, and the FHIR Bulk Data
profile version.

## Annex E — Async-operation pattern

Long-running operations (bulk export, batch outcome
ingest, post-hoc rater scoring) follow the FHIR R5
async pattern:

```
POST   /v1/{op}              → 202 Accepted, Content-Location header
GET    /v1/{op-status}/{id}  → 202 in-progress, 200 complete with manifest
GET    /v1/{op-result}/{id}  → 200 binary (NDJSON / CSV / Parquet)
DELETE /v1/{op}/{id}         → 202 cancellation accepted
```

The 202 response carries `X-Progress: <token>` for
human-readable progress and `Retry-After: <seconds>`
for the polling client. Manifest output uses NDJSON
per the FHIR Bulk specification, one resource per line.

## Annex F — Audit and rate-limit headers

Every API response carries the following headers for
auditability:

| Header                    | Source / Meaning                        |
|---------------------------|-----------------------------------------|
| `X-Request-Id`            | client-set, echoed by server            |
| `X-Audit-Event-Id`        | server-set, links to PHASE 3 §8 event   |
| `X-Trace-Id`              | W3C Trace Context (`traceparent`)       |
| `X-Rate-Limit-Limit`      | quota in this window                    |
| `X-Rate-Limit-Remaining`  | quota remaining                         |
| `X-Rate-Limit-Reset`      | seconds until reset                     |
| `Retry-After`             | for 429 / 503 responses                 |
| `Content-Digest`          | RFC 9530 SHA-256 of the response body   |

## Annex G — Versioning safeguards

API consumers send the `WIA-ME-Schema-Version: 1.0`
header on each request. The server returns the
matched schema version in the response. A request that
declares an unsupported schema version is rejected with
`406 Not Acceptable` and a Problem Details body that
lists the supported schema versions.

## Annex H — Worked AE submission (informative)

```http
POST /v1/adverse-events HTTP/1.1
Host: me.example
Authorization: Bearer ...
Content-Type: application/json
WIA-ME-Schema-Version: 1.0
Idempotency-Key: 0c2d8b18-4a7e-4f0d-9a53-3a48c3a91111

{
  "subjectRef": "ME-007",
  "sessionRef": "f63f4f04-9a76-4d2c-9f53-2c4d09a1bbb1",
  "onsetTime": "2026-04-12T09:33:12+09:00",
  "meddraTerm": {"llt": "Generalised tonic-clonic seizure", "pt": "Seizure"},
  "severity": "life-threatening",
  "causality": "probable",
  "serious": true,
  "expedited": true,
  "outcome": "recovered-with-sequelae"
}
```

Response:

```http
HTTP/1.1 201 Created
Location: /v1/adverse-events/3c0a8b18-4a7e-4f0d-9a53-3a48c3a91234
Content-Type: application/json
X-Audit-Event-Id: 11ee5e80-1240-6f00-8000-0242ac11000a

{
  "aeRef": "3c0a8b18-4a7e-4f0d-9a53-3a48c3a91234",
  "icsrId": "ME-SP-2026-000123",
  "regulatoryClock": {"start": "2026-04-12T09:33:12+09:00",
                       "deadline": "2026-04-19T09:33:12+09:00",
                       "rule": "ICH-E2A-7day-fatal-life-threat"}
}
```
