# WIA-prosthetic-control PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-prosthetic-control
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTP API contract that an accredited
prosthetic-control programme exposes for the records defined in
PHASE-1. The contract is consumed by clinics that fit devices, by
prosthesis manufacturers that operate post-market surveillance, by
clinical-trial sponsors that aggregate device-use data, and by national
medical-device authorities that audit serious adverse events. It is not
intended for direct end-user consumption; consumer-facing apps mediate
through the operating clinic's electronic health record.

References (CITATION-POLICY ALLOW only):

- IETF RFC 9110 (HTTP Semantics)
- IETF RFC 9111 (HTTP Caching)
- IETF RFC 9457 (Problem Details for HTTP APIs)
- IETF RFC 6901 (JSON Pointer)
- IETF RFC 6902 (JSON Patch)
- IETF RFC 8288 (Web Linking)
- IETF RFC 8259 (JSON)
- IETF RFC 5789 (PATCH method)
- IETF RFC 9421 (HTTP Message Signatures)
- ISO 8601 (date and time)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 27701:2019 (privacy information management)
- HL7 FHIR R5 (Patient, Device, DeviceUseStatement, Observation,
  ClinicalImpression resources)
- W3C Trace Context

---

## §1 Scope and Versioning

The API is JSON-over-HTTPS served on a domain published by the operating
programme. Versioning uses URL path segments (`/v1/`) and follows
Semantic Versioning 2.0.0 at the major-version level. The OpenAPI 3.1
document at `/v1/openapi.json` is the canonical machine-readable
description.

## §2 Root Discovery

```
GET /v1/
```

Response:

```json
{
  "standard": "WIA-prosthetic-control",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "subjects":          "/v1/subjects",
    "acquisitions":      "/v1/acquisitions",
    "decoders":          "/v1/decoders",
    "motorCommands":     "/v1/motor-commands",
    "calibrations":      "/v1/calibrations",
    "feedback":          "/v1/feedback",
    "adverseEvents":     "/v1/adverse-events",
    "evidence":          "/v1/evidence",
    "fhirBridge":        "/v1/fhir",
    "openapi":           "/v1/openapi.json"
  }
}
```

## §3 Subjects

```
POST   /v1/subjects                — register a fitted subject
GET    /v1/subjects/{subjectId}    — retrieve subject record
PATCH  /v1/subjects/{subjectId}    — update mutable fields
GET    /v1/subjects?cohort={c}     — list subjects in cohort
```

Subject records carry the opaque `subjectId` token defined in PHASE-1 §2;
the API never returns clinical identifiers. A request that attempts to
publish a clinical identifier in the body returns `422` with type
`urn:wia:prosthetic-control:identifier-leak`.

## §4 Acquisitions

```
POST   /v1/subjects/{sid}/acquisitions     — register an acquisition
GET    /v1/acquisitions/{aid}              — retrieve acquisition
GET    /v1/acquisitions/{aid}/raw          — fetch raw archive
PATCH  /v1/acquisitions/{aid}/notes        — append clinical notes
```

Raw archives are immutable resources delivered with content-addressed
URLs and `Cache-Control: max-age=31536000, immutable`. Acquisitions
performed during home use are uploaded in batches at the next
clinic-side sync; the upload endpoint accepts batched archives via
`POST /v1/acquisitions/batch`.

## §5 Decoders

```
POST   /v1/subjects/{sid}/decoders         — register a decoder
GET    /v1/decoders/{did}                  — decoder record
GET    /v1/decoders/{did}/model            — fetch model artefact
POST   /v1/decoders/{did}/online-update    — append an adaptation step
```

Online-update events are appended monotonically; servers MUST NOT permit
out-of-order appends and MUST reject updates whose timestamps regress
with type `urn:wia:prosthetic-control:adaptation-regress`. Adaptation
logs are retained per PHASE-3 §7.

## §6 Motor Commands

```
POST   /v1/subjects/{sid}/motor-commands         — bulk command upload
GET    /v1/motor-commands/{cid}                  — single command record
GET    /v1/motor-commands?subject={sid}&from={t} — query a window
```

Motor-command uploads are typically batched at the end of a use-window
or a clinic visit; servers MUST accept gzip-compressed batches and MUST
emit a per-batch ingest receipt that lists the count of accepted
records, the count of rejected records, and the rejection-reason
histogram.

## §7 Calibration Sessions

```
POST   /v1/subjects/{sid}/calibrations    — register a calibration session
GET    /v1/calibrations/{calId}           — calibration record
PATCH  /v1/calibrations/{calId}/outcome   — update outcome
```

Calibration outcomes drive the device's clinical state; an outcome of
`needs-rework` triggers a follow-up scheduling event in the clinic
EHR via the FHIR bridge (§9).

## §8 Feedback and Stimulation

```
POST   /v1/subjects/{sid}/feedback        — register a feedback session
GET    /v1/feedback/{fid}                 — feedback record
PATCH  /v1/feedback/{fid}/limits          — update per-channel limits
```

Stimulation-based feedback configurations (TENS, peripheral-nerve) are
gated by IEC 60601-aligned safety limits that the API enforces:
attempts to set limits beyond the device's certified envelope return
`422` with type `urn:wia:prosthetic-control:limit-exceeded`.

## §9 FHIR Bridge

The API exposes a read-only FHIR R5 facade that translates subject and
device-use records into FHIR `Patient`, `Device`,
`DeviceUseStatement`, `Observation`, and `ClinicalImpression` resources.
The bridge does not expose the FHIR write endpoints; data flows clinic
→ programme through the WIA-native endpoints, and the FHIR facade is a
read facade for clinic EHRs that are FHIR-only.

## §10 Errors

All error responses are `application/problem+json` per RFC 9457. Defined
types under `urn:wia:prosthetic-control` include:

- `urn:wia:prosthetic-control:identifier-leak`
- `urn:wia:prosthetic-control:adaptation-regress`
- `urn:wia:prosthetic-control:limit-exceeded`
- `urn:wia:prosthetic-control:risk-file-missing`
- `urn:wia:prosthetic-control:adverse-event-late-report`
- `urn:wia:prosthetic-control:evidence-mismatch`

## §11 Adverse-Event Reporting Endpoint

```
POST   /v1/adverse-events                 — report an adverse event
GET    /v1/adverse-events/{eid}           — retrieve a report
PATCH  /v1/adverse-events/{eid}/follow-up — append follow-up notes
```

An event registered with `severity = "serious"` or
`severity = "life-threatening"` automatically triggers an outbound
notification to the configured national medical-device authority via
the integration described in PHASE-4 §10. The authority's report
reference is appended to the event record on receipt.

## §12 Caching, ETag, and Conditional Requests

Stable resources (calibration outcomes, completed sessions, evidence
packages) are cacheable with `Cache-Control: max-age=31536000,
immutable`. Live subject records are cacheable for 60 seconds. ETags
are mandatory on every PATCH endpoint with conditional requests via
`If-Match`.

## §13 Worked Example: From Fitting to Adverse-Event Report

1. Clinic POSTs a subject record at fitting; PATCHes the subject's
   `controlMode` after a control-mode revision.
2. Acquisitions are uploaded across the fitting visit and during
   home-use sync events.
3. Decoders are registered and updated; online-update events flow as
   the user's signals drift.
4. Motor-command batches arrive at clinic syncs with ingest receipts.
5. Calibration sessions are POSTed; an outcome of `needs-rework`
   triggers a clinic follow-up via the FHIR bridge.
6. A serious adverse event is reported; the API forwards the event to
   the national authority and records the authority's report
   reference.

A conformant server completes this flow without error for the canonical
positive vector.

## §14 Outcomes and Configuration-Snapshot Endpoints

```
POST   /v1/subjects/{sid}/outcomes              — register an outcome
GET    /v1/outcomes/{oid}                       — outcome record
GET    /v1/subjects/{sid}/outcomes?inst={i}     — series for an instrument
POST   /v1/subjects/{sid}/configuration-snapshots — register a snapshot
POST   /v1/subjects/{sid}/configuration-rollback  — request a rollback
```

A rollback request names a prior snapshot identifier; the API
validates that the snapshot is present and signed, schedules the
rollback for the next clinic visit (rollback-on-firmware is permitted
in some jurisdictions but is not the default), and returns an
operation resource that the clinic worklist consumes.

## §15 Streaming Subscriptions

Long device-use windows produce a continuous motor-command and
feedback stream. Subscribers (clinic dashboards, manufacturer
surveillance teams) consume the streams via Server-Sent Events at
`/v1/subjects/{sid}/stream` with topic filters
(`?topic=motor`, `?topic=adverse`, `?topic=adaptation`).

A subscription includes a heartbeat every 30 seconds. Subscribers
that disconnect can resume from the last seen event identifier via
the `Last-Event-ID` header (W3C EventSource semantics). Stream events
do not carry raw acquisition payloads; subscribers fetch raw archives
through the acquisition endpoints when needed.

## §16 Audit and Observability

Every endpoint emits structured logs with `subjectId`, `traceId`, the
issuing client certificate's subject, and the device's clock skew vs
the clinic's reference NTP source. Audit logs are retained per
PHASE-3 §7.

## §17 Bulk Operations

Long observation windows and clinical-trial cohorts produce large
volumes of acquisitions, motor commands, and outcomes that are
exchanged in bulk rather than per-record. Bulk endpoints accept
arrays of records and emit operation resources that track the
batch as a unit:

```
POST   /v1/bulk/acquisitions       — submit a batched acquisition upload
POST   /v1/bulk/motor-commands     — submit a batched motor-command set
POST   /v1/bulk/outcomes           — submit a batched outcome set
GET    /v1/bulk/{operationId}      — retrieve operation status
GET    /v1/bulk/{operationId}/items — per-item status
```

Bulk operations report aggregate progress and per-item terminal
status. Servers MUST tolerate partial success and emit a per-batch
ingest receipt so that clinics can reconcile against their local
records.

## §18 Pagination Conventions

List endpoints (`/v1/subjects`, `/v1/acquisitions`,
`/v1/motor-commands`, `/v1/bulk/...`) use cursor-based pagination via
the `cursor` query parameter and a `Link` header (RFC 8288) carrying
`rel="next"` and `rel="prev"` relations. Cursors are opaque to
clients; servers MUST persist enough state to resolve a cursor for
at least 24 hours after issuance so that clinic-side workflows are
not interrupted by cursor expiry.

## §19 Privacy-Preserving Aggregation Endpoints

Aggregate-only consumers (insurance carriers, public-health analysts)
fetch population-level statistics through endpoints that emit only
counts, means, and dispersions. The endpoints honour a minimum
cohort-size threshold so that small cohorts do not allow
re-identification by intersection.

```
GET    /v1/aggregate/outcomes?instrument=...&cohort=...
GET    /v1/aggregate/adverse-events?category=...&period=...
```

Requests that target cohorts smaller than the threshold return `403
Forbidden` with type
`urn:wia:prosthetic-control:cohort-too-small`.

## §20 Conformance

A conformant server passes the test vectors published under
`tests/phase-vectors/phase-2-api-interface/`, emits an OpenAPI 3.1
document, signs evidence packages per RFC 9421, and rejects clinical-
identifier leaks at request validation.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-prosthetic-control
- **Last Updated:** 2026-04-27
