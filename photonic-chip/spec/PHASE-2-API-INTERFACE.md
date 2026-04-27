# WIA-photonic-chip PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-photonic-chip
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTP API contract that an accredited
photonic-chip programme exposes for the records defined in PHASE-1.
Consumers include design houses that submit layouts, foundries that
publish PDKs and consume tape-out submissions, wafer-test laboratories
that emit per-die reports, packaging facilities, system integrators,
and reference laboratories that re-measure deployed modules.

References (CITATION-POLICY ALLOW only):

- IETF RFC 9110 (HTTP Semantics)
- IETF RFC 9111 (HTTP Caching)
- IETF RFC 9457 (Problem Details)
- IETF RFC 6901 (JSON Pointer)
- IETF RFC 6902 (JSON Patch)
- IETF RFC 8288 (Web Linking)
- IETF RFC 8259 (JSON)
- IETF RFC 5789 (PATCH)
- IETF RFC 9421 (HTTP Message Signatures)
- ISO 8601 (date and time)
- ISO/IEC 27001:2022 (information security management)
- W3C Trace Context

---

## §1 Scope and Versioning

JSON-over-HTTPS, served from a domain published by the operating
programme. Versioning uses `/v1/` path segments and Semantic
Versioning 2.0.0 at the major-version level. The OpenAPI 3.1 document
at `/v1/openapi.json` is canonical.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-photonic-chip",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "designs":          "/v1/designs",
    "schematics":       "/v1/schematics",
    "layouts":          "/v1/layouts",
    "fabricationRuns":  "/v1/fabrication-runs",
    "waferTests":       "/v1/wafer-tests",
    "components":       "/v1/components",
    "packages":         "/v1/packages",
    "telemetry":        "/v1/telemetry",
    "evidence":         "/v1/evidence",
    "openapi":          "/v1/openapi.json"
  }
}
```

## §3 Designs and PDK Pinning

```
POST   /v1/designs                   — register a design
GET    /v1/designs/{designId}        — retrieve a design
PATCH  /v1/designs/{designId}        — update mutable fields
GET    /v1/designs?platform={p}      — list designs by platform
```

A design submission MUST pin a PDK identifier and version (PHASE-1 §3);
submissions whose PDK reference does not resolve return `422` with
type `urn:wia:photonic-chip:pdk-unresolvable`.

## §4 Schematics and Layouts

```
POST   /v1/designs/{designId}/schematics    — register a schematic
GET    /v1/schematics/{sid}                 — retrieve schematic
POST   /v1/designs/{designId}/layouts       — register a layout
GET    /v1/layouts/{lid}                    — retrieve layout
GET    /v1/layouts/{lid}/drc                — fetch DRC report
GET    /v1/layouts/{lid}/lvs                — fetch LVS report
```

Layouts whose DRC or LVS reports are missing or fail return `422` with
type `urn:wia:photonic-chip:drc-lvs-incomplete`. Foundries MUST refuse
tape-out acceptance for such layouts and the API enforces the refusal
at submission time.

## §5 Fabrication Runs

```
POST   /v1/fabrication-runs              — foundry registers a run
GET    /v1/fabrication-runs/{rid}        — retrieve run
PATCH  /v1/fabrication-runs/{rid}/yield  — update yield estimate
```

Foundries register fabrication runs against the design via the API.
The foundry's client certificate is bound to the foundry's identifier
so that the API can verify the registering party's authority over the
run.

## §6 Wafer-Test and Component Measurements

```
POST   /v1/fabrication-runs/{rid}/wafer-tests   — register a wafer test
GET    /v1/wafer-tests/{wid}                    — retrieve a wafer test
POST   /v1/wafer-tests/{wid}/component-measurements — append per-component
                                                       measurements
GET    /v1/components/{cid}                     — retrieve component
                                                  measurement record
```

Component measurements are immutable once posted; subsequent
measurements of the same component on the same die emit new
measurement records rather than overwriting existing ones.

## §7 Packaging

```
POST   /v1/wafer-tests/{wid}/packages    — register a packaged die
GET    /v1/packages/{pid}                — retrieve package record
PATCH  /v1/packages/{pid}/laser-class    — update laser-safety class
                                          after re-classification
```

Laser-safety class changes that cross IEC 60825-1 boundaries trigger
notification to downstream consumers via the streaming subscription
defined in §13.

## §8 Telemetry

```
POST   /v1/packages/{pid}/telemetry     — append a telemetry sample
GET    /v1/telemetry/{tid}              — retrieve a telemetry record
GET    /v1/packages/{pid}/telemetry?since={t}  — telemetry window
```

Telemetry uploads are typically batched at the deployed module's next
sync; the API accepts gzip-compressed batched uploads at
`POST /v1/bulk/telemetry` and emits per-batch ingest receipts.

## §9 Evidence Package

```
POST   /v1/packages/{pid}/evidence    — request package generation
GET    /v1/evidence/{packageId}       — retrieve package
GET    /v1/evidence/{packageId}/manifest  — manifest only
```

The evidence-package format is governed by PHASE-4 §3 and contains the
design, schematic, layout, fabrication run, wafer-test, component
measurements, package record, and signed manifest.

## §10 Errors

All error responses are `application/problem+json` per RFC 9457.
Defined types under `urn:wia:photonic-chip` include:

- `urn:wia:photonic-chip:pdk-unresolvable`
- `urn:wia:photonic-chip:drc-lvs-incomplete`
- `urn:wia:photonic-chip:laser-class-mismatch`
- `urn:wia:photonic-chip:component-measurement-missing`
- `urn:wia:photonic-chip:evidence-mismatch`

## §11 Authentication

The API uses mutually-authenticated TLS for foundry-to-programme,
laboratory-to-programme, and integrator-to-programme connections.
Public read-only endpoints (the design landing pages, the OpenAPI
document, the well-known discovery resource) are reachable without a
client certificate.

## §12 Caching

Stable resources (completed fabrication runs, signed evidence
packages) are cacheable with `Cache-Control: max-age=31536000,
immutable`. Mutable resources (draft designs, in-progress test
sessions) are cacheable for 60 seconds. ETags are mandatory on every
PATCH endpoint with conditional requests via `If-Match`.

## §13 Streaming Subscriptions

Consumers monitor long fabrication and test campaigns via
Server-Sent Events at `/v1/designs/{designId}/events`. Events
emitted include fabrication-run status changes, wafer-test
completions, package emissions, and laser-class re-classifications.

## §14 Worked Example: Tape-Out to Citation

1. Design house registers the design with a pinned PDK.
2. Schematic and layout are registered; DRC and LVS reports
   accompany the layout.
3. Foundry accepts tape-out and registers a fabrication run.
4. Wafer-test laboratory registers per-die test results with
   component measurements.
5. Packaging facility registers packaged dies; laser-safety class
   is set per IEC 60825-1.
6. System integrator deploys the module and uploads field telemetry.
7. Reference laboratory re-measures deployed modules; new
   measurement records are appended.
8. Citation tool requests an evidence package and pins the manifest
   digest.

## §15 Wavelength-Plan and Calibration Endpoints

```
POST   /v1/designs/{designId}/wavelength-plans      — register a plan
GET    /v1/wavelength-plans/{wpid}                  — retrieve plan
POST   /v1/packages/{pid}/phase-calibrations        — register calibration
GET    /v1/phase-calibrations/{cid}                 — retrieve calibration
GET    /v1/packages/{pid}/phase-calibrations?since={t} — calibration series
```

Calibration submissions are signed by the laboratory's client
certificate. Submissions whose drive-level sweep falls outside the
device's certified envelope return `422` with type
`urn:wia:photonic-chip:phase-calibration-out-of-envelope`.

## §16 Bulk Operations

Wafer-test campaigns produce hundreds of die-level records per wafer
and tens of thousands of component-measurement records per campaign;
these are exchanged in bulk. The bulk endpoints accept arrays of
records and emit operation resources that track the batch:

```
POST   /v1/bulk/wafer-tests           — submit a batched wafer-test
                                          campaign
POST   /v1/bulk/component-measurements — submit a batched per-component
                                          measurement set
GET    /v1/bulk/{operationId}         — campaign status
```

Bulk operations report aggregate progress and per-item status. Servers
MUST tolerate partial success and emit a per-batch ingest receipt.

## §17 Pagination Conventions

List endpoints use cursor-based pagination via the `cursor` query
parameter and `Link` headers (RFC 8288). Cursors are opaque to
clients; servers MUST persist enough state to resolve a cursor for
at least 24 hours.

## §18 Audit and Observability

Every endpoint emits structured logs with `designId`, `traceId`, the
issuing client certificate's subject, and the foundry / laboratory
clock skew vs the reference NTP source.

## §19 Loss-Budget and Reliability Endpoints

```
POST   /v1/designs/{designId}/loss-budgets         — register a budget
GET    /v1/loss-budgets/{bid}                      — retrieve a budget
PATCH  /v1/loss-budgets/{bid}                      — update measured-db
POST   /v1/designs/{designId}/reliability          — register a result
GET    /v1/reliability/{rid}                       — retrieve a result
```

Reliability submissions are signed by the test laboratory's client
certificate. The API rejects population sizes below the minimum that
the operating programme has registered for the relevant test family
(typical minima are documented in the foundry's qualification
package); rejections return `422` with type
`urn:wia:photonic-chip:reliability-population-too-small`.

## §20 Recall-Notice Endpoint

```
POST   /v1/recall-notices                          — issue a recall
GET    /v1/recall-notices/{nid}                    — retrieve a notice
POST   /v1/recall-notices/{nid}/receipts           — system-integrator
                                                    acknowledgement
GET    /v1/recall-notices/{nid}/reach              — reach metrics
```

Recall notices are signed by the operating programme's release key.
System-integrator receipts are signed by the integrator's client
certificate; receipts identify the deployed devices that contain
modules covered by the notice. Reach metrics are computed from
receipts and are exposed to the relevant authority on request.

## §21 Privacy-Preserving Aggregation

Aggregate-only consumers (supply-chain analysts, public-policy
analysts) fetch population-level statistics through aggregation
endpoints that emit counts, means, and dispersions. The endpoints
honour a minimum cohort threshold so that small-volume foundries
are not exposed to inference-by-intersection.

```
GET    /v1/aggregate/yield?platform=...&period=...
GET    /v1/aggregate/recall-reach?period=...
```

Out-of-policy queries return `403 Forbidden` with type
`urn:wia:photonic-chip:cohort-too-small`.

## §22 FHIR-Adjacent Bridges (optional)

Programmes whose modules deploy into clinical or biomedical settings
MAY expose a FHIR R5 facade that translates module identity and
calibration records into FHIR `Device` and `DeviceMetric`
resources. The facade is read-only and is consumed by clinical
deployment software that prefers FHIR over WIA-native APIs.

## §23 Conformance

A conformant server passes the test vectors published under
`tests/phase-vectors/phase-2-api-interface/`, emits an OpenAPI 3.1
document, and signs evidence packages per RFC 9421.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-photonic-chip
- **Last Updated:** 2026-04-27
