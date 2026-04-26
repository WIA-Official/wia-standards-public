# WIA-metamaterial PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-metamaterial
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTP API contract that an accredited metamaterial
design and characterisation programme exposes for the records defined in
PHASE-1. The contract is consumed by collaborating design groups, by
fabrication vendors, by accredited measurement laboratories, and by
publication and citation tools that resolve a metamaterial result to its
underlying simulation and measurement evidence.

References (CITATION-POLICY ALLOW only):

- IETF RFC 9110 (HTTP Semantics)
- IETF RFC 9111 (HTTP Caching)
- IETF RFC 9112 (HTTP/1.1)
- IETF RFC 9113 (HTTP/2)
- IETF RFC 9114 (HTTP/3)
- IETF RFC 9457 (Problem Details for HTTP APIs)
- IETF RFC 6901 (JSON Pointer)
- IETF RFC 6902 (JSON Patch)
- IETF RFC 8288 (Web Linking)
- IETF RFC 8259 (JSON)
- IETF RFC 5789 (PATCH method)
- IETF RFC 9421 (HTTP Message Signatures)
- ISO 8601 (date and time)
- ISO/IEC 27001:2022 (information security management)
- W3C Trace Context (correlation propagation)

---

## §1 Scope and Versioning

The API is a JSON-over-HTTPS interface served on a domain published by the
operating programme. Versioning uses URL path segments (`/v1/`) and follows
Semantic Versioning 2.0.0 at the major-version level. Additive changes in
shape are permitted in-place; non-additive changes require a new major
path segment. The OpenAPI 3.1 document at `/v1/openapi.json` is the
canonical machine-readable description.

## §2 Root Discovery

```
GET /v1/
```

Response:

```json
{
  "standard": "WIA-metamaterial",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "designs":       "/v1/designs",
    "unitCells":     "/v1/unit-cells",
    "simulations":   "/v1/simulations",
    "sParameters":   "/v1/s-parameters",
    "dispersion":    "/v1/dispersion",
    "retrievals":    "/v1/retrievals",
    "tolerances":    "/v1/tolerances",
    "measurements":  "/v1/measurements",
    "evidence":      "/v1/evidence",
    "openapi":       "/v1/openapi.json"
  }
}
```

## §3 Designs

```
POST   /v1/designs                    — create a design
GET    /v1/designs/{designId}         — retrieve a design
PATCH  /v1/designs/{designId}         — update mutable fields
GET    /v1/designs?domain={d}&since={ts}  — list with cursor pagination
```

`POST /v1/designs` accepts the design record from PHASE-1 §2. The server
generates `designId` server-side as a UUIDv7 and returns the canonical
record. Status transitions follow the schedule defined in PHASE-1 §2;
invalid transitions return `422 Unprocessable Entity` with type
`urn:wia:metamaterial:status-transition`.

## §4 Unit Cells, Simulations, and Spectra

```
POST   /v1/designs/{designId}/unit-cells    — register a unit cell
POST   /v1/unit-cells/{ucid}/simulations    — submit a simulation request
GET    /v1/simulations/{simId}/s-parameters — retrieve scattering data
GET    /v1/simulations/{simId}/dispersion   — retrieve dispersion bands
```

Simulation submissions are asynchronous: the server responds `202 Accepted`
with a `Location` header pointing at the simulation resource. The
simulation transitions through `pending` → `running` → `complete` (or
`failed`). Streaming progress is exposed via Server-Sent Events at
`/v1/simulations/{simId}/events`.

A request with a meshing budget that the server cannot satisfy returns
`507 Insufficient Storage` with type
`urn:wia:metamaterial:simulation-budget`.

## §5 Effective-Parameter Retrieval

```
POST   /v1/simulations/{simId}/retrievals   — perform a retrieval
GET    /v1/retrievals/{retrievalId}         — fetch retrieval result
GET    /v1/retrievals/{retrievalId}/branches — branch-selection metadata
```

Retrieval is a separate request because the inversion may be re-run with
different branch-selection policies; the design record retains every
retrieval that was applied so that downstream consumers can compare them.

## §6 Tolerance Budgets

```
POST   /v1/designs/{designId}/tolerances    — register a tolerance budget
PATCH  /v1/tolerances/{tid}                 — adjust sensitivities
GET    /v1/tolerances/{tid}/yield-estimate  — predicted yield given sigma
```

The yield estimate accepts an optional `samples` query parameter to
control the Monte-Carlo sample size used by the server.

## §7 Measurements

```
POST   /v1/designs/{designId}/measurements        — register a measurement
GET    /v1/measurements/{mid}                     — retrieve measurement
GET    /v1/measurements/{mid}/raw                 — fetch the raw archive
PATCH  /v1/measurements/{mid}/uncertainty         — append uncertainty
                                                    contributions
```

The raw archive is delivered as an immutable resource with
`Cache-Control: max-age=31536000, immutable`; the URL is content-addressed
by the SHA-256 of the archive.

## §8 Evidence Package

```
POST   /v1/designs/{designId}/evidence    — request package generation
GET    /v1/evidence/{packageId}           — retrieve a package
GET    /v1/evidence/{packageId}/manifest  — retrieve manifest only
```

The evidence package format is governed by PHASE-4 §3 and contains the
design record, unit cell, simulations, retrievals, tolerance budgets,
measurements, and signed manifest.

## §9 Authentication

The API uses mutually-authenticated TLS for laboratory-to-laboratory and
laboratory-to-publisher connections, with client certificates issued by an
accreditation root operated by the certifying body. Public read-only
endpoints (the design landing pages, the openapi document, the well-known
discovery resource) are reachable without a client certificate.

## §10 Errors

All error responses are `application/problem+json` per RFC 9457. The
`type` field is a URN under the `urn:wia:metamaterial` namespace. Defined
types include:

- `urn:wia:metamaterial:status-transition` — invalid status change.
- `urn:wia:metamaterial:geometry-out-of-domain` — inclusion outside cell.
- `urn:wia:metamaterial:simulation-budget` — request exceeds budget.
- `urn:wia:metamaterial:retrieval-branch` — ambiguous branch choice.
- `urn:wia:metamaterial:measurement-uncertainty` — uncertainty contribution
  missing.
- `urn:wia:metamaterial:evidence-mismatch` — package digest mismatch.

Each problem response carries a `traceId` per W3C Trace Context.

## §11 Caching and Conditional Requests

GET responses for stable resources (completed simulations, measurements,
evidence packages) are cacheable with `Cache-Control: max-age=31536000,
immutable`. Mutable resources (draft designs, in-progress simulations) are
cacheable for 60 seconds. ETags are mandatory on every PATCH endpoint;
conditional requests use `If-Match`. Servers MUST return `412 Precondition
Failed` on stale ETags with the current ETag exposed in the
`serverETag` problem extension.

## §12 Worked Example: From Design to Evidence

The canonical positive vector exercises the API end-to-end:

1. Create a design under `/v1/designs` with the operating band declared.
2. Register a unit cell describing the lattice and inclusions.
3. Submit a simulation request; poll the simulation resource (or
   subscribe to its event stream) until it transitions to `complete`.
4. Retrieve effective parameters and capture the chosen branch policy.
5. Register a tolerance budget and request a yield estimate.
6. Register a measurement on the realised sample and append uncertainty
   contributions.
7. Request an evidence package, retrieve the package manifest, and pin
   the manifest digest as the externally citable reference.

A conformant server completes this flow without error for the canonical
positive vector under `tests/phase-vectors/phase-2-api-interface/`.

## §13 Field-Distribution Endpoints

For designs that carry field-distribution payloads (PHASE-1 §11), the
following endpoints expose the field archives to consumers:

```
GET    /v1/simulations/{simId}/field-distributions
GET    /v1/field-distributions/{fdid}                 — metadata
GET    /v1/field-distributions/{fdid}/archive         — raw archive
GET    /v1/field-distributions/{fdid}/slice?z={m}     — server-rendered
                                                        2-D slice as JSON
GET    /v1/field-distributions/{fdid}/probe?x={m}&y={m}&z={m}
                                                      — single-point sample
```

Server-rendered slices and probes are convenience endpoints; consumers
that require full-fidelity field data MUST retrieve the archive. The
slice endpoint accepts an `Accept` header to negotiate JSON, NumPy
`.npy`, or HDF5 packaging; the archive is delivered in the encoding
declared on the field-distribution metadata.

## §14 Streaming Subscriptions

Consumers that require live updates during a long-running simulation
campaign subscribe to a Server-Sent Events stream at
`/v1/designs/{designId}/events`. Events emitted include simulation
status changes, measurement registrations, fabrication-certificate
arrivals, and evidence-package availability.

A subscription includes a heartbeat every 30 seconds so that
intermediaries can detect stale connections and reconnect. Replays from
the last known event identifier are supported via the
`Last-Event-ID` header (W3C EventSource semantics).

## §15 Audit and Observability

Every endpoint emits structured logs with `designId`, `traceId`, the
issuing client certificate's subject, and the operator's clock skew vs
the reference NTP source. Audit logs are retained for at least seven
calendar years from the last access of the design.

## §16 Bulk Operations

Long-running design sweeps (parameter sweeps over lattice constants,
parametric studies on inclusion geometry, statistical campaigns over
fabrication tolerances) are submitted as bulk operations rather than as
individual requests. A bulk operation accepts an array of design,
unit-cell, simulation, or retrieval specifications and emits an
operation resource that tracks the sweep as a unit.

```
POST   /v1/bulk/simulations         — submit a bulk simulation sweep
GET    /v1/bulk/{operationId}       — retrieve operation status
GET    /v1/bulk/{operationId}/items — per-item status and results
DELETE /v1/bulk/{operationId}       — cancel a running operation
```

Bulk operations report aggregate progress (`pendingCount`, `runningCount`,
`completeCount`, `failedCount`) and per-item terminal status. Cancellation
is best-effort; items already running on a solver continue to completion
and emit their results normally so that work is not lost.

The bulk-operation endpoints share the rate-limit budget with their
single-request counterparts. Programmes that anticipate sustained sweep
load SHOULD negotiate a higher budget with the certifying body and
record the negotiated budget in their quality dossier.

## §17 Pagination Conventions

List endpoints (`/v1/designs`, `/v1/measurements`, `/v1/bulk/...`) use
cursor-based pagination via the `cursor` query parameter and a
`Link` header (RFC 8288) carrying `rel="next"` and `rel="prev"`
relations. Cursors are opaque to clients; servers MUST persist enough
state to resolve a cursor for at least 24 hours after issuance so that
mid-sweep clients are not interrupted by cursor expiry.

## §18 Conformance

A conformant server passes the test vectors published under
`tests/phase-vectors/phase-2-api-interface/`, emits an OpenAPI 3.1 document
that round-trips the resource shapes defined in this PHASE, and signs
exported evidence packages per RFC 9421.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-metamaterial
- **Last Updated:** 2026-04-27
