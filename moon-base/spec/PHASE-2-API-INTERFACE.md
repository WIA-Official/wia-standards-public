# WIA-moon-base PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-moon-base
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTP API contract that an accredited
moon-base programme exposes for the records defined in PHASE-1.
Consumers include partner mission operations centres, surface-
operations support teams, ground science teams, supply and crew-
rotation logistics, archive services, and citation tools that
resolve published moon-base operations records to their underlying
data.

References (CITATION-POLICY ALLOW only):

- IETF RFC 9110 (HTTP Semantics)
- IETF RFC 9111 (HTTP Caching)
- IETF RFC 9457 (Problem Details)
- IETF RFC 6901 (JSON Pointer)
- IETF RFC 6902 (JSON Patch)
- IETF RFC 8288 (Web Linking)
- IETF RFC 8259 (JSON)
- IETF RFC 9421 (HTTP Message Signatures)
- ISO 8601 (date and time)
- ISO/IEC 27001:2022 (information security management)
- CCSDS 132.0-B / 232.0-B (Space Data Link Protocols)
- CCSDS 727.0-B (CFDP)
- W3C Trace Context

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published by the operating
programme. Versioning uses `/v1/` path segments. The OpenAPI 3.1
document at `/v1/openapi.json` is canonical.

The API is a control-plane and metadata facade; bulk telemetry flows
over CCSDS protocols and CFDP, with the API exposing references to
the data objects in their canonical archives.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-moon-base",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "bases":              "/v1/bases",
    "habitatModules":     "/v1/habitat-modules",
    "lifeSupport":        "/v1/life-support",
    "power":              "/v1/power",
    "isru":               "/v1/isru",
    "mobility":           "/v1/mobility",
    "eva":                "/v1/eva",
    "comms":              "/v1/comms",
    "evidence":           "/v1/evidence",
    "openapi":            "/v1/openapi.json"
  }
}
```

## §3 Bases and Habitat Modules

```
POST   /v1/bases                              — register a base
GET    /v1/bases/{bid}                        — retrieve base record
PATCH  /v1/bases/{bid}/phase                  — advance phase
POST   /v1/bases/{bid}/habitat-modules        — register a module
GET    /v1/habitat-modules/{mid}              — retrieve module
PATCH  /v1/habitat-modules/{mid}/configuration — update configuration
                                                  reference
```

Phase transitions follow the schedule in PHASE-1 §2; invalid
transitions return `422` with type
`urn:wia:moon-base:phase-transition`.

## §4 Life-Support Telemetry

```
POST   /v1/habitat-modules/{mid}/life-support       — append a sample
POST   /v1/bulk/life-support                        — batched append
GET    /v1/life-support/{sid}                       — retrieve sample
GET    /v1/habitat-modules/{mid}/life-support?from={t}&to={t}
                                                    — query window
GET    /v1/habitat-modules/{mid}/life-support/alerts — current alerts
```

Samples are append-only. Servers MUST refuse out-of-order timestamp
submissions with type `urn:wia:moon-base:timestamp-regress` and
MUST raise an alert through the streaming endpoint when an alert
threshold is crossed.

## §5 Power and Microgrid

```
POST   /v1/bases/{bid}/power                  — append a power sample
GET    /v1/power/{sid}                        — retrieve sample
GET    /v1/bases/{bid}/power?from={t}&to={t}  — query window
PATCH  /v1/bases/{bid}/microgrid-state         — update microgrid state
```

Microgrid state changes (e.g. `normal` → `load-shedding`) are
broadcast over the streaming endpoint so that surface assets and
ground operations see the change in near-real time.

## §6 ISRU and Mobility

```
POST   /v1/bases/{bid}/isru-plants            — register an ISRU plant
GET    /v1/isru/{pid}                         — retrieve plant state
PATCH  /v1/isru/{pid}/state                   — update plant state
POST   /v1/bases/{bid}/mobility-assets        — register a mobility
                                                 asset
GET    /v1/mobility/{aid}                     — retrieve asset state
PATCH  /v1/mobility/{aid}/task-queue          — update task queue
```

Mobility-asset task queue updates are gated by surface-operations
approval; submissions that exceed the asset's planned operating
envelope (slope, drive distance, battery margin) return `422` with
type `urn:wia:moon-base:operating-envelope-violation`.

## §7 EVA Operations

```
POST   /v1/bases/{bid}/eva-operations         — register an EVA
GET    /v1/eva/{eid}                          — retrieve EVA record
PATCH  /v1/eva/{eid}/incidents                — append incident
PATCH  /v1/eva/{eid}/ingress                  — record ingress
                                                 details
```

EVA records cite crew members through opaque tokens; clinical
identifiers are never accepted in the body, and submissions that
include them return `422` with type
`urn:wia:moon-base:identifier-leak`.

## §8 Surface Communications

```
POST   /v1/bases/{bid}/comms/link-logs        — append a link log
GET    /v1/comms/{lid}                        — retrieve link log
GET    /v1/bases/{bid}/comms?from={t}&to={t}  — query window
```

## §9 Evidence Package

```
POST   /v1/bases/{bid}/evidence               — request package
                                                 generation
GET    /v1/evidence/{packageId}               — retrieve package
GET    /v1/evidence/{packageId}/manifest      — manifest only
```

## §10 Errors

All error responses are `application/problem+json` per RFC 9457.
Defined types include:

- `urn:wia:moon-base:phase-transition`
- `urn:wia:moon-base:timestamp-regress`
- `urn:wia:moon-base:operating-envelope-violation`
- `urn:wia:moon-base:identifier-leak`
- `urn:wia:moon-base:life-support-alert`
- `urn:wia:moon-base:evidence-mismatch`

## §11 Authentication

The API uses mutually-authenticated TLS for partner-MOC,
ground-team, and integrator connections. Public read-only endpoints
are reachable without a client certificate.

## §12 Streaming Subscriptions

Consumers subscribe via Server-Sent Events at
`/v1/bases/{bid}/events`. Topics include life-support alerts,
microgrid state changes, ISRU plant state changes, EVA operations,
and comms-link availability.

## §13 Worked Example: Sol of Operations

1. Crew wakes; life-support telemetry continues at the configured
   cadence. Daily summary is emitted to the operations team.
2. Power microgrid moves into `load-shedding` because solar
   irradiance drops; ground operations receives the streaming alert.
3. ISRU plant transitions to `idle` to honour the load-shedding
   regime.
4. Mobility-asset task queue is updated to defer non-urgent traverses.
5. EVA operation is registered, executed, and closed with consumables
   reconciliation.
6. End-of-sol evidence-package generation is requested; ground
   operations and partner agencies pin the manifest digest for the
   day's record.

## §14 Supply-Chain and Inventory Endpoints

```
POST   /v1/bases/{bid}/supply-launches      — register a launch
GET    /v1/supply-launches/{lid}            — retrieve manifest
PATCH  /v1/supply-launches/{lid}/arrival    — register arrival delta
POST   /v1/bases/{bid}/inventory-snapshots  — register an inventory
                                              snapshot
GET    /v1/bases/{bid}/inventory?at={t}     — retrieve nearest
                                              snapshot to a time
```

Arrival deltas adjust the on-base running inventory monotonically.
Submissions whose delta would drive the inventory below the
contingency window threshold (PHASE-3 §3) return `409 Conflict`
with type `urn:wia:moon-base:contingency-window-breach` and require
the operations director's countersignature to proceed.

## §15 Crew-Rotation Endpoints

```
POST   /v1/bases/{bid}/crew-rotations    — register a rotation
GET    /v1/crew-rotations/{rid}          — retrieve rotation record
PATCH  /v1/crew-rotations/{rid}/occur    — mark rotation complete
```

Crew rotations carry only opaque tokens; clinical identifiers in
the body return `422` with type
`urn:wia:moon-base:identifier-leak`.

## §16 Heritage / Exclusion Endpoints

```
POST   /v1/bases/{bid}/exclusion-zones   — register a zone
GET    /v1/exclusion-zones/{zid}         — retrieve zone
GET    /v1/bases/{bid}/exclusion-zones?intersects={geometry}
                                          — query zones intersecting
                                            a candidate route
```

The intersect query is consumed by route-planning systems to verify
that a proposed traverse does not breach an exclusion zone;
breaches return `422` from the mobility task-queue endpoint with
type `urn:wia:moon-base:exclusion-breach`.

## §17 Bulk Operations

```
POST   /v1/bulk/life-support      — batched life-support telemetry
POST   /v1/bulk/power             — batched power telemetry
POST   /v1/bulk/comms             — batched link-log upload
GET    /v1/bulk/{operationId}     — operation status
```

## §18 Pagination Conventions

List endpoints use cursor-based pagination via the `cursor` query
parameter and `Link` headers. Cursors are opaque; servers MUST
persist cursor state for at least 24 hours.

## §19 Audit and Observability

Every endpoint emits structured logs with `baseId`, `traceId`, the
issuing client certificate's subject, and the surface clock skew vs
the relay-network reference time at request issuance.

## §20 Radiation and Sol Endpoints

```
POST   /v1/bases/{bid}/radiation              — append radiation
                                                 sample
GET    /v1/radiation/{sid}                    — retrieve sample
GET    /v1/bases/{bid}/radiation?from={t}&to={t}
                                              — query window
GET    /v1/bases/{bid}/crew-dose-ledger       — retrieve current
                                                 ledgers
POST   /v1/bases/{bid}/sols                   — register a sol
GET    /v1/sols/{sid}                         — retrieve sol record
```

Crew-dose ledger entries are append-only; corrections emit a new
entry that supersedes the prior. The API enforces dose-policy
limits and emits an alert through the streaming endpoint when a
crew member approaches the policy limit.

## §21 Privacy-Preserving Aggregation

Aggregate-only consumers fetch population-level statistics through
endpoints that emit counts, means, and dispersions:

```
GET    /v1/aggregate/eva-duration?period=...
GET    /v1/aggregate/inventory-trend?class=...&period=...
GET    /v1/aggregate/radiation-exposure?period=...
```

Out-of-policy queries return `403 Forbidden` with type
`urn:wia:moon-base:cohort-too-small`.

## §22 Provenance Endpoint

```
GET    /v1/provenance/{recordId}    — retrieve provenance entry for
                                       any PHASE-1 record
```

Provenance entries trace a record to its parents (base record,
habitat module, sol record, supply launch) so that auditors can
walk the chain of records during retrospective review.

## §23 Conformance

A conformant server passes the test vectors published under
`tests/phase-vectors/phase-2-api-interface/`, emits an OpenAPI 3.1
document, and signs evidence packages per RFC 9421.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-moon-base
- **Last Updated:** 2026-04-27
