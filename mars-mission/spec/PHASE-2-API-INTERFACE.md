# WIA-mars-mission PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-mars-mission
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTP API contract that an accredited Mars-
mission programme exposes for the records defined in PHASE-1.
Consumers include partner mission operations centres, science
operations centres, deep-space communications providers, navigation
teams, planetary-protection officers, archives, and citation tools
that resolve published mission products to their underlying records.

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
- CCSDS 633.0-B (Mission Operations Services)
- W3C Trace Context

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published by the operating
programme. Versioning uses `/v1/` path segments and Semantic
Versioning 2.0.0. The OpenAPI 3.1 document at `/v1/openapi.json` is
canonical.

The API is a control-plane and metadata facade; bulk product data
flows over CCSDS protocols and CFDP (CCSDS 727.0-B), with the API
exposing references to the data objects in their canonical
archives.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-mars-mission",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "missions":            "/v1/missions",
    "spacecraft":          "/v1/spacecraft",
    "trajectoryProducts":  "/v1/trajectory-products",
    "ttcPackets":          "/v1/ttc-packets",
    "observationRequests": "/v1/observation-requests",
    "scienceProducts":     "/v1/science-products",
    "planetaryProtection": "/v1/planetary-protection",
    "evidence":            "/v1/evidence",
    "openapi":             "/v1/openapi.json"
  }
}
```

## §3 Missions and Spacecraft

```
POST   /v1/missions                      — register a mission
GET    /v1/missions/{mid}                — retrieve mission record
PATCH  /v1/missions/{mid}/phase          — advance mission phase
POST   /v1/missions/{mid}/spacecraft     — register a spacecraft
GET    /v1/spacecraft/{sid}              — retrieve spacecraft record
```

Mission-phase transitions follow the schedule in PHASE-1 §2; invalid
transitions return `422` with type
`urn:wia:mars-mission:phase-transition`.

## §4 Trajectory and State Products

```
POST   /v1/spacecraft/{sid}/trajectory-products    — register a product
GET    /v1/trajectory-products/{tpid}              — retrieve product
GET    /v1/trajectory-products/{tpid}/archive      — fetch artefact
```

Products are immutable once registered; corrections emit a new
product with the prior product referenced as `predecessor`.
Navigation teams subscribe to the corresponding event stream
(§13) so that downstream consumers can fetch the latest valid
product without polling.

## §5 TT&C Packet Catalogue

```
POST   /v1/spacecraft/{sid}/ttc-packets/batch     — batched packet
                                                    catalogue upload
GET    /v1/ttc-packets/{pid}                      — retrieve packet
                                                    metadata
GET    /v1/spacecraft/{sid}/ttc-packets?from={t}  — query a window
```

Packet uploads carry only metadata and the SHA-256 of the framed
payload; the framed octets are referenced into the canonical archive.
Consumers that require the framed packet retrieve it from the
archive using the content-address.

## §6 Observation Requests and Science Products

```
POST   /v1/missions/{mid}/observation-requests        — submit a request
GET    /v1/observation-requests/{rid}                 — retrieve request
PATCH  /v1/observation-requests/{rid}/approvals       — append approval
POST   /v1/observation-requests/{rid}/products        — register product
GET    /v1/science-products/{pid}                     — retrieve product
GET    /v1/science-products/{pid}/archive             — fetch artefact
```

Observation requests are accepted only after all required approvals
(science working group, mission ops, planetary protection) have
been recorded. Submissions missing an approval return `422` with
type `urn:wia:mars-mission:approval-incomplete`.

## §7 Planetary Protection

```
POST   /v1/missions/{mid}/planetary-protection      — register a record
GET    /v1/planetary-protection/{ppid}              — retrieve record
PATCH  /v1/planetary-protection/{ppid}/allowed-activities
                                                     — update allowed
                                                       activities
```

Planetary-protection records are signed by the planetary-protection
officer's client certificate; the API verifies the signature
against the officer's accreditation in the public register.

## §8 Evidence Package

```
POST   /v1/missions/{mid}/evidence       — request package generation
GET    /v1/evidence/{packageId}          — retrieve a package
GET    /v1/evidence/{packageId}/manifest — manifest only
```

The evidence-package format is governed by PHASE-4 §3 and contains
mission, spacecraft, trajectory-product, TT&C-packet, observation-
request, science-product, and planetary-protection records, plus
the signed manifest.

## §9 Errors

All error responses are `application/problem+json` per RFC 9457.
Defined types include:

- `urn:wia:mars-mission:phase-transition`
- `urn:wia:mars-mission:approval-incomplete`
- `urn:wia:mars-mission:planetary-protection-required`
- `urn:wia:mars-mission:trajectory-window-stale`
- `urn:wia:mars-mission:packet-archive-mismatch`
- `urn:wia:mars-mission:evidence-mismatch`

## §10 Authentication

The API uses mutually-authenticated TLS for partner-MOC,
science-team, and archive connections. Public read-only endpoints
(release-day press kits, the OpenAPI document, the well-known
discovery resource) are reachable without a client certificate.

## §11 Caching

Stable resources (released science products, completed evidence
packages) are cacheable with `Cache-Control: max-age=31536000,
immutable`. Mutable resources (observation requests in approval,
in-flight planning artefacts) are cacheable for 60 seconds.

## §12 Worked Example: From Plan to Citation

1. SOC submits an observation request; approvals are appended.
2. MOC schedules the request and uplinks the commanded sequence;
   TT&C packets are catalogued.
3. Spacecraft executes the observation; downlink TT&C packets are
   catalogued.
4. SOC processes the downlinked data through the agreed pipeline
   and registers Level-1 through Level-3 science products.
5. Citation tool requests an evidence package and pins the manifest
   digest as the externally citable reference.

## §13 Streaming Subscriptions

Consumers subscribe via Server-Sent Events at
`/v1/missions/{mid}/events`. Topics include trajectory-product
release, TT&C-packet catalogue updates, observation-request status
changes, science-product release, and planetary-protection-record
updates.

## §14 Sol Activity Plan and Sample-Return Endpoints

```
POST   /v1/spacecraft/{sid}/sol-activity-plans   — submit a sol plan
GET    /v1/sol-activity-plans/{pid}              — retrieve plan
PATCH  /v1/sol-activity-plans/{pid}/approvals    — append approval
POST   /v1/sol-activity-plans/{pid}/execute      — uplink the plan
POST   /v1/sample-steps                          — register a chain step
GET    /v1/sample-chains/{sampleId}              — retrieve full chain
PATCH  /v1/sample-steps/{sid}/curator            — update curator on
                                                   handover
```

Plans whose energy or thermal margins fail return `422` with type
`urn:wia:mars-mission:sol-margin-violation`. Plans submitted after
the uplink window cut-off (defined per ground-station booking)
return `409` with type `urn:wia:mars-mission:uplink-window-closed`.

Sample-return chain steps MUST chain to a known predecessor (except
for the acquisition step). Submissions whose `predecessorRef` does
not resolve return `422` with type
`urn:wia:mars-mission:sample-chain-broken`.

## §15 Bulk Operations

Telemetry windows produce many TT&C catalogue entries; bulk uploads
are supported at:

```
POST   /v1/bulk/ttc-packets       — batched TT&C upload
POST   /v1/bulk/science-products  — batched product registrations
GET    /v1/bulk/{operationId}     — operation status
```

Bulk operations report aggregate progress and per-item status with
ingest receipts.

## §16 Pagination Conventions

List endpoints use cursor-based pagination via the `cursor` query
parameter and `Link` headers. Cursors are opaque; servers MUST
persist cursor state for at least 24 hours.

## §17 Audit and Observability

Every endpoint emits structured logs with `missionId`, `traceId`,
the issuing client certificate's subject, and the spacecraft event
time vs ground time skew at request issuance.

## §18 Environmental Endpoints

```
POST   /v1/spacecraft/{sid}/environment    — register environmental
                                             observation
GET    /v1/environment/{eid}               — retrieve record
GET    /v1/spacecraft/{sid}/environment?from={t}&to={t}
                                           — query a window
```

Environmental observations submitted from atmospheric instruments
(weather stations on landers, descent imagers during EDL) are
keyed to the spacecraft and the observation interval. Out-of-window
submissions return `422` with type
`urn:wia:mars-mission:environment-window-mismatch`.

## §19 Provenance Endpoint

```
GET    /v1/provenance/{recordId}    — retrieve provenance entry for
                                       any PHASE-1 record
```

Provenance entries trace a record to its parents (mission record,
spacecraft record, observation request, etc.) so that auditors can
walk the chain end-to-end. Citation tools use the provenance
endpoint to present the chain alongside the cited record.

## §20 Privacy-Preserving Aggregation

Aggregate consumers (mission archive analysts, public-policy
analysts) fetch population-level statistics through aggregation
endpoints that emit counts, means, and dispersions:

```
GET    /v1/aggregate/observations?instrument=...&period=...
GET    /v1/aggregate/uplink-traffic?period=...
```

Out-of-policy queries return `403 Forbidden` with type
`urn:wia:mars-mission:cohort-too-small`.

## §21 Press-Embargoed Endpoints

Press releases that accompany science-product release sometimes
carry a coordinated-publication embargo lifted at a fixed wall-clock
time. Press-embargoed endpoints serve the embargoed press kit to
authorised journalists in advance under the embargo's release-time
condition; the embargo endpoint enforces the release time
server-side and returns `403 Forbidden` with type
`urn:wia:mars-mission:press-embargo-active` to non-authorised
clients before the release time.

## §22 Conformance

A conformant server passes the test vectors published under
`tests/phase-vectors/phase-2-api-interface/`, emits an OpenAPI 3.1
document, and signs evidence packages per RFC 9421.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-mars-mission
- **Last Updated:** 2026-04-27
