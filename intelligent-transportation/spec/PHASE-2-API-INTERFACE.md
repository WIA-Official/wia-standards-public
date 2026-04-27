# WIA-intelligent-transportation PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-intelligent-transportation
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTPS API contract that an accredited
intelligent-transportation programme exposes for the records
defined in PHASE-1. Consumers include partner operators (mutual
aid TMCs), transit agencies, journey-planning aggregators,
emergency-services dispatch systems, regulators, and the
operator's own analytics and historian platforms.

References (CITATION-POLICY ALLOW only):

- IETF RFC 9110 (HTTP Semantics)
- IETF RFC 9111 (HTTP Caching)
- IETF RFC 9457 (Problem Details)
- IETF RFC 6901 / 6902 (JSON Pointer / Patch)
- IETF RFC 8288 (Web Linking)
- IETF RFC 8259 (JSON)
- IETF RFC 9421 (HTTP Message Signatures)
- ISO 8601 (date and time)
- ISO/IEC 27001:2022 (information security management)
- W3C Trace Context
- TMDD v3.1 (Traffic Management Data Dictionary)
- GTFS-Realtime v2.0
- SIRI v2.1

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published by the operating
programme. Versioning uses `/v1/` path segments. The OpenAPI 3.1
document at `/v1/openapi.json` is canonical.

V2X over-the-air messaging (BSM, SPaT, MAP, DENM, CAM) does not
flow through this API; this API is the metadata, evidence, and
operational facade. Over-the-air message exchange follows IEEE
1609 / SAE J2735 / ETSI ITS-G5 directly.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-intelligent-transportation",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "programmes":          "/v1/programmes",
    "networkReferences":   "/v1/network-references",
    "rsus":                "/v1/rsus",
    "obus":                "/v1/obus",
    "v2xCaptures":         "/v1/v2x-captures",
    "signalControlStates": "/v1/signal-control-states",
    "incidents":           "/v1/incidents",
    "transitPositions":    "/v1/transit-vehicle-positions",
    "evidence":            "/v1/evidence",
    "openapi":             "/v1/openapi.json"
  }
}
```

## §3 Programme Lifecycle

```
POST   /v1/programmes                  — register a programme
GET    /v1/programmes/{pid}            — retrieve a programme
PATCH  /v1/programmes/{pid}/status     — advance status
GET    /v1/programmes?jurisdiction={c} — list by jurisdiction
```

Status transitions follow PHASE-1 §2; invalid transitions return
`422` with type `urn:wia:intelligent-transportation:status-transition`.

## §4 Network References

```
POST   /v1/network-references             — register a reference
GET    /v1/network-references/{rid}       — retrieve reference
GET    /v1/network-references/decode?encoded={e}&method={m}
                                          — decode an external
                                            reference against the
                                            programme's map
```

Decode failures (unknown method, unresolvable reference) return
`422` with type
`urn:wia:intelligent-transportation:reference-not-resolvable`.

## §5 RSU and OBU Lifecycle

```
POST   /v1/programmes/{pid}/rsus          — register an RSU
PATCH  /v1/rsus/{rid}/status              — advance RSU status
PATCH  /v1/rsus/{rid}/firmware            — update firmware
                                            reference
GET    /v1/rsus/{rid}                     — retrieve RSU record
POST   /v1/programmes/{pid}/obus          — register an OBU
PATCH  /v1/obus/{oid}/firmware            — update firmware
                                            reference
GET    /v1/obus/{oid}                     — retrieve OBU record
```

RSU firmware updates that change the radio stack require a fresh
type-approval reference; submissions without an updated type
approval return `409 Conflict` with type
`urn:wia:intelligent-transportation:type-approval-required`.

## §6 V2X Capture Endpoints

```
POST   /v1/programmes/{pid}/v2x-captures  — register a capture
GET    /v1/v2x-captures/{cid}             — retrieve capture
                                            metadata
GET    /v1/v2x-captures/{cid}/archive     — download archive
                                            (gated by retention
                                            and by privacy filter
                                            authorisation)
```

Archive downloads carry the operator's privacy-filter version in
the response so that downstream consumers know which redaction
rule was applied.

## §7 Signal Control States and Streaming

```
POST   /v1/programmes/{pid}/signal-control-states
                                          — append a state record
GET    /v1/signal-control-states/{sid}    — retrieve state
GET    /v1/programmes/{pid}/signal-control-states?
       intersection={ref}&from={t}&to={t}
                                          — query window
```

Real-time consumers subscribe via Server-Sent Events at
`/v1/intersections/{intersectionRef}/events` for SPaT, preemption,
and cycle-change events.

## §8 Incidents

```
POST   /v1/programmes/{pid}/incidents     — register an incident
GET    /v1/incidents/{iid}                — retrieve incident
PATCH  /v1/incidents/{iid}/severity       — update severity
PATCH  /v1/incidents/{iid}/resolved       — close incident
PATCH  /v1/incidents/{iid}/detour         — attach detour plan
```

Incident classification updates beyond `critical` automatically
trigger an outbound notification to mutual-aid partner operators
through the integration described in PHASE-4 §6.

## §9 Transit Vehicle Positions

```
POST   /v1/programmes/{pid}/transit-positions
                                          — append a position
                                            record (typically
                                            batched)
POST   /v1/bulk/transit-positions         — batched append
GET    /v1/transit-positions/{tpid}       — retrieve position
GET    /v1/programmes/{pid}/transit-positions?route={r}&from={t}
                                          — query window
```

Position records that arrive more than 5 minutes after their
`observedAt` are flagged `stale` and excluded from real-time
streaming; consumers may still retrieve them for historical
analysis.

## §10 Bulk Operations and Pagination

Bulk endpoints accept arrays for high-volume signal control state
ingest, transit position ingest, and incident batch import.
Cursor-based pagination uses the `cursor` query parameter and
`Link` headers (RFC 8288); cursors persist for at least 24 hours.

## §11 Authentication and Authorisation

The API uses mutually-authenticated TLS for partner-operator,
transit-agency, regulator, and emergency-services connections.
Public read-only endpoints (incident summaries above `info`,
transit-position aggregates) are reachable without a client
certificate; per-OBU position queries are restricted to the
operating programme and to consents the programme has authorised.

## §12 Errors

All error responses are `application/problem+json` per RFC 9457.
Defined types include:

- `urn:wia:intelligent-transportation:status-transition`
- `urn:wia:intelligent-transportation:reference-not-resolvable`
- `urn:wia:intelligent-transportation:type-approval-required`
- `urn:wia:intelligent-transportation:capture-retention-elapsed`
- `urn:wia:intelligent-transportation:detour-conflicts`
- `urn:wia:intelligent-transportation:evidence-mismatch`

## §13 Caching and Concurrency

Stable resources (closed incidents with detours expired,
historic v2x capture metadata, signed evidence packages) are
cacheable with `Cache-Control: max-age=31536000, immutable`.
Real-time signal-control state and live transit-position records
are cacheable for 5 seconds. ETags are mandatory on every PATCH
endpoint with `If-Match` conditional requests.

## §14 Audit and Observability

Every endpoint emits structured logs with `programmeId`,
`traceId`, the issuing client certificate's subject, and the
operator clock skew vs the reference NTP source.

## §15 Worked Example: Incident-Driven Detour

1. Operator CCTV detects a multi-vehicle collision on a freeway
   segment. The operator opens an incident at severity `major`.
2. The TMC computes a detour plan that re-routes traffic through
   the signed arterial. The detour plan is attached to the
   incident.
3. The operator's signal-coordination engine ramps adjacent
   signals into a coordinated arterial plan; signal-control state
   records reflect the change.
4. The operator publishes a TIM (Traveller Information Message)
   through the affected RSUs; OBUs in the area display the
   advisory.
5. Mutual-aid partners receive the incident notification through
   the partner-operator integration.
6. On resolution, the operator closes the incident and reverts the
   coordination plan; final evidence package is generated.

## §16 VRU Awareness and Work-Zone Endpoints

```
POST   /v1/programmes/{pid}/vru-awareness        — register
                                                   awareness
GET    /v1/vru-awareness/{vid}                   — retrieve
                                                   awareness
POST   /v1/programmes/{pid}/work-zones           — register a
                                                   work zone
PATCH  /v1/work-zones/{wzid}/lane-closure-plan   — update plan
PATCH  /v1/work-zones/{wzid}/end                 — close work
                                                   zone
GET    /v1/work-zones/{wzid}                     — retrieve
                                                   work zone
GET    /v1/programmes/{pid}/work-zones?
       reference={ref}&from={t}&to={t}            — query window
```

Work-zone updates that overlap an active incident emit a
`urn:wia:intelligent-transportation:detour-conflicts` problem
document so that the operator can re-plan before activation.

## §17 Provenance Endpoint

```
GET    /v1/provenance/{recordId}    — retrieve provenance entry
                                       for any PHASE-1 record
```

Provenance entries trace a record to its parents (programme,
network reference, RSU, OBU, capture archive, signal-control
state) so that auditors can walk the chain end-to-end.

## §18 Privacy-Preserving Aggregation

Aggregate consumers (research collaboratives, public-policy
analysts, journey-planning aggregators with limited authorisation)
fetch population-level statistics through aggregation endpoints
that emit counts, means, and dispersions:

```
GET    /v1/aggregate/incident-rate?period=...&jurisdiction=...
GET    /v1/aggregate/transit-on-time?period=...&route=...
GET    /v1/aggregate/vru-detection-counts?period=...
```

Out-of-policy queries (cohort below threshold, OBU-level detail
requested without authorisation) return `403 Forbidden` with type
`urn:wia:intelligent-transportation:cohort-too-small`.

## §19 Conformance

A conformant server passes the test vectors published under
`tests/phase-vectors/phase-2-api-interface/`, emits an OpenAPI 3.1
document, signs evidence packages per RFC 9421, and rejects
direct V2X over-the-air messaging on the metadata endpoints.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-intelligent-transportation
- **Last Updated:** 2026-04-28
