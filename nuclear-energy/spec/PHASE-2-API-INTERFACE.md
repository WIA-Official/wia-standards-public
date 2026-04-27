# WIA-nuclear-energy PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-nuclear-energy
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTP API contract that an accredited
nuclear-energy programme exposes for the records defined in PHASE-1.
Consumers include national nuclear regulators, the international
safeguards inspectorate, fuel-cycle facility operators, energy market
operators that schedule nuclear-generated dispatch, and decommissioning
contractors that consume end-of-life records.

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
- IAEA Safety Standards Series and IAEA Safeguards Agreements
  (cited normatively for terminology)
- ICRP Publication 103 (radiation protection terminology)
- W3C Trace Context

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published by the operating
programme. Versioning uses `/v1/` path segments. The OpenAPI 3.1
document at `/v1/openapi.json` is canonical.

Operational telemetry from plant safety-related systems is **not**
exposed through this API; this API is the metadata, regulatory, and
inventory facade. Safety-related instrumentation flows through
plant-internal protocols on isolated networks per PHASE-3 §10.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-nuclear-energy",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "plants":              "/v1/plants",
    "coreConfigurations":  "/v1/core-configurations",
    "fuelAssemblies":      "/v1/fuel-assemblies",
    "operatingStates":     "/v1/operating-states",
    "outages":             "/v1/outages",
    "radiationProtection": "/v1/radiation-protection",
    "spentFuel":           "/v1/spent-fuel",
    "environmentalReleases": "/v1/environmental-releases",
    "safetyEvents":        "/v1/safety-events",
    "decommissioning":     "/v1/decommissioning",
    "evidence":            "/v1/evidence",
    "openapi":             "/v1/openapi.json"
  }
}
```

## §3 Plants and Core Configuration

```
POST   /v1/plants                            — register a plant
GET    /v1/plants/{pid}                      — retrieve plant record
PATCH  /v1/plants/{pid}/operating-phase      — advance phase
POST   /v1/plants/{pid}/core-configurations  — register a core
                                                 configuration
GET    /v1/core-configurations/{cid}         — retrieve configuration
```

Operating-phase transitions follow the schedule in PHASE-1 §2;
invalid transitions return `422` with type
`urn:wia:nuclear-energy:phase-transition`. Phase advancement to
`operating` requires the regulator's authorisation reference, which
the API verifies against the regulator's public register.

## §4 Fuel Assemblies and Burnup History

```
POST   /v1/fuel-assemblies                — register an assembly
GET    /v1/fuel-assemblies/{aid}          — retrieve assembly record
POST   /v1/fuel-assemblies/{aid}/burnup   — append burnup record
GET    /v1/fuel-assemblies?status={s}     — list by status (in-core,
                                            in-pool, in-dry-storage,
                                            shipped)
```

Burnup records are append-only; corrections emit a new record that
references the prior record as `predecessor`. Fuel assembly
identifiers MUST persist for the assembly's full lifecycle from
fabrication through final disposition.

## §5 Operating State Stream

```
POST   /v1/plants/{pid}/operating-states  — append operating state
                                             record (typically batched)
POST   /v1/bulk/operating-states          — batched append
GET    /v1/operating-states/{sid}         — retrieve state record
GET    /v1/plants/{pid}/operating-states?from={t}&to={t}
                                          — query window
```

Operating-state records are summary observations at a configurable
cadence (typical: 5-minute or 15-minute intervals); high-rate
safety-system data is not exposed here.

## §6 Outages

```
POST   /v1/plants/{pid}/outages           — register an outage
GET    /v1/outages/{oid}                  — retrieve outage record
PATCH  /v1/outages/{oid}/end              — close outage
PATCH  /v1/outages/{oid}/post-startup-report — attach post-outage
                                              startup report
```

Closing an outage with `endedAt` triggers reconciliation between
`fuelAssembliesIn` / `fuelAssembliesOut` (PHASE-1 §6) and the
spent-fuel inventory; reconciliation mismatches return `409
Conflict` with type `urn:wia:nuclear-energy:assembly-imbalance`.

## §7 Radiation Protection

```
POST   /v1/plants/{pid}/radiation-protection  — append RP sample
GET    /v1/radiation-protection/{rid}         — retrieve sample
GET    /v1/plants/{pid}/personnel-dose-ledger — current ledgers
```

Personnel dose ledger queries return only opaque worker tokens with
their cumulative dose; clinical identity flows only inside the
adjacent radiation-protection standard's facade.

## §8 Spent-Fuel and Waste

```
POST   /v1/plants/{pid}/spent-fuel-inventory  — register inventory
                                                 snapshot
GET    /v1/spent-fuel/{iid}                   — retrieve snapshot
POST   /v1/plants/{pid}/waste                 — register waste
                                                 categorisation
GET    /v1/spent-fuel/{iid}/balance           — material balance
                                                 per safeguards
                                                 conventions
```

The material-balance endpoint is consumed by safeguards inspectors;
queries from non-safeguards client certificates return `403
Forbidden` with type
`urn:wia:nuclear-energy:safeguards-scope`.

## §9 Environmental Releases

```
POST   /v1/plants/{pid}/environmental-releases  — register report
GET    /v1/environmental-releases/{rid}         — retrieve report
GET    /v1/plants/{pid}/environmental-releases?period=...
                                                — query reports
```

Public consumers (researchers, NGOs, the public) read the reports
through the read-only profile; raw monitoring data references in
the report body are content-addressed but require a separate
authorisation to fetch the full archive.

## §10 Safety-Significant Events

```
POST   /v1/plants/{pid}/safety-events     — register an event
GET    /v1/safety-events/{eid}            — retrieve event record
PATCH  /v1/safety-events/{eid}/classification — update classification
PATCH  /v1/safety-events/{eid}/root-cause    — attach root-cause
                                              investigation
```

Event registrations of INES-2 and above automatically trigger an
outbound notification to the regulator and the IAEA Incident and
Emergency Centre via the integration described in PHASE-4 §10.

## §11 Decommissioning

```
POST   /v1/plants/{pid}/decommissioning   — register a decommissioning
                                             plan
GET    /v1/decommissioning/{did}          — retrieve plan
POST   /v1/decommissioning/{did}/milestones — append a milestone
```

## §12 Errors

All error responses are `application/problem+json` per RFC 9457.
Defined types include:

- `urn:wia:nuclear-energy:phase-transition`
- `urn:wia:nuclear-energy:assembly-imbalance`
- `urn:wia:nuclear-energy:safeguards-scope`
- `urn:wia:nuclear-energy:dose-policy-breach`
- `urn:wia:nuclear-energy:release-window-mismatch`
- `urn:wia:nuclear-energy:evidence-mismatch`

## §13 Authentication

The API uses mutually-authenticated TLS for regulator, safeguards
inspector, fuel-cycle facility, and market-operator connections.
Public read-only endpoints (released environmental reports, safety-
event summaries above operator-precursor classification, the
OpenAPI document) are reachable without a client certificate.

## §14 Caching

Stable resources (closed outages, released environmental reports,
signed evidence packages) are cacheable with `Cache-Control:
max-age=31536000, immutable`. Mutable resources (in-progress
outages, draft safety-event classifications) are cacheable for 60
seconds. ETags are mandatory on every PATCH endpoint with
`If-Match` conditional requests.

## §15 Worked Example: Cycle to Citation

1. Plant registers a new core configuration at refuelling outage.
2. Operating-state stream populates with at-power observations.
3. RP samples and personnel dose-ledger entries are appended
   throughout the cycle.
4. Environmental-release report is filed at the end of the
   reporting interval.
5. End-of-cycle outage starts; assembly movements update the
   spent-fuel inventory.
6. Citation tool requests an evidence package for the cycle;
   manifest digest is pinned for external reference.

## §16 Plant-State Document Endpoints

```
POST   /v1/plants/{pid}/plant-state-documents   — register a document
GET    /v1/plant-state-documents/{did}          — retrieve document
PATCH  /v1/plant-state-documents/{did}/regulator-approval
                                                — append approval ref
GET    /v1/plants/{pid}/plant-state-documents?category={c}&at={t}
                                                — query documents in
                                                  force at a time
```

Time-based queries return the documents that were in force at the
given timestamp; safety-event records reference the documents in
force at occurrence time so that retrospective audit can reconstruct
the licensing state without polling.

## §17 Streaming Subscriptions and Heartbeat

Regulators and energy-market consumers subscribe to plant events
via Server-Sent Events at `/v1/plants/{pid}/events`. Topics include
operating-state transitions, outage start/end, safety-event
classifications, environmental-release filings, and dose-policy
breaches. Subscriptions emit a heartbeat every 30 seconds; replays
support `Last-Event-ID` headers (W3C EventSource semantics).

## §18 Audit and Observability

Every endpoint emits structured logs with `plantId`, `traceId`, the
issuing client certificate's subject, and the plant clock skew vs
the reference NTP source. Audit logs retain indefinitely.

## §19 In-Service Inspection Endpoints

```
POST   /v1/plants/{pid}/isi-records      — register an ISI record
GET    /v1/isi-records/{iid}             — retrieve ISI record
PATCH  /v1/isi-records/{iid}/disposition — append engineering
                                            evaluation reference
```

ISI records of `indication-rejectable` outcomes are flagged in the
streaming subscription so that the operating organisation's
disposition queue receives the indication immediately.

## §20 Bulk and Pagination

Bulk endpoints accept arrays for high-volume submissions (operating
state, RP samples, ISI records). Cursor-based pagination uses the
`cursor` query parameter and `Link` headers (RFC 8288); cursors are
opaque to clients and persist for at least 24 hours.

## §21 Privacy-Preserving Aggregation

Public consumers (researchers, NGOs, regulators producing
sector-wide statistics) fetch aggregated trends through endpoints
that emit counts, means, and dispersions. Cohort-size policies
prevent inference-by-intersection on small fleets:

```
GET    /v1/aggregate/capacity-factor?period=...
GET    /v1/aggregate/dose-to-public?period=...
GET    /v1/aggregate/safety-events?ines-min=1&period=...
```

## §22 Conformance

A conformant server passes the test vectors published under
`tests/phase-vectors/phase-2-api-interface/`, emits an OpenAPI 3.1
document, signs evidence packages per RFC 9421, and rejects
operational-telemetry submissions on the metadata endpoints.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-nuclear-energy
- **Last Updated:** 2026-04-27
