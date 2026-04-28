# WIA-interplanetary-travel PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-interplanetary-travel
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTPS API contract that an
interplanetary-travel programme exposes for the records
defined in PHASE-1. Consumers include trajectory design teams,
deep-space tracking network operators, planetary-protection
authorities, regulators, partner agencies, and the operator's
own analytics and audit platforms.

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
- CCSDS 503.0-B / 504.0-B / 505.0-B / 508.0-B / 727.0-B
- COSPAR Planetary Protection Policy

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published by the operator.
Versioning uses `/v1/` path segments. The OpenAPI 3.1 document
at `/v1/openapi.json` is canonical.

Spacecraft command links and high-rate science telemetry do
NOT flow through this API; this API is the metadata, mission-
operations evidence, and inter-agency coordination facade.
Command-link traffic follows the operator's CCSDS-aligned
deep-space link directly.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-interplanetary-travel",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "missions":            "/v1/missions",
    "referenceFrames":     "/v1/reference-frames",
    "trajectories":        "/v1/trajectories",
    "consumableBudgets":   "/v1/consumable-budgets",
    "radiationLedgers":    "/v1/radiation-ledgers",
    "conjunctions":        "/v1/conjunctions",
    "ppCompliances":       "/v1/pp-compliances",
    "evidence":            "/v1/evidence",
    "openapi":             "/v1/openapi.json"
  }
}
```

## §3 Mission Lifecycle

```
POST   /v1/missions                — register a mission
GET    /v1/missions/{mid}          — retrieve mission
PATCH  /v1/missions/{mid}/status   — advance status
PATCH  /v1/missions/{mid}/cospar-category
                                   — update category
                                      (planetary-protection
                                      officer authorisation
                                      required)
```

Mission-status transitions follow the lifecycle in PHASE-1 §2;
invalid transitions return `422` with type
`urn:wia:interplanetary-travel:status-transition`. Category
changes that downgrade restrictiveness require COSPAR-aligned
re-review and PP officer signoff.

## §4 Reference Frames

```
POST   /v1/reference-frames        — register a frame
GET    /v1/reference-frames/{fid}  — retrieve frame
```

Reference-frame submissions whose `bodyKind` is `IAU-body-
fixed` MUST cite a SPICE kernel reference; submissions without
the kernel return `422` with type
`urn:wia:interplanetary-travel:spice-kernel-required`.

## §5 Trajectories

```
POST   /v1/missions/{mid}/trajectories       — register a
                                                trajectory
GET    /v1/trajectories/{tid}                — retrieve
                                                trajectory
PATCH  /v1/trajectories/{tid}/segments       — append a
                                                segment
GET    /v1/missions/{mid}/trajectories?
       designIteration={n}                    — list trajectory
                                                iterations
```

OEM artefacts are content-addressed; the API rejects OEM
submissions whose digest disagrees with the trajectory's
declared `orbitDataMessageRef` digest with type
`urn:wia:interplanetary-travel:oem-digest-mismatch`.

## §6 Consumable Budgets

```
POST   /v1/missions/{mid}/consumable-budgets — register a
                                                budget
GET    /v1/consumable-budgets/{bid}          — retrieve budget
GET    /v1/missions/{mid}/consumable-budgets?from={t}&to={t}
                                              — query window
```

Budgets that imply consumable depletion before the end of
mission return `422` with type
`urn:wia:interplanetary-travel:consumable-depletion-projected`.

## §7 Radiation Ledgers

```
POST   /v1/missions/{mid}/radiation-ledgers  — register a
                                                ledger interval
GET    /v1/radiation-ledgers/{lid}           — retrieve ledger
GET    /v1/missions/{mid}/radiation-ledgers?
       crew={t}&from={t}&to={t}               — query window
                                                (operator
                                                authorisation
                                                required;
                                                clinical identity
                                                never returned)
```

Ledger updates that would push cumulative-mission or career
dose past the operator's per-crew limits return `409 Conflict`
with type `urn:wia:interplanetary-travel:dose-limit-exceeded`
and trigger the operator's flight-surgeon escalation.

## §8 Conjunctions

```
POST   /v1/missions/{mid}/conjunctions       — register a
                                                conjunction
PATCH  /v1/conjunctions/{cid}/decision       — record decision
PATCH  /v1/conjunctions/{cid}/manoeuvre      — attach
                                                manoeuvre plan
                                                reference
GET    /v1/conjunctions/{cid}                — retrieve
                                                conjunction
```

Conjunction CDMs are content-addressed per CCSDS 508.0-B; the
API verifies the embedded probability against the operator's
chosen Pc model and rejects mismatches with type
`urn:wia:interplanetary-travel:cdm-probability-mismatch`.

## §9 Planetary-Protection Compliance

```
POST   /v1/missions/{mid}/pp-compliances     — register a
                                                compliance
                                                record
PATCH  /v1/pp-compliances/{cid}/state        — advance state
PATCH  /v1/pp-compliances/{cid}/bioburden    — attach
                                                bioburden assay
                                                reference
GET    /v1/pp-compliances/{cid}              — retrieve record
```

State transitions to `approved-for-launch` require an attached
bioburden assay reference and an approval-authority signoff;
submissions without both return `409 Conflict` with type
`urn:wia:interplanetary-travel:pp-approval-incomplete`.

## §10 Errors

All error responses are `application/problem+json` per RFC
9457. Defined types include those above plus:

- `urn:wia:interplanetary-travel:cospar-category-protect`
- `urn:wia:interplanetary-travel:cosparcategory-downgrade-blocked`
- `urn:wia:interplanetary-travel:evidence-mismatch`

## §11 Authentication

Mutually-authenticated TLS for trajectory-design tools, deep-
space tracking network operators, planetary-protection
authorities, regulators, and partner agencies. Public read-
only endpoints (mission summaries, post-end-of-mission
disposition records) are reachable without a client
certificate.

## §12 Caching, Concurrency, and Audit

Stable resources (frozen trajectory iterations, settled
conjunction decisions, post-launch PP records, signed evidence
packages) are cacheable with `Cache-Control: max-age=
31536000, immutable`. Mutable resources (in-design trajectory
iterations, in-flight consumable budgets) are cacheable for
60 seconds. ETags are mandatory on every PATCH endpoint.
Audit logs carry `missionId`, `traceId`, and the operator's
clock skew vs the operator's CCSDS 301.0-B reference.

## §13 Worked Example: Trajectory to Approval to Launch

1. The trajectory team registers a `design` iteration with
   ballistic-cruise + flyby segments and an OEM artefact.
2. The PP officer registers a compliance record at
   `planning`, attaches a bioburden assay, and advances to
   `interim-approved`.
3. The launch service provider files the launch corridor;
   conjunction screening pre-launch confirms no high-Pc
   conjunctions in the early operations window.
4. The operator advances mission status to `integrated-pre-
   launch` and the PP record to `approved-for-launch`.
5. Post-launch, the operator opens the in-cruise telemetry
   ingest path (out-of-band of this API), and registers
   subsequent trajectory iterations as the cruise unfolds.

## §14 Communications and Arrival Endpoints

```
POST   /v1/missions/{mid}/comms-schedules    — register a
                                                schedule
PATCH  /v1/comms-schedules/{sid}/status      — advance status
GET    /v1/comms-schedules/{sid}             — retrieve
                                                schedule
POST   /v1/missions/{mid}/arrivals           — register an
                                                arrival event
PATCH  /v1/arrivals/{aid}/outcome            — record arrival
                                                outcome
GET    /v1/arrivals/{aid}                    — retrieve arrival
```

Arrival outcomes of `loss` automatically trigger an outbound
notification to the COSPAR Panel on Planetary Protection
when the missed object's terminal state cannot be confirmed
to satisfy the mission's PP category disposal expectation
(PHASE-3 §8).

## §15 Bulk Operations and Pagination

Bulk endpoints accept arrays for high-volume tracking-pass
ingest, conjunction-feed ingest, and historical
trajectory-iteration backfill. Cursor-based pagination uses
the `cursor` query parameter and `Link` headers (RFC 8288)
with cursors persisted for at least 24 hours.

## §16 Streaming Subscription

Consumers subscribe via Server-Sent Events at:

- `/v1/missions/{mid}/events` — mission-wide events
  (status transitions, COSPAR category amendments, anomaly
  notifications).
- `/v1/conjunctions/{cid}/events` — conjunction-scoped
  events (probability re-computations, decision changes,
  manoeuvre execution confirmations).
- `/v1/arrivals/{aid}/events` — arrival-event scoped
  events (final-go reviews, telemetry arrival, reconstructed
  state publication).

Subscribers reconnect via the `Last-Event-ID` header (W3C
EventSource semantics) so that downstream operations consoles
do not lose visibility of priority-1 events during
reconnection windows.

## §17 Provenance and Aggregation

```
GET    /v1/provenance/{recordId}    — retrieve provenance
                                       entry for any PHASE-1
                                       record
GET    /v1/aggregate/conjunction-volume?period=...
GET    /v1/aggregate/tracking-pass-utilisation?network=...
```

Aggregate queries that exceed the operator's rate-limit
return `429 Too Many Requests` with `Retry-After`.

## §18 Anomaly Endpoints

```
POST   /v1/missions/{mid}/anomalies          — register an
                                                anomaly
PATCH  /v1/anomalies/{aid}/severity          — update severity
PATCH  /v1/anomalies/{aid}/root-cause        — attach root-
                                                cause reference
PATCH  /v1/anomalies/{aid}/closed            — close anomaly
GET    /v1/anomalies/{aid}                   — retrieve anomaly
```

Anomalies of severity `critical` or `loss` automatically
escalate to the agency-level anomaly investigation board
(PHASE-3 §17 / PHASE-4 §21).

## §19 Conformance

A conformant server passes the test vectors published under
`tests/phase-vectors/phase-2-api-interface/`, emits an OpenAPI
3.1 document, signs evidence packages per RFC 9421, and
rejects high-rate spacecraft telemetry on the metadata
endpoints.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-interplanetary-travel
- **Last Updated:** 2026-04-28
