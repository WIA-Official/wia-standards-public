# WIA-fusion-energy PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-fusion-energy
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTPS API contract that a
fusion-energy facility exposes for the records defined
in PHASE-1. Consumers include the operating
jurisdiction's nuclear-or-radiation-safety regulator
(US NRC, UK ONR, EU national regulators per Council
Directive 2009/71/Euratom and 2013/59/Euratom basic-
safety-standards, JP NRA, KR NSSC), the IAEA where the
operating jurisdiction reports voluntary fusion-safety
information, the host site's safety committee, the
facility's external technical-safety reviewer, the
facility's contracted ASME Code Section III nuclear-
component-quality-assurance auditor, and the
facility's plasma-physics community partners under the
operating jurisdiction's data-sharing rules.

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
- IAEA Safety Standards GSR Part 1 to 7 + fusion-
  specific Specific Safety Guides (SSG-77 / SSG-78 /
  SSG-79 in publication tracking)
- ASME BPVC Section III + ASME NQA-1
- IEC 61508 + IEC 60880

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published by the
facility. Versioning uses `/v1/` path segments. The
OpenAPI 3.1 document at `/v1/openapi.json` is canonical.

This API is the facility-facing facade for fusion-
safety records. Real-time plasma diagnostic data flows
through the facility's plasma-control surface; this
API records the artefacts of regulatory-grade
significance (safety case, postulated initiating
events, safety-classified components, operating
limits, tritium accountancy, reportable events).

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-fusion-energy",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "programmes":               "/v1/programmes",
    "safetyCases":              "/v1/safety-cases",
    "tritiumInventory":         "/v1/tritium-inventory",
    "postulatedEvents":         "/v1/postulated-events",
    "safetyClassifiedComponents": "/v1/safety-classified-components",
    "operatingLimits":          "/v1/operating-limits",
    "plasmaOperations":         "/v1/plasma-operations",
    "reportableEvents":         "/v1/reportable-events",
    "decommissioning":          "/v1/decommissioning",
    "evidence":                 "/v1/evidence",
    "openapi":                  "/v1/openapi.json"
  }
}
```

## §3 Programme Lifecycle

```
POST   /v1/programmes              — register a programme
GET    /v1/programmes/{pid}        — retrieve programme
PATCH  /v1/programmes/{pid}/operating-phase
                                   — advance operating
                                     phase
PATCH  /v1/programmes/{pid}/fuel-cycle
                                   — record fuel-cycle
                                     transition (e.g.
                                     hydrogen-only-
                                     research → deuterium
                                     -only-research →
                                     deuterium-tritium-
                                     low-inventory)
```

Programmes whose `fuelCycle` is `deuterium-tritium-
full-inventory` without an `effectiveSafetyCaseId`
return `409` with type
`urn:wia:fusion-energy:safety-case-required-for-
tritium`.

## §4 Safety Cases

```
POST   /v1/programmes/{pid}/safety-cases
                                   — register a safety
                                     case
PATCH  /v1/safety-cases/{scid}/regulator-approval
                                   — record regulator
                                     approval reference
PATCH  /v1/safety-cases/{scid}/superseded-by
                                   — record successor
                                     safety case
GET    /v1/safety-cases/{scid}     — retrieve safety
                                     case
GET    /v1/safety-cases/{scid}/public-summary
                                   — fetch the public
                                     summary version
                                     where the
                                     operating
                                     jurisdiction
                                     publishes one
```

Safety-case submissions whose `hazardCategorisation`
is `significant-tritium-or-activation-inventory`
without a regulator-approval reference cannot serve
as the `effectiveSafetyCaseId` and return `409` on
operating-phase advance with type
`urn:wia:fusion-energy:regulator-approval-required-
for-significant-hazard`.

## §5 Tritium-Inventory Records

For programmes whose `fuelCycle` includes tritium:

```
POST   /v1/programmes/{pid}/tritium-inventory
                                   — register a
                                     tritium-inventory
                                     measurement
GET    /v1/tritium-inventory/{tid} — retrieve record
GET    /v1/programmes/{pid}/tritium-inventory?location={l}&period={p}
                                   — query tritium
                                     accountancy
```

Tritium-inventory submissions whose
`uncertaintyClass` is `greater-than-10-percent`
return `422` with type
`urn:wia:fusion-energy:tritium-accountancy-uncertainty-
out-of-bound`. The accountancy is the central record-
keeping under the operating jurisdiction's safeguards
regime; the API enforces a per-period mass-balance
check across the fuel-cycle locations.

## §6 Postulated Initiating Events

```
POST   /v1/programmes/{pid}/postulated-events
                                   — register a
                                     postulated
                                     initiating event
PATCH  /v1/postulated-events/{eid}/protection-functions
                                   — link protection
                                     functions per
                                     PHASE-1 §6
GET    /v1/postulated-events/{eid} — retrieve record
```

The postulated-events register frames the safety case
and is the cross-reference point for safety-classified-
component qualification.

## §7 Safety-Classified Components

```
POST   /v1/programmes/{pid}/safety-classified-components
                                   — register a
                                     safety-classified
                                     component
PATCH  /v1/safety-classified-components/{cid}/qualification
                                   — record
                                     qualification
                                     reference
GET    /v1/safety-classified-components/{cid}
                                   — retrieve record
```

Components declared `safety-class-1` or `safety-class-
2` without an ASME BPVC Section III design-code
declaration (where the operating jurisdiction adopts
ASME BPVC for fusion components) return `409` with
type `urn:wia:fusion-energy:asme-bpvc-design-code-
required-for-safety-class`. Computer-based components
performing category A functions per IEC 60880 without
the IEC 60880 categorisation field return `422` with
type
`urn:wia:fusion-energy:iec-60880-category-required`.

## §8 Operating Limits

```
POST   /v1/programmes/{pid}/operating-limits
                                   — register an
                                     operating limit
PATCH  /v1/operating-limits/{lid}/surveillance
                                   — record
                                     surveillance
                                     completion
GET    /v1/operating-limits/{lid}  — retrieve record
```

Limits declared `safety-limit` without a `responseOn
Excursion` field return `422` with type
`urn:wia:fusion-energy:safety-limit-response-required`.

## §9 Plasma Operations

```
POST   /v1/programmes/{pid}/plasma-operations
                                   — register a
                                     discharge
PATCH  /v1/plasma-operations/{sid}/protection-invocation
                                   — record protection-
                                     function
                                     invocation
                                     during the shot
GET    /v1/plasma-operations/{sid} — retrieve record
```

Plasma operations on a programme whose effective
safety case has been superseded without a successor
safety case in force return `409` on `POST` with type
`urn:wia:fusion-energy:no-effective-safety-case-cannot-
operate`.

## §10 Reportable Events

```
POST   /v1/programmes/{pid}/reportable-events
                                   — register a
                                     reportable event
PATCH  /v1/reportable-events/{eid}/regulator-notification
                                   — record regulator
                                     notification
PATCH  /v1/reportable-events/{eid}/root-cause
                                   — record root-cause
                                     analysis
GET    /v1/reportable-events/{eid} — retrieve record
```

Reportable events of classification
`abnormal-tritium-release-above-permit` or
`personnel-dose-above-investigation-level` without a
`regulatorNotifiedAt` field within the operating
jurisdiction's reportable-event deadline return `409`
on `PATCH /root-cause` with type
`urn:wia:fusion-energy:regulator-notification-required-
for-classification`.

## §11 Decommissioning

```
POST   /v1/programmes/{pid}/decommissioning
                                   — register
                                     decommissioning
                                     phase
PATCH  /v1/decommissioning/{did}/phase
                                   — advance
                                     decommissioning
                                     phase
GET    /v1/decommissioning/{did}   — retrieve record
```

The waste-route reference is canonical at the
operating jurisdiction's radioactive-waste regulator;
this endpoint cites the regulator's record reference
rather than carrying the route inline.

## §12 Errors, Authentication, Caching, Audit

Errors: `application/problem+json` per RFC 9457 with
the types named above plus
`urn:wia:fusion-energy:evidence-mismatch`.
Authentication: mutually-authenticated TLS for
regulator, IAEA, ASME-NQA-1 auditor, technical-safety
reviewer, and partner consumers. Caching: stable
resources (superseded safety cases, closed reportable
events, completed decommissioning phases, archived
programmes) cacheable with `Cache-Control: max-age=
31536000, immutable`. Audit logs carry `programmeId`,
`shotId`, `traceId`, the issuing client certificate's
subject, and the facility's clock skew vs the
operating jurisdiction's NTP service.

## §13 Streaming Subscription, Bulk, Pagination, Provenance

SSE at `/v1/programmes/{pid}/events` for programme-
wide events (safety case approved, tritium inventory
out-of-band, protection-function invocation,
reportable-event detected). Subscribers reconnect via
`Last-Event-ID`. Bulk endpoints: `/v1/bulk/plasma-
operations`, `/v1/bulk/operating-limits`,
`/v1/bulk/safety-classified-components`. Cursor-based
pagination via `cursor` and `Link` headers. Provenance
via `/v1/provenance/{recordId}` emits the in-toto
attestation chain for any record.

## §14 Worked Example: Tritium Fuel-Cycle Transition

1. Facility operating in `deuterium-only-research`
   prepares for transition to `deuterium-tritium-low-
   inventory`.
2. Operator registers an updated safety case via
   `POST /safety-cases` with revised hazard
   categorisation `significant-tritium-or-activation-
   inventory` and the design-basis-events analysis
   updated to include tritium-bypass and tritium-
   release scenarios.
3. Regulator review per the operating jurisdiction's
   regulatory pathway (`us-nrc-risk-informed-
   performance-based-fusion`, `uk-onr-fusion-safety`,
   etc.); on approval, the regulator-approval
   reference is recorded via `PATCH /regulator-
   approval`.
4. Tritium-inventory baseline measurement is recorded
   via `POST /tritium-inventory` for each fuel-cycle
   location.
5. Safety-classified-component qualifications updated
   for the tritium-handling components (per ASME BPVC
   Section III where adopted, with IEC 61508 / IEC
   60880 evidence for the protection system).
6. Fuel-cycle transition executed via `PATCH /fuel-
   cycle`; first tritium-introducing shot recorded
   via `POST /plasma-operations`.

## §15 Data-Sharing-Partner Endpoint

```
GET    /v1/programmes/{pid}/plasma-operations?shareable=true
                                   — list discharges
                                     marked shareable
                                     under the
                                     operating
                                     jurisdiction's
                                     data-sharing
                                     rules
```

The plasma-physics community shares reduced datasets
under the operating jurisdiction's data-sharing rules
and the IAEA's data-exchange guidance; tritium and
safety-classified data are NOT shared through this
endpoint.

## §16 Aggregate and Provenance Endpoints

```
GET    /v1/provenance/{recordId}
GET    /v1/aggregate/discharge-count?period=...
GET    /v1/aggregate/protection-invocation-count?period=...
GET    /v1/aggregate/tritium-inventory-mass-balance?period=...
```

## §17 Conformance

A conformant server passes the test vectors published
under `tests/phase-vectors/phase-2-api-interface/`,
emits an OpenAPI 3.1 document, signs evidence packages
per RFC 9421, refuses operation under a superseded
safety case without successor in force, refuses
tritium-fuel-cycle programmes without a regulator-
approved safety case, and refuses safety-class-1 / 2
components without ASME BPVC Section III design-code
declarations.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-fusion-energy
- **Last Updated:** 2026-04-28
