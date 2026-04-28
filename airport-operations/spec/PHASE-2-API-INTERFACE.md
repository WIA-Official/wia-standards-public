# WIA-airport-operations PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-airport-operations
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the API contract that an
airport operator exposes for the records defined in
PHASE-1. Three complementary surfaces are described:
the SWIM (System-Wide Information Management) flight-
data and surface-data surface; the IATA AHM messaging
surface for ground-handling coordination; and the
HTTPS / JSON RESTful surface for operational
visibility, the airline customer-facing channel, the
SMS reporting channel, and the regulatory examination
scope.

References (CITATION-POLICY ALLOW only):

- ICAO Annex 14 + Doc 9981 + Doc 9859 + Doc 4444
- ICAO Doc 9870 + Doc 9137
- IATA AHM 803 / 810 / 911 / 913
- IATA Resolution 753
- IATA RP 1750 (A-CDM)
- EUROCAE ED-99 + ED-87 (A-SMGCS)
- EUROCAE ED-133 (Flight Object Interoperability)
- ICAO FIXM (Flight Information Exchange Model)
- ICAO AIXM (Aeronautical Information Exchange
  Model)
- ARINC 424 + ARINC 620
- IETF RFC 9110 (HTTP Semantics), RFC 9111 (HTTP
  Caching), RFC 9457 (Problem Details), RFC 6901 /
  6902 (JSON Pointer / Patch), RFC 8288 (Web
  Linking), RFC 8259 (JSON), RFC 9421 (HTTP Message
  Signatures)
- ISO 8601 (date and time)
- ISO/IEC 27001:2022
- W3C Trace Context

---

## §1 Scope and Versioning

The operator exposes:

- The SWIM endpoints carrying FIXM (flight) and
  AIXM (aeronautical information) messages.
- The IATA AHM messaging endpoints for ground-
  handler coordination.
- The HTTPS / JSON RESTful surface served from a
  domain published by the operator under `/v1/`.

The OpenAPI 3.1 document at `/v1/openapi.json` is
canonical for the JSON surface; the FIXM XSD schema
and the AIXM 5.1.1 schema are canonical for SWIM.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-airport-operations",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "programmes":              "/v1/programmes",
    "movementArea":            "/v1/movement-area",
    "airfieldLighting":        "/v1/airfield-lighting",
    "acdmMilestones":          "/v1/acdm-milestones",
    "groundHandling":          "/v1/ground-handling",
    "deicing":                 "/v1/deicing",
    "incursionRecords":        "/v1/incursion-records",
    "wildlifeStrikes":         "/v1/wildlife-strikes",
    "fodRecords":              "/v1/fod-records",
    "aepRecords":              "/v1/aep-records",
    "smsRecords":              "/v1/sms-records",
    "examination":             "/v1/examination",
    "openapi":                 "/v1/openapi.json"
  }
}
```

## §3 SWIM FIXM / AIXM Surface

The operator's SWIM endpoints expose:

```
POST   /swim/fixm/flight                (FIXM Flight
                                         Object updates)
GET    /swim/fixm/flights?airport={icao}
POST   /swim/aixm/aerodrome              (AIXM Aerodrome
                                         feature
                                         updates)
GET    /swim/aixm/aerodrome/{icao}       (AIXM 5.1.1
                                         GML payload)
```

FIXM Flight Object updates carry the EUROCAE ED-133
Flight Object Interoperability fields; AIXM
features cover the runway, taxiway, apron, and
navigation-aid hierarchy.

## §4 IATA AHM Messaging Surface

```
POST   /ahm/movement-message      (IATA AHM 780
                                   movement message)
POST   /ahm/load-message          (IATA AHM 350 /
                                   355 loadsheet)
POST   /ahm/loadplan-message      (IATA AHM
                                   loadplan)
POST   /ahm/passenger-final-loadsheet
                                  (IATA AHM final
                                   loadsheet
                                   confirmation)
POST   /ahm/baggage-tracking      (IATA Resolution
                                   753 bag-event)
```

## §5 Movement-Area and Airfield-Lighting Endpoints

```
GET    /v1/movement-area/runways
GET    /v1/movement-area/runways/{runwayId}
GET    /v1/movement-area/taxiways
GET    /v1/movement-area/aprons
GET    /v1/airfield-lighting
PATCH  /v1/airfield-lighting/{lightingId}/intensity
GET    /v1/airfield-lighting/{lightingId}/inspection-history
```

## §6 A-CDM and Ground-Handling Endpoints

```
GET    /v1/acdm-milestones?flight={flightId}
POST   /v1/acdm-milestones
GET    /v1/ground-handling?flight={flightId}
POST   /v1/ground-handling
GET    /v1/ground-handling/{recordId}/iata-753-events
```

## §7 De-Icing Endpoints

```
GET    /v1/deicing?flight={flightId}
POST   /v1/deicing
GET    /v1/deicing/{recordId}/holdover-status
```

## §8 Incursion / Wildlife / FOD Endpoints

```
POST   /v1/incursion-records
GET    /v1/incursion-records?from={iso}&to={iso}
POST   /v1/wildlife-strikes
GET    /v1/wildlife-strikes/{strikeId}/ibis-export
       (ICAO IBIS-format export of the wildlife-
        strike report)
POST   /v1/fod-records
PATCH  /v1/fod-records/{fodId}/removed
```

## §9 Airport-Emergency-Plan Endpoints

```
GET    /v1/aep-records
POST   /v1/aep-records              (record an AEP
                                     activation
                                     event)
GET    /v1/aep-records/{aepId}/drill-history
```

## §10 SMS Reporting Endpoints

```
POST   /v1/sms-records              (file a hazard /
                                     occurrence
                                     report)
GET    /v1/sms-records/{recordId}
PATCH  /v1/sms-records/{recordId}/risk-assessment
PATCH  /v1/sms-records/{recordId}/corrective-actions
```

The SMS surface accepts voluntary-reporter and
mandatory-occurrence-reporter submissions; for EU-
jurisdiction operators the mandatory occurrence
reporting under Regulation (EU) 376/2014 is
forwarded to the European Central Repository through
the operator's eccairs adapter.

## §11 Examination Endpoints

```
GET    /v1/examination/programmes
GET    /v1/examination/movement-area
GET    /v1/examination/sms-records
GET    /v1/examination/incursion-records
GET    /v1/examination/aep-records
GET    /v1/examination/audit-events
```

The examination scope is read-only and bound to the
authority's identity (FAA Office of Airports for US;
EASA for EU; KR 국토교통부 항공정책실 for KR; ICAO
audit teams under USOAP).

## §12 Authentication and Authorisation

Bearer tokens conform to OAuth 2.1 with audiences
declared per surface. SWIM endpoints use TLS with
mutual authentication. Internal subsystem-to-
subsystem calls use mutual TLS with the operator's
internal certificate authority.

## §13 HTTP Status Codes

Standard codes apply (200 / 201 / 202 / 400 / 401 /
403 / 404 / 409 / 422 / 429 / 503) with Problem
Details bodies.

## §14 Webhook and Event Surface

Lifecycle events:

- `acdm.milestone-emitted`
- `ground-handling.service-completed`
- `deicing.holdover-expired`
- `incursion.detected`
- `wildlife-strike.reported`
- `fod.detected`, `fod.removed`
- `aep.activated`, `aep.deactivated`
- `sms.report-filed`, `sms.corrective-action-closed`

Webhook signatures use HTTP Message Signatures (RFC
9421).

## §15 Bulk-Export and Audit Surface

```
POST   /v1/bulk-export
GET    /v1/bulk-export/{exportId}/status
GET    /v1/bulk-export/{exportId}/manifest
GET    /v1/audit-events?from={iso}&to={iso}
```

Bulk exports support the supervisory authority's
periodic audit requests (FAA 14 CFR 139 inspection,
EASA aerodrome inspection, ICAO USOAP audit, KR
국토교통부 항공안전감독). Audit-event retrieval
supports the operator's internal investigation
workflow.

## §16 Passenger-Facing Operational Information
        Surface

For passenger-facing flight information:

```
GET    /portal/v1/flights/arrivals
GET    /portal/v1/flights/departures
GET    /portal/v1/flights/{flightNumber}
GET    /portal/v1/security-wait-times
GET    /portal/v1/parking-availability
GET    /portal/v1/gate-changes      (server-sent
                                     events)
```

The passenger-portal surface follows the operator's
i18n catalogue and the WCAG 2.2 + EU EAA + ADA
accessibility discipline cross-referenced from
WIA-content-ai.

## §17 NOTAM and Snowtam Surface

```
GET    /v1/notam              (current NOTAMs at the
                               operator's aerodrome)
POST   /v1/notam              (originate a NOTAM)
GET    /v1/snowtam            (current SNOWTAMs)
POST   /v1/snowtam            (originate a SNOWTAM
                               with GRF runway-
                               condition-code)
```

NOTAMs follow the ICAO Doc 8126 + Doc 10066 (PANS-
AIM) format; SNOWTAMs follow the new GRF-aligned
format introduced in ICAO Doc 10064 + Annex 14
Amendment 14B. The operator's NOTAM origination is
coordinated with the national NOTAM office for
international distribution.

## §18 SWIM Subscription Surface

```
GET    /swim/subscriptions
POST   /swim/subscriptions    (subscribe to FIXM /
                               AIXM / WXXM update
                               streams)
DELETE /swim/subscriptions/{subscriptionId}
```

Subscriptions support the airline / handler /
ANSP-side consumers receiving the operator's
flight-data and aerodrome-data updates.

## §19 Examination Bulk-Download Surface

```
GET    /v1/examination/incursion-records.csv
GET    /v1/examination/wildlife-strikes.csv
GET    /v1/examination/sms-records.csv
GET    /v1/examination/ground-handling.csv
GET    /v1/examination/aep-drills.csv
```

The CSV downloads support the supervisory authority's
trend analysis and the operator's annual safety-
performance report. Per-export digest declarations
in the manifest preserve integrity for the receiving
authority.

## §20 Real-Time Surface-Movement Telemetry

For consumers integrating the airport's A-SMGCS:

```
GET    /v1/asmgcs/surface-track-stream      (server-
                                              sent
                                              events)
GET    /v1/asmgcs/conflict-alerts             (real-
                                               time
                                               alerts)
```

The surface-track stream carries the per-target
position, velocity, and identification per EUROCAE
ED-87 + ED-99 (A-SMGCS Levels 1-4).

## §21 Conformance

Implementations claiming PHASE-2 conformance publish
the OpenAPI document, expose the SWIM FIXM / AIXM
surfaces relevant to the operator's role, expose the
IATA AHM messaging surface for ground-handler
coordination, expose the regulatory examination
surface, and propagate trace-context across the
flight-to-departure chain.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-airport-operations
- **Last Updated:** 2026-04-28
