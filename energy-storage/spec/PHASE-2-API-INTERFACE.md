# WIA-energy-storage PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-energy-storage
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the API contract that an
energy-storage operator exposes for the records
defined in PHASE-1. Three complementary surfaces are
described: the IEEE 2030.5 + SunSpec Modbus surface
(cross-domain reference to WIA-distributed-energy)
for asset-level monitoring and control; the safety-
event reporting surface for thermal-runaway / off-
gas / fire incidents; and the HTTPS / JSON RESTful
surface for operational visibility, certification
records, lifecycle tracking, and supervisory
examination.

References (CITATION-POLICY ALLOW only):

- IEEE 2030.5-2018 + SunSpec Alliance Modbus
- IEC 61850-7-420:2021 + 90-7 (cross-domain
  reference)
- IEC 62933-1 + 62933-2-1 + 62933-2-2 + 62933-5-2
- IETF RFC 9110 (HTTP Semantics), RFC 9111 (HTTP
  Caching), RFC 9457 (Problem Details), RFC 6901 /
  6902 (JSON Pointer / Patch), RFC 8288 (Web
  Linking), RFC 8259 (JSON), RFC 9421 (HTTP Message
  Signatures)
- ISO 8601 (date and time)
- ISO/IEC 27001:2022, ISO/IEC 27019:2024
- W3C Trace Context

---

## §1 Scope and Versioning

The operator exposes:

- The IEEE 2030.5 + SunSpec Modbus monitoring
  surface (cross-reference to WIA-distributed-
  energy PHASE-2).
- The HTTPS / JSON RESTful surface served from a
  domain published by the operator under `/v1/`.
- The safety-event reporting surface integrated with
  the operating jurisdiction's authority having
  jurisdiction (AHJ) / fire department / electrical
  inspector / KR 한국전기안전공사.

The OpenAPI 3.1 document at `/v1/openapi.json` is
canonical for the JSON surface.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-energy-storage",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "programmes":              "/v1/programmes",
    "storageAssets":           "/v1/storage-assets",
    "certifications":          "/v1/certifications",
    "bmsConfigurations":       "/v1/bms-configurations",
    "stateRecords":            "/v1/state-records",
    "cycleLifeRecords":        "/v1/cycle-life-records",
    "marketParticipations":    "/v1/market-participations",
    "incidents":               "/v1/incidents",
    "endOfLifeRecords":        "/v1/end-of-life-records",
    "examination":             "/v1/examination",
    "openapi":                 "/v1/openapi.json"
  }
}
```

## §3 Storage Asset and Certification Endpoints

```
GET    /v1/storage-assets
GET    /v1/storage-assets/{assetId}
POST   /v1/storage-assets
PATCH  /v1/storage-assets/{assetId}
GET    /v1/certifications?asset={assetId}
POST   /v1/certifications
GET    /v1/certifications/{certificationId}
GET    /v1/certifications/{certificationId}/9540a-reports
       (cell-, module-, unit-, installation-level
        UL 9540A test reports)
```

## §4 BMS Configuration and State-Telemetry Endpoints

```
GET    /v1/bms-configurations?asset={assetId}
GET    /v1/bms-configurations/{bmsId}
PATCH  /v1/bms-configurations/{bmsId}/firmware
       (firmware-update event recording — secure-
        boot signed-firmware policy required)
GET    /v1/state-records?asset={assetId}&from={iso}&to={iso}
GET    /v1/state-records/{recordId}
GET    /v1/state-records/realtime/{assetId}
       (server-sent events stream of the per-asset
        state telemetry)
```

## §5 Cycle-Life and Degradation Endpoints

```
GET    /v1/cycle-life-records?asset={assetId}&period={iso}
POST   /v1/cycle-life-records
GET    /v1/cycle-life-records/{recordId}
GET    /v1/cycle-life-records/{recordId}/degradation-forecast
       (the forward end-of-life forecast under the
        operator's degradation model)
```

## §6 Market Participation Endpoints

```
GET    /v1/market-participations?asset={assetId}
GET    /v1/market-participations/{participationId}
POST   /v1/market-participations/{participationId}/bids
GET    /v1/market-participations/{participationId}/awards
GET    /v1/market-participations/{participationId}/settlements
```

## §7 Incident-Reporting Endpoints

```
POST   /v1/incidents                  (record a
                                       thermal-runaway
                                       / off-gas /
                                       fire / smoke
                                       incident)
GET    /v1/incidents?asset={assetId}
GET    /v1/incidents/{incidentId}
PATCH  /v1/incidents/{incidentId}/post-mortem
       (record the NFPA 855 + UL 9540A-aligned post-
        mortem narrative)
PATCH  /v1/incidents/{incidentId}/ahj-reported
       (record AHJ notification timestamp)
```

The incident-reporting surface integrates with the
operating jurisdiction's emergency-services and AHJ
notification channels — the operator's incident
record is forwarded automatically where the
jurisdiction's regime requires.

## §8 End-of-Life and Recycling Endpoints

```
GET    /v1/end-of-life-records?asset={assetId}
POST   /v1/end-of-life-records
GET    /v1/end-of-life-records/{recordId}
GET    /v1/end-of-life-records/{recordId}/recovery-attestation
       (the recycler's per-material recovery
        attestation — Li / Ni / Co / Cu fractions)
```

## §9 Examination Endpoints

```
GET    /v1/examination/programmes
GET    /v1/examination/storage-assets
GET    /v1/examination/certifications
GET    /v1/examination/incidents
GET    /v1/examination/cycle-life-records
GET    /v1/examination/end-of-life-records
GET    /v1/examination/audit-events
```

The examination scope is read-only and bound to the
authority's identity (NERC + FERC + state-PUC for
US-jurisdiction; ENISA + Member-State NCA for EU-
jurisdiction; KR FSC + 산업통상자원부 + 소방청 +
한국전기안전공사 for KR-jurisdiction).

## §10 Authentication and Authorisation

Bearer tokens conform to OAuth 2.1 with audiences
declared per surface. Internal subsystem-to-subsystem
calls use mutual TLS with the operator's internal
certificate authority. The incident-reporting and
firmware-update endpoints require elevated scope
and four-eyes approval.

## §11 HTTP Status Codes

Standard codes apply (200 / 201 / 202 / 400 / 401 /
403 / 404 / 409 / 422 / 429 / 503) with Problem
Details bodies.

## §12 Webhook and Event Surface

Lifecycle events through a webhook channel:

- `storage-asset.commissioned`,
  `storage-asset.retired`.
- `bms.firmware-updated`.
- `state.threshold-breached` (over-temperature,
  over-voltage, etc.).
- `incident.detected`, `incident.resolved`,
  `incident.ahj-reported`.
- `cycle-life.warranty-claim-filed`.
- `end-of-life.decision-recorded`.
- `market-participation.award-received`.

Signatures use HTTP Message Signatures (RFC 9421).

## §13 Streaming Telemetry Surface

For real-time per-asset state telemetry the
operator exposes:

- WebSocket Secure (WSS) endpoints for browser-
  class consumers.
- Server-Sent Events (SSE) for one-way streaming.
- AMQP / MQTT for backend consumers.

The streaming surface follows the operator's
published QoS budget per topic.

## §14 Caching, Trace-Context

`ETag` carries the resource's version-id. PII /
operational-sensitive responses use `Cache-Control:
private, no-store`. Trace-context (`traceparent`) is
propagated across the operator's pipeline.

## §15 Bulk-Export and Forensic Surface

```
POST   /v1/bulk-export
GET    /v1/bulk-export/{exportId}/status
GET    /v1/bulk-export/{exportId}/manifest
GET    /v1/forensic/incident-reconstruction?incident={incidentId}
```

The forensic-reconstruction endpoint produces the
deterministic post-incident packaging — the BMS
state record at and around the incident, the cell-
voltage / temperature time-series, the fire-
suppression system response timeline, and the AHJ-
notification audit trail. The packaging is signed
using the operator's signing key for downstream
forensic consumers (insurance investigation, AHJ
review, manufacturer's recall investigation).

## §16 Battery-Passport Surface (EU Battery Regulation
        2023/1542)

For storage operators placing batteries on the EU
market under Reg (EU) 2023/1542:

```
GET    /v1/battery-passport/{assetId}
GET    /v1/battery-passport/{assetId}/qr-code
GET    /v1/battery-passport/{assetId}/recycled-content
GET    /v1/battery-passport/{assetId}/carbon-footprint
GET    /v1/battery-passport/{assetId}/due-diligence-attestation
```

The battery-passport surface satisfies Article 77
of Reg 2023/1542 — the persistent, machine-readable
record carrying chemistry, performance, recycled-
content, carbon-footprint, and raw-material due-
diligence information for each industrial or EV
battery placed on the EU market.

## §17 Examination Surface for Storage Audits

Supervisory authorities and external auditors with
the operator's examination scope read records
through:

```
GET    /v1/examination/programmes
GET    /v1/examination/storage-assets
GET    /v1/examination/certifications
GET    /v1/examination/incidents
GET    /v1/examination/cycle-life-records
GET    /v1/examination/end-of-life-records
GET    /v1/examination/audit-events
GET    /v1/examination/battery-passport-summary
       (EU Battery Regulation 2023/1542 examination
        view)
GET    /v1/examination/incident-rate-by-chemistry
       (statistical aggregation across the operator's
        fleet — for AHJ / regulator trend analysis)
```

The examination surface is read-only and bound to
the supervisory authority's identity (NERC + FERC +
state-PUC + AHJ for US-jurisdiction; ENISA + Member-
State NCA + EU Battery Authority for EU-jurisdiction;
KR FSC + 산업통상자원부 + 소방청 + 한국전기안전공사
+ 환경부 for KR-jurisdiction).

## §18 Conformance

Implementations claiming PHASE-2 conformance publish
the OpenAPI document, expose the IEEE 2030.5 +
SunSpec monitoring surface for the operator's storage
assets, expose the incident-reporting surface
integrated with the operating jurisdiction's AHJ /
emergency-services notification, and propagate trace-
context across the state-to-incident chain.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-energy-storage
- **Last Updated:** 2026-04-28
