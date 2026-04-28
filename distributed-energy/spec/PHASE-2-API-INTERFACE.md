# WIA-distributed-energy PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-distributed-energy
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the API contract that a
distributed-energy operator exposes for the records
defined in PHASE-1. Three complementary surfaces are
described: the IEEE 2030.5 Smart Energy Profile
surface — the canonical IP-based application-layer
protocol for utility-DER communication; the SunSpec
Modbus surface for inverter-level monitoring; the
OpenADR 2.0 + OCPP 2.0.1 surfaces for demand-response
and EV-charging coordination; and the HTTPS / JSON
RESTful surface for operational visibility, the
operator's compliance functions, and the supervisory
or balancing-authority examination scope.

References (CITATION-POLICY ALLOW only):

- IEEE 2030.5-2018 (Smart Energy Profile)
- SunSpec Alliance Modbus specification
- OpenADR Alliance OpenADR 2.0a / 2.0b
- Open Charge Alliance OCPP 2.0.1
- IEC 61850-7-420:2021 (DER object model)
- IETF RFC 9110 (HTTP Semantics), RFC 9111 (HTTP
  Caching), RFC 9457 (Problem Details), RFC 6901 /
  6902 (JSON Pointer / Patch), RFC 8288 (Web Linking),
  RFC 8259 (JSON), RFC 9421 (HTTP Message Signatures)
- ISO 8601 (date and time)
- ISO/IEC 27001:2022, ISO/IEC 27019:2024 (information
  security in the energy industry)
- W3C Trace Context

---

## §1 Scope and Versioning

The operator exposes:

- The IEEE 2030.5-2018 endpoints over HTTPS / TLS
  per the IEEE 2030.5 specification (REST resource
  hierarchy).
- The SunSpec Modbus endpoints (Modbus TCP / RTU)
  for inverter-level monitoring.
- The OpenADR 2.0b VTN ↔ VEN endpoints over HTTPS.
- The OCPP 2.0.1 endpoints over WebSocket Secure
  for EV-supply equipment.
- The HTTPS / JSON RESTful surface served from a
  domain published by the operator under `/v1/`.

The OpenAPI 3.1 document at `/v1/openapi.json` is
canonical for the JSON surface; the IEEE 2030.5
device-capability list, the SunSpec model registry,
the OpenADR conformance certificate, and the OCPP
conformance certificate are canonical for their
respective surfaces.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-distributed-energy",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "programmes":              "/v1/programmes",
    "derAssets":               "/v1/der-assets",
    "interconnections":        "/v1/interconnections",
    "derControllers":          "/v1/der-controllers",
    "ieee20305":               "/sep2/",
    "sunspec":                 "/sunspec/",
    "openadr":                 "/openadr/",
    "ocpp":                    "/ocpp/",
    "events":                  "/v1/events",
    "examination":             "/v1/examination",
    "openapi":                 "/v1/openapi.json"
  }
}
```

## §3 IEEE 2030.5 Smart Energy Profile Surface

The IEEE 2030.5-2018 resource hierarchy:

```
GET  /sep2/
GET  /sep2/dcap                 (DeviceCapability)
GET  /sep2/edev/{eId}/der       (DER function set)
PUT  /sep2/edev/{eId}/der/dera  (DERAvailability)
PUT  /sep2/edev/{eId}/der/derc  (DERCapability)
PUT  /sep2/edev/{eId}/der/derg  (DERSettings —
                                 voltage-volt-var,
                                 watt-volt-var,
                                 watt-power-factor,
                                 voltage-frequency
                                 ride-through curves)
GET  /sep2/edev/{eId}/derp      (DER program)
GET  /sep2/mr                   (MeterReading)
GET  /sep2/drlc                 (Demand-Response
                                 Load-Control)
```

The CSIP (Common Smart Inverter Profile) constrains
the IEEE 2030.5 surface to the subset that California
Rule 21 + the IEEE 1547-2018 reactive-power /
ride-through requirements demand; the Australia CSIP-
AUS variant applies for AS/NZS-jurisdiction
deployments.

## §4 SunSpec Modbus Surface

The SunSpec Modbus surface follows the inverter's
published model map:

```
Common Model 1            (manufacturer + model +
                           serial number)
Inverter Model 101-103    (single-/split-/three-
                           phase inverter telemetry)
Storage Model 124         (storage-specific control)
DER Models 701-705        (IEEE 2030.5-aligned DER
                           function set)
Network Model 11/12       (NTP, time)
```

Each model exposes its register map per the SunSpec
specification; the operator's monitoring agent reads
on the operator's polling cadence.

## §5 OpenADR 2.0 Surface

The OpenADR 2.0b VTN-to-VEN message exchange:

```
POST /openadr/oadrPoll        (VEN polls VTN for
                               events)
POST /openadr/oadrCreatedEvent (VEN acknowledges
                               event)
POST /openadr/oadrDistributeEvent (VTN distributes
                                   event payload)
POST /openadr/oadrResponse     (acknowledgement /
                                error)
POST /openadr/oadrUpdateReport (VEN reports back)
```

OpenADR profiles A and B are both supported; profile
B carries the price-relative and absolute-price
signal types in addition to simple level signals.

## §6 OCPP 2.0.1 Surface

The OCPP 2.0.1 messages over WebSocket Secure:

```
BootNotification, Heartbeat, StatusNotification
TransactionEvent (Started, Updated, Ended)
Authorize, MeterValues
SetChargingProfile, GetChargingProfiles,
ClearChargingProfile, ReportChargingProfiles
GetCompositeSchedule
NotifyChargingLimit
DataTransfer (vendor-extension)
```

ISO 15118-2:2014 + ISO 15118-20:2022 plug-and-charge
authentication and bidirectional V2G is layered above
OCPP 2.0.1.

## §7 DER Asset and Interconnection Endpoints

```
GET    /v1/der-assets
GET    /v1/der-assets/{assetId}
POST   /v1/der-assets
PATCH  /v1/der-assets/{assetId}
GET    /v1/interconnections?asset={assetId}
POST   /v1/interconnections
GET    /v1/interconnections/{interconnectionId}
```

## §8 DER Controller and DERMS Endpoints

```
GET    /v1/der-controllers
GET    /v1/der-controllers/{controllerId}
POST   /v1/der-controllers
GET    /v1/derms
GET    /v1/derms/{dermsId}
POST   /v1/derms/{dermsId}/dispatch     (issue a
                                         dispatch
                                         instruction
                                         to a
                                         controller)
```

## §9 Event and Ride-Through Endpoints

```
GET    /v1/events?asset={assetId}&from={iso}&to={iso}
GET    /v1/events/{eventId}
GET    /v1/events/{eventId}/comtrade   (download the
                                        IEEE C37.111
                                        COMTRADE
                                        waveform
                                        record where
                                        captured)
```

## §10 Examination Endpoints

```
GET    /v1/examination/programmes
GET    /v1/examination/der-assets
GET    /v1/examination/interconnections
GET    /v1/examination/cybersecurity-posture
GET    /v1/examination/events
GET    /v1/examination/storage-records
GET    /v1/examination/audit-events
```

The examination scope is read-only and bound to the
authority's identity (NERC for US bulk-electric-
system operators; FERC for FERC-jurisdictional
markets; KR FSC + 한국에너지공단 + KEPCO for KR-
jurisdiction; the local Public Utility Commission
for the operator's service territory).

## §11 Authentication and Authorisation

IEEE 2030.5 endpoints use TLS with mutual
authentication per the specification's certificate
discipline. SunSpec Modbus over TCP is layered behind
the operator's network-segmentation firewall; Modbus
TCP/Secure (the IEC 62351-derived secure variant) is
the recommended mode where supported. OpenADR endpoints
use TLS with the OpenADR-published certificate-
authority chain. OCPP endpoints use WSS with TLS
client authentication. The HTTPS / JSON surface uses
OAuth 2.1 bearer tokens per surface.

## §12 HTTP Status Codes

Standard codes apply (200 / 201 / 202 / 400 / 401 /
403 / 404 / 409 / 422 / 429 / 503) with Problem
Details bodies; IEEE 2030.5 / OpenADR / OCPP each
follow their published response-code semantics.

## §13 Caching and Trace-Context

`ETag` carries the resource's version-id. Trace-
context (`traceparent`) is propagated across the
operator's pipeline. Real-time telemetry endpoints
use `Cache-Control: no-store`.

## §14 Webhook and Event Surface

The operator publishes lifecycle events through a
webhook channel:

- `der-asset.commissioned`, `der-asset.retired`.
- `interconnection.approved`,
  `interconnection.amended`.
- `event.ride-through-detected`,
  `event.thermal-runaway-detected`.
- `derms.dispatch-issued`,
  `derms.dispatch-acknowledged`.
- `openadr.event-distributed`,
  `openadr.opt-changed`.
- `ocpp.transaction-started`,
  `ocpp.transaction-ended`.

Webhook signatures use HTTP Message Signatures (RFC
9421).

## §15 Telemetry Bulk-Export and Examination Surface

```
POST   /v1/bulk-export
GET    /v1/bulk-export/{exportId}/status
GET    /v1/bulk-export/{exportId}/manifest
GET    /v1/examination/forecast-vs-actual?asset={assetId}&period={iso}
```

Bulk exports support the supervisory authority's
periodic data calls and the wholesale-market
settlement reconciliation (FERC Order 2222 settlement-
data retrieval, ISO 14064-3 audit data calls). The
examination forecast-vs-actual endpoint surfaces the
asset's IEEE 2030.5 DERAvailability forecasts against
its actual output for the requested period — the
RTO / ISO settlement engine uses this data for
performance reconciliation under the operator's
demand-response or capacity programme.

## §16 Per-Asset State Snapshot Surface

```
GET    /v1/der-assets/{assetId}/state
GET    /v1/der-assets/{assetId}/state/history?from={iso}&to={iso}
```

The state snapshot returns the asset's instantaneous
state-of-charge (storage), output power, voltage /
frequency observations, ride-through curve in effect,
DERControl in effect, and any active fault codes.
History returns the windowed time-series at the
operator's published granularity (typically 1-minute
or 5-minute averages).

## §17 Conformance

Implementations claiming PHASE-2 conformance publish
the OpenAPI document, expose the IEEE 2030.5 + SunSpec
+ OpenADR + OCPP surfaces relevant to the operator's
asset mix, expose the supervisory examination surface,
and propagate trace-context across the dispatch-and-
ride-through chain.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-distributed-energy
- **Last Updated:** 2026-04-28
