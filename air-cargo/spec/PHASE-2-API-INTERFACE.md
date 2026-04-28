# WIA-air-cargo PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-air-cargo
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the API contract that an
air-cargo operator exposes for the records defined in
PHASE-1. Three complementary surfaces are described:
the IATA Cargo-IMP / Cargo-XML messaging surface; the
UN/EDIFACT and WCO Cargo-XML surface for customs
filing; and the HTTPS / JSON RESTful surface for
operational visibility, the customer-facing booking
channel, the regulated-agent / known-consignor
discipline, and the regulatory examination scope.

References (CITATION-POLICY ALLOW only):

- IATA Cargo-IMP (FFM, FWB, FHL, FNA, FBL, FRC,
  FOH, FFA, FZB messages)
- IATA Cargo-XML (Booking Request, Pouch, Status
  Update messages)
- IATA e-AWB Resolution 672 + Multilateral
  Agreement
- IATA Cargo iQ Resolution 674 milestone events
- UN/EDIFACT IFTSTA / IFTMIN / IFTMCS
- WCO Cargo-XML (Cargo Report, Cargo Response)
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

- The IATA Cargo-IMP messaging endpoint (Type B over
  the airline's communication network).
- The IATA Cargo-XML web-service endpoint over
  HTTPS.
- The UN/EDIFACT message endpoints over the EDI
  network (or via Cargo-XML envelope).
- The WCO Cargo-XML endpoint for customs filing.
- The HTTPS / JSON RESTful surface served from a
  domain published by the operator under `/v1/`.

The OpenAPI 3.1 document at `/v1/openapi.json` is
canonical for the JSON surface; the IATA Cargo-XML
schema and the WCO Cargo-XML schema are canonical
for their respective surfaces.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-air-cargo",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "programmes":              "/v1/programmes",
    "parties":                 "/v1/parties",
    "bookings":                "/v1/bookings",
    "eAwbs":                   "/v1/e-awbs",
    "dgDeclarations":          "/v1/dg-declarations",
    "specialisedHandling":     "/v1/specialised-handling",
    "securityScreening":       "/v1/security-screening",
    "customsDeclarations":     "/v1/customs-declarations",
    "cargoIqEvents":           "/v1/cargo-iq-events",
    "examination":             "/v1/examination",
    "openapi":                 "/v1/openapi.json"
  }
}
```

## §3 IATA Cargo-XML Surface

```
POST   /cargo-xml/booking-request    (BookingRequest
                                      message)
POST   /cargo-xml/booking-confirmation (Booking
                                       Confirmation
                                       message)
POST   /cargo-xml/pouch-message      (Pouch carrying
                                      multiple
                                      shipment
                                      records)
POST   /cargo-xml/status-update      (Cargo iQ
                                      milestone
                                      update)
POST   /cargo-xml/awb-message        (e-AWB FWB
                                      payload)
```

## §4 IATA Cargo-IMP Surface (Legacy)

For airlines and forwarders using the legacy
SITAtex / ARINC Type B network:

```
POST   /cargo-imp/ffm  (Flight Manifest)
POST   /cargo-imp/fwb  (Master AWB Data)
POST   /cargo-imp/fhl  (House AWB Data)
POST   /cargo-imp/foh  (Freight On Hand)
POST   /cargo-imp/fra  (Allotment Request)
POST   /cargo-imp/ffa  (Freight Forward
                        Acknowledgement)
POST   /cargo-imp/fzb  (Specialised Cargo Service
                        Confirmation — DGR / PER /
                        AVI / etc.)
```

## §5 UN/EDIFACT Customs Surface

```
POST   /edifact/iftsta   (Status Report)
POST   /edifact/iftmin   (Instruction Message)
POST   /edifact/iftmcs   (Booking Confirmation)
POST   /edifact/cuscar   (Customs Cargo Report)
POST   /edifact/cusdec   (Customs Declaration)
POST   /edifact/cusrep   (Customs Response)
POST   /edifact/cusexp   (Customs Express Consignment)
```

## §6 WCO Cargo-XML Customs Surface

```
POST   /wco-cargo-xml/cargo-report
POST   /wco-cargo-xml/cargo-response
POST   /wco-cargo-xml/declaration
POST   /wco-cargo-xml/declaration-response
```

## §7 Booking and e-AWB Endpoints

```
GET    /v1/bookings?awb={awbNumber}
POST   /v1/bookings                  (create a
                                      booking)
GET    /v1/bookings/{bookingId}
GET    /v1/e-awbs?awb={awbNumber}
POST   /v1/e-awbs                    (issue an e-AWB)
GET    /v1/e-awbs/{awbId}
GET    /v1/e-awbs/{awbId}/digest     (canonical
                                      hash for
                                      e-AWB
                                      tamper-
                                      detection)
```

## §8 Dangerous-Goods, Specialised, and Security
       Endpoints

```
POST   /v1/dg-declarations           (record a DGR
                                      declaration
                                      with the IATA
                                      DGR §9
                                      acceptance
                                      checklist)
GET    /v1/dg-declarations/{declarationId}
POST   /v1/specialised-handling      (CEIV / LAR /
                                      PCR / TCR
                                      record)
POST   /v1/security-screening        (regulated-
                                      agent or
                                      airline
                                      screening
                                      event)
GET    /v1/security-screening?awb={awbNumber}
```

## §9 Customs Declaration Endpoints

```
POST   /v1/customs-declarations
GET    /v1/customs-declarations/{declarationId}
GET    /v1/customs-declarations/{declarationId}/outcome
       (release / inspection / hold)
```

## §10 Cargo-iQ Endpoints

```
GET    /v1/cargo-iq-events?awb={awbNumber}
POST   /v1/cargo-iq-events
GET    /v1/cargo-iq-events/srk/{srk}/timeline
       (full milestone timeline for the shipment-
        record-key)
```

## §11 Examination Endpoints

```
GET    /v1/examination/programmes
GET    /v1/examination/dg-declarations
GET    /v1/examination/security-screening
GET    /v1/examination/customs-declarations
GET    /v1/examination/cargo-iq-events
GET    /v1/examination/audit-events
```

The examination scope is read-only and bound to the
authority's identity (US TSA + CBP + DOT PHMSA for
US-jurisdiction; EU EASA + Member-State NCA + EU
Member-State customs for EU-jurisdiction; KR 국토
교통부 + 관세청 + 항공보안협회 for KR-jurisdiction;
ICAO USOAP audit teams).

## §12 Authentication and Authorisation

Bearer tokens conform to OAuth 2.1 with audiences
declared per surface. Cargo-IMP / Cargo-XML / EDI
endpoints use mutual TLS. The DG-declaration and
security-screening endpoints require elevated scope
and the four-eyes review on alarm resolution.

## §13 HTTP Status Codes

Standard codes apply (200 / 201 / 202 / 400 / 401 /
403 / 404 / 409 / 422 / 429 / 503) with Problem
Details bodies.

## §14 Webhook and Event Surface

Lifecycle events:

- `booking.confirmed`, `booking.cancelled`
- `e-awb.issued`, `e-awb.amended`
- `dg-declaration.accepted`,
  `dg-declaration.rejected`
- `security-screening.completed`,
  `security-screening.alarm-not-resolved`
- `customs.released`, `customs.held-for-inspection`
- `cargo-iq.milestone-emitted`

Webhook signatures use HTTP Message Signatures (RFC
9421).

## §15 IATA ONE Record API Surface

For operators adopting IATA ONE Record:

```
GET    /one-record/api/logistics-objects/{id}
POST   /one-record/api/logistics-objects
PATCH  /one-record/api/logistics-objects/{id}
GET    /one-record/api/logistics-events?awb={awbNumber}
```

ONE Record uses JSON-LD over HTTPS with W3C Linked
Data conventions; logistics-objects (Shipment,
House Waybill, Master Waybill, Booking, Piece,
Item) form a graph with linked references. Subscriptions
to logistics-event streams replace the EDI push
model.

## §16 Bulk-Export and Audit Surface

```
POST   /v1/bulk-export
GET    /v1/bulk-export/{exportId}/manifest
GET    /v1/audit-events?from={iso}&to={iso}
```

Bulk exports support TSA / CBP / EU customs / KR
관세청 audit data calls and the operator's CASS /
SIS settlement reconciliation.

## §17 Examination Bulk-Download Surface

```
GET    /v1/examination/dg-declarations.csv
GET    /v1/examination/security-screening.csv
GET    /v1/examination/customs-declarations.csv
GET    /v1/examination/cargo-iq-events.csv
```

The CSV downloads support the supervisory authority's
trend analysis and the operator's annual security-
performance report. Per-export digest declarations
preserve integrity for the receiving authority.

## §18 Real-Time Track-and-Trace Surface

For shippers and consignees:

```
GET    /portal/v1/track/{awbNumber}
GET    /portal/v1/track/{awbNumber}/timeline
       (server-sent events of milestone updates)
```

The track-and-trace surface respects the operator's
data-protection regime — the consignee identity is
verified through the consignee's authentication
flow before the full timeline is exposed.

## §19 Cargo iQ Performance Reporting Surface

For Cargo iQ members:

```
GET    /v1/cargo-iq/performance?carrier={code}&period={iso}
GET    /v1/cargo-iq/performance/lane?origin={loc}&destination={loc}
GET    /v1/cargo-iq/performance/airline-on-time
GET    /v1/cargo-iq/performance/forwarder-on-time
```

The performance-reporting surface aggregates the
shipment-record-key milestone events into the IATA
Cargo iQ baseline metrics — On Time Performance
(OTP), Door-to-Door cycle time, segment-level
service-level deviations.

## §20 Live-Animal and Perishable Special-Endpoint
        Surface

```
POST   /v1/specialised-handling/live-animal/health-cert
POST   /v1/specialised-handling/perishable/temperature-log
GET    /v1/specialised-handling/{recordId}/temperature-chart
GET    /v1/specialised-handling/{recordId}/cites-permit
       (CITES permit verification for live-animal /
        plant-product shipments)
```

The CITES-permit verification endpoint cross-checks
the shipper's-supplied permit against the issuing
authority's published CITES register.

## §21 Conformance

Implementations claiming PHASE-2 conformance publish
the OpenAPI document, expose the IATA Cargo-XML +
Cargo-IMP messaging surfaces, expose the UN/EDIFACT
+ WCO Cargo-XML customs surfaces, expose the
regulatory examination surface, and propagate trace-
context across the booking-to-delivery chain.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-air-cargo
- **Last Updated:** 2026-04-28
