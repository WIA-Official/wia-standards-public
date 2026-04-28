# WIA-energy-cloud PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-energy-cloud
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the API contract that an
energy-cloud operator exposes for the records defined
in PHASE-1. Three complementary surfaces are
described: the IEC 61968 / 61970 CIM exchange surface
(message envelopes, RDF profiles); the IEC 61850
control-and-monitoring surface (where the operator
interfaces directly with substation automation); and
the HTTPS / JSON RESTful surface for the customer-
facing engagement, third-party-app catalogue, and
operator examination.

References (CITATION-POLICY ALLOW only):

- IEC 61968-100:2022 (implementation profiles for
  IEC 61968 messages — JMS, AMQP, web services)
- IEC 61968-13:2019 (CIM RDF model exchange)
- IEC 61970-301 (CIM base) + IEC 61970-452 + 453 +
  456 (profiles)
- IEC 61850-7-2 + 7-420
- IEC 62325 series (energy-market communications —
  ENTSO-E + EMIX schemas)
- IETF RFC 9110 (HTTP Semantics), RFC 9111 (HTTP
  Caching), RFC 9457 (Problem Details), RFC 6901 /
  6902 (JSON Pointer / Patch), RFC 8288 (Web Linking),
  RFC 8259 (JSON), RFC 9421 (HTTP Message Signatures)
- ISO 8601, ISO/IEC 27001:2022, ISO/IEC 27019:2024
- W3C Trace Context

---

## §1 Scope and Versioning

The operator exposes:

- IEC 61968-100 message-envelope endpoints (JMS /
  AMQP / web-services binding) for distribution-
  management interactions.
- IEC 61970 web-service endpoints for transmission
  EMS interactions (where the operator participates
  in transmission-system EMS).
- The HTTPS / JSON RESTful surface served from a
  domain published by the operator under `/v1/`.
- The customer-portal endpoints under `/portal/v1/`.

The OpenAPI 3.1 document at `/v1/openapi.json` is
canonical for the JSON surface; the IEC 61968-100
WSDL / AMQP descriptor is canonical for the CIM
surface; the IEC 61850 SCD is canonical for the
substation-automation surface.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-energy-cloud",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "programmes":              "/v1/programmes",
    "cimNetworkModels":        "/v1/cim-network-models",
    "tenancies":               "/v1/tenancies",
    "derFleetAggregations":    "/v1/der-fleet-aggregations",
    "forecasts":               "/v1/forecasts",
    "marketParticipations":    "/v1/market-participations",
    "engagements":             "/v1/engagements",
    "appCatalogue":            "/v1/app-catalogue",
    "operationsEvents":        "/v1/operations-events",
    "examination":             "/v1/examination",
    "openapi":                 "/v1/openapi.json"
  }
}
```

## §3 IEC 61968 Message-Envelope Surface

IEC 61968-100 envelopes carry the per-message-type
payloads:

```
NetworkOperations (61968-3) — messages: GetSwitchesRequest,
SwitchActionRequest, AlarmEvent, CrewDispatch
RecordsAssetManagement (61968-4) — AssetCondition,
WorkRequest, MaintenanceSchedule
CustomerSupport (61968-8) — CustomerInquiry,
ServiceRequestEnvelope, OutageNotification
MeterReading (61968-9) — MeterReadingRequest /
Response, EndDeviceEvent, MeterAddRemove
DistributionConfiguration (61968-13) — model exchange
of distribution network topology and equipment
parameters
```

The envelope verb / noun pattern (Get / Show / Cancel /
Change / Create / Close / Delete · Asset / Customer /
Meter / Switch · Request / Reply / Event) is
canonical.

## §4 IEC 61970 EMS-API Surface

For transmission-EMS-class deployments:

```
GET    /ems/v1/topology                  (CIM topology
                                          export per
                                          61970-452)
GET    /ems/v1/state-estimator-results   (steady-state
                                          solution per
                                          61970-456)
POST   /ems/v1/scada-write               (SCADA-class
                                          control writes)
```

## §5 Tenancy and Customer-Site Endpoints

```
GET    /v1/tenancies?premise={geo}
GET    /v1/tenancies/{tenancyId}
POST   /v1/tenancies
PATCH  /v1/tenancies/{tenancyId}
GET    /v1/tenancies/{tenancyId}/meters
GET    /v1/tenancies/{tenancyId}/der-assets
GET    /v1/tenancies/{tenancyId}/program-enrolments
```

## §6 DER Fleet and Forecasting Endpoints

```
GET    /v1/der-fleet-aggregations?scope={scope}
GET    /v1/der-fleet-aggregations/{aggregationId}
POST   /v1/der-fleet-aggregations
GET    /v1/forecasts?kind={kind}&horizon={hours}
POST   /v1/forecasts
GET    /v1/forecasts/{forecastId}/values
GET    /v1/forecasts/{forecastId}/error-metrics
```

## §7 Wholesale-Market Endpoints

```
GET    /v1/market-participations
GET    /v1/market-participations/{participationId}
POST   /v1/market-participations/{participationId}/bids
GET    /v1/market-participations/{participationId}/awards
GET    /v1/market-participations/{participationId}/settlements
```

## §8 Customer-Engagement and Third-Party App
       Endpoints

```
GET    /portal/v1/me                        (tenant's
                                             tenancy)
GET    /portal/v1/me/usage?from={iso}&to={iso}
GET    /portal/v1/me/forecast
POST   /portal/v1/me/program-enrolment
GET    /v1/app-catalogue
POST   /v1/app-catalogue                    (register a
                                             third-party
                                             app)
PATCH  /v1/app-catalogue/{appId}/approve
PATCH  /v1/app-catalogue/{appId}/suspend
```

## §9 Operations-Event Endpoints

```
GET    /v1/operations-events?from={iso}&to={iso}
GET    /v1/operations-events/{eventId}
POST   /v1/operations-events                (record an
                                             operations
                                             event)
```

## §10 Examination Endpoints

```
GET    /v1/examination/programmes
GET    /v1/examination/cim-network-models
GET    /v1/examination/operations-events
GET    /v1/examination/cybersecurity-posture
GET    /v1/examination/audit-events
```

The examination scope is read-only and bound to the
authority's identity (NERC for US bulk-electric-
system operators; FERC for FERC-jurisdictional
markets; KR FSC + KEPCO + KPX for KR-jurisdiction;
the local Public Utility Commission).

## §11 Authentication and Authorisation

Bearer tokens conform to OAuth 2.1 with audiences
declared per surface. The IEC 61968-100 message-
envelope surface uses TLS with mutual authentication
per IEC 62351-3. The customer-portal surface uses
consumer-OAuth flows.

## §12 HTTP Status Codes and Caching

Standard codes apply (200 / 201 / 202 / 400 / 401 /
403 / 404 / 409 / 422 / 429 / 503). `ETag` carries
the resource's version-id; PII / NPI-bearing
responses use `Cache-Control: private, no-store`.

## §13 Webhook and Event Surface

Lifecycle events through a webhook channel:

- `cim-model.published`, `cim-model.deprecated`
- `der-fleet.dispatch-issued`,
  `der-fleet.dispatch-acknowledged`
- `forecast.updated`
- `market-participation.bid-submitted`,
  `market-participation.award-received`
- `operations-event.declared`,
  `operations-event.restored`
- `app-catalogue.app-approved`,
  `app-catalogue.app-suspended`

Signatures use HTTP Message Signatures (RFC 9421).

## §14 Bulk-Export and Settlement-Reconciliation Surface

```
POST   /v1/bulk-export
GET    /v1/bulk-export/{exportId}/status
GET    /v1/settlement-reconciliation?period={iso}
```

The settlement-reconciliation endpoint surfaces the
fleet-level dispatch instructions, the actual
metered output, and the settlement-engine reconciled
deltas for the requested billing period.

## §15 Per-Tenancy Energy-Data Subscription Surface

The Green Button Connect My Data convention (NAESB
REQ.21 in US; equivalent in EU and KR) underlies the
operator's per-tenancy energy-data subscription:

```
GET    /portal/v1/me/usage.xml          (Green Button
                                         Connect My
                                         Data ESPI 1.1)
GET    /portal/v1/me/usage.json         (operator
                                         JSON variant)
POST   /portal/v1/me/data-sharing/grant (grant a
                                         third-party
                                         app the
                                         tenant's
                                         energy-data
                                         scope)
PATCH  /portal/v1/me/data-sharing/{grantId}/revoke
```

Energy-data subscriptions follow the customer's
explicit consent under the operating jurisdiction's
data-protection regime; the operator's audit-events
record every grant, every read, and every revocation.

## §16 Synchrophasor and PMU Integration Surface

For operators integrating phasor-measurement-unit
data:

```
GET    /v1/synchrophasor/sources
GET    /v1/synchrophasor/sources/{sourceId}/streams
GET    /v1/synchrophasor/streams/{streamId}/snapshot
```

The synchrophasor data substrate uses IEEE C37.118
+ IEEE C37.118.2 message envelopes; the cloud-side
ingest aggregates per-PMU streams for wide-area
situational awareness and oscillation-detection
analytics.

## §17 GraphQL Federation Surface

For analytics consumers preferring GraphQL the
operator exposes a federated GraphQL endpoint:

```
POST   /v1/graphql        (federated GraphQL with
                           per-resolver scope
                           gating)
GET    /v1/graphql/schema (introspection of the
                           published schema)
```

The GraphQL schema federates the tenancy, der-
fleet-aggregation, forecast, market-participation,
operations-event, and engagement subgraphs. Per-
resolver authorisation enforces the per-tenancy and
per-app scope discipline.

## §18 Streaming Telemetry Surface

Real-time streaming telemetry is exposed via:

- WebSocket over TLS for browser-class consumers.
- Server-Sent Events (SSE) for one-way streaming.
- Apache Kafka or AMQP for backend consumers
  receiving high-cardinality telemetry.

The streaming surface carries the canonical IEC
61968 / 61970 envelopes; quality-of-service
guarantees follow the operator's published
publication-rate budget per topic.

## §19 Examination Bulk-Export Audit Surface

The examination scope's bulk-export endpoints
support the supervisory authority's annual data
calls:

```
POST   /v1/examination/bulk-export
GET    /v1/examination/bulk-export/{exportId}/manifest
GET    /v1/examination/operations-events.csv
GET    /v1/examination/cybersecurity-posture.csv
```

The manifest declares the cryptographic digest of
each NDJSON / CSV file produced; the receiving
authority can verify integrity end-to-end.

## §20 Conformance

Implementations claiming PHASE-2 conformance publish
the OpenAPI document, expose the IEC 61968-100 +
IEC 61970 surfaces relevant to the operator's role,
expose the customer-portal and third-party-app
catalogue surfaces, expose the supervisory examination
surface, and propagate trace-context across the
forecast-to-dispatch-to-settlement chain.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-energy-cloud
- **Last Updated:** 2026-04-28
