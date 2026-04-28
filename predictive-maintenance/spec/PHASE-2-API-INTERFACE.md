# WIA-predictive-maintenance PHASE 2 — API Interface Specification

**Standard:** WIA-predictive-maintenance (WIA-IND-026)
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable

This PHASE defines the API surfaces that WIA-
predictive-maintenance participants expose so that
operators, OEMs, MRO providers, asset-management
systems, and audit authorities can manage assets,
ingest telemetry, surface anomalies, post
prognoses, and reconcile work orders through a
single contract.

References (CITATION-POLICY ALLOW only):
- OpenAPI Specification 3.1
- IETF RFC 9110 (HTTP Semantics), RFC 9457 (Problem Details)
- IETF RFC 7515 (JWS), RFC 7519 (JWT)
- IETF RFC 9421 (HTTP Message Signatures)
- ISO 14224:2016, ISO 13374-2..-7, ISO 13381-1, ISO 17359, ISO 55001
- ISO 17442:2020 (LEI)
- IEC 62541 (OPC UA), MTConnect, MQTT v5.0, AMQP 1.0
- Asset Administration Shell (AAS) IEC 63278-1
- Sparkplug B (informative)

---

## §1 Scope

This PHASE specifies the HTTP-based interfaces
between operators, OEMs, MRO providers, and audit
authorities. It does not specify the on-shop-floor
fieldbus (governed by OPC UA / MTConnect / TSN
deployments).

## §2 Operation groups

| Prefix              | Group                                           |
|---------------------|-------------------------------------------------|
| `/v1/assets`        | asset registry                                   |
| `/v1/sensors`       | sensor registry                                  |
| `/v1/streams`       | telemetry-stream descriptor store                |
| `/v1/conditions`    | signal-condition records                         |
| `/v1/anomalies`     | anomaly registry                                 |
| `/v1/prognoses`     | RUL records                                      |
| `/v1/orders`        | maintenance-order lifecycle                      |
| `/v1/twins`         | twin-model registry                              |
| `/v1/registry`      | registry directory                               |

## §3 Authentication

Read endpoints accept anonymous or JWT bearer.
Write endpoints require a JWT bound to the
operator or OEM identity (LEI). Industrial
safety-critical endpoints additionally require an
operator-signed authorisation declaration.

## §4 Asset operations

### 4.1 Register asset

```
POST /v1/assets
```

Body: asset record (PHASE-1 §2) signed by the
operator.

### 4.2 Lookup

```
GET /v1/assets/{assetRef}
```

Returns the canonical asset record.

### 4.3 Asset Administration Shell binding

```
PUT /v1/assets/{assetRef}/aas
```

Body: AAS reference per IEC 63278-1.

## §5 Sensor operations

### 5.1 Register sensor

```
POST /v1/sensors
```

Body: sensor record (PHASE-1 §3).

### 5.2 Calibration upload

```
POST /v1/sensors/{sensorRef}/calibration
```

Body: calibration record per ISO 17025.

## §6 Telemetry operations

### 6.1 Register stream

```
POST /v1/streams
```

Body: stream descriptor (PHASE-1 §4). The actual
telemetry flows over the declared transport (OPC
UA / MTConnect / MQTT / AMQP).

### 6.2 Replay

```
POST /v1/streams/{streamRef}/replay
```

Body: time window. Response: a signed URL to a
replayable archive of the telemetry within the
window.

## §7 Condition operations

### 7.1 Submit conditions

```
POST /v1/conditions/bulk
```

Body: a list of signal-condition records.

### 7.2 Threshold update

```
PUT /v1/sensors/{sensorRef}/thresholds
```

Body: thresholds per ISO 13373-3.

## §8 Anomaly operations

### 8.1 Submit

```
POST /v1/anomalies
```

Body: anomaly record (PHASE-1 §6). Response:
`anomalyRef` plus a triage suggestion (informative).

### 8.2 Acknowledge

```
PUT /v1/anomalies/{anomalyRef}/ack
```

Body: acknowledging operator identity and notes.

## §9 Prognosis operations

### 9.1 Submit RUL

```
POST /v1/prognoses
```

Body: prognosis record (PHASE-1 §7) with model card
URL.

### 9.2 Lookup

```
GET /v1/assets/{assetRef}/prognoses
```

Returns the latest prognoses for the asset.

## §10 Maintenance order operations

### 10.1 Create order

```
POST /v1/orders
```

Body: maintenance order record (PHASE-1 §8).

### 10.2 Complete order

```
POST /v1/orders/{orderRef}/complete
```

Body: completion record (PHASE-1 §9).

### 10.3 Lookup

```
GET /v1/assets/{assetRef}/orders?state=open
```

Returns open orders for the asset.

## §11 Twin-model operations

### 11.1 Register twin

```
POST /v1/twins
```

Body: twin-model record (PHASE-1 §10).

### 11.2 Lookup

```
GET /v1/twins/{twinRef}
```

Returns the canonical twin record.

## §12 Error semantics

Errors are `application/problem+json` (RFC 9457)
namespaced under
`https://wiastandards.com/errors/predictive-maintenance/`.

## §13 Caching and rate limits

Telemetry replay archives are immutable; they
carry `Cache-Control: public, max-age=31536000,
immutable`. Anomaly and order records carry strong
`ETag`. Rate-limit headers follow the
draft-ietf-httpapi-ratelimit-headers convention.

## Annex A — OpenAPI 3.1 fragment

```yaml
openapi: 3.1.0
info: {title: WIA-predictive-maintenance API, version: 1.0.0}
paths:
  /v1/anomalies:
    post:
      summary: Submit an anomaly
      requestBody:
        required: true
        content:
          application/json:
            schema: {$ref: 'AnomalyRecord.schema.json'}
      responses:
        '201': {description: Anomaly submitted}
```

## Annex B — Idempotency

Mutating operations honour `Idempotency-Key` for
24h.

## Annex C — Webhook subscriptions

Subscribers receive events on `anomaly.detected`,
`prognosis.updated`, `order.created`,
`order.completed`. Delivery is signed with HMAC-
SHA-256.

## Annex D — Federation

Federation between operator and OEM registries
follows the discovery contract in PHASE-3.

## Annex E — Bulk export

`POST /v1/registry/export` returns a signed URL to
a `tar.zst` of the deployment's records filtered
by asset and date range.

## Annex F — Sandbox

`/v1/sandbox` mirrors production with synthetic
assets.

## Annex G — Quotas

Per-operator quotas: 100,000 conditions / hour;
1,000 anomalies / hour.

## Annex H — Audit feed

`GET /v1/registry/audit?since=<timestamp>`.

## Annex I — Public introspection

`GET /v1/registry/stats`.

## Annex J — Webhook payload shape

```json
{
  "event": "order.created",
  "orderRef": "f63f4f04-...",
  "assetRef": "asset-072",
  "priority": "urgent"
}
```

## Annex K — OPC UA companion crosswalk

`GET /v1/registry/opcua-crosswalk` returns the
mapping between WIA-predictive-maintenance fields
and OPC UA Companion Specification nodes (Pumps,
Robotics, Machinery).

## Annex L — Reliability dataset export

```
POST /v1/registry/reliability-export
```

Body: a SHACL filter over assets and a date range.
Response: a signed URL referencing a `tar.zst` of
ISO 14224-aligned reliability data (failure rates,
MTBF, MTTR, downtime tables) for the operator's
reliability-engineering team.

## Annex M — Calibration registry

```
GET /v1/registry/calibration?sensorRef={ref}
```

Returns the calibration-record history for the
sensor, including expiry dates and calibration
laboratory accreditation references (ISO 17025
scope).

## Annex N — Failure-mode taxonomy lookup

```
GET /v1/registry/failure-modes?taxonomy=ISO14224&class={class}
```

Returns the ISO 14224 failure-mode codes for the
asset class. Used by anomaly publishers to ensure
they apply the canonical taxonomy.

## Annex O — Twin-model FMI binding

```
POST /v1/twins/{twinRef}/fmi
```

Body: an FMI 3.0 model archive. The registry
catalogues the FMI binding so that downstream
co-simulation can interoperate without
vendor-specific adapters.

## Annex P — IEC 62443 zone declaration

```
PUT /v1/registry/iec62443-zone
```

Body: the deployment's IEC 62443 zone topology
with conduit declarations. The registry surfaces
the topology to auditors.

## Annex Q — Reviewer queue endpoint

```
GET /v1/anomalies/queue?reviewerRef={ref}
```

Returns the reviewer's pending queue. Reviewers
acknowledge anomalies before order creation.

## Annex R — CMMS callback

```
POST /v1/orders/{orderRef}/cmms-callback
```

Body: CMMS-system callback URL. The registry
posts order-state changes to the callback so that
the CMMS remains the system of record.

## Annex S — Asset Administration Shell submodel
##           lookup

```
GET /v1/assets/{assetRef}/aas-submodels
```

Returns the AAS submodels (IEC 63278) bound to the
asset: nameplate, technical data, condition
monitoring, maintenance plan. Used by partners
that consume asset data via AAS.

## Annex T — Spare-parts catalogue resolution

```
POST /v1/parts/resolve
```

Body: a part list with EAN / GTIN / OEM PN. The
registry resolves to the operator's preferred
supplier and returns the procurement reference.

## Annex U — Vendor onboarding

```
POST /v1/registry/vendors
```

Body: OEM vendor record with LEI, signing key set,
and supported asset taxonomies. The registry
verifies the LEI at GLEIF before activation.

## Annex V — Audit-grade replay

```
POST /v1/streams/{streamRef}/replay-audit
```

Body: time window. Response: a signed audit-grade
replay archive that includes the operator's
signature on the entire window plus per-record
signatures for tamper-evident traceability.

## Annex W — RUL benchmark export

```
POST /v1/registry/rul-benchmark
```

Body: a SHACL filter over assets. Response: a
report comparing the deployment's RUL accuracy
metrics against the registry's anonymised
benchmark population.

## Annex X — Per-asset cost surface

```
GET /v1/assets/{assetRef}/cost
```

Returns the asset's lifetime maintenance cost
summary derived from completion records and ERP
join. Used by total-cost-of-ownership analyses.

## Annex Y — Reliability dashboard data

```
GET /v1/registry/reliability-dashboard
```

Returns aggregate counters for the operator's
reliability dashboard: MTBF, MTTR, availability,
OEE, anomaly trend, prognosis-accuracy trend.
Counters are eventually consistent.

## Annex Z — Cross-deployment benchmarking

```
POST /v1/registry/benchmark
```

Body: a SHACL filter over assets and a date range.
Response: anonymised benchmark of the deployment
against the registered population for the same
asset class. Used by operators to compare their
maintenance program effectiveness.

## Annex AA — Bulk anomaly submission

```
POST /v1/anomalies/bulk
Content-Type: application/jsonl
```

Body: a JSON-Lines stream of anomaly records.
Response: a per-line verdict for downstream
processing.

## Annex AB — Vendor migration kit

```
POST /v1/registry/vendor-migration
```

Body: source vendor identity and a SHACL filter
over assets to migrate. Response: a signed URL to
a `tar.zst` containing asset, sensor, calibration,
order, and completion records suitable for import
at a destination vendor.

## Annex AC — Webhook delivery shape

Webhook payloads are canonicalised per RFC 8785
before HMAC-SHA-256 signing. The signature
appears in `X-WIA-Signature: sha256=<hex>`.
Webhook deliveries follow at-least-once semantics
with exponential backoff capped at 9 attempts.

## Annex AD — Audit-grade publisher report

```
POST /v1/registry/publisher-report
```

Body: operator or OEM reference and date range.
Response: a `tar.zst` of the publisher's records,
signing key set, and audit-feed entries suitable
for auditor review.

弘益人間 (Hongik Ingan) — Benefit All Humanity
