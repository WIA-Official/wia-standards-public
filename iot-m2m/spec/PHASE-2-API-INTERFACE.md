# WIA-iot-m2m PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-iot-m2m
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTPS API contract that an
IoT-M2M deployment exposes for the records defined in
PHASE-1. Consumers include application servers,
provisioning consoles, lifecycle-management services,
analytics platforms, regulators (where the deployment is
subject to sector-specific regulation), and the operator's
own audit and observability platforms.

References (CITATION-POLICY ALLOW only):

- IETF RFC 9110 (HTTP Semantics)
- IETF RFC 9111 (HTTP Caching)
- IETF RFC 9457 (Problem Details)
- IETF RFC 6901 / 6902 (JSON Pointer / Patch)
- IETF RFC 8288 (Web Linking)
- IETF RFC 8259 (JSON)
- IETF RFC 9421 (HTTP Message Signatures)
- IETF RFC 7252 (CoAP; for the device-side facade only)
- ISO 8601 (date and time)
- ISO/IEC 27001:2022 (information security management)
- W3C Trace Context
- W3C WoT TD 1.1
- oneM2M TS-0001 / TS-0008 / TS-0009
- OMA LwM2M v1.2
- OASIS MQTT v5.0

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published by the
operator. Versioning uses `/v1/` path segments. The OpenAPI
3.1 document at `/v1/openapi.json` is canonical.

This API is the operator-facing facade. The device-facing
facade speaks CoAP (RFC 7252), MQTT (OASIS MQTT v5.0), or
LwM2M (OMA v1.2) directly to constrained devices and is
documented by the underlying service-layer specifications;
this PHASE does not redefine those device-facing protocols
but does record the bindings (PHASE-1 §4
`protocolBindings`).

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-iot-m2m",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "deployments":          "/v1/deployments",
    "devices":              "/v1/devices",
    "thingDescriptions":    "/v1/thing-descriptions",
    "resourceTreeNodes":    "/v1/resource-tree-nodes",
    "telemetry":            "/v1/telemetry",
    "actuations":           "/v1/actuations",
    "fota":                 "/v1/fota",
    "evidence":             "/v1/evidence",
    "openapi":              "/v1/openapi.json"
  }
}
```

## §3 Deployment Lifecycle

```
POST   /v1/deployments              — register a deployment
GET    /v1/deployments/{did}        — retrieve deployment
PATCH  /v1/deployments/{did}/status — advance status
```

## §4 Device Lifecycle

```
POST   /v1/deployments/{did}/devices         — register a
                                                 device
PATCH  /v1/devices/{dvid}/status             — advance status
PATCH  /v1/devices/{dvid}/firmware-pointer   — update firmware
                                                 reference (used
                                                 by FOTA after a
                                                 successful
                                                 application)
GET    /v1/devices/{dvid}                    — retrieve device
```

Device submissions whose `radioStack` includes a regulated
band (cellular IoT bands, LoRaWAN ISM bands per region) are
verified against the operator's per-jurisdiction radio-band
authorisation; mismatches return `409 Conflict` with type
`urn:wia:iot-m2m:radio-band-not-authorised`.

## §5 Thing Description Lifecycle

```
POST   /v1/devices/{dvid}/thing-description  — register or
                                                 update the
                                                 device's TD
GET    /v1/thing-descriptions/{tid}          — retrieve TD
                                                 metadata
GET    /v1/thing-descriptions/{tid}/content  — fetch TD
                                                 JSON-LD body
```

TD updates that downgrade an affordance class (a property
that becomes non-readable, an action that disappears from
the TD) require operator authorisation and emit a
notification to consuming applications through the streaming
subscription.

## §6 Resource Tree

```
POST   /v1/deployments/{did}/resource-tree-nodes
                                              — register a
                                                 resource-tree
                                                 node
GET    /v1/resource-tree-nodes/{nid}          — retrieve node
GET    /v1/resource-tree-nodes/{nid}/subtree
                                              — server-rendered
                                                 subtree
                                                 traversal
                                                 (configurable
                                                 depth limit)
```

The subtree endpoint honours depth limits to bound response
size; deep traversals receive a `Link: rel="next"` header for
pagination.

## §7 Telemetry Ingest

```
POST   /v1/devices/{dvid}/telemetry           — append a
                                                 telemetry
                                                 record
POST   /v1/bulk/telemetry                     — batched ingest
GET    /v1/telemetry/{tid}                    — retrieve
                                                 telemetry
GET    /v1/devices/{dvid}/telemetry?
       affordance={a}&from={t}&to={t}          — query window
```

High-volume telemetry from gateway aggregation devices
(thousands of constrained devices behind one gateway) flows
through `/v1/bulk/telemetry`; ingest backpressure follows
the operator's per-tenant rate limit.

## §8 Actuation

```
POST   /v1/devices/{dvid}/actuations          — request an
                                                 actuation
PATCH  /v1/actuations/{aid}/acknowledged      — record device
                                                 acknowledgement
PATCH  /v1/actuations/{aid}/completed         — record completion
GET    /v1/actuations/{aid}                   — retrieve
                                                 actuation
```

Actuation requests targeted at devices in `suspended` or
`fault` status return `409 Conflict` with type
`urn:wia:iot-m2m:device-not-actuatable`. Actuations whose
inputs do not validate against the device's TD action
schema return `422` with type
`urn:wia:iot-m2m:td-schema-violation`.

## §9 FOTA Lifecycle

```
POST   /v1/devices/{dvid}/fota                — initiate a
                                                 FOTA event
PATCH  /v1/fota/{fid}/state                   — advance FOTA
                                                 state
PATCH  /v1/fota/{fid}/rollback                — record rollback
                                                 evidence
GET    /v1/fota/{fid}                         — retrieve FOTA
                                                 record
```

FOTA submissions whose `toFirmwareRef` is not signed under
the operator's firmware-signing policy return `409` with
type `urn:wia:iot-m2m:firmware-signature-required`.

## §10 Errors

All error responses are `application/problem+json` per RFC
9457. Defined types include those above plus:

- `urn:wia:iot-m2m:retention-window-elapsed`
- `urn:wia:iot-m2m:cohort-too-small`
- `urn:wia:iot-m2m:evidence-mismatch`

## §11 Authentication

Mutually-authenticated TLS for application-server consumers,
provisioning consoles, lifecycle-management services, and
regulators. Public read-only endpoints (deployment summary,
non-sensitive telemetry aggregates) are reachable without a
client certificate.

Device-facing facades use device-class-appropriate
authentication: DTLS (RFC 9147) for CoAP, OAuth 2.0
device-flow or pre-provisioned credentials for HTTP, MQTT
v5.0 enhanced authentication, LwM2M's pre-shared-key /
public-key bootstrapping. This API records the operator's
chosen credential framework but does not redefine it.

## §12 Caching, Concurrency, Audit

Stable resources (frozen device records for decommissioned
devices, completed FOTA records, signed evidence packages)
are cacheable with `Cache-Control: max-age=31536000,
immutable`. Telemetry and actuation records are cacheable
for 30 seconds. ETags are mandatory on every PATCH endpoint.
Audit logs carry `deploymentId`, `deviceRef`, `traceId`,
the issuing client certificate's subject, and the operator's
clock skew vs the operating jurisdiction's NTP service.

## §13 Streaming Subscription

Consumers subscribe via Server-Sent Events at:

- `/v1/deployments/{did}/events` — deployment-wide events
  (TD updates, FOTA campaigns, device-population health
  alerts).
- `/v1/devices/{dvid}/events` — device-scoped events (status
  transitions, actuation completions, FOTA progress).

Subscribers reconnect via the `Last-Event-ID` header (W3C
EventSource semantics).

## §14 Worked Example: Provisioning to Telemetry to Actuation

1. The operator provisions a device through the operator's
   provisioning console: a manufactured device receives its
   identity, root-key derivation reference, and TD.
2. The device's status advances to `provisioned` and then
   to `deployed-active` upon first contact with the service
   layer.
3. The device emits telemetry through the device-facing
   facade; the service layer ingests through `/v1/devices/
   {dvid}/telemetry` (or via the gateway's bulk endpoint).
4. An application server emits an actuation through `/v1/
   devices/{dvid}/actuations`; the device acknowledges and
   completes; the actuation outcome is recorded.
5. A scheduled FOTA campaign updates the device firmware;
   the operator monitors the FOTA roll-out through the
   streaming subscription.

## §15 Subscription and Audit Endpoints

```
POST   /v1/deployments/{did}/subscriptions    — register a
                                                 subscription
PATCH  /v1/subscriptions/{sid}/renew          — renew expired
                                                 subscription
GET    /v1/subscriptions/{sid}                — retrieve
                                                 subscription
GET    /v1/deployments/{did}/audit-logs?
       from={t}&to={t}&actor={a}                — query audit
                                                 log
```

Audit-log queries respect the operator's per-actor-class
authorisation: per-device audit access is restricted to the
device's owner-organisation and the operator's incident-
response team.

## §16 Provenance and Aggregation

```
GET    /v1/provenance/{recordId}    — provenance entry for
                                       any PHASE-1 record
GET    /v1/aggregate/device-population-by-status?period=...
GET    /v1/aggregate/fota-success-rate?period=...
GET    /v1/aggregate/telemetry-volume-by-vertical?period=...
```

Aggregate consumers fetch population-level statistics
without per-device attribution. Out-of-policy queries return
`403 Forbidden` with type
`urn:wia:iot-m2m:cohort-too-small`.

## §17 Gateway Aggregation Endpoint

```
POST   /v1/devices/{gid}/aggregations         — register a
                                                 gateway-child
                                                 binding
PATCH  /v1/aggregations/{aid}/revoke          — revoke a
                                                 binding
GET    /v1/aggregations/{aid}                 — retrieve
                                                 aggregation
                                                 record
GET    /v1/devices/{gid}/aggregations         — list children
                                                 behind a
                                                 gateway
```

Aggregation submissions whose gateway and child are not in
the same deployment return `422` with type
`urn:wia:iot-m2m:cross-deployment-binding`.

## §18 Conformance

A conformant server passes the test vectors published under
`tests/phase-vectors/phase-2-api-interface/`, emits an
OpenAPI 3.1 document, signs evidence packages per RFC 9421,
and rejects FOTA submissions whose firmware is not signed
under the operator's policy.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-iot-m2m
- **Last Updated:** 2026-04-28
