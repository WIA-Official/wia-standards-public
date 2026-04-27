# WIA-infrastructure-integration PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-infrastructure-integration
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTP API contract that an accredited
infrastructure-integration operator exposes for the records defined
in PHASE-1. Consumers include integration architects authoring
new flows, integration brokers operating in production, vendor
support teams onboarding contributing systems, regulators reviewing
critical-infrastructure integration deployments, and citation tools
that resolve published flow descriptions to their underlying
records.

References (CITATION-POLICY ALLOW only):

- IETF RFC 9110 (HTTP Semantics) / 9111 (HTTP Caching) /
  9457 (Problem Details) / 6901 / 6902 / 8288 (Web Linking) /
  8259 (JSON) / 9421 (HTTP Message Signatures)
- ISO 8601 (date and time)
- ISO 15926 (life-cycle integration)
- IEC 62541 (OPC UA)
- IEC 61850 (substation automation)
- IEC 61968 / 61970 (CIM)
- W3C Trace Context

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published by the operating
federation. Versioning uses `/v1/` path segments and Semantic
Versioning 2.0.0. The OpenAPI 3.1 document at `/v1/openapi.json`
is canonical.

The API is a *control-plane and metadata facade*; production
message traffic flows over the protocol bindings recorded in
PHASE-1 §4 (OPC UA, IEC 61850 GOOSE/MMS, MQTT, AMQP, Kafka, and
others), and the API references those flows but does not
itself carry production payloads.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-infrastructure-integration",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "federations":      "/v1/federations",
    "systems":          "/v1/systems",
    "protocolBindings": "/v1/protocol-bindings",
    "semanticMappings": "/v1/semantic-mappings",
    "namespaceMappings":"/v1/namespace-mappings",
    "messageFlows":     "/v1/message-flows",
    "replayWindows":    "/v1/replay-windows",
    "auditTrails":      "/v1/audit-trails",
    "changeControls":   "/v1/change-controls",
    "evidence":         "/v1/evidence",
    "openapi":          "/v1/openapi.json"
  }
}
```

## §3 Federation Endpoints

```
POST   /v1/federations                  — establish a federation
GET    /v1/federations/{fid}            — retrieve a federation
PATCH  /v1/federations/{fid}/charter    — replace the governance
                                          charter content-address
GET    /v1/federations/{fid}/topology   — return a topology map
                                          of contributing systems
                                          and active flows
```

A federation registration whose governance charter does not
resolve to a content-address whose digest matches the submitted
hash returns `422` with type
`urn:wia:infrastructure-integration:charter-mismatch`.

## §4 System Endpoints

```
POST   /v1/federations/{fid}/systems   — register a contributing
                                          system
GET    /v1/systems/{sid}               — retrieve system descriptor
PATCH  /v1/systems/{sid}/version       — upgrade product version;
                                          emits an audit record
DELETE /v1/systems/{sid}               — initiate decommissioning;
                                          emits a decommissioning
                                          record (PHASE-1 §11)
GET    /v1/systems/{sid}/protocol-bindings — list bindings
```

System version upgrades trigger automatic re-validation of
semantic mappings whose source or target is the upgraded system;
the API returns the list of mappings that require re-validation
in the response so the integration architect can plan the
upgrade.

## §5 Protocol-Binding Endpoints

```
POST   /v1/systems/{sid}/protocol-bindings — register a binding
GET    /v1/protocol-bindings/{bid}         — retrieve binding
PATCH  /v1/protocol-bindings/{bid}/conformance-class
                                           — record a vendor-
                                              published conformance
                                              class change
```

Binding registrations whose `messageSchemaContentAddress` is not
a recognised schema family (OPC UA NodeSet2, IEC 61850 SCL,
OpenAPI 3.1, AsyncAPI 2/3) return `422` with type
`urn:wia:infrastructure-integration:schema-family-unrecognised`.

## §6 Semantic-Mapping Endpoints

```
POST   /v1/semantic-mappings                — register a mapping
GET    /v1/semantic-mappings/{mid}          — retrieve mapping
PATCH  /v1/semantic-mappings/{mid}/validate — append a validation
                                              record
GET    /v1/semantic-mappings/{mid}/lineage  — trace the source
                                              and target model
                                              references back to
                                              the contributing
                                              systems' schemas
```

Validation records are signed by the integration architect and
counter-signed by a reviewing architect; submissions whose
counter-signature is missing return `422` with type
`urn:wia:infrastructure-integration:counter-signature-required`.

## §7 Namespace-Mapping Endpoints

```
POST   /v1/namespace-mappings              — register a namespace
                                              mapping
GET    /v1/namespace-mappings/{nmid}        — retrieve mapping
GET    /v1/namespace-mappings/{nmid}/resolve?systemId={sid}&local={localId}
                                            — resolve a local
                                              identifier to the
                                              authoritative
                                              identifier
```

The resolver endpoint is read-heavy and is cached aggressively;
responses are immutable for a given `(systemId, localId, asOf)`
triple. Cache-Control directives (PHASE-2 §13) reflect that
immutability.

## §8 Message-Flow Endpoints

```
POST   /v1/message-flows               — register a message flow
GET    /v1/message-flows/{flowId}      — retrieve flow record
PATCH  /v1/message-flows/{flowId}/qos  — update quality-of-service
                                         contract
PATCH  /v1/message-flows/{flowId}/state — suspend / restore /
                                          deprecate the flow
GET    /v1/message-flows/{flowId}/lineage — trace from publisher
                                            through subscribers
                                            with the active
                                            mappings
```

State transitions on a flow emit audit-trail records (PHASE-1 §9)
automatically; clients do not need to write the audit records
directly, but they MAY query them for visibility.

## §9 Replay-Window Endpoints

```
POST   /v1/replay-windows              — request a replay window
                                         on a flow
GET    /v1/replay-windows/{windowId}   — retrieve window record
PATCH  /v1/replay-windows/{windowId}/approvals
                                       — append approvals
PATCH  /v1/replay-windows/{windowId}/state
                                       — initiate / pause /
                                         complete / abort the
                                         replay
```

Replay-window approvals require the federation's change-control
board chair's signature; submissions whose chair's signature is
missing return `422` with type
`urn:wia:infrastructure-integration:replay-approvals-incomplete`.

## §10 Audit-Trail Endpoints

```
GET    /v1/audit-trails?federationId={fid}&from={iso8601}
                                       — query audit trail for a
                                         federation over a window
GET    /v1/audit-trails/{auditId}      — retrieve a specific audit
                                         record
```

Audit-trail responses are streamed using the `text/event-stream`
content type when the requested window exceeds 10,000 events;
small windows return JSON arrays with hyperlinks to the
individual audit records.

## §11 Change-Control Endpoints

```
POST   /v1/change-controls             — propose a change
PATCH  /v1/change-controls/{cid}/approvals
                                       — append approvals from
                                         change-control board
                                         members
PATCH  /v1/change-controls/{cid}/rollout-state
                                       — start / monitor / complete
                                         / rollback the change
GET    /v1/change-controls/{cid}/affected-flows
                                       — list flows affected
```

## §12 Evidence Package

```
POST   /v1/federations/{fid}/evidence  — request evidence package
                                         generation
GET    /v1/evidence/{packageId}        — retrieve a package
GET    /v1/evidence/{packageId}/manifest — manifest only
```

The evidence-package format is governed by PHASE-4 §3 and contains
the federation record, the contributing-system catalogue, the
protocol-binding catalogue, the semantic-mapping catalogue, the
namespace mapping, the message-flow catalogue, the audit trail,
and the change-control register, plus the signed manifest.

## §13 Errors

All error responses are `application/problem+json` per RFC 9457.
Defined types include:

- `urn:wia:infrastructure-integration:charter-mismatch`
- `urn:wia:infrastructure-integration:schema-family-unrecognised`
- `urn:wia:infrastructure-integration:counter-signature-required`
- `urn:wia:infrastructure-integration:replay-approvals-incomplete`
- `urn:wia:infrastructure-integration:flow-locked`
- `urn:wia:infrastructure-integration:evidence-mismatch`

## §14 Authentication

Mutually-authenticated TLS for all write operations; client
certificates are scoped to the federations the client architect
or vendor supports. Public read-only endpoints (federation
governance charter content-address, system catalogue published
under the federation's transparency policy) are reachable
without a client certificate.

## §15 Caching

Stable resources (signed semantic mappings, completed replay
windows, executed change-control records) are cacheable with
`Cache-Control: max-age=31536000, immutable`. Mutable resources
(in-flight change controls, draft mappings) are cacheable for
60 seconds; ETags are mandatory on every PATCH endpoint with
`If-Match` conditional requests.

## §16 Streaming Subscriptions

Consumers subscribe via Server-Sent Events at
`/v1/federations/{fid}/events`. Topics include flow QoS events,
mapping validation events, replay-window state changes, change-
control state changes, and decommissioning notifications.

## §17 Provenance Endpoint

```
GET    /v1/provenance/{recordId}    — retrieve provenance entry
                                       for any PHASE-1 record
```

## §18 Audit and Observability

Every endpoint emits structured logs with `federationId`,
`traceId`, the issuing client certificate's subject, and the
broker's clock skew vs the reference NTP source.

## §19 Bulk Operations

Long onboarding campaigns and large catalogue migrations produce
many records that are exchanged in bulk:

```
POST   /v1/bulk/systems            — bulk system onboarding
POST   /v1/bulk/protocol-bindings  — bulk binding registration
POST   /v1/bulk/semantic-mappings  — bulk mapping registration
POST   /v1/bulk/message-flows      — bulk flow registration
GET    /v1/bulk/{operationId}      — operation status
```

Bulk operations are idempotent on the operation identifier;
retried submissions resolve to the same operation identifier and
return the prior outcome rather than producing duplicate records.

## §20 Worked Example: Onboard a Substation IED Federation Member

1. The vendor publishes the IED's IEC 61850 SCL ICD file and the
   operator's integration architect uploads the file to the
   integration broker.
2. The broker emits a system descriptor draft and protocol-
   binding draft; the architect signs the drafts via PATCH.
3. The architect drafts initial semantic mappings from the IED's
   Logical Nodes to the federation's CIM model; mappings are
   counter-validated by a reviewing architect.
4. A change-control record is opened citing the test-bed
   verification result.
5. After approval and rollout, the broker promotes the system
   to production and emits an audit-trail record.

## §21 Conformance

A conformant server passes the test vectors published under
`tests/phase-vectors/phase-2-api-interface/`, emits an OpenAPI
3.1 document, and signs evidence packages per RFC 9421.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-infrastructure-integration
- **Last Updated:** 2026-04-28
