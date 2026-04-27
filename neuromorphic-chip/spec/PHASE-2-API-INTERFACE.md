# WIA-neuromorphic-chip PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-neuromorphic-chip
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTP API contract that an accredited
neuromorphic-chip programme exposes for the records defined in
PHASE-1. Consumers include SDK developers that publish compiled
networks, hardware vendors that publish device descriptions, system
integrators that consume mapping outputs, reference laboratories that
emit characterisation records, and citation tools that resolve
published neuromorphic-compute results.

References (CITATION-POLICY ALLOW only):

- IETF RFC 9110 (HTTP Semantics)
- IETF RFC 9111 (HTTP Caching)
- IETF RFC 9457 (Problem Details)
- IETF RFC 6901 (JSON Pointer)
- IETF RFC 6902 (JSON Patch)
- IETF RFC 8288 (Web Linking)
- IETF RFC 8259 (JSON)
- IETF RFC 5789 (PATCH)
- IETF RFC 9421 (HTTP Message Signatures)
- ISO 8601 (date and time)
- ISO/IEC 27001:2022 (information security management)
- W3C Trace Context

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published by the operating
programme. Versioning uses `/v1/` path segments and Semantic
Versioning 2.0.0. The OpenAPI 3.1 document at `/v1/openapi.json` is
canonical.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-neuromorphic-chip",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "networks":          "/v1/networks",
    "hardware":          "/v1/hardware",
    "mappings":          "/v1/mappings",
    "aerStreams":        "/v1/aer-streams",
    "plasticity":        "/v1/plasticity",
    "characterisations": "/v1/characterisations",
    "telemetry":         "/v1/telemetry",
    "evidence":          "/v1/evidence",
    "openapi":           "/v1/openapi.json"
  }
}
```

## §3 Networks

```
POST   /v1/networks                    — register a network description
GET    /v1/networks/{nid}              — retrieve a network
PATCH  /v1/networks/{nid}              — update mutable fields
GET    /v1/networks/{nid}/description  — fetch the description archive
```

A network's status transitions follow PHASE-1 §2; invalid transitions
return `422` with type
`urn:wia:neuromorphic-chip:status-transition`.

## §4 Hardware Descriptions

```
POST   /v1/hardware                    — vendor registers a hardware
                                          description
GET    /v1/hardware/{hid}              — retrieve description
GET    /v1/hardware?vendor={v}         — list by vendor
```

Hardware descriptions are signed by the vendor's release key. The API
verifies the signature on registration and rejects unsigned or
mis-signed submissions with type
`urn:wia:neuromorphic-chip:hardware-signature-invalid`.

## §5 Compilation and Mapping

```
POST   /v1/networks/{nid}/mappings     — request a compilation
GET    /v1/mappings/{mid}              — retrieve mapping
GET    /v1/mappings/{mid}/utilisation  — retrieve utilisation summary
```

Compilation is asynchronous; the POST returns `202 Accepted` with a
`Location` header. The compilation transitions through `pending` →
`mapping` → `complete` or `failed`. A request for a network whose
`neuronModel` or `synapseModel` is not in the target hardware's
`modelSupport` list returns `422` with type
`urn:wia:neuromorphic-chip:model-unsupported`.

## §6 AER Streams and Plasticity

```
POST   /v1/mappings/{mid}/aer-streams         — register an AER stream
GET    /v1/aer-streams/{sid}                  — retrieve stream metadata
GET    /v1/aer-streams/{sid}/archive          — fetch stream archive
POST   /v1/aer-streams/{sid}/plasticity       — append plasticity events
GET    /v1/plasticity/{eid}                   — retrieve event
```

Streams in `summary-windowed` encoding (PHASE-1 §6) are appendable
during a long-running deployment; the API accepts incremental window
appends with monotonically increasing window-start timestamps and
returns `409 Conflict` on regression with type
`urn:wia:neuromorphic-chip:window-regress`.

## §7 Characterisation

```
POST   /v1/hardware/{hid}/characterisations    — register characterisation
GET    /v1/characterisations/{cid}             — retrieve characterisation
GET    /v1/characterisations/{cid}/calibration — fetch calibration table
```

Characterisation records are signed by the laboratory's client
certificate; the API verifies the signature against the laboratory's
ISO/IEC 17025 accreditation in the public register before accepting
the record.

## §8 Telemetry

```
POST   /v1/mappings/{mid}/telemetry      — append telemetry sample
GET    /v1/telemetry/{tid}               — retrieve telemetry record
GET    /v1/mappings/{mid}/telemetry?since={t}  — telemetry window
```

Telemetry uploads are typically batched at the deployed system's next
sync; bulk uploads are accepted at `POST /v1/bulk/telemetry`.

## §9 Evidence Package

```
POST   /v1/mappings/{mid}/evidence    — request evidence-package
                                         generation
GET    /v1/evidence/{packageId}       — retrieve package
GET    /v1/evidence/{packageId}/manifest — manifest only
```

The evidence-package format is governed by PHASE-4 §3 and contains
the network description, the hardware description, the mapping, the
AER stream summary or archive, the plasticity event log, the
characterisation, the telemetry, and the signed manifest.

## §10 Errors

All error responses are `application/problem+json` per RFC 9457.
Defined types include:

- `urn:wia:neuromorphic-chip:status-transition`
- `urn:wia:neuromorphic-chip:model-unsupported`
- `urn:wia:neuromorphic-chip:hardware-signature-invalid`
- `urn:wia:neuromorphic-chip:window-regress`
- `urn:wia:neuromorphic-chip:characterisation-incomplete`
- `urn:wia:neuromorphic-chip:evidence-mismatch`

## §11 Authentication

The API uses mutually-authenticated TLS for vendor-to-programme,
laboratory-to-programme, and integrator-to-programme connections.
Public read-only endpoints are reachable without a client certificate.

## §12 Caching

Stable resources (completed mappings, signed characterisations,
evidence packages) are cacheable with `Cache-Control: max-age=
31536000, immutable`. Mutable resources are cacheable for 60 seconds.
ETags are mandatory on every PATCH endpoint with `If-Match`
conditional requests.

## §13 Streaming Subscriptions

Consumers subscribe to deployment events via Server-Sent Events at
`/v1/mappings/{mid}/events`. Events emitted include AER-stream
window summaries, plasticity bursts, telemetry alerts, and
characterisation revisions.

## §14 Bulk Operations

Long deployments and large network compilations produce many records
that are exchanged in bulk:

```
POST   /v1/bulk/mappings              — submit a bulk compile sweep
POST   /v1/bulk/aer-streams           — submit a bulk stream upload
POST   /v1/bulk/plasticity            — submit a bulk plasticity log
POST   /v1/bulk/telemetry             — submit a bulk telemetry batch
GET    /v1/bulk/{operationId}         — operation status
```

## §15 Worked Example: Compile, Deploy, Cite

1. Vendor registers a hardware description.
2. SDK developer registers a network description.
3. SDK requests a compile against the hardware; mapping completes.
4. Integrator deploys the mapping; AER streams and telemetry are
   appended.
5. Reference laboratory registers a characterisation; calibration
   tables are loaded into the deployment.
6. Citation tool requests the evidence package and pins the
   manifest digest.

## §16 Workload and Energy-Account Endpoints

```
POST   /v1/workload-definitions             — register a workload
GET    /v1/workload-definitions/{wid}       — retrieve workload
POST   /v1/mappings/{mid}/energy-accounts   — register an energy account
GET    /v1/energy-accounts/{aid}            — retrieve energy account
```

Energy-account submissions are signed by the laboratory or integrator
that performed the measurement. Submissions whose
`measurementMethod` is not in the workload's allowed list return
`422` with type
`urn:wia:neuromorphic-chip:measurement-method-mismatch`.

## §17 Pagination Conventions

List endpoints use cursor-based pagination via the `cursor` query
parameter and `Link` headers. Cursors are opaque to clients; servers
MUST persist cursor state for at least 24 hours.

## §18 Privacy-Preserving Aggregation

Aggregate-only consumers (research collaboratives, public-policy
analysts) fetch population-level statistics through aggregation
endpoints that emit counts, means, and dispersions:

```
GET    /v1/aggregate/energy?workload=...&platform=...
GET    /v1/aggregate/accuracy?workload=...&platform=...
```

Out-of-policy queries return `403 Forbidden` with type
`urn:wia:neuromorphic-chip:cohort-too-small`.

## §19 Audit and Observability

Every endpoint emits structured logs with `mappingId`, `traceId`, the
issuing client certificate's subject, and the deployed system's
clock skew vs the reference NTP source.

## §20 Encoder/Decoder Recipe Endpoints

```
POST   /v1/encoder-recipes          — register an encoder recipe
GET    /v1/encoder-recipes/{rid}    — retrieve encoder recipe
POST   /v1/decoder-recipes          — register a decoder recipe
GET    /v1/decoder-recipes/{rid}    — retrieve decoder recipe
GET    /v1/encoder-recipes/{rid}/test-vectors — fetch canonical vectors
```

Recipe registrations are signed by the publishing entity. The API
verifies the recipe against the `parameterSchema` by running the
recipe's test vectors at registration; recipes whose vectors do not
match return `422` with type
`urn:wia:neuromorphic-chip:recipe-vectors-mismatch`.

## §21 Streaming Subscriptions and Heartbeat

Long deployments produce continuous AER traffic and plasticity
streams. Subscribers consume the streams via Server-Sent Events at
`/v1/mappings/{mid}/events` with topic filters
(`?topic=aer`, `?topic=plasticity`, `?topic=telemetry`,
`?topic=alerts`). Each subscription emits a heartbeat every 30
seconds; reconnections support replay via `Last-Event-ID` headers
(W3C EventSource semantics). High-rate AER traffic is delivered as
window summaries rather than per-event packets to prevent
subscriber overrun.

## §22 Provenance Endpoint

```
GET    /v1/provenance/{recordId}    — retrieve the provenance entry
                                       for any record described in
                                       PHASE-1
```

Provenance entries trace a record to its parents (network description,
hardware description, encoder recipes, workload definition). Tools
that audit a deployed mapping consume this endpoint to walk the
chain from mapping back to the originating network and hardware.

## §23 Health and Safety Annotations

Mappings deployed in safety-relevant settings (healthcare assistive
devices, autonomous robotics, wearable medical sensing) carry safety
annotations that name the safety standard the deployment relies on
(e.g. an adjacent WIA medical-device standard) and the safety
controls the runtime enforces (rate limits, watchdog triggers,
fail-safe states). The annotations are exposed via a dedicated query
endpoint so that auditors can compile a deployment's safety profile
without traversing every record.

## §24 Conformance

A conformant server passes the test vectors published under
`tests/phase-vectors/phase-2-api-interface/`, emits an OpenAPI 3.1
document, signs evidence packages per RFC 9421, and verifies vendor
hardware-description signatures on registration.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-neuromorphic-chip
- **Last Updated:** 2026-04-27
