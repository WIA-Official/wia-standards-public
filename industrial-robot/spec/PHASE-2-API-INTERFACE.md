# WIA-industrial-robot PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-industrial-robot
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTP API contract that an accredited
industrial-robot programme exposes for the records defined in
PHASE-1. Consumers include system integrators, MES platforms
that schedule robot tasks, APM platforms that monitor robot
health, occupational-safety inspectorates that audit incidents,
robot-vendor service teams, and citation tools that resolve
published reliability or compliance reports to their underlying
records.

References (CITATION-POLICY ALLOW only):

- IETF RFC 9110 / 9111 / 9457 / 6901 / 6902 / 8259 / 8288 / 9421
- IETF RFC 5789 (PATCH method)
- ISO 8601 (date and time)
- ISO/IEC 27001:2022 (information security management)
- ISO 10218-1 / ISO 10218-2 (robot safety)
- ISO/TS 15066 (collaborative robots)
- IEC 62443 (industrial automation security)
- OPC UA Robotics Companion Specification
- W3C Trace Context

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published by the operating
programme. Versioning uses `/v1/` path segments. The OpenAPI 3.1
document at `/v1/openapi.json` is canonical. High-rate motion
telemetry flows through the OPC UA Robotics companion in the
operator's industrial network; the WIA API exposes references
to historised samples rather than carrying high-rate streams in
the JSON layer.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-industrial-robot",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "robots":           "/v1/robots",
    "workCells":        "/v1/work-cells",
    "tasks":            "/v1/tasks",
    "motion":           "/v1/motion",
    "safetyConfig":     "/v1/safety-config",
    "safetyIncidents":  "/v1/safety-incidents",
    "maintenance":      "/v1/maintenance",
    "cyberPosture":     "/v1/cyber-posture",
    "evidence":         "/v1/evidence",
    "openapi":          "/v1/openapi.json"
  }
}
```

## §3 Robots and Work Cells

```
POST   /v1/robots                          — register a robot
GET    /v1/robots/{rid}                    — retrieve robot record
PATCH  /v1/robots/{rid}/firmware           — update controller
                                              firmware version
POST   /v1/work-cells                      — register a cell
GET    /v1/work-cells/{cid}                — retrieve cell
PATCH  /v1/work-cells/{cid}/safety-class   — update ISO 10218
                                              collaborative class
```

Submissions whose `iso10218SafetyClass` falls outside the four
ISO 10218 collaborative modes return `422` with type
`urn:wia:industrial-robot:safety-class-invalid`.

## §4 Tasks

```
POST   /v1/work-cells/{cid}/tasks          — register a task
GET    /v1/tasks/{tid}                     — retrieve task record
GET    /v1/tasks/{tid}/program             — fetch the program
                                              artefact
PATCH  /v1/tasks/{tid}/cycle-budget        — update cycle budget
```

Tasks running under collaborative cells (ISO/TS 15066) include
the biomechanical-limit profile in their record; the API rejects
task registrations that target collaborative cells without a
biomechanical-limit profile with type
`urn:wia:industrial-robot:collaborative-profile-required`.

## §5 Motion Telemetry

```
POST   /v1/robots/{rid}/motion            — append a motion sample
POST   /v1/bulk/motion                    — batched append
GET    /v1/motion/{mid}                   — retrieve sample
GET    /v1/robots/{rid}/motion?from={t}&to={t}
                                          — query window
GET    /v1/robots/{rid}/motion/around-incident?incident={iid}
                                          — fetch reconstruction
                                              window for an
                                              incident
```

The `around-incident` endpoint resolves the reconstruction window
configured against the incident record (PHASE-1 §7) and serves
the motion samples that fell within the window so that
investigators do not need to compute the time bounds themselves.

## §6 Safety Configuration

```
POST   /v1/work-cells/{cid}/safety-config — register a safety
                                              configuration
GET    /v1/safety-config/{scid}           — retrieve configuration
PATCH  /v1/safety-config/{scid}           — append revision (the
                                              configuration's
                                              prior content-
                                              address is preserved)
```

Configuration submissions whose declared `iec62061SilLevel` or
`iso13849PerformanceLevel` is below the safety-engineering
matrix's required level for the cell's intended operations return
`422` with type
`urn:wia:industrial-robot:safety-level-insufficient`.

## §7 Safety Incidents

```
POST   /v1/work-cells/{cid}/safety-incidents — register an
                                                incident
GET    /v1/safety-incidents/{iid}            — retrieve incident
PATCH  /v1/safety-incidents/{iid}/follow-up  — append follow-up
                                                investigation
```

Incidents of `severity` ≥ `major-injury` automatically trigger
an outbound notification to the configured occupational-safety
authority via the integration described in PHASE-4 §10.

## §8 Maintenance

```
POST   /v1/robots/{rid}/maintenance       — register a record
GET    /v1/maintenance/{mid}              — retrieve record
GET    /v1/robots/{rid}/maintenance?from={t}
                                          — query history
GET    /v1/robots/{rid}/repeatability?period={p}
                                          — derived ISO 9283
                                              repeatability
                                              metrics
```

## §9 Cybersecurity Posture

```
POST   /v1/work-cells/{cid}/cyber-posture — register posture
GET    /v1/cyber-posture/{pid}            — retrieve posture
PATCH  /v1/cyber-posture/{pid}/review     — append review event
```

## §10 Evidence Package

```
POST   /v1/work-cells/{cid}/evidence      — request package
                                              generation
GET    /v1/evidence/{packageId}           — retrieve a package
GET    /v1/evidence/{packageId}/manifest  — manifest only
```

## §11 Errors

All error responses are `application/problem+json` per RFC 9457.
Defined types include:

- `urn:wia:industrial-robot:safety-class-invalid`
- `urn:wia:industrial-robot:collaborative-profile-required`
- `urn:wia:industrial-robot:safety-level-insufficient`
- `urn:wia:industrial-robot:speed-zone-violation`
- `urn:wia:industrial-robot:bus-rate-mismatch`
- `urn:wia:industrial-robot:evidence-mismatch`

## §12 Authentication

The API uses mutually-authenticated TLS for system integrator,
MES, APM, vendor-service, and regulator client certificates.
Robot-controller cells authenticate through the operator's
industrial-security broker before reaching the WIA API.

## §13 Streaming

Subscribers consume cell events via Server-Sent Events at
`/v1/work-cells/{cid}/events`. Topics include safety-incident
emissions, safety-configuration revisions, task completions, and
firmware updates.

## §14 Bulk and Pagination

Bulk endpoints accept arrays of motion samples for high-rate
ingest from the operator's historian. Cursor-based pagination
uses the `cursor` query parameter and `Link` headers (RFC 8288).

## §15 Privacy-Preserving Aggregation

Aggregate consumers (industry analysts, occupational-safety
researchers) fetch population-level statistics through endpoints
that emit counts, means, and dispersions:

```
GET    /v1/aggregate/incident-rate?category=...&period=...
GET    /v1/aggregate/repeatability?model=...&period=...
GET    /v1/aggregate/cycle-time?task-kind=...&period=...
```

Out-of-policy queries return `403 Forbidden` with type
`urn:wia:industrial-robot:cohort-too-small`.

## §16 Worked Example: From Cell Commissioning to Citation

1. Integrator POSTs the robot record at commissioning; the
   kinematic descriptor is content-addressed.
2. Work cell is registered with collaborative-mode classification.
3. Safety configuration is registered with SIL/PL levels and
   biomechanical limits.
4. Tasks are registered as the cell ramps up production.
5. Motion telemetry streams into the historian; periodic
   aggregations populate ISO 9283 repeatability KPIs.
6. A speed-limit exceedance is recorded; the
   `around-incident` endpoint surfaces the motion-sample window
   for investigators.
7. Citation tool requests an evidence package for the cell;
   manifest digest is pinned for downstream reference.

## §17 End-Effector Endpoints

```
POST   /v1/robots/{rid}/end-effectors    — register an end-
                                            effector for the robot
GET    /v1/end-effectors/{eeid}          — retrieve effector
                                            record
PATCH  /v1/end-effectors/{eeid}/swap     — record an effector
                                            swap event
```

Swap events trigger re-validation of the safety configuration's
biomechanical-limit profile when the cell operates under
collaborative modes; the API records the re-validation outcome
against the swap event and refuses to dispatch new tasks until
the validation completes.

## §18 Provenance Endpoint

```
GET    /v1/provenance/{recordId}    — retrieve provenance entry
                                        for any PHASE-1 record
```

Provenance entries trace a cell's evidence package back to its
parents (robot record, kinematic descriptor, safety configuration
revisions, integrator commissioning evidence) so that auditors
can walk the chain end-to-end.

## §19 Audit and Observability

Every endpoint emits structured logs with `cellId`, `robotId`,
`traceId`, the issuing client certificate's subject, and the
controller / historian clock skew vs the reference NTP source.

## §20 Operator-Authorisation Endpoints

```
POST   /v1/work-cells/{cid}/authorised-operators
                                          — register an authorised
                                            operator
GET    /v1/work-cells/{cid}/authorised-operators
                                          — list authorised
                                            operators
PATCH  /v1/work-cells/{cid}/authorised-operators/{oid}/scope
                                          — adjust operator scope
                                            (production / teach /
                                            safety-config)
```

Authorised-operator records carry only opaque tokens; clinical
or HR identifiers in the body return `422` with type
`urn:wia:industrial-robot:identifier-leak`.

## §21 Embargo and Coordinated Releases

Coordinated launches (a new collaborative-cell deployment that
ties to a public press announcement) are held under embargo
until the agreed release time. The operator records the embargo
holder and the release time; the API returns `403 Forbidden`
with type `urn:wia:industrial-robot:embargo-active` to
non-authorised clients before the release time.

## §22 Vision-Binding Endpoints

```
POST   /v1/robots/{rid}/vision-bindings  — register a vision
                                            calibration binding
GET    /v1/vision-bindings/{vbid}        — retrieve binding
PATCH  /v1/vision-bindings/{vbid}/recalibrate
                                          — record a recalibration
                                            outcome
```

Binding submissions whose `reprojectionErrorPx` exceeds the
operator's declared threshold flag the cell for re-validation
under §6 safety-configuration governance.

## §23 Conformance

A conformant server passes the test vectors published under
`tests/phase-vectors/phase-2-api-interface/`, emits an OpenAPI
3.1 document, and signs evidence packages per RFC 9421.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-industrial-robot
- **Last Updated:** 2026-04-27
