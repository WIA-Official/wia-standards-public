# WIA-infrastructure-monitoring PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-infrastructure-monitoring
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTP API contract that an accredited
infrastructure-monitoring operator exposes for the records defined
in PHASE-1. Consumers include asset owners receiving monitoring
status, SHM contractors authoring sensor and analysis records,
calibration laboratories binding ISO/IEC 17025 certificates to
sensors, post-event responders pulling captured windows, regulators
reviewing dam-safety or bridge-safety monitoring evidence, and
citation tools that resolve published monitoring reports to their
underlying records.

References (CITATION-POLICY ALLOW only):

- IETF RFC 9110 (HTTP Semantics) / 9111 (HTTP Caching) /
  9457 (Problem Details) / 6901 / 6902 / 8288 (Web Linking) /
  8259 (JSON) / 9421 (HTTP Message Signatures)
- ISO 8601 (date and time)
- ISO 13822 (assessment of existing structures)
- ISO 16587 (SHM performance parameters)
- ISO 4866 (vibration of fixed structures)
- IEEE 1451 (smart-transducer interface)
- OGC SensorThings API 1.1
- W3C Trace Context

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published by the operator.
Versioning uses `/v1/` path segments and Semantic Versioning
2.0.0. The OpenAPI 3.1 document at `/v1/openapi.json` is
canonical.

The API is a control-plane and metadata facade; bulk raw-sample
data flows over the operator's archive (referenced by the
`archivalContentAddress` field in PHASE-1 §6) — the API
*describes* the sample windows but does not stream the bulk
samples themselves.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-infrastructure-monitoring",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "sensors":           "/v1/sensors",
    "mountings":         "/v1/mountings",
    "calibrations":      "/v1/calibrations",
    "timeSyncs":         "/v1/time-syncs",
    "sampleWindows":     "/v1/sample-windows",
    "derivedMetrics":    "/v1/derived-metrics",
    "thresholdBreaches": "/v1/threshold-breaches",
    "alerts":            "/v1/alerts",
    "postEventCaptures": "/v1/post-event-captures",
    "evidence":          "/v1/evidence",
    "openapi":           "/v1/openapi.json"
  }
}
```

## §3 Sensor Endpoints

```
POST   /v1/sensors                  — register a sensor
GET    /v1/sensors/{sid}            — retrieve sensor record
PATCH  /v1/sensors/{sid}/sensitivity — replace sensitivity (must
                                       cite calibrationId)
DELETE /v1/sensors/{sid}            — initiate decommissioning
GET    /v1/sensors/{sid}/timeline   — chronological timeline of
                                       commissioning, calibration,
                                       remount, post-event capture,
                                       decommissioning
```

Sensor sensitivity updates that do not cite a calibration record
return `422` with type
`urn:wia:infrastructure-monitoring:calibration-required`.

## §4 Mounting Endpoints

```
POST   /v1/sensors/{sid}/mountings   — register a mounting
GET    /v1/mountings/{mid}           — retrieve mounting record
PATCH  /v1/mountings/{mid}/remount   — replace mounting; emits
                                        successor record
GET    /v1/sensors/{sid}/mountings?at={iso8601}
                                     — resolve mounting as of a
                                       given date
```

The "as-of" query is critical for historical sample windows:
samples that were recorded before a remount must resolve to the
mounting that produced them.

## §5 Calibration Endpoints

```
POST   /v1/sensors/{sid}/calibrations    — register a calibration
GET    /v1/calibrations/{cid}             — retrieve calibration
PATCH  /v1/calibrations/{cid}/revoke      — revoke calibration;
                                            emits successor with
                                            stated revocation
                                            reason
```

Calibration submissions whose `traceabilityRef` does not resolve
to a recognised national metrology institute (KRISS, NIST, NPL,
PTB, or BIPM-equivalent) return `422` with type
`urn:wia:infrastructure-monitoring:traceability-unresolved`.

## §6 Time-Sync Endpoints

```
POST   /v1/sensors/{sid}/time-syncs        — register a sync
                                              binding observation
GET    /v1/time-syncs/{tsid}                — retrieve binding
GET    /v1/sensors/{sid}/time-syncs?from={iso8601}
                                            — query a sync history
                                              window
```

Time-sync observations whose `observedSkewMs` exceeds the per-
sensor budget (operator-declared in the procedure register) emit
priority-1 events on the operator's event stream.

## §7 Sample-Window Endpoints

```
POST   /v1/sensors/{sid}/sample-windows   — register a sample
                                             window envelope
GET    /v1/sample-windows/{wid}           — retrieve envelope
GET    /v1/sample-windows/{wid}/archive   — resolve to the bulk-
                                             sample archive
                                             content-address
PATCH  /v1/sample-windows/{wid}/correction — register a clock-
                                             skew correction or
                                             re-windowing; emits
                                             successor envelope
```

Sample-window submissions whose `samplingRateHz` is below the
sensor's procedure-register minimum (e.g. an accelerometer
sampled below 100 Hz when the procedure mandates 1 kHz) return
`422` with type
`urn:wia:infrastructure-monitoring:sampling-rate-below-procedure`.

## §8 Derived-Metric Endpoints

```
POST   /v1/derived-metrics                 — register a derived
                                              metric value
GET    /v1/derived-metrics/{dmid}          — retrieve metric
GET    /v1/derived-metrics?sensorId={sid}&from={iso8601}
                                            — query a metric
                                              series for a sensor
```

Re-derivation of a metric (e.g. the natural-frequency estimate
re-computed under an updated modal-analysis pipeline) emits a
successor metric record with `predecessor` set; the prior
record remains addressable for citation purposes.

## §9 Threshold-Breach and Alert Endpoints

```
POST   /v1/threshold-breaches              — register a breach
GET    /v1/threshold-breaches/{bid}        — retrieve breach
PATCH  /v1/threshold-breaches/{bid}/end    — record breach end

POST   /v1/alerts                          — register an alert
GET    /v1/alerts/{aid}                    — retrieve alert
PATCH  /v1/alerts/{aid}/acknowledge        — acknowledge
PATCH  /v1/alerts/{aid}/resolve            — resolve
```

Alert acknowledgement and resolution emit audit events; the
operator's incident-response procedure register specifies the
maximum acknowledgement latency per severity class, and the API
emits priority-1 events when acknowledgement deadlines lapse.

## §10 Post-Event Capture Endpoints

```
POST   /v1/post-event-captures             — register a capture
GET    /v1/post-event-captures/{cid}       — retrieve capture
GET    /v1/post-event-captures?eventRef={ref}
                                            — find captures by
                                              triggering event
```

Post-event captures bind a triggering event reference (national
seismic-network event ID, NWS storm ID, owner incident ID) to
the set of sensor windows captured during the response so that
post-event analyses resolve to the canonical captured set.

## §11 Evidence Package

```
POST   /v1/sensors/{sid}/evidence          — request evidence
                                              package generation
GET    /v1/evidence/{packageId}            — retrieve package
GET    /v1/evidence/{packageId}/manifest   — manifest only
```

The evidence-package format is governed by PHASE-4 §3 and contains
the sensor record, the mounting history, the calibration chain,
the time-sync history, sample-window envelopes, derived-metric
series, threshold breaches and alerts, post-event captures, and
the signed manifest.

## §12 Errors

All error responses are `application/problem+json` per RFC 9457.
Defined types include:

- `urn:wia:infrastructure-monitoring:calibration-required`
- `urn:wia:infrastructure-monitoring:traceability-unresolved`
- `urn:wia:infrastructure-monitoring:sampling-rate-below-procedure`
- `urn:wia:infrastructure-monitoring:mounting-not-found`
- `urn:wia:infrastructure-monitoring:evidence-mismatch`

## §13 Authentication

Mutually-authenticated TLS for all write operations; client
certificates are scoped to the sensor populations the contractor
or laboratory operates. Public read-only endpoints (sensor
catalogue summaries, regulator-published monitoring indices) are
reachable without a client certificate.

## §14 Caching

Stable resources (signed calibrations, completed sample-window
envelopes, resolved alerts) are cacheable with `Cache-Control:
max-age=31536000, immutable`. Mutable resources (open alerts,
in-flight breaches) are cacheable for 60 seconds; ETags are
mandatory on every PATCH endpoint with `If-Match` conditional
requests.

## §15 Streaming Subscriptions

Consumers subscribe via Server-Sent Events at
`/v1/sensors/{sid}/events`. Topics include sample-window
publication, threshold-breach onset, alert state change, and
post-event capture publication. Heartbeats every 30 seconds with
`Last-Event-ID` resume support.

## §16 Bulk Operations

```
POST   /v1/bulk/sample-windows         — submit envelopes for many
                                          sensors and windows
POST   /v1/bulk/derived-metrics        — submit derived metrics
                                          for a campaign
GET    /v1/bulk/{operationId}          — operation status
```

## §17 Provenance

```
GET    /v1/provenance/{recordId}    — retrieve provenance entry
                                       for any PHASE-1 record
```

## §18 Worked Example: Capture and Notify after a Seismic Event

1. Regional seismic network publishes a magnitude-4.6 event near
   a monitored long-span bridge.
2. The operator's acquisition system pre-buffers windows for
   accelerometers and tilt-meters on the bridge; on receipt of
   the event ID it materialises a post-event capture (PHASE-1
   §10) covering 30 s pre-event to 5 min post-event.
3. The acquisition system computes ISO 4866 vibration-severity
   metrics on the captured windows and registers a derived-
   metric record per accelerometer.
4. Threshold breaches at severity `alert` automatically emit
   alert records and notify the on-call rota.
5. The asset programme's post-event inspector pulls the
   captured windows via §10, performs an in-depth inspection
   per WIA-infrastructure PHASE-1 §5, and records the finding.
6. The operator's regulator notification intake receives the
   threshold-breach notification within the regulator's
   declared latency budget.

## §19 Audit and Observability

Every endpoint emits structured logs with `assetId`, `sensorId`,
`traceId`, the issuing client certificate's subject, and the
acquisition unit's clock skew vs the reference sync source.
Audit logs are immutable and sealed daily into the operator's
transparency log.

## §20 Conformance

A conformant server passes the test vectors published under
`tests/phase-vectors/phase-2-api-interface/`, emits an OpenAPI
3.1 document, and signs evidence packages per RFC 9421.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-infrastructure-monitoring
- **Last Updated:** 2026-04-28
