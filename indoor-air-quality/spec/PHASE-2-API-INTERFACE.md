# WIA-indoor-air-quality PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-indoor-air-quality
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTP API contract that an accredited
indoor-air-quality programme exposes for the records defined in
PHASE-1. Consumers include building owners and operators,
mechanical contractors, accredited environmental laboratories,
sensor-package vendors, occupational-health representatives,
public-health authorities, and citation tools that resolve
published IAQ findings to their underlying records.

References (CITATION-POLICY ALLOW only):

- IETF RFC 9110 (HTTP Semantics)
- IETF RFC 9111 (HTTP Caching)
- IETF RFC 9457 (Problem Details)
- IETF RFC 6901 (JSON Pointer)
- IETF RFC 6902 (JSON Patch)
- IETF RFC 8288 (Web Linking)
- IETF RFC 8259 (JSON)
- IETF RFC 9421 (HTTP Message Signatures)
- ISO 8601 (date and time)
- ISO 16000 series
- ISO/IEC 27001:2022 (information security management)
- ASHRAE Standard 62.1 / 62.2
- WHO Guidelines for Indoor Air Quality
- W3C Trace Context

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published by the operating
programme. Versioning uses `/v1/` path segments. The OpenAPI 3.1
document at `/v1/openapi.json` is canonical.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-indoor-air-quality",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "sites":            "/v1/sites",
    "zones":            "/v1/zones",
    "iaqSamples":       "/v1/iaq-samples",
    "episodicSamples":  "/v1/episodic-samples",
    "verifications":    "/v1/verifications",
    "symptoms":         "/v1/symptoms",
    "investigations":   "/v1/investigations",
    "remediations":     "/v1/remediations",
    "evidence":         "/v1/evidence",
    "openapi":          "/v1/openapi.json"
  }
}
```

## §3 Sites and Zones

```
POST   /v1/sites                            — register a site
GET    /v1/sites/{sid}                      — retrieve site record
PATCH  /v1/sites/{sid}/ventilation-strategy — update strategy
POST   /v1/sites/{sid}/zones                — register a zone
GET    /v1/zones/{zid}                      — retrieve zone record
PATCH  /v1/zones/{zid}/filters              — append filter rotation
                                              event
```

## §4 Continuous IAQ Samples

```
POST   /v1/zones/{zid}/iaq-samples         — append a sample
POST   /v1/bulk/iaq-samples                — batched append
GET    /v1/iaq-samples/{sid}               — retrieve sample
GET    /v1/zones/{zid}/iaq-samples?from={t}&to={t}
                                            — query window
GET    /v1/zones/{zid}/iaq-samples/alerts  — current threshold
                                              alerts
```

Sensor-package category gates which submissions are accepted into
records that drive regulatory or public-health-relevant decisions:
`consumer-grade` packages are accepted into trend records; only
`accredited-laboratory-grade` packages or episodic samples (§5) are
accepted as evidence for ventilation-verification or occupational-
exposure decisions. Submissions from out-of-scope categories return
`409 Conflict` with type
`urn:wia:indoor-air-quality:sensor-category-mismatch`.

## §5 Episodic Sampling

```
POST   /v1/zones/{zid}/episodic-samples     — register a sample
                                              result
GET    /v1/episodic-samples/{sid}           — retrieve sample
PATCH  /v1/episodic-samples/{sid}/uncertainty — append uncertainty
                                              contributions
```

Episodic samples are signed by the analysing laboratory's client
certificate; the API verifies the laboratory's ISO/IEC 17025
accreditation against the public register before accepting the
record.

## §6 Ventilation Verifications

```
POST   /v1/sites/{sid}/verifications      — register a verification
GET    /v1/verifications/{vid}            — retrieve verification
GET    /v1/verifications/{vid}/report     — fetch full commissioning
                                            report
```

Verifications whose `outdoorAirMeasuredCfm` falls below
`outdoorAirRequiredCfm` flag the affected zones in the public
catalogue and emit an alert through the streaming endpoint so that
operators can plan corrective action.

## §7 Symptoms

```
POST   /v1/zones/{zid}/symptoms        — register an occupant symptom
GET    /v1/symptoms/{sid}              — retrieve symptom record
GET    /v1/zones/{zid}/symptoms?since={t}&category={c}
                                       — query window by category
```

Symptom records cite occupants through opaque tokens (PHASE-1 §7);
clinical identifiers in the body return `422` with type
`urn:wia:indoor-air-quality:identifier-leak`. Free-text fields are
served only to authorised occupational-health roles.

## §8 Investigations and Remediations

```
POST   /v1/sites/{sid}/investigations       — register an investigation
GET    /v1/investigations/{iid}             — retrieve investigation
PATCH  /v1/investigations/{iid}/hypothesis  — append hypothesis entry
PATCH  /v1/investigations/{iid}/close       — close with root cause
POST   /v1/sites/{sid}/remediations         — register a remediation
GET    /v1/remediations/{rid}               — retrieve remediation
PATCH  /v1/remediations/{rid}/post-verification — attach post-action
                                                  verification
```

## §9 Evidence Package

```
POST   /v1/sites/{sid}/evidence       — request package generation
GET    /v1/evidence/{packageId}       — retrieve package
GET    /v1/evidence/{packageId}/manifest — manifest only
```

The evidence-package format is governed by PHASE-4 §3 and contains
site, zone, IAQ-sample (optionally summarised), episodic-sample,
verification, investigation, remediation, and signed manifest.

## §10 Errors

All error responses are `application/problem+json` per RFC 9457.
Defined types include:

- `urn:wia:indoor-air-quality:sensor-category-mismatch`
- `urn:wia:indoor-air-quality:identifier-leak`
- `urn:wia:indoor-air-quality:laboratory-not-accredited`
- `urn:wia:indoor-air-quality:ventilation-deficit`
- `urn:wia:indoor-air-quality:evidence-mismatch`

## §11 Authentication

The API uses mutually-authenticated TLS for laboratory, contractor,
and authority connections. Public read-only endpoints (released
verification summaries, aggregate trend reports, the OpenAPI
document) are reachable without a client certificate.

## §12 Caching

Stable resources (signed verifications, completed investigations,
evidence packages) are cacheable with `Cache-Control: max-age=
31536000, immutable`. Mutable resources are cacheable for 60 seconds.

## §13 Worked Example: From Telemetry to Remediation

1. Continuous IAQ sensor reports rising CO2 in a classroom zone.
2. Bulk-uploaded samples populate the trend dataset; an alert fires
   at the configured threshold.
3. Operator schedules an episodic sample to confirm the trend.
4. Episodic sample (ISO 16000-32 radon, ISO 16000-37 PM) confirms
   the air-change-rate deficit.
5. Source investigation identifies a stuck damper.
6. Remediation action repairs the damper; post-action verification
   confirms restoration to the ASHRAE 62.1 outdoor-air requirement.
7. Citation tool requests an evidence package for the incident;
   manifest digest is pinned for stakeholder reference.

## §14 Sensor-Package and Threshold Endpoints

```
POST   /v1/sensor-packages              — register a sensor package
GET    /v1/sensor-packages/{pid}        — retrieve package record
PATCH  /v1/sensor-packages/{pid}/firmware — update firmware version
POST   /v1/sensor-packages/{pid}/calibrations — append calibration
                                                entry
POST   /v1/threshold-tables             — publish a threshold table
GET    /v1/threshold-tables/{tid}       — retrieve a threshold table
GET    /v1/zones/{zid}/alerts?table={tid}
                                        — alerts driven by a table
```

Threshold tables are versioned; consumers pin a particular table
revision in their alerting subscriptions so that subsequent table
updates do not silently change alert behaviour.

## §15 Streaming Subscriptions

Operators and occupational-health teams subscribe to zone events
via Server-Sent Events at `/v1/zones/{zid}/events`. Topics include
threshold breaches, ventilation-deficit detections, episodic-
sample arrivals, and remediation completions. Subscriptions emit a
heartbeat every 30 seconds; replays support `Last-Event-ID` headers
(W3C EventSource semantics).

## §16 Bulk and Pagination

Bulk endpoints accept arrays of continuous samples. Cursor-based
pagination uses the `cursor` query parameter and `Link` headers
(RFC 8288); cursors are opaque to clients and persist for at least
24 hours.

## §17 Privacy-Preserving Aggregation

Public consumers fetch aggregate IAQ trends through endpoints that
emit counts, means, and dispersions. The cohort-size threshold
prevents inference-by-intersection on small operator portfolios:

```
GET    /v1/aggregate/co2?function=...&period=...
GET    /v1/aggregate/pm25?function=...&period=...
GET    /v1/aggregate/symptom-rates?function=...&category=...
```

Out-of-policy queries return `403 Forbidden` with type
`urn:wia:indoor-air-quality:cohort-too-small`.

## §18 Audit and Observability

Every endpoint emits structured logs with `siteId`, `zoneId`,
`traceId`, the issuing client certificate's subject, and the
sensor-package or laboratory clock skew vs the reference NTP source.

## §19 Thermal-Comfort Endpoints

```
POST   /v1/zones/{zid}/thermal-comfort     — append observation
GET    /v1/thermal-comfort/{oid}           — retrieve observation
GET    /v1/zones/{zid}/thermal-comfort?from={t}&to={t}
                                            — query window
```

Thermal-comfort observations are accepted independently of IAQ
samples; integrators consume the streams together when the symptom
analysis warrants distinguishing thermal from air-quality
contributions.

## §20 Provenance Endpoint

```
GET    /v1/provenance/{recordId}    — retrieve provenance entry
                                       for any PHASE-1 record
```

Provenance entries trace IAQ samples and episodic results to their
parents (sensor package, laboratory accreditation, threshold table)
so that auditors can walk the evidence chain.

## §21 Worked Example: From Sensor Drift to Recalibration

1. Continuous IAQ sensor reports anomalous CO2 trends following a
   firmware update.
2. Drift-policy alert fires through the streaming endpoint.
3. Operator schedules an episodic comparison against a co-located
   accredited reference instrument.
4. Comparison result is appended; sensor is recategorised as
   `professional-grade-non-accredited` until recalibration.
5. Vendor performs recalibration; calibration-entry append updates
   the package's history.
6. Operator re-accepts recent submissions only after the comparison
   demonstrates restored agreement.

## §22 FHIR Bridge for Symptom Aggregates

Sites that integrate with healthcare facility management MAY
expose a read-only FHIR R5 facade that translates aggregated
symptom counts into FHIR `Observation` resources at the
population-cohort level. Identifiable symptom records remain
inside the operator's CRM; only aggregates flow to the FHIR facade.

## §23 Conformance

A conformant server passes the test vectors published under
`tests/phase-vectors/phase-2-api-interface/`, emits an OpenAPI 3.1
document, and signs evidence packages per RFC 9421.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-indoor-air-quality
- **Last Updated:** 2026-04-27
