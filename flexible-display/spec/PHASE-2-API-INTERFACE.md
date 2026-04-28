# WIA-flexible-display PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-flexible-display
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTPS API contract that
a flexible-display operator (display
manufacturer, contract module assembler,
metrology laboratory, notified body, or
certification body) exposes for the records
defined in PHASE-1. The contract carries the
display registration, optical-performance test
upload, mechanical-reliability test upload,
environmental-reliability test upload, EMC-and-
safety test upload, and chain-of-custody
anchoring endpoints.

References (CITATION-POLICY ALLOW only):

- IETF RFC 9110, RFC 9111, RFC 9457, RFC 8288,
  RFC 6901 / 6902, RFC 8259, RFC 4122, RFC
  9421, RFC 8615
- W3C Trace Context
- ISO/IEC 27001:2022, ISO/IEC 17021-1:2015,
  ISO/IEC 17065:2012, ISO/IEC 17025:2017
- IEC 62977 series (multimedia display
  measurement methods) and IEC 62715 series
  (flexible display devices)
- IEC 62341 series (OLED display) and IEC 61747
  series (LCD module)
- IEC TS 62687:2014, ISO 9241-302/-303/-306/-307
- SID IDMS 1.03
- JEITA RC-9131 and KS C IEC 62715-1-1 / KS C
  IEC 62977-1
- IEC 60068 series (environmental testing) and
  IEC 61000 series (EMC)
- IEC 62368-1:2018+AMD1:2020 (audio-video safety)
- EU Low Voltage Directive 2014/35/EU, EU EMC
  Directive 2014/30/EU, EU RoHS 2011/65/EU, EU
  REACH (EC) 1907/2006

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published by
the operator. The OpenAPI 3.1 document at
`/v1/openapi.json` is canonical. Schema changes
follow the non-breaking conventions in PHASE-1
§2. Every endpoint carries a per-request
signature using HTTP Message Signatures (RFC
9421) anchored to the operator's ISO/IEC 17025
laboratory accreditation, the operator's ISO/IEC
17065 certification-body accreditation, or the
operator's notified-body designation under the
relevant EU directive; the signature key set is
published at
`/.well-known/wia/flexible-display/keys.json`.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-flexible-display",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "programmes":         "/v1/programmes",
    "displays":           "/v1/displays",
    "opticalRecords":     "/v1/optical-records",
    "mechanicalRecords":  "/v1/mechanical-records",
    "environmentalRecords": "/v1/environmental-records",
    "emcSafetyRecords":   "/v1/emc-safety-records",
    "custody":            "/v1/custody-events",
    "openapi":            "/v1/openapi.json",
    "wellKnown":          "/.well-known/wia/flexible-display"
  }
}
```

## §3 Display Registration Endpoints

### §3.1 Register a display

```
POST /v1/displays
Content-Type: application/json
Signature: <RFC 9421 signature>
```

Request body carries the §3 record from PHASE-1
(displayFamily, substrate, resolution,
pixelPitchMicron, flexibilityClass,
declaredBendRadiusMm, declaredFoldCycles). The
server validates the `flexibilityClass` against
the declared `substrate`: a `stretchable`
flexibility class declared on a `thin-glass`
substrate is rejected with `422 Unprocessable
Entity` at `/problems/iec62715-substrate-
flexibility-mismatch` and the offending field
expressed as a JSON Pointer (RFC 6901), since
thin-glass substrates do not support the strain
range of stretchable applications under IEC
62715-5-1.

### §3.2 Retrieve a display

```
GET /v1/displays/{displayId}
Accept: application/json
```

### §3.3 Search displays

```
GET /v1/displays?family={family}
&substrate={substrate}
&flexibilityClass={class}
&minBendRadiusMm={number}
&page={cursor}&size={size}
```

## §4 Optical-Performance Test Endpoints

### §4.1 Upload an optical-performance test

```
POST /v1/optical-records
Content-Type: multipart/form-data; boundary=...
Signature: <RFC 9421 signature from a metrology
            laboratory's ISO/IEC 17025 certificate>
```

The multipart body carries one JSON part with the
§4 record from PHASE-1 (testStandard,
testLaboratory, measurementResult,
measurementGeometry) and one or more file parts
holding the spectroradiometer raw data, the
luminance-meter data, the reflectometer data,
and the supporting calibration record. The
server stores the raw-data files under a content-
addressable URI.

The server enforces the test-method-to-
measurement-result mapping table:

- An IEC 62977-2-1 (colorimetry) record MUST
  carry the JNCD value, the colour-volume
  coverage, and the gamma value computed per
  the standard.
- An IEC 62977-2-2 (electro-optical) record
  MUST carry the contrast ratio and the gamma
  curve.
- An IEC 62977-2-3 (uniformity) record MUST
  carry the per-zone luminance variation per
  the SID IDMS 1.03 §6 luminance-uniformity
  procedure.

### §4.2 Retrieve an optical record

```
GET /v1/optical-records/{opticalRecordId}
Accept: application/json
```

## §5 Mechanical-Reliability Test Endpoints

### §5.1 Upload a folding-endurance test

```
POST /v1/mechanical-records
Content-Type: multipart/form-data; boundary=...
Signature: <RFC 9421 signature>
```

Request body carries the §5 record from PHASE-1.
The server validates that the test-protocol
parameters (declared bend radius, fold-cycle
speed, environmental-chamber temperature, and
relative humidity) match the IEC 62715-6-1
default protocol or carry the deviation
declaration the laboratory must publish per the
standard's §5.4. A test conducted outside the
default protocol without a deviation declaration
is rejected with `422 Unprocessable Entity` at
`/problems/iec62715-6-1-deviation-undeclared`.

### §5.2 Retrieve a mechanical record

```
GET /v1/mechanical-records/{mechanicalId}
Accept: application/json
```

## §6 Environmental-Reliability Test Endpoints

### §6.1 Upload an environmental test

```
POST /v1/environmental-records
Content-Type: multipart/form-data; boundary=...
Signature: <RFC 9421 signature>
```

Request body carries the §6 record from PHASE-1.
The server enforces that an `IEC-60068-2-30
(damp heat)` test record carries the cyclic-
temperature-and-humidity profile per the
standard's §4 (the upper-limit temperature, the
relative humidity, the cycle count, and the
ramp-rate envelope).

### §6.2 Retrieve an environmental record

```
GET /v1/environmental-records/{environmentalId}
Accept: application/json
```

## §7 EMC-and-Safety Test Endpoints

### §7.1 Upload an EMC or safety test

```
POST /v1/emc-safety-records
Content-Type: application/json
Signature: <RFC 9421 signature>
```

Request body carries the §7 record from PHASE-1.
The server validates that an `IEC-62368-1
(audio-video safety)` record references the
device's classification under the standard's
hazard-based safety engineering (HBSE) energy-
source classification table.

### §7.2 Retrieve an EMC or safety record

```
GET /v1/emc-safety-records/{emcRecordId}
Accept: application/json
```

## §8 Custody and Error Reporting

### §8.1 Anchor a custody event

```
POST /v1/custody-events
Content-Type: application/json
Signature: <RFC 9421 signature>
```

### §8.2 Error envelope

Errors are returned using RFC 9457 Problem
Details. Validation errors carry a `pointer`
(RFC 6901). The server emits a per-request
`traceparent` header (W3C Trace Context).

## §9 Concurrency and Cache

Every retrieval endpoint emits an `ETag` header
(RFC 9110 §8.8.3). Conditional requests
(`If-Match`, `If-None-Match`) are honoured.

## §10 Bulk Export for Quality-Assurance Audits

```
GET /v1/displays:bulk
Accept: application/x-ndjson
```

A QA team running a bulk audit of the
manufacturer's released displays requests the
operator's display register as a newline-
delimited JSON stream. The endpoint streams the
display records in release-date order; a
consumer resuming the stream provides an
`If-Resume-After` header carrying the last-
received release timestamp.

## §11 Notified-Body Surveillance Endpoint

```
GET /v1/programmes/{programmeId}/surveillance
Accept: application/json
Signature: <RFC 9421 signature from a notified
            body or certification body>
```

A notified body conducting EU LVD or EU EMC
Directive surveillance, or a certification body
running ISO/IEC 17065 product-certification
surveillance, requests the operator's per-batch
test result set, the per-batch FPC measurement,
and the per-batch CE-marking attestation.

## §12 Schema-Validation and Conformance Endpoint

The OpenAPI 3.1 document at `/v1/openapi.json`
carries JSON Schema 2020-12 schemas for every
request and response envelope.

```
GET /v1/conformance/test-vectors
Accept: application/json
```

The operator publishes the conformance test
vectors used to qualify the API implementation
against this specification. Each vector carries
the request, the expected response, and the
SID IDMS 1.03 measurement-procedure traceability
link so that a downstream metrology laboratory
can confirm the operator's API reproduces the
published measurement-result derivation
deterministically.

## §13 Bilingual Public Retrieval

```
GET /v1/displays/{displayId}?lang={lang}
Accept: application/json
```

The operator's public retrieval endpoint accepts
a `lang` query parameter (`en`, `ko`, `ja`,
`zh`, `de`, `fr`) and returns the public fields
with the harmonised-standard reference in the
requested language. Numeric measurement values,
ISO 8601 timestamps, and IEC term-codes are
language-neutral and carried in the canonical
form across all retrievals.

## §14 Webhook Endpoint for Recall Notifications

```
POST /v1/programmes/{programmeId}/webhooks
Content-Type: application/json
Signature: <RFC 9421 signature>
```

A regulator or distributor registers a webhook
to receive a push notification when a
manufacturer issues a recall under EU LVD
Article 8 or the equivalent national-
jurisdiction recall regime. The webhook envelope
carries the operator's signing-key reference,
the event type, and the resource identifier.

## §15 Per-Geometry Optical Comparison Endpoint

```
GET /v1/displays/{displayId}/optical-comparison
  ?geometryA={geom}&geometryB={geom}
Accept: application/json
```

The endpoint returns the per-metric difference
between two measurement geometries (for example
flat-state vs. folded-fully) so that a downstream
buyer or system integrator can quantify the
optical-performance change across the
mechanical-flexibility envelope. The differences
are computed against the SID IDMS 1.03
measurement procedures and carry the per-metric
JNCD (just-noticeable colour difference) value
where the comparison applies to colour metrics.

## §16 Module-to-Product Linking Endpoint

```
POST /v1/displays/{displayId}/system-integrators
Content-Type: application/json
Signature: <RFC 9421 signature from a system-
            integrator's identifier>
```

A system integrator (a phone manufacturer, a
TV manufacturer, an automotive cluster
integrator) declares the system-level product
binding. The request carries the system-level
product identifier, the system integrator's
legal name, and the system-level certification
reference (the system-level CE marking, FCC ID,
KR KC marking) so that a downstream consumer
warranty-claim or recall workflow can trace the
system-level event to the underlying flexible-
display module.
