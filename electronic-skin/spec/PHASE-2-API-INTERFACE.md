# WIA-electronic-skin PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-electronic-skin
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTPS API contract that
an electronic-skin operator (device manufacturer,
contract manufacturer, notified body, clinical-
investigation sponsor, treating-healthcare
provider, or analytics service) exposes for the
records defined in PHASE-1. The contract carries
the device-registration, observation upload, FHIR
mapping publication, patient binding and consent,
software-update distribution, biocompatibility-
test ingestion, and chain-of-custody anchoring
endpoints.

References (CITATION-POLICY ALLOW only):

- IETF RFC 9110 (HTTP Semantics), RFC 9111 (HTTP
  Caching), RFC 9457 (Problem Details), RFC 8288
  (Web Linking), RFC 6901 / 6902 (JSON Pointer /
  Patch), RFC 8259 (JSON), RFC 4122 (UUID), RFC
  9421 (HTTP Message Signatures), RFC 8615 (well-
  known URIs)
- W3C Trace Context
- ISO/IEC 27001:2022, ISO/IEC 17021-1:2015
- IEC 60601-1:2005+AMD2:2020 (medical electrical
  equipment safety) and IEC 80601-2-49:2018
  (multifunction patient monitoring)
- IEC 62304:2006+AMD1:2015 (medical-device
  software life-cycle), IEC 62366-1:2015+AMD1:
  2020 (usability engineering)
- ISO 14971:2019 (medical-device risk
  management) and ISO 13485:2016+A11:2021
  (medical-device QMS)
- IEEE 11073-10101:2019 (PHD nomenclature) and
  IEEE 11073 specialisation profiles cited in
  PHASE-1 references
- HL7 FHIR Release 5 RESTful API (search, read,
  create, update, history interactions)
- ISO 10993-1:2018, ISO 10993-5:2009, ISO 10993-
  10:2010, ISO 10993-23:2021 (biocompatibility
  test envelope carried by §6)
- EU Medical Device Regulation (EU) 2017/745
  (UDI carried in registration), US 21 CFR Part
  820 (QSR for shipment-record fields), KR
  의료기기법 (KR registration discipline)

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published by
the operator. The OpenAPI 3.1 document at
`/v1/openapi.json` is canonical for the WIA
endpoints, and the FHIR `CapabilityStatement`
resource at `/fhir/r5/metadata` is canonical for
the FHIR endpoints. Schema changes follow the
non-breaking conventions in PHASE-1 §2 (additive
fields, additive enum values). Every endpoint
carries a per-request signature using HTTP
Message Signatures (RFC 9421) anchored to the
operator's ISO 13485 QMS certification, the
device's UDI, or the patient-binding pseudonymous
identifier where the request originates from a
patient-portal session; the signature key set is
published at
`/.well-known/wia/electronic-skin/keys.json`.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-electronic-skin",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "programmes":         "/v1/programmes",
    "devices":            "/v1/devices",
    "observations":       "/v1/observations",
    "fhirBase":           "/fhir/r5/",
    "patientBindings":    "/v1/patient-bindings",
    "softwareUpdates":    "/v1/software-updates",
    "biocompatibility":   "/v1/biocompat-records",
    "custody":            "/v1/custody-events",
    "openapi":            "/v1/openapi.json",
    "wellKnown":          "/.well-known/wia/electronic-skin"
  }
}
```

## §3 Device Registration Endpoints

### §3.1 Register a device

```
POST /v1/devices
Content-Type: application/json
Signature: <RFC 9421 signature>
```

Request body carries the §3 record from PHASE-1.
The server validates that the declared `deviceClass`
is consistent with the declared `applicableParticular`
particular standards: a device declaring `IEC-
60601-2-47` (ambulatory ECG) MUST also declare a
`signalChannels` entry with the IEEE 11073-10101
ECG term code, otherwise the request returns
`422 Unprocessable Entity` at `/problems/iec60601-
particular-channel-mismatch` and the offending
field expressed as a JSON Pointer (RFC 6901).

### §3.2 Retrieve a device

```
GET /v1/devices/{deviceId}
Accept: application/json
```

### §3.3 Search devices

```
GET /v1/devices?class={class}&particular={iec}
&channelTermCode={ieee11073Code}
&qmsCertificateNumber={number}
&page={cursor}&size={size}
```

## §4 Observation Upload Endpoints

### §4.1 Upload a sample block

```
POST /v1/observations
Content-Type: multipart/form-data; boundary=...
Signature: <RFC 9421 signature from the device's
            attestation key>
```

The multipart body carries one JSON part with the
§4 record from PHASE-1 (deviceRef, patientRef,
channelRef, sampleStartTime, sampleRateHz,
sampleResolution, qualityFlags) and one or more
file parts holding the encoded sample block. The
server stores the sample block under a content-
addressable URI (the SHA-256 hex digest is the
path segment).

### §4.2 Retrieve an observation

```
GET /v1/observations/{observationId}
Accept: application/json
```

### §4.3 Stream observations to an analytics
       service

```
GET /v1/devices/{deviceId}/observations:stream
Accept: application/x-ndjson
```

The endpoint streams observation envelopes in
sample-time order; a consumer resuming the stream
provides an `If-Resume-After` header carrying the
last-received sample-start timestamp.

## §5 FHIR Observation Endpoints

### §5.1 Read FHIR Observation

```
GET /fhir/r5/Observation/{observationId}
Accept: application/fhir+json
```

The FHIR endpoint mirrors the §5 mapping from
PHASE-1. The response is signed using the
operator's FHIR-server signing key and carries
the IEEE 11073-10101 cross-reference in the
`code.coding` array.

### §5.2 Search FHIR Observations

```
GET /fhir/r5/Observation
?subject=Patient/{patientId}
&device=Device/{deviceRef}
&category=vital-signs
&date=geXXXX-XX-XXTXX:XX:XXZ
&_count=N
```

## §6 Biocompatibility Endpoints

### §6.1 Upload a biocompatibility test

```
POST /v1/biocompat-records
Content-Type: application/json
Signature: <RFC 9421 signature>
```

Request body carries the per-test ISO 10993
reference (ISO 10993-5 cytotoxicity, ISO 10993-10
or ISO 10993-23 irritation), the test laboratory
identifier, the laboratory's ISO/IEC 17025
accreditation reference, the test result, and the
date of issue. The server links the test record
to the `deviceRef` and verifies that the test
laboratory's accreditation scope covers the
declared test method.

### §6.2 Retrieve a biocompatibility record

```
GET /v1/biocompat-records/{recordId}
Accept: application/json
```

## §7 Software-Update Endpoints

### §7.1 Publish a software update

```
POST /v1/software-updates
Content-Type: application/json
Signature: <RFC 9421 signature>
```

Request body carries the §7 record from PHASE-1.
The server validates that the IEC 62304 software
class is consistent with the declared release
level: a software update marked as `security-
only` for an IEC 62304 class C device requires
the `signedManifest` to reference a regulator-
notified field-safety corrective-action record
where the security defect was reported under the
EU MDR Article 89 vigilance reporting.

### §7.2 Retrieve a software update

```
GET /v1/software-updates/{updateId}
Accept: application/json
```

### §7.3 Device-side software-update poll

```
GET /v1/devices/{deviceId}/software-updates:next
Accept: application/json
```

The device polls for the next applicable update.
The response carries the update's signed
manifest, the rollback policy, and the post-
deployment surveillance reporting cadence.

## §8 Patient Binding and Consent Endpoints

### §8.1 Bind a patient

```
POST /v1/patient-bindings
Content-Type: application/json
Signature: <RFC 9421 signature>
```

Request body carries the §6 record from PHASE-1
(pseudonymousId, consentDirective). The consent
directive references the FHIR Consent resource at
`/fhir/r5/Consent/{consentId}` and is required
before any observation upload referencing the
patient is accepted.

### §8.2 Withdraw consent

```
DELETE /v1/patient-bindings/{patientId}/consent
Signature: <RFC 9421 signature from the patient
            or an authorised representative>
```

The operator's API records the withdrawal as a
chain-of-custody event and ceases acceptance of
new observations referencing the patient. The
withdrawal does not retroactively delete already-
uploaded observations; deletion follows the
patient's GDPR Article 17 erasure or HIPAA 45
CFR 164.522 access-restriction request through
the operator's data-protection workflow.

## §9 Custody and Error Reporting

### §9.1 Anchor a custody event

```
POST /v1/custody-events
Content-Type: application/json
Signature: <RFC 9421 signature>
```

### §9.2 Error envelope

Errors are returned using RFC 9457 Problem
Details. The problem-type identifiers are stable
strings rooted at `/problems/`. Validation errors
carry a `pointer` field whose value is a JSON
Pointer (RFC 6901) into the offending request
body. The server emits a per-request
`traceparent` header (W3C Trace Context).

## §10 Concurrency and Cache

Every retrieval endpoint emits an `ETag` header
(RFC 9110 §8.8.3) computed over the canonical
representation of the resource. Conditional
requests (`If-Match`, `If-None-Match`) are
honoured on update endpoints so that a concurrent
update does not overwrite a peer's update. The
server returns `412 Precondition Failed` where
the conditional request does not match,
together with an RFC 9457 problem document
carrying the current ETag.

## §11 Bulk Export for Clinical Research

### §11.1 Bulk export

```
POST /fhir/r5/$export
Accept: application/fhir+ndjson
```

Where the operator participates in a clinical
investigation under the EU MDR Annex XV or the
US FDA IDE pathway, the operator publishes the
de-identified observation set as a FHIR Bulk
Data export following the HL7 FHIR R5 Bulk Data
Access specification. The export is gated on
the institutional review board's approval and
on the patient's consent flag at de-identified-
research scope.

## §12 Device-Side Telemetry over IEEE 802.15.6

A patch communicating with a relay over IEEE
802.15.6:2012 BAN binds the BAN session to the
operator's API session at the relay. The relay
publishes a `ble-bridge` endpoint at
`/v1/devices/{deviceId}/ble-bridge` carrying the
BAN session identifier, the relay's certificate,
and the per-session encryption parameters so
that an external auditor can correlate the BAN
session with the API-side session for
traceability.

## §13 Risk-Management File Linking

```
GET /v1/devices/{deviceId}/risk-management-file
Accept: application/json
Signature: <RFC 9421 signature from a notified
            body or auditor>
```

A notified body or auditor reviewing the device's
ISO 14971 risk-management file requests the
file through the dedicated endpoint. The endpoint
returns the risk register summary, the per-
hazard risk-control reference, the residual-risk
evaluation, and the date of the most recent
post-market surveillance feedback loop.
Authentication is gated on the requester's
notified-body identifier or the operator's audit
agreement reference.

## §14 Conformance Test Vector Endpoint

```
GET /v1/conformance/test-vectors
Accept: application/json
```

The operator publishes the conformance test
vectors used to qualify the API implementation
against this specification. Each vector carries
the request, the expected response, the IEC
62304 traceability link to the software unit
under test, and the per-vector approval
reference signed by the operator's QMS reviewer
under ISO 13485 §7.3.6 design verification.
