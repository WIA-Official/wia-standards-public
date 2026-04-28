# WIA-education-robot PHASE 2 — API Interface Specification

**Standard:** WIA-education-robot
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable

This document defines the HTTPS API contract that
an education-robot operator (manufacturer,
school deployer, after-school academy, inclusive-
education provider, telepresence-robot operator,
museum-based deployer, certification body)
exposes for the records defined in PHASE-1. The
contract carries the robot-platform registration,
ontology binding, safety-conformity test upload,
interaction-event recording, lifecycle event
recording, and chain-of-custody anchoring
endpoints.

References (CITATION-POLICY ALLOW only):

- IETF RFC 9110, RFC 9111, RFC 9457, RFC 8288,
  RFC 6901 / 6902, RFC 8259, RFC 4122, RFC
  9421, RFC 8615
- W3C Trace Context, W3C WebRTC 1.0
- ISO/IEC 27001:2022, ISO/IEC 17021-1:2015,
  ISO/IEC 17065:2012, ISO/IEC 17025:2017
- ISO 13482:2014, ISO 10218-1:2011 / -2:2011,
  ISO/TS 15066:2016
- IEC 62366-1:2015/Amd 1:2020, IEC 61508 series
- IEEE Std 1872-2015 (CORA), IEEE Std 1872.2-
  2021 (AuR)
- IEC 62443-3-3, IEC 62368-1:2018+AMD1:2020
- ISO/IEC 23894:2023, ISO/IEC 42001:2023
- EU Machinery Regulation (EU) 2023/1230
- EU AI Act (Regulation (EU) 2024/1689)
- KR 어린이제품 안전 특별법

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published
by the operator. The OpenAPI 3.1 document at
`/v1/openapi.json` is canonical. Schema changes
follow the non-breaking conventions in PHASE-1
§2. Every endpoint carries a per-request
signature using HTTP Message Signatures (RFC
9421) anchored to the operator's accreditation
reference (the ISO/IEC 17025 testing-laboratory
accreditation, the ISO/IEC 17065 product-
certification accreditation, the EU notified-
body designation under the EU Machinery
Regulation); the signature key set is published
at `/.well-known/wia/education-robot/keys.json`.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-education-robot",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "programmes":          "/v1/programmes",
    "robots":              "/v1/robots",
    "ontologyRecords":     "/v1/ontology-records",
    "safetyRecords":       "/v1/safety-records",
    "interactionEvents":   "/v1/interaction-events",
    "lifecycleRecords":    "/v1/lifecycle-records",
    "custody":             "/v1/custody-events",
    "openapi":             "/v1/openapi.json",
    "wellKnown":           "/.well-known/wia/education-robot"
  }
}
```

## §3 Robot Platform Endpoints

### §3.1 Register a robot platform

```
POST /v1/robots
Content-Type: application/json
Signature: <RFC 9421 signature>
```

Request body carries the §3 record from
PHASE-1 (robotKind, iso13482Class, payload,
workspaceClass, intendedAgeBand,
intendedSupervision). The server validates the
declared `iso13482Class` against the declared
`robotKind`: a `programmable-block-robot` or a
`drone-educational` declared with an
`iso13482Class` of `physical-assistant` is
rejected with `422 Unprocessable Entity`
carrying an RFC 9457 problem document at
`/problems/iso13482-class-robot-kind-mismatch`,
since ISO 13482 personal-care-robot
classification does not apply to those robot
kinds.

### §3.2 Retrieve a robot platform

```
GET /v1/robots/{robotId}
Accept: application/json
```

### §3.3 Search robots

```
GET /v1/robots?kind={kind}
&iso13482Class={class}
&workspaceClass={workspace}
&intendedAgeBand={age}
&page={cursor}&size={size}
```

## §4 Ontology Endpoints

### §4.1 Bind a robot to IEEE Std 1872 ontology

```
POST /v1/ontology-records
Content-Type: application/json
Signature: <RFC 9421 signature>
```

Request body carries the §4 record from
PHASE-1. The server validates that the
`coraClassBinding` references the IEEE Std
1872-2015 CORA classes (Robot, RobotPart,
Sensor, Effector, Environment, Position,
Orientation) and that the `aurBehaviour`
references the IEEE Std 1872.2-2021 AuR
ontology classes.

### §4.2 Retrieve an ontology record

```
GET /v1/ontology-records/{ontologyRecordId}
Accept: application/json
```

## §5 Safety-Conformity Endpoints

### §5.1 Upload a safety-conformity test

```
POST /v1/safety-records
Content-Type: multipart/form-data; boundary=...
Signature: <RFC 9421 signature from a testing
            laboratory's ISO/IEC 17025
            certificate>
```

The multipart body carries one JSON part with
the §5 record from PHASE-1 (testStandard,
testLaboratory, measurementResult, passOrFail)
and one or more file parts holding the force-
and-pressure measurement raw data, the
separation-distance trace, and the supporting
calibration record. The server stores the raw-
data files under a content-addressable URI.

The server enforces the test-method-to-
measurement-result mapping table:

- An `ISO-TS-15066` (collaborative-robot)
  record MUST carry the per-body-region quasi-
  static and transient pressure-and-force
  values measured against the ISO/TS 15066
  Annex A reference values.
- An `ISO-13482` record MUST carry the per-
  scenario hazard-and-risk assessment per the
  standard's risk-assessment table.
- An `IEC-61508-SIL-2` or `IEC-61508-SIL-3`
  record MUST carry the per-channel safety
  integrity level evidence.

### §5.2 Retrieve a safety-conformity record

```
GET /v1/safety-records/{safetyRecordId}
Accept: application/json
```

## §6 Interaction-Event Endpoints

### §6.1 Record an interaction event

```
POST /v1/interaction-events
Content-Type: application/json
Signature: <RFC 9421 signature>
```

Request body carries the §6 record from
PHASE-1. The server enforces that:

- A learner who is a minor MUST be bound to a
  consent envelope from the parent or
  guardian.
- An inclusive-education session MUST be bound
  to a supervising-therapist identifier.
- A telepresence session MUST be bound to a
  W3C WebRTC signalling-server reference and
  to the operator's privacy declaration.

### §6.2 Retrieve an interaction event

```
GET /v1/interaction-events/{eventId}
Accept: application/json
Authorization: <bearer token from the
                 supervising teacher, the
                 therapist, the parent or
                 guardian, or an audit
                 observer>
```

The response carries the interaction-event
envelope. The learner's directly-identifying
record is redacted unless the requesting
principal has the per-learner authorisation.

## §7 Lifecycle and Recall Endpoints

### §7.1 Record a lifecycle event

```
POST /v1/lifecycle-records
Content-Type: application/json
Signature: <RFC 9421 signature>
```

### §7.2 Issue a recall

```
POST /v1/robots/{robotId}/recall
Content-Type: application/json
Signature: <RFC 9421 signature from the
            robot manufacturer>
```

The recall envelope carries the recall-
authority reference, the per-batch identifier
range, the recall-trigger event description,
and the operator's remediation plan. The
server propagates the recall to all subscribed
deployer endpoints via webhook.

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

Every retrieval endpoint emits an `ETag`
header (RFC 9110 §8.8.3). Conditional
requests are honoured.

## §10 Bulk Export for Audit Bodies

```
GET /v1/robots:bulk
Accept: application/x-ndjson
Authorization: <bearer token from the
                 certification body or a
                 supervisory regulator>
```

A regulator running a portfolio-level audit
or a certification body running a surveillance
audit requests the operator's robot register
as a newline-delimited JSON stream.

## §11 Webhook Endpoint for Recall Notifications

```
POST /v1/programmes/{programmeId}/webhooks
Content-Type: application/json
Signature: <RFC 9421 signature>
```

A school deployer, an after-school academy, an
inclusive-education provider registers a
webhook to receive a push notification when a
recall is issued, an incident is reported, or
a firmware update is published.

## §12 Schema-Validation and Conformance

The OpenAPI 3.1 document at
`/v1/openapi.json` carries JSON Schema 2020-12
schemas for every request and response
envelope.

```
GET /v1/conformance/test-vectors
Accept: application/json
```

The operator publishes the conformance test
vectors used to qualify the API implementation
against this specification.

## §13 Telepresence-Specific Endpoints

```
POST /v1/robots/{robotId}/telepresence-session
Content-Type: application/json
Signature: <RFC 9421 signature>
```

A telepresence robot's per-session signalling
envelope is published per the W3C WebRTC 1.0
specification. The operator's API surfaces the
signalling-server reference, the per-session
ICE-and-TURN configuration, and the operator's
privacy declaration.

## §14 Multi-Language Surface

```
GET /v1/robots/{robotId}?lang={lang}
Accept: application/json
```

The robot's product-of-record envelope is
published in each operator-declared language
(English, Korean, Japanese, German, French,
Spanish, Arabic) so that a downstream consumer
in a different jurisdiction can interpret the
declaration.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## §15 Bulk Operations and Quotas

```
GET /v1/safety-records:bulk
Accept: application/x-ndjson
```

A national accreditation authority running a
batch audit requests the operator's safety-
record register as a newline-delimited JSON
stream. The endpoint streams the safety
records in test-date order; a consumer
resuming the stream provides an
`If-Resume-After` header carrying the last-
received test timestamp. Per-token rate
limits default to 60 record-intake requests
per minute for testing-laboratory accounts;
bulk export uses a separate quota tuned to
the per-deployment audit window.

## §16 Inclusive-Education Clinical Endpoint

```
POST /v1/inclusive-sessions
Content-Type: application/json
Signature: <RFC 9421 signature from a
            licensed clinical supervisor>
```

An inclusive-education session is bound to a
clinical-supervisor identifier, a per-session
clinical-outcome envelope, and the operator's
data-sharing agreement. The clinical-outcome
envelope is encoded per the operator's
clinical-record vocabulary so that the
supervisor's clinical-record system can ingest
the envelope.

## §17 Conformance Test Vectors

```
GET /v1/conformance/test-vectors
Accept: application/json
```

The operator publishes the conformance test
vectors used to qualify the API implementation
against this specification. Each vector
carries the request, the expected response,
the test-report identifier from PHASE-1 §5,
and the safety-test outcome that the test-of-
record example reproduces. A laboratory or
notified body running an interoperability
check uses the endpoint to confirm that the
operator's API reproduces the published
classification logic deterministically across
the ISO 13482 / ISO 10218 / ISO/TS 15066
enumeration set.

## §18 Authorities and Roles

| Role                    | Capabilities |
|-------------------------|------|
| `robot-manufacturer`    | Register robot, upload safety-record, issue recall |
| `school-deployer`       | Register deployment, record interaction event, subscribe webhook |
| `clinical-supervisor`   | Read per-session interaction event, publish clinical outcome |
| `parent-or-guardian`    | Issue per-feature consent, revoke consent |
| `notified-body`         | Issue CE Declaration of Conformity, run conformity assessment |
| `regulator`             | Read full operator state, audit per-platform safety record |

The operator's API enforces the per-role
authorisation policy; an unauthorised request
is rejected with `403 Forbidden` and an RFC
9457 problem document.
