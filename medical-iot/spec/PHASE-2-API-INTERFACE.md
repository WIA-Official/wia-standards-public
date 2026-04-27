# WIA-medical-iot PHASE 2 — API Interface Specification

**Standard:** WIA-medical-iot
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable

This PHASE defines the HTTP API surface a medical-IoT boundary
exposes to gateways, clinical-workstation EHRs, biomedical-engineering
operations consoles, regulator audit clients, and the device
fleet itself. The shape is FHIR R5 RESTful, layered with this
standard's IEEE 11073 nomenclature, IEC 60601-1-8 alarm semantics,
and IEC 80001-1 risk-management metadata.

References (CITATION-POLICY ALLOW only):
- HL7 FHIR R5 — RESTful API (R5/http.html), Bundle, OperationOutcome,
  CapabilityStatement
- HL7 SMART App Launch 2.2 (smarthealthit.org/specification) — for
  clinical-app authentication and scope strings
- IEEE 11073-20601:2019 — application profile message exchange
- IEEE 11073-10206:2021 — abstract content information model for
  REST-style resource access
- IEC 80001-1:2021 — IT-network risk management
- IETF RFC 9457 (Problem Details), RFC 8615 (well-known URIs),
  RFC 7515 (JWS), RFC 7519 (JWT), RFC 8259 (JSON), RFC 3339
- IHE PCD-01 / PCD-04 transactions (HL7 v2 + FHIR mappings)

---

## §1 Capability discovery

The boundary publishes a FHIR CapabilityStatement at
`/metadata`, augmented with WIA extensions:

```
GET /metadata HTTP/1.1
Accept: application/fhir+json
```

The response declares supported resources (Device, DeviceMetric,
Observation, AlarmCondition extension, DeviceAssociation), the
IEEE 11073-10101 metric codes accepted, the alarm-priority
mapping, the cipher-suite floor (cross-reference to
WIA-network-security), and the migration phase (cross-reference
to WIA-pq-crypto). Clients SHOULD cache the capability statement
for the session lifetime.

A second discovery endpoint advertises the deployment's IEC 80001-1
risk-network properties:

```
GET /.well-known/wia/medical-iot HTTP/1.1
```

Returns:

- supported `connectivityKind` values
- declared `riskClass` (per IEC 80001-1 Annex)
- maintenance-window schedule
- offline-tolerance window
- alarm-condition escalation policy reference

## §2 Device registration

A new device is admitted to the deployment via:

```
POST /Device HTTP/1.1
Authorization: Bearer <jws-bme-jwt>
Content-Type: application/fhir+json
```

Body is a FHIR Device resource conforming to PHASE 1 §2. The
boundary verifies UDI integrity (FDA GUDID lookup or EU EUDAMED
lookup or manufacturer-attestation chain), assigns the
`deviceRef` URN, and emits an AuditEvent with `agent` =
biomedical-engineer principal.

Device updates use `PUT /Device/<id>`; the prior version is
preserved in version history because regulatory audit may
require reconstruction of the device's configuration at any
past timestamp.

Devices are retired (`status: inactive`) rather than deleted;
historical observations remain bound to the retired Device.

## §3 Observation streaming

High-frequency observations (ECG, plethysmography, capnography)
flow on a streaming endpoint:

```
POST /Observation/$ingest HTTP/1.1
Content-Type: application/json+ndjson
WIA-Device-Ref: urn:wia:miot:device:M-PULSE-1.2.3:SN-91A7
WIA-Stream-Id: s-91a7-2026-04-28
WIA-Sequence-Start: 0
```

The body is NDJSON of Observation resources; sequence numbers
are monotonically increasing per stream. The boundary acknowledges
the highest received sequence number; on reconnect the client
resumes from `lastAckedSeq + 1`. Out-of-order frames within a
configurable reordering window are buffered; outside the window
they are recorded with `interpretation: "out-of-order"`.

Lower-frequency observations (vital-sign episodics, glucose
readings, weight) use the standard FHIR endpoint:

```
POST /Observation HTTP/1.1
```

## §4 Alarm conditions

Alarm endpoints use the WIA AlarmCondition extension on top of
FHIR's resource framework:

```
POST /AlarmCondition HTTP/1.1
PUT  /AlarmCondition/<id>/$acknowledge HTTP/1.1
PUT  /AlarmCondition/<id>/$resolve HTTP/1.1
GET  /AlarmCondition?subject=<patientRef>&status=active
```

The boundary applies IEC 60601-1-8 escalation cadence:
unacknowledged high-priority alarms escalate to a paired
clinical-care role within the priority's tau (typically 10 s
for high, 30 s for medium, 5 min for low). Escalation events
are recorded as additions to the alarm's `escalation[]` log,
not as separate alarms.

## §5 Calibration

```
POST /Device/<deviceRef>/$calibrate HTTP/1.1
Content-Type: application/json
```

Body is a calibration record per PHASE 1 §6. The boundary
verifies the calibration certificate's signature against the
biomedical-engineering key, updates the device's `nextDueAt`,
and emits an AuditEvent.

```
GET /Device/<deviceRef>/calibration?at=<RFC3339>
```

Returns the calibration record active at the supplied time,
or 404 if the device was uncalibrated at that time.

## §6 Patient association

```
POST /DeviceAssociation HTTP/1.1
PUT  /DeviceAssociation/<id>/$end HTTP/1.1
GET  /Patient/<subjectRef>/$active-devices
```

The `$active-devices` operation returns the bundle of
DeviceAssociation entries currently active for the patient,
annotated with the device's most recent observation timestamp
and any active alarm conditions.

## §7 Search filtering and minimum necessary

A search query returns *only* the subset consented for the
declared purpose:

```
GET /Observation?patient=<subjectRef>&code=150456&date=ge2026-04-01 HTTP/1.1
WIA-Purpose-Of-Use: TREAT
```

The boundary verifies the consent (cross-reference to
WIA-medical-data-privacy) before returning results. Records
filtered out by minimum-necessary do not appear in the response,
nor in the count; the response carries a `link` of
`relation: prohibited` whose `url` is the consent record explaining
the scope of redaction.

## §8 Error model (Problem Details)

All non-2xx responses use RFC 9457 Problem Details. Reserved
`type` URIs:

| URI                                                | Status | Meaning                                                  |
|----------------------------------------------------|-------:|----------------------------------------------------------|
| `urn:wia:miot:problem:udi-required`                | 422    | UDI absent or unparseable                                |
| `urn:wia:miot:problem:metric-code-out-of-range`    | 422    | 11073 code not in nomenclature or vendor-extension range |
| `urn:wia:miot:problem:calibration-overdue`         | 200    | accepted but flagged                                     |
| `urn:wia:miot:problem:alarm-priority-required`     | 422    | IEC 60601-1-8 priority absent                            |
| `urn:wia:miot:problem:device-offline`              | 503    | device has no active connectivity binding                |
| `urn:wia:miot:problem:no-active-consent`           | 403    | no consent matches the declared purpose                  |
| `urn:wia:miot:problem:out-of-order-window`         | 200    | out-of-order frame, recorded with annotation              |

Body example:

```json
{
  "type": "urn:wia:miot:problem:udi-required",
  "title": "UDI is required for device admission",
  "status": 422,
  "detail": "POST /Device received a payload without a parseable UDI carrier.",
  "instance": "/Device"
}
```

## §9 Bulk export

The FHIR R5 Bulk Data Access (Flat FHIR) operations are
supported with PHASE 1 §10 minimum-necessary discipline:

```
GET /$export?_type=Observation,Device,AlarmCondition&purpose=HRESCH&consentBundle=<id>
```

The export is allowed only if the consent bundle names every
subject in the cohort. The exported NDJSON is signed; the
manifest references a de-identification job record (per
WIA-medical-data-privacy PHASE 1 §7) so the recipient can
verify the residual-risk band of the dataset.

## §10 Subject self-service

A patient or their authorised proxy queries their own data:

- `GET /Patient/<self>/$device-summary` — devices currently
  associated and their most recent observation per metric
- `GET /Patient/<self>/AlarmCondition` — alarms for the patient
- `POST /Patient/<self>/$rectification` — correct an erroneous
  observation (subject to clinical-review workflow)

Self-service authentication uses the deployment's identity
provider; AuditEvents are emitted for self-service calls so
that impersonation patterns are detectable.

## §11 Versioning

The API version is announced in the CapabilityStatement
(`software.version`). Breaking changes require a new path
prefix (`/v2/...`) with at least 90 days of parallel operation.

## §12 Idempotency

Mutating endpoints accept `Idempotency-Key`. The boundary
stores keys for 30 days. Replays return the original response;
conflicts return `urn:wia:miot:problem:idempotency-conflict`.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Worked alarm acknowledgement (informative)

```
PUT /AlarmCondition/a-91a7/$acknowledge HTTP/1.1
Authorization: Bearer <rn-jwt>
Content-Type: application/json
WIA-Purpose-Of-Use: TREAT

{
  "acknowledgedBy": "urn:wia:hr:rn:nurse-stn-3-floor-7-rn-1",
  "acknowledgedAt": "2026-04-28T03:14:09+09:00",
  "note": "Bedside; Spo2 transient drop on patient turning. No intervention."
}
```

```
200 OK
Content-Type: application/fhir+json

{
  "resourceType": "AlarmCondition",
  "id": "a-91a7",
  "status": "acknowledged",
  "alarmKind": "high",
  "alarmCategory": "physiological",
  "triggeringObservationRef": "urn:wia:miot:obs:o-77c2",
  ...
}
```

## Annex B — Capability statement excerpt (informative)

```json
{
  "resourceType": "CapabilityStatement",
  "status": "active",
  "fhirVersion": "5.0.0",
  "format": ["application/fhir+json"],
  "rest": [{
    "mode": "server",
    "resource": [
      {"type": "Device", "interaction": [{"code": "read"},{"code": "search-type"},{"code": "create"},{"code": "update"}]},
      {"type": "Observation", "interaction": [{"code": "read"},{"code": "search-type"},{"code": "create"}]},
      {"type": "DeviceAssociation", "interaction": [{"code": "read"},{"code": "search-type"},{"code": "create"},{"code": "update"}]}
    ]
  }],
  "extension": [{
    "url": "https://wia.example/fhir/StructureDefinition/wia-medical-iot-capabilities",
    "extension": [
      {"url": "metricCodeSystem", "valueUri": "urn:iso:std:iso:11073:10101"},
      {"url": "alarmStandard", "valueUri": "urn:iec:std:iec:60601-1-8"},
      {"url": "riskClass", "valueString": "IEC 80001-1 Class C"}
    ]
  }]
}
```

## Annex C — Cross-domain reference resolution (informative)

```
GET /Observation/o-77c2 HTTP/1.1
Authorization: Bearer ...
WIA-Purpose-Of-Use: TREAT
Accept: application/fhir+json
```

Boundary verifies the consent at WIA-medical-data-privacy
(`/Consent?subject=<subjectRef>&status=active`) before
returning. If the consent reference resolves but its
purpose binding does not include `TREAT`, the response is:

```
403 Forbidden
Content-Type: application/problem+json

{
  "type": "urn:wia:miot:problem:no-active-consent",
  "title": "No consent matches purpose TREAT for this subject",
  "status": 403,
  "detail": "Subject's active consent is scoped HRESCH only.",
  "instance": "/Observation/o-77c2"
}
```

The 403 carries the cross-domain trace so clinicians can
escalate to the medical-data-privacy boundary for a renewed
consent rather than re-attempting the same call.

## Annex D — Subscription delivery contract (informative)

When an EHR subscribes to alarm conditions for a patient
cohort, the boundary delivers a notification bundle:

```json
{
  "resourceType": "Bundle",
  "type": "history",
  "entry": [
    {"resource": {"resourceType": "AlarmCondition", "id": "a-91a7", "status": "active", "alarmKind": "high", ...}},
    {"resource": {"resourceType": "Observation", "id": "o-77c2", "status": "final", ...}}
  ]
}
```

Delivery is signed (`Wia-Signature` header carrying detached
JWS). The EHR responds 200 OK on successful processing or
4xx with retry guidance; the boundary retries per the
deployment's exponential-backoff schedule, capped at 24
hours. After 24 hours of failed delivery the subscription is
suspended and operations are alerted.
