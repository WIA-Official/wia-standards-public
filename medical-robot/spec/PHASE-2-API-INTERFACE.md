# WIA-medical-robot PHASE 2 — API Interface Specification

**Standard:** WIA-medical-robot
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable

This PHASE defines the HTTP API surface a medical-robot boundary
exposes to OR-control consoles, surgeon workstations, biomedical-
engineering ops, regulator audit clients, manufacturers, and the
robot fleet itself. The surface is FHIR R5 RESTful, layered with
robot-specific resources and operations: procedure orchestration,
motion telemetry capture, intervention-event acknowledgement,
imaging cross-reference, and tele-link management.

References (CITATION-POLICY ALLOW only):
- HL7 FHIR R5 — RESTful API, Procedure, Device, ServiceRequest,
  Subscription, Bundle, OperationOutcome, CapabilityStatement
- HL7 SMART App Launch 2.2
- IEEE 11073-10101 — for vital-sign metric codes when robot
  monitors patient vitals during procedure
- IETF RFC 9457 (Problem Details), RFC 8615 (well-known URIs),
  RFC 7515 (JWS), RFC 7519 (JWT), RFC 8259 (JSON), RFC 3339
- IEC 80601-2-77 §201.4.4 (operator control)

---

## §1 Capability discovery

```
GET /metadata HTTP/1.1
Accept: application/fhir+json
```

Returns FHIR CapabilityStatement augmented with WIA extensions:
declared particular standards (IEC 80601-2-77, -78, ISO 13482),
supported robot kinds (PHASE 1 §2 catalogue), tele-surgery
support, motion-telemetry sampling rate.

```
GET /.well-known/wia/medical-robot HTTP/1.1
```

Returns robot-fleet summary: kinds in inventory, current
maintenance status, deployment policy URL, IEC 80001-1 risk
file reference.

## §2 Robot registration

```
POST /Robot HTTP/1.1
Authorization: Bearer <bme-jwt>
Content-Type: application/fhir+json
```

Body is a FHIR Device resource conforming to PHASE 1 §2 with
the WIA medical-robot extension. Boundary verifies UDI
integrity, particular-standard declaration, and end-effector
catalogue (each end effector is registered separately as a
sub-Device).

```
PUT /Robot/<id> HTTP/1.1
PUT /Robot/<id>/$retire HTTP/1.1
GET /Robot?kindRef=<kind>&siteRef=<site>
```

Retirement closes active associations and triggers a
data-egress check before the robot is reassigned.

## §3 Procedure orchestration

```
POST /Procedure HTTP/1.1
Content-Type: application/fhir+json
WIA-Purpose-Of-Use: TREAT
```

Body is a FHIR Procedure resource. Boundary verifies:

1. Active consent for `TREAT` purpose at WIA-medical-data-privacy
2. Robot is calibrated (PHASE 1 §7) and verified
3. Required end effectors are within use-life
4. Operator has surgical-credentials for the procedure code

```
PUT /Procedure/<id>/$start HTTP/1.1
PUT /Procedure/<id>/$pause HTTP/1.1
PUT /Procedure/<id>/$resume HTTP/1.1
PUT /Procedure/<id>/$complete HTTP/1.1
PUT /Procedure/<id>/$abort HTTP/1.1
```

The `$abort` operation accepts a structured reason and emits
a high-priority audit event; aborts do not delete telemetry
or intervention-event records, which remain for review.

## §4 Motion telemetry

For high-frequency telemetry, a streaming endpoint:

```
POST /Procedure/<procedureRef>/telemetry/$ingest HTTP/1.1
Content-Type: application/json+ndjson
WIA-Robot-Ref: urn:wia:mrobot:robot:M-LAP-1.2.3:SN-91A7
WIA-Stream-Id: t-91a7-2026-04-28
```

NDJSON of telemetry samples (PHASE 1 §4); sequence numbers are
monotonic per stream. The boundary acknowledges the highest
received sequence; on reconnect the client resumes.

Alternatively, the boundary supports an out-of-band time-
series upload at procedure completion:

```
POST /Procedure/<procedureRef>/telemetry/$archive HTTP/1.1
Content-Type: application/octet-stream
Content-Encoding: zstd
```

For procedures completed under tele-link loss where streaming
is impossible, this archive path captures the buffered
telemetry on reconnection.

## §5 Intervention events

```
POST /Procedure/<procedureRef>/InterventionEvent HTTP/1.1
GET  /Procedure/<procedureRef>/InterventionEvent?kind=force-limit-violation
```

Events are append-only; an erroneous event is corrected with
a follow-up event of `kind: correction` referencing the
original.

## §6 Alarm conditions

Same as WIA-medical-iot PHASE 2 §4, with the addition of
robot-specific alarm catalogue (PHASE 1 §6) and the
`procedureRef` cross-reference.

## §7 Tele-link management

For tele-surgery and remote-console scenarios:

```
POST /Procedure/<procedureRef>/$tele-link-establish HTTP/1.1
PUT  /Procedure/<procedureRef>/$tele-link-handover HTTP/1.1
PUT  /Procedure/<procedureRef>/$tele-link-end HTTP/1.1
```

Establishment requires both ends to authenticate; handover
authorises an alternative remote surgeon to take control on
a documented quorum. The link's quality (latency, packet
loss) is observed continuously and surfaced via:

```
GET /Procedure/<procedureRef>/$tele-link-status HTTP/1.1
```

Link-quality degradation past deployment-declared thresholds
triggers a `tele-link-loss` intervention event.

## §8 Imaging cross-reference

```
PUT /Procedure/<procedureRef>/imaging HTTP/1.1
Content-Type: application/json

{
  "imagingRefs": [
    {"studyInstanceUid": "1.2.840.113619.2...", "modality": "US", "acquisitionPeriod": {"start": "2026-04-28T03:14:00+09:00", "end": "2026-04-28T04:01:30+09:00"}}
  ]
}
```

Boundary verifies the studyInstanceUid resolves at the
WIA-medical-imaging boundary before binding.

## §9 Search filtering and minimum necessary

Same approach as WIA-medical-iot PHASE 2 §7. Search results
respect the patient's consent scope; the response carries a
`link` of `relation: prohibited` for filtered records to
preserve audit transparency without leaking content.

## §10 Error model

RFC 9457 Problem Details with reserved `type` URIs:

| URI                                                  | Status | Meaning                                   |
|------------------------------------------------------|-------:|-------------------------------------------|
| `urn:wia:mrobot:problem:udi-required`                | 422    | UDI absent on robot or end effector       |
| `urn:wia:mrobot:problem:end-effector-life-exceeded`  | 422    | End effector beyond use-life threshold    |
| `urn:wia:mrobot:problem:calibration-overdue`         | 403    | Robot calibration lapsed                  |
| `urn:wia:mrobot:problem:operator-not-credentialed`   | 403    | Operator lacks credentials for procedure  |
| `urn:wia:mrobot:problem:no-active-consent`           | 403    | No consent matches purpose                |
| `urn:wia:mrobot:problem:tele-link-quality-degraded`  | 200    | Accepted with telemetry annotation        |
| `urn:wia:mrobot:problem:procedure-not-startable`     | 422    | Pre-start checks failed                   |

## §11 Bulk export

The boundary supports FHIR Bulk Data Access for procedure,
intervention-event, and telemetry-summary resources:

```
GET /$export?_type=Procedure,InterventionEvent&purpose=HRESCH&consentBundle=<id>
```

Telemetry samples are exported as a downsampled summary by
default (one sample per second) to bound dataset size; the
high-frequency raw telemetry is available on a separate
endpoint with explicit research authorisation.

## §12 Subject self-service

Patients access their own procedure records:

```
GET /Patient/<self>/$procedure-summary HTTP/1.1
```

Returns:

- Procedures performed with procedure-code translation to
  patient-friendly description
- Each procedure's outcome and the team that performed it
- Active follow-up plans

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Worked procedure-start sequence (informative)

```
POST /Procedure HTTP/1.1
{
  "resourceType": "Procedure",
  "status": "preparation",
  "code": {"coding": [{"system": "http://snomed.info/sct", "code": "82918005", "display": "Laparoscopic cholecystectomy"}]},
  "subject": {"reference": "urn:wia:mdp:subject:f4c2-9bd1-7a05-3e8e"},
  "performer": [
    {"function": {"coding": [{"system": "http://terminology.hl7.org/CodeSystem/v3-ParticipationType", "code": "PRF"}]}, "actor": {"reference": "urn:wia:hr:surgeon:s-91a7"}},
    {"function": {"coding": [{"system": "http://terminology.hl7.org/CodeSystem/v3-ParticipationType", "code": "ASSIST"}]}, "actor": {"reference": "urn:wia:mrobot:robot:M-LAP-1.2.3:SN-91A7"}}
  ]
}
```

Boundary returns 201 Created with the assigned `procedureRef`;
operator then issues `$start` to begin telemetry streaming.

## Annex B — Tele-link quality observation (informative)

The boundary samples tele-link quality continuously; observations
are recorded as:

```json
{
  "linkQualityId": "urn:wia:mrobot:tlq:p-91a7:s-1024",
  "procedureRef": "urn:wia:mrobot:procedure:p-91a7",
  "at": "2026-04-28T03:30:15+09:00",
  "rttMs": 18,
  "packetLossPct": 0.03,
  "jitterMs": 2.1,
  "qualityBand": "good"
}
```

Quality bands: `good` (rtt ≤ 50 ms, loss ≤ 0.1%), `degraded`
(rtt 50–200 ms or loss 0.1–1%), `unusable` (rtt > 200 ms or
loss > 1%). Transitions to `unusable` trigger immediate alarm.

## Annex C — Subscription delivery for OR consoles (informative)

OR consoles register a subscription to the procedure's events
and alarms; the boundary delivers a Bundle on every event,
signed with detached JWS. Consoles acknowledge with 200 OK or
flag with 4xx for human review.

## Annex D — Tele-surgery handover sequence (informative)

A surgeon-handover from a primary remote surgeon to a secondary
during a tele-surgery procedure follows a documented sequence:

```
POST /Procedure/<procedureRef>/$tele-link-handover-request HTTP/1.1
Authorization: Bearer <primary-surgeon-jwt>
Content-Type: application/json

{
  "primarySurgeonRef": "urn:wia:hr:surgeon:s-91a7",
  "secondarySurgeonRef": "urn:wia:hr:surgeon:s-77c2",
  "reason": "primary-fatigue",
  "expectedDuration": "PT10M"
}
```

The boundary verifies:

1. The secondary surgeon holds equivalent credentials for
   the procedure code
2. The secondary surgeon's tele-link quality is currently
   in `good` band
3. The patient consent for the procedure does not exclude
   surgeon-substitution
4. The OR-side anesthesia team has acknowledged the handover

If all checks pass, the boundary issues a 200 OK with a
handover-token. Both surgeons countersign the handover-token
within 60 seconds; the boundary then issues the actual
control transfer at a documented checkpoint in the procedure
(typically a tool-disengagement instant). The handover and
both signatures are recorded in the audit chain.

If any check fails, the response carries a Problem Details
document explaining the refusal. Persistent refusal patterns
surface in the quarterly compliance report.

## Annex E — Operator-credential discovery (informative)

```
GET /Operator/<operatorRef>/$credentials HTTP/1.1
Accept: application/fhir+json
```

Returns the operator's currently-valid credentials and their
expiry; the caller (typically the procedure-scheduler) uses
this to verify eligibility before scheduling.

## Annex F — Pagination and rate limiting (informative)

List endpoints paginate at ≤ 200 procedures per page. Per-token
rate limits default to 10 procedure-start calls per minute
(operational sufficient bound) and 1000 telemetry-ingest calls
per minute per stream (matches highest sample-rate target).
Rate-limit refusals carry `urn:wia:mrobot:problem:rate-limited`.

## Annex G — Conformance disclosure

Sections §1, §2, §3, §4, §5, §6, §10 are mandatory. §7 (Tele-
link management) is mandatory for deployments performing
tele-surgery; non-tele deployments may exclude §7 with
documented reason. §8 (Imaging cross-reference) is mandatory
for procedures that use intraoperative imaging. §11 (Bulk
export) is mandatory where the deployment shares data with a
clinical-data warehouse. §12 (Subject self-service) is
mandatory where the deployment provides a patient-facing app
that exposes procedure records.

A deployment that excludes any mandatory section reports the
gap in its compliance package; deployments that exclude
optional sections record their reason in the deployment policy
referenced from the capability discovery endpoint.

## Annex H — Backwards-compatibility for v1 → v2 transitions

When the boundary moves from v1 to v2 of this PHASE, the v1
endpoints remain operational under a `/v1/` path prefix for
the deployment-declared overlap window. v1 procedures
in-flight at the cutover continue under v1 semantics until
they complete; new procedures use v2. The audit chain
records the version of each operation so reviewers can
distinguish v1 and v2 entries when reconstructing history.
