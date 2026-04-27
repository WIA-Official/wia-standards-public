# WIA-medical-alert-system PHASE 2 — API Interface Specification

**Standard:** WIA-medical-alert-system
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable

This PHASE defines the HTTP API surface a medical-alert-system
boundary exposes to PERS devices, social-alarm base units,
nurse-call panels, central monitoring stations, dispatched
emergency services, family-contact apps, regulator clients, and
care-management EHRs. The shape is FHIR R5 RESTful, layered
with alarm-specific operations for activation, routing,
escalation, acknowledgement, resolution, and false-alarm
classification.

References (CITATION-POLICY ALLOW only):
- HL7 FHIR R5 — RESTful API, Communication, CommunicationRequest,
  Subscription, Bundle, OperationOutcome, CapabilityStatement
- HL7 SMART App Launch 2.2
- IEC 60601-1-8:2020 — Alarm system requirements
- EN 50134-7:2017 — Application guidelines (escalation cadence)
- IETF RFC 9457 (Problem Details), RFC 8615 (well-known URIs),
  RFC 7515 (JWS), RFC 7519 (JWT), RFC 8259 (JSON), RFC 3339

---

## §1 Capability discovery

```
GET /metadata HTTP/1.1
Accept: application/fhir+json
```

FHIR CapabilityStatement augmented with WIA extensions:
declared particular standards (IEC 60601-1-8, EN 50134),
supported alert-device kinds, monitoring-station integration,
escalation-cadence policy.

```
GET /.well-known/wia/medical-alert-system HTTP/1.1
```

Returns deployment policy summary: routing policies,
monitoring-station roster, family-contact app integration,
risk-file reference.

## §2 Device and installation registration

```
POST /AlertDevice HTTP/1.1
POST /AlertInstallation HTTP/1.1
PUT  /AlertDevice/<id> HTTP/1.1
PUT  /AlertInstallation/<id> HTTP/1.1
```

Devices and fixed installations are admitted under separate
endpoints. The boundary verifies UDI for medical-class devices
and installation-specific certifications (EN 50134-3 for
social-alarm base units; IEC 80601-2-58 for nurse-call panels).

Retirement closes active assignments; for shared devices
(PERS pendants reassigned to a new patient), a sanitisation
record is required before re-deployment.

## §3 Alarm activation

```
POST /AlarmEvent HTTP/1.1
Content-Type: application/fhir+json
WIA-Device-Ref: urn:wia:malert:device:PERS-PDT-1.0:SN-91A7
```

Body is a FHIR Communication resource with the WIA medical-
alert-event extension (PHASE 1 §3 Annex A example). The
boundary:

1. Verifies the device or installation reference
2. Resolves the subject reference (or queues for resolution
   if subject is not yet identified for a shared device)
3. Applies the routing policy
4. Initiates the first routing attempt within deployment-
   declared latency

A successful POST returns 201 Created with the assigned
`alarmEventId` and the routing policy that will be applied.

## §4 Routing and escalation operations

```
POST /AlarmEvent/<id>/$route HTTP/1.1
PUT  /AlarmEvent/<id>/$escalate HTTP/1.1
GET  /AlarmEvent/<id>/$routing-status HTTP/1.1
```

`$route` triggers a routing attempt; the boundary executes
according to the routing policy. `$escalate` advances to the
next escalation level; this is typically system-driven on
timeout but may be operator-driven if intelligence suggests
the current level cannot respond.

## §5 Acknowledgement and resolution

```
PUT /AlarmEvent/<id>/$acknowledge HTTP/1.1
PUT /AlarmEvent/<id>/$triage HTTP/1.1
PUT /AlarmEvent/<id>/$dispatch HTTP/1.1
PUT /AlarmEvent/<id>/$on-scene HTTP/1.1
PUT /AlarmEvent/<id>/$resolve HTTP/1.1
PUT /AlarmEvent/<id>/$classify-false-alarm HTTP/1.1
PUT /AlarmEvent/<id>/$transfer-to-care HTTP/1.1
```

State-transition operations record each step with the
responsible actor and any clinical or operational note.

## §6 Welfare-check operations

```
POST /WelfareCheck HTTP/1.1
PUT  /WelfareCheck/<id>/$acknowledge HTTP/1.1
PUT  /WelfareCheck/<id>/$resolve HTTP/1.1
```

Welfare-check creation may be system-driven (inactivity
detection, scheduled wellness check-ins) or operator-driven.
The deployment policy declares the schedule and escalation
behaviour.

## §7 Search filtering

```
GET /AlarmEvent?subject=<subjectRef>&since=...&priority=urgent
```

Search results filter to consented subset. Privacy boundary
honours WIA-medical-data-privacy purpose-of-use; emergency
dispatch operators have an `EMERGENCY` purpose grant that
unlocks limited subject identifiers.

## §8 Error model

| URI                                                   | Status | Meaning                                        |
|-------------------------------------------------------|-------:|------------------------------------------------|
| `urn:wia:malert:problem:trigger-kind-required`        | 422    | Trigger kind absent on alarm event             |
| `urn:wia:malert:problem:device-not-registered`        | 404    | Device or installation reference unknown       |
| `urn:wia:malert:problem:subject-required`             | 422    | Welfare check missing subject reference        |
| `urn:wia:malert:problem:break-glass-not-authorised`   | 403    | Operator lacks break-glass authority           |
| `urn:wia:malert:problem:routing-policy-not-found`     | 422    | No routing policy for device kind / context    |
| `urn:wia:malert:problem:no-active-consent`            | 403    | No consent matches caregiver disclosure        |

## §9 Subscriptions

Monitoring stations subscribe to all alarm events for their
covered installations:

```
POST /Subscription HTTP/1.1
{
  "topic": "https://wia.example/SubscriptionTopic/medical-alert-activations",
  "channel": {"type": "rest-hook", "endpoint": "https://monitoring.example/wia-webhook"},
  "filterBy": [{"resourceType": "AlarmEvent", "filterParameter": "installationRef", "value": "<facility-set>"}]
}
```

Webhook delivery uses TLS 1.3 with detached JWS in
`Wia-Signature`. Failure to acknowledge within deployment-
declared retry window fails over to the secondary monitoring
station per the routing policy.

## §10 Family-contact integration

For PERS deployments where family contacts are part of the
escalation chain:

```
POST /Subject/<subjectRef>/FamilyContact HTTP/1.1
PUT  /Subject/<subjectRef>/FamilyContact/<id> HTTP/1.1
GET  /Subject/<subjectRef>/FamilyContact
```

Family contacts are notified per the subject's consent (which
may include limited disclosure: "an alarm has been triggered;
please call our monitoring station"). Specific clinical detail
is not disclosed without break-glass authority.

## §11 Bulk export

```
GET /$export?_type=AlarmEvent,Communication&purpose=HRESCH&consentBundle=<id>
```

For research and quality-improvement programmes, the boundary
supports bulk export with the same minimum-necessary discipline
as sibling standards.

## §12 Subject self-service

```
GET /Patient/<self>/AlarmEvent
GET /Patient/<self>/$alert-summary
```

The subject (or proxy) reviews their own activation history,
including false-alarm classifications. Self-service emits
audit events to detect impersonation patterns.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Worked monitoring-station handoff (informative)

```
1. PERS pendant pressed; device sends activation to boundary
2. Boundary creates AlarmEvent; first routing attempt to primary
   monitoring station
3. Monitoring station receives webhook; operator picks up
4. Operator calls subject; subject says "I fell, can't get up"
5. Operator $triage with note + $dispatch to local EMS
6. Operator $on-scene when EMS arrives
7. EMS resolves; subject transported to hospital
8. Operator $transfer-to-care with hospital reference
9. Hospital admission ticks the event closed
```

The audit chain captures every step.

## Annex B — Pagination and rate limiting (informative)

List endpoints paginate at ≤ 200 alarm events per page.
Per-token rate limits default to 100 alarm activations per
minute per device (much higher than expected; protects
against device malfunction). Rate-limit refusals carry
`urn:wia:malert:problem:rate-limited`.

## Annex C — Conformance disclosure

Sections §1, §2, §3, §4, §5 are mandatory. §6 (welfare-check)
is mandatory for assisted-living and home-care deployments.
§9 (subscriptions) is mandatory for any deployment with a
monitoring station. §10, §11, §12 are mandatory where the
corresponding flow is offered.

## Annex D — Worked routing-policy update (informative)

```
PUT /RoutingPolicy/<id> HTTP/1.1
Authorization: Bearer <operations-director-jwt>
Content-Type: application/json
If-Match: <etag>

{
  "escalationLevels": [
    {"level": 0, "channels": ["app-push"], "targets": ["primary-monitoring-station"], "ackTimeoutSec": 60},
    ...
  ]
}
```

The boundary verifies the policy author has policy-edit
authority and that the new policy meets EN 50134-7 cadence
requirements. Rejected updates carry a problem-detail
explaining the violation. Accepted updates create a new
policy version; the prior version remains valid for in-flight
alarms.

## Annex E — App-push integration (informative)

For deployments where the family-contact app is the primary
escalation channel:

- Boundary uses the platform's push notification service
  (FCM for Android, APNs for iOS) via the deployment's app
  identifier
- Push payload is opaque to the platform; clinical detail
  stays inside the app's encrypted storage
- Acknowledgement is signaled by the app sending an
  $acknowledge POST back to the boundary
- The boundary records the round-trip latency for SLA
  reporting

## Annex F — Pagination cursor format (informative)

List endpoints support cursor pagination. The cursor is
opaque, signed by the boundary, and valid for 30 minutes.
Cursors carry the snapshot epoch so a paginating client
sees a stable result-set even as new alarms occur.

## Annex G — Idempotency and replay handling

All write endpoints accept `Idempotency-Key`. The boundary
stores keys for 30 days. Replays return the original
response. Conflicts (different body, same key) return
`urn:wia:malert:problem:idempotency-conflict`.

For PERS devices in poor connectivity, replay is the norm
rather than the exception; a device that lost the
acknowledgement re-sends the activation. The Idempotency-Key
ensures the boundary creates the alarm event once.

## Annex H — Subject-to-installation mapping

For social-alarm and assisted-living deployments, the
boundary maintains a subject-to-installation mapping that
the routing policy uses to resolve activations. The
mapping is updated on:

- New resident admission
- Resident transfer between units
- Resident departure (discharge, hospital admission, end of life)

The mapping is referenced from each alarm event so
retrospective review can reconstruct which subject was
in which unit at the activation time.

## Annex I — Capability cache freshness

Clients (monitoring stations, EHR, family-contact apps)
cache capability statements. Capability documents include a
`fresh-until` timestamp; clients refresh on or before that
timestamp. Critical changes (routing-policy update,
escalation-cadence change) trigger a push notification to
subscribed clients so cache invalidates eagerly rather than
on next read.

## Annex J — Conformance level discovery

Clients query the boundary's declared conformance level via:

```
GET /.well-known/wia/medical-alert-system/conformance HTTP/1.1
```

Returns the deployment's declared level (Surface / Verified /
Anchored), the most recent audit date, and the auditor's
signature. Anchored deployments include the inclusion-proof
URI for the audit-chain witness.
