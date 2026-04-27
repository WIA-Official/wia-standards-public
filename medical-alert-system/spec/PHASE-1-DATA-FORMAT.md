# WIA-medical-alert-system PHASE 1 — Data Format Specification

**Standard:** WIA-medical-alert-system
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable

This PHASE defines the canonical data format for medical
alert systems: personal emergency-response systems (PERS)
worn or kept by older adults and people with chronic
conditions, fall-detection devices, social alarms in
assisted living, in-hospital nurse-call and rapid-response
alarms, and the central monitoring services that respond
to alarm activations. The shape interoperates with FHIR R5
Communication, CommunicationRequest, and Encounter resources
and with EN 50134 social-alarm protocols where applicable.

References (CITATION-POLICY ALLOW only):
- HL7 FHIR R5 — Communication, CommunicationRequest,
  Encounter, Person, RelatedPerson, ServiceRequest
- IEC 60601-1-8:2020 — Alarm systems for medical electrical
  equipment
- IEC 80601-2-58:2019 — Particular requirements for nurse-call
  systems (where applicable; legacy installations follow this)
- EN 50134 series — Alarm systems for social alarms
  - EN 50134-1:2002 — System requirements
  - EN 50134-2:2017 — Trigger devices
  - EN 50134-3:2012 — Local unit and controller
  - EN 50134-5:2004 — Interconnections and communications
  - EN 50134-7:2017 — Application guidelines
- ISO 13485:2016 — QMS for medical devices
- ISO 14971:2019 — Risk management
- IEC 62304:2006/A1:2015 — Medical device software life cycle
- IEC 62366-1:2015/A1:2020 — Usability engineering
- IETF RFC 8259 (JSON), RFC 7515 (JWS), RFC 3339

---

## §1 Scope

This PHASE applies to systems that detect, report, escalate,
and resolve medical alarms across three primary deployment
contexts:

- **PERS / mobile alarm** — patient-worn or patient-carried
  device that initiates an alarm via button press, fall
  detection, or vital-sign threshold; reaches a central
  monitoring station or family contacts
- **Social alarm / assisted-living** — fixed-installation
  systems per EN 50134 in care homes, sheltered housing, and
  multi-tenant elderly-services buildings
- **In-hospital nurse-call and rapid-response** — fixed-
  installation alarms in clinical settings; coordinates with
  IEC 80601-2-58 nurse-call systems and the deployment's
  rapid-response team workflow

In scope: alarm activation, classification, routing,
acknowledgement, resolution, and audit. Out of scope: cardiac-
device implant alarms (covered by a sibling implant-cardiac
standard), vital-sign-monitor alarms during procedures
(covered by WIA-medical-iot).

## §2 Alert device identity

Each alert device or installation is identified:

- `deviceRef` — URN of form `urn:wia:malert:device:<udi-di>:<udi-pi>`
  for medical-class devices (PERS pendants are typically
  Class II medical devices)
- `installationRef` — URN of form `urn:wia:malert:install:<facility>:<unit>`
  for fixed installations (social alarm systems, nurse-call)
- `kindRef` — URN denoting alert-device kind:
  - `urn:wia:malert:kind:pers-pendant` — patient-worn pendant
  - `urn:wia:malert:kind:pers-watch` — wristworn alert device
  - `urn:wia:malert:kind:fall-detector` — automated fall detector
  - `urn:wia:malert:kind:social-alarm-base` — EN 50134 base unit
  - `urn:wia:malert:kind:nursecall-bedside` — IEC 80601-2-58 bedside
  - `urn:wia:malert:kind:nursecall-bath` — bathroom pull-cord
  - `urn:wia:malert:kind:rapid-response-button` — clinical area RRT button
- `installationContext` — `home`, `assisted-living`, `nursing-home`,
  `acute-care`, `outpatient-clinic`

A record without a recognised device or installation reference
is rejected at the edge.

## §3 Alarm event record

Every alarm activation captures:

- `alarmEventId` — URN of form `urn:wia:malert:event:<deviceOrInstall>:<seq>`
- `subjectRef` — pseudonymous person under care (cross-domain
  to medical-data-privacy); for shared devices the subject
  may be unknown until correlation with a beacon, room
  occupancy, or post-resolution attribution
- `triggerKind` — closed enum: `manual-button`, `pull-cord`,
  `fall-detected`, `vital-threshold-exceeded`, `inactivity-detected`,
  `bed-exit`, `door-monitor`, `caregiver-initiated`,
  `system-self-test`, `tamper-detected`
- `priority` — IEC 60601-1-8 priority for clinical contexts:
  `high`, `medium`, `low`; for social alarm contexts: `urgent`,
  `routine`
- `category` — `physiological`, `technical`, `welfare-check`
- `assertedAt` — RFC 3339 with offset
- `location` — best-known location at time of trigger:
  - device-reported coordinates (if PERS device has GNSS)
  - installation-room URN (for fixed installations)
  - last-known-zone (for indoor positioning systems)
  - "unknown" if no localisation data
- `vitalsAttached[]` — IEEE 11073-10101 metrics carried with
  the alarm if the device is also a vital-sign monitor
- `falseAlarmIndicator` — `unknown` initially; updated post-
  resolution

Alarm events are append-only; subsequent state transitions
form additional records that reference the original event.

## §4 Routing and escalation

Each alarm event carries its routing record:

- `routingId` — URN
- `alarmEventRef` — parent alarm
- `routingPolicyRef` — URN of the deployment's routing policy
  for this device kind and installation context
- `attempts[]` — ordered list of routing attempts:
  - `at` — RFC 3339
  - `target` — URN of routed party (caregiver, monitoring-
    station operator, on-call clinician, family contact)
  - `channel` — `voice-call`, `sms`, `app-push`, `pager`,
    `console-banner`
  - `outcome` — `acknowledged`, `voicemail`, `no-answer`,
    `declined`, `escalated-out`
- `escalationLevel` — current level (0 = first contact)

Escalation cadence respects IEC 60601-1-8 (for clinical
contexts) and EN 50134-7 (for social-alarm contexts). The
boundary publishes the deployment's documented cadence.

## §5 Acknowledgement and resolution

Every alarm transitions through:

- `acknowledged` — first responder confirms receipt
- `triaged` — clinical evaluation determined disposition
- `dispatched` — emergency services or in-person caregiver
  dispatched
- `on-scene` — responder physically present
- `resolved` — situation closed
- `false-alarm` — classified retrospectively as false-alarm
- `transferred-to-care` — handed off to receiving care
  context (e.g., admitted to hospital from PERS activation)

Each transition emits a record:

- `transitionId` — URN
- `alarmEventRef` — parent alarm
- `priorState`, `nextState` — closed enum
- `at` — RFC 3339
- `actor` — principal URN responsible for the transition
- `note` — free-text clinical or operational context

## §6 Welfare-check record

Some deployments implement scheduled welfare checks (passive
inactivity monitoring, scheduled wellness check-ins). When a
welfare-check fires:

- `welfareCheckId` — URN
- `subjectRef` — person under check
- `triggerKind` — `inactivity`, `scheduled-check`, `bed-not-occupied`,
  `door-not-opened`
- `detectedAt`, `acknowledgedAt`, `resolvedAt` — RFC 3339
- `responseChannel[]` — channels used to verify welfare

Welfare-check escalation differs from emergency alarm
escalation; the routing policy distinguishes the two.

## §7 False-alarm reporting

Post-resolution, the responsible operator classifies the
event:

- `falseAlarmKind` — closed enum: `not-false-alarm`,
  `accidental-button`, `device-malfunction`, `caregiver-test`,
  `system-test`, `false-trigger-fall-detection`,
  `false-trigger-vital-monitor`

False-alarm rates per device kind feed quarterly compliance
reporting (PHASE 4) so deployments can tune sensitivity
without reducing essential performance.

## §8 Cross-domain references

| Reference                  | Use site                                                  |
|----------------------------|-----------------------------------------------------------|
| WIA-medical-data-privacy   | every alarm references the consent scope for caregivers   |
| WIA-medical-iot            | vital-sign metrics carried with alarms                    |
| WIA-medical-data-privacy   | break-glass override for emergency disclosure to first responders |
| WIA-network-security       | TLS cipher-suite floor for monitoring-station links       |
| WIA-pq-crypto              | post-quantum migration phase                              |

## §9 Subject identifier scope

Subject identifiers are pseudonymous per WIA-medical-data-privacy
`subjectRef` shape; the medical-alert-system boundary may resolve
to a direct identifier only at the moment of emergency dispatch
under an explicit break-glass override per WIA-medical-data-privacy
PHASE 1 §6.

## §10 Conformance levels

| Level     | Scope                                                                  |
|-----------|------------------------------------------------------------------------|
| Surface   | data formats accepted; self-attested                                   |
| Verified  | annual third-party audit + IEC 60601-1-8 / EN 50134 conformance review |
| Anchored  | continuous evidence package + IEC 80001-1 risk file when device-coupled |

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Worked PERS activation (informative)

```json
{
  "resourceType": "Communication",
  "status": "in-progress",
  "category": [{"coding": [{"system": "urn:wia:malert:category", "code": "physiological"}]}],
  "priority": "urgent",
  "subject": {"reference": "urn:wia:mdp:subject:f4c2-9bd1-7a05-3e8e"},
  "sender": {"reference": "urn:wia:malert:device:PERS-PDT-1.0:SN-91A7"},
  "received": "2026-04-28T09:42:11+09:00",
  "extension": [{
    "url": "https://wia.example/fhir/StructureDefinition/wia-medical-alert-event",
    "extension": [
      {"url": "triggerKind", "valueCode": "fall-detected"},
      {"url": "location", "valueString": "37.5665,126.9780 ±5m"},
      {"url": "vitalsAttached", "valueString": "MDC_PULS_RATE=98 bpm"}
    ]
  }]
}
```

## Annex B — Negative test vectors (informative)

| Stimulus                                             | Expected outcome                                |
|------------------------------------------------------|-------------------------------------------------|
| Alarm without trigger kind                           | 422 + `trigger-kind-required`                   |
| Routing attempt to disabled contact                  | recorded as failed; next attempt scheduled      |
| False-alarm without retrospective classification     | accepted; flagged in quarterly compliance report |
| Welfare-check without subject reference              | 422 + `subject-required`                        |
| Break-glass disclosure without active emergency      | 403 + `break-glass-not-authorised`              |

## Annex C — Routing policy record (informative)

The routing policy is a structured document referenced from
each alarm:

```json
{
  "routingPolicyId": "urn:wia:malert:routing-policy:facility-X:policy-2026",
  "appliesTo": {"installationContext": "assisted-living"},
  "escalationLevels": [
    {"level": 0, "channels": ["app-push","voice-call"], "targets": ["primary-monitoring-station"], "ackTimeoutSec": 60},
    {"level": 1, "channels": ["voice-call","sms"], "targets": ["secondary-monitoring-station","on-call-clinician"], "ackTimeoutSec": 90},
    {"level": 2, "channels": ["voice-call"], "targets": ["family-contact-1","family-contact-2"], "ackTimeoutSec": 120},
    {"level": 3, "channels": ["voice-call"], "targets": ["emergency-services"], "ackTimeoutSec": 0}
  ],
  "voiceCallProvider": "primary-sip-trunk",
  "voiceFailoverProvider": "backup-pstn-trunk",
  "publishedAt": "2026-04-01T00:00:00+09:00",
  "publishedBy": "urn:wia:hr:operations-director:od-1"
}
```

The policy is signed (JWS detached); rev-ed policies are
preserved in version history because alarm reconstruction
must reference the policy active at the time of the event.

## Annex D — Welfare-check schedule record (informative)

```json
{
  "welfareScheduleId": "urn:wia:malert:welfare-schedule:facility-X:s-2026",
  "appliesTo": {"installationContext": "assisted-living"},
  "checks": [
    {"kind": "morning-wakeup", "expectedWindow": {"start": "07:00:00", "end": "09:00:00"}, "trigger": "bed-not-occupied"},
    {"kind": "evening-medication", "expectedWindow": {"start": "20:00:00", "end": "21:00:00"}, "trigger": "scheduled-check"},
    {"kind": "night-presence", "expectedWindow": {"start": "23:00:00", "end": "06:00:00"}, "trigger": "inactivity"}
  ],
  "tolerance": {"sec": 1800}
}
```

Welfare-checks that fire outside the expected window
escalate per the deployment's documented escalation cadence.

## Annex E — Versioning

Versioning follows Semantic Versioning 2.0.0. Major bumps
require ≥ 90 days overlap on all fielded reference
implementations. Deprecated trigger-kind enumerations
remain valid for historical alarms during the sunset window.
