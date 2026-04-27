# WIA-medical-alert-system PHASE 3 — Protocol Specification

**Standard:** WIA-medical-alert-system
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable

This PHASE specifies the protocols binding data formats
(PHASE 1) and API surface (PHASE 2) to operational exchanges:
PERS device authentication and connectivity, social-alarm
controller protocols (per EN 50134-5), nurse-call wired
protocols (IEC 80601-2-58), monitoring-station handoff,
voice-call routing, escalation-cadence enforcement, time
discipline, and audit-chain construction.

References (CITATION-POLICY ALLOW only):
- IETF RFC 8446 (TLS 1.3), RFC 7525, RFC 9162 (CT pattern)
- IETF RFC 7252 (CoAP), RFC 8613 (OSCORE) — for constrained
  battery-powered PERS pendants
- IETF RFC 3261 (SIP) — for voice-call routing where the
  monitoring station integrates with PSTN/cellular voice
- IETF RFC 8829 (WebRTC) — for app-based audio between
  responders and subjects
- 3GPP TS 22.011 — Cellular emergency calling (where PERS
  uses cellular bearer)
- IEC 60601-1-8:2020, IEC 80601-2-58:2019
- EN 50134-5:2004 — Interconnections and communications
- EN 50134-7:2017 — Application guidelines

---

## §1 Authentication

PERS devices, social-alarm base units, nurse-call panels,
monitoring-station operators, family contacts, and EMS
liaisons authenticate via JWS-signed JWTs:

- `iss`, `sub`, `aud`, `iat`, `exp`
- `wia.role` — one of {`alert-device`, `alert-installation`,
  `monitoring-operator`, `clinician`, `family-contact`,
  `ems-liaison`, `auditor`}
- `wia.deviceRef` / `wia.installationRef` — bound device
  or installation
- `cnf` — confirmation claim binding to TLS client certificate

Monitoring-station operator tokens require two-factor
authentication on shift sign-on. Family-contact tokens are
limited to read access on specific subjects.

## §2 PERS device connectivity

PERS devices typically use one of:

- **Cellular bearer** — 3GPP cellular with TS 22.011
  emergency-calling capability; the device itself maintains
  a long-lived bearer for periodic heartbeat and instant
  activation
- **Wi-Fi + Cellular fallback** — home Wi-Fi for normal
  operation, cellular fallback when out-of-home
- **Bluetooth + smartphone gateway** — for wearable devices
  paired to a patient-owned smartphone

Heartbeat cadence: typically every 60 seconds for cellular-
only devices; lengthier for battery-conserving devices with
deployment-policy-declared limits. Missed heartbeats beyond
the policy threshold trigger a `device-offline` welfare-check.

## §3 Social-alarm protocol (EN 50134-5)

Fixed-installation social-alarm systems implement EN 50134-5
on the wired backplane (typically LAN-based for modern
installations, BS 8521 / DTMF for legacy):

- The base unit detects trigger-device activations
  (pull-cords, mat sensors, bed-exit sensors)
- The base unit transmits an alarm to the boundary via
  TLS 1.3 over LAN
- For legacy DTMF / Voice connect, the boundary supports
  a SIP-to-FHIR translator gateway

The boundary respects the EN 50134-7 escalation cadence
(typically 90s acknowledgement window before escalating to
the next contact level).

## §4 Nurse-call protocol (IEC 80601-2-58)

In-hospital nurse-call panels follow IEC 80601-2-58:

- Wired backplane between bedside / bathroom / room buttons
  and the nursing station
- Modern installations expose a network-side API (typically
  REST or HL7 v2 messages) translated to FHIR by the boundary
- Voice channel between bedside and nursing station follows
  the manufacturer's audio protocol; the boundary records
  the timestamp of voice-link establishment but does not
  record audio content

Rapid-response (Code Blue, Code Stroke) integrate with the
hospital's overhead-paging and rapid-response-team workflow;
the boundary emits CommunicationRequest resources that the
RRT app consumes.

## §5 Monitoring-station handoff

Monitoring-station handoff uses webhook delivery as the
canonical channel:

- TLS 1.3 with mTLS; monitoring station presents a partner
  certificate
- Webhook payload is a FHIR Bundle with the AlarmEvent
- Detached JWS in `Wia-Signature`
- Boundary expects 200 OK acknowledgement within deployment-
  declared latency
- Failure to acknowledge triggers fail-over to the secondary
  monitoring station per the routing policy

For monitoring stations integrating with PSTN voice (typical
for PERS), the boundary additionally initiates a SIP call to
the operator's queue with the alarm-event-id in a custom
SIP header so the operator's CTI can correlate.

## §6 Voice-call routing

Voice calls between subjects, monitoring stations, EMS, and
family contacts route per the deployment's voice topology:

- **Primary** — SIP via the deployment's session-border
  controller to the relevant terminating network
- **Backup** — PSTN trunk via a documented backup carrier
- **In-app audio** — WebRTC for app-based responder voice
  (typically used for app-only deployments without
  monitoring-station)

Voice-call records (timestamp, parties, outcome) are
captured but content is not recorded except where consent
explicitly authorises (e.g., quality-improvement recording).

## §7 Time discipline

Boundary clock: NTPv4 stratum-2. Devices on AC mains:
NTPv4 stratum-3. Battery-powered devices: time set on each
heartbeat or activation.

A device whose declared timestamp is more than the
deployment-declared skew tolerance from boundary time has
the alarm accepted but flagged `time-skew`. Persistent
skew triggers a maintenance ticket.

## §8 Audit chain

Every alarm event, routing attempt, transition, voice-call
record, and welfare-check emits an AuditEvent. Chain
construction follows the same pattern as WIA-medical-iot
PHASE 3 §5.

For high-volume deployments (large nursing homes,
multi-tenant social-alarm), the chain is sharded by
`installationRef` hash prefix.

## §9 Replay protection

All write endpoints accept `Idempotency-Key`; the boundary
stores keys for 30 days. CoAP/OSCORE replay protection uses
the OSCORE sequence-number window for constrained PERS
devices.

## §10 Disaster recovery

Boundary outages: monitoring stations operate from cached
roster + last-known device states. New activations queue at
the device until reconnect. Subjects with critical risk
profiles (recent hospital discharge, fall history) trigger
heightened-monitoring at the monitoring station during
boundary outage.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Algorithm choices (informative)

| Concern              | Default                            | Notes                                      |
|----------------------|------------------------------------|--------------------------------------------|
| Token signing        | ES256                              | mTLS-bound (RFC 8705)                      |
| TLS                  | 1.3 (RFC 8446)                     | hybrid groups when WIA-pq-crypto hybrid    |
| OSCORE AEAD          | AES-CCM-16-64-128                  | for constrained PERS devices               |
| Audit hash           | SHA-256                            |                                            |
| SIP signalling       | SIPS / TLS                         | TLS-protected SIP                          |

## Annex B — PERS heartbeat protocol (informative)

```
Device → Boundary: POST /Heartbeat
{
  "deviceRef": "urn:wia:malert:device:PERS-PDT-1.0:SN-91A7",
  "at": "2026-04-28T09:42:00+09:00",
  "battery": 0.74,
  "linkRsrp": -85,
  "buttonState": "idle",
  "lastSelfTestAt": "2026-04-27T03:00:00+09:00"
}

Boundary → Device: 200 OK
{"nextHeartbeatBy": "2026-04-28T09:43:00+09:00", "config": {...}}
```

Missed heartbeats trigger welfare-checks; the boundary
publishes the missed-heartbeat threshold per device kind in
the capability document.

## Annex C — Negative-test vectors (informative)

| Stimulus                                              | Expected outcome                                 |
|-------------------------------------------------------|--------------------------------------------------|
| Activation without subject reference (shared device)  | accepted; queued for subject resolution          |
| Activation with battery below operating threshold     | accepted; flagged with `low-battery` annotation  |
| Monitoring station offline beyond fail-over window    | secondary monitoring station receives           |
| Wifi-only device on cellular fallback                  | recorded as `bearer-fallback` annotation         |
| EN 50134-7 escalation timeout missed                   | next-level routing executed automatically        |

## Annex D — Disaster recovery worked example (informative)

A monitoring station experiences a network outage during a
busy evening. The deployment's failover policy:

1. Boundary detects no acknowledgement within 30 seconds
2. Failover to secondary monitoring station
3. Secondary station picks up; alarm event marked routed
4. Primary station logs reconnect on recovery
5. Boundary records the failover event in the audit chain
6. Quarterly compliance report counts failover events

## Annex E — Voice-call SIP integration (informative)

For monitoring stations integrating via SIP:

```
INVITE sip:operator-queue@monitoring-station.example SIP/2.0
From: <sip:wia-boundary@boundary.example>;tag=<random>
To: <sip:operator-queue@monitoring-station.example>
Call-ID: <random>
CSeq: 1 INVITE
X-WIA-Alarm-Event-Id: urn:wia:malert:event:PERS-PDT-1.0:SN-91A7:e-001
X-WIA-Subject-Hint: <redacted unless break-glass>
Contact: <sip:wia-boundary@boundary.example>
Content-Type: application/sdp
...
```

The X-WIA-Alarm-Event-Id header lets the operator's CTI
correlate the call with the alarm event in the FHIR
notification. Subject identifiers are not included unless
the operator's role grants break-glass authority for the
emergency context.

## Annex F — Battery management (informative)

PERS devices report battery state on every heartbeat. The
boundary computes:

- Current battery percentage
- Estimated remaining-life based on usage pattern
- Replacement-due indicator at deployment-declared threshold

Replacement scheduling integrates with the deployment's
biomedical-engineering or facilities operations workflow.

## Annex G — Decommissioning hardware-erase

PERS devices reassigned between subjects undergo a
hardware-erase sequence:

1. Operations submits decommission intent via control-plane
2. Device receives erase command over its bearer
3. Device clears all subject-bound storage and emits a
   completion attestation
4. Boundary verifies the attestation before clearing the
   subject-association record
5. Reuse is permitted only after attestation verified

The attestation is part of the device-pairing audit chain
so the next-subject pairing event traces back to the
sanitised state.

## Annex H — DTMF legacy social-alarm gateway

For legacy installations using DTMF / voice-band signalling
(BS 8521 in UK, similar in EU member states), the boundary
operates a translation gateway:

1. Legacy base unit places a voice call to the boundary's
   PSTN number
2. Gateway answers; legacy unit transmits DTMF identification
   and event-type codes
3. Gateway translates DTMF codes to FHIR AlarmEvent per the
   manufacturer's DTMF code map
4. Modern monitoring-station receives the FHIR AlarmEvent

The gateway is a transitional component; deployments are
encouraged to migrate to native LAN-based EN 50134-5
installations as legacy hardware reaches end of life.

## Annex I — Tamper detection protocol

Devices supporting tamper detection emit a tamper signal
on:

- Battery removed
- Pendant case opened
- Wall unit moved beyond declared installation position
- Device removed from charging cradle for an extended period

Tamper events are recorded as alarm events with
triggerKind=tamper-detected. Routing policy may handle
tamper events differently from physiological alarms (e.g.,
notify maintenance rather than EMS).

## Annex J — Boundary-to-EMS NEMSIS payload mapping

The boundary maps internal AlarmEvent fields to NEMSIS-style
payload fields for EMS dispatch (US):

| Internal field             | NEMSIS field                                      |
|----------------------------|---------------------------------------------------|
| triggerKind                | eDispatch.01 (Complaint Reported by Dispatch)    |
| location                   | eScene.06 (Incident Address) + eScene.16 (GPS)   |
| priority                   | eDispatch.05 (Possible Injury)                   |
| vitalsAttached             | eVitals.06 (...) carried as on-scene observations |
| subject demographic        | ePatient.01 (...) per consent + break-glass      |

Equivalent mappings exist for other regions; the deployment
declares its dispatch standard in the capability document.
