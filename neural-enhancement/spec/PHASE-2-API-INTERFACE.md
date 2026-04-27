# WIA-neural-enhancement PHASE 2 — API Interface Specification

**Standard:** WIA-neural-enhancement
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable

This PHASE defines the API surface a neural-enhancement
deployment exposes for device registry, recording-session
publication, stimulation-session control, decoded-intent
streams, calibration management, adverse-event reporting,
consent management, and clinical-context capture. The shape
is HTTP/JSON for clinical-system planes; high-rate neural
streaming uses the binary projection in PHASE 3.

References (CITATION-POLICY ALLOW only):
- IETF RFC 9457 (Problem Details), RFC 7515 (JWS)
- HL7 FHIR R5 — for AE-reporting resources
- Brain Imaging Data Structure (BIDS) — for recording bundle layout
- Neurodata Without Borders (NWB) 2.x
- WIA-privacy PHASE 2 — for participant-pseudonym binding
- IEC 62366-1 — for usability-engineering linkage

---

## §1 Device registry endpoints

```
POST /devices HTTP/1.1
Authorization: Bearer <jws-clinician-jwt>
Content-Type: application/json
```

Body is a PHASE 1 §2 device record. Successful intake
returns 201 Created with the boundary's URN binding. Updates
to firmware version use PUT against the device URN; the
boundary verifies the firmware-update artifact's provenance
through the WIA-supply-chain integration.

```
GET /devices/{deviceRef}
GET /devices?vendorRef=…&deviceClass=intracortical-array
```

## §2 Recording-session publication

```
POST /recordings HTTP/1.1
```

Body is a PHASE 1 §3 recording record. The boundary verifies
`consentRef` is current and includes `recording` in scope;
without that, the response is 403 with
`urn:wia:nenh:problem:consent-missing-recording-scope`.

```
GET /recordings/{recordingId}
GET /recordings?participantPseudonym=…&since=…
```

The BIDS/NWB bundle is referenced by URI; the boundary
does not store raw neural data inline. Bundles are stored
in the deployment's clinical-data store with appropriate
access controls.

## §3 Stimulation-session control

```
POST /stimulation/sessions HTTP/1.1
```

Body is a PHASE 1 §4 stimulation record. Pre-checks at
intake:

- `consentRef` is current and includes `stimulation` scope
- `safetyEnvelopeRef` is current and signed by the
  responsible clinician
- Device is within calibration window (or under documented
  exception)

```
PUT /stimulation/sessions/{stimulationId}/state
```

State transitions: `proposed` → `scheduled` → `in-progress`
→ `completed`. Mid-session interruption requires the safety-
envelope's `interrupt` action to be issued; the boundary
records the interruption rationale.

```
POST /stimulation/sessions/{stimulationId}/interrupt
```

Issued by the safety system or clinician; the boundary halts
ongoing stimulation, records the interruption, and emits a
`stimulation-interrupted` audit-chain entry. Re-engagement
requires an explicit re-authorisation step.

## §4 Decoded-intent stream

For BCI applications, decoded-intent records are streamed:

```
GET /decode/sessions/{recordingId}/stream HTTP/1.1
Accept: text/event-stream
```

Each event is a PHASE 1 §5 decoded-intent record. The
stream is filtered to the analyst's authorised scope and
respects the participant's consent. Effector-output
dispatch is subject to the active safety envelope.

```
POST /decode/effector-acks HTTP/1.1
```

Effector control planes acknowledge dispatch by posting an
ack record referencing the decoded-intent URN.

## §5 Calibration management

```
POST /calibrations HTTP/1.1
GET /calibrations/{calibrationId}
GET /calibrations?deviceRef=…&since=…
```

POST submits a PHASE 1 §6 calibration record. The boundary
updates the device's `lastServiceAt` and `reCalibrationDueAt`
based on the methodology. Devices past their due date are
flagged in the registry; subsequent stimulation sessions
against them are refused (or flagged for documented
exception).

## §6 Adverse-event reporting

```
POST /adverse-events HTTP/1.1
```

Body is a PHASE 1 §7 adverse-event record. The boundary:

- Verifies `participantPseudonym` matches an active
  participant
- Notifies the clinical-team's AE workflow via webhook
- Pauses outstanding stimulation sessions for the
  participant pending review
- Triggers PHASE 4 §6 regulatory-reporting workflow

```
PUT /adverse-events/{aeId}/state
```

State transitions follow the deployment's AE workflow:
`open` → `under-investigation` → `assessed` → `reported` →
`closed`. Each transition is signed.

## §7 Consent management

```
POST /consents HTTP/1.1
```

Body is a PHASE 1 §8 consent record. The boundary verifies
the protocol exists and the participant pseudonym is
recognised.

```
PUT /consents/{consentId}/withdraw
```

Withdraws an active consent. The withdrawal:

- Revokes outstanding session-permissions associated with
  the consent's scope
- Notifies the clinical team
- Preserves data already collected per the deployment's
  retention policy
- Audit-chains the withdrawal

## §8 Clinical-context note publication

```
POST /clinical-notes HTTP/1.1
```

Body is a PHASE 1 §9 clinical-context note. The boundary
stores the note and links it to referenced sessions; notes
are accessible to the participant's care team and the
research analytics team per the consent's scope.

## §9 Capability discovery

```
GET /.well-known/wia/neural-enhancement HTTP/1.1
```

Returns the capability document:

```json
{
  "wia.standardVersion": "1.0",
  "wia.implementationVersion": "clinic-x-2.1.0",
  "supportedDeviceClasses": ["intracortical-array","ecog-grid","eeg-headset"],
  "supportedModalities": ["electrical","magnetic"],
  "preferredBundleFormat": "bids",
  "consentFrameworkRef": "urn:wia:nenh:consent-framework:clinic-x-v3",
  "manifest": "https://clinic-x.example/.well-known/wia/neural-enhancement/manifest.jws"
}
```

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Idempotency

Recording POST and stimulation POST accept `Idempotency-Key`.
Boundary stores keys for 7 days. Replays return the original
response.

## Annex B — Pagination

List endpoints support cursor pagination with cursors signed
by the boundary, valid for 30 minutes. Page envelopes
declare the snapshot epoch.

## Annex C — Negative-test vectors (informative)

| Stimulus                                            | Expected response                              |
|-----------------------------------------------------|------------------------------------------------|
| Recording POST without consent in scope             | 403 + consent-missing-recording-scope          |
| Stimulation POST against device past calibration    | 403 + calibration-expired (or under-exception) |
| Stimulation parameters outside safety envelope      | 422 + safety-envelope-violation                |
| AE POST referencing unknown participant pseudonym   | 422 + participant-unknown                      |
| Consent withdrawal during in-progress stimulation   | 200 + queued; safe-exit triggered              |

## Annex D — Bulk export

For research replay, bulk export endpoints serve NDJSON over
a date range, gated by the deployment's bulk-quota policy:

```
GET /export/recordings?from=…&to=…&protocolRef=…
GET /export/decode?from=…&to=…
```

Bulk export honours the consent's data-sharing scope; export
of non-sharing-scoped data is refused.

## Annex E — Webhook subscriptions

Push subscriptions for clinical teams, research teams, and
device vendors:

```
POST /subscriptions HTTP/1.1
```

Body declares event classes (`stimulation-interrupted`,
`adverse-event-opened`, `consent-withdrawn`, `device-firmware-
updated`) and filters. Webhook delivery uses TLS 1.3 with a
detached JWS in `Wia-Signature`.

## Annex F — Authorities and roles

| Role                    | Scope                                            |
|-------------------------|--------------------------------------------------|
| `responsible-clinician` | full read/write on managed participants          |
| `treating-clinician`    | session-level write within consent               |
| `research-analyst`      | recording read within consent's data-sharing scope |
| `device-vendor`         | firmware update artifact submission              |
| `regulatory-officer`    | AE-reporting workflow                            |
| `participant`           | own consent and own data access                  |
| `auditor`               | read-only across engagement scope                |

## Annex G — Worked AE record (informative)

```json
{
  "aeId": "urn:wia:nenh:ae:clinic-x:ae-2026-04-27-001",
  "participantPseudonym": "p-clinic-x-PR-014",
  "deviceRef": "urn:wia:nenh:device:clinic-x:d-001",
  "relatedSessionRefs": ["urn:wia:nenh:stimulation:clinic-x:s-2026-04-27-014"],
  "onsetAt": "2026-04-27T14:50:00+09:00",
  "severity": "moderate",
  "narrativeRef": "urn:wia:nenh:note:clinic-x:n-001",
  "causalityAssessment": "possible",
  "reportingObligationsRef": "urn:wia:reg:rok-mfds-medical-device-ae"
}
```

## Annex H — Capability versioning

Capability documents declare both `wia.standardVersion` and
`wia.implementationVersion`. A standard-version mismatch is
a hard refusal; an implementation-version mismatch is logged
but not refusing.

## Annex I — Audit-chain replay

For regulators and Anchored auditors, the boundary serves a
selective audit-chain replay at `/audit/chain` filtered by
kind and time range. Restricted kinds (e.g., AE) require
the requester's role to authorise access.

## Annex J — Re-identification request endpoint

When clinically necessary, re-identification of a
participant pseudonym may be requested:

```
POST /reidentify HTTP/1.1
Authorization: Bearer <jws-responsible-clinician-jwt>
Content-Type: application/json

{
  "reidentifyId": "urn:wia:nenh:reid:clinic-x:r-001",
  "participantPseudonym": "p-clinic-x-PR-014",
  "rationale": "treating-clinician requires medical-history access for AE assessment",
  "consentRef": "urn:wia:nenh:consent:clinic-x:c-014-2026-q1",
  "responsibleClinicianSignature": "<jws>"
}
```

The boundary verifies the consent includes `re-identification`
in scope; without that, the request is refused. Approved
re-identification is audit-chained at
`kind=participant-reidentified`.

## Annex K — Effector control-plane handshake

Effector control planes register with the boundary via:

```
POST /effectors HTTP/1.1
Authorization: Bearer <jws-effector-control-plane-jwt>

{
  "effectorRef": "urn:wia:nenh:effector:clinic-x:e-001",
  "effectorType": "prosthetic-arm",
  "vendorRef": "urn:wia:vendor:effector-vendor-a",
  "safetyEnvelopeHash": "<sha-256>",
  "controlPlaneSignature": "<jws>"
}
```

The boundary verifies the safety-envelope hash against the
clinician-signed envelope referenced by sessions targeting
the effector. Mismatches cause refusal of dispatch until
reconciled.

## Annex L — Sandbox lane

For pre-production validation, the boundary exposes a
sandbox lane:

```
POST /sandbox/recordings
POST /sandbox/stimulation/sessions
```

Sandbox endpoints accept records under a synthetic-data
participant pseudonym. Sandbox results never feed clinical
review or regulatory reporting.

## Annex M — Data-access request handling

Participant data-access requests are received via the portal
or a written request to the deployment's privacy officer:

- Request is recorded as a `data-access-request` artifact
- The privacy officer reviews scope and produces the
  exportable bundle (PHASE 1 §3 records, AE records,
  consent records, decoded-intent summaries)
- Bundle is delivered via a secure channel chosen by the
  participant
- Request handling is audit-chained at `kind=data-access-fulfilled`

## Annex N — Operations dashboard endpoints

For SOC-leadership and risk-officer visibility, the boundary
exposes aggregate dashboards:

```
GET /metrics/sessions?period=2026-Q2&groupBy=deviceClass
GET /metrics/ae?period=2026-Q2&groupBy=severity
GET /metrics/calibration-trend?deviceClass=intracortical-array
```

Metrics are aggregated and respect TLP markings on the
underlying records.
