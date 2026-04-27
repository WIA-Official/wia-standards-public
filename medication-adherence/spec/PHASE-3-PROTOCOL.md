# WIA-medication-adherence PHASE 3 — Protocol Specification

**Standard:** WIA-medication-adherence
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable

This PHASE specifies the protocols binding the data formats
(PHASE 1) and the API surface (PHASE 2) to operational exchanges:
authentication of clinicians, pharmacies, devices, patients and
caregivers; smart-pill-bottle pairing and verification; ingestion-
sensor data flow; eMAR integration; controlled-substance handling
protocol; time discipline; audit-chain construction.

References (CITATION-POLICY ALLOW only):
- IETF RFC 8446 (TLS 1.3), RFC 7525, RFC 9162 (CT pattern)
- IETF RFC 7252 (CoAP) and RFC 8613 (OSCORE) — for constrained
  pill-bottle / blister-pack devices
- IETF RFC 7515 (JWS), RFC 7519 (JWT), RFC 8705 (mTLS-bound JWT)
- HL7 v2.x messaging protocols
- IHE Pharmacy Hospital Medication Workflow profile
- ISO/IEC 11073-10472 — medication monitor specialisation
- ISO/IEC 11073-20601 — application profile (optimised exchange)
- DEA 21 CFR §1311 — for US controlled-substance electronic
  prescribing requirements (where deployment is in US scope)
- KFDA / K-MFDS 의약품 안전사용서비스 — for KR scope

---

## §1 Authentication

Principals authenticate via JWS-signed JWTs:

- `iss`, `sub`, `aud`, `iat`, `exp`
- `wia.role` — `prescriber`, `pharmacist`, `nurse`, `patient`,
  `caregiver`, `device`, `regulator`, `auditor`
- `wia.holderRef` — for clinician/pharmacist tokens, the
  organisation whose patients/dispenses may be acted upon
- `wia.deviceRef` — for device tokens, the URN of the bound
  smart device
- `cnf` — confirmation claim binding to a TLS client certificate
  (RFC 8705 mTLS-bound JWT)

Prescriber tokens for controlled substances additionally carry
the prescriber's DEA-equivalent identifier (or the deployment's
jurisdictional equivalent) and require two-factor authentication
per the controlling regulation.

## §2 Smart pill bottle / blister pack pairing

Smart medication-monitor devices (per ISO/IEC 11073-10472)
pair to the deployment via a documented sequence:

1. Patient receives the device with a pairing token printed on
   a tear-off sticker
2. Patient opens the patient-facing app and scans the pairing
   token
3. App + device complete a Bluetooth LE pairing per IEEE 11073-
   20601 application profile
4. Boundary records the pairing as a `device-pairing` event
   with `gatewayRef` = patient's smartphone
5. Device emits subsequent dose-event observations through the
   patient's smartphone gateway

Unpairing is initiated by the patient or the deployment;
boundary records the event and refuses subsequent observations
from the unpaired device.

## §3 Smart device dose-event protocol

Dose events from smart bottles / blister packs:

- `eventKind` — `cap-opened`, `cap-closed`, `blister-pressed`,
  `dose-dispensed-confirmed`, `dose-skipped-confirmed`
- `at` — RFC 3339 with offset
- `dose` — for dispense-confirmed events, the actual dose
  delivered (if the device measures it)
- `deviceObs` — supplementary observations (battery level,
  desiccant indicator, sensor health)

The boundary correlates events with the patient's scheduled
doses; an open-then-close within a tolerance window is treated
as a tentative administration. Confirmation requires a second
signal (ingestion sensor, patient self-report, or a follow-up
inquiry) before the administration is recorded with high-
reliability evidence.

## §4 Ingestion-sensor protocol

Ingestion-sensor systems (FDA-cleared digital medicines with
ingestible sensor + body-worn patch + smartphone gateway):

- Patch detects the sensor's signal at ingestion; signal
  decodes to a manufacturer-specific identifier
- Patch's gateway uploads the detection through the smartphone
- Boundary cross-references the manufacturer ID against
  active prescriptions
- Mismatches (sensor ID not matching any active prescription)
  are flagged for clinical review

The boundary stores patch-detected signals as
MedicationAdministration records with
`evidenceSource: ingestion-sensor`. Patient consent for
ingestion-sensor monitoring is a separate consent class (per
WIA-medical-data-privacy `provision.purpose`) so opt-in is
explicit.

## §5 eMAR integration protocol

In-hospital eMAR systems integrate via:

- HL7 v2.x RAS/RGV messages over MLLP (legacy)
- FHIR MedicationAdministration POST (modern)
- IHE PHMP profile

The boundary translates between protocols transparently;
audit-chain entries record the source protocol so retrospective
review can reconstruct.

## §6 Controlled-substance handling protocol

Controlled-substance prescriptions follow elevated discipline:

- Prescriber two-factor authentication on every prescription
- Pharmacy verification of DEA-equivalent ID before dispense
- Per-jurisdiction reporting cadence (US DEA's PMP, KR 마약류
  통합관리시스템, etc.) — boundary emits the required reports
  on schedule
- Diversion monitoring: unusual patterns (rapid early refills,
  multiple prescribers, multiple pharmacies) flagged for
  pharmacist + medical-leadership review
- Disposal records: boundary tracks returned / unused
  controlled-substance disposal per regulatory requirement

## §7 Time discipline

Devices and gateways synchronise to authoritative sources;
boundary clock is NTPv4 stratum-2. Smart devices on battery
typically derive time from their gateway (smartphone) on
each handshake; gateway clock is the smartphone OS time
source.

A device whose declared event timestamp is more than the
deployment-declared skew tolerance from boundary time has
the event accepted but flagged `evidenceReliability: time-skew`.

## §8 Audit chain

Every prescription mutation, dispense, scheduled-dose
generation, administration, deviation, refill, and adherence-
summary computation emits an AuditEvent. Chain construction
follows the same Merkle-chain pattern as WIA-medical-iot
PHASE 3 §5.

For high-volume in-hospital eMAR deployments, the chain is
sharded by `subjectRef` hash prefix.

## §9 Replay protection

All write endpoints accept `Idempotency-Key`; the boundary
stores keys for 30 days. CoAP/OSCORE replay protection uses
the OSCORE sequence-number window for constrained devices.

## §10 Disaster recovery

Boundary outages during eMAR operation degrade to local-mode:
the eMAR continues recording locally; on reconnection the
boundary accepts the buffered records with their original
timestamps. The eMAR's local audit chain is sealed at
reconnection and merged into the boundary's chain.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Algorithm choices (informative)

| Concern              | Default                            | Notes                                    |
|----------------------|------------------------------------|------------------------------------------|
| Token signing        | ES256                              | mTLS-bound (RFC 8705)                    |
| TLS                  | 1.3 (RFC 8446)                     | hybrid groups when WIA-pq-crypto hybrid  |
| OSCORE AEAD          | AES-CCM-16-64-128                  | for smart bottles / blister packs        |
| Audit hash           | SHA-256                            |                                          |

## Annex B — Smart device pairing worked example (informative)

```
1. Patient scans pairing token "WIA-PAIR-91A7-77C2" via app
2. App resolves token to deviceRef = urn:wia:mdadh:device:smart-bottle-1.0:SN-91A7
3. App + device complete BLE pairing with the device's GATT service
4. App posts to /devices/$pair with patient JWT + deviceRef + pairing evidence
5. Boundary verifies pairing evidence, creates DeviceAssociation
6. Device begins emitting dose-events through the gateway
```

## Annex C — Controlled-substance reporting cadence (informative)

| Jurisdiction | Required cadence                                       |
|--------------|--------------------------------------------------------|
| US DEA       | Per state PMP requirement (typically 24 hours after dispense) |
| KR M-FDS     | 마약류 통합관리시스템: 익일 보고                         |
| EU           | Per Member State (varies)                              |
| JP           | Per PMD Act controlled-substance pharmacist reporting   |

The boundary publishes the deployment's reporting schedule
in the deployment policy.

## Annex D — Negative-test vectors (informative)

| Stimulus                                              | Expected outcome                                    |
|-------------------------------------------------------|-----------------------------------------------------|
| Smart device claim without prior pairing              | 422 + `device-not-paired`                           |
| Controlled-substance dispense from unauthorised pharmacist | 403 + `controlled-substance-authority-required` |
| Late dose for time-critical medication                | clinical-alert deviation + clinician escalation     |
| Eperal: Smart bottle event timestamp > 5 min skew     | accepted, flagged `time-skew`                       |
| Ingestion-sensor signal not matching active Rx        | flagged for clinical review; not auto-recorded as administration |

## Annex E — Patient-app local-mode (informative)

When the patient-facing app loses connectivity:

- Local self-reports are buffered with locally-generated event IDs
- Smart-device events are buffered through the gateway role
- On reconnection, buffered events sync with server-side
  validation; duplicates are de-duplicated via Idempotency-Key

A patient who re-installs the app needs to re-authenticate
and re-pair smart devices; the boundary records the re-install
as a security event for review.

## Annex F — Adverse-event linkage (informative)

When a patient experiences an adverse event potentially related
to a medication:

1. Clinician submits an AdverseEvent resource
2. Boundary cross-references administrations within the
   declared time window
3. Adverse event is linked to candidate administrations for
   pharmacovigilance review
4. The pharmacovigilance partner subscription delivers the
   linked bundle for evaluation

The linkage is informational; causality determination is the
clinician's responsibility.

## Annex G — Mail-order shipping-delay handling

Mail-order pharmacies may experience shipping delays that
affect supply continuity. The boundary detects:

- Predicted end-of-supply within deployment-declared window
  (e.g., 3 days)
- No new dispense in flight
- Triggers an `expedite-refill` event for the pharmacy partner
- If still no dispense within 2 days, escalates to clinical
  team for bridge-supply or alternate-pharmacy resolution

This pattern protects patients on chronic medications from
inadvertent missed doses due to logistics issues.

## Annex H — eMAR-to-boundary translation cadence (informative)

The boundary translates between in-hospital eMAR formats:

| Direction                  | Latency target | Notes                                  |
|----------------------------|---------------|----------------------------------------|
| HL7 v2 RAS → FHIR Admin    | ≤ 1 s         | MLLP listener with checksum verification |
| FHIR Admin → HL7 v2 RAS    | ≤ 2 s         | for legacy partners                    |
| RGV (Give) → FHIR Admin    | ≤ 1 s         | per administration                     |
| RDS (Dispense) → FHIR Disp | ≤ 1 s         | per dispense                           |

Translation failures are recorded as audit events and
delivered to the eMAR's operations team. Persistent translation
failure on a partner triggers a partner-roster review.

## Annex I — Disaster recovery worked example (informative)

Boundary outage during a busy in-hospital eMAR shift:

1. eMAR detects boundary unreachable
2. eMAR continues recording locally with same per-event IDs
3. Local audit chain accumulates with eMAR's signing key
4. Boundary recovers; eMAR retries buffered events with
   Idempotency-Key
5. Boundary accepts buffered events; merges local audit chain
   into boundary's chain
6. Boundary publishes recovery-evidence record
7. Operations review the gap window and any policy gaps
