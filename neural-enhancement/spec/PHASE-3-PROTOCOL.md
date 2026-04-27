# WIA-neural-enhancement PHASE 3 — Protocol Specification

**Standard:** WIA-neural-enhancement
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable

This PHASE specifies the protocols binding the data format
(PHASE 1) to the API surface (PHASE 2): authentication of
clinicians, devices, participants, and regulators; the
high-rate streaming binary projection for neural recording;
the safety-envelope enforcement protocol; the participant-
pseudonym lifecycle; the audit-chain construction; time
discipline; cryptographic signing; and post-quantum
migration.

References (CITATION-POLICY ALLOW only):
- IETF RFC 8446 (TLS 1.3), RFC 7515 (JWS), RFC 7517 (JWK),
  RFC 9162 (Certificate Transparency 2.0)
- HL7 FHIR R5 — for AE-reporting interoperability
- IEC 62304 / ISO 13485 — for medical-device QMS linkage
- ISO 14971:2019 — risk management framework reference
- WIA-pq-crypto PHASE 3 — for ML-KEM/ML-DSA migration profiles

---

## §1 Authentication

Clinicians, devices, participants, regulators, and research
analysts authenticate using JWS-signed JWTs issued by the
deployment's identity authority. Token claims:

- `iss`, `sub`, `aud`, `iat`, `exp`
- `wia.role` — one of the roles in PHASE 2 Annex F
- `wia.scope[]` — operation-class scopes
- `wia.deviceRef` — for device tokens, the URN of the
  device the token speaks for
- `wia.participantPseudonym` — for participant tokens

Clinician write tokens are short-lived (typically 15
minutes); device tokens may be longer-lived (24 hours)
where rotated by the deployment's secrets pipeline.
Participant self-service tokens are short-lived.

## §2 Streaming binary projection

Recording sessions stream raw neural data to the boundary's
clinical-data store via a length-prefixed CBOR stream over a
mutual-TLS 1.3 connection. Each frame:

- 4-byte length prefix
- CBOR-encoded recording slice with channel data
- 32-byte per-frame MAC (HMAC-SHA-256 with rotated key)

Stream cadence is device-class-dependent (typically tens of
milliseconds per slice for intracortical arrays, hundreds
for EEG). Stream resumption uses opaque resume tokens; the
boundary persists tokens until session closure.

## §3 Safety-envelope enforcement

Stimulation safety envelopes enforce per-session limits at
multiple layers:

- Device-firmware enforcement of hard hardware limits
- Boundary-side enforcement of session-level limits (cumulative
  charge per electrode, cumulative session duration, peak
  amplitude)
- Clinician-supervised enforcement of contextual limits
  (e.g., automatically pausing on detected after-discharges)

A safety-envelope mutation requires a responsible-clinician
counter-signature, an updated risk assessment per ISO 14971,
and an audit-chain entry with the prior envelope version
referenced. Real-time envelope violations trigger an
immediate session interrupt; the audit chain records the
violation parameters.

## §4 Audit chain

Every boundary state transition is appended to a Merkle
audit log:

- `entryId`, `parent`, `at`, `actor`, `kind`, `payloadHash`,
  `signature`
- `kind` enum: `device-registered`, `device-firmware-updated`,
  `recording-published`, `stimulation-scheduled`,
  `stimulation-started`, `stimulation-interrupted`,
  `stimulation-completed`, `decode-event`, `effector-dispatched`,
  `safety-envelope-violation`, `calibration-published`,
  `ae-opened`, `ae-state-changed`, `ae-closed`,
  `consent-published`, `consent-withdrawn`,
  `participant-reidentified`, `regulatory-witnessed`

Anchored deployments mirror the audit chain to a regulator-
trusted witness on a declared cadence.

## §5 Participant-pseudonym lifecycle

Participant pseudonyms are issued at enrolment and follow
this lifecycle:

- Issued on enrolment with a corresponding consent record
- Stable across sessions within the deployment
- Bound to PII via a separate, access-controlled service
  ("participant vault")
- Re-identification requires a documented clinical or
  regulatory necessity (PHASE 2 Annex J)
- Retired on participant withdrawal; vault binding is held
  per the deployment's records-retention policy

Cross-deployment portability is out of scope; participants
in multi-site studies receive site-specific pseudonyms.

## §6 Time discipline

All record timestamps use RFC 3339 with explicit offset.
Recording-site clocks are disciplined to GPS-based time
references; drift outside declared bound tags subsequent
sessions `provisional` until recovered. Device-internal
clocks are typically less accurate but are post-processed
against the recording-site reference at session close.

## §7 Transport security

All endpoints require TLS 1.3 (RFC 8446) with a deployment-
declared cipher-suite list. Mutual TLS is required for
device, regulator, and binary-projection endpoints.
Certificate revocation is published through the deployment's
revocation surface.

## §8 Privacy-preserving disclosure protocol

For data-sharing scope on consent records, the boundary
applies a documented disclosure protocol:

- Disclosed data is filtered to the consent's
  data-sharing-scope subfields
- Optional aggregation or de-identification per declared
  policy
- The disclosure event is itself audit-chained at
  `kind=data-disclosed`
- Disclosure recipients are recorded in the consent's
  recipient roster

A disclosure outside scope is refused at the boundary and
audit-chained at `kind=disclosure-refused`.

## §9 Replay protection

Recording POST, stimulation POST, AE POST, and consent POST
require `Idempotency-Key`; the boundary stores keys for 7
days. Replays return the original response; conflicts return
`urn:wia:nenh:problem:idempotency-conflict`.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Cryptographic signature suite

Default signature algorithms are ECDSA P-256 with SHA-256 for
clinician tokens and EdDSA (Ed25519) for audit-chain entries.
Deployments may declare alternative suites in the capability
document; partners verify suite compatibility on initial
connection.

## Annex B — Post-quantum migration

The standard supports a phased PQC migration aligned with
WIA-pq-crypto PHASE 3 (Phase A → Hybrid → PQ-only). Devices
in unmaintained appliances may remain on Phase A under
documented exception; the exception is recorded in the
capability document and tracked for end-of-support
remediation.

## Annex C — Cipher-suite floors

Endpoints accept only TLS 1.3 cipher suites with forward
secrecy. The cipher-suite floor is published in the
capability document; partners verify compatibility before
exchange. Device-class onboarding playbooks declare floors
per device class so vendors verify support.

## Annex D — Negative-test vectors for protocol layer

| Stimulus                                              | Expected outcome                              |
|-------------------------------------------------------|-----------------------------------------------|
| Device token without `wia.deviceRef`                  | 422 + device-token-malformed                  |
| Stimulation parameter exceeding safety envelope       | session interrupt + audit-chain entry         |
| Pseudonym reuse across deployments                    | 422 + pseudonym-not-portable                  |
| Disclosure outside consent's data-sharing scope       | 403 + disclosure-out-of-scope                 |
| Audit-chain entry with broken parent hash             | rejected at append; boundary alerts           |

## Annex E — Algorithm registry

The deployment maintains an algorithm registry naming the
cipher and signature algorithms in use per role. The
registry is published in the capability document and tracked
across PQ migration phases for partner verification.

## Annex F — Boundary-clock health

The boundary publishes a clock-health record in the
capability document including the primary and secondary time
sources, the most recent successful sync, and the current
drift estimate.

## Annex G — Device-onboarding handshake

Device onboarding follows a documented handshake:

1. Vendor presents a signed device-bundle manifest
   declaring `deviceRef`, `udi`, regulatory clearances,
   and supported modalities/parameters
2. Boundary verifies the manifest's signature against the
   vendor's published key
3. The responsible clinician reviews and approves the
   device for use within their patient population
4. Initial calibration session is conducted; the
   calibration record initiates the device's lifecycle
5. Device is recorded in the registry and capability
   document

A device whose vendor manifest signature fails or whose
regulatory clearance lapses is moved to `restricted` state;
new sessions are refused until reinstatement.

## Annex H — Risk-management cross-reference

The deployment's ISO 14971 risk-management file references
PHASE 1 §7 AE records and PHASE 3 §3 safety-envelope
mutations as sources of post-market data. The risk file is
versioned and audit-chained alongside the safety envelope.

## Annex I — Stream-ack cadence

The streaming binary projection (§2) acknowledges in batches.
The default cadence is once per recording slice (typically
tens of milliseconds for intracortical arrays); deployments
with bandwidth constraints may declare a longer batch
cadence. Acknowledgement loss is detected by clients
comparing the boundary's last-known-good cursor to the
client's last-emitted slice; clients retransmit any
unacknowledged tail upon reconnect.

## Annex J — Safety-envelope mutation workflow

Safety-envelope mutations follow a documented workflow:

1. Responsible clinician proposes the mutation with rationale
2. The deployment's risk-management programme assesses the
   change against ISO 14971
3. Risk file is updated and re-signed
4. Mutation is staged in the sandbox lane for verification
5. Promotion to production requires sign-off from the
   responsible clinician and the deployment's risk-officer
6. Audit-chain entry records the prior envelope version,
   the new version, and the risk-file version

A mutation that bypasses the workflow is refused at the
boundary; the boundary's mutation endpoint requires both
signatures before applying.

## Annex K — Worked safety-envelope record

```json
{
  "envelopeId": "urn:wia:nenh:envelope:clinic-x:e-001",
  "deviceRef": "urn:wia:nenh:device:clinic-x:d-001",
  "modality": "electrical",
  "perElectrodeCumulativeChargeUcLimit": 30.0,
  "sessionDurationMinutesLimit": 90,
  "peakAmplitudeMaLimit": 5.0,
  "perPulseChargeUcLimit": 0.45,
  "afterDischargeMonitor": {
    "channels": ["m1-ch-3","m1-ch-4"],
    "thresholdUv": 80,
    "actionOnDetect": "interrupt"
  },
  "responsibleClinicianRef": "urn:wia:auth:clinic-x-clinician-7",
  "riskFileVersion": "rmf-2026-q2",
  "responsibleClinicianSignature": "<jws-detached>",
  "riskOfficerCounterSignature": "<jws-detached>"
}
```

The boundary verifies both signatures before applying the
envelope; mutation requires re-signing both. Real-time
violations of the envelope (e.g., after-discharge detected)
trigger immediate session interrupt and audit-chain entry.

## Annex L — Cross-deployment audit harmonisation

For multi-site studies, audit chains across sites are
harmonised by:

- A study-coordinator-trusted witness mirrors selected entry
  classes (`ae-opened`, `consent-withdrawn`, `safety-envelope-
  violation`) from each site's audit chain
- Cross-site reconciliation runs on the witness, surfacing
  patterns that no single site would notice (e.g., a clustered
  AE pattern across multiple sites against a single device
  class)
- The coordinator-witness's reconciliation outputs are
  themselves audit-chained on each contributing site's chain
  for cross-site replay

## Annex M — JWT claim cookbook

Worked clinician token:

```json
{
  "iss": "urn:wia:auth:clinic-x-identity",
  "sub": "urn:wia:auth:clinic-x-clinician-7",
  "aud": "urn:wia:nenh:boundary:clinic-x",
  "iat": "2026-04-27T14:00:00+09:00",
  "exp": "2026-04-27T14:15:00+09:00",
  "wia.role": "responsible-clinician",
  "wia.scope": ["devices:write","recordings:write","stimulation:write","ae:write"]
}
```
