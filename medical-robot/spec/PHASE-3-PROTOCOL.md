# WIA-medical-robot PHASE 3 — Protocol Specification

**Standard:** WIA-medical-robot
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable

This PHASE specifies the protocols binding the data formats
(PHASE 1) and the API surface (PHASE 2) to operational exchanges:
robot authentication, real-time control-link discipline, tele-
surgery transport, motion-telemetry pacing, time discipline, audit-
chain construction, and the safety protocols that enforce
IEC 80601-2-77 / IEC 80601-2-78 essential performance.

References (CITATION-POLICY ALLOW only):
- IETF RFC 8446 (TLS 1.3), RFC 7525, RFC 9162 (CT pattern)
- IETF RFC 7252 (CoAP), RFC 8323 (CoAP/TLS over TCP), RFC 8613
  (OSCORE) — for constrained satellite devices on the robot
- IETF RFC 8030 (Web Push) — for low-priority operator
  notifications outside the OR
- ISO 8000 series — data quality (where motion telemetry must
  satisfy a quality contract)
- IEC 80601-2-77:2019, IEC 80601-2-78:2019
- ISO 13482:2014 — for service-robot kinds
- IEC 60601-1-8:2020 — alarm escalation timing (referenced from
  PHASE 1 §6)
- 3GPP TS 33.501 (5G security) — for tele-surgery over public
  cellular when local network is unavailable (escalation only)

---

## §1 Authentication

Robots, operators, BME, manufacturer agents, and auditors
authenticate via JWS-signed JWTs. Token claims:

- `iss`, `sub`, `aud`, `iat`, `exp`
- `wia.role` — `robot`, `operator`, `bme`, `manufacturer`,
  `auditor`, `patient`
- `wia.robotRef` — for robot tokens, the URN of the bound robot
- `wia.operatorCredentials[]` — for operator tokens, the
  credentials that determine procedure eligibility
- `cnf` — confirmation claim binding the token to a TLS client
  certificate (RFC 8705)

Robot tokens rotate hourly during procedures (because the
session is bounded) and daily when idle. Operator tokens are
SMART-launch grants from the EHR, scoped to the procedure.

## §2 Robot control link

Within an OR, the robot, the operator console, and the
boundary share a private LAN. Control-link transport:

- TLS 1.3 with mTLS between robot and console
- Control messages encoded in CBOR for bandwidth efficiency
- Heartbeat every 50 ms; missed heartbeat for > 200 ms
  triggers `tele-link-loss` alarm
- Watchdog timer in the robot (independent of the operating
  system) trips a safe-state transition if no command arrives
  for > 200 ms

Cross-room or cross-site control (tele-surgery) requires
additional protocols (§3).

## §3 Tele-surgery transport

For tele-surgery scenarios, the boundary supports:

- Dedicated low-latency network path (typically MPLS or
  point-to-point fibre) with sub-50 ms one-way latency
- TLS 1.3 with mTLS, hybrid groups when the deployment is in
  WIA-pq-crypto `hybrid` phase
- Forward Error Correction (FEC) at the application layer to
  tolerate single-frame packet loss without retransmission
- Jitter buffer sized to the 99th-percentile observed jitter,
  refilled continuously during the procedure
- Quality-of-Service (QoS) marking (DSCP EF) on the bearer
  network to guarantee priority over best-effort traffic

The boundary observes link quality continuously; persistent
degradation triggers handover to a backup link or, if no
backup is available, a `tele-link-loss` event with safe-state
transition (§4).

## §4 Safe-state transition

On any of the following events, the robot transitions to
safe state per IEC 80601-2-77 §201.4.4:

- Tele-link loss (heartbeat or quality threshold)
- Operator emergency-stop press
- Force-limit violation
- Safety-envelope violation
- Watchdog trip

Safe state is robot-kind-specific:

- **Surgical (laparoscopic, ortho, neuro, ophthalmic)** — end
  effectors disengage, instruments retract to a defined park
  position, force feedback to operator is preserved if the
  link allows
- **Rehabilitation** — patient-side mechanism unlocks to
  permit human intervention; force is reduced to
  zero-impedance mode
- **Pharmacy compounding** — current task aborts; partial
  preparations are flagged for waste-stream handling
- **Service robot** — mobile platform stops in place, lights
  alarm beacon, calls maintenance

The transition itself emits a high-priority alarm and an
intervention event. Re-entry to operational state requires
explicit operator acknowledgement plus a re-attestation
sequence.

## §5 Motion-telemetry pacing

Motion telemetry frequency varies by robot kind:

| Kind                              | Sample rate target | Bound on dropped samples |
|-----------------------------------|--------------------|--------------------------|
| Surgical (laparoscopic, ortho)    | 1000 Hz            | < 1% per minute          |
| Surgical (neurosurgical)          | 1000 Hz            | < 0.5% per minute        |
| Surgical (ophthalmic)             | 2000 Hz            | < 0.1% per minute        |
| Rehabilitation                    | 200 Hz             | < 1% per minute          |
| Pharmacy compounding              | 50 Hz              | < 5% per minute          |
| Service robot                     | 10 Hz              | best-effort              |

Telemetry is delivered to the boundary via a separate stream
from the control link so control bandwidth is preserved.
Buffer-overflow during connectivity loss triggers the
archive-upload path (PHASE 2 §4) on reconnection.

## §6 Audit chain

Every procedure-state mutation, intervention event, motion-
telemetry archive submission, alarm transition, calibration
record, and tele-link state change emits an AuditEvent.
Chain construction follows the same pattern as WIA-medical-iot
PHASE 3 §5.

For high-volume surgical procedures, the chain is sharded by
`procedureRef` hash prefix; sharding is itself audited so
re-sharding is detectable.

## §7 Time discipline

Robots use:

- **Surgical/rehabilitation** — PTPv2 IEEE 1588 to a master
  clock in the OR network for sub-millisecond synchronisation
- **Pharmacy/service** — NTPv4 stratum-3 to the boundary

A robot whose clock skew exceeds the deployment-declared
tolerance for the kind cannot start a new procedure;
in-progress procedures are flagged but not interrupted to
avoid harming the patient.

## §8 Manufacturer attestation channel

Manufacturers exchange post-market surveillance data with the
boundary on a dedicated channel:

- mTLS-bound JWTs with manufacturer GLN as `holderRef`
- Signed manufacturer-side reports referencing UDI-DI scope
- The boundary aggregates and anonymises before sharing
  (per WIA-medical-iot PHASE 4 §6 thresholds)

## §9 Replay protection

Same as WIA-medical-iot PHASE 3 §9 — Idempotency-Key for HTTP
endpoints; OSCORE sequence-number window for any constrained
satellite device on the robot.

## §10 Disaster-recovery

If the boundary loses connectivity or experiences a partial
failure during a procedure:

- Robot continues under operator control with local-link
  control (control-plane survives boundary loss)
- Telemetry buffers locally
- Boundary reconnection triggers archive-upload path
- Audit-chain reconstruction follows WIA-medical-data-privacy
  PHASE 3 disaster-recovery pattern; gaps are flagged for
  manual review

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Algorithm choices (informative)

| Concern           | Default                            | Notes                                      |
|-------------------|------------------------------------|--------------------------------------------|
| Token signing     | ES256                              | mTLS-bound (RFC 8705)                      |
| TLS               | 1.3 (RFC 8446)                     | hybrid groups when WIA-pq-crypto hybrid    |
| OSCORE AEAD       | AES-CCM-16-64-128                  | for constrained satellites                 |
| Audit hash        | SHA-256                            |                                            |
| Telemetry archive | zstd level 9                       | bandwidth-efficient lossless compression   |

## Annex B — Tele-surgery network requirements (informative)

| Concern                  | Requirement                                     |
|--------------------------|-------------------------------------------------|
| One-way latency          | ≤ 50 ms (typical)                                |
| Jitter                   | ≤ 5 ms                                          |
| Packet loss              | ≤ 0.1%                                          |
| Bandwidth                | ≥ 100 Mbps full-duplex per active surgeon       |
| Path redundancy          | ≥ 2 paths with diverse last-mile providers      |
| QoS marking              | DSCP EF on bearer                               |
| Encryption               | TLS 1.3 with hybrid groups (post-quantum ready) |
| Boundary monitoring      | continuous link-quality observations            |

## Annex C — Negative-test vectors (informative)

| Stimulus                                              | Expected outcome                                   |
|-------------------------------------------------------|---------------------------------------------------|
| Heartbeat missed > 200 ms                              | safe-state + `tele-link-loss` alarm + intervention event |
| Operator emergency-stop press                          | safe-state + alarm + intervention event           |
| Force-limit violation                                   | safe-state + high-priority alarm                  |
| Telemetry rate drops below kind target by > 5%         | flagged `telemetry-rate-degraded` annotation      |
| Robot clock skew beyond tolerance, new procedure        | refused with `clock-skew-exceeded`                |

## Annex D — Pre-procedure attestation chain (informative)

Before procedure-start the boundary executes a sequence of
verifications, each emitting an audit-chain entry:

1. Robot self-test report (signed by robot)
2. Calibration record verification (boundary-side)
3. End-effector inventory match against the planned tool list
4. Operator credential verification
5. Patient consent verification at WIA-medical-data-privacy
6. Imaging-system readiness if imaging cross-reference is bound
7. OR-network segmentation check
8. Tele-surgery link-quality observation if tele-procedure

A failure at any step blocks `$start` and surfaces the failing
verification in the response. Successful pre-procedure
attestation is itself a chain entry that auditors use to
confirm the procedure-start was preceded by all checks.

## Annex E — Manufacturer reporting cadence (informative)

| Reporting class                    | Cadence                           |
|------------------------------------|-----------------------------------|
| Routine operational summary         | Quarterly                         |
| Adverse event (FDA MDR / EU PSUR)   | Per regulatory deadline           |
| Field safety notice acknowledgement | On notice receipt                 |
| Recall scope query                  | On manufacturer request           |
| End-effector lifetime distribution  | Quarterly                         |
| Software-related issue (IEC 62304)  | Per problem-resolution cadence    |

The boundary publishes the schedule in the deployment policy.
Manufacturers receiving these reports verify signatures and
may post counter-signed acknowledgements that close the loop.

## Annex F — Tele-link backup activation (informative)

When the primary tele-link transitions to `degraded` or
`unusable`, the boundary attempts backup-link activation:

1. Attempt 1: secondary path on the same provider with
   different last-mile (typically diverse fibre route)
2. Attempt 2: tertiary path on a different provider
3. Attempt 3: 5G public-cellular bearer with TS 33.501
   security as last resort

If all three fail within the deployment-declared window
(typically 30 seconds), the boundary triggers safe-state
transition (§4) and surfaces a high-priority alarm.

The transition between primary and backup is itself an
intervention event so post-procedure review can examine
the link history.
