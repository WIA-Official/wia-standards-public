# WIA-military-robot PHASE 4 — Integration Specification

**Standard:** WIA-military-robot
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable

This PHASE describes how a military-robot deployment integrates the
data, APIs, and protocols from PHASEs 1–3 with the operational
picture: platform fleet management, GCS integration, mission planner,
manned-unmanned teaming, NBC-defense bridge, EOD coordination, UUV
support-ship integration, fire-control consumers, and coalition
exchange. It is non-prescriptive about specific vendors; it specifies
the integration *contracts* a deployment must satisfy.

References (CITATION-POLICY ALLOW only):
- STANAG 4856 — UGV common architecture
- STANAG 4677 — Dismounted Soldier Reference Architecture
- IEC 61508 — Safety Integrity Levels
- WIA-military-drone (PHASE 1–4) — for shared GCS infrastructure
- WIA-missile-defense (PHASE 1–4) — for cross-domain weapon-release
- WIA-nbc-defense (PHASE 1–4) — for NBC-sensor integration
- WIA-medical-data-privacy — for casualty integration

---

## §1 Platform fleet registry

The deployment maintains a registry of every fielded platform:

- `platformRef` — URN
- vendor, model, serial, domain (UGV/USV/UUV)
- weight class, mobility profile
- payload capability set
- supported link classes
- firmware version + last update date
- TLS client certificate fingerprint and expiry
- maintenance schedule + last-mission-hour count
- safety SIL (per IEC 61508)
- operating-area policy (geographic + autonomy-level scope)

A platform not in the registry, with expired certificate, with stale
firmware, or whose declared mission autonomy exceeds the platform's
SIL is refused at PHASE 2 §1.

## §2 GCS integration

GCS integration follows STANAG 4856 (UGV/USV) or STANAG 4677
(soldier-deployable). Multi-platform GCSs use the same authentication
and audit-chain shape as WIA-military-drone PHASE 4 §2; per-operator
authentication is mandatory for accountability.

The deployment may share GCS hardware and operator workflows between
UAS and ground-robot operations; the boundary distinguishes by
`platformRef.domain` so audit attribution remains correct.

## §3 Mission planner

The mission planner is a separate service:

- Route construction with platform-specific mobility profiles
- Geofence drafting (2D for ground/surface, 3D for UUV)
- Autonomy-level declaration drafting
- Lost-link behaviour selection
- Loadout planning for armed missions

The planner submits via PHASE 2 §1; the boundary's role is gatekeeping
and audit.

## §4 Manned-unmanned teaming (MUM-T)

When a robot operates as part of a manned formation:

- The lead manned platform's commander is the operator-on-loop
- The robot's mission record references the manned platform's
  mission record
- Sensor observations flow into the manned platform's situational
  awareness display (typically via a shared C2 system)
- Weapon-release authority for the robot defers to the manned
  commander's release authority

The boundary verifies the manned-mission cross-reference exists
before accepting the robot's mission plan; orphan robot missions
require explicit authorisation outside the MUM-T frame.

## §5 NBC-defense bridge

When a robot's payload includes NBC sensors:

- Observations cross-reference WIA-nbc-defense PHASE 2 §1 ingest
- The robot's chemical/radiological sniffer becomes a WIA-nbc-defense
  sensor for purposes of plume-modelling and threat-assessment
- The sensor's calibration is recorded in both registries (NBC
  fleet + robot fleet)

## §6 EOD coordination

EOD operations have specific integration:

- Mission plans declare the suspect-device location and category
- The robot's manipulator and disrupt-charge inventory are tracked
- Disrupt-charge fire requires dual signatures (operator + EOD lead)
- Post-disrupt sensor sweep verifies neutralisation; the result is
  recorded as the mission's outcome

## §7 UUV support-ship integration

UUV deployments integrate with their surface support ship:

- The support ship hosts the GCS, the acoustic-modem, the launch
  & recovery system
- UUV and support ship time-discipline syncs at every acoustic ping
- Support ship's position acts as the UUV's reference frame for
  navigation reconstruction
- Recovery records reference the support ship's recovery operation
  identifier

Loss of the support ship (during operations) requires emergency
recovery procedures defined in deployment policy; the UUV's
fully-autonomous recovery behaviour engages.

## §8 Operational SLAs

| Concern                                          | Default SLA              |
|--------------------------------------------------|--------------------------|
| Mission-plan submission p95 latency              | ≤ 200 ms                 |
| Platform state stream sample latency             | ≤ 100 ms (ground/surface)|
|                                                  | per acoustic window (UUV)|
| Weapon-release authorisation turnaround          | ≤ 500 ms                 |
| Sensor observation publication                   | ≤ 1 s after sensor frame |
| Lost-link detection threshold (ground)           | ≤ 10 s                   |
| Audit chain entry available after operation      | ≤ 1 s (live link); per acoustic window (UUV) |
| Mishap response — token freeze                   | ≤ 5 s                    |
| Mishap response — chain snapshot                 | ≤ 60 s                   |

## §9 Acceptance criteria

A deployment claims conformance when:

1. Every fielded platform is in the registry with current certificate,
   firmware, and SIL declaration
2. Every mission has a matching audit chain entry with verifiable
   inclusion proof
3. Every weapon-release event has dual signatures plus autonomy-level
   compatibility check
4. Geofence-evidence records are on file for every mission
5. Lost-link events have recovery records or mishap-investigation
   evidence
6. UUV missions have INS-drift records consistent with the deployment's
   policy
7. Mishap investigations close within 30 days of incident (or have
   formal extension authorisations)
8. Cross-coalition releases have both release-authority signatures
9. Quarterly compliance report has no integrity-check failures

## Annex A — Common pitfalls (informative)

- **Autonomy declaration drift** — operator changes mission scope
  in-flight without re-declaring autonomy; the platform's onboard
  supervisor refuses, but the operator's expectation drifts. The
  GCS UI MUST surface the active declaration prominently
- **Acoustic-modem desync (UUV)** — multi-UUV ops with overlapping
  acoustic ranges experience modem collision; the deployment's TDMA
  schedule MUST be designed for the worst-case overlap
- **Manned-unmanned cross-reference orphaning** — manned mission ends
  while robot mission continues; the boundary surfaces the orphan
  and the operator must either re-authorise standalone or recover
- **EOD disrupt-charge inventory** — inventory must reconcile after
  every mission; an unaccounted charge is itself a safety incident

## Annex B — Decommissioning (informative)

When a platform is decommissioned: outstanding missions complete or
transfer, the system is removed from the registry, final mission-hour
count is recorded, audit records are preserved per deployment retention
rules. The decommissioning manifest is itself an audit event.

## Annex C — Quarterly compliance report (informative)

The report covers total missions by type and outcome, sensor-observation
volumes, weapon-release events with authorisation chain, autonomy-
level declarations and exceedances, geofence-breach incidence,
lost-link events and recovery rates, mishap investigations (open /
closed within 30d / closed >30d), federation peer activity, and
audit-chain integrity check results. The report is signed and is
itself in scope for the audit chain.

## Annex D — Worked MUM-T sequence (informative)

1. Manned commander launches an MUM-T mission incorporating an EOD
   robot
2. Manned mission plan references the robot's mission plan; both
   are recorded in the boundary
3. Robot proceeds along the cleared route under operator-on-loop
   autonomy; sensor observations flow to the commander's display
4. Manned commander authorises a disrupt against a suspect device;
   dual signatures are committed
5. Robot acts; outcome is recorded; manned commander reviews
6. Both missions close; the audit chains are linked

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex E — UUV operations notes (informative)

UUV deployments differ from UGV/USV in several integration points:

- **Acoustic-modem bandwidth budget** — typically 1-10 kbps; design
  the deployment's PHASE 2 traffic to fit within the budget
- **GNSS denied** — the UUV cannot fix position underwater except via
  surface, USBL/LBL acoustic positioning, or pre-mapped landmark
  matching; the deployment policy specifies acceptable INS drift
- **Pressure hull integrity** — incidents (water ingress, hull-strike)
  trigger mishap workflow even when the platform recovers; investigation
  retrieves prior-window audit chain
- **Recovery weather windows** — surface ship + UUV recovery is sea-
  state-dependent; mission planning should account for weather windows

## Annex F — Versioning and deprecation (informative)

Versioning follows Semantic Versioning 2.0.0. STANAG 4856 / 4677
versions bump independently from this PHASE; the deployment policy
maps each PHASE version to the STANAG version it honours.
Deprecation enters a 12-month sunset window with migration notes
recorded in the audit chain.

## Annex G — Lessons-learned register (informative)

Recurring lessons-learned across robot operations:

- **Tether snag** (EOD): wired-fibre tethers snag on debris; the
  platform's tether-management system MUST detect snag and either
  retract or signal operator
- **Manipulator drift** (EOD): hydraulic manipulators drift from
  commanded position over time; per-mission pre-flight calibration
  reduces the drift impact
- **Sea-state recovery window** (USV/UUV): the launch & recovery
  system has a sea-state ceiling; missions extending into degrading
  weather must include alternate recovery plans
- **Acoustic crosstalk** (UUV multi-platform ops): TDMA scheduling
  with margin for propagation-delay variance prevents collisions

## Annex H — Cross-platform identifier reuse

A platform's URN is preserved across firmware updates. Major hardware refurbishment that changes the platform's serial number or safety SIL issues a new URN; the prior URN is retired with a recorded successor reference.
