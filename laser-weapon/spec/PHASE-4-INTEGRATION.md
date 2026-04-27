# WIA-laser-weapon PHASE 4 — Integration Specification

**Standard:** WIA-laser-weapon
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable

This PHASE describes how a laser-weapon deployment integrates the
data, APIs, and protocols from PHASEs 1–3 with the operational
picture: weapon-system fleet management, missile-defence fusion-
engine integration, eye-safety-zone planner, atmospheric-monitoring
sensors, range-control infrastructure, and coalition exchange. It
is non-prescriptive about specific vendors; it specifies the
integration *contracts* a deployment must satisfy.

References (CITATION-POLICY ALLOW only):
- IEC 60825-1, ANSI Z136.1 — eye-safety
- ISO 11146 / 13694 — beam-quality / power-density measurement
- WIA-missile-defense (PHASE 1–4) — for engagement decision intake
- WIA-military-communication (PHASE 1–4) — for cross-domain transport
- WIA-nbc-defense (PHASE 1–4) — for casualty-coordination when an
  intercept produces NBC consequences (e.g., chem-warhead disabled)

---

## §1 Weapon-system fleet registry

The deployment maintains a registry of every fielded laser weapon:

- `wsRef` — URN
- vendor, model, serial number
- nominal power band, wavelength band
- aperture diameter, gimbal type, IEC 60825-1 class
- firmware version, last firmware update
- TLS client certificate fingerprint and expiry
- calibration status (M² baseline, pointing-stability baseline,
  power-meter calibration date, calibrating laboratory)
- operating-area policy (geographic + RoE scope)

A weapon system not in the registry, or with expired calibration or
expired certificate, is refused at PHASE 2 §1 engagement intake.

## §2 Missile-defence fusion-engine integration

Engagement decisions originate at WIA-missile-defense PHASE 2 §4.
The integration contract:

- Fusion engine forwards the decision to the laser-weapon boundary
  via PHASE 2 §1 `/commands/engage`
- Boundary cross-references the WIA-missile-defense decision,
  applies the laser-specific RoE check (counter-UAS, C-RAM, etc.),
  runs eye-safety-zone check, then issues fire instruction
- Outcome (PHASE 1 §6 weapon-status state-transition `success` /
  `miss`) propagates back to fusion engine for the
  WIA-missile-defense PHASE 1 §8 outcome record

This bridge ties two audit chains across the engagement lifecycle.

## §3 Eye-safety-zone planner

The eye-safety-zone planner is a separate service that owns:

- Human-rated zones (NOHD-bounded zones around populated areas,
  friendly-troop positions, civil aircraft corridors)
- Time-bounded zones (e.g., a closed range during a planned test
  window vs. open to civil traffic outside the window)
- Atmospheric-corrected NOHD recomputation (high-haze atmosphere
  has shorter NOHD than clear)

The planner exposes a signed query API; the boundary calls it on
every engagement command. A planner offline beyond cache window
suspends fire authorisation; the deployment policy decides whether
cached zones honoured for an emergency window or fire is refused
outright.

## §4 Atmospheric-monitoring sensors

Atmospheric sensors (visibility, scintillometer for Cn², weather
station) feed the boundary at the engagement location:

- Sample rate: 1 Hz (visibility), 10 Hz (Cn²), 1 Hz (weather)
- Sensors signed; boundary refuses unsigned atmospheric records
- Atmospheric records pair with each emission for post-shot
  beam-quality interpretation (PHASE 1 §9)

Sensor failure drops the engagement's atmospheric record to
`unknown` channel; engagement permitted but flagged for safety
review.

## §5 Range-control infrastructure (test deployments)

Range-test deployments integrate with range-control:

- Range-control issues range-test schedules (PHASE 2 §7)
- Range-control infrastructure (range-clear sensors, observer
  stations, photodetector arrays) feeds the deployment's beam-
  quality and atmospheric streams
- Test outcomes (M², pointing-stability, achieved energy at target)
  are recorded in the deployment's audit chain plus the range's
  test-results database

A range-test deployment SHOULD NOT engage live targets; range-test
RoE category is the only authorised category for test-only weapon
systems.

## §6 Coolant logistics

Liquid-cooled HEL systems require coolant logistics:

- Coolant inventory (per-unit volume, per-station refill capacity)
- Refill schedule (coolant gets contaminated; routine refill
  cadence per vendor specification)
- Reservoir-low alerts (PHASE 1 §6 weapon-status `coolantState`
  drives a maintenance request)

Coolant operations are out of audit-chain scope but the resulting
weapon-status records are auditable.

## §7 Quarterly compliance report

The boundary emits a quarterly compliance report:

- Total engagements by category (counter-UAS, C-RAM, etc.)
- Eye-safety-zone-check pass / violation rates
- M² and pointing-stability trends per weapon system
- Thermal-limit-exceeded events (count, mean time to recover)
- Atmospheric-monitoring sensor uptime
- RoE authorisation usage (which RoEs invoked how often)
- Cross-coalition releases by federation peer
- Audit-chain integrity check results

The report is signed and is itself in scope for the audit chain.

## §8 Operational SLAs

| Concern                                          | Default SLA              |
|--------------------------------------------------|--------------------------|
| Engagement-command intake p95 latency            | ≤ 50 ms                  |
| Eye-safety-zone-check turnaround                 | ≤ 100 ms                 |
| Beam-emission lifecycle event publication        | ≤ 50 ms after event      |
| Thermal stream sample latency                    | ≤ 100 ms (10 Hz stream)  |
| Atmospheric-monitoring sensor refresh            | per sensor sample rate   |
| Audit chain entry available after operation      | ≤ 1 s                    |

## §9 Acceptance criteria

A deployment claims conformance when:

1. Every fielded weapon system is in the registry with current
   calibration and certificate
2. Every engagement command in the past quarter has a matching
   audit chain entry
3. Eye-safety-zone-check evidence is on file for every engagement
4. Atmospheric records pair with every emission (or `unknown`
   flagged with explanation)
5. M² trends are within the deployment's operational threshold
6. Cross-coalition releases have both release-authority signatures
7. Quarterly compliance report has no integrity-check failures

## Annex A — Common pitfalls (informative)

- **Calibration drift** — M² and pointing-stability baselines drift
  over time; deployments SHOULD recalibrate at vendor-specified
  intervals (typically 6 months)
- **Atmospheric-sensor lag** — high-haze events develop faster than
  some scintillometers refresh; deployments SHOULD pair multiple
  sensor types so a stale reading is detectable
- **Coolant reservoir starvation mid-engagement** — engagement
  authorities relying on full-power emission against extended dwell
  threats may exhaust coolant; the deployment SHOULD pre-warm and
  pre-fill before high-tempo operations
- **NOHD mis-estimation in haze** — atmospheric scattering shortens
  effective NOHD; the eye-safety-zone planner MUST incorporate
  current atmospheric observation, not nominal-clear-air NOHD
弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex B — Decommissioning (informative)

When a weapon system is decommissioned: outstanding engagements close,
the system transitions to `cold` and is removed from the registry,
final calibration is recorded, the audit chain reflects the
decommissioning. Coolant logistics close out per vendor procedures.

The decommissioning manifest is itself an audit event in the chain,
signed by both outgoing custodian and (if transferred) incoming
custodian.

## Annex C — Lessons-learned register (informative)

- **NOHD recomputation cadence** — atmospheric haze events develop
  inside one minute; recomputing NOHD slower than that misses brief
  hazardous windows
- **Pointing-stability under wind** — gimbal jitter increases under
  high wind; the deployment SHOULD throttle full-power emissions in
  high-wind weather to preserve M²
- **Coolant reservoir level vs. demand** — extended dwell-time
  threats (e.g., hardened C-RAM target) consume coolant faster than
  expected; the deployment SHOULD pre-fill prior to high-tempo
  operations

## Annex D — Operational SLAs (informative)

| Concern                                          | Default SLA              |
|--------------------------------------------------|--------------------------|
| Engagement-command intake p95 latency            | ≤ 50 ms                  |
| Eye-safety-zone-check turnaround                 | ≤ 100 ms                 |
| Beam-emission lifecycle event publication        | ≤ 50 ms after event      |
| Thermal stream sample latency                    | ≤ 100 ms (10 Hz stream)  |
| Atmospheric-monitoring sensor refresh            | per sensor sample rate   |
| Audit chain entry available after operation      | ≤ 1 s                    |
| RoE authorisation-cache refresh                  | ≤ 5 minutes              |
| Federation manifest expiry alert lead time       | ≥ 30 days                |

Tighter SLAs negotiable per deployment; loosening requires
operational sign-off.

## Annex E — Quarterly compliance report (informative)

The boundary emits a quarterly compliance report covering:

- Total engagements by category and outcome
- Eye-safety-zone-check pass / violation rates
- M² and pointing-stability per-weapon-system trends
- Thermal-limit-exceeded events
- Coolant logistics SLA adherence
- Atmospheric-sensor uptime by location
- Cross-coalition releases by federation peer
- Audit-chain integrity check results

The report is signed and is itself in scope for the audit chain.

## Annex F — Acceptance criteria (informative, restated)

A deployment claims conformance when:

1. Every fielded weapon system is in the registry with current
   calibration (M² baseline, pointing-stability baseline,
   power-meter calibration, IEC 60825-1 certification)
2. Every engagement command in the past quarter has a matching
   audit chain entry with verifiable inclusion proof
3. Eye-safety-zone-check evidence is on file for every engagement
4. Atmospheric records pair with every emission (or `unknown`
   flagged with explanation)
5. M² and pointing-stability trends are within the deployment's
   operational threshold per weapon system
6. Cross-coalition releases have both release-authority signatures
7. Range-test deployments have current range-control authorisations
   for every test in the period
8. Quarterly compliance report has no integrity-check failures

A deployment failing any of these reports the gap in its compliance
package rather than concealing it.

## Annex G — Versioning and deprecation (informative)

Versioning follows Semantic Versioning 2.0.0. Coalition operations
prefer staged migration: a major-version bump rolls out at one
deployment per quarter so partners adapt incrementally. Deprecated
versions enter a 12-month sunset window during which the registry
marks them deprecated and surfaces migration notes in the audit chain.
Patch-level errata are issued without a deprecation window because
they do not change normative behaviour.

## Annex H — Decommissioning (informative)

When a weapon system is decommissioned: outstanding engagements close
or transfer to a successor weapon system, the system transitions to
`cold` and is removed from the registry, final calibration is recorded,
the audit chain reflects the decommissioning. Coolant logistics close
out per vendor procedures.

## Annex I — Common pitfalls (extension)

- **Pointing-stability under wind** — gimbal jitter increases under high wind; deployments SHOULD throttle full-power emissions to preserve M².
- **Aperture-power vs. on-target-intensity drift** — aperture power may be in spec while on-target intensity falls due to atmospheric loss; pair with the channel record before declaring a mission-kill miss.

## Annex J — Clock discipline

Weapon-system clocks discipline to GNSS PPS; failure modes per PHASE 3 §6.
