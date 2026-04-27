# WIA-military-robot PHASE 1 — Data Format Specification

**Standard:** WIA-military-robot
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable

This PHASE defines the canonical data format for military unmanned
ground / surface / sub-surface robotic systems (UGV/USV/UUV) records:
platform identification, mission plan, command-and-control link
state, sensor observations, weapon-release records (where authorised),
geofence evidence, autonomy-level declarations and human-on-the-loop
interaction, mishap records, and the cross-references that bind
these to the broader operational picture. Aerial platforms are
covered separately by WIA-military-drone; this PHASE covers ground,
surface (boats), and sub-surface (UUV) platforms with sufficient
overlap that the shapes are deliberately compatible.

References (CITATION-POLICY ALLOW only):
- STANAG 4677 — Dismounted Soldier Reference Architecture (for soldier-deployable robots)
- STANAG 4856 — Standardisation of Common Architecture for UGVs
- STANAG 4669 — UAV Test Procedures (informationally referenced for envelope tests)
- IEEE P1872 — Ontologies for Robotics and Automation
- IEC 61508 — Functional Safety of Electrical/Electronic/Programmable Electronic Safety-related Systems (referenced for autonomy-level safety integrity)
- ISO 13482 — Personal-care robots (referenced informationally for human-interaction zones)
- ISO 22166 — Modularity for Service Robots (informational)
- WGS-84 — geodetic reference frame
- IETF RFC 7515 (JWS), RFC 8259 (JSON), RFC 9162 (Certificate Transparency 2.0 pattern)
- WIA-military-drone — referenced for aerial-equivalent records
- WIA-missile-defense — referenced for cross-domain weapon-release coordination

---

## §1 Scope

This PHASE applies to systems that schedule, command, or report on
military unmanned ground vehicles (UGV), unmanned surface vessels
(USV), and unmanned underwater vehicles (UUV): EOD (explosive-
ordnance-disposal) robots, route-clearance robots, manned-unmanned
teaming platforms, autonomous logistics carriers, sentry / armed
robots, and minesweeping / sub-sea reconnaissance vehicles.

The standard is autonomy-level-aware: an implementation MUST declare
the autonomy level under which a given mission operates (per IEC 61508
SIL plus operator-on-loop / on-loop / off-loop / fully-autonomous
classification) so downstream consumers apply the correct interpretation
of operator-intervention windows and weapon-release authority.

In scope: platform identification, mission plan, link state, sensor
observation, weapon-release records (where authorised), geofence
evidence, autonomy-level declarations, mishap records. Out of scope:
aerial platforms (WIA-military-drone), kinetic-warhead intercept by
robot-launched munitions (WIA-missile-defense from intercept onward),
swarm-coordination protocols (separate WIA standard).

## §2 Platform identifier

Each platform carries:

| Field             | Source / Binding                                             |
|-------------------|--------------------------------------------------------------|
| `platformRef`     | URN of form `urn:wia:md-rob:platform:<authority>:<id>`       |
| `vendor`          | manufacturer of record                                       |
| `model`           | model designation                                            |
| `domain`          | one of `ugv`, `usv`, `uuv`                                   |
| `weightClass`     | (UGV) man-portable / SUGV / MUGV / TUGV per US Army size class |
| `mobilityProfile` | terrain / sea-state / depth-band the platform is rated for   |
| `payloadCapability` | closed enum subset (manipulator, sensor-suite, weapon, comms, EOD-disrupt) |
| `linkClasses`     | RF-LOS, mesh-relay, tethered-fibre, acoustic-modem (UUV)     |
| `firmware`        | firmware version + last update timestamp                     |
| `tlsClientCertFp` | mTLS client certificate fingerprint                          |
| `safetySIL`       | IEC 61508 Safety Integrity Level                             |

A platform not in the deployment's registry (PHASE 4 §1) is refused
at PHASE 2 §1 mission-plan intake.

## §3 Mission plan record

A mission plan binds a route to operational intent:

- `missionId` — URN of form `urn:wia:md-rob:mission:<authority>:<seq>`
- `platformRef` — assigned platform URN
- `missionType` — closed enum: `route-clearance`, `eod-investigation`,
  `eod-disrupt`, `sentry-patrol`, `logistics-resupply`, `recon-isr`,
  `minesweeping` (USV/UUV), `subsea-recon` (UUV), `training`,
  `range-test`
- `route` — sequence of waypoints; per-waypoint: WGS-84 lat/lon/alt,
  speed, allowed-deviation radius, timing window
- `geofence` — closed polygon (UGV/USV) or 3D volume (UUV) outside
  which the platform is forbidden
- `autonomyLevel` — closed enum: `tele-operated`, `operator-on-loop`,
  `operator-on-loop-with-supervised-autonomy`, `bounded-autonomy`,
  `fully-autonomous`
- `humanOnLoopMonitoring` — for `bounded-autonomy` or
  `fully-autonomous`: monitoring cadence + escalation thresholds
- `weaponLoadout` — (for armed missions) loadout manifest
- `releaseAuthority` — URN of operator authorised to issue weapon-
  release commands
- `lostLinkBehaviour` — `return-to-staging`, `loiter-at-last-waypoint`,
  `safe-shutdown-in-place`, `proceed-and-return`

Mission plans are signed by the operations authority. Re-tasking
in-flight emits a new plan record referencing the prior.

## §4 Link state record

Each link-state change emits:

- `linkStateId`, `platformRef`, `linkClass`, `linkProvider`,
  `state` (`acquired`/`tracking`/`degraded`/`lost`),
  `signalQualityBand`, `linkLatencyMs`, `transitionTimestamp`

UUVs have a special acoustic-modem link class with high-latency,
low-bandwidth characteristics; the deployment policy enumerates
acceptable acoustic-link tolerances.

## §5 Sensor observation record

Sensor observations follow:

- `obsId`, `platformRef`, `payloadRef`, `sensorType` (`eo`, `ir`,
  `lidar`, `radar`, `sonar`, `chemical-sniffer`, `radiological-sensor`),
  `geometry`, `targetGroundFootprint` (or `targetWaterColumn` for UUV
  / `targetLocation` for UGV), `observationTimestamp`,
  `observationProductRef`, `metadataRef`

For NBC/CBRNE-relevant sensors, observations cross-reference WIA-nbc-
defense PHASE 2 §1 ingest; the platform's sensor effectively becomes
a NBC sensor for that domain.

## §6 Weapon-release record (armed platforms)

For armed sentry / EOD-disrupt platforms:

- `releaseId`, `platformRef`, `missionRef`, `decisionRef`, `munitionType`
  (e.g., disrupt-charge for EOD, less-lethal munitions, escalation-
  ladder lethal munitions), `targetRef`, `aimpoint`, `releaseConditions`,
  `releaseTimestamp`, `dualSignatures` (operator + release authority)

Armed military robots tend to have stricter human-on-loop requirements
than UAVs because of the proximity to friendly forces and civilians;
the deployment policy enumerates which platforms are armed and which
RoE categories they may invoke.

## §7 Geofence evidence record

Per mission leg, the platform emits:

- `geofenceEvidenceId`, `platformRef`, `legRef`, `actualTrack[]`,
  `geofenceBreachEvents[]`

UUV geofence evidence carries 3D position (lat/lon/depth) instead of
2D ground-track. Persistent breaches escalate to mishap-investigation
workflow per §9.

## §8 Autonomy-level declaration record

Each mission carries a signed autonomy-level declaration:

- `declarationId`, `missionRef`, `declaredAutonomyLevel`,
  `humanOnLoopOperator` (URN), `escalationThresholds`,
  `safetyEnvelope` (per IEC 61508 SIL), `declaringAuthority`

The declaration is auditable; mission events that would exceed the
declared autonomy level (e.g., a `bounded-autonomy` platform attempts
a behaviour outside its declared envelope) trigger automatic
human-on-loop alert plus mishap-investigation flag.

## §9 Mishap record

A mishap (platform loss, unintended damage, friendly-fire, civilian
incident) triggers:

- `mishapId`, `platformRef`, `missionRef`, `mishapType`
  (`platform-loss`, `unintended-target-engagement`, `friendly-fire`,
  `civilian-injury`, `geofence-breach-with-consequence`,
  `autonomy-envelope-violation`),
- `incidentTimestamp`, `incidentLocation`, `priorAuditWindowRef`
  (audit-chain reference for the 30-minute window prior to the
  incident), `investigationLeadAuthority`, `investigationStatus`
  (`open`/`closed-with-finding`)

Mishap records are append-only; investigations append additional
entries with findings, lessons-learned, and corrective-action
references. A 30-day investigation window is the default; extensions
require operations-command sign-off.

## Annex A — Cross-domain references (informative)

| Reference                  | Use site                                                    |
|----------------------------|-------------------------------------------------------------|
| WIA-military-drone         | aerial-equivalent records and shared GCS infrastructure     |
| WIA-missile-defense        | cross-domain weapon-release coordination                    |
| WIA-nbc-defense            | NBC-sensor observations from chemical/radiological sniffers |
| WIA-military-communication | cross-coalition transport                                   |
| WIA-medical-data-privacy   | medical-evacuation casualty integration                     |

## Annex B — Conformance disclosure

Implementations declare per-section conformance in their published
capability document. Sections marked `partial` or `excluded`
reference the deployment policy. A deployment that is `partial` or
`excluded` on §3 (Mission plan), §6 (Weapon release), §8 (Autonomy
declaration), or §9 (Mishap record) is non-conformant overall.

## Annex C — Conformance levels

| Level     | Scope                                                            |
|-----------|------------------------------------------------------------------|
| Surface   | data formats accepted; self-attested                              |
| Verified  | annual third-party audit (STANAG 4856 conformance + RoE)          |
| Anchored  | continuous evidence package per audit chain transparency          |

## Annex D — Versioning and deprecation

Versioning follows Semantic Versioning 2.0.0. Coalition operations
prefer staged migration. Deprecation enters a 12-month sunset window
with migration notes recorded in the audit chain.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex E — Worked autonomy-level escalation (informative)

A `bounded-autonomy` route-clearance UGV encounters an obstacle:

```json
{
  "exceedanceId": "urn:wia:md-rob:exceedance:rcv-bot-3:x-91a7",
  "platformRef": "urn:wia:md-rob:platform:rok-army:rcv-bot-3",
  "missionRef": "urn:wia:md-rob:mission:rok-army:m-91a7",
  "envelopeFieldExceeded": "lateral-deviation",
  "reportedValue": "12 m",
  "envelopeBound": "5 m",
  "exceedanceTimestamp": "2026-04-27T09:31:14+09:00",
  "humanOnLoopAlertSent": true
}
```

The operator either authorises an envelope-extension for this leg
(signed via PHASE 2 §7 supplementary autonomy declaration) or aborts
the mission. Both decisions are recorded; the exceedance record is
preserved for post-mission review.

## Annex F — Time-precision worked example

Per-record timestamp precision varies with platform domain: UGV/USV
carry millisecond precision; UUV carry millisecond + INS-drift
annotation referencing the most-recent GNSS-disciplined reference;
mishap timestamps are always millisecond.

## Annex G — Worked mission-plan record (informative)

A typical EOD-investigation mission for a man-portable UGV:

```json
{
  "missionId": "urn:wia:md-rob:mission:rok-army:m-91a7-2026-04-27",
  "platformRef": "urn:wia:md-rob:platform:rok-army:eod-bot-12",
  "missionType": "eod-investigation",
  "route": [
    {"waypointId": "wp-staging", "lat": 37.500, "lon": 127.000, "alt": 0, "speed": 0.5, "legDuration": 30},
    {"waypointId": "wp-approach", "lat": 37.501, "lon": 127.001, "alt": 0, "speed": 0.5, "legDuration": 60},
    {"waypointId": "wp-investigate", "lat": 37.5012, "lon": 127.0012, "alt": 0, "speed": 0.1, "legDuration": 600}
  ],
  "geofence": {"type": "Polygon", "coordinates": [[[127.000, 37.500],[127.002, 37.500],[127.002, 37.502],[127.000, 37.502],[127.000, 37.500]]]},
  "autonomyLevel": "operator-on-loop",
  "rulesOfEngagement": "urn:wia:md-rob:roe:rok-army.adcc:eod-rules-2026",
  "lostLinkBehaviour": "safe-shutdown-in-place",
  "weaponLoadout": [{"type": "disrupt-charge", "count": 1}],
  "releaseAuthority": "urn:wia:md-rob:operator:rok-army:eod-team-lead-7"
}
```

The operations authority signs the plan; the boundary verifies the
platform's mobility profile permits the route, the geofence is closed,
the autonomy level is below the platform's SIL, and the operator is
in the deployment's release-authority registry.

## Annex H — UUV-specific record extensions (informative)

UUV records carry additional fields:

- `inferredFromInsAtTimeRef` — RFC 3339 reference timestamp at which
  the INS was last GNSS-disciplined (or acoustic-time-pulse-disciplined)
- `insDriftEstimateMetres` — estimated drift since the reference
- `acousticTimePulseId` — most-recent acoustic time-pulse received
- `depthMetres` — current depth (positive numeric)
- `pressureBar` — measured pressure at depth
- `hullIntegrityStatus` — `nominal`, `warning`, `critical`

A UUV operating below the GNSS-fix tolerance must include these
fields on every state event so post-mission reconstruction can
correct INS drift.

## Annex I — Loadout inventory record

Armed platforms carry a loadout inventory linked to their mission plan; per-munition fields include type, count, fusing options, and per-munition serial number where applicable. Inventory reconciliation is performed pre-mission and post-mission; unaccounted munitions trigger a safety-incident report.

## Annex J — Decommissioning-platform record
Decommissioned platforms emit a final record naming the successor (if any) plus the final-mission-hour count, final calibration, and audit-retention plan. The record is signed by both outgoing and (if transferred) incoming custodians.
