# WIA-military-drone PHASE 1 — Data Format Specification

**Standard:** WIA-military-drone
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable

This PHASE defines the canonical data format for military unmanned
aircraft system (UAS) records: airframe identification, mission
plan, command-and-control link state, payload event records,
sensor observations, weapon-release records (where authorised),
geofence and rules-of-engagement evidence, link-loss/lost-link
behaviour, and the cross-references that bind these to the broader
fire-control / ISR picture. The shape interoperates with allied
tactical-data-link standards (STANAG 4609, STANAG 4586, MIL-STD-6016
J-series) so a coalition deployment does not require parallel data
models.

References (CITATION-POLICY ALLOW only):
- STANAG 4586 — Standard Interfaces of UAV Control System (UCS) for NATO UAV Interoperability
- STANAG 4609 — NATO Digital Motion Imagery Standard (MISB ST 0601, MISB ST 0902 KLV metadata)
- STANAG 4670 — Recommended Guidance for the Training of UAS Pilots
- STANAG 5516 — Tactical Data Link 16 (J-series messages, J3.x track, J11.x weapon-direction)
- MISB ST 0601 — UAS Datalink Local Set (KLV metadata)
- MISB ST 0903 — Video Moving Target Indicator and Track Metadata
- MIL-STD-6016 — J-series message realisation (US)
- ICAO Annex 2 — Rules of the Air; ICAO Doc 4444 — air-traffic management procedures (for civil-airspace overlap)
- WGS-84 — geodetic reference frame
- ISO 21384-3 — UAS operational procedures (informational)
- IETF RFC 7515 (JWS), RFC 8259 (JSON), RFC 9162 (Certificate Transparency 2.0 pattern)

---

## §1 Scope

This PHASE applies to systems that schedule, command, or report on
military UAS missions: ISR (intelligence/surveillance/reconnaissance),
strike, communications relay, electronic warfare, and signals
intelligence platforms operating from MAV (micro) through HALE (high-
altitude long-endurance). It addresses the *shape* of operational
records; protocols for transport are addressed in PHASE 3, and
integration with broader C2 in PHASE 4.

The standard is rules-of-engagement-aware: an implementation MUST
declare the controlling RoE under which its records originate
(coalition, national, training-range, civil-airspace permit) so
downstream consumers apply the correct interpretation of weapons-
release authority and target-class authorisation.

In scope: airframe identification, mission plan and route, C2 link
state, payload event records (gimbal pointing, sensor mode), sensor
observation records (full-motion video metadata, signals intelligence
digests, hyperspectral pull-outs), weapon-release records (when
authorised), geofence evidence, link-loss/lost-link behaviour,
recovery records. Out of scope: airframe internal flight-control
firmware (vendor-specific), kinetic intercept by a UAS-launched
munition (handled by WIA-missile-defense from intercept onward),
swarm-coordination protocols (separate WIA standard).

## §2 Airframe identifier

Each UAS airframe carries a stable identifier:

| Field             | Source / Binding                                            |
|-------------------|-------------------------------------------------------------|
| `airframeRef`     | URN of form `urn:wia:md-uas:airframe:<authority>:<tail>`   |
| `vendor`          | manufacturer of record                                      |
| `model`           | model designation                                           |
| `mtow`            | Maximum Take-Off Weight band (MAV / SUAS / Tactical / MALE / HALE) |
| `endurance`       | nominal endurance band (≤ 1h / 1-4h / 4-12h / 12-24h / >24h) |
| `payloadCapability`| closed enum subset: ISR-EO/IR, ISR-SAR, ISR-SIGINT, comms-relay, strike, EW |
| `c2LinkClasses`   | supported link classes (LOS RF, BLOS-SATCOM, mesh-relay)   |
| `flightControlFirmware` | firmware version + last update timestamp              |
| `tlsClientCertFp` | mTLS client certificate fingerprint                         |
| `eyeSafetyClass`  | (when carrying a laser designator) IEC 60825-1 class        |

An airframe not in the deployment's registry (PHASE 4 §1) is refused
at PHASE 2 §1 mission-plan intake. Registry changes are auditable.

## §3 Mission plan record

A mission plan binds a route to operational intent:

- `missionId` — URN of form `urn:wia:md-uas:mission:<authority>:<seq>`
- `airframeRef` — assigned airframe URN
- `missionType` — closed enum: `isr-patrol`, `isr-named-target`,
  `strike-prebriefed`, `strike-time-sensitive`, `comms-relay`,
  `ew-jamming`, `sigint-collection`, `training`, `range-test`
- `route` — sequence of waypoints; each waypoint: WGS-84 lat/lon/alt,
  speed, leg-duration, turn-radius, mandatory-altitude bands per ICAO
  airspace
- `geofence` — closed polygon outside which the airframe is forbidden
  to operate (combined with no-fly zones from civil airspace)
- `rulesOfEngagement` — RoE authorisation reference (URN) that the
  mission operates under
- `weaponLoadout` — (for strike missions) carried-munition manifest
  with type, count, fusing options
- `releaseAuthority` — URN of the operator authorised to issue
  weapon-release commands
- `lostLinkBehaviour` — closed enum: `return-to-base`, `loiter-at-last-waypoint`,
  `proceed-and-return`, `ditch-immediately`
- `civilAirspaceCoordinationRef` — (when airspace overlaps civil)
  URN of the ICAO-aligned coordination message issued to ATC

Mission plans are signed by the operations authority. Re-planning
in flight (re-task) emits a new plan record referencing the prior;
the prior plan is preserved for audit.

## §4 C2 link state record

Every change in C2 link state emits a record:

- `linkStateId` — URN
- `airframeRef` — airframe
- `linkClass` — `los-rf` / `blos-satcom` / `mesh-relay`
- `linkProvider` — vendor/system providing the link (e.g., Inmarsat,
  IRIDIUM, KMilSatCom, UAV-tactical-link)
- `state` — `acquired`, `tracking`, `degraded`, `lost`
- `signalQualityBand` — closed enum: `strong`, `nominal`, `marginal`,
  `lost`
- `linkLatencyMs` — measured one-way latency band (or numeric where
  measurable)
- `transitionTimestamp` — RFC 3339 with offset

Link-state changes drive the deployment's operational picture
(operator alerts) and the airframe's autonomous behaviours (link-loss
governs lost-link behaviour from §3).

## §5 Sensor observation record (MISB-aligned)

ISR sensor observations follow MISB ST 0601 KLV-metadata layout
projected into JSON for the boundary:

- `obsId` — URN
- `airframeRef` — emitting airframe
- `payloadRef` — specific payload (gimbal/sensor) on the airframe
- `sensorType` — `eo` (electro-optical), `ir` (infrared), `sar`
  (synthetic-aperture radar), `hsi` (hyperspectral imager),
  `lidar`, `sigint-comint`, `sigint-elint`
- `geometry` — gimbal pointing (roll/pitch/yaw at observation time),
  airframe state (lat/lon/alt/heading/speed)
- `targetGroundFootprint` — WGS-84 polygon of the ground footprint
  at observation time
- `observationTimestamp` — RFC 3339 with offset (sub-second precision
  for tactical correlation)
- `observationProductRef` — URI of the underlying product (full-motion
  video segment, SAR image, hyperspectral cube, COMINT digest)
- `klvMetadataRef` — for MISB ST 0601 KLV streams, URI of the per-
  frame KLV metadata file

For MTI (moving-target-indicator) observations, an additional
`mtiTrack` block carries per-track range/range-rate per MISB ST 0903.

## §6 Weapon-release record

When a strike-authorised UAS releases a munition:

- `releaseId` — URN
- `airframeRef` — releasing airframe
- `missionRef` — mission plan
- `decisionRef` — engagement decision (often delegated from
  WIA-missile-defense PHASE 1 §6, but UAS releases against pre-briefed
  named targets carry a mission-internal decision URN)
- `munitionType` — coded munition designation (e.g.,
  AGM-114, SDB, brimstone, GBU-39)
- `munitionCount` — count of munitions released in this event
- `targetRef` — target URN (track or named target)
- `aimpointGeometry` — WGS-84 lat/lon/alt of intended impact
- `releaseConditions` — speed, altitude, attitude at release
- `releaseTimestamp` — RFC 3339 with offset (microsecond precision
  for fire-control correlation)
- `releaseAuthorityCertificate` — JWS asserting release authorisation

A release event is signed by the airframe's signing key and by the
release authority's signing key (dual-signature). A release without
both signatures is treated as an unauthorised release and triggers
the deployment's safety-override workflow.

## §7 Geofence evidence record

For each leg of a mission, the airframe records geofence compliance:

- `geofenceEvidenceId` — URN
- `airframeRef` — airframe
- `legRef` — mission-leg reference
- `actualTrack` — sampled WGS-84 positions (sample rate per
  deployment, typically 1 Hz to 10 Hz)
- `geofenceBreachEvents[]` — empty if compliant; otherwise per
  breach: timestamp, position, breach type (`lateral`, `altitude`,
  `temporal`), resolution (`auto-corrected`, `operator-corrected`,
  `mission-aborted`)

Geofence evidence is reviewed at mission end; persistent breaches
trigger flight-controller diagnostic and operator review.

## §8 Lost-link / link-loss event

When C2 link is lost beyond the deployment's threshold:

- `lostLinkId` — URN
- `airframeRef` — airframe
- `lostFromTimestamp` — RFC 3339 with offset
- `lostUntilTimestamp` — RFC 3339 with offset (or `null` if recovered)
- `behaviourTriggered` — actual behaviour invoked (per mission-plan §3)
- `recoveryEvidence` — link-state record at recovery (if recovered)
- `mishapFlag` — boolean; true if the airframe was lost or
  significantly damaged during the lost-link window

Lost-link events are reviewed in operator and engineering meetings
within 72 hours. A mishap-flagged event triggers a formal incident-
response workflow.

## §9 Recovery / landing record

Mission end is recorded:

- `recoveryId` — URN
- `airframeRef` — airframe
- `missionRef` — completed mission
- `recoveryType` — `runway-landing`, `vertical-landing`,
  `parachute-recovery`, `arrested-recovery`, `mishap`
- `recoveryLocation` — WGS-84 lat/lon/alt
- `flightDurationSeconds` — total airborne time
- `fuelRemainingPct` — percent of mission-start fuel remaining
- `payloadStatus` — payload state at recovery (intact / damaged /
  expended)
- `airframeCondition` — operational condition assessment

Recovery records close the mission's audit chain. A `mishap`
recovery triggers the deployment's mishap investigation procedure
including preservation of relevant audit records and physical
evidence.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Cross-domain references (informative)

| Reference                       | Use site                                                |
|---------------------------------|---------------------------------------------------------|
| WIA-missile-defense             | engagement-decision cross-reference for strike releases |
| WIA-military-communication      | cross-coalition disclosure of ISR products               |
| WIA-laser-weapon                | UAS-mounted laser-designator integration                 |
| WIA-public-safety               | civil-emergency notification when overflying populated areas |
| WIA-medical-data-privacy        | medical-evacuation reconnaissance flows                  |

## Annex B — Conformance disclosure

Implementations declare per-section conformance in their published
capability document. Sections marked `partial` or `excluded` reference
the deployment policy. A deployment that is `partial` or `excluded`
on §3 (Mission plan), §6 (Weapon release), or §7 (Geofence evidence)
is non-conformant overall.

## Annex C — KLV metadata mapping (informative)

MISB ST 0601 KLV tags to PHASE 1 §5 sensor observation fields:

| KLV tag     | Description                              | PHASE 1 §5 field             |
|-------------|------------------------------------------|------------------------------|
| 2           | UNIX timestamp                           | `observationTimestamp`        |
| 13          | Sensor latitude                          | `geometry.airframeState.lat`  |
| 14          | Sensor longitude                         | `geometry.airframeState.lon`  |
| 15          | Sensor altitude                          | `geometry.airframeState.alt`  |
| 16          | Sensor horizontal field of view           | `geometry.fovHorizontal`      |
| 17          | Sensor vertical field of view             | `geometry.fovVertical`        |
| 23          | Frame center latitude                     | `targetGroundFootprint.center.lat` |
| 24          | Frame center longitude                    | `targetGroundFootprint.center.lon` |

This is a non-exhaustive selection; the deployment policy enumerates
the full mapping.

## Annex D — Confidence and freshness
Sensor observations and mti tracks carry per-record confidence bands and freshness metadata so consumers downstream (fusion engines, intelligence analysts) can correctly weight stale or low-confidence entries.
