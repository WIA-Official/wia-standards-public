# WIA-mars-mission PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-mars-mission
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format layer for
WIA-mars-mission. The standard covers data exchange among the entities
that participate in a robotic or crewed Mars mission programme:
spacecraft trajectory and state determination products, mission
planning artefacts, telemetry, tracking, and command (TT&C) packets,
science observation requests and downlinked products, planetary
protection records, and post-mission archive bundles. The format is
a thin envelope that carries community standards (notably the CCSDS
suite) so that interoperation across agencies, prime contractors, and
science teams is preserved.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time representation)
- ISO/IEC 17025:2017 (testing and calibration laboratories)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 11578 (UUID)
- IETF RFC 4122 (UUID URN)
- IETF RFC 8259 (JSON)
- IETF RFC 9457 (Problem Details for HTTP APIs)
- CCSDS 132.0-B (TM Space Data Link Protocol)
- CCSDS 232.0-B (TC Space Data Link Protocol)
- CCSDS 301.0-B (Time Code Formats)
- CCSDS 633.0-B (Mission Operations Services)
- CCSDS 727.0-B (CCSDS File Delivery Protocol — CFDP)
- ITU Radio Regulations (deep-space band allocations)
- COSPAR Planetary Protection Policy

---

## §1 Scope

This PHASE document defines persistent shapes for the records that
flow during the planning, cruise, surface-operations, and
post-mission phases of a Mars-bound mission. Implementations covered
include:

- Mission Operations Centres (MOCs) that command and monitor
  spacecraft.
- Science Operations Centres (SOCs) that plan instrument
  observations and process science products.
- Deep-space communications providers that route TT&C packets.
- Navigation teams that produce trajectory and state estimates.
- Planetary protection officers that maintain bioburden inventories
  and approve activities at sensitive surface targets.
- Data archives that preserve mission products for decades.

Crew-specific medical and life-support records are governed by
adjacent WIA standards and are out of scope here.

## §2 Mission Identifier

```
missionId         : string (uuidv7)
missionRegisteredAt : string (ISO 8601 / RFC 3339)
missionAuthor     : string (institutional identifier of the lead
                       mission organisation)
missionPhase      : enum  ("pre-launch" | "launch" | "cruise" |
                       "approach" | "EDL" | "surface-ops" |
                       "extended-ops" | "decommissioned")
launchVehicle     : string (vehicle family identifier; vehicle-vendor
                       PII held externally)
spacecraftClass   : enum ("orbiter" | "lander" | "rover" | "helicopter" |
                       "sample-return" | "communications-relay")
```

The `missionPhase` is the canonical state machine that drives
phase-specific access controls and report cadences.

## §3 Spacecraft Identifier and Configuration

```
spacecraft:
  spacecraftId    : string (uuidv7)
  missionId       : string (uuidv7)
  norad_id        : string (catalogue identifier; absent for assets
                       not tracked under the public catalogue)
  spacecraftClass : string (matches §2.spacecraftClass)
  payload         : array of InstrumentRef
  configurationRef: string (content-addressed URI of the spacecraft
                       configuration bundle, including allocations of
                       mass, power, and pointing budget)

InstrumentRef:
  instrumentId    : string
  modality        : enum ("imager-vis" | "imager-ir" | "spectrometer" |
                       "radar-ground-penetrating" | "magnetometer" |
                       "weather-station" | "seismometer" |
                       "lidar" | "neutron-spectrometer" |
                       "mass-spectrometer" | "context-camera" |
                       "comms-relay" | "user-defined")
  calibrationRef  : string (content-addressed URI of the calibration
                       bundle that the instrument's products reference)
```

## §4 Trajectory and State Record

State vectors and trajectory products carry the canonical SPICE
(NASA NAIF)-compatible shapes when available, in the SPK / CK /
PCK / FK / IK / SCLK families, and the equivalent ESA OEM
(Orbit Ephemeris Message) and OPM / OMM where exchanged with ESA
partners.

```
trajectoryProduct:
  productId       : string (uuidv7)
  spacecraftId    : string (uuidv7)
  productKind     : enum ("oem" | "opm" | "omm" | "spk" | "ck" |
                       "sclk" | "pck")
  artefactRef     : string (content-addressed URI)
  validFrom       : string (ISO 8601 / RFC 3339)
  validTo         : string (ISO 8601 / RFC 3339)
  generatedBy     : string (navigation team identifier)
  generationProcess : string (process recipe identifier)
  uncertainty     : object (covariance representation; the canonical
                       form is the OEM-aligned 6×6 covariance for
                       state vectors)
```

## §5 Telemetry, Tracking, and Command Packet Record

TT&C packets follow CCSDS 132.0-B (TM Space Data Link) and
CCSDS 232.0-B (TC Space Data Link) framing. Records track the
packet's timeline rather than the framed octets directly; the framed
octets are stored in a content-addressed archive and referenced.

```
ttcPacket:
  packetId        : string (uuidv7)
  spacecraftId    : string (uuidv7)
  direction       : enum ("uplink" | "downlink")
  apid            : integer (CCSDS Application Process Identifier)
  scetUtc         : string (ISO 8601, spacecraft event time)
  ertUtc          : string (ISO 8601, earth received time;
                       absent for uplink)
  groundStation   : string (DSN / ESTRACK / commercial deep-space
                       provider identifier)
  payloadDigest   : string (SHA-256 of the packet payload)
  framedRef       : string (content-addressed URI of the framed
                       packet within the archive)
```

## §6 Science Observation Request

```
observationRequest:
  requestId       : string (uuidv7)
  missionId       : string (uuidv7)
  instrumentId    : string
  startSceUtc     : string (ISO 8601 spacecraft event time)
  endSceUtc       : string (ISO 8601)
  pointing        : object (per-instrument pointing geometry —
                       quaternion, target reference frame, target
                       body fixed frame, etc.)
  sequenceArtefactRef : string (content-addressed URI of the
                       commanded sequence)
  approvals       : array of ApprovalEntry
```

Observation requests cite the approving science working group, the
mission operations approval, and the planetary-protection approval
where applicable.

## §7 Science Product Record

```
scienceProduct:
  productId       : string (uuidv7)
  observationRequestId : string (uuidv7)
  level           : enum ("0" | "1a" | "1b" | "2" | "3" | "4")
  format          : string (PDS4-compatible product type when
                       available; FITS-compatible alternatives are
                       carried as an envelope around the FITS file)
  artefactRef     : string (content-addressed URI)
  pdsLogicalIdentifier : string (PDS LIDVID, when the product is
                       deposited at a PDS-aligned archive)
  generatedBy     : string (processing pipeline identifier)
  qualityFlags    : object (per-pixel or per-record flags)
```

## §8 Planetary Protection Record

Planetary-protection records track the bioburden state of mission
hardware, the controls applied during assembly and integration, and
the allowed activities at sensitive surface targets per COSPAR
policy.

```
planetaryProtection:
  recordId        : string (uuidv7)
  missionId       : string (uuidv7)
  category        : enum ("I" | "II" | "III" | "IVa" | "IVb" |
                       "IVc" | "V")
  bioburdenAssayRef : string (content-addressed URI of the assay
                       results)
  allowedActivities : array of string (sample-return excavation,
                       contact with special regions, etc.)
  approvedBy      : string (planetary-protection officer identifier)
  approvedAt      : string (ISO 8601)
```

## §9 Crew-Activity Reference (Optional)

Crewed missions reference crew-activity records held under adjacent
WIA standards (life-support, EVA, biomedical). The reference is a
content-addressed pointer; mission records do not duplicate
crew-specific clinical content here.

## §10 Surface Operations Activity Plan

Surface assets (rovers, landers, helicopters) operate on activity
plans that schedule mobility, instrument operation, communications
windows, and rest cycles per Mars-local solar day (sol). Activity
plans are exchanged among the science operations team, the rover
operations team, and the mission ops team in the format below.

```
solActivityPlan:
  planId          : string (uuidv7)
  spacecraftId    : string (uuidv7)
  solNumber       : integer (mission sol from landing)
  ltstWindow:
    startLtst     : string (Local True Solar Time, hh:mm)
    endLtst       : string
  activities      : array of SolActivity
  energyBudget    : EnergyBudget
  thermalBudget   : ThermalBudget
  approvalChain   : array of ApprovalEntry

SolActivity:
  activityId      : string
  activityType    : enum ("drive" | "instrument-deploy" |
                       "instrument-observe" | "comms-pass" |
                       "imaging-survey" | "sampling" |
                       "thermal-conditioning" | "sleep")
  startLtst       : string
  durationS       : integer
  prerequisites   : array of string (other activity IDs)
  energyJoules    : number
  dataVolumeMb    : number
  pointing        : object (per-activity pointing geometry)

EnergyBudget:
  startWh         : number (battery state of charge at sol start)
  endWh           : number (predicted state at sol end)
  marginWh        : number (operational margin retained)

ThermalBudget:
  minPredictedKelvin : number
  maxPredictedKelvin : number
  warmupWindows   : array of object (start/end LTST and target
                       component identifier)
```

Activity plans MUST close cleanly: the predicted end-of-sol energy
and thermal state MUST satisfy the published margins before the API
accepts the plan. Plans that fail margin checks return `422` with
type `urn:wia:mars-mission:sol-margin-violation`.

## §11 Sample-Return Chain Records

Sample-return chains carry per-sample metadata from acquisition on
the surface through cache, retrieval, return cruise, atmospheric
entry, recovery on Earth, and curatorial intake. Each step in the
chain emits a record with the prior step referenced, so that
provenance for a returned sample is reconstructible end-to-end.

```
sampleStep:
  stepId          : string (uuidv7)
  sampleId        : string (uuidv7)
  predecessorRef  : string (URI of the prior step; absent for the
                       acquisition step)
  occurredAt      : string (ISO 8601 / RFC 3339)
  occurredOn      : enum ("surface" | "transit" | "earth-recovery" |
                       "earth-curation")
  custodian       : string (institutional identifier)
  containerId     : string (cache tube or capsule identifier)
  environment     : object (temperature, pressure, atmospheric
                       composition during the step)
  notes           : string (free text; redacted on export when
                       contains restricted information)
```

Sample-return chains are subject to additional planetary-protection
controls under §8 of this PHASE; the protocol layer documents the
conditions under which the chain may be released to non-curatorial
consumers.

## §12 Atmospheric and Surface Environmental Record

Surface assets and atmospheric reconstruction products carry an
environmental record that captures the local meteorology, dust
opacity, surface temperature, and solar irradiance over the
relevant observation interval.

```
environment:
  environmentId   : string (uuidv7)
  spacecraftId    : string (uuidv7)
  intervalStart   : string (ISO 8601)
  intervalEnd     : string (ISO 8601)
  atmTemperatureKelvin: number
  atmPressurePa   : number
  windSpeedMs     : number
  windDirectionDeg: number
  dustOpacity     : number (column-integrated dust optical depth)
  solarIrradianceWm2 : number (top-of-atmosphere or surface;
                       record which)
  notes           : string (clouds, haze, sandstorm signatures)
```

Environmental records inform energy and thermal budgeting in sol
activity plans (§10) and are surfaced to mission planners through
the streaming subscription endpoints (PHASE-2 §13).

## §13 Anomaly Investigation Record

Spacecraft and mission anomalies are tracked through a dedicated
record so that the operational response and the lessons-learned
write-up are reconstructible during retrospective audit.

```
anomalyInvestigation:
  investigationId : string (uuidv7)
  missionId       : string (uuidv7)
  detectedAt      : string (ISO 8601)
  affectedAssets  : array of string (spacecraft and instrument
                       identifiers)
  initialClassification : enum ("spacecraft-safe-mode" |
                       "instrument-fault" | "comms-loss" |
                       "thermal-excursion" | "navigation-divergence" |
                       "command-rejected" | "user-defined")
  responseLog     : array of ResponseEntry
  rootCause       : string (free-text root-cause statement once
                       investigation closes)
  closedAt        : string (ISO 8601; absent until closed)
  lessonsLearnedRef : string (content-addressed URI of the
                       lessons-learned document)
```

Anomaly investigations are referenced from sol activity plans and
science products that the anomaly affected so that downstream
consumers see the context surrounding affected operations.

## §14 Conformance

Implementations claiming PHASE-1 conformance emit each of the
records defined above for every supported mission and honour the
content-addressing rules in §3-§8.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-mars-mission
- **Last Updated:** 2026-04-27
