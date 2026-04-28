# WIA-interplanetary-travel PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-interplanetary-travel
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format layer for
WIA-interplanetary-travel. The standard covers the persistent
record shapes that govern crewed and uncrewed interplanetary
travel beyond the Earth-Moon system: trajectory plans across
the heliocentric reference frame, life-support consumable
budgets, radiation-exposure ledgers for crewed missions,
gravity-assist flyby plans, deep-space communications
schedules, planetary-protection compliance, and entry-descent-
landing or orbit-insertion records at the destination. The
format is consumed by mission operations centres, launch
service providers, deep-space tracking networks, planetary-
protection authorities, and the regulators that license the
mission.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time representation)
- ISO 31 (quantities and units)
- ISO/IEC 11578 (UUID)
- ISO 14620-1 (space systems — safety requirements — system
  safety)
- ISO 14624 (space systems — safety and compatibility of
  materials)
- ISO 24113:2023 (space systems — space debris mitigation
  requirements)
- ISO 27852 (space systems — estimation of orbit lifetime)
- ISO/IEC 27001:2022 (information security management)
- IETF RFC 4122 (UUID URN)
- IETF RFC 8259 (JSON)
- IETF RFC 9457 (Problem Details)
- CCSDS 301.0-B (Time Code Formats)
- CCSDS 503.0-B (Tracking Data Message)
- CCSDS 504.0-B (Orbit Data Message)
- CCSDS 505.0-B (Attitude Data Message)
- CCSDS 508.0-B (Conjunction Data Message)
- CCSDS 727.0-B (CFDP — CCSDS File Delivery Protocol)
- ITU-R RR Article 22 (deep-space telecommunications
  allocations and protection rules)
- IAU Working Group on Cartographic Coordinates and Rotational
  Elements report (planetary body reference frames)
- IAU Standards of Fundamental Astronomy (SOFA library)
- COSPAR Planetary Protection Policy
- NASA NPR 8020.12 (Planetary Protection — cited normatively
  for category definitions and tier requirements)
- ICRP Publication 132 (Radiological Protection from Cosmic
  Radiation in Aviation; cited as the protection-framework
  reference baseline that crewed-mission programmes adapt to
  the deep-space environment)

---

## §1 Scope

This PHASE defines persistent shapes for the artefacts produced
and consumed by an interplanetary-travel programme. Implementations
covered include:

- Mission Operations Centres (MOCs) running uncrewed
  interplanetary missions (orbiter, lander, flyby, sample-
  return) and crewed missions to lunar gateway and beyond.
- Trajectory design teams using JPL ephemerides (DE441 or
  equivalent), SPICE toolkit kernels, and the IAU body-fixed
  reference frames.
- Launch service providers operating Earth-departure stages.
- Deep-space tracking network operators (NASA Deep Space
  Network, ESA ESTRACK, JAXA, CSNA, commercial DSN providers).
- Planetary-protection officers operating per COSPAR
  category assignments.
- Regulators licensing the mission under the operating
  jurisdiction's space activities law.

Lunar-only missions and Earth-orbit missions are addressed in
adjacent WIA standards (WIA-moon-base, WIA-earth-orbit-
operations) and are out of scope here.

## §2 Mission Identifier

```
missionId          : string (uuidv7)
missionOperator    : string (institutional identifier of the
                       mission operator)
missionRegistered  : string (ISO 8601 / RFC 3339)
missionClass       : enum ("uncrewed-orbiter" |
                       "uncrewed-lander" |
                       "uncrewed-rover" |
                       "uncrewed-flyby" |
                       "uncrewed-sample-return" |
                       "crewed-flyby" |
                       "crewed-orbital" |
                       "crewed-surface" |
                       "technology-demonstration")
destinationBody    : enum ("mercury" | "venus" | "mars" |
                       "jupiter" | "saturn" | "uranus" |
                       "neptune" | "asteroid-belt-target" |
                       "near-earth-asteroid" |
                       "comet-target" | "trans-neptunian-object"
                       | "heliocentric-only")
cosparCategory     : enum ("I" | "II" | "III" | "IVa" | "IVb" |
                       "IVc" | "V-restricted-earth-return" |
                       "V-unrestricted-earth-return")
spaceLawAuthorisationRef : string (mission licence reference
                       under the operating jurisdiction's
                       space activities law)
missionStatus      : enum ("design" | "manufacturing" |
                       "integrated-pre-launch" |
                       "in-cruise" |
                       "in-operation" |
                       "extended-mission" |
                       "end-of-mission" |
                       "disposed")
```

## §3 Reference-Frame Record

All position and attitude states reference an explicit frame so
that downstream consumers can transform between frames without
ambiguity.

```
referenceFrame:
  frameId            : string (uuidv7)
  frameKind          : enum ("ICRF" | "EME2000" |
                       "ECLIPJ2000" | "IAU-body-fixed" |
                       "spacecraft-body" |
                       "instrument-body" |
                       "user-defined")
  bodyRef            : string (IAU body identifier when the
                       frame is body-fixed; absent for inertial
                       frames)
  spiceKernelRef     : string (URI of the SPICE kernel that
                       defines the frame, when applicable)
  ephemerisVersion   : string (e.g. "DE441" or successor)
```

## §4 Trajectory Record

```
trajectory:
  trajectoryId       : string (uuidv7)
  missionId          : string (uuidv7)
  designIteration    : integer (per-design-cycle iteration
                       counter)
  epoch              : string (ISO 8601 / CCSDS 301.0-B
                       compatible)
  orbitDataMessageRef: string (content-addressed URI of the
                       OEM artefact per CCSDS 504.0-B)
  trajectorySegments : array of TrajectorySegment

TrajectorySegment:
  segmentId          : string
  startEpoch         : string (ISO 8601)
  endEpoch           : string (ISO 8601)
  segmentKind        : enum ("ballistic-cruise" |
                       "powered-burn" | "low-thrust-arc" |
                       "gravity-assist-flyby" |
                       "aerocapture" | "aerobraking" |
                       "orbit-insertion" |
                       "edl-descent")
  deltaVMps          : number (segment delta-V budget in m/s)
  flybyBodyRef       : string (IAU body identifier for
                       gravity-assist flybys)
  closestApproachKm  : number (km altitude / closest approach
                       distance for flybys)
```

## §5 Life-Support Consumable Budget Record

```
consumableBudget:
  budgetId           : string (uuidv7)
  missionId          : string (uuidv7)
  budgetEpoch        : string (ISO 8601)
  crewSizePersons    : integer (0 for uncrewed)
  consumableLines    : array of object (per-consumable code
                       — e.g. "LOX-kg", "WATER-kg", "FOOD-kcal",
                       "N2-kg", "LITHIUM-HYDROXIDE-kg" — with
                       starting mass, daily-rate envelope,
                       reserve mass, recovery-loop assumptions)
  closureRatio       : number (ECLSS closure ratio per the
                       operator's life-support model; 0 for
                       open-loop, approaches 1 for closed-loop)
```

## §6 Radiation-Exposure Ledger Record (Crewed Missions)

Crewed deep-space missions incur galactic-cosmic-ray (GCR) and
solar-particle-event (SPE) exposure that the mission operator
manages within the crew's career and per-mission dose limits
defined by the operating space agency's Radiation Health
Office.

```
radiationLedger:
  ledgerId           : string (uuidv7)
  missionId          : string (uuidv7)
  crewMemberToken    : string (opaque crew identifier; clinical
                       identity held in the operator's medical
                       records)
  intervalStart      : string (ISO 8601)
  intervalEnd        : string (ISO 8601)
  ambientGcrMsv      : number (effective dose contribution from
                       GCR over the interval, mSv)
  ambientSpeMsv      : number (effective dose contribution from
                       SPEs over the interval, mSv)
  shieldingState     : enum ("transit-vehicle" |
                       "spe-storm-shelter" |
                       "surface-habitat" |
                       "evita-suit-extra-vehicular")
  cumulativeMissionMsv : number (cumulative mission dose to
                       this crew member)
  cumulativeCareerMsv  : number (cumulative career dose; held
                       in agency Radiation Health records,
                       referenced here for ledger continuity)
```

## §7 Conjunction and Debris-Mitigation Record

Per ISO 24113:2023, missions operating in any region with
conjunction risk (Earth departure orbits, Mars orbit
operations, Jupiter system) emit conjunction analyses and
manoeuvre decisions.

```
conjunction:
  conjunctionId      : string (uuidv7)
  missionId          : string (uuidv7)
  detectedAt         : string (ISO 8601)
  cdmRef             : string (content-addressed URI of the
                       Conjunction Data Message per CCSDS
                       508.0-B)
  secondaryObjectRef : string (other party identifier; e.g.
                       NORAD catalogue, COSPAR designation,
                       mission identifier)
  toca               : string (time of closest approach,
                       ISO 8601)
  missDistanceKm     : number
  collisionProbability : number (operator's chosen Pc model)
  decision           : enum ("monitor" | "manoeuvre-planned" |
                       "manoeuvre-executed" |
                       "no-action-justified")
```

## §8 Planetary-Protection Compliance Record

```
ppCompliance:
  complianceId       : string (uuidv7)
  missionId          : string (uuidv7)
  cosparCategory     : enum (matches §2 cosparCategory)
  bioburdenAssayRef  : string (URI of the bioburden assay
                       record per the operator's COSPAR-
                       aligned protocol)
  cleanroomFacilityRef : string (cleanroom facility
                       certification reference)
  approvalAuthorityRef : string (planetary-protection officer
                       and authority of record)
  approvalState      : enum ("planning" | "interim-approved" |
                       "approved-for-launch" |
                       "post-launch-monitoring" |
                       "end-of-mission-disposition")
```

## §9 Communications Schedule Record

```
commsSchedule:
  scheduleId         : string (uuidv7)
  missionId          : string (uuidv7)
  windowStart        : string (ISO 8601 / RFC 3339)
  windowEnd          : string (ISO 8601)
  groundStationRef   : string (deep-space tracking station
                       identifier; e.g. "DSS-14", "DSS-43",
                       "DSS-25", "ESTRACK-NNORCIA")
  bandSelection      : enum ("S-band" | "X-band" | "Ka-band"
                       | "optical-laser-comms")
  uplinkCommandPlanRef: string (URI of the uplink command
                       sequence)
  downlinkProductExpectedBytes : integer
  oneWayLightTimeS   : number (light-time at window midpoint;
                       seconds)
  scheduleStatus     : enum ("requested" | "allocated" |
                       "executed" | "missed-station-side" |
                       "missed-spacecraft-side")
```

Communications schedules carry the one-way-light-time at the
window midpoint so that operators can plan command-and-
response cycles with awareness of the round-trip delay
(minutes for inner planets, hours for outer planets).

## §10 Entry-Descent-Landing or Orbit-Insertion Record

```
arrival:
  arrivalId          : string (uuidv7)
  missionId          : string (uuidv7)
  arrivalKind        : enum ("orbit-insertion" |
                       "edl-direct" | "edl-following-orbit" |
                       "flyby-only" | "atmospheric-entry-only")
  arrivalEpoch       : string (ISO 8601 / RFC 3339)
  predictedSequenceRef: string (URI of the arrival timeline)
  observedTelemetryRef: string (URI of the post-arrival
                       telemetry archive; absent until the
                       event occurs and downlink is received)
  arrivalOutcome     : enum ("nominal" | "off-nominal-recovered"
                       | "anomaly-investigation" | "loss")
  postArrivalRef     : string (URI of the post-arrival report)
```

## §11 Anomaly Investigation Record

Mission anomalies — telemetry off-nominal events, on-board
fault-management triggers, unexpected hardware degradation,
mission-loss events — are recorded against the mission and
referenced from the relevant trajectory iteration, consumable
budget, radiation ledger, or arrival event.

```
anomaly:
  anomalyId          : string (uuidv7)
  missionId          : string (uuidv7)
  detectedAt         : string (ISO 8601)
  detectorRef        : enum ("on-board-fault-management" |
                       "ground-telemetry-screening" |
                       "trend-analysis" |
                       "operator-manual-detection")
  severityClass      : enum ("informational" | "minor" |
                       "major" | "critical" | "loss")
  affectedSubsystem  : string (operator's subsystem code)
  rootCauseRef       : string (URI of root-cause investigation
                       report; absent until the investigation
                       concludes)
  correctiveActionRef: string (URI of corrective-action plan)
  closedAt           : string (ISO 8601; absent until closed)
```

## §12 Conformance

Implementations claiming PHASE-1 conformance emit each of the
records defined above for every mission and honour the COSPAR
category assignment in §2.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-interplanetary-travel
- **Last Updated:** 2026-04-28
