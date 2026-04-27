# WIA-moon-base PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-moon-base
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format layer for
WIA-moon-base. The standard covers data exchange among the entities
that operate persistent or recurrent crewed and robotic surface
infrastructure on the Moon: habitat modules and life-support
telemetry, in-situ resource utilisation (ISRU) plant operation,
power-generation and -distribution telemetry, mobility-asset
operations, surface-network communications, EVA scheduling, science
and engineering operations on the surface, and the supply and
crew-rotation chain that connects the surface base to Earth.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time representation)
- ISO/IEC 17025:2017 (testing and calibration laboratories)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 11578 (UUID)
- IETF RFC 4122 (UUID URN)
- IETF RFC 8259 (JSON)
- IETF RFC 9457 (Problem Details)
- ISO 11073-10101 (medical device communication — nomenclature; used
  for crew-medical observation envelopes only)
- ITU Radio Regulations (lunar surface and cislunar band allocations)
- CCSDS 132.0-B / 232.0-B (Space Data Link Protocols for the
  cislunar relay link)
- CCSDS 727.0-B (CFDP)

---

## §1 Scope

This PHASE document defines persistent shapes for the records that
flow during the construction, commissioning, occupied-operation, and
unoccupied-caretaker phases of a Moon base. Implementations covered
include:

- Habitat module operating systems that emit life-support telemetry.
- ISRU plant control systems (oxygen extraction, water-ice
  processing, regolith handling, propellant production).
- Power systems (solar arrays, regenerative fuel cells, fission
  surface-power units, energy storage, microgrid management).
- Surface mobility assets (rovers, hoppers, surface excavators,
  cable-laying equipment).
- Surface communications networks and the cislunar relay link.
- EVA support systems including suit telemetry and life-support
  consumables.
- Surface science and engineering operations centres.

Crew clinical records are referenced into adjacent WIA crew-medical
standards rather than carried inline here.

## §2 Base Identifier

```
baseId            : string (uuidv7)
baseRegisteredAt  : string (ISO 8601 / RFC 3339)
baseAuthor        : string (institutional identifier of the lead
                       operating organisation)
basePhase         : enum  ("design" | "construction" |
                       "commissioning" | "occupied" |
                       "unoccupied-caretaker" | "decommissioned")
landingSiteRef    : string (selenographic coordinate triplet plus
                       lunar reference frame identifier)
operatingPartners : array of string (partner institutional
                       identifiers)
```

## §3 Habitat Module Record

```
habitatModule:
  moduleId        : string (uuidv7)
  baseId          : string (uuidv7)
  moduleClass     : enum ("crew-habitat" | "lab-module" |
                       "airlock" | "logistics-module" |
                       "external-pressurised-rover" |
                       "agriculture-module" | "medical-module")
  pressurisedVolumeM3 : number
  emergencyConsumablesDays : integer (planned days of life-support
                       consumables held in module under nominal
                       crew loading)
  configurationRef: string (content-addressed URI of the module's
                       configuration bundle)
  craftYearOfBuild: integer
  certificationRef: string (URI of the module's safety certification)
```

## §4 Life-Support Telemetry Record

Life-support telemetry covers the atmospheric composition, the
pressure, the temperature, and the moisture content inside each
pressurised volume, plus the consumables-management state.

```
lifeSupportSample:
  sampleId        : string (uuidv7)
  moduleId        : string (uuidv7)
  capturedAt      : string (ISO 8601 / RFC 3339)
  atmospherePressurePa : number
  oxygenPartialPaPa    : number
  carbonDioxideMm Hg   : number (mmHg of CO2)
  trace:
    coVolPpm      : number (carbon monoxide)
    nh3VolPpm     : number
    voctotalVolPpm: number (total volatile organic compounds)
  temperatureKelvin: number
  humidityPercent  : number
  consumablesState:
    o2KgRemaining : number
    h2oKgRemaining: number
    co2ScrubberPercentRemaining : number
```

Implementations MUST emit life-support samples at a cadence at least
as fast as the safety-engineering threshold response time, and MUST
preserve the full sample history during occupied phases for
incident reconstruction.

## §5 Power and Microgrid Record

```
powerSample:
  sampleId        : string (uuidv7)
  baseId          : string (uuidv7)
  capturedAt      : string (ISO 8601)
  generationKw:
    solar         : number
    fissionSurfacePower : number (zero when not deployed)
    regenFuelCell : number (positive when generating, negative when
                       charging)
  storageState:
    batterySocPercent : number
    flywheelKwh   : number
  loadKw:
    habitat       : number
    isru          : number
    science       : number
    mobility      : number
    other         : number
  microgridState  : enum ("normal" | "load-shedding" |
                       "islanded-from-isru" | "emergency-only")
```

## §6 ISRU Plant Record

ISRU plants extract oxygen, water, and feedstock from regolith and
volatiles. The plant record carries process state, throughput, and
consumable inventory.

```
isruPlant:
  plantId         : string (uuidv7)
  baseId          : string (uuidv7)
  process         : enum ("hydrogen-reduction" |
                       "molten-regolith-electrolysis" |
                       "water-ice-mining" |
                       "carbothermal-reduction" |
                       "user-defined")
  feedstockKgPerSol : number
  productKgPerSol :
    o2            : number
    h2o           : number
    metals        : number
    other         : number
  consumablesIn:
    h2KgPerSol    : number (when applicable)
    powerKwAvg    : number
  state           : enum ("idle" | "starting" | "running" |
                       "shutting-down" | "fault")
```

## §7 Mobility Asset Record

Surface mobility assets (rovers, hoppers, surface excavators) emit
telemetry summarising their state, position, traversed terrain, and
mission tasks.

```
mobilityAsset:
  assetId         : string (uuidv7)
  baseId          : string (uuidv7)
  assetClass      : enum ("crewed-rover" | "uncrewed-rover" |
                       "surface-hopper" | "excavator" |
                       "cable-layer" | "mobile-power-cart")
  position        : object (selenographic latitude, longitude,
                       elevation; reference frame identifier)
  velocityMs      : number
  batterySocPercent : number
  taskQueueRef    : string (URI of current task queue)
  faultLog        : array of FaultEvent
```

## §8 EVA Operations Record

```
evaOperation:
  evaId           : string (uuidv7)
  baseId          : string (uuidv7)
  crewMemberRef   : string (opaque token; clinical identity held
                       in adjacent crew-medical standard)
  startedAt       : string (ISO 8601)
  plannedDurationS: integer
  actualDurationS : integer (absent until completed)
  egressAirlockId : string
  ingressAirlockId: string
  routePlanRef    : string (URI of route plan)
  consumablesUsed : object (oxygen, water, battery, scrubber state
                       at egress vs ingress)
  incidents       : array of EvaIncident
```

EVA records cite the crew member through the opaque token defined
in PHASE-1 §2 of the crew-medical standard rather than carrying
clinical identity directly.

## §9 Surface Communications Record

```
linkLog:
  linkId          : string (uuidv7)
  baseId          : string (uuidv7)
  linkKind        : enum ("intra-base" | "base-to-relay" |
                       "relay-to-earth" | "eva-suit-uplink" |
                       "eva-suit-downlink")
  startedAt       : string (ISO 8601)
  durationS       : integer
  signalToNoiseDb : number
  bitErrorRate    : number
```

## §10 Supply Chain and Inventory Record

Supply-chain records carry per-launch manifests, per-arrival inventory
deltas, and the running on-base inventory across consumables and
spares.

```
supplyLaunch:
  launchId        : string (uuidv7)
  baseId          : string (uuidv7)
  launchAt        : string (ISO 8601)
  predictedArrivalAt : string (ISO 8601)
  manifest        : array of ManifestItem

ManifestItem:
  itemClass       : enum ("consumable-o2" | "consumable-h2o" |
                       "consumable-food" | "consumable-co2-scrubber" |
                       "spare-part" | "scientific-payload" |
                       "isru-feedstock" | "fuel-propellant" |
                       "personal-cargo")
  quantity        : number
  unit            : string
  ownerRef        : string (institutional identifier of the
                       requesting team)
```

```
inventory:
  inventoryId     : string (uuidv7)
  baseId          : string (uuidv7)
  capturedAt      : string (ISO 8601)
  consumables     : object (per-class running totals: o2Kg, h2oKg,
                       foodPersonDays, scrubberCapacityRemaining)
  spares          : array of SpareInventoryEntry
  scientificPayloads : array of PayloadInventoryEntry
```

Inventory snapshots are emitted at each cargo arrival, after every
EVA that consumes consumables, and on a daily summary cadence.

## §11 Crew Rotation Record

```
crewRotation:
  rotationId      : string (uuidv7)
  baseId          : string (uuidv7)
  rotationDirection : enum ("inbound" | "outbound" | "intra-base")
  scheduledAt     : string (ISO 8601)
  occurredAt      : string (ISO 8601; absent until completed)
  crewMemberRefs  : array of string (opaque tokens, see §8)
  vehicleId       : string (transit vehicle identifier)
  notes           : string (free text; redacted on export when
                       contains medical detail)
```

## §12 Heritage and Exclusion Zone Record

The base's surface footprint excludes designated heritage sites
(prior crewed-mission landing zones, scientific reserves) and
exclusion zones around active operations (landing pads, ISRU
plant work areas, EVA traverse staging). The exclusion-zone record
captures each zone's polygon, the reason for exclusion, the
authority that established the zone, and the conditions under which
the zone may be entered.

## §13 Radiation Environment Record

Surface and near-surface radiation environments inform crew
exposure budgeting, shielding decisions, and instrument-electronics
upset rates. The record carries time-binned dose-rate observations,
solar-particle-event flags, and the running cumulative dose
attributed to crew and instrument deployments.

```
radiationEnvironmentSample:
  sampleId        : string (uuidv7)
  baseId          : string (uuidv7)
  capturedAt      : string (ISO 8601)
  intervalDurationS : integer
  doseRateMicroSvPerHour : number
  solarParticleEventFlag : boolean
  galacticCosmicRayFluxFraction : number (relative to recent
                       baseline)
  shieldingContext : enum ("inside-habitat" | "inside-rover" |
                       "open-eva" | "deep-shelter")
```

```
crewDoseLedger:
  crewMemberRef   : string (opaque token)
  cumulativeMicroSv : number
  observedAt      : string (ISO 8601)
  policyMaxMicroSv : number (the operating organisation's
                       cumulative limit; clinical-side conversion
                       to effective dose is held in the crew-
                       medical standard)
```

## §14 Lunar-Local Time and Sol Record

The base operates on a lunar-day-aligned activity rhythm that is
nonetheless reconciled to UTC for inter-agency exchange. The sol
record carries the mapping from base-local time to UTC and the
operational state during each lunar-day phase (sunrise, mid-day,
sunset, lunar night).

```
lunarSol:
  solId           : string (uuidv7)
  baseId          : string (uuidv7)
  startUtc        : string (ISO 8601)
  endUtc          : string (ISO 8601)
  illuminationProfile : enum ("permanent-shadow" | "polar-near-
                       constant-illumination" | "equatorial-cycle" |
                       "user-defined")
  thermalProfileRef : string (URI of the thermal model the operating
                       organisation uses for the sol)
```

## §15 Anomaly Investigation Reference

Base-anomaly investigations follow the record shape used by the
adjacent WIA-mars-mission standard. The reference is included here
so that downstream consumers do not need to traverse two standards
to reconstruct an anomaly's context. The investigation record
carries the timeline of detection and response, the root-cause
statement once available, and the lessons-learned write-up that
re-enters the programme's quality dossier.

## §16 Conformance

Implementations claiming PHASE-1 conformance emit each of the
records defined above for every operating module and asset and
honour the content-addressing rules in §3-§9.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-moon-base
- **Last Updated:** 2026-04-27
