# WIA-nuclear-energy PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-nuclear-energy
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format layer for
WIA-nuclear-energy. The standard covers data exchange among entities
that operate civil nuclear power generation: plant operators, fuel-
cycle facilities (mining, conversion, enrichment, fabrication,
reprocessing, storage and disposal), regulators, safeguards
inspectors, decommissioning organisations, and the energy market
operators that schedule and dispatch nuclear-generated electricity.
The format captures plant identity and configuration, reactor and
core operating state, fuel-assembly inventories, refuelling and
outage records, radiation-protection observations, environmental-
release reports, safety-significant events, decommissioning records,
and the associated regulatory submissions.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time representation)
- ISO/IEC 17025:2017 (testing and calibration laboratories)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 11578 (UUID)
- ISO 9001:2015 (quality management systems)
- ISO 14001:2015 (environmental management systems)
- ISO 45001:2018 (occupational health and safety management)
- ISO 19443:2018 (quality management for nuclear-supplier organisations)
- IETF RFC 4122 (UUID URN)
- IETF RFC 8259 (JSON)
- IETF RFC 9457 (Problem Details for HTTP APIs)
- IAEA Safety Standards Series (cited by series identifier where
  applicable; the IAEA Safety Glossary defines the controlling
  terminology)
- ICRP Publication 103 (terminology for radiation protection
  quantities)

---

## §1 Scope

This PHASE document defines persistent shapes for the records that
flow across the operating life of a civil nuclear power plant and
its supporting fuel-cycle facilities. Implementations covered
include:

- Reactor plant operating systems that emit core operating state.
- Fuel-cycle facilities (conversion, enrichment, fabrication,
  reprocessing) that emit material-movement records.
- Radiation-protection programmes that emit dose-rate observations
  and personnel dose ledgers.
- Spent-fuel and waste-management programmes that track inventories.
- Safeguards inspectorates that consume material balances.
- Decommissioning programmes that track end-of-life dispositions.

Crew-medical records that intersect occupational dosimetry are
referenced into adjacent WIA radiation-protection standards rather
than carried inline here.

## §2 Plant Identifier

```
plantId           : string (uuidv7)
plantRegisteredAt : string (ISO 8601 / RFC 3339)
plantOperator     : string (institutional identifier of the operating
                       organisation)
reactorClass      : enum  ("PWR" | "BWR" | "CANDU" | "VVER" |
                       "RBMK-legacy" | "SMR-pwr" | "SMR-bwr" |
                       "VHTR" | "MSR" | "SFR" | "LFR" | "GFR" |
                       "SCWR")
nominalElectricalMWe : number
nominalThermalMWt    : number
firstCriticalityAt   : string (ISO 8601 date; absent for plants
                       still under construction)
operatingPhase    : enum ("design" | "construction" |
                       "commissioning" | "operating" |
                       "outage" | "post-shutdown-defueling" |
                       "decommissioning" | "released")
```

## §3 Reactor Core Configuration

```
coreConfiguration:
  configurationId : string (uuidv7)
  plantId         : string (uuidv7)
  cycleNumber     : integer (operating cycle index)
  loadingPattern  : array of FuelAssemblyEntry
  controlSystem:
    nControlRods  : integer
    rodGroups     : array of string (e.g. ["regulating",
                       "shutdown-A", "shutdown-B"])
    burnableAbsorberStrategy : string (recipe identifier)
  moderator       : enum ("light-water" | "heavy-water" |
                       "graphite" | "molten-salt" | "lead" |
                       "sodium" | "helium")
  coolant         : enum ("light-water" | "heavy-water" |
                       "molten-salt" | "lead" | "lead-bismuth" |
                       "sodium" | "helium" | "supercritical-water")
  thermalHydraulicModelRef : string (content-addressed URI of the
                       T-H model used for licensing analyses)

FuelAssemblyEntry:
  position        : string (loading-pattern position identifier)
  assemblyId      : string (uuidv7, references §4)
```

Loading-pattern revisions emit new configuration records; prior
records remain addressable as the historical core state.

## §4 Fuel Assembly Record

```
fuelAssembly:
  assemblyId      : string (uuidv7)
  manufacturer    : string (institutional identifier)
  fabricatedAt    : string (ISO 8601 date)
  geometry        : enum ("17x17" | "16x16" | "15x15" | "14x14" |
                       "10x10-bwr" | "9x9-bwr" | "8x8-bwr" |
                       "candu-37r" | "vver-440" | "vver-1000" |
                       "trizo-coated" | "metallic-fast" |
                       "molten-salt-cell")
  enrichment_pct  : number (initial U-235 enrichment, weight-percent)
  burnableAbsorber: string (e.g. "Gd2O3", "ZrB2", "none")
  initialMassKgU  : number
  serialNumber    : string (manufacturer-assigned serial; subject
                       to safeguards confidentiality where relevant)
  burnupHistory   : array of BurnupRecord (cycle-by-cycle exposure)

BurnupRecord:
  cycleNumber     : integer
  cycleDurationDays : integer
  averageBurnupMWdPerKgU : number
```

## §5 Operating State Record

```
operatingState:
  stateId         : string (uuidv7)
  plantId         : string (uuidv7)
  capturedAt      : string (ISO 8601 / RFC 3339)
  thermalPowerMWt : number
  electricalGrossMWe : number
  electricalNetMWe   : number
  primaryPressurePa  : number (primary loop pressure, where
                       applicable)
  primaryTempInletK  : number
  primaryTempOutletK : number
  controlRodPositionsRef : string (URI of full positions map)
  reactorState    : enum ("at-power" | "ramping" | "hot-zero-power"
                       | "hot-shutdown" | "cold-shutdown" |
                       "refueling" | "trip-recovery")
```

## §6 Refuelling and Outage Record

```
outage:
  outageId        : string (uuidv7)
  plantId         : string (uuidv7)
  outageKind      : enum ("planned-refuel" | "planned-maintenance"
                       | "forced-trip-investigation" |
                       "regulatory-inspection-shutdown" |
                       "extended-shutdown")
  startedAt       : string (ISO 8601 / RFC 3339)
  endedAt         : string (ISO 8601; absent until completed)
  workOrders      : array of WorkOrderRef
  fuelAssembliesIn  : array of string (assembly IDs loaded)
  fuelAssembliesOut : array of string (assembly IDs offloaded;
                       continuing into spent-fuel records §8)
  postOutageStartupReportRef : string (URI of the post-outage
                       startup test summary)
```

## §7 Radiation Protection Record

```
radiationProtectionSample:
  sampleId        : string (uuidv7)
  plantId         : string (uuidv7)
  capturedAt      : string (ISO 8601)
  zone            : enum ("controlled-area" | "supervised-area" |
                       "non-controlled" | "spent-fuel-pool" |
                       "containment" | "auxiliary-building" |
                       "user-defined")
  doseRateMicroSvPerHour : number
  contaminationBqPerCm2  : number (loose alpha + beta contamination,
                       reported as effective Bq/cm² per ICRP 103)
  airborneDpac    : number (derived air concentration multiple,
                       when continuous air monitors are deployed)
  workerEntries  : integer (zone entries during the sample window)

personnelDoseLedger:
  workerRef       : string (opaque token; clinical identity in
                       adjacent radiation-protection standard)
  cumulativeMicroSv : number
  observedAt      : string (ISO 8601)
  policyMaxMicroSv : number (operator's annual cumulative limit,
                       which MUST not exceed the regulator's limit)
```

## §8 Spent-Fuel and Waste Record

```
spentFuelInventory:
  inventoryId     : string (uuidv7)
  plantId         : string (uuidv7)
  capturedAt      : string (ISO 8601)
  poolPositions   : array of PoolPosition
  dryStorageCanisters : array of CanisterEntry
  reprocessingShipments : array of ShipmentRef
  totalAssembliesPool : integer
  totalAssembliesDry  : integer

wasteCategorisation:
  wasteId         : string (uuidv7)
  category        : enum ("HLW" | "ILW" | "LLW" | "VLLW" |
                       "exempt")
  containerId     : string
  volumeM3        : number
  activityBq      : number
  heatGeneratingFlag : boolean
  storageLocationId : string
  destinedFor     : enum ("interim-storage" | "encapsulation" |
                       "deep-geological-disposal" |
                       "near-surface-disposal" |
                       "decay-store-then-clearance")
```

## §9 Environmental Release Report

```
environmentalRelease:
  reportId        : string (uuidv7)
  plantId         : string (uuidv7)
  intervalStart   : string (ISO 8601)
  intervalEnd     : string (ISO 8601)
  airborneReleaseBq : object (per-radionuclide totals; tritium,
                       carbon-14, krypton-85, xenon-133, iodine-131,
                       particulate families)
  liquidReleaseBq   : object (per-radionuclide totals; tritium,
                       gross-beta, gross-alpha, cobalt-60,
                       caesium-137)
  publicDoseEstimateMicroSv : number (per the operator's
                       atmospheric dispersion and aquatic
                       exposure model)
  modelRef        : string (content-addressed URI of the dispersion
                       model the operator used)
```

## §10 Safety-Significant Event Record

```
safetyEvent:
  eventId         : string (uuidv7)
  plantId         : string (uuidv7)
  occurredAt      : string (ISO 8601)
  classification  : enum ("INES-0" | "INES-1" | "INES-2" |
                       "INES-3" | "INES-4" | "INES-5" |
                       "INES-6" | "INES-7" | "operator-precursor")
  category        : enum ("trip" | "loss-of-offsite-power" |
                       "ECCS-actuation" | "containment-breach" |
                       "fuel-failure" | "operator-error" |
                       "external-hazard" | "cybersecurity")
  reportedToRegulatorAt : string (ISO 8601)
  rootCauseRef    : string (URI of the root-cause investigation)
  correctiveActionRef : string (URI of the corrective-action plan)
```

INES classifications follow the IAEA International Nuclear and
Radiological Event Scale; classification at the time of detection
MAY be revised as investigation proceeds, with the revision history
preserved as an append-only audit log against the event record.

## §11 Decommissioning Record

```
decommissioning:
  decommissioningId : string (uuidv7)
  plantId         : string (uuidv7)
  strategy        : enum ("immediate-decon" | "deferred-safstor" |
                       "entomb")
  startedAt       : string (ISO 8601)
  expectedEndAt   : string (ISO 8601 date)
  fundingMechanism: string (operator's decommissioning fund
                       reference)
  endStateGoal    : enum ("greenfield-release" |
                       "industrial-reuse" |
                       "restricted-release-for-future-use" |
                       "entombed-perpetual-care")
  milestones      : array of MilestoneRef
```

## §12 Plant-State Documentation

The plant's licensing-relevant configuration (Final Safety Analysis
Report, Technical Specifications, Quality Assurance Manual,
Emergency Plan) is held under change control and referenced by the
records described above.

```
plantStateDocument:
  documentId      : string (uuidv7)
  plantId         : string (uuidv7)
  category        : enum ("FSAR" | "tech-specs" | "QA-manual" |
                       "emergency-plan" | "security-plan" |
                       "decommissioning-plan")
  version         : string (Semantic Versioning 2.0.0)
  approvedAt      : string (ISO 8601)
  regulatorApprovalRef : string (regulator-issued amendment number)
  artefactRef     : string (content-addressed URI)
```

Each safety-event, outage, and core-configuration record references
the plant-state documents in force at the time of the event so that
retrospective audit can reconstruct the licensing context.

## §13 In-Service Inspection Record

In-service inspection (ISI) of safety-related components is governed
by the regulator-approved inspection programme. The ISI record
captures the inspection method, the inspected component, the result,
and the disposition for any indication that exceeds the recordable
threshold.

```
isiRecord:
  isiId           : string (uuidv7)
  plantId         : string (uuidv7)
  componentRef    : string (in-service-inspection plan component
                       identifier)
  performedAt     : string (ISO 8601)
  method          : enum ("visual" | "ultrasonic" | "eddy-current" |
                       "radiographic" | "liquid-penetrant" |
                       "magnetic-particle" | "phased-array" |
                       "guided-wave")
  result          : enum ("acceptable" | "indication-recordable" |
                       "indication-rejectable")
  dispositionRef  : string (URI of the engineering evaluation;
                       absent when result = acceptable)
  laboratoryId    : string (ISO/IEC 17025-accredited inspection
                       laboratory)
```

ISI records of `indication-rejectable` outcomes trigger the
operating organisation's component-replacement or repair pathway,
which is itself a licensing-relevant activity recorded against the
plant.

## §14 Conformance

Implementations claiming PHASE-1 conformance emit each of the
records defined above for every operating plant and honour the
content-addressing rules in §3-§11.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-nuclear-energy
- **Last Updated:** 2026-04-27
