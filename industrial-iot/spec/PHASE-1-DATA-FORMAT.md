# WIA-industrial-iot PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-industrial-iot
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format layer for
WIA-industrial-iot. The standard covers data exchange among the
entities that operate Industrial Internet of Things (IIoT)
deployments: shop-floor sensors and actuators, edge gateways,
historians, supervisory control and data acquisition (SCADA)
systems, manufacturing execution systems (MES), enterprise
resource planning (ERP) systems, asset performance management
(APM) services, and the regulators and certifying bodies that
inspect industrial deployments. The format captures device
identity and capability descriptions, time-series telemetry,
asset hierarchies, control-loop configuration, alarm and event
records, lifecycle and maintenance evidence, and the
cybersecurity posture of the deployment.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time representation)
- ISO/IEC 17025:2017 (testing and calibration laboratories — used
  for instrument calibration)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 27019:2024 (information security controls for energy
  utilities)
- ISO/IEC 11578 (UUID)
- ISO 14224:2016 (petroleum, petrochemical and natural-gas
  industries — collection and exchange of reliability and
  maintenance data; cited normatively for asset reliability data
  dictionary)
- IETF RFC 4122 (UUID URN)
- IETF RFC 8259 (JSON)
- IETF RFC 9457 (Problem Details for HTTP APIs)
- IEC 61131-3 (programmable controllers — programming languages;
  cited normatively for control-language data types)
- IEC 62264 (enterprise-control system integration — ISA-95;
  cited normatively for asset-hierarchy and operations data
  exchange)
- IEC 62443 (security for industrial automation and control
  systems; cited normatively for cybersecurity zones, conduits,
  and security levels)
- IEC 61850 (communication networks for substation automation;
  cited normatively for power-utility data envelope)
- OPC Unified Architecture (UA) Specification (OPC 10000-1 et
  seq.; cited normatively as the dominant industrial data-
  exchange envelope)
- W3C XML Schema Definition 1.1 (legacy industrial XML imports)

---

## §1 Scope

This PHASE document defines persistent shapes for the records that
flow across the operating life of an IIoT deployment:
commissioning, run-time telemetry, control-loop tuning,
alarm/event reporting, maintenance, and decommissioning.
Implementations covered include:

- Edge-gateway firmware that aggregates field-bus traffic.
- Historians and time-series databases that store telemetry.
- SCADA front-ends that present operator displays.
- MES and APM services that consume aggregated telemetry.
- Asset-performance and reliability-engineering teams that
  produce KPIs from the deployment.
- ERP integrations that consume material-flow events from the
  shop floor.
- Regulators and certifying bodies that audit deployments under
  IEC 62443 / ISO/IEC 27019 expectations.

Real-time motion-control loops (sub-millisecond closed-loop
trajectories) and safety-instrumented-system (SIS) bus protocols
are out of scope; they are covered by adjacent WIA standards.

## §2 Site and Asset Hierarchy

Asset hierarchies follow the IEC 62264 (ISA-95) Site / Area /
Production-Unit / Work-Cell / Equipment-Module model.

```
site:
  siteId          : string (uuidv7)
  operatorRef     : string (institutional identifier of the
                       operating organisation)
  location        : object (ISO 3166 country code + plant code)
  isa95Level      : enum ("enterprise" | "site" | "area" |
                       "production-unit" | "work-cell" |
                       "equipment-module")
  parentSiteId    : string (uuidv7; absent for top-level)
  iec62443Zone    : string (zone identifier per the operator's
                       IEC 62443 architecture)
  securityLevel   : enum ("SL-1" | "SL-2" | "SL-3" | "SL-4")
```

Sites carry the IEC 62443 zone classification so that downstream
consumers (regulators, audit tooling, anomaly-detection services)
know which security level applies to data flowing from the site.

## §3 Asset / Device Record

```
asset:
  assetId         : string (uuidv7)
  siteId          : string (uuidv7)
  assetClass      : enum ("sensor" | "actuator" | "controller-plc" |
                       "controller-rtu" | "drive" | "motor" |
                       "valve" | "pump" | "compressor" |
                       "heat-exchanger" | "robot" |
                       "vision-system" | "edge-gateway" |
                       "historian" | "user-defined")
  manufacturerRef : string (institutional identifier)
  modelName       : string
  serialNumber    : string (manufacturer-assigned)
  firmwareVersion : string (Semantic Versioning 2.0.0)
  capabilityProfileRef : string (content-addressed URI of the OPC
                       UA companion-spec or vendor capability
                       descriptor)
  iec14224Class   : string (asset class per ISO 14224 dictionary
                       when applicable; e.g. "centrifugal pump",
                       "electric-driven compressor")
  installedAt     : string (ISO 8601 date)
  decommissionAt  : string (ISO 8601 date; absent for active
                       assets)
  serviceTier     : enum ("safety-related" | "production-critical" |
                       "process-support" | "non-critical")
```

Asset records carry both the vendor-supplied identifier (model +
serial) and the operator-assigned `assetId`; the binding is the
operator's responsibility and is preserved across vendor handoffs.

## §4 Telemetry Record

```
telemetrySample:
  sampleId        : string (uuidv7)
  assetId         : string (uuidv7)
  capturedAt      : string (ISO 8601 / RFC 3339, sub-second
                       precision)
  capturedAtSource: enum ("plc-clock" | "edge-gateway-clock" |
                       "historian-clock" | "synced-time-source")
  signalRef       : string (signal identifier per the asset's
                       capability profile)
  value           : number
  unit            : string (UCUM-compliant unit code where
                       available)
  qualityCode     : enum ("good" | "uncertain-non-specific" |
                       "uncertain-last-usable-value" |
                       "bad-not-connected" | "bad-device-failure" |
                       "bad-out-of-service")
  serverTimestamp : string (ISO 8601, when the sample reached
                       the historian)
```

Telemetry samples follow OPC UA quality-code semantics so that
downstream consumers can distinguish good values from values
emitted under sensor fault. Samples whose `qualityCode` is in the
`bad-*` family are accepted into the historian for forensic
reasons but are excluded from KPI computation by default.

## §5 Control-Loop Record

```
controlLoop:
  loopId          : string (uuidv7)
  siteId          : string (uuidv7)
  controllerAssetId : string (uuidv7)
  controlledVariable : string (signal identifier; the process
                       variable being regulated)
  manipulatedVariable : string (signal identifier of the
                       actuator output)
  setpointSource  : enum ("operator-manual" | "scheduled" |
                       "cascade-from-supervisor" |
                       "model-predictive-control" |
                       "user-defined")
  controlMode     : enum ("manual" | "auto" | "cascade" |
                       "remote-output" | "out-of-service")
  tuningRef       : string (content-addressed URI of the tuning
                       record; PID gains, deadband, output
                       limits)
  iec61131Language : enum ("LD" | "FBD" | "ST" | "IL" | "SFC")
                       (when the loop is implemented in IEC
                       61131-3 logic)
```

Tuning revisions emit new tuning records; prior tunings remain
addressable so that retrospective analyses (post-incident review,
performance benchmarking) can reproduce the loop's behaviour at
the time of interest.

## §6 Alarm and Event Record

```
alarmEvent:
  eventId         : string (uuidv7)
  assetId         : string (uuidv7)
  occurredAt      : string (ISO 8601 / RFC 3339)
  category        : enum ("process-alarm" | "diagnostic-alarm" |
                       "communication-alarm" | "security-alarm" |
                       "operator-action" | "system-event")
  priority        : integer (1-4 per ISA-18.2 priority levels;
                       1 highest)
  description     : string (operator-readable text)
  relatedSignalRef: string (signal identifier when applicable)
  acknowledgedAt  : string (ISO 8601; absent until acknowledged)
  clearedAt       : string (ISO 8601; absent until cleared)
  operatorRef     : string (acknowledging operator's institutional
                       identifier; absent until acknowledged)
```

Alarm and event records retain indefinitely for safety-related
assets per the operator's records-retention policy in PHASE-3.

## §7 Maintenance and Reliability Record

```
maintenance:
  recordId        : string (uuidv7)
  assetId         : string (uuidv7)
  performedAt     : string (ISO 8601)
  category        : enum ("preventive-scheduled" |
                       "predictive-condition-based" |
                       "corrective-emergency" |
                       "corrective-deferred" |
                       "calibration-iso-17025")
  iec14224Failure : string (ISO 14224 failure-mechanism code when
                       category is corrective-emergency or
                       corrective-deferred)
  technicianRef   : string (institutional identifier; PII held
                       in operator HR system)
  workOrderRef    : string (CMMS work-order reference)
  partsConsumed   : array of PartConsumption
  durationMinutes : integer
```

## §8 Cybersecurity Posture Record

The site's cybersecurity posture is recorded at the level the
IEC 62443 architecture dictates: zone definitions, conduit
definitions between zones, security levels, and the controls in
force.

```
cyberPosture:
  postureId       : string (uuidv7)
  siteId          : string (uuidv7)
  zones           : array of ZoneDefinition (per IEC 62443)
  conduits        : array of ConduitDefinition
  controlsInForce : array of string (control identifiers from
                       IEC 62443-3-3)
  lastReviewedAt  : string (ISO 8601)
  reviewerRef     : string (operator security officer or external
                       certifying body identifier)
```

## §9 Production Order and Material Traceability Record

ERP integrations consume production orders that link enterprise-
level demand to shop-floor execution. Material traceability binds
input-lot identifiers to output-lot identifiers so that recall
events can resolve from finished goods back to source material.

```
productionOrder:
  orderId         : string (uuidv7)
  siteId          : string (uuidv7)
  erpReference    : string (ERP-side order identifier)
  releasedAt      : string (ISO 8601)
  scheduledStart  : string (ISO 8601)
  workCellAssetId : string (uuidv7; references the work cell)
  recipeRef       : string (content-addressed URI of the master
                       batch recipe)
  inputLots       : array of LotReference
  outputLots      : array of LotReference
  status          : enum ("released" | "started" | "in-progress" |
                       "complete" | "scrapped" | "on-hold")

LotReference:
  lotId           : string
  materialCode    : string
  quantity        : number
  unit            : string (UCUM)
```

Material traceability records preserve the binding from input lot
to output lot for the period the operating jurisdiction's product-
liability law requires (typically the longer of product expected
service life plus 10 years, or the regulator's required window).

## §10 Energy and Sustainability Telemetry

Many industrial deployments report energy consumption and
emissions telemetry alongside production telemetry to support the
operator's ISO 14001-aligned environmental management and any
applicable greenhouse-gas reporting obligations.

```
energySample:
  sampleId        : string (uuidv7)
  siteId          : string (uuidv7)
  capturedAt      : string (ISO 8601)
  intervalDurationS : integer
  electricityKwh  : number
  thermalEnergyKwh : number (heating / cooling consumption when
                       metered)
  fuelConsumption : object (per-fuel volumes; e.g. natural gas in
                       cubic metres, diesel in litres)
  scope1Tco2e     : number (direct emissions, when reportable)
  scope2Tco2e     : number (indirect electricity emissions)
  meterRef        : string (asset identifier of the meter that
                       produced the reading)
```

Energy and sustainability records are subject to the same
historian retention and time-synchronisation rules as process
telemetry.

## §11 Configuration Snapshot Record

Per-asset configuration snapshots capture the full configurable
state of a controller, drive, or edge gateway at a point in
time. Snapshots are content-addressed so that configuration
changes can be audited end-to-end.

```
configurationSnapshot:
  snapshotId      : string (uuidv7)
  assetId         : string (uuidv7)
  capturedAt      : string (ISO 8601 / RFC 3339)
  artefactRef     : string (content-addressed URI of the
                       configuration export)
  artefactDigest  : string (SHA-256)
  changedFromSnapshotId : string (uuidv7; absent for the first
                       snapshot)
  changeReasonRef : string (URI of the change-control record)
```

Snapshots are emitted at every commissioning, every firmware
update, every loop tuning revision, and on a calendar cadence
(typically weekly or monthly) so that drift between intentional
changes is observable.

## §12 Conformance

Implementations claiming PHASE-1 conformance emit each of the
records defined above for every operating site and honour the
content-addressing rules in §3-§9.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-industrial-iot
- **Last Updated:** 2026-04-27
