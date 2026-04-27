# WIA-infrastructure-monitoring PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-infrastructure-monitoring
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format layer for WIA-
infrastructure-monitoring. The standard governs the records that an
accredited operator publishes when instrumenting and monitoring
civil-infrastructure assets — bridges, dams, levees, tunnels,
buildings of public significance, water-distribution mains,
transmission-tower foundations, transit viaducts, and harbour
walls — with structural-health-monitoring (SHM) sensor networks,
including strain gauges, accelerometers, fibre-optic distributed
sensors, displacement transducers, settlement monitors, tilt-meters,
crack monitors, acoustic-emission sensors, and corrosion sensors.

References (CITATION-POLICY ALLOW only):

- ISO 13822 (assessment of existing structures)
- ISO 16587 (mechanical vibration — performance parameters for
  the structural-health monitoring of fixed structures)
- ISO 4866 (vibration of fixed structures — measurement and
  evaluation)
- ISO 10816 (vibration severity for rotating machinery — used for
  monitored mechanical sub-assemblies)
- ISO 18649 (mechanical vibration — evaluation of measurement
  results)
- ISO 8601 (date and time)
- ISO/IEC 11578 (UUID)
- ISO/IEC 17025:2017 (testing and calibration laboratories)
- ISO/IEC 27001:2022 (information security management)
- IEC 61400-25 (communications for wind-turbine monitoring —
  applicable to monitored utility-scale wind assets)
- IEEE 1451 (smart-transducer interface)
- IETF RFC 4122 (UUID URN), RFC 8259 (JSON), RFC 9457 (Problem
  Details)
- OGC SensorThings API 1.1
- W3C Sensor APIs (W3C Recommendation)

---

## §1 Scope

This PHASE document defines the persistent shapes for the records
that an SHM operator exchanges across the lifecycle of a monitored
asset: sensor commissioning, calibration, time-synchronisation
binding, raw-sample window, derived-metric series, threshold
breach, alert, post-event capture, decommissioning, and archival.
It is intended for use by:

- Asset owners that operate SHM systems on safety-critical
  structures (dam owners, bridge owners, transit operators,
  utility owners).
- SHM contractors that install and operate sensor networks on
  behalf of asset owners.
- Calibration laboratories that issue ISO/IEC 17025 calibration
  certificates for the sensors used.
- Long-term archives that hold monitoring records beyond the
  operating life of the SHM system.

Out of scope are the asset-management records governed by WIA-
infrastructure (which this standard cross-references) and the
process-control SCADA loops governed by WIA-infrastructure-
integration (which this standard publishes telemetry into).

## §2 Sensor Identifier and Class

```
sensorId           : string (uuidv7)
operatorOrgId      : string (institutional identifier)
parentAssetRef     : string (WIA-infrastructure asset identifier
                     this sensor is bound to; may be a component
                     identifier within the parent asset)
sensorClass        : enum  ("strain-gauge-foil" |
                     "strain-gauge-vibrating-wire" |
                     "accelerometer-mems" |
                     "accelerometer-piezoelectric" |
                     "fibre-optic-fbg" |
                     "fibre-optic-distributed-strain" |
                     "fibre-optic-distributed-temperature" |
                     "displacement-lvdt" |
                     "displacement-laser" |
                     "settlement-magnetometer" |
                     "tilt-meter-electrolytic" |
                     "tilt-meter-mems" |
                     "crack-monitor-vibrating-wire" |
                     "crack-monitor-electrical" |
                     "acoustic-emission" |
                     "half-cell-corrosion" |
                     "linear-polarisation-corrosion" |
                     "user-defined")
manufacturerRef    : string (institutional manufacturer identifier)
modelName          : string
serialNumber       : string
nominalRange       : object
  rangeMin         : number
  rangeMax         : number
  rangeUnit        : string (SI; e.g. "microstrain", "g",
                     "mm", "degree", "millivolt")
sensitivity        : number (units of output per unit of input;
                     publication units recorded in
                     `sensitivityUnit`)
sensitivityUnit    : string
```

Sensor identifiers are opaque and are generated when the sensor
is commissioned. Identifier reuse is forbidden — sensors that are
removed from service retain their identifier permanently for
citation purposes; replacement sensors get a new identifier and a
`predecessor` reference if they replace a removed unit.

## §3 Spatial-Mounting Identity

```
mounting:
  mountingId      : string (uuidv7)
  sensorId        : string (uuidv7)
  parentComponentRef : string (WIA-infrastructure component
                       identifier of the structural component
                       this sensor is mounted to)
  mountingPosition : object
    x             : number
    y             : number
    z             : number
    crs           : string (EPSG code or local-frame reference;
                       local-frame references include the
                       parent component's structural axes)
  mountingMethod  : string (operator-controlled vocabulary —
                       "epoxy-bonded", "welded-stud", "bracket-
                       bolted", "fibre-embedded", "drilled-
                       inclinometer-casing", etc.)
  mountedAt       : string (ISO 8601)
  mountedBy       : string (institutional identifier of the SHM
                       contractor)
  validFromPeriod : object
    from          : string (ISO 8601)
    until         : string (ISO 8601, nullable)
```

Mounting records are versioned: every re-bond, re-position, or
post-event remount emits a new mounting record with
`validFromPeriod` set so that historical samples resolve to the
mounting that produced them.

## §4 Calibration Record

```
calibration:
  calibrationId    : string (uuidv7)
  sensorId        : string (uuidv7)
  laboratoryRef   : string (institutional identifier of the
                     ISO/IEC 17025-accredited laboratory)
  certificateContentAddress : string (URI of the calibration
                     certificate)
  calibrationDate : string (ISO 8601)
  validUntil      : string (ISO 8601)
  measurementUncertainty : object
    coverageFactor : number (per ISO/IEC Guide 98-3)
    expandedUncertaintyValue : number
    expandedUncertaintyUnit  : string
  traceabilityRef : string (national-metrology-institute reference
                     to which the calibration is traceable —
                     KRISS, NIST, NPL, PTB, BIPM equivalent)
```

Calibration records are immutable; revoked calibrations emit a
successor record with `supersedes` set so the chain is
auditable.

## §5 Time-Synchronisation Binding

```
timeSync:
  timeSyncId      : string (uuidv7)
  sensorId        : string (uuidv7)
  acquisitionUnitRef : string (institutional identifier of the
                     data-acquisition unit reading this sensor)
  syncSource      : enum  ("ntp-stratum-2" |
                     "ntp-stratum-3" |
                     "ptp-grandmaster" |
                     "gnss-disciplined-oscillator" |
                     "manual")
  syncSourceRef   : string (host or device identifier of the
                     reference)
  observedSkewMs  : number (last observation of clock skew vs
                     the reference)
  observedAt      : string (ISO 8601)
```

Time-synchronisation bindings record the *current* sync state.
For high-frequency dynamic measurements (modal-analysis
campaigns, blast monitoring) PTP grandmasters with sub-
microsecond precision are typical and are recorded as such.

## §6 Raw-Sample Window

Raw sensor samples accumulate at high cadence (kHz-range for
accelerometers, sub-Hz for static gauges). They are bundled into
*windows* whose envelopes are recorded separately from the bulk
sample data.

```
sampleWindow:
  windowId        : string (uuidv7)
  sensorId        : string (uuidv7)
  windowStart     : string (ISO 8601 / RFC 3339)
  windowEnd       : string (ISO 8601 / RFC 3339)
  sampleCount     : integer
  samplingRateHz  : number
  archivalContentAddress : string (URI of the bulk-sample archive
                     — operator-published format; the operator's
                     procedure register specifies the chosen
                     format, typically TDMS, MAT, HDF5, or a
                     SensorThings observation collection)
  envelope        : object
    minValue      : number
    maxValue      : number
    rmsValue      : number
    meanValue     : number
    standardDeviation : number
```

Raw-sample windows are content-addressable; superseded windows
(corrected for clock-skew, re-windowed for analysis) emit
successor records but do not overwrite earlier windows.

## §7 Derived-Metric Series

Derived metrics — natural-frequency estimates, modal-shape
coefficients, displacement double-integrations, RMS vibration,
crack-extent estimates — are recorded separately from the raw
windows because they can be re-derived if the analysis method
is updated.

```
derivedMetric:
  metricId        : string (uuidv7)
  sensorRefs      : array of string (sensor identifiers used)
  metricFamily    : enum  ("natural-frequency" |
                     "modal-shape" |
                     "displacement-double-integration" |
                     "vibration-rms-velocity" |
                     "vibration-rms-acceleration" |
                     "crack-extent-estimate" |
                     "settlement-cumulative" |
                     "tilt-cumulative" |
                     "corrosion-rate" |
                     "user-defined")
  windowRefs      : array of string (sample-window identifiers
                     used as input)
  derivationMethod : string (operator-published method reference
                     — paper citation, internal procedure, vendor
                     algorithm version)
  derivedValue    : number
  derivedUnit     : string
  uncertaintyEstimate : number (nullable)
```

## §8 Threshold-Breach Record

```
thresholdBreach:
  breachId        : string (uuidv7)
  metricRef       : string (derivedMetric identifier or sensor
                     envelope identifier)
  thresholdScheme : string (operator-published scheme — typically
                     bound to ISO 4866 vibration severity classes,
                     ISO 10816 vibration severity, or owner-
                     specific limit-state ladder)
  thresholdValue  : number
  thresholdUnit   : string
  observedValue   : number
  breachStart     : string (ISO 8601)
  breachEnd       : string (ISO 8601, nullable while ongoing)
  severity        : enum ("watch" | "alert" | "alarm" | "trip")
```

Threshold breaches at severity `alert` or higher emit alert
records (PHASE-1 §9) automatically.

## §9 Alert Record

```
alert:
  alertId         : string (uuidv7)
  breachRef       : string (thresholdBreach identifier)
  notifiedAt      : string (ISO 8601)
  notifiedRecipients : array of object (recipient identifier,
                     channel — email, SMS, paging, on-call rota)
  acknowledgedBy  : string (institutional identifier; nullable)
  acknowledgedAt  : string (ISO 8601, nullable)
  resolvedAt      : string (ISO 8601, nullable)
  resolutionContentAddress : string (URI of the resolution
                     report when resolved — inspection finding,
                     temporary-restriction notice, false-positive
                     re-classification)
```

## §10 Post-Event Capture

```
postEventCapture:
  captureId       : string (uuidv7)
  triggeringEvent : object
    eventType     : enum  ("seismic" | "wind" | "flood" |
                       "vehicle-impact" | "vessel-impact" |
                       "fire" | "blast" | "user-defined")
    eventReference : string (national-seismic-network event ID,
                       NWS storm ID, owner-recorded incident
                       identifier, etc.)
  capturedSensorRefs : array of string
  capturedWindowRefs : array of string
  postEventReportContentAddress : string
```

## §11 Decommissioning

```
decommissioning:
  decommissioningId : string (uuidv7)
  sensorId        : string (uuidv7)
  reason          : enum  ("end-of-service-life" |
                     "calibration-failure" |
                     "destroyed-by-event" |
                     "removed-during-rehabilitation")
  decommissionDate : string (ISO 8601)
  archivalDepositRef : string
```

## §12 Conformance

Implementations claiming PHASE-1 conformance emit each record
type for every sensor under their operation, hold the
calibration certificates and post-event reports at the content-
addresses recorded above, and honour identifier-permanence and
immutability rules in §2 / §4.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-infrastructure-monitoring
- **Last Updated:** 2026-04-28
