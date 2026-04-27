# WIA-indoor-air-quality PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-indoor-air-quality
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format layer for
WIA-indoor-air-quality. The standard covers exchange of indoor air
quality (IAQ) records among building operators, occupant-health
representatives, environmental engineers, mechanical contractors,
sensor and sampling-equipment vendors, and the public-health
authorities that publish IAQ guidance. The format captures site
identity and ventilation configuration, time-series IAQ
observations, episodic-sampling laboratory results, ventilation
performance verification, occupant-reported symptom data, source-
identification investigations, and remediation actions.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time representation)
- ISO 16000 series (Indoor Air — sampling and test-method
  standards covering general requirements, formaldehyde,
  VOCs, microbial contamination, particulate matter, radon, and
  related determinations)
- ISO 7726:1998 (ergonomics of the thermal environment —
  measurement of physical quantities)
- ISO 7730:2005 (ergonomics — analytical determination and
  interpretation of thermal comfort)
- ISO/IEC 17025:2017 (testing and calibration laboratories)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 11578 (UUID)
- IETF RFC 4122 (UUID URN)
- IETF RFC 8259 (JSON)
- IETF RFC 9457 (Problem Details)
- ASHRAE Standard 62.1 / 62.2 (Ventilation for Acceptable Indoor
  Air Quality; cited normatively for engineering definitions and
  outdoor-air calculation procedures)
- ASHRAE Standard 55 (Thermal Environmental Conditions for Human
  Occupancy)
- WHO Guidelines for Indoor Air Quality (2009 series and
  successors; cited normatively for population-health reference
  concentrations)

---

## §1 Scope

This PHASE document defines persistent shapes for the records that
flow during commissioning, occupied operation, post-incident
investigation, and refurbishment of indoor environments.
Implementations covered include:

- Building management systems that emit IAQ telemetry.
- Continuous IAQ-sensor packages (consumer-grade and
  professional-grade alike, distinguished by accreditation).
- Episodic sampling programmes performed by certified industrial
  hygienists or environmental laboratories.
- Mechanical contractors that perform ventilation verification.
- Public-health authorities that aggregate IAQ trends across the
  building stock.

Outdoor air quality, occupational exposure assessments inside
industrial process boundaries, and clinical air-quality programmes
inside medical isolation rooms are out of scope; they are
governed by adjacent WIA standards.

## §2 Site Identifier

```
siteId            : string (uuidv7)
siteRegisteredAt  : string (ISO 8601 / RFC 3339)
siteOperator      : string (institutional identifier of the
                       operating organisation; building owner or
                       facility manager)
siteFunction      : enum  ("residential" | "office" | "school" |
                       "childcare" | "hospitality" | "retail" |
                       "warehouse" | "transit-hub" | "religious" |
                       "library-archive" | "data-centre" |
                       "mixed-use")
constructionYear  : integer
totalFloorAreaM2  : number
ventilationStrategy : enum ("natural" | "mechanical-balanced" |
                       "mechanical-supply-only" |
                       "mechanical-extract-only" |
                       "hybrid" | "demand-controlled")
```

A site that comprises multiple ventilation zones emits per-zone
records (§3) referenced from this site record.

## §3 Ventilation Zone Record

```
ventilationZone:
  zoneId          : string (uuidv7)
  siteId          : string (uuidv7)
  designOccupancy : integer
  outdoorAirCfm   : number (per ASHRAE 62.1 calculation; absent
                       for naturally ventilated zones)
  recirculationFraction : number (0-1; absent for once-through
                       systems)
  filterStages    : array of FilterStage

FilterStage:
  stageId         : string
  filterClass     : enum ("MERV-7" | "MERV-8" | "MERV-11" |
                       "MERV-13" | "MERV-14" | "MERV-15" |
                       "MERV-16" | "HEPA-H13" | "HEPA-H14" |
                       "ULPA-U15" | "user-defined")
  installedAt     : string (ISO 8601 date)
  rotationDueAt   : string (ISO 8601 date)
```

Filter rotation events are recorded against the filter stage as
an append-only sequence so that rotation cadence is auditable.

## §4 Continuous IAQ Sample

```
iaqSample:
  sampleId        : string (uuidv7)
  zoneId          : string (uuidv7)
  capturedAt      : string (ISO 8601 / RFC 3339)
  pm10UgM3        : number
  pm25UgM3        : number
  pm1UgM3         : number
  co2Ppm          : number
  voctotalUgM3    : number
  formaldehydeUgM3: number
  no2UgM3         : number
  ozoneUgM3       : number
  radonBqM3       : number
  temperatureC    : number
  relativeHumidityPct : number
  occupancyEstimate : integer
  sensorPackageId : string (instrument register entry)
```

Continuous samples are emitted at the cadence the sensor package
supports; bulk uploads are accepted via the API in PHASE-2 §4.
Sensor packages categorise themselves as `consumer-grade`,
`professional-grade-non-accredited`, or
`accredited-laboratory-grade`; the categorisation governs which
endpoints accept the package's submissions for regulatory-relevant
records.

## §5 Episodic Sampling Laboratory Result

Episodic sampling — sorbent-tube VOC, passive formaldehyde,
particle-impactor microbial, settle-plate fungal, charcoal radon
— produces laboratory results that complement continuous
telemetry.

```
episodicSample:
  sampleId        : string (uuidv7)
  zoneId          : string (uuidv7)
  collectedAt     : string (ISO 8601)
  durationS       : integer (sampling duration; instantaneous
                       grabs use 0)
  method          : enum ("iso-16000-3-formaldehyde" |
                       "iso-16000-6-voc-tenax" |
                       "iso-16000-19-mould-impactor" |
                       "iso-16000-30-svoc" |
                       "iso-16000-32-radon" |
                       "iso-16000-37-pm" |
                       "user-defined-method")
  laboratoryId    : string (ISO/IEC 17025-accredited laboratory
                       identifier)
  measurand       : string (analyte identifier)
  resultUgM3      : number (for mass concentrations)
  resultBqM3      : number (for radon)
  resultCfuM3     : number (for microbial)
  uncertainty     : Uncertainty (type-A and type-B per JCGM 100)
  loqBelowFlag    : boolean (result below limit of quantification)
```

## §6 Ventilation Performance Verification Record

```
ventilationVerification:
  verificationId  : string (uuidv7)
  siteId          : string (uuidv7)
  performedAt     : string (ISO 8601)
  performedBy     : string (institutional identifier of the
                       commissioning agent)
  zonesVerified   : array of string (zone identifiers)
  outdoorAirMeasuredCfm  : number (sum across verified zones)
  outdoorAirRequiredCfm  : number (per ASHRAE 62.1 calculation)
  pressurisationStrategy : enum ("positive" | "negative" |
                       "neutral")
  airChangeRatePerHour   : number
  recommendations        : string (free text; redacted on export
                       when contains operator-confidential detail)
  reportRef       : string (content-addressed URI of the full
                       commissioning report)
```

## §7 Occupant Symptom Record (Optional)

Where occupants opt in to provide symptom feedback, the operating
organisation records the feedback under privacy-preserving controls.
Records carry only opaque tokens for occupants; clinical detail is
held in adjacent WIA occupational-health or clinical standards.

```
occupantSymptom:
  symptomId       : string (uuidv7)
  zoneId          : string (uuidv7)
  reportedAt      : string (ISO 8601)
  occupantToken   : string (opaque token; clinical identity in
                       adjacent standard)
  category        : enum ("headache" | "eye-irritation" |
                       "cough" | "shortness-of-breath" |
                       "skin-irritation" | "fatigue" |
                       "concentration-loss" | "odour" |
                       "thermal-discomfort")
  severity        : enum ("mild" | "moderate" | "severe")
  freeText        : string (redacted on export; restricted to
                       authorised occupational-health roles)
```

## §8 Source-Identification Investigation Record

```
sourceInvestigation:
  investigationId : string (uuidv7)
  siteId          : string (uuidv7)
  triggeredBy     : enum ("episodic-sample-exceedance" |
                       "occupant-complaint-cluster" |
                       "continuous-telemetry-alert" |
                       "scheduled-audit")
  startedAt       : string (ISO 8601)
  closedAt        : string (ISO 8601; absent until closed)
  hypothesisLog   : array of HypothesisEntry
  rootCause       : string (free text once concluded)
  remediationActionRef : string (URI of the remediation plan)
```

## §9 Remediation Action Record

```
remediation:
  actionId        : string (uuidv7)
  siteId          : string (uuidv7)
  initiatedAt     : string (ISO 8601)
  completedAt     : string (ISO 8601; absent until completed)
  category        : enum ("source-removal" | "ventilation-increase"
                       | "filter-upgrade" |
                       "occupant-relocation" | "moisture-mitigation"
                       | "maintenance-correction" |
                       "operational-policy-change")
  expectedEffect  : string (anticipated reduction or elimination
                       of the IAQ exceedance)
  postActionVerificationRef : string (URI of the post-action
                       verification, when performed)
```

## §10 Sensor Package Register

Every sensor package contributing samples to the API carries a
register entry that records its category (PHASE-3 §1), calibration
history, firmware version, and the analytes it covers. The
register is the audit anchor for sensor traceability.

```
sensorPackage:
  packageId       : string (uuidv7)
  vendorId        : string (institutional identifier)
  modelName       : string
  firmwareVersion : string (Semantic Versioning 2.0.0)
  category        : enum ("consumer-grade" |
                       "professional-grade-non-accredited" |
                       "accredited-laboratory-grade")
  analytesCovered : array of string (e.g. ["pm25", "co2",
                       "voctotal", "temperatureC"])
  calibrationHistory : array of CalibrationEntry

CalibrationEntry:
  performedAt     : string (ISO 8601)
  laboratoryId    : string (calibrating lab; absent for
                       consumer-grade packages)
  certificateRef  : string (URI of the calibration certificate)
  uncertainty     : Uncertainty (per JCGM 100; present for
                       accredited-laboratory-grade only)
```

Consumer-grade packages MAY omit the calibration history entirely;
their records are accepted into trend datasets but not into
regulatory-relevant decisions per PHASE-3 §1.

## §11 Standards-Reference Threshold Table

The standard publishes a reference threshold table that downstream
consumers MAY consume to drive alerting on continuous samples. The
table is content-addressed and is governed by the operating
programme's quality dossier.

```
thresholdTable:
  tableId         : string (uuidv7)
  publishedAt     : string (ISO 8601)
  jurisdictionScope : array of string (ISO 3166 country codes the
                       table applies to)
  references      : array of string (e.g. "WHO-IAQ-2010",
                       "ASHRAE-62.1-current")
  thresholds      : array of ThresholdEntry

ThresholdEntry:
  measurand       : string (e.g. "pm25UgM3", "co2Ppm",
                       "formaldehydeUgM3")
  averagingPeriod : enum ("instantaneous" | "1-hour" | "8-hour" |
                       "24-hour" | "annual")
  threshold       : number
  unit            : string
  basis           : enum ("guidance" | "regulatory" | "operator-
                       internal")
```

Threshold tables are referenced by alerting endpoints (PHASE-2 §4)
so that operators can subscribe to alerts driven by the published
guidance rather than by ad-hoc operator-internal numbers.

## §12 Thermal Comfort Observation (Optional)

Sites that integrate thermal-comfort observations under ISO 7730
emit per-zone thermal-comfort records that complement IAQ samples.
Thermal-comfort observations are not strictly IAQ but they are
collected by the same sensors at the same cadence and are
relevant to the occupant-symptom record (§7) where headache or
fatigue may correlate with thermal load rather than air-quality
specifically.

```
thermalComfortObservation:
  observationId   : string (uuidv7)
  zoneId          : string (uuidv7)
  capturedAt      : string (ISO 8601)
  airTemperatureC : number
  meanRadiantTemperatureC : number
  airSpeedMs      : number
  relativeHumidityPct : number
  metabolicRateMet  : number
  clothingInsulationClo : number
  pmv             : number (Predicted Mean Vote per ISO 7730)
  ppdPercent      : number (Predicted Percentage Dissatisfied)
```

## §13 Conformance

Implementations claiming PHASE-1 conformance emit each of the
records defined above for every operating site and honour the
content-addressing rules in §3-§9.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-indoor-air-quality
- **Last Updated:** 2026-04-27
