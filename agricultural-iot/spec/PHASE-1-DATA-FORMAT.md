# WIA-agricultural-iot PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-agricultural-iot
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format layer for
WIA-agricultural-iot. The standard covers persistent record
shapes for IoT-instrumented agriculture — field sensor
networks (soil moisture, soil temperature, soil EC,
canopy temperature, leaf wetness, microclimate, water-level,
tank-level), variable-rate application controllers (spray,
fertiliser, seed), greenhouse climate controllers,
livestock RFID and behaviour monitors, irrigation
controllers, and the agricultural management software
(FMIS) that consumes the records. The format is consumed by
farm operators, agronomy advisors, equipment OEMs (per
ISOBUS task controllers), agricultural software vendors,
and the regulators that oversee water-rights, pesticide
application, and animal welfare.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time representation)
- ISO 3166-1 (country codes)
- ISO/IEC 11578 (UUID)
- ISO 11783 series (Tractors and machinery for agriculture
  and forestry — serial control and communications data
  network — known as ISOBUS)
  - ISO 11783-1 (general standard for mobile data
    communication)
  - ISO 11783-7 (implement messages application layer)
  - ISO 11783-10 (task controller and management
    information system data interchange)
  - ISO 11783-11 (mobile data element dictionary)
- ISO 19156:2011 (Geographic information — Observations and
  measurements — O&M)
- ISO 19115-1 (Geographic information — Metadata)
- ISO 19111 (Geographic information — Referencing by
  coordinates)
- ISO/IEC 27001:2022 (information security management)
- IETF RFC 4122 (UUID URN)
- IETF RFC 7252 (CoAP)
- IETF RFC 7390 (Group Communication for CoAP)
- IETF RFC 8949 (CBOR)
- IETF RFC 8259 (JSON)
- IETF RFC 9457 (Problem Details)
- OASIS MQTT v5.0
- OASIS MQTT-SN (sensor-network MQTT for constrained
  agricultural devices)
- OGC SensorThings API 1.1 (canonical IoT data model for
  sensor observations)
- OGC SOS 2.0 (Sensor Observation Service legacy
  reference)
- W3C SSN / SOSA (Semantic Sensor Network ontology /
  Sensor, Observation, Sample, Actuator)
- AgGateway ADAPT (Agricultural Data Application
  Programming Toolkit; canonical interchange envelope for
  ISOBUS task data, OEM telemetry, FMIS-side ingest)
- FAO AGROVOC multilingual thesaurus (cited as the
  canonical agricultural-vocabulary reference for crop /
  livestock / activity classification)
- USDA Soil Taxonomy / Keys to Soil Taxonomy (cited as the
  reference soil-classification vocabulary; FAO World
  Reference Base for Soil Resources is the international
  alternative)
- LoRa Alliance LoRaWAN 1.1
- 3GPP TS 36.300 / TS 38.300 (NB-IoT / Cat-M)

---

## §1 Scope

This PHASE defines persistent shapes for the artefacts an
agricultural-IoT operator manages. Implementations covered
include:

- Field-sensor networks (soil-moisture probes, weather
  stations, leaf-wetness sensors, tank-level sensors,
  flow meters on irrigation lines).
- Variable-rate application controllers (sprayer
  controllers, fertiliser-spreader controllers, planter
  population-control modules, all communicating ISOBUS).
- Greenhouse climate controllers (vent / heat / shade /
  CO2-injection / irrigation controllers).
- Livestock monitoring (RFID ear-tag readers, rumen-bolus
  sensors, accelerometer-based behaviour monitors,
  in-line milk-quality analysers).
- Irrigation controllers (centre-pivot, drip-irrigation
  zone controllers, gate-and-canal controllers).
- Farm Management Information Systems (FMIS) ingesting the
  above through AgGateway ADAPT.

Direct chemistry of crop protection and food-safety
traceability after farm-gate are addressed in adjacent WIA
standards (WIA-crop-monitoring, WIA-food-traceability) and
are out of scope here.

## §2 Operation Identifier

```
operationId          : string (uuidv7)
operationOperator    : string (institutional identifier of
                         the farm operator or co-operative)
operationRegistered  : string (ISO 8601 / RFC 3339)
operatingDomain      : array of enum ("row-crop-grain" |
                         "row-crop-oilseed" |
                         "specialty-crop-vegetable" |
                         "specialty-crop-fruit" |
                         "viticulture-vineyard" |
                         "greenhouse-protected" |
                         "vertical-farming-indoor" |
                         "rangeland-livestock" |
                         "intensive-livestock-dairy" |
                         "intensive-livestock-poultry" |
                         "intensive-livestock-swine" |
                         "aquaculture-pond" |
                         "aquaculture-marine-net-pen" |
                         "user-defined")
jurisdictionScope    : array of string (ISO 3166-1 / 3166-2)
operationStatus      : enum ("draft" | "operating" |
                         "fallow-suspended" | "archived")
```

## §3 Field / Zone Geographic Reference

Geographic features (fields, irrigation zones, livestock
enclosures, sensor locations) follow ISO 19115-1 / 19111
referencing.

```
geoReference:
  refId              : string (uuidv7)
  operationId        : string (uuidv7)
  refKind            : enum ("field-boundary" |
                         "management-zone" |
                         "irrigation-block" |
                         "livestock-enclosure" |
                         "sensor-point" |
                         "structure-greenhouse")
  crsRef             : string (EPSG code or per-jurisdiction
                         CRS identifier)
  geometryEncoding   : enum ("geojson-rfc7946" |
                         "wkt" | "gml-3.2" |
                         "isoxml-iso-11783-10")
  geometryArtefactRef: string (content-addressed URI of
                         the geometry artefact)
```

## §4 Device Record

```
device:
  deviceId           : string (uuidv7)
  operationId        : string (uuidv7)
  manufacturerRef    : string (institutional identifier)
  modelNumber        : string
  hardwareRevision   : string
  firmwareRef        : string (content-addressed firmware URI)
  deviceClass        : enum ("soil-probe" |
                         "weather-station" |
                         "leaf-wetness-sensor" |
                         "tank-level-sensor" |
                         "flow-meter" |
                         "isobus-controller-implement" |
                         "isobus-task-controller-tractor" |
                         "greenhouse-controller-climate" |
                         "rfid-ear-tag-reader" |
                         "livestock-behaviour-collar" |
                         "irrigation-pivot-controller" |
                         "drip-zone-valve")
  radioStack         : array of enum ("lorawan-1.1" |
                         "nb-iot-cat-m" |
                         "5g-redcap" |
                         "wi-fi-classic" |
                         "wi-fi-halow" |
                         "ieee-802-15-4-zigbee" |
                         "wired-rs-485" |
                         "wired-isobus-can" |
                         "user-defined")
  ssnSosaProfileRef  : string (URI of the W3C SSN / SOSA
                         profile that describes the
                         device's properties / actions /
                         events)
  deviceStatus       : enum ("manufactured" | "provisioned"
                         | "deployed-active" | "fault" |
                         "decommissioned")
```

## §5 Observation Record (OGC SensorThings / O&M Aligned)

Observations follow OGC SensorThings API 1.1 and the
underlying ISO 19156:2011 Observations and Measurements
data model.

```
observation:
  observationId      : string (uuidv7)
  deviceRef          : string (device UUID)
  observedAt         : string (ISO 8601)
  observedProperty   : enum ("soil-moisture-vwc" |
                         "soil-temperature-celsius" |
                         "soil-electrical-conductivity" |
                         "soil-ph" |
                         "canopy-temperature-celsius" |
                         "air-temperature-celsius" |
                         "relative-humidity-percent" |
                         "leaf-wetness-binary" |
                         "leaf-wetness-minutes" |
                         "rainfall-mm" |
                         "wind-speed-ms" |
                         "wind-direction-deg" |
                         "solar-radiation-wm2" |
                         "tank-level-percent" |
                         "tank-level-litres" |
                         "irrigation-flow-lpm" |
                         "user-defined")
  numericValue       : number
  unitCode           : string (UN/CEFACT recommendation 20
                         common code or operator-extension)
  geoRef             : string (geo-reference UUID; absent
                         when the device's permanent
                         location is recorded against the
                         device record)
  qualityFlag        : enum ("nominal" | "estimated" |
                         "uncertain" | "out-of-range" |
                         "device-fault")
```

## §6 ISOBUS Task and TaskData Record

ISOBUS task controllers (per ISO 11783-10) exchange task
data with FMIS through AgGateway ADAPT envelopes that wrap
the ISOXML representation of the task.

```
isobusTask:
  taskId             : string (uuidv7)
  operationId        : string (uuidv7)
  taskKind           : enum ("planting" | "fertiliser-application"
                         | "spraying" | "harvesting" |
                         "tillage" | "irrigation" |
                         "soil-sampling")
  plannedStart       : string (ISO 8601)
  plannedEnd         : string (ISO 8601)
  actualStart        : string (ISO 8601; absent until
                         started)
  actualEnd          : string (ISO 8601; absent until
                         completed)
  prescriptionMapRef : string (content-addressed URI of
                         the ISOXML PRESCRIPTION-MAP for
                         variable-rate application; absent
                         for non-VR tasks)
  asAppliedMapRef    : string (content-addressed URI of
                         the ISOXML AS-APPLIED map after
                         completion)
  agroVocCropRef     : string (FAO AGROVOC URI for the
                         crop being managed)
  agroVocActivityRef : string (FAO AGROVOC URI for the
                         activity classification)
```

## §7 Livestock Animal Record

```
animal:
  animalId           : string (uuidv7)
  operationId        : string (uuidv7)
  rfidTag            : string (ISO 11784 / 11785 RFID code)
  speciesRef         : string (FAO AGROVOC URI for the
                         species)
  breedClassification: string (national breed-registry
                         identifier where applicable)
  sex                : enum ("female" | "male" |
                         "intersex" | "unknown")
  birthdate          : string (ISO 8601 date; precision
                         MAY be reduced when unknown)
  damRef             : string (animal UUID of the dam;
                         absent for foundation animals)
  sireRef            : string (animal UUID of the sire;
                         absent for foundation animals)
  registrationStatus : enum ("registered-pedigree" |
                         "commercial-only" |
                         "rescued-unknown-pedigree")
```

## §8 Irrigation Plan and Application Record

```
irrigationPlan:
  planId             : string (uuidv7)
  operationId        : string (uuidv7)
  zoneRef            : string (geo-reference UUID; the
                         zone that the plan irrigates)
  plannedStart       : string (ISO 8601)
  plannedDurationS   : integer
  plannedRateMmPerHr : number (water depth applied per
                         hour over the zone)
  appliedAt          : string (ISO 8601; absent until
                         applied)
  appliedDurationS   : integer
  appliedDepthMm     : number
  waterRightRef      : string (URI of the water-rights
                         allocation citation; the operator's
                         water-rights administrator binds
                         this to the operating jurisdiction's
                         water-rights register)
```

## §9 Soil Classification Record

```
soilClassification:
  classificationId   : string (uuidv7)
  geoRef             : string (geo-reference UUID; the
                         soil-sample point)
  classifiedAt       : string (ISO 8601)
  taxonomyScheme     : enum ("usda-soil-taxonomy-13ed" |
                         "fao-world-reference-base-2014" |
                         "national-soil-classification")
  taxonomyClass      : string (per-scheme taxonomic
                         classification; e.g.
                         "Mollisols / Hapludolls" for
                         USDA, "Chernozems" for WRB)
  observerRef        : string (operator-internal observer
                         token)
```

## §10 Pesticide Application Detail Record

```
pesticideApplication:
  applicationId      : string (uuidv7)
  isobusTaskRef      : string (ISOBUS task UUID this
                         application implements)
  productRegistrationRef : string (URI of the regulator's
                         product registration record)
  rateUnitCode       : string (UN/CEFACT rec.20 — e.g.
                         "L_per_HA" for litres per hectare,
                         "KGM_per_HA" for kg per hectare)
  rateValue          : number
  bufferZoneMetres   : number (per-product buffer distance
                         from sensitive receptors)
  reentryIntervalHours : number (per-product re-entry
                         interval)
  applicatorTokenRef : string (operator-internal applicator
                         certification token; clinical
                         identity in operator HR)
  applicatorCertificationRef : string (URI of the regulator's
                         applicator certification record)
  driftCardRefs      : array of string (URIs of drift card
                         placement and assessment records)
```

## §11 Animal Health Event Record

```
animalHealthEvent:
  eventId            : string (uuidv7)
  animalRef          : string (animal UUID)
  observedAt         : string (ISO 8601)
  eventKind          : enum ("vaccination" |
                         "treatment-administered" |
                         "veterinary-examination" |
                         "diagnostic-test" |
                         "calving-or-parturition" |
                         "weaning" | "movement-on-farm" |
                         "transport-off-farm" |
                         "death-on-farm" |
                         "slaughter")
  productRef         : string (URI of the registered
                         veterinary product where applicable)
  withdrawalPeriodDays : number (per-product withdrawal
                         period before slaughter or milk
                         use)
  veterinarianRef    : string (operator-internal vet token)
```

## §12 Conformance

Implementations claiming PHASE-1 conformance emit each of
the records defined above for every operating field and
honour the AgGateway ADAPT envelope per §6.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-agricultural-iot
- **Last Updated:** 2026-04-28
