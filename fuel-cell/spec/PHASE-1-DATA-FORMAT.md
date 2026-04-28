# WIA-fuel-cell PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-fuel-cell
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format layer for
WIA-fuel-cell. The standard covers persistent record
shapes for fuel-cell systems deployed in stationary,
transport, portable, and back-up applications under the
IEC 62282 fuel-cell-technologies series — fuel-cell
stack registration, balance-of-plant (BoP) registration,
hydrogen-fuel-quality verification per ISO 14687, grid
interconnection per IEEE 1547 (where the fuel cell is
grid-coupled), explosion-protected (Ex) zones per IECEx
where the installation handles flammable hydrogen,
on-board hydrogen-vehicle compliance per UN GTR 13 (the
United Nations Global Technical Regulation 13 on
hydrogen and fuel-cell vehicles), commissioning
records, periodic inspection records, and incident
records. The format is consumed by fuel-cell system
manufacturers, system integrators, the operating
jurisdiction's authority having jurisdiction (AHJ), the
operating jurisdiction's grid system operator (where
the fuel cell is grid-coupled), the operating
jurisdiction's vehicle-type-approval authority (where
the fuel cell powers a vehicle), and the certification
body issuing IECEx, ISO 14687, and IEC 62282
attestations.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time representation)
- ISO/IEC 11578 (UUID)
- ISO/IEC 27001:2022 (information security management)
- ISO 14687 (hydrogen fuel quality; cited normatively
  for the fuel-quality grades the system requires —
  Grade A, B, C, D, E for various PEMFC, MCFC, SOFC,
  AFC and PAFC subtypes)
- ISO 19880-1 (gaseous hydrogen — fuelling stations)
  cited where the system is paired with a refuelling
  station
- ISO 22734 (hydrogen generators using water
  electrolysis); cited where the system pairs with an
  on-site electrolyser
- ISO 19887 (gaseous hydrogen — fuel system
  components for hydrogen-fuelled vehicles)
- IETF RFC 4122 (UUID URN)
- IETF RFC 8259 (JSON)
- IETF RFC 9457 (Problem Details)
- IEC 62282-2:2020 (fuel-cell modules)
- IEC 62282-3-100:2019 (stationary fuel-cell power
  systems — safety)
- IEC 62282-3-200:2015 (stationary fuel-cell power
  systems — performance test methods)
- IEC 62282-3-300:2012 (stationary fuel-cell power
  systems — installation)
- IEC 62282-4-101:2014 (fuel-cell power systems for
  industrial electric trucks)
- IEC 62282-6-100:2010 (micro fuel-cell power systems
  — safety)
- IEC 62282-6-200:2016 (micro fuel-cell power systems
  — performance test methods)
- IEC 62282-7-1 / 7-2 (single-cell and stack test
  methods)
- IEC 62282-8 (energy storage systems using fuel cells
  in reverse mode — regenerative fuel cells)
- IEEE 1547-2018 (interconnecting distributed energy
  resources with electric power systems); cited where
  the system is grid-coupled at distribution voltage
- IEEE 1547.1-2020 (test procedures for IEEE 1547
  conformance)
- IECEx (the IEC System for Certification to Standards
  Relating to Equipment for Use in Explosive
  Atmospheres) and the IEC 60079 series referenced by
  IECEx; cited normatively for hydrogen-handling Ex
  zones
- IEC 60079-10-1 (classification of areas — explosive
  gas atmospheres)
- IEC 60079-14 (electrical installations design,
  selection and erection)
- UN GTR 13 (Global Technical Regulation No. 13 on
  hydrogen and fuel-cell vehicles); cited where the
  fuel cell powers a vehicle subject to type approval
  under the UNECE 1958 Agreement
- UN ECE Regulation No. 134 (uniform provisions
  concerning the approval of motor vehicles and their
  components with regard to the safety-related
  performance of hydrogen-fuelled vehicles)
- US DOE Hydrogen Program (cited as a community-
  recognised technical reference for hydrogen
  technologies; non-binding)
- W3C Verifiable Credentials Data Model 2.0 (used in
  PHASE-4 for optional re-issuance)

---

## §1 Scope

This PHASE defines persistent shapes for the artefacts a
fuel-cell deployment manages. Implementations covered
include:

- Stationary fuel-cell power systems (IEC 62282-3
  series): PEMFC, SOFC, MCFC, PAFC, AFC.
- Industrial-electric-truck fuel-cell systems (IEC
  62282-4-101): forklifts and similar materials-
  handling vehicles.
- Micro fuel-cell power systems (IEC 62282-6 series):
  portable electronics back-up.
- Regenerative fuel-cell energy-storage systems
  (IEC 62282-8): reverse-mode operation.
- Vehicle fuel-cell systems (UN GTR 13 / UN R134): on-
  road vehicles.
- Auxiliary-power-unit fuel cells: rail, marine, and
  aviation auxiliary power.

Hydrogen-storage subsystem requirements (Type 1 / 2 / 3
/ 4 cylinders per ISO 11119 series) are referenced
through the BoP record in §4 but the cylinder-itself
records are out of scope of this PHASE; they are
governed by the hydrogen-storage standards series.

## §2 Programme Identifier

```
programmeId          : string (uuidv7)
operatorName         : string (legal name of the system
                       operator)
deploymentSite       : object (site identifier, ISO
                       3166-1 country, postal address,
                       AHJ identity)
applicationClass     : enum ("stationary-power" |
                       "stationary-chp" |
                       "stationary-backup" |
                       "industrial-electric-truck" |
                       "micro-portable" |
                       "regenerative-energy-storage" |
                       "vehicle-passenger" |
                       "vehicle-heavy-duty" |
                       "auxiliary-power-unit" |
                       "user-defined")
fuelCellChemistry    : enum ("pemfc" | "sofc" | "mcfc"
                       | "pafc" | "afc" | "user-
                       defined")
governingFrameworks  : array of enum ("IEC-62282-2" |
                       "IEC-62282-3-100" |
                       "IEC-62282-3-200" |
                       "IEC-62282-3-300" |
                       "IEC-62282-4-101" |
                       "IEC-62282-6-100" |
                       "IEC-62282-6-200" |
                       "IEC-62282-7-1" |
                       "IEC-62282-7-2" |
                       "IEC-62282-8" |
                       "ISO-14687" |
                       "ISO-19880-1" |
                       "IEEE-1547-2018" |
                       "IECEx-system" |
                       "UN-GTR-13" |
                       "UN-R134" |
                       "user-defined")
gridInterconnection  : enum ("islanded-only" |
                       "grid-paralleled-low-voltage" |
                       "grid-paralleled-medium-voltage"
                       | "vehicle-onboard" |
                       "user-defined")
hazardousAreaClassification : enum ("non-zoned" |
                       "iec-60079-zone-2" |
                       "iec-60079-zone-1" |
                       "iec-60079-zone-0" |
                       "user-defined")
programmeStatus      : enum ("design" |
                       "commissioning" | "operating" |
                       "limited-rollout" |
                       "planned-outage" |
                       "decommissioning" | "archived")
```

## §3 Fuel-Cell Stack Record

```
stack:
  stackId            : string (uuidv7)
  programmeId        : string (uuidv7)
  manufacturer       : string
  model              : string (manufacturer's model
                       designation)
  serialNumber       : string (manufacturer's serial)
  cellChemistry      : enum (PHASE-1 §2 enumeration)
  ratedPowerKw       : number (rated electrical output
                       at the manufacturer's rated
                       operating point)
  ratedVoltageVdc    : number
  fuelInletGrade     : enum ("iso-14687-grade-a" |
                       "iso-14687-grade-b" |
                       "iso-14687-grade-c" |
                       "iso-14687-grade-d" |
                       "iso-14687-grade-e" |
                       "user-defined") (the minimum
                       hydrogen-fuel-quality grade the
                       stack tolerates per the
                       manufacturer's data sheet)
  iec62282TestRef    : string (URI of the IEC 62282
                       test report — IEC 62282-7-1 / 7-2
                       single-cell-and-stack test
                       methods, IEC 62282-2 module
                       test, IEC 62282-3-200
                       performance test for stationary)
  endOfLifeCriteria  : object (degradation rate,
                       projected end-of-life voltage
                       drop, projected operating
                       hours)
  manufacturerWarrantyTerms : string (URI of the
                       warranty document)
```

## §4 Balance-of-Plant (BoP) Record

```
balanceOfPlant:
  bopId              : string (uuidv7)
  programmeId        : string (uuidv7)
  hydrogenStorageRefs : array of string (cylinder /
                       tank identifiers; cylinders are
                       governed by the hydrogen-storage
                       standards series, referenced
                       here only)
  hydrogenSupplyRail : enum ("pipeline-grid" |
                       "compressed-cylinder-bundle" |
                       "tube-trailer-replacement" |
                       "liquid-hydrogen-vapouriser" |
                       "on-site-electrolyser" |
                       "ammonia-cracker" |
                       "natural-gas-reformer" |
                       "user-defined")
  inverterRef        : string (URI of the IEEE 1547-
                       2018 conformance evidence for
                       grid-paralleled installations;
                       absent for islanded-only)
  thermalIntegration : enum ("electrical-only" |
                       "chp-thermal-recovery" |
                       "tri-generation" |
                       "user-defined")
  airSubsystem       : object (compressor or blower
                       reference, intake-filter
                       reference, exhaust-treatment
                       reference)
  waterManagement    : object (humidifier reference,
                       water-treatment reference,
                       discharge-disposal reference)
  exZoneClassification : enum (PHASE-1 §2 enumeration)
  iecExEquipmentRefs : array of string (per-equipment
                       IECEx Certificate of Conformity
                       references, where Ex zones
                       apply)
```

## §5 Fuel-Quality Record (ISO 14687)

```
fuelQuality:
  fuelQualityId      : string (uuidv7)
  programmeId        : string (uuidv7)
  sampledAt          : string (ISO 8601)
  sampleLocation     : enum ("supplier-handoff" |
                       "site-storage" |
                       "stack-inlet" |
                       "user-defined")
  iso14687Grade      : enum ("iso-14687-grade-a" |
                       "iso-14687-grade-b" |
                       "iso-14687-grade-c" |
                       "iso-14687-grade-d" |
                       "iso-14687-grade-e")
  contaminantPanel   : object (per-contaminant ppm
                       measured: total non-H2 gases,
                       water, total hydrocarbons, O2,
                       He, N2 + Ar, CO2, CO, total
                       sulphur, formaldehyde, formic
                       acid, ammonia, total halogenated
                       compounds, particulate
                       concentration)
  laboratoryRef      : string (laboratory identity and
                       ISO/IEC 17025 accreditation
                       reference)
  conformanceVerdict : enum ("conforming" |
                       "non-conforming-stack-tolerance-
                       exceeded" |
                       "non-conforming-supply-rejected")
  contaminantImpactNarrativeRef : string (URI of the
                       narrative when non-conforming;
                       contaminants on the ISO 14687
                       panel cause stack performance
                       degradation that may be
                       reversible or irreversible
                       depending on contaminant
                       species and exposure)
```

## §6 Grid-Interconnection Record (IEEE 1547)

For grid-paralleled installations (PHASE-1 §2
`gridInterconnection` of `grid-paralleled-low-voltage`
or `grid-paralleled-medium-voltage`):

```
gridInterconnection:
  interconnectionId  : string (uuidv7)
  programmeId        : string (uuidv7)
  ieee1547TestRef    : string (URI of the IEEE 1547.1-
                       2020 conformance test report)
  rideThroughCategory : enum ("category-i" |
                       "category-ii" |
                       "category-iii") (per IEEE 1547-
                       2018 abnormal-condition
                       performance categories)
  interoperabilityProtocol : enum ("ieee-1547-2018-
                       sunspec-modbus" |
                       "ieee-1547-2018-ieee-2030-5" |
                       "ieee-1547-2018-dnp3" |
                       "user-defined")
  utilityInterconnectionAgreementRef : string (URI of
                       the executed agreement with the
                       grid system operator)
  pccVoltageNominal  : number (point-of-common-coupling
                       nominal voltage)
  pccCapacityKva     : number
```

## §7 Vehicle-Onboard Record (UN GTR 13 / UN R134)

For vehicle-onboard installations (PHASE-1 §2
`applicationClass` of `vehicle-passenger`,
`vehicle-heavy-duty`, or `auxiliary-power-unit` for
on-board APU):

```
vehicleOnboard:
  vehicleOnboardId   : string (uuidv7)
  programmeId        : string (uuidv7)
  vehicleClass       : enum ("M1-passenger" |
                       "M2-light-bus" |
                       "M3-heavy-bus" |
                       "N1-light-commercial" |
                       "N2-medium-commercial" |
                       "N3-heavy-commercial" |
                       "rail" | "marine" |
                       "user-defined")
  unGtr13TestRef     : string (URI of the UN GTR 13
                       test report — crash, post-crash
                       fuel leakage, hydrogen-storage
                       container, fuel-system
                       integrity)
  unR134TypeApprovalRef : string (URI of the UN
                       Regulation No. 134 type-approval
                       certificate, where applicable)
  storageContainerSpec : object (working pressure —
                       typically 70 MPa for passenger
                       cars and 35-70 MPa for heavy
                       duty — and Type 4 / Type 3 /
                       Type 2 / Type 1 designation per
                       ISO 11119 series)
  fuelingProtocol    : enum ("sae-j2601-2020" |
                       "iso-19880-1-2020" |
                       "user-defined")
```

## §8 Commissioning Record

```
commissioning:
  commissioningId    : string (uuidv7)
  programmeId        : string (uuidv7)
  commissionedAt     : string (ISO 8601)
  ahjAcceptanceRef   : string (URI of the AHJ
                       acceptance — the operating
                       jurisdiction's permitting
                       authority for the application
                       class)
  iec62282InstallationRef : string (URI of the IEC
                       62282-3-300 installation
                       inspection record, for stationary
                       systems)
  iecExZoneAcceptanceRef : string (URI of the Ex zone
                       installation acceptance —
                       typically by an IECEx-recognised
                       inspection body, per IEC
                       60079-14 design verification
                       and IEC 60079-17 inspection)
  initialFuelQualityRef : string (PHASE-1 §5 record
                       reference — pre-energising
                       fuel-quality verification)
  performanceAcceptanceRef : string (URI of the IEC
                       62282-3-200 performance test
                       record — rated power, voltage
                       regulation, response time,
                       efficiency)
```

## §9 Periodic Inspection Record

```
periodicInspection:
  inspectionId       : string (uuidv7)
  programmeId        : string (uuidv7)
  inspectedAt        : string (ISO 8601)
  inspectionKind     : enum ("manufacturer-recommended-
                       maintenance" |
                       "iec-60079-17-ex-equipment-
                       inspection" |
                       "ahj-mandated-periodic-inspection"
                       | "fuel-quality-recheck" |
                       "post-incident-recheck" |
                       "user-defined")
  inspectorRef       : string (inspector identity and
                       qualification reference)
  observationsRef    : string (URI of the inspection
                       observations narrative)
  remediationActions : array of string (URIs of
                       remediation actions taken in
                       response to observations)
```

## §10 Incident Record

```
incident:
  incidentId         : string (uuidv7)
  programmeId        : string (uuidv7)
  detectedAt         : string (ISO 8601)
  incidentKind       : enum ("hydrogen-leak-detected" |
                       "hydrogen-flame-detected" |
                       "stack-fault" |
                       "bop-fault" |
                       "fuel-quality-out-of-spec" |
                       "grid-disconnection-event" |
                       "vehicle-collision-fuel-leak" |
                       "user-defined")
  severityClass      : enum ("near-miss" |
                       "operations-affecting" |
                       "personnel-injury" |
                       "environmental-release" |
                       "user-defined")
  containmentNarrativeRef : string (URI of the
                       containment-and-recovery
                       narrative)
  rootCauseNarrativeRef : string (URI of the root-
                       cause narrative; absent until
                       analysis complete)
  ahjNotifiedAt      : string (ISO 8601; absent unless
                       AHJ notification was required
                       per the operating jurisdiction's
                       incident-reporting threshold)
```

## §11 Conformance

Implementations claiming PHASE-1 conformance emit each
of the records defined above for every operating
programme, retain commissioning records for the
operating life of the system plus the operating
jurisdiction's records-retention horizon, and preserve
incident records and root-cause analyses per the AHJ's
required retention.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-fuel-cell
- **Last Updated:** 2026-04-28
