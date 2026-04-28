# WIA-energy-storage PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-energy-storage
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format
layer for WIA-energy-storage. The standard covers
persistent record shapes for the lifecycle of an
electrical-energy-storage (EES) operator — the
storage asset register (Li-ion NMC / LFP / NCA,
flow batteries, lead-acid, sodium-ion); the safety-
test certification record (UL 9540, UL 9540A
thermal-runaway test, IEC 62619, IEC 62933 series,
NFPA 855 fire-code compliance, UN 38.3 transport-
test); the battery-management-system (BMS) record;
the state-of-charge / state-of-health / state-of-
power record; the cycle-life and degradation tracking
record; the wholesale-market participation record;
the IEEE 1547.9 + IEEE 1679 grid-interconnection
record; the thermal-runaway / off-gas / fire-event
record; the second-life and recycling record; and
the supervisory and emergency-services correspondence
record. Whereas WIA-distributed-energy covers the
broader DER fleet and WIA-energy-cloud covers the
utility-cloud substrate, this standard focuses on
the storage-asset-specific safety, lifecycle, and
emergency-response artefacts that storage operators
maintain.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time representation)
- ISO/IEC 11578 (UUID) and IETF RFC 4122 (UUID URN)
- ISO/IEC 27001:2022 (information security management)
- IETF RFC 8259 (JSON), RFC 9457 (Problem Details)
- IEC 62933-1:2018 (Electrical Energy Storage —
  Vocabulary)
- IEC 62933-2-1:2017 (Unit parameters and testing
  methods — General specification)
- IEC 62933-2-2:2024 (Unit parameters and testing
  methods — Application and performance testing)
- IEC 62933-5-2:2020 (Safety requirements for grid-
  integrated EES systems — electrochemical-based
  systems)
- IEC 62619:2022 (Secondary cells and batteries
  containing alkaline or other non-acid electrolytes
  — Safety requirements for secondary lithium cells
  and batteries, for use in industrial applications)
- IEC 62620:2014 (Secondary cells and batteries —
  Performance and safety requirements)
- IEC 60086-1/-2/-4 (Primary batteries — General /
  Physical and electrical specifications / Safety)
- IEC 61960-3:2017 (Secondary lithium cells and
  batteries — Performance)
- IEEE 1679-2020 (Recommended Practice for the
  Characterization and Evaluation of Energy Storage
  Technologies in Stationary Applications)
- IEEE 1547.9-2022 (Recommended Practice for Storage
  Interconnection)
- IEEE 2030.2.1-2019 (Guide for Design, Operation,
  and Maintenance of Battery Energy Storage Systems
  for Stationary Applications)
- UL 9540 (Standard for Energy Storage Systems and
  Equipment)
- UL 9540A (Test Method for Evaluating Thermal
  Runaway Fire Propagation in Battery Energy
  Storage Systems — cell-, module-, unit-,
  installation-level tests)
- UL 1973 (Batteries for Use in Stationary and
  Motive Auxiliary Power Applications)
- NFPA 855 (Standard for the Installation of
  Stationary Energy Storage Systems)
- UN ST/SG/AC.10/11/Rev.7 (Manual of Tests and
  Criteria, including UN 38.3 lithium-battery
  transport tests)
- UN ECE Regulation R100.03 (electric powertrain
  including REESS)
- US DOE OE Energy Storage Program guidance
- US OSHA + KR 산업안전보건법 occupational-safety
  rules for storage installations
- KR KEPIC (Korea Electric Power Industry Code)
- KR 전기사업법 + KR 신·재생에너지 개발·이용·보급
  촉진법

---

## §1 Scope

This PHASE defines persistent shapes for the
artefacts an energy-storage operator (an
independent-power-producer, a behind-the-meter
storage owner, an aggregator, a utility-scale
storage developer, an EV-fleet manager, a storage-
as-a-service vendor) maintains:

- The storage asset register.
- The safety-test certification record.
- The BMS configuration record.
- The state-of-charge / state-of-health / state-of-
  power telemetry record.
- The cycle-life and degradation tracking record.
- The market-participation record.
- The grid-interconnection record.
- The thermal-runaway / off-gas / fire-event record.
- The second-life and recycling record.
- The supervisory correspondence record.

## §2 Programme Identifier

```
programmeId          : string (uuidv7)
operatorName         : string (legal name)
operatorRole         : enum ("independent-power-
                       producer" | "utility-scale-
                       storage-developer" |
                       "behind-the-meter-storage-
                       owner" | "storage-aggregator"
                       | "ev-fleet-manager" |
                       "storage-as-a-service" |
                       "user-defined")
operatorJurisdiction : array of string (ISO 3166-1)
governingFrameworks  : array of enum ("IEC-62933-1"
                       | "IEC-62933-2-1" |
                       "IEC-62933-2-2" |
                       "IEC-62933-5-2" |
                       "IEC-62619-2022" |
                       "IEC-62620" | "IEC-61960-3"
                       | "IEEE-1679-2020" |
                       "IEEE-1547-9-2022" |
                       "IEEE-2030-2-1-2019" |
                       "UL-9540" | "UL-9540A" |
                       "UL-1973" | "NFPA-855" |
                       "UN-38-3" | "UN-ECE-R100-03"
                       | "KEPIC" |
                       "KR-전기사업법" |
                       "KR-신·재생에너지법" |
                       "KR-산업안전보건법" |
                       "DOE-OE-ESS" |
                       "user-defined")
chemistryUsed        : array of enum ("li-ion-nmc" |
                       "li-ion-lfp" | "li-ion-nca"
                       | "li-ion-lto" | "vanadium-
                       redox-flow" | "zinc-bromine
                       -flow" | "iron-flow" |
                       "lead-acid" | "sodium-sulfur"
                       | "sodium-ion" | "saline-
                       water" | "user-defined")
programmeStatus      : enum ("design" | "commissioning"
                       | "operating" | "limited-
                       rollout" | "wind-down" |
                       "decommissioning" |
                       "archived")
```

## §3 Storage Asset Register Record

```
storageAsset:
  assetId            : string (uuidv7)
  assetName          : string
  installedCapacity  : object (kWh nominal energy +
                       kW nominal power; round-trip
                       efficiency at the operator's
                       reference conditions)
  c-Rate             : object (max charge / discharge
                       C-rate)
  voltageWindow      : object (min cell voltage,
                       max cell voltage at the
                       operating temperature range)
  manufacturerRef    : string (the manufacturer's
                       legal identity)
  modelRef           : string (the manufacturer's
                       model designation)
  serialNumber       : string
  installedAt        : object (ISO 8601 + facility
                       geographic identifier + ISO
                       3166-1 country)
  ratedTemperatureRange : object (min / max
                       operating ambient temperature)
  enclosureKind      : enum ("indoor-walk-in" |
                       "outdoor-walk-in" | "outdoor-
                       containerised" | "outdoor-
                       cabinet" | "rack-mount" |
                       "user-defined")
  fireSuppressionKind : enum ("water-mist" |
                       "clean-agent-fk-5-1-12" |
                       "co2-discharge" | "aerosol"
                       | "none" | "user-defined")
  ventilationKind    : enum ("natural" | "mechanical
                       -forced" | "deflagration-
                       venting-nfpa-68" | "user-
                       defined")
```

## §4 Safety-Test Certification Record

The certification record encodes the full safety
test programme:

```
certificationRecord:
  certificationId    : string (uuidv7)
  assetRef           : string (PHASE-1 §3)
  ul9540Ref          : string (URI of the UL 9540
                       certificate)
  ul9540aReportRef   : object (URIs of the cell-,
                       module-, unit-, installation-
                       level test reports — UL
                       9540A is a test method, not a
                       pass/fail certificate)
  iec62619Ref        : string (URI of the IEC
                       62619:2022 industrial Li-ion
                       safety certificate)
  iec62933_5_2Ref    : string (URI of the IEC
                       62933-5-2 grid-integrated
                       EES safety certificate)
  ul1973Ref          : string (URI of the UL 1973
                       battery cell / pack
                       certificate)
  un38_3Ref          : string (URI of the UN 38.3
                       transport-test certificate)
  nfpa855ComplianceRef : string (URI of the NFPA
                       855-aligned site permit /
                       fire-code compliance
                       attestation)
  authorityHavingJurisdictionRef : string (the local
                       authority having jurisdiction
                       — fire marshal, electrical
                       inspector — that approved the
                       installation)
  ahJrRef            : string (the operating
                       jurisdiction's analogue —
                       e.g., KR 소방청 + KR 한국
                       전기안전공사)
```

## §5 BMS Configuration Record

```
bmsConfiguration:
  bmsId              : string (uuidv7)
  assetRef           : string
  bmsArchitecture    : enum ("modular-master-slave"
                       | "centralised" |
                       "distributed-mesh" |
                       "user-defined")
  cellMonitoringResolution : enum ("per-cell" |
                       "per-pair" | "per-module")
  protectionThresholds : object (over-voltage,
                       under-voltage, over-
                       temperature, over-current,
                       cell-imbalance trip
                       thresholds)
  balancingMethod    : enum ("passive-resistive" |
                       "active-charge-shuttling" |
                       "user-defined")
  socEstimationAlgorithm : enum ("coulomb-counting"
                       | "open-circuit-voltage" |
                       "extended-kalman-filter" |
                       "particle-filter" |
                       "machine-learning" |
                       "user-defined")
  firmwareDigest     : string (the BMS firmware's
                       cryptographic digest;
                       firmware-update events are
                       audit-logged)
```

## §6 State-of-Charge / State-of-Health / State-of-
       Power Telemetry Record

```
stateRecord:
  recordId           : string (uuidv7)
  assetRef           : string
  reportedAt         : string (ISO 8601 with at-
                       least second precision)
  stateOfCharge      : number (percentage, 0-100)
  stateOfHealth      : number (percentage relative
                       to beginning-of-life nominal
                       capacity; IEEE 1679-2020
                       reference method)
  stateOfPower       : object (instantaneous
                       available charge / discharge
                       power kW)
  cellVoltageMin     : number (V)
  cellVoltageMax     : number (V)
  cellTemperatureMin : number (°C)
  cellTemperatureMax : number (°C)
  internalResistance : number (mΩ; per-cell weighted
                       average — degradation
                       indicator)
```

## §7 Cycle-Life and Degradation Record

```
cycleLifeRecord:
  recordId           : string (uuidv7)
  assetRef           : string
  reportingPeriod    : object (the reporting window
                       — typically monthly or
                       quarterly)
  fullEquivalentCycles : integer (sum of fractional
                       cycles in the reporting
                       window)
  energyThroughputKwh : number (cumulative kWh
                       discharged / charged)
  capacityRetention  : number (percentage of
                       beginning-of-life nominal
                       capacity)
  degradationModelRef : string (the degradation
                       model the operator uses for
                       end-of-life forecasting)
  warrantyEvent      : enum ("within-warranty" |
                       "warranty-claim-filed" |
                       "out-of-warranty" |
                       "user-defined")
```

## §8 Market Participation Record

```
storageMarketParticipation:
  participationId    : string (uuidv7)
  assetRef           : string
  marketRef          : enum ("ferc-rto-caiso" |
                       "ferc-rto-pjm" | "ferc-rto-
                       miso" | "ferc-rto-ercot" |
                       "ferc-rto-nyiso" | "ferc-rto
                       -isone" | "ferc-rto-spp" |
                       "kr-kpx-전력거래소" |
                       "behind-the-meter-only" |
                       "user-defined")
  productKinds       : array of enum ("energy-day-
                       ahead" | "energy-real-time" |
                       "regulation-up" | "regulation
                       -down" | "spinning-reserve" |
                       "non-spinning-reserve" |
                       "capacity" | "frequency-
                       response" | "black-start" |
                       "voltage-support" | "behind-
                       the-meter-arbitrage" |
                       "user-defined")
  ferc841Compliant   : boolean (FERC Order 841
                       storage market participation
                       compliance)
```

## §9 Thermal-Runaway / Off-Gas / Fire-Event Record

```
incidentRecord:
  incidentId         : string (uuidv7)
  assetRef           : string
  detectedAt         : string (ISO 8601)
  incidentKind       : enum ("cell-vent-no-
                       propagation" | "module-vent-
                       no-propagation" | "thermal-
                       runaway-single-cell" |
                       "thermal-runaway-propagation"
                       | "off-gas-detection-no-fire"
                       | "fire-suppression-discharged
                       -no-runaway" | "smoke-
                       detection-no-runaway" |
                       "user-defined")
  detectionSource    : enum ("voltage-anomaly" |
                       "temperature-anomaly" |
                       "off-gas-sensor" |
                       "smoke-detector" | "fire-
                       suppression-trigger" |
                       "external-observation" |
                       "user-defined")
  emergencyServicesNotifiedAt : string (ISO 8601;
                       absent unless services were
                       notified)
  postMortemAnalysisRef : string (URI of the
                       NFPA 855 + UL 9540A-aligned
                       post-mortem narrative)
  ahJrReportRef      : string (URI of the report
                       to the authority having
                       jurisdiction; absent unless
                       reportable)
```

## §10 Second-Life and Recycling Record

```
endOfLifeRecord:
  recordId           : string (uuidv7)
  assetRef           : string
  endOfLifeDecision  : enum ("second-life-
                       repurposed-stationary" |
                       "second-life-repurposed-
                       vehicle" | "recycle-
                       hydrometallurgical" |
                       "recycle-pyrometallurgical"
                       | "recycle-direct" |
                       "disposal-hazardous-waste"
                       | "user-defined")
  recyclerRef        : string (the recycler's legal
                       identity; recycler-licensing
                       per the operating jurisdiction
                       — KR 전기·전자제품 및 자동차의
                       자원순환에 관한 법률 + EU
                       Battery Regulation (EU)
                       2023/1542 where applicable)
  declaredRecoveryRate : object (the recycler's
                       declared per-material recovery
                       fraction — Li, Ni, Co, Cu)
```

## §11 Conformance

Implementations claiming PHASE-1 conformance maintain
each of the records defined above for every storage
asset the operator manages, exercise the UL 9540A
test reporting at the cell-, module-, unit-, and
installation-levels, satisfy the NFPA 855 site-
permit discipline, and preserve the records under
the operating jurisdiction's recordkeeping discipline.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-energy-storage
- **Last Updated:** 2026-04-28
