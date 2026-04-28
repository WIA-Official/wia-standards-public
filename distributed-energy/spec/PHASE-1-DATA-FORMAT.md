# WIA-distributed-energy PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-distributed-energy
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format
layer for WIA-distributed-energy. The standard
covers persistent record shapes for the lifecycle of
a distributed-energy-resources (DER) operator —
the DER asset register (rooftop solar, behind-the-
meter battery storage, electric-vehicle supply
equipment, fuel-cell stationary installations, micro-
turbines, demand-response loads); the interconnection
record under IEEE 1547-2018 and the operating
jurisdiction's interconnection rules; the DER
controller and DERMS (Distributed Energy Resources
Management System) record; the IEC 61850-7-420 DER
information model record; the cybersecurity record
(IEC 62351 + NERC CIP-002~014); the IEEE 2030.5
Smart Energy Profile record for utility-DER
communication; the SunSpec Modbus record for
inverter-level monitoring; the OpenADR 2.0 demand-
response signal record; the OCPP 2.0.1 EV-supply
charging-station record; the Reg 1547.9 storage-
specific record; the UL 1741-SB / IEEE 1547.1-2020
type-test certification record; the disaster-and-
ride-through event record; and the supervisory and
balancing-authority correspondence record.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time representation)
- ISO/IEC 11578 (UUID) and IETF RFC 4122 (UUID URN)
- ISO/IEC 27001:2022 (information security management)
- IETF RFC 8259 (JSON), RFC 9457 (Problem Details)
- IEEE 1547-2018 (Standard for Interconnection and
  Interoperability of Distributed Energy Resources
  with Associated Electric Power Systems Interfaces)
  — including Category I / II / III performance
  classes, voltage / frequency ride-through profiles,
  voltage / frequency / power-quality / reactive-
  power requirements
- IEEE 1547.1-2020 (Conformance Test Procedures for
  Equipment Interconnecting Distributed Energy
  Resources with Electric Power Systems and
  Associated Interfaces)
- IEEE 1547.9 (Recommended Practice for Storage
  Interconnection)
- IEEE 2030.5-2018 (Smart Energy Profile, the IP-
  based application-layer protocol for utility-DER
  communication; the IEEE 2030.5 Common Smart
  Inverter Profile is the Rule-21 / Rule-1547 baseline
  in California and similar jurisdictions)
- IEC 61850-7-420:2021 (DER information model — the
  international DER object-model standard)
- IEC 61850-90-7 (object models for DER inverters)
- IEC 62351 series (Power systems management and
  associated information exchange — Data and
  communications security)
- IEC 61968 + IEC 61970 CIM (Common Information
  Model — utility-side ADMS / DERMS data model)
- IEC 62933-1 + IEC 62933-5-2 (Electrical Energy
  Storage — terminology and safety)
- IEC 62619:2022 (Industrial Li-ion battery safety)
- UL 1741-SB (Inverters, Converters, Controllers and
  Interconnection System Equipment for Use With
  Distributed Energy Resources — Supplement B for
  IEEE 1547-2018)
- UL 9540 + UL 9540A (energy-storage systems and
  thermal-runaway test method)
- SunSpec Alliance Modbus specification (the de facto
  inverter-level wire format)
- OpenADR Alliance OpenADR 2.0a / 2.0b (the demand-
  response signaling specification)
- Open Charge Alliance OCPP 2.0.1 (EV supply
  equipment to central-management-system protocol)
- NERC CIP-002 through CIP-014 (Critical
  Infrastructure Protection standards)
- US DOE OE / FERC Order 2222 (DER aggregation
  participation in wholesale markets) and FERC Order
  2003-D for generator interconnection
- NIST SP 800-82 Rev 3 (Operational Technology
  Security)
- IEC 60364-7-712 (electrical installations of solar
  photovoltaic power-supply systems)
- KR 신·재생에너지 개발·이용·보급 촉진법 (KR Renewable
  Energy Promotion Act) and 한국전력공사 분산전원
  계통연계 기준 (KEPCO DER interconnection technical
  standards) and KR KEPIC code

---

## §1 Scope

This PHASE defines persistent shapes for the
artefacts a distributed-energy operator (a behind-
the-meter prosumer, an aggregator, a utility's DER-
integration team, a Virtual Power Plant operator, a
DER-as-a-service vendor, an electric-vehicle charge-
network operator) maintains:

- The DER asset register.
- The interconnection record.
- The DER controller and DERMS record.
- The cybersecurity posture record.
- The smart-energy-profile and SunSpec record.
- The OpenADR / OCPP record.
- The storage-specific record.
- The certification record.
- The event-and-ride-through record.
- The supervisory correspondence record.

## §2 Programme Identifier

```
programmeId          : string (uuidv7)
operatorName         : string (legal name)
operatorRole         : enum ("prosumer" |
                       "aggregator" | "utility-der-
                       team" | "virtual-power-plant"
                       | "der-as-a-service" |
                       "ev-charge-network" |
                       "balancing-authority" |
                       "user-defined")
operatorJurisdiction : array of string (ISO 3166-1)
governingFrameworks  : array of enum ("IEEE-1547-
                       2018-CAT-I" | "IEEE-1547-
                       2018-CAT-II" | "IEEE-1547-
                       2018-CAT-III" |
                       "IEEE-1547-1-2020" |
                       "IEEE-1547-9" |
                       "IEEE-2030-5-2018" |
                       "IEC-61850-7-420-2021" |
                       "IEC-61850-90-7" |
                       "IEC-62351" |
                       "IEC-61968" | "IEC-61970-CIM"
                       | "IEC-62933-1" |
                       "IEC-62933-5-2" |
                       "IEC-62619-2022" |
                       "UL-1741-SB" |
                       "UL-9540" | "UL-9540A" |
                       "SUNSPEC-MODBUS" |
                       "OPENADR-2-0B" |
                       "OCPP-2-0-1" |
                       "NERC-CIP-002-014" |
                       "FERC-ORDER-2222" |
                       "FERC-ORDER-2003-D" |
                       "NIST-SP-800-82-R3" |
                       "KR-신·재생에너지법" |
                       "KEPCO-DER-계통연계" |
                       "KEPIC" | "user-defined")
programmeStatus      : enum ("design" | "operating"
                       | "limited-rollout" |
                       "wind-down" | "archived")
```

## §3 DER Asset Register Record

```
derAsset:
  assetId            : string (uuidv7)
  assetName          : string
  assetKind          : enum ("rooftop-solar-pv" |
                       "ground-mount-solar-pv" |
                       "wind-turbine-small-scale" |
                       "battery-energy-storage-
                       system-li-ion" |
                       "battery-energy-storage-
                       system-flow" |
                       "fuel-cell-stationary" |
                       "micro-chp" | "micro-turbine"
                       | "diesel-genset-emergency" |
                       "ev-supply-equipment-l2" |
                       "ev-supply-equipment-dcfc" |
                       "demand-response-load" |
                       "user-defined")
  ratedCapacity      : object (per-asset rated power
                       in kW + per-storage-asset
                       rated energy in kWh)
  manufacturerRef    : string (the manufacturer's
                       legal identity)
  modelRef           : string (the manufacturer's
                       model designation)
  serialNumber       : string
  installedAt        : object (ISO 8601 + facility
                       geographic identifier)
  ieee15472018Category : enum ("category-i" |
                       "category-ii" | "category-iii"
                       | "n/a-ev-supply" |
                       "n/a-demand-response" |
                       "user-defined")
  certifiedTo        : array of string (UL 1741-SB /
                       IEEE 1547.1-2020 conformance
                       test certificate references;
                       UL 9540 / 9540A for storage;
                       IEC 62619 for industrial Li-
                       ion)
```

## §4 Interconnection Record

```
interconnectionRecord:
  interconnectionId  : string (uuidv7)
  assetRef           : string (PHASE-1 §3)
  utilityServiceTerritoryRef : string (the utility's
                       service territory; for KR
                       this is KEPCO + the local
                       distribution operator)
  pointOfCommonCoupling : object (the POCC busbar
                       reference, voltage class,
                       and feeder identifier)
  studyRefSet        : array of object (the
                       interconnection-study
                       reports — feasibility study,
                       system-impact study,
                       facilities study)
  rideThroughProfile : enum ("ieee-1547-2018-cat-i"
                       | "ieee-1547-2018-cat-ii" |
                       "ieee-1547-2018-cat-iii" |
                       "user-defined")
  reactivePowerCapability : enum ("ieee-1547-2018-
                       cat-a" | "ieee-1547-2018-cat
                       -b" | "user-defined")
  voltageRegulationMode : enum ("constant-power-
                       factor" | "voltage-reactive
                       -power-vv" | "watt-power-
                       factor-wp" | "watt-var-wv" |
                       "user-defined")
  protectionSettingRef : string (URI of the relay /
                       inverter protection-setting
                       record covering anti-island
                       protection, over-/under-
                       voltage, over-/under-frequency
                       trip thresholds)
  approvedAt         : string (ISO 8601)
  inServiceFrom      : string (ISO 8601)
```

## §5 DER Controller and DERMS Record

```
derController:
  controllerId       : string (uuidv7)
  assetRefs          : array of string (the assets
                       under the controller's
                       authority)
  controlMode        : enum ("local-autonomous-
                       sunspec" | "local-autonomous-
                       ieee-2030-5" | "remote-
                       supervisory-derms" | "remote-
                       direct-utility" | "user-
                       defined")
  dermsRef           : string (the DERMS the
                       controller integrates with;
                       absent for local-autonomous
                       mode)

dermsRecord:
  dermsId            : string (uuidv7)
  cimAlignment       : enum ("iec-61968-cim" | "iec-
                       61970-cim" | "user-defined")
  marketParticipationRef : array of string (the
                       wholesale-market participation
                       references — FERC Order 2222
                       compliant aggregator
                       enrolment, KR 전력시장운영규칙
                       등록)
```

## §6 Cybersecurity Posture Record

The cybersecurity discipline aligns with NERC CIP
(US bulk-electric-system) and IEC 62351 (international
power-systems-security) and NIST SP 800-82 Rev 3
(OT security):

```
cybersecurityPosture:
  postureId          : string (uuidv7)
  cipImpactClass     : enum ("low" | "medium" |
                       "high" | "n/a-non-bes")
  applicableCipStandards : array of enum ("CIP-002"
                       | "CIP-003" | "CIP-004" |
                       "CIP-005" | "CIP-006" |
                       "CIP-007" | "CIP-008" |
                       "CIP-009" | "CIP-010" |
                       "CIP-011" | "CIP-013" |
                       "CIP-014")
  iec62351Profiles   : array of enum ("iec-62351-3"
                       | "iec-62351-4" |
                       "iec-62351-5" |
                       "iec-62351-6" |
                       "iec-62351-7" |
                       "iec-62351-8" |
                       "iec-62351-9" |
                       "user-defined")
  cryptoProfileRef   : string (the operator's TLS /
                       certificate-management
                       profile reference)
  patchManagementRef : string (the operator's OT
                       patch-management cadence
                       reference per NIST SP 800-82
                       Rev 3 §5.4)
```

## §7 IEEE 2030.5 + SunSpec Modbus Record

```
ieee20305Record:
  endpointId         : string (uuidv7)
  assetRef           : string
  utilityServerUri   : string (the utility's IEEE
                       2030.5 server endpoint)
  csipProfile        : enum ("csip-conformant" |
                       "csip-aus-conformant" |
                       "user-defined") (CSIP =
                       Common Smart Inverter
                       Profile)
  meterReadingResource : object (the EnergyMeter
                       function set reference)
  derResource        : object (the DER function
                       set reference)
  drlcResource       : object (the demand-response
                       and load-control function
                       set reference)

sunspecRecord:
  endpointId         : string (uuidv7)
  assetRef           : string
  modbusTransport    : enum ("modbus-rtu" | "modbus-
                       tcp" | "user-defined")
  sunspecModelMap    : array of object (per-Sunspec-
                       model-id reference and the
                       inverter's published register
                       map)
```

## §8 OpenADR 2.0 + OCPP 2.0.1 Record

```
openadrEvent:
  eventId            : string (uuidv7)
  vtnRef             : string (the OpenADR Virtual
                       Top Node reference)
  venRef             : string (the operator's Virtual
                       End Node reference)
  signalKind         : enum ("simple-level" |
                       "delta" | "multiplier" |
                       "price-relative" | "price-
                       absolute" | "user-defined")
  intervalSet        : array of object (per-interval
                       start, duration, signal value)
  receivedAt         : string (ISO 8601)
  optStatus          : enum ("opt-in" | "opt-out" |
                       "no-response")

ocppRecord:
  ocppEndpointId     : string (uuidv7)
  csmsRef            : string (Central System
                       reference)
  chargingStationRef : string
  ocppVersion        : enum ("2-0-1" | "1-6-j" |
                       "user-defined")
  iso15118Plug       : enum ("iso-15118-2-2014" |
                       "iso-15118-20-2022" |
                       "n/a")
  smartChargingProfileSet : array of object (the
                       OCPP 2.0.1 ChargingProfile
                       references)
```

## §9 Storage-Specific Record (IEEE 1547.9 + UL 9540)

```
storageRecord:
  storageId          : string (uuidv7)
  assetRef           : string
  chemistry          : enum ("li-ion-nmc" | "li-ion-
                       lfp" | "li-ion-nca" | "vanadium
                       -redox-flow" | "zinc-bromine"
                       | "lead-acid" | "user-defined")
  ratedEnergy        : object (kWh)
  ratedDischargeRate : object (kW)
  socOperatingRange  : object (min / max state-of-
                       charge percentage)
  thermalManagementRef : string (the storage's
                       thermal-management system
                       reference)
  ul9540aTestRef     : string (URI of the UL 9540A
                       cell-, module-, unit-, and
                       installation-level thermal-
                       runaway test report)
  ieee15479Compliance : boolean (the storage's
                       IEEE 1547.9 storage-
                       interconnection-recommended-
                       practice compliance)
```

## §10 Event and Ride-Through Record

```
eventRecord:
  eventId            : string (uuidv7)
  assetRef           : string
  eventKind          : enum ("voltage-ride-through"
                       | "frequency-ride-through" |
                       "anti-island-trip" |
                       "over-current-trip" |
                       "thermal-runaway" |
                       "communication-loss-fail-
                       safe" | "remote-curtailment-
                       received" | "remote-
                       disconnect-received" |
                       "user-defined")
  eventStartedAt     : string (ISO 8601 with at-
                       least millisecond precision)
  eventEndedAt       : string (ISO 8601)
  recordedWaveformRef : string (URI of the COMTRADE
                       waveform-record per IEEE
                       C37.111; absent for non-
                       waveform events)
  outcomeKind        : enum ("ride-through-
                       successful" | "trip-and-
                       reconnect" | "trip-no-
                       reconnect" | "manual-
                       intervention" | "user-
                       defined")
```

## §11 Conformance

Implementations claiming PHASE-1 conformance maintain
the asset register, the interconnection record, and
the cybersecurity posture record for every DER asset
the operator manages, exercise the IEEE 1547-2018
ride-through profiles applicable to the asset's
category, and preserve the records under the
operating jurisdiction's recordkeeping discipline
(NERC CIP retention; FERC retention; KR 신·재생에너지법
보존 의무).

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-distributed-energy
- **Last Updated:** 2026-04-28
