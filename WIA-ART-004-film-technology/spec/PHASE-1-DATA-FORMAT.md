# WIA-ART-004-film-technology PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-ART-004-film-technology
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format layer for
WIA-ART-004-film-technology. The standard covers persistent record
shapes for the hydrogen energy value chain — production
(electrolytic, steam-methane-reforming, autothermal-reforming,
biomass gasification, by-product), purification, compression,
liquefaction, transport (tube trailer, pipeline, marine
carrier, ammonia carrier, LOHC carrier), storage (compressed,
liquid, metal hydride, salt cavern), distribution (refuelling
stations, industrial offtake, residential blending), and
end-use (fuel cell vehicles, industrial heat, refining,
fertilizer, steel reduction). The format is consumed by
hydrogen producers, midstream operators, refuellers, gas
network operators, hydrogen-vehicle OEMs, and the regulators
that license the value chain.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time representation)
- ISO 4217 (currency codes)
- ISO 3166-1 (country codes)
- ISO/IEC 11578 (UUID)
- ISO 14687:2019 (hydrogen fuel quality — product
  specification)
- ISO 19880-1:2020 (gaseous hydrogen — fuelling stations —
  general requirements)
- ISO 19880-3 (gaseous hydrogen — fuelling stations —
  valves)
- ISO 19880-5 (gaseous hydrogen — fuelling stations —
  dispensers)
- ISO 19880-8 (gaseous hydrogen — fuelling stations — fuel
  quality control)
- ISO 22734:2019 (hydrogen generators using water electrolysis
  — industrial, commercial, and residential applications)
- ISO 13985:2006 (liquid hydrogen — land vehicle fuel tanks)
- ISO 13984:1999 (liquid hydrogen — land vehicle fuelling
  system interface)
- ISO 17268:2020 (gaseous hydrogen land vehicle refuelling
  connection devices)
- ISO 26142:2010 (hydrogen detection apparatus — stationary
  applications)
- IEC 60079 series (explosive atmospheres equipment)
- IEC 62282 series (fuel cell technologies — modules,
  stationary, portable, road-vehicle)
- IETF RFC 4122 (UUID URN)
- IETF RFC 8259 (JSON)
- IETF RFC 9457 (Problem Details)
- SAE J2601:2020 (fueling protocols for light-duty gaseous
  hydrogen surface vehicles)
- SAE J2799:2019 (hydrogen surface vehicle to station
  communications hardware and software)
- SAE J2719:2020 (hydrogen fuel quality for fuel cell
  vehicles)
- IEA Global Hydrogen Review (technology benchmark
  reference)
- CertifHy guarantee-of-origin scheme (cited as the
  reference low-carbon and renewable hydrogen disclosure
  framework adopted across the EU)

---

## §1 Scope

This PHASE defines persistent shapes for the artefacts a
WIA-ART-004-film-technology value chain operator manages.
Implementations covered include:

- Electrolyser plants (alkaline, PEM, AEM, solid-oxide).
- SMR / ATR plants with optional carbon capture.
- Hydrogen purification and compression facilities.
- Liquefaction and re-gasification terminals.
- Tube-trailer, pipeline, ammonia, and LOHC transport.
- Hydrogen storage facilities (compressed, liquid, metal
  hydride, salt cavern).
- Hydrogen refuelling stations (HRS) per ISO 19880-1.
- Industrial offtake operators (refining, ammonia, methanol,
  steel direct-reduction).
- Hydrogen-vehicle OEMs and fuel-cell-stack manufacturers.

End-use combustion in transport (rail, marine, aviation) is
out of scope where adjacent WIA standards already cover the
mode-specific records.

## §2 Programme Identifier

```
programmeId          : string (uuidv7)
programmeOperator    : string (institutional identifier of
                         the operator)
programmeRegistered  : string (ISO 8601 / RFC 3339)
valueChainSegments   : array of enum ("production-electrolysis"
                         | "production-smr-atr" |
                         "production-by-product" |
                         "purification" | "compression" |
                         "liquefaction" |
                         "transport-tube-trailer" |
                         "transport-pipeline" |
                         "transport-marine-cryogenic" |
                         "transport-ammonia-carrier" |
                         "transport-lohc-carrier" |
                         "storage-compressed" |
                         "storage-liquid" |
                         "storage-metal-hydride" |
                         "storage-salt-cavern" |
                         "refuelling-station" |
                         "industrial-offtake" |
                         "blending-into-gas-network")
jurisdictionScope    : array of string (ISO 3166-1)
guaranteeOfOriginRef : string (CertifHy or local equivalent
                         scheme reference for the operator's
                         registered hydrogen)
programmeStatus      : enum ("design" | "construction" |
                         "commissioning" | "operating" |
                         "decommissioning" | "archived")
```

## §3 Production Record

```
productionRecord:
  recordId           : string (uuidv7)
  programmeId        : string (uuidv7)
  facilityRef        : string (operator's facility identifier)
  productionPath     : enum ("alkaline-electrolysis" |
                         "pem-electrolysis" |
                         "aem-electrolysis" |
                         "solid-oxide-electrolysis" |
                         "smr-without-ccs" |
                         "smr-with-ccs" |
                         "atr-with-ccs" |
                         "biomass-gasification" |
                         "chlor-alkali-by-product" |
                         "user-defined")
  intervalStart      : string (ISO 8601)
  intervalEnd        : string (ISO 8601)
  hydrogenMassKg     : number (kg of H2 produced over the
                         interval)
  electricityInputMWh: number (for electrolysis paths)
  electricitySourceMix : object (per ISO 14064-1 emissions
                         accounting; renewable / nuclear /
                         grid-mix breakdown with per-source
                         MWh and per-source emissions
                         intensity)
  feedstockMassKg    : object (for SMR/ATR/biomass paths;
                         per-feedstock mass)
  capturedCo2Mass    : number (kg CO2 captured at the source;
                         present for CCS-equipped paths)
  carbonIntensity    : number (kgCO2eq per kgH2 over the full
                         well-to-gate scope per the operator's
                         declared LCA boundary)
```

## §4 Hydrogen Quality Record

Hydrogen for fuel-cell vehicle use follows ISO 14687:2019
Type I Grade D (gaseous hydrogen for road vehicles); other
end-uses follow lower or higher purity grades per the
end-user's specification.

```
hydrogenQuality:
  qualityId          : string (uuidv7)
  productionRecordRef: string (URI of the production batch
                         the assay characterises)
  sampledAt          : string (ISO 8601)
  qualityGrade       : enum ("ISO-14687-Type-I-Grade-A" |
                         "ISO-14687-Type-I-Grade-B" |
                         "ISO-14687-Type-I-Grade-C" |
                         "ISO-14687-Type-I-Grade-D" |
                         "ISO-14687-Type-II-Grade-D" |
                         "ISO-14687-Type-III" |
                         "industrial-99.5" |
                         "industrial-99.9" |
                         "user-defined")
  measuredImpurities : object (per-impurity ppm or ppb —
                         water, total hydrocarbons, oxygen,
                         helium, nitrogen, argon, carbon
                         dioxide, carbon monoxide, sulphur,
                         formaldehyde, formic acid, ammonia,
                         particulates per ISO 14687 limits)
  laboratoryRef      : string (ISO/IEC 17025-accredited
                         laboratory identifier)
  certificateRef     : string (URI of the test certificate)
```

## §5 Storage and Inventory Record

```
storageInventory:
  inventoryId        : string (uuidv7)
  facilityRef        : string (storage facility identifier)
  capturedAt         : string (ISO 8601)
  storageMode        : enum ("compressed-tube-bank" |
                         "compressed-vessel-700-bar" |
                         "compressed-vessel-350-bar" |
                         "liquid-bulk-cryogenic" |
                         "metal-hydride-bed" |
                         "salt-cavern" |
                         "lined-rock-cavern" |
                         "ammonia-carrier-bulk" |
                         "lohc-carrier-bulk")
  inventoryMassKg    : number
  workingPressureMpa : number
  temperatureKelvin  : number
  boilOffMassKg      : number (cumulative boil-off since
                         last reset; relevant for liquid
                         storage)
  boilOffRecaptureRef: string (URI of the boil-off recapture
                         record where applicable)
```

## §6 Refuelling Event Record (HRS)

Hydrogen refuelling stations operate under ISO 19880-1 and
follow SAE J2601 fueling protocols with SAE J2799 station-
to-vehicle communication.

```
refuellingEvent:
  eventId            : string (uuidv7)
  hrsFacilityRef     : string (refuelling station identifier)
  vehicleTokenRef    : string (opaque vehicle token; clinical
                         vehicle identity held in the
                         operator's CRM, never on this API)
  startedAt          : string (ISO 8601 / RFC 3339)
  endedAt            : string (ISO 8601)
  protocolUsed       : enum ("J2601-A70-MC-default" |
                         "J2601-A70-non-comm" |
                         "J2601-H70-comm" |
                         "J2601-T20-MC" |
                         "J2601-T40-MC" |
                         "user-defined")
  initialPressureMpa : number
  finalPressureMpa   : number
  dispensedMassKg    : number
  ambientTempCelsius : number
  precooledTempCelsius : number (precooler outlet temperature
                         per SAE J2601 envelope)
  qualityCertificateRef : string (URI of the per-batch
                         quality certificate per ISO 19880-8
                         and SAE J2719)
  outcome            : enum ("nominal" | "early-stop-pressure" |
                         "early-stop-temperature" |
                         "early-stop-fault" | "aborted")
```

## §7 Transport Record

```
transportEvent:
  eventId            : string (uuidv7)
  programmeId        : string (uuidv7)
  carrierKind        : enum ("tube-trailer-200-bar" |
                         "tube-trailer-500-bar" |
                         "liquid-tanker-cryogenic" |
                         "pipeline-shared-natural-gas" |
                         "pipeline-dedicated-h2" |
                         "ammonia-marine-carrier" |
                         "lohc-marine-carrier" |
                         "ammonia-rail-tanker")
  loadedAt           : string (ISO 8601)
  loadedMassKg       : number
  originFacilityRef  : string
  destinationFacilityRef : string
  unloadedAt         : string (ISO 8601; absent until
                         unloaded)
  unloadedMassKg     : number (absent until unloaded; should
                         match loaded mass less in-transit
                         losses)
  routeHazardClass   : enum ("UN-class-2-1-flammable-gas" |
                         "UN-class-2-3-toxic-gas-ammonia" |
                         "UN-class-9-misc-lohc")
```

## §8 Safety Incident Record

Hydrogen handling carries deflagration / detonation risk per
the operating jurisdiction's industrial-safety regime
(IEC 60079 explosive atmospheres equipment, NFPA 2 in the
US, Korean KGS Code in Korea, equivalent rules elsewhere).

```
safetyIncident:
  incidentId         : string (uuidv7)
  programmeId        : string (uuidv7)
  occurredAt         : string (ISO 8601)
  facilityRef        : string
  classification     : enum ("leak-detected-no-ignition" |
                         "leak-with-deflagration" |
                         "detonation" |
                         "low-temperature-exposure" |
                         "asphyxiation-hazard" |
                         "equipment-failure-no-release" |
                         "spill-cryogenic" |
                         "near-miss")
  severity           : enum ("informational" | "minor" |
                         "major" | "critical")
  releaseQuantityKg  : number (estimated H2 released; absent
                         where the incident did not involve
                         release)
  rootCauseRef       : string (URI of the root-cause
                         investigation report)
  regulatorNotificationRef : string (URI of the regulator
                         notification artefact)
```

## §9 Carrier-Conversion Record (Ammonia / LOHC)

Hydrogen carriers (ammonia, methylcyclohexane and other
LOHCs, methanol-as-carrier) move hydrogen across long
distances more economically than compressed or liquefied
H2. The carrier-conversion record captures the conversion
events at the originating and receiving terminals.

```
carrierConversion:
  conversionId       : string (uuidv7)
  programmeId        : string (uuidv7)
  facilityRef        : string (terminal identifier)
  capturedAt         : string (ISO 8601)
  conversionDirection: enum ("hydrogen-to-ammonia" |
                         "ammonia-to-hydrogen-cracking" |
                         "hydrogen-to-lohc-loading" |
                         "lohc-to-hydrogen-dehydrogenation" |
                         "hydrogen-to-methanol-synthesis" |
                         "methanol-to-hydrogen-reforming")
  inputHydrogenMassKg: number (when input is H2; absent
                         when input is the carrier)
  outputHydrogenMassKg : number (when output is H2; absent
                         when output is the carrier)
  carrierMassKg      : number (mass of the carrier on the
                         carrier side of the conversion)
  energyInputMWh     : number (process energy input;
                         dehydrogenation is endothermic and
                         requires significant heat)
  catalystRef        : string (URI of the catalyst-management
                         record where conversion uses
                         catalysts requiring lifetime
                         tracking)
```

## §10 Embrittlement-Coupon Record (Pipelines)

```
embrittlementCoupon:
  couponId           : string (uuidv7)
  pipelineRef        : string (pipeline segment identifier)
  installedAt        : string (ISO 8601)
  retrievedAt        : string (ISO 8601; absent until
                         retrieved)
  materialGrade      : string (per the pipeline's material
                         classification, e.g. "API 5L X60")
  exposureProfile    : object (per-period H2 partial pressure,
                         blending percentage, temperature)
  destructiveTestRef : string (URI of the laboratory's
                         destructive test report; absent
                         until tested)
  embrittlementVerdict : enum ("within-design-envelope" |
                         "envelope-exceeded-investigation" |
                         "remediation-required")
```

## §11 Conformance

Implementations claiming PHASE-1 conformance emit each of
the records defined above for every operating facility and
honour the ISO 14687:2019 quality grade in §4.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-ART-004-film-technology
- **Last Updated:** 2026-04-28
