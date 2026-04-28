# WIA-fusion-energy PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-fusion-energy
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format layer for
WIA-fusion-energy. The standard covers persistent record
shapes for fusion-energy facilities — research and
demonstration tokamaks, stellarators, magnetised target
fusion devices, inertial confinement fusion (ICF) /
laser-fusion facilities, and small-scale alternative-
concept devices — operating under the IAEA's safety-
guide series for fusion (Safety Guides SSG-77 / SSG-78
/ SSG-79 issued by the IAEA's Fusion Safety Working
Group through the IAEA Fusion Safety project), the
operating jurisdiction's nuclear-or-radiation-safety
regulator's safety case, the IEC 61508 functional-safety
discipline as adopted for protection-system electronics,
the ASME Boiler and Pressure Vessel Code where the
operating jurisdiction's regulator adopts BPVC for
fusion components, and the operating jurisdiction's
radiation-protection regime. The format is consumed by
the operating jurisdiction's safety regulator (US NRC,
UK ONR, EU national regulators per the Council
Directive 2009/71/Euratom basic-safety-standards
framework, JP NRA, KR NSSC), the IAEA where the
operating jurisdiction reports voluntary fusion-safety
information, the host site's safety committee, the
facility's safety organisation, and the facility's
external technical-safety reviewer.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time representation)
- ISO/IEC 11578 (UUID)
- ISO/IEC 27001:2022 (information security management)
- ISO 19115 (geographic information — metadata)
- IETF RFC 4122 (UUID URN)
- IETF RFC 8259 (JSON)
- IETF RFC 9457 (Problem Details)
- IAEA Safety Standards Series — General Safety
  Requirements (GSR Part 1 to 7) where the operating
  jurisdiction's regulator adopts the GSR framework
  for fusion (the IAEA fusion-safety project has
  developed Specific Safety Guides addressing
  fusion-specific safety topics, with SSG-77 / SSG-78
  / SSG-79 in publication tracking)
- IAEA TECDOC series and the ITER Organization's
  technical-safety reports (cited as community-
  recognised baselines for tokamak design and
  operation)
- IEA Fusion Implementing Agreement (cited where the
  operating jurisdiction participates in the
  International Energy Agency Fusion technology
  collaboration programme)
- ASME Boiler and Pressure Vessel Code (BPVC) Section
  III (rules for construction of nuclear facility
  components) where the operating jurisdiction's
  regulator adopts ASME BPVC for fusion components
- ASME B31.1 (power piping) where the operating
  jurisdiction adopts B31.1 for the fusion facility's
  piping systems
- ASME NQA-1 (quality assurance requirements for
  nuclear facility applications) where the operating
  jurisdiction's regulator adopts NQA-1 for fusion-
  facility quality assurance
- ANS-58 series (American Nuclear Society safety
  standards for nuclear facilities) where the
  operating jurisdiction adopts ANS standards in
  parallel with ASME
- IEC 61508-1 to -7 (functional safety of
  electrical/electronic/programmable electronic
  safety-related systems) where the operating
  jurisdiction adopts IEC 61508 for the protection
  system; IEC 61511 for the process-industry
  application of IEC 61508
- IEC 60880 (nuclear power plants — instrumentation
  and control important to safety — software aspects
  for computer-based systems performing category A
  functions)
- IEC 60709 (separation of redundant safety-classified
  channels) referenced through IEC 60880
- IAEA Safety Standards GS-R-3 / GSR Part 2 (leadership
  and management for safety)
- ICRP Publication 103 (recommendations of the
  International Commission on Radiological Protection)
- W3C Verifiable Credentials Data Model 2.0 (used in
  PHASE-4 for optional re-issuance)

---

## §1 Scope

This PHASE defines persistent shapes for the artefacts a
fusion-energy facility manages. Implementations covered
include:

- Magnetic-confinement-fusion devices: tokamaks
  (research, demonstration, prototype reactors),
  stellarators, spherical tokamaks, and reversed-field
  pinch and other alternative magnetic configurations.
- Inertial-confinement-fusion devices: laser-driven
  ICF facilities (the major direct-drive and
  indirect-drive configurations operated by national
  laboratories) and magnetised-target fusion devices.
- Plasma-based research devices that operate at
  parameters short of net energy gain (linear-
  geometry plasma physics test stands, plasma-
  material-interaction test stands).
- Pre-construction and construction-phase safety
  records for fusion-pilot-plant designs operating
  under the operating jurisdiction's regulatory
  pathway (e.g. US NRC's risk-informed performance-
  based regulatory framework for fusion, UK ONR's
  fusion-safety adoption, KR NSSC's pathway).

Disposal of fusion-derived activated materials is
addressed through the operating jurisdiction's
radioactive-waste regulator's records (a separate
record-keeping regime that this PHASE references but
does not duplicate).

## §2 Programme Identifier

```
programmeId          : string (uuidv7)
operatorName         : string (legal name of the
                       facility operator)
hostSite             : object (site identifier, ISO
                       3166-1 country code, the
                       operating jurisdiction's
                       regulatory body identity)
deviceClass          : enum ("tokamak" | "stellarator"
                       | "spherical-tokamak" |
                       "reversed-field-pinch" |
                       "inertial-confinement-laser-
                       direct-drive" | "inertial-
                       confinement-laser-indirect-
                       drive" | "magnetised-target-
                       fusion" | "plasma-physics-test-
                       stand" | "pilot-plant-design-
                       phase" | "user-defined")
operatingPhase       : enum ("design-basis-development"
                       | "construction" | "commissioning"
                       | "first-plasma-and-low-
                       performance-research" | "high-
                       performance-research" | "pilot-
                       plant-net-energy-research" |
                       "decommissioning" | "archived")
governingFrameworks  : array of enum ("IAEA-GSR-Part-
                       1-to-7" | "IAEA-fusion-SSG-77-
                       78-79" | "ASME-BPVC-Section-III"
                       | "ASME-NQA-1" | "IEC-61508" |
                       "IEC-60880" | "ANS-58-series" |
                       "operating-jurisdiction-
                       fusion-licensing-pathway" |
                       "user-defined")
fuelCycle            : enum ("hydrogen-only-research-
                       no-tritium" | "deuterium-only-
                       research-no-tritium" |
                       "deuterium-deuterium" |
                       "deuterium-tritium-low-
                       inventory" | "deuterium-tritium-
                       full-inventory" |
                       "alternative-fuel-research" |
                       "user-defined")
programmeStatus      : enum ("design" | "operating" |
                       "limited-rollout" |
                       "wind-down" | "archived")
```

## §3 Safety-Case Record

```
safetyCase:
  safetyCaseId       : string (uuidv7)
  programmeId        : string (uuidv7)
  publishedAt        : string (ISO 8601)
  effectiveFrom      : string (ISO 8601)
  effectiveUntil     : string (ISO 8601; absent until
                       superseded)
  designBasisRef     : string (URI of the device
                       design-basis-events analysis;
                       the analysis enumerates
                       postulated initiating events
                       and the device's response,
                       framing the safety functions
                       the protection system performs)
  hazardCategorisation : enum ("non-nuclear-fusion-
                       research" | "limited-tritium-
                       inventory-modest-hazard-
                       category" | "significant-
                       tritium-or-activation-inventory"
                       | "pilot-plant-pre-licensing-
                       framework" | "user-defined")
  regulatoryPathway  : enum ("us-nrc-risk-informed-
                       performance-based-fusion" |
                       "uk-onr-fusion-safety" |
                       "eu-member-state-basic-
                       safety-standards" |
                       "jp-nra-fusion-safety" |
                       "kr-nssc-fusion-safety" |
                       "user-defined")
  approvalRef        : string (URI of the operating
                       jurisdiction's regulator
                       approval; absent until
                       approval issued)
  publicSummaryRef   : string (URI of the public
                       summary version of the safety
                       case, where the operating
                       jurisdiction publishes the
                       summary)
```

## §4 Tritium-Inventory Record

For programmes whose `fuelCycle` includes tritium
(low-inventory or full-inventory):

```
tritiumInventory:
  inventoryId        : string (uuidv7)
  programmeId        : string (uuidv7)
  measurementAt      : string (ISO 8601)
  inventoryLocation  : enum ("tritium-storage-vessel"
                       | "fuel-cycle-loop" |
                       "vacuum-vessel-retention" |
                       "first-wall-retention" |
                       "facility-stack-release-
                       monitoring" | "off-site-
                       transfer-shipment" |
                       "user-defined")
  inventoryMassKg    : number (the operator's
                       reported total grams or
                       kilograms of tritium at the
                       location; tritium accountancy
                       is the central record-keeping
                       under the operating
                       jurisdiction's safeguards
                       regime)
  measurementMethodRef : string (URI of the methodology
                       reference — calorimetry,
                       beta-radiation in-line monitor,
                       gas chromatography, ion
                       chamber)
  uncertaintyClass   : enum ("better-than-1-percent"
                       | "1-to-5-percent" |
                       "5-to-10-percent" | "greater-
                       than-10-percent")
  reportingObligationRef : string (URI of the
                       operating jurisdiction's
                       tritium-accountancy reporting
                       obligation reference)
```

## §5 Postulated-Initiating-Event Record

```
postulatedEvent:
  eventId            : string (uuidv7)
  programmeId        : string (uuidv7)
  eventCategory      : enum ("loss-of-vacuum-with-air-
                       ingress" | "loss-of-coolant" |
                       "magnet-quench-without-
                       protection" | "tritium-bypass-
                       primary-confinement" | "in-
                       vessel-component-failure" |
                       "off-normal-plasma-disruption"
                       | "external-hazard-seismic-
                       wind-flood" | "loss-of-off-
                       site-power-prolonged" |
                       "user-defined")
  frequencyClass     : enum ("anticipated-operational-
                       occurrence" | "design-basis-
                       accident" | "beyond-design-
                       basis-accident" | "design-
                       extension-condition")
  consequenceCategoryRef : string (URI of the
                       consequence-category narrative
                       per the operating
                       jurisdiction's classification
                       — radiation dose to public,
                       worker dose, environmental
                       release)
  protectionFunctionsCited : array of string (the
                       safety functions the protection
                       system invokes for this event;
                       cross-references PHASE-1 §6)
```

## §6 Safety-Classified Component Record

```
safetyClassifiedComponent:
  componentId        : string (uuidv7)
  programmeId        : string (uuidv7)
  componentName      : string
  safetyClass        : enum ("safety-class-1" |
                       "safety-class-2" |
                       "safety-class-3" |
                       "non-safety-but-important-to-
                       safety" | "non-safety")
  asmeBpvcDesignCode : enum ("asme-bpvc-section-iii-
                       class-1" | "asme-bpvc-section-
                       iii-class-2" | "asme-bpvc-
                       section-iii-class-3" |
                       "asme-bpvc-section-viii-
                       division-1" | "asme-bpvc-
                       section-viii-division-2" |
                       "non-asme" | "user-defined")
  iec61508Sil        : enum ("sil-1" | "sil-2" |
                       "sil-3" | "sil-4" | "non-
                       safety-instrumented")
                       (where the component is part
                       of the protection system's
                       electrical / electronic /
                       programmable-electronic
                       chain)
  iec60880Category   : enum ("category-a" | "category-
                       b" | "category-c" | "non-
                       categorised")
                       (where the component is a
                       computer-based instrumentation-
                       and-control system performing
                       a safety function)
  qualificationRef   : string (URI of the
                       qualification record — type
                       test, analysis, operating-
                       experience, or seismic
                       qualification per the
                       operating jurisdiction's
                       seismic-qualification
                       standard)
```

## §7 Operating-Limit and Condition Record

```
operatingLimit:
  limitId            : string (uuidv7)
  programmeId        : string (uuidv7)
  limitName          : string
  limitKind          : enum ("safety-limit" |
                       "limiting-condition-for-
                       operation" | "surveillance-
                       requirement" | "design-feature-
                       declaration" | "administrative-
                       control" | "user-defined")
  limitNarrativeRef  : string (URI of the limit
                       narrative; the narrative
                       cites the design-basis
                       analysis and the protection
                       function that maintains the
                       limit)
  surveillanceFrequency : enum ("continuous" |
                       "per-shift" | "daily" |
                       "weekly" | "monthly" |
                       "quarterly" | "annual" |
                       "per-cycle" | "per-shutdown")
  responseOnExcursion : string (URI of the response
                       narrative — corrective
                       action, shutdown, regulatory
                       reporting per the operating
                       jurisdiction's reportable-
                       event threshold)
```

## §8 Plasma-Operation Record

```
plasmaOperation:
  shotId             : string (uuidv7) (the per-
                       discharge identifier the
                       operator uses for plasma-
                       physics record-keeping)
  programmeId        : string (uuidv7)
  performedAt        : string (ISO 8601)
  fuelMix            : enum (PHASE-1 §2 fuelCycle
                       enumeration plus per-shot
                       composition)
  plasmaParametersRef : string (URI of the per-shot
                       parameters — toroidal field,
                       plasma current, density,
                       temperature, energy, duration,
                       confinement-time-and-quality
                       indicators)
  protectionInvocationRef : string (URI of any
                       protection-function
                       invocations during the shot;
                       absent for nominal shots)
  reportableEventRef : string (URI of any reportable-
                       event record; absent unless
                       the shot triggered a
                       reportable event)
```

## §9 Reportable-Event Record

```
reportableEvent:
  eventId            : string (uuidv7)
  programmeId        : string (uuidv7)
  detectedAt         : string (ISO 8601)
  eventClassification : enum ("operational-occurrence-
                       no-safety-significance" |
                       "anticipated-operational-
                       occurrence-with-protection-
                       success" | "design-basis-
                       accident-prevented" |
                       "abnormal-tritium-release-
                       within-permit" |
                       "abnormal-tritium-release-
                       above-permit" | "personnel-
                       dose-above-investigation-
                       level" | "safety-system-
                       degradation-without-event" |
                       "user-defined")
  regulatorNotifiedAt : string (ISO 8601; absent if
                       the event does not meet the
                       operating jurisdiction's
                       reportable threshold)
  rootCauseNarrativeRef : string (URI; absent until
                       analysis complete)
```

## §10 Decommissioning Record

```
decommissioning:
  decommissioningId  : string (uuidv7)
  programmeId        : string (uuidv7)
  declaredAt         : string (ISO 8601)
  decommissioningPhase : enum ("planning-pre-
                       cessation-of-operations" |
                       "preparation-for-decommissioning"
                       | "active-decommissioning" |
                       "post-decommissioning-
                       monitoring" | "site-release")
  activatedComponentInventoryRef : string (URI of the
                       activated-component inventory
                       — first-wall, divertor,
                       blanket modules, structural
                       supports — derived from the
                       fusion-derived neutron-fluence
                       analysis)
  wasteRouteRef      : string (URI of the waste-
                       route declaration — the
                       operating jurisdiction's
                       radioactive-waste regulator
                       holds the canonical record of
                       the waste route, referenced
                       here)
  siteReleaseCriteriaRef : string (URI of the operating
                       jurisdiction's site-release
                       criteria; absent until
                       release planning commences)
```

## §11 Conformance

Implementations claiming PHASE-1 conformance emit each
of the records defined above for every operating
programme, retain safety-case records for the operating
life of the facility plus the operating jurisdiction's
records-retention horizon, preserve tritium-inventory
records per the operating jurisdiction's accountancy
rules, and preserve reportable-event records per the
operating jurisdiction's investigation-and-disclosure
rules.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-fusion-energy
- **Last Updated:** 2026-04-28
