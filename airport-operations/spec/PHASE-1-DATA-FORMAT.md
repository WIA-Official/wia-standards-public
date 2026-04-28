# WIA-airport-operations PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-airport-operations
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format
layer for WIA-airport-operations. The standard
covers persistent record shapes for the lifecycle of
a commercial airport operator — the airport's
identification and movement-area inventory; the
runway and taxiway record under ICAO Annex 14 + Doc
9981 PANS-Aerodromes; the airfield-lighting record
under ICAO Annex 14 Volume I + Doc 9157
Aerodrome Design Manual Part 4; the airport-collaborative-
decision-making (A-CDM) milestone record; the
ground-handling record under IATA AHM 803/810/911/
913; the de-icing record under SAE AS 6286; the
runway-incursion-prevention record under ICAO Doc
9870 + EUROCAE ED-99; the apron-management record;
the bird-and-wildlife-hazard record under ICAO Doc
9137 Part 3; the airport-emergency-plan record
under ICAO Annex 14 Chapter 9 + Doc 9137 Part 7;
the safety-management-system (SMS) record under
ICAO Annex 19 + Doc 9859; the supervisory and
investigatory correspondence record. Records are
consumed by the airport authority, the airline
ground-handler, the air-navigation-service-provider
(ANSP), the national civil aviation authority, the
European Union Aviation Safety Agency (EASA), the
Federal Aviation Administration (FAA), the
International Civil Aviation Organization (ICAO),
the Korean Civil Aviation Authority + 인천국제공항
공사, and the airport's external auditors.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time representation)
- ISO/IEC 11578 (UUID) and IETF RFC 4122 (UUID URN)
- ISO/IEC 27001:2022 (information security management)
- IETF RFC 8259 (JSON), RFC 9457 (Problem Details)
- ICAO Annex 14 Volume I (Aerodromes — Aerodrome
  Design and Operations) — including Chapter 3
  (physical characteristics — runway, taxiway,
  apron); Chapter 5 (visual aids — markings, lights,
  signs); Chapter 6 (visual aids for denoting
  obstacles); Chapter 7 (visual aids for denoting
  restricted use of areas); Chapter 8 (electrical
  systems); Chapter 9 (aerodrome operational
  services, equipment and installations including
  emergency planning, ARFF, removal of disabled
  aircraft, wildlife hazard management); Chapter 10
  (aerodrome maintenance)
- ICAO Annex 14 Volume II (Heliports)
- ICAO Annex 19 (Safety Management) and Doc 9859
  (Safety Management Manual)
- ICAO Doc 9137 Airport Services Manual — Part 1
  (Rescue and Firefighting), Part 3 (Wildlife
  Hazard Management), Part 7 (Airport Emergency
  Planning), Part 9 (Airport Maintenance Practices)
- ICAO Doc 9157 Aerodrome Design Manual — Part 1
  (Runways), Part 2 (Taxiways, Aprons and Holding
  Bays), Part 4 (Visual Aids), Part 5 (Electrical
  Systems), Part 6 (Frangibility)
- ICAO Doc 9870 (Manual on the Prevention of
  Runway Incursions)
- ICAO Doc 9981 PANS-Aerodromes (Procedures for Air
  Navigation Services — Aerodromes)
- ICAO Doc 9774 (Manual on Certification of
  Aerodromes)
- ICAO Doc 4444 (PANS-ATM)
- IATA AHM 803 / 810 / 911 / 913 (Airport Handling
  Manual: Standard Ground Handling Agreement,
  Aircraft Handling Service Specification, Cabin
  Service Specification, Cabin Cleaning Service
  Specification)
- IATA Resolution 753 (Implementation of bag
  tracking)
- IATA RP 1750 (A-CDM)
- ACI APEX in Safety / APEX in Security operational
  excellence programmes
- SAE AS 6286 (Training and Qualification Program
  for Deicing/Anti-icing of Aircraft on Ground)
- EUROCAE ED-99 (Functional and Performance
  Requirements for Advanced Surface Movement
  Guidance and Control System (A-SMGCS))
- EUROCAE ED-87 (A-SMGCS Levels 1 and 2)
- ARINC 424 (Navigation System Database) + ARINC
  620 (DataLink Ground System Standard) + ARINC
  653 (Avionics Application Software Standard
  Interface)
- US FAA 14 CFR Part 139 (Certification of
  Airports) + AC 150/5210 series (Airport Safety,
  Operations and Training Advisory Circulars)
- KR 항공안전법 + KR 공항시설법 + KR 인천국제공항
  공사법 + 한국공항공사법 + KR 국토교통부 항공정책실
- ICAO Doc 9303 (Machine Readable Travel Documents,
  cited cross-domain to passenger-processing)

---

## §1 Scope

This PHASE defines persistent shapes for the
artefacts an airport operator (a national-airport-
authority, a private operator under concession, an
airline serving as airport operator at a small
airfield) maintains:

- The aerodrome reference record.
- The movement-area record (runways, taxiways,
  aprons).
- The airfield-lighting record.
- The A-CDM milestone record.
- The ground-handling record.
- The de-icing-and-anti-icing record.
- The runway-incursion / wildlife-hazard / FOD
  (foreign-object debris) record.
- The airport-emergency-plan record.
- The SMS hazard-and-occurrence record.
- The supervisory-correspondence record.

## §2 Programme Identifier

```
programmeId          : string (uuidv7)
operatorName         : string (legal name of the
                       airport operator)
icaoCode             : string (ICAO 4-letter
                       location indicator per ICAO
                       Doc 7910)
iataCode             : string (IATA 3-letter code)
operatorJurisdiction : string (ISO 3166-1)
governingFrameworks  : array of enum ("ICAO-ANNEX-
                       14-VOL-I" | "ICAO-ANNEX-14-
                       VOL-II" | "ICAO-ANNEX-19" |
                       "ICAO-DOC-9981-PANS-AGA" |
                       "ICAO-DOC-9774" |
                       "ICAO-DOC-9137-PT-1" |
                       "ICAO-DOC-9137-PT-3" |
                       "ICAO-DOC-9137-PT-7" |
                       "ICAO-DOC-9157-PT-1-2-4-5-6"
                       | "ICAO-DOC-9870" |
                       "ICAO-DOC-4444" |
                       "ICAO-DOC-9859" |
                       "IATA-AHM-803-810-911-913" |
                       "IATA-RP-1750-A-CDM" |
                       "IATA-RES-753" |
                       "ACI-APEX-SAFETY" |
                       "SAE-AS-6286" |
                       "EUROCAE-ED-99-A-SMGCS" |
                       "EUROCAE-ED-87" |
                       "US-FAA-14-CFR-PART-139" |
                       "KR-항공안전법" |
                       "KR-공항시설법" |
                       "user-defined")
arffCategoryIcao     : enum ("cat-1" | "cat-2" |
                       "cat-3" | "cat-4" | "cat-5"
                       | "cat-6" | "cat-7" | "cat-8"
                       | "cat-9" | "cat-10")
                       (ICAO Annex 14 §9.2.2 ARFF
                       category based on aircraft
                       length / fuselage width)
programmeStatus      : enum ("operating" | "limited-
                       rollout" | "wind-down" |
                       "archived" | "user-defined")
```

## §3 Movement-Area Record

```
runwayRecord:
  runwayId           : string (uuidv7)
  runwayDesignator   : string (e.g. "09/27", "15L/
                       33R")
  runwayLength       : number (metres)
  runwayWidth        : number (metres)
  surface            : enum ("asphalt" | "concrete"
                       | "grass" | "gravel" |
                       "user-defined")
  bearingStrength    : object (ACN-PCN per ICAO
                       Annex 14 §1.7 — pavement
                       classification number, tyre
                       pressure category, evaluation
                       method)
  approachClassification : enum ("non-instrument" |
                       "instrument-non-precision" |
                       "instrument-precision-cat-i"
                       | "instrument-precision-cat-
                       ii" | "instrument-precision-
                       cat-iiia" | "instrument-
                       precision-cat-iiib" |
                       "instrument-precision-cat-iiic")
  rwyEndsRef         : array of object (per-end
                       displaced-threshold offset,
                       stopway, clearway, overrun
                       area)

taxiwayRecord:
  taxiwayId          : string (uuidv7)
  taxiwayDesignator  : string
  taxiwayClass       : enum ("rapid-exit" | "exit"
                       | "high-speed" | "parallel"
                       | "apron-edge" | "user-
                       defined")
  surfaceWidth       : number (metres)

apronRecord:
  apronId            : string (uuidv7)
  apronDesignator    : string
  standCount         : integer (number of aircraft
                       stands)
  apronUseClass      : enum ("commercial-passenger"
                       | "general-aviation" |
                       "cargo" | "maintenance" |
                       "remote-de-icing" |
                       "user-defined")
```

## §4 Airfield-Lighting Record

```
lightingRecord:
  lightingId         : string (uuidv7)
  systemKind         : enum ("approach-lighting-
                       system-cat-i" | "approach-
                       lighting-system-cat-ii-iii"
                       | "papi" | "vasis" | "rwy-
                       edge" | "rwy-centerline" |
                       "rwy-touchdown-zone" | "rwy-
                       end" | "twy-edge" | "twy-
                       centerline" | "stop-bar" |
                       "user-defined")
  intensitySteps     : array of integer (per-step
                       intensity setting available)
  controlSourceRef   : string (the airport's CCR
                       constant-current regulator
                       reference)
  lastInspectionAt   : string (ISO 8601)
  inspectionResult   : enum ("conforming" | "non-
                       conforming-corrective-action-
                       opened" | "user-defined")
```

## §5 A-CDM Milestone Record

The Airport Collaborative Decision Making (A-CDM)
milestone record per IATA RP 1750 + Eurocontrol
A-CDM Implementation Manual:

```
acdmMilestone:
  flightId           : string (uuidv7)
  airlineCode        : string (IATA / ICAO airline
                       code)
  flightNumber       : string
  registration       : string (aircraft registration
                       per ICAO Annex 7)
  milestone          : enum ("ms-1-flight-plan-
                       activation" | "ms-2-eobt-
                       update" | "ms-3-take-off-
                       at-origin" | "ms-4-local-
                       radar-update" | "ms-5-final-
                       approach" | "ms-6-landing"
                       | "ms-7-in-block" | "ms-8-
                       ground-handling-start" |
                       "ms-9-tobt-update" | "ms-
                       10-target-startup-approval"
                       | "ms-11-startup-request" |
                       "ms-12-off-block" | "ms-13
                       -take-off" | "ms-14-flight-
                       plan-activation-departure"
                       | "ms-15-airborne" | "ms-16
                       -take-off-update")
  observedAt         : string (ISO 8601 with
                       at-least minute precision)
  source             : enum ("airline-doc" |
                       "ground-handler-msg" |
                       "ansp-msg" | "airport-
                       msg" | "automated-detection"
                       | "user-defined")
```

## §6 Ground-Handling Record

```
groundHandlingRecord:
  recordId           : string (uuidv7)
  flightRef          : string (PHASE-1 §5)
  serviceCategory    : enum ("ramp-supervision" |
                       "marshalling" | "passenger-
                       boarding-bridge-or-stairs" |
                       "baggage-handling" |
                       "loading-unloading-cargo" |
                       "fuelling" | "potable-water"
                       | "lavatory" | "ground-
                       power-unit" | "air-start"
                       | "pushback-tow" | "deicing"
                       | "user-defined")
  ahmReferenceClause : string (the IATA AHM clause
                       the service is performed
                       under)
  agentRef           : string (the ground-handling
                       agent's identity)
  startedAt          : string (ISO 8601)
  completedAt        : string (ISO 8601)
  iata753BagReconciliationRef : array of string
                       (IATA Resolution 753 bag-
                       tracking events for the
                       flight)
```

## §7 De-Icing and Anti-Icing Record

```
deicingRecord:
  recordId           : string (uuidv7)
  flightRef          : string
  deicingPadRef      : string (the de-icing pad /
                       remote stand identifier)
  fluidsApplied      : array of object (per-fluid
                       application — Type I / II /
                       III / IV per AEA / ISO
                       11075-11078 holdover-time
                       guidance, ratio, application
                       time)
  outsideAirTemperature : number (°C)
  precipitationKind  : enum ("snow" | "freezing-
                       rain" | "freezing-fog" |
                       "ice-pellets" | "rain-on-
                       cold-soaked-wing" | "frost"
                       | "user-defined")
  holdoverTimeStart  : string (ISO 8601)
  saeAs6286TrainingRef : string (the deicer's
                       SAE AS 6286 training-and-
                       qualification record)
  postDeicingCheckAt : string (ISO 8601)
```

## §8 Runway-Incursion / Wildlife-Hazard / FOD Record

```
incursionRecord:
  incursionId        : string (uuidv7)
  detectedAt         : string (ISO 8601)
  incursionKind      : enum ("category-a-collision-
                       avoidance" | "category-b-
                       significant-potential-for-
                       collision" | "category-c-
                       ample-time-and-distance" |
                       "category-d-no-immediate-
                       safety-consequence" |
                       "category-e-could-not-be-
                       assessed")
  affectedRunwayRef  : string (PHASE-1 §3)
  preliminaryFindingsRef : string (URI of the ICAO
                       Doc 9870 preliminary-findings
                       narrative)

wildlifeStrikeRecord:
  strikeId           : string (uuidv7)
  occurredAt         : string (ISO 8601)
  speciesRef         : string (the species
                       identification per the
                       airport's wildlife-hazard-
                       management identification
                       protocol)
  flightPhase        : enum ("take-off-roll" |
                       "initial-climb" | "approach"
                       | "landing-roll" | "taxi"
                       | "parked" | "user-defined")
  damageReportRef    : string (URI of the FAA
                       Form 5200-7 / ICAO IBIS
                       wildlife-strike report)

fodRecord:
  fodId              : string (uuidv7)
  detectedAt         : string (ISO 8601)
  detectedBy         : enum ("runway-inspection-
                       walk" | "vehicle-mounted-
                       sweep" | "automated-fod-
                       detection-system" | "pilot-
                       report" | "user-defined")
  fodLocationRef     : string (the runway / taxiway
                       reference + offset)
  removalAt          : string (ISO 8601)
```

## §9 Airport-Emergency-Plan Record

```
aepRecord:
  aepId              : string (uuidv7)
  aepKind            : enum ("aircraft-accident-on
                       -aerodrome" | "aircraft-
                       incident-in-flight" |
                       "structural-fire" |
                       "dangerous-goods-incident"
                       | "natural-disaster" |
                       "bomb-threat-or-unlawful-
                       interference" | "medical-
                       emergency" | "user-defined")
  arffResponseRef    : object (ARFF response time,
                       crew assignment, agent
                       deployed)
  drillCadence       : enum ("annual-full-scale"
                       | "biennial-tabletop" |
                       "quarterly-partial-
                       activation" | "user-defined")
  lastFullScaleDrillAt : string (ISO 8601)
```

## §10 SMS Hazard and Occurrence Record

```
smsRecord:
  recordId           : string (uuidv7)
  reportKind         : enum ("hazard-identification"
                       | "occurrence-report" |
                       "voluntary-reporter-program"
                       | "mandatory-occurrence-
                       reporting-eu-376-2014" |
                       "user-defined")
  reportedAt         : string (ISO 8601)
  riskAssessmentRef  : string (URI of the ICAO Doc
                       9859 risk-matrix assessment)
  correctiveActions  : array of object (planned
                       remediation, owner, due
                       date, completion status)
```

## §11 Conformance

Implementations claiming PHASE-1 conformance maintain
each of the records defined above for the operator's
aerodrome, exercise the SMS hazard-identification and
occurrence-reporting discipline, and preserve the
records under the operating jurisdiction's
recordkeeping discipline (FAA 14 CFR 139.301
recordkeeping; KR 항공안전법 보존; EU Reg 376/2014
mandatory-occurrence-reporting retention).

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-airport-operations
- **Last Updated:** 2026-04-28
