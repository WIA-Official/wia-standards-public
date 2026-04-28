# WIA-fire-resistant-material PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-fire-resistant-material
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format
layer for WIA-fire-resistant-material. The
standard covers the persistent record shapes that
a fire-resistant-material producer (a building-
material manufacturer of cementitious panels,
gypsum boards, intumescent coatings, mineral-wool
insulation, fire-rated glass, fire-rated doors
and door assemblies, fire-protective fabrics, or
flame-retardant polymers), a third-party fire-
testing laboratory, a notified body issuing the
EU Construction Products Regulation conformity
assessment, an Authority Having Jurisdiction
inspecting a building-code submission, a fire-
investigation expert, and a public-procurement
authority running a building-safety programme
maintain when classifying a material's fire
performance, registering its calibrated test
result against an internationally recognised
test method, declaring its reaction-to-fire and
resistance-to-fire performance, and tracking the
test-laboratory accreditation chain. Records are
consumed by the building designer integrating the
material into a fire-protection design, by the
building official auditing the construction-
product Declaration of Performance, by the
insurance underwriter pricing the building's risk
profile, and — where the product is sold across
borders — by the customs authority verifying the
classification reported on the commercial
invoice.

References (CITATION-POLICY ALLOW only):

- EN 13501-1:2018+A1:2019 (fire classification
  of construction products and building elements
  — reaction-to-fire) and EN 13501-2:2023
  (resistance-to-fire classification)
- EN 13823:2020+A1:2022 (Single Burning Item
  test for the reaction-to-fire classification of
  construction products other than floorings)
- EN ISO 11925-2:2020 (small-flame ignitability
  test for direct flame impingement)
- ISO 1182:2020 (non-combustibility test at
  750 °C)
- ISO 1716:2018 (determination of the gross
  heat of combustion — calorific potential)
- ISO 5660-1:2015 (cone calorimeter — heat
  release, smoke production, and mass loss rate)
  and ISO 5660-2:2002 (smoke production rate)
- ISO 9239-1:2010 (reaction-to-fire test for
  floorings — radiant heat source) and ISO
  9239-2:2002 (determination of burning
  behaviour at a heat flux level of 25 kW/m²)
- ISO 13943:2017 (fire-safety vocabulary)
- ASTM E84-23 (Standard test method for surface
  burning characteristics of building materials —
  the Steiner tunnel) and UL 723:2018 (the same
  test under UL's certification programme)
- ASTM E119-23a (standard test methods for fire
  tests of building construction and materials)
- ASTM E136-22 (non-combustibility of building
  materials at 750 °C)
- ASTM E2768-11(2018) (extended duration surface
  burning) and ASTM E2257-22 (room corner test)
- NFPA 251:2006 (fire endurance test method),
  NFPA 252:2017 (fire test of door assemblies),
  NFPA 257:2017 (fire test of window assemblies),
  NFPA 259:2018 (heat of combustion test),
  NFPA 268:2017 (radiant heat exposure)
- EN 1363-1:2020 (fire-resistance tests — general
  requirements), EN 1363-2:1999 (alternative and
  additional procedures), EN 1364 series (load-
  bearing element tests), EN 1365 series (load-
  bearing element tests for separating elements),
  EN 1366 series (service installation tests)
- KS F 2271:2016 (surface burning characteristics
  of building materials, the KR Steiner-tunnel
  test reference) and KS F 2257-1:2019 (fire-
  resistance tests for elements of building
  construction)
- IETF RFC 8259 (JSON), RFC 4122 (UUID), ISO
  8601 (date-time)
- ISO/IEC 27001:2022 (information-security
  management — used for the chain-of-custody
  record discipline in §8)
- EU Construction Products Regulation (EU)
  305/2011 (declared performance and CE marking
  for construction products), EU Commission
  Decision 2000/147/EC (classes for the
  reaction-to-fire performance), EU Commission
  Decision 2000/367/EC (classes for the
  resistance-to-fire performance)

---

## §1 Scope

This PHASE defines persistent shapes for the
artefacts exchanged when a fire-resistant
material is registered, tested, classified, and
declared on a Declaration of Performance.
Implementations covered include:

- A cementitious-panel manufacturer publishing
  the EN 13501-2 resistance-to-fire
  classification of the panel against a separating
  load-bearing element test under EN 1364.
- A gypsum-board manufacturer publishing the EN
  13501-1 reaction-to-fire classification (A2-
  s1, d0) anchored to ISO 1716, ISO 1182, and
  EN 13823 test reports.
- A mineral-wool insulation manufacturer
  publishing the same classification anchored to
  the same test set.
- An intumescent-coating manufacturer publishing
  EN 1363-1 / EN 13381-series fire-protection
  performance against structural-steel exposure.
- A fire-rated-door-assembly manufacturer
  publishing the EN 1634 or NFPA 252 fire-
  resistance classification.
- A flame-retardant polymer manufacturer
  publishing the ASTM E84 surface-burning
  classification (Class A, B, or C) against the
  Steiner-tunnel test.

The reaction-to-fire envelope, the resistance-to-
fire envelope, and the surface-burning envelope
receive distinct encodings in this PHASE; the
additional safeguards required by each
classification regime are encoded in PHASE-3
§3.

## §2 Programme Identifier

```
programmeId          : string (uuidv7)
operatorName         : string (legal name of the
                       operator — manufacturer,
                       fire-testing laboratory,
                       notified body, certification
                       body, or public-procurement
                       authority)
operatorRole         : enum ("material-producer" |
                       "fire-testing-laboratory" |
                       "notified-body" |
                       "certification-body" |
                       "authority-having-
                       jurisdiction" | "public-
                       procurement-authority" |
                       "user-defined")
governingFrameworks  : array of enum ("EN-13501-1" |
                       "EN-13501-2" | "EN-13823" |
                       "EN-ISO-11925-2" |
                       "EN-1363-1" | "EN-1364" |
                       "EN-1365" | "EN-1366" |
                       "EN-1634" | "ISO-1182" |
                       "ISO-1716" | "ISO-5660-1" |
                       "ISO-5660-2" | "ISO-9239-1" |
                       "ISO-13943" | "ASTM-E84" |
                       "UL-723" | "ASTM-E119" |
                       "ASTM-E136" | "ASTM-E2257" |
                       "NFPA-251" | "NFPA-252" |
                       "NFPA-257" | "NFPA-259" |
                       "KS-F-2271" | "KS-F-2257-1"
                       | "EU-CPR-305-2011" |
                       "EU-Decision-2000-147" |
                       "EU-Decision-2000-367" |
                       "user-defined")
accreditationStatus  : object (the ISO/IEC 17025
                       accreditation reference for
                       fire-testing laboratory
                       operators, the ISO/IEC
                       17065 accreditation
                       reference for product-
                       certification bodies, and
                       the EU CPR notified-body
                       designation reference)
programmeStatus      : enum ("design" | "operating"
                       | "limited-rollout" |
                       "wind-down" | "archived")
```

## §3 Material Record

```
materialRecord:
  materialId         : string (uuidv7)
  identifierBindings : array of object (per-
                       jurisdiction product
                       identifiers — for example
                       the EU CPR Declaration of
                       Performance number, the
                       UL Listing identifier, the
                       FM Approvals identifier,
                       the KR 건설기술 진흥법 인증
                       번호 — each carrying the
                       issuing authority and the
                       scope of use)
  materialFamily     : enum ("cementitious-panel" |
                       "gypsum-board" |
                       "mineral-wool" |
                       "intumescent-coating" |
                       "fire-rated-glass" |
                       "fire-rated-door" |
                       "fire-rated-window" |
                       "fire-protective-fabric" |
                       "flame-retardant-polymer" |
                       "structural-steel-fire-
                       protection" | "user-
                       defined")
  thicknessOrDensity : object (the geometry-and-
                       composition declaration
                       per the relevant EU CPR
                       harmonised technical
                       specification)
  intendedUse        : enum ("interior-wall" |
                       "exterior-wall" |
                       "ceiling" | "floor" |
                       "roof" | "structural-
                       beam-column" |
                       "compartment-separation" |
                       "egress-route-finish" |
                       "user-defined")
```

## §4 Reaction-to-Fire Test Record (EN 13501-1)

```
rtfTestRecord:
  rtfTestId          : string (uuidv7)
  materialRef        : string (PHASE-1 §3 record
                       reference)
  testStandard       : enum ("ISO-1182" |
                       "ISO-1716" | "EN-13823" |
                       "EN-ISO-11925-2" |
                       "ISO-9239-1" | "ASTM-E84" |
                       "UL-723" | "user-defined")
  testLaboratory     : object (laboratory name +
                       the laboratory's ISO/IEC
                       17025 accreditation
                       reference + the laboratory's
                       fire-test report identifier)
  measurementResult  : object (per-method derived
                       quantities — ISO 1182
                       temperature rise (delta T)
                       in K, ISO 1716 gross heat
                       of combustion (PCS) in
                       MJ/kg, EN 13823 fire-
                       growth rate index
                       (FIGRA) in W/s, total
                       heat release in 600 s
                       (THR-600s) in MJ, smoke-
                       growth rate index (SMOGRA)
                       in m²/s², total smoke
                       production in 600 s
                       (TSP-600s) in m², lateral
                       flame spread (LFS) flag,
                       flaming droplets / particles
                       (FDP) flag for ISO 11925-2,
                       ASTM E84 Flame Spread
                       Index (FSI), Smoke Developed
                       Index (SDI))
  classification     : object (the Euroclass
                       declared per EU Commission
                       Decision 2000/147/EC —
                       A1, A2, B, C, D, E, F
                       primary class + s1/s2/s3
                       smoke class + d0/d1/d2
                       droplet class)
```

## §5 Resistance-to-Fire Test Record (EN 13501-2)

```
rfTestRecord:
  rfTestId           : string (uuidv7)
  materialRef        : string (PHASE-1 §3 record
                       reference)
  testStandard       : enum ("EN-1363-1" |
                       "EN-1364-1" | "EN-1364-2" |
                       "EN-1364-3" | "EN-1364-4" |
                       "EN-1365-1" | "EN-1365-2" |
                       "EN-1365-3" | "EN-1365-4" |
                       "EN-1366-1" | "EN-1366-3" |
                       "EN-1634-1" | "ASTM-E119" |
                       "NFPA-251" | "NFPA-252" |
                       "NFPA-257" | "KS-F-2257-1" |
                       "user-defined")
  testLaboratory     : object (laboratory name +
                       the laboratory's ISO/IEC
                       17025 accreditation
                       reference + the laboratory's
                       fire-test report identifier)
  measurementResult  : object (the EN 1363-1
                       failure criteria — load-
                       bearing capacity (R),
                       integrity (E), and
                       insulation (I) — together
                       with the time-to-failure
                       in minutes, the heating
                       curve declaration (ISO
                       834 standard cellulosic,
                       hydrocarbon, slow-heating,
                       external, tunnel), and
                       the corresponding ASTM
                       E119 endpoint reading
                       where the test was run
                       under the ASTM regime)
  classification     : object (the resistance-to-
                       fire class declared per EU
                       Commission Decision 2000/
                       367/EC — for example
                       REI 60, EI 90, R 120 —
                       carrying the time-to-
                       failure rounded to the
                       declaration interval)
```

## §6 Declaration-of-Performance Record (EU CPR)

```
dopRecord:
  dopId              : string (uuidv7)
  materialRef        : string (PHASE-1 §3 record
                       reference)
  harmonisedSpec     : string (the EU CPR
                       harmonised technical
                       specification reference —
                       the harmonised standard or
                       the European Assessment
                       Document)
  declaredPerformance : array of object (per-
                       essential-characteristic
                       declared performance — the
                       reaction-to-fire Euroclass,
                       the resistance-to-fire
                       class, the smoke-
                       production class, the
                       dangerous-substances
                       declaration)
  notifiedBodyRef    : string (the notified body
                       performing the EU CPR
                       conformity-assessment
                       system 1+ / 1 / 2+ / 3 / 4
                       task)
  ceMarkingDate      : string (ISO 8601 date)
```

## §7 Authority-Having-Jurisdiction Audit Record

```
ahjAuditRecord:
  auditId            : string (uuidv7)
  materialRef        : string (PHASE-1 §3 record
                       reference)
  ahjIdentifier      : string (the building
                       official, fire marshal, or
                       the KR 시·도 건축위원회
                       reference)
  auditOutcome       : enum ("conforms" |
                       "non-conforms" |
                       "conditional" | "user-
                       defined")
  buildingProjectRef : string (the building-permit
                       reference under which the
                       material is installed)
  auditDate          : string (ISO 8601 date)
```

## §8 Chain-of-Custody Record

```
custodyRecord:
  custodyId          : string (uuidv7)
  artefactRef        : string (the test record,
                       DoP record, or audit
                       record identifier)
  custodyEvent       : enum ("test-conducted" |
                       "test-report-issued" |
                       "classification-derived" |
                       "dop-published" |
                       "ce-marking-applied" |
                       "ahj-audit-completed" |
                       "withdrawal" | "user-
                       defined")
  eventTimestamp     : string (ISO 8601 date-time)
  performingParty    : string (legal entity)
  hashOfArtefacts    : string (SHA-256 hex digest)
```

## §9 Manifest

Implementations publish a signed manifest carrying
`standardSlug` (constant value "fire-resistant-
material"), `version`, `implementation`, the
operator's `accreditationStatus`, and the
`profile` declaration that selects which of the
optional records (reaction-to-fire, resistance-to-
fire, DoP, AHJ audit) the implementation supports.
The manifest is signed using a key whose public
part is published on the operator's
`.well-known/wia/fire-resistant-material/`
discovery endpoint declared in PHASE-2.
