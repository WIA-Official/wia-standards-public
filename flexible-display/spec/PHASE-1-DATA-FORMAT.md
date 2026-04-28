# WIA-flexible-display PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-flexible-display
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format
layer for WIA-flexible-display. The standard
covers the persistent record shapes that a
flexible-display device manufacturer (a foldable-
phone OLED display, a rollable-television
display, a bendable-tablet display, an automotive
curved cluster display, a wearable-device
display on a polymer substrate, a stretchable-
sensor display patch, an electronic-paper roll-
to-roll display), a contract module assembler, a
display-laboratory metrology service, a notified
body issuing the EU CE-marking conformity
assessment under the relevant directive, an
optical-performance characterisation laboratory,
and a public-procurement authority running an
electronic-equipment procurement programme
maintain when registering a flexible-display
device, classifying its mechanical-flexibility
profile, declaring its optical-performance
envelope under the SID International Display
Metrology Committee (IDMC) baseline, recording
the foldability and rollability life-cycle test
results, and tracking the environmental and
electromagnetic-compatibility certifications.
Records are consumed by the device-design team
integrating the display into a product, by the
quality-assurance team running the line
acceptance tests, by the warranty-claim
adjudicator processing a customer-return event,
and — where the device is sold across borders —
by the customs authority verifying the optical
classification on the commercial invoice.

References (CITATION-POLICY ALLOW only):

- IEC 62977 series (multimedia display
  measurement methods) — IEC 62977-1:2018
  (general definitions), IEC 62977-2-1:2018
  (colorimetry), IEC 62977-2-2:2017
  (electro-optical characteristics), IEC
  62977-2-3:2017 (uniformity), IEC 62977-3-1:
  2017 (high-dynamic-range characteristics)
- IEC 62715 series (flexible display devices) —
  IEC 62715-1-1:2013 (terminology and letter
  symbols), IEC 62715-5-1:2017 (mechanical
  stress test method), IEC 62715-6-1:2016
  (folding endurance test method)
- IEC 62341 series (OLED display) — IEC
  62341-1-2:2014 (terminology and letter
  symbols), IEC 62341-5-2:2018 (image quality
  measurement), IEC 62341-6-2:2017 (mechanical
  characteristics test method)
- IEC 61747 series (liquid crystal display
  module) — cited where the flexible display is
  an LCD module variant
- IEC TS 62687:2014 (display compatibility
  testing for digital-signage applications)
- ISO 9241 series (ergonomics of human-system
  interaction) — ISO 9241-307:2008 (analysis
  and compliance test methods for electronic
  visual displays), ISO 9241-303:2011 (electronic
  visual display requirements), ISO 9241-302:
  2008 (terminology), ISO 9241-306:2018 (field
  assessment methods for electronic visual
  displays)
- SID IDMS (Information Display Measurements
  Standard) version 1.03 published by the
  Society for Information Display International
  Display Metrology Committee (cited normatively
  for the per-measurement procedure carried by
  the §4 envelope)
- JEITA RC-9131 (Japanese Electronics and
  Information Technology Industries Association
  flexible-display industry standard cited where
  a JP-jurisdiction operator declares the
  industry-standard reference)
- KS C IEC 62715-1-1 (Korean adoption of IEC
  62715-1-1) and KS C IEC 62977-1 (Korean
  adoption of IEC 62977-1)
- IEC 61000 series (electromagnetic compatibility
  test methods)
- IEC 60950-1:2005+AMD2:2013 (information
  technology equipment — safety) and IEC 62368-1:
  2018+AMD1:2020 (audio / video, information
  and communication technology equipment —
  safety requirements, the modern successor)
- IETF RFC 8259 (JSON), RFC 4122 (UUID), ISO
  8601 (date-time)
- ISO/IEC 27001:2022 (information-security
  management — used for the chain-of-custody
  record discipline in §8)
- EU Low Voltage Directive 2014/35/EU and EU
  EMC Directive 2014/30/EU (cited where the
  device is in scope of CE marking)
- EU RoHS Directive 2011/65/EU (restriction of
  hazardous substances)
- EU REACH Regulation (EC) No 1907/2006

---

## §1 Scope

This PHASE defines persistent shapes for the
artefacts exchanged when a flexible-display
device is registered, characterised through the
SID IDMS measurement set, qualified through IEC
62715 mechanical stress and folding endurance
tests, declared in a product-of-record envelope,
and tracked through the chain-of-custody trail.
Implementations covered include:

- A foldable-phone display module characterised
  per IEC 62715-6-1 folding-endurance test (a
  documented number of fold cycles at the
  declared bend radius without optical
  degradation beyond the declared threshold).
- A rollable-television display characterised
  per IEC 62715-5-1 mechanical stress test (the
  rolled state, the unrolled state, and the
  transition states under controlled radius and
  speed).
- A bendable-tablet display characterised per
  ISO 9241-307 §5 measurement methods.
- An automotive curved-cluster display
  characterised per the ISO 9241-303 luminance
  uniformity, contrast, and reflectance methods.
- A wearable-device display on a polymer
  substrate characterised per the IEC 62341-5-2
  image-quality measurement.
- An electronic-paper roll-to-roll display
  characterised per the SID IDMS bistable-
  display measurement section.

The mechanical-flexibility envelope, the
optical-performance envelope, and the
environmental-reliability envelope receive
distinct encodings in this PHASE; the
additional safeguards required by each
certification regime are encoded in PHASE-3 §3.

## §2 Programme Identifier

```
programmeId          : string (uuidv7)
operatorName         : string (legal name of the
                       operator — display device
                       manufacturer, contract
                       module assembler, display-
                       metrology laboratory,
                       notified body, or
                       certification body)
operatorRole         : enum ("display-manufacturer"
                       | "contract-module-
                       assembler" | "metrology-
                       laboratory" | "notified-
                       body" | "certification-
                       body" | "user-defined")
governingFrameworks  : array of enum ("IEC-62977-1"
                       | "IEC-62977-2-1" |
                       "IEC-62977-2-2" |
                       "IEC-62977-2-3" |
                       "IEC-62977-3-1" |
                       "IEC-62715-1-1" |
                       "IEC-62715-5-1" |
                       "IEC-62715-6-1" |
                       "IEC-62341-1-2" |
                       "IEC-62341-5-2" |
                       "IEC-62341-6-2" |
                       "IEC-61747" |
                       "IEC-TS-62687" |
                       "ISO-9241-307" |
                       "ISO-9241-303" |
                       "ISO-9241-302" |
                       "ISO-9241-306" |
                       "SID-IDMS-1.03" |
                       "JEITA-RC-9131" |
                       "KS-C-IEC-62715-1-1" |
                       "KS-C-IEC-62977-1" |
                       "IEC-62368-1" |
                       "EU-LVD-2014-35-EU" |
                       "EU-EMC-2014-30-EU" |
                       "EU-RoHS-2011-65-EU" |
                       "user-defined")
accreditationStatus  : object (the ISO/IEC 17025
                       accreditation reference for
                       metrology laboratory
                       operators, the ISO/IEC
                       17065 accreditation
                       reference for product-
                       certification bodies, and
                       the EU notified-body
                       designation reference)
programmeStatus      : enum ("design" | "operating"
                       | "limited-rollout" |
                       "wind-down" | "archived")
```

## §3 Display Device Record

```
displayRecord:
  displayId          : string (uuidv7)
  identifierBindings : array of object (per-
                       jurisdiction product
                       identifiers — for example
                       the EU CE Declaration of
                       Conformity reference, the
                       UL Recognised Component
                       identifier, the KR 전파법
                       방송통신기자재등의 적합성평가
                       인증 번호 — each carrying
                       the issuing authority and
                       the scope of use)
  displayFamily      : enum ("oled-foldable" |
                       "oled-rollable" |
                       "oled-bendable" |
                       "oled-curved" |
                       "oled-stretchable" |
                       "micro-led-flexible" |
                       "mini-led-flexible" |
                       "lcd-flexible" |
                       "electronic-paper-flexible"
                       | "user-defined")
  substrate          : enum ("polyimide-PI" |
                       "polyethylene-
                       terephthalate-PET" |
                       "polyethylene-naphthalate-
                       PEN" | "thin-glass" |
                       "stainless-steel-foil" |
                       "thermoplastic-
                       polyurethane-TPU" | "user-
                       defined")
  resolution         : object (horizontal-pixel
                       count, vertical-pixel
                       count, sub-pixel
                       arrangement per IEC
                       62715-1-1)
  pixelPitchMicron   : number (per IEC 62977-1
                       §3 pixel-pitch definition)
  flexibilityClass   : enum ("rigid" | "curved-
                       fixed-radius" | "bendable"
                       | "rollable-cylindrical" |
                       "foldable" | "stretchable"
                       | "user-defined")
  declaredBendRadiusMm : number (the minimum
                       bend-radius in mm declared
                       by the manufacturer per
                       IEC 62715-5-1 / 62715-6-1)
  declaredFoldCycles : number (the rated number
                       of fold cycles the device
                       is qualified for, per IEC
                       62715-6-1)
```

## §4 Optical-Performance Record (SID IDMS)

```
opticalRecord:
  opticalRecordId    : string (uuidv7)
  displayRef         : string (PHASE-1 §3 record
                       reference)
  testStandard       : enum ("IEC-62977-2-1" |
                       "IEC-62977-2-2" |
                       "IEC-62977-2-3" |
                       "IEC-62977-3-1" |
                       "IEC-62341-5-2" |
                       "ISO-9241-307" |
                       "SID-IDMS-1.03" |
                       "user-defined")
  testLaboratory     : object (laboratory name +
                       the laboratory's ISO/IEC
                       17025 accreditation
                       reference + the laboratory's
                       optical-test report
                       identifier)
  measurementResult  : object (per-method derived
                       quantities — peak luminance
                       in cd/m^2, ANSI checkerboard
                       contrast ratio, full-on
                       full-off contrast ratio,
                       black-state luminance in
                       cd/m^2, color-volume
                       coverage in DCI-P3 / Rec
                       2020, JNCD per IEC 62977-2-1
                       §6, viewing-angle envelope
                       per ISO 9241-303 §6, gamma
                       per IEC 62977-2-2 §5,
                       reflectance per ISO 9241-303
                       §8)
  measurementGeometry : enum ("flat-state" |
                       "bent-at-declared-radius" |
                       "rolled-fully" |
                       "rolled-partially" |
                       "folded-fully" |
                       "folded-partially" |
                       "user-defined")
```

## §5 Mechanical-Reliability Record (IEC 62715)

```
mechanicalRecord:
  mechanicalId       : string (uuidv7)
  displayRef         : string (PHASE-1 §3 record
                       reference)
  testStandard       : enum ("IEC-62715-5-1" |
                       "IEC-62715-6-1" |
                       "IEC-62341-6-2" | "user-
                       defined")
  testLaboratory     : object (laboratory name +
                       ISO/IEC 17025 reference +
                       test-report identifier)
  testProtocol       : object (declared bend
                       radius in mm, fold-cycle
                       speed in cycles per
                       second, environmental-
                       chamber temperature in K,
                       relative humidity in %,
                       fold-direction class
                       — inwards, outwards, or
                       both)
  measurementResult  : object (number of fold
                       cycles to first crease-
                       formation in 1, optical-
                       degradation level after
                       declared cycle count, peel-
                       strength of the substrate-
                       to-encapsulation interface
                       in N/m per ASTM D1876)
  passOrFail         : enum ("pass" | "fail" |
                       "conditional")
```

## §6 Environmental-Reliability Record

```
environmentalRecord:
  environmentalId    : string (uuidv7)
  displayRef         : string (PHASE-1 §3 record
                       reference)
  testStandard       : enum ("IEC-60068-2-14
                       (thermal-cycling)" |
                       "IEC-60068-2-30 (damp
                       heat)" | "IEC-60068-2-78
                       (steady-state damp heat)"
                       | "IEC-60068-2-64
                       (random-vibration)" |
                       "IEC-60068-2-27 (mechanical
                       shock)" | "user-defined")
  measurementResult  : object (the per-test
                       endpoint — pixel-defect
                       count change, dark-defect
                       area in mm^2, brightness
                       drift in cd/m^2, colour-
                       coordinate drift in
                       Δu'v')
```

## §7 EMC and Safety Record

```
emcSafetyRecord:
  emcRecordId        : string (uuidv7)
  displayRef         : string (PHASE-1 §3 record
                       reference)
  testStandard       : enum ("IEC-61000-3-2 (line
                       harmonics)" |
                       "IEC-61000-3-3 (line
                       voltage variations)" |
                       "IEC-61000-4-2 (ESD)" |
                       "IEC-61000-4-3 (radiated
                       immunity)" |
                       "IEC-62368-1 (audio-video
                       safety)" |
                       "EU-EMC-2014-30-EU" |
                       "EU-LVD-2014-35-EU" |
                       "user-defined")
  measurementResult  : object (per-test pass-or-
                       fail outcome with the
                       quantitative endpoint —
                       voltage in V, current in A,
                       field strength in V/m)
```

## §8 Chain-of-Custody Record

```
custodyRecord:
  custodyId          : string (uuidv7)
  artefactRef        : string (the test record,
                       device record, or
                       declaration identifier)
  custodyEvent       : enum ("display-batch-
                       released" | "optical-test-
                       conducted" |
                       "mechanical-test-
                       conducted" |
                       "environmental-test-
                       conducted" |
                       "emc-test-conducted" |
                       "ce-marking-applied" |
                       "ul-mark-applied" |
                       "shipment" | "warranty-
                       claim" | "recall" |
                       "user-defined")
  eventTimestamp     : string (ISO 8601 date-time)
  performingParty    : string (legal entity)
  hashOfArtefacts    : string (SHA-256 hex digest)
```

## §9 Manifest

Implementations publish a signed manifest carrying
`standardSlug` (constant value "flexible-
display"), `version`, `implementation`, the
operator's `accreditationStatus`, and the
`profile` declaration that selects which of the
optional records (optical, mechanical,
environmental, EMC) the implementation supports.
The manifest is signed using a key whose public
part is published on the operator's
`.well-known/wia/flexible-display/` discovery
endpoint declared in PHASE-2.
