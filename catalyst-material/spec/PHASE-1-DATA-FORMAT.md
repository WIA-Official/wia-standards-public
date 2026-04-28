# WIA-catalyst-material PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-catalyst-material
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format layer
for WIA-catalyst-material. The standard covers the
persistent record shapes that a catalyst-research
laboratory, a reference-material producer, an
industrial-catalyst manufacturer, a process-licensor's
qualification team, or a regulatory-conformity body
maintains when registering a catalyst, recording its
physico-chemical characterisation, declaring its
kinetic and selectivity performance, attesting its
status as a certified reference material, tracking its
deactivation across an industrial campaign, and
participating in inter-laboratory comparison rounds.
Records are consumed by the catalyst supplier's
qualification team, by the licensee operating the
catalyst in a production unit, by the certification
body issuing the ISO 17034 reference-material
attestation, by the testing laboratory accredited under
ISO/IEC 17025, by the customs authority enforcing the
REACH and CLP labelling discipline, and — where the
catalyst handles a precious metal or a regulated
substance — by the supervisory authority that audits
the chain-of-custody record.

References (CITATION-POLICY ALLOW only):

- IUPAC Recommendations 2007 — "Manual of Methods and
  Procedures for Catalyst Characterization" (Pure and
  Applied Chemistry, normative for the characterisation
  vocabulary used in this PHASE)
- IUPAC Compendium of Chemical Terminology (the
  "Gold Book") — the normative source for the
  catalysis-specific terms (catalyst, catalysis,
  turnover frequency, active site, selectivity,
  inhibitor, promoter, support) cited in §3 and §5
- IUPAC Quantities, Units, and Symbols in Physical
  Chemistry (the "Green Book") — the normative
  source for SI units, kinetic-parameter symbols, and
  thermodynamic-quantity symbols used in §5 and §6
- IUPAC Recommendations on Catalysis Nomenclature
  (Pure Appl. Chem. 73, 2001, 1227–1241) — the
  normative source for the catalyst-classification
  taxonomy used in §2 (homogeneous, heterogeneous,
  enzymatic, biomimetic, photocatalytic,
  electrocatalytic)
- ISO 17034:2016 (general requirements for the
  competence of reference-material producers)
- ISO/IEC 17025:2017 (general requirements for the
  competence of testing and calibration laboratories)
- ISO Guide 30:2015, ISO Guide 31:2015, ISO Guide 33:
  2015, ISO Guide 35:2017 (reference-material
  vocabulary, certificate content, characterisation
  approaches, and uncertainty estimation)
- ISO 5725-1:2023 / 5725-2:2019 / 5725-3:2023 /
  5725-4:2020 / 5725-6:1994 (accuracy and precision
  of measurement methods and results — repeatability,
  reproducibility, intermediate precision, and
  practical application of accuracy values)
- ISO 9277:2022 (specific surface area determination
  of solids by gas adsorption — the BET method)
- ISO 13322-1:2014 / 13322-2:2021 (particle-size
  analysis — image-analysis methods, static and
  dynamic)
- ISO 15901-1:2016 / 15901-2:2022 / 15901-3:2007
  (mercury porosimetry and gas-adsorption pore-size
  distribution)
- ISO 18757:2003 (fine ceramics — determination of
  specific surface area by BET)
- ASTM D3663-20 (standard test method for surface
  area of catalysts and catalyst carriers)
- ASTM D4222-20 (standard test method for
  determination of nitrogen adsorption and desorption
  isotherms of catalysts)
- ASTM D4567-19 (single-point determination of
  surface area of catalysts and catalyst carriers)
- ASTM D4641-17 (calculation of pore-size distribution
  of catalysts and catalyst carriers from nitrogen
  desorption isotherms)
- ASTM D4824-13 (determination of catalyst acidity by
  ammonia chemisorption)
- IETF RFC 8259 (JSON) and RFC 4122 (UUID)
- ISO 8601 (date and time representation)
- ISO/IEC 27001:2022 (information-security management
  — used for the chain-of-custody record discipline
  in §8)
- EU REACH Regulation (EC) No 1907/2006 (chemical-
  substance registration; cited where the catalyst
  contains a registered substance) and EU CLP
  Regulation (EC) No 1272/2008 (GHS classification
  and labelling)

---

## §1 Scope

This PHASE defines persistent shapes for the artefacts
exchanged when a catalyst is registered, characterised,
qualified, and tracked across an industrial-campaign
boundary. Implementations covered include:

- Single-laboratory catalyst-development teams
  exchanging characterisation results with a
  manufacturing scale-up partner.
- Multi-site catalyst manufacturers operating a
  qualification batch register that links the
  laboratory record to the production-batch record.
- Reference-material producers operating under ISO
  17034 issuing a certified reference material (CRM)
  for catalyst characterisation cross-comparison.
- Process licensors operating a catalyst-performance
  qualification register linked to the licensee's
  production-unit telemetry.
- Inter-laboratory comparison organisers running
  proficiency-testing rounds against an ISO 5725
  precision-protocol design.

The reference-material certificate, the laboratory
test report, and the manufacturer's qualification
record receive identical encoding in this PHASE; the
additional safeguards required by the ISO 17034 RM
producer audit, by the ISO/IEC 17025 laboratory
accreditation audit, and by the REACH / CLP
substance-registration audit are encoded in PHASE-3
§5.

## §2 Programme Identifier

```
programmeId          : string (uuidv7)
operatorName         : string (legal name of the
                       operator — research laboratory,
                       reference-material producer,
                       manufacturer, process licensor,
                       or proficiency-testing organiser)
operatorRole         : enum ("research-laboratory" |
                       "reference-material-producer" |
                       "manufacturer-qualification" |
                       "process-licensor" |
                       "proficiency-testing-organiser" |
                       "accredited-testing-laboratory" |
                       "user-defined")
accreditationStatus  : object (ISO 17034 RM-producer
                       accreditation reference and / or
                       ISO/IEC 17025 testing-laboratory
                       accreditation reference; each
                       carrying the issuing accreditation
                       body, the scope of the
                       accreditation, and the certificate
                       number)
governingFrameworks  : array of enum ("IUPAC-CATAL-
                       NOMENCLATURE-2001" |
                       "IUPAC-CHARACTERIZATION-2007" |
                       "ISO-17034" | "ISO-17025" |
                       "ISO-5725" | "ISO-9277" |
                       "ISO-13322" | "ASTM-D3663" |
                       "ASTM-D4222" | "ASTM-D4824" |
                       "EU-REACH" | "EU-CLP" |
                       "user-defined")
programmeStatus      : enum ("design" | "operating" |
                       "limited-rollout" | "wind-down"
                       | "archived")
```

The combination of `accreditationStatus` and
`governingFrameworks` records the operator's
qualification baseline. Implementations that publish
a reference-material certificate declare both the ISO
17034 accreditation reference and the ISO 5725
characterisation-design reference so that PHASE-3 §3
(certificate-issuance discipline) can derive the
applicable assigned-value-and-uncertainty mechanism.

## §3 Catalyst Material Record

The catalyst material record aligns with the IUPAC
2007 characterisation-manual taxonomy:

```
catalystRecord:
  catalystId         : string (uuidv7; the operator's
                       internal catalyst identifier)
  identifierBindings : array of object (per-
                       jurisdiction substance
                       identifiers — for example the
                       EU REACH registration number,
                       the CAS Registry Number, the
                       EC inventory number, the IUPAC
                       systematic name — each carrying
                       the issuing authority and the
                       scope of use)
  catalystClass      : enum ("homogeneous" |
                       "heterogeneous" | "enzymatic" |
                       "biomimetic" | "photocatalytic" |
                       "electrocatalytic" | "user-
                       defined")
  composition        : object (active-component
                       composition expressed as mass-
                       fraction or mole-fraction with
                       the IUPAC stoichiometric formula
                       for each component, the support
                       material identification with the
                       IUPAC inorganic-nomenclature
                       string, and the promoter set
                       with per-promoter mass-fraction)
  preparationMethod  : enum ("incipient-wetness-
                       impregnation" |
                       "co-precipitation" |
                       "sol-gel" | "deposition-
                       precipitation" | "hydrothermal" |
                       "flame-spray-pyrolysis" |
                       "atomic-layer-deposition" |
                       "user-defined")
  pretreatment       : object (calcination temperature,
                       calcination atmosphere, reduction
                       temperature, reduction
                       atmosphere — each value
                       qualified by the IUPAC Green-
                       Book quantity-and-unit symbol)
  hazardLabelling    : object (CLP Regulation hazard-
                       class assignments and signal-
                       word — recorded where any
                       component triggers a CLP
                       classification)
```

## §4 Characterisation Record

The characterisation record carries the per-technique
measurement set defined by the IUPAC 2007 manual:

```
characterisationRecord:
  characterisationId : string (uuidv7)
  catalystRef        : string (PHASE-1 §3 record
                       reference)
  technique          : enum ("BET-N2-77K" | "BET-Ar-
                       87K" | "Hg-porosimetry" |
                       "XRD-powder" | "XRD-Rietveld" |
                       "TEM-bright-field" | "HAADF-
                       STEM" | "SEM-secondary-
                       electron" | "EDS-mapping" |
                       "XPS-survey" | "XPS-high-
                       resolution" | "TPR-H2" |
                       "TPD-NH3" | "TPD-CO2" |
                       "TPO-O2" | "DRIFTS-pyridine" |
                       "DRIFTS-CO" | "FT-IR-CO" |
                       "in-situ-XAS-XANES" | "in-
                       situ-XAS-EXAFS" | "user-
                       defined")
  instrument         : object (manufacturer + model +
                       configuration; the Trigno-style
                       manufacturer-model identification
                       per CITATION-POLICY §2.1
                       "tools and products of public
                       sale")
  testStandard       : enum ("ISO-9277" | "ISO-15901-
                       1" | "ISO-15901-2" | "ISO-
                       15901-3" | "ASTM-D3663" |
                       "ASTM-D4222" | "ASTM-D4567" |
                       "ASTM-D4641" | "ASTM-D4824" |
                       "user-defined")
  rawData            : string (URI of the raw-data
                       file — the Sigstore-anchored
                       attestation of the raw data
                       hash is in PHASE-3 §6)
  derivedMetrics     : object (per-technique derived
                       quantities — BET specific
                       surface area in m^2 / g, BJH
                       pore-size-distribution mean and
                       standard deviation in nm, XRD
                       crystallite size by Scherrer
                       in nm, TEM particle-size mean
                       and standard deviation in nm,
                       NH3-TPD acid-site density in
                       µmol / g, H2-TPR consumption
                       integrated peak in µmol / g)
  uncertaintyBudget  : object (the ISO 5725
                       repeatability and reproducibility
                       components, expressed at the
                       ISO Guide 35 confidence level
                       declared in §6)
```

## §5 Performance Record

The performance record carries the kinetic-and-
selectivity data set defined by the IUPAC Green-Book
symbols:

```
performanceRecord:
  performanceId      : string (uuidv7)
  catalystRef        : string (PHASE-1 §3 record
                       reference)
  reactionDescriptor : object (IUPAC reaction-name +
                       balanced equation + reactant
                       and product CAS Registry
                       Numbers)
  testRig            : enum ("fixed-bed-tubular" |
                       "CSTR-stirred-tank" |
                       "rotating-basket" | "fluidised-
                       bed" | "trickle-bed" | "user-
                       defined")
  operatingPoint     : object (temperature in K,
                       pressure in Pa, mass-hourly
                       space velocity in 1 / h or gas
                       hourly space velocity in
                       1 / h, contact time in s, feed
                       composition in mole-fraction
                       per IUPAC Green-Book §2)
  performanceMetrics : object (conversion in mole-
                       fraction, selectivity in mole-
                       fraction per product, yield in
                       mole-fraction, turnover
                       frequency in 1 / s per IUPAC
                       Gold-Book "turnover frequency"
                       definition, apparent activation
                       energy in kJ / mol from
                       Arrhenius regression, time-on-
                       stream stability in h)
  deactivationCurve  : array of object (per-time-on-
                       stream conversion-and-
                       selectivity sample with the
                       sample timestamp in ISO 8601)
```

## §6 Reference-Material Certificate

The reference-material certificate aligns with the
ISO 17034 / ISO Guide 31 mandatory-content list:

```
crmCertificate:
  certificateId      : string (uuidv7)
  catalystRef        : string (PHASE-1 §3 record
                       reference)
  rmType             : enum ("certified-reference-
                       material" | "reference-
                       material" | "in-house-
                       reference" | "user-defined")
  assignedValues     : array of object (per-property
                       assigned value with the
                       expanded uncertainty at k=2
                       per ISO Guide 35 and the
                       coverage factor declaration)
  characterisationDesign : enum ("ISO-Guide-35-§5-
                       single-laboratory" |
                       "ISO-Guide-35-§6-inter-
                       laboratory" | "ISO-Guide-35-
                       §7-batch-certification" |
                       "user-defined")
  homogeneityStudy   : object (ISO Guide 35 §8
                       between-bottle and within-
                       bottle component estimates)
  stabilityStudy     : object (ISO Guide 35 §9 short-
                       term and long-term stability
                       components, each with the
                       monitoring interval and the
                       certificate's expiry date)
  intendedUse        : string (the ISO Guide 31 §3
                       intended-use declaration that
                       binds the certificate to the
                       characterisation technique
                       declared in §4)
```

## §7 Inter-Laboratory Comparison Record

```
ilcRecord:
  comparisonId       : string (uuidv7)
  comparisonScheme   : enum ("ISO-5725-design" |
                       "ISO-13528-PT" | "user-
                       defined")
  participantSet     : array of object (each
                       participant carrying the
                       accreditation reference and
                       the per-participant blinded
                       result)
  assignedReference  : string (PHASE-1 §6 certificate
                       reference, where the round
                       uses a CRM as reference;
                       otherwise the consensus
                       statistic per ISO 13528 §7)
  precisionEstimates : object (ISO 5725-2 repeatability
                       and reproducibility standard
                       deviations with the per-
                       laboratory contribution)
  outlierTreatment   : enum ("ISO-5725-2-§7-Cochran-
                       Grubbs" | "ISO-13528-§9-
                       robust-Z" | "user-defined")
```

## §8 Chain-of-Custody Record

```
custodyRecord:
  custodyId          : string (uuidv7)
  catalystRef        : string (PHASE-1 §3 record
                       reference)
  custodyEvent       : enum ("synthesis" |
                       "calcination" | "reduction-
                       activation" | "loading-into-
                       reactor" | "discharge-from-
                       reactor" | "regeneration" |
                       "disposal" | "user-defined")
  eventTimestamp     : string (ISO 8601 date-time)
  performingParty    : string (the legal entity that
                       executed the event)
  observerParty      : string (the auditor, customs
                       authority, or accreditation-
                       body witness present at the
                       event, where applicable)
  hashOfArtefacts    : string (SHA-256 hex digest of
                       the bundled raw-data and
                       certificate artefacts at the
                       moment of the event;
                       PHASE-3 §6 anchors this digest
                       to a Sigstore Rekor entry)
```

## §9 Manifest

Implementations publish a signed manifest containing
`standardSlug` (constant value "catalyst-material"),
`version` (Semantic Versioning 2.0.0),
`implementation` (legal name + build digest + SBOM
URL), `accreditationStatus` (the §2 record), and the
`profile` declaration that selects which of the
optional records (CRM certificate, ILC record) the
implementation supports. The manifest is signed using
a key whose public part is published on the
operator's `.well-known/wia/catalyst-material/`
discovery endpoint declared in PHASE-2.
