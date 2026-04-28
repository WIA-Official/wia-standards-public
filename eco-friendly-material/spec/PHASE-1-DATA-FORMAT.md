# WIA-eco-friendly-material PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-eco-friendly-material
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format
layer for WIA-eco-friendly-material. The standard
covers the persistent record shapes that an eco-
friendly-material producer, a third-party
verification body, an environmental-product-
declaration programme operator, a construction-
product manufacturer, a textile or packaging
manufacturer, or a public-procurement authority
maintains when declaring a material as having a
defined environmental performance, registering a
life-cycle assessment (LCA), publishing an
environmental product declaration (EPD), reporting
a product carbon footprint, attesting a self-
declared environmental claim (Type II), or
participating in a Type I ecolabel licensing
scheme. Records are consumed by the public-
procurement buyer evaluating the bid, by the
construction-project designer integrating the EPD
into the building-level assessment, by the consumer
reading the Type II self-declared claim, by the
regulator auditing the supply-chain-due-diligence
dossier, and — where the product is sold across
borders — by the customs authority enforcing the
carbon-border-adjustment-mechanism reporting
discipline.

References (CITATION-POLICY ALLOW only):

- ISO 14040:2006/Amd 1:2020 (life-cycle assessment
  — principles and framework) and ISO 14044:2006/
  Amd 1:2017/Amd 2:2020 (LCA — requirements and
  guidelines)
- ISO 14025:2006 (environmental labels and
  declarations — Type III environmental
  declarations — principles and procedures, the
  EPD baseline)
- ISO 14067:2018 (carbon footprint of products —
  requirements and guidelines for quantification)
- ISO 14021:2016 (environmental labels and
  declarations — self-declared environmental
  claims, the Type II baseline)
- ISO 14024:2018 (environmental labels and
  declarations — Type I, the third-party-verified
  ecolabel baseline)
- ISO 14001:2015 (environmental management
  systems — requirements with guidance for use)
- ISO 14004:2016 (environmental management
  systems — general guidelines on implementation)
- ISO 14006:2020 (environmental management
  systems — guidelines for incorporating eco-
  design)
- ISO 14064-1:2018 (GHG — Part 1: organisation-
  level quantification and reporting), ISO 14064-2:
  2019 (Part 2: project-level quantification,
  monitoring, and reporting), and ISO 14064-3:2019
  (Part 3: validation and verification)
- ISO 14065:2020 (general principles and
  requirements for bodies validating and verifying
  environmental information)
- EN 15804:2012+A2:2019 (sustainability of
  construction works — EPD core rules for the
  product category of construction products) and
  EN 15978:2011 (assessment of environmental
  performance of buildings — calculation method)
- EU Construction Products Regulation (EU)
  305/2011 — declared performance and CE marking
  for construction products
- ISO 21930:2017 (sustainability in buildings and
  civil engineering works — core rules for EPDs of
  construction products and services)
- IETF RFC 8259 (JSON), RFC 4122 (UUID), ISO 8601
  (date-time)
- ISO/IEC 27001:2022 (information-security
  management — used for the chain-of-custody
  record discipline in §8)
- EU REACH Regulation (EC) No 1907/2006 Article 33
  (substances of very high concern communication
  in the supply chain)
- EU Carbon Border Adjustment Mechanism
  Regulation (EU) 2023/956 (cited where the
  product is in scope of the CBAM transitional or
  definitive regime)

---

## §1 Scope

This PHASE defines persistent shapes for the
artefacts exchanged when an eco-friendly material
is registered, characterised through an LCA,
declared in an EPD, attested with a self-declared
environmental claim, or licensed under a Type I
ecolabel. Implementations covered include:

- A single-product material producer publishing a
  Type III EPD against a Product Category Rule
  (PCR) in conformance with ISO 14025 and a
  programme operator's instructions.
- A multi-product manufacturer operating a
  product-portfolio EPD register linked to a
  shared LCA dataset.
- A construction-product manufacturer publishing
  EPDs in conformance with EN 15804+A2:2019 and
  ISO 21930:2017 for use in EN 15978 building-
  level assessments.
- A Type I ecolabel licensee operating under a
  scheme in conformance with ISO 14024:2018.
- A Type II self-declared environmental claimant
  operating under ISO 14021:2016 (recyclable,
  recycled-content, compostable, bio-based-
  content, reusable, refillable, durable,
  designed-for-disassembly, energy-efficient).
- An organisation publishing an organisation-
  level GHG inventory under ISO 14064-1.
- A project developer publishing a project-level
  GHG report under ISO 14064-2.

The Type III EPD, the Type I ecolabel licence, and
the Type II self-declared claim receive distinct
encodings in this PHASE; the additional safeguards
required by each verification regime are encoded
in PHASE-3 §3.

## §2 Programme Identifier

```
programmeId          : string (uuidv7)
operatorName         : string (legal name of the
                       operator — manufacturer,
                       Type I scheme operator,
                       EPD programme operator,
                       verification body, or
                       organisation publishing a
                       GHG inventory)
operatorRole         : enum ("material-producer" |
                       "epd-programme-operator" |
                       "type-i-scheme-operator" |
                       "type-ii-self-declarant" |
                       "verification-body" |
                       "organisation-ghg-reporter" |
                       "project-ghg-reporter" |
                       "user-defined")
governingFrameworks  : array of enum ("ISO-14040" |
                       "ISO-14044" | "ISO-14025" |
                       "ISO-14067" | "ISO-14021" |
                       "ISO-14024" | "ISO-14001" |
                       "ISO-14064-1" |
                       "ISO-14064-2" |
                       "ISO-14064-3" |
                       "ISO-14065" |
                       "EN-15804-A2" |
                       "EN-15978" | "ISO-21930" |
                       "EU-CPR-305-2011" |
                       "EU-CBAM-2023-956" |
                       "user-defined")
verificationStatus   : object (the ISO 14064-3 / ISO
                       14025 / ISO 14024 verification
                       reference for the operator's
                       publications, where applicable;
                       carrying the verifier's legal
                       name, the verifier's ISO 14065
                       accreditation reference, and
                       the certificate number)
programmeStatus      : enum ("design" | "operating" |
                       "limited-rollout" | "wind-
                       down" | "archived")
```

## §3 Material Record

```
materialRecord:
  materialId         : string (uuidv7)
  identifierBindings : array of object (per-
                       jurisdiction substance
                       identifiers — for example
                       the EU REACH registration
                       number, the CAS Registry
                       Number, the UNSPSC code,
                       the customs HS code — each
                       carrying the issuing
                       authority and the scope of
                       use)
  materialFamily     : enum ("metal" | "polymer" |
                       "ceramic" | "composite" |
                       "wood-product" |
                       "textile" | "paper-and-
                       pulp" | "construction-
                       binder" | "construction-
                       aggregate" | "user-defined")
  ecoAttribute       : array of enum ("recycled-
                       content" | "bio-based-
                       content" | "recyclable" |
                       "compostable" |
                       "biodegradable" |
                       "reusable" | "refillable" |
                       "durable" | "designed-for-
                       disassembly" | "energy-
                       efficient" | "reduced-
                       resource-use" | "user-
                       defined")
  declaredUnit       : object (the ISO 14025 §6.6
                       declared-unit specification
                       — quantity, unit symbol per
                       SI Brochure, and the
                       reference flow per EN
                       15804 §6.3.5)
  productCategoryRule : string (the PCR identifier
                       under which the EPD or
                       Type II claim is made; the
                       PCR is published by the
                       programme operator)
```

## §4 Life-Cycle Assessment Record

```
lcaRecord:
  lcaId              : string (uuidv7)
  materialRef        : string (PHASE-1 §3 record
                       reference)
  systemBoundary     : enum ("cradle-to-gate" |
                       "cradle-to-grave" |
                       "cradle-to-cradle" | "gate-
                       to-gate" | "user-defined")
  modulesCovered     : array of enum ("A1" | "A2" |
                       "A3" | "A4" | "A5" | "B1" |
                       "B2" | "B3" | "B4" | "B5" |
                       "B6" | "B7" | "C1" | "C2" |
                       "C3" | "C4" | "D")
  impactCategories   : array of object (per-
                       category indicator —
                       global-warming potential
                       in kg CO2-eq per declared
                       unit, ozone-depletion in
                       kg CFC-11-eq, acidification
                       in mol H+-eq, eutrophication
                       freshwater in kg P-eq,
                       eutrophication marine in
                       kg N-eq, photochemical-
                       ozone formation in kg
                       NMVOC-eq, abiotic-resource-
                       depletion-elements in kg
                       Sb-eq, abiotic-resource-
                       depletion-fossil in MJ,
                       water-use in m^3 world-eq,
                       per ISO 14044 §4.4 and EN
                       15804 §6.5.4)
  inventoryDataset   : string (URI of the LCI
                       dataset bound to the
                       computation; the dataset's
                       SHA-256 hex digest is
                       carried in §8 chain-of-
                       custody record)
  cutoffRules        : object (the ISO 14040 §4.1
                       cut-off rule applied to
                       inputs and outputs below
                       the cut-off threshold)
  allocationRules    : enum ("none" | "physical" |
                       "economic" | "system-
                       expansion" | "user-defined")
  uncertaintyAnalysis : object (Monte Carlo or
                       pedigree-matrix declaration
                       per ISO 14044 §4.5.2)
```

## §5 Environmental Product Declaration Record

```
epdRecord:
  epdId              : string (uuidv7)
  materialRef        : string (PHASE-1 §3 record
                       reference)
  programmeRef       : string (programme operator
                       identifier — the ISO 14025
                       §7 programme operator's
                       published instructions
                       reference)
  pcrIdentifier      : string (the PCR identifier
                       under which the EPD is
                       made)
  validityPeriod     : object (issuanceDate and
                       expiryDate; the ISO 14025
                       §7.2.4 default validity is
                       five years unless the PCR
                       declares a different
                       period)
  verificationType   : enum ("internal" |
                       "external" | "third-party-
                       individual" | "third-party-
                       panel" | "user-defined")
  verifierReference  : object (the verifier's
                       legal name, the verifier's
                       ISO 14065 accreditation
                       reference, and the
                       verification-report
                       identifier)
```

## §6 Type II Self-Declared Claim Record

```
type2Claim:
  claimId            : string (uuidv7)
  materialRef        : string (PHASE-1 §3 record
                       reference)
  claimText          : string (the verbatim claim
                       statement made in the
                       declarant's marketing
                       material, encoded as the
                       ISO 14021 §7 claim template
                       — recyclable, recycled-
                       content with the declared
                       percentage, compostable
                       with the declared standard)
  evaluationMethod   : enum ("ISO-14021-§7-
                       recyclable-evaluation" |
                       "ISO-14021-§7-recycled-
                       content-evaluation" |
                       "ISO-14021-§7-compostable-
                       evaluation" | "ISO-14021-
                       §7-bio-based-content-
                       evaluation" | "user-
                       defined")
  evidenceArtefacts  : array of string (URIs of
                       the evidence artefacts
                       supporting the claim — test
                       reports, supplier
                       declarations, mass-balance
                       certificates)
```

## §7 Organisation- and Project-Level GHG Records

```
ghgRecord:
  ghgId              : string (uuidv7)
  reportLevel        : enum ("organisation-iso-
                       14064-1" | "project-iso-
                       14064-2" | "user-defined")
  reportingPeriod    : object (start and end dates
                       per ISO 8601)
  scope              : object (Scope 1 / Scope 2 /
                       Scope 3 totals per ISO
                       14064-1 §6.4 categories;
                       project-level reports declare
                       the project baseline and
                       project emissions per ISO
                       14064-2 §5.4)
  verificationLevel  : enum ("limited-iso-14064-3" |
                       "reasonable-iso-14064-3" |
                       "not-verified" | "user-
                       defined")
```

## §8 Chain-of-Custody Record

```
custodyRecord:
  custodyId          : string (uuidv7)
  artefactRef        : string (the LCA / EPD /
                       Type II / GHG record
                       identifier the event
                       attests)
  custodyEvent       : enum ("dataset-finalised" |
                       "lca-computed" | "verifier-
                       review-opened" | "verifier-
                       review-closed" | "epd-
                       published" | "claim-
                       published" | "withdrawn" |
                       "user-defined")
  eventTimestamp     : string (ISO 8601 date-time)
  performingParty    : string (legal entity)
  hashOfArtefacts    : string (SHA-256 hex digest)
```

## §9 Manifest

Implementations publish a signed manifest carrying
`standardSlug` (constant value "eco-friendly-
material"), `version`, `implementation`, the
programme record's `verificationStatus`, and the
`profile` declaration that selects which of the
optional records (EPD, Type II claim, GHG report)
the implementation supports. The manifest is signed
using a key whose public part is published on the
operator's `.well-known/wia/eco-friendly-material/`
discovery endpoint declared in PHASE-2.
