# WIA-eco-friendly-material PHASE 3 — PROTOCOL Specification

**Standard:** WIA-eco-friendly-material
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern an
eco-friendly-material operator across the producer-
to-programme-to-verifier-to-buyer value chain: the
ISO 14040 / ISO 14044 LCA goal-and-scope discipline,
the ISO 14025 §7 EPD programme operator's
instructions discipline, the ISO 14021 §7 Type II
self-declared claim discipline that gates the
publication of an "eco-friendly" claim, the ISO
14024 Type I scheme operator's licensing
discipline, the ISO 14067 product carbon footprint
quantification discipline, the ISO 14064-1 / -2 /
-3 GHG reporting and verification discipline, the
EN 15804+A2 / EN 15978 construction-product
discipline that aligns the EPD with the building-
level assessment, the EU CBAM declaration
discipline that links the embedded emissions to
the customs declaration, the chain-of-custody
anchoring discipline that prevents silent mutation
of the published artefact, and the data-quality
discipline that enforces the LCI dataset
provenance.

References (CITATION-POLICY ALLOW only):

- ISO 14040:2006/Amd 1:2020 (LCA principles and
  framework) and ISO 14044:2006/Amd 1:2017/Amd 2:
  2020 (LCA requirements and guidelines)
- ISO 14025:2006 (Type III EPD baseline)
- ISO 14021:2016 (Type II self-declared claims)
- ISO 14024:2018 (Type I third-party ecolabels)
- ISO 14067:2018 (carbon footprint of products)
- ISO 14064-1:2018, ISO 14064-2:2019, ISO 14064-3:
  2019 (GHG quantification, project, validation /
  verification)
- ISO 14065:2020 (GHG verification body
  competence)
- EN 15804:2012+A2:2019, EN 15978:2011, ISO
  21930:2017 (construction-product EPD core rules
  and building-level assessment)
- ISO 9001:2015 (quality management systems)
- ISO/IEC 17000:2020 (conformity-assessment
  vocabulary), ISO/IEC 17021-1:2015 (audit of
  management systems), ISO/IEC 17065:2012 (product
  conformity-assessment bodies)
- IETF RFC 9110, RFC 9421, RFC 9457, RFC 8615, RFC
  6962 (the per-event transparency log structure)
- W3C Trace Context
- EU REACH Regulation (EC) No 1907/2006 Article
  33 (SVHC supply-chain communication)
- EU Construction Products Regulation (EU)
  305/2011 (declared performance and CE marking)
- EU Carbon Border Adjustment Mechanism
  Regulation (EU) 2023/956

---

## §1 LCA Goal-and-Scope Discipline

Every LCA record carried by the operator's API
declares a goal-and-scope envelope per ISO 14040
§5.1. The envelope binds the LCA to a single
`materialRef` (PHASE-1 §3) and to a single
`systemBoundary` (PHASE-1 §4). The operator's API
rejects an LCA upload whose `systemBoundary` does
not match the modules covered by the LCI dataset
referenced in `inventoryDataset`: a `cradle-to-
grave` boundary that does not declare modules A1
through C4 returns `422 Unprocessable Entity` at
`/problems/iso14040-system-boundary-incomplete`.

## §2 EPD Programme Operator Discipline

### §2.1 Programme operator's instructions

An EPD record carried by the operator's API is
publishable only against a programme operator's
published instructions that satisfy ISO 14025 §7.
The instructions declare the panel that approves
PCRs, the verification scheme (internal, external,
third-party individual, third-party panel), the
mandatory content of the EPD, and the duration of
the EPD's validity (default five years per ISO
14025 §7.2.4).

### §2.2 PCR review-and-amendment cycle

A Product Category Rule is reviewed at least every
three years per ISO 14025 §6.7.1. The operator's
API carries the PCR's review status; an EPD
publication against a PCR whose review is overdue
is annotated with a `pcrReviewOverdue: true` flag
in the response envelope so that a downstream
consumer can take the flag into account when
relying on the EPD.

## §3 Verification Discipline

### §3.1 ISO 14025 EPD verification

An EPD whose `verificationType` is `third-party-
individual` or `third-party-panel` is publishable
only when the `verifierReference` carries a current
ISO 14065:2020 accreditation reference. The
operator's API queries the issuing accreditation
body's register on each publication request and
refuses publication where the reference is
suspended, withdrawn, or expired. The register
response is cached for the TTL declared in the
accreditation body's HTTP caching headers (RFC
9111).

### §3.2 ISO 14064-3 GHG verification

A GHG report whose `verificationLevel` is
`limited-iso-14064-3` or `reasonable-iso-14064-3`
is publishable as verified only when the
verifier's ISO 14065 accreditation covers the
report's scope (organisation-level, project-level,
or product-level). A scope mismatch returns `403
Forbidden` at `/problems/iso14064-3-scope-
mismatch`.

### §3.3 ISO 14024 Type I licence discipline

A Type I licence carried by the operator's API
references the scheme's published criteria and the
scheme operator's accreditation under ISO/IEC
17065:2012. The licence is publishable only by an
operator whose programme record carries the
correct ISO/IEC 17065 reference; renewal is
gated on the scheme's review cycle.

## §4 Type II Claim Discipline

### §4.1 Evaluation-method enforcement

Every Type II claim record carries the
`evaluationMethod` enumeration declared in
PHASE-1 §6. The operator's API enforces the
claim-to-evaluation-method mapping table per ISO
14021 §7:

- A `recyclable` claim MUST cite the `ISO-14021-
  §7-recyclable-evaluation` method.
- A `recycled-content` claim MUST cite the `ISO-
  14021-§7-recycled-content-evaluation` method
  and MUST declare the recycled-content
  percentage as either `pre-consumer` or `post-
  consumer` (ISO 14021 §7.8).
- A `compostable` claim MUST cite the `ISO-14021-
  §7-compostable-evaluation` method and MUST
  reference a current EN 13432 or ASTM D6400
  test report in `evidenceArtefacts`.

### §4.2 Marketing-claim verbatim record

The `claimText` field carried in PHASE-1 §6 is the
verbatim claim made in the declarant's marketing
material. The operator's API rejects a claim
whose `claimText` includes a comparative claim
(such as "more recyclable than the previous
version") without the ISO 14021 §5 declared
comparison basis.

## §5 Construction-Product Discipline

### §5.1 EN 15804+A2 module discipline

An EPD whose `materialFamily` is `construction-
binder` or `construction-aggregate` is published
in conformance with EN 15804+A2:2019. The operator's
API enforces the indicator set declared by EN
15804+A2 §6.5.4: an EPD that does not carry the
seven core indicators declared by the standard
(GWP-fossil, GWP-biogenic, GWP-luluc, ozone-
depletion, acidification, eutrophication
freshwater, eutrophication marine) is rejected
with `422 Unprocessable Entity`.

### §5.2 EN 15978 building-level integration

The operator publishes a machine-readable
summary suitable for ingestion into an EN
15978-compliant building-level assessment tool.
The summary is carried in the EPD response
envelope and is signed using the operator's
public-key set so that a downstream tool can
validate the binding without re-fetching the EPD.

## §6 Chain-of-Custody Anchoring Discipline

### §6.1 Per-event transparency log

Every custody event carried by PHASE-1 §8 is
appended to a per-operator transparency log
modelled on the IETF RFC 6962 Certificate
Transparency append-only-log structure. The log
publishes a signed tree-head every signed-tree-
head period (default 24 h, configurable per
programme); the signed tree-head is anchored to
the operator's public-key set declared in
`/.well-known/wia/eco-friendly-material/keys.
json`.

### §6.2 Mutation prevention

A custody event cannot be retroactively edited;
an amendment is recorded as a new event with
`previousEventRef` pointing at the event being
amended. The amending event's narrative carries
the reason and is reviewed by the operator's
quality manager under the ISO 9001:2015 §10.2
nonconformity-and-corrective-action discipline.

## §7 Carbon Border Adjustment Discipline

### §7.1 CBAM declaration linking

An organisation in scope of EU CBAM Regulation
(EU) 2023/956 publishes a per-product embedded-
emissions declaration that links the carbon
footprint computed under ISO 14067:2018 to the
CBAM declaration filed with the customs authority.
The operator's API carries the per-product CBAM
report identifier and the linked carbon footprint
record so that the customs authority can verify
the embedded-emissions value against the carbon-
footprint computation.

### §7.2 Direct, indirect, and precursor emissions

The CBAM declaration distinguishes direct
emissions, indirect emissions (electricity), and
precursor-product emissions. The operator's API
carries the three components separately, and the
sum is reported as the total embedded emissions
in tonnes of CO2-eq per tonne of product.

## §8 Data-Quality Discipline

### §8.1 LCI dataset provenance

Every LCA record carries an LCI dataset under a
content-addressable URI; the dataset's SHA-256
hex digest is recorded in the chain-of-custody
record so that a downstream consumer can verify
that the dataset has not been silently mutated
since the LCA was published.

### §8.2 Pedigree-matrix uncertainty

An LCA whose `uncertaintyAnalysis` is the
pedigree-matrix declaration carries the per-
indicator pedigree score (reliability, completeness,
temporal correlation, geographical correlation,
technological correlation) per ISO 14044 §4.5.2.
The operator's API enforces the pedigree-matrix
score set: a missing dimension returns `422
Unprocessable Entity` at `/problems/iso14044-
pedigree-matrix-incomplete`.

## §9 Quality-Management Discipline

The operator runs an ISO 9001:2015 quality
management system covering the material-
registration, LCA-upload, EPD-publication, Type II
claim-attestation, GHG report-ingestion, and
chain-of-custody processes. Internal audits run
on a frequency declared in the programme's
quality manual; the nonconformity register is
reviewed in the ISO 9001 §9.3 management-review
cycle. The operator's environmental-management
system declared under ISO 14001:2015 covers the
operator's own environmental aspects (the
operator-side eco-design and own-emissions
discipline) and is audited under ISO 19011 by an
independent reviewer.

## §10 Greenwashing-Prevention Discipline

### §10.1 Vague-claim refusal

A Type II self-declared claim whose `claimText`
includes a vague descriptor (such as "eco-
friendly", "green", "sustainable", "natural",
"non-toxic") without a specific evaluation
method declared under ISO 14021 §7 is refused at
publication. The refusal is recorded as a
nonconformity event in the operator's quality
register so that the ISO 9001 §10.2 corrective-
action discipline can be applied.

### §10.2 Comparative-claim discipline

A comparative claim ("more recyclable than the
previous version", "lower carbon footprint than
the industry average") is publishable only when
the comparison basis is declared per ISO 14021
§5.7 and the comparison's reference data set is
itself a published artefact (an industry-average
EPD pool, a previous-version EPD with a
comparable PCR, a sector benchmark issued by an
authoritative body).

### §10.3 Logo-and-label discipline

A Type I ecolabel licensee uses the scheme's
logo only on the products that the licence
covers, and only for the duration of the
licence's validity. The operator's API publishes
the licence's `logoBindingScope` field so that a
consumer can verify the binding before relying on
the label.

## §11 Building-Level EN 15978 Discipline

### §11.1 Reference Service Life

An EPD whose `materialFamily` is a construction-
product family carries a Reference Service Life
declaration consistent with ISO 15686-1:2011 and
ISO 15686-2:2012. The operator's API enforces the
range constraint declared in EN 15804+A2 §6.4 and
rejects an RSL declaration that exceeds the
boundary defined by the PCR.

### §11.2 Use-stage scenario binding

A construction-product EPD whose modules cover
B1–B7 declares the use-stage scenario per EN
15804+A2 §6.5: the operator's API records the
declared scenario (cleaning frequency,
maintenance cycle, repair cycle, replacement
cycle, refurbishment cycle, energy use during
operation, water use during operation) so that
the building-level assessment tool can
parameterise the per-building scenario from the
EPD.
