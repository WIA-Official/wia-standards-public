# WIA-catalyst-material PHASE 3 — PROTOCOL Specification

**Standard:** WIA-catalyst-material
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern a
catalyst-research operator across the laboratory-to-
manufacturer-to-licensee value chain: the catalyst-
classification discipline that anchors every record
to the IUPAC catalysis-nomenclature taxonomy, the
characterisation discipline that ties each measurement
to its declared test method, the certificate-issuance
discipline that gates the publication of a certified
reference material, the inter-laboratory comparison
discipline that drives the ISO 5725 precision
estimate, the substance-classification discipline
that connects the registration record to the EU
REACH and CLP hazard-class outcome, the chain-of-
custody anchoring discipline that prevents silent
mutation of the laboratory record, and the
deactivation-tracking discipline that aligns an
industrial-campaign performance trace with the
catalyst-development laboratory's hypothesis.

References (CITATION-POLICY ALLOW only):

- ISO 9001:2015 (quality management systems)
- ISO 17034:2016 (reference-material producer
  competence) and ISO Guide 31:2015 / ISO Guide 33:
  2015 / ISO Guide 35:2017 (reference-material
  certificate content, characterisation approach,
  uncertainty estimation)
- ISO/IEC 17025:2017 (testing and calibration
  laboratory competence)
- ISO/IEC 17000:2020 (conformity-assessment
  vocabulary) and ISO/IEC 17021-1:2015 (audit and
  certification of management systems)
- ISO 5725-1:2023 / 5725-2:2019 / 5725-3:2023 /
  5725-4:2020 / 5725-6:1994 (accuracy and precision
  of measurement methods and results)
- ISO 13528:2022 (statistical methods for
  proficiency testing by inter-laboratory
  comparison)
- ISO 9277:2022, ISO 13322-1:2014 / 13322-2:2021,
  ISO 15901-1:2016 / 15901-2:2022 / 15901-3:2007
  (the test-method standards cited in §2)
- ASTM D3663-20, ASTM D4222-20, ASTM D4567-19,
  ASTM D4641-17, ASTM D4824-13 (the test-method
  standards cited in §2)
- IUPAC Recommendations 2007 — Manual of Methods
  and Procedures for Catalyst Characterization
- IUPAC Quantities, Units and Symbols in Physical
  Chemistry (the "Green Book") and IUPAC Compendium
  of Chemical Terminology (the "Gold Book")
- IUPAC Recommendations on Catalysis Nomenclature
  (Pure Appl. Chem. 73, 2001, 1227–1241)
- IETF RFC 9110 (HTTP Semantics), RFC 9421 (HTTP
  Message Signatures), RFC 9457 (Problem Details),
  RFC 8259 (JSON), RFC 8615 (well-known URIs),
  RFC 6962 (Certificate Transparency — used as the
  template for the per-event transparency log)
- W3C Trace Context
- EU REACH Regulation (EC) No 1907/2006 (Articles
  10, 14, 31 — registration dossier, CSR, and
  Safety Data Sheet supply-chain communication)
- EU CLP Regulation (EC) No 1272/2008 (Articles
  4–18 — substance and mixture classification)
- EU Construction Products Regulation (EU)
  305/2011 (cited where the catalyst is sold as a
  construction-product additive)

---

## §1 Catalyst-Classification Discipline

Every record carried by the operator's API is
anchored to the IUPAC 2001 catalysis-nomenclature
taxonomy declared in PHASE-1 §3 `catalystClass`. The
operator's registry rejects a registration where the
declared `catalystClass` is inconsistent with the
declared `composition` — for example, a record whose
`catalystClass` is `enzymatic` but whose
`composition.activeComponent` is an inorganic
formula returns `422 Unprocessable Entity` with a
problem document carrying
`type: /problems/iupac-class-composition-mismatch`.

The classification rule is enforced before any
characterisation record is accepted: a
characterisation pointing at a catalyst whose class
is `homogeneous` rejects the `BET-N2-77K` technique
(BET surface area is defined by IUPAC Gold Book for
solids only) and returns `422 Unprocessable Entity`.

## §2 Characterisation Discipline

### §2.1 Test-method binding

Every characterisation record carries the
`testStandard` enumeration declared in PHASE-2 §4.1.
The operator's API enforces the technique-to-test-
method mapping table:

- `BET-N2-77K` and `BET-Ar-87K` MUST cite
  ISO 9277:2022, ASTM D3663-20, ASTM D4222-20,
  ASTM D4567-19, or ASTM D4641-17.
- `Hg-porosimetry` MUST cite ISO 15901-1:2016.
- Gas-adsorption pore-size distribution MUST cite
  ISO 15901-2:2022 (mesopores) or ISO 15901-3:2007
  (micropores).
- `TPD-NH3` for acidity quantification MUST cite
  ASTM D4824-13.
- Particle-size analysis by image method MUST cite
  ISO 13322-1:2014 (static) or ISO 13322-2:2021
  (dynamic).

### §2.2 Uncertainty discipline

Every derived metric carries an uncertainty budget
expressed at the ISO Guide 35 confidence level. The
operator's API rejects a `derivedMetrics` envelope
whose declared expanded uncertainty does not equal
the coverage factor multiplied by the declared
combined standard uncertainty (the ISO Guide 35 §6
formula). The rejection carries a problem document
with the offending JSON Pointer and the recomputed
expected value so that the originating laboratory
can correct the upload without re-running the
measurement.

### §2.3 Raw-data anchoring

Every characterisation upload carries a raw-data
file. The server stores the raw data under a
content-addressable URI (the SHA-256 hex digest of
the raw bytes is the path segment); the
characterisation record references the digest, and
PHASE-2 §4.2 retrieval returns the digest in the
response envelope so that a downstream consumer can
verify the raw bytes against the digest at fetch
time.

## §3 Certificate-Issuance Discipline

### §3.1 ISO 17034 accreditation gate

A CRM certificate is publishable only by an operator
whose programme record carries an active ISO 17034
RM-producer accreditation reference. The operator's
API verifies the accreditation reference against the
issuing accreditation-body's published register on
each publication request and refuses publication
where the reference is suspended, withdrawn, or
expired.

### §3.2 Characterisation design selection

The certificate's `characterisationDesign`
enumeration declared in PHASE-1 §6 selects the
ISO Guide 35 chapter that governs the assignment
algorithm:

- `ISO-Guide-35-§5-single-laboratory` — the
  reference-material producer characterises the
  candidate material in a single accredited
  laboratory; the assigned value is the laboratory's
  measurement and the uncertainty budget combines
  measurement-method bias, repeatability, and the
  homogeneity / stability components.
- `ISO-Guide-35-§6-inter-laboratory` — the candidate
  material is characterised by a panel of
  accredited laboratories; the assigned value is
  the panel mean (after the ISO 13528 §9 robust
  statistic is applied) and the uncertainty budget
  combines the panel reproducibility with the
  homogeneity / stability components.
- `ISO-Guide-35-§7-batch-certification` — the
  reference-material producer certifies a batch of
  candidate material against an existing higher-
  order CRM; the assigned value is the higher-order
  CRM's value adjusted for the measured difference
  and the uncertainty budget combines the higher-
  order CRM's expanded uncertainty with the
  difference's measurement uncertainty.

### §3.3 Intended-use binding

The `intendedUse` declaration in PHASE-1 §6 binds
the certificate to a characterisation technique
declared in §2.1. A mismatch between the certificate
and a downstream use returns `422 Unprocessable
Entity` with a problem document at
`/problems/crm-intended-use-mismatch`.

## §4 Inter-Laboratory Comparison Discipline

### §4.1 Comparison-scheme enforcement

The operator's API enforces the comparison-scheme
enumeration declared in PHASE-1 §7. An
`ISO-5725-design` round runs the ISO 5725-2 §7
Cochran outlier-detection step before the
repeatability and reproducibility standard
deviations are computed. An `ISO-13528-PT` round
runs the ISO 13528 §9 robust-Z step in place of
classical outlier detection, and the per-laboratory
z-score is computed using the ISO 13528 §7 assigned
value and standard deviation for proficiency
assessment.

### §4.2 Participant-result authentication

Every participant submission carries an HTTP Message
Signature (RFC 9421) issued under the participant's
ISO/IEC 17025 testing-laboratory accreditation
certificate. The server verifies the signature, the
accreditation reference's currency in the issuing
accreditation-body's register, and the scope of
the accreditation against the comparison's declared
test method. A submission whose accreditation scope
does not cover the comparison's test method returns
`403 Forbidden`.

## §5 Substance-Classification Discipline

### §5.1 REACH registration linking

A catalyst whose `composition.activeComponent`
matches a REACH-registered substance MUST carry the
REACH registration number in
`identifierBindings.reachRegistrationNumber`. The
operator's API enforces the registration-number
format defined by Annex VI of EU REACH Regulation
(EC) No 1907/2006 and rejects a registration where
the format check fails.

### §5.2 CLP hazard-class declaration

The `hazardLabelling` field declared in PHASE-1 §3
carries the CLP Regulation hazard-class assignments
and the signal word. The operator's API rejects a
combination that is internally inconsistent — for
example, a `Carcinogenicity Cat 1A` classification
without the `H350` hazard statement, or a `Pyrophoric
solid Cat 1` classification without the `H250`
hazard statement — returning `422 Unprocessable
Entity` with a problem document that names the
expected hazard statement per the CLP Annex I
classification rule.

## §6 Chain-of-Custody Anchoring Discipline

### §6.1 Per-event transparency log

Every chain-of-custody event carried by PHASE-1 §8
is appended to a per-operator transparency log
modelled on the IETF RFC 6962 Certificate
Transparency append-only-log structure. The log
publishes a signed tree-head every signed-tree-head
period (default 24 h, configurable per programme);
the signed tree-head is anchored to the operator's
public-key set declared in
`/.well-known/wia/catalyst-material/keys.json`.

### §6.2 Mutation prevention

A custody event cannot be retroactively edited; an
amendment is recorded as a new event with the
`custodyEvent` value set to the same enumeration and
a `previousEventRef` pointing at the event being
amended. The amending event's narrative carries the
reason (laboratory-recorded transcription error,
witness re-attestation, customs-authority correction)
and is reviewed by the operator's quality manager
under the ISO 9001:2015 §10.2 nonconformity-and-
corrective-action discipline.

## §7 Deactivation-Tracking Discipline

### §7.1 Time-on-stream sampling cadence

The performance record's `deactivationCurve`
declared in PHASE-1 §5 is sampled at a cadence that
the catalyst-development laboratory declares at
qualification time and the licensee operating the
catalyst in a production unit honours during the
industrial campaign.

### §7.2 Mechanism inference handoff

Where the deactivation-curve slope exceeds the
qualification envelope, the operator's API marks
the performance record `deactivationStatus` as
`exceeds-envelope`. A downstream root-cause-analysis
hand-off is initiated against the catalyst-
development laboratory, and the hand-off envelope
carries the suspected mechanism class (sintering,
coke deposition, leaching, sulphur poisoning,
phosphorus poisoning, alkali poisoning, structural
collapse) under a controlled vocabulary mapped
back to the IUPAC 2007 characterisation manual's
deactivation-mechanism nomenclature.

## §8 Quality-Management Discipline

The operator runs an ISO 9001:2015 quality
management system covering the catalyst-registration,
characterisation-upload, certificate-issuance,
inter-laboratory-comparison, and chain-of-custody
processes. Internal audits run on a frequency
declared in the programme's quality manual; the
nonconformity register is reviewed in the ISO 9001
§9.3 management-review cycle. ISO 17034 RM-producer
operators add the ISO Guide 31 §3 certificate-
issuance review and the ISO Guide 35 §10 stability-
study renewal cycle on top of the ISO 9001
discipline.
