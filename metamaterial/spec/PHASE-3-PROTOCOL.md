# WIA-metamaterial PHASE 3 — PROTOCOL Specification

**Standard:** WIA-metamaterial
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern the operating procedure of
an accredited metamaterial design and characterisation programme:
laboratory accreditation, simulation convergence and reproducibility,
fabrication-vendor handover, measurement laboratory protocols,
inter-laboratory round-robin reproducibility, and the publication and
deprecation of metamaterial designs. The PROTOCOL layer binds the data
shapes of PHASE-1 and the API contract of PHASE-2 to the reproducibility
expectations under which metamaterial results are externally citable.

References (CITATION-POLICY ALLOW only):

- ISO/IEC 17025:2017 (testing and calibration laboratories)
- ISO/IEC 17043:2010 (proficiency testing — general requirements)
- ISO/IEC 27001:2022 (information security management)
- ISO 9001:2015 (quality management systems)
- ISO 5725 series (accuracy of measurement methods and results)
- ISO 8601 (date and time)
- ISO 10110 (optical drawings)
- IETF RFC 9457 (Problem Details for HTTP APIs)
- IETF RFC 5905 (NTPv4)
- IETF RFC 8446 (TLS 1.3)
- BIPM JCGM 100 (Guide to the Expression of Uncertainty in Measurement;
  cited only normatively for terminology)

---

## §1 Programme Accreditation

A metamaterial design and characterisation programme MAY claim conformance
to WIA-metamaterial only after a recognised accreditation body has issued
a valid certificate against ISO/IEC 17025:2017 for the scopes the
programme exercises (electromagnetic, acoustic, elastic, or thermal
characterisation). Each scope is a distinct accreditation line item; a
programme that contracts out optical-frequency characterisation to a
photometry laboratory MUST ensure that the laboratory's accreditation
covers the relevant frequency band.

The accreditation register is exposed to the API as a read-only resource
and MUST be re-fetched at least once per calendar quarter. A revoked or
expired accreditation deprecates all of the programme's published designs
in the same quarter; designs already cited externally remain addressable
but are flagged `deprecated` in the design status (PHASE-1 §2).

## §2 Simulation Reproducibility

A simulation record is reproducible when an independent solver of the
same family, given the same unit-cell geometry, the same materials
database identifier and version, and the same meshing and convergence
parameters, produces S-parameter spectra within the published
reproducibility tolerance.

The reproducibility tolerance is recorded against the simulation as part
of its convergence record. Programmes SHOULD report convergence both as a
relative change in S-parameter magnitude across the last two refinement
levels and as a residual on stored energy, so that consumers can
distinguish convergence in the scattering response from convergence in
the field distribution.

A simulation that has not converged within the recorded `maxIterations`
is recorded as `failed`; the programme MAY re-run the simulation with a
revised mesh budget but MUST emit a new simulation record rather than
overwriting the failed one.

## §3 Materials Database Versioning

Constitutive databases evolve as new measurements become available. A
programme MUST pin the database identifier and version in every
simulation, retrieval, and measurement record so that the same record can
be re-evaluated against a later database version without ambiguity.

When a database update changes a material model that an externally cited
design depends on, the programme republishes a derivative simulation
referencing the new database version. The original simulation remains
addressable; consumers MAY pin either the original or the derivative
simulation, depending on whether they are reproducing or extending the
result.

## §4 Fabrication-Vendor Handover

Designs that are realised in hardware are handed to a fabrication vendor
under a written process agreement that includes the design's tolerance
budget (PHASE-1 §8), the realisation tolerances the vendor expects to
achieve, and the metrology that will be used to verify the realised
geometry before electromagnetic characterisation. The handover record
references the design ID, the fabrication vendor's identifier, and the
content-address of the agreement.

Vendors deliver a certificate of fabrication that lists the realised
linewidth bias, sidewall angle, layer-thickness variation, alignment
error, and surface roughness as measured by the agreed metrology. The
certificate is appended to the design record under
`/v1/designs/{designId}/fabrication-certificates`.

## §5 Measurement Laboratory Protocol

Reference laboratories operate under ISO/IEC 17025:2017 procedures with
specific extensions for metamaterial characterisation:

- **Calibration cadence**: vector network analysers and equivalent
  instruments are calibrated against traceable standards at the start of
  each measurement campaign and at fixed intervals during the campaign.
- **Sample handling**: samples are stored under controlled humidity and
  temperature; environmental excursions are recorded against the
  measurement record in the uncertainty contributions.
- **Reference standards**: every measurement campaign includes at least
  one reference structure whose response is known so that drift can be
  detected and quantified across the campaign.
- **Uncertainty reporting**: measurement records carry uncertainty
  contributions in the type-A / type-B taxonomy of JCGM 100, with
  expanded uncertainty reported at the coverage factor agreed with the
  consumer.

## §6 Inter-Laboratory Round-Robins

Programmes that publish externally cited designs participate in
inter-laboratory round-robin exercises at least once every two calendar
years. The round-robin protocol follows ISO/IEC 17043:2010 and ISO 5725
expectations for reproducibility studies. The round-robin coordinator is
the certifying body or a body it nominates.

Round-robin reports are appended to participating programmes' records and
are referenced from the programme's well-known discovery document. A
programme that does not participate in the most recent round-robin MAY
still publish designs but MUST flag the designs as
`single-laboratory-characterised` in the public catalogue.

## §7 Records Retention

Programme records — every record defined in PHASE-1, the API audit logs
defined in PHASE-2 §12, simulation artefacts, raw measurement data, and
the agreements and certificates defined in this PHASE — are retained for
a minimum of seven calendar years from the last access of the design.
Externally cited designs are retained indefinitely as long as the
programme is operating; on programme wind-down, indefinite-retention
records are transferred to a recognised long-term archive.

## §8 Time Synchronisation

Programmes operate on synchronised time per RFC 5905 (NTPv4) so that
simulation, fabrication, and measurement events can be ordered
unambiguously across instruments and across sites. The reference time
source is the national metrological laboratory's stratum-1 service or an
equivalent recognised service; the operator records the source and the
maximum observed offset in the programme's quality dossier.

## §9 Programme Wind-Down

A programme that ceases operations transitions all draft designs to
`deprecated`, transfers indefinite-retention records to a recognised
long-term archive, and notifies known external citers via the well-known
discovery document. Externally pinned evidence packages remain accessible
through the archive at the same content-addressed URLs.

## §10 Welfare and Ethics

Computational and laboratory metamaterial work has no direct
animal-welfare exposure. Where designs are intended for medical use
(implantable acoustic lenses, tissue-mimicking phantoms), the programme
follows the welfare framework of the medical-use programme and references
that framework rather than re-inventing one here.

## §11 Quality Dossier and Welfare-Equivalent Framework

The programme's quality dossier is a written document maintained at a
publicly resolvable URL. The dossier records the programme's accreditation
scopes, the material database revisions it tracks, the fabrication
vendors with which it has standing agreements, the measurement laboratory
or laboratories that it works with, the round-robin exercises it has
participated in, and the deprecation history of its published designs.
The dossier is reviewed at least annually by the programme's quality
manager and is read during the annual ISO/IEC 17025 surveillance audit.

## §12 Cross-Border Programme Operation

A programme that operates across borders maintains a primary jurisdiction
of registration and one or more operating jurisdictions where simulation,
fabrication, or measurement activity occurs. The primary jurisdiction's
accreditation body is the lead authority; operating jurisdictions consume
the lead authority's certifications under mutual recognition where one
exists.

Cross-border data transfers honour the data-protection law of the
contributing institution, which is recorded in the operator's collaboration
agreements at the start of each project. The published records carry only
institutional identifiers; researcher PII is held in collaboration-internal
systems and is exposed externally only as authorship metadata at the time
of publication.

## §13 Solver Validation Studies

A programme that publishes externally cited designs SHOULD periodically
participate in or contribute to solver validation studies that compare
solver families on canonical test cases (a homogeneous half-space
reflection, a unit-cell of known closed-form effective parameters, a
finite-array edge effect). Validation studies are coordinated by the
certifying body or by community working groups; their outcome is recorded
in the programme's quality dossier.

The purpose of solver validation is to bound the systematic difference
between solver families so that a result obtained on solver A can be
compared against a result obtained on solver B without conflating
solver-family bias with design-level bias. Validation studies do not
replace inter-laboratory round-robins, which exercise measurement, not
simulation.

## §14 Reproducibility Receipts

Every externally cited result carries a reproducibility receipt: the
content-addresses of the unit-cell, the simulation record, the materials
database revision, the retrieval method, the fabrication certificate (if
realised), and the measurement record (if measured). The receipt is
emitted as part of the evidence package (PHASE-4 §3) and is the citation
target that downstream readers pin.

Programmes MUST refuse to issue a reproducibility receipt for a result
that is incomplete in the design, simulation, retrieval, fabrication, or
measurement records that the result claims to depend on. Incomplete
receipts return a Problem-Details (RFC 9457) response of type
`urn:wia:metamaterial:incomplete-receipt`.

## §15 Citation Hygiene

A programme is responsible for the integrity of citations to its
designs. When an external publication cites a design that the programme
has subsequently revised or deprecated, the programme MUST keep the
originally cited evidence package addressable at its content-addressed
URL and MUST publish a successor relationship in the discovery document
so that downstream readers can navigate from the cited design to its
current revision.

Programmes SHOULD reach out to known external citers when a deprecation
occurs, but the obligation to maintain addressability does not depend on
successful outreach; the content-addressed URL is the durable contract.

## §16 Conformance and Auditing

A programme conformant with WIA-metamaterial publishes its accreditation
certificate, its programme code registration, its quality dossier, and the
catalogue of designs it has published, and answers an annual
self-assessment that maps each clause of this PHASE to the programme's
implementation. The self-assessment is reviewed during the annual
ISO/IEC 17025 surveillance audit.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-metamaterial
- **Last Updated:** 2026-04-27
