# WIA-protein-dynamics PHASE 3 — PROTOCOL Specification

**Standard:** WIA-protein-dynamics
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern the operating procedure
of an accredited protein-dynamics programme: laboratory accreditation,
biospecimen consent and access, simulation reproducibility, force-field
versioning, experimental laboratory protocols, inter-laboratory
round-robin reproducibility, records retention, and the publication and
deprecation of dynamics studies. The PROTOCOL layer binds the data
shapes of PHASE-1 and the API contract of PHASE-2 to the reproducibility
expectations under which protein-dynamics results are externally
citable.

References (CITATION-POLICY ALLOW only):

- ISO/IEC 17025:2017 (testing and calibration laboratories)
- ISO/IEC 17043:2010 (proficiency testing)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 27701:2019 (privacy information management)
- ISO 9001:2015 (quality management systems)
- ISO 5725 series (accuracy of measurement methods)
- ISO 8601 (date and time)
- IETF RFC 9457 (Problem Details)
- IETF RFC 5905 (NTPv4)
- IETF RFC 8446 (TLS 1.3)
- HL7 FHIR R5 (biospecimen and consent resources)
- BIPM JCGM 100 (Guide to the Expression of Uncertainty in Measurement;
  cited only normatively for terminology)

---

## §1 Programme Accreditation

A protein-dynamics programme MAY claim conformance to WIA-protein-dynamics
only after a recognised accreditation body has issued a valid certificate
against ISO/IEC 17025:2017 for the scopes the programme exercises (one or
more of: classical molecular dynamics, enhanced-sampling molecular
dynamics, NMR relaxation spectroscopy, single-molecule FRET, hydrogen-
deuterium exchange mass spectrometry, time-resolved X-ray scattering).
Each scope is a distinct accreditation line item; a programme that
contracts out NMR experiments to an external NMR facility MUST ensure
that the facility's accreditation covers the relevant scope.

The accreditation register is exposed to the API as a read-only resource
and MUST be re-fetched at least once per calendar quarter. A revoked or
expired accreditation deprecates all of the programme's published studies
in the same quarter; studies already cited externally remain addressable
but are flagged `deprecated`.

## §2 Biospecimen Consent and Access

When a preparation derives from a clinical biospecimen (PHASE-1 §3), the
study is accepted only after the biobank's consent chain confirms that
the specific use described in the study's purpose statement is
authorised. Consent verification follows the FHIR R5 `Consent` resource
semantics; the API endpoint defined in PHASE-2 §9 mediates the
verification.

Consent revocation by the source biospecimen donor triggers a study-level
freeze: the study transitions to `withdrawn` status, the API blocks
further analysis registrations against the affected preparation, and the
evidence package is amended with a withdrawal notice. Analyses already
published before the revocation remain addressable but are flagged with
the withdrawal notice; the canonical content-addressed URLs do not
change.

## §3 Simulation Reproducibility and Force-Field Versioning

A simulation record is reproducible when an independent operator, given
the same starting structure, the same engine and force-field family at
the recorded version, the same water model and ion treatment, the same
ensemble and integrator parameters, and the same random-number-generator
seed if specified, produces a trajectory whose statistical observables
agree with the original within the published reproducibility tolerance.

Force fields evolve; a programme MUST pin the force-field family and
version in every simulation record and MUST emit a new simulation record
when the force-field version is bumped, rather than overwriting the
existing record. Programmes that publish externally cited results
SHOULD periodically re-run the simulation against the latest force-field
revision and publish the comparison so that downstream readers can judge
how robust the result is to force-field choice.

## §4 Experimental Laboratory Protocols

Experimental laboratories operate under ISO/IEC 17025:2017 procedures
with modality-specific extensions:

- **NMR relaxation**: probes are calibrated with dedicated standards
  before each campaign; magnetic-field stability is verified daily;
  uncertainty contributions include shimming residuals and reference-
  compound concentration uncertainties.
- **Single-molecule FRET**: instrument response functions are recorded
  at the start of each campaign; donor-only and acceptor-only controls
  are run alongside dual-labelled samples; uncertainty includes
  background-subtraction and crosstalk-correction contributions.
- **HDX-MS**: deuterium-uptake calibration uses internal reference
  peptides; uncertainty includes back-exchange correction.
- **Time-resolved X-ray scattering**: sample radiation dose is bounded
  by the published damage threshold and the dose is recorded against
  each measurement.

The programme's quality dossier (PHASE-3 §11) documents the modality-
specific procedures it follows and the reference standards it uses.

## §5 Inter-Laboratory Round-Robins

Programmes that publish externally cited studies participate in
inter-laboratory round-robin exercises at least once every two calendar
years per active modality. The round-robin protocol follows
ISO/IEC 17043:2010 and ISO 5725 expectations for reproducibility
studies. The round-robin coordinator is the certifying body or a body
it nominates.

Round-robin reports are appended to participating programmes' records
and are referenced from the programme's well-known discovery document.
A programme that does not participate in the most recent round-robin
MAY still publish studies but MUST flag the studies as
`single-laboratory-characterised` in the public catalogue.

## §6 Sampling Tolerance and Convergence

Computational dynamics campaigns publish a convergence assessment as
part of the analysis output. Common assessments include block-averaged
errors on observables, autocorrelation-time analysis, and
multiple-replica reproducibility. The chosen assessment is recorded in
the analysis record so that consumers can judge whether the campaign
is sufficient for the claimed observable.

A programme that publishes an externally cited free-energy surface or
Markov state model SHOULD include a convergence assessment that meets
the community-recommended thresholds for that observable, recognising
that the threshold is observable-specific and is not prescribed here.

## §7 Records Retention

Programme records — every record defined in PHASE-1, the API audit logs
defined in PHASE-2 §13, simulation trajectories, raw observables, and
the agreements and certificates defined in this PHASE — are retained
for a minimum of seven calendar years from the last access of the
study. Externally cited studies are retained indefinitely as long as
the programme is operating; on programme wind-down, indefinite-retention
records are transferred to a recognised long-term archive.

## §8 Time Synchronisation

Programmes operate on synchronised time per RFC 5905 (NTPv4) so that
simulation, experiment, and analysis events can be ordered unambiguously
across instruments and across sites. The reference time source is the
national metrological laboratory's stratum-1 service or an equivalent
recognised service; the operator records the source and the maximum
observed offset in the programme's quality dossier.

## §9 Programme Wind-Down

A programme that ceases operations transitions all draft studies to
`deprecated`, transfers indefinite-retention records to a recognised
long-term archive, and notifies known external citers via the well-known
discovery document. Externally pinned evidence packages remain accessible
through the archive at the same content-addressed URLs.

## §10 Welfare and Ethics

Protein-dynamics work that involves human biospecimens follows the
governance framework of the source biobank, which itself operates under
the ethics-committee approvals applicable in its jurisdiction. The
programme records the relevant approvals (institutional review board /
ethics committee references) against each preparation that derives from
a biospecimen.

For animal-derived biospecimens, the programme records the welfare-
committee approval reference against the preparation. The programme
does not re-derive a welfare framework here; it consumes the framework
of the source institution.

## §11 Quality Dossier

The programme's quality dossier is a written document maintained at a
publicly resolvable URL. The dossier records the programme's
accreditation scopes, the modalities it operates, the force-field
revisions it tracks, the round-robin exercises it has participated in,
and the deprecation history of its published studies. The dossier is
reviewed at least annually by the programme's quality manager and is
read during the annual ISO/IEC 17025 surveillance audit.

## §12 Cross-Border Programme Operation

A programme that operates across borders maintains a primary jurisdiction
of registration and one or more operating jurisdictions where laboratory
or computational activity occurs. Cross-border data transfers honour the
data-protection law of the source biospecimen's jurisdiction, which is
recorded in the operator's collaboration agreements at the start of each
project.

## §13 Reproducibility Receipts

Every externally cited result carries a reproducibility receipt: the
content-addresses of the starting structure, the simulation or
observable record, the analysis pipeline, the locked dependency
environment, and the cross-validation report. The receipt is emitted as
part of the evidence package (PHASE-4 §3) and is the citation target
that downstream readers pin.

## §14 Citation Hygiene and Successor Relationships

A programme is responsible for the integrity of citations to its
studies. When an external publication cites a study that the programme
has subsequently revised (force-field bump, analysis re-run with a
corrected pipeline) or deprecated (consent revocation, instrument
recall), the programme MUST keep the originally cited evidence package
addressable at its content-addressed URL and MUST publish a successor
relationship in the discovery document so that downstream readers can
navigate from the cited study to its current revision.

When the successor differs from the predecessor in conclusion (e.g. a
re-run with a corrected pipeline produces a different free-energy
ordering), the successor record carries an explicit `supersededReason`
field that describes the difference. Programmes SHOULD reach out to
known external citers when a substantive supersession occurs, but the
obligation to maintain addressability does not depend on successful
outreach; the content-addressed URL is the durable contract.

## §15 Pre-Registration of Hypothesis-Driven Studies

A programme that publishes hypothesis-driven dynamics results SHOULD
pre-register the analysis plan before the simulation campaign or the
experiment is run. Pre-registration records the observables that will be
analysed, the statistical tests that will be applied, and the criteria
for accepting or rejecting the hypothesis, and is deposited at a
recognised pre-registration registry whose identifier is held against
the study record.

Pre-registration is not a conformance requirement under PHASE-3, but
the public catalogue distinguishes pre-registered studies from
exploratory studies so that downstream readers can weigh evidence
accordingly.

## §16 Conformance and Auditing

A programme conformant with WIA-protein-dynamics publishes its
accreditation certificate, its programme code registration, its quality
dossier, and the catalogue of studies it has published, and answers an
annual self-assessment that maps each clause of this PHASE to the
programme's implementation. The self-assessment is reviewed during the
annual ISO/IEC 17025 surveillance audit.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-protein-dynamics
- **Last Updated:** 2026-04-27
