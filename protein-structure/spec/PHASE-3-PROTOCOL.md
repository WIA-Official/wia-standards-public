# WIA-protein-structure PHASE 3 — PROTOCOL Specification

**Standard:** WIA-protein-structure
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern an accredited
protein-structure programme: laboratory accreditation, deposit
acceptance, mmCIF dictionary and Chemical Component Dictionary
versioning, prediction-service governance, validation procedures,
records retention, similarity-tool registration, citation
hygiene, and programme wind-down.

References (CITATION-POLICY ALLOW only):

- ISO/IEC 17025:2017 (testing and calibration laboratories)
- ISO/IEC 17043:2010 (proficiency testing)
- ISO/IEC 27001:2022 (information security management)
- ISO 9001:2015 (quality management systems)
- ISO 8601 (date and time)
- IETF RFC 5905 (NTPv4)
- IETF RFC 8446 (TLS 1.3)
- IETF RFC 9457 (Problem Details)
- HL7 FHIR R5 (`MolecularSequence`)

---

## §1 Laboratory Accreditation

A structural-biology laboratory MAY claim conformance to
WIA-protein-structure only after a recognised accreditation body
has issued a valid certificate against ISO/IEC 17025:2017 for the
methods the laboratory exercises (X-ray crystallography, cryo-EM,
solution NMR, etc.). The accreditation register is exposed to the
API as a read-only resource and MUST be re-fetched at least once
per calendar quarter.

A laboratory whose accreditation is revoked has its open deposits
flagged in the public catalogue; deposits already cited externally
remain addressable but are flagged with the accreditation lapse.

## §2 Deposit Acceptance

Deposits flow through an acceptance pipeline that validates the
mmCIF coordinate file against the in-force dictionary, runs
geometric and stereochemical checks, generates a wwPDB-style
validation report, and assigns the structure an external archive
accession when the deposit is mirrored externally. Deposits that
fail validation enter a remediation cycle with the depositor; the
record stays in the operator's draft state until the validation
issues are resolved.

## §3 mmCIF Dictionary Versioning

The mmCIF / PDBx dictionary evolves with revisions to data
categories and items. Programmes pin the dictionary version each
deposit conforms to in the coordinate record (PHASE-1 §3) and
preserve historical deposits at their original dictionary version
so that downstream consumers can re-parse without surprise.
Dictionary upgrades are announced through the well-known discovery
document with a deprecation timeline for prior versions.

## §4 Chemical Component Dictionary Governance

The Chemical Component Dictionary (CCD) catalogues canonical
ligand and modified-residue identifiers. New CCD entries are
registered through the wwPDB-led process; the operating programme
records the CCD version in force at deposit time and re-validates
historical deposits when CCD entries are subsequently revised.
Revisions that change ligand connectivity emit cross-reference
records (PHASE-1 §9) so that downstream consumers can navigate
the change.

## §5 Prediction-Service Governance

Prediction services that publish records into the API register
their engine family, version, and the input pipelines that produce
predictions. Records are signed by the service operator's release
key. Service operators MUST publish per-residue confidence
(pLDDT-style) and pairwise confidence (PAE-style) when the engine
emits them; predictions without confidence metadata are rejected
with `urn:wia:protein-structure:prediction-confidence-missing`.

A prediction service that ceases operation transitions its records
to `archived` status; the records remain addressable through the
long-term archive integration (PHASE-4 §10).

## §6 Validation Procedures

Validation reports follow the wwPDB-aligned procedures:
geometric/stereochemical checks (bond lengths, bond angles,
chirality, planarity), Ramachandran-plot occupancy review,
ligand-validation (ideal-vs-observed conformation), and method-
specific checks (resolution-vs-clash-score for crystallography,
local map quality for cryo-EM, NOE-violation analysis for NMR).
Validation reports are signed by the operating programme and
referenced from the structure record.

## §7 Records Retention

Deposited structures retain indefinitely; the wwPDB and equivalent
archives have multi-decade preservation commitments and the
operating programme deposits each release into the archive on the
agreed cadence. Validation reports, raw experimental data
archives, and prediction-confidence artefacts retain for at least
the operating programme's records-management baseline (typically
seven calendar years from last access, longer for externally cited
records).

## §8 Time Synchronisation

Programme clocks synchronise per RFC 5905 (NTPv4) so that deposit,
validation, and citation events can be ordered unambiguously
across laboratories, archives, and consumers.

## §9 Similarity-Tool Registration

Similarity-search tools (Foldseek, DALI, TM-align, and successors)
are registered with the operating programme so that similarity
records (PHASE-1 §8) can be re-executed against the same tool and
version. Tool registrations record the tool's identifier, the
version, the citation reference for the underlying algorithm, and
the recommended scoring conventions. Tool revisions emit new
registration entries; prior registrations remain addressable.

## §10 Citation Hygiene

Programmes are responsible for the integrity of citations to their
structures. When an external publication cites a structure that
the programme has subsequently superseded (a revised refinement,
a corrected sequence, an improved prediction), the programme
keeps the originally cited record addressable at its content-
addressed URL and publishes a successor relationship through the
discovery document so that downstream readers can navigate from
the cited record to its current revision.

## §11 Inter-Laboratory Round-Robin

Programmes that publish externally cited validation results
participate in inter-laboratory round-robin exercises at least
once every two calendar years. The round-robin protocol follows
ISO/IEC 17043:2010 expectations and is coordinated by the
certifying body.

## §12 Cybersecurity and Software Updates

Coordinate uploads are authenticated and integrity-protected over
TLS 1.3 (RFC 8446). Validation pipelines are signed by the
operating programme's release key; failed signature verification
on a pipeline component blocks deposits from running through that
component until the issue is resolved.

## §13 Cross-Border Programme Operation

Multi-jurisdiction programmes (international cooperatives,
mirrored archive consortia) honour each participating
jurisdiction's data-protection law for any sequence record that
cross-links to clinical-genomics data via FHIR.

## §14 Programme Wind-Down

A programme that ceases operations transfers indefinite-retention
records to a recognised long-term archive, notifies known external
citers via the well-known discovery document, and publishes a
sunset timeline for in-flight deposits. Externally pinned records
remain accessible through the archive at the same content-
addressed URLs.

## §15 Quality Dossier

The programme's quality dossier records the laboratories it works
with, the prediction services it integrates with, the dictionary
and CCD versions it tracks, the round-robin exercises it has
participated in, and the deprecation history of its records. The
dossier is reviewed at least annually by the operating
organisation's quality manager.

## §16 Provisional CCD Process

Modifications observed in experimental data that lack a CCD entry
follow a provisional-CCD process: the depositor flags the
modification as `provisional-ccd-<requestId>` (PHASE-1 §10), the
operating programme forwards the request to the CCD maintainer, and
the deposit is accepted with the provisional flag pending CCD
adjudication. Once the CCD entry is finalised, the operating
programme rewrites the PTM record to reference the canonical
identifier and emits a cross-reference recording the rewrite.

## §17 Predicted Structure Confidence Reporting

Predicted structures emit per-residue confidence (pLDDT-style
zero-to-one quality) and, where the engine supports it, pairwise
confidence (PAE-style residue-pair distance error). Programmes
publishing externally cited predictions MUST emit both when the
engine supports them; the API rejects predictions missing required
confidence fields.

Predicted structures that fall below the engine's recommended
confidence threshold for cited use are flagged in the public
catalogue so that downstream consumers do not consume low-
confidence predictions as if they were experimentally validated.

## §18 Embargo and Release Schedule

Deposits associated with a manuscript under journal review are
held under embargo until the manuscript publication date or the
embargo expiry, whichever is sooner. The operating programme
records the embargo holder, the expected release date, and the
notification list for embargo-lift events. Premature release of
embargoed records returns a Problem-Details (RFC 9457) response of
type `urn:wia:protein-structure:embargo-active` and is logged as
an audit event for retrospective review.

## §19 Annotation Source Versioning

Annotation sources (SCOP, CATH, ECOD, GO, InterPro, UniProt
feature tables) emit periodic releases. The operating programme
pins the source version each annotation conforms to so that a
re-fetch of the same annotation against a later source release
returns the historical pinning rather than the latest source.
Source-revision events emit cross-references when the revised
source materially changes a previously published annotation.

## §20 Coverage-Tracking and UniProt Synchronisation

Programmes track structural coverage of UniProt accessions over
time so that gaps in the experimental and predicted-structure
landscape are visible to consumers. Coverage records are
content-addressed and refresh on a cadence the operating programme
declares (typically monthly). A UniProt accession that loses
coverage (e.g. a structure is withdrawn) is flagged in the
streaming subscription so that downstream consumers can re-run
queries that depended on that coverage.

## §21 Cross-Method Reconciliation

Programmes that publish both experimental and predicted structures
for the same UniProt accession reconcile the records through
cross-references (PHASE-1 §9). When the predicted and experimental
structures disagree beyond the engine's published confidence
envelope, the operating programme records a reconciliation note
that explains the disagreement and points to follow-up records
(re-refinement, alternate conformation, prediction with updated
inputs). Reconciliation notes are content-addressed.

## §22 Programme Wind-Down

A programme that ceases operations transfers indefinite-retention
records to the wwPDB or an equivalent recognised archive,
notifies known external citers via the well-known discovery
document, and publishes a sunset timeline for in-flight deposits.

## §23 Withdrawal and Supersession Governance

Withdrawal and supersession (PHASE-1 §13) are governed under a
process that protects citation integrity. The operating programme
publishes a public notice naming the affected structure, the
reason, and the successor (when one exists). Subsequent consumers
that retrieve the affected structure receive the public notice
alongside the canonical record so that downstream readers see the
withdrawal context without bespoke checks.

## §24 Conformance and Auditing

A programme conformant with WIA-protein-structure publishes its
laboratory accreditation references, its programme code
registration, its quality dossier, the catalogue of structures it
has released, and the catalogue of cross-references it has
published, and answers an annual self-assessment that maps each
clause of this PHASE to the programme's implementation.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-protein-structure
- **Last Updated:** 2026-04-27
