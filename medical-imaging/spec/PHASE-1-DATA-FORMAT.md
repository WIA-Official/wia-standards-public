# WIA-medical-imaging PHASE 1 — Data Format Specification

**Standard:** WIA-medical-imaging
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable

This PHASE defines the canonical data format for medical-imaging
artefacts: study identifiers, series and instance metadata,
acquisition device descriptors, structured report bindings,
de-identified extracts, and the cross-references that bind imaging
records to clinical context. The shape is interoperable with DICOM
PS3.3 (information object definitions) and HL7 FHIR R5 ImagingStudy
so that an existing PACS or VNA can adopt this PHASE without
inventing parallel data models.

References (CITATION-POLICY ALLOW only):
- DICOM PS3.3 — Information Object Definitions (NEMA PS3.3)
- DICOM PS3.4 — Service Class Specifications
- DICOM PS3.6 — Data Dictionary
- DICOM PS3.10 — Media Storage and File Format
- DICOM PS3.15 — Security and System Management Profiles
- DICOM PS3.18 — Web Services (DICOMweb: WADO-RS, QIDO-RS, STOW-RS)
- HL7 FHIR R5 — ImagingStudy (R5/imagingstudy.html), DiagnosticReport (R5/diagnosticreport.html)
- IHE Radiology Technical Framework — Scheduled Workflow (SWF.b), Cross-Enterprise Document Sharing for Imaging (XDS-I.b)
- ISO 12052:2017 — Health informatics: DICOM
- IETF RFC 7515 (JWS), RFC 7519 (JWT) — for non-DICOM signing layers

---

## §1 Scope

This PHASE applies to systems that acquire, store, route, display,
or analyse medical images and the associated structured reports.
It addresses the imaging-specific shape of data; cross-cutting
privacy concerns (consent, audit, retention) defer to
WIA-medical-data-privacy. The two standards are complementary —
this one names the imaging artefacts; the privacy standard names
the consent gates that surround disclosure of those artefacts.

In scope: cross-sectional (CT, MR), nuclear (PT, NM), projection
(CR, DX, MG), ultrasound (US), fluoroscopy (XA, RF), endoscopy
(ES), pathology whole-slide imaging (SM), and ophthalmic imaging
(OP, OPT). Out of scope: vendor-proprietary streaming protocols
that do not yield DICOM-conformant artefacts at rest.

## §2 Identifier hierarchy

DICOM models imaging as a four-level hierarchy. This PHASE
preserves that hierarchy and binds it to the privacy boundary's
pseudonymous subject identifier:

| Level    | DICOM identifier         | Binding                                   |
|----------|--------------------------|-------------------------------------------|
| Patient  | (0010,0020) PatientID    | mapped to `subjectRef` per §3 below       |
| Study    | (0020,000D) StudyInstanceUID | one per imaging encounter             |
| Series   | (0020,000E) SeriesInstanceUID | one per acquisition protocol         |
| Instance | (0008,0018) SOPInstanceUID  | one per stored object                  |

UIDs are ISO/IEC 9834-3 OIDs in DICOM dotted-decimal form, root-
allocated under the deployment's DICOM root UID (registered with
the controlling jurisdiction's medical-device registrar). UIDs are
*globally unique and never reused* — even after deletion the UID
is retained so that audit trails referencing it remain meaningful.

## §3 Patient identifier mapping

The DICOM `PatientID` element (0010,0020) holds the deployment's
internal medical-record number (MRN). In every disclosure outside
the controller's boundary, this element is replaced with the
pseudonymous `subjectRef` defined in WIA-medical-data-privacy
PHASE 1 §2. The original `PatientID` is preserved only inside the
controller's identity broker.

`PatientName` (0010,0010) and `PatientBirthDate` (0010,0030) are
direct identifiers and follow the same boundary rule. In de-identified
extracts the `PatientName` is set to a synthetic stable token and
the `PatientBirthDate` is generalised (year-only by default; per-
study date offsets are applied uniformly across that subject's
studies to preserve temporal relationships).

## §4 Acquisition metadata

Each Series carries acquisition metadata describing how the
image was obtained:

- Modality (0008,0060) — closed value set per DICOM PS3.16 CID 29
- Manufacturer (0008,0070), Model Name (0008,1090), Software Versions (0018,1020)
- Acquisition geometry — slice thickness (0018,0050), spacing between
  slices (0018,0088), pixel spacing (0028,0030), rows/columns (0028,0010/11)
- Acquisition technique — for CT: KVP (0018,0060), exposure (0018,1152),
  CTDIvol (0018,9345); for MR: scanning sequence (0018,0020), TR/TE
  (0018,0080/81), magnetic field strength (0018,0087)
- Contrast administration — agent (0018,0010), volume, route, timing
- Time stamps — acquisition date (0008,0022), acquisition time
  (0008,0032), all in UTC with offset; series time and study time
  consistent with the patient encounter

Acquisition metadata is read-only post-storage. Corrections require
issuing a new instance referencing the prior with a corrigenda
relationship; the prior instance remains in the store for audit.

## §5 Pixel data and transfer syntax

Pixel data lives at (7FE0,0010) and is encoded by a Transfer Syntax
UID (0002,0010 in the file meta). The standard accepts these
transfer syntaxes for new acquisitions:

| UID                            | Encoding                                  |
|--------------------------------|-------------------------------------------|
| 1.2.840.10008.1.2.1            | Explicit VR Little Endian (uncompressed)  |
| 1.2.840.10008.1.2.4.70         | JPEG Lossless, hierarchical (SV1)         |
| 1.2.840.10008.1.2.4.90         | JPEG 2000 Lossless                        |
| 1.2.840.10008.1.2.4.91         | JPEG 2000 (lossy, primary capture only)   |
| 1.2.840.10008.1.2.4.81         | JPEG-LS Lossless                          |
| 1.2.840.10008.1.2.4.92/93      | JPEG 2000 Multi-component                 |
| 1.2.840.10008.1.2.4.110        | Deflated Explicit VR Little Endian        |

Lossy compression for diagnostic intent is rejected; primary
capture lossy is permitted only when the modality emits no
lossless option. Trans-coding from one accepted syntax to
another preserves the original UID in (0008,1110) ReferencedStudy
so that auditors can confirm the lineage.

## §6 Structured reports

Structured reports use DICOM SR (Modality SR, IOD = Basic Text
or Enhanced SR). The report references the imaging study, the
interpreting physician, the report's verification state, and
the codified findings. Reports are also represented in HL7 FHIR
R5 DiagnosticReport for non-DICOM consumers; the DICOM SR remains
the source of truth, the FHIR resource is a projection.

Codified findings draw from RadLex and SNOMED CT. The FHIR
DiagnosticReport's `code` and `conclusionCode` reference the same
codes; mismatch between the SR and the FHIR projection is treated
as a data-integrity incident.

## §7 Imaging-specific de-identification

De-identification in DICOM follows PS3.15 Annex E "Application
Confidentiality Profiles". This PHASE binds:

- Default profile: PS3.15 Basic Profile + Clean Pixel Data Option
  (suspect text in pixel rows 0–N is overpainted)
- Clean Descriptors Option for free-text comments (0008,4000),
  derivation description (0008,2111), and patient comments
  (0010,4000)
- Retain Patient Characteristics for sex (0010,0040) and patient
  weight (0010,1030) only when the study type requires them for
  dosimetry (CT, NM, PT)
- Retain Longitudinal Temporal Information with Date Modification
  (uniform per-subject offset)

The de-identification job record from WIA-medical-data-privacy
PHASE 1 §7 references the PS3.15 profile in use, the pixel-cleanup
algorithm, and the residual-risk band.

## §8 Linkage to clinical context

An ImagingStudy (FHIR R5) references one or more DiagnosticReport
resources, an Encounter, and a Patient. Linking imaging to clinical
context is gated by the privacy boundary's purpose check. A research
extract that requires linking imaging to outcomes (e.g., cancer
recurrence) MUST hold a linkage authorisation per WIA-medical-data-
privacy PHASE 1 §5; the linkage authorisation names the imaging
study, the clinical resource, and the linked subject.

## §9 Provenance for derived images

Derived images (reformats, MIP/MPR, segmentation masks, AI-generated
annotations) carry a Provenance reference to the source instance(s)
and the algorithm that produced them:

- (0008,2111) Derivation Description
- (0008,9215) Derivation Code Sequence (DICOM PS3.16 CID 7203)
- (0040,A124) UID of the derivation step

For AI-generated outputs, the algorithm identifier (0066,002F),
the algorithm version, and the model's regulatory clearance ID
(when applicable) are recorded so that downstream readers can
audit which model produced which annotation.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Modality-specific obligations (informative)

Different modalities carry distinct PHI risk profiles in pixel
data and metadata.

- **CT, MR, PT, NM** — burned-in patient identifiers are rare in
  modern modalities but can appear in scout/topogram images;
  the deployment's pixel-cleanup region covers these
- **CR, DX, MG** — burn-in is more common (annotations, technique
  labels); cleanup covers the configured rows
- **US** — burn-in routine (institution name, patient name);
  cleanup covers a larger header region per probe type
- **OP, OPT** — burn-in occasional; check scanning-laser
  ophthalmoscope outputs specifically
- **SM (whole-slide)** — large file sizes (gigapixels) require
  pixel-cleanup at acquisition rather than at de-identification
  time

The deployment's modality-fleet configuration declares the cleanup
region per modality model so that the de-identification engine can
apply it deterministically.

## Annex B — Multi-frame and enhanced IODs (informative)

Enhanced multi-frame IODs (Enhanced CT, Enhanced MR, Enhanced PT,
Multi-frame Grayscale Word) consolidate per-frame metadata into a
shared functional groups sequence (5200,9229) and per-frame
sequence (5200,9230). The deployment SHOULD prefer enhanced IODs
where modality firmware supports them; legacy single-frame IODs
remain valid but each frame stored as a separate SOP instance has
audit-chain implications (more events, more storage, more cross-
references).

## Annex C — Cross-references between standards (informative)

The medical-imaging standard references several other WIA
standards by intent:

| Reference                    | Use site                                              |
|------------------------------|-------------------------------------------------------|
| WIA-medical-data-privacy     | consent gate, audit chain, retention enforcement       |
| WIA-medical-iot              | modality fleet device telemetry (planned in v1.1)      |
| WIA-medical-ai-ethics        | AI inference governance (PHASE 4 §5)                  |
| WIA-prosthetic-control       | imaging-derived prosthetic fitting workflows           |
| WIA-medical-alert-system     | imaging-finding-driven alerts                          |

Each cross-reference is non-normative for this PHASE and points
to the canonical source of the cross-cutting concern. A deployment
SHOULD claim conformance with the cross-referenced standards
together with this one when the deployment touches both domains.

## Annex D — Conformance disclosure

Implementations declare per-section conformance in their published
DICOMweb conformance statement (PHASE 2 §9 Annex B). Sections marked
`partial` reference the deployment policy explaining the gap.
Sections marked `excluded` carry a reason citing the controlling
jurisdiction's allowance for that exclusion. An implementation that
is `partial` or `excluded` on §3 (Patient identifier mapping),
§7 (De-identification), or §9 (Provenance for derived images) is
non-conformant overall and cannot claim Deep v3 status.

## Annex E — Pixel-cleanup region examples (informative)

The pixel-cleanup region is modality-specific and modality-version-
specific. Examples below illustrate the shape of the configuration;
production deployments tune the region per their actual modality
fleet output.

```yaml
modality_cleanup:
  CT:
    "Siemens Somatom go.Top":
      regions: [{rows: [0, 60], description: "vendor banner with patient name"}]
    "GE Revolution":
      regions: [{rows: [0, 80], description: "patient banner"}]
  MR:
    "Philips Ingenia":
      regions: [{rows: [0, 70], description: "study label and patient name"}]
  US:
    "GE Voluson E10":
      regions: [{rows: [0, 100], description: "header with hospital and patient name"}]
      probe_specific:
        cardiac: {rows: [0, 120], description: "extended header for echo"}
  DX:
    generic: {rows: [0, 50], cols: [0, 200], description: "left-corner annotation block"}
```

A study from a modality model not present in the configuration
fails the de-identification gate; a configuration entry must exist
before research extraction is permitted on that modality model. The
deployment SHOULD validate new modality firmware against the
configured cleanup region before production traffic.

## Annex F — Pathology and ophthalmic notes (informative)

Whole-slide imaging (SM) and ophthalmic imaging (OP, OPT) have
data-format quirks worth calling out:

- **Whole-slide** — gigapixel images stored as tiled multi-resolution
  pyramids; transfer-syntax UID typically 1.2.840.10008.1.2.4.50
  (JPEG Baseline) per tile; cleanup at acquisition is preferred
  because de-identification at extraction time scales poorly
- **Ophthalmic 3D OCT** — Enhanced OPT IODs have per-frame functional
  groups; the deployment SHOULD prefer Enhanced over legacy OPT
- **Stereoscopic** — paired left/right images use Stereoscopic
  IODs; both pair members share a series and a study identifier
