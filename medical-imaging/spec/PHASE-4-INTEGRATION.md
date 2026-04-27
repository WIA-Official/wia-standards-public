# WIA-medical-imaging PHASE 4 — Integration Specification

**Standard:** WIA-medical-imaging
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable

This PHASE describes how a deployment integrates the imaging data,
APIs, and protocols defined in PHASEs 1–3 with adjacent systems:
RIS work-flow, PACS/VNA storage, modality fleets, viewers, AI
inference services, research data warehouses, and tele-radiology
exchange. It is non-prescriptive about specific vendor products;
it specifies the integration *contracts* a deployment must satisfy.

References (CITATION-POLICY ALLOW only):
- IHE Radiology Technical Framework — Scheduled Workflow (SWF.b),
  Cross-Enterprise Document Sharing for Imaging (XDS-I.b),
  Imaging Object Change Management (IOCM), Mammography Image (MAMMO)
- DICOM PS3.4 — Service Class Specifications
- DICOM PS3.15 — Security Profiles
- DICOM PS3.18 — Web Services
- HL7 FHIR R5 — ImagingStudy, DiagnosticReport, ImagingSelection
- WIA-medical-data-privacy (PHASE 1–4)

---

## §1 RIS-driven workflow

The Radiology Information System (RIS) drives orders. The
integration contract:

- The RIS publishes orders as FHIR ServiceRequest resources, each
  referencing a Patient (`subjectRef`), an Encounter, and a
  procedure code
- The boundary projects approved ServiceRequests into UPS-RS
  work-items consumed by modalities (PHASE 2 §6)
- Completion of a work-item updates the originating ServiceRequest
  status (`completed`, `revoked`, etc.) and links the produced
  ImagingStudy

The RIS owns the schedule and the procedure code; the PACS owns
the produced images; the boundary owns the privacy gate and the
audit chain. Each owns its own truth.

## §2 PACS / VNA integration

The deployment may run a single PACS, a vendor-neutral archive
(VNA) backed by multiple departmental PACSes, or a cloud-native
imaging store. The integration contract:

- DICOM Storage SCP exposed for STOW-RS (PHASE 2 §5) and for
  C-STORE on the upper layer
- Query/Retrieve services (QIDO-RS, WADO-RS, C-FIND, C-MOVE)
  exposed through the boundary
- Lifecycle hooks for IOCM events (study merged, study split,
  patient ID corrected, instance rejected) so that the audit
  chain reflects retroactive corrections

Multi-PACS VNA deployments register their constituent storage
domains with the boundary so that QIDO queries fan out and
results aggregate — the caller sees a single virtual archive.

## §3 Modality fleet

Modality devices run firmware versions, hold local cache, and
participate in clinical workflow. The integration contract:

- Each modality registers an Application Entity (AE) title and
  a TLS client certificate with the deployment
- Modality firmware versions are tracked in the deployment's
  configuration management database; firmware affecting
  acquisition technique or pixel encoding requires a re-validation
  cycle before being routed live traffic
- Modalities emit DICOM PS3.15 audit messages to the boundary's
  ATNA receiver; the boundary projects these into the AuditEvent
  chain

Modality replacement preserves AE-title continuity (the new
modality acquires the old AE title only after the old modality's
last study is fully audited and retired).

## §4 Viewer integration

Diagnostic viewers retrieve via WADO-RS (or DICOMweb's RESTful
metadata + bulk data path). Integration contract:

- Viewers authenticate via SMART App Launch and carry purpose
  headers (TREAT for diagnostic reading; HOPERAT for QA review)
- Viewer-side annotations (Presentation States, Key Object
  Selections, Structured Reports) are stored back via STOW-RS
  with provenance referencing the source instances
- Hanging protocols (PHASE 3 §5) ensure consistent layout across
  viewers in the deployment

A viewer that bypasses the boundary (direct PACS access) is
non-conformant; the deployment policy SHOULD block direct PACS
network access except from the boundary itself.

## §5 AI inference services

AI services (CADx, segmentation, prioritisation) consume images
and produce annotations. Integration contract:

- AI services are registered with their FDA / KFDA / CE / NMPA
  clearance numbers and intended-use statements; the boundary
  refuses to route to an unregistered service for clinical use
- Inputs to AI services are gated by purpose `HOPERAT` (operations)
  unless the service is part of a clinical workflow tied to a
  patient consent for `TREAT`
- AI outputs are stored as new DICOM instances (Segmentation, SR,
  or Comprehensive 3D SR) carrying the algorithm identifier
  (PHASE 1 §9)
- AI confidence scores are recorded but not displayed to clinicians
  without context — the deployment's clinical governance committee
  signs off on the disclosure rules per service

Research-purpose AI inference (model evaluation against historic
extracts) flows through the de-identified extract pipeline and
does not affect production routing.

## §6 Research data warehouse

Research access to imaging follows WIA-medical-data-privacy PHASE 4
§2 plus imaging-specific obligations:

- Each research extract names the PS3.15 de-identification profile
  in use and the residual-risk band per the de-identification
  job record
- Pixel-level burn-in (text or PHI in pixels) is cleaned per the
  configured cleanup region; a study with pixel-level PHI in a
  region not covered by the cleanup configuration is rejected for
  research export
- The exported NDJSON Bundle includes ImagingStudy resources only
  (with references); the actual DICOM bulk data is held in the
  research-store's WADO-RS endpoint and retrieved on demand by
  the analyst's workflow

Re-identification of a research extract (mapping a de-identified
StudyInstanceUID back to a patient) requires a linkage
authorisation per WIA-medical-data-privacy PHASE 1 §5.

## §7 Tele-radiology exchange

Cross-organisation reading (a remote radiologist reads images
acquired at a different site) follows IHE XDS-I.b plus the
boundary's purpose gate:

- The receiving organisation's reading radiologist holds a SMART
  grant naming TREAT scope on the originating organisation's
  patients (typically negotiated bilaterally and renewed
  per-quarter)
- A read session retrieves only the studies named in the
  current case list; opportunistic browsing is not permitted
- Returned reports flow back via STOW-RS and reverse-projection
  to FHIR DiagnosticReport in the originating system

The two organisations share an audit chain segment for the
exchange duration; on session end, the segment is sealed and
both audit chains record the seal.

## §8 IOCM corrections

Imaging Object Change Management (IHE-RAD IOCM) handles after-the-
fact corrections:

- Study merged: two studies for the same encounter unified under
  one StudyInstanceUID; affected references updated; audit chain
  records both before and after
- Study split: misattributed instances moved to their correct
  patient; consent gates re-evaluated; previous reads against
  the misattributed series flagged for review
- Instance rejected: an instance withdrawn from clinical use
  (e.g., wrong patient) marked rejected (PS3.3 IOCM rejection
  reason codes); the rejected instance remains in the store
  for audit but is excluded from clinical retrievals

## §9 Acceptance criteria

A deployment claims conformance when:

1. Every clinically-routed instance has a matching audit chain
   entry within the operational SLA
2. Every cross-organisation exchange has a valid bilateral grant
   on file for the duration of the exchange
3. The de-identification engine has been validated against the
   deployment's modality fleet (no escaped PHI in the
   most-recent quarterly spot-check sample)
4. AI services in production have current regulatory clearance
   on file
5. Modality firmware tracking is up to date (no untracked firmware
   in the configuration database)
6. IOCM corrections are reflected in the audit chain within the
   operational SLA after the correction is issued

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Modality-fleet configuration (informative)

The configuration database holds, per modality:

- AE title (DICOM identifier)
- Vendor, model, serial number
- Firmware version and last firmware update timestamp
- Active TLS certificate fingerprint and expiry
- Pixel-cleanup region(s) used by de-identification
- Default transfer syntax(es) used at storage
- Active routing rules (which PACS / VNA receives which series type)
- Validation status (validated / not validated for current firmware)

A modality not in the database, or with a firmware version not
matching the database record, is refused association by the
boundary.

## Annex B — Vendor-neutral storage notes (informative)

VNA deployments aggregate multiple departmental archives. The
boundary handles aggregation:

- A QIDO query at the boundary fans out to each constituent
  archive and merges results
- Duplicate StudyInstanceUIDs across archives are an integrity
  incident (UIDs are globally unique by construction); the
  boundary refuses to merge and routes the conflict to the
  imaging operations team
- Storage tiering (hot / warm / cold) is internal to the VNA;
  the boundary does not see tier transitions and does not need to

## Annex C — Operational SLAs (informative)

Default operational SLAs for a deployment:

| Concern                                          | SLA                              |
|--------------------------------------------------|----------------------------------|
| Audit-chain entry available after instance store | ≤ 5 seconds                      |
| Boundary p95 added latency                       | ≤ 50 ms                          |
| Modality-to-PACS first byte after C-STORE start  | ≤ 200 ms                         |
| WADO-RS bulk first byte                          | ≤ 500 ms                         |
| Study merge audit reflection (PHASE 4 §8)        | ≤ 60 minutes after IOCM correction |
| Daily-root publication                           | ≤ 5 minutes after UTC midnight   |

The deployment policy SHOULD tighten these SLAs where modality
fleet capability allows; loosening them requires sign-off from
clinical operations.

## Annex D — Audit reporting cadence (informative)

The boundary emits a quarterly audit report covering:

- Total studies stored and retrieved
- Studies filtered by consent (count only, no patient-level
  detail)
- Break-glass invocations (count, average review-time, count
  of overdue reviews)
- AI inference events and their associated regulatory clearances
- IOCM corrections issued and their downstream impacts
- Cross-organisation exchange volumes per partner

The report is signed and is itself in scope for the audit chain so
that report tampering would surface in the chain.

## Annex E — Migration from legacy archive (informative)

A deployment migrating from a legacy PACS into a boundary-fronted
deployment follows a one-time backfill:

1. Export the legacy archive's metadata (DICOM JSON or HL7 v2 ORU)
2. Wrap each instance with a FHIR ImagingStudy resource at the
   boundary; the original DICOM remains canonical, the FHIR resource
   is a projection
3. Insert AuditEvent records for the import, signed at insertion
   time, with the prior storage time preserved in `entity.detail`
4. The chain genesis root for the post-migration era references the
   final wrapped event's hash so audit continuity is preserved

This is non-lossy: auditors can still read the legacy DICOM exactly
as it was; the boundary projection is a layered view, not a
replacement.

## Annex F — Imaging-AI service registration

The deployment maintains a registry of approved AI inference
services:

```yaml
ai_services:
  - id: vendor-A.lung-nodule-cadx
    intended_use: "Detect pulmonary nodules in chest CT, ≥3mm"
    regulatory:
      kfda: "제2025-1234호"
      fda: "K251234"
    input_modality: CT
    output_iod: 1.2.840.10008.5.1.4.1.1.66.4   # Segmentation
    confidence_disclosure: aggregate-only
  - id: vendor-B.bone-age
    intended_use: "Estimate skeletal maturity from hand DX"
    regulatory:
      ce_class: "IIa"
    input_modality: DX
    output_iod: 1.2.840.10008.5.1.4.1.1.88.11   # Basic Text SR
    confidence_disclosure: per-image
```

A service not in the registry is refused for clinical routing.
Research-purpose evaluation against historic extracts is permitted
without registry entry but is gated by HRESCH purpose.
