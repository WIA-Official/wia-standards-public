# WIA-medical-imaging PHASE 3 — Protocol Specification

**Standard:** WIA-medical-imaging
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable

This PHASE specifies the protocols that bind the imaging data format
(PHASE 1) to the API surface (PHASE 2): DICOM upper layer, DICOMweb
transport, modality-to-PACS routing, image-display profiles, and the
audit trail for imaging operations. Where a domain protocol exists
(DICOM PS3.7, IHE Radiology integration profiles), this standard
defers; gaps are specified here.

References (CITATION-POLICY ALLOW only):
- DICOM PS3.7 — Message Exchange (DIMSE-C, DIMSE-N services)
- DICOM PS3.8 — Network Communication Support for Message Exchange
- DICOM PS3.15 — Security and System Management Profiles
- DICOM PS3.18 — Web Services
- DICOM PS3.20 — Imaging Reports using HL7 Clinical Document Architecture
- IHE Radiology — Audit Trail and Node Authentication for Radiology, Consistent Presentation of Images, Cross-Enterprise Document Sharing for Imaging
- IETF RFC 8446 (TLS 1.3), RFC 9293 (TCP)

---

## §1 Transport options

A deployment supports both DICOM upper-layer (PS3.7/PS3.8) and
DICOMweb (PS3.18) transports. The two have distinct strengths:

| Property             | DICOM upper layer (DIMSE)      | DICOMweb (HTTP)                  |
|----------------------|--------------------------------|----------------------------------|
| Modality vendor support | Universal (all imaging devices) | Newer modalities                 |
| Latency              | Single-association, persistent  | HTTP per call                    |
| Firewall friendliness | Custom port (104, 11112)       | 443 (HTTPS)                      |
| Cloud-native         | Awkward                        | Native                           |
| Streaming            | Yes (C-STORE, C-MOVE)          | Bulk via DICOMweb chunked        |

Modality acquisition typically uses DIMSE; viewer/EHR retrieval
typically uses DICOMweb. The boundary translates between them so
that downstream callers see a single privacy-gated surface.

## §2 DICOM upper-layer protocol

Associations follow PS3.8: ACSE A-ASSOCIATE → presentation contexts
→ DIMSE messages → A-RELEASE. The boundary enforces:

- TLS 1.3 on the upper-layer association (DICOM PS3.15 §B; the
  legacy "Basic TLS Profile" using TLS 1.2 is permitted only on
  legacy-modality associations explicitly listed in the deployment
  policy)
- Mutual authentication using X.509 certificates issued by the
  deployment's PKI; the modality's Application Entity Title is
  bound to its certificate so that AE-title spoofing fails the
  TLS check
- Per-association audit (DICOM PS3.15 §A) emitted to the
  AuditEvent chain shared with the privacy boundary

C-STORE, C-FIND, C-MOVE, and C-GET each carry the calling AE
title; the boundary maps that to a SMART system principal so that
the call appears in the audit chain with the same shape as a
DICOMweb call.

## §3 DICOMweb protocol

DICOMweb (PS3.18) is HTTP/1.1 or HTTP/2 over TLS 1.3. The boundary:

- Terminates TLS at its edge using a deployment-managed certificate
- Validates the SMART access token (per WIA-medical-data-privacy
  PHASE 3 §1) before forwarding to the backing service
- Forwards privacy headers (`WIA-Purpose-Of-Use`, etc.) into the
  DICOMweb context so that the downstream PACS sees the same
  purpose declaration

For cloud-hosted PACS the boundary additionally signs the
forwarded request body with a backend-only signing key so that
the PACS can verify the request originated from the boundary
(defence in depth: stolen tokens cannot bypass the boundary).

## §4 Modality-to-PACS routing

A modality emits one or more series per acquisition and routes
them to the PACS. The deployment's modality work-list (PHASE 2
§6 UPS-RS) drives the destination — the modality knows where to
route because the work-item carries the receiving AE.

Routing failures (network, AE rejection, transfer-syntax mismatch)
are logged as IHE-RAD audit events and queued for retry. A modality
that fails to deliver after the deployment's retry budget elevates
to the imaging operations team; the boundary emits a structured
incident record so that the trail is auditable.

## §5 Image display protocol

Viewers retrieve images via WADO-RS. The boundary supports:

- Window/level and pixel transformation hints in PS3.4 N-GET
  responses
- IHE Consistent Presentation of Images — the same instance
  rendered by two viewers MUST produce the same pixel output
  given the same presentation parameters
- Hanging protocols (DICOM PS3.3 Hanging Protocol IOD) — preserved
  as instances and retrievable like any other DICOM object

Viewer side-effects (annotations, key images) are written back as
new DICOM instances (Presentation State, Key Object Selection)
referencing the source; the source instance is not modified.

## §6 Imaging audit (PS3.15 + boundary)

DICOM PS3.15 §A.5 defines a closed set of audit message schemas.
This PHASE binds them onto the AuditEvent chain shared with
WIA-medical-data-privacy PHASE 3 §4:

| DICOM audit event           | FHIR AuditEvent type                |
|-----------------------------|-------------------------------------|
| Patient Record              | rest / read or update on Patient    |
| Study used                  | rest / read on ImagingStudy         |
| Begin transferring DICOM instances | rest / create on ImagingStudy.series |
| DICOM instances accessed    | rest / read on Binary               |
| Application activity (start/stop) | application activity event       |
| Security alert              | security alert                      |
| Audit log used              | audit log read                      |

Mapping is handled at the boundary; the PACS continues to emit
its native PS3.15 audit messages and the boundary projects them
into the chain.

## §7 Compression and trans-coding

Trans-coding from one accepted transfer syntax to another (PHASE 1
§5) preserves a derivation chain. The protocol:

1. Source instance read via WADO-RS (or C-GET on DIMSE)
2. Trans-code engine applies the target syntax
3. New instance stored with `(0008,1110) ReferencedStudySequence`
   pointing to the source and `(0008,9215) DerivationCodeSequence`
   coded as PS3.16 CID 7203 "Code recoding"
4. The trans-coded instance acquires a fresh `SOPInstanceUID`;
   the source UID is preserved in the reference
5. The original instance is not deleted unless the deployment's
   retention policy explicitly allows lossless re-derivation

Trans-coding is auditable; trans-coding to a lossy syntax for
diagnostic intent is rejected.

## §8 De-identification protocol

The de-identification engine implements PHASE 1 §7. The protocol:

1. Source instance retrieved with the requestor's purpose
   (HRESCH, EDU, etc.)
2. The configured PS3.15 profile is applied: tags are removed,
   replaced, or generalised per the profile's action codes
3. Pixel-level cleanup (overpaint of suspect text) runs against
   the configured pixel-cleanup region (modality-specific defaults
   are listed in the deployment policy)
4. The de-identified instance is stored with a fresh UID under the
   research store's UID root; the original UID is preserved in the
   de-identification job record only (not on the instance itself)
5. The de-identification job record (WIA-medical-data-privacy
   PHASE 1 §7) references the configured profile, the pixel
   cleanup algorithm, and the residual-risk band

The PHI-bearing original is not altered; the de-identified copy
is a new artefact.

## §9 Failure modes

| Failure                              | Behaviour                                            |
|--------------------------------------|------------------------------------------------------|
| Modality association rejected        | Modality retries per its routing rules; boundary logs|
| TLS handshake failure with PACS      | Connection abandoned; IHE security alert emitted     |
| Transfer-syntax negotiation failure  | Boundary returns 415; modality logs; ops team paged  |
| UID collision on STOW-RS             | Original instance preserved; new instance rejected   |
| Audit-chain write failure            | Imaging operation rejected (consistency w/ MDP §9)   |
| De-identification engine unavailable | Research-purpose retrievals queued; clinical reads continue |

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — IHE profile crosswalk (informative)

| IHE Radiology profile | Mapping to this PHASE                                |
|-----------------------|------------------------------------------------------|
| Scheduled Workflow.b  | UPS-RS work-list flow (PHASE 2 §6)                   |
| Cross-Enterprise XDS-I.b | Tele-radiology exchange (PHASE 4 §7)              |
| Imaging Object Change Mgmt | IOCM corrections (PHASE 4 §8)                   |
| Audit Trail and Node Auth | DICOM PS3.15 audit projection (PHASE 3 §6)       |
| Consistent Presentation | Hanging protocols + grayscale display calibration |

Where an IHE profile already covers a behaviour, this PHASE does
not redefine it; it specifies the privacy-gate overlay so that
implementations can hold both the IHE conformance and the WIA
conformance simultaneously.

## Annex B — TLS profile choices (informative)

| Concern             | Default                              | Legacy permitted |
|---------------------|--------------------------------------|------------------|
| TLS version         | 1.3 (RFC 8446)                       | 1.2 explicit-list|
| Cipher suite        | TLS_AES_256_GCM_SHA384, TLS_CHACHA20_POLY1305_SHA256 | as TLS 1.2 listed |
| Mutual auth         | X.509 client cert mandatory          | n/a              |
| Cert rotation       | every 365 days                       | every 730 days   |

Quantum-resistance is out of scope; a migration plan will be
added in a future minor version after NIST PQ standards stabilise
for healthcare HSM support.

## Annex C — DIMSE service classes (informative)

The boundary supports the following DIMSE-C service classes on
the upper-layer port:

| SOP Class UID                            | Service                              |
|------------------------------------------|--------------------------------------|
| 1.2.840.10008.1.1                        | Verification (C-ECHO)                |
| 1.2.840.10008.5.1.4.1.1.*                | Storage SOP classes (per modality)   |
| 1.2.840.10008.5.1.4.31                   | Modality Worklist Information Model  |
| 1.2.840.10008.5.1.4.1.2.1.1              | Patient Root Q/R — FIND              |
| 1.2.840.10008.5.1.4.1.2.2.1              | Study Root Q/R — FIND                |
| 1.2.840.10008.5.1.4.1.2.2.2              | Study Root Q/R — MOVE                |
| 1.2.840.10008.5.1.4.1.2.2.3              | Study Root Q/R — GET                 |

DIMSE-N services (N-CREATE, N-SET, N-EVENT-REPORT) are supported for
storage commitment, modality performed procedure step, and unified
procedure step. Each service emits the corresponding DICOM PS3.15
audit message; the boundary projects each into the FHIR AuditEvent
chain.

## Annex D — Network resilience

Modalities lose connectivity. The boundary tolerates this by:

- Buffering outbound C-MOVE jobs for retry
- Holding storage commitments open until acknowledged
- Surfacing a stable view of in-flight transfers to the imaging
  operations team

A modality that has been offline for longer than the deployment's
resilience window (typically 48 hours) is flagged for the operations
team. The boundary continues to accept new associations from that
modality but emits a structured incident record so that recovery
work has a starting point.

## Annex E — Vendor presentation-state interoperability (informative)

DICOM Presentation State (PS) IODs (Grayscale Softcopy PS, Color
Softcopy PS, Pseudocolour Softcopy PS, Blending Softcopy PS,
Advanced Blending PS) capture viewer-side rendering decisions.
Vendor differences:

- Some vendors persist PS instances under the original study; the
  boundary follows that convention by default
- Some vendors ship proprietary state alongside DICOM PS; the
  boundary refuses to forward proprietary state, which keeps the
  audit chain free of opaque vendor blobs
- Hanging Protocol IOD instances (1.2.840.10008.5.1.4.38.1) describe
  layout templates; the boundary preserves them as instances and
  does not interpret them

A viewer that needs vendor-specific state SHOULD persist that state
in a vendor-controlled side store and reference the DICOM source
study by UID, not embed proprietary state into the DICOM stream.

## Annex F — Multi-tenant boundary topology (informative)

Larger deployments host multiple controllers (multi-hospital health
systems). The boundary topology supports tenant isolation:

- Each tenant has its own DICOM root UID, JWKS, and audit chain
- Cross-tenant retrieval requires explicit tenant federation per
  the deployment policy (typically a lateral consent grant)
- A tenant that exits the federation has its data preserved in its
  own audit chain segment; no cross-tenant audit collapse is
  permitted

Multi-tenant deployments document the federation rules in the
deployment policy and audit federation events themselves.
