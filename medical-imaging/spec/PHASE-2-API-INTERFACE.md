# WIA-medical-imaging PHASE 2 — API Interface Specification

**Standard:** WIA-medical-imaging
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable

This PHASE defines the API surface a medical-imaging deployment
exposes for retrieval, storage, query, and report exchange. The
shape is DICOMweb (PS3.18) for imaging operations and FHIR R5 for
non-image clinical context. Both surfaces share the privacy gate
defined in WIA-medical-data-privacy PHASE 2.

References (CITATION-POLICY ALLOW only):
- DICOM PS3.18 — Web Services (WADO-RS, QIDO-RS, STOW-RS, UPS-RS)
- HL7 FHIR R5 — RESTful API, ImagingStudy, DiagnosticReport, ImagingSelection
- IHE Radiology — Scheduled Workflow (SWF.b), Cross-Enterprise Document Sharing for Imaging (XDS-I.b), Imaging Object Change Management (IOCM)
- IETF RFC 9457 (Problem Details), RFC 7515 (JWS)

---

## §1 DICOMweb endpoints

The following PS3.18 services are mandatory:

| Service  | Purpose                                                           |
|----------|-------------------------------------------------------------------|
| QIDO-RS  | Query for Studies / Series / Instances (returns metadata)         |
| WADO-RS  | Retrieve Study / Series / Instance / Frame / Bulkdata / Metadata  |
| STOW-RS  | Store new instances                                               |
| UPS-RS   | Unified Procedure Step (work-list management)                     |

Endpoints follow the canonical PS3.18 paths:

```
/studies                                 # QIDO-RS, STOW-RS
/studies/{StudyInstanceUID}              # WADO-RS, QIDO-RS series
/studies/{StudyInstanceUID}/series/{SeriesInstanceUID}
/studies/{StudyInstanceUID}/series/{SeriesInstanceUID}/instances/{SOPInstanceUID}
/workitems                               # UPS-RS
```

## §2 Privacy headers

Every DICOMweb call carries the privacy headers from WIA-medical-data-
privacy PHASE 2:

```
WIA-Purpose-Of-Use: TREAT
WIA-Justification: pre-operative review, scheduled
```

The boundary translates these into a consent gate before forwarding
to the backing PACS/VNA. Calls without privacy headers are rejected
with `urn:wia:mdp:problem:purpose-mismatch`. The boundary's gate
decision is itself a DICOM AuditEvent (DICOM PS3.15 §A.5).

## §3 QIDO-RS query model

Queries return only the studies, series, and instances visible
to the caller's purpose. The boundary applies these filters:

- StudyInstanceUID lookup — returns the study only if the caller's
  consent includes the Patient on the study and the purpose is
  granted on that consent
- Patient-level query (`/studies?PatientID=...`) — the PatientID
  parameter MUST be a `subjectRef`; raw MRN is rejected at the
  boundary
- Date-range query — returns studies whose date falls inside the
  active consent's period

QIDO-RS responses are DICOM JSON (Annex F of PS3.18). The boundary
inserts a `0064,0007 ReferencedConsentSequence` field referencing
the consent UID that authorised the call (a non-standard tag held
in the deployment's private dictionary).

## §4 WADO-RS retrieval

Retrieve operations honour the consent gate at the per-instance
level. A study with mixed-consent (e.g., the patient consented to
release of imaging instances except those in a sensitive series)
returns only the consented subset. The response carries:

- An `In-Reply-To` header naming the consent UID
- A `Warning: 199 - "filtered by consent"` header if any instances
  were excluded
- For DICOMweb metadata responses, a `0064,0008 ConsentExclusionCount`
  tag indicating how many instances were filtered out (count only;
  no metadata)

Bulk data retrieval (multi-frame) is gated identically to
per-instance retrieval; partial frame retrieval requires that the
entire instance pass the consent gate (no per-frame consent).

## §5 STOW-RS storage

Storing new instances requires:

- `WIA-Purpose-Of-Use: TREAT` or `HOPERAT` (research storage uses a
  different consent class and is gated separately)
- The instance's PatientID matches a known subject in the identity
  broker — otherwise the boundary creates a new pseudonymous
  subjectRef and emits a Provenance recording the binding
- Instance UIDs are unique against the global UID registry; collision
  is rejected with `urn:wia:mdp:problem:uid-collision`

Successful STOW returns `200 OK` with a DICOM XML response listing
the stored instances and any rejections. Each accepted instance
emits a DICOM AuditEvent and a FHIR AuditEvent referencing the same
chain.

## §6 UPS-RS work-list

The Unified Procedure Step service exposes scheduled imaging work
for modalities to claim. Privacy treatment:

- Work-items reference the patient by `subjectRef`, never MRN
- The modality device authenticates via a SMART system token and
  carries `purpose:TREAT`
- A claimed work-item transitions through the IHE SWF.b state
  machine (SCHEDULED → IN PROGRESS → COMPLETED / CANCELED)
- Each transition emits an AuditEvent

## §7 Report exchange

Structured reports flow through both DICOMweb (as DICOM SR
instances) and FHIR (as DiagnosticReport resources). The boundary
mirrors changes in either direction:

- A DICOM SR stored via STOW-RS produces a FHIR DiagnosticReport
  via the boundary's projection
- A FHIR DiagnosticReport created via the FHIR API produces a
  DICOM SR via reverse projection (only when the deployment policy
  enables FHIR-first authoring)

The mirrored resource carries a Provenance reference to the source
so that auditors can see which is canonical.

## §8 Image-on-the-fly transformation

WADO-RS supports rendering parameters (`/rendered` paths). The
boundary applies the consent gate before rendering; rendered
output inherits the source's consent UID. The boundary refuses to
render content the caller could not retrieve raw.

## §9 Errors and warnings

| URI                                              | Status | Meaning                                |
|--------------------------------------------------|-------:|----------------------------------------|
| `urn:wia:mi:problem:study-not-found`             | 404    | StudyInstanceUID unknown or unconsented|
| `urn:wia:mi:problem:transfer-syntax-not-supported` | 415  | Requested syntax not in the accept list|
| `urn:wia:mi:problem:uid-collision`               | 409    | Instance UID already in use            |
| `urn:wia:mi:problem:lossy-rejected`              | 422    | Lossy compression on diagnostic intent |
| `urn:wia:mi:problem:work-item-claimed`           | 409    | UPS work-item claimed by another modality |

Warnings (200-OK responses with consent-driven filtering applied)
use `Warning:` headers per RFC 7234 §5.5 with the deployment's
warning codes namespaced under `wia-mi-`.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Worked DICOMweb exchange (informative)

A typical viewer retrieving a study:

```
GET /studies/1.2.840.113619.2.55.3.604688.123/series HTTP/1.1
Host: dicomweb.hospital-k.example
Authorization: Bearer eyJhbGciOiJFUzI1NiIsImtpZCI6ImsxIn0...
WIA-Purpose-Of-Use: TREAT
WIA-Justification: scheduled outpatient review
Accept: application/dicom+json
```

The boundary validates the token, checks that the patient bound
to the StudyInstanceUID has an active TREAT consent for the
calling clinician, and forwards to the backing PACS. The response
DICOM JSON includes the consent reference so downstream auditors
can replay the gate decision deterministically.

## Annex B — Capability advertisement (informative)

The DICOMweb capability is advertised at `/dicomweb/conformance`
in line with PS3.18 §6. The advertised conformance is augmented
with WIA-specific extensions naming the purpose vocabulary,
the consent profile, and the supported transfer syntaxes for
STOW-RS.

## Annex C — Long-running operations (informative)

Bulk operations (large multi-series exports, AI inference batch
jobs) follow the WADO-RS asynchronous pattern: the request is
accepted with `202 Accepted` and a status URI; the client polls
the status URI until completion. The boundary emits AuditEvents
both at request acceptance and at result retrieval so that the
gap between the two is observable.

## Annex D — Long-poll and subscription model (informative)

For workflow-driven systems (RIS-PACS-AI pipelines), DICOMweb is
extended with the FHIR Subscriptions framework. A deployment MAY
expose:

- `POST /Subscription` — register a subscription on
  `ImagingStudy.create` or `ImagingStudy.update`
- `GET /$ws-binding` — WebSocket binding for push notifications
  (per the FHIR R5 SubscriptionTopic / R5/subscription.html)

Subscriptions inherit the privacy gate: the subscriber receives
only events for studies the subscriber's purpose grant covers.
Notifications carry the `WIA-Audit-Event-Id` header binding the
notification to its underlying audit event so auditors can
trace pushed work.

## Annex E — Bandwidth and compression policy (informative)

Bulk operations are bandwidth-sensitive. Deployments use
HTTP/2 multiplexing and chunked transfer encoding for large
WADO-RS responses. Trans-coding to a smaller-but-lossless syntax
(e.g., JPEG 2000 Lossless) at the boundary trades CPU for
bandwidth. The trade-off is recorded in the deployment policy
so that auditors can confirm pixel content is preserved across
trans-codes.

## Annex F — Imaging worked search (informative)

A typical patient-scoped search for a clinician:

```
GET /studies?PatientID=urn:wia:mdp:subject:f4c2-9bd1-7a05-3e8e
            &ModalitiesInStudy=CT,MR
            &StudyDate=20260101-20260427
            &includefield=00080050
            &includefield=00080090
HTTP/1.1
Host: dicomweb.hospital-k.example
Authorization: Bearer eyJhbGciOiJFUzI1NiIsImtpZCI6ImsxIn0...
WIA-Purpose-Of-Use: TREAT
WIA-Justification: encounter prep, scheduled
Accept: application/dicom+json
```

The boundary returns DICOM JSON describing the matching studies,
filtered to those for which the requesting clinician's consent
grant covers TREAT on this patient. Studies in the date range that
the clinician cannot access are omitted entirely (no count is
returned beyond the visible subset). The response carries a
`WIA-Audit-Event-Id` and `WIA-Consent-Receipt` header set so that
the search itself is replay-able.

## Annex G — Pagination and large result sets (informative)

QIDO-RS supports `limit` and `offset` parameters. The boundary
caps `limit` at 5000 results per call; larger result sets paginate.
Each page emits its own AuditEvent so that the audit chain reflects
every view, not only the first. The boundary sets the response's
`Warning: 200` header when the total count exceeds the cap so that
clients know to paginate.
