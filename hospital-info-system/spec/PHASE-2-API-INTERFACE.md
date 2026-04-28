# WIA-hospital-info-system PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-hospital-info-system
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the API contract that a hospital
information system (HIS) exposes for the records defined
in PHASE-1. Two complementary surfaces are defined: the
HL7 v2.x messaging surface — the operating substrate for
most in-hospital subsystem boundaries — and the HL7
FHIR R5 RESTful surface, which is the modern facade for
EMR access, patient-portal access, and external-partner
integration.

References (CITATION-POLICY ALLOW only):

- HL7 v2.5.1 / v2.8.2 (the operating-site messaging
  substrate; the Minimal Lower Layer Protocol over
  TCP is the wire format for inbound and outbound v2
  messaging)
- HL7 FHIR Release 5 RESTful API
- IHE LAB transactions LTW (laboratory testing
  workflow), LCSD (laboratory code-set distribution),
  LBL (laboratory barcode labelling), LPOCT (point-of-
  care testing)
- IHE PCC profiles for clinical-document content
- DICOM PS3.4 (service classes — query/retrieve,
  storage, MWL, MPPS) and DICOM PS3.18 DICOMweb (QIDO-
  RS, WADO-RS, STOW-RS) for the radiology subsystem
- IETF RFC 9110 (HTTP Semantics), RFC 9111 (HTTP
  Caching), RFC 9457 (Problem Details), RFC 6901 /
  6902 (JSON Pointer / Patch), RFC 8288 (Web Linking),
  RFC 8259 (JSON), RFC 9421 (HTTP Message Signatures)
- ISO 8601, ISO/IEC 27001:2022, ISO 27799:2016
- W3C Trace Context
- US HIPAA Security Rule (45 CFR Part 164, Subpart C)
  technical-safeguards clauses
- KR Medical Service Act Article 23 electronic-medical-
  record curation discipline

---

## §1 Scope and Versioning

The HIS exposes:

- HL7 v2.x messaging endpoints (TCP/IP / MLLP) for the
  in-hospital subsystem boundaries (ADT ↔ LIS, ADT ↔
  RIS, ADT ↔ pharmacy, CPOE ↔ pharmacy / LIS / RIS,
  pharmacy ↔ eMAR, billing ↔ EMR).
- HL7 FHIR R5 RESTful endpoints (HTTPS) for the EMR
  facade, patient-portal, and external-partner
  integration.
- DICOM endpoints for the radiology subsystem.
- HIS-specific WIA endpoints under `/v1/` for
  programme, charge, and audit access.

The FHIR `CapabilityStatement` at `/fhir/r5/metadata`
is canonical for the FHIR resources; the OpenAPI 3.1
document at `/v1/openapi.json` is canonical for the
WIA endpoints; the v2 conformance profile at
`/messaging/conformance.xml` is canonical for the v2
messaging interfaces.

## §2 HL7 v2 Messaging Surface

The v2 endpoints accept:

- ADT (admit-discharge-transfer) — `A01` admit,
  `A02` transfer, `A03` discharge, `A04` register,
  `A05` pre-admit, `A08` update, `A11` cancel admit,
  `A12` cancel transfer, `A13` cancel discharge.
- ORM (orders) — `O01` general order message; the
  hospital's CPOE → pharmacy / LIS / RIS routing.
- ORU (observations) — `R01` unsolicited
  observation message; the LIS → CPOE / EMR
  laboratory-result feed.
- DFT (charge transactions) — `P03` post-detail-
  financial-transaction; the hospital's billing-
  to-EMR feed.
- BAR (billing account record) — `P01` add patient
  account, `P02` purge.
- MFN (master file notification) — `M02` staff /
  practitioner master, `M04` charge description
  master, `M08` test / observation master, `M09`
  test / observation batteries master.
- SIU (scheduling information unsolicited) — `S12`
  add appointment, `S13` reschedule, `S15` cancel.

Each message is acknowledged with an `ACK` (or `MFK`
for master-file messages); the hospital's interface
engine routes between the subsystems and persists each
message to the integration-layer audit log.

## §3 FHIR R5 RESTful Surface

```
GET  /fhir/r5/metadata
GET  /fhir/r5/Patient/{patientId}
GET  /fhir/r5/Patient?identifier=urn:oid:1.2.3|MRN-123
GET  /fhir/r5/Encounter?subject=Patient/{patientId}
POST /fhir/r5/Encounter
GET  /fhir/r5/Observation?subject=Patient/{patientId}&code=http://loinc.org|2160-0
GET  /fhir/r5/MedicationRequest?subject=Patient/{patientId}
POST /fhir/r5/MedicationRequest
GET  /fhir/r5/MedicationAdministration?subject=Patient/{patientId}
GET  /fhir/r5/DiagnosticReport?subject=Patient/{patientId}
GET  /fhir/r5/ImagingStudy?subject=Patient/{patientId}
POST /fhir/r5/Consent
GET  /fhir/r5/AuditEvent?patient=Patient/{patientId}
GET  /fhir/r5/Patient/{patientId}/$everything
```

The FHIR `CapabilityStatement` declares the supported
search parameters per resource and the supported
interactions (read, vread, search, history, create,
update, patch, delete) per resource.

## §4 DICOM Endpoints

```
DICOM C-FIND-RQ     — query the modality worklist
DICOM C-MOVE-RQ     — retrieve a study from the PACS
DICOM C-STORE-RQ    — store a study to the PACS
DICOM N-CREATE-RQ   — modality performed procedure step
DICOM N-SET-RQ      — modality performed procedure step
                      update
```

DICOMweb endpoints are the modern wire format:

```
GET  /dicomweb/studies?PatientID={mrn}            (QIDO-RS)
GET  /dicomweb/studies/{study-uid}/series         (QIDO-RS)
GET  /dicomweb/studies/{study-uid}/series/{series-uid}/instances/{instance-uid}/frames/{frame}  (WADO-RS)
POST /dicomweb/studies                            (STOW-RS)
```

## §5 WIA Programme and Audit Endpoints

```
GET  /v1/
GET  /v1/programmes
GET  /v1/programmes/{programmeId}
GET  /v1/charge-records?encounter={encounterId}
GET  /v1/audit-events?patient={patientId}&from={iso}&to={iso}
GET  /v1/audit-events/{eventId}
```

The audit-event endpoint exposes the FHIR R5
`AuditEvent` resource for compliance-and-audit roles;
the patient sees only their own audit trail through
the patient-portal `/$everything` operation, filtered
by the patient identity bound to the bearer token.

## §6 Patient-Portal Endpoints

```
GET  /portal/v1/me
GET  /portal/v1/me/encounters
GET  /portal/v1/me/observations
GET  /portal/v1/me/medications
GET  /portal/v1/me/imaging-studies
GET  /portal/v1/me/consents
POST /portal/v1/me/consents
PATCH /portal/v1/me/consents/{consentId}    (withdraw)
```

The patient's identity is bound to the bearer token
through the hospital's IdP; the portal surface enforces
the patient-only scope on every request.

## §7 Authentication and Authorisation

Bearer tokens conform to OAuth 2.1; the FHIR R5
SMART-on-FHIR profile is the recommended baseline for
external applications. Internal subsystem-to-subsystem
calls use mutual TLS (mTLS) with the hospital's
internal certificate authority. The token's audience
is the FHIR base URL or the WIA `/v1/` base URL;
scopes follow the SMART-on-FHIR `patient/`, `user/`,
and `system/` syntax. Break-the-glass access uses an
additional scope claim and triggers the audit event of
type `break-the-glass`.

## §8 HTTP Status Codes

- `200 OK` — read or search success
- `201 Created` — create success (Location header
  carries the new resource URL)
- `204 No Content` — delete success
- `400 Bad Request` — malformed FHIR payload (FHIR
  `OperationOutcome` body)
- `401 Unauthorized` — missing or invalid bearer token
- `403 Forbidden` — access-discipline rejection
- `404 Not Found` — resource not registered
- `409 Conflict` — version-mismatch on update (FHIR
  optimistic-locking via `If-Match`)
- `412 Precondition Failed` — `If-None-Match` mismatch
- `422 Unprocessable Content` — FHIR validation
  failure with `OperationOutcome` issue details
- `429 Too Many Requests` — rate-limit exceeded
- `503 Service Unavailable` — downstream subsystem
  unavailable

## §9 Caching and Conditional Requests

`ETag` carries the FHIR `Resource.meta.versionId`. The
client uses `If-None-Match` for conditional reads and
`If-Match` for conditional updates. PHI-bearing
responses use `Cache-Control: private, max-age=0,
must-revalidate`; HIPAA Privacy Rule 45 CFR 164.502
minimum-necessary applies.

## §10 Content Negotiation and Encoding

The FHIR endpoints accept and produce
`application/fhir+json` (the FHIR R5 default) and
`application/fhir+xml` for clients that require the
XML serialisation. The DICOMweb endpoints follow the
PS3.18 content negotiation discipline:
`application/dicom+json` for QIDO-RS responses,
`multipart/related` with the appropriate sub-type for
WADO-RS instance retrieval, and the DICOM PS3.10 file
format for STOW-RS upload. The HL7 v2.x messaging
surface uses the v2.5.1 or v2.8.2 ER7 encoding (vertical
bar / caret / tilde / backslash / ampersand delimiters)
over the Minimal Lower Layer Protocol.

## §11 Rate Limiting and Throttling

Each subsystem-to-subsystem v2 messaging interface has
a per-second message-rate budget declared in the v2
conformance profile; exceeding the budget produces
`AR` (application reject) acknowledgements with the
overload reason in MSA-3 and triggers the operating
site's flow-control discipline. The FHIR endpoints
publish a per-client `RateLimit` header (RFC 9110)
declaring the remaining budget and the reset window;
exceeding the budget produces `429 Too Many Requests`.
The DICOM endpoints' worklist-query and study-
retrieve operations are limited per the operating
site's PACS capacity declaration.

## §12 Webhook and Subscription Surface

The FHIR R5 `Subscription` resource and the FHIR
`SubscriptionTopic` resource publish the hospital's
event surface (encounter-status change, result
finalisation, admission-discharge-transfer) to
external applications that subscribe under the
SMART-on-FHIR backend-services pattern. The
`Subscription.channel` carries the webhook endpoint
URL and the bearer-token reference; the hospital's
event router fans subscribed events to the registered
endpoints and re-tries on transient failures per the
declared retry budget.

## §13 Bulk-Data Export Surface

The FHIR R5 Bulk Data export operation is exposed at:

```
GET /fhir/r5/Group/{groupId}/$export
GET /fhir/r5/Patient/$export
GET /fhir/r5/$export
```

Group-level export targets a defined population
(payer-covered members, research-cohort enrolment,
public-health surveillance cohort). Patient-level
export targets the patient's record for a HIPAA
45 CFR 164.524 access-right or GDPR Article 20
portability response. System-level export targets the
hospital's full data set and is available only with
the explicit governance approval recorded in the
operating site's bulk-data-governance register.
Export status is tracked through a `Content-Location`
URL polled by the requesting client; on completion
the manifest URL lists the NDJSON resource files
produced.

## §14 Trace-Context and Correlation Discipline

Every FHIR / WIA request and v2 message carries a
trace-context identifier (W3C Trace Context
`traceparent`) propagated end-to-end through the
hospital's interface engine, FHIR server, ancillary
subsystems, and audit log. Correlated identifiers
allow the operating site to reconstruct the order /
result / administration chain across the subsystems
that participated in the patient's encounter.

## §15 Conformance

Implementations claiming PHASE-2 conformance publish
the v2 conformance profile at
`/messaging/conformance.xml`, the FHIR
`CapabilityStatement` at `/fhir/r5/metadata`, the
DICOM SCP / SCU declarations the operating site
exposes, and the OpenAPI document for the WIA
endpoints. The portal surface enforces the patient-
only scope on every request; the v2 messaging surface
acknowledges every received message; the FHIR surface
emits the `AuditEvent` for every action; the bulk-
data export surface is governed by the operating
site's bulk-data-governance register; trace-context
identifiers are propagated end to end.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-hospital-info-system
- **Last Updated:** 2026-04-28
