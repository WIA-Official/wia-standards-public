# WIA-healthcare-integration PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-healthcare-integration
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTPS API contract that a
healthcare operator (provider, payer, HIE, public-health
authority, or intermediary) exposes for the records
defined in PHASE-1. The contract is layered on the HL7
FHIR R5 RESTful API and the IHE XDS.b / XCA SOAP
profiles where the operator participates in document-
sharing communities.

References (CITATION-POLICY ALLOW only):

- HL7 FHIR R5 RESTful API (the FHIR HTTP RESTful
  interaction spec; cited normatively for the search,
  read, create, update, and history interactions)
- IHE XDS.b (Cross-Enterprise Document Sharing) and
  IHE XCA (Cross-Community Access) transactions ITI-
  41 / ITI-43 / ITI-38 / ITI-39
- IHE PIX / PDQ (patient identifier cross-reference and
  patient demographics query)
- DICOMweb (QIDO-RS, WADO-RS, STOW-RS) for imaging-
  study integration
- IETF RFC 9110 (HTTP Semantics), RFC 9111 (HTTP
  Caching), RFC 9457 (Problem Details), RFC 6901 /
  6902 (JSON Pointer / Patch), RFC 8288 (Web Linking),
  RFC 8259 (JSON), RFC 9421 (HTTP Message Signatures)
- ISO 8601, ISO/IEC 27001:2022, ISO 27799:2016
- W3C Trace Context
- US HIPAA Security Rule (45 CFR Part 164, Subpart C)
  technical-safeguards clauses for transmission
  security
- KR Medical Service Act electronic medical record
  provisions

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published by the
operator. The FHIR RESTful surface uses the FHIR R5
base path (`/fhir/r5/`) for FHIR-resource endpoints and
the WIA convention path (`/v1/`) for the WIA-specific
programme, audit, and cross-border transfer endpoints.
The OpenAPI 3.1 document at `/v1/openapi.json` is
canonical for the WIA endpoints; the FHIR
`CapabilityStatement` resource at `/fhir/r5/metadata`
is canonical for the FHIR endpoints.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-healthcare-integration",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "programmes":         "/v1/programmes",
    "consentDirectives":  "/v1/consent-directives",
    "documentReferences": "/v1/document-references",
    "auditEvents":        "/v1/audit-events",
    "crossBorderTransfers": "/v1/cross-border-transfers",
    "fhirBase":           "/fhir/r5/",
    "fhirCapability":     "/fhir/r5/metadata",
    "openapi":            "/v1/openapi.json"
  }
}
```

The FHIR `CapabilityStatement` returned by `/fhir/r5/
metadata` declares the FHIR resources the operator
supports (Patient, Encounter, Observation,
DiagnosticReport, MedicationRequest, MedicationAdministration,
Immunization, Condition, AllergyIntolerance, Consent,
DocumentReference, AuditEvent), the supported search
parameters per resource, and the supported interactions
(read, vread, search, history, create, update, patch,
delete) per resource.

## §3 Patient Search and Read (FHIR R5)

```
GET /fhir/r5/Patient?identifier=urn:oid:1.2.840.114350|MRN-12345
GET /fhir/r5/Patient/{patientId}
```

Search is constrained to the operator's master patient
index. Cross-community patient identifier resolution
goes through the IHE PIX V3 ITI-45 transaction (or its
FHIR R5 `Patient/$match` operation equivalent) so that
a patient identified at one community can be located
at another. Responses follow the FHIR R5 Bundle
semantics with `searchset` mode.

## §4 Encounter and Observation Endpoints

```
GET /fhir/r5/Encounter?subject=Patient/{patientId}
GET /fhir/r5/Observation?subject=Patient/{patientId}&code=http://loinc.org|2160-0
```

Search responses include the `_include` parameter for
related resources (subject, performer) and the `_revinclude`
parameter for resources referencing the result set.
Observations are coded with LOINC (laboratory and most
vital-signs observations) or SNOMED CT (coded clinical
findings); ICD-11 cross-walk is applied where the
observation represents a diagnostic conclusion.

## §5 Document Sharing (IHE XDS / XCA + FHIR
       DocumentReference)

The IHE XDS.b transactions are layered over SOAP /
ebRIM:

```
POST /xds/registry/iti41   (Provide and Register Document Set)
POST /xds/registry/iti43   (Retrieve Document Set)
POST /xds/registry/iti18   (Registry Stored Query)
POST /xca/initiating/iti38 (Cross Gateway Query)
POST /xca/initiating/iti39 (Cross Gateway Retrieve)
```

The FHIR R5 equivalents are:

```
POST /fhir/r5/DocumentReference   (create — equivalent to ITI-41)
GET  /fhir/r5/DocumentReference?subject=Patient/{patientId}
GET  /fhir/r5/Binary/{binaryId}   (retrieve — equivalent to ITI-43)
```

Operators publishing documents to an IHE XDS.b
registry MAY also expose the FHIR `DocumentReference`
endpoint as a parallel surface; clients query whichever
is most convenient.

## §6 Imaging-Study Integration (DICOMweb)

```
GET /dicomweb/studies?PatientID={mrn}            (QIDO-RS)
GET /dicomweb/studies/{study-uid}/series         (QIDO-RS)
GET /dicomweb/studies/{study-uid}/series/{series-uid}/instances/{instance-uid}/frames/{frame}  (WADO-RS)
POST /dicomweb/studies                            (STOW-RS)
```

The DICOMweb endpoints carry the DICOM Study Instance
UID and the DICOM SOP Instance UID; FHIR
`ImagingStudy` and `DiagnosticReport` resources cross-
reference these UIDs via the `series.uid` /
`identifier` fields.

## §7 Consent Directive Endpoints

```
GET    /v1/consent-directives?patient={patientId}
GET    /v1/consent-directives/{directiveId}
POST   /v1/consent-directives
PATCH  /v1/consent-directives/{directiveId}     (withdrawal)
```

The FHIR R5 `Consent` resource is the canonical wire
format for the directive body. Withdrawal is captured
through a PATCH that sets `withdrawalAt` and
`withdrawalChannel`; the operator's PHASE-3 §4 access-
discipline retires the directive's `provision` from
the active access matrix on the next access decision.

## §8 Audit-Event Endpoints

```
GET /v1/audit-events?subject={patientId}&from={iso}&to={iso}
GET /v1/audit-events/{eventId}
```

The FHIR R5 `AuditEvent` resource is the canonical
wire format. The `agent`, `entity`, `outcome`, and
`recorded` fields are populated for every query,
retrieve, create, update, delete, register-document,
consent-capture, consent-withdrawal, and break-the-
glass event. The operator MUST NOT expose audit events
beyond the patient's own access scope or the operator's
authorised compliance-and-audit role; HIPAA Security
Rule audit-controls (45 CFR 164.312(b)) apply.

## §9 Cross-Border Transfer Endpoints

```
GET    /v1/cross-border-transfers?patient={patientId}
POST   /v1/cross-border-transfers
PATCH  /v1/cross-border-transfers/{transferId}    (close)
```

Cross-border transfers carry the destination ISO 3166-1
country code, the transfer mechanism (PHASE-1 §11
enumeration), and the transfer-impact assessment
reference. The operator's GDPR Article 9 / HIPAA
business-associate-agreement / KR-PIPA Article 17
discipline (PHASE-3 §6) is consulted before the
transfer is recorded; the API rejects transfers that
do not satisfy the discipline with a 422 problem
document carrying the pre-condition cross-reference.

## §10 Search Conventions and Pagination

Search responses are FHIR Bundles (`type:searchset`)
with `link.relation` of `self` / `next` / `previous`
per FHIR R5. Page size defaults to 20 entries; the
operator's `Capability­Statement` declares the maximum
page size. Time-ranged searches use the FHIR `_lastUpdated`
and `_since` parameters.

## §11 HTTP Status Codes

- `200 OK` — read or search success
- `201 Created` — create success (Location header
  carries the new resource URL)
- `204 No Content` — delete success
- `400 Bad Request` — malformed FHIR payload (FHIR
  `OperationOutcome` body)
- `401 Unauthorized` — missing or invalid bearer token
- `403 Forbidden` — access-discipline rejection (the
  `OperationOutcome` references the consent-directive
  or break-the-glass discipline that rejected access)
- `404 Not Found` — resource not registered with the
  operator's index
- `409 Conflict` — version-mismatch on update (FHIR
  optimistic-locking via `If-Match`)
- `412 Precondition Failed` — `If-None-Match` mismatch
- `422 Unprocessable Content` — FHIR validation
  failure with `OperationOutcome` issue details
- `429 Too Many Requests` — rate-limit exceeded
- `503 Service Unavailable` — operator's downstream
  registry / repository unavailable

## §12 Authentication and Authorisation

Bearer tokens conform to OAuth 2.1 (the FHIR R5
SMART-on-FHIR profile is the recommended baseline).
The token's audience is the operator's FHIR base URL
or the operator's WIA `/v1/` base URL. The token
carries scopes that follow the SMART-on-FHIR
`patient/`, `user/`, and `system/` scope syntax;
break-the-glass access uses an additional scope claim
(`launch/break-the-glass`) and triggers the audit-
event of type `break-the-glass`. The operator's IdP
issues the token after authenticating the practitioner
(or the patient through patient-portal
authentication).

## §13 Caching and Conditional Requests

`ETag` carries the FHIR `Resource.meta.versionId`. The
client uses `If-None-Match` for conditional reads and
`If-Match` for conditional updates so that lost-update
hazards are detected at the wire layer. Search-result
freshness is governed by the operator's `Cache-Control:
private, max-age=0, must-revalidate` policy for all
PHI-bearing responses; HIPAA Privacy Rule 45 CFR
164.502 minimum-necessary applies to caching as it
applies to any other disclosure.

## §14 Conformance

Implementations claiming PHASE-2 conformance publish
the FHIR `CapabilityStatement` declaring the resources
listed above, expose the IHE XDS / XCA endpoints
declared in their published Affinity Domain, emit the
FHIR `AuditEvent` for every query / retrieve /
modification, and reject cross-border transfers that
do not satisfy the PHASE-3 §6 discipline.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-healthcare-integration
- **Last Updated:** 2026-04-28
