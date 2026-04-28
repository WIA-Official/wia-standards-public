# WIA-healthcare-integration PHASE 4 — INTEGRATION Specification

**Standard:** WIA-healthcare-integration
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how a healthcare operator
integrates with the systems that surround clinical
record exchange: the IHE Affinity Domain and Cross-
Community communities; the FHIR ecosystem (FHIR R5
servers, SMART-on-FHIR launch sequences, FHIR Bulk
Data export endpoints); the public-health authority
notifiable-disease and immunisation registries; the
payer claims-adjudication systems; the patient-portal
record-export surface (HIPAA 45 CFR 164.524 access
right and GDPR Article 15 right of access); the
external auditor and ISO/IEC 27001 / ISO 27799
certification body; the supervisory data-protection
authority for cross-border transfers; and — where
the EHDS infrastructure is used — the Member State's
MyHealth@EU National Contact Point.

References (CITATION-POLICY ALLOW only):

- HL7 FHIR R5 (RESTful API, SMART-on-FHIR launch
  sequence, Bulk Data export specification)
- IHE Affinity Domain governance practice
- IHE PCC / XDS / XCA cross-community profiles
- DICOMweb (DICOM PS3.18)
- IETF RFC 8259 / 9457 / 8615 / 8288 / 9421
- ISO/IEC 27001:2022 (information security management)
- ISO 27799:2016 (information security in health)
- ISO/IEC 17021-1:2015 (audit and certification)
- ISO/IEC 17065:2012 (conformity-assessment bodies)
- ISO 8601, ISO 13606
- W3C Verifiable Credentials Data Model 2.0 (optional
  for re-issuance of attestations)
- US HIPAA Privacy Rule (45 CFR Part 164, Subpart E)
- US HIPAA Security Rule (45 CFR Part 164, Subpart C)
- EU GDPR Articles 9, 15, 20, 33, 34, 46
- KR Medical Service Act and KR PIPA
- EU European Health Data Space Regulation

---

## §1 IHE Affinity Domain Integration

The operator publishes its participation in an IHE
Affinity Domain — the multi-organisation governance
agreement that establishes the document-sharing
community. The Affinity Domain agreement carries:

- The community's `homeCommunityId` ISO OID.
- The list of registered Document Source / Document
  Repository / Document Registry actors.
- The trust-anchor certificate authorities for the
  community's TLS endpoints.
- The supported document classes (LOINC document-
  type codes the community accepts).
- The patient-identity discipline (PIX V3 cross-
  reference manager identity).

Cross-community integration uses the IHE XCA gateway
pattern (Initiating Gateway at the operator's
boundary, Responding Gateway at the counterparty's
boundary); the trust-anchor sets are exchanged through
the operator's published `.well-known/wia/healthcare-
integration` discovery document.

## §2 SMART-on-FHIR App Integration

External applications integrate with the operator's
FHIR R5 endpoint through the SMART-on-FHIR launch
sequence:

- Standalone launch — the application redirects the
  practitioner to the operator's authorisation
  endpoint, the practitioner authenticates, and the
  application receives an OAuth 2.1 bearer token with
  the requested scopes.
- EHR launch — the operator's EHR launches the
  application with a launch context (patient, encounter)
  carried in the `launch` parameter, and the
  application receives the bearer token bound to that
  context.
- Backend services — the system-level token (no user
  in the loop) carries `system/` scopes for batch
  workloads.

The application's published manifest declares the
requested scopes and the capabilities the application
exercises (search, read, write, bulk-data export); the
operator's catalogue review approves the manifest
before the application is enabled.

## §3 FHIR Bulk Data Export Integration

The operator exposes the FHIR R5 Bulk Data export
operation (`$export`) for population-level workloads:

- Group-level export — a defined population (e.g. a
  payer's covered members for the operator's claims-
  adjudication workflow).
- Patient-level export — the patient's own record for
  a HIPAA 45 CFR 164.524 access-right or GDPR Article
  20 portability request.
- System-level export — the operator's full data set
  (only with explicit governance approval).

The export produces NDJSON files that follow the FHIR
R5 resource shape; the recipient's transfer-impact
assessment (cross-border) and consent-directive
discipline are evaluated before the export is started.

## §4 Public-Health Reporting Integration

The operator integrates with the jurisdiction's public-
health authority notifiable-disease, immunisation, and
syndromic-surveillance registries:

- Notifiable-disease reporting follows the
  jurisdiction's case-reporting wire format (FHIR R5
  electronic case reporting, HL7 v2.5.1 ELR, or the
  jurisdiction-specific equivalent).
- Immunisation reporting follows the FHIR R5
  `Immunization` resource shape mapped to the
  jurisdiction's registry interface.
- Syndromic-surveillance reporting follows the FHIR
  R5 `Encounter` and `Observation` shape with the
  jurisdiction's syndromic-surveillance value set.

Public-health reporting is exercised under the
HIPAA Privacy Rule 45 CFR 164.512(b) public-health
exception, the GDPR Article 9(2)(i) public-health
basis, or the KR PIPA Article 18 public-health
exception, depending on the operating jurisdiction.

## §5 Payer Claims Integration

Payer integration uses the FHIR R5 financial resources
(`Claim`, `ClaimResponse`, `ExplanationOfBenefit`,
`Coverage`, `EnrollmentRequest`) and the IHE PCC
encounter-summary / discharge-summary documents for
clinical evidence supporting claims. The payer's
business-associate-agreement (HIPAA) or the payer's
GDPR processor agreement (EU) carries the allowable
uses and disclosures for the integration.

## §6 Patient-Portal and Access-Right Integration

Patient access to the record is exercised through the
patient-portal:

- HIPAA Privacy Rule 45 CFR 164.524 grants the
  patient the right of access; the portal exposes
  the FHIR R5 `Patient/$everything` operation
  filtered by the patient's master-patient-index
  identifier.
- GDPR Article 15 grants the data subject the right
  of access; the portal exposes the same operation
  filtered by the data subject's identity.
- GDPR Article 20 grants the data subject the right
  of portability; the portal exposes the FHIR R5
  Bulk Data patient-level export.
- KR Medical Service Act Article 21 grants the
  patient the right of inspection; the portal
  exposes the inspection record alongside the FHIR
  Bundle export.

## §7 External Audit and Certification Integration

The operator's information security management
system (ISMS) is certified against ISO/IEC 27001:2022
and the ISMS scope explicitly extends to the FHIR /
IHE endpoints. Health-specific guidance follows ISO
27799:2016. The certification body operates under
ISO/IEC 17021-1; the conformity-assessment body for
WIA-healthcare-integration operates under ISO/IEC
17065. Surveillance audits run on the certification
body's published cadence.

## §8 Supervisory Data-Protection Authority Integration

For GDPR-regulated operators the supervisory data-
protection authority (the lead DPA for cross-border
processing per GDPR Article 56) is the integration
counterparty for breach notification (Article 33),
data-protection-impact-assessment prior consultation
(Article 36), and Article 9 special-category
processing review. The integration carries the
supervisory authority's identifier, the per-
jurisdiction notification-channel endpoint, and the
operator's response SLA.

## §9 EHDS MyHealth@EU Integration

For operators participating in the EHDS infrastructure
the Member State's MyHealth@EU National Contact Point
is the integration boundary for cross-border patient
summary, ePrescription, electronic dispensation,
laboratory results, hospital discharge reports, and
medical imaging. The operator's MyHealth@EU
integration carries the National Contact Point
endpoint, the patient-summary cross-border mapping,
and the EHDS regulation's secondary-use governance
references.

## §10 Long-Term Archival Integration

Records governed by jurisdictional retention horizons
(HIPAA 45 CFR 164.530(j) six years; KR Medical Service
Act ten years; KR PIPA Article 21 retention discipline;
GDPR Article 5(1)(e) no-longer-than-necessary
horizon) are migrated to the operator's long-term
archive at the close of the active retention window.
The archive preserves the FHIR Bundle, the IHE
DocumentReference metadata, the audit-event trail,
and — for ISO 13606-shaped systems — the archetype
versions in effect at the time of curation.

## §11 Conformance

Implementations claiming PHASE-4 conformance publish
their IHE Affinity Domain agreement, expose the
FHIR R5 SMART-on-FHIR launch and Bulk Data
endpoints declared in their `CapabilityStatement`,
operate the public-health-reporting integration that
their jurisdiction requires, expose the patient-
portal access-right surface, and maintain the
external-audit and supervisory-authority
integrations described above.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-healthcare-integration
- **Last Updated:** 2026-04-28
