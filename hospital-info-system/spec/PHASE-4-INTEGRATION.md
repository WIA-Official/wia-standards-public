# WIA-hospital-info-system PHASE 4 — INTEGRATION Specification

**Standard:** WIA-hospital-info-system
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how a hospital information system
integrates with the systems that surround the in-
hospital record substrate: the regional health-
information exchange (HIE) for cross-organisation
exchange; the public-health authority for notifiable-
disease, immunisation, and syndromic-surveillance
reporting; the payer for claims adjudication; the
patient-portal for the patient's right of access; the
external auditor and ISO/IEC 27001 / ISO 27799
certification body; the EMR-certification body that
operates the operating jurisdiction's certification
programme; the supervisory data-protection authority;
and the long-term archive that preserves records past
the retention horizon.

References (CITATION-POLICY ALLOW only):

- HL7 v2.5.1 / v2.8.2 messaging
- HL7 FHIR R5 (RESTful API, SMART-on-FHIR launch
  sequence, Bulk Data export specification)
- IHE PCC / XDS / XCA cross-community profiles for
  cross-organisation exchange
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
- KR Medical Service Act, KR PIPA, and the KR EMR
  certification programme
- EU European Health Data Space Regulation

---

## §1 Health-Information-Exchange Integration

The hospital integrates with its regional health-
information exchange through the IHE XDS.b actor
pattern (Document Source / Document Repository /
Document Registry) at the hospital boundary. The
hospital publishes the IHE PCC encounter-summary or
discharge-summary documents to the regional registry
on the operating cadence; the HIE consumers (other
hospitals, primary-care practices) retrieve the
documents through the registry stored-query (ITI-18)
and document-retrieve (ITI-43) transactions. Cross-
community access uses the IHE XCA Initiating /
Responding Gateway pattern at the HIE boundary.

## §2 SMART-on-FHIR App Catalogue Integration

External applications integrate with the hospital's
EMR through the SMART-on-FHIR launch sequence:

- Standalone launch — clinician redirected to the
  hospital's authorisation server, returns with a
  bearer token.
- EHR launch — the EMR launches the application with
  patient + encounter context.
- Backend services — system-level token for batch
  workloads.

The hospital's app catalogue reviews each
application's published manifest (requested scopes,
exercised capabilities) before enabling it.

## §3 Public-Health Reporting Integration

The hospital integrates with the jurisdiction's
public-health authority for:

- Notifiable-disease reporting (electronic case
  reporting via FHIR R5 or the jurisdiction-specific
  format).
- Immunisation reporting (FHIR `Immunization` mapped
  to the registry interface).
- Syndromic-surveillance reporting (FHIR `Encounter`
  + `Observation` shape).
- Mortality reporting (the jurisdictional vital-
  records interface).
- Cancer-registry reporting (the jurisdictional
  cancer-registry interface).

The reporting basis is HIPAA 45 CFR 164.512(b)
public-health, GDPR Article 9(2)(i) public-health,
or KR PIPA Article 18 public-health, depending on
the operating jurisdiction.

## §4 Payer Integration

Payer integration uses:

- The HL7 v2 DFT message for charge transactions
  posted to the payer's claim-intake interface.
- The FHIR R5 financial resources (`Claim`,
  `ClaimResponse`, `ExplanationOfBenefit`,
  `Coverage`) for the modern claims surface.
- The IHE PCC encounter-summary / discharge-summary
  documents for clinical evidence supporting claims.

The payer's business-associate-agreement (HIPAA) or
the payer's GDPR processor agreement (EU) carries the
allowable uses and disclosures.

## §5 Patient-Portal Integration

The patient-portal exposes:

- HIPAA 45 CFR 164.524 access right — the FHIR
  `Patient/$everything` operation filtered by the
  patient's identity.
- HIPAA 45 CFR 164.526 amendment right — the
  patient's amendment request workflow.
- HIPAA 45 CFR 164.528 accounting of disclosures —
  the audit-log surface filtered by the patient's
  identity.
- GDPR Article 15 right of access, Article 20 right
  of portability — the same `$everything` and
  Bulk Data patient-level export, plus the formats
  GDPR requires.
- KR Medical Service Act Article 21 right of
  inspection — the inspection-record surface.

## §6 EMR Certification Integration

The KR EMR certification programme (전자의무기록 시스템
인증, KR Medical Service Act Article 23-2) audits the
hospital's electronic medical-record system against
the certification scheme; the certification record
references the hospital's compliance with the curation
discipline, the audit discipline, and the access-
control discipline. Hospitals operating in
jurisdictions without an equivalent national
certification programme adopt ISO/IEC 27001 + ISO
27799 certification as the baseline.

## §7 ISMS Certification and External Audit

The hospital's information security management system
(ISMS) is certified against ISO/IEC 27001:2022 with
the scope explicitly extending to the HL7 v2 / FHIR /
DICOM / WIA endpoints; ISO 27799:2016 sector guidance
applies. The certification body operates under
ISO/IEC 17021-1; the conformity-assessment body for
WIA-hospital-info-system operates under ISO/IEC
17065. Surveillance audits run on the certification
body's published cadence.

## §8 Supervisory Data-Protection Authority Integration

For GDPR-regulated hospitals the supervisory data-
protection authority (the lead DPA per GDPR Article
56) is the integration counterparty for breach
notification (Article 33), data-protection-impact-
assessment prior consultation (Article 36), and
Article 9 special-category processing review. For
KR-regulated hospitals the Personal Information
Protection Commission (PIPC) is the integration
counterparty.

## §9 EHDS MyHealth@EU Integration

For hospitals participating in the EHDS infrastructure
the Member State's MyHealth@EU National Contact Point
is the integration boundary for the cross-border
patient summary, ePrescription, electronic
dispensation, laboratory results, hospital discharge
reports, and medical imaging. The hospital's
MyHealth@EU integration carries the National Contact
Point endpoint and the patient-summary cross-border
mapping.

## §10 Long-Term Archival Integration

Records governed by the hospital's retention horizons
(PHASE-3 §9) are migrated to the long-term archive at
the close of the active retention window. The archive
preserves:

- The HL7 v2 message stream (for the integration-
  layer audit).
- The FHIR Bundle representation of the longitudinal
  record.
- The DICOM study set (for imaging).
- The chargemaster snapshot in effect at the time of
  service.
- The audit-event trail.

## §11 Cross-Border Patient-Movement Discipline

Hospitals that receive patients from outside the
operating jurisdiction (cross-border tourism, foreign-
worker care, international referral) integrate with
the patient's home-jurisdiction record holder through
either the EHDS MyHealth@EU NCP (for EU-resident
patients), the IHE XCA cross-community gateway (for
patients whose home record is held in a community
participating in IHE), or a bilateral data-sharing
agreement (for jurisdictions outside both). The
hospital's encounter record carries the home-
jurisdiction reference so that discharge documentation
can be exchanged back to the patient's home record
holder.

## §12 Disaster-Recovery and Business-Continuity Integration

The hospital's disaster-recovery (DR) and business-
continuity (BCP) discipline runs alongside the live
HIS:

- The DR site receives synchronous or asynchronous
  replication of the master files, the encounter
  records, the order and result records, the eMAR
  records, and the audit log.
- The BCP discipline declares the recovery-time
  objective (RTO) and recovery-point objective (RPO)
  per HIPAA Security Rule 45 CFR 164.308(a)(7)
  contingency plan; the operating cadence for DR
  drills is documented.
- Manual workarounds (paper-based ADT, paper-based
  CPOE, paper-based eMAR) are pre-printed and
  distributed to the operating units so that an HIS
  outage does not arrest patient care; reconciliation
  with the HIS resumes when service is restored.

## §13 Research-Data Use Integration

Secondary use of HIS records for clinical research is
gated by:

- The patient's consent under HIPAA 45 CFR 164.508
  authorisation, GDPR Article 9(2)(j) scientific-
  research basis, or KR PIPA Article 28-2
  pseudonymised data for scientific research.
- The institutional review board's protocol approval.
- The de-identification or pseudonymisation applied
  before the research data set is released.
- The data-use-agreement that the receiving research
  team executes.

The hospital's research-data integration carries the
de-identification log, the protocol reference, and
the research-team identifier so that the audit trail
spans the secondary-use transfer.

## §14 Conformance

Implementations claiming PHASE-4 conformance publish
the IHE Affinity Domain agreement (or its
equivalent), expose the FHIR R5 SMART-on-FHIR launch
and Bulk Data endpoints declared in their
`CapabilityStatement`, operate the public-health-
reporting integration that their jurisdiction
requires, expose the patient-portal access-right
surface, hold the EMR-certification record (for
jurisdictions where one is operated), maintain the
DR / BCP discipline against the HIPAA Security Rule
contingency-plan requirements, and operate the
research-data integration described above so that
secondary use of HIS records is governed end to end.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-hospital-info-system
- **Last Updated:** 2026-04-28
