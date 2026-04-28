# WIA-hospital-info-system PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-hospital-info-system
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format layer for
WIA-hospital-info-system. The standard covers the
persistent record shapes that a hospital information
system (HIS) maintains for its admissions, clinical,
pharmacy, laboratory, radiology, billing, electronic
medical record (EMR), and operational subsystems.
Whereas the WIA-healthcare-integration standard governs
record exchange between organisations, this standard
governs the in-hospital record substrate and the
internal-subsystem boundaries within a single
organisation. Records are consumed by clinicians at the
operating site, by hospital administration for
operational and statutory reporting, by the patient
through the in-hospital patient-portal, by the payer for
claims adjudication, by external auditors, and by the
supervisory data-protection authority for inspections.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time representation)
- ISO/IEC 11578 (UUID) and IETF RFC 4122 (UUID URN)
- ISO/IEC 27001:2022 (information security management)
- ISO 27799:2016 (information security in health using
  ISO/IEC 27002)
- ISO 13606-1:2019 (electronic health record reference
  model) and ISO 13606-2:2019 (archetype interchange)
- HL7 v2.5.1 / v2.8.2 messaging (the operating-site
  messaging substrate that most hospital information
  systems still use for ADT, ORM, ORU, BAR, MFN, and
  SIU message types)
- HL7 FHIR Release 5 (the modern resource-shape
  substrate for the EMR facade and for API integration
  with external partners)
- IHE PCC (Patient Care Coordination) profiles for
  clinical-document content
- IHE LAB profiles (LTW, LCSD, LBL, LCC, LPOCT) for
  laboratory operations
- DICOM PS3 (Digital Imaging and Communications in
  Medicine; cited for the radiology subsystem)
- LOINC (Logical Observation Identifiers Names and
  Codes), SNOMED CT, and ICD-11 (the World Health
  Organization's eleventh revision of the International
  Classification of Diseases) for code-system bindings
- IETF RFC 8259 (JSON), RFC 9457 (Problem Details)
- US HIPAA Security Rule (45 CFR Part 164, Subpart C)
  and Privacy Rule (45 CFR Part 164, Subpart E)
- KR Medical Service Act (의료법) Article 22 (medical-
  record curation), Article 23 (electronic medical-
  record curation discipline) and the KR EMR
  certification programme (전자의무기록 시스템 인증)
  operated under Article 23-2
- KR Personal Information Protection Act (PIPA,
  개인정보보호법) Article 23 sensitive-information
  clauses
- EU European Health Data Space Regulation (cited
  where the hospital participates in the EHDS
  infrastructure)

---

## §1 Scope

This PHASE defines persistent shapes for the artefacts
the hospital information system maintains:

- Admissions, discharges, and transfers (ADT) — bed
  occupancy, encounter status, ward and unit
  assignment.
- Computerised provider order entry (CPOE) — the
  orders the clinician places against the patient's
  encounter.
- Pharmacy — the dispense-and-administration cycle
  paired with the medication order.
- Laboratory information system (LIS) — the order-
  result cycle for laboratory tests.
- Radiology information system (RIS) and PACS — the
  imaging-order, study, and report cycle.
- Billing and claims — the charge-master and the
  claim record.
- EMR — the longitudinal clinical record visible to
  the treating clinician.
- Master files — the patient master, provider master,
  service master, and code-system master.

KR Medical Service Act medical-record curation
(Article 22) requires ten-year retention for medical
records and three-year retention for prescription and
nursing records; the discipline is encoded in PHASE-3
§9.

## §2 Hospital and Programme Identifier

```
hospitalProgrammeId  : string (uuidv7)
hospitalName         : string (legal name of the
                       hospital)
hospitalKind         : enum ("tertiary" | "general" |
                       "specialty" | "primary-care" |
                       "long-term-care" | "psychiatric"
                       | "rehabilitation" | "user-
                       defined")
hospitalJurisdiction : string (ISO 3166-1 country code)
operatingFrameworks  : array of enum ("HL7-V2-5-1" |
                       "HL7-V2-8-2" | "HL7-FHIR-R5" |
                       "DICOM-PS3" | "ISO-13606" |
                       "IHE-PCC" | "IHE-LAB" |
                       "KR-EMR-Certification" |
                       "US-HIPAA-PRIVACY" |
                       "US-HIPAA-SECURITY" |
                       "KR-Medical-Service-Act" |
                       "KR-PIPA-SENSITIVE" |
                       "EU-EHDS" | "user-defined")
codeSystemsUsed      : array of enum ("LOINC" |
                       "SNOMED-CT" | "ICD-11" |
                       "ICD-10-CM" | "RxNorm" |
                       "ATC" | "KCD-8" | "user-defined")
emrCertificationRef  : string (URI of the operating
                       jurisdiction's EMR certification
                       record — for KR hospitals this is
                       the 전자의무기록 시스템 인증
                       record)
programmeStatus      : enum ("commissioning" |
                       "operating" | "rollout-limited"
                       | "wind-down" | "archived")
```

## §3 Patient Master Record

```
patientMaster:
  patientId          : string (uuidv7; the hospital's
                       master-patient-index identifier)
  jurisdictionalIdentifiers : array of object (KR
                       resident registration number, US
                       medical record number, EU
                       Member-State national identifier;
                       the hospital records the issuing-
                       authority OID and the scope of
                       use)
  givenName          : array of string
  familyName         : string
  birthDate          : string (ISO 8601 date)
  administrativeSex  : enum ("female" | "male" |
                       "unknown" | "other")
  preferredLanguage  : string (BCP 47)
  primaryContact     : object (the next-of-kin or
                       authorised contact reference)
  insuranceCoverage  : array of object (per-payer
                       coverage records)
```

## §4 Encounter and Bed-Occupancy Record (ADT)

The ADT discipline is captured per HL7 v2.5.1 ADT
message-type semantics; the persistent shape is:

```
encounterRecord:
  encounterId        : string (uuidv7)
  patientRef         : string (PHASE-1 §3)
  encounterClass     : enum ("inpatient" | "outpatient"
                       | "emergency" | "day-surgery" |
                       "observation" | "home-care" |
                       "user-defined")
  admissionAt        : string (ISO 8601)
  dischargeAt        : string (ISO 8601; absent until
                       discharge — corresponds to the
                       v2 A03 message)
  admittingProvider  : string (PHASE-1 §6 reference)
  attendingProvider  : string
  servicedDepartment : string (e.g. internal-medicine,
                       cardiology, oncology, paediatrics)
  bedOccupancy:
    wardCode         : string
    roomCode         : string
    bedCode          : string
    occupancyFrom    : string (ISO 8601)
    occupancyTo      : string (ISO 8601; absent until
                       transferred or discharged —
                       corresponds to the v2 A02
                       transfer message)
  visitNumber        : string (the v2 PV1.19 visit
                       number for legacy interfaces)
```

## §5 Order and Order-Result Record (CPOE)

```
orderRecord:
  orderId            : string (uuidv7)
  encounterRef       : string (PHASE-1 §4)
  orderClass         : enum ("medication" | "laboratory"
                       | "radiology" | "consultation" |
                       "procedure" | "diet" | "nursing"
                       | "blood-product" | "user-
                       defined")
  orderingProvider   : string
  orderedAt          : string (ISO 8601)
  serviceCode        : object (LOINC for laboratory and
                       most diagnostic orders; SNOMED CT
                       for procedures; the hospital's
                       service-master code as a parallel
                       coding for charge-master mapping)
  priorityKind       : enum ("routine" | "stat" | "urgent"
                       | "asap" | "timing-critical")
  status             : enum ("draft" | "active" |
                       "completed" | "cancelled" |
                       "stopped" | "entered-in-error")
  resultObservations : array of string (PHASE-1 §7
                       observation references for
                       laboratory orders; the v2 ORU
                       result message that fulfils the
                       order is referenced here)
  imagingStudyRef    : string (PHASE-1 §8 reference for
                       radiology orders)
```

## §6 Provider Master Record

```
providerMaster:
  providerId         : string (uuidv7; the hospital's
                       internal provider identifier)
  jurisdictionalLicence : array of object (KR medical-
                       practitioner licence number, US
                       NPI, EU Member-State licensing-
                       authority registration)
  givenName          : array of string
  familyName         : string
  practitionerRoles  : array of object (per-role
                       assignment — physician, nurse,
                       pharmacist, technician,
                       clinical-research-coordinator)
  privileges         : array of string (the hospital's
                       privilege-and-credentialling
                       record reference; controls what
                       order classes the provider may
                       place)
```

## §7 Laboratory and Pathology Result Record

The laboratory-information-system (LIS) record-shape
follows the IHE LAB profile for the hospital's
laboratory operations:

```
labResultRecord:
  resultId           : string (uuidv7)
  orderRef           : string (PHASE-1 §5)
  resultStatus       : enum ("preliminary" | "final" |
                       "amended" | "corrected" |
                       "cancelled" | "entered-in-error")
  performedAt        : string (ISO 8601)
  performingLabRef   : string (the hospital's in-house
                       laboratory or the reference
                       laboratory the work was sent to)
  observationCode    : object (LOINC code is required;
                       for LIS-internal bench codes the
                       hospital records both the LOINC
                       code and the bench code)
  valueQuantity      : object (UCUM-quantified value)
  valueCodeable      : object (coded result for
                       categorical observations)
  referenceRange     : array of object
  abnormalFlag       : enum ("normal" | "low" | "high"
                       | "low-low" | "high-high" |
                       "abnormal" | "out-of-range" |
                       "user-defined")
  pathologyNarrativeRef : string (URI of the
                       pathology-report narrative —
                       absent for non-anatomic-pathology
                       results)
```

## §8 Imaging-Study Record (RIS / PACS)

```
imagingStudyRecord:
  studyId            : string (uuidv7)
  patientRef         : string
  encounterRef       : string
  orderRef           : string (PHASE-1 §5)
  studyInstanceUid   : string (DICOM Study Instance UID
                       — the canonical study identifier
                       for PACS)
  modality           : enum ("CR" | "CT" | "MR" | "NM"
                       | "PT" | "US" | "MG" | "DX" |
                       "RF" | "XA" | "OT" | "user-
                       defined")
  performedAt        : string (ISO 8601)
  performingTechnologist : string
  interpretingProvider : string (radiologist
                       reference)
  reportObservations : array of string (PHASE-1 §7
                       observation references for
                       structured findings)
  reportNarrativeRef : string (URI of the imaging-
                       report narrative)
  pacsInstanceCount  : integer (number of DICOM
                       instances in the study)
```

## §9 Pharmacy Dispense and Administration Record

```
pharmacyDispense:
  dispenseId         : string (uuidv7)
  orderRef           : string (the medication-order
                       reference)
  dispensedAt        : string (ISO 8601)
  dispensingPharmacist : string
  medicationCode     : object (RxNorm for US, ATC for
                       EU, KR National Drug Master File
                       for KR)
  quantity           : object (UCUM quantity)
  lotNumberRef       : string
  expiryDate         : string (ISO 8601 date)
  administrationRefs : array of string (PHASE-1 §10
                       administration references)
```

## §10 Medication Administration Record (eMAR)

```
medicationAdministration:
  administrationId   : string (uuidv7)
  dispenseRef        : string
  administeredAt     : string (ISO 8601)
  administeringNurse : string
  routeKind          : enum ("oral" | "intravenous" |
                       "intramuscular" | "subcutaneous"
                       | "transdermal" | "inhalational"
                       | "rectal" | "topical" |
                       "user-defined")
  doseGiven          : object (UCUM quantity)
  refusalReason      : string (absent unless refused;
                       the hospital's nursing
                       discipline records the reason)
```

## §11 Billing and Charge Record

```
chargeRecord:
  chargeId           : string (uuidv7)
  encounterRef       : string
  chargeMasterCode   : string (the hospital's
                       chargemaster code — the
                       chargemaster cross-walks to the
                       jurisdictional billing-code
                       system)
  jurisdictionalBillingCode : object (US CPT or
                       HCPCS, KR EDI procedure code,
                       EU national equivalent)
  serviceDate        : string (ISO 8601 date)
  performingProvider : string
  quantity           : integer
  unitPrice          : object (the chargemaster's
                       per-unit price in the hospital's
                       reporting currency)
  payerRef           : string (the responsible payer
                       reference; absent for self-pay)
```

## §12 Audit-and-Access-Log Record

```
auditEvent:
  eventId            : string (uuidv7)
  occurredAt         : string (ISO 8601 instant)
  actorRef           : string (provider or system
                       identity)
  subjectPatientRef  : string (absent for system events)
  eventKind          : enum ("login" | "view-record" |
                       "modify-record" | "place-order"
                       | "result-access" |
                       "break-the-glass" |
                       "consent-capture" |
                       "consent-withdrawal" |
                       "report-print" | "user-defined")
  outcomeKind        : enum ("success" | "minor-
                       failure" | "serious-failure")
  rationale          : string (URI of the rationale
                       narrative for break-the-glass
                       events)
```

## §13 Conformance

Implementations claiming PHASE-1 conformance maintain
the master files, encounter, order, result,
administration, charge, and audit records described
above. The KR EMR certification programme (전자의무기록
시스템 인증, KR Medical Service Act Article 23-2)
imposes additional record-shape requirements that the
hospital satisfies through its certification-record
reference; HIPAA Security Rule 45 CFR 164.312(b)
audit controls apply to the audit-event record.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-hospital-info-system
- **Last Updated:** 2026-04-28
