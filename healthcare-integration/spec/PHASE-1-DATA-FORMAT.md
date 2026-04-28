# WIA-healthcare-integration PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-healthcare-integration
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format layer for
WIA-healthcare-integration. The standard covers the
persistent record shapes that a healthcare provider,
payer, public-health authority, health-information
exchange, or health-data-sharing intermediary maintains
when integrating clinical and administrative information
across organisational boundaries — patient demographics,
encounters, observations, diagnostic reports, medication
orders, immunisation records, document references for
clinical narratives, consent directives, audit events,
and the cross-organisational identity bindings that allow
a record curated in one jurisdiction to be located,
retrieved, and trusted in another. Records are consumed
by treating clinicians at the requesting site, by the
patient through the patient-portal export, by the payer
for claims adjudication, by public-health authorities
under their statutory reporting basis, by external
auditors, and — where the integration crosses
jurisdictional borders — by the supervisory data-
protection authority that oversees the cross-border
transfer.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time representation)
- ISO/IEC 11578 (UUID) and IETF RFC 4122 (UUID URN)
- ISO/IEC 27001:2022 (information security management)
- ISO 27799:2016 (information security management in
  health using ISO/IEC 27002)
- ISO 13606-1:2019 (electronic health record
  communication, reference model) and ISO 13606-2:2019
  (archetype interchange)
- HL7 FHIR Release 5 (the international standard for
  representing and exchanging healthcare information
  electronically; cited normatively for every resource
  that this PHASE references)
- HL7 v2.x (cited where the operating site uses v2
  messaging for legacy admit-discharge-transfer or
  laboratory feeds; FHIR is the preferred wire format)
- IHE XDS.b (Cross-Enterprise Document Sharing) and
  IHE XCA (Cross-Community Access) profiles, cited for
  the document-registry and document-repository record
  shapes
- IHE PCC (Patient Care Coordination) profiles, cited
  for the encounter-summary record shape
- DICOM PS3 (Digital Imaging and Communications in
  Medicine; cited for the imaging study reference)
- LOINC (Logical Observation Identifiers Names and
  Codes), SNOMED CT (Systematized Nomenclature of
  Medicine — Clinical Terms), and ICD-11 (the World
  Health Organization's eleventh revision of the
  International Classification of Diseases) are the
  required code systems for observation, problem,
  and procedure coding
- IETF RFC 8259 (JSON), RFC 9457 (Problem Details)
- EU GDPR (Regulation (EU) 2016/679) Article 9 health-
  data special-category processing basis
- US HIPAA Privacy Rule (45 CFR Part 164, Subpart E)
  and Security Rule (45 CFR Part 164, Subpart C)
- KR Medical Service Act (의료법) electronic medical
  record provisions and KR Personal Information
  Protection Act (PIPA, 개인정보보호법) sensitive-
  information clauses for health data
- EU European Health Data Space Regulation (cited where
  the cross-border transfer uses the EHDS infrastructure)

---

## §1 Scope

This PHASE defines persistent shapes for the artefacts
exchanged when a record curated by one healthcare
organisation is shared with — or accessed by — another.
Implementations covered include:

- Single-site provider organisations exchanging documents
  with referral partners.
- Multi-site provider organisations operating a regional
  health-information exchange (HIE).
- Cross-jurisdictional exchanges operating under the
  EHDS or under a comparable framework where the patient
  is treated outside the country where the record was
  curated.
- Payers (statutory or private) integrating clinical data
  for utilisation review and claims adjudication.
- Public-health authorities receiving notifiable-disease,
  immunisation, and population-health reports under the
  statutory public-health reporting basis.

GDPR Article 9 health data, HIPAA "protected health
information" (PHI), and KR-PIPA sensitive personal
information receive identical encoding in this PHASE; the
additional safeguards required by each regime are encoded
in PHASE-3 §4.

## §2 Programme Identifier

```
programmeId          : string (uuidv7)
operatorName         : string (legal name of the
                       operator — provider organisation,
                       HIE, payer, or public-health
                       authority)
operatorJurisdiction : array of string (ISO 3166-1
                       country codes of the operator's
                       establishments)
operatorRole         : enum ("provider-single-site" |
                       "provider-multi-site" |
                       "health-information-exchange" |
                       "payer" | "public-health-
                       authority" | "intermediary" |
                       "user-defined")
governingFrameworks  : array of enum ("HL7-FHIR-R5" |
                       "HL7-V2-X" | "IHE-XDS-B" |
                       "IHE-XCA" | "IHE-PCC" |
                       "DICOM-PS3" | "ISO-13606" |
                       "EU-EHDS" | "EU-GDPR-ART-9" |
                       "US-HIPAA-PRIVACY" |
                       "US-HIPAA-SECURITY" |
                       "KR-Medical-Service-Act" |
                       "KR-PIPA-SENSITIVE" |
                       "user-defined")
codeSystemsUsed      : array of enum ("LOINC" |
                       "SNOMED-CT" | "ICD-11" |
                       "ICD-10-CM" | "RxNorm" |
                       "ATC" | "KCD-8" | "user-defined")
programmeStatus      : enum ("design" | "operating" |
                       "limited-rollout" | "wind-down"
                       | "archived")
```

The combination of `governingFrameworks` and
`codeSystemsUsed` records the operator's interoperability
baseline. Implementations that exchange records with
counterparties in another jurisdiction declare both
jurisdictions' frameworks so that PHASE-3 §6 (cross-
border discipline) can derive the applicable transfer
mechanism.

## §3 Patient Demographic Record (FHIR Patient resource)

The patient demographic record aligns with the FHIR R5
`Patient` resource:

```
patientRecord:
  patientId          : string (uuidv7; the operator's
                       master-patient-index identifier)
  identifierBindings : array of object (per-jurisdiction
                       identifiers — for example the KR
                       resident registration number, the
                       US medical record number, the EU
                       Member State national identifier
                       — each carrying the issuing
                       authority and the scope of use
                       per the issuing-authority's
                       guidance)
  givenName          : array of string
  familyName         : string
  birthDate          : string (ISO 8601 date; partial
                       dates allowed per FHIR R5 patient
                       resource cardinality rules)
  administrativeSex  : enum ("female" | "male" |
                       "unknown" | "other")
  preferredLanguage  : string (BCP 47)
  contactDirectives  : array of object (per-channel
                       contact preferences; the
                       operator's HIPAA Privacy Rule
                       45 CFR 164.522 confidential-
                       communications request, where
                       made, is encoded here)
  consentReferences  : array of string (PHASE-1 §8
                       consent-record references)
```

## §4 Encounter Record (FHIR Encounter resource)

```
encounterRecord:
  encounterId        : string (uuidv7)
  patientRef         : string (PHASE-1 §3 record
                       reference)
  encounterClass     : enum ("ambulatory" | "emergency"
                       | "inpatient" | "home-health" |
                       "virtual" | "user-defined")
  serviceLocation    : object (FHIR Location reference;
                       for cross-jurisdictional records
                       the location includes the ISO
                       3166-1 country code)
  performingProvider : array of object (per-role
                       practitioner references; coded
                       against the operator's provider
                       directory)
  encounterStart     : string (ISO 8601)
  encounterEnd       : string (ISO 8601; absent until
                       discharge)
  encounterReason    : array of object (ICD-11 or
                       SNOMED CT coded encounter
                       reason; ICD-11 is the WHO-
                       maintained reference)
  encounterDiagnosis : array of object (admit, working,
                       and discharge diagnoses with
                       their condition lifecycle stage
                       per FHIR Condition resource
                       cardinality)
```

## §5 Observation Record (FHIR Observation resource)

The observation record applies to laboratory results,
vital signs, social-history observations, and patient-
reported outcomes:

```
observationRecord:
  observationId      : string (uuidv7)
  patientRef         : string
  encounterRef       : string (PHASE-1 §4 record
                       reference; absent for free-
                       standing observations)
  observationCode    : object (LOINC code is required
                       for laboratory observations;
                       SNOMED CT or LOINC for vital
                       signs; ICD-11 cross-walked for
                       coded clinical findings where
                       LOINC has no direct concept)
  observationStatus  : enum ("registered" | "preliminary"
                       | "final" | "amended" |
                       "corrected" | "cancelled" |
                       "entered-in-error")
  effectiveTime      : string (ISO 8601 instant or
                       interval)
  valueQuantity      : object (numerical value with
                       UCUM unit)
  valueCodeable      : object (coded result, for
                       categorical observations)
  referenceRange     : array of object (laboratory
                       reference range with low, high,
                       text, applies-to, age-applies-to)
  performerRef       : array of string (laboratory or
                       practitioner references)
```

## §6 Diagnostic Report and Imaging Reference

```
diagnosticReport:
  reportId           : string (uuidv7)
  patientRef         : string
  encounterRef       : string
  reportCategory     : enum ("laboratory" | "radiology"
                       | "pathology" | "cardiology" |
                       "molecular-genetics" |
                       "user-defined")
  reportCode         : object (LOINC document type)
  resultObservations : array of string (PHASE-1 §5
                       observation references rolled
                       up into the report)
  imagingStudyRef    : object (DICOM Study Instance UID
                       and the IHE XDS-I.b document
                       reference; absent unless the
                       report carries imaging)
  presentedFormRef   : array of string (URI of the PDF
                       or rendered narrative; the
                       operator MAY emit both a
                       structured form (FHIR Bundle)
                       and a presentation form)
```

## §7 Medication Order and Administration Record

```
medicationOrder:
  orderId            : string (uuidv7)
  patientRef         : string
  encounterRef       : string
  medicationCode     : object (RxNorm for US-jurisdiction
                       orders; ATC for EU-jurisdiction
                       orders; KR National Drug Master
                       File code for KR-jurisdiction
                       orders)
  dosageInstruction  : array of object (dose, route,
                       frequency, duration; FHIR
                       Dosage data type)
  prescriberRef      : string
  authoredOn         : string (ISO 8601)
  status             : enum ("active" | "completed" |
                       "stopped" | "on-hold" |
                       "cancelled" | "entered-in-error")

medicationAdministration:
  administrationId   : string (uuidv7)
  orderRef           : string
  administeredAt     : string (ISO 8601)
  performerRef       : string
  doseGiven          : object (UCUM-quantified dose)
  refusalReason      : string (absent unless the patient
                       refused)
```

## §8 Consent Directive Record

The consent directive record encodes the patient's
informed consent for the curation, sharing, and
secondary use of their record. The directive applies
across the regulatory regimes the operator declares in
§2; HIPAA, GDPR Article 9, and KR-PIPA each accept the
shape with the regime-specific basis encoded in the
applicable field:

```
consentDirective:
  directiveId        : string (uuidv7)
  patientRef         : string
  programmeRef       : string (PHASE-1 §2 reference)
  capturedAt         : string (ISO 8601)
  directiveBasis     : enum ("hipaa-authorization-45-
                       cfr-164-508" | "hipaa-treatment-
                       payment-operations" | "gdpr-art-
                       9-2-h-health-care" | "gdpr-art-
                       9-2-i-public-health" | "gdpr-
                       art-9-2-a-explicit-consent" |
                       "kr-pipa-sensitive-art-23-
                       explicit-consent" | "kr-
                       medical-service-act-emergency"
                       | "user-defined")
  scopeOfUse         : array of enum ("treatment" |
                       "payment" | "operations" |
                       "research-de-identified" |
                       "research-identifiable" |
                       "secondary-use-public-health"
                       | "marketing" | "user-defined")
  recipientCategory  : array of string (the categories
                       of recipient organisations the
                       directive authorises — referral
                       providers, payers, public-
                       health authorities, etc.)
  effectiveFrom      : string (ISO 8601)
  effectiveUntil     : string (ISO 8601; absent for
                       open-ended directives)
  withdrawalAt       : string (ISO 8601; absent until
                       withdrawn)
  withdrawalChannel  : string (URI of the withdrawal
                       channel — under GDPR Article 7(3)
                       the channel must be as easy as
                       the capture channel; HIPAA
                       45 CFR 164.508(b)(5) requires
                       written revocation)
```

## §9 Document Reference Record (IHE XDS / XCA)

The document reference record is the metadata envelope
used by IHE XDS.b document registries and IHE XCA
cross-community gateways:

```
documentReference:
  documentId         : string (uuidv7)
  patientRef         : string
  documentClass      : object (LOINC document-type code)
  authorRef          : array of string (practitioner
                       and organisation references)
  authoredAt         : string (ISO 8601)
  serviceStartTime   : string (ISO 8601)
  serviceStopTime    : string (ISO 8601; absent for
                       point-in-time documents)
  formatCode         : object (IHE-published formatCode
                       or FHIR document-reference
                       format)
  mimeType           : string (e.g. "application/pdf",
                       "application/fhir+json")
  contentRef         : string (URI of the binary
                       content; HTTPS, integrity-
                       protected per PHASE-3 §3)
  homeCommunityId    : string (ISO OID of the source
                       community for cross-community
                       access per IHE XCA; absent for
                       intra-community documents)
```

## §10 Audit-Event Record

```
auditEvent:
  eventId            : string (uuidv7)
  occurredAt         : string (ISO 8601 instant)
  eventType          : enum ("query" | "retrieve" |
                       "create" | "update" | "delete"
                       | "register-document" |
                       "query-cross-community" |
                       "consent-capture" |
                       "consent-withdrawal" |
                       "break-the-glass" |
                       "user-defined")
  subjectPatientRef  : string (PHASE-1 §3 reference;
                       absent for system-level events)
  actorRef           : object (the practitioner or
                       system identity invoking the
                       action)
  outcomeKind        : enum ("success" | "minor-
                       failure" | "serious-failure" |
                       "major-failure")
  rationale          : string (URI of the rationale
                       narrative for break-the-glass
                       events; absent for routine
                       events)
```

## §11 Cross-Border Transfer Record

```
crossBorderTransfer:
  transferId         : string (uuidv7)
  programmeRef       : string
  patientRef         : string
  destinationCountry : string (ISO 3166-1)
  transferKind       : enum ("ehds-data-altruism" |
                       "ehds-cross-border-care" |
                       "gdpr-art-49-derogation-vital-
                       interest" | "gdpr-art-46-2-c-
                       scc" | "gdpr-art-46-2-b-bcr" |
                       "hipaa-business-associate-
                       agreement" | "user-defined")
  transferImpactAssessmentRef : string (URI of the
                       transfer-impact assessment per
                       the destination jurisdiction's
                       regulatory regime)
  effectiveFrom      : string (ISO 8601)
  effectiveUntil     : string (ISO 8601; absent until
                       superseded)
```

## §12 Conformance

Implementations claiming PHASE-1 conformance emit each
of the records defined above for the patients within
the operating programme, maintain the audit-event
record for every query, retrieve, or modification, and
preserve the document-reference and consent-directive
records for the operating retention horizon (HIPAA
45 CFR 164.530(j) requires six years; GDPR Article
5(1)(e) requires a no-longer-than-necessary horizon
documented in PHASE-3 §3; KR Medical Service Act
Article 22(2) requires ten years for medical records).

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-healthcare-integration
- **Last Updated:** 2026-04-28
