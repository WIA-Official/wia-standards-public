# WIA-healthcare-integration PHASE 3 — PROTOCOL Specification

**Standard:** WIA-healthcare-integration
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern a
healthcare operator integrating clinical and
administrative records across organisational and
jurisdictional boundaries: the FHIR R5 resource-shape
discipline, the IHE document-sharing actor discipline,
the consent-discipline that gates access to PHI, the
break-the-glass discipline that authorises emergency
access outside ordinary consent, the cross-border
discipline that mediates GDPR Article 9 / HIPAA
business-associate-agreement / KR-PIPA Article 17
transfers, the audit-discipline that produces the
record-of-disclosure that a HIPAA-Privacy-Rule
accounting-of-disclosures (45 CFR 164.528) and a GDPR
Article 15 right of access can both be served from,
and the data-quality discipline that keeps the FHIR
resources internally consistent.

References (CITATION-POLICY ALLOW only):

- ISO 9001:2015 (quality management systems)
- ISO/IEC 27001:2022 (information security management)
- ISO 27799:2016 (information security management in
  health using ISO/IEC 27002)
- ISO 13606-1:2019 / 13606-2:2019 (EHR communication
  reference model and archetype interchange)
- HL7 FHIR R5 conformance and search specifications
- HL7 v2.x messaging (legacy ADT / ORU feeds)
- IHE XDS.b transactions ITI-41 / ITI-43 / ITI-18
- IHE XCA transactions ITI-38 / ITI-39
- IHE PIX V3 ITI-45 / IHE PDQ V3 ITI-47
- IHE PCC profiles for encounter-summary and care-
  plan documents
- DICOM PS3.4 (service classes), PS3.18 (DICOMweb)
- IETF RFC 5905 (NTPv4), RFC 9421 (HTTP Message
  Signatures), RFC 9457 (Problem Details), RFC 8259
  (JSON), RFC 8615 (well-known URIs)
- US HIPAA Privacy Rule (45 CFR Part 164, Subpart E,
  including 164.502 minimum necessary, 164.508
  authorization, 164.510 use and disclosure for
  involvement, 164.512 public-interest exceptions,
  164.522 access restrictions, 164.524 access right,
  164.526 amendment right, 164.528 accounting of
  disclosures, 164.530 administrative safeguards)
- US HIPAA Security Rule (45 CFR Part 164, Subpart C,
  including 164.308 administrative, 164.310 physical,
  164.312 technical safeguards, 164.314 organisational
  safeguards, 164.316 documentation)
- EU GDPR (Regulation (EU) 2016/679) Articles 5, 6,
  9, 12 to 22, 24 to 30, 32, 33 to 34
- KR Medical Service Act Article 21 (medical-record
  inspection right) and Article 23 (electronic
  medical-record curation discipline)
- KR PIPA Articles 17 (third-party disclosure), 23
  (sensitive information), 28-2 (pseudonymised data
  for scientific research)
- EU European Health Data Space Regulation (cross-
  border secondary-use governance)

---

## §1 FHIR R5 Resource-Shape Discipline

The operator's FHIR server enforces the FHIR R5
profile constraints on every resource it accepts:

- `Patient.identifier` MUST carry the issuing-
  authority OID or the FHIR-published namespace URI;
  jurisdictional identifiers (KR resident registration
  number, US medical record number, EU Member State
  national identifier) follow the issuing-authority's
  identifier-format guidance.
- `Encounter.subject` MUST resolve to a `Patient`
  resource within the operator's master patient index
  or — for cross-community records — within the
  community's PIX-resolved cross-reference.
- `Observation.code` MUST carry a LOINC code for
  laboratory observations; the operator MAY add a
  SNOMED CT translation under the FHIR `coding`
  array.
- `MedicationRequest.medication` MUST carry the
  jurisdiction-appropriate code (RxNorm for US, ATC
  for EU, KR National Drug Master File for KR).
- `Consent` MUST carry a `provision` tree that maps
  to the consent-directive scope-of-use (PHASE-1 §8).

Resources that fail the shape discipline are rejected
with FHIR `OperationOutcome` of severity `error` and
the issue's `expression` field locating the failing
element.

## §2 IHE Actor Discipline

The operator declares the IHE actors it implements:

- Document Source — the actor that submits documents
  to the registry (ITI-41).
- Document Repository — the actor that stores
  documents (ITI-42 / ITI-43).
- Document Registry — the actor that indexes
  documents (ITI-18 stored query).
- Document Consumer — the actor that queries and
  retrieves.
- Initiating Gateway / Responding Gateway — the XCA
  cross-community actors (ITI-38 / ITI-39).
- Patient Identity Source / Patient Identifier Cross-
  Reference Manager — the PIX actors (ITI-44 /
  ITI-45).

Each actor is registered with the operator's IHE
Affinity Domain agreement; the agreement declares the
allowed `homeCommunityId` values and the trust-
relationship between communities.

## §3 Consent-Directive Access Discipline

Access to PHI is gated by the consent-directive
discipline:

1. The requesting practitioner's identity, role, and
   purpose-of-use are extracted from the OAuth 2.1
   bearer token (PHASE-2 §12).
2. The operator's policy-decision point evaluates the
   patient's active consent directives against the
   request's purpose-of-use, recipient category, and
   scope-of-use.
3. If the directives authorise the request the policy-
   decision point returns `permit`; the policy-
   enforcement point allows the FHIR or IHE
   transaction to proceed and emits a `success`
   audit-event.
4. If the directives reject the request the policy-
   decision point returns `deny`; the FHIR layer
   returns a 403 with the rejecting directive
   reference in the `OperationOutcome` and emits a
   `serious-failure` audit-event of type `query` or
   `retrieve` per the requested action.
5. If the directives are silent the operator falls
   back to the regulatory baseline — for HIPAA-
   regulated operators this is the 45 CFR 164.506
   treatment-payment-operations basis (permit for
   treatment, payment, operations purposes); for
   GDPR-regulated operators this is the Article 9(2)
   basis declared in the programme record.

## §4 Break-the-Glass Discipline

Break-the-glass access (emergency access outside the
ordinary consent discipline, justified by an imminent
threat to the patient's life or health) requires:

- The requesting practitioner's role to be one of the
  break-the-glass-eligible roles declared in the
  operator's policy.
- The OAuth scope `launch/break-the-glass` to be
  present on the bearer token.
- The free-text `rationale` URI to be supplied by
  the requesting practitioner.
- The audit-event to be emitted with `eventType =
  "break-the-glass"` and the rationale captured.
- The patient (or the patient's authorised
  representative) to be notified within the operator's
  break-the-glass review cadence (HIPAA 45 CFR 164.510
  uses-and-disclosures-for-involvement applies; KR
  Medical Service Act Article 21 right-of-inspection
  applies).

## §5 Audit and Accounting-of-Disclosures Discipline

Every action the operator's FHIR / IHE / WIA endpoints
take produces a FHIR `AuditEvent` (PHASE-1 §10). The
audit log is the source-of-record for both:

- The HIPAA Privacy Rule 45 CFR 164.528 accounting-of-
  disclosures the patient may request — disclosures
  outside treatment-payment-operations (TPO) are
  reported to the patient on demand within the
  rule's six-year horizon.
- The GDPR Article 15(1)(c) right-of-access response
  — the recipients of the data subject's personal
  data are reported on demand.
- The KR Medical Service Act Article 21 right-of-
  inspection response — patients (and authorised
  representatives) are entitled to inspect their
  records.

The audit log is integrity-protected per HIPAA Security
Rule 45 CFR 164.312(c) (integrity controls); the
operator MAY append-only-log the audit events with the
operator's chosen tamper-evident mechanism.

## §6 Cross-Border Transfer Discipline

Cross-border transfers are recorded in PHASE-1 §11
and gated by the discipline:

1. The operator identifies the destination jurisdiction
   from the recipient's ISO 3166-1 country code and
   the recipient's regulatory regime.
2. For GDPR-regulated source data the discipline
   applies the Chapter V mechanism declared in the
   transfer record — Article 45 adequacy decision,
   Article 46 SCC / BCR, or Article 49 derogation
   (vital interest, explicit consent, important
   reasons of public interest).
3. For HIPAA-regulated source data the discipline
   applies the business-associate-agreement (45 CFR
   164.504(e)) when the recipient is a business
   associate; the agreement carries the allowable
   uses and disclosures.
4. For KR-PIPA-regulated source data the discipline
   applies Article 17 (third-party disclosure) and
   Article 28-8 (cross-border transfer) — the
   patient's prior consent, the transfer-impact
   assessment, and the destination jurisdiction's
   adequacy status are reviewed.
5. If the EHDS infrastructure is used the operator
   integrates with the Member State's MyHealth@EU
   National Contact Point per the EHDS regulation.

A transfer that fails any of the above is rejected
at the API layer with a 422 problem document.

## §7 Data-Quality Discipline

Implementations apply ISO 9001 quality discipline to
the FHIR resource population:

- LOINC mapping table reviews on the operator's
  declared cadence (typically annual).
- ICD-11 / SNOMED CT mapping table reviews on the
  WHO / SNOMED International release cadence.
- Inter-rater reliability checks on the practitioner's
  problem-list and encounter-diagnosis coding.
- Reconciliation between the source EHR and the FHIR
  facade where the FHIR layer is a thin facade over a
  legacy v2 messaging substrate; reconciliation
  failures produce data-quality issues that are tracked
  to closure.

## §8 Imaging Integration Discipline

Imaging studies follow the DICOM PS3.4 service-class
discipline; DICOMweb (PS3.18) is the recommended wire
format. The operator's PACS records the DICOM Study
Instance UID and the DICOM SOP Instance UID; the FHIR
`ImagingStudy` resource cross-references these UIDs.
Imaging studies traversing community boundaries use
the IHE XDS-I.b document-source / document-repository
pattern with the imaging manifest serialised as a
DICOM Key Object Selection document.

## §9 Time-and-Identity Discipline

NTPv4 (RFC 5905) stratum-2 or better is the operator's
clock baseline. FHIR `instant` values are emitted with
millisecond precision in UTC. Practitioner identity is
maintained in the operator's provider directory and
referenced from the FHIR `Practitioner` /
`PractitionerRole` resources; patient identity is
maintained in the master patient index and resolved
through PIX V3 / FHIR `Patient/$match` for cross-
community queries.

## §10 Conformance

Implementations claiming PHASE-3 conformance enforce
the discipline at the policy-decision point, emit the
audit-events for every action, satisfy the cross-
border discipline before recording any transfer, and
preserve the FHIR / IHE artefacts for the operating
retention horizon. The operator's compliance-and-audit
function exercises the discipline through the
operator's QMS (PHASE-4 §3).

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-healthcare-integration
- **Last Updated:** 2026-04-28
