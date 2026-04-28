# WIA-hospital-info-system PHASE 3 — PROTOCOL Specification

**Standard:** WIA-hospital-info-system
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern a
hospital information system: the inter-subsystem
messaging discipline that ties ADT ↔ CPOE ↔ pharmacy ↔
LIS ↔ RIS ↔ billing ↔ EMR; the medication-use safety
discipline (CPOE, pharmacy verification, eMAR closed-
loop); the clinical decision-support discipline; the
break-the-glass discipline that authorises emergency
access outside the ordinary consent pattern; the
medical-record curation discipline (KR Medical Service
Act Article 22 / 23 and the KR EMR-certification
discipline; HIPAA Privacy Rule 45 CFR 164.530(j)
retention discipline; GDPR Article 5(1)(e) storage-
limitation discipline); and the audit-and-disclosure
discipline that produces the disclosure-account record
the supervisory authority and the patient can both
read.

References (CITATION-POLICY ALLOW only):

- ISO 9001:2015 (quality management systems)
- ISO/IEC 27001:2022 (information security management)
- ISO 27799:2016 (information security in health using
  ISO/IEC 27002)
- ISO 13606-1:2019 / 13606-2:2019
- HL7 v2.5.1 / v2.8.2 messaging conformance
- HL7 FHIR R5 conformance and search specifications
- IHE LAB profiles
- IHE PCC profiles
- DICOM PS3.4 (service classes), PS3.18 (DICOMweb)
- IETF RFC 5905 (NTPv4), RFC 9421 (HTTP Message
  Signatures), RFC 9457 (Problem Details), RFC 8259
  (JSON), RFC 8615 (well-known URIs)
- US HIPAA Privacy Rule (45 CFR Part 164, Subpart E,
  including 164.502 minimum necessary, 164.506
  treatment-payment-operations, 164.508 authorization,
  164.510 use and disclosure for involvement, 164.512
  public-interest exceptions, 164.522 access
  restrictions, 164.524 access right, 164.526
  amendment right, 164.528 accounting of disclosures,
  164.530 administrative safeguards)
- US HIPAA Security Rule (45 CFR Part 164, Subpart C,
  including 164.308 administrative, 164.310 physical,
  164.312 technical safeguards, 164.314 organisational
  safeguards, 164.316 documentation)
- KR Medical Service Act Article 21 (medical-record
  inspection right), Article 22 (medical-record
  curation), Article 23 (electronic medical-record
  curation discipline), Article 23-2 (EMR
  certification programme)
- KR PIPA Articles 17 (third-party disclosure), 23
  (sensitive information)
- EU GDPR Articles 5, 9, 12 to 22, 32, 33 to 34

---

## §1 Inter-Subsystem Messaging Discipline

The hospital's interface engine routes messages
between subsystems on a defined conformance profile:

- ADT message routing — every ADT event (admit,
  transfer, discharge, registration update,
  cancellation) is fanned out from the ADT system to
  CPOE, pharmacy, LIS, RIS, billing, and EMR.
- Order routing — the ORM message produced by CPOE is
  routed to pharmacy (for medication orders), LIS
  (for laboratory orders), RIS (for radiology
  orders), and the consult-request system (for
  consultation orders).
- Result routing — the ORU message produced by LIS is
  routed back to CPOE for the placer to view, to the
  EMR for longitudinal record-keeping, and (where a
  critical-value alert is configured) to the
  alerting system.
- Charge routing — DFT messages produced by ancillary
  subsystems are routed to billing for chargemaster
  reconciliation; the EMR receives a notification so
  that the encounter's financial summary stays
  current.

Each routed message is acknowledged by the receiver;
the integration-engine audit log records the message-
handling chain for every routed message.

## §2 Medication Closed-Loop Safety Discipline

Medication safety is enforced at four discrete checkpoints:

1. CPOE order entry — the medication-order ORM
   message carries the patient identifier, the
   medication code, the dose, the route, the
   frequency, and the duration. The CPOE engine
   evaluates allergy / drug-drug-interaction / dose-
   range alerts at this checkpoint.
2. Pharmacy verification — a registered pharmacist
   reviews the order in the pharmacy subsystem and
   produces the dispense record (PHASE-1 §9). Order
   modification at this checkpoint is recorded with
   the pharmacist's identifier.
3. eMAR administration — the administering nurse
   scans the patient's wristband and the medication's
   barcode (the hospital's IHE-LAB-LBL-aligned
   labelling discipline); the eMAR engine confirms
   the five-rights match (right patient, right
   medication, right dose, right route, right time)
   before recording the administration.
4. Outcome surveillance — adverse-drug-reaction
   surveillance (the hospital's pharmacovigilance
   discipline) reviews administration outcomes and
   produces incident records that loop back into the
   formulary-and-protocol committee.

## §3 Clinical Decision-Support Discipline

Clinical decision-support fires at well-defined
moments:

- Order entry — allergy / interaction / dose / age-
  appropriateness alerts.
- Result review — out-of-range and critical-value
  alerts on laboratory results, with the operating
  policy for time-bound critical-value
  acknowledgement.
- Encounter check-in — protocol prompts (immunisation
  due, screening due, preventive-care due) drawn from
  the patient's longitudinal record.
- Order set — protocol-driven order sets (sepsis,
  stroke, acute coronary syndrome) that bundle the
  evidence-based ordering pattern.

Decision-support firings are logged so that the
quality-improvement committee can review alert burden
(false-positive rate) and modify thresholds.

## §4 Consent and Access Discipline

Access to the patient's record is gated by:

- The provider's privileges (PHASE-1 §6) — the
  privilege-and-credentialling record determines what
  order classes and what record sections the
  provider may access.
- The patient's consent directives — captured under
  HIPAA 45 CFR 164.508 authorisation, GDPR Article
  9(2)(a) explicit consent, KR PIPA Article 23
  sensitive-information consent, or KR Medical
  Service Act Article 21 inspection-right rule, as
  applicable.
- The relationship between the provider and the
  patient — the hospital's encounter-context
  discipline restricts access to providers with an
  active care-team relationship to the patient
  (treatment-payment-operations under HIPAA 45 CFR
  164.506).
- Break-the-glass — emergency access outside the
  ordinary discipline carries the rationale URI and
  triggers the audit event.

## §5 Break-the-Glass Discipline

Break-the-glass access requires:

- The provider's role to be one of the break-the-
  glass-eligible roles declared in the hospital's
  policy.
- The OAuth scope `launch/break-the-glass` to be
  present.
- The free-text rationale URI to be supplied.
- The audit event to be emitted with `eventKind =
  "break-the-glass"` and the rationale captured.
- The patient (or authorised representative) to be
  notified within the operating site's break-the-
  glass review cadence.

## §6 Identity and Time Discipline

Patient identifiers are unique within the hospital's
master-patient-index. Cross-source identifier
reconciliation (a v2 A04 register message bringing in
an identifier from a regional health-information
exchange) is handled by the hospital's PIX manager.
Provider identifiers are issued from the hospital's
provider-master and bound to the OAuth identity at
authentication time. Time is NTPv4 stratum-2 or
better; v2 timestamps and FHIR `instant` values are
emitted in the operating-site time zone with the
offset declared per ISO 8601.

## §7 Data-Quality Discipline

The hospital applies ISO 9001 quality discipline to
the master-file and clinical-record content:

- Master-file reviews — the chargemaster, the
  service-master, the test-master, and the formulary
  are reviewed on the operating cadence (typically
  quarterly) so that code-system updates (LOINC,
  RxNorm, ICD-11, CPT) are reflected.
- Reconciliation — automated reconciliation between
  the LIS bench codes and the LOINC mapping; between
  the chargemaster and the jurisdictional billing-
  code system; between the formulary and the
  hospital's drug master.
- Inter-rater reliability — coder discipline checks
  on ICD-11 / CPT abstraction.

## §8 Audit and Accounting-of-Disclosures Discipline

Every action against PHI generates an audit event;
the audit log is the source-of-record for:

- The HIPAA Privacy Rule 45 CFR 164.528 accounting-
  of-disclosures the patient may request.
- The GDPR Article 15(1)(c) right-of-access response.
- The KR Medical Service Act Article 21 right-of-
  inspection response.
- The supervisory data-protection authority's
  inspection.

The audit log is integrity-protected per HIPAA
Security Rule 45 CFR 164.312(c); the operating site
MAY append-only-log the audit events with the chosen
tamper-evident mechanism.

## §9 Retention Discipline

The hospital's retention discipline aligns with:

- KR Medical Service Act Article 22(2) — ten years
  for medical records, three years for prescriptions
  and nursing records.
- HIPAA 45 CFR 164.530(j) — six years for
  policy / procedure / authorisation records.
- GDPR Article 5(1)(e) — no longer than necessary;
  the hospital documents the per-record-class
  horizon in the retention schedule.

Records past the retention horizon are migrated to
the long-term archive (PHASE-4 §6) or deleted per
the retention schedule.

## §10 Adverse-Event and Patient-Safety Discipline

The hospital's patient-safety committee operates an
adverse-event surveillance loop:

- Sentinel events (events resulting in death or
  permanent harm) are reported to the operating
  jurisdiction's reporting authority on the published
  cadence.
- Near-misses are recorded in the hospital's incident-
  reporting system with the de-identified narrative.
- Root-cause analyses are produced for every sentinel
  event; the analysis identifies the system contributors
  and the corrective actions.
- Corrective actions feed back into the CPOE
  configuration, the formulary, the order-set
  protocols, and the staff-credentialling
  discipline.

The discipline operates within the operating
jurisdiction's patient-safety regulatory framework
(US Patient Safety Quality Improvement Act, KR Patient
Safety Act 환자안전법, EU Member-State equivalents).

## §11 Infection-Prevention and Antimicrobial-Stewardship Discipline

The hospital's antimicrobial-stewardship discipline
runs across CPOE / pharmacy / LIS:

- Antimicrobial-order CPOE alerts incorporate the
  patient's culture results, the local antibiogram,
  and the formulary-restricted-agent rules.
- Pharmacy verification escalates broad-spectrum or
  restricted agents to the stewardship pharmacist
  for review.
- LIS culture-result delivery triggers prompts for
  de-escalation review when a narrower agent is
  indicated.
- Surveillance reporting feeds the operating
  jurisdiction's antimicrobial-resistance surveillance
  programme on the published cadence.

## §12 Conformance

Implementations claiming PHASE-3 conformance enforce
the discipline at the policy-decision point, emit
the audit event for every action, satisfy the
medication closed-loop discipline at the four
checkpoints, and preserve the records for the
operating retention horizon.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-hospital-info-system
- **Last Updated:** 2026-04-28
