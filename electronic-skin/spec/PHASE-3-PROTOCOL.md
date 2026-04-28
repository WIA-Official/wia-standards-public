# WIA-electronic-skin PHASE 3 — PROTOCOL Specification

**Standard:** WIA-electronic-skin
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern
an electronic-skin operator across the
manufacturer-to-clinician-to-patient value chain:
the IEC 60601-1 essential-performance discipline,
the IEC 80601-2-49 multifunction patient-monitor
discipline that gates a multi-parameter patch,
the ISO 14971 risk-management discipline that
links risk control to the device's design history
file, the IEC 62304 software discipline that
gates an over-the-air update, the ISO 10993
biocompatibility discipline that qualifies the
patch substrate against ISO 10993-5 cytotoxicity
and ISO 10993-10 / ISO 10993-23 irritation, the
IEEE 11073-10101 nomenclature discipline that
binds every observation to a controlled term
code, the FHIR R5 mapping discipline that aligns
the device telemetry with the clinical record,
the consent-and-privacy discipline that gates
access to body-borne sensor data, the cross-
border discipline that mediates GDPR Article 9 /
HIPAA business-associate-agreement / KR-PIPA
Article 17 transfers, the post-market
surveillance discipline that detects emergent
safety signals, and the chain-of-custody anchoring
discipline that prevents silent mutation of the
clinical record.

References (CITATION-POLICY ALLOW only):

- IEC 60601-1:2005+AMD1:2012+AMD2:2020 (general
  requirements for basic safety and essential
  performance)
- IEC 60601-1-2:2014+AMD1:2020 (electromagnetic
  disturbances)
- IEC 60601-1-6:2010+AMD1:2013+AMD2:2020
  (usability)
- IEC 60601-1-9:2007+AMD1:2013+AMD2:2020
  (environmentally conscious design)
- IEC 80601-2-49:2018 (multifunction patient
  monitoring)
- IEC 60601-2-25/-27/-47/-26/-40 (per-modality
  particular standards)
- IEC 62304:2006+AMD1:2015 (medical-device
  software life-cycle)
- IEC 62366-1:2015+AMD1:2020 (usability
  engineering)
- ISO 14971:2019 (risk management)
- ISO 13485:2016+A11:2021 (medical-device QMS)
- ISO 10993-1:2018, ISO 10993-5:2009, ISO 10993-
  10:2010, ISO 10993-23:2021 (biological
  evaluation)
- IEEE 11073-10101:2019 and the IEEE 11073
  specialisation profiles cited in PHASE-1
  references
- IEEE 802.15.6:2012 (BAN)
- HL7 FHIR Release 5 (Patient, Encounter,
  Observation, Device, Consent, AuditEvent
  resources and the search interaction)
- IETF RFC 9110 / 9111 / 9421 / 9457 / 8615 / 6962
- W3C Trace Context
- EU Medical Device Regulation (EU) 2017/745
  (Annex I general safety and performance
  requirements, Annex II technical documentation,
  Annex III post-market surveillance, Annex IX
  conformity assessment)
- US 21 CFR Part 820 (QSR), 21 CFR Part 803
  (Medical Device Reporting)
- KR 의료기기법 (Medical Devices Act) and KR
  PIPA Articles 17 / 23 / 28-2

---

## §1 IEC 60601-1 Essential-Performance Discipline

Every device record carried by the operator's API
declares the essential-performance characteristics
per IEC 60601-1:2005+AMD2:2020 §4.3 and the
applicable particular standards. The operator's
API rejects a device registration whose declared
`signalChannels` set does not satisfy the
essential-performance requirements of the cited
particular standard — for example, an IEC
60601-2-47 ambulatory ECG device whose declared
sampling rate is below the standard's minimum
returns `422 Unprocessable Entity` at
`/problems/iec60601-particular-essential-
performance`.

## §2 IEC 80601-2-49 Multifunction Discipline

A multifunction patient-monitor device (an e-skin
patch reporting two or more vital-sign
modalities) MUST cite IEC 80601-2-49:2018 in its
`applicableParticular` set. The operator's API
verifies that the device's IEC 80601-2-49
collateral conformance test report references the
particular standards for each modality declared
in `signalChannels` so that the multifunction
discipline does not bypass the per-modality
particular standard.

## §3 ISO 14971 Risk-Management Discipline

### §3.1 Risk register binding

Every device record carries a reference to the
ISO 14971:2019 risk-management file. The risk
register is reviewed annually under the ISO 14971
§4.5 risk-management plan and is updated when a
post-market surveillance signal triggers the
ISO 14971 §10 production-and-post-production
information feedback loop.

### §3.2 Risk-control verification

A residual-risk evaluation per ISO 14971 §7 is
the gate for the regulatory submission's safety-
and-performance evidence. The operator's API
records the risk-control verification reference
in the device's audit envelope.

## §4 ISO 10993 Biocompatibility Discipline

### §4.1 Test-method binding

Every biocompatibility record carries the ISO
10993 test reference enumeration:

- A patch substrate making prolonged skin contact
  (longer than 24 hours, per ISO 10993-1:2018
  Table A.1) MUST cite ISO 10993-5 cytotoxicity,
  ISO 10993-10 or ISO 10993-23 irritation, and
  ISO 10993-10 sensitisation in the
  biocompatibility record set.
- A patch with extractable adhesive components
  declares the chemical-characterisation test per
  ISO 10993-18.

### §4.2 Test-laboratory accreditation

The test laboratory's ISO/IEC 17025 accreditation
scope MUST cover the declared test method. The
operator's API verifies the scope at upload time
and records the verification result in the
record's audit envelope.

## §5 IEC 62304 Software-Update Discipline

### §5.1 Software-class binding

Every software-update record declares the IEC
62304 software class (A, B, or C). A class C
device's update follows the IEC 62304 §5
verification-and-validation discipline before
distribution; the operator's API gates the
publication of a class C update on the
verification-record reference declared in the
update envelope.

### §5.2 Field-safety corrective-action linking

Where the update addresses a regulator-reportable
safety defect, the update envelope references the
EU MDR Article 89 vigilance report or the US 21
CFR Part 803 Medical Device Reporting record so
that the regulator's audit can trace the
corrective action to the originating defect.

## §6 IEEE 11073-10101 Nomenclature Discipline

Every observation carried by the operator's API
binds the channel modality to a term code from
the IEEE 11073-10101:2019 nomenclature. The
operator's API rejects an observation whose
`channelRef.signalChannels[].termCode` is not
present in the IEEE 11073-10101 master list. The
master list is updated by IEEE on a multi-year
cycle; the operator's API tracks the master list
version and refreshes the local copy on every
publication of a new version.

## §7 FHIR R5 Mapping Discipline

### §7.1 Code-system cross-reference

The FHIR Observation `code.coding` array carries
both the IEEE 11073-10101 term code and the
LOINC code so that a downstream consumer can
correlate the device-side telemetry with the
clinical-record vocabulary. The operator's API
maintains the IEEE 11073-10101 ↔ LOINC mapping
table per the published cross-reference; a code
in either vocabulary that does not have a peer
in the cross-reference returns the FHIR
Observation with the unmapped code carried as a
`category` extension so that the omission is
visible to the consumer.

### §7.2 Patient-resource alignment

The FHIR Patient resource referenced by an
observation is the operator's master-patient
record. Where the patient is treated by an
external healthcare provider, the operator
publishes a SMART-on-FHIR launch sequence so
that the external provider's FHIR client can
load the operator's observation into the
provider's clinical workflow without bilateral
integration.

## §8 Consent-and-Privacy Discipline

### §8.1 Consent-directive enforcement

Every observation referencing a patient is
gated on the patient's consent directive
declared in PHASE-1 §6. The operator's API
queries the FHIR Consent resource on each
observation upload and refuses the upload where
the consent directive does not include the
operator's data-processing scope.

### §8.2 Cross-border transfer

Where the patient's data is transferred across
a jurisdictional boundary, the operator's API
binds the transfer to the GDPR Article 46
appropriate-safeguard, the HIPAA business-
associate-agreement, or the KR PIPA Article 17
transfer mechanism. The transfer is recorded in
the chain-of-custody record so that the
supervisory data-protection authority can audit
the transfer trail.

## §9 Post-Market Surveillance Discipline

The operator runs an EU MDR Annex III post-
market surveillance plan covering vigilance
reporting, periodic safety update reporting, and
field-safety corrective actions. The operator's
API records the per-event reference and links
the post-market signal to the device's risk-
management file under PHASE-3 §3.1 so that the
risk register is updated as a function of real-
world performance data.

## §10 Chain-of-Custody Anchoring Discipline

### §10.1 Per-event transparency log

Every chain-of-custody event carried by PHASE-1
§8 is appended to a per-operator transparency log
modelled on the IETF RFC 6962 Certificate
Transparency append-only-log structure. The log
publishes a signed tree-head every signed-tree-
head period (default 24 h, configurable per
programme); the signed tree-head is anchored to
the operator's public-key set declared in
`/.well-known/wia/electronic-skin/keys.json`.

### §10.2 Mutation prevention

A custody event cannot be retroactively edited;
an amendment is recorded as a new event with
`previousEventRef` pointing at the event being
amended.

## §11 Quality-Management Discipline

The operator runs an ISO 13485:2016 quality
management system covering the device design,
manufacture, installation, post-market service,
and decommission processes. Internal audits run
on a frequency declared in the QMS quality
manual; the nonconformity register is reviewed
in the ISO 13485 §5.6 management-review cycle.
The QMS certification reference declared in the
programme record is queried against the
certification body's published register on each
device-registration request and is cached for
the TTL declared in the certification body's
HTTP caching headers.

## §12 IEC 62366-1 Usability Discipline

### §12.1 Use-error and abnormal-use file

The operator's API records the IEC 62366-1:2015
+AMD1:2020 usability engineering file reference
in the device record. The file declares the
device's intended-use environment, the user
profile (lay user, healthcare professional,
caregiver), the use-error severity assessment,
and the formative-and-summative usability
evaluation summary.

### §12.2 Risk-management cross-link

The IEC 62366-1 usability file is cross-linked
to the ISO 14971 risk-management file declared
in PHASE-3 §3 so that a use-error identified
during summative evaluation flows into the risk
register.

## §13 Electromagnetic-Compatibility Discipline

A device communicating wirelessly (BAN per IEEE
802.15.6, or a higher-layer wireless transport)
satisfies the IEC 60601-1-2:2014+AMD1:2020
electromagnetic-compatibility requirements. The
operator's API records the IEC 60601-1-2 type-
test report reference in the device record and
binds the report to the relevant IEC test
laboratory under PHASE-4 §3.2.

## §14 Cybersecurity Discipline

### §14.1 IEC 81001-5-1 software-security cross-
       reference

The IEC 62304 software life-cycle is
complemented by the IEC 81001-5-1 health-
software security activities. The operator's
software-update record declares the IEC
81001-5-1 §5 / §7 / §8 evidence reference (the
threat-model summary, the security-test report,
the security-update plan) so that the regulator
can audit the software-security activities
independently of the safety-related verification.

### §14.2 EU MDR Annex I §17 cybersecurity

A device in scope of EU MDR Annex I §17 declares
the cybersecurity measures, the security-incident
response plan, and the intended-use cybersecurity
context. The operator's API publishes the EU
MDCG 2019-16 guidance reference where the device
operates a network interface.
