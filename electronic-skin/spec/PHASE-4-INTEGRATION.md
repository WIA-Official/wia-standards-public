# WIA-electronic-skin PHASE 4 — INTEGRATION Specification

**Standard:** WIA-electronic-skin
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how an electronic-skin
operator integrates with the systems that
surround a body-conformal medical device: the
notified body issuing the EU MDR Annex IX
conformity assessment; the certification body
issuing the ISO 13485 QMS certificate; the
testing laboratory running ISO 10993
biocompatibility tests; the device's IEC 60601
type-test laboratory; the IEEE 11073-10101
nomenclature secretariat; the institutional
review board approving a clinical investigation;
the healthcare provider's FHIR R5 server
ingesting the observation envelope; the device's
software-update distribution service; the post-
market surveillance authority receiving vigilance
reports; the supervisory data-protection
authority overseeing cross-border transfers; and
— where the device is sold across jurisdictions
— the regulatory authority of each market.

References (CITATION-POLICY ALLOW only):

- IEC 60601-1, IEC 80601-2-49, and the per-
  modality particular standards cited in PHASE-1
  references
- IEEE 11073-10101:2019 and the IEEE 11073
  specialisation profiles cited in PHASE-1
  references
- IEEE 802.15.6:2012
- ISO 14971:2019, ISO 13485:2016+A11:2021, IEC
  62304:2006+AMD1:2015, IEC 62366-1:2015+AMD1:
  2020
- ISO 10993-1, ISO 10993-5, ISO 10993-10, ISO
  10993-23 (and ISO 10993-18 chemical
  characterisation, where the device declares
  extractables)
- HL7 FHIR R5 (RESTful API, SMART-on-FHIR launch
  sequence, Bulk Data export)
- ISO/IEC 17021-1:2015 (audit and certification),
  ISO/IEC 17065:2012 (product certification),
  ISO/IEC 17025:2017 (testing-laboratory
  competence)
- IETF RFC 8259, RFC 9457, RFC 8615, RFC 9421,
  RFC 6962
- W3C Verifiable Credentials Data Model 2.0
- EU Medical Device Regulation (EU) 2017/745
  (Annex I, II, III, IX), EU MDCG guidance
  documents (the MDCG harmonised technical
  guidance series)
- US 21 CFR Part 820 (QSR), 21 CFR Part 803
  (MDR), 21 CFR Part 11 (electronic records and
  signatures)
- KR 의료기기법 (Medical Devices Act), KR
  PIPA Articles 17 / 23 / 28-2
- KS C ISO 13485 (Korean adoption of ISO 13485)
- KS C IEC 60601-1 (Korean adoption of IEC 60601-1)

---

## §1 Notified-Body Integration

### §1.1 EU MDR Annex IX conformity assessment

A class IIa, IIb, or III device per EU MDR
Regulation (EU) 2017/745 Annex VIII undergoes a
notified-body conformity assessment under Annex
IX (full quality system + technical documentation
review). The operator's API binds the device
record to the notified body's certificate
identifier and the certificate's expiry date so
that the device's market-status flag can be
derived without bilateral integration. The
notified body's published certificate register is
queried on each device-registration request; a
suspended or withdrawn certificate triggers the
device's post-market surveillance team to issue
a market-status update.

### §1.2 Annex IX surveillance audit

The notified body conducts a surveillance audit
on the operator's QMS at least once a year per
EU MDR Annex IX §3. The operator's audit envelope
records the audit findings, the corrective
actions, and the closure date so that the
notified body's audit trail is preserved across
audit cycles.

## §2 Certification-Body Integration

### §2.1 ISO 13485 certificate register

The operator's QMS certification reference
declared in the programme record is bound to the
issuing certification body's ISO/IEC 17021-1
accreditation. The operator's API queries the
certification body's published register on each
device-registration request; a withdrawn
certificate returns `410 Gone` for the
registration endpoint until the certificate is
re-issued.

## §3 Testing-Laboratory Integration

### §3.1 ISO 10993 biocompatibility laboratory

The biocompatibility test laboratory referenced
in a biocompatibility record is verified against
the issuing accreditation body's ISO/IEC 17025
register on each upload. The accreditation scope
MUST cover the declared test method (ISO 10993-5
cytotoxicity, ISO 10993-10 / ISO 10993-23
irritation, ISO 10993-18 chemical
characterisation).

### §3.2 IEC 60601 type-test laboratory

The IEC 60601 type-test laboratory issuing the
device's safety-and-performance test report is
bound to the device record. The test report
identifier is published in the technical-
documentation summary so that the notified body
can locate the report during the Annex IX
review.

## §4 IEEE 11073-10101 Vocabulary Integration

The operator's API binds the channel modality
declared in PHASE-1 §3 `signalChannels` to the
IEEE 11073-10101 term code. The IEEE 11073
secretariat publishes nomenclature updates on a
multi-year cycle; the operator subscribes to the
secretariat's publishing endpoint and triggers
an internal review cycle when an update is
published. A term code that is deprecated by
IEEE is marked deprecated in the operator's
enumeration set but is not removed for the
duration of the operator's record-retention
period so that already-published observations
remain decodable.

## §5 Healthcare-Provider Integration

### §5.1 SMART-on-FHIR launch

A treating clinician launches the operator's
patient-side observation viewer through the
SMART-on-FHIR launch sequence. The operator
publishes the launch endpoint in the discovery
document and exchanges OAuth-style scopes with
the provider's authorisation server (the
operator's FHIR endpoint enforces scope-bound
access).

### §5.2 FHIR Bulk Data export for population
       analytics

The operator publishes a FHIR Bulk Data export
endpoint per the HL7 FHIR R5 Bulk Data Access
specification. The export is gated on the
clinical investigation's institutional-review-
board approval, on the patient's consent flag at
de-identified-research scope, and on the
provider's data-use agreement.

## §6 Post-Market Surveillance Integration

### §6.1 EU MDR vigilance reporting

A serious incident as defined by EU MDR Article
2(64) triggers a vigilance report under Article
87. The operator's API records the report
identifier and the regulator's
acknowledgement reference so that the
regulator's audit trail is preserved across
vigilance cycles.

### §6.2 US FDA Medical Device Reporting

A device-related death, serious injury, or
malfunction reportable under US 21 CFR Part 803
triggers an MDR submission to the FDA. The
operator's API records the eMDR submission
identifier and the FDA acknowledgement so that
the regulator's audit trail is preserved.

### §6.3 KR 의료기기 부작용 보고

A device-related adverse event in the KR market
is reported to the Ministry of Food and Drug
Safety under the KR Medical Devices Act and the
KR Adverse Event Reporting Regulation. The
operator's API records the submission identifier
and the regulator's acknowledgement.

## §7 Software-Update Distribution Integration

The operator's software-update distribution
service publishes the IEC 62304-conforming
update package signed by the operator's signing
key. The device-side update agent verifies the
signature against the operator's public-key set
published at the discovery endpoint. A signature
mismatch refuses the update; a refusal event is
recorded as a chain-of-custody event so that
the operator's QMS can investigate the refusal
under the ISO 13485 §8.2.4 monitoring-and-
measurement-of-product clause.

## §8 Patient-Portal and Re-Issuance

### §8.1 Patient-portal export

A patient retrieves the patient's own observation
record through the operator's patient-portal
endpoint. The endpoint authenticates the patient
through the operator's identity provider and
returns the FHIR Bundle with the patient's
observations, device records, and consent
directives so that the patient can satisfy the
HIPAA 45 CFR 164.524 access right or the GDPR
Article 15 right of access.

### §8.2 Verifiable-credentials re-issuance

A patient receiving a regulatory-grade
attestation (the device's CE marking validity,
the device's FDA clearance reference, the
biocompatibility attestation summary) can re-
issue the attestation as a W3C Verifiable
Credential signed by the operator's public-key
set so that a downstream healthcare provider can
validate the attestation without contacting the
operator directly.

## §9 KR-Jurisdiction Integration

### §9.1 KS C IEC 60601-1 / KS C ISO 13485
       adoption

A KR-jurisdiction operator declares the KS C
IEC 60601-1 and KS C ISO 13485 adoption
references in the programme record's
`governingFrameworks`. The KR Ministry of Food
and Drug Safety operates the KR medical-device
品目허가번호 register; the operator's API queries
the register on device-registration and on each
retrieval after the caching TTL.

### §9.2 KR PIPA sensitive-information binding

A KR-jurisdiction patient's body-borne sensor
data is sensitive personal information under KR
PIPA Article 23. The operator's API records the
KR PIPA Article 18 third-party-disclosure consent
where the data is shared with an external
analytics service, and the operator publishes
the privacy notice that names the recipient,
the purpose of disclosure, and the retention
period.

## §10 References (consolidated)

The references list across PHASE-1 to PHASE-4 is
the canonical citation set for the WIA-
electronic-skin standard. Implementations cite
the standards by their issuing organisation
(IEC, ISO, IEEE, HL7, IETF, W3C, EU regulatory
text, US FDA regulatory text, KS) and the
publication year so that a downstream consumer
can locate the authoritative text. Updates to a
cited standard (for example, an amendment to ISO
10993-23) trigger an internal review cycle in
the operator's quality-management discipline
declared in PHASE-3 §11 before the new revision
is bound into the operator's enumeration set.

## §11 Clinical-Investigation Sponsor Integration

A sponsor running a clinical investigation under
EU MDR Annex XV (or the equivalent US FDA IDE
regulation under 21 CFR Part 812) integrates
with the operator's API by registering the
investigation identifier (the EU MDR investigation
number, the FDA IDE number, the KR 의료기기
임상시험 계획 승인 번호), the institutional
review-board approval reference, and the
investigation-protocol identifier. The operator's
API gates the per-patient observation upload on
the patient's enrolment in the registered
investigation and on the patient's investigation-
specific consent.

## §12 Adverse-Event Cross-Border Federation

A device sold across multiple jurisdictions
generates adverse-event reports that satisfy the
reporting requirements of each market. The
operator's API publishes a per-event report
package that includes the EU MDR vigilance
report, the US FDA MDR submission, and the KR
부작용 보고 in the machine-readable form that
each regulator's intake system accepts. The
operator's audit envelope records each
acknowledgement so that the regulator audit
trail is preserved.

## §13 KR PIPA Cross-Border-Transfer Discipline

Where a KR-jurisdiction patient's body-borne
sensor data is transferred to an analytics
service in another jurisdiction, the operator's
API binds the transfer to the KR PIPA Article
17(3) cross-border-transfer notice and consent.
The transfer is recorded in the chain-of-custody
record, and the operator publishes the per-
transfer privacy notice that names the recipient,
the recipient's privacy framework, the purpose
of the transfer, and the retention period in the
recipient's jurisdiction.

## §14 Verifiable Credentials for Compliance Re-
       Issuance

The operator publishes a verifiable-credentials
bundle covering the device's CE marking, the FDA
clearance reference, the ISO 13485 QMS
certificate, and the most recent ISO 10993
biocompatibility attestation. The bundle is
signed under W3C Verifiable Credentials Data
Model 2.0 with the operator's public-key set so
that a downstream healthcare provider, hospital
procurement office, or insurance reimbursement
system can validate the compliance package
without bilateral integration with the operator
or with each issuing authority.
