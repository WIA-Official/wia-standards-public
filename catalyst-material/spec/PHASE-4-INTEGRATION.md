# WIA-catalyst-material PHASE 4 — INTEGRATION Specification

**Standard:** WIA-catalyst-material
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how a catalyst-research
operator integrates with the systems that surround
catalyst registration, characterisation, certified-
reference-material publication, and industrial
qualification: the accreditation body issuing the
ISO 17034 RM-producer or the ISO/IEC 17025 testing-
laboratory certificate; the Joint Committee for
Guides in Metrology (JCGM) host of the GUM
uncertainty-of-measurement vocabulary; the IUPAC
secretariat publishing the catalysis-nomenclature
manual; the industrial-catalyst manufacturer's
qualification-batch register; the process licensor's
licensee telemetry feed; the EU REACH and CLP
regulatory authority and the catalyst-substance
supply-chain communication endpoints; and the
proficiency-testing organiser running ISO 5725 or
ISO 13528 rounds.

References (CITATION-POLICY ALLOW only):

- ISO 17034:2016 (RM-producer competence)
- ISO/IEC 17025:2017 (testing and calibration
  laboratory competence)
- ISO/IEC 17021-1:2015 (audit and certification)
- ISO/IEC 17065:2012 (conformity-assessment bodies
  certifying products, processes, and services)
- ISO/IEC 17000:2020 (conformity-assessment
  vocabulary)
- ISO Guide 30:2015, ISO Guide 31:2015, ISO Guide
  33:2015, ISO Guide 35:2017
- ISO 5725 series (1–6) and ISO 13528:2022
- ISO 9001:2015 (quality management systems)
- ISO/IEC 27001:2022 (information-security
  management) and ISO/IEC 17021-1:2015 (management-
  system audit)
- IUPAC Recommendations 2007 — Manual of Methods and
  Procedures for Catalyst Characterization
- IUPAC Recommendations on Catalysis Nomenclature
  (Pure Appl. Chem. 73, 2001, 1227–1241)
- IUPAC Compendium of Chemical Terminology (the
  "Gold Book") and IUPAC Quantities, Units and
  Symbols in Physical Chemistry (the "Green Book")
- JCGM 100:2008 (Evaluation of measurement data —
  Guide to the expression of uncertainty in
  measurement, the GUM) and JCGM 200:2012 (the
  International Vocabulary of Metrology, the VIM)
- BIPM SI Brochure (the 9th edition definition of
  the SI base and derived units cited in the unit-
  symbol enforcement of PHASE-2 §5)
- IETF RFC 8259 / 9457 / 8615 / 9421 / 6962
- W3C Verifiable Credentials Data Model 2.0
  (optional, used for the re-issuance of an
  accreditation reference into a customer's
  laboratory-information-management system)
- EU REACH Regulation (EC) No 1907/2006 (Articles
  10, 14, 31 — registration dossier, chemical
  safety report, supply-chain communication via
  the Safety Data Sheet)
- EU CLP Regulation (EC) No 1272/2008 (Articles
  4–18, Annex I — substance and mixture
  classification rules)
- EU Construction Products Regulation (EU)
  305/2011 (where the catalyst is sold as a
  construction-product additive)
- KS Q ISO 17034 (Korean adoption of ISO 17034 —
  used for the KR-jurisdiction RM-producer
  accreditation register integration)

---

## §1 Accreditation-Body Integration

### §1.1 ISO 17034 register

The operator's API binds every certificate-issuance
record to the ISO 17034 accreditation reference
declared in the programme record. The accreditation
body's published register is queried at certificate-
publication time; the response is cached for the
TTL declared in the accreditation body's HTTP
caching headers (RFC 9111). A withdrawn or
suspended reference returns `410 Gone` for the
operator's `/v1/crm-certificates` POST endpoint and
`200 OK` with a `withdrawn: true` flag for the
public retrieval endpoint, so that downstream
consumers can detect the certificate state without
having to query the accreditation body directly.

### §1.2 ISO/IEC 17025 register

Every characterisation-upload signature carried by
the laboratory is verified against the ISO/IEC
17025 register on the accreditation body's
endpoint. The register-lookup result is recorded as
part of the upload's audit envelope for later
review by the operator's quality manager.

## §2 IUPAC Vocabulary Integration

The operator's API binds the controlled vocabularies
declared in PHASE-1 §3 (`catalystClass`), §4
(`technique`), and §7 (`outlierTreatment`) to the
IUPAC 2001 catalysis-nomenclature manual, the IUPAC
2007 characterisation manual, and the IUPAC
Quantities-Units-Symbols Green Book respectively.
Vocabulary updates are published by IUPAC on a
multi-year cycle; the operator subscribes to the
IUPAC publishing endpoint and triggers an internal
review cycle when a vocabulary update is published.
A vocabulary entry that is deprecated by IUPAC is
marked deprecated in the operator's enumeration set
but is not removed for the duration of the
operator's record-retention period.

## §3 Industrial-Manufacturer Qualification Integration

### §3.1 Qualification-batch register

A catalyst manufacturer's qualification-batch
register integrates with the operator's API by
publishing a per-batch attestation envelope that
links the laboratory characterisation record to the
manufacturing batch identifier. The attestation
carries the laboratory's ISO/IEC 17025
accreditation reference, the catalyst record's
identifier, the manufacturing batch identifier, and
the per-batch characterisation summary so that the
licensee operating the catalyst in a production
unit can validate the batch against the
qualification baseline before commissioning.

### §3.2 Production-unit telemetry binding

The licensee's production-unit telemetry feed
publishes time-on-stream conversion and selectivity
data points at the cadence declared in the
performance record's `deactivationCurve`. The
operator's API ingests the telemetry feed into the
performance record and updates the
`deactivationStatus` field in the licensee's
namespace. Where the slope exceeds the qualification
envelope, the licensee receives a notification at
the channel declared in their programme record's
contact directives.

## §4 Proficiency-Testing Organiser Integration

A proficiency-testing organiser running ISO 13528
rounds integrates with the operator's API as both a
producer (publishing the round envelope to the
participant set) and a consumer (ingesting the
participant submissions). The organiser's
accreditation reference is verified at round
opening; the organiser's signed assigned-value
declaration is bound to the round identifier and is
not editable after round closure.

## §5 Substance-Registration Integration

### §5.1 EU REACH dossier integration

Where the catalyst contains a substance whose REACH
registration dossier is held by the operator or a
member of its consortium, the operator publishes a
link from the catalyst record's
`identifierBindings.reachRegistrationNumber` to the
dossier-publication endpoint operated by the
European Chemicals Agency. The operator's API does
not redistribute the dossier itself; the link is
informational and is rendered by the public
retrieval endpoint as a clickable reference.

### §5.2 CLP supply-chain communication

The operator's API publishes the CLP hazard-class
declaration in a Safety Data Sheet envelope per
REACH Article 31 and CLP Regulation Article 17.
The SDS envelope is signed using the operator's
public-key set and is retrievable at
`/v1/catalysts/{catalystId}/sds`.

## §6 KR-Jurisdiction Integration

### §6.1 KS Q ISO 17034 adoption

A KR-jurisdiction RM-producer operator declares the
KS Q ISO 17034 adoption reference in the programme
record's `accreditationStatus`. The KR-Korea
Laboratory Accreditation Scheme (KOLAS) operates
the accreditation register; the operator's API
queries the KOLAS register on certificate-
publication and on each retrieval after the
caching TTL.

### §6.2 KR Chemical Substances Control Act linkage

A KR-jurisdiction catalyst whose composition
includes a substance controlled under the KR
Chemical Substances Control Act (화학물질관리법)
declares the act's substance reference in the
catalyst record's `identifierBindings`. The
operator's API does not gate registration on the
act's review status — the registration is informational
and is reviewed by the operator's regulatory-affairs
team under the operator's quality-management
discipline declared in PHASE-3 §8.

## §7 Public Retrieval and Re-Issuance

### §7.1 Public catalyst-record retrieval

A public consumer (a licensee evaluating a
qualification batch, a regulator running a market-
surveillance audit, an academic researcher
referencing a CRM in a publication) retrieves the
catalyst record at `/v1/catalysts/{catalystId}`
without authentication; the response carries the
public fields and elides the per-laboratory raw-data
URIs unless the consumer presents an authorisation
issued by the laboratory.

### §7.2 Verifiable-credentials re-issuance

An accreditation reference can be re-issued into a
laboratory-information-management system as a
W3C Verifiable Credential. The credential carries
the accreditation body's issuer identifier, the
accreditation reference, the scope of the
accreditation, the certificate's expiry, and the
issuing date; the credential is signed using the
accreditation body's public-key set so that a
downstream verifier can validate the credential
without contacting the accreditation body
directly.

## §8 Audit and Conformity-Assessment Integration

### §8.1 ISO/IEC 17021-1 management-system audit

The operator's quality-management system declared in
PHASE-3 §8 is audited under ISO/IEC 17021-1 by an
accredited certification body. The audit result is
stored in the operator's audit envelope and is
referenced from the programme record's
`accreditationStatus`. The certification body's
public-key set is published at the certification
body's endpoint; the operator's API verifies the
audit certificate's signature against the
certification body's public key on retrieval.

### §8.2 ISO/IEC 17065 product certification

Where a catalyst is sold as a certified product
(for example a CRM), the operator's CRM
certificate is itself a product-certification
deliverable under ISO/IEC 17065. The certification
body's marking, scope of certification, and the
certified product's identifier are published in the
certificate envelope so that a downstream consumer
can verify the certification scope before relying
on the certificate.

## §9 References (consolidated)

The references list across PHASE-1 to PHASE-4 is
the canonical citation set for the WIA-catalyst-
material standard. Implementations cite the
standards by their issuing organisation (ISO, IEC,
IUPAC, JCGM, BIPM, ASTM International, IETF, W3C,
EU regulatory text, KS) and the publication year so
that a downstream consumer can locate the
authoritative text. Updates to a cited standard
(for example, the 2023 revision of ISO 5725-1)
trigger an internal review cycle in the operator's
quality-management discipline declared in PHASE-3
§8 before the new revision is bound into the
operator's enumeration set.
