# WIA-pet-cloning PHASE 3 — PROTOCOL Specification

**Standard:** WIA-pet-cloning
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern the operating procedure of
an accredited pet-cloning programme: laboratory accreditation, regulatory
authorisation, programme registration, owner consent, animal welfare review,
incident reporting, and inter-laboratory transfer of biological material.
The PROTOCOL layer binds the data shapes of PHASE-1 and the API contract of
PHASE-2 to the regulatory and ethical context in which programmes actually
operate.

References (CITATION-POLICY ALLOW only):

- ISO/IEC 17025:2017 (general requirements for testing and calibration laboratories)
- ISO/IEC 17065:2012 (conformity assessment — bodies certifying products, processes and services)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 27701:2019 (privacy information management extension to ISO/IEC 27001)
- ISO 9001:2015 (quality management systems)
- ISO 8601 (date and time)
- ISO 11784 / 11785 (animal RFID)
- IETF RFC 9457 (Problem Details for HTTP APIs)
- IETF RFC 4122 (UUID)
- IETF RFC 5905 (NTPv4)
- IETF RFC 8446 (TLS 1.3)

---

## §1 Programme Accreditation

A pet-cloning operator MAY claim conformance to WIA-pet-cloning only after a
recognised accreditation body has issued a valid certificate against
ISO/IEC 17025:2017 for the laboratory scopes that the programme exercises
(cell culture, oocyte handling, electrofusion, embryo culture, and
cryopreservation). Each scope is a distinct accreditation line item; a
programme that contracts out cryopreservation to a third-party biobank MUST
ensure that the biobank's accreditation covers the relevant scope.

The accreditation register is exposed to the API as a read-only resource and
MUST be re-fetched at least once per calendar quarter. A revoked or expired
accreditation transitions all of the programme's open cases from `active`
to `ceased` automatically; cases already in `transferred` or `verified`
state remain unchanged.

## §2 Programme Registration

The programme code (`caseProgramme` in PHASE-1 §2) is assigned by the WIA
secretariat upon successful accreditation. The code follows the pattern
`WIA-PC-{COUNTRY}-{NNNN}` where `COUNTRY` is the ISO 3166-1 alpha-2 code in
upper case and `NNNN` is a four-digit zero-padded sequence assigned in the
order applications are accepted. The code is permanent and is not reused
when a programme winds down.

A programme that operates from more than one site emits per-site sub-codes
(`WIA-PC-K-0012-A`, `WIA-PC-K-0012-B`) so that audit trails can be scoped
to the originating physical facility. Sub-codes share the parent's
accreditation record but maintain independent custody logs.

## §3 Regulatory Authorisation

Where national authority is required for somatic cloning of companion
animals (the requirement varies by jurisdiction), the authorisation
reference is recorded against the case at creation time. Implementations
MUST NOT advance a case beyond `draft` status without a regulatory
authorisation reference in jurisdictions that require one.

The authorisation reference is a free-form string opaque to this standard;
the operating programme is responsible for verifying that the reference is
valid against the issuing authority's register. WIA does not arbitrate
national rules.

For cross-species SCNT (PHASE-1 §10), the authorisation reference MUST be
present regardless of jurisdiction; WIA-pet-cloning declines to publish
verification reports for cross-species cases without authorisation, and
the API enforces this by returning `403 Forbidden` with a problem document
typed `urn:wia:pet-cloning:authorisation-required`.

## §4 Animal Welfare Review

Every case MUST be reviewed against an animal welfare framework prior to
oocyte donor selection or surrogate selection. The reviewing body is the
operating programme's institutional welfare committee or an equivalent
external committee recognised by the accreditation body.

The review record carries the committee's identifier, the date of review,
the reviewing veterinarian's professional registration number (held in the
operator's HR system, not exposed in the case record), and an outcome of
`approved`, `approved-with-conditions`, or `declined`. Cases marked
`declined` MUST be ceased; cases marked `approved-with-conditions` MUST
record the conditions in the case notes and verify their fulfilment before
oocyte donor selection.

A reasonable cap on surrogate-pregnancy attempts per donor pair, on oocyte
donor procedures per donor animal, and on embryo transfers per recipient
SHOULD be established by the welfare framework; this PHASE does not
prescribe specific numerical caps because veterinary practice varies.

## §5 Sample Handling Windows

Post-mortem tissue acquisition is bounded by species-specific viability
windows beyond which the recovered fibroblast cultures are unreliable as
donor sources. The recommended windows for routine clinical practice are:

- Felis catus: up to 5 days post-mortem under refrigeration.
- Canis familiaris: up to 5 days post-mortem under refrigeration.
- Other species: window is set per programme based on prior validation;
  validation evidence is recorded in the programme's quality dossier.

Implementations MUST refuse to register a sample whose `postMortemHours`
exceeds the species window. Programmes MAY define more conservative
windows; the standard requires the published window to be at least as
strict as the species default.

## §6 Inter-Laboratory Transfer

When a sample, oocyte, or embryo crosses programme boundaries (for example
a frozen sample shipped from a tissue bank to an embryology laboratory),
the receiving programme adopts the existing custody log and continues
appending events. The transfer event is recorded with both programmes'
codes; both programmes' API endpoints MUST publish the transfer event
within 72 hours of the physical handover.

Cross-border transfers additionally record the customs declaration
reference and the receiving country's import authorisation reference, both
of which are opaque to this standard but serve as audit anchors for
regulators. CITES-listed species MUST carry the appropriate CITES permit
identifier when applicable.

## §7 Records Retention

Programme records — every record defined in PHASE-1, the API audit logs
defined in PHASE-2 §12, and the regulatory and welfare review documents
defined in this PHASE — are retained for a minimum of seven calendar years
from the last access of the case, with the following exceptions:

- Verification reports: retained indefinitely.
- Live offspring health monitoring records: retained for at least the
  natural lifespan of the offspring, plus three years.
- Personal data carried in operator CRM systems: retained per the
  jurisdiction's data-protection law and the contract with the owner;
  the operator MAY purge personal data without affecting the integrity of
  the case records, since the case records carry only the opaque
  `ownerReference`.

Retention is enforced by the operator's records-management system and is
audited annually as part of ISO/IEC 27001:2022 surveillance audits.

## §8 Time Synchronisation

Programmes operate on synchronised time per RFC 5905 (NTPv4) so that
custody event ordering across instruments and across programme sites is
consistent. The reference time source is the national metrological
laboratory's stratum-1 service or an equivalent recognised service; the
operator records the source and the maximum observed offset in the
programme's quality dossier.

## §9 Witness Laboratories for Verification

Post-natal genetic verification reports (PHASE-2 §7) carry signatures from
at least one witness laboratory in addition to the issuing reference
laboratory. The witness laboratory MUST be operated under independent
ownership and accredited under ISO/IEC 17025 for the relevant DNA-based
identity-confirmation scope.

The witness laboratory receives a sealed aliquot of donor and offspring
genomic DNA, performs the comparison independently, and signs the report
upon agreement. Disagreement triggers a third-laboratory adjudication and
the case retains `transferred` status until adjudication concludes. The
adjudicating laboratory's report is appended to the verification record
and the case advances to `verified` only if the adjudication concurs with
identity.

## §10 Incident Reporting

Adverse events — recipient mortality, severe dystocia, surgical
complications, abnormal phenotype at parturition, instrument failures,
and security incidents that affect any record — are reported to the
accreditation body within 30 calendar days. The incident report uses the
problem-document format defined in PHASE-2 §10 with type
`urn:wia:pet-cloning:incident-report` and is appended to the case.

Anonymised incident statistics are aggregated by the accreditation body
and published quarterly so that the broader programme community can
calibrate practice; raw incident reports remain confidential to the
reporting programme, the accreditation body, and the regulator.

## §11 Programme Wind-Down

A programme that ceases operations transitions all open cases to `ceased`,
transfers cryopreserved samples to a successor programme or to long-term
custody at a recognised biobank, and notifies all donors of record via the
operator's CRM. Cryopreserved samples without a documented successor
custodian are retained at the biobank for at least the seven-year records
retention window described in §7 before any disposition decision.

## §12 Welfare-Review Framework Reference

The welfare-review framework named in §4 is a written document maintained by
the operating programme that defines the composition of the welfare
committee, its quorum and decision-making process, the species-specific
clinical limits the committee applies (anaesthesia regimes, analgesia
protocols, surgical recovery housing), the triggers that escalate a case
to a full review, and the appeal procedure available to a programme
veterinarian whose case is declined. The framework is reviewed at least
annually by the programme's quality manager and is published on the
programme's well-known discovery URL (PHASE-4 §5) so that owners,
auditors, and the accreditation body can read it directly without making
a separate request.

A programme that adopts an externally-published framework verbatim records
the source document, its version, and the date of adoption. Local
amendments to an externally-published framework are clearly marked as such
and reviewed independently before the welfare committee operates against
them.

## §13 Cross-Border Programme Operation

A programme that operates across borders maintains a primary jurisdiction
of registration (the country of the WIA programme code) and one or more
operating jurisdictions where laboratory or clinical activity occurs. The
primary jurisdiction's accreditation body is the lead authority; operating
jurisdictions consume the lead authority's certifications under mutual
recognition where one exists. Where mutual recognition does not exist, the
operating jurisdiction issues its own accreditation and the programme
holds parallel certificates.

Cross-border data transfers honour the data-protection law of the donor's
country of residence, which is recorded in the operator CRM at consent
time. The case record never carries the donor's country directly; the CRM
applies the country-specific transfer rules when producing exports.

## §14 Conformance and Auditing

A programme conformant with WIA-pet-cloning publishes its accreditation
certificate, its programme code registration, its welfare-review framework,
and its quality dossier on a public landing page and answers an annual
self-assessment that maps each clause of this PHASE to the programme's
implementation. The self-assessment is reviewed during the annual
ISO/IEC 17025 surveillance audit by the accreditation body.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-pet-cloning
- **Last Updated:** 2026-04-27
