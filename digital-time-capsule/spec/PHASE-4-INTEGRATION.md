# WIA-digital-time-capsule PHASE 4 — INTEGRATION Specification

**Standard:** WIA-digital-time-capsule
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how a digital-preservation
operator integrates with the systems that
surround long-term preservation: the audit
certification body issuing the ISO 16363
attestation; the format-register operator
maintaining the Library of Congress
Sustainability of Digital Formats register or
the PRONOM register; the ISO/IEC 17025-
accredited validation-tool service running a
JHOVE / veraPDF / FFV1-framemd5 sweep; the
EU qualified trust service provider issuing
the qualified-electronic-signature certificate
under EU eIDAS-2; the IIPC member network
exchanging WARC files; the LOCKSS or DPN
network operating the cross-institutional
replication; the supervisory data-protection
authority overseeing GDPR Article 89 archival
processing; and the public-procurement
authority running a digital-preservation
programme.

References (CITATION-POLICY ALLOW only):

- ISO 14721:2012 (OAIS)
- ISO 16363:2012, ISO 16919:2020
- ISO 14641:2018
- PREMIS v3.0
- ISO 28500:2017 (WARC)
- ISO 19005-1/-2/-3/-4 (PDF/A) and ISO 32000-2:
  2020 (PDF 2.0)
- ISO 15836-1/-2, METS, MODS
- IIPC WARC and CDX guidelines
- ISO 9001:2015 (QMS), ISO/IEC 27001:2022 (ISMS)
- ISO/IEC 17021-1:2015, ISO/IEC 17065:2012,
  ISO/IEC 17025:2017
- IETF RFC 8259, RFC 9457, RFC 8615, RFC 9421,
  RFC 6234, RFC 8032, RFC 8493, RFC 6962
- W3C Trace Context, W3C VC Data Model v2.0,
  W3C ODRL 2.2
- EU Regulation (EU) 910/2014 (eIDAS) and EU
  Regulation (EU) 2024/1183 (eIDAS-2) — and the
  EU Trusted Lists Implementing Act
- EU GDPR Articles 6, 9, 89
- KR 공공기록물 관리에 관한 법률, KR 전자문서법
- LOCKSS Programme published by Stanford
  University Libraries (the cross-institutional
  replication framework cited where the
  operator participates in a LOCKSS network)
- Digital Preservation Network (DPN) published
  governance framework

---

## §1 Audit-Certification-Body Integration

### §1.1 ISO 16919 register query

Every audit record's signature is verified
against the issuing accreditation body's
ISO 16919:2020 register. The accreditation
body's published register is queried at audit-
record ingestion time and on each retrieval
after the caching TTL.

### §1.2 ISO 16363 audit cycle

The operator's TDR audit cycle (typically
every three to five years) binds the operator
to the audit body for the cycle's duration.
The operator publishes the cycle's report on
the public-portal so that a downstream
consumer can rely on the attestation.

## §2 Format-Register Integration

### §2.1 Library of Congress Sustainability of
       Digital Formats register

The operator subscribes to the Library of
Congress's published format-register endpoint
and triggers an internal review cycle when a
format is added, deprecated, or has its
sustainability profile updated. A format
deprecated by the Library of Congress is
marked deprecated in the operator's enumeration
set but is not removed for the duration of the
operator's record-retention period.

### §2.2 PRONOM register

The operator additionally binds the per-format
identifier to the PRONOM PUID maintained by
The National Archives (UK). The PRONOM register
is queried on each retrieval after the caching
TTL.

## §3 Validation-Tool Integration

### §3.1 JHOVE module integration

The operator runs JHOVE for per-file format
validation. The JHOVE module's version is
recorded in the validation report so that a
downstream auditor can reproduce the
validation outcome.

### §3.2 veraPDF integration for PDF/A

The operator runs veraPDF for PDF/A validation.
The veraPDF policy profile (PDF/A-1a, -1b,
-2a, -2b, -2u, -3a, -3b, -3u, -4) is bound to
the operator's declared preservation format.

### §3.3 FFV1-framemd5 integration for AV

The operator runs the FFV1-framemd5 utility
for AV file fixity. The framemd5 digest is
bound to the per-frame fixity envelope so that
a partial-file restore can detect the
corrupted frames.

## §4 EU Trusted Trust Service Provider Integration

### §4.1 eIDAS-2 qualified seal issuance

The operator's qualified-electronic-signature
seal is issued by an EU-jurisdiction qualified
trust service provider (QTSP). The QTSP's
certificate is published on the EU Trusted
Lists; the operator's API verifies the seal
against the EU Trusted Lists at retrieval
time.

### §4.2 Long-term validation (LTV) discipline

The qualified seal is stored with embedded
revocation information per the EU eIDAS-2
Long-Term Validation profile so that the seal
remains verifiable after the issuing QTSP's
certificate has expired.

## §5 IIPC Member Network Integration

The IIPC (International Internet Preservation
Consortium) member network exchanges WARC
files between member archives. The operator
publishes the per-collection WARC list and
the per-WARC CDX index through the IIPC's
shared discovery endpoint so that other
member archives can co-host or co-replicate
the collection.

## §6 LOCKSS / DPN Network Integration

### §6.1 LOCKSS-network replication

A LOCKSS-network member operator contributes
the AIP set to the LOCKSS replicated cache.
The LOCKSS daemon performs periodic peer-
verification of the cached AIPs and reports
fixity divergence to the operator.

### §6.2 Cross-institutional dark archive

Where the operator participates in a cross-
institutional dark archive, the AIP is
replicated to each participating institution
under the dark-archive's governance framework.
A trigger event (the operator's institutional
closure, a regulatory withdrawal of the
operator's licence) opens the dark archive
to the surviving institutions per the
governance framework.

## §7 Supervisory Data-Protection Authority Integration

### §7.1 GDPR Article 89 archival processing

The supervisory data-protection authority
overseeing the operator's GDPR Article 89
archival processing audits the operator's
records of processing activities (Article 30)
on demand. The operator's API publishes the
records to the authority's endpoint.

### §7.2 GDPR Article 17 right-of-erasure
       interaction

A subject exercising the GDPR Article 17 right
of erasure on a record held in the archive is
handled per the operator's documented
balancing of Article 17(3)(d) — processing
necessary for archiving in the public interest
— against the subject's erasure interest. The
operator publishes the balancing decision in
the chain-of-custody record.

## §8 Public Retrieval and Re-Issuance

### §8.1 Public DIP retrieval

A public consumer (a researcher, an educator,
a journalist, the future curator) retrieves
the DIP at `/v1/oais-packages/{packageId}/dip`
without authentication where the package's
rights expression permits public retrieval.

### §8.2 Verifiable-credentials re-issuance

The TDR audit attestation is re-issuable as a
W3C Verifiable Credential signed by the audit
body's public-key set so that a downstream
consumer can validate the attestation without
contacting the audit body directly.

## §9 KR-Jurisdiction Integration

### §9.1 KR 국가기록원 register integration

A KR-jurisdiction operator binds the AIP
register to the KR National Archives' union
catalogue. The KR Ministry of the Interior
and Safety operates the register; the
operator's API queries the register on each
retrieval after the caching TTL.

### §9.2 KR 공공기록물법 binding

The operator declares the KR 공공기록물 관리에
관한 법률 (Public Records Management Act)
reference in the programme record's
`governingFrameworks` set where the operator
is in scope of the act.

### §9.3 KR 전자문서법 evidentiary binding

The operator's AIP signed under the KR 전자문서
및 전자거래 기본법 §5 discipline is admissible
in KR civil and administrative proceedings.
The signed AIP carries the KR Time-Stamping
Authority timestamp so that the per-AIP
signing time is preserved.

## §10 Audit and Conformity-Assessment Integration

### §10.1 ISO/IEC 17021-1 management-system audit

The operator's quality-management system
declared in PHASE-3 §11 is audited under
ISO/IEC 17021-1 by an accredited certification
body. The audit result is stored in the
operator's audit envelope.

### §10.2 ISO/IEC 17065 product certification

Where the operator's per-package attestation
(the per-AIP digital seal, the per-WARC
attestation) is issued under a product-
certification scheme, the certification body's
ISO/IEC 17065:2012 accreditation is bound to
the certification reference.

### §10.3 ISO/IEC 17025 validation-tool
       laboratory accreditation

A validation-tool laboratory running JHOVE /
veraPDF / FFV1-framemd5 under an ISO/IEC
17025 accreditation publishes the per-tool
validation report under the accreditation's
scope. The operator's API verifies the tool
laboratory's accreditation at retrieval time.

## §11 Beneficiary-Identity Issuer Integration

### §11.1 W3C Verifiable Credentials issuer

A `beneficiary-trigger` capsule referencing a
W3C Verifiable Credential issuer binds the
beneficiary's identity to the issuer's
public-key set. The operator's API queries the
issuer's revocation endpoint at the capsule-
opening request; a revoked credential refuses
the opening request and records the refusal
as a chain-of-custody event.

### §11.2 National identity provider integration

For natural-person beneficiaries, the operator
typically binds the beneficiary identity to a
national identity provider (the EU eIDAS-2
European Digital Identity Wallet under EU
Regulation (EU) 2024/1183, the KR 본인확인
기관 issuing 본인확인서비스, the US state-
level digital-driver-licence trust framework).
The provider's authoritative directory is
queried at the opening request to verify the
beneficiary's continuing legal identity.

### §11.3 Issuer Trust Lists

The operator subscribes to the EU Trusted Lists
endpoint and to the relevant national identity
provider's certificate-revocation list to keep
the per-beneficiary credential verification
current. A scheduled refresh runs at the
cadence declared in the issuer's signed trust-
list profile (typically once per day) so that a
revoked beneficiary credential is detected
within one refresh cycle of the issuer's
revocation publication.

## §12 References (consolidated)

The references list across PHASE-1 to PHASE-4
is the canonical citation set for the WIA-
digital-time-capsule standard. Implementations
cite the standards by their issuing
organisation (ISO, IEC, IETF, W3C, Library of
Congress, IIPC, EU regulatory text, KR
regulatory text) and the publication year so
that a downstream consumer can locate the
authoritative text. Updates to a cited
standard (for example, an amendment to ISO
14721 or a new edition of the PREMIS Data
Dictionary) trigger an internal review cycle
in the operator's quality-management
discipline declared in PHASE-3 §11 before
the new revision is bound into the operator's
enumeration set.
