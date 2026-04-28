# WIA-digital-time-capsule PHASE 3 — PROTOCOL Specification

**Standard:** WIA-digital-time-capsule
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern
a digital-preservation operator across the
depositor-to-repository-to-future-consumer
value chain: the OAIS Information Package
discipline that anchors every artefact to the
ISO 14721 reference model, the PREMIS metadata
discipline that records every preservation
event, the format-identification-and-validation
discipline that classifies every file against
the published format register, the BagIt
transfer discipline that gates inter-repository
package transfer, the WARC discipline that
governs web-archive ingestion, the trustworthy-
digital-repository discipline that anchors the
operator's accreditation to ISO 16363 / ISO
16919, the seal-and-release discipline that
governs the per-capsule opening condition, the
fixity-and-integrity discipline that detects
silent bit-rot across decades, the legal
admissibility discipline that aligns with EU
eIDAS-2 / KR 전자문서법, the chain-of-custody
anchoring discipline, and the post-audit
remediation discipline.

References (CITATION-POLICY ALLOW only):

- ISO 14721:2012 (OAIS) and its terminology
  vocabulary in §1
- ISO 16363:2012 (audit and certification of
  trustworthy digital repositories)
- ISO 16919:2020 (audit certification body
  requirements)
- ISO 14641:2018 (electronic archiving
  specifications)
- PREMIS Data Dictionary v3.0 (and PREMIS
  ontology)
- ISO 28500:2017 (WARC), ISO 19005-1/-2/-3/-4
  (PDF/A), ISO 32000-2:2020 (PDF 2.0)
- ISO 15836-1:2017 / 15836-2:2019
- METS / MODS / DDI 4 (research-data DDI
  Lifecycle)
- IIPC WARC and CDX format guidelines
- ISO 9001:2015 (QMS) and ISO/IEC 27001:2022
  (ISMS)
- ISO/IEC 17021-1:2015 / ISO/IEC 17065:2012
- IETF RFC 9110, RFC 6234 (SHA-256), RFC 8032
  (Ed25519), RFC 8493 (BagIt), RFC 9421 (HTTP
  Message Signatures), RFC 9457 (Problem
  Details), RFC 8615 (well-known URIs), RFC
  6962 (Certificate Transparency, the per-event
  log template)
- W3C Trace Context, W3C ODRL 2.2, W3C VC Data
  Model v2.0
- EU Regulation (EU) 910/2014 (eIDAS) and EU
  Regulation (EU) 2024/1183 (eIDAS-2)
- EU Regulation (EU) 2016/679 (GDPR) Articles
  6, 9, 89
- KR 공공기록물 관리에 관한 법률, KR 전자문서
  및 전자거래 기본법, KR 정보공개법

---

## §1 OAIS Information Package Discipline

### §1.1 Mandatory Properties

Every package carried by the operator's API
declares the OAIS Mandatory Properties per
ISO 14721 §4.2.1.3 — Reference Information,
Provenance Information, Context Information,
Fixity Information, and Access Rights
Information. The operator's API rejects a
package omitting any Mandatory Property at
ingestion time and returns the field's JSON
Pointer in the RFC 9457 problem document.

### §1.2 SIP-to-AIP transformation

The SIP-to-AIP transformation per ISO 14721
§4.1.1.4 is performed inside the operator's
AIP-generation workflow. The transformation
runs the per-event PREMIS Event records (a
ValidationOK event, a FixityCheckOK event, a
FormatIdentificationOK event, a VirusCheckOK
event) and binds the events to the AIP's
Provenance Information.

### §1.3 AIP-to-DIP packaging

The AIP-to-DIP packaging per ISO 14721 §4.1.1.5
adapts the AIP to the consumer's request
envelope. The DIP carries the same Content
Information as the AIP but with the
Representation Information adapted to the
consumer's requested rendering format
(HTML, PDF/A, plain-text-UTF-8, image
thumbnail, video preview).

## §2 PREMIS Metadata Discipline

### §2.1 Per-event recording

Every preservation event is recorded as a PREMIS
Event entity with `eventType` from the PREMIS
Event Type Vocabulary (the Library of Congress-
maintained vocabulary). The operator's API
rejects an event with an unknown `eventType` at
publication time.

### §2.2 Per-object characteristics

The PREMIS Object's `objectCharacteristics`
carries the `size`, the `fixity` (SHA-256 hex
digest per RFC 6234), and the `format`
identifier from the format register declared in
PHASE-1 §5.

### §2.3 Per-rights envelope

The PREMIS Rights envelope binds the rights
basis (statutory, license, donor agreement)
to the package. The operator's API enforces
that the rights-basis declaration is consistent
with the operator's declared statutory or
licensed processing basis.

## §3 Format-Identification-and-Validation Discipline

### §3.1 Format register binding

Every file's format identifier is bound to the
Library of Congress Sustainability of Digital
Formats register identifier or the PRONOM PUID.
A file whose format identifier is not in either
register is annotated as `format-unidentified`
and triggers a manual review by the
preservation engineer.

### §3.2 Validation outcome enforcement

Every file's validation outcome (well-formed-
and-valid, well-formed-not-valid, not-well-
formed) is recorded against the per-format
validator's report — the JHOVE module outcome,
the veraPDF outcome for PDF/A, the FFV1
framemd5 outcome for AV files. A `not-well-
formed` outcome triggers the operator's
remediation workflow (re-ingestion from the
depositor, file repair, per-format migration).

### §3.3 Preservation-format migration

The operator periodically migrates files into a
preservation format (PDF/A, FLAC, FFV1-MKV,
WARC) per the operator's documented migration
plan. Each migration is recorded as a PREMIS
Event with `eventType: migration`.

## §4 BagIt Transfer Discipline

### §4.1 Manifest digest verification

Every BagIt package's manifest is verified
against the recomputed digest (SHA-256 or
SHA-512 per RFC 6234) at ingestion time. A
mismatch returns `422 Unprocessable Entity` at
`/problems/bagit-manifest-digest-mismatch`.

### §4.2 Tagmanifest discipline

Where the BagIt package carries a
`tagmanifest-{algorithm}.txt`, the operator
verifies the tagmanifest digest against the
`bag-info.txt`, the `bagit.txt`, and the
manifest file. A tagmanifest mismatch is
recorded as a chain-of-custody event.

### §4.3 BagIt signature

Where the package crosses an EU jurisdiction,
the BagIt package carries a qualified-
electronic-signature per EU eIDAS-2 over the
manifest digest. The operator's API verifies
the qualified signature against the issuing
qualified trust service provider's certificate.

## §5 WARC Discipline

### §5.1 Per-record validation

Every WARC record is validated against the
ISO 28500:2017 WARC schema. The operator's API
refuses a record with a missing `WARC-Type`,
`WARC-Record-ID`, `WARC-Date`, or
`Content-Length` header.

### §5.2 CDX index generation

The operator generates the per-WARC CDX index
per the IIPC CDX format guideline so that a
downstream consumer can locate WARC records
by URL, MIME type, status code, or date.

## §6 Trustworthy-Digital-Repository Discipline

### §6.1 ISO 16363 audit binding

The operator's API binds every audit record to
the ISO 16363:2012 criteria. A `certified`
outcome carries the per-criterion findings; a
`conditional` outcome lists the conditions
that the operator must satisfy before the next
audit cycle.

### §6.2 ISO 16919 audit body accreditation

The audit body's ISO 16919:2020 accreditation
is verified at audit-record ingestion time. A
withdrawn or expired accreditation refuses the
audit-record publication.

## §7 Capsule-Seal-and-Release Discipline

### §7.1 Seal envelope

The operator's API records the seal envelope
declared in the seal request — the depositor
identity, the named beneficiary (which may be
"the public at the declared date", an
individual identified by W3C Verifiable
Credential, or a group identified by an
operator-defined attribute), the declared
opening date, the release-condition envelope,
and the rights-expression for post-opening
re-use.

### §7.2 Time-trigger discipline

A `time-trigger` capsule is opened on or after
the declared opening date. The operator's API
runs a daily cron job that scans the seal
register and triggers the opening event for
capsules whose opening date has been reached.

### §7.3 Beneficiary-trigger discipline

A `beneficiary-trigger` capsule is opened on
the named beneficiary's authenticated request.
The beneficiary's identity is verified against
the W3C Verifiable Credential declared in the
seal envelope.

### §7.4 Event-trigger discipline

An `event-trigger` capsule is opened on the
declared trigger event (for example, the
depositor's death verified by a national-
register death certificate, the closure of a
named institution verified by the institution's
public dissolution notice).

## §8 Fixity-and-Integrity Discipline

### §8.1 Per-cycle fixity check

The operator runs a periodic fixity check on
every package per the operator's declared
cycle (typically annual for AIPs at storage
tier 1, monthly for storage tier 2 with
cheaper media). The check recomputes the
SHA-256 digest of every file and compares it
with the recorded digest in the PREMIS Object.

### §8.2 Bit-rot detection and remediation

A fixity mismatch triggers the operator's bit-
rot remediation workflow. The package is
restored from a replicated copy held in a
separate storage tier, the affected file is
re-ingested from the depositor where the
depositor copy is preserved, or the file is
recreated from a documented re-derivation
recipe.

### §8.3 Replication discipline

The operator maintains a documented number of
replicated copies (typically three copies
across two geographic regions and two
storage technologies — disk + tape, or disk +
cloud-cold-storage) per the LOCKSS principle
that the operator declares in the programme
record.

## §9 Legal-Admissibility Discipline

### §9.1 EU eIDAS-2 qualified seal

Where the operator targets legal-admissibility
in an EU jurisdiction, the operator applies a
qualified-electronic-signature seal per EU
Regulation (EU) 2024/1183 to every AIP's
manifest. The qualified seal is verifiable
under the EU Trusted Lists scheme.

### §9.2 KR 전자문서법 binding

A KR-jurisdiction operator binds the AIP's
manifest to the KR 전자문서 및 전자거래 기본법
(Framework Act on Electronic Documents and
Electronic Transactions) §5 evidentiary value
discipline so that the AIP is admissible in
KR civil and administrative proceedings.

## §10 Chain-of-Custody Anchoring Discipline

### §10.1 Per-event transparency log

Every chain-of-custody event carried by PHASE-1
§8 is appended to a per-operator transparency
log modelled on the IETF RFC 6962 Certificate
Transparency append-only-log structure.

### §10.2 Mutation prevention

A custody event cannot be retroactively edited;
an amendment is recorded as a new event with
`previousEventRef` pointing at the event being
amended.

## §11 Quality-Management Discipline

The operator runs an ISO 9001:2015 quality
management system covering the ingestion,
validation, format-identification, fixity-
check, replication, audit, seal, and release
processes. The operator's information-security
management system declared under ISO/IEC
27001:2022 covers the protection of the
preserved data and the operator's signing keys.

## §12 Post-Audit Remediation Discipline

### §12.1 Conditional-audit closure

An audit outcome of `conditional` triggers
the operator's remediation workflow against
each declared condition. The closure is
gated on a re-audit by the same audit body or
an equivalent body.

### §12.2 Non-certified withdrawal

An audit outcome of `non-certified` triggers
a public withdrawal of the operator's
trustworthy-digital-repository attestation.
The operator's API publishes the withdrawal
notice on the public retrieval endpoint and
the chain-of-custody record.
