# WIA-digital-time-capsule PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-digital-time-capsule
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format
layer for WIA-digital-time-capsule. The
standard covers the persistent record shapes
that a digital-preservation operator (a
national archive operating an electronic-
records repository, a library digital
preservation department, a research-data
repository under the FAIR principles, a web
archive participating in the IIPC, an audio-
visual digital-preservation service, an
institutional repository for academic
publications, a personal digital-time-capsule
service curating individual or family digital
assets), an external auditor running a
trustworthy-digital-repository audit, a
custodial heir or beneficiary receiving a
sealed time-capsule, and a public-procurement
authority running a digital-preservation
programme maintain when ingesting a digital
artefact, generating preservation metadata,
performing format identification and validation,
sealing the capsule against a future opening
date, and tracking the per-capsule fixity-and-
integrity trail. Records are consumed by the
preservation engineer running the per-capsule
review cycle, by the auditor producing the
trustworthy-digital-repository attestation, by
the future researcher accessing the capsule's
contents at the declared opening date, and —
where the capsule is sealed against a private
beneficiary — by the named beneficiary at the
declared release event.

References (CITATION-POLICY ALLOW only):

- ISO 14721:2012 (Open Archival Information
  System — Reference Model, the OAIS framework
  cited normatively for the per-capsule
  Information Package envelope in §3 and §4)
- ISO 16363:2012 (Audit and certification of
  trustworthy digital repositories) — the
  audit framework cited normatively for the
  per-repository attestation envelope in §7
- ISO 16919:2020 (Requirements for bodies
  providing audit and certification of
  candidate trustworthy digital repositories)
  — the auditor-competence framework
- ISO 14641:2018 (Electronic archiving —
  Specifications concerning the design and
  operation of an information system for
  electronic information preservation)
- PREMIS Data Dictionary for Preservation
  Metadata version 3.0 (the Library of Congress-
  maintained data model cited normatively for
  the per-Object / per-Event / per-Agent /
  per-Rights envelope in §4)
- ISO 15836-1:2017 / 15836-2:2019 (Dublin Core
  Metadata Element Set, Parts 1 and 2)
- ISO 28500:2017 (WARC File Format) — the IIPC-
  maintained format cited normatively for the
  web-archive Information Package envelope
- IETF RFC 8493 (BagIt File Packaging Format
  v1.0) — the cross-repository transfer
  envelope cited normatively for the §6
  package
- ISO 19005-1:2005 / 19005-2:2011 / 19005-3:
  2012 / 19005-4:2020 (PDF/A — Long-term
  preservation, Parts 1 to 4) and ISO 32000-2:
  2020 (PDF 2.0)
- ISO 28560-1:2014 / 28560-2:2014 / 28560-3:
  2014 (RFID in libraries — cited where the
  capsule is materialised in a physical
  storage container)
- METS (Metadata Encoding and Transmission
  Standard) and MODS (Metadata Object
  Description Schema) maintained by the
  Library of Congress
- The JHOVE format-identification and
  validation tool — cited for the format-
  identification envelope (open-source tool;
  reference is to the published format
  signatures in the Library of Congress
  Sustainability of Digital Formats register)
- IIPC (International Internet Preservation
  Consortium) WARC format guidelines and
  CDX index format
- IETF RFC 8259 (JSON), RFC 4122 (UUID), ISO
  8601 (date-time)
- IETF RFC 6234 (US Secure Hash Algorithms —
  SHA-256 family) and IETF RFC 8032 (Edwards-
  Curve Digital Signature Algorithm — Ed25519)
- ISO/IEC 27001:2022 (information-security
  management — used for the chain-of-custody
  record discipline in §8)
- W3C Open Digital Rights Language (ODRL) 2.2
  — cited for the rights expression carried by
  every Information Package
- W3C Verifiable Credentials Data Model v2.0 —
  optional, cited for the re-issuance of audit
  attestations
- EU Regulation (EU) 910/2014 (eIDAS) and EU
  Regulation (EU) 2024/1183 (eIDAS-2) — cited
  for the qualified-electronic-signature seal
  on the time-capsule manifest
- KR 공공기록물 관리에 관한 법률 (Public
  Records Management Act) and KR 전자문서 및
  전자거래 기본법 (Framework Act on Electronic
  Documents and Electronic Transactions)

---

## §1 Scope

This PHASE defines persistent shapes for the
artefacts exchanged when a digital artefact is
ingested into a preservation repository,
described under the OAIS Information Package
discipline, packaged under BagIt for inter-
repository transfer, validated against the PDF/A
or other declared preservation format, sealed
against a future opening date, and reviewed
under the trustworthy-digital-repository audit
discipline. Implementations covered include:

- A national-archive electronic-records
  repository ingesting government-record
  series under the OAIS Submission Information
  Package discipline.
- A library digital-preservation department
  preserving digitised heritage materials and
  born-digital publications.
- A research-data repository preserving
  research-output datasets under the FAIR
  principles.
- A web archive participating in the IIPC
  capturing web pages as WARC files.
- An institutional repository preserving the
  output of an institution's research staff.
- A personal digital-time-capsule service
  curating individual or family digital
  assets sealed for a future generation.

The OAIS Submission Information Package, the
PREMIS preservation metadata envelope, the BagIt
transfer package, the WARC web-archive package,
and the trustworthy-digital-repository audit
record receive distinct encodings in this
PHASE; the additional safeguards required by
each preservation regime are encoded in
PHASE-3 §3.

## §2 Programme Identifier

```
programmeId          : string (uuidv7)
operatorName         : string (legal name of the
                       operator — national
                       archive, library, research-
                       data repository, web
                       archive, institutional
                       repository, or personal-
                       capsule service operator)
operatorRole         : enum ("national-archive" |
                       "library-preservation" |
                       "research-data-repository"
                       | "iipc-web-archive" |
                       "institutional-
                       repository" | "personal-
                       capsule-service" |
                       "audit-certification-body"
                       | "user-defined")
governingFrameworks  : array of enum ("ISO-14721-
                       OAIS" | "ISO-16363-TDR" |
                       "ISO-16919-TDR-AUDITOR" |
                       "ISO-14641-EARCH" |
                       "PREMIS-3" |
                       "ISO-15836-1" |
                       "ISO-15836-2" |
                       "ISO-28500-WARC" |
                       "BAGIT-RFC-8493" |
                       "ISO-19005-PDF-A-1" |
                       "ISO-19005-PDF-A-2" |
                       "ISO-19005-PDF-A-3" |
                       "ISO-19005-PDF-A-4" |
                       "ISO-32000-2-PDF-2" |
                       "METS" | "MODS" |
                       "W3C-ODRL" |
                       "EU-EIDAS" |
                       "EU-EIDAS-2" |
                       "KR-공공기록물법" |
                       "KR-전자문서법" |
                       "user-defined")
trustworthinessAudit : object (the ISO 16363
                       audit certificate, the
                       issuing audit body, the
                       certificate identifier,
                       and the certificate's
                       expiry date)
programmeStatus      : enum ("design" |
                       "operating" | "limited-
                       rollout" | "wind-down" |
                       "archived")
```

## §3 OAIS Information Package Record

```
oaisPackage:
  packageId          : string (uuidv7)
  packageType        : enum ("SIP-Submission" |
                       "AIP-Archival" |
                       "DIP-Dissemination" |
                       "user-defined")
  contentInformation : object (the OAIS Content
                       Information envelope —
                       the data object together
                       with the per-format
                       Representation Information
                       — Structure, Semantic,
                       Other — that allows the
                       consumer to interpret the
                       data object at the
                       opening date)
  preservationDescriptionInformation : object
                       (the OAIS Preservation
                       Description Information —
                       Reference Information,
                       Provenance Information,
                       Context Information,
                       Fixity Information, Access
                       Rights Information per
                       OAIS §4.2)
  packageDescription : object (the SIP / AIP /
                       DIP description envelope
                       suitable for OAIS
                       Discovery)
```

## §4 PREMIS Metadata Record

The PREMIS Data Dictionary v3.0 entity model
(Object, Event, Agent, Rights) is encoded as:

```
premisRecord:
  premisId           : string (uuidv7)
  packageRef         : string (PHASE-1 §3
                       record reference)
  objectEntity       : object (the PREMIS Object
                       — File, Bitstream, or
                       Representation; carrying
                       objectIdentifier,
                       objectCharacteristics
                       (size, fixity per RFC
                       6234, format identifier
                       from the format register),
                       preservationLevel, and
                       relationships to other
                       Objects)
  eventEntity        : array of object (per-
                       event PREMIS Event —
                       ingestion, validation,
                       fixity-check, format-
                       migration, virus-check,
                       replication, access; each
                       carrying eventIdentifier,
                       eventDateTime, eventType,
                       eventDetail, and
                       eventOutcome)
  agentEntity        : array of object (per-
                       agent PREMIS Agent —
                       depositor, preservation-
                       engineer, software, audit-
                       body; each carrying
                       agentIdentifier and
                       agentName)
  rightsEntity       : object (the PREMIS
                       Rights envelope —
                       rightsBasis (statutory,
                       license, donor agreement),
                       rightsGranted, rightsExtent)
```

## §5 Format Identification and Validation Record

```
formatRecord:
  formatRecordId     : string (uuidv7)
  packageRef         : string (PHASE-1 §3
                       record reference)
  formatIdentifier   : object (the per-file
                       format identification —
                       the Library of Congress
                       Sustainability of Digital
                       Formats register
                       identifier, the JHOVE
                       module name, the PRONOM
                       PUID where applicable)
  validationOutcome  : enum ("well-formed-and-
                       valid" | "well-formed-not-
                       valid" | "not-well-formed"
                       | "user-defined")
  validationReport   : object (the per-tool
                       validation report — the
                       JHOVE outcome envelope,
                       the veraPDF report for
                       PDF/A files, the FFV1
                       framemd5 for AV files)
  preservationFormat : enum ("PDF-A-1" |
                       "PDF-A-2" | "PDF-A-3" |
                       "PDF-A-4" | "PDF-2-0" |
                       "TIFF-baseline" | "JPEG-
                       2000-Part-1" |
                       "FLAC" | "WAV-BWF" |
                       "FFV1-MKV" | "WARC-1-1"
                       | "PLAIN-TEXT-UTF-8" |
                       "user-defined")
```

## §6 BagIt Transfer Package Record

```
bagitPackage:
  bagitId            : string (uuidv7)
  packageRef         : string (PHASE-1 §3
                       record reference)
  bagitVersion       : enum ("BagIt-1-0-RFC-
                       8493" | "BagIt-0-97-
                       legacy")
  payloadOxum        : string (the Payload-Oxum
                       declared in `bag-info.txt`
                       — total octets and total
                       streams)
  manifestAlgorithms : array of enum ("sha256"
                       | "sha512")
  baginfoFields      : object (the per-bag-info
                       field set — Source-
                       Organization, Contact-
                       Name, Contact-Email,
                       External-Description,
                       External-Identifier,
                       Bag-Size, Bagging-Date,
                       Bag-Group-Identifier,
                       Bag-Count)
  bagitSignature     : object (an Ed25519
                       signature per RFC 8032
                       over the manifest digest;
                       optionally a qualified
                       electronic signature per
                       EU eIDAS-2 where the
                       package crosses an EU
                       jurisdiction)
```

## §7 Trustworthy-Digital-Repository Audit Record

```
tdrAuditRecord:
  auditId            : string (uuidv7)
  programmeRef       : string (the operator's
                       programme identifier)
  auditFramework     : enum ("ISO-16363" |
                       "DIN-31644" |
                       "DSA-Core-Trust-Seal" |
                       "user-defined")
  auditScope         : object (the per-criterion
                       findings against ISO
                       16363's Organisational
                       Infrastructure / Digital-
                       Object Management /
                       Infrastructure-and-
                       Security risk Management
                       criteria)
  auditOutcome       : enum ("certified" |
                       "conditional" |
                       "non-certified" |
                       "user-defined")
  auditBody          : object (the audit
                       certification body's
                       legal name, the body's
                       ISO 16919:2020
                       accreditation reference,
                       and the certificate's
                       expiry)
```

## §8 Chain-of-Custody Record

```
custodyRecord:
  custodyId          : string (uuidv7)
  artefactRef        : string (the package,
                       PREMIS, format, BagIt, or
                       audit record identifier)
  custodyEvent       : enum ("ingestion" |
                       "fixity-check" |
                       "format-migration" |
                       "replication" |
                       "audit-conducted" |
                       "seal-applied" |
                       "seal-broken" |
                       "release-to-beneficiary"
                       | "withdrawal" |
                       "user-defined")
  eventTimestamp     : string (ISO 8601 date-
                       time)
  performingParty    : string (legal entity)
  hashOfArtefacts    : string (SHA-256 hex
                       digest per RFC 6234)
```

## §9 Manifest

Implementations publish a signed manifest
carrying `standardSlug` (constant value
"digital-time-capsule"), `version`,
`implementation`, the operator's
`trustworthinessAudit` envelope, and the
`profile` declaration that selects which of
the optional records (OAIS, PREMIS, format,
BagIt, audit) the implementation supports.
The manifest is signed using a key whose
public part is published on the operator's
`.well-known/wia/digital-time-capsule/`
discovery endpoint declared in PHASE-2.
