# WIA-TIME-011 (Historical Integrity) — Phase 1: Data Format Specification

> **Version:** 1.0.0
> **Status:** Official
> **Phase:** 1 of 4 (Data Format)
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

This document specifies the canonical data formats used by WIA-TIME-011 *Historical Integrity* deployments. The formats describe **historical events**, **timeline ledgers**, **time-stamp tokens**, **evidence records**, and **transparency logs** in a way that interoperates with the IETF Time-Stamp Protocol (RFC 3161), the Evidence Record Syntax (RFC 4998), the Certificate Transparency family (RFC 6962, RFC 9162), and the ISO/IEC 18014 *Time-stamping services* family.

### 1.1 Goals

- Provide a JSON / CBOR canonical encoding that captures every entity needed to assert and verify historical integrity at internet scale.
- Carry RFC 3161 *time-stamp tokens* and RFC 4998 *evidence records* losslessly so that any compliant verifier can reproduce the integrity proofs.
- Express transparency-log inclusion proofs and Merkle-tree consistency proofs compatible with RFC 9162 *Certificate Transparency v2*.
- Preserve the chain-of-custody fields required by ISO/IEC 27037 *Guidelines for identification, collection, acquisition and preservation of digital evidence*.

### 1.2 Non-goals

- Defining new hash functions. Hashing defers to FIPS 180-4 (SHA-2) and FIPS 202 (SHA-3).
- Defining new cryptographic time sources. Time-source traceability defers to ISO/IEC 18014-4:2015.
- Defining new public-key infrastructure. PKI defers to RFC 5280 *X.509 Certificate and CRL Profile*.

---

## 2. Top-level Object Model

A WIA-TIME-011 deployment serializes into seven root entities:

```
HistoricalIntegrityScope ──┬── Event[]
                           ├── TimelineLedger[]
                           ├── TimeStampToken[]
                           ├── EvidenceRecord[]
                           ├── TransparencyLog[]
                           └── Verifier[]
```

Each entity has a stable URI of the form `wia-history://<scope-id>/<entity-type>/<entity-id>`.

### 2.1 HistoricalIntegrityScope

```json
{
  "type": "HistoricalIntegrityScope",
  "version": "1.0.0",
  "scopeId": "<UUID v4 per RFC 9562>",
  "operator": {
    "name": "string",
    "iso27001Scope": "string"
  },
  "policyOid": "<X.680 ASN.1 OID for the integrity policy>",
  "createdAt": "<RFC 3339 date-time>",
  "updatedAt": "<RFC 3339 date-time>"
}
```

The `policyOid` field carries the policy identifier of the time-stamping authority (TSA) used by this scope, per RFC 3161 §2.4.1.

### 2.2 Event

An *Event* is the atomic record whose integrity is asserted. Events are addressable by content digest.

```json
{
  "type": "Event",
  "eventId": "string",
  "occurredAt": "<RFC 3339 date-time>",
  "kind": "DOCUMENT|TRANSACTION|SENSOR-OBSERVATION|GOVERNANCE-RECORD|MEDIA",
  "contentDigest": {
    "algorithm": "SHA-256|SHA-384|SHA-512|SHA3-256|SHA3-512",
    "value": "<hex-encoded digest>"
  },
  "contentType": "<RFC 6838 media type>",
  "lineage": [
    {"parentEventId": "string", "relation": "DERIVES-FROM|REPLACES|RESPONDS-TO|ANNOTATES"}
  ]
}
```

Hash-algorithm identifiers MUST be FIPS-approved (FIPS 180-4 or FIPS 202). Multi-hash redundancy is permitted via repeated `contentDigest` entries.

### 2.3 TimelineLedger

A *TimelineLedger* records the totally-ordered sequence of events in a scope. Ledger entries are linked by chained hashing and are independently verifiable.

```json
{
  "type": "TimelineLedger",
  "ledgerId": "string",
  "head": {"index": "uint64", "rootHash": "<hex>"},
  "linkAlgorithm": "SHA-256|SHA-512",
  "snapshotPolicy": "PERIODIC|ON-WRITE|HYBRID",
  "isoIec18014PartReference": "ISO/IEC 18014-3"
}
```

Linked time-stamp tokens (ISO/IEC 18014-3) MAY be used in place of independent tokens when long-term verification is required.

### 2.4 TimeStampToken

A *TimeStampToken* is the canonical proof-of-existence artefact, modelled on RFC 3161 §2.4.2 and ISO/IEC 18014-2.

```json
{
  "type": "TimeStampToken",
  "tokenId": "string",
  "subjectDigest": "<hex>",
  "subjectAlgorithm": "SHA-256|SHA-384|SHA-512|SHA3-256|SHA3-512",
  "policyOid": "<ASN.1 OID>",
  "tsaIdentifier": "<URI of TSA>",
  "issuedAt": "<RFC 3339 date-time>",
  "accuracyMillis": "uint",
  "ordering": "boolean",
  "nonce": "<hex|null>",
  "tokenBytes": "<base64 RFC 3161 TimeStampToken DER>"
}
```

The `tokenBytes` field carries the literal RFC 3161 *TimeStampToken* (a CMS *SignedData* structure per RFC 5652) so that any RFC 3161 verifier can validate it without re-encoding.

### 2.5 EvidenceRecord

An *EvidenceRecord* is the long-term archival proof structure modelled on RFC 4998 *Evidence Record Syntax (ERS)*.

```json
{
  "type": "EvidenceRecord",
  "recordId": "string",
  "subjectEventIds": ["string"],
  "version": 1,
  "digestAlgorithm": "SHA-256|SHA-384|SHA-512",
  "cryptoInfos": "<base64 ERS CryptoInfos>",
  "encryptionInfo": "<base64 ERS EncryptionInfo|null>",
  "archiveTimeStampSequences": [
    {
      "sequenceIndex": "uint",
      "archiveTimeStamps": ["<base64 ERS ArchiveTimeStamp>"]
    }
  ]
}
```

The `archiveTimeStampSequences` field carries the literal ERS sequences so that re-time-stamping (RFC 4998 §5.2) can be performed by a downstream archive.

### 2.6 TransparencyLog

A *TransparencyLog* is a public-verifiable append-only log modelled on RFC 9162 *Certificate Transparency v2*. WIA-TIME-011 uses CT v2's data-structure conventions for non-PKI domains.

```json
{
  "type": "TransparencyLog",
  "logId": "string",
  "logUrl": "<URI>",
  "logSpkiHash": "<hex SHA-256 of log public key SPKI>",
  "treeSize": "uint64",
  "rootHash": "<hex>",
  "signedTreeHead": "<base64 RFC 9162 SignedTreeHead>",
  "maxMergeDelaySeconds": "uint",
  "transItemSchemaVersion": 2
}
```

### 2.7 Verifier

A *Verifier* describes a third-party verifier that has been authorised to read or attest portions of the scope.

```json
{
  "type": "Verifier",
  "verifierId": "string",
  "kind": "OPERATOR|AUDITOR|TSA|TRANSPARENCY-MONITOR",
  "iso27037Role": "FIRST-RESPONDER|EVIDENCE-COLLECTOR|EVIDENCE-CUSTODIAN|EXAMINER",
  "publicKey": {"format": "SPKI", "spkiBytes": "<base64 SubjectPublicKeyInfo>"}
}
```

---

## 3. Canonical Encodings

WIA-TIME-011 mandates two interchangeable encodings:

1. **JSON** — RFC 8259, UTF-8, NFC-normalised strings.
2. **CBOR** — RFC 8949, with the *Concise Data Definition Language* (RFC 8610) schema published alongside.

Both encodings MUST be byte-equivalent under round-trip via the canonical CBOR deterministic encoding rules (RFC 8949 §4.2). End-to-end protection of payloads uses *CBOR Object Signing and Encryption* (COSE, RFC 9052 / 9053).

### 3.1 Time

Time-stamp tokens are the authoritative time anchors. RFC 3339 *date-time* strings are used in all human-readable surfaces; the binding source is the TSA identified in §2.4.

### 3.2 Identifiers

UUID v4 (RFC 9562) is the default. Internal Merkle-tree leaf indices follow the RFC 9162 §4.1 *TransItem* indexing model.

### 3.3 Hash agility

Each digest record carries an explicit algorithm identifier. Implementations MUST support SHA-256 and SHA-384 at minimum and SHOULD support SHA-512 and the SHA-3 family for forward-compatibility.

---

## 4. Versioning and Compatibility

The `version` field uses semver 2.0.0. Phase 1 v1.x payloads are backwards-compatible within the major version. Edition references for normative work were:

- ISO/IEC 18014-1:2008 — *Time-stamping services — Framework.*
- ISO/IEC 18014-2:2021 — *Mechanisms producing independent tokens.*
- ISO/IEC 18014-3 — *Mechanisms producing linked tokens.*
- ISO/IEC 18014-4:2015 — *Traceability of time sources.*
- ISO/IEC 27037:2012 — *Digital evidence handling.*
- ISO/IEC 27042:2015 — *Analysis and interpretation of digital evidence.*
- RFC 3161 — *Internet X.509 PKI Time-Stamp Protocol (TSP).*
- RFC 4998 — *Evidence Record Syntax (ERS).*
- RFC 5816 — *ESSCertIDv2 update for RFC 3161.*
- RFC 5652 — *Cryptographic Message Syntax (CMS).*
- RFC 6962 — *Certificate Transparency v1.*
- RFC 9162 — *Certificate Transparency v2.*
- FIPS 180-4 — *Secure Hash Standard (SHA-1, SHA-2 family).*
- FIPS 202 — *SHA-3 Standard.*

---

## 5. Privacy and Confidentiality

Historical-integrity records may include personal data and operator-confidential information. Phase 1 reserves the following controls:

- `confidentialityClass`: OPEN | OPERATOR-CONFIDENTIAL | REGULATORY-RESTRICTED | EXPORT-CONTROLLED.
- `Event.privacyMaskRegions`: optional structure for redacting bytes inside the digested content (the redaction does not invalidate the digest, since the digest is over the full pre-redaction bytes; the privacy mask is enforced at presentation time only).
- `Verifier.scopeOfAccess`: array of policy strings limiting what a verifier may read.

Personal data classification follows the operator's ISO/IEC 27701:2019 PIM record.

---

## 6. References

1. ISO/IEC 18014-1:2008; ISO/IEC 18014-2:2021; ISO/IEC 18014-3; ISO/IEC 18014-4:2015 — *Time-stamping services.*
2. ISO/IEC 27001:2022; ISO/IEC 27037:2012; ISO/IEC 27042:2015; ISO/IEC 27701:2019.
3. RFC 3161 — *Time-Stamp Protocol (TSP).*
4. RFC 3339 — *Date and Time on the Internet.*
5. RFC 4998 — *Evidence Record Syntax (ERS).*
6. RFC 5280 — *X.509 PKI Certificate and CRL Profile.*
7. RFC 5652 — *Cryptographic Message Syntax (CMS).*
8. RFC 5816 — *ESSCertIDv2 update for RFC 3161.*
9. RFC 6838 — *Media Type Specifications and Registration Procedures.*
10. RFC 6962 — *Certificate Transparency v1.*
11. RFC 8259; RFC 8610; RFC 8949; RFC 9052; RFC 9053; RFC 9562.
12. RFC 9162 — *Certificate Transparency v2.*
13. FIPS 180-4 — *SHA family.*
14. FIPS 202 — *SHA-3 family.*

---

## 7. Detailed Field Conventions

### 7.1 Digest encoding

The `contentDigest.value` field is a hex-encoded string in lowercase. Implementations MUST normalise to lowercase on serialisation and MUST accept either case on deserialisation. The hex string length is exactly `2 × digestLengthBytes` for the chosen algorithm:

| Algorithm | Digest length (bytes) | Hex length |
|-----------|-----------------------|------------|
| SHA-256 | 32 | 64 |
| SHA-384 | 48 | 96 |
| SHA-512 | 64 | 128 |
| SHA3-256 | 32 | 64 |
| SHA3-512 | 64 | 128 |

### 7.2 Token DER encoding

The `TimeStampToken.tokenBytes` field carries the literal RFC 3161 *TimeStampToken* in DER (Distinguished Encoding Rules per ITU-T X.690). The DER bytes are then base64-encoded for JSON transport (RFC 4648 §4) and embedded directly as a binary string for CBOR transport (RFC 8949 §3.1).

### 7.3 Evidence record encoding

The `EvidenceRecord.cryptoInfos` and `archiveTimeStamps` fields carry the literal RFC 4998 ASN.1 structures in DER. The encoding rules are identical to the token-DER conventions of §7.2.

### 7.4 STH binding

The `TransparencyLog.signedTreeHead` field carries the literal RFC 9162 *SignedTreeHead* TLS-encoded structure (RFC 9162 §4.10) so that any monitor can verify the STH against the log's published public key without re-encoding.

## 8. Lineage Conventions

The `lineage` array on each Event records semantic relationships to other events. The relations are:

- `DERIVES-FROM` — the event content is derived from the parent (e.g. a redacted version, a translation, an extracted summary).
- `REPLACES` — the event supersedes the parent; the parent remains in the ledger but is logically deprecated.
- `RESPONDS-TO` — the event is a response to the parent (e.g. a regulatory response to a filing).
- `ANNOTATES` — the event annotates the parent without changing its content.

Lineage relations are themselves digested as part of the event's content; modifications to lineage produce a new event.

## 9. Multi-tenant Naming

When a deployment serves multiple tenants, the `scopeId` is opaque and globally unique. The scope-internal entity identifiers (`eventId`, `tokenId`, etc.) are scoped to the parent scope and MUST NOT be assumed to be unique across scopes. Cross-scope references use the full URI form `wia-history://<scope-id>/<entity-type>/<entity-id>`.
