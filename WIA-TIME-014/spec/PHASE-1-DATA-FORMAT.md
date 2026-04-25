# WIA-TIME-014 (Data Time Transport) — Phase 1: Data Format Specification

> **Version:** 1.0.0
> **Status:** Official
> **Phase:** 1 of 4 (Data Format)
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

This document specifies the canonical data formats used by WIA-TIME-014 *Data Time Transport* deployments. The standard governs the deliberate movement of data through time: archival storage with verifiable provenance, scheduled deferred release ("time capsules"), tiered retention, and cold-to-warm data transitions. The formats interoperate with the IETF Time-Stamp Protocol (RFC 3161), the Evidence Record Syntax (RFC 4998), the ISO/IEC 18014 *Time-stamping services* family, and the ISO/IEC 27040 *Storage security* baseline.

### 1.1 Goals

- Provide a JSON / CBOR canonical encoding that captures every entity needed to assert and verify data movement across time, including time-anchored deposits, scheduled releases, and renewal cycles.
- Carry RFC 3161 *time-stamp tokens* and RFC 4998 *evidence records* losslessly so that any compliant verifier can reproduce the integrity proofs across decades.
- Express scheduled-release ("time capsule") policies in a way that is transparent to verifiers and auditors.
- Preserve the chain-of-custody fields required by ISO/IEC 27037 *Guidelines for identification, collection, acquisition and preservation of digital evidence*.

### 1.2 Non-goals

- Defining new hash functions. Hashing defers to FIPS 180-4 (SHA-2) and FIPS 202 (SHA-3).
- Defining new cryptographic time sources. Time-source traceability defers to ISO/IEC 18014-4:2015.
- Defining new storage hardware behaviour. Storage security defers to ISO/IEC 27040:2024.

---

## 2. Top-level Object Model

A WIA-TIME-014 deployment serializes into seven root entities:

```
DataTimeTransportDomain ──┬── Deposit[]
                          ├── Capsule[]
                          ├── ReleasePolicy[]
                          ├── RetentionPolicy[]
                          ├── EvidenceRecord[]
                          ├── StorageVolume[]
                          └── Auditor[]
```

Each entity has a stable URI of the form `wia-data-time://<domain-id>/<entity-type>/<entity-id>`.

### 2.1 DataTimeTransportDomain

```json
{
  "type": "DataTimeTransportDomain",
  "version": "1.0.0",
  "domainId": "<UUID v4 per RFC 9562>",
  "operator": {
    "name": "string",
    "iso27001Scope": "string",
    "iso27040StorageScope": "string"
  },
  "policyOid": "<X.680 ASN.1 OID>",
  "createdAt": "<RFC 3339 date-time>",
  "updatedAt": "<RFC 3339 date-time>"
}
```

The `iso27040StorageScope` field carries the boundary statement of the operator's ISO/IEC 27040 storage-security regime.

### 2.2 Deposit

A *Deposit* is the canonical record of a data placement into time-transport storage.

```json
{
  "type": "Deposit",
  "depositId": "string",
  "depositorRef": "<URI of depositor>",
  "depositedAt": "<RFC 3339 date-time>",
  "contentDigest": {
    "algorithm": "SHA-256|SHA-384|SHA-512|SHA3-256|SHA3-512",
    "value": "<hex-encoded digest>"
  },
  "contentType": "<RFC 6838 media type>",
  "contentSizeBytes": "uint64",
  "storageVolumeRefs": ["<StorageVolume URI>"],
  "encryptionAtRest": {
    "algorithm": "AES-256-GCM|AES-128-GCM",
    "keyProvenance": "ISO 11770-2|ISO 11770-3"
  },
  "initialTokenRef": "<TimeStampToken URI>"
}
```

### 2.3 Capsule

A *Capsule* is a scheduled-release wrapper around one or more deposits. The release schedule is declarative; release proceeds only when the policy conditions are satisfied.

```json
{
  "type": "Capsule",
  "capsuleId": "string",
  "depositIds": ["string"],
  "releasePolicyRef": "<ReleasePolicy URI>",
  "createdAt": "<RFC 3339 date-time>",
  "earliestReleaseAt": "<RFC 3339 date-time>",
  "latestReleaseAt": "<RFC 3339 date-time | null>",
  "releaseEvidence": "<URI of release-event evidence | null>",
  "state": "PENDING|ELIGIBLE|RELEASED|REVOKED"
}
```

### 2.4 ReleasePolicy

```json
{
  "type": "ReleasePolicy",
  "releasePolicyId": "string",
  "kind": "TIME-ABSOLUTE|TIME-RELATIVE|EVENT-TRIGGERED|MULTI-PARTY-CONSENT",
  "conditions": [
    {"kind": "TIME-AFTER", "afterRfc3339": "<RFC 3339 date-time>"},
    {"kind": "EVENT", "eventDigest": "<hex>"},
    {"kind": "QUORUM", "approverIds": ["string"], "threshold": "uint"}
  ]
}
```

Conditions are AND-combined; multiple policies can be referenced for OR semantics.

### 2.5 RetentionPolicy

```json
{
  "type": "RetentionPolicy",
  "retentionPolicyId": "string",
  "tier": "HOT|WARM|COLD|FROZEN",
  "minRetentionDays": "uint",
  "maxRetentionDays": "uint|null",
  "lawfulBasisRef": "<URI of lawful-basis record>",
  "destructionPolicy": "PHYSICAL|CRYPTOGRAPHIC|BOTH"
}
```

The destruction policy MUST be aligned with ISO/IEC 27040 §6.7 (Sanitisation) when the deployment uses physical destruction, and with the operator's cryptographic-erasure procedure when the deployment uses cryptographic destruction.

### 2.6 EvidenceRecord

The *EvidenceRecord* is the long-term archival proof structure modelled on RFC 4998 *Evidence Record Syntax (ERS)*. Its structure mirrors the WIA-TIME-011 EvidenceRecord; the cross-reference is normative for cross-deployment verification.

```json
{
  "type": "EvidenceRecord",
  "recordId": "string",
  "subjectDepositIds": ["string"],
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

### 2.7 StorageVolume

A *StorageVolume* describes a tier of storage operated under ISO/IEC 27040 controls.

```json
{
  "type": "StorageVolume",
  "volumeId": "string",
  "tier": "HOT|WARM|COLD|FROZEN",
  "capacityTb": "number",
  "encryptionAtRest": {
    "algorithm": "AES-256-GCM|AES-128-GCM",
    "keyProvenance": "ISO 11770-2|ISO 11770-3"
  },
  "physicalLocations": [
    {"iso6709": "+37.5326-126.9905+50/", "iso27040Zone": "string"}
  ],
  "iso27040ControlSet": "string"
}
```

### 2.8 Auditor

```json
{
  "type": "Auditor",
  "auditorId": "string",
  "kind": "INTERNAL|EXTERNAL|REGULATOR|SUBJECT-ACCESS",
  "publicKey": {"format": "SPKI", "spkiBytes": "<base64 SubjectPublicKeyInfo>"}
}
```

---

## 3. Canonical Encodings

WIA-TIME-014 mandates two interchangeable encodings:

1. **JSON** — RFC 8259, UTF-8, NFC-normalised strings.
2. **CBOR** — RFC 8949, with the *Concise Data Definition Language* (RFC 8610) schema published alongside.

Both encodings MUST be byte-equivalent under round-trip via the canonical CBOR deterministic encoding rules (RFC 8949 §4.2). End-to-end protection of payloads uses *CBOR Object Signing and Encryption* (COSE, RFC 9052 / 9053).

### 3.1 Time

Time-stamp tokens are the authoritative time anchors for any deposit or release event. RFC 3339 *date-time* strings appear in human-readable surfaces; the binding source is the TSA identified by the deployment.

### 3.2 Identifiers

UUID v4 (RFC 9562) is the default. Cross-deployment references to WIA-TIME-011 entities use the full URI form `wia-history://...`.

### 3.3 Hash agility

Each digest record carries an explicit algorithm identifier. Implementations MUST support SHA-256 and SHA-384 at minimum and SHOULD support SHA-512 and the SHA-3 family for forward-compatibility.

---

## 4. Versioning and Compatibility

The `version` field uses semver 2.0.0. Phase 1 v1.x payloads are backwards-compatible within the major version. Edition references for normative work were:

- ISO/IEC 18014-1:2008; ISO/IEC 18014-2:2021; ISO/IEC 18014-3; ISO/IEC 18014-4:2015.
- ISO/IEC 27001:2022; ISO/IEC 27002:2022; ISO/IEC 27037:2012; ISO/IEC 27042:2015.
- ISO/IEC 27040:2024 — *Storage security.*
- RFC 3161 — *TSP.*
- RFC 4998 — *ERS.*
- RFC 5280 — *X.509 PKI.*
- RFC 5816 — *ESSCertIDv2.*
- RFC 8392 — *CWT.*
- FIPS 180-4; FIPS 202.

---

## 5. Privacy and Confidentiality

Time-transported data may contain personal data and operator-confidential information. Phase 1 reserves the following controls:

- `confidentialityClass`: OPEN | OPERATOR-CONFIDENTIAL | REGULATORY-RESTRICTED | EXPORT-CONTROLLED.
- `Deposit.privacyAttributes`: optional structure carrying retention and lawful-basis references.
- `Capsule.releasePolicy`: when the policy itself contains personal data (e.g. quorum approver identities), the Auditor scope-of-access governs visibility.

Personal data classification follows the operator's ISO/IEC 27701:2019 PIM record.

---

## 6. References

1. ISO/IEC 18014 (all parts).
2. ISO/IEC 27001:2022; ISO/IEC 27037:2012; ISO/IEC 27040:2024; ISO/IEC 27042:2015; ISO/IEC 27701:2019.
3. ISO 11770-2; ISO 11770-3 — *Key management.*
4. RFC 3161; RFC 4998; RFC 5280; RFC 5816; RFC 5905; RFC 6838.
5. RFC 7252; RFC 7641 — *CoAP, OBSERVE.*
6. RFC 8259; RFC 8392; RFC 8446; RFC 8610; RFC 8915; RFC 8949.
7. RFC 9052; RFC 9053; RFC 9110; RFC 9147; RFC 9162; RFC 9457; RFC 9562; RFC 9700.
8. FIPS 180-4; FIPS 197; FIPS 202.

---

## 7. Detailed Field Conventions

### 7.1 Deposit immutability

A *Deposit* is immutable once recorded. Modifications produce a new deposit linked via the `lineage` field (Phase 1 §2.2 of WIA-TIME-011, mirrored here for cross-deployment compatibility). A deposit MAY be marked `superseded`, but its content digest, time-stamp token, and audit trail remain present indefinitely.

### 7.2 Capsule revocation

A capsule MAY be revoked before release. Revocation is itself a state-change event recorded in the audit log; revocation does NOT delete the underlying deposit. Release policies that include irreversible conditions (e.g. multi-party consent already given) MUST refuse revocation and MUST surface the refusal as a problem detail.

### 7.3 Tier transition

Tier transitions (HOT → WARM → COLD → FROZEN) MUST:

- Record a transition event in the audit log.
- Re-time-stamp the affected deposits if the transition crosses a renewal cadence boundary.
- Update the StorageVolume reference on every affected deposit.

### 7.4 Multi-deposit capsule

A capsule MAY reference multiple deposits. The release event releases all referenced deposits atomically; partial release is forbidden. The release evidence MUST list every released deposit identifier.

## 8. Cross-Deployment References

WIA-TIME-014 deployments commonly cross-reference WIA-TIME-011 (Historical Integrity) scopes. The cross-reference is expressed via the `wia-history://...` URI form, and verifiers SHOULD validate both deployments' tokens before trusting the asserted integrity of a deposit.

## 9. Receipt Structure

Every deposit results in a signed *Receipt* delivered to the depositor. The Receipt structure carries:

- The assigned `depositId`.
- The bound TimeStampToken URI.
- The chosen storage volume references.
- The retention policy reference.
- The COSE_Sign1 signature over the Receipt manifest, produced by the gateway's signing key.
- The gateway's public-key fingerprint, allowing the depositor to verify the signature against the discovery document's published key.

Receipts MUST be retained by the depositor for at least the retention horizon of the underlying deposit. The Receipt is itself eligible for deposit into a separate WIA-TIME-014 domain when long-term proof of the original deposit transaction is required.

## 10. Tiering Heuristics

Operators commonly apply the following heuristics when assigning a deposit to a tier:

| Tier | Typical access cadence | Recommended use |
|------|------------------------|-----------------|
| HOT | Daily / hourly | Active records, recent transactions |
| WARM | Weekly / monthly | Recently-aged records, regulatory ready-access |
| COLD | Annual / on-demand | Long-term retention with infrequent access |
| FROZEN | Rare / event-triggered | Decadal retention, time-capsule deposits |

The heuristics are informative; the binding tier choice is recorded in the Phase-1 *Deposit.storageVolumeRefs* field and in the bound *RetentionPolicy*.