# WIA-TIME-011 (Historical Integrity) — Phase 3: Protocol Specification

> **Version:** 1.0.0
> **Status:** Official
> **Phase:** 3 of 4 (Protocol)
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

Phase 3 specifies the wire protocols used by WIA-TIME-011 deployments. The objective is to give a deterministic mapping of every Phase-2 verb to a concrete byte-level behaviour and to fix the operational interoperability points required by RFC 3161 (TSP), RFC 4998 (ERS), and RFC 9162 (CT v2), within the ISO/IEC 18014 *Time-stamping services* family.

### 1.1 Protocol stack

```
+---------------------------------------------------+
| WIA-TIME-011 application semantics (Phase 1+2)    |
+---------------------------------------------------+
| Mediation: gateway, TSA client, log monitor       |
+---------------------------------------------------+
| Transport (one of):                               |
|   RFC 3161 TSP over HTTPS                         |
|   RFC 9162 CT v2 over HTTPS                       |
|   HTTP/REST (Phase 2 §2)                          |
+---------------------------------------------------+
| Cryptographic: TLS 1.3, COSE, CMS                 |
+---------------------------------------------------+
```

---

## 2. RFC 3161 TSP Wire Profile

### 2.1 Request

A `TimeStampReq` carries:

- `version` = 1.
- `messageImprint` — `{ algorithm, hashedMessage }`.
- `reqPolicy` — OID; MUST match the scope policy OID where known.
- `nonce` — random 64-bit integer for freshness, REQUIRED for any production-grade deployment.
- `certReq` — boolean; the server MUST honour this flag.
- `extensions` — optional, for ESSCertIDv2 etc.

### 2.2 Response

A `TimeStampResp` carries `status` and `timeStampToken`. The `timeStampToken` is a CMS *SignedData* (RFC 5652) whose `eContent` is the `TSTInfo` structure. The `TSTInfo` structure includes:

- `version` = 1.
- `policy` — OID identifying the TSA policy.
- `messageImprint` — echo of the request.
- `serialNumber` — issuer-unique integer.
- `genTime` — `GeneralizedTime` per ASN.1 §11.7.
- `accuracy` — optional `{seconds, millis, micros}`.
- `ordering` — optional boolean.
- `nonce` — echo of the request nonce.
- `tsa` — optional GeneralName.
- `extensions` — including the ESSCertIDv2 (RFC 5816).

### 2.3 Verification

A verifier MUST:

1. Verify the CMS signature against the TSA's certificate chain (RFC 5280).
2. Verify the certificate has the *id-kp-timeStamping* extended-key-usage (RFC 3161 §2.3).
3. Verify the `messageImprint` matches the digest the verifier computed.
4. Verify the policy OID is acceptable for the verification context.
5. Verify the nonce echo when one was provided in the request.
6. Verify that `genTime` falls within the certificate validity window.

### 2.4 Long-term verification

For long-term verification, the verifier MUST also validate:

- The TSA certificate revocation status as of `genTime` (RFC 5280, OCSP RFC 6960, or CRL).
- Any successor evidence record per RFC 4998 §5.

---

## 3. RFC 4998 Evidence-Record Wire Profile

### 3.1 Construction

An *EvidenceRecord* is constructed by:

1. Building a Merkle hash tree over the subject digests using the chosen `digestAlgorithm`.
2. Time-stamping the Merkle root with an RFC 3161 token (the *initial archive time-stamp*).
3. Periodically re-time-stamping when the underlying hash algorithm or TSA certificate approaches end-of-life (RFC 4998 §5.2).
4. Appending each new archive time-stamp to the `archiveTimeStampSequences` list.

### 3.2 Verification

A verifier MUST:

1. Reconstruct the Merkle path from each subject digest to the root.
2. Verify the chain of archive time-stamps from oldest to newest.
3. Verify each TSA token per §2.3.

### 3.3 Renewal triggers

Renewal triggers MUST include:

- Hash-algorithm deprecation (e.g. SHA-1 → SHA-256 transition).
- TSA certificate expiry within the renewal window.
- Operator-policy renewal cadence (default: 5 years; configurable per scope).

---

## 4. RFC 9162 Certificate Transparency v2 Wire Profile

### 4.1 Inclusion

An item is included in the transparency log when the operator submits a `TransItem` to the log via the operator-controlled submit interface. The log returns a *Signed Certificate Timestamp v2* (SCT v2) which is the log's promise to incorporate the item within the *MaxMergeDelay* window.

### 4.2 Tree updates

The log appends to a Merkle tree over `TransItem` leaves. The current state is summarised by the *SignedTreeHead* (STH), retrievable via `GET /ct/v2/get-sth`.

### 4.3 Inclusion proof

`GET /ct/v2/get-proof-by-hash?hash=...&tree_size=...` returns an inclusion proof that can be verified against the STH the verifier holds.

### 4.4 Consistency proof

`GET /ct/v2/get-sth-consistency?first=...&second=...` returns a consistency proof between two tree sizes, allowing a monitor to verify that no historical entries were rewritten.

### 4.5 Append-only invariant

The append-only invariant is the central guarantee of the transparency log. Any monitor that detects a violation MUST raise an alarm to the audit log of every WIA-TIME-011 scope bound to the log.

---

## 5. Identity, Time, and Cryptography

### 5.1 Identity

- **TSA**: an X.509 v3 certificate (RFC 5280) with the *id-kp-timeStamping* extended-key-usage.
- **Transparency log**: an EC public key signing the STH.
- **Operators**: federated identity over OAuth 2.1 (RFC 9700) and OpenID Connect.

### 5.2 Time

The TSA's clock is the authoritative time source for all time-stamp tokens issued by the deployment. The TSA's clock MUST be slaved to a traceable time source per ISO/IEC 18014-4:2015 *Traceability of time sources*. Operators MAY layer NTPv4 with NTS (RFC 5905, RFC 8915) on top of the ISO/IEC 18014-4 traceability chain.

### 5.3 Cryptographic algorithms

| Layer | Algorithm | Reference |
|-------|-----------|-----------|
| TLS | TLS 1.3 cipher suites | RFC 8446 |
| Hashing | SHA-256 / SHA-384 / SHA-512 / SHA-3 | FIPS 180-4, FIPS 202 |
| TSA signature | RSA-PSS, ECDSA, EdDSA | RFC 5652, RFC 8410 |
| COSE signature | ES256, EdDSA | RFC 9053 |
| Storage encryption | AES-256-GCM | ISO/IEC 18033-3, FIPS 197 |

Implementations MUST refuse cipher suites whose IETF status is "not recommended" for new deployments.

---

## 6. Failure Handling

### 6.1 TSA unavailable

When the bound TSA is unavailable, the gateway MUST:

- Queue token requests up to the configured retention limit.
- Surface the queued state through the health endpoint.
- Return `503 Service Unavailable` with a `Retry-After` header for new requests.

### 6.2 Inconsistent tree

When a transparency-log monitor detects an inconsistency, the gateway MUST:

- Raise the `history/inconsistent-tree` problem on every scope bound to the log.
- Disable new submissions to the log.
- Log an audit entry per IEC 62443-3-3 SR 6.1.

### 6.3 Hash deprecation

When a hash algorithm's IETF or NIST status changes to "deprecated" or "disallowed", the gateway MUST:

- Refuse new submissions using the affected algorithm.
- Trigger renewal on every evidence record that uses the algorithm.
- Surface the renewal queue through the health endpoint.

### 6.4 Certificate expiry

When the TSA's certificate approaches expiry, the gateway MUST trigger renewal of every evidence record dependent on the certificate, ahead of the certificate's expiry by at least the renewal-window duration.

---

## 7. Conformance Profiles

### 7.1 Baseline profile (P-B)

A P-B gateway MUST:

- Implement the HTTP/REST surface (Phase 2 §2).
- Implement the RFC 3161 TSP surface (§2).
- Implement the RFC 4998 evidence-record builder (§3).
- Issue COSE-signed audit-log digests.

### 7.2 Transparency profile (P-T)

A P-T gateway MUST additionally:

- Implement the RFC 9162 CT v2 surface (§4).
- Implement the monitor protocol (§4.4).
- Maintain a hash-algorithm-agile pipeline (§6.3).

### 7.3 Long-term-archive profile (P-LTA)

A P-LTA gateway MUST additionally:

- Implement the renewal triggers (§3.3).
- Maintain a documented renewal log per scope.
- Validate every renewal cycle against ISO/IEC 18014-4 traceability records.

---

## 8. Conformance Tests

The conformance suite includes:

- **TSP round-trip** vectors per RFC 3161 §3.
- **ERS Merkle reconstruction** tests per RFC 4998.
- **CT v2 inclusion / consistency** tests per RFC 9162 §6.
- **TLS 1.3 cipher inventory** test against the RFC 8446 mandatory profile.
- **Renewal cadence** test that simulates certificate expiry and verifies that renewal is triggered before the expiry threshold.

A conformant gateway MUST pass every vector in the suite for its declared profile set.

---

## 9. References

1. ISO/IEC 18014-1:2008; ISO/IEC 18014-2:2021; ISO/IEC 18014-3; ISO/IEC 18014-4:2015.
2. ISO/IEC 18033-3:2010; ISO/IEC 27001:2022; ISO/IEC 27037:2012; ISO/IEC 27042:2015.
3. IEC 62443-3-3:2013.
4. RFC 3161 — *TSP.*
5. RFC 4998 — *ERS.*
6. RFC 5280 — *X.509 PKI.*
7. RFC 5652 — *CMS.*
8. RFC 5816 — *ESSCertIDv2.*
9. RFC 5905; RFC 8915 — *NTPv4, NTS.*
10. RFC 6960 — *OCSP.*
11. RFC 8410 — *EdDSA in CMS.*
12. RFC 8446 — *TLS 1.3.*
13. RFC 8949 — *CBOR.*
14. RFC 9052; RFC 9053 — *COSE.*
15. RFC 9110; RFC 9457 — *HTTP, problem details.*
16. RFC 9162 — *CT v2.*
17. RFC 9700 — *OAuth 2.1.*
18. FIPS 180-4 — *SHA family.*
19. FIPS 197 — *AES.*
20. FIPS 202 — *SHA-3 family.*

---

## 10. Detailed Examples

### 10.1 TSP request and response over HTTPS

A canonical TSP exchange is:

```
POST /wia-time-011/v1/scopes/sc-archive/timestamps/tsp HTTP/1.1
Host: gateway.example.org
Content-Type: application/timestamp-query
Authorization: Bearer <token>
Content-Length: 51

<DER-encoded TimeStampReq>
```

```
HTTP/1.1 200 OK
Content-Type: application/timestamp-reply
Content-Length: 1843

<DER-encoded TimeStampResp>
```

The response body's `timeStampToken` is the literal RFC 3161 token, suitable for storage as Phase-1 *TimeStampToken.tokenBytes*.

### 10.2 ERS Merkle reconstruction

For an evidence record with three subject digests `D1, D2, D3`, the Merkle reconstruction proceeds as:

1. Compute `H(D1)`, `H(D2)`, `H(D3)` (already supplied by the caller).
2. Compute `H12 = H(D1 || D2)`.
3. Compute `Hroot = H(H12 || D3)`.
4. Verify the archive time-stamp covers `Hroot`.

The exact concatenation order follows RFC 4998 §5.1 (lexicographic ordering of subject digests when no explicit order is supplied).

### 10.3 CT v2 inclusion proof

For an item with `LeafIndex = 17` in a tree of size 20, the inclusion proof contains the auxiliary nodes needed to reconstruct the path to the root. The verifier recomputes the root and compares against the STH the verifier currently holds.

---

## 11. Time Discontinuities

Time discontinuities (leap seconds, DST changes, manual adjustments to the TSA's clock) are operational events that MUST:

- Be logged in the audit log per IEC 62443-3-3 SR 6.1.
- Trigger an immediate ISO/IEC 18014-4 traceability re-attestation.
- NOT silently rewrite historical token timestamps.

---

## 12. Profile Selection Guidance

Operators choose a conformance profile based on the deployment's role:

- **Internal records management (P-B)** — TSA-backed timestamping for an organisation's records.
- **Public-verifiable records (P-T)** — adds a transparency log so external monitors can verify the append-only invariant.
- **Long-term archives (P-LTA)** — adds renewal cadence so evidence remains verifiable across hash and certificate transitions.

A deployment MAY combine profiles (e.g. P-T + P-LTA). Combined profiles MUST surface every constituent tag.
