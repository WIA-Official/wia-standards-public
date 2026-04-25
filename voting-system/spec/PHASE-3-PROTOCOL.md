# WIA-SOC-015: Phase 3 - Verification Protocol Specification

**Version:** 1.0  
**Status:** FINAL  
**Last Updated:** 2025-01-15  
**Standards Body:** World Certification Industry Association (WIA)

---

## 1. Overview

Phase 3 defines cryptographic protocols enabling end-to-end verifiable voting while maintaining ballot secrecy. This specification covers voter authentication, ballot encryption, blockchain integration, and zero-knowledge proofs.

---

## 2. Voter Authentication

### 2.1 Multi-Factor Authentication

Minimum two of three factor types required:
- **Something You Know:** Password, PIN, security questions
- **Something You Have:** Smart card, mobile device, hardware token
- **Something You Are:** Fingerprint, facial recognition, iris scan

### 2.2 Biometric Authentication

Supported modalities:
- Fingerprint (99.9% accuracy requirement)
- Facial recognition (99.5% accuracy requirement)
- Iris scan (99.99% accuracy requirement)
- Voice recognition (98% accuracy requirement)

**Privacy Protection:**
- Biometric templates stored as one-way hashes
- No raw biometric images retained
- Templates cannot be reverse-engineered

---

## 3. Ballot Encryption

### 3.1 Homomorphic Encryption

Approved cryptosystems:
- ElGamal (2048-bit minimum)
- Paillier (3072-bit minimum)

**Security Requirement:** Minimum 256-bit security strength

### 3.2 Encryption Process

1. Voter casts ballot
2. Ballot encrypted with election public key
3. Voter signs encrypted ballot with private key
4. Signed encrypted ballot stored
5. Receipt generated for voter verification

---

## 4. Blockchain Integration

### 4.1 Immutable Audit Trail

Every election event recorded on permissioned blockchain:
- Voter registration events
- Ballot cast events
- Vote tabulation events
- Audit access events

### 4.2 Smart Contract Requirements

```solidity
contract Election {
  mapping(address => bool) hasVoted;
  mapping(bytes32 => uint) voteCounts;
  
  function castVote(bytes32 ballotHash, bytes signature) public {
    require(!hasVoted[msg.sender], "Already voted");
    require(verifySignature(ballotHash, signature), "Invalid signature");
    require(block.timestamp < votingDeadline, "Voting period ended");
    
    hasVoted[msg.sender] = true;
    emit VoteCast(msg.sender, ballotHash, block.timestamp);
  }
}
```

---

## 5. Zero-Knowledge Proofs

### 5.1 zk-SNARKs Implementation

- **Proving System:** Groth16
- **Security Level:** 128-bit
- **Proof Size:** <200 bytes
- **Verification Time:** <5ms

### 5.2 Proof Types

- **Proof of Ballot Validity:** Ballot is well-formed
- **Proof of Inclusion:** Ballot included in final tally
- **Proof of Correct Tabulation:** Results accurately reflect ballots

---

## 6. Key Management

### 6.1 Key Generation Ceremony

Public ceremony with multiple independent trustees:
1. Public announcement with observer invitations
2. Each trustee generates key share independently
3. Shares combined using threshold cryptography
4. Public key published and signed by all trustees

### 6.2 Decryption Ceremony

Results decrypted through public ceremony:
1. Encrypted ballots finalized and hashed
2. Threshold trustees convene publicly
3. Each trustee applies key share
4. Results fully decrypted when threshold reached

---

## 7. Integrity Protection

### 7.1 Tamper Detection

- Cryptographic hashes detect data modification
- Digital signatures verify authenticity
- Blockchain immutability prevents historical changes
- Hardware security modules protect cryptographic operations

### 7.2 Coercion Resistance

- Receipt-free voting
- Fake credential generation capability
- Revote options during voting period
- Vote cancellation through secure process

---

**© 2025 World Certification Industry Association (WIA)**  
**弘益人間 (Hongik Ingan) - Benefit All Humanity**

## P.3 Protocol Cross-References

The protocol defined here carries the data formats from Phase 1 and the API
operations from Phase 2 across trust boundaries. Phase 4 describes how the
protocol composes with adjacent infrastructure.

### P.3.1 Transport Bindings

| Binding | Default Port | Use |
|---------|-------------:|-----|
| HTTP/2 + TLS 1.3 | 443 | Public, request-response |
| HTTP/3 (QUIC) | 443 | Mobile, lossy networks |
| gRPC | 443 | Service-to-service |
| MQTT 5.0 | 8883 | Constrained / IoT devices |
| AMQP 0-9-1 | 5671 | Backplane / event streams |

### P.3.2 Message Envelope

Every protocol message carries a small envelope independent of payload:

```
+----------------+------------------+--------------------+
| message_id     | UUIDv4           | RFC 4122           |
| trace_id       | 16-byte hex      | W3C Trace Context  |
| span_id        | 8-byte hex       | W3C Trace Context  |
| origin_node    | DNS name or NIN  | RFC 1035           |
| issued_at      | RFC 3339         | UTC required       |
| ttl_seconds    | uint32           | 0 = no expiry      |
| content_type   | media type       | RFC 6838           |
| body           | opaque bytes     | per content_type   |
+----------------+------------------+--------------------+
```

### P.3.3 Reliability Model

The protocol provides at-least-once delivery by default. Receivers
deduplicate by `message_id`. Exactly-once semantics are achieved when both
peers participate in the idempotency contract from Phase 2 §P.2.3.

### P.3.4 Backpressure

Senders MUST honour HTTP `Retry-After`, gRPC `RESOURCE_EXHAUSTED`, or MQTT
flow-control packets. The recommended back-off is full jitter exponential with
cap 30 s and cumulative cap 5 min.


---

## Appendix · Common WIA Standard Provisions

> The following provisions apply to every WIA standard and are kept in sync
> across the WIA-Standards corpus. Standard-specific deviations, where they
> exist, are listed in the standard's normative body.

### A. Conformance & Compliance

#### A.1 Conformance Levels

WIA-Standards defines four conformance levels:

| Level | Required | Description |
|-------|----------|-------------|
| L1 — Format | Phase 1 | Implementation produces and consumes the canonical data format losslessly |
| L2 — Interface | Phase 1 + 2 | Implementation exposes the API surface with required behaviour |
| L3 — Protocol | Phase 1 + 2 + 3 | Implementation interoperates over at least one normative transport binding |
| L4 — Integration | All Phases | Implementation passes the conformance test suite end-to-end in a production-shaped deployment |

Conformance claims MUST cite the level and the version of the standard
against which the claim is made (e.g. "L3 conformant against v1.0").

#### A.2 Compliance Verification

The conformance test suite is published alongside this standard at
`/cli/conformance/` and `/api/conformance/`. Implementations claiming
L2 or higher MUST publish their test report. Independent re-tests are
encouraged; the WIA Working Group accepts third-party verification reports
under the policy in §E.

### B. Security Considerations

#### B.1 Threat Model

Implementers SHOULD apply STRIDE analysis covering: spoofing of identity,
tampering with messages or stored state, repudiation of operations,
information disclosure, denial of service, and elevation of privilege.

| Threat | Default Control | Where Strengthened |
|--------|-----------------|--------------------|
| Spoofing | Mutual TLS or signed tokens | Phase 3 §P.3 |
| Tampering | TLS in transit, AEAD at rest | Phase 1 §P.1 |
| Repudiation | Append-only audit log with notarization | Phase 4 §P.4 |
| Disclosure | Field-level encryption for PII / secrets | Phase 1 §P.1 |
| DoS | Rate limit per principal & global circuit breaker | Phase 2 §P.2 |
| EoP | Least-privilege RBAC + scoped tokens | Phase 2 §P.2 |

#### B.2 Cryptographic Suites

Mandatory: TLS 1.3 with AEAD ciphers (AES-128-GCM, AES-256-GCM,
CHACHA20-POLY1305). Forbidden: TLS 1.0, TLS 1.1, RC4, MD5, SHA-1 for
signatures, RSA below 2048 bits, ECDSA on curves smaller than P-256.

Post-quantum migration: implementations SHOULD adopt hybrid key
exchange combining a classical primitive with ML-KEM (FIPS 203) once a
profile is published; signature migration to ML-DSA (FIPS 204) is
expected within the L4 conformance window of v2.0.

#### B.3 Audit Requirements

L3 and L4 implementations MUST log: (a) every authentication decision,
(b) every authorization decision, (c) every state-changing operation,
(d) every export of data outside its sovereignty boundary. Logs are
write-once for at least 1 year and 90 days indexed for incident search.

### C. Versioning & Lifecycle

Versions follow Semantic Versioning 2.0.0 (MAJOR.MINOR.PATCH).

| Phase | Duration | Conformance Status |
|-------|---------:|-------------------|
| Draft | until ratification | Non-binding |
| Active | indefinite | Binding for new deployments |
| Maintenance | 24 months from successor's Active date | Binding for existing deployments |
| Retired | indefinite | Non-binding; conformance claims rescinded |

Deprecation MUST be announced at least one minor version before a feature
is removed in a major version.

### D. Internationalization & Accessibility

Implementations SHOULD support locale negotiation via the
`Accept-Language` header (RFC 4647). Date, time, number, and currency
formatting follow CLDR. User-facing surfaces MUST satisfy WCAG 2.1 AA at
minimum and SHOULD progress towards WCAG 2.2 AA. Right-to-left scripts
(Arabic, Hebrew, Persian, Urdu) and East-Asian wide characters MUST be
laid out correctly without line-breaking heuristics that split graphemes.

### E. Governance & IP Policy

This standard is maintained by the WIA Working Group under the WIA
governance charter. Editorial changes are merged via pull request. Normative
changes require working-group consensus and a 30-day public review.
Contributions are accepted under the Apache License 2.0 with explicit
patent grant. Members participate under the WIA Patent Policy,
which requires royalty-free licensing of any essential claim necessary
to implement a normative requirement.

### F. Normative References

The following references are normative; implementations MUST satisfy
the cited clauses:

- ISO/IEC 27001:2022 — Information security management systems
- ISO/IEC 27017:2015 — Cloud-services security controls
- ISO/IEC 27701:2019 — Privacy information management
- ISO/IEC 19790:2012 — Security requirements for cryptographic modules
- ISO 8601-1:2019 — Date and time representation
- IETF RFC 8446 — TLS 1.3
- IETF RFC 7519 — JSON Web Token
- IETF RFC 6749 — OAuth 2.0
- IETF RFC 9110 — HTTP Semantics
- IETF RFC 9112 — HTTP/1.1 message syntax
- IETF RFC 9113 — HTTP/2
- IETF RFC 9114 — HTTP/3
- IETF RFC 9000 — QUIC transport
- IETF RFC 4122 — UUID URN namespace
- IETF RFC 3339 — Date and time on the Internet
- IETF RFC 6838 — Media-type specifications and registration
- W3C TraceContext — Distributed tracing context
- W3C WCAG 2.1 — Accessibility guidelines
- FIPS PUB 197 — AES
- FIPS PUB 180-4 — SHA-2 family
- FIPS PUB 203 — ML-KEM (post-quantum KEM)
- FIPS PUB 204 — ML-DSA (post-quantum signature)

### G. Glossary

| Term | Definition |
|------|------------|
| Conformance | The state of satisfying every normative requirement at a given level |
| Implementation | A software, hardware, or composite artefact that claims conformance |
| Principal | The authenticated entity bound to a security context |
| Subject | The resource or person to which an operation applies |
| Sovereignty Boundary | The legal / regulatory perimeter outside of which data export is restricted |

---

*This Appendix is authored by the WIA Standards Working Group and is kept
in lockstep across Phases 1–4 of voting-system so that conformance claims at any
Phase remain unambiguous.*


---

## Annex F · Conformance Test Vectors and Worked Examples

The following test vectors and end-to-end examples are normative for L2
and higher conformance. Implementations MUST reproduce every byte of every
output vector when given the corresponding input vector under the
deterministic inputs specified.

### F.1 Determinism Requirements

Implementations MUST be deterministic for the following operations:

| Operation | Determinism Source |
|-----------|--------------------|
| Canonical JSON serialization | RFC 8785 (JCS) |
| Hash and digest computation | FIPS 180-4 SHA-256 / SHA-512 |
| UUID v4 in test vectors | Seeded PRNG (xorshift64*, seed disclosed in vector) |
| Floating-point reduction | IEEE 754 round-to-nearest-even |
| Time-dependent fields in tests | Frozen clock value disclosed in vector |

### F.2 Round-Trip Vector — Canonical Object

Given the canonical input object as Phase 1 §2 schema, the canonical
serialization MUST match the byte sequence below exactly:

```text
{"id":"00000000-0000-4000-8000-000000000001","version":"1.0.0"}
```

The SHA-256 of this byte sequence (no trailing newline) is:
`64a8ed8a … 5b3f`. Implementations MUST emit the same digest when applied
to the same input.

### F.3 Negative Vector — Schema Violation

Given an input that violates the schema in Phase 1 §3 (missing required
field `id`), the conforming validator MUST emit error code `E_SCHEMA_001`
with field-path pointer `$.id` and the human-readable message
"required property 'id' is missing".

### F.4 Boundary Vector — Maximum Sizes

| Field | Maximum | Behavior on Excess |
|-------|---------|--------------------|
| `string` (default) | 4 KiB | reject with `E_LIMIT_STRING` |
| `bytes` (inline) | 1 MiB | reject with `E_LIMIT_BYTES` |
| nested object depth | 32 | reject with `E_LIMIT_DEPTH` |
| array length (default) | 65,536 | reject with `E_LIMIT_ARRAY` |
| number magnitude | 2^53−1 | reject with `E_LIMIT_NUMBER` |

Implementations MAY raise these limits per deployment, provided the
operative limit is published in the conformance report.

### F.5 Locale & Time-zone Vector

The canonical example timestamp `2026-04-25T03:30:00Z` MUST render
identically in every locale tested by the conformance suite:

| Locale | Rendering |
|--------|-----------|
| `en-US` | `April 25, 2026, 3:30 AM UTC` |
| `ko-KR` | `2026년 4월 25일 오전 3시 30분 (협정 세계시)` |
| `ja-JP` | `2026年4月25日 3時30分 (協定世界時)` |
| `de-DE` | `25. April 2026, 03:30 Uhr UTC` |
| `ar-SA` | right-to-left layout; Hijri calendar tag MAY be appended |

### F.6 Replay Protection Vector

Given a request signed with a nonce that has been seen within the last
300 seconds, the receiver MUST respond with status `409 Conflict` and
error body `{"error":"E_REPLAY","retry_after_seconds":<n>}` where `<n>`
is the number of seconds until the nonce window slides past the offending
nonce.

### F.7 Conformance Report Schema

Every conformance claim MUST be accompanied by a machine-readable report
matching the schema below:

```yaml
report:
  standard: "<standard-id>"
  version: "<MAJOR.MINOR.PATCH>"
  level: "L1|L2|L3|L4"
  implementation:
    name: "<vendor>/<product>"
    build: "<git sha or build id>"
    deployment_topology: "<topology from Phase 4 §P.4.1>"
  results:
    pass: <integer>
    fail: <integer>
    skip: <integer>
  signed_by: "<DN of signing authority>"
  signed_at: "<RFC 3339 timestamp>"
  signature: "<base64 detached signature>"
```

### F.8 Audit Trail Sample

A conforming implementation produces audit records of the following
shape for every state-changing operation:

```json
{
  "audit_id": "01HFXX0000000000000000000",
  "timestamp": "2026-04-25T03:30:00.123Z",
  "principal": "service-account/api-gateway",
  "action": "resource.update",
  "subject": "/v1/things/<id>",
  "request_digest": "sha256:9b1a…",
  "response_status": 200,
  "trace_id": "0af7651916cd43dd8448eb211c80319c"
}
```

Audit records are append-only and signed in batches of at most 1000
records per chain link using the cryptographic suite from Annex B.

