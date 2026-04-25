# WIA-TIME-014 (Data Time Transport) — Phase 3: Protocol Specification

> **Version:** 1.0.0
> **Status:** Official
> **Phase:** 3 of 4 (Protocol)
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

Phase 3 specifies the wire protocols used by WIA-TIME-014 deployments. The objective is to give a deterministic mapping of every Phase-2 verb to a concrete byte-level behaviour and to fix the operational interoperability points required by RFC 3161 (TSP), RFC 4998 (ERS), and ISO/IEC 18014 *Time-stamping services*, within the ISO/IEC 27040 *Storage security* baseline.

### 1.1 Protocol stack

```
+---------------------------------------------------+
| WIA-TIME-014 application semantics (Phase 1+2)    |
+---------------------------------------------------+
| Mediation: gateway, TSA client, capsule arbiter   |
+---------------------------------------------------+
| Transport (one of):                               |
|   HTTP/REST (Phase 2 §2)                          |
|   CoAP/UDP/IP                                     |
|   RFC 3161 TSP over HTTPS                         |
+---------------------------------------------------+
| Cryptographic: TLS 1.3, COSE, CMS                 |
+---------------------------------------------------+
| Storage: ISO/IEC 27040 controlled volumes         |
+---------------------------------------------------+
```

---

## 2. Deposit Workflow

### 2.1 Submission

A depositor calls `POST /domains/{id}/deposits` with the canonical Phase-1 *Deposit* JSON. The gateway:

1. Validates the deposit against the Phase-1 schema.
2. Checks the depositor's verb scope (`data-time:deposit`).
3. Issues an RFC 3161 `TimeStampReq` for the supplied `contentDigest` to the bound TSA.
4. Persists the resulting `TimeStampToken` and binds it to the new deposit.
5. Returns the assigned `depositId` and the token URI.

### 2.2 Storage

Content uploads are routed to the chosen `storageVolumeRefs`. The gateway MUST:

- Encrypt content at rest per the volume's `encryptionAtRest` field.
- Honour the volume's ISO/IEC 27040 control set.
- Log the storage placement in the audit log.

### 2.3 Multi-volume placement

Deposits MAY reference multiple storage volumes for redundancy. The gateway MUST surface placement failures as warnings on the deposit's audit trail and MUST trigger a re-placement attempt within the operator-configured retry window.

---

## 3. Capsule Workflow

### 3.1 Construction

A *Capsule* is constructed by `POST /domains/{id}/capsules` with the deposit identifiers and the release-policy reference. The gateway:

1. Validates that all referenced deposits exist and are not already capsuled in a conflicting way.
2. Validates the release policy.
3. Computes the `earliestReleaseAt` and (optionally) `latestReleaseAt` from the policy conditions.
4. Persists the capsule in `state=PENDING`.

### 3.2 State transitions

The capsule progresses through states:

```
PENDING → ELIGIBLE → RELEASED
       ↘ REVOKED
```

The transition to ELIGIBLE happens automatically when the gateway evaluates the release policy and finds it satisfied. The transition to RELEASED requires an explicit `:release` operation (Phase 2 §5.3) that surfaces the release evidence.

### 3.3 Release evidence

Release evidence is a COSE_Sign1 (RFC 9052) wrapper over a manifest containing:

- `capsuleId`.
- `releasedDepositIds`.
- `releasedAt` — RFC 3339 date-time.
- `policyConditionsSatisfied` — array of structured assertions.
- `actorRef` — URI of the operator who initiated the release.
- `tsaTokenRef` — URI of a fresh RFC 3161 token over the manifest digest.

### 3.4 Revocation

A capsule MAY be revoked while in `state=PENDING`. Revocation is itself a state-change event recorded in the audit log; revocation does NOT delete the underlying deposits. Capsules in `state=ELIGIBLE` MAY be revoked only with privileged authorisation; capsules in `state=RELEASED` MUST NOT be revoked.

---

## 4. Retention and Tier Transition Workflow

### 4.1 Retention enforcement

The gateway evaluates retention policies on a periodic cadence (default: hourly). For each deposit:

- If the retention horizon has elapsed and the destruction policy is `PHYSICAL` or `BOTH`, the deposit is queued for sanitisation per ISO/IEC 27040 §6.7.
- If the destruction policy is `CRYPTOGRAPHIC` or `BOTH`, the deposit's encryption key is queued for cryptographic erasure.
- The audit log records every queued action.

### 4.2 Sanitisation

Physical sanitisation of storage media follows ISO/IEC 27040 §6.7 *Storage sanitisation*. Media MUST NOT be reused for non-equivalent classification levels until sanitisation is documented as complete.

### 4.3 Cryptographic erasure

Cryptographic erasure relies on the operator's key-management procedure (ISO 11770-2 / ISO 11770-3). The gateway MUST surface erasure-completion through the audit log and MUST refuse retrieval of any deposit whose key has been erased.

### 4.4 Tier transition

Tier transitions HOT → WARM → COLD → FROZEN MUST:

- Issue a fresh RFC 3161 token over the deposit's content digest at the new tier.
- Update the StorageVolume reference on the deposit.
- Trigger renewal of the bound evidence record per Phase 3 §3 of WIA-TIME-011 (mirrored here).

---

## 5. Identity, Time, and Cryptography

### 5.1 Identity

- **Depositors and auditors**: federated identity over OAuth 2.1 (RFC 9700) and OpenID Connect.
- **Storage volumes**: machine identities under the deployment PKI (RFC 5280).
- **TSA**: external X.509 v3 certificate (RFC 5280) with the *id-kp-timeStamping* EKU.

### 5.2 Time

The TSA's clock is the authoritative time anchor for every deposit and release event. The TSA's clock MUST be slaved to a traceable time source per ISO/IEC 18014-4:2015. Operators MAY layer NTPv4 with NTS (RFC 5905, RFC 8915) on top of the ISO/IEC 18014-4 traceability chain.

### 5.3 Cryptographic algorithms

| Layer | Algorithm | Reference |
|-------|-----------|-----------|
| TLS | TLS 1.3 cipher suites | RFC 8446 |
| Hashing | SHA-256 / SHA-384 / SHA-512 / SHA-3 | FIPS 180-4, FIPS 202 |
| TSA signature | RSA-PSS, ECDSA, EdDSA | RFC 5652, RFC 8410 |
| COSE signature | ES256, EdDSA | RFC 9053 |
| At-rest storage | AES-256-GCM | ISO/IEC 18033-3, FIPS 197 |
| Key management | ISO 11770-2 / ISO 11770-3 | ISO 11770 |

Implementations MUST refuse cipher suites whose IETF status is "not recommended" for new deployments.

---

## 6. Failure Handling

### 6.1 TSA unavailable

When the bound TSA is unavailable, the gateway MUST queue deposit-time-stamp requests up to the configured retention limit, surface the queued state through the health endpoint, and return `503 Service Unavailable` with a `Retry-After` header for new deposit requests that cannot be queued.

### 6.2 Storage volume unavailable

When a storage volume is unavailable, the gateway MUST refuse new placements on that volume and MUST attempt re-placement on the deposit's secondary volumes. Persistent unavailability triggers an audit-log alarm and operator notification.

### 6.3 Release-policy unmet

If a release attempt encounters an unmet policy condition, the gateway MUST return `422 Unprocessable Entity` with the `data-time/release-policy-unmet` problem and a structured `errors` field listing the unmet conditions.

### 6.4 Cryptographic failure

Failed signature verification, expired certificate, or OCSP "revoked" status on any of the deployment's critical certificates MUST disable the affected operation paths until the issue is resolved.

### 6.5 Renewal failure

Renewal failures MUST surface through the health endpoint and MUST NOT silently delete or corrupt the existing evidence record. The operator's incident-response playbook governs the next steps.

---

## 7. Conformance Profiles

### 7.1 Baseline profile (P-B)

A P-B gateway MUST:

- Implement the HTTP/REST surface (Phase 2 §2).
- Implement deposit and capsule workflows (§2 / §3).
- Implement RFC 4998 evidence-record build for any deposit older than the operator's renewal cadence.
- Issue COSE-signed audit-log digests.

### 7.2 Constrained-depositor profile (P-CD)

A P-CD gateway MUST additionally:

- Implement CoAP surface (Phase 2 §3).
- Implement OSCORE (RFC 8613) for end-to-end protection.
- Implement CoRE Resource Directory registration (RFC 9176).

### 7.3 Long-term-archive profile (P-LTA)

A P-LTA gateway MUST additionally:

- Implement the renewal triggers (mirrored from Phase 3 §3.3 of WIA-TIME-011).
- Maintain a documented renewal log per domain.
- Validate every renewal cycle against ISO/IEC 18014-4 traceability records.

---

## 8. Conformance Tests

The conformance suite includes:

- **TSP round-trip** vectors per RFC 3161.
- **ERS Merkle reconstruction** tests per RFC 4998.
- **Capsule workflow** tests covering PENDING → ELIGIBLE → RELEASED and the REVOKED branch.
- **Tier transition** tests covering HOT → WARM → COLD → FROZEN.
- **TLS 1.3 cipher inventory** test against the RFC 8446 mandatory profile.
- **NTS time-sync** test against an RFC 8915 reference server.
- **Sanitisation** test that simulates physical sanitisation and verifies audit-log completeness.

A conformant gateway MUST pass every vector in the suite for its declared profile set.

---

## 9. References

1. ISO/IEC 18014 (all parts); ISO/IEC 18033-3:2010; ISO/IEC 27001:2022; ISO/IEC 27037:2012; ISO/IEC 27040:2024; ISO/IEC 27042:2015.
2. ISO 11770-2; ISO 11770-3.
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
13. RFC 8613 — *OSCORE.*
14. RFC 8949 — *CBOR.*
15. RFC 9052; RFC 9053 — *COSE.*
16. RFC 9110; RFC 9147; RFC 9457 — *HTTP, DTLS 1.3, problem details.*
17. RFC 9176 — *CoRE Resource Directory.*
18. RFC 9700 — *OAuth 2.1.*
19. FIPS 180-4; FIPS 197; FIPS 202.

---

## 10. Detailed Examples

### 10.1 Deposit at deposit time

A depositor submits a 1.2 MB document for cold storage with a 50-year retention. The gateway validates the deposit, requests a TSA token over the SHA-256 digest, persists the document on a COLD-tier volume with AES-256-GCM at rest, and returns a deposit URI plus the token URI.

### 10.2 Time-capsule release

A capsule with a release policy `kind=TIME-ABSOLUTE; afterRfc3339=2050-12-31T23:59:59Z` becomes ELIGIBLE on 2051-01-01. An operator with the `data-time:release-capsule` verb posts to the `:release` endpoint with the operator's justification. The gateway evaluates the policy, builds the release evidence, fetches a fresh TSA token over the manifest, and returns the signed release evidence.

### 10.3 Cryptographic erasure on retention expiry

A deposit with a 7-year retention policy and `destructionPolicy=CRYPTOGRAPHIC` reaches its retention horizon. The gateway queues the deposit's encryption key for erasure, completes the erasure within the operator's procedure, and writes an audit-log entry. Subsequent retrieval attempts return `410 Gone` with the `data-time/evidence-renewal-required` problem code if the deposit is also subject to renewal, or a domain-specific `gone` status if the erasure is final.

---

## 11. Latency Budgets

The gateway MUST publish, per service tier, the following latency budgets (informative for HOT and WARM tiers, normative for COLD and FROZEN):

| Operation | HOT | WARM | COLD | FROZEN |
|-----------|-----|------|------|--------|
| Deposit ingest | ≤ 500 ms 95p | ≤ 2 s 95p | ≤ 30 s 95p | ≤ 5 min 95p |
| Content retrieval | ≤ 1 s 95p | ≤ 5 s 95p | ≤ 24 h 95p | ≤ 7 d 95p |
| Capsule release evidence | ≤ 5 s 95p | ≤ 15 s 95p | ≤ 60 s 95p | ≤ 5 min 95p |
| Tier transition | best-effort | best-effort | best-effort | best-effort |

The latency table is per-tier; a deposit's effective budget is the budget of its lowest-latency volume reference.

---

## 12. Profile Selection Guidance

Operators choose a conformance profile based on the deployment's role:

- **General records management (P-B)** — deposit and retention; capsules optional.
- **IoT-class depositors (P-CD)** — constrained-depositor profile; CoAP and OSCORE on the depositor side.
- **Long-term archives (P-LTA)** — adds renewal cadence so evidence remains verifiable across hash and certificate transitions.

A deployment MAY combine profiles. Combined profiles MUST surface every constituent tag in Phase 4 §10.