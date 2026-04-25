# WIA-TIME-014 (Data Time Transport) — Phase 2: API Interface Specification

> **Version:** 1.0.0
> **Status:** Official
> **Phase:** 2 of 4 (API Interface)
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

This phase defines the network-facing API of WIA-TIME-014. Two surfaces are specified:

- **HTTP/REST** — for management plane operations (deposit, capsule, release-policy, retention-policy, audit), built on RFC 9110 (HTTP semantics) with TLS 1.3 (RFC 8446) as the only permitted carrier.
- **CoAP** — for constrained depositors (e.g. IoT sensors that deposit data summaries directly), built on RFC 7252 with OBSERVE (RFC 7641), block transfer (RFC 7959), DTLS 1.3 (RFC 9147), or OSCORE (RFC 8613).

OpenAPI 3.1 documents the HTTP/REST surface. The CoAP surface uses a CoRE Resource Directory descriptor (RFC 9176).

### 1.1 Authentication and authorization

| Concern | Mechanism | Reference |
|---------|-----------|-----------|
| Token format | JWT with PoP, or CWT | RFC 7519, RFC 7800; RFC 8392 |
| Authorization framework | OAuth 2.1 | RFC 9700 (BCP) |
| Channel security (HTTP) | TLS 1.3 | RFC 8446 |
| Channel security (CoAP) | DTLS 1.3 or OSCORE | RFC 9147; RFC 8613 |
| At-rest token integrity | COSE_Sign1 | RFC 9052 |

Tokens MUST carry an `aud` claim equal to the canonical domain URI from Phase 1 §2.1, and a `scope` string composed of one or more verbs from §4.

---

## 2. HTTP/REST Surface

### 2.1 Base URL

```
https://<host>/wia-time-014/v1
```

Servers MUST advertise version 1 of this specification through the `Server` header and the `/.well-known/wia-data-time` document (RFC 8615 Well-Known URIs).

### 2.2 Resource map

| Resource | Methods | Purpose |
|----------|---------|---------|
| `/domains` | GET | List domains visible to caller |
| `/domains/{id}` | GET, PATCH | Domain descriptor |
| `/domains/{id}/deposits` | GET, POST | Deposit collection |
| `/domains/{id}/deposits/{depositId}` | GET | Deposit descriptor |
| `/domains/{id}/deposits/{depositId}/content` | GET | Retrieve raw content (subject to release policy) |
| `/domains/{id}/capsules` | GET, POST | Capsule collection |
| `/domains/{id}/capsules/{capsuleId}` | GET, PATCH | Capsule descriptor |
| `/domains/{id}/capsules/{capsuleId}:release` | POST | Initiate a release attempt |
| `/domains/{id}/capsules/{capsuleId}:revoke` | POST | Revoke a pending capsule |
| `/domains/{id}/release-policies` | GET, POST | Release policies |
| `/domains/{id}/retention-policies` | GET, POST | Retention policies |
| `/domains/{id}/storage-volumes` | GET, POST | Storage volumes |
| `/domains/{id}/evidence-records` | GET, POST | Evidence records |
| `/domains/{id}/auditors` | GET, POST | Auditor records |
| `/domains/{id}/audit` | GET | Audit log per IEC 62443-3-3 SR 6.1 |
| `/domains/{id}/health` | GET | Liveness / readiness |

`PATCH` requests use JSON Merge Patch (RFC 7396) by default and JSON Patch (RFC 6902) when `Content-Type: application/json-patch+json`.

### 2.3 Errors

Errors use *Problem Details for HTTP APIs* (RFC 9457). The minimum problem types:

| Code | HTTP | Meaning |
|------|------|---------|
| `data-time/forbidden-domain` | 403 | Caller lacks domain privilege |
| `data-time/release-policy-unmet` | 422 | Release attempted before policy conditions are satisfied |
| `data-time/capsule-state-conflict` | 409 | Capsule state forbids the requested operation |
| `data-time/storage-volume-unavailable` | 503 | Underlying storage volume unavailable |
| `data-time/lawful-basis-required` | 403 | Retention policy requires lawful-basis record |
| `data-time/evidence-renewal-required` | 410 | Evidence record needs re-time-stamping |
| `data-time/quota-exceeded` | 429 | Tenant exceeded contracted plan |

### 2.4 Pagination and conditional requests

Cursor-based pagination with `Link` headers per RFC 8288. ETag/If-Match/If-None-Match per RFC 9110 §13.

### 2.5 Long-running operations

Bulk imports, mass-renewal, and tier transitions are long-running. They follow `202 Accepted` + status-URI pattern, with optional `Prefer: respond-async` (RFC 7240).

---

## 3. CoAP Surface

### 3.1 URI scheme

```
coaps://<gateway-host>/wia-data-time/v1/{path}
```

Resource paths mirror the HTTP surface but elide `domains/{id}` because each gateway is bound to a single domain.

### 3.2 Methods and options

CoAP methods (RFC 7252 §5.8) map directly: GET, POST, PUT, DELETE. Resource representations use CBOR (RFC 8949) with COSE_Sign1 / COSE_Encrypt0 wrappers (RFC 9052 / 9053) when end-to-end protection is required. Block-wise transfers (RFC 7959) MUST be supported for any payload exceeding 1024 bytes.

### 3.3 Resource Directory

Gateways MUST register their resource set with a CoRE Resource Directory (RFC 9176). Resource types:

- `rt="wia.dt.deposit"` — deposit endpoint
- `rt="wia.dt.capsule"` — capsule endpoint
- `if="wia.dt.observe"` — capsule-state observe interface

---

## 4. Verbs and Scopes

| Verb | Capability |
|------|------------|
| `data-time:read` | Read catalog (deposits, capsules, policies) |
| `data-time:deposit` | Create new deposits |
| `data-time:retrieve-content` | Retrieve deposit content (subject to release policy) |
| `data-time:author-capsule` | Create / edit capsules |
| `data-time:release-capsule` | Attempt capsule release |
| `data-time:revoke-capsule` | Revoke pending capsules |
| `data-time:author-policy` | Author release / retention policies |
| `data-time:tier-transition` | Initiate storage tier transitions |
| `data-time:audit-read` | Read audit log |
| `data-time:auditor-administration` | Manage Auditor records |

Each verb MUST be enforced at every surface. Tokens missing a required verb produce HTTP 403 / CoAP 4.03 with the `data-time/forbidden-domain` problem code.

---

## 5. Sample Operations

### 5.1 Create a deposit

```
POST /wia-time-014/v1/domains/dm-archive/deposits HTTP/1.1
Authorization: DPoP <token>
Content-Type: application/json

{"contentDigest":{"algorithm":"SHA-256","value":"..."},"contentType":"application/pdf","contentSizeBytes":1234567,"storageVolumeRefs":["wia-data-time://dm-archive/storage-volumes/cold-1"]}
```

The response includes the assigned `depositId`, the bound TimeStampToken URI, and the signed receipt.

### 5.2 Create a capsule

```
POST /wia-time-014/v1/domains/dm-archive/capsules HTTP/1.1
Content-Type: application/json

{"depositIds":["dp-001","dp-002"],"releasePolicyRef":"wia-data-time://dm-archive/release-policies/2050-anniversary"}
```

The response includes the assigned `capsuleId` and `state=PENDING`.

### 5.3 Attempt a release

```
POST /wia-time-014/v1/domains/dm-archive/capsules/cap-099:release HTTP/1.1
Content-Type: application/json

{"justification":"Anniversary date reached","approverIds":["op-1","op-2"]}
```

If the policy conditions are satisfied, the response is `200 OK` with the release evidence. If not, the response is `422` with the `data-time/release-policy-unmet` problem code.

---

## 6. Conformance

A WIA-TIME-014 v1 *gateway* MUST implement:

- HTTP/REST surface (§2).
- Verbs from §4 with audit logging on every state-changing call.
- Problem details from §2.3.
- Evidence-record build (RFC 4998) when the deployment uses long-term archival.

A WIA-TIME-014 v1 *client* MUST:

- Honour ETag-based concurrency.
- Tolerate unknown problem-detail extensions per RFC 9457 §4.4.
- Validate received evidence records against the supplied COSE detached signatures.

---

## 7. Operational Notes

### 7.1 Idempotency

Mutating verbs accept `Idempotency-Key`. Servers retain the response per key for 24 hours.

### 7.2 Discovery

`/domains/{id}/.well-known/wia-data-time` returns supported verbs, supported encodings, supported storage tiers, and the conformance tags from Phase 4.

### 7.3 Health endpoint

`/domains/{id}/health` MUST return `{state, version, uptimeSeconds, deposits:{total, byTier}, capsules:{pending, eligible, released}, storageVolumes:{total, healthy}, audit:{lastEntryAt, queueDepth}}`.

### 7.4 Backpressure

Token-bucket rate limiting with the IETF `RateLimit-*` headers. Tier-transition operations and bulk renewals MUST be subject to a per-domain burst limit.

### 7.5 Versioning

Major version bumps require a parallel HTTP path (`/wia-time-014/v2`). Minor and patch revisions are backwards-compatible within `/v1`.

### 7.6 Release event durability

Release events MUST be written to durable storage and to the audit log before the release operation returns success. Partial-release semantics are forbidden.

---

## 8. Discovery Document Schema

```json
{
  "version": "string (semver)",
  "supportedVerbs": ["array of verb tokens from §4"],
  "supportedDigestAlgorithms": ["SHA-256", "SHA-384", "SHA-512", "SHA3-256", "SHA3-512"],
  "supportedStorageTiers": ["HOT", "WARM", "COLD", "FROZEN"],
  "supportedEncodings": ["json", "cbor"],
  "tsaIdentifier": "string (URI of the bound TSA)",
  "openApiUrl": "string (URI)",
  "coreResourceDirectoryUrl": "string (URI) | null",
  "conformanceTags": ["array of conformance tags from Phase 4 §10"]
}
```

---

## 9. References

1. RFC 6902; RFC 7396 — *JSON Patch / Merge Patch.*
2. RFC 7240 — *Prefer Header for HTTP.*
3. RFC 7252; RFC 7641; RFC 7959 — *CoAP family.*
4. RFC 7519; RFC 7800; RFC 8392 — *JWT, PoP, CWT.*
5. RFC 8259; RFC 8610; RFC 8615; RFC 8949 — *JSON, CDDL, well-known URIs, CBOR.*
6. RFC 8288 — *Web Linking.*
7. RFC 8446; RFC 9147 — *TLS 1.3, DTLS 1.3.*
8. RFC 8613 — *OSCORE.*
9. RFC 9052; RFC 9053 — *COSE.*
10. RFC 9110; RFC 9457 — *HTTP semantics, problem details.*
11. RFC 9176 — *CoRE Resource Directory.*
12. RFC 9700 — *OAuth 2.1.*
13. RFC 9562 — *UUIDs.*
14. RFC 3161 — *TSP.*
15. RFC 4998 — *ERS.*
16. ISO/IEC 18014 (all parts); ISO/IEC 27001:2022; ISO/IEC 27037:2012; ISO/IEC 27040:2024.
17. IEC 62443-3-3:2013.

---

## 10. Compatibility With External Verifiers

The Phase-2 surface is designed so that any RFC 3161 TSP verifier can validate the bound `TimeStampToken` for a deposit without re-encoding, and any RFC 4998 ERS verifier can validate the bound evidence record. The deployment MUST NOT introduce custom envelopes that prevent direct interoperability with these external verifiers.

---

## 11. Capsule Discovery

Auditors MAY discover capsules eligible for release through the read-only endpoint `GET /domains/{id}/capsules?state=ELIGIBLE`. Servers MUST honour cursor pagination and MUST NOT include capsules whose release policy involves personal data unless the auditor's verb scope explicitly permits the disclosure.

---

## 12. Receipt Retrieval

Depositors retrieve their signed Receipts (Phase 1 §9) through `GET /domains/{id}/deposits/{depositId}/receipt`. Receipts are immutable; the endpoint returns `200 OK` with the Receipt body and a strong ETag.

---

## 13. Versioning Policy

Major version bumps require a parallel HTTP path (`/wia-time-014/v2`). Minor and patch revisions are backwards-compatible within `/v1`. Clients MUST tolerate unknown fields and unknown problem-detail extensions per RFC 9457 §4.4. Breaking changes to the Receipt structure (Phase 1 §9) require a major bump even when the rest of the surface is unchanged.

---

## 14. Audit Log Read Contract

The `/domains/{id}/audit` endpoint exposes the audit log per IEC 62443-3-3 SR 6.1. Read access requires the `data-time:audit-read` verb. The endpoint supports cursor pagination and time-range filtering via query parameters `from` and `to` (RFC 3339 date-times). Audit-log entries are append-only and carry COSE_Sign1 detached signatures over a canonical entry digest, allowing third-party auditors to verify the log's integrity.

---

## 15. Audit Log Schema

Each audit-log entry MUST include the following normative fields:

- `entryId` — opaque identifier, monotonically increasing within the log.
- `entryDigestPrev` — digest of the previous entry, forming a hash chain.
- `at` — RFC 3339 date-time.
- `actorRef` — URI of the actor that triggered the entry.
- `verb` — Phase-2 verb that triggered the entry.
- `subject` — URI of the affected entity.
- `outcome` — `OK | DENIED | ERROR`.
- `signature` — COSE_Sign1 detached over the canonical entry digest.

Entries MUST NOT be deleted or rewritten; corrections MUST be expressed as new entries that reference the corrected entry by `entryId`.
