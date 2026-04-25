# WIA-TIME-011 (Historical Integrity) — Phase 2: API Interface Specification

> **Version:** 1.0.0
> **Status:** Official
> **Phase:** 2 of 4 (API Interface)
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

This phase defines the network-facing API of WIA-TIME-011. Three coordinated surfaces are specified:

- **HTTP/REST** — for management plane operations (event submission, ledger queries, evidence-record retrieval, verifier administration), built on RFC 9110 (HTTP semantics) with HTTPS over TLS 1.3 (RFC 8446) as the only permitted carrier.
- **RFC 3161 TSP** — for direct time-stamp requests and responses against a TSA, in the canonical RFC 3161 ASN.1 form transported over HTTPS.
- **RFC 9162 CT v2** — for transparency-log read endpoints (`get-sth`, `get-proof-by-hash`, `get-entries`), in CT v2's canonical form.

OpenAPI 3.1 documents the HTTP/REST surface; the TSP and CT v2 surfaces conform to their respective IETF specifications.

### 1.1 Authentication and authorization

| Concern | Mechanism | Reference |
|---------|-----------|-----------|
| Token format | JWT with PoP, or CWT | RFC 7519, RFC 7800; RFC 8392 |
| Authorization framework | OAuth 2.1 | RFC 9700 (BCP) |
| Channel security | TLS 1.3 | RFC 8446 |
| At-rest token integrity | COSE_Sign1 | RFC 9052 |

Tokens MUST carry an `aud` claim equal to the canonical scope URI from Phase 1 §2.1, and a `scope` string composed of one or more verbs from §4.

---

## 2. HTTP/REST Surface

### 2.1 Base URL

```
https://<host>/wia-time-011/v1
```

Servers MUST advertise version 1 of this specification through the `Server` header and the `/.well-known/wia-history` document (RFC 8615 Well-Known URIs).

### 2.2 Resource map

| Resource | Methods | Purpose |
|----------|---------|---------|
| `/scopes` | GET | List scopes visible to caller |
| `/scopes/{id}` | GET, PATCH | Scope descriptor |
| `/scopes/{id}/events` | GET, POST | Event collection |
| `/scopes/{id}/events/{eventId}` | GET | Event descriptor |
| `/scopes/{id}/ledger` | GET | Timeline ledger summary |
| `/scopes/{id}/ledger/entries` | GET | Ledger entries (range query) |
| `/scopes/{id}/timestamps` | POST | Submit a content digest, receive a TimeStampToken |
| `/scopes/{id}/timestamps/{tokenId}` | GET | Retrieve a previously issued token |
| `/scopes/{id}/evidence-records` | POST | Build an EvidenceRecord for a set of events |
| `/scopes/{id}/evidence-records/{rid}` | GET | Retrieve EvidenceRecord |
| `/scopes/{id}/transparency-logs` | GET, POST | Manage the bound transparency log(s) |
| `/scopes/{id}/verifications` | POST | Verify a token / record / inclusion proof |
| `/scopes/{id}/audit` | GET | Audit log per IEC 62443-3-3 SR 6.1 |
| `/scopes/{id}/health` | GET | Liveness / readiness |

`PATCH` requests use JSON Merge Patch (RFC 7396) by default and JSON Patch (RFC 6902) when `Content-Type: application/json-patch+json`.

### 2.3 Errors

Errors use *Problem Details for HTTP APIs* (RFC 9457). The minimum problem types:

| Code | HTTP | Meaning |
|------|------|---------|
| `history/forbidden-scope` | 403 | Caller lacks scope privilege |
| `history/unsupported-digest` | 422 | Content digest algorithm not supported by TSA |
| `history/tsa-unavailable` | 503 | Bound TSA unavailable |
| `history/inconsistent-tree` | 409 | Transparency-log consistency proof failed |
| `history/event-not-found` | 404 | Event not found in scope |
| `history/evidence-renewal-required` | 410 | Evidence record needs re-time-stamping per RFC 4998 §5.2 |

### 2.4 Pagination and conditional requests

Cursor-based pagination with `Link` headers per RFC 8288. ETag/If-Match/If-None-Match per RFC 9110 §13. Strong validators required for any state-changing request.

---

## 3. RFC 3161 Time-Stamp Surface

### 3.1 Endpoint

```
POST /wia-time-011/v1/scopes/{id}/timestamps/tsp HTTP/1.1
Content-Type: application/timestamp-query
```

Request and response bodies follow the RFC 3161 ASN.1 *TimeStampReq* / *TimeStampResp* DER encoding. The response carries a *TimeStampToken* (RFC 3161 §2.4.2) parseable by any compliant verifier.

### 3.2 Behaviour

The TSA MUST:

- Validate the digest algorithm against its supported list (declared in the discovery document).
- Bind the issued token to the policy OID declared in the scope.
- Honour the nonce mechanism per RFC 3161 §2.4.1 when one is provided.
- Append the issued token to the bound transparency log within the log's MaxMergeDelay.

### 3.3 ESSCertIDv2

Issued tokens MUST include the *ESSCertIDv2* structure (RFC 5816) so that verifiers can identify the TSA's certificate without ambiguity.

---

## 4. RFC 9162 Transparency-Log Surface

### 4.1 Endpoints

The transparency-log surface conforms to RFC 9162 §6:

- `GET /ct/v2/get-sth`
- `GET /ct/v2/get-sth-consistency?first=<old_ts>&second=<new_ts>`
- `GET /ct/v2/get-proof-by-hash?hash=<base64>&tree_size=<ts>`
- `GET /ct/v2/get-all-by-hash?hash=<base64>&tree_size=<ts>`
- `GET /ct/v2/get-entries?start=<i>&end=<j>`
- `GET /ct/v2/get-roots`

### 4.2 TransItem schema

WIA-TIME-011 v1 uses CT v2's *TransItem* schema for non-PKI domains. The `TransItem.version` MUST be 2; the `TransItem.type` MUST be a registered value or a value documented in the scope's policy.

### 4.3 Monitor protocol

The monitor protocol uses the `get-sth` and `get-sth-consistency` endpoints to verify append-only behaviour. Monitors MUST raise an alarm to the audit log when consistency verification fails.

---

## 5. Verbs and Scopes

| Verb | Capability |
|------|------------|
| `history:read` | Read events, ledger, tokens, evidence records |
| `history:submit-event` | Submit new events |
| `history:request-tsp` | Request a TimeStampToken |
| `history:build-evidence` | Build an EvidenceRecord |
| `history:verify` | Run verification queries |
| `history:configure-log` | Bind / unbind transparency logs |
| `history:author-verifier` | Add / edit Verifier records |
| `history:audit-read` | Read audit log |

Each verb MUST be enforced at every surface. Tokens missing a required verb produce HTTP 403 with the `history/forbidden-scope` problem code.

---

## 6. Sample Operations

### 6.1 Submit an event and receive a token

```
POST /wia-time-011/v1/scopes/sc-archive/events HTTP/1.1
Authorization: DPoP <token>
Content-Type: application/json

{"kind":"DOCUMENT","occurredAt":"2026-04-26T08:00:00+09:00","contentDigest":{"algorithm":"SHA-256","value":"..."},"contentType":"application/pdf"}
```

The response includes the assigned `eventId` and a fresh `TimeStampToken` for the event's content digest.

### 6.2 Build an EvidenceRecord

```
POST /wia-time-011/v1/scopes/sc-archive/evidence-records HTTP/1.1
Content-Type: application/json

{"subjectEventIds":["ev-001","ev-002","ev-003"],"digestAlgorithm":"SHA-256"}
```

The response is the canonical Phase-1 *EvidenceRecord* with embedded ERS structures.

### 6.3 Verify an inclusion proof

```
POST /wia-time-011/v1/scopes/sc-archive/verifications HTTP/1.1
Content-Type: application/json

{"kind":"INCLUSION","eventDigest":"<hex>","logId":"log-1","treeSize":91234}
```

The response is `{verified: true|false, witnesses:[...], reason:"..."}`.

---

## 7. Conformance

A WIA-TIME-011 v1 *gateway* MUST implement:

- HTTP/REST surface (§2).
- RFC 3161 TSP surface (§3) when a TSA is bound.
- RFC 9162 CT v2 surface (§4) when a transparency log is bound.
- Verbs from §5 with audit logging on every state-changing call.

A WIA-TIME-011 v1 *client* MUST:

- Honour ETag-based concurrency.
- Validate received tokens against ASN.1 DER conformance.
- Validate transparency-log proofs against CT v2 canonical bytes.

---

## 8. Operational Notes

### 8.1 Idempotency

Mutating verbs accept `Idempotency-Key`. Servers retain the response per key for 24 hours.

### 8.2 Discovery

`/scopes/{id}/.well-known/wia-history` returns supported digest algorithms, supported encodings, bound TSA URI, bound transparency-log URI, and the conformance tags from Phase 4.

### 8.3 Backpressure

Token issuance and transparency-log queries MUST be subject to a per-token rate limit. The surface MUST emit IETF `RateLimit-*` headers on rejected requests.

### 8.4 Versioning

Major version bumps require a parallel HTTP path (`/wia-time-011/v2`). Minor and patch revisions are backwards-compatible within `/v1`. Clients MUST tolerate unknown fields and unknown problem-detail extensions per RFC 9457 §4.4.

---

## 9. References

1. RFC 3161 — *Time-Stamp Protocol (TSP).*
2. RFC 3339 — *Date and Time on the Internet.*
3. RFC 4998 — *Evidence Record Syntax (ERS).*
4. RFC 5280 — *X.509 PKI Certificate and CRL Profile.*
5. RFC 5816 — *ESSCertIDv2.*
6. RFC 6838 — *Media Type Specifications.*
7. RFC 6902; RFC 7396 — *JSON Patch / Merge Patch.*
8. RFC 7519; RFC 7800; RFC 8392 — *JWT, PoP, CWT.*
9. RFC 8259; RFC 8610; RFC 8615; RFC 8949 — *JSON, CDDL, well-known URIs, CBOR.*
10. RFC 8288 — *Web Linking.*
11. RFC 8446 — *TLS 1.3.*
12. RFC 9052; RFC 9053 — *COSE.*
13. RFC 9110; RFC 9457 — *HTTP semantics, problem details.*
14. RFC 9162 — *Certificate Transparency v2.*
15. RFC 9700 — *OAuth 2.1.*
16. ISO/IEC 18014 (all parts) — *Time-stamping services.*
17. ISO/IEC 27001:2022; ISO/IEC 27037:2012; ISO/IEC 27042:2015.
18. IEC 62443-3-3:2013.

---

## 10. Discovery Document Schema

The `/.well-known/wia-history` discovery document MUST conform to the following JSON schema fragment:

```json
{
  "version": "string (semver)",
  "supportedVerbs": ["array of verb tokens from §5"],
  "supportedDigestAlgorithms": ["SHA-256", "SHA-384", "SHA-512", "SHA3-256", "SHA3-512"],
  "supportedEncodings": ["json", "cbor"],
  "tsaIdentifier": "string (URI of the bound TSA)",
  "tsaPolicyOid": "string (ASN.1 OID)",
  "transparencyLogIdentifier": "string (URI) | null",
  "transparencyLogPublicKey": "string (base64 SPKI) | null",
  "openApiUrl": "string (URI)",
  "conformanceTags": ["array of conformance tags from Phase 4 §10"]
}
```

The discovery document SHOULD be cached for 60 seconds by clients; the gateway MUST emit `Cache-Control: max-age=60`.

---

## 11. Health Endpoint Contract

The `/scopes/{id}/health` endpoint MUST return a JSON document with at least:

- `state` ∈ `{OK, DEGRADED, CRITICAL, OFFLINE}`.
- `version` — implementation semver.
- `uptimeSeconds` — non-negative integer.
- `tsa` — `{state, lastTokenAt, certExpiresAt}`.
- `transparencyLog` — `{state, treeSize, lastSthAt}`.
- `evidence` — `{totalRecords, renewalQueueDepth, oldestRecordAge}`.
- `audit` — `{lastEntryAt, queueDepth}`.

The endpoint MUST be served without authentication for the `state` and `version` fields only; full detail requires `history:read`.

---

## 12. Backpressure and Quotas

Token issuance and transparency-log queries are subject to per-scope and per-tenant quotas. Quota exhaustion returns HTTP 429 with the IETF `RateLimit-*` headers and the problem code `history/quota-exceeded` (an extension of the §2.3 list).

---

## 13. Versioning Policy

Major version bumps require a parallel HTTP path (`/wia-time-011/v2`). Minor and patch revisions are backwards-compatible within `/v1`. Clients MUST tolerate unknown fields and unknown problem-detail extensions per RFC 9457 §4.4. Breaking changes to the RFC 3161 TSP profile (§3) are forbidden within `/v1` and require a major bump even when the HTTP/REST surface is unchanged.

---

## 14. Compatibility With External Verifiers

The Phase-2 surface is designed so that any RFC 3161 TSP verifier can validate a `TimeStampToken` retrieved from the deployment without re-encoding, and any RFC 9162 CT v2 monitor can validate STH-and-consistency proofs by issuing the canonical `get-sth` and `get-sth-consistency` requests. The deployment MUST NOT introduce custom envelopes that prevent direct interoperability with these external verifiers.
