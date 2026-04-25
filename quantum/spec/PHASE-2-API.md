# WIA Quantum SDK and Client API Specification

**Phase 2: SDK / Client API**

**Version**: 1.0.0
**Status**: Draft
**Date**: 2026-04-26

---

## 1. Overview

### 1.1 Purpose

WIA Quantum Phase 2는 Phase 1 데이터 포맷을 사용하는 클라이언트 SDK와 RESTful Client API의 표준 계약을 정의합니다. 양자 회로 제출, 결과 조회, PQC 키 관리, QKD 세션 협상까지 통일된 인터페이스로 제공합니다.

This document defines the client-facing surface area: the language-agnostic REST API endpoints and the reference TypeScript SDK shape that backends MUST implement and clients MAY rely on.

### 1.2 Design Goals

1. **OpenQASM 3 Compatible** — payloads carry OpenQASM 3 source verbatim or as the equivalent JSON IR defined in PHASE 1.
2. **NIST PQC Aligned** — public-key material is encoded per FIPS 203 (ML-KEM), FIPS 204 (ML-DSA), FIPS 205 (SLH-DSA).
3. **Stable** — REST contract is versioned independently of the underlying data schema.
4. **Idiomatic SDK** — the reference TypeScript SDK presents a Promise-based, awaitable surface and MUST NOT depend on the transport layer for behavior.
5. **Backend Portable** — the same client SDK MUST be able to talk to any compliant backend without code change.

### 1.3 Scope

| In scope | Out of scope |
|----------|--------------|
| REST endpoint contract | Vendor-specific QPU drivers |
| TypeScript reference SDK shape | Compilation passes (handled by backend) |
| OpenAPI 3.0 surface | Real-time bidirectional protocol (PHASE 3) |
| Authentication and quotas | Ecosystem integrations (PHASE 4) |
| Error catalog | Hardware calibration data |

---

## 2. Transport and Encoding

### 2.1 Base URL

```
https://{backend-host}/api/v1
```

A backend MAY support additional URL prefixes for legacy clients but MUST expose `/api/v1` for WIA Quantum Phase 2 conformance.

### 2.2 Content Type

All request and response bodies use `application/json; charset=utf-8` unless explicitly noted (binary state vectors use `application/octet-stream`).

### 2.3 Versioning Header

Clients MUST send the WIA Quantum version they were built against:

```
X-WIA-Quantum-Version: 1.0.0
```

If the backend cannot honor the requested version, it MUST return HTTP 400 with `error.code = "UNSUPPORTED_VERSION"`.

### 2.4 Idempotency

Mutating endpoints (`POST /jobs`, `POST /pqc/keys`, `POST /qkd/sessions`) accept an `Idempotency-Key` header (UUIDv4). When a duplicate key is observed within 24 hours, the backend MUST return the original response without re-executing the operation.

---

## 3. Authentication

### 3.1 OAuth 2.0

Authentication follows OAuth 2.0 (IETF RFC 6749) with PKCE (IETF RFC 7636) as the default flow for interactive clients. Service accounts MAY use the client-credentials grant.

```
Authorization: Bearer <jwt>
```

JWTs follow IETF RFC 7519 with at minimum the following claims:

| Claim | Purpose |
|-------|---------|
| `sub` | Stable user or service identifier |
| `aud` | `wia-quantum` |
| `iat`, `exp` | Issued / expiry timestamps |
| `scopes` | Space-separated list (see §3.2) |

### 3.2 Scopes

| Scope | Grants |
|-------|--------|
| `circuits.read` | List and fetch circuit definitions |
| `circuits.write` | Create or update circuit definitions |
| `jobs.submit` | Submit jobs against a backend |
| `jobs.read` | Read job status and results |
| `pqc.keys.manage` | Generate and rotate PQC keypairs |
| `qkd.sessions.manage` | Negotiate and tear down QKD sessions |
| `network.read` | Read network topology |

### 3.3 PQC-Authenticated Requests (Optional)

Backends MAY accept request signatures using a registered ML-DSA (FIPS 204) public key in addition to the bearer token. The signature is carried in:

```
WIA-PQC-Signature: alg=ML-DSA-65; key-id=<id>; sig=<base64url>
```

The signed payload is the canonical JSON of the request body per IETF RFC 8785 (JSON Canonicalization Scheme).

---

## 4. Resource Endpoints

### 4.1 Circuits

| Method | Path | Description |
|--------|------|-------------|
| `GET`    | `/circuits`              | List circuits (paginated, see §6) |
| `POST`   | `/circuits`              | Create a new circuit |
| `GET`    | `/circuits/{id}`         | Fetch a circuit |
| `PUT`    | `/circuits/{id}`         | Replace a circuit |
| `DELETE` | `/circuits/{id}`         | Soft-delete a circuit |
| `GET`    | `/circuits/{id}/qasm`    | Export OpenQASM 3 source |

The `Circuit` payload is exactly the PHASE 1 §3 Circuit object.

### 4.2 Jobs

| Method | Path | Description |
|--------|------|-------------|
| `GET`    | `/jobs`              | List jobs |
| `POST`   | `/jobs`              | Submit a new job |
| `GET`    | `/jobs/{id}`         | Read job status |
| `DELETE` | `/jobs/{id}`         | Cancel a queued or running job |
| `GET`    | `/jobs/{id}/result`  | Fetch the `Result` object once `status = "completed"` |

Job states: `queued`, `validating`, `running`, `completed`, `failed`, `cancelled`.

### 4.3 Backends

| Method | Path | Description |
|--------|------|-------------|
| `GET` | `/backends`             | List available backends (QPU + simulators) |
| `GET` | `/backends/{id}`        | Backend descriptor (qubits, topology, gate set) |
| `GET` | `/backends/{id}/status` | Live status (queue depth, calibration timestamp) |

### 4.4 PQC Keys

| Method | Path | Description |
|--------|------|-------------|
| `GET`    | `/pqc/keys`        | List managed keys |
| `POST`   | `/pqc/keys`        | Generate new keypair (ML-KEM or ML-DSA) |
| `GET`    | `/pqc/keys/{id}`   | Fetch the public part |
| `DELETE` | `/pqc/keys/{id}`   | Revoke a key |

The `Crypto.PqcKey` payload is exactly the PHASE 1 §7 representation; private material never leaves the backend.

### 4.5 QKD Sessions

| Method | Path | Description |
|--------|------|-------------|
| `POST`   | `/qkd/sessions`        | Open a session against a peer node |
| `GET`    | `/qkd/sessions/{id}`   | Read session metadata (key generation rate, QBER) |
| `DELETE` | `/qkd/sessions/{id}`   | Close the session |

QKD operations follow the data shape defined in PHASE 1 §7.3 and are aligned with ETSI GS QKD 014 key delivery semantics.

---

## 5. Reference TypeScript SDK Shape

The reference SDK ships under `@wia-official/quantum`. Backends are not required to ship an SDK, but every compliant backend MUST be reachable from this SDK without modification.

```ts
import { WiaQuantumClient } from "@wia-official/quantum";

const client = new WiaQuantumClient({
  baseUrl: "https://qpu.example.com/api/v1",
  auth: { kind: "bearer", token: process.env.WIA_QUANTUM_TOKEN },
  version: "1.0.0",
});

// Submit a circuit
const circuit = await client.circuits.create({
  name: "Bell State",
  qasm: "OPENQASM 3.0;\\ninclude \\"stdgates.inc\\";\\n...",
});

const job = await client.jobs.submit({
  circuitId: circuit.id,
  backend: "wia-sim-32q",
  shots: 1024,
});

const result = await client.jobs.waitForResult(job.id, {
  pollIntervalMs: 500,
  timeoutMs: 60_000,
});
```

The SDK MUST:

1. Encode all retries with exponential backoff (initial 250 ms, factor 2, jitter ±25 %, max 6 attempts).
2. Surface every backend error as a `WiaQuantumError` with `code`, `message`, and `requestId` properties.
3. Expose a `client.raw` escape hatch for low-level fetch access without losing authentication context.
4. Honor `Retry-After` on HTTP 429 responses.

---

## 6. Pagination, Filtering, Sorting

List endpoints accept the following query parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `page_size`  | 25 | 1–200 |
| `page_token` | (none) | opaque cursor returned in the previous response |
| `filter`     | (none) | RFC 7644 SCIM-like filter expression |
| `order_by`   | `created_at desc` | sortable fields documented per resource |

Pagination responses include `next_page_token` when more results exist.

---

## 7. Error Model

All error responses follow IETF RFC 9457 (Problem Details for HTTP APIs) extended with WIA fields:

```json
{
  "type": "https://wiastandards.com/errors/quantum/circuit-invalid",
  "title": "Circuit failed validation",
  "status": 400,
  "code": "CIRCUIT_INVALID",
  "detail": "Gate `cx` references undeclared qubit q[3]",
  "instance": "/api/v1/circuits",
  "request_id": "req_<32+chars>"
}
```

### 7.1 Standard Codes

| Code | HTTP | Semantics |
|------|------|-----------|
| `INVALID_REQUEST`     | 400 | Schema validation failed |
| `UNAUTHENTICATED`     | 401 | Missing or invalid bearer token |
| `FORBIDDEN`           | 403 | Insufficient scope |
| `NOT_FOUND`           | 404 | Resource does not exist |
| `CONFLICT`            | 409 | Idempotency key reuse with mismatched body |
| `UNSUPPORTED_VERSION` | 400 | `X-WIA-Quantum-Version` not supported |
| `BACKEND_BUSY`        | 503 | Queue full or backend in calibration |
| `QUOTA_EXCEEDED`      | 429 | Per-tenant quota |
| `INTERNAL_ERROR`      | 500 | Server-side fault |

---

## 8. Quotas and Rate Limits

| Tier | Default rate limit | Notes |
|------|--------------------|-------|
| Public | 60 requests / minute | sliding window |
| Authenticated | 600 requests / minute | per token |
| Premium | negotiated | per contract |

`POST /jobs` is rate-limited independently to protect the queue. When a backend deploys quota controls it MUST emit the standard `RateLimit-Limit`, `RateLimit-Remaining`, and `RateLimit-Reset` headers per the IETF draft.

---

## 9. Observability Contract

Compliant backends MUST emit the following request log fields:

- `request_id` (echoed in the `X-Request-Id` response header)
- `tenant_id`
- `endpoint` and `http_status`
- `latency_ms`
- `error_code` when present

Sensitive fields (PQC private keys, raw OpenQASM source if classified) MUST be redacted from logs. Backends SHOULD expose a `/healthz` and `/readyz` endpoint for orchestration probes.

---

## 10. Compliance and Certification

| Conformance level | Required surface |
|-------------------|------------------|
| Tier 1 — Self-declared | All §4 endpoints + RFC 9457 errors + OpenAPI document |
| Tier 2 — Assessed | Tier 1 + signed third-party assessor report against this PHASE |
| Tier 3 — Accredited | Tier 2 + WIA accreditation + ISO/IEC 17065:2012 assessment + 7-year evidence retention |

A backend that omits §4.4 (PQC keys) or §4.5 (QKD sessions) MAY still claim Tier 1/2 conformance for the implemented subset, but MUST advertise the omission in its OpenAPI document via `info.x-wia-quantum-omits`.

---

## 11. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2026-04-26 | Initial PHASE 2 specification (this document) |

---

## 12. References

- WIA Quantum PHASE 1 Data Format Specification (this repository)
- WIA Quantum PHASE 3 Communication Protocol (this repository)
- WIA Quantum PHASE 4 Ecosystem Integration (this repository)
- OpenQASM 3.0 — open specification (IBM Research and contributors)
- NIST FIPS 203 — Module-Lattice-Based Key-Encapsulation Mechanism Standard (ML-KEM)
- NIST FIPS 204 — Module-Lattice-Based Digital Signature Standard (ML-DSA)
- NIST FIPS 205 — Stateless Hash-Based Digital Signature Standard (SLH-DSA)
- IETF RFC 6749 — The OAuth 2.0 Authorization Framework
- IETF RFC 7636 — Proof Key for Code Exchange by OAuth Public Clients
- IETF RFC 7519 — JSON Web Token (JWT)
- IETF RFC 8785 — JSON Canonicalization Scheme (JCS)
- IETF RFC 9457 — Problem Details for HTTP APIs
- ETSI GS QKD 014 — Quantum Key Distribution; Protocol and data format of REST-based key delivery API
- ITU-T Y.3800 — Overview on networks supporting quantum key distribution
- ISO/IEC 17065:2012 — Conformity assessment

---

## Annex A — Conformance Tier Matrix

WIA conformance for the WIA Quantum SDK and Client API surface is evaluated across three tiers, applied to circuit submission, job lifecycle, PQC key handling, and QKD session negotiation:

| Tier | Scope | Mandatory artifacts | Audit cadence |
|------|-------|--------------------|----------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | OpenAPI 3.0 contract, JSON Schema validation report, security threat model | Annual self-review |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Tier 1 artifacts + signed third-party assessor report against this PHASE | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Tier 2 artifacts + WIA accreditation, ISO/IEC 17065:2012 conformity assessment, evidence retention ≥ 7 years | Every 12 months |

Implementations MUST clearly disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page. Tier downgrade events MUST be reported to the WIA registry within 30 days.

---

## Annex B — Cross-Walk to International Standards

The PHASE specification reuses or normatively references the standards listed in §12. Implementers SHOULD review the listed standards alongside this PHASE document; where a conflict exists, the more specific WIA requirement governs unless explicitly superseded by a binding national regulation.

This cross-walk is informative only. WIA does not republish the referenced documents; readers MUST obtain authoritative copies from the issuing body. Cross-walk entries are reviewed at every minor version of this PHASE.

---

## Annex C — Reference Implementations and Test Vectors

### C.1 Reference Implementations

WIA does not require implementers to use a particular library, but maintains pointers to the canonical reference implementation directories under the WIA-Official GitHub organization:

- `wia-standards/standards/quantum/api/` — TypeScript SDK skeleton aligned with §5
- `wia-standards/standards/quantum/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/quantum/simulator/` — interactive browser-based simulator for the PHASE protocol

Each reference artifact ships with an MIT license and is intended as a starting point, not as a production-grade implementation.

### C.2 Test Vectors

A normative set of request/response pairs covering the schemas defined in this PHASE is published alongside this document. Implementations claiming Tier 2 or Tier 3 conformance MUST pass every published test vector in both serialization (request) and deserialization (response) directions, and MUST publish a signed report identifying which vectors were exercised.

Test vectors are versioned independently of the PHASE document; refer to the `test-vectors/` directory for the active version.

---

## Annex D — Open Questions and Future Work

This PHASE document captures the consensus position at v1.0.0. The following items are tracked for future minor or major revisions:

1. **Streaming circuits** — chunked submission of long-running circuits and partial-result delivery before job completion.
2. **Hybrid classical-quantum jobs** — first-class support for variational algorithms requiring tight client-loop iterations (currently handled in PHASE 3).
3. **Multi-region affinity** — declarative routing hints when several backends meet a job's gate-set requirements.
4. **PQC algorithm agility** — guidance for negotiating which FIPS 203/204/205 parameter sets are accepted.
5. **QKD federation** — interoperability profile for chains of ETSI GS QKD 014 compliant key suppliers.

Items in this annex are non-normative. Comments and proposals are accepted via the GitHub issues tracker on the WIA-Official organization.

---

**弘益人間 · Benefit All Humanity**

© 2026 WIA — World Certification Industry Association
Licensed under MIT License
