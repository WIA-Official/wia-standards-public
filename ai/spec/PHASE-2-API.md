# WIA AI SDK and Client API Specification

**Phase 2: SDK / Client API**

**Version**: 1.0.0
**Status**: Draft
**Date**: 2026-04-26

---

## 1. Overview

### 1.1 Purpose

WIA AI Phase 2는 Phase 1 데이터 포맷을 사용하는 클라이언트 SDK와 RESTful Client API의 표준 계약을 정의합니다. 모델 등록, 데이터셋 등록, 추론 작업 제출, 평가 결과 회수까지 통일된 인터페이스로 제공합니다.

This document defines the client-facing surface area: the language-agnostic REST API endpoints and the reference TypeScript SDK shape that AI platform backends MUST implement and clients MAY rely on.

### 1.2 Design Goals

1. **Format-Faithful** — payloads carry the WIA AI objects defined in PHASE 1 verbatim, with no lossy normalization.
2. **Risk-Aware** — every model registration MUST declare a risk classification aligned with NIST AI RMF 1.0 categories.
3. **Provenance First** — datasets, model versions, and evaluation runs are linked through W3C PROV-DM compatible records.
4. **Stable** — the REST contract is versioned independently of the underlying schema.
5. **Backend Portable** — the same SDK MUST be able to talk to any compliant backend without code change.

### 1.3 Scope

| In scope | Out of scope |
|----------|--------------|
| REST endpoint contract | Specific model architectures |
| TypeScript reference SDK shape | Training-time hyperparameter search |
| OpenAPI 3.0 surface | Model artifact storage formats (ONNX, GGUF, etc. — referenced, not redefined) |
| Authentication and quotas | Real-time inference streaming (PHASE 3) |
| Error catalog | Ecosystem integrations (PHASE 4) |

---

## 2. Transport and Encoding

### 2.1 Base URL

```
https://{backend-host}/api/v1
```

### 2.2 Content Type

All request and response bodies use `application/json; charset=utf-8`. Model artifact uploads MAY use `application/octet-stream` with the artifact format declared in metadata (`onnx`, `pt`, `safetensors`, `gguf`, ...).

### 2.3 Versioning Header

Clients MUST send the WIA AI version they were built against:

```
X-WIA-AI-Version: 1.0.0
```

If the backend cannot honor the requested version, it MUST return HTTP 400 with `error.code = "UNSUPPORTED_VERSION"`.

### 2.4 Idempotency

Mutating endpoints accept an `Idempotency-Key` header (UUIDv4). When a duplicate key is observed within 24 hours, the backend MUST return the original response without re-executing the operation.

---

## 3. Authentication

OAuth 2.0 (IETF RFC 6749) with PKCE (IETF RFC 7636) is the default for interactive clients; service accounts MAY use the client-credentials grant. JWTs follow IETF RFC 7519.

### 3.1 Scopes

| Scope | Grants |
|-------|--------|
| `models.read` / `.write` | Read or register model versions |
| `datasets.read` / `.write` | Read or register dataset records |
| `inferences.submit`     | Submit inference jobs |
| `inferences.read`       | Read inference results |
| `evaluations.manage`    | Submit and read evaluation runs |
| `provenance.read`       | Read provenance graphs |

### 3.2 Risk-Aware Operations

Endpoints that register a high-risk model (per NIST AI RMF 1.0 categorization) MUST require an additional `models.high-risk` scope and MUST persist the operator identity in the provenance record.

---

## 4. Resource Endpoints

### 4.1 Models

| Method | Path | Description |
|--------|------|-------------|
| `GET`    | `/models`                 | List model versions |
| `POST`   | `/models`                 | Register a new model version |
| `GET`    | `/models/{id}`            | Read a model record |
| `PUT`    | `/models/{id}`            | Replace a model record |
| `DELETE` | `/models/{id}`            | Decommission a model |
| `GET`    | `/models/{id}/artifact`   | Download the artifact (if hosted) |

The `Model` payload is exactly the PHASE 1 §3 model object.

### 4.2 Datasets

| Method | Path | Description |
|--------|------|-------------|
| `GET`    | `/datasets`         | List datasets |
| `POST`   | `/datasets`         | Register a dataset record |
| `GET`    | `/datasets/{id}`    | Read a dataset record |

### 4.3 Inferences

| Method | Path | Description |
|--------|------|-------------|
| `POST`   | `/inferences`           | Submit an inference job |
| `GET`    | `/inferences/{id}`      | Read job status |
| `GET`    | `/inferences/{id}/result` | Fetch result once `status = "completed"` |
| `DELETE` | `/inferences/{id}`      | Cancel a queued or running job |

### 4.4 Evaluations

| Method | Path | Description |
|--------|------|-------------|
| `GET`    | `/evaluations`            | List evaluation runs |
| `POST`   | `/evaluations`            | Submit a new evaluation run |
| `GET`    | `/evaluations/{id}`       | Read evaluation results |

### 4.5 Provenance

| Method | Path | Description |
|--------|------|-------------|
| `GET` | `/provenance/{resource_id}` | Return the W3C PROV-DM graph |

---

## 5. Reference TypeScript SDK Shape

```ts
import { WiaAIClient } from "@wia-official/ai";

const client = new WiaAIClient({
  baseUrl: "https://ai.example.com/api/v1",
  auth: { kind: "bearer", token: process.env.WIA_AI_TOKEN },
  version: "1.0.0",
});

// Submit an inference job
const job = await client.inferences.submit({
  modelId: "model-classifier-v3",
  input: { text: "..." },
});

const result = await client.inferences.waitForResult(job.id, {
  pollIntervalMs: 500,
  timeoutMs: 60_000,
});
```

The SDK MUST: encode retries with exponential backoff (250 ms initial, factor 2, jitter ±25 %, max 6); surface every backend error as a `WiaAIError` with `code`, `message`, `requestId`; honor `Retry-After` on HTTP 429.

---

## 6. Pagination, Filtering, Sorting

| Parameter | Default | Notes |
|-----------|---------|-------|
| `page_size`  | 25 | 1–200 |
| `page_token` | (none) | opaque cursor |
| `filter`     | (none) | RFC 7644 SCIM-like filter |
| `order_by`   | `created_at desc` | sortable per resource |

---

## 7. Error Model

All error responses follow IETF RFC 9457 (Problem Details for HTTP APIs):

```json
{
  "type": "https://wiastandards.com/errors/ai/model-invalid",
  "title": "Model failed validation",
  "status": 400,
  "code": "MODEL_INVALID",
  "detail": "artifact format `pt` is incompatible with declared runtime",
  "instance": "/api/v1/models",
  "request_id": "req_<32+chars>"
}
```

Standard codes: `INVALID_REQUEST` (400), `UNAUTHENTICATED` (401), `FORBIDDEN` (403), `NOT_FOUND` (404), `CONFLICT` (409), `UNSUPPORTED_VERSION` (400), `RISK_REVIEW_REQUIRED` (403), `QUOTA_EXCEEDED` (429), `INTERNAL_ERROR` (500).

---

## 8. Quotas and Rate Limits

| Tier | Default rate limit |
|------|--------------------|
| Public | 60 requests / minute (sliding window) |
| Authenticated | 600 requests / minute per token |
| Premium | negotiated |

`POST /inferences` is rate-limited independently to protect compute. Backends emit the standard `RateLimit-Limit`, `RateLimit-Remaining`, `RateLimit-Reset` headers.

---

## 9. Observability and Provenance Contract

Compliant backends MUST emit `request_id` (echoed in `X-Request-Id`), `tenant_id`, `endpoint`, `http_status`, `latency_ms`, `error_code`. Provenance entries written for every mutating call MUST follow W3C PROV-DM and reference `prov:wasAttributedTo` (the authenticated principal) and `prov:generatedAtTime`.

`/healthz` and `/readyz` SHOULD be exposed for orchestration probes. PII in logs MUST be redacted.

---

## 10. Compliance and Certification

| Tier | Required surface |
|------|------------------|
| Tier 1 — Self-declared | All §4 endpoints + RFC 9457 errors + OpenAPI document + risk classification per NIST AI RMF 1.0 |
| Tier 2 — Assessed | Tier 1 + signed third-party assessor report against this PHASE |
| Tier 3 — Accredited | Tier 2 + WIA accreditation + ISO/IEC 17065:2012 + alignment with ISO/IEC 23053:2022 governance practices for high-risk deployments |

---

## 11. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2026-04-26 | Initial PHASE 2 specification (this document) |

---

## 12. References

- WIA AI PHASE 1 Data Format Specification (this repository)
- WIA AI PHASE 3 Communication Protocol (this repository)
- WIA AI PHASE 4 Ecosystem Integration (this repository)
- ISO/IEC 22989:2022 — Information technology — Artificial intelligence — Concepts and terminology
- ISO/IEC 23053:2022 — Framework for Artificial Intelligence systems using Machine Learning
- ISO/IEC 25059:2023 — Quality model for AI systems
- ISO/IEC 17065:2012 — Conformity assessment
- NIST AI Risk Management Framework 1.0 (NIST AI 100-1)
- W3C PROV-DM — provenance data model
- IETF RFC 6749 — The OAuth 2.0 Authorization Framework
- IETF RFC 7519 — JSON Web Token (JWT)
- IETF RFC 7636 — Proof Key for Code Exchange by OAuth Public Clients
- IETF RFC 7644 — System for Cross-domain Identity Management: Protocol
- IETF RFC 9457 — Problem Details for HTTP APIs

---

## Annex A — Conformance Tier Matrix

WIA conformance for the WIA AI SDK and Client API surface is evaluated across three tiers, applied to model registration, dataset registration, inference lifecycle, and evaluation runs:

| Tier | Scope | Mandatory artifacts | Audit cadence |
|------|-------|--------------------|----------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | OpenAPI 3.0 contract, JSON Schema validation report, security threat model, NIST AI RMF 1.0 classification per registered model | Annual self-review |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Tier 1 artifacts + signed third-party assessor report against this PHASE | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Tier 2 artifacts + WIA accreditation, ISO/IEC 17065:2012 conformity assessment, evidence retention ≥ 7 years | Every 12 months |

Implementations MUST disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension.

---

## Annex B — Cross-Walk to International Standards

The PHASE specification reuses or normatively references the standards listed in §12. Implementers SHOULD review the listed standards alongside this PHASE document; where a conflict exists, the more specific WIA requirement governs unless explicitly superseded by a binding national regulation. This cross-walk is informative only.

---

## Annex C — Reference Implementations and Test Vectors

The WIA-Official GitHub organization publishes:

- `wia-standards/standards/ai/api/` — TypeScript SDK skeleton
- `wia-standards/standards/ai/cli/` — POSIX shell client
- `wia-standards/standards/ai/simulator/` — interactive browser-based simulator

Test vectors covering the PHASE 1 schemas are published alongside this document. Tier 2/3 implementations MUST pass every published test vector.

---

## Annex D — Open Questions and Future Work

1. **Streaming inference** — chunked submission and partial-result delivery before job completion (currently in PHASE 3).
2. **Federated training contract** — interoperability profile for cross-organization training without raw-data sharing.
3. **Differential privacy negotiation** — explicit fields for declaring privacy budgets per inference.
4. **Carbon disclosure** — energy-per-inference reporting fields.
5. **Multilingual error catalogs** — localization profile for the §7 error codes.

Items in this annex are non-normative.

---

**弘益人間 · Benefit All Humanity** — © 2026 WIA. Licensed under MIT.
