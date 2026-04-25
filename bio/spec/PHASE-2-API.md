# WIA Biotech SDK and Client API Specification

**Phase 2: SDK / Client API**

**Version**: 1.0.0
**Status**: Draft
**Date**: 2026-04-26

---

## 1. Overview

### 1.1 Purpose

WIA Biotech Phase 2는 Phase 1 데이터 포맷을 사용하는 클라이언트 SDK와 RESTful Client API의 표준 계약을 정의합니다. 서열 등록, 실험 메타데이터 조회, 편집 요청 제출, 구조 결과 회수까지 통일된 인터페이스로 제공합니다.

This document defines the client-facing surface area: the language-agnostic REST API endpoints and the reference TypeScript SDK shape that biotech data backends MUST implement and clients MAY rely on.

### 1.2 Design Goals

1. **Format-Faithful** — payloads carry the WIA Biotech objects defined in PHASE 1 verbatim, with no lossy normalization.
2. **Open-Spec Aligned** — the API surface respects the boundaries of FASTA, FASTQ, SBOL 3.0, GFF3, and PDB without re-defining their semantics.
3. **Provenance First** — every mutating call records W3C PROV-DM compatible provenance metadata.
4. **Stable** — the REST contract is versioned independently of the underlying schema.
5. **Backend Portable** — the same SDK MUST be able to talk to any compliant backend without code change.

### 1.3 Scope

| In scope | Out of scope |
|----------|--------------|
| REST endpoint contract | Wet-lab automation drivers |
| TypeScript reference SDK shape | Statistical analysis pipelines |
| OpenAPI 3.0 surface | Real-time biosensor streaming (PHASE 3) |
| Authentication and quotas | Foundry orchestration (PHASE 4) |
| Error catalog | Clinical or diagnostic claims |

This specification does not claim, prescribe, or evaluate medical or clinical efficacy. Operators in regulated jurisdictions remain individually responsible for sectoral compliance (e.g., the Korean 의료기기법, EU IVDR/MDR equivalents, US FDA pathways) when applicable.

---

## 2. Transport and Encoding

### 2.1 Base URL

```
https://{backend-host}/api/v1
```

A backend MAY support additional URL prefixes for legacy clients but MUST expose `/api/v1` for WIA Biotech Phase 2 conformance.

### 2.2 Content Type

All request and response bodies use `application/json; charset=utf-8`. Bulk sequence uploads MAY use `application/x-fasta`, `application/x-fastq`, `chemical/x-pdb`, or `application/sbol+xml` for content types defined by their respective community specifications.

### 2.3 Versioning Header

Clients MUST send the WIA Biotech version they were built against:

```
X-WIA-Biotech-Version: 1.0.0
```

If the backend cannot honor the requested version, it MUST return HTTP 400 with `error.code = "UNSUPPORTED_VERSION"`.

### 2.4 Idempotency

Mutating endpoints accept an `Idempotency-Key` header (UUIDv4). When a duplicate key is observed within 24 hours, the backend MUST return the original response without re-executing the operation.

---

## 3. Authentication

### 3.1 OAuth 2.0

Authentication follows OAuth 2.0 (IETF RFC 6749) with PKCE (IETF RFC 7636) as the default flow for interactive clients. Service accounts MAY use the client-credentials grant.

```
Authorization: Bearer <jwt>
```

JWTs follow IETF RFC 7519 with at minimum the following claims: `sub`, `aud` = `wia-biotech`, `iat`, `exp`, `scopes`.

### 3.2 Scopes

| Scope | Grants |
|-------|--------|
| `sequences.read` / `.write` | Read or register sequence records |
| `experiments.read` / `.write` | Read or register CRISPR / editing experiments |
| `structures.read` / `.write` | Read or register structure predictions |
| `parts.read` / `.write` | Read or register SBOL parts |
| `provenance.read` | Read provenance graphs |

### 3.3 Sensitive Data Boundaries

Endpoints MUST NOT accept human germline edit instructions or fields that imply prohibited modifications. Backends MAY enforce policy by rejecting payloads that match a configured deny-list with HTTP 451 (`POLICY_DENIED`).

---

## 4. Resource Endpoints

### 4.1 Sequences

| Method | Path | Description |
|--------|------|-------------|
| `GET`    | `/sequences`              | List sequences (paginated) |
| `POST`   | `/sequences`              | Register a new sequence record |
| `GET`    | `/sequences/{id}`         | Fetch a sequence record |
| `PUT`    | `/sequences/{id}`         | Replace a sequence record |
| `DELETE` | `/sequences/{id}`         | Soft-delete a sequence |
| `GET`    | `/sequences/{id}/fasta`   | Export FASTA payload |
| `GET`    | `/sequences/{id}/fastq`   | Export FASTQ payload (if quality scores recorded) |

The `Sequence` payload is exactly the PHASE 1 §3 sequence object.

### 4.2 Experiments

| Method | Path | Description |
|--------|------|-------------|
| `GET`    | `/experiments`              | List experiments |
| `POST`   | `/experiments`              | Register a new experiment record |
| `GET`    | `/experiments/{id}`         | Read an experiment record |
| `PUT`    | `/experiments/{id}`         | Replace an experiment record |
| `GET`    | `/experiments/{id}/results` | Fetch the experiment's result object |

### 4.3 Structures

| Method | Path | Description |
|--------|------|-------------|
| `GET`    | `/structures`              | List structures |
| `POST`   | `/structures`              | Register a structure prediction |
| `GET`    | `/structures/{id}`         | Read a structure record |
| `GET`    | `/structures/{id}/pdb`     | Export the PDB-format payload |
| `GET`    | `/structures/{id}/confidence` | Read pLDDT / PAE confidence metrics |

### 4.4 Parts

| Method | Path | Description |
|--------|------|-------------|
| `GET`    | `/parts`        | List SBOL parts |
| `POST`   | `/parts`        | Register a new SBOL 3.0 part |
| `GET`    | `/parts/{id}`   | Read part metadata |
| `GET`    | `/parts/{id}/sbol` | Export `application/sbol+xml` |

### 4.5 Provenance

| Method | Path | Description |
|--------|------|-------------|
| `GET`  | `/provenance/{resource_id}` | Return the W3C PROV-DM graph for a resource |

---

## 5. Reference TypeScript SDK Shape

The reference SDK ships under `@wia-official/biotech`. Backends are not required to ship an SDK, but every compliant backend MUST be reachable from this SDK without modification.

```ts
import { WiaBiotechClient } from "@wia-official/biotech";

const client = new WiaBiotechClient({
  baseUrl: "https://lab.example.com/api/v1",
  auth: { kind: "bearer", token: process.env.WIA_BIOTECH_TOKEN },
  version: "1.0.0",
});

// Register a sequence
const seq = await client.sequences.create({
  name: "EGFP",
  organism: "Aequorea victoria (origin)",
  sequence: "ATGGTGAGCAAGGGCGAGGAGCTGTTC...",
  modality: "dna",
});

// Read provenance
const prov = await client.provenance.fetch(seq.id);
```

The SDK MUST:

1. Encode all retries with exponential backoff (initial 250 ms, factor 2, jitter ±25 %, max 6 attempts).
2. Surface every backend error as a `WiaBiotechError` with `code`, `message`, and `requestId` properties.
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
  "type": "https://wiastandards.com/errors/biotech/sequence-invalid",
  "title": "Sequence failed validation",
  "status": 400,
  "code": "SEQUENCE_INVALID",
  "detail": "Character 'Z' is not in the IUPAC nucleotide alphabet",
  "instance": "/api/v1/sequences",
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
| `CONFLICT`            | 409 | Idempotency-key reuse with mismatched body |
| `UNSUPPORTED_VERSION` | 400 | `X-WIA-Biotech-Version` not supported |
| `POLICY_DENIED`       | 451 | Payload violates a configured policy boundary |
| `QUOTA_EXCEEDED`      | 429 | Per-tenant quota |
| `INTERNAL_ERROR`      | 500 | Server-side fault |

---

## 8. Quotas and Rate Limits

| Tier | Default rate limit | Notes |
|------|--------------------|-------|
| Public | 60 requests / minute | sliding window |
| Authenticated | 600 requests / minute | per token |
| Premium | negotiated | per contract |

Bulk sequence uploads are rate-limited independently to protect storage. Backends emit the standard `RateLimit-Limit`, `RateLimit-Remaining`, and `RateLimit-Reset` headers.

---

## 9. Observability and Provenance Contract

Compliant backends MUST emit the following request log fields:

- `request_id` (echoed in the `X-Request-Id` response header)
- `tenant_id`
- `endpoint` and `http_status`
- `latency_ms`
- `error_code` when present

Provenance entries written for every mutating call MUST follow W3C PROV-DM and at minimum reference the `prov:wasAttributedTo` agent (the authenticated principal) and the `prov:generatedAtTime` timestamp.

Backends SHOULD expose `/healthz` and `/readyz` endpoints for orchestration probes. Sensitive fields (full personal genomic identifiers) MUST be redacted from logs.

---

## 10. Compliance and Certification

| Conformance level | Required surface |
|-------------------|------------------|
| Tier 1 — Self-declared | All §4 endpoints + RFC 9457 errors + OpenAPI document + provenance writes per §9 |
| Tier 2 — Assessed | Tier 1 + signed third-party assessor report against this PHASE |
| Tier 3 — Accredited | Tier 2 + WIA accreditation + ISO/IEC 17065:2012 assessment + 7-year evidence retention + ISO 20387:2018 alignment for biobanking deployments |

A backend that omits §4.4 (parts) or §4.5 (provenance) MAY still claim Tier 1 conformance for the implemented subset, but MUST advertise the omission in its OpenAPI document via `info.x-wia-biotech-omits`.

---

## 11. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2026-04-26 | Initial PHASE 2 specification (this document) |

---

## 12. References

- WIA Biotech PHASE 1 Data Format Specification (this repository)
- WIA Biotech PHASE 3 Communication Protocol (this repository)
- WIA Biotech PHASE 4 Ecosystem Integration (this repository)
- FASTA — community sequence format (open spec)
- FASTQ — community sequence-with-quality format (open spec)
- GFF3 — Generic Feature Format Version 3 (Sequence Ontology Project, open spec)
- SBOL 3.0 — Synthetic Biology Open Language (open spec)
- PDB Format — Protein Data Bank exchange format (worldwide PDB consortium)
- W3C PROV-DM — provenance data model
- IETF RFC 6749 — The OAuth 2.0 Authorization Framework
- IETF RFC 7636 — Proof Key for Code Exchange by OAuth Public Clients
- IETF RFC 7519 — JSON Web Token (JWT)
- IETF RFC 7644 — System for Cross-domain Identity Management: Protocol
- IETF RFC 9457 — Problem Details for HTTP APIs
- ISO 20387:2018 — Biotechnology — Biobanking — General requirements for biobanking
- ISO 35001:2019 — Biorisk management for laboratories and other related organisations
- ISO/IEC 17065:2012 — Conformity assessment

---

## Annex A — Conformance Tier Matrix

WIA conformance for the WIA Biotech SDK and Client API surface is evaluated across three tiers, applied to sequence registration, experiment lifecycle, structure prediction storage, and SBOL part exchange:

| Tier | Scope | Mandatory artifacts | Audit cadence |
|------|-------|--------------------|----------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | OpenAPI 3.0 contract, JSON Schema validation report, security threat model, provenance record samples | Annual self-review |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Tier 1 artifacts + signed third-party assessor report against this PHASE | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Tier 2 artifacts + WIA accreditation, ISO/IEC 17065:2012 conformity assessment, ISO 20387:2018 biobanking alignment where applicable, evidence retention ≥ 7 years | Every 12 months |

Implementations MUST disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page. Tier downgrade events MUST be reported to the WIA registry within 30 days.

---

## Annex B — Cross-Walk to International Standards

The PHASE specification reuses or normatively references the standards listed in §12. Implementers SHOULD review the listed standards alongside this PHASE document; where a conflict exists, the more specific WIA requirement governs unless explicitly superseded by a binding national regulation.

This cross-walk is informative only. WIA does not republish the referenced documents; readers MUST obtain authoritative copies from the issuing body. Cross-walk entries are reviewed at every minor version of this PHASE.

---

## Annex C — Reference Implementations and Test Vectors

### C.1 Reference Implementations

WIA does not require implementers to use a particular library, but maintains pointers to the canonical reference implementation directories under the WIA-Official GitHub organization:

- `wia-standards/standards/bio/api/` — TypeScript SDK skeleton aligned with §5
- `wia-standards/standards/bio/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/bio/simulator/` — interactive browser-based simulator for the PHASE protocol

Each reference artifact ships with an MIT license and is intended as a starting point, not as a production-grade implementation.

### C.2 Test Vectors

A normative set of request/response pairs covering the schemas defined in this PHASE is published alongside this document. Implementations claiming Tier 2 or Tier 3 conformance MUST pass every published test vector in both serialization and deserialization directions, and MUST publish a signed report identifying which vectors were exercised.

Test vectors are versioned independently of the PHASE document; refer to the `test-vectors/` directory for the active version.

---

## Annex D — Open Questions and Future Work

This PHASE document captures the consensus position at v1.0.0. The following items are tracked for future minor or major revisions:

1. **Streaming uploads** — chunked submission of long FASTQ files with on-the-fly checksum verification.
2. **Federated registries** — interoperability profile for cross-organization sequence and parts catalogs without re-uploading material.
3. **Privacy of personal genomic identifiers** — guidance on de-identification before transit.
4. **Sustainability disclosure** — optional fields for energy reporting tied to wet-lab operations covered by this PHASE in PHASE 4.
5. **Differentiated retention policies** — per-resource-class retention controls that respect sectoral regulations across jurisdictions.

Items in this annex are non-normative. Comments and proposals are accepted via the GitHub issues tracker on the WIA-Official organization.

---

**弘益人間 · Benefit All Humanity**

© 2026 WIA — World Certification Industry Association
Licensed under MIT License
