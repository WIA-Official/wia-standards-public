# WIA-AI-027 — AI Privacy Preservation

> AI 프라이버시 표준 — 개인정보를 쓰는 AI 시스템의 처리 통제·공개 경계·차등 프라이버시 회계·정보주체 요청·적합성 평가

## Scope

WIA-AI-027 v1.0 defines technical and operational requirements for preserving
privacy throughout AI data collection, preparation, training, evaluation,
deployment, inference, and retirement. It covers:

- A common resource model for privacy domains, datasets, processing jobs,
  artifacts, and subject requests.
- Enforceable boundaries between private computation and authorized disclosure
  (release modes `CONTROLLED`, `DP_AGGREGATE`, `DP_POSTPROCESS`).
- A person-level differential privacy mechanism for bounded integer aggregates
  (`DISCRETE_LAPLACE_VECTOR`, add/remove-person adjacency) with permanent,
  cumulative privacy accounting.
- Lineage, retention, source-data erasure, and model remediation, including
  restriction and erasure requests and their effect on dependent models.
- Management interfaces (`/wia/ai-privacy/v1`), idempotency and concurrency
  rules, error and audit-event contracts.
- Test cases, conformance reporting, certification, and versioning.

Out of scope for v1.0: legal validity of a processing authority, a universal
acceptable memorization risk, a general DP accountant for arbitrary training
(including DP-SGD), cryptographic protocols for federated learning or MPC,
universal machine unlearning, and recovery of copies already held by
independent recipients.

## Conformance levels

| Level | Name | Adds |
|---|---|---|
| Level 1 | Controlled Processing | Enforceable handling and release controls |
| Level 2 | Managed AI Privacy | Automated subject-request workflows, model-impact handling, documented privacy evaluation |
| Level 3 | Accounted Private Release | Core DP mechanism, permanent accounting, controlled postprocessing |

Conformance applies to a named implementation, configuration, tenant scope,
and declared boundary. It does not establish legal compliance, prove that data
are anonymous, or demonstrate that a model cannot disclose personal information.

## Normative references

- RFC 2119, RFC 8174 (requirement keywords)
- RFC 8259 (JSON), RFC 3339 (timestamps), RFC 8785 (JSON Canonicalization Scheme)
- RFC 8446 (TLS 1.3), RFC 9110 (HTTP Semantics)
- JSON Schema Draft 2020-12

## Files

- `spec/WIA-AI-027-v1.0.md` — the full specification (v1.0)
- Companion guide (8 chapters, Korean and English): https://wiastandards.com/ai-privacy/ebook/en/ · https://wiastandards.com/ai-privacy/ebook/ko/

## License and publisher

© SmileStory Inc. · WIA Standards
