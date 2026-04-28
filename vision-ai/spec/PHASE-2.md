# WIA-AI-021 Vision AI Standard — PHASE 2 Specification

**Version:** 1.0
**Status:** Draft
**Date:** 2026-04-28
**Organization:** World Certification Industry Association (WIA)

---

## 弘益人間 (Benefit All Humanity)

---

## 1. Overview

Phase 2 specifies the API surface a WIA-AI-021 vision-AI deployment exposes to model operators, MLOps platforms, downstream applications, and downstream consumers (auditors, regulators, dataset providers). The surface is rooted in the canonical conventions of the modern computer-vision and AI-deployment domain: ONNX as the cross-framework model interchange format, OpenVINO and NVIDIA Triton as the canonical inference engines for production deployment, MLflow as the canonical experiment-tracking and model-registry tool, MLPerf Inference as the benchmark suite, and ISO/IEC 22989 / 23053 / 24029 as the AI-management normative anchors.

### 1.1 Authorization model

Authorization on a vision-AI deployment composes:

| Concern | Mechanism | Reference |
|---------|-----------|-----------|
| Operator console identity | X.509 client certificate per ISO/IEC 27001 §A.5.16 | ISO/IEC 27001:2022 |
| Per-model entitlement | Documented model-publish governance with role-based access | ISO/IEC 23053:2022 |
| Per-inference rate limit | Operator's published quota per consumer identity | NIST SP 800-53 Rev 5 SC controls |
| Records retention | NIST SP 800-53 Rev 5 AU controls + AI-impact assessment retention | NIST SP 800-53 Rev 5 |
| Privacy floor (face / biometric data) | DPIA per ISO/IEC 29134:2023 + jurisdiction biometric law | ISO/IEC 29134:2023 |
| Robustness assurance | ISO/IEC 24029-2:2023 robustness assessment | ISO/IEC 24029-2:2023 |

API requests originate from authenticated operator consoles or downstream-application service accounts. The console signs every API request with an X.509 certificate; the certificate chain anchors at the operator's PKI per their ISO/IEC 27001 access-control register.

### 1.2 Privacy floor for face and biometric data

Face-recognition, gait-recognition, and other biometric inferences are the highest-privacy-impact data class in the standard. Per-individual biometric inferences carry irreversible re-identification risk and (in many jurisdictions) require explicit informed consent before processing. Endpoints that perform biometric inference MUST honour the deployment's published Data Protection Impact Assessment (DPIA, ISO/IEC 29134:2023) and the applicable jurisdiction biometric-data law (GDPR Article 9, KR PIPA Article 23 민감정보 처리, EU AI Act Article 5 prohibited practices, US BIPA in Illinois, US FRA in California). The discovery document declares which biometric-class inferences the deployment offers and under which legal basis.

---

## 2. HTTP/REST Surface

### 2.1 Base URL and discovery

```
https://<host>/wia-ai-021/v1
```

Discovery document at `/.well-known/wia-ai-021`:

```json
{
  "wia_ai_021_version": "1.0.0",
  "deployment_id": "<UUID v4 per RFC 9562>",
  "operator": "<operator-name>",
  "supported_runtimes": ["ONNX Runtime 1.x", "OpenVINO 2024", "NVIDIA Triton 2.x", "TensorFlow Serving 2.x"],
  "model_registry_uri": "<URI of MLflow or operator-managed registry>",
  "supported_modalities": ["classification", "object_detection", "segmentation", "ocr", "vlm"],
  "biometric_classes_offered": ["face_verification"],
  "ai_act_risk_tier": "high_risk",
  "endpoints": { "model": "...", "inference": "...", "dataset": "...", "evaluation": "..." }
}
```

### 2.2 Model endpoint

```
GET    /model                        → list models
GET    /model/{id}                   → model descriptor (Phase 1 §2.2)
POST   /model                         → publish a model (operator only)
PUT    /model/{id}/promote            → promote a model from staging to production
GET    /model/{id}/card               → model card (ISO/IEC 23053 conformant)
DELETE /model/{id}                   → archive (retain history)
```

Models are published with their ONNX or OpenVINO IR artefact, the model card (ISO/IEC 23053 conformant), the training-dataset reference (Phase 3 §3 evidence chain), and the evaluation results (Phase 1 §2.4 evaluation envelope). Promotion from staging to production is gated on the operator's published evaluation thresholds.

### 2.3 Inference endpoint

```
POST   /inference                    → submit an inference request
POST   /inference/batch              → batch inference (synchronous up to N items)
GET    /inference/{id}               → result of an asynchronous inference
GET    /inference/stream             → SSE stream of low-latency inferences
```

The inference endpoint accepts an image (base64 or URI reference), the target model identifier, and operator-specific parameters (confidence threshold, top-K, NMS configuration for object detection). Results return the inference output plus a confidence-calibration evidence reference and the model-card reference so consumers can verify the inference provenance.

### 2.4 Dataset endpoint

```
GET    /dataset/{id}                 → dataset descriptor
POST   /dataset                       → register a dataset (with provenance and licensing)
GET    /dataset/{id}/manifest        → dataset manifest with hash commitments
PUT    /dataset/{id}/datasheet       → upload a Datasheet for Datasets (Gebru-style documentation)
```

Datasets are registered with their full provenance chain (source attribution, collection method, licensing, demographic distribution), a hash-committed manifest of every included sample, and a datasheet documenting limitations and intended uses. Dataset privacy is governed by the same DPIA discipline as model inference.

### 2.5 Evaluation endpoint

```
GET    /evaluation/{model_id}        → evaluation history for a model
POST   /evaluation                    → submit a new evaluation run
GET    /evaluation/{id}/per-cohort   → per-cohort fairness metrics
```

Every model carries a documented evaluation history. The per-cohort endpoint returns fairness metrics across demographic cohorts where the deployment supports cohort-aware evaluation (typically aligned to ISO/IEC TR 24027:2021 bias-in-AI guidance).

### 2.6 NIST FRVT integration endpoint (face-recognition only)

```
GET    /frvt/score                   → published NIST FRVT score for face-recognition models
GET    /frvt/test-vectors            → NIST FRVT-compatible test vector set
```

Face-recognition models offered through the deployment SHOULD publish their NIST FRVT (Face Recognition Vendor Test) score as an objective accuracy reference. The test-vector endpoint exposes the FRVT-compatible vectors so independent evaluators can replicate the score.

---

## 3. Idempotency and retry semantics

Every write endpoint accepts the `Idempotency-Key` header per IETF draft `draft-ietf-httpapi-idempotency-key-header`. Hosts retain a 24-hour replay cache per console identity. Inference retries on flaky downstream-application networks rely on the discipline.

---

## 4. Pagination, filtering, and bulk export

Collection endpoints support cursor pagination per IETF `draft-ietf-httpapi-link-relations`. Bulk export at `POST /exports` accepts a time window plus entity filter and returns a signed manifest with a Merkle root over the included envelopes. AI-Act-regulated deployments use the bulk export as the audit-trail evidence for high-risk-system inspections.

---

## 5. Health and observability

```
GET /health   → liveness
GET /ready    → readiness (includes runtime-pool connectivity check)
GET /metrics  → Prometheus exposition
```

The `/metrics` endpoint exposes: inferences-served per minute per model, p50/p95/p99 inference latency, error rates, GPU/CPU utilisation per worker, and per-cohort-fairness drift indicators. Telemetry MUST NOT include high-cardinality labels (per-image identifiers, per-individual biometric identifiers).

---

## 6. Error model

Errors return RFC 9457 problem documents. Reserved problem types relevant to Phase 2:

| Type | Status | Meaning |
|------|--------|---------|
| `…/dpia-violation` | 403 | The requested operation falls outside the deployment's published DPIA purpose limitations. |
| `…/biometric-consent-required` | 403 | Biometric inference requires explicit consent per the applicable jurisdiction biometric law. |
| `…/model-not-promoted` | 412 | The requested model is in staging; production promotion required. |
| `…/dataset-license-violation` | 403 | The requested operation would violate the dataset's documented licensing terms. |
| `…/runtime-overloaded` | 503 | The inference runtime pool is at capacity; retry with `Retry-After`. |
| `…/ai-act-prohibited-practice` | 403 | The requested inference falls under EU AI Act Article 5 prohibited practices. |
| `…/calibration-drift-detected` | 422 | The model's calibration has drifted beyond the operator's threshold; re-evaluation required before further inference. |

---

## 7. Conformance test suite

A black-box conformance test suite is published at `https://github.com/WIA-Official/wia-vision-ai-conformance` and walks through every Phase 2 endpoint, the ONNX model round-trip, the OpenVINO IR round-trip, the MLPerf Inference benchmark replication, the per-cohort fairness evaluation, the NIST FRVT integration (when face recognition is offered), and the bulk-export Merkle root check.

---

## 8. References

- ONNX Specification (Open Neural Network Exchange) — Linux Foundation
- OpenVINO 2024 — Intel inference engine documentation
- NVIDIA Triton Inference Server 2.x documentation
- TensorFlow Serving 2.x documentation
- MLflow — Open-source ML lifecycle platform (Linux Foundation)
- MLPerf Inference v4.x — MLCommons benchmark suite
- NIST FRVT (Face Recognition Vendor Test) — NIST Information Access Division
- ISO/IEC 22989:2022 — Information technology — Artificial intelligence — Concepts and terminology
- ISO/IEC 23053:2022 — Framework for AI systems using machine learning
- ISO/IEC 24029-1:2021 / 24029-2:2023 — Assessment of robustness of neural networks
- ISO/IEC TR 24027:2021 — Bias in AI systems and AI-aided decision making
- ISO/IEC TR 24368:2022 — Overview of ethical and societal concerns
- ISO/IEC 25059:2023 — Quality model for AI systems
- ISO/IEC 42001:2023 — AI management system
- ISO/IEC 27001:2022 — Information security management
- ISO/IEC 29134:2023 — Privacy impact assessment
- EU AI Act (Regulation 2024/1689) — high-risk-system requirements
- US BIPA (Illinois Biometric Information Privacy Act, 740 ILCS 14)
- KR PIPA Article 23 — sensitive-information processing
- IETF RFC 9457 — Problem Details for HTTP APIs
- IETF RFC 9562 — UUIDs
- IETF RFC 8446 — TLS 1.3
- BIPM SI Brochure — Time scales

---

## 9. Implementer note — operational lifecycle

Vision-AI models drift: a face-recognition model trained on 2023 data will degrade against 2027 demographic shifts and adversarial-augmentation patterns. The wire-format discipline is what lets the operator catch drift early — through the per-cohort fairness metrics, the calibration-drift problem document, and the audit log of every inference. A first deployment that follows the runbook reaches production in about 60 days; ongoing model-refresh cycles typically run quarterly with full re-evaluation against the published thresholds before promotion.

弘益人間 — Benefit All Humanity.


## 10. Closing implementer note for Phase 2

Phase 2 endpoints are the day-to-day surface a vision-AI deployment exposes to its consumers. The discipline is high — every model published with full evaluation evidence, every inference auditable, every biometric query gated on the published DPIA — because vision AI's failure modes are visible to end users and impactful when wrong. The discipline is what distinguishes a vision-AI deployment auditable to EU AI Act standards from a vision-AI deployment that fails its first regulatory inquiry.

A first deployment that follows the runbook reaches production in about 60 days. The 60 days are dominated by dataset-evidence chain establishment, robustness assessment, and fairness evaluation; the actual model-serving infrastructure is well-trodden ground that integrates against existing runtimes within days.
