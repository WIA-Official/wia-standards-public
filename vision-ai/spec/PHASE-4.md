# WIA-AI-021 Vision AI Standard — PHASE 4 Specification

**Version:** 1.0
**Status:** Draft
**Date:** 2026-04-28
**Organization:** World Certification Industry Association (WIA)

---

## 弘益人間 (Benefit All Humanity)

---

## 1. Overview

Phase 4 specifies how a WIA-AI-021 vision-AI deployment integrates with the broader AI-engineering ecosystem: MLOps platforms, model-registry tooling, dataset-licensing infrastructure, regulatory-reporting pipelines (especially under the EU AI Act), and downstream-application stacks that consume vision inferences. The integration is layered: ONNX / OpenVINO / Triton carry the runtime interoperability; MLflow carries the experiment tracking; ISO/IEC 42001 carries the AI-management baseline; the EU AI Act and equivalent national laws carry the regulatory floor.

---

## 2. Bridge profiles

### 2.1 Bridge to model registries

The bridge profile maps Phase 1 model descriptors and Phase 2 model endpoints to common model-registry platforms:

| Registry | Native artefact | Bridge mapping |
|----------|----------------|----------------|
| MLflow | MLflow Model + experiment | ONNX export + Phase 1 model envelope |
| Hugging Face Hub | model card + repository | model card translated to ISO/IEC 23053 conformant format |
| Amazon SageMaker Model Registry | SageMaker model package | model package mapped to Phase 1 envelope |
| Azure ML Model Registry | Azure ML model | similar to above |
| Vertex AI Model Registry | Vertex model | similar |
| Korean 네이버 ClovaStudio | ClovaStudio model card | Korean-language model card with Phase 1 envelope wrap |

Bridge containers ship at `https://github.com/WIA-Official/wia-vision-ai-bridges`.

### 2.2 Bridge to inference runtimes

The runtime bridge maps Phase 2 inference endpoints to runtime-native invocation:

- **ONNX Runtime** — primary cross-platform runtime; CPU + GPU + ARM
- **NVIDIA Triton Inference Server** — GPU-optimised; Triton native ensemble support
- **OpenVINO** — Intel CPU + GPU + VPU; OpenVINO IR format
- **TensorFlow Serving** — TensorFlow SavedModel format
- **TorchServe** — PyTorch native serving
- **NVIDIA TensorRT** — GPU-optimised inference; latency-sensitive deployments
- **Apple Core ML** — on-device inference for iOS / macOS
- **Android NN API / TFLite** — on-device inference for Android

### 2.3 Bridge to MLOps platforms

MLOps platforms (Kubeflow, MLflow Pipelines, Apache Airflow + MLflow, Argo Workflows, Vertex AI Pipelines, Azure ML Pipelines, SageMaker Pipelines, Korean 카카오엔터프라이즈 KAIROS) consume Phase 1 model and dataset envelopes for orchestration. The bridge profile maps the standards artefacts to platform-native pipeline definitions.

### 2.4 Bridge to downstream applications

Downstream applications (manufacturing-defect detection, retail-checkout vision, medical-imaging assist, autonomous-vehicle perception, smart-city CCTV analytics) consume Phase 2 inference endpoints. The bridge profile documents per-application integration patterns and the model-card requirements that gate inferences for sensitive deployments.

---

## 3. Regulatory integration

### 3.1 EU AI Act (Regulation 2024/1689) high-risk-system flow

EU AI Act high-risk systems require:

- **Article 16** — provider obligations (registration, conformity assessment, CE marking)
- **Article 17** — quality management system (mapped to ISO/IEC 42001 attestation)
- **Article 18** — record-keeping (mapped to Phase 3 §8 audit log)
- **Article 23-29** — distributor / importer / deployer obligations
- **Article 49-50** — conformity assessment procedure
- **Article 71** — fundamental-rights impact assessment for public-deployer use

The bridge profile maps each Article to specific Phase 1-3 envelopes so a deployer can demonstrate compliance via envelope audit.

### 3.2 NIST AI Risk Management Framework integration

The NIST AI RMF (AI 100-1) provides a US-centric AI-risk approach. The bridge maps the four NIST RMF functions (Govern, Map, Measure, Manage) to specific Phase 1-3 envelopes:

| NIST AI RMF function | Phase artefact |
|----------------------|---------------|
| Govern | Operator's ISO/IEC 42001 attestation envelope |
| Map | Dataset evidence + use-case documentation in model card |
| Measure | Evaluation envelope + per-cohort fairness |
| Manage | Drift signal envelope + audit log |

### 3.3 Korean AI Framework Act (시행 2026) integration

Korea's AI Framework Act enters into force in 2026 with high-risk-system obligations parallel to the EU AI Act. The bridge profile maps the Korean Framework Act obligations to the same Phase 1-3 envelopes used for EU AI Act compliance, with Korean-language model cards and Korean-jurisdiction privacy floor (KR PIPA Article 23 sensitive-information processing).

### 3.4 Lawful-intercept compatibility

Jurisdictions requiring lawful intercept on biometric inference (rare but real in some surveillance regimes) declare the requirement in the discovery document. Sessions in those jurisdictions emit a notice envelope; the actual intercept happens through a separate signed channel that audit-logs every access.

---

## 4. Cross-standard composition

This Phase composes with adjacent WIA-family standards:

- **WIA-OMNI-API** — operator and downstream-application identity
- **WIA-AIR-SHIELD** — runtime trust list and key rotation for cross-operator federation
- **WIA-SOCIAL Phase 3 §5** — federation receipt shape reused for cross-operator model-publication chains
- **WIA-INTENT** — outermost-layer AI-system-intent declaration so deployers can verify intent matches the deployed model
- **WIA Secure Enclave (WIA-SEC-013)** — composes when models or inference inputs are sensitive (medical imaging, defence, biometric); the Phase 4 confidential-inference profile maps to the Secure Enclave's TEE-attested workload class
- **WIA Quantum Machine Learning** — composes when the operator deploys quantum-classical hybrid inference workloads
- **WIA Smart Lighting / Smart City / Sensory Enhancement** — the canonical downstream-application standards that consume Phase 2 inferences

---

## 5. Operational deployment runbook

A first vision-AI deployment that reaches production typically follows the runbook:

| Phase | Activity | Duration |
|-------|----------|----------|
| Day 0 | Reference container stood up; conformance suite run | 1 day |
| Day 1-7 | Model registry imported from operator's MLflow / SageMaker | 1 week |
| Day 8-21 | Dataset evidence chain established for at least one production dataset | 2 weeks |
| Day 22-35 | First model published with full robustness + fairness evaluation | 2 weeks |
| Day 36-50 | Drift-detection protocol wired to operator's monitoring system | 2 weeks |
| Day 51-60 | Bulk-export bridge to EU-AI-Act audit pipeline (when applicable) | 1-2 weeks |
| Day 61+ | Production cutover with shadow inference through Day 60; legacy retained as fallback | open-ended |

Lighter deployments (single-model inference services) compress this to 30 days; large deployments (model fleets serving multiple downstream applications) may take 6-12 months for full bridge coverage.

---

## 6. Compliance and certification

The standard maps to:

- **ISO/IEC 22989:2022** — AI concepts and terminology
- **ISO/IEC 23053:2022** — ML framework
- **ISO/IEC 24029-1 / -2** — Neural-network robustness
- **ISO/IEC TR 24027:2021** — Bias in AI systems
- **ISO/IEC TR 24368:2022** — Ethical and societal concerns
- **ISO/IEC 25059:2023** — AI quality model
- **ISO/IEC 42001:2023** — AI management system
- **ISO/IEC 27001:2022** — Information security management
- **ISO/IEC 29134:2023** — Privacy impact assessment
- **EU AI Act (Regulation 2024/1689)**
- **Korean AI Framework Act (시행 2026)**
- **NIST AI Risk Management Framework (AI 100-1)**
- **NIST AI 600-1** — Generative AI Profile
- **US BIPA / FRA / consumer-privacy laws** — biometric-data jurisdictions

Operators publish a signed conformance attestation envelope that names which compliance frames they claim and which audit evidence supports each claim.

---

## 7. Roadmap

| Version | Focus |
|---------|-------|
| 1.0.0 | Initial publication: ONNX + OpenVINO + Triton bridges; MLflow registry bridge; EU AI Act + NIST AI RMF compliance |
| 1.1.x | Additive: more model-registry bridges; deeper VLM (vision-language-model) coverage |
| 1.2.x | Additive: confidential vision-AI inside TEEs (composes with WIA Secure Enclave) |
| 1.3.x | Additive: on-device inference reference profiles (Apple Core ML, Android NN API, edge accelerators) |
| 1.4.x | Additive: synthetic-data provenance integration (composes with C2PA) |
| 2.0.0 (no earlier than 2028) | Possible breaking change: post-quantum signature suite migration |

The standard is maintained by the WIA Standards Committee. Change proposals follow the WIA RFC process; breaking changes require a two-thirds Committee vote plus a 12-month deprecation window per IETF RFC 8594 / 9745.

---

## 8. References

- ONNX — Linux Foundation
- OpenVINO 2024 — Intel
- NVIDIA Triton Inference Server 2.x
- TensorFlow Serving 2.x
- TorchServe / NVIDIA TensorRT / Apple Core ML / Android NN API / TFLite
- MLflow — Open-source ML lifecycle (Linux Foundation)
- MLPerf Inference v4.x — MLCommons
- NIST FRVT — Face Recognition Vendor Test
- ISO/IEC 22989 / 23053 / 24029-1 / 24029-2 / TR 24027 / TR 24368 / 25059 / 42001
- ISO/IEC 27001:2022
- ISO/IEC 29134:2023
- EU AI Act (Regulation 2024/1689)
- Korean AI Framework Act (시행 2026)
- NIST AI Risk Management Framework (NIST AI 100-1)
- NIST AI 600-1 — Generative AI Profile
- C2PA Content Credentials specification
- IETF RFC 8446 — TLS 1.3
- IETF RFC 8594 — sunset HTTP header
- IETF RFC 9745 — deprecation HTTP header

---

## 9. Closing implementer note

Vision AI is one of the highest-stakes AI domains in the WIA family: a misclassified medical image causes harm; a biased face-recognition model causes wrongful identification; a drifted manufacturing-defect detector causes recalls. The wire-format discipline is what lets operators, deployers, regulators, and impacted individuals verify the chain from training data through deployed inference without each consumer having to re-implement the trust machinery.

A first deployment that follows the runbook reaches production in about 60 days. The depth of dataset-evidence, robustness-assessment, fairness-evaluation, and EU-AI-Act-compliance work concentrated in those 60 days is what justifies the wire-format discipline. Subsequent model refreshes reuse the same machinery with per-refresh evaluation and approval gates.

弘益人間 — Benefit All Humanity.


## 10. Glossary expansion

VLM: Vision-Language Model. NMS: Non-Maximum Suppression, the standard post-processing step in object detection. ONNX IR: ONNX Intermediate Representation. OpenVINO IR: OpenVINO Intermediate Representation, Intel's framework-neutral model format. TFLite: TensorFlow Lite, the on-device variant. Triton: NVIDIA Triton Inference Server. MLflow: open-source ML-lifecycle platform. C2PA: Coalition for Content Provenance and Authenticity.

## 11. Implementer note — biometric special-category handling

Biometric inference (face recognition, gait recognition, iris recognition, voice biometrics) sits at the intersection of every privacy-law special-category protection: GDPR Article 9, KR PIPA Article 23, EU AI Act Article 5 (prohibited practices) and Annex III (high-risk practices), US BIPA in Illinois, US FRA in California. The standard's biometric-class handling (Phase 2 §1.2) is intentionally conservative: explicit consent, documented legal basis in discovery document, audit log of every biometric inference. Operators that find the discipline cumbersome should consider whether their use case actually requires biometric inference or whether a privacy-preserving alternative (anonymous classification, hashed-identifier matching) would suffice.
