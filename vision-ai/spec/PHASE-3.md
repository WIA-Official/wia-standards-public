# WIA-AI-021 Vision AI Standard — PHASE 3 Specification

**Version:** 1.0
**Status:** Draft
**Date:** 2026-04-28
**Organization:** World Certification Industry Association (WIA)

---

## 弘益人間 (Benefit All Humanity)

---

## 1. Overview

Phase 3 specifies the on-the-wire protocols by which a WIA-AI-021 deployment coordinates across trust boundaries — between model operators and dataset providers, between operators and downstream-application consumers, between operators and regulators (especially under the EU AI Act high-risk-system regime), between operators and independent evaluators (NIST, MLPerf, academic peer reviewers). The protocols are layered above the Phase 2 API surface and inherit ISO/IEC 23053 ML-framework conventions, ISO/IEC 24029 robustness-assessment conventions, and ISO/IEC 42001 AI-management conventions.

### 1.1 Time discipline

All Phase 3 protocol exchanges carry timestamps in **TAI** per BIPM SI Brochure conventions (RFC 3339 with explicit TAI offset). Drift-detection windows depend on accurate timestamps; the protocol preserves wall-clock-correct timestamps even across leap-second events.

### 1.2 Replay defence bounds

Every protocol envelope carries a 96-bit nonce and a TAI timestamp. Receivers reject envelopes with skew greater than ±300 seconds and maintain a 600-second seen-nonce cache. The cache is persistent across console restarts so a power cycle does not re-open the window for a previously-blocked replay.

---

## 2. Model-publication protocol

When an operator publishes a new model (initial release or refresh), the publication protocol formalises the dependency chain that the model relies on: training-dataset provenance, evaluation evidence, fairness assessment, robustness assessment, and the model card.

### 2.1 Publication state machine

```
DRAFT     → registered in operator's MLflow with experiment artefacts
EVALUATED → evaluation envelopes published; per-cohort fairness reported
ROBUST    → ISO/IEC 24029-2 robustness assessment passed
STAGED    → promoted to staging environment for shadow-traffic evaluation
PRODUCTION → promoted to production after operator's published threshold met
DEPRECATED → marked for retirement; consumers MUST migrate within deprecation window
RETIRED   → no further inferences served; archive retained for audit
```

### 2.2 Publication envelope schema

```json
{
  "wia_ai_021_version": "1.0.0",
  "type": "model_publication",
  "model_id": "mdl_01HX...",
  "modality": "object_detection",
  "framework": "PyTorch 2.x",
  "exported_runtime": "ONNX 1.16",
  "training_dataset_ref": "ds_01HX...",
  "evaluation_ref": "eval_01HX...",
  "robustness_ref": "rob_01HX...",
  "model_card_ref": "<URI of ISO/IEC 23053 conformant model card>",
  "ai_act_risk_tier": "high_risk",
  "promoted_at_tai": "<TAI>",
  "operator_signature": { "alg": "Ed25519", "value": "..." }
}
```

The `operator_signature` is the operator's commitment to the published model's claimed properties. Downstream consumers verify the signature before deploying the model in their applications.

---

## 3. Dataset-evidence chain protocol

Training-dataset provenance is the highest-leverage attack surface in any AI standard: a corrupted dataset produces a corrupted model, and the corruption can survive every downstream evaluation that uses the same corrupted data. The standard requires every published model to chain to a documented dataset-evidence envelope.

### 3.1 Dataset envelope schema

```json
{
  "wia_ai_021_version": "1.0.0",
  "type": "dataset_evidence",
  "dataset_id": "ds_01HX...",
  "name": "...",
  "source_attribution": [
    { "source": "ImageNet 21k", "license": "ImageNet-1k Research", "sample_count": 1281167 }
  ],
  "collection_method": "<methodology reference>",
  "demographic_distribution": {
    "method": "ISO/IEC TR 24027:2021 cohort definition",
    "report_uri": "<URI>"
  },
  "manifest_merkle_root": "0x...",
  "datasheet_for_datasets_uri": "<URI of Gebru-style datasheet>",
  "issued_at_tai": "<TAI>",
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

### 3.2 Per-sample auditability

The dataset manifest carries a hash commitment over every included sample. A consumer can verify any individual sample's inclusion via Merkle proof without downloading the full dataset. The discipline supports adversarial-sample audits where investigators verify whether a specific image was used in training a specific model.

### 3.3 Licensing chain

Dataset licensing is enforced at the protocol layer: every dataset evidence envelope declares the license; every model publication envelope chains to a licensed dataset; the operator's bulk-export endpoint refuses exports that would violate the chained licensing terms.

---

## 4. Robustness-assessment protocol

ISO/IEC 24029-2:2023 specifies the robustness-assessment methodology for neural networks. The standard's robustness protocol formalises the wire format for publishing assessment results and supporting independent re-evaluation.

### 4.1 Robustness envelope schema

```json
{
  "wia_ai_021_version": "1.0.0",
  "type": "robustness_assessment",
  "assessment_id": "rob_01HX...",
  "model_id": "mdl_01HX...",
  "iso_24029_2_version": "2023",
  "perturbation_classes": [
    { "kind": "adversarial_l_inf", "epsilon": 0.03, "attack": "PGD" },
    { "kind": "common_corruption", "library": "ImageNet-C", "severity": "1-5" },
    { "kind": "spatial", "operations": ["rotation", "scaling", "translation"] }
  ],
  "metric_results": {
    "clean_accuracy": 0.84,
    "robust_accuracy_l_inf_eps_0_03": 0.42,
    "common_corruption_mean_accuracy": 0.71
  },
  "test_vector_set_ref": "<URI>",
  "issued_at_tai": "<TAI>",
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

### 4.2 Independent verification

The standard does not adjudicate robustness; it provides the wire format that lets independent evaluators re-run the assessment against the same test-vector set and publish their own envelope. Disputes between operator-reported and independent-evaluator-reported robustness flow through the audit log so consumers can decide which to trust.

---

## 5. Fairness-evaluation protocol

Per-cohort fairness evaluation follows ISO/IEC TR 24027:2021. The protocol envelopes carry per-cohort accuracy / true-positive-rate / false-positive-rate / equalised-odds metrics across the operator's documented cohort definition.

### 5.1 Cohort definition discipline

Cohort definitions MUST be documented in advance (typically in the model card under §"Use cases" and §"Out-of-scope use cases"). Post-hoc cohort selection that hunts for favourable metrics is the canonical bias-laundering pattern; the standard's discipline (cohort definition published before evaluation, evaluation re-runnable from envelope alone) blocks the pattern.

### 5.2 Aggregation discipline

Aggregate fairness metrics across heterogeneous cohorts can hide disparate impact. The protocol envelope publishes per-cohort metrics in addition to any aggregate so a consumer can verify the operator's aggregate is not aggregation-bias.

---

## 6. Drift-detection protocol

Production model drift (data drift, concept drift, fairness drift) is a recurring failure mode. The drift-detection protocol formalises the wire format for publishing drift signals and triggering re-evaluation.

### 6.1 Drift envelope schema

```json
{
  "wia_ai_021_version": "1.0.0",
  "type": "drift_signal",
  "model_id": "mdl_01HX...",
  "kind": "data_drift" | "concept_drift" | "fairness_drift" | "calibration_drift",
  "metric": "...",
  "baseline_value": 0.84,
  "current_value": 0.78,
  "threshold_breached": true,
  "detection_window_start_tai": "<TAI>",
  "detection_window_end_tai": "<TAI>",
  "recommended_action": "re_evaluate" | "rollback_to_previous" | "operator_review",
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

### 6.2 Operational discipline

When drift breaches threshold, the operator MUST re-evaluate the model against the published thresholds before continued production use. The audit log carries every drift signal so a regulator inquiring after a problematic inference can reconstruct whether the operator was on notice of the drift.

---

## 7. EU-AI-Act compliance protocol (high-risk systems)

EU AI Act Regulation 2024/1689 imposes specific obligations on high-risk AI systems: risk management, data governance, technical documentation, record-keeping, transparency, human oversight, accuracy, robustness, cybersecurity. The compliance protocol maps each Article-9 through Article-15 obligation to specific Phase 1-3 envelopes:

| EU AI Act Article | Phase artefact |
|-------------------|---------------|
| Art. 9 — Risk management | Model card §"Risk assessment" + robustness envelope (§4) |
| Art. 10 — Data governance | Dataset evidence envelope (§3) |
| Art. 11 — Technical documentation | Model card + Phase 1 evaluation envelope |
| Art. 12 — Record-keeping | Audit log of every inference + Phase 2 §4 bulk export |
| Art. 13 — Transparency | Model card + Phase 2 §2.6 NIST FRVT score |
| Art. 14 — Human oversight | Phase 1 §2.x human-oversight protocol declaration |
| Art. 15 — Accuracy / robustness / cybersecurity | Robustness envelope + ISO/IEC 27001 attestation |

---

## 8. Audit log discipline

Every Phase 3 protocol envelope is written to an append-only log replicated across at least two storage backends. Retention is sized to the longest applicable regulatory window: typically 10 years for EU-AI-Act high-risk-system records, 7 years for general AI-management records, and per-incident retention extending to the close of any active investigation.

---

## 9. Cross-standard composition

Phase 3 composes with: WIA-OMNI-API for operator and reviewer identity, WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 §5 for the federation receipt shape, WIA Secure Enclave for confidential inference (when models or inputs are sensitive), and WIA-INTENT for declaring outermost-layer AI-system intent.

---

## 10. References

- ONNX Specification — Linux Foundation
- ISO/IEC 22989:2022 / 23053:2022 — AI concepts and ML framework
- ISO/IEC 24029-1:2021 / 24029-2:2023 — Robustness of neural networks
- ISO/IEC TR 24027:2021 — Bias in AI systems
- ISO/IEC 25059:2023 — Quality model for AI systems
- ISO/IEC 42001:2023 — AI management system
- ISO/IEC 27001:2022 — Information security management
- ISO/IEC 29134:2023 — Privacy impact assessment
- EU AI Act (Regulation 2024/1689)
- NIST AI Risk Management Framework (NIST AI 100-1)
- NIST FRVT — Face Recognition Vendor Test
- MLPerf Inference v4.x — MLCommons benchmark suite
- BIPM SI Brochure — Time scales (TAI / UTC / leap seconds)
- IETF RFC 8446 — TLS 1.3

---

弘益人間 — Benefit All Humanity.


## 11. Glossary expansion

Drift: change in production distribution of inputs (data drift) or in the relationship between inputs and labels (concept drift) over time. Calibration: alignment between model-reported confidence and observed accuracy. Robustness: tolerance to adversarial or naturally-occurring input perturbations. Fairness: comparable performance across demographic cohorts. Datasheet for Datasets: structured documentation of dataset provenance, collection method, intended uses, and limitations, formalised by ISO/IEC TR 24368:2022 conventions for AI-system documentation. NIST FRVT: Face Recognition Vendor Test. MLPerf: industry-standard ML benchmark suite (MLCommons consortium). C2PA: Coalition for Content Provenance and Authenticity, the canonical synthetic-content provenance standard.

## 12. Implementer note — sustainable AI governance

The standards Phase 3 protocol layer is what lets operators run sustainable AI governance across long deployments. Without the published model-publication, dataset-evidence, robustness, and fairness envelopes, governance becomes ad-hoc per refresh; with them, governance is automated against documented thresholds. The discipline scales: an operator running 50 production models reuses the same envelope shapes and the same audit pipeline across all 50.


## 13. Closing protocol note for Phase 3

Phase 3 protocol exchanges are the load-bearing layer where vision-AI accountability either survives or fails. A model that ships without dataset evidence, robustness assessment, or fairness evaluation is a model that an EU AI Act inspector will refuse to clear and that a US BIPA plaintiff will use as evidence of negligence. The discipline is rigid by design.
