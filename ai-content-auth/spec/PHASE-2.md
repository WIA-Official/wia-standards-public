# WIA-AI-017 Specification: PHASE 2
## Advanced Detection and Verification

**Version:** 1.0
**Status:** Published
**Date:** 2025-12-25
**Philosophy:** 弘益人間 (홍익인간) · Benefit All Humanity

---

## 1. Deepfake Detection

### 1.1 Detection Models

Implementations SHOULD support at least one detection approach:
- **CNN-based detectors** (REQUIRED)
- Transformer-based detectors (RECOMMENDED)
- Ensemble detectors (OPTIONAL)

### 1.2 Detection API

**Endpoint:** `POST /api/v1/detect/deepfake`

Request:
```json
{
    "media": "base64EncodedMedia",
    "media_type": "image|video|audio",
    "models": ["cnn", "transformer", "forensic"]
}
```

Response:
```json
{
    "is_synthetic": false,
    "confidence": 0.92,
    "analysis": {
        "visual_artifacts": 0.15,
        "temporal_consistency": 0.98,
        "audio_mismatch": 0.05
    },
    "suspicious_regions": [...]
}
```

### 1.3 Performance Requirements

- **Latency:** target latency declared in the deployment's published SLO.
- **Accuracy:** target accuracy declared in the deployment's transparency report against the WIA reference benchmark suite (§7.1).
- **False Positive Rate:** target rate declared in the deployment's transparency report.

Operating points and calibration are addressed in §7.1 (operating points) and §7.3 (calibration). Specific operating-point selection is a policy decision made by the deploying platform.

---

## 2. Forensic Analysis

### 2.1 Error Level Analysis (ELA)

Implementations SHOULD provide ELA for tamper detection:
```python
def perform_ela(image, quality=95):
    # Re-compress at specified quality
    # Compute pixel-wise difference
    # Amplify differences
    # Return anomaly map
```

### 2.2 Noise Pattern Analysis

Detect inconsistent camera noise patterns indicating manipulation.

### 2.3 JPEG Compression Artifacts

Analyze compression artifacts for evidence of re-compression or splicing.

---

## 3. Multi-Modal Verification

### 3.1 Audio-Visual Synchronization

For video content, verify lip-sync accuracy:
```json
{
    "sync_score": 0.89,
    "mismatch_frames": [45, 67, 89],
    "confidence": 0.91
}
```

### 3.2 Cross-Modal Consistency

Check for consistency across:
- Visual and audio features
- Lighting and shadows
- Reflections and refractions

---

## 4. Blockchain Integration

### 4.1 Provenance Anchoring

Optional blockchain anchoring for immutable provenance:

**Supported Blockchains:**
- Ethereum (RECOMMENDED)
- Polygon (OPTIONAL)
- Hyperledger Fabric (OPTIONAL)

### 4.2 Smart Contract Interface

```solidity
contract ProvenanceAnchor {
    struct ContentRecord {
        bytes32 contentHash;
        bytes32 manifestHash;
        address creator;
        uint256 timestamp;
    }

    mapping(bytes32 => ContentRecord) public records;

    function anchorContent(
        bytes32 contentId,
        bytes32 contentHash,
        bytes32 manifestHash
    ) external;

    function verifyContent(
        bytes32 contentId
    ) external view returns (ContentRecord memory);
}
```

---

## 5. Advanced Watermarking

### 5.1 Adaptive Watermarking

Watermark strength adapts to content characteristics:
- High texture regions: Stronger embedding
- Smooth regions: Weaker embedding

### 5.2 Multi-Bit Watermarks

Support for up to 256-bit payloads with error correction.

### 5.3 Synchronized Video Watermarking

Temporal watermark sequences across video frames.

---

## Detailed Detection Specifications

### Visual Forensic Stack

A conformant Phase 2 detection pipeline is layered. Each layer below is independent and contributes a confidence signal that the orchestrator combines:

1. **Compression-domain forensics** — Error Level Analysis on JPEG (ISO/IEC 10918-1), JPEG 2000 (ISO/IEC 15444-1), HEIF (ISO/IEC 23008-12), AVIF (AOMedia AV1), and PNG (ISO/IEC 15948) re-encodes. Output: pixel-wise tamper map.
2. **Frequency-domain forensics** — Discrete Cosine Transform (DCT) coefficient analysis, DFT-based residual analysis, wavelet decomposition (Daubechies family). Output: per-block authenticity score.
3. **Sensor-noise forensics** — Camera Photo-Response Non-Uniformity (PRNU) extraction and matching against the declared capture device fingerprint. Output: device-attribution score.
4. **Geometric consistency** — Lighting-direction estimation, shadow consistency, perspective consistency between background and inserted regions. Output: structural-coherence score.

### Audio Forensic Stack

For audio content, four independent signals are gathered:

1. **Spectral residual analysis** — FFT residuals between observed and reconstructed signal under the declared codec (Opus per RFC 6716, AAC per ISO/IEC 14496-3).
2. **Phase-coherence analysis** — Cross-channel phase coherence checks for synthesis artefacts in stereo and surround content.
3. **Voice-print mismatch** — When a declared speaker identity is present, the observed voice fingerprint is matched against the declared speaker profile.
4. **Environmental cue analysis** — Reverberation tail, room-impulse-response consistency, and background-noise stationarity checks.

### Multi-Modal Fusion

Per-modality scores are combined by a calibrated fusion model. The reference fusion uses logistic regression with isotonic-regression calibration on a held-out validation set. The output is a probability of synthetic origin in the range [0, 1] together with a confidence interval derived from bootstrap resampling.

### Threshold Policy

The conformant orchestrator emits one of three classes:

- **AUTHENTIC** — fused probability < 0.10 with confidence interval narrower than ±0.05.
- **INCONCLUSIVE** — fused probability in [0.10, 0.90] or wide confidence interval.
- **SYNTHETIC** — fused probability > 0.90 with confidence interval narrower than ±0.05.

Implementations MUST surface the INCONCLUSIVE class as a first-class outcome rather than collapsing to a binary decision.

## 6. Reference Standards Alignment

### 6.1 Cryptographic Foundations

WIA-AI-017 detection and verification operations are layered above well-established cryptographic standards. Implementations MUST conform to one of the following signature algorithm families:

| Family | Reference | Notes |
|--------|-----------|-------|
| EdDSA (Ed25519) | RFC 8032 | Default for new deployments. |
| ECDSA (P-256, P-384) | NIST FIPS 186-5; ISO/IEC 14888-3:2018 | Acceptable for legacy interoperability. |
| RSA-PSS | RFC 8017 §8.1; NIST FIPS 186-5 | Acceptable when constrained by hardware support. |

Hash functions for content fingerprinting MUST be one of: SHA-256, SHA-384, or SHA-512 per FIPS 180-4 (ISO/IEC 10118-3:2018), or SHA3-256 / SHA3-512 per FIPS 202 (ISO/IEC 10118-3:2018).

### 6.2 Token and Container Formats

The signed manifest carrying detection metadata follows COSE per RFC 9052 with CBOR encoding per RFC 8949. JOSE encodings (JWS per RFC 7515, JWE per RFC 7516) are accepted as the JSON-native alternative, with JWT claims per RFC 7519 for the manifest envelope.

### 6.3 Provenance Vocabulary

Detection result annotations interoperate with the C2PA (Coalition for Content Provenance and Authenticity) public technical specification 2.x, the W3C PROV-O vocabulary for provenance graphs, and the W3C Verifiable Credentials Data Model 2.0 for creator attestations. Implementations publishing C2PA-compatible manifests MUST include the WIA-AI-017 detection assertion under the `wia.detection.v1` label.

### 6.4 Time and Identity

Signed timestamps follow RFC 3161 (Internet X.509 PKI Time-Stamp Protocol) with the alternate CMS-Less profile per RFC 5816. Decentralised identifier resolution conforms to the W3C DID Core 1.0 specification when blockchain anchoring is used, with did:web and did:key as the recommended methods for creator attribution.

### 6.5 Conformance

A Phase 2 implementation is conformant when:

1. At least one detection model class declared in §1 is operational and meets the §1.3 performance bounds.
2. Forensic and multi-modal checks declared in §2–§3 are exposed via the API or documented as not supported.
3. Cryptographic primitives match §6.1 with declared family in the manifest.
4. Manifest container format is COSE/CBOR or JOSE/JSON per §6.2.

## 7. Implementation Appendix

### 7.1 Operating points and calibration

A detection pipeline is characterised by its receiver operating characteristic (ROC) and by the calibration of its score-to-probability mapping. Conformant implementations publish:

- The ROC curve evaluated on the WIA reference benchmark suite.
- The Brier score and expected calibration error (ECE).
- The chosen operating point with its false-positive and false-negative rates.

Operators select a default operating point per the policy of their deployment context. Newsroom deployments typically select a low false-positive operating point to avoid wrongful repudiation; platform-scale moderation may select a higher recall point to surface candidates for human review.

### 7.2 Adversarial robustness

Detection models are evaluated against the standard adversarial protocols documented in the public literature on adversarial machine learning. The reference evaluation includes:

- Black-box transfer attacks using surrogate models trained on the same task.
- White-box gradient attacks bounded by L2 and L∞ perturbation budgets.
- Physical-world attacks that survive recapture (printing and rephotographing).

Implementations report robust accuracy at three perturbation budgets and disclose any model-specific defensive measures (e.g., randomised smoothing, certified robustness training).

### 7.3 Drift and monitoring

Synthetic content distributions evolve. The reference monitoring stack tracks:

- Daily detection-score histograms by content modality and class.
- Per-domain false-positive rates derived from human-review queues.
- Time-series of model confidence on a stationary canary set.

Significant drift on the canary set (Kolmogorov-Smirnov D-statistic exceeding the policy threshold) triggers a revalidation or retraining workflow. Retraining cycles are governed by the Phase 3 enterprise governance procedures.

### 7.4 Test plan

The reference test plan covers seven categories: end-to-end detection, modality-specific detection, fusion correctness, calibration, robustness, drift detection, and API conformance. Each category contains scripted test vectors with expected outputs, and the suite is published as an open package so that any implementation can self-assess conformance prior to a formal review.

### 7.5 Operating cost considerations

Detection workloads are bounded by GPU and accelerator availability. Reference deployments report compute cost in three units:

- **Verifications per dollar** at a fixed pricing reference (publicly available cloud GPU instance types).
- **Energy per verification** in joules, derived from the GPU's reported power state and the verification's wall-clock time.
- **Carbon-equivalent per verification** using the published grid carbon intensity for the deployment region.

These three figures appear on the conformance report so adopters can match a deployment topology to their cost and sustainability constraints.

### 7.6 Human-in-the-loop interactions

When detection produces an INCONCLUSIVE outcome, the orchestrator routes the asset to a human review queue. The reference review tool surfaces:

- The fused detection probability and confidence interval.
- Per-modality scores with explanatory visualisations.
- The originating manifest and any prior detection telemetry on the same asset.
- Reviewer guidance keyed to the specific failure mode that produced the inconclusive verdict.

Reviewer outcomes are written back to the audit log with the reviewer's identity (or pseudonym, depending on policy) and used as supervisory signal for future calibration of the fusion model.

### 7.7 Cross-reference Matrix

| Phase 2 capability | Phase 1 dependency | Phase 3 dependency |
|--------------------|---------------------|---------------------|
| Detection orchestrator | manifest container (§6.2) | throughput SLO (§1.1) |
| Forensic analysis | hash function (§6.1) | telemetry retention (§7.5) |
| Multi-modal verification | media codec encodings (§6.5) | observability (§6.2) |
| Blockchain anchoring | DID identity (§6.4) | regulatory alignment (§6.4) |
| Watermark (adaptive) | signature family (§6.1) | edge tier deployment (§3) |

This matrix is informative; conformance is determined by the per-phase clauses, not by this matrix.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
