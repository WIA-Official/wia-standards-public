# WIA-AI-014: Neural Network Format - Phase 4 Integration Specification

**Version:** 1.0.0
**Philosophy:** 弘益人間 - Benefit All Humanity

## 1. MLOps Integration

### 1.1 CI/CD Pipeline Integration

```yaml
# .github/workflows/model-deployment.yml
name: Model Deployment
on:
  push:
    paths:
      - 'models/**'

jobs:
  validate-and-deploy:
    runs-on: ubuntu-latest
    steps:
      - name: Validate Model
        run: wia-ai-014 validate model.wia

      - name: Test Inference
        run: wia-ai-014 test model.wia --test-data test_inputs.json

      - name: Deploy to Registry
        run: wia-ai-014 deploy model.wia --registry prod
```

### 1.2 Model Registry Integration

```python
from wia_ai_014 import ModelRegistry

# Connect to registry
registry = ModelRegistry("https://registry.company.com")

# Register model
registry.register(
    name="resnet50",
    version="2.1.0",
    model_path="model.wia",
    metadata={
        "accuracy": 0.761,
        "framework": "pytorch"
    }
)
```

## 2. Cloud Platform Integration

### 2.1 Kubernetes Deployment

```yaml
apiVersion: v1
kind: ConfigMap
metadata:
  name: model-config
data:
  model.wia: |
    # WIA-AI-014 model file

---
apiVersion: apps/v1
kind: Deployment
metadata:
  name: model-server
spec:
  replicas: 3
  template:
    spec:
      containers:
      - name: server
        image: wia/model-server:latest
        env:
        - name: MODEL_PATH
          value: /models/model.wia
```

### 2.2 Cloud Storage Integration

```typescript
// AWS S3
import { S3ModelStorage } from '@wia/ai-014-aws';

const storage = new S3ModelStorage({
    bucket: 'models',
    region: 'us-east-1'
});

await storage.upload('model.wia', 'models/resnet50/v2.1.0');

// GCP Cloud Storage
import { GCSModelStorage } from '@wia/ai-014-gcp';

const gcs = new GCSModelStorage({
    bucket: 'ml-models',
    project: 'my-project'
});
```

## 3. Monitoring Integration

### 3.1 Prometheus Metrics

```
# HELP model_prediction_requests_total Total prediction requests
# TYPE model_prediction_requests_total counter
model_prediction_requests_total{model="resnet50",version="2.1.0"} 1000

# HELP model_prediction_latency_seconds Prediction latency
# TYPE model_prediction_latency_seconds histogram
model_prediction_latency_seconds{model="resnet50",version="2.1.0"} 0.015
```

### 3.2 Logging Integration

```json
{
  "timestamp": "2025-01-15T10:00:00Z",
  "level": "INFO",
  "model": "resnet50",
  "version": "2.1.0",
  "request_id": "req-123",
  "latency_ms": 15.2,
  "input_shape": [1, 3, 224, 224],
  "output_shape": [1, 1000]
}
```

## 4. Framework Compatibility

### 4.1 Supported Frameworks

- PyTorch ≥ 1.11
- TensorFlow ≥ 2.8
- ONNX Runtime ≥ 1.11
- scikit-learn ≥ 1.0

### 4.2 Platform Support

- Linux (x86_64, ARM64)
- macOS (Intel, Apple Silicon)
- Windows (x86_64)
- Mobile (iOS, Android via TFLite/CoreML)
- Web (ONNX.js, TF.js)

## 5. Hardware Backend Integration

### 5.1 NVIDIA / CUDA

| Component | Recommendation | Notes |
|-----------|----------------|-------|
| Driver | ≥ 535 LTS | required for FP8 ops |
| CUDA Toolkit | ≥ 12.1 | aligned with CUTLASS 3.x |
| TensorRT | ≥ 9.0 | INT8 + FP8 quantization |
| Triton Inference Server | ≥ 23.10 | ships native WIA-AI-014 backend |

A reference Triton `model_repository` layout:

```
model_repository/resnet50/1/model.wia
model_repository/resnet50/config.pbtxt
```

`config.pbtxt` MUST set `backend: "wia_ai_014"` and `parameters [{ key: "format_version" value: { string_value: "1.0" } }]`.

### 5.2 AMD ROCm

- ROCm ≥ 6.0 with MIGraphX runtime
- HIP-aware kernels MUST publish FP16 and BF16 throughput in the model card
- Hot-swap to `rocblas` for GEMMs, `rocFFT` for transforms

### 5.3 Apple Silicon

- macOS 14+ via Metal Performance Shaders Graph (MPSGraph)
- Core ML 7+ for on-device deployment via `coremltools.convert(model, source='wia-ai-014')`

### 5.4 Edge / TinyML

- ARM Ethos-U / Cortex-M via `wia-ai-014 export --target ethosu55`
- Quantization MUST be int8 symmetric to fit constrained MCUs
- The exporter MUST produce a TFLite-compatible buffer with the WIA metadata block at the head

## 6. Observability Standards

### 6.1 OpenTelemetry Mapping

Every prediction span MUST carry the following attributes:

| Attribute | Value |
|-----------|-------|
| `wia.ai_014.model_name` | string |
| `wia.ai_014.model_version` | semver string |
| `wia.ai_014.batch_size` | int |
| `wia.ai_014.device` | string |
| `wia.ai_014.precision` | enum |
| `wia.ai_014.cache_hit_ratio` | float [0,1] |

Trace context MUST follow W3C Trace Context (`traceparent`/`tracestate` headers).

### 6.2 Drift & Outlier Telemetry

Prediction servers SHOULD emit population telemetry every minute:

```json
{
  "model": "resnet50",
  "version": "2.1.0",
  "windowSeconds": 60,
  "inputDistribution": {"mean": [...], "std": [...]},
  "outputEntropy": 2.31,
  "outlierFraction": 0.018
}
```

Down-stream drift detectors (Evidently, Whylogs, Arize) MAY ingest this stream directly without bespoke parsers.

## 7. Security Integration

### 7.1 Secret Management

- Model encryption-at-rest MUST use AES-GCM-256 with keys held in HSM, KMS, or SPIRE-issued workload secrets.
- Decryption MUST occur after process boot and never be persisted to disk.
- Rotated keys MUST be honoured within 24 hours of issuance.

### 7.2 Supply Chain

- Every released model artifact MUST ship a SLSA Provenance v1 attestation alongside the `model.wia` blob.
- Sigstore (Cosign) signatures over the artifact digest MUST be verified at deploy time.
- The model card MUST reference the SBOM (CycloneDX 1.5 or SPDX 2.3) used to build the runtime.

## 8. Disaster Recovery

| Scenario | RTO | RPO | Mechanism |
|----------|-----|-----|-----------|
| Single replica crash | < 30 s | 0 | Kubernetes restart |
| Region failure | < 5 min | 0 | Active-active across regions |
| Model corruption | < 30 min | last good revision | Registry rollback to previous SemVer |
| Registry outage | < 60 min | 0 | Mirror to cold-standby registry |

Operators MUST run a quarterly tabletop exercise verifying the rollback flow on a sample model.

## 9. Conformance Suite

```bash
wia-ai-014 conformance run \
  --target https://serving.example.com \
  --model resnet50 \
  --suite all \
  --report ./conformance-report.json
```

The suite covers:

- Tensor wire framing fixtures (Phase 3 §6)
- Streaming inference fixtures (Phase 3 §7)
- Error model assertions (RFC 9457 / `google.rpc.Status`)
- Quantization round-trip tests
- Provenance + signature verification

A clean run is a precondition for the public "WIA-AI-014 Conformant" badge.

## 10. Operational Runbooks

| Symptom | Most likely cause | First step |
|---------|-------------------|------------|
| 503 spikes after deploy | Cold cache, model not warm | Issue `:warmup` with representative batch |
| p99 latency drift | Memory fragmentation | Restart on schedule, enable workspace reuse |
| Throughput plateau on GPU | Kernel launch overhead | Raise `max_batch_size`, enable CUDA Graphs |
| OOM under load | Activation memory growth | Lower batch, enable activation checkpointing |
| Stalled streams | Back-pressure exhausted | Increase `MaxConcurrentStreams`, scale replicas |

Each runbook entry MUST point to an ops dashboard URL stored in the model card.

## 11. Migration & Backwards Compatibility

### 11.1 Format Version Migration

The `wia` blob MUST embed `format_version` at offset 0. Loaders MUST refuse blobs whose major version exceeds their supported set. Deprecation is two-phase:

1. **Soft deprecation**: warning log, telemetry counter `deprecated_format_load_total` increments.
2. **Hard removal**: loader returns `ModelFormatError` with type slug `format-too-old`.

Each minor format version MUST ship a deterministic upgrader binary (`wia-ai-014 migrate`).

### 11.2 Model Card Schema

Model cards MUST conform to the WIA Model Card v1 schema published at `https://standards.wia.example/neural-network-format/schemas/v1/model-card.schema.json`. Required fields:

```json
{
  "name": "resnet50",
  "version": "2.1.0",
  "task": "image-classification",
  "intendedUse": "Educational reference; not for medical, legal, or safety-critical decisions",
  "limitations": "Trained on a public benchmark; performance off-distribution is not guaranteed.",
  "trainingData": {"description": "Public benchmark", "license": "CC-BY-4.0"},
  "evaluationProtocol": {"split": "val", "metrics": ["top1","top5"]},
  "responsibleAi": {
    "biasReviewed": true,
    "fairnessNotes": "See companion model card.",
    "carbonEstimate": null
  },
  "provenance": {
    "slsaLevel": 3,
    "sbomUrl": "https://...",
    "signature": "https://..."
  }
}
```

## 11.3 Hot-Reload Procedure

For latency-sensitive deployments the registry MUST support zero-downtime reloads:

```
serve_replica:
  on_new_version(name, new_v):
    handle = load(new_v)               # warmed in shadow process
    swap_atomic(active_handle, handle) # CAS pointer
    drain(old_handle, deadline=30s)
    free(old_handle)
```

A replica MUST emit `model.swap` telemetry before and after the swap so dashboards can correlate latency spikes with the rollout boundary.

## 11.4 A/B and Canary

Operators MAY assign weighted traffic to multiple versions of the same model:

```yaml
trafficSplit:
  - version: 2.0.0
    weight: 95
  - version: 2.1.0
    weight: 5
guardrails:
  - metric: p99_latency_ms
    direction: lower-is-better
    rollback_threshold: 1.20  # 20% worse than baseline
  - metric: error_rate
    rollback_threshold: 0.02
```

If any guardrail trips, the replica MUST automatically route 100% back to the baseline version and page the on-call.

## 12. Compliance Mapping

| Regulation | Mapped controls |
|-----------|----------------|
| EU AI Act (high-risk systems) | Phase 2 §13 versioning, Phase 3 §10 auth, Phase 4 §6.2 drift telemetry |
| ISO/IEC 42001 (AI management) | Phase 4 §11.2 model card, §6 observability |
| NIST AI RMF | Phase 4 §10 runbooks, §11 migration |
| ISO/IEC 27001 | Phase 4 §7 security integration |

The mapping is informative; conformance with a given regulation requires the operator to layer their own attestations on top of the technical controls.

## 12.1 Privacy & Data Protection

Implementations that handle personal data MUST:

- Honour data-subject deletion requests by purging cached embeddings and any retained input/output tensors.
- Encrypt logs at rest with the same KMS key class as the model itself.
- Tag all telemetry that originates from a regulated tenant with `wia.regulated=true` so downstream consumers can apply purpose limitation.
- Refuse to embed raw personal data inside Telemetry frames; only structural metrics are permitted there.

## 12.2 Geographic Routing

For deployments that span jurisdictions the `X-WIA-Region` header MUST be honoured:

```
X-WIA-Region: eu-west
```

Routers MUST refuse to forward a request whose declared region is not present in the model card's `availableRegions` list.

## 12.3 Audit Trail

A tamper-evident audit log MUST capture every model swap, configuration change, and access-policy update. Each entry is a signed JWS (RFC 7515) chained via the previous entry's hash. The log itself MUST be persisted to immutable storage (object store with object-lock) with a minimum retention of 365 days for production tenants.

## 12.4 Cross-Border Transfer Controls

Operators that cross GDPR/PIPEDA/PIPA boundaries MUST configure the runtime with an explicit `dataResidencyZones` list. Requests originating outside the listed zones MUST receive HTTP 451 (Unavailable For Legal Reasons, RFC 7725) with a `Link: <…>; rel="ToS"` header pointing at the operator's data-protection page.

Clients SHOULD interpret 451 responses as terminal — retrying with the same identity will not change the outcome, and only an operator-side configuration change can re-enable access.

## 13. Normative References

- ISO/IEC 27001:2022 — Information Security Management
- ISO/IEC 42001:2023 — AI management systems
- ISO/IEC 5230:2020 — OpenChain Specification
- IETF RFC 9457 — Problem Details for HTTP APIs
- W3C Trace Context — Distributed tracing identifiers
- DMTF DSP0266 — Redfish Specification (telemetry transport)
- SLSA Framework v1.0 — Supply-chain Levels for Software Artifacts
- CycloneDX 1.5 — Software Bill of Materials
- SPDX 2.3 — Software Package Data Exchange
- ONNX Operator Set v18

---

**Copyright © 2025 WIA**
**弘益人間 · Benefit All Humanity**
