# WIA-AI-012: Federated Learning - Phase 1: Data Format

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-01-09

## Overview

This specification defines the data formats and message structures for federated learning systems, ensuring interoperability between clients and servers from different vendors.

## Philosophy

弘益人間 (Hongik Ingan) - Benefit All Humanity

This standard enables privacy-preserving collaborative machine learning by defining clear, open data formats that any implementation can adopt.

## 1. Model Representation

### 1.1 Model Package Format

```json
{
  "model_id": "uuid-v4-string",
  "version": "semver-string",
  "created_at": "ISO-8601-timestamp",
  "framework": "tensorflow|pytorch|jax|onnx",
  "architecture": {
    "type": "neural_network|linear_model|tree_ensemble",
    "layers": [...],
    "parameters_count": 0
  },
  "weights": {
    "format": "base64|binary|compressed",
    "encoding": "float32|float16|int8",
    "data": "..."
  },
  "metadata": {
    "task": "classification|regression|generation",
    "input_shape": [batch, height, width, channels],
    "output_shape": [batch, classes],
    "performance_metrics": {}
  }
}
```

### 1.2 Model Update Format

```json
{
  "update_id": "uuid",
  "client_id": "hashed-client-identifier",
  "round_number": 0,
  "base_model_version": "semver",
  "update_type": "gradient|weights|delta",
  "update_data": {
    "format": "sparse|dense|compressed",
    "compression": "quantization|sparsification|none",
    "parameters": {
      "layer_name": {
        "shape": [dim1, dim2, ...],
        "dtype": "float32|int8",
        "values": "base64-encoded-data"
      }
    }
  },
  "training_metrics": {
    "local_epochs": 5,
    "batch_size": 32,
    "samples_trained": 1000,
    "local_loss": 0.123,
    "local_accuracy": 0.956
  },
  "privacy": {
    "differential_privacy": {
      "epsilon": 1.0,
      "delta": 1e-5,
      "noise_multiplier": 0.1
    },
    "secure_aggregation": true
  },
  "signature": "cryptographic-signature"
}
```

## 2. Communication Messages

### 2.1 Server to Client: Training Invitation

```json
{
  "message_type": "training_invitation",
  "message_id": "uuid",
  "timestamp": "ISO-8601",
  "round_number": 42,
  "global_model": {
    "url": "https://server/models/v1.2.3.model",
    "checksum": "sha256-hash",
    "size_bytes": 104857600
  },
  "training_config": {
    "local_epochs": 5,
    "batch_size": 32,
    "learning_rate": 0.01,
    "optimizer": "sgd|adam|adagrad",
    "loss_function": "cross_entropy|mse"
  },
  "deadline": "ISO-8601-timestamp",
  "min_clients_required": 100,
  "eligibility_criteria": {
    "min_battery_level": 0.3,
    "required_network": "wifi",
    "min_storage_mb": 100
  }
}
```

### 2.2 Client to Server: Update Submission

```json
{
  "message_type": "update_submission",
  "message_id": "uuid",
  "timestamp": "ISO-8601",
  "client_id": "hashed-id",
  "round_number": 42,
  "model_update": {
    "url": "https://client/updates/update-42.bin",
    "checksum": "sha256",
    "size_bytes": 1048576,
    "format": "model_update_v1"
  },
  "training_report": {
    "started_at": "ISO-8601",
    "completed_at": "ISO-8601",
    "duration_seconds": 120,
    "samples_trained": 1000,
    "final_loss": 0.123,
    "final_accuracy": 0.956
  },
  "device_info": {
    "platform": "android|ios|linux|windows",
    "cpu": "arm|x86",
    "memory_mb": 4096,
    "network_type": "wifi|4g|5g"
  },
  "signature": "cryptographic-signature"
}
```

### 2.3 Server to Client: Aggregation Result

```json
{
  "message_type": "aggregation_result",
  "message_id": "uuid",
  "timestamp": "ISO-8601",
  "round_number": 42,
  "status": "success|failed|partial",
  "new_global_model": {
    "version": "1.2.4",
    "url": "https://server/models/v1.2.4.model",
    "checksum": "sha256",
    "size_bytes": 104857600
  },
  "round_statistics": {
    "clients_invited": 5000,
    "clients_participated": 2341,
    "updates_accepted": 2300,
    "updates_rejected": 41,
    "global_loss": 0.089,
    "global_accuracy": 0.967
  },
  "next_round": {
    "scheduled_start": "ISO-8601",
    "estimated_participants": 2500
  }
}
```

## 3. Privacy-Preserving Formats

### 3.1 Differential Privacy Metadata

```json
{
  "privacy_mechanism": "differential_privacy",
  "epsilon": 1.0,
  "delta": 1e-5,
  "sensitivity": 1.0,
  "noise_distribution": "laplace|gaussian",
  "noise_scale": 0.1,
  "clipping_norm": 1.0,
  "composition_method": "basic|advanced|renyi"
}
```

### 3.2 Secure Aggregation Envelope

```json
{
  "secure_aggregation": {
    "protocol": "bonawitz2017|bell2020",
    "participant_id": "client-hash",
    "round_id": "uuid",
    "encrypted_share": "base64-encoded-ciphertext",
    "commitment": "sha256-hash",
    "public_key": "base64-encoded-public-key"
  }
}
```

## 4. Compression Formats

### 4.1 Quantization Metadata

```json
{
  "compression_type": "quantization",
  "original_dtype": "float32",
  "quantized_dtype": "int8|uint8|int16",
  "num_bits": 8,
  "scale": 0.01,
  "zero_point": 128,
  "min_value": -1.28,
  "max_value": 1.27
}
```

### 4.2 Sparsification Metadata

```json
{
  "compression_type": "sparsification",
  "sparsity_ratio": 0.01,
  "num_nonzero": 10000,
  "total_elements": 1000000,
  "indices_format": "coo|csr|csc",
  "indices": "base64-encoded-indices",
  "values": "base64-encoded-values"
}
```

## 5. Error Handling

### 5.1 Error Response Format

```json
{
  "error": {
    "code": "error-code-string",
    "message": "human-readable-message",
    "details": {
      "field": "field-name",
      "reason": "validation-error-reason"
    },
    "retry_after": "ISO-8601-timestamp",
    "support_id": "uuid"
  }
}
```

### 5.2 Common Error Codes

- `INVALID_MODEL_FORMAT`: Model data doesn't match specification
- `CHECKSUM_MISMATCH`: Downloaded model checksum doesn't match
- `ROUND_EXPIRED`: Training deadline passed
- `INSUFFICIENT_RESOURCES`: Client doesn't meet resource requirements
- `PRIVACY_BUDGET_EXCEEDED`: Epsilon budget exhausted
- `SIGNATURE_INVALID`: Cryptographic signature verification failed

## 6. Versioning

### 6.1 Message Version Header

All messages MUST include a version header:

```json
{
  "spec_version": "1.0.0",
  "message_type": "...",
  ...
}
```

### 6.2 Compatibility Rules

- **Major version**: Breaking changes (incompatible)
- **Minor version**: Backward-compatible additions
- **Patch version**: Backward-compatible fixes

Clients MUST reject messages with unsupported major versions.

## 7. Security Requirements

### 7.1 Transport Security

- All communication MUST use TLS 1.3 or higher
- Certificate pinning RECOMMENDED for mobile clients
- Perfect forward secrecy REQUIRED

### 7.2 Message Authentication

- All client updates MUST include cryptographic signatures
- Server responses SHOULD be signed
- Signature algorithm: Ed25519 or ECDSA (P-256)

## 8. Compliance

This data format specification enables compliance with:

- **GDPR**: Minimal data collection, client pseudonymization
- **HIPAA**: No raw data transmission, audit trails
- **CCPA**: User data control, deletion support

## Appendix A: Complete Example

See [examples/federated-learning-data-format.json](../examples/) for a complete working example.

## Appendix B: Schema Validation

JSON Schema files available at:
- Model format: `schemas/model.schema.json`
- Update format: `schemas/update.schema.json`
- Messages: `schemas/messages.schema.json`

---

**Copyright © 2025 SmileStory Inc. / World Certification Industry Association**
## Reference Standards Alignment

The Phase 1 data formats are layered above well-established machine-learning, cryptographic, and IT primitives so that conformant implementations interoperate without vendor lock-in.

| Concern | Reference |
|---------|-----------|
| Tensor and model format | ONNX (Open Neural Network Exchange) v1.14+; ISO/IEC JTC 1/SC 42 working drafts |
| Protocol Buffers | Google Protocol Buffers Language Specification proto3 |
| Apache Arrow | Apache Arrow Columnar Format Specification 1.0 |
| Numeric encoding | IEEE 754-2019 (binary32, binary64, bfloat16 informative) |
| Integer types | ISO/IEC 9899:2018 (C18) `<stdint.h>` semantics |
| Hash | FIPS 180-4 (SHA-2), FIPS 202 (SHA-3) |
| JSON | RFC 8259 |
| CBOR | RFC 8949 |
| Time encoding | ISO 8601:2019 |
| Geographic metadata | ISO 19115-1:2014 (when applicable) |
| Privacy framework | ISO/IEC 29100:2011 |
| Differential privacy | ISO/IEC 27559:2022 (privacy-preserving data publishing) |
| Information security | ISO/IEC 27001:2022 |

All references conform to the WIA Citation & Veracity Policy v1.0 §2.1 ALLOW.

## Conformance

A Phase 1 implementation is conformant when:

1. Model and tensor encodings follow ONNX v1.14+ or a documented profile derived from it.
2. Numeric encodings match IEEE 754-2019 with explicit precision declarations.
3. Hashes use a FIPS-approved algorithm and are declared in the manifest.
4. Time fields conform to ISO 8601:2019.

## Implementation Appendix

### A. Tensor canonicalisation

When tensors are signed (for example to attest a model checkpoint), the canonicalisation procedure is:

1. Convert the tensor to its canonical numeric type (defaults: float32 for parameters, int64 for indices, int8 for quantised payloads).
2. Serialise in row-major order with little-endian byte ordering.
3. Compute the hash over the serialised bytes plus a SHA-256 of the tensor's shape descriptor.
4. Record the hash, shape, and dtype in the manifest.

Canonicalisation is deterministic so any conformant implementation produces the same hash for the same logical tensor.

### B. Differential-privacy budget accounting

When central or local differential privacy is in use, the Phase 1 manifest carries the per-round and cumulative privacy budget (epsilon, delta) so that downstream consumers can audit cumulative privacy spend over the lifetime of the model.

The accounting follows the moments-accountant convention with explicit composition theorem applied at each round; the conventions, theorems, and parameters are recorded by their canonical names and parameter values rather than by external citation.

### C. Bias and fairness metadata

When the deployment requires bias and fairness reporting (for example under the EU AI Act or under the IEEE 7003-2024 algorithmic-bias standard), the manifest carries an aggregate fairness-metrics block scoped to the cohort segments declared in the deployment's risk-management plan. Aggregate statistics avoid disclosing individual data-subject information.

### D. Ecosystem hooks

The data format is designed to be consumed by widely deployed ML toolchains without bespoke conversion. Reference adapters exist for PyTorch, TensorFlow, JAX, scikit-learn, and ONNX Runtime; adapters for emerging frameworks follow the same conformance test plan.

---

**弘益人間 (Hongik Ingan) · Benefit All Humanity**

**License:** CC BY 4.0
