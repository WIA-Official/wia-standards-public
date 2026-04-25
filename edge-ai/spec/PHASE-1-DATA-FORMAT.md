# WIA-AI-019 PHASE 1: Data Format Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-12-25

## Overview

This specification defines standard data formats for edge AI model deployment, inference inputs/outputs, and telemetry data. Standardized formats enable interoperability across frameworks, hardware platforms, and deployment environments.

## Model Package Format

### Model Metadata

Every edge AI model MUST include metadata in JSON format:

```json
{
  "modelId": "unique-model-identifier",
  "version": "1.2.3",
  "framework": "tflite|coreml|onnx|pytorch-mobile",
  "created": "2025-12-25T10:00:00Z",
  "architecture": {
    "type": "cnn|transformer|rnn|hybrid",
    "inputShape": [1, 224, 224, 3],
    "outputShape": [1, 1000],
    "parameters": 5200000,
    "layers": 50
  },
  "optimization": {
    "quantization": "int8|fp16|fp32",
    "pruning": 0.5,
    "compression": "gzip|none"
  },
  "hardware": {
    "minRAM": "256MB",
    "recommendedAccelerator": "npu|gpu|cpu",
    "platforms": ["ios", "android", "linux-arm"]
  },
  "performance": {
    "avgLatency": "25ms",
    "throughput": "40fps",
    "accuracy": 0.923
  },
  "license": "MIT|Apache-2.0|Proprietary",
  "author": "Organization Name",
  "checksum": "sha256:abc123..."
}
```

### Model File Structure

```
model_package.zip
├── metadata.json (required)
├── model.tflite|model.mlmodel|model.onnx (required)
├── labels.txt (optional, for classification)
├── README.md (optional)
├── LICENSE (required)
└── signature.sig (optional, for verification)
```

## Inference Data Format

### Input Tensor Format

```json
{
  "tensors": [
    {
      "name": "input_0",
      "shape": [1, 224, 224, 3],
      "dtype": "float32|uint8|int8",
      "data": "<base64_encoded_data>",
      "preprocessing": {
        "normalization": "[-1, 1]|[0, 1]",
        "mean": [0.485, 0.456, 0.406],
        "std": [0.229, 0.224, 0.225]
      }
    }
  ],
  "metadata": {
    "timestamp": "2025-12-25T10:00:00Z",
    "deviceId": "device-12345",
    "sessionId": "session-abc"
  }
}
```

### Output Tensor Format

```json
{
  "tensors": [
    {
      "name": "output_0",
      "shape": [1, 1000],
      "dtype": "float32",
      "data": "<base64_encoded_data>"
    }
  ],
  "predictions": [
    {
      "class": "cat",
      "confidence": 0.95,
      "index": 281
    },
    {
      "class": "dog",
      "confidence": 0.03,
      "index": 152
    }
  ],
  "metadata": {
    "inferenceTime": "25ms",
    "modelVersion": "1.2.3",
    "accelerator": "npu"
  }
}
```

## Telemetry Data Format

### Inference Telemetry

```json
{
  "event": "inference",
  "timestamp": "2025-12-25T10:00:00.123Z",
  "modelId": "model-identifier",
  "modelVersion": "1.2.3",
  "deviceInfo": {
    "deviceId": "device-12345",
    "platform": "ios",
    "osVersion": "17.2",
    "hardwareModel": "iPhone15,2",
    "accelerator": "neural_engine"
  },
  "performance": {
    "latency": 25.3,
    "confidence": 0.95,
    "memoryUsed": "45MB",
    "powerDraw": "250mW"
  },
  "result": {
    "success": true,
    "topClass": "cat",
    "topConfidence": 0.95
  }
}
```

### Model Update Telemetry

```json
{
  "event": "model_update",
  "timestamp": "2025-12-25T10:00:00Z",
  "oldVersion": "1.2.2",
  "newVersion": "1.2.3",
  "updateSize": "25MB",
  "updateDuration": "15s",
  "success": true,
  "rollback": false
}
```

## Privacy and Security

### Data Anonymization

All telemetry data MUST be anonymized:
- Device IDs SHOULD be hashed or pseudonymized
- Personal identifiable information (PII) MUST NOT be included
- Input/output data SHOULD NOT be transmitted unless explicitly required and consented

### Encryption

Sensitive data in transit MUST use:
- TLS 1.3 or higher for network transmission
- AES-256-GCM for data at rest
- SHA-256 for checksums and integrity verification

## Compatibility

This specification is compatible with:
- TensorFlow Lite 2.x
- Core ML 5.0+
- ONNX Runtime 1.10+
- PyTorch Mobile 1.12+

## Versioning

This specification follows Semantic Versioning (SemVer):
- MAJOR version for incompatible API changes
- MINOR version for backwards-compatible functionality additions
- PATCH version for backwards-compatible bug fixes

---



## Reference Standards Alignment

The Phase 1 data formats are layered above well-established machine-learning, IoT, and IT primitives so that conformant implementations interoperate without vendor lock-in.

| Concern | Reference |
|---------|-----------|
| Tensor and model format | ONNX (Open Neural Network Exchange) v1.14+ |
| Quantised model format | TFLite reference (Google Open Source) |
| Tensor metadata | Apache Arrow Columnar Format Specification 1.0 |
| Numeric encoding | IEEE 754-2019 (binary32, binary64) |
| Reduced-precision encoding | IEEE 754-2019 binary16; bfloat16 informative |
| Integer types | ISO/IEC 9899:2018 (C18) `<stdint.h>` semantics |
| Quantisation | INT8, INT16 per the deploying runtime's documented profile |
| Hash | FIPS 180-4 (SHA-2), FIPS 202 (SHA-3) |
| JSON | RFC 8259 |
| CBOR | RFC 8949 |
| Time encoding | ISO 8601:2019 |
| Sensor-data ontology | W3C SOSA/SSN |
| Industrial protocols | IEC 62541 (OPC UA), Modbus Application Protocol Specification V1.1b3 |
| IoT messaging | OASIS MQTT 5.0 |
| Real-time clocks | IEEE 1588-2019 (PTPv2), RFC 5905 (NTPv4) |

All references conform to the WIA Citation & Veracity Policy v1.0 §2.1 ALLOW.

## Conformance

A Phase 1 implementation is conformant when:

1. Model and tensor encodings follow ONNX v1.14+ or a documented profile derived from it.
2. Numeric encodings match IEEE 754-2019 with explicit precision declarations.
3. Quantisation parameters are declared in the manifest when reduced-precision arithmetic is used.
4. Hashes use a FIPS-approved algorithm and are declared in the manifest.
5. Time fields conform to ISO 8601:2019.

## Implementation Appendix

### A. Tensor canonicalisation

When tensors are signed (for example to attest a model checkpoint), the canonicalisation procedure is:

1. Convert the tensor to its canonical numeric type (defaults: float32 for parameters, int8 for quantised payloads).
2. Serialise in row-major order with little-endian byte ordering.
3. Compute the hash over the serialised bytes plus a SHA-256 of the tensor's shape descriptor and the quantisation parameters.
4. Record the hash, shape, dtype, and quantisation parameters in the manifest.

Canonicalisation is deterministic so any conformant implementation produces the same hash for the same logical tensor.

### B. Edge inference profile

Edge deployments declare a runtime profile that constrains the legal subset of operators and quantisation modes. The reference profiles are:

- **EDGE-MIN** — INT8 quantisation, fixed graph topology, no dynamic shapes, no control flow operators. Suitable for microcontroller-class targets.
- **EDGE-STD** — Mixed INT8 / FP16 precision, limited dynamic shapes, simple control flow (If, Loop). Suitable for mobile-class targets.
- **EDGE-MAX** — Full ONNX operator set with hardware-accelerator-specific extensions. Suitable for high-end edge gateways and accelerator-equipped servers.

The profile is declared in the manifest so downstream consumers can decide whether to deploy or to fall back.

### C. Quantisation Parameters

When reduced-precision quantisation is in use, the manifest carries the quantisation parameters per tensor:

- **dtype** — the quantised numeric type (INT8, INT16).
- **scale** — the scale factor relating quantised integers to floating-point values (IEEE 754-2019 binary32 by default).
- **zero_point** — the integer zero-point offset.
- **per_channel** — boolean indicating per-channel vs. per-tensor quantisation.
- **axis** — the channel axis when per-channel quantisation is used.

Quantisation parameters are signed alongside the tensor bytes so that any change to the parameters invalidates the signature and prevents silent precision drift.

### D. Operator Set Pinning

ONNX opset versions are pinned in the manifest. Implementations MUST refuse to silently substitute a different opset version, since opset upgrades may change operator semantics in ways that affect numerical outputs. Backward compatibility is preserved by allowing producers and consumers to negotiate opset version through capability discovery prior to inference.

### E. Reference Test Vectors

The conformance test suite provides reference test vectors covering:

- Single-operator correctness against published ONNX reference outputs.
- Cross-implementation consistency between two conformant runtimes.
- Quantisation round-trip preservation within the documented tolerance.
- Opset-version interoperability within the supported version range.

Implementations report their test-vector pass rate as part of conformance documentation.

### F. Hardware-Accelerator Profiles

When hardware-accelerator-specific operators are used, the manifest carries:

- **accelerator_family** — the family identifier (e.g., NVIDIA Jetson, Google Coral, Intel Movidius).
- **accelerator_capabilities** — a structured list of capability identifiers.
- **fallback_path** — the operator path to use on hardware lacking the declared capabilities.

Fallback paths preserve correctness on heterogeneous fleets where some devices have accelerators and others do not.

### G. Compression and Optimisation

Edge models are typically compressed and optimised for deployment. The manifest records the optimisation pipeline:

- **pruning** — sparsity ratio, sparsity pattern (unstructured / 2:4 / block-sparse).
- **quantisation** — as documented in §C above.
- **distillation** — teacher model identifier and distillation loss configuration.
- **fusion** — operator-fusion passes applied (Conv-BN-ReLU fusion, attention fusion, etc.).
- **graph_simplification** — algebraic simplification passes (constant folding, common-subexpression elimination).

Optimisation is reproducible: applying the same pipeline to the same input model produces the same optimised model, modulo declared non-deterministic optimisations.

### H. Provenance Linkage

Each model artefact in the manifest carries a provenance link to its origin: the training-data identifier (or a hash of the training-data manifest), the training-script identifier, the training-run identifier, and the responsible-party identifier. Provenance follows the W3C PROV-O ontology so downstream consumers can answer the basic governance questions about what produced this model, with what data, and under whose responsibility.

### I. Model Card

A machine-readable model card accompanies each released model. The card carries the intended use, training-data summary, evaluation summary, ethical-use considerations, known limitations, and the responsible party. Model cards are versioned with the model and travel with the manifest. The reference card schema is informative; the operating organisation may extend it with deployment-specific fields without breaking conformance.

### J. Conformance Reporting

Operating organisations publish a conformance report for each released model that lists the §Reference Standards alignment status, the supported runtime profile, the supported hardware target classes, the conformance test-vector pass rate, and the responsible party.

---

**Copyright © 2025 World Certification Industry Association (WIA)**
**License:** CC BY 4.0
**弘益人間** - Benefit All Humanity
