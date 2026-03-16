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

**Copyright © 2025 World Certification Industry Association (WIA)**
**License:** CC BY 4.0
**弘益人間** - Benefit All Humanity
