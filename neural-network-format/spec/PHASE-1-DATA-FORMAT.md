# WIA-AI-014: Neural Network Format - Phase 1 Data Format Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-01-15
**Philosophy:** 弘益人間 (Hongik Ingan) - Benefit All Humanity

## 1. Introduction

This specification defines the data format requirements for WIA-AI-014 Neural Network Format standard, enabling universal model interchange and deployment across frameworks and platforms.

### 1.1 Scope

This phase covers:
- Binary serialization format
- Tensor representation
- Graph structure encoding
- Metadata schema
- Compression and optimization

### 1.2 Goals

- **Interoperability:** Seamless conversion between major ML frameworks
- **Efficiency:** Optimized storage and loading performance
- **Completeness:** Preserve all necessary model information
- **Extensibility:** Support future architectures and operations

## 2. File Format Structure

### 2.1 Container Format

WIA-AI-014 uses a hierarchical container structure:

```
WIA-AI-014 Model Container
├── Header (256 bytes)
├── Metadata Block
├── Graph Definition Block
├── Tensor Data Block
└── Signature Block
```

### 2.2 Header Format

```c
struct WIAHeader {
    uint32_t magic;           // 0x57494131 ('WIA1')
    uint16_t version_major;   // 1
    uint16_t version_minor;   // 0
    uint64_t metadata_offset;
    uint64_t metadata_size;
    uint64_t graph_offset;
    uint64_t graph_size;
    uint64_t tensor_offset;
    uint64_t tensor_size;
    uint64_t signature_offset;
    uint64_t signature_size;
    uint8_t  compression;     // 0=none, 1=gzip, 2=zstd
    uint8_t  encryption;      // 0=none, 1=AES-256
    uint8_t  reserved[214];
};
```

## 3. Tensor Representation

### 3.1 Tensor Data Types

Supported data types with enum values:

| Type | Code | Bytes | Description |
|------|------|-------|-------------|
| FLOAT32 | 1 | 4 | IEEE 754 single precision |
| FLOAT16 | 2 | 2 | IEEE 754 half precision |
| BFLOAT16 | 3 | 2 | Brain floating point |
| INT64 | 4 | 8 | 64-bit signed integer |
| INT32 | 5 | 4 | 32-bit signed integer |
| INT16 | 6 | 2 | 16-bit signed integer |
| INT8 | 7 | 1 | 8-bit signed integer |
| UINT8 | 8 | 1 | 8-bit unsigned integer |
| BOOL | 9 | 1 | Boolean (0 or 1) |
| COMPLEX64 | 10 | 8 | Complex number (2x float32) |

### 3.2 Tensor Structure

```protobuf
message Tensor {
    string name = 1;
    repeated int64 dims = 2;
    DataType dtype = 3;

    oneof data {
        bytes raw_data = 4;
        ExternalData external_data = 5;
    }

    TensorMetadata metadata = 6;
}

message ExternalData {
    string location = 1;
    int64 offset = 2;
    int64 length = 3;
    string checksum = 4;  // SHA-256
}

message TensorMetadata {
    string doc_string = 1;
    repeated KeyValue properties = 2;
    QuantizationParams quantization = 3;
}
```

### 3.3 Tensor Layout

Tensors use row-major (C-style) layout by default:

```python
# For tensor with shape [N, C, H, W]
# Index calculation: index = n*C*H*W + c*H*W + h*W + w

# Memory layout:
[
  N0C0H0W0, N0C0H0W1, ..., N0C0H0Wn,
  N0C0H1W0, N0C0H1W1, ..., N0C0H1Wn,
  ...
]
```

## 4. Graph Representation

### 4.1 Computational Graph Structure

```protobuf
message Graph {
    string name = 1;
    repeated Node nodes = 2;
    repeated ValueInfo inputs = 3;
    repeated ValueInfo outputs = 4;
    repeated Tensor initializers = 5;
    GraphMetadata metadata = 6;
}

message Node {
    string name = 1;
    string op_type = 2;
    repeated string inputs = 3;
    repeated string outputs = 4;
    repeated Attribute attributes = 5;
    string doc_string = 6;
}

message ValueInfo {
    string name = 1;
    TypeProto type = 2;
    string doc_string = 3;
}
```

### 4.2 Operator Definitions

Operators follow ONNX operator set conventions:

```yaml
operators:
  - name: Conv
    domain: ""
    since_version: 1
    inputs:
      - name: X
        type: T
      - name: W
        type: T
      - name: B
        type: T
        optional: true
    outputs:
      - name: Y
        type: T
    attributes:
      - name: kernel_shape
        type: ints
        required: true
      - name: strides
        type: ints
        default: [1, 1]
```

## 5. Metadata Schema

### 5.1 Model Metadata

```json
{
  "model_info": {
    "name": "string",
    "version": "semver",
    "description": "string",
    "author": "string",
    "license": "string",
    "created_at": "iso8601",
    "framework": "string",
    "framework_version": "string"
  },
  "training_info": {
    "dataset": "string",
    "hyperparameters": {},
    "metrics": {}
  },
  "deployment_info": {
    "target_platforms": ["string"],
    "input_preprocessing": {},
    "output_postprocessing": {}
  }
}
```

## 6. Compression and Optimization

### 6.1 Supported Compression

- **None (0):** No compression
- **GZIP (1):** Standard gzip compression
- **ZSTD (2):** Zstandard compression (recommended)

### 6.2 Tensor Compression

Individual tensors may be compressed:

```protobuf
message CompressedTensor {
    Tensor tensor = 1;
    CompressionType compression = 2;
    bytes compressed_data = 3;
    int64 uncompressed_size = 4;
}
```

## 7. Validation and Integrity

### 7.1 Checksums

Each major block includes SHA-256 checksum:

```protobuf
message BlockChecksum {
    string block_type = 1;
    bytes sha256 = 2;
}
```

### 7.2 Model Signature

Optional cryptographic signature:

```protobuf
message Signature {
    string algorithm = 1;  // "RSA-SHA256", "ECDSA"
    bytes public_key = 2;
    bytes signature = 3;
    int64 timestamp = 4;
}
```

## 8. Quantization Support

### 8.1 Quantization Parameters

```protobuf
message QuantizationParams {
    QuantizationType type = 1;  // LINEAR, LOGARITHMIC
    float scale = 2;
    int32 zero_point = 3;
    DataType quantized_type = 4;
}

enum QuantizationType {
    NONE = 0;
    LINEAR = 1;
    LOGARITHMIC = 2;
}
```

### 8.2 Per-Channel Quantization

```protobuf
message PerChannelQuantization {
    repeated float scales = 1;
    repeated int32 zero_points = 2;
    int32 channel_dim = 3;
}
```

## 9. External Data References

### 9.1 Large Tensor Storage

For models >2GB, tensors can be stored externally:

```yaml
model_structure:
  - model.wia:
      - header
      - metadata
      - graph_definition
  - weights/:
      - layer1.bin
      - layer2.bin
      - layer3.bin
```

### 9.2 Reference Format

```json
{
  "external_data": {
    "location": "weights/layer1.bin",
    "offset": 0,
    "length": 1048576,
    "checksum": "sha256:abc123..."
  }
}
```

## 10. Versioning and Compatibility

### 10.1 Version Numbering

- **Major version:** Breaking changes to format structure
- **Minor version:** Backward-compatible additions
- **Patch version:** Bug fixes and clarifications

### 10.2 Compatibility Rules

- Readers MUST support all minor versions within same major version
- Readers MAY support older major versions
- Writers SHOULD write latest stable version
- Unknown extensions MUST be ignored (forward compatibility)

## 11. Example: Complete Model

```python
# Pseudo-code for WIA-AI-014 model structure
model = WIAModel(
    header=WIAHeader(
        magic=0x57494131,
        version=(1, 0),
        compression=ZSTD
    ),
    metadata=ModelMetadata(
        name="ResNet50",
        version="2.1.0",
        framework="pytorch",
        created_at="2025-01-15T10:00:00Z"
    ),
    graph=Graph(
        name="main",
        nodes=[
            Node(name="conv1", op_type="Conv", ...),
            Node(name="relu1", op_type="Relu", ...)
        ],
        inputs=[ValueInfo(name="input", type=TensorType(...))],
        outputs=[ValueInfo(name="output", type=TensorType(...))]
    ),
    tensors={
        "conv1.weight": Tensor(dims=[64,3,7,7], dtype=FLOAT32, ...),
        "conv1.bias": Tensor(dims=[64], dtype=FLOAT32, ...)
    }
)
```

## 12. Compliance Requirements

### 12.1 Minimum Requirements

A WIA-AI-014 compliant implementation MUST:
- Support all data types in Section 3.1
- Implement tensor serialization per Section 3.2
- Support graph representation per Section 4.1
- Validate checksums per Section 7.1

### 12.2 Optional Features

Implementations MAY support:
- Encryption (Section 2.2)
- Compression beyond GZIP (Section 6.1)
- External tensor storage (Section 9)
- Cryptographic signatures (Section 7.2)

## 13. References

- ONNX Format Specification: https://github.com/onnx/onnx/blob/main/docs/IR.md
- Protocol Buffers: https://developers.google.com/protocol-buffers
- IEEE 754 Floating Point: https://ieeexplore.ieee.org/document/8766229

---

**Copyright © 2025 WIA (World Certification Industry Association)**
**弘益人間 (Hongik Ingan) · Benefit All Humanity**
