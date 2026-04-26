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

## 13. Operator Set Manifest

The graph IR is parameterized by an opset version that pins the semantics of every operator referenced. Models MUST embed:

```json
{
  "opsets": [
    {"domain": "ai.wia", "version": 14},
    {"domain": "ai.onnx", "version": 18},
    {"domain": "ai.onnx.ml", "version": 3}
  ],
  "extensions": ["wia.fp8", "wia.dynamic_shapes"]
}
```

Loaders MUST refuse models that reference an opset version they do not implement. Mixing operators from incompatible opset versions in the same graph is forbidden.

## 14. Graph Constraints

| Rule | Required |
|------|----------|
| Graph MUST be a directed acyclic graph (DAG) | Yes |
| Cycles in tensor dependencies | Forbidden |
| Constant initializers MUST be declared in the `initializers` section | Yes |
| Dynamic shapes MUST be expressed via symbolic dimensions, not -1 | Yes |
| Custom ops MUST live under a non-empty `domain` string | Yes |
| External weight references MUST use the `external_data` block per §9 | Yes |
| Subgraphs (loops, branches) MUST inherit parent opset versions | Yes |

A reference graph validator implementing the rules above is published at `cli/wia-ai-014-validate.sh`.

## 15. Sample Round-Trip

```python
from wia_ai_014 import WIAModel, Graph, Node, ValueInfo, TensorType, DataType

m = WIAModel(graph=Graph(
    name="add_one",
    nodes=[Node(name="add", op_type="Add", inputs=["x","one"], outputs=["y"])],
    inputs=[ValueInfo("x", TensorType(DataType.FLOAT32, [None, 4]))],
    outputs=[ValueInfo("y", TensorType(DataType.FLOAT32, [None, 4]))],
    initializers={"one": [1.0,1.0,1.0,1.0]},
))
m.save("add_one.wia")
assert WIAModel.load("add_one.wia").graph.nodes[0].op_type == "Add"
```

## 16. Tensor Layout & Strides

Tensor data is stored contiguous in row-major (NCHW for 4-D feature maps) by default. When the producing framework natively prefers another layout the tensor MUST carry an explicit `layout` attribute.

| Layout | Example shape | Notes |
|--------|---------------|-------|
| `row-major` (default) | `[N, C, H, W]` | most common |
| `channels-last` | `[N, H, W, C]` | TF / TFLite / mobile |
| `nd-hwc` | `[N, D, H, W, C]` | volumetric |
| `packed-int8` | `[N, C/4, H, W, 4]` | int8 inference kernels |

Strides MAY be omitted when the layout is contiguous; otherwise stride-per-axis MUST be present:

```json
{
  "name": "conv1.weight",
  "dims": [64, 3, 7, 7],
  "dtype": "FLOAT32",
  "layout": "row-major",
  "strides": [147, 49, 7, 1],
  "elementSize": 4
}
```

## 17. Constant Folding & Initializer Promotion

The format permits two physical homes for constants:

1. **Inline initializers**: small constants (≤ 1 MiB) embedded in the graph proto.
2. **External initializers**: large constants stored in companion files referenced by `external_data` per §9.

Loaders MUST treat both forms identically at the API surface; the difference is only physical layout.

```json
{
  "name": "embedding.weight",
  "dims": [50257, 768],
  "dtype": "FLOAT16",
  "external_data": {
    "location": "weights/embedding.bin",
    "offset": 0,
    "length": 77194752,
    "checksum": "sha256:9c1c..."
  }
}
```

## 18. Quantization Metadata Block

Every quantized model MUST carry a quantization manifest at the top of the model proto:

```json
{
  "scheme": "post-training-static",
  "calibrationDataset": "wia-internal://calib-2024-Q4",
  "calibrationSamples": 256,
  "perChannel": true,
  "axis": 0,
  "activationDtype": "INT8",
  "weightDtype": "INT8",
  "biasDtype": "INT32",
  "outlierClipping": "percentile-99.99"
}
```

QAT (quantization-aware training) and FP8 schemes MUST also include a `range` array (min/max per channel) so that runtime kernels can re-scale without re-running calibration.

- ONNX Format Specification: https://github.com/onnx/onnx/blob/main/docs/IR.md
- Protocol Buffers: https://developers.google.com/protocol-buffers
- IEEE 754 Floating Point: https://ieeexplore.ieee.org/document/8766229

---

**Copyright © 2025 WIA (World Certification Industry Association)**
**弘益人間 (Hongik Ingan) · Benefit All Humanity**
