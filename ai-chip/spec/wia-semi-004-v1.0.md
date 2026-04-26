# WIA-SEMI-004: AI Chip Standard Specification v1.0

**Status**: Approved  
**Date**: 2025-01-01  
**Maintainer**: WIA Standards Committee  
**License**: CC BY-SA 4.0

---

## Abstract

This specification defines the WIA-SEMI-004 standard for AI chip design, performance benchmarking, deployment protocols, and interoperability requirements. The standard covers Neural Processing Units (NPUs), Tensor Processing Units (TPUs), Graphics Processing Units (GPUs) for AI, and custom AI accelerators.

## Scope

This standard applies to:
- AI chip manufacturers
- System integrators  
- Software developers optimizing for AI hardware
- Cloud providers deploying AI infrastructure
- Edge device manufacturers

## Normative References

- IEEE 754-2019: Floating-Point Arithmetic
- MLPerf v3.0: Machine Learning Performance Benchmark
- ONNX 1.14: Open Neural Network Exchange
- PCIe 6.0: Peripheral Component Interconnect Express
- JEDEC HBM3: High Bandwidth Memory specification

---

## Section 1: Performance Metrics

### 1.1 TOPS (Tera Operations Per Second)

**Definition**: TOPS measures the theoretical peak compute performance of an AI chip.

**Measurement**: 
```
TOPS = (Number of MACs) × (Clock Frequency in GHz)

Example:
Chip with 8192 INT8 MAC units at 1.5 GHz:
TOPS = 8192 × 1.5 = 12.288 TOPS (INT8)
```

**Requirements**:
- MUST specify precision (INT4, INT8, FP16, BF16, FP32)
- MUST report both peak and sustained TOPS
- SHOULD measure utilization under real workloads

### 1.2 TOPS/W (Power Efficiency)

**Definition**: Energy efficiency measured in tera operations per watt.

**Measurement**:
```
TOPS/W = TOPS / TDP (Watts)

Example:
35 TOPS at 6W TDP:
TOPS/W = 35 / 6 = 5.83 TOPS/W
```

**Requirements**:
- MUST measure at TDP (Thermal Design Power)
- SHOULD report at different power states
- MUST include all chip power (compute + memory + I/O)

### 1.3 Memory Bandwidth

**Definition**: Rate of data transfer between compute units and memory.

**Requirements**:
- MUST report peak theoretical bandwidth (GB/s)
- SHOULD measure sustained bandwidth under AI workloads
- MUST specify memory type (GDDR6, HBM2e, HBM3, LPDDR5X)

**Calculation**:
```
Bandwidth = (Memory Clock) × (Bus Width) × (Data Rate) / 8

Example HBM3:
Clock: 6.4 Gbps per pin
512-bit interface per stack
5 stacks
Bandwidth = 6.4 × 512 × 5 / 8 = 2048 GB/s
```

### 1.4 Latency and Throughput

**Latency**: Time for single inference (milliseconds)
**Throughput**: Inferences per second

**Requirements**:
- MUST report p50, p95, p99 latency percentiles
- SHOULD measure with representative models
- MUST specify batch size and input dimensions

---

## Section 2: Architecture Requirements

### 2.1 Compute Units

**Mandatory**:
- Matrix multiplication acceleration (systolic array, tensor cores, or equivalent)
- Support for quantized precisions (INT8 minimum)
- Activation function hardware acceleration (ReLU, GELU, etc.)

**Optional**:
- Sparsity acceleration
- Custom operator support
- Reconfigurable logic

### 2.2 Memory Hierarchy

**Levels** (minimum 3 levels required):
1. **Registers/L1**: < 1 cycle latency, 16-128 KB
2. **L2 Cache**: 5-20 cycle latency, 256 KB - 8 MB
3. **Main Memory**: 100+ cycle latency, 4-192 GB

**Requirements**:
- L1 bandwidth MUST exceed 1 TB/s
- L2 bandwidth MUST exceed 100 GB/s
- Cache coherency protocol MUST be documented

### 2.3 Precision Support

**Mandatory Formats**:
- INT8: 8-bit integer quantization
- FP16: IEEE 754 half-precision floating-point

**Recommended Formats**:
- INT4: 4-bit integer quantization
- BF16: Brain Float (1-sign, 8-exponent, 7-mantissa)
- FP8: 8-bit floating-point (E4M3 and E5M2)

**Conversion Requirements**:
- MUST support runtime precision conversion
- SHOULD minimize accuracy loss during quantization
- MUST document quantization scheme

---

## Section 3: Benchmarking Protocol

### 3.1 Standard Benchmarks

**Computer Vision**:
- ResNet-50 (ImageNet, batch sizes: 1, 32, 256)
- MobileNet V2 (ImageNet, batch size: 1)
- YOLOv5 (COCO, batch size: 1)

**Natural Language Processing**:
- BERT-Base (SQuAD, sequence length: 384)
- GPT-2 (text generation, sequence length: 1024)

**Generative AI**:
- Stable Diffusion (512×512 image generation)

### 3.2 Reporting Format

```json
{
  "chip": "Vendor-Model-SKU",
  "benchmark": "ResNet-50",
  "batch_size": 32,
  "precision": "INT8",
  "throughput": 5000,
  "throughput_unit": "images/sec",
  "latency_p50": 6.4,
  "latency_p95": 7.2,
  "latency_p99": 8.1,
  "latency_unit": "ms",
  "power": 75,
  "power_unit": "watts",
  "date": "2025-01-01"
}
```

### 3.3 MLPerf Compliance

**Requirements**:
- SHOULD submit to MLPerf Inference v3.0+
- MUST follow MLPerf submission rules
- MAY optimize for MLPerf workloads but MUST document

---

## Section 4: Software Compatibility

### 4.1 Framework Support

**Tier 1 (Mandatory)**:
- TensorFlow Lite OR PyTorch Mobile
- ONNX Runtime

**Tier 2 (Recommended)**:
- TensorFlow (full)
- PyTorch (full)
- JAX

### 4.2 Model Formats

**Supported Formats** (minimum 2 required):
- ONNX (.onnx)
- TensorFlow SavedModel
- TorchScript (.pt)
- TensorFlow Lite (.tflite)

### 4.3 API Standards

**Hardware Abstraction Layer**:
```cpp
// Minimal HAL interface

class AIChipHAL {
public:
    // Initialization
    virtual Status initialize() = 0;
    virtual Status shutdown() = 0;
    
    // Model loading
    virtual Status loadModel(const std::string& path, ModelHandle* handle) = 0;
    virtual Status unloadModel(ModelHandle handle) = 0;
    
    // Inference
    virtual Status inference(
        ModelHandle handle,
        const Tensor* inputs,
        size_t num_inputs,
        Tensor* outputs,
        size_t num_outputs
    ) = 0;
    
    // Profiling
    virtual Status getPerformanceMetrics(PerformanceMetrics* metrics) = 0;
};
```

---

## Section 5: Quantization Standards

### 5.1 Symmetric Quantization

**Formula**:
```
quantized_value = round(real_value / scale)
dequantized_value = quantized_value * scale

scale = max(abs(min_value), abs(max_value)) / (2^(bits-1) - 1)
```

**Requirements**:
- MUST support symmetric INT8 quantization
- Scale factor MUST be per-tensor or per-channel
- SHOULD minimize quantization error

### 5.2 Asymmetric Quantization

**Formula**:
```
quantized_value = round(real_value / scale) + zero_point
dequantized_value = (quantized_value - zero_point) * scale

scale = (max_value - min_value) / (2^bits - 1)
zero_point = -round(min_value / scale)
```

**Requirements**:
- MAY support asymmetric quantization
- Zero point MUST be within quantized range
- MUST document calibration method

### 5.3 Accuracy Requirements

**Acceptable Degradation**:
- Post-training quantization (PTQ): < 3% accuracy loss
- Quantization-aware training (QAT): < 1% accuracy loss

**Validation**:
- MUST validate on standard benchmarks
- SHOULD provide quantization toolkit
- MUST document accuracy/performance trade-offs

---

## Section 6: Deployment Protocols

### 6.1 Model Compilation

**Compilation Pipeline**:
```
Input Model (ONNX/TF/PyTorch)
    ↓
[Graph Optimization]
  - Constant folding
  - Operator fusion
  - Dead code elimination
    ↓
[Quantization]
  - INT8/INT4 conversion
  - Calibration
    ↓
[Hardware Mapping]
  - Tiling
  - Memory allocation
  - Kernel selection
    ↓
Compiled Binary
```

**Requirements**:
- MUST provide compiler toolchain
- SHOULD optimize for target hardware
- MUST validate functional correctness

### 6.2 Runtime Optimizations

**Batching**:
- SHOULD support dynamic batching
- MUST handle variable sequence lengths (NLP)
- MAY implement continuous batching

**Caching**:
- SHOULD cache compiled models
- MAY cache KV pairs (transformers)
- MUST document cache invalidation policy

---

## Section 7: Interconnect Standards

### 7.1 Host Interface

**Supported Interfaces**:
- PCIe Gen 4.0 or higher (x16 minimum)
- CXL 2.0+ (optional for memory pooling)

**Requirements**:
- Bandwidth MUST exceed 64 GB/s
- Latency SHOULD be < 10 μs for simple operations
- MUST support peer-to-peer DMA

### 7.2 Multi-Chip Scaling

**Inter-Chip Links**:
- NVIDIA NVLink
- AMD Infinity Fabric
- Intel Xe Link
- Proprietary high-speed links

**Requirements**:
- Bandwidth MUST exceed 400 GB/s per link
- SHOULD support all-to-all topology
- MUST provide API for distributed inference

---

## Section 8: Power Management

### 8.1 Power States

**Defined States**:
- P0: Maximum performance (TDP)
- P1: Balanced (75% TDP)
- P2: Power saver (50% TDP)
- P3: Idle (<5W)

**Requirements**:
- MUST support at least 2 power states
- Transition time SHOULD be < 100ms
- MUST expose power control API

### 8.2 Thermal Management

**Thermal Limits**:
- Junction temperature MUST be specified
- Throttling policy MUST be documented
- SHOULD provide thermal sensors API

---

## Section 9: Security and Safety

### 9.1 Secure Boot

**Requirements**:
- SHOULD support signed firmware/models
- MAY implement secure enclave
- MUST prevent unauthorized access

### 9.2 Confidential Computing

**Optional Features**:
- Memory encryption
- Attestation support
- Isolated execution environments

### 9.3 Functional Safety (Automotive)

**For automotive AI chips**:
- MUST comply with ISO 26262 ASIL-B minimum
- SHOULD support redundancy/fault tolerance
- MUST provide diagnostic capabilities

---

## Section 10: Compliance and Certification

### 10.1 Self-Certification

Vendors MAY self-certify compliance by:
1. Implementing required features (Sections 1-9)
2. Passing benchmark suite
3. Publishing results per Section 3.2
4. Open-sourcing driver/SDK (recommended)

### 10.2 WIA Certification

**Process**:
1. Submit technical documentation
2. Third-party benchmark verification
3. Review by WIA committee
4. Certification valid for 2 years

**Benefits**:
- WIA-SEMI-004 Certified badge
- Listing on WIA website
- Interoperability guaranteed

---

## Appendix A: Example Implementation

```python
# Example Python SDK wrapper

from wia_semi_004 import AIChip

# Initialize
chip = AIChip()
chip.initialize()

# Load model
model = chip.load_model("model.onnx", precision="INT8")

# Prepare input
input_data = np.random.randn(1, 3, 224, 224).astype(np.float32)

# Run inference
output = model.inference(input_data)

# Get metrics
metrics = chip.get_performance_metrics()
print(f"Throughput: {metrics.throughput} inferences/sec")
print(f"Latency: {metrics.latency_p99} ms (p99)")
print(f"Power: {metrics.power} W")

# Cleanup
chip.shutdown()
```

## Appendix B: Revision History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2025-01-01 | Initial release |

---

*© 2025 SmileStory Inc. / WIA*  
*弘益人間 (홍익인간) · Benefit All Humanity*
