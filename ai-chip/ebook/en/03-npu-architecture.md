# Chapter 3: NPU (Neural Processing Unit) Architecture

## Specialized Processors for Neural Network Acceleration

Neural Processing Units (NPUs) represent a paradigm shift in AI hardware design. Unlike general-purpose GPUs that excel at parallel floating-point operations, NPUs are purpose-built for the specific computational patterns found in neural networks. This chapter explores NPU architecture, design principles, and implementation strategies.

---

## What is an NPU?

### Definition and Scope

A **Neural Processing Unit (NPU)** is a specialized processor designed specifically to accelerate neural network operations, particularly inference workloads. NPUs optimize for:

1. **Matrix multiplication** (the core operation in neural networks)
2. **Low-precision arithmetic** (INT8, INT4 quantization)
3. **Power efficiency** (TOPS/W optimization)
4. **Memory bandwidth utilization** (data reuse patterns)
5. **Deterministic latency** (real-time requirements)

### NPU vs GPU vs TPU vs CPU

Understanding the distinctions:

| Feature | CPU | GPU | TPU | NPU |
|---------|-----|-----|-----|-----|
| **Primary Use** | General computing | Graphics + AI | AI training/inference | AI inference |
| **Parallelism** | 8-64 cores | 1000s of cores | 1000s of matrix units | 100s-1000s of MAC units |
| **Precision** | FP64, INT64 | FP32, FP16 | BF16, FP16, INT8 | INT8, INT4, INT16 |
| **Power** | 65-350W | 200-700W | 200-450W | 2-30W |
| **Latency** | Microseconds | Milliseconds | Milliseconds | Microseconds |
| **Deployment** | Everywhere | Data center, PC | Data center | Edge, mobile |
| **Flexibility** | Highest | High | Medium | Lower |
| **Efficiency** | Low for AI | Medium | High | Highest |

**Key Insight**: NPUs sacrifice flexibility for extreme efficiency in specific workloads.

---

## NPU Architecture Fundamentals

### Core Design Principles

#### 1. Systolic Array Architecture

The systolic array is the heart of most NPU designs:

```
Systolic Array (8x8 example):

Input Data Flow (→)
↓
[PE][PE][PE][PE][PE][PE][PE][PE]   ← Weights
[PE][PE][PE][PE][PE][PE][PE][PE]
[PE][PE][PE][PE][PE][PE][PE][PE]
[PE][PE][PE][PE][PE][PE][PE][PE]
[PE][PE][PE][PE][PE][PE][PE][PE]
[PE][PE][PE][PE][PE][PE][PE][PE]
[PE][PE][PE][PE][PE][PE][PE][PE]
[PE][PE][PE][PE][PE][PE][PE][PE]
↓
Output (accumulates)

PE = Processing Element (MAC unit)
```

**How it works**:
1. Weights flow horizontally through PEs
2. Input data flows vertically through PEs
3. Each PE performs multiply-accumulate (MAC)
4. Results accumulate and flow downward
5. High data reuse minimizes memory access

**Advantages**:
- **Efficient Data Reuse**: Each weight/activation used multiple times
- **Regular Structure**: Easy to design and verify
- **Scalability**: Can scale to 256x256 or larger arrays
- **Predictable Performance**: Regular computation pattern

**Example**: Google TPU uses 256x256 systolic array = 65,536 MACs per cycle

#### 2. Memory Hierarchy Optimization

NPUs implement sophisticated memory hierarchies:

```
Memory Hierarchy (NPU):

┌─────────────────────────────────┐
│   External DRAM (GB scale)      │
│   Bandwidth: 50-200 GB/s        │
└────────────┬────────────────────┘
             │
┌────────────▼────────────────────┐
│   L3 Cache / Shared SRAM        │
│   Size: 4-32 MB                 │
│   Bandwidth: 500-2000 GB/s      │
└────────────┬────────────────────┘
             │
┌────────────▼────────────────────┐
│   L2 Cache (Per Cluster)        │
│   Size: 256-2048 KB             │
│   Bandwidth: 1-4 TB/s           │
└────────────┬────────────────────┘
             │
┌────────────▼────────────────────┐
│   L1 Cache / Register File      │
│   Size: 16-128 KB               │
│   Bandwidth: 5-20 TB/s          │
└────────────┬────────────────────┘
             │
     [Systolic Array]
```

**Design Goals**:
1. **Maximize Data Reuse**: Keep frequently accessed data in fast memory
2. **Minimize DRAM Access**: DRAM is power-expensive
3. **Prefetching**: Hide memory latency with predictive loading
4. **Tiling**: Break large operations into cache-friendly chunks

**Power Impact**:
- L1 access: 1 pJ (picojoule)
- L2 access: 5 pJ
- DRAM access: 100-200 pJ

**Example**: Keeping data in L1 reduces energy by 100-200x

#### 3. Quantization-Aware Design

NPUs are optimized for low-precision arithmetic:

**Precision Levels**:
- **INT4**: 4-bit integers (-8 to 7 or 0 to 15)
  - 4x memory reduction vs INT16
  - 4x bandwidth reduction
  - Suitable for many inference tasks

- **INT8**: 8-bit integers (-128 to 127 or 0 to 255)
  - Standard for production inference
  - <1% accuracy loss with proper quantization
  - 4x reduction vs FP32

- **INT16**: 16-bit integers
  - Higher precision when needed
  - 2x reduction vs FP32

**Hardware Implementation**:
```
INT8 MAC (Multiply-Accumulate) Unit:

Input A (INT8)  ─┐
                 ├─→ [Multiplier] ─→ [Accumulator] ─→ Output (INT32)
Input B (INT8)  ─┘                        ↑
                                          │
                                    Previous Sum
```

**Quantization Formula**:
```
Quantized_value = round(Real_value / Scale) + Zero_point

Where:
- Scale: Mapping from real to quantized range
- Zero_point: Offset for asymmetric ranges
```

**Example**:
```python
# Real value: -1.5 to 2.5
# Map to INT8: -128 to 127

Scale = (2.5 - (-1.5)) / (127 - (-128)) = 4.0 / 255 ≈ 0.0157
Zero_point = -128 - round(-1.5 / 0.0157) ≈ -32

# Quantize real value 1.2:
Quantized = round(1.2 / 0.0157) + (-32) = 76 - 32 = 44

# Dequantize back:
Real = (44 - (-32)) * 0.0157 = 76 * 0.0157 ≈ 1.19
```

**Benefits**:
- 4x smaller models (INT8 vs FP32)
- 4x less memory bandwidth required
- Faster computation (INT8 MACs are simpler)
- Lower power consumption

---

## NPU Architectural Components

### 1. Compute Cores

#### Processing Element (PE) Design

A typical NPU PE contains:

```
Processing Element (PE):

┌─────────────────────────────────┐
│  Input Buffers                  │
│  ┌───────┐      ┌───────┐       │
│  │ Act   │      │Weight │       │
│  └───┬───┘      └───┬───┘       │
│      │              │           │
│      └─────┬────────┘           │
│            ▼                    │
│      [Multiplier]               │
│            │                    │
│            ▼                    │
│      [Accumulator] ←─ Previous │
│            │                    │
│            ▼                    │
│      [Activation]               │
│      (ReLU, etc.)               │
│            │                    │
│            ▼                    │
│      Output Buffer              │
└─────────────────────────────────┘
```

**Specifications (typical)**:
- **Throughput**: 2-8 MACs per cycle
- **Precision**: INT4/INT8/INT16 support
- **Activation Functions**: Hardware ReLU, sigmoid, tanh
- **Accumulator Width**: 32-bit to prevent overflow
- **Area**: 0.01-0.05 mm² per PE (7nm process)

#### Multi-Core Architecture

NPUs organize PEs into hierarchical structures:

```
NPU Multi-Core Layout:

┌───────────────────────────────────────┐
│          NPU (System Level)           │
│                                       │
│  ┌─────────┐  ┌─────────┐            │
│  │ Cluster │  │ Cluster │            │
│  │    0    │  │    1    │  ...       │
│  │         │  │         │            │
│  │ ┌─────┐ │  │ ┌─────┐ │            │
│  │ │Core │ │  │ │Core │ │            │
│  │ │  0  │ │  │ │  0  │ │            │
│  │ └─────┘ │  │ └─────┘ │            │
│  │ ┌─────┐ │  │ ┌─────┐ │            │
│  │ │Core │ │  │ │Core │ │            │
│  │ │  1  │ │  │ │  1  │ │            │
│  │ └─────┘ │  │ └─────┘ │            │
│  └────┬────┘  └────┬────┘            │
│       └────────────┼─────────────┐   │
│                    ▼             │   │
│          [Interconnect Fabric]   │   │
│                    │             │   │
│                    ▼             │   │
│          [Shared Memory/Cache]   │   │
└───────────────────────────────────────┘
```

**Scalability Examples**:
- **Apple Neural Engine (A17 Pro)**: 16 cores, 35 TOPS
- **Qualcomm Hexagon NPU (SD 8 Gen 3)**: 73 TOPS
- **Google Edge TPU**: 4 TOPS at 2W
- **Intel Movidius Myriad X**: 1 TOPS at 1.5W

### 2. Data Movement Engine (DME)

The DME handles all memory transfers:

**Responsibilities**:
1. **DMA Operations**: Move data between memory tiers
2. **Prefetching**: Predict and load next data
3. **Format Conversion**: Handle different tensor layouts
4. **Compression/Decompression**: Reduce bandwidth usage

**Example DMA Engine**:
```
DMA Transfer Pipeline:

[DRAM] → [Decompressor] → [Format Converter] → [L2 Cache]
                                                     ↓
                                              [Prefetcher]
                                                     ↓
                                                [L1 Cache]
                                                     ↓
                                             [Compute Core]
```

**Performance Metrics**:
- **Bandwidth**: 50-500 GB/s depending on tier
- **Latency**: 10-100 cycles for L2, 100-1000 for DRAM
- **Outstanding Requests**: 32-256 concurrent transfers
- **Compression Ratio**: 2-4x for typical neural network weights

### 3. Control and Scheduling Unit

The control unit orchestrates NPU operations:

**Functions**:
1. **Instruction Decode**: Parse NPU commands
2. **Dataflow Scheduling**: Manage PE utilization
3. **Memory Arbitration**: Resolve access conflicts
4. **Power Management**: Dynamic voltage/frequency scaling

**Scheduling Strategies**:

#### Weight Stationary
```
Weights stay in PEs, activations stream through

Pros: Minimal weight reloading
Cons: Limited by PE memory capacity
Best for: Inference with small models
```

#### Output Stationary
```
Partial sums accumulate in PEs

Pros: Minimal output write-back
Cons: More complex accumulation logic
Best for: Large batch sizes
```

#### No Local Reuse (NLR)
```
All operands streamed from memory

Pros: Flexible, handles any operation
Cons: High memory bandwidth requirement
Best for: Diverse workloads
```

**Example**: Google TPU uses **weight stationary** dataflow.

---

## Real-World NPU Implementations

### Case Study 1: Apple Neural Engine

**Architecture Overview**:
- **Debut**: A11 Bionic (2017), 0.6 TOPS
- **Latest**: A17 Pro (2023), 35 TOPS
- **Cores**: 16-core design
- **Precision**: INT8, INT16
- **Integration**: Unified memory architecture with CPU/GPU

**Key Features**:
1. **Unified Memory**: NPU, CPU, GPU share memory pool
2. **Power Efficiency**: 1.5+ TOPS/W
3. **ML Framework Integration**: Core ML optimizations
4. **Privacy Focus**: All processing on-device

**Applications**:
- Face ID (facial recognition)
- Animoji (real-time 3D face tracking)
- Photography (Smart HDR, Deep Fusion)
- Siri (natural language processing)
- Live Text (OCR in camera)

**Performance Evolution**:
| Chip | Year | TOPS | Power | TOPS/W |
|------|------|------|-------|--------|
| A11  | 2017 | 0.6  | ~2W   | 0.3    |
| A12  | 2018 | 5.0  | ~3W   | 1.67   |
| A13  | 2019 | 6.0  | ~3W   | 2.0    |
| A14  | 2020 | 11.0 | ~4W   | 2.75   |
| A15  | 2021 | 15.8 | ~5W   | 3.16   |
| A16  | 2022 | 17.0 | ~5W   | 3.4    |
| A17  | 2023 | 35.0 | ~6W   | 5.8    |

**Innovation**: Apple's tight integration of hardware and software enables unprecedented efficiency.

### Case Study 2: Qualcomm Hexagon NPU

**Architecture Overview**:
- **Generation**: 7th gen Hexagon in Snapdragon 8 Gen 3
- **Performance**: 73 TOPS
- **Precision**: INT4, INT8, INT16, FP16
- **Unique Feature**: Micro Tile Inferencing

**Micro Tile Inferencing**:
```
Traditional Inference:
Load entire model → Process → Unload
Memory requirement: Full model size

Micro Tile:
Load tile 1 → Process → Load tile 2 → Process → ...
Memory requirement: One tile (10-20% of model)

Enables: Running 7B-13B LLMs on smartphone
```

**Architectural Components**:
1. **Scalar Unit**: Control flow and simple operations
2. **Vector Unit**: SIMD operations for activations
3. **Tensor Accelerator**: Matrix multiplication
4. **HMX (Hexagon Matrix eXtensions)**: Specialized for AI

**Applications**:
- On-device Stable Diffusion
- Real-time language translation
- Generative AI (LLaMA 2 7B on-device)
- Camera AI (multi-frame HDR)
- Voice assistants

**Software Stack**:
- **SNPE (Snapdragon Neural Processing Engine)**: Runtime
- **AIMET**: AI Model Efficiency Toolkit (quantization)
- **Qualcomm AI Hub**: Pre-optimized models

### Case Study 3: Google Edge TPU

**Architecture Overview**:
- **Target**: Edge inference devices
- **Performance**: 4 TOPS
- **Power**: 2W
- **Efficiency**: 2 TOPS/W
- **Form Factors**: M.2, USB, PCIe

**Design Philosophy**:
```
Cloud TPU: Maximum performance
Edge TPU: Maximum efficiency

Optimizations:
- INT8 only (no FP16)
- Quantization-aware training required
- Optimized for MobileNet, EfficientNet
- Small footprint (25mm²)
```

**Target Applications**:
- Smart cameras (Nest Cam)
- IoT devices
- Robotics (e.g., autonomous drones)
- Industrial automation
- Retail (smart checkout)

**Deployment Model**:
```python
# Edge TPU Deployment

1. Train model (Cloud)
2. Apply quantization-aware training
3. Convert to TensorFlow Lite
4. Compile for Edge TPU
   $ edgetpu_compiler model.tflite
5. Deploy to device

import tflite_runtime.interpreter as tflite

interpreter = tflite.Interpreter(
    model_path='model_edgetpu.tflite',
    experimental_delegates=[
        tflite.load_delegate('libedgetpu.so.1')
    ]
)
```

**Performance Benchmarks**:
| Model | Latency | Power | Throughput |
|-------|---------|-------|------------|
| MobileNet V2 | 1.2ms | 0.5W | 833 fps |
| Inception V3 | 3.5ms | 1.5W | 285 fps |
| ResNet-50 | 8.0ms | 2.0W | 125 fps |

---

## NPU Programming Models

### 1. High-Level Frameworks

Most NPUs are programmed through AI frameworks:

**TensorFlow Lite**:
```python
import tensorflow as tf

# Create model
model = tf.keras.Sequential([...])

# Convert to TFLite
converter = tf.lite.TFLiteConverter.from_keras_model(model)
converter.optimizations = [tf.lite.Optimize.DEFAULT]
converter.target_spec.supported_ops = [
    tf.lite.OpsSet.TFLITE_BUILTINS_INT8
]
tflite_model = converter.convert()

# Quantization
converter.representative_dataset = representative_data_gen

# Save
with open('model.tflite', 'wb') as f:
    f.write(tflite_model)
```

**PyTorch Mobile**:
```python
import torch

model = MyModel()
model.eval()

# Quantize
quantized_model = torch.quantization.quantize_dynamic(
    model, {torch.nn.Linear}, dtype=torch.qint8
)

# Script for mobile
scripted_model = torch.jit.script(quantized_model)
scripted_model.save("model.pt")
```

**ONNX Runtime**:
```python
import onnxruntime as ort

# Load model
session = ort.InferenceSession(
    "model.onnx",
    providers=['DmlExecutionProvider']  # DirectML for NPU
)

# Run inference
inputs = {session.get_inputs()[0].name: input_data}
outputs = session.run(None, inputs)
```

### 2. Compiler Infrastructure

NPU compilers optimize computation graphs:

**Compilation Pipeline**:
```
High-Level Model (TensorFlow, PyTorch)
           ↓
    [Graph Import]
           ↓
Intermediate Representation (IR)
           ↓
    [Optimization Passes]
    - Operator fusion
    - Constant folding
    - Dead code elimination
    - Quantization
           ↓
   [Hardware Mapping]
    - Tiling strategy
    - Memory allocation
    - Scheduling
           ↓
   NPU Binary/Bytecode
```

**Example Optimizations**:

1. **Operator Fusion**:
```
Before:
Conv → BatchNorm → ReLU → Add

After:
FusedConvBNReLUAdd (single kernel)

Benefit: 3x fewer memory accesses
```

2. **Quantization**:
```
Before (FP32):
Weights: 4 bytes × 1M parameters = 4 MB
Bandwidth: 4 MB per inference

After (INT8):
Weights: 1 byte × 1M parameters = 1 MB
Bandwidth: 1 MB per inference

Speedup: 4x less data movement
```

3. **Tiling**:
```
Large convolution (won't fit in cache):
Break into tiles that fit in L1/L2

Example:
Input: 512×512×64
Tile size: 64×64×64

Process 8×8 = 64 tiles sequentially
Each tile stays in fast memory
```

---

## Performance Optimization Techniques

### 1. Model Architecture Selection

Choose NPU-friendly architectures:

**Efficient Architectures**:
- **MobileNet V2/V3**: Depthwise separable convolutions
- **EfficientNet**: Compound scaling
- **SqueezeNet**: Fire modules
- **ShuffleNet**: Channel shuffle operations

**Architecture Comparison** (ImageNet, INT8):
| Model | Parameters | MACs | Top-1 Acc | NPU Latency |
|-------|------------|------|-----------|-------------|
| ResNet-50 | 25.6M | 4.1B | 76.2% | 25ms |
| MobileNet V2 | 3.5M | 300M | 72.0% | 3.5ms |
| EfficientNet-B0 | 5.3M | 390M | 77.1% | 4.2ms |

**Recommendation**: Use MobileNet or EfficientNet for edge NPUs.

### 2. Quantization Strategies

#### Post-Training Quantization (PTQ)
```python
import tensorflow as tf

converter = tf.lite.TFLiteConverter.from_saved_model('model')
converter.optimizations = [tf.lite.Optimize.DEFAULT]

# INT8 quantization
converter.representative_dataset = representative_dataset
converter.target_spec.supported_ops = [
    tf.lite.OpsSet.TFLITE_BUILTINS_INT8
]

tflite_model = converter.convert()
```

**Pros**: No retraining required
**Cons**: Some accuracy loss (0.5-3%)

#### Quantization-Aware Training (QAT)
```python
import tensorflow_model_optimization as tfmot

# Apply QAT
quantize_model = tfmot.quantization.keras.quantize_model

q_aware_model = quantize_model(model)

# Train
q_aware_model.compile(...)
q_aware_model.fit(...)

# Convert
converter = tf.lite.TFLiteConverter.from_keras_model(q_aware_model)
```

**Pros**: Minimal accuracy loss (<0.5%)
**Cons**: Requires retraining

#### Mixed Precision
```
Sensitive layers (first, last): INT16 or FP16
Middle layers: INT8
```

**Example**:
- First conv: INT16 (preserves input details)
- Hidden layers: INT8 (bulk of computation)
- Final FC: INT16 (preserves classification precision)

### 3. Memory Layout Optimization

**NCHW vs NHWC**:
```
NCHW (Batch, Channel, Height, Width):
[N=1, C=64, H=224, W=224]
Channel-contiguous

NHWC (Batch, Height, Width, Channel):
[N=1, H=224, W=224, C=64]
Spatial-contiguous

NPUs typically prefer NHWC for better cache utilization
```

**Weight Reordering**:
```
Original weights: [Out, In, H, W]
NPU optimized: [H, W, In_tile, Out_tile]

Enables efficient tile-based computation
```

---

## Power and Efficiency Analysis

### Power Breakdown (Typical NPU)

```
Total Power: 10W (example)

Compute (Systolic Array): 40% = 4W
Memory Access:
  - DRAM: 30% = 3W
  - Cache: 10% = 1W
Data Movement (DME): 10% = 1W
Control Logic: 5% = 0.5W
Leakage: 5% = 0.5W
```

**Optimization Priorities**:
1. **Reduce DRAM Access**: Accounts for 30% of power
2. **Optimize Compute Utilization**: Idle units waste static power
3. **Data Reuse**: Keep data in cache

### Efficiency Metrics

**TOPS/W (Tera Operations Per Second per Watt)**:
```
Apple A17 Pro: 35 TOPS / 6W = 5.8 TOPS/W
Qualcomm SD 8 Gen 3: 73 TOPS / 8W = 9.1 TOPS/W
Google Edge TPU: 4 TOPS / 2W = 2 TOPS/W
NVIDIA Jetson Orin: 275 TOPS / 25W = 11 TOPS/W
```

**Note**: Higher TOPS/W doesn't always mean better for real workloads.

**Effective TOPS** (accounting for utilization):
```
Peak TOPS: 100
Utilization: 60% (typical for real workloads)
Effective TOPS: 60

Reason: Memory bandwidth bottlenecks, control overhead
```

---

## Challenges and Future Directions

### Current Limitations

1. **Limited Flexibility**: Optimized for specific operations
2. **Software Ecosystem**: Less mature than GPUs
3. **Debugging Tools**: Limited profiling and visualization
4. **Training Support**: Most NPUs inference-only

### Emerging Trends

**1. Hybrid Architectures**:
```
CPU + GPU + NPU on same die

Example: Apple M4
- 10 CPU cores (general compute)
- 10 GPU cores (graphics + some AI)
- 16 NPU cores (specialized AI)
- Unified memory architecture
```

**2. Sparsity Support**:
```
Neural networks are 50-90% sparse after pruning

Sparse NPU:
- Skip zero multiplications
- Compressed weight storage
- 2-5x effective speedup
```

**3. Transformer-Specific Units**:
```
Attention Accelerators:
- Flash attention in hardware
- Fused QKV projection
- Softmax optimization
- KV cache management

Example: NVIDIA H100 Transformer Engine
```

**4. Neuromorphic Evolution**:
```
Spiking Neural Networks (SNNs)
Event-driven computation
Ultra-low power (mW scale)

Example: Intel Loihi 2
```

---

## Conclusion

NPUs represent the future of edge AI, offering unparalleled efficiency for neural network inference. Key takeaways:

**For Hardware Designers**:
- Systolic arrays provide efficient matrix computation
- Memory hierarchy is critical for performance
- Quantization support is essential
- Power efficiency matters more than peak TOPS

**For Software Developers**:
- Use framework-provided NPU support
- Quantization-aware training improves accuracy
- Profile and optimize memory access patterns
- Choose efficient model architectures

**For System Architects**:
- NPUs complement CPUs and GPUs
- Consider TOPS/W, not just peak TOPS
- Software ecosystem maturity is crucial
- Plan for emerging workloads (LLMs on edge)

The next chapter explores TPU (Tensor Processing Unit) architecture, examining how Google's custom ASICs achieve massive scale for AI training and inference.

---

*© 2025 SmileStory Inc. / WIA*
*弘益人間 (Hongik Ingan) · Benefit All Humanity*
