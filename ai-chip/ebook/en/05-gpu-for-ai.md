# Chapter 5: GPU Architecture for AI

## From Graphics to General-Purpose AI Acceleration

Graphics Processing Units (GPUs) have evolved from specialized graphics accelerators into the dominant platform for AI training and inference. This chapter explores modern GPU architectures optimized for machine learning, with focus on NVIDIA's leadership, AMD's challenge, and Intel's entry into the AI GPU market.

---

## GPU Evolution: Graphics to AI

### The CUDA Revolution (2006)

NVIDIA's CUDA (Compute Unified Device Architecture) transformed GPUs into general-purpose processors:

**Before CUDA (2000-2006)**:
- GPUs designed exclusively for graphics pipelines
- Fixed-function hardware for rendering
- Difficult to program for non-graphics tasks
- Limited precision (designed for colors, not math)

**After CUDA (2007-present)**:
- Programmable compute cores
- C/C++ programming model
- Double-precision floating-point
- General parallel computing platform

**The Pivotal Moment** (2012):
```
AlexNet wins ImageNet competition
- Trained on 2x NVIDIA GTX 580 GPUs
- 10x faster than CPU implementation
- Sparked deep learning revolution

Result: GPUs became essential for AI research
```

---

## NVIDIA GPU Architecture for AI

### Turing Architecture (2018): RT and Tensor Cores

**Key Features**:
- RT Cores (ray tracing)
- 2nd Gen Tensor Cores
- INT8/INT4 precision for inference
- DLSS (Deep Learning Super Sampling)

**Tensor Core Evolution**:
```
1st Gen (Volta, 2017):
- FP16 matrix multiplication
- 125 TFLOPS (mixed precision)

2nd Gen (Turing, 2018):
- INT8, INT4 support
- 500 TOPS (INT8)

3rd Gen (Ampere, 2020):
- FP64, TF32, BF16, FP16, INT8
- 312 TFLOPS (TF32)
- Sparse tensor acceleration

4th Gen (Ada/Hopper, 2022):
- FP8 precision
- Transformer Engine
- 2,000 TOPS (FP8)
```

### Ampere Architecture (2020): A100 Data Center GPU

**Specifications** (A100 80GB):
- **Process**: TSMC 7nm
- **Transistors**: 54.2 billion
- **Die Size**: 826 mm²
- **CUDA Cores**: 6,912
- **Tensor Cores**: 432 (3rd gen)
- **Memory**: 80GB HBM2e
- **Bandwidth**: 2.0 TB/s
- **TDP**: 400W
- **Interconnect**: NVLink 600 GB/s

**Performance**:
- FP64: 9.7 TFLOPS
- FP32: 19.5 TFLOPS
- TF32: 156 TFLOPS (with Tensor Cores)
- FP16: 312 TFLOPS
- INT8: 624 TOPS
- **Sparsity**: 2x acceleration for 50% sparse models

**Multi-Instance GPU (MIG)**:
```
Partition single A100 into up to 7 independent GPUs:

A100 (full):
┌────────────────────────────────┐
│ 80GB │ 6912 CUDA cores         │
└────────────────────────────────┘

A100 (7× MIG instances):
┌────┬────┬────┬────┬────┬────┬────┐
│10GB│10GB│10GB│10GB│10GB│10GB│10GB│
│ 1G │ 1G │ 1G │ 1G │ 1G │ 1G │ 1G │
└────┴────┴────┴────┴────┴────┴────┘

Benefits:
- Isolate workloads
- Maximize utilization
- QoS guarantees
- Multi-tenancy
```

**Use Cases**:
- Large-scale AI training (GPT-3, DALL-E)
- High-performance computing (molecular dynamics)
- Data analytics (Apache Spark GPU)
- Inference serving (TensorRT)

### Hopper Architecture (2022): H100 - Built for Transformers

**Specifications** (H100 80GB):
- **Process**: TSMC 4N (custom 4nm)
- **Transistors**: 80 billion
- **Die Size**: 814 mm²
- **SM (Streaming Multiprocessors)**: 132
- **CUDA Cores**: 16,896
- **Tensor Cores**: 528 (4th gen)
- **Memory**: 80GB HBM3
- **Bandwidth**: 3.35 TB/s
- **TDP**: 700W
- **NVLink**: 900 GB/s

**Performance** (vs A100):
- FP64: 60 TFLOPS (6x improvement)
- FP32: 120 TFLOPS (6x)
- TF32: 1,000 TFLOPS (6x)
- FP16: 2,000 TFLOPS (6x)
- FP8: 4,000 TFLOPS (2x over FP16)
- INT8: 4,000 TOPS

**Revolutionary Transformer Engine**:
```
Automatic FP8 Precision Management:

Traditional Mixed Precision:
Developer manually selects FP16/FP32 for each layer
Risk of overflow or underflow
Requires extensive tuning

Transformer Engine:
- Automatic precision selection per layer
- Dynamic range analysis at runtime
- FP8 for compute, FP16 for accumulate
- Zero manual tuning required

Result:
- 2x faster training vs FP16
- Same accuracy as FP16
- 6x faster vs A100 for GPT-3
```

**Other H100 Innovations**:

1. **DPX Instructions** (Dynamic Programming):
```
Accelerate algorithms like:
- Smith-Waterman (gene sequencing)
- Floyd-Warshall (shortest path)
- Navi-Stokes (fluid dynamics)

Performance: 7x faster than A100
```

2. **Confidential Computing**:
```
Hardware-based encryption of memory
Secure multi-party AI training
Protected from unauthorized access
```

3. **NVLink Switch System**:
```
Traditional NVLink:
GPU ←→ GPU peer-to-peer

NVLink Switch (256 H100s):
         ┌───────┐
     ┌───┤Switch ├───┐
     │   └───────┘   │
  ┌──▼──┐         ┌──▼──┐
  │ GPU │   ...   │ GPU │
  └─────┘         └─────┘

Benefits:
- All-to-all connectivity
- 57.6 TB/s aggregate bandwidth
- Scales to 256 GPUs
```

**H100 Benchmarks** (MLPerf Training v3.0):

| Workload | H100 | A100 | Speedup |
|----------|------|------|---------|
| BERT | 3.3 min | 10.4 min | 3.2x |
| GPT-3 175B | 10.9 min | 51.0 min | 4.7x |
| Stable Diffusion | 10.6 min | 28.7 min | 2.7x |
| ResNet-50 | 1.1 min | 2.5 min | 2.3x |

### H200: Memory-Centric AI (2023)

**Key Upgrade**: Focus on LLM inference

**Specifications**:
- Same die as H100 (no compute changes)
- **Memory**: **141GB HBM3e** (vs 80GB on H100)
- **Bandwidth**: **4.8 TB/s** (vs 3.35 TB/s)
- Connector: Same as H100 (drop-in replacement)

**Target Workload**: Large Language Model Inference

```
LLM Inference Bottleneck Analysis:

Compute-bound: ✗ (GPUs have excess FLOPS)
Memory-bound: ✓ (limited by bandwidth and capacity)

Example: GPT-3 175B inference
- Model size: 350GB (FP16)
- Fits on 3× H100 (80GB each = 240GB) ✗
- Fits on 2× H200 (141GB each = 282GB) ✓

Benefits:
- 1.4x memory capacity
- 1.4x memory bandwidth
- Fewer GPUs needed (lower cost)
- 1.6-1.9x throughput for LLMs
```

**Pricing**:
- H200 SXM: ~$30,000-35,000 per GPU
- Premium over H100: ~$5,000 (17%)
- TCO Improvement: 30-40% for LLM serving (fewer GPUs)

---

## NVIDIA Software Ecosystem

### CUDA: The Moat

**CUDA Advantages**:
1. **20+ Years of Development** (2006-2025)
2. **3 Million+ Developers**
3. **Comprehensive Libraries**:
   - cuDNN (deep learning primitives)
   - cuBLAS (linear algebra)
   - TensorRT (inference optimization)
   - NCCL (multi-GPU communication)
4. **Framework Integration**:
   - PyTorch (CUDA backend default)
   - TensorFlow (CUDA + cuDNN)
   - JAX (XLA → CUDA)

**CUDA Programming Example**:
```cuda
// Matrix multiplication kernel
__global__ void matmul(float *A, float *B, float *C, int N) {
    int row = blockIdx.y * blockDim.y + threadIdx.y;
    int col = blockIdx.x * blockDim.x + threadIdx.x;

    float sum = 0.0f;
    for (int k = 0; k < N; k++) {
        sum += A[row * N + k] * B[k * N + col];
    }
    C[row * N + col] = sum;
}

// Host code
dim3 threads(16, 16);
dim3 blocks(N/16, N/16);
matmul<<<blocks, threads>>>(d_A, d_B, d_C, N);
```

### TensorRT: Inference Optimization

**What is TensorRT?**
- NVIDIA's inference optimization engine
- Reduces latency and increases throughput
- Supports FP16, INT8, INT4 quantization

**Optimization Techniques**:
```
1. Layer Fusion:
   Conv + BatchNorm + ReLU → Single fused kernel

2. Precision Calibration:
   Automatic INT8 quantization with <1% accuracy loss

3. Kernel Auto-tuning:
   Select fastest kernel for your GPU and input size

4. Dynamic Tensor Memory:
   Reuse memory across layers

5. Multi-Stream Execution:
   Pipeline multiple inferences
```

**Performance Example**:
```python
import tensorrt as trt

# Convert ONNX to TensorRT
logger = trt.Logger(trt.Logger.WARNING)
builder = trt.Builder(logger)
network = builder.create_network()
parser = trt.OnnxParser(network, logger)

with open('model.onnx', 'rb') as model:
    parser.parse(model.read())

config = builder.create_builder_config()
config.set_flag(trt.BuilderFlag.FP16)  # Enable FP16

engine = builder.build_engine(network, config)

# Result: 2-5x faster inference
```

**Benchmarks** (ResNet-50 inference, batch size 1):
| Platform | Latency | Throughput |
|----------|---------|------------|
| PyTorch (FP32) | 7.2ms | 139 img/s |
| TensorRT (FP16) | 1.8ms | 556 img/s |
| TensorRT (INT8) | 0.9ms | 1111 img/s |

---

## AMD GPU Architecture for AI

### CDNA Architecture: Compute-Focused Design

Unlike NVIDIA's gaming+AI approach, AMD split:
- **RDNA**: Gaming GPUs (Radeon RX 7000 series)
- **CDNA**: Compute/AI GPUs (Instinct MI series)

**Philosophy**: Separate architectures optimize for different workloads.

### MI250X (2021): Dual-Die Design

**Specifications** (per GPU):
- **Process**: TSMC 6nm
- **Architecture**: CDNA 2
- **Compute Units**: 220 (dual-die, 110 per die)
- **Stream Processors**: 14,080
- **Matrix Cores**: 220
- **Memory**: 128GB HBM2e (64GB per die)
- **Bandwidth**: 3.2 TB/s
- **TDP**: 560W
- **Interconnect**: Infinity Fabric 400 GB/s

**Performance**:
- FP64: 47.9 TFLOPS (best in industry)
- FP32: 95.7 TFLOPS
- FP16: 383 TFLOPS
- BF16: 383 TFLOPS
- INT8: 383 TOPS

**Unique Feature**: Industry-leading FP64 performance
```
Scientific Computing Applications:
- Molecular dynamics simulations
- Climate modeling
- Computational fluid dynamics
- Quantum chemistry

MI250X dominates Frontier supercomputer:
- #1 on Top500 (June 2022-present)
- 1.1 exaflops (FP64)
- 9,472 AMD Instinct MI250X GPUs
```

### MI300X (2023): Chiplet Revolution

**Revolutionary 3D Chiplet Design**:
```
Traditional GPU:
┌─────────────────────┐
│   Monolithic Die    │
│  (Graphics + HBM)   │
└─────────────────────┘

MI300X (3D Stacked):
┌─────────────────────┐  ← HBM3 (8 stacks)
├─────────────────────┤
│   GPU Chiplets (8)  │  ← Compute dies
├─────────────────────┤
│   I/O Die           │  ← Interconnect
└─────────────────────┘

Benefits:
- Better yield (smaller dies)
- Modular scaling
- Shorter HBM paths (lower latency)
```

**Specifications**:
- **Process**: TSMC 5nm (GPU), 6nm (I/O)
- **Transistors**: 153 billion (most in a GPU)
- **Compute Chiplets**: 8× GPU dies
- **Compute Units**: 304
- **Memory**: **192GB HBM3** (most in industry)
- **Bandwidth**: 5.2 TB/s
- **TDP**: 750W
- **Interconnect**: Infinity Fabric 896 GB/s

**Performance**:
- FP64: 163.4 TFLOPS
- FP32: 653.7 TFLOPS
- FP16: 1,307 TFLOPS
- BF16: 1,307 TFLOPS
- FP8: 2,614 TFLOPS
- INT8: 2,614 TOPS

**Key Differentiator: Memory Capacity**
```
LLM Serving Comparison:

LLaMA 2 70B model (FP16):
- Model size: 140GB
- NVIDIA H100 (80GB): Needs 2 GPUs
- AMD MI300X (192GB): Fits on 1 GPU

Advantages:
- Lower latency (no multi-GPU overhead)
- Higher throughput
- Lower total cost
- Simpler deployment
```

**Early Adopters**:
- Microsoft Azure
- Oracle Cloud
- Meta (Llama inference)

**Pricing**:
- MI300X: ~$15,000-18,000 per GPU
- 40-50% cheaper than H100
- Better price/performance for LLM inference

### ROCm: AMD's CUDA Alternative

**ROCm** (Radeon Open Compute):
- Open-source compute platform
- Supports AMD GPUs
- CUDA→ROCm porting tools

**Architecture**:
```
ROCm Stack:

┌─────────────────────────────────┐
│  Applications & Frameworks      │
│  (PyTorch, TensorFlow)          │
├─────────────────────────────────┤
│  Libraries                      │
│  (MIOpen, rocBLAS, rocFFT)      │
├─────────────────────────────────┤
│  Language Runtimes              │
│  (HIP, OpenCL, OpenMP)          │
├─────────────────────────────────┤
│  Drivers (amdgpu)               │
└─────────────────────────────────┘
```

**HIP** (Heterogeneous Interface for Portability):
```cpp
// CUDA code
__global__ void add(float *a, float *b, float *c) {
    int i = threadIdx.x;
    c[i] = a[i] + b[i];
}

// Convert to HIP (mostly automatic)
__global__ void add(float *a, float *b, float *c) {
    int i = threadIdx.x;
    c[i] = a[i] + b[i];
}

// HIP works on both NVIDIA and AMD GPUs
// hipify tool automates CUDA→HIP conversion
```

**Challenges**:
1. **Ecosystem Gap**: Fewer libraries than CUDA
2. **Documentation**: Less comprehensive
3. **Community**: Smaller developer base
4. **Framework Support**: Improving but incomplete

**Progress**:
- PyTorch: Official ROCm support (since 2020)
- TensorFlow: ROCm builds available
- ONNX Runtime: ROCm provider
- MLPerf: AMD submits competitive results

---

## Intel GPU Architecture for AI

### Ponte Vecchio: HPC and AI Convergence

**Specifications** (Data Center GPU Max 1550):
- **Process**: TSMC 5nm + Intel 7nm
- **Architecture**: Xe HPC
- **Tiles**: 47 (compute + HBM + I/O)
- **Xe Cores**: 128
- **Vector Engines**: 512
- **Matrix Engines**: 512 (XMX units)
- **Memory**: 128GB HBM2e
- **Bandwidth**: 3.2 TB/s
- **TDP**: 600W
- **Interconnect**: Xe Link 512 GB/s

**Performance**:
- FP64: 44.8 TFLOPS
- FP32: 89.6 TFLOPS
- TF32: 179.2 TFLOPS
- FP16: 358.4 TFLOPS
- BF16: 358.4 TFLOPS
- INT8: 716.8 TOPS

**Target Market**:
- High-performance computing (Aurora supercomputer)
- AI training and inference
- Scientific simulations

**Deployment**:
```
Aurora Supercomputer (Argonne National Lab):
- 10,624× Intel Data Center GPU Max
- 2 exaflops peak performance
- #2 on Top500 (November 2023)

Applications:
- Climate modeling
- Cosmology simulations
- Drug discovery
- AI research
```

### oneAPI: Intel's Unified Programming Model

**Philosophy**: Write once, run anywhere (CPU, GPU, FPGA)

```cpp
// oneAPI DPC++ (Data Parallel C++)
#include <sycl/sycl.hpp>

sycl::queue q;

// Allocate memory
float *a = sycl::malloc_shared<float>(N, q);
float *b = sycl::malloc_shared<float>(N, q);
float *c = sycl::malloc_shared<float>(N, q);

// Kernel
q.parallel_for(N, [=](sycl::id<1> i) {
    c[i] = a[i] + b[i];
}).wait();

// Works on Intel CPU, Intel GPU, NVIDIA GPU (via plugin)
```

**Advantages**:
- Standards-based (SYCL, OpenMP)
- Cross-vendor support
- Single codebase for heterogeneous systems

**Challenges**:
- Adoption (developers prefer CUDA or ROCm)
- Performance parity with native CUDA

---

## GPU Memory Technologies

### HBM (High Bandwidth Memory) Evolution

| Generation | Bandwidth/Stack | Capacity/Stack | Introduction |
|------------|-----------------|----------------|--------------|
| HBM1 | 128 GB/s | 1-4 GB | 2015 |
| HBM2 | 256 GB/s | 4-8 GB | 2016 |
| HBM2e | 460 GB/s | 8-16 GB | 2018 |
| HBM3 | 819 GB/s | 16-24 GB | 2022 |
| HBM3e | 1.15 TB/s | 24-36 GB | 2023 |

**H200 Example**:
- 5× HBM3e stacks
- 36GB each (Samsung)
- Total: 180GB (141GB usable)
- Aggregate bandwidth: 4.8 TB/s

### NVLink: GPU-to-GPU Interconnect

**Evolution**:
```
PCIe 4.0: 64 GB/s (16 lanes)

NVLink v1 (P100): 160 GB/s
NVLink v2 (V100): 300 GB/s
NVLink v3 (A100): 600 GB/s
NVLink v4 (H100): 900 GB/s

Advantage: 14x faster than PCIe
```

**NVLink Topologies**:
```
2× GPU: Direct connection
4× GPU: All-to-all mesh
8× GPU: NVSwitch-based full connectivity

DGX H100:
┌──────┐   ┌──────┐
│ GPU0 │═══│ GPU1 │
└───┬──┘   └──┬───┘
    ║         ║
┌───╬─────────╬───┐
│   ║  NVSwitch  ║ │
└───╬─────────╬───┘
    ║         ║
┌───┴──┐   ┌──┴───┐
│ GPU2 │═══│ GPU3 │
└──────┘   └──────┘

Bandwidth: 900 GB/s per GPU, all-to-all
```

---

## Conclusion

GPUs remain the dominant AI accelerator due to:

**NVIDIA Strengths**:
- CUDA ecosystem lock-in
- Continuous innovation (Hopper, H200)
- Complete software stack
- 80%+ market share

**AMD Opportunities**:
- Competitive hardware (MI300X)
- Superior memory capacity
- Lower pricing
- Open-source software (ROCm)

**Intel Potential**:
- oneAPI portability
- Integration with CPUs
- Foundry services
- HPC crossover

**For Practitioners**:
- Default to NVIDIA for maximum compatibility
- Consider AMD for LLM inference (memory advantage)
- Intel for HPC+AI hybrid workloads
- Monitor ROCm ecosystem maturity

Next chapter: Edge AI chips (Qualcomm, Apple, mobile processors).

---

*© 2025 SmileStory Inc. / WIA*
*弘益人間 (Hongik Ingan) · Benefit All Humanity*
