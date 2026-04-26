# Chapter 4: TPU Deep Dive - Google's Tensor Processing Units

## Custom ASICs for Machine Learning at Scale

Google's Tensor Processing Units (TPUs) represent one of the most significant innovations in AI hardware design. This chapter provides an in-depth exploration of TPU architecture, design decisions, and the evolution from inference-only v1 to the massive training clusters of v5p.

---

## The TPU Story: Why Google Built Custom AI Chips

### The Motivation (2013-2015)

In 2013, Google engineers made a startling discovery:

> "If every person used voice search for just three minutes per day, Google would need to **double its data center capacity**." - Google Research

This realization led to a critical question: Could custom hardware accelerate neural networks more efficiently than CPUs or GPUs?

**Key Drivers**:
1. **Cost**: GPU solutions were expensive at scale
2. **Power**: Data center power consumption unsustainable
3. **Performance**: Faster inference needed for real-time applications
4. **Control**: Independence from GPU vendors

**The Decision** (2015): Design a custom ASIC optimized specifically for TensorFlow operations.

---

## TPU v1: The Inference Revolution (2016)

### Architecture Overview

TPU v1 was laser-focused on inference:

**Specifications**:
- **Peak Performance**: 92 TOPS (INT8)
- **Process**: 28nm
- **Die Size**: < 331 mm²
- **Power**: 28-40W
- **Memory**: 28GB off-chip (4GB on-chip)
- **Deployment**: Over 1 million deployed by 2017

### Core Architecture

#### Matrix Multiply Unit (MXU)

The heart of TPU v1 is a 256x256 systolic array:

```
TPU v1 Systolic Array:

256 ┌─────────────────────────────────┐
    │  [PE][PE][PE]...[PE] (256 PEs)  │
    │  [PE][PE][PE]...[PE]            │
    │  [PE][PE][PE]...[PE]            │
    │   ...                           │
256 │  [PE][PE][PE]...[PE]            │
    └─────────────────────────────────┘

Total PEs: 256 × 256 = 65,536
Each PE: 8-bit multiply-accumulate (MAC)

Peak Throughput:
65,536 MACs × Clock Frequency (700 MHz)
= 92 TOPS (INT8 operations)
```

**Key Innovations**:
1. **Weight Preloading**: Weights loaded once, reused for entire batch
2. **Reduced Precision**: INT8 only (vs FP32 on GPUs)
3. **Deterministic Performance**: No branches or memory divergence
4. **Custom Dataflow**: Optimized for neural network patterns

#### Memory Architecture

```
TPU v1 Memory Hierarchy:

┌──────────────────────────────┐
│   Host DRAM (via PCIe)       │
│   Weights, Activations       │
└──────────────┬───────────────┘
               │ PCIe Gen3 x16
               │ 16 GB/s
┌──────────────▼───────────────┐
│   Unified Buffer (24MB)      │
│   On-chip SRAM               │
│   Bandwidth: 4 TB/s          │
└──────────────┬───────────────┘
               │
┌──────────────▼───────────────┐
│   Matrix Multiply Unit       │
│   256×256 Systolic Array     │
└──────────────────────────────┘
```

**Design Philosophy**:
- Large on-chip buffer reduces DRAM access
- High internal bandwidth (4 TB/s)
- Simple addressing (no complex caching)

### Quantization Strategy

TPU v1 uses aggressive INT8 quantization:

```python
# Quantization Process (simplified)

# 1. Collect activation statistics during calibration
min_val, max_val = collect_activation_range(model, calibration_data)

# 2. Calculate quantization parameters
scale = (max_val - min_val) / 255.0
zero_point = -round(min_val / scale)

# 3. Quantize weights and activations
def quantize(x):
    return np.clip(round(x / scale) + zero_point, 0, 255).astype(np.uint8)

# 4. Dequantize outputs
def dequantize(x_q):
    return (x_q.astype(np.float32) - zero_point) * scale
```

**Accuracy Impact**:
- Most models: <1% top-1 accuracy loss
- Computer vision: Negligible impact
- NLP (BERT): 0.3-0.5% impact

### Real-World Applications

TPU v1 powered Google's production services:

**Google Search**:
- RankBrain (search ranking)
- Featured Snippets extraction
- Query understanding
- **Latency**: <5ms for inference
- **Throughput**: Millions of queries per second

**Google Photos**:
- Image recognition and labeling
- Face detection and grouping
- Object detection
- **Processing**: 3 billion images per day

**Google Translate**:
- Neural Machine Translation (NMT)
- 100+ language pairs
- **Latency**: <200ms for sentence translation
- **Quality**: 60% error reduction vs phrase-based

**Gmail**:
- Smart Reply suggestions
- Spam detection
- Priority Inbox ranking

### Performance Comparison (2017)

| Platform | Performance | Power | TOPS/W | Cost |
|----------|-------------|-------|--------|------|
| Intel Haswell CPU | 0.3 TOPS | 150W | 0.002 | $1000 |
| NVIDIA K80 GPU | 8.7 TOPS | 300W | 0.029 | $5000 |
| **Google TPU v1** | **92 TOPS** | **40W** | **2.3** | **~$1500** |

**Key Insight**: TPU v1 achieved **80x better performance per watt** than CPUs.

---

## TPU v2: Training Breakthrough (2017)

### Major Architectural Changes

TPU v2 added training capabilities:

**Specifications**:
- **Peak Performance**: 180 TFLOPS (FP32), 45 TFLOPS per chip
- **Process**: 16nm
- **Memory**: 16GB HBM (High Bandwidth Memory)
- **Bandwidth**: 600 GB/s per chip
- **Interconnect**: 2D toroidal mesh
- **Power**: 280W per chip
- **Configuration**: 4-chip board, 256-chip pod

### Floating-Point Support

Unlike v1, TPU v2 supports **bfloat16** (Brain Float):

```
IEEE FP32:  [Sign: 1 bit][Exponent: 8 bits][Mantissa: 23 bits]
bfloat16:   [Sign: 1 bit][Exponent: 8 bits][Mantissa: 7 bits]

Key Properties:
- Same exponent range as FP32
- Reduced mantissa precision
- Direct truncation from FP32
- Better for neural networks than FP16
```

**Advantages**:
1. **Wider Dynamic Range**: Same as FP32 (vs FP16's limited range)
2. **Easy Conversion**: Just drop lower 16 bits of FP32
3. **Training Stability**: Rarely requires loss scaling
4. **Hardware Efficiency**: 2x smaller than FP32

**Example**:
```python
# FP32 to bfloat16 conversion
import numpy as np

def fp32_to_bf16(x):
    # Simple truncation (in practice, uses rounding)
    return np.frombuffer(
        x.astype(np.float32).tobytes()[2::4] +
        x.astype(np.float32).tobytes()[3::4],
        dtype=np.uint16
    )
```

### TPU v2 Pod Architecture

A TPU v2 Pod contains 256 chips:

```
TPU v2 Pod (256 chips):

2D Toroidal Mesh Interconnect:
┌────┬────┬────┬────┬────┬────┬────┬────┐
│ T  │ T  │ T  │ T  │ T  │ T  │ T  │ T  │
├────┼────┼────┼────┼────┼────┼────┼────┤
│ T  │ T  │ T  │ T  │ T  │ T  │ T  │ T  │
├────┼────┼────┼────┼────┼────┼────┼────┤
│ T  │ T  │ T  │ T  │ T  │ T  │ T  │ T  │
├────┼────┼────┼────┼────┼────┼────┼────┤
...  (32 rows × 8 columns = 256 TPUs)

Each TPU (T) connected to 4 neighbors
Bandwidth: 496 GB/s per link
Total Pod Memory: 4 TB (16GB × 256)
Total Pod Compute: 11.5 petaflops
```

**Training at Scale**:
- **BERT-Large**: 4 days on 16-chip pod (vs 4 weeks on GPUs)
- **ResNet-50**: 30 minutes to 76% accuracy (ImageNet)
- **Transformer-Big**: 14 hours on 256-chip pod

### Software Stack: TensorFlow Integration

TPU v2 requires TensorFlow-specific code:

```python
import tensorflow as tf

# TPU initialization
resolver = tf.distribute.cluster_resolver.TPUClusterResolver()
tf.config.experimental_connect_to_cluster(resolver)
tf.tpu.experimental.initialize_tpu_system(resolver)

# Distribution strategy
strategy = tf.distribute.TPUStrategy(resolver)

# Model definition within strategy scope
with strategy.scope():
    model = create_model()
    model.compile(...)

# Training
model.fit(dataset, epochs=10)
```

**XLA Compilation** (Accelerated Linear Algebra):
```
TensorFlow Graph → XLA Compiler → TPU Instructions

Optimizations:
- Operator fusion
- Layout optimization
- Constant folding
- Dead code elimination
```

---

## TPU v3: Scaling to Exascale (2018)

### Key Improvements

**Specifications**:
- **Peak Performance**: 420 TFLOPS (bfloat16) per chip
- **Memory**: 32GB HBM2 per chip
- **Bandwidth**: 900 GB/s per chip
- **Power**: 450W per chip
- **Cooling**: **Liquid cooling** (first for Google)
- **Pod Size**: 1024 chips (v3 mega-pod)
- **Total Compute**: 100+ petaflops per pod

### Liquid Cooling Innovation

TPU v3's power density required liquid cooling:

```
Traditional Air Cooling Limit: ~250W per chip
TPU v3 Requirement: 450W per chip

Solution: Direct liquid cooling
- Coolant flows through cold plates
- Heat transferred to facility water
- 30% more power efficient
- Quieter data centers
```

**Environmental Impact**:
- 2x power density vs air cooling
- Same space, 2x compute
- Reduced data center footprint

### Pod Interconnect Evolution

```
TPU v3 Pod (1024 chips):

3D Toroidal Mesh:
- Each chip connects to 6 neighbors (up/down/left/right/front/back)
- Bandwidth: 656 GB/s per link
- All-reduce latency: <100μs
- Total bisection bandwidth: >100 TB/s

Enables:
- Model parallelism (models larger than 32GB)
- Data parallelism (1024-way batching)
- Pipeline parallelism (stage different layers)
```

### Training Achievements

**Record-Breaking Models** (trained on TPU v3):

1. **BERT-Large** (340M parameters)
   - Training time: 76 minutes (1024 chips)
   - Accuracy: State-of-the-art on GLUE benchmark

2. **AmoebaNet-A** (ImageNet)
   - Architecture discovered via neural architecture search
   - Training: 7 days on 450-chip pod
   - Top-1 accuracy: 83.9%

3. **GPipe** (557M parameter translation)
   - 4.3 BLEU improvement over Transformer-Big
   - 25-billion-word dataset
   - Pipeline parallelism across 8 chips

---

## TPU v4: Optical Interconnects (2021)

### Revolutionary Optical Circuit Switch (OCS)

TPU v4 introduced **photonic** interconnects:

**Problem**: Electrical interconnects limit pod scaling
- Copper bandwidth plateauing
- Power consumption high
- Physical constraints

**Solution**: Optical Circuit Switch
```
Electrical (TPU v3):
Chip → Electrical SerDes → Copper Cable → SerDes → Chip
Bandwidth: 656 GB/s per link
Power: ~5W per link

Optical (TPU v4):
Chip → Optical Transceiver → Fiber → OCS → Fiber → Transceiver → Chip
Bandwidth: 10-100x potential
Power: <2W per link
Latency: Similar to electrical
```

**Specifications**:
- **Compute**: 275 TFLOPS (bfloat16) per chip
- **Memory**: 32GB HBM2
- **Pod Size**: 4096 chips (optically connected)
- **Total Compute**: 1.1 exaflops per pod
- **Interconnect**: Optical Circuit Switch

### Pod Architecture

```
TPU v4 Pod (4096 chips):

Optical Circuit Switch Topology:
┌─────────────────────────────────────┐
│     Optical Circuit Switch (OCS)    │
│                                     │
│   ┌─┬─┬─┐     ┌─┬─┬─┐              │
│   │T│T│T│ ... │T│T│T│ (4096 TPUs)  │
│   └─┴─┴─┘     └─┴─┴─┘              │
│                                     │
│   Reconfigurable optical paths      │
│   Sub-microsecond reconfiguration   │
│   Zero electrical switching         │
└─────────────────────────────────────┘

Benefits:
- Dynamic topology reconfiguration
- Zero contention
- Perfect load balancing
- Scales to 10,000+ chips
```

**Training Achievements**:

1. **PaLM-540B** (540 billion parameters)
   - Largest language model (at time)
   - 6144 TPU v4 chips
   - 50 days of training
   - Cost: ~$10M

2. **Minerva** (Math reasoning)
   - Based on PaLM architecture
   - Specialized math training
   - SOTA on math benchmarks

---

## TPU v5e and v5p: Efficiency and Performance (2023)

### TPU v5e: Cost-Optimized Inference

**Target**: Affordable AI inference at scale

**Specifications**:
- **Performance**: 197 TFLOPS (bfloat16)
- **Memory**: 16GB HBM
- **Price/Performance**: 2x better than v4
- **Power**: <200W
- **Use Case**: Serving LLMs in production

**Optimizations**:
- Reduced precision paths (INT8, INT4)
- Inference-specific operator fusion
- Smaller die size (lower cost)
- Shared infrastructure with v5p

### TPU v5p: Flagship Training Chip

**Specifications**:
- **Performance**: 459 TFLOPS (bfloat16) per chip
- **Memory**: 95GB HBM2e
- **Bandwidth**: 1.6 TB/s
- **Pod Size**: 8960 chips
- **Total Compute**: 4.1 exaflops
- **Power**: 500W per chip

**Architectural Innovations**:

1. **SparseCore**: Hardware sparsity acceleration
```
Dense Computation:
[1, 0, 0, 3, 0, 5] × [2, 4, 1, 0, 3, 2]
= 1×2 + 0×4 + 0×1 + 3×0 + 0×3 + 5×2 = 12

Sparse (90% zeros):
Skip zero multiplications
Compressed storage
2-10x speedup for sparse models
```

2. **FlashAttention v2**: Hardware-accelerated attention
```
Attention(Q, K, V) = softmax(QK^T / √d) V

Traditional: Materializes full QK^T matrix (memory-bound)
FlashAttention: Fused kernel, tiled computation
Result: 2-4x faster for transformers
```

3. **ICI (Inter-Chip Interconnect)**: 4.8 TB/s per chip

**Training Records**:

**Gemini 1.0** (Google's multimodal LLM):
- Multiple model sizes (Nano, Pro, Ultra)
- Trained on TPU v5p
- Text, image, video, audio understanding

**PaLM 2**:
- Multilingual, reasoning, coding
- More efficient than PaLM (smaller, better)
- Powers Bard (now Gemini)

---

## TPU Programming and Optimization

### JAX: The Modern TPU Framework

```python
import jax
import jax.numpy as jnp
from jax import grad, jit, vmap

# Automatic differentiation
def loss_fn(params, x, y):
    predictions = model(params, x)
    return jnp.mean((predictions - y) ** 2)

# Compile to TPU
@jit  # Just-In-Time compilation to XLA
def train_step(params, x, y, lr):
    loss, grads = jax.value_and_grad(loss_fn)(params, x, y)
    params = jax.tree_map(lambda p, g: p - lr * g, params, grads)
    return params, loss

# Vectorize across batch
batched_train = vmap(train_step, in_axes=(None, 0, 0, None))

# Multi-device training
from jax.experimental import mesh_utils
from jax.sharding import PositionalSharding

devices = mesh_utils.create_device_mesh((8,))
sharding = PositionalSharding(devices)

# Shard data across TPU cores
sharded_data = jax.device_put(data, sharding)
```

### Pipelining for Large Models

```python
# Model parallelism with pjit
from jax.experimental.pjit import pjit

# Partition large model across devices
with mesh(devices, ('model',)):
    @pjit(
        in_axis_resources=(P('model'), P(None)),
        out_axis_resources=P('model')
    )
    def sharded_forward(params, x):
        return large_model(params, x)
```

### Performance Optimization Tips

**1. Batch Size**:
```
Small batch (32): Poor TPU utilization (~40%)
Large batch (1024): Good utilization (~85%)

Rule: Use largest batch that fits in memory
```

**2. XLA Optimization**:
```python
# Enable XLA optimizations
tf.config.optimizer.set_jit(True)

# Profile and optimize
with tf.profiler.experimental.Profile('logdir'):
    model.fit(...)
```

**3. Mixed Precision**:
```python
# Use bfloat16 for compute, fp32 for storage
policy = tf.keras.mixed_precision.Policy('mixed_bfloat16')
tf.keras.mixed_precision.set_global_policy(policy)
```

---

## TPU vs GPU: Detailed Comparison

### Architecture Philosophy

| Aspect | TPU | GPU |
|--------|-----|-----|
| **Design Goal** | ML-specific ASIC | General parallel compute |
| **Dataflow** | Systolic array | SIMT (Single Instruction, Multiple Threads) |
| **Precision** | INT8, bfloat16 | FP16, FP32, FP64 |
| **Memory** | High bandwidth HBM | GDDR6/HBM |
| **Programmability** | TensorFlow/JAX | CUDA, HIP, SYCL |
| **Ecosystem** | Google Cloud only | Widespread |

### Performance Comparison (Training)

**BERT-Large Training** (to convergence):
| Platform | Time | Cost |
|----------|------|------|
| 8× NVIDIA V100 | 3 days | $192 |
| TPU v3 (8 cores) | 76 min | $9.76 |
| TPU v4 (16 cores) | 40 min | $5.20 |

**ResNet-50 Training** (to 76% ImageNet accuracy):
| Platform | Time | Cost |
|----------|------|------|
| 8× NVIDIA V100 | 1.5 hours | $12 |
| TPU v3 Pod slice (32 cores) | 13 min | $2.34 |

### When to Choose TPU vs GPU

**Choose TPU when**:
- Using TensorFlow or JAX
- Training large models (>1B parameters)
- Running on Google Cloud Platform
- Need cost-efficient training at scale
- Working with Google AI frameworks

**Choose GPU when**:
- Need maximum flexibility
- Using PyTorch (primary framework)
- Require on-premise deployment
- Need CUDA ecosystem libraries
- Doing graphics + AI workloads

---

## Future of TPU Architecture

### Anticipated TPU v6 Features

**Predictions** (based on industry trends):
1. **Advanced Process Node**: 3nm or 2nm
2. **Higher HBM Capacity**: 128-192GB per chip
3. **Improved Sparsity**: 50-80% zero-skip efficiency
4. **FP8 Support**: Emerging standard for LLM training
5. **Enhanced Optical Interconnect**: 10 TB/s+ per chip
6. **Specialized Attention Units**: Transformer-optimized hardware

### Long-Term Vision

**Photonic Computing**:
- All-optical neural networks
- Light-speed matrix multiplication
- Near-zero power for data movement

**Analog Computing**:
- In-memory computation
- Crossbar array architectures
- 100-1000x power efficiency potential

**Neuromorphic Integration**:
- Spiking neural network support
- Event-driven sparse computation
- Brain-inspired architectures

---

## Conclusion

Google's TPU journey demonstrates:

**Technical Excellence**:
- Systolic arrays are highly efficient for ML
- Custom ASICs can outperform GPUs for specific workloads
- Vertical integration (hardware + software) enables optimization

**Strategic Impact**:
- Independence from GPU vendors
- Cost savings (40-60% vs GPU solutions)
- Competitive advantage in AI research

**Lessons for Industry**:
- Domain-specific architectures matter
- Software ecosystem is as important as hardware
- Scale requires innovation at all levels (silicon, interconnect, cooling)

**For Practitioners**:
- TPUs excel at large-scale training
- TensorFlow/JAX optimization crucial
- Cloud-only limits adoption
- Consider cost, not just performance

The next chapter examines GPU architectures for AI, exploring how NVIDIA, AMD, and Intel optimize graphics processors for machine learning workloads.

---

*© 2025 SmileStory Inc. / WIA*
*弘益人間 (Hongik Ingan) · Benefit All Humanity*
