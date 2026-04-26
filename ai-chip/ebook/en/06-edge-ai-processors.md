# Chapter 6: Edge AI Processors

## AI at the Edge: Mobile, IoT, and Embedded Intelligence

Edge AI processors bring neural network inference to devices with limited power budgets, thermal constraints, and real-time requirements. This chapter explores specialized edge AI chips from Qualcomm, Apple, MediaTek, and emerging players optimizing for on-device intelligence.

---

## Why Edge AI?

### The Edge Computing Imperative

**Definition**: Edge AI runs neural networks locally on devices rather than in the cloud.

**Drivers**:
1. **Privacy**: GDPR, CCPA mandate local processing for sensitive data
2. **Latency**: Cloud round-trip (100-500ms) vs local inference (1-50ms)
3. **Bandwidth**: Streaming all data to cloud is expensive and impractical
4. **Reliability**: Works offline, no internet dependency
5. **Cost**: Avoid cloud inference fees (can be $1000s/month at scale)

**Market Size**:
- 2023: $13.5 billion
- 2030 (projected): $112 billion
- CAGR: 35.4%

**Applications**:
```
Smartphone AI:
- Photography (HDR, portrait mode, night mode)
- Voice assistants (Siri, Google Assistant, Alexa)
- Face unlock, fingerprint recognition
- AR filters, real-time translation
- Predictive text, smart replies

Automotive:
- Advanced Driver Assistance Systems (ADAS)
- In-cabin monitoring (driver drowsiness)
- Object detection, lane keeping
- Parking assistance

IoT:
- Smart cameras (person detection, package detection)
- Industrial automation (defect detection)
- Health monitoring (wearables, medical devices)
- Agriculture (crop monitoring, livestock tracking)
```

---

## Qualcomm: Snapdragon AI Dominance

### Hexagon Architecture Evolution

Qualcomm's Hexagon DSP has evolved into a dedicated AI accelerator:

**Generation Timeline**:
```
2015: Hexagon 680 (DSP with vector extensions)
2017: Hexagon 685 + HVX (Vector Processing)
2018: Hexagon 690 + Dedicated AI Engine
2019: Hexagon 698 (4th gen AI Engine, 15 TOPS)
2020: Hexagon 780 (6th gen AI Engine, 26 TOPS)
2021: Hexagon (7th gen, Snapdragon 8 Gen 1, 27 TOPS)
2022: Hexagon (Snapdragon 8 Gen 2, 60 TOPS)
2023: Hexagon (Snapdragon 8 Gen 3, 73 TOPS)
```

### Snapdragon 8 Gen 3 (2023): Flagship Architecture

**System-on-Chip Layout**:
```
┌─────────────────────────────────────┐
│   Snapdragon 8 Gen 3 (TSMC 4nm)    │
├─────────────────────────────────────┤
│  CPU Cluster:                       │
│  - 1× Cortex-X4 (3.3 GHz)           │
│  - 3× Cortex-A720 (3.2 GHz)         │
│  - 4× Cortex-A520 (2.3 GHz)         │
├─────────────────────────────────────┤
│  GPU: Adreno 750                    │
│  - Ray tracing, mesh shading        │
├─────────────────────────────────────┤
│  NPU: Hexagon AI Engine             │
│  - 73 TOPS (INT8)                   │
│  - Micro Tile Inferencing           │
│  - INT4 precision support           │
├─────────────────────────────────────┤
│  Memory: LPDDR5X (4200 MHz)         │
│  ISP: Spectra Triple 18-bit         │
│  Modem: X75 5G (10 Gbps)            │
└─────────────────────────────────────┘
```

**Hexagon NPU Specifications**:
- **Performance**: 73 TOPS (INT8)
- **Precision**: INT4, INT8, INT16, FP16
- **Tensor Accelerator**: Dedicated matrix multiplication units
- **Scalar Core**: Control flow and lightweight operations
- **Vector Extensions (HVX)**: SIMD operations for activations
- **Power**: 2-8W (dynamic, depending on workload)

**Key Innovation: Micro Tile Inferencing**
```
Problem: LLMs (7B-13B parameters) too large for mobile memory

Traditional Inference:
┌────────────────────────┐
│  Load entire 7B model  │
│  (14GB in FP16)        │  ← Doesn't fit in 12GB RAM!
└────────────────────────┘

Micro Tile Inferencing:
┌────┐
│Tile│ → Process → Discard
└────┘
┌────┐
│Tile│ → Process → Discard
└────┘
  ... (stream tiles from storage)

Benefits:
- Memory requirement: 1-2GB (just tile + activations)
- Enables LLaMA 2 7B on smartphone
- Tokens/second: 10-15 (acceptable for chat)
```

**Performance Benchmarks**:
| Model | Latency | Throughput |
|-------|---------|------------|
| MobileNet V2 | 0.8ms | 1250 fps |
| ResNet-50 | 2.1ms | 476 fps |
| BERT-Base | 15ms | 67 queries/s |
| Stable Diffusion | 12s | 0.08 images/s |
| LLaMA 2 7B | 85ms/token | 11.8 tokens/s |

### Qualcomm AI Stack

**Software Architecture**:
```
┌─────────────────────────────────────┐
│  ML Frameworks                      │
│  (TensorFlow Lite, PyTorch Mobile,  │
│   ONNX Runtime, MediaPipe)          │
├─────────────────────────────────────┤
│  QNN (Qualcomm Neural Network SDK)  │
│  - Model conversion                 │
│  - Graph optimization               │
│  - Quantization                     │
├─────────────────────────────────────┤
│  Runtime:                           │
│  - CPU backend (ARM Neon)           │
│  - GPU backend (Adreno)             │
│  - NPU backend (Hexagon)            │
├─────────────────────────────────────┤
│  Drivers (Hexagon DSP firmware)     │
└─────────────────────────────────────┘
```

**AIMET** (AI Model Efficiency Toolkit):
```python
from aimet_torch.quantsim import QuantizationSimModel

# Quantization-aware training
sim = QuantizationSimModel(
    model,
    dummy_input=torch.randn(1, 3, 224, 224),
    quant_scheme='tf_enhanced',  # Qualcomm-optimized
    default_param_bw=8,  # 8-bit weights
    default_output_bw=8  # 8-bit activations
)

# Fine-tune with quantization
sim.compute_encodings(forward_pass_callback, forward_pass_callback_args)
sim.model.train()
# ... training loop ...

# Export for Hexagon
sim.export(path='quantized_model', filename_prefix='model')
```

---

## Apple Silicon: Neural Engine Excellence

### A-Series (iPhone/iPad)

**A17 Pro (2023)**: iPhone 15 Pro

**Architecture**:
```
┌─────────────────────────────────────┐
│   A17 Pro (TSMC 3nm, 1st gen)      │
├─────────────────────────────────────┤
│  CPU:                               │
│  - 2× Performance cores (Avalanche) │
│  - 4× Efficiency cores (Blizzard)   │
├─────────────────────────────────────┤
│  GPU: 6-core (ray tracing)          │
├─────────────────────────────────────┤
│  Neural Engine: 16 cores            │
│  - 35 TOPS                          │
│  - INT8 precision                   │
│  - Unified memory architecture      │
├─────────────────────────────────────┤
│  Memory: Unified 8GB LPDDR5         │
│  ProRes/ProRAW accelerators         │
│  Secure Enclave (on-device AI)      │
└─────────────────────────────────────┘
```

**Neural Engine Specifications**:
- **Cores**: 16
- **Performance**: 35 TOPS (INT8)
- **Precision**: INT8 (primarily), INT16, FP16
- **Efficiency**: ~5.8 TOPS/W
- **Memory**: Shared with CPU/GPU (unified architecture)
- **Latency**: <5ms for typical models

**Unified Memory Advantage**:
```
Traditional Mobile SoC:
CPU Memory ↔ [Copy] ↔ NPU Memory
             ↕
         GPU Memory

Apple Unified Memory:
┌────────────────────────┐
│   Shared Memory Pool   │
└───┬─────┬────┬─────┬───┘
    │     │    │     │
   CPU   GPU  NPU  ISP

Benefits:
- Zero-copy data transfer
- Lower latency
- Reduced power (no memory copies)
- Larger effective memory for AI
```

**Performance Evolution**:
| Chip | Year | TOPS | Power | TOPS/W | Process |
|------|------|------|-------|--------|---------|
| A11 | 2017 | 0.6 | ~2W | 0.3 | 10nm |
| A12 | 2018 | 5.0 | ~3W | 1.67 | 7nm |
| A13 | 2019 | 6.0 | ~3W | 2.0 | 7nm+ |
| A14 | 2020 | 11.0 | ~4W | 2.75 | 5nm |
| A15 | 2021 | 15.8 | ~5W | 3.16 | 5nm |
| A16 | 2022 | 17.0 | ~5W | 3.4 | 4nm |
| A17 Pro | 2023 | 35.0 | ~6W | 5.8 | 3nm |

### M-Series (Mac)

**M4 (2024)**: Latest Mac chip

**Specifications**:
- **CPU**: Up to 10 cores (4P + 6E)
- **GPU**: Up to 10 cores
- **Neural Engine**: 16 cores, **38 TOPS**
- **Memory**: Up to 24GB unified
- **Process**: TSMC 3nm (2nd gen)
- **TDP**: 22W (full chip)

**M4 vs M3 vs M2**:
| Feature | M2 | M3 | M4 |
|---------|----|----|-----|
| CPU Cores | 8 | 8 | 10 |
| GPU Cores | 10 | 10 | 10 |
| Neural Engine TOPS | 15.8 | 18 | **38** |
| Memory Bandwidth | 100 GB/s | 100 GB/s | 120 GB/s |
| Process | 5nm | 3nm | 3nm (2nd) |

**Apple Intelligence Features** (requires M4/A17 Pro):
```
On-Device AI Capabilities:
- Writing tools (rewrite, summarize, proofread)
- Image generation (Genmoji, Image Playground)
- Smart notifications (priority sorting)
- Voice transcription and summarization
- Advanced photo editing

All processing: On-device (privacy-first)
No data sent to cloud
```

### Core ML: Apple's AI Framework

```swift
import CoreML

// Load model optimized for Neural Engine
guard let model = try? VNCoreMLModel(for: MyModel().model) else {
    fatalError("Failed to load Core ML model")
}

// Create request
let request = VNCoreMLRequest(model: model) { request, error in
    guard let results = request.results as? [VNClassificationObservation] else {
        return
    }
    // Process results
}

// Configure for Neural Engine
request.usesCPUOnly = false  // Allow Neural Engine

// Perform inference
let handler = VNImageRequestHandler(ciImage: image)
try? handler.perform([request])
```

**Model Optimization**:
```python
import coremltools as ct

# Convert PyTorch to Core ML
model = torch.load('model.pth')
traced_model = torch.jit.trace(model, example_input)

mlmodel = ct.convert(
    traced_model,
    inputs=[ct.TensorType(shape=(1, 3, 224, 224))],
    compute_units=ct.ComputeUnit.ALL  # Use Neural Engine + GPU
)

# Quantization for Neural Engine
mlmodel_quantized = ct.models.neural_network.quantization_utils.quantize_weights(
    mlmodel, nbits=8
)

mlmodel_quantized.save('model.mlmodel')
```

---

## MediaTek: Challenger in Mobile AI

### Dimensity 9300 (2023)

**Architecture**:
- **Process**: TSMC 4nm
- **CPU**: 4× Cortex-X4 + 4× Cortex-A720 (all big cores!)
- **GPU**: Immortalis-G720 MC12 (ray tracing)
- **NPU**: APU 790 (**45 TOPS**)

**APU (AI Processing Unit) Specifications**:
- Performance: 45 TOPS (INT8)
- Precision: INT4, INT8, FP16, BF16
- Architecture: 6th generation
- Mixed precision processing
- Power: 2-6W

**Innovation: Generative AI Focus**:
```
MediaTek APU optimized for:
- LLM inference (7B models)
- Image generation (Stable Diffusion)
- Real-time video AI effects
- Multimodal AI (text + image + audio)

Demo: Llama 2 7B running natively on phone
Speed: 8-12 tokens/second
```

**Software Stack**: NeuroPilot
- Supports TensorFlow Lite, PyTorch, ONNX
- Quantization tools
- Model zoo with optimized models

---

## Specialized Edge AI Chips

### Hailo-8: Purpose-Built Inference

**Target**: Smart cameras, robotics, industrial automation

**Specifications**:
- **Performance**: 26 TOPS
- **Power**: 2.5W
- **Form Factor**: M.2 module
- **Precision**: INT8
- **Price**: $70-100 per unit

**Architecture**: Dataflow architecture
```
No von Neumann bottleneck:
- Direct data flow between processing elements
- Minimal DRAM access
- Structured sparsity support

Performance:
ResNet-50: 970 fps at 2.5W
YOLOv5: 180 fps at 1080p
```

**Applications**:
- Smart city cameras (vehicle/person detection)
- Industrial inspection (defect detection)
- Retail analytics (customer counting, heatmaps)
- Agriculture (crop disease detection)

### Google Tensor (Pixel Phones)

**Tensor G3 (2023)**: Pixel 8 Pro

**Architecture**:
- **Process**: Samsung 4nm
- **CPU**: 1× Cortex-X3 + 4× A715 + 4× A510
- **GPU**: Mali-G715
- **TPU**: Google-designed **Tensor Processing Unit**
- **Image Signal Processor**: Custom

**TPU Specifications** (estimated):
- Performance: ~30 TOPS
- Optimized for: Google AI models
- On-device speech recognition
- Computational photography
- Magic Eraser, Best Take, Audio Magic Eraser

**Unique Features**:
```
Tight integration with Google services:
- On-device Google Assistant
- Live Translate (40+ languages)
- Call screening with AI
- Spam detection
- Real Tone (accurate skin tones in photos)
```

### Ambarella CV Series: Automotive Vision

**CV3-AD685 (2023)**: Autonomous driving AI

**Specifications**:
- **Performance**: 254 TOPS
- **Power**: 30W
- **CVflow Architecture**: Computer vision dataflow
- **ISP**: Up to 12 cameras (48MP each)
- **Safety**: ISO 26262 ASIL-B certified

**Target**: Autonomous vehicles (Level 2+/Level 3)
```
Sensor Fusion:
- 12× camera inputs
- 4× radar inputs
- 6× lidar inputs
- Real-time object detection, tracking, prediction

Tesla FSD Computer comparison:
- Tesla: 2× custom chips, 144 TOPS, 72W
- Ambarella CV3: Single chip, 254 TOPS, 30W
```

---

## Edge AI Optimization Techniques

### Model Compression

**1. Pruning**:
```python
import torch.nn.utils.prune as prune

# Prune 50% of weights
prune.l1_unstructured(module, name='weight', amount=0.5)

# Structured pruning (entire channels)
prune.ln_structured(module, name='weight', amount=0.3, n=2, dim=0)

Result: 2-5x smaller model, 10-30% accuracy impact
```

**2. Quantization**:
```python
# Post-training quantization (PTQ)
quantized_model = torch.quantization.quantize_dynamic(
    model, {torch.nn.Linear}, dtype=torch.qint8
)

# Quantization-aware training (QAT)
model.qconfig = torch.quantization.get_default_qat_qconfig('fbgemm')
model_prepared = torch.quantization.prepare_qat(model)
# Train...
model_quantized = torch.quantization.convert(model_prepared)

Result: 4x smaller (INT8), <1% accuracy loss
```

**3. Knowledge Distillation**:
```python
# Teacher model (large)
teacher_model = ResNet152()

# Student model (small)
student_model = MobileNetV3()

# Distillation loss
def distillation_loss(student_logits, teacher_logits, labels, temperature=3.0):
    soft_loss = F.kl_div(
        F.log_softmax(student_logits / temperature, dim=1),
        F.softmax(teacher_logits / temperature, dim=1),
        reduction='batchmean'
    ) * (temperature ** 2)

    hard_loss = F.cross_entropy(student_logits, labels)

    return 0.7 * soft_loss + 0.3 * hard_loss

Result: Small model with 90-95% of large model's accuracy
```

### Efficient Architectures

**MobileNet V3**:
```
Key Techniques:
- Depthwise separable convolutions (9x fewer parameters)
- Squeeze-and-Excitation blocks (adaptive channel weighting)
- Hard-swish activation (efficient on mobile)
- Network Architecture Search (NAS) optimized

Performance:
- Parameters: 5.4M (vs 25.6M for ResNet-50)
- Latency: 3.5ms on Snapdragon 8 Gen 3
- Top-1 Accuracy: 75.2% (ImageNet)
```

**EfficientNet**:
```
Compound Scaling:
- Width: Number of channels
- Depth: Number of layers
- Resolution: Input image size

Scale all three simultaneously with optimal ratios

EfficientNet-B0:
- Parameters: 5.3M
- FLOPs: 390M
- Top-1: 77.1% (better than larger models!)
```

---

## Power Management and Thermal Constraints

### Dynamic Voltage and Frequency Scaling (DVFS)

```
Edge devices must balance performance and battery life:

┌────────────────────────────────────┐
│  Workload Scenarios                │
├────────────────────────────────────┤
│  High Performance (camera open):   │
│  - NPU: 2.0 GHz, 1.0V              │
│  - Power: 6-8W                     │
│  - Duration: Seconds to minutes    │
├────────────────────────────────────┤
│  Balanced (voice assistant):       │
│  - NPU: 1.2 GHz, 0.8V              │
│  - Power: 2-3W                     │
│  - Duration: Minutes               │
├────────────────────────────────────┤
│  Low Power (always-on detection):  │
│  - NPU: 400 MHz, 0.6V              │
│  - Power: 50-200mW                 │
│  - Duration: Hours to days         │
└────────────────────────────────────┘
```

### Thermal Throttling

```python
# Simplified thermal management

def adjust_npu_frequency(temperature, workload):
    if temperature > 85:  # Critical
        return 0.4  # 40% max frequency
    elif temperature > 75:  # High
        return 0.6
    elif temperature > 65:  # Warm
        return 0.8
    else:  # Normal
        return 1.0

    # Also consider skin temperature (user comfort)
    # Throttle more aggressively if back of phone is hot
```

---

## Future Trends in Edge AI

### 1. On-Device LLMs

**Current State** (2025):
- 7B models running at 10-15 tokens/s
- Requires INT4 quantization + micro-tiling
- Limited to simpler tasks (chat, writing assistance)

**Near Future** (2026-2027):
- 13B models with better performance
- Multi-modal (text + vision + audio)
- Fine-tuned for personalization

### 2. Federated Learning

```
Train models across millions of devices without centralizing data:

1. Server sends initial model
2. Each device trains on local data
3. Devices send only model updates (not data)
4. Server aggregates updates
5. Improved model distributed back

Privacy preserved, better models
```

### 3. Neuromorphic Edge Chips

**Spiking Neural Networks** (SNNs):
- Event-driven computation
- 100-1000x power efficiency potential
- Ultra-low latency (<1ms)

**Examples**:
- Intel Loihi 2 (research)
- BrainChip Akida (commercial)
- IBM TrueNorth (retired)

### 4. Photonic AI Accelerators

```
Light-based computation:
- Near-zero energy for data movement
- Massively parallel
- TeraHz speed potential

Timeline: 2027-2030 for commercial products
```

---

## Conclusion

Edge AI processors represent the future of ubiquitous intelligence:

**Key Takeaways**:
1. **Qualcomm leads** mobile AI (45% market share)
2. **Apple excels** in efficiency and integration
3. **MediaTek challenges** with aggressive specs
4. **Specialized chips** (Hailo, Ambarella) dominate niches

**For Developers**:
- Optimize models for INT8/INT4
- Use vendor-specific SDKs for best performance
- Balance accuracy vs latency vs power
- Test on real devices, not just benchmarks

**For Product Managers**:
- On-device AI is competitive advantage
- Privacy features resonate with users
- Battery life cannot be compromised
- Differentiate through unique AI features

**Market Outlook**:
- 35%+ CAGR through 2030
- Every smartphone will have 50+ TOPS by 2026
- Edge AI enables new product categories
- Privacy regulations favor on-device processing

Next chapter: Quantization techniques (INT8/FP16 precision and model optimization).

---

*© 2025 SmileStory Inc. / WIA*
*弘益人間 (Hongik Ingan) · Benefit All Humanity*
