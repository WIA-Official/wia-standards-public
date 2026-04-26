# Chapter 7: Quantization Techniques - INT8/FP16 Precision and Model Optimization

## From 32-bit Precision to 4-bit Inference

Quantization is the process of reducing the numerical precision of model weights and activations from floating-point (FP32) to lower-bit representations (INT8, INT4, FP16). This chapter explores quantization theory, practical techniques, and their impact on AI chip performance and accuracy.

---

## Why Quantization Matters

### The Memory and Compute Problem

**Model Size Growth**:
```
2018: BERT-Base (110M parameters) = 440MB (FP32)
2020: GPT-3 (175B parameters) = 700GB (FP32)
2023: PaLM 2 (340B parameters) = 1.36TB (FP32)
2024: GPT-4 (estimated 1.8T parameters) = 7.2TB (FP32)

Problem: Models don't fit in GPU memory!
```

**Quantization Impact**:
| Precision | Model Size | Memory Bandwidth | Relative Speed |
|-----------|------------|------------------|----------------|
| FP32 | 100% | 100% | 1x |
| FP16 | 50% | 50% | 2x |
| INT8 | 25% | 25% | 4x |
| INT4 | 12.5% | 12.5% | 8x |

**Real Example (LLaMA 2 70B)**:
```
FP32: 280GB (doesn't fit on 8× A100 80GB)
FP16: 140GB (fits on 2× A100 80GB) ✓
INT8: 70GB (fits on 1× A100 80GB) ✓✓
INT4: 35GB (fits on 1× A100 40GB) ✓✓✓

INT4 quantization: 8x reduction, <2% accuracy loss
```

---

## Precision Formats

### Floating-Point Formats

**FP32 (IEEE 754 Single Precision)**:
```
[Sign: 1 bit][Exponent: 8 bits][Mantissa: 23 bits]

Range: ±3.4 × 10^38
Precision: ~7 decimal digits
Example: 3.141592653589793 → 3.1415927 (rounded)
```

**FP16 (Half Precision)**:
```
[Sign: 1 bit][Exponent: 5 bits][Mantissa: 10 bits]

Range: ±65,504
Precision: ~3 decimal digits
Problem: Limited range can cause overflow

Example:
FP32: 65536.0 → FP16: Inf (overflow!)
```

**bfloat16 (Brain Float)**:
```
[Sign: 1 bit][Exponent: 8 bits][Mantissa: 7 bits]

Range: Same as FP32 (±3.4 × 10^38)
Precision: Reduced but acceptable
Advantage: Easy conversion from FP32 (truncate mantissa)

Google's TPU, NVIDIA's A100/H100 support bfloat16
```

**FP8 (NVIDIA Hopper)**:
```
Two formats:

E4M3: [Sign: 1][Exponent: 4][Mantissa: 3]
- Range: ±448
- Better for forward pass

E5M2: [Sign: 1][Exponent: 5][Mantissa: 2]
- Range: ±57,344
- Better for gradients (backprop)

Automatic switching in NVIDIA Transformer Engine
```

### Integer Formats

**INT8 (8-bit Integer)**:
```
Signed: -128 to 127
Unsigned: 0 to 255

Quantization mapping:
Real value range [-2.5, 3.5] → INT8 [0, 255]

Scale = (3.5 - (-2.5)) / 255 = 0.0235
Zero_point = -round(-2.5 / 0.0235) = 106

Quantize(1.5) = round(1.5 / 0.0235) + 106 = 170
Dequantize(170) = (170 - 106) × 0.0235 = 1.504
```

**INT4 (4-bit Integer)**:
```
Signed: -8 to 7
Unsigned: 0 to 15

Extreme compression for LLM inference
Requires careful calibration to minimize accuracy loss
```

---

## Quantization Methods

### 1. Post-Training Quantization (PTQ)

**Static PTQ**:
```python
import tensorflow as tf

# Load trained model
model = tf.keras.models.load_model('model.h5')

# Convert to TFLite with quantization
converter = tf.lite.TFLiteConverter.from_keras_model(model)

# Configuration
converter.optimizations = [tf.lite.Optimize.DEFAULT]
converter.representative_dataset = representative_data_gen

# Target INT8
converter.target_spec.supported_ops = [
    tf.lite.OpsSet.TFLITE_BUILTINS_INT8
]
converter.inference_input_type = tf.int8
converter.inference_output_type = tf.int8

# Convert
quantized_model = converter.convert()

# Save
with open('model_quantized.tflite', 'wb') as f:
    f.write(quantized_model)
```

**Dynamic PTQ** (PyTorch):
```python
import torch

model = torch.load('model.pth')

# Dynamic quantization (activations quantized at runtime)
quantized_model = torch.quantization.quantize_dynamic(
    model,
    {torch.nn.Linear, torch.nn.Conv2d},  # Layers to quantize
    dtype=torch.qint8
)

# Save
torch.save(quantized_model.state_dict(), 'model_quantized.pth')

# Result: 4x smaller, 2-3x faster inference
```

**Accuracy Impact**:
| Model | FP32 Acc | INT8 PTQ Acc | Degradation |
|-------|----------|--------------|-------------|
| ResNet-50 | 76.1% | 75.8% | -0.3% |
| MobileNet V2 | 72.0% | 71.2% | -0.8% |
| BERT-Base | 84.5% | 83.9% | -0.6% |
| YOLOv5 | 37.2 mAP | 36.5 mAP | -0.7% |

### 2. Quantization-Aware Training (QAT)

**Concept**: Simulate quantization during training
```
Forward pass:
1. Weights → Quantize → Dequantize → Use in computation
2. Activations → Quantize → Dequantize → Next layer

Backward pass:
- Gradients flow through dequantized values
- Model learns to be robust to quantization noise

Result: Minimal accuracy loss vs FP32
```

**PyTorch QAT**:
```python
import torch
import torch.quantization as quantization

model = MyModel()

# Prepare model for QAT
model.qconfig = quantization.get_default_qat_qconfig('fbgemm')
model_prepared = quantization.prepare_qat(model)

# Training loop with quantization simulation
for epoch in range(num_epochs):
    for data, target in train_loader:
        optimizer.zero_grad()
        output = model_prepared(data)
        loss = criterion(output, target)
        loss.backward()
        optimizer.step()

# Convert to actual quantized model
quantized_model = quantization.convert(model_prepared)

# Save
torch.jit.save(torch.jit.script(quantized_model), 'model_qat.pt')
```

**Accuracy Comparison**:
| Model | FP32 | INT8 PTQ | INT8 QAT |
|-------|------|----------|----------|
| ResNet-50 | 76.1% | 75.8% (-0.3%) | 76.0% (-0.1%) |
| MobileNet V2 | 72.0% | 71.2% (-0.8%) | 71.9% (-0.1%) |
| BERT-Base | 84.5% | 83.9% (-0.6%) | 84.4% (-0.1%) |

**Trade-off**: QAT requires retraining but achieves better accuracy.

### 3. Mixed Precision

**Strategy**: Use different precisions for different layers
```
Precision Selection:
┌────────────────────────────────┐
│  Input (Image)                 │
│  ↓ INT8 (minimal info loss)    │
│  First Conv Layer              │
│  ↓ INT8 (bulk computation)     │
│  Middle Layers                 │
│  ↓ INT8                        │
│  Last Few Layers               │
│  ↓ INT16 (preserve precision)  │
│  Fully Connected               │
│  ↓ FP16 (softmax sensitive)    │
│  Output (Probabilities)        │
└────────────────────────────────┘
```

**NVIDIA Automatic Mixed Precision (AMP)**:
```python
import torch
from torch.cuda.amp import autocast, GradScaler

model = MyModel().cuda()
optimizer = torch.optim.Adam(model.parameters())
scaler = GradScaler()

for data, target in train_loader:
    optimizer.zero_grad()

    # Automatic mixed precision
    with autocast():
        output = model(data)
        loss = criterion(output, target)

    # Scale loss to prevent underflow
    scaler.scale(loss).backward()
    scaler.step(optimizer)
    scaler.update()

# Result: 2-3x faster training, same accuracy as FP32
```

---

## Advanced Quantization Techniques

### 1. Per-Channel Quantization

**Problem**: Different channels have different value distributions
```
Weights shape: [Out_channels, In_channels, H, W]

Per-tensor quantization:
- Single scale/zero_point for entire tensor
- Suboptimal for channels with different ranges

Per-channel quantization:
- Scale[i] and zero_point[i] for each output channel
- Better accuracy, minimal overhead
```

**Implementation**:
```python
# PyTorch per-channel quantization
qconfig = torch.quantization.QConfig(
    activation=torch.quantization.default_observer,
    weight=torch.quantization.per_channel_min_max_observer
)

model.qconfig = qconfig
```

**Accuracy Improvement**: 0.5-1.5% over per-tensor quantization.

### 2. GPTQ (Generative Pre-trained Transformer Quantization)

**For Large Language Models**:
```python
from auto_gptq import AutoGPTQForCausalLM, BaseQuantizeConfig

# Quantization configuration
quantize_config = BaseQuantizeConfig(
    bits=4,  # INT4 quantization
    group_size=128,  # Group weights for better precision
    desc_act=False,  # Activation ordering
)

# Load and quantize model
model = AutoGPTQForCausalLM.from_pretrained(
    "meta-llama/Llama-2-7b-hf",
    quantize_config=quantize_config
)

# Calibrate with sample data
model.quantize(calibration_data)

# Save
model.save_quantized("llama-2-7b-gptq-4bit")

# Result: 4-bit LLaMA 2 7B (3.5GB vs 14GB FP16)
# Accuracy: 95-98% of FP16 quality
```

**Performance**:
| Model | FP16 Size | INT4 Size | Perplexity (lower is better) |
|-------|-----------|-----------|------------------------------|
| LLaMA 2 7B | 14GB | 3.5GB | 5.68 (FP16) vs 5.82 (INT4) |
| LLaMA 2 13B | 26GB | 6.5GB | 5.09 vs 5.18 |

### 3. GGUF/GGML (Georgi Gerganov Quantization)

**Popular for local LLM deployment**:
```
Quantization schemes:
- Q4_0: 4-bit weights, no zero-point
- Q4_K_M: 4-bit with mixed precision
- Q5_K_M: 5-bit mixed precision
- Q8_0: 8-bit quantization

Trade-off: Quality vs size
```

**Example** (LLaMA 2 7B):
```
Original FP16: 13.5 GB

Q2_K: 3.08 GB (perplexity: 8.27) - Poor quality
Q4_K_M: 4.37 GB (perplexity: 5.82) - Good balance ✓
Q5_K_M: 5.33 GB (perplexity: 5.72) - Better quality
Q8_0: 7.87 GB (perplexity: 5.69) - Near-FP16 quality
```

### 4. LLM.int8() (8-bit Matrix Multiplication)

**Technique** (from HuggingFace bitsandbytes):
```python
from transformers import AutoModelForCausalLM

# Load model with 8-bit quantization
model = AutoModelForCausalLM.from_pretrained(
    "meta-llama/Llama-2-70b-hf",
    device_map="auto",
    load_in_8bit=True  # Enable 8-bit quantization
)

# How it works:
# 1. Separate outlier values (>6σ) - keep in FP16
# 2. Quantize normal values to INT8
# 3. Perform matrix multiplication
# 4. Combine results

# Result: 70B model fits in 40GB (vs 140GB in FP16)
# Accuracy: 99.5% of FP16 quality
```

---

## Hardware Support for Quantization

### NVIDIA Tensor Cores

**INT8 Tensor Cores** (A100/H100):
```
Matrix multiplication: C = A × B + C

A: INT8 [M×K]
B: INT8 [K×N]
C: INT32 [M×N] (accumulator)

Throughput:
- FP16: 312 TFLOPS (A100)
- INT8: 624 TOPS (A100) - 2x faster
- FP8: 2000 TFLOPS (H100) - 6.4x faster than A100 FP16
```

**Transformer Engine** (H100):
```
Automatic FP8 quantization:

Per-layer analysis:
1. Measure activation/gradient ranges
2. Select FP8 E4M3 (forward) or E5M2 (backward)
3. Apply per-tensor scaling
4. Accumulate in FP16/FP32

No manual tuning required
Result: 2x faster training, same accuracy
```

### Qualcomm Hexagon NPU

**INT4/INT8 Acceleration**:
```
Hexagon Tensor Accelerator:
- Native INT4, INT8, INT16 support
- 2× throughput for INT4 vs INT8
- 4× throughput for INT4 vs INT16

Use case: On-device LLM inference
- LLaMA 2 7B in INT4
- 11 tokens/second on Snapdragon 8 Gen 3
```

### Apple Neural Engine

**INT8-Optimized**:
```
A17 Pro Neural Engine:
- Primary: INT8
- Secondary: INT16 for sensitive layers
- 35 TOPS (INT8)

Core ML automatic quantization:
- Analyzes model sensitivity
- Selects INT8 or INT16 per-layer
- Maintains >99% accuracy
```

---

## Quantization Challenges and Solutions

### Challenge 1: Activation Outliers

**Problem**: Large outlier values cause quantization error
```
Activation distribution:
99% of values: [-1.0, 1.0]
1% of values: [-100, 100] (outliers)

Naive quantization:
Scale = 200 / 255 = 0.784
Result: Poor utilization of INT8 range
```

**Solution 1: Clipping**
```python
# Clip outliers to percentile
def clip_quantize(x, percentile=99.9):
    min_val = np.percentile(x, 100 - percentile)
    max_val = np.percentile(x, percentile)
    x_clipped = np.clip(x, min_val, max_val)
    return quantize(x_clipped)

# Trade-off: Slight accuracy loss, better quantization
```

**Solution 2: LLM.int8() Approach**
```python
# Separate outliers
outliers = x[abs(x) > threshold]  # Keep in FP16
normal = x[abs(x) <= threshold]   # Quantize to INT8

# Compute separately and combine
output = matmul_int8(normal) + matmul_fp16(outliers)
```

### Challenge 2: Batch Normalization

**Problem**: BatchNorm statistics change during quantization
```
BatchNorm:
y = (x - mean) / sqrt(var + ε) * γ + β

During training: mean/var computed per-batch
During inference: mean/var are fixed (running average)

Quantization affects these statistics
```

**Solution: Fold BatchNorm into Convolution**
```python
def fuse_conv_bn(conv, bn):
    # Extract parameters
    w = conv.weight
    b = conv.bias if conv.bias is not None else 0

    gamma = bn.weight
    beta = bn.bias
    mean = bn.running_mean
    var = bn.running_var
    eps = bn.eps

    # Fused weights
    scale = gamma / torch.sqrt(var + eps)
    w_fused = w * scale.view(-1, 1, 1, 1)
    b_fused = (b - mean) * scale + beta

    # Create new conv layer
    conv_fused = nn.Conv2d(...)
    conv_fused.weight = w_fused
    conv_fused.bias = b_fused

    return conv_fused

# Result: Conv+BN → Single conv (easier to quantize)
```

### Challenge 3: Per-Token Dynamic Range (LLMs)

**Problem**: Each token has different activation range
```
Sentence: "The AI model is powerful"
Token "AI": Activations in range [-2, 5]
Token "powerful": Activations in range [-10, 15]

Static quantization: Suboptimal for both
```

**Solution: Dynamic Quantization**
```python
# Quantize each token's activations independently
for token in sequence:
    min_val, max_val = token.activations.min(), token.activations.max()
    scale = (max_val - min_val) / 255
    zero_point = -min_val / scale

    quantized = (token.activations / scale + zero_point).round().clip(0, 255)
    # ... compute with quantized values ...

# Overhead: Compute scale/zero_point per token
# Benefit: Much better accuracy
```

---

## Practical Quantization Workflow

### Step-by-Step Guide

**1. Baseline FP32 Model**
```python
# Train FP32 model
model = train_model(data)
fp32_accuracy = evaluate(model, test_data)

print(f"FP32 Accuracy: {fp32_accuracy}")
# Example: 76.5%
```

**2. Post-Training Quantization (Quick Test)**
```python
# Try PTQ INT8
quantized_model = torch.quantization.quantize_dynamic(
    model, {torch.nn.Linear}, dtype=torch.qint8
)

ptq_accuracy = evaluate(quantized_model, test_data)
degradation = fp32_accuracy - ptq_accuracy

print(f"INT8 PTQ Accuracy: {ptq_accuracy}")
print(f"Degradation: {degradation}")
# Example: 75.8%, degradation: 0.7%

if degradation < 1.0:
    print("PTQ acceptable, deploy!")
else:
    print("Need QAT or mixed precision")
```

**3. Quantization-Aware Training (If Needed)**
```python
# If PTQ degradation > 1%, use QAT
model.qconfig = torch.quantization.get_default_qat_qconfig('fbgemm')
model_qat = torch.quantization.prepare_qat(model)

# Fine-tune for 5-10 epochs
for epoch in range(5):
    train_one_epoch(model_qat, train_data)

model_quantized = torch.quantization.convert(model_qat)
qat_accuracy = evaluate(model_quantized, test_data)

print(f"INT8 QAT Accuracy: {qat_accuracy}")
# Example: 76.3%, degradation: 0.2% ✓
```

**4. Benchmark Performance**
```python
import time

# Measure latency
for _ in range(100):  # Warmup
    model(sample_input)

start = time.time()
for _ in range(1000):
    model_quantized(sample_input)
end = time.time()

latency_quantized = (end - start) / 1000

print(f"FP32 Latency: {latency_fp32*1000:.2f}ms")
print(f"INT8 Latency: {latency_quantized*1000:.2f}ms")
print(f"Speedup: {latency_fp32 / latency_quantized:.2f}x")
# Example: FP32: 25ms, INT8: 6.8ms, Speedup: 3.7x
```

---

## Quantization for Different Hardware

### NVIDIA GPUs (TensorRT)
```python
import tensorrt as trt

# INT8 calibration
calibrator = MyCalibrator(calibration_data)

builder = trt.Builder(logger)
config = builder.create_builder_config()
config.set_flag(trt.BuilderFlag.INT8)
config.int8_calibrator = calibrator

engine = builder.build_engine(network, config)

# Result: Optimized INT8 engine for NVIDIA GPU
```

### Qualcomm (SNPE)
```bash
# Convert and quantize for Hexagon NPU
snpe-dlc-quantize \
  --input_dlc model_fp32.dlc \
  --input_list calibration_data.txt \
  --output_dlc model_quantized.dlc \
  --use_enhanced_quantizer \
  --use_per_channel_quantization
```

### Apple (Core ML)
```python
import coremltools as ct

# Convert with quantization
mlmodel = ct.convert(
    torch_model,
    inputs=[ct.TensorType(shape=(1, 3, 224, 224))],
    compute_units=ct.ComputeUnit.ALL
)

# Quantize
mlmodel_int8 = ct.models.neural_network.quantization_utils.quantize_weights(
    mlmodel, nbits=8, quantization_mode='linear'
)
```

---

## Conclusion

Quantization is essential for deploying AI models:

**Key Takeaways**:
1. **INT8 is standard** for production inference (4x speedup, <1% accuracy loss)
2. **INT4 enables** large models on edge devices (LLMs on smartphones)
3. **QAT beats PTQ** for accuracy but requires retraining
4. **Mixed precision** balances performance and quality
5. **Hardware support** critical (Tensor Cores, NPUs)

**Best Practices**:
- Start with PTQ, use QAT if needed
- Per-channel quantization for better accuracy
- Calibration data should represent real distribution
- Always measure accuracy AND latency
- Test on target hardware

**Future**: FP4, adaptive precision, automated quantization.

Next chapter: Transformer acceleration and LLM optimization.

---

*© 2025 SmileStory Inc. / WIA*
*弘益人間 (Hongik Ingan) · Benefit All Humanity*
