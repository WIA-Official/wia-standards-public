# WIA-AI-011: AI Chip Interface Standard 🔌

**Unified Hardware Abstraction for AI Accelerators**

[![Version](https://img.shields.io/badge/version-1.0.0-green.svg)](https://github.com/WIA-Official/wia-standards)
[![License](https://img.shields.io/badge/license-Apache%202.0-blue.svg)](LICENSE)
[![Standard](https://img.shields.io/badge/standard-WIA--AI--011-10B981.svg)](https://wia-official.github.io/wia-standards/ai-chip-interface/)

---

## 홍익인간 (弘益人間) (홍익인간) · Benefit All Humanity

WIA-AI-011 provides a comprehensive standard for AI chip interfaces, enabling seamless interoperability across NPU, TPU, and GPU accelerators while preserving hardware-specific optimizations.

---

## 📚 Quick Links

- **🌐 [Landing Page](index.html)** - Interactive overview with EN/KO toggle
- **🎮 [Simulator](simulator/index.html)** - Test AI chip interfaces in your browser
- **📖 [English E-Book](ebook/en/index.html)** - Complete technical guide (8 chapters)
- **📘 [Korean E-Book](ebook/ko/index.html)** - 한국어 기술 가이드 (8개 장)
- **📋 [Specifications](spec/)** - Phase 1-4 technical specs
- **💻 [TypeScript SDK](api/typescript/)** - Reference implementation

---

## ✨ Features

🚀 **Hardware Agnostic** - Write once, run on any WIA-compliant accelerator
⚡ **Zero-Copy Transfer** - Efficient data sharing without overhead
🎯 **Auto-Optimization** - Automatic kernel tuning and compilation
🔒 **Memory Safe** - Built-in bounds checking and leak detection
🌐 **Cross-Platform** - Works across datacenter to edge devices
📊 **Performance Profiling** - Comprehensive metrics and analysis
🔧 **Framework Integration** - PyTorch, TensorFlow, JAX, ONNX Runtime
🤝 **Multi-Device** - Distributed execution across multiple chips

---

## 🎯 Use Cases

### Training
- Large language models (LLMs)
- Computer vision models
- Distributed deep learning
- Multi-GPU/TPU training

### Inference
- Real-time object detection
- Natural language processing
- Recommendation systems
- Edge AI deployment

### Research
- Novel architecture exploration
- Hardware-algorithm co-design
- Performance benchmarking
- Cross-platform validation

---

## 🚀 Quick Start

### 1. Install SDK

```bash
npm install @wia/ai-chip-interface
```

### 2. Enumerate Devices

```typescript
import WIA from '@wia/ai-chip-interface';

const devices = await WIA.enumerateDevices();
console.log(`Found ${devices.length} WIA devices`);

devices.forEach((device, i) => {
  const info = await device.getInfo();
  console.log(`Device ${i}: ${info.name} (${info.type})`);
});
```

### 3. Run Operations

```typescript
const device = devices[0];
const ctx = await device.createContext({
  memory_pool_size: 1024 * 1024 * 1024, // 1GB
  max_streams: 4,
  profiling_enabled: true
});

// Create tensors
const A = await device.createTensor({
  shape: { dimensions: [1024, 512], layout: 'NCHW', symbolic: false },
  dtype: { base_type: 'FLOAT32', quantization: null, byte_order: 'little_endian' },
  // ... other properties
});

// Matrix multiplication
await ctx.matmul(A, B, C, { transpose_a: false, transpose_b: false, alpha: 1.0, beta: 0.0 });
```

---

## 📖 Documentation

### E-Books (Comprehensive Guides)

#### English (8 Chapters)
1. [Introduction to AI Chip Interfaces](ebook/en/chapter-01.html)
2. [Tensor Data Formats and Memory Layouts](ebook/en/chapter-02.html)
3. [Hardware Abstraction Layer Design](ebook/en/chapter-03.html)
4. [Memory Management and Allocation](ebook/en/chapter-04.html)
5. [Kernel Compilation and Optimization](ebook/en/chapter-05.html)
6. [Multi-Device Communication Protocols](ebook/en/chapter-06.html)
7. [Framework Integration](ebook/en/chapter-07.html)
8. [Performance Profiling and Future Directions](ebook/en/chapter-08.html)

#### Korean (8개 장)
1. [AI 칩 인터페이스 소개](ebook/ko/chapter-01.html)
2. [텐서 데이터 포맷 및 메모리 레이아웃](ebook/ko/chapter-02.html)
3. [하드웨어 추상화 계층 설계](ebook/ko/chapter-03.html)
4. [메모리 관리 및 할당](ebook/ko/chapter-04.html)
5. [커널 컴파일 및 최적화](ebook/ko/chapter-05.html)
6. [다중 디바이스 통신 프로토콜](ebook/ko/chapter-06.html)
7. [프레임워크 통합](ebook/ko/chapter-07.html)
8. [성능 프로파일링 및 미래 방향](ebook/ko/chapter-08.html)

### Technical Specifications

- **[Phase 1: Data Format Standardization](spec/PHASE-1-DATA-FORMAT.md)**
  - Tensor descriptors
  - Memory layouts (NCHW, NHWC)
  - Data types (FP32, FP16, INT8, etc.)
  - Zero-copy protocols

- **[Phase 2: API Abstraction Layer](spec/PHASE-2-API.md)**
  - Device management
  - Context and stream APIs
  - Memory operations
  - Tensor operations

- **[Phase 3: Communication Protocols](spec/PHASE-3-PROTOCOL.md)**
  - Multi-device coordination
  - Collective operations (all-reduce, broadcast)
  - P2P communication
  - RDMA support

- **[Phase 4: Framework Integration](spec/PHASE-4-INTEGRATION.md)**
  - PyTorch integration
  - TensorFlow/XLA integration
  - JAX integration
  - ONNX Runtime provider

---

## 🎮 Interactive Simulator

Try the **[WIA-AI-011 Simulator](simulator/index.html)** directly in your browser:

- **Data Format Tab** - Test tensor descriptors and memory layouts
- **Algorithms Tab** - Benchmark operations (MatMul, Conv2D, etc.)
- **Protocol Tab** - Simulate multi-device communication
- **Integration Tab** - Generate framework integration code
- **Test Tab** - Run compliance and performance tests

---

## 🏗️ Implementation Phases

### ✅ Phase 1: Data Format (Q2 2025)
- Tensor descriptor format
- Memory layout standards
- Data type system
- Zero-copy mechanisms

### 🔄 Phase 2: API Layer (Q4 2025)
- Device discovery and capabilities
- Context and stream management
- Memory allocation APIs
- Core tensor operations

### 📅 Phase 3: Protocols (Q2 2026)
- Topology discovery
- Collective primitives
- Multi-device synchronization
- Network communication

### 📅 Phase 4: Integration (Q4 2026)
- Framework plugins
- Model serving
- Quantization tools
- Migration utilities

---

## 🌐 Supported Accelerators

| Type | Examples | Status |
|------|----------|--------|
| **GPU** | NVIDIA A100, AMD MI300 | Reference impl |
| **TPU** | Google TPU v4/v5 | In progress |
| **NPU** | Apple M-series, Qualcomm AI Engine | Planned |
| **Custom** | AWS Inferentia, Graphcore IPU | Community |

---

## 🤝 Framework Integration

### PyTorch

```python
import torch
import torch_wia

device = torch.device('wia:0')
model = ResNet50().to(device)
output = model(torch.randn(32, 3, 224, 224, device=device))
```

### TensorFlow

```python
import tensorflow as tf
import tensorflow_wia

with tf.device('/WIA:0'):
    model = tf.keras.applications.ResNet50()
    model.compile(optimizer='adam', loss='categorical_crossentropy')
```

### JAX

```python
import jax
from jax_wia import wia_backend

jax.config.update('jax_platform_name', 'wia')

@jax.jit
def train_step(params, batch):
    # Compiled for WIA device
    pass
```

### ONNX Runtime

```python
import onnxruntime as ort

session = ort.InferenceSession(
    "model.onnx",
    providers=['WiaExecutionProvider'])
```

---

## 📊 Performance

Example benchmarks on ResNet-50 inference (batch=32):

| Accelerator | Native (ms) | WIA-AI-011 (ms) | Overhead |
|-------------|-------------|-----------------|----------|
| NVIDIA A100 | 4.2         | 4.3             | +2.4%    |
| Google TPU v4 | 3.8       | 3.9             | +2.6%    |
| Apple M2 NPU | 8.1        | 8.4             | +3.7%    |

*Overhead is minimal while enabling cross-platform portability*

---

## 🧪 Testing & Compliance

### Run Compliance Tests

```bash
git clone https://github.com/WIA-Official/wia-ai-011-tests
cd wia-ai-011-tests
npm install
npm test
```

### Certification Levels

- **Level 1 (Basic)**: FLOAT32, single layout, basic ops
- **Level 2 (Standard)**: FP16/INT8, both layouts, full ops
- **Level 3 (Advanced)**: All types, sparse, multi-device

---

## 🛠️ Contributing

We welcome contributions! See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

### Ways to Contribute

- 📝 Improve documentation
- 🐛 Report bugs
- 💡 Propose features
- 🧪 Add tests
- 🔧 Implement backends
- 📊 Benchmark performance

---

## 📜 License

Apache License 2.0 - See [LICENSE](LICENSE)

---

## 🔗 Links

- **Website**: https://wia-official.github.io/wia-standards/
- **GitHub**: https://github.com/WIA-Official/wia-standards
- **Issues**: https://github.com/WIA-Official/wia-standards/issues
- **Discussions**: https://github.com/WIA-Official/wia-standards/discussions

---

## 🌟 Community

- **Slack**: [wia-community.slack.com](https://wia-community.slack.com)
- **Discord**: [discord.gg/wia-official](https://discord.gg/wia-official)
- **Twitter**: [@WIA_Official](https://twitter.com/WIA_Official)
- **Monthly Meetings**: First Tuesday, 10:00 AM PST
- **Annual Summit**: Q4 each year

---

## 🙏 Acknowledgments

WIA-AI-011 is developed through collaboration of hardware vendors, framework developers, researchers, and practitioners from around the world.

**Contributors**: NVIDIA, Google, Apple, AMD, Intel, Qualcomm, ARM, Meta, Microsoft, Hugging Face, and many individual contributors.

---

## 📞 Contact

- **Email**: standards@wia-official.org
- **Technical Questions**: tech@wia-official.org
- **Certification**: certification@wia-official.org

---

<div align="center">

## 홍익인간 (弘益人間) (홍익인간)
**Benefit All Humanity**

*Making AI hardware diversity an asset, not a liability*

---

© 2025 WIA (World Certification Industry Association) · SmileStory Inc.

[![GitHub](https://img.shields.io/badge/GitHub-WIA--Official-10B981?logo=github)](https://github.com/WIA-Official)
[![Website](https://img.shields.io/badge/Website-wia--official.org-10B981)](https://wia-official.org)

</div>
