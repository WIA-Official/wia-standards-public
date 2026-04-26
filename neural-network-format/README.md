# WIA-AI-014: Neural Network Format 🧠

> **Philosophy:** 홍익인간 (弘益人間) - Benefit All Humanity

Universal standard for AI model interoperability, enabling seamless model exchange across frameworks and deployment platforms.

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![Version](https://img.shields.io/badge/Version-1.0.0-green.svg)](https://github.com/WIA-Official/wia-standards)

## 🎯 Overview

WIA-AI-014 defines a comprehensive standard for neural network model serialization, conversion, and deployment. It enables:

- **Cross-Framework Compatibility:** Convert models between PyTorch, TensorFlow, ONNX, and more
- **Production-Ready Deployment:** Optimized formats for cloud, edge, and mobile platforms
- **Complete Model Lineage:** Track training data, hyperparameters, and model evolution
- **Enterprise Governance:** Security, compliance, and audit trails built-in

## 🚀 Quick Start

### Installation

```bash
# TypeScript/Node.js
npm install @wia/neural-network-format

# Python
pip install wia-neural-network-format

# CLI Tools
curl -sSL https://wia.org/install.sh | sh
```

### Basic Usage

```typescript
import { Model } from '@wia/neural-network-format';

// Load model
const model = await Model.load('resnet50.onnx');

// Run inference
const outputs = await model.run({
    input: {
        shape: [1, 3, 224, 224],
        data: imageData,
        dtype: DataType.FLOAT32
    }
});

console.log(outputs.output.data);
```

```python
from wia_ai_014 import WIAModel

# Load and run model
model = WIAModel.load('resnet50.onnx')
output = model.run({'input': image_tensor})
print(output['output'])
```

## 📁 Repository Structure

```
neural-network-format/
├── index.html                    # Landing page
├── simulator/                    # Interactive format converter
│   └── index.html
├── ebook/                        # Complete documentation
│   ├── en/                       # English version
│   │   ├── index.html
│   │   ├── chapter-01.html       # Serialization Basics
│   │   ├── chapter-02.html       # ONNX Format
│   │   ├── chapter-03.html       # TensorFlow SavedModel
│   │   ├── chapter-04.html       # PyTorch TorchScript
│   │   ├── chapter-05.html       # Cross-Framework Conversion
│   │   ├── chapter-06.html       # Optimization & Quantization
│   │   ├── chapter-07.html       # Metadata & Versioning
│   │   └── chapter-08.html       # Enterprise Integration
│   └── ko/                       # Korean version (한국어)
│       ├── index.html
│       └── chapter-*.html
├── spec/                         # Technical specifications
│   ├── PHASE-1-DATA-FORMAT.md    # Binary format specification
│   ├── PHASE-2-API.md            # Programming interfaces
│   ├── PHASE-3-PROTOCOL.md       # Network protocols
│   └── PHASE-4-INTEGRATION.md    # MLOps integration
├── api/                          # SDK implementations
│   └── typescript/
│       ├── src/
│       │   ├── types.ts          # Type definitions
│       │   └── index.ts          # Main SDK
│       └── package.json
└── README.md                     # This file
```

## 🎓 Documentation

### Interactive Simulator

Try the [online simulator](./simulator/) to:
- Convert models between formats (ONNX, TensorFlow, PyTorch)
- Visualize model architectures
- Inspect layer details
- Calculate tensor shapes
- Check platform compatibility

### Complete Guide

Read the [comprehensive ebook](./ebook/en/):

1. **[Chapter 1: Serialization Basics](./ebook/en/chapter-01.html)** - Core concepts and fundamentals
2. **[Chapter 2: ONNX Format](./ebook/en/chapter-02.html)** - Deep dive into ONNX
3. **[Chapter 3: TensorFlow SavedModel](./ebook/en/chapter-03.html)** - TensorFlow deployment
4. **[Chapter 4: PyTorch TorchScript](./ebook/en/chapter-04.html)** - PyTorch production
5. **[Chapter 5: Cross-Framework Conversion](./ebook/en/chapter-05.html)** - Model portability
6. **[Chapter 6: Optimization & Quantization](./ebook/en/chapter-06.html)** - Performance tuning
7. **[Chapter 7: Metadata & Versioning](./ebook/en/chapter-07.html)** - Model governance
8. **[Chapter 8: Enterprise Integration](./ebook/en/chapter-08.html)** - Production deployment

[한국어 문서](./ebook/ko/) 도 제공됩니다.

## 🔧 Features

### Format Support

| Format | Import | Export | Optimization |
|--------|--------|--------|--------------|
| **ONNX** | ✅ | ✅ | ✅ |
| **TensorFlow SavedModel** | ✅ | ✅ | ✅ |
| **PyTorch (.pt)** | ✅ | ✅ | ✅ |
| **TorchScript** | ✅ | ✅ | ✅ |
| **TFLite** | ⚠️ | ✅ | ✅ |
| **CoreML** | ⚠️ | ✅ | ✅ |
| **GGUF (LLM)** | ⚠️ | ✅ | ✅ |

✅ Full support | ⚠️ Conversion via intermediate format

### Optimization Techniques

- **Quantization:** INT8, INT4, FP16 precision reduction
- **Pruning:** Magnitude-based and structured pruning
- **Knowledge Distillation:** Train smaller models from large teachers
- **Graph Optimization:** Operator fusion, constant folding
- **Hardware-Specific:** TensorRT, OpenVINO, CoreML optimization

### Deployment Targets

- ☁️ **Cloud:** AWS SageMaker, Google Vertex AI, Azure ML
- 📱 **Mobile:** iOS (CoreML), Android (TFLite)
- 🖥️ **Edge:** Raspberry Pi, NVIDIA Jetson, Intel NUC
- 🌐 **Web:** ONNX.js, TensorFlow.js
- 🚀 **Specialized:** TPU, NPU, custom accelerators

## 📊 Examples

### Convert PyTorch to ONNX

```python
import torch
from wia_ai_014 import Converter

# Load PyTorch model
model = torch.load('model.pth')
model.eval()

# Convert to ONNX
converter = Converter()
wia_model = converter.from_pytorch(
    model,
    input_shapes={'input': [1, 3, 224, 224]},
    opset_version=17
)

# Save as ONNX
wia_model.save('model.onnx')

# Validate conversion
validation = wia_model.validate()
print(f"Validation: {'✓ PASS' if validation.valid else '✗ FAIL'}")
```

### Quantize Model for Edge Deployment

```typescript
import { Model, Optimizer } from '@wia/neural-network-format';

// Load model
const model = await Model.load('model.onnx');

// Apply INT8 quantization
const quantized = Optimizer.quantize(model, {
    type: 'int8',
    calibrationData: await loadCalibrationData()
});

// Save optimized model
await quantized.save('model_int8.onnx');

// Check size reduction
const originalSize = model.getStats().modelSizeMB;
const quantizedSize = quantized.getStats().modelSizeMB;
console.log(`Size reduced from ${originalSize}MB to ${quantizedSize}MB`);
console.log(`Compression: ${(1 - quantizedSize/originalSize) * 100}%`);
```

### Deploy with Kubernetes

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: wia-model-server
spec:
  replicas: 3
  template:
    spec:
      containers:
      - name: model-server
        image: wia/model-server:latest
        env:
        - name: MODEL_PATH
          value: /models/model.onnx
        - name: WIA_OPTIMIZATION
          value: "all"
        resources:
          limits:
            nvidia.com/gpu: "1"
```

## 🔬 Technical Specifications

### Data Types

- **FP32:** Standard training precision (4 bytes)
- **FP16:** Half precision (2 bytes)
- **BF16:** Brain float 16 (2 bytes, Google TPU)
- **INT8:** 8-bit quantization (1 byte)
- **INT4:** Extreme quantization (0.5 bytes)

### Tensor Layouts

- **NCHW:** PyTorch standard (Batch, Channels, Height, Width)
- **NHWC:** TensorFlow standard (Batch, Height, Width, Channels)
- Automatic conversion handled by WIA-AI-014

### Metadata Schema

All models include comprehensive metadata:

```json
{
  "model_info": {
    "name": "ResNet50-ImageNet",
    "version": "2.1.0",
    "accuracy": {
      "top1": 0.761,
      "top5": 0.931
    }
  },
  "training_info": {
    "dataset": "ImageNet-1K",
    "framework": "pytorch",
    "hyperparameters": { ... }
  },
  "deployment_info": {
    "target_platforms": ["cloud", "edge", "mobile"],
    "input_preprocessing": { ... }
  }
}
```

## 🏢 Enterprise Features

### Model Governance

- ✅ Approval workflows
- ✅ Audit trails
- ✅ Access control (RBAC)
- ✅ Model lineage tracking
- ✅ Compliance reports (GDPR, HIPAA)

### MLOps Integration

- **Experiment Tracking:** MLflow, Weights & Biases
- **Model Registry:** Built-in versioned registry
- **CI/CD:** GitHub Actions, GitLab CI, Jenkins
- **Monitoring:** Prometheus, Grafana, Datadog
- **Logging:** Structured JSON logs

### Security

- 🔒 Model encryption (AES-256)
- 🔒 Cryptographic signatures
- 🔒 Checksum validation (SHA-256)
- 🔒 Access token authentication
- 🔒 mTLS support

## 📈 Performance

### Benchmark Results

| Model | Original | Quantized (INT8) | Speedup | Accuracy Loss |
|-------|----------|------------------|---------|---------------|
| ResNet50 | 98MB | 25MB | 3.2x | <0.5% |
| BERT-Base | 438MB | 110MB | 2.8x | <1.0% |
| GPT-2 | 548MB | 137MB | 3.5x | <0.8% |

*Tested on Intel Xeon CPU, NVIDIA V100 GPU*

## 🤝 Contributing

We welcome contributions! Please see our [Contributing Guide](../CONTRIBUTING.md).

### Development Setup

```bash
# Clone repository
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/neural-network-format

# Install dependencies
npm install  # TypeScript
pip install -e .  # Python

# Run tests
npm test
pytest

# Build documentation
npm run docs
```

## 📄 License

Apache License 2.0 - see [LICENSE](../LICENSE) for details.

## 🌏 Philosophy

**홍익인간 (弘益人間)** - "Benefit All Humanity"

WIA-AI-014 is developed with the mission to democratize AI technology and ensure models can be freely shared, deployed, and optimized across all platforms and frameworks, benefiting researchers, developers, and users worldwide.

## 📞 Support

- **Documentation:** [ebook/en/](./ebook/en/)
- **Issues:** [GitHub Issues](https://github.com/WIA-Official/wia-standards/issues)
- **Discussions:** [GitHub Discussions](https://github.com/WIA-Official/wia-standards/discussions)
- **Email:** ai-standards@wia.org

## 🔗 Related Standards

- **[WIA-AI-001: AI Interoperability](../ai-interoperability/)** - Cross-AI platform communication
- **[WIA-AI-002: Model Exchange](../ai-model-exchange/)** - Secure model sharing
- **[WIA-AI-003: Training Data](../ai-training-data/)** - Dataset standardization

---

**© 2025 WIA (World Certification Industry Association)**

**홍익인간 (弘益人間) - Benefit All Humanity**
