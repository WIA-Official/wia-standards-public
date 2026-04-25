# WIA-AI-008: AI Model Exchange Standard 📦

[![WIA Standard](https://img.shields.io/badge/WIA--AI--008-1.0.0-green.svg)](https://wia.org/standards/ai-008)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![TypeScript](https://img.shields.io/badge/TypeScript-SDK-blue.svg)](api/typescript)

> **홍익인간 (弘益人間)** (Hong-ik In-gan) - *Broadly Benefiting Humanity*

The WIA-AI-008 standard defines best practices for packaging, converting, distributing, and serving machine learning models across different frameworks and platforms.

## 🌟 Overview

WIA-AI-008 provides a comprehensive framework for AI model exchange that enables:

- **Cross-Framework Compatibility**: Seamlessly convert models between PyTorch, TensorFlow, JAX, and more
- **Universal Format**: ONNX as the standard interchange format
- **Model Versioning**: Semantic versioning and lifecycle management
- **Secure Distribution**: Cryptographic signatures and access control
- **Production Deployment**: Standards for model serving and monitoring
- **Open Ecosystem**: Promote collaboration and model sharing

## 📋 Table of Contents

- [Quick Start](#-quick-start)
- [Standard Structure](#-standard-structure)
- [Phase Overview](#-phase-overview)
- [Interactive Tools](#-interactive-tools)
- [Documentation](#-documentation)
- [SDK & API](#-sdk--api)
- [Examples](#-examples)
- [Contributing](#-contributing)
- [License](#-license)

## 🚀 Quick Start

### View the Standard

1. **Landing Page**: Open `index.html` in your browser for an interactive overview
2. **Simulator**: Explore `simulator/index.html` to try model conversion workflows
3. **E-Books**: Read comprehensive guides in `ebook/en/` or `ebook/ko/`
4. **Specifications**: Review detailed specs in `spec/`

### Install TypeScript SDK

```bash
npm install @wia/ai-model-exchange
```

### Basic Usage

```typescript
import { RegistryClient, TaskType } from '@wia/ai-model-exchange';

// Connect to model registry
const client = new RegistryClient({
  registry_url: 'https://registry.example.com',
  api_key: process.env.WIA_API_KEY
});

// Search for models
const results = await client.searchModels({
  query: 'sentiment analysis',
  task_type: TaskType.TextClassification,
  framework: 'pytorch'
});

// Download model
await client.downloadModel(
  'bert-sentiment',
  '1.0.0',
  './models/'
);

// Use model for inference
import { ModelServerClient } from '@wia/ai-model-exchange';

const server = new ModelServerClient('http://localhost:8080');
const prediction = await server.predict({
  model_name: 'bert-sentiment',
  inputs: { text: 'This movie was amazing!' }
});
```

## 🏗️ Standard Structure

```
ai-model-exchange/
├── index.html                 # Landing page with standard overview
├── simulator/                 # Interactive model exchange simulator
│   └── index.html            # 5-tab simulator (Format, Conversion, Protocol, Integration, Test)
├── ebook/                    # Comprehensive learning resources
│   ├── en/                   # English e-book (8 chapters)
│   │   ├── index.html
│   │   └── chapter-01.html to chapter-08.html
│   └── ko/                   # Korean e-book (8 chapters)
│       ├── index.html
│       └── chapter-01.html to chapter-08.html
├── spec/                     # Detailed specifications
│   ├── PHASE-1-DATA-FORMAT.md        # Model formats & packaging
│   ├── PHASE-2-API.md                # Conversion APIs & algorithms
│   ├── PHASE-3-PROTOCOL.md           # Registry & distribution
│   └── PHASE-4-INTEGRATION.md        # Serving & deployment
├── api/                      # SDK implementations
│   └── typescript/           # TypeScript SDK
│       ├── src/
│       │   ├── index.ts      # Main SDK
│       │   └── types.ts      # Type definitions
│       └── package.json
└── README.md                 # This file
```

## 📊 Phase Overview

### Phase 1: Data Format & Packaging

Defines model serialization formats, metadata schemas, and packaging conventions.

**Key Topics:**
- ONNX as universal format
- Model card metadata standard
- Semantic versioning
- Input/output specifications
- Preprocessing/postprocessing pipelines

**Specification**: [PHASE-1-DATA-FORMAT.md](spec/PHASE-1-DATA-FORMAT.md)

### Phase 2: API & Conversion Algorithms

Provides APIs for cross-framework conversion, optimization, and validation.

**Key Topics:**
- PyTorch ↔ ONNX ↔ TensorFlow conversions
- Quantization (FP16, INT8, INT4)
- Pruning and knowledge distillation
- Graph optimization
- Numerical validation

**Specification**: [PHASE-2-API.md](spec/PHASE-2-API.md)

### Phase 3: Exchange Protocol & Registry

Defines protocols for model distribution, authentication, and version management.

**Key Topics:**
- REST API specification
- Authentication & authorization (RBAC)
- Model lifecycle stages
- Cryptographic signatures
- Search and discovery

**Specification**: [PHASE-3-PROTOCOL.md](spec/PHASE-3-PROTOCOL.md)

### Phase 4: Integration & Model Serving

Standards for production deployment, monitoring, and operations.

**Key Topics:**
- Serving infrastructure (TorchServe, TF Serving, Triton)
- Health checks and monitoring
- Auto-scaling
- A/B testing and canary deployments
- Disaster recovery

**Specification**: [PHASE-4-INTEGRATION.md](spec/PHASE-4-INTEGRATION.md)

## 🎮 Interactive Tools

### Model Exchange Simulator

The simulator provides hands-on experience with model exchange workflows:

1. **Data Format Tab**: Create model cards and explore packaging formats
2. **Algorithms Tab**: Run model conversions with different optimization levels
3. **Protocol Tab**: Interact with model registry APIs
4. **Integration Tab**: Generate deployment code for various platforms
5. **Test Tab**: Validate models with comprehensive test suites

**Access**: Open `simulator/index.html` in your browser

### Features:
- ✅ Live code generation
- ✅ Interactive parameter tuning
- ✅ Real-time validation
- ✅ Performance metrics simulation
- ✅ Multi-language support (EN/KO)

## 📚 Documentation

### E-Books

Comprehensive 8-chapter guides covering the entire model exchange lifecycle:

#### English E-Book (`ebook/en/`)

1. **Introduction to Model Exchange** - Overview and motivation
2. **Model Formats & Serialization** - Deep dive into formats
3. **ONNX: The Universal Format** - Complete ONNX guide
4. **Cross-Framework Conversion** - PyTorch, TensorFlow, JAX
5. **Model Optimization & Quantization** - Performance techniques
6. **Model Registries & Version Control** - MLflow, Hugging Face, DVC
7. **Model Serving & Deployment** - Production infrastructure
8. **Best Practices & Case Studies** - Real-world examples

#### Korean E-Book (`ebook/ko/`)

Complete Korean translation of all 8 chapters with culturally relevant examples.

**Each chapter includes:**
- Detailed explanations with code examples
- Practical exercises
- Summary of key concepts
- Review questions

## 🛠️ SDK & API

### TypeScript SDK

Full-featured SDK for model registry and serving operations.

**Installation:**
```bash
npm install @wia/ai-model-exchange
```

**Features:**
- ✅ Model registry client (search, upload, download)
- ✅ Model server client (inference, batch prediction)
- ✅ Utility functions (validation, versioning)
- ✅ Full TypeScript types
- ✅ Promise-based async API

**Documentation**: See [api/typescript/README.md](api/typescript/README.md)

### Python SDK (Coming Soon)

```python
from wia_ai import RegistryClient

client = RegistryClient('https://registry.example.com')
client.download_model('resnet50', version='1.0.0', path='./models/')
```

## 💡 Examples

### Example 1: Export PyTorch Model to ONNX

```python
import torch
import torchvision.models as models

# Load model
model = models.resnet50(pretrained=True)
model.eval()

# Export to ONNX
dummy_input = torch.randn(1, 3, 224, 224)
torch.onnx.export(
    model,
    dummy_input,
    "resnet50.onnx",
    opset_version=15,
    input_names=['input'],
    output_names=['output'],
    dynamic_axes={'input': {0: 'batch'}, 'output': {0: 'batch'}}
)
```

### Example 2: Create Model Card

```typescript
import { ModelUtils } from '@wia/ai-model-exchange';

const modelCard = ModelUtils.generateModelCardTemplate('my-model', '1.0.0');

// Customize metadata
modelCard.model_details = {
  architecture: 'ResNet-50',
  task_type: TaskType.ImageClassification,
  framework: 'pytorch',
  framework_version: '2.0.1',
  description: 'Image classification model trained on ImageNet'
};

// Validate
const { valid, errors } = ModelUtils.validateModelCard(modelCard);
```

### Example 3: Upload to Registry

```typescript
// Register new model
await client.registerModel({
  name: 'my-classifier',
  description: 'Custom image classifier',
  owner: 'team@company.com',
  license: 'apache-2.0',
  version: '1.0.0',
  metadata: modelCard
});

// Upload model file
await client.uploadModelVersion(
  'my-classifier',
  '1.0.0',
  './models/classifier.onnx',
  modelCard
);
```

### Example 4: Model Serving

```typescript
import { ModelServerClient } from '@wia/ai-model-exchange';

const server = new ModelServerClient('http://localhost:8080');

// Single prediction
const result = await server.predict({
  model_name: 'my-classifier',
  model_version: 'latest',
  inputs: { image: imageData }
});

// Batch prediction
const results = await server.batchPredict(
  'my-classifier',
  [image1, image2, image3],
  'latest'
);
```

## 🌐 Supported Frameworks

- **PyTorch** 2.0+
- **TensorFlow** 2.12+
- **JAX** 0.4+
- **ONNX** 1.13+
- **Keras** 2.12+
- **scikit-learn** 1.2+
- **XGBoost** 1.7+
- **LightGBM** 3.3+

## 📈 Compliance

To comply with WIA-AI-008, models MUST:

1. ✅ Include comprehensive model card (`model_card.json`)
2. ✅ Use semantic versioning (MAJOR.MINOR.PATCH)
3. ✅ Export to ONNX format (opset 13+)
4. ✅ Include WIA standard metadata:
   ```json
   {
     "wia_standard": {
       "standard_id": "WIA-AI-008",
       "version": "1.0.0",
       "philosophy": "홍익인간 (弘益人間) - Benefit All Humanity"
     }
   }
   ```
5. ✅ Pass validation tests
6. ✅ Include license information
7. ✅ Document intended use and limitations

## 🤝 Contributing

We welcome contributions to improve the WIA-AI-008 standard!

### How to Contribute

1. **Feedback**: Open issues for suggestions or clarifications
2. **Examples**: Share real-world use cases
3. **Translations**: Help translate documentation
4. **Tools**: Contribute validators, converters, or utilities
5. **Case Studies**: Share implementation experiences

### Contribution Guidelines

- Follow the philosophy of **홍익인간 (弘益人間)** (Benefit All Humanity)
- Ensure backward compatibility
- Include comprehensive documentation
- Add test coverage
- Use semantic versioning

## 📄 License

The WIA-AI-008 standard is licensed under **Apache License 2.0**.

```
Copyright 2025 WIA (World Certification Industry Association)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
```

## 🔗 Links

- **WIA Official**: https://wia.org
- **Standard Page**: https://wia.org/standards/ai-008
- **GitHub Repository**: https://github.com/WIA-Official/wia-standards
- **Issue Tracker**: https://github.com/WIA-Official/wia-standards/issues
- **NPM Package**: https://npmjs.com/package/@wia/ai-model-exchange

## 📧 Contact

- **Email**: standards@wia.org
- **Discussion Forum**: https://forum.wia.org/ai-008
- **Twitter**: @WIA_Standards

## 🙏 Acknowledgments

The WIA-AI-008 standard was developed with contributions from:

- Machine learning researchers and practitioners worldwide
- Framework maintainers (PyTorch, TensorFlow, ONNX teams)
- Industry leaders in AI deployment
- Open source community members

Special thanks to all who contributed to making AI models more accessible and interoperable.

---

<div align="center">

## 홍익인간 (弘益人間)
### *Benefit All Humanity*

**WIA-AI-008 v1.0.0** | **2025**

[![WIA](https://img.shields.io/badge/WIA-Standards-green.svg)](https://wia.org)
[![Community](https://img.shields.io/badge/Community-Driven-blue.svg)](https://forum.wia.org)
[![Open Source](https://img.shields.io/badge/Open-Source-orange.svg)](LICENSE)

</div>
