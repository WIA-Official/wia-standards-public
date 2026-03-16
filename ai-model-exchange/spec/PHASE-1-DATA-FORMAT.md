# WIA-AI-008 PHASE 1: Data Format & Packaging

**Standard ID**: WIA-AI-008-P1
**Version**: 1.0.0
**Status**: Active
**Last Updated**: 2025-01-15

## 弘益人間 - Benefit All Humanity

---

## 1. Overview

Phase 1 of the WIA-AI-008 standard defines specifications for model data formats, serialization methods, and packaging conventions. This phase ensures models are self-documenting, reproducible, and portable across different environments.

## 2. Model Serialization Formats

### 2.1 Supported Formats

| Format | Extension | Framework | Primary Use Case |
|--------|-----------|-----------|-----------------|
| ONNX | `.onnx` | Universal | Cross-framework exchange |
| PyTorch State Dict | `.pth`, `.pt` | PyTorch | Training checkpoints |
| TorchScript | `.pt` | PyTorch | Production deployment |
| TensorFlow SavedModel | Directory | TensorFlow | Production serving |
| HDF5 | `.h5` | Keras | Legacy Keras models |
| TensorFlow Lite | `.tflite` | TensorFlow | Mobile/edge deployment |
| CoreML | `.mlmodel` | Apple | iOS/macOS deployment |

### 2.2 ONNX as Universal Format

ONNX SHALL be supported as the primary interchange format for framework-agnostic model exchange.

**Requirements:**
- MUST use ONNX opset version 13 or higher
- MUST include all model parameters (weights and biases)
- SHOULD support dynamic batch dimensions
- MUST pass ONNX checker validation

**Example:**
```python
import torch.onnx

torch.onnx.export(
    model,
    dummy_input,
    "model.onnx",
    opset_version=15,
    export_params=True,
    do_constant_folding=True,
    input_names=['input'],
    output_names=['output'],
    dynamic_axes={'input': {0: 'batch'}, 'output': {0: 'batch'}}
)
```

## 3. Model Metadata Schema

### 3.1 Required Metadata

Every model package MUST include a `model_card.json` file with the following structure:

```json
{
  "model_identity": {
    "name": "string (required)",
    "version": "string (semver, required)",
    "unique_id": "string (uuid, required)",
    "created_at": "string (ISO 8601, required)",
    "updated_at": "string (ISO 8601, required)"
  },
  "model_details": {
    "architecture": "string (required)",
    "task_type": "enum (required)",
    "framework": "string (required)",
    "framework_version": "string (required)",
    "description": "string (optional)"
  },
  "training_info": {
    "dataset": "string (required)",
    "dataset_version": "string (optional)",
    "training_date": "string (ISO 8601, required)",
    "hyperparameters": "object (required)",
    "preprocessing": "object (optional)"
  },
  "performance_metrics": {
    "primary_metric": "object (required)",
    "additional_metrics": "object (optional)",
    "evaluation_dataset": "string (required)"
  },
  "input_output_spec": {
    "inputs": [
      {
        "name": "string",
        "shape": "array",
        "dtype": "string",
        "description": "string"
      }
    ],
    "outputs": [
      {
        "name": "string",
        "shape": "array",
        "dtype": "string",
        "description": "string"
      }
    ]
  },
  "deployment": {
    "target_platforms": ["array of strings"],
    "min_memory_mb": "number",
    "min_compute": "string",
    "inference_latency_ms": "number (optional)"
  },
  "governance": {
    "owner": "string (required)",
    "license": "string (required)",
    "intended_use": "string (required)",
    "limitations": "string (required)",
    "ethical_considerations": "string (optional)",
    "bias_analysis": "boolean",
    "privacy_compliant": "boolean"
  },
  "provenance": {
    "base_model": "string (optional)",
    "derived_from": "string (optional)",
    "git_commit": "string (optional)",
    "experiment_id": "string (optional)"
  },
  "wia_standard": {
    "standard_id": "WIA-AI-008",
    "version": "1.0.0",
    "philosophy": "弘益人間 - Benefit All Humanity"
  }
}
```

### 3.2 Task Type Enumeration

Valid values for `task_type`:
- `image-classification`
- `object-detection`
- `semantic-segmentation`
- `instance-segmentation`
- `text-classification`
- `named-entity-recognition`
- `question-answering`
- `text-generation`
- `translation`
- `speech-recognition`
- `speech-synthesis`
- `recommendation`
- `time-series-forecasting`
- `anomaly-detection`
- `custom`

## 4. Model Packaging Structure

### 4.1 Standard Directory Layout

```
model-package-v1.0.0/
├── model_card.json          # Required: Metadata
├── model.onnx               # Required: ONNX format
├── model.{framework}        # Optional: Framework-specific format
├── README.md                # Required: Human-readable documentation
├── LICENSE                  # Required: License file
├── requirements.txt         # Optional: Python dependencies
├── config.yaml              # Optional: Configuration
├── preprocessing/           # Optional: Preprocessing code
│   ├── __init__.py
│   └── transform.py
├── postprocessing/          # Optional: Postprocessing code
│   ├── __init__.py
│   └── decode.py
├── examples/                # Optional: Usage examples
│   ├── inference.py
│   └── sample_data/
├── tests/                   # Optional: Test suite
│   ├── test_model.py
│   └── test_data/
└── docs/                    # Optional: Additional documentation
    ├── architecture.md
    └── training_procedure.md
```

### 4.2 Archive Format

Model packages SHOULD be distributed as compressed archives:
- `.tar.gz` for cross-platform compatibility
- `.zip` for Windows compatibility
- Maximum uncompressed size: 10 GB (recommendation)

Naming convention: `{model-name}-{version}.tar.gz`

Example: `resnet50-classifier-1.0.0.tar.gz`

## 5. Model Versioning

### 5.1 Semantic Versioning

Model versions MUST follow semantic versioning (SemVer 2.0.0):

**Format:** `MAJOR.MINOR.PATCH`

- **MAJOR**: Incompatible API changes (input/output shapes, data types)
- **MINOR**: Backward-compatible functionality (improved accuracy, new features)
- **PATCH**: Backward-compatible bug fixes (numerical fixes, documentation updates)

**Examples:**
- `1.0.0` → `2.0.0`: Changed input shape from (224, 224) to (256, 256)
- `1.0.0` → `1.1.0`: Improved accuracy from 90% to 93%
- `1.0.0` → `1.0.1`: Fixed preprocessing bug

### 5.2 Version Metadata

Each version MUST include:
- **changelog.md**: Description of changes
- **migration_guide.md**: Instructions for upgrading (for MAJOR changes)
- **performance_comparison.md**: Comparison with previous version

## 6. Model Card Documentation

### 6.1 README.md Structure

The README.md file MUST include:

```markdown
# {Model Name} v{Version}

## Quick Start
[Code example for immediate use]

## Model Description
[Overview of model purpose and capabilities]

## Model Details
- **Architecture:** {architecture name}
- **Task:** {task type}
- **Framework:** {framework} {version}
- **Parameters:** {parameter count}
- **Model Size:** {size in MB}

## Training Data
[Description of training dataset]

## Performance
[Key metrics and benchmarks]

## Usage
[Detailed usage instructions with code examples]

## Limitations
[Known limitations and constraints]

## License
[License information]

## Citation
[How to cite this model]

## Acknowledgments
[Credits and acknowledgments]

弘益人間 - Benefit All Humanity
```

## 7. Input/Output Specifications

### 7.1 Input Specification Format

```json
{
  "inputs": [
    {
      "name": "image",
      "shape": [1, 3, 224, 224],
      "dtype": "float32",
      "format": "NCHW",
      "range": [0.0, 1.0],
      "preprocessing": {
        "normalization": {
          "mean": [0.485, 0.456, 0.406],
          "std": [0.229, 0.224, 0.225]
        },
        "resize": 256,
        "center_crop": 224
      },
      "description": "RGB image tensor"
    }
  ]
}
```

### 7.2 Output Specification Format

```json
{
  "outputs": [
    {
      "name": "logits",
      "shape": [1, 1000],
      "dtype": "float32",
      "description": "Class logits for 1000 ImageNet classes",
      "postprocessing": {
        "softmax": true,
        "top_k": 5
      }
    }
  ]
}
```

## 8. Preprocessing and Postprocessing

### 8.1 Preprocessing Pipeline

Preprocessing logic SHOULD be included as Python code:

```python
# preprocessing/transform.py

def preprocess(image_path):
    """
    Preprocess image for model inference

    Args:
        image_path: Path to input image

    Returns:
        Preprocessed tensor ready for inference
    """
    from PIL import Image
    import numpy as np

    # Load image
    image = Image.open(image_path).convert('RGB')

    # Resize and crop
    image = image.resize((256, 256))
    image = center_crop(image, 224)

    # Convert to numpy array
    array = np.array(image, dtype=np.float32) / 255.0

    # Normalize
    mean = np.array([0.485, 0.456, 0.406])
    std = np.array([0.229, 0.224, 0.225])
    array = (array - mean) / std

    # NCHW format
    array = np.transpose(array, (2, 0, 1))
    array = np.expand_dims(array, axis=0)

    return array
```

### 8.2 Postprocessing Pipeline

```python
# postprocessing/decode.py

def postprocess(output, top_k=5):
    """
    Postprocess model output

    Args:
        output: Raw model output
        top_k: Number of top predictions to return

    Returns:
        List of (class_name, probability) tuples
    """
    import numpy as np

    # Apply softmax
    exp_output = np.exp(output - np.max(output))
    probabilities = exp_output / exp_output.sum()

    # Get top-k predictions
    top_indices = np.argsort(probabilities[0])[-top_k:][::-1]

    # Load class labels
    with open('class_labels.json') as f:
        labels = json.load(f)

    results = [
        (labels[idx], float(probabilities[0][idx]))
        for idx in top_indices
    ]

    return results
```

## 9. Validation and Testing

### 9.1 Model Validation Checklist

Before packaging, models MUST pass:

- [ ] **Format Validation**: ONNX checker passes
- [ ] **Metadata Validation**: All required fields present
- [ ] **Numerical Validation**: Outputs match reference implementation
- [ ] **Shape Validation**: Input/output shapes match specification
- [ ] **Performance Validation**: Meets minimum accuracy threshold
- [ ] **Documentation Validation**: README.md is complete
- [ ] **License Validation**: LICENSE file present and valid

### 9.2 Test Suite

Model packages SHOULD include automated tests:

```python
# tests/test_model.py

import pytest
import onnxruntime as ort
import numpy as np

def test_model_loads():
    """Test that model loads successfully"""
    session = ort.InferenceSession("model.onnx")
    assert session is not None

def test_input_shape():
    """Test that input shape is correct"""
    session = ort.InferenceSession("model.onnx")
    input_shape = session.get_inputs()[0].shape
    assert input_shape == [None, 3, 224, 224]

def test_inference():
    """Test that inference runs successfully"""
    session = ort.InferenceSession("model.onnx")
    input_data = np.random.randn(1, 3, 224, 224).astype(np.float32)
    output = session.run(None, {'input': input_data})
    assert output[0].shape == (1, 1000)

def test_numerical_stability():
    """Test numerical stability"""
    session = ort.InferenceSession("model.onnx")
    input_data = np.random.randn(1, 3, 224, 224).astype(np.float32)

    output1 = session.run(None, {'input': input_data})[0]
    output2 = session.run(None, {'input': input_data})[0]

    np.testing.assert_allclose(output1, output2, rtol=1e-5)
```

## 10. Security Considerations

### 10.1 Model Integrity

Model packages SHOULD include checksums:

```bash
# Generate SHA256 checksum
sha256sum model.onnx > model.onnx.sha256

# Verify checksum
sha256sum -c model.onnx.sha256
```

### 10.2 Secure Distribution

When distributing models:
- MUST use HTTPS for downloads
- SHOULD sign model packages with GPG
- SHOULD provide checksum verification
- MUST scan for malware before distribution

## 11. Compliance

Models following this specification:
- MUST include `"wia_standard": {"standard_id": "WIA-AI-008", "version": "1.0.0"}` in metadata
- MUST include philosophy statement: `"philosophy": "弘益人間 - Benefit All Humanity"`
- SHOULD include WIA-AI-008 badge in README.md

**Badge:**
```markdown
[![WIA-AI-008](https://img.shields.io/badge/WIA--AI--008-1.0.0-green.svg)](https://wia.org/standards/ai-008)
```

---

## Appendix A: Complete Example

See reference implementation at: `examples/resnet50-classifier/`

## Appendix B: Validation Tools

```bash
# Install WIA-AI-008 validator
pip install wia-ai-008-validator

# Validate model package
wia-validator validate model-package-v1.0.0/

# Generate model card template
wia-validator init-card --name my-model --version 1.0.0
```

---

**弘益人間 - Benefit All Humanity**

© 2025 WIA (World Certification Industry Association)
Licensed under Apache 2.0
