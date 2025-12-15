# WIA Pretrained Gesture Recognition Models

This directory contains specifications for pretrained models compatible with
the WIA Myoelectric Gesture Recognition API.

## Model Formats

### ONNX Models (Recommended)

All pretrained models should be exported in ONNX format for maximum
cross-platform compatibility.

| Model | File | Input Shape | Output Shape | Size |
|-------|------|-------------|--------------|------|
| LDA Baseline | `lda_5gesture.onnx` | (1, 20) | (1, 5) | ~8KB |
| SVM RBF | `svm_5gesture.onnx` | (1, 20) | (1, 5) | ~50KB |
| CNN Small | `cnn_5gesture.onnx` | (1, 4, 200) | (1, 5) | ~100KB |
| Transformer Tiny | `transformer_5gesture.onnx` | (1, 10, 20) | (1, 5) | ~500KB |

### Model Input Specifications

#### Feature Vector Input (LDA, SVM)

20-dimensional feature vector per classification:

| Index | Feature | Channel |
|-------|---------|---------|
| 0-3 | MAV | Ch 1-4 |
| 4-7 | RMS | Ch 1-4 |
| 8-11 | WL | Ch 1-4 |
| 12-15 | ZC | Ch 1-4 |
| 16-19 | SSC | Ch 1-4 |

#### Raw Signal Input (CNN)

- Shape: `(batch, channels, samples)`
- Channels: 4 EMG channels
- Samples: 200 samples (200ms window at 1kHz)
- Normalization: Zero-mean, unit variance per channel

#### Sequence Input (Transformer)

- Shape: `(batch, sequence_length, features)`
- Sequence: 10 feature vectors (temporal context)
- Features: 20-dimensional feature vector

## Gesture Classes

All models classify the following 5 gestures:

| Index | Gesture | Description |
|-------|---------|-------------|
| 0 | Rest | No intentional muscle activation |
| 1 | HandOpen | Fingers extended, palm open |
| 2 | HandClose | Fist grip |
| 3 | WristFlexion | Wrist bent toward palm |
| 4 | WristExtension | Wrist bent away from palm |

## Training Data Requirements

### For Custom Training

Minimum requirements for acceptable classification accuracy:

| Gesture | Repetitions | Duration Each |
|---------|-------------|---------------|
| Rest | 20 | 3 seconds |
| HandOpen | 20 | 3 seconds |
| HandClose | 20 | 3 seconds |
| WristFlexion | 20 | 3 seconds |
| WristExtension | 20 | 3 seconds |

Total minimum: ~5 minutes of labeled data per user.

### Data Augmentation

Recommended augmentations during training:

1. **Additive Noise**: Gaussian noise σ = 0.01-0.05
2. **Time Shifting**: ±10 samples random shift
3. **Amplitude Scaling**: 0.8-1.2× random scaling
4. **Channel Dropout**: 10% probability per channel

## Model Performance Targets

### Latency Requirements

| Tier | Max Latency | Use Case |
|------|-------------|----------|
| Real-time | < 50ms | Prosthetic control |
| Interactive | < 100ms | Gaming, interfaces |
| Batch | < 500ms | Research, logging |

### Accuracy Targets

| Model Type | 5-Gesture | 10-Gesture |
|------------|-----------|------------|
| LDA Baseline | 85% | 75% |
| SVM RBF | 90% | 82% |
| CNN | 93% | 87% |
| Transformer | 95% | 90% |

### Hardware Requirements

| Model | CPU Only | With GPU |
|-------|----------|----------|
| LDA | RPi Zero | N/A |
| SVM | RPi 3+ | N/A |
| CNN | RPi 4+ | Jetson Nano |
| Transformer | PC/Mac | Any CUDA GPU |

## Model Versioning

Model files follow semantic versioning:

```
wia_<model>_<gestures>_v<major>.<minor>.<patch>.onnx
```

Example: `wia_transformer_5gesture_v1.0.0.onnx`

### Version Compatibility

- **Major**: Breaking changes to input/output format
- **Minor**: New features, improved accuracy
- **Patch**: Bug fixes, minor improvements

## Calibration Integration

Pretrained models can be fine-tuned using the calibration API:

```python
from wia_myoelectric import TransformerClassifier, CalibrationSession

# Load pretrained model
classifier = TransformerClassifier()
classifier.load("wia_transformer_5gesture_v1.0.0.onnx")

# Fine-tune with user data
session = CalibrationSession(classifier, gestures=STANDARD_GESTURES)
for gesture in session.get_gestures():
    data = collect_user_data(gesture)
    session.record_samples(gesture, data)

# Update model
result = session.train()
classifier.save("user_customized_model.onnx")
```

## License

All pretrained models are released under the MIT License.
Models may be used for commercial and non-commercial purposes.
