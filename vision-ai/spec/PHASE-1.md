# WIA-AI-021 Vision AI Standard - PHASE 1 Specification

**Version:** 1.0
**Status:** Official Standard
**Date:** 2025-01-15
**Organization:** World Certification Industry Association (WIA)

---

## 弘益人間 (Benefit All Humanity)

Vision AI technology that sees the world clearly and helps humanity thrive.

---

## 1. Executive Summary

WIA-AI-021 Vision AI Standard defines a comprehensive framework for developing, deploying, and certifying computer vision systems. Phase 1 focuses on core foundations: image processing, classification, and detection capabilities that form the building blocks of all vision AI applications.

### 1.1 Scope

This specification covers:
- Image processing and preprocessing standards
- Image classification requirements
- Object detection protocols
- Performance benchmarks
- API specifications
- Ethical guidelines

### 1.2 Target Audience

- ML Engineers and Data Scientists
- Computer Vision Developers
- System Architects
- Product Managers
- Compliance Officers

---

## 2. Core Principles

### 2.1 Accuracy and Reliability

**Requirement:** All vision AI systems must achieve minimum performance thresholds:
- Image Classification: ≥ 90% Top-5 accuracy on ImageNet-1K
- Object Detection: ≥ 50% mAP@0.5 on COCO dataset
- Processing Time: ≤ 100ms per image (640x640) on standard hardware

### 2.2 Privacy and Security

**Requirements:**
- Encrypt all image data in transit (TLS 1.3+)
- Encrypt stored images (AES-256 minimum)
- Support on-device processing for sensitive data
- Implement differential privacy where applicable
- Provide clear data retention policies

### 2.3 Transparency and Explainability

**Requirements:**
- Provide confidence scores for all predictions
- Enable visualization of activation maps (e.g., Grad-CAM)
- Document training data sources and biases
- Maintain audit logs of all predictions
- Support model cards for documentation

### 2.4 Fairness and Non-Discrimination

**Requirements:**
- Test for bias across demographic groups
- Maintain balanced performance (±5% accuracy) across groups
- Provide bias mitigation strategies
- Regular fairness audits (quarterly minimum)

### 2.5 弘益人間 (Benefit All Humanity)

**Requirements:**
- Prioritize applications that improve human welfare
- Consider environmental impact (carbon footprint)
- Ensure accessibility for people with disabilities
- Respect cultural sensitivities
- Support open-source contributions where possible

---

## 3. Image Processing Standards

### 3.1 Supported Image Formats

**MUST Support:**
- JPEG/JPG (baseline and progressive)
- PNG (8-bit and 24-bit)
- WebP
- TIFF (uncompressed and LZW compressed)

**SHOULD Support:**
- HEIF/HEIC
- BMP
- GIF (first frame for static analysis)

### 3.2 Image Dimensions

**Minimum Resolution:** 224 x 224 pixels
**Maximum Resolution:** 4096 x 4096 pixels (unless specifically designed for higher)
**Aspect Ratio:** Support arbitrary aspect ratios with appropriate preprocessing

### 3.3 Color Spaces

**MUST Support:**
- RGB (Red, Green, Blue)
- Grayscale (single channel)

**SHOULD Support:**
- BGR (for OpenCV compatibility)
- HSV (Hue, Saturation, Value)
- LAB (CIELAB color space)

### 3.4 Preprocessing Pipeline

Standard preprocessing steps:

```python
def preprocess_image(image, target_size=(224, 224)):
    """
    Standard WIA-AI-021 image preprocessing

    Args:
        image: Input image (PIL Image or numpy array)
        target_size: Target dimensions (width, height)

    Returns:
        Preprocessed image tensor
    """
    # 1. Resize
    image = resize(image, target_size, interpolation='bilinear')

    # 2. Normalize to [0, 1]
    image = image.astype(np.float32) / 255.0

    # 3. Standardize (ImageNet mean/std)
    mean = np.array([0.485, 0.456, 0.406])
    std = np.array([0.229, 0.224, 0.225])
    image = (image - mean) / std

    # 4. Convert to tensor
    return torch.from_numpy(image).permute(2, 0, 1)
```

---

## 4. Image Classification

### 4.1 Classification Requirements

**Input:**
- Single image
- Minimum 224x224 pixels
- RGB or grayscale

**Output:**
```json
{
  "predictions": [
    {
      "class_id": 281,
      "class_name": "tabby_cat",
      "confidence": 0.94,
      "bounding_box": null
    },
    {
      "class_id": 282,
      "class_name": "tiger_cat",
      "confidence": 0.03,
      "bounding_box": null
    }
  ],
  "processing_time_ms": 45,
  "model_version": "resnet50-v1.0",
  "timestamp": "2025-01-15T10:30:00Z"
}
```

### 4.2 Performance Benchmarks

| Model Type | Min Accuracy (Top-5) | Max Inference Time | Parameters |
|-----------|---------------------|-------------------|------------|
| Mobile | 85% | 50ms | ≤ 10M |
| Standard | 90% | 100ms | 10M-50M |
| Large | 95% | 500ms | > 50M |

### 4.3 Supported Architectures

**MUST Support at least one:**
- ResNet (18, 34, 50, 101, 152)
- EfficientNet (B0-B7)
- Vision Transformer (ViT-B, ViT-L)

**MAY Support:**
- MobileNet
- DenseNet
- Inception
- ConvNeXt

---

## 5. Object Detection

### 5.1 Detection Requirements

**Input:**
- Single image
- Minimum 416x416 pixels recommended
- RGB

**Output:**
```json
{
  "detections": [
    {
      "class_id": 0,
      "class_name": "person",
      "confidence": 0.96,
      "bounding_box": {
        "x_min": 120,
        "y_min": 80,
        "x_max": 340,
        "y_max": 520,
        "format": "xyxy"
      }
    },
    {
      "class_id": 2,
      "class_name": "car",
      "confidence": 0.88,
      "bounding_box": {
        "x_min": 450,
        "y_min": 200,
        "x_max": 780,
        "y_max": 480,
        "format": "xyxy"
      }
    }
  ],
  "processing_time_ms": 65,
  "model_version": "yolov8n-v1.0",
  "image_size": [640, 640],
  "timestamp": "2025-01-15T10:30:00Z"
}
```

### 5.2 Bounding Box Formats

**MUST Support:**
- `xyxy`: (x_min, y_min, x_max, y_max)
- `xywh`: (x_center, y_center, width, height)

**Coordinate System:**
- Origin: Top-left corner (0, 0)
- X-axis: Horizontal (left to right)
- Y-axis: Vertical (top to bottom)

### 5.3 Performance Benchmarks

| Model Type | Min mAP@0.5 | Max Inference Time | Use Case |
|-----------|------------|-------------------|----------|
| Real-time | 40% | 30ms | Video streams |
| Standard | 50% | 100ms | General purpose |
| High Accuracy | 60% | 500ms | Critical applications |

### 5.4 Non-Maximum Suppression (NMS)

**Requirements:**
- IoU Threshold: Configurable (default 0.45)
- Confidence Threshold: Configurable (default 0.25)
- Maximum Detections: Configurable (default 300)

```python
def nms(boxes, scores, iou_threshold=0.45, conf_threshold=0.25):
    """
    WIA-AI-021 compliant NMS implementation

    Args:
        boxes: Bounding boxes [N, 4] (x1, y1, x2, y2)
        scores: Confidence scores [N]
        iou_threshold: IoU threshold for suppression
        conf_threshold: Minimum confidence threshold

    Returns:
        Indices of boxes to keep
    """
    # Filter by confidence
    mask = scores > conf_threshold
    boxes = boxes[mask]
    scores = scores[mask]

    # Sort by score
    sorted_indices = np.argsort(scores)[::-1]

    keep = []
    while len(sorted_indices) > 0:
        current = sorted_indices[0]
        keep.append(current)

        if len(sorted_indices) == 1:
            break

        ious = compute_iou(boxes[current], boxes[sorted_indices[1:]])
        sorted_indices = sorted_indices[1:][ious < iou_threshold]

    return np.array(keep)
```

---

## 6. API Specifications

### 6.1 REST API Endpoints

#### 6.1.1 Image Classification

```
POST /api/v1/classify
```

**Request:**
```json
{
  "image": "base64_encoded_image_data",
  "top_k": 5,
  "min_confidence": 0.01
}
```

**Response:**
```json
{
  "status": "success",
  "predictions": [...],
  "processing_time_ms": 45
}
```

#### 6.1.2 Object Detection

```
POST /api/v1/detect
```

**Request:**
```json
{
  "image": "base64_encoded_image_data",
  "confidence_threshold": 0.25,
  "iou_threshold": 0.45,
  "max_detections": 300
}
```

**Response:**
```json
{
  "status": "success",
  "detections": [...],
  "processing_time_ms": 65
}
```

### 6.2 Error Handling

**Standard Error Response:**
```json
{
  "status": "error",
  "error": {
    "code": "INVALID_IMAGE",
    "message": "Image format not supported",
    "details": "Supported formats: JPEG, PNG, WebP"
  },
  "timestamp": "2025-01-15T10:30:00Z"
}
```

**Error Codes:**
- `INVALID_IMAGE`: Invalid image format or corrupted data
- `IMAGE_TOO_LARGE`: Image exceeds maximum size
- `INVALID_PARAMETERS`: Invalid request parameters
- `MODEL_ERROR`: Internal model inference error
- `RATE_LIMIT_EXCEEDED`: Too many requests
- `AUTHENTICATION_FAILED`: Invalid API key

### 6.3 Rate Limiting

**Requirements:**
- Implement rate limiting per API key
- Return `429 Too Many Requests` when exceeded
- Include `X-RateLimit-*` headers

**Headers:**
```
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 999
X-RateLimit-Reset: 1705315200
```

---

## 7. Performance Requirements

### 7.1 Latency

| Deployment Type | P50 | P95 | P99 |
|----------------|-----|-----|-----|
| Cloud (GPU) | 50ms | 100ms | 200ms |
| Edge (CPU) | 200ms | 500ms | 1000ms |
| Mobile | 500ms | 1000ms | 2000ms |

### 7.2 Throughput

**Minimum Requirements:**
- Cloud: 100 requests/second per GPU
- Edge: 10 requests/second per CPU core
- Mobile: 1 request/second

### 7.3 Availability

**Requirements:**
- Uptime: ≥ 99.9% (excluding planned maintenance)
- Maximum planned downtime: 4 hours/month
- Disaster recovery: RTO < 4 hours, RPO < 1 hour

---

## 8. Security Requirements

### 8.1 Data Protection

**Requirements:**
- TLS 1.3+ for all network communications
- AES-256 encryption for data at rest
- No logging of sensitive image content (unless explicitly required)
- Secure key management (HSM or KMS)

### 8.2 Authentication & Authorization

**MUST Support:**
- API key authentication
- OAuth 2.0
- JWT tokens

**SHOULD Support:**
- mTLS (mutual TLS)
- SAML 2.0

### 8.3 Input Validation

**Requirements:**
- Validate image format and size
- Scan for malicious content
- Implement request size limits
- Sanitize all user inputs

---

## 9. Testing and Validation

### 9.1 Unit Tests

**Coverage Requirements:**
- Code coverage: ≥ 80%
- All public APIs must have tests
- Edge cases must be tested

### 9.2 Integration Tests

**Requirements:**
- End-to-end API testing
- Performance benchmarking
- Load testing (sustained and peak)

### 9.3 Validation Datasets

**Standard Benchmarks:**
- ImageNet-1K (classification)
- COCO (detection)
- Additional domain-specific datasets

---

## 10. Compliance and Certification

### 10.1 Certification Process

1. **Self-Assessment:** Complete WIA-AI-021 checklist
2. **Documentation:** Submit technical documentation
3. **Testing:** Pass all required benchmarks
4. **Audit:** Third-party security and ethics review
5. **Certification:** Receive WIA-AI-021 Phase 1 certificate

### 10.2 Recertification

**Requirements:**
- Annual recertification required
- Update required for major version changes
- Continuous monitoring of performance metrics

---

## 11. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2025-01-15 | Initial release of Phase 1 specification |

---

## 12. References

1. ImageNet Large Scale Visual Recognition Challenge (ILSVRC)
2. Microsoft COCO: Common Objects in Context
3. NIST AI Risk Management Framework
4. ISO/IEC 42001:2023 - AI Management System
5. GDPR (General Data Protection Regulation)
6. IEEE 2830-2021 - Technical Framework for AI Bias

---

## 13. Contact

**World Certification Industry Association (WIA)**
Email: standards@wia-official.org
Website: https://wia-official.org

**Document Maintainer:** WIA Technical Committee on Vision AI

---

© 2025 SmileStory Inc. / World Certification Industry Association
弘益人間 · Benefit All Humanity

## Implementer note — operational lifecycle

Vision-AI models are not static artefacts: they drift, degrade, and require periodic refresh as the world they were trained on shifts. The wire-format discipline (signed model envelopes with chained dataset evidence, per-cohort fairness, drift signals, audit log) is what enables sustainable production operation across multi-year horizons. A model trained for a 2026 production deployment will likely be refreshed quarterly or semi-annually; the standards artefacts let each refresh be evaluated against the same documented thresholds without re-inventing the evaluation harness for each cycle.
