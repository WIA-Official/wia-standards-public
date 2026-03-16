# WIA-AI-021 Vision AI Standard - PHASE 2 Specification

**Version:** 1.0
**Status:** Official Standard
**Date:** 2025-01-15
**Organization:** World Certification Industry Association (WIA)

---

## 弘益人間 (Benefit All Humanity)

Advanced vision capabilities for deeper scene understanding.

---

## 1. Executive Summary

Phase 2 extends WIA-AI-021 with advanced capabilities: image segmentation, OCR, pose estimation, and facial analysis. These capabilities enable pixel-level understanding and detailed visual information extraction.

### 1.1 Scope

- Semantic and Instance Segmentation
- Optical Character Recognition (OCR)
- Human Pose Estimation
- Facial Analysis (Detection, Landmarks, Recognition)
- Privacy-Preserving Techniques

---

## 2. Image Segmentation

### 2.1 Semantic Segmentation

**Input:** RGB image, minimum 512x512 pixels

**Output:**
```json
{
  "segmentation_map": "base64_encoded_mask",
  "classes": [
    {"id": 0, "name": "background", "pixel_count": 245760, "percentage": 94.2},
    {"id": 15, "name": "person", "pixel_count": 12480, "percentage": 4.8},
    {"id": 7, "name": "car", "pixel_count": 2560, "percentage": 1.0}
  ],
  "image_size": [512, 512],
  "processing_time_ms": 120
}
```

**Performance Requirements:**
- mIoU ≥ 70% on PASCAL VOC
- mIoU ≥ 60% on Cityscapes
- Inference time ≤ 200ms (512x512 on GPU)

### 2.2 Instance Segmentation

**Output:**
```json
{
  "instances": [
    {
      "instance_id": 1,
      "class_id": 15,
      "class_name": "person",
      "confidence": 0.94,
      "bounding_box": {"x_min": 120, "y_min": 80, "x_max": 340, "y_max": 520},
      "mask": "base64_encoded_binary_mask",
      "area": 15840
    }
  ],
  "processing_time_ms": 180
}
```

**Performance Requirements:**
- mAP@0.5 ≥ 40% on COCO
- Support ≥ 80 object categories

---

## 3. Optical Character Recognition (OCR)

### 3.1 Text Detection

**Requirements:**
- Detect text in natural scenes
- Support multiple orientations
- Handle curved text
- Multi-scale detection

**Output:**
```json
{
  "text_regions": [
    {
      "region_id": 1,
      "polygon": [[10, 20], [200, 20], [200, 50], [10, 50]],
      "confidence": 0.95,
      "text": "VISION AI"
    }
  ]
}
```

### 3.2 Text Recognition

**Requirements:**
- Support English, digits, common symbols
- SHOULD support: Korean, Japanese, Chinese, Arabic
- Accuracy ≥ 95% on ICDAR datasets

**Supported Languages (Minimum):**
- English (Latin)
- Numbers (0-9)
- Common symbols (!, ?, ., etc.)

**Output:**
```json
{
  "text": "VISION AI STANDARD WIA-AI-021",
  "confidence": 0.92,
  "words": [
    {"text": "VISION", "confidence": 0.98, "bbox": [10, 20, 80, 50]},
    {"text": "AI", "confidence": 0.96, "bbox": [85, 20, 110, 50]},
    {"text": "STANDARD", "confidence": 0.94, "bbox": [115, 20, 220, 50]},
    {"text": "WIA-AI-021", "confidence": 0.89, "bbox": [225, 20, 350, 50]}
  ]
}
```

---

## 4. Human Pose Estimation

### 4.1 2D Pose Estimation

**Keypoints (COCO format):**
1. Nose
2. Left Eye
3. Right Eye
4. Left Ear
5. Right Ear
6. Left Shoulder
7. Right Shoulder
8. Left Elbow
9. Right Elbow
10. Left Wrist
11. Right Wrist
12. Left Hip
13. Right Hip
14. Left Knee
15. Right Knee
16. Left Ankle
17. Right Ankle

**Output:**
```json
{
  "poses": [
    {
      "person_id": 1,
      "keypoints": [
        {"id": 0, "name": "nose", "x": 250, "y": 120, "confidence": 0.95, "visible": true},
        {"id": 1, "name": "left_eye", "x": 240, "y": 110, "confidence": 0.92, "visible": true}
      ],
      "bounding_box": {"x_min": 180, "y_min": 80, "x_max": 320, "y_max": 480},
      "overall_confidence": 0.89
    }
  ]
}
```

**Performance Requirements:**
- AP@0.5 ≥ 70% on COCO Keypoints
- Inference time ≤ 150ms per person

---

## 5. Facial Analysis

### 5.1 Face Detection

**Requirements:**
- Detect faces at multiple scales
- Minimum face size: 40x40 pixels
- Maximum faces per image: 100

**Output:**
```json
{
  "faces": [
    {
      "face_id": 1,
      "bounding_box": {"x_min": 200, "y_min": 150, "x_max": 350, "y_max": 350},
      "confidence": 0.98,
      "landmarks": {
        "left_eye": [230, 200],
        "right_eye": [310, 200],
        "nose": [270, 250],
        "left_mouth": [240, 300],
        "right_mouth": [300, 300]
      }
    }
  ]
}
```

### 5.2 Privacy Requirements

**MANDATORY:**
- Obtain explicit consent before facial recognition
- Provide opt-out mechanisms
- Support face anonymization/blurring
- Comply with GDPR, CCPA, BIPA
- No storage of biometric data without consent
- Regular privacy audits

**Face Anonymization API:**
```json
{
  "anonymization": {
    "method": "gaussian_blur",
    "radius": 25,
    "anonymized_image": "base64_encoded_result"
  }
}
```

### 5.3 Facial Attributes

**Optional Capabilities:**
- Age estimation (±5 years)
- Gender classification
- Emotion recognition
- Glasses detection
- Facial hair detection

**Ethical Requirements:**
- Document potential biases
- Test across demographics
- Provide confidence intervals
- Include disclaimers for sensitive attributes

---

## 6. Privacy-Preserving Techniques

### 6.1 On-Device Processing

**Requirements:**
- Support local inference (no cloud)
- Provide lightweight models for edge devices
- Minimize data transmission

### 6.2 Differential Privacy

**For aggregate statistics:**
- Implement ε-differential privacy
- ε ≤ 1.0 for sensitive applications
- Document privacy budget

### 6.3 Data Minimization

**Requirements:**
- Process only necessary data
- Delete raw images after processing
- Return only required outputs
- Configurable retention policies

---

## 7. API Endpoints (Phase 2)

### 7.1 Semantic Segmentation

```
POST /api/v2/segment
```

**Request:**
```json
{
  "image": "base64_encoded_image",
  "model": "deeplabv3",
  "output_format": "mask" // or "overlay", "both"
}
```

### 7.2 OCR

```
POST /api/v2/ocr
```

**Request:**
```json
{
  "image": "base64_encoded_image",
  "languages": ["en", "ko"],
  "detect_orientation": true
}
```

### 7.3 Pose Estimation

```
POST /api/v2/pose
```

**Request:**
```json
{
  "image": "base64_encoded_image",
  "max_persons": 10,
  "min_confidence": 0.5
}
```

### 7.4 Face Detection

```
POST /api/v2/face/detect
```

**Request:**
```json
{
  "image": "base64_encoded_image",
  "detect_landmarks": true,
  "anonymize": false
}
```

---

## 8. Performance Benchmarks

| Task | Model | mAP/mIoU | Latency (GPU) | Latency (CPU) |
|------|-------|----------|---------------|---------------|
| Semantic Seg | DeepLabv3 | 70% | 150ms | 800ms |
| Instance Seg | Mask R-CNN | 40% | 200ms | 1200ms |
| OCR | CRAFT + CRNN | 95% | 100ms | 500ms |
| Pose | HRNet | 75% | 120ms | 600ms |
| Face Detect | RetinaFace | 95% | 50ms | 250ms |

---

## 9. Testing Requirements

### 9.1 Segmentation Tests

- PASCAL VOC validation set
- Cityscapes validation set
- Custom domain-specific datasets

### 9.2 OCR Tests

- ICDAR 2015
- ICDAR 2019
- Multi-language datasets

### 9.3 Pose Tests

- COCO Keypoints validation
- MPII Human Pose
- Custom activity datasets

### 9.4 Face Tests

- WIDER FACE
- Demographic diversity tests
- Edge case scenarios (occlusion, lighting)

---

## 10. Compliance

### 10.1 Biometric Data Regulations

**MUST Comply:**
- GDPR (Europe)
- CCPA (California)
- BIPA (Illinois)
- LGPD (Brazil)

### 10.2 Ethical AI Principles

- Transparency in capabilities and limitations
- Fairness across demographic groups
- Privacy by design
- Human oversight for critical decisions
- Right to explanation

---

© 2025 SmileStory Inc. / World Certification Industry Association
弘益人間 · Benefit All Humanity