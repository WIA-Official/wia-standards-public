# WIA-SEC-007: Biometric Authentication - Phase 1: Foundation

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-01
**Primary Color:** #8B5CF6 (Purple - Security)

---

## 1. Overview

This specification defines the foundational standards for biometric authentication systems, covering fingerprint, iris, and face recognition technologies. It establishes data formats, quality metrics, and template standards in accordance with ISO/IEC 19794 (Biometric Data Interchange Formats) and ISO/IEC 30107 (Presentation Attack Detection).

### 1.1 Scope

- **Biometric Modalities**: Fingerprint, iris, face recognition
- **Applications**: Authentication, identification, verification
- **Compliance**: ISO/IEC 19794, ISO/IEC 24745, FIDO2, WebAuthn
- **Security**: Template protection, liveness detection, privacy preservation

### 1.2 Normative References

- **ISO/IEC 19794-2**: Fingerprint minutiae data
- **ISO/IEC 19794-6**: Iris image data
- **ISO/IEC 19794-5**: Face image data
- **ISO/IEC 30107**: Presentation Attack Detection (PAD)
- **ISO/IEC 24745**: Biometric template protection
- **NIST SP 800-63B**: Digital Identity Guidelines
- **W3C WebAuthn**: Web Authentication specification
- **FIDO2**: Client to Authenticator Protocol (CTAP)

---

## 2. Biometric Data Types

### 2.1 Fingerprint Recognition

#### 2.1.1 Image Capture Requirements

- **Resolution**: Minimum 500 DPI (1000 DPI recommended)
- **Image Size**:
  - 500 DPI: 300×400 pixels (15.2×20.3 mm)
  - 1000 DPI: 600×800 pixels (15.2×20.3 mm)
- **Bit Depth**: 8-bit grayscale (256 levels)
- **Image Format**: Raw, WSQ compressed, PNG, JPEG2000

#### 2.1.2 Minutiae Data Format

Minutiae represent ridge endings and bifurcations extracted from fingerprint images.

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "type": "object",
  "required": ["fingerprintID", "minutiae", "imageInfo"],
  "properties": {
    "fingerprintID": {
      "type": "string",
      "description": "Unique fingerprint identifier"
    },
    "captureDate": {
      "type": "string",
      "format": "date-time",
      "description": "ISO 8601 timestamp"
    },
    "imageInfo": {
      "type": "object",
      "required": ["width", "height", "resolution"],
      "properties": {
        "width": { "type": "integer", "minimum": 300 },
        "height": { "type": "integer", "minimum": 400 },
        "resolution": {
          "type": "integer",
          "enum": [500, 1000],
          "description": "DPI"
        }
      }
    },
    "quality": {
      "type": "integer",
      "minimum": 0,
      "maximum": 100,
      "description": "NFIQ quality score (0-100)"
    },
    "minutiae": {
      "type": "array",
      "items": {
        "type": "object",
        "required": ["x", "y", "type", "angle"],
        "properties": {
          "x": { "type": "number", "description": "X coordinate (mm)" },
          "y": { "type": "number", "description": "Y coordinate (mm)" },
          "type": {
            "type": "string",
            "enum": ["ending", "bifurcation"],
            "description": "Minutiae type"
          },
          "angle": {
            "type": "number",
            "minimum": 0,
            "maximum": 360,
            "description": "Ridge direction (degrees)"
          },
          "quality": {
            "type": "number",
            "minimum": 0,
            "maximum": 1,
            "description": "Minutia quality (0-1)"
          }
        }
      }
    },
    "corePoints": {
      "type": "array",
      "description": "Core points in fingerprint pattern",
      "items": {
        "type": "object",
        "properties": {
          "x": { "type": "number" },
          "y": { "type": "number" },
          "angle": { "type": "number" }
        }
      }
    },
    "deltaPoints": {
      "type": "array",
      "description": "Delta (triangular) points",
      "items": {
        "type": "object",
        "properties": {
          "x": { "type": "number" },
          "y": { "type": "number" }
        }
      }
    }
  }
}
```

#### 2.1.3 Template Size

- **Typical size**: 200-500 bytes
- **Average minutiae count**: 20-50 points
- **Storage**: Encrypted with AES-256-GCM

---

### 2.2 Iris Recognition

#### 2.2.1 Image Capture Requirements

- **Wavelength**: Near-infrared (700-900 nm) to minimize pupil constriction
- **Resolution**: Minimum 640×480 pixels (1280×960 recommended)
- **Iris Diameter**: Minimum 150 pixels across iris diameter
- **Focus**: Clear iris texture, sharp collarette boundary
- **Distance**: 10-30 cm from sensor

#### 2.2.2 Iris Code Format

Iris recognition uses Daugman's IrisCode algorithm to encode iris texture patterns.

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "type": "object",
  "required": ["irisID", "irisCode", "maskCode"],
  "properties": {
    "irisID": {
      "type": "string",
      "description": "Unique iris identifier"
    },
    "eye": {
      "type": "string",
      "enum": ["left", "right"],
      "description": "Which eye"
    },
    "captureDate": {
      "type": "string",
      "format": "date-time"
    },
    "imageInfo": {
      "type": "object",
      "properties": {
        "width": { "type": "integer" },
        "height": { "type": "integer" },
        "irisDiameter": {
          "type": "integer",
          "minimum": 150,
          "description": "Iris diameter in pixels"
        }
      }
    },
    "quality": {
      "type": "number",
      "minimum": 0,
      "maximum": 1,
      "description": "ISO/IEC 29794-6 quality score"
    },
    "irisCode": {
      "type": "string",
      "description": "Base64-encoded 2048-bit IrisCode",
      "pattern": "^[A-Za-z0-9+/]+={0,2}$"
    },
    "maskCode": {
      "type": "string",
      "description": "Base64-encoded mask for occluded regions",
      "pattern": "^[A-Za-z0-9+/]+={0,2}$"
    },
    "rotation": {
      "type": "number",
      "description": "Iris rotation angle for alignment (radians)"
    },
    "pupilCenter": {
      "type": "object",
      "properties": {
        "x": { "type": "number" },
        "y": { "type": "number" }
      }
    },
    "pupilRadius": { "type": "number" },
    "irisRadius": { "type": "number" }
  }
}
```

#### 2.2.3 Template Size

- **IrisCode size**: 256 bytes (2048 bits)
- **Mask size**: 256 bytes
- **Total template**: ~512 bytes

---

### 2.3 Face Recognition

#### 2.3.1 Image Capture Requirements

- **Resolution**: Minimum 1280×720 pixels (1920×1080 recommended)
- **Lighting**: Uniform illumination, avoid shadows
- **Face Size**: Minimum 200 pixels between eyes
- **Pose**: Frontal ±15° yaw/pitch/roll
- **Expression**: Neutral preferred
- **Sensors**: RGB camera + depth sensor (structured light, ToF, or stereo)

#### 2.3.2 Face Template Format

Modern face recognition uses deep learning embeddings (e.g., FaceNet, ArcFace).

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "type": "object",
  "required": ["faceID", "embedding", "landmarks"],
  "properties": {
    "faceID": {
      "type": "string",
      "description": "Unique face template identifier"
    },
    "captureDate": {
      "type": "string",
      "format": "date-time"
    },
    "imageInfo": {
      "type": "object",
      "properties": {
        "width": { "type": "integer" },
        "height": { "type": "integer" },
        "colorSpace": {
          "type": "string",
          "enum": ["RGB", "grayscale"]
        }
      }
    },
    "quality": {
      "type": "number",
      "minimum": 0,
      "maximum": 1,
      "description": "Face image quality score"
    },
    "embedding": {
      "type": "array",
      "description": "128 or 512-dimensional face embedding vector",
      "items": { "type": "number" },
      "minItems": 128,
      "maxItems": 512
    },
    "landmarks": {
      "type": "array",
      "description": "68 or 98 facial landmark points",
      "items": {
        "type": "object",
        "properties": {
          "x": { "type": "number" },
          "y": { "type": "number" },
          "z": { "type": "number", "description": "Depth (optional)" }
        }
      }
    },
    "boundingBox": {
      "type": "object",
      "properties": {
        "x": { "type": "number" },
        "y": { "type": "number" },
        "width": { "type": "number" },
        "height": { "type": "number" }
      }
    },
    "pose": {
      "type": "object",
      "properties": {
        "yaw": { "type": "number", "description": "degrees" },
        "pitch": { "type": "number" },
        "roll": { "type": "number" }
      }
    },
    "livenessScore": {
      "type": "number",
      "minimum": 0,
      "maximum": 1,
      "description": "Liveness detection confidence"
    }
  }
}
```

#### 2.3.3 Template Size

- **128-D embedding**: ~512 bytes (128 × 4 bytes per float)
- **512-D embedding**: ~2 KB
- **With landmarks**: Additional 1-2 KB

---

## 3. Quality Metrics

### 3.1 Fingerprint Quality (NFIQ)

NIST Fingerprint Image Quality (NFIQ) scores:

| NFIQ Score | Quality Level | Description |
|------------|---------------|-------------|
| 1 | Excellent | Clear ridges, minimal noise |
| 2 | Very Good | Good ridge clarity |
| 3 | Good | Acceptable quality |
| 4 | Fair | Marginal quality |
| 5 | Poor | Unusable for matching |

**Rejection Threshold**: NFIQ ≥ 4 (reject poor quality samples)

### 3.2 Iris Quality (ISO/IEC 29794-6)

Quality factors:
- **Focus**: Sharp iris texture
- **Occlusion**: < 30% by eyelids/lashes
- **Pupil dilation**: 30-70% of iris diameter
- **Motion blur**: Minimal
- **Lighting**: No specular reflections on iris

**Quality Score**: 0.0 (poor) to 1.0 (excellent)
**Rejection Threshold**: Quality < 0.6

### 3.3 Face Quality

Quality factors:
- **Resolution**: Inter-eye distance ≥ 200 pixels
- **Pose**: |yaw|, |pitch|, |roll| ≤ 15°
- **Illumination**: Even lighting, no harsh shadows
- **Expression**: Neutral face preferred
- **Occlusion**: Minimal (no sunglasses, face masks)

---

## 4. Template Protection

### 4.1 Cancelable Biometrics

Biometric templates should be protected using cancelable/revocable transforms:

1. **BioHashing**: Hash template with user-specific key
2. **Fuzzy Vault**: Combine template with chaff points
3. **Homomorphic Encryption**: Encrypt template, match in encrypted domain

### 4.2 Secure Storage

```json
{
  "encryptedTemplate": {
    "algorithm": "AES-256-GCM",
    "iv": "base64-encoded initialization vector",
    "authTag": "base64-encoded authentication tag",
    "ciphertext": "base64-encoded encrypted biometric template"
  },
  "metadata": {
    "templateType": "fingerprint|iris|face",
    "createdAt": "ISO 8601 timestamp",
    "version": "1.0.0"
  }
}
```

---

## 5. Liveness Detection

### 5.1 Fingerprint PAD (Presentation Attack Detection)

- **Capacitive sensors**: Detect live skin conductivity
- **Optical sensors**: Multi-spectral imaging (visible + NIR)
- **Ultrasonic sensors**: Detect subsurface fingerprint structure
- **Challenge-response**: Require finger movement

### 5.2 Iris PAD

- **Pupil response**: Light reflex test
- **Texture analysis**: Detect printed/synthetic iris
- **NIR imaging**: Distinguish live eye from photo/video

### 5.3 Face PAD

- **3D depth**: Structured light/ToF to detect 2D attacks
- **Texture analysis**: Detect photo/video replay
- **Motion analysis**: Require user to blink, smile, or turn head
- **Thermal imaging**: Detect body heat signature

---

## 6. Privacy & Compliance

### 6.1 GDPR Compliance

Biometric data is classified as **special category personal data** under GDPR Article 9:

- **Explicit consent**: Required before collection
- **Purpose limitation**: Specify exact use case
- **Data minimization**: Collect only necessary biometrics
- **Right to erasure**: Allow users to delete biometric data
- **Data portability**: Export templates in standard format

### 6.2 Data Retention

- **Authentication systems**: Retain only as long as user account is active
- **Forensic systems**: Follow local law enforcement retention policies
- **Health systems**: Follow HIPAA/medical record retention rules

---

## 7. Example: Complete Fingerprint Template

```json
{
  "templateID": "fp-550e8400-e29b-41d4-a716-446655440000",
  "userID": "user-12345",
  "templateType": "fingerprint",
  "finger": "right-index",
  "captureDate": "2025-01-15T10:30:00Z",
  "version": "1.0.0",

  "imageInfo": {
    "width": 600,
    "height": 800,
    "resolution": 1000,
    "format": "WSQ"
  },

  "quality": {
    "nfiq": 2,
    "score": 85
  },

  "minutiae": [
    {
      "x": 5.2,
      "y": 8.7,
      "type": "ending",
      "angle": 45.3,
      "quality": 0.92
    },
    {
      "x": 7.1,
      "y": 10.2,
      "type": "bifurcation",
      "angle": 120.5,
      "quality": 0.88
    }
    // ... 20-50 minutiae points
  ],

  "encryption": {
    "algorithm": "AES-256-GCM",
    "iv": "MTIzNDU2Nzg5MGFiY2RlZg==",
    "authTag": "dGFnMTIzNDU2Nzg5MGFiY2Q=",
    "ciphertext": "ZW5jcnlwdGVkX3RlbXBsYXRlX2RhdGE="
  }
}
```

---

## 8. Compliance Checklist

- [ ] Implement quality checks (NFIQ, ISO/IEC 29794-6)
- [ ] Apply template protection (cancelable biometrics)
- [ ] Encrypt templates with AES-256-GCM
- [ ] Implement liveness detection (PAD)
- [ ] Obtain explicit user consent (GDPR)
- [ ] Provide data deletion mechanism
- [ ] Support data export (portability)
- [ ] Conduct regular security audits

---

**弘益人間 · Benefit All Humanity**

© 2025 World Certification Industry Association (WIA)
Licensed under MIT License
