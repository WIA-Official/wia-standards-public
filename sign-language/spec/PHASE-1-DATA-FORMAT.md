# WIA Sign Language Recognition Standard
## Phase 1: Data Format Specification

**Version:** 1.0.0  
**Status:** Final  
**Last Updated:** 2025-12-25  
**Author:** WIA Technical Committee  

---

## Table of Contents

1. [Introduction](#introduction)
2. [Video Data Model](#video-data-model)
3. [Gesture Representation](#gesture-representation)
4. [Keypoint Schema](#keypoint-schema)
5. [JSON Schema Definitions](#json-schema-definitions)
6. [Sign Language Encoding](#sign-language-encoding)
7. [Validation Rules](#validation-rules)
8. [Example Payloads](#example-payloads)

---

## 1. Introduction

This specification defines standardized data formats for sign language video, keypoints, and recognition results.

### 1.1 Design Principles

- **Universal**: Support all 300+ sign languages
- **Efficient**: Optimize for real-time streaming
- **Extensible**: Allow future enhancements
- **Privacy**: No PII in standard format

### 1.2 Scope

- Video format specifications
- Hand/body keypoint representation
- Gesture classification format
- Multi-language sign encoding

---

## 2. Video Data Model

### 2.1 Video Requirements

```typescript
interface VideoInput {
  /** Format: MP4, WebM, raw frames */
  format: 'mp4' | 'webm' | 'raw';
  
  /** Codec: H.264, VP9, AV1 */
  codec: 'h264' | 'vp9' | 'av1';
  
  /** Resolution (minimum 720p) */
  width: number;
  height: number;
  
  /** Frame rate (minimum 30fps) */
  fps: number;
  
  /** Bitrate for streaming */
  bitrate?: number;
}
```

### 2.2 Frame Data

```typescript
interface VideoFrame {
  /** Frame timestamp in milliseconds */
  timestamp: number;
  
  /** Frame index */
  frameIndex: number;
  
  /** Base64 encoded image or video chunk */
  data: string;
  
  /** Frame metadata */
  metadata?: FrameMetadata;
}
```

---

## 3. Gesture Representation

### 3.1 Sign Data Structure

```typescript
interface SignLanguageGesture {
  /** Unique gesture ID */
  gestureId: string;
  
  /** Sign language (ISO 639-3 code) */
  language: 'ase' | 'bfi' | 'kvk' | 'jsl' | string;
  
  /** Confidence score (0-1) */
  confidence: number;
  
  /** Text translation */
  text: string;
  
  /** Keypoint data */
  keypoints: KeypointData;
  
  /** Bounding box */
  boundingBox?: BoundingBox;
}
```

---

## 4. Keypoint Schema

### 4.1 Hand Landmarks (21 points per hand)

```typescript
interface HandKeypoints {
  /** Left hand landmarks */
  left: Landmark[];
  
  /** Right hand landmarks */
  right: Landmark[];
}

interface Landmark {
  /** X coordinate (normalized 0-1) */
  x: number;
  
  /** Y coordinate (normalized 0-1) */
  y: number;
  
  /** Z coordinate (depth, normalized) */
  z: number;
  
  /** Visibility score (0-1) */
  visibility: number;
}
```

### 4.2 Pose Keypoints (33 body landmarks)

```typescript
interface PoseKeypoints {
  /** Body landmarks (MediaPipe format) */
  landmarks: Landmark[];
}
```

### 4.3 Face Landmarks (468 facial points)

```typescript
interface FaceKeypoints {
  /** Facial landmarks */
  landmarks: Landmark[];
}
```

---

## 5. JSON Schema Definitions

### 5.1 Recognition Request

```json
{
  "format": "WIA-SIGN-LANGUAGE-v1.0",
  "timestamp": "2025-12-25T10:30:00Z",
  "input": {
    "type": "video",
    "language": "ase",
    "video": {
      "format": "mp4",
      "width": 1280,
      "height": 720,
      "fps": 30,
      "data": "base64_encoded_video"
    }
  },
  "options": {
    "realtime": true,
    "confidenceThreshold": 0.85
  }
}
```

### 5.2 Recognition Response

```json
{
  "format": "WIA-SIGN-LANGUAGE-v1.0",
  "timestamp": "2025-12-25T10:30:01Z",
  "result": {
    "gestures": [{
      "gestureId": "hello",
      "language": "ase",
      "confidence": 0.985,
      "text": "Hello",
      "keypoints": {
        "leftHand": [],
        "rightHand": [],
        "pose": [],
        "face": []
      }
    }]
  }
}
```

---

## 6. Sign Language Encoding

### 6.1 ISO 639-3 Language Codes

- `ase`: American Sign Language (ASL)
- `bfi`: British Sign Language (BSL)
- `kvk`: Korean Sign Language (KSL)
- `jsl`: Japanese Sign Language
- `csl`: Chinese Sign Language

### 6.2 Custom Extensions

Support for regional dialects and variations.

---

## 7. Validation Rules

- Video must be at least 720p @ 30fps
- Confidence scores between 0.0 and 1.0
- All keypoints must have x, y, z, visibility
- Timestamps in ISO 8601 format

---

## 8. Example Payloads

See full examples in documentation.

---

**弘益人間 · Benefit All Humanity**

© 2025 WIA - World Certification Industry Association
