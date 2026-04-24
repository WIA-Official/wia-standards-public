# WIA Bionic Eye - Processed Visual Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-12-15

## 1. Overview

이 문서는 카메라 이미지의 처리 결과 데이터 형식을 정의합니다.

## 2. Processed Visual Structure

### 2.1 Edge Detection

```typescript
interface EdgeDetection {
  data: Uint8Array;           // Edge map (same resolution)
  algorithm: 'canny' | 'sobel' | 'laplacian' | 'custom';
  parameters: {
    lowThreshold: number;     // 0-255
    highThreshold: number;    // 0-255
    kernelSize: number;       // 3, 5, or 7
  };
  processingTime: number;     // ms
}
```

### 2.2 Object Detection

```typescript
interface DetectedObject {
  objectId: string;
  label: string;              // "person", "door", "stairs", "car"
  labelKorean: string;        // "사람", "문", "계단", "자동차"
  confidence: number;         // 0.0-1.0
  boundingBox: {
    x: number;                // top-left x (0-1 normalized)
    y: number;                // top-left y (0-1 normalized)
    width: number;            // 0-1 normalized
    height: number;           // 0-1 normalized
  };
  center: { x: number; y: number };
  distance: number;           // meters (from depth sensor)
  priority: ObjectPriority;
  threat: boolean;            // 위험 객체 여부
  moving: boolean;            // 움직이는 객체
  velocity?: { x: number; y: number };  // m/s
}

enum ObjectPriority {
  CRITICAL = 4,    // 장애물, 위험, 차량
  HIGH = 3,        // 사람, 얼굴, 계단
  MEDIUM = 2,      // 물체, 가구, 문
  LOW = 1,         // 배경, 벽
}
```

### 2.3 Face Detection

```typescript
interface DetectedFace {
  faceId: string;
  boundingBox: BoundingBox;
  landmarks: {
    leftEye: [number, number];
    rightEye: [number, number];
    nose: [number, number];
    leftMouth: [number, number];
    rightMouth: [number, number];
  };
  identity?: {
    name: string;
    confidence: number;
  };
  emotion?: {
    type: 'neutral' | 'happy' | 'sad' | 'angry' | 'surprised';
    confidence: number;
  };
  gazeDirection?: {
    yaw: number;              // degrees
    pitch: number;            // degrees
  };
  distance: number;           // meters
}
```

### 2.4 Depth Information

```typescript
interface DepthData {
  map: Float32Array;          // depth values in meters
  width: number;
  height: number;
  minDistance: number;        // meters
  maxDistance: number;        // meters
  confidenceMap: Uint8Array;  // 0-255 confidence per pixel
  method: 'stereo' | 'tof' | 'structured_light' | 'lidar';
}
```

### 2.5 Text Recognition (OCR)

```typescript
interface TextRegion {
  text: string;
  boundingBox: BoundingBox;
  confidence: number;
  language: string;           // ISO 639-1 code
  fontSize: 'small' | 'medium' | 'large';
  orientation: number;        // degrees
}
```

### 2.6 Motion Detection

```typescript
interface MotionData {
  flowField: Float32Array;    // optical flow vectors
  movingRegions: {
    boundingBox: BoundingBox;
    velocity: { x: number; y: number };  // pixels/frame
    area: number;             // normalized 0-1
  }[];
  globalMotion: {
    translation: { x: number; y: number };
    rotation: number;
  };
  sceneChange: boolean;
}
```

## 3. Priority Objects for Visual Prosthesis

| Priority | Objects | Reason |
|----------|---------|--------|
| CRITICAL | 차량, 자전거, 계단, 구멍 | 즉각적 위험 |
| HIGH | 사람, 얼굴, 문, 신호등 | 네비게이션/소통 |
| MEDIUM | 의자, 테이블, 벽 | 일상 이동 |
| LOW | 배경, 장식 | 컨텍스트 |

## 4. Processing Pipeline

```
Raw Frame
    │
    ├──► Edge Detection ──────────┐
    │                             │
    ├──► Object Detection ────────┤
    │                             │
    ├──► Face Detection ──────────┼──► ProcessedVisual
    │                             │
    ├──► Depth Estimation ────────┤
    │                             │
    ├──► OCR ─────────────────────┤
    │                             │
    └──► Motion Detection ────────┘
```

## 5. Related Specifications

- [FRAME-DATA-SPEC.md](./FRAME-DATA-SPEC.md)
- [STIMULATION-MAP-SPEC.md](./STIMULATION-MAP-SPEC.md)
