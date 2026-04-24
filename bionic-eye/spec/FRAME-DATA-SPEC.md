# WIA Bionic Eye - Frame Data Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-12-15

## 1. Overview

이 문서는 인공 시각 시스템의 이미지 캡처 및 프레임 데이터 형식을 정의합니다.

## 2. Captured Frame Structure

### 2.1 Frame Metadata

| Field | Type | Description |
|-------|------|-------------|
| frameId | string | 고유 프레임 식별자 (UUID) |
| timestamp | number | Unix timestamp (ms) |
| sequenceNumber | number | 연속 프레임 번호 |
| deviceId | string | 카메라 장치 ID |

### 2.2 Image Data

```typescript
interface ImageData {
  width: number;              // 픽셀 (320-1920)
  height: number;             // 픽셀 (240-1080)
  format: ImageFormat;
  data: Uint8Array;
  encoding: 'raw' | 'jpeg' | 'h264' | 'h265';
  compression?: number;       // 압축률 (0-100)
}

enum ImageFormat {
  GRAY8 = 'gray8',           // 8-bit grayscale
  GRAY16 = 'gray16',         // 16-bit grayscale
  RGB24 = 'rgb24',           // 24-bit RGB
  RGBD32 = 'rgbd32',         // RGB + 8-bit Depth
  DEPTH16 = 'depth16',       // 16-bit depth map
  IR8 = 'ir8',               // 8-bit infrared
}
```

### 2.3 Camera Parameters

```typescript
interface CameraParams {
  fov: {
    horizontal: number;       // degrees (60-120)
    vertical: number;         // degrees (45-90)
  };
  exposure: number;           // ms (0.1-100)
  gain: number;               // ISO equivalent (100-6400)
  whiteBalance: number;       // Kelvin (2500-10000)
  focusDistance: number;      // meters (0.1-infinity)
  aperture?: number;          // f-stop
}
```

### 2.4 Sensor Data

```typescript
interface SensorData {
  ambientLight: number;       // lux (0-100000)
  proximity: number;          // cm (0-500)
  imu: {
    accelerometer: [number, number, number];  // m/s²
    gyroscope: [number, number, number];      // rad/s
    magnetometer: [number, number, number];   // μT
    orientation: {
      quaternion: [number, number, number, number];
      euler: { roll: number; pitch: number; yaw: number };
    };
  };
  timestamp: number;
}
```

## 3. Frame Rate Requirements

| Use Case | Min FPS | Recommended FPS | Max Latency |
|----------|---------|-----------------|-------------|
| Navigation | 10 | 15-20 | 100ms |
| Object Detection | 5 | 10 | 200ms |
| Reading | 1 | 5 | 500ms |
| Face Recognition | 10 | 15 | 150ms |

## 4. Data Flow

```
Camera Sensor
     │
     ▼
┌─────────────┐
│ Raw Capture │ → CapturedFrame
└─────────────┘
     │
     ▼
┌─────────────┐
│ Processing  │ → ProcessedVisual
└─────────────┘
     │
     ▼
┌─────────────┐
│ Stimulation │ → StimulationMap
│   Mapping   │
└─────────────┘
     │
     ▼
  Electrode Array
```

## 5. Related Specifications

- [PROCESSED-VISUAL-SPEC.md](./PROCESSED-VISUAL-SPEC.md)
- [STIMULATION-MAP-SPEC.md](./STIMULATION-MAP-SPEC.md)
