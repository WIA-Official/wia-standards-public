# WIA Eye Gaze Standard - Data Format Specification

**Version**: 1.0.0-draft
**Status**: Phase 1 Draft
**Last Updated**: 2025-01-XX

---

## 1. 개요 (Overview)

이 문서는 WIA Eye Gaze Interoperability Protocol의 데이터 포맷을 정의합니다.
모든 시선 추적 디바이스와 애플리케이션이 상호운용될 수 있도록 공통 데이터 구조를 명세합니다.

### 1.1 설계 원칙

1. **정규화 (Normalization)**: 화면 좌표는 0.0 - 1.0 범위로 정규화
2. **양안 지원 (Binocular)**: 좌/우 눈 개별 데이터 + 결합 데이터
3. **확장성 (Extensibility)**: 필수 필드 최소화, 선택 필드로 확장
4. **정밀도 (Precision)**: 고정밀 타임스탬프 및 좌표
5. **품질 표시 (Quality)**: 신뢰도 및 유효성 정보 포함

---

## 2. 기본 타입 (Base Types)

### 2.1 Vector2D

2D 좌표를 표현합니다.

```typescript
interface Vector2D {
  x: number;  // X 좌표
  y: number;  // Y 좌표
}
```

### 2.2 Vector3D

3D 좌표를 표현합니다.

```typescript
interface Vector3D {
  x: number;  // X 좌표 (mm)
  y: number;  // Y 좌표 (mm)
  z: number;  // Z 좌표 (mm, 디바이스로부터의 거리)
}
```

### 2.3 BoundingBox

화면상 영역을 표현합니다.

```typescript
interface BoundingBox {
  x: number;       // 좌상단 X (정규화 좌표)
  y: number;       // 좌상단 Y (정규화 좌표)
  width: number;   // 너비 (정규화 값)
  height: number;  // 높이 (정규화 값)
}
```

---

## 3. 핵심 데이터 구조 (Core Data Structures)

### 3.1 GazePoint

시선 데이터의 기본 단위입니다.

```typescript
interface GazePoint {
  // === 필수 필드 (Required) ===

  /** 타임스탬프 - Unix timestamp (밀리초) */
  timestamp: number;

  /** 화면 X 좌표 (0.0 = 왼쪽, 1.0 = 오른쪽) */
  x: number;

  /** 화면 Y 좌표 (0.0 = 위, 1.0 = 아래) */
  y: number;

  /** 데이터 신뢰도 (0.0 - 1.0) */
  confidence: number;

  /** 데이터 유효 여부 */
  valid: boolean;

  // === 선택 필드 (Optional) ===

  /** 왼쪽 눈 데이터 */
  leftEye?: EyeData;

  /** 오른쪽 눈 데이터 */
  rightEye?: EyeData;

  /** Fixation 여부 (응시 상태) */
  fixation?: boolean;

  /** Saccade 여부 (빠른 이동 상태) */
  saccade?: boolean;

  /** Fixation ID (같은 fixation에 속하면 동일 ID) */
  fixationId?: string;

  /** 디바이스 고유 타임스탬프 */
  deviceTimestamp?: number;

  /** 추가 메타데이터 */
  metadata?: Record<string, unknown>;
}
```

### 3.2 EyeData

개별 눈의 상세 데이터입니다.

```typescript
interface EyeData {
  // === 기본 시선 데이터 ===

  /** 해당 눈의 화면 시선 좌표 (정규화) */
  gaze: Vector2D;

  /** 데이터 유효 여부 */
  valid: boolean;

  // === 동공 데이터 ===

  /** 동공 직경 (mm) */
  pupilDiameter?: number;

  /** 동공 중심 위치 (카메라 이미지 내 정규화 좌표) */
  pupilCenter?: Vector2D;

  // === 3D 데이터 ===

  /** 눈 위치 (3D, mm 단위, 트래커 기준) */
  gazeOrigin?: Vector3D;

  /** 시선 방향 (단위 벡터) */
  gazeDirection?: Vector3D;

  // === 눈 상태 ===

  /** 눈 열림 정도 (0.0 = 닫힘, 1.0 = 완전히 열림) */
  eyeOpenness?: number;

  /** 눈 열림 (mm 단위) */
  eyeOpennessMm?: number;
}
```

---

## 4. 좌표계 정의 (Coordinate System)

### 4.1 화면 좌표 (Screen Coordinates)

```
(0.0, 0.0) ─────────────────────────► X (1.0, 0.0)
     │
     │     ┌─────────────────────┐
     │     │                     │
     │     │      Screen         │
     │     │                     │
     │     │                     │
     │     └─────────────────────┘
     │
     ▼
Y (0.0, 1.0)                    (1.0, 1.0)
```

- **원점 (0, 0)**: 화면 좌상단
- **X축**: 오른쪽으로 증가 (0.0 → 1.0)
- **Y축**: 아래쪽으로 증가 (0.0 → 1.0)
- **범위**: 화면 안은 [0.0, 1.0], 화면 밖은 음수 또는 1.0 초과 가능

### 4.2 3D 좌표 (3D Coordinates)

```
                 Y (위)
                 │
                 │
                 │
                 ┼──────────► X (오른쪽)
                /
               /
              /
             Z (사용자 방향)

    * Eye Tracker 위치가 원점
    * 단위: 밀리미터 (mm)
```

- **원점**: Eye Tracker 중심
- **X축**: 오른쪽이 양수
- **Y축**: 위쪽이 양수
- **Z축**: 사용자 방향이 양수
- **단위**: 밀리미터 (mm)

---

## 5. 데이터 스트림 포맷 (Data Stream Format)

### 5.1 단일 샘플 (Single Sample)

```json
{
  "timestamp": 1704067200000,
  "x": 0.5234,
  "y": 0.3891,
  "confidence": 0.95,
  "valid": true,
  "leftEye": {
    "gaze": { "x": 0.5198, "y": 0.3876 },
    "valid": true,
    "pupilDiameter": 4.2,
    "eyeOpenness": 0.92
  },
  "rightEye": {
    "gaze": { "x": 0.5270, "y": 0.3906 },
    "valid": true,
    "pupilDiameter": 4.1,
    "eyeOpenness": 0.91
  },
  "fixation": true,
  "fixationId": "fix_001"
}
```

### 5.2 배치 데이터 (Batch Data)

여러 샘플을 묶어서 전송할 때 사용합니다.

```json
{
  "version": "1.0.0",
  "deviceId": "tobii-pro-fusion-001",
  "sessionId": "session_2025_001",
  "startTimestamp": 1704067200000,
  "endTimestamp": 1704067200100,
  "samplingRate": 120,
  "samples": [
    { "timestamp": 1704067200000, "x": 0.52, "y": 0.39, "confidence": 0.95, "valid": true },
    { "timestamp": 1704067200008, "x": 0.52, "y": 0.39, "confidence": 0.94, "valid": true },
    { "timestamp": 1704067200016, "x": 0.53, "y": 0.38, "confidence": 0.96, "valid": true }
  ]
}
```

---

## 6. 데이터 품질 지표 (Data Quality Indicators)

### 6.1 Confidence 값 해석

| 값 범위 | 의미 | 권장 조치 |
|--------|------|----------|
| 0.9 - 1.0 | 매우 높음 | 정상 사용 |
| 0.7 - 0.9 | 높음 | 정상 사용 |
| 0.5 - 0.7 | 중간 | 주의 필요 |
| 0.3 - 0.5 | 낮음 | 보정/확인 권장 |
| 0.0 - 0.3 | 매우 낮음 | 데이터 사용 주의 |

### 6.2 유효성 플래그 (Validity Flag)

```typescript
// valid가 false인 경우
{
  "timestamp": 1704067200000,
  "x": 0,       // 유효하지 않은 값
  "y": 0,       // 유효하지 않은 값
  "confidence": 0,
  "valid": false,
  "invalidReason": "blink"  // 선택적: 무효 사유
}
```

**무효 사유 (invalidReason) 값**:
- `"blink"`: 눈 깜빡임
- `"tracking_lost"`: 추적 손실
- `"out_of_range"`: 추적 범위 이탈
- `"calibration_required"`: 보정 필요
- `"unknown"`: 알 수 없음

---

## 7. 픽셀 좌표 변환 (Pixel Coordinate Conversion)

정규화 좌표에서 픽셀 좌표로 변환:

```typescript
function toPixelCoordinates(
  normalizedPoint: Vector2D,
  screenWidth: number,
  screenHeight: number
): { px: number; py: number } {
  return {
    px: Math.round(normalizedPoint.x * screenWidth),
    py: Math.round(normalizedPoint.y * screenHeight)
  };
}

// 예시: 1920x1080 화면에서
// (0.5, 0.5) → (960, 540)
```

픽셀 좌표에서 정규화 좌표로 변환:

```typescript
function toNormalizedCoordinates(
  px: number,
  py: number,
  screenWidth: number,
  screenHeight: number
): Vector2D {
  return {
    x: px / screenWidth,
    y: py / screenHeight
  };
}
```

---

## 8. 호환성 (Compatibility)

### 8.1 제조사별 매핑

| WIA 필드 | Tobii | Gazepoint | Pupil Labs |
|---------|-------|-----------|------------|
| timestamp | system_time_stamp | TIME | timestamp |
| x | gaze_point_on_display_area[0] | FPOGX | norm_pos[0] |
| y | gaze_point_on_display_area[1] | FPOGY | 1 - norm_pos[1] * |
| confidence | - (계산 필요) | FPOGV | confidence |
| valid | validity | FPOGV > 0 | confidence > 0.6 |
| pupilDiameter | pupil_diameter | LPD/RPD | diameter_3d |

\* Pupil Labs는 Y축이 반전되어 있어 변환 필요

### 8.2 샘플링 레이트 차이 처리

다른 샘플링 레이트의 디바이스를 동기화할 때:

```typescript
interface SynchronizedSample {
  timestamp: number;
  primary: GazePoint;
  interpolated: boolean;  // 보간된 데이터인지 여부
}
```

---

## 9. TypeScript 전체 타입 정의

```typescript
// wia-eye-gaze-types.ts

export interface Vector2D {
  x: number;
  y: number;
}

export interface Vector3D {
  x: number;
  y: number;
  z: number;
}

export interface BoundingBox {
  x: number;
  y: number;
  width: number;
  height: number;
}

export interface EyeData {
  gaze: Vector2D;
  valid: boolean;
  pupilDiameter?: number;
  pupilCenter?: Vector2D;
  gazeOrigin?: Vector3D;
  gazeDirection?: Vector3D;
  eyeOpenness?: number;
  eyeOpennessMm?: number;
}

export interface GazePoint {
  timestamp: number;
  x: number;
  y: number;
  confidence: number;
  valid: boolean;
  leftEye?: EyeData;
  rightEye?: EyeData;
  fixation?: boolean;
  saccade?: boolean;
  fixationId?: string;
  deviceTimestamp?: number;
  metadata?: Record<string, unknown>;
}

export type InvalidReason =
  | 'blink'
  | 'tracking_lost'
  | 'out_of_range'
  | 'calibration_required'
  | 'unknown';

export interface GazeBatch {
  version: string;
  deviceId: string;
  sessionId: string;
  startTimestamp: number;
  endTimestamp: number;
  samplingRate: number;
  samples: GazePoint[];
}
```

---

## 10. Python 타입 정의

```python
# wia_eye_gaze/types.py

from dataclasses import dataclass
from typing import Optional, Dict, Any, List
from enum import Enum

@dataclass
class Vector2D:
    x: float
    y: float

@dataclass
class Vector3D:
    x: float
    y: float
    z: float

@dataclass
class BoundingBox:
    x: float
    y: float
    width: float
    height: float

@dataclass
class EyeData:
    gaze: Vector2D
    valid: bool
    pupil_diameter: Optional[float] = None
    pupil_center: Optional[Vector2D] = None
    gaze_origin: Optional[Vector3D] = None
    gaze_direction: Optional[Vector3D] = None
    eye_openness: Optional[float] = None
    eye_openness_mm: Optional[float] = None

@dataclass
class GazePoint:
    timestamp: int
    x: float
    y: float
    confidence: float
    valid: bool
    left_eye: Optional[EyeData] = None
    right_eye: Optional[EyeData] = None
    fixation: Optional[bool] = None
    saccade: Optional[bool] = None
    fixation_id: Optional[str] = None
    device_timestamp: Optional[int] = None
    metadata: Optional[Dict[str, Any]] = None

class InvalidReason(Enum):
    BLINK = "blink"
    TRACKING_LOST = "tracking_lost"
    OUT_OF_RANGE = "out_of_range"
    CALIBRATION_REQUIRED = "calibration_required"
    UNKNOWN = "unknown"

@dataclass
class GazeBatch:
    version: str
    device_id: str
    session_id: str
    start_timestamp: int
    end_timestamp: int
    sampling_rate: int
    samples: List[GazePoint]
```

---

<div align="center">

**WIA Eye Gaze Data Format Specification v1.0.0-draft**

**홍익인간** - 널리 인간을 이롭게

</div>
