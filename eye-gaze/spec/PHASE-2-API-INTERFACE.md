# WIA Eye Gaze Standard - Event Specification

**Version**: 1.0.0-draft
**Status**: Phase 1 Draft
**Last Updated**: 2025-01-XX

---

## 1. 개요 (Overview)

이 문서는 WIA Eye Gaze Interoperability Protocol의 이벤트 시스템을 정의합니다.
시선 추적에서 발생하는 다양한 이벤트를 표준화하여 애플리케이션 간 상호운용성을 보장합니다.

### 1.1 이벤트 카테고리

| 카테고리 | 설명 |
|---------|------|
| **Eye Movement** | 눈 움직임 관련 이벤트 (Fixation, Saccade) |
| **Eye State** | 눈 상태 이벤트 (Blink, Wink) |
| **Interaction** | 사용자 상호작용 이벤트 (Dwell, Selection) |
| **System** | 시스템/디바이스 이벤트 (Calibration, Status) |

---

## 2. 이벤트 타입 정의 (Event Types)

### 2.1 GazeEventType Enum

```typescript
enum GazeEventType {
  // === Eye Movement Events ===
  FIXATION_START = 'fixation_start',       // 응시 시작
  FIXATION_UPDATE = 'fixation_update',     // 응시 업데이트 (위치 변경)
  FIXATION_END = 'fixation_end',           // 응시 종료

  SACCADE_START = 'saccade_start',         // 빠른 이동 시작
  SACCADE_END = 'saccade_end',             // 빠른 이동 종료

  SMOOTH_PURSUIT_START = 'smooth_pursuit_start',  // 부드러운 추적 시작
  SMOOTH_PURSUIT_END = 'smooth_pursuit_end',      // 부드러운 추적 종료

  // === Eye State Events ===
  BLINK_START = 'blink_start',             // 눈 깜빡임 시작
  BLINK_END = 'blink_end',                 // 눈 깜빡임 종료
  BLINK = 'blink',                         // 눈 깜빡임 완료 (시작+종료 통합)

  WINK_LEFT = 'wink_left',                 // 왼쪽 눈 윙크
  WINK_RIGHT = 'wink_right',               // 오른쪽 눈 윙크

  DOUBLE_BLINK = 'double_blink',           // 이중 깜빡임

  // === Interaction Events ===
  DWELL_START = 'dwell_start',             // Dwell 시작 (타겟에 시선 진입)
  DWELL_PROGRESS = 'dwell_progress',       // Dwell 진행 중
  DWELL_COMPLETE = 'dwell_complete',       // Dwell 완료 (선택 트리거)
  DWELL_CANCEL = 'dwell_cancel',           // Dwell 취소 (시선 이탈)

  GAZE_ENTER = 'gaze_enter',               // 요소에 시선 진입
  GAZE_LEAVE = 'gaze_leave',               // 요소에서 시선 이탈

  // === System Events ===
  CALIBRATION_START = 'calibration_start', // 캘리브레이션 시작
  CALIBRATION_POINT = 'calibration_point', // 캘리브레이션 포인트
  CALIBRATION_END = 'calibration_end',     // 캘리브레이션 종료

  TRACKING_LOST = 'tracking_lost',         // 추적 손실
  TRACKING_RECOVERED = 'tracking_recovered', // 추적 복구

  DEVICE_CONNECTED = 'device_connected',   // 디바이스 연결
  DEVICE_DISCONNECTED = 'device_disconnected', // 디바이스 연결 해제
}
```

---

## 3. 이벤트 구조 (Event Structure)

### 3.1 기본 이벤트 구조

```typescript
interface GazeEvent {
  // === 필수 필드 ===

  /** 이벤트 타입 */
  type: GazeEventType;

  /** 이벤트 발생 시간 (Unix timestamp, ms) */
  timestamp: number;

  /** 고유 이벤트 ID */
  eventId: string;

  // === 선택 필드 ===

  /** 이벤트 지속 시간 (ms) */
  duration?: number;

  /** 이벤트 발생 위치 */
  position?: Vector2D;

  /** 타겟 요소 정보 */
  target?: GazeTarget;

  /** 관련 GazePoint 데이터 */
  gazeData?: GazePoint;

  /** 이전 이벤트 ID (연결된 이벤트) */
  previousEventId?: string;

  /** 추가 메타데이터 */
  metadata?: Record<string, unknown>;
}
```

### 3.2 GazeTarget 구조

이벤트가 발생한 대상 UI 요소를 표현합니다.

```typescript
interface GazeTarget {
  /** 요소 고유 ID */
  elementId: string;

  /** 요소 영역 */
  boundingBox: BoundingBox;

  /** 요소 시맨틱 타입 */
  semanticType: TargetSemanticType;

  /** 요소 레이블/이름 */
  label?: string;

  /** 접근성 정보 */
  accessibility?: AccessibilityInfo;

  /** 추가 속성 */
  attributes?: Record<string, unknown>;
}

type TargetSemanticType =
  | 'button'
  | 'link'
  | 'text'
  | 'input'
  | 'image'
  | 'video'
  | 'menu'
  | 'menuitem'
  | 'listitem'
  | 'scrollbar'
  | 'keyboard_key'  // AAC 키보드 키
  | 'aac_symbol'    // AAC 심볼
  | 'custom';

interface AccessibilityInfo {
  role?: string;
  name?: string;
  description?: string;
  state?: string[];
}
```

---

## 4. 이벤트 상세 명세 (Detailed Event Specifications)

### 4.1 Fixation Events

응시(Fixation)는 시선이 특정 위치에 일정 시간 이상 머무는 상태입니다.

```typescript
interface FixationEvent extends GazeEvent {
  type: 'fixation_start' | 'fixation_update' | 'fixation_end';

  /** Fixation ID (동일 응시에 대해 같은 ID) */
  fixationId: string;

  /** 응시 중심 좌표 */
  centroid: Vector2D;

  /** 응시 영역 반경 (정규화 좌표) */
  radius?: number;

  /** 응시 동안의 분산 (degrees) */
  dispersion?: number;

  /** fixation_end에만: 최종 지속 시간 (ms) */
  duration?: number;
}
```

**Fixation 판정 기준**:
- 최소 지속 시간: 100ms (설정 가능)
- 최대 분산: 1.0도 (설정 가능)

### 4.2 Saccade Events

Saccade는 한 위치에서 다른 위치로의 빠른 눈 움직임입니다.

```typescript
interface SaccadeEvent extends GazeEvent {
  type: 'saccade_start' | 'saccade_end';

  /** Saccade ID */
  saccadeId: string;

  /** 시작 위치 */
  startPosition: Vector2D;

  /** 종료 위치 (saccade_end에서) */
  endPosition?: Vector2D;

  /** 이동 거리 (degrees) */
  amplitude?: number;

  /** 최대 속도 (degrees/second) */
  peakVelocity?: number;

  /** 지속 시간 (ms) */
  duration?: number;
}
```

### 4.3 Blink Events

눈 깜빡임 이벤트입니다.

```typescript
interface BlinkEvent extends GazeEvent {
  type: 'blink_start' | 'blink_end' | 'blink' | 'double_blink';

  /** Blink ID */
  blinkId: string;

  /** 어느 눈이 깜빡였는지 */
  eye: 'left' | 'right' | 'both';

  /** 깜빡임 지속 시간 (ms) */
  duration?: number;

  /** 이중 깜빡임의 경우 간격 (ms) */
  interval?: number;
}
```

**일반적인 깜빡임 특성**:
- 정상 깜빡임 지속 시간: 100-400ms
- 의도적 깜빡임: 400ms 이상

### 4.4 Wink Events

윙크는 한쪽 눈만 의도적으로 감는 동작입니다.

```typescript
interface WinkEvent extends GazeEvent {
  type: 'wink_left' | 'wink_right';

  /** Wink ID */
  winkId: string;

  /** 윙크 지속 시간 (ms) */
  duration: number;

  /** 의도적 여부 추정 (400ms 이상이면 true) */
  intentional: boolean;
}
```

### 4.5 Dwell Events

Dwell은 시선 기반 선택의 핵심 메커니즘입니다.

```typescript
interface DwellEvent extends GazeEvent {
  type: 'dwell_start' | 'dwell_progress' | 'dwell_complete' | 'dwell_cancel';

  /** Dwell ID */
  dwellId: string;

  /** 타겟 요소 */
  target: GazeTarget;

  /** 설정된 Dwell 시간 (ms) */
  dwellThreshold: number;

  /** 현재까지 경과 시간 (ms) */
  elapsed: number;

  /** 진행률 (0.0 - 1.0) */
  progress: number;

  /** dwell_cancel의 경우: 취소 사유 */
  cancelReason?: 'gaze_left' | 'timeout' | 'user_cancel';
}
```

**Dwell 흐름**:
```
gaze_enter → dwell_start → dwell_progress (반복) → dwell_complete
                                                 → dwell_cancel (시선 이탈 시)
```

### 4.6 Gaze Enter/Leave Events

UI 요소에 대한 시선 진입/이탈 이벤트입니다.

```typescript
interface GazeEnterLeaveEvent extends GazeEvent {
  type: 'gaze_enter' | 'gaze_leave';

  /** 타겟 요소 */
  target: GazeTarget;

  /** gaze_leave의 경우: 머문 시간 (ms) */
  dwellTime?: number;

  /** 다음 타겟 (gaze_leave에서, 알 수 있는 경우) */
  nextTarget?: GazeTarget;
}
```

### 4.7 Calibration Events

캘리브레이션 과정의 이벤트입니다.

```typescript
interface CalibrationEvent extends GazeEvent {
  type: 'calibration_start' | 'calibration_point' | 'calibration_end';

  /** 캘리브레이션 세션 ID */
  calibrationId: string;

  /** 전체 포인트 수 */
  totalPoints?: number;

  /** 현재 포인트 인덱스 (1-based) */
  currentPoint?: number;

  /** 포인트 위치 */
  pointPosition?: Vector2D;

  /** 캘리브레이션 결과 (calibration_end에서) */
  result?: CalibrationResult;
}

interface CalibrationResult {
  /** 성공 여부 */
  success: boolean;

  /** 평균 정확도 (degrees) */
  averageAccuracy?: number;

  /** 평균 정밀도 (degrees) */
  averagePrecision?: number;

  /** 포인트별 결과 */
  pointResults?: PointCalibrationResult[];
}

interface PointCalibrationResult {
  pointIndex: number;
  position: Vector2D;
  accuracy: number;
  precision: number;
  valid: boolean;
}
```

### 4.8 System Events

시스템 및 디바이스 상태 이벤트입니다.

```typescript
interface SystemEvent extends GazeEvent {
  type: 'tracking_lost' | 'tracking_recovered' | 'device_connected' | 'device_disconnected';

  /** 디바이스 ID */
  deviceId: string;

  /** 상태 정보 */
  status?: DeviceStatus;

  /** 오류 정보 (해당 시) */
  error?: ErrorInfo;
}

interface DeviceStatus {
  connected: boolean;
  tracking: boolean;
  calibrated: boolean;
  batteryLevel?: number;  // 0-100
}

interface ErrorInfo {
  code: string;
  message: string;
  recoverable: boolean;
}
```

---

## 5. 이벤트 흐름 예시 (Event Flow Examples)

### 5.1 버튼 클릭 (Dwell Selection)

```
User looks at button
    │
    ▼
gaze_enter {target: button_1}
    │
    ▼
dwell_start {target: button_1, threshold: 800}
    │
    ├─(시선 유지)─▶ dwell_progress {progress: 0.25}
    │               dwell_progress {progress: 0.50}
    │               dwell_progress {progress: 0.75}
    │               dwell_progress {progress: 1.00}
    │                   │
    │                   ▼
    │               dwell_complete {target: button_1}
    │
    └─(시선 이탈)─▶ dwell_cancel {reason: 'gaze_left'}
                        │
                        ▼
                    gaze_leave {target: button_1}
```

### 5.2 텍스트 읽기

```
fixation_start {centroid: (0.2, 0.3)}
fixation_end {duration: 250ms}
    │
saccade_start {startPosition: (0.2, 0.3)}
saccade_end {endPosition: (0.35, 0.3), amplitude: 2.5°}
    │
fixation_start {centroid: (0.35, 0.3)}
fixation_end {duration: 200ms}
    │
... (반복)
```

### 5.3 의도적 깜빡임 선택

```
fixation_start {target: button_1}
    │
    ▼
blink_start {eye: 'both'}
blink_end {duration: 450ms}  // > 400ms = intentional
    │
    ▼
blink {intentional: true}
    │
    ▼
[Application triggers selection]
```

---

## 6. 이벤트 설정 (Event Configuration)

### 6.1 Fixation 설정

```typescript
interface FixationConfig {
  /** 최소 응시 시간 (ms) */
  minDuration: number;  // default: 100

  /** 최대 분산 (degrees) */
  maxDispersion: number;  // default: 1.0

  /** 병합 거리 (degrees) - 가까운 fixation 병합 */
  mergeDistance: number;  // default: 0.5
}
```

### 6.2 Dwell 설정

```typescript
interface DwellConfig {
  /** Dwell 선택 활성화 시간 (ms) */
  threshold: number;  // default: 800

  /** Dwell 진행 업데이트 간격 (ms) */
  progressInterval: number;  // default: 100

  /** 시각적 피드백 활성화 */
  visualFeedback: boolean;  // default: true

  /** 선택 후 대기 시간 (ms) - 재선택 방지 */
  cooldownPeriod: number;  // default: 500
}
```

### 6.3 Blink 설정

```typescript
interface BlinkConfig {
  /** 의도적 깜빡임 판정 시간 (ms) */
  intentionalThreshold: number;  // default: 400

  /** 이중 깜빡임 최대 간격 (ms) */
  doubleBlinkInterval: number;  // default: 500

  /** 깜빡임 입력 활성화 */
  blinkAsInput: boolean;  // default: false
}
```

---

## 7. TypeScript 전체 타입 정의

```typescript
// wia-eye-gaze-events.ts

export enum GazeEventType {
  FIXATION_START = 'fixation_start',
  FIXATION_UPDATE = 'fixation_update',
  FIXATION_END = 'fixation_end',
  SACCADE_START = 'saccade_start',
  SACCADE_END = 'saccade_end',
  SMOOTH_PURSUIT_START = 'smooth_pursuit_start',
  SMOOTH_PURSUIT_END = 'smooth_pursuit_end',
  BLINK_START = 'blink_start',
  BLINK_END = 'blink_end',
  BLINK = 'blink',
  WINK_LEFT = 'wink_left',
  WINK_RIGHT = 'wink_right',
  DOUBLE_BLINK = 'double_blink',
  DWELL_START = 'dwell_start',
  DWELL_PROGRESS = 'dwell_progress',
  DWELL_COMPLETE = 'dwell_complete',
  DWELL_CANCEL = 'dwell_cancel',
  GAZE_ENTER = 'gaze_enter',
  GAZE_LEAVE = 'gaze_leave',
  CALIBRATION_START = 'calibration_start',
  CALIBRATION_POINT = 'calibration_point',
  CALIBRATION_END = 'calibration_end',
  TRACKING_LOST = 'tracking_lost',
  TRACKING_RECOVERED = 'tracking_recovered',
  DEVICE_CONNECTED = 'device_connected',
  DEVICE_DISCONNECTED = 'device_disconnected',
}

export type TargetSemanticType =
  | 'button' | 'link' | 'text' | 'input' | 'image'
  | 'video' | 'menu' | 'menuitem' | 'listitem' | 'scrollbar'
  | 'keyboard_key' | 'aac_symbol' | 'custom';

export interface AccessibilityInfo {
  role?: string;
  name?: string;
  description?: string;
  state?: string[];
}

export interface GazeTarget {
  elementId: string;
  boundingBox: BoundingBox;
  semanticType: TargetSemanticType;
  label?: string;
  accessibility?: AccessibilityInfo;
  attributes?: Record<string, unknown>;
}

export interface GazeEvent {
  type: GazeEventType;
  timestamp: number;
  eventId: string;
  duration?: number;
  position?: Vector2D;
  target?: GazeTarget;
  gazeData?: GazePoint;
  previousEventId?: string;
  metadata?: Record<string, unknown>;
}

export interface FixationConfig {
  minDuration: number;
  maxDispersion: number;
  mergeDistance: number;
}

export interface DwellConfig {
  threshold: number;
  progressInterval: number;
  visualFeedback: boolean;
  cooldownPeriod: number;
}

export interface BlinkConfig {
  intentionalThreshold: number;
  doubleBlinkInterval: number;
  blinkAsInput: boolean;
}
```

---

## 8. Python 타입 정의

```python
# wia_eye_gaze/events.py

from dataclasses import dataclass
from typing import Optional, Dict, Any, List
from enum import Enum

class GazeEventType(Enum):
    FIXATION_START = "fixation_start"
    FIXATION_UPDATE = "fixation_update"
    FIXATION_END = "fixation_end"
    SACCADE_START = "saccade_start"
    SACCADE_END = "saccade_end"
    SMOOTH_PURSUIT_START = "smooth_pursuit_start"
    SMOOTH_PURSUIT_END = "smooth_pursuit_end"
    BLINK_START = "blink_start"
    BLINK_END = "blink_end"
    BLINK = "blink"
    WINK_LEFT = "wink_left"
    WINK_RIGHT = "wink_right"
    DOUBLE_BLINK = "double_blink"
    DWELL_START = "dwell_start"
    DWELL_PROGRESS = "dwell_progress"
    DWELL_COMPLETE = "dwell_complete"
    DWELL_CANCEL = "dwell_cancel"
    GAZE_ENTER = "gaze_enter"
    GAZE_LEAVE = "gaze_leave"
    CALIBRATION_START = "calibration_start"
    CALIBRATION_POINT = "calibration_point"
    CALIBRATION_END = "calibration_end"
    TRACKING_LOST = "tracking_lost"
    TRACKING_RECOVERED = "tracking_recovered"
    DEVICE_CONNECTED = "device_connected"
    DEVICE_DISCONNECTED = "device_disconnected"

class TargetSemanticType(Enum):
    BUTTON = "button"
    LINK = "link"
    TEXT = "text"
    INPUT = "input"
    IMAGE = "image"
    VIDEO = "video"
    MENU = "menu"
    MENUITEM = "menuitem"
    LISTITEM = "listitem"
    SCROLLBAR = "scrollbar"
    KEYBOARD_KEY = "keyboard_key"
    AAC_SYMBOL = "aac_symbol"
    CUSTOM = "custom"

@dataclass
class AccessibilityInfo:
    role: Optional[str] = None
    name: Optional[str] = None
    description: Optional[str] = None
    state: Optional[List[str]] = None

@dataclass
class GazeTarget:
    element_id: str
    bounding_box: "BoundingBox"
    semantic_type: TargetSemanticType
    label: Optional[str] = None
    accessibility: Optional[AccessibilityInfo] = None
    attributes: Optional[Dict[str, Any]] = None

@dataclass
class GazeEvent:
    type: GazeEventType
    timestamp: int
    event_id: str
    duration: Optional[int] = None
    position: Optional["Vector2D"] = None
    target: Optional[GazeTarget] = None
    gaze_data: Optional["GazePoint"] = None
    previous_event_id: Optional[str] = None
    metadata: Optional[Dict[str, Any]] = None

@dataclass
class FixationConfig:
    min_duration: int = 100
    max_dispersion: float = 1.0
    merge_distance: float = 0.5

@dataclass
class DwellConfig:
    threshold: int = 800
    progress_interval: int = 100
    visual_feedback: bool = True
    cooldown_period: int = 500

@dataclass
class BlinkConfig:
    intentional_threshold: int = 400
    double_blink_interval: int = 500
    blink_as_input: bool = False
```

---

<div align="center">

**WIA Eye Gaze Event Specification v1.0.0-draft**

**홍익인간** - 널리 인간을 이롭게

</div>
