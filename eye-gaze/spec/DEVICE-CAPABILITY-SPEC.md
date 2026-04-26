# WIA Eye Gaze Standard - Device Capability Specification

**Version**: 1.0.0-draft
**Status**: Phase 1 Draft
**Last Updated**: 2025-01-XX

---

## 1. 개요 (Overview)

이 문서는 시선 추적 디바이스의 역량(Capability)을 표준화된 방식으로 표현하는 방법을 정의합니다.
애플리케이션이 디바이스의 기능을 자동으로 인식하고 적절히 대응할 수 있도록 합니다.

### 1.1 목적

- 디바이스 기능의 **자동 검색(Discovery)**
- 애플리케이션의 **호환성 확인**
- **적응형 UI/UX** 구현 지원
- 사용자에게 **디바이스 정보 제공**

---

## 2. 디바이스 정보 구조 (Device Information Structure)

### 2.1 EyeTrackerInfo

디바이스의 기본 정보를 담습니다.

```typescript
interface EyeTrackerInfo {
  // === 필수 필드 ===

  /** 디바이스 고유 ID */
  deviceId: string;

  /** 제조사 */
  vendor: string;

  /** 모델명 */
  model: string;

  /** 펌웨어 버전 */
  firmwareVersion: string;

  /** WIA 프로토콜 버전 */
  protocolVersion: string;

  // === 선택 필드 ===

  /** 시리얼 번호 */
  serialNumber?: string;

  /** 디바이스 유형 */
  deviceType?: DeviceType;

  /** 제조사 URL */
  vendorUrl?: string;

  /** 제품 URL */
  productUrl?: string;

  /** 추가 정보 */
  metadata?: Record<string, unknown>;
}

type DeviceType =
  | 'screen_based'      // 화면 장착형 (Tobii Pro, Gazepoint 등)
  | 'wearable'          // 착용형 (Tobii Glasses, Pupil Labs 등)
  | 'remote'            // 원격형 (독립 장치)
  | 'integrated'        // 통합형 (노트북 내장 등)
  | 'webcam_based'      // 웹캠 기반 (소프트웨어 솔루션)
  | 'unknown';
```

---

## 3. 역량 구조 (Capability Structure)

### 3.1 EyeTrackerCapabilities

디바이스의 모든 역량을 포괄하는 구조입니다.

```typescript
interface EyeTrackerCapabilities {
  /** 기본 디바이스 정보 */
  device: EyeTrackerInfo;

  /** 추적 역량 */
  tracking: TrackingCapabilities;

  /** 데이터 역량 */
  data: DataCapabilities;

  /** 캘리브레이션 역량 */
  calibration: CalibrationCapabilities;

  /** 접근성 역량 (AAC 관련) */
  accessibility?: AccessibilityCapabilities;

  /** 연결 역량 */
  connectivity: ConnectivityCapabilities;

  /** 지원 기능 목록 */
  supportedFeatures: FeatureFlag[];
}
```

### 3.2 TrackingCapabilities

추적 관련 역량입니다.

```typescript
interface TrackingCapabilities {
  // === 추적 모드 ===

  /** 양안 추적 지원 */
  binocular: boolean;

  /** 머리 추적 지원 */
  headTracking: boolean;

  /** 3D 시선 추적 지원 */
  gaze3D: boolean;

  // === 성능 스펙 ===

  /** 샘플링 레이트 (Hz) */
  samplingRate: SamplingRateSpec;

  /** 정확도 (degrees) */
  accuracy: AccuracySpec;

  /** 정밀도 (degrees) */
  precision: PrecisionSpec;

  /** 지연 시간 (ms) */
  latency: LatencySpec;

  // === 작동 범위 ===

  /** 작동 거리 범위 (cm) */
  operatingDistance: RangeSpec;

  /** 추적 가능 영역 (degrees) */
  trackingArea: TrackingAreaSpec;

  /** 허용 머리 움직임 */
  headMovementTolerance?: HeadMovementSpec;
}

interface SamplingRateSpec {
  /** 지원하는 샘플링 레이트 목록 */
  supported: number[];

  /** 기본값 */
  default: number;

  /** 현재 설정값 */
  current?: number;
}

interface AccuracySpec {
  /** 일반적 정확도 (degrees) */
  typical: number;

  /** 최상 조건 정확도 */
  best?: number;

  /** 측정 방법 */
  measurementMethod?: string;
}

interface PrecisionSpec {
  /** 일반적 정밀도 (degrees) */
  typical: number;

  /** RMS 정밀도 */
  rms?: number;
}

interface LatencySpec {
  /** 평균 지연 (ms) */
  average: number;

  /** 최대 지연 (ms) */
  maximum?: number;
}

interface RangeSpec {
  min: number;
  max: number;
  optimal?: number;
}

interface TrackingAreaSpec {
  /** 수평 추적 범위 (degrees) */
  horizontal: number;

  /** 수직 추적 범위 (degrees) */
  vertical: number;
}

interface HeadMovementSpec {
  /** 수평 이동 허용 범위 (cm) */
  horizontal: number;

  /** 수직 이동 허용 범위 (cm) */
  vertical: number;

  /** 깊이 이동 허용 범위 (cm) */
  depth: number;
}
```

### 3.3 DataCapabilities

데이터 출력 역량입니다.

```typescript
interface DataCapabilities {
  // === 데이터 스트림 ===

  /** 시선 포인트 데이터 */
  gazePoint: boolean;

  /** 개별 눈 데이터 */
  eyeData: boolean;

  /** 동공 직경 */
  pupilDiameter: boolean;

  /** 동공 위치 */
  pupilPosition: boolean;

  /** 눈 열림 정도 */
  eyeOpenness: boolean;

  /** 눈 이미지 스트림 */
  eyeImages: boolean;

  /** 3D 시선 원점 */
  gazeOrigin3D: boolean;

  /** 3D 시선 방향 */
  gazeDirection3D: boolean;

  // === 이벤트 검출 ===

  /** 내장 Fixation 검출 */
  builtInFixationDetection: boolean;

  /** 내장 Saccade 검출 */
  builtInSaccadeDetection: boolean;

  /** 내장 Blink 검출 */
  builtInBlinkDetection: boolean;

  // === 타임스탬프 ===

  /** 디바이스 타임스탬프 지원 */
  deviceTimestamp: boolean;

  /** 시스템 타임스탬프 지원 */
  systemTimestamp: boolean;

  /** 외부 동기화 신호 지원 */
  externalSync: boolean;
}
```

### 3.4 CalibrationCapabilities

캘리브레이션 역량입니다.

```typescript
interface CalibrationCapabilities {
  /** 캘리브레이션 필요 여부 */
  required: boolean;

  /** 지원 캘리브레이션 유형 */
  types: CalibrationType[];

  /** 지원 포인트 수 옵션 */
  pointOptions: number[];

  /** 기본 포인트 수 */
  defaultPoints: number;

  /** 자동 캘리브레이션 지원 */
  autoCalibration: boolean;

  /** 캘리브레이션 저장/불러오기 */
  profileManagement: boolean;

  /** 캘리브레이션 품질 평가 */
  qualityAssessment: boolean;

  /** 적응형 캘리브레이션 (AAC용) */
  adaptiveCalibration: boolean;
}

type CalibrationType =
  | 'standard'          // 표준 포인트 캘리브레이션
  | 'quick'             // 빠른 캘리브레이션 (적은 포인트)
  | 'infant'            // 영유아용
  | 'gaming'            // 게이밍 (빠르고 간편)
  | 'accessibility'     // 접근성 (AAC용, 큰 타겟)
  | 'automatic';        // 자동 (사용자 입력 최소)
```

### 3.5 AccessibilityCapabilities

접근성 및 AAC 관련 역량입니다.

```typescript
interface AccessibilityCapabilities {
  // === 대체 입력 ===

  /** Dwell 선택 내장 지원 */
  dwellSelection: boolean;

  /** 깜빡임 입력 지원 */
  blinkInput: boolean;

  /** 윙크 입력 지원 */
  winkInput: boolean;

  /** 스위치 에뮬레이션 */
  switchEmulation: boolean;

  // === 적응형 기능 ===

  /** 적응형 Dwell 시간 */
  adaptiveDwellTime: boolean;

  /** 적응형 타겟 크기 */
  adaptiveTargetSize: boolean;

  /** 오류 보정/스무딩 */
  errorSmoothing: boolean;

  /** 떨림/불수의 운동 보정 */
  tremorCompensation: boolean;

  // === AAC 특화 ===

  /** AAC 소프트웨어 최적화 모드 */
  aacOptimizedMode: boolean;

  /** 긴 사용 세션 최적화 */
  longSessionOptimization: boolean;

  /** 피로도 감지 */
  fatigueDetection: boolean;

  /** 자동 휴식 알림 */
  breakReminder: boolean;
}
```

### 3.6 ConnectivityCapabilities

연결 역량입니다.

```typescript
interface ConnectivityCapabilities {
  /** 연결 유형 */
  connectionTypes: ConnectionType[];

  /** API 프로토콜 */
  apiProtocols: ApiProtocol[];

  /** 다중 클라이언트 지원 */
  multiClient: boolean;

  /** 원격 연결 지원 */
  remoteConnection: boolean;

  /** 모바일 연결 지원 */
  mobileConnection: boolean;
}

type ConnectionType =
  | 'usb'
  | 'usb_c'
  | 'bluetooth'
  | 'wifi'
  | 'ethernet'
  | 'hdmi';

type ApiProtocol =
  | 'native_sdk'        // 제조사 네이티브 SDK
  | 'wia_standard'      // WIA 표준 프로토콜
  | 'tcp_ip'            // TCP/IP 소켓
  | 'websocket'         // WebSocket
  | 'rest_api'          // REST API
  | 'webrtc';           // WebRTC
```

### 3.7 FeatureFlag

지원 기능 플래그입니다.

```typescript
type FeatureFlag =
  // 기본 기능
  | 'GAZE_POINT'
  | 'BINOCULAR'
  | 'HEAD_TRACKING'
  | 'GAZE_3D'

  // 동공 데이터
  | 'PUPIL_DIAMETER'
  | 'PUPIL_POSITION'

  // 눈 상태
  | 'EYE_OPENNESS'
  | 'EYE_IMAGES'

  // 이벤트 검출
  | 'FIXATION_DETECTION'
  | 'SACCADE_DETECTION'
  | 'BLINK_DETECTION'

  // 캘리브레이션
  | 'CALIBRATION'
  | 'AUTO_CALIBRATION'
  | 'CALIBRATION_PROFILES'

  // 접근성
  | 'DWELL_SELECTION'
  | 'BLINK_INPUT'
  | 'AAC_MODE'
  | 'TREMOR_COMPENSATION'

  // 동기화
  | 'DEVICE_TIMESTAMP'
  | 'EXTERNAL_SYNC'

  // 기타
  | 'MULTI_MONITOR'
  | 'SCREEN_RECORDING'
  | 'HEATMAP_GENERATION';
```

---

## 4. 역량 조회 및 협상 (Capability Query and Negotiation)

### 4.1 역량 조회 요청

```typescript
interface CapabilityQuery {
  /** 쿼리 타입 */
  queryType: 'full' | 'summary' | 'specific';

  /** specific인 경우 조회할 역량 목록 */
  capabilities?: string[];
}
```

### 4.2 역량 조회 응답

```json
{
  "device": {
    "deviceId": "tobii-pro-fusion-001",
    "vendor": "Tobii",
    "model": "Pro Fusion",
    "firmwareVersion": "2.1.0",
    "protocolVersion": "1.0.0",
    "deviceType": "screen_based"
  },
  "tracking": {
    "binocular": true,
    "headTracking": true,
    "gaze3D": true,
    "samplingRate": {
      "supported": [60, 120, 250],
      "default": 120,
      "current": 120
    },
    "accuracy": {
      "typical": 0.5,
      "best": 0.3
    },
    "precision": {
      "typical": 0.1,
      "rms": 0.08
    },
    "latency": {
      "average": 3,
      "maximum": 10
    },
    "operatingDistance": {
      "min": 50,
      "max": 80,
      "optimal": 65
    },
    "trackingArea": {
      "horizontal": 30,
      "vertical": 25
    }
  },
  "data": {
    "gazePoint": true,
    "eyeData": true,
    "pupilDiameter": true,
    "pupilPosition": true,
    "eyeOpenness": true,
    "eyeImages": true,
    "gazeOrigin3D": true,
    "gazeDirection3D": true,
    "builtInFixationDetection": false,
    "builtInSaccadeDetection": false,
    "builtInBlinkDetection": false,
    "deviceTimestamp": true,
    "systemTimestamp": true,
    "externalSync": true
  },
  "calibration": {
    "required": true,
    "types": ["standard", "quick", "accessibility"],
    "pointOptions": [1, 2, 5, 9],
    "defaultPoints": 5,
    "autoCalibration": false,
    "profileManagement": true,
    "qualityAssessment": true,
    "adaptiveCalibration": true
  },
  "accessibility": {
    "dwellSelection": false,
    "blinkInput": false,
    "winkInput": false,
    "switchEmulation": false,
    "adaptiveDwellTime": false,
    "adaptiveTargetSize": false,
    "errorSmoothing": true,
    "tremorCompensation": false,
    "aacOptimizedMode": false,
    "longSessionOptimization": false,
    "fatigueDetection": false,
    "breakReminder": false
  },
  "connectivity": {
    "connectionTypes": ["usb_c"],
    "apiProtocols": ["native_sdk", "wia_standard"],
    "multiClient": true,
    "remoteConnection": false,
    "mobileConnection": false
  },
  "supportedFeatures": [
    "GAZE_POINT",
    "BINOCULAR",
    "HEAD_TRACKING",
    "GAZE_3D",
    "PUPIL_DIAMETER",
    "PUPIL_POSITION",
    "EYE_OPENNESS",
    "EYE_IMAGES",
    "CALIBRATION",
    "CALIBRATION_PROFILES",
    "DEVICE_TIMESTAMP",
    "EXTERNAL_SYNC"
  ]
}
```

### 4.3 역량 협상 (Capability Negotiation)

애플리케이션과 디바이스 간 역량 협상입니다.

```typescript
interface CapabilityNegotiation {
  /** 애플리케이션이 요구하는 역량 */
  required: FeatureFlag[];

  /** 선호하지만 필수 아닌 역량 */
  preferred: FeatureFlag[];

  /** 원하는 샘플링 레이트 */
  preferredSamplingRate?: number;
}

interface NegotiationResult {
  /** 협상 성공 여부 */
  success: boolean;

  /** 충족된 필수 역량 */
  satisfiedRequired: FeatureFlag[];

  /** 미충족 필수 역량 */
  unsatisfiedRequired: FeatureFlag[];

  /** 충족된 선호 역량 */
  satisfiedPreferred: FeatureFlag[];

  /** 설정된 샘플링 레이트 */
  negotiatedSamplingRate?: number;

  /** 경고 메시지 */
  warnings?: string[];
}
```

---

## 5. 일반 디바이스 프로파일 (Common Device Profiles)

### 5.1 화면 장착형 - 고급 연구용

```typescript
const researchGradeProfile: Partial<EyeTrackerCapabilities> = {
  tracking: {
    binocular: true,
    headTracking: true,
    gaze3D: true,
    samplingRate: { supported: [120, 250, 600], default: 250 },
    accuracy: { typical: 0.4 },
    precision: { typical: 0.08 },
  },
  data: {
    gazePoint: true,
    eyeData: true,
    pupilDiameter: true,
    eyeOpenness: true,
    eyeImages: true,
    gazeOrigin3D: true,
    externalSync: true,
  },
};
```

### 5.2 화면 장착형 - AAC 전용

```typescript
const aacOptimizedProfile: Partial<EyeTrackerCapabilities> = {
  tracking: {
    binocular: true,
    headTracking: true,
    samplingRate: { supported: [30, 60], default: 60 },
    accuracy: { typical: 1.0 },
    operatingDistance: { min: 40, max: 100, optimal: 60 },
  },
  accessibility: {
    dwellSelection: true,
    adaptiveDwellTime: true,
    adaptiveTargetSize: true,
    errorSmoothing: true,
    tremorCompensation: true,
    aacOptimizedMode: true,
    longSessionOptimization: true,
    fatigueDetection: true,
    breakReminder: true,
  },
};
```

### 5.3 웹캠 기반 - 보급형

```typescript
const webcamBasedProfile: Partial<EyeTrackerCapabilities> = {
  device: {
    deviceType: 'webcam_based',
  },
  tracking: {
    binocular: false,
    headTracking: true,
    gaze3D: false,
    samplingRate: { supported: [30], default: 30 },
    accuracy: { typical: 2.0 },
    precision: { typical: 0.5 },
  },
  calibration: {
    required: true,
    types: ['standard', 'quick'],
    autoCalibration: false,
  },
};
```

---

## 6. TypeScript 전체 타입 정의

```typescript
// wia-eye-gaze-capability.ts

export type DeviceType =
  | 'screen_based' | 'wearable' | 'remote'
  | 'integrated' | 'webcam_based' | 'unknown';

export type CalibrationType =
  | 'standard' | 'quick' | 'infant'
  | 'gaming' | 'accessibility' | 'automatic';

export type ConnectionType =
  | 'usb' | 'usb_c' | 'bluetooth'
  | 'wifi' | 'ethernet' | 'hdmi';

export type ApiProtocol =
  | 'native_sdk' | 'wia_standard' | 'tcp_ip'
  | 'websocket' | 'rest_api' | 'webrtc';

export type FeatureFlag =
  | 'GAZE_POINT' | 'BINOCULAR' | 'HEAD_TRACKING' | 'GAZE_3D'
  | 'PUPIL_DIAMETER' | 'PUPIL_POSITION'
  | 'EYE_OPENNESS' | 'EYE_IMAGES'
  | 'FIXATION_DETECTION' | 'SACCADE_DETECTION' | 'BLINK_DETECTION'
  | 'CALIBRATION' | 'AUTO_CALIBRATION' | 'CALIBRATION_PROFILES'
  | 'DWELL_SELECTION' | 'BLINK_INPUT' | 'AAC_MODE' | 'TREMOR_COMPENSATION'
  | 'DEVICE_TIMESTAMP' | 'EXTERNAL_SYNC'
  | 'MULTI_MONITOR' | 'SCREEN_RECORDING' | 'HEATMAP_GENERATION';

export interface EyeTrackerInfo {
  deviceId: string;
  vendor: string;
  model: string;
  firmwareVersion: string;
  protocolVersion: string;
  serialNumber?: string;
  deviceType?: DeviceType;
  vendorUrl?: string;
  productUrl?: string;
  metadata?: Record<string, unknown>;
}

export interface SamplingRateSpec {
  supported: number[];
  default: number;
  current?: number;
}

export interface AccuracySpec {
  typical: number;
  best?: number;
  measurementMethod?: string;
}

export interface PrecisionSpec {
  typical: number;
  rms?: number;
}

export interface LatencySpec {
  average: number;
  maximum?: number;
}

export interface RangeSpec {
  min: number;
  max: number;
  optimal?: number;
}

export interface TrackingAreaSpec {
  horizontal: number;
  vertical: number;
}

export interface HeadMovementSpec {
  horizontal: number;
  vertical: number;
  depth: number;
}

export interface TrackingCapabilities {
  binocular: boolean;
  headTracking: boolean;
  gaze3D: boolean;
  samplingRate: SamplingRateSpec;
  accuracy: AccuracySpec;
  precision: PrecisionSpec;
  latency: LatencySpec;
  operatingDistance: RangeSpec;
  trackingArea: TrackingAreaSpec;
  headMovementTolerance?: HeadMovementSpec;
}

export interface DataCapabilities {
  gazePoint: boolean;
  eyeData: boolean;
  pupilDiameter: boolean;
  pupilPosition: boolean;
  eyeOpenness: boolean;
  eyeImages: boolean;
  gazeOrigin3D: boolean;
  gazeDirection3D: boolean;
  builtInFixationDetection: boolean;
  builtInSaccadeDetection: boolean;
  builtInBlinkDetection: boolean;
  deviceTimestamp: boolean;
  systemTimestamp: boolean;
  externalSync: boolean;
}

export interface CalibrationCapabilities {
  required: boolean;
  types: CalibrationType[];
  pointOptions: number[];
  defaultPoints: number;
  autoCalibration: boolean;
  profileManagement: boolean;
  qualityAssessment: boolean;
  adaptiveCalibration: boolean;
}

export interface AccessibilityCapabilities {
  dwellSelection: boolean;
  blinkInput: boolean;
  winkInput: boolean;
  switchEmulation: boolean;
  adaptiveDwellTime: boolean;
  adaptiveTargetSize: boolean;
  errorSmoothing: boolean;
  tremorCompensation: boolean;
  aacOptimizedMode: boolean;
  longSessionOptimization: boolean;
  fatigueDetection: boolean;
  breakReminder: boolean;
}

export interface ConnectivityCapabilities {
  connectionTypes: ConnectionType[];
  apiProtocols: ApiProtocol[];
  multiClient: boolean;
  remoteConnection: boolean;
  mobileConnection: boolean;
}

export interface EyeTrackerCapabilities {
  device: EyeTrackerInfo;
  tracking: TrackingCapabilities;
  data: DataCapabilities;
  calibration: CalibrationCapabilities;
  accessibility?: AccessibilityCapabilities;
  connectivity: ConnectivityCapabilities;
  supportedFeatures: FeatureFlag[];
}
```

---

## 7. Python 타입 정의

```python
# wia_eye_gaze/capability.py

from dataclasses import dataclass
from typing import Optional, Dict, Any, List
from enum import Enum

class DeviceType(Enum):
    SCREEN_BASED = "screen_based"
    WEARABLE = "wearable"
    REMOTE = "remote"
    INTEGRATED = "integrated"
    WEBCAM_BASED = "webcam_based"
    UNKNOWN = "unknown"

class CalibrationType(Enum):
    STANDARD = "standard"
    QUICK = "quick"
    INFANT = "infant"
    GAMING = "gaming"
    ACCESSIBILITY = "accessibility"
    AUTOMATIC = "automatic"

class ConnectionType(Enum):
    USB = "usb"
    USB_C = "usb_c"
    BLUETOOTH = "bluetooth"
    WIFI = "wifi"
    ETHERNET = "ethernet"
    HDMI = "hdmi"

class ApiProtocol(Enum):
    NATIVE_SDK = "native_sdk"
    WIA_STANDARD = "wia_standard"
    TCP_IP = "tcp_ip"
    WEBSOCKET = "websocket"
    REST_API = "rest_api"
    WEBRTC = "webrtc"

class FeatureFlag(Enum):
    GAZE_POINT = "GAZE_POINT"
    BINOCULAR = "BINOCULAR"
    HEAD_TRACKING = "HEAD_TRACKING"
    GAZE_3D = "GAZE_3D"
    PUPIL_DIAMETER = "PUPIL_DIAMETER"
    PUPIL_POSITION = "PUPIL_POSITION"
    EYE_OPENNESS = "EYE_OPENNESS"
    EYE_IMAGES = "EYE_IMAGES"
    FIXATION_DETECTION = "FIXATION_DETECTION"
    SACCADE_DETECTION = "SACCADE_DETECTION"
    BLINK_DETECTION = "BLINK_DETECTION"
    CALIBRATION = "CALIBRATION"
    AUTO_CALIBRATION = "AUTO_CALIBRATION"
    CALIBRATION_PROFILES = "CALIBRATION_PROFILES"
    DWELL_SELECTION = "DWELL_SELECTION"
    BLINK_INPUT = "BLINK_INPUT"
    AAC_MODE = "AAC_MODE"
    TREMOR_COMPENSATION = "TREMOR_COMPENSATION"
    DEVICE_TIMESTAMP = "DEVICE_TIMESTAMP"
    EXTERNAL_SYNC = "EXTERNAL_SYNC"
    MULTI_MONITOR = "MULTI_MONITOR"
    SCREEN_RECORDING = "SCREEN_RECORDING"
    HEATMAP_GENERATION = "HEATMAP_GENERATION"

@dataclass
class EyeTrackerInfo:
    device_id: str
    vendor: str
    model: str
    firmware_version: str
    protocol_version: str
    serial_number: Optional[str] = None
    device_type: Optional[DeviceType] = None
    vendor_url: Optional[str] = None
    product_url: Optional[str] = None
    metadata: Optional[Dict[str, Any]] = None

@dataclass
class SamplingRateSpec:
    supported: List[int]
    default: int
    current: Optional[int] = None

@dataclass
class AccuracySpec:
    typical: float
    best: Optional[float] = None
    measurement_method: Optional[str] = None

@dataclass
class PrecisionSpec:
    typical: float
    rms: Optional[float] = None

@dataclass
class LatencySpec:
    average: float
    maximum: Optional[float] = None

@dataclass
class RangeSpec:
    min: float
    max: float
    optimal: Optional[float] = None

@dataclass
class TrackingAreaSpec:
    horizontal: float
    vertical: float

@dataclass
class HeadMovementSpec:
    horizontal: float
    vertical: float
    depth: float

@dataclass
class TrackingCapabilities:
    binocular: bool
    head_tracking: bool
    gaze_3d: bool
    sampling_rate: SamplingRateSpec
    accuracy: AccuracySpec
    precision: PrecisionSpec
    latency: LatencySpec
    operating_distance: RangeSpec
    tracking_area: TrackingAreaSpec
    head_movement_tolerance: Optional[HeadMovementSpec] = None

@dataclass
class DataCapabilities:
    gaze_point: bool
    eye_data: bool
    pupil_diameter: bool
    pupil_position: bool
    eye_openness: bool
    eye_images: bool
    gaze_origin_3d: bool
    gaze_direction_3d: bool
    built_in_fixation_detection: bool
    built_in_saccade_detection: bool
    built_in_blink_detection: bool
    device_timestamp: bool
    system_timestamp: bool
    external_sync: bool

@dataclass
class CalibrationCapabilities:
    required: bool
    types: List[CalibrationType]
    point_options: List[int]
    default_points: int
    auto_calibration: bool
    profile_management: bool
    quality_assessment: bool
    adaptive_calibration: bool

@dataclass
class AccessibilityCapabilities:
    dwell_selection: bool
    blink_input: bool
    wink_input: bool
    switch_emulation: bool
    adaptive_dwell_time: bool
    adaptive_target_size: bool
    error_smoothing: bool
    tremor_compensation: bool
    aac_optimized_mode: bool
    long_session_optimization: bool
    fatigue_detection: bool
    break_reminder: bool

@dataclass
class ConnectivityCapabilities:
    connection_types: List[ConnectionType]
    api_protocols: List[ApiProtocol]
    multi_client: bool
    remote_connection: bool
    mobile_connection: bool

@dataclass
class EyeTrackerCapabilities:
    device: EyeTrackerInfo
    tracking: TrackingCapabilities
    data: DataCapabilities
    calibration: CalibrationCapabilities
    connectivity: ConnectivityCapabilities
    supported_features: List[FeatureFlag]
    accessibility: Optional[AccessibilityCapabilities] = None
```

---

<div align="center">

**WIA Eye Gaze Device Capability Specification v1.0.0-draft**

**홍익인간** - 널리 인간을 이롭게

</div>
