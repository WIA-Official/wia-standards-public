# WIA Eye Gaze Standard - Phase 1: Interoperability Protocol Definition

## 철학
**홍익인간 (弘益人間)** - 널리 인간을 이롭게 하라

## 배경

시선 추적(Eye Gaze) 기술은 ALS, 뇌성마비, 척수손상 등으로 손을 사용할 수 없는 사용자들이
컴퓨터와 의사소통하는 핵심 보조기기입니다.

### 현재 문제점
- 제조사마다 **독자 인터페이스** 사용 (Tobii, EyeTech, LC Technologies 등)
- 시선 추적 앱들 간 **상호운용성 0** - 서로 인식/협력 불가
- [ATIA Eye Gaze Standards Working Group](https://www.atia.org/eyegazestandards/)이 2024년 1월에야 첫 회의
- 시장 성장 억제 요인으로 지목됨

### WIA의 목표
**WIA Eye Gaze Protocol** - 모든 시선 추적 디바이스와 앱이 서로 통신할 수 있는 오픈 표준

---

## Phase 1 목표: 데이터 포맷 및 이벤트 표준

### 1.1 시선 데이터 포맷 정의

```typescript
interface GazePoint {
  timestamp: number;           // Unix timestamp (ms)
  x: number;                   // Screen X coordinate (0.0 - 1.0 normalized)
  y: number;                   // Screen Y coordinate (0.0 - 1.0 normalized)
  leftEye?: EyeData;           // Left eye specific data
  rightEye?: EyeData;          // Right eye specific data
  confidence: number;          // 0.0 - 1.0
  fixation: boolean;           // Is this point part of a fixation?
  saccade: boolean;            // Is this point part of a saccade?
}

interface EyeData {
  pupilDiameter: number;       // mm
  gazeOrigin: Vector3D;        // 3D gaze origin point
  gazeDirection: Vector3D;     // 3D gaze direction vector
  eyeOpenness: number;         // 0.0 (closed) - 1.0 (fully open)
}

interface Vector3D {
  x: number;
  y: number;
  z: number;
}
```

### 1.2 시선 이벤트 표준

```typescript
enum GazeEventType {
  FIXATION_START = 'fixation_start',
  FIXATION_END = 'fixation_end',
  SACCADE_START = 'saccade_start',
  SACCADE_END = 'saccade_end',
  BLINK = 'blink',
  DWELL_COMPLETE = 'dwell_complete',      // Dwell selection 완료
  SMOOTH_PURSUIT = 'smooth_pursuit',       // 부드러운 추적 시작
}

interface GazeEvent {
  type: GazeEventType;
  timestamp: number;
  duration?: number;           // ms (for fixation, blink)
  target?: GazeTarget;         // 이벤트 대상 UI 요소
  metadata?: Record<string, unknown>;
}

interface GazeTarget {
  elementId: string;
  boundingBox: BoundingBox;
  semanticType: string;        // 'button', 'link', 'text', 'custom'
}
```

### 1.3 디바이스 Capability 표준

```typescript
interface EyeTrackerCapabilities {
  vendor: string;
  model: string;
  trackingFrequency: number;   // Hz (30, 60, 120, 250, etc.)
  trackingAccuracy: number;    // degrees
  trackingPrecision: number;   // degrees
  binocular: boolean;          // Both eyes or single
  headTracking: boolean;       // Head position tracking
  pupilTracking: boolean;      // Pupil diameter measurement
  distanceRange: {             // Operating distance
    min: number;               // cm
    max: number;               // cm
  };
  screenSize?: {               // Calibrated screen size
    width: number;             // pixels
    height: number;            // pixels
  };
}
```

---

## 연구 과제

Phase 1에서 조사해야 할 항목:

1. **기존 프로토콜 분석**
   - Tobii Pro SDK 데이터 포맷
   - EyeTech TM5 데이터 포맷
   - Gazepoint GP3 데이터 포맷
   - PupilLabs 오픈소스 포맷
   - OpenGazer 포맷

2. **학술 표준 조사**
   - IEEE 시선 추적 관련 표준
   - ISO 9241-971 (Eye tracking)
   - W3C Eye Tracking Working Group 동향

3. **Accessibility 표준 연계**
   - WCAG 시선 추적 관련 지침
   - Section 508 요구사항
   - EN 301 549 (European accessibility standard)

---

## 산출물

Phase 1 완료 시 생성되어야 할 파일:

```
eye-gaze/
├── spec/
│   ├── RESEARCH-PHASE-1.md       # 연구 결과
│   ├── DATA-FORMAT-SPEC.md       # 데이터 포맷 명세
│   ├── EVENT-SPEC.md             # 이벤트 명세
│   └── DEVICE-CAPABILITY-SPEC.md # 디바이스 역량 명세
├── schemas/
│   ├── gaze-point.schema.json
│   ├── gaze-event.schema.json
│   └── device-capability.schema.json
└── docs/
    └── PHASE-1-SUMMARY.md
```

---

## 다음 단계

Phase 1 완료 후:
- `prompts/PHASE-2-PROMPT.md` 읽고 Phase 2 (API Interface) 시작
