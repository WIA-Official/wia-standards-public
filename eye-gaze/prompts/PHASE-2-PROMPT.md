# WIA Eye Gaze Standard - Phase 2: API Interface & SDK

## 전제조건
- Phase 1 (데이터 포맷 표준) 완료
- `spec/DATA-FORMAT-SPEC.md` 참조

## Phase 2 목표: 개발자 SDK 구현

### 2.1 핵심 API 설계

```typescript
// Core Eye Tracker Interface
interface IWiaEyeTracker {
  // Connection
  connect(): Promise<void>;
  disconnect(): Promise<void>;
  isConnected(): boolean;

  // Calibration
  startCalibration(points: CalibrationPoint[]): Promise<CalibrationResult>;
  getCalibrationQuality(): CalibrationQuality;

  // Data Streaming
  subscribe(callback: (data: GazePoint) => void): Subscription;
  unsubscribe(subscription: Subscription): void;

  // Events
  on(event: GazeEventType, handler: (e: GazeEvent) => void): void;
  off(event: GazeEventType, handler: (e: GazeEvent) => void): void;

  // Device Info
  getCapabilities(): EyeTrackerCapabilities;
  getStatus(): TrackerStatus;
}

// Gaze-Aware Application Interface
interface IGazeAwareApp {
  // Registration
  register(appId: string, capabilities: AppCapabilities): Promise<void>;
  unregister(): Promise<void>;

  // Inter-app Communication
  announceGazeControl(): void;          // "I'm taking gaze control"
  releaseGazeControl(): void;           // "I'm releasing gaze control"
  onGazeControlRequest(handler: (appId: string) => boolean): void;

  // Target Registration
  registerTarget(target: GazeTarget): void;
  unregisterTarget(targetId: string): void;
  getActiveTargets(): GazeTarget[];
}
```

### 2.2 Dwell Selection API

```typescript
interface DwellController {
  // Configuration
  setDwellTime(ms: number): void;         // Default: 800ms
  setDwellFeedback(type: DwellFeedbackType): void;
  setDwellCancelZone(pixels: number): void;

  // Visual Feedback
  showDwellProgress(target: GazeTarget, progress: number): void;
  hideDwellProgress(): void;

  // Events
  onDwellStart(handler: (target: GazeTarget) => void): void;
  onDwellProgress(handler: (target: GazeTarget, progress: number) => void): void;
  onDwellComplete(handler: (target: GazeTarget) => void): void;
  onDwellCancel(handler: (target: GazeTarget) => void): void;
}

enum DwellFeedbackType {
  CIRCULAR_FILL = 'circular_fill',
  LINEAR_BAR = 'linear_bar',
  SHRINKING_RING = 'shrinking_ring',
  AUDIO_ONLY = 'audio_only',
  CUSTOM = 'custom'
}
```

### 2.3 앱 간 통신 프로토콜

시선 추적 앱들이 서로 인식하고 협력하기 위한 프로토콜:

```typescript
// Message Protocol
interface GazeAppMessage {
  type: GazeAppMessageType;
  appId: string;
  timestamp: number;
  payload: unknown;
}

enum GazeAppMessageType {
  // Discovery
  APP_ANNOUNCE = 'app_announce',         // 앱 존재 알림
  APP_QUERY = 'app_query',               // 앱 검색
  APP_RESPONSE = 'app_response',         // 검색 응답

  // Control
  CONTROL_REQUEST = 'control_request',   // 시선 제어권 요청
  CONTROL_GRANT = 'control_grant',       // 제어권 허용
  CONTROL_DENY = 'control_deny',         // 제어권 거부
  CONTROL_RELEASE = 'control_release',   // 제어권 반환

  // Coordination
  PAUSE_TRACKING = 'pause_tracking',     // 추적 일시정지 요청
  RESUME_TRACKING = 'resume_tracking',   // 추적 재개 요청
  TARGET_SYNC = 'target_sync',           // 타겟 동기화
}

// Transport Layer Options
interface TransportConfig {
  type: 'websocket' | 'ipc' | 'broadcast_channel';
  endpoint?: string;
}
```

---

## 구현 요구사항

### TypeScript SDK

```
eye-gaze/
├── api/
│   └── typescript/
│       ├── src/
│       │   ├── index.ts
│       │   ├── tracker/
│       │   │   ├── WiaEyeTracker.ts
│       │   │   ├── adapters/
│       │   │   │   ├── TobiiAdapter.ts
│       │   │   │   ├── GazepointAdapter.ts
│       │   │   │   └── PupilLabsAdapter.ts
│       │   │   └── MockTracker.ts
│       │   ├── app/
│       │   │   ├── GazeAwareApp.ts
│       │   │   └── AppCommunicator.ts
│       │   ├── dwell/
│       │   │   └── DwellController.ts
│       │   └── types/
│       │       └── index.ts
│       ├── package.json
│       └── tsconfig.json
```

### Python SDK

```
eye-gaze/
├── api/
│   └── python/
│       ├── wia_eye_gaze/
│       │   ├── __init__.py
│       │   ├── tracker.py
│       │   ├── adapters/
│       │   │   ├── tobii.py
│       │   │   ├── gazepoint.py
│       │   │   └── pupil_labs.py
│       │   ├── app.py
│       │   ├── dwell.py
│       │   └── types.py
│       ├── setup.py
│       └── requirements.txt
```

### Rust SDK (고성능 real-time용)

```
eye-gaze/
├── api/
│   └── rust/
│       ├── src/
│       │   ├── lib.rs
│       │   ├── tracker.rs
│       │   ├── app.rs
│       │   ├── dwell.rs
│       │   └── protocol.rs
│       └── Cargo.toml
```

---

## 산출물

Phase 2 완료 시:
- TypeScript SDK (npm 배포 가능)
- Python SDK (PyPI 배포 가능)
- Rust SDK (crates.io 배포 가능)
- API Reference 문서

---

## 다음 단계

Phase 2 완료 후:
- `prompts/PHASE-3-PROMPT.md` 읽고 Phase 3 (실시간 통신 프로토콜) 시작
