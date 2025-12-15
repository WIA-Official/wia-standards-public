# WIA Eye Gaze Standard - Phase 2 Summary

**Phase 2: API Interface & SDK**
**Status**: Complete
**Date**: 2025-01

---

## 1. Phase 2 개요 (Overview)

Phase 2에서는 WIA Eye Gaze Interoperability Protocol의 SDK를 TypeScript, Python, Rust 세 가지 언어로 구현했습니다.

### 1.1 목표

- **통합 API 인터페이스** - 모든 시선 추적 디바이스에 대한 일관된 인터페이스
- **디바이스 어댑터** - Tobii, Gazepoint, Pupil Labs 지원
- **Dwell 선택 컨트롤러** - AAC 접근성을 위한 시선 기반 선택
- **앱 간 통신** - 시선 인식 앱들의 상호운용성

### 1.2 철학

**弘益人間 (홍익인간)** - 널리 인간을 이롭게

---

## 2. SDK 구현 현황

### 2.1 TypeScript SDK (`@anthropics/wia-eye-gaze`)

| 컴포넌트 | 파일 | 설명 |
|---------|-----|------|
| **Types** | `src/types/index.ts` | 모든 타입 정의 |
| **IEyeTracker** | `src/tracker/IEyeTracker.ts` | 트래커 인터페이스 |
| **WiaEyeTracker** | `src/tracker/WiaEyeTracker.ts` | 메인 구현 |
| **MockAdapter** | `src/tracker/adapters/MockAdapter.ts` | 테스트용 어댑터 |
| **TobiiAdapter** | `src/tracker/adapters/TobiiAdapter.ts` | Tobii Pro 지원 |
| **GazepointAdapter** | `src/tracker/adapters/GazepointAdapter.ts` | Gazepoint 지원 |
| **PupilLabsAdapter** | `src/tracker/adapters/PupilLabsAdapter.ts` | Pupil Labs 지원 |
| **DwellController** | `src/dwell/DwellController.ts` | 응시 선택 컨트롤러 |
| **GazeAwareApp** | `src/app/GazeAwareApp.ts` | 앱 간 통신 |

### 2.2 Python SDK (`wia-eye-gaze`)

| 컴포넌트 | 파일 | 설명 |
|---------|-----|------|
| **Types** | `wia_eye_gaze/types.py` | 타입 및 데이터클래스 |
| **Tracker** | `wia_eye_gaze/tracker.py` | 트래커 구현 |
| **MockAdapter** | `wia_eye_gaze/adapters/mock.py` | 테스트용 어댑터 |
| **DwellController** | `wia_eye_gaze/dwell.py` | 응시 선택 컨트롤러 |
| **GazeAwareApp** | `wia_eye_gaze/app.py` | 앱 간 통신 |

### 2.3 Rust SDK (`wia-eye-gaze`)

| 컴포넌트 | 파일 | 설명 |
|---------|-----|------|
| **Types** | `src/types.rs` | 타입 정의 |
| **Tracker** | `src/tracker.rs` | 트래커 구현 |
| **Adapters** | `src/adapters.rs` | MockAdapter |
| **Dwell** | `src/dwell.rs` | 응시 선택 컨트롤러 |
| **App** | `src/app.rs` | 앱 간 통신 |

---

## 3. 핵심 API

### 3.1 EyeTracker Interface

```typescript
interface IEyeTracker {
  // Connection
  connect(): Promise<void>;
  disconnect(): Promise<void>;
  isConnected(): boolean;

  // Calibration
  startCalibration(points?: CalibrationPoint[]): Promise<CalibrationResult>;
  getCalibrationQuality(): CalibrationQuality | null;

  // Data Streaming
  subscribe(callback: GazeCallback): Subscription;
  unsubscribe(subscription: Subscription): void;
  startTracking(): void;
  stopTracking(): void;

  // Events
  on(event: GazeEventType, handler: EventCallback): void;
  off(event: GazeEventType, handler: EventCallback): void;

  // Device Info
  getCapabilities(): EyeTrackerCapabilities;
  getStatus(): TrackerStatus;
}
```

### 3.2 DwellController API

```typescript
interface DwellController {
  // Configuration
  setDwellTime(ms: number): void;
  setCooldownPeriod(ms: number): void;

  // Target Management
  registerTarget(target: GazeTarget): void;
  unregisterTarget(targetId: string): void;

  // Event Handlers
  onDwellStart(handler: DwellEventHandler): void;
  onDwellProgress(handler: DwellEventHandler): void;
  onDwellComplete(handler: DwellEventHandler): void;
  onDwellCancel(handler: DwellEventHandler): void;

  // Control
  start(): void;
  stop(): void;
}
```

### 3.3 GazeAwareApp API

```typescript
interface GazeAwareApp {
  // Registration
  register(): Promise<void>;
  unregister(): Promise<void>;

  // Gaze Control
  announceGazeControl(): void;
  releaseGazeControl(): void;
  onGazeControlRequest(handler: ControlRequestHandler): void;

  // Target Management
  registerTarget(target: GazeTarget): void;
  unregisterTarget(targetId: string): void;
}
```

---

## 4. 사용 예시

### 4.1 기본 사용법 (TypeScript)

```typescript
import {
  createEyeTracker,
  createMockAdapter,
  createDwellController,
} from '@anthropics/wia-eye-gaze';

// 트래커 생성
const adapter = createMockAdapter({ samplingRate: 60 });
const tracker = createEyeTracker(adapter);

// 연결 및 캘리브레이션
await tracker.connect();
await tracker.startCalibration();

// 시선 데이터 구독
tracker.subscribe(gaze => {
  console.log(`Gaze: (${gaze.x.toFixed(2)}, ${gaze.y.toFixed(2)})`);
});

tracker.startTracking();

// Dwell 선택
const dwell = createDwellController({ tracker, threshold: 800 });

dwell.registerTarget({
  elementId: 'btn-ok',
  boundingBox: { x: 0.4, y: 0.5, width: 0.2, height: 0.1 },
  semanticType: 'button',
  label: 'OK',
});

dwell.onDwellComplete(target => {
  console.log(`Selected: ${target.label}`);
});

dwell.start();
```

### 4.2 Python 사용법

```python
from wia_eye_gaze import (
    create_eye_tracker,
    MockAdapter,
    create_dwell_controller,
    GazeTarget,
    BoundingBox,
    TargetSemanticType,
)

# 트래커 생성
adapter = MockAdapter(sampling_rate=60)
tracker = create_eye_tracker(adapter)

# 연결
await tracker.connect()
await tracker.start_calibration()

# 시선 데이터 구독
tracker.subscribe(lambda gaze: print(f"Gaze: ({gaze.x:.2f}, {gaze.y:.2f})"))
tracker.start_tracking()

# Dwell 선택
dwell = create_dwell_controller(tracker=tracker, threshold=800)
dwell.register_target(GazeTarget(
    element_id="btn-ok",
    bounding_box=BoundingBox(x=0.4, y=0.5, width=0.2, height=0.1),
    semantic_type=TargetSemanticType.BUTTON,
    label="OK",
))
dwell.on_dwell_complete(lambda t, _: print(f"Selected: {t.label}"))
dwell.start()
```

### 4.3 Rust 사용법

```rust
use wia_eye_gaze::{
    EyeTracker, MockAdapter, DwellController, DwellConfig,
    GazeTarget, BoundingBox, TargetSemanticType,
};

// 트래커 생성
let adapter = MockAdapter::new(60);
let mut tracker = EyeTracker::new(adapter);

// 연결
tracker.connect().await?;
tracker.start_calibration(None).await?;

// 시선 데이터 구독
tracker.subscribe(|gaze| {
    println!("Gaze: ({:.2}, {:.2})", gaze.x, gaze.y);
});

tracker.start_tracking();

// Dwell 선택
let dwell = DwellController::new(DwellConfig::default());
dwell.register_target(GazeTarget::new(
    "btn-ok",
    BoundingBox::new(0.4, 0.5, 0.2, 0.1),
    TargetSemanticType::Button,
).with_label("OK"));

dwell.on_dwell_complete(|target, _| {
    println!("Selected: {:?}", target.label);
});

dwell.start();
```

---

## 5. 디렉토리 구조

```
eye-gaze/api/
├── typescript/
│   ├── src/
│   │   ├── types/index.ts
│   │   ├── tracker/
│   │   │   ├── IEyeTracker.ts
│   │   │   ├── WiaEyeTracker.ts
│   │   │   └── adapters/
│   │   │       ├── MockAdapter.ts
│   │   │       ├── TobiiAdapter.ts
│   │   │       ├── GazepointAdapter.ts
│   │   │       └── PupilLabsAdapter.ts
│   │   ├── dwell/DwellController.ts
│   │   ├── app/GazeAwareApp.ts
│   │   └── index.ts
│   ├── package.json
│   └── tsconfig.json
│
├── python/
│   ├── wia_eye_gaze/
│   │   ├── __init__.py
│   │   ├── types.py
│   │   ├── tracker.py
│   │   ├── adapters/mock.py
│   │   ├── dwell.py
│   │   └── app.py
│   ├── setup.py
│   └── requirements.txt
│
└── rust/
    ├── src/
    │   ├── lib.rs
    │   ├── types.rs
    │   ├── tracker.rs
    │   ├── adapters.rs
    │   ├── dwell.rs
    │   └── app.rs
    └── Cargo.toml
```

---

## 6. 다음 단계

### Phase 3: Communication Protocol

```
목표: 실시간 통신 프로토콜 표준화

작업:
├── WebSocket 메시지 포맷
├── REST API 엔드포인트
├── 실시간 스트리밍 프로토콜
└── 앱 간 동기화 메커니즘
```

### Phase 4: Integration

```
목표: WIA 생태계 통합

작업:
├── WIA AAC Standard 연동
├── 데모 애플리케이션
├── 문서 및 튜토리얼
└── 패키지 배포 (npm, PyPI, crates.io)
```

---

## 7. 결론

Phase 2에서 WIA Eye Gaze SDK를 세 가지 주요 언어로 구현 완료했습니다.

### 주요 성과

1. **TypeScript SDK** - 웹/Node.js 환경을 위한 완전한 SDK
2. **Python SDK** - 연구/데이터 분석용 SDK
3. **Rust SDK** - 고성능 실시간 처리용 SDK
4. **통합 API** - 모든 언어에서 동일한 패턴 사용
5. **디바이스 어댑터** - Tobii, Gazepoint, Pupil Labs 지원

### 기대 효과

- 개발자의 **시선 추적 앱 개발 진입 장벽** 감소
- **AAC 앱 개발** 가속화
- **상호운용성** 향상
- 시선 추적 **에코시스템** 확장

---

<div align="center">

**WIA Eye Gaze Standard - Phase 2 Complete**

**弘益人間** - 널리 인간을 이롭게

🤟

**Next: Phase 3 - Communication Protocol**

</div>
