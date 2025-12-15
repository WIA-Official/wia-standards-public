# WIA AAC API Interface Standard
## Phase 2 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-12-13
**Authors**: WIA (World Industry Authentication Association) / SmileStory Inc.
**License**: MIT

---

## 목차 (Table of Contents)

1. [개요 (Overview)](#1-개요-overview)
2. [용어 정의 (Terminology)](#2-용어-정의-terminology)
3. [핵심 인터페이스 (Core Interfaces)](#3-핵심-인터페이스-core-interfaces)
4. [센서별 어댑터 (Sensor Adapters)](#4-센서별-어댑터-sensor-adapters)
5. [이벤트 시스템 (Event System)](#5-이벤트-시스템-event-system)
6. [설정 관리 (Configuration)](#6-설정-관리-configuration)
7. [오류 처리 (Error Handling)](#7-오류-처리-error-handling)
8. [사용 예제 (Usage Examples)](#8-사용-예제-usage-examples)
9. [참고문헌 (References)](#9-참고문헌-references)

---

## 1. 개요 (Overview)

### 1.1 목적 (Purpose)

WIA AAC API Interface Standard는 Phase 1에서 정의한 Signal Format을 소프트웨어에서 사용하기 위한 표준 API를 정의합니다.

**핵심 목표**:
- 모든 AAC 센서를 동일한 API로 제어
- 이벤트 기반 비동기 아키텍처
- TypeScript/Python 구현 제공
- 확장 가능한 어댑터 패턴

### 1.2 적용 범위 (Scope)

본 표준은 다음을 정의합니다:

| 항목 | 설명 |
|------|------|
| **Core API** | 메인 WiaAac 클래스 인터페이스 |
| **Event System** | 이벤트 발행/구독 메커니즘 |
| **Adapters** | 센서별 어댑터 인터페이스 |
| **Types** | Phase 1 Signal과 호환되는 타입 정의 |

### 1.3 Phase 1 호환성 (Phase 1 Compatibility)

Phase 2 API는 Phase 1 Signal Format과 완전히 호환됩니다:

```
Phase 1: Signal Format (JSON 구조)
    ↓
Phase 2: API Interface (프로그래밍 인터페이스)
    ↓
Phase 3: Communication Protocol (전송 프로토콜)
```

---

## 2. 용어 정의 (Terminology)

### 2.1 핵심 용어

| 용어 | 정의 |
|------|------|
| **WiaAac** | 메인 API 클래스 (진입점) |
| **Adapter** | 센서별 연결 및 데이터 처리 담당 |
| **Signal** | Phase 1에서 정의한 센서 메시지 |
| **Event** | API에서 발생하는 비동기 이벤트 |
| **Handler** | 이벤트를 처리하는 콜백 함수 |
| **Config** | 센서 연결 설정 |

### 2.2 이벤트 유형

| 이벤트 | 설명 | 데이터 |
|--------|------|--------|
| `signal` | 센서 신호 수신 | WiaAacSignal |
| `selection` | 선택 완료 | SelectionEvent |
| `gesture` | 제스처 인식 | GestureEvent |
| `error` | 오류 발생 | WiaAacError |
| `connected` | 연결 완료 | DeviceInfo |
| `disconnected` | 연결 해제 | DisconnectReason |

---

## 3. 핵심 인터페이스 (Core Interfaces)

### 3.1 WiaAac 클래스

메인 API 진입점입니다.

#### TypeScript
```typescript
class WiaAac {
  // 생성자
  constructor(options?: WiaAacOptions);

  // 연결 관리
  connect(config: SensorConfig): Promise<void>;
  disconnect(): Promise<void>;
  isConnected(): boolean;

  // 장치 탐색
  listDevices(): Promise<DeviceInfo[]>;
  getDeviceInfo(): DeviceInfo | null;

  // 이벤트 처리
  on<T extends EventType>(event: T, handler: EventHandler<T>): void;
  off<T extends EventType>(event: T, handler: EventHandler<T>): void;
  once<T extends EventType>(event: T, handler: EventHandler<T>): void;
  emit<T extends EventType>(event: T, data: EventData<T>): void;

  // 상태 조회
  getLastSignal(): WiaAacSignal | null;
  getConnectionState(): ConnectionState;

  // 설정
  configure(options: Partial<SensorOptions>): void;
  getConfig(): SensorConfig | null;
}
```

#### Python
```python
class WiaAac:
    def __init__(self, options: Optional[WiaAacOptions] = None):
        ...

    # 연결 관리
    async def connect(self, config: SensorConfig) -> None: ...
    async def disconnect(self) -> None: ...
    def is_connected(self) -> bool: ...

    # 장치 탐색
    async def list_devices(self) -> List[DeviceInfo]: ...
    def get_device_info(self) -> Optional[DeviceInfo]: ...

    # 이벤트 처리
    def on(self, event: EventType, handler: EventHandler) -> None: ...
    def off(self, event: EventType, handler: EventHandler) -> None: ...
    def once(self, event: EventType, handler: EventHandler) -> None: ...
    def emit(self, event: EventType, data: Any) -> None: ...

    # 상태 조회
    def get_last_signal(self) -> Optional[WiaAacSignal]: ...
    def get_connection_state(self) -> ConnectionState: ...

    # 설정
    def configure(self, options: SensorOptions) -> None: ...
    def get_config(self) -> Optional[SensorConfig]: ...
```

### 3.2 WiaAacOptions

```typescript
interface WiaAacOptions {
  // 자동 재연결
  autoReconnect?: boolean;        // default: true
  reconnectInterval?: number;     // default: 3000 (ms)
  maxReconnectAttempts?: number;  // default: 5

  // 신호 처리
  signalBufferSize?: number;      // default: 100
  validateSignals?: boolean;      // default: true

  // 로깅
  logLevel?: 'debug' | 'info' | 'warn' | 'error' | 'none';
}
```

### 3.3 ConnectionState

```typescript
enum ConnectionState {
  DISCONNECTED = 'disconnected',
  CONNECTING = 'connecting',
  CONNECTED = 'connected',
  RECONNECTING = 'reconnecting',
  ERROR = 'error'
}
```

---

## 4. 센서별 어댑터 (Sensor Adapters)

### 4.1 어댑터 인터페이스

모든 센서 어댑터가 구현해야 하는 기본 인터페이스입니다.

```typescript
interface ISensorAdapter {
  // 속성
  readonly type: SensorType;
  readonly deviceInfo: DeviceInfo | null;
  readonly isConnected: boolean;

  // 연결 관리
  connect(config: SensorConfig): Promise<void>;
  disconnect(): Promise<void>;

  // 신호 처리
  getLastSignal(): WiaAacSignal | null;
  onSignal(handler: SignalHandler): void;
  offSignal(handler: SignalHandler): void;

  // 설정
  configure(options: SensorOptions): void;
  getSupportedOptions(): SensorOptionDescriptor[];
}
```

### 4.2 센서별 어댑터 목록

| 어댑터 | 클래스명 | 센서 유형 |
|--------|----------|----------|
| Eye Tracker | `EyeTrackerAdapter` | `eye_tracker` |
| Switch | `SwitchAdapter` | `switch` |
| Muscle Sensor | `MuscleSensorAdapter` | `muscle_sensor` |
| Brain Interface | `BrainInterfaceAdapter` | `brain_interface` |
| Breath | `BreathAdapter` | `breath` |
| Head Movement | `HeadMovementAdapter` | `head_movement` |

### 4.3 어댑터 등록

```typescript
// 커스텀 어댑터 등록
WiaAac.registerAdapter('custom', CustomAdapter);

// 어댑터 조회
const adapter = WiaAac.getAdapter('eye_tracker');
```

### 4.4 Mock 어댑터

테스트 및 개발용 Mock 어댑터:

```typescript
import { MockAdapter } from 'wia-aac/testing';

const aac = new WiaAac();
aac.useAdapter(new MockAdapter({
  type: 'eye_tracker',
  simulateSignals: true,
  signalInterval: 100
}));
```

---

## 5. 이벤트 시스템 (Event System)

### 5.1 이벤트 구독

```typescript
// 기본 구독
aac.on('signal', (signal) => {
  console.log('Signal received:', signal);
});

// 일회성 구독
aac.once('connected', (info) => {
  console.log('Connected to:', info.model);
});

// 구독 해제
const handler = (signal) => { ... };
aac.on('signal', handler);
aac.off('signal', handler);
```

### 5.2 이벤트 데이터 타입

#### SignalEvent
```typescript
// 'signal' 이벤트 데이터는 WiaAacSignal (Phase 1)
type SignalEventData = WiaAacSignal;
```

#### SelectionEvent
```typescript
interface SelectionEvent {
  timestamp: number;
  targetId: string;
  targetType: 'key' | 'button' | 'area' | 'custom';
  selectionMethod: 'dwell' | 'click' | 'gesture' | 'switch';
  position?: { x: number; y: number };
  confidence: number;
}
```

#### GestureEvent
```typescript
interface GestureEvent {
  timestamp: number;
  gesture: string;
  sensorType: SensorType;
  confidence: number;
  metadata?: Record<string, unknown>;
}
```

#### ErrorEvent
```typescript
interface WiaAacError {
  code: ErrorCode;
  message: string;
  timestamp: number;
  recoverable: boolean;
  details?: Record<string, unknown>;
}
```

### 5.3 이벤트 필터링

```typescript
// 특정 조건의 신호만 수신
aac.on('signal', (signal) => {
  // 신뢰도 80% 이상만 처리
  if (signal.meta?.confidence >= 0.8) {
    processSignal(signal);
  }
});

// 또는 필터 옵션 사용
aac.on('signal', handler, {
  filter: (signal) => signal.meta?.confidence >= 0.8
});
```

---

## 6. 설정 관리 (Configuration)

### 6.1 SensorConfig

```typescript
interface SensorConfig {
  // 필수
  type: SensorType;

  // 장치 식별 (선택)
  device?: {
    manufacturer?: string;
    model?: string;
    serial?: string;
  };

  // 연결 옵션
  connection?: {
    protocol?: 'usb' | 'bluetooth' | 'wifi' | 'serial';
    port?: string;
    baudRate?: number;
  };

  // 센서 옵션
  options?: SensorOptions;
}
```

### 6.2 SensorOptions

#### 공통 옵션
```typescript
interface BaseSensorOptions {
  sampleRate?: number;       // Hz
  sensitivity?: number;      // 0.0 ~ 1.0
  dwellTime?: number;        // ms (선택 대기 시간)
  smoothing?: boolean;       // 신호 평활화
  smoothingFactor?: number;  // 0.0 ~ 1.0
}
```

#### Eye Tracker 옵션
```typescript
interface EyeTrackerOptions extends BaseSensorOptions {
  trackBothEyes?: boolean;
  trackPupil?: boolean;
  trackBlink?: boolean;
  fixationThreshold?: number;  // ms
  gazeFilter?: 'none' | 'average' | 'kalman';
}
```

#### Switch 옵션
```typescript
interface SwitchOptions extends BaseSensorOptions {
  debounceTime?: number;  // ms
  holdThreshold?: number; // ms (held 상태 전환)
  multiPressWindow?: number; // ms (연속 누름 인식)
}
```

#### Muscle Sensor 옵션
```typescript
interface MuscleSensorOptions extends BaseSensorOptions {
  activationThreshold?: number;  // 0.0 ~ 1.0
  gestureRecognition?: boolean;
  channels?: number[];  // 활성화할 채널
}
```

#### Brain Interface 옵션
```typescript
interface BrainInterfaceOptions extends BaseSensorOptions {
  channels?: string[];  // 10-20 시스템 채널명
  bandPassFilter?: { low: number; high: number };
  artifactRejection?: boolean;
  classificationModel?: string;
}
```

#### Breath 옵션
```typescript
interface BreathOptions extends BaseSensorOptions {
  sipThreshold?: number;   // kPa
  puffThreshold?: number;  // kPa
  hardThreshold?: number;  // 강한 동작 임계값
}
```

#### Head Movement 옵션
```typescript
interface HeadMovementOptions extends BaseSensorOptions {
  trackRotation?: boolean;
  gestureRecognition?: boolean;
  dwellRadius?: number;  // 정지 인식 반경 (정규화)
}
```

---

## 7. 오류 처리 (Error Handling)

### 7.1 에러 코드

```typescript
enum ErrorCode {
  // 연결 오류 (1xx)
  CONNECTION_FAILED = 100,
  CONNECTION_LOST = 101,
  CONNECTION_TIMEOUT = 102,
  DEVICE_NOT_FOUND = 103,
  DEVICE_BUSY = 104,
  PERMISSION_DENIED = 105,

  // 설정 오류 (2xx)
  INVALID_CONFIG = 200,
  UNSUPPORTED_OPTION = 201,
  INVALID_SENSOR_TYPE = 202,

  // 런타임 오류 (3xx)
  SIGNAL_VALIDATION_FAILED = 300,
  ADAPTER_ERROR = 301,
  INTERNAL_ERROR = 302,

  // 프로토콜 오류 (4xx)
  PROTOCOL_ERROR = 400,
  MESSAGE_PARSE_ERROR = 401,
  UNSUPPORTED_VERSION = 402
}
```

### 7.2 에러 핸들링

```typescript
// 이벤트 기반
aac.on('error', (error) => {
  console.error(`Error [${error.code}]: ${error.message}`);

  if (error.recoverable) {
    // 자동 복구 시도
  } else {
    // 사용자에게 알림
  }
});

// try-catch
try {
  await aac.connect(config);
} catch (error) {
  if (error instanceof WiaAacError) {
    handleWiaAacError(error);
  }
}
```

### 7.3 복구 전략

```typescript
interface RecoveryStrategy {
  // 자동 재연결
  autoReconnect: boolean;
  reconnectDelay: number;
  maxAttempts: number;

  // 콜백
  onReconnecting?: (attempt: number) => void;
  onReconnected?: () => void;
  onReconnectFailed?: () => void;
}
```

---

## 8. 사용 예제 (Usage Examples)

### 8.1 기본 사용법

#### TypeScript
```typescript
import { WiaAac, SensorType } from 'wia-aac';

async function main() {
  const aac = new WiaAac({
    autoReconnect: true,
    logLevel: 'info'
  });

  // 이벤트 핸들러 등록
  aac.on('signal', (signal) => {
    console.log(`[${signal.type}] Signal received`);
  });

  aac.on('error', (error) => {
    console.error(`Error: ${error.message}`);
  });

  // 센서 연결
  await aac.connect({
    type: SensorType.EYE_TRACKER,
    device: { manufacturer: 'Tobii' }
  });

  console.log('Connected:', aac.isConnected());

  // 10초 후 연결 해제
  setTimeout(async () => {
    await aac.disconnect();
  }, 10000);
}

main().catch(console.error);
```

#### Python
```python
import asyncio
from wia_aac import WiaAac, SensorType

async def main():
    aac = WiaAac(
        auto_reconnect=True,
        log_level='info'
    )

    # 이벤트 핸들러 등록
    @aac.on('signal')
    def on_signal(signal):
        print(f'[{signal.type}] Signal received')

    @aac.on('error')
    def on_error(error):
        print(f'Error: {error.message}')

    # 센서 연결
    await aac.connect(
        type=SensorType.EYE_TRACKER,
        device={'manufacturer': 'Tobii'}
    )

    print('Connected:', aac.is_connected())

    # 10초 대기
    await asyncio.sleep(10)

    # 연결 해제
    await aac.disconnect()

asyncio.run(main())
```

### 8.2 Eye Tracker 예제

```typescript
import { WiaAac, SensorType } from 'wia-aac';

const aac = new WiaAac();

aac.on('signal', (signal) => {
  if (signal.type !== 'eye_tracker') return;

  const { gaze, fixation } = signal.data;

  // 시선 위치 출력
  console.log(`Gaze: (${gaze.x.toFixed(2)}, ${gaze.y.toFixed(2)})`);

  // 응시 감지
  if (fixation?.active) {
    console.log(`Fixation on: ${fixation.target_id} for ${fixation.duration_ms}ms`);
  }
});

aac.on('selection', (event) => {
  console.log(`Selected: ${event.targetId} via ${event.selectionMethod}`);
});

await aac.connect({
  type: SensorType.EYE_TRACKER,
  options: {
    dwellTime: 1000,      // 1초 응시로 선택
    gazeFilter: 'kalman'  // 칼만 필터 적용
  }
});
```

### 8.3 멀티 센서 예제

```typescript
import { WiaAac, SensorType } from 'wia-aac';

// 여러 센서 동시 사용
const eyeTracker = new WiaAac();
const switchInput = new WiaAac();

// Eye tracker로 위치 이동
eyeTracker.on('signal', (signal) => {
  moveCursor(signal.data.gaze.x, signal.data.gaze.y);
});

// Switch로 클릭
switchInput.on('signal', (signal) => {
  if (signal.data.state === 'pressed') {
    performClick();
  }
});

await Promise.all([
  eyeTracker.connect({ type: SensorType.EYE_TRACKER }),
  switchInput.connect({ type: SensorType.SWITCH })
]);
```

### 8.4 Brain Interface 예제

```typescript
import { WiaAac, SensorType } from 'wia-aac';

const aac = new WiaAac();

aac.on('signal', (signal) => {
  if (signal.type !== 'brain_interface') return;

  const { bands, classification } = signal.data;

  // 알파파 수준 모니터링
  if (bands?.alpha > 0.4) {
    console.log('High alpha - User is relaxed');
  }

  // BCI 분류 결과
  if (classification?.intent === 'select' && classification.confidence > 0.8) {
    performSelection();
  }
});

await aac.connect({
  type: SensorType.BRAIN_INTERFACE,
  device: { manufacturer: 'OpenBCI' },
  options: {
    channels: ['Fp1', 'Fp2', 'C3', 'C4'],
    artifactRejection: true
  }
});
```

---

## 9. 참고문헌 (References)

### 관련 표준

- [WIA AAC Signal Format Standard (Phase 1)](/spec/PHASE-1-SIGNAL-FORMAT.md)
- [WIA AAC Signal JSON Schema](/spec/schemas/)

### 참고 프로젝트

- [Intel ACAT](https://github.com/intel/acat)
- [Tobii Pro SDK](https://developer.tobiipro.com/)
- [BrainFlow](https://brainflow.readthedocs.io/)

### 설계 패턴

- Event Emitter Pattern
- Adapter Pattern
- Factory Pattern

---

## 부록 A: 타입 정의 요약

### SensorType
```typescript
type SensorType =
  | 'eye_tracker'
  | 'switch'
  | 'muscle_sensor'
  | 'brain_interface'
  | 'breath'
  | 'head_movement'
  | 'custom';
```

### EventType
```typescript
type EventType =
  | 'signal'
  | 'selection'
  | 'gesture'
  | 'error'
  | 'connected'
  | 'disconnected';
```

### WiaAacSignal
Phase 1 Signal Format과 동일 (참조: `/spec/PHASE-1-SIGNAL-FORMAT.md`)

---

## 부록 B: API 구현 파일 구조

### TypeScript
```
/api/typescript/
├── src/
│   ├── index.ts
│   ├── types/
│   ├── core/
│   ├── adapters/
│   └── utils/
└── tests/
```

### Python
```
/api/python/
├── wia_aac/
│   ├── types/
│   ├── core/
│   ├── adapters/
│   └── utils/
└── tests/
```

---

<div align="center">

**WIA AAC API Interface Standard v1.0.0**

**弘益人間 (홍익인간)** - 널리 인간을 이롭게

🤟

---

**© 2025 SmileStory Inc. / WIA**

**MIT License**

</div>
