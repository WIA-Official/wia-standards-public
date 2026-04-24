# Phase 2: API Interface Standard
## Claude Code 작업 프롬프트

---

**Phase**: 2 of 4
**목표**: 소프트웨어 개발자를 위한 표준 API 인터페이스 정의
**난이도**: ★★★★☆
**예상 작업량**: 스펙 문서 1개 + TypeScript API + Python API + 예제

---

## 🎯 Phase 2 목표

### 핵심 질문
```
"Phase 1에서 정의한 Signal Format을
 소프트웨어에서 어떻게 사용할 것인가?

 센서 연결, 이벤트 수신, 데이터 처리를
 표준화된 API로 제공할 수 있을까?"
```

### 목표
```
모든 AAC 센서를 동일한 API로 제어할 수 있도록
표준 인터페이스를 정의하고 TypeScript/Python으로 구현한다.
```

---

## 📋 Phase 1 결과물 활용

Phase 2는 Phase 1의 Signal Format을 기반으로 합니다:

| Phase 1 산출물 | Phase 2 활용 |
|---------------|-------------|
| Signal Format Spec | API 메시지 타입 정의 |
| JSON Schema | 런타임 검증 |
| 센서별 data 구조 | 센서별 이벤트 타입 |

---

## 🏗️ API 설계 요구사항

### 1. 핵심 인터페이스

#### 1.1 연결 관리 (Connection Management)
```typescript
// 센서 연결
connect(config: SensorConfig): Promise<void>

// 센서 연결 해제
disconnect(): Promise<void>

// 연결 상태 확인
isConnected(): boolean

// 사용 가능한 센서 목록
listDevices(): Promise<DeviceInfo[]>
```

#### 1.2 이벤트 처리 (Event Handling)
```typescript
// 이벤트 구독
on(event: EventType, handler: EventHandler): void

// 이벤트 구독 해제
off(event: EventType, handler: EventHandler): void

// 일회성 이벤트 구독
once(event: EventType, handler: EventHandler): void
```

#### 1.3 이벤트 유형 (Event Types)
```typescript
type EventType =
  | 'signal'      // 센서 신호 수신
  | 'selection'   // 선택 완료 (dwell, click 등)
  | 'gesture'     // 제스처 인식
  | 'error'       // 오류 발생
  | 'connected'   // 연결됨
  | 'disconnected' // 연결 해제됨
```

### 2. 센서별 어댑터 (Adapters)

각 센서 유형에 맞는 어댑터 인터페이스:

```typescript
interface ISensorAdapter {
  type: SensorType;
  connect(): Promise<void>;
  disconnect(): Promise<void>;
  getSignal(): WiaAacSignal | null;
  onSignal(handler: SignalHandler): void;
}
```

### 3. 설정 관리 (Configuration)

```typescript
interface SensorConfig {
  type: SensorType;
  device?: {
    manufacturer?: string;
    model?: string;
    serial?: string;
  };
  options?: {
    sampleRate?: number;
    sensitivity?: number;
    dwellTime?: number;
    // 센서별 옵션
  };
}
```

---

## 📁 산출물 목록

Phase 2 완료 시 다음 파일을 생성해야 합니다:

### 1. 표준 스펙 문서
```
/spec/PHASE-2-API-INTERFACE.md

내용:
1. 개요 (Overview)
2. 용어 정의 (Terminology)
3. 핵심 인터페이스 (Core Interfaces)
4. 센서별 어댑터 (Sensor Adapters)
5. 이벤트 시스템 (Event System)
6. 설정 관리 (Configuration)
7. 오류 처리 (Error Handling)
8. 사용 예제 (Usage Examples)
9. 참고문헌 (References)
```

### 2. TypeScript API
```
/api/typescript/
├── package.json
├── tsconfig.json
├── src/
│   ├── index.ts              # 메인 진입점
│   ├── types/
│   │   ├── signal.ts         # Phase 1 Signal 타입
│   │   ├── events.ts         # 이벤트 타입
│   │   └── config.ts         # 설정 타입
│   ├── core/
│   │   ├── WiaAac.ts         # 메인 클래스
│   │   ├── EventEmitter.ts   # 이벤트 처리
│   │   └── SignalValidator.ts # 스키마 검증
│   ├── adapters/
│   │   ├── BaseAdapter.ts    # 기본 어댑터
│   │   ├── EyeTrackerAdapter.ts
│   │   ├── SwitchAdapter.ts
│   │   ├── MuscleSensorAdapter.ts
│   │   ├── BrainInterfaceAdapter.ts
│   │   ├── BreathAdapter.ts
│   │   └── HeadMovementAdapter.ts
│   └── utils/
│       └── helpers.ts
├── tests/
│   ├── WiaAac.test.ts
│   └── adapters.test.ts
└── README.md
```

### 3. Python API
```
/api/python/
├── pyproject.toml
├── wia_aac/
│   ├── __init__.py
│   ├── types/
│   │   ├── __init__.py
│   │   ├── signal.py
│   │   ├── events.py
│   │   └── config.py
│   ├── core/
│   │   ├── __init__.py
│   │   ├── wia_aac.py
│   │   ├── event_emitter.py
│   │   └── signal_validator.py
│   ├── adapters/
│   │   ├── __init__.py
│   │   ├── base_adapter.py
│   │   ├── eye_tracker_adapter.py
│   │   ├── switch_adapter.py
│   │   ├── muscle_sensor_adapter.py
│   │   ├── brain_interface_adapter.py
│   │   ├── breath_adapter.py
│   │   └── head_movement_adapter.py
│   └── utils/
│       └── helpers.py
├── tests/
│   ├── test_wia_aac.py
│   └── test_adapters.py
└── README.md
```

### 4. 사용 예제
```
/examples/api-usage/
├── typescript/
│   ├── basic-usage.ts
│   ├── eye-tracker-example.ts
│   └── multi-sensor-example.ts
└── python/
    ├── basic_usage.py
    ├── eye_tracker_example.py
    └── multi_sensor_example.py
```

---

## ✅ 완료 체크리스트

Phase 2 완료 전 확인:

```
□ /spec/PHASE-2-API-INTERFACE.md 작성 완료
□ TypeScript API 구현 완료
□ Python API 구현 완료
□ 타입 정의 완료 (Phase 1 Signal 호환)
□ 이벤트 시스템 구현 완료
□ 센서별 어댑터 구현 완료 (6개)
□ 단위 테스트 작성 완료
□ 테스트 통과
□ 사용 예제 작성 완료
□ README 업데이트 (Phase 2 완료 표시)
```

---

## 🔄 작업 순서

```
1. Phase 1 결과물 확인 (Signal Format, JSON Schema)
   ↓
2. API 인터페이스 설계
   ↓
3. /spec/PHASE-2-API-INTERFACE.md 작성
   ↓
4. TypeScript 타입 정의
   ↓
5. TypeScript 핵심 클래스 구현
   ↓
6. TypeScript 어댑터 구현
   ↓
7. Python 타입 정의
   ↓
8. Python 핵심 클래스 구현
   ↓
9. Python 어댑터 구현
   ↓
10. 단위 테스트 작성 및 실행
   ↓
11. 사용 예제 작성
   ↓
12. 완료 체크리스트 확인
   ↓
13. Phase 3 시작 가능
```

---

## 💡 설계 가이드라인

### DO (해야 할 것)

```
✅ Phase 1 Signal Format과 완전히 호환
✅ 동기/비동기 API 모두 지원
✅ 이벤트 기반 아키텍처 사용
✅ 명확한 타입 정의
✅ 에러 처리 포함
✅ 확장 가능한 어댑터 패턴
```

### DON'T (하지 말 것)

```
❌ 실제 하드웨어 드라이버 구현 (어댑터 인터페이스만)
❌ 특정 센서 제조사에 종속되는 코드
❌ Phase 1 스키마와 불일치
❌ 동기화 없는 상태 관리
```

---

## 📝 API 사용 예시 (목표)

### TypeScript
```typescript
import { WiaAac } from 'wia-aac';

const aac = new WiaAac();

// 센서 연결
await aac.connect({
  type: 'eye_tracker',
  device: { manufacturer: 'Tobii' }
});

// 신호 이벤트 수신
aac.on('signal', (signal) => {
  console.log('Gaze:', signal.data.gaze);
});

// 선택 이벤트 수신
aac.on('selection', (event) => {
  console.log('Selected:', event.targetId);
});

// 연결 해제
await aac.disconnect();
```

### Python
```python
from wia_aac import WiaAac

aac = WiaAac()

# 센서 연결
await aac.connect(
    type='eye_tracker',
    device={'manufacturer': 'Tobii'}
)

# 신호 이벤트 수신
@aac.on('signal')
def on_signal(signal):
    print('Gaze:', signal.data.gaze)

# 선택 이벤트 수신
@aac.on('selection')
def on_selection(event):
    print('Selected:', event.target_id)

# 연결 해제
await aac.disconnect()
```

---

## 🚀 작업 시작

이제 Phase 2 작업을 시작하세요.

첫 번째 단계: **Phase 1 Signal Format 타입을 TypeScript/Python으로 변환**

화이팅! 🤟

---

<div align="center">

**Phase 2 of 4**

API Interface Standard

</div>
