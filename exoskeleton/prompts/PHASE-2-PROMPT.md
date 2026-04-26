# WIA Exoskeleton - Phase 2: Control Command Standard

## 목표
외골격 제어 명령의 표준 인터페이스를 정의합니다.

## 2.1 제어 모드

```typescript
interface ExoController {
  // 제어 모드 선택
  setControlMode(mode: ControlMode): void;

  // 위치 제어
  positionControl(joint: JointType, side: Side, angle: number): void;

  // 속도 제어
  velocityControl(joint: JointType, side: Side, velocity: number): void;

  // 토크 제어
  torqueControl(joint: JointType, side: Side, torque: number): void;

  // 임피던스 제어
  impedanceControl(
    joint: JointType,
    side: Side,
    params: ImpedanceParams
  ): void;
}

enum ControlMode {
  POSITION = 'position',       // 위치 추종
  VELOCITY = 'velocity',       // 속도 추종
  TORQUE = 'torque',           // 토크 추종
  IMPEDANCE = 'impedance',     // 임피던스 (강성/댐핑)
  ADMITTANCE = 'admittance',   // 어드미턴스
  ZERO_TORQUE = 'zero_torque', // 투명 모드
}

interface ImpedanceParams {
  stiffness: number;           // Nm/rad (K)
  damping: number;             // Nm·s/rad (B)
  equilibriumAngle: number;    // degrees (θ₀)
}
```

## 2.2 사용자 의도 기반 제어

```typescript
interface IntentBasedController {
  // 의도 감지 소스
  intentSources: {
    emg?: EMGIntentDetector;           // 근전도
    forceplate?: ForceIntentDetector;  // 지면 반력
    imu?: IMUIntentDetector;           // 가속도/자이로
    button?: ButtonIntentDetector;     // 버튼 입력
    bci?: BCIIntentDetector;           // 뇌-컴퓨터 인터페이스
  };

  // 의도 → 동작 매핑
  detectIntent(): UserIntent;
  executeIntent(intent: UserIntent): void;
}

enum UserIntent {
  STAND_UP = 'stand_up',
  SIT_DOWN = 'sit_down',
  WALK_FORWARD = 'walk_forward',
  WALK_BACKWARD = 'walk_backward',
  TURN_LEFT = 'turn_left',
  TURN_RIGHT = 'turn_right',
  STOP = 'stop',
  STAIR_ASCEND = 'stair_ascend',
  STAIR_DESCEND = 'stair_descend',
}
```

## 2.3 보조 수준 조절

```typescript
interface AssistanceController {
  // 정적 보조 수준
  setAssistanceLevel(percent: number): void;  // 0-100%

  // 동적 보조 (적응형)
  adaptiveAssistance: {
    enabled: boolean;
    algorithm: 'error_based' | 'emg_based' | 'fatigue_based';
    minAssistance: number;
    maxAssistance: number;
    adaptationRate: number;
  };

  // 관절별 개별 조절
  setJointAssistance(joint: JointType, side: Side, percent: number): void;
}

// 예: 오류 기반 적응형 보조
function errorBasedAdaptation(
  targetTrajectory: number[],
  actualTrajectory: number[],
  currentAssistance: number
): number {
  const trackingError = calculateRMSE(targetTrajectory, actualTrajectory);
  if (trackingError > THRESHOLD) {
    return Math.min(currentAssistance + 5, 100);  // 보조 증가
  } else {
    return Math.max(currentAssistance - 2, 0);    // 보조 감소
  }
}
```

---

## 산출물

```
exoskeleton/
├── api/
│   ├── typescript/src/
│   │   ├── controller/
│   │   └── intent/
│   └── rust/src/
│       ├── control.rs
│       └── intent.rs
├── spec/
│   ├── CONTROL-MODES.md
│   └── INTENT-DETECTION.md
```

---

## 다음: Phase 3 (안전 프로토콜)
