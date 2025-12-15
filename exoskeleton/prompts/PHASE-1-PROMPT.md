# WIA Exoskeleton - Phase 1: Motion Data Standard

## 목표
재활 외골격의 운동 데이터 표준 포맷을 정의합니다.

## 1.1 관절 상태 데이터

```typescript
interface JointState {
  // 식별
  joint: JointType;
  side: 'left' | 'right';
  timestamp: number;           // Unix ms

  // 운동학 (Kinematics)
  kinematics: {
    angle: number;             // degrees (해부학적 기준)
    angularVelocity: number;   // deg/s
    angularAcceleration: number; // deg/s²
  };

  // 운동역학 (Kinetics)
  kinetics: {
    torque: number;            // Nm (사용자 생성)
    assistTorque: number;      // Nm (외골격 제공)
    netTorque: number;         // Nm (합산)
  };

  // 센서 원시 데이터
  sensors: {
    encoder: number;           // raw encoder counts
    imu?: IMUData;
    forceplate?: ForceData;
  };
}

enum JointType {
  HIP = 'hip',
  KNEE = 'knee',
  ANKLE = 'ankle',
}

interface IMUData {
  accelerometer: Vector3D;     // m/s²
  gyroscope: Vector3D;         // deg/s
  magnetometer?: Vector3D;     // μT
}
```

## 1.2 보행 주기 데이터

```typescript
interface GaitCycle {
  // 보행 단계
  phase: GaitPhase;
  percentComplete: number;     // 0-100% of cycle

  // 시간 파라미터
  timing: {
    cycleStart: number;        // timestamp
    stanceStart: number;
    swingStart: number;
    cycleDuration: number;     // ms
    stanceDuration: number;
    swingDuration: number;
  };

  // 공간 파라미터
  spatial: {
    strideLength: number;      // cm
    stepLength: number;        // cm
    stepWidth: number;         // cm
    cadence: number;           // steps/min
    velocity: number;          // m/s
  };
}

enum GaitPhase {
  // 단일 지지 (한 발)
  RIGHT_STANCE = 'right_stance',
  LEFT_STANCE = 'left_stance',

  // 이중 지지 (두 발)
  INITIAL_DOUBLE_SUPPORT = 'initial_double_support',
  TERMINAL_DOUBLE_SUPPORT = 'terminal_double_support',

  // 유각기 (발이 땅에서 떨어짐)
  RIGHT_SWING = 'right_swing',
  LEFT_SWING = 'left_swing',

  // 세부 단계
  HEEL_STRIKE = 'heel_strike',
  FOOT_FLAT = 'foot_flat',
  MIDSTANCE = 'midstance',
  HEEL_OFF = 'heel_off',
  TOE_OFF = 'toe_off',
}
```

## 1.3 세션 데이터 구조

```typescript
interface ExoSession {
  // 메타데이터
  metadata: {
    sessionId: string;
    userId: string;
    startTime: Date;
    endTime: Date;
    deviceInfo: ExoDeviceInfo;
  };

  // 설정
  configuration: {
    assistanceMode: AssistanceMode;
    assistanceLevel: number;   // 0-100%
    jointLimits: JointLimits;
  };

  // 시계열 데이터
  timeSeries: {
    sampleRate: number;        // Hz
    jointStates: JointState[];
    gaitCycles: GaitCycle[];
    events: ExoEvent[];
  };

  // 요약 통계
  summary: SessionSummary;
}

enum AssistanceMode {
  PASSIVE = 'passive',         // 저항 없음
  ACTIVE_ASSIST = 'active_assist', // 움직임 보조
  ACTIVE_RESIST = 'active_resist', // 저항 훈련
  TRANSPARENT = 'transparent', // 최소 간섭
  CHALLENGE = 'challenge',     // 적응형 저항
}
```

---

## 산출물

```
exoskeleton/
├── spec/
│   ├── JOINT-STATE-SPEC.md
│   ├── GAIT-CYCLE-SPEC.md
│   └── SESSION-DATA-SPEC.md
├── schemas/
│   ├── joint-state.schema.json
│   ├── gait-cycle.schema.json
│   └── session.schema.json
```

---

## 다음: Phase 2 (제어 명령 표준)
