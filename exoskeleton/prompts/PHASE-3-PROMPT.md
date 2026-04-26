# WIA Exoskeleton - Phase 3: Safety Protocol

## 목표
외골격 운용의 안전 표준을 정의합니다.

## 3.1 비상 정지 (E-Stop)

```typescript
interface EmergencyStop {
  // 트리거 조건
  triggers: {
    hardwareButton: boolean;   // 물리 버튼
    softwareCommand: boolean;  // 소프트웨어 명령
    watchdog: boolean;         // 통신 타임아웃
    limitSwitch: boolean;      // 기계적 한계
    overcurrent: boolean;      // 과전류
    overtemperature: boolean;  // 과열
  };

  // E-Stop 동작
  action: {
    motorDisable: boolean;     // 모터 비활성화
    brakeEngage: boolean;      // 브레이크 작동
    alertSound: boolean;       // 경고음
    logEvent: boolean;         // 이벤트 기록
  };

  // E-Stop 해제
  reset: {
    requireManualReset: boolean;
    safetyCheckRequired: boolean;
    cooldownPeriod: number;    // ms
  };
}

// E-Stop 실행
function executeEmergencyStop(reason: EStopReason): void {
  // 1. 즉시 모터 비활성화
  disableAllMotors();

  // 2. 브레이크 작동
  engageAllBrakes();

  // 3. 알림
  soundAlarm();
  notifyOperator(reason);

  // 4. 기록
  logEmergencyStop(reason, Date.now());
}
```

## 3.2 관절 가동 범위 (ROM) 제한

```typescript
interface JointLimits {
  joint: JointType;
  side: Side;

  // 각도 한계
  angle: {
    min: number;               // degrees
    max: number;               // degrees
    softLimitMargin: number;   // 소프트 리밋 여유
  };

  // 속도 한계
  velocity: {
    max: number;               // deg/s
    rampRate: number;          // deg/s²
  };

  // 토크 한계
  torque: {
    max: number;               // Nm
    continuous: number;        // 연속 최대
    peak: number;              // 순간 최대
  };
}

// 기본 인체 관절 한계 (건강한 성인 기준)
const HUMAN_ROM_LIMITS: JointLimits[] = [
  { joint: 'hip', side: 'both', angle: { min: -20, max: 120, softLimitMargin: 5 } },
  { joint: 'knee', side: 'both', angle: { min: 0, max: 140, softLimitMargin: 5 } },
  { joint: 'ankle', side: 'both', angle: { min: -20, max: 50, softLimitMargin: 5 } },
];
```

## 3.3 과부하 감지

```typescript
interface OverloadDetection {
  // 모터 과부하
  motor: {
    currentLimit: number;      // A
    temperatureLimit: number;  // °C
    dutyCycleLimit: number;    // %
  };

  // 구조 과부하
  structural: {
    strainGaugeLimit: number;  // N
    torqueLimit: number;       // Nm
  };

  // 배터리
  battery: {
    lowVoltageThreshold: number;   // V
    highCurrentThreshold: number;  // A
    temperatureLimit: number;      // °C
  };

  // 과부하 시 동작
  onOverload(type: OverloadType, value: number): void;
}

enum OverloadType {
  MOTOR_CURRENT = 'motor_current',
  MOTOR_TEMP = 'motor_temp',
  STRUCTURAL = 'structural',
  BATTERY_LOW = 'battery_low',
  BATTERY_TEMP = 'battery_temp',
}
```

## 3.4 안전 체크리스트

```typescript
interface PreOperationChecklist {
  // 하드웨어 점검
  hardware: {
    batteryLevel: () => boolean;       // > 20%
    motorConnections: () => boolean;   // 모든 모터 연결
    sensorCalibration: () => boolean;  // 센서 캘리브레이션
    mechanicalIntegrity: () => boolean; // 기계적 무결성
    eStopFunctional: () => boolean;    // E-Stop 동작
  };

  // 사용자 점검
  user: {
    properFit: () => boolean;          // 착용 상태
    comfortCheck: () => boolean;       // 편안함
    rangeOfMotionTest: () => boolean;  // ROM 테스트
    emergencyProcedure: () => boolean; // 비상 절차 숙지
  };

  // 실행 전 전체 점검
  runChecklist(): ChecklistResult;
}
```

---

## 산출물

```
exoskeleton/
├── spec/
│   ├── EMERGENCY-STOP.md
│   ├── JOINT-LIMITS.md
│   ├── OVERLOAD-DETECTION.md
│   └── SAFETY-CHECKLIST.md
├── api/rust/src/
│   └── safety/
│       ├── estop.rs
│       ├── limits.rs
│       └── overload.rs
```

---

## 다음: Phase 4 (재활 프로토콜)
