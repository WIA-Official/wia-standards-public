# WIA Exoskeleton Emergency Stop Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-12-14
**Safety Classification:** IEC 62443 SL2 / ISO 13849 PLd

## 1. Overview

이 문서는 재활 외골격 시스템의 비상 정지(E-Stop) 프로토콜을 정의합니다.
비상 정지는 사용자와 장비의 안전을 보장하기 위한 최우선 기능으로,
모든 상황에서 즉각적이고 신뢰할 수 있게 작동해야 합니다.

## 2. E-Stop Categories

### 2.1 Stop Categories (IEC 60204-1)

| Category | Description | Response Time | Method |
|----------|-------------|---------------|--------|
| **Category 0** | 즉시 전원 차단 | < 10 ms | Uncontrolled stop |
| **Category 1** | 제어된 정지 후 전원 차단 | < 500 ms | Controlled stop |
| **Category 2** | 제어된 정지, 전원 유지 | < 1000 ms | Controlled stop |

### 2.2 WIA Exoskeleton E-Stop Implementation

| Trigger Type | Stop Category | Application |
|--------------|---------------|-------------|
| Hardware Button | Category 0 | 물리적 비상 버튼 |
| Software Command | Category 1 | 소프트웨어 비상 정지 |
| Watchdog Timeout | Category 1 | 통신 두절 |
| Limit Switch | Category 1 | 기계적 한계 도달 |
| Overcurrent | Category 0 | 과전류 감지 |
| Overtemperature | Category 1 | 과열 감지 |

## 3. Trigger Conditions

### 3.1 Hardware Triggers

| Trigger | Threshold | Response Time | Priority |
|---------|-----------|---------------|----------|
| E-Stop Button | Active Low | < 5 ms | 0 (Highest) |
| Limit Switch | Contact Open | < 10 ms | 1 |
| Overcurrent | > 150% rated | < 5 ms | 0 |
| Overtemperature | > 80°C motor | < 100 ms | 2 |
| Overvoltage | > 60V DC | < 10 ms | 1 |
| Undervoltage | < 18V DC | < 100 ms | 2 |

### 3.2 Software Triggers

| Trigger | Condition | Response Time | Priority |
|---------|-----------|---------------|----------|
| Software Command | API call | < 20 ms | 1 |
| Watchdog Timeout | No heartbeat > 100ms | < 150 ms | 1 |
| Position Error | > 20° deviation | < 50 ms | 2 |
| Velocity Error | > 50% deviation | < 50 ms | 2 |
| Sensor Failure | Invalid reading | < 50 ms | 1 |
| Fall Detection | IMU trigger | < 100 ms | 1 |

### 3.3 Trigger Data Structure

```typescript
interface EStopTrigger {
  id: string;
  timestamp: number;
  type: EStopTriggerType;
  source: EStopSource;
  reason: string;
  value?: number;
  threshold?: number;
  priority: number;
}

enum EStopTriggerType {
  HARDWARE_BUTTON = 'hardware_button',
  SOFTWARE_COMMAND = 'software_command',
  WATCHDOG_TIMEOUT = 'watchdog_timeout',
  LIMIT_SWITCH = 'limit_switch',
  OVERCURRENT = 'overcurrent',
  OVERTEMPERATURE = 'overtemperature',
  OVERVOLTAGE = 'overvoltage',
  UNDERVOLTAGE = 'undervoltage',
  POSITION_ERROR = 'position_error',
  VELOCITY_ERROR = 'velocity_error',
  SENSOR_FAILURE = 'sensor_failure',
  FALL_DETECTION = 'fall_detection',
}

enum EStopSource {
  MOTOR_CONTROLLER = 'motor_controller',
  SAFETY_PROCESSOR = 'safety_processor',
  MAIN_PROCESSOR = 'main_processor',
  USER_INPUT = 'user_input',
  EXTERNAL_SYSTEM = 'external_system',
}
```

## 4. E-Stop Actions

### 4.1 Action Sequence

```
┌─────────────────────────────────────────────────────────────────┐
│                     E-STOP TRIGGERED                            │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│ PHASE 1: Immediate Actions (< 10ms)                             │
│ ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  │
│ │ Disable Motors  │  │ Engage Brakes   │  │ Cut Power (Cat0)│  │
│ └─────────────────┘  └─────────────────┘  └─────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│ PHASE 2: Alert Actions (< 100ms)                                │
│ ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  │
│ │ Sound Alarm     │  │ Flash LED       │  │ Notify Operator │  │
│ └─────────────────┘  └─────────────────┘  └─────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│ PHASE 3: Logging Actions (< 500ms)                              │
│ ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  │
│ │ Log Event       │  │ Save State      │  │ Generate Report │  │
│ └─────────────────┘  └─────────────────┘  └─────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

### 4.2 Action Configuration

| Action | Default | Configurable | Description |
|--------|---------|--------------|-------------|
| `motorDisable` | true | No | 모든 모터 비활성화 |
| `brakeEngage` | true | No | 전자식 브레이크 작동 |
| `powerCut` | Cat0 only | No | 전원 차단 |
| `alertSound` | true | Yes | 경고음 발생 |
| `alertVisual` | true | Yes | LED 점멸 |
| `logEvent` | true | No | 이벤트 로깅 |
| `notifyOperator` | true | Yes | 운영자 알림 |
| `notifyRemote` | false | Yes | 원격 알림 |

### 4.3 Action Data Structure

```typescript
interface EStopAction {
  motorDisable: boolean;
  brakeEngage: boolean;
  powerCut: boolean;
  alertSound: boolean;
  alertVisual: boolean;
  logEvent: boolean;
  notifyOperator: boolean;
  notifyRemote: boolean;
}

interface EStopEvent {
  id: string;
  timestamp: number;
  trigger: EStopTrigger;
  actions: EStopAction;
  systemState: SystemStateSnapshot;
  resetInfo?: EStopResetInfo;
}
```

## 5. E-Stop Reset

### 5.1 Reset Requirements

| Requirement | Category 0 | Category 1 | Category 2 |
|-------------|------------|------------|------------|
| Manual Reset Required | Yes | Yes | Optional |
| Safety Check Required | Yes | Yes | Yes |
| Cooldown Period | 5000 ms | 2000 ms | 1000 ms |
| Operator Confirmation | Yes | Yes | Optional |
| System Self-Test | Full | Partial | Quick |

### 5.2 Reset Procedure

```
┌─────────────────────────────────────────────────────────────────┐
│                     E-STOP RESET REQUEST                        │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│ CHECK 1: Cooldown Period Elapsed?                               │
│          └── No → Wait remaining time                           │
└─────────────────────────────────────────────────────────────────┘
                              │ Yes
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│ CHECK 2: Trigger Condition Cleared?                             │
│          └── No → Cannot reset until condition cleared          │
└─────────────────────────────────────────────────────────────────┘
                              │ Yes
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│ CHECK 3: Manual Reset Button Pressed?                           │
│          └── No → Await operator action                         │
└─────────────────────────────────────────────────────────────────┘
                              │ Yes
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│ CHECK 4: System Self-Test Passed?                               │
│          └── No → Report failure, remain in E-Stop              │
└─────────────────────────────────────────────────────────────────┘
                              │ Yes
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                     E-STOP RESET COMPLETE                       │
│                     System Ready for Operation                  │
└─────────────────────────────────────────────────────────────────┘
```

### 5.3 Reset Configuration

```typescript
interface EStopResetConfig {
  requireManualReset: boolean;
  safetyCheckRequired: boolean;
  cooldownPeriod: number;        // ms
  operatorConfirmation: boolean;
  selfTestLevel: SelfTestLevel;
  maxResetAttempts: number;
  resetLockoutTime: number;      // ms after max attempts
}

enum SelfTestLevel {
  QUICK = 'quick',              // Basic connectivity
  PARTIAL = 'partial',          // Sensors + motors
  FULL = 'full',                // Complete system test
}

interface EStopResetInfo {
  resetTimestamp: number;
  resetBy: string;              // Operator ID
  selfTestResult: SelfTestResult;
  resetAttempt: number;
}
```

## 6. Safety Architecture

### 6.1 Dual-Channel Safety

```
┌─────────────────────────────────────────────────────────────────┐
│                    SAFETY ARCHITECTURE                          │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌─────────────────────┐     ┌─────────────────────┐           │
│  │  CHANNEL A          │     │  CHANNEL B          │           │
│  │  (Safety Processor) │     │  (Main Processor)   │           │
│  │                     │     │                     │           │
│  │  • E-Stop Logic     │     │  • E-Stop Logic     │           │
│  │  • Watchdog         │     │  • Watchdog         │           │
│  │  • Limit Monitoring │     │  • Limit Monitoring │           │
│  └──────────┬──────────┘     └──────────┬──────────┘           │
│             │                           │                       │
│             ▼                           ▼                       │
│  ┌──────────────────────────────────────────────────┐          │
│  │              DUAL-CHANNEL VOTER                  │          │
│  │         (Either channel can trigger E-Stop)      │          │
│  └──────────────────────────────────────────────────┘          │
│                           │                                     │
│                           ▼                                     │
│  ┌──────────────────────────────────────────────────┐          │
│  │              POWER SWITCHING                     │          │
│  │         (Redundant contactors)                   │          │
│  └──────────────────────────────────────────────────┘          │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### 6.2 Watchdog Timer

| Parameter | Value | Description |
|-----------|-------|-------------|
| Heartbeat Interval | 50 ms | 정상 동작 신호 간격 |
| Timeout Threshold | 100 ms | 타임아웃 임계값 |
| Max Missed Heartbeats | 2 | 최대 허용 누락 횟수 |
| Recovery Time | 500 ms | 복구 후 안정화 시간 |

## 7. E-Stop Interface

### 7.1 API Interface

```typescript
interface IEmergencyStop {
  // Trigger E-Stop
  trigger(reason: EStopTriggerType, source: EStopSource): void;

  // Check E-Stop status
  isActive(): boolean;
  getActiveEvent(): EStopEvent | null;

  // Reset E-Stop
  requestReset(operatorId: string): ResetResult;
  canReset(): ResetEligibility;

  // Configuration
  configure(config: Partial<EStopConfig>): void;
  getConfig(): EStopConfig;

  // Event handlers
  onTriggered(callback: (event: EStopEvent) => void): void;
  onReset(callback: (info: EStopResetInfo) => void): void;

  // Status
  getStatus(): EStopStatus;
  getHistory(limit?: number): EStopEvent[];
}

interface EStopStatus {
  isActive: boolean;
  category: StopCategory;
  activeEvent?: EStopEvent;
  timeSinceTriggered?: number;
  resetEligible: boolean;
  cooldownRemaining?: number;
}

interface ResetEligibility {
  canReset: boolean;
  reasons: string[];
  cooldownRemaining: number;
  pendingChecks: string[];
}

interface ResetResult {
  success: boolean;
  reason?: string;
  selfTestResult?: SelfTestResult;
}
```

## 8. Examples

### 8.1 Hardware E-Stop Event

```json
{
  "id": "estop-001",
  "timestamp": 1702598400000,
  "trigger": {
    "id": "trig-001",
    "timestamp": 1702598400000,
    "type": "hardware_button",
    "source": "user_input",
    "reason": "Physical E-Stop button pressed",
    "priority": 0
  },
  "actions": {
    "motorDisable": true,
    "brakeEngage": true,
    "powerCut": true,
    "alertSound": true,
    "alertVisual": true,
    "logEvent": true,
    "notifyOperator": true,
    "notifyRemote": false
  },
  "systemState": {
    "jointStates": [...],
    "batteryLevel": 85,
    "motorTemperatures": [45, 42, 48, 44, 46, 43]
  }
}
```

### 8.2 Overcurrent E-Stop Event

```json
{
  "id": "estop-002",
  "timestamp": 1702598500000,
  "trigger": {
    "id": "trig-002",
    "timestamp": 1702598500000,
    "type": "overcurrent",
    "source": "motor_controller",
    "reason": "Motor 3 (right knee) overcurrent detected",
    "value": 18.5,
    "threshold": 12.0,
    "priority": 0
  },
  "actions": {
    "motorDisable": true,
    "brakeEngage": true,
    "powerCut": true,
    "alertSound": true,
    "alertVisual": true,
    "logEvent": true,
    "notifyOperator": true,
    "notifyRemote": true
  }
}
```

## 9. Testing Requirements

### 9.1 Required Tests

| Test | Frequency | Pass Criteria |
|------|-----------|---------------|
| Hardware Button Response | Daily | < 10 ms response |
| Software E-Stop Response | Daily | < 50 ms response |
| Watchdog Timeout | Weekly | < 150 ms trigger |
| Limit Switch Function | Weekly | < 10 ms response |
| Overcurrent Protection | Monthly | Correct threshold |
| Full Reset Procedure | Monthly | Complete success |

### 9.2 Test Protocol

1. **Pre-Test Safety Check**: 테스트 환경 안전 확인
2. **Controlled Trigger**: 각 트리거 유형별 테스트
3. **Response Time Measurement**: 응답 시간 측정
4. **Action Verification**: 모든 동작 확인
5. **Reset Verification**: 리셋 절차 검증
6. **Documentation**: 테스트 결과 기록

## 10. Related Specifications

- [JOINT-LIMITS.md](./JOINT-LIMITS.md) - 관절 가동 범위 제한
- [OVERLOAD-DETECTION.md](./OVERLOAD-DETECTION.md) - 과부하 감지
- [SAFETY-CHECKLIST.md](./SAFETY-CHECKLIST.md) - 안전 체크리스트

## 11. Revision History

| Version | Date | Author | Description |
|---------|------|--------|-------------|
| 1.0.0 | 2025-12-14 | WIA | Initial specification |
