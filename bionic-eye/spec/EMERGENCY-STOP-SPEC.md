# WIA Bionic Eye - Emergency Stop Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-12-15

## 1. Overview

이 문서는 인공 시각 시스템의 비상 정지 프로토콜을 정의합니다.

## 2. Emergency Triggers

### 2.1 Automatic Triggers

```typescript
interface AutoTriggers {
  // 과전류 감지
  overCurrent: {
    enabled: boolean;
    threshold: number;            // μA per electrode
    totalThreshold: number;       // mA total
    responseTime: number;         // μs (< 100)
  };

  // 과전압 감지
  overVoltage: {
    enabled: boolean;
    threshold: number;            // V
    responseTime: number;         // μs
  };

  // 임피던스 이상
  impedanceAnomaly: {
    enabled: boolean;
    suddenChangeThreshold: number; // % (급격한 변화)
    absoluteHighThreshold: number; // kΩ (개방)
    absoluteLowThreshold: number;  // kΩ (단락)
    responseTime: number;          // ms
  };

  // 온도 초과
  thermalOverload: {
    enabled: boolean;
    maxTemperatureRise: number;   // °C above baseline
    absoluteMax: number;          // °C
    responseTime: number;         // ms
  };

  // 전하 불균형
  chargeImbalance: {
    enabled: boolean;
    threshold: number;            // % imbalance
    windowDuration: number;       // ms
  };

  // 통신 오류
  communicationLoss: {
    enabled: boolean;
    timeout: number;              // ms
  };
}
```

### 2.2 Manual Triggers

```typescript
interface ManualTriggers {
  // 환자 버튼
  patientButton: {
    enabled: boolean;
    location: 'glasses_frame' | 'pendant' | 'wristband' | 'remote';
    confirmationRequired: boolean;
    holdDuration: number;         // ms (0 = instant)
  };

  // 음성 명령
  voiceCommand: {
    enabled: boolean;
    keywords: string[];           // ["stop", "emergency", "멈춰", "정지"]
    confirmationRequired: boolean;
    sensitivity: number;          // 0-1
  };

  // 보호자 리모컨
  caregiverRemote: {
    enabled: boolean;
    range: number;                // meters
    authentication: boolean;
  };

  // 앱 제어
  appControl: {
    enabled: boolean;
    authentication: 'pin' | 'biometric' | 'both';
  };
}
```

## 3. Stop Sequence

### 3.1 Three-Phase Stop

```typescript
interface StopSequence {
  // Phase 1: 즉시 (< 1ms)
  immediate: {
    disableAllOutputs: boolean;
    shortCircuitElectrodes: boolean;  // 안전 방전
    dischargeCapacitors: boolean;
    openCircuitBackup: boolean;
  };

  // Phase 2: 빠름 (< 100ms)
  fast: {
    logTriggerEvent: boolean;
    logLastParameters: boolean;
    saveCurrentState: boolean;
    notifyProcessor: boolean;
  };

  // Phase 3: 후속 (< 1s)
  followup: {
    alertPatient: AlertMethod[];
    alertCaregiver: boolean;
    alertMedicalTeam: boolean;
    uploadEventLog: boolean;
  };
}

type AlertMethod = 'vibration' | 'beep' | 'voice' | 'visual';
```

### 3.2 Timing Requirements

| Phase | Max Time | Actions |
|-------|----------|---------|
| Immediate | < 1ms | Output disable, discharge |
| Fast | < 100ms | Logging, notification |
| Followup | < 1s | Alerts, upload |

## 4. Safe State Definition

```typescript
interface SafeState {
  allElectrodesDisabled: boolean;
  capacitorsCharged: number;      // should be 0
  outputVoltage: number;          // should be 0
  dcLeakage: number;              // nA (should be < 10)

  indicatorStatus: {
    patientLED: 'red_blinking';
    processorLED: 'red_solid';
    audioAlert: boolean;
  };

  communicationStatus: 'active';   // 통신은 유지
}
```

## 5. Restart Protocol

### 5.1 Restart Conditions

```typescript
interface RestartProtocol {
  // 재시작 조건
  conditions: {
    autoRestartAllowed: boolean;  // 자동 재시작 허용 여부
    cooldownPeriod: number;       // seconds (최소 대기)
    maxAutoRestarts: number;      // per hour
  };

  // 필수 점검
  checksRequired: {
    impedanceCheck: boolean;
    voltageCheck: boolean;
    temperatureCheck: boolean;
    calibrationCheck: boolean;
    patientConfirmation: boolean;
  };

  // 트리거별 정책
  triggerPolicies: {
    overCurrent: RestartPolicy;
    overVoltage: RestartPolicy;
    impedanceAnomaly: RestartPolicy;
    thermalOverload: RestartPolicy;
    patientRequest: RestartPolicy;
  };
}

interface RestartPolicy {
  autoRestartAllowed: boolean;
  requiredCooldown: number;       // seconds
  requireMedicalClearance: boolean;
  maxOccurrences: number;         // before lockout
  lockoutDuration: number;        // hours
}
```

### 5.2 Restart Checklist

```
□ E-Stop 원인 해결 확인
□ 냉각 시간 경과 확인
□ 임피던스 점검 통과
□ 전압 출력 정상 범위
□ 온도 정상 범위
□ 환자 상태 확인
□ 환자 동의 획득
```

## 6. Event Logging

```typescript
interface EmergencyEvent {
  eventId: string;
  timestamp: Date;

  trigger: {
    type: TriggerType;
    source: 'automatic' | 'manual';
    value?: number;               // 트리거 값
    threshold?: number;           // 임계값
  };

  context: {
    activeElectrodes: number[];
    currentAmplitudes: number[];
    impedanceValues: number[];
    temperature: number;
    sessionDuration: number;      // minutes
  };

  response: {
    stopLatency: number;          // μs
    allPhasesCompleted: boolean;
    alertsSent: string[];
  };

  resolution?: {
    resolvedAt: Date;
    resolvedBy: string;
    action: string;
    notes: string;
  };
}
```

## 7. Related Specifications

- [ELECTRICAL-SAFETY-SPEC.md](./ELECTRICAL-SAFETY-SPEC.md)
- [TISSUE-SAFETY-SPEC.md](./TISSUE-SAFETY-SPEC.md)
