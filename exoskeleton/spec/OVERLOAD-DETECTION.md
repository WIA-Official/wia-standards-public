# WIA Exoskeleton Overload Detection Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-12-14
**Safety Classification:** ISO 13849 PLd

## 1. Overview

이 문서는 재활 외골격 시스템의 과부하 감지 및 보호 메커니즘을 정의합니다.
모터, 구조물, 배터리, 전자 장치의 과부하 상태를 실시간으로 모니터링하고
적절한 보호 조치를 취합니다.

## 2. Overload Types

### 2.1 Classification

| Type | Component | Risk Level | Response Time |
|------|-----------|------------|---------------|
| Motor Current | Motor Driver | High | < 5 ms |
| Motor Temperature | Motor | Medium | < 100 ms |
| Structural | Frame/Joints | High | < 10 ms |
| Battery Voltage | Battery | Medium | < 50 ms |
| Battery Current | Battery | High | < 10 ms |
| Battery Temperature | Battery | Medium | < 100 ms |
| Electronics | PCB | High | < 20 ms |

### 2.2 Overload Type Enumeration

```typescript
enum OverloadType {
  // Motor overloads
  MOTOR_CURRENT = 'motor_current',
  MOTOR_TEMPERATURE = 'motor_temp',
  MOTOR_VOLTAGE = 'motor_voltage',

  // Structural overloads
  STRUCTURAL_FORCE = 'structural_force',
  STRUCTURAL_TORQUE = 'structural_torque',

  // Battery overloads
  BATTERY_VOLTAGE_LOW = 'battery_voltage_low',
  BATTERY_VOLTAGE_HIGH = 'battery_voltage_high',
  BATTERY_CURRENT = 'battery_current',
  BATTERY_TEMPERATURE = 'battery_temp',

  // Electronics overloads
  PROCESSOR_TEMPERATURE = 'processor_temp',
  DRIVER_TEMPERATURE = 'driver_temp',

  // Communication
  COMMUNICATION_TIMEOUT = 'comm_timeout',
}
```

## 3. Motor Overload Protection

### 3.1 Current Limits

| Joint | Continuous | Peak | Peak Duration | Trip Threshold |
|-------|------------|------|---------------|----------------|
| Hip | 8 A | 12 A | 1000 ms | 15 A |
| Knee | 10 A | 15 A | 1000 ms | 18 A |
| Ankle | 6 A | 9 A | 1000 ms | 11 A |

### 3.2 I²t Protection

```typescript
interface I2tProtection {
  // Thermal model
  thermalCapacity: number;     // A²·s (thermal mass)
  coolingRate: number;         // A²/s (heat dissipation)
  tripThreshold: number;       // A²·s (trip level)
  warningThreshold: number;    // A²·s (warning level)

  // Current state
  accumulator: number;         // Current I²t value
  lastUpdate: number;          // Last calculation time
}

function updateI2t(
  protection: I2tProtection,
  current: number,
  dt: number
): I2tResult {
  // Calculate I²t contribution
  const i2tContribution = current * current * dt;

  // Calculate cooling
  const cooling = protection.coolingRate * dt;

  // Update accumulator
  protection.accumulator = Math.max(
    0,
    protection.accumulator + i2tContribution - cooling
  );

  // Check thresholds
  if (protection.accumulator >= protection.tripThreshold) {
    return { status: 'TRIP', level: protection.accumulator };
  }
  if (protection.accumulator >= protection.warningThreshold) {
    return { status: 'WARNING', level: protection.accumulator };
  }
  return { status: 'OK', level: protection.accumulator };
}
```

### 3.3 Temperature Limits

| Component | Warning | Limit | Shutdown | Recovery |
|-----------|---------|-------|----------|----------|
| Motor Winding | 70°C | 85°C | 100°C | 60°C |
| Motor Case | 60°C | 75°C | 85°C | 50°C |
| Motor Driver | 70°C | 85°C | 95°C | 60°C |
| Gearbox | 50°C | 65°C | 75°C | 45°C |

### 3.4 Temperature Derating

```
Torque %
   100│ ────────┐
      │         │
    80│         └───┐
      │             │
    60│             └───┐
      │                 │
    40│                 └───┐
      │                     │
    20│                     └───┐
      │                         │
     0└─────────────────────────┴──► Temperature
       50   60   70   80   90  100 °C
```

## 4. Structural Overload Protection

### 4.1 Force/Torque Limits

| Joint | Max Force | Max Torque | Safety Factor |
|-------|-----------|------------|---------------|
| Hip | 2000 N | 80 Nm | 2.0 |
| Knee | 2500 N | 100 Nm | 2.0 |
| Ankle | 1500 N | 50 Nm | 2.0 |
| Frame | 3000 N | - | 2.5 |

### 4.2 Strain Gauge Monitoring

```typescript
interface StrainGaugeConfig {
  id: string;
  location: string;
  calibrationFactor: number;   // N/V or Nm/V
  zeroDrift: number;           // V
  maxOutput: number;           // V
  samplingRate: number;        // Hz

  // Limits
  warningThreshold: number;    // % of limit
  tripThreshold: number;       // % of limit
}

interface StrainGaugeReading {
  timestamp: number;
  rawValue: number;            // V
  calibratedValue: number;     // N or Nm
  percentOfLimit: number;      // %
  status: 'OK' | 'WARNING' | 'OVERLOAD';
}
```

### 4.3 Impact Detection

```typescript
interface ImpactDetector {
  // Configuration
  accelerationThreshold: number;   // g
  jerkThreshold: number;           // g/s
  integrationWindow: number;       // ms

  // Detection
  detectImpact(imuData: IMUData[]): ImpactEvent | null;
}

interface ImpactEvent {
  timestamp: number;
  peakAcceleration: number;    // g
  direction: Vector3D;
  duration: number;            // ms
  severity: 'LIGHT' | 'MODERATE' | 'SEVERE';
  location: string;
}
```

## 5. Battery Overload Protection

### 5.1 Voltage Limits

| State | Min Voltage | Nominal | Max Voltage |
|-------|-------------|---------|-------------|
| Shutdown | 18 V | - | 60 V |
| Low Warning | 22 V | - | - |
| Normal | 22 V | 48 V | 54 V |
| High Warning | - | - | 56 V |
| Charging | 44 V | 54 V | 58 V |

### 5.2 Current Limits

| Condition | Continuous | Peak | Duration |
|-----------|------------|------|----------|
| Discharge | 30 A | 50 A | 10 s |
| Charge | 10 A | 15 A | - |
| Regenerative | 15 A | 25 A | 5 s |

### 5.3 Battery Management

```typescript
interface BatteryProtection {
  voltage: {
    shutdownLow: number;       // V
    warningLow: number;        // V
    warningHigh: number;       // V
    shutdownHigh: number;      // V
  };

  current: {
    continuousMax: number;     // A
    peakMax: number;           // A
    peakDuration: number;      // ms
  };

  temperature: {
    chargeMin: number;         // °C
    chargeMax: number;         // °C
    dischargeMin: number;      // °C
    dischargeMax: number;      // °C
    shutdownMax: number;       // °C
  };

  soc: {
    shutdownLevel: number;     // %
    warningLevel: number;      // %
    fullLevel: number;         // %
  };
}

const DEFAULT_BATTERY_PROTECTION: BatteryProtection = {
  voltage: {
    shutdownLow: 18,
    warningLow: 22,
    warningHigh: 56,
    shutdownHigh: 60,
  },
  current: {
    continuousMax: 30,
    peakMax: 50,
    peakDuration: 10000,
  },
  temperature: {
    chargeMin: 0,
    chargeMax: 45,
    dischargeMin: -20,
    dischargeMax: 60,
    shutdownMax: 70,
  },
  soc: {
    shutdownLevel: 5,
    warningLevel: 20,
    fullLevel: 100,
  },
};
```

## 6. Overload Detection System

### 6.1 Detection Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                   OVERLOAD DETECTION SYSTEM                     │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐          │
│  │ Motor Sensors│  │Strain Gauges │  │Battery Monitor│          │
│  │ • Current    │  │ • Force      │  │ • Voltage    │          │
│  │ • Temperature│  │ • Torque     │  │ • Current    │          │
│  │ • Voltage    │  │              │  │ • Temperature│          │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘          │
│         │                 │                 │                   │
│         ▼                 ▼                 ▼                   │
│  ┌─────────────────────────────────────────────────────┐       │
│  │              SIGNAL CONDITIONING                    │       │
│  │         (Filtering, Calibration, Scaling)           │       │
│  └─────────────────────────┬───────────────────────────┘       │
│                            │                                    │
│                            ▼                                    │
│  ┌─────────────────────────────────────────────────────┐       │
│  │              THRESHOLD COMPARATORS                  │       │
│  │    ┌─────────┐  ┌─────────┐  ┌─────────┐           │       │
│  │    │ Warning │  │  Limit  │  │Shutdown │           │       │
│  │    └────┬────┘  └────┬────┘  └────┬────┘           │       │
│  └─────────┼────────────┼────────────┼─────────────────┘       │
│            │            │            │                          │
│            ▼            ▼            ▼                          │
│  ┌─────────────────────────────────────────────────────┐       │
│  │               ACTION CONTROLLER                     │       │
│  │                                                     │       │
│  │  Warning  → Alert + Log                             │       │
│  │  Limit    → Reduce Power + Alert                    │       │
│  │  Shutdown → E-Stop                                  │       │
│  └─────────────────────────────────────────────────────┘       │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### 6.2 Detection Interface

```typescript
interface IOverloadDetector {
  // Configuration
  configure(config: OverloadConfig): void;
  getConfig(): OverloadConfig;

  // Real-time monitoring
  update(sensorData: SensorData): OverloadStatus;
  getStatus(): OverloadStatus;

  // Individual checks
  checkMotorOverload(motorId: number): MotorOverloadStatus;
  checkStructuralOverload(): StructuralOverloadStatus;
  checkBatteryOverload(): BatteryOverloadStatus;

  // Event handling
  onOverload(callback: (event: OverloadEvent) => void): void;

  // History
  getOverloadHistory(limit?: number): OverloadEvent[];

  // Reset
  reset(type?: OverloadType): void;
  acknowledgeWarning(eventId: string): void;
}
```

### 6.3 Overload Status

```typescript
interface OverloadStatus {
  timestamp: number;
  overall: OverloadLevel;

  motor: {
    [motorId: number]: {
      current: { value: number; status: OverloadLevel };
      temperature: { value: number; status: OverloadLevel };
      i2t: { value: number; status: OverloadLevel };
    };
  };

  structural: {
    [location: string]: {
      force: { value: number; status: OverloadLevel };
      torque: { value: number; status: OverloadLevel };
    };
  };

  battery: {
    voltage: { value: number; status: OverloadLevel };
    current: { value: number; status: OverloadLevel };
    temperature: { value: number; status: OverloadLevel };
    soc: { value: number; status: OverloadLevel };
  };

  activeOverloads: OverloadEvent[];
  warnings: OverloadEvent[];
}

enum OverloadLevel {
  OK = 'ok',
  WARNING = 'warning',
  LIMIT = 'limit',
  SHUTDOWN = 'shutdown',
}
```

## 7. Overload Response

### 7.1 Response Actions

| Level | Response | Description |
|-------|----------|-------------|
| WARNING | Alert + Log | 경고 표시, 이벤트 기록 |
| LIMIT | Reduce Power | 출력 제한, 지속적 모니터링 |
| SHUTDOWN | E-Stop | 즉시 정지, 안전 조치 |

### 7.2 Power Reduction Strategy

```typescript
interface PowerReductionStrategy {
  // Current reduction
  reduceMotorCurrent(
    motorId: number,
    currentLimit: number
  ): void;

  // Torque limiting
  reduceTorqueLimit(
    joint: JointType,
    side: Side,
    factor: number     // 0.0 - 1.0
  ): void;

  // Velocity limiting
  reduceVelocityLimit(
    joint: JointType,
    side: Side,
    factor: number
  ): void;

  // Assistance reduction
  reduceAssistance(factor: number): void;

  // Recovery
  restoreNormal(gradual: boolean): void;
}
```

### 7.3 Recovery Procedure

```
┌─────────────────────────────────────────────────────────────────┐
│                   OVERLOAD RECOVERY                             │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│ STEP 1: Wait for cooldown period                                │
│         Motor: 30s, Battery: 60s, Structural: immediate         │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│ STEP 2: Verify parameter below recovery threshold               │
│         (Usually 80% of warning threshold)                      │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│ STEP 3: Gradually restore power                                 │
│         Start at 50%, increase 10% every 5 seconds              │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│ STEP 4: Monitor for recurrence                                  │
│         If overload recurs, reduce recovery target              │
└─────────────────────────────────────────────────────────────────┘
```

## 8. Overload Event Structure

```typescript
interface OverloadEvent {
  id: string;
  timestamp: number;
  type: OverloadType;
  level: OverloadLevel;
  source: string;              // Motor ID, battery, etc.
  value: number;
  threshold: number;
  unit: string;
  duration?: number;           // How long exceeded
  action: OverloadAction;
  resolved: boolean;
  resolvedAt?: number;
  notes?: string;
}

enum OverloadAction {
  LOG = 'log',
  ALERT = 'alert',
  REDUCE_POWER = 'reduce_power',
  DISABLE_JOINT = 'disable_joint',
  E_STOP = 'e_stop',
}
```

## 9. Configuration

### 9.1 Overload Configuration

```typescript
interface OverloadConfig {
  motor: {
    currentWarning: number;     // % of rated
    currentLimit: number;       // % of rated
    currentShutdown: number;    // % of rated
    tempWarning: number;        // °C
    tempLimit: number;          // °C
    tempShutdown: number;       // °C
    i2tCapacity: number;        // A²·s
  };

  structural: {
    forceWarning: number;       // % of limit
    forceShutdown: number;      // % of limit
    torqueWarning: number;      // % of limit
    torqueShutdown: number;     // % of limit
  };

  battery: {
    voltageWarningLow: number;  // V
    voltageShutdownLow: number; // V
    voltageWarningHigh: number; // V
    voltageShutdownHigh: number;// V
    currentWarning: number;     // A
    currentShutdown: number;    // A
    tempWarning: number;        // °C
    tempShutdown: number;       // °C
  };

  recovery: {
    cooldownPeriod: number;     // ms
    recoveryThreshold: number;  // % of warning
    gradualRecovery: boolean;
    recoveryRate: number;       // % per second
  };
}
```

## 10. Related Specifications

- [EMERGENCY-STOP.md](./EMERGENCY-STOP.md) - 비상 정지 명세
- [JOINT-LIMITS.md](./JOINT-LIMITS.md) - 관절 가동 범위 제한
- [SAFETY-CHECKLIST.md](./SAFETY-CHECKLIST.md) - 안전 체크리스트

## 11. Revision History

| Version | Date | Author | Description |
|---------|------|--------|-------------|
| 1.0.0 | 2025-12-14 | WIA | Initial specification |
