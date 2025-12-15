# WIA Exoskeleton Joint Limits Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-12-14
**Safety Classification:** ISO 13849 PLd

## 1. Overview

이 문서는 재활 외골격의 관절 가동 범위(ROM) 제한을 정의합니다.
안전한 운용을 위해 각도, 속도, 토크의 제한값을 설정하고
이를 실시간으로 모니터링합니다.

## 2. Human Anatomical Reference

### 2.1 Normal ROM (Healthy Adult)

| Joint | Motion | Normal ROM | Functional ROM |
|-------|--------|------------|----------------|
| **Hip** | Flexion | 0° → 120° | 0° → 100° |
| | Extension | 0° → -30° | 0° → -20° |
| | Abduction | 0° → 45° | 0° → 30° |
| | Adduction | 0° → -30° | 0° → -20° |
| **Knee** | Flexion | 0° → 140° | 0° → 120° |
| | Extension | 0° (full) | 0° |
| **Ankle** | Dorsiflexion | 0° → 20° | 0° → 15° |
| | Plantarflexion | 0° → -50° | 0° → -40° |

### 2.2 Anatomical Reference Position

- **Zero Position (0°)**: 해부학적 직립 자세
- **Flexion**: 양수 (+)
- **Extension**: 음수 (-)

```
Standing Position (0°)

     Head
      │
   ┌──┴──┐
   │     │  ← Hip 0°
   │     │
   │     │
   └──┬──┘
      │     ← Knee 0°
      │
   ┌──┴──┐
   │     │  ← Ankle 0°
   └─────┘
```

## 3. Limit Types

### 3.1 Hardware Limits (Hard Limits)

물리적으로 초과 불가능한 한계:

| Joint | Min Angle | Max Angle | Enforcement |
|-------|-----------|-----------|-------------|
| Hip | -35° | 125° | Mechanical stop |
| Knee | -5° | 145° | Mechanical stop |
| Ankle | -35° | 55° | Mechanical stop |

### 3.2 Software Limits (Soft Limits)

소프트웨어로 제어되는 한계:

| Joint | Default Min | Default Max | Configurable |
|-------|-------------|-------------|--------------|
| Hip | -20° | 110° | Yes |
| Knee | 0° | 130° | Yes |
| Ankle | -25° | 45° | Yes |

### 3.3 User-Specific Limits

개별 사용자에 맞춤 설정되는 한계:

```typescript
interface UserJointLimits {
  userId: string;
  prescription: {
    prescribedBy: string;      // Therapist ID
    prescribedDate: Date;
    diagnosis?: string;
  };
  limits: {
    hip: PersonalizedLimit;
    knee: PersonalizedLimit;
    ankle: PersonalizedLimit;
  };
}

interface PersonalizedLimit {
  left: {
    min: number;
    max: number;
    painThreshold?: number;    // Angle where pain begins
    targetROM?: number;        // Rehabilitation goal
  };
  right: {
    min: number;
    max: number;
    painThreshold?: number;
    targetROM?: number;
  };
}
```

## 4. Limit Data Structure

### 4.1 JointLimits Interface

```typescript
interface JointLimits {
  joint: JointType;
  side: Side;

  // Angle limits
  angle: {
    hardMin: number;           // Hardware limit (degrees)
    hardMax: number;           // Hardware limit (degrees)
    softMin: number;           // Software limit (degrees)
    softMax: number;           // Software limit (degrees)
    softLimitMargin: number;   // Warning zone before soft limit
  };

  // Velocity limits
  velocity: {
    max: number;               // Maximum velocity (deg/s)
    rampRate: number;          // Maximum acceleration (deg/s²)
    approachSpeed: number;     // Speed near limits (deg/s)
  };

  // Torque limits
  torque: {
    continuous: number;        // Continuous max torque (Nm)
    peak: number;              // Peak torque (Nm)
    peakDuration: number;      // Max peak duration (ms)
    assistMax: number;         // Max assistance torque (Nm)
    resistMax: number;         // Max resistance torque (Nm)
  };
}
```

### 4.2 Default Limits

```typescript
const DEFAULT_JOINT_LIMITS: Record<JointType, JointLimits> = {
  [JointType.HIP]: {
    joint: JointType.HIP,
    side: 'both',
    angle: {
      hardMin: -35,
      hardMax: 125,
      softMin: -20,
      softMax: 110,
      softLimitMargin: 10,
    },
    velocity: {
      max: 200,
      rampRate: 400,
      approachSpeed: 50,
    },
    torque: {
      continuous: 40,
      peak: 60,
      peakDuration: 1000,
      assistMax: 50,
      resistMax: 30,
    },
  },
  [JointType.KNEE]: {
    joint: JointType.KNEE,
    side: 'both',
    angle: {
      hardMin: -5,
      hardMax: 145,
      softMin: 0,
      softMax: 130,
      softLimitMargin: 10,
    },
    velocity: {
      max: 250,
      rampRate: 500,
      approachSpeed: 60,
    },
    torque: {
      continuous: 50,
      peak: 80,
      peakDuration: 1000,
      assistMax: 60,
      resistMax: 40,
    },
  },
  [JointType.ANKLE]: {
    joint: JointType.ANKLE,
    side: 'both',
    angle: {
      hardMin: -35,
      hardMax: 55,
      softMin: -25,
      softMax: 45,
      softLimitMargin: 8,
    },
    velocity: {
      max: 150,
      rampRate: 300,
      approachSpeed: 40,
    },
    torque: {
      continuous: 25,
      peak: 40,
      peakDuration: 1000,
      assistMax: 35,
      resistMax: 20,
    },
  },
};
```

## 5. Limit Enforcement

### 5.1 Enforcement Zones

```
          Hard Min                              Hard Max
             │                                     │
             │    Soft Min              Soft Max   │
             │       │                     │       │
             │       │    Warning   Warning│       │
             │       │      Zone     Zone  │       │
             ▼       ▼       ▼       ▼     ▼       ▼
    ─────────┼───────┼───────┼───────┼─────┼───────┼─────────
    BLOCKED  │DANGER │WARNING│ SAFE  │WARN │DANGER │ BLOCKED
             │       │       │       │     │       │
             │  ◄────┼───────┼───────┼─────┼────►  │
             │       │       │       │     │       │
             │    Soft Limit        Soft Limit     │
             │    Margin            Margin         │
```

### 5.2 Zone Actions

| Zone | Action | Response |
|------|--------|----------|
| **SAFE** | Normal operation | No restrictions |
| **WARNING** | Reduce velocity | `approachSpeed` limit applied |
| **DANGER** | Resist movement | Increasing resistance torque |
| **BLOCKED** | Prevent movement | E-Stop if breached |

### 5.3 Enforcement Algorithm

```typescript
function enforceLimits(
  joint: JointType,
  side: Side,
  currentAngle: number,
  commandedTorque: number
): EnforcementResult {
  const limits = getLimits(joint, side);

  // Check hard limits (immediate E-Stop)
  if (currentAngle <= limits.angle.hardMin ||
      currentAngle >= limits.angle.hardMax) {
    return {
      zone: 'BLOCKED',
      action: 'E_STOP',
      torque: 0,
      reason: 'Hard limit reached',
    };
  }

  // Check soft limits
  const marginMin = limits.angle.softMin + limits.angle.softLimitMargin;
  const marginMax = limits.angle.softMax - limits.angle.softLimitMargin;

  // Danger zone (between soft and hard limits)
  if (currentAngle < limits.angle.softMin ||
      currentAngle > limits.angle.softMax) {
    const resistTorque = calculateResistTorque(
      currentAngle,
      limits,
      commandedTorque
    );
    return {
      zone: 'DANGER',
      action: 'RESIST',
      torque: resistTorque,
      reason: 'Soft limit exceeded',
    };
  }

  // Warning zone
  if (currentAngle < marginMin || currentAngle > marginMax) {
    return {
      zone: 'WARNING',
      action: 'REDUCE_VELOCITY',
      torque: commandedTorque,
      maxVelocity: limits.velocity.approachSpeed,
      reason: 'Approaching soft limit',
    };
  }

  // Safe zone
  return {
    zone: 'SAFE',
    action: 'NONE',
    torque: commandedTorque,
    reason: null,
  };
}
```

## 6. Velocity Limiting

### 6.1 Velocity Profiles

| Condition | Max Velocity | Ramp Rate |
|-----------|--------------|-----------|
| Normal Operation | 100% | 100% |
| Warning Zone | 25% | 50% |
| Near Hard Limit | 10% | 25% |
| Recovery Mode | 50% | 50% |

### 6.2 Velocity Ramping

```
Velocity
   ▲
   │           ┌─────────────────┐
Max│           │                 │
   │          ╱                   ╲
   │         ╱                     ╲
   │        ╱                       ╲
   │       ╱                         ╲
   │      ╱                           ╲
   │     ╱                             ╲
   └────┴───────────────────────────────┴──► Time
        │← Ramp Up →│← Constant →│← Ramp Down →│
```

### 6.3 Velocity Limit Check

```typescript
function checkVelocityLimit(
  joint: JointType,
  side: Side,
  currentVelocity: number,
  targetVelocity: number,
  currentAngle: number
): VelocityCheckResult {
  const limits = getLimits(joint, side);
  const zone = getZone(currentAngle, limits);

  // Determine max allowed velocity based on zone
  let maxVelocity: number;
  switch (zone) {
    case 'SAFE':
      maxVelocity = limits.velocity.max;
      break;
    case 'WARNING':
      maxVelocity = limits.velocity.approachSpeed;
      break;
    case 'DANGER':
      maxVelocity = limits.velocity.approachSpeed * 0.4;
      break;
    default:
      maxVelocity = 0;
  }

  // Check if target velocity exceeds limit
  const clampedVelocity = Math.sign(targetVelocity) *
    Math.min(Math.abs(targetVelocity), maxVelocity);

  // Check acceleration limit
  const dt = 0.001; // 1ms sample time
  const acceleration = (clampedVelocity - currentVelocity) / dt;
  const maxAccel = limits.velocity.rampRate;

  if (Math.abs(acceleration) > maxAccel) {
    const rampedVelocity = currentVelocity +
      Math.sign(acceleration) * maxAccel * dt;
    return {
      allowed: true,
      velocity: rampedVelocity,
      limited: true,
      reason: 'Acceleration limited',
    };
  }

  return {
    allowed: true,
    velocity: clampedVelocity,
    limited: clampedVelocity !== targetVelocity,
    reason: clampedVelocity !== targetVelocity ? 'Velocity limited' : null,
  };
}
```

## 7. Torque Limiting

### 7.1 Torque Limit Types

| Type | Limit | Duration | Application |
|------|-------|----------|-------------|
| Continuous | 100% rated | Unlimited | Normal operation |
| Peak | 150% rated | < 1000 ms | Transient loads |
| Assist | Variable | Unlimited | User assistance |
| Resist | Variable | Unlimited | Resistance training |

### 7.2 Thermal Model

```typescript
interface ThermalModel {
  ambientTemp: number;         // °C
  motorTemp: number;           // °C
  thermalTimeConstant: number; // seconds
  maxTemp: number;             // °C
  deratingCurve: DerateCurve;
}

interface DerateCurve {
  tempThresholds: number[];    // [50, 60, 70, 80]
  torqueFactors: number[];     // [1.0, 0.9, 0.7, 0.5]
}

function getTorqueDerating(motorTemp: number): number {
  // Linear interpolation on derating curve
  // Returns factor 0.0 - 1.0
}
```

## 8. Limit Violation Handling

### 8.1 Violation Severity

| Severity | Condition | Response |
|----------|-----------|----------|
| **INFO** | Approaching warning zone | Log only |
| **WARNING** | In warning zone | Reduce velocity, alert |
| **ERROR** | Soft limit exceeded | Resist movement, alarm |
| **CRITICAL** | Hard limit reached | E-Stop |

### 8.2 Violation Event

```typescript
interface LimitViolation {
  id: string;
  timestamp: number;
  joint: JointType;
  side: Side;
  limitType: 'angle' | 'velocity' | 'torque';
  severity: ViolationSeverity;
  currentValue: number;
  limitValue: number;
  exceedance: number;         // How much over limit
  duration: number;           // How long exceeded (ms)
  action: ViolationAction;
}

enum ViolationSeverity {
  INFO = 'info',
  WARNING = 'warning',
  ERROR = 'error',
  CRITICAL = 'critical',
}

enum ViolationAction {
  LOG = 'log',
  ALERT = 'alert',
  REDUCE_VELOCITY = 'reduce_velocity',
  RESIST = 'resist',
  E_STOP = 'e_stop',
}
```

## 9. Configuration Interface

### 9.1 Limit Configuration API

```typescript
interface IJointLimitsConfig {
  // Get current limits
  getLimits(joint: JointType, side: Side): JointLimits;

  // Set soft limits (within hard limits)
  setSoftLimits(
    joint: JointType,
    side: Side,
    min: number,
    max: number
  ): ConfigResult;

  // Set velocity limits
  setVelocityLimits(
    joint: JointType,
    side: Side,
    max: number,
    rampRate: number
  ): ConfigResult;

  // Set torque limits
  setTorqueLimits(
    joint: JointType,
    side: Side,
    continuous: number,
    peak: number
  ): ConfigResult;

  // Apply user-specific limits
  applyUserLimits(userLimits: UserJointLimits): ConfigResult;

  // Reset to defaults
  resetToDefaults(joint?: JointType, side?: Side): void;

  // Validation
  validateLimits(limits: JointLimits): ValidationResult;
}
```

## 10. Related Specifications

- [EMERGENCY-STOP.md](./EMERGENCY-STOP.md) - 비상 정지 명세
- [OVERLOAD-DETECTION.md](./OVERLOAD-DETECTION.md) - 과부하 감지
- [SAFETY-CHECKLIST.md](./SAFETY-CHECKLIST.md) - 안전 체크리스트

## 11. Revision History

| Version | Date | Author | Description |
|---------|------|--------|-------------|
| 1.0.0 | 2025-12-14 | WIA | Initial specification |
