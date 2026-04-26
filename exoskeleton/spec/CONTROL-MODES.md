# WIA Exoskeleton Control Modes Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-12-14

## 1. Overview

이 문서는 재활 외골격 시스템의 제어 모드와 명령 인터페이스를 정의합니다.
외골격 제어는 위치, 속도, 토크, 임피던스 등 다양한 방식으로 수행되며,
재활 목적에 따라 적절한 모드를 선택합니다.

## 2. Control Modes

### 2.1 Mode Overview

| Mode | Description | Use Case | Response Time |
|------|-------------|----------|---------------|
| `position` | 목표 각도 추종 | 반복 훈련, 패턴 학습 | < 10 ms |
| `velocity` | 목표 속도 추종 | 속도 제어 훈련 | < 10 ms |
| `torque` | 목표 토크 추종 | 근력 훈련, 저항 훈련 | < 5 ms |
| `impedance` | 가상 강성/댐핑 | 자연스러운 상호작용 | < 5 ms |
| `admittance` | 힘→움직임 변환 | 힘 감지 기반 제어 | < 10 ms |
| `zero_torque` | 투명 모드 | 평가, 자유 움직임 | < 2 ms |

### 2.2 Position Control

목표 각도로 관절을 이동시키는 제어 모드입니다.

```
τ = Kp(θd - θ) + Kd(θ̇d - θ̇)
```

| Parameter | Type | Unit | Description |
|-----------|------|------|-------------|
| `targetAngle` | number | degrees | 목표 각도 |
| `maxVelocity` | number | deg/s | 최대 허용 속도 |
| `maxAcceleration` | number | deg/s² | 최대 허용 가속도 |
| `kp` | number | Nm/deg | 비례 게인 |
| `kd` | number | Nm·s/deg | 미분 게인 |

**Safety Constraints:**
- 최대 속도 제한: 200 deg/s
- 최대 토크 제한: 관절별 상이 (hip: 60Nm, knee: 80Nm, ankle: 40Nm)
- 가동 범위 제한 활성화 필수

### 2.3 Velocity Control

목표 각속도로 관절을 움직이는 제어 모드입니다.

```
τ = Kv(θ̇d - θ̇) + Ki∫(θ̇d - θ̇)dt
```

| Parameter | Type | Unit | Description |
|-----------|------|------|-------------|
| `targetVelocity` | number | deg/s | 목표 각속도 |
| `maxTorque` | number | Nm | 최대 허용 토크 |
| `kv` | number | Nm·s/deg | 속도 게인 |
| `ki` | number | Nm/deg | 적분 게인 |

### 2.4 Torque Control

목표 토크를 직접 제어하는 모드입니다.

| Parameter | Type | Unit | Description |
|-----------|------|------|-------------|
| `targetTorque` | number | Nm | 목표 토크 |
| `rampRate` | number | Nm/s | 토크 증가율 제한 |
| `feedforward` | number | Nm | 피드포워드 토크 |

**Safety Constraints:**
- 급격한 토크 변화 방지 (rampRate 필수)
- 사용자 저항 감지 시 즉시 감소

### 2.5 Impedance Control

가상의 기계적 임피던스(강성 + 댐핑)를 구현합니다.

```
τ = K(θ0 - θ) + B(0 - θ̇)
```

| Parameter | Type | Unit | Description |
|-----------|------|------|-------------|
| `stiffness` | number | Nm/rad | 가상 강성 (K) |
| `damping` | number | Nm·s/rad | 가상 댐핑 (B) |
| `equilibriumAngle` | number | degrees | 평형 각도 (θ₀) |
| `inertia` | number | kg·m² | 가상 관성 (optional) |

**Stiffness Ranges:**

| Level | Stiffness (Nm/rad) | Use Case |
|-------|-------------------|----------|
| Very Low | 0 - 5 | 자유 움직임, 평가 |
| Low | 5 - 20 | 가이드 보조 |
| Medium | 20 - 50 | 일반 보행 보조 |
| High | 50 - 100 | 강한 보조/저항 |
| Very High | 100+ | 고정, 잠금 |

### 2.6 Admittance Control

외력을 감지하여 움직임으로 변환합니다.

```
θ̈d = (1/M)(F - B·θ̇d - K·θd)
```

| Parameter | Type | Unit | Description |
|-----------|------|------|-------------|
| `virtualMass` | number | kg | 가상 질량 (M) |
| `virtualDamping` | number | Ns/m | 가상 댐핑 (B) |
| `virtualStiffness` | number | N/m | 가상 강성 (K) |
| `forceThreshold` | number | N | 움직임 개시 힘 |

### 2.7 Zero Torque Mode (Transparent)

모터가 관절 움직임을 방해하지 않도록 합니다.

```
τ = -τ_friction - τ_gravity
```

| Parameter | Type | Unit | Description |
|-----------|------|------|-------------|
| `frictionCompensation` | boolean | - | 마찰 보상 활성화 |
| `gravityCompensation` | boolean | - | 중력 보상 활성화 |
| `inertiaCompensation` | boolean | - | 관성 보상 활성화 |

## 3. Controller Interface

### 3.1 ControlCommand Structure

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `id` | string | Yes | 명령 고유 식별자 |
| `timestamp` | number | Yes | 명령 생성 시간 (Unix ms) |
| `joint` | JointType | Yes | 대상 관절 |
| `side` | 'left' \| 'right' | Yes | 좌/우 |
| `mode` | ControlMode | Yes | 제어 모드 |
| `params` | ControlParams | Yes | 모드별 파라미터 |
| `priority` | CommandPriority | Yes | 명령 우선순위 |
| `timeout` | number | No | 명령 유효 시간 (ms) |

### 3.2 CommandPriority

| Priority | Value | Description |
|----------|-------|-------------|
| `emergency` | 0 | 비상 정지 (최우선) |
| `safety` | 1 | 안전 관련 명령 |
| `therapeutic` | 2 | 치료 목적 명령 |
| `normal` | 3 | 일반 제어 명령 |
| `background` | 4 | 배경 작업 |

### 3.3 ControlResponse Structure

| Field | Type | Description |
|-------|------|-------------|
| `commandId` | string | 원본 명령 ID |
| `status` | ResponseStatus | 실행 상태 |
| `timestamp` | number | 응답 시간 |
| `actualValue` | number | 실제 달성 값 |
| `error` | ControlError | 오류 정보 (있을 경우) |

### 3.4 ResponseStatus

| Status | Description |
|--------|-------------|
| `accepted` | 명령 수락됨 |
| `executing` | 실행 중 |
| `completed` | 완료됨 |
| `rejected` | 거부됨 |
| `timeout` | 시간 초과 |
| `error` | 오류 발생 |

## 4. Assistance Level Control

### 4.1 Static Assistance

고정된 보조 수준을 설정합니다.

| Parameter | Type | Range | Description |
|-----------|------|-------|-------------|
| `level` | number | 0-100% | 전체 보조 수준 |
| `hipAssist` | number | 0-100% | 고관절 보조 수준 |
| `kneeAssist` | number | 0-100% | 슬관절 보조 수준 |
| `ankleAssist` | number | 0-100% | 족관절 보조 수준 |

### 4.2 Adaptive Assistance

실시간으로 보조 수준을 조절합니다.

| Algorithm | Basis | Description |
|-----------|-------|-------------|
| `error_based` | 추적 오류 | 목표 궤적 추종 오류 기반 |
| `emg_based` | 근전도 | 근육 활성화 수준 기반 |
| `fatigue_based` | 피로도 | 피로 징후 감지 기반 |
| `performance_based` | 수행도 | 보행 파라미터 기반 |

### 4.3 Adaptive Parameters

| Parameter | Type | Description |
|-----------|------|-------------|
| `enabled` | boolean | 적응형 제어 활성화 |
| `algorithm` | string | 적응 알고리즘 선택 |
| `minAssistance` | number | 최소 보조 수준 (%) |
| `maxAssistance` | number | 최대 보조 수준 (%) |
| `adaptationRate` | number | 적응 속도 (%/s) |
| `windowSize` | number | 평가 윈도우 크기 (ms) |

### 4.4 Error-Based Adaptation Algorithm

```
if RMSE > threshold_high:
    assistance = min(current + rate, max_assistance)
elif RMSE < threshold_low:
    assistance = max(current - rate/2, min_assistance)
else:
    assistance = current  # 유지
```

## 5. Trajectory Generation

### 5.1 Reference Trajectories

| Type | Description | Parameters |
|------|-------------|------------|
| `minimum_jerk` | 최소 저크 궤적 | duration, start, end |
| `cycloid` | 사이클로이드 궤적 | amplitude, period |
| `gait_template` | 보행 패턴 템플릿 | gait_type, speed |
| `custom` | 사용자 정의 | waypoints[] |

### 5.2 Minimum Jerk Trajectory

```
θ(t) = θ0 + (θf - θ0) × [10(t/T)³ - 15(t/T)⁴ + 6(t/T)⁵]
```

### 5.3 Gait Templates

| Template | Description | Default Speed |
|----------|-------------|---------------|
| `normal_walk` | 정상 보행 | 1.2 m/s |
| `slow_walk` | 느린 보행 | 0.6 m/s |
| `stair_ascend` | 계단 오르기 | 0.4 m/s |
| `stair_descend` | 계단 내리기 | 0.4 m/s |
| `sit_to_stand` | 앉았다 일어서기 | 2.0 s duration |
| `stand_to_sit` | 섰다 앉기 | 2.5 s duration |

## 6. Command Examples

### 6.1 Position Control Command

```json
{
  "id": "cmd-001",
  "timestamp": 1702598400000,
  "joint": "knee",
  "side": "right",
  "mode": "position",
  "params": {
    "targetAngle": 45.0,
    "maxVelocity": 100.0,
    "maxAcceleration": 200.0,
    "kp": 2.5,
    "kd": 0.1
  },
  "priority": "normal",
  "timeout": 5000
}
```

### 6.2 Impedance Control Command

```json
{
  "id": "cmd-002",
  "timestamp": 1702598400100,
  "joint": "hip",
  "side": "left",
  "mode": "impedance",
  "params": {
    "stiffness": 30.0,
    "damping": 2.0,
    "equilibriumAngle": 20.0
  },
  "priority": "therapeutic",
  "timeout": 10000
}
```

### 6.3 Adaptive Assistance Command

```json
{
  "id": "cmd-003",
  "timestamp": 1702598400200,
  "type": "assistance_config",
  "params": {
    "enabled": true,
    "algorithm": "error_based",
    "minAssistance": 20,
    "maxAssistance": 80,
    "adaptationRate": 2.0,
    "windowSize": 1000
  }
}
```

## 7. Safety Constraints

### 7.1 Hard Limits (Hardware Enforced)

| Joint | Max Angle | Min Angle | Max Velocity | Max Torque |
|-------|-----------|-----------|--------------|------------|
| Hip | 120° | -30° | 200 deg/s | 60 Nm |
| Knee | 135° | 0° | 250 deg/s | 80 Nm |
| Ankle | 50° | -30° | 150 deg/s | 40 Nm |

### 7.2 Soft Limits (Software Configurable)

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `softAngleMargin` | 10° | 5-20° | 하드 리밋 전 감속 시작 |
| `softVelocityLimit` | 80% | 50-100% | 최대 속도의 비율 |
| `softTorqueLimit` | 70% | 50-100% | 최대 토크의 비율 |

## 8. Related Specifications

- [JOINT-STATE-SPEC.md](./JOINT-STATE-SPEC.md) - 관절 상태 데이터 명세
- [INTENT-DETECTION.md](./INTENT-DETECTION.md) - 의도 감지 명세
- [SESSION-DATA-SPEC.md](./SESSION-DATA-SPEC.md) - 세션 데이터 명세

## 9. Revision History

| Version | Date | Author | Description |
|---------|------|--------|-------------|
| 1.0.0 | 2025-12-14 | WIA | Initial specification |
