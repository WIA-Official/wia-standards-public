# WIA Exoskeleton Joint State Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-12-14

## 1. Overview

이 문서는 재활 외골격 시스템에서 관절 상태 데이터를 표현하기 위한 표준 형식을 정의합니다.
관절 상태 데이터는 외골격의 각 관절에서 실시간으로 수집되는 운동학적(kinematic) 및
운동역학적(kinetic) 정보를 포함합니다.

## 2. Data Structure

### 2.1 JointState

관절의 현재 상태를 나타내는 기본 데이터 구조입니다.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `joint` | JointType | Yes | 관절 유형 (hip, knee, ankle) |
| `side` | string | Yes | 좌/우 구분 ('left' \| 'right') |
| `timestamp` | number | Yes | Unix timestamp (milliseconds) |
| `kinematics` | Kinematics | Yes | 운동학적 데이터 |
| `kinetics` | Kinetics | Yes | 운동역학적 데이터 |
| `sensors` | SensorData | Yes | 센서 원시 데이터 |

### 2.2 JointType Enumeration

지원되는 관절 유형:

| Value | Description | Range of Motion |
|-------|-------------|-----------------|
| `hip` | 고관절 (Hip) | Flexion: 0-120°, Extension: 0-30° |
| `knee` | 슬관절 (Knee) | Flexion: 0-135°, Extension: 0° |
| `ankle` | 족관절 (Ankle) | Dorsiflexion: 0-20°, Plantarflexion: 0-50° |

## 3. Kinematics Data

운동학적 데이터는 관절의 위치, 속도, 가속도 정보를 포함합니다.

### 3.1 Fields

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `angle` | number | degrees | 관절 각도 (해부학적 기준) |
| `angularVelocity` | number | deg/s | 각속도 |
| `angularAcceleration` | number | deg/s² | 각가속도 |

### 3.2 Coordinate System

- **해부학적 기준위치 (Anatomical Reference Position)**: 직립 자세에서 모든 관절은 0°
- **Flexion (굴곡)**: 양수 값 (+)
- **Extension (신전)**: 음수 값 (-)

### 3.3 Example

```json
{
  "angle": 45.2,
  "angularVelocity": 120.5,
  "angularAcceleration": 15.3
}
```

## 4. Kinetics Data

운동역학적 데이터는 관절에 작용하는 힘(토크) 정보를 포함합니다.

### 4.1 Fields

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `torque` | number | Nm | 사용자가 생성한 토크 |
| `assistTorque` | number | Nm | 외골격이 제공하는 보조 토크 |
| `netTorque` | number | Nm | 총 순 토크 (torque + assistTorque) |

### 4.2 Sign Convention

- **양수 (+)**: Flexion 방향 토크
- **음수 (-)**: Extension 방향 토크

### 4.3 Example

```json
{
  "torque": 12.5,
  "assistTorque": 8.3,
  "netTorque": 20.8
}
```

## 5. Sensor Data

원시 센서 데이터를 포함합니다.

### 5.1 Fields

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `encoder` | number | Yes | 인코더 raw counts |
| `imu` | IMUData | No | IMU 센서 데이터 |
| `forceplate` | ForceData | No | 힘판 데이터 |

### 5.2 IMU Data Structure

관성측정장치(IMU) 데이터:

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `accelerometer` | Vector3D | m/s² | 3축 가속도 |
| `gyroscope` | Vector3D | deg/s | 3축 각속도 |
| `magnetometer` | Vector3D | μT | 3축 자기장 (optional) |

### 5.3 Vector3D Structure

```json
{
  "x": 0.0,
  "y": 0.0,
  "z": -9.81
}
```

### 5.4 Force Data Structure

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `fx` | number | N | X축 방향 힘 |
| `fy` | number | N | Y축 방향 힘 |
| `fz` | number | N | Z축 방향 힘 (수직 지면 반력) |
| `mx` | number | Nm | X축 모멘트 |
| `my` | number | Nm | Y축 모멘트 |
| `mz` | number | Nm | Z축 모멘트 |
| `cop` | Vector2D | m | 압력 중심 (Center of Pressure) |

## 6. Complete Example

```json
{
  "joint": "knee",
  "side": "right",
  "timestamp": 1702598400000,
  "kinematics": {
    "angle": 45.2,
    "angularVelocity": 120.5,
    "angularAcceleration": 15.3
  },
  "kinetics": {
    "torque": 12.5,
    "assistTorque": 8.3,
    "netTorque": 20.8
  },
  "sensors": {
    "encoder": 32768,
    "imu": {
      "accelerometer": { "x": 0.15, "y": -0.08, "z": -9.78 },
      "gyroscope": { "x": 5.2, "y": -2.1, "z": 0.3 }
    }
  }
}
```

## 7. Sampling Requirements

| Parameter | Minimum | Recommended | Maximum |
|-----------|---------|-------------|---------|
| Sample Rate | 100 Hz | 200 Hz | 1000 Hz |
| Timestamp Resolution | 1 ms | 1 ms | 0.1 ms |
| Angle Resolution | 0.1° | 0.01° | 0.001° |
| Torque Resolution | 0.1 Nm | 0.01 Nm | 0.001 Nm |

## 8. Validation Rules

1. `joint`는 반드시 'hip', 'knee', 'ankle' 중 하나여야 합니다.
2. `side`는 반드시 'left' 또는 'right'여야 합니다.
3. `timestamp`는 양수 정수여야 합니다.
4. `angle`은 관절별 물리적 가동범위 내에 있어야 합니다.
5. `netTorque`는 `torque + assistTorque`와 같아야 합니다.

## 9. Related Specifications

- [GAIT-CYCLE-SPEC.md](./GAIT-CYCLE-SPEC.md) - 보행 주기 데이터 명세
- [SESSION-DATA-SPEC.md](./SESSION-DATA-SPEC.md) - 세션 데이터 명세

## 10. Revision History

| Version | Date | Author | Description |
|---------|------|--------|-------------|
| 1.0.0 | 2025-12-14 | WIA | Initial specification |
