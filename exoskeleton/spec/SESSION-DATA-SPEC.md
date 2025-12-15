# WIA Exoskeleton Session Data Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-12-14

## 1. Overview

이 문서는 재활 외골격 훈련 세션의 전체 데이터 구조를 정의합니다.
세션 데이터는 메타데이터, 설정, 시계열 데이터, 요약 통계를 포함하는
완전한 훈련 기록을 표현합니다.

## 2. Data Structure

### 2.1 ExoSession (Top-level)

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `metadata` | SessionMetadata | Yes | 세션 메타데이터 |
| `configuration` | SessionConfiguration | Yes | 외골격 설정 |
| `timeSeries` | TimeSeriesData | Yes | 시계열 데이터 |
| `summary` | SessionSummary | Yes | 요약 통계 |

## 3. Session Metadata

### 3.1 SessionMetadata Structure

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `sessionId` | string | Yes | 고유 세션 식별자 (UUID v4) |
| `userId` | string | Yes | 사용자/환자 식별자 |
| `startTime` | string | Yes | 세션 시작 시간 (ISO 8601) |
| `endTime` | string | Yes | 세션 종료 시간 (ISO 8601) |
| `deviceInfo` | ExoDeviceInfo | Yes | 외골격 장치 정보 |

### 3.2 ExoDeviceInfo Structure

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `deviceId` | string | Yes | 장치 고유 식별자 |
| `model` | string | Yes | 외골격 모델명 |
| `firmwareVersion` | string | Yes | 펌웨어 버전 |
| `calibrationDate` | string | Yes | 마지막 캘리브레이션 날짜 |
| `sensorConfig` | SensorConfiguration | No | 센서 설정 정보 |

### 3.3 Example

```json
{
  "sessionId": "550e8400-e29b-41d4-a716-446655440000",
  "userId": "patient-001",
  "startTime": "2025-12-14T09:00:00Z",
  "endTime": "2025-12-14T09:30:00Z",
  "deviceInfo": {
    "deviceId": "EXO-2025-001",
    "model": "WIA-RehabExo-Pro",
    "firmwareVersion": "2.5.1",
    "calibrationDate": "2025-12-01"
  }
}
```

## 4. Session Configuration

### 4.1 SessionConfiguration Structure

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `assistanceMode` | AssistanceMode | Yes | 보조 모드 |
| `assistanceLevel` | number | Yes | 보조 강도 (0-100%) |
| `jointLimits` | JointLimits | Yes | 관절 가동 범위 제한 |

### 4.2 AssistanceMode Enumeration

| Value | Description | Use Case |
|-------|-------------|----------|
| `passive` | 저항 없음, 자유 움직임 | 평가, 워밍업 |
| `active_assist` | 움직임 보조 | 초기 재활, 약한 근력 |
| `active_resist` | 저항 훈련 | 근력 강화 |
| `transparent` | 최소 간섭 | 자연스러운 보행 |
| `challenge` | 적응형 저항 | 진행된 재활 |

### 4.3 AssistanceMode Details

```typescript
enum AssistanceMode {
  PASSIVE = 'passive',           // 모터 비활성화, 자유 움직임
  ACTIVE_ASSIST = 'active_assist',   // 사용자 의도 감지 후 보조
  ACTIVE_RESIST = 'active_resist',   // 움직임에 저항 제공
  TRANSPARENT = 'transparent',   // 마찰/관성 보상만
  CHALLENGE = 'challenge',       // 실시간 난이도 조절
}
```

### 4.4 JointLimits Structure

각 관절의 가동 범위 제한:

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `hip` | RangeLimit | degrees | 고관절 제한 |
| `knee` | RangeLimit | degrees | 슬관절 제한 |
| `ankle` | RangeLimit | degrees | 족관절 제한 |

### 4.5 RangeLimit Structure

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `min` | number | degrees | 최소 각도 (Extension 제한) |
| `max` | number | degrees | 최대 각도 (Flexion 제한) |
| `velocityLimit` | number | deg/s | 최대 각속도 제한 |
| `torqueLimit` | number | Nm | 최대 토크 제한 |

### 4.6 Configuration Example

```json
{
  "assistanceMode": "active_assist",
  "assistanceLevel": 60,
  "jointLimits": {
    "hip": {
      "min": -10,
      "max": 100,
      "velocityLimit": 180,
      "torqueLimit": 40
    },
    "knee": {
      "min": 0,
      "max": 120,
      "velocityLimit": 200,
      "torqueLimit": 50
    },
    "ankle": {
      "min": -20,
      "max": 40,
      "velocityLimit": 150,
      "torqueLimit": 30
    }
  }
}
```

## 5. Time Series Data

### 5.1 TimeSeriesData Structure

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `sampleRate` | number | Yes | 샘플링 주파수 (Hz) |
| `jointStates` | JointState[] | Yes | 관절 상태 배열 |
| `gaitCycles` | GaitCycle[] | Yes | 보행 주기 배열 |
| `events` | ExoEvent[] | Yes | 이벤트 배열 |

### 5.2 ExoEvent Structure

세션 중 발생한 이벤트 기록:

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `timestamp` | number | Yes | 이벤트 발생 시간 (Unix ms) |
| `type` | EventType | Yes | 이벤트 유형 |
| `severity` | EventSeverity | Yes | 심각도 |
| `message` | string | Yes | 이벤트 설명 |
| `data` | object | No | 추가 데이터 |

### 5.3 EventType Enumeration

| Value | Description |
|-------|-------------|
| `session_start` | 세션 시작 |
| `session_end` | 세션 종료 |
| `mode_change` | 보조 모드 변경 |
| `assistance_change` | 보조 강도 변경 |
| `fall_detected` | 낙상 감지 |
| `stumble_detected` | 비틀거림 감지 |
| `fatigue_warning` | 피로 경고 |
| `limit_reached` | 관절 제한 도달 |
| `emergency_stop` | 비상 정지 |
| `calibration` | 캘리브레이션 수행 |
| `rest_period` | 휴식 시작/종료 |

### 5.4 EventSeverity Enumeration

| Value | Description | Action |
|-------|-------------|--------|
| `info` | 정보성 | 기록만 |
| `warning` | 경고 | 모니터링 필요 |
| `error` | 오류 | 조치 필요 |
| `critical` | 위험 | 즉시 조치 |

### 5.5 Event Example

```json
{
  "timestamp": 1702598400500,
  "type": "fatigue_warning",
  "severity": "warning",
  "message": "User fatigue detected - consider rest",
  "data": {
    "fatigueScore": 0.75,
    "elapsedTime": 1200000,
    "recommendedRestDuration": 300000
  }
}
```

## 6. Session Summary

### 6.1 SessionSummary Structure

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `duration` | DurationSummary | Yes | 시간 요약 |
| `distance` | DistanceSummary | Yes | 거리 요약 |
| `gait` | GaitSummary | Yes | 보행 요약 |
| `performance` | PerformanceSummary | Yes | 수행 요약 |
| `energy` | EnergySummary | No | 에너지 요약 |

### 6.2 DurationSummary

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `total` | number | ms | 총 세션 시간 |
| `active` | number | ms | 활동 시간 |
| `rest` | number | ms | 휴식 시간 |
| `standing` | number | ms | 기립 시간 |
| `walking` | number | ms | 보행 시간 |

### 6.3 DistanceSummary

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `total` | number | m | 총 이동 거리 |
| `steps` | number | count | 총 걸음 수 |
| `strides` | number | count | 총 활보 수 |

### 6.4 GaitSummary

| Field | Type | Description |
|-------|------|-------------|
| `avgVelocity` | number | 평균 보행 속도 (m/s) |
| `maxVelocity` | number | 최대 보행 속도 (m/s) |
| `avgCadence` | number | 평균 분속수 (steps/min) |
| `avgStrideLength` | number | 평균 활보장 (cm) |
| `symmetryIndex` | number | 좌우 대칭성 지수 (%) |
| `stanceSwingRatio` | number | 입각/유각 비율 |

### 6.5 PerformanceSummary

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `avgAssistanceUsed` | number | % | 평균 보조 사용량 |
| `peakTorque` | TorqueSummary | Nm | 최대 토크 기록 |
| `rangeOfMotion` | ROMSummary | degrees | 가동 범위 사용 |
| `fatigueProgression` | number[] | score | 피로도 진행 |

### 6.6 EnergySummary

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `metabolicCost` | number | J/kg/m | 대사 비용 |
| `assistEnergy` | number | J | 외골격 제공 에너지 |
| `userEnergy` | number | J | 사용자 소모 에너지 |
| `efficiency` | number | % | 에너지 효율 |

### 6.7 Complete Summary Example

```json
{
  "duration": {
    "total": 1800000,
    "active": 1500000,
    "rest": 300000,
    "standing": 200000,
    "walking": 1300000
  },
  "distance": {
    "total": 850.5,
    "steps": 1250,
    "strides": 625
  },
  "gait": {
    "avgVelocity": 0.85,
    "maxVelocity": 1.15,
    "avgCadence": 95,
    "avgStrideLength": 136.08,
    "symmetryIndex": 8.5,
    "stanceSwingRatio": 1.45
  },
  "performance": {
    "avgAssistanceUsed": 45.2,
    "peakTorque": {
      "hip": { "left": 35.2, "right": 38.1 },
      "knee": { "left": 42.5, "right": 45.8 },
      "ankle": { "left": 22.3, "right": 24.1 }
    },
    "rangeOfMotion": {
      "hip": { "left": { "min": -5, "max": 85 }, "right": { "min": -8, "max": 88 } },
      "knee": { "left": { "min": 5, "max": 110 }, "right": { "min": 3, "max": 115 } },
      "ankle": { "left": { "min": -15, "max": 35 }, "right": { "min": -12, "max": 38 } }
    },
    "fatigueProgression": [0.1, 0.25, 0.35, 0.5, 0.65, 0.75]
  },
  "energy": {
    "metabolicCost": 3.2,
    "assistEnergy": 15000,
    "userEnergy": 45000,
    "efficiency": 78.5
  }
}
```

## 7. Complete Session Example

```json
{
  "metadata": {
    "sessionId": "550e8400-e29b-41d4-a716-446655440000",
    "userId": "patient-001",
    "startTime": "2025-12-14T09:00:00Z",
    "endTime": "2025-12-14T09:30:00Z",
    "deviceInfo": {
      "deviceId": "EXO-2025-001",
      "model": "WIA-RehabExo-Pro",
      "firmwareVersion": "2.5.1",
      "calibrationDate": "2025-12-01"
    }
  },
  "configuration": {
    "assistanceMode": "active_assist",
    "assistanceLevel": 60,
    "jointLimits": {
      "hip": { "min": -10, "max": 100, "velocityLimit": 180, "torqueLimit": 40 },
      "knee": { "min": 0, "max": 120, "velocityLimit": 200, "torqueLimit": 50 },
      "ankle": { "min": -20, "max": 40, "velocityLimit": 150, "torqueLimit": 30 }
    }
  },
  "timeSeries": {
    "sampleRate": 200,
    "jointStates": [],
    "gaitCycles": [],
    "events": [
      {
        "timestamp": 1702598400000,
        "type": "session_start",
        "severity": "info",
        "message": "Session started"
      }
    ]
  },
  "summary": {
    "duration": { "total": 1800000, "active": 1500000, "rest": 300000, "standing": 200000, "walking": 1300000 },
    "distance": { "total": 850.5, "steps": 1250, "strides": 625 },
    "gait": { "avgVelocity": 0.85, "maxVelocity": 1.15, "avgCadence": 95, "avgStrideLength": 136.08, "symmetryIndex": 8.5, "stanceSwingRatio": 1.45 },
    "performance": { "avgAssistanceUsed": 45.2 }
  }
}
```

## 8. Data Storage

### 8.1 File Naming Convention

```
{userId}_{sessionId}_{startDate}.exo.json
```

Example: `patient-001_550e8400_2025-12-14.exo.json`

### 8.2 Compression

- 시계열 데이터는 gzip 압축 권장
- 압축 시 확장자: `.exo.json.gz`

### 8.3 Data Retention

| Data Type | Retention Period |
|-----------|------------------|
| Raw Time Series | 90 days |
| Compressed Archive | 5 years |
| Summary Statistics | Indefinite |

## 9. Validation Rules

1. `sessionId`는 유효한 UUID v4 형식이어야 합니다.
2. `startTime`은 `endTime`보다 이전이어야 합니다.
3. `assistanceLevel`은 0-100 범위여야 합니다.
4. `sampleRate`는 50Hz 이상이어야 합니다.
5. `jointStates` 타임스탬프는 순차적이어야 합니다.
6. 요약 통계는 시계열 데이터와 일관성이 있어야 합니다.

## 10. Related Specifications

- [JOINT-STATE-SPEC.md](./JOINT-STATE-SPEC.md) - 관절 상태 데이터 명세
- [GAIT-CYCLE-SPEC.md](./GAIT-CYCLE-SPEC.md) - 보행 주기 데이터 명세

## 11. Revision History

| Version | Date | Author | Description |
|---------|------|--------|-------------|
| 1.0.0 | 2025-12-14 | WIA | Initial specification |
