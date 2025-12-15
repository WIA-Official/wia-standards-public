# WIA Exoskeleton Gait Cycle Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-12-14

## 1. Overview

이 문서는 재활 외골격 시스템에서 보행 주기(Gait Cycle) 데이터를 표현하기 위한 표준 형식을
정의합니다. 보행 주기는 한 발의 뒤꿈치 접촉(heel strike)부터 같은 발의 다음 뒤꿈치 접촉까지의
완전한 주기를 나타냅니다.

## 2. Gait Cycle Fundamentals

### 2.1 Normal Gait Cycle Division

정상 보행 주기는 다음과 같이 구분됩니다:

| Phase | Percentage | Description |
|-------|------------|-------------|
| **Stance Phase** | 0-60% | 발이 지면에 접촉한 상태 |
| **Swing Phase** | 60-100% | 발이 공중에 있는 상태 |

### 2.2 Stance Phase Sub-divisions

| Sub-phase | Percentage | Description |
|-----------|------------|-------------|
| Initial Contact | 0-2% | 뒤꿈치 접촉 (Heel Strike) |
| Loading Response | 2-12% | 체중 부하 (Foot Flat) |
| Mid-stance | 12-31% | 중간 입각기 |
| Terminal Stance | 31-50% | 말기 입각기 (Heel Off) |
| Pre-swing | 50-60% | 전유각기 (Toe Off 준비) |

### 2.3 Swing Phase Sub-divisions

| Sub-phase | Percentage | Description |
|-----------|------------|-------------|
| Initial Swing | 60-73% | 초기 유각기 |
| Mid-swing | 73-87% | 중간 유각기 |
| Terminal Swing | 87-100% | 말기 유각기 |

## 3. Data Structure

### 3.1 GaitCycle

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `phase` | GaitPhase | Yes | 현재 보행 단계 |
| `percentComplete` | number | Yes | 주기 진행률 (0-100%) |
| `timing` | GaitTiming | Yes | 시간 파라미터 |
| `spatial` | SpatialParameters | Yes | 공간 파라미터 |

### 3.2 GaitPhase Enumeration

```typescript
enum GaitPhase {
  // Stance Phases (지지기)
  RIGHT_STANCE = 'right_stance',
  LEFT_STANCE = 'left_stance',

  // Double Support Phases (이중 지지기)
  INITIAL_DOUBLE_SUPPORT = 'initial_double_support',
  TERMINAL_DOUBLE_SUPPORT = 'terminal_double_support',

  // Swing Phases (유각기)
  RIGHT_SWING = 'right_swing',
  LEFT_SWING = 'left_swing',

  // Detailed Sub-phases (세부 단계)
  HEEL_STRIKE = 'heel_strike',
  FOOT_FLAT = 'foot_flat',
  MIDSTANCE = 'midstance',
  HEEL_OFF = 'heel_off',
  TOE_OFF = 'toe_off',
}
```

## 4. Timing Parameters

### 4.1 GaitTiming Structure

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `cycleStart` | number | ms (timestamp) | 주기 시작 시간 |
| `stanceStart` | number | ms (timestamp) | 입각기 시작 시간 |
| `swingStart` | number | ms (timestamp) | 유각기 시작 시간 |
| `cycleDuration` | number | ms | 전체 주기 지속 시간 |
| `stanceDuration` | number | ms | 입각기 지속 시간 |
| `swingDuration` | number | ms | 유각기 지속 시간 |

### 4.2 Normal Timing Values

정상 성인 보행 (자유 보행 속도 기준):

| Parameter | Normal Range | Description |
|-----------|--------------|-------------|
| Cycle Duration | 1000-1200 ms | 전체 주기 |
| Stance Duration | 600-720 ms | 입각기 (60%) |
| Swing Duration | 400-480 ms | 유각기 (40%) |
| Double Support | 200-240 ms | 이중 지지기 (총 20%) |
| Single Support | 400-480 ms | 단일 지지기 (40%) |

### 4.3 Timing Ratios

| Ratio | Normal Value | Clinical Significance |
|-------|--------------|----------------------|
| Stance/Swing | 60:40 | 비대칭 시 이상 보행 |
| Double Support % | 20-25% | 증가 시 안정성 저하 |
| Step Time Symmetry | ~1.0 | 1.0에서 벗어날수록 비대칭 |

## 5. Spatial Parameters

### 5.1 SpatialParameters Structure

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `strideLength` | number | cm | 활보장 (한 발 두 번 디딤) |
| `stepLength` | number | cm | 보장 (한 발 한 번 디딤) |
| `stepWidth` | number | cm | 보폭 (좌우 간격) |
| `cadence` | number | steps/min | 분속수 |
| `velocity` | number | m/s | 보행 속도 |

### 5.2 Normal Spatial Values

정상 성인 보행 기준값:

| Parameter | Normal Range | Description |
|-----------|--------------|-------------|
| Stride Length | 130-160 cm | 신장의 약 80% |
| Step Length | 65-80 cm | 활보장의 50% |
| Step Width | 8-12 cm | 골반 너비 기준 |
| Cadence | 100-120 steps/min | 자유 보행 |
| Velocity | 1.2-1.4 m/s | 자유 보행 속도 |

### 5.3 Derived Metrics

```
velocity = (stride_length × cadence) / 120
stride_length = step_length_left + step_length_right
symmetry_index = |left - right| / ((left + right) / 2) × 100
```

## 6. Complete Example

```json
{
  "phase": "midstance",
  "percentComplete": 25.5,
  "timing": {
    "cycleStart": 1702598400000,
    "stanceStart": 1702598400000,
    "swingStart": 1702598400660,
    "cycleDuration": 1100,
    "stanceDuration": 660,
    "swingDuration": 440
  },
  "spatial": {
    "strideLength": 145.2,
    "stepLength": 72.8,
    "stepWidth": 10.5,
    "cadence": 109,
    "velocity": 1.32
  }
}
```

## 7. Phase Transition Detection

### 7.1 Detection Methods

| Transition | Primary Method | Secondary Method |
|------------|----------------|------------------|
| Heel Strike | Vertical GRF > threshold | Foot switch |
| Foot Flat | Foot switch pattern | Ankle angle |
| Heel Off | Heel pressure = 0 | CoP movement |
| Toe Off | Vertical GRF < threshold | Toe switch |

### 7.2 Threshold Values

| Parameter | Typical Threshold |
|-----------|-------------------|
| Heel Strike GRF | > 20 N |
| Toe Off GRF | < 20 N |
| Swing Detection | GRF < 10 N for > 50 ms |

## 8. Gait Events Timeline

```
Time (% of cycle)
0%        10%       20%       30%       40%       50%       60%       70%       80%       90%      100%
|---------|---------|---------|---------|---------|---------|---------|---------|---------|---------|
|←-------- STANCE PHASE (60%) --------→|←------- SWING PHASE (40%) ------→|
|IC |LR   |         MS         |   TS   |PSw|ISw  |    MSw    |  TSw   |

Legend:
IC  = Initial Contact (Heel Strike)
LR  = Loading Response (Foot Flat)
MS  = Mid-stance
TS  = Terminal Stance (Heel Off)
PSw = Pre-swing (Toe Off)
ISw = Initial Swing
MSw = Mid-swing
TSw = Terminal Swing
```

## 9. Validation Rules

1. `percentComplete`는 0 이상 100 이하여야 합니다.
2. `stanceDuration + swingDuration = cycleDuration`
3. `cycleDuration`은 500ms 이상 3000ms 이하 (비정상 보행 포함)
4. `velocity`는 0 이상 3.0 m/s 이하 (달리기 제외)
5. `cadence`는 30 이상 180 이하 steps/min
6. `strideLength`는 30cm 이상 250cm 이하

## 10. Clinical Significance

### 10.1 Common Gait Deviations

| Deviation | Affected Parameter | Typical Pattern |
|-----------|-------------------|-----------------|
| 편마비 (Hemiplegia) | Symmetry | 한쪽 swing 시간 증가 |
| 파킨슨 (Parkinson's) | Stride length | 감소, 동결 현상 |
| 근위축증 (Muscular Dystrophy) | Step width | 증가 (안정성 보상) |
| 관절염 (Arthritis) | Stance time | 통증 측 감소 |

### 10.2 Rehabilitation Goals

| Parameter | Goal | Clinical Meaning |
|-----------|------|------------------|
| Symmetry Index | < 10% | 정상 대칭성 |
| Stance/Swing Ratio | 55-65:35-45 | 정상 비율 회복 |
| Velocity | > 0.8 m/s | 지역사회 보행 능력 |

## 11. Related Specifications

- [JOINT-STATE-SPEC.md](./JOINT-STATE-SPEC.md) - 관절 상태 데이터 명세
- [SESSION-DATA-SPEC.md](./SESSION-DATA-SPEC.md) - 세션 데이터 명세

## 12. Revision History

| Version | Date | Author | Description |
|---------|------|--------|-------------|
| 1.0.0 | 2025-12-14 | WIA | Initial specification |
