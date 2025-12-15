# WIA Exoskeleton Intent Detection Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-12-14

## 1. Overview

이 문서는 재활 외골격 시스템에서 사용자의 움직임 의도를 감지하고
해석하는 표준 인터페이스를 정의합니다. 의도 기반 제어는 사용자의
자연스러운 상호작용을 가능하게 합니다.

## 2. Intent Sources

### 2.1 Supported Input Sources

| Source | Abbreviation | Latency | Accuracy | Invasiveness |
|--------|--------------|---------|----------|--------------|
| EMG (근전도) | EMG | 50-100ms | 85-95% | Non-invasive |
| Force Plate (힘판) | GRF | 10-30ms | 90-98% | Non-invasive |
| IMU (관성센서) | IMU | 20-50ms | 80-90% | Non-invasive |
| Button (버튼) | BTN | < 10ms | 100% | Non-invasive |
| BCI (뇌-컴퓨터) | BCI | 100-500ms | 70-85% | Varies |
| Voice (음성) | VOC | 200-500ms | 85-95% | Non-invasive |

### 2.2 Source Priority

다중 소스 사용 시 우선순위:

1. **Button** - 명시적 사용자 입력 (최우선)
2. **GRF** - 지면 반력 기반 (빠른 응답)
3. **EMG** - 근육 활성화 기반
4. **IMU** - 움직임 패턴 기반
5. **BCI** - 뇌파 기반 (보조적)

### 2.3 Sensor Fusion

```
Intent_final = Σ(wi × Intenti) / Σwi

where:
  wi = confidence_i × priority_i × availability_i
```

## 3. User Intents

### 3.1 Intent Enumeration

| Intent | Code | Description | Required Sources |
|--------|------|-------------|------------------|
| `stand_up` | SU | 앉은 자세에서 일어서기 | GRF, EMG |
| `sit_down` | SD | 선 자세에서 앉기 | GRF, EMG |
| `walk_forward` | WF | 전방 보행 시작 | GRF, IMU |
| `walk_backward` | WB | 후방 보행 시작 | GRF, IMU |
| `turn_left` | TL | 좌회전 | IMU, GRF |
| `turn_right` | TR | 우회전 | IMU, GRF |
| `stop` | ST | 정지 | GRF, EMG |
| `stair_ascend` | SA | 계단 오르기 | IMU, GRF |
| `stair_descend` | SB | 계단 내리기 | IMU, GRF |
| `step_over` | SO | 장애물 넘기 | IMU, EMG |
| `kick` | KK | 차기 동작 | EMG, IMU |
| `balance` | BL | 균형 유지 | IMU, GRF |

### 3.2 Intent State Machine

```
                    ┌─────────────┐
                    │   IDLE      │
                    └──────┬──────┘
                           │ intent detected
                    ┌──────▼──────┐
                    │  DETECTING  │
                    └──────┬──────┘
                           │ confidence > threshold
          ┌────────────────┼────────────────┐
          │                │                │
   ┌──────▼──────┐  ┌──────▼──────┐  ┌──────▼──────┐
   │  CONFIRMED  │  │  UNCERTAIN  │  │  REJECTED   │
   └──────┬──────┘  └──────┬──────┘  └──────┬──────┘
          │                │                │
          │ execute        │ wait           │ reset
   ┌──────▼──────┐         │         ┌──────▼──────┐
   │  EXECUTING  │◄────────┘         │    IDLE     │
   └──────┬──────┘                   └─────────────┘
          │ completed
   ┌──────▼──────┐
   │  COMPLETED  │
   └─────────────┘
```

## 4. EMG-Based Intent Detection

### 4.1 EMG Channels

| Channel | Muscle | Location | Primary Intent |
|---------|--------|----------|----------------|
| CH1 | Rectus Femoris | 대퇴직근 | Knee extension |
| CH2 | Biceps Femoris | 대퇴이두근 | Knee flexion |
| CH3 | Tibialis Anterior | 전경골근 | Ankle dorsiflexion |
| CH4 | Gastrocnemius | 비복근 | Ankle plantarflexion |
| CH5 | Gluteus Maximus | 대둔근 | Hip extension |
| CH6 | Iliopsoas | 장요근 | Hip flexion |

### 4.2 EMG Processing Pipeline

```
Raw EMG → Bandpass Filter → Rectification → Envelope → Normalization → Feature Extraction
          (20-450 Hz)       (Full-wave)    (RMS/MAV)   (MVC %)         (Intent)
```

### 4.3 EMG Features

| Feature | Abbreviation | Description |
|---------|--------------|-------------|
| Mean Absolute Value | MAV | 평균 절대값 |
| Root Mean Square | RMS | 제곱평균제곱근 |
| Waveform Length | WL | 파형 길이 |
| Zero Crossing | ZC | 영점 교차 횟수 |
| Slope Sign Changes | SSC | 기울기 부호 변화 |
| Integrated EMG | IEMG | 적분 근전도 |

### 4.4 EMG Intent Thresholds

| Intent | Primary Muscle | Threshold (% MVC) | Hold Time |
|--------|----------------|-------------------|-----------|
| Stand up | Rectus Femoris + Glut Max | > 30% | 200 ms |
| Sit down | Biceps Femoris | > 20% | 200 ms |
| Step | Tibialis Anterior | > 25% | 100 ms |

## 5. GRF-Based Intent Detection

### 5.1 GRF Parameters

| Parameter | Unit | Description |
|-----------|------|-------------|
| Fz | N | 수직 지면 반력 |
| Fx | N | 전후 방향 힘 |
| Fy | N | 좌우 방향 힘 |
| CoPx | m | 압력 중심 X |
| CoPy | m | 압력 중심 Y |

### 5.2 GRF Intent Patterns

| Intent | Pattern | Detection Logic |
|--------|---------|-----------------|
| Step initiation | CoP shift → Weight unload | CoPx shift > 3cm, Fz drop > 20% |
| Stand up | CoP forward → Fz increase | CoPx forward, Fz ramp up |
| Sit down | CoP backward → Fz decrease | CoPx backward, Fz ramp down |
| Turn | Asymmetric CoP | ΔCoPy > 5cm |

### 5.3 Gait Phase Detection from GRF

```
Heel Strike: Fz > 50N (rising edge)
Foot Flat:   Fz stable, CoP mid-foot
Mid-stance:  Fz peak, CoP forward
Heel Off:    CoP at forefoot
Toe Off:     Fz < 20N (falling edge)
Swing:       Fz ≈ 0N
```

## 6. IMU-Based Intent Detection

### 6.1 IMU Placement

| Location | Sensors | Primary Detection |
|----------|---------|-------------------|
| Pelvis | Acc, Gyro | Trunk inclination, turns |
| Thigh (L/R) | Acc, Gyro | Hip angle, leg swing |
| Shank (L/R) | Acc, Gyro | Knee angle, gait phase |
| Foot (L/R) | Acc, Gyro | Foot orientation, contact |

### 6.2 IMU Features

| Feature | Calculation | Intent Application |
|---------|-------------|-------------------|
| Tilt angle | atan2(ay, az) | Stand/Sit detection |
| Angular velocity | Gyro integration | Movement speed |
| Acceleration magnitude | √(ax²+ay²+az²) | Activity level |
| Jerk | d(acc)/dt | Movement initiation |

### 6.3 IMU Intent Patterns

| Intent | IMU Pattern | Threshold |
|--------|-------------|-----------|
| Walk forward | Pelvis pitch forward | > 5° |
| Walk backward | Pelvis pitch backward | > 5° |
| Turn left | Pelvis yaw left | > 10°/s |
| Turn right | Pelvis yaw right | > 10°/s |
| Stair ascend | Pelvis pitch up + high step | pitch > 10° |
| Stair descend | Pelvis pitch down + controlled step | pitch < -5° |

## 7. Intent Data Structure

### 7.1 IntentDetection

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `id` | string | Yes | 감지 이벤트 ID |
| `timestamp` | number | Yes | 감지 시간 (Unix ms) |
| `intent` | UserIntent | Yes | 감지된 의도 |
| `confidence` | number | Yes | 신뢰도 (0-1) |
| `source` | IntentSource | Yes | 주요 소스 |
| `allSources` | SourceResult[] | No | 모든 소스 결과 |
| `features` | FeatureVector | No | 추출된 특징 |

### 7.2 SourceResult

| Field | Type | Description |
|-------|------|-------------|
| `source` | IntentSource | 소스 유형 |
| `intent` | UserIntent | 감지된 의도 |
| `confidence` | number | 신뢰도 |
| `latency` | number | 처리 지연 (ms) |
| `rawData` | object | 원시 데이터 |

### 7.3 FeatureVector

| Field | Type | Description |
|-------|------|-------------|
| `emgFeatures` | EMGFeatures | EMG 특징 벡터 |
| `grfFeatures` | GRFFeatures | GRF 특징 벡터 |
| `imuFeatures` | IMUFeatures | IMU 특징 벡터 |

## 8. Intent Detection Configuration

### 8.1 DetectorConfig

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `enabledSources` | string[] | ['grf', 'imu'] | 활성 소스 목록 |
| `fusionMethod` | string | 'weighted' | 융합 방법 |
| `confidenceThreshold` | number | 0.7 | 최소 신뢰도 |
| `detectionWindow` | number | 200 | 감지 윈도우 (ms) |
| `debounceTime` | number | 500 | 디바운스 시간 (ms) |
| `maxLatency` | number | 300 | 최대 허용 지연 (ms) |

### 8.2 Source-Specific Config

```json
{
  "emg": {
    "sampleRate": 1000,
    "channels": [1, 2, 3, 4, 5, 6],
    "bandpass": { "low": 20, "high": 450 },
    "windowSize": 150,
    "overlapRatio": 0.5
  },
  "grf": {
    "sampleRate": 200,
    "forceThreshold": 20,
    "copSmoothing": 10
  },
  "imu": {
    "sampleRate": 200,
    "complementaryFilterAlpha": 0.98,
    "gyroDeadband": 0.5
  }
}
```

## 9. Machine Learning Models

### 9.1 Supported Models

| Model | Input | Output | Latency | Accuracy |
|-------|-------|--------|---------|----------|
| Threshold-based | Features | Binary | < 10ms | 75-85% |
| LDA | EMG features | Intent | < 20ms | 80-90% |
| SVM | Multi-source | Intent | < 30ms | 85-92% |
| Random Forest | Multi-source | Intent | < 40ms | 88-94% |
| LSTM | Time series | Intent | < 50ms | 90-96% |
| CNN-LSTM | Raw signals | Intent | < 100ms | 92-97% |

### 9.2 Model Interface

| Method | Input | Output | Description |
|--------|-------|--------|-------------|
| `predict` | FeatureVector | IntentPrediction | 단일 예측 |
| `predictProba` | FeatureVector | ProbabilityMap | 확률 분포 |
| `update` | TrainingData | void | 온라인 학습 |
| `calibrate` | CalibrationData | void | 사용자 캘리브레이션 |

### 9.3 Calibration Protocol

1. **Rest baseline** - 휴식 상태 기록 (10초)
2. **MVC collection** - 최대 수의적 수축 (각 근육 3회)
3. **Intent rehearsal** - 각 의도 동작 연습 (5회씩)
4. **Threshold tuning** - 개인별 임계값 조정
5. **Validation** - 검증 세트로 정확도 확인

## 10. Intent Examples

### 10.1 Walk Forward Detection

```json
{
  "id": "intent-001",
  "timestamp": 1702598400000,
  "intent": "walk_forward",
  "confidence": 0.92,
  "source": "grf",
  "allSources": [
    {
      "source": "grf",
      "intent": "walk_forward",
      "confidence": 0.95,
      "latency": 25
    },
    {
      "source": "imu",
      "intent": "walk_forward",
      "confidence": 0.88,
      "latency": 35
    },
    {
      "source": "emg",
      "intent": "walk_forward",
      "confidence": 0.82,
      "latency": 85
    }
  ],
  "features": {
    "grfFeatures": {
      "copShiftX": 4.5,
      "fzChange": -15.2,
      "asymmetry": 0.12
    },
    "imuFeatures": {
      "pelvisPitch": 6.2,
      "pelvisYaw": 0.5
    }
  }
}
```

### 10.2 Stand Up Detection

```json
{
  "id": "intent-002",
  "timestamp": 1702598400500,
  "intent": "stand_up",
  "confidence": 0.88,
  "source": "emg",
  "allSources": [
    {
      "source": "emg",
      "intent": "stand_up",
      "confidence": 0.90,
      "latency": 80
    },
    {
      "source": "grf",
      "intent": "stand_up",
      "confidence": 0.85,
      "latency": 20
    }
  ],
  "features": {
    "emgFeatures": {
      "rectusFemorris": 0.42,
      "gluteusMaximus": 0.38,
      "gastrocnemius": 0.25
    }
  }
}
```

## 11. Safety Considerations

### 11.1 False Positive Handling

| Scenario | Detection | Action |
|----------|-----------|--------|
| Unintended trigger | Rapid velocity check | Slow execution start |
| Conflicting intents | Priority resolution | Safest intent selected |
| Low confidence | Threshold check | Request confirmation |

### 11.2 Confirmation Modes

| Mode | Description | Use Case |
|------|-------------|----------|
| `auto` | 자동 실행 | 고신뢰도 감지 |
| `confirm_once` | 첫 감지 확인 | 새로운 동작 |
| `confirm_always` | 항상 확인 | 안전 최우선 |
| `manual_only` | 수동 입력만 | 평가 모드 |

## 12. Related Specifications

- [CONTROL-MODES.md](./CONTROL-MODES.md) - 제어 모드 명세
- [JOINT-STATE-SPEC.md](./JOINT-STATE-SPEC.md) - 관절 상태 데이터 명세
- [SESSION-DATA-SPEC.md](./SESSION-DATA-SPEC.md) - 세션 데이터 명세

## 13. Revision History

| Version | Date | Author | Description |
|---------|------|--------|-------------|
| 1.0.0 | 2025-12-14 | WIA | Initial specification |
