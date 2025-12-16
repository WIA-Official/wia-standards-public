# Phase 4 사전 조사 결과

**WIA Robot Standard - Ecosystem Integration Research**

---

## 1. 로봇 시각화 도구 비교

### RViz2 (ROS2 Visualization)

**개요**
- ROS2용 3D 시각화 도구
- Marker 메시지를 통한 프로그래밍 방식 시각화
- visualization_msgs/msg/Marker 및 MarkerArray 지원

**Marker 메시지 타입**
| 타입 | 설명 |
|------|------|
| ARROW | 화살표 (방향 표시) |
| CUBE | 큐브 |
| SPHERE | 구체 |
| CYLINDER | 원통 |
| LINE_STRIP | 연속 선 |
| LINE_LIST | 선 목록 |
| CUBE_LIST | 큐브 목록 (배치 렌더링) |
| SPHERE_LIST | 구체 목록 |
| POINTS | 점 클라우드 |
| TEXT_VIEW_FACING | 텍스트 |
| MESH_RESOURCE | 메쉬 파일 |

**주요 속성**
- `namespace`: 마커 그룹 식별
- `id`: 마커 고유 ID
- `scale`: 크기 (1.0 = 1m)
- `color`: RGBA (0.0 ~ 1.0)
- `lifetime`: 표시 지속 시간
- `frame_id`: 좌표 프레임

**WIA Robot 적용**
- ✅ 외골격 관절 위치 실시간 시각화
- ✅ 재활 로봇 궤적 표시
- ✅ 수술 로봇 작업 공간 경계 표시
- ✅ 안전 영역 시각화
- ⚠️ 리소스 집약적 (임베디드 시스템 주의)

**참고**
- [RViz User Guide](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html)
- [Marker Display Types](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/Marker-Display-types/Marker-Display-types.html)

---

### Gazebo Simulation

**개요**
- ROS2 통합 로봇 시뮬레이터
- 물리 엔진 기반 시뮬레이션
- SDF (Simulation Description Format) 네이티브 지원

**URDF vs SDF**
| 특성 | URDF | SDF |
|------|------|-----|
| 범위 | 로봇만 | 월드 + 로봇 |
| 루프 조인트 | ❌ | ✅ |
| 마찰 속성 | 제한적 | 완전 지원 |
| 센서 정의 | 제한적 | 완전 지원 |
| 월드 배치 | ❌ | ✅ |

**변환 방식**
```
URDF → libsdformat → SDF (내부 변환)
```

**최신 동향 (2024)**
- sdformat_urdf 패키지로 SDF 직접 사용 가능
- URDF와 SDF 이중 관리 불필요
- Fixed joint 링크 자동 병합

**WIA Robot 적용**
- ✅ 재활 로봇 물리 시뮬레이션
- ✅ 외골격 보행 시뮬레이션
- ✅ 의수/의족 그립 시뮬레이션
- ✅ 안전 테스트 환경
- ⚠️ 각 링크에 inertial 태그 필요

**참고**
- [Gazebo URDF Tutorial](https://classic.gazebosim.org/tutorials/?tut=ros_urdf)
- [Gazebo ROS2 Interoperability](https://gazebosim.org/docs/latest/ros2_interop/)

---

### Unity Robotics Hub

**개요**
- Unity Technologies 공식 로봇 시뮬레이션 도구
- ROS/ROS2 통합 지원
- 고품질 렌더링 및 물리 엔진

**ROS2 통합 아키텍처**
```
Unity ←──TCP──→ ROS-TCP-Endpoint ←──→ ROS2 네트워크
```

**주요 컴포넌트**
- **MessageGeneration**: .msg → C# 클래스 생성
- **ROSConnection**: Unity 스크립트로 Pub/Sub/Service 호출
- **ros_tcp_connector**: Unity-ROS 통신

**기능**
- Navigation 2 연동
- slam_toolbox 통합
- Turtlebot3 시뮬레이션 예제

**제한사항**
- TCP 기반 통신 (QoS 제한)
- 3D LiDAR, RGB-D 카메라 기본 미제공
- 활발한 개발 중 (API 변경 가능)

**WIA Robot 적용**
- ✅ VR/AR 재활 훈련 환경
- ✅ 고품질 시각화
- ✅ 게임 엔진 기반 인터랙션
- ⚠️ ROS2 QoS 완전 지원 안됨

**참고**
- [Unity Robotics Hub GitHub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- [Unity ROS2 Blog](https://blog.unity.com/engine-platform/advance-your-robot-autonomy-with-ros-2-and-unity)

---

## 2. 의료 데이터 통합

### HL7 FHIR DeviceMetric

**개요**
- 의료 기기 측정 특성을 표현하는 FHIR 리소스
- 직접/파생, 정량/정성 측정 모델링
- ISO/IEEE 11073 표준과 호환

**리소스 계층 구조**
```
Observation → DeviceMetric → Device(Channel) → Device(VMD) → Device
```

**DeviceMetric 주요 속성**
| 속성 | 설명 |
|------|------|
| operationalStatus | on, off, standby, entered-in-error |
| category | measurement, setting, calculation |
| color | 표시 색상 코드 |
| calibration | 보정 정보 |

**Observation 연동**
```
Observation.subject = Patient
Observation.device = Device 또는 DeviceMetric
```

**FHIR 리소스 분류**
1. **정의/기능**: Device, DeviceDefinition, DeviceMetric
2. **사용/주문**: DeviceAssociation, DeviceRequest, DeviceDispense
3. **실제 동작**: Observation, DeviceAlert, Procedure

**WIA Robot 적용**
- ✅ 재활 로봇 ROM 측정 → Observation
- ✅ 바이탈 모니터링 → DeviceMetric
- ✅ EMR 시스템 연동
- ✅ 치료 기록 자동화
- ⚠️ 의료 규정 준수 필요 (HIPAA, GDPR)

**참고**
- [FHIR DeviceMetric R4](http://hl7.org/fhir/R4/devicemetric.html)
- [Devices on FHIR](https://confluence.hl7.org/spaces/DOF/pages/184923592/Navigating+Device+Informatics+in+FHIR)

---

## 3. AI/ML 연동

### TensorFlow/PyTorch 데이터셋

**시계열 데이터 처리**
- TensorFlow 공식 time_series 튜토리얼
- LSTM/Transformer 기반 예측 모델
- 윈도우 기반 데이터 분할

**EMG 신호 분류 데이터셋**

| 데이터셋 | 특징 |
|---------|------|
| EMG_SignalClassification | 3클래스 분류, TensorFlow Lite |
| EMG-Robot-Dataset | IMU + EMG, 의수 제어용 |
| Nature EMG Dataset (2024) | 8명, 6 제스처, 다중 암 위치 |
| Multi-channel sEMG | 40명, 10 제스처, 4채널 |

**EMG 데이터 구조**
```python
# 일반적인 EMG 학습 데이터 형식
{
    "timestamp": 1702483200,
    "channels": [emg_0, emg_1, emg_2, emg_3],
    "imu_accel": [ax, ay, az],
    "imu_gyro": [gx, gy, gz],
    "label": "grip"  # or "open", "point", etc.
}
```

**WIA Robot 적용**
- ✅ 의수 EMG 제스처 인식 모델 학습
- ✅ 재활 ROM 예측
- ✅ 보행 패턴 분류
- ✅ 낙상 예측 모델
- ⚠️ 충분한 학습 데이터 필요

**참고**
- [TensorFlow Time Series](https://www.tensorflow.org/tutorials/structured_data/time_series)
- [Nature EMG Dataset 2024](https://www.nature.com/articles/s41597-024-04296-8)

---

## 4. 결론 및 권장 사항

### 권장 시각화 방식

| 사용 사례 | 권장 도구 | 이유 |
|----------|----------|------|
| ROS2 로봇 상태 | RViz2 | 네이티브 통합, 실시간 |
| 물리 시뮬레이션 | Gazebo | 정확한 물리 엔진 |
| VR/AR 훈련 | Unity | 고품질 렌더링 |
| 웹 대시보드 | Three.js/WebGL | 브라우저 호환 |

### 의료 데이터 매핑

```
WIA Robot Data → HL7 FHIR Mapping
─────────────────────────────────
ExoskeletonSpec.joints → Observation (joint angles)
RehabilitationSpec.performance → Observation (ROM)
CareRobotSpec.vital_signs → Observation (vitals)
ProstheticSpec.emg_sensors → DeviceMetric (EMG)
SafetyStatus → DeviceAlert (safety events)
```

### AI 학습 데이터 형식

**권장 출력 형식**
1. **JSON Lines (.jsonl)**: 스트리밍 처리 용이
2. **CSV**: Pandas 호환, 범용
3. **TFRecord**: TensorFlow 최적화
4. **Parquet**: 대용량 데이터

**특징 벡터 구성**
```
EMG: [signal_mv × n_channels, activation_level × n_channels]
IMU: [accel_xyz, gyro_xyz, orientation_quat]
Joint: [angle, velocity, torque] × n_joints
```

### 출력 계층 아키텍처

```
┌────────────────────────────────────────────────────┐
│              WIA Robot Data (Phase 1-3)            │
└───────────────────────┬────────────────────────────┘
                        │
                        ▼
┌────────────────────────────────────────────────────┐
│              OutputManager (Phase 4)               │
├────────────┬────────────┬────────────┬────────────┤
│Visualization│  Medical  │   AI/ML   │  Export    │
├────────────┼────────────┼────────────┼────────────┤
│ RViz2      │ HL7 FHIR   │ TensorFlow │ JSON      │
│ Gazebo     │ DICOM      │ PyTorch    │ CSV       │
│ Unity      │ OpenEHR    │ ONNX       │ URDF/SDF  │
│ WebGL      │            │            │ ROS Bag   │
└────────────┴────────────┴────────────┴────────────┘
```

---

## 참고문헌

1. [ROS2 RViz Documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/)
2. [Gazebo Simulation](https://gazebosim.org/docs/)
3. [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
4. [HL7 FHIR DeviceMetric](http://hl7.org/fhir/R4/devicemetric.html)
5. [TensorFlow Time Series](https://www.tensorflow.org/tutorials/structured_data/time_series)
6. [Nature EMG Dataset 2024](https://www.nature.com/articles/s41597-024-04296-8)

---

*연구 완료일: 2025-01-15*
*WIA Robot Standard Phase 4*
