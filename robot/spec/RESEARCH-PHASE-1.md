# WIA Robot Standard - Phase 1 Research

## Version Information
- **Document Version**: 1.0.0
- **Last Updated**: 2025-01-15
- **Status**: Research Complete
- **Standard**: WIA-ROBOT-RES-001

---

## 1. Overview

본 문서는 WIA Robot 접근성 표준의 Phase 1 데이터 형식 정의를 위한 사전 조사 결과입니다.

### 1.1 Research Objectives

```
"외골격 로봇, 의수/의족, 재활 로봇, 돌봄 로봇, 수술 보조 로봇...
각각 다른 제조사, 다른 프로토콜, 다른 데이터 형식을 사용한다.

장애인이 다양한 보조 로봇을 함께 사용하려면?
재활 로봇의 데이터를 외골격에 전달하려면?
의수의 센서 데이터를 AI 모델과 연동하려면?

이걸 하나의 표준 형식으로 통일할 수 있을까?"
```

---

## 2. Assistive Robot Technology Survey

### 2.1 Exoskeleton (외골격 로봇)

#### ReWalk Robotics
- **Model**: ReWalk Personal 6.0
- **Target Users**: 하지 마비 환자 (T7-L5 척수 손상)
- **Data Characteristics**:
  - 관절 센서: 고관절, 무릎 (각도, 속도, 토크)
  - IMU: 가속도계, 자이로스코프
  - 발 압력 센서: 힐 스트라이크 감지
  - 배터리 상태: 전압, 잔량, 온도
- **Control Interface**: 목발 버튼 + 체중 이동 감지
- **Communication**: Bluetooth LE (모바일 앱 연동)

#### Ekso Bionics (EksoNR)
- **Model**: EksoNR (재활용)
- **Target Users**: 뇌졸중, 척수 손상 환자
- **Data Characteristics**:
  - 6 자유도 관절 데이터
  - SmartAssist: 환자별 적응형 보조
  - 보행 패턴 분석 데이터
  - 세션 기록 및 진척도
- **특징**: 클라우드 기반 데이터 분석 플랫폼

#### Hyundai H-MEX / H-WEX
- **Model**: H-MEX (의료), H-WEX (산업)
- **Data Characteristics**:
  - 고관절/무릎 4자유도
  - 보행 의도 감지 (무게 중심 이동)
  - 실시간 보행 분석

### 2.2 Prosthetics (의수/의족)

#### Open Bionics Hero Arm
- **Type**: 근전도(EMG) 의수
- **Data Characteristics**:
  - 2채널 EMG 센서
  - 손가락별 위치/압력 데이터
  - 그립 패턴 (14개 이상)
  - 진동 피드백 데이터
- **API**: Bluetooth LE, Mobile SDK 제공

#### Ottobock Genium / C-Leg
- **Type**: 마이크로프로세서 의족
- **Data Characteristics**:
  - 관절 각도 센서
  - 압력 센서 (발바닥)
  - 자이로스코프/가속도계
  - 적응형 보행 모드
- **특징**: 실시간 지형 적응

#### LUKE Arm (Mobius Bionics)
- **Type**: 고자유도 의수 (10 DOF)
- **Data Characteristics**:
  - 다중 EMG 채널
  - 촉각 피드백 센서
  - 관절별 위치/토크
  - 신경 인터페이스 지원

### 2.3 Rehabilitation Robot (재활 로봇)

#### Bionik InMotion ARM
- **Type**: 상지 재활 로봇
- **Data Characteristics**:
  - 2D/3D 작업 공간 좌표
  - 환자 힘/속도 측정
  - 운동 궤적 데이터
  - 세션별 성능 메트릭
- **Protocol**: 게임화된 치료 프로그램

#### Hocoma Lokomat
- **Type**: 하지 재활 (트레드밀 기반)
- **Data Characteristics**:
  - 양측 고관절/무릎 데이터
  - 체중 지지 비율
  - 보행 패턴 분석
  - Augmented Performance Feedback

#### Tyromotion Pablo/Amadeo
- **Type**: 손/손가락 재활
- **Data Characteristics**:
  - 개별 손가락 ROM
  - 악력 측정
  - 움직임 정확도
  - 인지 훈련 연동

### 2.4 Care Robot (돌봄 로봇)

#### SoftBank Pepper
- **Type**: 휴머노이드 돌봄 로봇
- **Data Characteristics**:
  - 음성 인식/합성
  - 얼굴 인식 및 감정 분석
  - 터치 센서 (머리, 손, 몸통)
  - 내비게이션 데이터
- **API**: NAOqi SDK, Python/C++ API

#### PARO Therapeutic Robot
- **Type**: 치료용 동물 로봇
- **Data Characteristics**:
  - 촉각 센서 (전신)
  - 청각 센서 (이름 인식)
  - 온도 센서
  - 상호작용 로그

#### KIST FURO / Hyundai DAL-e
- **Type**: 안내/서비스 로봇
- **Data Characteristics**:
  - 자율 주행 데이터
  - 음성 대화 로그
  - 서비스 요청/응답

### 2.5 Surgical Assistant (수술 보조 로봇)

#### Intuitive da Vinci
- **Type**: 최소 침습 수술 로봇
- **Data Characteristics**:
  - 4개 로봇 암 데이터
  - 3D HD 내시경 영상
  - 기구 위치/방향/힘
  - 수술자 콘솔 입력
- **특징**: 직관적 텔레오퍼레이션

#### Medtronic Hugo RAS
- **Type**: 로봇 수술 시스템
- **Data Characteristics**:
  - 모듈형 설계 데이터
  - 수술 중 영상 스트리밍
  - 기구 교환 로그

#### CMR Versius
- **Type**: 소형 수술 로봇
- **Data Characteristics**:
  - 독립 암 데이터
  - 햅틱 피드백
  - 클라우드 연동 분석

### 2.6 Mobility Aid (이동 보조 로봇)

#### Whill Model Ci2
- **Type**: 차세대 전동 휠체어
- **Data Characteristics**:
  - 옴니휠 속도/방향
  - 장애물 감지 센서
  - 배터리/주행 거리
  - 스마트폰 연동

#### Toyota HSR (Human Support Robot)
- **Type**: 생활 지원 로봇
- **Data Characteristics**:
  - 자율 내비게이션
  - 매니퓰레이터 데이터
  - 환경 인식 맵

#### ARGO ReWalk / Indego
- **Type**: 보행 보조 외골격
- **Data Characteristics**:
  - 휠체어 → 보행 전환
  - 안전 모니터링
  - 훈련 진척도

---

## 3. Existing Standards and Protocols

### 3.1 ROS (Robot Operating System)

#### Standard Message Types
```
sensor_msgs/JointState:
  - name: string[]
  - position: float64[]
  - velocity: float64[]
  - effort: float64[]

sensor_msgs/Imu:
  - orientation: geometry_msgs/Quaternion
  - angular_velocity: geometry_msgs/Vector3
  - linear_acceleration: geometry_msgs/Vector3

geometry_msgs/Pose:
  - position: Point (x, y, z)
  - orientation: Quaternion (x, y, z, w)
```

#### ROS 2 DDS
- **Transport**: Data Distribution Service
- **QoS**: Reliability, Durability, Deadline
- **IDL**: Interface Definition Language

### 3.2 ISO Standards

#### ISO 13482:2014
- **Title**: Safety for Personal Care Robots
- **Categories**:
  - Mobile Servant Robot
  - Physical Assistant Robot
  - Person Carrier Robot
- **Key Requirements**:
  - 위험 분석 및 평가
  - 비상 정지 기능
  - 안전 관련 제어 시스템

#### ISO 10218-1/2
- **Title**: Industrial Robot Safety
- **Relevance**: 재활 로봇 설계 참고

#### ISO 13485:2016
- **Title**: Medical Device QMS
- **Relevance**: 의료용 보조 로봇 품질 관리

#### IEC 62304
- **Title**: Medical Device Software Lifecycle
- **Classes**: A (No injury), B (Non-serious), C (Death/Serious)

### 3.3 IEEE Standards

#### IEEE P2751 (Draft)
- **Title**: 3D Map Data for Robot Navigation
- **Content**: 로봇 내비게이션용 3D 맵 포맷

#### IEEE 1872
- **Title**: Ontology for Robotics and Automation
- **Content**: 로봇 용어 및 개념 표준화

### 3.4 Other Protocols

#### URDF (Unified Robot Description Format)
```xml
<robot name="example">
  <link name="base_link">
    <visual>...</visual>
    <collision>...</collision>
    <inertial>...</inertial>
  </link>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <limit lower="-1.57" upper="1.57"/>
  </joint>
</robot>
```

#### SDF (Simulation Description Format)
- Gazebo 시뮬레이터 표준 포맷
- 물리 속성, 센서, 플러그인 정의

#### DICOM (Medical Imaging)
- 의료 영상 표준
- 수술 로봇 영상 연동 시 참고

---

## 4. Common Data Fields Analysis

### 4.1 All Robot Types - Common Fields

| Field | Description | Required |
|-------|-------------|----------|
| device_id | 고유 디바이스 식별자 | Yes |
| device_type | 로봇 유형 | Yes |
| manufacturer | 제조사 | Yes |
| model | 모델명 | Yes |
| firmware_version | 펌웨어 버전 | Yes |
| timestamp | ISO 8601 타임스탬프 | Yes |
| status | operational/standby/error | Yes |
| battery_percent | 배터리 잔량 | Yes |
| emergency_stop | 비상 정지 상태 | Yes |

### 4.2 Motion-Related Fields

| Robot Type | Motion Data |
|------------|-------------|
| Exoskeleton | Joint angles, velocities, torques |
| Prosthetics | Finger positions, grip force |
| Rehabilitation | Trajectory, ROM, patient effort |
| Care Robot | Navigation pose, following status |
| Surgical | Instrument position/orientation |
| Mobility Aid | Velocity (linear/angular), pose |

### 4.3 Sensor Data Fields

| Sensor Type | Data Format | Used By |
|-------------|-------------|---------|
| IMU | accel, gyro, orientation | Exo, Prosthetics, Mobility |
| EMG | channel, signal_mv, activation | Prosthetics |
| Force/Pressure | location, force_n | All |
| LiDAR | range, fov, obstacles | Mobility, Care |
| Camera | resolution, frame_rate | Surgical, Care, Mobility |
| Tactile | location, pressure, temperature | Prosthetics, Care |

### 4.4 Safety Fields

| Field | Description | Criticality |
|-------|-------------|-------------|
| emergency_stop | 비상 정지 활성화 여부 | Critical |
| fall_detection | 낙상 감지 | Critical |
| collision_avoidance | 충돌 회피 활성화 | High |
| force_limit_exceeded | 힘 한계 초과 | High |
| battery_critical | 배터리 위험 수준 | High |
| workspace_boundary_ok | 작업 영역 이탈 여부 | High |
| vital_signs_ok | 사용자 생체 신호 정상 | Medium |

---

## 5. Accessibility Requirements

### 5.1 User Customization

- 개인별 보조 수준 조정
- 입력 방식 선택 (조이스틱, 음성, 시선, 스위치)
- 피드백 모달리티 선택 (시각, 청각, 촉각)
- 속도/민감도 조정

### 5.2 Sensory Feedback

| Disability Type | Feedback Method |
|-----------------|-----------------|
| Visual Impairment | 청각 안내, 촉각 진동 |
| Hearing Impairment | 시각 표시, 진동 알림 |
| Motor Impairment | 적응형 입력, 음성 제어 |
| Cognitive Impairment | 단순화된 UI, 단계별 안내 |

### 5.3 WIA Device Integration

- **WIA-EXO**: 외골격 연동 (모션 데이터 공유)
- **WIA-SIGHT**: 바이오닉 아이 연동 (AR 오버레이)
- **WIA-LANG**: 음성-수어 번역 연동
- **WIA-MOBILITY**: 스마트 휠체어 연동

---

## 6. Conclusions and Recommendations

### 6.1 Standard Design Direction

1. **Hierarchical Structure**
   - Common base (모든 로봇 공통)
   - Robot-type specific (유형별 확장)
   - Custom extensions (제조사별 확장)

2. **ROS Compatibility**
   - ROS 메시지 타입과 매핑 가능한 구조
   - 필드명 및 단위 호환성

3. **Safety First**
   - 안전 관련 필드 필수화
   - 비상 정지 상태 항상 포함

4. **Extensibility**
   - JSON Schema additionalProperties 허용
   - 버전 관리 체계

### 6.2 Key Decisions

| Decision | Rationale |
|----------|-----------|
| JSON format | 웹 호환성, 가독성, 광범위한 지원 |
| ISO 8601 timestamps | 국제 표준, 시간대 지원 |
| SI units | 국제 단위계 (degrees, meters, Nm) |
| Snake_case naming | ROS 호환, Python 친화적 |

### 6.3 Next Steps

1. PHASE-1-DATA-FORMAT.md 작성
2. JSON Schema 정의
3. 예제 데이터 생성
4. 스키마 검증

---

## Document Information

- **Document ID**: WIA-ROBOT-RES-001
- **Classification**: Public Standard
- **License**: Open Standard (CC BY 4.0)

---

弘益人間 - Research for Accessible Robotics
