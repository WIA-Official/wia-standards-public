# WIA Smart Wheelchair Standard - Master Prompt

## 프로젝트 개요

**WIA Smart Wheelchair Protocol**은 스마트 휠체어 모듈 간 통신 및 자율주행 표준입니다.

### 핵심 문제
- 제조사마다 **독점 통신 시스템** 사용
- 모듈 상호운용 보장하는 **오픈 표준 없음**
- [CAN 버스](https://pmc.ncbi.nlm.nih.gov/articles/PMC10928074/) 사용하지만 메시지 포맷 다름
- ROS 미들웨어 있지만 휠체어 특화 표준 없음

### WIA 솔루션

| Phase | 내용 | 산출물 |
|-------|------|--------|
| 1 | 통신 프로토콜 | CAN/ROS 메시지 표준 |
| 2 | 센서 인터페이스 | LiDAR, 카메라, IMU 통합 |
| 3 | 자율주행 | 경로 계획, 장애물 회피 |
| 4 | 보조기기 연동 | Eye Gaze, BCI, AAC 통합 |

---

## 핵심 기술

```typescript
interface SmartWheelchair {
  // 모터 제어
  motors: {
    left: MotorController;
    right: MotorController;
  };

  // 센서
  sensors: {
    lidar?: LiDARSensor;
    camera?: Camera[];
    imu: IMUSensor;
    encoders: Encoder[];
    ultrasonic?: UltrasonicSensor[];
  };

  // 내비게이션
  navigation: {
    localization: Localization;
    pathPlanning: PathPlanner;
    obstacleAvoidance: ObstacleAvoidance;
  };

  // 사용자 인터페이스
  userInterface: {
    joystick: JoystickInput;
    headArray?: HeadArrayInput;
    sipAndPuff?: SipAndPuffInput;
    eyeGaze?: EyeGazeInput;
    bci?: BCIInput;
  };
}
```

---

## 4-Phase 요약

### Phase 1: 통신 프로토콜
- CAN 버스 메시지 ID/포맷 표준
- ROS2 토픽/서비스 정의
- WebSocket/MQTT IoT 연동

### Phase 2: 센서 인터페이스
- LiDAR 포인트 클라우드 포맷
- 카메라 스트림 표준
- 센서 퓨전 프레임워크

### Phase 3: 자율주행
- SLAM (동시 위치추정 및 지도작성)
- 경로 계획 알고리즘
- 장애물 감지/회피

### Phase 4: 보조기기 연동
- WIA Eye Gaze 연동
- WIA BCI 연동
- 음성 명령 (AAC)
- 외골격 연동 (환승)

---

## 참조 리소스

- [ANSI/RESNA 휠체어 표준](https://www.resna.org/)
- [ROS Navigation Stack](https://navigation.ros.org/)
- [ISO 7176 휠체어 시리즈](https://www.iso.org/standard/70180.html)

---

## 철학

**홍익인간 (弘益人間)** - 널리 인간을 이롭게 하라

이동의 자유를 모든 사람에게.
