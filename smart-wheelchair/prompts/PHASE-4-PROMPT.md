# WIA Smart Wheelchair - Phase 4: Assistive Device Integration

## 목표
WIA 에코시스템 보조기기들과 스마트 휠체어를 통합합니다.

## 4.1 WIA Eye Gaze 연동

```typescript
interface EyeGazeWheelchairControl {
  // 시선 기반 방향 제어
  gazeToDirection: {
    enabled: boolean;
    sensitivity: number;       // 0.1 - 2.0
    deadzone: number;          // 화면 중앙 무시 영역
    dwellToConfirm: boolean;   // 응시로 확인
  };

  // 시선 기반 목적지 선택
  gazeToGoal: {
    enabled: boolean;
    mapDisplay: boolean;       // 화면에 지도 표시
    locationLabels: boolean;   // 위치 라벨 표시
  };

  // 시선 + 버튼 조합
  gazeWithSwitch: {
    gazeToSelect: boolean;     // 시선으로 선택
    switchToConfirm: boolean;  // 스위치로 확인
  };
}

// 시선 방향 → 휠체어 속도 매핑
function gazeToVelocity(
  gazeX: number,
  gazeY: number,
  config: GazeConfig
): Twist {
  // 화면 좌표 → 속도 명령 변환
  const linear = (gazeY - 0.5) * config.maxLinearSpeed * 2;
  const angular = -(gazeX - 0.5) * config.maxAngularSpeed * 2;

  return { linear, angular };
}
```

## 4.2 WIA BCI 연동

```typescript
interface BCIWheelchairControl {
  // BCI 신호 → 명령 매핑
  intentMapping: {
    forward: BCIPattern;       // 전진 패턴
    stop: BCIPattern;          // 정지 패턴
    left: BCIPattern;          // 좌회전 패턴
    right: BCIPattern;         // 우회전 패턴
    select: BCIPattern;        // 선택 패턴
  };

  // 안전 모드
  safetyMode: {
    requireConfirmation: boolean;  // 이중 확인 필요
    maxSpeed: number;              // BCI 제어 시 최대 속도
    autonomousAssist: boolean;     // 자율주행 보조
  };

  // 하이브리드 제어
  hybridControl: {
    bciForIntent: boolean;     // BCI로 의도 감지
    autonomousExecution: boolean; // 자율주행으로 실행
  };
}
```

## 4.3 음성 명령 (AAC 연동)

```typescript
interface VoiceWheelchairControl {
  // 음성 명령
  commands: {
    'go forward': () => setVelocity(0.5, 0);
    'stop': () => emergencyStop();
    'turn left': () => setVelocity(0, 0.5);
    'turn right': () => setVelocity(0, -0.5);
    'go to kitchen': () => navigateTo('kitchen');
    'go to bathroom': () => navigateTo('bathroom');
    'come here': () => navigateToUser();
  };

  // AAC 연동
  aacIntegration: {
    symbolToCommand: Map<string, WheelchairCommand>;
    voiceOutputFeedback: boolean;  // 음성 피드백
  };

  // 자연어 처리
  nlpEnabled: boolean;
  customCommands: Map<string, WheelchairCommand>;
}
```

## 4.4 외골격 연동 (환승)

```typescript
interface ExoskeletonTransition {
  // 환승 시퀀스
  transitionToExo: {
    // 1. 휠체어 위치 조정
    alignWithExo(): Promise<void>;
    // 2. 팔걸이 열기
    openArmrests(): Promise<void>;
    // 3. 안전 확인
    safetyCheck(): Promise<boolean>;
    // 4. 외골격 활성화 신호
    signalExoReady(): void;
  };

  transitionFromExo: {
    // 1. 휠체어 위치 확인
    confirmPosition(): Promise<boolean>;
    // 2. 착석 보조
    assistSeating(): Promise<void>;
    // 3. 팔걸이 닫기
    closeArmrests(): Promise<void>;
    // 4. 휠체어 활성화
    activateWheelchair(): void;
  };

  // 조율된 이동
  coordinatedMobility: {
    exoWalking: boolean;
    wheelchairFollowing: boolean;
    seamlessHandoff: boolean;
  };
}
```

## 4.5 스마트홈 연동

```typescript
interface SmartHomeIntegration {
  // Matter/Thread 프로토콜
  matter: {
    discoverDevices(): SmartDevice[];
    controlDevice(deviceId: string, action: string): void;
  };

  // 접근성 자동화
  accessibilityAutomation: {
    autoOpenDoors: boolean;        // 문 자동 열기
    autoCallElevator: boolean;     // 엘리베이터 자동 호출
    autoTurnOnLights: boolean;     // 조명 자동 켜기
    pathLighting: boolean;         // 이동 경로 조명
  };

  // 위치 기반 자동화
  locationTriggers: Map<string, SmartHomeAction[]>;

  // 음성 비서 연동
  voiceAssistant: {
    alexa: boolean;
    googleHome: boolean;
    siri: boolean;
  };
}
```

---

## 산출물

```
smart-wheelchair/
├── integrations/
│   ├── eye-gaze/
│   │   └── gaze_controller.py
│   ├── bci/
│   │   └── bci_controller.py
│   ├── voice/
│   │   └── voice_controller.py
│   ├── exoskeleton/
│   │   └── transition_manager.py
│   └── smarthome/
│       └── matter_bridge.py
├── ros2_ws/src/
│   └── wia_wheelchair_interfaces/
│       └── nodes/
├── examples/
│   ├── eye-gaze-control/
│   ├── bci-control/
│   └── voice-control/
└── docs/
    ├── EYE-GAZE-GUIDE.md
    ├── BCI-GUIDE.md
    └── VOICE-COMMAND-GUIDE.md
```

---

## 완료 체크리스트

- [ ] 통신 프로토콜 (Phase 1)
- [ ] 센서 인터페이스 (Phase 2)
- [ ] 자율주행 (Phase 3)
- [ ] Eye Gaze 연동 (Phase 4)
- [ ] BCI 연동 (Phase 4)
- [ ] 음성 명령 (Phase 4)
- [ ] 외골격 환승 (Phase 4)
- [ ] 스마트홈 연동 (Phase 4)

## 홍익인간

이동의 자유를. 모든 사람에게. 어떤 방식으로든.
