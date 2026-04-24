# WIA Autonomous Vehicle Accessibility - Phase 4: Ecosystem Integration

## 목표
WIA 에코시스템 보조기기들과 자율주행 차량을 통합합니다.

## 4.1 WIA Smart Wheelchair 연동

```typescript
interface WheelchairVehicleIntegration {
  // 도킹 시스템
  docking: {
    autoDetect: boolean;           // 휠체어 자동 감지
    guidanceSystem: boolean;       // 도킹 안내 시스템
    securementSequence: string[];  // 고정 시퀀스
  };

  // 실시간 상태 공유
  statusSharing: {
    wheelchairBattery: boolean;    // 배터리 상태
    passengerVitals: boolean;      // 승객 상태
    securementStatus: boolean;     // 고정 상태
  };

  // 협조 경로 계획
  coordinatedNavigation: {
    wheelchairToVehicle: boolean;  // 휠체어→차량 이동
    vehicleToDestination: boolean; // 차량→목적지
    lastMileHandoff: boolean;      // 최종 구간 인계
  };
}

// 도킹 프로토콜
async function dockWheelchair(
  wheelchair: WheelchairInfo,
  vehicle: VehicleCapabilities
): Promise<DockingResult> {
  // 1. 램프/리프트 배치
  await vehicle.deployEntry(wheelchair.entryPreference);
  // 2. 휠체어 진입 안내
  await vehicle.guideEntry(wheelchair.id);
  // 3. 위치 확인 및 고정
  await vehicle.secureWheelchair(wheelchair);
  // 4. 고정 검증
  return vehicle.verifySecurement();
}
```

## 4.2 WIA Eye Gaze 연동

```typescript
interface EyeGazeVehicleControl {
  // 차량 내 시선 인터페이스
  inVehicleGaze: {
    destinationSelection: boolean; // 목적지 선택
    hvacControl: boolean;          // 공조 제어
    entertainmentControl: boolean; // 엔터테인먼트
    emergencyTrigger: boolean;     // 비상 트리거
  };

  // 응시 확인 설정
  dwellConfig: {
    dwellTime: number;             // 확인 시간
    visualFeedback: boolean;       // 시각적 피드백
    audioFeedback: boolean;        // 청각 피드백
  };

  // 시선 기반 승객 모니터링
  passengerMonitoring: {
    alertnessDetection: boolean;   // 각성 상태 감지
    distressDetection: boolean;    // 고통 감지
    needsAssistance: boolean;      // 도움 필요 감지
  };
}
```

## 4.3 WIA BCI 연동

```typescript
interface BCIVehicleInterface {
  // 비상 BCI 명령
  emergencyCommands: {
    stopVehicle: BCIPattern;       // 차량 정지
    callHelp: BCIPattern;          // 도움 요청
    openDoor: BCIPattern;          // 문 열기
  };

  // 기본 제어
  basicControl: {
    confirmDestination: BCIPattern;
    cancelTrip: BCIPattern;
    adjustTemperature: BCIPattern;
  };

  // 안전 설정
  safetyConfig: {
    requiredConfidence: number;    // 최소 신뢰도
    multipleConfirmation: boolean; // 다중 확인
    voiceFallback: boolean;        // 음성 대체
  };
}
```

## 4.4 WIA AAC 연동

```typescript
interface AACVehicleIntegration {
  // 음성 명령
  voiceCommands: {
    '목적지 변경': () => requestNewDestination();
    '정차해 주세요': () => requestPullOver();
    '온도 올려주세요': () => adjustTemperature(+2);
    '창문 열어주세요': () => openWindow();
    '도움이 필요해요': () => requestAssistance();
  };

  // AAC 심볼 지원
  symbolSupport: {
    displaySymbols: boolean;       // 심볼 표시
    symbolToAction: Map<string, VehicleAction>;
  };

  // TTS 피드백
  ttsConfig: {
    announceStops: boolean;        // 정류장 안내
    announceEta: boolean;          // 도착 예정 안내
    emergencyAlerts: boolean;      // 비상 알림
  };
}
```

## 4.5 대중교통/플릿 연동

```typescript
interface FleetIntegration {
  // MaaS 연동
  maas: {
    gtfsRealtimeSupport: boolean;  // GTFS-RT 지원
    transactionApi: boolean;       // 결제 API
    multimodalRouting: boolean;    // 다중 모드 경로
  };

  // 플릿 관리
  fleetManagement: {
    vehiclePooling: boolean;       // 차량 풀링
    demandPrediction: boolean;     // 수요 예측
    accessibilityRouting: boolean; // 접근성 우선 경로
  };

  // 환승 지원
  transferSupport: {
    wheelchairReservation: boolean;// 휠체어 공간 예약
    assistanceRequest: boolean;    // 도움 요청
    seamlessTransfer: boolean;     // 원활한 환승
  };
}
```

## 4.6 CLI 도구

```bash
# 프로필 관리
wia-auto profile create --name "홍길동" --wheelchair power
wia-auto profile list
wia-auto profile export my-profile.json

# 차량 조회
wia-auto vehicle find --wheelchair --ramp --location "서울역"
wia-auto vehicle status <vehicle-id>
wia-auto vehicle capabilities <vehicle-id>

# 이동 요청
wia-auto trip request --from "현재위치" --to "강남역"
wia-auto trip status <trip-id>
wia-auto trip cancel <trip-id>

# 시뮬레이션
wia-auto sim start --scenario wheelchair-boarding
wia-auto sim vehicle add --accessible
wia-auto sim trip simulate
```

---

## 산출물

```
auto/
├── prompts/
│   └── PHASE-4-INTEGRATION-PROMPT.md
├── spec/
│   └── PHASE-4-INTEGRATION.md
├── api/rust/src/
│   ├── integrations/
│   │   ├── mod.rs
│   │   ├── wheelchair.rs       # 스마트휠체어 연동
│   │   ├── eye_gaze.rs         # Eye Gaze 연동
│   │   ├── bci.rs              # BCI 연동
│   │   ├── aac.rs              # AAC 연동
│   │   └── fleet.rs            # 플릿/MaaS 연동
│   └── cli/
│       ├── mod.rs
│       └── commands.rs         # CLI 명령어
└── examples/
    ├── wheelchair_docking.rs
    ├── multimodal_trip.rs
    └── fleet_integration.rs
```

---

## 완료 체크리스트

- [ ] 데이터 포맷 (Phase 1)
- [ ] Rust API (Phase 2)
- [ ] 프로토콜 명세 (Phase 3)
- [ ] Smart Wheelchair 연동 (Phase 4)
- [ ] Eye Gaze 연동 (Phase 4)
- [ ] BCI 연동 (Phase 4)
- [ ] AAC 연동 (Phase 4)
- [ ] Fleet/MaaS 연동 (Phase 4)
- [ ] CLI 도구 (Phase 4)

## 弘益人間

이동의 자유를. 모든 사람에게. 어디서든.
