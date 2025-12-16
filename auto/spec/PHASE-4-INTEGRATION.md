# WIA Autonomous Vehicle Accessibility - Phase 4: Ecosystem Integration Specification

## 1. Overview

이 문서는 WIA 자율주행 차량의 에코시스템 통합 명세를 정의합니다.
스마트 휠체어, Eye Gaze, BCI, AAC, 대중교통 시스템과의 연동을 포함합니다.

### 1.1 Integration Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                     WIA Auto Integration Hub                              │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐      │
│  │Wheelchair│ │ Eye Gaze │ │   BCI    │ │   AAC    │ │  Fleet   │      │
│  │   Dock   │ │ Control  │ │Interface │ │  Voice   │ │  MaaS    │      │
│  └────┬─────┘ └────┬─────┘ └────┬─────┘ └────┬─────┘ └────┬─────┘      │
│       │            │            │            │            │              │
│       └────────────┴─────┬──────┴────────────┴────────────┘              │
│                          │                                                │
│                   ┌──────▼──────┐                                        │
│                   │  Command    │                                        │
│                   │  Arbiter    │                                        │
│                   └──────┬──────┘                                        │
│                          │                                                │
│            ┌─────────────┼─────────────┐                                │
│            │             │             │                                │
│       ┌────▼────┐  ┌─────▼─────┐  ┌────▼────┐                          │
│       │ Safety  │  │  Vehicle  │  │  HMI    │                          │
│       │ Monitor │  │  Control  │  │ Manager │                          │
│       └─────────┘  └───────────┘  └─────────┘                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### 1.2 Integration Principles

| Principle | Description |
|-----------|-------------|
| Safety First | 모든 통합에서 안전이 최우선 |
| Graceful Degradation | 연결 실패 시 기본 기능 유지 |
| Universal Access | 다양한 입력 방식 지원 |
| Privacy by Design | 개인정보 최소 수집 |

---

## 2. Smart Wheelchair Integration

### 2.1 Docking System

#### 2.1.1 Detection Phase

```yaml
detection:
  methods:
    - bluetooth_beacon        # BLE 비콘
    - uwb_positioning         # UWB 정밀 위치
    - visual_marker           # 시각 마커
    - nfc_tag                 # NFC 태그

  range:
    discovery: 50m            # 발견 범위
    approach: 10m             # 접근 안내 시작
    docking: 2m               # 도킹 모드 시작

  wheelchair_profile:
    dimensions:
      width: float            # cm
      length: float           # cm
      height: float           # cm
    type: manual | power | scooter
    weight: float             # kg
    turning_radius: float     # cm
```

#### 2.1.2 Docking Sequence

```
┌──────────────────────────────────────────────────────────────┐
│                     DOCKING SEQUENCE                          │
├──────────────────────────────────────────────────────────────┤
│                                                               │
│  1. APPROACH                                                  │
│     ├── Vehicle stops at designated zone                      │
│     ├── Deploy ramp/lift                                      │
│     └── Enable guidance system                                │
│                                                               │
│  2. ENTRY                                                     │
│     ├── Wheelchair detected at entry                          │
│     ├── Visual/audio guidance active                          │
│     ├── Speed limited to 0.5 m/s                              │
│     └── Obstacle detection active                             │
│                                                               │
│  3. POSITIONING                                               │
│     ├── Guide to securement zone                              │
│     ├── Fine position adjustment                              │
│     └── Confirm optimal position                              │
│                                                               │
│  4. SECUREMENT                                                │
│     ├── Deploy front/rear tie-downs                           │
│     ├── Apply tension (15-25 lbs)                             │
│     ├── Deploy wheel locks                                    │
│     └── Verify all points secured                             │
│                                                               │
│  5. VERIFICATION                                              │
│     ├── Load cell verification                                │
│     ├── Visual confirmation request                           │
│     ├── Passenger comfort check                               │
│     └── Ready signal to vehicle                               │
│                                                               │
└──────────────────────────────────────────────────────────────┘
```

#### 2.1.3 Communication Protocol

```typescript
// WebSocket Events for Wheelchair Integration
enum WheelchairEvent {
  WHEELCHAIR_DETECTED = 'wheelchair.detected',
  APPROACH_STARTED = 'wheelchair.approach.started',
  ENTRY_STARTED = 'wheelchair.entry.started',
  POSITION_CONFIRMED = 'wheelchair.position.confirmed',
  SECUREMENT_STARTED = 'wheelchair.securement.started',
  SECUREMENT_COMPLETE = 'wheelchair.securement.complete',
  SECUREMENT_FAILED = 'wheelchair.securement.failed',
  EXIT_REQUESTED = 'wheelchair.exit.requested',
  EXIT_COMPLETE = 'wheelchair.exit.complete',
}

interface WheelchairDockingPayload {
  wheelchair_id: string;
  vehicle_id: string;
  phase: 'approach' | 'entry' | 'positioning' | 'securement' | 'verified';
  progress_percent: number;
  estimated_time_remaining: number;  // seconds
  guidance?: {
    direction: 'forward' | 'back' | 'left' | 'right' | 'stop';
    distance_cm: number;
    audio_cue: string;
  };
  securement_points?: SecurementPoint[];
}
```

### 2.2 Last-Mile Coordination

```yaml
last_mile_handoff:
  # 차량 → 휠체어 인계
  vehicle_to_wheelchair:
    arrival_notification: true
    door_auto_open: true
    ramp_auto_deploy: true
    securement_auto_release: true
    exit_guidance: true

  # 목적지 정보 전송
  destination_handoff:
    building_name: string
    entrance_type: accessible | main | side
    indoor_navigation: boolean
    contact_info: optional

  # 환경 정보
  environment_data:
    surface_type: paved | gravel | grass
    slope_degrees: float
    obstacles: Obstacle[]
    weather_advisory: optional
```

---

## 3. Eye Gaze Integration

### 3.1 In-Vehicle Display Zones

```
┌─────────────────────────────────────────────────────────────┐
│                    VEHICLE DISPLAY                           │
├───────────────┬─────────────────────────┬───────────────────┤
│               │                         │                   │
│   EMERGENCY   │     MAIN CONTENT        │    QUICK          │
│   ZONE        │     - Map               │    ACTIONS        │
│   - Stop      │     - Destination       │    - Temperature  │
│   - Help      │     - ETA               │    - Window       │
│               │     - Route options     │    - Entertainment│
│               │                         │                   │
├───────────────┴─────────────────────────┴───────────────────┤
│                      STATUS BAR                              │
│   Battery | Connection | Securement | Accessibility Mode    │
└─────────────────────────────────────────────────────────────┘
```

### 3.2 Dwell Selection Configuration

```yaml
gaze_config:
  dwell_time:
    emergency: 0.5s        # 비상 버튼 (짧음)
    standard: 1.0s         # 일반 버튼
    critical: 1.5s         # 중요 결정 (취소 등)

  feedback:
    visual:
      progress_ring: true
      highlight_color: '#FFD700'
      confirmation_animation: true
    audio:
      progress_tone: true
      confirmation_sound: true
    haptic:
      seat_vibration: optional

  calibration:
    auto_calibrate: true
    recalibrate_interval: 30min
    head_tracking_assist: true

  accessibility_modes:
    high_contrast: boolean
    large_targets: boolean
    extended_dwell: boolean
```

### 3.3 Passenger Monitoring

```typescript
interface GazeMonitoring {
  // 각성 상태 감지
  alertness: {
    eyes_open: boolean;
    blink_rate: number;           // per minute
    gaze_stability: number;       // 0-1
    alertness_score: number;      // 0-100
  };

  // 불편/고통 감지
  distress_detection: {
    rapid_eye_movement: boolean;
    prolonged_eye_closure: boolean;
    erratic_gaze_pattern: boolean;
    distress_score: number;       // 0-100
  };

  // 도움 필요 감지
  assistance_needed: {
    looking_at_help_zone: boolean;
    dwell_on_emergency: boolean;
    gaze_towards_door: boolean;
  };

  // 임계값
  thresholds: {
    alertness_warning: 30;
    distress_alert: 70;
    assistance_trigger: 2.0;      // seconds on help zone
  };
}
```

---

## 4. BCI Integration

### 4.1 Safety-Critical Commands

```yaml
bci_commands:
  emergency:
    stop_vehicle:
      pattern: motor_imagery_both_hands
      confidence_threshold: 0.85
      confirmation_required: false    # 즉시 실행
      cooldown: 0s

    call_help:
      pattern: p300_emergency_symbol
      confidence_threshold: 0.80
      confirmation_required: true     # 1초 유지
      cooldown: 5s

  navigation:
    confirm_destination:
      pattern: motor_imagery_right
      confidence_threshold: 0.75
      confirmation_required: true
      cooldown: 2s

    cancel_trip:
      pattern: motor_imagery_left
      confidence_threshold: 0.80
      confirmation_required: true
      cooldown: 3s

  comfort:
    temperature_up:
      pattern: ssvep_8hz
      confidence_threshold: 0.70
      confirmation_required: false
      cooldown: 10s

    temperature_down:
      pattern: ssvep_10hz
      confidence_threshold: 0.70
      confirmation_required: false
      cooldown: 10s
```

### 4.2 Signal Processing Pipeline

```
┌──────────────┐    ┌──────────────┐    ┌──────────────┐
│  EEG Signal  │───►│  Filtering   │───►│  Feature     │
│  Acquisition │    │  & Artifact  │    │  Extraction  │
└──────────────┘    │  Removal     │    └──────┬───────┘
                    └──────────────┘           │
                                               ▼
┌──────────────┐    ┌──────────────┐    ┌──────────────┐
│  Command     │◄───│  Intent      │◄───│  Pattern     │
│  Execution   │    │  Validation  │    │  Recognition │
└──────────────┘    └──────────────┘    └──────────────┘
```

### 4.3 Fallback Hierarchy

```typescript
interface BCIFallback {
  // BCI 신호 품질 저하 시
  signal_quality_low: {
    action: 'switch_to_gaze' | 'switch_to_voice' | 'alert_passenger';
    threshold: 0.5;
    delay_seconds: 10;
  };

  // BCI 연결 끊김
  connection_lost: {
    action: 'switch_to_alternative';
    alternatives: ['eye_gaze', 'voice', 'touch'];
    alert_passenger: true;
  };

  // 위급 상황 시
  emergency_override: {
    auto_pullover: true;
    contact_support: true;
    notify_emergency_contact: true;
  };
}
```

---

## 5. AAC Integration

### 5.1 Voice Command Schema

```yaml
voice_commands:
  navigation:
    - pattern: "(목적지|행선지)를? (.+)(으로|로) (변경|바꿔)"
      action: change_destination
      params: [destination]
      confirmation: true

    - pattern: "(여기|지금) (세워|정차|멈춰)"
      action: pull_over
      confirmation: false  # 즉시 실행

    - pattern: "(이동|여행) (취소|그만)"
      action: cancel_trip
      confirmation: true

  comfort:
    - pattern: "온도 (올려|높여|따뜻하게)"
      action: temperature_up
      params: [delta: 2]

    - pattern: "온도 (내려|낮춰|시원하게)"
      action: temperature_down
      params: [delta: 2]

    - pattern: "(창문|창) (열어|내려)"
      action: window_open
      params: [position: 50]

  emergency:
    - pattern: "(도움|도와줘|help|긴급)"
      action: request_assistance
      priority: high
      confirmation: false

  information:
    - pattern: "(언제|몇 시에) (도착|도달)"
      action: announce_eta
      response: tts

    - pattern: "(어디|현재 위치)"
      action: announce_location
      response: tts
```

### 5.2 Symbol-to-Action Mapping

```typescript
const AAC_SYMBOL_MAP: Map<string, VehicleAction> = new Map([
  // PCS (Picture Communication Symbols) 매핑
  ['pcs_stop', { action: 'emergency_stop', priority: 'critical' }],
  ['pcs_go', { action: 'resume_trip', priority: 'normal' }],
  ['pcs_help', { action: 'request_assistance', priority: 'high' }],
  ['pcs_home', { action: 'navigate_home', priority: 'normal' }],
  ['pcs_hot', { action: 'temperature_down', priority: 'low' }],
  ['pcs_cold', { action: 'temperature_up', priority: 'low' }],
  ['pcs_toilet', { action: 'find_restroom', priority: 'normal' }],
  ['pcs_eat', { action: 'find_restaurant', priority: 'low' }],

  // Blissymbols 매핑
  ['bliss_vehicle', { action: 'vehicle_status', priority: 'low' }],
  ['bliss_time', { action: 'announce_eta', priority: 'low' }],
  ['bliss_place', { action: 'announce_location', priority: 'low' }],
]);
```

### 5.3 TTS Announcements

```yaml
tts_config:
  voice:
    language: ko-KR
    rate: 0.9              # 약간 느리게
    pitch: 1.0
    voice_id: ko-KR-Standard-A

  announcements:
    trip_start:
      template: "{destination}(으)로 출발합니다. 예상 도착 시간은 {eta}입니다."
      priority: normal

    approaching_destination:
      template: "곧 {destination}에 도착합니다. 하차 준비해 주세요."
      advance_time: 2min
      priority: normal

    securement_reminder:
      template: "휠체어 고정을 확인해 주세요."
      trigger: trip_start
      priority: high

    emergency_alert:
      template: "긴급 상황입니다. {action}을 진행합니다."
      priority: critical
      volume_boost: true

  accessibility:
    repeat_on_request: true
    slow_mode_available: true
    visual_text_display: true
```

---

## 6. Fleet/MaaS Integration

### 6.1 GTFS-RT Accessibility Extension

```protobuf
// GTFS-RT 접근성 확장
message AccessibleVehiclePosition {
  required VehiclePosition vehicle = 1;

  // 접근성 상태
  optional AccessibilityStatus accessibility = 100;
}

message AccessibilityStatus {
  // 휠체어 공간
  optional int32 wheelchair_spaces_available = 1;
  optional int32 wheelchair_spaces_total = 2;

  // 진입 방식
  optional EntryType entry_type = 3;
  optional bool ramp_deployed = 4;
  optional bool lift_operational = 5;

  // 현재 승객
  repeated PassengerAccessibilityInfo passengers = 6;
}

enum EntryType {
  RAMP = 0;
  LIFT = 1;
  LEVEL_BOARDING = 2;
}

message PassengerAccessibilityInfo {
  optional string passenger_id = 1;
  optional MobilityAidType mobility_aid = 2;
  optional bool secured = 3;
}
```

### 6.2 Multi-Modal Trip Planning

```typescript
interface AccessibleTripPlan {
  // 전체 여정
  journey: {
    origin: Location;
    destination: Location;
    total_duration: Duration;
    total_transfers: number;
  };

  // 구간별 상세
  legs: TripLeg[];

  // 접근성 요약
  accessibility_summary: {
    all_segments_accessible: boolean;
    wheelchair_accessible: boolean;
    step_free: boolean;
    audio_guidance_available: boolean;
    issues: AccessibilityIssue[];
  };
}

interface TripLeg {
  mode: 'walk' | 'wheelchair' | 'autonomous_vehicle' | 'bus' | 'subway';
  from: Location;
  to: Location;
  duration: Duration;

  // 모드별 상세
  walk_details?: {
    distance: number;
    surface_type: string;
    slope_info: SlopeInfo;
    obstacles: Obstacle[];
  };

  vehicle_details?: {
    vehicle_id: string;
    accessibility_features: AccessibilityFeatures;
    boarding_time: Duration;
    securement_type: SecurementType;
  };

  transfer_details?: {
    transfer_time: Duration;
    assistance_available: boolean;
    accessible_path: boolean;
  };
}
```

### 6.3 Reservation System

```yaml
wheelchair_reservation:
  # 예약 요청
  request:
    trip_id: string
    passenger_profile_id: string
    wheelchair_type: manual | power | scooter
    dimensions:
      width: float
      length: float
    weight: float
    companion: boolean
    assistance_needed: boolean

  # 예약 응답
  response:
    reservation_id: string
    confirmed: boolean
    vehicle_id: string
    boarding_point: Location
    boarding_time: DateTime
    space_assigned: number     # 1 or 2
    special_instructions: string

  # 예약 정책
  policy:
    advance_booking: 30min     # 최소 사전 예약
    hold_time: 5min            # 도착 대기 시간
    priority_boarding: true    # 우선 탑승
    companion_free: true       # 동반자 무료
```

---

## 7. CLI Tool Specification

### 7.1 Command Structure

```
wia-auto <command> <subcommand> [options] [arguments]

Commands:
  profile     Manage passenger profiles
  vehicle     Query and interact with vehicles
  trip        Request and manage trips
  sim         Run simulations
  config      Configure CLI settings
  auth        Authentication management
```

### 7.2 Profile Commands

```bash
# 프로필 생성
wia-auto profile create \
  --name "홍길동" \
  --wheelchair power \
  --securement rear \
  --entry-preference ramp \
  --output profile.json

# 프로필 목록
wia-auto profile list [--format table|json]

# 프로필 상세
wia-auto profile show <profile-id>

# 프로필 수정
wia-auto profile update <profile-id> \
  --entry-preference lift

# 프로필 삭제
wia-auto profile delete <profile-id>

# 프로필 내보내기/가져오기
wia-auto profile export <profile-id> -o profile.json
wia-auto profile import profile.json
```

### 7.3 Vehicle Commands

```bash
# 차량 검색
wia-auto vehicle find \
  --location "37.5665,126.9780" \
  --radius 5km \
  --wheelchair \
  --ramp

# 차량 상태
wia-auto vehicle status <vehicle-id>

# 차량 기능
wia-auto vehicle capabilities <vehicle-id>

# 실시간 위치 추적
wia-auto vehicle track <vehicle-id> --follow
```

### 7.4 Trip Commands

```bash
# 이동 요청
wia-auto trip request \
  --from "서울역" \
  --to "강남역" \
  --profile <profile-id> \
  --passengers 1

# 이동 상태 확인
wia-auto trip status <trip-id>

# 이동 취소
wia-auto trip cancel <trip-id>

# 이동 기록
wia-auto trip history [--from 2024-01-01] [--to 2024-12-31]
```

### 7.5 Simulation Commands

```bash
# 시뮬레이션 시작
wia-auto sim start \
  --scenario wheelchair-boarding \
  --vehicles 5

# 차량 추가
wia-auto sim vehicle add \
  --type accessible \
  --location "37.5,127.0"

# 이동 시뮬레이션
wia-auto sim trip \
  --from "37.5,126.9" \
  --to "37.6,127.0" \
  --profile default

# 시나리오 실행
wia-auto sim scenario run emergency-stop
wia-auto sim scenario run multi-wheelchair
wia-auto sim scenario run fleet-dispatch
```

### 7.6 Configuration

```bash
# 설정 보기
wia-auto config show

# 설정 변경
wia-auto config set api.endpoint https://api.wia.auto
wia-auto config set output.format json
wia-auto config set language ko

# 인증
wia-auto auth login
wia-auto auth logout
wia-auto auth status
```

---

## 8. Security Considerations

### 8.1 Integration Security

```yaml
security:
  wheelchair_integration:
    authentication: mutual_tls
    encryption: aes_256_gcm
    message_signing: hmac_sha256
    session_timeout: 30min

  bci_integration:
    data_encryption: required
    local_processing: preferred
    cloud_fallback: encrypted_only
    consent_required: explicit

  fleet_integration:
    api_authentication: oauth2
    data_anonymization: true
    audit_logging: required
```

### 8.2 Privacy Protection

```typescript
interface PrivacyConfig {
  // 데이터 최소화
  data_minimization: {
    collect_only_necessary: true;
    auto_delete_after: '30days';
    anonymize_for_analytics: true;
  };

  // 동의 관리
  consent: {
    location_sharing: 'per_trip';
    health_data: 'explicit';
    usage_analytics: 'opt_in';
  };

  // 데이터 주권
  data_sovereignty: {
    storage_region: 'local';
    cross_border_transfer: 'prohibited';
    right_to_deletion: true;
  };
}
```

---

## 9. Error Handling

### 9.1 Integration Errors

```typescript
enum IntegrationError {
  // 휠체어 연동
  WHEELCHAIR_NOT_DETECTED = 'INT_WC_001',
  DOCKING_ALIGNMENT_FAILED = 'INT_WC_002',
  SECUREMENT_VERIFICATION_FAILED = 'INT_WC_003',

  // Eye Gaze 연동
  GAZE_CALIBRATION_FAILED = 'INT_EG_001',
  GAZE_TRACKING_LOST = 'INT_EG_002',

  // BCI 연동
  BCI_SIGNAL_QUALITY_LOW = 'INT_BCI_001',
  BCI_CONNECTION_LOST = 'INT_BCI_002',
  BCI_PATTERN_UNRECOGNIZED = 'INT_BCI_003',

  // AAC 연동
  VOICE_RECOGNITION_FAILED = 'INT_AAC_001',
  COMMAND_NOT_UNDERSTOOD = 'INT_AAC_002',

  // Fleet 연동
  FLEET_API_UNAVAILABLE = 'INT_FLEET_001',
  RESERVATION_FAILED = 'INT_FLEET_002',
  NO_ACCESSIBLE_VEHICLE = 'INT_FLEET_003',
}
```

### 9.2 Recovery Procedures

```yaml
recovery_procedures:
  wheelchair_docking_failed:
    steps:
      - announce: "도킹에 문제가 발생했습니다. 다시 시도합니다."
      - retry: 2
      - fallback: manual_guidance
      - escalate: request_assistance

  bci_connection_lost:
    steps:
      - announce: "BCI 연결이 끊어졌습니다."
      - switch_to: eye_gaze
      - if_unavailable: voice_control
      - notify: support_team

  no_accessible_vehicle:
    steps:
      - search_expanded_radius: true
      - suggest_alternatives: true
      - offer_wait_time: true
      - contact_support: if_critical
```

---

## 10. 弘益人間

이동의 자유를. 모든 사람에게. 어디서든.

### 10.1 Design Philosophy

- **포용적 설계**: 모든 능력의 사람들을 위한 설계
- **기술의 인간화**: 기술이 사람을 돕는 도구
- **장벽 없는 이동**: 물리적, 디지털 장벽 제거
- **존엄한 경험**: 모든 이용자에게 동등한 경험

### 10.2 Commitment

WIA 자율주행 차량 접근성 표준은 이동의 자유가 기본권임을 인정하고,
기술을 통해 모든 사람이 어디서든 자유롭게 이동할 수 있는 세상을 만들기 위해 노력합니다.

---

*WIA - Worldwide Inclusive Accessibility*
*弘益人間 - 널리 인간을 이롭게 하다*
