# WIA-CITY-014: 보안 시스템 표준 v1.0 🔒

> **弘益人間 (홍익인간) - 널리 인간을 이롭게 하라**

**표준 ID:** WIA-CITY-014
**버전:** 1.0.0
**발행일:** 2025-12-25
**상태:** 활성
**카테고리:** 스마트 시티 (CITY)

---

## 목차

1. [개요](#1-개요)
2. [적용 범위](#2-적용-범위)
3. [용어 정의](#3-용어-정의)
4. [CCTV 감시 시스템](#4-cctv-감시-시스템)
5. [침입 탐지 시스템](#5-침입-탐지-시스템)
6. [경비 시스템 통합](#6-경비-시스템-통합)
7. [비상 호출 시스템](#7-비상-호출-시스템)
8. [영상 분석 및 AI 감지](#8-영상-분석-및-ai-감지)
9. [보안 이벤트 관리](#9-보안-이벤트-관리)
10. [통합 관제 센터](#10-통합-관제-센터)
11. [접근 제어 통합](#11-접근-제어-통합)
12. [센서 네트워크](#12-센서-네트워크)
13. [데이터 모델](#13-데이터-모델)
14. [API 명세](#14-api-명세)
15. [보안 및 개인정보보호](#15-보안-및-개인정보보호)
16. [성과 지표 (KPI)](#16-성과-지표-kpi)

---

## 1. 개요

### 1.1 목적

WIA-CITY-014 보안 시스템 표준은 도시 및 건물의 안전과 보안을 보장하기 위한 통합 보안 시스템의 국제 표준입니다.

본 표준은 CCTV 감시, 침입 탐지, 경비 시스템, 비상 호출, AI 영상 분석을 통합하여 포괄적인 보안 솔루션을 제공합니다.

### 1.2 핵심 원칙

- **통합 플랫폼 (Integrated Platform)**: 모든 보안 시스템의 단일 플랫폼 통합
- **실시간 감시 (Real-time Monitoring)**: 24/7 실시간 모니터링 및 대응
- **지능형 분석 (Intelligent Analytics)**: AI 기반 위협 탐지 및 예측
- **신속한 대응 (Rapid Response)**: 자동화된 경보 및 즉각적인 대응 체계
- **개인정보 보호 (Privacy Protection)**: GDPR 및 개인정보보호법 준수
- **상호운용성 (Interoperability)**: 다양한 보안 장비 간 호환
- **확장성 (Scalability)**: 소규모부터 대규모까지 유연한 확장
- **신뢰성 (Reliability)**: 99.9% 이상의 시스템 가용성 보장

### 1.3 적용 대상

- 스마트 빌딩 및 오피스
- 주거 단지 및 아파트
- 상업 시설 (쇼핑몰, 은행, 매장)
- 산업 시설 (공장, 물류센터, 창고)
- 공공 시설 (학교, 병원, 관공서)
- 교통 인프라 (공항, 역, 터미널)
- 스마트 시티 통합 보안 체계

---

## 2. 적용 범위

### 2.1 보안 시스템 구성요소

본 표준은 다음 구성요소에 적용됩니다:

- **CCTV 카메라 (Cameras)**: IP 카메라, PTZ 카메라, 열화상 카메라
- **침입 탐지 센서 (Intrusion Sensors)**: PIR, 도어/창문 센서, 유리 파손 감지
- **경비 시스템 (Guard System)**: 순찰 관리, 경비원 위치 추적
- **비상 호출 (Emergency Call)**: 비상 버튼, 인터폰, SOS 시스템
- **영상 분석 (Video Analytics)**: AI 객체 인식, 행동 분석, 얼굴 인식
- **관제 센터 (Control Center)**: 통합 모니터링 및 제어 시스템
- **저장 장치 (Storage)**: NVR, DVR, 클라우드 스토리지
- **네트워크 (Network)**: 유무선 통신 인프라

### 2.2 보안 시나리오

- **침입 탐지 (Intrusion Detection)**: 무단 침입 감지 및 경보
- **화재 감지 (Fire Detection)**: 화재 센서 통합 및 자동 알림
- **출입 통제 (Access Control)**: 출입 권한 관리 및 기록
- **주차 관리 (Parking Management)**: 차량 번호 인식 및 주차 보안
- **군중 관리 (Crowd Management)**: 과밀 감지 및 안전 관리
- **응급 대응 (Emergency Response)**: 신속한 비상 상황 대응
- **자산 보호 (Asset Protection)**: 중요 자산 실시간 감시

### 2.3 관리 단계

- 위험 평가 및 보안 설계
- 장비 선정 및 설치
- 네트워크 구축 및 연결
- 시스템 통합 및 테스트
- 운영 및 모니터링
- 사고 대응 및 조사
- 유지보수 및 업그레이드

---

## 3. 용어 정의

### 3.1 보안 시스템 용어

| 용어 | 정의 |
|------|------|
| CCTV | Closed-Circuit Television (폐쇄회로 텔레비전) |
| PTZ | Pan-Tilt-Zoom (회전-틸트-줌 카메라) |
| NVR | Network Video Recorder (네트워크 비디오 녹화기) |
| DVR | Digital Video Recorder (디지털 비디오 녹화기) |
| PIR | Passive Infrared (수동 적외선 센서) |
| IVS | Intelligent Video Surveillance (지능형 영상 감시) |
| VMS | Video Management System (영상 관리 시스템) |
| PSIM | Physical Security Information Management |
| ANPR | Automatic Number Plate Recognition (번호판 인식) |
| LPR | License Plate Recognition (번호판 인식) |

### 3.2 영상 해상도

| 해상도 | 픽셀 | 용도 |
|--------|------|------|
| VGA | 640x480 | 기본 감시 |
| HD (720p) | 1280x720 | 일반 CCTV |
| Full HD (1080p) | 1920x1080 | 고화질 CCTV |
| 2K | 2048x1080 | 상세 감시 |
| 4K (UHD) | 3840x2160 | 초고화질 감시 |
| 8K | 7680x4320 | 극세밀 감시 |

### 3.3 센서 유형

- **PIR 센서**: 인체 열 감지
- **마이크로파 센서**: 움직임 감지
- **도어/창문 센서**: 개폐 감지
- **유리 파손 센서**: 유리 깨짐 감지
- **진동 센서**: 충격 및 진동 감지
- **압력 센서**: 압력 변화 감지
- **레이저/빔 센서**: 경계 침입 감지

---

## 4. CCTV 감시 시스템

### 4.1 카메라 유형

#### 4.1.1 고정형 카메라 (Fixed Camera)

```json
{
  "cameraType": "fixed",
  "resolution": "1920x1080",
  "fps": 30,
  "lens": {
    "focal_length": "3.6mm",
    "field_of_view": 85,
    "aperture": "f/1.6"
  },
  "features": {
    "night_vision": true,
    "ir_distance": 30,
    "wdr": true,
    "defog": true
  }
}
```

#### 4.1.2 PTZ 카메라 (Pan-Tilt-Zoom Camera)

```json
{
  "cameraType": "ptz",
  "resolution": "3840x2160",
  "fps": 60,
  "pan": {
    "range": 360,
    "speed": 300
  },
  "tilt": {
    "range": 180,
    "speed": 120
  },
  "zoom": {
    "optical": "30x",
    "digital": "16x"
  },
  "presets": 256,
  "tours": 8,
  "auto_tracking": true
}
```

#### 4.1.3 열화상 카메라 (Thermal Camera)

```json
{
  "cameraType": "thermal",
  "resolution": "640x512",
  "detector": "uncooled VOx microbolometer",
  "spectral_range": "8-14μm",
  "temperature_range": {
    "min": -40,
    "max": 550
  },
  "sensitivity": "<50mK",
  "features": {
    "fire_detection": true,
    "temperature_alarm": true,
    "fusion_mode": "picture_in_picture"
  }
}
```

### 4.2 카메라 배치 전략

#### 4.2.1 커버리지 분석

- **입구/출구**: 모든 출입구 100% 커버
- **주요 동선**: 복도, 계단, 엘리베이터 커버
- **중요 구역**: 금고, 서버실, 자산 보관소
- **사각지대 제거**: 중복 카메라 배치로 사각지대 최소화
- **높이 설정**: 2.5~3m 높이로 파손 방지 및 최적 화각 확보

#### 4.2.2 카메라 존 (Camera Zone)

```typescript
interface CameraZone {
  zone_id: string;
  zone_name: string;
  cameras: string[];              // Camera IDs
  priority: 'high' | 'medium' | 'low';
  recording_mode: 'continuous' | 'motion' | 'schedule' | 'event';
  retention_days: number;
  analytics_enabled: boolean;
}
```

### 4.3 영상 녹화 및 저장

#### 4.3.1 녹화 모드

| 모드 | 설명 | 저장 용량 | 용도 |
|------|------|----------|------|
| 연속 녹화 | 24시간 녹화 | 매우 높음 | 중요 구역 |
| 모션 감지 | 움직임 감지 시 녹화 | 중간 | 일반 구역 |
| 스케줄 | 특정 시간대만 녹화 | 낮음 | 야간/주말만 |
| 이벤트 | 경보 발생 시 녹화 | 매우 낮음 | 보조 카메라 |
| 스마트 | AI 분석 기반 선택적 녹화 | 낮음 | 최적화 |

#### 4.3.2 저장 용량 계산

```
용량 (GB) = (비트레이트 x 3600 x 시간 x 일수) / (8 x 1024 x 1024 x 1024)

예시 (1080p, 30fps, H.265):
- 비트레이트: 2 Mbps
- 녹화 시간: 24시간
- 보관 기간: 30일
- 카메라 수: 16대

용량 = (2 x 3600 x 24 x 30 x 16) / (8 x 1024 x 1024 x 1024)
    ≈ 4,953 GB (약 5TB)
```

#### 4.3.3 NVR 사양

```json
{
  "nvr_model": "WIA-NVR-64CH",
  "channels": 64,
  "resolution_support": "up to 12MP",
  "recording_bandwidth": "320 Mbps",
  "storage": {
    "bays": 16,
    "capacity_per_bay": "14TB",
    "total_capacity": "224TB",
    "raid_support": ["RAID 0", "RAID 1", "RAID 5", "RAID 6", "RAID 10"]
  },
  "network": {
    "ports": ["4x GbE", "2x 10GbE"],
    "redundant": true
  },
  "playback": {
    "simultaneous_channels": 16,
    "speed_range": "1/8x ~ 256x"
  }
}
```

### 4.4 실시간 모니터링

#### 4.4.1 비디오 월 (Video Wall)

```typescript
interface VideoWall {
  wall_id: string;
  layout: {
    rows: number;
    columns: number;
    total_screens: number;
  };
  displays: {
    display_id: string;
    resolution: string;           // "1920x1080"
    size: number;                 // inches
    bezel_width: number;          // mm
  }[];
  presets: {
    preset_id: string;
    name: string;
    camera_assignments: {
      display_id: string;
      cameras: string[];          // Camera IDs to show
      layout: string;             // "1x1", "2x2", "3x3", etc.
    }[];
  }[];
}
```

#### 4.4.2 라이브 뷰 기능

- **멀티뷰**: 1/4/9/16/25/36/64 분할 화면
- **시퀀스 뷰**: 자동 카메라 전환 (설정 가능한 간격)
- **PTZ 제어**: 조이스틱 또는 마우스로 실시간 제어
- **디지털 줌**: 라이브 영상 확대/축소
- **북마크**: 중요 순간 즉시 마킹
- **스냅샷**: 정지 화면 캡처 및 저장

---

## 5. 침입 탐지 시스템

### 5.1 센서 유형 및 사양

#### 5.1.1 PIR 모션 센서

```json
{
  "sensor_type": "pir_motion",
  "detection_range": 12,          // meters
  "detection_angle": 110,         // degrees
  "mounting_height": "2.1-2.4m",
  "sensitivity": {
    "adjustable": true,
    "levels": 10,
    "pet_immunity": "up to 25kg"
  },
  "environmental": {
    "temperature_range": "-10°C to 55°C",
    "humidity": "0-95% RH (non-condensing)"
  },
  "alarm_output": {
    "type": "normally_closed",
    "duration": "2-200 seconds (adjustable)"
  }
}
```

#### 5.1.2 도어/창문 센서

```json
{
  "sensor_type": "door_window",
  "technology": "magnetic_reed_switch",
  "gap_tolerance": "25mm",
  "contact_resistance": "<100Ω",
  "tamper_protection": true,
  "installation": {
    "surface_mount": true,
    "concealed_mount": true
  },
  "indicators": {
    "led": true,
    "buzzer": false
  }
}
```

#### 5.1.3 유리 파손 감지기

```json
{
  "sensor_type": "glass_break",
  "detection_method": "acoustic",
  "frequency_range": "0.1-20 kHz",
  "coverage": {
    "radius": 9,                  // meters
    "max_glass_area": 100         // square meters
  },
  "sensitivity": {
    "adjustable": true,
    "glass_types": [
      "tempered",
      "laminated",
      "wired",
      "plate"
    ]
  },
  "false_alarm_rejection": {
    "dual_verification": true,
    "frequency_analysis": true
  }
}
```

#### 5.1.4 진동 센서

```json
{
  "sensor_type": "vibration",
  "technology": "piezoelectric",
  "sensitivity": "adjustable (0-9)",
  "detection": {
    "drilling": true,
    "hammering": true,
    "cutting": true,
    "impact": true
  },
  "mounting": "wall/ceiling/floor",
  "coverage_area": "up to 50m²"
}
```

### 5.2 침입 탐지 존 (Intrusion Detection Zone)

#### 5.2.1 보안 존 설정

```typescript
interface SecurityZone {
  zone_id: string;
  zone_name: string;
  zone_type: 'perimeter' | 'interior' | 'critical' | 'access_control';

  sensors: {
    sensor_id: string;
    sensor_type: string;
    location: {
      building: string;
      floor: string;
      room: string;
      coordinates: { x: number; y: number };
    };
  }[];

  arming_schedule: {
    weekdays: { start: string; end: string };
    weekends: { start: string; end: string };
    holidays: string[];           // Exception dates
  };

  alarm_settings: {
    entry_delay: number;          // seconds
    exit_delay: number;           // seconds
    alarm_duration: number;       // seconds
    chime: boolean;
    notification: {
      sms: string[];
      email: string[];
      push: boolean;
      patrol: boolean;
    };
  };

  response_plan: {
    priority: 'critical' | 'high' | 'medium' | 'low';
    auto_actions: string[];       // ["lock_doors", "turn_on_lights", "sound_siren"]
    dispatch_security: boolean;
    notify_police: boolean;
  };
}
```

#### 5.2.2 다층 방어 (Defense in Depth)

```
1층 - 외곽 경계 (Perimeter):
  - 레이저/빔 센서
  - 울타리 진동 센서
  - 외부 PIR 센서
  - 외곽 CCTV

2층 - 건물 외벽 (Building Envelope):
  - 도어/창문 센서
  - 유리 파손 감지
  - 진동 센서
  - 외벽 카메라

3층 - 내부 공간 (Interior):
  - 내부 PIR 센서
  - 복도 카메라
  - 내부 도어 센서

4층 - 중요 구역 (Critical Areas):
  - 고감도 PIR
  - 고해상도 카메라
  - 출입 통제
  - 온도/습도 센서
```

### 5.3 경보 발생 및 처리

#### 5.3.1 경보 우선순위

| 레벨 | 이름 | 대응 시간 | 조치 |
|------|------|----------|------|
| 1 | Critical | 즉시 | 경찰 출동, 경비 출동, 자동 잠금 |
| 2 | High | 1분 이내 | 경비 출동, 관리자 알림 |
| 3 | Medium | 5분 이내 | 현장 확인, 기록 |
| 4 | Low | 30분 이내 | 로그 기록, 주간 확인 |
| 5 | Info | 일일 보고 | 통계 집계 |

#### 5.3.2 경보 검증 (Alarm Verification)

```typescript
interface AlarmVerification {
  alarm_id: string;
  timestamp: string;
  zone_id: string;
  sensor_id: string;
  alarm_type: string;

  verification_methods: {
    video_verification: {
      enabled: boolean;
      pre_alarm_seconds: number;   // 10초 전부터 녹화
      post_alarm_seconds: number;  // 30초 후까지 녹화
      cameras: string[];
    };

    audio_verification: {
      enabled: boolean;
      two_way_audio: boolean;
      listen_in: boolean;
    };

    multi_sensor_correlation: {
      enabled: boolean;
      required_sensors: number;    // 2개 이상 센서 동시 감지 시 경보
      time_window: number;         // 10초 이내
    };

    sequential_verification: {
      enabled: boolean;
      sequence: string[];          // ["door_sensor", "motion_sensor"]
      timeout: number;             // 60초
    };
  };

  false_alarm_prevention: {
    pet_immunity: boolean;
    wind_compensation: boolean;
    temperature_compensation: boolean;
    adaptive_learning: boolean;
  };
}
```

---

## 6. 경비 시스템 통합

### 6.1 순찰 관리

#### 6.1.1 순찰 경로 및 체크포인트

```typescript
interface PatrolRoute {
  route_id: string;
  route_name: string;
  shift: 'day' | 'night' | 'graveyard';

  checkpoints: {
    checkpoint_id: string;
    location: {
      building: string;
      floor: string;
      area: string;
      gps?: { latitude: number; longitude: number };
    };
    sequence: number;
    scan_method: 'nfc' | 'qr_code' | 'rfid' | 'gps';
    required_tasks: string[];     // ["check_doors", "check_lights"]
    estimated_duration: number;   // minutes
  }[];

  schedule: {
    frequency: number;            // times per shift
    interval: number;             // minutes between patrols
    start_time: string;
    end_time: string;
  };

  compliance: {
    tolerance_minutes: number;    // 5분 허용 오차
    missed_checkpoint_action: 'alert' | 'log' | 'critical';
    incomplete_patrol_action: 'alert' | 'log' | 'escalate';
  };
}
```

#### 6.1.2 경비원 위치 추적

```typescript
interface GuardTracking {
  guard_id: string;
  name: string;
  shift: string;

  current_location: {
    timestamp: string;
    location: {
      building: string;
      floor: string;
      coordinates: { x: number; y: number };
      gps?: { latitude: number; longitude: number };
    };
    tracking_method: 'gps' | 'wifi' | 'ble_beacon' | 'checkpoint_scan';
  };

  status: 'on_patrol' | 'at_post' | 'break' | 'responding' | 'off_duty';

  communication: {
    radio_channel: number;
    phone: string;
    panic_button: {
      enabled: boolean;
      last_test: string;
    };
  };

  assigned_zones: string[];
  equipment: string[];            // ["flashlight", "radio", "taser"]
}
```

### 6.2 사건 대응

#### 6.2.1 대응 절차 (Response Procedure)

```typescript
interface ResponseProcedure {
  incident_type: string;
  priority: 'critical' | 'high' | 'medium' | 'low';

  steps: {
    step_number: number;
    description: string;
    responsible: 'guard' | 'supervisor' | 'police' | 'fire_dept';
    max_duration: number;         // seconds
    required_actions: string[];
    verification_required: boolean;
  }[];

  escalation: {
    trigger: 'time_exceeded' | 'severity_increase' | 'manual';
    notify: string[];             // Roles to notify
    auto_escalate_after: number;  // minutes
  };

  reporting: {
    incident_report_required: boolean;
    report_template: string;
    attachments_required: string[]; // ["photos", "video", "statements"]
    submission_deadline: number;  // hours
  };
}
```

---

## 7. 비상 호출 시스템

### 7.1 비상 버튼 및 인터폰

#### 7.1.1 비상 버튼 (Panic Button)

```json
{
  "device_type": "panic_button",
  "installation": "wall_mounted",
  "features": {
    "single_press": "alert",
    "double_press": "silent_alert",
    "hold_3s": "critical_alert"
  },
  "indicators": {
    "led": {
      "normal": "green",
      "pressed": "red_blinking",
      "acknowledged": "steady_red"
    },
    "audible": {
      "local_siren": true,
      "siren_delay": 3
    }
  },
  "communication": {
    "protocol": "ip",
    "backup": "cellular",
    "two_way_audio": true
  },
  "locations": [
    "elevators",
    "parking_garage",
    "stairwells",
    "restrooms",
    "isolated_areas"
  ]
}
```

#### 7.1.2 비디오 인터폰

```typescript
interface VideoIntercom {
  device_id: string;
  location: string;

  features: {
    video: {
      resolution: string;         // "1080p"
      camera: {
        field_of_view: number;
        night_vision: boolean;
        wide_dynamic_range: boolean;
      };
    };

    audio: {
      two_way: boolean;
      noise_cancellation: boolean;
      volume_adjustable: boolean;
    };

    display: {
      size: string;               // "7 inch"
      touchscreen: boolean;
      brightness_adjustable: boolean;
    };

    access_control: {
      integrated: boolean;
      unlock_methods: string[];   // ["keypad", "rfid", "face_recognition"]
      temporary_codes: boolean;
    };
  };

  call_routing: {
    primary: string;              // Guard station
    backup: string[];             // Supervisor, mobile app
    timeout: number;              // seconds before escalation
    recording: {
      enabled: boolean;
      storage_days: number;
    };
  };
}
```

### 7.2 긴급 상황 대응

#### 7.2.1 긴급 상황 유형

```typescript
type EmergencyType =
  | 'medical'              // 의료 응급
  | 'fire'                 // 화재
  | 'intrusion'            // 침입
  | 'assault'              // 폭행
  | 'natural_disaster'     // 자연재해
  | 'bomb_threat'          // 폭탄 위협
  | 'active_shooter'       // 총기 난사
  | 'hazmat'               // 위험물질 유출
  | 'evacuation'           // 긴급 대피
  | 'other';               // 기타

interface EmergencyAlert {
  alert_id: string;
  timestamp: string;
  type: EmergencyType;
  priority: 'critical' | 'high' | 'medium';

  location: {
    building: string;
    floor: string;
    room: string;
    coordinates: { x: number; y: number };
  };

  reporter: {
    type: 'guard' | 'employee' | 'visitor' | 'sensor' | 'ai_detection';
    id?: string;
    name?: string;
  };

  details: {
    description: string;
    casualties?: number;
    hazard_level?: 'low' | 'medium' | 'high';
    spread_risk?: boolean;
  };

  response: {
    dispatched: string[];         // ["guard_team_1", "ambulance", "fire_dept"]
    eta: number[];                // minutes
    status: 'dispatched' | 'en_route' | 'on_scene' | 'resolved';
  };

  communications: {
    pa_announcement: boolean;
    sms_broadcast: boolean;
    app_notification: boolean;
    email_notification: boolean;
  };
}
```

---

## 8. 영상 분석 및 AI 감지

### 8.1 AI 비디오 분석 기능

#### 8.1.1 객체 탐지 및 추적

```typescript
interface ObjectDetection {
  detection_id: string;
  timestamp: string;
  camera_id: string;

  object: {
    type: 'person' | 'vehicle' | 'animal' | 'package' | 'unknown';
    subtype?: string;             // "car", "truck", "bicycle", etc.
    confidence: number;           // 0.0 - 1.0

    bounding_box: {
      x: number;
      y: number;
      width: number;
      height: number;
    };

    attributes?: {
      color?: string;
      size?: 'small' | 'medium' | 'large';
      speed?: number;             // pixels/second
      direction?: number;         // degrees (0-360)
    };
  };

  tracking: {
    track_id: string;
    first_seen: string;
    last_seen: string;
    path: { x: number; y: number; timestamp: string }[];
    cameras_visited: string[];
  };
}
```

#### 8.1.2 행동 분석 (Behavior Analysis)

```typescript
interface BehaviorAnalysis {
  analysis_id: string;
  timestamp: string;
  camera_id: string;

  detected_behaviors: {
    behavior_type:
      | 'loitering'           // 배회
      | 'running'             // 달리기
      | 'fighting'            // 싸움
      | 'falling'             // 넘어짐
      | 'crowd_forming'       // 군중 형성
      | 'wrong_direction'     // 역주행
      | 'tailgating'          // 뒤따라가기
      | 'abandoned_object'    // 유기 물체
      | 'object_removed'      // 물건 제거
      | 'fence_jumping'       // 울타리 넘기
      | 'vehicle_stopped';    // 차량 정지

    confidence: number;
    duration: number;             // seconds
    location: { x: number; y: number };
    subjects: string[];           // tracking IDs

    alert: {
      triggered: boolean;
      severity: 'info' | 'warning' | 'critical';
      notification_sent: boolean;
    };
  }[];
}
```

#### 8.1.3 얼굴 인식

```typescript
interface FaceRecognition {
  detection_id: string;
  timestamp: string;
  camera_id: string;

  face: {
    bounding_box: {
      x: number;
      y: number;
      width: number;
      height: number;
    };

    quality: {
      score: number;              // 0.0 - 1.0
      sharpness: number;
      brightness: number;
      angle: {
        yaw: number;              // -90 to 90
        pitch: number;
        roll: number;
      };
    };

    features: {
      embedding: number[];        // 512-dimensional vector
      landmarks: {
        left_eye: { x: number; y: number };
        right_eye: { x: number; y: number };
        nose: { x: number; y: number };
        left_mouth: { x: number; y: number };
        right_mouth: { x: number; y: number };
      };
    };
  };

  recognition: {
    matched: boolean;
    person_id?: string;
    confidence?: number;
    watchlist_match?: {
      list_name: string;
      reason: string;
      alert_triggered: boolean;
    };
  };

  privacy: {
    anonymized: boolean;
    consent_obtained: boolean;
    retention_until: string;
  };
}
```

#### 8.1.4 차량 번호판 인식 (ANPR/LPR)

```typescript
interface LicensePlateRecognition {
  detection_id: string;
  timestamp: string;
  camera_id: string;

  vehicle: {
    type: 'car' | 'truck' | 'motorcycle' | 'bus';
    color?: string;
    make?: string;
    model?: string;
    bounding_box: {
      x: number;
      y: number;
      width: number;
      height: number;
    };
  };

  license_plate: {
    number: string;
    country: string;
    region?: string;
    confidence: number;

    bounding_box: {
      x: number;
      y: number;
      width: number;
      height: number;
    };

    image: {
      full_frame: string;         // URL
      cropped_plate: string;      // URL
    };
  };

  access_control: {
    authorized: boolean;
    whitelist_match?: boolean;
    blacklist_match?: boolean;
    visitor_pass?: string;
    action: 'allow' | 'deny' | 'alert' | 'manual_review';
  };

  parking: {
    entry_time?: string;
    exit_time?: string;
    duration?: number;            // minutes
    parking_spot?: string;
    fee?: number;
  };
}
```

### 8.2 이상 행동 탐지

#### 8.2.1 군중 분석

```typescript
interface CrowdAnalytics {
  analysis_id: string;
  timestamp: string;
  camera_id: string;
  zone_id: string;

  crowd_metrics: {
    people_count: number;
    density: number;              // people per m²
    flow_rate: number;            // people per minute

    occupancy: {
      current: number;
      capacity: number;
      percentage: number;
      status: 'normal' | 'busy' | 'crowded' | 'critical';
    };

    movement: {
      average_speed: number;      // m/s
      direction: {
        dominant: number;         // degrees
        entropy: number;          // 0.0 - 1.0 (higher = more chaotic)
      };
      bottleneck_detected: boolean;
    };
  };

  alerts: {
    overcrowding: boolean;
    stampede_risk: boolean;
    panic_detected: boolean;
    queue_too_long: boolean;
    exit_blocked: boolean;
  };

  recommendations: string[];      // ["open_additional_gate", "redirect_crowd"]
}
```

---

## 9. 보안 이벤트 관리

### 9.1 이벤트 로깅

```typescript
interface SecurityEvent {
  event_id: string;
  timestamp: string;
  event_type:
    | 'alarm'
    | 'access_granted'
    | 'access_denied'
    | 'sensor_triggered'
    | 'camera_motion'
    | 'patrol_checkpoint'
    | 'emergency_call'
    | 'system_status'
    | 'user_action'
    | 'configuration_change';

  severity: 'critical' | 'high' | 'medium' | 'low' | 'info';

  source: {
    type: 'camera' | 'sensor' | 'access_control' | 'guard' | 'system' | 'user';
    id: string;
    location?: string;
  };

  details: {
    description: string;
    metadata: Record<string, any>;
    related_events?: string[];    // Event IDs
  };

  response: {
    acknowledged: boolean;
    acknowledged_by?: string;
    acknowledged_at?: string;
    actions_taken?: string[];
    resolved: boolean;
    resolved_at?: string;
    notes?: string;
  };

  evidence: {
    video_clips?: string[];       // URLs
    photos?: string[];            // URLs
    audio_recordings?: string[];  // URLs
    documents?: string[];         // URLs
  };
}
```

### 9.2 사고 조사

```typescript
interface IncidentInvestigation {
  incident_id: string;
  created_at: string;
  status: 'open' | 'investigating' | 'closed' | 'archived';

  incident_details: {
    date: string;
    time: string;
    location: string;
    type: string;
    severity: 'critical' | 'high' | 'medium' | 'low';
    description: string;
  };

  involved_parties: {
    victims?: { name: string; contact: string; statement?: string }[];
    suspects?: { description: string; photo?: string }[];
    witnesses?: { name: string; contact: string; statement?: string }[];
  };

  evidence_collected: {
    video: {
      camera_id: string;
      start_time: string;
      end_time: string;
      file_url: string;
      notes?: string;
    }[];

    photos: {
      file_url: string;
      timestamp: string;
      description: string;
    }[];

    sensor_logs: {
      sensor_id: string;
      data: any[];
    }[];

    other_evidence: {
      type: string;
      description: string;
      file_url?: string;
    }[];
  };

  investigation: {
    lead_investigator: string;
    team_members: string[];
    findings: string;
    conclusion: string;
    recommendations: string[];
  };

  actions_taken: {
    disciplinary?: string;
    policy_changes?: string;
    system_upgrades?: string;
    training?: string;
  };

  reporting: {
    internal_report: string;      // URL
    police_report_filed: boolean;
    police_report_number?: string;
    insurance_claim?: string;
  };
}
```

---

## 10. 통합 관제 센터

### 10.1 관제 센터 구성

```typescript
interface ControlCenter {
  center_id: string;
  name: string;
  location: string;

  operators: {
    shift: 'day' | 'night' | 'graveyard';
    operators_on_duty: number;
    supervisor: string;
  };

  workstations: {
    workstation_id: string;
    operator_id?: string;

    displays: {
      primary: {
        size: string;             // "32 inch"
        resolution: string;       // "3840x2160"
        video_wall_feed: boolean;
      };
      secondary: {
        size: string;
        resolution: string;
        applications: string[];   // ["alarm_panel", "access_control"]
      }[];
    };

    controls: {
      joystick: boolean;
      keyboard: boolean;
      mouse: boolean;
      emergency_buttons: string[];
    };
  }[];

  systems_integrated: {
    cctv: boolean;
    intrusion_detection: boolean;
    access_control: boolean;
    fire_alarm: boolean;
    building_automation: boolean;
    intercom: boolean;
    parking: boolean;
  };
}
```

---

## 11. 접근 제어 통합

### 11.1 출입 통제 연동

```typescript
interface AccessControlIntegration {
  integration_id: string;

  access_events: {
    event_id: string;
    timestamp: string;
    door_id: string;
    user_id?: string;
    credential_type: 'card' | 'pin' | 'biometric' | 'mobile';
    result: 'granted' | 'denied';

    reason?: 'unauthorized' | 'expired' | 'invalid_pin' | 'door_forced' | 'held_open';

    camera_snapshot?: {
      camera_id: string;
      image_url: string;
    };

    actions_triggered: {
      alert_sent: boolean;
      lock_door: boolean;
      notify_security: boolean;
      log_event: boolean;
    };
  };

  tailgating_detection: {
    enabled: boolean;
    camera_id: string;
    algorithm: 'count_people' | 'measure_distance' | 'ai_detection';
    threshold: number;
    action_on_detect: 'alert' | 'deny_access' | 'log_only';
  };
}
```

---

## 12. 센서 네트워크

### 12.1 센서 통신 프로토콜

- **유선**: RS-485, Ethernet
- **무선**: Zigbee, Z-Wave, LoRaWAN, NB-IoT
- **하이브리드**: 유선 백본 + 무선 센서

### 12.2 센서 상태 모니터링

```typescript
interface SensorHealth {
  sensor_id: string;
  status: 'online' | 'offline' | 'low_battery' | 'malfunction';

  battery: {
    level: number;                // %
    voltage: number;              // V
    estimated_days_remaining: number;
  };

  communication: {
    signal_strength: number;      // dBm
    packet_loss: number;          // %
    last_heartbeat: string;
  };

  diagnostics: {
    temperature: number;          // °C
    humidity: number;             // %
    uptime: number;               // hours
    reboot_count: number;
  };

  maintenance: {
    last_service: string;
    next_service: string;
    calibration_due: string;
  };
}
```

---

## 13. 데이터 모델

### 13.1 핵심 데이터 구조

상세한 TypeScript 타입 정의는 `api/typescript/src/types.ts` 참조

---

## 14. API 명세

### 14.1 REST API 엔드포인트

#### 카메라 관리

```
GET    /api/v1/cameras              - 카메라 목록 조회
GET    /api/v1/cameras/{id}         - 카메라 상세 조회
POST   /api/v1/cameras              - 카메라 등록
PUT    /api/v1/cameras/{id}         - 카메라 정보 수정
DELETE /api/v1/cameras/{id}         - 카메라 삭제
GET    /api/v1/cameras/{id}/stream  - 실시간 스트림
POST   /api/v1/cameras/{id}/ptz     - PTZ 제어
POST   /api/v1/cameras/{id}/snapshot - 스냅샷 캡처
```

#### 센서 관리

```
GET    /api/v1/sensors              - 센서 목록 조회
GET    /api/v1/sensors/{id}         - 센서 상세 조회
POST   /api/v1/sensors              - 센서 등록
PUT    /api/v1/sensors/{id}         - 센서 정보 수정
DELETE /api/v1/sensors/{id}         - 센서 삭제
GET    /api/v1/sensors/{id}/status  - 센서 상태 조회
```

#### 경보 관리

```
GET    /api/v1/alerts               - 경보 목록 조회
GET    /api/v1/alerts/{id}          - 경보 상세 조회
POST   /api/v1/alerts/{id}/acknowledge - 경보 확인
POST   /api/v1/alerts/{id}/resolve  - 경보 해결
GET    /api/v1/alerts/active        - 활성 경보 목록
```

#### 이벤트 조회

```
GET    /api/v1/events               - 이벤트 목록 조회
GET    /api/v1/events/{id}          - 이벤트 상세 조회
GET    /api/v1/events/search        - 이벤트 검색
GET    /api/v1/events/export        - 이벤트 내보내기
```

---

## 15. 보안 및 개인정보보호

### 15.1 데이터 보안

- **암호화**: TLS 1.3, AES-256
- **인증**: OAuth 2.0, JWT
- **권한**: RBAC (Role-Based Access Control)
- **감사 로그**: 모든 접근 및 조작 기록

### 15.2 개인정보보호

- **영상 보관 기간**: 법적 요구사항 준수 (일반적으로 30일)
- **접근 권한**: 최소 권한 원칙
- **익명화**: 필요 시 얼굴 블러링
- **동의**: 촬영 안내 및 동의 획득
- **GDPR 준수**: EU 지역 운영 시 GDPR 완전 준수

---

## 16. 성과 지표 (KPI)

### 16.1 시스템 성능 KPI

| KPI | 목표 | 측정 방법 |
|-----|------|----------|
| 시스템 가용성 | 99.9% | 월간 다운타임 측정 |
| 경보 대응 시간 | 1분 이내 | 평균 대응 시간 |
| 오경보율 | 5% 이하 | 오경보 / 총 경보 |
| 영상 손실률 | 0.1% 이하 | 손실 시간 / 전체 시간 |
| 센서 온라인율 | 98% 이상 | 온라인 센서 / 전체 센서 |

### 16.2 보안 효과 KPI

| KPI | 측정 방법 |
|-----|----------|
| 침입 탐지율 | 탐지된 침입 / 총 침입 시도 |
| 사고 해결 시간 | 사고 발생 ~ 해결 완료 시간 |
| 재발 방지율 | (1 - 재발 사고 / 총 사고) x 100% |
| 순찰 완료율 | 완료된 순찰 / 예정된 순찰 |

---

**弘益人間 (홍익인간) - Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 MIT License*
