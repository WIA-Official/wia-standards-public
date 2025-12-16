# WIA Smart Home - Phase 1: Data Format Specification

## 1. Overview

이 문서는 WIA Smart Home Accessibility Standard의 데이터 형식을 정의합니다.
스마트홈 기기의 접근성 기능을 표준화하여 상호운용성과 포용적 경험을 제공합니다.

### 1.1 Design Principles

| 원칙 | 설명 |
|------|------|
| **Matter 호환** | CSA Matter 데이터 모델과 매핑 가능 |
| **WCAG 준수** | 웹 접근성 4대 원칙 반영 |
| **다중 모달** | 다양한 입출력 방식 지원 |
| **확장 가능** | 새로운 기기/기능 추가 용이 |
| **프라이버시** | 민감 정보 최소 수집 |

### 1.2 Specification Version

```
Version: 1.0.0
Status: Draft
Date: 2025-12-16
```

---

## 2. Core Entities

### 2.1 Entity Relationship

```
┌─────────────────┐      ┌─────────────────┐
│  UserProfile    │──────│AccessibilityReq │
└────────┬────────┘      └─────────────────┘
         │
         │ has
         ▼
┌─────────────────┐      ┌─────────────────┐
│     Home        │──────│      Zone       │
└────────┬────────┘      └────────┬────────┘
         │                        │
         │ contains               │ contains
         ▼                        ▼
┌─────────────────┐      ┌─────────────────┐
│     Device      │──────│AccessibleDevice │
└────────┬────────┘      └─────────────────┘
         │
         │ triggers
         ▼
┌─────────────────┐      ┌─────────────────┐
│   Automation    │──────│  Notification   │
└─────────────────┘      └─────────────────┘
```

---

## 3. User Profile

### 3.1 Schema Overview

사용자의 접근성 요구사항과 선호도를 정의합니다.

```json
{
  "profile_id": "uuid",
  "version": "1.0.0",
  "personal_info": { ... },
  "accessibility_requirements": { ... },
  "interaction_preferences": { ... },
  "notification_preferences": { ... },
  "privacy_settings": { ... }
}
```

### 3.2 Accessibility Requirements

```json
{
  "accessibility_requirements": {
    "primary_disabilities": ["visual_blind", "motor_limited"],
    "severity_levels": {
      "visual": "severe",
      "motor": "moderate"
    },
    "assistive_technologies": [
      {
        "type": "screen_reader",
        "name": "VoiceOver",
        "platform": "ios"
      }
    ],
    "required_modalities": {
      "input": ["voice", "switch"],
      "output": ["audio_tts", "haptic"]
    }
  }
}
```

### 3.3 Disability Types

| 코드 | 설명 |
|------|------|
| `visual_blind` | 전맹 |
| `visual_low_vision` | 저시력 |
| `visual_color_blind` | 색각 이상 |
| `hearing_deaf` | 청각 장애 |
| `hearing_hard` | 난청 |
| `motor_limited` | 운동 제한 |
| `motor_tremor` | 떨림 |
| `motor_paralysis` | 마비 |
| `cognitive_learning` | 학습 장애 |
| `cognitive_memory` | 기억 장애 |
| `cognitive_attention` | 주의력 장애 |
| `speech` | 언어 장애 |

### 3.4 Interaction Modalities

**입력 (Input):**

| 코드 | 설명 | 장애 유형 |
|------|------|----------|
| `voice` | 음성 명령 | 시각, 운동 |
| `touch` | 터치스크린 | 청각 |
| `switch` | 외부 스위치 | 운동 |
| `gaze` | 시선 추적 | 운동 |
| `gesture` | 제스처 | 청각, 운동 |
| `bci` | 뇌-컴퓨터 인터페이스 | 중증 운동 |
| `sip_puff` | 불기/빨기 | 중증 운동 |

**출력 (Output):**

| 코드 | 설명 | 장애 유형 |
|------|------|----------|
| `visual_screen` | 화면 표시 | 청각 |
| `visual_led` | LED 표시등 | 청각 |
| `audio_tts` | 음성 합성 | 시각 |
| `audio_tone` | 비프/멜로디 | 시각 |
| `haptic` | 진동/촉각 | 시각, 청각 |
| `braille` | 점자 디스플레이 | 시각 |

---

## 4. Home & Zone

### 4.1 Home Schema

```json
{
  "home_id": "uuid",
  "name": "My Home",
  "address": { ... },
  "timezone": "Asia/Seoul",
  "owner_profile_id": "uuid",
  "member_profiles": ["uuid"],
  "zones": ["uuid"],
  "accessibility_settings": {
    "default_modalities": {
      "input": ["voice"],
      "output": ["audio_tts", "visual_screen"]
    },
    "emergency_contacts": [...]
  }
}
```

### 4.2 Zone Schema

방/공간별 설정을 정의합니다.

```json
{
  "zone_id": "uuid",
  "home_id": "uuid",
  "name": "Living Room",
  "type": "living_room",
  "floor": 1,
  "devices": ["uuid"],
  "accessibility_overrides": {
    "volume_level": 80,
    "brightness_level": 70,
    "announcement_enabled": true
  }
}
```

### 4.3 Zone Types

```
bedroom, living_room, kitchen, bathroom,
office, hallway, entrance, garage, outdoor
```

---

## 5. Device & Accessibility Features

### 5.1 Device Schema

```json
{
  "device_id": "uuid",
  "matter_node_id": "optional",
  "vendor_id": "0x1234",
  "product_id": "0x5678",
  "device_type": "light",
  "name": "Living Room Light",
  "zone_id": "uuid",
  "status": "online",
  "capabilities": { ... },
  "accessibility_features": { ... }
}
```

### 5.2 Device Types

| 카테고리 | 타입 | 설명 |
|---------|------|------|
| **Lighting** | `light`, `light_dimmer`, `light_color` | 조명 |
| **Climate** | `thermostat`, `fan`, `air_purifier` | 환경 제어 |
| **Security** | `lock`, `doorbell`, `camera`, `alarm` | 보안 |
| **Cover** | `blind`, `curtain`, `garage_door` | 커버/도어 |
| **Sensor** | `motion`, `contact`, `temperature`, `humidity` | 센서 |
| **Media** | `speaker`, `tv`, `media_player` | 미디어 |
| **Appliance** | `outlet`, `switch`, `vacuum`, `washer` | 가전 |

### 5.3 Accessibility Features

```json
{
  "accessibility_features": {
    "supported_inputs": ["voice", "touch", "switch"],
    "supported_outputs": ["visual_led", "audio_tone", "haptic"],
    "voice_commands": [
      {
        "command": "turn on",
        "aliases": ["켜", "불 켜", "on"],
        "action": "set_power",
        "parameters": { "state": true }
      }
    ],
    "audio_feedback": {
      "enabled": true,
      "volume": 70,
      "tts_voice": "ko-KR-Standard-A",
      "tones": {
        "on": "tone_positive",
        "off": "tone_neutral",
        "error": "tone_negative"
      }
    },
    "visual_feedback": {
      "led_indicators": true,
      "high_contrast": true,
      "large_icons": true
    },
    "haptic_feedback": {
      "enabled": true,
      "pattern": "double_tap"
    },
    "timing": {
      "response_timeout_ms": 5000,
      "confirmation_required": false,
      "dwell_time_ms": 1000
    }
  }
}
```

---

## 6. Automation

### 6.1 Automation Rule Schema

```json
{
  "automation_id": "uuid",
  "name": "Night Mode",
  "enabled": true,
  "home_id": "uuid",
  "trigger": { ... },
  "conditions": [...],
  "actions": [...],
  "accessibility_settings": {
    "announce_activation": true,
    "confirmation_required": false,
    "override_priority": "low"
  }
}
```

### 6.2 Trigger Types

```json
{
  "trigger": {
    "type": "time",
    "config": {
      "time": "22:00",
      "days": ["mon", "tue", "wed", "thu", "fri"]
    }
  }
}
```

| 트리거 타입 | 설명 |
|------------|------|
| `time` | 시간 기반 |
| `sunrise`/`sunset` | 일출/일몰 |
| `device_state` | 기기 상태 변화 |
| `sensor_value` | 센서 값 임계치 |
| `location` | 위치 (집 도착/떠남) |
| `voice_command` | 음성 명령 |
| `manual` | 수동 트리거 |

### 6.3 Action Types

```json
{
  "actions": [
    {
      "type": "device_control",
      "device_id": "uuid",
      "command": "set_brightness",
      "parameters": { "level": 30 }
    },
    {
      "type": "notification",
      "message": "Night mode activated",
      "modalities": ["audio_tts", "visual_screen"]
    }
  ]
}
```

---

## 7. Notification

### 7.1 Notification Schema

```json
{
  "notification_id": "uuid",
  "type": "alert",
  "priority": "high",
  "source": {
    "device_id": "uuid",
    "device_type": "doorbell"
  },
  "message": {
    "default": "Someone is at the door",
    "ko": "현관에 방문자가 있습니다",
    "en": "Someone is at the door"
  },
  "delivery": {
    "modalities": ["audio_tts", "visual_screen", "haptic"],
    "audio": {
      "tts_text": "현관에 방문자가 있습니다",
      "tone": "doorbell_ring"
    },
    "visual": {
      "icon": "doorbell",
      "color": "#FFA500",
      "flash_pattern": "pulse"
    },
    "haptic": {
      "pattern": "long_buzz",
      "repeat": 3
    }
  },
  "actions": [
    {
      "id": "view",
      "label": "View Camera",
      "command": "open_camera"
    },
    {
      "id": "unlock",
      "label": "Unlock Door",
      "command": "unlock",
      "confirmation_required": true
    }
  ],
  "timestamp": "2025-12-16T10:30:00Z",
  "expires_at": "2025-12-16T10:35:00Z"
}
```

### 7.2 Notification Types

| 타입 | 설명 | 기본 우선순위 |
|------|------|--------------|
| `alert` | 즉각 주의 필요 | high |
| `warning` | 경고 | medium |
| `info` | 정보 | low |
| `reminder` | 리마인더 | low |
| `emergency` | 비상 | critical |

### 7.3 Priority Levels

| 우선순위 | 동작 |
|---------|------|
| `critical` | 모든 모달리티 즉시 전달, 반복 |
| `high` | 모든 모달리티 즉시 전달 |
| `medium` | 사용자 설정에 따라 전달 |
| `low` | 조용한 알림 |

---

## 8. Accessibility Event

### 8.1 Event Schema

```json
{
  "event_id": "uuid",
  "event_type": "accessibility_feature_used",
  "timestamp": "2025-12-16T10:30:00Z",
  "user_id": "uuid",
  "device_id": "uuid",
  "details": {
    "feature": "voice_command",
    "command": "turn on living room light",
    "success": true,
    "response_time_ms": 450
  }
}
```

### 8.2 Event Types

| 이벤트 타입 | 설명 |
|-----------|------|
| `accessibility_feature_used` | 접근성 기능 사용 |
| `modality_switched` | 입출력 모달리티 변경 |
| `assistance_requested` | 도움 요청 |
| `emergency_triggered` | 비상 상황 |
| `accessibility_error` | 접근성 관련 오류 |

---

## 9. Matter Cluster Mapping

### 9.1 Standard Cluster Mapping

| WIA 기능 | Matter Cluster |
|---------|----------------|
| Light Control | On/Off, Level Control, Color Control |
| Lock Control | Door Lock |
| Climate Control | Thermostat, Fan Control |
| Sensor Data | Temperature, Humidity, Occupancy |
| Media Control | Media Playback |

### 9.2 Proposed Accessibility Cluster

```
Cluster: Accessibility (0x0XXX)
  Attributes:
    - SupportedInputModalities (list<InputModality>)
    - SupportedOutputModalities (list<OutputModality>)
    - ActiveInputModality (InputModality)
    - ActiveOutputModality (OutputModality)
    - TTSEnabled (bool)
    - TTSVolume (uint8)
    - HapticEnabled (bool)
    - HighContrastEnabled (bool)
    - LargeTextEnabled (bool)
    - ResponseTimeoutMs (uint32)

  Commands:
    - SetInputModality(modality)
    - SetOutputModality(modality)
    - AnnounceText(text, language)
    - TriggerHaptic(pattern)
    - SetAccessibilityProfile(profile_id)

  Events:
    - AccessibilityFeatureActivated
    - ModalitySwitched
    - AssistanceRequested
```

---

## 10. JSON Schema Files

### 10.1 Schema File Structure

```
smarthome/spec/schemas/
├── user-profile.schema.json
├── accessibility-requirements.schema.json
├── home.schema.json
├── zone.schema.json
├── device.schema.json
├── accessible-device.schema.json
├── automation.schema.json
├── notification.schema.json
└── accessibility-event.schema.json
```

### 10.2 Schema Versioning

- Major: 호환성 변경
- Minor: 기능 추가 (하위 호환)
- Patch: 버그 수정

---

## 11. 弘益人間

이 표준은 모든 사람이 스마트홈의 편의를 누릴 수 있도록 설계되었습니다.
장애는 기술의 장벽이 아니라, 혁신의 기회입니다.

---

*WIA Smart Home Accessibility Standard v1.0.0*
*弘益人間 - 널리 인간을 이롭게 하다*
