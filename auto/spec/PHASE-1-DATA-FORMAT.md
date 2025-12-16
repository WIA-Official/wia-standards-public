# WIA Autonomous Vehicle Accessibility - Phase 1: Data Format Standard

## 1. Overview

이 문서는 WIA 자율주행차 접근성 표준의 데이터 형식을 정의합니다.
장애인 승객, 차량 기능, 경로 요청, HMI 설정 등의 데이터 구조를 표준화합니다.

### 1.1 Scope

- 승객 접근성 프로파일
- 차량 접근성 기능
- 경로 요청 및 응답
- HMI 구성
- 휠체어 고정 상태
- 비상 상황 처리

### 1.2 Compliance

이 표준은 다음과 연동됩니다:
- SAE J3016 (Levels of Driving Automation)
- 49 CFR Part 38 (ADA Accessibility Specifications)
- SAE J2092/J2093/J2094 (Wheelchair Standards)

---

## 2. Core Data Types

### 2.1 Disability Types

```typescript
enum DisabilityType {
  VISUAL_BLIND = "visual_blind",
  VISUAL_LOW_VISION = "visual_low_vision",
  HEARING_DEAF = "hearing_deaf",
  HEARING_HARD = "hearing_hard",
  MOBILITY_WHEELCHAIR_MANUAL = "mobility_wheelchair_manual",
  MOBILITY_WHEELCHAIR_POWER = "mobility_wheelchair_power",
  MOBILITY_WALKER = "mobility_walker",
  MOBILITY_CANE = "mobility_cane",
  MOBILITY_CRUTCHES = "mobility_crutches",
  COGNITIVE_INTELLECTUAL = "cognitive_intellectual",
  COGNITIVE_AUTISM = "cognitive_autism",
  COGNITIVE_DEMENTIA = "cognitive_dementia",
  SPEECH = "speech",
  MULTIPLE = "multiple"
}
```

### 2.2 SAE Automation Levels

```typescript
enum SAELevel {
  LEVEL_0 = 0,  // No Automation
  LEVEL_1 = 1,  // Driver Assistance
  LEVEL_2 = 2,  // Partial Automation
  LEVEL_3 = 3,  // Conditional Automation
  LEVEL_4 = 4,  // High Automation
  LEVEL_5 = 5   // Full Automation
}
```

### 2.3 Interaction Modalities

```typescript
enum InteractionModality {
  VISUAL_SCREEN = "visual_screen",
  VISUAL_LED = "visual_led",
  AUDIO_TTS = "audio_tts",
  AUDIO_CHIME = "audio_chime",
  AUDIO_SPEECH_REC = "audio_speech_rec",
  HAPTIC_VIBRATION = "haptic_vibration",
  HAPTIC_FORCE = "haptic_force",
  BRAILLE = "braille",
  PHYSICAL_BUTTON = "physical_button"
}
```

---

## 3. Passenger Accessibility Profile

### 3.1 Schema

```json
{
  "profile_id": "uuid",
  "version": "1.0.0",
  "passenger": {
    "name": "string (optional)",
    "disabilities": ["DisabilityType"],
    "service_animal": {
      "has_animal": "boolean",
      "animal_type": "string",
      "animal_size": "small | medium | large"
    }
  },
  "mobility_aid": {
    "type": "none | manual_wheelchair | power_wheelchair | scooter | walker | cane | crutches",
    "dimensions": {
      "width_cm": "number",
      "length_cm": "number",
      "height_cm": "number",
      "weight_kg": "number"
    },
    "requires_ramp": "boolean",
    "requires_lift": "boolean",
    "requires_securement": "boolean"
  },
  "communication": {
    "preferred_language": "string (ISO 639-1)",
    "preferred_modalities": ["InteractionModality"],
    "aac_device": "boolean",
    "sign_language": "string | null"
  },
  "assistance": {
    "needs_boarding_help": "boolean",
    "needs_securement_help": "boolean",
    "companion_present": "boolean",
    "emergency_contact": {
      "name": "string",
      "phone": "string",
      "relationship": "string"
    }
  },
  "preferences": {
    "audio_volume": "number (0-100)",
    "screen_brightness": "number (0-100)",
    "high_contrast": "boolean",
    "large_text": "boolean",
    "haptic_intensity": "number (0-100)",
    "minimize_walking": "boolean"
  }
}
```

### 3.2 Example

```json
{
  "profile_id": "550e8400-e29b-41d4-a716-446655440000",
  "version": "1.0.0",
  "passenger": {
    "disabilities": ["visual_blind"],
    "service_animal": {
      "has_animal": true,
      "animal_type": "guide_dog",
      "animal_size": "large"
    }
  },
  "mobility_aid": {
    "type": "none",
    "requires_ramp": false,
    "requires_lift": false,
    "requires_securement": false
  },
  "communication": {
    "preferred_language": "ko",
    "preferred_modalities": ["audio_tts", "braille", "haptic_vibration"],
    "aac_device": false
  },
  "assistance": {
    "needs_boarding_help": true,
    "needs_securement_help": false,
    "companion_present": false,
    "emergency_contact": {
      "name": "김철수",
      "phone": "+82-10-1234-5678",
      "relationship": "spouse"
    }
  },
  "preferences": {
    "audio_volume": 80,
    "haptic_intensity": 70,
    "minimize_walking": true
  }
}
```

---

## 4. Vehicle Accessibility Capabilities

### 4.1 Schema

```json
{
  "vehicle_id": "uuid",
  "version": "1.0.0",
  "vehicle_info": {
    "make": "string",
    "model": "string",
    "year": "number",
    "sae_level": "SAELevel",
    "license_plate": "string"
  },
  "accessibility_features": {
    "entry": {
      "ramp": {
        "available": "boolean",
        "type": "fold | slide | deploy",
        "max_weight_kg": "number",
        "width_cm": "number",
        "auto_deploy": "boolean"
      },
      "lift": {
        "available": "boolean",
        "type": "platform | rotary",
        "max_weight_kg": "number",
        "width_cm": "number"
      },
      "doors": {
        "type": "swing | slide | gullwing",
        "width_cm": "number",
        "auto_open": "boolean",
        "low_step": "boolean",
        "step_height_cm": "number"
      }
    },
    "interior": {
      "wheelchair_spaces": "number",
      "wheelchair_securement": {
        "available": "boolean",
        "type": "manual | semi_auto | full_auto",
        "max_width_cm": "number",
        "max_length_cm": "number",
        "max_weight_kg": "number"
      },
      "transfer_seat": "boolean",
      "lowered_floor": "boolean",
      "floor_height_cm": "number",
      "headroom_cm": "number"
    },
    "hmi": {
      "screen": {
        "available": "boolean",
        "size_inches": "number",
        "touch": "boolean",
        "high_contrast": "boolean",
        "adjustable_brightness": "boolean"
      },
      "audio": {
        "tts": "boolean",
        "speech_recognition": "boolean",
        "languages": ["string"],
        "volume_control": "boolean",
        "chimes": "boolean"
      },
      "haptic": {
        "vibration": "boolean",
        "force_feedback": "boolean"
      },
      "physical_controls": {
        "braille_labels": "boolean",
        "tactile_buttons": "boolean",
        "panic_button": "boolean",
        "pull_over_button": "boolean"
      }
    },
    "communication": {
      "live_support": "boolean",
      "video_call": "boolean",
      "text_chat": "boolean",
      "sign_language_support": "boolean"
    }
  },
  "capacity": {
    "total_passengers": "number",
    "wheelchair_passengers": "number",
    "service_animals": "boolean"
  }
}
```

### 4.2 Example

```json
{
  "vehicle_id": "660e8400-e29b-41d4-a716-446655440001",
  "version": "1.0.0",
  "vehicle_info": {
    "make": "Waymo",
    "model": "Jaguar I-PACE",
    "year": 2024,
    "sae_level": 4,
    "license_plate": "AV-1234"
  },
  "accessibility_features": {
    "entry": {
      "ramp": {
        "available": false
      },
      "lift": {
        "available": false
      },
      "doors": {
        "type": "swing",
        "width_cm": 95,
        "auto_open": true,
        "low_step": true,
        "step_height_cm": 15
      }
    },
    "interior": {
      "wheelchair_spaces": 0,
      "wheelchair_securement": {
        "available": false
      },
      "transfer_seat": false,
      "lowered_floor": false
    },
    "hmi": {
      "screen": {
        "available": true,
        "size_inches": 10,
        "touch": true,
        "high_contrast": true,
        "adjustable_brightness": true
      },
      "audio": {
        "tts": true,
        "speech_recognition": true,
        "languages": ["en", "es", "ko", "zh"],
        "volume_control": true,
        "chimes": true
      },
      "haptic": {
        "vibration": true,
        "force_feedback": false
      },
      "physical_controls": {
        "braille_labels": true,
        "tactile_buttons": true,
        "panic_button": true,
        "pull_over_button": true
      }
    },
    "communication": {
      "live_support": true,
      "video_call": false,
      "text_chat": true,
      "sign_language_support": false
    }
  },
  "capacity": {
    "total_passengers": 4,
    "wheelchair_passengers": 0,
    "service_animals": true
  }
}
```

---

## 5. Trip Request

### 5.1 Schema

```json
{
  "request_id": "uuid",
  "version": "1.0.0",
  "timestamp": "ISO 8601",
  "passenger_profile_id": "uuid",
  "trip": {
    "pickup": {
      "location": {
        "latitude": "number",
        "longitude": "number",
        "address": "string"
      },
      "notes": "string",
      "pickup_side": "same_side | any",
      "curb_to_curb": "boolean"
    },
    "dropoff": {
      "location": {
        "latitude": "number",
        "longitude": "number",
        "address": "string"
      },
      "notes": "string"
    },
    "scheduled_time": "ISO 8601 | null",
    "flexibility_minutes": "number"
  },
  "accessibility_requirements": {
    "wheelchair_accessible": "boolean",
    "ramp_required": "boolean",
    "lift_required": "boolean",
    "service_animal_space": "boolean",
    "companion_space": "number",
    "preferred_modalities": ["InteractionModality"]
  },
  "preferences": {
    "minimize_walking": "boolean",
    "audio_guidance": "boolean",
    "visual_guidance": "boolean",
    "haptic_feedback": "boolean",
    "quiet_ride": "boolean"
  }
}
```

---

## 6. Trip Response

### 6.1 Schema

```json
{
  "response_id": "uuid",
  "request_id": "uuid",
  "version": "1.0.0",
  "timestamp": "ISO 8601",
  "status": "confirmed | no_vehicle | pending",
  "vehicle": {
    "vehicle_id": "uuid",
    "eta_minutes": "number",
    "distance_km": "number"
  },
  "accessibility_match": {
    "wheelchair_accessible": "boolean",
    "ramp_available": "boolean",
    "lift_available": "boolean",
    "service_animal_ok": "boolean",
    "modalities_supported": ["InteractionModality"],
    "match_score": "number (0-100)"
  },
  "wayfinding": {
    "pickup_instructions": "string",
    "audio_guidance_available": "boolean",
    "find_vehicle_features": ["horn", "lights", "melody"],
    "braille_identifier": "string"
  }
}
```

---

## 7. HMI Configuration

### 7.1 Schema

```json
{
  "config_id": "uuid",
  "version": "1.0.0",
  "active": "boolean",
  "visual": {
    "enabled": "boolean",
    "brightness": "number (0-100)",
    "contrast": "normal | high",
    "text_size": "small | medium | large | extra_large",
    "color_scheme": "default | high_contrast | dark | light",
    "animations": "boolean"
  },
  "audio": {
    "enabled": "boolean",
    "volume": "number (0-100)",
    "tts_voice": "string",
    "tts_speed": "slow | normal | fast",
    "chimes_enabled": "boolean",
    "speech_recognition": "boolean",
    "language": "string (ISO 639-1)"
  },
  "haptic": {
    "enabled": "boolean",
    "intensity": "number (0-100)",
    "patterns": {
      "notification": "boolean",
      "navigation": "boolean",
      "warning": "boolean",
      "confirmation": "boolean"
    }
  },
  "physical": {
    "braille_output": "boolean",
    "button_audio_feedback": "boolean",
    "button_haptic_feedback": "boolean"
  }
}
```

---

## 8. Wheelchair Securement Status

### 8.1 Schema

```json
{
  "status_id": "uuid",
  "version": "1.0.0",
  "timestamp": "ISO 8601",
  "vehicle_id": "uuid",
  "securement_points": [
    {
      "point_id": "string",
      "position": "front_left | front_right | rear_left | rear_right",
      "status": "disengaged | engaging | engaged | error",
      "force_newtons": "number",
      "locked": "boolean"
    }
  ],
  "overall_status": "unsecured | securing | secured | error",
  "wheelchair_detected": "boolean",
  "wheelchair_type": "manual | power | scooter",
  "safety_check_passed": "boolean",
  "ready_to_move": "boolean",
  "error_message": "string | null"
}
```

---

## 9. Emergency Event

### 9.1 Schema

```json
{
  "event_id": "uuid",
  "version": "1.0.0",
  "timestamp": "ISO 8601",
  "vehicle_id": "uuid",
  "trip_id": "uuid | null",
  "event_type": "panic_button | pull_over | collision | medical | fire | other",
  "severity": "low | medium | high | critical",
  "location": {
    "latitude": "number",
    "longitude": "number",
    "address": "string"
  },
  "passenger": {
    "profile_id": "uuid",
    "disabilities": ["DisabilityType"],
    "emergency_contact": {
      "name": "string",
      "phone": "string"
    }
  },
  "vehicle_status": {
    "speed_kmh": "number",
    "is_moving": "boolean",
    "doors_locked": "boolean",
    "securement_status": "secured | unsecured"
  },
  "response": {
    "auto_pulled_over": "boolean",
    "support_contacted": "boolean",
    "emergency_services_called": "boolean",
    "eta_support_minutes": "number | null"
  }
}
```

---

## 10. Data Exchange Protocol

### 10.1 Message Envelope

```json
{
  "wia_auto": {
    "version": "1.0.0",
    "message_id": "uuid",
    "timestamp": "ISO 8601",
    "source": "passenger_app | vehicle | fleet_mgmt | support",
    "destination": "passenger_app | vehicle | fleet_mgmt | support",
    "message_type": "profile | capabilities | trip_request | trip_response | hmi_config | securement | emergency",
    "payload": { }
  }
}
```

### 10.2 Message Types

| Type | Direction | Description |
|------|-----------|-------------|
| profile | App → Fleet | 승객 프로파일 전송 |
| capabilities | Vehicle → App | 차량 기능 조회 |
| trip_request | App → Fleet | 경로 요청 |
| trip_response | Fleet → App | 경로 응답 |
| hmi_config | App → Vehicle | HMI 설정 |
| securement | Vehicle → App | 고정 상태 |
| emergency | Vehicle → All | 비상 상황 |

---

## 11. Validation Rules

### 11.1 Required Fields

| Schema | Required Fields |
|--------|-----------------|
| PassengerProfile | profile_id, version, passenger.disabilities |
| VehicleCapabilities | vehicle_id, version, vehicle_info, accessibility_features |
| TripRequest | request_id, version, trip.pickup, trip.dropoff |
| SecurementStatus | status_id, version, overall_status |
| EmergencyEvent | event_id, version, event_type, severity, location |

### 11.2 Value Constraints

| Field | Constraint |
|-------|------------|
| sae_level | 0-5 |
| volume/brightness/intensity | 0-100 |
| latitude | -90 to 90 |
| longitude | -180 to 180 |
| weight_kg | > 0 |
| dimensions (cm) | > 0 |

---

## 12. Versioning

- **Major version** (1.x.x): Breaking changes
- **Minor version** (x.1.x): New features, backward compatible
- **Patch version** (x.x.1): Bug fixes

All messages MUST include `version` field.

---

## 13. Security Considerations

- All passenger data is PII (Personal Identifiable Information)
- Encryption required for transit and rest
- Access control by role
- Audit logging for all operations
- GDPR/HIPAA compliance considerations

---

## 14. Schema Files

JSON Schema 파일은 다음 위치에 있습니다:

```
/auto/spec/schemas/
├── passenger-profile.schema.json
├── vehicle-capabilities.schema.json
├── trip-request.schema.json
├── trip-response.schema.json
├── hmi-config.schema.json
├── securement-status.schema.json
├── emergency-event.schema.json
└── message-envelope.schema.json
```

---

*WIA Autonomous Vehicle Accessibility Standard*
*Phase 1: Data Format Standard v1.0.0*
*December 2024*
