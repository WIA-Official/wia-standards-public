# WIA Medical Device Accessibility: Phase 1 Data Format Specification

## 1. Overview

본 문서는 의료기기 접근성 데이터의 표준 형식을 정의합니다.

**Version:** 1.0.0
**Status:** Draft
**Phase:** 1 of 4

---

## 2. Core Data Structures

### 2.1 Medical Device Accessibility Profile

```typescript
interface MedicalDeviceAccessibilityProfile {
  // 프로필 식별
  profile_id: string;
  profile_version: string;
  created_at: string;              // ISO 8601
  updated_at: string;

  // 기기 정보
  device: MedicalDeviceInfo;

  // 접근성 기능
  accessibility: DeviceAccessibilityFeatures;

  // 규제 준수
  compliance: RegulatoryCompliance;

  // WIA 통합
  wia_integration?: WIAIntegration;

  // 메타데이터
  metadata: ProfileMetadata;
}
```

### 2.2 User Medical Accessibility Profile

```typescript
interface UserMedicalAccessibilityProfile {
  // 사용자 식별 (익명화)
  user_id: string;
  profile_version: string;

  // 장애/접근성 요구사항
  accessibility_needs: UserAccessibilityNeeds;

  // 감각 선호도
  sensory_preferences: SensoryPreferences;

  // 입력 선호도
  input_preferences: InputPreferences;

  // 인지 지원
  cognitive_support: CognitiveSupport;

  // 의료 컨텍스트
  medical_context?: MedicalContext;

  // WIA 기기 설정
  wia_devices?: WIADeviceSettings;
}
```

---

## 3. Medical Device Information

### 3.1 Device Info Schema

```typescript
interface MedicalDeviceInfo {
  // 기기 식별
  device_id: string;
  manufacturer: string;
  model: string;
  device_name: string;

  // FDA 분류
  fda_classification: {
    class: "I" | "II" | "III";
    product_code?: string;
    regulation_number?: string;
    clearance_type?: "510k" | "PMA" | "De_Novo" | "Exempt";
  };

  // 기기 유형
  device_type: MedicalDeviceType;
  device_category: DeviceCategory;

  // 사용 환경
  use_environment: "home" | "clinical" | "both";
  intended_users: IntendedUser[];

  // 인터페이스
  interface: DeviceInterface;

  // 연결성
  connectivity: DeviceConnectivity;
}

type MedicalDeviceType =
  | "diagnostic"           // 진단 장비
  | "monitoring"           // 모니터링 장비
  | "therapeutic"          // 치료 장비
  | "assistive"            // 보조 장비
  | "wearable"             // 웨어러블
  | "implantable"          // 이식형
  | "imaging"              // 영상 장비
  | "laboratory";          // 실험실 장비

type DeviceCategory =
  | "blood_glucose_monitor"
  | "blood_pressure_monitor"
  | "pulse_oximeter"
  | "thermometer"
  | "weight_scale"
  | "ecg_monitor"
  | "cgm"                  // 연속 혈당 모니터
  | "insulin_pump"
  | "cpap_bipap"
  | "hearing_aid"
  | "cochlear_implant"
  | "examination_table"
  | "imaging_equipment"
  | "infusion_pump"
  | "ventilator"
  | "defibrillator"
  | "other";

interface IntendedUser {
  type: "patient" | "caregiver" | "healthcare_professional";
  training_required: boolean;
  supervision_required: boolean;
}
```

### 3.2 Device Interface

```typescript
interface DeviceInterface {
  // 디스플레이
  display?: {
    type: "lcd" | "led" | "oled" | "e_ink" | "none";
    size_inches?: number;
    resolution?: Resolution;
    touch_enabled: boolean;
    color: boolean;
    brightness_adjustable: boolean;
    contrast_adjustable: boolean;
  };

  // 물리적 컨트롤
  physical_controls?: {
    buttons: ButtonInfo[];
    dials?: DialInfo[];
    switches?: SwitchInfo[];
  };

  // 오디오
  audio?: {
    speaker: boolean;
    speaker_volume_adjustable: boolean;
    microphone: boolean;
    audio_jack: boolean;
    bluetooth_audio: boolean;
  };

  // 햅틱
  haptic?: {
    vibration: boolean;
    intensity_levels: number;
    patterns_supported: boolean;
  };
}

interface ButtonInfo {
  id: string;
  label: string;
  tactile_marking: boolean;
  size_mm: number;
  force_required_grams: number;
  location: string;
}
```

### 3.3 Device Connectivity

```typescript
interface DeviceConnectivity {
  // 무선 연결
  wireless: {
    bluetooth?: {
      version: string;
      profiles: string[];
    };
    wifi?: {
      standards: string[];
    };
    cellular?: {
      technology: string;
    };
    nfc?: boolean;
  };

  // 유선 연결
  wired: {
    usb?: {
      type: string;
      version: string;
    };
    audio_jack?: boolean;
    proprietary?: string;
  };

  // 앱 연동
  companion_app?: {
    platforms: ("ios" | "android" | "windows" | "macos")[];
    accessibility_features: string[];
  };

  // 클라우드 연동
  cloud_integration?: {
    provider: string;
    data_sync: boolean;
    remote_monitoring: boolean;
  };
}
```

---

## 4. Accessibility Features

### 4.1 Device Accessibility Features

```typescript
interface DeviceAccessibilityFeatures {
  // 시각 접근성
  visual: VisualAccessibility;

  // 청각 접근성
  auditory: AuditoryAccessibility;

  // 운동/촉각 접근성
  motor: MotorAccessibility;

  // 인지 접근성
  cognitive: CognitiveAccessibility;

  // 물리적 접근성
  physical: PhysicalAccessibility;

  // 접근성 점수
  accessibility_score: AccessibilityScore;
}
```

### 4.2 Visual Accessibility

```typescript
interface VisualAccessibility {
  // 스크린 리더 지원
  screen_reader: {
    supported: boolean;
    level: "none" | "partial" | "full";
    protocols: ("talkback" | "voiceover" | "nvda" | "jaws" | "custom")[];
  };

  // 음성 출력
  voice_output: {
    supported: boolean;
    readings: VoiceReading[];
    languages: string[];
    speed_adjustable: boolean;
    volume_adjustable: boolean;
  };

  // 디스플레이 접근성
  display_accessibility: {
    high_contrast_mode: boolean;
    large_text_mode: boolean;
    text_size_adjustable: boolean;
    color_inversion: boolean;
    color_blind_modes: ColorBlindMode[];
  };

  // 비시각적 대안
  non_visual_alternatives: {
    audio_feedback: boolean;
    haptic_feedback: boolean;
    braille_output: boolean;
  };
}

interface VoiceReading {
  data_type: string;         // "blood_glucose", "blood_pressure", etc.
  format: string;            // 읽기 형식
  units_spoken: boolean;
  range_indication: boolean; // 정상/비정상 표시
}

type ColorBlindMode =
  | "protanopia"
  | "deuteranopia"
  | "tritanopia"
  | "achromatopsia";
```

### 4.3 Auditory Accessibility

```typescript
interface AuditoryAccessibility {
  // 시각적 알림
  visual_alerts: {
    supported: boolean;
    types: VisualAlertType[];
    customizable: boolean;
  };

  // 햅틱 알림
  haptic_alerts: {
    supported: boolean;
    patterns: HapticPattern[];
    intensity_levels: number;
  };

  // 오디오 조정
  audio_adjustments: {
    volume_range: [number, number];      // dB
    frequency_adjustable: boolean;
    mono_audio: boolean;
    hearing_aid_compatible: boolean;
    t_coil_compatible: boolean;
  };

  // 자막/텍스트
  text_alternatives: {
    on_screen_text: boolean;
    closed_captions: boolean;
    real_time_transcription: boolean;
  };
}

type VisualAlertType =
  | "screen_flash"
  | "led_indicator"
  | "icon_display"
  | "text_notification"
  | "color_change";

interface HapticPattern {
  id: string;
  name: string;
  meaning: string;           // "alarm", "success", "error", etc.
  duration_ms: number;
  intensity: number;
}
```

### 4.4 Motor Accessibility

```typescript
interface MotorAccessibility {
  // 대체 입력
  alternative_input: {
    voice_control: boolean;
    switch_access: boolean;
    eye_tracking: boolean;
    head_tracking: boolean;
    wia_exoskeleton: boolean;
  };

  // 물리적 컨트롤 접근성
  physical_controls: {
    large_buttons: boolean;
    button_spacing_adequate: boolean;
    low_force_buttons: boolean;
    one_handed_operation: boolean;
    no_fine_motor_required: boolean;
  };

  // 터치스크린 접근성
  touchscreen: {
    gesture_alternatives: boolean;
    touch_accommodation: boolean;
    dwell_control: boolean;
    haptic_feedback: boolean;
  };

  // 자동화
  automation: {
    auto_measurement: boolean;
    scheduled_operation: boolean;
    remote_control: boolean;
  };
}
```

### 4.5 Cognitive Accessibility

```typescript
interface CognitiveAccessibility {
  // 단순화된 인터페이스
  simplified_interface: {
    available: boolean;
    reduced_options: boolean;
    step_by_step_guidance: boolean;
    clear_icons: boolean;
  };

  // 메모리 지원
  memory_support: {
    reminders: boolean;
    history_log: boolean;
    caregiver_notifications: boolean;
    auto_data_sync: boolean;
  };

  // 오류 방지
  error_prevention: {
    confirmation_prompts: boolean;
    undo_capability: boolean;
    clear_error_messages: boolean;
    recovery_guidance: boolean;
  };

  // 언어
  language_support: {
    languages: string[];
    simple_language_mode: boolean;
    icon_based_navigation: boolean;
    pictogram_support: boolean;
  };
}
```

### 4.6 Physical Accessibility (MDE Standards)

```typescript
interface PhysicalAccessibility {
  // 이동 높이 (MDE Standards)
  transfer_height: {
    adjustable: boolean;
    minimum_height_inches: number;
    maximum_height_inches: number;
    mde_compliant: boolean;          // 17인치 준수
  };

  // 휠체어 접근성
  wheelchair_access: {
    accessible: boolean;
    clear_floor_space: boolean;
    approach_type: "front" | "side" | "both";
    knee_clearance: boolean;
  };

  // 물리적 치수
  dimensions: {
    weight_kg: number;
    portable: boolean;
    height_adjustable: boolean;
    tilt_adjustable: boolean;
  };

  // 지지대
  supports: {
    armrests: boolean;
    grab_bars: boolean;
    head_support: boolean;
    leg_support: boolean;
  };
}
```

---

## 5. Regulatory Compliance

### 5.1 Compliance Schema

```typescript
interface RegulatoryCompliance {
  // FDA 준수
  fda: {
    registered: boolean;
    clearance_number?: string;
    human_factors_validated: boolean;
    accessibility_tested: boolean;
  };

  // MDE Standards 준수
  mde_standards: {
    compliant: boolean;
    transfer_height_compliant: boolean;
    last_assessment_date?: string;
  };

  // ADA 준수
  ada: {
    title_ii_compliant: boolean;
    title_iii_compliant: boolean;
  };

  // 국제 표준
  international: {
    iec_62366: boolean;             // 사용성 공학
    iso_14971: boolean;             // 위험 관리
    en_301_549: boolean;            // EU ICT 접근성
    mdr_compliant: boolean;         // EU MDR
  };

  // WIA 인증
  wia_certification?: {
    level: "bronze" | "silver" | "gold" | "platinum";
    certificate_id: string;
    valid_until: string;
  };

  // 접근성 적합성 선언
  accessibility_conformance: {
    vpat_available: boolean;        // Voluntary Product Accessibility Template
    vpat_url?: string;
    wcag_level?: "A" | "AA" | "AAA";
    section_508_compliant: boolean;
  };
}
```

---

## 6. Alarm and Alert System

### 6.1 Multi-Sensory Alarm Schema

```typescript
interface MedicalAlarmSystem {
  // 알람 카테고리
  alarm_categories: AlarmCategory[];

  // 다중 감각 출력
  output_modalities: {
    visual: VisualAlarmConfig;
    auditory: AuditoryAlarmConfig;
    haptic: HapticAlarmConfig;
  };

  // 접근성 설정
  accessibility_settings: {
    allow_single_modality: boolean;
    minimum_modalities: number;
    user_configurable: boolean;
  };

  // 에스컬레이션
  escalation: {
    enabled: boolean;
    timeout_seconds: number;
    escalation_steps: EscalationStep[];
  };
}

interface AlarmCategory {
  id: string;
  name: string;
  priority: "low" | "medium" | "high" | "critical";

  // 의미
  meaning: string;
  recommended_action: string;

  // 각 모달리티 설정
  visual: {
    color: string;
    pattern: "solid" | "flashing" | "pulsing";
    icon: string;
    text: string;
  };

  auditory: {
    tone_frequency_hz: number;
    pattern: "continuous" | "intermittent" | "escalating";
    volume_db: number;
    voice_announcement?: string;
  };

  haptic: {
    pattern_id: string;
    intensity: number;
    duration_ms: number;
  };
}

interface EscalationStep {
  delay_seconds: number;
  action: "repeat" | "increase_intensity" | "notify_caregiver" | "emergency_call";
}
```

---

## 7. WIA Integration

### 7.1 WIA Integration Schema

```typescript
interface WIAIntegration {
  // 지원 프로토콜
  supported_protocols: {
    exoskeleton: boolean;
    bionic_eye: boolean;
    voice_sign: boolean;
    haptic: boolean;
    smart_wheelchair: boolean;
  };

  // 외골격 통합
  exoskeleton?: {
    haptic_feedback_mapping: HapticMapping[];
    motion_assistance: boolean;
    rehabilitation_mode: boolean;
  };

  // 생체 눈 통합
  bionic_eye?: {
    display_optimization: boolean;
    contrast_enhancement: boolean;
    pattern_simplification: boolean;
  };

  // 음성-수화 통합
  voice_sign?: {
    medical_terminology_support: boolean;
    real_time_translation: boolean;
    emergency_phrases: string[];
  };

  // 햅틱 통합
  haptic?: {
    alarm_mapping: HapticAlarmMapping[];
    data_haptic_encoding: boolean;
  };

  // 스마트 휠체어 통합
  smart_wheelchair?: {
    device_positioning: boolean;
    height_adjustment_sync: boolean;
  };
}

interface HapticMapping {
  device_event: string;
  haptic_pattern: string;
  target: "left_hand" | "right_hand" | "torso" | "custom";
  intensity: number;
}

interface HapticAlarmMapping {
  alarm_priority: string;
  haptic_pattern: string;
  body_location: string;
}
```

---

## 8. Data Exchange

### 8.1 Medical Data Accessibility

```typescript
interface MedicalDataAccessibility {
  // 데이터 형식
  data_format: {
    standards: ("HL7_FHIR" | "HL7_V2" | "CDA" | "proprietary")[];
    accessibility_extensions: boolean;
  };

  // 접근 가능한 출력
  accessible_output: {
    formats: AccessibleDataFormat[];
    export_options: ExportOption[];
  };

  // 데이터 읽기
  data_reading: {
    voice_reading: boolean;
    screen_reader_compatible: boolean;
    braille_export: boolean;
  };

  // 시각화 접근성
  visualization: {
    high_contrast_charts: boolean;
    pattern_differentiation: boolean;  // 색상 대신 패턴
    alt_text_for_charts: boolean;
    data_table_alternative: boolean;
  };
}

interface AccessibleDataFormat {
  format: "text" | "audio" | "haptic_encoding" | "braille";
  description: string;
}

interface ExportOption {
  format: "pdf" | "csv" | "json" | "audio_summary";
  accessibility_features: string[];
}
```

---

## 9. User Accessibility Needs

### 9.1 User Needs Schema

```typescript
interface UserAccessibilityNeeds {
  // 감각 장애
  sensory: {
    visual: {
      level: "none" | "low_vision" | "legally_blind" | "totally_blind";
      color_blind?: ColorBlindMode;
      light_sensitivity?: boolean;
    };

    auditory: {
      level: "none" | "hard_of_hearing" | "deaf";
      uses_hearing_aid?: boolean;
      uses_cochlear_implant?: boolean;
    };
  };

  // 운동 장애
  motor: {
    upper_limb: {
      level: "none" | "mild" | "moderate" | "severe";
      affected_side?: "left" | "right" | "both";
      tremor?: boolean;
    };

    fine_motor: {
      level: "none" | "mild" | "moderate" | "severe";
    };

    mobility: {
      level: "ambulatory" | "assisted" | "wheelchair" | "bed_bound";
    };
  };

  // 인지
  cognitive: {
    memory: "normal" | "mild_impairment" | "moderate_impairment";
    attention: "normal" | "limited";
    reading_level?: "basic" | "intermediate" | "advanced";
    language: string;
  };

  // 의료 조건
  medical_conditions?: {
    condition: string;
    relevance: string;
  }[];
}
```

### 9.2 Sensory Preferences

```typescript
interface SensoryPreferences {
  // 시각 선호
  visual: {
    text_size: "small" | "medium" | "large" | "extra_large";
    high_contrast: boolean;
    dark_mode: boolean;
    color_scheme?: string;
    reduce_motion: boolean;
  };

  // 청각 선호
  auditory: {
    volume_level: number;              // 0-100
    prefer_voice: boolean;
    voice_speed: number;               // 0.5 - 2.0
    voice_pitch: "low" | "medium" | "high";
    prefer_tones: boolean;
    tone_frequency_preference?: number;
  };

  // 햅틱 선호
  haptic: {
    enabled: boolean;
    intensity: number;                 // 0-100
    prefer_haptic_over_audio: boolean;
    wia_haptic_device?: string;
  };
}
```

---

## 10. Profile Metadata

### 10.1 Metadata Schema

```typescript
interface ProfileMetadata {
  // 프로필 정보
  profile_type: "device" | "user" | "combination";
  schema_version: string;
  language: string;

  // 생성/수정
  created_by: string;
  created_at: string;
  updated_at: string;
  update_history?: UpdateRecord[];

  // 검증
  validation: {
    validated: boolean;
    validator?: string;
    validation_date?: string;
  };

  // 태그
  tags?: string[];
  notes?: string;
}

interface UpdateRecord {
  timestamp: string;
  updated_by: string;
  changes: string[];
}
```

---

## 11. JSON Schema Files

### 11.1 Schema Index

| Schema File | Description |
|-------------|-------------|
| `medical-device-profile.schema.json` | 의료기기 접근성 프로필 |
| `user-medical-profile.schema.json` | 사용자 의료 접근성 프로필 |
| `device-info.schema.json` | 기기 정보 |
| `accessibility-features.schema.json` | 접근성 기능 |
| `alarm-system.schema.json` | 다중 감각 알람 시스템 |
| `wia-integration.schema.json` | WIA 통합 |
| `regulatory-compliance.schema.json` | 규제 준수 |

---

## 12. Example Profiles

### 12.1 Blood Glucose Monitor Example

```json
{
  "profile_id": "bgm-example-001",
  "profile_version": "1.0.0",
  "device": {
    "device_id": "dexcom-g7-001",
    "manufacturer": "Dexcom",
    "model": "G7",
    "device_name": "Dexcom G7 CGM System",
    "fda_classification": {
      "class": "II",
      "clearance_type": "510k"
    },
    "device_type": "monitoring",
    "device_category": "cgm",
    "use_environment": "home"
  },
  "accessibility": {
    "visual": {
      "screen_reader": {
        "supported": true,
        "level": "full",
        "protocols": ["voiceover", "talkback"]
      },
      "voice_output": {
        "supported": true,
        "readings": [
          {
            "data_type": "blood_glucose",
            "format": "{value} milligrams per deciliter",
            "units_spoken": true,
            "range_indication": true
          }
        ]
      }
    },
    "auditory": {
      "visual_alerts": {
        "supported": true,
        "types": ["screen_flash", "icon_display"]
      },
      "haptic_alerts": {
        "supported": true,
        "patterns": [
          {
            "id": "urgent_low",
            "name": "Urgent Low Alert",
            "meaning": "Blood glucose critically low",
            "intensity": 1.0
          }
        ]
      }
    }
  },
  "wia_integration": {
    "supported_protocols": {
      "haptic": true,
      "voice_sign": true
    },
    "haptic": {
      "alarm_mapping": [
        {
          "alarm_priority": "critical",
          "haptic_pattern": "urgent_pulse",
          "body_location": "wrist"
        }
      ]
    }
  }
}
```

---

## 13. References

- IEC 62366-1:2015 Medical devices — Usability engineering
- ISO 14971:2019 Medical devices — Risk management
- U.S. Access Board MDE Standards (36 CFR 1195)
- FDA Human Factors Guidance
- ADA Title II/III Requirements
- WIA Accessibility Framework v1.0
