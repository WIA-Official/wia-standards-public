# WIA XR Accessibility: Safety Protocol Specification

## 1. Overview

본 문서는 XR(VR/AR/MR) 접근성 시스템의 안전 프로토콜 표준을 정의합니다.

**Version:** 1.0.0
**Status:** Draft
**Phase:** 3 - Safety & Quality Protocol

---

## 2. VR Sickness Prevention

### 2.1 Motion Sickness Risk Factors

| Factor | Risk Level | Mitigation |
|--------|------------|------------|
| **Locomotion Type** | High | 텔레포트/스냅턴 기본 제공 |
| **Frame Rate** | Critical | 최소 72fps 유지 |
| **Latency** | Critical | < 20ms 모션-투-포톤 |
| **FOV Mismatch** | Medium | 사용자 IPD 맞춤 설정 |
| **Vection** | High | 비네트 효과 적용 |
| **Acceleration** | High | 점진적 가속/감속 |

### 2.2 Comfort Vignette System

```typescript
interface ComfortVignetteConfig {
  // 활성화 조건
  triggers: {
    onMovement: boolean;
    onRotation: boolean;
    onAcceleration: boolean;
    onLowFrameRate: boolean;
  };

  // 비네트 설정
  settings: {
    baseIntensity: number;     // 0.0 - 1.0
    dynamicScaling: boolean;   // 속도에 따른 조절
    fadeInTime: number;        // ms
    fadeOutTime: number;       // ms
    shape: 'circular' | 'rectangular' | 'custom';
  };

  // 사용자 프로필 기반
  adaptiveSettings: {
    sensitivityLevel: 'low' | 'medium' | 'high' | 'extreme';
    historyBasedAdjustment: boolean;
  };
}
```

### 2.3 Movement Safety Presets

```json
{
  "movement_presets": {
    "comfortable": {
      "description": "VR 초보자 및 민감한 사용자용",
      "locomotion": "teleport_only",
      "rotation": "snap_turn",
      "snap_angle_degrees": 45,
      "vignette_intensity": 0.8,
      "max_speed": 2.0,
      "acceleration_smoothing": 0.9
    },
    "moderate": {
      "description": "일반 사용자용",
      "locomotion": "smooth_optional",
      "rotation": "smooth_optional",
      "snap_angle_degrees": 30,
      "vignette_intensity": 0.5,
      "max_speed": 4.0,
      "acceleration_smoothing": 0.7
    },
    "unrestricted": {
      "description": "경험자용 (자기 책임)",
      "locomotion": "smooth",
      "rotation": "smooth",
      "vignette_intensity": 0.0,
      "max_speed": 10.0,
      "acceleration_smoothing": 0.3,
      "requires_acknowledgment": true
    }
  }
}
```

---

## 3. Photosensitive Epilepsy Protection

### 3.1 Flash Detection System

```typescript
interface FlashDetectionConfig {
  // 감지 기준 (WCAG 2.1 기반)
  thresholds: {
    // 일반 플래시: 1초에 3회 이상 금지
    generalFlashHz: 3;

    // 붉은 플래시: 더 엄격한 제한
    redFlashHz: 1;

    // 화면 비율: 전체의 25% 이상 플래시 금지
    screenAreaThreshold: 0.25;

    // 휘도 변화: 0.8 cd/m² 이상 금지
    luminanceChangeThreshold: 0.8;
  };

  // 감지 방법
  detection: {
    realTimeAnalysis: boolean;
    preRenderCheck: boolean;
    contentMetadata: boolean;
  };

  // 대응 조치
  response: {
    autoBlock: boolean;
    userWarning: boolean;
    alternativeContent: boolean;
    emergencyDim: boolean;  // 즉시 화면 어둡게
  };
}
```

### 3.2 Safe Visual Parameters

| Parameter | Safe Limit | Action if Exceeded |
|-----------|------------|-------------------|
| Flash Rate | < 3 Hz | 자동 차단 |
| Red Flash | < 1 Hz | 즉시 차단 |
| Contrast Ratio | < 4:1 급격한 변화 | 완화 적용 |
| Pattern Frequency | 피해야 할 패턴 | 대체 렌더링 |
| Strobe Effects | 금지 | 비활성화 |

### 3.3 Content Warning System

```typescript
interface PhotosensitivityWarning {
  // 콘텐츠 등급
  rating: {
    safe: boolean;           // 완전 안전
    caution: boolean;        // 주의 필요
    warning: boolean;        // 경고 표시
    blocked: boolean;        // 차단됨
  };

  // 경고 메시지
  warnings: {
    preContentWarning: string;
    skipOption: boolean;
    alternativeAvailable: boolean;
    medicalDisclaimer: string;
  };

  // 사용자 설정
  userPreferences: {
    autoBlockFlashing: boolean;
    showWarningsAlways: boolean;
    reduceAllEffects: boolean;
    emergencyExitEnabled: boolean;
  };
}
```

---

## 4. Eye Strain Prevention

### 4.1 Session Time Monitoring

```typescript
interface SessionHealthMonitor {
  // 세션 추적
  tracking: {
    totalSessionTime: number;      // 분
    continuousUseTime: number;     // 휴식 없이 연속 사용
    blinkRateMonitor: boolean;     // 눈 깜빡임 감지
    gazePatternAnalysis: boolean;  // 시선 패턴 분석
  };

  // 휴식 알림
  breakReminders: {
    enabled: boolean;
    intervalMinutes: number;       // 기본: 30분
    breakDurationMinutes: number;  // 기본: 5분
    enforceBreak: boolean;         // 강제 휴식
    progressiveSeverity: boolean;  // 무시 시 경고 강화
  };

  // 건강 지표
  healthMetrics: {
    fatigueScore: number;          // 0-100
    strainIndicators: string[];
    recommendedAction: string;
  };
}
```

### 4.2 Eye Comfort Settings

```json
{
  "eye_comfort": {
    "display_settings": {
      "brightness_auto_adjust": true,
      "brightness_range": [0.3, 1.0],
      "blue_light_filter": {
        "enabled": true,
        "intensity": 0.3,
        "schedule_based": true
      },
      "contrast_optimization": true
    },
    "focus_settings": {
      "depth_of_field": "adaptive",
      "focal_distance_meters": 2.0,
      "vergence_accommodation_support": true
    },
    "rest_features": {
      "20_20_20_rule": true,
      "blink_reminders": true,
      "eye_exercise_prompts": false
    }
  }
}
```

### 4.3 Visual Fatigue Detection

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│  Eye Tracking   │────▶│  Fatigue        │────▶│  Intervention   │
│  Data           │     │  Analysis       │     │  Decision       │
└─────────────────┘     └─────────────────┘     └─────────────────┘
        │                       │                       │
        ▼                       ▼                       ▼
  [Blink Rate]           [Fatigue Score]          [Rest Prompt]
  [Gaze Pattern]         [Strain Level]           [Break Enforce]
  [Pupil Dilation]       [Duration Risk]          [Session End]
```

---

## 5. Physical Safety

### 5.1 Boundary/Guardian System

```typescript
interface BoundarySafetyConfig {
  // 경계 설정
  boundary: {
    enabled: boolean;
    type: 'stationary' | 'roomscale' | 'custom';
    warningDistance: number;      // meters
    hardStopDistance: number;     // meters
    floorDetection: boolean;
  };

  // 시각적 경고
  visualWarnings: {
    gridPattern: boolean;
    colorGradient: boolean;       // 거리에 따른 색상
    passthrough: boolean;         // 실제 환경 표시
    audioAlert: boolean;
  };

  // 접근성 대안
  accessibilityAlternatives: {
    audioOnlyWarning: boolean;    // 시각 장애인용
    hapticWarning: boolean;       // 청각 장애인용
    voiceAnnouncement: boolean;
    intensifiedVibration: boolean;
  };

  // 장애물 감지
  obstacleDetection: {
    enabled: boolean;
    realTimeScanning: boolean;
    warnOnNewObstacle: boolean;
  };
}
```

### 5.2 Seated/Standing Mode Safety

```typescript
interface PostureSafetyConfig {
  // 자세 모드
  mode: 'seated' | 'standing' | 'roomscale' | 'lying';

  // 자세별 설정
  seatedMode: {
    heightOffset: number;
    reachLimitations: boolean;
    noStandingPrompts: boolean;
    wheelchairCompatible: boolean;
  };

  standingMode: {
    fallDetection: boolean;
    stabilityMonitor: boolean;
    dizzinessCheck: boolean;
    emergencySitDown: boolean;
  };

  // 접근성
  accessibility: {
    oneHandedMode: boolean;
    limitedMobility: boolean;
    noReachAboveHead: boolean;
    noBending: boolean;
  };
}
```

### 5.3 Cable/Hardware Safety

```typescript
interface HardwareSafetyConfig {
  // 케이블 관리
  cableManagement: {
    cableDetection: boolean;
    directionIndicator: boolean;  // 케이블 위치 표시
    tangleWarning: boolean;
    wirelessPreferred: boolean;
  };

  // 과열 방지
  thermalSafety: {
    temperatureMonitor: boolean;
    overheatingWarning: number;   // 섭씨
    autoCooldown: boolean;
    ventilationReminder: boolean;
  };

  // 배터리 안전
  batterySafety: {
    lowBatteryWarning: number;    // 퍼센트
    criticalBatteryAction: 'warn' | 'pause' | 'save_and_exit';
    chargingInUseWarning: boolean;
  };
}
```

---

## 6. Emergency Exit System

### 6.1 Panic Exit Protocol

```typescript
interface EmergencyExitConfig {
  // 즉시 탈출
  panicExit: {
    enabled: boolean;
    trigger: 'button' | 'gesture' | 'voice' | 'all';
    buttonCombination: string[];   // 예: ['menu', 'trigger']
    voiceCommand: string[];        // 예: ['exit vr', 'help', 'stop']
    gestureType: string;           // 예: 'cover_eyes'
  };

  // 탈출 시 동작
  exitBehavior: {
    immediatePassthrough: boolean;
    fadeToBlack: boolean;
    audioConfirmation: boolean;
    hapticConfirmation: boolean;
    saveProgress: boolean;
  };

  // Safe Space 기능
  safeSpace: {
    enabled: boolean;
    environment: 'void' | 'calm_room' | 'nature' | 'custom';
    transitionTime: number;       // ms
    breathingExercise: boolean;
    exitFromSafeSpace: boolean;
  };
}
```

### 6.2 Accessibility Emergency Features

```json
{
  "emergency_accessibility": {
    "visual_impairment": {
      "audio_exit_confirmation": true,
      "haptic_guidance": true,
      "voice_command_priority": "high"
    },
    "hearing_impairment": {
      "visual_exit_indicator": true,
      "vibration_pattern": "emergency",
      "screen_flash_safe": true
    },
    "motor_impairment": {
      "extended_hold_time": false,
      "single_button_exit": true,
      "eye_tracking_exit": true,
      "voice_only_exit": true
    },
    "cognitive_impairment": {
      "simple_exit_process": true,
      "large_exit_button": true,
      "consistent_placement": true,
      "no_confirmation_required": true
    }
  }
}
```

---

## 7. Session Health Management

### 7.1 Health Monitoring Dashboard

```typescript
interface HealthDashboard {
  // 실시간 지표
  realTimeMetrics: {
    sessionDuration: number;
    continuousUse: number;
    heartRateEstimate?: number;    // 옵션: 외부 기기
    stressIndicators: number;
    fatigueLevel: number;
  };

  // 경고 시스템
  warnings: {
    timeWarnings: number[];        // [30, 60, 90, 120] 분
    fatigueWarning: number;        // 임계값
    healthRiskWarning: boolean;
  };

  // 권장 사항
  recommendations: {
    takeBreak: boolean;
    endSession: boolean;
    adjustSettings: string[];
    healthTips: string[];
  };

  // 기록
  sessionHistory: {
    totalUsageToday: number;
    averageSessionLength: number;
    breaksTaken: number;
    healthScore: number;
  };
}
```

### 7.2 Automatic Safety Interventions

```
┌─────────────────────────────────────────────────────────────────────┐
│                    Safety Intervention Levels                        │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  Level 1: Gentle Reminder (30분)                                     │
│  ├── 작은 알림 표시                                                   │
│  ├── 소리/진동 알림                                                   │
│  └── 무시 가능                                                        │
│                                                                      │
│  Level 2: Strong Suggestion (60분)                                   │
│  ├── 화면 중앙 알림                                                   │
│  ├── 휴식 권장 메시지                                                 │
│  └── 확인 필요                                                        │
│                                                                      │
│  Level 3: Insistent Warning (90분)                                   │
│  ├── 지속적 알림                                                      │
│  ├── 건강 위험 경고                                                   │
│  └── 연장 시 동의 필요                                                │
│                                                                      │
│  Level 4: Enforced Break (120분, 설정 가능)                          │
│  ├── 자동 Safe Space 전환                                            │
│  ├── 최소 휴식 시간 강제                                              │
│  └── 의료 면책 표시                                                   │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 8. WIA Device Integration Safety

### 8.1 Exoskeleton Safety

```typescript
interface ExoskeletonSafetyConfig {
  // 연결 상태
  connectionSafety: {
    heartbeat: number;            // ms
    disconnectGracePeriod: number;
    fallbackBehavior: 'pause' | 'safe_mode' | 'exit';
  };

  // 움직임 안전
  movementSafety: {
    maxForce: number;             // 뉴턴
    maxSpeed: number;             // m/s
    rangeOfMotion: RangeLimit[];
    emergencyStop: boolean;
  };

  // XR 동기화
  xrSync: {
    latencyThreshold: number;     // ms
    syncLossAction: 'pause' | 'reduce_speed' | 'exit';
    visualMismatchCorrection: boolean;
  };
}
```

### 8.2 Bionic Eye Safety

```typescript
interface BionicEyeSafetyConfig {
  // 자극 안전
  stimulationSafety: {
    maxIntensity: number;
    maxDuration: number;
    restInterval: number;
    overloadProtection: boolean;
  };

  // XR 통합 안전
  xrIntegration: {
    brightnessLimit: number;
    contrastLimit: number;
    flashPrevention: boolean;
    calibrationRequired: boolean;
  };

  // 의료 모니터링
  medicalMonitor: {
    abnormalActivityDetection: boolean;
    autoShutdown: boolean;
    logForMedicalReview: boolean;
  };
}
```

---

## 9. Safety Compliance Requirements

### 9.1 Regulatory Standards

| Standard | Region | Requirement |
|----------|--------|-------------|
| **FDA 21 CFR** | USA | 의료 기기 연동 시 |
| **EU MDR** | EU | 의료 기기 규정 |
| **ISO 14971** | Global | 위험 관리 |
| **IEC 62366** | Global | 사용성 엔지니어링 |
| **WCAG 2.1** | Global | 웹 접근성 (XR 적용) |

### 9.2 Safety Certification Checklist

```yaml
safety_certification:
  required_tests:
    - photosensitivity_analysis
    - motion_sickness_evaluation
    - eye_strain_assessment
    - physical_safety_audit
    - emergency_exit_verification
    - accessibility_compliance

  documentation:
    - risk_assessment_report
    - safety_test_results
    - user_manual_safety_section
    - medical_disclaimer

  user_protections:
    - health_warning_displayed: true
    - consent_obtained: true
    - opt_out_available: true
    - data_handling_disclosed: true
```

---

## 10. Implementation Requirements

### 10.1 Safety API

```rust
/// Safety check result
pub struct SafetyCheckResult {
    pub passed: bool,
    pub warnings: Vec<SafetyWarning>,
    pub blocked_features: Vec<String>,
    pub required_actions: Vec<SafetyAction>,
}

/// Perform comprehensive safety check
pub async fn perform_safety_check(
    profile: &XRAccessibilityProfile,
    content: &ContentMetadata,
    device: &XRDeviceCapabilities,
) -> SafetyCheckResult {
    // Implementation
}

/// Emergency exit handler
pub async fn trigger_emergency_exit(
    session: &mut XRSession,
    reason: EmergencyReason,
) -> Result<()> {
    // Implementation
}
```

### 10.2 Safety Configuration Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "XR Safety Configuration",
  "type": "object",
  "required": ["version", "safety_level", "emergency_exit"],
  "properties": {
    "version": { "type": "string" },
    "safety_level": {
      "type": "string",
      "enum": ["maximum", "standard", "minimal", "custom"]
    },
    "vr_sickness_prevention": {
      "$ref": "#/definitions/vrSicknessPrevention"
    },
    "photosensitivity_protection": {
      "$ref": "#/definitions/photosensitivityProtection"
    },
    "eye_strain_prevention": {
      "$ref": "#/definitions/eyeStrainPrevention"
    },
    "physical_safety": {
      "$ref": "#/definitions/physicalSafety"
    },
    "emergency_exit": {
      "$ref": "#/definitions/emergencyExit"
    },
    "session_health": {
      "$ref": "#/definitions/sessionHealth"
    }
  }
}
```

---

## 11. References

- ISO 14971:2019 Medical devices - Risk management
- IEC 62366-1:2015 Medical devices - Usability engineering
- WCAG 2.1 Guidelines (Photosensitivity)
- FDA Guidance on Virtual Reality
- XR Safety Initiative (XRSI) Guidelines
- WIA Safety Framework v1.0
