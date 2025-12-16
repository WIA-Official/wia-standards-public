# WIA Medical Device Accessibility: Alert Transmission Protocol

## 1. Overview

본 문서는 의료기기 알림의 다중 감각 전송 프로토콜을 정의합니다.

**Version:** 1.0.0
**Status:** Draft
**Phase:** 3 - Communication Protocol

---

## 2. Alert Classification

### 2.1 Priority Levels

| Level | Name | Response Time | User Impact | Examples |
|-------|------|---------------|-------------|----------|
| **P0** | Critical | < 100ms | Life-threatening | Urgent low glucose, cardiac arrest |
| **P1** | High | < 500ms | Immediate attention | Low glucose, high BP |
| **P2** | Medium | < 2s | Timely attention | Approaching threshold |
| **P3** | Low | < 5s | Informational | Battery low, sync complete |
| **P4** | Notification | Best effort | Non-urgent | Tips, reminders |

### 2.2 Alert Categories

```typescript
enum AlertCategory {
  // Vital alerts
  VITAL_CRITICAL = 'vital_critical',
  VITAL_WARNING = 'vital_warning',
  VITAL_INFO = 'vital_info',

  // Device alerts
  DEVICE_ERROR = 'device_error',
  DEVICE_WARNING = 'device_warning',
  DEVICE_STATUS = 'device_status',

  // User action alerts
  ACTION_REQUIRED = 'action_required',
  ACTION_REMINDER = 'action_reminder',
  ACTION_COMPLETE = 'action_complete',

  // System alerts
  SYSTEM_ERROR = 'system_error',
  SYSTEM_WARNING = 'system_warning',
  SYSTEM_INFO = 'system_info'
}
```

---

## 3. Multi-Modal Output Specification

### 3.1 Output Modality Requirements

| Priority | Visual | Auditory | Haptic | Min Modalities |
|----------|--------|----------|--------|----------------|
| P0 | Required | Required | Required | 3 (all) |
| P1 | Required | Required | Recommended | 2 |
| P2 | Required | Optional | Optional | 1 |
| P3 | Optional | Optional | Optional | 1 |
| P4 | Optional | Optional | Optional | 1 |

### 3.2 Visual Alert Specification

```typescript
interface VisualAlertSpec {
  // Display type
  displayType: VisualDisplayType;

  // Content
  content: {
    // Icon/symbol
    icon: {
      id: string;
      fallbackEmoji: string;
      description: string;      // For screen readers
    };

    // Text
    text: {
      primary: string;
      simple: string;           // Cognitive accessibility
      detailed?: string;
    };

    // Localization
    localization: Record<string, {
      primary: string;
      simple: string;
    }>;
  };

  // Visual treatment
  treatment: {
    // Colors
    colors: {
      background: string;
      foreground: string;
      accent: string;
      highContrastBackground: string;
      highContrastForeground: string;
    };

    // Animation
    animation: {
      type: 'none' | 'pulse' | 'flash' | 'slide';
      duration: number;
      // Flash rate limited to max 3 Hz for photosensitivity
      flashRateHz?: number;
    };

    // Size/prominence
    prominence: 'minimal' | 'standard' | 'emphasized' | 'fullscreen';
  };

  // Accessibility
  accessibility: {
    highContrastMode: boolean;
    largeTextMode: boolean;
    colorBlindSafe: boolean;
    photosensitivitySafe: boolean;
    screenReaderText: string;
  };
}

enum VisualDisplayType {
  BANNER = 'banner',
  POPUP = 'popup',
  OVERLAY = 'overlay',
  STATUS_ICON = 'status_icon',
  FULLSCREEN = 'fullscreen',
  LED_INDICATOR = 'led_indicator'
}
```

### 3.3 Auditory Alert Specification

```typescript
interface AuditoryAlertSpec {
  // Alert type
  alertType: AuditoryAlertType;

  // Tone configuration
  tone?: {
    // Frequency
    frequency: {
      primary: number;         // Hz
      secondary?: number;      // For two-tone alerts
    };

    // Pattern
    pattern: {
      type: 'continuous' | 'intermittent' | 'escalating' | 'pulsing';
      onDuration: number;      // ms
      offDuration: number;     // ms
      cycles: number;
    };

    // Volume
    volume: {
      level: number;           // 0-100
      minimum: number;         // Minimum for critical alerts
      escalationStep?: number;
    };

    // Accessibility
    accessibility: {
      hearingAidOptimized: boolean;
      frequencyShiftAvailable: boolean;
    };
  };

  // Voice configuration
  voice?: {
    // Content
    content: {
      text: string;
      textSimple: string;
      ssml?: string;           // For advanced TTS
    };

    // Localization
    localization: Record<string, {
      text: string;
      textSimple: string;
    }>;

    // Voice settings
    settings: {
      voiceId?: string;
      language: string;
      speed: number;           // 0.5 - 2.0
      pitch: 'low' | 'medium' | 'high';
      repeatCount: number;
      repeatInterval: number;  // ms
    };

    // Accessibility
    accessibility: {
      spellingAvailable: boolean;
      slowModeAvailable: boolean;
    };
  };
}

enum AuditoryAlertType {
  TONE_ONLY = 'tone_only',
  VOICE_ONLY = 'voice_only',
  TONE_THEN_VOICE = 'tone_then_voice',
  VOICE_THEN_TONE = 'voice_then_tone',
  SIMULTANEOUS = 'simultaneous'
}
```

### 3.4 Haptic Alert Specification

```typescript
interface HapticAlertSpec {
  // Target device
  target: {
    deviceType: HapticDeviceType;
    deviceId?: string;
    bodyLocation: HapticBodyLocation;
  };

  // Pattern definition
  pattern: {
    // Predefined pattern
    patternId?: string;

    // Custom pattern
    custom?: {
      // Vibration sequence
      sequence: VibrationSegment[];

      // Total duration
      totalDuration: number;

      // Repeat configuration
      repeat: {
        count: number;
        interval: number;      // ms between repeats
      };
    };
  };

  // Intensity
  intensity: {
    level: number;             // 0.0 - 1.0
    minimum: number;           // Minimum for critical
    dynamic?: {
      // Scale by value
      valueScaling?: {
        enabled: boolean;
        minValue: number;
        maxValue: number;
      };
      // Escalation over time
      escalation?: {
        enabled: boolean;
        stepSize: number;
        interval: number;
      };
    };
  };

  // Accessibility
  accessibility: {
    strengthOptions: number[]; // Available intensity levels
    patternDescriptions: Record<string, string>;
  };
}

interface VibrationSegment {
  type: 'vibrate' | 'pause';
  duration: number;            // ms
  intensity?: number;          // 0.0 - 1.0 for vibrate segments
  frequency?: number;          // Hz if device supports
}

enum HapticDeviceType {
  SMARTPHONE = 'smartphone',
  SMARTWATCH = 'smartwatch',
  WIA_EXOSKELETON = 'wia_exoskeleton',
  WIA_HAPTIC_GLOVES = 'wia_haptic_gloves',
  MEDICAL_DEVICE = 'medical_device',
  CUSTOM = 'custom'
}

enum HapticBodyLocation {
  WRIST_LEFT = 'wrist_left',
  WRIST_RIGHT = 'wrist_right',
  ARM_LEFT = 'arm_left',
  ARM_RIGHT = 'arm_right',
  HAND_LEFT = 'hand_left',
  HAND_RIGHT = 'hand_right',
  FINGER = 'finger',
  TORSO = 'torso',
  BACK = 'back',
  LEG = 'leg',
  CUSTOM = 'custom'
}
```

---

## 4. Alert Transmission Protocol

### 4.1 Alert Message Format

```typescript
interface AlertMessage {
  // Header
  header: {
    messageId: string;
    version: number;
    timestamp: number;
    priority: AlertPriority;
    category: AlertCategory;
  };

  // Source
  source: {
    deviceId: string;
    deviceType: string;
    measurementType?: string;
  };

  // Alert data
  alert: {
    code: string;
    type: string;

    // Medical data (if applicable)
    medicalData?: {
      value: number;
      unit: string;
      referenceRange?: {
        low: number;
        high: number;
      };
      trend?: 'rising' | 'falling' | 'stable';
    };

    // Recommended action
    action: {
      type: string;
      description: string;
      descriptionSimple: string;
      urgency: string;
    };
  };

  // Output specifications
  outputs: {
    visual: VisualAlertSpec;
    auditory: AuditoryAlertSpec;
    haptic: HapticAlertSpec;
  };

  // Delivery configuration
  delivery: {
    targets: DeliveryTarget[];
    escalation: EscalationConfig;
    acknowledgment: AcknowledgmentConfig;
  };

  // Localization
  localization: {
    defaultLanguage: string;
    supportedLanguages: string[];
    translations: Record<string, AlertTranslation>;
  };
}

interface DeliveryTarget {
  deviceId: string;
  deviceType: string;
  modalities: ('visual' | 'auditory' | 'haptic')[];
  priority: number;
  required: boolean;
}

interface EscalationConfig {
  enabled: boolean;
  timeout: number;
  steps: EscalationStep[];
}

interface EscalationStep {
  delay: number;
  action: EscalationAction;
  targets?: string[];
}

enum EscalationAction {
  REPEAT = 'repeat',
  INCREASE_INTENSITY = 'increase_intensity',
  ADD_MODALITY = 'add_modality',
  ADD_DEVICE = 'add_device',
  NOTIFY_CAREGIVER = 'notify_caregiver',
  NOTIFY_PROVIDER = 'notify_provider',
  CALL_EMERGENCY = 'call_emergency'
}

interface AcknowledgmentConfig {
  required: boolean;
  timeout: number;
  methods: AcknowledgmentMethod[];
  autoSnoozeOnAck: boolean;
  snoozeOptions: number[];    // minutes
}

enum AcknowledgmentMethod {
  BUTTON_PRESS = 'button_press',
  VOICE_COMMAND = 'voice_command',
  GESTURE = 'gesture',
  TOUCH = 'touch',
  PROXIMITY = 'proximity'
}
```

### 4.2 Transmission Sequence

```
┌──────────────┐     ┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│  Medical     │     │  Primary     │     │  Secondary   │     │  Caregiver   │
│  Device      │     │  Output      │     │  Output      │     │  Device      │
└──────┬───────┘     └──────┬───────┘     └──────┬───────┘     └──────┬───────┘
       │                     │                    │                    │
       │  1. Alert Generated │                    │                    │
       ├─────────────────────┼────────────────────┼────────────────────│
       │                     │                    │                    │
       │  2a. Visual/Audio   │                    │                    │
       ├────────────────────►│                    │                    │
       │                     │                    │                    │
       │  2b. Haptic         │                    │                    │
       ├─────────────────────┼───────────────────►│                    │
       │                     │                    │                    │
       │  3. ACK/NACK        │                    │                    │
       │◄────────────────────┤                    │                    │
       │◄────────────────────┼────────────────────┤                    │
       │                     │                    │                    │
       │  4. No ACK → Escalate                    │                    │
       ├─────────────────────┼────────────────────┼───────────────────►│
       │                     │                    │                    │
       │  5. Caregiver ACK   │                    │                    │
       │◄────────────────────┼────────────────────┼────────────────────┤
       │                     │                    │                    │
```

---

## 5. Predefined Alert Templates

### 5.1 Glucose Alerts (CGM)

```typescript
const glucoseAlertTemplates: Record<string, AlertMessage> = {
  urgent_low: {
    header: {
      messageId: 'template_urgent_low',
      version: 1,
      timestamp: 0,
      priority: AlertPriority.P0,
      category: AlertCategory.VITAL_CRITICAL
    },
    source: {
      deviceId: '',
      deviceType: 'cgm',
      measurementType: 'glucose'
    },
    alert: {
      code: 'GLUCOSE_URGENT_LOW',
      type: 'threshold_breach',
      action: {
        type: 'consume_carbs',
        description: 'Consume 15-20 grams of fast-acting carbohydrates immediately',
        descriptionSimple: 'Eat sugar now!',
        urgency: 'immediate'
      }
    },
    outputs: {
      visual: {
        displayType: VisualDisplayType.FULLSCREEN,
        content: {
          icon: { id: 'glucose_critical', fallbackEmoji: '🚨', description: 'Critical low glucose alert' },
          text: {
            primary: 'URGENT: Blood glucose is critically low',
            simple: 'SUGAR VERY LOW!',
            detailed: 'Your blood glucose has dropped to a dangerous level. Take immediate action.'
          },
          localization: {
            'ko': { primary: '긴급: 혈당이 위험 수준입니다', simple: '혈당 매우 낮음!' },
            'es': { primary: 'URGENTE: Glucosa críticamente baja', simple: '¡AZÚCAR MUY BAJO!' }
          }
        },
        treatment: {
          colors: {
            background: '#FF0000',
            foreground: '#FFFFFF',
            accent: '#FF0000',
            highContrastBackground: '#000000',
            highContrastForeground: '#FF0000'
          },
          animation: { type: 'pulse', duration: 500 },
          prominence: 'fullscreen'
        },
        accessibility: {
          highContrastMode: true,
          largeTextMode: true,
          colorBlindSafe: true,
          photosensitivitySafe: true,
          screenReaderText: 'Critical alert. Blood glucose is dangerously low. Eat sugar immediately.'
        }
      },
      auditory: {
        alertType: AuditoryAlertType.TONE_THEN_VOICE,
        tone: {
          frequency: { primary: 1000, secondary: 1500 },
          pattern: { type: 'escalating', onDuration: 200, offDuration: 100, cycles: 5 },
          volume: { level: 90, minimum: 70, escalationStep: 5 },
          accessibility: { hearingAidOptimized: true, frequencyShiftAvailable: true }
        },
        voice: {
          content: {
            text: 'Urgent! Your blood sugar is critically low. Eat sugar immediately.',
            textSimple: 'Low sugar! Eat now!',
            ssml: '<speak><emphasis level="strong">Urgent!</emphasis> Your blood sugar is critically low. <break time="300ms"/> Eat sugar immediately.</speak>'
          },
          localization: {
            'ko': { text: '긴급! 혈당이 위험합니다. 즉시 당분을 섭취하세요.', textSimple: '혈당 낮음! 먹어요!' },
            'es': { text: '¡Urgente! Su azúcar en sangre está muy bajo. Coma azúcar inmediatamente.', textSimple: '¡Azúcar bajo! ¡Comer!' }
          },
          settings: {
            language: 'en',
            speed: 1.0,
            pitch: 'medium',
            repeatCount: 3,
            repeatInterval: 5000
          },
          accessibility: { spellingAvailable: true, slowModeAvailable: true }
        }
      },
      haptic: {
        target: {
          deviceType: HapticDeviceType.WIA_EXOSKELETON,
          bodyLocation: HapticBodyLocation.WRIST_LEFT
        },
        pattern: {
          patternId: 'urgent_continuous',
          custom: {
            sequence: [
              { type: 'vibrate', duration: 300, intensity: 1.0 },
              { type: 'pause', duration: 100 },
              { type: 'vibrate', duration: 300, intensity: 1.0 },
              { type: 'pause', duration: 100 },
              { type: 'vibrate', duration: 500, intensity: 1.0 }
            ],
            totalDuration: 1300,
            repeat: { count: 10, interval: 2000 }
          }
        },
        intensity: {
          level: 1.0,
          minimum: 0.8,
          dynamic: { escalation: { enabled: true, stepSize: 0.1, interval: 5000 } }
        },
        accessibility: {
          strengthOptions: [0.5, 0.7, 0.85, 1.0],
          patternDescriptions: {
            'urgent_continuous': 'Strong continuous vibration indicating critical alert'
          }
        }
      }
    },
    delivery: {
      targets: [
        { deviceId: 'primary', deviceType: 'smartphone', modalities: ['visual', 'auditory'], priority: 1, required: true },
        { deviceId: 'exoskeleton', deviceType: 'wia_exoskeleton', modalities: ['haptic'], priority: 1, required: false },
        { deviceId: 'smartwatch', deviceType: 'smartwatch', modalities: ['visual', 'haptic'], priority: 2, required: false }
      ],
      escalation: {
        enabled: true,
        timeout: 30000,
        steps: [
          { delay: 30000, action: EscalationAction.INCREASE_INTENSITY },
          { delay: 60000, action: EscalationAction.NOTIFY_CAREGIVER },
          { delay: 120000, action: EscalationAction.CALL_EMERGENCY }
        ]
      },
      acknowledgment: {
        required: true,
        timeout: 60000,
        methods: [AcknowledgmentMethod.BUTTON_PRESS, AcknowledgmentMethod.VOICE_COMMAND, AcknowledgmentMethod.GESTURE],
        autoSnoozeOnAck: false,
        snoozeOptions: []
      }
    },
    localization: {
      defaultLanguage: 'en',
      supportedLanguages: ['en', 'ko', 'es', 'ja', 'zh'],
      translations: {}
    }
  }
};
```

### 5.2 Blood Pressure Alerts

```typescript
const bpAlertTemplates: Record<string, Partial<AlertMessage>> = {
  hypertensive_crisis: {
    header: {
      priority: AlertPriority.P0,
      category: AlertCategory.VITAL_CRITICAL
    },
    alert: {
      code: 'BP_HYPERTENSIVE_CRISIS',
      type: 'threshold_breach',
      action: {
        type: 'seek_emergency_care',
        description: 'Seek emergency medical care immediately. Blood pressure is dangerously high.',
        descriptionSimple: 'Go to hospital now!',
        urgency: 'immediate'
      }
    }
  },

  elevated: {
    header: {
      priority: AlertPriority.P2,
      category: AlertCategory.VITAL_WARNING
    },
    alert: {
      code: 'BP_ELEVATED',
      type: 'threshold_warning',
      action: {
        type: 'rest_recheck',
        description: 'Rest for 5 minutes and take another reading',
        descriptionSimple: 'Rest, then check again',
        urgency: 'soon'
      }
    }
  }
};
```

---

## 6. User Preference Overrides

### 6.1 Alert Customization

```typescript
interface UserAlertPreferences {
  // Global settings
  global: {
    // Master volume/intensity
    volumeMultiplier: number;
    hapticMultiplier: number;

    // Quiet hours
    quietHours: {
      enabled: boolean;
      start: string;           // HH:MM
      end: string;
      criticalOverride: boolean;
    };

    // Modality preferences
    preferredModalities: ('visual' | 'auditory' | 'haptic')[];
    disabledModalities: ('visual' | 'auditory' | 'haptic')[];
  };

  // Per-category settings
  categoryOverrides: Record<AlertCategory, CategoryPreference>;

  // Per-alert settings
  alertOverrides: Record<string, AlertPreference>;

  // Accessibility settings
  accessibility: {
    // Visual
    visual: {
      highContrastRequired: boolean;
      largeTextRequired: boolean;
      photosensitivityMode: boolean;
    };

    // Auditory
    auditory: {
      preferVoice: boolean;
      voiceSpeed: number;
      frequencyShift: boolean;
      minimumVolume: number;
    };

    // Haptic
    haptic: {
      preferHaptic: boolean;
      minimumIntensity: number;
      wiaDevicePreferred: boolean;
    };

    // Cognitive
    cognitive: {
      simplifiedMessagesOnly: boolean;
      stepByStepGuidance: boolean;
      confirmationRequired: boolean;
    };
  };
}

interface CategoryPreference {
  enabled: boolean;
  modalities: ('visual' | 'auditory' | 'haptic')[];
  volumeOverride?: number;
  hapticOverride?: number;
  snoozeAllowed: boolean;
  snoozeMaxDuration?: number;
}

interface AlertPreference {
  enabled: boolean;
  customThreshold?: number;
  customMessage?: string;
  customVoice?: string;
  customHapticPattern?: string;
}
```

---

## 7. Testing and Compliance

### 7.1 Alert Test Suite

```typescript
interface AlertTestSuite {
  // Test cases for each alert type
  testCases: AlertTestCase[];

  // Accessibility compliance tests
  accessibilityTests: AccessibilityTestCase[];

  // Performance tests
  performanceTests: PerformanceTestCase[];
}

interface AlertTestCase {
  id: string;
  name: string;
  alertType: string;
  priority: AlertPriority;

  // Test parameters
  params: {
    triggerCondition: any;
    expectedOutputs: ExpectedOutput[];
    expectedTiming: TimingRequirement[];
  };

  // Pass criteria
  criteria: {
    visualDelivered: boolean;
    auditoryDelivered: boolean;
    hapticDelivered: boolean;
    timingMet: boolean;
    accessibilityCompliant: boolean;
  };
}

interface AccessibilityTestCase {
  id: string;
  category: 'visual' | 'auditory' | 'haptic' | 'cognitive';
  requirement: string;
  testMethod: string;
  passCriteria: string;
}
```

### 7.2 Compliance Requirements

| Requirement | Priority | Description |
|------------|----------|-------------|
| Multi-modal delivery | P0-P1 | All critical/high alerts must use 2+ modalities |
| Response time | All | Must meet priority-specific timing |
| Escalation | P0-P1 | Must implement escalation for unacknowledged alerts |
| Accessibility | All | Must meet WCAG 2.1 AA equivalent |
| Photosensitivity | Visual | Flash rate ≤ 3 Hz |
| Volume safety | Auditory | Max volume limits respected |
| Haptic safety | Haptic | Intensity limits respected |

---

## 8. References

- IEC 60601-1-8: Medical Electrical Equipment - Alarms
- AAMI HE75: Human Factors Engineering
- WCAG 2.1 Level AA
- FDA Human Factors Guidance
- WIA Haptic Standard v1.0
