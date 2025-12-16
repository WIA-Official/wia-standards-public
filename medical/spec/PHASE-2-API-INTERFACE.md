# WIA Medical Device Accessibility: Device Integration Specification

## 1. Overview

본 문서는 의료기기와 WIA 생태계 기기 간의 통합 프로토콜을 정의합니다.

**Version:** 1.0.0
**Status:** Draft
**Phase:** 3 - Communication Protocol

---

## 2. WIA Device Ecosystem

### 2.1 Supported WIA Devices

```
┌─────────────────────────────────────────────────────────────────┐
│                    WIA Medical Device Hub                        │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌───────────────┐   ┌───────────────┐   ┌───────────────┐     │
│  │  Exoskeleton  │   │  Bionic Eye   │   │  Voice-Sign   │     │
│  │  (Haptic)     │   │  (Visual)     │   │  (Sign Lang)  │     │
│  └───────┬───────┘   └───────┬───────┘   └───────┬───────┘     │
│          │                   │                   │               │
│          └───────────────────┼───────────────────┘               │
│                              │                                   │
│                    ┌─────────┴─────────┐                        │
│                    │  Medical Device   │                        │
│                    │  (CGM, BP, etc.)  │                        │
│                    └───────────────────┘                        │
│                                                                  │
│  ┌───────────────┐   ┌───────────────┐   ┌───────────────┐     │
│  │  Smart        │   │  Haptic       │   │  Braille      │     │
│  │  Wheelchair   │   │  Gloves       │   │  Display      │     │
│  └───────────────┘   └───────────────┘   └───────────────┘     │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### 2.2 Device Capability Matrix

| Device | Visual | Auditory | Haptic | Input | Mobility |
|--------|--------|----------|--------|-------|----------|
| **Exoskeleton** | - | - | ★★★ | ★★ | ★★★ |
| **Bionic Eye** | ★★★ | - | - | - | - |
| **Voice-Sign** | ★★ | ★★★ | - | ★ | - |
| **Smart Wheelchair** | - | - | ★ | ★ | ★★★ |
| **Haptic Gloves** | - | - | ★★★ | ★★ | - |
| **Braille Display** | ★★ | - | ★★ | ★ | - |

---

## 3. Exoskeleton Integration

### 3.1 Connection Workflow

```
┌──────────────┐                              ┌──────────────┐
│  Medical     │                              │  WIA         │
│  Device      │                              │  Exoskeleton │
└──────┬───────┘                              └──────┬───────┘
       │                                              │
       │  1. Discovery (WIA Service UUID)             │
       │─────────────────────────────────────────────►│
       │                                              │
       │  2. Capability Exchange                      │
       │◄────────────────────────────────────────────►│
       │                                              │
       │  3. Profile Synchronization                  │
       │─────────────────────────────────────────────►│
       │                                              │
       │  4. Haptic Mapping Configuration             │
       │─────────────────────────────────────────────►│
       │                                              │
       │  5. Connection Established                   │
       │◄────────────────────────────────────────────►│
       │                                              │
```

### 3.2 Haptic Mapping Configuration

```typescript
interface ExoskeletonHapticMapping {
  // Device identification
  deviceId: string;
  exoskeletonId: string;

  // Mapping version
  version: string;
  timestamp: number;

  // Alert mappings
  alertMappings: AlertHapticMapping[];

  // Data feedback mappings
  dataFeedbackMappings: DataHapticMapping[];

  // Navigation mappings
  navigationMappings: NavigationHapticMapping[];
}

interface AlertHapticMapping {
  // Source alert
  alertType: string;
  alertPriority: AlarmPriority;

  // Target haptic output
  hapticOutput: {
    target: HapticTarget;
    pattern: HapticPatternConfig;
  };

  // Conditions
  conditions?: {
    timeOfDay?: TimeRange;
    activityLevel?: ActivityLevel;
    userOverride?: boolean;
  };
}

interface HapticPatternConfig {
  // Predefined pattern
  patternId?: string;

  // Custom pattern
  custom?: {
    // Vibration sequence [duration_ms, pause_ms, ...]
    sequence: number[];

    // Intensity curve [0.0 - 1.0]
    intensityCurve: number[];

    // Frequency (if supported)
    frequency?: number;
  };

  // Dynamic parameters
  dynamic?: {
    // Scale intensity by value
    intensityByValue?: {
      minValue: number;
      maxValue: number;
      minIntensity: number;
      maxIntensity: number;
    };

    // Scale pattern speed
    speedByUrgency?: boolean;
  };
}
```

### 3.3 Medical Data to Haptic Encoding

```typescript
interface DataHapticMapping {
  // Data type
  dataType: MedicalDataType;

  // Encoding method
  encoding: HapticEncodingMethod;

  // Target location
  target: HapticTarget;

  // Value ranges
  ranges: DataHapticRange[];
}

enum HapticEncodingMethod {
  // Intensity maps to value
  INTENSITY_LINEAR = 'intensity_linear',

  // Frequency maps to value
  FREQUENCY_LINEAR = 'frequency_linear',

  // Pattern changes by range
  PATTERN_DISCRETE = 'pattern_discrete',

  // Pulse count encodes digits
  PULSE_COUNT = 'pulse_count',

  // Spatial encoding (multiple motors)
  SPATIAL = 'spatial'
}

interface DataHapticRange {
  minValue: number;
  maxValue: number;
  label: string;            // 'low', 'normal', 'high', etc.

  // Haptic output for this range
  hapticConfig: {
    pattern: string;
    baseIntensity: number;
    modulation?: 'none' | 'pulse' | 'wave';
  };
}

// Example: Blood glucose haptic encoding
const glucoseHapticMapping: DataHapticMapping = {
  dataType: MedicalDataType.GLUCOSE,
  encoding: HapticEncodingMethod.PATTERN_DISCRETE,
  target: HapticTarget.LEFT_WRIST,
  ranges: [
    {
      minValue: 0,
      maxValue: 54,
      label: 'urgent_low',
      hapticConfig: {
        pattern: 'urgent_pulse',
        baseIntensity: 1.0,
        modulation: 'pulse'
      }
    },
    {
      minValue: 55,
      maxValue: 69,
      label: 'low',
      hapticConfig: {
        pattern: 'gentle_pulse',
        baseIntensity: 0.7,
        modulation: 'pulse'
      }
    },
    {
      minValue: 70,
      maxValue: 180,
      label: 'normal',
      hapticConfig: {
        pattern: 'subtle_tick',
        baseIntensity: 0.2,
        modulation: 'none'
      }
    },
    {
      minValue: 181,
      maxValue: 250,
      label: 'high',
      hapticConfig: {
        pattern: 'double_tap',
        baseIntensity: 0.5,
        modulation: 'none'
      }
    },
    {
      minValue: 251,
      maxValue: 999,
      label: 'urgent_high',
      hapticConfig: {
        pattern: 'escalating',
        baseIntensity: 0.8,
        modulation: 'wave'
      }
    }
  ]
};
```

### 3.4 Input Event Handling

```typescript
interface ExoskeletonInputEvent {
  // Event identification
  eventId: string;
  timestamp: number;

  // Event type
  type: ExoskeletonEventType;

  // Event data
  data: ExoskeletonEventData;
}

enum ExoskeletonEventType {
  // Gesture events
  GESTURE_TAP = 'gesture_tap',
  GESTURE_DOUBLE_TAP = 'gesture_double_tap',
  GESTURE_LONG_PRESS = 'gesture_long_press',
  GESTURE_SWIPE = 'gesture_swipe',

  // Motion events
  MOTION_POINT = 'motion_point',
  MOTION_GRIP = 'motion_grip',
  MOTION_RELEASE = 'motion_release',

  // Button events (if available)
  BUTTON_PRESS = 'button_press',
  BUTTON_RELEASE = 'button_release'
}

interface ExoskeletonEventData {
  // Gesture data
  gesture?: {
    direction?: 'up' | 'down' | 'left' | 'right';
    velocity?: number;
    position?: { x: number; y: number; z: number };
  };

  // Motion data
  motion?: {
    jointAngles?: Record<string, number>;
    force?: number;
    precision?: number;
  };

  // Button data
  button?: {
    buttonId: string;
    state: 'pressed' | 'released';
  };
}

// Input action mapping
interface InputActionMapping {
  eventType: ExoskeletonEventType;
  eventFilter?: Partial<ExoskeletonEventData>;

  // Medical device action
  action: MedicalDeviceAction;
}

enum MedicalDeviceAction {
  READ_CURRENT_VALUE = 'read_current_value',
  READ_TREND = 'read_trend',
  ACKNOWLEDGE_ALERT = 'acknowledge_alert',
  SNOOZE_ALERT = 'snooze_alert',
  REQUEST_HELP = 'request_help',
  NAVIGATE_MENU = 'navigate_menu',
  SELECT_ITEM = 'select_item',
  GO_BACK = 'go_back'
}
```

---

## 4. Bionic Eye Integration

### 4.1 Display Optimization Protocol

```typescript
interface BionicEyeIntegrationConfig {
  // Device identification
  deviceId: string;
  bionicEyeId: string;

  // Display capabilities
  capabilities: {
    resolution: Resolution;
    colorDepth: number;
    refreshRate: number;
    fovDegrees: number;
  };

  // Optimization settings
  optimization: BionicEyeOptimization;

  // Content mappings
  contentMappings: ContentDisplayMapping[];
}

interface BionicEyeOptimization {
  // Contrast enhancement
  contrast: {
    enabled: boolean;
    level: number;            // 0.0 - 2.0
    adaptiveEnabled: boolean;
  };

  // Edge enhancement
  edges: {
    enabled: boolean;
    strength: number;         // 0.0 - 1.0
    colorCoding: boolean;
  };

  // Simplification
  simplification: {
    enabled: boolean;
    level: number;            // 0.0 - 1.0
    preserveText: boolean;
    preserveNumbers: boolean;
  };

  // Color adaptation
  colorAdaptation: {
    mode: ColorAdaptationMode;
    customMapping?: ColorMap;
  };

  // Focus assistance
  focus: {
    highlightImportant: boolean;
    dimBackground: boolean;
    focusIndicator: boolean;
  };
}

enum ColorAdaptationMode {
  NORMAL = 'normal',
  HIGH_CONTRAST_LIGHT = 'high_contrast_light',
  HIGH_CONTRAST_DARK = 'high_contrast_dark',
  PROTANOPIA_SAFE = 'protanopia_safe',
  DEUTERANOPIA_SAFE = 'deuteranopia_safe',
  TRITANOPIA_SAFE = 'tritanopia_safe',
  GRAYSCALE = 'grayscale',
  CUSTOM = 'custom'
}
```

### 4.2 Medical Data Visualization

```typescript
interface ContentDisplayMapping {
  // Content type
  contentType: MedicalContentType;

  // Display configuration
  display: {
    layout: DisplayLayout;
    priority: number;
    position: ScreenPosition;
  };

  // Visual encoding
  visualEncoding: VisualEncodingConfig;
}

enum MedicalContentType {
  READING_NUMERIC = 'reading_numeric',
  READING_GRAPH = 'reading_graph',
  TREND_INDICATOR = 'trend_indicator',
  ALERT_BANNER = 'alert_banner',
  STATUS_ICON = 'status_icon',
  MENU = 'menu'
}

interface VisualEncodingConfig {
  // Numeric reading display
  numeric?: {
    fontSize: 'small' | 'medium' | 'large' | 'extra_large';
    fontWeight: 'normal' | 'bold';
    colorByRange: boolean;
    rangeColors: Record<string, string>;
  };

  // Graph display
  graph?: {
    type: 'line' | 'bar' | 'area';
    thickness: number;
    patternEncoding: boolean;  // Use patterns instead of colors
    gridEnabled: boolean;
    labelsEnabled: boolean;
  };

  // Trend indicator
  trend?: {
    type: 'arrow' | 'icon' | 'animation';
    size: number;
    position: 'inline' | 'adjacent';
  };

  // Alert display
  alert?: {
    style: 'banner' | 'popup' | 'border' | 'fullscreen';
    animation: 'none' | 'pulse' | 'flash';
    flashRate: number;        // Hz (max 3 for safety)
  };
}
```

### 4.3 Priority-Based Display

```typescript
interface PriorityDisplayManager {
  // Display queue
  queue: DisplayQueueItem[];

  // Priority rules
  rules: {
    // Emergency always highest
    emergencyOverride: boolean;

    // Timeout for each priority level
    timeouts: Record<number, number>;

    // Stacking behavior
    stackBehavior: 'replace' | 'queue' | 'overlay';
  };
}

interface DisplayQueueItem {
  id: string;
  priority: number;
  content: MedicalContent;
  displayConfig: ContentDisplayMapping;
  timestamp: number;
  expiresAt?: number;
  acknowledged: boolean;
}
```

---

## 5. Voice-Sign Integration

### 5.1 Translation Pipeline

```
┌────────────────┐     ┌────────────────┐     ┌────────────────┐
│  Medical Data  │────►│  Text          │────►│  Sign Language │
│  (Numeric/     │     │  Generation    │     │  Translation   │
│   Alert)       │     │                │     │                │
└────────────────┘     └────────────────┘     └────────────────┘
                                                      │
                       ┌────────────────┐             │
                       │  Avatar/Video  │◄────────────┘
                       │  Output        │
                       └────────────────┘
```

### 5.2 Medical Terminology Support

```typescript
interface MedicalTerminologyConfig {
  // Supported sign languages
  signLanguage: SignLanguage;

  // Medical term dictionaries
  dictionaries: {
    conditions: MedicalTermDictionary;
    measurements: MedicalTermDictionary;
    actions: MedicalTermDictionary;
    alerts: MedicalTermDictionary;
  };

  // Translation preferences
  preferences: {
    // Use fingerspelling for unknown terms
    fingerspellUnknown: boolean;

    // Simplified mode for cognitive accessibility
    simplifiedMode: boolean;

    // Include numeric values in signs
    includeNumericValues: boolean;

    // Speed of signing
    signingSpeed: 'slow' | 'normal' | 'fast';
  };
}

interface MedicalTermDictionary {
  terms: MedicalTermEntry[];
  version: string;
  lastUpdated: number;
}

interface MedicalTermEntry {
  // Source term
  term: string;
  aliases: string[];

  // Sign language representation
  signData: {
    // Standard sign
    signId: string;

    // Alternative signs by region
    regionalVariants?: Record<string, string>;

    // Fingerspelling fallback
    fingerspelling?: string;
  };

  // Context
  context: {
    category: string;
    usage: string;
    examples: string[];
  };
}

// Example medical terms
const glucoseTerms: MedicalTermEntry[] = [
  {
    term: 'blood glucose',
    aliases: ['blood sugar', 'glucose level', 'BG'],
    signData: {
      signId: 'BLOOD_GLUCOSE_001',
      regionalVariants: {
        'ASL': 'BLOOD_GLUCOSE_ASL',
        'BSL': 'BLOOD_GLUCOSE_BSL'
      }
    },
    context: {
      category: 'measurement',
      usage: 'diabetes management',
      examples: ['Your blood glucose is 120']
    }
  },
  {
    term: 'hypoglycemia',
    aliases: ['low blood sugar', 'hypo'],
    signData: {
      signId: 'HYPOGLYCEMIA_001',
      fingerspelling: 'H-Y-P-O'
    },
    context: {
      category: 'condition',
      usage: 'diabetes emergency',
      examples: ['Warning: hypoglycemia detected']
    }
  }
];
```

### 5.3 Emergency Phrase Library

```typescript
interface EmergencyPhraseLibrary {
  phrases: EmergencyPhrase[];
  language: SignLanguage;
  version: string;
}

interface EmergencyPhrase {
  // Phrase identification
  id: string;
  category: 'medical' | 'help' | 'location' | 'identity';

  // Text content
  text: string;
  textSimple: string;

  // Sign data
  signData: {
    signSequence: string[];    // Sequence of sign IDs
    duration: number;          // Estimated duration in ms
    priority: number;          // Display priority
  };

  // Usage conditions
  conditions: {
    alertTypes?: string[];
    priorityLevels?: string[];
    autoTrigger?: boolean;
  };
}

// Example emergency phrases
const emergencyPhrases: EmergencyPhrase[] = [
  {
    id: 'EMERGENCY_LOW_SUGAR',
    category: 'medical',
    text: 'My blood sugar is very low. I need sugar immediately.',
    textSimple: 'Low sugar! Need sugar now!',
    signData: {
      signSequence: ['MY', 'BLOOD_SUGAR', 'VERY_LOW', 'NEED', 'SUGAR', 'NOW'],
      duration: 4000,
      priority: 1
    },
    conditions: {
      alertTypes: ['urgent_low_glucose'],
      priorityLevels: ['critical'],
      autoTrigger: true
    }
  },
  {
    id: 'EMERGENCY_CALL_HELP',
    category: 'help',
    text: 'Please call emergency services. I am having a medical emergency.',
    textSimple: 'Call 911! Emergency!',
    signData: {
      signSequence: ['PLEASE', 'CALL', 'EMERGENCY', 'HELP', 'MEDICAL'],
      duration: 3500,
      priority: 1
    },
    conditions: {
      priorityLevels: ['critical'],
      autoTrigger: true
    }
  }
];
```

---

## 6. Smart Wheelchair Integration

### 6.1 Positioning Protocol

```typescript
interface WheelchairIntegrationConfig {
  // Device identification
  medicalDeviceId: string;
  wheelchairId: string;

  // Positioning capabilities
  positioning: {
    heightAdjustable: boolean;
    tiltAdjustable: boolean;
    legRestAdjustable: boolean;
    autoPositioning: boolean;
  };

  // Medical device mounting
  mounting: {
    position: MountPosition;
    orientation: MountOrientation;
    quickRelease: boolean;
  };

  // Coordination settings
  coordination: WheelchairCoordinationConfig;
}

enum MountPosition {
  ARMREST_LEFT = 'armrest_left',
  ARMREST_RIGHT = 'armrest_right',
  TRAY = 'tray',
  SIDE_MOUNT = 'side_mount',
  CUSTOM = 'custom'
}

interface WheelchairCoordinationConfig {
  // Auto-height adjustment for medical procedures
  autoHeightForProcedure: {
    enabled: boolean;
    procedureHeights: Record<string, number>;  // procedure -> height in inches
  };

  // Alert-triggered adjustments
  alertAdjustments: {
    enabled: boolean;
    onCriticalAlert: {
      stopMovement: boolean;
      adjustToSafePosition: boolean;
      notifyCaregiver: boolean;
    };
  };

  // Accessibility features
  accessibility: {
    voiceControlEnabled: boolean;
    gestureControlEnabled: boolean;
    eyeTrackingEnabled: boolean;
  };
}
```

### 6.2 Medical Procedure Support

```typescript
interface MedicalProcedureSupport {
  procedures: ProcedureConfig[];
}

interface ProcedureConfig {
  // Procedure identification
  procedureId: string;
  name: string;
  deviceTypes: DeviceCategory[];

  // Wheelchair configuration
  wheelchairConfig: {
    // Required position
    position: {
      height: number;
      tilt: number;
      legRest: number;
    };

    // Movement restrictions
    restrictions: {
      lockPosition: boolean;
      disableManualControl: boolean;
      duration: number;        // seconds
    };
  };

  // User guidance
  guidance: {
    voiceInstructions: string[];
    visualInstructions: string[];
    hapticCues: string[];
  };
}

// Example: Blood pressure measurement
const bpMeasurementProcedure: ProcedureConfig = {
  procedureId: 'BP_MEASUREMENT',
  name: 'Blood Pressure Measurement',
  deviceTypes: [DeviceCategory.BloodPressureMonitor],
  wheelchairConfig: {
    position: {
      height: 18,              // inches
      tilt: 0,                 // level
      legRest: 90             // degrees
    },
    restrictions: {
      lockPosition: true,
      disableManualControl: false,
      duration: 120           // 2 minutes
    }
  },
  guidance: {
    voiceInstructions: [
      'Please remain still for blood pressure measurement',
      'Keep your arm relaxed at heart level',
      'Measurement will take about 30 seconds'
    ],
    visualInstructions: [
      'Sit still',
      'Relax arm',
      'Measuring...'
    ],
    hapticCues: [
      'gentle_pulse'          // Start measurement
    ]
  }
};
```

---

## 7. Multi-Device Coordination

### 7.1 Device Mesh Network

```typescript
interface DeviceMeshConfig {
  // Mesh identification
  meshId: string;
  userId: string;

  // Device registry
  devices: MeshDevice[];

  // Coordination rules
  rules: CoordinationRule[];

  // Failover configuration
  failover: FailoverConfig;
}

interface MeshDevice {
  deviceId: string;
  deviceType: string;
  role: 'primary' | 'secondary' | 'relay';
  capabilities: string[];
  priority: number;
  status: DeviceStatus;
}

interface CoordinationRule {
  ruleId: string;
  trigger: CoordinationTrigger;
  actions: CoordinationAction[];
  priority: number;
}

interface CoordinationTrigger {
  type: 'alert' | 'reading' | 'status_change' | 'schedule';
  conditions: Record<string, any>;
}

interface CoordinationAction {
  targetDevices: string[] | 'all';
  action: string;
  parameters: Record<string, any>;
  timeout: number;
}
```

### 7.2 Alert Cascade

```typescript
interface AlertCascadeConfig {
  // Cascade levels
  levels: CascadeLevel[];

  // Timing
  timing: {
    levelDelay: number;        // ms between levels
    acknowledgmentTimeout: number;
  };

  // Failover
  failover: {
    enabled: boolean;
    fallbackDevices: string[];
    externalNotification: boolean;
  };
}

interface CascadeLevel {
  level: number;
  devices: CascadeDevice[];
  requiresAcknowledgment: boolean;
}

interface CascadeDevice {
  deviceId: string;
  deviceType: string;
  outputConfig: {
    visual?: VisualAlertConfig;
    auditory?: AuditoryAlertConfig;
    haptic?: HapticAlertConfig;
  };
}

// Example cascade for critical alert
const criticalAlertCascade: AlertCascadeConfig = {
  levels: [
    {
      level: 1,
      devices: [
        {
          deviceId: 'wrist_exo',
          deviceType: 'exoskeleton',
          outputConfig: {
            haptic: { patternId: 'urgent_pulse', intensity: 1.0 }
          }
        }
      ],
      requiresAcknowledgment: true
    },
    {
      level: 2,
      devices: [
        {
          deviceId: 'bionic_eye',
          deviceType: 'bionic_eye',
          outputConfig: {
            visual: { style: 'fullscreen', animation: 'pulse' }
          }
        },
        {
          deviceId: 'voice_sign',
          deviceType: 'voice_sign',
          outputConfig: {
            auditory: { voiceEnabled: true, toneEnabled: true }
          }
        }
      ],
      requiresAcknowledgment: true
    },
    {
      level: 3,
      devices: [
        {
          deviceId: 'caregiver_app',
          deviceType: 'smartphone',
          outputConfig: {
            auditory: { voiceEnabled: true },
            visual: { style: 'popup' }
          }
        }
      ],
      requiresAcknowledgment: false
    }
  ],
  timing: {
    levelDelay: 15000,         // 15 seconds
    acknowledgmentTimeout: 30000
  },
  failover: {
    enabled: true,
    fallbackDevices: ['smartphone_primary'],
    externalNotification: true
  }
};
```

---

## 8. Testing and Validation

### 8.1 Integration Test Framework

```typescript
interface IntegrationTestSuite {
  // Test identification
  suiteId: string;
  version: string;

  // Device under test
  targetDevice: DeviceUnderTest;

  // WIA devices for testing
  testDevices: TestDevice[];

  // Test cases
  testCases: IntegrationTestCase[];
}

interface IntegrationTestCase {
  testId: string;
  name: string;
  category: 'connection' | 'data_flow' | 'alert' | 'failover';

  // Test steps
  steps: TestStep[];

  // Expected outcomes
  expectedOutcomes: ExpectedOutcome[];

  // Accessibility validation
  accessibilityChecks: AccessibilityCheck[];
}

interface AccessibilityCheck {
  checkId: string;
  category: 'visual' | 'auditory' | 'haptic' | 'cognitive';
  description: string;
  criteria: string;
  required: boolean;
}
```

---

## 9. References

- WIA Exoskeleton Protocol Specification v1.0
- WIA Bionic Eye Integration Guide v1.0
- WIA Voice-Sign Technical Reference v1.0
- WIA Smart Wheelchair API v1.0
- IEEE 11073 PHD Communication Model
