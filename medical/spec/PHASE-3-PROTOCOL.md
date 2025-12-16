# WIA Medical Device Accessibility: Communication Protocol Specification

## 1. Overview

본 문서는 의료기기 접근성 데이터의 통신 프로토콜 표준을 정의합니다.

**Version:** 1.0.0
**Status:** Draft
**Phase:** 3 of 4

---

## 2. Protocol Architecture

### 2.1 Communication Layers

```
┌─────────────────────────────────────────────────────────────────┐
│                    Application Layer                             │
│         (Profile Exchange, Alarm Sync, Data Sharing)            │
├─────────────────────────────────────────────────────────────────┤
│                    Accessibility Layer                           │
│    (Multi-Modal Output, Input Adaptation, WIA Integration)      │
├─────────────────────────────────────────────────────────────────┤
│                    Transport Layer                               │
│         (BLE, WiFi, USB, HL7 FHIR, Proprietary)                │
├─────────────────────────────────────────────────────────────────┤
│                    Physical Layer                                │
│              (Bluetooth 5.0+, WiFi, USB-C)                      │
└─────────────────────────────────────────────────────────────────┘
```

### 2.2 Communication Endpoints

| Endpoint Type | Description | Protocol |
|--------------|-------------|----------|
| **Device-to-App** | 의료기기 ↔ 스마트폰 앱 | BLE GATT |
| **Device-to-Device** | 의료기기 ↔ WIA 기기 | WIA Protocol |
| **Device-to-Cloud** | 의료기기 ↔ 클라우드 | HTTPS/WSS |
| **Device-to-EHR** | 의료기기 ↔ 의료시스템 | HL7 FHIR |

---

## 3. Bluetooth LE GATT Profile

### 3.1 WIA Medical Accessibility Service

```
Service UUID: 0x1850 (WIA Medical Accessibility)

Characteristics:
├── Accessibility Profile (0x2B50)
│   ├── Properties: Read, Write, Notify
│   └── Format: JSON (compressed)
│
├── User Preferences (0x2B51)
│   ├── Properties: Read, Write
│   └── Format: JSON
│
├── Alert Configuration (0x2B52)
│   ├── Properties: Read, Write, Notify
│   └── Format: Binary
│
├── Multi-Modal Output (0x2B53)
│   ├── Properties: Notify
│   └── Format: Binary (Audio/Haptic patterns)
│
├── WIA Device Link (0x2B54)
│   ├── Properties: Read, Write, Notify
│   └── Format: WIA Protocol Frame
│
└── Emergency Alert (0x2B55)
    ├── Properties: Notify
    └── Format: Binary (High Priority)
```

### 3.2 Characteristic Definitions

#### 3.2.1 Accessibility Profile Characteristic

```typescript
interface AccessibilityProfileCharacteristic {
  // Header
  header: {
    version: number;           // Protocol version (1)
    flags: number;             // Feature flags
    timestamp: number;         // Unix timestamp
  };

  // Compressed profile data
  payload: {
    compression: 'none' | 'gzip' | 'lz4';
    data: Uint8Array;          // Compressed JSON
  };

  // Checksum
  checksum: number;            // CRC32
}
```

#### 3.2.2 Alert Configuration Characteristic

```typescript
interface AlertConfigCharacteristic {
  // Alert settings per modality
  visual: {
    enabled: boolean;
    brightness: number;        // 0-100
    flashRate: number;         // Hz (max 3 for photosensitivity)
  };

  auditory: {
    enabled: boolean;
    volume: number;            // 0-100
    voiceEnabled: boolean;
    toneFrequency: number;     // Hz
  };

  haptic: {
    enabled: boolean;
    intensity: number;         // 0-100
    wiaDeviceId?: string;      // WIA device target
  };

  // Priority thresholds
  priorities: {
    critical: AlertOutput;
    high: AlertOutput;
    medium: AlertOutput;
    low: AlertOutput;
  };
}

interface AlertOutput {
  modalities: ('visual' | 'auditory' | 'haptic')[];
  escalationEnabled: boolean;
  escalationDelay: number;     // seconds
}
```

### 3.3 GATT Operations

#### Connection Sequence

```
┌──────────────┐                              ┌──────────────┐
│  Medical     │                              │  Smartphone  │
│  Device      │                              │  App         │
└──────┬───────┘                              └──────┬───────┘
       │                                              │
       │  1. Advertising (Service UUID 0x1850)        │
       │◄─────────────────────────────────────────────│
       │                                              │
       │  2. Connection Request                       │
       │◄─────────────────────────────────────────────│
       │                                              │
       │  3. Service Discovery                        │
       │─────────────────────────────────────────────►│
       │                                              │
       │  4. Read Accessibility Profile               │
       │─────────────────────────────────────────────►│
       │                                              │
       │  5. Write User Preferences                   │
       │◄─────────────────────────────────────────────│
       │                                              │
       │  6. Subscribe to Alerts                      │
       │◄─────────────────────────────────────────────│
       │                                              │
       │  7. Ongoing Notifications                    │
       │─────────────────────────────────────────────►│
       │                                              │
```

---

## 4. WIA Device Integration Protocol

### 4.1 WIA Protocol Frame Format

```
┌────────────────────────────────────────────────────────────────┐
│  Byte 0   │  Byte 1-2  │  Byte 3   │  Byte 4-N  │  Byte N+1-2 │
├───────────┼────────────┼───────────┼────────────┼─────────────┤
│  Header   │  Length    │  Type     │  Payload   │  CRC16      │
│  (0xWA)   │  (uint16)  │  (uint8)  │  (varies)  │             │
└────────────────────────────────────────────────────────────────┘
```

### 4.2 Message Types

| Type | Name | Direction | Description |
|------|------|-----------|-------------|
| 0x01 | PROFILE_SYNC | Bidirectional | 접근성 프로필 동기화 |
| 0x02 | ALERT_FORWARD | Medical → WIA | 알림 전달 |
| 0x03 | HAPTIC_COMMAND | Medical → WIA | 햅틱 명령 |
| 0x04 | INPUT_EVENT | WIA → Medical | 입력 이벤트 |
| 0x05 | STATUS_REQUEST | Bidirectional | 상태 조회 |
| 0x06 | STATUS_RESPONSE | Bidirectional | 상태 응답 |
| 0x10 | EMERGENCY | Medical → WIA | 긴급 알림 |
| 0x11 | EMERGENCY_ACK | WIA → Medical | 긴급 확인 |

### 4.3 Exoskeleton Integration

#### Haptic Command Message

```typescript
interface HapticCommandMessage {
  type: 0x03;
  payload: {
    // Target body location
    target: HapticTarget;

    // Pattern specification
    pattern: {
      id: string;              // Predefined pattern ID
      // OR custom pattern
      custom?: {
        vibrations: number[];  // [on_ms, off_ms, on_ms, ...]
        intensity: number[];   // Intensity for each vibration
      };
    };

    // Timing
    startDelay: number;        // ms
    repeatCount: number;

    // Context
    context: {
      alarmPriority: AlarmPriority;
      medicalEvent: string;
      value?: number;          // e.g., glucose level
    };
  };
}

enum HapticTarget {
  LEFT_WRIST = 0x01,
  RIGHT_WRIST = 0x02,
  LEFT_ARM = 0x03,
  RIGHT_ARM = 0x04,
  LEFT_HAND = 0x05,
  RIGHT_HAND = 0x06,
  TORSO = 0x07,
  BACK = 0x08,
  CUSTOM = 0xFF
}
```

### 4.4 Voice-Sign Integration

#### Translation Request Message

```typescript
interface VoiceSignRequestMessage {
  type: 0x20;
  payload: {
    // Source content
    source: {
      type: 'text' | 'medical_reading' | 'alarm';
      content: string;
      language: string;
    };

    // Translation preferences
    preferences: {
      signLanguage: SignLanguage;
      medicalTermsMode: boolean;
      simplifiedMode: boolean;
    };

    // Priority
    priority: 'normal' | 'urgent' | 'emergency';

    // Context
    context?: {
      deviceType: string;
      measurementType: string;
      value?: number;
      unit?: string;
    };
  };
}

enum SignLanguage {
  ASL = 0x01,
  BSL = 0x02,
  KSL = 0x03,
  JSL = 0x04,
  DGS = 0x05,
  LSF = 0x06,
  AUSLAN = 0x07
}
```

### 4.5 Bionic Eye Integration

#### Display Optimization Message

```typescript
interface BionicEyeOptimizationMessage {
  type: 0x30;
  payload: {
    // Content to display
    content: {
      type: 'reading' | 'graph' | 'alert' | 'menu';
      data: any;
    };

    // Optimization parameters
    optimization: {
      contrastEnhancement: number;    // 0.0 - 2.0
      edgeEnhancement: number;        // 0.0 - 1.0
      simplification: number;         // 0.0 - 1.0
      colorMapping: ColorMappingMode;
      focusRegion?: Rectangle;
    };

    // Priority regions
    priorityAreas: {
      primary: Rectangle;
      secondary?: Rectangle;
    };
  };
}

enum ColorMappingMode {
  NORMAL = 0x00,
  HIGH_CONTRAST = 0x01,
  GRAYSCALE = 0x02,
  PROTANOPIA_SAFE = 0x03,
  DEUTERANOPIA_SAFE = 0x04,
  TRITANOPIA_SAFE = 0x05
}
```

---

## 5. Emergency Alert Protocol

### 5.1 Emergency Levels

| Level | Name | Response Time | Modalities |
|-------|------|---------------|------------|
| 0 | CRITICAL | < 100ms | All + Escalation |
| 1 | URGENT | < 500ms | All |
| 2 | HIGH | < 1s | User preference |
| 3 | MEDIUM | < 5s | User preference |
| 4 | LOW | Best effort | User preference |

### 5.2 Emergency Message Format

```typescript
interface EmergencyMessage {
  type: 0x10;
  payload: {
    // Emergency identification
    header: {
      level: EmergencyLevel;
      code: number;            // Predefined emergency code
      timestamp: number;
      sequence: number;        // For deduplication
    };

    // Medical data
    medical: {
      eventType: string;       // 'hypoglycemia', 'fall', etc.
      value?: number;
      unit?: string;
      trend?: 'rising' | 'falling' | 'stable';
      deviceId: string;
    };

    // Alert outputs
    outputs: {
      visual: {
        color: string;
        pattern: string;
        text: string;
        textSimple: string;    // Cognitive accessibility
      };

      auditory: {
        toneId: string;
        voiceText: string;
        voiceTextSimple: string;
        repeatCount: number;
      };

      haptic: {
        patternId: string;
        intensity: number;
        targets: HapticTarget[];
      };
    };

    // Escalation plan
    escalation: {
      enabled: boolean;
      steps: EscalationStep[];
    };

    // Localization
    localization: {
      language: string;
      messages: Record<string, string>;
    };
  };
}
```

### 5.3 Emergency Acknowledgment Flow

```
┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│  Medical     │     │  WIA         │     │  Caregiver   │
│  Device      │     │  Device      │     │  App         │
└──────┬───────┘     └──────┬───────┘     └──────┬───────┘
       │                     │                    │
       │  EMERGENCY (0x10)   │                    │
       ├────────────────────►│                    │
       │                     │                    │
       │  EMERGENCY (0x10)   │                    │
       ├─────────────────────┼───────────────────►│
       │                     │                    │
       │                     │  User Interaction  │
       │                     │◄───────────────────│
       │                     │                    │
       │  EMERGENCY_ACK      │                    │
       │◄────────────────────┤                    │
       │                     │                    │
       │                     │  EMERGENCY_ACK     │
       │◄────────────────────┼────────────────────│
       │                     │                    │
       │  Stop Escalation    │                    │
       │                     │                    │
```

---

## 6. Data Synchronization Protocol

### 6.1 Profile Synchronization

```typescript
interface ProfileSyncProtocol {
  // Sync request
  request: {
    type: 'full' | 'delta';
    lastSyncTimestamp?: number;
    profileTypes: ('device' | 'user' | 'alarm')[];
  };

  // Sync response
  response: {
    status: 'success' | 'conflict' | 'error';
    profiles: SyncedProfile[];
    conflicts?: ConflictInfo[];
    nextSyncToken: string;
  };
}

interface SyncedProfile {
  type: string;
  id: string;
  version: string;
  timestamp: number;
  data: any;
  checksum: string;
}

interface ConflictInfo {
  profileId: string;
  localVersion: string;
  remoteVersion: string;
  conflictType: 'update' | 'delete';
  resolution: 'local_wins' | 'remote_wins' | 'merge' | 'manual';
}
```

### 6.2 Real-time Data Streaming

```typescript
interface DataStreamConfig {
  // Stream identification
  streamId: string;
  deviceId: string;

  // Data types to stream
  dataTypes: DataStreamType[];

  // Streaming parameters
  parameters: {
    sampleRate: number;        // Hz
    bufferSize: number;        // samples
    compression: 'none' | 'delta' | 'lossy';
  };

  // Accessibility adaptations
  accessibility: {
    voiceReadingInterval: number;  // seconds, 0 = disabled
    hapticFeedbackEnabled: boolean;
    trendNotifications: boolean;
  };
}

enum DataStreamType {
  GLUCOSE = 0x01,
  BLOOD_PRESSURE = 0x02,
  HEART_RATE = 0x03,
  SPO2 = 0x04,
  TEMPERATURE = 0x05,
  WEIGHT = 0x06,
  ACTIVITY = 0x07,
  SLEEP = 0x08
}
```

---

## 7. Security Protocol

### 7.1 Authentication

```typescript
interface AuthenticationProtocol {
  // Device pairing
  pairing: {
    method: 'pin' | 'oob' | 'just_works';
    pinLength: number;
    accessiblePinEntry: boolean;  // Voice/haptic feedback
  };

  // Session authentication
  session: {
    method: 'token' | 'certificate';
    tokenLifetime: number;     // seconds
    refreshEnabled: boolean;
  };

  // Encryption
  encryption: {
    algorithm: 'AES-128-GCM' | 'AES-256-GCM';
    keyExchange: 'ECDH-P256';
  };
}
```

### 7.2 Accessible Security Features

```typescript
interface AccessibleSecurityFeatures {
  // PIN entry with accessibility
  pinEntry: {
    voiceFeedback: boolean;
    hapticFeedback: boolean;
    largeButtons: boolean;
    extendedTimeout: number;   // seconds
  };

  // Security notifications
  notifications: {
    voiceAlerts: boolean;
    visualAlerts: boolean;
    hapticAlerts: boolean;
    simplifiedMessages: boolean;
  };

  // Emergency override
  emergencyOverride: {
    enabled: boolean;
    bypassPinForEmergency: boolean;
    caregiverAccess: boolean;
  };
}
```

---

## 8. Error Handling

### 8.1 Error Codes

| Code | Name | Description | Recovery |
|------|------|-------------|----------|
| 0x01 | CONN_FAILED | 연결 실패 | 재연결 시도 |
| 0x02 | AUTH_FAILED | 인증 실패 | 재인증 요청 |
| 0x03 | PROFILE_INVALID | 프로필 무효 | 프로필 재동기화 |
| 0x04 | SYNC_CONFLICT | 동기화 충돌 | 충돌 해결 |
| 0x05 | TIMEOUT | 시간 초과 | 재시도 |
| 0x10 | WIA_NOT_FOUND | WIA 기기 미발견 | 검색 재시도 |
| 0x11 | WIA_INCOMPATIBLE | WIA 버전 불일치 | 업데이트 알림 |
| 0x20 | EMERGENCY_FAILED | 긴급 알림 실패 | 대체 경로 시도 |

### 8.2 Error Recovery with Accessibility

```typescript
interface AccessibleErrorRecovery {
  // Error notification
  notification: {
    visual: {
      icon: string;
      text: string;
      textSimple: string;      // Cognitive accessibility
      highContrast: boolean;
    };

    auditory: {
      tonePattern: string;
      voiceMessage: string;
      voiceMessageSimple: string;
    };

    haptic: {
      pattern: string;
      intensity: number;
    };
  };

  // Recovery guidance
  guidance: {
    steps: RecoveryStep[];
    voiceGuidance: boolean;
    stepByStep: boolean;
  };

  // Escalation on failure
  escalation: {
    enabled: boolean;
    notifyCaregiver: boolean;
    autoRetryCount: number;
  };
}

interface RecoveryStep {
  order: number;
  action: string;
  description: string;
  descriptionSimple: string;
  voiceInstruction: string;
  estimatedTime: number;       // seconds
}
```

---

## 9. Protocol Compliance

### 9.1 Required Features

| Feature | Priority | Accessibility Impact |
|---------|----------|---------------------|
| Multi-modal alerts | Required | Visual/auditory/haptic alternatives |
| Profile sync | Required | User preferences consistency |
| Emergency protocol | Required | Life-critical alerts |
| WIA integration | Recommended | Enhanced accessibility |
| Voice output | Recommended | Blind user support |
| Haptic patterns | Recommended | Deaf user support |
| Simple messages | Recommended | Cognitive accessibility |

### 9.2 Conformance Levels

| Level | Requirements |
|-------|-------------|
| **Basic** | BLE GATT, Multi-modal alerts, Profile sync |
| **Standard** | Basic + WIA integration, Emergency protocol |
| **Advanced** | Standard + Full WIA suite, Real-time streaming |

---

## 10. References

- Bluetooth Core Specification v5.3
- HL7 FHIR R4 Specification
- WIA Exoskeleton Protocol v1.0
- WIA Voice-Sign Protocol v1.0
- WIA Haptic Standard v1.0
- IEEE 11073 Personal Health Devices
