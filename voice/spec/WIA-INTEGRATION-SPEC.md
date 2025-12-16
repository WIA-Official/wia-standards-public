# WIA Voice-Sign Integration Specification

Version: 1.0.0
Status: Draft
Last Updated: 2025-01-15

## 1. Overview

This specification defines integration patterns between WIA Voice-Sign and other WIA standards, including Exoskeleton and Bionic Eye systems.

## 2. WIA Ecosystem Architecture

### 2.1 System Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                        WIA Ecosystem                            │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐        │
│  │ Voice-Sign  │◀──▶│ WIA Event   │◀──▶│ Exoskeleton │        │
│  │   System    │    │    Bus      │    │   System    │        │
│  └─────────────┘    └─────────────┘    └─────────────┘        │
│         │                  │                  │                 │
│         │                  │                  │                 │
│         │          ┌───────┴───────┐          │                 │
│         │          │               │          │                 │
│         ▼          ▼               ▼          ▼                 │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐        │
│  │ Bionic Eye  │    │  Identity   │    │    Data     │        │
│  │   System    │    │   Service   │    │    Lake     │        │
│  └─────────────┘    └─────────────┘    └─────────────┘        │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### 2.2 Integration Points

| Integration | Direction | Protocol | Use Case |
|-------------|-----------|----------|----------|
| Voice-Sign → Exoskeleton | Uni | Event Bus | Sign gesture commands |
| Voice-Sign → Bionic Eye | Uni | Event Bus | Visual sign display |
| Exoskeleton → Voice-Sign | Uni | Event Bus | Gesture-to-sign feedback |
| Voice-Sign ↔ Identity | Bi | gRPC | User authentication |
| All → Data Lake | Uni | Event Bus | Analytics aggregation |

## 3. WIA Event Bus

### 3.1 Event Bus Specification

```yaml
event_bus:
  type: Apache Kafka
  version: "3.6+"

  topics:
    voice_sign:
      - wia.voice-sign.translation.requested
      - wia.voice-sign.translation.completed
      - wia.voice-sign.emergency.detected
      - wia.voice-sign.quality.alert

    exoskeleton:
      - wia.exoskeleton.gesture.detected
      - wia.exoskeleton.movement.command
      - wia.exoskeleton.safety.alert

    bionic_eye:
      - wia.bionic-eye.display.request
      - wia.bionic-eye.recognition.result

    system:
      - wia.system.health.status
      - wia.system.config.changed

  configuration:
    partitions: 12
    replication_factor: 3
    retention_hours: 168  # 7 days
    compression: lz4
```

### 3.2 Event Schema (CloudEvents)

```json
{
  "specversion": "1.0",
  "type": "wia.voice-sign.translation.completed",
  "source": "/wia/voice-sign/api/v1",
  "id": "evt-2025-0115-001234",
  "time": "2025-01-15T10:30:00Z",
  "datacontenttype": "application/json",
  "subject": "translation/req-12345",
  "wiaversion": "1.0.0",
  "wiacorrelationid": "corr-abcd1234",
  "data": {
    "request_id": "req-12345",
    "source_language": "en",
    "target_language": "ASL",
    "status": "success",
    "quality_score": 0.95,
    "gloss_sequence": ["HELLO", "HOW", "YOU"],
    "pose_available": true
  }
}
```

### 3.3 Event Types

#### Voice-Sign Events

```typescript
// Translation events
interface TranslationRequestedEvent {
  type: "wia.voice-sign.translation.requested";
  data: {
    request_id: string;
    source_language: string;
    target_language: string;
    input_type: "audio" | "text";
    priority: "normal" | "high" | "emergency";
  };
}

interface TranslationCompletedEvent {
  type: "wia.voice-sign.translation.completed";
  data: {
    request_id: string;
    source_language: string;
    target_language: string;
    status: "success" | "partial" | "failed";
    quality_score: number;
    gloss_sequence: string[];
    pose_available: boolean;
    render_available: boolean;
    duration_ms: number;
  };
}

// Emergency events
interface EmergencyDetectedEvent {
  type: "wia.voice-sign.emergency.detected";
  data: {
    request_id: string;
    urgency_level: "low" | "medium" | "high" | "critical";
    keywords: string[];
    action: string;
    requires_immediate_response: boolean;
  };
}
```

#### Cross-System Events

```typescript
// Sign gesture command to exoskeleton
interface SignGestureCommandEvent {
  type: "wia.integration.sign-gesture.command";
  data: {
    source: "voice-sign";
    target: "exoskeleton";
    gesture: {
      type: "sign_language";
      gloss: string;
      pose_data: PoseData;
      timing: TimingInfo;
    };
    user_consent: boolean;
    assistance_level: "full" | "partial" | "guide";
  };
}

// Visual display request to bionic eye
interface VisualDisplayRequestEvent {
  type: "wia.integration.visual-display.request";
  data: {
    source: "voice-sign";
    target: "bionic-eye";
    display: {
      type: "sign_avatar" | "gloss_text" | "caption";
      content: any;
      position: "center" | "peripheral";
      duration_ms: number;
    };
    priority: "normal" | "urgent";
  };
}
```

## 4. Exoskeleton Integration

### 4.1 Gesture Feedback Loop

```
┌─────────────┐          ┌─────────────┐          ┌─────────────┐
│  User       │ speaks   │ Voice-Sign  │ gesture  │ Exoskeleton │
│  Speech     │─────────▶│  System     │─────────▶│  System     │
└─────────────┘          └─────────────┘          └─────────────┘
                                │                        │
                                │                        │
                                │     ┌──────────────────┘
                                │     │ movement feedback
                                ▼     ▼
                         ┌─────────────┐
                         │   User      │
                         │ (signing)   │
                         └─────────────┘
```

### 4.2 Gesture Command Protocol

```rust
/// Sign gesture command for exoskeleton
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SignGestureCommand {
    /// Unique command ID
    pub command_id: String,

    /// Source request ID from voice-sign
    pub source_request_id: String,

    /// Sign being produced
    pub sign: SignInfo,

    /// Target joint angles
    pub joint_targets: Vec<JointTarget>,

    /// Timing information
    pub timing: GestureTiming,

    /// Safety parameters
    pub safety: GestureSafety,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SignInfo {
    pub gloss: String,
    pub notation: Option<String>,  // HamNoSys
    pub hand_shape: HandShape,
    pub movement_type: MovementType,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct JointTarget {
    pub joint_id: String,
    pub angle_degrees: f32,
    pub velocity_limit: f32,
    pub torque_limit: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GestureTiming {
    /// Start time (relative to command receipt)
    pub start_delay_ms: u32,
    /// Duration of the gesture
    pub duration_ms: u32,
    /// Transition type
    pub transition: TransitionType,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum TransitionType {
    Smooth,
    Sharp,
    Hold,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GestureSafety {
    /// Maximum force allowed
    pub max_force_n: f32,
    /// Emergency stop enabled
    pub emergency_stop: bool,
    /// User can override
    pub user_override: bool,
    /// Collision detection enabled
    pub collision_detection: bool,
}
```

### 4.3 Assistance Modes

```yaml
assistance_modes:
  full_assist:
    description: "Exoskeleton produces complete sign"
    user_control: minimal
    use_case: "Learning or mobility impaired users"

  partial_assist:
    description: "Exoskeleton assists with difficult movements"
    user_control: shared
    use_case: "Users with partial mobility"

  guide_mode:
    description: "Gentle guidance without force"
    user_control: full
    use_case: "Learning to sign"

  mirror_mode:
    description: "Follow user's movements with feedback"
    user_control: full
    use_case: "Practice and correction"
```

### 4.4 Safety Constraints

```yaml
safety_constraints:
  force_limits:
    wrist: 10N
    elbow: 15N
    shoulder: 20N
    finger: 2N

  velocity_limits:
    wrist: 90deg/s
    elbow: 120deg/s
    shoulder: 90deg/s
    finger: 180deg/s

  range_of_motion:
    enforce: true
    buffer_degrees: 5

  emergency_stop:
    button: physical
    voice_command: "stop"
    gesture_command: true
    automatic: on_resistance

  collision_avoidance:
    enabled: true
    zones:
      - face
      - body
      - obstacles
```

## 5. Bionic Eye Integration

### 5.1 Visual Display Pipeline

```
┌─────────────┐          ┌─────────────┐          ┌─────────────┐
│ Voice-Sign  │ display  │  Display    │ render   │ Bionic Eye  │
│  System     │─────────▶│  Composer   │─────────▶│  Display    │
└─────────────┘          └─────────────┘          └─────────────┘
      │                        │
      │                        │
      │ avatar                 │ layout
      │ render                 │ optimization
      ▼                        ▼
┌─────────────┐          ┌─────────────┐
│  Avatar     │          │   Visual    │
│  Renderer   │          │   Encoder   │
└─────────────┘          └─────────────┘
```

### 5.2 Display Request Protocol

```rust
/// Display request for bionic eye
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BionicEyeDisplayRequest {
    /// Request ID
    pub request_id: String,

    /// Display content
    pub content: DisplayContent,

    /// Display parameters
    pub parameters: DisplayParameters,

    /// Priority
    pub priority: DisplayPriority,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum DisplayContent {
    /// Sign language avatar
    SignAvatar {
        /// Rendered avatar data
        avatar_data: AvatarData,
        /// Loop count (0 = infinite)
        loop_count: u32,
    },

    /// Gloss text overlay
    GlossText {
        /// Gloss sequence
        glosses: Vec<String>,
        /// Highlight current
        highlight_current: bool,
    },

    /// Caption text
    Caption {
        /// Caption text
        text: String,
        /// Language
        language: String,
    },

    /// Combined display
    Combined {
        avatar: Option<AvatarData>,
        gloss: Option<Vec<String>>,
        caption: Option<String>,
    },
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DisplayParameters {
    /// Position in visual field
    pub position: DisplayPosition,
    /// Size (percentage of field)
    pub size_percent: f32,
    /// Opacity (0.0 - 1.0)
    pub opacity: f32,
    /// Duration in ms (0 = until dismissed)
    pub duration_ms: u32,
    /// Accessibility settings
    pub accessibility: AccessibilitySettings,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum DisplayPosition {
    Center,
    TopLeft,
    TopRight,
    BottomLeft,
    BottomRight,
    Peripheral,
    Custom { x: f32, y: f32 },
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum DisplayPriority {
    Background,
    Normal,
    Important,
    Urgent,
    Emergency,
}
```

### 5.3 Visual Accessibility

```yaml
visual_accessibility:
  contrast:
    minimum_ratio: 7.0  # WCAG AAA
    high_contrast_mode: available

  sizing:
    minimum_avatar_size: 15%  # of visual field
    scalable: true
    maximum_size: 50%

  positioning:
    avoid_center_vision: configurable
    peripheral_option: true
    customizable: true

  colors:
    color_blind_safe: true
    customizable_palette: true

  motion:
    reduce_motion_option: true
    pause_control: always_available
```

## 6. Cross-Standard Data Formats

### 6.1 Unified User Profile

```typescript
interface WiaUserProfile {
  // Identity
  user_id: string;
  wia_id: string;  // Unified WIA identifier

  // Preferences
  preferences: {
    voice_sign: VoiceSignPreferences;
    exoskeleton: ExoskeletonPreferences;
    bionic_eye: BionicEyePreferences;
  };

  // Accessibility needs
  accessibility: {
    hearing: HearingProfile;
    vision: VisionProfile;
    mobility: MobilityProfile;
    cognitive: CognitiveProfile;
  };

  // Connected devices
  devices: {
    exoskeleton?: ExoskeletonDevice;
    bionic_eye?: BionicEyeDevice;
    other: WiaDevice[];
  };
}

interface VoiceSignPreferences {
  preferred_sign_language: string;
  source_languages: string[];
  playback_speed: number;
  quality_threshold: number;
  emergency_contacts: EmergencyContact[];
}
```

### 6.2 Shared Pose Format

```typescript
// Unified pose format for cross-system use
interface WiaPoseData {
  format_version: "1.0.0";
  timestamp: string;
  source_system: "voice-sign" | "exoskeleton" | "bionic-eye";

  // Skeleton data
  skeleton: {
    joints: Joint[];
    bones: Bone[];
    coordinate_system: "right-handed-y-up";
  };

  // Hand data (detailed)
  hands: {
    left: HandPose;
    right: HandPose;
  };

  // Face data (for sign language)
  face?: {
    landmarks: FaceLandmark[];
    expression: FacialExpression;
  };

  // Metadata
  metadata: {
    confidence: number;
    source_sign?: string;
    timing?: PoseTiming;
  };
}
```

## 7. Authentication & Authorization

### 7.1 WIA Identity Service

```yaml
identity_service:
  protocol: OpenID Connect
  provider: WIA Identity Provider

  endpoints:
    authorization: https://id.wia.org/oauth2/authorize
    token: https://id.wia.org/oauth2/token
    userinfo: https://id.wia.org/oauth2/userinfo
    introspect: https://id.wia.org/oauth2/introspect

  scopes:
    - openid
    - profile
    - wia.voice-sign
    - wia.exoskeleton
    - wia.bionic-eye
    - wia.integration

  claims:
    - sub (subject - WIA user ID)
    - wia_id (unified identifier)
    - devices (connected devices)
    - accessibility_profile
```

### 7.2 Service-to-Service Authentication

```yaml
service_auth:
  type: mTLS + JWT

  mtls:
    ca: /etc/wia/ca.crt
    cert: /etc/wia/service.crt
    key: /etc/wia/service.key

  jwt:
    issuer: https://id.wia.org
    audience: wia-services
    expiry: 1h
    claims:
      - service_id
      - scopes
      - instance_id
```

### 7.3 Authorization Policies

```yaml
authorization:
  engine: OPA (Open Policy Agent)

  policies:
    voice_sign_to_exoskeleton:
      action: send_gesture_command
      conditions:
        - user_consent: required
        - device_paired: true
        - safety_check: passed

    voice_sign_to_bionic_eye:
      action: display_content
      conditions:
        - device_connected: true
        - display_permission: granted

    emergency_override:
      action: emergency_broadcast
      conditions:
        - emergency_detected: true
        - urgency: critical
      override:
        - normal_permissions
```

## 8. API Gateway Patterns

### 8.1 Gateway Architecture

```yaml
api_gateway:
  type: Kong / Envoy

  routes:
    /api/v1/voice-sign/*:
      upstream: voice-sign-service
      plugins:
        - rate-limiting
        - jwt-auth
        - request-transformer

    /api/v1/integration/*:
      upstream: integration-service
      plugins:
        - rate-limiting
        - jwt-auth
        - cors

  rate_limiting:
    default:
      requests_per_minute: 100
      burst: 20

    emergency:
      requests_per_minute: 1000
      burst: 100

  cors:
    origins:
      - https://app.wia.org
      - https://dashboard.wia.org
    methods: [GET, POST, PUT, DELETE]
    headers: [Authorization, Content-Type, X-Request-ID]
```

### 8.2 API Versioning

```yaml
versioning:
  strategy: url-path
  format: /api/v{major}/

  versions:
    v1:
      status: current
      deprecation: null

    v2:
      status: beta
      features:
        - enhanced_pose_format
        - realtime_streaming_v2

  deprecation_policy:
    notice_period: 12_months
    sunset_period: 6_months
```

## 9. Error Handling

### 9.1 Cross-System Error Codes

```yaml
error_codes:
  # Integration errors (7xxx)
  7001:
    name: INTEGRATION_TARGET_UNAVAILABLE
    message: "Target system is not available"
    http_status: 503

  7002:
    name: INTEGRATION_PERMISSION_DENIED
    message: "Permission denied for cross-system operation"
    http_status: 403

  7003:
    name: INTEGRATION_DATA_FORMAT_MISMATCH
    message: "Data format incompatible between systems"
    http_status: 422

  7004:
    name: INTEGRATION_TIMEOUT
    message: "Cross-system operation timed out"
    http_status: 504

  7005:
    name: INTEGRATION_USER_NOT_LINKED
    message: "User accounts not linked between systems"
    http_status: 400

  # Device errors (7100)
  7101:
    name: DEVICE_NOT_CONNECTED
    message: "Target device is not connected"
    http_status: 503

  7102:
    name: DEVICE_BUSY
    message: "Target device is busy with another operation"
    http_status: 409

  7103:
    name: DEVICE_SAFETY_LOCK
    message: "Device is in safety lock mode"
    http_status: 423
```

### 9.2 Error Response Format

```json
{
  "error": {
    "code": "7001",
    "name": "INTEGRATION_TARGET_UNAVAILABLE",
    "message": "Target system is not available",
    "details": {
      "target_system": "exoskeleton",
      "last_seen": "2025-01-15T10:25:00Z",
      "retry_after_seconds": 30
    },
    "request_id": "req-12345",
    "correlation_id": "corr-abcd",
    "timestamp": "2025-01-15T10:30:00Z"
  }
}
```

## 10. Testing Integration

### 10.1 Integration Test Framework

```rust
#[cfg(test)]
mod integration_tests {
    use super::*;
    use wia_test_harness::*;

    #[tokio::test]
    async fn test_voice_sign_to_exoskeleton_gesture() {
        // Setup mock exoskeleton service
        let exo_mock = MockExoskeletonService::new()
            .expect_gesture_command()
            .returning(|cmd| Ok(GestureAck { accepted: true }));

        // Create integration bridge
        let bridge = WiaBridge::new()
            .with_exoskeleton(exo_mock)
            .build();

        // Send gesture command
        let command = SignGestureCommand {
            command_id: "cmd-001".to_string(),
            sign: SignInfo {
                gloss: "HELLO".to_string(),
                ..Default::default()
            },
            ..Default::default()
        };

        let result = bridge.send_gesture_command(command).await;
        assert!(result.is_ok());
    }

    #[tokio::test]
    async fn test_emergency_broadcast() {
        // Setup mock services
        let exo_mock = MockExoskeletonService::new()
            .expect_emergency_alert();
        let eye_mock = MockBionicEyeService::new()
            .expect_emergency_display();

        let bridge = WiaBridge::new()
            .with_exoskeleton(exo_mock)
            .with_bionic_eye(eye_mock)
            .build();

        // Trigger emergency
        let emergency = EmergencyEvent {
            urgency: UrgencyLevel::Critical,
            message: "Help needed".to_string(),
        };

        let result = bridge.broadcast_emergency(emergency).await;
        assert!(result.all_systems_notified());
    }
}
```

### 10.2 Contract Testing

```yaml
contract_tests:
  provider: voice-sign
  consumers:
    - exoskeleton
    - bionic-eye

  pacts:
    - name: voice-sign-exoskeleton
      interactions:
        - description: "Send gesture command"
          request:
            method: POST
            path: /api/v1/integration/gesture
          response:
            status: 202

    - name: voice-sign-bionic-eye
      interactions:
        - description: "Request display"
          request:
            method: POST
            path: /api/v1/integration/display
          response:
            status: 200
```
