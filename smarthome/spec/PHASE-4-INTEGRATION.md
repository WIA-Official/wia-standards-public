# WIA Smart Home Phase 4: Ecosystem Integration

**Version**: 1.0.0
**Status**: Draft
**Last Updated**: 2025-01-XX

---

## 1. Overview

Phase 4 integrates WIA Smart Home with the broader WIA assistive technology ecosystem and external smart home platforms, enabling seamless control for users with diverse disabilities.

### 1.1 Design Principles

- **Universal Access**: Every integration supports multiple input modalities
- **Graceful Degradation**: Fallback to simpler controls when advanced methods unavailable
- **User Autonomy**: User maintains control over automation level
- **Privacy First**: Local processing where possible, explicit consent for cloud

---

## 2. Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     WIA Smart Home Hub                          │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐             │
│  │  Eye Gaze   │  │    BCI      │  │    AAC      │             │
│  │  Adapter    │  │   Adapter   │  │   Adapter   │             │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘             │
│         │                │                │                     │
│  ┌──────┴────────────────┴────────────────┴──────┐             │
│  │           Unified Command Processor            │             │
│  └──────────────────────┬────────────────────────┘             │
│                         │                                       │
│  ┌──────────────────────┴────────────────────────┐             │
│  │              Device Controller                 │             │
│  │  (Phase 1-3: Types, Core, Protocol)           │             │
│  └──────────────────────┬────────────────────────┘             │
│                         │                                       │
├─────────────────────────┼───────────────────────────────────────┤
│                         ▼                                       │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐             │
│  │  Wheelchair │  │ Exoskeleton │  │   Haptic    │             │
│  │   Bridge    │  │   Bridge    │  │   Bridge    │             │
│  └─────────────┘  └─────────────┘  └─────────────┘             │
│                                                                 │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐             │
│  │   Alexa     │  │   Google    │  │  HomeKit    │             │
│  │   Skill     │  │    Home     │  │   Bridge    │             │
│  └─────────────┘  └─────────────┘  └─────────────┘             │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## 3. WIA Eye Gaze Integration

### 3.1 Gaze-to-Device Mapping

```
┌────────────────────────────────────────┐
│           Screen Layout                │
│  ┌──────────┬──────────┬──────────┐   │
│  │  Light   │   Fan    │   TV     │   │
│  │  Zone 1  │  Zone 2  │  Zone 3  │   │
│  ├──────────┼──────────┼──────────┤   │
│  │  Door    │ Thermo-  │  Blind   │   │
│  │  Zone 4  │  stat 5  │  Zone 6  │   │
│  └──────────┴──────────┴──────────┘   │
│                                        │
│  Gaze Point (x, y) → Zone → Device    │
└────────────────────────────────────────┘
```

### 3.2 Interaction Patterns

| Pattern | Description | Use Case |
|---------|-------------|----------|
| **Dwell Select** | Look at device for N ms to select | Primary selection |
| **Gaze + Blink** | Gaze to select, blink to confirm | Motor impaired users |
| **Gaze + Switch** | Gaze to select, switch to confirm | High accuracy needed |
| **Smooth Pursuit** | Follow moving target to control | Dimmer, thermostat |

### 3.3 Protocol Messages

```json
{
  "type": "eye_gaze_command",
  "timestamp": "2025-01-15T10:30:00Z",
  "gaze_point": { "x": 0.75, "y": 0.25 },
  "dwell_duration_ms": 1200,
  "pattern": "dwell_select",
  "target_device": "uuid:device-123",
  "confidence": 0.95,
  "calibration_quality": "good"
}
```

---

## 4. WIA BCI Integration

### 4.1 Supported Paradigms

| Paradigm | Signal | Control Type |
|----------|--------|--------------|
| **SSVEP** | Steady-State Visual Evoked Potential | Device selection |
| **P300** | Event-Related Potential | Menu selection |
| **Motor Imagery** | μ/β rhythm modulation | On/Off, directional |
| **Error Potential** | Automatic error correction | Undo actions |

### 4.2 Intent Mapping

```
┌─────────────────────────────────────┐
│        BCI Intent Mapping           │
├─────────────────────────────────────┤
│  Motor Imagery Left  → Turn Off     │
│  Motor Imagery Right → Turn On      │
│  P300 Flash Grid     → Select Item  │
│  SSVEP 12Hz          → Device A     │
│  SSVEP 15Hz          → Device B     │
│  Error Potential     → Undo Last    │
└─────────────────────────────────────┘
```

### 4.3 Safety Constraints

- **Confidence Threshold**: Min 0.7 for non-critical, 0.9 for safety-critical
- **Confirmation Required**: Door locks, security systems
- **Timeout**: Auto-cancel after 5s without confirmation
- **Fatigue Detection**: Reduce session after 30min continuous use

---

## 5. WIA AAC Integration

### 5.1 Symbol-to-Command Mapping

| Symbol Category | Example Symbols | Commands |
|-----------------|-----------------|----------|
| **Lighting** | 💡 🌙 ☀️ | on, off, dim, bright |
| **Climate** | 🌡️ ❄️ 🔥 | temp up, temp down, fan |
| **Security** | 🔒 🔓 🚪 | lock, unlock, open |
| **Entertainment** | 📺 🔊 🎵 | TV, volume, music |
| **Emergency** | 🆘 📞 🚨 | help, call, alarm |

### 5.2 Voice Command Grammar

```
Korean (ko-KR):
  "{장소}의 {장치}를 {동작}해줘"
  Examples:
    - "거실의 불을 켜줘"
    - "침실의 에어컨을 꺼줘"
    - "현관문을 잠가줘"

English (en-US):
  "Turn {action} the {device} in the {location}"
  Examples:
    - "Turn on the lights in the living room"
    - "Turn off the AC in the bedroom"
    - "Lock the front door"
```

### 5.3 Feedback Generation

```json
{
  "feedback_type": "command_result",
  "success": true,
  "message": {
    "ko": "거실 조명을 켰습니다",
    "en": "Living room lights turned on"
  },
  "output_modalities": ["tts", "symbol", "haptic"],
  "symbol": "💡✅",
  "haptic_pattern": "success_short"
}
```

---

## 6. WIA Smart Wheelchair Integration

### 6.1 Location-Based Automation

```
┌─────────────────────────────────────────┐
│         Home Floor Plan                 │
│                                         │
│   ┌─────────┐    ┌─────────────────┐   │
│   │ Bedroom │    │   Living Room   │   │
│   │  Zone A │    │     Zone B      │   │
│   │         │    │                 │   │
│   └────┬────┘    └────────┬────────┘   │
│        │                  │             │
│   ┌────┴──────────────────┴────────┐   │
│   │         Hallway Zone C          │   │
│   └────┬──────────────────┬────────┘   │
│        │                  │             │
│   ┌────┴────┐        ┌────┴────┐       │
│   │ Kitchen │        │Bathroom │       │
│   │ Zone D  │        │ Zone E  │       │
│   └─────────┘        └─────────┘       │
│                                         │
│  Wheelchair enters Zone → Trigger      │
└─────────────────────────────────────────┘
```

### 6.2 Automation Rules

| Trigger | Condition | Action |
|---------|-----------|--------|
| Enter Zone | Wheelchair enters | Lights on, adjust temp |
| Exit Zone | Wheelchair leaves | Lights off (delay 5min) |
| Approach Door | Within 2m of door | Open door automatically |
| Path Request | Navigation started | Light path corridor |
| Emergency | Tilt detected | Alert, unlock doors |

### 6.3 Coordinated Control

```rust
// Wheelchair position triggers smart home automation
pub struct WheelchairLocationTrigger {
    pub zone_id: ZoneId,
    pub enter_actions: Vec<DeviceAction>,
    pub exit_actions: Vec<DeviceAction>,
    pub proximity_actions: Vec<ProximityAction>,
}

pub struct ProximityAction {
    pub device_id: DeviceId,
    pub trigger_distance_m: f32,
    pub action: DeviceAction,
}
```

---

## 7. WIA Exoskeleton Integration

### 7.1 Mobility State Detection

| State | Smart Home Response |
|-------|---------------------|
| **Seated** | Standard automation |
| **Standing** | Adjust counter-height devices |
| **Walking** | Path lighting, clear obstacles |
| **Stairs** | Enhanced lighting, rail heating |
| **Transitioning** | Pause automation, safety check |

### 7.2 Balance-Assist Environment

```json
{
  "balance_assist_mode": {
    "enabled": true,
    "stabilization_lighting": true,
    "floor_obstacle_alert": true,
    "handrail_locations": ["hallway", "bathroom"],
    "emergency_grab_points": ["kitchen_counter", "bathroom_bar"]
  }
}
```

---

## 8. WIA Haptic Integration

### 8.1 Feedback Patterns

| Event | Pattern | Duration | Intensity |
|-------|---------|----------|-----------|
| Device On | Double pulse | 200ms | Medium |
| Device Off | Single pulse | 100ms | Light |
| Error | Triple pulse | 300ms | Strong |
| Alert | Continuous | 1000ms | Strong |
| Navigation | Directional | Variable | Medium |

### 8.2 Device State Encoding

```
Haptic Encoding for Device States:
┌────────────────────────────────────┐
│  Light Level: Pulse frequency      │
│    Off   = No pulse                │
│    25%   = 1 pulse/sec             │
│    50%   = 2 pulse/sec             │
│    75%   = 3 pulse/sec             │
│    100%  = 4 pulse/sec             │
├────────────────────────────────────┤
│  Temperature: Intensity gradient   │
│    Cold  = Light, slow             │
│    Warm  = Medium, moderate        │
│    Hot   = Strong, fast            │
├────────────────────────────────────┤
│  Door State: Pattern               │
│    Locked   = Firm double tap      │
│    Unlocked = Soft single tap      │
│    Open     = Sweeping motion      │
└────────────────────────────────────┘
```

---

## 9. External Platform Integration

### 9.1 Amazon Alexa

```json
{
  "alexa_skill": {
    "skill_id": "amzn1.ask.skill.wia-smarthome",
    "supported_interfaces": [
      "Alexa.PowerController",
      "Alexa.BrightnessController",
      "Alexa.ThermostatController",
      "Alexa.LockController"
    ],
    "accessibility_features": {
      "slow_speech": true,
      "confirmation_required": true,
      "korean_support": true
    }
  }
}
```

### 9.2 Google Home

```json
{
  "google_home_action": {
    "project_id": "wia-smarthome",
    "supported_traits": [
      "action.devices.traits.OnOff",
      "action.devices.traits.Brightness",
      "action.devices.traits.TemperatureSetting",
      "action.devices.traits.LockUnlock"
    ],
    "fulfillment_url": "https://api.wia.live/smarthome/google"
  }
}
```

### 9.3 Apple HomeKit

```json
{
  "homekit_bridge": {
    "accessory_id": "wia-smarthome-bridge",
    "category": "bridge",
    "supported_services": [
      "Lightbulb",
      "Thermostat",
      "LockMechanism",
      "GarageDoorOpener"
    ],
    "accessibility_services": [
      "VoiceOver",
      "SwitchControl",
      "DwellControl"
    ]
  }
}
```

---

## 10. Unified Command Protocol

### 10.1 Command Structure

```rust
pub struct UnifiedCommand {
    pub id: Uuid,
    pub source: CommandSource,
    pub timestamp: DateTime<Utc>,
    pub target: CommandTarget,
    pub action: DeviceAction,
    pub context: AccessibilityContext,
    pub priority: CommandPriority,
    pub confirmation: ConfirmationRequirement,
}

pub enum CommandSource {
    EyeGaze { gaze_point: GazePoint, dwell_ms: u32 },
    BCI { paradigm: BCIParadigm, confidence: f32 },
    AAC { symbol: Option<String>, voice: Option<String> },
    Wheelchair { zone: ZoneId, trigger: TriggerType },
    Exoskeleton { state: MobilityState },
    Haptic { gesture: HapticGesture },
    Alexa { intent: String, slots: HashMap<String, String> },
    GoogleHome { trait_name: String, params: Value },
    HomeKit { characteristic: String, value: Value },
}
```

### 10.2 Priority Resolution

| Priority | Sources | Override |
|----------|---------|----------|
| **Emergency** | Any emergency trigger | Always |
| **Safety** | BCI high-confidence, Voice | Override automation |
| **User** | Eye Gaze, AAC, Haptic | Override automation |
| **Automation** | Wheelchair, Exoskeleton | Lowest |

---

## 11. Security Considerations

### 11.1 Authentication per Source

| Source | Auth Method |
|--------|-------------|
| Eye Gaze | Device pairing + calibration verification |
| BCI | Neural signature verification |
| AAC | Voice print + device pairing |
| Wheelchair | Proximity + device certificate |
| External | OAuth 2.0 + API keys |

### 11.2 Command Verification

```rust
pub struct CommandVerification {
    pub source_authenticated: bool,
    pub user_confirmed: bool,
    pub safety_check_passed: bool,
    pub within_permissions: bool,
    pub rate_limit_ok: bool,
}
```

---

## 12. Implementation Checklist

- [ ] Eye Gaze adapter with dwell selection
- [ ] BCI adapter with SSVEP/P300 support
- [ ] AAC adapter with Korean/English commands
- [ ] Wheelchair location-based automation
- [ ] Exoskeleton mobility state integration
- [ ] Haptic feedback patterns
- [ ] Alexa skill connector
- [ ] Google Home action connector
- [ ] HomeKit bridge connector
- [ ] Unified command processor
- [ ] Priority resolution engine
- [ ] Security authentication layer
- [ ] Integration test suite

---

## 13. References

- WIA Eye Gaze Standard v1.0
- WIA BCI Standard v1.0
- WIA AAC Standard v1.0
- WIA Smart Wheelchair Standard v1.0
- WIA Exoskeleton Standard v1.0
- WIA Haptic Standard v1.0
- Amazon Alexa Smart Home API v3
- Google Smart Home API
- Apple HomeKit Accessory Protocol Specification

---

**弘益人間** - Accessible Smart Homes for Everyone
