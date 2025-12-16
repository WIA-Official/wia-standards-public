# WIA Smart Home Standard

**Smart Home Accessibility Standards**

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)
[![Version](https://img.shields.io/badge/version-0.1.0-blue.svg)](https://github.com/WIA-Official/wia-standards)
[![Standard](https://img.shields.io/badge/standard-WIA%20SMARTHOME-orange.svg)](https://smarthome.wia.live)

---

<div align="center">

🏠 **Part of WIA Standards Ecosystem**

[WIA Standards Hub](https://wia.live/standards) • [API Portal](https://api.wia.live)

---

**弘益人間** - *Benefit All Humanity*

</div>

---

## 🌍 Overview

WIA Smart Home is an open standard for smart home accessibility standards.

This standard aims to:
- Unify data formats across the industry
- Provide standard APIs for developers
- Enable interoperability between devices and systems
- Accelerate innovation through open collaboration

---

## 📋 Specification Phases

| Phase | Title | Description | Status |
|:-----:|-------|-------------|:------:|
| **1** | Data Format | JSON Schema for accessibility | ✅ Complete |
| **2** | API Interface | Rust SDK for developers | ✅ Complete |
| **3** | Communication Protocol | Matter + accessibility extensions | ✅ Complete |
| **4** | Ecosystem Integration | WIA ecosystem + external platforms | ✅ Complete |

---

## 🚀 Quick Start

### Phase 1: Data Format Schemas

All schemas are available in `spec/schemas/`:

| Schema | Description |
|--------|-------------|
| `user-profile.schema.json` | User accessibility profiles and preferences |
| `accessibility-requirements.schema.json` | Disability types and assistive tech requirements |
| `device.schema.json` | Smart home devices with accessibility features |
| `home.schema.json` | Home configuration with global accessibility settings |
| `zone.schema.json` | Room/zone with accessibility overrides |
| `automation.schema.json` | Automation rules with accessibility considerations |
| `notification.schema.json` | Multi-modal notifications |
| `accessibility-event.schema.json` | Accessibility event tracking |

### Phase 2: Rust API

```toml
[dependencies]
wia-smarthome = "0.1.0"
```

```rust
use wia_smarthome::{SmartHomeController, types::*, adapters::*};
use std::sync::Arc;

#[tokio::main]
async fn main() {
    // Create controller with simulator
    let adapter = Arc::new(SimulatorDeviceAdapter::new());
    let notification = Arc::new(SimulatorNotificationService::new());

    let controller = SmartHomeController::new()
        .with_device_adapter(adapter)
        .with_notification_service(notification);

    // Create user profile with accessibility settings
    let mut profile = controller.create_profile().await.unwrap();
    profile.accessibility_requirements.primary_disabilities
        .push(DisabilityType::VisualLowVision);
    profile.interaction_preferences.preferred_output_modalities
        .push(OutputModality::AudioTts);

    // Create home
    let home = controller
        .create_home("My Home".to_string(), profile.profile_id)
        .await.unwrap();
}
```

### Phase 3: Protocol

```rust
use wia_smarthome::protocol::*;

// Matter adapter for device communication
let adapter = MatterAdapter::new(0);

// Device discovery
let discovery = DiscoveryService::new();
discovery.start_scan();

// Accessibility extensions
let voice_cluster = VoiceCommandCluster::default();
let audio_feedback = AudioFeedbackCluster::default();
```

### Phase 4: Ecosystem Integration

```rust
use wia_smarthome::ecosystem::*;

// Eye Gaze control
let mut eye_gaze = EyeGazeAdapter::new();
eye_gaze.create_grid_layout(2, 3, devices);

// BCI control
let mut bci = BCIAdapter::new();
bci.setup_default_ssvep(devices);

// AAC voice commands
let aac = AACAdapter::new();
let result = aac.process_voice(VoiceCommand {
    text: "거실의 불을 켜줘".to_string(),
    language: Language::Korean,
    confidence: 0.9,
    alternatives: vec![],
});

// Smart wheelchair location automation
let mut wheelchair = WheelchairAdapter::new();
wheelchair.setup_living_room_automation(zone_id, light_id, None);

// External platforms (Alexa, Google Home, HomeKit)
let bridge = PlatformBridge::new()
    .with_alexa(AlexaAdapter::new(config))
    .with_google(GoogleHomeAdapter::new(config))
    .with_homekit(HomeKitAdapter::new(config));
```

### Key Features

- **Multi-modal interaction**: Voice, touch, switch, eye-gaze, BCI support
- **Disability-aware**: Visual, hearing, motor, cognitive accessibility
- **Matter Protocol compatible**: Integration with CSA-IoT Matter standard
- **WCAG-aligned**: Following web accessibility guidelines principles
- **Korean language support**: ko-KR TTS and voice commands

---

## 📁 Structure

```
smarthome/
├── spec/
│   ├── schemas/             # JSON Schema definitions
│   │   ├── user-profile.schema.json
│   │   ├── accessibility-requirements.schema.json
│   │   ├── device.schema.json
│   │   ├── home.schema.json
│   │   ├── zone.schema.json
│   │   ├── automation.schema.json
│   │   ├── notification.schema.json
│   │   └── accessibility-event.schema.json
│   ├── PHASE-1-DATA-FORMAT.md
│   ├── PHASE-3-PROTOCOL.md
│   ├── PHASE-4-INTEGRATION.md
│   └── RESEARCH-PHASE-1.md
├── api/
│   └── rust/                # Rust SDK
│       ├── Cargo.toml
│       ├── src/
│       │   ├── lib.rs       # Main library
│       │   ├── types.rs     # Type definitions
│       │   ├── error.rs     # Error types
│       │   ├── core/        # Core implementation
│       │   ├── adapters/    # Device adapters
│       │   ├── protocol/    # Phase 3: Matter protocol
│       │   │   ├── matter/
│       │   │   ├── discovery/
│       │   │   └── accessibility/
│       │   └── ecosystem/   # Phase 4: Ecosystem integration
│       │       ├── eye_gaze.rs
│       │       ├── bci.rs
│       │       ├── aac.rs
│       │       ├── wheelchair.rs
│       │       ├── exoskeleton.rs
│       │       ├── haptic.rs
│       │       └── external/
│       │           ├── alexa.rs
│       │           ├── google.rs
│       │           └── homekit.rs
│       ├── tests/
│       └── examples/
├── prompts/                 # Claude Code prompts
└── docs/
```

---

## 🔗 Links

| Resource | URL |
|----------|-----|
| **Website** | https://smarthome.wia.live |
| **Standards Hub** | https://wia.live/standards |
| **GitHub** | https://github.com/WIA-Official/wia-standards/tree/main/smarthome |

---

## 📜 License

MIT License - This standard belongs to humanity.

---

<div align="center">

🤟 **弘益人間** - Benefit All Humanity

© 2025 SmileStory Inc. / WIA

</div>
