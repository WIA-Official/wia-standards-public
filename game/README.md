# WIA Game Standard

**Gaming Accessibility Standards**

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)
[![Version](https://img.shields.io/badge/version-0.1.0-blue.svg)](https://github.com/WIA-Official/wia-standards)
[![Standard](https://img.shields.io/badge/standard-WIA%20GAME-orange.svg)](https://game.wia.live)

---

<div align="center">

🎮 **Part of WIA Standards Ecosystem**

[WIA Standards Hub](https://wia.live/standards) • [API Portal](https://api.wia.live)

---

**弘益人間** - *Benefit All Humanity*

</div>

---

## 🌍 Overview

WIA Game is an open standard for gaming accessibility standards.

This standard aims to:
- Unify data formats across the industry
- Provide standard APIs for developers  
- Enable interoperability between devices and systems
- Accelerate innovation through open collaboration

---

## 📋 Specification Phases

| Phase | Title | Description | Status |
|:-----:|-------|-------------|:------:|
| **1** | Data Format | JSON schemas for player profiles | ✅ Complete |
| **2** | API Interface | Rust SDK for developers | ✅ Complete |
| **3** | Communication Protocol | HID, eye tracking, switch access | ✅ Complete |
| **4** | Ecosystem Integration | BCI, AAC, platforms, cloud sync | ✅ Complete |

---

## 🚀 Quick Start

### Rust

```rust
use wia_game::{GameController, types::*};

fn main() {
    // Create controller
    let mut controller = GameController::new();

    // Create profile for blind user
    let profile = controller.create_profile_for_disability(DisabilityType::Blind);

    // Recommended settings auto-configured
    assert!(profile.visual_settings.screen_reader.enabled);
    assert!(profile.motor_settings.aim_assist.auto_aim);
}
```

### Device Protocol

```rust
use wia_game::protocol::*;

#[tokio::main]
async fn main() {
    let mut manager = ProtocolManager::new();

    // Add Xbox Adaptive Controller
    let adapter = XboxAdaptiveAdapter::simulated();
    manager.add_device(adapter.device_info().clone());

    // Process events with switch scanning
    manager.enable_switch_access();
}
```

---

## 📁 Structure

```
game/
├── spec/
│   ├── PHASE-1-DATA-FORMAT.md    # JSON schemas spec
│   ├── PHASE-3-PROTOCOL.md       # Device protocol spec
│   └── PHASE-4-INTEGRATION.md    # Ecosystem integration spec
├── api/
│   └── rust/                      # Rust SDK (wia-game)
│       ├── src/
│       │   ├── lib.rs
│       │   ├── types.rs           # Player profiles, settings
│       │   ├── core/              # Profile, preset, game managers
│       │   ├── protocol/          # HID, events, adapters
│       │   ├── ecosystem/         # BCI, AAC, platforms, cloud
│       │   └── adapters/          # Storage adapters
│       └── Cargo.toml
├── prompts/                       # Claude Code prompts
└── docs/
```

---

## 🔗 Links

| Resource | URL |
|----------|-----|
| **Website** | https://game.wia.live |
| **Standards Hub** | https://wia.live/standards |
| **GitHub** | https://github.com/WIA-Official/wia-standards/tree/main/game |

---

## 📜 License

MIT License - This standard belongs to humanity.

---

<div align="center">

🤟 **弘益人間** - Benefit All Humanity

© 2025 SmileStory Inc. / WIA

</div>
