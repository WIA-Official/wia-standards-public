# WIA Autonomous Standard

**Autonomous Vehicle Accessibility Standards**

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)
[![Version](https://img.shields.io/badge/version-0.1.0-blue.svg)](https://github.com/WIA-Official/wia-standards)
[![Standard](https://img.shields.io/badge/standard-WIA%20AUTO-orange.svg)](https://auto.wia.live)

---

<div align="center">

🚗 **Part of WIA Standards Ecosystem**

[WIA Standards Hub](https://wia.live/standards) • [API Portal](https://api.wia.live)

---

**弘益人間** - *Benefit All Humanity*

</div>

---

## 🌍 Overview

WIA Autonomous is an open standard for autonomous vehicle accessibility standards.

This standard aims to:
- Unify data formats across the industry
- Provide standard APIs for developers  
- Enable interoperability between devices and systems
- Accelerate innovation through open collaboration

---

## 📋 Specification Phases

| Phase | Title | Description | Status |
|:-----:|-------|-------------|:------:|
| **1** | Data Format | Passenger profiles, vehicle capabilities, trip requests | ✅ Complete |
| **2** | API Interface | TypeScript/Rust SDK for developers | ⏳ Planned |
| **3** | Communication Protocol | V2X, Fleet management protocols | ⏳ Planned |
| **4** | Ecosystem Integration | WIA integration, Smart city | ⏳ Planned |

---

## 🚀 Quick Start

### Phase 1: Data Formats

```json
{
  "wia_auto": {
    "version": "1.0.0",
    "message_type": "trip_request",
    "payload": {
      "accessibility_requirements": {
        "wheelchair_accessible": true,
        "ramp_required": true,
        "preferred_modalities": ["audio_tts", "haptic_vibration"]
      }
    }
  }
}
```

See [spec/PHASE-1-DATA-FORMAT.md](spec/PHASE-1-DATA-FORMAT.md) for full specification.

---

## 📁 Structure

```
auto/
├── spec/
│   ├── RESEARCH-PHASE-1.md          # Research findings
│   ├── PHASE-1-DATA-FORMAT.md       # Data format specification
│   └── schemas/
│       ├── passenger-profile.schema.json
│       ├── vehicle-capabilities.schema.json
│       ├── trip-request.schema.json
│       ├── trip-response.schema.json
│       ├── hmi-config.schema.json
│       ├── securement-status.schema.json
│       ├── emergency-event.schema.json
│       └── message-envelope.schema.json
├── api/
│   ├── typescript/          # TypeScript SDK (Phase 2)
│   └── rust/                # Rust SDK (Phase 2)
├── prompts/                 # Claude Code prompts
└── README.md
```

---

## 🔗 Links

| Resource | URL |
|----------|-----|
| **Website** | https://auto.wia.live |
| **Standards Hub** | https://wia.live/standards |
| **GitHub** | https://github.com/WIA-Official/wia-standards/tree/main/auto |

---

## 📜 License

MIT License - This standard belongs to humanity.

---

<div align="center">

🤟 **弘益人間** - Benefit All Humanity

© 2025 SmileStory Inc. / WIA

</div>
