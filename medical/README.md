# WIA Medical Standard

**Medical Device Accessibility Standards**

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)
[![Version](https://img.shields.io/badge/version-0.1.0-blue.svg)](https://github.com/WIA-Official/wia-standards)
[![Standard](https://img.shields.io/badge/standard-WIA%20MEDICAL-orange.svg)](https://medical.wia.live)

---

<div align="center">

🏥 **Part of WIA Standards Ecosystem**

[WIA Standards Hub](https://wia.live/standards) • [API Portal](https://api.wia.live)

---

**홍익인간 (弘益人間)** - *Benefit All Humanity*

</div>

---

## 🌍 Overview

WIA Medical is an open standard for medical device accessibility standards.

This standard aims to:
- Unify data formats across the industry
- Provide standard APIs for developers  
- Enable interoperability between devices and systems
- Accelerate innovation through open collaboration

---

## 📋 Specification Phases

| Phase | Title | Description | Status |
|:-----:|-------|-------------|:------:|
| **1** | Data Format | Standard data format | ✅ Complete |
| **2** | API Interface | SDK for developers | ✅ Complete |
| **3** | Communication Protocol | Device protocols | ✅ Complete |
| **4** | Ecosystem Integration | WIA integration | ✅ Complete |

### Phase 1: Data Format (Complete)

- **Research**: Regulatory environment (FDA, MDE Standards, ADA, IEC 62366)
- **Data Structures**: Medical device & user accessibility profiles
- **JSON Schemas**: Device profile, user profile, alarm system schemas

### Phase 2: Rust API (Complete)

- **Core Types**: MedicalDeviceAccessibilityProfile, UserMedicalAccessibilityProfile
- **Profile Management**: Device/user profile CRUD operations
- **Compatibility Matching**: Device-user accessibility compatibility checker
- **Score Calculator**: Accessibility score computation
- **WIA Integration**: Exoskeleton, Voice-Sign, Haptic device support

### Phase 3: Communication Protocol (Complete)

- **COMMUNICATION-PROTOCOL.md**: BLE GATT, WIA Protocol, Emergency Alert Protocol
- **DEVICE-INTEGRATION.md**: Exoskeleton, Bionic Eye, Voice-Sign, Smart Wheelchair
- **ALERT-TRANSMISSION.md**: Multi-modal alerts, priority levels, user preferences
- **HEALTHCARE-INTEROP.md**: HL7 FHIR, EHR integration, accessible data exchange

### Phase 4: Ecosystem Integration (Complete)

- **WIA-ECOSYSTEM-INTEGRATION.md**: Unified profile, cross-domain integration, device orchestration
- **CERTIFICATION-FRAMEWORK.md**: Bronze/Silver/Gold/Platinum levels, testing procedures
- **DEPLOYMENT-GUIDE.md**: Production deployment, Kubernetes, security, compliance

---

## 🚀 Quick Start

```rust
use wia_medical::prelude::*;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Create simulator with sample data
    let adapter = SimulatorAdapter::with_sample_data().await;

    // Get device profile
    let device = adapter.get_device_profile("cgm_dexcom_g7").await?;

    // Get user profile
    let user = adapter.get_user_profile("user_blind_001").await?;

    // Check compatibility
    let result = ProfileMatcher::is_compatible(&device, &user);
    println!("Compatible: {}, Score: {:.1}%", result.compatible, result.score);

    Ok(())
}
```

---

## 📁 Structure

```
medical/
├── spec/                    # Specifications
│   ├── PHASE-1-DATA-FORMAT.md
│   ├── RESEARCH-PHASE-1.md
│   ├── COMMUNICATION-PROTOCOL.md    # Phase 3
│   ├── DEVICE-INTEGRATION.md        # Phase 3
│   ├── ALERT-TRANSMISSION.md        # Phase 3
│   ├── HEALTHCARE-INTEROP.md        # Phase 3
│   ├── WIA-ECOSYSTEM-INTEGRATION.md # Phase 4
│   ├── CERTIFICATION-FRAMEWORK.md   # Phase 4
│   ├── DEPLOYMENT-GUIDE.md          # Phase 4
│   └── schemas/             # JSON Schemas
├── api/
│   └── rust/                # Rust SDK
│       ├── src/
│       │   ├── lib.rs
│       │   ├── types.rs     # Type definitions
│       │   ├── error.rs     # Error handling
│       │   ├── core/        # Core logic
│       │   └── adapters/    # Storage adapters
│       ├── tests/
│       └── examples/
├── prompts/                 # Claude Code prompts
└── docs/
```

---

## 🔗 Links

| Resource | URL |
|----------|-----|
| **Website** | https://medical.wia.live |
| **Standards Hub** | https://wia.live/standards |
| **GitHub** | https://github.com/WIA-Official/wia-standards/tree/main/medical |

---

## 📜 License

MIT License - This standard belongs to humanity.

---

<div align="center">

🤟 **홍익인간 (弘益人間)** - Benefit All Humanity

© 2025 SmileStory Inc. / WIA

</div>
