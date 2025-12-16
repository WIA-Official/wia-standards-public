# WIA Fintech Standard

**Financial Technology Accessibility Standards**

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)
[![Version](https://img.shields.io/badge/version-0.1.0-blue.svg)](https://github.com/WIA-Official/wia-standards)
[![Standard](https://img.shields.io/badge/standard-WIA%20FINTECH-orange.svg)](https://fintech.wia.live)

---

<div align="center">

💳 **Part of WIA Standards Ecosystem**

[WIA Standards Hub](https://wia.live/standards) • [API Portal](https://api.wia.live)

---

**弘益人間** - *Benefit All Humanity*

</div>

---

## 🌍 Overview

WIA Fintech is an open standard for financial technology accessibility standards.

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
| **2** | API Interface | Rust SDK for developers | ✅ Complete |
| **3** | Communication Protocol | Device protocols | ✅ Complete |
| **4** | Ecosystem Integration | WIA integration | ✅ Complete |

### Phase 1: Data Format (Complete)

- **Research**: ADA, EAA, WCAG banking compliance, ATM accessibility
- **Data Structures**: User financial accessibility profile, ATM profile
- **JSON Schemas**: User profile, ATM profile, accessible notification schemas
- **WIA Integration**: Exoskeleton, Bionic Eye, Voice-Sign financial support

### Phase 2: Rust API (Complete)

- **Types**: Comprehensive type definitions (~1200 lines)
- **Core Logic**: ProfileManager, ATMManager, AccessibilityScoreCalculator
- **Adapters**: SimulatorAdapter with sample data
- **Builders**: NotificationBuilder, UserProfileBuilder with fluent API
- **Tests**: 19 tests (unit, integration, doc tests)

### Phase 3: Communication Protocol (Complete)

- **Protocol Architecture**: REST API, WebSocket, BLE GATT for ATM/WIA
- **OpenAPI Spec**: Full API specification with accessibility extensions
- **Security**: OAuth2, mTLS, PCI-DSS compliance with accessible auth
- **WIA Integration**: Haptic feedback, Bionic Eye overlay, ATM guidance

### Phase 4: Ecosystem Integration (Complete)

- **WIA Unified Profile**: Financial domain extension for cross-domain sync
- **Device Integration**: Exoskeleton, Bionic Eye, Voice-Sign, Smart Wheelchair
- **Deployment Guide**: Kubernetes, ATM firmware, mobile app deployment
- **Certification Framework**: Bronze/Silver/Gold/Platinum levels

---

## 🚀 Quick Start

```typescript
import { UserFinancialAccessibilityProfile } from '@wia/fintech';

const userProfile: UserFinancialAccessibilityProfile = {
  profileId: "user-001",
  version: "1.0.0",
  personalInfo: {
    preferredLanguage: "en-US",
    region: "US"
  },
  accessibilityNeeds: {
    sensory: {
      visual: {
        level: "low-vision",
        preferences: {
          fontSize: "large",
          highContrast: true
        }
      }
    }
  },
  financialPreferences: {
    preferredAuthMethod: ["biometric_fingerprint", "pin"]
  }
};
```

---

## 📁 Structure

```
fintech/
├── spec/                    # Specifications
│   ├── RESEARCH-PHASE-1.md
│   ├── PHASE-1-DATA-FORMAT.md
│   ├── PHASE-3-COMMUNICATION-PROTOCOL.md
│   ├── PHASE-4-ECOSYSTEM-INTEGRATION.md
│   ├── SECURITY-SPECIFICATION.md
│   ├── DEPLOYMENT-GUIDE.md
│   ├── schemas/
│   │   ├── user-financial-accessibility-profile.schema.json
│   │   ├── atm-accessibility-profile.schema.json
│   │   └── accessible-notification.schema.json
│   └── api/
│       └── openapi.yaml     # OpenAPI 3.1 Specification
├── api/
│   └── rust/                # Rust SDK (wia-fintech crate)
│       ├── src/
│       │   ├── lib.rs
│       │   ├── types.rs
│       │   ├── error.rs
│       │   ├── core/
│       │   └── adapters/
│       ├── tests/
│       └── examples/
├── prompts/                 # Claude Code prompts
└── docs/
```

---

## 🔗 Links

| Resource | URL |
|----------|-----|
| **Website** | https://fintech.wia.live |
| **Standards Hub** | https://wia.live/standards |
| **GitHub** | https://github.com/WIA-Official/wia-standards/tree/main/fintech |

---

## 📜 License

MIT License - This standard belongs to humanity.

---

<div align="center">

🤟 **弘益人間** - Benefit All Humanity

© 2025 SmileStory Inc. / WIA

</div>
