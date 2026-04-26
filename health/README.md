# WIA Health Standard

**Health & Longevity Standards**

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)
[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)](https://github.com/WIA-Official/wia-standards)
[![Standard](https://img.shields.io/badge/standard-WIA%20HEALTH-orange.svg)](https://health.wia.live)

---

<div align="center">

💚 **Part of WIA Standards Ecosystem**

[WIA Standards Hub](https://wia.live/standards) • [API Portal](https://api.wia.live)

---

**홍익인간 (弘益人間)** - *Benefit All Humanity*

</div>

---

## 🌍 Overview

WIA Health is an open standard for health & longevity standards.

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
| **3** | Communication Protocol | Real-time streaming protocol | ✅ Complete |
| **4** | Ecosystem Integration | WIA integration | ⏳ Planned |

---

## 🚀 Quick Start

### Phase 1: Data Format

The WIA Health Standard defines comprehensive data formats for health and longevity data:

- **Biomarkers**: Inflammatory, metabolic, hormonal markers and aging clocks
- **Genomics**: Sequencing, variants, pharmacogenomics, polygenic risk scores
- **Epigenetics**: Methylation age, senescence markers, reprogramming history
- **Telomeres**: Length measurements, telomerase activity, interventions
- **Digital Twin**: Multi-modal health simulation and predictions
- **Interventions**: Longevity treatment tracking and outcomes

See [spec/PHASE-1-DATA-FORMAT.md](spec/PHASE-1-DATA-FORMAT.md) for detailed specifications.

### Phase 2: Rust SDK

High-performance Rust implementation with async support:

```rust
use wia_health::prelude::*;

// Create health profile
let profile = HealthProfileBuilder::new()
    .subject(subject)
    .biomarkers(biomarkers)
    .build()?;

// Calculate longevity index
let index = LongevityIndexCalculator::calculate(&profile)?;
println!("Longevity Index: {:.1}", index.overall);

// Run digital twin simulation
let manager = DigitalTwinManager::new();
let result = manager.predict_aging(&profile, 10.0).await?;
```

See [api/rust/README.md](api/rust/README.md) for detailed documentation.

### Phase 3: Communication Protocol

Real-time streaming protocol for health data exchange:

```rust
use wia_health::protocol::*;
use wia_health::transport::*;

// Create biomarker stream message
let biomarker = MessageBuilder::biomarker(BiomarkerPayload {
    stream_id: "stream-001".to_string(),
    subject_id: "patient-123".to_string(),
    sequence: 42,
    data: BiomarkerData {
        marker: "heart_rate".to_string(),
        value: 72.0,
        unit: "bpm".to_string(),
        timestamp: Utc::now().timestamp_millis(),
        source: Some(DataSource {
            device: "apple_watch".to_string(),
            model: Some("Series 9".to_string()),
            firmware: None,
        }),
        quality: Some(0.95),
        metadata: None,
    },
});

// Configure WebSocket transport
let config = TransportConfig::with_url("wss://api.wia-health.org/ws")
    .timeout(5000)
    .reconnect_delay(1000);
```

**Supported Transports:**
- WebSocket (primary) - Real-time streaming
- MQTT - IoT device integration
- BLE - Wearable devices
- FHIR REST - Healthcare system interoperability

See [spec/PHASE-3-PROTOCOL.md](spec/PHASE-3-PROTOCOL.md) for detailed specifications.

---

## 📁 Structure

```
health/
├── spec/
│   ├── RESEARCH-PHASE-1.md      # Phase 1 research findings
│   ├── PHASE-1-DATA-FORMAT.md   # Data format specification
│   ├── RESEARCH-PHASE-3.md      # Phase 3 protocol research
│   ├── PHASE-3-PROTOCOL.md      # Communication protocol spec
│   └── schemas/
│       ├── health-profile.schema.json
│       ├── biomarkers.schema.json
│       ├── genomics.schema.json
│       ├── epigenetics.schema.json
│       ├── telomeres.schema.json
│       ├── digital-twin.schema.json
│       ├── interventions.schema.json
│       └── common.schema.json
├── api/
│   └── rust/                    # Rust SDK
│       ├── Cargo.toml
│       ├── src/
│       │   ├── lib.rs
│       │   ├── types.rs
│       │   ├── error.rs
│       │   ├── core/           # Health calculators
│       │   ├── adapters/       # Digital twin simulation
│       │   ├── protocol/       # Phase 3 protocol messages
│       │   └── transport/      # WebSocket transport
│       ├── tests/
│       └── examples/
├── prompts/                     # Claude Code prompts
└── docs/
```

---

## 🔗 Links

| Resource | URL |
|----------|-----|
| **Website** | https://health.wia.live |
| **Standards Hub** | https://wia.live/standards |
| **GitHub** | https://github.com/WIA-Official/wia-standards/tree/main/health |

---

## 📜 License

MIT License - This standard belongs to humanity.

---

<div align="center">

🤟 **홍익인간 (弘益人間)** - Benefit All Humanity

© 2025 SmileStory Inc. / WIA

</div>
