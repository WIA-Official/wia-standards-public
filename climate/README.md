# WIA Climate Standard

**Climate & Environment Standards**

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)
[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)](https://github.com/WIA-Official/wia-standards)
[![Standard](https://img.shields.io/badge/standard-WIA%20CLIMATE-orange.svg)](https://climate.wia.live)

---

<div align="center">

🌍 **Part of WIA Standards Ecosystem**

[WIA Standards Hub](https://wia.live/standards) • [API Portal](https://api.wia.live)

---

**弘益人間** - *Benefit All Humanity*

</div>

---

## 🌍 Overview

WIA Climate is an open standard for climate & environment data exchange. It provides a unified JSON-based format for diverse climate technologies including carbon capture, weather modification, geoengineering, vertical farming, ocean cleanup, and climate modeling.

**Key Features**:
- **Unified Data Format**: JSON-based standard for all climate/environment data
- **6 Domain Coverage**: Carbon Capture, Weather Control, Geoengineering, Vertical Farming, Ocean Cleanup, Climate Modeling
- **CMIP6 Compatible**: Aligned with international climate data standards
- **Extensible Design**: Easy to add new data types and fields

---

## 📋 Specification Phases

| Phase | Title | Description | Status |
|:-----:|-------|-------------|:------:|
| **1** | Data Format | Standard data format | ✅ Complete |
| **2** | API Interface | Rust SDK for developers | ✅ Complete |
| **3** | Communication Protocol | WebSocket/MQTT protocols | ✅ Complete |
| **4** | Ecosystem Integration | Dashboards, Storage, Alerts | ✅ Complete |

---

## 🚀 Quick Start

### Data Format Example

```json
{
    "version": "1.0.0",
    "type": "carbon_capture",
    "timestamp": {
        "unix_ms": 1702468800000,
        "iso8601": "2024-12-14T12:00:00.000Z"
    },
    "location": {
        "latitude": 64.0,
        "longitude": -21.0,
        "altitude_m": 100
    },
    "device": {
        "manufacturer": "Climeworks",
        "model": "Orca DAC"
    },
    "data": {
        "technology": "dac",
        "capture_rate_kg_per_hour": 125.5,
        "co2_purity_percentage": 99.2
    }
}
```

### Supported Data Types

| Type | Description |
|------|-------------|
| `carbon_capture` | Carbon capture, utilization & storage (CCUS/DAC) |
| `weather_control` | Weather modification & cloud seeding |
| `geoengineering` | Solar radiation management & CDR |
| `vertical_farming` | Indoor agriculture & CEA |
| `ocean_cleanup` | Marine pollution removal |
| `climate_model` | Climate simulation data (CMIP6) |

### Rust API Example

```rust
use wia_climate::prelude::*;

fn main() -> Result<()> {
    let message = ClimateMessage::builder()
        .location(Location::new(64.0, -21.0))
        .device(Device::new("Climeworks", "Orca DAC"))
        .carbon_capture_data(CarbonCaptureData {
            technology: CarbonCaptureTechnology::Dac,
            capture_rate_kg_per_hour: 125.5,
            ..Default::default()
        })
        .build()?;

    let json = message.to_json_pretty()?;
    println!("{}", json);
    Ok(())
}
```

### WebSocket Client Example (Phase 3)

```rust
use wia_climate::prelude::*;
use wia_climate::transport::WebSocketClient;

#[tokio::main]
async fn main() -> Result<()> {
    // Connect to WIA Climate server
    let mut client = WebSocketClient::new("my-sensor");
    client.connect("wss://api.example.com/wia-climate/v1/ws").await?;

    // Create and send climate data
    let message = ClimateMessage::builder()
        .location(Location::new(64.0, -21.0))
        .device(Device::new("Climeworks", "Orca DAC"))
        .carbon_capture_data(CarbonCaptureData::default())
        .build()?;

    client.send_data(&message).await?;
    client.disconnect().await?;
    Ok(())
}
```

---

## 📁 Structure

```
climate/
├── spec/
│   ├── RESEARCH-PHASE-1.md       # Phase 1 research findings
│   ├── RESEARCH-PHASE-3.md       # Phase 3 protocol research
│   ├── PHASE-1-DATA-FORMAT.md    # Data format specification
│   ├── PHASE-3-PROTOCOL.md       # Communication protocol spec
│   └── schemas/                  # JSON Schema files
├── api/
│   └── rust/                     # Rust SDK (Phase 2 & 3)
│       ├── Cargo.toml
│       ├── src/
│       │   ├── protocol/         # Protocol implementation
│       │   └── transport/        # Transport layers (WebSocket)
│       ├── tests/
│       └── examples/
├── examples/                     # Sample data
├── prompts/                      # Claude Code prompts
└── docs/
```

---

## 📖 Documentation

| Document | Description |
|----------|-------------|
| [RESEARCH-PHASE-1.md](spec/RESEARCH-PHASE-1.md) | Technology research & analysis |
| [PHASE-1-DATA-FORMAT.md](spec/PHASE-1-DATA-FORMAT.md) | Data format specification |
| [RESEARCH-PHASE-3.md](spec/RESEARCH-PHASE-3.md) | Communication protocol research |
| [PHASE-3-PROTOCOL.md](spec/PHASE-3-PROTOCOL.md) | Protocol specification |
| [Rust API README](api/rust/README.md) | Rust SDK documentation |

---

## 🔗 Links

| Resource | URL |
|----------|-----|
| **Website** | https://climate.wia.live |
| **Standards Hub** | https://wia.live/standards |
| **GitHub** | https://github.com/WIA-Official/wia-standards/tree/main/climate |

---

## 🌐 Related Standards

| Standard | Relevance |
|----------|-----------|
| [CF Conventions](https://cfconventions.org/) | Climate data metadata |
| [CMIP6](https://pcmdi.llnl.gov/CMIP6/) | Climate model intercomparison |
| [ISO 27914](https://www.iso.org/standard/64148.html) | Carbon capture & storage |
| [GHG Protocol](https://ghgprotocol.org/) | Greenhouse gas accounting |

---

## 📜 License

MIT License - This standard belongs to humanity.

---

<div align="center">

🤟 **弘益人間** - Benefit All Humanity

© 2025 SmileStory Inc. / WIA

</div>
