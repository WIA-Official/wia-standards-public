# WIA Physics Standard

**Physics & Energy Standards**

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)
[![Version](https://img.shields.io/badge/version-0.1.0-blue.svg)](https://github.com/WIA-Official/wia-standards)
[![Standard](https://img.shields.io/badge/standard-WIA%20PHYSICS-orange.svg)](https://physics.wia.live)

---

<div align="center">

⚡ **Part of WIA Standards Ecosystem**

[WIA Standards Hub](https://wia.live/standards) • [API Portal](https://api.wia.live)

---

**홍익인간 (弘益人間)** - *Benefit All Humanity*

</div>

---

## 🌍 Overview

WIA Physics is an open standard for physics & energy standards.

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
| **3** | Communication Protocol | WPP streaming protocol | ✅ Complete |
| **4** | Ecosystem Integration | EPICS, HDF5, InfluxDB adapters | ✅ Complete |

---

## 🚀 Quick Start

### Data Formats

The WIA Physics Standard defines data formats for six key physics domains:

| Domain | Schema | Description |
|--------|--------|-------------|
| **Nuclear Fusion** | `fusion.schema.json` | Plasma, magnetic confinement, energy balance |
| **Time Crystals** | `time-crystal.schema.json` | Oscillation, coherence, quantum properties |
| **Particle Physics** | `particle.schema.json` | Particles, events, cross-sections |
| **Dark Matter** | `dark-matter.schema.json` | Detection events, exclusion limits, axion searches |
| **Antimatter** | `antimatter.schema.json` | Antiparticles, traps, spectroscopy, CPT tests |
| **Quantum Gravity** | `quantum-gravity.schema.json` | LQG, string theory, black holes |

### Using the Schemas

```json
{
  "$schema": "https://wia.live/schemas/physics/fusion.schema.json",
  "metadata": {
    "id": "fusion-2025-001",
    "experiment": {"name": "ITER"},
    "created": "2025-12-14T10:00:00Z"
  },
  "plasma": {
    "temperature": {
      "value": 150000000,
      "uncertainty": {"total": 11180340},
      "unit": "K"
    }
  }
}
```

### Rust SDK

```toml
# Cargo.toml
[dependencies]
wia-physics = "1.0"
```

```rust
use wia_physics::prelude::*;

// Create fusion data
let fusion_data = FusionDataBuilder::new()
    .experiment("ITER")
    .plasma_simple(150e6, "K", 1e20, "m^-3")
    .tokamak(5.3, 6.2, 2.0)
    .build()?;

// Physics calculations
let triple_product = FusionPhysics::triple_product(1e20, 15.0, 3.0);
let q_factor = FusionPhysics::q_factor(500.0, 50.0)?;

// Serialize to JSON
let json = serde_json::to_string_pretty(&fusion_data)?;
```

### WIA Physics Protocol (WPP)

Real-time streaming protocol for physics data:

```rust
use wia_physics::protocol::*;
use wia_physics::transport::*;

// Create transport and connect
let mut transport = MockTransport::new();
transport.connect().await?;

// Send protocol messages
transport.send(WppMessage::connect("client-id", "My App")).await?;
transport.send(WppMessage::subscribe("fusion/iter/plasma")).await?;

// Receive data
let response = transport.receive().await?;
```

**Protocol Features:**
- WebSocket, TCP, and gRPC transport support
- Pub/Sub channel subscriptions with wildcards
- Command/response for device control
- Real-time event notifications
- Binary mode with LZ4 compression

### Ecosystem Integration

Integrate physics data with external systems:

```rust
use wia_physics::integration::*;

// HDF5 Archive
let mut hdf5 = HDF5Adapter::new();
hdf5.create_file("/data/fusion.h5")?;
hdf5.write_dataset("/plasma/temperature", &[150e6], DatasetOptions::default())?;

// InfluxDB Time-Series
let mut influx = InfluxDBAdapter::new("http://localhost:8086", "wia_physics");
influx.connect()?;
influx.write_point(DataPoint::new("wia_fusion_plasma")
    .tag("experiment", "ITER")
    .field_float("temperature", 150e6))?;

// EPICS Control System
let mut epics = EPICSAdapter::new();
epics.connect()?;
epics.put_pv("ITER:PLASMA:TEMP:CORE", PVValue::double(150e6).with_units("K"))?;
```

**Integration Features:**
- HDF5 data archives with compression
- InfluxDB/Grafana for real-time monitoring
- EPICS Channel Access for control systems
- Unified IntegrationManager for multiple backends

---

## 📁 Structure

```
physics/
├── spec/                           # Specifications
│   ├── RESEARCH-PHASE-1.md         # Phase 1 research
│   ├── PHASE-1-DATA-FORMAT.md      # Data format spec
│   ├── RESEARCH-PHASE-3.md         # Phase 3 research
│   ├── PHASE-3-PROTOCOL.md         # Protocol spec
│   ├── RESEARCH-PHASE-4.md         # Phase 4 research
│   ├── PHASE-4-ECOSYSTEM.md        # Ecosystem integration spec
│   └── schemas/                    # JSON Schema definitions
│       ├── common.schema.json      # Common types
│       ├── fusion.schema.json      # Nuclear fusion
│       ├── particle.schema.json    # Particle physics
│       ├── dark-matter.schema.json # Dark matter
│       ├── antimatter.schema.json  # Antimatter
│       ├── quantum-gravity.schema.json # Quantum gravity
│       └── protocol.schema.json    # Protocol messages
├── api/
│   └── rust/                       # Rust SDK
│       ├── Cargo.toml
│       ├── src/
│       │   ├── lib.rs              # Main library
│       │   ├── types.rs            # Type definitions
│       │   ├── error.rs            # Error handling
│       │   ├── core/               # Physics calculations
│       │   ├── adapters/           # Simulators
│       │   ├── protocol/           # WPP protocol
│       │   ├── transport/          # Transport adapters
│       │   └── integration/        # Ecosystem integration
│       ├── tests/
│       └── examples/
├── examples/
├── prompts/                        # Claude Code prompts
└── docs/
```

---

## 🔗 Links

| Resource | URL |
|----------|-----|
| **Website** | https://physics.wia.live |
| **Standards Hub** | https://wia.live/standards |
| **GitHub** | https://github.com/WIA-Official/wia-standards/tree/main/physics |

---

## 📜 License

MIT License - This standard belongs to humanity.

---

<div align="center">

🤟 **홍익인간 (弘益人間)** - Benefit All Humanity

© 2025 SmileStory Inc. / WIA

</div>
