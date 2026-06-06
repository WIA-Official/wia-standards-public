# Perception Clarity

> a cross-cutting WIA standard for measuring, stating, and reporting how clearly physical-AI sensors see

[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)]()
[![License](https://img.shields.io/badge/license-MIT-green.svg)]()
[![Standard](https://img.shields.io/badge/WIA-Standard-purple.svg)]()

홍익인간 (弘益人間) - Benefit All Humanity

## 📋 Overview

The WIA Perception Clarity standard defines how physical-AI agents — autonomous vehicles, robots, drones, and AMRs — **measure, state, and report** how clearly their optical and sensing surfaces currently see. It standardizes a single shared language for perception health so that fleets, regulators, and safety systems can reason about sensor clarity across vendors.

The standard specifies three things:

- **MEASURE** — the Perception Clarity Index (PCI), a 0–100 score computed per sensor from occlusion, detection-distance degradation, and contrast/MTF reduction.
- **STATE** — a fixed clarity-state enum: `clear` / `degraded` / `obstructed` / `blind`.
- **REPORT** — the `SensorClarityReport` message schema for sharing PCI, state, contaminant classification, and conformance with fleet and safety systems.

It standardizes the *measurement, state, and reporting model*. It does **not** standardize cleaning hardware, nozzles, cleaning fluids, or vendor algorithms (to avoid patent-thicket conflicts).

### Key Features

- ✅ PCI 0–100 measured per sensor, mapped to a fixed state enum
- ✅ Sensor-class-aware weighting (camera, IR, LiDAR, radar, ultrasonic)
- ✅ Standard `SensorClarityReport` message + conformance declaration
- ✅ 弘益人間 philosophy integration
- ✅ Open-source MIT license

## 🚀 Quick Start

### Installation

```bash
# Clone the repository
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/standards/perception-clarity

# Install dependencies (if applicable)
./install.sh
```

### Basic Usage

```bash
# CLI tool
./cli/perception-clarity.sh info
./cli/perception-clarity.sh validate report.json
```

```rust
// Rust SDK example (api/rust)
use perception_clarity::{SensorClarityReport, Pci, SensorClass};

let pci = Pci::compute(
    SensorClass::RgbCamera,
    /* occlusion */ 0.18,
    /* distance_degradation */ 0.22,
    /* mtf_reduction */ 0.40,
);

println!("PCI = {} ({:?})", pci.value(), pci.state());
```

## 📚 Documentation

- **Specification**: `spec/` - Complete technical specification (PCI, state enum, schemas)
- **API Reference**: `api/rust/` - Rust SDK documentation
- **CLI Tool**: `cli/` - Command-line interface
- **Examples**: `examples/` - Usage examples and tutorials

## 🏗️ Architecture

Modular architecture: a measurement core (PCI computation), a state-mapping layer, a reporting layer (`SensorClarityReport`), and conformance declaration — designed to drop into any physical-AI agent regardless of sensor vendor.

## 🔧 Components

### Specification
Complete technical specifications covering:
- PCI (Perception Clarity Index) data model and axis weighting
- Sensor class and clarity state enums
- Contaminant classification
- Sensor reporting message schema and conformance levels

### API SDK
- Rust implementation (primary, high-performance)
- Type-safe interfaces for PCI and report structures
- Comprehensive validation
- Error handling and logging

### CLI Tool
- Validates `SensorClarityReport` JSON against the standard
- Computes PCI and resolves clarity state
- Compliance checking
- Status monitoring and reporting

## 📖 Examples

See the `examples/` directory for comprehensive usage examples including:
- Computing PCI from raw clarity axes
- Building and validating a `SensorClarityReport`
- Mapping PCI to clarity state and safe actions
- Fleet-side ingestion patterns

## 🌍 Philosophy

**홍익인간 (弘益人間)** - Benefit All Humanity

This standard embodies the Korean philosophy of 弘益人間, aiming to benefit all humanity through safer physical-AI perception. We believe technology should serve the greater good and create positive impact for everyone.

## 🤝 Contributing

Contributions are welcome! Please follow these steps:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 📄 License

MIT License

Copyright (c) 2025 WIA (World Intelligence Alliance)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

## 🔗 Related Standards

- [WIA-ROBOT](../robot) - Robotics agent standard
- [WIA-AUTO](../auto) - Autonomous vehicle standard
- [WIA-DRONE](../drone) - Drone agent standard
- [WIA-LIDAR-SENSOR](../lidar-sensor) - LiDAR sensing standard
- [WIA-VISION-AI](../vision-ai) - Vision AI perception standard
- [WIA-AI-SENSOR-FUSION](../ai-sensor-fusion) - Multi-sensor fusion standard

## 📞 Contact

- **Organization**: WIA (World Intelligence Alliance)
- **Website**: https://github.com/WIA-Official
- **Repository (full, private)**: https://github.com/WIA-Official/wia-standards
- **Repository (public)**: https://github.com/WIA-Official/wia-standards-public/tree/main/perception-clarity
- **Issues**: https://github.com/WIA-Official/wia-standards/issues
- **Discord**: https://discord.gg/wia-standards

---

© 2025 WIA / SmileStory Inc.

**弘益人間 (Benefit All Humanity)** 🌍
