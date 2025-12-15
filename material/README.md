# WIA Material Standard

**Material Science Standards**

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)
[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)](https://github.com/WIA-Official/wia-standards)
[![Standard](https://img.shields.io/badge/standard-WIA%20MATERIAL-orange.svg)](https://material.wia.live)

---

<div align="center">

**Part of WIA Standards Ecosystem**

[WIA Standards Hub](https://wia.live/standards) | [API Portal](https://api.wia.live)

---

**弘益人間** - *Benefit All Humanity*

</div>

---

## Overview

WIA Material is an open standard for advanced material science data exchange. This standard provides unified data formats for cutting-edge materials research including superconductors, metamaterials, programmable matter, holographic storage, memristors, and topological insulators.

**Key Objectives**:
- Unify data formats across material science domains
- Enable interoperability between research institutions and industry
- Provide standard APIs for developers
- Accelerate innovation through open collaboration

---

## Covered Domains

| Domain | Description | Market Trend |
|--------|-------------|--------------|
| **Superconductor** | Room temperature and high-Tc superconductors | Active research |
| **Metamaterial** | Electromagnetic/acoustic/mechanical metamaterials | $18B by 2032 |
| **Programmable Matter** | Claytronics and self-reconfiguring materials | R&D phase |
| **Holographic Storage** | 3D volumetric data storage | $294M in 2025 |
| **Memristor** | Neuromorphic computing components | $13.5B by 2027 |
| **Topological Insulator** | Quantum materials for spintronics | $15M by 2035 |

---

## Specification Phases

| Phase | Title | Description | Status |
|:-----:|-------|-------------|:------:|
| **1** | Data Format | Standard data format for material science | ✅ Complete |
| **2** | API Interface | Rust SDK for developers | ✅ Complete |
| **3** | Communication Protocol | REST/WebSocket protocols | ✅ Complete |
| **4** | Ecosystem Integration | WIA integration | ⏳ Planned |

---

## Quick Start

### Rust SDK

```rust
use wia_material::prelude::*;

#[tokio::main]
async fn main() -> Result<(), MaterialError> {
    let client = WiaMaterial::new();

    // Create a superconductor
    let props = SuperconductorProperties {
        critical_temperature_k: 93.0,
        critical_pressure_pa: Some(101325.0),
        meissner_effect: Some(true),
        ..Default::default()
    };

    let material = client
        .create_superconductor("YBCO", "YBa2Cu3O7-x", props)
        .await?;

    println!("Created: {}", material.material_id);
    Ok(())
}
```

### Data Format Example

```json
{
  "$schema": "https://wia.live/material/v1/schema.json",
  "version": "1.0.0",
  "material_type": "topological_insulator",
  "material_id": "wia-mat-00000001",
  "timestamp": {
    "created": "2025-12-14T00:00:00Z"
  },
  "identity": {
    "name": "Bismuth Selenide",
    "formula": "Bi2Se3",
    "classification": ["chalcogenide", "topological_insulator"]
  },
  "structure": {
    "crystal_system": "rhombohedral",
    "space_group": "R-3m"
  },
  "properties": {
    "domain_specific": {
      "band_gap_eV": 0.3,
      "z2_invariant": [1, 0, 0, 0],
      "spin_hall_angle": 0.3
    }
  }
}
```

---

## Structure

```
material/
├── spec/
│   ├── RESEARCH-PHASE-1.md      # Phase 1 research findings
│   ├── PHASE-1-DATA-FORMAT.md   # Data format specification
│   ├── RESEARCH-PHASE-3.md      # Phase 3 protocol research
│   ├── PHASE-3-PROTOCOL.md      # Communication protocol spec
│   └── schemas/                 # JSON Schema files
├── api/
│   └── rust/                    # Rust SDK ✅
│       ├── src/
│       │   ├── lib.rs           # Main library
│       │   ├── types.rs         # Type definitions
│       │   ├── error.rs         # Error handling
│       │   ├── core/            # Core logic
│       │   ├── adapters/        # Data adapters
│       │   ├── protocol/        # Protocol layer (Phase 3)
│       │   └── transport/       # Transport layer (Phase 3)
│       ├── tests/               # Integration tests
│       └── examples/            # Usage examples
├── prompts/                     # Claude Code prompts
└── docs/                        # Documentation
```

---

## JSON Schemas

| Schema | Description |
|--------|-------------|
| `wia-material-v1.schema.json` | Base schema for all materials |
| `superconductor.schema.json` | Superconductor-specific fields |
| `metamaterial.schema.json` | Metamaterial properties |
| `programmable-matter.schema.json` | Claytronics/catom data |
| `holographic-storage.schema.json` | HDS media and performance |
| `memristor.schema.json` | Memristor and neuromorphic data |
| `topological-insulator.schema.json` | Topological material properties |

---

## Compatibility

WIA Material is designed to be compatible with existing standards:

- **Materials Project API** - External reference support
- **OPTIMADE** - JSON:API compatible structure
- **CIF (Crystallographic Information File)** - Crystal structure mapping
- **NIST MDCS** - Provenance and metadata alignment

---

## Links

| Resource | URL |
|----------|-----|
| **Website** | https://material.wia.live |
| **Standards Hub** | https://wia.live/standards |
| **GitHub** | https://github.com/WIA-Official/wia-standards/tree/main/material |
| **Materials Project** | https://materialsproject.org |
| **OPTIMADE** | https://www.optimade.org |

---

## License

MIT License - This standard belongs to humanity.

---

<div align="center">

🤟 **弘益人間** - Benefit All Humanity

© 2025 SmileStory Inc. / WIA

</div>
