# WIA Nanotechnology Rust SDK

弘益人間 (Benefit All Humanity)

## Overview

Rust SDK for the WIA Nanotechnology Standard. This SDK provides tools for nanoparticle design, synthesis simulation, and nanomaterial property analysis.

## Installation

Add to your `Cargo.toml`:

```toml
[dependencies]
wia-nanotechnology = "1.0.0"
```

## Quick Start

```rust
use wia_nanotechnology::*;

#[tokio::main]
async fn main() -> Result<()> {
    // Create synthesis parameters
    let params = SynthesisParams {
        method: SynthesisMethod::ChemicalVaporDeposition,
        temperature_celsius: 800.0,
        pressure_kpa: 101.325,
        duration_minutes: 120.0,
        precursors: vec!["Methane".to_string()],
    };

    // Create client
    let client = NanotechnologyClient::new(
        "https://api.wia.org/nano".to_string()
    );

    // Design nanoparticle
    let nanoparticle = client.design_nanoparticle(params).await?;
    println!("Designed: {}", utils::format_nanoparticle_info(&nanoparticle));

    Ok(())
}
```

## Features

- **Design**: Nanoparticle design and optimization
- **Synthesis**: Synthesis parameter calculation and validation
- **Simulation**: Property simulation and prediction
- **Materials**: Support for multiple nanomaterial types
- **Type Safety**: Strong type system with Rust

## Supported Nanomaterials

- Carbon Nanotubes (CNT)
- Graphene
- Quantum Dots
- Fullerenes
- Metal Oxides
- Gold Nanoparticles
- Silver Nanoparticles
- Dendrimers

## Synthesis Methods

- Chemical Vapor Deposition (CVD)
- Sol-Gel
- Hydrothermal
- Precipitation
- Ball Milling
- Laser Ablation

## API

### Client

- `NanotechnologyClient::new(base_url)` - Create new client
- `client.design_nanoparticle(params)` - Design nanoparticle
- `client.simulate(nanoparticle)` - Simulate properties
- `client.get_nanoparticle(id)` - Get nanoparticle by ID

### Validators

- `validate_nanoparticle(particle)` - Validate nanoparticle spec
- `validate_synthesis_params(params)` - Validate synthesis parameters
- `validate_optical_properties(props)` - Validate optical properties

### Utils

- `calculate_sa_to_v_ratio(size)` - Calculate surface area to volume ratio
- `estimate_synthesis_time(method)` - Estimate synthesis duration
- `calculate_optimal_temperature(material)` - Get optimal temperature
- `format_nanoparticle_info(particle)` - Format particle information

## Testing

```bash
cargo test
```

## Examples

```bash
cargo run --example basic_usage
```

## License

MIT License - See LICENSE file for details

## Contact

- Website: https://wia.org
- Email: tech@wia.org
- GitHub: https://github.com/WIA-Official/wia-standards

---

© 2025 SmileStory Inc. / WIA
弘益人間 (Benefit All Humanity)
