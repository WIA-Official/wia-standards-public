# WIA Physics SDK (Rust)

**High-performance Rust SDK for physics and energy research data**

[![Crates.io](https://img.shields.io/crates/v/wia-physics.svg)](https://crates.io/crates/wia-physics)
[![Documentation](https://docs.rs/wia-physics/badge.svg)](https://docs.rs/wia-physics)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)

---

## Overview

The WIA Physics SDK provides comprehensive data types and utilities for:

| Domain | Description |
|--------|-------------|
| **Nuclear Fusion** | Plasma parameters, magnetic confinement, energy balance |
| **Time Crystals** | Oscillation dynamics, quantum properties |
| **Particle Physics** | Collision events, cross-sections, particle properties |
| **Dark Matter** | Detection events, exclusion limits, axion searches |
| **Antimatter** | Antiparticle properties, traps, spectroscopy |
| **Quantum Gravity** | Theoretical predictions, black hole physics |

---

## Installation

Add to your `Cargo.toml`:

```toml
[dependencies]
wia-physics = "1.0"
```

With optional features:

```toml
[dependencies]
wia-physics = { version = "1.0", features = ["wasm", "python"] }
```

---

## Quick Start

```rust
use wia_physics::prelude::*;

fn main() -> PhysicsResult<()> {
    // Create a measurement with uncertainty
    let temperature = Measurement::new(150_000_000.0, 5_000_000.0, "K");
    println!("Temperature: {} ± {} {}",
             temperature.value,
             temperature.uncertainty.total,
             temperature.unit);

    // Build fusion data
    let fusion_data = FusionDataBuilder::new()
        .experiment("ITER")
        .plasma_simple(150e6, "K", 1e20, "m^-3")
        .tokamak(5.3, 6.2, 2.0)
        .build()?;

    // Serialize to JSON
    let json = serde_json::to_string_pretty(&fusion_data)?;
    println!("{}", json);

    Ok(())
}
```

---

## Core Features

### Measurements with Uncertainty

```rust
// Simple measurement
let mass = Measurement::new(125.25, 0.18, "GeV");

// With statistical and systematic uncertainties
let mass = Measurement::with_uncertainty(125.25, 0.15, 0.10, "GeV");

// Check compatibility
let m1 = Measurement::new(100.0, 5.0, "GeV");
let m2 = Measurement::new(108.0, 5.0, "GeV");
assert!(m1.is_compatible_with(&m2)); // Within 2-sigma
```

### Physics Calculations

```rust
// Fusion physics
let triple_product = FusionPhysics::triple_product(1e20, 15.0, 3.0);
let q_factor = FusionPhysics::q_factor(500.0, 50.0)?;
let is_ignition = FusionPhysics::is_ignition_possible(triple_product);

// Particle physics
let mass = ParticlePhysics::invariant_mass(e, px, py, pz);
let significance = ParticlePhysics::significance(signal, background)?;
let is_discovery = ParticlePhysics::is_discovery(significance);

// Unit conversions
let kelvin = UnitConverter::ev_to_kelvin(1.0);
let gev = UnitConverter::kg_to_gev(mass_kg);
```

### Simulators

```rust
use wia_physics::adapters::*;

// Fusion simulator
let mut sim = FusionSimulator::iter_like();
for _ in 0..1000 {
    sim.step().await?;
}
let state = sim.get_state()?;

// Event generator
let mut gen = EventGenerator::lhc_run3();
let event = gen.generate_event();
```

---

## Data Types

### Fusion Data

```rust
let data = FusionDataBuilder::new()
    .experiment("ITER")
    .plasma(PlasmaParameters {
        temperature: Measurement::new(150e6, 5e6, "K"),
        density: Measurement::new(1e20, 5e18, "m^-3"),
        confinement_time: Some(Measurement::new(3.0, 0.2, "s")),
        // ...
    })
    .tokamak(5.3, 6.2, 2.0)
    .energy_balance(EnergyBalance {
        input_power: Some(Measurement::new(50.0, 1.0, "MW")),
        fusion_power: Some(Measurement::new(500.0, 10.0, "MW")),
        q_factor: Some(Measurement::new(10.0, 0.5, "")),
        // ...
    })
    .quality(QualityFlag::Good)
    .build()?;
```

### Particle Data

```rust
let particle = ParticleProperties {
    name: "Higgs boson".to_string(),
    pdg_id: 25,
    mass: Measurement::new(125.25, 0.18, "GeV"),
    charge: 0.0,
    spin: 0.0,
    lifetime: Some(Measurement::new(1.56e-22, 0.01e-22, "s")),
    // ...
};

let event = CollisionEvent {
    event_id: "run3-evt-12345".to_string(),
    collision: CollisionInfo {
        sqrt_s: Measurement::new(13600.0, 1.0, "GeV"),
        beam1: "proton".to_string(),
        beam2: "proton".to_string(),
    },
    jets: vec![...],
    missing_et: Some(MissingET { ... }),
    // ...
};
```

### Dark Matter Data

```rust
let limit = ExclusionLimit {
    dm_candidate: DarkMatterCandidate::Wimp,
    dm_mass: Some(Measurement::new(100.0, 10.0, "GeV")),
    cross_section_limit: Some(Measurement::new(1e-47, 1e-48, "cm^2")),
    confidence_level: 0.90,
    interaction_type: InteractionType::SI,
    // ...
};
```

---

## Physical Constants

```rust
use wia_physics::core::constants;

let c = constants::SPEED_OF_LIGHT;      // 299,792,458 m/s
let h = constants::PLANCK_CONSTANT;     // 6.626e-34 J·s
let k_b = constants::BOLTZMANN;         // 1.381e-23 J/K
let e = constants::ELEMENTARY_CHARGE;   // 1.602e-19 C
let l_p = constants::PLANCK_LENGTH;     // 1.616e-35 m
```

---

## Examples

Run the examples:

```bash
# Basic usage
cargo run --example basic_usage

# Fusion simulation
cargo run --example fusion_simulation
```

---

## Testing

```bash
# Run all tests
cargo test

# Run with output
cargo test -- --nocapture

# Run integration tests
cargo test --test integration_test
```

---

## Features

| Feature | Description |
|---------|-------------|
| `default` | Core functionality |
| `wasm` | WebAssembly support via wasm-bindgen |
| `python` | Python bindings via PyO3 |
| `full` | All optional features |

---

## Project Structure

```
src/
├── lib.rs           # Main library entry
├── types.rs         # Data type definitions
├── error.rs         # Error handling
├── core/
│   ├── mod.rs
│   └── physics.rs   # Physics calculations
└── adapters/
    ├── mod.rs
    └── simulator.rs # Simulators and generators

tests/
└── integration_test.rs

examples/
├── basic_usage.rs
└── fusion_simulation.rs
```

---

## License

MIT License - This standard belongs to humanity.

---

**弘益人間** - Benefit All Humanity 🤟

© 2025 SmileStory Inc. / WIA
