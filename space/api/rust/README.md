# WIA Space Rust API

**High-performance Rust implementation of the WIA Space Standard**

[![Crates.io](https://img.shields.io/crates/v/wia-space.svg)](https://crates.io/crates/wia-space)
[![Documentation](https://docs.rs/wia-space/badge.svg)](https://docs.rs/wia-space)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)

---

## Overview

This crate provides Rust types and utilities for working with the WIA Space Standard, covering six key space technology areas:

- **Dyson Sphere** - Stellar energy harvesting megastructures
- **Mars Terraforming** - Planetary environment modification
- **Warp Drive** - Spacetime propulsion systems
- **Space Elevator** - Orbital access infrastructure
- **Asteroid Mining** - Space resource extraction
- **Interstellar Travel** - Interstellar mission systems

## Installation

Add to your `Cargo.toml`:

```toml
[dependencies]
wia-space = "1.0"
```

## Quick Start

```rust
use wia_space::prelude::*;

// Create an asteroid mining specification
let asteroid = TargetAsteroid::new("16 Psyche", AsteroidType::MType)
    .with_diameter_km(226.0)
    .with_mass_kg(2.72e19);

let mining = AsteroidMiningSpec::new("mining-001", asteroid);

// Serialize to JSON
let json = serde_json::to_string_pretty(&mining)?;
println!("{}", json);
```

## Technology Categories

### Dyson Sphere

```rust
use wia_space::prelude::*;

let dyson = DysonSphereSpec::sol_swarm("dyson-001")
    .with_structure(DysonStructureParameters {
        orbital_radius_au: 1.0,
        coverage_fraction: Some(0.04),
        ..Default::default()
    })
    .with_energy(EnergyHarvesting {
        efficiency_percent: 25.0,
        ..Default::default()
    });

if let Some(power) = dyson.calculate_power() {
    println!("Harvestable power: {:.2e} W", power);
}
```

### Mars Terraforming

```rust
use wia_space::prelude::*;

let mars = MarsTerraformingSpec::mars("terraform-001")
    .add_method(InterventionMethod {
        method: TerraformingMethod::SolarMirrors,
        temperature_effect_kelvin: Some(10.0),
        implementation_time_years: Some(50.0),
        ..Default::default()
    });
```

### Warp Drive

```rust
use wia_space::prelude::*;

// Create subluminal Alcubierre drive (2024 research)
let warp = WarpDriveSpec::subluminal("warp-001")
    .with_performance(WarpPerformance {
        max_velocity_c: Some(0.1),
        cruise_velocity_c: Some(0.05),
        ..Default::default()
    });

// Validate physics constraints
warp.validate()?;
```

### Space Elevator

```rust
use wia_space::prelude::*;

let elevator = SpaceElevatorSpec::earth_equatorial("elevator-001");

// Calculate trip time to GEO
if let Some(hours) = elevator.calculate_geo_trip_time() {
    println!("Trip time to GEO: {:.1} hours", hours);
}
```

### Asteroid Mining

```rust
use wia_space::prelude::*;

// Pre-configured 16 Psyche mission
let mining = AsteroidMiningSpec::psyche("mining-psyche");

println!("Target: {}", mining.target_asteroid.name);
println!("Type: {:?}", mining.target_asteroid.asteroid_type);
```

### Interstellar Travel

```rust
use wia_space::prelude::*;

// Breakthrough Starshot-style mission
let starshot = InterstellarTravelSpec::alpha_centauri_starshot("starshot-001");

println!("Distance: {} ly", starshot.mission.distance_ly);
println!("Travel time: {} years", starshot.trajectory.unwrap().travel_time_years);
```

## Simulation Adapters

The crate includes simulation utilities:

```rust
use wia_space::adapters::*;

// Orbital mechanics
let delta_v = OrbitalSimulator::hohmann_delta_v(1.0, 1.524); // Earth to Mars
let transfer_time = OrbitalSimulator::hohmann_transfer_time(1.0, 1.524);

// Interstellar calculations
let travel_time = InterstellarCalculator::travel_time_years(4.246, 0.2);
let gamma = InterstellarCalculator::lorentz_factor(0.2);
let ship_time = InterstellarCalculator::ship_time_years(4.246, 0.2);

// Dyson sphere energy
let power = DysonCalculator::harvestable_power(1.0, 0.01, 0.25);
```

## I/O Utilities

```rust
use wia_space::adapters::*;

// Read/write JSON files
let spec: AsteroidMiningSpec = read_json("mission.json")?;
write_json("output.json", &spec)?;

// Detect specification type
let category = SpecType::detect(&json_string)?;
```

## WIA Space Protocol (Phase 3)

The crate includes the WIA Space Protocol (WSP) for mission communication:

### Message Building

```rust
use wia_space::protocol::*;

// Create message builder
let builder = MessageBuilder::ground_station("goldstone-dsn");
let destination = Endpoint::spacecraft("mars-orbiter-01");

// Build a ping message
let ping = builder.ping(&destination);

// Build a telemetry message
let telemetry = builder.telemetry(&destination, TelemetryPayload {
    mission_id: "mars-terraform-2035".to_string(),
    subsystem: "atmospheric_processor".to_string(),
    readings: serde_json::json!({
        "co2_concentration": 0.953,
        "temperature_k": 210.5
    }),
    status: "nominal".to_string(),
    timestamp: None,
});
```

### Protocol Handler

```rust
use wia_space::protocol::*;

#[tokio::main]
async fn main() {
    let handler = ProtocolHandler::new();

    // Create session
    let session_id = handler.create_session(
        Endpoint::ground_station("gs-01"),
        Endpoint::ground_station("mission-control"),
    ).await;

    // Process incoming messages
    if let Some(response) = handler.process(incoming_message).await? {
        // Send response
    }
}
```

### Deep Space Latency Simulation

```rust
use wia_space::transport::*;

// Pre-configured targets
let leo = DeepSpaceTarget::Leo;           // 2ms
let moon = DeepSpaceTarget::Moon;         // 1.3s
let mars = DeepSpaceTarget::MarsOpposition; // 3 min
let jupiter = DeepSpaceTarget::Jupiter;   // 45 min

// Get one-way delay
let delay = moon.light_delay_ms();

// Create latency simulator
let config = LatencyConfig::mars_opposition();
let mut simulator = LatencySimulator::new(config);

// Simulate transmission with realistic delay
let result = simulator.simulate_transmission().await;
```

### Transport Layer

```rust
use wia_space::transport::*;

// Create mock transport for testing
let (mut transport_a, mut transport_b) = MockTransport::create_pair();

// Send and receive messages
transport_a.send(&message).await?;
let received = transport_b.receive().await?;

// With latency simulation
let config = LatencyConfig::moon();
let transport = MockTransport::new().with_latency(config);
```

## Features

- `default` - Core functionality
- `wasm` - WebAssembly support for browsers
- `python` - Python bindings via PyO3

```toml
[dependencies]
wia-space = { version = "1.0", features = ["wasm"] }
```

## Examples

Run examples with:

```bash
cargo run --example basic_usage
cargo run --example asteroid_mining
cargo run --example interstellar_mission
cargo run --example protocol_demo     # Phase 3 protocol demo
```

## Testing

```bash
cargo test
cargo test --all-features
```

## Project Structure

```
src/
├── lib.rs           # Main library entry
├── types.rs         # Type definitions
├── error.rs         # Error types
├── core/
│   ├── mod.rs
│   ├── project.rs   # Project management
│   └── space.rs     # Technology specifications
├── adapters/
│   ├── mod.rs
│   ├── simulator.rs # Simulation utilities
│   └── io.rs        # I/O utilities
├── protocol/        # Phase 3: WIA Space Protocol
│   ├── mod.rs
│   ├── message.rs   # Message types
│   ├── builder.rs   # Message builder
│   ├── handler.rs   # Protocol handler
│   └── error.rs     # Protocol errors
└── transport/       # Phase 3: Transport Layer
    ├── mod.rs
    ├── base.rs      # Transport trait
    ├── mock.rs      # Mock transport
    └── latency.rs   # Latency simulation
```

## Related Crates

- `wia-aac` - AAC/BCI Standard implementation
- `wia-braille` - Tactile Braille Reader

## License

MIT License - This standard belongs to humanity.

---

**弘益人間** - *Benefit All Humanity*

© 2025 WIA / SmileStory Inc.
