# WIA Material SDK (Rust)

Rust SDK for WIA Material Science Standards.

[![Crates.io](https://img.shields.io/crates/v/wia-material.svg)](https://crates.io/crates/wia-material)
[![Documentation](https://docs.rs/wia-material/badge.svg)](https://docs.rs/wia-material)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)

---

## Overview

This SDK provides a standardized interface for working with material science data, supporting:

- **Superconductors** - High-Tc and room-temperature superconductors
- **Metamaterials** - Electromagnetic, acoustic, and mechanical metamaterials
- **Memristors** - Resistive switching devices for neuromorphic computing
- **Topological Insulators** - Quantum materials with topological surface states
- **Holographic Storage** - Volumetric data storage media
- **Programmable Matter** - Self-reconfiguring materials (claytronics)

## Installation

Add to your `Cargo.toml`:

```toml
[dependencies]
wia-material = "1.0.0"
tokio = { version = "1", features = ["full"] }
```

## Quick Start

```rust
use wia_material::prelude::*;

#[tokio::main]
async fn main() -> Result<(), MaterialError> {
    // Create a client
    let client = WiaMaterial::new();

    // Build a superconductor material
    let material = MaterialBuilder::new()
        .material_type(MaterialType::Superconductor)
        .name("YBCO")
        .formula("YBa2Cu3O7-x")
        .confidence(0.95)
        .build()?;

    // Store the material
    let stored = client.create_material(material).await?;
    println!("Created: {}", stored.material_id);

    // Retrieve by ID
    let retrieved = client.get_material(&stored.material_id).await?;
    println!("Name: {}", retrieved.identity.name);

    Ok(())
}
```

## Examples

### Creating a Superconductor

```rust
use wia_material::prelude::*;

let props = SuperconductorProperties {
    critical_temperature_k: 93.0,
    critical_pressure_pa: Some(101325.0),
    critical_magnetic_field_t: Some(100.0),
    meissner_effect: Some(true),
    superconductor_type: Some(SuperconductorType::TypeII),
    coherence_length_nm: Some(1.5),
    penetration_depth_nm: Some(150.0),
    ..Default::default()
};

let material = client
    .create_superconductor("YBCO", "YBa2Cu3O7-x", props)
    .await?;
```

### Creating a Topological Insulator

```rust
let props = TopologicalInsulatorProperties {
    band_gap_ev: 0.3,
    z2_invariant: Some(vec![1, 0, 0, 0]),
    dirac_point_ev: Some(-0.1),
    surface_state: Some(SurfaceState {
        fermi_velocity_m_s: Some(5e5),
        spin_texture: Some("helical".to_string()),
        surface_conductivity_s: None,
    }),
    spin_hall_angle: Some(0.3),
    ..Default::default()
};

let material = client
    .create_topological_insulator("Bi2Se3", "Bi2Se3", props)
    .await?;
```

### Creating a Memristor

```rust
let props = MemristorProperties {
    material: "TiO2".to_string(),
    structure: Some(MemristorStructure::MetalInsulatorMetal),
    resistance_high_ohm: 1e6,
    resistance_low_ohm: 1e3,
    on_off_ratio: Some(1000.0),
    set_voltage_v: Some(1.0),
    reset_voltage_v: Some(-0.8),
    neuromorphic: Some(NeuromorphicProperties {
        synaptic_weight: Some(0.5),
        plasticity: Some("stdp".to_string()),
        analog_states: Some(128),
        ..Default::default()
    }),
    ..Default::default()
};

let material = client
    .create_memristor("TiO2 Memristor", "TiO2", props)
    .await?;
```

### Using the Simulator Adapter

```rust
use wia_material::prelude::*;

// Create and connect simulator
let mut adapter = SimulatorAdapter::new();
adapter.connect().await?;

// Populate with sample data
adapter.populate_sample_data().await?;

// Search for materials
let query = MaterialQuery::new()
    .with_type(MaterialType::Superconductor)
    .with_min_confidence(0.9);

let results = adapter.search(&query).await?;

// Disconnect when done
adapter.disconnect().await?;
```

### Using the Protocol Layer (Phase 3)

```rust
use wia_material::prelude::*;

// Create a query with filters
let query = QueryPayload {
    material_type: Some(MaterialType::Superconductor),
    filter: Some(
        FilterBuilder::new()
            .gt("properties.superconductor.critical_temperature_k", 77.0)
            .build_and()
            .unwrap()
    ),
    sort: Some(SortConfig {
        field: "timestamp".to_string(),
        order: SortOrder::Desc,
    }),
    pagination: Some(Pagination { offset: 0, limit: 100 }),
    fields: None,
};

// Create protocol message
let message = MessageBuilder::query(query);
let json = message.to_json()?;
println!("{}", json);
```

### Using HTTP Transport

```rust
use wia_material::prelude::*;

// Configure transport
let config = TransportConfig::new("https://api.example.com")
    .with_api_key("your-api-key");

// Connect
let mut transport = HttpTransport::new(config);
transport.connect().await?;

// Query materials
let response = transport.query(QueryPayload::default()).await?;
println!("Found {} materials", response.meta.total_count);

// Disconnect
transport.disconnect().await?;
```

### Using WebSocket Transport

```rust
use wia_material::prelude::*;
use wia_material::protocol::SubscribePayload;

// Configure WebSocket
let config = WebSocketConfig::new("wss://api.example.com")
    .with_api_key("your-api-key");

let mut transport = WebSocketTransport::new(config);
transport.connect().await?;

// Subscribe to real-time measurements
let ack = transport.subscribe(SubscribePayload {
    channel: SubscriptionChannel::Measurements,
    material_type: Some(MaterialType::Superconductor),
    filter: None,
}).await?;

println!("Subscribed: {}", ack.subscription_id);

// Receive data stream
loop {
    match transport.receive().await {
        Ok(msg) => println!("Received: {}", msg),
        Err(_) => break,
    }
}

transport.disconnect().await?;
```

### JSON Serialization

```rust
let client = WiaMaterial::new();

// Build material
let material = MaterialBuilder::new()
    .material_type(MaterialType::Superconductor)
    .name("Test")
    .formula("Test")
    .build()?;

// Serialize to JSON
let json = client.to_json(&material)?;
println!("{}", json);

// Parse from JSON
let parsed = client.parse_json(&json)?;
```

## API Reference

### WiaMaterial

Main client for material operations.

| Method | Description |
|--------|-------------|
| `new()` | Create with default config |
| `with_config(config)` | Create with custom config |
| `create_material(data)` | Store material data |
| `get_material(id)` | Get material by ID |
| `get_materials()` | Get all materials |
| `get_materials_by_type(type)` | Filter by type |
| `validate(data)` | Validate material data |
| `to_json(data)` | Convert to JSON |
| `parse_json(json)` | Parse from JSON |
| `get_statistics()` | Get storage statistics |

### MaterialBuilder

Builder pattern for creating materials.

```rust
MaterialBuilder::new()
    .material_type(MaterialType::Superconductor)
    .name("Name")
    .formula("Formula")
    .classification(vec!["tag1".into(), "tag2".into()])
    .structure(structure)
    .electrical_properties(props)
    .measurement(measurement)
    .confidence(0.95)
    .notes("Notes")
    .build()?;
```

### MaterialQuery

Query builder for searching materials.

```rust
MaterialQuery::new()
    .with_type(MaterialType::Superconductor)
    .with_name("YBCO")
    .with_formula("YBa2Cu3O7-x")
    .with_classification("high_tc")
    .with_min_confidence(0.9)
    .with_limit(10)
    .with_offset(0);
```

### Protocol Module (Phase 3)

Message types and protocol handling.

| Type | Description |
|------|-------------|
| `Message<T>` | Protocol message envelope |
| `MessageBuilder` | Builder for creating messages |
| `MessageType` | Request/response type enum |
| `QueryPayload` | Query request payload |
| `FilterBuilder` | Filter condition builder |
| `Filter` | Compound filter with AND/OR |

### Transport Module (Phase 3)

HTTP and WebSocket transport implementations.

| Type | Description |
|------|-------------|
| `Transport` | Base transport trait |
| `StreamingTransport` | Streaming transport trait |
| `HttpTransport` | REST API client |
| `WebSocketTransport` | Real-time streaming client |
| `TransportConfig` | HTTP configuration |
| `WebSocketConfig` | WebSocket configuration |

### Integration Module (Phase 4)

Ecosystem integration with external databases and file formats.

#### Data Providers

```rust
use wia_material::integration::*;

// Create integration manager
let mut manager = IntegrationManagerBuilder::new()
    .enable_mock()
    .with_mp_api_key("your-api-key")
    .add_optimade_provider("https://optimade.example.org")
    .build();

// Connect to mock provider for testing
manager.connect_provider("mock").await?;

// Search for materials with iron and oxygen
let query = ProviderQuery::new()
    .with_elements(vec!["Fe".to_string(), "O".to_string()])
    .with_limit(50);

let results = manager.search(&query).await?;
```

#### File Format Export/Import

```rust
use wia_material::integration::*;

// Export to CIF format
let cif_content = manager.export(ExportFormat::Cif, &material, &ExportOptions::default())?;

// Export to POSCAR format
let poscar = manager.export(ExportFormat::Poscar, &material, &ExportOptions::default())?;

// Import from CIF
let imported = manager.import(ExportFormat::Cif, &cif_content, &ImportOptions::default())?;
```

| Type | Description |
|------|-------------|
| `IntegrationManager` | Central manager for providers and exporters |
| `DataProvider` | Trait for external data sources |
| `MockProvider` | Testing provider with sample data |
| `OptimadeProvider` | OPTIMADE API client |
| `MaterialsProjectProvider` | Materials Project API client |
| `Exporter` | Trait for file format handlers |
| `CifExporter` | CIF format export/import |
| `PoscarExporter` | VASP POSCAR format export/import |
| `XyzExporter` | XYZ coordinate format export/import |

## Material Types

| Type | Description |
|------|-------------|
| `Superconductor` | Superconducting materials |
| `Metamaterial` | Engineered materials with unusual properties |
| `Memristor` | Resistive switching memory devices |
| `TopologicalInsulator` | Quantum materials with surface states |
| `HolographicStorage` | Volumetric optical storage |
| `ProgrammableMatter` | Self-reconfiguring materials |
| `Custom` | User-defined material types |

## Features

- `default` - Core functionality
- `wasm` - WebAssembly support
- `python` - Python bindings via PyO3

```toml
[dependencies]
wia-material = { version = "1.0.0", features = ["wasm"] }
```

## Running Examples

```bash
# Basic usage
cargo run --example basic_usage

# Superconductor example
cargo run --example superconductor

# Protocol usage (Phase 3)
cargo run --example protocol_usage
```

## Running Tests

```bash
# Run all tests
cargo test

# Run with output
cargo test -- --nocapture

# Run specific test
cargo test test_create_superconductor
```

## License

MIT License - This standard belongs to humanity.

---

<div align="center">

🤟 **弘益人間** - Benefit All Humanity 🦀

© 2025 SmileStory Inc. / WIA

</div>
