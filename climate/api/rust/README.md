# WIA Climate - Rust API

**High-performance Rust implementation of the WIA Climate Standard**

[![Crates.io](https://img.shields.io/crates/v/wia-climate.svg)](https://crates.io/crates/wia-climate)
[![Documentation](https://docs.rs/wia-climate/badge.svg)](https://docs.rs/wia-climate)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)

---

## Overview

`wia-climate` is a Rust library for working with climate and environment data following the WIA Climate Standard specification. It provides type-safe data structures, validation, and serialization for climate data exchange.

## Features

- **Type-safe data structures** for all climate domains
- **Async-first design** with tokio runtime support
- **Validation** of data against the standard schema
- **JSON serialization** with serde
- **Simulator adapters** for testing and development
- **CMIP6 compatible** climate model data types
- **Communication protocol** (Phase 3) with message types
- **WebSocket transport** for real-time data exchange
- **Mock transport** for testing
- **Ecosystem integration** (Phase 4) with output adapters
- **Alert engine** for threshold-based notifications
- **Dashboard adapters** (Grafana, Cesium)
- **Storage adapters** (InfluxDB, TimescaleDB)
- **Webhook notifications** for alerts

## Supported Domains

| Domain | Type Identifier | Description |
|--------|-----------------|-------------|
| Carbon Capture | `carbon_capture` | CCUS, DAC, storage |
| Weather Control | `weather_control` | Cloud seeding, precipitation |
| Geoengineering | `geoengineering` | SRM, CDR interventions |
| Vertical Farming | `vertical_farming` | CEA, hydroponics |
| Ocean Cleanup | `ocean_cleanup` | Plastic removal, marine |
| Climate Model | `climate_model` | CMIP6 projections |

## Installation

Add to your `Cargo.toml`:

```toml
[dependencies]
wia-climate = "1.0.0"
tokio = { version = "1", features = ["full"] }
```

## Quick Start

### Creating a Climate Message

```rust
use wia_climate::prelude::*;

fn main() -> Result<()> {
    // Create a carbon capture message
    let message = ClimateMessage::builder()
        .location(Location::new(64.0, -21.0))
        .device(Device::new("Climeworks", "Orca DAC"))
        .carbon_capture_data(CarbonCaptureData {
            technology: CarbonCaptureTechnology::Dac,
            capture_rate_kg_per_hour: 125.5,
            co2_purity_percentage: Some(99.2),
            ..Default::default()
        })
        .meta(Metadata::with_quality(0.98))
        .build()?;

    // Serialize to JSON
    let json = message.to_json_pretty()?;
    println!("{}", json);

    Ok(())
}
```

### Using Simulator Adapters

```rust
use wia_climate::prelude::*;

#[tokio::main]
async fn main() -> Result<()> {
    // Create a carbon capture simulator
    let simulator = CarbonCaptureSimulator::new(
        Location::new(64.0, -21.0),
        Device::new("Climeworks", "Orca DAC Simulator"),
    )
    .with_capture_rate(125.0)
    .with_variation(0.1);

    // Read simulated data
    let message = simulator.read().await?;
    println!("Capture rate: {:?}", message.data);

    Ok(())
}
```

### Deserializing JSON

```rust
use wia_climate::prelude::*;

fn main() -> Result<()> {
    let json = r#"{
        "version": "1.0.0",
        "type": "carbon_capture",
        "timestamp": { "unix_ms": 1702468800000 },
        "location": { "latitude": 64.0, "longitude": -21.0 },
        "device": { "manufacturer": "Climeworks", "model": "Orca DAC" },
        "data": {
            "technology": "dac",
            "capture_rate_kg_per_hour": 125.5
        }
    }"#;

    let message = ClimateMessage::from_json(json)?;
    println!("Type: {:?}", message.data_type);

    Ok(())
}
```

### WebSocket Client (Phase 3)

```rust
use wia_climate::prelude::*;
use wia_climate::transport::WebSocketClient;

#[tokio::main]
async fn main() -> Result<()> {
    // Connect to server
    let mut client = WebSocketClient::new("sensor-001");
    client.connect("wss://api.example.com/wia-climate/v1/ws").await?;

    // Send climate data
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

### Protocol Message Builder

```rust
use wia_climate::protocol::*;

fn main() -> Result<(), &'static str> {
    // Build a subscribe message
    let msg = SubscribeBuilder::new()
        .topic("carbon_capture/*")
        .topic("vertical_farming/sensors/*")
        .qos(QoS::AtLeastOnce)
        .build()?;

    // Build a command message
    let cmd = CommandBuilder::new()
        .target("device-001")
        .action("set_capture_rate")
        .param("rate_kg_per_hour", 150.0)
        .timeout(5000)
        .build()?;

    Ok(())
}
```

### Integration Pipeline (Phase 4)

```rust
use wia_climate::prelude::*;
use wia_climate::integration::adapters::*;

#[tokio::main]
async fn main() -> Result<()> {
    // Create output manager with fanout strategy
    let config = OutputConfig::fanout()
        .with_buffer_size(100)
        .with_circuit_breaker(true);
    let mut manager = OutputManager::new(config);

    // Add adapters
    manager.add_adapter(ConsoleAdapter::new("debug").verbose());
    manager.add_adapter(WebhookAdapter::simple("alerts", "https://example.com/webhook"));

    // Process climate data through all adapters
    let message = ClimateMessage::builder()
        .location(Location::new(64.0, -21.0))
        .device(Device::new("Climeworks", "Orca DAC"))
        .carbon_capture_data(CarbonCaptureData::default())
        .build()?;

    let result = manager.process(&message).await?;
    println!("Processed by: {:?}", result.success);

    manager.shutdown().await?;
    Ok(())
}
```

### Alert Engine (Phase 4)

```rust
use wia_climate::prelude::*;

fn main() -> Result<()> {
    let mut engine = AlertEngine::new();

    // Add alert rules
    let rule = AlertRule::builder("capture-low", "Low Capture Rate")
        .data_type(DataType::CarbonCapture)
        .condition("capture_rate_kg_per_hour", ComparisonOperator::LessThan, 100.0)
        .duration_secs(300)
        .severity(AlertSeverity::Warning)
        .notify("webhook-alerts")
        .build();

    engine.add_rule(rule);

    // Evaluate messages
    let message = ClimateMessage::builder()
        .location(Location::new(64.0, -21.0))
        .device(Device::new("Climeworks", "Orca DAC"))
        .carbon_capture_data(CarbonCaptureData {
            capture_rate_kg_per_hour: 85.0, // Below threshold
            ..Default::default()
        })
        .build()?;

    let events = engine.evaluate(&message);
    for event in events {
        println!("Alert: {} [{:?}]", event.rule_name, event.severity);
    }
    Ok(())
}
```

## API Reference

### Core Types

| Type | Description |
|------|-------------|
| `ClimateMessage` | Main message container |
| `ClimateMessageBuilder` | Builder for creating messages |
| `DataType` | Enum of supported data types |
| `Location` | Geographic coordinates (WGS84) |
| `Device` | Device/sensor information |
| `Timestamp` | UNIX ms + ISO 8601 timestamp |
| `Metadata` | Quality score, source, etc. |

### Domain Data Types

| Type | Description |
|------|-------------|
| `CarbonCaptureData` | DAC, CCUS capture data |
| `WeatherControlData` | Cloud seeding operations |
| `GeoengineeringData` | SRM/CDR interventions |
| `VerticalFarmingData` | CEA environment data |
| `OceanCleanupData` | Marine debris collection |
| `ClimateModelData` | CMIP6 model output |

### Simulator Adapters

| Adapter | Description |
|---------|-------------|
| `CarbonCaptureSimulator` | Simulates DAC facility data |
| `VerticalFarmingSimulator` | Simulates CEA sensor data |
| `OceanCleanupSimulator` | Simulates cleanup operations |
| `ClimateModelSimulator` | Simulates CMIP6 projections |

### Protocol Types (Phase 3)

| Type | Description |
|------|-------------|
| `ProtocolMessage` | Protocol message wrapper |
| `MessageType` | Message type enum (connect, data, command, etc.) |
| `ConnectBuilder` | Builder for connect messages |
| `CommandBuilder` | Builder for command messages |
| `SubscribeBuilder` | Builder for subscribe messages |
| `MessageHandler` | Trait for handling incoming messages |
| `MessageDispatcher` | Routes messages to handlers |

### Transport Types

| Type | Description |
|------|-------------|
| `Transport` | Trait for transport implementations |
| `WebSocketTransport` | WebSocket transport layer |
| `WebSocketClient` | High-level WebSocket client |
| `MockTransport` | Mock transport for testing |

### Integration Types (Phase 4)

| Type | Description |
|------|-------------|
| `OutputAdapter` | Trait for output adapters |
| `OutputManager` | Coordinates multiple adapters |
| `OutputConfig` | Configuration for output manager |
| `ProcessStrategy` | Fanout, Pipeline, or Selective |
| `ConsoleAdapter` | Console output for debugging |
| `WebhookAdapter` | HTTP webhook notifications |
| `InfluxDBAdapter` | InfluxDB time-series storage |
| `AlertEngine` | Evaluates alert rules |
| `AlertRule` | Threshold-based alert definition |
| `AlertCondition` | Field comparison conditions |

## Examples

Run the examples with:

```bash
# Basic usage
cargo run --example basic_usage

# Carbon capture simulator
cargo run --example carbon_capture

# Climate model projections
cargo run --example climate_model

# WebSocket client demo
cargo run --example websocket_client

# WebSocket server demo
cargo run --example websocket_server

# Integration pipeline (Phase 4)
cargo run --example integration_pipeline

# Alert engine (Phase 4)
cargo run --example alert_engine
```

## Testing

```bash
# Run all tests
cargo test

# Run with verbose output
cargo test -- --nocapture

# Run specific test
cargo test test_carbon_capture_message_creation
```

## Features

Enable optional features in `Cargo.toml`:

```toml
[dependencies]
wia-climate = { version = "1.0.0", features = ["wasm", "python"] }
```

| Feature | Description |
|---------|-------------|
| `wasm` | WebAssembly support via wasm-bindgen |
| `python` | Python bindings via PyO3 |
| `full` | All optional features |

## Project Structure

```
src/
├── lib.rs              # Library entry point
├── types.rs            # All type definitions
├── error.rs            # Error types
├── core/
│   ├── mod.rs
│   └── climate.rs      # ClimateMessage & builder
├── adapters/
│   ├── mod.rs
│   └── simulator.rs    # Simulator adapters
├── protocol/           # Phase 3: Communication protocol
│   ├── mod.rs
│   ├── message.rs      # ProtocolMessage type
│   ├── message_types.rs # Payload types
│   ├── builder.rs      # Message builders
│   └── handler.rs      # Message handlers
├── transport/          # Phase 3: Transport layers
│   ├── mod.rs
│   ├── websocket.rs    # WebSocket transport
│   └── mock.rs         # Mock for testing
└── integration/        # Phase 4: Ecosystem integration
    ├── mod.rs
    ├── adapter.rs      # OutputAdapter trait
    ├── manager.rs      # OutputManager
    ├── alert.rs        # Alert engine
    └── adapters/       # Built-in adapters
        ├── mod.rs
        ├── console.rs  # Console output
        ├── webhook.rs  # Webhook notifications
        └── influxdb.rs # InfluxDB storage

tests/
└── integration_test.rs # Integration tests

examples/
├── basic_usage.rs        # Basic usage patterns
├── carbon_capture.rs     # Carbon capture example
├── climate_model.rs      # Climate model example
├── websocket_client.rs   # WebSocket client example
├── websocket_server.rs   # WebSocket server example
├── integration_pipeline.rs # Output pipeline example
└── alert_engine.rs       # Alert engine example
```

## Related Standards

- [CF Conventions](https://cfconventions.org/) - Climate data metadata
- [CMIP6](https://pcmdi.llnl.gov/CMIP6/) - Climate model intercomparison
- [ISO 27914](https://www.iso.org/standard/64148.html) - Carbon capture & storage

## License

MIT License - This standard belongs to humanity.

---

<div align="center">

**弘益人間** - Benefit All Humanity

🌍 🦀

© 2025 SmileStory Inc. / WIA

</div>
