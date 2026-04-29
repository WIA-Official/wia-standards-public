# WIA-HOME Rust SDK

> 弘益人間 (홍익인간) - Benefit All Humanity through Smart Home Technology

Official Rust SDK for the WIA-HOME standard - Smart Home Automation and IoT Device Management.

## Features

- **Device Management**: Discover, control, and monitor smart home devices
- **Room Organization**: Organize devices by rooms and floors
- **Scene Automation**: Create and execute complex automation scenes
- **Energy Monitoring**: Track and optimize energy consumption
- **Real-time Control**: Instant device control and status updates
- **Type-Safe**: Full Rust type safety with comprehensive error handling
- **Async/Await**: Modern async Rust for high performance
- **Well-Tested**: Comprehensive test coverage

## Installation

Add this to your `Cargo.toml`:

```toml
[dependencies]
wia-home = "1.0.0"
tokio = { version = "1.0", features = ["full"] }
```

## Quick Start

```rust
use wia_home::{HomeClient, DeviceType};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Create client
    let client = HomeClient::new(
        "https://api.wia-home.org",
        "your-api-key"
    )?;

    // Discover devices
    let devices = client.discover_devices().await?;
    println!("Found {} devices", devices.len());

    // Control a device
    let device_id = devices[0].id;
    client.control_device(
        device_id,
        "turn_on",
        serde_json::json!({"brightness": 80})
    ).await?;

    Ok(())
}
```

## Examples

### Device Discovery and Control

```rust
use wia_home::{HomeClient, DeviceType};

let client = HomeClient::new("https://api.wia-home.org", "api-key")?;

// Discover all devices
let devices = client.discover_devices().await?;

// Get specific device
let device = client.get_device(device_id).await?;

// Control device
client.control_device(
    device_id,
    "set_temperature",
    serde_json::json!({"temperature": 72})
).await?;
```

### Scene Automation

```rust
use wia_home::{Scene, SceneAction, SceneTrigger, TriggerType};
use uuid::Uuid;
use chrono::Utc;

let scene = Scene {
    id: Uuid::new_v4(),
    name: "Good Morning".to_string(),
    description: Some("Morning routine".to_string()),
    actions: vec![
        SceneAction {
            device_id: light_id,
            action: "turn_on".to_string(),
            parameters: serde_json::json!({"brightness": 60}),
            delay_seconds: None,
        },
    ],
    triggers: vec![
        SceneTrigger {
            trigger_type: TriggerType::Schedule,
            condition: serde_json::json!({"time": "07:00"}),
        },
    ],
    enabled: true,
    created_at: Utc::now(),
};

let created_scene = client.create_scene(&scene).await?;
client.execute_scene(created_scene.id).await?;
```

### Energy Monitoring

```rust
// Get energy consumption for a device
let energy_data = client.get_energy_data(device_id, 7).await?;

// Calculate total consumption
let total: f64 = energy_data.iter()
    .map(|d| d.consumption_kwh)
    .sum();

println!("Total energy (7 days): {:.2} kWh", total);
```

### Room Management

```rust
// List all rooms
let rooms = client.list_rooms().await?;

for room in rooms {
    println!("Room: {} ({} devices)", room.name, room.devices.len());
}

// Filter devices by room
let kitchen_devices = utils::filter_devices_by_room(&devices, "Kitchen");
```

## API Overview

### HomeClient

The main client for interacting with the WIA-HOME API:

- `new(base_url, api_key)` - Create a new client
- `discover_devices()` - Discover all devices
- `get_device(id)` - Get specific device
- `control_device(id, action, params)` - Control a device
- `list_rooms()` - List all rooms
- `create_scene(scene)` - Create automation scene
- `execute_scene(id)` - Execute a scene
- `get_energy_data(id, days)` - Get energy consumption
- `get_home_config()` - Get home configuration

### Types

- `Device` - Smart home device information
- `Room` - Room configuration
- `Scene` - Automation scene
- `EnergyData` - Energy consumption data
- `DeviceType` - Device type enumeration
- `DeviceStatus` - Device status enumeration

### Error Handling

```rust
use wia_home::HomeError;

match client.get_device(device_id).await {
    Ok(device) => println!("Device: {}", device.name),
    Err(HomeError::DeviceNotFound(id)) => {
        eprintln!("Device {} not found", id);
    }
    Err(HomeError::NetworkError(e)) => {
        eprintln!("Network error: {}", e);
        // Retry logic here
    }
    Err(e) => eprintln!("Error: {}", e),
}
```

## Philosophy

**弘益人間 (홍익인간) - Benefit All Humanity**

The WIA-HOME standard is built on the principle of making smart home technology accessible, reliable, and beneficial to all people. We believe that home automation should:

- Enhance quality of life
- Promote energy efficiency
- Ensure privacy and security
- Be accessible to everyone
- Work across all devices and platforms

## Requirements

- Rust 1.70 or higher
- Tokio runtime for async operations

## Dependencies

- `serde` - Serialization/deserialization
- `tokio` - Async runtime
- `reqwest` - HTTP client
- `uuid` - UUID generation
- `chrono` - Date/time handling
- `thiserror` - Error handling

## Testing

Run the test suite:

```bash
cargo test
```

Run integration tests:

```bash
cargo test --test integration_test
```

## Examples

Run the basic usage example:

```bash
cargo run --example basic_usage
```

## Documentation

Generate and view documentation:

```bash
cargo doc --open
```

## Contributing

Contributions are welcome! Please read our contributing guidelines and submit pull requests to our repository.

## License

MIT License - see LICENSE file for details

## Support

- Documentation: https://docs.wia.org/home
- API Reference: https://api.wia.org/home/reference
- GitHub: https://github.com/WIA-Official/wia-standards
- Issues: https://github.com/WIA-Official/wia-standards/issues

## Related Standards

- WIA-SOCIAL - Social Media Integration
- WIA-INTENT - Intent Recognition
- WIA-OMNI-API - Unified API Gateway

---

**© 2025 SmileStory Inc. / WIA**

弘益人間 (홍익인간) · Benefit All Humanity
