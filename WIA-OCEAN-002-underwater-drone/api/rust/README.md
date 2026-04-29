# WIA Ocean Underwater Drone Rust SDK

弘益人間 (Benefit All Humanity)

## Overview

Rust SDK for the WIA Ocean Underwater Drone Standard. This SDK provides tools for controlling AUVs, ROVs, and underwater gliders, including mission planning and telemetry monitoring.

## Installation

Add to your `Cargo.toml`:

```toml
[dependencies]
wia-ocean-002-underwater-drone = "1.0.0"
```

## Quick Start

```rust
use wia_ocean_002_underwater_drone::*;

#[tokio::main]
async fn main() -> Result<()> {
    // Create underwater drone
    let drone = UnderwaterDrone {
        id: utils::generate_drone_id(),
        name: "AUV-Neptune".to_string(),
        drone_type: DroneType::Auv,
        max_depth_meters: 6000.0,
        battery_capacity_kwh: 10.0,
        max_speed_knots: 5.0,
        sensors: vec![Sensor::Camera, Sensor::Sonar],
        status: DroneStatus::Idle,
    };

    // Create client
    let client = UnderwaterDroneClient::new(
        "https://api.wia.org/ocean/drone".to_string()
    );

    // Register drone
    let registered = client.register_drone(drone).await?;
    println!("Drone registered: {}", registered.name);

    Ok(())
}
```

## Features

- **Drone Control**: AUV, ROV, and glider management
- **Navigation**: Waypoint-based mission planning
- **Telemetry**: Real-time position and status monitoring
- **Battery Management**: Consumption estimation
- **Safety**: Depth limits and emergency protocols

## Drone Types

- **AUV**: Autonomous Underwater Vehicle
- **ROV**: Remotely Operated Vehicle
- **Glider**: Buoyancy-driven underwater glider
- **Hybrid**: Combination of multiple types

## Sensors

- Camera (Photo/Video)
- Sonar
- LiDAR
- CTD Sensor (Conductivity, Temperature, Depth)
- Chemical Sensor
- Magnetometer
- Accelerometer

## Waypoint Actions

- Take Photo
- Collect Sample
- Record Video
- Scan Sonar
- Measure Temperature
- Hover

## API

### Client

- `UnderwaterDroneClient::new(base_url)` - Create new client
- `client.register_drone(drone)` - Register drone
- `client.send_command(command)` - Send navigation command
- `client.get_telemetry(drone_id)` - Get current telemetry

### Validators

- `validate_drone(drone)` - Validate drone specification
- `validate_waypoint(waypoint)` - Validate waypoint
- `validate_command(command, drone)` - Validate navigation command
- `validate_telemetry(telemetry)` - Validate telemetry data

### Utils

- `calculate_mission_time(waypoints, speed)` - Estimate mission duration
- `calculate_distance(lat1, lon1, lat2, lon2)` - Calculate distance
- `estimate_battery_consumption(distance, speed, capacity)` - Estimate battery
- `format_drone_status(drone)` - Format drone information

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
