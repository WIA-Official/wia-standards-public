# WIA Ocean Deep Sea Exploration Rust SDK

弘益人間 (Benefit All Humanity)

## Overview

Rust SDK for the WIA Ocean Deep Sea Exploration Standard. This SDK provides tools for planning, executing, and monitoring deep sea exploration missions.

## Installation

Add to your `Cargo.toml`:

```toml
[dependencies]
wia-ocean-001-deep-sea-exploration = "1.0.0"
```

## Quick Start

```rust
use wia_ocean_001_deep_sea_exploration::*;
use chrono::Utc;

#[tokio::main]
async fn main() -> Result<()> {
    // Create exploration mission
    let mission = ExplorationMission {
        id: utils::generate_mission_id(),
        name: "Mariana Trench Survey".to_string(),
        location: OceanLocation {
            latitude: 36.2048,
            longitude: -112.0723,
            ocean_name: "Pacific Ocean".to_string(),
            zone: OceanZone::Hadopelagic,
        },
        depth_meters: 10994.0,
        start_time: Utc::now(),
        duration_hours: 8.0,
        vessel: Vessel {
            vessel_id: "DSV-001".to_string(),
            vessel_type: VesselType::DeepSeaDrone,
            max_depth_meters: 11000.0,
            crew_capacity: 3,
            equipment: vec!["4K Camera".to_string()],
        },
        objectives: vec![MissionObjective::BiologicalSurvey],
        status: MissionStatus::Planned,
    };

    // Create client
    let client = DeepSeaExplorationClient::new(
        "https://api.wia.org/ocean/deep-sea".to_string()
    );

    // Create mission
    let created = client.create_mission(mission).await?;
    println!("Mission created: {}", created.name);

    Ok(())
}
```

## Features

- **Mission Planning**: Create and manage exploration missions
- **Zone Classification**: Automatic ocean zone determination
- **Pressure Calculation**: Water pressure estimation at depth
- **Vessel Recommendation**: Optimal vessel type selection
- **Type Safety**: Strong type system with Rust

## Ocean Zones

- **Epipelagic** (0-200m): Sunlight zone
- **Mesopelagic** (200-1000m): Twilight zone
- **Bathypelagic** (1000-4000m): Midnight zone
- **Abyssopelagic** (4000-6000m): Abyss
- **Hadopelagic** (6000m+): Trenches

## Vessel Types

- **ROV**: Remotely Operated Vehicle (< 300m)
- **Submersible**: Manned submersible (300-2000m)
- **AUV**: Autonomous Underwater Vehicle (2000-6000m)
- **Deep Sea Drone**: Ultra-deep exploration (6000m+)

## Mission Objectives

- Biological Survey
- Geological Mapping
- Resource Discovery
- Wreck Exploration
- Environmental Monitoring
- Scientific Research

## API

### Client

- `DeepSeaExplorationClient::new(base_url)` - Create new client
- `client.create_mission(mission)` - Create mission
- `client.get_mission(id)` - Get mission by ID
- `client.submit_data(data)` - Submit exploration data

### Validators

- `validate_location(location)` - Validate ocean location
- `validate_mission(mission)` - Validate exploration mission
- `validate_vessel(vessel)` - Validate vessel specification

### Utils

- `determine_zone(depth)` - Determine ocean zone
- `calculate_pressure_bar(depth)` - Calculate water pressure
- `recommend_vessel_type(depth)` - Recommend vessel type
- `estimate_mission_duration(depth, objectives)` - Estimate duration
- `format_mission_summary(mission)` - Format mission info

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
