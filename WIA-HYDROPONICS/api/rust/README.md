# WIA-HYDROPONICS Rust SDK

> 弘益人間 (홍익인간) - Benefit All Humanity through Sustainable Agriculture

Official Rust SDK for the WIA-HYDROPONICS standard - Smart Hydroponic System Management.

## Features

- **System Management**: Monitor and control hydroponic systems
- **Plant Tracking**: Track plant growth and health
- **Nutrient Control**: Manage pH, EC, and nutrient solutions
- **Environment Monitoring**: Temperature, humidity, light, CO2 tracking
- **Alert System**: Real-time alerts for system issues
- **Harvest Management**: Track and optimize harvest yields
- **Type-Safe**: Full Rust type safety
- **Async/Await**: Modern async Rust

## Installation

```toml
[dependencies]
wia-hydroponics = "1.0.0"
tokio = { version = "1.0", features = ["full"] }
```

## Quick Start

```rust
use wia_hydroponics::HydroponicsClient;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let client = HydroponicsClient::new(
        "https://api.wia-hydroponics.org",
        "your-api-key"
    )?;

    // List systems
    let systems = client.list_systems().await?;
    println!("Found {} systems", systems.len());

    // Get nutrient data
    let system_id = systems[0].id;
    let nutrients = client.get_nutrient_data(system_id, 24).await?;

    if let Some(latest) = nutrients.last() {
        println!("pH: {:.2}, EC: {:.2}", latest.ph_level, latest.ec_level);
    }

    Ok(())
}
```

## Examples

### System Monitoring

```rust
let systems = client.list_systems().await?;
let system = client.get_system(system_id).await?;

println!("System: {} ({:?})", system.name, system.system_type);
println!("Capacity: {:.1}L", system.capacity_liters);
```

### Plant Management

```rust
let plants = client.get_plants(system_id).await?;

for plant in plants {
    println!("{} - {:?} (Health: {:.1}%)",
        plant.species,
        plant.growth_stage,
        plant.health_score
    );
}
```

### Nutrient Control

```rust
// Adjust pH
client.adjust_ph(system_id, 6.0).await?;

// Adjust EC
client.adjust_ec(system_id, 1.5).await?;

// Check optimal ranges
let (min_ph, max_ph) = utils::optimal_ph_range("lettuce");
println!("Optimal pH: {:.1} - {:.1}", min_ph, max_ph);
```

### Alert Management

```rust
let alerts = client.get_alerts(system_id).await?;

for alert in alerts {
    if alert.resolved_at.is_none() {
        println!("[{:?}] {:?}: {}",
            alert.severity,
            alert.alert_type,
            alert.message
        );
    }
}
```

## Philosophy

**弘益人間 (홍익인간) - Benefit All Humanity**

Sustainable agriculture through technology, making fresh food accessible to everyone.

## License

MIT License

---

**© 2025 SmileStory Inc. / WIA**

弘益人間 (홍익인간) · Benefit All Humanity
