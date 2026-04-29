//! Basic usage example for WIA-HOME SDK
//!
//! 弘益人間 - Benefit All Humanity through smart home automation

use wia_home::{HomeClient, DeviceType, SceneAction, Scene, SceneTrigger, TriggerType};
use uuid::Uuid;
use chrono::Utc;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("WIA-HOME SDK Example");
    println!("弘益人間 (홍익인간) - Benefit All Humanity\n");

    // Create client
    let client = HomeClient::new(
        "https://api.wia-home.org",
        "your-api-key-here"
    )?;

    // Discover devices
    println!("Discovering devices...");
    match client.discover_devices().await {
        Ok(devices) => {
            println!("Found {} devices:", devices.len());
            for device in &devices {
                println!("  - {} ({:?}) - {:?}",
                    device.name,
                    device.device_type,
                    device.status
                );
            }
        }
        Err(e) => println!("Error discovering devices: {}", e),
    }

    // List rooms
    println!("\nListing rooms...");
    match client.list_rooms().await {
        Ok(rooms) => {
            println!("Found {} rooms:", rooms.len());
            for room in &rooms {
                println!("  - {} ({} devices)", room.name, room.devices.len());
            }
        }
        Err(e) => println!("Error listing rooms: {}", e),
    }

    // Get home configuration
    println!("\nGetting home configuration...");
    match client.get_home_config().await {
        Ok(config) => {
            println!("Home: {}", config.name);
            println!("Timezone: {}", config.timezone);
            if let Some(address) = &config.address {
                println!("Address: {}", address);
            }
        }
        Err(e) => println!("Error getting config: {}", e),
    }

    // Example: Control a light (mock device ID)
    let device_id = Uuid::new_v4();
    println!("\nControlling device {}...", device_id);
    let control_params = serde_json::json!({
        "brightness": 80,
        "color": "#FFFFFF"
    });

    match client.control_device(device_id, "set_state", control_params).await {
        Ok(_) => println!("Device controlled successfully"),
        Err(e) => println!("Error controlling device: {}", e),
    }

    // Create a scene
    println!("\nCreating automation scene...");
    let scene = Scene {
        id: Uuid::new_v4(),
        name: "Good Morning".to_string(),
        description: Some("Morning routine automation".to_string()),
        actions: vec![
            SceneAction {
                device_id: Uuid::new_v4(),
                action: "turn_on".to_string(),
                parameters: serde_json::json!({"brightness": 50}),
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

    match client.create_scene(&scene).await {
        Ok(created) => println!("Scene created: {}", created.name),
        Err(e) => println!("Error creating scene: {}", e),
    }

    // Get energy data (mock device ID)
    println!("\nGetting energy consumption data...");
    match client.get_energy_data(device_id, 7).await {
        Ok(data) => {
            let total: f64 = data.iter().map(|d| d.consumption_kwh).sum();
            println!("Total energy consumption (7 days): {:.2} kWh", total);
        }
        Err(e) => println!("Error getting energy data: {}", e),
    }

    println!("\n✅ Example completed!");
    Ok(())
}
