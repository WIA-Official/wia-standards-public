//! Basic usage example for WIA-HYDROPONICS SDK
//!
//! 弘益人間 - Sustainable agriculture for all humanity

use wia_hydroponics::{HydroponicsClient, Plant, GrowthStage, HarvestRecord};
use uuid::Uuid;
use chrono::Utc;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("WIA-HYDROPONICS SDK Example");
    println!("弘益人間 (홍익인간) - Benefit All Humanity\n");

    // Create client
    let client = HydroponicsClient::new(
        "https://api.wia-hydroponics.org",
        "your-api-key-here"
    )?;

    // List all systems
    println!("Listing hydroponic systems...");
    match client.list_systems().await {
        Ok(systems) => {
            println!("Found {} systems:", systems.len());
            for system in &systems {
                println!("  - {} ({:?}) - {} plants",
                    system.name,
                    system.system_type,
                    system.plant_count
                );
            }
        }
        Err(e) => println!("Error listing systems: {}", e),
    }

    // Mock system ID for examples
    let system_id = Uuid::new_v4();

    // Get system details
    println!("\nGetting system details...");
    match client.get_system(system_id).await {
        Ok(system) => {
            println!("System: {}", system.name);
            println!("Type: {:?}", system.system_type);
            println!("Capacity: {:.1}L", system.capacity_liters);
            println!("Status: {:?}", system.status);
        }
        Err(e) => println!("Error getting system: {}", e),
    }

    // Get plants
    println!("\nGetting plants in system...");
    match client.get_plants(system_id).await {
        Ok(plants) => {
            println!("Found {} plants:", plants.len());
            for plant in &plants {
                println!("  - {} ({:?}) - Health: {:.1}%",
                    plant.species,
                    plant.growth_stage,
                    plant.health_score
                );
            }
        }
        Err(e) => println!("Error getting plants: {}", e),
    }

    // Get environment data
    println!("\nGetting environment data (24 hours)...");
    match client.get_environment_data(system_id, 24).await {
        Ok(data) => {
            if let Some(latest) = data.last() {
                println!("Latest environment readings:");
                println!("  Temperature: {:.1}°C", latest.temperature_celsius);
                println!("  Humidity: {:.1}%", latest.humidity_percent);
                println!("  Light: {:.1} lux", latest.light_intensity_lux);
            }
        }
        Err(e) => println!("Error getting environment data: {}", e),
    }

    // Get nutrient data
    println!("\nGetting nutrient data (24 hours)...");
    match client.get_nutrient_data(system_id, 24).await {
        Ok(data) => {
            if let Some(latest) = data.last() {
                println!("Latest nutrient readings:");
                println!("  pH: {:.2}", latest.ph_level);
                println!("  EC: {:.2} mS/cm", latest.ec_level);
                println!("  Temperature: {:.1}°C", latest.temperature_celsius);
            }
        }
        Err(e) => println!("Error getting nutrient data: {}", e),
    }

    // Adjust pH
    println!("\nAdjusting pH level...");
    match client.adjust_ph(system_id, 6.0).await {
        Ok(_) => println!("pH adjusted to 6.0"),
        Err(e) => println!("Error adjusting pH: {}", e),
    }

    // Get nutrient formulas
    println!("\nGetting nutrient formulas...");
    match client.get_nutrient_formulas().await {
        Ok(formulas) => {
            println!("Available formulas: {}", formulas.len());
            for formula in formulas.iter().take(3) {
                println!("  - {} (pH: {:.1}, EC: {:.1})",
                    formula.name,
                    formula.target_ph,
                    formula.target_ec
                );
            }
        }
        Err(e) => println!("Error getting formulas: {}", e),
    }

    // Get alerts
    println!("\nGetting system alerts...");
    match client.get_alerts(system_id).await {
        Ok(alerts) => {
            let active_alerts: Vec<_> = alerts.iter()
                .filter(|a| a.resolved_at.is_none())
                .collect();
            println!("Active alerts: {}", active_alerts.len());
            for alert in active_alerts {
                println!("  - [{:?}] {:?}: {}",
                    alert.severity,
                    alert.alert_type,
                    alert.message
                );
            }
        }
        Err(e) => println!("Error getting alerts: {}", e),
    }

    println!("\n✅ Example completed!");
    Ok(())
}
