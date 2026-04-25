//! Basic usage example for Marine Sensor SDK
//!
//! 弘益人間 (Benefit All Humanity)

use wia_ocean_007_marine_sensor::*;
use chrono::Utc;

#[tokio::main]
async fn main() -> Result<()> {
    println!("Marine Sensor SDK v{}", VERSION);
    println!("弘益人間 - Benefit All Humanity\n");

    let item = MarineSensor {
        id: utils::generate_id(),
        name: "Example".to_string(),
        created_at: Utc::now(),
        status: Status::Active,
    };

    validators::validate_item(&item)?;
    println!("✓ Item validated: {}", utils::format_info(&item));

    Ok(())
}
