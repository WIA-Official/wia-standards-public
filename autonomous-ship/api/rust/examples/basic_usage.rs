//! Basic usage example for Autonomous Ship SDK
//!
//! 弘益人間 (Benefit All Humanity)

use autonomous_ship::*;
use chrono::Utc;

#[tokio::main]
async fn main() -> Result<()> {
    println!("Autonomous Ship SDK v{}", VERSION);
    println!("弘益人間 - Benefit All Humanity\n");

    let resource = Resource {
        id: utils::generate_id(),
        name: "Example".to_string(),
        created_at: Utc::now(),
        status: Status::Active,
    };

    validators::validate_resource(&resource)?;
    println!("✓ Resource validated: {}", utils::format_info(&resource));

    Ok(())
}
