//! Basic usage example for Submarine Technology SDK
//!
//! 弘益人間 (Benefit All Humanity)

use wia_ocean_006_submarine_tech::*;
use chrono::Utc;

#[tokio::main]
async fn main() -> Result<()> {
    println!("Submarine Technology SDK v{}", VERSION);
    println!("弘益人間 - Benefit All Humanity\n");

    let item = SubmarineTech {
        id: utils::generate_id(),
        name: "Example".to_string(),
        created_at: Utc::now(),
        status: Status::Active,
    };

    validators::validate_item(&item)?;
    println!("✓ Item validated: {}", utils::format_info(&item));

    Ok(())
}
