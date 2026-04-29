//! Basic usage example for Ocean Resource SDK
//!
//! 弘益人間 (Benefit All Humanity)

use wia_ocean_010_ocean_resource::*;
use chrono::Utc;

#[tokio::main]
async fn main() -> Result<()> {
    println!("Ocean Resource SDK v{}", VERSION);
    println!("弘益人間 - Benefit All Humanity\n");

    let item = OceanResource {
        id: utils::generate_id(),
        name: "Example".to_string(),
        created_at: Utc::now(),
        status: Status::Active,
    };

    validators::validate_item(&item)?;
    println!("✓ Item validated: {}", utils::format_info(&item));

    Ok(())
}
