//! Basic usage example for Access Control SDK
//!
//! 弘益人間 (Benefit All Humanity)

use access_control::*;
use chrono::Utc;

#[tokio::main]
async fn main() -> Result<()> {
    println!("Access Control SDK v{}", VERSION);
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
