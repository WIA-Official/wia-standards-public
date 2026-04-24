//! Basic usage example for Big Data SDK
//!
//! 弘益人間 (Benefit All Humanity)

use big_data::*;
use chrono::Utc;

#[tokio::main]
async fn main() -> Result<()> {
    println!("Big Data SDK v{}", VERSION);
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
