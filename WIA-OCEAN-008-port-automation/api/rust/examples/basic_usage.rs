//! Basic usage example for Port Automation SDK
//!
//! 弘益人間 (Benefit All Humanity)

use wia_ocean_008_port_automation::*;
use chrono::Utc;

#[tokio::main]
async fn main() -> Result<()> {
    println!("Port Automation SDK v{}", VERSION);
    println!("弘益人間 - Benefit All Humanity\n");

    let item = PortAutomation {
        id: utils::generate_id(),
        name: "Example".to_string(),
        created_at: Utc::now(),
        status: Status::Active,
    };

    validators::validate_item(&item)?;
    println!("✓ Item validated: {}", utils::format_info(&item));

    Ok(())
}
