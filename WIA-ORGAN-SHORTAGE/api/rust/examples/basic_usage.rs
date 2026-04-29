//! Basic usage example for Organ Shortage SDK
//!
//! 弘益人間 (Benefit All Humanity)

use wia_organ_shortage::*;
use chrono::Utc;

#[tokio::main]
async fn main() -> Result<()> {
    println!("Organ Shortage SDK v{}", VERSION);
    println!("弘益人間 - Benefit All Humanity\n");

    let item = OrganShortage {
        id: utils::generate_id(),
        name: "Example".to_string(),
        created_at: Utc::now(),
        status: Status::Active,
    };

    validators::validate_item(&item)?;
    println!("✓ Item validated: {}", utils::format_info(&item));

    Ok(())
}
