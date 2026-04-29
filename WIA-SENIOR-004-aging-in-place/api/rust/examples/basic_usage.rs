//! Basic usage example for Aging In Place SDK
//!
//! 弘益人間 (Benefit All Humanity)

use wia_senior_004_aging_in_place::*;
use chrono::Utc;

#[tokio::main]
async fn main() -> Result<()> {
    println!("Aging In Place SDK v{}", VERSION);
    println!("弘益人間 - Benefit All Humanity\n");

    let item = AgingInPlace {
        id: utils::generate_id(),
        name: "Example".to_string(),
        created_at: Utc::now(),
        status: Status::Active,
    };

    validators::validate_item(&item)?;
    println!("✓ Item validated: {}", utils::format_info(&item));

    Ok(())
}
