//! Basic usage example for Loneliness Prevention SDK
//!
//! 弘益人間 (Benefit All Humanity)

use wia_senior_005_loneliness_prevention::*;
use chrono::Utc;

#[tokio::main]
async fn main() -> Result<()> {
    println!("Loneliness Prevention SDK v{}", VERSION);
    println!("弘益人間 - Benefit All Humanity\n");

    let item = LonelinessPrevention {
        id: utils::generate_id(),
        name: "Example".to_string(),
        created_at: Utc::now(),
        status: Status::Active,
    };

    validators::validate_item(&item)?;
    println!("✓ Item validated: {}", utils::format_info(&item));

    Ok(())
}
