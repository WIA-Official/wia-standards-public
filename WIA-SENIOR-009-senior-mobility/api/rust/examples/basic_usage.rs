//! Basic usage example for Senior Mobility SDK
//!
//! 弘益人間 (Benefit All Humanity)

use wia_senior_009_senior_mobility::*;
use chrono::Utc;

#[tokio::main]
async fn main() -> Result<()> {
    println!("Senior Mobility SDK v{}", VERSION);
    println!("弘益人間 - Benefit All Humanity\n");

    let item = SeniorMobility {
        id: utils::generate_id(),
        name: "Example".to_string(),
        created_at: Utc::now(),
        status: Status::Active,
    };

    validators::validate_item(&item)?;
    println!("✓ Item validated: {}", utils::format_info(&item));

    Ok(())
}
