//! Basic usage example for Senior Wearable SDK
//!
//! 弘益人間 (Benefit All Humanity)

use wia_senior_007_senior_wearable::*;
use chrono::Utc;

#[tokio::main]
async fn main() -> Result<()> {
    println!("Senior Wearable SDK v{}", VERSION);
    println!("弘益人間 - Benefit All Humanity\n");

    let item = SeniorWearable {
        id: utils::generate_id(),
        name: "Example".to_string(),
        created_at: Utc::now(),
        status: Status::Active,
    };

    validators::validate_item(&item)?;
    println!("✓ Item validated: {}", utils::format_info(&item));

    Ok(())
}
