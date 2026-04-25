//! Basic usage example for Elder Care Technology SDK
//!
//! 弘益人間 (Benefit All Humanity)

use wia_senior_001_elder_care_tech::*;
use chrono::Utc;

#[tokio::main]
async fn main() -> Result<()> {
    println!("Elder Care Technology SDK v{}", VERSION);
    println!("弘益人間 - Benefit All Humanity\n");

    let item = ElderCare {
        id: utils::generate_id(),
        name: "Example".to_string(),
        created_at: Utc::now(),
        status: Status::Active,
    };

    validators::validate_item(&item)?;
    println!("✓ Item validated: {}", utils::format_info(&item));

    Ok(())
}
