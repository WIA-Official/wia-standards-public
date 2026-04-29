//! Basic usage example for Fall Detection SDK
//!
//! 弘益人間 (Benefit All Humanity)

use wia_senior_003_fall_detection::*;
use chrono::Utc;

#[tokio::main]
async fn main() -> Result<()> {
    println!("Fall Detection SDK v{}", VERSION);
    println!("弘益人間 - Benefit All Humanity\n");

    let item = FallDetection {
        id: utils::generate_id(),
        name: "Example".to_string(),
        created_at: Utc::now(),
        status: Status::Active,
    };

    validators::validate_item(&item)?;
    println!("✓ Item validated: {}", utils::format_info(&item));

    Ok(())
}
