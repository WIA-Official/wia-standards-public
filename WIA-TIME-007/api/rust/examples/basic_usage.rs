//! Basic usage example for Time Management 007 SDK
//!
//! 弘益人間 (Benefit All Humanity)

use wia_time_007::*;
use chrono::Utc;

#[tokio::main]
async fn main() -> Result<()> {
    println!("Time Management 007 SDK v{}", VERSION);
    println!("弘益人間 - Benefit All Humanity\n");

    let item = TimeManagement {
        id: utils::generate_id(),
        name: "Example".to_string(),
        created_at: Utc::now(),
        status: Status::Active,
    };

    validators::validate_item(&item)?;
    println!("✓ Item validated: {}", utils::format_info(&item));

    Ok(())
}
