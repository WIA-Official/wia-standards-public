//! Basic usage example for Dementia Care SDK
//!
//! 弘益人間 (Benefit All Humanity)

use wia_senior_002_dementia_care::*;
use chrono::Utc;

#[tokio::main]
async fn main() -> Result<()> {
    println!("Dementia Care SDK v{}", VERSION);
    println!("弘益人間 - Benefit All Humanity\n");

    let item = DementiaCare {
        id: utils::generate_id(),
        name: "Example".to_string(),
        created_at: Utc::now(),
        status: Status::Active,
    };

    validators::validate_item(&item)?;
    println!("✓ Item validated: {}", utils::format_info(&item));

    Ok(())
}
