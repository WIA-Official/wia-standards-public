//! Basic usage example for Intergenerational SDK
//!
//! 弘益人間 (Benefit All Humanity)

use wia_senior_010_intergenerational::*;
use chrono::Utc;

#[tokio::main]
async fn main() -> Result<()> {
    println!("Intergenerational SDK v{}", VERSION);
    println!("弘益人間 - Benefit All Humanity\n");

    let item = Intergenerational {
        id: utils::generate_id(),
        name: "Example".to_string(),
        created_at: Utc::now(),
        status: Status::Active,
    };

    validators::validate_item(&item)?;
    println!("✓ Item validated: {}", utils::format_info(&item));

    Ok(())
}
