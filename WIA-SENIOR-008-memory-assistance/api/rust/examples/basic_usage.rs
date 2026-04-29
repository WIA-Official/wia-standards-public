//! Basic usage example for Memory Assistance SDK
//!
//! 弘益人間 (Benefit All Humanity)

use wia_senior_008_memory_assistance::*;
use chrono::Utc;

#[tokio::main]
async fn main() -> Result<()> {
    println!("Memory Assistance SDK v{}", VERSION);
    println!("弘益人間 - Benefit All Humanity\n");

    let item = MemoryAssistance {
        id: utils::generate_id(),
        name: "Example".to_string(),
        created_at: Utc::now(),
        status: Status::Active,
    };

    validators::validate_item(&item)?;
    println!("✓ Item validated: {}", utils::format_info(&item));

    Ok(())
}
