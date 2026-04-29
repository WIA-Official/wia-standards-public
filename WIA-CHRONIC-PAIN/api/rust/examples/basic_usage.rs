//! Basic usage example for WIA Chronic Pain Management SDK
//!
//! 弘益人間 (홍익인간) - Relief for all who suffer

use wia_chronic_pain::{Client, PainProfile, PainType};
use uuid::Uuid;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("💊 WIA Chronic Pain Management SDK");
    println!("弘益人間 - Pain relief for everyone\n");

    let client = Client::new(
        "https://api.wia.global",
        std::env::var("WIA_API_KEY").unwrap_or_else(|_| "demo-key".to_string()),
    )?;

    println!("Managing chronic pain treatment...");
    println!("\n弘益人間 - Better quality of life for all 🌍");

    Ok(())
}
