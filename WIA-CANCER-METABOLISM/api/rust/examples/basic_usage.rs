//! Basic usage example for WIA Cancer Metabolism Research SDK
//!
//! 弘益人間 (홍익인간) - Research for humanity

use wia_cancer_metabolism::{Client, MetabolicProfile, CancerType};
use uuid::Uuid;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("🔬 WIA Cancer Metabolism Research SDK");
    println!("弘益人間 - Advancing cancer research\n");

    let client = Client::new(
        "https://api.wia.global",
        std::env::var("WIA_API_KEY").unwrap_or_else(|_| "demo-key".to_string()),
    )?;

    println!("Analyzing metabolic profiles...");
    println!("\n弘益人間 - Research that benefits all 🌍");

    Ok(())
}
