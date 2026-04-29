//! Basic usage example for WIA Autoimmune Disease Management SDK
//!
//! 弘益人間 (홍익인간) - Healthcare for all humanity

use wia_autoimmune::{Client, Patient, AutoimmuneDiseaseType, TreatmentPlan};
use uuid::Uuid;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("🏥 WIA Autoimmune Disease Management SDK");
    println!("弘익人間 - Healthcare for everyone\n");

    let client = Client::new(
        "https://api.wia.global",
        std::env::var("WIA_API_KEY").unwrap_or_else(|_| "demo-key".to_string()),
    )?;

    println!("Managing autoimmune disease treatment...");
    println!("\n弘益人間 - Better health for all 🌍");

    Ok(())
}
