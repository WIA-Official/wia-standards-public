//! Basic usage example for WIA-DATA_QUALITY SDK
//!
//! 홍익인간 (弘益人間) - Benefit All Humanity through Quality Data

use wia_data_quality::DataQualityClient;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("📊 WIA-DATA_QUALITY Demo");
    println!("========================\n");

    let api_key = std::env::var("WIA_API_KEY").unwrap_or_else(|_| "demo-key".to_string());
    let client = DataQualityClient::new(api_key);

    println!("🔍 Assessing data quality...");
    println!("   Dataset: sample-dataset-001");

    println!("\n🌟 Demo complete - 홍익인간 (弘益人間)");
    Ok(())
}
