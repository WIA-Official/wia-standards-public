//! Basic usage example
//! 弘익人間 - Protecting all children

use wia_child_parental_control::{Client, ParentalControl};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("🛡️ WIA Parental Control SDK");
    println!("弘益人間 - Child safety for all\n");

    let client = Client::new("https://api.wia.global",
        std::env::var("WIA_API_KEY").unwrap_or_else(|_| "demo-key".to_string()))?;

    println!("Managing parental controls...");
    println!("\n弘益人間 - Protecting children worldwide 🌍");

    Ok(())
}
