//! Basic usage example
//! 弘益人間 (홍익인간) - Benefit All Humanity

use wia_first_contact::Client;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("🚀 WIA First Contact Protocol SDK");
    println!("弘익人間 - Technology for all\n");

    let client = Client::new("https://api.wia.global",
        std::env::var("WIA_API_KEY").unwrap_or_else(|_| "demo-key".to_string()))?;

    println!("SDK ready!");
    println!("\n弘益人間 - Benefit All Humanity 🌍");

    Ok(())
}
