//! Basic usage example for WIA-IND-001
//!
//! 弘益人間 - Benefit All Humanity through fashion technology

use wia_fashion_tech::FashionTechClient;
use uuid::Uuid;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("WIA-IND-001 Fashion Technology SDK");
    println!("弘益人間 - Benefit All Humanity\n");

    let client = FashionTechClient::new(
        "https://api.wia-fashion-tech.org",
        "your-api-key-here"
    )?;

    let product_id = Uuid::new_v4();
    let user_id = Uuid::new_v4();

    // Get product
    match client.get_product(product_id).await {
        Ok(product) => println!("Product: {} by {}", product.name, product.brand),
        Err(e) => println!("Error: {}", e),
    }

    // Get size recommendation
    match client.get_size_recommendation(user_id, product_id).await {
        Ok(rec) => println!("Recommended size: {} (confidence: {:.1}%)", rec.recommended_size, rec.confidence * 100.0),
        Err(e) => println!("Error: {}", e),
    }

    // Create virtual try-on
    match client.create_virtual_tryon(user_id, product_id).await {
        Ok(tryon) => println!("Virtual try-on created: {}", tryon.image_url),
        Err(e) => println!("Error: {}", e),
    }

    println!("\n✅ Example completed!");
    Ok(())
}
