//! Basic usage example for WIA-IND-004-beauty-tech
//! 弘益人間 - Benefit All Humanity

use wia_ind_004_beauty_tech::Client;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("WIA-IND-004-beauty-tech SDK Example");
    println!("弘益人間 - Benefit All Humanity\n");

    let client = Client::new("https://api.wia.org", "your-api-key-here")?;

    match client.list_resources().await {
        Ok(resources) => {
            println!("Found {} resources", resources.len());
            for resource in resources {
                println!("  - {} ({:?})", resource.name, resource.status);
            }
        }
        Err(e) => println!("Error: {}", e),
    }

    println!("\n✅ Example completed!");
    Ok(())
}
