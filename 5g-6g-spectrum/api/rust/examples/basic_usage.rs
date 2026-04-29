//! Basic usage example for the 5g-6g-spectrum SDK

use 5g_6g_spectrum_sdk::{FiveGSixGSpectrumClient, Config, CreateFiveGSixGSpectrumRequest};
use serde_json::json;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Initialize client
    let client = FiveGSixGSpectrumClient::new(Config {
        api_key: std::env::var("WIA_API_KEY").unwrap_or_else(|_| "demo-key".to_string()),
        base_url: None,
        timeout_secs: 30,
    });

    println!("🚀 5g-6g-spectrum SDK Example");
    println!("弘益人間 (Benefit All Humanity)\n");

    // Example 1: Validate data
    println!("1️⃣  Validating data...");
    let data = json!({
        "field1": "value1",
        "field2": 123
    });

    let validation = client.validate(&data).await?;
    println!("   Valid: {}", validation.valid);
    if !validation.errors.is_empty() {
        println!("   Errors: {:?}", validation.errors);
    }

    // Example 2: Create data
    println!("\n2️⃣  Creating data...");
    let request = CreateFiveGSixGSpectrumRequest {
        content: data.clone(),
        metadata: Some(json!({
            "source": "example",
            "version": "1.0"
        })),
    };

    match client.create(request).await {
        Ok(created) => {
            println!("   Created ID: {}", created.id);
            println!("   Timestamp: {}", created.created_at);

            // Example 3: Retrieve data
            println!("\n3️⃣  Retrieving data...");
            match client.get(&created.id.to_string()).await {
                Ok(retrieved) => {
                    println!("   Retrieved ID: {}", retrieved.id);
                    println!("   Content: {}", retrieved.content);
                }
                Err(e) => println!("   Error: {}", e),
            }

            // Example 4: Delete data
            println!("\n4️⃣  Deleting data...");
            match client.delete(&created.id.to_string()).await {
                Ok(_) => println!("   Deleted successfully"),
                Err(e) => println!("   Error: {}", e),
            }
        }
        Err(e) => println!("   Error creating: {}", e),
    }

    // Example 5: List data
    println!("\n5️⃣  Listing data...");
    match client.list(None).await {
        Ok(list) => {
            println!("   Total items: {}", list.total);
            println!("   Page: {}/{}", list.page + 1, (list.total + list.limit - 1) / list.limit);
        }
        Err(e) => println!("   Error: {}", e),
    }

    println!("\n✅ Example completed!");
    Ok(())
}
