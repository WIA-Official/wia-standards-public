//! WIA Material Protocol Usage Example
//!
//! This example demonstrates how to use the WIA Material Protocol
//! for data exchange and real-time streaming.

use wia_material::prelude::*;
use wia_material::protocol::{CreatePayload, SubscribePayload};
use wia_material::MaterialType;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== WIA Material Protocol Example ===\n");

    // =========================================================================
    // Part 1: Protocol Messages
    // =========================================================================
    println!("1. Creating Protocol Messages\n");

    // Create a query message
    let query_payload = QueryPayload {
        material_type: Some(MaterialType::Superconductor),
        filter: Some(
            FilterBuilder::new()
                .gt("properties.superconductor.critical_temperature_k", 77.0)
                .contains("identity.name", "YBCO")
                .build_and()
                .unwrap(),
        ),
        sort: Some(SortConfig {
            field: "timestamp".to_string(),
            order: SortOrder::Desc,
        }),
        pagination: Some(Pagination {
            offset: 0,
            limit: 50,
        }),
        fields: Some(vec![
            "material_id".to_string(),
            "identity".to_string(),
            "properties.superconductor".to_string(),
        ]),
    };

    let query_message = MessageBuilder::query(query_payload);
    println!("Query Message:");
    println!("{}\n", query_message.to_json_pretty()?);

    // Create a ping message
    let ping = MessageBuilder::ping();
    println!("Ping Message:");
    println!("{}\n", ping.to_json_pretty()?);

    // =========================================================================
    // Part 2: HTTP Transport
    // =========================================================================
    println!("2. Using HTTP Transport\n");

    let config = TransportConfig::new("https://api.example.com")
        .with_api_key("wia_live_example_key")
        .with_max_retries(3);

    println!("Transport Config:");
    println!("  Base URL: {}", config.base_url);
    println!("  Materials URL: {}", config.materials_url());
    println!("  API Key: {:?}\n", config.api_key);

    let mut http_transport = HttpTransport::new(config);
    http_transport.connect().await?;
    println!("HTTP Transport connected: {}\n", http_transport.is_connected());

    // Create a material
    let material = MaterialBuilder::new()
        .material_type(MaterialType::Superconductor)
        .name("YBCO Sample")
        .formula("YBa2Cu3O7-x")
        .confidence(0.95)
        .build()?;

    let create_response = http_transport
        .create(CreatePayload {
            material: material.clone(),
        })
        .await?;

    println!("Created Material:");
    println!("  ID: {}", create_response.data.material_id);
    println!("  Name: {}", create_response.data.identity.name);
    println!("  Formula: {}\n", create_response.data.identity.formula);

    // Query materials
    let query_response = http_transport
        .query(QueryPayload::default())
        .await?;

    println!("Query Results:");
    println!("  Total: {}", query_response.meta.total_count);
    println!("  Returned: {}\n", query_response.meta.returned_count);

    http_transport.disconnect().await?;

    // =========================================================================
    // Part 3: WebSocket Transport
    // =========================================================================
    println!("3. Using WebSocket Transport\n");

    let ws_config = WebSocketConfig::new("wss://api.example.com")
        .with_api_key("wia_live_example_key");

    let mut ws_transport = WebSocketTransport::new(ws_config);
    ws_transport.connect().await?;

    println!("WebSocket State: {:?}", ws_transport.state().await);
    println!("Session ID: {:?}\n", ws_transport.session_id().await);

    // Subscribe to measurements
    let sub_ack = ws_transport
        .subscribe(SubscribePayload {
            channel: SubscriptionChannel::Measurements,
            material_type: Some(MaterialType::Superconductor),
            filter: None,
        })
        .await?;

    println!("Subscribed to: {:?}", sub_ack.channel);
    println!("Subscription ID: {}\n", sub_ack.subscription_id);

    // Simulate receiving data
    let sample_material = MaterialBuilder::new()
        .material_type(MaterialType::TopologicalInsulator)
        .name("Bi2Se3 Sample")
        .formula("Bi2Se3")
        .build()?;

    ws_transport
        .simulate_data(sample_material, SubscriptionChannel::Measurements)
        .await;

    let received = ws_transport.receive().await?;
    println!("Received Data Message:");
    println!("{}\n", received);

    // Cleanup
    ws_transport.disconnect().await?;

    // =========================================================================
    // Part 4: Filter Examples
    // =========================================================================
    println!("4. Filter Examples\n");

    // Simple filter
    let simple_filter = Filter::condition(FilterCondition::gt("tc", 77.0));
    println!(
        "Simple Filter: {}\n",
        serde_json::to_string_pretty(&simple_filter)?
    );

    // Complex AND filter
    let and_filter = FilterBuilder::new()
        .eq("material_type", "superconductor")
        .gt("properties.superconductor.critical_temperature_k", 77.0)
        .lte("properties.superconductor.critical_temperature_k", 273.15)
        .exists("properties.superconductor.critical_magnetic_field_t")
        .build_and()
        .unwrap();

    println!(
        "Complex AND Filter: {}\n",
        serde_json::to_string_pretty(&and_filter)?
    );

    // OR filter
    let or_filter = Filter::or(vec![
        Filter::condition(FilterCondition::eq("material_type", "superconductor")),
        Filter::condition(FilterCondition::eq("material_type", "topological_insulator")),
    ]);

    println!(
        "OR Filter: {}\n",
        serde_json::to_string_pretty(&or_filter)?
    );

    // =========================================================================
    // Part 5: Error Handling
    // =========================================================================
    println!("5. Error Handling\n");

    let error_msg = MessageBuilder::error(
        wia_material::protocol::ErrorCode::MaterialNotFound,
        "Material with ID 'wia-mat-999' not found",
    );
    println!("Error Message:");
    println!("{}\n", error_msg.to_json_pretty()?);

    println!("=== Example Complete ===");
    Ok(())
}
