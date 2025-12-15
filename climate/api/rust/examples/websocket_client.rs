//! WebSocket Client Example
//!
//! This example demonstrates how to use the WIA Climate WebSocket client
//! to connect to a server and send climate data.
//!
//! Note: This example requires a running WIA Climate WebSocket server.
//! For testing purposes, use the mock transport instead.

use wia_climate::prelude::*;
use wia_climate::protocol::*;
use wia_climate::transport::*;

#[tokio::main]
async fn main() -> Result<()> {
    // Initialize logging
    tracing_subscriber::fmt::init();

    println!("=== WIA Climate - WebSocket Client Example ===\n");

    // For demonstration, we'll use a mock transport since we don't have a real server
    demonstrate_with_mock().await?;

    // Uncomment below to connect to a real server:
    // connect_to_real_server("wss://api.example.com/wia-climate/v1/ws").await?;

    println!("\n弘益人間 - Benefit All Humanity 🌍");
    Ok(())
}

async fn demonstrate_with_mock() -> Result<()> {
    println!("Using MockTransport for demonstration...\n");

    // Create a mock transport
    let mut transport = MockTransport::new();

    // Queue a connect_ack response
    let connect_ack = ConnectAckPayload::success("session-12345".to_string());
    let ack_msg = ProtocolMessage::connect_ack(connect_ack)
        .map_err(|e| ClimateError::SerializationError(e.to_string()))?;
    transport.queue_receive(ack_msg);

    // Connect the transport
    transport.connect("mock://test-server").await?;
    println!("✓ Transport connected");

    // Send a connect message
    let connect_msg = ConnectBuilder::new()
        .client_id("carbon-sensor-001")
        .client_type(ClientType::Sensor)
        .capability("carbon_capture")
        .capability("vertical_farming")
        .api_key("test-api-key")
        .build()
        .map_err(|e| ClimateError::Validation(e.to_string()))?;

    transport.send(&connect_msg).await?;
    println!("✓ Connect message sent");

    // Receive the connect_ack
    let response = transport.receive().await?;
    println!("✓ Received response: {:?}", response.message_type);

    if response.message_type == MessageType::ConnectAck {
        if let Some(payload) = response.get_payload::<ConnectAckPayload>() {
            let ack = payload.map_err(|e| ClimateError::SerializationError(e.to_string()))?;
            if ack.success {
                println!("✓ Connected! Session ID: {:?}", ack.session_id);
            }
        }
    }

    // Create and send climate data
    println!("\n--- Sending Climate Data ---\n");

    let climate_message = ClimateMessage::builder()
        .location(Location::new(64.0, -21.0).with_altitude(100.0))
        .device(
            Device::new("Climeworks", "Orca DAC")
                .with_serial("ORCA-2024-001"),
        )
        .carbon_capture_data(CarbonCaptureData {
            technology: CarbonCaptureTechnology::Dac,
            capture_rate_kg_per_hour: 125.5,
            co2_purity_percentage: Some(99.2),
            energy_consumption_kwh: Some(2500.0),
            ..Default::default()
        })
        .build()?;

    let data_msg = ProtocolMessage::data(&climate_message)
        .map_err(|e| ClimateError::SerializationError(e.to_string()))?;

    transport.send(&data_msg).await?;
    println!("✓ Climate data sent");

    // Show the sent message
    let sent = transport.sent_messages();
    println!("\n--- Messages Sent ({}) ---", sent.len());
    for (i, msg) in sent.iter().enumerate() {
        println!("  {}. Type: {}, ID: {}", i + 1, msg.message_type, &msg.message_id[..8]);
    }

    // Send a command
    println!("\n--- Sending Command ---\n");

    let command_msg = CommandBuilder::new()
        .target("device-orca-001")
        .action("set_capture_rate")
        .param("rate_kg_per_hour", 150.0)
        .timeout(5000)
        .build()
        .map_err(|e| ClimateError::Validation(e.to_string()))?;

    transport.send(&command_msg).await?;
    println!("✓ Command sent: set_capture_rate");

    // Subscribe to data
    println!("\n--- Subscribing to Data ---\n");

    let subscribe_msg = SubscribeBuilder::new()
        .topic("carbon_capture/*")
        .topic("vertical_farming/sensors/*")
        .qos(QoS::AtLeastOnce)
        .build()
        .map_err(|e| ClimateError::Validation(e.to_string()))?;

    transport.send(&subscribe_msg).await?;
    println!("✓ Subscribed to topics");

    // Disconnect
    let disconnect_msg = ProtocolMessage::disconnect(Some(DisconnectPayload::normal()))
        .map_err(|e| ClimateError::SerializationError(e.to_string()))?;

    transport.send(&disconnect_msg).await?;
    transport.disconnect().await?;
    println!("✓ Disconnected");

    // Show all sent messages
    let sent = transport.sent_messages();
    println!("\n=== Summary ===");
    println!("Total messages sent: {}", sent.len());

    Ok(())
}

#[allow(dead_code)]
async fn connect_to_real_server(url: &str) -> Result<()> {
    println!("Connecting to server: {}\n", url);

    // Create a WebSocket client
    let mut client = WebSocketClient::new("carbon-sensor-001");

    // Connect (this performs the protocol handshake)
    client.connect(url).await?;
    println!("✓ Connected! Session ID: {:?}", client.session_id());

    // Send climate data
    let message = ClimateMessage::builder()
        .location(Location::new(64.0, -21.0))
        .device(Device::new("Climeworks", "Orca DAC"))
        .carbon_capture_data(CarbonCaptureData {
            technology: CarbonCaptureTechnology::Dac,
            capture_rate_kg_per_hour: 125.5,
            ..Default::default()
        })
        .build()?;

    client.send_data(&message).await?;
    println!("✓ Climate data sent");

    // Disconnect
    client.disconnect().await?;
    println!("✓ Disconnected");

    Ok(())
}
