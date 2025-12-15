//! Protocol Client Example
//!
//! Demonstrates connecting to a quantum backend and submitting jobs
//! using the WIA Quantum Protocol.

use wia_quantum::prelude::*;
use wia_quantum::protocol::{JobConfig, JobPriority};

#[tokio::main]
async fn main() -> Result<()> {
    println!("=== WIA Quantum: Protocol Client Example ===\n");

    // =========================================
    // Part 1: Create Client Configuration
    // =========================================
    println!("--- Client Configuration ---\n");

    let config = ClientConfig {
        url: "wss://quantum.wia.live/wia-quantum".to_string(),
        api_key: Some("wia_quantum_demo_key".to_string()),
        client_id: "demo-client".to_string(),
        client_name: Some("WIA Quantum Demo".to_string()),
        ..Default::default()
    };

    println!("URL: {}", config.url);
    println!("Client ID: {}", config.client_id);

    // =========================================
    // Part 2: Create Client (Mock Transport)
    // =========================================
    println!("\n--- Creating Client ---\n");

    let client = Client::new_mock(config);

    // Connect to backend
    client.connect().await?;
    println!("Connected: {}", client.is_connected().await);

    // =========================================
    // Part 3: Create and Submit Circuit
    // =========================================
    println!("\n--- Creating Quantum Circuit ---\n");

    let mut circuit = QuantumCircuit::new(2, 2)
        .with_name("Bell State")
        .with_description("Create entangled Bell state");

    circuit.h(0)?;
    circuit.cx(0, 1)?;
    circuit.measure_all()?;

    println!("Circuit: {} qubits, {} gates",
        circuit.num_qubits(),
        circuit.gate_count()
    );
    println!("OpenQASM:\n{}", circuit.to_qasm());

    // =========================================
    // Part 4: Submit Job
    // =========================================
    println!("\n--- Submitting Job ---\n");

    let job_config = JobConfig {
        shots: 1024,
        seed: Some(42),
        optimization_level: 1,
        error_mitigation: false,
        ..Default::default()
    };

    let job = client.submit_job(
        &circuit,
        "simulator_statevector",
        Some(job_config),
    ).await?;

    println!("Job ID: {}", job.job_id);
    println!("Backend: {}", job.backend_id);
    println!("Submitted: {} ms", job.submitted_at);

    // =========================================
    // Part 5: Check Job Status
    // =========================================
    println!("\n--- Job Status ---\n");

    let status = client.get_job_status(&job.job_id).await?;
    println!("Status: {:?}", status);

    // =========================================
    // Part 6: Subscribe to Updates
    // =========================================
    println!("\n--- Subscribing to Updates ---\n");

    client.subscribe(vec![
        "job_updates".to_string(),
        "backend_status".to_string(),
    ]).await?;

    println!("Subscribed to: job_updates, backend_status");

    // =========================================
    // Part 7: Ping/Pong
    // =========================================
    println!("\n--- Connection Check ---\n");

    client.ping().await?;
    println!("Ping sent successfully");

    // =========================================
    // Part 8: Disconnect
    // =========================================
    println!("\n--- Disconnecting ---\n");

    client.disconnect().await?;
    println!("Disconnected: {}", !client.is_connected().await);

    // =========================================
    // Part 9: Message Examples
    // =========================================
    println!("\n--- Protocol Message Examples ---\n");

    // Create a connect message
    use wia_quantum::protocol::ConnectPayload;

    let connect_msg = Message::new(MessageType::Connect, ConnectPayload {
        client_id: "example-client".to_string(),
        client_name: Some("Example App".to_string()),
        client_version: Some("1.0.0".to_string()),
        capabilities: Some(vec!["circuits".to_string(), "jobs".to_string()]),
        auth: None,
        options: None,
    });

    println!("Connect Message:");
    println!("  Type: {:?}", connect_msg.message_type);
    println!("  Message ID: {}", connect_msg.message_id);
    println!("  Protocol: {} v{}", connect_msg.protocol, connect_msg.version);

    // Serialize to JSON
    let json = connect_msg.to_json()?;
    println!("\nJSON (truncated):");
    for line in json.lines().take(10) {
        println!("  {}", line);
    }
    println!("  ...");

    println!("\n=== Example Complete ===");

    Ok(())
}
