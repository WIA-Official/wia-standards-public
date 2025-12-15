//! WIA Health Protocol Example
//!
//! This example demonstrates the WIA Health communication protocol
//! including message creation, handling, and WebSocket transport.

use chrono::Utc;
use uuid::Uuid;
use wia_health::protocol::{
    AlertAction, AlertPayload, AlertSeverity, AuthPayload, BiomarkerData, BiomarkerPayload,
    ConnectOptions, ConnectPayload, DataSource, ErrorPayload, Message, MessageBuilder,
    ProfileChange, ProfileUpdatePayload, SimulationPrediction, SimulationRecommendation,
    SimulationResultPayload, SimulationResults, StreamFilter, StreamOptions, StreamSubscription,
    UpdateOperation,
};
use wia_health::transport::TransportConfig;

fn main() {
    println!("=== WIA Health Protocol Example ===\n");

    // 1. Create protocol messages
    println!("1. Creating protocol messages...");

    // Connect message
    let connect_payload = ConnectPayload {
        client_id: "health-app-001".to_string(),
        client_name: Some("WIA Health Mobile App".to_string()),
        client_version: Some("2.0.0".to_string()),
        capabilities: Some(vec![
            "biomarkers".to_string(),
            "digital-twin".to_string(),
            "alerts".to_string(),
        ]),
        subscriptions: Some(vec!["biomarkers".to_string(), "alerts".to_string()]),
        options: Some(ConnectOptions {
            stream_rate: Some(60),
            compression: Some(false),
            format: Some("json".to_string()),
        }),
        auth: Some(AuthPayload {
            auth_type: "bearer".to_string(),
            token: "eyJhbGciOiJSUzI1NiIs...".to_string(),
            refresh_token: None,
        }),
    };

    let connect_msg = MessageBuilder::connect(connect_payload);
    println!("   Connect message ID: {}", connect_msg.message_id);
    println!("   Protocol: {} v{}", connect_msg.protocol, connect_msg.version);

    // 2. Subscribe to data streams
    println!("\n2. Creating subscription messages...");

    let streams = vec![
        StreamSubscription {
            stream_type: "biomarkers".to_string(),
            filter: Some(StreamFilter {
                markers: Some(vec![
                    "heart_rate".to_string(),
                    "blood_pressure".to_string(),
                    "glucose".to_string(),
                ]),
                sources: Some(vec!["apple_watch".to_string(), "dexcom_g7".to_string()]),
                min_quality: Some(0.8),
                clock_types: None,
            }),
            options: Some(StreamOptions {
                rate: Some(30),
                aggregation: Some("average".to_string()),
            }),
        },
        StreamSubscription {
            stream_type: "aging_clocks".to_string(),
            filter: Some(StreamFilter {
                markers: None,
                sources: None,
                min_quality: None,
                clock_types: Some(vec![
                    "horvath".to_string(),
                    "grimage".to_string(),
                    "phenoage".to_string(),
                ]),
            }),
            options: None,
        },
    ];

    let subscribe_msg = MessageBuilder::subscribe(streams);
    println!("   Subscribe message ID: {}", subscribe_msg.message_id);

    // 3. Biomarker data message
    println!("\n3. Creating biomarker data message...");

    let biomarker_payload = BiomarkerPayload {
        stream_id: "stream-001".to_string(),
        subject_id: "patient-123".to_string(),
        sequence: 42,
        data: BiomarkerData {
            marker: "heart_rate".to_string(),
            value: 72.0,
            unit: "bpm".to_string(),
            timestamp: Utc::now().timestamp_millis(),
            source: Some(DataSource {
                device: "apple_watch".to_string(),
                model: Some("Series 9".to_string()),
                firmware: Some("10.2".to_string()),
            }),
            quality: Some(0.95),
            metadata: Some(serde_json::json!({
                "activity_state": "resting",
                "position": "sitting"
            })),
        },
    };

    let biomarker_msg = MessageBuilder::biomarker(biomarker_payload);
    println!("   Biomarker message created");
    println!("   Message JSON preview:");
    if let Ok(json) = biomarker_msg.to_json() {
        // Print first 200 chars
        let preview: String = json.chars().take(200).collect();
        println!("   {}...", preview);
    }

    // 4. Profile update message
    println!("\n4. Creating profile update message...");

    let profile_update = ProfileUpdatePayload {
        subject_id: "patient-123".to_string(),
        update_type: "biomarkers".to_string(),
        changes: ProfileChange {
            path: "/biomarkers/inflammatoryMarkers/crp".to_string(),
            operation: UpdateOperation::Update,
            data: serde_json::json!({
                "value": 2.1,
                "unit": "mg/L",
                "timestamp": Utc::now().timestamp_millis()
            }),
        },
        version: 5,
    };

    let update_msg = MessageBuilder::profile_update(profile_update);
    println!("   Profile update message ID: {}", update_msg.message_id);

    // 5. Simulation result message
    println!("\n5. Creating simulation result message...");

    let simulation_result = SimulationResultPayload {
        simulation_id: Uuid::new_v4().to_string(),
        subject_id: "patient-123".to_string(),
        simulation_type: "aging_prediction".to_string(),
        status: "completed".to_string(),
        results: SimulationResults {
            predictions: vec![
                SimulationPrediction {
                    outcome: "biological_age".to_string(),
                    value: 47.5,
                    confidence: 0.89,
                    timeframe: "5_years".to_string(),
                },
                SimulationPrediction {
                    outcome: "mortality_risk".to_string(),
                    value: 0.12,
                    confidence: 0.75,
                    timeframe: "10_years".to_string(),
                },
            ],
            recommendations: Some(vec![
                SimulationRecommendation {
                    intervention: "Mediterranean diet".to_string(),
                    expected_benefit: 0.15,
                    confidence: 0.82,
                },
                SimulationRecommendation {
                    intervention: "Regular exercise".to_string(),
                    expected_benefit: 0.20,
                    confidence: 0.88,
                },
            ]),
        },
        duration: 1250,
    };

    let sim_msg = MessageBuilder::simulation_result(simulation_result);
    println!("   Simulation result message created");

    // 6. Alert message
    println!("\n6. Creating alert message...");

    let alert_payload = AlertPayload {
        alert_id: Uuid::new_v4().to_string(),
        subject_id: "patient-123".to_string(),
        severity: AlertSeverity::Warning,
        category: "biomarker_threshold".to_string(),
        title: "Elevated Glucose Level".to_string(),
        message: "Fasting glucose reading of 126 mg/dL exceeds recommended threshold of 100 mg/dL"
            .to_string(),
        data: Some(serde_json::json!({
            "marker": "glucose",
            "value": 126,
            "threshold": 100,
            "unit": "mg/dL"
        })),
        actions: Some(vec![
            AlertAction {
                action_type: "acknowledge".to_string(),
                label: "Acknowledge".to_string(),
                url: None,
            },
            AlertAction {
                action_type: "view_details".to_string(),
                label: "View Details".to_string(),
                url: Some("/health/biomarkers/glucose".to_string()),
            },
        ]),
    };

    let alert_msg = MessageBuilder::alert(alert_payload);
    println!("   Alert severity: {:?}", AlertSeverity::Warning);
    println!("   Alert message ID: {}", alert_msg.message_id);

    // 7. Ping/Pong for keepalive
    println!("\n7. Creating keepalive messages...");

    let ping_msg = MessageBuilder::ping();
    println!("   Ping message ID: {}", ping_msg.message_id);

    let pong_msg = MessageBuilder::pong(ping_msg.message_id, Utc::now().timestamp_millis() - 50);
    println!("   Pong correlation ID: {:?}", pong_msg.correlation_id);

    // 8. Error message
    println!("\n8. Creating error message...");

    let error_msg = MessageBuilder::error_simple(4003, "RATE_LIMITED", "Too many requests", true);

    let error_payload: ErrorPayload = serde_json::from_value(error_msg.payload.clone()).unwrap();
    println!("   Error code: {}", error_payload.code);
    println!("   Error name: {}", error_payload.name);
    println!("   Recoverable: {}", error_payload.recoverable);

    // 9. Message serialization
    println!("\n9. Testing message serialization...");

    let test_msg = MessageBuilder::disconnect("Client requested disconnect", 1000);
    let json = test_msg.to_json().expect("Serialization failed");
    println!("   Serialized disconnect message:");
    println!("   {}", json);

    let parsed: Message = Message::from_json(&json).expect("Deserialization failed");
    println!("   Deserialized message type: {:?}", parsed.message_type);

    // 10. Transport configuration
    println!("\n10. Transport configuration...");

    let config = TransportConfig::with_url("wss://api.wia-health.org/ws")
        .no_reconnect()
        .timeout(5000)
        .reconnect_delay(2000);

    println!("   URL: {}", config.url);
    println!("   Auto-reconnect: {}", config.auto_reconnect);
    println!("   Timeout: {}ms", config.connect_timeout_ms);

    // Summary
    println!("\n=== Protocol Example Complete ===");
    println!("\nMessage Types Demonstrated:");
    println!("   - Connect / ConnectAck");
    println!("   - Subscribe / SubscribeAck");
    println!("   - Biomarker data streaming");
    println!("   - Profile updates");
    println!("   - Simulation results");
    println!("   - Alerts");
    println!("   - Ping / Pong keepalive");
    println!("   - Error handling");

    println!("\n弘益人間 - Benefit All Humanity 🤟");
}
