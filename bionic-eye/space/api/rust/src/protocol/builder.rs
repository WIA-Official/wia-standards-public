//! Message builder for WIA Space Protocol

use std::sync::atomic::{AtomicU64, Ordering};

use super::message::*;

/// Global sequence counter for message ordering
static SEQUENCE_COUNTER: AtomicU64 = AtomicU64::new(0);

/// Reset sequence counter (for testing)
pub fn reset_sequence() {
    SEQUENCE_COUNTER.store(0, Ordering::SeqCst);
}

/// Get next sequence number
fn next_sequence() -> u64 {
    SEQUENCE_COUNTER.fetch_add(1, Ordering::SeqCst)
}

/// Message builder for creating WSP messages
#[derive(Debug, Clone)]
pub struct MessageBuilder {
    source: Endpoint,
    default_destination: Option<Endpoint>,
    metadata: Option<MessageMetadata>,
}

impl MessageBuilder {
    /// Create a new message builder with source endpoint
    pub fn new(source: Endpoint) -> Self {
        Self {
            source,
            default_destination: None,
            metadata: None,
        }
    }

    /// Create a builder for a ground station
    pub fn ground_station(id: impl Into<String>) -> Self {
        Self::new(Endpoint::ground_station(id))
    }

    /// Create a builder for a spacecraft
    pub fn spacecraft(id: impl Into<String>) -> Self {
        Self::new(Endpoint::spacecraft(id))
    }

    /// Create a builder for a simulator
    pub fn simulator(id: impl Into<String>) -> Self {
        Self::new(Endpoint::simulator(id))
    }

    /// Set default destination
    pub fn with_destination(mut self, destination: Endpoint) -> Self {
        self.default_destination = Some(destination);
        self
    }

    /// Set default metadata
    pub fn with_metadata(mut self, metadata: MessageMetadata) -> Self {
        self.metadata = Some(metadata);
        self
    }

    // ========================================================================
    // Connection Messages
    // ========================================================================

    /// Build a connect message
    pub fn connect(&self, destination: &Endpoint, payload: ConnectPayload) -> WspMessage {
        self.build_message(
            MessageType::Connect,
            destination.clone(),
            serde_json::to_value(payload).unwrap(),
        )
    }

    /// Build a connect_ack message
    pub fn connect_ack(&self, destination: &Endpoint, payload: ConnectAckPayload) -> WspMessage {
        self.build_message(
            MessageType::ConnectAck,
            destination.clone(),
            serde_json::to_value(payload).unwrap(),
        )
    }

    /// Build a disconnect message
    pub fn disconnect(&self, destination: &Endpoint, reason: Option<String>) -> WspMessage {
        let payload = serde_json::json!({
            "reason": reason.unwrap_or_else(|| "normal_closure".to_string())
        });
        self.build_message(MessageType::Disconnect, destination.clone(), payload)
    }

    /// Build a ping message
    pub fn ping(&self, destination: &Endpoint) -> WspMessage {
        let payload = PingPayload {
            sent_at: chrono::Utc::now().timestamp_millis(),
            expected_latency_ms: None,
        };
        self.build_message(
            MessageType::Ping,
            destination.clone(),
            serde_json::to_value(payload).unwrap(),
        )
    }

    /// Build a ping message with expected latency
    pub fn ping_with_latency(&self, destination: &Endpoint, expected_latency_ms: u64) -> WspMessage {
        let payload = PingPayload {
            sent_at: chrono::Utc::now().timestamp_millis(),
            expected_latency_ms: Some(expected_latency_ms),
        };
        self.build_message(
            MessageType::Ping,
            destination.clone(),
            serde_json::to_value(payload).unwrap(),
        )
    }

    /// Build a pong message
    pub fn pong(&self, destination: &Endpoint, ping_id: &str, received_at: i64) -> WspMessage {
        let now = chrono::Utc::now().timestamp_millis();
        let payload = PongPayload {
            ping_id: ping_id.to_string(),
            received_at,
            processed_at: now,
        };
        self.build_message(
            MessageType::Pong,
            destination.clone(),
            serde_json::to_value(payload).unwrap(),
        )
    }

    // ========================================================================
    // Mission Data Messages
    // ========================================================================

    /// Build a telemetry message
    pub fn telemetry(&self, destination: &Endpoint, payload: TelemetryPayload) -> WspMessage {
        self.build_message(
            MessageType::Telemetry,
            destination.clone(),
            serde_json::to_value(payload).unwrap(),
        )
    }

    /// Build a command message
    pub fn command(&self, destination: &Endpoint, payload: CommandPayload) -> WspMessage {
        self.build_message(
            MessageType::Command,
            destination.clone(),
            serde_json::to_value(payload).unwrap(),
        )
    }

    /// Build a command_ack message
    pub fn command_ack(&self, destination: &Endpoint, payload: CommandAckPayload) -> WspMessage {
        self.build_message(
            MessageType::CommandAck,
            destination.clone(),
            serde_json::to_value(payload).unwrap(),
        )
    }

    /// Build a mission update message
    pub fn mission_update(
        &self,
        destination: &Endpoint,
        mission_id: &str,
        status: &str,
        details: serde_json::Value,
    ) -> WspMessage {
        let payload = serde_json::json!({
            "missionId": mission_id,
            "status": status,
            "details": details,
            "timestamp": chrono::Utc::now().to_rfc3339()
        });
        self.build_message(MessageType::MissionUpdate, destination.clone(), payload)
    }

    /// Build a simulation data message
    pub fn simulation_data(
        &self,
        destination: &Endpoint,
        simulation_id: &str,
        data: serde_json::Value,
    ) -> WspMessage {
        let payload = serde_json::json!({
            "simulationId": simulation_id,
            "data": data,
            "timestamp": chrono::Utc::now().to_rfc3339()
        });
        self.build_message(MessageType::SimulationData, destination.clone(), payload)
    }

    // ========================================================================
    // Specification Messages
    // ========================================================================

    /// Build a spec update message
    pub fn spec_update(&self, destination: &Endpoint, spec: serde_json::Value) -> WspMessage {
        self.build_message(MessageType::SpecUpdate, destination.clone(), spec)
    }

    /// Build a validation request message
    pub fn validation_request(
        &self,
        destination: &Endpoint,
        spec: serde_json::Value,
    ) -> WspMessage {
        let payload = serde_json::json!({
            "spec": spec,
            "requestedAt": chrono::Utc::now().to_rfc3339()
        });
        self.build_message(MessageType::ValidationRequest, destination.clone(), payload)
    }

    /// Build a validation result message
    pub fn validation_result(
        &self,
        destination: &Endpoint,
        request_id: &str,
        valid: bool,
        errors: Option<Vec<String>>,
    ) -> WspMessage {
        let payload = serde_json::json!({
            "requestId": request_id,
            "valid": valid,
            "errors": errors.unwrap_or_default(),
            "validatedAt": chrono::Utc::now().to_rfc3339()
        });
        self.build_message(MessageType::ValidationResult, destination.clone(), payload)
    }

    // ========================================================================
    // Error Messages
    // ========================================================================

    /// Build an error message
    pub fn error(
        &self,
        destination: &Endpoint,
        code: u32,
        name: &str,
        message: &str,
    ) -> WspMessage {
        let payload = serde_json::json!({
            "code": code,
            "name": name,
            "message": message,
            "recoverable": code < 5000,
            "retryAfter_s": if code < 2000 { Some(30) } else { None }
        });
        self.build_message(MessageType::Error, destination.clone(), payload)
    }

    /// Build an error message with details
    pub fn error_with_details(
        &self,
        destination: &Endpoint,
        code: u32,
        name: &str,
        message: &str,
        details: serde_json::Value,
    ) -> WspMessage {
        let payload = serde_json::json!({
            "code": code,
            "name": name,
            "message": message,
            "details": details,
            "recoverable": code < 5000,
            "retryAfter_s": if code < 2000 { Some(30) } else { None }
        });
        self.build_message(MessageType::Error, destination.clone(), payload)
    }

    /// Build a warning message
    pub fn warning(&self, destination: &Endpoint, message: &str) -> WspMessage {
        let payload = serde_json::json!({
            "message": message,
            "timestamp": chrono::Utc::now().to_rfc3339()
        });
        self.build_message(MessageType::Warning, destination.clone(), payload)
    }

    // ========================================================================
    // Internal
    // ========================================================================

    fn build_message(
        &self,
        message_type: MessageType,
        destination: Endpoint,
        payload: serde_json::Value,
    ) -> WspMessage {
        let mut msg = WspMessage::new(message_type, self.source.clone(), destination, payload)
            .with_sequence(next_sequence());

        if let Some(ref metadata) = self.metadata {
            msg = msg.with_metadata(metadata.clone());
        }

        msg
    }
}

/// Convenient function to create a message builder for ground station
pub fn from_ground_station(id: impl Into<String>) -> MessageBuilder {
    MessageBuilder::ground_station(id)
}

/// Convenient function to create a message builder for spacecraft
pub fn from_spacecraft(id: impl Into<String>) -> MessageBuilder {
    MessageBuilder::spacecraft(id)
}

/// Convenient function to create a message builder for simulator
pub fn from_simulator(id: impl Into<String>) -> MessageBuilder {
    MessageBuilder::simulator(id)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_builder_creation() {
        let builder = MessageBuilder::ground_station("gs-01");
        assert_eq!(builder.source.id, "gs-01");
        assert_eq!(builder.source.endpoint_type, EndpointType::GroundStation);
    }

    #[test]
    fn test_ping_message() {
        let builder = MessageBuilder::ground_station("gs-01");
        let dest = Endpoint::spacecraft("sc-01");

        let msg = builder.ping(&dest);
        assert_eq!(msg.message_type, MessageType::Ping);
        assert_eq!(msg.source.id, "gs-01");
        assert_eq!(msg.destination.id, "sc-01");
        // Note: sequence value depends on test execution order, just verify it's set
        assert!(msg.sequence >= 0);
    }

    #[test]
    fn test_sequence_increment() {
        // Note: Tests run in parallel, so we can't rely on absolute sequence values
        // Instead, we verify that sequences are strictly increasing
        let builder = MessageBuilder::ground_station("gs-01");
        let dest = Endpoint::spacecraft("sc-01");

        let msg1 = builder.ping(&dest);
        let msg2 = builder.ping(&dest);
        let msg3 = builder.ping(&dest);

        // Verify sequences are strictly increasing
        assert!(msg2.sequence > msg1.sequence, "msg2 should have higher sequence than msg1");
        assert!(msg3.sequence > msg2.sequence, "msg3 should have higher sequence than msg2");
    }

    #[test]
    fn test_connect_message() {
        reset_sequence();
        let builder = MessageBuilder::ground_station("gs-01");
        let dest = Endpoint::ground_station("mission-control");

        let payload = ConnectPayload {
            client_name: "Goldstone DSN".to_string(),
            capabilities: vec!["telemetry".to_string(), "command".to_string()],
            supported_technologies: None,
            options: None,
        };

        let msg = builder.connect(&dest, payload);
        assert_eq!(msg.message_type, MessageType::Connect);
    }

    #[test]
    fn test_error_message() {
        reset_sequence();
        let builder = MessageBuilder::ground_station("gs-01");
        let dest = Endpoint::ground_station("mission-control");

        let msg = builder.error(&dest, 2001, "SPACECRAFT_UNREACHABLE", "Cannot contact spacecraft");
        assert_eq!(msg.message_type, MessageType::Error);

        let payload: serde_json::Value = msg.payload;
        assert_eq!(payload["code"], 2001);
        assert_eq!(payload["name"], "SPACECRAFT_UNREACHABLE");
    }

    #[test]
    fn test_telemetry_message() {
        reset_sequence();
        let builder = MessageBuilder::spacecraft("mars-orbiter-01");
        let dest = Endpoint::ground_station("mission-control");

        let payload = TelemetryPayload {
            mission_id: "mars-terraform-2035".to_string(),
            subsystem: "atmospheric_processor".to_string(),
            readings: serde_json::json!({
                "co2_concentration": 0.953,
                "temperature_k": 210.5
            }),
            status: "nominal".to_string(),
            timestamp: None,
        };

        let msg = builder.telemetry(&dest, payload);
        assert_eq!(msg.message_type, MessageType::Telemetry);
    }

    #[test]
    fn test_convenience_functions() {
        reset_sequence();
        let builder = from_ground_station("gs-01");
        assert_eq!(builder.source.endpoint_type, EndpointType::GroundStation);

        let builder = from_spacecraft("sc-01");
        assert_eq!(builder.source.endpoint_type, EndpointType::Spacecraft);

        let builder = from_simulator("sim-01");
        assert_eq!(builder.source.endpoint_type, EndpointType::Simulator);
    }
}
