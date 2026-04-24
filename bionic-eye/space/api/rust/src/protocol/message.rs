//! WIA Space Protocol message types

use serde::{Deserialize, Serialize};
use uuid::Uuid;
use chrono::Utc;

use super::{PROTOCOL_ID, PROTOCOL_VERSION};

/// Message type enumeration
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MessageType {
    // Connection management
    Connect,
    ConnectAck,
    Disconnect,
    Ping,
    Pong,

    // Mission data
    Telemetry,
    Command,
    CommandAck,
    MissionUpdate,
    SimulationData,

    // Specification
    SpecUpdate,
    ValidationRequest,
    ValidationResult,

    // Errors
    Error,
    Warning,
}

impl MessageType {
    /// Check if this is a connection management message
    pub fn is_connection(&self) -> bool {
        matches!(
            self,
            MessageType::Connect
                | MessageType::ConnectAck
                | MessageType::Disconnect
                | MessageType::Ping
                | MessageType::Pong
        )
    }

    /// Check if this is a mission data message
    pub fn is_mission_data(&self) -> bool {
        matches!(
            self,
            MessageType::Telemetry
                | MessageType::Command
                | MessageType::CommandAck
                | MessageType::MissionUpdate
                | MessageType::SimulationData
        )
    }

    /// Check if this is an error message
    pub fn is_error(&self) -> bool {
        matches!(self, MessageType::Error | MessageType::Warning)
    }
}

/// Endpoint type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EndpointType {
    GroundStation,
    Spacecraft,
    Simulator,
    Relay,
}

/// Message endpoint (source or destination)
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Endpoint {
    pub id: String,
    #[serde(rename = "type")]
    pub endpoint_type: EndpointType,
}

impl Endpoint {
    pub fn new(id: impl Into<String>, endpoint_type: EndpointType) -> Self {
        Self {
            id: id.into(),
            endpoint_type,
        }
    }

    pub fn ground_station(id: impl Into<String>) -> Self {
        Self::new(id, EndpointType::GroundStation)
    }

    pub fn spacecraft(id: impl Into<String>) -> Self {
        Self::new(id, EndpointType::Spacecraft)
    }

    pub fn simulator(id: impl Into<String>) -> Self {
        Self::new(id, EndpointType::Simulator)
    }

    pub fn relay(id: impl Into<String>) -> Self {
        Self::new(id, EndpointType::Relay)
    }
}

/// Message priority
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
#[serde(rename_all = "lowercase")]
pub enum Priority {
    Critical,
    High,
    #[default]
    Normal,
    Low,
}

/// Compression type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
#[serde(rename_all = "lowercase")]
pub enum Compression {
    #[default]
    None,
    Gzip,
    Lz4,
}

/// Encryption type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
#[serde(rename_all = "kebab-case")]
pub enum Encryption {
    #[default]
    None,
    Aes256Gcm,
}

/// Message metadata
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct MessageMetadata {
    #[serde(default)]
    pub priority: Priority,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub ttl: Option<u64>,

    #[serde(default)]
    pub compression: Compression,

    #[serde(default)]
    pub encryption: Encryption,
}

impl MessageMetadata {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn critical() -> Self {
        Self {
            priority: Priority::Critical,
            ..Default::default()
        }
    }

    pub fn with_priority(mut self, priority: Priority) -> Self {
        self.priority = priority;
        self
    }

    pub fn with_ttl(mut self, ttl_seconds: u64) -> Self {
        self.ttl = Some(ttl_seconds);
        self
    }

    pub fn with_compression(mut self, compression: Compression) -> Self {
        self.compression = compression;
        self
    }

    pub fn with_encryption(mut self, encryption: Encryption) -> Self {
        self.encryption = encryption;
        self
    }
}

/// WIA Space Protocol message
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct WspMessage {
    pub protocol: String,
    pub version: String,
    pub message_id: String,
    pub timestamp: i64,
    pub sequence: u64,
    #[serde(rename = "type")]
    pub message_type: MessageType,
    pub source: Endpoint,
    pub destination: Endpoint,
    pub payload: serde_json::Value,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub metadata: Option<MessageMetadata>,
}

impl WspMessage {
    /// Create a new WSP message
    pub fn new(
        message_type: MessageType,
        source: Endpoint,
        destination: Endpoint,
        payload: serde_json::Value,
    ) -> Self {
        Self {
            protocol: PROTOCOL_ID.to_string(),
            version: PROTOCOL_VERSION.to_string(),
            message_id: Uuid::new_v4().to_string(),
            timestamp: Utc::now().timestamp_millis(),
            sequence: 0,
            message_type,
            source,
            destination,
            payload,
            metadata: None,
        }
    }

    /// Set sequence number
    pub fn with_sequence(mut self, sequence: u64) -> Self {
        self.sequence = sequence;
        self
    }

    /// Set metadata
    pub fn with_metadata(mut self, metadata: MessageMetadata) -> Self {
        self.metadata = Some(metadata);
        self
    }

    /// Serialize to JSON string
    pub fn to_json(&self) -> Result<String, serde_json::Error> {
        serde_json::to_string(self)
    }

    /// Serialize to pretty JSON string
    pub fn to_json_pretty(&self) -> Result<String, serde_json::Error> {
        serde_json::to_string_pretty(self)
    }

    /// Deserialize from JSON string
    pub fn from_json(json: &str) -> Result<Self, serde_json::Error> {
        serde_json::from_str(json)
    }

    /// Get message age in milliseconds
    pub fn age_ms(&self) -> i64 {
        Utc::now().timestamp_millis() - self.timestamp
    }

    /// Check if message is expired (based on TTL)
    pub fn is_expired(&self) -> bool {
        if let Some(ref metadata) = self.metadata {
            if let Some(ttl) = metadata.ttl {
                return self.age_ms() > (ttl * 1000) as i64;
            }
        }
        false
    }
}

// ============================================================================
// Payload Types
// ============================================================================

/// Connect payload
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ConnectPayload {
    pub client_name: String,
    pub capabilities: Vec<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub supported_technologies: Option<Vec<String>>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub options: Option<ConnectOptions>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ConnectOptions {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub data_rate: Option<u64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub compression: Option<bool>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub encryption: Option<bool>,
}

/// Connect acknowledgment payload
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ConnectAckPayload {
    pub session_id: String,
    pub accepted: bool,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub server_name: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub heartbeat_interval_s: Option<u64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub reason: Option<String>,
}

/// Ping payload
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct PingPayload {
    pub sent_at: i64,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub expected_latency_ms: Option<u64>,
}

/// Pong payload
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct PongPayload {
    pub ping_id: String,
    pub received_at: i64,
    pub processed_at: i64,
}

/// Telemetry payload
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct TelemetryPayload {
    pub mission_id: String,
    pub subsystem: String,
    pub readings: serde_json::Value,
    pub status: String,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub timestamp: Option<TelemetryTimestamp>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct TelemetryTimestamp {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mission_elapsed_time_s: Option<f64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub utc: Option<String>,
}

/// Command payload
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct CommandPayload {
    pub command_id: String,
    pub command_type: String,
    pub target_subsystem: String,
    pub parameters: serde_json::Value,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub execution_time: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub requires_ack: Option<bool>,
}

/// Command acknowledgment payload
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct CommandAckPayload {
    pub command_id: String,
    pub status: CommandStatus,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub executed_at: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub result: Option<serde_json::Value>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CommandStatus {
    Received,
    Queued,
    Executing,
    Completed,
    Failed,
    Rejected,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_message_type_classification() {
        assert!(MessageType::Connect.is_connection());
        assert!(MessageType::Telemetry.is_mission_data());
        assert!(MessageType::Error.is_error());
    }

    #[test]
    fn test_endpoint_creation() {
        let gs = Endpoint::ground_station("gs-01");
        assert_eq!(gs.id, "gs-01");
        assert_eq!(gs.endpoint_type, EndpointType::GroundStation);

        let sc = Endpoint::spacecraft("sc-01");
        assert_eq!(sc.endpoint_type, EndpointType::Spacecraft);
    }

    #[test]
    fn test_message_serialization() {
        let msg = WspMessage::new(
            MessageType::Ping,
            Endpoint::ground_station("gs-01"),
            Endpoint::spacecraft("sc-01"),
            serde_json::json!({ "sentAt": 1234567890 }),
        );

        let json = msg.to_json().unwrap();
        assert!(json.contains("wia-space"));
        assert!(json.contains("ping"));

        let parsed = WspMessage::from_json(&json).unwrap();
        assert_eq!(parsed.message_type, MessageType::Ping);
    }

    #[test]
    fn test_metadata() {
        let metadata = MessageMetadata::new()
            .with_priority(Priority::Critical)
            .with_ttl(3600)
            .with_compression(Compression::Gzip);

        assert_eq!(metadata.priority, Priority::Critical);
        assert_eq!(metadata.ttl, Some(3600));
        assert_eq!(metadata.compression, Compression::Gzip);
    }

    #[test]
    fn test_connect_payload() {
        let payload = ConnectPayload {
            client_name: "Test Client".to_string(),
            capabilities: vec!["telemetry".to_string(), "command".to_string()],
            supported_technologies: Some(vec!["dyson_sphere".to_string()]),
            options: Some(ConnectOptions {
                data_rate: Some(1000000),
                compression: Some(true),
                encryption: Some(true),
            }),
        };

        let json = serde_json::to_string(&payload).unwrap();
        let parsed: ConnectPayload = serde_json::from_str(&json).unwrap();
        assert_eq!(parsed.client_name, "Test Client");
    }
}
